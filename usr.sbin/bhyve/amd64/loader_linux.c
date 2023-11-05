/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Rob Norris <robn@despairlabs.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/linker_set.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stddef.h>
#include <unistd.h>

#include <machine/specialreg.h>
#include <machine/segments.h>
#include <vm/pmap.h>

#include <vmmapi.h>

#include "loader.h"
#include "debug.h"

#include "amd64/e820.h"

/*
 * This implements the Linux x86 64-bit boot protocol.
 *
 * The boot protocol is complicated and messy, mostly because x86 is
 * complicated and messy. Unfortunately the documentation is missing or
 * ambiguous on some key points, most implementations available have minimal
 * documentation or commentary, and there's lots of variations in the process
 * depending on age, supported kernel variations, targeting bare-metal or
 * hypervisor (or both!), and if they're booting from EFI or real mode inside
 * the guest or directly from the outside.
 *
 * This is the smallest I could come up with. The main limitation is that it
 * only boots 64-bit kernels, supporting the 64-bit boot protocol, in bzImage
 * format, loaded to high memory. In practice this basically all vendor kernels
 * since Linux 3.8.
 */

/*
 * Standard Linux setup header, describing the image within.
 */
struct setup_header {
	uint8_t		setup_sects;
	uint16_t	root_flags;
	uint32_t	syssize;
	uint16_t	ram_size;
	uint16_t	vid_mode;
	uint16_t	root_dev;
	uint16_t	boot_flag;
	uint16_t	jump;
	uint32_t	header;
	uint16_t	version;
	uint32_t	realmode_swtch;
	uint16_t	start_sys_seg;
	uint16_t	kernel_version;
	uint8_t		type_of_loader;
	uint8_t		loadflags;
	uint16_t	setup_move_size;
	uint32_t	code32_start;
	uint32_t	ramdisk_image;
	uint32_t	ramdisk_size;
	uint32_t	bootsect_kludge;
	uint16_t	heap_end_ptr;
	uint8_t		ext_loader_ver;
	uint8_t		ext_loader_type;
	uint32_t	cmd_line_ptr;
	uint32_t	initrd_addr_max;
	uint32_t	kernel_alignment;
	uint8_t		relocatable_kernel;
	uint8_t		min_alignment;
	uint16_t	xloadflags;
	uint32_t	cmdline_size;
	uint32_t	hardware_subarch;
	uint64_t	hardware_subarch_data;
	uint32_t	payload_offset;
	uint32_t	payload_length;
	uint64_t	setup_data;
	uint64_t	pref_address;
	uint32_t	init_size;
	uint32_t	handover_offset;
	uint32_t	kernel_info_offset;
} __packed;

/*
 * The boot structure, where the bootloader communicates to the kernel
 * interesting things about the host system. Its a complex and deeply nested
 * structure and most of it we don't use, so this just defines the pieces we
 * do, and pads the rest.
 */
#define	BP_E820_MAX_ENTRIES	(0x80)
struct boot_params {
	uint8_t			_pad1[0x1e8];
	uint8_t			e820_entries;		/* 0x1e8 */
	uint8_t			_pad2[0x8];
	struct setup_header	hdr;			/* 0x1f1 */
	uint8_t			_pad3[0x2d0-0x1f1-sizeof(struct setup_header)];
	struct e820_entry	e820_table[BP_E820_MAX_ENTRIES]; /* 0x2d0 */
	uint8_t			_pad4[0x330];
} __packed;

/* boot_params sanity checks; its far to easy to screw up */
static_assert(offsetof(struct boot_params, e820_entries) == 0x1e8,
    "boot_params.e820_entries should be at 0x1e8");
static_assert(offsetof(struct boot_params, hdr) == 0x1f1,
    "boot_params.hdr should be at 0x1f1");
static_assert(offsetof(struct boot_params, e820_table) == 0x2d0,
    "boot_params.e820_table should be at 0x2d0");
static_assert(sizeof(struct boot_params) == 0x1000,
    "boot_params should have size 0x1000");

/* Magic number to identify lernel images */
#define	LINUX_MAGIC		(0x53726448)

/* Load kernel in highmem (1M) */
#define	KERNEL_START		(0x00100000lu)

/* Places to load kernel config/params to */
#define	CMDLINE_START		(0x00020000llu)
#define	BOOT_PARAMS_START	(0x00007000llu)

/* Initial stack pointer */
#define	BOOT_STACK_POINTER	(0x8ff0llu)

/* Initial GDT/IDT location and size */
#define	BOOT_GDT_OFFSET		(0x0500llu)
#define	BOOT_GDT_NENTRIES	(4)
#define	BOOT_IDT_OFFSET		(0x0520llu)
#define	BOOT_IDT_NENTRIES	(1)

/* Initial page table locations */
#define	BOOT_PML4_START		(0x9000llu)
#define	BOOT_PDPTE_START	(0xa000llu)
#define	BOOT_PDE_START		(0xb000llu)

/*
 * Default kernel command line.
 * - reboot=t: halt/reboot via triple-fault
 * - panic=1: on panic, wait 1s, then reboot
 * - console=ttyS0: kernel console to first serial port (COM1)
 * - earlyprintk=ttyS0: send early (pre-console) output to COM1 too
 */
static const char *default_cmdline =
    "reboot=t panic=1 console=ttyS0 earlyprintk=ttyS0";

/* helper; mmap a file for read */
static int
_mmap_read_file(const char *filename, char **addr, size_t *size)
{
	int fd = open(filename, O_RDONLY);
	if (fd < 0)
		return (errno);

	struct stat st;
	if (fstat(fd, &st) < 0) {
		int err = errno;
		close(fd);
		return (err);
	}

	*size = roundup(st.st_size, PAGE_SIZE);

	*addr = mmap(NULL, *size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (addr == NULL) {
		int err = errno;
		close(fd);
		return (err);
	}

	close(fd);
	return (0);
}

/*
 * Memory setup entry point. Get the kernel, initrd and associated stuff into
 * guest memory and hook it all up.
 */
static int
loader_linux_setup_memory(struct vmctx *ctx)
{
	int err = 0;
	char *kbase; size_t ksize;

	const char *kernel = get_config_value("loader.kernel");
	const char *initrd = get_config_value("loader.initrd");
	const char *cmdline = get_config_value("loader.cmdline");

	if (!kernel) {
		EPRINTLN("loader_linux: no kernel supplied, "
		    "did you set 'loader.kernel'?");
		return (EINVAL);
	}

	err = _mmap_read_file(kernel, &kbase, &ksize);
	if (err != 0) {
		EPRINTLN("loader_linux: error loading kernel '%s': %s",
		    kernel, strerror(err));
		return (err);
	}

	/* The setup header is at 0x1f1, see boot.rst */
	struct setup_header *setup_hdr = (struct setup_header *)(kbase + 0x1f1);

	/* Magic number sanity check */
	if (setup_hdr->header != 0x53726448) {
		EPRINTLN("loader_linux: invalid linux image: bad magic");
		munmap(kbase, ksize);
		return (EINVAL);
	}

	/*
	 * We want the "new" commandline protocol introduced in 2.02.  Also,
	 * we're not supporting pre-bzImage kernels, which will all have
	 * LOAD_HIGH set (loadflags & 0x1).
	 */
	if ((setup_hdr->version < 0x202) || !(setup_hdr->loadflags & 0x1)) {
		EPRINTLN("loader_linux invalid kernel image: ancient");
		munmap(kbase, ksize);
		return (EINVAL);
	}

	/*
	 * The compressed kernel image is some number of 512-byte sectors into
	 * the kernel file proper. That number is setup_sects + one boot
	 * sector. If setup_sects is 0, then use 4.
	 */
	off_t koff = (((setup_hdr->setup_sects == 0) ?
	    4 : setup_hdr->setup_sects) + 1) * 512;

	/*
	 * "Load" the kernel, by mapping a region at the load address and
	 * copying it in.
	 */
	char *hkbase = vm_map_gpa(ctx,
	    KERNEL_START, roundup(ksize-koff, PAGE_SIZE));
	memcpy(hkbase, kbase+koff, ksize-koff);

	/* Copy the commandline in */
	char *hcmdline = vm_map_gpa(ctx, CMDLINE_START, PAGE_SIZE);
	strlcpy(hcmdline, cmdline ? cmdline : default_cmdline, PAGE_SIZE);

	/* Fill out boot params */
	struct boot_params *hbp = vm_map_gpa(ctx, BOOT_PARAMS_START,
	    sizeof (struct boot_params));
	bzero(hbp, sizeof (struct boot_params));

	/* Copy the original header in */
	memcpy(&hbp->hdr, setup_hdr, sizeof (struct setup_header));

	/* "undefined" loader, until we register one for bhyve */
	hbp->hdr.type_of_loader = 0xff;

	/* Commandline, including trailing zero */
	hbp->hdr.cmd_line_ptr = CMDLINE_START;
	hbp->hdr.cmdline_size = strlen(hcmdline)+1;

	if (initrd) {
		char *ibase; size_t isize;
		err = _mmap_read_file(initrd, &ibase, &isize);
		if (err != 0) {
			EPRINTLN("loader_linux: error loading initrd '%s': %s",
			    initrd, strerror(err));
			return (err);
		}

		/*
		 * initrd doesn't really matter where it gets loaded, but its a
		 * little more involved to place it beyond the max address
		 * provided in the setup header. By convention it goes at the
		 * top of memory (bounded by the provided max), so we do that
		 * too because it works.
		 */

		/*
		 * XXX this is a little naive, as it doesn't take into account
		 * any thing else that has been placed in memory. Really we
		 * need an allocator for guest memory setup (ie not the e820
		 * allocator, which publishes its allocations to the guest.
		 * This is fine for now though. -- robn, 2023-11-07
		 */
		uint64_t initrd_start = roundup(
		    MIN(setup_hdr->initrd_addr_max, vm_get_lowmem_size(ctx)-1) -
		    isize - PAGE_SIZE, PAGE_SIZE);

		char *hibase = vm_map_gpa(ctx, initrd_start, isize);
		memcpy(hibase, ibase, isize);

		/* Tell the kernel where to find it */
		hbp->hdr.ramdisk_image = initrd_start;
		hbp->hdr.ramdisk_size = isize;

		munmap(ibase, isize);
	}

	munmap(kbase, ksize);
	return (err);
}

/*
 * GDT setup. Linux doesn't want much; just code and data segments that cover
 * the first 4G. It doesn't actually require a TSS, but some loaders set it.
 * The Linux real-mode boot code suggests this that Intel VT-x would prefer
 * TSS/TR to be non-NULL, which I can't easily confirm, but there seems to be
 * no harm in setting it up.
 *
 * Here we set up a basic table and a couple of helpers to help with assembling
 * the GDT and setting the shadow registers. FreeBSD has roughly equivalent
 * stuff in sys/amd64/include/segments.h but the conversion functions aren't
 * available to userspace, so its easier to do this ourselves.
 */
struct gdt_def {
	uint32_t base;
	uint32_t limit;
	uint32_t flags;
};
static struct gdt_def gdt_def[] = {
	{ 0, 0, 0 },		/* NULL */
	{ 0, 0xfffff, 0xa09b },	/* CODE */
	{ 0, 0xfffff, 0xc093 },	/* DATA */
	{ 0, 0xfffff, 0x808b },	/* TSS */
};

/*
 * Convert our nice simple definitions into the complex 64-bit descriptor
 * format expected by the CPU.
 */
#define	_GDT_ENTRY(def)					\
	(((((def)->base)  & 0xff000000ull) << 32) |	\
	 ((((def)->flags) & 0x0000f0ffull) << 40) |	\
	 ((((def)->limit) & 0x000f0000ull) << 32) |	\
	 ((((def)->base)  & 0x00ffffffull) << 16) |	\
	 ((((def)->limit) & 0x0000ffffull)))

/* helper; set the segment register and its shadow register in one call */
static inline int
_set_reg_desc(struct vcpu *vcpu, int reg, int idx)
{
	int err;
	struct gdt_def *def = &gdt_def[idx];
	if (!(err = vm_set_desc(vcpu, reg, def->base, def->limit, def->flags)))
		err = vm_set_register(vcpu, reg, idx << 3);
	return (err);
}

static int
loader_linux_setup_boot_cpu(struct vmctx *ctx, struct vcpu *vcpu)
{
	int err;

	/*
	 * XXX Some implementations check/set CPUID and MSRs here. Its doesn't
	 *     appear to be necessary, and its sort of complicated to do in
	 *     bhyve (rdmsr/wrmsr is always a VM exit?) so I'm hoping to avoid
	 *     it for the moment.
	 */

	/* Install the GDT */
	uint64_t *gdt = vm_map_gpa(ctx, BOOT_GDT_OFFSET,
	    sizeof (uint64_t) * BOOT_GDT_NENTRIES);
	for (int i = 0; i < BOOT_GDT_NENTRIES; i++)
		gdt[i] = _GDT_ENTRY(&gdt_def[i]);
	if ((err = vm_set_desc(vcpu, VM_REG_GUEST_GDTR, BOOT_GDT_OFFSET,
	    sizeof (uint64_t) * BOOT_GDT_NENTRIES -1, 0)))
		goto fail;

	/* Zero IDT */
	uint64_t *idt = vm_map_gpa(ctx, BOOT_IDT_OFFSET,
	    sizeof (uint64_t) * BOOT_IDT_NENTRIES);
	idt[0] = 0;
	if ((err = vm_set_desc(vcpu, VM_REG_GUEST_IDTR, BOOT_IDT_OFFSET,
	    sizeof (uint64_t) * BOOT_IDT_NENTRIES -1, 0)))
		goto fail;

	/*
	 * CS to gdt[1]. We have to set the shadow register as well because
	 * this isn't a proper register load.
	 */
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_CS, 1)))
		goto fail;

	/*
	 * DS to gdt[2]. boot.rst documents that DS, ES and SS must be set this
	 * way, but most implementations set FS and GS as well.
	 */
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_DS, 2)))
		goto fail;
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_ES, 2)))
		goto fail;
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_FS, 2)))
		goto fail;
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_GS, 2)))
		goto fail;
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_SS, 2)))
		goto fail;

	/*
	 * TR to gdt[3].
	 */
	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_TR, 3)))
		goto fail;

	/*
	 * Page table:
	 * PML4[0]  = 0..512GB
	 * PDPTE[0] = 0..1GB
	 * PDE[x]   = x*2MB..+2MB
	 */
	uint64_t *pml4 = vm_map_gpa(ctx, BOOT_PML4_START, PAGE_SIZE);
	uint64_t *pdpte = vm_map_gpa(ctx, BOOT_PDPTE_START, PAGE_SIZE);
	uint64_t *pde = vm_map_gpa(ctx, BOOT_PDE_START, PAGE_SIZE);

	pml4[0] = BOOT_PDPTE_START | PG_V | PG_RW;
	pdpte[0] = BOOT_PDE_START | PG_V | PG_RW;
	for (int i = 0; i < 512; i++)
		pde[i] = (i*2*1024*1024) | PG_V | PG_RW | PG_PS;

	/* 64-bit protected mode, paging enabled */
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_CR0, CR0_PE | CR0_PG)))
		goto fail;
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_CR4, CR4_PAE)))
		goto fail;
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_EFER, EFER_LME | EFER_LMA)))
		goto fail;
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_CR3, BOOT_PML4_START)))
		goto fail;

	/*
	 * EFLAGS always starts at 0x2 (SDM v1 3.4.3). Linux wants interrupts
	 * disabled to start.
	 */
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RFLAGS, 0x2)))
		goto fail;

	/* 64-bit entry point is kernel load addr + 0x200 */
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RIP, KERNEL_START+0x200)))
		goto fail;

	/* RSI points to boot params */
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RSI, BOOT_PARAMS_START)))
		goto fail;

	/* Initial stack and frame pointers */
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RSP, BOOT_STACK_POINTER)))
		goto fail;
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RBP, BOOT_STACK_POINTER)))
		goto fail;

	/*
	 * Install memory map into boot params. We have to do this here because
	 * setup_memory() runs before other devices have claimed memory regions.
	 */
	struct boot_params *hbp = vm_map_gpa(ctx, BOOT_PARAMS_START,
	    sizeof (struct boot_params));
	hbp->e820_entries = (uint8_t) e820_fill_table(hbp->e820_table,
	    BP_E820_MAX_ENTRIES);

fail:
	return (err);
}

static const struct loader loader_linux = {
	.l_name = "linux",
	.l_setup_memory = loader_linux_setup_memory,
	.l_setup_boot_cpu = loader_linux_setup_boot_cpu,
};
LOADER_SET(loader_linux);

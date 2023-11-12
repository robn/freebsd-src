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

#define	BOOT_PARAMS_START	(0x00007000llu)

/* Initial GDT/IDT location and size */
#define	BOOT_GDT_OFFSET		(0x0500llu)
#define	BOOT_GDT_NENTRIES	(4)
#define	BOOT_IDT_OFFSET		(0x0520llu)
#define	BOOT_IDT_NENTRIES	(1)

#define	MB2_MAGIC	(0xe85250d6)

#define	MB2_DONE			(0)
#define	MB2_INFO_REQUEST		(1)
#define	MB2_ADDRESS			(2)
#define	MB2_ENTRY_ADDRESS		(3)
#define	MB2_FLAGS			(4)
#define	MB2_FRAMEBUFFER			(5)
#define	MB2_MODULE_ALIGNMENT		(6)
#define	MB2_EFI_BOOT_SERVICES		(7)
#define	MB2_EFI_I386_ENTRY_ADDRESS	(8)
#define	MB2_EFI_AMD64_ENTRY_ADDRESS	(9)
#define	MB2_RELOCATABLE			(10)

#define	MB2_FLAG_OPTIONAL		(1<<0)

#define	MB2_PLACEMENT_ANY		(0)
#define	MB2_PLACEMENT_LOW		(1)
#define MB2_PLACEMENT_HIGH		(2)

#define MB2_BOOT_DONE			(0)
#define MB2_BOOT_CMDLINE		(1)
#define MB2_BOOT_LOADER			(2)
#define MB2_BOOT_MODULES		(3)
#define MB2_BOOT_MEMORY_INFO		(4)
#define MB2_BOOT_BIOS_DEV		(5)
#define MB2_BOOT_MEMORY_MAP		(6)
#define MB2_BOOT_VBE			(7)
#define MB2_BOOT_FRAMEBUFFER		(8)
#define MB2_BOOT_ELF			(9)
#define MB2_BOOT_APM			(10)
#define MB2_BOOT_EFI_32_SYSTEM		(11)
#define MB2_BOOT_EFI_64_SYSTEM		(12)
#define MB2_BOOT_SMBIOS			(13)
#define MB2_BOOT_ACPI_RSDP		(14)
#define MB2_BOOT_ACPI2_RSDP		(15)
#define MB2_BOOT_NETWORK		(16)
#define MB2_BOOT_EFI_MEMORY_MAP		(17)
#define MB2_BOOT_EFI_SERVICES_NOEXIT	(18)
#define MB2_BOOT_EFI_32_IMAGE		(19)
#define MB2_BOOT_EFI_64_IMAGE		(20)
#define MB2_BOOT_LOAD_BASE		(21)

struct multiboot2_header {
	uint32_t	magic;
	uint32_t	architecture;
	uint32_t	header_length;
	uint32_t	checksum;
	char		tags[];
} __packed;

struct multiboot2_tag {
	uint16_t			type;
	uint16_t			flags;
	uint32_t			size;
	union {
		/* MB2_RELOCATABLE */
		struct {
			uint32_t	min_addr;
			uint32_t	max_addr;
			uint32_t	align;
			uint32_t	preference;
		} __packed relocatable;
	} __packed;
} __packed;

struct multiboot2_bootinfo {
	uint32_t	size;
	uint32_t	reserved;
	char		tags[];
} __packed;

struct multiboot2_bootinfo_tag {
	uint32_t	type;
	uint32_t	size;
	union {
		/* MB2_CMDLINE */
		struct {
			char		string[1];
		} __packed cmdline;
		/* MB2_BOOT_LOAD_BASE */
		struct {
			uint32_t	load_base_addr;
		} __packed load_base;
	} __packed;
} __packed;

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
loader_multiboot2_setup_memory(struct vmctx *ctx)
{
	(void) ctx;

	int err = 0;
	char *ibase; size_t isize;

	const char *image = get_config_value("loader.image");
	if (!image) {
		EPRINTLN("loader_multiboot2: no image supplied, "
		    "did you set 'loader.image'?");
		return (EINVAL);
	}

	err = _mmap_read_file(image, &ibase, &isize);
	if (err != 0) {
		EPRINTLN("loader_multiboot2: error loading image '%s': %s",
		    image, strerror(err));
		return (err);
	}

	struct multiboot2_header *hdr = NULL;
	for (size_t i = 0; i < 32768-sizeof(struct multiboot2_header); i += 4) {
		hdr = (struct multiboot2_header *)(ibase + i);
		if (hdr->magic != MB2_MAGIC) {
			hdr = NULL;
			continue;
		}

		/* XXX this overflows, do it better */
		if (((hdr->magic + hdr->architecture + hdr->header_length +
		    hdr->checksum) & 0xffffffffu) != 0) {
			EPRINTLN("loader_multiboot2: header at 0x%04lx has "
			    "invalid checksum", i);
			hdr = NULL;
			continue;
		}

		if (hdr->architecture != 0) {
			EPRINTLN("loader_multiboot2: header at 0x%04lx has "
			    "unknown architecture 0x%08x", i, hdr->architecture);
			hdr = NULL;
			continue;
		}

		if ((hdr->header_length < sizeof (struct multiboot2_header)) ||
		    (hdr->header_length & 0x7)) {
			EPRINTLN("loader_multiboot2: header at 0x%04lx has "
			    "invalid length 0x%08x",
			    i, hdr->header_length);
			hdr = NULL;
			continue;
		}

		break;
	}

	if (hdr == NULL) {
		EPRINTLN("loader_multiboot2: multiboot2 header not found");
		err = EINVAL;
		goto done;
	}

	uint32_t load_addr = UINT32_MAX;

	uint32_t tagpos = 0;
	while (tagpos <
	    (hdr->header_length - sizeof (struct multiboot2_header))) {
		struct multiboot2_tag *tag =
		    (struct multiboot2_tag *)&hdr->tags[tagpos];

		EPRINTLN("tag: type %u flags %04x size %08x",
		    tag->type, tag->flags, tag->size);

		switch (tag->type) {
		case MB2_DONE:
			break;

		case MB2_RELOCATABLE: {
			EPRINTLN("loader_multiboot2: reloc header: "
			    "min_addr %08x max_addr %08x "
			    "align %08x preference %08x",
			    tag->relocatable.min_addr,
			    tag->relocatable.max_addr,
			    tag->relocatable.align,
			    tag->relocatable.preference);

			switch (tag->relocatable.preference) {
			case MB2_PLACEMENT_HIGH:
				/* XXX awks */
				/* fall through */

			case MB2_PLACEMENT_ANY:
			case MB2_PLACEMENT_LOW:
				load_addr = roundup(tag->relocatable.min_addr,
				    tag->relocatable.align);
				break;
			}

			break;
		}

		default:
			if (!(tag->flags & MB2_FLAG_OPTIONAL)) {
				EPRINTLN("loader_multiboot2: image requires "
				    "unsupported tag type %d", tag->type);
				err = ENOTSUP;
				goto done;
			}
			break;
		}

		tagpos += roundup(tag->size, 8);
		if (tag->type == MB2_DONE)
			break;
	}

	if (load_addr == UINT32_MAX) {
		/* XXX probably just load it at top of RAM */
		EPRINTLN("loader_multiboot2: unable to determine load address");
		err = EINVAL;
		goto done;
	}

	EPRINTLN("loader_multiboot2: load_addr %08x", load_addr);

	char *hibase = vm_map_gpa(ctx, load_addr, isize);
	memcpy(hibase, ibase, isize);

	/* XXX hacked up just for hedron */
	struct multiboot2_bootinfo *bootinfo = (struct multiboot2_bootinfo *)
	    vm_map_gpa(ctx, 0x7000llu, PAGE_SIZE);

	tagpos = 0;
	struct multiboot2_bootinfo_tag *tag;

	tag = (struct multiboot2_bootinfo_tag *)&bootinfo->tags[tagpos];
	tag->type = MB2_BOOT_LOAD_BASE;
	tag->size = 12;
	tag->load_base.load_base_addr = load_addr;
	tagpos += roundup(tag->size, 8);

	tag = (struct multiboot2_bootinfo_tag *)&bootinfo->tags[tagpos];
	tag->type = MB2_BOOT_CMDLINE;
	strcpy(tag->cmdline.string, "serial");
	tag->size = 8 + strlen(tag->cmdline.string) + 1;
	tagpos += roundup(tag->size, 8);

	tag = (struct multiboot2_bootinfo_tag *)&bootinfo->tags[tagpos];
	tag->type = MB2_BOOT_DONE;
	tag->size = 8;
	tagpos += roundup(tag->size, 8);

	bootinfo->size = sizeof (struct multiboot2_bootinfo) + tagpos;
	bootinfo->reserved = 0;

done:
	munmap(ibase, isize);
	return (err);
}

static int
loader_multiboot2_setup_boot_cpu(struct vmctx *ctx, struct vcpu *vcpu)
{
	EPRINTLN("lets go multiboot!");

	int err = 0;

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

	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_CS, 1)))
		goto fail;

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

	if ((err = _set_reg_desc(vcpu, VM_REG_GUEST_TR, 3)))
		goto fail;

	if ((err = vm_set_register(vcpu, VM_REG_GUEST_CR0, CR0_PE)))
		goto fail;
	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RFLAGS, 0x2)))
		goto fail;

	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RAX, 0x36d76289)))
		goto fail;

	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RBX, 0x7000llu)))
		goto fail;

	if ((err = vm_set_register(vcpu, VM_REG_GUEST_RIP, 0x6600000llu)))
		goto fail;

fail:
	return (err);
}

static const struct loader loader_multiboot2 = {
	.l_name = "multiboot2",
	.l_setup_memory = loader_multiboot2_setup_memory,
	.l_setup_boot_cpu = loader_multiboot2_setup_boot_cpu,
};
LOADER_SET(loader_multiboot2);

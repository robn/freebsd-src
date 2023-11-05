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

#ifndef	_LOADER_H_
#define	_LOADER_H_

#include "config.h"

struct vmctx;
struct vcpu;

struct loader {
	const char	*l_name;

	int	(*l_setup_memory)(struct vmctx *ctx);
	int	(*l_setup_boot_cpu)(struct vmctx *ctx, struct vcpu *vcpu);
};
#define	LOADER_SET(l)	DATA_SET(loader_set, l)

int loader_init(struct loader **olp);

#define	_l_methodcall(l, m, args...)	\
	(((l) && (l)->m) ? (l)->m(args) : (0))

#define	loader_setup_memory(l, ctx)		\
	_l_methodcall(l, l_setup_memory, ctx)
#define	loader_setup_boot_cpu(l, ctx, vcpu)	\
	_l_methodcall(l, l_setup_boot_cpu, ctx, vcpu)

#endif

/*
 * manhattan - asm_io.h
 *
 * Copyright (C) 2012  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef ASM_IO_INCLUDED
#define ASM_IO_INCLUDED

#include "compiler.h"
#include "asm_types.h"
#include "string.h"

#define __raw_readb(addr) (*(const volatile u8 *)(addr))
#define __raw_readw(addr) (*(const volatile u16 *)(addr))
#define __raw_readl(addr) (*(const volatile u32 *)(addr))
#define __raw_writeb(x, addr) (*(volatile u8 *)(addr) = (x))
#define __raw_writew(x, addr) (*(volatile u16 *)(addr) = (x))
#define __raw_writel(x, addr) (*(volatile u32 *)(addr) = (x))

static inline u8 readb(const volatile void __iomem *addr)
{
	u8 ret = __raw_readb(addr);
	mb();
	return ret;
}

static inline u16 readw(const volatile void __iomem *addr)
{
	u16 ret = __raw_readw(addr);
	mb();
	return ret;
}

static inline u32 readl(const volatile void __iomem *addr)
{
	u32 ret = __raw_readl(addr);
	mb();
	return ret;
}

static inline void writeb(u8 b, volatile void __iomem *addr)
{
	__raw_writeb(b, addr);
	mb();
}

static inline void writew(u16 b, volatile void __iomem *addr)
{
	__raw_writew(b, addr);
	mb();
}

static inline void writel(u32 b, volatile void __iomem *addr)
{
	__raw_writel(b, addr);
	mb();
}

static inline void memcpy_fromio(void *dst, const void *src, size_t count)
{
	memcpy(dst, src, count);
	mb();
}

static inline void memcpy_toio(void *dst, const void *src, size_t count)
{
	memcpy(dst, src, count);
	mb();
}

#endif

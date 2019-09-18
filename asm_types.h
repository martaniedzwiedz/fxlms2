/*
 * manhattan - asm_types.h
 *
 * Copyright (C) 2012  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef ASM_TYPES_INCLUDED
#define ASM_TYPES_INCLUDED

#include "compiler.h"
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

typedef u16 __bitwise be16;
typedef u32 __bitwise be32;

#endif

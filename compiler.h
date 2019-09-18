/*
 * manhattan - compiler.h
 *
 * Copyright (C) 2010..2012  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * some code taken from Linux 3.6
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef COMPILER_H_INCLUDED
#define COMPILER_H_INCLUDED

#include <unistd.h>

#if __GNUC__ < 2 || (__GNUC__ == 2 && __GNUC_MINOR__ < 96)
# define __builtin_expect(x, expected_value) (x)
#endif

#define likely(x)	__builtin_expect((x), 1)
#define unlikely(x)	__builtin_expect((x), 0)

#ifdef __CHECKER__
#define __bitwise__ __attribute__((bitwise))
#else
#define __bitwise__
#endif
#ifdef __CHECK_ENDIAN__
#define __bitwise __bitwise__
#else
#define __bitwise
#endif

#define __packed	__attribute__((packed))

#if __GNUC__
# define barrier()	__asm__ __volatile__ ("" : : : "memory")
#else
# define barrier()	do { } while (0)
#endif

#if __CHECKER__
#define mb() barrier()
#endif

#ifndef mb
#if (__GNUC > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 1)
#define mb() __sync_synchronize()
#define rmb() mb()
#define wmb() mb()
#else
#define mb()				barrier()
#define rmb()				mb()
#define wmb()				mb()
#endif
#endif

#ifndef __iomem
#define __iomem
#endif

/* The `const' in roundup() prevents gcc-3.3 from calling __divdi3 */
#define roundup(x, y) ({				\
	const typeof(y) __y = y;			\
	(((x) + (__y - 1)) / __y) * __y;		\
})

/*
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 */
#define container_of(ptr, type, member) ({			\
	typeof(((type *)0)->member) *__mptr = (ptr);	\
	(type *)((char *)__mptr - offsetof(type, member)); })

#define get_unaligned(ptr)						\
	({								\
		const struct __str {					\
			typeof(*(ptr)) __v;				\
		} __packed *__p = (const struct __str *) (ptr);		\
		__p->__v;						\
	})

#define put_unaligned(val, ptr)						\
	do {								\
		struct __str {						\
			typeof(*(ptr)) __v;				\
		} __packed *__p = (struct __str *) (ptr);		\
		__p->__v = (val);					\
	} while (0)

#if defined(__HP_cc) && (__HP_cc >= 61000)
#define NORETURN __attribute__((noreturn))
#define NORETURN_PTR
#elif defined(__GNUC__) && !defined(NO_NORETURN)
#define NORETURN __attribute__((__noreturn__))
#define NORETURN_PTR __attribute__((__noreturn__))
#elif defined(_MSC_VER)
#define NORETURN __declspec(noreturn)
#define NORETURN_PTR
#else
#define NORETURN
#define NORETURN_PTR
#ifndef __attribute__
#define __attribute__(x)
#endif
#endif

#endif

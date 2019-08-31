/*
 * lflib - ring.h
 *
 * Copyright (C) 2010..2014  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * ring buffer implementation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef LF_RING_H_INCLUDED
#define LF_RING_H_INCLUDED

struct lf_ring {
	float *x;		/* buffer data */
	unsigned int mask;	/* mask used to mask buffer elements */
	unsigned int i;		/* index of current sample */
	unsigned int n;		/* length of buffer */
	int allocated;		/* buffer is dynamically allocated */
};

#define LF_RING_STATIC_SIZE(min_n)	(\
		(min_n < 16) ? 16 : \
		(min_n < 64) ? 64 : \
		(min_n < 128) ? 128 : \
		(min_n < 256) ? 256 : \
		(min_n < 512) ? 512 : \
		(min_n < 1024) ? 1024 : \
		(min_n < 2048) ? 2048 : \
		(min_n < 4096) ? 4096 : \
		(min_n < 8192) ? 8192 : -1\
	)

#define LF_RING_DECLARE(name, min_n) \
	float name[LF_RING_STATIC_SIZE(min_n)]

#define lf_ring_get(r, j)	((r)->x[((r)->i + (j)) & (r)->mask])

/**
 * lf_ring_add - add sample to ring buffer
 * @r: ring buffer
 * @v: new sample
 */
static inline void lf_ring_add(struct lf_ring *r, float v)
{
	r->i = (r->i - 1) & r->mask;
	r->x[r->i] = v;
}

int lf_ring_init(struct lf_ring *r, unsigned int min_n);
int lf_ring_set_buffer(struct lf_ring *r, float *x, unsigned int m);
void lf_ring_destroy(struct lf_ring *r);


#endif

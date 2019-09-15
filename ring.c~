/*
 * librtdsp - ring.c
 * Copyright (C) 2010  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Fundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include "ring.h"
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

/**
 * lf_ring_init - initialize ring buffer
 * @r: ring buffer
 * @min_n: minimal ring buffer length
 *
 * This function returns 0 on success, negative error code otherwise.
 */
int lf_ring_init(struct lf_ring *r, unsigned int min_n)
{
	unsigned int n;

	for (n = 1; n < min_n; n *= 2);

	r->n = n;
	r->mask = n - 1;
	r->i = 0;
	r->x = NULL;
	r->allocated = 0;
	return 0;
}

/**
 * lf_ring_set_buffer - set ring buffer
 * @r: ring buffer
 * @x: pointer to buffer memory, if NULL then dynamically allocated
 * @m: length of buffer
 * 
 * This function returns 0 on success, negative error code otherwise.
 */
int lf_ring_set_buffer(struct lf_ring *r, float *x, unsigned int m)
{
	unsigned int i;
	int allocated = 0;

	if (x == NULL) {
		x = malloc(r->n * sizeof(float));
		if (x == NULL)
			return -ENOMEM;
		allocated = 1;
	} else {
		if (m < r->n)
			return -EINVAL;
	}

	for (i = 0; i < r->n; i++)
		x[i] = 0.0;

	if (r->allocated)
		free(r->x);

	r->x = x;
	r->allocated = allocated;
	return 0;
}

/**
 * lf_ring_destroy - destroy ring buffer
 * @r: ring buffer to destroy
 */
void lf_ring_destroy(struct lf_ring *r)
{
	if (r->allocated)
		free(r->x);
}

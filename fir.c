/*
 * lflib - fir.c
 * Copyright (C) 2009..2014  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include "fir.h"
#include <stdio.h>

#ifndef _nassert
#define _nassert(cnd)	do { } while (0)
#endif

static float __lf_fir(const float *restrict x,
		const float *restrict w,
		unsigned int i, unsigned int mask, unsigned int n)
{
	unsigned int j;
	unsigned int chunk;
	float sum0 = 0;
	float sum1 = 0;
	float sum2 = 0;
	float sum3 = 0;

	_nassert (((int)(x) & 0x7) == 0);
	_nassert (((int)(w) & 0x7) == 0);
	chunk = (mask + 1) - i;
	if (chunk > n)
		chunk = n;
	j = 0;
	if (chunk >= 4) {
		for (; j < chunk - 3; j += 4, i += 4) {
			sum0 += x[i + 0] * w[j + 0];
			sum1 += x[i + 1] * w[j + 1];
			sum2 += x[i + 2] * w[j + 2];
			sum3 += x[i + 3] * w[j + 3];
		}
	}

	for (; j < chunk; j++, i++)
		sum0 += x[i] * w[j];

	i = 0;
	if (n >= 4) {
		for (; j < n - 3; j += 4, i += 4) {
			sum0 += x[i + 0] * w[j + 0];
			sum1 += x[i + 1] * w[j + 1];
			sum2 += x[i + 2] * w[j + 2];
			sum3 += x[i + 3] * w[j + 3];
		}
	}

	for (; j < n; j++, i++)
		sum0 += x[i] * w[j];

	sum0 += sum1;
	sum2 += sum3;
	sum0 += sum2;
	return sum0;
}

/**
 * lf_fir - compute FIR filter output
 * @r: ring buffer
 * @w: FIR coefficients
 * @n: number of coefficients
 *
 * This function returns FIR filter output.
 */
float lf_fir(const struct lf_ring *r, const float *w, unsigned int n)
{
	return __lf_fir(r->x, w, r->i, r->mask, n);
}

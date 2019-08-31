/*
 * lflib - fir.h
 * Copyright (C) 2009..2014  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef LF_FIR_H_INCLUDED
#define LF_FIR_H_INCLUDED

#include "ring.h"

float lf_fir(const struct lf_ring *r, const float *w, unsigned int n);

#endif

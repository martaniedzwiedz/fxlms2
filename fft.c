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
#include <stdlib.h>
#include <complex.h>



double PI = 3.14;
//#include "fft.h"

void show(const char *s, complex *buf, int size) {
    printf("%s\n", s);
    for (int i = 0; i < size; i++) {
        if (!cimag(buf[i]))
            printf("%1.15f ", creal(buf[i]));
        else
            printf("(%1.15f, %1.15f) ", creal(buf[i]), cimag(buf[i]));
        printf("\n");
    }
}

void _fft(complex *buf, complex out[], int n, int step) {

    if (step < n) {
        _fft(out, buf, n, step * 2);
        _fft(out + step, buf + step, n, step * 2);
        for (int i = 0; i < n; i += 2 * step) {

            complex t = cexp(-I * PI * i / n) * out[i + step];
            buf[i / 2] = out[i] + t;
            buf[(i + n) / 2] = out[i] - t;
        }
    }
}

void fft(complex *buf, int size) {
    complex out[size];
    for (int i = 0; i < size; i++) {
        out[i] = buf[i];
    }
    _fft(buf, out, size, 1);
}

void ifft(complex *buf, int n) {
    for (int i = 0; i < n; i++) buf[i] = conj(buf[i]);
    fft(buf, n);
    for (int i = 0; i < n; i++) buf[i] = conj(buf[i]);
    for (int i = 0; i < n; i++) buf[i] /= n;
}



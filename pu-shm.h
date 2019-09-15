/*
 * pu-shm.h
 *
 * pu shared memory communication protocol
 *
 * Copyright (C) 2012  Krzysztof Mazur <krzysiek@podlesie.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef PU_SHM_H_INCLUDED
#define PU_SHM_H_INCLUDED

/*
 * PU header located at beginning of shm
 */
#define DSP_MAGIC		0x00
#define DSP_VERSION		0x04
#define DSP_SEQNUM		0x08
#define DSP_GENERATION		0x10
#define DSP_SAMPLE_SIZE		0x14
#define DSP_RING_ENTRIES	0x18
#define DSP_SAMPLES_PER_GROUP	0x1c
//#define DSP_SHM_SIZE		0x20
#define DSP_IGROUP		0x28	/* last important group */
#define DSP_FIRST		0x30

#define DSP_GENERATION_BUSY	0x80000000

//#define DSP_ALIGNMENT		0x1000
//#define DSP_RING_BASE		0x1000

#define DSP_MAGIC_VALUE		0x50750a00

#endif

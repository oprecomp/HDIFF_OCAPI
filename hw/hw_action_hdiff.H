#ifndef __ACTION_HLS_HDIFF_H__
#define __ACTION_HLS_HDIFF_H__

/*
 * Copyright 2017 International Business Machines
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <string.h>
#include <ap_int.h>

#include "hls_snap_1024.H"
#include <action_hdiff.h> /* Memcopy Job definition */


//#define MAX_NB_OF_LINES_READ 2 // stencil 8*8
#define MAX_NB_OF_LINES_READ 512 // stencil 128*128
//#define MAX_NB_OF_LINES_READ 2048 // stencil 256*256


//#define TOTAL_MEMORY_LINES 16 //8x8x8
#define TOTAL_MEMORY_LINES 40960 //128*128*80 -----  HORIZONTAL DIFFUSION
// #define TOTAL_MEMORY_LINES 131072 //256*256*64 -----  HORIZONTAL DIFFUSION

#define MANY_ACC 1


#define MAX_NB_OF_BYTES_READ MAX_NB_OF_LINES_READ*64

// #define MAX_NB_OF_BYTES_READ	(256 * 1024)
#define CARD_DRAM_SIZE		(1 * 1024 *1024 * 1024)
#define MAX_NB_OF_WORDS_READ_512    (MAX_NB_OF_BYTES_READ/BPERDW_512)
#define MAX_NB_OF_WORDS_READ_1024	(MAX_NB_OF_BYTES_READ/BPERDW_1024)

//---------------------------------------------------------------------
typedef struct {
	CONTROL Control;	/*  16 bytes */
	hdiff_job_t Data;	/* up to 108 bytes */
	uint8_t padding[SNAP_HLS_JOBSIZE - sizeof(hdiff_job_t)];
} action_reg;
	// change to float or double depending on your needs

#endif  /* __ACTION_HLS_HDIFF_H__ */

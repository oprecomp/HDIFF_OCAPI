/*
 * Copyright 2019 International Business Machines
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
#ifndef __ACTION_HDIFF_H__
#define __ACTION_HDIFF_H__

/*
 * Copyright 2016, 2017 International Business Machines
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

#include <osnap_types.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------ MUST READ -----------
// ACTION_TYPE and RELEASE_LEVEL are automatically handled. 
// 1. Define them in header file (here), use HEX 32bits numbers
// 2. They will be extracted by hardware/setup/patch_version.sh
// 3. And put into snap_global_vars.v
// 4. Used by hardware/hls/action_wrapper.v
#define ACTION_TYPE               0x10143009
#define RELEASE_LEVEL             0x00000003
// For snap_maint, Action descriptions are decoded with the help of software/tools/snap_actions.h
// Please modify this file so snap_maint can recognize this action.
// ------------ MUST READ -----------


typedef float  mat_elmt_t; 
typedef struct hdiff_job {
	struct snap_addr in;	/* input data */
	struct snap_addr out;   /* offset table */
} hdiff_job_t;


#define GRIDROWS 128
#define GRIDCOLUMNS 128
#define GRIDDEPTH 80

#define ROWS 128
#define COLUMNS 128
#define DEPTH 80


//
//#define GRIDROWS 8
//#define GRIDCOLUMNS 8
//#define GRIDDEPTH 8
//
//#define ROWS 8
//#define COLUMNS 8
//#define DEPTH 8



/*
 #define GRIDROWS 256
#define GRIDCOLUMNS 256
 #define GRIDDEPTH 64

#define ROWS 256
#define COLUMNS 256
#define DEPTH 64
*/


#ifdef __cplusplus
}
#endif

#endif	/* __ACTION_HDIFF_H__ */

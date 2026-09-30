/*
==========================================================================
    Copyright (C) 2025,2026 Axel Sandstedt 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==========================================================================
*/

#ifndef __DREAMSCAPE_TYPES_H__
#define __DREAMSCAPE_TYPES_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#include "ds_define.h"

/************** Common Types **************/

typedef	uint8_t 	u8;
typedef	uint16_t	u16;
typedef	uint32_t 	u32;
typedef	uint64_t 	u64;

typedef	int8_t  	i8;
typedef	int16_t 	i16;
typedef	int32_t 	i32;
typedef	int64_t 	i64;

typedef float		f32;
typedef double		f64;

typedef union
{ 
	i8 	i;
       	u8 	u; 
} b8;

typedef union
{ 
	i16 	i;
       	u16 	u; 
} b16;

typedef union
{ 
	i32 	i;
       	u32 	u; 
	f32	f;
} b32;

typedef union
{ 
	i64 	i;
       	u64 	u; 
	f64	f;
} b64;

#define U8_MAX		0xff
#define U16_MAX		0xffff
#define U32_MAX		0xffffffff
#define U64_MAX		0xffffffffffffffff

#define I8_MAX		127
#define I16_MAX		32767
#define I32_MAX		2147483647
#define I64_MAX		9223372036854775807

#define I8_MIN		((b64) { .u = 0x80 }).i	
#define I16_MIN		((b64) { .u = 0x8000 }).i	
#define I32_MIN		((b64) { .u = 0x80000000 }).i	
#define I64_MIN		((b64) { .u = 0x8000000000000000 }).i	

/************** Allocator Helper **************/

struct slot
{
	void *	address;
	u32	index;
};

#define empty_slot	(struct slot) { .address = NULL, .index = U32_MAX }

/************** Math Types **************/

/* { x, y, z, w }, w is real part */
typedef f32 quat[4];
typedef f32 (*quatptr)[4];

/* { x, y, z, w }, w is real part */
typedef union q
{
	struct 
    { 
        f32 x;
        f32 y; 
        f32 z; 
        f32 w; 
    };
	f32 buf[4];
} q;

#define Q(_x, _y, _z, _w)	((q) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

typedef f32 vec2[2];
typedef f32 vec3[3];
typedef f32 vec4[4];

typedef f32 (*vec2ptr)[2];
typedef f32 (*vec3ptr)[3];
typedef f32 (*vec4ptr)[4];

typedef const f32 (*constvec2ptr)[2];
typedef const f32 (*constvec3ptr)[3];
typedef const f32 (*constvec4ptr)[4];

typedef u32 vec2u32[2];
typedef u32 vec3u32[3];
typedef u32 vec4u32[4];

typedef u32 (*vec2u32ptr)[2];
typedef u32 (*vec3u32ptr)[3];
typedef u32 (*vec4u32ptr)[4];

typedef u64 vec2u64[2];
typedef u64 vec3u64[3];
typedef u64 vec4u64[4];

typedef u64 (*vec2u64ptr)[2];
typedef u64 (*vec3u64ptr)[3];
typedef u64 (*vec4u64ptr)[4];

typedef i32 vec2i32[2];
typedef i32 vec3i32[3];
typedef i32 vec4i32[4];

typedef i32 (*vec2i32ptr)[2];
typedef i32 (*vec3i32ptr)[3];
typedef i32 (*vec4i32ptr)[4];

typedef i64 vec2i64[2];
typedef i64 vec3i64[3];
typedef i64 vec4i64[4];

typedef i64 (*vec2i64ptr)[2];
typedef i64 (*vec3i64ptr)[3];
typedef i64 (*vec4i64ptr)[4];

typedef union v2
{
	struct 
    { 
        f32 x;
        f32 y; 
    };
	f32 buf[2];
} v2;

typedef union v3
{
	struct 
    { 
        f32 x;
        f32 y; 
        f32 z; 
    };
	f32 buf[3];
} v3;

typedef union v4
{
	struct 
    { 
        f32 x;
        f32 y; 
        f32 z; 
        f32 w; 
    };
	f32 buf[4];
} v4;

typedef union v2u32
{
	struct 
    { 
        u32 x;
        u32 y; 
    };
	u32 buf[2];
} v2u32;

typedef union v3u32
{
	struct 
    { 
        u32 x;
        u32 y; 
        u32 z; 
    };
	u32 buf[3];
} v3u32;

typedef union v4u32
{
	struct 
    { 
        u32 x;
        u32 y; 
        u32 z; 
        u32 w; 
    };
	u32 buf[4];
} v4u32;

typedef union v2u64
{
	struct 
    { 
        u64 x;
        u64 y; 
    };
	u64 buf[2];
} v2u64;

typedef union v3u64
{
	struct 
    { 
        u64 x;
        u64 y; 
        u64 z; 
    };
	u64 buf[3];
} v3u64;

typedef union v4u64
{
	struct 
    { 
        u64 x;
        u64 y; 
        u64 z; 
        u64 w; 
    };
	u64 buf[4];
} v4u64;

typedef union v2i32
{
	struct 
    { 
        i32 x;
        i32 y; 
    };
	i32 buf[2];
} v2i32;

typedef union v3i32
{
	struct 
    { 
        i32 x;
        i32 y; 
        i32 z; 
    };
	i32 buf[3];
} v3i32;

typedef union v4i32
{
	struct 
    { 
        i32 x;
        i32 y; 
        i32 z; 
        i32 w; 
    };
	i32 buf[4];
} v4i32;

typedef union v2i64
{
	struct 
    { 
        i64 x;
        i64 y; 
    };
	i64 buf[2];
} v2i64;

typedef union v3i64
{
	struct 
    { 
        i64 x;
        i64 y; 
        i64 z; 
    };
	i64 buf[3];
} v3i64;

typedef union v4i64
{
	struct 
    { 
        i64 x;
        i64 y; 
        i64 z; 
        i64 w; 
    };
	i64 buf[4];
} v4i64;

#define V2(_x, _y)	((v2) { .x = (_x), .y = (_y) })
#define V3(_x, _y, _z)	((v3) { .x = (_x), .y = (_y), .z = (_z) })
#define V4(_x, _y, _z, _w)	((v4) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

#define V2U32(_x, _y)	((v2u32) { .x = (_x), .y = (_y) })
#define V3U32(_x, _y, _z)	((v3u32) { .x = (_x), .y = (_y), .z = (_z) })
#define V4U32(_x, _y, _z, _w)	((v4u32) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

#define V2U64(_x, _y)	((v2u64) { .x = (_x), .y = (_y) })
#define V3U64(_x, _y, _z)	((v3u64) { .x = (_x), .y = (_y), .z = (_z) })
#define V4U64(_x, _y, _z, _w)	((v4u64) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

#define V2I32(_x, _y)	((v2i32) { .x = (_x), .y = (_y) })
#define V3I32(_x, _y, _z)	((v3i32) { .x = (_x), .y = (_y), .z = (_z) })
#define V4I32(_x, _y, _z, _w)	((v4i32) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

#define V2I64(_x, _y)	((v2i64) { .x = (_x), .y = (_y) })
#define V3I64(_x, _y, _z)	((v3i64) { .x = (_x), .y = (_y), .z = (_z) })
#define V4I64(_x, _y, _z, _w)	((v4i64) { .x = (_x), .y = (_y), .z = (_z), .w = (_w) })

/*
 * Column-major: aij is row i, column j. Elements are stored column by column
 * (a11, a21, a31, a12, ...), so col[j-1] is column j and buf[(j-1)*N + (i-1)] is aij.
 */
typedef union m2
{
	struct 
    { 
        f32 a11, a21;	/* column 1 */
        f32 a12, a22;	/* column 2 */
    };
	v2 col[2];
	f32 buf[4];
} m2;

typedef union m3
{
	struct 
    { 
        f32 a11, a21, a31;	/* column 1 */
        f32 a12, a22, a32;	/* column 2 */
        f32 a13, a23, a33;	/* column 3 */
    };
	v3 col[3];
	f32 buf[9];
} m3;

typedef union m4
{
	struct 
    { 
        f32 a11, a21, a31, a41;	/* column 1 */
        f32 a12, a22, a32, a42;	/* column 2 */
        f32 a13, a23, a33, a43;	/* column 3 */
        f32 a14, a24, a34, a44;	/* column 4 */
    };
	v4 col[4];
	f32 buf[16];
} m4;

/* Arguments in column-major order (a11, a21, a31, a12, ...) */
#define M2(_a11, _a21,\
           _a12, _a22)	\
	((m2) { .a11 = (_a11), .a21 = (_a21), .a12 = (_a12), .a22 = (_a22) })

#define M3(_a11, _a21, _a31,\
           _a12, _a22, _a32,\
           _a13, _a23, _a33)	\
	((m3) { .a11 = (_a11), .a21 = (_a21), .a31 = (_a31), .a12 = (_a12), .a22 = (_a22), .a32 = (_a32), .a13 = (_a13), .a23 = (_a23), .a33 = (_a33) })

#define M4(_a11, _a21, _a31, _a41,\
           _a12, _a22, _a32, _a42,\
           _a13, _a23, _a33, _a43,\
           _a14, _a24, _a34, _a44)	\
	((m4) { .a11 = (_a11), .a21 = (_a21), .a31 = (_a31), .a41 = (_a41), .a12 = (_a12), .a22 = (_a22), .a32 = (_a32), .a42 = (_a42), .a13 = (_a13), .a23 = (_a23), .a33 = (_a33), .a43 = (_a43), .a14 = (_a14), .a24 = (_a24), .a34 = (_a34), .a44 = (_a44) })

typedef vec2 mat2[2];
typedef vec3 mat3[3];
typedef vec4 mat4[4];

typedef vec2 (*mat2ptr)[2];
typedef vec3 (*mat3ptr)[3];
typedef vec4 (*mat4ptr)[4];

struct dsBuffer
{
	u8 *	data;
	u64 	size; 
	u64 	mem_left;
};
#define ds_buffer_empty	(struct dsBuffer) { .data = NULL, .size = 0, .mem_left = 0, }

typedef struct intv
{
	union
	{
		struct
		{
			f32	low;
			f32	high;
		};

		vec2	v;
	};
} intv;
#define intv_inline(_low, _high) (intv) { .low = (_low), .high = (_high) }

typedef struct
{
	union
	{
		struct
		{
			u64	low;
			u64	high;
		};

		vec2u64	v;
	};
} intvu64;
#define intvu64_inline(_low, _high) (intvu64) { .low = (_low), .high = (_high) }

typedef struct
{
	union
	{
		struct
		{
			i64	low;
			i64	high;
		};

		vec2i64	v;
	};
} intvi64;
#define intvi64_inline(_low, _high) (intvi64) { .low = (_low), .high = (_high) }

typedef struct
{
	u32	u;
	f32	f;
} u32f32;
#define u32f32_inline(u_in, f_in)	(u32f32) { .u = u_in, .f = f_in }

union reg
{
	u8	u8;
	u16	u16;
	u32	u32;
	u64	u64;
	i8	i8;
	i16	i16;
	i32	i32;
	i64	i64;
	f32	f32;
	f64	f64;
	void *	ptr;
	intv	intv;
};

typedef enum axis_2
{
	AXIS_2_X = 0,
	AXIS_2_Y = 1,
	AXIS_2_COUNT
} axis_2;

enum alignment_x
{
	ALIGN_LEFT,
	ALIGN_X_CENTER,
	ALIGN_RIGHT,
	ALIGN_X_COUNT
};

enum alignment_y
{
	ALIGN_TOP,
	ALIGN_Y_CENTER,
	ALIGN_BOTTOM,
	ALIGN_Y_COUNT
};

typedef enum axis_3
{
	AXIS_3_X = 0,
	AXIS_3_Y = 1,
	AXIS_3_Z = 2,
	AXIS_3_COUNT
} axis_3;

typedef enum box_corner 
{
	BOX_CORNER_BR = 0,
	BOX_CORNER_TR = 1,
	BOX_CORNER_TL = 2,
	BOX_CORNER_BL = 3,
	BOX_CORNER_COUNT
} box_corner;

/* system identifiers for Logger, profiler ... */
enum system_id
{
	T_SYSTEM,
	T_RENDERER,
	T_PHYSICS,
	T_CSG,
	T_ASSET,
	T_UTILITY,
	T_PROFILER,
	T_ASSERT,
	T_GAME,
	T_UI,
	T_LED,
	T_COUNT
};

/* system identifiers for Logger, profiler ... */
enum severity_id
{
	S_SUCCESS,
	S_NOTE,
	S_WARNING,
	S_ERROR,
	S_FATAL,
	S_COUNT
};



#ifdef __cplusplus
}
#endif

#endif

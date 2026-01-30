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

typedef struct
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

/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

#ifndef __MATRIX_MATH__
#define __MATRIX_MATH__

#ifdef __cplusplus
extern "C" { 
#endif

#include "vector.h"

void mat2_print(const char *text, mat2 m);
void mat3_print(const char *text, mat3 m);
void mat4_print(const char *text, mat4 m);

/* Fill in column-major order */
void mat2_set(mat2 dst, const f32 a11, const f32 a21,
    			const f32 a12, const f32 a22);

/* Fill in column-major order */
void mat3_set(mat3 dst, const f32 a11, const f32 a21, const f32 a31,
			const f32 a12, const f32 a22, const f32 a32,  
			const f32 a13, const f32 a23, const f32 a33);

/* Fill in column-major order */
void mat4_set(mat4 dst, const f32 a11, const f32 a21, const f32 a31, const f32 a41,
			const f32 a12, const f32 a22, const f32 a32, const f32 a42, 
			const f32 a13, const f32 a23, const f32 a33, const f32 a43, 
			const f32 a14, const f32 a24, const f32 a34, const f32 a44);

void mat2_set_columns(mat2 dst, const vec2 c1, const vec2 c2);
void mat3_set_columns(mat3 dst, const vec3 c1, const vec3 c2, const vec3 c3);
void mat4_set_columns(mat4 dst, const vec4 c1, const vec4 c2, const vec4 c3, const vec4 c4);

void mat2_set_rows(mat2 dst, const vec2 r1, const vec2 r2);
void mat3_set_rows(mat3 dst, const vec3 r1, const vec3 r2, const vec3 r3);
void mat4_set_rows(mat4 dst, const vec4 r1, const vec4 r2, const vec4 r3, const vec4 r4);

void mat2_identity(mat2 dst);
void mat3_identity(mat3 dst);
void mat4_identity(mat4 dst);

/* dst = vec*mat */
void vec2_mat_mul(vec2 dst, const vec2 vec, mat2 mat);
void vec3_mat_mul(vec3 dst, const vec3 vec, mat3 mat);
void vec4_mat_mul(vec4 dst, const vec4 vec, mat4 mat);

/* dst = mat * vec */
void mat2_vec_mul(vec2 dst, mat2 mat, const vec2 vec);
void mat3_vec_mul(vec3 dst, mat3 mat, const vec3 vec);
void mat4_vec_mul(vec4 dst, mat4 mat, const vec4 vec);

void mat2_add(mat2 dst, mat2 a, mat2 b);
void mat3_add(mat3 dst, mat3 a, mat3 b);
void mat4_add(mat4 dst, mat4 a, mat4 b);

/* dst = a*b */
void mat2_mult(mat2 dst, mat2 a, mat2 b);
void mat3_mult(mat3 dst, mat3 a, mat3 b);
void mat4_mult(mat4 dst, mat4 a, mat4 b);

/* dst = src^T */
void mat2_transpose_to(mat2 dst, mat2 src);
void mat3_transpose_to(mat3 dst, mat3 src);
void mat4_transpose_to(mat4 dst, mat4 src);

/* Returns determinant of src, and sets inverse, or garbage if it does not exist. */
f32 mat2_inverse(mat2 dst, mat2 src);
f32 mat3_inverse(mat3 dst, mat3 src);
f32 mat4_inverse(mat4 dst, mat4 src);

/* min |a_ij| */
f32 mat2_abs_min(mat2 src);
f32 mat3_abs_min(mat3 src);
f32 mat4_abs_min(mat4 src);

/* max |a_ij| */
f32 mat2_abs_max(mat2 src);
f32 mat3_abs_max(mat3 src);
f32 mat4_abs_max(mat4 src);

void mat2_copy(mat2 dst, mat2 src);
void mat3_copy(mat3 dst, mat3 src);
void mat4_copy(mat4 dst, mat4 src);

#ifdef __cplusplus
} 
#endif

#endif


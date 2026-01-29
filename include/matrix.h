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

void Mat2Print(const char *text, mat2 m);
void Mat3Print(const char *text, mat3 m);
void Mat4Print(const char *text, mat4 m);

/* Fill in column-major order */
void Mat2Set(mat2 dst, const f32 a11, const f32 a21,
    			const f32 a12, const f32 a22);

/* Fill in column-major order */
void Mat3Set(mat3 dst, const f32 a11, const f32 a21, const f32 a31,
			const f32 a12, const f32 a22, const f32 a32,  
			const f32 a13, const f32 a23, const f32 a33);

/* Fill in column-major order */
void Mat4Set(mat4 dst, const f32 a11, const f32 a21, const f32 a31, const f32 a41,
			const f32 a12, const f32 a22, const f32 a32, const f32 a42, 
			const f32 a13, const f32 a23, const f32 a33, const f32 a43, 
			const f32 a14, const f32 a24, const f32 a34, const f32 a44);

void Mat2SetColumns(mat2 dst, const vec2 c1, const vec2 c2);
void Mat3SetColumns(mat3 dst, const vec3 c1, const vec3 c2, const vec3 c3);
void Mat4SetColumns(mat4 dst, const vec4 c1, const vec4 c2, const vec4 c3, const vec4 c4);

void Mat2SetRows(mat2 dst, const vec2 r1, const vec2 r2);
void Mat3SetRows(mat3 dst, const vec3 r1, const vec3 r2, const vec3 r3);
void Mat4SetRows(mat4 dst, const vec4 r1, const vec4 r2, const vec4 r3, const vec4 r4);

void Mat2Identity(mat2 dst);
void Mat3Identity(mat3 dst);
void Mat4Identity(mat4 dst);

/* dst = vec*mat */
void Vec2MatMul(vec2 dst, const vec2 vec, mat2 mat);
void Vec3MatMul(vec3 dst, const vec3 vec, mat3 mat);
void Vec4MatMul(vec4 dst, const vec4 vec, mat4 mat);

/* dst = mat * vec */
void Mat2VecMul(vec2 dst, mat2 mat, const vec2 vec);
void Mat3VecMul(vec3 dst, mat3 mat, const vec3 vec);
void Mat4VecMul(vec4 dst, mat4 mat, const vec4 vec);

void Mat2Add(mat2 dst, mat2 a, mat2 b);
void Mat3Add(mat3 dst, mat3 a, mat3 b);
void Mat4Add(mat4 dst, mat4 a, mat4 b);

/* dst = a*b */
void Mat2Mul(mat2 dst, mat2 a, mat2 b);
void Mat3Mul(mat3 dst, mat3 a, mat3 b);
void Mat4Mul(mat4 dst, mat4 a, mat4 b);

/* dst = src^T */
void Mat2Transpose(mat2 dst, mat2 src);
void Mat3Transpose(mat3 dst, mat3 src);
void Mat4Transpose(mat4 dst, mat4 src);

/* Returns determinant of src, and sets inverse, or garbage if it does not exist. */
f32 Mat2Inverse(mat2 dst, mat2 src);
f32 Mat3Inverse(mat3 dst, mat3 src);
f32 Mat4Inverse(mat4 dst, mat4 src);

/* min |a_ij| */
f32 Mat2AbsMin(mat2 src);
f32 Mat3AbsMin(mat3 src);
f32 Mat4AbsMin(mat4 src);

/* max |a_ij| */
f32 Mat2AbsMax(mat2 src);
f32 Mat3AbsMax(mat3 src);
f32 Mat4AbsMax(mat4 src);

void Mat2Copy(mat2 dst, mat2 src);
void Mat3Copy(mat3 dst, mat3 src);
void Mat4Copy(mat4 dst, mat4 src);

#ifdef __cplusplus
} 
#endif

#endif


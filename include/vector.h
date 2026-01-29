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

#ifndef __DS_VECTOR_MATH__
#define __DS_VECTOR_MATH__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_common.h"

#define vec2_inline(a,b) ((vec2) { a, b })
#define vec3_inline(a,b,c) ((vec3) { a, b, c })
#define vec4_inline(a,b,c,d) ((vec4) { a, b, c, d })

#define vec2u32_inline(a,b) ((vec2u32) { a, b })
#define vec3u32_inline(a,b,c) ((vec2u32) { a, b, c })
#define vec4u32_inline(a,b,c,d) ((vec2u32) { a, b, c, d })

#define vec2u64_inline(a,b) ((vec2u64) { a, b })
#define vec3u64_inline(a,b,c) ((vec2u64) { a, b, c })
#define vec4u64_inline(a,b,c,d) ((vec2u64) { a, b, c, d })

#define vec2i32_inline(a,b) ((vec2i32) { a, b })
#define vec3i32_inline(a,b,c) ((vec2i32) { a, b, c })
#define vec4i32_inline(a,b,c,d) ((vec2i32) { a, b, c, d })

#define vec2i64_inline(a,b) ((vec2i64) { a, b })
#define vec3i64_inline(a,b,c) ((vec2i64) { a, b, c })
#define vec4i64_inline(a,b,c,d) ((vec2i64) { a, b, c, d })

void vec2_print(const char *text, const vec2 v);
void vec3_print(const char *text, const vec3 v);
void vec4_print(const char *text, const vec4 v);

void vec2u32_print(const char *text, const vec2u32 v);
void vec3u32_print(const char *text, const vec3u32 v);
void vec4u32_print(const char *text, const vec4u32 v);

void vec2u64_print(const char *text, const vec2u64 v);
void vec3u64_print(const char *text, const vec3u64 v);
void vec4u64_print(const char *text, const vec4u64 v);

void vec2i32_print(const char *text, const vec2i32 v);
void vec3i32_print(const char *text, const vec3i32 v);
void vec4i32_print(const char *text, const vec4i32 v);

void vec2i64_print(const char *text, const vec2i64 v);
void vec3i64_print(const char *text, const vec3i64 v);
void vec4i64_print(const char *text, const vec4i64 v);


#define VEC2_ZERO { 0.0f, 0.0f }
#define VEC3_ZERO { 0.0f, 0.0f, 0.0f }
#define VEC4_ZERO { 0.0f, 0.0f, 0.0f, 0.0f }
#define VEC2U_ZERO { 0, 0 }
#define VEC3U_ZERO { 0, 0, 0 }
#define VEC4U_ZERO { 0, 0, 0, 0 }

void vec2_print(const char *text, const vec2 v);
void vec3_print(const char *text, const vec3 v);
void vec4_print(const char *text, const vec4 v);

void vec2u32_print(const char *text, const vec2u32 v);
void vec3u32_print(const char *text, const vec3u32 v);
void vec4u32_print(const char *text, const vec4u32 v);

void vec2u64_print(const char *text, const vec2u64 v);
void vec3u64_print(const char *text, const vec3u64 v);
void vec4u64_print(const char *text, const vec4u64 v);

void vec2i32_print(const char *text, const vec2i32 v);
void vec3i32_print(const char *text, const vec3i32 v);
void vec4i32_print(const char *text, const vec4i32 v);

void vec2i64_print(const char *text, const vec2i64 v);
void vec3i64_print(const char *text, const vec3i64 v);
void vec4i64_print(const char *text, const vec4i64 v);

void vec2u32_set(vec2u32 dst, const u32 x, const u32 y);
void vec2u64_set(vec2u64 dst, const u64 x, const u64 y);
void vec2i32_set(vec2i32 dst, const i32 x, const i32 y);
void vec2i64_set(vec2i64 dst, const i64 x, const i64 y);

void vec3u32_set(vec3u32 dst, const u32 x, const u32 y, const u32 z);
void vec3u64_set(vec3u64 dst, const u64 x, const u64 y, const u64 z);
void vec3i32_set(vec3i32 dst, const i32 x, const i32 y, const i32 z);
void vec3i64_set(vec3i64 dst, const i64 x, const i64 y, const i64 z);

void vec4u32_set(vec4u32 dst, const u32 x, const u32 y, const u32 z, const u32 w);
void vec4u64_set(vec4u64 dst, const u64 x, const u64 y, const u64 z, const u64 w);
void vec4i32_set(vec4i32 dst, const i32 x, const i32 y, const i32 z, const i32 w);
void vec4i64_set(vec4i64 dst, const i64 x, const i64 y, const i64 z, const i64 w);
                                        
void vec2u32_copy(vec2u32 dst, const vec2u32 src); 
void vec2u64_copy(vec2u64 dst, const vec2u64 src); 
void vec2i32_copy(vec2i32 dst, const vec2i32 src); 
void vec2i64_copy(vec2i64 dst, const vec2i64 src); 
                                       
void vec3u32_copy(vec3u32 dst, const vec3u32 src); 
void vec3u64_copy(vec3u64 dst, const vec3u64 src); 
void vec3i32_copy(vec3i32 dst, const vec3i32 src); 
void vec3i64_copy(vec3i64 dst, const vec3i64 src); 
                                      
void vec4u32_copy(vec4u32 dst, const vec4u32 src); 
void vec4u64_copy(vec4u64 dst, const vec4u64 src); 
void vec4i32_copy(vec4i32 dst, const vec4i32 src); 
void vec4i64_copy(vec4i64 dst, const vec4i64 src); 

void vec2_mix(vec2 a, const vec2 b);	/* interpolate (alpha = 0.5f);*/
void vec3_mix(vec3 a, const vec3 b);	/* interpolate (alpha = 0.5f);*/
void vec4_mix(vec4 a, const vec4 b);	/* interpolate (alpha = 0.5f);*/

void vec2_translate_scaled(vec2 dst, const vec2 to_scale, const f32 scale); 
void vec3_translate_scaled(vec3 dst, const vec3 to_scale, const f32 scale);
void vec4_translate_scaled(vec4 dst, const vec4 to_scale, const f32 scale);

void vec2_negative(vec2 v);
void vec3_negative(vec3 v);
void vec4_negative(vec4 v);

void vec2_negative_to(vec2 dst, const vec2 src);
void vec3_negative_to(vec3 dst, const vec3 src);
void vec4_negative_to(vec4 dst, const vec4 src);

void vec2_abs(vec2 v);
void vec3_abs(vec3 v);
void vec4_abs(vec4 v);

void vec2_abs_to(vec2 dst, const vec2 src);
void vec3_abs_to(vec3 dst, const vec3 src);
void vec4_abs_to(vec4 dst, const vec4 src);

f32 vec2_distance(const vec2 a, const vec2 b); 
f32 vec3_distance(const vec3 a, const vec3 b);
f32 vec4_distance(const vec4 a, const vec4 b);

f32 vec2_distance_squared(const vec2 a, const vec2 b); 
f32 vec3_distance_squared(const vec3 a, const vec3 b);
f32 vec4_distance_squared(const vec4 a, const vec4 b);

f32 vec2_length_squared(const vec2 a);
f32 vec3_length_squared(const vec3 a);
f32 vec4_length_squared(const vec4 a);

void vec2_set(vec2 dst, const f32 x, const f32 y);
void vec2_copy(vec2 dst, const vec2 src);
void vec2_add(vec2 dst, const vec2 a, const vec2 b);
void vec2_sub(vec2 dst, const vec2 a, const vec2 b); /* a - b */
void vec2_mul(vec2 dst, const vec2 a, const vec2 b);
void vec2_div(vec2 dst, const vec2 a, const vec2 b); /* a / b */
void vec2_scale(vec2 dst, const vec2 src, const f32 scale);
f32 vec2_length(const vec2 a);
void vec2_normalize(vec2 dst, const vec2 a);
void vec2_translate(vec2 dst, const vec2 translation);
void vec2_add_constant(vec2 dst, const f32 c);
void vec2_mul_constant(vec2 dst, const f32 c);
f32 vec2_dot(const vec2 a, const vec2 b);
void vec2_interpolate(vec2 dst, const vec2 a, const vec2 b, const f32 alpha);
void vec2_interpolate_piecewise(vec2 dst, const vec2 a, const vec2 b, const vec2 alpha);

void vec3_set(vec3 dst, const f32 x, const f32 y, const f32 z);
void vec3_copy(vec3 dst, const vec3 src);
void vec3_add(vec3 dst, const vec3 a, const vec3 b);
void vec3_sub(vec3 dst, const vec3 a, const vec3 b); /*	a - b */
void vec3_mul(vec3 dst, const vec3 a, const vec3 b);
void vec3_div(vec3 dst, const vec3 a, const vec3 b); /* a / b */
void vec3_scale(vec3 dst, const vec3 src, const f32 scale);
f32 vec3_length(const vec3 a);
void vec3_normalize(vec3 dst, const vec3 a);
void vec3_translate(vec3 dst, const vec3 translation);
void vec3_add_constant(vec3 dst, const f32 c);
void vec3_mul_constant(vec3 dst, const f32 c);
f32 vec3_dot(const vec3 a, const vec3 b);
void vec3_cross(vec3 dst, const vec3 a, const vec3 b); /* a cross b */
void vec3_rotate_y(vec3 dst, const vec3 a, const f32 angle);
void vec3_interpolate(vec3 dst, const vec3 a, const vec3 b, const f32 alpha);
void vec3_interpolate_piecewise(vec3 dst, const vec3 a, const vec3 b, const vec3 alpha);
/* (a x b) x c */
void vec3_triple_product(vec3 dst, const vec3 a, const vec3 b, const vec3 c);
void vec3_create_basis_from_normal(vec3 n1, vec3 n2, const vec3 n3);

/* (a-center) x (b-center) */
void vec3_recenter_cross(vec3 dst, const vec3 center, const vec3 a, const vec3 b);

void vec4_set(vec4 dst, const f32 x, const f32 y, const f32 z, const f32 w);
void vec4_copy(vec4 dst, const vec4 src);
void vec4_add(vec4 dst, const vec4 a, const vec4 b);
void vec4_sub(vec4 dst, const vec4 a, const vec4 b); /*	a - b */
void vec4_mul(vec4 dst, const vec4 a, const vec4 b);
void vec4_div(vec4 dst, const vec4 a, const vec4 b); /* a / b */
void vec4_scale(vec4 dst, const vec4 src, const f32 scale);
f32 vec4_length(const vec4 a);
void vec4_normalize(vec4 dst, const vec4 a);
void vec4_translate(vec4 dst, const vec4 translation);
void vec4_add_constant(vec4 dst, const f32 c);
void vec4_mul_constant(vec4 dst, const f32 c);
f32 vec4_dot(const vec4 a, const vec4 b);
void vec4_interpolate(vec4 dst, const vec4 a, const vec4 b, const f32 alpha);	/* a * alpha + b * (1-alpha) */
void vec4_interpolate_piecewise(vec4 dst, const vec4 a, const vec4 b, const vec4 alpha);

#ifdef __cplusplus
} 
#endif

#endif

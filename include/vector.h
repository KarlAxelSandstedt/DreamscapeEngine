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

#include "ds_types.h"

#define VEC2_ZERO { 0.0f, 0.0f }
#define VEC3_ZERO { 0.0f, 0.0f, 0.0f }
#define VEC4_ZERO { 0.0f, 0.0f, 0.0f, 0.0f }
#define VEC2U_ZERO { 0, 0 }
#define VEC3U_ZERO { 0, 0, 0 }
#define VEC4U_ZERO { 0, 0, 0, 0 }

#define Vec2Inline(a,b) ((vec2) { a, b })
#define Vec3Inline(a,b,c) ((vec3) { a, b, c })
#define Vec4Inline(a,b,c,d) ((vec4) { a, b, c, d })

#define Vec2U32Inline(a,b) ((vec2u32) { a, b })
#define Vec3U32Inline(a,b,c) ((vec2u32) { a, b, c })
#define Vec4U32Inline(a,b,c,d) ((vec2u32) { a, b, c, d })

#define Vec2U64Inline(a,b) ((vec2u64) { a, b })
#define Vec3U64Inline(a,b,c) ((vec2u64) { a, b, c })
#define Vec4U64Inline(a,b,c,d) ((vec2u64) { a, b, c, d })

#define Vec2I32Inline(a,b) ((vec2i32) { a, b })
#define Vec3I32Inline(a,b,c) ((vec2i32) { a, b, c })
#define Vec4I32Inline(a,b,c,d) ((vec2i32) { a, b, c, d })

#define Vec2I64Inline(a,b) ((vec2i64) { a, b })
#define Vec3I64Inline(a,b,c) ((vec2i64) { a, b, c })
#define Vec4I64Inline(a,b,c,d) ((vec2i64) { a, b, c, d })

void 	Vec2Print(const char *text, const vec2 v);
void 	Vec3Print(const char *text, const vec3 v);
void 	Vec4Print(const char *text, const vec4 v);

void 	Vec2U32Print(const char *text, const vec2u32 v);
void 	Vec3U32Print(const char *text, const vec3u32 v);
void 	Vec4U32Print(const char *text, const vec4u32 v);

void 	Vec2U64Print(const char *text, const vec2u64 v);
void 	Vec3U64Print(const char *text, const vec3u64 v);
void 	Vec4U64Print(const char *text, const vec4u64 v);

void 	Vec2I32Print(const char *text, const vec2i32 v);
void 	Vec3I32Print(const char *text, const vec3i32 v);
void 	Vec4I32Print(const char *text, const vec4i32 v);

void 	Vec2I64Print(const char *text, const vec2i64 v);
void 	Vec3I64Print(const char *text, const vec3i64 v);
void 	Vec4I64Print(const char *text, const vec4i64 v);

void 	Vec2Print(const char *text, const vec2 v);
void 	Vec3Print(const char *text, const vec3 v);
void 	Vec4Print(const char *text, const vec4 v);

void 	Vec2U32Print(const char *text, const vec2u32 v);
void 	Vec3U32Print(const char *text, const vec3u32 v);
void 	Vec4U32Print(const char *text, const vec4u32 v);

void 	Vec2U64Print(const char *text, const vec2u64 v);
void 	Vec3U64Print(const char *text, const vec3u64 v);
void 	Vec4U64Print(const char *text, const vec4u64 v);

void 	Vec2I32Print(const char *text, const vec2i32 v);
void 	Vec3I32Print(const char *text, const vec3i32 v);
void 	Vec4I32Print(const char *text, const vec4i32 v);

void 	Vec2I64Print(const char *text, const vec2i64 v);
void 	Vec3I64Print(const char *text, const vec3i64 v);
void 	Vec4I64Print(const char *text, const vec4i64 v);

void 	Vec2U32Set(vec2u32 dst, const u32 x, const u32 y);
void 	Vec2U64Set(vec2u64 dst, const u64 x, const u64 y);
void 	Vec2I32Set(vec2i32 dst, const i32 x, const i32 y);
void 	Vec2I64Set(vec2i64 dst, const i64 x, const i64 y);

void 	Vec3U32Set(vec3u32 dst, const u32 x, const u32 y, const u32 z);
void 	Vec3U64Set(vec3u64 dst, const u64 x, const u64 y, const u64 z);
void 	Vec3I32Set(vec3i32 dst, const i32 x, const i32 y, const i32 z);
void 	Vec3I64Set(vec3i64 dst, const i64 x, const i64 y, const i64 z);

void 	Vec4U32Set(vec4u32 dst, const u32 x, const u32 y, const u32 z, const u32 w);
void 	Vec4U64Set(vec4u64 dst, const u64 x, const u64 y, const u64 z, const u64 w);
void 	Vec4I32Set(vec4i32 dst, const i32 x, const i32 y, const i32 z, const i32 w);
void 	Vec4I64Set(vec4i64 dst, const i64 x, const i64 y, const i64 z, const i64 w);
     	                                   
void 	Vec2U32Copy(vec2u32 dst, const vec2u32 src); 
void 	Vec2U64Copy(vec2u64 dst, const vec2u64 src); 
void 	Vec2I32Copy(vec2i32 dst, const vec2i32 src); 
void 	Vec2I64Copy(vec2i64 dst, const vec2i64 src); 
     	                                  
void 	Vec3U32Copy(vec3u32 dst, const vec3u32 src); 
void 	Vec3U64Copy(vec3u64 dst, const vec3u64 src); 
void 	Vec3I32Copy(vec3i32 dst, const vec3i32 src); 
void 	Vec3I64Copy(vec3i64 dst, const vec3i64 src); 
     	                                 
void 	Vec4U32Copy(vec4u32 dst, const vec4u32 src); 
void 	Vec4U64Copy(vec4u64 dst, const vec4u64 src); 
void 	Vec4I32Copy(vec4i32 dst, const vec4i32 src); 
void 	Vec4I64Copy(vec4i64 dst, const vec4i64 src); 

void 	Vec2Mix(vec2 a, const vec2 b);	/* interpolate (alpha = 0.5f);*/
void 	Vec3Mix(vec3 a, const vec3 b);	/* interpolate (alpha = 0.5f);*/
void 	Vec4Mix(vec4 a, const vec4 b);	/* interpolate (alpha = 0.5f);*/

void 	Vec2TranslateScaled(vec2 dst, const vec2 to_scale, const f32 scale); 
void 	Vec3TranslateScaled(vec3 dst, const vec3 to_scale, const f32 scale);
void 	Vec4TranslateScaled(vec4 dst, const vec4 to_scale, const f32 scale);

void 	Vec2NegateSelf(vec2 v);
void 	Vec3NegateSelf(vec3 v);
void 	Vec4NegateSelf(vec4 v);

void 	Vec2Negate(vec2 dst, const vec2 src);
void 	Vec3Negate(vec3 dst, const vec3 src);
void 	Vec4Negate(vec4 dst, const vec4 src);

void 	Vec2AbsSelf(vec2 v);
void 	Vec3AbsSelf(vec3 v);
void 	Vec4AbsSelf(vec4 v);

void 	Vec2Abs(vec2 dst, const vec2 src);
void 	Vec3Abs(vec3 dst, const vec3 src);
void 	Vec4Abs(vec4 dst, const vec4 src);

f32 	Vec2Distance(const vec2 a, const vec2 b); 
f32 	Vec3Distance(const vec3 a, const vec3 b);
f32 	Vec4Distance(const vec4 a, const vec4 b);

f32 	Vec2DistanceSquared(const vec2 a, const vec2 b); 
f32 	Vec3DistanceSquared(const vec3 a, const vec3 b);
f32 	Vec4DistanceSquared(const vec4 a, const vec4 b);

f32 	Vec2LengthSquared(const vec2 a);
f32 	Vec3LengthSquared(const vec3 a);
f32 	Vec4LengthSquared(const vec4 a);

void 	Vec2Set(vec2 dst, const f32 x, const f32 y);
void 	Vec3Set(vec3 dst, const f32 x, const f32 y, const f32 z);
void 	Vec4Set(vec4 dst, const f32 x, const f32 y, const f32 z, const f32 w);

void 	Vec2Copy(vec2 dst, const vec2 src);
void 	Vec3Copy(vec3 dst, const vec3 src);
void 	Vec4Copy(vec4 dst, const vec4 src);

void 	Vec2Add(vec2 dst, const vec2 a, const vec2 b);
void 	Vec3Add(vec3 dst, const vec3 a, const vec3 b);
void 	Vec4Add(vec4 dst, const vec4 a, const vec4 b);

void 	Vec2Sub(vec2 dst, const vec2 a, const vec2 b); /* a - b */
void 	Vec3Sub(vec3 dst, const vec3 a, const vec3 b); /*	a - b */
void 	Vec4Sub(vec4 dst, const vec4 a, const vec4 b); /*	a - b */

void 	Vec2Mul(vec2 dst, const vec2 a, const vec2 b);
void 	Vec3Mul(vec3 dst, const vec3 a, const vec3 b);
void 	Vec4Mul(vec4 dst, const vec4 a, const vec4 b);

void 	Vec2Div(vec2 dst, const vec2 a, const vec2 b); /* a / b */
void 	Vec3Div(vec3 dst, const vec3 a, const vec3 b); /* a / b */
void 	Vec4Div(vec4 dst, const vec4 a, const vec4 b); /* a / b */

void 	Vec2Scale(vec2 dst, const vec2 src, const f32 scale);
void 	Vec3Scale(vec3 dst, const vec3 src, const f32 scale);
void 	Vec4Scale(vec4 dst, const vec4 src, const f32 scale);

f32 	Vec2Length(const vec2 a);
f32 	Vec3Length(const vec3 a);
f32 	Vec4Length(const vec4 a);

void 	Vec2Normalize(vec2 dst, const vec2 a);
void 	Vec3Normalize(vec3 dst, const vec3 a);
void 	Vec4Normalize(vec4 dst, const vec4 a);

void 	Vec2Translate(vec2 dst, const vec2 translation);
void 	Vec3Translate(vec3 dst, const vec3 translation);
void 	Vec4Translate(vec4 dst, const vec4 translation);

void 	Vec2AddConstant(vec2 dst, const f32 c);
void 	Vec3AddConstant(vec3 dst, const f32 c);
void 	Vec4AddConstant(vec4 dst, const f32 c);

void 	Vec2ScaleSelf(vec2 dst, const f32 c);
void 	Vec3ScaleSelf(vec3 dst, const f32 c);
void 	Vec4ScaleSelf(vec4 dst, const f32 c);

f32 	Vec2Dot(const vec2 a, const vec2 b);
f32 	Vec3Dot(const vec3 a, const vec3 b);
f32 	Vec4Dot(const vec4 a, const vec4 b);

void 	Vec2Interpolate(vec2 dst, const vec2 a, const vec2 b, const f32 alpha);
void 	Vec3Interpolate(vec3 dst, const vec3 a, const vec3 b, const f32 alpha);
void 	Vec4Interpolate(vec4 dst, const vec4 a, const vec4 b, const f32 alpha);	/* a * alpha + b * (1-alpha) */

void 	Vec2InterpolatePiecewise(vec2 dst, const vec2 a, const vec2 b, const vec2 alpha);
void 	Vec3InterpolatePiecewise(vec3 dst, const vec3 a, const vec3 b, const vec3 alpha);
void 	Vec4InterpolatePiecewise(vec4 dst, const vec4 a, const vec4 b, const vec4 alpha);

void 	Vec3Cross(vec3 dst, const vec3 a, const vec3 b); 		/* a cross b */
void 	Vec3RotateY(vec3 dst, const vec3 a, const f32 angle);	/* (a x b) x c */
void 	Vec3TripleProduct(vec3 dst, const vec3 a, const vec3 b, const vec3 c);
/* n3 Must be normalized!*/
void 	Vec3CreateBasis(vec3 n1, vec3 n2, const vec3 n3);
/* (a-center) x (b-center) */
void 	Vec3RecenterCross(vec3 dst, const vec3 center, const vec3 a, const vec3 b);

#ifdef __cplusplus
} 
#endif

#endif

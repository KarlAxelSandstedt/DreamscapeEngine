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

#include <stdio.h>
#include <float.h>

#include "ds_base.h"
#include "float32.h"
#include "matrix.h"
#include "vector.h"

void Vec2Print(const char *text, const vec2 v)
{
	fprintf(stderr, "%s: (%f, %f), \n", text, v[0], v[1]);
}

void Vec3Print(const char *text, const vec3 v)
{
	fprintf(stderr, "%s: (%f, %f, %f), \n", text, v[0], v[1], v[2]);
}

void Vec4Print(const char *text, const vec4 v)
{
	fprintf(stderr, "%s: (%f, %f, %f, %f), \n", text, v[0], v[1], v[2], v[3]);
}

void Vec2u32Print(const char *text, const vec2u32 v)
{
	fprintf(stderr, "%s: (%u, %u), \n", text, v[0], v[1]);
}

void Vec3u32Print(const char *text, const vec3u32 v)
{
	fprintf(stderr, "%s: (%u, %u, %u), \n", text, v[0], v[1], v[2]);
}

void Vec4u32Print(const char *text, const vec4u32 v)
{
	fprintf(stderr, "%s: (%u, %u, %u, %u), \n", text, v[0], v[1], v[2], v[3]);
}

void Vec2i32Print(const char *text, const vec2i32 v)
{
	fprintf(stderr, "%s: (%i, %i), \n", text, v[0], v[1]);
}

void Vec3i32Print(const char *text, const vec3i32 v)
{
	fprintf(stderr, "%s: (%i, %i, %i), \n", text, v[0], v[1], v[2]);
}

void Vec4i32Print(const char *text, const vec4i32 v)
{
	fprintf(stderr, "%s: (%i, %i, %i, %i), \n", text, v[0], v[1], v[2], v[3]);
}

#if (__DS_COMPILER__ == __DS_GCC__)
void Vec2i64Print(const char *text, const vec2i64 v)
{
	fprintf(stderr, "%s: (%li, %li), \n", text, v[0], v[1]);
}

void Vec3i64Print(const char *text, const vec3i64 v)
{
	fprintf(stderr, "%s: (%li, %li, %li), \n", text, v[0], v[1], v[2]);
}

void Vec4i64Print(const char *text, const vec4i64 v)
{
	fprintf(stderr, "%s: (%li, %li, %li, %li), \n", text, v[0], v[1], v[2], v[3]);
}

void Vec2u64Print(const char *text, const vec2u64 v)
{
	fprintf(stderr, "%s: (%lu, %lu), \n", text, v[0], v[1]);
}

void Vec3u64Print(const char *text, const vec3u64 v)
{
	fprintf(stderr, "%s: (%lu, %lu, %lu), \n", text, v[0], v[1], v[2]);
}

void Vec4u64Print(const char *text, const vec4u64 v)
{
	fprintf(stderr, "%s: (%lu, %lu, %lu, %lu), \n", text, v[0], v[1], v[2], v[3]);
}
#elif (__DS_COMPILER__ == __DS_MSVC__ || __DS_COMPILER__ == _DS_CLANG__ || __DS_PLATFORM__ == __DS_WEB__)
void Vec2i64Print(const char *text, const vec2i64 v)
{
	fprintf(stderr, "%s: (%lli, %lli), \n", text, v[0], v[1]);
}

void Vec3i64Print(const char *text, const vec3i64 v)
{
	fprintf(stderr, "%s: (%lli, %lli, %lli), \n", text, v[0], v[1], v[2]);
}

void Vec4i64Print(const char *text, const vec4i64 v)
{
	fprintf(stderr, "%s: (%lli, %lli, %lli, %lli), \n", text, v[0], v[1], v[2], v[3]);
}

void Vec2u64Print(const char *text, const vec2u64 v)
{
	fprintf(stderr, "%s: (%llu, %llu), \n", text, v[0], v[1]);
}

void Vec3u64Print(const char *text, const vec3u64 v)
{
	fprintf(stderr, "%s: (%llu, %llu, %llu), \n", text, v[0], v[1], v[2]);
}

void Vec4u64Print(const char *text, const vec4u64 v)
{
	fprintf(stderr, "%s: (%llu, %llu, %llu, %llu), \n", text, v[0], v[1], v[2], v[3]);
}

#endif

void Vec2u32Set(vec2u32 dst, const u32 x, const u32 y)
{
	dst[0] = x;
	dst[1] = y;
}

void Vec2u64Set(vec2u64 dst, const u64 x, const u64 y)
{
	dst[0] = x;
	dst[1] = y;
}

void Vec2i32Set(vec2i32 dst, const i32 x, const i32 y)
{
	dst[0] = x;
	dst[1] = y;
}
void Vec2i64Set(vec2i64 dst, const i64 x, const i64 y)
{
	dst[0] = x;
	dst[1] = y;
}

void Vec3u32Set(vec3u32 dst, const u32 x, const u32 y, const u32 z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

void Vec3u64Set(vec3u64 dst, const u64 x, const u64 y, const u64 z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

void Vec3i32Set(vec3i32 dst, const i32 x, const i32 y, const i32 z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

void Vec3i64Set(vec3i64 dst, const i64 x, const i64 y, const i64 z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

void Vec4u32Set(vec4u32 dst, const u32 x, const u32 y, const u32 z, const u32 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}

void Vec4u64Set(vec4u64 dst, const u64 x, const u64 y, const u64 z, const u64 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}

void Vec4i32Set(vec4i32 dst, const i32 x, const i32 y, const i32 z, const i32 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}

void Vec4i64Set(vec4i64 dst, const i64 x, const i64 y, const i64 z, const i64 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}                                       

void Vec2u32Copy(vec2u32 dst, const vec2u32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}

void Vec2u64Copy(vec2u64 dst, const vec2u64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}

void Vec2i32Copy(vec2i32 dst, const vec2i32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}

void Vec2i64Copy(vec2i64 dst, const vec2i64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}
                                       
void Vec3u32Copy(vec3u32 dst, const vec3u32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

void Vec3u64Copy(vec3u64 dst, const vec3u64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

void Vec3i32Copy(vec3i32 dst, const vec3i32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

void Vec3i64Copy(vec3i64 dst, const vec3i64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}
                                      
void Vec4u32Copy(vec4u32 dst, const vec4u32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void Vec4u64Copy(vec4u64 dst, const vec4u64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void Vec4i32Copy(vec4i32 dst, const vec4i32 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void Vec4i64Copy(vec4i64 dst, const vec4i64 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void Vec2NegateSelf(vec2 v)
{
	v[0] = -v[0];
	v[1] = -v[1];
}

void Vec3NegateSelf(vec3 v)
{
	v[0] = -v[0];
	v[1] = -v[1];
	v[2] = -v[2];
}

void Vec4NegateSelf(vec4 v)
{
	v[0] = -v[0];
	v[1] = -v[1];
	v[2] = -v[2];
	v[3] = -v[3];
}

void Vec2AddConstant(vec2 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
}

void Vec3AddConstant(vec3 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
	dst[2] += c;
}

void Vec4AddConstant(vec4 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
	dst[2] += c;
	dst[3] += c;
}

void Vec2Scale(vec2 dst, const vec2 src, const f32 scale)
{
	dst[0] = scale * src[0];
	dst[1] = scale * src[1];
}

void Vec3Scale(vec3 dst, const vec3 src, const f32 scale)
{
	dst[0] = scale * src[0];
	dst[1] = scale * src[1];
	dst[2] = scale * src[2];
}

void Vec4Scale(vec4 dst, const vec4 src, const f32 scale)
{
	dst[0] = scale * src[0];
	dst[1] = scale * src[1];
	dst[2] = scale * src[2];
	dst[3] = scale * src[3];
}

void Vec2TranslateScaled(vec2 dst, const vec2 to_scale, const f32 scale)
{
	dst[0] += scale * to_scale[0];
	dst[1] += scale * to_scale[1];
}              
               
void Vec3TranslateScaled(vec3 dst, const vec3 to_scale, const f32 scale)
{
	dst[0] += scale * to_scale[0];
	dst[1] += scale * to_scale[1];
	dst[2] += scale * to_scale[2];
}              

void Vec4TranslateScaled(vec4 dst, const vec4 to_scale, const f32 scale)
{
	dst[0] += scale * to_scale[0];
	dst[1] += scale * to_scale[1];
	dst[2] += scale * to_scale[2];
	dst[3] += scale * to_scale[3];
}

f32 Vec2Distance(const vec2 a, const vec2 b)
{
	f32 sum = (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]);
	return f32_sqrt(sum);
}

f32 Vec3Distance(const vec3 a, const vec3 b)
{
	f32 sum = (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) + (b[2]-a[2])*(b[2]-a[2]);
	return f32_sqrt(sum);
}

f32 Vec4Distance(const vec4 a, const vec4 b)
{
	f32 sum = (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) + (b[2]-a[2])*(b[2]-a[2]) + (b[3]-a[3])*(b[3]-a[3]);
	return f32_sqrt(sum);
}

f32 Vec2DistanceSquared(const vec2 a, const vec2 b)
{
	return (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]);
}

f32 Vec3DistanceSquared(const vec3 a, const vec3 b)
{
	return (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) + (b[2]-a[2])*(b[2]-a[2]);
}

f32 Vec4DistanceSquared(const vec4 a, const vec4 b)
{
	return (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1]) + (b[2]-a[2])*(b[2]-a[2]) + (b[3]-a[3])*(b[3]-a[3]);
}



void Vec2Set(vec2 dst, const f32 x, const f32 y)
{
	dst[0] = x;
	dst[1] = y;
}

void Vec2Copy(vec2 dst, const vec2 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}

void Vec2Add(vec2 dst, const vec2 a, const vec2 b)
{
	dst[0] = a[0] + b[0];
	dst[1] = a[1] + b[1];
}

void Vec2Sub(vec2 dst, const vec2 a, const vec2 b)
{
	dst[0] = a[0] - b[0];
	dst[1] = a[1] - b[1];
}

void Vec2Mul(vec2 dst, const vec2 a, const vec2 b)
{
	dst[0] = a[0] * b[0];
	dst[1] = a[1] * b[1];
}

void Vec2Div(vec2 dst, const vec2 a, const vec2 b)
{
	ds_Assert(b[0] != 0.0f && b[1] != 0.0f);

	dst[0] = a[0] / b[0];
	dst[1] = a[1] / b[1];
}

f32 Vec2Length(const vec2 a)
{
	f32 sum = a[0]*a[0] + a[1]*a[1];
	return f32_sqrt(sum);
}

f32 Vec2LengthSquared(const vec2 a)
{
	return a[0]*a[0] + a[1]*a[1];
}

//TODO one mul operation instead of two!
void Vec2Normalize(vec2 dst, const vec2 a)
{
	f32 length = Vec2Length(a);
	Vec2Copy(dst, a);
	Vec2ScaleSelf(dst, 1/length);
}

void Vec2Translate(vec2 dst, const vec2 translation)
{
	dst[0] += translation[0];
	dst[1] += translation[1];
}

void Vec2Add_const(vec2 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
}

void Vec2ScaleSelf(vec2 dst, const f32 c)
{
	dst[0] *= c;
	dst[1] *= c;
}


f32 Vec2Dot(const vec2 a, const vec2 b)
{
	return a[0] * b[0] + a[1] * b[1];
}

void Vec2Interpolate(vec2 dst, const vec2 a, const vec2 b, const f32 alpha)
{
	dst[0] = a[0] * alpha + b[0] * (1.0f - alpha);
	dst[1] = a[1] * alpha + b[1] * (1.0f - alpha);
}

void Vec2InterpolatePiecewise(vec2 dst, const vec2 a, const vec2 b, const vec2 alpha)
{
	dst[0] = a[0] * alpha[0] + b[0] * (1.0f - alpha[0]);
	dst[1] = a[1] * alpha[1] + b[1] * (1.0f - alpha[1]);
}

void Vec3Set(vec3 dst, const f32 x, const f32 y, const f32 z)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

void Vec3Copy(vec3 dst, const vec3 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

void Vec3Add(vec3 dst, const vec3 a, const vec3 b)
{
	dst[0] = a[0] + b[0];
	dst[1] = a[1] + b[1];
	dst[2] = a[2] + b[2];
}

void Vec3Sub(vec3 dst, const vec3 a, const vec3 b)
{
	dst[0] = a[0] - b[0];
	dst[1] = a[1] - b[1];
	dst[2] = a[2] - b[2];
}

void Vec3Mul(vec3 dst, const vec3 a, const vec3 b)
{
	dst[0] = a[0] * b[0];
	dst[1] = a[1] * b[1];
	dst[2] = a[2] * b[2];
}

void Vec3Div(vec3 dst, const vec3 a, const vec3 b)
{
	ds_Assert(b[0] != 0.0f && b[1] != 0.0f && b[2] != 0.0f);

	dst[0] = a[0] / b[0];
	dst[1] = a[1] / b[1];
	dst[2] = a[2] / b[2];
}

f32 Vec3Length(const vec3 a)
{
	f32 sum = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
	return f32_sqrt(sum);
}

f32 Vec3LengthSquared(const vec3 a)
{
	return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}

//TODO one mul operation instead of two!
void Vec3Normalize(vec3 dst, const vec3 a)
{
	f32 length = Vec3Length(a);
	Vec3Copy(dst, a);
	Vec3ScaleSelf(dst, 1/length);
}

void Vec3Translate(vec3 dst, const vec3 translation)
{
	dst[0] += translation[0];
	dst[1] += translation[1];
	dst[2] += translation[2];
}

void Vec3ScaleSelf(vec3 dst, const f32 c)
{
	dst[0] *= c;
	dst[1] *= c;
	dst[2] *= c;
}

void Vec3Add_const(vec3 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
	dst[2] += c;
}

f32 Vec3Dot(const vec3 a, const vec3 b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Vec3Cross(vec3 dst, const vec3 a, const vec3 b)
{
	dst[0] = a[1] * b[2] - a[2] * b[1];
	dst[1] = a[2] * b[0] - a[0] * b[2];
	dst[2] = a[0] * b[1] - a[1] * b[0];
}

void Vec3RecenterCross(vec3 dst, const vec3 center, const vec3 a, const vec3 b)
{
	vec3 a_c, b_c;
	Vec3Sub(a_c, a, center);
	Vec3Sub(b_c, b, center);
	Vec3Cross(dst, a_c, b_c);
}
/* CCW */
void Vec3RotateY(vec3 dst, const vec3 a, const f32 angle)
{
	mat3 rot;
       	mat3_set(rot, f32_cos(angle), 0.0f, f32_sin(angle),
			    0.0f, 1.0f, 0.0f, 
			    -f32_sin(angle), 0.0f,  f32_cos(angle));
	vec3_mat_mul(dst, a, rot);	
}

void Vec3Interpolate(vec3 dst, const vec3 a, const vec3 b, const f32 alpha)
{
	dst[0] = a[0] * alpha + b[0] * (1.0f - alpha);
	dst[1] = a[1] * alpha + b[1] * (1.0f - alpha);
	dst[2] = a[2] * alpha + b[2] * (1.0f - alpha);
}

void Vec3InterpolatePiecewise(vec3 dst, const vec3 a, const vec3 b, const vec3 alpha)
{
	dst[0] = a[0] * alpha[0] + b[0] * (1.0f - alpha[0]);
	dst[1] = a[1] * alpha[1] + b[1] * (1.0f - alpha[1]);
	dst[2] = a[2] * alpha[2] + b[2] * (1.0f - alpha[2]);
}

void Vec3TripleProduct(vec3 dst, const vec3 a, const vec3 b, const vec3 c)
{
	vec3 tmp;
	Vec3Cross(tmp, a, b);
	Vec3Cross(dst, tmp, c);
}

void Vec4Set(vec4 dst, const f32 x, const f32 y, const f32 z, const f32 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}

void Vec4Copy(vec4 dst, const vec4 src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void Vec4Add(vec4 dst, const vec4 a, const vec4 b)
{
	dst[0] = a[0] + b[0];
	dst[1] = a[1] + b[1];
	dst[2] = a[2] + b[2];
	dst[3] = a[3] + b[3];
}

void Vec4Sub(vec4 dst, const vec4 a, const vec4 b)
{
	dst[0] = a[0] - b[0];
	dst[1] = a[1] - b[1];
	dst[2] = a[2] - b[2];
	dst[3] = a[3] - b[3];
}

void Vec4Mul(vec4 dst, const vec4 a, const vec4 b)
{
	dst[0] = a[0] * b[0];
	dst[1] = a[1] * b[1];
	dst[2] = a[2] * b[2];
	dst[3] = a[3] * b[3];
}

void Vec4Div(vec4 dst, const vec4 a, const vec4 b)
{
	ds_Assert(b[0] != 0.0f && b[1] != 0.0f && b[2] != 0.0f && b[3] != 0.0f);

	dst[0] = a[0] / b[0];
	dst[1] = a[1] / b[1];
	dst[2] = a[2] / b[2];
	dst[3] = a[3] / b[3];
}

f32 Vec4Length(const vec4 a)
{
	f32 sum = a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3];
	return f32_sqrt(sum);
}

f32 Vec4LengthSquared(const vec4 a)
{
	return a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3];
}

//TODO one mul operation instead of two!
void Vec4Normalize(vec4 dst, const vec4 a)
{
	f32 length = Vec4Length(a);
	Vec4Copy(dst, a);
	Vec4ScaleSelf(dst, 1/length);
}

void Vec4Translate(vec4 dst, const vec4 translation)
{
	dst[0] += translation[0];
	dst[1] += translation[1];
	dst[2] += translation[2];
	dst[3] += translation[3];
}

void Vec4Add_const(vec4 dst, const f32 c)
{
	dst[0] += c;
	dst[1] += c;
	dst[2] += c;
	dst[3] += c;
}

void Vec4ScaleSelf(vec4 dst, const f32 c)
{
	dst[0] *= c;
	dst[1] *= c;
	dst[2] *= c;
	dst[3] *= c;
}

f32 Vec4Dot(const vec4 a, const vec4 b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

void Vec4Interpolate(vec4 dst, const vec4 a, const vec4 b, const f32 alpha)
{
	dst[0] = a[0] * alpha + b[0] * (1.0f - alpha);
	dst[1] = a[1] * alpha + b[1] * (1.0f - alpha);
	dst[2] = a[2] * alpha + b[2] * (1.0f - alpha);
	dst[3] = a[3] * alpha + b[3] * (1.0f - alpha);
}

void Vec4InterpolatePiecewise(vec4 dst, const vec4 a, const vec4 b, const vec4 alpha)
{
	dst[0] = a[0] * alpha[0] + b[0] * (1.0f - alpha[0]);
	dst[1] = a[1] * alpha[1] + b[1] * (1.0f - alpha[1]);
	dst[2] = a[2] * alpha[2] + b[2] * (1.0f - alpha[2]);
	dst[3] = a[3] * alpha[3] + b[3] * (1.0f - alpha[3]);
}

void Vec2Negate(vec2 dst, const vec2 src)
{
	dst[0] = -src[0];
	dst[1] = -src[1];
}

void Vec3Negate(vec3 dst, const vec3 src)
{
	dst[0] = -src[0];
	dst[1] = -src[1];
	dst[2] = -src[2];
}

void Vec4Negate(vec4 dst, const vec4 src)
{
	dst[0] = -src[0];
	dst[1] = -src[1];
	dst[2] = -src[2];
	dst[3] = -src[3];
}

void Vec2AbsSelf(vec2 v)
{
	v[0] = f32_abs(v[0]);
	v[1] = f32_abs(v[1]);
}

void Vec3AbsSelf(vec3 v)
{
	v[0] = f32_abs(v[0]);
	v[1] = f32_abs(v[1]);
	v[2] = f32_abs(v[2]);
}

void Vec4AbsSelf(vec4 v)
{
	v[0] = f32_abs(v[0]);
	v[1] = f32_abs(v[1]);
	v[2] = f32_abs(v[2]);
	v[3] = f32_abs(v[3]);
}

void Vec2Abs(vec2 dst, const vec2 src)
{
	dst[0] = f32_abs(src[0]);
	dst[1] = f32_abs(src[1]);
}

void Vec3Abs(vec3 dst, const vec3 src)
{
	dst[0] = f32_abs(src[0]);
	dst[1] = f32_abs(src[1]);
	dst[2] = f32_abs(src[2]);
}

void Vec4Abs(vec4 dst, const vec4 src)
{
	dst[0] = f32_abs(src[0]);
	dst[1] = f32_abs(src[1]);
	dst[2] = f32_abs(src[2]);
	dst[3] = f32_abs(src[3]);
}

void Vec2Mix(vec2 a, const vec2 b)
{
	a[0] = 0.5f * (a[0] + b[0]);
	a[1] = 0.5f * (a[1] + b[1]);
}

void Vec3Mix(vec3 a, const vec3 b)
{
	a[0] = 0.5f * (a[0] + b[0]);
	a[1] = 0.5f * (a[1] + b[1]);
	a[2] = 0.5f * (a[2] + b[2]);
}

void Vec4Mix(vec4 a, const vec4 b)
{
	a[0] = 0.5f * (a[0] + b[0]);
	a[1] = 0.5f * (a[1] + b[1]);
	a[2] = 0.5f * (a[2] + b[2]);
	a[3] = 0.5f * (a[3] + b[3]);
}

void Vec3CreateBasis(vec3 n1, vec3 n2, const vec3 n3)
{
	ds_Assert(1.0f - F32_EPSILON*10000.0f <= Vec3Length(n3) && Vec3Length(n3) <= 1.0f + F32_EPSILON*10000.0f);

	if (n3[0]*n3[0] < n3[1]*n3[1])
	{
		if (n3[0]*n3[0] < n3[2]*n3[2]) { Vec3Set(n2, 1.0f, 0.0f, 0.0f); }
		else { Vec3Set(n2, 0.0f, 0.0f, 1.0f); }
	}
	else
	{
		if (n3[1]*n3[1] < n3[2]*n3[2]) { Vec3Set(n2, 0.0f, 1.0f, 0.0f); }
		else { Vec3Set(n2, 0.0f, 0.0f, 1.0f); }
	}
		
	Vec3Cross(n1, n3, n2);
	Vec3ScaleSelf(n1, 1.0f / Vec3Length(n1));
	Vec3Cross(n2, n1, n3);
	Vec3ScaleSelf(n2, 1.0f / Vec3Length(n2));

	ds_Assert(1.0f - F32_EPSILON*10000.0f <= Vec3Length(n1) && Vec3Length(n1) <= 1.0f + F32_EPSILON*10000.0f);
	ds_Assert(1.0f - F32_EPSILON*10000.0f <= Vec3Length(n2) && Vec3Length(n2) <= 1.0f + F32_EPSILON*10000.0f);
	ds_Assert(f32_abs(Vec3Dot(n1, n2)) <= F32_EPSILON * 100.0f);
	ds_Assert(f32_abs(Vec3Dot(n1, n3)) <= F32_EPSILON * 100.0f);
	ds_Assert(f32_abs(Vec3Dot(n2, n3)) <= F32_EPSILON * 100.0f);
}

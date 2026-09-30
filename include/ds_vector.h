/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt

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

#ifndef __DS_VECTOR_H__
#define __DS_VECTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <inttypes.h>

#include "ds_define.h"
#include "ds_types.h"
#include "ds_base.h"
#include "ds_float.h"

static ds_ForceInline v2 V2Zero(void)
{
	return v2(0.0f, 0.0f);
}

static ds_ForceInline v3 V3Zero(void)
{
	return v3(0.0f, 0.0f, 0.0f);
}

static ds_ForceInline v4 V4Zero(void)
{
	return v4(0.0f, 0.0f, 0.0f, 0.0f);
}

static ds_ForceInline v2u32 V2U32Zero(void)
{
	return v2u32(0, 0);
}

static ds_ForceInline v3u32 V3U32Zero(void)
{
	return v3u32(0, 0, 0);
}

static ds_ForceInline v4u32 V4U32Zero(void)
{
	return v4u32(0, 0, 0, 0);
}

static ds_ForceInline v2u64 V2U64Zero(void)
{
	return v2u64(0, 0);
}

static ds_ForceInline v3u64 V3U64Zero(void)
{
	return v3u64(0, 0, 0);
}

static ds_ForceInline v4u64 V4U64Zero(void)
{
	return v4u64(0, 0, 0, 0);
}

static ds_ForceInline v2i32 V2I32Zero(void)
{
	return v2i32(0, 0);
}

static ds_ForceInline v3i32 V3I32Zero(void)
{
	return v3i32(0, 0, 0);
}

static ds_ForceInline v4i32 V4I32Zero(void)
{
	return v4i32(0, 0, 0, 0);
}

static ds_ForceInline v2i64 V2I64Zero(void)
{
	return v2i64(0, 0);
}

static ds_ForceInline v3i64 V3I64Zero(void)
{
	return v3i64(0, 0, 0);
}

static ds_ForceInline v4i64 V4I64Zero(void)
{
	return v4i64(0, 0, 0, 0);
}

static inline void V2Print(const char *text, const v2 v)
{
	fprintf(stderr, "%s: (%f, %f), \n", text, v.x, v.y);
}

static inline void V3Print(const char *text, const v3 v)
{
	fprintf(stderr, "%s: (%f, %f, %f), \n", text, v.x, v.y, v.z);
}

static inline void V4Print(const char *text, const v4 v)
{
	fprintf(stderr, "%s: (%f, %f, %f, %f), \n", text, v.x, v.y, v.z, v.w);
}

static inline void V2U32Print(const char *text, const v2u32 v)
{
	fprintf(stderr, "%s: (%" PRIu32 ", %" PRIu32 "), \n", text, v.x, v.y);
}

static inline void V3U32Print(const char *text, const v3u32 v)
{
	fprintf(stderr, "%s: (%" PRIu32 ", %" PRIu32 ", %" PRIu32 "), \n", text, v.x, v.y, v.z);
}

static inline void V4U32Print(const char *text, const v4u32 v)
{
	fprintf(stderr, "%s: (%" PRIu32 ", %" PRIu32 ", %" PRIu32 ", %" PRIu32 "), \n", text, v.x, v.y, v.z, v.w);
}

static inline void V2U64Print(const char *text, const v2u64 v)
{
	fprintf(stderr, "%s: (%" PRIu64 ", %" PRIu64 "), \n", text, v.x, v.y);
}

static inline void V3U64Print(const char *text, const v3u64 v)
{
	fprintf(stderr, "%s: (%" PRIu64 ", %" PRIu64 ", %" PRIu64 "), \n", text, v.x, v.y, v.z);
}

static inline void V4U64Print(const char *text, const v4u64 v)
{
	fprintf(stderr, "%s: (%" PRIu64 ", %" PRIu64 ", %" PRIu64 ", %" PRIu64 "), \n", text, v.x, v.y, v.z, v.w);
}

static inline void V2I32Print(const char *text, const v2i32 v)
{
	fprintf(stderr, "%s: (%" PRIi32 ", %" PRIi32 "), \n", text, v.x, v.y);
}

static inline void V3I32Print(const char *text, const v3i32 v)
{
	fprintf(stderr, "%s: (%" PRIi32 ", %" PRIi32 ", %" PRIi32 "), \n", text, v.x, v.y, v.z);
}

static inline void V4I32Print(const char *text, const v4i32 v)
{
	fprintf(stderr, "%s: (%" PRIi32 ", %" PRIi32 ", %" PRIi32 ", %" PRIi32 "), \n", text, v.x, v.y, v.z, v.w);
}

static inline void V2I64Print(const char *text, const v2i64 v)
{
	fprintf(stderr, "%s: (%" PRIi64 ", %" PRIi64 "), \n", text, v.x, v.y);
}

static inline void V3I64Print(const char *text, const v3i64 v)
{
	fprintf(stderr, "%s: (%" PRIi64 ", %" PRIi64 ", %" PRIi64 "), \n", text, v.x, v.y, v.z);
}

static inline void V4I64Print(const char *text, const v4i64 v)
{
	fprintf(stderr, "%s: (%" PRIi64 ", %" PRIi64 ", %" PRIi64 ", %" PRIi64 "), \n", text, v.x, v.y, v.z, v.w);
}

static ds_ForceInline v2 V2Add(const v2 a, const v2 b)
{
	return v2(a.x + b.x, a.y + b.y);
}

static ds_ForceInline v3 V3Add(const v3 a, const v3 b)
{
	return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static ds_ForceInline v4 V4Add(const v4 a, const v4 b)
{
	return v4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

/* a - b */
static ds_ForceInline v2 V2Sub(const v2 a, const v2 b)
{
	return v2(a.x - b.x, a.y - b.y);
}

static ds_ForceInline v3 V3Sub(const v3 a, const v3 b)
{
	return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static ds_ForceInline v4 V4Sub(const v4 a, const v4 b)
{
	return v4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

static ds_ForceInline v2 V2Mul(const v2 a, const v2 b)
{
	return v2(a.x * b.x, a.y * b.y);
}

static ds_ForceInline v3 V3Mul(const v3 a, const v3 b)
{
	return v3(a.x * b.x, a.y * b.y, a.z * b.z);
}

static ds_ForceInline v4 V4Mul(const v4 a, const v4 b)
{
	return v4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

/* a / b */
static ds_ForceInline v2 V2Div(const v2 a, const v2 b)
{
	return v2(a.x / b.x, a.y / b.y);
}

static ds_ForceInline v3 V3Div(const v3 a, const v3 b)
{
	return v3(a.x / b.x, a.y / b.y, a.z / b.z);
}

static ds_ForceInline v4 V4Div(const v4 a, const v4 b)
{
	return v4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

static ds_ForceInline v2 V2Scale(const v2 a, const f32 scale)
{
	return v2(scale * a.x, scale * a.y);
}

static ds_ForceInline v3 V3Scale(const v3 a, const f32 scale)
{
	return v3(scale * a.x, scale * a.y, scale * a.z);
}

static ds_ForceInline v4 V4Scale(const v4 a, const f32 scale)
{
	return v4(scale * a.x, scale * a.y, scale * a.z, scale * a.w);
}

/* a + scale*b */
static ds_ForceInline v2 V2AddScaled(const v2 a, const v2 b, const f32 scale)
{
	return v2(a.x + scale * b.x, a.y + scale * b.y);
}

static ds_ForceInline v3 V3AddScaled(const v3 a, const v3 b, const f32 scale)
{
	return v3(a.x + scale * b.x, a.y + scale * b.y, a.z + scale * b.z);
}

static ds_ForceInline v4 V4AddScaled(const v4 a, const v4 b, const f32 scale)
{
	return v4(a.x + scale * b.x, a.y + scale * b.y, a.z + scale * b.z, a.w + scale * b.w);
}

static ds_ForceInline v2 V2AddConstant(const v2 a, const f32 c)
{
	return v2(a.x + c, a.y + c);
}

static ds_ForceInline v3 V3AddConstant(const v3 a, const f32 c)
{
	return v3(a.x + c, a.y + c, a.z + c);
}

static ds_ForceInline v4 V4AddConstant(const v4 a, const f32 c)
{
	return v4(a.x + c, a.y + c, a.z + c, a.w + c);
}

static ds_ForceInline v2 V2Negate(const v2 a)
{
	return v2(-a.x, -a.y);
}

static ds_ForceInline v3 V3Negate(const v3 a)
{
	return v3(-a.x, -a.y, -a.z);
}

static ds_ForceInline v4 V4Negate(const v4 a)
{
	return v4(-a.x, -a.y, -a.z, -a.w);
}

static ds_ForceInline v2 V2Abs(const v2 a)
{
	return v2(F32Abs(a.x), F32Abs(a.y));
}

static ds_ForceInline v3 V3Abs(const v3 a)
{
	return v3(F32Abs(a.x), F32Abs(a.y), F32Abs(a.z));
}

static ds_ForceInline v4 V4Abs(const v4 a)
{
	return v4(F32Abs(a.x), F32Abs(a.y), F32Abs(a.z), F32Abs(a.w));
}

/* min[i] = F32Min(a[i], b[i]) */
static ds_ForceInline v2 V2Min(const v2 a, const v2 b)
{
	return v2(F32Min(a.x, b.x), F32Min(a.y, b.y));
}

static ds_ForceInline v3 V3Min(const v3 a, const v3 b)
{
	return v3(F32Min(a.x, b.x), F32Min(a.y, b.y), F32Min(a.z, b.z));
}

static ds_ForceInline v4 V4Min(const v4 a, const v4 b)
{
	return v4(F32Min(a.x, b.x), F32Min(a.y, b.y), F32Min(a.z, b.z), F32Min(a.w, b.w));
}

/* max[i] = F32Max(a[i], b[i]) */
static ds_ForceInline v2 V2Max(const v2 a, const v2 b)
{
	return v2(F32Max(a.x, b.x), F32Max(a.y, b.y));
}

static ds_ForceInline v3 V3Max(const v3 a, const v3 b)
{
	return v3(F32Max(a.x, b.x), F32Max(a.y, b.y), F32Max(a.z, b.z));
}

static ds_ForceInline v4 V4Max(const v4 a, const v4 b)
{
	return v4(F32Max(a.x, b.x), F32Max(a.y, b.y), F32Max(a.z, b.z), F32Max(a.w, b.w));
}

/* interpolate (alpha = 0.5f) */
static ds_ForceInline v2 V2Mix(const v2 a, const v2 b)
{
	return v2(0.5f * (a.x + b.x), 0.5f * (a.y + b.y));
}

static ds_ForceInline v3 V3Mix(const v3 a, const v3 b)
{
	return v3(0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z));
}

static ds_ForceInline v4 V4Mix(const v4 a, const v4 b)
{
	return v4(0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z), 0.5f * (a.w + b.w));
}

static ds_ForceInline f32 V2Dot(const v2 a, const v2 b)
{
	return a.x * b.x + a.y * b.y;
}

static ds_ForceInline f32 V3Dot(const v3 a, const v3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static ds_ForceInline f32 V4Dot(const v4 a, const v4 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static ds_ForceInline f32 V2LengthSquared(const v2 a)
{
	return a.x*a.x + a.y*a.y;
}

static ds_ForceInline f32 V3LengthSquared(const v3 a)
{
	return a.x*a.x + a.y*a.y + a.z*a.z;
}

static ds_ForceInline f32 V4LengthSquared(const v4 a)
{
	return a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w;
}

static ds_ForceInline f32 V2Length(const v2 a)
{
	return F32Sqrt(V2LengthSquared(a));
}

static ds_ForceInline f32 V3Length(const v3 a)
{
	return F32Sqrt(V3LengthSquared(a));
}

static ds_ForceInline f32 V4Length(const v4 a)
{
	return F32Sqrt(V4LengthSquared(a));
}

static ds_ForceInline v2 V2Normalize(const v2 a)
{
	return V2Scale(a, 1.0f / V2Length(a));
}

static ds_ForceInline v3 V3Normalize(const v3 a)
{
	return V3Scale(a, 1.0f / V3Length(a));
}

static ds_ForceInline v4 V4Normalize(const v4 a)
{
	return V4Scale(a, 1.0f / V4Length(a));
}

static ds_ForceInline f32 V2DistanceSquared(const v2 a, const v2 b)
{
	return V2LengthSquared(V2Sub(b, a));
}

static ds_ForceInline f32 V3DistanceSquared(const v3 a, const v3 b)
{
	return V3LengthSquared(V3Sub(b, a));
}

static ds_ForceInline f32 V4DistanceSquared(const v4 a, const v4 b)
{
	return V4LengthSquared(V4Sub(b, a));
}

static ds_ForceInline f32 V2Distance(const v2 a, const v2 b)
{
	return F32Sqrt(V2DistanceSquared(a, b));
}

static ds_ForceInline f32 V3Distance(const v3 a, const v3 b)
{
	return F32Sqrt(V3DistanceSquared(a, b));
}

static ds_ForceInline f32 V4Distance(const v4 a, const v4 b)
{
	return F32Sqrt(V4DistanceSquared(a, b));
}

/* a * alpha + b * (1-alpha) */
static ds_ForceInline v2 V2Interpolate(const v2 a, const v2 b, const f32 alpha)
{
	return v2(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha));
}

static ds_ForceInline v3 V3Interpolate(const v3 a, const v3 b, const f32 alpha)
{
	return v3(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha),
		  a.z * alpha + b.z * (1.0f - alpha));
}

static ds_ForceInline v4 V4Interpolate(const v4 a, const v4 b, const f32 alpha)
{
	return v4(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha),
		  a.z * alpha + b.z * (1.0f - alpha),
		  a.w * alpha + b.w * (1.0f - alpha));
}

static ds_ForceInline v2 V2InterpolatePiecewise(const v2 a, const v2 b, const v2 alpha)
{
	return v2(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y));
}

static ds_ForceInline v3 V3InterpolatePiecewise(const v3 a, const v3 b, const v3 alpha)
{
	return v3(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y),
		  a.z * alpha.z + b.z * (1.0f - alpha.z));
}

static ds_ForceInline v4 V4InterpolatePiecewise(const v4 a, const v4 b, const v4 alpha)
{
	return v4(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y),
		  a.z * alpha.z + b.z * (1.0f - alpha.z),
		  a.w * alpha.w + b.w * (1.0f - alpha.w));
}

/* a cross b */
static ds_ForceInline v3 V3Cross(const v3 a, const v3 b)
{
	return v3(a.y * b.z - a.z * b.y,
		  a.z * b.x - a.x * b.z,
		  a.x * b.y - a.y * b.x);
}

/* (a-center) x (b-center) */
static ds_ForceInline v3 V3RecenterCross(const v3 center, const v3 a, const v3 b)
{
	return V3Cross(V3Sub(a, center), V3Sub(b, center));
}

/* (a x b) x c */
static ds_ForceInline v3 V3TripleProduct(const v3 a, const v3 b, const v3 c)
{
	return V3Cross(V3Cross(a, b), c);
}

/* CCW rotation around the y-axis */
static ds_ForceInline v3 V3RotateY(const v3 a, const f32 angle)
{
	const f32 c = F32Cos(angle);
	const f32 s = F32Sin(angle);
	return v3(c * a.x + s * a.z, a.y, c * a.z - s * a.x);
}

/* 
 * Generate an appropriate epsilon for V3ParallelCheck given the upper-bound degrees for two vectors to be 
 * considered parallel. 
 */
static ds_ForceInline f32 V3ParallelCheckEpsilon(const f32 degrees)
{
	/* eps = sin(theta)^2, see Dot(a,a)*Dot(b,b) - Dot(a,b)^2 = sin(theta)^2 * Dot(a,a)*Dot(b,b) */
	const f32 sin_theta = F32Sin(degrees * F32_PI2 / 360.0f);
	return sin_theta*sin_theta;
}

/* Return 1 if the vectors are parallel, otherwise return 0. */
static ds_ForceInline u32 V3ParallelCheck(const v3 a, const v3 b, const f32 eps)
{
	const f32 d1d2 = V3Dot(a, b);
	const f32 d1d1_d2d2 = V3Dot(a, a) * V3Dot(b, b);
	return (d1d1_d2d2 > 0.0f) && (d1d1_d2d2 - d1d2*d1d2 < eps*d1d1_d2d2);
}

/* n3 Must be normalized! */
static ds_ForceInline void V3CreateBasis(v3 *n1, v3 *n2, const v3 n3)
{
	ds_Assert(1.0f - F32_EPSILON*10000.0f <= V3Length(n3) && V3Length(n3) <= 1.0f + F32_EPSILON*10000.0f);

	const f32 inv_sqrt2 = 1.0f / F32Sqrt(2.0f);
	const v3 t = (F32Abs(n3.z) < inv_sqrt2)
		? v3(-n3.y, n3.x, 0.0f)
		: v3(0.0f, -n3.z, n3.y);

	*n1 = V3Cross(n3, t);
	*n1 = V3Scale(*n1, 1.0f / V3Length(*n1));
	*n2 = V3Cross(*n1, n3);
	*n2 = V3Scale(*n2, 1.0f / V3Length(*n2));

	ds_Assert(1.0f - F32_EPSILON*10000.0f <= V3Length(*n1) && V3Length(*n1) <= 1.0f + F32_EPSILON*10000.0f);
	ds_Assert(1.0f - F32_EPSILON*10000.0f <= V3Length(*n2) && V3Length(*n2) <= 1.0f + F32_EPSILON*10000.0f);
	ds_Assert(F32Abs(V3Dot(*n1, *n2)) <= F32_EPSILON * 100.0f);
	ds_Assert(F32Abs(V3Dot(*n1, n3)) <= F32_EPSILON * 100.0f);
	ds_Assert(F32Abs(V3Dot(*n2, n3)) <= F32_EPSILON * 100.0f);
}

#ifdef __cplusplus
}
#endif

#endif

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

/*
ds_vector.h
===========
Vector functions on the types of ds_types.h: the f32 vectors v2, v3 and v4 (most functions), the integer vectors
(zero and printing only) and v3f64 world positions. Constructors (V3(x, y, z), ...) are in ds_types.h. Included by
ds_math.h.

Every function is plain IEEE arithmetic in a fixed order (sums left to right, no contraction), so it gives the
same bits on every platform and compiler, except V3RotateY and V3ParallelCheckEpsilon (the C library's sin and
cos). Nothing is checked unless a function says so: a zero divisor or a zero-length vector gives inf or NaN.

Sections: zero vectors; component-wise arithmetic; world positions (v3f64); products, lengths, distances;
interpolation; directions and bases; printing. Inline definitions at the bottom, in the same order.
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

/*********************************** zero vectors ***********************************/

/*
The zero vector of each type.
*/

static ds_ForceInline v2 V2Zero(void);
static ds_ForceInline v3 V3Zero(void);
static ds_ForceInline v4 V4Zero(void);
static ds_ForceInline v2u32 V2U32Zero(void);
static ds_ForceInline v3u32 V3U32Zero(void);
static ds_ForceInline v4u32 V4U32Zero(void);
static ds_ForceInline v2u64 V2U64Zero(void);
static ds_ForceInline v3u64 V3U64Zero(void);
static ds_ForceInline v4u64 V4U64Zero(void);
static ds_ForceInline v2i32 V2I32Zero(void);
static ds_ForceInline v3i32 V3I32Zero(void);
static ds_ForceInline v4i32 V4I32Zero(void);
static ds_ForceInline v2i64 V2I64Zero(void);
static ds_ForceInline v3i64 V3I64Zero(void);
static ds_ForceInline v4i64 V4I64Zero(void);

/**************************** component-wise arithmetic *****************************/

/*
Per component: Add, Sub (a - b), Mul, Div (a / b), Scale (scale a), AddScaled (a + scale b), AddConstant (c added
to every component), Negate, Abs, Min and Max (F32Min/F32Max per component: a NaN in a gives b's component, a NaN
in b is returned).
*/

static ds_ForceInline v2 V2Add(const v2 a, const v2 b);
static ds_ForceInline v3 V3Add(const v3 a, const v3 b);
static ds_ForceInline v4 V4Add(const v4 a, const v4 b);
static ds_ForceInline v2 V2Sub(const v2 a, const v2 b);
static ds_ForceInline v3 V3Sub(const v3 a, const v3 b);
static ds_ForceInline v4 V4Sub(const v4 a, const v4 b);
static ds_ForceInline v2 V2Mul(const v2 a, const v2 b);
static ds_ForceInline v3 V3Mul(const v3 a, const v3 b);
static ds_ForceInline v4 V4Mul(const v4 a, const v4 b);
static ds_ForceInline v2 V2Div(const v2 a, const v2 b);
static ds_ForceInline v3 V3Div(const v3 a, const v3 b);
static ds_ForceInline v4 V4Div(const v4 a, const v4 b);
static ds_ForceInline v2 V2Scale(const v2 a, const f32 scale);
static ds_ForceInline v3 V3Scale(const v3 a, const f32 scale);
static ds_ForceInline v4 V4Scale(const v4 a, const f32 scale);
static ds_ForceInline v2 V2AddScaled(const v2 a, const v2 b, const f32 scale);
static ds_ForceInline v3 V3AddScaled(const v3 a, const v3 b, const f32 scale);
static ds_ForceInline v4 V4AddScaled(const v4 a, const v4 b, const f32 scale);
static ds_ForceInline v2 V2AddConstant(const v2 a, const f32 c);
static ds_ForceInline v3 V3AddConstant(const v3 a, const f32 c);
static ds_ForceInline v4 V4AddConstant(const v4 a, const f32 c);
static ds_ForceInline v2 V2Negate(const v2 a);
static ds_ForceInline v3 V3Negate(const v3 a);
static ds_ForceInline v4 V4Negate(const v4 a);
static ds_ForceInline v2 V2Abs(const v2 a);
static ds_ForceInline v3 V3Abs(const v3 a);
static ds_ForceInline v4 V4Abs(const v4 a);
static ds_ForceInline v2 V2Min(const v2 a, const v2 b);
static ds_ForceInline v3 V3Min(const v3 a, const v3 b);
static ds_ForceInline v4 V4Min(const v4 a, const v4 b);
static ds_ForceInline v2 V2Max(const v2 a, const v2 b);
static ds_ForceInline v3 V3Max(const v3 a, const v3 b);
static ds_ForceInline v4 V4Max(const v4 a, const v4 b);

/***************************** world positions (v3f64) ******************************/

/*
v3f64: world positions, precise far from the origin. f64 is only for storage and the crossing; math runs in f32
relative to a nearby point:

     local = V3F64Demote(V3F64Sub(p, base))      (rounding relative to |p - base|, not |p|)
     p     = V3F64Add(p, V3Promote(delta))       (f32 increments back into world positions)

V3Promote is exact. V3F64Demote rounds each coordinate to the nearest f32, V3F64DemoteDownwards toward -inf and
V3F64DemoteUpwards toward +inf (F64DemoteDownwards/Upwards; for conservative boxes). V3F64Sub is a - b.
*/

static ds_ForceInline v3f64 V3Promote(const v3 a);
static ds_ForceInline v3 V3F64Demote(const v3f64 a);
static ds_ForceInline v3 V3F64DemoteDownwards(const v3f64 a);
static ds_ForceInline v3 V3F64DemoteUpwards(const v3f64 a);
static ds_ForceInline v3f64 V3F64Add(const v3f64 a, const v3f64 b);
static ds_ForceInline v3f64 V3F64Sub(const v3f64 a, const v3f64 b);

/*************************** products, lengths, distances ***************************/

/*
Dot sums left to right ((x + y) + z). Length is the square root of LengthSquared, Distance of DistanceSquared
(|b - a|^2). The squares overflow for components above about 1.8e19 and underflow below about 1e-19
(subnormal; 0 below about 3e-23).

Normalize returns a / |a|. Precondition: a != 0 and |a| inside that range; a = 0 gives NaN components.

Cross is a x b (right-handed), RecenterCross (a - center) x (b - center), TripleProduct the vector (a x b) x c.
*/

static ds_ForceInline f32 V2Dot(const v2 a, const v2 b);
static ds_ForceInline f32 V3Dot(const v3 a, const v3 b);
static ds_ForceInline f32 V4Dot(const v4 a, const v4 b);
static ds_ForceInline f32 V2LengthSquared(const v2 a);
static ds_ForceInline f32 V3LengthSquared(const v3 a);
static ds_ForceInline f32 V4LengthSquared(const v4 a);
static ds_ForceInline f32 V2Length(const v2 a);
static ds_ForceInline f32 V3Length(const v3 a);
static ds_ForceInline f32 V4Length(const v4 a);
static ds_ForceInline v2 V2Normalize(const v2 a);
static ds_ForceInline v3 V3Normalize(const v3 a);
static ds_ForceInline v4 V4Normalize(const v4 a);
static ds_ForceInline f32 V2DistanceSquared(const v2 a, const v2 b);
static ds_ForceInline f32 V3DistanceSquared(const v3 a, const v3 b);
static ds_ForceInline f32 V4DistanceSquared(const v4 a, const v4 b);
static ds_ForceInline f32 V2Distance(const v2 a, const v2 b);
static ds_ForceInline f32 V3Distance(const v3 a, const v3 b);
static ds_ForceInline f32 V4Distance(const v4 a, const v4 b);
static ds_ForceInline v3 V3Cross(const v3 a, const v3 b);
static ds_ForceInline v3 V3RecenterCross(const v3 center, const v3 a, const v3 b);
static ds_ForceInline v3 V3TripleProduct(const v3 a, const v3 b, const v3 c);

/********************************** interpolation ***********************************/

/*
Interpolate returns a alpha + b (1 - alpha): alpha weights a, the reverse of the usual lerp(a, b, t); exactly a at
alpha = 1 and b at alpha = 0. InterpolatePiecewise takes one alpha per component. Mix is the midpoint 0.5 (a + b).
*/

static ds_ForceInline v2 V2Mix(const v2 a, const v2 b);
static ds_ForceInline v3 V3Mix(const v3 a, const v3 b);
static ds_ForceInline v4 V4Mix(const v4 a, const v4 b);
static ds_ForceInline v2 V2Interpolate(const v2 a, const v2 b, const f32 alpha);
static ds_ForceInline v3 V3Interpolate(const v3 a, const v3 b, const f32 alpha);
static ds_ForceInline v4 V4Interpolate(const v4 a, const v4 b, const f32 alpha);
static ds_ForceInline v2 V2InterpolatePiecewise(const v2 a, const v2 b, const v2 alpha);
static ds_ForceInline v3 V3InterpolatePiecewise(const v3 a, const v3 b, const v3 alpha);
static ds_ForceInline v4 V4InterpolatePiecewise(const v4 a, const v4 b, const v4 alpha);

/******************************* directions and bases *******************************/

/*
V3Support returns the index of the support vertex of v[0..count), the one maximizing dot(v[i], dir). Ties go to
the lowest index (deterministic). dir needs no normalization; dir = 0 returns 0. count > 0 (asserted).

V3RotateY rotates a by angle (radians) about +y by the right-hand rule (x turns toward -z).

V3ParallelCheck returns 1 if sin^2 of the angle between a and b is below eps (antiparallel counts as parallel), 0
if a or b is zero. It computes |a|^2 |b|^2 - (a.b)^2, which cancels for nearly parallel vectors: eps below about
1e-6 is not resolved. V3ParallelCheckEpsilon(degrees) returns that eps, sin^2(degrees).

V3CreateBasis writes n1 and n2: unit, orthogonal to each other and to n3. Precondition: |n3| = 1 (asserted within
1.2e-3). n1 x n2 = -n3: (n1, n2, n3) is left-handed, (n2, n1, n3) right-handed. The construction switches at
|n3.z| = 1/sqrt(2), so the basis jumps as n3 crosses it.

V3FiniteCheck returns 1 if no component is NaN or +-inf.
*/

static ds_ForceInline u32 V3Support(const v3 *v, const u32 count, const v3 dir);
static ds_ForceInline v3 V3RotateY(const v3 a, const f32 angle);
static ds_ForceInline f32 V3ParallelCheckEpsilon(const f32 degrees);
static ds_ForceInline u32 V3ParallelCheck(const v3 a, const v3 b, const f32 eps);
static ds_ForceInline void V3CreateBasis(v3 *n1, v3 *n2, const v3 n3);
static ds_ForceInline u32 V3FiniteCheck(const v3 v);

/************************************* printing *************************************/

/*
Print text and the components to stderr, for debugging.
*/

static inline void V2Print(const char *text, const v2 v);
static inline void V3Print(const char *text, const v3 v);
static inline void V4Print(const char *text, const v4 v);
static inline void V2U32Print(const char *text, const v2u32 v);
static inline void V3U32Print(const char *text, const v3u32 v);
static inline void V4U32Print(const char *text, const v4u32 v);
static inline void V2U64Print(const char *text, const v2u64 v);
static inline void V3U64Print(const char *text, const v3u64 v);
static inline void V4U64Print(const char *text, const v4u64 v);
static inline void V2I32Print(const char *text, const v2i32 v);
static inline void V3I32Print(const char *text, const v3i32 v);
static inline void V4I32Print(const char *text, const v4i32 v);
static inline void V2I64Print(const char *text, const v2i64 v);
static inline void V3I64Print(const char *text, const v3i64 v);
static inline void V4I64Print(const char *text, const v4i64 v);

/************************* zero vectors: inline definitions *************************/

static ds_ForceInline v2 V2Zero(void)
{
	return V2(0.0f, 0.0f);
}

static ds_ForceInline v3 V3Zero(void)
{
	return V3(0.0f, 0.0f, 0.0f);
}

static ds_ForceInline v4 V4Zero(void)
{
	return V4(0.0f, 0.0f, 0.0f, 0.0f);
}

static ds_ForceInline v2u32 V2U32Zero(void)
{
	return V2U32(0, 0);
}

static ds_ForceInline v3u32 V3U32Zero(void)
{
	return V3U32(0, 0, 0);
}

static ds_ForceInline v4u32 V4U32Zero(void)
{
	return V4U32(0, 0, 0, 0);
}

static ds_ForceInline v2u64 V2U64Zero(void)
{
	return V2U64(0, 0);
}

static ds_ForceInline v3u64 V3U64Zero(void)
{
	return V3U64(0, 0, 0);
}

static ds_ForceInline v4u64 V4U64Zero(void)
{
	return V4U64(0, 0, 0, 0);
}

static ds_ForceInline v2i32 V2I32Zero(void)
{
	return V2I32(0, 0);
}

static ds_ForceInline v3i32 V3I32Zero(void)
{
	return V3I32(0, 0, 0);
}

static ds_ForceInline v4i32 V4I32Zero(void)
{
	return V4I32(0, 0, 0, 0);
}

static ds_ForceInline v2i64 V2I64Zero(void)
{
	return V2I64(0, 0);
}

static ds_ForceInline v3i64 V3I64Zero(void)
{
	return V3I64(0, 0, 0);
}

static ds_ForceInline v4i64 V4I64Zero(void)
{
	return V4I64(0, 0, 0, 0);
}

/****************** component-wise arithmetic: inline definitions *******************/

static ds_ForceInline v2 V2Add(const v2 a, const v2 b)
{
	return V2(a.x + b.x, a.y + b.y);
}

static ds_ForceInline v3 V3Add(const v3 a, const v3 b)
{
	return V3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static ds_ForceInline v4 V4Add(const v4 a, const v4 b)
{
	return V4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

static ds_ForceInline v2 V2Sub(const v2 a, const v2 b)
{
	return V2(a.x - b.x, a.y - b.y);
}

static ds_ForceInline v3 V3Sub(const v3 a, const v3 b)
{
	return V3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static ds_ForceInline v4 V4Sub(const v4 a, const v4 b)
{
	return V4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

static ds_ForceInline v2 V2Mul(const v2 a, const v2 b)
{
	return V2(a.x * b.x, a.y * b.y);
}

static ds_ForceInline v3 V3Mul(const v3 a, const v3 b)
{
	return V3(a.x * b.x, a.y * b.y, a.z * b.z);
}

static ds_ForceInline v4 V4Mul(const v4 a, const v4 b)
{
	return V4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

static ds_ForceInline v2 V2Div(const v2 a, const v2 b)
{
	return V2(a.x / b.x, a.y / b.y);
}

static ds_ForceInline v3 V3Div(const v3 a, const v3 b)
{
	return V3(a.x / b.x, a.y / b.y, a.z / b.z);
}

static ds_ForceInline v4 V4Div(const v4 a, const v4 b)
{
	return V4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

static ds_ForceInline v2 V2Scale(const v2 a, const f32 scale)
{
	return V2(scale * a.x, scale * a.y);
}

static ds_ForceInline v3 V3Scale(const v3 a, const f32 scale)
{
	return V3(scale * a.x, scale * a.y, scale * a.z);
}

static ds_ForceInline v4 V4Scale(const v4 a, const f32 scale)
{
	return V4(scale * a.x, scale * a.y, scale * a.z, scale * a.w);
}

static ds_ForceInline v2 V2AddScaled(const v2 a, const v2 b, const f32 scale)
{
	return V2(a.x + scale * b.x, a.y + scale * b.y);
}

static ds_ForceInline v3 V3AddScaled(const v3 a, const v3 b, const f32 scale)
{
	return V3(a.x + scale * b.x, a.y + scale * b.y, a.z + scale * b.z);
}

static ds_ForceInline v4 V4AddScaled(const v4 a, const v4 b, const f32 scale)
{
	return V4(a.x + scale * b.x, a.y + scale * b.y, a.z + scale * b.z, a.w + scale * b.w);
}

static ds_ForceInline v2 V2AddConstant(const v2 a, const f32 c)
{
	return V2(a.x + c, a.y + c);
}

static ds_ForceInline v3 V3AddConstant(const v3 a, const f32 c)
{
	return V3(a.x + c, a.y + c, a.z + c);
}

static ds_ForceInline v4 V4AddConstant(const v4 a, const f32 c)
{
	return V4(a.x + c, a.y + c, a.z + c, a.w + c);
}

static ds_ForceInline v2 V2Negate(const v2 a)
{
	return V2(-a.x, -a.y);
}

static ds_ForceInline v3 V3Negate(const v3 a)
{
	return V3(-a.x, -a.y, -a.z);
}

static ds_ForceInline v4 V4Negate(const v4 a)
{
	return V4(-a.x, -a.y, -a.z, -a.w);
}

static ds_ForceInline v2 V2Abs(const v2 a)
{
	return V2(F32Abs(a.x), F32Abs(a.y));
}

static ds_ForceInline v3 V3Abs(const v3 a)
{
	return V3(F32Abs(a.x), F32Abs(a.y), F32Abs(a.z));
}

static ds_ForceInline v4 V4Abs(const v4 a)
{
	return V4(F32Abs(a.x), F32Abs(a.y), F32Abs(a.z), F32Abs(a.w));
}

static ds_ForceInline v2 V2Min(const v2 a, const v2 b)
{
	return V2(F32Min(a.x, b.x), F32Min(a.y, b.y));
}

static ds_ForceInline v3 V3Min(const v3 a, const v3 b)
{
	return V3(F32Min(a.x, b.x), F32Min(a.y, b.y), F32Min(a.z, b.z));
}

static ds_ForceInline v4 V4Min(const v4 a, const v4 b)
{
	return V4(F32Min(a.x, b.x), F32Min(a.y, b.y), F32Min(a.z, b.z), F32Min(a.w, b.w));
}

static ds_ForceInline v2 V2Max(const v2 a, const v2 b)
{
	return V2(F32Max(a.x, b.x), F32Max(a.y, b.y));
}

static ds_ForceInline v3 V3Max(const v3 a, const v3 b)
{
	return V3(F32Max(a.x, b.x), F32Max(a.y, b.y), F32Max(a.z, b.z));
}

static ds_ForceInline v4 V4Max(const v4 a, const v4 b)
{
	return V4(F32Max(a.x, b.x), F32Max(a.y, b.y), F32Max(a.z, b.z), F32Max(a.w, b.w));
}

/******************* world positions (v3f64): inline definitions ********************/

static ds_ForceInline v3f64 V3Promote(const v3 a)
{
	return V3F64(a.x, a.y, a.z);
}

static ds_ForceInline v3 V3F64Demote(const v3f64 a)
{
	return V3((f32) a.x, (f32) a.y, (f32) a.z);
}

static ds_ForceInline v3 V3F64DemoteDownwards(const v3f64 a)
{
	return V3(F64DemoteDownwards(a.x), F64DemoteDownwards(a.y), F64DemoteDownwards(a.z));
}

static ds_ForceInline v3 V3F64DemoteUpwards(const v3f64 a)
{
	return V3(F64DemoteUpwards(a.x), F64DemoteUpwards(a.y), F64DemoteUpwards(a.z));
}

static ds_ForceInline v3f64 V3F64Add(const v3f64 a, const v3f64 b)
{
	return V3F64(a.x + b.x, a.y + b.y, a.z + b.z);
}

static ds_ForceInline v3f64 V3F64Sub(const v3f64 a, const v3f64 b)
{
	return V3F64(a.x - b.x, a.y - b.y, a.z - b.z);
}

/***************** products, lengths, distances: inline definitions *****************/

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

static ds_ForceInline v3 V3Cross(const v3 a, const v3 b)
{
	return V3(a.y * b.z - a.z * b.y,
		  a.z * b.x - a.x * b.z,
		  a.x * b.y - a.y * b.x);
}

static ds_ForceInline v3 V3RecenterCross(const v3 center, const v3 a, const v3 b)
{
	return V3Cross(V3Sub(a, center), V3Sub(b, center));
}

static ds_ForceInline v3 V3TripleProduct(const v3 a, const v3 b, const v3 c)
{
	return V3Cross(V3Cross(a, b), c);
}

/************************ interpolation: inline definitions *************************/

static ds_ForceInline v2 V2Mix(const v2 a, const v2 b)
{
	return V2(0.5f * (a.x + b.x), 0.5f * (a.y + b.y));
}

static ds_ForceInline v3 V3Mix(const v3 a, const v3 b)
{
	return V3(0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z));
}

static ds_ForceInline v4 V4Mix(const v4 a, const v4 b)
{
	return V4(0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z), 0.5f * (a.w + b.w));
}

static ds_ForceInline v2 V2Interpolate(const v2 a, const v2 b, const f32 alpha)
{
	return V2(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha));
}

static ds_ForceInline v3 V3Interpolate(const v3 a, const v3 b, const f32 alpha)
{
	return V3(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha),
		  a.z * alpha + b.z * (1.0f - alpha));
}

static ds_ForceInline v4 V4Interpolate(const v4 a, const v4 b, const f32 alpha)
{
	return V4(a.x * alpha + b.x * (1.0f - alpha),
		  a.y * alpha + b.y * (1.0f - alpha),
		  a.z * alpha + b.z * (1.0f - alpha),
		  a.w * alpha + b.w * (1.0f - alpha));
}

static ds_ForceInline v2 V2InterpolatePiecewise(const v2 a, const v2 b, const v2 alpha)
{
	return V2(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y));
}

static ds_ForceInline v3 V3InterpolatePiecewise(const v3 a, const v3 b, const v3 alpha)
{
	return V3(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y),
		  a.z * alpha.z + b.z * (1.0f - alpha.z));
}

static ds_ForceInline v4 V4InterpolatePiecewise(const v4 a, const v4 b, const v4 alpha)
{
	return V4(a.x * alpha.x + b.x * (1.0f - alpha.x),
		  a.y * alpha.y + b.y * (1.0f - alpha.y),
		  a.z * alpha.z + b.z * (1.0f - alpha.z),
		  a.w * alpha.w + b.w * (1.0f - alpha.w));
}

/********************* directions and bases: inline definitions *********************/

static ds_ForceInline u32 V3Support(const v3 *v, const u32 count, const v3 dir)
{
	ds_Assert(count > 0);
	u32 best = 0;
	f32 best_dot = V3Dot(v[0], dir);
	for (u32 i = 1; i < count; ++i)
	{
		const f32 dot = V3Dot(v[i], dir);
		if (best_dot < dot)
		{
			best = i;
			best_dot = dot;
		}
	}

	return best;
}

static ds_ForceInline v3 V3RotateY(const v3 a, const f32 angle)
{
	const f32 c = F32Cos(angle);
	const f32 s = F32Sin(angle);
	return V3(c * a.x + s * a.z, a.y, c * a.z - s * a.x);
}

static ds_ForceInline f32 V3ParallelCheckEpsilon(const f32 degrees)
{
	/* eps = sin(theta)^2, see Dot(a,a)*Dot(b,b) - Dot(a,b)^2 = sin(theta)^2 * Dot(a,a)*Dot(b,b) */
	const f32 sin_theta = F32Sin(degrees * F32_PI2 / 360.0f);
	return sin_theta*sin_theta;
}

static ds_ForceInline u32 V3ParallelCheck(const v3 a, const v3 b, const f32 eps)
{
	const f32 d1d2 = V3Dot(a, b);
	const f32 d1d1_d2d2 = V3Dot(a, a) * V3Dot(b, b);
	return (d1d1_d2d2 > 0.0f) && (d1d1_d2d2 - d1d2*d1d2 < eps*d1d1_d2d2);
}

static ds_ForceInline void V3CreateBasis(v3 *n1, v3 *n2, const v3 n3)
{
	ds_Assert(1.0f - F32_EPSILON*10000.0f <= V3Length(n3) && V3Length(n3) <= 1.0f + F32_EPSILON*10000.0f);

	const f32 inv_sqrt2 = 1.0f / F32Sqrt(2.0f);
	const v3 t = (F32Abs(n3.z) < inv_sqrt2)
		? V3(-n3.y, n3.x, 0.0f)
		: V3(0.0f, -n3.z, n3.y);

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

static ds_ForceInline u32 V3FiniteCheck(const v3 v)
{
	return F32FiniteCheck(v.x) && F32FiniteCheck(v.y) && F32FiniteCheck(v.z);
}

/*************************** printing: inline definitions ***************************/

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

#ifdef __cplusplus
}
#endif

#endif

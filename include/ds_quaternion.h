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
ds_quaternion.h
===============
Quaternions q = (x, y, z, w), w the scalar part, as rotations: construction, arithmetic, rotating vectors. The
type and Q(x, y, z, w) are in ds_types.h; matrices from quaternions (M3QUnit) in ds_matrix.h. Included by ds_math.h.

Everything is plain IEEE arithmetic in a fixed order, the same bits on every platform and compiler, except
QAxisAngle and QUnitAxisAngle (the C library's sin and cos). Nothing is checked: a zero quaternion or axis gives NaN.

Sections: construction, arithmetic, rotating vectors; inline definitions at the bottom, in the same order.
*/

#ifndef __DS_Q_H__
#define __DS_Q_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_define.h"
#include "ds_types.h"
#include "ds_float.h"
#include "ds_vector.h"

/**
 * QUATERNION RULES:
 *
 *		   A Y
 *		   |
 *		   |
 *		   .------> X
 *	      /
 *	     L Z
 *
 *	      i^2 = j^2 = k^2 = -1
 *
 *	      (point i,j,k) * (axis i,j,k) = CW rotation [rules for i,j,k multiplication]
 *
 *	      ij =  k
 *	      ji = -k
 *	      ik = -j
 *	      ki =  j
 *	      jk =  i
 *	      kj = -i
 */

/*********************************** construction ***********************************/

/* the rotation by 0: (0, 0, 0, 1) */
static ds_ForceInline q QIdentity(void);
/*
 * Return quaternion representing rotation around (non-normalized) axis with given angle (radians, right-hand rule);
 * a zero axis gives NaN. Uses the C library's sin/cos.
 */
static ds_ForceInline q QAxisAngle(const v3 axis, const f32 angle);
/*
 * Return quaternion representing rotation around (Normalized!) axis with given angle (radians, right-hand rule):
 * (sin(angle/2) axis, cos(angle/2)). Uses the C library's sin/cos.
 */
static ds_ForceInline q QUnitAxisAngle(const v3 axis, const f32 angle);

/************************************ arithmetic ************************************/

/* a + b per component */
static ds_ForceInline q QAdd(const q a, const q b);
/* a - b */
static ds_ForceInline q QSub(const q a, const q b);
/* Hamilton product a b: rotating by QMul(a, b) rotates by b first, then by a */
static ds_ForceInline q QMul(const q a, const q b);
/* a * scale per component */
static ds_ForceInline q QScale(const q a, const f32 scale);
/* conj(a) = (-x, -y, -z, w): the inverse of a unit quaternion */
static ds_ForceInline q QConj(const q a);
/* |a| */
static ds_ForceInline f32 QNorm(const q a);
/* a^-1 = conj(a) / |a|^2 (a = 0 gives NaN) */
static ds_ForceInline q QInverse(const q a);
/* a / |a| (a = 0 gives NaN) */
static ds_ForceInline q QNormalize(const q a);

/********************************* rotating vectors *********************************/

/*
 * Rotate v by unit a: ava^-1 = v + 2*a_w*Cross(a_xyz, v) + 2*Cross(a_xyz, Cross(a_xyz, v)). Precondition |a| = 1, not
 * checked: otherwise the result is R v + (1 - |a|^2) (v - R v), R the rotation of a / |a|.
 */
static ds_ForceInline v3 QUnitRotate(const q a, const v3 v);
/* QUnitRotate with v = (0, y, 0); same addition order: the same result except the sign of a zero component */
static ds_ForceInline v3 QYUnitRotate(const q a, const f32 y);

/************************* construction: inline definitions *************************/

static ds_ForceInline q QIdentity(void)
{
	return Q(0.0f, 0.0f, 0.0f, 1.0f);
}

static ds_ForceInline q QAxisAngle(const v3 axis, const f32 angle)
{
	const f32 scale = F32Sin(angle/2.0f) / V3Length(axis);
	return Q(scale * axis.x, scale * axis.y, scale * axis.z, F32Cos(angle/2.0f));
}

static ds_ForceInline q QUnitAxisAngle(const v3 axis, const f32 angle)
{
	const f32 scale = F32Sin(angle/2.0f);
	return Q(scale * axis.x, scale * axis.y, scale * axis.z, F32Cos(angle/2.0f));
}

/************************** arithmetic: inline definitions **************************/

static ds_ForceInline q QAdd(const q a, const q b)
{
	return Q(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

static ds_ForceInline q QSub(const q a, const q b)
{
	return Q(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

static ds_ForceInline q QMul(const q a, const q b)
{
	return Q(a.x*b.w + a.w*b.x + a.y*b.z - a.z*b.y,
		     a.y*b.w + a.w*b.y + a.z*b.x - a.x*b.z,
		     a.z*b.w + a.w*b.z + a.x*b.y - a.y*b.x,
		     a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z);
}

static ds_ForceInline q QScale(const q a, const f32 scale)
{
	return Q(a.x * scale, a.y * scale, a.z * scale, a.w * scale);
}

static ds_ForceInline q QConj(const q a)
{
	return Q(-a.x, -a.y, -a.z, a.w);
}

static ds_ForceInline f32 QNorm(const q a)
{
	return F32Sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
}

static ds_ForceInline q QInverse(const q a)
{
	return QScale(QConj(a), 1.0f / (a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));
}

static ds_ForceInline q QNormalize(const q a)
{
	return QScale(a, 1.0f / QNorm(a));
}

/*********************** rotating vectors: inline definitions ***********************/

static ds_ForceInline v3 QUnitRotate(const q a, const v3 v)
{
	const v3 c = V3(2.0f*(a.y*v.z - a.z*v.y),
			2.0f*(a.z*v.x - a.x*v.z),
			2.0f*(a.x*v.y - a.y*v.x));

	return V3(v.x + a.w*c.x + a.y*c.z - a.z*c.y,
		  v.y + a.w*c.y + a.z*c.x - a.x*c.z,
		  v.z + a.w*c.z + a.x*c.y - a.y*c.x);
}

static ds_ForceInline v3 QYUnitRotate(const q a, const f32 y)
{
	const f32 cx = -2.0f*a.z*y;
	const f32 cz = 2.0f*a.x*y;

	return V3(a.w*cx + a.y*cz,
		  y + a.z*cx - a.x*cz,
		  a.w*cz - a.y*cx);
}

#ifdef __cplusplus
}
#endif

#endif

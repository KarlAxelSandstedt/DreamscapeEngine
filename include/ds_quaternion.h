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

static ds_ForceInline q QIdentity(void)
{
	return Q(0.0f, 0.0f, 0.0f, 1.0f);
}

/* Return quaternion representing rotation around (non-normalized) axis with given angle */
static ds_ForceInline q QAxisAngle(const v3 axis, const f32 angle)
{
	const f32 scale = F32Sin(angle/2.0f) / V3Length(axis);
	return Q(scale * axis.x, scale * axis.y, scale * axis.z, F32Cos(angle/2.0f));
}

/* Return quaternion representing rotation around (Normalized!) axis with given angle */
static ds_ForceInline q QUnitAxisAngle(const v3 axis, const f32 angle)
{
	const f32 scale = F32Sin(angle/2.0f);
	return Q(scale * axis.x, scale * axis.y, scale * axis.z, F32Cos(angle/2.0f));
}

static ds_ForceInline q QAdd(const q a, const q b)
{
	return Q(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

/* a - b */
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

/* a^-1 = conj(a) / |a|^2 */
static ds_ForceInline q QInverse(const q a)
{
	return QScale(QConj(a), 1.0f / (a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w));
}

static ds_ForceInline q QNormalize(const q a)
{
	return QScale(a, 1.0f / QNorm(a));
}

/* Rotate v by a: ava^-1 = v + 2*a_w*Cross(a_xyz, v) + 2*Cross(a_xyz, Cross(a_xyz, v)) */
static ds_ForceInline v3 QV3Rotate(const q a, const v3 v)
{
	const v3 c = V3(2.0f*(a.y*v.z - a.z*v.y),
			2.0f*(a.z*v.x - a.x*v.z),
			2.0f*(a.x*v.y - a.y*v.x));

	return V3(v.x + a.w*c.x + a.y*c.z - a.z*c.y,
		  v.y + a.w*c.y + a.z*c.x - a.x*c.z,
		  v.z + a.w*c.z + a.x*c.y - a.y*c.x);
}

#ifdef __cplusplus
}
#endif

#endif

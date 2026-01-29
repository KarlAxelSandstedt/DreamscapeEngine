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

#ifndef __DS_QUATERNION_H__
#define __DS_QUATERNION_H__

#ifdef __cplusplus
extern "C" { 
#endif

/**
 * QUATERNION RULES:
 *	
 *		A Y
 *		|
 *		|
 *		.------> X
 *	       /
 *	      L Z
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

/* Quaternion Creation */
/* Return quaternion representing rotation around (non-normalized) axis with given angle */
void	QuatAxisAngle(quat dst, const vec3 axis, const f32 angle);
/* Return quaternion representing rotation around (Normalized!) axis with given angle */
void 	QuatUnitAxisAngle(quat dst, const vec3 axis, const f32 angle);

/**
 * Quaternion Rotation operation matrix Q in: qvq* = Qv
 * q = [cos(t/2), sin(t/2)v] where |v| = 1 and v is the rotation axis. t is wanted angle rotation. For some point
 * v, the achieved rotation is calculated as qvq*.
 */
void 	Mat3Quat(mat3 dst, const quat q);
void 	Mat4Quat(mat4 dst, const quat q);



/* Quaternion Operations and Functions */
void 	QuatSet(quat dst, const f32 x, const f32 y, const f32 z, const f32 w);
void 	QuatAdd(quat dst, const quat p, const quat q);
void 	QuatTranslate(quat dst, const quat translation);
void 	QuatSub(quat dst, const quat p, const quat q);
void 	QuatMul(quat dst, const quat p, const quat q);
void 	QuatScale(quat dst, const f32 scale);
void 	QuatCopy(quat dst, const quat src);
void 	QuatConj(quat conj, const quat q);
void 	QuatInverse(quat inv, const quat q);
f32 	QuatNorm(const quat q);
void 	QuatNormalize(quat q);

#ifdef __cplusplus
} 
#endif

#endif

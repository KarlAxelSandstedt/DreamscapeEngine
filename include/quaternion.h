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

#include "ds_math.h"

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
/*(axis does not have to be normalized) */
void axis_angle_to_quaternion(quat dst, const vec3 axis, const f32 angle);
/*(axis have to be normalized) */
void unit_axis_angle_to_quaternion(quat dst, const vec3 axis, const f32 angle);

/* Quaternion Operations and Functions */
void quat_set(quat dst, const f32 x, const f32 y, const f32 z, const f32 w);
void quat_add(quat dst, const quat p, const quat q);
void quat_translate(quat dst, const quat translation);
void quat_sub(quat dst, const quat p, const quat q);
void quat_mult(quat dst, const quat p, const quat q);
void quat_scale(quat dst, const f32 scale);
void quat_copy(quat dst, const quat src);
void quat_conj(quat conj, const quat q);
void quat_inv(quat inv, const quat q);
f32 quat_norm(const quat q);
void quat_normalize(quat q);

/**
 * Quaternion Rotation operation matrix Q in: qvq* = Qv
 * q = [cos(t/2), sin(t/2)v] where |v| = 1 and v is the rotation axis. t is wanted angle rotation. For some point
 * v, the achieved rotation is calculated as qvq*.
 */
void quat_to_mat3(mat3 dst, const quat q);
void quat_to_mat4(mat4 dst, const quat q);

#ifdef __cplusplus
} 
#endif

#endif

/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#include "quaternion.h"

void quat_set(quat dst, const f32 x, const f32 y, const f32 z, const f32 w)
{
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
	dst[3] = w;
}

void quat_add(quat dst, const quat p, const quat q)
{
	dst[0] = p[0] + q[0];
	dst[1] = p[1] + q[1];
	dst[2] = p[2] + q[2];
	dst[3] = p[3] + q[3];
}

void quat_translate(quat dst, const quat translation)
{
	dst[0] += translation[0]; 
	dst[1] += translation[1];
	dst[2] += translation[2];
	dst[3] += translation[3];
}

void quat_sub(quat dst, const quat p, const quat q)
{
	dst[0] = p[0] - q[0];
	dst[1] = p[1] - q[1];
	dst[2] = p[2] - q[2];
	dst[3] = p[3] - q[3];
}

void quat_mult(quat dst, const quat p, const quat q)
{
	dst[0] = p[0] * q[3] + p[3] * q[0] + p[1] * q[2] - p[2] * q[1];
	dst[1] = p[1] * q[3] + p[3] * q[1] + p[2] * q[0] - p[0] * q[2];
	dst[2] = p[2] * q[3] + p[3] * q[2] + p[0] * q[1] - p[1] * q[0];
	dst[3] = p[3] * q[3] - p[0] * q[0] - p[1] * q[1] - p[2] * q[2];
}

void quat_scale(quat dst, const f32 scale)
{
	dst[0] = dst[0] * scale;
	dst[1] = dst[1] * scale;
	dst[2] = dst[2] * scale;
	dst[3] = dst[3] * scale;
}

void quat_copy(quat dst, const quat q)
{
	dst[0] = q[0];
	dst[1] = q[1];
	dst[2] = q[2];
	dst[3] = q[3];
}

void quat_conj(quat conj, const quat q)
{
	conj[0] = -q[0];
	conj[1] = -q[1];
	conj[2] = -q[2];
	conj[3] = q[3];
}

f32 quat_norm(const quat q)
{
	return f32_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void quat_inv(quat inv, const quat q)
{
	f32 norm_2_inv = 1.0f / quat_norm(q);
	quat_conj(inv, q);
	quat_scale(inv, norm_2_inv);
}

void quat_normalize(quat q)
{
	f32 norm_2_inv = 1.0f / quat_norm(q);
	quat_scale(q, norm_2_inv);
}

/**
 * CCW rot?
 */
void quat_to_mat3(mat3 dst, const quat q)
{
	const f32 tr_part = 2.0f*q[3]*q[3] - 1.0f;
	const f32 q12 = 2.0f*q[0]*q[1];
	const f32 q13 = 2.0f*q[0]*q[2];
	const f32 q10 = 2.0f*q[0]*q[3];
	const f32 q23 = 2.0f*q[1]*q[2];
	const f32 q20 = 2.0f*q[1]*q[3];
	const f32 q30 = 2.0f*q[2]*q[3];
	mat3_set(dst, tr_part + 2.0f*q[0]*q[0], q12 + q30, q13 - q20,
		      q12 - q30, tr_part + 2.0f*q[1]*q[1], q23 + q10,
		      q13 + q20, q23 - q10, tr_part + 2.0f*q[2]*q[2]);
}

/**
 * q is a normalised quaternion representing a CCW rotation.
 */
void quat_to_mat4(mat4 dst, const quat q)
{
	const f32 tr_part = 2.0f*q[3]*q[3] - 1.0f;
	const f32 q12 = 2.0f*q[0]*q[1];
	const f32 q13 = 2.0f*q[0]*q[2];
	const f32 q10 = 2.0f*q[0]*q[3];
	const f32 q23 = 2.0f*q[1]*q[2];
	const f32 q20 = 2.0f*q[1]*q[3];
	const f32 q30 = 2.0f*q[2]*q[3];
	mat4_set(dst, tr_part + 2.0f*q[0]*q[0], q12 + q30, q13 - q20, 0.0f,
		      q12 - q30, tr_part + 2.0f*q[1]*q[1], q23 + q10, 0.0f,
		      q13 + q20, q23 - q10, tr_part + 2.0f*q[2]*q[2], 0.0f, 
		      0.0f, 0.0f, 0.0f, 1.0f);
}

void axis_angle_to_quaternion(quat dst, const vec3 axis, const f32 angle)
{
	const f32 scale = f32_sin(angle/2.0f) / Vec3Length(axis);
	quat_set(dst, scale * axis[0], scale * axis[1], scale * axis[2], f32_cos(angle/2.0f));
}

void unit_axis_angle_to_quaternion(quat dst, const vec3 axis, const f32 angle)
{
	const f32 scale = f32_sin(angle/2.0f);
	quat_set(dst, scale * axis[0], scale * axis[1], scale * axis[2], f32_cos(angle/2.0f));
}

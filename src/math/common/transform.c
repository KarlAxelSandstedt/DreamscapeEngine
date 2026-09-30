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

#include "ds_base.h"
#include "transform.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_matrix.h"

m3 M3SequentialRotation(const v3 axis_1, const f32 angle_1, const v3 axis_2, const f32 angle_2)
{
	const m3 r_1 = M3Rotation(axis_1, angle_1);
	const v3 axis_snd = M3V3Mul(r_1, axis_2);
	const m3 r_2 = M3Rotation(axis_snd, angle_2);
	return M3Mul(r_2, r_1);
}

m3 M3Rotation(const v3 axis, const f32 angle)
{
	return M3Q(QAxisAngle(axis, angle));
}

v3 V3RotateCenter(const m3 rotation, const v3 center, const v3 src)
{
	return V3Add(M3V3Mul(rotation, V3Sub(src, center)), center);
}

m4 M4Perspective(const f32 aspect_ratio, const f32 fov_x, const f32 fz_near, const f32 fz_far)
{
	return M4(1.0f / F32Tan(fov_x / 2.0f), 0.0f, 0.0f, 0.0f,
	          0.0f, aspect_ratio / F32Tan(fov_x / 2.0f), 0.0f, 0.0f,
		  0.0f, 0.0f, (fz_near + fz_far) / (fz_near - fz_far), -1.0f,
		  0.0f, 0.0f, (2.0f * fz_near * fz_far) / (fz_near - fz_far), 0.0f);
}

m4 M4View(const v3 position, const v3 left, const v3 up, const v3 forward)
{
	/**
	 * (1) Translation to camera center 
	 * (2) Change to Camera basis
	 * (3) anything infront of camera must be reflected in x,z values againt (0,0), since
	 * Opengl expects camera looking down -Z axis, so mult left, and forward axes by (-1) .
	 */
	const m4 basis_change = M4(-left.x, up.x, -forward.x, 0.0f,
				   -left.y, up.y, -forward.y, 0.0f,
				   -left.z, up.z, -forward.z, 0.0f,
				   0.0f, 0.0f, 0.0f, 1.0f);
	const m4 translation = M4(1.0f, 0.0f, 0.0f, 0.0f,
				  0.0f, 1.0f, 0.0f, 0.0f,
				  0.0f, 0.0f, 1.0f, 0.0f,
				  -position.x, -position.y, -position.z, 1.0f);
	return M4Mul(basis_change, translation);
}

m4 M4ViewLookAt(const v3 position, const v3 target)
{
	v3 relative = V3Sub(target, position);
	v3 dir = V3Normalize(relative);
	const f32 pitch = F32_PI / 2.0f - F32Acos(V3Dot(V3(0.0f, 1.0f, 0.0f), dir));

	relative.y = 0.0f;
	dir = V3Normalize(relative);

	f32 yaw;
	if (dir.z < 0.0f) {
		yaw  = F32Acos(V3Dot(V3(1.0f, 0.0f, 0.0f), dir));	
	} else {
		yaw  = -F32Acos(V3Dot(V3(1.0f, 0.0f, 0.0f), dir));	
	}
	return M4ViewYawPitch(position, yaw, pitch);
}

m4 M4ViewYawPitch(const v3 position, const f32 yaw, const f32 pitch)
{
	const f32 cy = F32Cos(yaw / 2.0f);
	const f32 cp = F32Cos(pitch / 2.0f);
	const f32 sy = F32Sin(yaw / 2.0f);
	const f32 sp = F32Sin(pitch / 2.0f);
	const m3 rot = M3Q(Q(sy*sp, sy*cp, cy*sp, cy*cp));

	/* Assume no rotation is equivalent to looking down positive x-axis */
	const v3 left = M3V3Mul(rot, V3(0.0f, 0.0f, -1.0f));
	const v3 up = M3V3Mul(rot, V3(0.0f, 1.0f, 0.0f));
	const v3 forward = M3V3Mul(rot, V3(1.0f, 0.0f, 0.0f));

	return M4View(position, left, up, forward);
}

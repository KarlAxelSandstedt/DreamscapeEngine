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

/*
transform.h
===========
Rotation matrices from axes and angles, and the camera's view and projection matrices in OpenGL's conventions
(camera space looks down -z, clip space z/w in [-1, 1]). Angles are in radians. Everything here goes through the
C library's trigonometric functions (directly or through QAxisAngle), so results may differ in the last bit
between platforms: the renderer and the editor use these, the simulation does not. Source:
src/math/common/transform.c. Included by ds_math.h.
*/

#ifndef __DS_MATH_TRANSFORMATION__
#define __DS_MATH_TRANSFORMATION__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_types.h"

/*
 * rotation matrix of axis_1(angle_1) (R) -> [R(axis_2)](angle_2): rotate by angle_1 about axis_1, then by angle_2
 * about axis_2 as turned by the first rotation (body-fixed axes); R(axis_1, angle_1) R(axis_2, angle_2) up to rounding.
 */
m3 	M3SequentialRotation(const v3 axis_1, const f32 angle_1, const v3 axis_2, const f32 angle_2); 
/* rotation matrix by angle about axis (right-hand rule); axis may have any nonzero length (a zero axis gives NaN) */
m3 	M3Rotation(const v3 axis, const f32 angle);
/* src rotated by rotation about the point center */
v3 	V3RotateCenter(const m3 rotation, const v3 center, const v3 src);

/*
 * Projection for a horizontal field of view fov_x, aspect_ratio = width / height and the near and far distances
 * 0 < fz_near < fz_far: a camera-space point at z = -fz_near gets clip z/w = -1, at z = -fz_far +1, and
 * x = +-tan(fov_x / 2) |z| gets x/w = +-1 (y: the same over aspect_ratio).
 */
m4 	M4Perspective(const f32 aspect_ratio, const f32 fov_x, const f32 fz_near, const f32 fz_far);

/*
 * World space to camera space for a camera at position with the orthonormal directions left, up and forward
 * (left = up x forward): position goes to the origin, forward to -z, up to +y, left to -x. The camera's frame from
 * yaw and pitch: r_CameraUpdateAxes (r_camera.c; convention in ds_math.h).
 */
m4 	M4View(const v3 position, const v3 left, const v3 up, const v3 forward);

#ifdef __cplusplus
} 
#endif

#endif

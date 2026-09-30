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

#ifndef __DS_MATH_TRANSFORMATION__
#define __DS_MATH_TRANSFORMATION__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_types.h"

/* axes should be normalized */
/* rotation matrix of axis_1(angle_1) (R) -> [R(axis_2)](angle_2) */
m3 	M3SequentialRotation(const v3 axis_1, const f32 angle_1, const v3 axis_2, const f32 angle_2); 
m3 	M3Rotation(const v3 axis, const f32 angle);
v3 	V3RotateCenter(const m3 rotation, const v3 center, const v3 src);

m4 	M4Perspective(const f32 aspect_ratio, const f32 fov_x, const f32 fz_near, const f32 fz_far);

m4 	M4View(const v3 position, const v3 left, const v3 up, const v3 forward);
m4 	M4ViewLookAt(const v3 position, const v3 target);
m4 	M4ViewYawPitch(const v3 position, const f32 yaw, const f32 pitch);

#ifdef __cplusplus
} 
#endif

#endif

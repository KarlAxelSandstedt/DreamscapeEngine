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

#include "matrix.h"

/* axes should be normalized */
void 	sequential_rotation_matrix(mat3 dst, const vec3 axis_1, const f32 angle_1, const vec3 axis_2, const f32 angle_2); /* rotation matrix of axis_1(angle_1) (R) -> [R(axis_2)](angle_2) */
void 	perspective_matrix(mat4 dst, const f32 aspect_ratio, const f32 fov_x, const f32 fz_near, const f32 fz_far);
void 	view_matrix(mat4 dst, const vec3 position, const vec3 left, const vec3 up, const vec3 forward);
void 	view_matrix_look_at(mat4 dst, const vec3 position, const vec3 target);
void 	view_matrix_yaw_pitch(mat4 dst, const vec3 position, const f32 yaw, const f32 pitch);
void 	rotation_matrix(mat3 dst, const vec3 axis, const f32 angle);
void 	vec3_rotate_center(vec3 src_rotated, mat3 rotation, const vec3 center, const vec3 src);

#ifdef __cplusplus
} 
#endif

#endif

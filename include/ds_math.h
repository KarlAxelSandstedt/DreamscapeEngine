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

#ifndef __DS_MATH_H__
#define __DS_MATH_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_random.h"
#include "float32.h"
#include "vector.h"
#include "quaternion.h"
#include "matrix.h"

typedef struct
{
	quat	rotation;
	vec3	position;
} ds_Transform;

#include "transform.h"
#include "geometry.h"

static inline ds_Transform ds_TransformIdentity(void)
{
    ds_Transform transform = 
    { 
        .position = { 0.0f, 0.0f, 0.0f },
    };
    QuatIdentity(transform.rotation);
    return transform;
}

#ifdef __cplusplus
} 
#endif

#endif

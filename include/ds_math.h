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

/* Return the relative transform of target to reference:
 *
 *      ref->rot*rel->rot = ref->rot*ref->rot_inv*T->rot = target->rot
 *
 *      ref->rot*rel->pos + ref->pos = ref->rot*ref->rot_inv*(T->pos - ref->pos) + ref->pos = target->pos
 */
static inline ds_Transform ds_TransformRelative(const ds_Transform *reference, const ds_Transform *target)
{
    ds_Transform relative;
    quat inv;

    QuatInverse(inv, reference->rotation);
    QuatMul(relative.rotation, inv, target->rotation);
    Vec3Sub(relative.position, target->position, reference->position);
    QuatVec3RotateSelf(relative.position, inv);

    return relative;
}

/* Return the local position of target within reference frame:
 *
 *      ref->rot*rel + ref->pos = ref->rot*ref->rot_inv*(target - ref->pos) + ref->pos = target
 */
static inline void ds_TransformPointToLocal(vec3 local, const ds_Transform *reference, const vec3 target)
{
    quat inv;
    QuatInverse(inv, reference->rotation);
    Vec3Sub(local, target, reference->position);
    QuatVec3RotateSelf(local, inv);
}

#ifdef __cplusplus
} 
#endif

#endif

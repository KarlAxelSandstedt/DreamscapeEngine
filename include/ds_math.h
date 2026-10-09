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
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_matrix.h"

typedef struct
{
	q	rotation;
	v3	position;
} ds_Transform;

/* World pose: f64 position (far from the origin), f32 rotation (frame local, needs no extra range). */
typedef struct
{
	q	    rotation;
	v3f64	position;
} ds_TransformWorld;

#include "transform.h"
#include "geometry.h"

static inline ds_Transform ds_TransformIdentity(void)
{
    const ds_Transform transform = 
    { 
        .rotation = QIdentity(),
        .position = V3Zero(),
    };
    return transform;
}

/*
 * Return target's pose in reference's frame (reference^-1 * target): rel maps points from target's frame
 * into reference's frame.
 *
 *      p_reference = rel.rot*p_target + rel.pos
 *      rel.rot     = reference.rot^-1 * target.rot
 *      rel.pos     = reference.rot^-1 * (target.pos - reference.pos)
 */
static inline ds_Transform ds_TransformRelative(const ds_Transform *reference, const ds_Transform *target)
{
    const q inv = QInverse(reference->rotation);
    const ds_Transform relative =
    {
        .rotation = QMul(inv, target->rotation),
        .position = QV3Rotate(inv, V3Sub(target->position, reference->position)),
    };

    return relative;
}

/* Return the local position of target within reference frame:
 *
 *      ref->rot*rel + ref->pos = ref->rot*ref->rot_inv*(target - ref->pos) + ref->pos = target
 */
static inline v3 ds_TransformPointToLocal(const ds_Transform *reference, const v3 target)
{
    return QV3Rotate(QInverse(reference->rotation), V3Sub(target, reference->position));
}

/*
 * World transforms. Positions cross between f64 (world) and f32 (frames) only through these: world
 * positions are subtracted in f64 before demoting, so the f32 rounding is relative to the frames'
 * separation, not to their distance from the origin.
 */

static inline ds_TransformWorld ds_TransformWorldIdentity(void)
{
    const ds_TransformWorld transform = 
    { 
        .rotation = QIdentity(),
        .position = V3F64(0.0, 0.0, 0.0),
    };
    return transform;
}

/* lossless */
static inline ds_TransformWorld ds_TransformPromote(const ds_Transform *transform)
{
    const ds_TransformWorld world =
    {
        .rotation = transform->rotation,
        .position = V3Promote(transform->position),
    };
    return world;
}

/*
 * Return target's pose in reference's frame (reference^-1 * target): rel maps points from target's frame
 * into reference's frame. Positions are subtracted in f64, then demoted.
 *
 *      p_reference = rel.rot*p_target + rel.pos
 *      rel.rot     = reference.rot^-1 * target.rot
 *      rel.pos     = reference.rot^-1 * demote(target.pos - reference.pos)
 */
static inline ds_Transform ds_TransformWorldRelative(const ds_TransformWorld *reference, const ds_TransformWorld *target)
{
    const q inv = QInverse(reference->rotation);
    const ds_Transform relative =
    {
        .rotation = QMul(inv, target->rotation),
        .position = QV3Rotate(inv, V3F64Demote(V3F64Sub(target->position, reference->position))),
    };
    return relative;
}

/*
 * Return the world pose of child, given in parent's frame: points go through child, then parent.
 *
 *      result(p) = parent.rot*(child.rot*p + child.pos) + parent.pos
 */
static inline ds_TransformWorld ds_TransformWorldChild(const ds_TransformWorld *parent, const ds_Transform *child)
{
    const ds_TransformWorld result =
    {
        .rotation = QMul(parent->rotation, child->rotation),
        .position = V3F64Add(parent->position, V3Promote(QV3Rotate(parent->rotation, child->position))),
    };
    return result;
}

/* Return the local position of the world point target within reference's frame. */
static inline v3 ds_TransformWorldPointToLocal(const ds_TransformWorld *reference, const v3f64 target)
{
    return QV3Rotate(QInverse(reference->rotation), V3F64Demote(V3F64Sub(target, reference->position)));
}

/* Return the world position of the point local, given in reference's frame. */
static inline v3f64 ds_TransformWorldPointToWorld(const ds_TransformWorld *reference, const v3 local)
{
    return V3F64Add(reference->position, V3Promote(QV3Rotate(reference->rotation, local)));
}

#ifdef __cplusplus
} 
#endif

#endif

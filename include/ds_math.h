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
ds_math.h
=========
The math headers in one include (ds_random.h, ds_float.h, ds_vector.h, ds_quaternion.h, ds_matrix.h, transform.h,
ds_geometry.h), and poses. A pose maps points from its frame to its parent's frame: p_parent = rot p + pos, rot a
unit quaternion (not checked). ds_Transform has an f32 position, for frames near each other; ds_TransformWorld an
f64 position, for placing frames in the world, precise far from the origin. Positions cross between the two only
through the functions here: world positions are subtracted in f64 before the demotion, so the f32 rounding is
relative to the frames' separation, not to their distance from the origin. Plain IEEE arithmetic: the same bits
on every platform. src/math/common/ds_math.c is the math group's unity file.

Conventions (the whole engine)
------------------------------
    Units       1.0f = 1 m (DS_UNIT_M); physics time in seconds; angles in radians.
    Axes        right-handed, +Y up: gravity is (0, -g, 0).
    Rotations   unit quaternions q = (x, y, z, w), w the scalar part, Hamilton product (ij = k). A positive angle
                turns counterclockwise about its axis (right-hand rule). QMul(a, b) rotates by b first, then a.
    Matrices    column-major (aij: row i, column j); M2/M3/M4(...) take their arguments column by column;
                M3V3Mul(m, v) = m v, V3M3Mul(v, m) = v^T m.
    Frames      a pose maps its frame into its parent's (p_parent = rot p + pos). A body's frame has its origin at
                the body's position, not at its center of mass (local_center_of_mass, in the body frame); shapes
                sit in the body frame by their t_local. World positions are f64 (ds_TransformWorld).
    Winding     triangles and dcel faces are counterclockwise about their outward normal; mesh collision is
                two-sided (the winding orients render normals). GL culls back faces, front = CCW.
    Camera      the camera frame is left, up, forward with left = up x forward. View space is OpenGL's (looking
                down -z), clip space z/w in [-1, 1] (transform.h). The camera's yaw and pitch (r_camera.c,
                r_CameraUpdateAxes): at yaw = pitch = 0 it looks along +z with left +x; positive yaw turns forward
                toward +x, positive pitch looks down; pitch is clamped short of straight up/down.
    UI          window coordinates have the bottom-left corner at (0, 0), y up.

Sections: local poses, world poses; inline definitions at the bottom, in the same order.
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
#include "ds_geometry.h"

/*********************************** local poses ************************************/

/* Return the identity pose */
static inline ds_Transform ds_TransformIdentity(void);

/*
 * Return target's pose in reference's frame (reference^-1 * target): rel maps points from target's frame
 * into reference's frame.
 *
 *      p_reference = rel.rot*p_target + rel.pos
 *      rel.rot     = reference.rot^-1 * target.rot
 *      rel.pos     = reference.rot^-1 * (target.pos - reference.pos)
 */
static inline ds_Transform ds_TransformRelative(const ds_Transform *reference, const ds_Transform *target);

/*
 * Return the local position of target within reference's frame:
 *
 *      ref->rot*rel + ref->pos = ref->rot*ref->rot_inv*(target - ref->pos) + ref->pos = target
 */
static inline v3 ds_TransformPointToLocal(const ds_Transform *reference, const v3 target);

/*********************************** world poses ************************************/

/*
 * World transforms. Positions cross between f64 (world) and f32 (frames) only through these: world
 * positions are subtracted in f64 before demoting, so the f32 rounding is relative to the frames'
 * separation, not to their distance from the origin.
 */

/* Return the identity world pose (at the world origin) */
static inline ds_TransformWorld ds_TransformWorldIdentity(void);

/* Return transform as a world pose (lossless) */
static inline ds_TransformWorld ds_TransformPromote(const ds_Transform *transform);

/*
 * Return target's pose in reference's frame (reference^-1 * target): rel maps points from target's frame
 * into reference's frame. Positions are subtracted in f64, then demoted.
 *
 *      p_reference = rel.rot*p_target + rel.pos
 *      rel.rot     = reference.rot^-1 * target.rot
 *      rel.pos     = reference.rot^-1 * demote(target.pos - reference.pos)
 */
static inline ds_Transform ds_TransformWorldRelative(const ds_TransformWorld *reference, const ds_TransformWorld *target);

/*
 * Return the world pose of child, given in parent's frame: points go through child, then parent.
 *
 *      result(p) = parent.rot*(child.rot*p + child.pos) + parent.pos
 */
static inline ds_TransformWorld ds_TransformWorldChild(const ds_TransformWorld *parent, const ds_Transform *child);

/* Return the local position of the world point target within reference's frame */
static inline v3 ds_TransformWorldPointToLocal(const ds_TransformWorld *reference, const v3f64 target);

/* Return the world position of the point local, given in reference's frame */
static inline v3f64 ds_TransformWorldPointToWorld(const ds_TransformWorld *reference, const v3 local);

/************************* local poses: inline definitions **************************/

static inline ds_Transform ds_TransformIdentity(void)
{
    const ds_Transform transform = 
    { 
        .rotation = QIdentity(),
        .position = V3Zero(),
    };
    return transform;
}

static inline ds_Transform ds_TransformRelative(const ds_Transform *reference, const ds_Transform *target)
{
    const q inv = QConj(reference->rotation);
    const ds_Transform relative =
    {
        .rotation = QMul(inv, target->rotation),
        .position = QUnitRotate(inv, V3Sub(target->position, reference->position)),
    };

    return relative;
}

static inline v3 ds_TransformPointToLocal(const ds_Transform *reference, const v3 target)
{
    return QUnitRotate(QConj(reference->rotation), V3Sub(target, reference->position));
}

/************************* world poses: inline definitions **************************/

static inline ds_TransformWorld ds_TransformWorldIdentity(void)
{
    const ds_TransformWorld transform = 
    { 
        .rotation = QIdentity(),
        .position = V3F64(0.0, 0.0, 0.0),
    };
    return transform;
}

static inline ds_TransformWorld ds_TransformPromote(const ds_Transform *transform)
{
    const ds_TransformWorld world =
    {
        .rotation = transform->rotation,
        .position = V3Promote(transform->position),
    };
    return world;
}

static inline ds_Transform ds_TransformWorldRelative(const ds_TransformWorld *reference, const ds_TransformWorld *target)
{
    const q inv = QConj(reference->rotation);
    const ds_Transform relative =
    {
        .rotation = QMul(inv, target->rotation),
        .position = QUnitRotate(inv, V3F64Demote(V3F64Sub(target->position, reference->position))),
    };
    return relative;
}

static inline ds_TransformWorld ds_TransformWorldChild(const ds_TransformWorld *parent, const ds_Transform *child)
{
    const ds_TransformWorld result =
    {
        .rotation = QMul(parent->rotation, child->rotation),
        .position = V3F64Add(parent->position, V3Promote(QUnitRotate(parent->rotation, child->position))),
    };
    return result;
}

static inline v3 ds_TransformWorldPointToLocal(const ds_TransformWorld *reference, const v3f64 target)
{
    return QUnitRotate(QConj(reference->rotation), V3F64Demote(V3F64Sub(target, reference->position)));
}

static inline v3f64 ds_TransformWorldPointToWorld(const ds_TransformWorld *reference, const v3 local)
{
    return V3F64Add(reference->position, V3Promote(QUnitRotate(reference->rotation, local)));
}

#ifdef __cplusplus
} 
#endif

#endif

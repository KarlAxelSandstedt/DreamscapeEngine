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

#include "ds_dynamics.h"
#include "collision.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"

u32 c_SphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_SPHERE && s1->type == C_SHAPE_SPHERE);
    const f32 r_sum = s0->sphere.radius + s1->sphere.radius;
    const v3 delta = V3F64Demote(V3F64Sub(t1->position, t0->position));
    return V3LengthSquared(delta) <= r_sum*r_sum;
}

u32 c_CapsuleSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CAPSULE);
    ds_Assert(s1->type == C_SHAPE_SPHERE);

    const struct capsule *cap = &s0->capsule;
    const f32 r_sum = cap->radius + s1->sphere.radius;
    const v3 axis = QUnitRotate(t0->rotation, V3(0.0f, cap->half_height, 0.0f));
    const struct segment seg = SegmentConstruct(axis, V3Negate(axis));
    const v3 p = V3F64Demote(V3F64Sub(t1->position, t0->position));

    v3 c;
    return SegmentPointDistanceSquared(&c, &seg, p) <= r_sum*r_sum;
}

u32 c_CapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CAPSULE);
    ds_Assert(s1->type == C_SHAPE_CAPSULE);

    const f32 r_sum = s0->capsule.radius + s1->capsule.radius;
    const v3 axis0 = QUnitRotate(t0->rotation, V3(0.0f, s0->capsule.half_height, 0.0f));
    const v3 axis1 = QUnitRotate(t1->rotation, V3(0.0f, s1->capsule.half_height, 0.0f));
    const v3 p1 = V3F64Demote(V3F64Sub(t1->position, t0->position));
    const struct segment seg0 = SegmentConstruct(axis0, V3Negate(axis0));
    const struct segment seg1 = SegmentConstruct(V3Add(p1, axis1), V3Sub(p1, axis1));

    v3 q0, q1;
    return SegmentDistanceSquared(&q0, &q1, &seg0, &seg1) <= r_sum*r_sum;
}

u32 c_HullSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s1->type == C_SHAPE_SPHERE);

    v3f64 c0, c1;
    v3 n;
    struct GJKCache cache;
    return GJKWorld(&c0, &c1, &n, &cache, NULL, s0, t0, s1, t1, s1->sphere.radius) <= s1->sphere.radius;
}

u32 c_HullCapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s1->type == C_SHAPE_CAPSULE);

    v3f64 c0, c1;
    v3 n;
    struct GJKCache cache;
    return GJKWorld(&c0, &c1, &n, &cache, NULL, s0, t0, s1, t1, s1->capsule.radius) <= s1->capsule.radius;
}

u32 c_HullTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);

    v3f64 c0, c1;
    v3 n;
    struct GJKCache cache;
    return GJKWorld(&c0, &c1, &n, &cache, NULL, s0, t0, s1, t1, 0.0f) == 0.0f;
}

u32 c_TriMeshBvhSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    v3f64 c0, c1;
    return c_TriMeshBvhSphereDistance(&c0, &c1, s0, t0, s1, t1) == 0.0f;
}

u32 c_TriMeshBvhCapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    v3f64 c0, c1;
    return c_TriMeshBvhCapsuleDistance(&c0, &c1, s0, t0, s1, t1) == 0.0f;
}

u32 c_TriMeshBvhHullTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    v3f64 c0, c1;
    return c_TriMeshBvhHullDistance(&c0, &c1, s0, t0, s1, t1) == 0.0f;
}

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

/********************************** INTERSECTION TESTS **********************************/

/*
 * The analytic tests compare squared core distances relative to t1's position (one f64 subtraction,
 * see c_distance.c). The hull tests run GJK with the radius as cutoff_distance, so a separating plane
 * ends the search early; the answer is the same as comparing the full distance.
 */

u32 c_SphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_SPHERE && s2->type == C_SHAPE_SPHERE);
    const f32 r_sum = s1->sphere.radius + s2->sphere.radius;
    const v3 delta = V3F64Demote(V3F64Sub(t2->position, t1->position));
    return V3LengthSquared(delta) <= r_sum*r_sum;
}

u32 c_CapsuleSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

    const struct capsule *cap = &s1->capsule;
    const f32 r_sum = cap->radius + s2->sphere.radius;
    const v3 axis = QV3Rotate(t1->rotation, V3(0.0f, cap->half_height, 0.0f));
    const struct segment seg = SegmentConstruct(axis, V3Negate(axis));
    const v3 p = V3F64Demote(V3F64Sub(t2->position, t1->position));

    v3 c;
    return SegmentPointDistanceSquared(&c, &seg, p) <= r_sum*r_sum;
}

u32 c_CapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_CAPSULE);

    const f32 r_sum = s1->capsule.radius + s2->capsule.radius;
    const v3 axis1 = QV3Rotate(t1->rotation, V3(0.0f, s1->capsule.half_height, 0.0f));
    const v3 axis2 = QV3Rotate(t2->rotation, V3(0.0f, s2->capsule.half_height, 0.0f));
    const v3 p2 = V3F64Demote(V3F64Sub(t2->position, t1->position));
    const struct segment seg1 = SegmentConstruct(axis1, V3Negate(axis1));
    const struct segment seg2 = SegmentConstruct(V3Add(p2, axis2), V3Sub(p2, axis2));

    v3 q1, q2;
    return SegmentDistanceSquared(&q1, &q2, &seg1, &seg2) <= r_sum*r_sum;
}

u32 c_HullSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

    v3f64 c1, c2;
    v3 n;
    struct GJKCache cache;
    return GJK(&c1, &c2, &n, &cache, NULL, s1, t1, s2, t2, s2->sphere.radius) <= s2->sphere.radius;
}

u32 c_HullCapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s2->type == C_SHAPE_CAPSULE);

    v3f64 c1, c2;
    v3 n;
    struct GJKCache cache;
    return GJK(&c1, &c2, &n, &cache, NULL, s1, t1, s2, t2, s2->capsule.radius) <= s2->capsule.radius;
}

u32 c_HullTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
    ds_Assert(s2->type == C_SHAPE_CONVEX_HULL);

    v3f64 c1, c2;
    v3 n;
    struct GJKCache cache;
    return GJK(&c1, &c2, &n, &cache, NULL, s1, t1, s2, t2, 0.0f) == 0.0f;
}

u32 c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    v3f64 c1, c2;
    return c_TriMeshBvhSphereDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    v3f64 c1, c2;
    return c_TriMeshBvhCapsuleDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2)
{
    v3f64 c1, c2;
    return c_TriMeshBvhHullDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

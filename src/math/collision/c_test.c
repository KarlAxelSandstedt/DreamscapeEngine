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

u32 c_SphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_SPHERE && s2->type == C_SHAPE_SPHERE);
	const f32 r_sum = s1->sphere.radius + s2->sphere.radius;
	return V3DistanceSquared(t1->position, t2->position) <= r_sum*r_sum;
}

u32 c_CapsuleSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	const v3 c2 = V3Sub(t2->position, t1->position);

    const struct capsule *cap = &s1->capsule;
    const v3 s_p1 = QV3Rotate(t1->rotation, V3(0.0f, cap->half_height, 0.0f));
	const struct segment s = SegmentConstruct(s_p1, V3Negate(s_p1));

	const f32 r_sum = s1->capsule.radius + s2->sphere.radius;
	v3 c1;
	return SegmentPointDistanceSquared(&c1, &s, c2) <= r_sum*r_sum;
}

u32 c_CapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_CapsuleDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_HullSphereDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_HullCapsuleDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_HullDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_TriMeshBvhSphereDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_TriMeshBvhCapsuleDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	v3 c1, c2;
	return c_TriMeshBvhHullDistance(&c1, &c2, s1, t1, s2, t2) == 0.0f;
}

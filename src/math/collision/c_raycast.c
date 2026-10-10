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

/********************************** RAYCAST **********************************/

f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
    /*
            Notes from Ray Tracing Gems (2019), ch. 7, "Precision Improvements for Ray/Sphere
            Intersection" (Haines, Guenther, Akenine-Moeller); discriminant after Hearn & Baker.
            C*: center, r: radius, O: ray origin, D: ray direction
            (C* - (O + t*D)).(C* - (O + t*D)) = r^2

        Relative to the ray origin:
            C = C* - O
            (C - t*D).(C - t*D) = r^2
            t^2 (D.D) - 2t (D.C) + (C.C - r^2) = 0

        Yields:
            a = D.D, b = -2 (D.C), c = C.C - r^2

            t0,1 = (-b +- sqrt(b^2 - 4ac)) / 2a
                 = ( (D.C) +- sqrt((D.C)^2 - (D.D)(C.C - r^2)) ) / (D.D)
                 = ( (D.C) +- sqrt( discriminant ) ) / (D.D)

        Issue 1: term inside sqrt deals with squares **before** subtraction.
        Consider a small sphere far away from the origin:

            C.C >>> r^2

        Squaring turns the ratio |C|/r into (|C|/r)^2: r^2 falls below the rounding step of C.C
        and is lost in the subtraction, which means significant precision loss.

        if |C| = 8192*r = 2^13*r, then C.C = 2^26*r^2; f32 values near it are 8*r^2 apart, so

                C.C - r^2 = C.C        (r^2 < half a step: the sphere has radius 0)

        Hearn's discriminant. By Lagrange's identity (D.C)^2 - (D.D)(C.C) = -|C x D|^2, so

                discriminant = (D.C)^2 - (D.D)(C.C - r^2)
                             = D.D*r^2 + (D.C)^2 - D.D*C.C
                             = D.D*r^2 -|C x D|^2

        To understand why D.D*r^2 - |C x D|^2 gives a more accurate difference:

            P = ((D.C)/(D.D))*D     O to the ray line's closest point to C* (C projected onto D)
            H = C - P               that point to C*, perpendicular to D

            |C x D|^2 = (D.D)(C.C) - (D.C)^2 = (D.D)(C.C - |P|^2)
                      = (D.D)|H|^2,                 (Pythagoras: C.C = |P|^2 + |H|^2)

            Thus |C x D| = |D||H|: the distance between the ray's line and the center times the
            length of the direction. For a hit |H| <= r, so both terms are D.D times squares of
            sphere-sized lengths (r^2 and |H|^2), never |C|^2.

    */
	ds_Assert(shape->type == C_SHAPE_SPHERE);
    ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

    const f32 dd = V3Dot(ray->dir, ray->dir);
    const f32 r = shape->sphere.radius;
    const v3 c = V3F64Demote(V3F64Sub(transform->position, ray->origin));
    const f32 discriminant = dd*r*r - V3LengthSquared(V3Cross(c, ray->dir));
    /* No real solutions */
    if (discriminant < 0.0f)
    {
        return F32_INFINITY;
    }

    const f32 cd = V3Dot(c, ray->dir);
    const f32 discriminant_sqrt = F32Sqrt(discriminant);
    const f32 t_max = cd + discriminant_sqrt;
    /* sphere behind the origin */
    if (t_max < 0.0f)
    {
        return F32_INFINITY;
    }

    const f32 t_min = cd - discriminant_sqrt;
    return (0.0f <= t_min)
        ? t_min / dd
        : t_max / dd;
}

f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_CAPSULE);
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);
	/* TODO: refactor in the shape's frame; the old world f32 values for now */
	const ds_Transform t = { .rotation = transform->rotation, .position = V3F64Demote(transform->position) };
	const struct ray ray_f32 = { .origin = V3F64Demote(ray->origin), .dir = ray->dir };

	const f32 r = shape->capsule.radius;
	struct segment s = SegmentCapsuleTransform(&shape->capsule, &t);

	v3 p0, p1;
	const f32 dist_sq = RaySegmentDistanceSquared(&p0, &p1, &ray_f32, &s);
	if (dist_sq > r*r) { return F32_INFINITY; }

	struct sphere sph = SphereConstruct(p1, r);
	return SphereRaycastParameter(&sph, &ray_f32);
}

f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

	/* TODO: refactor in the shape's frame; the old world f32 values for now */
	const v3 position = V3F64Demote(transform->position);
	const struct ray ray_f32 = { .origin = V3F64Demote(ray->origin), .dir = ray->dir };
	const m3 rot = M3Q(transform->rotation);
	const struct dcel *h = &shape->hull;
	f32 t_best = F32_INFINITY;

	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		struct plane pl = DcelFacePlane(h, rot, position, fi);
		const f32 t = PlaneRaycastParameter(&pl, &ray_f32);
		if (t < t_best && t >= 0.0f)
		{
			const v3 p = RayPoint(&ray_f32, t);
			if (DcelFaceProjectedPointTest(h, rot, position, fi, p))
			{
				t_best = t;
			}
		}
	}

	return t_best;
}

f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_TRI_MESH);
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);
	const m3 inv_rot = M3Q(QInverse(transform->rotation));

	const struct triMeshBvh *mesh_bvh = &shape->mesh_bvh;
	struct ray rotated_ray;
	/* TODO: refactor in the shape's frame; the old world f32 values for now */
	rotated_ray.origin = M3V3Mul(inv_rot, V3Sub(V3F64Demote(ray->origin), V3F64Demote(transform->position)));
	rotated_ray.dir = M3V3Mul(inv_rot, ray->dir);

    struct arena *mem_tmp = ArenaPushScratch();
	const f32 t = TriMeshBvhRaycast(mem_tmp, mesh_bvh, &rotated_ray).f;
    ArenaPopScratch();

    return t;
}

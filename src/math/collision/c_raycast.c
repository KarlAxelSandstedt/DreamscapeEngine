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

f32 SphereRaycastParameter(const struct sphere *sphere, const struct ray *ray)
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

        In the sphere's frame C* = 0, so C = -O.
    */
    ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

    const f32 dd = V3Dot(ray->dir, ray->dir);
    const f32 r = sphere->radius;
    const v3 c = V3Negate(ray->origin);
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

f32 CapsuleRaycastParameter(const struct capsule *capsule, const struct ray *ray)
{
	/*
	References: Inigo Quilez, "Intersectors" (iquilezles.org/articles/intersectors, capsule); Hearn's
	discriminant (Hearn & Baker) as in Ray Tracing Gems (2019), ch. 7, "Precision Improvements for
	Ray/Sphere Intersection" (Haines, Guenther, Akenine-Moeller).
	*/
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

	/* Step 1: the infinite cylinder, Hearn's discriminant in 2D */
    const f32 h = capsule->half_height;
	const f32 r = capsule->radius;
	const f32 a = ray->dir.x*ray->dir.x + ray->dir.z*ray->dir.z;
	const f32 b = ray->origin.x*ray->dir.x + ray->origin.z*ray->dir.z;
	const f32 cross = ray->origin.x*ray->dir.z - ray->origin.z*ray->dir.x;
	const f32 discriminant = a*r*r - cross*cross;
	if (discriminant < 0.0f)
	{
		return F32_INFINITY;
	}

    /* Parallel check */
    if (a == 0.0f)
    {
        /* the line runs at distance rho from the axis (step 1 cannot reject it: cross = 0) */
        const f32 rho_sq = ray->origin.x*ray->origin.x + ray->origin.z*ray->origin.z;
        if (rho_sq > r*r)
        {
            return F32_INFINITY;
        }

        /* it meets the caps' surfaces at y = +-(h + s); a == 0 and dir != 0 give dir.y != 0 */
        ds_Assert(ray->dir.y != 0.0f);
        const f32 s = F32Sqrt(r*r - rho_sq);
        const f32 y_out = (0.0f < ray->dir.y) ? h + s : -(h + s);
        const f32 t_in = (-y_out - ray->origin.y) / ray->dir.y;
        if (0.0f <= t_in)
        {
            return t_in;
        }

        const f32 t_out = (y_out - ray->origin.y) / ray->dir.y;
        return (0.0f <= t_out)
            ? t_out
            : F32_INFINITY;
    }
    else
    {
        const f32 discriminant_sqrt = F32Sqrt(discriminant);
        const f32 t1 = (-b + discriminant_sqrt) / a;
        /* the cylinder, hence the capsule, is behind the origin */
        if (t1 < 0.0f)
        {
            return F32_INFINITY;
        }

        const f32 t0 = (-b - discriminant_sqrt) / a;
        const f32 y0 = ray->origin.y + t0*ray->dir.y;
        /* We enter the cylinder and capsule at the same time */
        if (-h <= y0 && y0 <= h)
        {
            /* The entry is in front of the origin */
            if (0.0f <= t0)
            {
                return t0;
            }

            /* the entry is behind the origin: the exit decides */
            const f32 y1 = ray->origin.y + t1*ray->dir.y;
            /* We leave the cylinder and capsule at the same time */
            if (-h <= y1 && y1 <= h)
            {
                return t1;
            }
        }

        const f32 dd = V3Dot(ray->dir, ray->dir);
        /* the line enters the cylinder beyond an end */
        if (y0 < -h || h < y0)
        {
            /* the cap's center relative to the origin */
            const v3 c = V3(-ray->origin.x, ((0.0f < y0) ? h : -h) - ray->origin.y, -ray->origin.z);
            const f32 cap_discriminant = dd*r*r - V3LengthSquared(V3Cross(c, ray->dir));
            if (cap_discriminant < 0.0f)
            {
                return F32_INFINITY;
            }

            const f32 t_in = (V3Dot(c, ray->dir) - F32Sqrt(cap_discriminant)) / dd;
            if (0.0f <= t_in)
            {
                return t_in;
            }
        }

        /* the entry is behind the origin: the exit decides */
        const f32 y1 = ray->origin.y + t1*ray->dir.y;
        if (-h <= y1 && y1 <= h)
        {
            return t1;
        }

        /* the line leaves the cylinder beyond an end: the capsule exit is that end's cap (far root) */
        const v3 c = V3(-ray->origin.x, ((0.0f < y1) ? h : -h) - ray->origin.y, -ray->origin.z);
        /* the line crosses the cap's disc, so >= 0 in exact arithmetic; clamped against rounding */
        const f32 cap_discriminant = F32Max(0.0f, dd*r*r - V3LengthSquared(V3Cross(c, ray->dir)));
        const f32 t_out = (V3Dot(c, ray->dir) + F32Sqrt(cap_discriminant)) / dd;
        /* the capsule is behind the origin */
        return (0.0f <= t_out)
            ? t_out
            : F32_INFINITY;
    }
}

f32 HullRaycastParameter(const struct dcel *hull, const struct ray *ray)
{
	/*
	References: Ericson, Real-Time Collision Detection, 5.3.8 "Intersecting Ray or Segment Against
	Convex Polyhedron" (Cyrus-Beck clipping); originally Eric Haines, "Fast Ray-Convex Polyhedron
	Intersection", Graphics Gems II (1991), V.1.
	*/
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

	/* the line's parameter interval inside the hull; unclipped, it is the whole line */
	f32 t_min = -F32_INFINITY;
	f32 t_max = F32_INFINITY;
	for (u32 fi = 0; fi < hull->f_count; ++fi)
	{
		/*
		The face's plane n.(p - v) = 0, n outward and unnormalized. On the line p = o + t*d:

			t = n.(v - o) / n.d = num / denom
		*/
		const v3 n = DcelFaceDirectionLocal(hull, fi);
		const v3 v = hull->v[hull->e[hull->f[fi].first].origin];
		const f32 denom = V3Dot(n, ray->dir);
		const f32 num = V3Dot(n, V3Sub(v, ray->origin));
		if (denom < 0.0f)
		{
			/* the line enters the face's halfspace */
			t_min = F32Max(t_min, num / denom);
		}
		else if (0.0f < denom)
		{
			/* the line leaves the face's halfspace */
			t_max = F32Min(t_max, num / denom);
		}
		else if (num < 0.0f)
		{
			/* parallel to the face and outside its halfspace (a degenerate face has n = 0: skipped) */
			return F32_INFINITY;
		}

		if (t_max < t_min)
		{
			return F32_INFINITY;
		}
	}

	if (0.0f <= t_min)
	{
		return t_min;
	}

	/* the entry is behind the origin: the exit decides */
	return (0.0f <= t_max)
		? t_max
		: F32_INFINITY;
}

f32 TriMeshBvhRaycastParameter(const struct triMeshBvh *mesh_bvh, const struct ray *ray)
{
	ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);

	struct arena *mem_tmp = ArenaPushScratch();
	const f32 t = TriMeshBvhRaycast(mem_tmp, mesh_bvh, ray).f;
	ArenaPopScratch();

	return t;
}

f32 SphereRaycastWorldParameter(const struct sphere *sphere, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	/* a sphere is rotation invariant: translating the origin gives its frame */
	const struct ray local =
	{
		.origin = V3F64Demote(V3F64Sub(ray->origin, transform->position)),
		.dir = ray->dir,
	};
	return SphereRaycastParameter(sphere, &local);
}

f32 CapsuleRaycastWorldParameter(const struct capsule *capsule, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	const struct ray local =
	{
		.origin = ds_TransformWorldPointToLocal(transform, ray->origin),
		.dir = QV3Rotate(QInverse(transform->rotation), ray->dir),
	};
	return CapsuleRaycastParameter(capsule, &local);
}

f32 HullRaycastWorldParameter(const struct dcel *hull, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	const struct ray local =
	{
		.origin = ds_TransformWorldPointToLocal(transform, ray->origin),
		.dir = QV3Rotate(QInverse(transform->rotation), ray->dir),
	};
	return HullRaycastParameter(hull, &local);
}

f32 TriMeshBvhRaycastWorldParameter(const struct triMeshBvh *mesh_bvh, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	const struct ray local =
	{
		.origin = ds_TransformWorldPointToLocal(transform, ray->origin),
		.dir = QV3Rotate(QInverse(transform->rotation), ray->dir),
	};
	return TriMeshBvhRaycastParameter(mesh_bvh, &local);
}

f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_SPHERE);
	return SphereRaycastWorldParameter(&shape->sphere, transform, ray);
}

f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_CAPSULE);
	return CapsuleRaycastWorldParameter(&shape->capsule, transform, ray);
}

f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_CONVEX_HULL);
	return HullRaycastWorldParameter(&shape->hull, transform, ray);
}

f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray)
{
	ds_Assert(shape->type == C_SHAPE_TRI_MESH);
	return TriMeshBvhRaycastWorldParameter(&shape->mesh_bvh, transform, ray);
}

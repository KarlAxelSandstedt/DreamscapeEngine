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

f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_SPHERE);
	struct sphere sph = SphereConstruct(transform->position, shape->sphere.radius);
	return SphereRaycastParameter(&sph, ray);
}

f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_CAPSULE);

	const f32 r = shape->capsule.radius;
	struct segment s = SegmentCapsuleTransform(&shape->capsule, transform);

	v3 p0, p1;
	const f32 dist_sq = RaySegmentDistanceSquared(&p0, &p1, ray, &s);
	if (dist_sq > r*r) { return F32_INFINITY; }

	struct sphere sph = SphereConstruct(p1, r);
	return SphereRaycastParameter(&sph, ray);
}

f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_CONVEX_HULL);

	const m3 rot = M3Q(transform->rotation);
	const struct dcel *h = &shape->hull;
	f32 t_best = F32_INFINITY;

	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		struct plane pl = DcelFacePlane(h, rot, transform->position, fi);
		const f32 t = PlaneRaycastParameter(&pl, ray);
		if (t < t_best && t >= 0.0f)
		{
			const v3 p = RayPoint(ray, t);
			if (DcelFaceProjectedPointTest(h, rot, transform->position, fi, p))
			{
				t_best = t;
			}
		}
	}

	return t_best;
}

f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	const m3 inv_rot = M3Q(QInverse(transform->rotation));

	const struct triMeshBvh *mesh_bvh = &shape->mesh_bvh;
	struct ray rotated_ray;
	rotated_ray.origin = M3V3Mul(inv_rot, V3Sub(ray->origin, transform->position));
	rotated_ray.dir = M3V3Mul(inv_rot, ray->dir);

    struct arena *mem_tmp = ArenaPushScratch();
	const f32 t = TriMeshBvhRaycast(mem_tmp, mesh_bvh, &rotated_ray).f;
    ArenaPopScratch();

    return t;
}

/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt

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

struct ds_GeometricMoverPlane
{
    struct plane    plane;
};
DEFINE_ARRAY_STRUCT(ds_GeometricMoverPlane);

static ds_Array(ds_GeometricMoverPlane) ds_GeometricMoverPlanes(struct arena *mem, const struct ds_Dynamics *dynamics, const v3f64 position, const struct capsule *capsule)
{
    struct arena *tmp = ArenaPushScratch();

    const f32 r = capsule->radius;
    const f32 h = capsule->half_height + r;
    const ds_TransformWorld t_mover = { .rotation = QIdentity(), .position = position };
    const struct BVH *bvh[3] = { &dynamics->static_bvh, &dynamics->kinematic_bvh, &dynamics->dynamic_bvh };
    const struct aabb local = { .min = V3(-r, -h, -r), .max = V3(r, h, r) };

    struct aabb bbox;
    AabbTranslateOutward(&bbox, &local, position);

    for (u32 b = 0; b < 3; ++b)
    {
        const struct BVHQuerySet query = BVHQuery(tmp, bvh[b], &bbox);
        for (u32 i = 0; i < query.count; ++i)
        {
            const struct ds_Shape *shape = dynamics->shape_pool.buf + query.shape[i];
            const struct c_Shape *c_shape = c_ShapeDBShape(dynamics->external->cshape_db, shape->cshape_handle);
            ds_TransformWorld t_world;
            ds_ShapeWorldTransform(&t_world, dynamics, shape);
            const ds_Transform t_shape = ds_TransformWorldRelative(&t_mover, &t_world);
            //TODO: the shape's planes in the mover frame
        }
        ArenaFlush(tmp);
    }

    ArenaPopScratch();

    //TODO
    return (ds_Array(ds_GeometricMoverPlane)) { .buf = NULL, .length = 0, .base = U32_MAX };
}

static v3 ds_GeometricMoverSolvePlanes(const struct ds_Dynamics *dynamics, const ds_Array(ds_GeometricMoverPlane) planes, const v3 target_relative)
{
    //TODO
    return target_relative;
}

static f32 ds_GeometricMoverCast(const struct ds_Dynamics *dynamics, const struct ds_GeometricMover *mover, const v3 delta)
{
    //TODO
    return 1.0f;
}

void ds_DynamicsGeometricMover(struct ds_Dynamics *dynamics, struct ds_GeometricMover *mover, const f32 tick)
{
    ProfZone;

    struct arena *tmp = ArenaPushScratch();

    const v3f64 target = V3F64Add(mover->position, V3Promote(V3Scale(mover->velocity, tick)));
    const f32 tol = g_numerics_config->geometric_mover_tolerance;

    for (u32 i = 0; i < g_numerics_config->geometric_mover_max_iterations; ++i)
    {
        ArenaFlush(tmp);

        const v3 target_relatve = V3F64Demote(V3F64Sub(target, mover->position));
        const ds_Array(ds_GeometricMoverPlane) planes = ds_GeometricMoverPlanes(tmp, dynamics, mover->position, &mover->capsule);
        const v3 delta = ds_GeometricMoverSolvePlanes(dynamics, planes, target_relatve);
        const f32 foi = ds_GeometricMoverCast(dynamics, mover, delta);
        const v3 sweep_foi = V3Scale(delta, foi);
        mover->position = V3F64Add(mover->position, V3Promote(sweep_foi));

        /* 
         * When convergence is achieved, one of three cases hold:
         *
         *  Target Reached: position ~= target, so sweep yields difference within
         *  tolerance. (|delta| < tol)
         *
         *  Blocked: We are moving straight into collision plane, or wedged between
         *  multiple planes: require new target for further movement. (|delta| < tol)
         *
         *  Cut short: The cast hit a new surface within tolerance. The next
         *  call slides along it. (|delta| >= tol, foi < 1)
         */
        if (V3LengthSquared(sweep_foi) < tol*tol)
        {
            break;
        }
    }

    //TODO: impulse on dynamic bodies
    //TODO: mover velocity update

    ArenaPopScratch();

    ProfZoneEnd;
}

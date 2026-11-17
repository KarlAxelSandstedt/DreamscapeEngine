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

#define GEOMETRIC_MOVER_PLANE_MAX   64
ds_StaticAssert(GEOMETRIC_MOVER_PLANE_MAX > 0 && (GEOMETRIC_MOVER_PLANE_MAX & (GEOMETRIC_MOVER_PLANE_MAX - 1)) == 0, "GEOMETRIC_MOVER_PLANE_MAX must be a power of two");

static void ds_GeometricMoverSpherePlane(ds_Array(ds_GeometricMoverPlane) *planes, const struct capsule *mover, const struct sphere *sphere, const ds_Transform *t_shape)
{
    const f32 r_sum = mover->radius + sphere->radius;
    const v3 axis = V3(0.0f, mover->half_height, 0.0f);
    const struct segment core = Segment(axis, V3Negate(axis));

    v3 n, q;
    const f32 distance_sq = SegmentPointDistanceSquared(&q, &core, t_shape->position);
    if (distance_sq > r_sum*r_sum)
    {
        return;
    }

    f32 distance = F32Sqrt(distance_sq);
    if (distance < g_solver_config->linear_slop)
    {
        n = V3(0.0f, 0.0f, -1.0f);
        distance = 0.0f;
    }
    else
    {
        n = V3Scale(V3Sub(q, t_shape->position), 1.0f / distance);
    }

    const u32 index = planes->length & (GEOMETRIC_MOVER_PLANE_MAX - 1);
    planes->buf[index].plane = (struct plane) { .normal = n, .signed_distance = r_sum - distance };
    planes->length += 1;
}

static void ds_GeometricMoverCapsulePlane(ds_Array(ds_GeometricMoverPlane) *planes, const struct capsule *mover, const struct capsule *capsule, const ds_Transform *t_shape)
{
    const f32 r_sum = mover->radius + capsule->radius;
    const v3 axis_mover = V3(0.0f, mover->half_height, 0.0f);
    const v3 axis_shape = QUnitRotate(t_shape->rotation, V3(0.0f, capsule->half_height, 0.0f));
    const struct segment core = Segment(axis_mover, V3Negate(axis_mover));
    const struct segment seg = Segment(V3Add(t_shape->position, axis_shape), V3Sub(t_shape->position, axis_shape));

    v3 p, q, n;
    const f32 distance_sq = SegmentDistanceSquared(&p, &q, &seg, &core);
    if (distance_sq > r_sum*r_sum)
    {
        return;
    }

    f32 distance = F32Sqrt(distance_sq);
    if (distance < g_solver_config->linear_slop)
    {
        n = V3(0.0f, 0.0f, -1.0f);
        distance = 0.0f;
    }
    else
    {
        n = V3Scale(V3Sub(q, p), 1.0f / distance);
    }

    const u32 index = planes->length & (GEOMETRIC_MOVER_PLANE_MAX - 1);
    planes->buf[index].plane = (struct plane) { .normal = n, .signed_distance = r_sum - distance };
    planes->length += 1;
}

static ds_Array(ds_GeometricMoverPlane) ds_GeometricMoverPlanes(struct ds_GeometricMoverPlane buf[GEOMETRIC_MOVER_PLANE_MAX], const struct ds_Dynamics *dynamics, const v3f64 position, const struct capsule *capsule)
{
    struct arena *tmp = ArenaPushScratch();

    const f32 r = capsule->radius;
    const f32 h = capsule->half_height + r;
    const ds_TransformWorld t_mover = { .rotation = QIdentity(), .position = position };
    const struct BVH *bvh[3] = { &dynamics->static_bvh, &dynamics->kinematic_bvh, &dynamics->dynamic_bvh };
    const struct aabb local = { .min = V3(-r, -h, -r), .max = V3(r, h, r) };

    struct aabb bbox;
    AabbTranslateOutward(&bbox, &local, position);

    ds_Array(ds_GeometricMoverPlane) planes = { .buf = buf, .length = 0, .base = U32_MAX };
    for (u32 b = 0; b < 3; ++b)
    {
        const ds_Array(u32) query = BVHQuery(tmp, bvh[b], &bbox);
        for (u32 i = 0; i < query.length; ++i)
        {
            const struct ds_Shape *shape = dynamics->shape_pool.buf + query.buf[i];
            const struct c_Shape *c_shape = c_ShapeDBShape(dynamics->external->cshape_db, shape->cshape_handle);
            ds_TransformWorld t_world;
            ds_ShapeWorldTransform(&t_world, dynamics, shape);
            const ds_Transform t_shape = ds_TransformWorldRelative(&t_mover, &t_world);

            switch (c_shape->type)
            {
                case C_SHAPE_SPHERE:
                {
                    ds_GeometricMoverSpherePlane(&planes, capsule, &c_shape->sphere, &t_shape);
                } break;

                case C_SHAPE_CAPSULE:
                {
                    ds_GeometricMoverCapsulePlane(&planes, capsule, &c_shape->capsule, &t_shape);
                } break;

                default:
                {
                    //TODO: hull and mesh planes
                } break;
            }
        }
    }

    if (planes.length > GEOMETRIC_MOVER_PLANE_MAX)
    {
        Log(T_PHYSICS, S_WARNING, "%s: %u planes, %u kept", __func__, planes.length, GEOMETRIC_MOVER_PLANE_MAX);
        planes.length = GEOMETRIC_MOVER_PLANE_MAX;
    }

    ArenaPopScratch();

    return planes;
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

    struct ds_GeometricMoverPlane plane_buf[GEOMETRIC_MOVER_PLANE_MAX];
    const v3f64 target = V3F64Add(mover->position, V3Promote(V3Scale(mover->velocity, tick)));
    const f32 tol = g_numerics_config->geometric_mover_tolerance;

    for (u32 i = 0; i < g_numerics_config->geometric_mover_max_iterations; ++i)
    {
        const v3 target_relatve = V3F64Demote(V3F64Sub(target, mover->position));
        const ds_Array(ds_GeometricMoverPlane) planes = ds_GeometricMoverPlanes(plane_buf, dynamics, mover->position, &mover->capsule);
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

    ProfZoneEnd;
}

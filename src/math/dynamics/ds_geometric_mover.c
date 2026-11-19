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
    f32             distance_pushed;
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
    planes->buf[index] = (struct ds_GeometricMoverPlane) { .plane = { .normal = n, .signed_distance = r_sum - distance }, .distance_pushed = 0.0f };
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
    planes->buf[index] = (struct ds_GeometricMoverPlane) { .plane = { .normal = n, .signed_distance = r_sum - distance }, .distance_pushed = 0.0f };
    planes->length += 1;
}

static void ds_GeometricMoverHullPlane(ds_Array(ds_GeometricMoverPlane) *planes, const struct capsule *mover, const struct c_Shape *hull, const ds_Transform *t_shape)
{
    ds_Assert(hull->type == C_SHAPE_CONVEX_HULL);

    const f32 radius = mover->radius;
    const struct c_Shape mover_shape = { .type = C_SHAPE_CAPSULE, .capsule = *mover };

    struct GJKCache cache;
    v3 c[2], n;
    const f32 distance = GJK(c+0, c+1, &n, &cache, NULL, &mover_shape, hull, t_shape, radius);
    if (radius < distance)
    {
        return;
    }

    /* shallow penetration; deep (the core touches the hull) gives no plane */
    if (0.0f < distance)
    {
        const u32 index = planes->length & (GEOMETRIC_MOVER_PLANE_MAX - 1);
        planes->buf[index] = (struct ds_GeometricMoverPlane) { .plane = { .normal = V3Negate(n), .signed_distance = radius - distance }, .distance_pushed = 0.0f };
        planes->length += 1;
    }
}

static void ds_GeometricMoverMeshPlanes(struct arena *mem, ds_Array(ds_GeometricMoverPlane) *planes, const struct capsule *mover, const struct mesh *mesh, const ds_Transform *mover_to_mesh)
{
    const f32 r = mover->radius;
    const f32 h = mover->half_height + r;
    const struct segment core = Segment(V3(0.0f, mover->half_height, 0.0f), V3(0.0f, -mover->half_height, 0.0f));
    const struct aabb local = { .min = V3(-r, -h, -r), .max = V3(r, h, r) };

    /* x_mesh = rot x_mover + pos */
    const m3 rot = M3QUnit(mover_to_mesh->rotation);
    const m3 rot_inv = M3QUnit(QConj(mover_to_mesh->rotation));
    const v3 pos = mover_to_mesh->position;

    /* the local box in mesh space: center pos, half widths |rot| local.max */
    const m3 rot_abs = M3Abs(rot);
    const v3 half = M3V3Mul(rot_abs, local.max);
    const struct aabb bbox = { .min = V3Sub(pos, half), .max = V3Add(pos, half) };

    const ds_Array(u32) query = MeshQuery(mem, mesh, &bbox);
    for (u32 i = 0; i < query.length; ++i)
    {
        const u32 *v = mesh->tri[query.buf[i]].buf;
        v3 tri[3];
        for (u32 k = 0; k < 3; ++k)
        {
            tri[k] = M3V3Mul(rot_inv, V3Sub(mesh->v[v[k]], pos));
        }

        const struct aabb tri_bbox = { .min = V3Min(V3Min(tri[0], tri[1]), tri[2]), .max = V3Max(V3Max(tri[0], tri[1]), tri[2]) };
        if (!AabbTest(&tri_bbox, &local))
        {
            continue;
        }

        /* 
         * TODO: we deviate from ordinary contacts here, we do back side culling. We should consdier
         * whether or not back-culling should be done for all cases.
         */
        const v3 cross = V3Cross(V3Sub(tri[1], tri[0]), V3Sub(tri[2], tri[0]));
        if (V3Dot(cross, tri[0]) > 0.0f)
        {
            continue;
        }

        f32 w[3], s;
        const enum voronoi region = TriSegmentVoronoi(w, &s, tri[0], tri[1], tri[2], core.p[0], core.p[1]);
        ds_AssertString(region != VORONOI_INVALID, "TriSegmentVoronoi: NaN input or distances beyond f32");

        const v3 c_t = V3Add(V3Add(V3Scale(tri[0], w[0]), V3Scale(tri[1], w[1])), V3Scale(tri[2], w[2]));
        const v3 c_s = SegmentBc(&core, s);
        const f32 distance_sq = V3DistanceSquared(c_t, c_s);
        if (distance_sq > r*r)
        {
            continue;
        }

        /* deep overlap: no plane */
        const f32 distance = F32Sqrt(distance_sq);
        if (distance < g_solver_config->linear_slop)
        {
            continue;
        }

        const v3 n = V3Scale(V3Sub(c_s, c_t), 1.0f / distance);
        const u32 index = planes->length & (GEOMETRIC_MOVER_PLANE_MAX - 1);
        planes->buf[index] = (struct ds_GeometricMoverPlane) { .plane = { .normal = n, .signed_distance = r - distance }, .distance_pushed = 0.0f };
        planes->length += 1;
    }
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

                case C_SHAPE_CONVEX_HULL:
                {
                    ds_GeometricMoverHullPlane(&planes, capsule, c_shape, &t_shape);
                } break;

                case C_SHAPE_TRI_MESH:
                {
                    const ds_Transform mover_to_mesh = ds_TransformWorldRelative(&t_world, &t_mover);
                    ds_GeometricMoverMeshPlanes(tmp, &planes, capsule, &c_shape->mesh, &mover_to_mesh);
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
    ProfZone;

    const f32 slop = g_solver_config->linear_slop;
    v3 target = target_relative;
    for (u32 iteration = 0; iteration < g_numerics_config->geometric_mover_solver_iterations; ++iteration)
    {
        f32 push_total = 0.0f;
        for (u32 i = 0; i < planes.length; ++i)
        {
            struct ds_GeometricMoverPlane *pl = planes.buf + i;
            const f32 s = PlanePointSignedDistance(&pl->plane, target) + slop;

            const f32 distance_pushed_old = pl->distance_pushed;
            pl->distance_pushed = F32Max(0.0f, distance_pushed_old - s);
            const f32 push = pl->distance_pushed - distance_pushed_old;

            target = V3Add(target, V3Scale(pl->plane.normal, push));
            push_total += F32Abs(push);
        }

        if (push_total < slop)
        {
            break;
        }
    }

    ProfZoneEnd;

    return target;
}

static f32 ds_GeometricMoverCastConvex(const struct capsule *mover, const struct c_Shape *shape, const ds_Transform *t_shape, const v3 delta, const f32 toi)
{
    f32 margin = mover->radius;
    if (shape->type == C_SHAPE_SPHERE)
    {
        margin += shape->sphere.radius;
    }
    else if (shape->type == C_SHAPE_CAPSULE)
    {
        margin += shape->capsule.radius;
    }

    const f32 slop = g_solver_config->linear_slop;
    const f32 target_distance = F32Max(slop, margin - slop);
    const f32 tolerance = 0.25f*slop;
    const struct c_Shape mover_shape = { .type = C_SHAPE_CAPSULE, .capsule = *mover };
    const u32 iterations = 20;

    f32 new_toi = 0.0f;
    ds_Transform t = *t_shape;

    v3 c0, c1, n;
    struct GJKCache cache[2] = { { .count = 1 }, { .count = 1 } };
    f32 distance = GJK(&c0, &c1, &n, cache + 1, cache + 0, &mover_shape, shape, &t, F32_INFINITY);
    if (distance < target_distance + tolerance)
    {
        return toi;
    }

    /*
     * We begin with new_toi = 0.0f, and monotonically increase it until we find a
     * new time-of-impact (if it exists).
     */
    for (u32 i = 0; i < iterations; ++i)
    {
        const f32 separation_velocity = -V3Dot(n, delta);
        if (separation_velocity >= 0.0f)
        {
            return toi;
        }

        /* 
         * Advance to pushed-out (by target_distance) old feature plane
         * => new distance >= target_distance.
         */
        new_toi += (target_distance - distance) / separation_velocity;
        if (new_toi >= toi)
        {
            return toi;
        }

        t.position = V3Sub(t_shape->position, V3Scale(delta, new_toi));
        distance = GJK(&c0, &c1, &n, cache + (i & 1), cache + ((i + 1) & 1), &mover_shape, shape, &t, F32_INFINITY);

        if (distance < target_distance + tolerance)
        {
            return new_toi;
        }
    }

    return new_toi;
}

static f32 ds_GeometricMoverCastConcave(const struct capsule *mover, const struct mesh *mesh, const ds_Transform *mover_to_mesh, const v3 delta, const f32 toi)
{
    const f32 r = mover->radius;
    const f32 h = mover->half_height + r;
    const struct aabb local = { .min = V3(-r, -h, -r), .max = V3(r, h, r) };

    const m3 rot = M3QUnit(mover_to_mesh->rotation);
    const m3 rot_inv = M3QUnit(QConj(mover_to_mesh->rotation));
    const v3 pos = mover_to_mesh->position;

    const m3 rot_abs = M3Abs(rot);
    const v3 half = M3V3Mul(rot_abs, local.max);
    const struct aabb bbox = { .min = V3Sub(pos, half), .max = V3Add(pos, half) };
    const v3 delta_mesh = M3V3Mul(rot, delta);

    struct arena *tmp = ArenaPushScratch();
    struct BVHCastInfo info = BVHAabbCastInit(tmp, &mesh->bvh, &bbox, delta_mesh);
    info.hit = u32f32_inline(U32_MAX, toi);
    while (info.hit_queue.count)
    {
        const u32f32 tuple = MQPop(&info.hit_queue);
        if (info.hit.f < tuple.f)
        {
            break;
        }

        if (BTLeafCheck(info.node + tuple.u))
        {
            const u32 tri_first = info.node[tuple.u].bt_child[0];
            const u32 tri_last = tri_first + info.node[tuple.u].bt_child[1] - 1;
            for (u32 i = tri_first; i <= tri_last; ++i)
            {
                const u32 *v = mesh->tri[mesh->leaf_tri[i]].buf;

                const struct aabb tri_bbox = 
                { 
                    .min = V3Min(V3Min(mesh->v[v[0]], mesh->v[v[1]]), mesh->v[v[2]]), 
                    .max = V3Max(V3Max(mesh->v[v[0]], mesh->v[v[1]]), mesh->v[v[2]]) 
                };
                const intv interval = AabbRayCastInterval(&info.ray, &tri_bbox);
                const f32 t = F32Max(interval.low, 0.0f);
                if (interval.high < t || info.hit.f < interval.low)
                {
                    continue;
                }

                v3 tri[3];
                for (u32 k = 0; k < 3; ++k)
                {
                    tri[k] = M3V3Mul(rot_inv, V3Sub(mesh->v[v[k]], pos));
                }

                /*
                 * TODO: we deviate from ordinary contacts here, we do back side culling. We should consdier
                 * whether or not back-culling should be done for all cases.
                 */
                const v3 cross = V3Cross(V3Sub(tri[1], tri[0]), V3Sub(tri[2], tri[0]));
                if (V3Dot(cross, tri[0]) > 0.0f)
                {
                    continue;
                }

                struct c_Shape tri_shape = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelTriStub() };
                tri_shape.hull.v = tri;
                const ds_Transform t_identity = ds_TransformIdentity();
                const f32 t_tri = ds_GeometricMoverCastConvex(mover, &tri_shape, &t_identity, delta, info.hit.f);
                if (t_tri < info.hit.f)
                {
                    info.hit = u32f32_inline(mesh->leaf_tri[i], t_tri);
                }
            }
        }
        else
        {
            BVHCastTestAndPushChildren(&info, tuple);
        }
    }

    ArenaPopScratch();

    return info.hit.f;
}

static f32 ds_GeometricMoverCast(const struct ds_Dynamics *dynamics, const struct ds_GeometricMover *mover, const v3 delta)
{
    /*
     * 1. The point here is to bound or mover somehow, and find all bound boxes it collides 
     * with along the way. 
     *
     * Answer: Sweep boundign box
     *
     * 2. Any bounding boxes we hit runs the chance of returning TOI between both the mover
     * capsule and the volume bounded by the bounding box, so we must check hit boxes.
     *
     * 3. We may cull future bounding boxes on our search if we hit the bounding box after
     * our current smallest TOI, this is should be damn should considering both capsule 
     * and AABB bounding box is axis aligned.
     *
     * Question: How do we reasonably cull collisions happening after time-of-impact? We
     * do not get a time-of-impact from the bounding box sweep.
     *
     * 4. How we determine TOI for internal volumes, we leave out for now, but this should
     * give us a full high-level loop which we can implement at least.
     */

    const f32 r = mover->capsule.radius;
    const f32 h = mover->capsule.half_height + r;
    const struct aabb bbox_local =
    {
        .min = V3(-r, -h, -r),
        .max = V3(r, h, r),
    };

    struct aabb bbox_world;
    AabbTranslateOutward(&bbox_world, &bbox_local, mover->position);

    const ds_TransformWorld t_mover = { .rotation = QIdentity(), .position = mover->position };
    const struct BVH *bvh[3] = { &dynamics->static_bvh, &dynamics->kinematic_bvh, &dynamics->dynamic_bvh };
    struct arena *tmp = ArenaPushScratch();

    u32f32 hit = u32f32_inline(U32_MAX, 1.0f);
    for (u32 b = 0; b < 3; ++b)
    {
        struct BVHCastInfo info = BVHAabbCastInit(tmp, bvh[b], &bbox_world, delta);
        info.hit = hit;
        while (info.hit_queue.count)
        {
            const u32f32 tuple = MQPop(&info.hit_queue);
            if (info.hit.f < tuple.f)
            {
                break;
            }

            if (BTLeafCheck(info.node + tuple.u))
            {
                const u32 si = info.node[tuple.u].bt_child[0];
                const struct ds_Shape *shape = dynamics->shape_pool.buf + si;
                const struct c_Shape *c_shape = c_ShapeDBShape(dynamics->external->cshape_db, shape->cshape_handle);
                ds_TransformWorld t_world;
                ds_ShapeWorldTransform(&t_world, dynamics, shape);

                f32 t = info.hit.f;
                if (c_shape->type == C_SHAPE_TRI_MESH)
                {
                    const ds_Transform mover_to_mesh = ds_TransformWorldRelative(&t_world, &t_mover);
                    t = ds_GeometricMoverCastConcave(&mover->capsule, &c_shape->mesh, &mover_to_mesh, delta, info.hit.f);
                }
                else
                {
                    const ds_Transform t_shape = ds_TransformWorldRelative(&t_mover, &t_world);
                    t = ds_GeometricMoverCastConvex(&mover->capsule, c_shape, &t_shape, delta, info.hit.f);
                }

                if (t < info.hit.f)
                {
                    info.hit = u32f32_inline(si, t);
                }

            }
            else
            {
                BVHCastTestAndPushChildren(&info, tuple);
            }
        }

        hit = info.hit;
        ArenaFlush(tmp);
    }

    ArenaPopScratch();

    return hit.f;
}

void ds_DynamicsGeometricMover(struct ds_Dynamics *dynamics, struct ds_GeometricMover *mover, const f32 tick)
{
    ProfZone;

    struct ds_GeometricMoverPlane plane_buf[GEOMETRIC_MOVER_PLANE_MAX];
    const v3f64 target = V3F64Add(mover->position, V3Promote(V3Scale(mover->velocity, tick)));
    const f32 tol = g_numerics_config->geometric_mover_tolerance;

    ds_Array(ds_GeometricMoverPlane) planes = { 0 };
    for (u32 i = 0; i < g_numerics_config->geometric_mover_max_iterations; ++i)
    {
        const v3 target_relatve = V3F64Demote(V3F64Sub(target, mover->position));
        planes = ds_GeometricMoverPlanes(plane_buf, dynamics, mover->position, &mover->capsule);
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
    for (u32 i = 0; i < planes.length; ++i)
    {
        const struct ds_GeometricMoverPlane *pl = planes.buf + i;
        if (pl->distance_pushed == 0.0f)
        {
            continue;
        }

        const f32 vn = V3Dot(mover->velocity, pl->plane.normal);
        if (vn < 0.0f)
        {
            mover->velocity = V3Sub(mover->velocity, V3Scale(pl->plane.normal, vn));
        }
    }

    ProfZoneEnd;
}

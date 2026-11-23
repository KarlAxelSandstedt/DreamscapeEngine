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

#include "ds_dynamics.h"
#include "ds_collision.h"

static ds_Transform ds_TOITransform(const struct ds_TimeOfImpactArg *arg, const f32 t)
{
    const q rotation_shape_world = QNLerp(arg->rotation_shape_world[0], arg->rotation_shape_world[1], t);
    const v3 com_anchor = V3Interpolate(arg->com_anchor[1], arg->com_anchor[0], t);
    const v3 shape_origin_anchor_world_diff = V3Sub(com_anchor, QUnitRotate(rotation_shape_world, arg->com_shape));
    const ds_Transform t_shape_anchor = { .rotation = rotation_shape_world, .position = shape_origin_anchor_world_diff };
    return t_shape_anchor;
}

enum ds_TOIPlaneType
{
    TOI_PLANE_VERTICES,
    TOI_PLANE_EDGES,
    TOI_PLANE_FACE,
};

struct ds_TOIPlane
{
    const struct ds_TimeOfImpactArg *   arg[2];
    enum ds_TOIPlaneType                type;
    union
    {
        struct
        {
            v3  world_axis;
        } vertices;

        struct
        {
            u32             index;      /* TOI_PLANE_FACE: index holding face */
            struct plane    plane;
        } face;
    };
};

static u32 ds_TOIUniqueCount(const u16 *index, const u32 count)
{
    u32 unique = 1;
    if (count >= 2 && index[1] != index[0])
    {
        unique += 1;
    }

    if (count == 3 && index[2] != index[0] && index[2] != index[1])
    {
        unique += 1;
    }

    return unique;
}

static struct ds_TOIPlane ds_TOIPlane(const struct ds_TimeOfImpactArg *arg0, const ds_Transform *t0, const struct ds_TimeOfImpactArg *arg1, const ds_Transform *t1, const struct GJKCache *cache, const v3 c0, const v3 c1, const v3 n)
{
    ds_Assert(1 <= cache->count && cache->count <= 3);

    const u32 unique0 = ds_TOIUniqueCount(cache->index0, cache->count);
    const u32 unique1 = ds_TOIUniqueCount(cache->index1, cache->count);

    const v3 world_axis = QUnitRotate(t0->rotation, n);

    //TODO check box3d's edge-edge plane (b3_separationEdges); edge-edge uses the 
    //world axis for now (TOI_PLANE_VERITCES)
    struct ds_TOIPlane sep = { .arg = { arg0, arg1 }, };

    if (unique0 == 3)
    {
        sep.type = TOI_PLANE_FACE;
        sep.face.index = 0;
        sep.face.plane = Plane(n, c0);
    }
    else if (unique1 == 3)
    {
        const v3 n1 = QUnitRotate(QConj(t1->rotation), V3Negate(world_axis));
        const v3 p1 = V3InverseTransform(V3Transform(c1, t0), t1);
        sep.type = TOI_PLANE_FACE;
        sep.face.index = 1;
        sep.face.plane = Plane(n1, p1);
    }
    else
    {
        sep.type = TOI_PLANE_VERTICES;
        sep.vertices.world_axis = world_axis;
    }

    return sep;
}

static f32 ds_TOIPlaneSignedDistance(const struct ds_TOIPlane *sep, const u32 index[2], const ds_Transform transform[2])
{
    const struct GJKVertexSet *vset[2] = { &sep->arg[0]->vset, &sep->arg[1]->vset };

    f32 signed_distance = 0.0f;
    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            const v3 p0 = V3Transform(vset[0]->v[index[0]], transform + 0);
            const v3 p1 = V3Transform(vset[1]->v[index[1]], transform + 1);
            signed_distance = V3Dot(V3Sub(p1, p0), sep->vertices.world_axis);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE:
        {
            const u32 i = sep->face.index;
            const u32 j = 1 - i;
            const v3 p = V3Transform(vset[j]->v[index[j]], transform + j);
            signed_distance = PlanePointSignedDistance(&sep->face.plane, V3InverseTransform(p, transform + i));
        } break;
    }

    return signed_distance;
}

static f32 ds_TOIPlaneDeepest(u32 index[2], const struct ds_TOIPlane *sep, const f32 t)
{
    const ds_Transform transform[2] = { ds_TOITransform(sep->arg[0], t), ds_TOITransform(sep->arg[1], t) };
    const struct GJKVertexSet *vset[2] = { &sep->arg[0]->vset, &sep->arg[1]->vset };

    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            const v3 axis0 = QUnitRotate(QConj(transform[0].rotation), sep->vertices.world_axis);
            const v3 axis1 = QUnitRotate(QConj(transform[1].rotation), V3Negate(sep->vertices.world_axis));
            index[0] = V3Support(vset[0]->v, vset[0]->v_count, axis0);
            index[1] = V3Support(vset[1]->v, vset[1]->v_count, axis1);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE:
        {
            const u32 i = sep->face.index;
            const u32 j = 1 - i;
            const v3 normal = QUnitRotate(transform[i].rotation, sep->face.plane.normal);
            const v3 axis = QUnitRotate(QConj(transform[j].rotation), V3Negate(normal));
            index[i] = U32_MAX;
            index[j] = V3Support(vset[j]->v, vset[j]->v_count, axis);
        } break;
    }

    return ds_TOIPlaneSignedDistance(sep, index, transform);
}

f32 ds_TimeOfImpactConvex(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 max_toi)
{
    ProfZone;

    ds_Assert(arg0->vset.margin == 0.0f && arg1->vset.margin == 0.0f);

    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 target = F32Max(slop, arg0->margin + arg1->margin - slop);
        
    struct GJKCache cache[2];
    const struct GJKCache *cache_in = NULL;

    f32 toi = 0.0f;
    for (u32 i = 0; i < g_numerics_config->toi_max_iterations; ++i)
    {
        ProfZoneNamed("Local TOI Advancement");

        const ds_Transform t_toi[2] = { ds_TOITransform(arg0, toi), ds_TOITransform(arg1, toi) };
        const ds_Transform t_1_to_0 = ds_TransformRelative(t_toi + 0, t_toi + 1);

        v3 c0, c1, n;
        struct GJKCache *cache_out = cache + (i & 1);
        const f32 distance = GJK(&c0, &c1, &n, cache_out, cache_in, &arg0->vset, &arg1->vset, &t_1_to_0, F32_INFINITY);
        cache_in = cache_out;

        /* Geomtries within tolerance at toi, or we begin with overlap at toi = 0.0f */
        if (distance <= target + tol)
        {
            ProfZoneEnd;
            goto end;
        }

        u32 index[2];
        const struct ds_TOIPlane sep = ds_TOIPlane(arg0, t_toi + 0, arg1, t_toi + 1, cache_out, c0, c1, n);

        /*
         * The plane is built from the GJK cache, not from the geometries, so it may be
         * flaky. Its signed distance at toi then need not agree with GJK's distance at
         * toi. If the plane puts a point within tolerance at toi although GJK does not,
         * there is nothing to push back: return toi.  
         */
        if (ds_TOIPlaneDeepest(index, &sep, toi) <= target + tol)
        {
            ProfZoneEnd;
            goto end;
        }

        intv plane_distance_at = { 0 };
        plane_distance_at.high = ds_TOIPlaneDeepest(index, &sep, max_toi);

        /* 
         * Geometries still separated and have not passed through each other at max_toi. 
         * No toi, so we return max_toi. (We may miss the any rotational in-out cases,
         * think of a corner being in front of separation plane at toi and max_toi,
         * but behind at some toi < t < max_toi.
         */
        if (plane_distance_at.high > target + tol)
        {
            toi = max_toi;
            ProfZoneEnd;
            goto end;
        }

        /* 
         * At this point, we must find a delta for which we may safely increment toi 
         * with, and that delta is the plane toi between the point cloud (+ margin) and
         * the separation plane. Do this this by backtracking from max_toi, repeatedly
         * finding the deepest point, and its plane_toi < max_toi.
         */
        f32 plane_toi = max_toi;
        for (u32 j = 0; j < arg0->vset.v_count + arg1->vset.v_count; ++j)
        {
            if (plane_distance_at.high >= target - tol)
            {
                break;
            }

            plane_distance_at.low = ds_TOIPlaneSignedDistance(&sep, index, t_toi);

            /* Root find deepest point plane itersection */
            ProfZoneNamed("RootFind");
            intv root = Intv(toi, plane_toi);
            for (u32 k = 0; k < g_numerics_config->toi_root_max_iterations; ++k)
            {
                const f32 t = (k & 1)
                          ? root.low + (target - plane_distance_at.low)*(root.high - root.low)/(plane_distance_at.high - plane_distance_at.low)
                          : 0.5f*(root.low + root.high);

                const ds_Transform transform[2] = { ds_TOITransform(arg0, t), ds_TOITransform(arg1, t) };
                const f32 s = ds_TOIPlaneSignedDistance(&sep, index, transform);
                if (F32Abs(s - target) <= tol)
                {
                    plane_toi = t;
                    break;
                }

                if (s > target)
                {
                    root.low = t;
                    plane_distance_at.low = s;
                }
                else
                {
                    root.high = t;
                    plane_distance_at.high = s;
                }
            }
            ProfZoneEnd;

            plane_distance_at.high = ds_TOIPlaneDeepest(index, &sep, plane_toi);
        }

        /*
         * If we never managed to separate the point-cloud from the plane, we cannot
         * continue, as plane_toi is not yet a new proper lower-bound for toi.
         */
        if (plane_distance_at.high < target - tol)
        {
            ProfZoneEnd;
            goto end;
        }
        toi = plane_toi;

        ProfZoneEnd;
    }

end:
    ProfZoneEnd;
    return toi;
}

f32 ds_TimeOfImpactConcave(struct arena *mem, const struct ds_TimeOfImpactArg *mesh, const struct ds_TimeOfImpactArg *convex, const f32 max_toi)
{
    ProfZone;

    ds_Assert(mesh->shape->type == C_SHAPE_TRI_MESH && convex->shape->type != C_SHAPE_TRI_MESH);
    const struct mesh *m = &mesh->shape->mesh;

    /* the mesh is static, and com_shape = 0 => constant pose is (rotation_shape_world[0], com_anchor[0]) */
    const ds_Transform mesh_to_anchor = { .rotation = mesh->rotation_shape_world[0], .position = mesh->com_anchor[0] };

    /* 
     * we want shape -> world-anchor-shape-diff-space
     *
     * Note: for meshes, com_shape = 0, so we have
     *         
     *          mesh -> world-anchor-shape-diff-space
     */
    const v3 shape_origin_anchor_world_diff0 = V3Sub(convex->com_anchor[0], QUnitRotate(convex->rotation_shape_world[0], convex->com_shape));
    const v3 shape_origin_anchor_world_diff1 = V3Sub(convex->com_anchor[1], QUnitRotate(convex->rotation_shape_world[1], convex->com_shape));
    const ds_Transform shape_to_anchor0 = { .rotation = convex->rotation_shape_world[0], .position = shape_origin_anchor_world_diff0 };
    const ds_Transform shape_to_anchor1 = { .rotation = convex->rotation_shape_world[1], .position = shape_origin_anchor_world_diff1 };
    const ds_Transform shape_to_mesh0 = ds_TransformRelative(&mesh_to_anchor, &shape_to_anchor0);
    const ds_Transform shape_to_mesh1 = ds_TransformRelative(&mesh_to_anchor, &shape_to_anchor1);
    const struct aabb bbox0 = c_ShapeBbox(convex->shape, &shape_to_mesh0);
    const struct aabb bbox1 = c_ShapeBbox(convex->shape, &shape_to_mesh1);
    const struct aabb bbox = BboxUnion(bbox0, bbox1);

    /* Limitation: pointy, fast-rotating shapes may leave the path's bounding box temporarily. */

    const f32 slop = g_solver_config->linear_slop;
    const f32 fallback_radius = F32Max(0.5f*convex->shape->center_of_mass_surface_distance, slop);
    const v3 shape_com_mesh0 = V3Transform(convex->shape->center_of_mass, &shape_to_mesh0);
    const v3 shape_com_mesh1 = V3Transform(convex->shape->center_of_mass, &shape_to_mesh1);

    ArenaPushRecord(mem);
    const ds_Array(u32) query = MeshQuery(mem, m, &bbox);
    f32 toi = max_toi;
    for (u32 i = 0; i < query.length; ++i)
    {
        const u32 *v = m->tri[query.buf[i]].buf;
        const v3 tri[3] = { m->v[v[0]], m->v[v[1]], m->v[v[2]] };

        /*
         * We have two triangle skip conditions:
         *
         * 1. The shape's COM starts behind the triangle; we are already behind, so CCD does
         * not apply.
         *
         * 2. The shape's COM ends sufficiently far in front of the triangle, and advanced
         * towards the triangle sufficiently slowly.
         *
         * Limitation: This is not fail-safe, but perhaps good enough. The source of the
         *  problem is that we have detected a shape-triangle pair which we ***believe***
         *  DCD will solve correctly in the next frame (here correctly means keeping the
         *  shape in front of the triangle).
         *
         *  The COM advancement is simply a heuristic for safety, not a guarantee. For
         *  example, it does not take rotation into account: for all we know, the shape
         *  may be a fast-rotating spike which kept COM1 in front while its pointy end
         *  ended up behind the triangle.
         */
        const v3 n = V3Normalize(V3Cross(V3Sub(tri[1], tri[0]), V3Sub(tri[2], tri[0])));
        const f32 offset0 = V3Dot(n, V3Sub(shape_com_mesh0, tri[0]));
        const f32 offset1 = V3Dot(n, V3Sub(shape_com_mesh1, tri[0]));
        if (offset0 < 0.0f)
        {
            continue;
        }

        if (offset1 - offset0 > -fallback_radius && offset1 > fallback_radius)
        {
            continue;
        }

        struct ds_TimeOfImpactArg triangle;
        ds_TimeOfImpactArgTriangle(&triangle, mesh, tri);

        const f32 t = ds_TimeOfImpactConvex(convex, &triangle, toi);
        if (t == 0.0f)
        {
            /*
             * Fallback: we've found a contact at t=0.0f; in theory, such a contact should
             * already have been solved, but for some reason it
             *
             *  1. wasn't found in the narrowphase, or
             *  2. wasn't sufficiently solved in the solver phase.
             *
             * In either case, since we are dealing with walls and ground here (static mesh),
             * we try to advance towards the triangle by running TimeOfImpact on the COM
             * sphere, a subset of the shape; if that fails to produce a toi > 0.0f, we are
             * off the rails and CCD fails.
             *
             * But why advance at all? Why not just be an honest citizen and stop at
             * toi=0.0f? Because we must allow DCD to actually discover new contact points,
             * something it may not be able to do at toi=0.0f. Otherwise we may get stuck at
             * toi=0.0f, never creating a contact that reduces the velocity.
             */
            struct ds_TimeOfImpactArg fallback_sphere = *convex;
            GJKVertexSetPoint(&fallback_sphere.vset, convex->shape->center_of_mass);
            fallback_sphere.margin = fallback_radius + slop;
            const f32 t_fallback = ds_TimeOfImpactConvex(&fallback_sphere, &triangle, toi);
            if (0.0f < t_fallback && t_fallback < toi)
            {
                toi = t_fallback;
            }
        }
        else if (t < toi)
        {
            toi = t;
        }
    }

    ArenaPopRecord(mem);

    ProfZoneEnd;

    return toi;
}

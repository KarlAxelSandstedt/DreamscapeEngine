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
    ds_Transform transform = ds_TransformLerp(arg->sweep + 0, arg->sweep + 1, t);
    transform.position = V3Sub(transform.position, QUnitRotate(transform.rotation, arg->local_center));
    return transform;
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
    u32                                 face;       /* TOI_PLANE_FACE: index holding face */
    v3                                  witness[2];
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

static void ds_TOIPlaneFace(struct ds_TOIPlane *sep, const u32 face, const ds_Transform *t_face, const ds_Transform *t_other, const u16 index_face[3], const u16 index_other)
{
    const struct GJKVertexSet *vset_face = &sep->arg[face]->vset;
    const struct GJKVertexSet *vset_other = &sep->arg[1 - face]->vset;

    const v3 v0 = vset_face->v[index_face[0]];
    const v3 v1 = vset_face->v[index_face[1]];
    const v3 v2 = vset_face->v[index_face[2]];
    //TODO degenerate: v0, v1, v2 (nearly) collinear, the cross is ~0
    v3 normal = V3Normalize(V3Cross(V3Sub(v1, v0), V3Sub(v2, v0)));
    const v3 point = V3Scale(V3Add(V3Add(v0, v1), v2), 1.0f / 3.0f);

    /* point the normal at the other set */
    const v3 other = QUnitRotate(t_other->rotation, vset_other->v[index_other]);
    const v3 face_point = QUnitRotate(t_face->rotation, point);
    const v3 other_relative = V3Sub(t_other->position, t_face->position);
    const v3 delta = V3Add(V3Sub(other, face_point), other_relative);
    if (V3Dot(delta, QUnitRotate(t_face->rotation, normal)) < 0.0f)
    {
        normal = V3Negate(normal);
    }

    sep->type = TOI_PLANE_FACE;
    sep->face = face;
    sep->witness[0] = normal;
    sep->witness[1] = point;
}

static struct ds_TOIPlane ds_TOIPlane(const struct ds_TimeOfImpactArg *arg0, const ds_Transform *t0, const struct ds_TimeOfImpactArg *arg1, const ds_Transform *t1, const struct GJKCache *cache, const v3 n)
{
    ds_Assert(1 <= cache->count && cache->count <= 3);

    const u32 unique0 = ds_TOIUniqueCount(cache->index0, cache->count);
    const u32 unique1 = ds_TOIUniqueCount(cache->index1, cache->count);

    //TODO
    struct ds_TOIPlane sep =
    {
        .arg = { arg0, arg1 },
        .type = TOI_PLANE_VERTICES,
        .witness[0] = QUnitRotate(t0->rotation, n),
    };

    if (unique0 == 3)
    {
        ds_TOIPlaneFace(&sep, 0, t0, t1, cache->index0, cache->index1[0]);
    }
    else if (unique1 == 3)
    {
        ds_TOIPlaneFace(&sep, 1, t1, t0, cache->index1, cache->index0[0]);
    }

    return sep;
}

static f32 ds_TOIPlaneSignedDistance(u32 index[2], const struct ds_TOIPlane *sep, const f32 t)
{
    const ds_Transform transform[2] = { ds_TOITransform(sep->arg[0], t), ds_TOITransform(sep->arg[1], t) };
    const struct GJKVertexSet *vset[2] = { &sep->arg[0]->vset, &sep->arg[1]->vset };

    f32 signed_distance = 0.0f;
    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            const v3 axis0 = QUnitRotate(QConj(transform[0].rotation), sep->witness[0]);
            const v3 axis1 = QUnitRotate(QConj(transform[1].rotation), V3Negate(sep->witness[0]));
            index[0] = V3Support(vset[0]->v, vset[0]->v_count, axis0);
            index[1] = V3Support(vset[1]->v, vset[1]->v_count, axis1);
            const v3 p0 = V3Add(QUnitRotate(transform[0].rotation, vset[0]->v[index[0]]), transform[0].position);
            const v3 p1 = V3Add(QUnitRotate(transform[1].rotation, vset[1]->v[index[1]]), transform[1].position);
            signed_distance = V3Dot(V3Sub(p1, p0), sep->witness[0]);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE:
        {
            const u32 i = sep->face;
            const u32 j = 1 - i;
            const v3 normal = QUnitRotate(transform[i].rotation, sep->witness[0]);
            const v3 point = V3Add(QUnitRotate(transform[i].rotation, sep->witness[1]), transform[i].position);
            const v3 axis = QUnitRotate(QConj(transform[j].rotation), V3Negate(normal));
            index[i] = U32_MAX;
            index[j] = V3Support(vset[j]->v, vset[j]->v_count, axis);
            const v3 p = V3Add(QUnitRotate(transform[j].rotation, vset[j]->v[index[j]]), transform[j].position);
            signed_distance = V3Dot(V3Sub(p, point), normal);
        } break;
    }

    return signed_distance;
}

static f32 ds_TOIPlaneEvaluate(const struct ds_TOIPlane *sep, const u32 index[2], const f32 t)
{
    const ds_Transform transform[2] = { ds_TOITransform(sep->arg[0], t), ds_TOITransform(sep->arg[1], t) };
    const struct GJKVertexSet *vset[2] = { &sep->arg[0]->vset, &sep->arg[1]->vset };

    f32 signed_distance = 0.0f;
    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            const v3 p0 = V3Add(QUnitRotate(transform[0].rotation, vset[0]->v[index[0]]), transform[0].position);
            const v3 p1 = V3Add(QUnitRotate(transform[1].rotation, vset[1]->v[index[1]]), transform[1].position);
            signed_distance = V3Dot(V3Sub(p1, p0), sep->witness[0]);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE:
        {
            const u32 i = sep->face;
            const u32 j = 1 - i;
            const v3 normal = QUnitRotate(transform[i].rotation, sep->witness[0]);
            const v3 point = V3Add(QUnitRotate(transform[i].rotation, sep->witness[1]), transform[i].position);
            const v3 p = V3Add(QUnitRotate(transform[j].rotation, vset[j]->v[index[j]]), transform[j].position);
            signed_distance = V3Dot(V3Sub(p, point), normal);
        } break;
    }

    return signed_distance;
}

f32 ds_TimeOfImpact(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 max_toi)
{
    ds_Assert(arg0->vset.margin == 0.0f && arg1->vset.margin == 0.0f);

    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 target = F32Max(slop, arg0->radius + arg1->radius - slop);
        
    struct GJKCache cache[2];
    const struct GJKCache *cache_in = NULL;

    f32 toi = 0.0f;
    for (u32 i = 0; i < g_numerics_config->toi_max_iterations; ++i)
    {
        const ds_Transform t0 = ds_TOITransform(arg0, toi);
        const ds_Transform t1 = ds_TOITransform(arg1, toi);
        const ds_Transform t_1_to_0 = ds_TransformRelative(&t0, &t1);

        v3 c0, c1, n;
        struct GJKCache *cache_out = cache + (i & 1);
        const f32 distance = GJK(&c0, &c1, &n, cache_out, cache_in, &arg0->vset, &arg1->vset, &t_1_to_0, F32_INFINITY);
        cache_in = cache_out;

        /* Geomtries within tolerance at toi, or we begin with overlap at toi = 0.0f */
        if (distance <= target + tol)
        {
            return toi;
        }

        u32 index[2];
        const struct ds_TOIPlane sep = ds_TOIPlane(arg0, &t0, arg1, &t1, cache_out, n);

        /*
         * The plane is built from the GJK cache, not from the geometries, so it may be
         * flaky. Its signed distance at toi then need not agree with GJK's distance at
         * toi. If the plane puts a point within tolerance at toi although GJK does not,
         * there is nothing to push back: return toi.  
         */
        if (ds_TOIPlaneSignedDistance(index, &sep, toi) <= target + tol)
        {
            return toi;
        }

        intv plane_distance_at = { 0 };
        plane_distance_at.high = ds_TOIPlaneSignedDistance(index, &sep, max_toi);

        /* 
         * Geometries still separated and have not passed through each other at max_toi. 
         * No toi, so we return max_toi. (We may miss the any rotational in-out cases,
         * think of a corner being in front of separation plane at toi and max_toi,
         * but behind at some toi < t < max_toi.
         */
        if (plane_distance_at.high > target + tol)
        {
            return max_toi;
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

            plane_distance_at.low = ds_TOIPlaneEvaluate(&sep, index, toi);

            /* Root find deepest point plane itersection */
            intv root = Intv(toi, plane_toi);
            for (u32 k = 0; k < g_numerics_config->toi_root_max_iterations; ++k)
            {
                const f32 t = (k & 1)
                          ? root.low + (target - plane_distance_at.low)*(root.high - root.low)/(plane_distance_at.high - plane_distance_at.low)
                          : 0.5f*(root.low + root.high);

                const f32 s = ds_TOIPlaneEvaluate(&sep, index, t);
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

            plane_distance_at.high = ds_TOIPlaneSignedDistance(index, &sep, plane_toi);
        }

        /*
         * If we never managed to separate the point-cloud from the plane, we cannot
         * continue, as plane_toi is not yet a new proper lower-bound for toi.
         */
        if (plane_distance_at.high < target - tol)
        {
            return toi;
        }
        toi = plane_toi;
    }

    return toi;
}

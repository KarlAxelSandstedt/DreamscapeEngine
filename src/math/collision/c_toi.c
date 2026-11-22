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
    TOI_PLANE_FACE0,
    TOI_PLANE_FACE1,
};

struct ds_TOIPlane
{
    const struct ds_TimeOfImpactArg *   arg0;
    const struct ds_TimeOfImpactArg *   arg1;
    enum ds_TOIPlaneType                type;
    v3                                  witness0;
    v3                                  witness1;
};

static struct ds_TOIPlane ds_TOIPlane(const struct ds_TimeOfImpactArg *arg0, const ds_Transform *t0, const struct ds_TimeOfImpactArg *arg1, const ds_Transform *t1, const struct GJKCache *cache, const v3 n)
{
    //TODO
    const struct ds_TOIPlane sep =
    {
        .arg0 = arg0,
        .arg1 = arg1,
        .type = TOI_PLANE_VERTICES,
        .witness0 = QUnitRotate(t0->rotation, n),
    };
    return sep;
}

static f32 ds_TOIPlaneSignedDistance(u32 *index0, u32 *index1, const struct ds_TOIPlane *sep, const f32 t)
{
    const ds_Transform t0 = ds_TOITransform(sep->arg0, t);
    const ds_Transform t1 = ds_TOITransform(sep->arg1, t);

    f32 signed_distance = 0.0f;
    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            *index0 = V3Support(sep->arg0->vset.v, sep->arg0->vset.v_count, QUnitRotate(QConj(t0.rotation), sep->witness0));
            *index1 = V3Support(sep->arg1->vset.v, sep->arg1->vset.v_count, QUnitRotate(QConj(t1.rotation), V3Negate(sep->witness0)));
            const v3 p0 = V3Add(QUnitRotate(t0.rotation, sep->arg0->vset.v[*index0]), t0.position);
            const v3 p1 = V3Add(QUnitRotate(t1.rotation, sep->arg1->vset.v[*index1]), t1.position);
            signed_distance = V3Dot(V3Sub(p1, p0), sep->witness0);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE0:
        {
        } break;

        case TOI_PLANE_FACE1:
        {
        } break;
    }

    return signed_distance;
}

static f32 ds_TOIPlaneEvaluate(const struct ds_TOIPlane *sep, const u32 index0, const u32 index1, const f32 t)
{
    const ds_Transform t0 = ds_TOITransform(sep->arg0, t);
    const ds_Transform t1 = ds_TOITransform(sep->arg1, t);

    f32 signed_distance = 0.0f;
    switch (sep->type)
    {
        case TOI_PLANE_VERTICES:
        {
            const v3 p0 = V3Add(QUnitRotate(t0.rotation, sep->arg0->vset.v[index0]), t0.position);
            const v3 p1 = V3Add(QUnitRotate(t1.rotation, sep->arg1->vset.v[index1]), t1.position);
            signed_distance = V3Dot(V3Sub(p1, p0), sep->witness0);
        } break;

        case TOI_PLANE_EDGES:
        {
        } break;

        case TOI_PLANE_FACE0:
        {
        } break;

        case TOI_PLANE_FACE1:
        {
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

        u32 index0, index1;
        const struct ds_TOIPlane sep = ds_TOIPlane(arg0, &t0, arg1, &t1, cache_out, n);

        /*
         * The plane is built from the GJK cache, not from the geometries, so it may be
         * flaky. Its signed distance at toi then need not agree with GJK's distance at
         * toi. If the plane puts a point within tolerance at toi although GJK does not,
         * there is nothing to push back: return toi.  
         */
        if (ds_TOIPlaneSignedDistance(&index0, &index1, &sep, toi) <= target + tol)
        {
            return toi;
        }

        intv plane_distance_at = { 0 };
        plane_distance_at.high = ds_TOIPlaneSignedDistance(&index0, &index1, &sep, max_toi);

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

            plane_distance_at.low = ds_TOIPlaneEvaluate(&sep, index0, index1, toi);

            /* Root find deepest point plane itersection */
            intv root = Intv(toi, plane_toi);
            for (u32 k = 0; k < g_numerics_config->toi_root_max_iterations; ++k)
            {
                const f32 t = (k & 1)
                          ? root.low + (target - plane_distance_at.low)*(root.high - root.low)/(plane_distance_at.high - plane_distance_at.low)
                          : 0.5f*(root.low + root.high);

                const f32 s = ds_TOIPlaneEvaluate(&sep, index0, index1, t);
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

            plane_distance_at.high = ds_TOIPlaneSignedDistance(&index0, &index1, &sep, plane_toi);
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

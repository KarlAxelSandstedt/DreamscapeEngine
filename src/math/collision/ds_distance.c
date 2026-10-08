
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
#include "collision.h"
#include "ds_voronoi.h"

/*
GJK distance
============
Box3D's variant (agent_notes/gjk_design.txt): runs in shape A's frame on the Minkowski difference
B - A, solves the simplex with the Voronoi API (ds_voronoi.h), takes the search direction from the
simplex geometry, stops on a repeated support pair, on no progress (restoring the previous simplex)
or on overlap, and warm-starts from a cache of support indices.
*/

/* Shape vertex set/storage */
struct GJKVertexSet
{
    const v3 *  v;
    u32         v_count;
    v3          buf[2];     /* storage for sphere/capsule points */
};

/* Simplex vertex: both support points in frame A and their difference. */
struct GJKSimplexVertex
{
    v3  a;                  /* support point of A */
    v3  b;                  /* support point of B, moved into frame A */
    v3  minkowski;          /* b - a: */
    u32 index_a;            /* GJKVertexSet vertex indices of a and b */
    u32 index_b;
};

struct GJKSimplex
{
    struct GJKSimplexVertex v[4];
    f32                     weight[4];  /* barycentric weights of the closest point, from the Voronoi API */
    u32                     count;      /* 1..4 */
};

/*
 * Data shared by the GJK functions of one query: both vertex sets (local frames), the transform of B into
 * A's frame, and A's frame for the world-space outputs. Never copy it: a sphere's/capsule's vset.v points
 * into its own vset.buf.
 */
struct GJKHelper
{
    struct GJKVertexSet     vset_a;
    struct GJKVertexSet     vset_b;
    m3                      rot;        /* B to A: p_A = rot * p_B + pos */
    m3                      rot_t;      /* rot^T: directions from A's frame into B's */
    v3                      pos;
    const ds_Transform *    t_a;        /* A to world */
};

/* Set up the vset of shape (hull, sphere or capsule) in its local frame. */
static void GJKVertexSetInit(struct GJKVertexSet *vset, const struct c_Shape *shape)
{
    switch (shape->type)
    {
        case C_SHAPE_SPHERE:
        {
            vset->v = vset->buf;
            vset->buf[0] = V3Zero();
            vset->v_count = 1;
        } break;

        case C_SHAPE_CAPSULE:
        {
            vset->v = vset->buf;
            vset->buf[0] = V3(0.0f,  shape->capsule.half_height, 0.0f);
            vset->buf[1] = V3(0.0f, -shape->capsule.half_height, 0.0f);
            vset->v_count = 2;
        } break;

        case C_SHAPE_CONVEX_HULL:
        {
            ds_AssertString(shape->hull.v_count <= U16_MAX + 1, "GJKCache stores vertex indices as u16");
            vset->v = shape->hull.v;
            vset->v_count = shape->hull.v_count;
        } break;

        default:
        {
            ds_AssertString(0, "Unreachable");
        } break;
    }
}

static void GJKHelperInit(struct GJKHelper *helper, const struct c_Shape *shape_a, const ds_Transform *t_a, const struct c_Shape *shape_b, const ds_Transform *t_b)
{
    GJKVertexSetInit(&helper->vset_a, shape_a);
    GJKVertexSetInit(&helper->vset_b, shape_b);
    const ds_Transform relative = ds_TransformRelative(t_a, t_b);
    helper->rot = M3Q(relative.rotation);
    helper->rot_t = M3Transpose(helper->rot);
    helper->pos = relative.position;
    helper->t_a = t_a;
}

/* Simplex vertex of index_a of A and index_b of B; B is moved into A's frame. */
static struct GJKSimplexVertex GJKSimplexVertexInit(const struct GJKHelper *helper, const u32 index_a, const u32 index_b)
{
    struct GJKSimplexVertex v;
    v.index_a = index_a;
    v.index_b = index_b;
    v.a = helper->vset_a.v[index_a];
    v.b = V3Add(M3V3Mul(helper->rot, helper->vset_b.v[index_b]), helper->pos);
    v.minkowski = V3Sub(v.b, v.a);
    return v;
}

/*
 * Set v to the support point of B - A in direction dir (frame A):
 *
 *      max_ab dot(b - a, dir) = max_b dot(b, dir) - min_a dot(a, dir)
 */
static void GJKSimplexVertexSupport(struct GJKSimplexVertex *v, const struct GJKHelper *helper, const v3 dir)
{
    const u32 index_a = V3Support(helper->vset_a.v, helper->vset_a.v_count, V3Negate(dir));
    const u32 index_b = V3Support(helper->vset_b.v, helper->vset_b.v_count, M3V3Mul(helper->rot_t, dir));
    *v = GJKSimplexVertexInit(helper, index_a, index_b);
}

/* Absolute volume/area/length (times a constant) of Simplex. Used to approve/reject cache_in */
static f32 GJKSimplexMetric(const struct GJKSimplex *simplex)
{
    const struct GJKSimplexVertex *v = simplex->v;
    f32 metric = 0.0f;
    switch (simplex->count)
    {
        case 2:
        {
            metric = V3Length(V3Sub(v[1].minkowski, v[0].minkowski));
        } break;

        case 3:
        {
            const v3 e1 = V3Sub(v[1].minkowski, v[0].minkowski);
            const v3 e2 = V3Sub(v[2].minkowski, v[0].minkowski);
            metric = V3Length(V3Cross(e1, e2));
        } break;

        case 4:
        {
            const v3 e1 = V3Sub(v[1].minkowski, v[0].minkowski);
            const v3 e2 = V3Sub(v[2].minkowski, v[0].minkowski);
            const v3 e3 = V3Sub(v[3].minkowski, v[0].minkowski);
            metric = F32Abs(V3Dot(e1, V3Cross(e2, e3)));
        } break;
    }

    return metric;
}

/*
 * Search direction: perpendicular to the closest feature, towards the origin. In exact math this is
 * -closest, but closest is blended with the weights and carries an absolute error of ~eps * shape size,
 * which dominates near contact (closest ~ 0). Derived from the feature's vertices instead, the error
 * only depends on the feature's shape, not on its distance to the origin.
 *
 *      vertex a:           -a
 *      edge ab:            ab x (ab x a)           (in the plane of ab and the origin)
 *      triangle abc:       +-(ab x ac)             (the side of the origin)
 */
static v3 GJKSimplexSearchDirection(const struct GJKSimplex *simplex)
{
    ds_Assert(1 <= simplex->count && simplex->count <= 3);
    const v3 a = simplex->v[0].minkowski;
    v3 dir = V3Negate(a);
    switch (simplex->count)
    {
        case 2:
        {
            const v3 ab = V3Sub(simplex->v[1].minkowski, a);
            dir = V3Cross(ab, V3Cross(ab, a));
        } break;

        case 3:
        {
            const v3 ab = V3Sub(simplex->v[1].minkowski, a);
            const v3 ac = V3Sub(simplex->v[2].minkowski, a);
            const v3 normal = V3Cross(ab, ac);
            dir = (V3Dot(normal, a) < 0.0f) ? normal : V3Negate(normal);
        } break;
    }

    return dir;
}

/*
 * Smallest usable |dir|^2 (dir is not normalized). Not a geometric tolerance: the vertex, edge and
 * triangle directions have different units (length, length^3, length^2), so the test only says "dir is
 * ~0". The margin above the smallest normal f32 keeps |dir|^2 and the normalization out of subnormals
 * (precision loss, Inf from 1 / sqrt(0)); any factor from ~10 to ~1e6 behaves the same.
 */
#define GJK_DIR_LENGTH_SQ_MIN (1000.0f * F32_MIN_POSITIVE_NORMAL)

/* Set simplex cache.  */
static void GJKCacheInit(struct GJKCache *cache, const struct GJKSimplex *simplex)
{
    ds_Assert(1 <= simplex->count && simplex->count <= 4);
    cache->metric = GJKSimplexMetric(simplex);
    cache->count = (u16) simplex->count;
    for (u32 i = 0; i < 4; ++i)
    {
        const u32 used = (i < simplex->count);
        cache->index_a[i] = (used) ? (u16) simplex->v[i].index_a : 0;
        cache->index_b[i] = (used) ? (u16) simplex->v[i].index_b : 0;
    }
}

/*
 * Overlap/touching result (1, 4a): the origin lies on the simplex of B - A, so sum_i w_i b_i = sum_i w_i a_i.
 * Both witness points are taken from A's blend, so c_a == c_b exactly (distance 0); n = 0.
 */
static f32 GJKOverlap(v3 *c_a, v3 *c_b, v3 *n, struct GJKCache *cache_out, const struct GJKHelper *helper, const struct GJKSimplex *simplex)
{
    v3 c = V3Zero();
    for (u32 i = 0; i < simplex->count; ++i)
    {
        c = V3AddScaled(c, simplex->v[i].a, simplex->weight[i]);
    }
    *c_a = V3Add(QV3Rotate(helper->t_a->rotation, c), helper->t_a->position);
    *c_b = *c_a;
    *n = V3Zero();
    GJKCacheInit(cache_out, simplex);
    return 0.0f;
}

/*
 * Distance between the shapes A and B. Outputs are in world space: c_a, c_b the closest points, 
 * n the unit normal from A to B.
 *
 *      cache_in            NULL or the pair's previous cache; warm-starts the search unless the cached
 *                          simplex changed size by more than 2x
 *      cache_out           always written (caches live on 2-frame arenas)
 *      cutoff_distance     distances above it are of no interest (contacts: r_a + r_b + speculative
 *                          margin); F32_INFINITY for distance queries
 *
 * Returns:
 *
 *      0.0f                overlap or touch; c_a == c_b, n = 0
 *      F32_INFINITY        a separating plane proves distance > cutoff_distance; c_a, c_b, n garbage
 *      otherwise           the distance; c_a, c_b, n valid. Only an upper bound if the iteration limit
 *                          was reached
 *
 * Not every stop tests cutoff_distance, so a finite result may still exceed it; callers compare.
 */
static f32 GJK(v3 *c_a, v3 *c_b, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *shape_a, const ds_Transform *t_a, const struct c_Shape *shape_b, const ds_Transform *t_b, const f32 cutoff_distance)
{
    struct GJKHelper helper;
    GJKHelperInit(&helper, shape_a, t_a, shape_b, t_b);
    const f32 cutoff_distance_sq = cutoff_distance*cutoff_distance;

    /* Use cache if it exist and its length/area/volume hasn't changed to much  */
    struct GJKSimplex simplex = { 0 };
    if (cache_in)
    {
        ds_Assert(1 <= cache_in->count && cache_in->count <= 4);
        simplex.count = cache_in->count;
        for (u32 i = 0; i < simplex.count; ++i)
        {
            simplex.v[i] = GJKSimplexVertexInit(&helper, cache_in->index_a[i], cache_in->index_b[i]);
        }

        const f32 metric_old = cache_in->metric;
        const f32 metric_new = GJKSimplexMetric(&simplex);
        if (2.0f*metric_old < metric_new || metric_new < 0.5f*metric_old || (simplex.count > 1 && metric_new < F32_EPSILON))
        {
            simplex.count = 0;
        }
    }

    if (simplex.count == 0)
    {
        simplex.count = 1;
        simplex.v[0] = GJKSimplexVertexInit(&helper, 0, 0);
    }
    struct GJKSimplex backup = { .count = 1, .v[0] = simplex.v[0], .weight[0] = 1.0f };

    /*
     * Termination. After the loop, the result simplex is solved and dir is its search direction with
     * |dir|^2 >= GJK_DIR_LENGTH_SQ_MIN, so the normal -dir / |dir| is finite.
     *
     *      #   condition                               meaning                             result
     *      1   region T0123                            origin inside the tetrahedron       return 0, n = 0
     *      2   region INVALID                          degenerate, can't classify          backup
     *      3   !(distance_sq > iteration_distance_sq)  no strict progress (or NaN)         backup
     *      4a  |dir|^2 < MIN, count 1                  |dir| is the distance: ~0           return 0, n = 0
     *      4b  |dir|^2 < MIN, count 2-3                degenerate edge/triangle            backup
     *      5   the support bound proves                early exit                          return F32_INFINITY
     *          distance > cutoff_distance
     *      6   the support point makes no progress     converged                           simplex
     *          along dir (repeated support pair)
     *      7   iteration limit                         not converged                       backup (after the loop:
     *                                                                                      drop the unsolved vertex)
     *
     * backup is the last simplex that passed 1-4, and dir its direction. Initially backup is the first
     * vertex, so dir starts as its direction, checked for 4a before the loop: a cached simplex can fall back
     * to backup (2, 4b) in the first iteration.
     */

    /* 4a */
    v3 dir = V3Negate(backup.v[0].minkowski);
    if (V3LengthSquared(dir) < GJK_DIR_LENGTH_SQ_MIN)
    {
        return GJKOverlap(c_a, c_b, n, cache_out, &helper, &backup);
    }

    f32 distance_sq = F32_INFINITY;
    u32 iter = 0;
    for (; iter < g_numerics_config->gjk_max_iterations; ++iter)
    {
        /* closest feature of the simplex to the origin */
        f32 w[4];
        enum voronoi region = VORONOI_INVALID;
        switch (simplex.count)
        {
            case 1:
            {
                region = VORONOI_V0;
                w[0] = 1.0f;
            } break;

            case 2:
            {
                region = SegmentOriginVoronoi(w, simplex.v[0].minkowski, simplex.v[1].minkowski);
            } break;

            case 3:
            {
                region = TriOriginVoronoi(w, simplex.v[0].minkowski, simplex.v[1].minkowski, simplex.v[2].minkowski);
            } break;

            case 4:
            {
                region = TetrahedronOriginVoronoi(w, simplex.v[0].minkowski, simplex.v[1].minkowski, simplex.v[2].minkowski, simplex.v[3].minkowski);
            } break;
        }

        /* 2: numerically degenerate simplex, the previous one is (nearly) the closest */
        if (region == VORONOI_INVALID)
        {
            simplex = backup;
            break;
        }

        /* Keep the vertices of the closest feature */
        u32 count = 0;
        for (u32 j = 0; j < simplex.count; ++j)
        {
            if (region & (1u << j))
            {
                simplex.v[count] = simplex.v[j];
                simplex.weight[count] = w[j];
                count += 1;
            }
        }
        simplex.count = count;

        /* 1 */
        if (region == VORONOI_T0123)
        {
            return GJKOverlap(c_a, c_b, n, cache_out, &helper, &simplex);
        }

        v3 closest = V3Zero();
        for (u32 j = 0; j < simplex.count; ++j)
        {
            closest = V3AddScaled(closest, simplex.v[j].minkowski, simplex.weight[j]);
        }

        /* 3: no strict progress (or NaN) */
        const f32 iteration_distance_sq = V3LengthSquared(closest);
        if (!(distance_sq > iteration_distance_sq))
        {
            simplex = backup;
            break;
        }

        /* dir must stay backup's direction until this one is accepted */
        const v3 next_dir = GJKSimplexSearchDirection(&simplex);
        if (V3LengthSquared(next_dir) < GJK_DIR_LENGTH_SQ_MIN)
        {
            /* 4a: |dir| = |a| = distance ~ 0 */
            if (simplex.count == 1)
            {
                return GJKOverlap(c_a, c_b, n, cache_out, &helper, &simplex);
            }

            /* 4b */
            simplex = backup;
            break;
        }
        dir = next_dir;
        distance_sq = iteration_distance_sq;

        /* candidate vertex in the free slot; added below if it passes 5 and 6 */
        GJKSimplexVertexSupport(simplex.v + simplex.count, &helper, dir);

        /*
         * 5: B - A lies in the half-space dot(p, dir) <= dot(w, dir) of the support point w, so
         *
         *      distance >= -dot(w, dir) / |dir|
         */
        const f32 support_dot = V3Dot(simplex.v[simplex.count].minkowski, dir);
        const f32 support_dot_sq = support_dot*support_dot;
        if (support_dot < 0.0f && support_dot_sq > cutoff_distance_sq*V3LengthSquared(dir))
        {
            /* the candidate isn't counted: the solved simplex is cached */
            GJKCacheInit(cache_out, &simplex);
            return F32_INFINITY;
        }

        /* 6: the support pair is already in the simplex: no point of B - A lies further along dir.  */
        u32 repeated = 0;
        for (u32 j = 0; j < simplex.count; ++j)
        {
            repeated |= (simplex.v[j].index_a == simplex.v[simplex.count].index_a
                      && simplex.v[j].index_b == simplex.v[simplex.count].index_b);
        }

        if (repeated)
        {
            break;
        }

        backup = simplex;
        simplex.count += 1;
    }

    /* 7: the last candidate was added but never solved; dropping it leaves backup */
    if (iter == g_numerics_config->gjk_max_iterations)
    {
        simplex.count -= 1;
    }

    v3 p_a = V3Zero();
    v3 p_b = V3Zero();
    v3 closest = V3Zero();
    for (u32 j = 0; j < simplex.count; ++j)
    {
        p_a = V3AddScaled(p_a, simplex.v[j].a, simplex.weight[j]);
        p_b = V3AddScaled(p_b, simplex.v[j].b, simplex.weight[j]);
        closest = V3AddScaled(closest, simplex.v[j].minkowski, simplex.weight[j]);
    }

    /*
     * The distance from the minkowski blend, not |p_b - p_a| (cancellation near contact); the normal from
     * dir (the feature's geometry), not from p_b - p_a. dir points from the closest point to the origin,
     * and closest = p_b - p_a, so the normal from A to B is -dir.
     */
    *c_a = V3Add(QV3Rotate(t_a->rotation, p_a), t_a->position);
    *c_b = V3Add(QV3Rotate(t_a->rotation, p_b), t_a->position);
    *n = QV3Rotate(t_a->rotation, V3Scale(dir, -1.0f / V3Length(dir)));
    GJKCacheInit(cache_out, &simplex);
    return V3Length(closest);
}

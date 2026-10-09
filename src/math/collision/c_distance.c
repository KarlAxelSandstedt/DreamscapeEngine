
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
=========================================== GJK ==============================================
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
    v3  minkowski;          /* b - a */
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
 * A's frame, and A's frame for the world-space outputs.
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

/* Absolute volume/area/length (times a constant) of the simplex; decides whether cache_in is reused. */
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

/*
 * Touch tolerance (1b; GJK_TOUCH_TOLERANCE in collision.h). The shapes are reported as touching when
 *
 *      |closest| <= GJK_TOUCH_TOLERANCE * eps * L,     L = max(|a|, |b|) over the query's support points
 *
 * Safe for any simplex: closest = sum_i w_i v_i lies in B - A, so |closest| bounds the distance from above.
 *
 * L is the noise scale: a support point v = b - a is computed from a and b in A's frame, so it carries a
 * rounding error of ~eps * max(|a|, |b|), also near contact where v itself cancels to a face's size.
 *
 * GJK_TOUCH_TOLERANCE is the band's width in units of that noise (eps * L). Its value bounds the error of
 * the returned normal: a separation d with that noise gives a normal off by (measured)
 *
 *      theta ~ 6 eps * L / d
 *
 * so with a width of 100, every separated result has d > 100 eps * L and theta <= ~6 / 100 = 0.06 rad;
 * closer pairs report touching, and contacts take their deep paths (feature normals). At L = 1 m the band
 * is 100 * eps * 1 m = 0.012 mm, Box3D's 100 * FLT_EPSILON contact threshold, but it scales with the
 * shapes. It is independent of the world position (L is measured in A's frame).
 */

/* Write the simplex's support indices and metric to cache. */
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
 * Overlap/touching result (1, 1b, 4a): the origin lies on the simplex of B - A (within the touch band), so
 * sum_i w_i b_i ~ sum_i w_i a_i. Both witness points are taken from A's blend: c_a == c_b, n = 0.
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

/* API and outputs: collision.h */
f32 GJK(v3 *c_a, v3 *c_b, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *shape_a, const ds_Transform *t_a, const struct c_Shape *shape_b, const ds_Transform *t_b, const f32 cutoff_distance)
{
    struct GJKHelper helper;
    GJKHelperInit(&helper, shape_a, t_a, shape_b, t_b);
    const f32 cutoff_distance_sq = cutoff_distance*cutoff_distance;

    /* warm start from the cache unless its length/area/volume changed by more than 2x */
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

    /* running max of |a|^2, |b|^2 over the support points, the scale of 1b */
    f32 vertex_length_sq_max = 0.0f;
    for (u32 j = 0; j < simplex.count; ++j)
    {
        vertex_length_sq_max = F32Max(vertex_length_sq_max, F32Max(V3LengthSquared(simplex.v[j].a), V3LengthSquared(simplex.v[j].b)));
    }

    /*
     * Termination. After the loop, the result simplex is solved and dir is its search direction with
     * |dir|^2 >= GJK_DIR_LENGTH_SQ_MIN, so the normal -dir / |dir| is finite.
     *
     *      #   condition                               meaning                             result
     *      1   region T0123                            origin inside the tetrahedron       return 0, n = 0
     *      1b  |closest| <= GJK_TOUCH_TOLERANCE*eps*L  origin on the simplex (touching)    return 0, n = 0
     *      2   region INVALID                          degenerate, can't classify          backup
     *      3   !(distance_sq > iteration_distance_sq)  no strict progress (or NaN)         backup
     *      4a  |dir|^2 < MIN, count 1                  |dir| is the distance: ~0           return 0, n = 0
     *      4b  |dir|^2 < MIN, count 2-3                degenerate edge/triangle (1b        backup
     *                                                  ruled out touching)
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

        /* 1b */
        const f32 iteration_distance_sq = V3LengthSquared(closest);
        const f32 touch_tolerance = GJK_TOUCH_TOLERANCE * F32_EPSILON;
        if (iteration_distance_sq <= touch_tolerance * touch_tolerance * vertex_length_sq_max)
        {
            return GJKOverlap(c_a, c_b, n, cache_out, &helper, &simplex);
        }

        /* 3: no strict progress (or NaN) */
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
        vertex_length_sq_max = F32Max(vertex_length_sq_max, F32Max(V3LengthSquared(simplex.v[simplex.count].a), V3LengthSquared(simplex.v[simplex.count].b)));

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

    /* the result simplex passed 1b (or is the first vertex, 4a): 0 is only returned through GJKOverlap */
    const f32 distance = V3Length(closest);
    ds_Assert(distance > 0.0f);
    return distance;
}

/********************************** DISTANCE METHODS **********************************/

f32 c_SphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_SPHERE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	f32 dist_sq = 0.0f;

	const f32 r_sum = s1->sphere.radius + s2->sphere.radius;
	if (V3DistanceSquared(t1->position, t2->position) > r_sum*r_sum)
	{
		v3 dir = V3Sub(t2->position, t1->position);
		dir = V3Scale(dir, 1.0f/V3Length(dir));
		*c1 = V3AddScaled(t1->position, dir,  s1->sphere.radius);
		*c2 = V3AddScaled(t2->position, dir, -s2->sphere.radius);
		dist_sq = V3DistanceSquared(*c1, *c2);
	}

	return F32Sqrt(dist_sq);
}

f32 c_CapsuleSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	const struct capsule *cap = &s1->capsule;
	const f32 r_sum = cap->radius + s2->sphere.radius;
	struct segment s = SegmentCapsuleTransform(cap, t1);

	f32 dist = 0.0f;
	if (SegmentPointDistanceSquared(c1, &s, t2->position) > r_sum*r_sum)
	{
		const v3 n = V3Normalize(V3Sub(t2->position, *c1));
		*c1 = V3AddScaled(*c1, n, cap->radius);
		*c2 = V3AddScaled(t2->position, n, -s2->sphere.radius);
		dist = F32Sqrt(V3DistanceSquared(*c1, *c2));
	}

	return dist;
}

f32 c_CapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_CAPSULE);

	const struct capsule *cap1 = &s1->capsule;
	const struct capsule *cap2 = &s2->capsule;
	const f32 r_sum = cap1->radius + cap2->radius;

	struct segment seg1 = SegmentCapsuleTransform(cap1, t1);
	struct segment seg2 = SegmentCapsuleTransform(cap2, t2);

	f32 dist = 0.0f;
	if (SegmentDistanceSquared(c1, c2, &seg1, &seg2) > r_sum*r_sum)
	{
		const v3 n = V3Normalize(V3Sub(*c2, *c1));
		*c1 = V3AddScaled(*c1, n, cap1->radius);
		*c2 = V3AddScaled(*c2, n, -cap2->radius);
		dist = F32Sqrt(V3DistanceSquared(*c1, *c2));
	}

	return dist;
}

f32 c_HullSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s2->type == C_SHAPE_SPHERE);

	v3 n;
	struct GJKCache cache;
	const f32 dist = GJK(c1, c2, &n, &cache, NULL, s1, t1, s2, t2, F32_INFINITY);
	if (dist <= s2->sphere.radius)
	{  
        return 0.0f;
	}
	
	*c2 = V3AddScaled(*c2, n, -s2->sphere.radius);
	return dist - s2->sphere.radius;
}

f32 c_HullCapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s2->type == C_SHAPE_CAPSULE);

	v3 n;
	struct GJKCache cache;
	const f32 dist = GJK(c1, c2, &n, &cache, NULL, s1, t1, s2, t2, F32_INFINITY);
	if (dist <= s2->capsule.radius)
	{
        return 0.0f;
	}

	*c2 = V3AddScaled(*c2, n, -s2->capsule.radius);
	return dist - s2->capsule.radius;
}

f32 c_HullDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert (s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert (s2->type == C_SHAPE_CONVEX_HULL);

	v3 n;
	struct GJKCache cache;
	return GJK(c1, c2, &n, &cache, NULL, s1, t1, s2, t2, F32_INFINITY);
}

f32 c_TriMeshBvhSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

f32 c_TriMeshBvhCapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

f32 c_TriMeshBvhHullDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

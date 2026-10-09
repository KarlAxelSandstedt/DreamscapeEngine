#include "ds_test.h"
#include "geometry.h"
#include "collision.h"
#include "ds_dynamics.h"

/*
 * GJK and its helpers are static in ds_distance.c (unity-included by ds_collision.c); including it here
 * gives this test its own copy, with access to cache_in, cutoff_distance and cache_out.
 */
#include "../src/math/collision/ds_distance.c"

/*
GJK tests
=========
Every GJK result is checked in double precision against the exact input shapes (the f32 vertices and
transforms), without a reference distance:

    separated (finite d > 0): cache_out names GJK's final simplex. A double precision GJK warm-started
    from it (GjkTestReference) brackets the true distance d* between certified bounds:

        upper bound     d* <= hi = |x|, x the closest point of a simplex of B - A (a point pair)
        lower bound     d* >= lo = max of g(u) = min_B dot(u, b) - max_A dot(u, a) over its directions u,
                        g(n), sat and 0 (a projection onto any unit u is a lower bound)

        d - lo                      <= tol          d doesn't overestimate (undecided if the reference
                                                    didn't converge: hi - lo > 1e-9 * L)
        lo - d                      <= tol          d doesn't underestimate
        |d - |c_b - c_a||           <= tol_p        d agrees with the points

    An overestimate is reported as an early stop if GJK's final simplex, solved in double (d_s), is
    itself above lo, else as f32 error on that simplex. A final simplex can be non-optimal and still
    give d within tol (its distance error is second order in its normal's error), and n can be inaccurate
    for d ~ eps * L or short simplex edges; both are only counted, and the worst normal gap d - g(n) printed.

    overlap (d == 0):   SAT finds no axis separating A and B by more than tol
    cutoff (F32_INFINITY): the cutoff-free result certifies and is >= cutoff - tol

sat is the largest separation of A and B over the SAT axes (face normals of both, cross products of
edge pairs): a lower bound on d*, and > 0 iff A and B are disjoint. Shapes are GJK's vertex sets: a
sphere is its center, a capsule its segment (radii are the callers' business).

Tolerances, with L = max(extent A, extent B, |pos_B - pos_A|): GJK works in A's frame, so its distance
error scales with L; the world-space points add the error of A's world position:

    tol   = GJK_TEST_TOLERANCE * F32_EPSILON * L
    tol_p = GJK_TEST_TOLERANCE * F32_EPSILON * (L + |pos_A|)

Each test prints the worst observed error in units of F32_EPSILON * L, to tighten the tolerance from data.
*/

#define GJK_TEST_TOLERANCE  64.0
#define GJK_TEST_HULL_COUNT 64

/********************************** double precision helpers **********************************/

struct d3
{
    f64 x;
    f64 y;
    f64 z;
};

static struct d3 D3(const f64 x, const f64 y, const f64 z)
{
    struct d3 r = { x, y, z };
    return r;
}

static struct d3 D3V3(const v3 v)
{
    return D3(v.x, v.y, v.z);
}

static struct d3 D3Add(const struct d3 a, const struct d3 b)
{
    return D3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static struct d3 D3Sub(const struct d3 a, const struct d3 b)
{
    return D3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static struct d3 D3Scale(const struct d3 a, const f64 s)
{
    return D3(a.x*s, a.y*s, a.z*s);
}

static f64 D3Dot(const struct d3 a, const struct d3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static struct d3 D3Cross(const struct d3 a, const struct d3 b)
{
    return D3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

/* f32 seed + 2 Newton steps: double precision. Returns 0 for x <= 0 and below the f32 range. */
static f64 F64SqrtTest(const f64 x)
{
    if (!(x > 0.0))
    {
        return 0.0;
    }

    f64 s = (f64) F32Sqrt((f32) x);
    if (s == 0.0)
    {
        return 0.0;
    }
    s = 0.5*(s + x/s);
    s = 0.5*(s + x/s);
    return s;
}

static f64 D3Length(const struct d3 a)
{
    return F64SqrtTest(D3Dot(a, a));
}

static f64 F64MaxTest(const f64 a, const f64 b)
{
    return (a > b) ? a : b;
}

static f64 F64AbsTest(const f64 a)
{
    return (a < 0.0) ? -a : a;
}

/* rows of the rotation matrix of rot, normalized in double */
static void D3Rotation(struct d3 r[3], const q rot)
{
    const f64 l = F64SqrtTest((f64) rot.x*rot.x + (f64) rot.y*rot.y + (f64) rot.z*rot.z + (f64) rot.w*rot.w);
    const f64 x = rot.x / l;
    const f64 y = rot.y / l;
    const f64 z = rot.z / l;
    const f64 w = rot.w / l;
    r[0] = D3(1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w), 2.0*(x*z + y*w));
    r[1] = D3(2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w));
    r[2] = D3(2.0*(x*z - y*w), 2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y));
}

static struct d3 D3Transform(const struct d3 r[3], const struct d3 pos, const v3 p)
{
    const struct d3 l = D3V3(p);
    return D3(D3Dot(r[0], l) + pos.x, D3Dot(r[1], l) + pos.y, D3Dot(r[2], l) + pos.z);
}

static v3 V3D3(const struct d3 a)
{
    return V3((f32) a.x, (f32) a.y, (f32) a.z);
}

/********************************** test shapes **********************************/

/* A GJK input shape and its world-space geometry in double. */
struct gjk_TestShape
{
    struct c_Shape  shape;
    ds_Transform    t;
    struct d3 *     v;          /* world vertices, in GJKVertexSet order                    */
    struct d3 *     face_n;     /* world unit face normals (hulls)                          */
    struct d3 *     edge;       /* world edge directions: one per twin pair, or the segment */
    u32             v_count;
    u32             face_count;
    u32             edge_count;
    f64             extent;     /* max |local vertex|                                       */
};

static void GjkTestShapeInit(struct arena *mem, struct gjk_TestShape *s, const struct c_Shape *shape, const ds_Transform t)
{
    s->shape = *shape;
    s->t = t;
    s->face_count = 0;
    s->edge_count = 0;

    struct d3 r[3];
    D3Rotation(r, t.rotation);
    const struct d3 pos = D3V3(t.position);

    v3 local[2];
    const v3 *local_v = local;
    switch (shape->type)
    {
        case C_SHAPE_SPHERE:
        {
            local[0] = V3Zero();
            s->v_count = 1;
        } break;

        case C_SHAPE_CAPSULE:
        {
            local[0] = V3(0.0f,  shape->capsule.half_height, 0.0f);
            local[1] = V3(0.0f, -shape->capsule.half_height, 0.0f);
            s->v_count = 2;
        } break;

        default:
        {
            local_v = shape->hull.v;
            s->v_count = shape->hull.v_count;
        } break;
    }

    s->v = ArenaPush(mem, s->v_count*sizeof(struct d3));
    s->extent = 0.0;
    for (u32 i = 0; i < s->v_count; ++i)
    {
        s->v[i] = D3Transform(r, pos, local_v[i]);
        s->extent = F64MaxTest(s->extent, D3Length(D3V3(local_v[i])));
    }

    if (shape->type == C_SHAPE_CAPSULE)
    {
        s->edge = ArenaPush(mem, sizeof(struct d3));
        s->edge[0] = D3Sub(s->v[0], s->v[1]);
        s->edge_count = (shape->capsule.half_height > 0.0f) ? 1 : 0;
    }
    else if (shape->type == C_SHAPE_CONVEX_HULL)
    {
        const struct dcel *h = &shape->hull;

        /* Newell: sum of cross(v_k, v_k+1) over the face polygon = 2 * area * normal */
        s->face_n = ArenaPush(mem, h->f_count*sizeof(struct d3));
        s->face_count = h->f_count;
        for (u32 fi = 0; fi < h->f_count; ++fi)
        {
            struct d3 n = D3(0.0, 0.0, 0.0);
            const struct dcelFace *f = h->f + fi;
            for (u32 k = 0; k < f->count; ++k)
            {
                const u32 i0 = h->e[f->first + k].origin;
                const u32 i1 = h->e[f->first + (k + 1) % f->count].origin;
                n = D3Add(n, D3Cross(s->v[i0], s->v[i1]));
            }
            s->face_n[fi] = D3Scale(n, 1.0 / D3Length(n));
        }

        s->edge = ArenaPush(mem, h->e_count*sizeof(struct d3));
        for (u32 ei = 0; ei < h->e_count; ++ei)
        {
            if (ei < h->e[ei].twin)
            {
                const u32 i0 = h->e[ei].origin;
                const u32 i1 = h->e[h->e[ei].twin].origin;
                s->edge[s->edge_count++] = D3Sub(s->v[i1], s->v[i0]);
            }
        }
    }
}

static void GjkTestProject(f64 *min, f64 *max, const struct gjk_TestShape *s, const struct d3 u)
{
    *min = D3Dot(s->v[0], u);
    *max = *min;
    for (u32 i = 1; i < s->v_count; ++i)
    {
        const f64 p = D3Dot(s->v[i], u);
        *min = (p < *min) ? p : *min;
        *max = (p > *max) ? p : *max;
    }
}

/* separation of A and B along the unit axis u (negative: overlapping projections) */
static f64 GjkTestAxisSeparation(const struct gjk_TestShape *a, const struct gjk_TestShape *b, const struct d3 u)
{
    f64 a_min, a_max, b_min, b_max;
    GjkTestProject(&a_min, &a_max, a, u);
    GjkTestProject(&b_min, &b_max, b, u);
    return F64MaxTest(b_min - a_max, a_min - b_max);
}

/* Largest separation over the SAT axes; A is a hull. Near-parallel edge pairs add no axis. */
static f64 GjkTestSatSeparation(const struct gjk_TestShape *a, const struct gjk_TestShape *b)
{
    f64 sep = -1e300;
    for (u32 i = 0; i < a->face_count; ++i)
    {
        sep = F64MaxTest(sep, GjkTestAxisSeparation(a, b, a->face_n[i]));
    }

    for (u32 i = 0; i < b->face_count; ++i)
    {
        sep = F64MaxTest(sep, GjkTestAxisSeparation(a, b, b->face_n[i]));
    }

    for (u32 i = 0; i < a->edge_count; ++i)
    {
        for (u32 j = 0; j < b->edge_count; ++j)
        {
            const struct d3 u = D3Cross(a->edge[i], b->edge[j]);
            const f64 len_sq = D3Dot(u, u);
            if (len_sq <= 1e-20 * D3Dot(a->edge[i], a->edge[i]) * D3Dot(b->edge[j], b->edge[j]))
            {
                continue;
            }
            sep = F64MaxTest(sep, GjkTestAxisSeparation(a, b, D3Scale(u, 1.0 / F64SqrtTest(len_sq))));
        }
    }

    return sep;
}

/********************************** query check **********************************/

struct gjk_TestStats
{
    u64 query_count;
    u64 overlap_count;
    u64 cutoff_count;
    u64 separated_count;
    f64 max_gap;            /* max (d_s - lo) / (eps L): final simplex optimality */
    f64 max_gap_d;          /* d at max_gap                                     */
    f64 max_round;          /* max |d - d_s| / (eps L)                          */
    f64 max_round_d;        /* d at max_round                                   */
    f64 max_normal_gap;     /* max (d - g) / (eps L), separated                 */
    f64 max_normal_gap_d;   /* d at max_normal_gap                              */
    u64 normal_gap_count;   /* separated results with d - g > tol               */
    u64 simplex_gap_count;  /* separated results with d_s - lo > tol            */
    f64 max_point;          /* max |d - |c_b - c_a|| / (eps (L + |pos_A|)) */
    f64 max_sat;            /* max sat / (eps L), overlaps          */
};

static void GjkTestStatsPrint(const char *id, const struct gjk_TestStats *stats)
{
    fprintf(stdout, "\t%s: %llu queries (%llu separated, %llu overlap, %llu cutoff); worst in eps*L: gap %.3g (at d = %.3g), rounding %.3g (at d = %.3g), points %.3g, overlap sat %.3g; normal gap %.3g (at d = %.3g), %llu above tol; %llu non-optimal final simplices\n",
        id,
        (long long unsigned int) stats->query_count,
        (long long unsigned int) stats->separated_count,
        (long long unsigned int) stats->overlap_count,
        (long long unsigned int) stats->cutoff_count,
        stats->max_gap, stats->max_gap_d, stats->max_round, stats->max_round_d, stats->max_point, stats->max_sat,
        stats->max_normal_gap, stats->max_normal_gap_d, (long long unsigned int) stats->normal_gap_count,
        (long long unsigned int) stats->simplex_gap_count);
}

static void GjkTestShapePrint(const char *name, const struct gjk_TestShape *s)
{
    fprintf(stderr, "\t%s: type %u, %u vertices, extent %.9g, half_height %.9g\n", name, s->shape.type, s->v_count, s->extent,
        (s->shape.type == C_SHAPE_CAPSULE) ? s->shape.capsule.half_height : 0.0f);
    fprintf(stderr, "\t\trotation (%.9g %.9g %.9g %.9g) position (%.9g %.9g %.9g)\n",
        s->t.rotation.x, s->t.rotation.y, s->t.rotation.z, s->t.rotation.w,
        s->t.position.x, s->t.position.y, s->t.position.z);
    if (s->shape.type == C_SHAPE_CONVEX_HULL)
    {
        const struct dcel *h = &s->shape.hull;
        for (u32 i = 0; i < h->v_count; ++i)
        {
            fprintf(stderr, "\t\tv %u: %.9g %.9g %.9g\n", i, h->v[i].x, h->v[i].y, h->v[i].z);
        }
        for (u32 fi = 0; fi < h->f_count; ++fi)
        {
            fprintf(stderr, "\t\tf %u:", fi);
            for (u32 k = 0; k < h->f[fi].count; ++k)
            {
                fprintf(stderr, " %u", h->e[h->f[fi].first + k].origin);
            }
            fprintf(stderr, "\n");
        }
    }
}

/*
 * Distance from the origin to conv(w[0..count)), count <= 4, in double; x is the closest point. The
 * minimum over the vertex subsets whose affine projection of the origin has non-negative weights (the
 * closest point lies in the relative interior of one of them). Singular subsets are skipped: a smaller
 * subset covers them.
 */
static f64 GjkTestSimplexDistance(struct d3 *x, u32 *x_mask, const struct d3 *w, const u32 count)
{
    f64 best = -1.0;
    for (u32 mask = 1; mask < (1u << count); ++mask)
    {
        struct d3 p[4];
        u32 k = 0;
        for (u32 j = 0; j < count; ++j)
        {
            if (mask & (1u << j))
            {
                p[k++] = w[j];
            }
        }

        /*
         * x = p0 + sum_i t_i e_i with e_i = p_i - p0 minimizes |x| over the affine hull where
         *
         *      sum_j dot(e_i, e_j) t_j = -dot(e_i, p0)
         *
         * Gaussian elimination with partial pivoting; weights (1 - sum t, t_1, ...) must be >= 0.
         */
        const u32 m = k - 1;
        struct d3 e[3];
        f64 a[3][4];
        f64 diag_max = 0.0;
        for (u32 i = 0; i < m; ++i)
        {
            e[i] = D3Sub(p[i + 1], p[0]);
        }
        for (u32 i = 0; i < m; ++i)
        {
            for (u32 j = 0; j < m; ++j)
            {
                a[i][j] = D3Dot(e[i], e[j]);
            }
            a[i][m] = -D3Dot(e[i], p[0]);
            diag_max = F64MaxTest(diag_max, a[i][i]);
        }

        u32 singular = 0;
        for (u32 c = 0; c < m && !singular; ++c)
        {
            u32 pivot = c;
            for (u32 r = c + 1; r < m; ++r)
            {
                pivot = (F64AbsTest(a[r][c]) > F64AbsTest(a[pivot][c])) ? r : pivot;
            }
            if (F64AbsTest(a[pivot][c]) <= 1e-13 * diag_max)
            {
                singular = 1;
                break;
            }
            for (u32 j = 0; j <= m; ++j)
            {
                const f64 tmp = a[c][j]; a[c][j] = a[pivot][j]; a[pivot][j] = tmp;
            }
            for (u32 r = c + 1; r < m; ++r)
            {
                const f64 f = a[r][c] / a[c][c];
                for (u32 j = c; j <= m; ++j)
                {
                    a[r][j] -= f*a[c][j];
                }
            }
        }
        if (singular)
        {
            continue;
        }

        f64 t[3];
        f64 t_sum = 0.0;
        u32 negative = 0;
        for (u32 i = m; i-- > 0;)
        {
            f64 v = a[i][m];
            for (u32 j = i + 1; j < m; ++j)
            {
                v -= a[i][j]*t[j];
            }
            t[i] = v / a[i][i];
            t_sum += t[i];
            negative |= (t[i] < 0.0);
        }
        if (negative || 1.0 - t_sum < 0.0)
        {
            continue;
        }

        struct d3 c = p[0];
        for (u32 i = 0; i < m; ++i)
        {
            c = D3Add(c, D3Scale(e[i], t[i]));
        }
        const f64 len = D3Length(c);
        if (best < 0.0 || len < best)
        {
            best = len;
            *x = c;
            *x_mask = mask;
        }
    }

    return best;
}

/*
 * Certified distance bounds lo <= d* <= hi: GJK in double, warm-started from the simplex (index_a, index_b)
 * of GJK's result. Every iteration's support point gives the lower bound g(x / |x|); the closest simplex
 * point the upper bound |x|. Returns hi; stops when hi - lo <= 1e-12 * scale or the support repeats.
 */
static f64 GjkTestReference(f64 *lo, const struct gjk_TestShape *a, const struct gjk_TestShape *b, const struct GJKCache *cache, const f64 scale)
{
    u32 ia[5], ib[5];
    struct d3 w[5];
    u32 count = cache->count;
    for (u32 i = 0; i < count; ++i)
    {
        ia[i] = cache->index_a[i];
        ib[i] = cache->index_b[i];
        w[i] = D3Sub(b->v[ib[i]], a->v[ia[i]]);
    }

    *lo = 0.0;
    f64 hi = 1e300;
    for (u32 iter = 0; iter < 64; ++iter)
    {
        struct d3 x;
        u32 mask;
        hi = GjkTestSimplexDistance(&x, &mask, w, count);
        u32 k = 0;
        for (u32 j = 0; j < count; ++j)
        {
            if (mask & (1u << j))
            {
                ia[k] = ia[j];
                ib[k] = ib[j];
                w[k] = w[j];
                k += 1;
            }
        }
        count = k;

        if (hi == 0.0)
        {
            break;
        }

        /* support of B - A in direction -x: min_B dot(b, u) - max_A dot(a, u), u = x / |x| */
        const struct d3 u = D3Scale(x, 1.0 / hi);
        u32 sa = 0, sb = 0;
        for (u32 j = 1; j < a->v_count; ++j)
        {
            sa = (D3Dot(a->v[j], u) > D3Dot(a->v[sa], u)) ? j : sa;
        }
        for (u32 j = 1; j < b->v_count; ++j)
        {
            sb = (D3Dot(b->v[j], u) < D3Dot(b->v[sb], u)) ? j : sb;
        }
        const struct d3 s = D3Sub(b->v[sb], a->v[sa]);
        *lo = F64MaxTest(*lo, D3Dot(s, u));
        if (hi - *lo <= 1e-12 * scale)
        {
            break;
        }

        u32 repeated = 0;
        for (u32 j = 0; j < count; ++j)
        {
            repeated |= (ia[j] == sa && ib[j] == sb);
        }
        if (repeated || count == 4)
        {
            break;
        }
        ia[count] = sa;
        ib[count] = sb;
        w[count] = s;
        count += 1;
    }

    return hi;
}

static u32 V3NanCheckTest(const v3 v)
{
    return (v.x != v.x) || (v.y != v.y) || (v.z != v.z);
}

/*
 * Run GJK on (a, b) and check the result (see the top of the file). converged = 0 for runs that may hit
 * the iteration limit: the result is then only an upper bound, so the gap isn't checked.
 */
static u32 GjkTestQuery(struct gjk_TestStats *stats, f32 *d_out, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct gjk_TestShape *a, const struct gjk_TestShape *b, const f32 cutoff, const u32 converged)
{
    v3 c_a, c_b, n;
    const f32 d = GJK(&c_a, &c_b, &n, cache_out, cache_in, &a->shape, &a->t, &b->shape, &b->t, cutoff);
    *d_out = d;
    stats->query_count += 1;

    const struct d3 pos_a = D3V3(a->t.position);
    const f64 scale = F64MaxTest(F64MaxTest(a->extent, b->extent), D3Length(D3Sub(D3V3(b->t.position), pos_a)));
    const f64 eps_l = F32_EPSILON * scale;
    const f64 eps_p = F32_EPSILON * (scale + D3Length(pos_a));
    const f64 tol = GJK_TEST_TOLERANCE * eps_l;
    const f64 tol_p = GJK_TEST_TOLERANCE * eps_p;

    const char *failure = NULL;
    f64 gap = 0.0, sat = 0.0, upper = 0.0, g = 0.0, d_s = 0.0, lo = 0.0, hi = 0.0;
    if (d != d || (d != F32_INFINITY && (V3NanCheckTest(c_a) || V3NanCheckTest(c_b) || V3NanCheckTest(n))))
    {
        failure = "NaN output";
    }
    else if (cache_out->count < 1 || 4 < cache_out->count)
    {
        failure = "cache count outside 1..4";
    }
    else
    {
        for (u32 i = 0; i < cache_out->count; ++i)
        {
            if (cache_out->index_a[i] >= a->v_count || cache_out->index_b[i] >= b->v_count)
            {
                failure = "cache index out of range";
            }
        }
    }

    if (failure)
    {
        /* reported below */
    }
    else if (d == F32_INFINITY)
    {
        stats->cutoff_count += 1;
        f32 d_full;
        struct GJKCache cache_full;
        if (cutoff == F32_INFINITY)
        {
            failure = "F32_INFINITY without a cutoff";
        }
        else if (!GjkTestQuery(stats, &d_full, &cache_full, NULL, a, b, F32_INFINITY, converged))
        {
            failure = "cutoff-free rerun failed";
        }
        else if (d_full < cutoff - tol)
        {
            failure = "cutoff exit, but the distance is below the cutoff";
        }
    }
    else if (d == 0.0f)
    {
        stats->overlap_count += 1;
        sat = GjkTestSatSeparation(a, b);
        stats->max_sat = F64MaxTest(stats->max_sat, sat / eps_l);
        if (c_a.x != c_b.x || c_a.y != c_b.y || c_a.z != c_b.z || n.x != 0.0f || n.y != 0.0f || n.z != 0.0f)
        {
            failure = "overlap: c_a != c_b or n != 0";
        }
        else if (sat > tol)
        {
            failure = "overlap, but SAT separates";
        }
    }
    else
    {
        stats->separated_count += 1;
        const struct d3 nd = D3V3(n);
        const f64 n_len = D3Length(nd);
        upper = D3Length(D3Sub(D3V3(c_b), D3V3(c_a)));
        f64 a_min, a_max, b_min, b_max;
        GjkTestProject(&a_min, &a_max, a, D3Scale(nd, 1.0 / n_len));
        GjkTestProject(&b_min, &b_max, b, D3Scale(nd, 1.0 / n_len));
        g = b_min - a_max;

        struct d3 w[4], x_s;
        u32 x_mask;
        for (u32 i = 0; i < cache_out->count; ++i)
        {
            w[i] = D3Sub(b->v[cache_out->index_b[i]], a->v[cache_out->index_a[i]]);
        }
        d_s = GjkTestSimplexDistance(&x_s, &x_mask, w, cache_out->count);
        sat = GjkTestSatSeparation(a, b);
        hi = GjkTestReference(&lo, a, b, cache_out, scale);
        lo = F64MaxTest(F64MaxTest(F64MaxTest(lo, g), sat), 0.0);
        gap = d_s - lo;
        const f64 round = F64AbsTest(d - d_s);
        const f64 normal_gap = d - g;
        stats->simplex_gap_count += (converged && gap > tol);

        const f64 point_err = F64AbsTest(d - upper);
        stats->max_point = F64MaxTest(stats->max_point, point_err / eps_p);
        if (converged && gap / eps_l > stats->max_gap)
        {
            stats->max_gap = gap / eps_l;
            stats->max_gap_d = d;
        }
        if (converged && round / eps_l > stats->max_round)
        {
            stats->max_round = round / eps_l;
            stats->max_round_d = d;
        }
        if (converged && normal_gap / eps_l > stats->max_normal_gap)
        {
            stats->max_normal_gap = normal_gap / eps_l;
            stats->max_normal_gap_d = d;
        }
        stats->normal_gap_count += (converged && normal_gap > tol);

        if (d < 0.0f)
        {
            failure = "negative distance";
        }
        else if (F64AbsTest(n_len - 1.0) > 1e-5)
        {
            failure = "normal not unit";
        }
        else if (point_err > tol_p)
        {
            failure = "d != |c_b - c_a|";
        }
        else if (converged && d - lo > tol)
        {
            failure = (hi - lo > 1e-9 * scale)
                    ? "undecided: d - lo > tol, but the reference didn't converge (test limitation)"
                    : (d_s - lo > tol)
                    ? "d overestimates the distance: early stop (final simplex not the closest pair)"
                    : "d overestimates the distance: f32 error on the final simplex";
        }
        else if (lo > d + tol)
        {
            failure = "d underestimates the distance";
        }
    }

    if (failure)
    {
        fprintf(stderr, "GJK check failed: %s\n", failure);
        fprintf(stderr, "\td %.9g cutoff %.9g cache_in %s; reference [%.9g, %.9g] d_s %.9g g %.9g |c_b - c_a| %.9g sat %.9g tol %.9g tol_p %.9g simplex %u\n",
            d, cutoff, (cache_in) ? "yes" : "no", lo, hi, d_s, g, upper, sat, tol, tol_p, cache_out->count);
        fprintf(stderr, "\tc_a (%.9g %.9g %.9g) c_b (%.9g %.9g %.9g) n (%.9g %.9g %.9g)\n",
            c_a.x, c_a.y, c_a.z, c_b.x, c_b.y, c_b.z, n.x, n.y, n.z);
        GjkTestShapePrint("A", a);
        GjkTestShapePrint("B", b);
    }

    return (failure == NULL);
}

/********************************** generators **********************************/

static struct ds_NumericsConfig g_gjk_test_config;

static void GjkTestConfigPush(const u32 gjk_max_iterations)
{
    g_gjk_test_config = ds_NumericsConfigDefault();
    if (gjk_max_iterations)
    {
        g_gjk_test_config.gjk_max_iterations_pending = gjk_max_iterations;
    }
    ds_NumericsConfigPush(&g_gjk_test_config);
}

static v3 GjkTestUnit(void)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 r = F32Sqrt(1.0f - z*z);
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    return V3(r*F32Cos(theta), r*F32Sin(theta), z);
}

static q GjkTestRotation(void)
{
    return QUnitAxisAngle(GjkTestUnit(), RngF32Range(0.0f, 2.0f*F32_PI));
}

/* rotation taking (0,1,0) to the unit vector t */
static q GjkTestRotationFromY(const v3 t)
{
    if (t.y < -0.999f)
    {
        return Q(1.0f, 0.0f, 0.0f, 0.0f);
    }
    return QNormalize(Q(t.z, 0.0f, -t.x, 1.0f + t.y));
}

/* sizes 1e-2 .. 3e2 */
static f32 GjkTestSize(void)
{
    const f32 decade[] = { 1e-2f, 1e-1f, 1.0f, 1e1f, 1e2f };
    return decade[RngU64Range(0, 4)] * RngF32Range(1.0f, 3.0f);
}

/* world offsets 0 .. 1e3 */
static v3 GjkTestWorldPosition(void)
{
    const f32 range[] = { 0.0f, 1.0f, 1e2f, 1e3f };
    return V3Scale(GjkTestUnit(), range[RngU64Range(0, 3)] * RngF32Normalized());
}

static struct c_Shape GjkTestRandomHull(struct arena *mem)
{
    const f32 size = GjkTestSize();
    v3 p[24];
    struct c_Shape shape = { .type = C_SHAPE_CONVEX_HULL };
    do
    {
        const u32 count = (u32) RngU64Range(4, 24);
        const v3 axis_scale = V3(size*RngF32Range(0.2f, 1.0f), size*RngF32Range(0.2f, 1.0f), size*RngF32Range(0.2f, 1.0f));
        for (u32 i = 0; i < count; ++i)
        {
            const v3 u = V3Scale(GjkTestUnit(), RngF32Range(0.5f, 1.0f));
            p[i] = V3(u.x*axis_scale.x, u.y*axis_scale.y, u.z*axis_scale.z);
        }
        shape.hull = DcelConvexHull(mem, p, count, 100.0f*F32_EPSILON*size);
    } while (shape.hull.v_count < 4);

    return shape;
}

static struct c_Shape *GjkTestHullPool(struct arena *mem)
{
    struct c_Shape *pool = ArenaPush(mem, GJK_TEST_HULL_COUNT*sizeof(struct c_Shape));
    for (u32 i = 0; i < GJK_TEST_HULL_COUNT; ++i)
    {
        pool[i] = GjkTestRandomHull(mem);
    }
    return pool;
}

static ds_Transform GjkTestTransform(const q rotation, const v3 position)
{
    ds_Transform t = { .rotation = rotation, .position = position };
    return t;
}

/*
 * A world point relative to hull A: far, near a vertex, near a face (both sides), or inside. Covers the
 * boundary cases of the inside test (cutoff 0).
 */
static v3 GjkTestPointNear(const struct gjk_TestShape *a, const u32 mode)
{
    const f64 offset_scale[] = { 1e-6, 1e-5, 1e-4, 1e-3, 1e-2 };
    const f64 offset = a->extent * offset_scale[RngU64Range(0, 4)] * ((RngU64Range(0, 1)) ? 1.0 : -1.0);
    switch (mode)
    {
        case 0:
        {
            return V3Add(a->t.position, V3Scale(GjkTestUnit(), (f32) a->extent * RngF32Range(0.0f, 2.0f)));
        }

        case 1:
        {
            const struct d3 v = a->v[RngU64Range(0, a->v_count - 1)];
            return V3D3(D3Add(v, D3Scale(D3V3(GjkTestUnit()), offset)));
        }

        case 2:
        {
            const struct dcel *h = &a->shape.hull;
            const u32 fi = (u32) RngU64Range(0, h->f_count - 1);
            const struct dcelFace *f = h->f + fi;
            struct d3 c = D3(0.0, 0.0, 0.0);
            for (u32 k = 0; k < f->count; ++k)
            {
                c = D3Add(c, a->v[h->e[f->first + k].origin]);
            }
            c = D3Scale(c, 1.0 / f->count);
            return V3D3(D3Add(c, D3Scale(a->face_n[fi], offset)));
        }

        default:
        {
            const struct d3 v0 = a->v[RngU64Range(0, a->v_count - 1)];
            const struct d3 v1 = a->v[RngU64Range(0, a->v_count - 1)];
            const struct d3 c = D3V3(a->t.position);
            const f64 s = RngF32Range(0.0f, 0.9f);
            return V3D3(D3Add(c, D3Scale(D3Sub(D3Scale(D3Add(v0, v1), 0.5), c), s)));
        }
    }
}

static f32 GjkTestCutoff(const struct gjk_TestShape *a, const u32 i)
{
    switch (i % 3)
    {
        case 0: return F32_INFINITY;
        case 1: return 0.0f;
        default: return (f32) a->extent * RngF32Range(0.0f, 1.0f);
    }
}

/********************************** tests **********************************/

/* The double rotation must match the engine's, or every check compares different shapes. */
struct test_Output GjkTestRotationConventionTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 i = 0; i < 1000; ++i)
    {
        const q rot = GjkTestRotation();
        const v3 p = V3Scale(GjkTestUnit(), RngF32Range(0.0f, 10.0f));
        struct d3 r[3];
        D3Rotation(r, rot);
        const struct d3 expected = D3Transform(r, D3(0.0, 0.0, 0.0), p);
        const struct d3 actual = D3V3(QV3Rotate(rot, p));
        if (D3Length(D3Sub(expected, actual)) > 1e-5)
        {
            TEST_FAILURE;
        }
    }

	return output;
}

/* hull vs point, including the inside test (cutoff 0) of c_HullCapsuleContact's deep path */
struct test_Output GjkHullPointTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    GjkTestConfigPush(0);

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape *hull = GjkTestHullPool(env->mem_1);
    const struct c_Shape point = { .type = C_SHAPE_SPHERE };
    for (u32 i = 0; i < 40000; ++i)
    {
        struct arena record = *env->mem_2;
        struct gjk_TestShape a, b;
        GjkTestShapeInit(env->mem_2, &a, hull + (i % GJK_TEST_HULL_COUNT), GjkTestTransform(GjkTestRotation(), GjkTestWorldPosition()));
        GjkTestShapeInit(env->mem_2, &b, &point, GjkTestTransform(QIdentity(), GjkTestPointNear(&a, (u32) RngU64Range(0, 3))));

        f32 d;
        struct GJKCache cache;
        const u32 ok = GjkTestQuery(&stats, &d, &cache, NULL, &a, &b, GjkTestCutoff(&a, i), 1);
        *env->mem_2 = record;
        if (!ok)
        {
            TEST_FAILURE;
        }
    }

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

/* hull vs segment (capsule), including zero-length segments and segments parallel to a face */
struct test_Output GjkHullSegmentTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    GjkTestConfigPush(0);

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape *hull = GjkTestHullPool(env->mem_1);
    for (u32 i = 0; i < 40000; ++i)
    {
        struct arena record = *env->mem_2;
        struct gjk_TestShape a, b;
        GjkTestShapeInit(env->mem_2, &a, hull + (i % GJK_TEST_HULL_COUNT), GjkTestTransform(GjkTestRotation(), GjkTestWorldPosition()));

        struct c_Shape capsule = { .type = C_SHAPE_CAPSULE };
        capsule.capsule.half_height = (i % 10 == 0) ? 0.0f : (f32) a.extent * RngF32Range(0.0f, 1.0f);
        capsule.capsule.radius = 0.0f;

        q rotation = GjkTestRotation();
        if (i % 4 == 0)
        {
            /* axis parallel to a random face of A */
            const struct d3 n = a.face_n[RngU64Range(0, a.face_count - 1)];
            const struct d3 t = D3Cross(n, D3V3(GjkTestUnit()));
            rotation = GjkTestRotationFromY(V3D3(D3Scale(t, 1.0 / D3Length(t))));
        }
        GjkTestShapeInit(env->mem_2, &b, &capsule, GjkTestTransform(rotation, GjkTestPointNear(&a, (u32) RngU64Range(0, 3))));

        f32 d;
        struct GJKCache cache;
        const u32 ok = GjkTestQuery(&stats, &d, &cache, NULL, &a, &b, GjkTestCutoff(&a, i), 1);
        *env->mem_2 = record;
        if (!ok)
        {
            TEST_FAILURE;
        }
    }

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

/* hull vs hull (c_HullDistance) */
struct test_Output GjkHullHullTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    GjkTestConfigPush(0);

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape *hull = GjkTestHullPool(env->mem_1);
    for (u32 i = 0; i < 20000; ++i)
    {
        struct arena record = *env->mem_2;
        struct gjk_TestShape a, b;
        GjkTestShapeInit(env->mem_2, &a, hull + (i % GJK_TEST_HULL_COUNT), GjkTestTransform(GjkTestRotation(), GjkTestWorldPosition()));

        const struct c_Shape *hull_b = hull + RngU64Range(0, GJK_TEST_HULL_COUNT - 1);
        f64 extent_b = 0.0;
        for (u32 j = 0; j < hull_b->hull.v_count; ++j)
        {
            extent_b = F64MaxTest(extent_b, D3Length(D3V3(hull_b->hull.v[j])));
        }
        const v3 position = V3Add(a.t.position, V3Scale(GjkTestUnit(), (f32) (a.extent + extent_b) * RngF32Range(0.0f, 1.5f)));
        GjkTestShapeInit(env->mem_2, &b, hull_b, GjkTestTransform(GjkTestRotation(), position));

        f32 d;
        struct GJKCache cache;
        const u32 ok = GjkTestQuery(&stats, &d, &cache, NULL, &a, &b, GjkTestCutoff(&a, i), 1);
        *env->mem_2 = record;
        if (!ok)
        {
            TEST_FAILURE;
        }
    }

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

/* hand-made degenerate configurations, with their exact distances */
struct test_Output GjkDegenerateTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    GjkTestConfigPush(0);

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const struct c_Shape slab = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1e-4f)) };
    const struct c_Shape tiny = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1e-3f, 1e-3f, 1e-3f)) };
    const struct c_Shape point = { .type = C_SHAPE_SPHERE };
    const struct c_Shape segment = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 0.7f } };
    const struct c_Shape segment_zero = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 0.0f } };
    const q rot_x = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), F32_PI / 2.0f);   /* (0,1,0) -> (-1,0,0) */
    const ds_Transform origin = GjkTestTransform(QIdentity(), V3Zero());

    struct
    {
        const char *            id;
        const struct c_Shape *  a;
        ds_Transform            t_a;
        const struct c_Shape *  b;
        ds_Transform            t_b;
        f32                     d;      /* exact distance */
    } c[] =
    {
        { "point on a face",                &box,   origin, &point,         GjkTestTransform(QIdentity(), V3(1.0f, 0.3f, 0.2f)), 0.0f },
        { "point on a vertex",              &box,   origin, &point,         GjkTestTransform(QIdentity(), V3(1.0f, 1.0f, 1.0f)), 0.0f },
        { "point on an edge",               &box,   origin, &point,         GjkTestTransform(QIdentity(), V3(1.0f, 1.0f, 0.5f)), 0.0f },
        { "point at the center",            &box,   origin, &point,         origin, 0.0f },
        { "point off a vertex",             &box,   origin, &point,         GjkTestTransform(QIdentity(), V3(2.0f, 2.0f, 2.0f)), 1.7320508f },
        { "identical boxes",                &box,   origin, &box,           origin, 0.0f },
        { "boxes face to face, touching",   &box,   origin, &box,           GjkTestTransform(QIdentity(), V3(2.0f, 0.0f, 0.0f)), 0.0f },
        { "boxes face to face, 1e-6 gap",   &box,   origin, &box,           GjkTestTransform(QIdentity(), V3(2.000001f, 0.0f, 0.0f)), 9.5367432e-7f },
        { "boxes face to face, offset",     &box,   origin, &box,           GjkTestTransform(QIdentity(), V3(2.5f, 1.3f, -0.4f)), 0.5f },
        { "segment parallel to a face",     &box,   origin, &segment,       GjkTestTransform(QIdentity(), V3(1.5f, 0.0f, 0.0f)), 0.5f },
        { "segment along a face normal",    &box,   origin, &segment,       GjkTestTransform(rot_x, V3(2.0f, 0.0f, 0.0f)), 0.3f },
        { "segment through the box",        &box,   origin, &segment,       GjkTestTransform(rot_x, V3(0.0f, 0.0f, 0.0f)), 0.0f },
        { "zero-length segment",            &box,   origin, &segment_zero,  GjkTestTransform(QIdentity(), V3(0.0f, 3.0f, 0.0f)), 2.0f },
        { "thin slab, point above",         &slab,  origin, &point,         GjkTestTransform(QIdentity(), V3(0.2f, 0.1f, 0.5f)), 0.4999f },
        { "thin slab, point inside",        &slab,  origin, &point,         GjkTestTransform(QIdentity(), V3(0.2f, 0.1f, 0.0f)), 0.0f },
        { "tiny box far out",               &tiny,  GjkTestTransform(QIdentity(), V3(1000.0f, 0.0f, 0.0f)), &point, GjkTestTransform(QIdentity(), V3(1000.0f, 0.002f, 0.0f)), 0.001f },
    };

    for (u32 i = 0; i < sizeof(c) / sizeof(c[0]); ++i)
    {
        struct arena record = *env->mem_2;
        struct gjk_TestShape a, b;
        GjkTestShapeInit(env->mem_2, &a, c[i].a, c[i].t_a);
        GjkTestShapeInit(env->mem_2, &b, c[i].b, c[i].t_b);

        f32 d;
        struct GJKCache cache;
        const u32 ok = GjkTestQuery(&stats, &d, &cache, NULL, &a, &b, F32_INFINITY, 1);

        const f64 scale = F64MaxTest(F64MaxTest(a.extent, b.extent), D3Length(D3Sub(D3V3(b.t.position), D3V3(a.t.position))));
        const f64 tol = GJK_TEST_TOLERANCE * F32_EPSILON * scale;
        const u32 exact = F64AbsTest((f64) d - (f64) c[i].d) <= tol;
        if (!exact)
        {
            fprintf(stderr, "GJK degenerate case \"%s\": d %.9g, expected %.9g (tol %.3g)\n", c[i].id, d, c[i].d, tol);
        }
        *env->mem_2 = record;
        if (!ok || !exact)
        {
            TEST_FAILURE;
        }
    }

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

/*
 * Warm starts: B moves in small steps and every step feeds cache_out back as cache_in, with an occasional
 * jump that changes the cached simplex by more than 2x (flush). Each step is also run without the cache.
 */
struct test_Output GjkCacheTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    GjkTestConfigPush(0);

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape *hull = GjkTestHullPool(env->mem_1);
    const struct c_Shape point = { .type = C_SHAPE_SPHERE };
    for (u32 sequence = 0; sequence < 600; ++sequence)
    {
        const struct c_Shape *shape_a = hull + (sequence % GJK_TEST_HULL_COUNT);
        struct c_Shape shape_b;
        switch (sequence % 3)
        {
            case 0: { shape_b = point; } break;
            case 1:
            {
                shape_b = (struct c_Shape) { .type = C_SHAPE_CAPSULE };
                shape_b.capsule.half_height = GjkTestSize();
            } break;
            default: { shape_b = hull[RngU64Range(0, GJK_TEST_HULL_COUNT - 1)]; } break;
        }

        const ds_Transform t_a = GjkTestTransform(GjkTestRotation(), GjkTestWorldPosition());
        struct arena record = *env->mem_2;
        struct gjk_TestShape a, b;
        GjkTestShapeInit(env->mem_2, &a, shape_a, t_a);

        const f32 step = 0.02f * (f32) a.extent;
        ds_Transform t_b = GjkTestTransform(GjkTestRotation(), GjkTestPointNear(&a, 0));
        struct GJKCache cache[2];
        u32 cache_valid = 0;
        for (u32 i = 0; i < 60; ++i)
        {
            struct arena record_step = *env->mem_2;
            if (i % 20 == 19)
            {
                t_b.position = GjkTestPointNear(&a, 0);
                t_b.rotation = GjkTestRotation();
            }
            else
            {
                t_b.position = V3Add(t_b.position, V3Scale(GjkTestUnit(), step));
                t_b.rotation = QNormalize(QMul(QUnitAxisAngle(GjkTestUnit(), 0.02f), t_b.rotation));
            }
            GjkTestShapeInit(env->mem_2, &b, &shape_b, t_b);

            f32 d_cached, d_fresh;
            struct GJKCache cache_fresh;
            const f32 cutoff = (i % 2) ? F32_INFINITY : (f32) a.extent * RngF32Range(0.0f, 1.0f);
            const u32 ok_cached = GjkTestQuery(&stats, &d_cached, &cache[i % 2], (cache_valid) ? &cache[(i + 1) % 2] : NULL, &a, &b, cutoff, 1);
            const u32 ok_fresh = GjkTestQuery(&stats, &d_fresh, &cache_fresh, NULL, &a, &b, cutoff, 1);
            cache_valid = 1;
            *env->mem_2 = record_step;
            if (!ok_cached || !ok_fresh)
            {
                fprintf(stderr, "\tsequence %u step %u\n", sequence, i);
                TEST_FAILURE;
            }
        }
        *env->mem_2 = record;
    }

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

/* Low iteration limits: results are upper bounds, never NaN, and overlaps are real. */
struct test_Output GjkIterationLimitTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct gjk_TestStats stats = { 0 };
    const struct c_Shape *hull = GjkTestHullPool(env->mem_1);
    for (u32 max_iterations = 1; max_iterations <= 3; ++max_iterations)
    {
        GjkTestConfigPush(max_iterations);
        for (u32 i = 0; i < 5000; ++i)
        {
            struct arena record = *env->mem_2;
            struct gjk_TestShape a, b;
            GjkTestShapeInit(env->mem_2, &a, hull + (i % GJK_TEST_HULL_COUNT), GjkTestTransform(GjkTestRotation(), GjkTestWorldPosition()));
            const v3 position = V3Add(a.t.position, V3Scale(GjkTestUnit(), (f32) a.extent * RngF32Range(0.0f, 3.0f)));
            GjkTestShapeInit(env->mem_2, &b, hull + RngU64Range(0, GJK_TEST_HULL_COUNT - 1), GjkTestTransform(GjkTestRotation(), position));

            f32 d;
            struct GJKCache cache;
            const u32 ok = GjkTestQuery(&stats, &d, &cache, NULL, &a, &b, F32_INFINITY, 0);
            *env->mem_2 = record;
            if (!ok)
            {
                GjkTestConfigPush(0);
                TEST_FAILURE;
            }
        }
    }
    GjkTestConfigPush(0);

    GjkTestStatsPrint(__func__, &stats);
	return output;
}

static struct test_Output (*gjk_tests[])(struct test_Environment *) =
{
    GjkTestRotationConventionTest,
    GjkDegenerateTest,
	GjkHullPointTest,
	GjkHullSegmentTest,
	GjkHullHullTest,
	GjkCacheTest,
	GjkIterationLimitTest,
};

struct suite_Correctness m_gjk_suite =
{
	.id = "gjk",
	.unit_test = gjk_tests,
	.unit_test_count = sizeof(gjk_tests) / sizeof(gjk_tests[0]),
};

struct suite_Correctness *gjk_correctness_suite = &m_gjk_suite;

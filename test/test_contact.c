#include "ds_test.h"
#include "geometry.h"
#include "collision.h"
#include "ds_dynamics.h"

/*
Contact tests
=============
Capsule vs capsule (TMP_c_CapsuleContact), checked in double precision against the exact inputs (the f32
shapes and rotations, the f64 positions). The reference is the distance between the two cores:

    dist* = min over the unit square of a convex quadratic: the unconstrained minimum if it lies inside,
            else the minimum over the 4 edges (endpoint vs segment distances)

Every manifold (world space, double) must satisfy, with L = max(h0, h1, r0, r1, |x1 - x0|) and
tol = CONTACT_TEST_TOLERANCE * F32_EPSILON * L:

    contact iff dist* <= r_sum                  (undecided within tol)
    c_ManifoldCheck, v_count in {1, 2}, |n| = 1
    v_i on capsule 0's surface                  |dist(v_i, core 0) - r0| <= tol
    1 point: depth_0 = r_sum - dist*            within tol
    n separates as deep as it claims            depth_i <= -sep(n) + tol, sep(n) = min_1 x.n - max_0 x.n - r_sum
    n is optimal (dist* well above 0)           -sep(n) <= r_sum - dist* + tol * (1 + L / dist*)
                                                (sep(n) <= dist* - r_sum holds for any n, so n is optimal
                                                iff sep(n) reaches it; a normal error e costs ~ e * L)

Other checks: swapping the capsules gives the same contact and deepest depth, and the opposite normal
(when dist* is well above 0: otherwise the normal is a fallback); translating both by an exactly
representable world offset gives identical bits.
*/

#define CONTACT_TEST_TOLERANCE  64.0

/* the contacts read thresholds from the global numerics config */
static struct ds_NumericsConfig g_contact_test_config;

static void ContactTestConfigPush(void)
{
    g_contact_test_config = ds_NumericsConfigDefault();
    ds_NumericsConfigPush(&g_contact_test_config);
}

struct d3 { f64 x, y, z; };

static struct d3 D3(const f64 x, const f64 y, const f64 z) { struct d3 r = { x, y, z }; return r; }
static struct d3 D3V3(const v3 v) { return D3(v.x, v.y, v.z); }
static struct d3 D3Add(const struct d3 a, const struct d3 b) { return D3(a.x + b.x, a.y + b.y, a.z + b.z); }
static struct d3 D3Sub(const struct d3 a, const struct d3 b) { return D3(a.x - b.x, a.y - b.y, a.z - b.z); }
static struct d3 D3Scale(const struct d3 a, const f64 s) { return D3(s*a.x, s*a.y, s*a.z); }
static f64 D3Dot(const struct d3 a, const struct d3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

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

static f64 D3Length(const struct d3 a) { return F64SqrtTest(D3Dot(a, a)); }
static f64 F64MaxTest(const f64 a, const f64 b) { return (a > b) ? a : b; }
static f64 F64MinTest(const f64 a, const f64 b) { return (a < b) ? a : b; }
static f64 F64AbsTest(const f64 a) { return (a < 0.0) ? -a : a; }
static f64 F64ClampTest(const f64 a, const f64 lo, const f64 hi) { return F64MinTest(hi, F64MaxTest(lo, a)); }

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

static struct d3 D3RotationMul(const struct d3 r[3], const struct d3 v)
{
    return D3(D3Dot(r[0], v), D3Dot(r[1], v), D3Dot(r[2], v));
}

/********************************** test capsules **********************************/

struct ct_Capsule
{
    struct c_Shape      shape;
    ds_TransformWorld   t;
    struct d3           r[3];       /* rotation (rows), double */
    struct d3           pos;        /* world position, double */
    struct d3           p0, p1;     /* world core endpoints, double */
};

static void CtCapsuleInit(struct ct_Capsule *c, const f32 half_height, const f32 radius, const q rotation, const struct d3 pos)
{
    c->shape = (struct c_Shape) { .type = C_SHAPE_CAPSULE };
    c->shape.capsule.half_height = half_height;
    c->shape.capsule.radius = radius;
    c->t.rotation = rotation;
    c->t.position = V3F64(pos.x, pos.y, pos.z);
    c->pos = pos;
    D3Rotation(c->r, rotation);
    const struct d3 axis = D3RotationMul(c->r, D3(0.0, half_height, 0.0));
    c->p0 = D3Sub(pos, axis);
    c->p1 = D3Add(pos, axis);
}

/* closest point parameter on segment [a, b] to p (0 for a zero-length segment) */
static f64 CtSegmentParameter(const struct d3 a, const struct d3 b, const struct d3 p)
{
    const struct d3 ab = D3Sub(b, a);
    const f64 ab_sq = D3Dot(ab, ab);
    return (ab_sq > 0.0) ? F64ClampTest(D3Dot(D3Sub(p, a), ab) / ab_sq, 0.0, 1.0) : 0.0;
}

static f64 CtPointSegmentDistance(const struct d3 a, const struct d3 b, const struct d3 p)
{
    const f64 t = CtSegmentParameter(a, b, p);
    return D3Length(D3Sub(p, D3Add(a, D3Scale(D3Sub(b, a), t))));
}

/* distance between segments [a0, a1] and [b0, b1] */
static f64 CtSegmentDistance(const struct d3 a0, const struct d3 a1, const struct d3 b0, const struct d3 b1)
{
    f64 dist = CtPointSegmentDistance(b0, b1, a0);
    dist = F64MinTest(dist, CtPointSegmentDistance(b0, b1, a1));
    dist = F64MinTest(dist, CtPointSegmentDistance(a0, a1, b0));
    dist = F64MinTest(dist, CtPointSegmentDistance(a0, a1, b1));

    /* interior minimum of |a0 + s da - b0 - t db|^2 */
    const struct d3 da = D3Sub(a1, a0);
    const struct d3 db = D3Sub(b1, b0);
    const struct d3 w = D3Sub(a0, b0);
    const f64 aa = D3Dot(da, da);
    const f64 bb = D3Dot(db, db);
    const f64 ab = D3Dot(da, db);
    const f64 det = aa*bb - ab*ab;
    if (det > 1e-12*aa*bb)
    {
        const f64 aw = D3Dot(da, w);
        const f64 bw = D3Dot(db, w);
        const f64 s = (ab*bw - bb*aw) / det;
        const f64 t = (aa*bw - ab*aw) / det;
        if (0.0 < s && s < 1.0 && 0.0 < t && t < 1.0)
        {
            dist = F64MinTest(dist, D3Length(D3Sub(D3Add(a0, D3Scale(da, s)), D3Add(b0, D3Scale(db, t)))));
        }
    }

    return dist;
}

/* a manifold of TMP_c_CapsuleContact in world space (double) */
struct ct_Manifold
{
    u32         contact;
    u32         v_count;
    struct d3   n;
    struct d3   v[2];
    f64         depth[2];
    struct c_Manifold local;
};

static u32 CtContact(struct arena *frame, struct ct_Manifold *m, const struct ct_Capsule *a, const struct ct_Capsule *b)
{
    const struct c_Shape *s[2] = { &a->shape, &b->shape };
    const ds_TransformWorld t[2] = { a->t, b->t };
    const struct c_ContactResult result = TMP_c_CapsuleContact(frame, NULL, s, t);

    *m = (struct ct_Manifold) { 0 };
    if (result.manifold_count == 0)
    {
        return 1;
    }

    if (result.manifold_count != 1 || !c_ManifoldCheck(result.manifold)
            || result.manifold->v_count < 1 || result.manifold->v_count > 2)
    {
        fprintf(stderr, "CtContact: invalid manifold (count %u)\n", result.manifold_count);
        if (result.manifold_count)
        {
            c_ManifoldDebugPrint(result.manifold);
        }
        return 0;
    }

    m->contact = 1;
    m->local = *result.manifold;
    m->v_count = result.manifold->v_count;
    m->n = D3RotationMul(a->r, D3V3(result.manifold->n));
    for (u32 i = 0; i < m->v_count; ++i)
    {
        m->v[i] = D3Add(a->pos, D3RotationMul(a->r, D3V3(result.manifold->v[i])));
        m->depth[i] = result.manifold->depth[i];
    }

    return 1;
}

struct ct_Stats
{
    u64 tests;
    u64 contacts;
    u64 two_point;
    u64 undecided;
    u64 two_point_steep;    /* two point with |n.y| > 0.1 in capsule 0's frame (n not across the axes) */
    f64 steep_dist;         /* largest dist* / L of those */
    f64 worst_depth;        /* 1 point: |depth - (r_sum - dist*)| / (eps L) */
    f64 worst_surface;      /* |dist(v, core 0) - r0| / (eps L) */
    f64 worst_normal;       /* (-sep(n) - (r_sum - dist*)) / (eps L), n optimality */
    f64 worst_swap;         /* |deepest(a, b) - deepest(b, a)| / (eps L) */
    f64 worst_inset;        /* two point: depth of v below capsule 0's surface / r0 (<= 1 - sqrt(1 - n.y^2)) */
    f64 worst_ny;           /* two point: |n.y| in capsule 0's frame */
};

static void CtStatsPrint(const char *id, const struct ct_Stats *s)
{
    fprintf(stdout, "\t%s: %llu tests, %llu contacts (%llu two point), %llu undecided, %llu steep two point (dist* / L <= %.3g); worst / (eps L): depth %.1f, surface %.1f, normal %.1f, swap %.1f; two point: |n.y| %.3g, inset / r0 %.3g\n",
        id, (unsigned long long) s->tests, (unsigned long long) s->contacts, (unsigned long long) s->two_point,
        (unsigned long long) s->undecided, (unsigned long long) s->two_point_steep, s->steep_dist, s->worst_depth, s->worst_surface, s->worst_normal, s->worst_swap,
        s->worst_ny, s->worst_inset);
}

static void CtCapsulePrint(const char *name, const struct ct_Capsule *c)
{
    fprintf(stderr, "\t%s: h %.9g r %.9g rot { %.9g, %.9g, %.9g, %.9g } pos { %.17g, %.17g, %.17g }\n", name,
        c->shape.capsule.half_height, c->shape.capsule.radius, c->t.rotation.x, c->t.rotation.y, c->t.rotation.z,
        c->t.rotation.w, c->pos.x, c->pos.y, c->pos.z);
}

static f64 CtDeepest(const struct ct_Manifold *m)
{
    f64 deepest = 0.0;
    for (u32 i = 0; i < m->v_count; ++i)
    {
        deepest = F64MaxTest(deepest, m->depth[i]);
    }
    return deepest;
}

/* separation of the capsules along unit n (from a towards b) */
static f64 CtSeparation(const struct ct_Capsule *a, const struct ct_Capsule *b, const struct d3 n)
{
    const f64 max_a = F64MaxTest(D3Dot(a->p0, n), D3Dot(a->p1, n));
    const f64 min_b = F64MinTest(D3Dot(b->p0, n), D3Dot(b->p1, n));
    return min_b - max_a - ((f64) a->shape.capsule.radius + (f64) b->shape.capsule.radius);
}

/* checks one manifold of (a, b) against the reference; prints the failure */
static u32 CtVerify(struct ct_Stats *stats, const struct ct_Manifold *m, const struct ct_Capsule *a, const struct ct_Capsule *b, const f64 dist, const f64 L)
{
    const f64 eps_l = F32_EPSILON * L;
    const f64 tol = CONTACT_TEST_TOLERANCE * eps_l;
    const f64 r0 = a->shape.capsule.radius;
    const f64 r_sum = r0 + (f64) b->shape.capsule.radius;
    const char *fail = NULL;

    if (F64AbsTest(dist - r_sum) <= tol)
    {
        stats->undecided += 1;
    }
    else if (m->contact != (dist < r_sum))
    {
        fail = "contact";
    }

    if (!fail && m->contact)
    {
        const f64 n_len = D3Length(m->n);
        const f64 sep = CtSeparation(a, b, D3Scale(m->n, 1.0 / n_len));
        if (F64AbsTest(n_len - 1.0) > 1e-5)
        {
            fail = "|n| != 1";
        }

        /*
         * Two point: v_i = p_i + r0 n with p_i beside the stretch's end, not the closest point, so for n
         * tilted from capsule 0's axis' normal plane (n.y != 0) v_i lies r0 (1 - sqrt(1 - n.y^2)) inside.
         */
        f64 inset = 0.0;
        if (m->v_count == 2)
        {
            const f64 ny = m->local.n.y;
            inset = r0 * (1.0 - F64SqrtTest(F64MaxTest(0.0, 1.0 - ny*ny)));
            stats->worst_ny = F64MaxTest(stats->worst_ny, F64AbsTest(ny));
            if (F64AbsTest(ny) > 0.1)
            {
                /* audit F1: a noise normal (cores overlapping); two point manifolds have |n.y| <= the tilt */
                stats->two_point_steep += 1;
                stats->steep_dist = F64MaxTest(stats->steep_dist, dist / L);
                fail = "two point: steep n (noise normal)";
            }
        }

        for (u32 i = 0; !fail && i < m->v_count; ++i)
        {
            const f64 surface = F64AbsTest(CtPointSegmentDistance(a->p0, a->p1, m->v[i]) - r0);
            if (m->v_count == 1)
            {
                stats->worst_surface = F64MaxTest(stats->worst_surface, surface / eps_l);
            }
            else if (r0 > 0.0)
            {
                const f64 below = r0 - CtPointSegmentDistance(a->p0, a->p1, m->v[i]);
                stats->worst_inset = F64MaxTest(stats->worst_inset, below / r0);
            }
            if (surface > tol + inset)
            {
                fail = "v not on capsule 0's surface";
            }
            else if (m->depth[i] > -sep + tol)
            {
                fail = "depth deeper than the separation along n";
            }
        }

        if (!fail && m->v_count == 1)
        {
            const f64 err = F64AbsTest(m->depth[0] - (r_sum - dist));
            stats->worst_depth = F64MaxTest(stats->worst_depth, err / eps_l);
            if (err > tol)
            {
                fail = "depth != r_sum - dist*";
            }
        }

        if (!fail && dist > 1e-3*L)
        {
            const f64 err = -sep - (r_sum - dist);
            stats->worst_normal = F64MaxTest(stats->worst_normal, err / eps_l);
            if (err > tol*(1.0 + L/dist))
            {
                fail = "n not optimal";
            }
        }
    }

    if (fail)
    {
        fprintf(stderr, "CtVerify failed (%s): dist* %.9g r_sum %.9g L %.9g tol %.3g\n", fail, dist, r_sum, L, tol);
        CtCapsulePrint("capsule 0", a);
        CtCapsulePrint("capsule 1", b);
        if (m->contact)
        {
            c_ManifoldDebugPrint(&m->local);
        }
        return 0;
    }

    stats->contacts += m->contact;
    stats->two_point += (m->v_count == 2);
    return 1;
}

/* contact of (a, b), checked against the reference, swapped and translated */
static u32 CtCheckPair(struct arena *frame, struct ct_Stats *stats, const struct ct_Capsule *a, const struct ct_Capsule *b)
{
    const f64 dist = CtSegmentDistance(a->p0, a->p1, b->p0, b->p1);
    f64 L = D3Length(D3Sub(b->pos, a->pos));
    L = F64MaxTest(L, F64MaxTest(a->shape.capsule.half_height, b->shape.capsule.half_height));
    L = F64MaxTest(L, F64MaxTest(a->shape.capsule.radius, b->shape.capsule.radius));
    const f64 tol = CONTACT_TEST_TOLERANCE * F32_EPSILON * L;
    const f64 r_sum = (f64) a->shape.capsule.radius + (f64) b->shape.capsule.radius;

    stats->tests += 1;
    struct ct_Manifold m_ab, m_ba;
    if (!CtContact(frame, &m_ab, a, b) || !CtVerify(stats, &m_ab, a, b, dist, L))
    {
        return 0;
    }

    if (!CtContact(frame, &m_ba, b, a) || !CtVerify(stats, &m_ba, b, a, dist, L))
    {
        return 0;
    }

    if (F64AbsTest(dist - r_sum) > tol)
    {
        const f64 err = F64AbsTest(CtDeepest(&m_ab) - CtDeepest(&m_ba));
        if (m_ab.v_count == 1 && m_ba.v_count == 1)
        {
            stats->worst_swap = F64MaxTest(stats->worst_swap, err / (F32_EPSILON * L));
        }
        /* two point manifolds clip different stretches (core 1 to core 0's heights and vice versa) */
        const u32 one_point = (m_ab.v_count == 1 && m_ba.v_count == 1);
        if (m_ab.contact != m_ba.contact || (one_point && err > 2.0*tol))
        {
            fprintf(stderr, "CtCheckPair: swapped contact differs (%u vs %u, deepest %.9g vs %.9g)\n",
                m_ab.contact, m_ba.contact, CtDeepest(&m_ab), CtDeepest(&m_ba));
            CtCapsulePrint("capsule 0", a);
            CtCapsulePrint("capsule 1", b);
            return 0;
        }

        if (m_ab.contact && dist > 1e-3*L && D3Dot(m_ab.n, m_ba.n) > -1.0 + 1e-3)
        {
            fprintf(stderr, "CtCheckPair: swapped normals not opposite (n_ab . n_ba = %.9g)\n", D3Dot(m_ab.n, m_ba.n));
            CtCapsulePrint("capsule 0", a);
            CtCapsulePrint("capsule 1", b);
            return 0;
        }
    }

    /* an exactly representable offset leaves x1 - x0 exact in f64: identical bits */
    struct ct_Capsule a_t = *a, b_t = *b;
    const struct d3 offset = D3(4096.0, -1024.0, 65536.0);
    a_t.t.position = V3F64(a->pos.x + offset.x, a->pos.y + offset.y, a->pos.z + offset.z);
    b_t.t.position = V3F64(b->pos.x + offset.x, b->pos.y + offset.y, b->pos.z + offset.z);
    if (V3F64Sub(b_t.t.position, a_t.t.position).x == V3F64Sub(b->t.position, a->t.position).x
     && V3F64Sub(b_t.t.position, a_t.t.position).y == V3F64Sub(b->t.position, a->t.position).y
     && V3F64Sub(b_t.t.position, a_t.t.position).z == V3F64Sub(b->t.position, a->t.position).z)
    {
        struct ct_Manifold m_t;
        CtContact(frame, &m_t, &a_t, &b_t);
        /* compare the used fields only: the manifold's unused slots are uninitialized arena memory */
        u32 same = (m_t.contact == m_ab.contact);
        if (same && m_t.contact)
        {
            same = m_t.local.v_count == m_ab.local.v_count && memcmp(&m_t.local.n, &m_ab.local.n, sizeof(v3)) == 0;
            for (u32 i = 0; same && i < m_t.local.v_count; ++i)
            {
                same = memcmp(&m_t.local.v[i], &m_ab.local.v[i], sizeof(v3)) == 0
                    && memcmp(&m_t.local.depth[i], &m_ab.local.depth[i], sizeof(f32)) == 0;
            }
        }
        if (!same)
        {
            fprintf(stderr, "CtCheckPair: translated result differs\n");
            CtCapsulePrint("capsule 0", a);
            CtCapsulePrint("capsule 1", b);
            return 0;
        }
    }

    return 1;
}

/********************************** generators **********************************/

static v3 CtUnit(void)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 r = F32Sqrt(1.0f - z*z);
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    return V3(r*F32Cos(theta), r*F32Sin(theta), z);
}

/* unit vector perpendicular to y */
static v3 CtUnitXZ(void)
{
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    return V3(F32Cos(theta), 0.0f, F32Sin(theta));
}

static q CtRotation(void)
{
    return QUnitAxisAngle(CtUnit(), RngF32Range(0.0f, 2.0f*F32_PI));
}

/* sizes 1e-2 .. 3e2 */
static f32 CtSize(void)
{
    const f32 decade[] = { 1e-2f, 1e-1f, 1.0f, 1e1f, 1e2f };
    return decade[RngU64Range(0, 4)] * RngF32Range(1.0f, 3.0f);
}

/* world positions 0 .. 1e6 */
static struct d3 CtWorldPosition(void)
{
    const f64 range[] = { 0.0, 1.0, 1e3, 1e6 };
    return D3Scale(D3V3(CtUnit()), range[RngU64Range(0, 3)] * RngF32Normalized());
}

/* a point given in a's frame, in world space */
static struct d3 CtLocalToWorld(const struct ct_Capsule *a, const v3 p)
{
    return D3Add(a->pos, D3RotationMul(a->r, D3V3(p)));
}

/*
 * Capsule 1 relative to capsule 0 (given), by mode:
 *  0: random pose near capsule 0
 *  1: nearly parallel (tilt 0 .. 0.1 rad, around the 0.05 threshold), side by side at ~ r_sum
 *  2: cores crossing or nearly crossing
 *  3: beyond core 0's end (cap region)
 *  4: coaxial, shifted along the axis
 *  5: random, but one capsule has half height 0
 */
static void CtCapsuleNear(struct ct_Capsule *b, struct ct_Capsule *a, const u32 mode)
{
    const f32 h0 = a->shape.capsule.half_height;
    const f32 r0 = a->shape.capsule.radius;
    f32 h1 = CtSize();
    const f32 r1 = CtSize();
    const f32 r_sum = r0 + r1;
    const f32 reach = h0 + h1 + r_sum;

    switch (mode)
    {
        case 0:
        {
            const v3 c = V3Scale(CtUnit(), reach * RngF32Range(0.0f, 1.2f));
            CtCapsuleInit(b, h1, r1, CtRotation(), CtLocalToWorld(a, c));
        } break;

        case 1:
        {
            const f32 tilt[] = { 0.0f, 1e-6f, 1e-3f, 0.02f, 0.0499f, 0.05f, 0.0501f, 0.1f };
            const v3 side = CtUnitXZ();
            const q rot = QMul(a->t.rotation, QUnitAxisAngle(CtUnitXZ(), tilt[RngU64Range(0, 7)]));
            const v3 c = V3Add(V3Scale(side, r_sum * RngF32Range(0.5f, 1.2f)),
                               V3(0.0f, (h0 + h1) * RngF32Range(-1.2f, 1.2f), 0.0f));
            CtCapsuleInit(b, h1, r1, (RngU64Range(0, 1)) ? rot : QMul(rot, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI)), CtLocalToWorld(a, c));
        } break;

        case 2:
        {
            const f32 offset[] = { 0.0f, 1e-7f, 1e-4f, 1e-2f };
            const v3 c = V3Add(V3(0.0f, h0 * RngF32Range(-1.0f, 1.0f), 0.0f),
                               V3Scale(CtUnitXZ(), reach * offset[RngU64Range(0, 3)]));
            CtCapsuleInit(b, h1, r1, CtRotation(), CtLocalToWorld(a, c));
        } break;

        case 3:
        {
            const f32 sign = (RngU64Range(0, 1)) ? 1.0f : -1.0f;
            const v3 c = V3Add(V3(0.0f, sign * (h0 + RngF32Range(0.0f, h1 + r_sum)), 0.0f),
                               V3Scale(CtUnitXZ(), r_sum * RngF32Range(0.0f, 1.5f)));
            CtCapsuleInit(b, h1, r1, CtRotation(), CtLocalToWorld(a, c));
        } break;

        case 4:
        {
            const v3 c = V3(0.0f, reach * RngF32Range(-1.2f, 1.2f), 0.0f);
            CtCapsuleInit(b, h1, r1, (RngU64Range(0, 1)) ? a->t.rotation : QMul(a->t.rotation, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI)), CtLocalToWorld(a, c));
        } break;

        default:
        {
            if (RngU64Range(0, 1))
            {
                h1 = 0.0f;
            }
            else
            {
                CtCapsuleInit(a, 0.0f, r0, a->t.rotation, a->pos);
            }
            const v3 c = V3Scale(CtUnit(), reach * RngF32Range(0.0f, 1.2f));
            CtCapsuleInit(b, h1, r1, CtRotation(), CtLocalToWorld(a, c));
        } break;
    }
}

/********************************** tests **********************************/

static u32 CtNear(const f64 a, const f64 b)
{
    return F64AbsTest(a - b) <= 1e-5;
}

static u32 CtNearD3(const struct d3 a, const struct d3 b)
{
    return CtNear(a.x, b.x) && CtNear(a.y, b.y) && CtNear(a.z, b.z);
}

/* hand-made cases with known answers; capsule 0 at the origin, unrotated, h0 = 1 */
struct test_Output CapsuleContactCasesTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const struct d3 o = D3(0.0, 0.0, 0.0);
    const q x_axis = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), F32_PI/2.0f);   /* y -> -x */
    struct ct_Capsule a, b;
    struct ct_Manifold m;
    CtCapsuleInit(&a, 1.0f, 0.5f, QIdentity(), o);

    /* separated: core 1 along x at z = 1.5 > r_sum = 1 */
    CtCapsuleInit(&b, 1.0f, 0.5f, x_axis, D3(0.0, 0.0, 1.5));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.contact, 0);

    /* touching exactly: dist == r_sum is a contact of depth 0 */
    CtCapsuleInit(&b, 1.0f, 0.5f, x_axis, D3(0.0, 0.0, 1.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.contact, 1);
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(m.depth[0] == 0.0);

    /* crossing at z = 0.75: n = +z, depth 0.25, v = (0, 0, r0) */
    CtCapsuleInit(&b, 1.0f, 0.5f, x_axis, D3(0.0, 0.25, 0.75));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(m.n, D3(0.0, 0.0, 1.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.25));
    TEST_TRUE(CtNearD3(m.v[0], D3(0.0, 0.25, 0.5)));

    /* cap: c = (0, 3, 0), e = (1, 1, 0) (45 deg, h1 = sqrt 2), r_sum = 1.5: closest pair (0, 1, 0), (-1, 2, 0) */
    CtCapsuleInit(&a, 1.0f, 0.75f, QIdentity(), o);
    CtCapsuleInit(&b, 1.41421356f, 0.75f, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -F32_PI/4.0f), D3(0.0, 3.0, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNear(m.depth[0], 1.5 - 1.41421356));
    TEST_TRUE(CtNearD3(m.n, D3(-0.70710678, 0.70710678, 0.0)));
    TEST_TRUE(CtNearD3(m.v[0], D3(-0.75*0.70710678, 1.0 + 0.75*0.70710678, 0.0)));

    /* cores cross exactly (d = 0): n = +-z (perpendicular to both), towards capsule 1's center; depth r_sum */
    CtCapsuleInit(&a, 1.0f, 0.5f, QIdentity(), o);
    CtCapsuleInit(&b, 1.0f, 0.5f, x_axis, D3(0.0, 0.5, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNear(F64AbsTest(m.n.z), 1.0));
    TEST_TRUE(CtNear(m.depth[0], 1.0));

    /* side by side, parallel: c = (0.8, 0.3, 0): stretch y in [-0.7, 1], two points at x = r0, depth 0.2 */
    CtCapsuleInit(&b, 1.0f, 0.5f, QIdentity(), D3(0.8, 0.3, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNearD3(m.n, D3(1.0, 0.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.2) && CtNear(m.depth[1], 0.2));
    TEST_TRUE(CtNear(F64MinTest(m.v[0].y, m.v[1].y), -0.7) && CtNear(F64MaxTest(m.v[0].y, m.v[1].y), 1.0));
    TEST_TRUE(CtNear(m.v[0].x, 0.5) && CtNear(m.v[1].x, 0.5));

    /* same, antiparallel (capsule 1 flipped): same contact */
    CtCapsuleInit(&b, 1.0f, 0.5f, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI), D3(0.8, 0.3, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNear(m.depth[0], 0.2) && CtNear(m.depth[1], 0.2));

    /* tilted 0.02 rad (< 2.86 deg) about z, both ends touch: two points, depths differ */
    CtCapsuleInit(&b, 1.0f, 0.5f, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.02f), D3(0.8, 0.0, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(m.depth[0] != m.depth[1]);

    /* tilted 0.02 rad at x = 0.99: core 1's ends at x = 0.97 (touches) and 1.01 (lifted): one point */
    CtCapsuleInit(&b, 1.0f, 0.5f, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.02f), D3(0.99, 0.0, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);

    /* tilted 0.06 rad (> 2.86 deg), both ends would touch: one point */
    CtCapsuleInit(&b, 1.0f, 0.5f, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.06f), D3(0.8, 0.0, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);

    /* coaxial, end to end: c = (0, 2.5, 0): closest (0, 1, 0), (0, 1.5, 0), n = +y, depth 0.5, one point */
    CtCapsuleInit(&b, 1.0f, 0.5f, QIdentity(), D3(0.0, 2.5, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(m.n, D3(0.0, 1.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.5));
    TEST_TRUE(CtNearD3(m.v[0], D3(0.0, 1.5, 0.0)));

    /* coaxial, overlapping: c = (0, 1.5, 0), d = 0: n = (1, 0, 0), two points of depth r_sum over y in [0.5, 1] */
    CtCapsuleInit(&b, 1.0f, 0.5f, QIdentity(), D3(0.0, 1.5, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNearD3(m.n, D3(1.0, 0.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 1.0) && CtNear(m.depth[1], 1.0));

    /* parallel, offset and end to end in y (c = (0.8, 2.2, 0)): no shared stretch, one point */
    CtCapsuleInit(&b, 1.0f, 0.5f, QIdentity(), D3(0.8, 2.2, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);

    /* capsule 1 of half height 0 (a sphere): side contact */
    CtCapsuleInit(&b, 0.0f, 0.5f, QIdentity(), D3(0.75, 0.5, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(m.n, D3(1.0, 0.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.25));

    /* capsule 1 of half height 0 exactly on core 0 */
    CtCapsuleInit(&b, 0.0f, 0.5f, QIdentity(), D3(0.0, 0.5, 0.0));
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNear(m.depth[0], 1.0));

    /* both half height 0, coincident centers */
    CtCapsuleInit(&a, 0.0f, 0.5f, QIdentity(), o);
    CtCapsuleInit(&b, 0.0f, 0.5f, QIdentity(), o);
    TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNear(m.depth[0], 1.0));

    /* rotated frame and far world position: the side by side case, rotated and moved */
    {
        const q rot = QUnitAxisAngle(V3(0.48f, 0.6f, 0.64f), 1.3f);
        const struct d3 pos = D3(1e6, -3e5, 2.5e5);
        CtCapsuleInit(&a, 1.0f, 0.5f, rot, pos);
        CtCapsuleInit(&b, 1.0f, 0.5f, rot, CtLocalToWorld(&a, V3(0.8f, 0.3f, 0.0f)));
        TEST_TRUE(CtContact(env->mem_2, &m, &a, &b));
        TEST_EQUAL(m.v_count, 2);
        TEST_TRUE(F64AbsTest(m.depth[0] - 0.2) < 1e-3 && F64AbsTest(m.depth[1] - 0.2) < 1e-3);
    }

	return output;
}

/* random pairs of every mode, checked against the double reference */
struct test_Output CapsuleContactRandomTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const char *mode_id[] = { "random", "parallel", "crossing", "cap", "coaxial", "zero length" };
    for (u32 mode = 0; mode < 6; ++mode)
    {
        struct ct_Stats stats = { 0 };
        for (u32 i = 0; i < 50000; ++i)
        {
            struct arena record = *env->mem_2;
            struct ct_Capsule a, b;
            CtCapsuleInit(&a, CtSize(), CtSize(), CtRotation(), CtWorldPosition());
            CtCapsuleNear(&b, &a, mode);
            const u32 ok = CtCheckPair(env->mem_2, &stats, &a, &b);
            *env->mem_2 = record;
            if (!ok)
            {
                CtStatsPrint(mode_id[mode], &stats);
                TEST_FAILURE;
            }
        }
        CtStatsPrint(mode_id[mode], &stats);
    }

	return output;
}

/*
 * Tilt sweep across the parallel threshold (capsule_parallel_max_degrees +- 0.002 rad): the normal must stay
 * continuous; the deepest depth jumps by at most the two point manifold's loss (closest pair inside the
 * shared stretch). Prints the largest jumps relative to the capsule size.
 */
struct test_Output CapsuleContactThresholdTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();
    const f32 threshold = g_numerics_config->capsule_parallel_max_degrees * F32_PI2 / 360;

    f64 worst_n = 0.0;
    f64 worst_depth = 0.0;
    for (u32 i = 0; i < 2000; ++i)
    {
        /* h <= 10 r: the tilt (<= 0.052 h) can't bring the cores together (offset >= 1.2 r) */
        const f32 r = CtSize();
        const f32 h = r * RngF32Range(1.0f, 10.0f);
        const q rot0 = CtRotation();
        const struct d3 pos = CtWorldPosition();
        const v3 tilt_axis = CtUnitXZ();
        const v3 side = CtUnitXZ();
        const f32 offset = 2.0f*r * RngF32Range(0.6f, 1.0f);
        const f32 shift = h * RngF32Range(-0.5f, 0.5f);

        struct ct_Manifold prev = { 0 };
        for (u32 k = 0; k <= 40; ++k)
        {
            struct arena record = *env->mem_2;
            const f32 tilt = threshold - 0.002f + 0.0001f*(f32) k;
            struct ct_Capsule a, b;
            CtCapsuleInit(&a, h, r, rot0, pos);
            const v3 c = V3Add(V3Scale(side, offset), V3(0.0f, shift, 0.0f));
            CtCapsuleInit(&b, h, r, QMul(rot0, QUnitAxisAngle(tilt_axis, tilt)), CtLocalToWorld(&a, c));

            struct ct_Manifold m;
            const u32 ok = CtContact(env->mem_2, &m, &a, &b);
            *env->mem_2 = record;
            if (!ok)
            {
                TEST_FAILURE;
            }

            if (k > 0 && m.contact && prev.contact)
            {
                worst_n = F64MaxTest(worst_n, D3Length(D3Sub(m.n, prev.n)));
                worst_depth = F64MaxTest(worst_depth, F64AbsTest(CtDeepest(&m) - CtDeepest(&prev)) / (f64) h);
            }
            prev = m;
        }
    }

    fprintf(stdout, "\t%s: largest step between tilts 1e-4 rad apart: |dn| %.3g, |d deepest| / h %.3g\n", __func__, worst_n, worst_depth);
    if (worst_n > 1e-2)
    {
        TEST_FAILURE;
    }

	return output;
}

/********************************** hull vs capsule **********************************/

/*
Hull vs capsule (TMP_c_HullCapsuleContact), checked in double precision in the hull's frame (the manifold's
frame). The hull's f32 vertices are exact; face normals are recomputed in double (Newell). With the core a, b
in the hull's frame and sep(n) = min(n.a, n.b) - max_hull n.x the core's separation along unit n:

    sat = max sep(n) over the face normals and cross(edge, b - a) (both signs)
        = the signed distance if the core intersects the hull (exact: these are the face normals of the
          Minkowski difference), else a lower bound of the distance
    boxes: dist* = exact core-box distance (convex in the core parameter: ternary search); sd = dist* > 0 ?
          dist* : sat

Every manifold must satisfy, with L = max(hull extent, h, r, |x1 - x0|), tol = CONTACT_TEST_TOLERANCE eps L:

    contact iff sd <= r (boxes; undecided within tol); other hulls: sat > r + tol -> none, sat < -tol -> contact
    c_ManifoldCheck, v_count in {1, 2}, |n| = 1
    depth_i <= r - sep(n) + tol                 (n separates as deep as it claims)
    v_i on the hull's surface                   |max_f (n_f.v - d_f)| <= tol
    v_i + (r - depth_i) n on the core           (point and depth agree; edge contacts within tol / sin of the
                                                edge parallel angle: nearly parallel lines' closest points)
    shallow one point (boxes): depth = r - dist*, n optimal: -sep(n) <= r - ... as in the capsule tests
    deep (sat < -tol): measured only (sat - sep(n)): a face contact competes with the edges through its
                                                clipped separation (Box3D), not the core's separation along n
    translating both shapes by an exactly representable offset gives identical bits
*/

#define CH_VERTEX_MAX   32
#define CH_FACE_MAX     64
#define CH_EDGE_MAX     96

struct ch_Hull
{
    struct c_Shape      shape;
    ds_TransformWorld   t;
    struct d3           r[3];               /* rotation (rows), double */
    struct d3           pos;                /* world position, double */
    u32                 is_box;
    struct d3           hw;                 /* boxes: half widths */
    f64                 extent;             /* max |v| */
    u32                 v_count;
    struct d3           v[CH_VERTEX_MAX];
    u32                 f_count;
    struct d3           fn[CH_FACE_MAX];    /* unit outward face normals (double Newell) */
    f64                 fd[CH_FACE_MAX];    /* max over the face's vertices of fn.v */
    u32                 e_count;
    struct d3           ed[CH_EDGE_MAX];    /* edge directions, each edge once */
};

/* R^T v */
static struct d3 D3RotationTMul(const struct d3 r[3], const struct d3 v)
{
    return D3Add(D3Add(D3Scale(r[0], v.x), D3Scale(r[1], v.y)), D3Scale(r[2], v.z));
}

static struct d3 D3Cross(const struct d3 a, const struct d3 b)
{
    return D3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

static void ChHullInit(struct ch_Hull *h, const struct c_Shape *shape, const u32 is_box, const v3 hw, const q rotation, const struct d3 pos)
{
    const struct dcel *dcel = &shape->hull;
    ds_Assert(dcel->v_count <= CH_VERTEX_MAX && dcel->f_count <= CH_FACE_MAX && dcel->e_count <= 2*CH_EDGE_MAX);

    h->shape = *shape;
    /* center of mass (the hull contact orients edge axes by it), as the engine sets it */
    c_ShapeUpdateMassProperties(&h->shape);
    h->t.rotation = rotation;
    h->t.position = V3F64(pos.x, pos.y, pos.z);
    h->pos = pos;
    D3Rotation(h->r, rotation);
    h->is_box = is_box;
    h->hw = D3V3(hw);

    h->extent = 0.0;
    h->v_count = dcel->v_count;
    for (u32 i = 0; i < dcel->v_count; ++i)
    {
        h->v[i] = D3V3(dcel->v[i]);
        h->extent = F64MaxTest(h->extent, D3Length(h->v[i]));
    }

    h->f_count = dcel->f_count;
    for (u32 fi = 0; fi < dcel->f_count; ++fi)
    {
        const struct dcelFace *f = dcel->f + fi;
        struct d3 n = D3(0.0, 0.0, 0.0);
        for (u32 i = 0; i < f->count; ++i)
        {
            const struct d3 a = h->v[dcel->e[f->first + i].origin];
            const struct d3 b = h->v[dcel->e[f->first + (i + 1) % f->count].origin];
            n = D3Add(n, D3((a.y - b.y)*(a.z + b.z), (a.z - b.z)*(a.x + b.x), (a.x - b.x)*(a.y + b.y)));
        }
        if (D3Dot(n, D3V3(dcel->plane[fi].normal)) < 0.0)
        {
            n = D3Scale(n, -1.0);
        }
        h->fn[fi] = D3Scale(n, 1.0 / D3Length(n));
        h->fd[fi] = -1e300;
        for (u32 i = 0; i < f->count; ++i)
        {
            h->fd[fi] = F64MaxTest(h->fd[fi], D3Dot(h->fn[fi], h->v[dcel->e[f->first + i].origin]));
        }
    }

    h->e_count = 0;
    for (u32 ei = 0; ei < dcel->e_count; ++ei)
    {
        const u32 twin = dcel->e[ei].twin;
        if (ei < twin)
        {
            h->ed[h->e_count++] = D3Sub(h->v[dcel->e[twin].origin], h->v[dcel->e[ei].origin]);
        }
    }
}

static f64 ChSupport(const struct ch_Hull *h, const struct d3 n)
{
    f64 max = -1e300;
    for (u32 i = 0; i < h->v_count; ++i)
    {
        max = F64MaxTest(max, D3Dot(n, h->v[i]));
    }
    return max;
}

/* the core's separation along unit n (hull frame) */
static f64 ChSeparation(const struct ch_Hull *h, const struct d3 a, const struct d3 b, const struct d3 n)
{
    return F64MinTest(D3Dot(n, a), D3Dot(n, b)) - ChSupport(h, n);
}

static f64 ChSat(const struct ch_Hull *h, const struct d3 a, const struct d3 b)
{
    f64 sat = -1e300;
    for (u32 fi = 0; fi < h->f_count; ++fi)
    {
        sat = F64MaxTest(sat, ChSeparation(h, a, b, h->fn[fi]));
    }

    const struct d3 e = D3Sub(b, a);
    for (u32 i = 0; i < h->e_count; ++i)
    {
        const struct d3 c = D3Cross(h->ed[i], e);
        const f64 c_sq = D3Dot(c, c);
        if (c_sq > 1e-24*D3Dot(h->ed[i], h->ed[i])*D3Dot(e, e) && c_sq > 0.0)
        {
            const struct d3 n = D3Scale(c, 1.0 / F64SqrtTest(c_sq));
            sat = F64MaxTest(sat, ChSeparation(h, a, b, n));
            sat = F64MaxTest(sat, ChSeparation(h, a, b, D3Scale(n, -1.0)));
        }
    }

    return sat;
}

static f64 ChBoxPointDistance(const struct ch_Hull *h, const struct d3 p)
{
    const struct d3 c = D3(F64ClampTest(p.x, -h->hw.x, h->hw.x), F64ClampTest(p.y, -h->hw.y, h->hw.y), F64ClampTest(p.z, -h->hw.z, h->hw.z));
    return D3Length(D3Sub(p, c));
}

/* exact core-box distance: the point-box distance along the core is convex */
static f64 ChBoxDistance(const struct ch_Hull *h, const struct d3 a, const struct d3 b)
{
    const struct d3 e = D3Sub(b, a);
    f64 lo = 0.0, hi = 1.0;
    for (u32 i = 0; i < 200; ++i)
    {
        const f64 t1 = lo + (hi - lo) / 3.0;
        const f64 t2 = hi - (hi - lo) / 3.0;
        if (ChBoxPointDistance(h, D3Add(a, D3Scale(e, t1))) <= ChBoxPointDistance(h, D3Add(a, D3Scale(e, t2))))
        {
            hi = t2;
        }
        else
        {
            lo = t1;
        }
    }
    f64 dist = ChBoxPointDistance(h, D3Add(a, D3Scale(e, 0.5*(lo + hi))));
    dist = F64MinTest(dist, ChBoxPointDistance(h, a));
    dist = F64MinTest(dist, ChBoxPointDistance(h, b));
    return dist;
}

/* max_f (n_f.p - d_f): 0 on the surface, < 0 inside */
static f64 ChHullSdf(const struct ch_Hull *h, const struct d3 p)
{
    f64 sdf = -1e300;
    for (u32 fi = 0; fi < h->f_count; ++fi)
    {
        sdf = F64MaxTest(sdf, D3Dot(h->fn[fi], p) - h->fd[fi]);
    }
    return sdf;
}

/* the capsule's core in the hull's frame */
static void ChCore(struct d3 *a, struct d3 *b, const struct ch_Hull *h, const struct ct_Capsule *c)
{
    *a = D3RotationTMul(h->r, D3Sub(c->p0, h->pos));
    *b = D3RotationTMul(h->r, D3Sub(c->p1, h->pos));
}

static u32 ChContact(struct arena *frame, struct c_Manifold *m, u32 *contact, const struct ch_Hull *h, const struct ct_Capsule *c)
{
    const struct c_Shape *s[2] = { &h->shape, &c->shape };
    const ds_TransformWorld t[2] = { h->t, c->t };
    const struct c_ContactResult cached = { 0 };
    const struct c_ContactResult result = TMP_c_HullCapsuleContact(frame, &cached, s, t);

    *contact = 0;
    if (result.manifold_count == 0)
    {
        return 1;
    }

    if (result.manifold_count != 1 || !c_ManifoldCheck(result.manifold)
            || result.manifold->v_count < 1 || result.manifold->v_count > 2)
    {
        fprintf(stderr, "ChContact: invalid manifold (count %u)\n", result.manifold_count);
        if (result.manifold_count)
        {
            c_ManifoldDebugPrint(result.manifold);
        }
        return 0;
    }

    *contact = 1;
    *m = *result.manifold;
    return 1;
}

struct ch_Stats
{
    u64 tests;
    u64 contacts;
    u64 two_point;
    u64 edge;               /* deep path (GJK touching), n not a face normal */
    u64 edge_clamped;       /* of those: v + (r - depth) n off the core (fails; kept for the printout) */
    u64 undecided;
    u64 unchecked;          /* other hulls: 0 <= sat <= r, contact not decided by the reference */
    u64 face_biased;        /* deep: n worse than the best axis by more than tol (face kept by the bias) */
    f64 worst_depth;        /* box shallow one point: |depth - (r - dist*)| / (eps L) */
    f64 worst_surface;      /* |sdf(v)| / (eps L) */
    f64 worst_core;         /* dist(v + (r - depth) n, core) / (eps L), edge contacts excluded */
    f64 worst_normal;       /* box shallow one point: (-sep(n) - (-dist*)) / (eps L) */
    f64 worst_gap;          /* deep: (sat - sep(n)) / L */
    f64 worst_gap_two;      /* box shallow two point: (dist* - sep(n)) / L (face normal vs the true normal) */
    u64 gjk_normal_outliers;/* box shallow one point: n beyond GJK's documented accuracy */
    f64 worst_gjk_outlier;  /* of those: (-sep(n) - (-dist*)) / L */
    u64 gjk_point_outliers; /* shallow one point: v + (r - depth) n off the core by more than tol (GJK) */
    f64 worst_clamped;      /* edge contacts with clamped closest points: dist(v + (r - depth) n, core) / L */
    f64 worst_edge_core;    /* edge contacts: dist(v + (r - depth) n, core) / (eps L) */
};

static void ChStatsPrint(const char *id, const struct ch_Stats *s)
{
    fprintf(stdout, "\t%s: %llu tests, %llu contacts (%llu two point, %llu edge, %llu clamped), %llu undecided, %llu unchecked, %llu face biased; worst / (eps L): depth %.1f, surface %.1f, core %.1f, normal %.1f; worst / L: deep gap %.3g, shallow two point gap %.3g; GJK normal outliers %llu (worst / L %.3g), point outliers %llu; clamped edge points off the core / L %.3g; edge core / (eps L) %.1f\n",
        id, (unsigned long long) s->tests, (unsigned long long) s->contacts, (unsigned long long) s->two_point,
        (unsigned long long) s->edge, (unsigned long long) s->edge_clamped, (unsigned long long) s->undecided,
        (unsigned long long) s->unchecked, (unsigned long long) s->face_biased, s->worst_depth, s->worst_surface,
        s->worst_core, s->worst_normal, s->worst_gap, s->worst_gap_two, (unsigned long long) s->gjk_normal_outliers, s->worst_gjk_outlier, (unsigned long long) s->gjk_point_outliers, s->worst_clamped, s->worst_edge_core);
}

static void ChPrint(const struct ch_Hull *h, const struct ct_Capsule *c)
{
    if (h->is_box)
    {
        fprintf(stderr, "\tbox: hw { %.9g, %.9g, %.9g }", h->hw.x, h->hw.y, h->hw.z);
    }
    else
    {
        fprintf(stderr, "\thull: %u vertices, extent %.9g", h->v_count, h->extent);
    }
    fprintf(stderr, " rot { %.9g, %.9g, %.9g, %.9g } pos { %.17g, %.17g, %.17g }\n", h->t.rotation.x, h->t.rotation.y,
        h->t.rotation.z, h->t.rotation.w, h->pos.x, h->pos.y, h->pos.z);
    CtCapsulePrint("capsule", c);
}

static u32 ChVerify(struct ch_Stats *stats, const u32 contact, const struct c_Manifold *m, const struct ch_Hull *h, const struct ct_Capsule *c)
{
    struct d3 a, b;
    ChCore(&a, &b, h, c);
    const f64 r = c->shape.capsule.radius;
    const f64 L = F64MaxTest(F64MaxTest(h->extent, D3Length(D3Sub(c->pos, h->pos))), F64MaxTest(c->shape.capsule.half_height, r));
    const f64 eps_l = F32_EPSILON * L;
    const f64 tol = CONTACT_TEST_TOLERANCE * eps_l;
    const f64 sat = ChSat(h, a, b);
    const f64 dist = (h->is_box) ? ChBoxDistance(h, a, b) : -1.0;
    const char *fail = NULL;

    stats->tests += 1;
    if (h->is_box)
    {
        const f64 sd = (dist > 0.0) ? dist : sat;
        if (F64AbsTest(sd - r) <= tol)
        {
            stats->undecided += 1;
        }
        else if (contact != (sd < r))
        {
            fail = "contact";
        }
    }
    else if (sat > r + tol)
    {
        fail = (contact) ? "contact, but the reference separates" : NULL;
    }
    else if (sat < -tol)
    {
        fail = (!contact) ? "no contact, but the core intersects the hull" : NULL;
    }
    else
    {
        stats->unchecked += 1;
    }

    if (!fail && contact)
    {
        const struct d3 n = D3V3(m->n);
        const f64 n_len = D3Length(n);
        const f64 sep = ChSeparation(h, a, b, D3Scale(n, 1.0 / n_len));
        const u32 deep = (sat < -tol);
        /* the contact's path: its own GJK query (touching up to the touch band -> the deep path) */
        const ds_Transform b_to_a = ds_TransformWorldRelative(&h->t, &c->t);
        struct GJKCache gjk_cache;
        v3 gjk_c[2], gjk_n;
        const u32 shallow = (GJK(gjk_c + 0, gjk_c + 1, &gjk_n, &gjk_cache, NULL, &h->shape, &c->shape, &b_to_a, (f32) r) > 0.0f);
        /* a face contact's n is the stored plane normal, bit for bit */
        u32 face_normal = 0;
        for (u32 fi = 0; fi < h->shape.hull.f_count; ++fi)
        {
            face_normal |= (memcmp(&m->n, &h->shape.hull.plane[fi].normal, sizeof(v3)) == 0);
        }
        const u32 edge = !shallow && !face_normal;

        if (F64AbsTest(n_len - 1.0) > 1e-5)
        {
            fail = "|n| != 1";
        }

        u32 clamped = 0;
        for (u32 i = 0; !fail && i < m->v_count; ++i)
        {
            const struct d3 v = D3V3(m->v[i]);
            const f64 surface = F64AbsTest(ChHullSdf(h, v));
            stats->worst_surface = F64MaxTest(stats->worst_surface, surface / eps_l);
            const struct d3 q = D3Add(v, D3Scale(n, r - (f64) m->depth[i]));
            const f64 core = CtPointSegmentDistance(a, b, q);
            /*
             * Edge contacts: the lines' closest points are ill-conditioned for nearly parallel lines (angle th):
             * rounding slides them by ~ eps L / sin^2 th along the lines, moving q ~ eps L / sin th off the core.
             * th >= the edge parallel angle.
             */
            const f64 edge_tol = tol / F64SqrtTest(g_numerics_config->edge_parallel_sin_sq);
            /* touching without intersecting (GJK's touch band, deep path): positions known to the band */
            const f64 core_tol = (!shallow && sat > -tol) ? F64MaxTest(tol, 2.0*GJK_TOUCH_TOLERANCE*eps_l) : tol;
            if (edge && core > edge_tol)
            {
                clamped = 1;
                stats->worst_clamped = F64MaxTest(stats->worst_clamped, core / L);
            }
            if (edge)
            {
                stats->worst_edge_core = F64MaxTest(stats->worst_edge_core, core / eps_l);
            }
            else
            {
                stats->worst_core = F64MaxTest(stats->worst_core, core / eps_l);
            }

            if (m->depth[i] > r - sep + tol)
            {
                fail = "depth deeper than the separation along n";
            }
            else if (surface > tol)
            {
                fail = "v not on the hull's surface";
            }
            else if (edge && core > edge_tol)
            {
                fail = "edge contact: v + (r - depth) n not on the core (closest points outside the segments)";
            }
            else if (!edge && core > core_tol)
            {
                if (m->v_count == 1 && shallow)
                {
                    /* shallow one point (GJK's closest points): GJK beyond its documented accuracy */
                    stats->gjk_point_outliers += 1;
                    fprintf(stdout, "\tGJK point outlier: dist(v + (r - depth) n, core) = %.3g (tol %.3g), dist* %.9g\n", core, tol, dist);
                    ChPrint(h, c);
                }
                else
                {
                    fail = "v + (r - depth) n not on the core";
                }
            }
        }

        /*
         * Deep: not checked for optimality. A face contact competes with the edges through its clipped
         * separation (as Box3D), which can be far above the core's separation along the face normal (the clip
         * drops the deep part), so n can be far from the best axis. Measured instead.
         */
        if (!fail && deep)
        {
            const f64 gap = sat - sep;
            stats->worst_gap = F64MaxTest(stats->worst_gap, gap / L);
            stats->face_biased += (gap > tol);
        }

        if (!fail && h->is_box && dist > 1e-3*L)
        {
            if (m->v_count == 1)
            {
                const f64 err = F64AbsTest(m->depth[0] - (r - dist));
                stats->worst_depth = F64MaxTest(stats->worst_depth, err / eps_l);
                const f64 n_err = -sep - (-dist);
                stats->worst_normal = F64MaxTest(stats->worst_normal, n_err / eps_l);
                if (err > tol)
                {
                    fail = "shallow: depth != r - dist*";
                }
                else if (n_err > tol*(1.0 + L/dist))
                {
                    /* GJK's normal beyond its documented accuracy (see the GJK suite's early stop failures) */
                    stats->gjk_normal_outliers += 1;
                    if (n_err / L > stats->worst_gjk_outlier)
                    {
                        stats->worst_gjk_outlier = n_err / L;
                        fprintf(stdout, "\tGJK normal outlier: -sep(n) - (-dist*) = %.3g (bound %.3g), dist* %.9g\n", n_err, tol*(1.0 + L/dist), dist);
                        ChPrint(h, c);
                    }
                }
            }
            else
            {
                stats->worst_gap_two = F64MaxTest(stats->worst_gap_two, (dist - sep) / L);
            }
        }

        if (!fail)
        {
            stats->contacts += 1;
            stats->two_point += (m->v_count == 2);
            stats->edge += edge;
            stats->edge_clamped += clamped;
        }
    }

    if (fail)
    {
        fprintf(stderr, "ChVerify failed (%s): sat %.9g dist* %.9g r %.9g L %.9g tol %.3g\n", fail, sat, dist, r, L, tol);
        ChPrint(h, c);
        if (contact)
        {
            c_ManifoldDebugPrint(m);
        }
        return 0;
    }

    return 1;
}

/* contact of (h, c), checked against the reference and translated */
static u32 ChCheckPair(struct arena *frame, struct ch_Stats *stats, const struct ch_Hull *h, const struct ct_Capsule *c)
{
    struct c_Manifold m;
    u32 contact;
    if (!ChContact(frame, &m, &contact, h, c) || !ChVerify(stats, contact, &m, h, c))
    {
        return 0;
    }

    /* an exactly representable offset leaves x1 - x0 exact in f64: identical bits */
    struct ch_Hull h_t = *h;
    struct ct_Capsule c_t = *c;
    const struct d3 offset = D3(4096.0, -1024.0, 65536.0);
    h_t.t.position = V3F64(h->pos.x + offset.x, h->pos.y + offset.y, h->pos.z + offset.z);
    c_t.t.position = V3F64(c->pos.x + offset.x, c->pos.y + offset.y, c->pos.z + offset.z);
    if (V3F64Sub(c_t.t.position, h_t.t.position).x == V3F64Sub(c->t.position, h->t.position).x
     && V3F64Sub(c_t.t.position, h_t.t.position).y == V3F64Sub(c->t.position, h->t.position).y
     && V3F64Sub(c_t.t.position, h_t.t.position).z == V3F64Sub(c->t.position, h->t.position).z)
    {
        struct c_Manifold m_t;
        u32 contact_t;
        ChContact(frame, &m_t, &contact_t, &h_t, &c_t);
        u32 same = (contact_t == contact);
        if (same && contact)
        {
            same = m_t.v_count == m.v_count && memcmp(&m_t.n, &m.n, sizeof(v3)) == 0;
            for (u32 i = 0; same && i < m.v_count; ++i)
            {
                same = memcmp(&m_t.v[i], &m.v[i], sizeof(v3)) == 0 && memcmp(&m_t.depth[i], &m.depth[i], sizeof(f32)) == 0;
            }
        }
        if (!same)
        {
            fprintf(stderr, "ChCheckPair: translated result differs\n");
            ChPrint(h, c);
            return 0;
        }
    }

    return 1;
}

/* rotation taking (0, 1, 0) to unit d */
static q ChRotationFromY(const v3 d)
{
    const v3 axis = V3Cross(V3(0.0f, 1.0f, 0.0f), d);
    const f32 s = V3Length(axis);
    if (s < 1e-6f)
    {
        return (d.y > 0.0f) ? QIdentity() : QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI);
    }
    return QUnitAxisAngle(V3Scale(axis, 1.0f / s), F32Acos(F32Clamp(d.y, -1.0f, 1.0f)));
}

/* unit d rotated by angle towards unit w (w perpendicular to d) */
static v3 ChTilt(const v3 d, const v3 w, const f32 angle)
{
    return V3Add(V3Scale(d, F32Cos(angle)), V3Scale(w, F32Sin(angle)));
}

/* a unit vector perpendicular to unit d */
static v3 ChPerpendicular(const v3 d)
{
    const v3 c = V3Cross(d, CtUnit());
    const f32 l = V3Length(c);
    return (l > 1e-3f) ? V3Scale(c, 1.0f / l) : ChPerpendicular(d);
}

static struct c_Shape ChRandomHullShape(struct arena *mem, const f32 size)
{
    v3 p[24];
    struct c_Shape shape = { .type = C_SHAPE_CONVEX_HULL };
    do
    {
        const u32 count = (u32) RngU64Range(4, 24);
        const v3 axis_scale = V3(size*RngF32Range(0.2f, 1.0f), size*RngF32Range(0.2f, 1.0f), size*RngF32Range(0.2f, 1.0f));
        for (u32 i = 0; i < count; ++i)
        {
            const v3 u = V3Scale(CtUnit(), RngF32Range(0.5f, 1.0f));
            p[i] = V3(u.x*axis_scale.x, u.y*axis_scale.y, u.z*axis_scale.z);
        }
        shape.hull = DcelConvexHull(mem, p, count, 100.0f*F32_EPSILON*size);
    } while (shape.hull.v_count < 4);

    return shape;
}

/* kind 0: box, 1: slab (one half width 1e-2 of the others), 2: random hull */
static void ChRandomHull(struct arena *mem, struct ch_Hull *h, const u32 kind)
{
    const f32 size = CtSize();
    const q rotation = CtRotation();
    const struct d3 pos = CtWorldPosition();
    if (kind < 2)
    {
        v3 hw = V3(size*RngF32Range(0.1f, 1.0f), size*RngF32Range(0.1f, 1.0f), size*RngF32Range(0.1f, 1.0f));
        if (kind == 1)
        {
            hw.y = 1e-2f*size;
        }
        const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(mem, hw) };
        ChHullInit(h, &box, 1, hw, rotation, pos);
    }
    else
    {
        const struct c_Shape hull = ChRandomHullShape(mem, size);
        ChHullInit(h, &hull, 0, V3(0.0f, 0.0f, 0.0f), rotation, pos);
    }
}

/* a point given in h's frame, in world space */
static struct d3 ChLocalToWorld(const struct ch_Hull *h, const v3 p)
{
    return D3Add(h->pos, D3RotationMul(h->r, D3V3(p)));
}

/*
 * Capsule relative to hull h, by mode (sizes relative to the hull):
 *  0: random pose near the hull
 *  1: lying along a face (tilted 0 .. 0.2 rad, around the face parallel threshold), at -r .. 1.3 r above it
 *  2: across an edge (perpendicular to it), at -r .. 1.3 r from it along the edge's face normal bisector
 *  3: along an edge (tilted 0 .. 0.05 rad, around the edge parallel threshold)
 *  4: center inside the hull, random direction
 *  5: half height 0, near the hull
 *  6: upright on a face (along its normal, tilted 0 .. 0.1 rad)
 */
static void ChCapsuleNear(struct ct_Capsule *c, const struct ch_Hull *h, const u32 mode)
{
    const struct dcel *dcel = &h->shape.hull;
    const f32 extent = (f32) h->extent;
    const f32 scale[] = { 0.01f, 0.1f, 0.5f, 1.0f, 2.0f };
    const f32 r = extent * scale[RngU64Range(0, 3)] * RngF32Range(0.5f, 1.0f);
    const f32 half_height = (mode == 5) ? 0.0f : extent * scale[RngU64Range(0, 4)] * RngF32Range(0.5f, 1.0f);
    const f32 offset = r * RngF32Range(-1.0f, 1.3f);

    /* a random face and a random half-edge */
    const u32 fi = (u32) RngU64Range(0, dcel->f_count - 1);
    const struct dcelFace *f = dcel->f + fi;
    v3 centroid = V3(0.0f, 0.0f, 0.0f);
    for (u32 i = 0; i < f->count; ++i)
    {
        centroid = V3Add(centroid, dcel->v[dcel->e[f->first + i].origin]);
    }
    centroid = V3Scale(centroid, 1.0f / (f32) f->count);
    const v3 fn = dcel->plane[fi].normal;
    const v3 corner = dcel->v[dcel->e[f->first + RngU64Range(0, f->count - 1)].origin];
    const v3 on_face = V3Add(centroid, V3Scale(V3Sub(corner, centroid), RngF32Range(0.0f, 1.0f)));

    const u32 ei = (u32) RngU64Range(0, dcel->e_count - 1);
    const u32 twin = dcel->e[ei].twin;
    const v3 e0 = dcel->v[dcel->e[ei].origin];
    const v3 e1 = dcel->v[dcel->e[twin].origin];
    const v3 edge_dir = V3Normalize(V3Sub(e1, e0));
    const v3 on_edge = V3Add(e0, V3Scale(V3Sub(e1, e0), RngF32Range(0.0f, 1.0f)));
    const v3 bisector = V3Normalize(V3Add(dcel->plane[dcel->e[ei].face_ccw].normal, dcel->plane[dcel->e[twin].face_ccw].normal));

    v3 center, dir;
    switch (mode)
    {
        case 1:
        {
            const f32 tilt[] = { 0.0f, 1e-4f, 1e-2f, 0.06f, 0.0632f, 0.0633f, 0.07f, 0.2f };
            const v3 tangent = ChPerpendicular(fn);
            dir = ChTilt(tangent, fn, tilt[RngU64Range(0, 7)] * ((RngU64Range(0, 1)) ? 1.0f : -1.0f));
            center = V3Add(on_face, V3Scale(fn, offset + F32Abs(V3Dot(dir, fn))*half_height));
        } break;

        case 2:
        {
            dir = ChTilt(V3Normalize(V3Cross(edge_dir, ChPerpendicular(edge_dir))), edge_dir, RngF32Range(-0.3f, 0.3f));
            center = V3Add(on_edge, V3Scale(bisector, offset));
        } break;

        case 3:
        {
            const f32 tilt[] = { 0.0f, 1e-4f, 1e-3f, 0.0086f, 0.0088f, 0.01f, 0.05f };
            dir = ChTilt(edge_dir, ChPerpendicular(edge_dir), tilt[RngU64Range(0, 6)]);
            center = V3Add(on_edge, V3Scale(bisector, offset));
        } break;

        case 4:
        {
            const v3 vertex = dcel->v[RngU64Range(0, dcel->v_count - 1)];
            center = V3Scale(vertex, RngF32Range(0.0f, 1.0f));
            dir = CtUnit();
        } break;

        case 6:
        {
            dir = ChTilt(fn, ChPerpendicular(fn), RngF32Range(0.0f, 0.1f));
            center = V3Add(on_face, V3Scale(fn, offset + half_height));
        } break;

        default:
        {
            center = V3Scale(CtUnit(), (extent + half_height + r) * RngF32Range(0.0f, 1.2f));
            dir = CtUnit();
        } break;
    }

    const q local = ChRotationFromY(dir);
    CtCapsuleInit(c, half_height, r, QMul(h->t.rotation, local), ChLocalToWorld(h, center));
}

/* hand-made cases on the box [-1, 1]^3 at the origin, unrotated; capsule radius 0.2 */
struct test_Output HullCapsuleContactCasesTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const v3 hw = V3(1.0f, 1.0f, 1.0f);
    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, hw) };
    struct ch_Hull h;
    ChHullInit(&h, &box, 1, hw, QIdentity(), D3(0.0, 0.0, 0.0));

    const q along_x = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -F32_PI/2.0f);      /* y -> x */
    struct ct_Capsule c;
    struct c_Manifold m;
    u32 contact;
    struct ch_Stats stats = { 0 };

    /* separated: lying along x at y = 1.3 (gap 0.3 > r) */
    CtCapsuleInit(&c, 0.5f, 0.2f, along_x, D3(0.0, 1.3, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(contact, 0);

    /* resting flat (shallow, gap 0.1): two points (+-0.5, 1, 0), n = +y, depth 0.1 */
    CtCapsuleInit(&c, 0.5f, 0.2f, along_x, D3(0.0, 1.1, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.1) && CtNear(m.depth[1], 0.1));
    TEST_TRUE(CtNear(F64AbsTest(m.v[0].x), 0.5) && CtNear(m.v[0].x, -m.v[1].x) && CtNear(m.v[0].y, 1.0));

    /* longer than the box: clipped to x = +-1 */
    CtCapsuleInit(&c, 2.0f, 0.2f, along_x, D3(0.0, 1.1, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNear(F64AbsTest(m.v[0].x), 1.0) && CtNear(m.v[0].x, -m.v[1].x));

    /* deep flat (core at y = 0.9): n = +y, depth r + 0.1 */
    CtCapsuleInit(&c, 0.5f, 0.2f, along_x, D3(0.0, 0.9, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 2);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.3) && CtNear(m.depth[1], 0.3) && CtNear(m.v[0].y, 1.0));

    /*
     * stick on the edge y = z = 1: core in the y-z plane at 45 deg on the line y + z = 2 - e. Edge axis
     * (0, 1, 1)/sqrt 2: -e/sqrt 2; top face clipped to its sides (z <= 1): -e. The edge wins iff
     * e (1 - 1/sqrt 2) > sat_face_bias (0.005), i.e. e > 0.017.
     * e = 0.05: n = edge axis, depth r + 0.05/sqrt 2, v = (0, 1, 1)
     */
    CtCapsuleInit(&c, 1.0f, 0.2f, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), -F32_PI/4.0f), D3(0.0, 0.975, 0.975));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 0.70710678, 0.70710678)));
    TEST_TRUE(CtNear(m.depth[0], 0.2 + 0.05/1.41421356));
    TEST_TRUE(CtNearD3(D3V3(m.v[0]), D3(0.0, 1.0, 1.0)));

    /* e = 0.01: the face bias keeps the top face: n = +y, one point (0, 1, 1) (the clipped end), depth r + 0.01 */
    CtCapsuleInit(&c, 1.0f, 0.2f, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), -F32_PI/4.0f), D3(0.0, 0.995, 0.995));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)));
    TEST_TRUE(CtNear(m.depth[0], 0.21));
    TEST_TRUE(CtNearD3(D3V3(m.v[0]), D3(0.0, 1.0, 1.0)));

    /* upright on the top face, low end at y = 1.1: one point (0, 1, 0), depth 0.1 */
    CtCapsuleInit(&c, 0.5f, 0.2f, QIdentity(), D3(0.0, 1.6, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)) && CtNear(m.depth[0], 0.1) && CtNearD3(D3V3(m.v[0]), D3(0.0, 1.0, 0.0)));

    /* upright, low end at y = 0.8 (deep): one point (0, 1, 0), depth 0.4 */
    CtCapsuleInit(&c, 0.5f, 0.2f, QIdentity(), D3(0.0, 1.3, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)) && CtNear(m.depth[0], 0.4) && CtNearD3(D3V3(m.v[0]), D3(0.0, 1.0, 0.0)));

    /* half height 0 (a sphere) at y = 0.95: n = +y, depth r + 0.05 (the clip of a point: two equal points) */
    CtCapsuleInit(&c, 0.0f, 0.2f, QIdentity(), D3(0.3, 0.95, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)) && CtNear(m.depth[0], 0.25));

    /* along the edge y = z = 1, 0.1 outside along both normals: GJK normal (0, 1, 1)/sqrt 2: one point on the edge */
    CtCapsuleInit(&c, 0.5f, 0.2f, along_x, D3(0.0, 1.1, 1.1));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 0.70710678, 0.70710678)) && CtNear(m.depth[0], 0.2 - 0.14142136));

    /*
     * audit H1: cores lying exactly on a slab's edge (touching). Rounding put the core outside the chosen face's side
     * plane, so the clip came out empty; the edge query then took a short edge accepted on noise (axis ~ the face
     * normal) and its clamped closest point: v on a vertex 6.7 mm from the core (2 cm slab). Both cores are
     * separated by less than GJK's touch band (the deep path runs), the second along an edge (skipped as parallel)
     * past the corner: no face or edge contact; GJK's touching point.
     */
    {
        const v3 slab_hw = V3(0.0199161991f, 0.000230795267f, 0.0167241152f);
        const struct c_Shape slab = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, slab_hw) };
        struct ch_Hull s_h;
        ChHullInit(&s_h, &slab, 1, slab_hw, Q(-0.174381122f, 0.0772159472f, 0.0419071764f, 0.980751097f), D3(0.0, 0.0, 0.0));
        CtCapsuleInit(&c, 0.00018988525f, 0.000168000537f, Q(-0.177893534f, -0.0687145814f, -0.663899481f, 0.723097265f),
            D3(-0.010536084430177315, 0.0048913000391571371, 0.017752716968054128));
        TEST_TRUE(ChContact(env->mem_2, &m, &contact, &s_h, &c) && ChVerify(&stats, contact, &m, &s_h, &c));
        TEST_EQUAL(contact, 1);
    }
    {
        const v3 slab_hw = V3(21.3581963f, 1.41826963f, 96.8468552f);
        const struct c_Shape slab = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, slab_hw) };
        struct ch_Hull s_h;
        ChHullInit(&s_h, &slab, 1, slab_hw, Q(-0.225835592f, -0.127913624f, -0.223402485f, 0.939535916f),
            D3(-261949.13634721929, 113098.77670043989, 51620.526489614349));
        CtCapsuleInit(&c, 64.103363f, 8.2836256f, Q(-0.0692858472f, -0.250127465f, -0.822304368f, 0.506410122f),
            D3(-261919.11767042111, 113043.83659593822, 51544.010835845787));
        TEST_TRUE(ChContact(env->mem_2, &m, &contact, &s_h, &c) && ChVerify(&stats, contact, &m, &s_h, &c));
        TEST_EQUAL(contact, 1);
    }

    /* lying on the top face tilted 0.2 rad: the high end is lifted beyond r: one point at the low end */
    CtCapsuleInit(&c, 0.5f, 0.2f, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -F32_PI/2.0f + 0.2f), D3(0.0, 1.1 + 0.5*0.19866933, 0.0));
    TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
    TEST_EQUAL(m.v_count, 1);
    TEST_TRUE(CtNearD3(D3V3(m.n), D3(0.0, 1.0, 0.0)) && CtNear(m.depth[0], 0.1));

	return output;
}

/* random hulls (boxes, slabs, hulls) and capsules, every mode, checked against the reference */
struct test_Output HullCapsuleContactRandomTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const char *kind_id[] = { "box", "slab", "hull" };
    const char *mode_id[] = { "random", "on face", "across edge", "along edge", "inside", "zero length", "upright" };
    const u32 count[] = { 20000, 10000, 4000 };
    for (u32 kind = 0; kind < 3; ++kind)
    {
        for (u32 mode = 0; mode < 7; ++mode)
        {
            struct ch_Stats stats = { 0 };
            for (u32 i = 0; i < count[kind]; ++i)
            {
                struct arena record = *env->mem_2;
                struct ch_Hull h;
                struct ct_Capsule c;
                ChRandomHull(env->mem_2, &h, kind);
                ChCapsuleNear(&c, &h, mode);
                const u32 ok = ChCheckPair(env->mem_2, &stats, &h, &c);
                *env->mem_2 = record;
                if (!ok)
                {
                    fprintf(stderr, "\t(%s, %s, case %u)\n", kind_id[kind], mode_id[mode], i);
                    TEST_FAILURE;
                }
            }
            char id[64];
            snprintf(id, sizeof(id), "%s, %s", kind_id[kind], mode_id[mode]);
            ChStatsPrint(id, &stats);
        }
    }

	return output;
}

/* a sweep on the box [-1, 1]^3: the largest jumps of the normal and the deepest depth between neighbours */
struct ch_Sweep
{
    u32 prev_contact;
    u32 prev_count;
    struct c_Manifold prev;
    f64 worst_n;
    f64 worst_depth;
    u32 count_changes;
    u32 normal_changes;
};

static f64 ChDeepest(const struct c_Manifold *m)
{
    return (m->v_count == 2) ? F64MaxTest(m->depth[0], m->depth[1]) : m->depth[0];
}

static void ChSweepStep(struct ch_Sweep *s, const u32 contact, const struct c_Manifold *m)
{
    if (contact && s->prev_contact)
    {
        const f64 dn = D3Length(D3Sub(D3V3(m->n), D3V3(s->prev.n)));
        s->worst_n = F64MaxTest(s->worst_n, dn);
        s->worst_depth = F64MaxTest(s->worst_depth, F64AbsTest(ChDeepest(m) - ChDeepest(&s->prev)));
        s->count_changes += (m->v_count != s->prev.v_count);
        s->normal_changes += (dn > 1e-3);
    }
    s->prev_contact = contact;
    if (contact)
    {
        s->prev = *m;
    }
}

/*
 * Continuity sweeps (h = 1, r = 0.2), every manifold verified:
 *  tilt:  lying on the top face, low end 0.1 above it, tilted 0 .. 0.2 rad (two points -> one point)
 *  sink:  tilted 0.02 rad, low end from 0.1 above to 0.1 below the face (shallow -> deep)
 *  edge:  centered 0.05 inside the edge y = z = 1 (along the bisector), turned about x from along z (top face)
 *         through across the edge (edge axis) to along y (front face)
 */
struct test_Output HullCapsuleContactSweepTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const v3 hw = V3(1.0f, 1.0f, 1.0f);
    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, hw) };
    struct ch_Hull h;
    ChHullInit(&h, &box, 1, hw, QIdentity(), D3(0.0, 0.0, 0.0));
    struct ch_Stats stats = { 0 };
    const f32 half_height = 1.0f;
    const f32 r = 0.2f;

    struct ch_Sweep tilt = { 0 };
    for (u32 k = 0; k <= 2000; ++k)
    {
        const f32 a = 1e-4f*(f32) k;
        /* core along x tilted by a about z; low end (x = -cos a) at y = 1.1 */
        struct ct_Capsule c;
        CtCapsuleInit(&c, half_height, r, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -F32_PI/2.0f + a), D3(0.0, 1.1 + (f64) half_height*F32Sin(a), 0.0));
        struct c_Manifold m;
        u32 contact;
        TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
        ChSweepStep(&tilt, contact, &m);
    }

    struct ch_Sweep sink = { 0 };
    for (u32 k = 0; k <= 2000; ++k)
    {
        const f32 a = 0.02f;
        const f64 low = 1.1 - 1e-4*(f64) k;
        struct ct_Capsule c;
        CtCapsuleInit(&c, half_height, r, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -F32_PI/2.0f + a), D3(0.0, low + (f64) half_height*F32Sin(a), 0.0));
        struct c_Manifold m;
        u32 contact;
        TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
        ChSweepStep(&sink, contact, &m);
    }

    struct ch_Sweep edge = { 0 };
    for (u32 k = 0; k <= 2000; ++k)
    {
        /* core direction (0, -sin phi, cos phi): along z (phi = 0), across the edge (pi/4), along -y (pi/2) */
        const f32 phi = (F32_PI/2.0f) * (f32) k / 2000.0f;
        const f64 inside = 0.05;
        struct ct_Capsule c;
        CtCapsuleInit(&c, half_height, r, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI/2.0f + phi), D3(0.0, 1.0 - inside/1.41421356, 1.0 - inside/1.41421356));
        struct c_Manifold m;
        u32 contact;
        TEST_TRUE(ChContact(env->mem_2, &m, &contact, &h, &c) && ChVerify(&stats, contact, &m, &h, &c));
        ChSweepStep(&edge, contact, &m);
        if (k == 1000)
        {
            /* across the edge: the edge axis */
            TEST_TRUE(contact && CtNearD3(D3V3(m.n), D3(0.0, 0.70710678, 0.70710678)));
        }
    }

    fprintf(stdout, "\t%s: largest jumps between neighbours |dn|, |d deepest|; point count changes; normal changes:\n", __func__);
    fprintf(stdout, "\t\ttilt (1e-4 rad): %.3g, %.3g; %u; %u\n", tilt.worst_n, tilt.worst_depth, tilt.count_changes, tilt.normal_changes);
    fprintf(stdout, "\t\tsink (1e-4):     %.3g, %.3g; %u; %u\n", sink.worst_n, sink.worst_depth, sink.count_changes, sink.normal_changes);
    fprintf(stdout, "\t\tedge (pi/4000):  %.3g, %.3g; %u; %u\n", edge.worst_n, edge.worst_depth, edge.count_changes, edge.normal_changes);

	return output;
}

/* ns per call on prepared random pairs (all kinds and modes), split by whether the core intersects the hull */
struct ch_TimingPair
{
    struct c_Shape      hull;
    ds_TransformWorld   t;
    struct ct_Capsule   capsule;
    u32                 deep;
};

struct test_Output HullCapsuleContactTimingTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();

    const u32 pair_count = 1024;
    struct ch_TimingPair *pair = ArenaPush(env->mem_1, pair_count*sizeof(struct ch_TimingPair));
    for (u32 i = 0; i < pair_count; ++i)
    {
        struct ch_Hull h;
        ChRandomHull(env->mem_1, &h, (u32) RngU64Range(0, 2));
        ChCapsuleNear(&pair[i].capsule, &h, (u32) RngU64Range(0, 6));
        struct d3 a, b;
        ChCore(&a, &b, &h, &pair[i].capsule);
        pair[i].hull = h.shape;
        pair[i].t = h.t;
        pair[i].deep = (ChSat(&h, a, b) < 0.0);
    }

    u64 ns[2] = { 0 };
    u64 calls[2] = { 0 };
    u64 manifolds = 0;
    for (u32 round = 0; round < 200; ++round)
    {
        for (u32 i = 0; i < pair_count; ++i)
        {
            struct arena record = *env->mem_2;
            const struct c_Shape *s[2] = { &pair[i].hull, &pair[i].capsule.shape };
            const ds_TransformWorld t[2] = { pair[i].t, pair[i].capsule.t };
            const struct c_ContactResult cached = { 0 };
            const u64 start = ds_TimeNs();
            const struct c_ContactResult result = TMP_c_HullCapsuleContact(env->mem_2, &cached, s, t);
            const u64 end = ds_TimeNs();
            manifolds += result.manifold_count;
            ns[pair[i].deep] += end - start;
            calls[pair[i].deep] += 1;
            *env->mem_2 = record;
        }
    }

    fprintf(stdout, "\t%s: %.0f ns per call core outside (%llu calls), %.0f ns core inside (%llu calls), %llu manifolds; includes ds_TimeNs, DS_DEBUG asserts\n", __func__,
        (f64) ns[0] / (f64) calls[0], (unsigned long long) calls[0], (f64) ns[1] / (f64) calls[1], (unsigned long long) calls[1], (unsigned long long) manifolds);

	return output;
}

/********************************** hull vs hull **********************************/

/*
Hull vs hull (TMP_c_HullContact), checked in double precision in A's frame (the manifold's frame). B's vertices,
face normals and edge directions are moved into A's frame in double. With sep(n) = min_B n.x - max_A n.x:

    sat = max sep(n) over A's face normals, minus B's face normals and +-cross(edge A, edge B)
        = the signed distance if A and B intersect (exact: the face normals of the Minkowski difference)

Every manifold must satisfy, with L = max(extent A, extent B, |x_B - x_A|), tol = CONTACT_TEST_TOLERANCE eps L:

    contact iff sat < 0                         (undecided within tol)
    c_ManifoldCheck, v_count in 1..4, |n| = 1
    depth_i <= -sep(n) + tol                    (n separates as deep as it claims)
    v_i on A's surface                          |sdf_A(v_i)| <= tol
    v_i - depth_i n on B's surface              |sdf_B(v_i - depth_i n)| <= tol (point and depth agree)
    translating both by an exactly representable offset gives identical bits
    the same checks with the previous result as the cache: same pose, and B moved by ~1e-3 L; a cached result
    may claim up to sat_cache_tolerance more depth (also for separated shapes): the cache's
    acceptance tolerance

Failures are counted by kind (first example printed); any failure fails the test.
*/

static struct ds_DynamicsWorker hh_worker[DS_THREAD_COUNT_MAX];

enum hh_Fail
{
    HH_FAIL_NONE,
    HH_FAIL_INVALID,        /* manifold count, c_ManifoldCheck, v_count */
    HH_FAIL_MISSED,         /* sat < -tol, no contact */
    HH_FAIL_FALSE,          /* sat > tol, contact */
    HH_FAIL_NORMAL,         /* |n| != 1 */
    HH_FAIL_DEPTH,          /* depth > -sep(n) + tol */
    HH_FAIL_SURFACE_A,      /* v not on A's surface */
    HH_FAIL_SURFACE_B,      /* v - depth n not on B's surface */
    HH_FAIL_TRANSLATION,    /* translated result differs */
    HH_FAIL_COUNT
};

static const char *hh_fail_id[HH_FAIL_COUNT] =
{
    "none", "invalid", "missed contact", "false contact", "|n| != 1", "depth deeper than sep(n)",
    "v off A", "v - depth n off B", "translation",
};

struct hh_Stats
{
    u64 tests;
    u64 contacts;
    u64 v_count[5];
    u64 undecided;
    u64 fail[HH_FAIL_COUNT];
    f64 worst_surface_a;    /* / (eps L) */
    f64 worst_surface_b;    /* / (eps L) */
    f64 worst_gap;          /* deep: (sat - sep(n)) / L */
};

/* B moved into A's frame (double): vertices, face normals and offsets, edge directions */
static void HhHullInFrame(struct ch_Hull *out, const struct ch_Hull *b, const struct ch_Hull *a)
{
    *out = *b;
    const struct dcel *dcel = &b->shape.hull;
    for (u32 i = 0; i < b->v_count; ++i)
    {
        out->v[i] = D3RotationTMul(a->r, D3Sub(D3Add(D3RotationMul(b->r, b->v[i]), b->pos), a->pos));
    }
    for (u32 fi = 0; fi < b->f_count; ++fi)
    {
        out->fn[fi] = D3RotationTMul(a->r, D3RotationMul(b->r, b->fn[fi]));
        const struct dcelFace *f = dcel->f + fi;
        out->fd[fi] = -1e300;
        for (u32 i = 0; i < f->count; ++i)
        {
            out->fd[fi] = F64MaxTest(out->fd[fi], D3Dot(out->fn[fi], out->v[dcel->e[f->first + i].origin]));
        }
    }
    for (u32 i = 0; i < b->e_count; ++i)
    {
        out->ed[i] = D3RotationTMul(a->r, D3RotationMul(b->r, b->ed[i]));
    }
}

static f64 HhMin(const struct ch_Hull *h, const struct d3 n)
{
    f64 min = 1e300;
    for (u32 i = 0; i < h->v_count; ++i)
    {
        min = F64MinTest(min, D3Dot(n, h->v[i]));
    }
    return min;
}

/* separation of A and B (in A's frame) along unit n, from A towards B */
static f64 HhSeparation(const struct ch_Hull *a, const struct ch_Hull *b, const struct d3 n)
{
    return HhMin(b, n) - ChSupport(a, n);
}

static f64 HhSat(const struct ch_Hull *a, const struct ch_Hull *b)
{
    f64 sat = -1e300;
    for (u32 fi = 0; fi < a->f_count; ++fi)
    {
        sat = F64MaxTest(sat, HhSeparation(a, b, a->fn[fi]));
    }
    for (u32 fi = 0; fi < b->f_count; ++fi)
    {
        sat = F64MaxTest(sat, HhSeparation(a, b, D3Scale(b->fn[fi], -1.0)));
    }
    for (u32 i = 0; i < a->e_count; ++i)
    {
        for (u32 j = 0; j < b->e_count; ++j)
        {
            const struct d3 c = D3Cross(a->ed[i], b->ed[j]);
            const f64 c_sq = D3Dot(c, c);
            if (c_sq > 1e-24*D3Dot(a->ed[i], a->ed[i])*D3Dot(b->ed[j], b->ed[j]) && c_sq > 0.0)
            {
                const struct d3 n = D3Scale(c, 1.0 / F64SqrtTest(c_sq));
                sat = F64MaxTest(sat, HhSeparation(a, b, n));
                sat = F64MaxTest(sat, HhSeparation(a, b, D3Scale(n, -1.0)));
            }
        }
    }
    return sat;
}

static void HhPrint(const struct ch_Hull *a, const struct ch_Hull *b)
{
    const struct ch_Hull *h[2] = { a, b };
    for (u32 k = 0; k < 2; ++k)
    {
        if (h[k]->is_box)
        {
            fprintf(stderr, "\t%c: box hw { %.9g, %.9g, %.9g }", 'A' + k, h[k]->hw.x, h[k]->hw.y, h[k]->hw.z);
        }
        else
        {
            fprintf(stderr, "\t%c: hull %u vertices, extent %.9g", 'A' + k, h[k]->v_count, h[k]->extent);
        }
        fprintf(stderr, " rot { %.9g, %.9g, %.9g, %.9g } pos { %.17g, %.17g, %.17g }\n", h[k]->t.rotation.x,
            h[k]->t.rotation.y, h[k]->t.rotation.z, h[k]->t.rotation.w, h[k]->pos.x, h[k]->pos.y, h[k]->pos.z);
    }
}

/* contact of (a, b); 0 and HH_FAIL_INVALID printed if the result is malformed */
static u32 HhContact(struct arena *frame, struct c_ContactResult *result, const struct ch_Hull *a, const struct ch_Hull *b, const struct c_ContactResult *cached)
{
    const struct c_Shape *s[2] = { &a->shape, &b->shape };
    const ds_TransformWorld t[2] = { a->t, b->t };
    const struct c_ContactResult none = { 0 };
    *result = TMP_c_HullContact(frame, (cached) ? cached : &none, s, t);
    return result->manifold_count == 0
        || (result->manifold_count == 1 && c_ManifoldCheck(result->manifold)
            && 1 <= result->manifold->v_count && result->manifold->v_count <= 4);
}

/* checks one result of (a, b) against the reference; returns the failure kind */
static enum hh_Fail HhVerifySlack(struct hh_Stats *stats, const struct c_ContactResult *result, const struct ch_Hull *a, const struct ch_Hull *b, const f64 slack)
{
    struct ch_Hull bt;
    HhHullInFrame(&bt, b, a);
    const f64 L = F64MaxTest(F64MaxTest(a->extent, b->extent), D3Length(D3Sub(b->pos, a->pos)));
    const f64 eps_l = F32_EPSILON * L;
    const f64 tol = CONTACT_TEST_TOLERANCE * eps_l;
    const f64 sat = HhSat(a, &bt);
    const u32 contact = (result->manifold_count == 1);

    stats->tests += 1;
    if (F64AbsTest(sat) <= tol)
    {
        stats->undecided += 1;
    }
    else if (sat < 0.0 && !contact)
    {
        return HH_FAIL_MISSED;
    }
    else if (sat > slack && contact)
    {
        return HH_FAIL_FALSE;
    }

    if (!contact)
    {
        return HH_FAIL_NONE;
    }

    const struct c_Manifold *m = result->manifold;
    const struct d3 n = D3V3(m->n);
    const f64 n_len = D3Length(n);
    if (F64AbsTest(n_len - 1.0) > 1e-5)
    {
        return HH_FAIL_NORMAL;
    }

    const f64 sep = HhSeparation(a, &bt, D3Scale(n, 1.0 / n_len));
    for (u32 i = 0; i < m->v_count; ++i)
    {
        const struct d3 v = D3V3(m->v[i]);
        const f64 surface_a = F64AbsTest(ChHullSdf(a, v));
        const f64 surface_b = F64AbsTest(ChHullSdf(&bt, D3Sub(v, D3Scale(n, m->depth[i]))));
        stats->worst_surface_a = F64MaxTest(stats->worst_surface_a, surface_a / eps_l);
        stats->worst_surface_b = F64MaxTest(stats->worst_surface_b, surface_b / eps_l);
        if (m->depth[i] > -sep + tol + slack)
        {
            return HH_FAIL_DEPTH;
        }
        if (surface_a > tol + slack)
        {
            return HH_FAIL_SURFACE_A;
        }
        if (surface_b > tol + slack)
        {
            return HH_FAIL_SURFACE_B;
        }
    }

    if (sat < -tol)
    {
        stats->worst_gap = F64MaxTest(stats->worst_gap, (sat - sep) / L);
    }
    stats->contacts += 1;
    stats->v_count[m->v_count] += 1;
    return HH_FAIL_NONE;
}

static enum hh_Fail HhVerify(struct hh_Stats *stats, const struct c_ContactResult *result, const struct ch_Hull *a, const struct ch_Hull *b)
{
    return HhVerifySlack(stats, result, a, b, 0.0);
}

/*
 * A cached result is accepted while its depth moved by less than sat_cache_tolerance (the cache
 * trades exactness for coherence): it may claim that much depth beyond the reference, also for separated shapes,
 * and its points may lie that far off the surfaces (a face rebuilt for separated hulls: contact_audit.txt I3).
 */
static enum hh_Fail HhVerifyCached(struct hh_Stats *stats, const struct c_ContactResult *result, const struct ch_Hull *a, const struct ch_Hull *b)
{
    return HhVerifySlack(stats, result, a, b, g_numerics_config->sat_cache_tolerance);
}

static void HhCount(struct hh_Stats *stats, const enum hh_Fail fail, const struct c_ContactResult *result, const struct ch_Hull *a, const struct ch_Hull *b, const char *what)
{
    if (fail == HH_FAIL_NONE)
    {
        return;
    }
    if (stats->fail[fail] == 0)
    {
        struct ch_Hull bt;
        HhHullInFrame(&bt, b, a);
        fprintf(stderr, "\tHull contact (%s) failed: %s, sat %.9g\n", what, hh_fail_id[fail], HhSat(a, &bt));
        HhPrint(a, b);
        if (result->manifold_count)
        {
            c_ManifoldDebugPrint(result->manifold);
        }
    }
    stats->fail[fail] += 1;
}

/* contact of (a, b): fresh, translated, with the cache (same pose and b moved) */
static void HhCheckPair(struct arena *frame, struct hh_Stats *fresh, struct hh_Stats *cached, const struct ch_Hull *a, const struct ch_Hull *b)
{
    struct c_ContactResult r;
    if (!HhContact(frame, &r, a, b, NULL))
    {
        HhCount(fresh, HH_FAIL_INVALID, &r, a, b, "fresh");
        return;
    }
    HhCount(fresh, HhVerify(fresh, &r, a, b), &r, a, b, "fresh");

    /* an exactly representable offset leaves x_B - x_A exact in f64: identical bits */
    struct ch_Hull a_t = *a, b_t = *b;
    const struct d3 offset = D3(4096.0, -1024.0, 65536.0);
    a_t.t.position = V3F64(a->pos.x + offset.x, a->pos.y + offset.y, a->pos.z + offset.z);
    b_t.t.position = V3F64(b->pos.x + offset.x, b->pos.y + offset.y, b->pos.z + offset.z);
    if (V3F64Sub(b_t.t.position, a_t.t.position).x == V3F64Sub(b->t.position, a->t.position).x
     && V3F64Sub(b_t.t.position, a_t.t.position).y == V3F64Sub(b->t.position, a->t.position).y
     && V3F64Sub(b_t.t.position, a_t.t.position).z == V3F64Sub(b->t.position, a->t.position).z)
    {
        struct c_ContactResult r_t;
        HhContact(frame, &r_t, &a_t, &b_t, NULL);
        u32 same = (r_t.manifold_count == r.manifold_count);
        if (same && r.manifold_count)
        {
            const struct c_Manifold *m = r.manifold, *m_t = r_t.manifold;
            same = m_t->v_count == m->v_count && memcmp(&m_t->n, &m->n, sizeof(v3)) == 0;
            for (u32 i = 0; same && i < m->v_count; ++i)
            {
                same = memcmp(&m_t->v[i], &m->v[i], sizeof(v3)) == 0 && memcmp(&m_t->depth[i], &m->depth[i], sizeof(f32)) == 0;
            }
        }
        if (!same)
        {
            HhCount(fresh, HH_FAIL_TRANSLATION, &r, a, b, "translated");
        }
    }

    /* the cache: same pose, then b moved by ~1e-3 L */
    struct c_ContactResult r_c;
    if (!HhContact(frame, &r_c, a, b, &r))
    {
        HhCount(cached, HH_FAIL_INVALID, &r_c, a, b, "cached, same pose");
        return;
    }
    HhCount(cached, HhVerifyCached(cached, &r_c, a, b), &r_c, a, b, "cached, same pose");

    struct ch_Hull b_m = *b;
    const f64 L = F64MaxTest(F64MaxTest(a->extent, b->extent), D3Length(D3Sub(b->pos, a->pos)));
    b_m.pos = D3Add(b->pos, D3Scale(D3V3(CtUnit()), 1e-3*L));
    b_m.t.position = V3F64(b_m.pos.x, b_m.pos.y, b_m.pos.z);
    if (!HhContact(frame, &r_c, a, &b_m, &r))
    {
        HhCount(cached, HH_FAIL_INVALID, &r_c, a, &b_m, "cached, moved");
        return;
    }
    HhCount(cached, HhVerifyCached(cached, &r_c, a, &b_m), &r_c, a, &b_m, "cached, moved");
}

static u64 HhFailures(const struct hh_Stats *s)
{
    u64 count = 0;
    for (u32 k = 1; k < HH_FAIL_COUNT; ++k)
    {
        count += s->fail[k];
    }
    return count;
}

static void HhStatsPrint(const char *id, const struct hh_Stats *s)
{
    fprintf(stdout, "\t%s: %llu tests, %llu contacts (points 1/2/3/4: %llu/%llu/%llu/%llu), %llu undecided; worst / (eps L): surface A %.1f, B %.1f; deep gap / L %.3g; failures:",
        id, (unsigned long long) s->tests, (unsigned long long) s->contacts, (unsigned long long) s->v_count[1],
        (unsigned long long) s->v_count[2], (unsigned long long) s->v_count[3], (unsigned long long) s->v_count[4],
        (unsigned long long) s->undecided, s->worst_surface_a, s->worst_surface_b, s->worst_gap);
    u32 any = 0;
    for (u32 k = 1; k < HH_FAIL_COUNT; ++k)
    {
        if (s->fail[k])
        {
            fprintf(stdout, " %s %llu,", hh_fail_id[k], (unsigned long long) s->fail[k]);
            any = 1;
        }
    }
    fprintf(stdout, "%s\n", (any) ? "" : " none");
}

/* rotation taking unit a to unit b */
static q HhRotationFromTo(const v3 a, const v3 b)
{
    const v3 axis = V3Cross(a, b);
    const f32 s = V3Length(axis);
    const f32 c = V3Dot(a, b);
    if (s < 1e-6f)
    {
        return (c > 0.0f) ? QIdentity() : QUnitAxisAngle(ChPerpendicular(a), F32_PI);
    }
    return QUnitAxisAngle(V3Scale(axis, 1.0f / s), F32Acos(F32Clamp(c, -1.0f, 1.0f)));
}

/* kind 0: box, 1: slab (one half width 1e-2 of the others), 2: random hull; size = its scale */
static void HhHull(struct arena *mem, struct ch_Hull *h, const u32 kind, const f32 size, const q rotation, const struct d3 pos)
{
    if (kind < 2)
    {
        v3 hw = V3(size*RngF32Range(0.1f, 1.0f), size*RngF32Range(0.1f, 1.0f), size*RngF32Range(0.1f, 1.0f));
        if (kind == 1)
        {
            hw.y = 1e-2f*size;
        }
        const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(mem, hw) };
        ChHullInit(h, &box, 1, hw, rotation, pos);
    }
    else
    {
        const struct c_Shape hull = ChRandomHullShape(mem, size);
        ChHullInit(h, &hull, 0, V3(0.0f, 0.0f, 0.0f), rotation, pos);
    }
}

static v3 HhFaceCentroid(const struct dcel *d, const u32 fi)
{
    const struct dcelFace *f = d->f + fi;
    v3 c = V3(0.0f, 0.0f, 0.0f);
    for (u32 i = 0; i < f->count; ++i)
    {
        c = V3Add(c, d->v[d->e[f->first + i].origin]);
    }
    return V3Scale(c, 1.0f / (f32) f->count);
}

/* min over d's vertices of n.(R v) */
static f32 HhMinRotated(const struct dcel *d, const q rot, const v3 n)
{
    f32 min = F32_INFINITY;
    for (u32 i = 0; i < d->v_count; ++i)
    {
        min = F32Min(min, V3Dot(n, QUnitRotate(rot, d->v[i])));
    }
    return min;
}

/*
 * B relative to A (given), by mode; B's local rotation and center in A's frame, then world:
 *  0: random pose near A
 *  1: face on face (B's face anti-parallel to A's, tilted 0 .. 0.2 rad), gap -0.05 .. 0.02 extent B
 *  2: edge across edge (B's edge bisector opposite A's, random twist), gap -0.03 .. 0.02 extents
 *  3: vertex on face (a vertex of B pointing into A's face), gap -0.05 .. 0.02 extent B
 *  4: center inside A
 *  5: apart (no contact): 1 .. 2 times the extents
 */
static void HhHullNear(struct arena *mem, struct ch_Hull *b, const struct ch_Hull *a, const u32 kind, const u32 mode)
{
    const struct dcel *da = &a->shape.hull;
    const f32 ext_a = (f32) a->extent;
    const f32 size = ext_a * RngF32Range(0.2f, 3.0f);

    /* B's shape first (at the origin), to place it */
    struct ch_Hull tmp;
    HhHull(mem, &tmp, kind, size, QIdentity(), D3(0.0, 0.0, 0.0));
    const struct dcel *db = &tmp.shape.hull;
    const f32 ext_b = (f32) tmp.extent;

    q rot = CtRotation();
    v3 center = V3(0.0f, 0.0f, 0.0f);
    switch (mode)
    {
        case 1:
        {
            const u32 fa = (u32) RngU64Range(0, da->f_count - 1);
            const u32 fb = (u32) RngU64Range(0, db->f_count - 1);
            const v3 n = da->plane[fa].normal;
            const f32 tilt[] = { 0.0f, 1e-3f, 1e-2f, 0.05f, 0.2f };
            rot = HhRotationFromTo(db->plane[fb].normal, V3Negate(n));
            rot = QMul(QUnitAxisAngle(n, RngF32Range(0.0f, 2.0f*F32_PI)), rot);
            rot = QMul(QUnitAxisAngle(ChPerpendicular(n), tilt[RngU64Range(0, 4)]), rot);
            const v3 corner = da->v[da->e[da->f[fa].first].origin];
            const v3 centroid = HhFaceCentroid(da, fa);
            const v3 on_face = V3Add(centroid, V3Scale(V3Sub(corner, centroid), RngF32Range(0.0f, 1.0f)));
            const f32 gap = ext_b * RngF32Range(-0.05f, 0.02f);
            center = V3Add(on_face, V3Scale(n, gap - HhMinRotated(db, rot, n)));
        } break;

        case 2:
        {
            const u32 ea = (u32) RngU64Range(0, da->e_count - 1);
            const u32 eb = (u32) RngU64Range(0, db->e_count - 1);
            const v3 ua = V3Normalize(V3Add(da->plane[da->e[ea].face_ccw].normal, da->plane[da->e[da->e[ea].twin].face_ccw].normal));
            const v3 ub = V3Normalize(V3Add(db->plane[db->e[eb].face_ccw].normal, db->plane[db->e[db->e[eb].twin].face_ccw].normal));
            rot = HhRotationFromTo(ub, V3Negate(ua));
            rot = QMul(QUnitAxisAngle(ua, RngF32Range(0.0f, 2.0f*F32_PI)), rot);
            const v3 a0 = da->v[da->e[ea].origin];
            const v3 a1 = da->v[da->e[da->e[ea].twin].origin];
            const v3 on_edge = V3Add(a0, V3Scale(V3Sub(a1, a0), RngF32Range(0.0f, 1.0f)));
            const v3 b_mid = V3Scale(V3Add(db->v[db->e[eb].origin], db->v[db->e[db->e[eb].twin].origin]), 0.5f);
            const f32 gap = (ext_a + ext_b) * RngF32Range(-0.03f, 0.02f);
            center = V3Sub(V3Add(on_edge, V3Scale(ua, gap)), QUnitRotate(rot, b_mid));
        } break;

        case 3:
        {
            const u32 fa = (u32) RngU64Range(0, da->f_count - 1);
            const v3 n = da->plane[fa].normal;
            const v3 vb = db->v[RngU64Range(0, db->v_count - 1)];
            rot = HhRotationFromTo(V3Normalize(vb), V3Negate(n));
            rot = QMul(QUnitAxisAngle(n, RngF32Range(0.0f, 2.0f*F32_PI)), rot);
            const v3 corner = da->v[da->e[da->f[fa].first].origin];
            const v3 centroid = HhFaceCentroid(da, fa);
            const v3 on_face = V3Add(centroid, V3Scale(V3Sub(corner, centroid), RngF32Range(0.0f, 1.0f)));
            const f32 gap = ext_b * RngF32Range(-0.05f, 0.02f);
            center = V3Add(on_face, V3Scale(n, gap - HhMinRotated(db, rot, n)));
        } break;

        case 4:
        {
            center = V3Scale(da->v[RngU64Range(0, da->v_count - 1)], RngF32Range(0.0f, 0.8f));
        } break;

        case 5:
        {
            center = V3Scale(CtUnit(), (ext_a + ext_b) * RngF32Range(1.0f, 2.0f));
        } break;

        default:
        {
            center = V3Scale(CtUnit(), (ext_a + ext_b) * RngF32Range(0.0f, 1.2f));
        } break;
    }

    ChHullInit(b, &tmp.shape, tmp.is_box, V3((f32) tmp.hw.x, (f32) tmp.hw.y, (f32) tmp.hw.z), QMul(a->t.rotation, rot), ChLocalToWorld(a, center));
}

static struct ds_DynamicsWorker *HhWorkerPush(void)
{
    struct ds_DynamicsWorker *prev = g_dynamics_worker;
    memset(hh_worker, 0, sizeof(hh_worker));
    g_dynamics_worker = hh_worker;
    return prev;
}

/* hand-made cases: A = [-1, 1]^3 at the origin, unrotated */
struct test_Output HullContactCasesTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();
    struct ds_DynamicsWorker *prev = HhWorkerPush();

    const v3 hw = V3(1.0f, 1.0f, 1.0f);
    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, hw) };
    struct ch_Hull a, b;
    ChHullInit(&a, &box, 1, hw, QIdentity(), D3(0.0, 0.0, 0.0));
    struct hh_Stats stats = { 0 };
    struct c_ContactResult r;

    /* separated: B 0.1 above */
    ChHullInit(&b, &box, 1, hw, QIdentity(), D3(0.0, 2.1, 0.0));
    TEST_TRUE(HhContact(env->mem_2, &r, &a, &b, NULL) && HhVerify(&stats, &r, &a, &b) == HH_FAIL_NONE);
    TEST_EQUAL(r.manifold_count, 0);

    /* face on face, 0.1 deep: four points at the corners (+-1, 1, +-1) (on A's side planes), n = +y, depth 0.1 */
    ChHullInit(&b, &box, 1, hw, QIdentity(), D3(0.0, 1.9, 0.0));
    TEST_TRUE(HhContact(env->mem_2, &r, &a, &b, NULL) && HhVerify(&stats, &r, &a, &b) == HH_FAIL_NONE);
    TEST_EQUAL(r.manifold->v_count, 4);
    TEST_TRUE(CtNearD3(D3V3(r.manifold->n), D3(0.0, 1.0, 0.0)));
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(CtNear(r.manifold->depth[i], 0.1) && CtNear(r.manifold->v[i].y, 1.0));
    }

    /* B turned 45 deg about y: the clip is an octagon, reduced to four points, all 0.1 deep */
    ChHullInit(&b, &box, 1, hw, QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), F32_PI/4.0f), D3(0.0, 1.9, 0.0));
    TEST_TRUE(HhContact(env->mem_2, &r, &a, &b, NULL) && HhVerify(&stats, &r, &a, &b) == HH_FAIL_NONE);
    TEST_EQUAL(r.manifold->v_count, 4);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(CtNear(r.manifold->depth[i], 0.1));
    }

    /* B turned 45 deg about x, its lower edge (along x, ends on A's side planes) 0.05 into the top face: two points (+-1, 1, 0), depth 0.05 */
    ChHullInit(&b, &box, 1, hw, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), F32_PI/4.0f), D3(0.0, 0.95 + 1.41421356, 0.0));
    TEST_TRUE(HhContact(env->mem_2, &r, &a, &b, NULL) && HhVerify(&stats, &r, &a, &b) == HH_FAIL_NONE);
    TEST_EQUAL(r.manifold->v_count, 2);
    TEST_TRUE(CtNearD3(D3V3(r.manifold->n), D3(0.0, 1.0, 0.0)));
    TEST_TRUE(CtNear(r.manifold->depth[0], 0.05) && CtNear(r.manifold->depth[1], 0.05));
    TEST_TRUE(CtNear(F64AbsTest(r.manifold->v[0].x), 1.0) && CtNear(r.manifold->v[0].x, -r.manifold->v[1].x));

    g_dynamics_worker = prev;
	return output;
}

/* random pairs: {box, box}, {box, hull}, {hull, hull}, {slab, box} x every mode; fresh, translated and cached */
struct test_Output HullContactRandomTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ContactTestConfigPush();
    struct ds_DynamicsWorker *prev = HhWorkerPush();

    const char *pair_id[] = { "box/box", "box/hull", "hull/hull", "slab/box" };
    const u32 kind_a[] = { 0, 0, 2, 1 };
    const u32 kind_b[] = { 0, 2, 2, 0 };
    const char *mode_id[] = { "random", "face on face", "edge on edge", "vertex on face", "inside", "apart" };
    const u32 count[] = { 4000, 2000, 1000, 2000 };
    u64 failures = 0;
    for (u32 far = 0; far < 2; ++far)
    for (u32 p = 0; p < 4; ++p)
    {
        for (u32 mode = 0; mode < 6; ++mode)
        {
            struct hh_Stats fresh = { 0 }, cached = { 0 };
            for (u32 i = 0; i < count[p]; ++i)
            {
                struct arena record = *env->mem_2;
                struct ch_Hull a, b;
                /* near the origin (|x| <= 1), then anywhere up to 1e6 */
                const struct d3 pos = (far) ? CtWorldPosition() : D3Scale(D3V3(CtUnit()), RngF32Normalized());
                HhHull(env->mem_2, &a, kind_a[p], CtSize(), CtRotation(), pos);
                HhHullNear(env->mem_2, &b, &a, kind_b[p], mode);
                HhCheckPair(env->mem_2, &fresh, &cached, &a, &b);
                *env->mem_2 = record;
            }
            char id[96];
            snprintf(id, sizeof(id), "%s, %s, %s", (far) ? "far" : "near", pair_id[p], mode_id[mode]);
            HhStatsPrint(id, &fresh);
            snprintf(id, sizeof(id), "%s, %s, %s (cached)", (far) ? "far" : "near", pair_id[p], mode_id[mode]);
            HhStatsPrint(id, &cached);
            failures += HhFailures(&fresh) + HhFailures(&cached);
        }
    }

    g_dynamics_worker = prev;
    if (failures)
    {
        TEST_FAILURE;
    }
	return output;
}

static struct test_Output (*contact_tests[])(struct test_Environment *) =
{
    CapsuleContactCasesTest,
    CapsuleContactRandomTest,
    CapsuleContactThresholdTest,
    HullCapsuleContactCasesTest,
    HullCapsuleContactRandomTest,
    HullCapsuleContactSweepTest,
    HullCapsuleContactTimingTest,
    HullContactCasesTest,
    HullContactRandomTest,
};

struct suite_Correctness m_contact_suite =
{
	.id = "contact",
	.unit_test = contact_tests,
	.unit_test_count = sizeof(contact_tests) / sizeof(contact_tests[0]),
};

struct suite_Correctness *contact_correctness_suite = &m_contact_suite;

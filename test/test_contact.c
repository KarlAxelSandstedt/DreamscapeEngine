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
                stats->two_point_steep += 1;
                stats->steep_dist = F64MaxTest(stats->steep_dist, dist / L);
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
 * Tilt sweep across the parallel threshold (0.05 rad): the normal must stay continuous; the deepest depth
 * jumps by at most the two point manifold's loss (closest pair inside the shared stretch). Prints the
 * largest jumps relative to the capsule size.
 */
struct test_Output CapsuleContactThresholdTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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
            const f32 tilt = 0.048f + 0.0001f*(f32) k;
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

static struct test_Output (*contact_tests[])(struct test_Environment *) =
{
    CapsuleContactCasesTest,
    CapsuleContactRandomTest,
    CapsuleContactThresholdTest,
};

struct suite_Correctness m_contact_suite =
{
	.id = "contact",
	.unit_test = contact_tests,
	.unit_test_count = sizeof(contact_tests) / sizeof(contact_tests[0]),
};

struct suite_Correctness *contact_correctness_suite = &m_contact_suite;

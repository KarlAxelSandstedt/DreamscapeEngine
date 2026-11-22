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

#include "ds_test.h"
#include "ds_dynamics.h"
#include "ds_collision.h"
#include "ds_geometry.h"

/*
Time of impact tests
====================
ds_TimeOfImpactConvex on hand-made sweeps with known outcomes (max_toi = 1). A hit is checked by the core distance
(GJK, margins 0) at the returned toi: it lies in [target - tol, target + tol], with

    target  = max(slop, r0 + r1 - slop)
    tol     = slop / 4

Static shapes sit at the origin. Every case also runs with the args swapped (arg1 first), with the same
expected outcome. The args' vertex sets are built in place: a point or segment vertex set points into its own
storage, so an arg is never copied.

ToiRandomTest runs tables of random shape pairs and sweeps against an oracle, the core distance sampled at
TOI_TEST_SAMPLES times along the sweep (alternate sweeps call with the args swapped). It fails on NaN, on a hit
outside [target - tol, target + tol] and on a sampled contact (distance < target - tol) before the returned toi,
the lower bounds widened by GJK's f32 noise (64 eps L, L the scene's size). Missed contacts (max_toi returned
although a sample touches) are counted and printed, not failed: the TOI misses contacts that a rotation makes
and undoes within the sweep. Turns stay <= 2 rad per sweep: at 3 rad the TOI also steps over such contacts now
and then (a sample before the toi touches; 4 in 32000 sweeps).
*/

static struct ds_SolverConfig g_toi_test_solver_config;
static struct ds_NumericsConfig g_toi_test_numerics_config;

static void ToiTestConfigPush(void)
{
    g_toi_test_solver_config = ds_SolverConfigDefault();
    g_toi_test_numerics_config = ds_NumericsConfigDefault();
    ds_SolverConfigPush(&g_toi_test_solver_config);
    ds_NumericsConfigPush(&g_toi_test_numerics_config);
}

/* the core distance of the args at t, posed as ds_TimeOfImpactConvex poses them */
static f32 ToiTestDistance(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 t)
{
    ds_Transform t0 = ds_TransformLerp(arg0->sweep + 0, arg0->sweep + 1, t);
    t0.position = V3Sub(t0.position, QUnitRotate(t0.rotation, arg0->local_center));
    ds_Transform t1 = ds_TransformLerp(arg1->sweep + 0, arg1->sweep + 1, t);
    t1.position = V3Sub(t1.position, QUnitRotate(t1.rotation, arg1->local_center));
    const ds_Transform t_1_to_0 = ds_TransformRelative(&t0, &t1);

    v3 c0, c1, n;
    struct GJKCache cache;
    return GJK(&c0, &c1, &n, &cache, NULL, &arg0->vset, &arg1->vset, &t_1_to_0, F32_INFINITY);
}

/* toi is a hit in (0, max_toi): the core distance at it lies in [target - tol, target + tol] */
static u32 ToiTestHitCheck(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 toi, const f32 max_toi)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 target = F32Max(slop, arg0->radius + arg1->radius - slop);
    const f32 distance = ToiTestDistance(arg0, arg1, toi);
    const u32 hit = 0.0f < toi && toi < max_toi && target - tol <= distance && distance <= target + tol;
    if (!hit)
    {
        fprintf(stdout, "  toi %.9g, distance %.9g, target %.9g +- %.9g\n", (f64) toi, (f64) distance, (f64) target, (f64) tol);
    }
    return hit;
}

/* box3d's case in 3D: a segment moves head-on into a static box's face, distance 1 - 2t */
struct test_Output ToiBoxSegmentTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct dcel box = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f));
    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg0.vset, &box);

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(2.0f, 0.0f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetSegment(&arg1.vset, V3(0.0f, -1.0f, 0.0f), V3(0.0f, 1.0f, 0.0f));

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg1, &arg0, toi_swapped, 1.0f));

    return output;
}

/* the same sweep, cut off at max_toi = 0.4 before the hit (distance 0.2 there) */
struct test_Output ToiBeyondMaxToiTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct dcel box = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f));
    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg0.vset, &box);

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(2.0f, 0.0f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetSegment(&arg1.vset, V3(0.0f, -1.0f, 0.0f), V3(0.0f, 1.0f, 0.0f));

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 0.4f);
    TEST_EQUAL(0.4f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 0.4f);
    TEST_EQUAL(0.4f, toi_swapped);

    return output;
}

/* spheres of radius 0.5 head-on (points with radii): core distance 3 - 6t, target 1 - slop */
struct test_Output ToiSphereSphereTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.5f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetPoint(&arg0.vset, V3Zero());

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.5f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(3.0f, 0.0f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3(-3.0f, 0.0f, 0.0f) },
        .local_center = V3Zero(),
    };
    GJKVertexSetPoint(&arg1.vset, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg1, &arg0, toi_swapped, 1.0f));

    return output;
}

/* boxes overlapping at t = 0, moving apart: left to discrete collision detection */
struct test_Output ToiOverlapTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct dcel box = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f));
    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg0.vset, &box);

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(1.5f, 0.0f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3(5.0f, 0.0f, 0.0f) },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg1.vset, &box);

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_EQUAL(0.0f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_EQUAL(0.0f, toi_swapped);

    return output;
}

/* spheres moving apart: no hit */
struct test_Output ToiSeparatingTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.5f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetPoint(&arg0.vset, V3Zero());

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.5f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(3.0f, 0.0f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3(6.0f, 0.0f, 0.0f) },
        .local_center = V3Zero(),
    };
    GJKVertexSetPoint(&arg1.vset, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_EQUAL(1.0f, toi_swapped);

    return output;
}

/*
 * a point passes 0.5 above a static box's top face: the first plane (from the box's edge at x = -1) is crossed,
 * the next one (the top face) is not
 */
struct test_Output ToiGlancingMissTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct dcel box = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f));
    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg0.vset, &box);

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(-3.0f, 1.5f, 0.0f) },
        .sweep[1] = { .rotation = QIdentity(), .position = V3(3.0f, 1.5f, 0.0f) },
        .local_center = V3Zero(),
    };
    GJKVertexSetPoint(&arg1.vset, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_EQUAL(1.0f, toi_swapped);

    return output;
}

/*
 * a segment of half length 1.2, centered 2 above a static box (top face at y = 1), turns a quarter about z: its
 * lower end comes down onto the face at x ~ -0.67 (t ~ 0.62 under the sweep's nlerp)
 */
struct test_Output ToiSpinningSegmentTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct dcel box = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f));
    struct ds_TimeOfImpactArg arg0 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3Zero() },
        .sweep[1] = { .rotation = QIdentity(), .position = V3Zero() },
        .local_center = V3Zero(),
    };
    GJKVertexSetHull(&arg0.vset, &box);

    struct ds_TimeOfImpactArg arg1 =
    {
        .radius = 0.0f,
        .sweep[0] = { .rotation = QIdentity(), .position = V3(0.0f, 2.0f, 0.0f) },
        .sweep[1] = { .rotation = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.5f*F32_PI), .position = V3(0.0f, 2.0f, 0.0f) },
        .local_center = V3Zero(),
    };
    GJKVertexSetSegment(&arg1.vset, V3(-1.2f, 0.0f, 0.0f), V3(1.2f, 0.0f, 0.0f));

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg1, &arg0, toi_swapped, 1.0f));

    return output;
}

#define TOI_TEST_SAMPLES 256

enum toi_TestKind
{
    TOI_TEST_HULL,
    TOI_TEST_BOX,
    TOI_TEST_STICK,
    TOI_TEST_PLATE,
    TOI_TEST_CAPSULE,
    TOI_TEST_SPHERE,
    TOI_TEST_TRIANGLE,
};

/* the shapes' long axes (stick, capsule) are their local y */
struct toi_TestShape
{
    enum toi_TestKind   kind;
    struct dcel         hull;
    v3                  tri[3];
    f32                 half_height;    /* capsule */
    f32                 radius;         /* capsule, sphere */
    f32                 extent;         /* the farthest point from the shape's origin, radius included */
};

/*
 * size: the shapes' sizes are drawn from [size_min, size_max], shape 1's times ratio (0: 1). turn: both turn by up
 * to this angle over the sweep. Set 1 starts reach (in units of the extents' sum, drawn from [1.2, reach_max];
 * 0: 3) away and ends beyond set 0, offset sideways. offset: the scene's distance from the origin. center: the
 * center of mass offset, a fraction of the extent. max_toi 0: 1. parallel: set 1 comes down (along z) onto set 0,
 * both long axes horizontal, set 1 turning about z through set 0's direction (roll: sticks rolled by 45 degrees
 * about their long axes, edge to edge).
 */
struct toi_TestScenario
{
    const char *        id;
    enum toi_TestKind   kind0;
    enum toi_TestKind   kind1;
    f32                 size_min;
    f32                 size_max;
    f32                 ratio;
    f32                 turn;
    f32                 reach_max;
    f32                 offset;
    f32                 center;
    f32                 max_toi;
    u32                 parallel;
    u32                 roll;
};

struct toi_TestCount
{
    u64 sweeps;
    u64 overlap;        /* overlapping at t = 0: skipped */
    u64 nan;
    u64 hit;            /* distance at toi in [target - tol, target + tol] */
    u64 early;          /* above the band: an iteration limit stopped the TOI (safe) */
    u64 overshoot;      /* below the band: fails */
    u64 late;           /* a sample before toi touches: fails */
    u64 miss;           /* max_toi returned */
    u64 missed;         /* max_toi returned although a sample touches */
};

static v3 ToiTestUnit(void)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 theta = RngF32Range(0.0f, 2.0f*F32_PI);
    const f32 r = F32Sqrt(1.0f - z*z);
    return V3(r*F32Cos(theta), r*F32Sin(theta), z);
}

static q ToiTestRotation(const f32 max_angle)
{
    const v3 axis = ToiTestUnit();
    const f32 angle = RngF32Range(0.0f, max_angle);
    return QUnitAxisAngle(axis, angle);
}

static void ToiTestShapeInit(struct toi_TestShape *shape, struct arena *mem, const enum toi_TestKind kind, const f32 size)
{
    *shape = (struct toi_TestShape) { .kind = kind };
    switch (kind)
    {
        case TOI_TEST_HULL:
        {
            v3 p[24];
            do
            {
                const u32 count = (u32) RngU64Range(4, 24);
                const f32 sx = size*RngF32Range(0.2f, 1.0f);
                const f32 sy = size*RngF32Range(0.2f, 1.0f);
                const f32 sz = size*RngF32Range(0.2f, 1.0f);
                for (u32 i = 0; i < count; ++i)
                {
                    const v3 u = ToiTestUnit();
                    const f32 r = RngF32Range(0.5f, 1.0f);
                    p[i] = V3(r*u.x*sx, r*u.y*sy, r*u.z*sz);
                }
                shape->hull = DcelConvexHull(mem, p, count);
            } while (shape->hull.v_count < 4);
        } break;

        case TOI_TEST_BOX:
        {
            const f32 x = size*RngF32Range(0.1f, 1.0f);
            const f32 y = size*RngF32Range(0.1f, 1.0f);
            const f32 z = size*RngF32Range(0.1f, 1.0f);
            shape->hull = DcelBox(mem, V3(x, y, z));
        } break;

        case TOI_TEST_STICK:
        {
            const f32 l = size*RngF32Range(0.5f, 1.0f);
            const f32 w = size*RngF32Range(0.02f, 0.1f);
            shape->hull = DcelBox(mem, V3(w, l, w));
        } break;

        case TOI_TEST_PLATE:
        {
            const f32 a = size*RngF32Range(0.5f, 1.0f);
            const f32 t = size*RngF32Range(0.005f, 0.02f);
            shape->hull = DcelBox(mem, V3(a, t, a));
        } break;

        case TOI_TEST_CAPSULE:
        {
            shape->half_height = size*RngF32Range(0.3f, 1.0f);
            shape->radius = size*RngF32Range(0.05f, 0.3f);
        } break;

        case TOI_TEST_SPHERE:
        {
            shape->radius = size*RngF32Range(0.1f, 1.0f);
        } break;

        case TOI_TEST_TRIANGLE:
        {
            for (u32 i = 0; i < 3; ++i)
            {
                const v3 u = ToiTestUnit();
                const f32 r = size*RngF32Range(0.3f, 1.0f);
                shape->tri[i] = V3Scale(u, r);
            }
        } break;
    }

    shape->extent = shape->half_height + shape->radius;
    if (kind == TOI_TEST_TRIANGLE)
    {
        for (u32 i = 0; i < 3; ++i)
        {
            shape->extent = F32Max(shape->extent, V3Length(shape->tri[i]));
        }
    }
    else if (kind != TOI_TEST_CAPSULE && kind != TOI_TEST_SPHERE)
    {
        for (u32 i = 0; i < shape->hull.v_count; ++i)
        {
            shape->extent = F32Max(shape->extent, V3Length(shape->hull.v[i]));
        }
    }
}

/* the vertex set is built in place: a capsule's or sphere's points live in the arg, a triangle's in the shape */
static void ToiTestArgInit(struct ds_TimeOfImpactArg *arg, const struct toi_TestShape *shape)
{
    arg->radius = shape->radius;
    if (shape->kind == TOI_TEST_CAPSULE)
    {
        GJKVertexSetSegment(&arg->vset, V3(0.0f, shape->half_height, 0.0f), V3(0.0f, -shape->half_height, 0.0f));
    }
    else if (shape->kind == TOI_TEST_SPHERE)
    {
        GJKVertexSetPoint(&arg->vset, V3Zero());
    }
    else if (shape->kind == TOI_TEST_TRIANGLE)
    {
        GJKVertexSetTriangle(&arg->vset, shape->tri);
    }
    else
    {
        GJKVertexSetHull(&arg->vset, &shape->hull);
    }
}

/* the sweeps of one random case; returns the scene's size L for the noise bound */
static f32 ToiTestSweeps(struct ds_TimeOfImpactArg *arg0, struct ds_TimeOfImpactArg *arg1, const struct toi_TestScenario *scenario, const struct toi_TestShape *shape0, const struct toi_TestShape *shape1)
{
    const f32 ext = shape0->extent + shape1->extent;
    const v3 offset_dir = ToiTestUnit();
    const v3 origin = V3Scale(offset_dir, scenario->offset);
    f32 scene = 0.0f;
    if (scenario->parallel)
    {
        const f32 phi = RngF32Range(0.0f, 2.0f*F32_PI);
        const f32 delta = RngF32Range(-0.6f, 0.6f);
        const q tilt = ToiTestRotation(0.1f);
        const f32 h = ext*RngF32Range(0.3f, 1.0f);
        const f32 dx = RngF32Range(-0.3f, 0.3f);
        const f32 dy = RngF32Range(-0.3f, 0.3f);

        /* the long axes (local y) stay horizontal; rolled about them for edge to edge */
        q lay = QIdentity();
        if (scenario->roll)
        {
            lay = QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), 0.25f*F32_PI);
        }

        const q q0 = QMul(QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), phi), lay);
        const q q1 = QMul(QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), phi + delta), lay);
        const q dq1 = QMul(tilt, QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -2.0f*delta));
        const v3 above = V3Add(origin, V3(dx, dy, h));
        const v3 below = V3Add(origin, V3(dx, dy, -h));
        arg0->sweep[0] = (ds_Transform) { .rotation = q0, .position = origin };
        arg0->sweep[1] = (ds_Transform) { .rotation = q0, .position = origin };
        arg1->sweep[0] = (ds_Transform) { .rotation = q1, .position = above };
        arg1->sweep[1] = (ds_Transform) { .rotation = QMul(dq1, q1), .position = below };
        scene = h + ext;
    }
    else
    {
        const q q0 = ToiTestRotation(2.0f*F32_PI);
        const q dq0 = ToiTestRotation(scenario->turn);
        const q q1 = ToiTestRotation(2.0f*F32_PI);
        const q dq1 = ToiTestRotation(scenario->turn);
        const v3 u = ToiTestUnit();
        const f32 reach_max = (scenario->reach_max > 0.0f) ? scenario->reach_max : 3.0f;
        const f32 reach = ext*RngF32Range(1.2f, reach_max);
        const v3 side_dir = ToiTestUnit();
        const f32 side = ext*RngF32Range(0.0f, 0.8f);
        const v3 drift_dir = ToiTestUnit();
        const v3 start1 = V3Add(origin, V3Scale(u, reach));
        const v3 end1 = V3Add(V3Add(origin, V3Scale(u, -0.5f*reach)), V3Scale(side_dir, side));
        arg0->sweep[0] = (ds_Transform) { .rotation = q0, .position = origin };
        arg0->sweep[1] = (ds_Transform) { .rotation = QMul(dq0, q0), .position = V3Add(origin, V3Scale(drift_dir, 0.1f*shape0->extent)) };
        arg1->sweep[0] = (ds_Transform) { .rotation = q1, .position = start1 };
        arg1->sweep[1] = (ds_Transform) { .rotation = QMul(dq1, q1), .position = end1 };
        scene = reach + side + ext;
    }

    arg0->local_center = V3Zero();
    arg1->local_center = V3Zero();
    if (scenario->center > 0.0f)
    {
        const v3 center0 = ToiTestUnit();
        const v3 center1 = ToiTestUnit();
        arg0->local_center = V3Scale(center0, scenario->center*shape0->extent);
        arg1->local_center = V3Scale(center1, scenario->center*shape1->extent);
    }

    return scenario->offset + scene;
}

static void ToiTestScenarioRun(struct toi_TestCount *count, struct arena *mem, const struct toi_TestScenario *scenario, const u32 sweeps)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 max_toi = (scenario->max_toi > 0.0f) ? scenario->max_toi : 1.0f;
    const f32 ratio = (scenario->ratio > 0.0f) ? scenario->ratio : 1.0f;
    for (u32 i = 0; i < sweeps; ++i)
    {
        const struct arena record = *mem;
        struct toi_TestShape shape0, shape1;
        const f32 size0 = RngF32Range(scenario->size_min, scenario->size_max);
        ToiTestShapeInit(&shape0, mem, scenario->kind0, size0);
        const f32 size1 = ratio*RngF32Range(scenario->size_min, scenario->size_max);
        ToiTestShapeInit(&shape1, mem, scenario->kind1, size1);

        struct ds_TimeOfImpactArg arg0, arg1;
        ToiTestArgInit(&arg0, &shape0);
        ToiTestArgInit(&arg1, &shape1);
        const f32 scene = ToiTestSweeps(&arg0, &arg1, scenario, &shape0, &shape1);
        const f32 noise = 64.0f*F32_EPSILON*scene;
        const f32 target = F32Max(slop, arg0.radius + arg1.radius - slop);

        count->sweeps += 1;
        if (ToiTestDistance(&arg0, &arg1, 0.0f) <= target + tol)
        {
            count->overlap += 1;
            *mem = record;
            continue;
        }

        const struct ds_TimeOfImpactArg *first = (i & 1) ? &arg1 : &arg0;
        const struct ds_TimeOfImpactArg *second = (i & 1) ? &arg0 : &arg1;
        const f32 toi = ds_TimeOfImpactConvex(first, second, max_toi);
        if (toi != toi)
        {
            count->nan += 1;
            *mem = record;
            continue;
        }

        const f32 t_end = F32Min(toi, max_toi);
        f32 sampled = F32_INFINITY;
        for (u32 k = 0; k < TOI_TEST_SAMPLES; ++k)
        {
            const f32 t = t_end*(f32) k / (f32) TOI_TEST_SAMPLES;
            sampled = F32Min(sampled, ToiTestDistance(&arg0, &arg1, t));
        }

        if (toi == max_toi)
        {
            count->miss += 1;
            if (sampled < target - tol - noise)
            {
                count->missed += 1;
            }
        }
        else
        {
            const f32 distance = ToiTestDistance(&arg0, &arg1, toi);
            if (distance < target - tol - noise)
            {
                count->overshoot += 1;
                fprintf(stdout, "  %s, sweep %u: toi %.9g, distance %.9g below the band [%.9g, %.9g]\n", scenario->id, i, (f64) toi, (f64) distance, (f64) (target - tol), (f64) (target + tol));
            }
            else if (distance > target + tol)
            {
                count->early += 1;
            }
            else
            {
                count->hit += 1;
            }

            if (sampled < target - tol - noise)
            {
                count->late += 1;
                fprintf(stdout, "  %s, sweep %u: toi %.9g, a sample before it touches (%.9g)\n", scenario->id, i, (f64) toi, (f64) sampled);
            }
        }

        *mem = record;
    }
}

static const struct toi_TestScenario g_toi_test_scenario[] =
{
    { .id = "hull-hull, no turn",               .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 0.0f },
    { .id = "hull-hull, turn 0.3",              .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "hull-hull, turn 1.5",              .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.5f },
    { .id = "hull-hull, turn 2.0",              .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 2.0f },
    { .id = "box-box, turn 1.0",                .kind0 = TOI_TEST_BOX,      .kind1 = TOI_TEST_BOX,      .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "stick-stick, turn 0.3",            .kind0 = TOI_TEST_STICK,    .kind1 = TOI_TEST_STICK,    .size_min = 0.5f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "stick-stick, turn 2.0",            .kind0 = TOI_TEST_STICK,    .kind1 = TOI_TEST_STICK,    .size_min = 0.5f,   .size_max = 2.0f, .turn = 2.0f },
    { .id = "capsule-capsule, turn 0.3",        .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "capsule-capsule, turn 2.0",        .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .turn = 2.0f },
    { .id = "capsule-hull, turn 1.0",           .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "sphere-hull, turn 1.0",            .kind0 = TOI_TEST_SPHERE,   .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "sphere-sphere, turn 1.0",          .kind0 = TOI_TEST_SPHERE,   .kind1 = TOI_TEST_SPHERE,   .size_min = 0.1f,   .size_max = 1.0f, .turn = 1.0f },
    { .id = "triangle-capsule, turn 0.5",       .kind0 = TOI_TEST_TRIANGLE, .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .turn = 0.5f },
    { .id = "triangle-hull, turn 0.5",          .kind0 = TOI_TEST_TRIANGLE, .kind1 = TOI_TEST_HULL,     .size_min = 0.3f,   .size_max = 2.0f, .turn = 0.5f },
    { .id = "plate-box, turn 2.0",              .kind0 = TOI_TEST_PLATE,    .kind1 = TOI_TEST_BOX,      .size_min = 0.5f,   .size_max = 2.0f, .turn = 2.0f },
    { .id = "plate-sphere, fast, turn 0.3",     .kind0 = TOI_TEST_PLATE,    .kind1 = TOI_TEST_SPHERE,   .size_min = 0.5f,   .size_max = 2.0f, .ratio = 0.1f, .turn = 0.3f, .reach_max = 50.0f },
    { .id = "small hull-large box, turn 0.3",   .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_BOX,      .size_min = 0.02f,  .size_max = 0.1f, .ratio = 50.0f, .turn = 0.3f },
    { .id = "small hulls, turn 0.3",            .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.02f,  .size_max = 0.2f, .turn = 0.3f },
    { .id = "large hulls, turn 0.3",            .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 5.0f,   .size_max = 20.0f, .turn = 0.3f },
    { .id = "hull-hull at 100 m, turn 0.5",     .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 0.5f, .offset = 100.0f },
    { .id = "hull-hull at 1000 m, turn 0.5",    .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 0.5f, .offset = 1000.0f },
    { .id = "hull-hull, centers off, turn 1.0", .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f, .center = 0.5f },
    { .id = "hull-hull, max_toi 0.5, turn 0.5", .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 0.5f, .max_toi = 0.5f },
    { .id = "capsules passing parallel",        .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .parallel = 1 },
    { .id = "sticks passing parallel, rolled",  .kind0 = TOI_TEST_STICK,    .kind1 = TOI_TEST_STICK,    .size_min = 0.5f,   .size_max = 2.0f, .parallel = 1, .roll = 1 },
};

#define TOI_TEST_SWEEPS 200

struct test_Output ToiRandomTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const u32 scenario_count = sizeof(g_toi_test_scenario) / sizeof(g_toi_test_scenario[0]);
    struct toi_TestCount total = { 0 };
    for (u32 i = 0; i < scenario_count; ++i)
    {
        struct toi_TestCount count = { 0 };
        ToiTestScenarioRun(&count, env->mem_1, g_toi_test_scenario + i, TOI_TEST_SWEEPS);
        fprintf(stdout, "  %-34s sweeps %llu overlap %llu nan %llu hit %llu early %llu overshoot %llu late %llu miss %llu (missed contacts %llu)\n",
            g_toi_test_scenario[i].id, (unsigned long long) count.sweeps, (unsigned long long) count.overlap, (unsigned long long) count.nan,
            (unsigned long long) count.hit, (unsigned long long) count.early, (unsigned long long) count.overshoot, (unsigned long long) count.late,
            (unsigned long long) count.miss, (unsigned long long) count.missed);
        total.nan += count.nan;
        total.overshoot += count.overshoot;
        total.late += count.late;
    }

    TEST_ZERO(total.nan);
    TEST_ZERO(total.overshoot);
    TEST_ZERO(total.late);
    return output;
}

static struct test_Output (*toi_tests[])(struct test_Environment *) =
{
    ToiBoxSegmentTest,
    ToiBeyondMaxToiTest,
    ToiSphereSphereTest,
    ToiOverlapTest,
    ToiSeparatingTest,
    ToiGlancingMissTest,
    ToiSpinningSegmentTest,
    ToiRandomTest,
};

struct suite_Correctness m_toi_suite =
{
    .id = "toi",
    .unit_test = toi_tests,
    .unit_test_count = sizeof(toi_tests) / sizeof(toi_tests[0]),
};

struct suite_Correctness *toi_correctness_suite = &m_toi_suite;

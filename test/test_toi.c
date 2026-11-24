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
expected outcome. An arg's vertex set is built in place (ds_TimeOfImpactArg*): a sphere's or capsule's points
live in the vertex set itself, so an arg is never copied.

ToiRandomTest runs tables of random shape pairs and sweeps against an oracle, the core distance sampled at
TOI_TEST_SAMPLES times along the sweep (alternate sweeps call with the args swapped). It fails on NaN, on a hit
outside [target - tol, target + tol] and on a sampled contact (distance < target - tol) before the returned toi,
the lower bounds widened by GJK's f32 noise (64 eps L, L the scene's size). Missed contacts (max_toi returned
although a sample touches) are counted and printed, not failed: the TOI misses contacts that a rotation makes
and undoes within the sweep. Turns stay <= 1 rad per sweep (about twice the solver's default spin limit, 0.52 rad
per tick at 60 Hz): at 1.5 to 2 rad the TOI also steps over such contacts now and then (a sample before the toi
touches; about 1 in 20000 sweeps), at 3 rad more often (4 in 32000).

The ToiMesh* tests run ds_TimeOfImpactConcave: a static mesh (a 4 x 4 m floor at y = 0 facing +y, optionally walls
at x = 4 and z = 4 facing inwards; or a random terrain grid) against a convex body (shape = body, its center of
mass the shape's). The oracle replays Concave's early-outs, computed as Concave computes them: a triangle the
shape's COM starts behind, or ends more than safe_advancement in front of having advanced less than that, is not
the TOI's business. The other triangles ("candidates") are checked as above: at a hit their smallest core distance
lies in [target - tol, target + tol]; after a fallback the COM sphere's distance lies in its own band (target
fallback_radius). ToiMeshRandomTest also limits the candidates to Concave's MeshQuery (the union of the shape's
start and end boxes): a triangle outside it that the shape touches before the toi (a pointy shape turning fast,
Concave's documented limitation; 1 in 64000 sweeps of sticks turning up to 1 rad) is counted as "outside the
path box", not failed. Missed contacts are counted as in ToiRandomTest.
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
    const q rotation_shape_world0 = QNLerp(arg0->rotation_shape_world[0], arg0->rotation_shape_world[1], t);
    const q rotation_shape_world1 = QNLerp(arg1->rotation_shape_world[0], arg1->rotation_shape_world[1], t);
    const v3 com_anchor0 = V3Interpolate(arg0->com_anchor[1], arg0->com_anchor[0], t);
    const v3 com_anchor1 = V3Interpolate(arg1->com_anchor[1], arg1->com_anchor[0], t);
    const v3 origin_to_com0 = QUnitRotate(rotation_shape_world0, arg0->com_shape);
    const v3 origin_to_com1 = QUnitRotate(rotation_shape_world1, arg1->com_shape);
    const ds_Transform t0 = { .rotation = rotation_shape_world0, .position = V3Sub(com_anchor0, origin_to_com0) };
    const ds_Transform t1 = { .rotation = rotation_shape_world1, .position = V3Sub(com_anchor1, origin_to_com1) };
    const ds_Transform t_1_to_0 = ds_TransformRelative(&t0, &t1);

    v3 c0, c1, n;
    struct GJKCache cache;
    return GJK(&c0, &c1, &n, &cache, NULL, &arg0->vset, &arg1->vset, &t_1_to_0, F32_INFINITY);
}


/* the arg of a sphere, capsule or hull moved by transform[0] -> transform[1] (shape = body: t_local identity, anchor 0) */
static void ToiTestArgInit(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform transform[2], const v3 com_body)
{
    const ds_Transform t_local = ds_TransformIdentity();
    const ds_TransformWorld t_body_world0 = ds_TransformPromote(transform + 0);
    const ds_TransformWorld t_body_world1 = ds_TransformPromote(transform + 1);
    const v3f64 anchor = V3F64(0.0, 0.0, 0.0);
    if (shape->type == C_SHAPE_SPHERE)
    {
        ds_TimeOfImpactArgSphere(arg, shape, &t_local, &t_body_world0, &t_body_world1, com_body, anchor);
    }
    else if (shape->type == C_SHAPE_CAPSULE)
    {
        ds_TimeOfImpactArgCapsule(arg, shape, &t_local, &t_body_world0, &t_body_world1, com_body, anchor);
    }
    else
    {
        ds_TimeOfImpactArgHull(arg, shape, &t_local, &t_body_world0, &t_body_world1, com_body, anchor);
    }
}

/* toi is a hit in (0, max_toi): the core distance at it lies in [target - tol, target + tol] */
static u32 ToiTestHitCheck(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 toi, const f32 max_toi)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 target = F32Max(slop, arg0->margin + arg1->margin - slop);
    const f32 distance = ToiTestDistance(arg0, arg1, toi);
    const u32 hit = 0.0f < toi && toi < max_toi && target - tol <= distance && distance <= target + tol;
    if (!hit)
    {
        fprintf(stdout, "  toi %.9g, distance %.9g, target %.9g +- %.9g\n", (f64) toi, (f64) distance, (f64) target, (f64) tol);
    }
    return hit;
}

/* box3d's case in 3D: a capsule (radius 0.05) moves head-on into a static box's face, core distance 1 - 2t */
struct test_Output ToiBoxSegmentTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 1.0f, .radius = 0.05f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &box, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 0.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &capsule, transform1, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg1, &arg0, toi_swapped, 1.0f));

    return output;
}

/* the same sweep, cut off at max_toi = 0.4 before the hit (core distance 0.2 there) */
struct test_Output ToiBeyondMaxToiTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 1.0f, .radius = 0.05f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &box, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 0.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &capsule, transform1, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 0.4f);
    TEST_EQUAL(0.4f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 0.4f);
    TEST_EQUAL(0.4f, toi_swapped);

    return output;
}

/* spheres of radius 0.5 head-on: core distance 3 - 6t, target 1 - slop */
struct test_Output ToiSphereSphereTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.5f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &sphere, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(3.0f, 0.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(-3.0f, 0.0f, 0.0f) },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &sphere, transform1, V3Zero());

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

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &box, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(1.5f, 0.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(5.0f, 0.0f, 0.0f) },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &box, transform1, V3Zero());

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

    const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.5f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &sphere, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(3.0f, 0.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(6.0f, 0.0f, 0.0f) },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &sphere, transform1, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_EQUAL(1.0f, toi_swapped);

    return output;
}

/*
 * a sphere (radius 0.1) passes 0.4 above a static box's top face: the first plane (from the box's edge at x = -1)
 * is crossed, the next one (the top face) is not
 */
struct test_Output ToiGlancingMissTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.1f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &box, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QIdentity(), .position = V3(-3.0f, 1.5f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(3.0f, 1.5f, 0.0f) },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &sphere, transform1, V3Zero());

    const f32 toi = ds_TimeOfImpactConvex(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

    const f32 toi_swapped = ds_TimeOfImpactConvex(&arg1, &arg0, 1.0f);
    TEST_EQUAL(1.0f, toi_swapped);

    return output;
}

/*
 * a capsule (half height 1.2, radius 0.05) centered 2 above a static box (top face at y = 1), lying along x, turns
 * a quarter about z until it stands: its lower end comes down onto the face at x ~ -0.73
 */
struct test_Output ToiSpinningSegmentTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(1.0f, 1.0f, 1.0f)) };
    const struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 1.2f, .radius = 0.05f } };
    const ds_Transform transform0[2] =
    {
        { .rotation = QIdentity(), .position = V3Zero() },
        { .rotation = QIdentity(), .position = V3Zero() },
    };
    struct ds_TimeOfImpactArg arg0;
    ToiTestArgInit(&arg0, &box, transform0, V3Zero());

    const ds_Transform transform1[2] =
    {
        { .rotation = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -0.5f*F32_PI), .position = V3(0.0f, 2.0f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(0.0f, 2.0f, 0.0f) },
    };
    struct ds_TimeOfImpactArg arg1;
    ToiTestArgInit(&arg1, &capsule, transform1, V3Zero());

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
};

/* the shapes' long axes (stick, capsule) are their local y */
struct toi_TestShape
{
    struct c_Shape  shape;
    f32             extent;     /* the farthest point from the shape's origin, radius included */
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
    *shape = (struct toi_TestShape) { .shape.type = C_SHAPE_CONVEX_HULL };
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
                shape->shape.hull = DcelConvexHull(mem, p, count);
            } while (shape->shape.hull.v_count < 4);
        } break;

        case TOI_TEST_BOX:
        {
            const f32 x = size*RngF32Range(0.1f, 1.0f);
            const f32 y = size*RngF32Range(0.1f, 1.0f);
            const f32 z = size*RngF32Range(0.1f, 1.0f);
            shape->shape.hull = DcelBox(mem, V3(x, y, z));
        } break;

        case TOI_TEST_STICK:
        {
            const f32 l = size*RngF32Range(0.5f, 1.0f);
            const f32 w = size*RngF32Range(0.02f, 0.1f);
            shape->shape.hull = DcelBox(mem, V3(w, l, w));
        } break;

        case TOI_TEST_PLATE:
        {
            const f32 a = size*RngF32Range(0.5f, 1.0f);
            const f32 t = size*RngF32Range(0.005f, 0.02f);
            shape->shape.hull = DcelBox(mem, V3(a, t, a));
        } break;

        case TOI_TEST_CAPSULE:
        {
            shape->shape.type = C_SHAPE_CAPSULE;
            shape->shape.capsule.half_height = size*RngF32Range(0.3f, 1.0f);
            shape->shape.capsule.radius = size*RngF32Range(0.05f, 0.3f);
        } break;

        case TOI_TEST_SPHERE:
        {
            shape->shape.type = C_SHAPE_SPHERE;
            shape->shape.sphere.radius = size*RngF32Range(0.1f, 1.0f);
        } break;
    }

    if (shape->shape.type == C_SHAPE_CAPSULE)
    {
        shape->extent = shape->shape.capsule.half_height + shape->shape.capsule.radius;
    }
    else if (shape->shape.type == C_SHAPE_SPHERE)
    {
        shape->extent = shape->shape.sphere.radius;
    }
    else
    {
        for (u32 i = 0; i < shape->shape.hull.v_count; ++i)
        {
            shape->extent = F32Max(shape->extent, V3Length(shape->shape.hull.v[i]));
        }
    }
}

/* the start and end transforms (and centers of mass) of one random case; returns the scene's size L for the noise bound */
static f32 ToiTestSweeps(ds_Transform transform0[2], v3 *com_body0, ds_Transform transform1[2], v3 *com_body1, const struct toi_TestScenario *scenario, const struct toi_TestShape *shape0, const struct toi_TestShape *shape1)
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
        transform0[0] = (ds_Transform) { .rotation = q0, .position = origin };
        transform0[1] = (ds_Transform) { .rotation = q0, .position = origin };
        transform1[0] = (ds_Transform) { .rotation = q1, .position = above };
        transform1[1] = (ds_Transform) { .rotation = QMul(dq1, q1), .position = below };
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
        transform0[0] = (ds_Transform) { .rotation = q0, .position = origin };
        transform0[1] = (ds_Transform) { .rotation = QMul(dq0, q0), .position = V3Add(origin, V3Scale(drift_dir, 0.1f*shape0->extent)) };
        transform1[0] = (ds_Transform) { .rotation = q1, .position = start1 };
        transform1[1] = (ds_Transform) { .rotation = QMul(dq1, q1), .position = end1 };
        scene = reach + side + ext;
    }

    *com_body0 = V3Zero();
    *com_body1 = V3Zero();
    if (scenario->center > 0.0f)
    {
        const v3 dir0 = ToiTestUnit();
        const v3 dir1 = ToiTestUnit();
        *com_body0 = V3Scale(dir0, scenario->center*shape0->extent);
        *com_body1 = V3Scale(dir1, scenario->center*shape1->extent);
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

        ds_Transform transform0[2], transform1[2];
        v3 com_body0, com_body1;
        const f32 scene = ToiTestSweeps(transform0, &com_body0, transform1, &com_body1, scenario, &shape0, &shape1);
        struct ds_TimeOfImpactArg arg0, arg1;
        ToiTestArgInit(&arg0, &shape0.shape, transform0, com_body0);
        ToiTestArgInit(&arg1, &shape1.shape, transform1, com_body1);
        const f32 noise = 64.0f*F32_EPSILON*scene;
        const f32 target = F32Max(slop, arg0.margin + arg1.margin - slop);

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
    { .id = "hull-hull, turn 1.0",              .kind0 = TOI_TEST_HULL,     .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "box-box, turn 1.0",                .kind0 = TOI_TEST_BOX,      .kind1 = TOI_TEST_BOX,      .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "stick-stick, turn 0.3",            .kind0 = TOI_TEST_STICK,    .kind1 = TOI_TEST_STICK,    .size_min = 0.5f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "stick-stick, turn 1.0",            .kind0 = TOI_TEST_STICK,    .kind1 = TOI_TEST_STICK,    .size_min = 0.5f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "capsule-capsule, turn 0.3",        .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "capsule-capsule, turn 1.0",        .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_CAPSULE,  .size_min = 0.3f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "capsule-hull, turn 1.0",           .kind0 = TOI_TEST_CAPSULE,  .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "sphere-hull, turn 1.0",            .kind0 = TOI_TEST_SPHERE,   .kind1 = TOI_TEST_HULL,     .size_min = 0.2f,   .size_max = 2.0f, .turn = 1.0f },
    { .id = "sphere-sphere, turn 1.0",          .kind0 = TOI_TEST_SPHERE,   .kind1 = TOI_TEST_SPHERE,   .size_min = 0.1f,   .size_max = 1.0f, .turn = 1.0f },
    { .id = "plate-box, turn 1.0",              .kind0 = TOI_TEST_PLATE,    .kind1 = TOI_TEST_BOX,      .size_min = 0.5f,   .size_max = 2.0f, .turn = 1.0f },
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

/* the arg of a sphere, capsule or hull at t_local in its body, the body moved by t_body_world[0] -> t_body_world[1] */
static void ToiTestArgInitWorld(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld t_body_world[2], const v3 com_body, const v3f64 anchor)
{
    if (shape->type == C_SHAPE_SPHERE)
    {
        ds_TimeOfImpactArgSphere(arg, shape, t_local, t_body_world + 0, t_body_world + 1, com_body, anchor);
    }
    else if (shape->type == C_SHAPE_CAPSULE)
    {
        ds_TimeOfImpactArgCapsule(arg, shape, t_local, t_body_world + 0, t_body_world + 1, com_body, anchor);
    }
    else
    {
        ds_TimeOfImpactArgHull(arg, shape, t_local, t_body_world + 0, t_body_world + 1, com_body, anchor);
    }
}

#define TOI_TEST_ROOM_WALL_X    1u      /* the wall x = 4 (y, z in [0, 4]), facing -x */
#define TOI_TEST_ROOM_WALL_Z    2u      /* the wall z = 4 (x, y in [0, 4]), facing -z */

/* the floor y = 0 over x, z in [0, 4] (16 cells of 2 triangles, facing +y), plus the walls in the mask */
static void ToiTestRoom(struct c_Shape *shape, struct arena *mem, const u32 walls)
{
    v3 quad[4*18];
    u32 quad_count = 0;
    for (u32 i = 0; i < 4; ++i)
    {
        for (u32 j = 0; j < 4; ++j)
        {
            const f32 x0 = (f32) i;
            const f32 x1 = (f32) (i + 1);
            const f32 z0 = (f32) j;
            const f32 z1 = (f32) (j + 1);
            quad[4*quad_count + 0] = V3(x0, 0.0f, z0);
            quad[4*quad_count + 1] = V3(x0, 0.0f, z1);
            quad[4*quad_count + 2] = V3(x1, 0.0f, z1);
            quad[4*quad_count + 3] = V3(x1, 0.0f, z0);
            quad_count += 1;
        }
    }

    if (walls & TOI_TEST_ROOM_WALL_X)
    {
        quad[4*quad_count + 0] = V3(4.0f, 0.0f, 0.0f);
        quad[4*quad_count + 1] = V3(4.0f, 0.0f, 4.0f);
        quad[4*quad_count + 2] = V3(4.0f, 4.0f, 4.0f);
        quad[4*quad_count + 3] = V3(4.0f, 4.0f, 0.0f);
        quad_count += 1;
    }

    if (walls & TOI_TEST_ROOM_WALL_Z)
    {
        quad[4*quad_count + 0] = V3(0.0f, 0.0f, 4.0f);
        quad[4*quad_count + 1] = V3(0.0f, 4.0f, 4.0f);
        quad[4*quad_count + 2] = V3(4.0f, 4.0f, 4.0f);
        quad[4*quad_count + 3] = V3(4.0f, 0.0f, 4.0f);
        quad_count += 1;
    }

    /* each quad is CCW about its front normal: triangles 0 1 2 and 0 2 3 */
    *shape = (struct c_Shape) { .type = C_SHAPE_TRI_MESH };
    struct mesh *mesh = &shape->mesh;
    mesh->v_count = 4*quad_count;
    mesh->tri_count = 2*quad_count;
    mesh->v = ArenaPush(mem, mesh->v_count*sizeof(v3));
    mesh->tri = ArenaPush(mem, mesh->tri_count*sizeof(v3u32));
    for (u32 i = 0; i < mesh->v_count; ++i)
    {
        mesh->v[i] = quad[i];
    }

    for (u32 i = 0; i < quad_count; ++i)
    {
        mesh->tri[2*i + 0] = (v3u32) { .buf = { 4*i + 0, 4*i + 1, 4*i + 2 } };
        mesh->tri[2*i + 1] = (v3u32) { .buf = { 4*i + 0, 4*i + 2, 4*i + 3 } };
    }

    MeshConstruct(mem, mesh, 8);
}

#define TOI_TEST_TERRAIN_CELLS 6

/* a grid of TOI_TEST_TERRAIN_CELLS^2 unit cells over x, z in [0, cells], heights drawn from [0, height_max], facing +y */
static void ToiTestTerrain(struct c_Shape *shape, struct arena *mem, const f32 height_max)
{
    const u32 n = TOI_TEST_TERRAIN_CELLS;
    const u32 side = n + 1;
    *shape = (struct c_Shape) { .type = C_SHAPE_TRI_MESH };
    struct mesh *mesh = &shape->mesh;
    mesh->v_count = side*side;
    mesh->tri_count = 2*n*n;
    mesh->v = ArenaPush(mem, mesh->v_count*sizeof(v3));
    mesh->tri = ArenaPush(mem, mesh->tri_count*sizeof(v3u32));
    for (u32 i = 0; i < side; ++i)
    {
        for (u32 j = 0; j < side; ++j)
        {
            f32 h = 0.0f;
            if (height_max > 0.0f)
            {
                h = RngF32Range(0.0f, height_max);
            }

            mesh->v[i*side + j] = V3((f32) i, h, (f32) j);
        }
    }

    u32 t = 0;
    for (u32 i = 0; i < n; ++i)
    {
        for (u32 j = 0; j < n; ++j)
        {
            const u32 a = i*side + j;
            const u32 b = (i + 1)*side + j;
            const u32 c = i*side + j + 1;
            const u32 d = (i + 1)*side + j + 1;
            mesh->tri[t + 0] = (v3u32) { .buf = { a, c, b } };
            mesh->tri[t + 1] = (v3u32) { .buf = { b, c, d } };
            t += 2;
        }
    }

    MeshConstruct(mem, mesh, 8);
}

/*
 * The triangles of the mesh arg that ds_TimeOfImpactConcave does not skip (its early-outs, computed as it computes
 * them) and that lie within reach of the convex shape's COM path (reach: the shape's farthest point from its COM;
 * F32_INFINITY: all); path_box_only: also only those of its MeshQuery (the union of the shape's start and end boxes).
 * Allocated on mem; count set.
 */
static u32 *ToiTestMeshCandidates(struct arena *mem, u32 *count, const struct ds_TimeOfImpactArg *mesh, const struct ds_TimeOfImpactArg *convex, const f32 reach, const u32 path_box_only)
{
    const struct mesh *m = &mesh->shape->mesh;
    const f32 slop = g_solver_config->linear_slop;

    const ds_Transform mesh_to_anchor = { .rotation = mesh->rotation_shape_world[0], .position = mesh->com_anchor[0] };
    const v3 shape_origin_anchor_world_diff0 = V3Sub(convex->com_anchor[0], QUnitRotate(convex->rotation_shape_world[0], convex->com_shape));
    const v3 shape_origin_anchor_world_diff1 = V3Sub(convex->com_anchor[1], QUnitRotate(convex->rotation_shape_world[1], convex->com_shape));
    const ds_Transform shape_to_anchor0 = { .rotation = convex->rotation_shape_world[0], .position = shape_origin_anchor_world_diff0 };
    const ds_Transform shape_to_anchor1 = { .rotation = convex->rotation_shape_world[1], .position = shape_origin_anchor_world_diff1 };
    const ds_Transform shape_to_mesh0 = ds_TransformRelative(&mesh_to_anchor, &shape_to_anchor0);
    const ds_Transform shape_to_mesh1 = ds_TransformRelative(&mesh_to_anchor, &shape_to_anchor1);
    const f32 safe_advancement = F32Max(g_solver_config->ccd_safe_advancement_factor*convex->shape->com_inscribed_sphere_radius, slop);
    const v3 shape_com_mesh0 = V3Transform(convex->shape->center_of_mass, &shape_to_mesh0);
    const v3 shape_com_mesh1 = V3Transform(convex->shape->center_of_mass, &shape_to_mesh1);

    const f32 grow = reach + slop;
    const struct aabb reach_box =
    {
        .min = V3Sub(V3Min(shape_com_mesh0, shape_com_mesh1), V3(grow, grow, grow)),
        .max = V3Add(V3Max(shape_com_mesh0, shape_com_mesh1), V3(grow, grow, grow)),
    };

    u32 *queried = ArenaPush(mem, m->tri_count*sizeof(u32));
    for (u32 i = 0; i < m->tri_count; ++i)
    {
        queried[i] = !path_box_only;
    }

    if (path_box_only)
    {
        const struct aabb bbox0 = c_ShapeBbox(convex->shape, &shape_to_mesh0);
        const struct aabb bbox1 = c_ShapeBbox(convex->shape, &shape_to_mesh1);
        const struct aabb path_box = BboxUnion(bbox0, bbox1);
        const ds_Array(u32) query = MeshQuery(mem, m, &path_box);
        for (u32 i = 0; i < query.length; ++i)
        {
            queried[query.buf[i]] = 1;
        }
    }

    u32 *tri_index = ArenaPush(mem, m->tri_count*sizeof(u32));
    *count = 0;
    for (u32 i = 0; i < m->tri_count; ++i)
    {
        const u32 *v = m->tri[i].buf;
        const v3 tri[3] = { m->v[v[0]], m->v[v[1]], m->v[v[2]] };
        const v3 n = V3Normalize(V3Cross(V3Sub(tri[1], tri[0]), V3Sub(tri[2], tri[0])));
        const f32 offset0 = V3Dot(n, V3Sub(shape_com_mesh0, tri[0]));
        const f32 offset1 = V3Dot(n, V3Sub(shape_com_mesh1, tri[0]));
        const u32 behind = offset0 < 0.0f;
        const u32 safe = offset1 - offset0 > -safe_advancement && offset1 > safe_advancement;
        const struct aabb tri_bbox = { .min = V3Min(V3Min(tri[0], tri[1]), tri[2]), .max = V3Max(V3Max(tri[0], tri[1]), tri[2]) };
        if (!behind && !safe && queried[i] && AabbTest(&reach_box, &tri_bbox))
        {
            tri_index[*count] = i;
            *count += 1;
        }
    }

    return tri_index;
}

/* the smallest core distance at t from the convex arg to the listed triangles of the mesh arg (none: F32_INFINITY) */
static f32 ToiTestMeshDistance(const struct ds_TimeOfImpactArg *mesh, const struct ds_TimeOfImpactArg *convex, const u32 *tri_index, const u32 count, const f32 t)
{
    const struct mesh *m = &mesh->shape->mesh;
    f32 distance = F32_INFINITY;
    for (u32 i = 0; i < count; ++i)
    {
        const u32 *v = m->tri[tri_index[i]].buf;
        const v3 tri[3] = { m->v[v[0]], m->v[v[1]], m->v[v[2]] };
        struct ds_TimeOfImpactArg triangle;
        ds_TimeOfImpactArgTriangle(&triangle, mesh, tri);
        distance = F32Min(distance, ToiTestDistance(convex, &triangle, t));
    }

    return distance;
}

/* toi is a hit in (0, max_toi): the smallest core distance to the listed triangles lies in [target - tol, target + tol] */
static u32 ToiTestMeshHitCheck(const struct ds_TimeOfImpactArg *mesh, const struct ds_TimeOfImpactArg *convex, const u32 *tri_index, const u32 count, const f32 toi, const f32 max_toi)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 target = F32Max(slop, convex->margin - slop);
    const f32 distance = ToiTestMeshDistance(mesh, convex, tri_index, count, toi);
    const u32 hit = 0.0f < toi && toi < max_toi && target - tol <= distance && distance <= target + tol;
    if (!hit)
    {
        fprintf(stdout, "  toi %.9g, distance %.9g, target %.9g +- %.9g\n", (f64) toi, (f64) distance, (f64) target, (f64) tol);
    }

    return hit;
}

/* the fallback sphere ds_TimeOfImpactConcave sweeps for a triangle TOI of 0: the shape's COM, radius fallback_radius + slop */
static void ToiTestFallbackSphere(struct ds_TimeOfImpactArg *sphere, const struct ds_TimeOfImpactArg *convex)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 fallback_radius = F32Max(0.5f*convex->shape->com_inscribed_sphere_radius, slop);
    *sphere = *convex;
    GJKVertexSetPoint(&sphere->vset, convex->shape->center_of_mass);
    sphere->margin = fallback_radius + slop;
}

/* a box (half width 0.5) and a sphere (radius 0.5) fall 2 m from 1 m above the floor onto it */
struct test_Output ToiMeshFallTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.5f } };
    c_ShapeUpdateMassProperties(&box);
    c_ShapeUpdateMassProperties(&sphere);
    const struct c_Shape *shape[2] = { &box, &sphere };

    const ds_Transform transform[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 1.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, -0.5f, 2.0f) },
    };

    for (u32 i = 0; i < 2; ++i)
    {
        struct ds_TimeOfImpactArg convex;
        ToiTestArgInit(&convex, shape[i], transform, shape[i]->center_of_mass);
        u32 count = 0;
        const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh, &convex, F32_INFINITY, 0);
        TEST_TRUE(count > 0);
        const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
        TEST_TRUE(ToiTestMeshHitCheck(&mesh, &convex, candidate, count, toi, 1.0f));
    }

    return output;
}

/* the floor fall cut off at max_toi = 0.3, before the hit (~0.5) */
struct test_Output ToiMeshBeyondMaxToiTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);
    const ds_Transform transform[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 1.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, -0.5f, 2.0f) },
    };
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &box, transform, box.center_of_mass);

    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 0.3f);
    TEST_EQUAL(0.3f, toi);

    return output;
}

/*
 * no hit: a box resting on the floor (and one hovering half a slop above it) slides 2 m along it, a box resting on
 * it lifts off, a box under it rises through it (one-sided: the COM starts behind), a box falls beside it
 */
struct test_Output ToiMeshSkipTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();
    const f32 slop = g_solver_config->linear_slop;

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);

    const v3 sweep[5][2] =
    {
        { V3(1.0f, 0.5f, 2.0f),                 V3(3.0f, 0.5f, 2.0f) },
        { V3(1.0f, 0.5f + 0.5f*slop, 2.0f),     V3(3.0f, 0.5f + 0.5f*slop, 2.0f) },
        { V3(2.0f, 0.5f, 2.0f),                 V3(2.0f, 2.5f, 2.0f) },
        { V3(2.0f, -1.5f, 2.0f),                V3(2.0f, 1.5f, 2.0f) },
        { V3(6.0f, 1.5f, 2.0f),                 V3(6.0f, -0.5f, 2.0f) },
    };

    for (u32 i = 0; i < 5; ++i)
    {
        const ds_Transform transform[2] =
        {
            { .rotation = QIdentity(), .position = sweep[i][0] },
            { .rotation = QIdentity(), .position = sweep[i][1] },
        };
        struct ds_TimeOfImpactArg convex;
        ToiTestArgInit(&convex, &box, transform, box.center_of_mass);
        const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
        TEST_EQUAL(1.0f, toi);
    }

    return output;
}

/*
 * a box resting on the floor (and one hovering half a slop above it, no contact there) is driven 2 m into it: the
 * triangles give a TOI of 0 and the fallback sphere stops the COM fallback_radius = 0.25 above the floor. A box
 * already deeper (COM 0.2 above the floor) gets no hit: CCD gives up.
 */
struct test_Output ToiMeshFallbackTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();
    const f32 slop = g_solver_config->linear_slop;

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);

    const f32 height[2] = { 0.5f, 0.5f + 0.5f*slop };
    for (u32 i = 0; i < 2; ++i)
    {
        const ds_Transform transform[2] =
        {
            { .rotation = QIdentity(), .position = V3(2.0f, height[i], 2.0f) },
            { .rotation = QIdentity(), .position = V3(2.0f, height[i] - 2.0f, 2.0f) },
        };
        struct ds_TimeOfImpactArg convex;
        ToiTestArgInit(&convex, &box, transform, box.center_of_mass);
        u32 count = 0;
        const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh, &convex, F32_INFINITY, 0);
        const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
        TEST_TRUE(ToiTestMeshDistance(&mesh, &convex, candidate, count, 0.0f) <= slop + 0.25f*slop);

        struct ds_TimeOfImpactArg fallback_sphere;
        ToiTestFallbackSphere(&fallback_sphere, &convex);
        TEST_TRUE(ToiTestMeshHitCheck(&mesh, &fallback_sphere, candidate, count, toi, 1.0f));
    }

    const ds_Transform deep[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 0.2f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, -1.8f, 2.0f) },
    };
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &box, deep, box.center_of_mass);
    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
    TEST_EQUAL(1.0f, toi);

    return output;
}

/*
 * a box resting on the floor slides 4 m into the wall x = 4: the floor is skipped, the wall stops the box's face a
 * slop short. A sphere (radius 0.5) moves diagonally into the corner of the walls x = 4 and z = 4: both at once.
 */
struct test_Output ToiMeshWallTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, TOI_TEST_ROOM_WALL_X | TOI_TEST_ROOM_WALL_Z);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);
    const ds_Transform slide[2] =
    {
        { .rotation = QIdentity(), .position = V3(1.5f, 0.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(5.5f, 0.5f, 2.0f) },
    };
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &box, slide, box.center_of_mass);
    u32 count = 0;
    const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh, &convex, F32_INFINITY, 0);
    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
    TEST_TRUE(ToiTestMeshHitCheck(&mesh, &convex, candidate, count, toi, 1.0f));

    struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.5f } };
    c_ShapeUpdateMassProperties(&sphere);
    const ds_Transform corner[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 2.0f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(6.0f, 2.0f, 6.0f) },
    };
    ToiTestArgInit(&convex, &sphere, corner, sphere.center_of_mass);
    candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh, &convex, F32_INFINITY, 0);
    const f32 toi_corner = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
    TEST_TRUE(ToiTestMeshHitCheck(&mesh, &convex, candidate, count, toi_corner, 1.0f));

    return output;
}

/*
 * a capsule (half height 1.2, radius 0.05) lying along x 1 m above the floor turns a quarter about z until it
 * stands, its lower end swinging through the floor. In place, its COM never approaches the floor and every
 * triangle is skipped: the early-outs do not see rotation (ds_TimeOfImpactConcave's documented limitation). Coming
 * down 0.5 m while turning, the triangles get their TOI and the lower end stops on the floor.
 */
struct test_Output ToiMeshSpinningStickTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));

    struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 1.2f, .radius = 0.05f } };
    c_ShapeUpdateMassProperties(&capsule);
    const q lying = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -0.5f*F32_PI);

    const ds_Transform in_place[2] =
    {
        { .rotation = lying, .position = V3(2.0f, 1.0f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, 1.0f, 2.0f) },
    };
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &capsule, in_place, capsule.center_of_mass);
    const f32 toi_in_place = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
    TEST_EQUAL(1.0f, toi_in_place);

    const ds_Transform coming_down[2] =
    {
        { .rotation = lying, .position = V3(2.0f, 1.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, 1.0f, 2.0f) },
    };
    ToiTestArgInit(&convex, &capsule, coming_down, capsule.center_of_mass);
    u32 count = 0;
    const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh, &convex, F32_INFINITY, 0);
    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);
    TEST_TRUE(ToiTestMeshHitCheck(&mesh, &convex, candidate, count, toi, 1.0f));

    return output;
}

/*
 * ToiMeshFallTest's box fall with the mesh body at a world pose g, the mesh at t_local l_mesh in it and the box at
 * t_local l_box in its body (COM off the body's origin): a hit, the same toi up to rounding
 */
struct test_Output ToiMeshTransformTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    const ds_Transform transform[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 1.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, -0.5f, 2.0f) },
    };

    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &box, transform, box.center_of_mass);
    const f32 toi_plain = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);

    const ds_TransformWorld g = { .rotation = QUnitAxisAngle(V3Normalize(V3(1.0f, 2.0f, 3.0f)), 0.7f), .position = V3F64(3.0, -2.0, 5.0) };
    const ds_Transform l_mesh = { .rotation = QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), 0.3f), .position = V3(0.5f, 0.1f, -0.2f) };
    const ds_Transform l_box = { .rotation = QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), 0.4f), .position = V3(0.2f, -0.3f, 0.1f) };
    const ds_Transform l_box_inverse = ds_TransformRelative(&l_box, &t_identity);
    const ds_TransformWorld mesh_world = ds_TransformWorldChild(&g, &l_mesh);
    const ds_TransformWorld box_world0 = ds_TransformWorldChild(&mesh_world, transform + 0);
    const ds_TransformWorld box_world1 = ds_TransformWorldChild(&mesh_world, transform + 1);
    const ds_TransformWorld box_body_world[2] =
    {
        ds_TransformWorldChild(&box_world0, &l_box_inverse),
        ds_TransformWorldChild(&box_world1, &l_box_inverse),
    };

    struct ds_TimeOfImpactArg mesh_moved;
    ds_TimeOfImpactArgMesh(&mesh_moved, &room, &l_mesh, &g, V3F64(0.0, 0.0, 0.0));
    struct ds_TimeOfImpactArg convex_moved;
    ToiTestArgInitWorld(&convex_moved, &box, &l_box, box_body_world, V3Transform(box.center_of_mass, &l_box), V3F64(0.0, 0.0, 0.0));
    u32 count = 0;
    const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh_moved, &convex_moved, F32_INFINITY, 0);
    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh_moved, &convex_moved, 1.0f);
    TEST_TRUE(ToiTestMeshHitCheck(&mesh_moved, &convex_moved, candidate, count, toi, 1.0f));
    TEST_TRUE(F32Abs(toi - toi_plain) <= 1.0e-3f);

    return output;
}

/*
 * ToiMeshFallTest's box fall 10 km from the world origin: anchored at the mesh, the args equal the plain case's and
 * so does the toi (bit for bit); anchored 0.25, 0.5, 0.75 away from it, still a hit
 */
struct test_Output ToiMeshAnchorTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    struct c_Shape room;
    ToiTestRoom(&room, env->mem_1, 0);
    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, V3(0.5f, 0.5f, 0.5f)) };
    c_ShapeUpdateMassProperties(&box);
    const ds_Transform t_identity = ds_TransformIdentity();
    const ds_TransformWorld t_world_identity = ds_TransformWorldIdentity();
    const ds_Transform transform[2] =
    {
        { .rotation = QIdentity(), .position = V3(2.0f, 1.5f, 2.0f) },
        { .rotation = QIdentity(), .position = V3(2.0f, -0.5f, 2.0f) },
    };

    struct ds_TimeOfImpactArg mesh;
    ds_TimeOfImpactArgMesh(&mesh, &room, &t_identity, &t_world_identity, V3F64(0.0, 0.0, 0.0));
    struct ds_TimeOfImpactArg convex;
    ToiTestArgInit(&convex, &box, transform, box.center_of_mass);
    const f32 toi_plain = ds_TimeOfImpactConcave(env->mem_2, &mesh, &convex, 1.0f);

    const v3f64 far = V3F64(10000.0, 0.0, -10000.0);
    const ds_TransformWorld mesh_world = { .rotation = QIdentity(), .position = far };
    const ds_TransformWorld box_world[2] =
    {
        { .rotation = QIdentity(), .position = V3F64Add(far, V3Promote(transform[0].position)) },
        { .rotation = QIdentity(), .position = V3F64Add(far, V3Promote(transform[1].position)) },
    };

    struct ds_TimeOfImpactArg mesh_far;
    ds_TimeOfImpactArgMesh(&mesh_far, &room, &t_identity, &mesh_world, far);
    struct ds_TimeOfImpactArg convex_far;
    ToiTestArgInitWorld(&convex_far, &box, &t_identity, box_world, box.center_of_mass, far);
    const f32 toi = ds_TimeOfImpactConcave(env->mem_2, &mesh_far, &convex_far, 1.0f);
    TEST_EQUAL(toi_plain, toi);

    const v3f64 near = V3F64Add(far, V3F64(0.25, 0.5, -0.75));
    ds_TimeOfImpactArgMesh(&mesh_far, &room, &t_identity, &mesh_world, near);
    ToiTestArgInitWorld(&convex_far, &box, &t_identity, box_world, box.center_of_mass, near);
    u32 count = 0;
    const u32 *candidate = ToiTestMeshCandidates(env->mem_2, &count, &mesh_far, &convex_far, F32_INFINITY, 0);
    const f32 toi_near = ds_TimeOfImpactConcave(env->mem_2, &mesh_far, &convex_far, 1.0f);
    TEST_TRUE(ToiTestMeshHitCheck(&mesh_far, &convex_far, candidate, count, toi_near, 1.0f));

    return output;
}

#define TOI_TEST_MESH_SAMPLES   64
#define TOI_TEST_MESH_SWEEPS    100

/*
 * kind: the convex shape, its size drawn from [size_min, size_max]. height_max: the terrain's heights. turn: the
 * shape turns by up to this angle over the sweep. offset: the scene's distance from the world origin; anchored: the
 * args' anchor sits at the terrain (else at the world origin). graze: sweeps run sideways just above the terrain
 * instead of falling onto it. max_toi 0: 1.
 */
struct toi_TestMeshScenario
{
    const char *        id;
    enum toi_TestKind   kind;
    f32                 size_min;
    f32                 size_max;
    f32                 height_max;
    f32                 turn;
    f64                 offset;
    u32                 anchored;
    u32                 graze;
    f32                 max_toi;
};

struct toi_TestMeshCount
{
    u64 sweeps;
    u64 touching;       /* a candidate within target + tol at t = 0: skipped */
    u64 nan;
    u64 hit;            /* candidates' distance at toi in [target - tol, target + tol] */
    u64 fallback;       /* the fallback sphere's distance at toi in its band (a triangle TOI of 0 without touching) */
    u64 early;          /* above the band: an iteration limit stopped a triangle's TOI (safe) */
    u64 overshoot;      /* below the band: fails */
    u64 late;           /* a sample before toi touches a candidate: fails */
    u64 miss;           /* max_toi returned */
    u64 missed;         /* max_toi returned although a sample touches a candidate */
    u64 escaped;        /* a triangle outside the path box (not a candidate) touched before or at toi */
};

static void ToiTestMeshScenarioRun(struct toi_TestMeshCount *count, struct arena *mem, const struct toi_TestMeshScenario *scenario, const u32 sweeps)
{
    const f32 slop = g_solver_config->linear_slop;
    const f32 tol = 0.25f*slop;
    const f32 max_toi = (scenario->max_toi > 0.0f) ? scenario->max_toi : 1.0f;
    const f32 cells = (f32) TOI_TEST_TERRAIN_CELLS;
    const ds_Transform t_identity = ds_TransformIdentity();
    for (u32 i = 0; i < sweeps; ++i)
    {
        const struct arena record = *mem;
        struct c_Shape terrain;
        ToiTestTerrain(&terrain, mem, scenario->height_max);
        struct toi_TestShape shape;
        const f32 size = RngF32Range(scenario->size_min, scenario->size_max);
        ToiTestShapeInit(&shape, mem, scenario->kind, size);
        c_ShapeUpdateMassProperties(&shape.shape);
        const f32 reach = shape.extent + V3Length(shape.shape.center_of_mass);

        /* the terrain's world pose and the anchor */
        const q q_mesh = ToiTestRotation(2.0f*F32_PI);
        const v3 offset_dir = ToiTestUnit();
        const v3f64 origin = V3F64(scenario->offset*(f64) offset_dir.x, scenario->offset*(f64) offset_dir.y, scenario->offset*(f64) offset_dir.z);
        const ds_TransformWorld t_mesh_world = { .rotation = q_mesh, .position = origin };
        v3f64 anchor = V3F64(0.0, 0.0, 0.0);
        if (scenario->anchored)
        {
            anchor = origin;
        }

        /* the shape's sweep in the terrain's frame: from above onto it, or sideways just above it */
        const f32 x0 = RngF32Range(1.0f, cells - 1.0f);
        const f32 z0 = RngF32Range(1.0f, cells - 1.0f);
        const f32 x1 = RngF32Range(1.0f, cells - 1.0f);
        const f32 z1 = RngF32Range(1.0f, cells - 1.0f);
        f32 y0 = 0.0f;
        f32 y1 = 0.0f;
        if (scenario->graze)
        {
            y0 = scenario->height_max + reach*RngF32Range(0.3f, 1.2f);
            y1 = scenario->height_max + reach*RngF32Range(0.3f, 1.2f);
        }
        else
        {
            y0 = scenario->height_max + reach + RngF32Range(0.1f, 2.0f);
            y1 = RngF32Range(-reach, scenario->height_max);
        }

        const q q0 = ToiTestRotation(2.0f*F32_PI);
        const q dq = ToiTestRotation(scenario->turn);
        const q q1 = QMul(dq, q0);
        const v3 p0 = QUnitRotate(q_mesh, V3(x0, y0, z0));
        const v3 p1 = QUnitRotate(q_mesh, V3(x1, y1, z1));
        const ds_TransformWorld t_body_world[2] =
        {
            { .rotation = QMul(q_mesh, q0), .position = V3F64Add(origin, V3Promote(p0)) },
            { .rotation = QMul(q_mesh, q1), .position = V3F64Add(origin, V3Promote(p1)) },
        };

        struct ds_TimeOfImpactArg mesh_arg;
        ds_TimeOfImpactArgMesh(&mesh_arg, &terrain, &t_identity, &t_mesh_world, anchor);
        struct ds_TimeOfImpactArg convex;
        ToiTestArgInitWorld(&convex, &shape.shape, &t_identity, t_body_world, shape.shape.center_of_mass, anchor);

        u32 candidate_count = 0;
        const u32 *candidate = ToiTestMeshCandidates(mem, &candidate_count, &mesh_arg, &convex, reach, 1);
        u32 candidate_all_count = 0;
        const u32 *candidate_all = ToiTestMeshCandidates(mem, &candidate_all_count, &mesh_arg, &convex, reach, 0);
        const f32 target = F32Max(slop, convex.margin - slop);
        f32 far = 0.0f;
        if (!scenario->anchored)
        {
            far = (f32) scenario->offset;
        }

        const f32 noise = 64.0f*F32_EPSILON*(far + 2.0f*cells + 4.0f*reach);

        count->sweeps += 1;
        if (ToiTestMeshDistance(&mesh_arg, &convex, candidate, candidate_count, 0.0f) <= target + tol)
        {
            count->touching += 1;
            *mem = record;
            continue;
        }

        const f32 toi = ds_TimeOfImpactConcave(mem, &mesh_arg, &convex, max_toi);
        if (toi != toi)
        {
            count->nan += 1;
            *mem = record;
            continue;
        }

        const f32 t_end = F32Min(toi, max_toi);
        const f32 touch = target - tol - noise;
        f32 sampled = F32_INFINITY;
        f32 sampled_all = F32_INFINITY;
        for (u32 k = 0; k < TOI_TEST_MESH_SAMPLES; ++k)
        {
            const f32 t = t_end*(f32) k / (f32) TOI_TEST_MESH_SAMPLES;
            sampled = F32Min(sampled, ToiTestMeshDistance(&mesh_arg, &convex, candidate, candidate_count, t));
            sampled_all = F32Min(sampled_all, ToiTestMeshDistance(&mesh_arg, &convex, candidate_all, candidate_all_count, t));
        }

        u32 escaped = sampled_all < touch && sampled >= touch;
        if (toi < max_toi)
        {
            const f32 distance_all = ToiTestMeshDistance(&mesh_arg, &convex, candidate_all, candidate_all_count, toi);
            const f32 distance_box = ToiTestMeshDistance(&mesh_arg, &convex, candidate, candidate_count, toi);
            escaped = escaped || (distance_all < touch && distance_box >= touch);
        }

        count->escaped += escaped;

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
            struct ds_TimeOfImpactArg fallback_sphere;
            ToiTestFallbackSphere(&fallback_sphere, &convex);
            const f32 fallback_target = fallback_sphere.margin - slop;
            const f32 fallback_distance = ToiTestMeshDistance(&mesh_arg, &fallback_sphere, candidate, candidate_count, toi);
            const f32 distance = ToiTestMeshDistance(&mesh_arg, &convex, candidate, candidate_count, toi);
            if (target - tol - noise <= distance && distance <= target + tol)
            {
                count->hit += 1;
            }
            else if (fallback_target - tol - noise <= fallback_distance && fallback_distance <= fallback_target + tol)
            {
                count->fallback += 1;
            }
            else if (distance > target + tol)
            {
                count->early += 1;
            }
            else
            {
                count->overshoot += 1;
                fprintf(stdout, "  %s, sweep %u: toi %.9g, distance %.9g below the band [%.9g, %.9g]\n", scenario->id, i, (f64) toi, (f64) distance, (f64) (target - tol), (f64) (target + tol));
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

static const struct toi_TestMeshScenario g_toi_test_mesh_scenario[] =
{
    { .id = "box onto a flat floor",                .kind = TOI_TEST_BOX,       .size_min = 0.2f,   .size_max = 2.0f },
    { .id = "box onto bumps, turn 0.3",             .kind = TOI_TEST_BOX,       .size_min = 0.2f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 0.3f },
    { .id = "hull onto rough terrain, turn 0.3",    .kind = TOI_TEST_HULL,      .size_min = 0.2f,   .size_max = 2.0f, .height_max = 1.0f, .turn = 0.3f },
    { .id = "hull onto rough terrain, turn 1.0",    .kind = TOI_TEST_HULL,      .size_min = 0.2f,   .size_max = 2.0f, .height_max = 1.0f, .turn = 1.0f },
    { .id = "capsule onto bumps, turn 0.3",         .kind = TOI_TEST_CAPSULE,   .size_min = 0.3f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 0.3f },
    { .id = "sphere onto rough terrain",            .kind = TOI_TEST_SPHERE,    .size_min = 0.1f,   .size_max = 1.0f, .height_max = 1.0f },
    { .id = "stick onto bumps, turn 1.0",           .kind = TOI_TEST_STICK,     .size_min = 0.5f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 1.0f },
    { .id = "plate onto a flat floor, turn 0.3",    .kind = TOI_TEST_PLATE,     .size_min = 0.5f,   .size_max = 2.0f, .turn = 0.3f },
    { .id = "small hulls onto bumps, turn 0.3",     .kind = TOI_TEST_HULL,      .size_min = 0.02f,  .size_max = 0.2f, .height_max = 0.3f, .turn = 0.3f },
    { .id = "box grazing bumps",                    .kind = TOI_TEST_BOX,       .size_min = 0.2f,   .size_max = 2.0f, .height_max = 0.3f, .graze = 1 },
    { .id = "capsule grazing terrain, turn 0.3",    .kind = TOI_TEST_CAPSULE,   .size_min = 0.3f,   .size_max = 2.0f, .height_max = 1.0f, .turn = 0.3f, .graze = 1 },
    { .id = "hull at 1000 m, anchored, turn 0.3",   .kind = TOI_TEST_HULL,      .size_min = 0.2f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 0.3f, .offset = 1000.0, .anchored = 1 },
    { .id = "hull at 100 m, turn 0.3",              .kind = TOI_TEST_HULL,      .size_min = 0.2f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 0.3f, .offset = 100.0 },
    { .id = "hull, max_toi 0.5, turn 0.3",          .kind = TOI_TEST_HULL,      .size_min = 0.2f,   .size_max = 2.0f, .height_max = 0.3f, .turn = 0.3f, .max_toi = 0.5f },
};

struct test_Output ToiMeshRandomTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    ToiTestConfigPush();

    const u32 scenario_count = sizeof(g_toi_test_mesh_scenario) / sizeof(g_toi_test_mesh_scenario[0]);
    struct toi_TestMeshCount total = { 0 };
    for (u32 i = 0; i < scenario_count; ++i)
    {
        struct toi_TestMeshCount count = { 0 };
        ToiTestMeshScenarioRun(&count, env->mem_1, g_toi_test_mesh_scenario + i, TOI_TEST_MESH_SWEEPS);
        fprintf(stdout, "  %-38s sweeps %llu touching %llu nan %llu hit %llu fallback %llu early %llu overshoot %llu late %llu miss %llu (missed contacts %llu, outside the path box %llu)\n",
            g_toi_test_mesh_scenario[i].id, (unsigned long long) count.sweeps, (unsigned long long) count.touching, (unsigned long long) count.nan,
            (unsigned long long) count.hit, (unsigned long long) count.fallback, (unsigned long long) count.early, (unsigned long long) count.overshoot,
            (unsigned long long) count.late, (unsigned long long) count.miss, (unsigned long long) count.missed, (unsigned long long) count.escaped);
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
    ToiMeshFallTest,
    ToiMeshBeyondMaxToiTest,
    ToiMeshSkipTest,
    ToiMeshFallbackTest,
    ToiMeshWallTest,
    ToiMeshSpinningStickTest,
    ToiMeshTransformTest,
    ToiMeshAnchorTest,
    ToiMeshRandomTest,
};

struct suite_Correctness m_toi_suite =
{
    .id = "toi",
    .unit_test = toi_tests,
    .unit_test_count = sizeof(toi_tests) / sizeof(toi_tests[0]),
};

struct suite_Correctness *toi_correctness_suite = &m_toi_suite;

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
ds_TimeOfImpact on hand-made sweeps with known outcomes (max_toi = 1). A hit is checked by the core distance
(GJK, margins 0) at the returned toi: it lies in [target - tol, target + tol], with

    target  = max(slop, r0 + r1 - slop)
    tol     = slop / 4

Static shapes sit at the origin. The args' vertex sets are built in place: a point or segment vertex set
points into its own storage, so an arg is never copied.
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

/* the core distance of the args at t, posed as ds_TimeOfImpact poses them */
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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 0.4f);
    TEST_EQUAL(0.4f, toi);

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_EQUAL(0.0f, toi);

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_EQUAL(1.0f, toi);

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

    const f32 toi = ds_TimeOfImpact(&arg0, &arg1, 1.0f);
    TEST_TRUE(ToiTestHitCheck(&arg0, &arg1, toi, 1.0f));

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
};

struct suite_Correctness m_toi_suite =
{
    .id = "toi",
    .unit_test = toi_tests,
    .unit_test_count = sizeof(toi_tests) / sizeof(toi_tests[0]),
};

struct suite_Correctness *toi_correctness_suite = &m_toi_suite;

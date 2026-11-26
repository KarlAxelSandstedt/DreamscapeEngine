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

#include <stdio.h>
#include <string.h>

#include "ds_test.h"
#include "ds_dynamics.h"
#include "ds_collision.h"
#include "ds_geometry.h"

/*
Kinematic body tests
====================
A kinematic body moves by the velocity it was given (ds_BodyVelocitySet) and by nothing else. Every body here is
a unit cube; h is the tick, 1/60 s. The tests so far cover a kinematic body on its own: it finds no pairs yet.

    Motion      position and rotation after a second of a set velocity, next to a dynamic cube that falls
    NoCap       velocities above both caps are kept bit for bit
    Proxy       a ray cast finds the cube after a move its proxy absorbs in place and after one that reinserts it
    Sleep       asleep after resting, awake at 1 mm/s (below the dynamic rest speed), woken by the setter
    Setter      static bodies and dead ids are left alone; a sleeping dynamic body is woken and written
    Rebuild     moving kinematic cubes alone never rebuild the dynamic tree; with a rebuild every tick their proxies still follow
    Remove      removing a kinematic body, awake or asleep, empties the kinematic tree
*/

struct kinematic_TestWorld
{
    struct arena                mem;
    struct arena                tick;
    struct c_ShapeDB            cshape_db;
    struct ds_DynamicsExternal  external;
    struct ds_Dynamics          pipeline;
    u32                         cube;
};

/* the world must stay where it is: the pipeline keeps the address of world->external */
static void KinematicTestWorldInit(struct kinematic_TestWorld *world)
{
    world->mem = ArenaAlloc(NULL, 16*1024*1024);
    world->tick = ArenaAlloc(NULL, 16*1024*1024);
    world->cshape_db = (struct c_ShapeDB) { 0 };
    c_ShapeDBAlloc(NULL, &world->cshape_db, 32, GROWABLE);
    world->external = (struct ds_DynamicsExternal) { .cshape_db = &world->cshape_db };
    world->pipeline = ds_DynamicsAllocDefault(&world->external, NSEC_PER_SEC / 60);
    ds_DynamicsConfigPush(&world->pipeline);

    const struct c_Shape cube = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(&world->mem, V3(0.5f, 0.5f, 0.5f)) };
    world->cube = c_ShapeDBAdd(&world->cshape_db, Utf8Inline("kinematic_cube"), &cube).index;
}

static void KinematicTestWorldFree(struct kinematic_TestWorld *world)
{
    ds_DynamicsConfigPop(&world->pipeline);
    ds_DynamicsDealloc(&world->pipeline);
    c_ShapeDBDealloc(&world->cshape_db);
    ArenaFree(&world->tick);
    ArenaFree(&world->mem);
}

static void KinematicTestTick(struct kinematic_TestWorld *world, const u32 count)
{
    for (u32 i = 0; i < count; ++i)
    {
        ds_DynamicsTick(&world->tick, &world->pipeline, NSEC_PER_SEC / 60);
        ds_PoolFlush(world->pipeline.event_pool);
        DLLFlush(world->pipeline.event_list);
    }
}

/* a unit cube body of the given type (BODY_FLAG_DYNAMIC, BODY_FLAG_KINEMATIC or 0) */
static ds_BodyId KinematicTestCubeAdd(struct kinematic_TestWorld *world, const u32 flags, const v3f64 position, const u32 entity)
{
    const struct ds_BodyPrefab body_prefab = { .flags = flags };
    const ds_TransformWorld t_world = { .rotation = QIdentity(), .position = position };
    const ds_BodyId id = ds_BodyAdd(&world->pipeline, &body_prefab, &t_world, entity);
    const struct ds_ShapePrefab shape_prefab = { .cshape = world->cube, .density = 1.0f, .restitution = 0.0f, .friction = 0.5f };
    const ds_Transform t_local = ds_TransformIdentity();
    ds_ShapeAdd(&world->pipeline, &shape_prefab, &t_local, id);
    return id;
}

static ds_TransformWorld KinematicTestPose(const struct kinematic_TestWorld *world, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(&world->pipeline, id).address;
    const struct ds_SolverSet *set = world->pipeline.solver_set_pool.buf + body->set;
    return set->body_sim_pool.buf[body->sim].world;
}

static u32 KinematicTestAsleepCheck(const struct kinematic_TestWorld *world, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(&world->pipeline, id).address;
    return body->set >= SOLVER_SET_SLEEPING_FIRST;
}

/* the shape a ray straight down from 5 m above (x, 0, z) hits first, with its distance; U32_MAX: none */
static u32f32 KinematicTestRayDown(const struct kinematic_TestWorld *world, const f64 x, const f64 z)
{
    const struct rayWorld ray = { .origin = V3F64(x, 5.0, z), .dir = V3(0.0f, -1.0f, 0.0f) };
    return ds_DynamicsRayCastParameter(&world->pipeline, &ray);
}

/* 1 if the ray down at the body's position hits the body's cube on its top face (4.5 m below the ray's origin) */
static u32 KinematicTestRayFindsCheck(const struct kinematic_TestWorld *world, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(&world->pipeline, id).address;
    const ds_TransformWorld pose = KinematicTestPose(world, id);
    const u32f32 hit = KinematicTestRayDown(world, pose.position.x, pose.position.z);
    return hit.u == (u32) body->shape_list.first && F32Abs(hit.f - 4.5f) <= 1.0e-4f;
}

static struct test_Output KinematicMotionTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    TEST_TRUE(world.pipeline.config.solver.gravity.y < 0.0f);

    const v3f64 start = V3F64(0.0, 10.0, 0.0);
    const v3 v = V3(1.0f, 2.0f, -3.0f);
    const v3 w = V3(0.0f, 0.5f, 0.0f);
    const ds_BodyId kinematic = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, start, 1);
    const ds_BodyId dynamic = KinematicTestCubeAdd(&world, BODY_FLAG_DYNAMIC, V3F64(50.0, 10.0, 0.0), 2);
    ds_BodyVelocitySet(&world.pipeline, kinematic, v, w);

    const u32 tick_count = 60;
    KinematicTestTick(&world, tick_count);

    /* position = start + sum of v h; the rotation turns 2 atan(h |w| / 2) per tick about y, 0.5 rad in all to 3e-6 */
    const f64 t = (f64) tick_count * (f64) world.pipeline.timestep;
    const ds_TransformWorld pose = KinematicTestPose(&world, kinematic);
    const f64 dx = pose.position.x - (start.x + (f64) v.x * t);
    const f64 dy = pose.position.y - (start.y + (f64) v.y * t);
    const f64 dz = pose.position.z - (start.z + (f64) v.z * t);
    const q expected = Q(0.0f, F32Sin(0.25f), 0.0f, F32Cos(0.25f));
    const f32 rotation_error = F32Abs(pose.rotation.x - expected.x) + F32Abs(pose.rotation.y - expected.y) + F32Abs(pose.rotation.z - expected.z) + F32Abs(pose.rotation.w - expected.w);

    /* no gravity, no damping: the stored velocities are the given ones bit for bit */
    const struct ds_Body *body = ds_BodyLookup(&world.pipeline, kinematic).address;
    TEST_EQUAL(body->set, SOLVER_SET_ACTIVE);
    const struct ds_BodyCompute *compute = world.pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf + body->sim;
    const u32 velocity_kept = memcmp(&compute->linear_velocity, &v, sizeof(v)) == 0 && memcmp(&compute->angular_velocity, &w, sizeof(w)) == 0;

    /* the dynamic cube beside it fell about g t^2 / 2 = 4.9 m */
    const ds_TransformWorld dynamic_pose = KinematicTestPose(&world, dynamic);
    const u32 ok = dx*dx + dy*dy + dz*dz <= 1.0e-5*1.0e-5 && rotation_error <= 1.0e-5f && velocity_kept && dynamic_pose.position.y < 6.0;
    if (!ok)
    {
        fprintf(stderr, "kinematic motion: position error (%g, %g, %g), rotation error %g, velocity kept %u, dynamic y %g\n", dx, dy, dz, (f64) rotation_error, velocity_kept, dynamic_pose.position.y);
        TEST_FAILURE;
    }

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicNoCapTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    const f32 v_max = world.pipeline.config.solver.max_linear_velocity_magnitude;
    const f32 w_max = world.pipeline.config.solver.max_angular_velocity_magnitude;
    TEST_TRUE(v_max > 0.0f && w_max > 0.0f);

    const v3f64 start = V3F64(0.0, 10.0, 0.0);
    const v3 v = V3Scale(V3Normalize(V3(1.0f, 2.0f, -3.0f)), 2.0f*v_max);
    const v3 w = V3Scale(V3Normalize(V3(-2.0f, 1.0f, 1.0f)), 3.0f*w_max);
    const ds_BodyId id = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, start, 1);
    ds_BodyVelocitySet(&world.pipeline, id, v, w);
    KinematicTestTick(&world, 1);

    const struct ds_Body *body = ds_BodyLookup(&world.pipeline, id).address;
    TEST_EQUAL(body->set, SOLVER_SET_ACTIVE);
    const struct ds_BodyCompute *compute = world.pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf + body->sim;
    const u32 velocity_kept = memcmp(&compute->linear_velocity, &v, sizeof(v)) == 0 && memcmp(&compute->angular_velocity, &w, sizeof(w)) == 0;

    const ds_TransformWorld pose = KinematicTestPose(&world, id);
    const f64 dx = pose.position.x - start.x;
    const f64 dy = pose.position.y - start.y;
    const f64 dz = pose.position.z - start.z;
    const f64 moved_sq = dx*dx + dy*dy + dz*dz;
    const f64 step = 2.0 * (f64) v_max * (f64) world.pipeline.timestep;
    const u32 ok = velocity_kept && moved_sq >= step*step*(1.0 - 1.0e-4) && moved_sq <= step*step*(1.0 + 1.0e-4);
    if (!ok)
    {
        fprintf(stderr, "kinematic no cap: velocity kept %u, moved^2 %g (expected %g)\n", velocity_kept, moved_sq, step*step);
        TEST_FAILURE;
    }

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicProxyTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* two leaves: the root box spans both, so a small move of one is absorbed by enlarging its proxy in place */
    const ds_BodyId near = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId far = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, V3F64(100.0, 0.0, 0.0), 2);
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 2);
    TEST_EQUAL(BTLeafCount(world.pipeline.dynamic_bvh.bt), 0);
    TEST_TRUE(KinematicTestRayFindsCheck(&world, near));
    TEST_TRUE(KinematicTestRayFindsCheck(&world, far));

    /* 0.1 m along x: past the proxy's margin (0.07 m), inside the root box */
    ds_BodyVelocitySet(&world.pipeline, near, V3(6.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 1);
    TEST_TRUE(KinematicTestRayFindsCheck(&world, near));
    TEST_EQUAL(KinematicTestRayDown(&world, -0.45, 0.0).u, U32_MAX);

    /* 10 m along z: outside the root box, so the proxy is reinserted */
    ds_BodyVelocitySet(&world.pipeline, near, V3(0.0f, 0.0f, 600.0f), V3Zero());
    KinematicTestTick(&world, 1);
    TEST_TRUE(KinematicTestRayFindsCheck(&world, near));
    TEST_EQUAL(KinematicTestRayDown(&world, 0.1, 0.0).u, U32_MAX);
    TEST_TRUE(KinematicTestRayFindsCheck(&world, far));
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 2);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicSleepTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    TEST_TRUE(world.pipeline.config.solver.sleep_enabled);
    TEST_TRUE(world.pipeline.config.solver.sleep_time_threshold <= 0.5f);

    /* 40 ticks = 0.67 s at rest: asleep */
    const v3f64 start = V3F64(0.0, 10.0, 0.0);
    const ds_BodyId id = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, start, 1);
    KinematicTestTick(&world, 40);
    TEST_TRUE(KinematicTestAsleepCheck(&world, id));

    /* a zero velocity leaves it asleep */
    ds_BodyVelocitySet(&world.pipeline, id, V3Zero(), V3Zero());
    TEST_TRUE(KinematicTestAsleepCheck(&world, id));

    /* 1 mm/s is below the rest speed of a dynamic body (1 cm/s): the setter wakes it and it never rests */
    const v3 slow = V3(0.001f, 0.0f, 0.0f);
    TEST_TRUE(V3Dot(slow, slow) < world.pipeline.config.solver.sleep_linear_velocity_sq_limit);
    ds_BodyVelocitySet(&world.pipeline, id, slow, V3Zero());
    TEST_FALSE(KinematicTestAsleepCheck(&world, id));
    KinematicTestTick(&world, 120);
    TEST_FALSE(KinematicTestAsleepCheck(&world, id));
    const ds_TransformWorld pose = KinematicTestPose(&world, id);
    const f64 moved = pose.position.x - start.x;
    const f64 expected = 120.0 * (f64) world.pipeline.timestep * (f64) slow.x;
    if (!(moved >= expected - 1.0e-6 && moved <= expected + 1.0e-6))
    {
        fprintf(stderr, "kinematic sleep: moved %g at 1 mm/s (expected %g)\n", moved, expected);
        TEST_FAILURE;
    }

    /* stopped: asleep again; an angular velocity alone wakes it as well */
    ds_BodyVelocitySet(&world.pipeline, id, V3Zero(), V3Zero());
    KinematicTestTick(&world, 40);
    TEST_TRUE(KinematicTestAsleepCheck(&world, id));
    ds_BodyVelocitySet(&world.pipeline, id, V3Zero(), V3(0.0f, 0.001f, 0.0f));
    TEST_FALSE(KinematicTestAsleepCheck(&world, id));
    KinematicTestTick(&world, 120);
    TEST_FALSE(KinematicTestAsleepCheck(&world, id));

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicSetterTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    world.pipeline.config_pending.solver.gravity = V3(0.0f, 0.0f, 0.0f);
    ds_DynamicsConfigUpdatePending(&world.pipeline);

    /* a static body has no velocity to set */
    const v3 v = V3(1.0f, 0.0f, 0.0f);
    const ds_BodyId fixed = KinematicTestCubeAdd(&world, 0, V3F64(0.0, 0.0, 0.0), 1);
    ds_BodyVelocitySet(&world.pipeline, fixed, v, V3Zero());
    KinematicTestTick(&world, 1);
    const ds_TransformWorld fixed_pose = KinematicTestPose(&world, fixed);
    TEST_TRUE(fixed_pose.position.x == 0.0 && fixed_pose.position.y == 0.0 && fixed_pose.position.z == 0.0);

    /* a dynamic body at rest without gravity sleeps; the setter wakes it and writes the velocity */
    const ds_BodyId dynamic = KinematicTestCubeAdd(&world, BODY_FLAG_DYNAMIC, V3F64(50.0, 0.0, 0.0), 2);
    KinematicTestTick(&world, 40);
    TEST_TRUE(KinematicTestAsleepCheck(&world, dynamic));
    ds_BodyVelocitySet(&world.pipeline, dynamic, v, V3Zero());
    TEST_FALSE(KinematicTestAsleepCheck(&world, dynamic));
    const struct ds_Body *body = ds_BodyLookup(&world.pipeline, dynamic).address;
    const struct ds_BodyCompute *compute = world.pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf + body->sim;
    TEST_TRUE(memcmp(&compute->linear_velocity, &v, sizeof(v)) == 0);
    KinematicTestTick(&world, 1);
    TEST_TRUE(KinematicTestPose(&world, dynamic).position.x > 50.0);

    /* a dead id is ignored */
    ds_BodyRemove(&world.mem, &world.pipeline, dynamic);
    ds_BodyVelocitySet(&world.pipeline, dynamic, v, V3Zero());
    KinematicTestTick(&world, 1);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicRebuildTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    world.pipeline.config_pending.solver.gravity = V3(0.0f, 0.0f, 0.0f);
    world.pipeline.config_pending.solver.sleep_enabled = 0;
    ds_DynamicsConfigUpdatePending(&world.pipeline);

    /* four kinematic cubes, each reinserted every tick (0.5 m per tick), and no dynamic body: nothing to rebuild */
    ds_BodyId kinematic[4];
    for (u32 i = 0; i < 4; ++i)
    {
        kinematic[i] = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, V3F64(0.0, 0.0, 20.0 * (f64) i), i);
        ds_BodyVelocitySet(&world.pipeline, kinematic[i], V3(30.0f, 0.0f, 0.0f), V3Zero());
    }
    KinematicTestTick(&world, 30);
    TEST_EQUAL(BTLeafCount(world.pipeline.dynamic_bvh.bt), 0);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(KinematicTestRayFindsCheck(&world, kinematic[i]));
    }

    /* eight dynamic cubes that all reinsert every tick: the dynamic tree is rebuilt every tick, the kinematic proxies still follow */
    ds_BodyId dynamic[8];
    for (u32 i = 0; i < 8; ++i)
    {
        dynamic[i] = KinematicTestCubeAdd(&world, BODY_FLAG_DYNAMIC, V3F64(0.0, 0.0, -20.0 * (f64) (i + 1)), 4 + i);
        ds_BodyVelocitySet(&world.pipeline, dynamic[i], V3(30.0f, 0.0f, 0.0f), V3Zero());
    }
    KinematicTestTick(&world, 30);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(KinematicTestRayFindsCheck(&world, kinematic[i]));
    }

    for (u32 i = 0; i < 8; ++i)
    {
        TEST_TRUE(KinematicTestRayFindsCheck(&world, dynamic[i]));
    }
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 4);
    TEST_EQUAL(BTLeafCount(world.pipeline.dynamic_bvh.bt), 8);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicRemoveTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* awake */
    const ds_BodyId awake = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    ds_BodyVelocitySet(&world.pipeline, awake, V3(1.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 2);
    TEST_FALSE(KinematicTestAsleepCheck(&world, awake));
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 1);
    const f64 x = KinematicTestPose(&world, awake).position.x;
    ds_BodyRemove(&world.mem, &world.pipeline, awake);
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 0);
    TEST_EQUAL(KinematicTestRayDown(&world, x, 0.0).u, U32_MAX);
    KinematicTestTick(&world, 2);

    /* asleep */
    const ds_BodyId asleep = KinematicTestCubeAdd(&world, BODY_FLAG_KINEMATIC, V3F64(0.0, 0.0, 0.0), 2);
    KinematicTestTick(&world, 40);
    TEST_TRUE(KinematicTestAsleepCheck(&world, asleep));
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 1);
    ds_BodyRemove(&world.mem, &world.pipeline, asleep);
    TEST_EQUAL(BTLeafCount(world.pipeline.kinematic_bvh.bt), 0);
    TEST_EQUAL(KinematicTestRayDown(&world, 0.0, 0.0).u, U32_MAX);
    KinematicTestTick(&world, 2);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output (*kinematic_tests[])(struct test_Environment *) =
{
    KinematicMotionTest,
    KinematicNoCapTest,
    KinematicProxyTest,
    KinematicSleepTest,
    KinematicSetterTest,
    KinematicRebuildTest,
    KinematicRemoveTest,
};

struct suite_Correctness m_kinematic_suite =
{
    .id = "kinematic",
    .unit_test = kinematic_tests,
    .unit_test_count = sizeof(kinematic_tests) / sizeof(kinematic_tests[0]),
};

struct suite_Correctness *kinematic_correctness_suite = &m_kinematic_suite;

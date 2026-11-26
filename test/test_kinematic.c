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
A kinematic body moves by the velocity it was given (ds_BodyVelocitySet) and by nothing else. A body here is a unit
cube or a 4 x 0.5 x 4 m slab (the platforms and the ground); h is the tick, 1/60 s.

A kinematic body on its own:

    Motion      position and rotation after a second of a set velocity, next to a dynamic cube that falls
    NoCap       velocities above both caps are kept bit for bit
    Proxy       a ray cast finds the cube after a move its proxy absorbs in place and after one that reinserts it
    Sleep       asleep after resting, awake at 1 mm/s (below the dynamic rest speed), woken by the setter
    Setter      static bodies and dead ids are left alone; a sleeping dynamic body is woken and written
    Rebuild     moving kinematic cubes alone never rebuild the dynamic tree; with a rebuild every tick their proxies still follow
    Remove      removing a kinematic body, awake or asleep, empties the kinematic tree

Against dynamic bodies (a cube on a kinematic slab unless said otherwise):

    Carry           the cube follows a slab moving sideways (friction)
    Elevator        the cube does not sink into a rising slab, and lands on and follows a descending one
    SleepTogether   slab and cubes are one island and sleep in one set; the setter wakes all; they sleep again after the stop
    WakeOnTouch     a kinematic cube runs into a cube asleep on static ground: the pair is found from the kinematic side
    ContactRemoved  the contact of a kinematic and a dynamic cube goes when the kinematic one leaves
    NoPairs         kinematic-kinematic and kinematic-static overlaps get no contact
    Colours         the contacts of nine cubes with one slab share the first kinematic colour; a cube on two slabs takes two;
                    nine cubes on a static slab share the first static colour
    Tumbler         27 cubes in a turning kinematic box (five seconds): none leaves, and a second run hashes the same every tick
    Overflow        cubes with five contacts each on one kinematic body: the fifth goes to the serial colour; a second run hashes the same
*/

struct kinematic_TestWorld
{
    struct arena                mem;
    struct arena                tick;
    struct c_ShapeDB            cshape_db;
    struct ds_DynamicsExternal  external;
    struct ds_Dynamics          pipeline;
    u32                         cube;
    u32                         slab;
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
    const struct c_Shape slab = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(&world->mem, V3(2.0f, 0.25f, 2.0f)) };
    world->slab = c_ShapeDBAdd(&world->cshape_db, Utf8Inline("kinematic_slab"), &slab).index;
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

/* a body with one shape (world->cube or world->slab) of the given motion (DS_MOTION_STATIC, DS_MOTION_DYNAMIC or DS_MOTION_KINEMATIC) */
static ds_BodyId KinematicTestBodyAdd(struct kinematic_TestWorld *world, const u32 cshape, const u32 motion, const v3f64 position, const u32 entity)
{
    const struct ds_BodyPrefab body_prefab = { .motion = motion };
    const ds_TransformWorld t_world = { .rotation = QIdentity(), .position = position };
    const ds_BodyId id = ds_BodyAdd(&world->pipeline, &body_prefab, &t_world, entity);
    const struct ds_ShapePrefab shape_prefab = { .cshape = cshape, .density = 1.0f, .restitution = 0.0f, .friction = 0.5f };
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
    const ds_BodyId kinematic = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, start, 1);
    const ds_BodyId dynamic = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(50.0, 10.0, 0.0), 2);
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
    const ds_BodyId id = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, start, 1);
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
    const ds_BodyId near = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId far = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(100.0, 0.0, 0.0), 2);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 2);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_DYNAMIC].bt), 0);
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
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 2);

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
    const ds_BodyId id = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, start, 1);
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
    const ds_BodyId fixed = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_STATIC, V3F64(0.0, 0.0, 0.0), 1);
    ds_BodyVelocitySet(&world.pipeline, fixed, v, V3Zero());
    KinematicTestTick(&world, 1);
    const ds_TransformWorld fixed_pose = KinematicTestPose(&world, fixed);
    TEST_TRUE(fixed_pose.position.x == 0.0 && fixed_pose.position.y == 0.0 && fixed_pose.position.z == 0.0);

    /* a dynamic body at rest without gravity sleeps; the setter wakes it and writes the velocity */
    const ds_BodyId dynamic = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(50.0, 0.0, 0.0), 2);
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
        kinematic[i] = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 20.0 * (f64) i), i);
        ds_BodyVelocitySet(&world.pipeline, kinematic[i], V3(30.0f, 0.0f, 0.0f), V3Zero());
    }
    KinematicTestTick(&world, 30);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_DYNAMIC].bt), 0);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(KinematicTestRayFindsCheck(&world, kinematic[i]));
    }

    /* eight dynamic cubes that all reinsert every tick: the dynamic tree is rebuilt every tick, the kinematic proxies still follow */
    ds_BodyId dynamic[8];
    for (u32 i = 0; i < 8; ++i)
    {
        dynamic[i] = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(0.0, 0.0, -20.0 * (f64) (i + 1)), 4 + i);
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
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 4);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_DYNAMIC].bt), 8);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicRemoveTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* awake */
    const ds_BodyId awake = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    ds_BodyVelocitySet(&world.pipeline, awake, V3(1.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 2);
    TEST_FALSE(KinematicTestAsleepCheck(&world, awake));
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 1);
    const f64 x = KinematicTestPose(&world, awake).position.x;
    ds_BodyRemove(&world.mem, &world.pipeline, awake);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 0);
    TEST_EQUAL(KinematicTestRayDown(&world, x, 0.0).u, U32_MAX);
    KinematicTestTick(&world, 2);

    /* asleep */
    const ds_BodyId asleep = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 2);
    KinematicTestTick(&world, 40);
    TEST_TRUE(KinematicTestAsleepCheck(&world, asleep));
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 1);
    ds_BodyRemove(&world.mem, &world.pipeline, asleep);
    TEST_EQUAL(BTLeafCount(world.pipeline.bvh[DS_MOTION_KINEMATIC].bt), 0);
    TEST_EQUAL(KinematicTestRayDown(&world, 0.0, 0.0).u, U32_MAX);
    KinematicTestTick(&world, 2);

    KinematicTestWorldFree(&world);
    return output;
}

/* the linear velocity of a body; zero while it sleeps */
static v3 KinematicTestVelocity(const struct kinematic_TestWorld *world, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(&world->pipeline, id).address;
    if (body->set != SOLVER_SET_ACTIVE)
    {
        return V3Zero();
    }

    return world->pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf[body->sim].linear_velocity;
}

static struct test_Output KinematicCarryTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* the slab's top is at y = 0.25; the cube lands from 1 cm above it */
    const ds_BodyId slab = KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId cube = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(0.0, 0.76, 0.0), 2);
    KinematicTestTick(&world, 30);

    ds_BodyVelocitySet(&world.pipeline, slab, V3(1.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 120);

    /* 2 s at 1 m/s: the slab is 2 m on; friction brought the cube to the slab's speed within a slip of some 10 cm */
    const ds_TransformWorld slab_pose = KinematicTestPose(&world, slab);
    const ds_TransformWorld cube_pose = KinematicTestPose(&world, cube);
    const v3 cube_velocity = KinematicTestVelocity(&world, cube);
    const u32 awake = !KinematicTestAsleepCheck(&world, slab) && !KinematicTestAsleepCheck(&world, cube);
    const u32 ok = awake && slab_pose.position.x > 1.99 && slab_pose.position.x < 2.01 && cube_pose.position.x > 1.8 && cube_pose.position.x < 2.01 && cube_pose.position.y > 0.73 && cube_pose.position.y < 0.77 && cube_velocity.x > 0.95f && cube_velocity.x < 1.05f;
    if (!ok)
    {
        fprintf(stderr, "kinematic carry: awake %u, slab x %g, cube (%g, %g), cube vx %g\n", awake, slab_pose.position.x, cube_pose.position.x, cube_pose.position.y, (f64) cube_velocity.x);
        TEST_FAILURE;
    }

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicElevatorTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    const ds_BodyId slab = KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId cube = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(0.0, 0.76, 0.0), 2);
    KinematicTestTick(&world, 30);

    /* up at 2 m/s: a slab the solver took for standing still would be 3.3 cm into the cube every tick; within 1 mm here */
    ds_BodyVelocitySet(&world.pipeline, slab, V3(0.0f, 2.0f, 0.0f), V3Zero());
    f64 overlap_max = 0.0;
    for (u32 i = 0; i < 60; ++i)
    {
        KinematicTestTick(&world, 1);
        const f64 overlap = (KinematicTestPose(&world, slab).position.y + 0.25) - (KinematicTestPose(&world, cube).position.y - 0.5);
        if (overlap > overlap_max)
        {
            overlap_max = overlap;
        }
    }

    const f64 gap_up = (KinematicTestPose(&world, cube).position.y - 0.5) - (KinematicTestPose(&world, slab).position.y + 0.25);
    const v3 velocity_up = KinematicTestVelocity(&world, cube);
    const u32 up_ok = overlap_max < 0.001 && gap_up > -0.001 && gap_up < 0.001 && velocity_up.y > 1.9f && velocity_up.y < 2.1f;

    /* stop: the cube flies on (2 m/s up, 0.2 m), lands and rests. Then down at 1 m/s: it falls after the slab, lands and follows */
    ds_BodyVelocitySet(&world.pipeline, slab, V3Zero(), V3Zero());
    KinematicTestTick(&world, 60);
    ds_BodyVelocitySet(&world.pipeline, slab, V3(0.0f, -1.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 60);

    const f64 gap_down = (KinematicTestPose(&world, cube).position.y - 0.5) - (KinematicTestPose(&world, slab).position.y + 0.25);
    const v3 velocity_down = KinematicTestVelocity(&world, cube);
    const u32 down_ok = gap_down > -0.001 && gap_down < 0.001 && velocity_down.y > -1.1f && velocity_down.y < -0.9f;
    if (!up_ok || !down_ok)
    {
        fprintf(stderr, "kinematic elevator: up: deepest overlap %g, gap %g, cube vy %g; down: gap %g, cube vy %g\n", overlap_max, gap_up, (f64) velocity_up.y, gap_down, (f64) velocity_down.y);
        TEST_FAILURE;
    }

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicSleepTogetherTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* two cubes a metre apart on the slab: they touch the slab only */
    const ds_BodyId id[3] =
    {
        KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1),
        KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(-1.0, 0.76, 0.0), 2),
        KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(1.0, 0.76, 0.0), 3),
    };

    /* at rest: one island, asleep in one set */
    KinematicTestTick(&world, 60);
    const struct ds_Body *body[3] = { ds_BodyLookup(&world.pipeline, id[0]).address, ds_BodyLookup(&world.pipeline, id[1]).address, ds_BodyLookup(&world.pipeline, id[2]).address };
    TEST_TRUE(body[0]->island == body[1]->island && body[0]->island == body[2]->island);
    TEST_TRUE(body[0]->set >= SOLVER_SET_SLEEPING_FIRST && body[0]->set == body[1]->set && body[0]->set == body[2]->set);

    /* the setter wakes the slab's set: the cubes with it; nothing sleeps while the slab moves */
    ds_BodyVelocitySet(&world.pipeline, id[0], V3(1.0f, 0.0f, 0.0f), V3Zero());
    TEST_TRUE(body[0]->set == SOLVER_SET_ACTIVE && body[1]->set == SOLVER_SET_ACTIVE && body[2]->set == SOLVER_SET_ACTIVE);
    KinematicTestTick(&world, 60);
    TEST_TRUE(body[0]->set == SOLVER_SET_ACTIVE && body[1]->set == SOLVER_SET_ACTIVE && body[2]->set == SOLVER_SET_ACTIVE);

    /* stopped: the cubes slide to rest and all three sleep together again */
    ds_BodyVelocitySet(&world.pipeline, id[0], V3Zero(), V3Zero());
    KinematicTestTick(&world, 120);
    TEST_TRUE(body[0]->island == body[1]->island && body[0]->island == body[2]->island);
    TEST_TRUE(body[0]->set >= SOLVER_SET_SLEEPING_FIRST && body[0]->set == body[1]->set && body[0]->set == body[2]->set);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicWakeOnTouchTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    /* a cube asleep on static ground */
    KinematicTestBodyAdd(&world, world.slab, DS_MOTION_STATIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId cube = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(0.0, 0.76, 0.0), 2);
    KinematicTestTick(&world, 60);
    TEST_TRUE(KinematicTestAsleepCheck(&world, cube));

    /* a kinematic cube 2 m away comes at 2 m/s, just above the ground: half a second later it is still a metre off */
    const ds_BodyId pusher = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(-3.0, 0.76, 0.0), 3);
    ds_BodyVelocitySet(&world.pipeline, pusher, V3(2.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 30);
    TEST_TRUE(KinematicTestAsleepCheck(&world, cube));

    /* it arrives after 1 s. The sleeping cube's shape is not dirty, so the pair comes from the kinematic shape's query */
    KinematicTestTick(&world, 60);
    const ds_TransformWorld cube_pose = KinematicTestPose(&world, cube);
    const ds_TransformWorld pusher_pose = KinematicTestPose(&world, pusher);
    const u32 ok = !KinematicTestAsleepCheck(&world, cube) && cube_pose.position.x > 0.5 && cube_pose.position.x > pusher_pose.position.x + 0.95;
    if (!ok)
    {
        fprintf(stderr, "kinematic wake on touch: cube asleep %u, cube x %g, pusher x %g\n", KinematicTestAsleepCheck(&world, cube), cube_pose.position.x, pusher_pose.position.x);
        TEST_FAILURE;
    }

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicContactRemovedTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    world.pipeline.config_pending.solver.gravity = V3(0.0f, 0.0f, 0.0f);
    world.pipeline.config_pending.solver.sleep_enabled = 0;
    ds_DynamicsConfigUpdatePending(&world.pipeline);

    /* 3 cm apart: the proxies overlap */
    KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(0.0, 0.0, 0.0), 1);
    const ds_BodyId kinematic = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(1.03, 0.0, 0.0), 2);
    KinematicTestTick(&world, 1);
    TEST_EQUAL(world.pipeline.contact_pool.count, 1);

    ds_BodyVelocitySet(&world.pipeline, kinematic, V3(30.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 5);
    TEST_EQUAL(world.pipeline.contact_pool.count, 0);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicNoPairsTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    world.pipeline.config_pending.solver.gravity = V3(0.0f, 0.0f, 0.0f);
    world.pipeline.config_pending.solver.sleep_enabled = 0;
    ds_DynamicsConfigUpdatePending(&world.pipeline);

    /* two kinematic cubes inside each other, a static cube inside the first */
    const ds_BodyId first = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    KinematicTestBodyAdd(&world, world.cube, DS_MOTION_KINEMATIC, V3F64(0.5, 0.0, 0.0), 2);
    KinematicTestBodyAdd(&world, world.cube, DS_MOTION_STATIC, V3F64(0.0, 0.5, 0.0), 3);
    KinematicTestTick(&world, 3);
    TEST_EQUAL(world.pipeline.contact_pool.count, 0);

    /* moving through them, and a static cube added inside a kinematic one */
    ds_BodyVelocitySet(&world.pipeline, first, V3(1.0f, 0.0f, 0.0f), V3Zero());
    KinematicTestTick(&world, 10);
    KinematicTestBodyAdd(&world, world.cube, DS_MOTION_STATIC, V3F64(0.5, 0.5, 0.0), 4);
    KinematicTestTick(&world, 3);
    TEST_EQUAL(world.pipeline.contact_pool.count, 0);

    KinematicTestWorldFree(&world);
    return output;
}

static struct test_Output KinematicColoursTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);
    world.pipeline.config_pending.solver.sleep_enabled = 0;
    ds_DynamicsConfigUpdatePending(&world.pipeline);

    /* nine cubes apart from each other on one slab: a kinematic body takes no colour bit, so one colour holds all nine contacts */
    KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 0.0), 1);
    for (u32 i = 0; i < 9; ++i)
    {
        KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(-1.3 + 1.3 * (f64) (i % 3), 0.76, -1.3 + 1.3 * (f64) (i / 3)), 2 + i);
    }
    KinematicTestTick(&world, 30);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_KINEMATIC_COLOR_FIRST].contact_pool.count, 9);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_KINEMATIC_COLOR_FIRST + 1].contact_pool.count, 0);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count, 0);

    /* a cube across the seam of two more slabs: its two contacts need two kinematic colours */
    KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(0.0, 0.0, 100.0), 11);
    KinematicTestBodyAdd(&world, world.slab, DS_MOTION_KINEMATIC, V3F64(4.0, 0.0, 100.0), 12);
    KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(2.0, 0.76, 100.0), 13);
    KinematicTestTick(&world, 30);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_KINEMATIC_COLOR_FIRST].contact_pool.count, 10);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_KINEMATIC_COLOR_FIRST + 1].contact_pool.count, 1);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count, 0);
    for (u32 c = CG_DYNAMIC_COLOR_FIRST; c <= CG_DYNAMIC_COLOR_LAST; ++c)
    {
        TEST_EQUAL(world.pipeline.cgraph.color[c].contact_pool.count, 0);
    }

    /* the static colours work the same way (and share the warm-up): nine cubes on a static slab, one static colour */
    KinematicTestBodyAdd(&world, world.slab, DS_MOTION_STATIC, V3F64(0.0, 0.0, 200.0), 21);
    for (u32 i = 0; i < 9; ++i)
    {
        KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(-1.3 + 1.3 * (f64) (i % 3), 0.76, 200.0 - 1.3 + 1.3 * (f64) (i / 3)), 22 + i);
    }
    KinematicTestTick(&world, 30);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_STATIC_COLOR_FIRST].contact_pool.count, 9);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_KINEMATIC_COLOR_FIRST].contact_pool.count, 10);
    TEST_EQUAL(world.pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count, 0);

    KinematicTestWorldFree(&world);
    return output;
}

#define KINEMATIC_TUMBLER_TICKS 300

/*
 * A closed box, one kinematic body of six 5 x 0.5 x 5 m walls around a 4 m cavity, turning at 1 rad/s about z with
 * 27 cubes inside. Writes the pipeline's hash after every tick; returns the largest squared distance of a cube
 * from the box's center at the end (a cube inside is within 2.6 m, one that left beyond 3 m).
 */
static f64 KinematicTestTumblerRun(u64 hash[KINEMATIC_TUMBLER_TICKS])
{
    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    const struct c_Shape wall = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(&world.mem, V3(2.5f, 0.25f, 2.5f)) };
    const u32 wall_handle = c_ShapeDBAdd(&world.cshape_db, Utf8Inline("kinematic_tumbler_wall"), &wall).index;
    const struct ds_ShapePrefab wall_prefab = { .cshape = wall_handle, .density = 1.0f, .restitution = 0.0f, .friction = 0.5f };

    /* floor and ceiling as they are; the x walls turned a quarter about z, the z walls a quarter about x */
    const q quarter_z = QNormalize(Q(0.0f, 0.0f, 1.0f, 1.0f));
    const q quarter_x = QNormalize(Q(1.0f, 0.0f, 0.0f, 1.0f));
    const ds_Transform wall_local[6] =
    {
        { .rotation = QIdentity(), .position = V3(0.0f, -2.25f, 0.0f) },
        { .rotation = QIdentity(), .position = V3(0.0f, 2.25f, 0.0f) },
        { .rotation = quarter_z, .position = V3(-2.25f, 0.0f, 0.0f) },
        { .rotation = quarter_z, .position = V3(2.25f, 0.0f, 0.0f) },
        { .rotation = quarter_x, .position = V3(0.0f, 0.0f, -2.25f) },
        { .rotation = quarter_x, .position = V3(0.0f, 0.0f, 2.25f) },
    };

    const struct ds_BodyPrefab box_prefab = { .motion = DS_MOTION_KINEMATIC };
    const ds_TransformWorld box_world = ds_TransformWorldIdentity();
    const ds_BodyId box = ds_BodyAdd(&world.pipeline, &box_prefab, &box_world, 1);
    for (u32 i = 0; i < 6; ++i)
    {
        ds_ShapeAdd(&world.pipeline, &wall_prefab, wall_local + i, box);
    }

    ds_BodyId cube[27];
    for (u32 i = 0; i < 27; ++i)
    {
        const v3f64 position = V3F64(-1.2 + 1.2 * (f64) (i % 3), -1.2 + 1.2 * (f64) ((i / 3) % 3), -1.2 + 1.2 * (f64) (i / 9));
        cube[i] = KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, position, 2 + i);
    }

    ds_BodyVelocitySet(&world.pipeline, box, V3Zero(), V3(0.0f, 0.0f, 1.0f));
    for (u32 i = 0; i < KINEMATIC_TUMBLER_TICKS; ++i)
    {
        KinematicTestTick(&world, 1);
        hash[i] = ds_DynamicsHash(&world.pipeline);
    }

    f64 far_sq = 0.0;
    for (u32 i = 0; i < 27; ++i)
    {
        const v3f64 p = KinematicTestPose(&world, cube[i]).position;
        const f64 distance_sq = p.x*p.x + p.y*p.y + p.z*p.z;
        if (distance_sq > far_sq)
        {
            far_sq = distance_sq;
        }
    }

    KinematicTestWorldFree(&world);
    return far_sq;
}

static struct test_Output KinematicTumblerTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    static u64 hash[2][KINEMATIC_TUMBLER_TICKS];
    const f64 far_sq[2] = { KinematicTestTumblerRun(hash[0]), KinematicTestTumblerRun(hash[1]) };

    /* the same hash every tick of both runs (compare it across thread counts by hand: the line below) */
    const u32 repeats = memcmp(hash[0], hash[1], sizeof(hash[0])) == 0;
    fprintf(stderr, "kinematic tumbler: %u ticks, final hash %016llx, farthest cube %g m\n", KINEMATIC_TUMBLER_TICKS, (unsigned long long) hash[0][KINEMATIC_TUMBLER_TICKS - 1], (f64) F32Sqrt((f32) far_sq[0]));
    if (!repeats || far_sq[0] >= 2.8*2.8 || far_sq[1] >= 2.8*2.8)
    {
        fprintf(stderr, "kinematic tumbler: second run repeats %u, farthest cube^2 %g and %g (inside: < %g)\n", repeats, far_sq[0], far_sq[1], 2.8*2.8);
        TEST_FAILURE;
    }

    return output;
}

#define KINEMATIC_OVERFLOW_TICKS    120
#define KINEMATIC_OVERFLOW_POCKETS  12

/*
 * One kinematic body of 12 pockets, each a floor cube and four wall cubes 1 cm from the dynamic cube inside (within
 * the speculative distance): five kinematic contacts per dynamic cube, one more than there are kinematic colours.
 * The body moves and turns. Writes the pipeline's hash after every tick; returns the most contacts the serial colour
 * held.
 */
static u32 KinematicTestOverflowRun(u64 hash[KINEMATIC_OVERFLOW_TICKS])
{
    struct kinematic_TestWorld world;
    KinematicTestWorldInit(&world);

    const struct ds_ShapePrefab cube_prefab = { .cshape = world.cube, .density = 1.0f, .restitution = 0.0f, .friction = 0.5f };
    const struct ds_BodyPrefab tray_prefab = { .motion = DS_MOTION_KINEMATIC };
    const ds_TransformWorld tray_world = ds_TransformWorldIdentity();
    const ds_BodyId tray = ds_BodyAdd(&world.pipeline, &tray_prefab, &tray_world, 1);
    for (u32 p = 0; p < KINEMATIC_OVERFLOW_POCKETS; ++p)
    {
        const f32 x = 3.5f * (f32) p;
        const ds_Transform local[5] =
        {
            { .rotation = QIdentity(), .position = V3(x, -1.0f, 0.0f) },
            { .rotation = QIdentity(), .position = V3(x - 1.01f, 0.0f, 0.0f) },
            { .rotation = QIdentity(), .position = V3(x + 1.01f, 0.0f, 0.0f) },
            { .rotation = QIdentity(), .position = V3(x, 0.0f, -1.01f) },
            { .rotation = QIdentity(), .position = V3(x, 0.0f, 1.01f) },
        };
        for (u32 i = 0; i < 5; ++i)
        {
            ds_ShapeAdd(&world.pipeline, &cube_prefab, local + i, tray);
        }

        KinematicTestBodyAdd(&world, world.cube, DS_MOTION_DYNAMIC, V3F64(3.5 * (f64) p, 0.005, 0.0), 2 + p);
    }

    ds_BodyVelocitySet(&world.pipeline, tray, V3(0.3f, 0.0f, 0.0f), V3(0.0f, 0.05f, 0.0f));
    u32 serial_max = 0;
    for (u32 i = 0; i < KINEMATIC_OVERFLOW_TICKS; ++i)
    {
        KinematicTestTick(&world, 1);
        hash[i] = ds_DynamicsHash(&world.pipeline);
        const u32 serial = world.pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count;
        if (serial > serial_max)
        {
            serial_max = serial;
        }
    }

    KinematicTestWorldFree(&world);
    return serial_max;
}

static struct test_Output KinematicOverflowTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    static u64 hash[2][KINEMATIC_OVERFLOW_TICKS];
    const u32 serial_max[2] = { KinematicTestOverflowRun(hash[0]), KinematicTestOverflowRun(hash[1]) };

    /* the serial colour did hold kinematic contacts, and both runs hash the same every tick (across thread counts: the line below) */
    const u32 repeats = memcmp(hash[0], hash[1], sizeof(hash[0])) == 0;
    fprintf(stderr, "kinematic overflow: %u ticks, final hash %016llx, most serial contacts %u\n", KINEMATIC_OVERFLOW_TICKS, (unsigned long long) hash[0][KINEMATIC_OVERFLOW_TICKS - 1], serial_max[0]);
    if (!repeats || serial_max[0] == 0 || serial_max[1] != serial_max[0])
    {
        fprintf(stderr, "kinematic overflow: second run repeats %u, most serial contacts %u and %u\n", repeats, serial_max[0], serial_max[1]);
        TEST_FAILURE;
    }

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
    KinematicCarryTest,
    KinematicElevatorTest,
    KinematicSleepTogetherTest,
    KinematicWakeOnTouchTest,
    KinematicContactRemovedTest,
    KinematicNoPairsTest,
    KinematicColoursTest,
    KinematicTumblerTest,
    KinematicOverflowTest,
};

struct suite_Correctness m_kinematic_suite =
{
    .id = "kinematic",
    .unit_test = kinematic_tests,
    .unit_test_count = sizeof(kinematic_tests) / sizeof(kinematic_tests[0]),
};

struct suite_Correctness *kinematic_correctness_suite = &m_kinematic_suite;

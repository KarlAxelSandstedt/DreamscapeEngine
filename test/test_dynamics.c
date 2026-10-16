#include "ds_test.h"
#include "ds_init.h"
#include "ds_dynamics.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "collision.h"
#include "ds_geometry.h"

/*
Dynamics tests
==============
Resting scenes built around several world origins (agent_notes/f64_positions.txt P7c). Sleeping is
disabled, so every body is solved every tick; once settled, nothing should move. Poses are measured
relative to the scene origin, in f64:

    sink    how far a body settled below its placement along the support normal: the resting
            penetration of the contacts below it (the solver steers towards linear_slop, 5 mm, per
            contact)
    drift   max |p(t) - p(s)| and max angle(q(t), q(s)) over the measured ticks, s = the first one
    jitter  max |p(t) - p(t - 1)| over the measured ticks

Checks, at every origin: sink, drift and jitter within their bounds, and the final relative poses within
DYNAMICS_TEST_ORIGIN_TOLERANCE (position) and DYNAMICS_TEST_ORIGIN_ANGLE (rotation) of those at origin 0:
the simulation must not depend on where the scene is. (With f32 world positions, the representable
positions alone were 1 mm apart at 10 km.)

The bounds come from measurements (2026-10-15, GCC, DS_DEBUG; worst over the origins):

    scene   sink        drift                   jitter      vs origin 0
    box     1.5 mm      1.1e-8 m, 2.2e-8 rad    2.0e-10 m   8.5e-10 m, 0 rad
    stack   4.8 mm      5.2e-4 m, 1.7e-4 rad    6.7e-5 m    2.4e-9 m, 0 rad
    slope   1.2 mm      3.6e-9 m, 2.5e-9 rad    2.0e-10 m   3.4e-9 m, 0 rad

The four-box stack still creeps after settling (the same at every origin): its bounds guard against
regressions, they are not a goal. Rounding the body positions to f32 after every step (as with f32
storage) fails all three: 1 mm off at 10 km, up to 6 cm (and a stack drifting 8 cm) at 100-1000 km.

g_solver_config is shared by all pipelines, and each tick applies its pending_sleep_enabled (a direct
ds_DynamicsSleepDisable would be undone): a run sets pending_sleep_enabled = 0, and before it returns sets it
back and enables sleeping again.

Snapshot round trip (DynamicsSnapshotTest; agent_notes/physics_serialization_design.txt): a scene with seven
islands that fall asleep at different frames is snapshotted at several frames. Each snapshot must
    - be as large as ds_DynamicsSerializeSize predicts;
    - fail to restore from every truncated stream, leaving the stream position and the arena as they were;
    - restore twice, both copies equal to the original right away and after each of the next
      SNAPSHOT_TEST_COMPARE_TICKS ticks, ticked side by side (frame count, pool counts, orientation hash, and
      every body's id, set, sim slot, island, sleep timer and pose, bit for bit);
    - re-serialize from a copy to the same size.
A lone box is removed before one check (pools with free slots), and the last check first wakes every sleeping
set in the original and in the copies. A restore points the dynamics globals at the copy, so the test points
them back at the original (ds_DynamicsSetGlobals) before calling into it outside a tick.
*/

#define DYNAMICS_TEST_SETTLE_TICKS      120     /* 2 s at 60 Hz */
#define DYNAMICS_TEST_MEASURE_TICKS     480
#define DYNAMICS_TEST_BOX_MAX           8

#define DYNAMICS_TEST_SINK_PER_CONTACT  0.005   /* m: linear_slop */
#define DYNAMICS_TEST_ORIGIN_TOLERANCE  1e-6    /* m */
#define DYNAMICS_TEST_ORIGIN_ANGLE      1e-6f   /* rad */

struct dynamics_TestBounds
{
    f64     drift;          /* m */
    f32     drift_angle;    /* rad */
    f64     jitter;         /* m */
};

/* resting bodies that should not move: about 100 times the measured values */
static const struct dynamics_TestBounds dynamics_test_bounds_rest = { .drift = 1e-6, .drift_angle = 1e-6f, .jitter = 1e-7 };
/* the four-box stack: about 4 times the measured creep */
static const struct dynamics_TestBounds dynamics_test_bounds_stack = { .drift = 2e-3, .drift_angle = 1e-3f, .jitter = 2e-4 };

struct dynamics_TestBox
{
    v3      position;   /* relative to the scene origin */
    q       rotation;
    v3      hw;
    u32     dynamic;
    f32     friction;
};

struct dynamics_TestScene
{
    struct dynamics_TestBox box[DYNAMICS_TEST_BOX_MAX];
    u32                     box_count;
    v3                      support_normal;     /* sink is measured along it */
};

struct dynamics_TestResult
{
    v3f64   position[DYNAMICS_TEST_BOX_MAX];    /* final, relative to the scene origin */
    q       rotation[DYNAMICS_TEST_BOX_MAX];
    f64     sink[DYNAMICS_TEST_BOX_MAX];
    f64     drift_sq;                           /* maxima over the dynamic bodies */
    f32     drift_angle;
    f64     jitter_sq;
};

static const v3f64 dynamics_test_origin[] =
{
    { .x = 0.0,         .y = 0.0,       .z = 0.0 },
    { .x = 1e4,         .y = 1e4,       .z = 1e4 },
    { .x = 1e5,         .y = 1e5,       .z = 1e5 },
    { .x = 1e6,         .y = -1e6,      .z = 1e6 },
    { .x = -10000.5,    .y = 5000.25,   .z = -12345.678 },
};

static f64 DynamicsTestDistanceSquared(const v3f64 a, const v3f64 b)
{
    const v3f64 d = V3F64Sub(a, b);
    return d.x*d.x + d.y*d.y + d.z*d.z;
}

/* small angles: 2 |vector part of a^-1 b| */
static f32 DynamicsTestAngle(const q a, const q b)
{
    const q r = QMul(QConj(a), b);
    return 2.0f * V3Length(V3(r.x, r.y, r.z));
}

static ds_TransformWorld DynamicsTestPose(const struct ds_Dynamics *pipeline, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(pipeline, id).address;
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    return set->body_sim_pool.buf[body->sim].world;
}

/* Add box as a body with its own collision shape at origin + box->position; index is unique per cshape_db. */
static ds_BodyId DynamicsTestBoxAdd(struct arena *mem, struct ds_Dynamics *pipeline, ds_SDB(c_Shape) *cshape_db, const struct dynamics_TestBox *box, const v3f64 origin, const u32 index)
{
    const utf8 id = Utf8Format(mem, "box%u", index);
    const struct slot slot = ds_SDBAddAlias(*cshape_db, id);
    struct c_Shape *cshape = slot.address;
    cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
    cshape->type = C_SHAPE_CONVEX_HULL;
    cshape->hull = DcelBox(mem, box->hw);
    c_ShapeUpdateMassProperties(cshape);

    const struct ds_BodyPrefab body_prefab = { .dynamic = box->dynamic };
    const ds_TransformWorld t_world = { .rotation = box->rotation, .position = V3F64Add(origin, V3Promote(box->position)) };
    const ds_BodyId body = ds_BodyAdd(pipeline, &body_prefab, &t_world, index);

    const struct ds_ShapePrefab shape_prefab =
    {
        .cshape = slot.index,
        .density = 1.0f,
        .restitution = 0.0f,
        .friction = box->friction,
        .margin = 0.5f,
    };
    const ds_Transform t_local = ds_TransformIdentity();
    ds_ShapeAdd(pipeline, &shape_prefab, &t_local, body);
    return body;
}

static void DynamicsTestRun(struct dynamics_TestResult *result, const struct dynamics_TestScene *scene, const v3f64 origin)
{
    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    ds_BodyId body[DYNAMICS_TEST_BOX_MAX];
    for (u32 i = 0; i < scene->box_count; ++i)
    {
        body[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene->box + i, origin, i);
    }

    ds_TransformWorld previous[DYNAMICS_TEST_BOX_MAX];
    ds_TransformWorld first[DYNAMICS_TEST_BOX_MAX];
    result->drift_sq = 0.0;
    result->drift_angle = 0.0f;
    result->jitter_sq = 0.0;
    for (u32 t = 0; t < DYNAMICS_TEST_SETTLE_TICKS + DYNAMICS_TEST_MEASURE_TICKS; ++t)
    {
        ds_DynamicsTick(&pipeline);
        ds_PoolFlush(pipeline.event_pool);
        DLLFlush(pipeline.event_list);

        if (t < DYNAMICS_TEST_SETTLE_TICKS)
        {
            continue;
        }

        for (u32 i = 0; i < scene->box_count; ++i)
        {
            const ds_TransformWorld pose = DynamicsTestPose(&pipeline, body[i]);
            if (t == DYNAMICS_TEST_SETTLE_TICKS)
            {
                first[i] = pose;
            }
            else if (scene->box[i].dynamic)
            {
                const f64 drift_sq = DynamicsTestDistanceSquared(pose.position, first[i].position);
                const f64 jitter_sq = DynamicsTestDistanceSquared(pose.position, previous[i].position);
                const f32 drift_angle = DynamicsTestAngle(first[i].rotation, pose.rotation);
                result->drift_sq = (result->drift_sq < drift_sq) ? drift_sq : result->drift_sq;
                result->jitter_sq = (result->jitter_sq < jitter_sq) ? jitter_sq : result->jitter_sq;
                result->drift_angle = F32Max(result->drift_angle, drift_angle);
            }
            previous[i] = pose;
        }
    }

    for (u32 i = 0; i < scene->box_count; ++i)
    {
        result->position[i] = V3F64Sub(previous[i].position, origin);
        result->rotation[i] = previous[i].rotation;
        const v3f64 settled = V3F64Sub(V3Promote(scene->box[i].position), result->position[i]);
        const v3 n = scene->support_normal;
        result->sink[i] = settled.x*n.x + settled.y*n.y + settled.z*n.z;
    }

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsSleepEnable(&pipeline);
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(prefab_db);
    ds_SDBDealloc(cshape_db);
    ArenaFree(&mem);
}

/*
 * Run scene at every origin: each run's metrics within bounds (contacts_below[i]: contacts in body i's
 * load path, for its sink bound) and its final relative poses within the origin tolerances of origin 0's.
 */
static u32 DynamicsTestScene(const char *name, const struct dynamics_TestScene *scene, const u32 contacts_below[], const struct dynamics_TestBounds *bounds)
{
    struct dynamics_TestResult reference;
    const u32 origin_count = sizeof(dynamics_test_origin) / sizeof(dynamics_test_origin[0]);
    u32 success = 1;
    for (u32 o = 0; o < origin_count; ++o)
    {
        struct dynamics_TestResult result;
        DynamicsTestRun(&result, scene, dynamics_test_origin[o]);
        if (o == 0)
        {
            reference = result;
        }

        f64 sink_max = 0.0;
        f64 origin_sq = 0.0;
        f32 origin_angle = 0.0f;
        for (u32 i = 0; i < scene->box_count; ++i)
        {
            if (!scene->box[i].dynamic)
            {
                continue;
            }

            sink_max = (sink_max < result.sink[i]) ? result.sink[i] : sink_max;
            if (result.sink[i] < -DYNAMICS_TEST_SINK_PER_CONTACT || result.sink[i] > contacts_below[i] * DYNAMICS_TEST_SINK_PER_CONTACT)
            {
                success = 0;
            }

            const f64 d_sq = DynamicsTestDistanceSquared(result.position[i], reference.position[i]);
            const f32 d_angle = DynamicsTestAngle(reference.rotation[i], result.rotation[i]);
            origin_sq = (origin_sq < d_sq) ? d_sq : origin_sq;
            origin_angle = F32Max(origin_angle, d_angle);
        }

        if (result.drift_sq > bounds->drift*bounds->drift
            || result.drift_angle > bounds->drift_angle
            || result.jitter_sq > bounds->jitter*bounds->jitter
            || origin_sq > DYNAMICS_TEST_ORIGIN_TOLERANCE*DYNAMICS_TEST_ORIGIN_TOLERANCE
            || origin_angle > DYNAMICS_TEST_ORIGIN_ANGLE)
        {
            success = 0;
        }

        fprintf(stdout, "  %-6s origin (%.6g, %.6g, %.6g): sink max %.3e  drift %.3e m %.3e rad  jitter %.3e  vs origin 0: %.3e m %.3e rad\n",
            name, dynamics_test_origin[o].x, dynamics_test_origin[o].y, dynamics_test_origin[o].z,
            sink_max, F32Sqrt((f32) result.drift_sq), result.drift_angle, F32Sqrt((f32) result.jitter_sq),
            F32Sqrt((f32) origin_sq), origin_angle);
    }

    return success;
}

/* a box resting on a static floor */
static struct test_Output DynamicsRestingBoxTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct dynamics_TestScene scene = { .box_count = 2, .support_normal = V3(0.0f, 1.0f, 0.0f) };
    scene.box[0] = (struct dynamics_TestBox) { .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(5.0f, 0.5f, 5.0f), .dynamic = 0, .friction = 0.8f };
    scene.box[1] = (struct dynamics_TestBox) { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 1.0f };
    const u32 contacts_below[] = { 0, 1 };

    TEST_TRUE(DynamicsTestScene("box", &scene, contacts_below, &dynamics_test_bounds_rest));

	return output;
}

/* a stack of four boxes on a static floor */
static struct test_Output DynamicsRestingStackTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct dynamics_TestScene scene = { .box_count = 5, .support_normal = V3(0.0f, 1.0f, 0.0f) };
    scene.box[0] = (struct dynamics_TestBox) { .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(5.0f, 0.5f, 5.0f), .dynamic = 0, .friction = 0.8f };
    u32 contacts_below[5] = { 0 };
    for (u32 i = 1; i < scene.box_count; ++i)
    {
        scene.box[i] = (struct dynamics_TestBox) { .position = V3(0.0f, (f32) i - 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 1.0f };
        contacts_below[i] = i;
    }

    TEST_TRUE(DynamicsTestScene("stack", &scene, contacts_below, &dynamics_test_bounds_stack));

	return output;
}

/* a box held by static friction on a 15 degree slope (tan 15 deg ~ 0.27 < friction) */
static struct test_Output DynamicsRestingSlopeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const q slope = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 15.0f * F32_PI / 180.0f);
    const v3 n = QUnitRotate(slope, V3(0.0f, 1.0f, 0.0f));
    struct dynamics_TestScene scene = { .box_count = 2, .support_normal = n };
    /* the slope's top face passes through the scene origin */
    scene.box[0] = (struct dynamics_TestBox) { .position = V3Scale(n, -0.5f), .rotation = slope, .hw = V3(5.0f, 0.5f, 5.0f), .dynamic = 0, .friction = 0.8f };
    scene.box[1] = (struct dynamics_TestBox) { .position = V3Scale(n, 0.5f), .rotation = slope, .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 1.0f };
    const u32 contacts_below[] = { 0, 1 };

    TEST_TRUE(DynamicsTestScene("slope", &scene, contacts_below, &dynamics_test_bounds_rest));

	return output;
}

#define SNAPSHOT_TEST_BOX_MAX           32
#define SNAPSHOT_TEST_COPIES            2
#define SNAPSHOT_TEST_COMPARE_TICKS     30
#define SNAPSHOT_TEST_CUTS              32      /* truncations spread over the stream, besides the short ones */
#define SNAPSHOT_TEST_REMOVE_FRAME      120     /* the last box (a lone one) is removed before this check */

/* frames of the checks; each check ticks the original SNAPSHOT_TEST_COMPARE_TICKS frames */
static const u64 snapshot_test_frame[] = { 0, 30, 60, 90, 120, 180, 240, 300, 360 };

/* stream lengths within and just past the header */
static const u64 snapshot_test_cut_short[] = { 0, 1, 8, 16, 28, 36, 44, 64, 96, 128 };

/*
 * A static floor, a six-layer pyramid, a two-box stack, a tumbling box and four lone boxes: seven islands that
 * fall asleep at different frames. Returns the box count.
 */
static u32 SnapshotTestScene(struct dynamics_TestBox box[SNAPSHOT_TEST_BOX_MAX])
{
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    u32 count = 0;
    box[count++] = (struct dynamics_TestBox) { .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(20.0f, 0.5f, 20.0f), .dynamic = 0, .friction = 0.8f };

    /* each box rests on the two below it, 5 cm gaps within a row */
    for (u32 layer = 0; layer < 6; ++layer)
    {
        const u32 row = 6 - layer;
        for (u32 j = 0; j < row; ++j)
        {
            const f32 x = 1.05f * ((f32) j - 0.5f * (f32) (row - 1));
            box[count++] = (struct dynamics_TestBox) { .position = V3(x, 0.5f + (f32) layer, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.8f };
        }
    }

    box[count++] = (struct dynamics_TestBox) { .position = V3(0.0f, 0.5f, 8.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.8f };
    box[count++] = (struct dynamics_TestBox) { .position = V3(0.0f, 1.5f, 8.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.8f };
    box[count++] = (struct dynamics_TestBox) { .position = V3(8.0f, 2.0f, 0.0f), .rotation = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.5f), .hw = hw, .dynamic = 1, .friction = 0.8f };

    const f32 lone[4][2] = { { -8.0f, -8.0f }, { 8.0f, -8.0f }, { -8.0f, 8.0f }, { 8.0f, 8.0f } };
    for (u32 i = 0; i < 4; ++i)
    {
        box[count++] = (struct dynamics_TestBox) { .position = V3(lone[i][0], 0.55f, lone[i][1]), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.8f };
    }

    ds_Assert(count <= SNAPSHOT_TEST_BOX_MAX);
    return count;
}

static void SnapshotTestTick(struct ds_Dynamics *pipeline)
{
    ds_DynamicsTick(pipeline);
    ds_PoolFlush(pipeline->event_pool);
    DLLFlush(pipeline->event_list);
}

static u32 SnapshotTestSleepingSetCount(const struct ds_Dynamics *pipeline)
{
    u32 count = 0;
    for (u32 i = SOLVER_SET_SLEEPING_FIRST; i < pipeline->solver_set_pool.count_max; ++i)
    {
        count += ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i);
    }
    return count;
}

/* Wake every sleeping set in index order; returns how many there were. */
static u32 SnapshotTestWakeAll(struct ds_Dynamics *pipeline)
{
    ds_DynamicsSetGlobals(pipeline);
    u32 count = 0;
    for (u32 i = SOLVER_SET_SLEEPING_FIRST; i < pipeline->solver_set_pool.count_max; ++i)
    {
        if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i))
        {
            ds_SolverSetWakeUp(pipeline, i);
            count += 1;
        }
    }
    return count;
}

/*
 * Bit for bit: frame count, pool counts, the orientation hash, and every body's id, set, sim slot, island, sleep
 * timer and simulated pose.
 */
static u32 SnapshotTestEqual(const struct ds_Dynamics *a, const struct ds_Dynamics *b)
{
    if (a->frames_completed != b->frames_completed
        || a->body_pool.count_max != b->body_pool.count_max
        || a->body_pool.count != b->body_pool.count
        || a->shape_pool.count != b->shape_pool.count
        || a->contact_pool.count != b->contact_pool.count
        || a->island_pool.count != b->island_pool.count
        || a->solver_set_pool.count != b->solver_set_pool.count
        || ds_DynamicsOrientationHash(a) != ds_DynamicsOrientationHash(b))
    {
        return 0;
    }

    for (u32 i = 0; i < a->body_pool.count_max; ++i)
    {
        const struct ds_Body *body_a = a->body_pool.buf + i;
        const struct ds_Body *body_b = b->body_pool.buf + i;
        if (ds_PoolSlotAllocated(body_a) != ds_PoolSlotAllocated(body_b))
        {
            return 0;
        }
        if (!ds_PoolSlotAllocated(body_a))
        {
            continue;
        }

        if (body_a->id != body_b->id || body_a->set != body_b->set || body_a->sim != body_b->sim || body_a->island != body_b->island
            || memcmp(&body_a->low_velocity_time, &body_b->low_velocity_time, sizeof(f32)) != 0)
        {
            return 0;
        }

        const struct ds_BodySim *sim_a = a->solver_set_pool.buf[body_a->set].body_sim_pool.buf + body_a->sim;
        const struct ds_BodySim *sim_b = b->solver_set_pool.buf[body_b->set].body_sim_pool.buf + body_b->sim;
        if (memcmp(&sim_a->world.position, &sim_b->world.position, sizeof(v3f64)) != 0
            || memcmp(&sim_a->world.rotation, &sim_b->world.rotation, sizeof(q)) != 0
            || memcmp(&sim_a->world_center_of_mass, &sim_b->world_center_of_mass, sizeof(v3f64)) != 0)
        {
            return 0;
        }
    }

    return 1;
}

/*
 * Snapshot original and check the round trip (file comment); with wake, every sleeping set is first woken in the
 * original and in each copy. Ticks the original SNAPSHOT_TEST_COMPARE_TICKS frames. Returns 1 if all checks pass,
 * else prints what failed.
 */
static u32 SnapshotTestCheck(struct arena *mem, struct ds_Dynamics *original, ds_SDB(c_Shape) *cshape_db, const u32 wake)
{
    ArenaPushRecord(mem);
    u32 success = 1;
    const unsigned long long frame = original->frames_completed;
    const u32 sleeping = SnapshotTestSleepingSetCount(original);

    const u64 size = ds_DynamicsSerializeSize(original);
    struct ss snap = ss_Alloc(mem, size);
    ds_DynamicsSerialize(&snap, original);
    if (snap.bit_index != 8*size)
    {
        fprintf(stdout, "    frame %llu: serialized %llu bits, predicted %llu\n", frame, (unsigned long long) snap.bit_index, (unsigned long long) (8*size));
        success = 0;
    }

    /* every truncated stream fails and leaves the stream position and the arena as they were */
    const u32 cut_short_count = sizeof(snapshot_test_cut_short) / sizeof(snapshot_test_cut_short[0]);
    u32 cut_count = 0;
    for (u32 k = 0; k < cut_short_count + SNAPSHOT_TEST_CUTS; ++k)
    {
        const u64 cut = (k < cut_short_count)
            ? snapshot_test_cut_short[k]
            : (size - 1) * (k - cut_short_count + 1) / SNAPSHOT_TEST_CUTS;
        if (cut >= size)
        {
            continue;
        }

        struct ss view = snap;
        view.bit_index = 0;
        view.bit_count = 8*cut;
        const u64 mem_left = mem->mem_left;
        struct ds_Dynamics copy;
        const u32 restored = ds_DynamicsTryDeserialize(mem, &view, &copy, cshape_db);
        if (restored)
        {
            ds_DynamicsFree(&copy);
        }
        if (restored || view.bit_index != 0 || mem->mem_left != mem_left)
        {
            fprintf(stdout, "    frame %llu: a stream cut at %llu of %llu bytes: restored %u, stream at bit %llu, arena %lld bytes off\n",
                frame, (unsigned long long) cut, (unsigned long long) size, restored, (unsigned long long) view.bit_index,
                (long long) (mem_left - mem->mem_left));
            success = 0;
        }
        cut_count += 1;
    }

    /* each copy equals the original as restored */
    struct ds_Dynamics copy[SNAPSHOT_TEST_COPIES];
    u32 copy_count = 0;
    for (u32 c = 0; c < SNAPSHOT_TEST_COPIES; ++c)
    {
        snap.bit_index = 0;
        if (!ds_DynamicsTryDeserialize(mem, &snap, copy + copy_count, cshape_db))
        {
            fprintf(stdout, "    frame %llu: restore %u failed\n", frame, c);
            success = 0;
            continue;
        }

        copy_count += 1;
        if (snap.bit_index != 8*size || !SnapshotTestEqual(copy + copy_count - 1, original))
        {
            fprintf(stdout, "    frame %llu: restore %u read %llu of %llu bits or differs from the original\n",
                frame, c, (unsigned long long) snap.bit_index, (unsigned long long) (8*size));
            success = 0;
        }
    }
    ds_DynamicsSetGlobals(original);

    if (copy_count && ds_DynamicsSerializeSize(copy) != size)
    {
        fprintf(stdout, "    frame %llu: a copy serializes to %llu bytes, the original to %llu\n",
            frame, (unsigned long long) ds_DynamicsSerializeSize(copy), (unsigned long long) size);
        success = 0;
    }

    if (wake)
    {
        const u32 woken = SnapshotTestWakeAll(original);
        for (u32 c = 0; c < copy_count; ++c)
        {
            if (SnapshotTestWakeAll(copy + c) != woken)
            {
                fprintf(stdout, "    frame %llu: copy %u has another number of sleeping sets than the original (%u)\n", frame, c, woken);
                success = 0;
            }
        }
        if (woken == 0)
        {
            fprintf(stdout, "    frame %llu: no sleeping set to wake\n", frame);
            success = 0;
        }
    }

    /* side by side */
    u32 differs = 0;
    for (u32 t = 0; t < SNAPSHOT_TEST_COMPARE_TICKS; ++t)
    {
        SnapshotTestTick(original);
        for (u32 c = 0; c < copy_count; ++c)
        {
            SnapshotTestTick(copy + c);
            if (!differs && !SnapshotTestEqual(copy + c, original))
            {
                fprintf(stdout, "    frame %llu: copy %u differs from the original %u ticks after the restore\n", frame, c, t + 1);
                differs = 1;
                success = 0;
            }
        }
    }

    for (u32 c = 0; c < copy_count; ++c)
    {
        ds_DynamicsFree(copy + c);
    }
    ds_DynamicsSetGlobals(original);
    ArenaPopRecord(mem);

    fprintf(stdout, "  snapshot frame %3llu%s: %7llu B, %2u sleeping sets; %u truncated streams rejected, %u copies over %u ticks: %s\n",
        frame, (wake) ? " (all woken)" : "", (unsigned long long) size, sleeping, cut_count, copy_count,
        SNAPSHOT_TEST_COMPARE_TICKS, (success) ? "equal" : "FAILED");
    return success;
}

static u32 SnapshotTestRun(void)
{
    struct arena mem = ArenaAlloc(NULL, 128*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 64, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 1;

    struct dynamics_TestBox box[SNAPSHOT_TEST_BOX_MAX];
    const u32 box_count = SnapshotTestScene(box);
    ds_BodyId body[SNAPSHOT_TEST_BOX_MAX];
    for (u32 i = 0; i < box_count; ++i)
    {
        body[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, box + i, dynamics_test_origin[0], i);
    }

    /* the check after the last listed frame wakes every sleeping set */
    u32 success = 1;
    const u32 check_count = sizeof(snapshot_test_frame) / sizeof(snapshot_test_frame[0]);
    for (u32 k = 0; k <= check_count; ++k)
    {
        const u64 target = (k < check_count) ? snapshot_test_frame[k] : pipeline.frames_completed;
        while (pipeline.frames_completed < target)
        {
            SnapshotTestTick(&pipeline);
        }

        if (k < check_count && target == SNAPSHOT_TEST_REMOVE_FRAME)
        {
            ArenaPushRecord(&mem);
            ds_BodyRemove(&mem, &pipeline, body[box_count - 1]);
            ArenaPopRecord(&mem);
        }

        success &= SnapshotTestCheck(&mem, &pipeline, &cshape_db, k == check_count);
    }

    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(prefab_db);
    ds_SDBDealloc(cshape_db);
    ArenaFree(&mem);
    return success;
}

/* snapshot round trips of a scene with sleeping islands (file comment) */
static struct test_Output DynamicsSnapshotTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    TEST_TRUE(SnapshotTestRun());

	return output;
}

static struct test_Output (*dynamics_tests[])(struct test_Environment *) =
{
    DynamicsRestingBoxTest,
    DynamicsRestingStackTest,
    DynamicsRestingSlopeTest,
    DynamicsSnapshotTest,
};

struct suite_Correctness m_dynamics_suite =
{
	.id = "dynamics",
	.unit_test = dynamics_tests,
	.unit_test_count = sizeof(dynamics_tests) / sizeof(dynamics_tests[0]),
};

struct suite_Correctness *dynamics_correctness_suite = &m_dynamics_suite;

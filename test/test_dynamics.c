#include "ds_test.h"
#include "ds_init.h"
#include "ds_dynamics.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_collision.h"
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

Solver sets (DynamicsSleepTest): a two-box stack and a lone box fall asleep into one set each (one island, the
bodies' simulation state without compute state, the contacts with their caches, all in the set's arena; the
active set empty). Waking a fixed set or the null index does nothing. Removing the top box wakes the stack's
set (slot freed, the bottom box active with compute state and a reset sleep timer, the lone box still asleep);
the bottom box sleeps again into the freed slot; disabling sleeping wakes everything.

Constraint graph (DynamicsGraphTest): a three-box row and a slab carrying nine boxes, more than the dynamic
colours. After a tick every contact is coloured and the invariants hold (GraphTestCheck: static colours hold
static-dynamic contacts, dynamic colours dynamic-dynamic ones, one contact per dynamic body and colour, a
colour's body bits mark exactly its dynamic bodies), the slab fills every dynamic colour and the rest of its
contacts are serial. Removing the slab clears its bits and keeps the rest coloured; once the small boxes
have landed on the ground, only the row's two contacts are in dynamic colours and none is serial.

Joints (DynamicsJointTest; the finished bookkeeping only, agent_notes/joint_design.txt has the rest): a
static-dynamic and a dynamic-dynamic joint found by id and not by a dead generation or the null id, in both
bodies' lists, coloured static and dynamic with their sims pointing back and the local frames copied; a removal
by id empties the lists, the colour and the lookup, a second one is a no-op; a reused slot gets a new
generation; a body's removal takes its joints. No tick runs with a joint alive.

Joints and islands (DynamicsJointIslandTest): three resting boxes in their own islands. A joint merges two of
them (the joint in the merged island's list), a second joint merges the third; removing the second marks the
island and the split, once it rests, keeps the still jointed pair together; removing the last joint splits the
pair. With sleeping on, the jointed pair's set carries the joint (its sim in the set's pool, no colour); a joint
to the sleeping third box wakes both sets and merges; removing a sleeping jointed body takes its joints and
wakes the rest. Each step ticks under the debug validation.

Joint collision filter (DynamicsJointCollisionTest): a box stacked on a box. A joint with collide_connected = 0
removes their contact at once without counting a constraint removal, ds_BodyPairMayCollide says 0, and ticks do
not bring the contact back; a second joint with the flag 1 changes nothing (0 wins); removing the suppressing
joint dirties the shapes and the next tick recreates the contact; a joint with the flag 1 alone leaves the
contact in place. Ground contacts stay throughout.

Pendulum (DynamicsPendulumTest): a box on a rigid distance joint to a static pivot, both frames at the
centres of mass, released 0.1 rad off the vertical, 10 s of swing. The length holds to linear_slop, the bob
neither spins nor leaves the swing plane, the period from the interpolated zero crossings matches
2 pi sqrt(L / g) within 1 %, and the swing maxima decay as the solver's velocity damping predicts (sqrt of the
per-tick factor per tick) within 0.02.

Slack rope (DynamicsSlackRopeTest): a box 2 m below a static anchor on a range joint [1, 3] m, both frames at
the centres of mass, thrown upwards at 6 m/s. It stops at range.low, falls freely, stops at range.high and hangs
there. The distance follows the one-dimensional model (gravity and damping, then the separating velocity clamped
so a bound is reached exactly at the end of the tick) within 1 mm, both bounds are reached and never violated
beyond linear_slop, the box rests at range.high for the last second, and it neither spins nor drifts sideways.

Solver (DynamicsSlideTest, DynamicsBounceTest): a frictionless box on a 30 degree slope slides for a second;
its downhill speed matches the per-tick recurrence v = (v + g sin(a) dt) / (1 + d dt) of the solver's gravity
and damping within 1 % (the contacts cancel only the normal part), nothing across the slope, no spin. A box
dropped flat with restitution 1 leaves the ground with its impact speed (within the tick's gravity), one with
restitution 0 keeps less than 5 % of it.

Pipeline (DynamicsFirstTickTest, DynamicsFlushTest): 101 bodies added to a pipeline of initial size 64 tick once
and every contact is coloured (the per-colour body bit sets must have grown with the body pool before the first
tick). A flushed pipeline is empty, forgets a split it had flagged, and simulates a new scene whose islands reuse
the old slots and ids.

Numerics config (DynamicsNumericsConfigTest): the defaults' derived values against tan^2, cos and sin^2 of their
angles; out-of-range pending values clamped into the applied and the pending fields; the global set and cleared.
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
    f32     restitution;
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
        .restitution = box->restitution,
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

        if (!ds_BodyIdEqual(body_a->id, body_b->id) || body_a->set != body_b->set || body_a->sim != body_b->sim || body_a->island != body_b->island
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


/********************************** shapes (ds_shape.c) **********************************/

/* a world point's containment in an f32 box, in f64 (the box is rounded outward from f64) */
static u32 DynamicsTestBboxContains(const struct aabb *box, const v3f64 p)
{
    return (f64) box->min.x <= p.x && p.x <= (f64) box->max.x
        && (f64) box->min.y <= p.y && p.y <= (f64) box->max.y
        && (f64) box->min.z <= p.z && p.z <= (f64) box->max.z;
}

/*
Shapes on a body far from the origin: lookups by id, the id's generation after a removal and a slot reuse, world
boxes containing the shapes' extreme points (f64), the pair queries symmetric in their arguments, the raycast equal
to the collision raycast on the shape's world pose.
*/
static struct test_Output DynamicsShapeTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);	/* the pipeline's frame memory comes from it */
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    ds_DynamicsSetGlobals(&pipeline);	/* the pair queries read the pipeline's numerics config through the globals */

    /* three collision shapes */
    const char *name[3] = { "sphere", "capsule", "box" };
    u32 cshape_index[3];
    for (u32 i = 0; i < 3; ++i)
    {
        const utf8 id = Utf8Format(&mem, "%s", name[i]);
        const struct slot slot = ds_SDBAddAlias(cshape_db, id);
        struct c_Shape *cshape = slot.address;
        cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
        switch (i)
        {
            case 0: { cshape->type = C_SHAPE_SPHERE; cshape->sphere.radius = 0.3f; } break;
            case 1: { cshape->type = C_SHAPE_CAPSULE; cshape->capsule.radius = 0.2f; cshape->capsule.half_height = 0.5f; } break;
            default: { cshape->type = C_SHAPE_CONVEX_HULL; cshape->hull = DcelBox(&mem, V3(0.5f, 0.3f, 0.2f)); } break;
        }
        c_ShapeUpdateMassProperties(cshape);
        cshape_index[i] = slot.index;
    }

    const struct ds_BodyPrefab body_prefab = { .dynamic = 1 };
    const ds_TransformWorld t_body = { .rotation = QNormalize(Q(0.3f, -0.5f, 0.1f, 0.8f)), .position = V3F64(1e6, -1e6, 1e6) };
    const ds_BodyId body_id = ds_BodyAdd(&pipeline, &body_prefab, &t_body, 0);
    struct ds_Body *body = ds_BodyLookup(&pipeline, body_id).address;
    TEST_NOT_ZERO(body != NULL);

    ds_ShapeId shape_id[3];
    const ds_Transform t_local[3] =
    {
        { .rotation = QIdentity(), .position = V3(1.0f, 0.0f, 0.0f) },
        { .rotation = QNormalize(Q(0.7f, 0.1f, -0.2f, 0.6f)), .position = V3(0.0f, 1.0f, 0.0f) },
        { .rotation = QNormalize(Q(-0.2f, 0.4f, 0.4f, 0.8f)), .position = V3(0.0f, 0.0f, 1.0f) },
    };
    for (u32 i = 0; i < 3; ++i)
    {
        const struct ds_ShapePrefab prefab = { .cshape = cshape_index[i], .density = 1.0f, .friction = 0.5f, .margin = 0.1f };
        shape_id[i] = ds_ShapeAdd(&pipeline, &prefab, &t_local[i], body_id);
        TEST_NOT_ZERO(!ds_ShapeIdNullCheck(shape_id[i]));
    }
    TEST_NOT_ZERO(body->shape_list.count == 3 && body->mass > 0.0f);

    /* lookups, and the world box contains each shape's extreme points */
    const struct ds_Shape *shape[3];
    for (u32 i = 0; i < 3; ++i)
    {
        const struct slot slot = ds_ShapeLookup(&pipeline, shape_id[i]);
        shape[i] = slot.address;
        TEST_NOT_ZERO(shape[i] != NULL && slot.index == ds_ShapeIdIndex(shape_id[i]) && shape[i]->body == ds_BodyIdIndex(body_id));

        ds_TransformWorld t;
        ds_ShapeWorldTransform(&t, &pipeline, shape[i]);
        const struct aabb box = ds_ShapeWorldBbox(&pipeline, shape[i]);
        const struct c_Shape *cshape = cshape_db.pool.buf + shape[i]->cshape_handle;
        u32 contained = 1;
        if (cshape->type == C_SHAPE_CONVEX_HULL)
        {
            for (u32 v = 0; v < cshape->hull.v_count; ++v)
            {
                contained &= DynamicsTestBboxContains(&box, ds_TransformWorldPointToWorld(&t, cshape->hull.v[v]));
            }
        }
        else
        {
            const f32 r = (cshape->type == C_SHAPE_SPHERE) ? cshape->sphere.radius : cshape->capsule.radius;
            const f32 h = (cshape->type == C_SHAPE_SPHERE) ? 0.0f : cshape->capsule.half_height;
            for (u32 k = 0; k < 6; ++k)
            {
                /* the six axis-extreme points of the sphere or the two caps */
                v3 local = V3Zero();
                local.buf[k % 3] = (k < 3) ? r : -r;
                local.y += (k < 3) ? h : -h;
                contained &= DynamicsTestBboxContains(&box, ds_TransformWorldPointToWorld(&t, local));
                local.y -= (k < 3) ? 2.0f*h : -2.0f*h;
                contained &= DynamicsTestBboxContains(&box, ds_TransformWorldPointToWorld(&t, local));
            }
        }
        if (!contained)
        {
            fprintf(stderr, "shape %s: world box does not contain its extreme points\n", name[i]);
            TEST_FAILURE;
        }
    }

    /* pair queries: symmetric in their arguments */
    for (u32 a = 0; a < 3; ++a)
    for (u32 b = a + 1; b < 3; ++b)
    {
        TEST_NOT_ZERO(ds_ShapeTest(&pipeline, shape[a], shape[b]) == ds_ShapeTest(&pipeline, shape[b], shape[a]));
        v3f64 c0, c1, d0, d1;
        const f32 dab = ds_ShapeDistance(&c0, &c1, &pipeline, shape[a], shape[b]);
        const f32 dba = ds_ShapeDistance(&d0, &d1, &pipeline, shape[b], shape[a]);
        TEST_NOT_ZERO(F32Abs(dab - dba) <= 1e-5f * (1.0f + dab));
        if (dab > 0.0f)
        {
            TEST_NOT_ZERO(DynamicsTestDistanceSquared(c0, d1) <= 1e-8 && DynamicsTestDistanceSquared(c1, d0) <= 1e-8);
            TEST_NOT_ZERO((ds_ShapeTest(&pipeline, shape[a], shape[b]) == 0));
        }
    }

    /* raycasts: the pipeline's shape raycast is the collision raycast on the shape's world pose */
    for (u32 i = 0; i < 3; ++i)
    {
        ds_TransformWorld t;
        ds_ShapeWorldTransform(&t, &pipeline, shape[i]);
        const struct c_Shape *cshape = cshape_db.pool.buf + shape[i]->cshape_handle;
        for (u32 k = 0; k < 50; ++k)
        {
            const struct rayWorld ray =
            {
                .origin = V3F64Add(t.position, V3Promote(V3(RngF32Range(-3.0f, 3.0f), RngF32Range(-3.0f, 3.0f), RngF32Range(-3.0f, 3.0f)))),
                .dir = V3(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f)),
            };
            if (V3LengthSquared(ray.dir) == 0.0f) { continue; }
            const f32 expected = (cshape->type == C_SHAPE_SPHERE) ? c_SphereRaycastParameter(cshape, &t, &ray)
                               : (cshape->type == C_SHAPE_CAPSULE) ? c_CapsuleRaycastParameter(cshape, &t, &ray)
                               : c_HullRaycastParameter(cshape, &t, &ray);
            TEST_NOT_ZERO(ds_ShapeRaycastParameter(&pipeline, shape[i], &ray) == expected);
        }
    }

    /* removal: the id dies, a new shape in the reused slot gets another generation */
    const ds_ShapeId removed = shape_id[1];
    const u32 removed_index = ds_ShapeIdIndex(removed);
    ds_ShapeDynamicRemove(&pipeline, body, removed_index, 1);
    TEST_NOT_ZERO(ds_ShapeLookup(&pipeline, removed).address == NULL && body->shape_list.count == 2);
    const struct ds_ShapePrefab again = { .cshape = cshape_index[1], .density = 1.0f, .friction = 0.5f, .margin = 0.1f };
    const ds_ShapeId reused = ds_ShapeAdd(&pipeline, &again, &t_local[1], body_id);
    TEST_NOT_ZERO(!ds_ShapeIdNullCheck(reused) && !ds_ShapeIdEqual(reused, removed));
    TEST_NOT_ZERO(ds_ShapeLookup(&pipeline, removed).address == NULL && ds_ShapeLookup(&pipeline, reused).address != NULL);
    TEST_NOT_ZERO(ds_ShapeLookup(&pipeline, ds_ShapeIdNull()).address == NULL);
    TEST_NOT_ZERO(body->shape_list.count == 3);

    ds_BodyRemove(&mem, &pipeline, body_id);
    TEST_NOT_ZERO(pipeline.shape_pool.count == 0 && ds_ShapeLookup(&pipeline, reused).address == NULL);

    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}


/********************************** bodies (ds_body.c) **********************************/

static u32 DynamicsTestNear(const f32 a, const f32 b, const f32 scale)
{
    return F32Abs(a - b) <= 1e-4f * scale;
}

/*
Bodies: a dynamic body of two spheres of different densities has the mass, center of mass and inverse inertia of
the closed forms (parallel axis); a static body has none; lookups by id; a removal frees the body's usage bit and a
second removal with the dead id is a no-op that leaves the other bodies untouched.
*/
static struct test_Output DynamicsBodyTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    ds_DynamicsSetGlobals(&pipeline);

    const f32 r = 0.5f;
    const utf8 id = Utf8Format(&mem, "sphere");
    const struct slot slot = ds_SDBAddAlias(cshape_db, id);
    struct c_Shape *cshape = slot.address;
    cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
    cshape->type = C_SHAPE_SPHERE;
    cshape->sphere.radius = r;
    c_ShapeUpdateMassProperties(cshape);
    const f32 volume = cshape->volume;

    const struct ds_BodyPrefab dynamic_prefab = { .dynamic = 1 };
    const struct ds_BodyPrefab static_prefab = { .dynamic = 0 };
    const ds_TransformWorld t_a = { .rotation = QIdentity(), .position = V3F64(10.0, 20.0, 30.0) };
    const ds_TransformWorld t_b = { .rotation = QNormalize(Q(0.1f, 0.2f, 0.3f, 0.9f)), .position = V3F64(-5.0, 0.0, 5.0) };
    const ds_BodyId a = ds_BodyAdd(&pipeline, &dynamic_prefab, &t_a, 0);
    const ds_BodyId b = ds_BodyAdd(&pipeline, &static_prefab, &t_b, 1);
    TEST_NOT_ZERO(!ds_BodyIdNullCheck(a) && !ds_BodyIdNullCheck(b) && !ds_BodyIdEqual(a, b));

    /* two spheres on a: density 1 at x = +1, density 3 at x = -1 */
    const struct ds_ShapePrefab light = { .cshape = slot.index, .density = 1.0f, .friction = 0.5f, .margin = 0.1f };
    const struct ds_ShapePrefab heavy = { .cshape = slot.index, .density = 3.0f, .friction = 0.5f, .margin = 0.1f };
    const ds_Transform at_plus = { .rotation = QIdentity(), .position = V3(1.0f, 0.0f, 0.0f) };
    const ds_Transform at_minus = { .rotation = QIdentity(), .position = V3(-1.0f, 0.0f, 0.0f) };
    TEST_NOT_ZERO(!ds_ShapeIdNullCheck(ds_ShapeAdd(&pipeline, &light, &at_plus, a)));
    TEST_NOT_ZERO(!ds_ShapeIdNullCheck(ds_ShapeAdd(&pipeline, &heavy, &at_minus, a)));
    TEST_NOT_ZERO(!ds_ShapeIdNullCheck(ds_ShapeAdd(&pipeline, &heavy, &at_minus, b)));

    /* closed forms: masses V and 3V, com at -0.5; I_xx = 2/5 (4V) r^2, I_yy = I_zz = I_xx + V 1.5^2 + 3V 0.5^2 */
    const f32 mass = 4.0f * volume;
    const f32 i_xx = 0.4f * mass * r*r;
    const f32 i_yy = i_xx + volume * 2.25f + 3.0f * volume * 0.25f;
    {
        const struct ds_Body *body = ds_BodyLookup(&pipeline, a).address;
        TEST_NOT_ZERO(body != NULL && body->set == SOLVER_SET_ACTIVE && body->shape_list.count == 2);
        const struct ds_SolverSet *set = pipeline.solver_set_pool.buf + body->set;
        const struct ds_BodySim *sim = set->body_sim_pool.buf + body->sim;
        TEST_NOT_ZERO(DynamicsTestNear(body->mass, mass, mass) && DynamicsTestNear(sim->inv_mass, 1.0f / mass, 1.0f / mass));
        TEST_NOT_ZERO(DynamicsTestNear(sim->local_center_of_mass.x, -0.5f, 1.0f) && DynamicsTestNear(sim->local_center_of_mass.y, 0.0f, 1.0f) && DynamicsTestNear(sim->local_center_of_mass.z, 0.0f, 1.0f));
        const m3 inv = sim->local_inv_inertia;
        const f32 scale = 1.0f / i_xx;
        TEST_NOT_ZERO(DynamicsTestNear(inv.a11, 1.0f / i_xx, scale) && DynamicsTestNear(inv.a22, 1.0f / i_yy, scale) && DynamicsTestNear(inv.a33, 1.0f / i_yy, scale));
        TEST_NOT_ZERO(DynamicsTestNear(inv.a12, 0.0f, scale) && DynamicsTestNear(inv.a13, 0.0f, scale) && DynamicsTestNear(inv.a23, 0.0f, scale));
        TEST_NOT_ZERO(sim->world.position.x == 10.0 && sim->world.position.y == 20.0 && sim->world.position.z == 30.0);
    }
    {
        const struct ds_Body *body = ds_BodyLookup(&pipeline, b).address;
        TEST_NOT_ZERO(body != NULL && body->set == SOLVER_SET_STATIC && body->island == POOL_NULL && body->mass == 0.0f);
        const struct ds_SolverSet *set = pipeline.solver_set_pool.buf + body->set;
        const struct ds_BodySim *sim = set->body_sim_pool.buf + body->sim;
        TEST_NOT_ZERO(sim->inv_mass == 0.0f && sim->local_inv_inertia.a11 == 0.0f && sim->local_inv_inertia.a22 == 0.0f && sim->local_inv_inertia.a33 == 0.0f);
    }

    /* lookups: another generation of the same index is not found */
    TEST_NOT_ZERO(ds_BodyLookup(&pipeline, ds_BodyIdNextGeneration(a)).address == NULL);
    TEST_NOT_ZERO(ds_BodyLookup(&pipeline, ds_BodyIdNull()).address == NULL);

    /* removal: the usage bit goes, the id dies, and removing it again changes nothing */
    const u32 index_a = ds_BodyIdIndex(a);
    ds_BodyRemove(&mem, &pipeline, a);
    TEST_NOT_ZERO(ds_BodyLookup(&pipeline, a).address == NULL && pipeline.body_pool.count == 1 && pipeline.shape_pool.count == 1);
    TEST_NOT_ZERO(ds_BitSetGet(&pipeline.body_usage_set, index_a) == 0);
    const struct ds_Body *body_b = ds_BodyLookup(&pipeline, b).address;
    const u32 sim_b = body_b->sim;
    ds_BodyRemove(&mem, &pipeline, a);
    TEST_NOT_ZERO(pipeline.body_pool.count == 1 && ds_BodyLookup(&pipeline, b).address == body_b && body_b->sim == sim_b);
    const struct ds_SolverSet *static_set = pipeline.solver_set_pool.buf + SOLVER_SET_STATIC;
    TEST_NOT_ZERO(static_set->body_sim_pool.count == 1 && static_set->body_sim_pool.buf[sim_b].world.position.x == -5.0);

    /* the slot is reusable with a new generation */
    const ds_BodyId c = ds_BodyAdd(&pipeline, &dynamic_prefab, &t_a, 2);
    TEST_NOT_ZERO(!ds_BodyIdNullCheck(c) && !ds_BodyIdEqual(c, a) && ds_BodyLookup(&pipeline, a).address == NULL && ds_BodyLookup(&pipeline, c).address != NULL);

    ds_BodyRemove(&mem, &pipeline, b);
    ds_BodyRemove(&mem, &pipeline, c);
    TEST_NOT_ZERO(pipeline.body_pool.count == 0 && pipeline.shape_pool.count == 0);

    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}


/********************************** contacts (ds_contact.c) **********************************/

/*
Contacts: two dynamic boxes resting on a static ground after one tick; the pair's contacts are found by key and by
id, the shapes list them, the bodies share an island, the validation holds; removing a body takes its contacts with
it, its ids die, and the rest validates.
*/
static struct test_Output DynamicsContactTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* ground, and two unit boxes side by side on it, overlapping each other slightly */
    const struct dynamics_TestBox box[3] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(5.0f, 1.0f, 5.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-0.49f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.49f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId body[3];
    for (u32 i = 0; i < 3; ++i)
    {
        body[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, box + i, (v3f64) { 0 }, i);
    }
    ds_DynamicsTick(&pipeline);
    ds_PoolFlush(pipeline.event_pool);
    DLLFlush(pipeline.event_list);
    ds_ContactValidateAll(&pipeline);

    /* the pair's contact by key and by id */
    const struct ds_Body *b1 = ds_BodyLookup(&pipeline, body[1]).address;
    const struct ds_Body *b2 = ds_BodyLookup(&pipeline, body[2]).address;
    TEST_NOT_ZERO(b1 && b2 && b1->shape_list.count == 1 && b2->shape_list.count == 1);
    const u32 s1 = b1->shape_list.first, s2 = b2->shape_list.first;
    const struct ds_ContactKey key = ds_ContactKeyCanonical(s2, s1);
    TEST_NOT_ZERO(key.shape[0] < key.shape[1] && ds_ContactKeyEquivalence(key, ds_ContactKeyCanonical(s1, s2)));
    const struct slot found = ds_ContactKeyLookup(&pipeline, key);
    const struct ds_Contact *c = found.address;
    TEST_NOT_ZERO(c != NULL && ds_ContactKeyEquivalence(c->key, key));
    TEST_NOT_ZERO(ds_ContactLookup(&pipeline, c->id).address == c);
    TEST_NOT_ZERO(ds_ContactLookup(&pipeline, ds_ContactIdNextGeneration(c->id)).address == NULL);
    TEST_NOT_ZERO(ds_ContactLookup(&pipeline, ds_ContactIdNull()).address == NULL);
    const ds_ContactId contact_id = c->id;

    /* the shapes list it (each with the ground's contact too), and the touching boxes share an island */
    const struct ds_Shape *shape1 = pipeline.shape_pool.buf + s1;
    const struct ds_Shape *shape2 = pipeline.shape_pool.buf + s2;
    TEST_NOT_ZERO(shape1->contact_list.count == 2 && shape2->contact_list.count == 2);
    TEST_NOT_ZERO(c->narrowphase.manifold_count > 0 && c->set == SOLVER_SET_NULL && c->color != CG_INVALID_COLOR);
    TEST_NOT_ZERO(b1->island == b2->island && c->island == b1->island);
    TEST_NOT_ZERO(pipeline.contact_pool.count == 3);

    /* removing box 2 takes its contacts: box 1 keeps the ground's */
    ds_BodyRemove(&mem, &pipeline, body[2]);
    ds_ContactValidateAll(&pipeline);
    TEST_NOT_ZERO(ds_ContactLookup(&pipeline, contact_id).address == NULL && ds_ContactKeyLookup(&pipeline, key).address == NULL);
    TEST_NOT_ZERO(shape1->contact_list.count == 1 && pipeline.contact_pool.count == 1);
    ds_DynamicsTick(&pipeline);
    ds_PoolFlush(pipeline.event_pool);
    DLLFlush(pipeline.event_list);
    ds_ContactValidateAll(&pipeline);
    TEST_NOT_ZERO(pipeline.contact_pool.count == 1 && ds_BodyLookup(&pipeline, body[1]).address != NULL);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}


/********************************** islands (ds_island.c) **********************************/

/*
Islands: a lone dynamic body's island; its id dies with the body and a body in the reused slot gets another
generation; three boxes in a row share one island, and removing the middle one splits the rest into two valid
islands on the next tick.
*/
static struct test_Output DynamicsIslandTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* a lone body: its island, its island's id, and the id's death */
    const struct dynamics_TestBox lone = { .position = V3(0.0f, 10.0f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f };
    const ds_BodyId a = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, &lone, (v3f64) { 0 }, 0);
    const struct ds_Body *body_a = ds_BodyLookup(&pipeline, a).address;
    const u32 island_a = body_a->island;
    const ds_IslandId island_a_id = pipeline.island_pool.buf[island_a].id;
    TEST_NOT_ZERO(ds_IslandLookup(&pipeline, island_a_id).index == island_a);
    TEST_NOT_ZERO(ds_IslandLookup(&pipeline, ds_IslandIdNextGeneration(island_a_id)).address == NULL);
    TEST_NOT_ZERO(ds_IslandLookup(&pipeline, ds_IslandIdNull()).address == NULL);
    ds_IslandValidateAll(&pipeline);

    ds_BodyRemove(&mem, &pipeline, a);
    TEST_NOT_ZERO(pipeline.island_pool.count == 0 && ds_IslandLookup(&pipeline, island_a_id).address == NULL);
    const ds_BodyId b = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, &lone, (v3f64) { 0 }, 1);
    const struct ds_Body *body_b = ds_BodyLookup(&pipeline, b).address;
    const ds_IslandId island_b_id = pipeline.island_pool.buf[body_b->island].id;
    if (ds_IslandIdEqual(island_b_id, island_a_id) || ds_IslandLookup(&pipeline, island_a_id).address != NULL)
    {
        fprintf(stderr, "island: the removed island's id %llx is alive again as %llx\n", (long long unsigned int) island_a_id.val, (long long unsigned int) island_b_id.val);
        TEST_FAILURE;
    }
    ds_BodyRemove(&mem, &pipeline, b);

    /* three boxes in a row on a ground: one island; without the middle one, two */
    const struct dynamics_TestBox row[4] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(8.0f, 1.0f, 8.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-0.98f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.98f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId body[4];
    for (u32 i = 0; i < 4; ++i)
    {
        body[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, row + i, (v3f64) { 0 }, 2 + i);
    }
    ds_DynamicsTick(&pipeline);
    ds_PoolFlush(pipeline.event_pool);
    DLLFlush(pipeline.event_list);
    ds_IslandValidateAll(&pipeline);
    const struct ds_Body *left = ds_BodyLookup(&pipeline, body[1]).address;
    const struct ds_Body *middle = ds_BodyLookup(&pipeline, body[2]).address;
    const struct ds_Body *right = ds_BodyLookup(&pipeline, body[3]).address;
    TEST_NOT_ZERO(left->island == middle->island && middle->island == right->island);
    TEST_NOT_ZERO(pipeline.island_pool.buf[left->island].body_list.count == 3 && pipeline.island_pool.count == 1);

    ds_BodyRemove(&mem, &pipeline, body[2]);
    /* the pipeline splits a flagged island once its bodies have rested for the sleep time threshold: let them settle */
    for (u32 t = 0; t < 600 && pipeline.island_pool.count < 2; ++t)
    {
        ds_DynamicsTick(&pipeline);
        ds_PoolFlush(pipeline.event_pool);
        DLLFlush(pipeline.event_list);
        ds_IslandValidateAll(&pipeline);
    }
    TEST_NOT_ZERO(left->island != right->island && pipeline.island_pool.count == 2);
    TEST_NOT_ZERO(pipeline.island_pool.buf[left->island].body_list.count == 1 && pipeline.island_pool.buf[right->island].body_list.count == 1);
    TEST_NOT_ZERO(pipeline.island_pool.buf[left->island].contact_list.count == 1 && pipeline.island_pool.buf[right->island].contact_list.count == 1);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* the solver sets' sleep and wake bookkeeping (file comment) */
static struct test_Output DynamicsSleepTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 1;

    /* a ground, a two-box stack and a lone box: two islands that fall asleep */
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    const struct dynamics_TestBox scene[4] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-4.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(-4.0f, 1.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(4.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[4];
    for (u32 i = 0; i < 4; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const struct ds_Body *bottom = ds_BodyLookup(&pipeline, id[1]).address;
    const struct ds_Body *top = ds_BodyLookup(&pipeline, id[2]).address;
    const struct ds_Body *lone = ds_BodyLookup(&pipeline, id[3]).address;
    const u32 bottom_index = ds_BodyLookup(&pipeline, id[1]).index;

    u32 t;
    for (t = 0; t < 600 && (bottom->set < SOLVER_SET_SLEEPING_FIRST || lone->set < SOLVER_SET_SLEEPING_FIRST); ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600);
    TEST_NOT_ZERO(top->set == bottom->set && bottom->set != lone->set);
    TEST_NOT_ZERO(SnapshotTestSleepingSetCount(&pipeline) == 2);

    /* the active set is empty; each sleeping set holds one island, its bodies' simulation state without compute state, and its contacts with their caches, in one arena */
    const struct ds_SolverSet *active = pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    TEST_NOT_ZERO(active->island_pool.count == 0 && active->body_sim_pool.count == 0 && active->body_compute_pool.count == 0);
    for (u32 i = SOLVER_SET_SLEEPING_FIRST; i < pipeline.solver_set_pool.count_max; ++i)
    {
        const struct ds_SolverSet *set = pipeline.solver_set_pool.buf + i;
        if (!ds_PoolSlotAllocated(set))
        {
            continue;
        }
        TEST_NOT_ZERO(set->island_pool.count == 1 && set->mem.mem_size != 0);
        const struct ds_Island *island = pipeline.island_pool.buf + set->island_pool.buf[0];
        TEST_NOT_ZERO(island->set == i && island->set_island_index == 0);
        TEST_NOT_ZERO(set->body_sim_pool.count == island->body_list.count && set->body_compute_pool.count == 0);
        TEST_NOT_ZERO(set->contact_pool.count == island->contact_list.count && set->contact_compute_pool.count == island->contact_list.count);
        ds_SolverSetValidate(&pipeline, i);
    }
    ds_IslandValidateAll(&pipeline);
    const struct ds_SolverSet *stack_set = pipeline.solver_set_pool.buf + bottom->set;
    const struct ds_SolverSet *lone_set = pipeline.solver_set_pool.buf + lone->set;
    TEST_NOT_ZERO(stack_set->body_sim_pool.count == 2 && stack_set->contact_pool.count == 2);
    TEST_NOT_ZERO(lone_set->body_sim_pool.count == 1 && lone_set->contact_pool.count == 1);
    TEST_NOT_ZERO(stack_set->body_sim_pool.buf[bottom->sim].body == bottom_index);

    /* waking a set that is not sleeping is a no-op */
    ds_SolverSetWakeUp(&pipeline, SOLVER_SET_ACTIVE);
    ds_SolverSetWakeUp(&pipeline, SOLVER_SET_STATIC);
    ds_SolverSetWakeUp(&pipeline, SOLVER_SET_NULL);
    active = pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    TEST_NOT_ZERO(SnapshotTestSleepingSetCount(&pipeline) == 2 && active->body_sim_pool.count == 0);

    /* removing the top box wakes the stack's set: the bottom box is active again with fresh compute state and a reset sleep timer; the lone box still sleeps */
    const u32 stack_set_index = bottom->set;
    ds_BodyRemove(&mem, &pipeline, id[2]);
    active = pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    TEST_NOT_ZERO(!ds_PoolSlotAllocated(pipeline.solver_set_pool.buf + stack_set_index));
    TEST_NOT_ZERO(bottom->set == SOLVER_SET_ACTIVE && lone->set >= SOLVER_SET_SLEEPING_FIRST);
    TEST_NOT_ZERO(active->body_sim_pool.count == 1 && active->body_compute_pool.count == 1 && active->island_pool.count == 1);
    TEST_NOT_ZERO(active->body_sim_pool.buf[bottom->sim].body == bottom_index && bottom->low_velocity_time == 0.0f);
    ds_SolverSetValidate(&pipeline, SOLVER_SET_ACTIVE);
    ds_IslandValidateAll(&pipeline);

    /* the bottom box rests and sleeps again, into the freed set slot */
    for (t = 0; t < 600 && bottom->set < SOLVER_SET_SLEEPING_FIRST; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && bottom->set == stack_set_index && SnapshotTestSleepingSetCount(&pipeline) == 2);

    /* disabling sleeping wakes everything */
    g_solver_config->pending_sleep_enabled = 0;
    SnapshotTestTick(&pipeline);
    active = pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    TEST_NOT_ZERO(SnapshotTestSleepingSetCount(&pipeline) == 0 && bottom->set == SOLVER_SET_ACTIVE && lone->set == SOLVER_SET_ACTIVE);
    TEST_NOT_ZERO(active->body_sim_pool.count == 2 && active->body_compute_pool.count == 2 && active->island_pool.count == 2);
    ds_SolverSetValidate(&pipeline, SOLVER_SET_ACTIVE);
    ds_IslandValidateAll(&pipeline);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/*
 * The broadphase after the dirty-set change: once every island sleeps, no shape queries (the dirty set is empty
 * after a tick) and the sleeping contacts persist; a box dropped onto the sleeping stack, with a larger body
 * index than every sleeping body, is paired with the top box by its own query and wakes the stack through that
 * contact; the three then rest and sleep as one island.
 */
static struct test_Output DynamicsWakeByContactTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 1;

    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    const struct dynamics_TestBox scene[4] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 1.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 4.0f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[4];
    for (u32 i = 0; i < 3; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const struct ds_Body *bottom = ds_BodyLookup(&pipeline, id[1]).address;
    const struct ds_Body *top = ds_BodyLookup(&pipeline, id[2]).address;

    u32 t;
    for (t = 0; t < 600 && bottom->set < SOLVER_SET_SLEEPING_FIRST; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && top->set == bottom->set);
    const u32 stack_set_index = bottom->set;
    const struct ds_SolverSet *stack_set = pipeline.solver_set_pool.buf + stack_set_index;
    TEST_NOT_ZERO(stack_set->contact_pool.count == 2);

    /* asleep: nothing is dirty after a tick, and the sleeping contacts stay */
    for (t = 0; t < 60; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    u64 dirty_bits = 0;
    for (u64 block = 0; block < pipeline.shape_dirty_set.block_count; ++block)
    {
        dirty_bits |= pipeline.shape_dirty_set.bits[block];
    }
    TEST_NOT_ZERO(dirty_bits == 0);
    TEST_NOT_ZERO(bottom->set == stack_set_index && stack_set->contact_pool.count == 2);

    /* the dropped box has the largest body index: its own query must report the pair with the sleeping top box */
    id[3] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + 3, (v3f64) { 0 }, 3);
    const struct ds_Body *dropped = ds_BodyLookup(&pipeline, id[3]).address;
    TEST_NOT_ZERO(ds_BodyLookup(&pipeline, id[3]).index > ds_BodyLookup(&pipeline, id[2]).index);
    for (t = 0; t < 600 && bottom->set != SOLVER_SET_ACTIVE; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && top->set == SOLVER_SET_ACTIVE && dropped->set == SOLVER_SET_ACTIVE);
    const struct ds_ContactKey key = ds_ContactKeyCanonical((u32) dropped->shape_list.first, (u32) top->shape_list.first);
    TEST_NOT_ZERO(ds_ContactKeyLookup(&pipeline, key).address != NULL);
    ds_IslandValidateAll(&pipeline);

    /* the three rest and sleep as one island */
    for (t = 0; t < 900 && dropped->set < SOLVER_SET_SLEEPING_FIRST; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 900 && dropped->set == bottom->set && top->set == bottom->set);
    TEST_NOT_ZERO(SnapshotTestSleepingSetCount(&pipeline) == 1);
    ds_SolverSetValidate(&pipeline, bottom->set);
    ds_IslandValidateAll(&pipeline);

    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/*
 * The constraint graph's invariants: every coloured contact is between an active island's bodies, a static
 * colour holds static-dynamic contacts and a dynamic colour dynamic-dynamic ones, no dynamic body has two
 * contacts of one colour, and a colour's body bit set marks exactly the dynamic bodies with a contact in it.
 * Returns the number of contacts in the colours, or U32_MAX on a violation.
 */
static u32 GraphTestCheck(const struct ds_Dynamics *pipeline)
{
    struct arena *tmp = ArenaPushScratch();
    u32 *colour_count = ArenaPush(tmp, pipeline->body_pool.count_max*sizeof(u32));
    u32 total = 0;
    u32 ok = 1;
    for (u32 c = 0; c < CG_COLOR_COUNT && ok; ++c)
    {
        const struct ds_CGraphColor *colour = pipeline->cgraph.color + c;
        for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
        {
            colour_count[i] = 0;
        }
        for (u32 i = 0; i < colour->contact_pool.count; ++i)
        {
            const struct ds_Contact *contact = pipeline->contact_pool.buf + colour->contact_pool.buf[i];
            const struct ds_Shape *shape[2] = { pipeline->shape_pool.buf + contact->key.shape[0], pipeline->shape_pool.buf + contact->key.shape[1] };
            const struct ds_Body *body[2] = { pipeline->body_pool.buf + shape[0]->body, pipeline->body_pool.buf + shape[1]->body };
            const u32 dynamic = (ds_BodyDynamicCheck(body[0]) != 0) + (ds_BodyDynamicCheck(body[1]) != 0);
            ok &= (contact->color == c && contact->compute == i && contact->set == SOLVER_SET_NULL);
            ok &= (pipeline->island_pool.buf[contact->island].set == SOLVER_SET_ACTIVE);
            ok &= (c > CG_STATIC_COLOR_LAST || dynamic == 1);
            ok &= (c < CG_DYNAMIC_COLOR_FIRST || c > CG_DYNAMIC_COLOR_LAST || dynamic == 2);
            for (u32 k = 0; k < 2; ++k)
            {
                if (ds_BodyDynamicCheck(body[k]))
                {
                    colour_count[shape[k]->body] += 1;
                }
            }
            total += 1;
        }
        if (c == CG_SERIAL_COLOR)
        {
            continue;
        }
        for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
        {
            ok &= (colour_count[i] <= 1);
            ok &= (ds_BitSetGet(&colour->body_bitset, i) == (colour_count[i] == 1));
        }
    }
    ArenaPopScratch();
    return (ok) ? total : U32_MAX;
}

/* the constraint graph's colouring (file comment) */
static struct test_Output DynamicsGraphTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /*
     * A ground, a three-box row touching along x, and a slab carrying two more small boxes than there are dynamic
     * colours, on a grid of 2.5 m: more neighbours than dynamic colours.
     */
    #define GRAPH_TEST_SLAB_BOXES   (CG_DYNAMIC_COLOR_COUNT + 2)
    #define GRAPH_TEST_SLAB_SIDE    5                           /* grid side: side^2 >= GRAPH_TEST_SLAB_BOXES */
    #define GRAPH_TEST_BOXES        (5 + GRAPH_TEST_SLAB_BOXES)      /* ground, row, slab, small boxes */
    ds_Assert(GRAPH_TEST_SLAB_SIDE*GRAPH_TEST_SLAB_SIDE >= GRAPH_TEST_SLAB_BOXES);
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    struct dynamics_TestBox scene[GRAPH_TEST_BOXES];
    u32 count = 0;
    scene[count++] = (struct dynamics_TestBox) { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f };
    for (u32 i = 0; i < 3; ++i)
    {
        scene[count++] = (struct dynamics_TestBox) { .position = V3(-8.0f + 0.98f * ((f32) i - 1.0f), 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f };
    }
    const u32 slab = count;
    const f32 slab_hw = 0.5f + 2.5f * 0.5f * (f32) GRAPH_TEST_SLAB_SIDE;
    scene[count++] = (struct dynamics_TestBox) { .position = V3(6.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(slab_hw, 0.5f, slab_hw), .dynamic = 1, .friction = 0.5f };
    for (u32 i = 0; i < GRAPH_TEST_SLAB_BOXES; ++i)
    {
        const f32 x = 6.0f + 2.5f * ((f32) (i % GRAPH_TEST_SLAB_SIDE) - 0.5f * (f32) (GRAPH_TEST_SLAB_SIDE - 1));
        const f32 z = 2.5f * ((f32) (i / GRAPH_TEST_SLAB_SIDE) - 0.5f * (f32) (GRAPH_TEST_SLAB_SIDE - 1));
        scene[count++] = (struct dynamics_TestBox) { .position = V3(x, 1.5f, z), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f };
    }
    ds_Assert(count == GRAPH_TEST_BOXES);
    ds_BodyId id[GRAPH_TEST_BOXES];
    for (u32 i = 0; i < count; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }

    /* after a tick every contact is coloured: 4 with the ground, 2 in the row, one per small box on the slab; at least two of the slab's overflow into the serial colour */
    SnapshotTestTick(&pipeline);
    ds_CGraphValidate(&pipeline);
    TEST_NOT_ZERO(GraphTestCheck(&pipeline) == 6 + GRAPH_TEST_SLAB_BOXES);
    TEST_NOT_ZERO(pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count >= GRAPH_TEST_SLAB_BOXES - CG_DYNAMIC_COLOR_COUNT);
    const u32 slab_index = ds_BodyLookup(&pipeline, id[slab]).index;
    u32 slab_dynamic_colours = 0;
    for (u32 c = CG_DYNAMIC_COLOR_FIRST; c <= CG_DYNAMIC_COLOR_LAST; ++c)
    {
        slab_dynamic_colours += ds_BitSetGet(&pipeline.cgraph.color[c].body_bitset, slab_index);
    }
    TEST_NOT_ZERO(slab_dynamic_colours == CG_DYNAMIC_COLOR_COUNT);

    /* removing the slab takes its contacts out of the colours and clears its bits; the rest stay coloured */
    ds_BodyRemove(&mem, &pipeline, id[slab]);
    ds_CGraphValidate(&pipeline);
    TEST_NOT_ZERO(GraphTestCheck(&pipeline) == 5);
    for (u32 c = 0; c < CG_COLOR_COUNT; ++c)
    {
        TEST_NOT_ZERO(c == CG_SERIAL_COLOR || ds_BitSetGet(&pipeline.cgraph.color[c].body_bitset, slab_index) == 0);
    }

    /* the small boxes land on the ground: static-dynamic contacts for them and the row, the row's two dynamic ones, nothing serial */
    for (u32 t = 0; t < 120; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    ds_CGraphValidate(&pipeline);
    TEST_NOT_ZERO(GraphTestCheck(&pipeline) == 5 + GRAPH_TEST_SLAB_BOXES);
    u32 dynamic_contacts = 0;
    for (u32 c = CG_DYNAMIC_COLOR_FIRST; c <= CG_DYNAMIC_COLOR_LAST; ++c)
    {
        dynamic_contacts += pipeline.cgraph.color[c].contact_pool.count;
    }
    TEST_NOT_ZERO(dynamic_contacts == 2 && pipeline.cgraph.color[CG_SERIAL_COLOR].contact_pool.count == 0);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

static u32 JointTestV3Equal(const v3 a, const v3 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/* the joint bookkeeping that is finished: add, lookup, remove (file comment) */
static struct test_Output DynamicsJointTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    const struct dynamics_TestBox scene[3] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(8.0f, 1.0f, 8.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-2.0f, 3.0f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(2.0f, 3.0f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[3];
    for (u32 i = 0; i < 3; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const struct ds_Body *ground = ds_BodyLookup(&pipeline, id[0]).address;
    const struct ds_Body *a = ds_BodyLookup(&pipeline, id[1]).address;
    const struct ds_Body *b = ds_BodyLookup(&pipeline, id[2]).address;
    const u32 index[3] = { ds_BodyLookup(&pipeline, id[0]).index, ds_BodyLookup(&pipeline, id[1]).index, ds_BodyLookup(&pipeline, id[2]).index };

    /* a static-dynamic joint and a dynamic-dynamic one: found by id, in both bodies' lists, coloured with their sims pointing back */
    const ds_Transform frame_ground = { .rotation = QIdentity(), .position = V3(-2.0f, 1.0f, 0.0f) };
    const ds_Transform frame_a = { .rotation = QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), 0.5f), .position = V3(0.0f, -0.5f, 0.0f) };
    const ds_Transform frame_b = { .rotation = QIdentity(), .position = V3(-0.5f, 0.0f, 0.0f) };
    struct ds_DistanceJointPrefab prefab;
    ds_DistanceJointPrefab(&prefab, 1.0f);
    const ds_JointId j0 = ds_DistanceJointAdd(&pipeline, &prefab, id[0], &frame_ground, id[1], &frame_a);
    const ds_JointId j1 = ds_DistanceJointAdd(&pipeline, &prefab, id[1], &frame_a, id[2], &frame_b);
    TEST_NOT_ZERO(!ds_JointIdNullCheck(j0) && !ds_JointIdNullCheck(j1) && !ds_JointIdEqual(j0, j1));
    TEST_NOT_ZERO(ds_JointIdNullCheck(ds_DistanceJointAdd(&pipeline, &prefab, id[0], &frame_ground, ds_BodyIdNull(), &frame_a)));
    TEST_NOT_ZERO(ds_JointLookup(&pipeline, ds_JointIdNull()).address == NULL);
    TEST_NOT_ZERO(ds_JointLookup(&pipeline, ds_JointIdNextGeneration(j0)).address == NULL);
    const struct slot s0 = ds_JointLookup(&pipeline, j0);
    const struct slot s1 = ds_JointLookup(&pipeline, j1);
    TEST_NOT_ZERO(s0.address != NULL && s1.address != NULL && s0.index == ds_JointIdIndex(j0) && s1.index == ds_JointIdIndex(j1));
    const struct ds_Joint *joint0 = s0.address;
    const struct ds_Joint *joint1 = s1.address;
    /* the add's canonical slot: the static ground is body 1 of joint0 although it was passed first */
    TEST_NOT_ZERO(joint0->body[0] == index[1] && joint0->body[1] == index[0] && joint1->body[0] == index[1] && joint1->body[1] == index[2]);
    TEST_NOT_ZERO(ground->joint_list.count == 1 && a->joint_list.count == 2 && b->joint_list.count == 1);
    TEST_NOT_ZERO(joint0->set == SOLVER_SET_NULL && joint0->color <= CG_STATIC_COLOR_LAST);
    TEST_NOT_ZERO(joint1->set == SOLVER_SET_NULL && CG_DYNAMIC_COLOR_FIRST <= joint1->color && joint1->color <= CG_DYNAMIC_COLOR_LAST);
    const struct ds_JointSim *sim0 = pipeline.cgraph.color[joint0->color].joint_sim_pool.buf + joint0->sim;
    const struct ds_JointSim *sim1 = pipeline.cgraph.color[joint1->color].joint_sim_pool.buf + joint1->sim;
    TEST_NOT_ZERO(sim0->joint == s0.index && sim1->joint == s1.index && sim0->type == DS_JOINT_TYPE_DISTANCE && sim1->type == DS_JOINT_TYPE_DISTANCE);
    TEST_NOT_ZERO(JointTestV3Equal(sim0->local_frame[0].position, frame_a.position) && JointTestV3Equal(sim0->local_frame[1].position, frame_ground.position));
    TEST_NOT_ZERO(sim1->local_frame[0].rotation.x == frame_a.rotation.x && sim1->local_frame[0].rotation.w == frame_a.rotation.w && JointTestV3Equal(sim1->local_frame[1].position, frame_b.position));

    /* removing one by id: gone from the lists, its colour and the lookup; a second removal is a no-op */
    const u32 colour0 = joint0->color;
    ds_JointRemove(&pipeline, j0);
    TEST_NOT_ZERO(ds_JointLookup(&pipeline, j0).address == NULL && ds_JointLookup(&pipeline, j1).address == joint1);
    TEST_NOT_ZERO(ground->joint_list.count == 0 && a->joint_list.count == 1 && b->joint_list.count == 1);
    TEST_NOT_ZERO(pipeline.cgraph.color[colour0].joint_sim_pool.count == 0 && pipeline.joint_pool.count == 1);
    ds_JointRemove(&pipeline, j0);
    TEST_NOT_ZERO(pipeline.joint_pool.count == 1 && a->joint_list.first == (i32) s1.index);

    /* a reused slot gets a new generation; removing a body takes its joints */
    const ds_JointId j2 = ds_DistanceJointAdd(&pipeline, &prefab, id[0], &frame_ground, id[2], &frame_b);
    TEST_NOT_ZERO(ds_JointIdIndex(j2) == ds_JointIdIndex(j0) && !ds_JointIdEqual(j2, j0) && ds_JointLookup(&pipeline, j0).address == NULL);
    TEST_NOT_ZERO(b->joint_list.count == 2 && ground->joint_list.count == 1);
    ds_BodyRemove(&mem, &pipeline, id[2]);
    TEST_NOT_ZERO(pipeline.joint_pool.count == 0 && ground->joint_list.count == 0 && a->joint_list.count == 0);
    TEST_NOT_ZERO(ds_JointLookup(&pipeline, j1).address == NULL && ds_JointLookup(&pipeline, j2).address == NULL);
    for (u32 c = 0; c < CG_COLOR_COUNT; ++c)
    {
        TEST_NOT_ZERO(pipeline.cgraph.color[c].joint_sim_pool.count == 0);
    }

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

static struct test_Output DynamicsJointIslandTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* a ground and three boxes resting on it, far enough apart never to touch */
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    const struct dynamics_TestBox scene[4] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-4.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(4.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[4];
    for (u32 i = 0; i < 4; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const struct ds_Body *a = ds_BodyLookup(&pipeline, id[1]).address;
    const struct ds_Body *b = ds_BodyLookup(&pipeline, id[2]).address;
    const struct ds_Body *c = ds_BodyLookup(&pipeline, id[3]).address;
    const ds_Transform frame = { .rotation = QIdentity(), .position = V3(0.0f, 0.0f, 0.0f) };
    struct ds_DistanceJointPrefab prefab;
    ds_DistanceJointPrefab(&prefab, 4.0f);
    u32 t;

    /* each box rests in its own island */
    for (t = 0; t < 5; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(a->island != b->island && b->island != c->island && a->island != c->island);

    /* a joint merges a and b: one island holding both bodies and the joint */
    const ds_JointId jab = ds_DistanceJointAdd(&pipeline, &prefab, id[1], &frame, id[2], &frame);
    const struct ds_Joint *joint_ab = ds_JointLookup(&pipeline, jab).address;
    TEST_NOT_ZERO(joint_ab != NULL && a->island == b->island && joint_ab->island == a->island);
    const struct ds_Island *island = pipeline.island_pool.buf + a->island;
    TEST_NOT_ZERO(island->body_list.count == 2 && island->joint_list.count == 1 && island->joint_list.first == (i32) ds_JointIdIndex(jab));
    ds_DynamicsValidate(&pipeline);
    SnapshotTestTick(&pipeline);

    /* a second joint merges c; removing it marks the island, and the split keeps the jointed pair together */
    const ds_JointId jbc = ds_DistanceJointAdd(&pipeline, &prefab, id[2], &frame, id[3], &frame);
    island = pipeline.island_pool.buf + a->island;
    TEST_NOT_ZERO(a->island == c->island && island->body_list.count == 3 && island->joint_list.count == 2);
    ds_DynamicsValidate(&pipeline);
    SnapshotTestTick(&pipeline);
    ds_JointRemove(&pipeline, jbc);
    island = pipeline.island_pool.buf + a->island;
    TEST_NOT_ZERO(island->constraint_remove_count == 1 && island->joint_list.count == 1 && b->joint_list.count == 1 && c->joint_list.count == 0);
    for (t = 0; t < 600 && a->island == c->island; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && a->island == b->island && a->island != c->island);
    joint_ab = ds_JointLookup(&pipeline, jab).address;
    island = pipeline.island_pool.buf + a->island;
    TEST_NOT_ZERO(joint_ab != NULL && joint_ab->island == a->island && island->joint_list.count == 1 && island->body_list.count == 2);
    TEST_NOT_ZERO((pipeline.island_pool.buf + c->island)->joint_list.count == 0 && (pipeline.island_pool.buf + c->island)->body_list.count == 1);
    ds_DynamicsValidate(&pipeline);

    /* removing the last joint splits the pair */
    ds_JointRemove(&pipeline, jab);
    for (t = 0; t < 600 && a->island == b->island; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && (pipeline.island_pool.buf + a->island)->joint_list.count == 0 && (pipeline.island_pool.buf + b->island)->joint_list.count == 0);
    ds_DynamicsValidate(&pipeline);

    /* with sleeping on, the jointed pair's set carries the joint: in the set's pool, out of the colours */
    const ds_JointId jab2 = ds_DistanceJointAdd(&pipeline, &prefab, id[1], &frame, id[2], &frame);
    g_solver_config->pending_sleep_enabled = 1;
    for (t = 0; t < 600 && (a->set < SOLVER_SET_SLEEPING_FIRST || c->set < SOLVER_SET_SLEEPING_FIRST); ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && a->set == b->set && a->set != c->set);
    const struct ds_Joint *joint_ab2 = ds_JointLookup(&pipeline, jab2).address;
    TEST_NOT_ZERO(joint_ab2 != NULL && joint_ab2->set == a->set && joint_ab2->color == CG_INVALID_COLOR && joint_ab2->island == a->island);
    const struct ds_SolverSet *set_ab = pipeline.solver_set_pool.buf + a->set;
    TEST_NOT_ZERO(set_ab->joint_sim_pool.count == 1 && joint_ab2->sim == 0);
    TEST_NOT_ZERO(set_ab->joint_sim_pool.buf[0].joint == ds_JointIdIndex(jab2) && set_ab->joint_sim_pool.buf[0].type == DS_JOINT_TYPE_DISTANCE);
    TEST_NOT_ZERO(set_ab->joint_sim_pool.buf[0].distance.target == 4.0f);
    for (u32 colour = 0; colour < CG_COLOR_COUNT; ++colour)
    {
        TEST_NOT_ZERO(pipeline.cgraph.color[colour].joint_sim_pool.count == 0);
    }
    ds_DynamicsValidate(&pipeline);

    /* a joint to the sleeping third box wakes both sets and merges the islands; both joints are coloured again */
    const ds_JointId jbc2 = ds_DistanceJointAdd(&pipeline, &prefab, id[2], &frame, id[3], &frame);
    joint_ab2 = ds_JointLookup(&pipeline, jab2).address;
    const struct ds_Joint *joint_bc2 = ds_JointLookup(&pipeline, jbc2).address;
    TEST_NOT_ZERO(joint_ab2 != NULL && joint_bc2 != NULL);
    TEST_NOT_ZERO(a->set == SOLVER_SET_ACTIVE && b->set == SOLVER_SET_ACTIVE && c->set == SOLVER_SET_ACTIVE && a->island == c->island);
    TEST_NOT_ZERO(joint_ab2->set == SOLVER_SET_NULL && joint_ab2->color != CG_INVALID_COLOR && joint_ab2->island == a->island);
    TEST_NOT_ZERO(joint_bc2->set == SOLVER_SET_NULL && joint_bc2->color != CG_INVALID_COLOR && joint_bc2->island == a->island);
    island = pipeline.island_pool.buf + a->island;
    TEST_NOT_ZERO(island->joint_list.count == 2 && island->body_list.count == 3 && SnapshotTestSleepingSetCount(&pipeline) == 0);
    TEST_NOT_ZERO(pipeline.cgraph.color[joint_ab2->color].joint_sim_pool.buf[joint_ab2->sim].distance.target == 4.0f);
    ds_DynamicsValidate(&pipeline);

    /* removing a sleeping jointed body takes its joints and wakes the others */
    for (t = 0; t < 600 && a->set < SOLVER_SET_SLEEPING_FIRST; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && a->set == c->set && (pipeline.solver_set_pool.buf + a->set)->joint_sim_pool.count == 2);
    ds_BodyRemove(&mem, &pipeline, id[2]);
    TEST_NOT_ZERO(pipeline.joint_pool.count == 0 && a->joint_list.count == 0 && c->joint_list.count == 0);
    TEST_NOT_ZERO(a->set == SOLVER_SET_ACTIVE && c->set == SOLVER_SET_ACTIVE && ds_JointLookup(&pipeline, jab2).address == NULL && ds_JointLookup(&pipeline, jbc2).address == NULL);
    ds_DynamicsValidate(&pipeline);
    for (t = 0; t < 600 && a->island == c->island; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600);
    ds_DynamicsValidate(&pipeline);

    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* the contact between the two bodies' first shapes, or NULL */
static const struct ds_Contact *JointTestContact(const struct ds_Dynamics *pipeline, const struct ds_Body *a, const struct ds_Body *b)
{
    const struct ds_ContactKey key = ds_ContactKeyCanonical((u32) a->shape_list.first, (u32) b->shape_list.first);
    return ds_ContactKeyLookup(pipeline, key).address;
}

static struct test_Output DynamicsJointCollisionTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* a ground, a box on it and a box on that: the stacked pair touches */
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
    const struct dynamics_TestBox scene[3] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 1.5f, 0.0f), .rotation = QIdentity(), .hw = hw, .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[3];
    for (u32 i = 0; i < 3; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const struct ds_Body *ground = ds_BodyLookup(&pipeline, id[0]).address;
    const struct ds_Body *a = ds_BodyLookup(&pipeline, id[1]).address;
    const struct ds_Body *b = ds_BodyLookup(&pipeline, id[2]).address;
    const u32 ia = ds_BodyLookup(&pipeline, id[1]).index;
    const u32 ib = ds_BodyLookup(&pipeline, id[2]).index;
    const ds_Transform frame = { .rotation = QIdentity(), .position = V3(0.0f, 0.0f, 0.0f) };
    struct ds_DistanceJointPrefab suppress, allow;
    ds_DistanceJointPrefab(&suppress, 1.0f);
    ds_DistanceJointPrefab(&allow, 1.0f);
    allow.collide_connected = 1;
    u32 t;

    for (t = 0; t < 3; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) != NULL && JointTestContact(&pipeline, ground, a) != NULL);
    TEST_NOT_ZERO(ds_BodyPairMayCollide(&pipeline, ia, ib) == 1 && a->island == b->island);
    const u32 contact_count = pipeline.contact_pool.count;

    /* a suppressing joint removes the pair's contact at once, without a counted constraint removal, and ticks do not bring it back */
    const ds_JointId j0 = ds_DistanceJointAdd(&pipeline, &suppress, id[1], &frame, id[2], &frame);
    TEST_NOT_ZERO(!ds_JointIdNullCheck(j0) && ds_BodyPairMayCollide(&pipeline, ia, ib) == 0);
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) == NULL && JointTestContact(&pipeline, ground, a) != NULL);
    TEST_NOT_ZERO(pipeline.contact_pool.count == contact_count - 1 && a->island == b->island);
    TEST_NOT_ZERO((pipeline.island_pool.buf + a->island)->constraint_remove_count == 0);
    ds_DynamicsValidate(&pipeline);
    for (t = 0; t < 3; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) == NULL && pipeline.contact_pool.count == contact_count - 1);

    /* a second joint allowing collision changes nothing: the suppressing one wins */
    const ds_JointId j1 = ds_DistanceJointAdd(&pipeline, &allow, id[1], &frame, id[2], &frame);
    TEST_NOT_ZERO(!ds_JointIdNullCheck(j1) && ds_BodyPairMayCollide(&pipeline, ia, ib) == 0);
    for (t = 0; t < 3; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) == NULL);

    /* removing the suppressing joint dirties the shapes: the next tick recreates the contact */
    ds_JointRemove(&pipeline, j0);
    TEST_NOT_ZERO(ds_BodyPairMayCollide(&pipeline, ia, ib) == 1 && ds_BitSetGet(&pipeline.shape_dirty_set, (u32) a->shape_list.first));
    SnapshotTestTick(&pipeline);
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) != NULL && pipeline.contact_pool.count == contact_count);
    ds_DynamicsValidate(&pipeline);

    /* a joint allowing collision alone leaves the contact in place; removing it changes nothing either */
    for (t = 0; t < 3; ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) != NULL && pipeline.contact_pool.count == contact_count);
    ds_JointRemove(&pipeline, j1);
    TEST_NOT_ZERO(ds_BodyPairMayCollide(&pipeline, ia, ib) == 1 && JointTestContact(&pipeline, a, b) != NULL);
    SnapshotTestTick(&pipeline);
    TEST_NOT_ZERO(JointTestContact(&pipeline, a, b) != NULL && pipeline.contact_pool.count == contact_count);
    ds_DynamicsValidate(&pipeline);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

static v3 SolverTestVelocity(const struct ds_Dynamics *pipeline, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(pipeline, id).address;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    ds_Assert(body->set == SOLVER_SET_ACTIVE);
    return active->body_compute_pool.buf[body->sim].linear_velocity;
}

/* a box hanging from a static pivot on a rigid distance joint: length held, period and decay of a damped pendulum (file comment) */
static struct test_Output DynamicsPendulumTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* the bob's frame is its centre of mass: no torque on it, so it swings as a point mass on a string of length L */
    const f32 L = 2.0f;
    const f32 angle_0 = 0.1f;
    const v3 pivot = V3(0.0f, 6.0f, 0.0f);
    const v3 bob = V3Add(pivot, V3Scale(V3(F32Sin(angle_0), -F32Cos(angle_0), 0.0f), L));
    const struct dynamics_TestBox scene[2] =
    {
        { .position = pivot, .rotation = QIdentity(), .hw = V3(0.25f, 0.25f, 0.25f), .dynamic = 0, .friction = 0.5f },
        { .position = bob, .rotation = QIdentity(), .hw = V3(0.25f, 0.25f, 0.25f), .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[2];
    for (u32 i = 0; i < 2; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const ds_Transform frame = ds_TransformIdentity();
    struct ds_DistanceJointPrefab prefab;
    ds_DistanceJointPrefab(&prefab, L);
    TEST_NOT_ZERO(!ds_JointIdNullCheck(ds_DistanceJointAdd(&pipeline, &prefab, id[0], &frame, id[1], &frame)));

    /*
     * 10 s of swing. The length error, the bob's spin and its motion out of the swing plane stay small; the
     * period comes from the zero crossings of x (interpolated between ticks) and the decay from the ratio of
     * successive swing maxima, against the solver's velocity damping v <- v / (1 + d dt) (amplitude ~ exp(-d t / 2)).
     */
    const f32 dt = pipeline.timestep;
    const u32 tick_count = 600;
    f32 length_error = 0.0f;
    f32 spin = 0.0f;
    f32 out_of_plane = 0.0f;
    f32 x_prev = L * F32Sin(angle_0);
    f32 crossing_first = 0.0f;
    f32 crossing_last = 0.0f;
    u32 crossing_count = 0;
    f32 max_current = x_prev;
    f32 max_first = 0.0f;
    f32 max_last = 0.0f;
    u32 max_count = 0;
    for (u32 t = 1; t <= tick_count; ++t)
    {
        SnapshotTestTick(&pipeline);
        const ds_TransformWorld pose = DynamicsTestPose(&pipeline, id[1]);
        const v3 d = V3F64Demote(V3F64Sub(pose.position, V3Promote(pivot)));
        length_error = F32Max(length_error, F32Abs(V3Length(d) - L));
        spin = F32Max(spin, DynamicsTestAngle(QIdentity(), pose.rotation));
        out_of_plane = F32Max(out_of_plane, F32Abs(d.z));

        const f32 x = d.x;
        if ((x_prev < 0.0f) != (x < 0.0f))
        {
            const f32 crossing = ((f32) (t - 1) + x_prev / (x_prev - x)) * dt;
            crossing_first = (crossing_count == 0) ? crossing : crossing_first;
            crossing_last = crossing;
            crossing_count += 1;

            /* the swing that just ended peaked at max_current */
            if (max_count > 0)
            {
                max_first = (max_count == 1) ? max_current : max_first;
                max_last = max_current;
            }
            max_current = 0.0f;
            max_count += 1;
        }
        max_current = F32Max(max_current, F32Abs(x));
        x_prev = x;
    }

    const f32 period_expected = F32_PI2 * F32Sqrt(L / -g_solver_config->gravity.y);
    const f32 period = (crossing_count >= 2) ? 2.0f * (crossing_last - crossing_first) / (f32) (crossing_count - 1) : 0.0f;
    /*
     * max_first is the peak of the first full half swing, max_last that of the last complete one, (max_count - 2) half
     * periods apart; velocity damping by damp per tick decays the amplitude by sqrt(damp) per tick
     */
    const f32 decay_expected = (max_count >= 3) ? F32Pow(1.0f / (1.0f + g_solver_config->linear_dampening * dt), 0.25f * (f32) (max_count - 2) * period / dt) : 1.0f;
    const f32 decay = (max_count >= 3 && max_first > 0.0f) ? max_last / max_first : 0.0f;
    const u32 ok = crossing_count >= 6 && max_count >= 3
                && length_error <= g_solver_config->linear_slop
                && spin <= 1e-3f && out_of_plane <= 1e-3f
                && F32Abs(period - period_expected) <= 0.01f * period_expected
                && F32Abs(decay - decay_expected) <= 0.02f;
    if (!ok)
    {
        fprintf(stderr, "pendulum: length error %g, spin %g, out of plane %g, period %f (expected %f, %u crossings), decay %f (expected %f, %u maxima)\n",
                length_error, spin, out_of_plane, period, period_expected, crossing_count, decay, decay_expected, max_count);
    }
    TEST_NOT_ZERO(ok);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* a box thrown up on a slack rope: free between the bounds, stopped at each bound by the range rows (file comment) */
static struct test_Output DynamicsSlackRopeTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* frames at the centres of mass: no torque, the motion stays on the anchor's vertical */
    const intv range = Intv(1.0f, 3.0f);
    const f32 distance_0 = 2.0f;
    const f32 throw_speed = 6.0f;
    const v3 anchor = V3(0.0f, 8.0f, 0.0f);
    const struct dynamics_TestBox scene[2] =
    {
        { .position = anchor, .rotation = QIdentity(), .hw = V3(0.25f, 0.25f, 0.25f), .dynamic = 0, .friction = 0.5f },
        { .position = V3Sub(anchor, V3(0.0f, distance_0, 0.0f)), .rotation = QIdentity(), .hw = V3(0.25f, 0.25f, 0.25f), .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId id[2];
    for (u32 i = 0; i < 2; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }
    const ds_Transform frame = ds_TransformIdentity();
    struct ds_DistanceJointPrefab prefab;
    ds_DistanceJointPrefabRange(&prefab, range);
    TEST_NOT_ZERO(!ds_JointIdNullCheck(ds_DistanceJointAdd(&pipeline, &prefab, id[0], &frame, id[1], &frame)));
    {
        const struct ds_Body *body = ds_BodyLookup(&pipeline, id[1]).address;
        TEST_NOT_ZERO(body->set == SOLVER_SET_ACTIVE);
        pipeline.solver_set_pool.buf[body->set].body_compute_pool.buf[body->sim].linear_velocity = V3(0.0f, throw_speed, 0.0f);
    }

    /*
     * 5 s. The model is the distance along the anchor's vertical: the solver's gravity and damping step
     * v <- (v + g dt) / (1 + d dt), then the range rows clamp the separating velocity to what reaches a bound at
     * the end of the tick, -(distance - low) / dt <= v <= (high - distance) / dt; inside the range, far from the
     * bounds, the joint applies nothing.
     */
    const f32 dt = pipeline.timestep;
    const f32 g = -g_solver_config->gravity.y;
    const f32 damp = 1.0f / (1.0f + g_solver_config->linear_dampening * dt);
    const u32 tick_count = 300;
    const u32 rest_ticks = 60;
    f32 distance_model = distance_0;
    f32 velocity_model = -throw_speed;
    f32 model_error = 0.0f;
    f32 violation = 0.0f;
    f32 low_gap = range.high;
    f32 high_gap = range.high;
    f32 rest_error = 0.0f;
    f32 lateral = 0.0f;
    f32 spin = 0.0f;
    for (u32 t = 1; t <= tick_count; ++t)
    {
        velocity_model = (velocity_model + g * dt) * damp;
        const f32 velocity_min = -F32Max(0.0f, distance_model - range.low) / dt;
        const f32 velocity_max = F32Max(0.0f, range.high - distance_model) / dt;
        velocity_model = F32Clamp(velocity_model, velocity_min, velocity_max);
        distance_model += velocity_model * dt;

        SnapshotTestTick(&pipeline);
        const ds_TransformWorld pose = DynamicsTestPose(&pipeline, id[1]);
        const v3 d = V3F64Demote(V3F64Sub(pose.position, V3Promote(anchor)));
        const f32 distance = V3Length(d);
        model_error = F32Max(model_error, F32Abs(distance - distance_model));
        violation = F32Max(violation, F32Max(range.low - distance, distance - range.high));
        low_gap = F32Min(low_gap, F32Abs(distance - range.low));
        high_gap = F32Min(high_gap, F32Abs(distance - range.high));
        rest_error = (t > tick_count - rest_ticks) ? F32Max(rest_error, F32Abs(distance - range.high)) : rest_error;
        lateral = F32Max(lateral, F32Max(F32Abs(d.x), F32Abs(d.z)));
        spin = F32Max(spin, DynamicsTestAngle(QIdentity(), pose.rotation));
    }

    const f32 slop = g_solver_config->linear_slop;
    const u32 ok = model_error <= 1e-3f
                && violation <= slop
                && low_gap <= slop && high_gap <= slop
                && rest_error <= slop
                && lateral <= 1e-3f && spin <= 1e-3f;
    if (!ok)
    {
        fprintf(stderr, "slack rope: model error %g, violation %g, gap to low %g, to high %g, rest error %g, lateral %g, spin %g\n",
                model_error, violation, low_gap, high_gap, rest_error, lateral, spin);
    }
    TEST_NOT_ZERO(ok);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* a frictionless box sliding down a 30 degree slope against the damped free-slide recurrence (file comment) */
static struct test_Output DynamicsSlideTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    const f32 angle = 30.0f * F32_PI / 180.0f;
    const q slope = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), angle);
    const v3 n = QUnitRotate(slope, V3(0.0f, 1.0f, 0.0f));
    const v3 gravity = g_solver_config->gravity;
    const v3 downhill = V3Normalize(V3Sub(gravity, V3Scale(n, V3Dot(n, gravity))));
    const struct dynamics_TestBox scene[2] =
    {
        { .position = V3Scale(n, -0.5f), .rotation = slope, .hw = V3(8.0f, 0.5f, 4.0f), .dynamic = 0, .friction = 0.0f },
        { .position = V3Scale(n, 0.5f), .rotation = slope, .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.0f },
    };
    ds_BodyId id[2];
    for (u32 i = 0; i < 2; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }

    /* each tick adds the downhill gravity and damps (ds_BodyUpdateSolverDataRange); the contacts only cancel the normal part */
    const f32 g_t = V3Dot(gravity, downhill);
    const f32 damp = 1.0f / (1.0f + g_solver_config->linear_dampening * pipeline.timestep);
    f32 expected = 0.0f;
    for (u32 t = 0; t < 60; ++t)
    {
        SnapshotTestTick(&pipeline);
        expected = (expected + g_t * pipeline.timestep) * damp;
    }
    const v3 v = SolverTestVelocity(&pipeline, id[1]);
    const f32 along = V3Dot(v, downhill);
    const v3 across = V3Sub(v, V3Scale(downhill, along));
    if (!(F32Abs(along - expected) <= 0.01f * expected && V3Length(across) <= 0.01f * expected))
    {
        fprintf(stderr, "slide: speed %f downhill (expected %f), %f across\n", along, expected, V3Length(across));
        TEST_FAILURE;
    }
    const struct ds_Body *box = ds_BodyLookup(&pipeline, id[1]).address;
    const v3 w = pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf[box->sim].angular_velocity;
    TEST_NOT_ZERO(V3Length(w) <= 0.01f);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* a box dropped flat on the ground with restitution 1 bounces with its impact speed, with 0 it stops (file comment) */
static struct test_Output DynamicsBounceTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    const struct dynamics_TestBox scene[3] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f, .restitution = 0.0f },
        { .position = V3(-4.0f, 3.0f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f, .restitution = 1.0f },
        { .position = V3(4.0f, 3.0f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f, .restitution = 0.0f },
    };
    ds_BodyId id[3];
    for (u32 i = 0; i < 3; ++i)
    {
        id[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, scene + i, (v3f64) { 0 }, i);
    }

    /* tick until the bouncy box moves up; the impact speed is the last downward speed plus one tick of gravity */
    const f32 g_dt = -g_solver_config->gravity.y * pipeline.timestep;
    f32 down = 0.0f;
    f32 up = 0.0f;
    u32 t;
    for (t = 0; t < 120; ++t)
    {
        SnapshotTestTick(&pipeline);
        const f32 vy = SolverTestVelocity(&pipeline, id[1]).y;
        if (vy > 0.0f)
        {
            up = vy;
            break;
        }
        down = -vy;
    }
    TEST_NOT_ZERO(t < 120 && down > 5.0f);
    if (!(0.9f * down <= up && up <= 1.05f * (down + g_dt)))
    {
        fprintf(stderr, "bounce: impact speed at least %f, bounced at %f\n", down, up);
        TEST_FAILURE;
    }
    /* the dead box, dropped at the same time, has no bounce: its speed after the impact tick stays within a few percent of the impact speed */
    const f32 dead = F32Abs(SolverTestVelocity(&pipeline, id[2]).y);
    TEST_NOT_ZERO(dead <= 0.05f * down);
    for (u32 k = 0; k < 30; ++k)
    {
        SnapshotTestTick(&pipeline);
        TEST_NOT_ZERO(F32Abs(SolverTestVelocity(&pipeline, id[2]).y) <= 0.05f * down);
    }

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* the first tick with more bodies than the pipeline's initial size (file comment) */
static struct test_Output DynamicsFirstTickTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* a ground and a 10 by 10 grid of boxes on it: 101 bodies, the pools grow past 64 before the first tick */
    const struct dynamics_TestBox ground = { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(16.0f, 1.0f, 16.0f), .dynamic = 0, .friction = 0.5f };
    DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, &ground, (v3f64) { 0 }, 0);
    for (u32 i = 0; i < 100; ++i)
    {
        const struct dynamics_TestBox box = { .position = V3(-13.5f + 3.0f * (f32) (i % 10), 0.5f, -13.5f + 3.0f * (f32) (i / 10)), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f };
        DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, &box, (v3f64) { 0 }, 1 + i);
    }
    TEST_NOT_ZERO(pipeline.body_pool.length > 64 && pipeline.body_pool.count == 101);

    SnapshotTestTick(&pipeline);
    ds_CGraphValidate(&pipeline);
    ds_IslandValidateAll(&pipeline);
    TEST_NOT_ZERO(GraphTestCheck(&pipeline) == 100 && pipeline.island_pool.count == 100);
    for (u32 c = 0; c < CG_COLOR_COUNT; ++c)
    {
        TEST_NOT_ZERO(c == CG_SERIAL_COLOR || pipeline.cgraph.color[c].body_bitset.bit_count >= pipeline.body_pool.length);
    }

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* a flushed pipeline is empty and simulates again; a split flagged before the flush does not survive it (file comment) */
static struct test_Output DynamicsFlushTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    /* three boxes in a row on a ground; without the middle one the island is flagged, and once rested, chosen for a split */
    const struct dynamics_TestBox row[4] =
    {
        { .position = V3(0.0f, -1.0f, 0.0f), .rotation = QIdentity(), .hw = V3(8.0f, 1.0f, 8.0f), .dynamic = 0, .friction = 0.5f },
        { .position = V3(-0.98f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.0f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
        { .position = V3(0.98f, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.5f },
    };
    ds_BodyId body[4];
    for (u32 i = 0; i < 4; ++i)
    {
        body[i] = DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, row + i, (v3f64) { 0 }, i);
    }
    SnapshotTestTick(&pipeline);
    ds_BodyRemove(&mem, &pipeline, body[2]);
    u32 t;
    for (t = 0; t < 600 && ds_IslandIdNullCheck(pipeline.island_to_split); ++t)
    {
        SnapshotTestTick(&pipeline);
    }
    TEST_NOT_ZERO(t < 600 && !ds_IslandIdNullCheck(pipeline.island_to_split));

    /* the flush empties everything, the flagged split included */
    ds_DynamicsFlush(&pipeline);
    TEST_NOT_ZERO(pipeline.body_pool.count == 0 && pipeline.shape_pool.count == 0 && pipeline.contact_pool.count == 0 && pipeline.island_pool.count == 0);
    TEST_NOT_ZERO(pipeline.frames_completed == 0 && ds_IslandIdNullCheck(pipeline.island_to_split));
    TEST_NOT_ZERO(SnapshotTestSleepingSetCount(&pipeline) == 0 && pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_sim_pool.count == 0);

    /* three lone boxes take the island slots the row had, with the ids a fresh pipeline gives; the ticks after the flush must not split any of them */
    for (u32 i = 0; i < 4; ++i)
    {
        const struct dynamics_TestBox box = { .position = V3(-4.0f + 2.0f * (f32) i, 0.5f, 0.0f), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = (i != 0), .friction = 0.5f };
        if (i == 0)
        {
            DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, row + 0, (v3f64) { 0 }, 4);
        }
        else
        {
            DynamicsTestBoxAdd(&mem, &pipeline, &cshape_db, &box, (v3f64) { 0 }, 4 + i);
        }
    }
    for (t = 0; t < 60; ++t)
    {
        SnapshotTestTick(&pipeline);
        ds_IslandValidateAll(&pipeline);
    }
    TEST_NOT_ZERO(pipeline.island_pool.count == 3 && GraphTestCheck(&pipeline) == 3);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

/* the numerics config's push: pending values clamped and applied, the derived values from the applied angles (file comment) */
static struct test_Output DynamicsNumericsConfigTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct ds_NumericsConfig *global_before = g_numerics_config;

    /* the defaults and their derived values (tan^2 2.86, cos 3.62, sin^2 0.5 degrees) */
    struct ds_NumericsConfig config = ds_NumericsConfigDefault();
    ds_NumericsConfigPush(&config);
    TEST_NOT_ZERO(g_numerics_config == &config);
    TEST_NOT_ZERO(config.dbvh_reinsert_threshold == config.dbvh_reinsert_threshold_pending && config.gjk_max_iterations == config.gjk_max_iterations_pending && config.gjk_max_iterations >= 1);
    TEST_NOT_ZERO(F32Abs(config.capsule_parallel_tan_sq - 0.00249579f) < 1e-6f);
    TEST_NOT_ZERO(F32Abs(config.face_parallel_cos - 0.99800475f) < 1e-6f);
    TEST_NOT_ZERO(F32Abs(config.edge_parallel_sin_sq - 7.6152422e-5f) < 1e-8f);

    /* out-of-range pending values are clamped into the applied ones and back into the pending ones */
    config.dbvh_reinsert_threshold_pending = 2.0f;
    config.gjk_max_iterations_pending = 0;
    config.capsule_parallel_max_degrees_pending = 90.0f;
    config.face_parallel_max_degrees_pending = -1.0f;
    config.edge_parallel_max_degrees_pending = 45.0f;
    config.sat_face_bias_pending = -1.0f;
    config.sat_cache_tolerance_pending = 3.0f;
    ds_NumericsConfigPush(&config);
    TEST_NOT_ZERO(config.dbvh_reinsert_threshold == 1.0f && config.dbvh_reinsert_threshold_pending == 1.0f);
    TEST_NOT_ZERO(config.gjk_max_iterations == 1 && config.gjk_max_iterations_pending == 1);
    TEST_NOT_ZERO(config.capsule_parallel_max_degrees == 45.0f && F32Abs(config.capsule_parallel_tan_sq - 1.0f) < 1e-5f);
    TEST_NOT_ZERO(config.face_parallel_max_degrees == 0.0f && config.face_parallel_cos == 1.0f);
    TEST_NOT_ZERO(config.edge_parallel_max_degrees == 45.0f && F32Abs(config.edge_parallel_sin_sq - 0.5f) < 1e-6f);
    TEST_NOT_ZERO(config.sat_face_bias == 0.0f && config.sat_cache_tolerance == 3.0f);

    ds_NumericsConfigPop();
    TEST_NOT_ZERO(g_numerics_config == NULL);
    g_numerics_config = global_before;
    return output;
}


/*
 * Warm start by id (agent_notes/warm_start_design.txt): the metrics count the constraint points with a cache and
 * those matched. Three scenes, the static ground always shape 0 of its contacts (added first): the aligned four-box
 * stack (degenerate clipping: vertices on the side planes), a resting pile (box pyramid, lying and standing
 * capsules, spheres) and a 20 degree slope with a box, a capsule and a sphere sliding and rolling down. Every
 * manifold's ids must be unique; prints the match rates over the measured ticks and checks their consistency.
 */
struct dynamics_WarmShape
{
    struct c_Shape  shape;      /* type and its parameters; the hull is built from hw */
    v3              hw;
    v3              position;
    q               rotation;
    u32             dynamic;
    f32             friction;
};

static ds_BodyId DynamicsWarmShapeAdd(struct arena *mem, struct ds_Dynamics *pipeline, ds_SDB(c_Shape) *cshape_db, const struct dynamics_WarmShape *ws, const u32 index)
{
    const utf8 id = Utf8Format(mem, "warm%u", index);
    const struct slot slot = ds_SDBAddAlias(*cshape_db, id);
    struct c_Shape *cshape = slot.address;
    cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
    cshape->type = ws->shape.type;
    switch (cshape->type)
    {
        case C_SHAPE_SPHERE:    cshape->sphere = ws->shape.sphere; break;
        case C_SHAPE_CAPSULE:   cshape->capsule = ws->shape.capsule; break;
        default:                cshape->hull = DcelBox(mem, ws->hw); break;
    }
    c_ShapeUpdateMassProperties(cshape);

    const struct ds_BodyPrefab body_prefab = { .dynamic = ws->dynamic };
    const ds_TransformWorld t_world = { .rotation = ws->rotation, .position = V3Promote(ws->position) };
    const ds_BodyId body = ds_BodyAdd(pipeline, &body_prefab, &t_world, index);
    const struct ds_ShapePrefab shape_prefab = { .cshape = slot.index, .density = 1.0f, .restitution = 0.0f, .friction = ws->friction, .margin = 0.5f };
    const ds_Transform t_local = ds_TransformIdentity();
    ds_ShapeAdd(pipeline, &shape_prefab, &t_local, body);
    return body;
}

#define DYNAMICS_WARM_SHAPE_MAX 64

struct dynamics_WarmScene
{
    struct dynamics_WarmShape   shape[DYNAMICS_WARM_SHAPE_MAX];
    u32                         count;
};

static void DynamicsWarmSceneAdd(struct dynamics_WarmScene *scene, const struct dynamics_WarmShape *ws)
{
    if (scene->count < DYNAMICS_WARM_SHAPE_MAX)
    {
        scene->shape[scene->count++] = *ws;
    }
}

/* run the scene settle + measure ticks; metrics summed over the measured ticks */
static void DynamicsWarmSceneRun(struct ds_DynamicsMetrics *sum, const struct dynamics_WarmScene *scene, const u32 settle_ticks, const u32 measure_ticks)
{
    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 128, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 128, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    for (u32 i = 0; i < scene->count; ++i)
    {
        DynamicsWarmShapeAdd(&mem, &pipeline, &cshape_db, scene->shape + i, i);
    }

    ds_DynamicsMetricsFlush(sum);
    u32 duplicate_manifolds = 0;
    u32 manifolds = 0;
    for (u32 t = 0; t < settle_ticks + measure_ticks; ++t)
    {
        ds_DynamicsTick(&pipeline);
        ds_PoolFlush(pipeline.event_pool);
        DLLFlush(pipeline.event_list);
        if (settle_ticks <= t)
        {
            ds_DynamicsMetricsAdd(sum, &pipeline.metrics);
        }

        /* ids unique within every manifold */
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            const struct ds_CGraphColor *color = pipeline.cgraph.color + ci;
            for (u32 i = 0; i < color->contact_pool.count; ++i)
            {
                const struct ds_Contact *c = pipeline.contact_pool.buf + color->contact_pool.buf[i];
                for (u32 mi = 0; mi < c->narrowphase.manifold_count; ++mi)
                {
                    const struct ds_SolverManifold *m = c->narrowphase.manifold + mi;
                    u32 duplicate = 0;
                    for (u32 a = 0; a < m->point_count; ++a)
                    {
                        for (u32 b = a + 1; b < m->point_count; ++b)
                        {
                            duplicate |= c_PointIdEqual(m->id[a], m->id[b]);
                        }
                    }
                    manifolds += 1;
                    if (duplicate)
                    {
                        if (duplicate_manifolds < 4)
                        {
                            fprintf(stdout, "\tduplicate ids, tick %u, %u points:", t, m->point_count);
                            for (u32 a = 0; a < m->point_count; ++a) { fprintf(stdout, " %08x", m->id[a].val); }
                            fprintf(stdout, "\n");
                        }
                        duplicate_manifolds += 1;
                    }
                }
            }
        }
    }
    fprintf(stdout, "\tmanifolds with duplicate ids: %u of %u\n", duplicate_manifolds, manifolds);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsSleepEnable(&pipeline);
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(prefab_db);
    ds_SDBDealloc(cshape_db);
    ArenaFree(&mem);
}

static u32 DynamicsWarmMetricsReport(const char *name, const struct ds_DynamicsMetrics *m, const u32 ticks)
{
    const f32 points = (f32) m->warm_point_count;
    fprintf(stdout, "\twarm start, %s: %u points over %u ticks (%.1f per tick); matched by id %.2f%%\n",
            name, m->warm_point_count, ticks, points / (f32) ticks, 100.0f * (f32) m->warm_match_id / points);
    return m->warm_point_count != 0 && m->warm_match_id <= m->warm_point_count;
}

static struct test_Output DynamicsWarmStartMetricsTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL };
    const struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 0.5f, .radius = 0.25f } };
    const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.3f } };
    const q lying = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.5f * F32_PI);

    /* resting pile: the ground first (shape 0 of every ground contact) */
    struct dynamics_WarmScene pile = { 0 };
    DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(10.0f, 0.5f, 10.0f), .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .dynamic = 0, .friction = 0.6f });
    const u32 layers = 4;
    for (u32 k = 0; k < layers; ++k)
    {
        const u32 n = layers - k;
        const f32 offset = -0.5f * (f32) (n - 1);
        for (u32 i = 0; i < n; ++i)
        {
            for (u32 j = 0; j < n; ++j)
            {
                DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(0.5f, 0.5f, 0.5f), .position = V3(offset + (f32) i, (f32) k + 0.5f, offset + (f32) j), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });
            }
        }
    }
    for (u32 i = 0; i < 4; ++i)
    {
        const f32 x = 4.0f + 1.5f * (f32) i;
        DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = capsule, .position = V3(x, 0.25f, 3.0f), .rotation = lying, .dynamic = 1, .friction = 0.6f });
        DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = capsule, .position = V3(x, 0.75f, -3.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });
        DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = sphere, .position = V3(-x, 0.3f, 3.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });
        DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(0.5f, 0.5f, 0.5f), .position = V3(-x, 0.5f, -3.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });
    }
    /* a box on a lying capsule and a sphere on a box: dynamic-dynamic pairs of mixed types */
    DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(0.5f, 0.5f, 0.5f), .position = V3(4.0f, 1.0f, 3.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });
    DynamicsWarmSceneAdd(&pile, &(struct dynamics_WarmShape) { .shape = sphere, .position = V3(-4.0f, 1.3f, -3.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 0.6f });

    /* slope: 20 degrees, the ground first; low friction so the box keeps sliding; the capsule lies across the fall line */
    const f32 angle = 20.0f * F32_PI / 180.0f;
    const q slope = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), angle);
    const v3 n = QUnitRotate(slope, V3(0.0f, 1.0f, 0.0f));
    const v3 up = QUnitRotate(slope, V3(1.0f, 0.0f, 0.0f));     /* uphill along the slope */
    struct dynamics_WarmScene ramp = { 0 };
    DynamicsWarmSceneAdd(&ramp, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(60.0f, 0.5f, 10.0f), .position = V3Scale(n, -0.5f), .rotation = slope, .dynamic = 0, .friction = 0.1f });
    for (u32 i = 0; i < 4; ++i)
    {
        const v3 base = V3Scale(up, 40.0f - 3.0f * (f32) i);
        DynamicsWarmSceneAdd(&ramp, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(0.5f, 0.5f, 0.5f), .position = V3Add(base, V3Add(V3Scale(n, 0.5f), V3(0.0f, 0.0f, -4.0f))), .rotation = slope, .dynamic = 1, .friction = 0.1f });
        DynamicsWarmSceneAdd(&ramp, &(struct dynamics_WarmShape) { .shape = capsule, .position = V3Add(base, V3Add(V3Scale(n, 0.25f), V3(0.0f, 0.0f, 0.0f))), .rotation = QMul(slope, lying), .dynamic = 1, .friction = 0.1f });
        DynamicsWarmSceneAdd(&ramp, &(struct dynamics_WarmShape) { .shape = sphere, .position = V3Add(base, V3Add(V3Scale(n, 0.3f), V3(0.0f, 0.0f, 4.0f))), .rotation = QIdentity(), .dynamic = 1, .friction = 0.1f });
    }

    /* the aligned four-box stack of DynamicsRestingStackTest: coincident edges, degenerate clipping */
    struct dynamics_WarmScene stack = { 0 };
    DynamicsWarmSceneAdd(&stack, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(5.0f, 0.5f, 5.0f), .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .dynamic = 0, .friction = 0.8f });
    for (u32 i = 1; i < 5; ++i)
    {
        DynamicsWarmSceneAdd(&stack, &(struct dynamics_WarmShape) { .shape = box, .hw = V3(0.5f, 0.5f, 0.5f), .position = V3(0.0f, (f32) i - 0.5f, 0.0f), .rotation = QIdentity(), .dynamic = 1, .friction = 1.0f });
    }

    struct ds_DynamicsMetrics m;
    DynamicsWarmSceneRun(&m, &stack, 0, DYNAMICS_TEST_SETTLE_TICKS);
    TEST_NOT_ZERO(DynamicsWarmMetricsReport("aligned stack, settling", &m, DYNAMICS_TEST_SETTLE_TICKS));
    DynamicsWarmSceneRun(&m, &stack, DYNAMICS_TEST_SETTLE_TICKS, DYNAMICS_TEST_MEASURE_TICKS);
    TEST_NOT_ZERO(DynamicsWarmMetricsReport("aligned stack", &m, DYNAMICS_TEST_MEASURE_TICKS));
    DynamicsWarmSceneRun(&m, &pile, DYNAMICS_TEST_SETTLE_TICKS, DYNAMICS_TEST_MEASURE_TICKS);
    TEST_NOT_ZERO(DynamicsWarmMetricsReport("resting pile", &m, DYNAMICS_TEST_MEASURE_TICKS));
    DynamicsWarmSceneRun(&m, &ramp, 30, 150);
    TEST_NOT_ZERO(DynamicsWarmMetricsReport("slope", &m, 150));

    return output;
}


/*
 * Warm start on sliding contacts (agent_notes/warm_start_design.txt): a box, a sphere and a capsule lying across
 * the fall line on a static 20 degree slope added first (shape 0 of every contact), friction 0.3 (tan 20 deg = 0.36:
 * the box slides, the others roll). Prints, over the run, the final downhill speed (the box against the Coulomb
 * recurrence v = (v + (g sin a - mu g cos a) dt) / (1 + d dt) from rest), the height above the slope's surface
 * (min, max), its largest tick-to-tick change, the box's largest angular speed and the rollers' v / (w r).
 * A measurement: no bounds.
 */
static f64 SlideWarmAbs(const f64 a) { return (a < 0.0) ? -a : a; }

static ds_BodyId SlideWarmAdd(struct arena *mem, struct ds_Dynamics *pipeline, ds_SDB(c_Shape) *cshape_db, const enum c_ShapeType type, const v3 hw, const f32 radius, const f32 half_height, const v3 position, const q rotation, const u32 dynamic, const f32 friction, const u32 index)
{
    const utf8 id = Utf8Format(mem, "slide%u", index);
    const struct slot slot = ds_SDBAddAlias(*cshape_db, id);
    struct c_Shape *cshape = slot.address;
    cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
    cshape->type = type;
    switch (type)
    {
        case C_SHAPE_SPHERE:    cshape->sphere.radius = radius; break;
        case C_SHAPE_CAPSULE:   cshape->capsule.radius = radius; cshape->capsule.half_height = half_height; break;
        default:                cshape->hull = DcelBox(mem, hw); break;
    }
    c_ShapeUpdateMassProperties(cshape);

    const struct ds_BodyPrefab body_prefab = { .dynamic = dynamic };
    const ds_TransformWorld t_world = { .rotation = rotation, .position = V3Promote(position) };
    const ds_BodyId body = ds_BodyAdd(pipeline, &body_prefab, &t_world, index);
    const struct ds_ShapePrefab shape_prefab = { .cshape = slot.index, .density = 1.0f, .restitution = 0.0f, .friction = friction, .margin = 0.5f };
    const ds_Transform t_local = ds_TransformIdentity();
    ds_ShapeAdd(pipeline, &shape_prefab, &t_local, body);
    return body;
}

static struct test_Output DynamicsSlideWarmTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDB(c_Shape) cshape_db = { 0 };
    ds_SDBAlloc(NULL, cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 64, NSEC_PER_SEC / 60, 16*1024*1024, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    const f32 angle = 20.0f * F32_PI / 180.0f;
    const f32 mu = 0.3f;
    const q slope = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), angle);
    const v3 n = QUnitRotate(slope, V3(0.0f, 1.0f, 0.0f));
    const v3 up = QUnitRotate(slope, V3(1.0f, 0.0f, 0.0f));
    const v3 gravity = g_solver_config->gravity;
    const v3 downhill = V3Normalize(V3Sub(gravity, V3Scale(n, V3Dot(n, gravity))));
    const q across = QMul(slope, QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), 0.5f * F32_PI));   /* the capsule's axis along z */

    /* the slope's top surface is the plane n . x = 0 */
    SlideWarmAdd(&mem, &pipeline, &cshape_db, C_SHAPE_CONVEX_HULL, V3(60.0f, 0.5f, 10.0f), 0.0f, 0.0f, V3Scale(n, -0.5f), slope, 0, mu, 0);
    const v3 start = V3Scale(up, 40.0f);
    const char *name[3] = { "box", "sphere", "capsule" };
    const f32 offset[3] = { 0.5f, 0.3f, 0.25f };    /* height of the center above the surface when touching */
    ds_BodyId body[3];
    body[0] = SlideWarmAdd(&mem, &pipeline, &cshape_db, C_SHAPE_CONVEX_HULL, V3(0.5f, 0.5f, 0.5f), 0.0f, 0.0f, V3Add(start, V3Add(V3Scale(n, offset[0]), V3(0.0f, 0.0f, -4.0f))), slope, 1, mu, 1);
    body[1] = SlideWarmAdd(&mem, &pipeline, &cshape_db, C_SHAPE_SPHERE, V3(0.0f, 0.0f, 0.0f), 0.3f, 0.0f, V3Add(start, V3Scale(n, offset[1])), QIdentity(), 1, mu, 2);
    body[2] = SlideWarmAdd(&mem, &pipeline, &cshape_db, C_SHAPE_CAPSULE, V3(0.0f, 0.0f, 0.0f), 0.25f, 0.5f, V3Add(start, V3Add(V3Scale(n, offset[2]), V3(0.0f, 0.0f, 4.0f))), across, 1, mu, 3);

    const u32 ticks = 240;
    f64 h_min[3] = { 1e30, 1e30, 1e30 }, h_max[3] = { -1e30, -1e30, -1e30 }, dh_max[3] = { 0.0, 0.0, 0.0 }, h_prev[3] = { 0.0, 0.0, 0.0 };
    f32 w_max[3] = { 0.0f, 0.0f, 0.0f };
    u32 dh_tick[3] = { 0, 0, 0 };
    f32 coulomb = 0.0f;
    const f32 g_t = V3Dot(gravity, downhill);
    const f32 g_n = -V3Dot(gravity, n);
    const f32 damp = 1.0f / (1.0f + g_solver_config->linear_dampening * pipeline.timestep);
    for (u32 t = 0; t < ticks; ++t)
    {
        ds_DynamicsTick(&pipeline);
        ds_PoolFlush(pipeline.event_pool);
        DLLFlush(pipeline.event_list);
        coulomb = (coulomb + (g_t - mu * g_n) * pipeline.timestep) * damp;
        for (u32 i = 0; i < 3; ++i)
        {
            const ds_TransformWorld pose = DynamicsTestPose(&pipeline, body[i]);
            const f64 h = (f64) n.x * pose.position.x + (f64) n.y * pose.position.y + (f64) n.z * pose.position.z - (f64) offset[i];
            if (t && dh_max[i] < SlideWarmAbs(h - h_prev[i]))
            {
                dh_max[i] = SlideWarmAbs(h - h_prev[i]);
                dh_tick[i] = t;
            }
            h_prev[i] = h;
            h_min[i] = (h < h_min[i]) ? h : h_min[i];
            h_max[i] = (h_max[i] < h) ? h : h_max[i];
            const struct ds_Body *b = ds_BodyLookup(&pipeline, body[i]).address;
            const v3 w = pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf[b->sim].angular_velocity;
            w_max[i] = F32Max(w_max[i], V3Length(w));
        }
    }

    for (u32 i = 0; i < 3; ++i)
    {
        const v3 v = SolverTestVelocity(&pipeline, body[i]);
        const struct ds_Body *b = ds_BodyLookup(&pipeline, body[i]).address;
        const v3 w = pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf[b->sim].angular_velocity;
        const f32 along = V3Dot(v, downhill);
        const f32 across_v = V3Length(V3Sub(v, V3Scale(downhill, along)));
        const f32 r = (i == 1) ? 0.3f : 0.25f;
        fprintf(stdout, "\tslide warm, %s: v %.4f m/s downhill (%.4f across); height above slope min %.3e max %.3e, max tick change %.3e at tick %u; w max %.4f",
                name[i], along, across_v, h_min[i], h_max[i], dh_max[i], dh_tick[i], w_max[i]);
        if (i == 0) { fprintf(stdout, "; coulomb %.4f\n", coulomb); }
        else        { fprintf(stdout, "; v / (w r) %.4f\n", along / (V3Length(w) * r)); }
    }

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsFree(&pipeline);
    ds_SDBDealloc(cshape_db);
    ds_SDBDealloc(prefab_db);
    ArenaFree(&mem);
    return output;
}

static struct test_Output (*dynamics_tests[])(struct test_Environment *) =
{
    DynamicsRestingBoxTest,
    DynamicsRestingStackTest,
    DynamicsRestingSlopeTest,
    DynamicsSnapshotTest,
    DynamicsShapeTest,
    DynamicsBodyTest,
    DynamicsContactTest,
    DynamicsIslandTest,
    DynamicsSleepTest,
    DynamicsWakeByContactTest,
    DynamicsGraphTest,
    DynamicsJointTest,
    DynamicsJointIslandTest,
    DynamicsJointCollisionTest,
    DynamicsPendulumTest,
    DynamicsSlackRopeTest,
    DynamicsSlideTest,
    DynamicsBounceTest,
    DynamicsFirstTickTest,
    DynamicsFlushTest,
    DynamicsNumericsConfigTest,
    DynamicsSlideWarmTest,
    DynamicsWarmStartMetricsTest,
};

struct suite_Correctness m_dynamics_suite =
{
	.id = "dynamics",
	.unit_test = dynamics_tests,
	.unit_test_count = sizeof(dynamics_tests) / sizeof(dynamics_tests[0]),
};

struct suite_Correctness *dynamics_correctness_suite = &m_dynamics_suite;

/********************************** Performance **********************************/

#ifdef DS_PERFORMANCE_MEASUREMENT_API

/*
Contact prep (ds_ContactConstraintInitRange) on a settled box pyramid: the real function, every color's full
range on the main thread, per repetition. Inputs are the post-tick state (sims, computes, manifolds), constant
across repetitions; the prep only writes its constraints on the main thread's frame, popped back per repetition.
Checksum: the sum of every point's effective masses.
*/

#define DYNAMICS_PERF_PYRAMID_LAYERS    5
#define DYNAMICS_PERF_SETTLE_TICKS      240

struct dynamics_PerfPrep
{
    struct arena            mem;
    ds_SDB(c_Shape)         cshape_db;
    ds_SDB(ds_BodyPrefab)   prefab_db;
    struct ds_Dynamics      pipeline;
    struct arena *          frame;          /* the main thread's worker frame */
    u32                     contact_count;
    u32                     manifold_count;
    u32                     point_count;
    f64                     checksum;
};

static void *DynamicsPerfPrepInit(void)
{
    struct dynamics_PerfPrep *in = malloc(sizeof(struct dynamics_PerfPrep));
    memset(in, 0, sizeof(*in));
    in->mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDBAlloc(NULL, in->cshape_db, 256, GROWABLE);
    ds_SDBAlloc(NULL, in->prefab_db, 32, GROWABLE);
    in->pipeline = ds_DynamicsAlloc(&in->mem, 256, NSEC_PER_SEC / 60, 16*1024*1024, &in->cshape_db, &in->prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;

    const u32 layers = DYNAMICS_PERF_PYRAMID_LAYERS;
    u32 index = 0;
    const struct dynamics_TestBox floor = { .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .hw = V3((f32) layers + 2.0f, 0.5f, (f32) layers + 2.0f), .dynamic = 0, .friction = 0.8f };
    DynamicsTestBoxAdd(&in->mem, &in->pipeline, &in->cshape_db, &floor, V3F64(0.0, 0.0, 0.0), index++);
    for (u32 k = 0; k < layers; ++k)
    {
        const u32 n = layers - k;
        const f32 offset = -0.5f * (f32) (n - 1);
        for (u32 i = 0; i < n; ++i)
        {
            for (u32 j = 0; j < n; ++j)
            {
                const struct dynamics_TestBox box = { .position = V3(offset + (f32) i, (f32) k + 0.5f, offset + (f32) j), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.8f };
                DynamicsTestBoxAdd(&in->mem, &in->pipeline, &in->cshape_db, &box, V3F64(0.0, 0.0, 0.0), index++);
            }
        }
    }

    for (u32 t = 0; t < DYNAMICS_PERF_SETTLE_TICKS; ++t)
    {
        ds_DynamicsTick(&in->pipeline);
        ds_PoolFlush(in->pipeline.event_pool);
        DLLFlush(in->pipeline.event_list);
    }

    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        const struct ds_CGraphColor *color = in->pipeline.cgraph.color + ci;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const struct ds_Contact *c = in->pipeline.contact_pool.buf + color->contact_pool.buf[i];
            in->contact_count += 1;
            in->manifold_count += c->narrowphase.manifold_count;
            for (u32 m = 0; m < c->narrowphase.manifold_count; ++m)
            {
                in->point_count += c->narrowphase.manifold[m].point_count;
            }
        }
    }
    fprintf(stdout, "\tbodies %u, contacts %u, manifolds %u, points %u; sizes: contact %u, compute %u, manifold %u, constraint %u, point %u, sim %u, bcomp %u, shape %u\n",
            index, in->contact_count, in->manifold_count, in->point_count,
            (u32) sizeof(struct ds_Contact), (u32) sizeof(struct ds_ContactCompute), (u32) sizeof(struct ds_SolverManifold),
            (u32) sizeof(struct ds_ContactConstraint), (u32) sizeof(struct ds_ContactConstraintPoint),
            (u32) sizeof(struct ds_BodySim), (u32) sizeof(struct ds_BodyCompute), (u32) sizeof(struct ds_Shape));

    in->frame = in->pipeline.worker[ds_ThreadSelfIndex()].frame;
    ArenaPushRecord(in->frame);
    return in;
}

static void DynamicsPerfPrepReset(void *args)
{
    struct dynamics_PerfPrep *in = args;
    ArenaPopRecord(in->frame);
    ArenaPushRecord(in->frame);
}

static void DynamicsPerfPrepTest(void *args)
{
    struct dynamics_PerfPrep *in = args;
    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        ds_ContactConstraintInitRange(&in->pipeline, ci, 0, in->pipeline.cgraph.color[ci].contact_pool.count);
    }
}

static void DynamicsPerfPrepFree(void *args)
{
    struct dynamics_PerfPrep *in = args;
    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        const struct ds_CGraphColor *color = in->pipeline.cgraph.color + ci;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + i;
            for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
            {
                const struct ds_ContactConstraint *cc = ccomp->cc + cci;
                for (u32 p = 0; p < cc->ccp_count; ++p)
                {
                    in->checksum += cc->ccp[p].normal_mass + cc->ccp[p].tangent_mass[0] + cc->ccp[p].tangent_mass[1];
                }
            }
        }
    }
    fprintf(stdout, "\tchecksum %.9e\n", in->checksum);

    ArenaPopRecord(in->frame);
    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsSleepEnable(&in->pipeline);
    ds_DynamicsFree(&in->pipeline);
    ds_SDBDealloc(in->prefab_db);
    ds_SDBDealloc(in->cshape_db);
    ArenaFree(&in->mem);
    free(in);
}

/*
Solver stage baselines on a serialized scene: ns per constraint point of the contact velocity iterate and the
position iterate, per colour group (static colours, dynamic colours, the serial colour), the real range functions
on the main thread. A scene is built and settled once, serialized, and every test deserializes the same bytes:
every stage, group and variant sees identical data. The init then runs the tick's stages up to the measured one
(body update, contact init, warm start; for the position stage also the velocity iterations, the velocity
integration and the position init) and snapshots the compute pool; a repetition restores it (memcpy), then runs
the measured stage over the group's colours, full ranges. Checksum: the sum of the velocities (velocity stage) or
of the position deltas and rotations (position stage) after the last repetition. The bytes line printed by the
init is the data a repetition touches (constraints and both bodies' sim and compute state per contact).
Variants (dynamics_perf_iterate / dynamics_perf_position): the functions compared on the same ranges; adding
one is a table entry. 2026-10-25: the static-colour stages measured 10-12 % (velocity) and 26 % (position) below
the generic ones per point, a dynamic-colour copy without the dummy select nothing (agent_notes/solver_perf.txt).
*/

#define DYNAMICS_PERF_TERRAIN_CELL      2.0f
#define DYNAMICS_PERF_TERRAIN_RELIEF    0.5f
#define DYNAMICS_PERF_TERRAIN_BOXES     8       /* per side */

/* test/test_contact.c: a MC_GRID x MC_GRID cell triangle grid, relief in cells, optional rotation */
struct c_Shape McTerrain(struct arena *mem, const f32 cell, const f32 relief, const q *rotation);

enum dynamics_PerfScene { DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_SCENE_COUNT };
enum dynamics_PerfStage { DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_STAGE_POSITION };
enum dynamics_PerfGroup { DYNAMICS_PERF_GROUP_STATIC, DYNAMICS_PERF_GROUP_DYNAMIC, DYNAMICS_PERF_GROUP_SERIAL };

typedef void (*dynamics_PerfStageRange)(struct ds_Dynamics *, const u32, const u32, const u32);

struct dynamics_PerfVariant
{
    const char *            id;
    dynamics_PerfStageRange range;
};

/* the generic stage runs on any colour; the static stage only on the static colours (one static body per contact) */
enum dynamics_PerfVariantIndex { DYNAMICS_PERF_VARIANT_GENERIC, DYNAMICS_PERF_VARIANT_STATIC };

static const struct dynamics_PerfVariant dynamics_perf_iterate[] =
{
    { .id = "generic", .range = &ds_ContactConstraintIterateRange },
    { .id = "static",  .range = &ds_ContactConstraintIterateStaticRange },
};

static const struct dynamics_PerfVariant dynamics_perf_position[] =
{
    { .id = "generic", .range = &ds_PositionConstraintIterateRange },
    { .id = "static",  .range = &ds_PositionConstraintIterateStaticRange },
};

struct dynamics_PerfSnapshot
{
    struct arena            mem;
    ds_SDB(c_Shape)         cshape_db;
    ds_SDB(ds_BodyPrefab)   prefab_db;
    struct ss               ss;
    u32                     built;
};

static struct dynamics_PerfSnapshot dynamics_perf_snapshot[DYNAMICS_PERF_SCENE_COUNT];

static void DynamicsPerfSnapshotBuild(const enum dynamics_PerfScene scene)
{
    struct dynamics_PerfSnapshot *snap = dynamics_perf_snapshot + scene;
    if (snap->built)
    {
        return;
    }
    snap->built = 1;
    snap->mem = ArenaAlloc(NULL, 64*1024*1024);
    ds_SDBAlloc(NULL, snap->cshape_db, 256, GROWABLE);
    ds_SDBAlloc(NULL, snap->prefab_db, 32, GROWABLE);

    struct arena mem = ArenaAlloc(NULL, 64*1024*1024);
    struct ds_Dynamics pipeline = ds_DynamicsAlloc(&mem, 256, NSEC_PER_SEC / 60, 16*1024*1024, &snap->cshape_db, &snap->prefab_db, g_config->thread_count, g_config->thread_framesize);
    g_solver_config->pending_sleep_enabled = 0;
    /* the terrain's relief draws random numbers: the same scene whatever ran before */
    ThreadXoshiro256InitSequence(1000 + scene);

    u32 index = 0;
    if (scene == DYNAMICS_PERF_SCENE_PYRAMID)
    {
        const u32 layers = DYNAMICS_PERF_PYRAMID_LAYERS;
        const struct dynamics_TestBox floor = { .position = V3(0.0f, -0.5f, 0.0f), .rotation = QIdentity(), .hw = V3((f32) layers + 2.0f, 0.5f, (f32) layers + 2.0f), .dynamic = 0, .friction = 0.8f };
        DynamicsTestBoxAdd(&snap->mem, &pipeline, &snap->cshape_db, &floor, V3F64(0.0, 0.0, 0.0), index++);
        for (u32 k = 0; k < layers; ++k)
        {
            const u32 n = layers - k;
            const f32 offset = -0.5f * (f32) (n - 1);
            for (u32 i = 0; i < n; ++i)
            {
                for (u32 j = 0; j < n; ++j)
                {
                    const struct dynamics_TestBox box = { .position = V3(offset + (f32) i, (f32) k + 0.5f, offset + (f32) j), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.8f };
                    DynamicsTestBoxAdd(&snap->mem, &pipeline, &snap->cshape_db, &box, V3F64(0.0, 0.0, 0.0), index++);
                }
            }
        }
    }
    else
    {
        /* the terrain mesh on a static body at the origin (mesh shapes sit at the body origin, unrotated) */
        const utf8 id = Utf8Inline("terrain");
        const struct slot slot = ds_SDBAddAlias(snap->cshape_db, id);
        struct c_Shape *cshape = slot.address;
        const struct c_Shape terrain = McTerrain(&snap->mem, DYNAMICS_PERF_TERRAIN_CELL, DYNAMICS_PERF_TERRAIN_RELIEF, NULL);
        cshape->id = Utf8CopyBuffered(cshape->id_buf, C_SHAPE_ID_SIZE, id);
        cshape->type = terrain.type;    /* the database's node fields live inside c_Shape: never assign the whole struct */
        cshape->mesh_bvh = terrain.mesh_bvh;
        const struct ds_BodyPrefab body_prefab = { .dynamic = 0 };
        const ds_TransformWorld t_world = { .rotation = QIdentity(), .position = V3F64(0.0, 0.0, 0.0) };
        const ds_BodyId body = ds_BodyAdd(&pipeline, &body_prefab, &t_world, index++);
        const struct ds_ShapePrefab shape_prefab = { .cshape = slot.index, .density = 1.0f, .restitution = 0.0f, .friction = 0.8f, .margin = 0.5f };
        const ds_Transform t_local = ds_TransformIdentity();
        ds_ShapeAdd(&pipeline, &shape_prefab, &t_local, body);

        /* a grid of boxes dropped onto it, inside the grid's extent */
        const f32 extent = DYNAMICS_PERF_TERRAIN_CELL * 12.0f;
        const f32 spacing = (extent - 4.0f) / (f32) (DYNAMICS_PERF_TERRAIN_BOXES - 1);
        for (u32 i = 0; i < DYNAMICS_PERF_TERRAIN_BOXES; ++i)
        {
            for (u32 j = 0; j < DYNAMICS_PERF_TERRAIN_BOXES; ++j)
            {
                const struct dynamics_TestBox box = { .position = V3(2.0f + spacing * (f32) i, 4.0f, 2.0f + spacing * (f32) j), .rotation = QIdentity(), .hw = V3(0.5f, 0.5f, 0.5f), .dynamic = 1, .friction = 0.8f };
                DynamicsTestBoxAdd(&snap->mem, &pipeline, &snap->cshape_db, &box, V3F64(0.0, 0.0, 0.0), index++);
            }
        }
    }

    for (u32 t = 0; t < DYNAMICS_PERF_SETTLE_TICKS; ++t)
    {
        ds_DynamicsTick(&pipeline);
        ds_PoolFlush(pipeline.event_pool);
        DLLFlush(pipeline.event_list);
    }

    const u64 size = ds_DynamicsSerializeSize(&pipeline);
    snap->ss = ss_Alloc(&snap->mem, size);
    ds_DynamicsSerialize(&snap->ss, &pipeline);
    ds_Assert(snap->ss.bit_index == 8*size);
    snap->ss.bit_count = snap->ss.bit_index;
    fprintf(stdout, "\tscene %u: %u bodies, %llu snapshot bytes\n", scene, index, (unsigned long long) size);

    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsSleepEnable(&pipeline);
    ds_DynamicsFree(&pipeline);
    ArenaFree(&mem);
}

struct dynamics_PerfSolve
{
    struct arena            mem;
    struct ds_Dynamics      pipeline;
    struct arena *          frame;          /* the main thread's worker frame */
    enum dynamics_PerfScene scene;
    enum dynamics_PerfStage stage;
    enum dynamics_PerfGroup group;
    u32                     variant;
    u32                     color_first;
    u32                     color_last;
    struct ds_BodyCompute * snapshot;       /* the compute pool before the measured stage */
    struct ds_ContactConstraint *cc_snapshot;/* every colour's constraints before the measured stage (the velocity
                                               iterate accumulates impulses in them), in colour and contact order */
    u32                     cc_total;
    u32                     body_count;
    u32                     contact_count;
    u32                     manifold_count;
    u32                     point_count;
};

/* to_snapshot: the colours' constraints into snapshot; else snapshot back into them (same order both ways) */
static void DynamicsPerfSolveConstraintsCopy(struct ds_ContactConstraint *snapshot, struct ds_Dynamics *pipeline, const u32 to_snapshot)
{
    u32 n = 0;
    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + i;
            const u64 bytes = ccomp->cc_count*sizeof(struct ds_ContactConstraint);
            if (to_snapshot)
            {
                memcpy(snapshot + n, ccomp->cc, bytes);
            }
            else
            {
                memcpy(ccomp->cc, snapshot + n, bytes);
            }
            n += ccomp->cc_count;
        }
    }
}

static void *DynamicsPerfSolveInit(const enum dynamics_PerfScene scene, const enum dynamics_PerfStage stage, const enum dynamics_PerfGroup group, const u32 variant)
{
    DynamicsPerfSnapshotBuild(scene);
    struct dynamics_PerfSnapshot *snap = dynamics_perf_snapshot + scene;

    struct dynamics_PerfSolve *in = malloc(sizeof(struct dynamics_PerfSolve));
    memset(in, 0, sizeof(*in));
    in->mem = ArenaAlloc(NULL, 64*1024*1024);
    in->scene = scene;
    in->stage = stage;
    in->group = group;
    in->variant = variant;

    struct ss view = snap->ss;
    view.bit_index = 0;
    const u32 restored = ds_DynamicsTryDeserialize(&in->mem, &view, &in->pipeline, &snap->cshape_db);
    ds_Assert(restored);
    if (!restored)
    {
        fprintf(stdout, "\tdeserialize failed\n");
        return in;
    }
    struct ds_Dynamics *pipeline = &in->pipeline;
    g_solver_config->pending_sleep_enabled = 0;
    ds_DynamicsSetGlobals(pipeline);
    ds_NumericsConfigPush(&pipeline->numerics_config);
    ds_CGraphFramePrepare(pipeline);
    in->frame = pipeline->worker[ds_ThreadSelfIndex()].frame;
    ArenaPushRecord(in->frame);

    /* the tick's stages up to the measured one, every colour's full range on this thread */
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    in->body_count = active->body_sim_pool.count;
    ds_BodyUpdateSolverDataRange(pipeline, 0, in->body_count);
    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        ds_ContactConstraintInitRange(pipeline, ci, 0, pipeline->cgraph.color[ci].contact_pool.count);
    }
    if (g_solver_config->warmup_solver)
    {
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            ds_ContactConstraintWarmupRange(pipeline, ci, 0, pipeline->cgraph.color[ci].contact_pool.count);
        }
    }
    if (stage == DYNAMICS_PERF_STAGE_POSITION)
    {
        for (u32 i = 0; i < g_solver_config->pgs_iteration_count; ++i)
        {
            for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
            {
                ds_ContactConstraintIterateRange(pipeline, ci, 0, pipeline->cgraph.color[ci].contact_pool.count);
            }
        }
        ds_BodyIntegrateVelocitiesRange(pipeline, 0, in->body_count);
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            ds_PositionConstraintInitAndCacheImpulsesRange(pipeline, ci, 0, pipeline->cgraph.color[ci].contact_pool.count);
        }
    }

    in->snapshot = ArenaPush(&in->mem, in->body_count*sizeof(struct ds_BodyCompute));
    memcpy(in->snapshot, active->body_compute_pool.buf, in->body_count*sizeof(struct ds_BodyCompute));
    for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            in->cc_total += color->contact_compute_pool.buf[i].cc_count;
        }
    }
    in->cc_snapshot = ArenaPush(&in->mem, in->cc_total*sizeof(struct ds_ContactConstraint));
    DynamicsPerfSolveConstraintsCopy(in->cc_snapshot, pipeline, 1);

    switch (group)
    {
        case DYNAMICS_PERF_GROUP_STATIC:    in->color_first = CG_STATIC_COLOR_FIRST;    in->color_last = CG_STATIC_COLOR_LAST;  break;
        case DYNAMICS_PERF_GROUP_DYNAMIC:   in->color_first = CG_DYNAMIC_COLOR_FIRST;   in->color_last = CG_DYNAMIC_COLOR_LAST; break;
        default:                            in->color_first = CG_SERIAL_COLOR;          in->color_last = CG_SERIAL_COLOR;       break;
    }

    u64 bytes = 0;
    for (u32 ci = in->color_first; ci <= in->color_last; ++ci)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + i;
            in->contact_count += 1;
            in->manifold_count += ccomp->cc_count;
            bytes += sizeof(struct ds_ContactCompute) + ccomp->cc_count*sizeof(struct ds_ContactConstraint) + 2*(sizeof(struct ds_BodySim) + sizeof(struct ds_BodyCompute));
            for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
            {
                in->point_count += ccomp->cc[cci].ccp_count;
            }
        }
    }
    fprintf(stdout, "\tscene %u, stage %u, group %u, variant %s: %u active bodies, %u contacts, %u manifolds, %u points, %llu bytes per repetition\n",
            scene, stage, group, (stage == DYNAMICS_PERF_STAGE_VELOCITY) ? dynamics_perf_iterate[variant].id : dynamics_perf_position[variant].id,
            in->body_count, in->contact_count, in->manifold_count, in->point_count, (unsigned long long) bytes);
    return in;
}

static void DynamicsPerfSolveReset(void *args)
{
    struct dynamics_PerfSolve *in = args;
    struct ds_SolverSet *active = in->pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    memcpy(active->body_compute_pool.buf, in->snapshot, in->body_count*sizeof(struct ds_BodyCompute));
    DynamicsPerfSolveConstraintsCopy(in->cc_snapshot, &in->pipeline, 0);
}

static void DynamicsPerfSolveTest(void *args)
{
    struct dynamics_PerfSolve *in = args;
    const dynamics_PerfStageRange range = (in->stage == DYNAMICS_PERF_STAGE_VELOCITY)
        ? dynamics_perf_iterate[in->variant].range
        : dynamics_perf_position[in->variant].range;
    for (u32 ci = in->color_first; ci <= in->color_last; ++ci)
    {
        range(&in->pipeline, ci, 0, in->pipeline.cgraph.color[ci].contact_pool.count);
    }
}

static void DynamicsPerfSolveFree(void *args)
{
    struct dynamics_PerfSolve *in = args;
    const struct ds_SolverSet *active = in->pipeline.solver_set_pool.buf + SOLVER_SET_ACTIVE;
    f64 checksum = 0.0;
    for (u32 i = 0; i < in->body_count; ++i)
    {
        const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;
        if (in->stage == DYNAMICS_PERF_STAGE_VELOCITY)
        {
            checksum += bcomp->linear_velocity.x + bcomp->linear_velocity.y + bcomp->linear_velocity.z
                      + bcomp->angular_velocity.x + bcomp->angular_velocity.y + bcomp->angular_velocity.z;
        }
        else
        {
            checksum += bcomp->delta_position.x + bcomp->delta_position.y + bcomp->delta_position.z
                      + bcomp->rotation.x + bcomp->rotation.y + bcomp->rotation.z + bcomp->rotation.w;
        }
    }
    fprintf(stdout, "\tchecksum %.9e (%u points)\n", checksum, in->point_count);

    ArenaPopRecord(in->frame);
    ds_NumericsConfigPop();
    g_solver_config->pending_sleep_enabled = 1;
    ds_DynamicsSleepEnable(&in->pipeline);
    ds_DynamicsFree(&in->pipeline);
    ArenaFree(&in->mem);
    free(in);
}

#define DYNAMICS_PERF_SOLVE_INIT(name, scene, stage, group)                                 \
    static void *name(void) { return DynamicsPerfSolveInit(scene, stage, group, DYNAMICS_PERF_VARIANT_GENERIC); }
#define DYNAMICS_PERF_SOLVE_INIT_SPECIAL(name, scene, stage, group, variant)                \
    static void *name(void) { return DynamicsPerfSolveInit(scene, stage, group, variant); }
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateStaticPyramidInit,   DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_STATIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateDynamicPyramidInit,  DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_DYNAMIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateSerialPyramidInit,   DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_SERIAL)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionStaticPyramidInit,  DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_STATIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionDynamicPyramidInit, DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_DYNAMIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionSerialPyramidInit,  DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_SERIAL)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateStaticTerrainInit,   DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_STATIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateDynamicTerrainInit,  DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_DYNAMIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfIterateSerialTerrainInit,   DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_SERIAL)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionStaticTerrainInit,  DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_STATIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionDynamicTerrainInit, DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_DYNAMIC)
DYNAMICS_PERF_SOLVE_INIT(DynamicsPerfPositionSerialTerrainInit,  DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_SERIAL)

DYNAMICS_PERF_SOLVE_INIT_SPECIAL(DynamicsPerfIterateStaticPyramidSpecInit,   DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_STATIC,  DYNAMICS_PERF_VARIANT_STATIC)
DYNAMICS_PERF_SOLVE_INIT_SPECIAL(DynamicsPerfPositionStaticPyramidSpecInit,  DYNAMICS_PERF_SCENE_PYRAMID, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_STATIC,  DYNAMICS_PERF_VARIANT_STATIC)
DYNAMICS_PERF_SOLVE_INIT_SPECIAL(DynamicsPerfIterateStaticTerrainSpecInit,   DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_VELOCITY, DYNAMICS_PERF_GROUP_STATIC,  DYNAMICS_PERF_VARIANT_STATIC)
DYNAMICS_PERF_SOLVE_INIT_SPECIAL(DynamicsPerfPositionStaticTerrainSpecInit,  DYNAMICS_PERF_SCENE_TERRAIN, DYNAMICS_PERF_STAGE_POSITION, DYNAMICS_PERF_GROUP_STATIC,  DYNAMICS_PERF_VARIANT_STATIC)


#define DYNAMICS_PERF_SOLVE_TEST(name, init)                                                                    \
    { .id = name, .size = 1, .test = &DynamicsPerfSolveTest, .test_init = &init, .test_reset = &DynamicsPerfSolveReset, .test_free = &DynamicsPerfSolveFree }

/* bytes per repetition: filled in from the first run's counts (see the report in agent_notes) */
#define DYNAMICS_PERF_PREP_BYTES    1

struct test_PerformanceSerial dynamics_serial_test[] =
{
    { .id = "contact_prep_pyramid", .size = DYNAMICS_PERF_PREP_BYTES, .test = &DynamicsPerfPrepTest, .test_init = &DynamicsPerfPrepInit, .test_reset = &DynamicsPerfPrepReset, .test_free = &DynamicsPerfPrepFree },
    DYNAMICS_PERF_SOLVE_TEST("iterate_static_pyramid",         DynamicsPerfIterateStaticPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_static_pyramid_static",  DynamicsPerfIterateStaticPyramidSpecInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_dynamic_pyramid",        DynamicsPerfIterateDynamicPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_serial_pyramid",         DynamicsPerfIterateSerialPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("position_static_pyramid",        DynamicsPerfPositionStaticPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("position_static_pyramid_static", DynamicsPerfPositionStaticPyramidSpecInit),
    DYNAMICS_PERF_SOLVE_TEST("position_dynamic_pyramid",       DynamicsPerfPositionDynamicPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("position_serial_pyramid",        DynamicsPerfPositionSerialPyramidInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_static_terrain",         DynamicsPerfIterateStaticTerrainInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_static_terrain_static",  DynamicsPerfIterateStaticTerrainSpecInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_dynamic_terrain",        DynamicsPerfIterateDynamicTerrainInit),
    DYNAMICS_PERF_SOLVE_TEST("iterate_serial_terrain",         DynamicsPerfIterateSerialTerrainInit),
    DYNAMICS_PERF_SOLVE_TEST("position_static_terrain",        DynamicsPerfPositionStaticTerrainInit),
    DYNAMICS_PERF_SOLVE_TEST("position_static_terrain_static", DynamicsPerfPositionStaticTerrainSpecInit),
    DYNAMICS_PERF_SOLVE_TEST("position_dynamic_terrain",       DynamicsPerfPositionDynamicTerrainInit),
    DYNAMICS_PERF_SOLVE_TEST("position_serial_terrain",        DynamicsPerfPositionSerialTerrainInit),
};

struct suite_Performance m_dynamics_performance_suite =
{
    .id = "dynamics",
    .serial_test = dynamics_serial_test,
    .serial_test_count = sizeof(dynamics_serial_test) / sizeof(dynamics_serial_test[0]),
};

struct suite_Performance *dynamics_performance_suite = &m_dynamics_performance_suite;

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

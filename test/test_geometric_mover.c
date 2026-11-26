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
#include "ds_init.h"
#include "ds_dynamics.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_collision.h"
#include "ds_geometry.h"
#include "ds_platform.h"

/*
Geometric mover tests
=====================
ds_DynamicsGeometricMover in hand-built worlds of static (and resting dynamic) shapes. The pipeline never
ticks; the test plays the game between calls: it adds gravity and sets walking velocities. The mover is a
capsule of radius 0.5 and half height 0.5, so it stands with its center 1 m above the ground.

The mover may sink up to linear_slop into a surface: the plane solver pushes the target out to slop depth,
and the cast stops it between slop and slop - tolerance deep (tolerance = slop / 4). Gaps are signed surface
separations, negative inside; bounds add MOVER_TEST_EPS for GJK and f32 noise.
*/

#define MOVER_TEST_RADIUS       0.5f
#define MOVER_TEST_HALF_HEIGHT  0.5f
#define MOVER_TEST_STAND        1.0     /* the center's height above the ground: half height + radius */
#define MOVER_TEST_EPS          1e-4    /* m */

/* print a failed check and go on, so the test still frees its world; the first failure is the test's */
#define MOVER_CHECK(cond)                                                                           \
    do                                                                                              \
    {                                                                                               \
        if (!(cond))                                                                                \
        {                                                                                           \
            fprintf(stdout, "  check failed (%s:%d): %s\n", __FILE__, __LINE__, #cond);             \
            if (out->success)                                                                       \
            {                                                                                       \
                out->success = 0;                                                                   \
                out->file = __FILE__;                                                               \
                out->line = __LINE__;                                                               \
            }                                                                                       \
        }                                                                                           \
    } while (0)

struct mover_TestWorld
{
    struct arena                mem;
    struct c_ShapeDB            cshape_db;
    struct ds_DynamicsExternal  external;
    struct ds_Dynamics          pipeline;
    v3f64                       origin;     /* positions given to the helpers are relative to it */
    u32                         count;      /* bodies added: unique shape ids and entities */
    f64                         slop;
    f32                         tick;
};

static void MoverTestWorldAlloc(struct mover_TestWorld *world, const v3f64 origin)
{
    world->mem = ArenaAlloc(NULL, 64*1024*1024);
    world->cshape_db = (struct c_ShapeDB) { 0 };
    c_ShapeDBAlloc(NULL, &world->cshape_db, 32, GROWABLE);
    world->external = (struct ds_DynamicsExternal) { .cshape_db = &world->cshape_db };
    world->pipeline = ds_DynamicsAllocDefault(&world->external, NSEC_PER_SEC / 60);
    ds_DynamicsConfigPush(&world->pipeline);
    world->origin = origin;
    world->count = 0;
    world->slop = world->pipeline.config.solver.linear_slop;
    world->tick = 1.0f / 60.0f;
}

static void MoverTestWorldDealloc(struct mover_TestWorld *world)
{
    ds_DynamicsConfigPop(&world->pipeline);
    ds_DynamicsDealloc(&world->pipeline);
    c_ShapeDBDealloc(&world->cshape_db);
    ArenaFree(&world->mem);
}

/* Add a body with the single shape src at origin + position; return the body */
static ds_BodyId MoverTestAdd(struct mover_TestWorld *world, const struct c_Shape *src, const v3 position, const q rotation, const u32 dynamic)
{
    const utf8 id = Utf8Format(&world->mem, "shape%u", world->count);
    const struct slot slot = c_ShapeDBAdd(&world->cshape_db, id, src);
    const struct ds_BodyPrefab body_prefab = { .motion = (dynamic) ? DS_MOTION_DYNAMIC : DS_MOTION_STATIC };
    const ds_TransformWorld t_world = { .rotation = rotation, .position = V3F64Add(world->origin, V3Promote(position)) };
    const ds_BodyId body = ds_BodyAdd(&world->pipeline, &body_prefab, &t_world, world->count);
    const struct ds_ShapePrefab shape_prefab = { .cshape = slot.index, .density = 1.0f, .restitution = 0.0f, .friction = 0.5f };
    const ds_Transform t_local = ds_TransformIdentity();
    ds_ShapeAdd(&world->pipeline, &shape_prefab, &t_local, body);
    world->count += 1;
    return body;
}

static void MoverTestBox(struct mover_TestWorld *world, const v3 hw, const v3 position, const q rotation, const u32 dynamic)
{
    const struct c_Shape src = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(&world->mem, hw) };
    MoverTestAdd(world, &src, position, rotation, dynamic);
}

static void MoverTestSphere(struct mover_TestWorld *world, const f32 radius, const v3 position)
{
    const struct c_Shape src = { .type = C_SHAPE_SPHERE, .sphere = { .radius = radius } };
    MoverTestAdd(world, &src, position, QIdentity(), 0);
}

/* a capsule along its local y axis */
static void MoverTestCapsule(struct mover_TestWorld *world, const f32 radius, const f32 half_height, const v3 position, const q rotation)
{
    const struct c_Shape src = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = half_height, .radius = radius } };
    MoverTestAdd(world, &src, position, rotation, 0);
}

/* a flat grid of n x n cells in its local y = 0 plane, from (0, 0, 0) to (n cell, 0, n cell), facing local +y */
static void MoverTestGrid(struct mover_TestWorld *world, const u32 n, const f32 cell, const v3 position, const q rotation)
{
    struct c_Shape src = { .type = C_SHAPE_TRI_MESH };
    struct mesh *mesh = &src.mesh;
    const u32 side = n + 1;
    mesh->v_count = side*side;
    mesh->tri_count = 2*n*n;
    mesh->v = ArenaPush(&world->mem, mesh->v_count*sizeof(v3));
    mesh->tri = ArenaPush(&world->mem, mesh->tri_count*sizeof(v3u32));
    for (u32 i = 0; i < side; ++i)
    {
        for (u32 j = 0; j < side; ++j)
        {
            mesh->v[i*side + j] = V3(cell*(f32) i, 0.0f, cell*(f32) j);
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
            mesh->tri[t++] = (v3u32) { .buf = { a, c, b } };
            mesh->tri[t++] = (v3u32) { .buf = { b, c, d } };
        }
    }

    MeshConstruct(&world->mem, mesh, 8);
    MoverTestAdd(world, &src, position, rotation, 0);
}

/* rotations taking a grid's +y face normal to -x (local x -> +y) and to -z (local z -> +y) */
static q MoverTestFacingMinusX(void)
{
    return QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.5f*F32_PI);
}

static q MoverTestFacingMinusZ(void)
{
    return QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), -0.5f*F32_PI);
}

static struct ds_GeometricMover MoverTestMover(const struct mover_TestWorld *world, const v3 position, const v3 velocity)
{
    const struct ds_GeometricMover mover =
    {
        .position = V3F64Add(world->origin, V3Promote(position)),
        .velocity = velocity,
        .capsule = { .half_height = MOVER_TEST_HALF_HEIGHT, .radius = MOVER_TEST_RADIUS },
    };
    return mover;
}

/* the mover's center relative to the world origin */
static v3f64 MoverTestCenter(const struct mover_TestWorld *world, const struct ds_GeometricMover *mover)
{
    return V3F64Sub(mover->position, world->origin);
}

/* one game tick: gravity (if on), then the mover */
static void MoverTestStep(struct mover_TestWorld *world, struct ds_GeometricMover *mover, const u32 gravity)
{
    if (gravity)
    {
        mover->velocity = V3Add(mover->velocity, V3Scale(world->pipeline.config.solver.gravity, world->tick));
    }
    ds_DynamicsGeometricMover(&world->pipeline, mover, V3Scale(mover->velocity, world->tick));
}

static f64 MoverTestDistance(const f64 a, const f64 b)
{
    return (a < b) ? b - a : a - b;
}

/* |a| (f32 sqrt of the f64 square: test output and bounds only) */
static f64 MoverTestLength(const v3f64 a)
{
    return (f64) F32Sqrt((f32) (a.x*a.x + a.y*a.y + a.z*a.z));
}

/* x - x is 0 exactly for finite x, NaN otherwise */
static u32 MoverTestFinite(const struct ds_GeometricMover *mover)
{
    const v3f64 p = mover->position;
    const v3 v = mover->velocity;
    return p.x - p.x == 0.0 && p.y - p.y == 0.0 && p.z - p.z == 0.0 && v.x - v.x == 0.0f && v.y - v.y == 0.0f && v.z - v.z == 0.0f;
}

/* no shapes: the mover moves velocity * tick each call and keeps its velocity */
static struct test_Output MoverEmptyWorldTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;

    struct mover_TestWorld world;
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    const v3 start = V3(1.0f, 2.0f, 3.0f);
    const v3 velocity = V3(3.0f, -1.0f, 2.0f);
    struct ds_GeometricMover mover = MoverTestMover(&world, start, velocity);
    for (u32 t = 0; t < 10; ++t)
    {
        MoverTestStep(&world, &mover, 0);
        MOVER_CHECK(mover.velocity.x == velocity.x && mover.velocity.y == velocity.y && mover.velocity.z == velocity.z);
    }

    const v3f64 c = MoverTestCenter(&world, &mover);
    const f64 duration = 10.0 * (f64) world.tick;
    MOVER_CHECK(MoverTestDistance(c.x, start.x + duration*velocity.x) < 1e-5);
    MOVER_CHECK(MoverTestDistance(c.y, start.y + duration*velocity.y) < 1e-5);
    MOVER_CHECK(MoverTestDistance(c.z, start.z + duration*velocity.z) < 1e-5);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Drop the mover from 2 m above the ground (height 0) at (x, z) for 180 ticks under gravity: never deeper than
 * slop, then at rest slop deep with no vertical velocity, no horizontal drift and no jitter.
 */
static void MoverTestFall(struct test_Output *out, struct mover_TestWorld *world, const char *name, const f32 x, const f32 z)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3(x, (f32) MOVER_TEST_STAND + 2.0f, z), V3Zero());
    f64 gap_min = 2.0;
    f64 jitter = 0.0;
    f64 y_previous = 0.0;
    for (u32 t = 0; t < 180; ++t)
    {
        MoverTestStep(world, &mover, 1);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = c.y - MOVER_TEST_STAND;
        gap_min = (gap < gap_min) ? gap : gap_min;
        if (t > 120)
        {
            const f64 step = MoverTestDistance(c.y, y_previous);
            jitter = (jitter < step) ? step : jitter;
        }
        y_previous = c.y;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 gap = c.y - MOVER_TEST_STAND;
    const f64 drift_x = MoverTestDistance(c.x, x);
    const f64 drift_z = MoverTestDistance(c.z, z);
    const f64 drift = (drift_x < drift_z) ? drift_z : drift_x;
    fprintf(stdout, "  %-24s gap min %+.3e  rest %+.3e  vy %+.3e  drift %.3e  jitter %.3e\n", name, gap_min, gap, mover.velocity.y, drift, jitter);
    MOVER_CHECK(gap_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(gap <= -0.75*world->slop + MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(mover.velocity.y, 0.0) < 1e-3);
    MOVER_CHECK(drift < MOVER_TEST_EPS);
    MOVER_CHECK(jitter < 1e-6);
}

/* falls onto a box, a dynamic box (never ticked), a sphere, a lying capsule and a mesh at its features */
static struct test_Output MoverFloorTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(20.0f, 0.5f, 20.0f), V3(0.0f, -0.5f, 0.0f), QIdentity(), 0);
    MoverTestFall(out, &world, "box", 0.3f, -0.7f);
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(5.0f, 0.5f, 5.0f), V3(0.0f, -0.5f, 0.0f), QIdentity(), 1);
    MoverTestFall(out, &world, "dynamic box", 0.3f, -0.7f);
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestSphere(&world, 4.0f, V3(0.0f, -4.0f, 0.0f));
    MoverTestFall(out, &world, "sphere top", 0.0f, 0.0f);
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestCapsule(&world, 1.0f, 5.0f, V3(0.0f, -1.0f, 0.0f), QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.5f*F32_PI));
    MoverTestFall(out, &world, "lying capsule", 0.7f, 0.0f);
    MoverTestWorldDealloc(&world);

    /* 1 m cells from (-10, 0, -10), diagonals along x + z = const */
    const f32 spot[4][2] = { { 0.0f, 0.0f }, { 0.5f, 0.0f }, { 0.5f, 0.5f }, { 0.3f, 0.6f } };
    const char *spot_name[4] = { "mesh vertex", "mesh edge", "mesh diagonal edge", "mesh face" };
    for (u32 i = 0; i < 4; ++i)
    {
        MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
        MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
        MoverTestFall(out, &world, spot_name[i], spot[i][0], spot[i][1]);
        MoverTestWorldDealloc(&world);
    }

    return output;
}

/*
 * One tick at 300 m/s (5 m along +x) from the origin toward an obstacle whose front is at x = 3 on the mover's
 * path: the cast stops the mover slop deep, the plane solver keeps it there and the clip removes vx.
 */
static void MoverTestHit(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3Zero(), V3(300.0f, 0.0f, 0.0f));
    MoverTestStep(world, &mover, 0);
    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 gap = 3.0 - c.x - MOVER_TEST_RADIUS;
    const f64 off_y = MoverTestDistance(c.y, 0.0);
    const f64 off_z = MoverTestDistance(c.z, 0.0);
    const f64 off = (off_y < off_z) ? off_z : off_y;
    fprintf(stdout, "  %-24s gap %+.3e  vx %+.3e  off path %.3e\n", name, gap, mover.velocity.x, off);
    MOVER_CHECK(gap >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(gap <= -0.75*world->slop + MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(mover.velocity.x, 0.0) < 1e-2);
    MOVER_CHECK(off < MOVER_TEST_EPS);
}

/* the same tick past an obstacle off the path (or a mesh's back face): the full 5 m, vx unchanged */
static void MoverTestMiss(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3Zero(), V3(300.0f, 0.0f, 0.0f));
    MoverTestStep(world, &mover, 0);
    const v3f64 c = MoverTestCenter(world, &mover);
    fprintf(stdout, "  %-24s x %.6f  vx %+.3e\n", name, c.x, mover.velocity.x);
    MOVER_CHECK(MoverTestDistance(c.x, 5.0) < MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(c.y, 0.0) < MOVER_TEST_EPS && MoverTestDistance(c.z, 0.0) < MOVER_TEST_EPS);
    MOVER_CHECK(mover.velocity.x == 300.0f);
}

/* every shape kind in front of a 5 m step (the cast, not the planes, must stop the mover), and near misses */
static struct test_Output MoverFastHitTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 2.0f, 2.0f), V3(3.25f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestHit(out, &world, "box");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.01f, 2.0f, 2.0f), V3(3.01f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestHit(out, &world, "thin box");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 2.0f, 2.0f), V3(3.25f, 0.0f, 0.0f), QIdentity(), 1);
    MoverTestHit(out, &world, "dynamic box");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestSphere(&world, 0.5f, V3(3.5f, 0.0f, 0.0f));
    MoverTestHit(out, &world, "sphere");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestCapsule(&world, 0.5f, 1.0f, V3(3.5f, 0.0f, 0.0f), QIdentity());
    MoverTestHit(out, &world, "upright capsule");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestCapsule(&world, 0.5f, 2.0f, V3(3.5f, 0.0f, 0.0f), QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), 0.5f*F32_PI));
    MoverTestHit(out, &world, "crossing capsule");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 10, 1.0f, V3(3.0f, -5.0f, -5.0f), MoverTestFacingMinusX());
    MoverTestHit(out, &world, "mesh wall");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestSphere(&world, 0.5f, V3(3.5f, 0.0f, 1.05f));
    MoverTestMiss(out, &world, "sphere beside");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 2.0f, 2.0f), V3(3.25f, 0.0f, 2.55f), QIdentity(), 0);
    MoverTestMiss(out, &world, "box beside");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 0.5f, 2.0f), V3(3.25f, 1.55f, 0.0f), QIdentity(), 0);
    MoverTestMiss(out, &world, "box above");
    MoverTestWorldDealloc(&world);

    /* back faces are culled: the mover passes a mesh from behind */
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 10, 1.0f, V3(3.0f, 5.0f, -5.0f), QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -0.5f*F32_PI));
    MoverTestMiss(out, &world, "mesh back face");
    MoverTestWorldDealloc(&world);

    return output;
}

/* two obstacles on the path, the nearer one (front at x = 3) in either tree and added first or last */
static struct test_Output MoverHitOrderTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    const v3 hw = V3(0.25f, 2.0f, 2.0f);
    const v3 near = V3(3.25f, 0.0f, 0.0f);
    const v3 far = V3(4.25f, 0.0f, 0.0f);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, hw, near, QIdentity(), 0);
    MoverTestBox(&world, hw, far, QIdentity(), 1);
    MoverTestHit(out, &world, "static near, dynamic far");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, hw, near, QIdentity(), 1);
    MoverTestBox(&world, hw, far, QIdentity(), 0);
    MoverTestHit(out, &world, "dynamic near, static far");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, hw, far, QIdentity(), 0);
    MoverTestBox(&world, hw, near, QIdentity(), 0);
    MoverTestHit(out, &world, "far added first");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 10, 1.0f, V3(3.0f, -5.0f, -5.0f), MoverTestFacingMinusX());
    MoverTestBox(&world, hw, far, QIdentity(), 0);
    MoverTestHit(out, &world, "mesh near, box far");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, hw, near, QIdentity(), 0);
    MoverTestGrid(&world, 10, 1.0f, V3(4.0f, -5.0f, -5.0f), MoverTestFacingMinusX());
    MoverTestHit(out, &world, "box near, mesh far");
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Walk (3, 0, 3) m/s into a wall at x = 2 from 0.5 m off it, 60 ticks without gravity: never deeper than slop;
 * after contact vx is clipped, while vz and the progress along z are unchanged (the mover has no friction).
 */
static void MoverTestSlide(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3(1.0f, 0.0f, 0.0f), V3(3.0f, 0.0f, 3.0f));
    f64 gap_min = 1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        MoverTestStep(world, &mover, 0);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = 2.0 - c.x - MOVER_TEST_RADIUS;
        gap_min = (gap < gap_min) ? gap : gap_min;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 z_expected = 60.0 * 3.0 * (f64) world->tick;
    fprintf(stdout, "  %-24s gap min %+.3e  vx %+.3e  vz %.6f  z %.6f (%.6f)  y %+.3e\n", name, gap_min, mover.velocity.x, mover.velocity.z, c.z, z_expected, c.y);
    MOVER_CHECK(gap_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(mover.velocity.x, 0.0) < 1e-3);
    MOVER_CHECK(MoverTestDistance(mover.velocity.z, 3.0) < 1e-4);
    MOVER_CHECK(MoverTestDistance(c.z, z_expected) < 1e-3);
    MOVER_CHECK(MoverTestDistance(c.y, 0.0) < MOVER_TEST_EPS);
}

/*
 * MoverTestSlide with the walking velocity (3, 0, 3) set every tick, as game input: the wall is pushed into every
 * tick, so the planes, not a clipped velocity, decide the path.
 */
static void MoverTestSlideInput(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3(1.0f, 0.0f, 0.0f), V3Zero());
    f64 gap_min = 1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        mover.velocity = V3(3.0f, 0.0f, 3.0f);
        MoverTestStep(world, &mover, 0);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = 2.0 - c.x - MOVER_TEST_RADIUS;
        gap_min = (gap < gap_min) ? gap : gap_min;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 z_expected = 60.0 * 3.0 * (f64) world->tick;
    fprintf(stdout, "  %-24s gap min %+.3e  z %.6f (%.6f)  y %+.3e\n", name, gap_min, c.z, z_expected, c.y);
    MOVER_CHECK(gap_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(c.z, z_expected) < 1e-3);
    MOVER_CHECK(MoverTestDistance(c.y, 0.0) < MOVER_TEST_EPS);
}

/*
 * Walk (3, ., 3) m/s on the ground (height 0) into a wall at x = 2 for 60 ticks under gravity, the walking
 * velocity set every tick: never deeper than slop into either, still on the ground, the full progress along z.
 */
static void MoverTestSlideOnGround(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3(1.0f, (f32) (MOVER_TEST_STAND - world->slop), 0.0f), V3Zero());
    f64 floor_min = 1.0;
    f64 wall_min = 1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        mover.velocity.x = 3.0f;
        mover.velocity.z = 3.0f;
        MoverTestStep(world, &mover, 1);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 floor_gap = c.y - MOVER_TEST_STAND;
        const f64 wall_gap = 2.0 - c.x - MOVER_TEST_RADIUS;
        floor_min = (floor_gap < floor_min) ? floor_gap : floor_min;
        wall_min = (wall_gap < wall_min) ? wall_gap : wall_min;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 floor_gap = c.y - MOVER_TEST_STAND;
    const f64 z_expected = 60.0 * 3.0 * (f64) world->tick;
    fprintf(stdout, "  %-24s floor min %+.3e  wall min %+.3e  floor end %+.3e  z %.6f (%.6f)\n", name, floor_min, wall_min, floor_gap, c.z, z_expected);
    MOVER_CHECK(floor_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(wall_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(floor_gap <= -0.75*world->slop + MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(c.z, z_expected) < 1e-2);
}

static struct test_Output MoverWallSlideTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestSlide(out, &world, "box wall");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestSlideInput(out, &world, "box wall, input");
    MoverTestWorldDealloc(&world);

    /* its diagonal y + z = 8 stays out of reach */
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 1, 20.0f, V3(2.0f, -10.0f, -2.0f), MoverTestFacingMinusX());
    MoverTestSlide(out, &world, "one-cell mesh wall");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(20.0f, 0.5f, 20.0f), V3(0.0f, -0.5f, 0.0f), QIdentity(), 0);
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 5.0f, 0.0f), QIdentity(), 0);
    MoverTestSlideOnGround(out, &world, "box ground, box wall");
    MoverTestWorldDealloc(&world);

    /* the diagonals x + z = 0 (ground) and y + z = 10 (wall) stay out of reach */
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 1, 20.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    MoverTestGrid(&world, 1, 20.0f, V3(2.0f, 0.0f, -10.0f), MoverTestFacingMinusX());
    MoverTestSlideOnGround(out, &world, "one-cell ground and wall");
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * The input-driven slides above along meshes whose internal edges (shared by coplanar triangles) come within the
 * mover's reach: every triangle gives its own plane, one touched at such an edge a tilted plane. Same checks as on
 * flat walls.
 */
static struct test_Output MoverMeshInternalEdgeTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(2.0f, -10.0f, -10.0f), MoverTestFacingMinusX());
    MoverTestSlideInput(out, &world, "1 m mesh wall");
    MoverTestWorldDealloc(&world);

    /* its diagonal y + z = 0 crosses the path */
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 1, 20.0f, V3(2.0f, -10.0f, -10.0f), MoverTestFacingMinusX());
    MoverTestSlideInput(out, &world, "one-cell wall, diagonal");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    MoverTestGrid(&world, 20, 1.0f, V3(2.0f, 0.0f, -10.0f), MoverTestFacingMinusX());
    MoverTestSlideOnGround(out, &world, "1 m ground, 1 m wall");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 5.0f, 0.0f), QIdentity(), 0);
    MoverTestSlideOnGround(out, &world, "1 m ground, box wall");
    MoverTestWorldDealloc(&world);

    return output;
}

/* Walk (3, 0, 3) m/s into the corner of walls at x = 2 and z = 2 for 60 ticks: wedged slop deep, velocity 0 */
static void MoverTestCorner(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    struct ds_GeometricMover mover = MoverTestMover(world, V3(1.0f, 0.0f, 1.0f), V3(3.0f, 0.0f, 3.0f));
    f64 gap_min = 1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        MoverTestStep(world, &mover, 0);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap_x = 2.0 - c.x - MOVER_TEST_RADIUS;
        const f64 gap_z = 2.0 - c.z - MOVER_TEST_RADIUS;
        gap_min = (gap_x < gap_min) ? gap_x : gap_min;
        gap_min = (gap_z < gap_min) ? gap_z : gap_min;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 gap_x = 2.0 - c.x - MOVER_TEST_RADIUS;
    const f64 gap_z = 2.0 - c.z - MOVER_TEST_RADIUS;
    fprintf(stdout, "  %-24s gap min %+.3e  end %+.3e %+.3e  v (%+.3e, %+.3e, %+.3e)\n", name, gap_min, gap_x, gap_z, mover.velocity.x, mover.velocity.y, mover.velocity.z);
    MOVER_CHECK(gap_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(gap_x <= -0.75*world->slop + MOVER_TEST_EPS);
    MOVER_CHECK(gap_z <= -0.75*world->slop + MOVER_TEST_EPS);
    MOVER_CHECK(V3Length(mover.velocity) < 1e-3f);
}

static struct test_Output MoverCornerTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestBox(&world, V3(20.0f, 5.0f, 0.25f), V3(0.0f, 0.0f, 2.25f), QIdentity(), 0);
    MoverTestCorner(out, &world, "box corner");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(2.0f, -10.0f, -10.0f), MoverTestFacingMinusX());
    MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, -10.0f, 2.0f), MoverTestFacingMinusZ());
    MoverTestCorner(out, &world, "mesh corner");
    MoverTestWorldDealloc(&world);

    return output;
}

/* Jump (10 m/s up, gravity) into a ceiling 2 m above the mover's top: stopped slop deep, vy clipped, then falls */
static struct test_Output MoverCeilingTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(5.0f, 0.5f, 5.0f), V3(0.0f, 3.5f, 0.0f), QIdentity(), 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3Zero(), V3(0.0f, 10.0f, 0.0f));

    const f64 top = MOVER_TEST_HALF_HEIGHT + MOVER_TEST_RADIUS;
    f64 gap_min = 2.0;
    u32 touch = U32_MAX;
    f32 vy_touch = 0.0f;
    f64 y_touch = 0.0;
    f64 y_after = 0.0;
    for (u32 t = 0; t < 60; ++t)
    {
        MoverTestStep(&world, &mover, 1);
        const v3f64 c = MoverTestCenter(&world, &mover);
        const f64 gap = 3.0 - c.y - top;
        gap_min = (gap < gap_min) ? gap : gap_min;
        if (touch == U32_MAX && gap <= 0.0)
        {
            touch = t;
            vy_touch = mover.velocity.y;
            y_touch = c.y;
        }

        if (touch != U32_MAX && t == touch + 10)
        {
            y_after = c.y;
        }
    }

    fprintf(stdout, "  %-24s gap min %+.3e  touch tick %u  vy there %+.3e  10 ticks later %+.3e m\n", "box ceiling", gap_min, touch, vy_touch, y_after - y_touch);
    MOVER_CHECK(touch != U32_MAX);
    MOVER_CHECK(gap_min >= -world.slop - MOVER_TEST_EPS);
    MOVER_CHECK(vy_touch <= 1e-3f);
    MOVER_CHECK(y_after < y_touch - 0.01);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * One tick at (3, 0, 60) m/s from touching a wall at x = 2, 0.6 m before the wall ends: the mover slides past
 * the end and on into the opening, and keeps vx, since the wall is no plane of the last iteration. A wall that
 * goes on clips vx.
 */
static struct test_Output MoverWallEndTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 5.0f), V3(2.25f, 0.0f, -5.0f), QIdentity(), 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3(1.5f, 0.0f, -0.6f), V3(3.0f, 0.0f, 60.0f));
    MoverTestStep(&world, &mover, 0);
    v3f64 c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s x %.6f  z %.6f  vx %+.3e\n", "wall ends", c.x, c.z, mover.velocity.x);
    MOVER_CHECK(mover.velocity.x == 3.0f);
    MOVER_CHECK(c.x > 1.54);
    MOVER_CHECK(MoverTestDistance(c.z, 0.4) < 1e-3);
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 20.0f), V3(2.25f, 0.0f, 0.0f), QIdentity(), 0);
    mover = MoverTestMover(&world, V3(1.5f, 0.0f, -0.6f), V3(3.0f, 0.0f, 60.0f));
    MoverTestStep(&world, &mover, 0);
    c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s x %.6f  z %.6f  vx %+.3e\n", "wall goes on", c.x, c.z, mover.velocity.x);
    MOVER_CHECK(MoverTestDistance(mover.velocity.x, 0.0) < 1e-3);
    MOVER_CHECK(2.0 - c.x - MOVER_TEST_RADIUS >= -world.slop - MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(c.z, 0.4) < 1e-3);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Creep (0.1 m/s) into a wall from touching it: the first ticks stay within slop, push nothing and keep vx;
 * once the target goes deeper than slop the wall pushes and the clip removes vx.
 */
static struct test_Output MoverClipPushedOnlyTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 5.0f), V3(2.25f, 0.0f, 0.0f), QIdentity(), 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3(1.5f, 0.0f, 0.0f), V3(0.1f, 0.0f, 0.0f));
    MoverTestStep(&world, &mover, 0);
    const f32 vx_first = mover.velocity.x;
    f64 gap_min = 1.0;
    for (u32 t = 0; t < 20; ++t)
    {
        MoverTestStep(&world, &mover, 0);
        const v3f64 c = MoverTestCenter(&world, &mover);
        const f64 gap = 2.0 - c.x - MOVER_TEST_RADIUS;
        gap_min = (gap < gap_min) ? gap : gap_min;
    }

    fprintf(stdout, "  %-24s vx after 1 tick %+.3e  after 21 %+.3e  gap min %+.3e\n", "box wall", vx_first, mover.velocity.x, gap_min);
    MOVER_CHECK(vx_first == 0.1f);
    MOVER_CHECK(MoverTestDistance(mover.velocity.x, 0.0) < 1e-6);
    MOVER_CHECK(gap_min >= -world.slop - MOVER_TEST_EPS);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Start inside shapes. Deep (the core inside a box): no plane and no hit, the mover moves freely. Shallow
 * (3 slop into the ground): one call pushes it out to slop depth.
 */
static struct test_Output MoverOverlapTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(1.0f, 1.0f, 1.0f), V3Zero(), QIdentity(), 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3Zero(), V3(1.0f, 0.0f, 0.0f));
    MoverTestStep(&world, &mover, 0);
    v3f64 c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s x %.6f (%.6f)  vx %+.3e\n", "deep in a box", c.x, (f64) world.tick, mover.velocity.x);
    MOVER_CHECK(MoverTestDistance(c.x, (f64) world.tick) < 1e-6);
    MOVER_CHECK(mover.velocity.x == 1.0f);
    MoverTestWorldDealloc(&world);

    const f32 shallow = (f32) (MOVER_TEST_STAND - 3.0*world.slop);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(20.0f, 0.5f, 20.0f), V3(0.0f, -0.5f, 0.0f), QIdentity(), 0);
    mover = MoverTestMover(&world, V3(0.0f, shallow, 0.0f), V3Zero());
    MoverTestStep(&world, &mover, 0);
    c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s gap %+.3e\n", "shallow in a box", c.y - MOVER_TEST_STAND);
    MOVER_CHECK(c.y - MOVER_TEST_STAND >= -world.slop - MOVER_TEST_EPS);
    MOVER_CHECK(c.y - MOVER_TEST_STAND <= -0.75*world.slop + MOVER_TEST_EPS);
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    mover = MoverTestMover(&world, V3(0.3f, shallow, 0.6f), V3Zero());
    MoverTestStep(&world, &mover, 0);
    c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s gap %+.3e\n", "shallow in a mesh", c.y - MOVER_TEST_STAND);
    MOVER_CHECK(c.y - MOVER_TEST_STAND >= -world.slop - MOVER_TEST_EPS);
    MOVER_CHECK(c.y - MOVER_TEST_STAND <= -0.75*world.slop + MOVER_TEST_EPS);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Squeezed between walls 0.9 m apart (the mover is 1 m wide) and pushed into one of them for 30 ticks: the
 * planes cannot all be met; the mover stays finite and between the walls.
 */
static struct test_Output MoverSqueezeTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(0.25f, 5.0f, 5.0f), V3(0.70f, 0.0f, 0.0f), QIdentity(), 0);
    MoverTestBox(&world, V3(0.25f, 5.0f, 5.0f), V3(-0.70f, 0.0f, 0.0f), QIdentity(), 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3Zero(), V3Zero());
    f64 x_max = 0.0;
    u32 finite = 1;
    for (u32 t = 0; t < 30; ++t)
    {
        mover.velocity.x = 1.0f;
        MoverTestStep(&world, &mover, 0);
        finite &= MoverTestFinite(&mover);
        const f64 x = MoverTestDistance(MoverTestCenter(&world, &mover).x, 0.0);
        x_max = (x_max < x) ? x : x_max;
    }

    fprintf(stdout, "  %-24s |x| max %.3e  finite %u\n", "box walls", x_max, finite);
    MOVER_CHECK(finite);
    MOVER_CHECK(x_max <= 0.05 + MOVER_TEST_EPS);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * A 30 degree ramp through the origin, rising along +x. Climb: walk 2 m/s along +x for 60 ticks under gravity,
 * the mover rises and never sinks deeper than slop. Slide: stand still under gravity (the mover has no
 * friction), it slides downhill and stays on the ramp.
 */
static void MoverTestRamp(struct test_Output *out, struct mover_TestWorld *world, const char *name, const v3 n)
{
    /* separation from the ramp's plane: its nearest core point is the bottom end */
    const f64 bottom = MOVER_TEST_HALF_HEIGHT*n.y + MOVER_TEST_RADIUS;
    const v3 start = V3Scale(n, (f32) (bottom - 0.5*world->slop));

    struct ds_GeometricMover mover = MoverTestMover(world, start, V3Zero());
    f64 climb_min = 1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        mover.velocity.x = 2.0f;
        mover.velocity.z = 0.0f;
        MoverTestStep(world, &mover, 1);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = n.x*c.x + n.y*c.y + n.z*c.z - bottom;
        climb_min = (gap < climb_min) ? gap : climb_min;
    }
    const f64 rise = MoverTestCenter(world, &mover).y - start.y;

    mover = MoverTestMover(world, start, V3Zero());
    f64 slide_min = 1.0;
    f64 slide_max = -1.0;
    for (u32 t = 0; t < 60; ++t)
    {
        MoverTestStep(world, &mover, 1);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = n.x*c.x + n.y*c.y + n.z*c.z - bottom;
        slide_min = (gap < slide_min) ? gap : slide_min;
        slide_max = (slide_max < gap) ? gap : slide_max;
    }
    const f64 run = MoverTestCenter(world, &mover).x - start.x;

    fprintf(stdout, "  %-24s climb: gap min %+.3e  rise %.3f m   slide: gap %+.3e .. %+.3e  run %.3f m\n", name, climb_min, rise, slide_min, slide_max, run);
    MOVER_CHECK(climb_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(rise > 0.5);
    MOVER_CHECK(slide_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(slide_max <= 0.05);
    MOVER_CHECK(run < -0.1);
}

static struct test_Output MoverRampTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    const f32 angle = F32_PI / 6.0f;
    const q rotation = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), angle);
    const v3 n = QUnitRotate(rotation, V3(0.0f, 1.0f, 0.0f));

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(20.0f, 0.5f, 5.0f), QUnitRotate(rotation, V3(0.0f, -0.5f, 0.0f)), rotation, 0);
    MoverTestRamp(out, &world, "box ramp", n);
    MoverTestWorldDealloc(&world);

    /* the grid's center (10, 0, 10) at the origin */
    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3Negate(QUnitRotate(rotation, V3(10.0f, 0.0f, 10.0f))), rotation);
    MoverTestRamp(out, &world, "mesh ramp", n);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * Walk (3, ., 1) m/s under gravity for 120 ticks from rest at x = -3, across the ground's seams (mesh edges,
 * the joint of two boxes at x = 0): never deeper than slop, never lifted off the ground, the full progress.
 */
static void MoverTestWalk(struct test_Output *out, struct mover_TestWorld *world, const char *name)
{
    const v3 start = V3(-3.0f, (f32) (MOVER_TEST_STAND - world->slop), 0.25f);
    struct ds_GeometricMover mover = MoverTestMover(world, start, V3Zero());
    f64 gap_min = 1.0;
    f64 gap_max = -1.0;
    for (u32 t = 0; t < 120; ++t)
    {
        mover.velocity.x = 3.0f;
        mover.velocity.z = 1.0f;
        MoverTestStep(world, &mover, 1);
        const v3f64 c = MoverTestCenter(world, &mover);
        const f64 gap = c.y - MOVER_TEST_STAND;
        gap_min = (gap < gap_min) ? gap : gap_min;
        gap_max = (gap_max < gap) ? gap : gap_max;
    }

    const v3f64 c = MoverTestCenter(world, &mover);
    const f64 duration = 120.0 * (f64) world->tick;
    const f64 progress_x = (c.x - start.x) / (3.0*duration);
    const f64 progress_z = (c.z - start.z) / (1.0*duration);
    fprintf(stdout, "  %-24s gap %+.3e .. %+.3e  progress x %.5f  z %.5f\n", name, gap_min, gap_max, progress_x, progress_z);
    MOVER_CHECK(gap_min >= -world->slop - MOVER_TEST_EPS);
    MOVER_CHECK(gap_max <= 0.0);
    MOVER_CHECK(progress_x > 0.99 && progress_x < 1.01);
    MOVER_CHECK(progress_z > 0.99 && progress_z < 1.01);
}

static struct test_Output MoverSeamWalkTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(20.0f, 0.5f, 20.0f), V3(0.0f, -0.5f, 0.0f), QIdentity(), 0);
    MoverTestWalk(out, &world, "box");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(5.0f, 0.5f, 20.0f), V3(-5.0f, -0.5f, 0.0f), QIdentity(), 0);
    MoverTestBox(&world, V3(5.0f, 0.5f, 20.0f), V3(5.0f, -0.5f, 0.0f), QIdentity(), 0);
    MoverTestWalk(out, &world, "two boxes");
    MoverTestWorldDealloc(&world);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestGrid(&world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    MoverTestWalk(out, &world, "mesh");
    MoverTestWorldDealloc(&world);

    return output;
}

/* Drop into a 90 degree V of two boxes from 0.2 m beside its axis: settles centered, slop deep in both, still */
static struct test_Output MoverValleyTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    const f32 s = 0.70710678f;
    const v3 n_left = V3(s, s, 0.0f);
    const v3 n_right = V3(-s, s, 0.0f);
    const q r_left = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), -0.25f*F32_PI);
    const q r_right = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), 0.25f*F32_PI);

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    MoverTestBox(&world, V3(5.0f, 0.5f, 5.0f), V3(-5.5f*s, 4.5f*s, 0.0f), r_left, 0);
    MoverTestBox(&world, V3(5.0f, 0.5f, 5.0f), V3(5.5f*s, 4.5f*s, 0.0f), r_right, 0);
    struct ds_GeometricMover mover = MoverTestMover(&world, V3(0.2f, 3.0f, 0.0f), V3Zero());

    const f64 bottom = MOVER_TEST_HALF_HEIGHT*s + MOVER_TEST_RADIUS;
    f64 gap_min = 1.0;
    for (u32 t = 0; t < 180; ++t)
    {
        MoverTestStep(&world, &mover, 1);
        const v3f64 c = MoverTestCenter(&world, &mover);
        const f64 gap_left = n_left.x*c.x + n_left.y*c.y - bottom;
        const f64 gap_right = n_right.x*c.x + n_right.y*c.y - bottom;
        gap_min = (gap_left < gap_min) ? gap_left : gap_min;
        gap_min = (gap_right < gap_min) ? gap_right : gap_min;
    }

    const v3f64 c = MoverTestCenter(&world, &mover);
    fprintf(stdout, "  %-24s gap min %+.3e  x %+.3e  |v| %.3e\n", "box valley", gap_min, c.x, V3Length(mover.velocity));
    MOVER_CHECK(gap_min >= -world.slop - MOVER_TEST_EPS);
    MOVER_CHECK(MoverTestDistance(c.x, 0.0) < 1e-3);
    MOVER_CHECK(V3Length(mover.velocity) < 1e-3f);
    MoverTestWorldDealloc(&world);

    return output;
}

/*
 * 80 small spheres touching the mover (half slop deep), more than the 64 planes kept (a warning): standing and
 * then pushing into the ring keeps the mover finite and in place.
 */
static struct test_Output MoverPlaneOverflowTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
    const f32 sphere_radius = 0.1f;
    const f32 ring = MOVER_TEST_RADIUS + sphere_radius - 0.5f*(f32) world.slop;
    for (u32 i = 0; i < 10; ++i)
    {
        const f32 angle = 2.0f*F32_PI*(f32) i / 10.0f;
        for (u32 j = 0; j < 8; ++j)
        {
            const f32 y = -MOVER_TEST_HALF_HEIGHT + 2.0f*MOVER_TEST_HALF_HEIGHT*(f32) j / 7.0f;
            MoverTestSphere(&world, sphere_radius, V3(ring*F32Cos(angle), y, ring*F32Sin(angle)));
        }
    }

    struct ds_GeometricMover mover = MoverTestMover(&world, V3Zero(), V3Zero());
    MoverTestStep(&world, &mover, 0);
    const f64 still = MoverTestLength(MoverTestCenter(&world, &mover));
    u32 finite = MoverTestFinite(&mover);
    for (u32 t = 0; t < 10; ++t)
    {
        mover.velocity = V3(1.0f, 0.0f, 0.0f);
        MoverTestStep(&world, &mover, 0);
        finite &= MoverTestFinite(&mover);
    }
    const f64 pushed = MoverTestLength(MoverTestCenter(&world, &mover));

    fprintf(stdout, "  %-24s standing moved %.3e  pushing moved %.3e  finite %u\n", "80 spheres", still, pushed, finite);
    MOVER_CHECK(finite);
    MOVER_CHECK(still < MOVER_TEST_EPS);
    MOVER_CHECK(pushed < 0.01);
    MoverTestWorldDealloc(&world);

    return output;
}

/* the body's velocities: zero outside the active set */
static void MoverTestBodyVelocity(v3 *linear, v3 *angular, const struct mover_TestWorld *world, const ds_BodyId id)
{
    const struct ds_Body *body = ds_BodyLookup(&world->pipeline, id).address;
    *linear = V3Zero();
    *angular = V3Zero();
    if (body->set == SOLVER_SET_ACTIVE)
    {
        const struct ds_BodyCompute *compute = world->pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf + body->sim;
        *linear = compute->linear_velocity;
        *angular = compute->angular_velocity;
    }
}

/*
 * Walk 3 m/s into a resting dynamic sphere 2 cm ahead on the mover's axis, one tick. A pushing mover stops at the
 * sphere as at a static one, and the sphere leaves at the mover's 3 m/s along the normal without spin; a sleeping
 * sphere is woken and pushed alike. A sphere already leaving at 5 m/s is not pulled back; a mover that does not
 * push (a camera) leaves the sphere at rest.
 */
static struct test_Output MoverPushTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    const char *name[4] = { "push", "push sleeping", "sphere leaving", "camera" };
    for (u32 i = 0; i < 4; ++i)
    {
        MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
        const struct c_Shape src = { .type = C_SHAPE_SPHERE, .sphere = { .radius = 0.5f } };
        const ds_BodyId id = MoverTestAdd(&world, &src, V3(2.0f, 0.0f, 0.0f), QIdentity(), 1);
        const struct ds_Body *body = ds_BodyLookup(&world.pipeline, id).address;
        if (i == 1)
        {
            ds_SolverSetSleep(&world.pipeline, body->island);
            MOVER_CHECK(body->set != SOLVER_SET_ACTIVE);
        }
        else if (i == 2)
        {
            world.pipeline.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.buf[body->sim].linear_velocity = V3(5.0f, 0.0f, 0.0f);
        }

        struct ds_GeometricMover mover = MoverTestMover(&world, V3(0.98f, 0.0f, 0.0f), V3(3.0f, 0.0f, 0.0f));
        mover.apply_impulse = (i != 3);
        MoverTestStep(&world, &mover, 0);

        v3 linear, angular;
        MoverTestBodyVelocity(&linear, &angular, &world, id);
        const v3f64 c = MoverTestCenter(&world, &mover);
        const f64 gap = 2.0 - c.x - 1.0;
        fprintf(stdout, "  %-24s sphere v (%+.6f, %+.3e, %+.3e)  |w| %.3e  set %u  mover gap %+.3e\n", name[i], linear.x, linear.y, linear.z, V3Length(angular), body->set, gap);
        MOVER_CHECK(gap >= -world.slop - MOVER_TEST_EPS && gap <= -0.75*world.slop + MOVER_TEST_EPS);
        MOVER_CHECK(MoverTestDistance(linear.y, 0.0) < 1e-4 && MoverTestDistance(linear.z, 0.0) < 1e-4);
        MOVER_CHECK(V3Length(angular) < 1e-4f);
        if (i < 2)
        {
            MOVER_CHECK(body->set == SOLVER_SET_ACTIVE);
            MOVER_CHECK(MoverTestDistance(linear.x, 3.0) < 1e-4);
        }
        else if (i == 2)
        {
            MOVER_CHECK(linear.x == 5.0f);
        }
        else
        {
            MOVER_CHECK(linear.x == 0.0f);
        }
        MoverTestWorldDealloc(&world);
    }

    return output;
}

/* a world with every shape kind and a scripted walk around it (input on a circle, gravity) */
static void MoverTestScript(struct mover_TestWorld *world, struct ds_GeometricMover *mover)
{
    MoverTestGrid(world, 20, 1.0f, V3(-10.0f, 0.0f, -10.0f), QIdentity());
    MoverTestBox(world, V3(0.25f, 2.0f, 10.0f), V3(2.25f, 2.0f, 0.0f), QIdentity(), 0);
    MoverTestBox(world, V3(0.5f, 0.5f, 0.5f), V3(-1.0f, 0.5f, 1.5f), QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), 0.4f), 1);
    MoverTestSphere(world, 0.5f, V3(0.5f, 0.5f, 2.0f));
    MoverTestCapsule(world, 0.3f, 1.0f, V3(-2.0f, 1.3f, -1.0f), QIdentity());

    *mover = MoverTestMover(world, V3(0.0f, (f32) MOVER_TEST_STAND, 0.0f), V3Zero());
    for (u32 t = 0; t < 240; ++t)
    {
        const f32 phase = 0.05f*(f32) t;
        mover->velocity.x = 3.0f*F32Cos(phase);
        mover->velocity.z = 3.0f*F32Sin(phase);
        MoverTestStep(world, mover, 1);
    }
}

/* the scripted walk twice in fresh worlds: bit-identical results */
static struct test_Output MoverDeterminismTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;
    struct ds_GeometricMover mover[2];

    for (u32 i = 0; i < 2; ++i)
    {
        MoverTestWorldAlloc(&world, V3F64(0.0, 0.0, 0.0));
        MoverTestScript(&world, mover + i);
        MoverTestWorldDealloc(&world);
    }

    fprintf(stdout, "  %-24s end (%.6f, %.6f, %.6f)\n", "script", mover[0].position.x, mover[0].position.y, mover[0].position.z);
    MOVER_CHECK(MoverTestFinite(mover + 0));
    MOVER_CHECK(mover[0].position.x == mover[1].position.x && mover[0].position.y == mover[1].position.y && mover[0].position.z == mover[1].position.z);
    MOVER_CHECK(mover[0].velocity.x == mover[1].velocity.x && mover[0].velocity.y == mover[1].velocity.y && mover[0].velocity.z == mover[1].velocity.z);

    return output;
}

static const v3f64 mover_test_origin[] =
{
    { .x = 0.0,         .y = 0.0,       .z = 0.0 },
    { .x = 1e4,         .y = 1e4,       .z = 1e4 },
    { .x = 1e5,         .y = 1e5,       .z = 1e5 },
    { .x = 1e6,         .y = -1e6,      .z = 1e6 },
    { .x = -10000.5,    .y = 5000.25,   .z = -12345.678 },
};

/* the scripted walk around several world origins: the same path relative to the origin */
static struct test_Output MoverOriginTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct test_Output *out = &output;
    struct mover_TestWorld world;

    v3f64 reference = V3F64(0.0, 0.0, 0.0);
    const u32 origin_count = sizeof(mover_test_origin) / sizeof(mover_test_origin[0]);
    for (u32 o = 0; o < origin_count; ++o)
    {
        struct ds_GeometricMover mover;
        MoverTestWorldAlloc(&world, mover_test_origin[o]);
        MoverTestScript(&world, &mover);
        const v3f64 c = MoverTestCenter(&world, &mover);
        MoverTestWorldDealloc(&world);

        if (o == 0)
        {
            reference = c;
        }

        const f64 deviation = MoverTestLength(V3F64Sub(c, reference));
        fprintf(stdout, "  origin (%.6g, %.6g, %.6g): end (%.6f, %.6f, %.6f)  vs origin 0 %.3e\n", mover_test_origin[o].x, mover_test_origin[o].y, mover_test_origin[o].z, c.x, c.y, c.z, deviation);
        MOVER_CHECK(MoverTestFinite(&mover));
        MOVER_CHECK(deviation < 5e-4);
    }

    return output;
}

static struct test_Output (*geometric_mover_tests[])(struct test_Environment *) =
{
    MoverEmptyWorldTest,
    MoverFloorTest,
    MoverFastHitTest,
    MoverHitOrderTest,
    MoverWallSlideTest,
    //MoverMeshInternalEdgeTest,
    MoverCornerTest,
    MoverCeilingTest,
    MoverWallEndTest,
    MoverClipPushedOnlyTest,
    MoverOverlapTest,
    MoverSqueezeTest,
    MoverRampTest,
    MoverSeamWalkTest,
    MoverValleyTest,
    MoverPlaneOverflowTest,
    MoverPushTest,
    MoverDeterminismTest,
    MoverOriginTest,
};

struct suite_Correctness m_geometric_mover_suite =
{
	.id = "geometric_mover",
	.unit_test = geometric_mover_tests,
	.unit_test_count = sizeof(geometric_mover_tests) / sizeof(geometric_mover_tests[0]),
};

struct suite_Correctness *geometric_mover_correctness_suite = &m_geometric_mover_suite;

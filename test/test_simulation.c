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
#include "ds_simulation.h"

/* 1 if every field of a and b is the same (floats by their bits) */
static u32 SimConfigTestEqual(const struct ds_SimConfig *a, const struct ds_SimConfig *b)
{
    u32 equal = 1;
    for (u32 i = 0; i < 4; ++i)
    {
        equal = equal && a->seed[i] == b->seed[i];
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        equal = equal && a->us_system_tick[i] == b->us_system_tick[i];
    }

    const b64 speed_a = { .f = a->speed };
    const b64 speed_b = { .f = b->speed };
    equal = equal && speed_a.u == speed_b.u;

    return equal;
}

/*
 * Round trip of the default config and of a config with every field changed (given and generated seeds);
 * the stream advances by exactly ds_SimConfigSerializeSize. Reads that must fail with the config zeroed and the
 * stream unchanged: a truncated stream, a wrong version word, a stream off its byte boundary, and every
 * out-of-range value (zero seed, nonzero external tick, zero and too-slow system ticks, negative, NaN and
 * too-fast speed).
 */
static struct test_Output SimConfigSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_SimConfig changed = ds_SimConfigDefault();
    const u64 seed[4] = { 0x1234, 0, 0xabcdef, U64_MAX };
    ds_SimConfigSeed(&changed, seed);
    TEST_TRUE(changed.seed[0] == 0x1234 && changed.seed[3] == U64_MAX);

    /* the all-zero seed is rejected: the default seed instead */
    const u64 zero[4] = { 0, 0, 0, 0 };
    struct ds_SimConfig rejected = changed;
    ds_SimConfigSeed(&rejected, zero);
    const struct ds_SimConfig def = ds_SimConfigDefault();
    TEST_TRUE(rejected.seed[0] == def.seed[0] && rejected.seed[1] == def.seed[1] && rejected.seed[2] == def.seed[2] && rejected.seed[3] == def.seed[3]);
    for (u32 i = SIM_SYSTEM_EXTERNAL + 1; i < SIM_SYSTEM_COUNT; ++i)
    {
        changed.us_system_tick[i] = 1000 * (i + 1);
    }
    changed.speed = 0.25;

    struct ds_SimConfig generated = ds_SimConfigDefault();
    ds_SimConfigGenerateSeed(&generated);
    TEST_TRUE((generated.seed[0] | generated.seed[1] | generated.seed[2] | generated.seed[3]) != 0);

    const struct ds_SimConfig valid[3] = { ds_SimConfigDefault(), changed, generated };
    for (u32 c = 0; c < 3; ++c)
    {
        const u64 size = ds_SimConfigSerializeSize(&valid[c]);
        struct ss ss = ss_Alloc(env->mem_1, size + 16);
        TEST_NOT_EQUAL(NULL, ss.buf);

        ds_SimConfigSerialize(&ss, &valid[c]);
        TEST_EQUAL(8*size, ss.bit_index);

        struct ds_SimConfig read = { 0 };
        ss.bit_index = 0;
        TEST_EQUAL(1, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(1, SimConfigTestEqual(&valid[c], &read));

        /* truncated: one byte short */
        struct ss truncated = ss_Buffered(ss.buf, size - 1);
        read = changed;
        TEST_EQUAL(0, ds_SimConfigTryDeserialize(&truncated, &read));
        TEST_EQUAL(0, truncated.bit_index);
        TEST_EQUAL(0, read.seed[0] | read.us_system_tick[SIM_SYSTEM_DYNAMICS]);

        /* off the byte boundary */
        ss.bit_index = 3;
        TEST_EQUAL(0, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(3, ss.bit_index);

        /* wrong version: the first word */
        ss.bit_index = 0;
        ss_WriteU32Le(&ss, 0xffffffffu);
        ss.bit_index = 0;
        TEST_EQUAL(0, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(0, ss.bit_index);
    }

    /* out-of-range values survive the write and fail the read */
    struct ds_SimConfig invalid[8];
    for (u32 i = 0; i < 8; ++i)
    {
        invalid[i] = changed;
    }
    invalid[0].seed[0] = invalid[0].seed[1] = invalid[0].seed[2] = invalid[0].seed[3] = 0;
    invalid[1].us_system_tick[SIM_SYSTEM_EXTERNAL] = 1;
    invalid[2].us_system_tick[SIM_SYSTEM_DYNAMICS] = 0;
    invalid[3].us_system_tick[SIM_SYSTEM_GAME] = DS_SIM_TICK_US_MAX + 1;
    invalid[4].speed = -1.0;
    invalid[5].speed = (b64) { .u = 0x7ff8000000000000ull }.f;      /* NaN */
    invalid[6].speed = DS_SIM_SPEED_MAX * 2.0;
    invalid[7].speed = (b64) { .u = 0x7ff0000000000000ull }.f;      /* +inf */

    for (u32 i = 0; i < 8; ++i)
    {
        const u64 size = ds_SimConfigSerializeSize(&invalid[i]);
        struct ss ss = ss_Alloc(env->mem_1, size);
        TEST_NOT_EQUAL(NULL, ss.buf);
        ds_SimConfigSerialize(&ss, &invalid[i]);
        ss.bit_index = 0;

        struct ds_SimConfig read = changed;
        TEST_EQUAL(0, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(0, ss.bit_index);
        TEST_EQUAL(0, read.seed[0] | read.us_system_tick[SIM_SYSTEM_DYNAMICS]);
    }

    /* the valid boundaries pass */
    struct ds_SimConfig edge = changed;
    edge.us_system_tick[SIM_SYSTEM_DYNAMICS] = 1;
    edge.us_system_tick[SIM_SYSTEM_GAME] = DS_SIM_TICK_US_MAX;
    edge.speed = 0.0;
    {
        const u64 size = ds_SimConfigSerializeSize(&edge);
        struct ss ss = ss_Alloc(env->mem_1, size);
        ds_SimConfigSerialize(&ss, &edge);
        ss.bit_index = 0;
        struct ds_SimConfig read = { 0 };
        TEST_EQUAL(1, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(1, SimConfigTestEqual(&edge, &read));
    }

    edge.speed = DS_SIM_SPEED_MAX;
    {
        const u64 size = ds_SimConfigSerializeSize(&edge);
        struct ss ss = ss_Alloc(env->mem_1, size);
        ds_SimConfigSerialize(&ss, &edge);
        ss.bit_index = 0;
        struct ds_SimConfig read = { 0 };
        TEST_EQUAL(1, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(1, SimConfigTestEqual(&edge, &read));
    }

	return output;
}

/* 1 if a and b serialize to the same bytes: every field compared through the stream, floats by their bits */
static u32 DynamicsConfigTestEqual(struct arena *mem, const struct ds_DynamicsConfig *a, const struct ds_DynamicsConfig *b)
{
    const u64 size = ds_DynamicsConfigSerializeSize(a);
    struct ss sa = ss_Alloc(mem, size);
    struct ss sb = ss_Alloc(mem, size);
    ds_DynamicsConfigSerialize(&sa, a);
    ds_DynamicsConfigSerialize(&sb, b);

    u32 equal = (sa.bit_index == sb.bit_index);
    for (u64 i = 0; equal && i < size; ++i)
    {
        equal = (sa.buf[i] == sb.buf[i]);
    }

    return equal;
}

/*
 * The default config passes the check and round trips (the stream advances by exactly the serialize size, the
 * read-back is field-for-field the same), as does a config with the pipeline, solver, numerics and draw values
 * changed. Reads that fail with the config zeroed and the stream unchanged: truncated, off the byte boundary, a
 * wrong version, and out-of-range values from each group (gravity NaN, a capacity that is no power of two, zero
 * iterations, baumgarte above 1, zero slop, a numerics angle above 45, a draw flag of 2, a mode past its count,
 * a negative point size, a colour with an infinite component).
 */
static struct test_Output DynamicsConfigSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct ds_DynamicsConfig def = ds_DynamicsConfigDefault();
    TEST_EQUAL(1, ds_DynamicsConfigCheck(&def));

    struct ds_DynamicsConfig changed = def;
    changed.gravity = V3(0.5f, -1.0f, 2.0f);
    changed.initial_size = 64;
    changed.frame_memory = 1024;
    changed.solver.pgs_iteration_count = 2;
    changed.solver.ngs_iteration_count = 1;
    changed.solver.warmup_solver = 0;
    changed.solver.baumgarte_constant = 1.0f;
    changed.solver.max_linear_velocity_magnitude = 0.0f;
    changed.solver.linear_slop = 0.001f;
    changed.solver.sleep_enabled = 0;
    changed.numerics.gjk_max_iterations = 7;
    changed.numerics.edge_parallel_max_degrees = 45.0f;
    changed.numerics.sat_face_bias = 0.0f;
    changed.draw.draw_shapes = 0;
    changed.draw.shape_color_mode = DRAW_COLOR_TOUCHING;
    changed.draw.point_color_mode = DRAW_POINT_GRAPH_COLOR;
    changed.draw.refinement = 3;
    changed.draw.color_point[DRAW_POINT_COLOR_COUNT - 1] = V4(0.1f, 0.2f, 0.3f, 0.4f);
    changed.draw.color_mass_center = V4(0.0f, 0.0f, 0.0f, 0.0f);
    TEST_EQUAL(1, ds_DynamicsConfigCheck(&changed));
    TEST_EQUAL(0, DynamicsConfigTestEqual(env->mem_1, &def, &changed));

    const struct ds_DynamicsConfig valid[2] = { def, changed };
    for (u32 c = 0; c < 2; ++c)
    {
        const u64 size = ds_DynamicsConfigSerializeSize(&valid[c]);
        struct ss ss = ss_Alloc(env->mem_1, size + 16);
        TEST_NOT_EQUAL(NULL, ss.buf);

        ds_DynamicsConfigSerialize(&ss, &valid[c]);
        TEST_EQUAL(8*size, ss.bit_index);

        struct ds_DynamicsConfig read = { 0 };
        ss.bit_index = 0;
        TEST_EQUAL(1, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(1, DynamicsConfigTestEqual(env->mem_1, &valid[c], &read));

        struct ss truncated = ss_Buffered(ss.buf, size - 1);
        read = changed;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&truncated, &read));
        TEST_EQUAL(0, truncated.bit_index);
        TEST_EQUAL(0, read.initial_size | read.solver.pgs_iteration_count);

        ss.bit_index = 3;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(3, ss.bit_index);

        ss.bit_index = 0;
        ss_WriteU32Le(&ss, 0xffffffffu);
        ss.bit_index = 0;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(0, ss.bit_index);
    }

    const f32 nan = (b32) { .u = 0x7fc00000u }.f;
    const f32 inf = (b32) { .u = 0x7f800000u }.f;
    struct ds_DynamicsConfig invalid[10];
    for (u32 i = 0; i < 10; ++i)
    {
        invalid[i] = def;
    }
    invalid[0].gravity.y = nan;
    invalid[1].initial_size = 1000;
    invalid[2].solver.pgs_iteration_count = 0;
    invalid[3].solver.baumgarte_constant = 1.5f;
    invalid[4].solver.linear_slop = 0.0f;
    invalid[5].numerics.capsule_parallel_max_degrees = 46.0f;
    invalid[6].draw.draw_joints = 2;
    invalid[7].draw.shape_color_mode = DRAW_COLOR_MODE_COUNT;
    invalid[8].draw.point_size = -1.0f;
    invalid[9].draw.color_awake.z = inf;

    for (u32 i = 0; i < 10; ++i)
    {
        TEST_EQUAL(0, ds_DynamicsConfigCheck(&invalid[i]));

        const u64 size = ds_DynamicsConfigSerializeSize(&invalid[i]);
        struct ss ss = ss_Alloc(env->mem_1, size);
        TEST_NOT_EQUAL(NULL, ss.buf);
        ds_DynamicsConfigSerialize(&ss, &invalid[i]);
        ss.bit_index = 0;

        struct ds_DynamicsConfig read = changed;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(0, ss.bit_index);
        TEST_EQUAL(0, read.initial_size | read.solver.pgs_iteration_count);
    }

	return output;
}

/*
 * ds_SimAlloc from the default config then ds_SimTick with fake wall-clock deltas: nothing ticks at time 0, the seed
 * slot empties on the first advancing frame, a frame of one dynamics period runs one physics tick and a frame of two
 * runs two, sub-microsecond deltas accumulate in the remainder until they make a microsecond, the budget caps a long
 * frame, speed 0 holds time still, and ds_SimFlush restarts at 0.
 */
static struct test_Output SimTickTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct c_ShapeDB cshape_db = { 0 };
    c_ShapeDBAlloc(NULL, &cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);

    struct ds_SimConfig config = ds_SimConfigDefault();
    config.dynamics.initial_size = 32;
    const u64 period = config.us_system_tick[SIM_SYSTEM_DYNAMICS];

    struct arena mem = ArenaAlloc(NULL, ds_DynamicsShellMemoryRequirement(g_config->thread_count, config.dynamics.frame_memory));
    struct ds_Sim sim;
    ds_SimAlloc(&mem, &sim, &config, 0, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    TEST_TRUE(mem.mem_left < mem.mem_size);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(SIM_MODE_NONE, sim.mode);
    TEST_EQUAL(1000 * period, sim.dynamics.ns_tick);
    TEST_EQUAL(0, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* no time, nothing happens */
    ds_SimTick(&sim, 0);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* sub-microsecond deltas: the remainder carries them */
    ds_SimTick(&sim, 600);
    TEST_EQUAL(0, sim.us_delta);
    TEST_EQUAL(600, sim.ns_remainder);
    TEST_EQUAL(0, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);
    ds_SimTick(&sim, 600);
    TEST_EQUAL(1, sim.us_delta);
    TEST_EQUAL(200, sim.ns_remainder);
    TEST_EQUAL(1, sim.us_time);
    /* the first advancing frame ran the seed slot, which is now empty, and no physics tick before its period */
    TEST_EQUAL(U64_MAX, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* one period: the first physics tick */
    ds_SimTick(&sim, 1000 * (period - 1));
    TEST_EQUAL(period, sim.us_time);
    TEST_EQUAL(1, sim.dynamics.frames_completed);

    /* the budget caps a long frame at the sum of the periods: two ticks, not twenty */
    ds_SimTick(&sim, 20 * 1000 * period);
    TEST_EQUAL(sim.ns_budget / 1000, sim.us_delta);
    TEST_EQUAL(period + sim.ns_budget / 1000, sim.us_time);
    TEST_EQUAL(1 + sim.ns_budget / (1000 * period), sim.dynamics.frames_completed);

    /* speed 0: time stands still, the remainder too */
    sim.speed = 0.0;
    ds_SimTick(&sim, 1000 * period);
    TEST_EQUAL(0, sim.us_delta);
    TEST_EQUAL(period + sim.ns_budget / 1000, sim.us_time);
    sim.speed = 2.0;
    ds_SimTick(&sim, 500 * period);
    TEST_EQUAL(period, sim.us_delta);

    /* flush: back to 0 with the seed slot armed */
    ds_SimFlush(&sim);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(0, sim.ns_remainder);
    TEST_EQUAL(0, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    ds_SimDealloc(&sim);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

static struct test_Output (*simulation_tests[])(struct test_Environment *) =
{
    SimConfigSerializeTest,
    DynamicsConfigSerializeTest,
    SimTickTest,
};

struct suite_Correctness m_simulation_suite =
{
	.id = "simulation",
	.unit_test = simulation_tests,
	.unit_test_count = sizeof(simulation_tests) / sizeof(simulation_tests[0]),
};

struct suite_Correctness *simulation_correctness_suite = &m_simulation_suite;

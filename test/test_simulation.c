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
        struct SS ss = SSAlloc(env->mem_1, size + 16);
        TEST_NOT_EQUAL(NULL, ss.buf);

        ds_SimConfigSerialize(&ss, &valid[c]);
        TEST_EQUAL(8*size, ss.bit_index);

        struct ds_SimConfig read = { 0 };
        ss.bit_index = 0;
        TEST_EQUAL(1, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(1, SimConfigTestEqual(&valid[c], &read));

        /* truncated: one byte short */
        struct SS truncated = SSBuffered(ss.buf, size - 1);
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
        SSWriteU32Le(&ss, 0xffffffffu);
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
        struct SS ss = SSAlloc(env->mem_1, size);
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
        struct SS ss = SSAlloc(env->mem_1, size);
        ds_SimConfigSerialize(&ss, &edge);
        ss.bit_index = 0;
        struct ds_SimConfig read = { 0 };
        TEST_EQUAL(1, ds_SimConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(1, SimConfigTestEqual(&edge, &read));
    }

    edge.speed = DS_SIM_SPEED_MAX;
    {
        const u64 size = ds_SimConfigSerializeSize(&edge);
        struct SS ss = SSAlloc(env->mem_1, size);
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
    struct SS sa = SSAlloc(mem, size);
    struct SS sb = SSAlloc(mem, size);
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
 * The default config round trips (the stream advances by exactly the serialize size, the read-back is
 * field-for-field the same), as does a config with the pipeline, solver and numerics values changed. Reads that
 * fail with the config zeroed and the stream unchanged: truncated, off the byte boundary, a wrong version in the
 * config and in each contained config. Out-of-range values are clamped by Derive, on read as well (a capacity
 * rounds up to a power of two, zero iterations to one, baumgarte above 1 to 1, a NaN gravity to 0, an angle above
 * 45 to 45). The draw config round trips on its own, clamped the same way (a flag of 2 to 1, a mode past its
 * count to the last, a negative point size to 0, an infinite colour component to 1).
 */
static struct test_Output DynamicsConfigSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct ds_DynamicsConfig def = ds_DynamicsConfigDefault();

    struct ds_DynamicsConfig changed = def;
    changed.solver.gravity = V3(0.5f, -1.0f, 2.0f);
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
    changed = ds_DynamicsConfigDerive(&changed);
    TEST_EQUAL(0, DynamicsConfigTestEqual(env->mem_1, &def, &changed));

    const struct ds_DynamicsConfig valid[2] = { def, changed };
    for (u32 c = 0; c < 2; ++c)
    {
        const u64 size = ds_DynamicsConfigSerializeSize(&valid[c]);
        struct SS ss = SSAlloc(env->mem_1, size + 16);
        TEST_NOT_EQUAL(NULL, ss.buf);

        ds_DynamicsConfigSerialize(&ss, &valid[c]);
        TEST_EQUAL(8*size, ss.bit_index);

        struct ds_DynamicsConfig read = { 0 };
        ss.bit_index = 0;
        TEST_EQUAL(1, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(1, DynamicsConfigTestEqual(env->mem_1, &valid[c], &read));

        struct SS truncated = SSBuffered(ss.buf, size - 1);
        read = changed;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&truncated, &read));
        TEST_EQUAL(0, truncated.bit_index);
        TEST_EQUAL(0, read.initial_size | read.solver.pgs_iteration_count);

        ss.bit_index = 3;
        TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&ss, &read));
        TEST_EQUAL(3, ss.bit_index);

        /* a wrong version word: the config's, then each contained config's (at their offsets in the stream) */
        const u64 version_offset[3] =
        {
            0,
            4 + 4 + 8,
            4 + 4 + 8 + ds_SolverConfigSerializeSize(&valid[c].solver),
        };
        for (u32 v = 0; v < 3; ++v)
        {
            ss.bit_index = 8*version_offset[v];
            const u32 version = SSReadU32Le(&ss);
            ss.bit_index = 8*version_offset[v];
            SSWriteU32Le(&ss, 0xffffffffu);
            ss.bit_index = 0;
            TEST_EQUAL(0, ds_DynamicsConfigTryDeserialize(&ss, &read));
            TEST_EQUAL(0, ss.bit_index);
            ss.bit_index = 8*version_offset[v];
            SSWriteU32Le(&ss, version);
        }
    }

    const f32 nan = (b32) { .u = 0x7fc00000u }.f;
    const f32 inf = (b32) { .u = 0x7f800000u }.f;
    struct ds_DynamicsConfig out = def;
    out.solver.gravity.y = nan;
    out.initial_size = 1000;
    out.solver.pgs_iteration_count = 0;
    out.solver.baumgarte_constant = 1.5f;
    out.solver.linear_slop = 0.0f;
    out.numerics.capsule_parallel_max_degrees = 46.0f;

    const struct ds_DynamicsConfig derived = ds_DynamicsConfigDerive(&out);
    const u64 size = ds_DynamicsConfigSerializeSize(&out);
    struct SS ss = SSAlloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, ss.buf);
    ds_DynamicsConfigSerialize(&ss, &out);
    ss.bit_index = 0;
    struct ds_DynamicsConfig read = changed;
    TEST_EQUAL(1, ds_DynamicsConfigTryDeserialize(&ss, &read));
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_EQUAL(1, DynamicsConfigTestEqual(env->mem_1, &derived, &read));

    TEST_TRUE(read.solver.gravity.y == 0.0f);
    TEST_EQUAL(1024u, read.initial_size);
    TEST_EQUAL(1u, read.solver.pgs_iteration_count);
    TEST_TRUE(read.solver.baumgarte_constant == 1.0f);
    TEST_TRUE(read.solver.linear_slop > 0.0f);
    TEST_TRUE(read.numerics.capsule_parallel_max_degrees == 45.0f);

    /* the draw config on its own */
    struct ds_DynamicsDrawConfig draw = ds_DynamicsDrawConfigDefault();
    draw.shape_color_mode = DRAW_COLOR_TOUCHING;
    draw.point_color_mode = DRAW_POINT_GRAPH_COLOR;
    draw.refinement = 3;
    draw.color_point[DRAW_POINT_COLOR_COUNT - 1] = V4(0.1f, 0.2f, 0.3f, 0.4f);
    draw.color_mass_center = V4(0.0f, 0.0f, 0.0f, 0.0f);
    draw.draw_joints = 2;
    draw.point_size = -1.0f;
    draw.color_awake.z = inf;
    const u64 draw_size = ds_DynamicsDrawConfigSerializeSize(&draw);
    struct SS dss = SSAlloc(env->mem_1, draw_size);
    TEST_NOT_EQUAL(NULL, dss.buf);
    ds_DynamicsDrawConfigSerialize(&dss, &draw);
    TEST_EQUAL(8*draw_size, dss.bit_index);
    dss.bit_index = 0;
    struct ds_DynamicsDrawConfig draw_read = { 0 };
    TEST_EQUAL(1, ds_DynamicsDrawConfigTryDeserialize(&dss, &draw_read));
    TEST_EQUAL(8*draw_size, dss.bit_index);
    TEST_EQUAL((u32) DRAW_COLOR_TOUCHING, (u32) draw_read.shape_color_mode);
    TEST_EQUAL((u32) DRAW_POINT_GRAPH_COLOR, (u32) draw_read.point_color_mode);
    TEST_EQUAL(3u, draw_read.refinement);
    TEST_TRUE(draw_read.color_point[DRAW_POINT_COLOR_COUNT - 1].w == 0.4f && draw_read.color_mass_center.w == 0.0f);
    TEST_EQUAL(1u, draw_read.draw_joints);
    TEST_TRUE(draw_read.point_size == 0.0f);
    TEST_TRUE(draw_read.color_awake.z == 1.0f);
    dss.bit_index = 0;
    SSWriteU32Le(&dss, 0xffffffffu);
    dss.bit_index = 0;
    TEST_EQUAL(0, ds_DynamicsDrawConfigTryDeserialize(&dss, &draw_read));
    TEST_EQUAL(0, dss.bit_index);
    TEST_EQUAL(0u, draw_read.refinement);

	return output;
}

/*
 * ds_SimAlloc from the default config (which runs the seed batch) then ds_SimTick with fake wall-clock deltas: nothing
 * ticks at time 0, a frame of one dynamics period runs one physics tick and a frame of two
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
    /* the seed batch ran inside ds_SimAlloc: the external slot is empty, time still 0 */
    TEST_EQUAL(U64_MAX, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* no time: no tick */
    ds_SimTick(&sim, 0);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(0, sim.dynamics.frames_completed);
    TEST_EQUAL(U64_MAX, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);

    /* sub-microsecond deltas: the remainder carries them */
    ds_SimTick(&sim, 600);
    TEST_EQUAL(0, sim.us_delta);
    TEST_EQUAL(600, sim.ns_remainder);
    ds_SimTick(&sim, 600);
    TEST_EQUAL(1, sim.us_delta);
    TEST_EQUAL(200, sim.ns_remainder);
    TEST_EQUAL(1, sim.us_time);
    /* no physics tick before its period */
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
    TEST_EQUAL(U64_MAX, sim.sys_schedule[SIM_SYSTEM_EXTERNAL]);    /* reseeded: the seed batch ran */
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    ds_SimDealloc(&sim);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

/*
 * Payloads round trip on hand-written 13-bit pseudo commands (the command serializers are not in yet): the serialized
 * form is dst, cmd_count, the command bits and the bytes holding them. TryDeserializeAndAlias views the stream's bytes and
 * fails on a short stream and nonzero bits past the last command, leaving the stream as before and the payload
 * empty; Append copies a view bit-exact (mid-byte included) into a payload with room, on the heap or an arena, and
 * refuses another dst or a payload without room.
 */
static struct test_Output SimCmdPayloadSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const u64 cmd_bits = 13;
    const u32 cmd_count = 3;
    const u64 bits = cmd_count * cmd_bits;     /* 39: ends mid-byte */
    const u64 bytes = (bits + 7) / 8;

    struct ds_SimCmdPayload payload = ds_SimCmdPayloadAlloc(NULL, ds_SimIdInit(3, 2), 64);
    for (u32 i = 0; i < cmd_count; ++i)
    {
        SSWriteU64Partial(&payload.stream, i + 1, cmd_bits);
    }
    payload.cmd_count = cmd_count;

    const u64 size = ds_SimCmdPayloadSerializeSize(&payload);
    TEST_EQUAL(16 + bytes, size);
    struct SS ss = SSAlloc(NULL, size);
    ds_SimCmdPayloadSerialize(&ss, &payload);
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_EQUAL(0, ss.buf[size - 1] >> (bits & 7));     /* the unused bits of the last byte are zero */

    /* a view: whatever the payload held is replaced, the bits read back in place */
    ss.bit_index = 0;
    struct ds_SimCmdPayload view = ds_SimCmdPayloadEmpty(ds_SimIdInit(4, 2));
    TEST_EQUAL(1, ds_SimCmdPayloadTryDeserializeAndAlias(&ss, &view));
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_TRUE(ds_SimIdEqual(payload.dst, view.dst) && view.stream.buf == ss.buf + 16 && !view.stream.heap_allocated);
    TEST_EQUAL(cmd_count, view.cmd_count);
    TEST_TRUE(view.stream.bit_index == bits && view.stream.bit_count == bits);
    struct SS back = SSBuffered(view.stream.buf, bytes);
    for (u32 i = 0; i < cmd_count; ++i)
    {
        TEST_EQUAL(i + 1, SSReadU64Partial(&back, cmd_bits));
    }

    /* short streams: the header cut, the bytes cut; nonzero bits past the last command */
    struct ds_SimCmdPayload bad = ds_SimCmdPayloadEmpty(payload.dst);
    struct SS truncated = SSBuffered(ss.buf, 15);
    TEST_EQUAL(0, ds_SimCmdPayloadTryDeserializeAndAlias(&truncated, &bad));
    TEST_EQUAL(0, truncated.bit_index);
    truncated = SSBuffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_SimCmdPayloadTryDeserializeAndAlias(&truncated, &bad));
    TEST_EQUAL(0, truncated.bit_index);
    ss.buf[size - 1] |= 0x80;
    ss.bit_index = 0;
    TEST_EQUAL(0, ds_SimCmdPayloadTryDeserializeAndAlias(&ss, &bad));
    TEST_TRUE(ss.bit_index == 0 && bad.stream.buf == NULL && bad.cmd_count == 0);
    ss.buf[size - 1] &= 0x7f;

    /* the view appended twice into a heap payload sized for both: six commands, the second three mid-byte */
    struct ds_SimCmdPayload fused = ds_SimCmdPayloadAlloc(NULL, payload.dst, 2*bytes);
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&fused, &view));
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&fused, &view));
    TEST_EQUAL(2*cmd_count, fused.cmd_count);
    TEST_EQUAL(2*bits, fused.stream.bit_index);
    back = SSBuffered(fused.stream.buf, (2*bits + 7) / 8);
    for (u32 i = 0; i < 2*cmd_count; ++i)
    {
        TEST_EQUAL(i % cmd_count + 1, SSReadU64Partial(&back, cmd_bits));
    }
    TEST_EQUAL(0, ds_SimCmdPayloadAppend(&fused, &view));      /* no room */
    struct ds_SimCmdPayload other = view;
    other.dst = ds_SimIdInit(4, 2);
    struct ds_SimCmdPayload roomy = ds_SimCmdPayloadAlloc(NULL, payload.dst, 2*bytes);
    TEST_EQUAL(0, ds_SimCmdPayloadAppend(&roomy, &other));     /* another dst */
    TEST_TRUE(roomy.stream.bit_index == 0 && roomy.cmd_count == 0);
    struct ds_SimCmdPayload adopt = ds_SimCmdPayloadAlloc(NULL, ds_SimIdNull(), bytes);
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&adopt, &other));
    TEST_TRUE(ds_SimIdEqual(other.dst, adopt.dst));
    ds_SimCmdPayloadDealloc(&adopt);
    ds_SimCmdPayloadDealloc(&roomy);
    ds_SimCmdPayloadDealloc(&fused);
    TEST_TRUE(fused.stream.buf == NULL && fused.cmd_count == 0);

    /* on an arena sized for both; on one with room for one payload the second append and a fresh payload are refused */
    struct arena arena = ArenaAlloc(NULL, 4*size);
    fused = ds_SimCmdPayloadAlloc(&arena, payload.dst, 2*bytes);
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&fused, &view));
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&fused, &view));
    TEST_EQUAL(2*bits, fused.stream.bit_index);
    back = SSBuffered(fused.stream.buf, (2*bits + 7) / 8);
    for (u32 i = 0; i < 2*cmd_count; ++i)
    {
        TEST_EQUAL(i % cmd_count + 1, SSReadU64Partial(&back, cmd_bits));
    }
    ArenaFree(&arena);

    struct arena tight = ArenaAlloc(NULL, bytes);
    struct ds_SimCmdPayload one = ds_SimCmdPayloadAlloc(&tight, payload.dst, bytes);
    TEST_TRUE(tight.mem_left == 0 && one.stream.buf != NULL);
    TEST_EQUAL(1, ds_SimCmdPayloadAppend(&one, &view));
    TEST_EQUAL(0, ds_SimCmdPayloadAppend(&one, &view));
    TEST_TRUE(one.stream.bit_index == bits && one.cmd_count == cmd_count);
    struct ds_SimCmdPayload none = ds_SimCmdPayloadAlloc(&tight, payload.dst, bytes);
    TEST_TRUE(none.stream.buf == NULL);
    TEST_EQUAL(0, ds_SimCmdPayloadAppend(&none, &view));
    ArenaFree(&tight);

    SSDealloc(&ss);
    ds_SimCmdPayloadDealloc(&payload);
	return output;
}

static struct test_Output (*simulation_tests[])(struct test_Environment *) =
{
    SimConfigSerializeTest,
    DynamicsConfigSerializeTest,
    SimCmdPayloadSerializeTest,
    SimTickTest,
};

struct suite_Correctness m_simulation_suite =
{
	.id = "simulation",
	.unit_test = simulation_tests,
	.unit_test_count = sizeof(simulation_tests) / sizeof(simulation_tests[0]),
};

struct suite_Correctness *simulation_correctness_suite = &m_simulation_suite;

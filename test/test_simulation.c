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
#include "ds_server.h"

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

    return equal;
}

/*
 * Round trip of the default config and of a config with every field changed (given and generated seeds);
 * the stream advances by exactly ds_SimConfigSerializeSize. Reads that must fail with the config zeroed and the
 * stream unchanged: a truncated stream, a wrong version word, a stream off its byte boundary, and every
 * out-of-range value (zero seed, nonzero external tick, zero and too-slow system ticks).
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
    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        changed.us_system_tick[i] = 1000 * (i + 1);
    }

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
    struct ds_SimConfig invalid[3];
    for (u32 i = 0; i < 3; ++i)
    {
        invalid[i] = changed;
    }
    invalid[0].seed[0] = invalid[0].seed[1] = invalid[0].seed[2] = invalid[0].seed[3] = 0;
    invalid[1].us_system_tick[SIM_SYSTEM_DYNAMICS] = 0;
    invalid[2].us_system_tick[SIM_SYSTEM_GAME] = DS_SIM_TICK_US_MAX + 1;

    for (u32 i = 0; i < 3; ++i)
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
 * ds_SimAlloc from the default config (which runs the seed batch) then ds_SimTick with simulation-time deltas and no
 * commands: nothing ticks at time 0 or before the first period, a frame of one dynamics period runs one physics tick
 * and a frame of two runs two, and ds_SimFlush restarts at 0.
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
    ds_SimAlloc(&mem, &sim, &config, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    TEST_TRUE(mem.mem_left < mem.mem_size);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(1000 * period, sim.dynamics.ns_tick);
    /* the seed batch ran inside ds_SimAlloc: time still 0, the first dynamics tick a period out */
    TEST_EQUAL(1, sim.initialized);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    ds_CPool(ds_SimCmd) none;
    ds_CPoolAlloc(env->mem_1, none, 1, NOT_GROWABLE);

    /* no time: no tick */
    ds_SimTick(&sim, &none, 0);
    TEST_EQUAL(0, sim.us_time);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* no physics tick before its period */
    ds_SimTick(&sim, &none, 1);
    TEST_EQUAL(1, sim.us_delta);
    TEST_EQUAL(1, sim.us_time);
    TEST_EQUAL(0, sim.dynamics.frames_completed);

    /* one period: the first physics tick; two periods: two */
    ds_SimTick(&sim, &none, period - 1);
    TEST_EQUAL(period, sim.us_time);
    TEST_EQUAL(1, sim.dynamics.frames_completed);
    ds_SimTick(&sim, &none, 2 * period);
    TEST_EQUAL(3 * period, sim.us_time);
    TEST_EQUAL(3, sim.dynamics.frames_completed);

    /* flush: back to 0, reseeded (the seed batch ran) */
    ds_SimFlush(&sim);
    TEST_EQUAL(0, sim.us_time);
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

/* Write a ConfigLoad by hand at ss's bit offset: type, size field, size bytes of body (then pad zero bytes). */
static void SimCmdTestWriteConfigLoad(struct SS *ss, const u64 type, const u64 size_field, const u8 *body, const u64 size, const u64 pad)
{
    SSWriteU64Partial(ss, type, 8);
    SSWriteU64Partial(ss, size_field, 32);
    struct SS from = SSBuffered((void *) body, size);
    while (SSBitsLeft(&from))
    {
        const u64 bit_count = U64Min(SSBitsLeft(&from), 64);
        SSWriteU64Partial(ss, SSReadU64Partial(&from, bit_count), bit_count);
    }

    for (u64 i = 0; i < pad; ++i)
    {
        SSWriteU64Partial(ss, 0, 8);
    }
}

/*
 * ConfigLoad commands through ds_SimCmdPayloadPushAll and back through ds_SimCmdTryDeserialize: two commands pack to
 * exactly their bit size and read back equal, the stream ends after them, and a read at a 3-bit offset works. Malformed
 * commands are refused with the stream position untouched: unknown type, body size over the stream or over the cap,
 * a truncated body, a body longer than the config and a body that is not a config.
 */
static struct test_Output SimCmdSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_SimConfig changed = ds_SimConfigDefault();
    changed.us_system_tick[SIM_SYSTEM_DYNAMICS] = 20000;
    const u64 seed[4] = { 7, 8, 9, 10 };
    ds_SimConfigSeed(&changed, seed);
    const u64 body_size = ds_SimConfigSerializeSize(&changed);
    const u64 cmd_bits = 8 + 32 + 8*body_size;

    ds_CPool(ds_SimCmd) pool;
    ds_CPoolAlloc(env->mem_1, pool, 2, NOT_GROWABLE);
    const struct ds_SimConfig def = ds_SimConfigDefault();
    const struct ds_SimCmd first = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = &def };
    const struct ds_SimCmd second = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = &changed };
    ds_CPoolPushValue(pool, first);
    ds_CPoolPushValue(pool, second);

    struct ds_SimCmdPayload payload = ds_SimCmdPayloadPushAll(env->mem_1, ds_SimIdInit(1, 0), &pool);
    TEST_EQUAL(2, payload.cmd_count);
    TEST_EQUAL(2*cmd_bits, payload.stream.bit_index);

    /* read back in order, then the stream is empty */
    struct SSView view;
    TEST_EQUAL(1, SSTryView(&view, &payload.stream, 0, payload.stream.bit_index));
    struct SS ss = SSSubstreamFromView(&view);
    struct ds_SimCmd read = { 0 };
    TEST_EQUAL(1, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(SIM_CMD_CONFIG_LOAD, read.type);
    TEST_EQUAL(1, SimConfigTestEqual(&def, read.load_config.config));
    TEST_EQUAL(1, ds_SimCmdFormVerify(&read));
    const struct ds_SimConfig *first_read = read.load_config.config;
    TEST_EQUAL(1, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(1, SimConfigTestEqual(&changed, read.load_config.config));
    TEST_TRUE(first_read != read.load_config.config && SimConfigTestEqual(&def, first_read));  /* each decode has its own copy on mem */
    TEST_EQUAL(0, SSBitsLeft(&ss));
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(2*cmd_bits, ss.bit_index);

    read.type = SIM_CMD_COUNT;
    TEST_EQUAL(0, ds_SimCmdFormVerify(&read));

    /* the serialized config by itself, for the hand-written cases */
    struct SS body = SSAlloc(env->mem_1, body_size);
    ds_SimConfigSerialize(&body, &changed);

    /* at a 3-bit offset */
    struct SS hand = SSAlloc(env->mem_1, body_size + 16);
    SSWriteU64Partial(&hand, 5, 3);
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, body_size, body.buf, body_size, 0);
    TEST_EQUAL(1, SSTryView(&view, &hand, 3, cmd_bits));
    ss = SSSubstreamFromView(&view);
    TEST_EQUAL(1, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(1, SimConfigTestEqual(&changed, read.load_config.config));
    TEST_EQUAL(0, SSBitsLeft(&ss));

    /* unknown type */
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_COUNT, body_size, body.buf, body_size, 0);
    ss = SSBuffered(hand.buf, body_size + 16);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(0, ss.bit_index);

    /* size over the stream, and over the cap */
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, body_size + 16, body.buf, body_size, 0);
    ss = SSBuffered(hand.buf, body_size + 16);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(0, ss.bit_index);
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, U32_MAX, body.buf, body_size, 0);
    ss = SSBuffered(hand.buf, body_size + 16);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));

    /* truncated: the stream ends inside the body */
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, body_size, body.buf, body_size, 0);
    TEST_EQUAL(1, SSTryView(&view, &hand, 0, cmd_bits - 8));
    ss = SSSubstreamFromView(&view);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(0, ss.bit_index);

    /* a body one byte longer than the config, and a body that is no config (zeroed) */
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, body_size + 1, body.buf, body_size, 1);
    ss = SSBuffered(hand.buf, body_size + 16);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(0, ss.bit_index);
    u8 *zero = ArenaPush(env->mem_1, body_size);
    memset(zero, 0, body_size);
    hand.bit_index = 0;
    SimCmdTestWriteConfigLoad(&hand, SIM_CMD_CONFIG_LOAD, body_size, zero, body_size, 0);
    ss = SSBuffered(hand.buf, body_size + 16);
    TEST_EQUAL(0, ds_SimCmdTryDeserialize(env->mem_1, &ss, &read));
    TEST_EQUAL(0, ss.bit_index);

	return output;
}

/* Run a zero-delta frame with the pool as the frame's commands. */
static void SimTestBatch(struct ds_Sim *sim, const ds_CPool(ds_SimCmd) *pool)
{
    ds_SimTick(sim, pool, 0);
}

/*
 * ConfigLoad through the tick: ds_SimAlloc's seed batch applied the config (initialized); a frame batch with more
 * solver iterations is applied to the dynamics and leaves the tick periods alone; a batch of two commands runs in
 * order (the second wins); one with another dynamics period is rejected and changes nothing; ds_SimFlush clears
 * initialized and reseeds.
 */
static struct test_Output SimConfigLoadTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct c_ShapeDB cshape_db = { 0 };
    c_ShapeDBAlloc(NULL, &cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);

    struct ds_SimConfig config = ds_SimConfigDefault();
    config.dynamics.initial_size = 32;
    const u32 iterations = config.dynamics.solver.pgs_iteration_count;
    struct arena mem = ArenaAlloc(NULL, ds_DynamicsShellMemoryRequirement(g_config->thread_count, config.dynamics.frame_memory));
    struct ds_Sim sim;
    ds_SimAlloc(&mem, &sim, &config, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    TEST_EQUAL(1, sim.initialized);     /* the seed batch ran */
    TEST_EQUAL(iterations, sim.dynamics.config.solver.pgs_iteration_count);

    ds_CPool(ds_SimCmd) pool;
    ds_CPoolAlloc(env->mem_1, pool, 2, NOT_GROWABLE);
    struct ds_SimCmd cmd = { .type = SIM_CMD_CONFIG_LOAD };

    /* a runtime load: more iterations */
    struct ds_SimConfig runtime = config;
    runtime.dynamics.solver.pgs_iteration_count = iterations + 3;
    cmd.load_config.config = &runtime;
    ds_CPoolPushValue(pool, cmd);
    SimTestBatch(&sim, &pool);
    TEST_EQUAL(iterations + 3, sim.dynamics.config.solver.pgs_iteration_count);
    TEST_EQUAL(iterations + 3, sim.dynamics.config_pending.solver.pgs_iteration_count);

    /* two commands in one batch run in order: the second wins */
    struct ds_SimConfig second = config;
    second.dynamics.solver.pgs_iteration_count = iterations + 5;
    cmd.load_config.config = &second;
    ds_CPoolPushValue(pool, cmd);
    SimTestBatch(&sim, &pool);
    TEST_EQUAL(iterations + 5, sim.dynamics.config.solver.pgs_iteration_count);

    /* an alloc-time value changed: rejected whole */
    ds_CPoolFlush(pool);
    struct ds_SimConfig rejected = runtime;
    rejected.dynamics.solver.pgs_iteration_count = iterations + 7;
    rejected.us_system_tick[SIM_SYSTEM_DYNAMICS] = 2 * config.us_system_tick[SIM_SYSTEM_DYNAMICS];
    cmd.load_config.config = &rejected;
    ds_CPoolPushValue(pool, cmd);
    SimTestBatch(&sim, &pool);
    TEST_EQUAL(iterations + 5, sim.dynamics.config.solver.pgs_iteration_count);
    TEST_EQUAL(config.us_system_tick[SIM_SYSTEM_DYNAMICS], sim.us_system_tick[SIM_SYSTEM_DYNAMICS]);
    rejected = runtime;
    rejected.dynamics.initial_size = 64;
    SimTestBatch(&sim, &pool);
    TEST_EQUAL(iterations + 5, sim.dynamics.config.solver.pgs_iteration_count);

    ds_SimFlush(&sim);
    TEST_EQUAL(1, sim.initialized);     /* reseeded */
    TEST_EQUAL(iterations, sim.dynamics.config.solver.pgs_iteration_count);

    ds_SimDealloc(&sim);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

/*
 * The server's clock: a hosted simulation ticked through ds_ServerTick with wall-clock deltas. Sub-microsecond
 * deltas accumulate in the host's remainder until they make a microsecond, the budget caps a long frame at the sum
 * of the periods, speed 0 holds time still and speed 2 doubles it.
 */
static struct test_Output ServerClockTest(struct test_Environment *env)
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

    struct ds_Server srv;
    ds_ServerAlloc(&srv, 1 << 20, 2);
    const ds_SimId id = ds_SimAdd(&srv, &mem, &config, SIM_MODE_NONE, NULL, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    struct ds_SimHost *host = ds_SimLookup(&srv, id).address;
    struct ds_Sim *sim = &host->sim;
    TEST_TRUE(host != NULL && host->mode == SIM_MODE_NONE && host->speed == 1.0 && sim->initialized);

    /* sub-microsecond deltas: the remainder carries them */
    ds_ServerTick(&srv, 600);
    TEST_EQUAL(0, sim->us_delta);
    TEST_EQUAL(600, host->ns_remainder);
    ds_ServerTick(&srv, 600);
    TEST_EQUAL(1, sim->us_delta);
    TEST_EQUAL(200, host->ns_remainder);
    TEST_EQUAL(1, sim->us_time);
    TEST_EQUAL(0, sim->dynamics.frames_completed);

    /* one period: the first physics tick */
    ds_ServerTick(&srv, 1000 * (period - 1));
    TEST_EQUAL(period, sim->us_time);
    TEST_EQUAL(1, sim->dynamics.frames_completed);

    /* the budget caps a long frame at the sum of the periods: two ticks, not twenty */
    ds_ServerTick(&srv, 20 * 1000 * period);
    TEST_EQUAL(host->ns_budget / 1000, sim->us_delta);
    TEST_EQUAL(period + host->ns_budget / 1000, sim->us_time);
    TEST_EQUAL(1 + host->ns_budget / (1000 * period), sim->dynamics.frames_completed);

    /* speed 0: time stands still, the remainder too */
    host->speed = 0.0;
    ds_ServerTick(&srv, 1000 * period);
    TEST_EQUAL(0, sim->us_delta);
    TEST_EQUAL(period + host->ns_budget / 1000, sim->us_time);
    host->speed = 2.0;
    ds_ServerTick(&srv, 500 * period);
    TEST_EQUAL(period, sim->us_delta);

    ds_ServerDealloc(&srv);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

/*
 * The server's door: a server with one hand-wired simulation and four arrivals in payload_pool, a ConfigLoad, a
 * payload for an unknown id, a payload whose command bits are corrupt, a payload whose command count is wrong, and a
 * second ConfigLoad. One ds_ServerTick: the two good payloads are decoded onto the simulation's pool in arrival order
 * and executed by its tick (the last wins), the bad ones are dropped before the fuse (the fused payload holds the two
 * good ones), payload_pool and cmd_pool are empty afterwards. An empty frame changes nothing.
 */
static struct test_Output ServerDoorTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct c_ShapeDB cshape_db = { 0 };
    c_ShapeDBAlloc(NULL, &cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);

    struct ds_SimConfig config = ds_SimConfigDefault();
    config.dynamics.initial_size = 32;
    const u32 iterations = config.dynamics.solver.pgs_iteration_count;
    struct arena mem = ArenaAlloc(NULL, ds_DynamicsShellMemoryRequirement(g_config->thread_count, config.dynamics.frame_memory));

    struct ds_Server srv;
    ds_ServerAlloc(&srv, 1 << 20, 2);
    const ds_SimId id = ds_SimAdd(&srv, &mem, &config, SIM_MODE_NONE, NULL, &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    struct ds_SimHost *host = ds_SimLookup(&srv, id).address;
    struct ds_Sim *sim = &host->sim;
    TEST_EQUAL(NULL, ds_SimLookup(&srv, ds_SimIdInit(1, 0)).address);

    /* the arrivals */
    struct ds_SimConfig load[2] = { config, config };
    load[0].dynamics.solver.pgs_iteration_count = iterations + 3;
    load[1].dynamics.solver.pgs_iteration_count = iterations + 5;
    ds_CPool(ds_SimCmd) pool;
    ds_CPoolAlloc(env->mem_1, pool, 1, NOT_GROWABLE);
    struct ds_SimCmd cmd = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = &load[0] };
    ds_CPoolPushValue(pool, cmd);

    struct ds_SimCmdPayload good_first = ds_SimCmdPayloadPushAll(env->mem_1, host->id, &pool);
    struct ds_SimCmdPayload unknown = ds_SimCmdPayloadPushAll(env->mem_1, ds_SimIdInit(1, 0), &pool);
    struct ds_SimCmdPayload corrupt = ds_SimCmdPayloadPushAll(env->mem_1, host->id, &pool);
    corrupt.stream.buf[0] = 0xff;                                               /* the type */
    struct ds_SimCmdPayload miscounted = ds_SimCmdPayloadPushAll(env->mem_1, host->id, &pool);
    miscounted.cmd_count = 2;
    cmd.load_config.config = &load[1];
    ds_CPoolFlush(pool);
    ds_CPoolPushValue(pool, cmd);
    struct ds_SimCmdPayload good_second = ds_SimCmdPayloadPushAll(env->mem_1, host->id, &pool);
    const u64 cmd_bits = good_first.stream.bit_index;

    ds_CPoolPushValue(srv.payload_pool, good_first);
    ds_CPoolPushValue(srv.payload_pool, unknown);
    ds_CPoolPushValue(srv.payload_pool, corrupt);
    ds_CPoolPushValue(srv.payload_pool, miscounted);
    ds_CPoolPushValue(srv.payload_pool, good_second);

    ds_ServerTick(&srv, 0);
    TEST_EQUAL(iterations + 5, sim->dynamics.config.solver.pgs_iteration_count);
    TEST_EQUAL(2, host->frame_payload.cmd_count);
    TEST_EQUAL(2*cmd_bits, host->frame_payload.stream.bit_index);
    TEST_EQUAL(0, srv.payload_pool.count);
    TEST_EQUAL(0, host->cmd_pool.count);

    /* an empty frame */
    ds_ServerTick(&srv, 0);
    TEST_EQUAL(iterations + 5, sim->dynamics.config.solver.pgs_iteration_count);
    TEST_EQUAL(0, host->frame_payload.cmd_count);

    ds_ServerDealloc(&srv);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

/*
 * Recording: a RECORD simulation writes its record file in the build directory. Entries: the seed (timestamp 0, one
 * ConfigLoad equal to the config), a frame with two commands at timestamp 0 (no time passed yet), nothing for an
 * empty frame, and a frame with one command at the period. Removing the simulation closes the file and frees its
 * id; the file reads back entry by entry with the payload and command readers.
 */
static struct test_Output ServerRecordTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct c_ShapeDB cshape_db = { 0 };
    c_ShapeDBAlloc(NULL, &cshape_db, 32, GROWABLE);
    ds_SDB(ds_BodyPrefab) prefab_db = { 0 };
    ds_SDBAlloc(NULL, prefab_db, 32, GROWABLE);

    struct ds_SimConfig config = ds_SimConfigDefault();
    config.dynamics.initial_size = 32;
    const u32 iterations = config.dynamics.solver.pgs_iteration_count;
    const u64 period = config.us_system_tick[SIM_SYSTEM_DYNAMICS];
    struct arena mem = ArenaAlloc(NULL, ds_DynamicsShellMemoryRequirement(g_config->thread_count, config.dynamics.frame_memory) + 1024);

    struct ds_Server srv;
    ds_ServerAlloc(&srv, 1 << 20, 2);
    const ds_SimId id = ds_SimAdd(&srv, &mem, &config, SIM_MODE_RECORD, "agent_test_record.bin", &cshape_db, &prefab_db, g_config->thread_count, g_config->thread_framesize);
    TEST_EQUAL(0, ds_SimIdNullCheck(id));
    struct ds_SimHost *host = ds_SimLookup(&srv, id).address;
    TEST_EQUAL(SIM_MODE_RECORD, host->mode);

    /* frame 1: two commands at time 0; frame 2: a period, no commands; frame 3: one command */
    struct ds_SimConfig load[3] = { config, config, config };
    ds_CPool(ds_SimCmd) pool;
    ds_CPoolAlloc(env->mem_1, pool, 2, NOT_GROWABLE);
    for (u32 i = 0; i < 3; ++i)
    {
        load[i].dynamics.solver.pgs_iteration_count = iterations + 3 + 2*i;
        const struct ds_SimCmd cmd = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = &load[i] };
        ds_CPoolPushValue(pool, cmd);
        if (i == 1)
        {
            ds_CPoolPushValue(srv.payload_pool, ds_SimCmdPayloadPushAll(env->mem_1, id, &pool));
            ds_ServerTick(&srv, 0);
            ds_ServerTick(&srv, 1000 * period);
            ds_CPoolFlush(pool);
        }
    }
    ds_CPoolPushValue(srv.payload_pool, ds_SimCmdPayloadPushAll(env->mem_1, id, &pool));
    ds_ServerTick(&srv, 0);
    TEST_EQUAL(iterations + 7, host->sim.dynamics.config.solver.pgs_iteration_count);
    TEST_EQUAL(period, host->sim.us_time);

    ds_SimRemove(&srv, id);
    TEST_EQUAL(NULL, ds_SimLookup(&srv, id).address);
    TEST_EQUAL(0, srv.sim_list.count);

    /* read it back */
    const struct dsBuffer file = FileDumpAtCwd(env->mem_1, "agent_test_record.bin");
    TEST_TRUE(file.size > 0);
    struct SS ss = SSBuffered(file.data, file.size);
    const u64 timestamps[3] = { 0, 0, period };
    const u32 counts[3] = { 1, 2, 1 };
    const u32 first_iterations[3] = { iterations, iterations + 3, iterations + 7 };
    for (u32 e = 0; e < 3; ++e)
    {
        TEST_EQUAL(timestamps[e], SSReadU64Le(&ss));
        struct ds_SimCmdPayload payload;
        TEST_EQUAL(1, ds_SimCmdPayloadTryDeserializeAndAlias(&ss, &payload));
        TEST_EQUAL(id.val, payload.dst.val);
        TEST_EQUAL(counts[e], payload.cmd_count);

        struct SSView view;
        TEST_EQUAL(1, SSTryView(&view, &payload.stream, 0, payload.stream.bit_index));
        struct SS cmds = SSSubstreamFromView(&view);
        struct ds_SimCmd cmd;
        TEST_EQUAL(1, ds_SimCmdTryDeserialize(env->mem_1, &cmds, &cmd));
        TEST_EQUAL(first_iterations[e], cmd.load_config.config->dynamics.solver.pgs_iteration_count);
        TEST_EQUAL(1, SimConfigTestEqual(&config, cmd.load_config.config));
        for (u32 c = 1; c < counts[e]; ++c)
        {
            TEST_EQUAL(1, ds_SimCmdTryDeserialize(env->mem_1, &cmds, &cmd));
        }
        TEST_EQUAL(0, SSBitsLeft(&cmds));
    }
    TEST_EQUAL(0, SSBitsLeft(&ss));

    ds_ServerDealloc(&srv);
    ArenaFree(&mem);
    ds_SDBDealloc(prefab_db);
    c_ShapeDBDealloc(&cshape_db);
	return output;
}

static struct test_Output (*simulation_tests[])(struct test_Environment *) =
{
    SimConfigSerializeTest,
    DynamicsConfigSerializeTest,
    SimCmdPayloadSerializeTest,
    SimCmdSerializeTest,
    SimTickTest,
    SimConfigLoadTest,
    ServerClockTest,
    ServerDoorTest,
    ServerRecordTest,
};

struct suite_Correctness m_simulation_suite =
{
	.id = "simulation",
	.unit_test = simulation_tests,
	.unit_test_count = sizeof(simulation_tests) / sizeof(simulation_tests[0]),
};

struct suite_Correctness *simulation_correctness_suite = &m_simulation_suite;

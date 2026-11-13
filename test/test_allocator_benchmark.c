/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt

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

/*
Slab benchmark: the mesh contact arrays (a manifold array and a tri cache array per contact) resized the
way the terrain scene resizes them, replayed on every worker against ds_TSlab and a no-allocator floor.
Per worker and kind SB_SLOTS contacts; a repetition is SB_TICKS ticks whose events (slot, two counts the
slot alternates between) are fixed per set at init, so every repetition does the same work. "rotate"
moves the set a worker replays to the next worker every second repetition, so arrays are freed by another
thread than the one that allocated them. "pow2" keeps an array while the count stays within its capacity.
The new counts and the change rate per tick come from the trace of DynamicsMeshArrayStatsTest (900 ticks,
64 contacts). Each event allocates the new array, copies the common prefix, frees the old one and writes
the new elements. Each worker holds one job before anyone starts (an arrival barrier), since the parallel
driver's master would otherwise run every job itself in the fastest repetitions.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ds_test.h"
#include "ds_init.h"

#define SB_SLOTS        512
#define SB_TICKS        8
#define SB_COUNT_MAX    9
#define SB_VM_SIZE      ((u64) 256 << 20)

struct sb_elem_m { u8 b[168]; };
struct sb_elem_c { u8 b[32]; };
DEFINE_ARRAY_STRUCT(sb_elem_m);
DEFINE_ARRAY_STRUCT(sb_elem_c);
DEFINE_TSLAB(sb_elem_m);
DEFINE_TSLAB(sb_elem_c);

enum sb_variant
{
    SB_NULL,
    SB_SLAB,
};

static const char *sb_variant_name[] = { "null", "ds_TSlab" };
static const u32 sb_elem_size[2] = { sizeof(struct sb_elem_m), sizeof(struct sb_elem_c) };
/* new-count histograms of the trace, manifold arrays and tri cache arrays */
static const u32 sb_hist[2][SB_COUNT_MAX] =
{
    { 193, 902, 1748, 1341, 391, 74, 7, 0, 0 },
    { 55, 109, 461, 716, 646, 298, 221, 125, 35 },
};
/* slots that change per tick, per mille */
static const u32 sb_change_permille[2] = { 81, 46 };

/* the slot goes to count, or to alt when it already holds count: every replay changes it */
struct sb_event
{
    u16 slot;
    u8  kind;
    u8  count;
    u8  alt;
};

struct sb_set
{
    u8                  count[2][SB_SLOTS];
    u8                  capacity[2][SB_SLOTS];
    u8 *                ptr[2][SB_SLOTS];
    u32                 index[2][SB_SLOTS];
    struct sb_event *   event;
    u32                 event_count;
    u64                 bytes;
    u64                 checksum;
    u8 *                null_buf[2];
};

struct sb_job
{
    u32 variant;
    u32 rotate;
    u32 pow2;
    u32 job_index;
};

/* pow2: arrays are allocated at the next power of two of the count and kept while the count stays within it */
static u32 g_pow2;

static struct sb_set *      g_set;
static u32                  g_set_count;
static u32                  g_init_count;
static u32                  g_rep;
static u32                  g_arrived;
static u32                  g_runs[64];
struct ds_Align(DS_CACHE_LINE) sb_counter { u64 alloc, free, copy; };
static struct sb_counter    g_counter[64];
static ds_TSlab(sb_elem_m)   g_slab_m;
static ds_TSlab(sb_elem_c)   g_slab_c;

static u32 sb_draw_count(struct rng *rng, const u32 kind)
{
    u32 total = 0;
    for (u32 i = 0; i < SB_COUNT_MAX; ++i) { total += sb_hist[kind][i]; }
    u64 r = RngLocalU64Range(rng, 0, total - 1);
    for (u32 i = 0; i < SB_COUNT_MAX; ++i)
    {
        if (r < sb_hist[kind][i]) { return i; }
        r -= sb_hist[kind][i];
    }
    return 0;
}

static u8 *sb_array_alloc(const u32 variant, const u32 kind, const u32 count, u32 *index)
{
    ds_Assert(variant == SB_SLAB);
    struct ds_ArrayUntyped arr;
    if (kind == 0) { const ds_Array(sb_elem_m) a = ds_TSlabAdd(g_slab_m, count); arr = (struct ds_ArrayUntyped) { .buf = a.buf, .length = a.length, .base = a.base }; }
    else           { const ds_Array(sb_elem_c) a = ds_TSlabAdd(g_slab_c, count); arr = (struct ds_ArrayUntyped) { .buf = a.buf, .length = a.length, .base = a.base }; }
    *index = arr.base;
    return arr.buf;
}

static void sb_array_free(const u32 variant, const u32 kind, u8 *ptr, const u32 index, const u32 count)
{
    ds_Assert(variant == SB_SLAB);
    (void) ptr;
    const u32 length = (u32) PowerOfTwoCeil(count);
    if (kind == 0) { const ds_Array(sb_elem_m) a = { .buf = NULL, .length = length, .base = index }; ds_TSlabRemove(g_slab_m, a); }
    else           { const ds_Array(sb_elem_c) a = { .buf = NULL, .length = length, .base = index }; ds_TSlabRemove(g_slab_c, a); }
}

/* one contact array goes from its current count to count: the new array, the common prefix, the old one, the new elements */
static void sb_resize(struct sb_set *s, const u32 variant, const u32 kind, const u32 slot, const u32 count)
{
    const u32 old = s->count[kind][slot];
    const u32 size = sb_elem_size[kind];
    u8 *p_old = s->ptr[kind][slot];
    u8 *p_new = NULL;
    u32 i_new = DS_TSLAB_NULL;
    u32 cap_old = s->capacity[kind][slot];
    u32 cap_new = count;
    if (g_pow2 && count)
    {
        cap_new = (u32) PowerOfTwoCeil(count);
    }

    if (variant == SB_NULL)
    {
        p_new = (count) ? s->null_buf[kind] + (u64) slot*SB_COUNT_MAX*size : NULL;
    }
    else if (cap_new == cap_old)
    {
        p_new = p_old;
        i_new = s->index[kind][slot];
    }
    else
    {
        if (cap_new)
        {
            p_new = sb_array_alloc(variant, kind, cap_new, &i_new);
        }

        if (old && count)
        {
            const u32 common = (old < count) ? old : count;
            memcpy(p_new, p_old, (u64) common*size);
            g_counter[ds_ThreadSelfIndex()].copy += (u64) common*size;
        }

        if (cap_old)
        {
            sb_array_free(variant, kind, p_old, s->index[kind][slot], cap_old);
        }
    }
    s->capacity[kind][slot] = (u8) cap_new;

    if (count > old)
    {
        memset(p_new + (u64) old*size, (int) (slot & 0xff), (u64) (count - old)*size);
    }

    s->ptr[kind][slot] = p_new;
    s->index[kind][slot] = i_new;
    s->count[kind][slot] = (u8) count;
}

static u64 sb_checksum(const struct sb_set *s)
{
    u64 sum = 0;
    for (u32 kind = 0; kind < 2; ++kind)
    {
        for (u32 slot = 0; slot < SB_SLOTS; ++slot)
        {
            const u32 count = s->count[kind][slot];
            if (count)
            {
                const u8 *p = s->ptr[kind][slot];
                u32 first;
                memcpy(&first, p, sizeof(first));
                sum += first + (u64) count*7 + p[(u64) count*sb_elem_size[kind] - 1];
            }
        }
    }
    return sum;
}

static void sb_set_init(struct sb_set *s, const u32 variant, const u32 set_index)
{
    struct rng rng;
    RngLocalInit(&rng, 0x5eed + set_index);
    memset(s, 0, sizeof(*s));
    for (u32 kind = 0; kind < 2; ++kind)
    {
        s->null_buf[kind] = malloc((u64) SB_SLOTS*SB_COUNT_MAX*sb_elem_size[kind]);
    }

    for (u32 kind = 0; kind < 2; ++kind)
    {
        for (u32 slot = 0; slot < SB_SLOTS; ++slot)
        {
            sb_resize(s, variant, kind, slot, sb_draw_count(&rng, kind));
        }
    }

    u8 cur[2][SB_SLOTS];
    memcpy(cur, s->count, sizeof(cur));
    s->event = malloc((u64) SB_TICKS*2*SB_SLOTS*sizeof(struct sb_event));
    for (u32 tick = 0; tick < SB_TICKS; ++tick)
    {
        for (u32 kind = 0; kind < 2; ++kind)
        {
            for (u32 slot = 0; slot < SB_SLOTS; ++slot)
            {
                if (RngLocalU64Range(&rng, 0, 999) < sb_change_permille[kind])
                {
                    const u32 count = sb_draw_count(&rng, kind);
                    if (count != cur[kind][slot])
                    {
                        u32 alt = sb_draw_count(&rng, kind);
                        while (alt == count) { alt = sb_draw_count(&rng, kind); }
                        s->event[s->event_count++] = (struct sb_event) { .slot = (u16) slot, .kind = (u8) kind, .count = (u8) count, .alt = (u8) alt };
                        cur[kind][slot] = (u8) count;
                    }
                }
            }
        }
    }

    /* bytes a repetition moves, averaged over the two steady-state replays (count and alt alternate) */
    for (u32 rep = 0; rep < 3; ++rep)
    {
        for (u32 i = 0; i < s->event_count; ++i)
        {
            const struct sb_event e = s->event[i];
            const u32 old = cur[e.kind][e.slot];
            const u32 count = (old == e.count) ? e.alt : e.count;
            const u32 common = (old < count) ? old : count;
            const u32 fresh = (count > old) ? count - old : 0;
            if (rep) { s->bytes += (u64) (2*common + fresh)*sb_elem_size[e.kind]; }
            cur[e.kind][e.slot] = (u8) count;
        }
    }
    s->bytes /= 2;
}

static void sb_set_free(struct sb_set *s, const u32 variant)
{
    for (u32 kind = 0; kind < 2; ++kind)
    {
        for (u32 slot = 0; slot < SB_SLOTS; ++slot)
        {
            if (s->count[kind][slot])
            {
                sb_resize(s, variant, kind, slot, 0);
            }
        }
        free(s->null_buf[kind]);
    }
    free(s->event);
}

static void *sb_init(const u32 variant, const u32 rotate, const u32 pow2)
{
    if (g_init_count == 0)
    {
        g_pow2 = pow2;
        g_rep = 0;
        g_arrived = 0;
        memset(g_runs, 0, sizeof(g_runs));
        memset(g_counter, 0, sizeof(g_counter));
        g_set_count = g_config->thread_count;
        g_set = malloc(g_set_count*sizeof(struct sb_set));
        if (variant == SB_SLAB)
        {
            ds_TSlabAlloc(NULL, g_slab_m, SB_VM_SIZE / sizeof(g_slab_m.buf[0]), g_set_count);
            ds_TSlabAlloc(NULL, g_slab_c, SB_VM_SIZE / sizeof(g_slab_c.buf[0]), g_set_count);
        }
        for (u32 i = 0; i < g_set_count; ++i)
        {
            sb_set_init(g_set + i, variant, i);
        }
    }

    struct sb_job *job = malloc(sizeof(*job));
    job->variant = variant;
    job->rotate = rotate;
    job->pow2 = pow2;
    job->job_index = g_init_count;
    g_init_count += 1;
    return job;
}

static void sb_reset(void *args)
{
    const struct sb_job *job = args;
    if (job->job_index == 0)
    {
        g_rep += 1;
    }
}

static void sb_test(void *args)
{
    const struct sb_job *job = args;
    /* every worker holds one job before anyone starts: the master must not run the others itself */
    AtomicFetchAddRlx32(&g_arrived, 1);
    while (AtomicLoadAcq32(&g_arrived) < g_rep*g_set_count);
    g_runs[ds_ThreadSelfIndex()] += 1;
    /* the set moves to the next worker every second repetition: both toggle directions run on one worker, so
     * no worker's lists drift */
    u32 set_index = ds_ThreadSelfIndex();
    if (job->rotate)
    {
        set_index = (set_index + g_rep/2) % g_set_count;
    }
    struct sb_set *s = g_set + set_index;

    for (u32 i = 0; i < s->event_count; ++i)
    {
        const struct sb_event e = s->event[i];
        const u32 count = (s->count[e.kind][e.slot] == e.count) ? e.alt : e.count;
        sb_resize(s, job->variant, e.kind, e.slot, count);
    }
    s->checksum = sb_checksum(s);
}

static void sb_free(void *args)
{
    struct sb_job *job = args;
    if (job->job_index == 0)
    {
        u64 events = 0, bytes = 0, checksum = 0, allocs = 0, frees = 0, copied = 0;
        for (u32 i = 0; i < g_set_count; ++i)
        {
            events += g_set[i].event_count;
            bytes += g_set[i].bytes;
            checksum += g_set[i].checksum;
            allocs += g_counter[i].alloc;
            frees += g_counter[i].free;
            copied += g_counter[i].copy;
        }
        fprintf(stdout, "\t%s %s%s: %u workers, %llu events and %llu bytes per repetition, %u repetitions, checksum %llx, runs per worker",
                sb_variant_name[job->variant], (job->rotate) ? "rotate" : "fixed", (job->pow2) ? " pow2" : "", g_set_count,
                (unsigned long long) events, (unsigned long long) bytes, g_rep, (unsigned long long) checksum);
        for (u32 i = 0; i < g_set_count; ++i) { fprintf(stdout, " %u", g_runs[i]); }
        fprintf(stdout, ", per repetition %.1f allocs %.1f frees %.0f copied bytes\n",
                (f64) allocs / g_rep, (f64) frees / g_rep, (f64) copied / g_rep);
    }

    g_init_count -= 1;
    if (g_init_count == 0)
    {
        for (u32 i = 0; i < g_set_count; ++i)
        {
            sb_set_free(g_set + i, job->variant);
        }
        free(g_set);
        if (job->variant == SB_SLAB)
        {
            ds_TSlabDealloc(g_slab_m);
            ds_TSlabDealloc(g_slab_c);
        }
    }
    free(job);
}

#define SB_INIT(name, variant, rotate, pow2) \
static void *sb_init_##name(void) { return sb_init(variant, rotate, pow2); }
SB_INIT(null_fixed, SB_NULL, 0, 0)
SB_INIT(slab_fixed, SB_SLAB, 0, 0)
SB_INIT(slab_rotate, SB_SLAB, 1, 0)
SB_INIT(slab_fixed_pow2, SB_SLAB, 0, 1)
SB_INIT(slab_rotate_pow2, SB_SLAB, 1, 1)

#define SB_ENTRY(name, id_str) { .id = id_str, .size = 1, .test = sb_test, .test_init = sb_init_##name, .test_reset = sb_reset, .test_free = sb_free }

static struct test_PerformanceParallel slab_parallel_test[] =
{
    SB_ENTRY(null_fixed, "MeshArraysNull"),
    SB_ENTRY(slab_fixed, "MeshArraysSlabFixed"),
    SB_ENTRY(slab_rotate, "MeshArraysSlabRotate"),
    SB_ENTRY(slab_fixed_pow2, "MeshArraysSlabFixedPow2"),
    SB_ENTRY(slab_rotate_pow2, "MeshArraysSlabRotatePow2"),
};

static struct suite_Performance storage_performance_slab_suite =
{
    .id = "Slab Performance",
    .parallel_test = slab_parallel_test,
    .parallel_test_count = sizeof(slab_parallel_test) / sizeof(slab_parallel_test[0]),
    .serial_test = NULL,
    .serial_test_count = 0,
};

struct suite_Performance *slab_performance_suite = &storage_performance_slab_suite;

/*
TStack contention: every worker pushes one node onto one shared ds_TStack and pops one back, TS_OPS times per
repetition, so all workers hammer the head's cache line. Variants: the engine's macros, and copies that pause
(ds_CpuPause) after a failed compare-exchange, once or TS_PAUSE_LONG times. Per worker: the pairs, the failed
compare-exchanges and the empty pops (retried), so a difference can be read against how often the pause ran.
*/

#define TS_OPS          100000
#define TS_PAUSE_LONG   8

struct ds_Align(DS_CACHE_LINE) ts_node { TREIBER_NODE; u8 pad[DS_CACHE_LINE - sizeof(u32)]; };
struct ds_Align(DS_CACHE_LINE) ts_counter { u64 cas_fail, pop_miss, pairs; };

enum ts_variant { TS_PLAIN, TS_PAUSE_1, TS_PAUSE_LONG_V };
static const char *ts_variant_name[] = { "plain", "pause 1", "pause 8" };

static struct ds_TStack    g_ts_stack;
static struct ts_node      g_ts_node[64];
static struct ts_counter   g_ts_counter[64];
static u32                 g_ts_init_count;
static u32                 g_ts_rep;
static u32                 g_ts_arrived;
static u32                 g_ts_workers;

struct ts_job { u32 variant; u32 job_index; };

/* ds_TStackPush / ds_TStackTryPop with _pause_ pause instructions after a failed compare-exchange */
#define TS_PUSH_PAUSE(_stack_, _buf_, _index_, _pause_)                                 \
do                                                                                      \
{                                                                                       \
    ds_TStackKey _head_ = AtomicLoadRlx64(&(_stack_)->a_head);                          \
    for (;;)                                                                            \
    {                                                                                   \
        AtomicStoreRlx32(&(_buf_)[_index_].treiber_next, ds_TStackKeyIndex(_head_));    \
        const ds_TStackKey _new_head_ = ds_TStackKeyInit((_index_), ds_TStackKeyTag(_head_) + 1); \
        if (AtomicCompareExchangeRelRlx64(&(_stack_)->a_head, &_head_, _new_head_)) { break; } \
        g_ts_counter[ds_ThreadSelfIndex()].cas_fail += 1;                               \
        ds_CpuPause(_pause_);                                                           \
    }                                                                                   \
} while (0)

#define TS_POP_PAUSE(_index_, _stack_, _buf_, _pause_)                                  \
do                                                                                      \
{                                                                                       \
    ds_TStackKey _head_ = AtomicLoadAcq64(&(_stack_)->a_head);                          \
    for (;;)                                                                            \
    {                                                                                   \
        (_index_) = ds_TStackKeyIndex(_head_);                                          \
        if ((_index_) == TSTACK_NULL) { break; }                                        \
        const ds_TStackKey _new_head_ = ds_TStackKeyInit(AtomicLoadRlx32(&(_buf_)[_index_].treiber_next), ds_TStackKeyTag(_head_) + 1); \
        if (AtomicCompareExchangeAcqRelAcq64(&(_stack_)->a_head, &_head_, _new_head_)) { break; } \
        g_ts_counter[ds_ThreadSelfIndex()].cas_fail += 1;                               \
        ds_CpuPause(_pause_);                                                           \
    }                                                                                   \
} while (0)

static void *ts_init(const u32 variant)
{
    if (g_ts_init_count == 0)
    {
        g_ts_rep = 0;
        g_ts_arrived = 0;
        g_ts_workers = g_config->thread_count;
        memset(g_ts_counter, 0, sizeof(g_ts_counter));
        for (u32 i = 0; i < 64; ++i) { g_ts_node[i].treiber_next = TSTACK_NULL; }
        ds_TStackFlush(&g_ts_stack);
    }
    struct ts_job *job = malloc(sizeof(*job));
    job->variant = variant;
    job->job_index = g_ts_init_count;
    g_ts_init_count += 1;
    return job;
}

static void ts_reset(void *args)
{
    const struct ts_job *job = args;
    if (job->job_index == 0) { g_ts_rep += 1; }
}

static void ts_test(void *args)
{
    const struct ts_job *job = args;
    AtomicFetchAddRlx32(&g_ts_arrived, 1);
    while (AtomicLoadAcq32(&g_ts_arrived) < g_ts_rep*g_ts_workers);
    const u32 self = ds_ThreadSelfIndex();
    struct ts_counter *c = g_ts_counter + self;
    u32 node = self;
    for (u32 i = 0; i < TS_OPS; ++i)
    {
        u32 popped;
        switch (job->variant)
        {
            case TS_PLAIN:
            {
                ds_TStackPush(&g_ts_stack, g_ts_node, node);
                do { ds_TStackTryPop(popped, &g_ts_stack, g_ts_node); c->pop_miss += (popped == TSTACK_NULL); } while (popped == TSTACK_NULL);
            } break;

            case TS_PAUSE_1:
            {
                TS_PUSH_PAUSE(&g_ts_stack, g_ts_node, node, 1);
                do { TS_POP_PAUSE(popped, &g_ts_stack, g_ts_node, 1); c->pop_miss += (popped == TSTACK_NULL); } while (popped == TSTACK_NULL);
            } break;

            default:
            {
                TS_PUSH_PAUSE(&g_ts_stack, g_ts_node, node, TS_PAUSE_LONG);
                do { TS_POP_PAUSE(popped, &g_ts_stack, g_ts_node, TS_PAUSE_LONG); c->pop_miss += (popped == TSTACK_NULL); } while (popped == TSTACK_NULL);
            } break;
        }
        g_ts_node[popped].treiber_next = TSTACK_NULL;
        node = popped;
    }
    c->pairs += TS_OPS;
}

static void ts_free(void *args)
{
    struct ts_job *job = args;
    if (job->job_index == 0)
    {
        u64 pairs = 0, cas_fail = 0, pop_miss = 0;
        for (u32 i = 0; i < g_ts_workers; ++i)
        {
            pairs += g_ts_counter[i].pairs;
            cas_fail += g_ts_counter[i].cas_fail;
            pop_miss += g_ts_counter[i].pop_miss;
        }
        fprintf(stdout, "\tTStack %s: %u workers, %u pairs per worker and repetition, %u repetitions; per pair %.3f failed CAS (plain: not counted) %.3f empty pops\n",
                ts_variant_name[job->variant], g_ts_workers, TS_OPS, g_ts_rep, (f64) cas_fail / (f64) pairs, (f64) pop_miss / (f64) pairs);
    }
    g_ts_init_count -= 1;
    free(job);
}

#define TS_INIT(name, variant) static void *ts_init_##name(void) { return ts_init(variant); }
TS_INIT(plain, TS_PLAIN)
TS_INIT(pause_1, TS_PAUSE_1)
TS_INIT(pause_long, TS_PAUSE_LONG_V)

#define TS_ENTRY(name, id_str) { .id = id_str, .size = 1, .test = ts_test, .test_init = ts_init_##name, .test_reset = ts_reset, .test_free = ts_free }

static struct test_PerformanceParallel tstack_parallel_test[] =
{
    TS_ENTRY(plain, "TStackPushPop"),
    TS_ENTRY(pause_1, "TStackPushPopPause1"),
    TS_ENTRY(pause_long, "TStackPushPopPause8"),
    TS_ENTRY(plain, "TStackPushPop"),
    TS_ENTRY(pause_1, "TStackPushPopPause1"),
    TS_ENTRY(pause_long, "TStackPushPopPause8"),
};

static struct suite_Performance storage_performance_tstack_suite =
{
    .id = "TStack Performance",
    .parallel_test = tstack_parallel_test,
    .parallel_test_count = sizeof(tstack_parallel_test) / sizeof(tstack_parallel_test[0]),
    .serial_test = NULL,
    .serial_test_count = 0,
};

struct suite_Performance *tstack_performance_suite = &storage_performance_tstack_suite;

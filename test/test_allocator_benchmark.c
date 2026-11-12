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
way the terrain scene resizes them, replayed on every worker against ds_TSlab, ds_Alloc/ds_Free, ds_Realloc
a central Treiber stack per count over the same region, per-thread Treiber stacks with stealing, per-thread
work-stealing deques (ds_WSDeque), Bonwick magazines, and a no-allocator floor. Per worker and kind SB_SLOTS contacts; a repetition is SB_TICKS ticks whose
events (slot, new count) are fixed per set at init, so every repetition from the second on does the same
work. "Rotate" shifts the set a worker replays by one per repetition, so arrays are freed by another
thread than the one that allocated them. The new counts and the change rate per tick come from the trace
of DynamicsMeshArrayStatsTest (900 ticks, 64 contacts). Each event allocates the new array, copies the
common prefix, frees the old one and writes the new elements.
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
    SB_ALLOC,
    SB_REALLOC,
    SB_CENTRAL,
    SB_STEAL,
    SB_DEQUE,
    SB_MAGAZINE,
    SB_OLD,
};

/* the full set of variants (ds_Alloc, ds_Realloc, the stack and deque designs) only on request */
#ifndef SB_EXTRA_VARIANTS
#define SB_EXTRA_VARIANTS 0
#endif

static const char *sb_variant_name[] =
{
    "null", "ds_TSlab", "ds_Alloc", "ds_Realloc", "central TStack", "per-thread TStack steal", "per-thread WSDeque steal",
    "magazines", "ds_TSlab old (per-thread lists)",
};
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

/*
central: the slab's region and bump with one Treiber stack per count shared by all threads. The head holds a
push/pop counter in its upper 32 bits (the ABA guard; the array's bytes are user data while live, so no tag can
live there) and the index below; the freed array's first u64 holds the previous head. An empty stack has index
U32_MAX.
*/
struct ds_Align(DS_CACHE_LINE) sb_cstack
{
    u64 a_head;
    u8  pad[DS_CACHE_LINE - sizeof(u64)];
};

struct sb_central
{
    struct VM           vm;
    u32                 a_count_max;
    u32                 length;
    u64                 slot_size;
    struct sb_cstack    free[SB_COUNT_MAX];
};

/* steal: one stack per thread and count (thread-major); Add pops its own, then every other thread's, then carves */
static struct sb_central    g_central[2];
static struct sb_cstack *   g_steal[2];
static u32                  g_steal_threads;

static void sb_central_alloc(struct sb_central *c, const u64 slot_size)
{
    c->vm = VMReserve(SB_VM_SIZE);
    c->a_count_max = 0;
    c->slot_size = slot_size;
    c->length = (u32) (c->vm.reserved / slot_size);
    for (u32 i = 0; i < SB_COUNT_MAX; ++i) { c->free[i].a_head = U32_MAX; }
}

static struct slot sb_cstack_pop(struct sb_cstack *stack, const struct sb_central *c)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    u64 head = AtomicLoadAcq64(&stack->a_head);
    while ((u32) head != U32_MAX)
    {
        u8 *node = c->vm.buf + (u64) (u32) head*c->slot_size;
        const u64 link = AtomicLoadRlx64((u64 *) node);
        const u64 new_head = (((head >> 32) + 1) << 32) | (u32) link;
        if (AtomicCompareExchangeAcqRelAcq64(&stack->a_head, &head, new_head))
        {
            slot.address = node;
            slot.index = (u32) head;
            break;
        }
    }
    return slot;
}

static void sb_cstack_push(struct sb_cstack *stack, const struct sb_central *c, const u32 index)
{
    u64 *link = (u64 *) (c->vm.buf + (u64) index*c->slot_size);
    u64 head = AtomicLoadRlx64(&stack->a_head);
    do
    {
        AtomicStoreRlx64(link, head);
    } while (!AtomicCompareExchangeRelRlx64(&stack->a_head, &head, (((head >> 32) + 1) << 32) | index));
}

static struct slot sb_central_carve(struct sb_central *c, const u32 count)
{
    struct slot slot;
    slot.index = AtomicFetchAddRlx32(&c->a_count_max, count);
    const u64 end = (u64) slot.index + count;
    if (end > c->length || !VMCommit(&c->vm, end*c->slot_size))
    {
        ds_TSlabOutOfMemoryInternal();
    }
    slot.address = c->vm.buf + slot.index*c->slot_size;
    return slot;
}

/* old ds_TSlab: per-thread free lists per count, the link in the freed array's first u32, a shared bump */
struct ds_Align(DS_CACHE_LINE) sb_old_thread
{
    u32 head[SB_COUNT_MAX];
    u8  pad[DS_CACHE_LINE - SB_COUNT_MAX*sizeof(u32)];
};

static struct sb_old_thread *g_old[2];

static struct slot sb_old_add(const u32 kind, const u32 count)
{
    struct sb_central *c = g_central + kind;
    struct sb_old_thread *t = g_old[kind] + ds_ThreadSelfIndex();
    struct slot slot;
    slot.index = t->head[count];
    if (slot.index != U32_MAX)
    {
        slot.address = c->vm.buf + (u64) slot.index*c->slot_size;
        t->head[count] = *(u32 *) slot.address;
        return slot;
    }
    return sb_central_carve(c, count);
}

static void sb_old_remove(const u32 kind, const u32 index, const u32 count)
{
    struct sb_central *c = g_central + kind;
    struct sb_old_thread *t = g_old[kind] + ds_ThreadSelfIndex();
    *(u32 *) (c->vm.buf + (u64) index*c->slot_size) = t->head[count];
    t->head[count] = index;
}

static struct slot sb_central_add(struct sb_central *c, const u32 count)
{
    const struct slot slot = sb_cstack_pop(c->free + count, c);
    if (slot.address)
    {
        return slot;
    }
    return sb_central_carve(c, count);
}

static void sb_central_remove(struct sb_central *c, const u32 index, const u32 count)
{
    sb_cstack_push(c->free + count, c, index);
}

static struct slot sb_steal_add(const u32 kind, const u32 count)
{
    const struct sb_central *c = g_central + kind;
    const u32 self = ds_ThreadSelfIndex();
    struct slot slot = sb_cstack_pop(g_steal[kind] + self*SB_COUNT_MAX + count, c);
    for (u32 i = 1; i < g_steal_threads && slot.address == NULL; ++i)
    {
        const u32 t = (self + i) % g_steal_threads;
        slot = sb_cstack_pop(g_steal[kind] + t*SB_COUNT_MAX + count, c);
    }

    if (slot.address == NULL)
    {
        slot = sb_central_carve(g_central + kind, count);
    }
    return slot;
}

static void sb_steal_remove(const u32 kind, const u32 index, const u32 count)
{
    sb_cstack_push(g_steal[kind] + ds_ThreadSelfIndex()*SB_COUNT_MAX + count, g_central + kind, index);
}

/* deque: one ds_WSDeque per thread and count (thread-major) over the same region; Remove pushes and publishes
 * on the calling thread's deque, Add pops its own bottom, then steals from every other thread's top, then carves */
static struct ds_WSDeque *  g_deque[2];

static struct slot sb_deque_add(const u32 kind, const u32 count)
{
    const struct sb_central *c = g_central + kind;
    const u32 self = ds_ThreadSelfIndex();
    ds_JobId id = ds_WSDequeTryPopBottom(g_deque[kind] + self*SB_COUNT_MAX + count);
    for (u32 i = 1; i < g_steal_threads && id >= DS_JOB_ID_NULL; ++i)
    {
        const u32 t = (self + i) % g_steal_threads;
        id = ds_WSDequeTrySteal(g_deque[kind] + t*SB_COUNT_MAX + count);
    }

    struct slot slot;
    if (id < DS_JOB_ID_NULL)
    {
        slot.index = ds_JobIdIndex(id);
        slot.address = c->vm.buf + (u64) slot.index*c->slot_size;
    }
    else
    {
        slot = sb_central_carve(g_central + kind, count);
    }
    return slot;
}

static void sb_deque_remove(const u32 kind, const u32 index, const u32 count)
{
    struct ds_WSDeque *deque = g_deque[kind] + ds_ThreadSelfIndex()*SB_COUNT_MAX + count;
    ds_WSDequePushBottom(deque, ds_JobIdInit(0, index));
    ds_WSDequePublish(deque);
}

/*
magazines (Bonwick): per thread and count a loaded and a previous list of at most SB_MAG_M arrays, linked through the
arrays' first u32; the depot per count is a Treiber stack of full lists (the magazine link in the head array's
second u32, the ABA counter in the depot head's upper 32 bits). A thread touches the depot once per SB_MAG_M
operations at most: Add with both lists empty pops a full magazine before carving, Remove with both lists full
pushes the loaded one.
*/
#define SB_MAG_M    32

struct sb_mag_list
{
    u32 head;
    u32 len;
};

struct ds_Align(DS_CACHE_LINE) sb_mag_thread
{
    struct sb_mag_list loaded[SB_COUNT_MAX];
    struct sb_mag_list prev[SB_COUNT_MAX];
};

static struct sb_mag_thread *   g_mag_thread[2];
static struct sb_cstack         g_depot[2][SB_COUNT_MAX];

static struct slot sb_mag_add(const u32 kind, const u32 count)
{
    const struct sb_central *c = g_central + kind;
    struct sb_mag_thread *t = g_mag_thread[kind] + ds_ThreadSelfIndex();
    struct sb_mag_list *loaded = t->loaded + count;
    if (loaded->len == 0)
    {
        if (t->prev[count].len > 0)
        {
            const struct sb_mag_list tmp = *loaded;
            *loaded = t->prev[count];
            t->prev[count] = tmp;
        }
        else
        {
            struct sb_cstack *depot = &g_depot[kind][count];
            u64 head = AtomicLoadAcq64(&depot->a_head);
            while ((u32) head != U32_MAX)
            {
                const u32 *mag = (const u32 *) (c->vm.buf + (u64) (u32) head*c->slot_size);
                const u64 new_head = (((head >> 32) + 1) << 32) | AtomicLoadRlx32(mag + 1);
                if (AtomicCompareExchangeAcqRelAcq64(&depot->a_head, &head, new_head))
                {
                    loaded->head = (u32) head;
                    loaded->len = SB_MAG_M;
                    break;
                }
            }
        }
    }

    struct slot slot;
    if (loaded->len == 0)
    {
        slot = sb_central_carve(g_central + kind, count);
    }
    else
    {
        slot.index = loaded->head;
        slot.address = c->vm.buf + (u64) slot.index*c->slot_size;
        loaded->head = *(const u32 *) slot.address;
        loaded->len -= 1;
    }
    return slot;
}

static void sb_mag_remove(const u32 kind, const u32 index, const u32 count)
{
    const struct sb_central *c = g_central + kind;
    struct sb_mag_thread *t = g_mag_thread[kind] + ds_ThreadSelfIndex();
    struct sb_mag_list *loaded = t->loaded + count;
    if (loaded->len == SB_MAG_M)
    {
        if (t->prev[count].len < SB_MAG_M)
        {
            const struct sb_mag_list tmp = *loaded;
            *loaded = t->prev[count];
            t->prev[count] = tmp;
        }
        else
        {
            struct sb_cstack *depot = &g_depot[kind][count];
            u32 *mag = (u32 *) (c->vm.buf + (u64) loaded->head*c->slot_size);
            u64 head = AtomicLoadRlx64(&depot->a_head);
            do
            {
                AtomicStoreRlx32(mag + 1, (u32) head);
            } while (!AtomicCompareExchangeRelRlx64(&depot->a_head, &head, (((head >> 32) + 1) << 32) | loaded->head));
            loaded->head = U32_MAX;
            loaded->len = 0;
        }
    }

    *(u32 *) (c->vm.buf + (u64) index*c->slot_size) = loaded->head;
    loaded->head = index;
    loaded->len += 1;
}

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
    const u64 size = (u64) count*sb_elem_size[kind];
    *index = DS_TSLAB_NULL;
    g_counter[ds_ThreadSelfIndex()].alloc += 1;
    if (variant == SB_SLAB)
    {
        struct ds_ArrayUntyped arr;
        if (kind == 0) { const ds_Array(sb_elem_m) a = ds_TSlabAdd(g_slab_m, count); arr = (struct ds_ArrayUntyped) { .buf = a.buf, .length = a.length, .base = a.base }; }
        else           { const ds_Array(sb_elem_c) a = ds_TSlabAdd(g_slab_c, count); arr = (struct ds_ArrayUntyped) { .buf = a.buf, .length = a.length, .base = a.base }; }
        *index = arr.base;
        return arr.buf;
    }
    else if (variant == SB_ALLOC)
    {
        return ds_Alloc(size, 8);
    }
    else if (variant == SB_CENTRAL)
    {
        const struct slot slot = sb_central_add(g_central + kind, count);
        *index = slot.index;
        return slot.address;
    }
    else if (variant == SB_STEAL)
    {
        const struct slot slot = sb_steal_add(kind, count);
        *index = slot.index;
        return slot.address;
    }
    else if (variant == SB_DEQUE)
    {
        const struct slot slot = sb_deque_add(kind, count);
        *index = slot.index;
        return slot.address;
    }
    else if (variant == SB_MAGAZINE)
    {
        const struct slot slot = sb_mag_add(kind, count);
        *index = slot.index;
        return slot.address;
    }
    else if (variant == SB_OLD)
    {
        const struct slot slot = sb_old_add(kind, count);
        *index = slot.index;
        return slot.address;
    }
    return NULL;
}

static void sb_array_free(const u32 variant, const u32 kind, u8 *ptr, const u32 index, const u32 count)
{
    g_counter[ds_ThreadSelfIndex()].free += 1;
    if (variant == SB_SLAB)
    {
        const u32 length = (u32) PowerOfTwoCeil(count);
        if (kind == 0) { const ds_Array(sb_elem_m) a = { .buf = NULL, .length = length, .base = index }; ds_TSlabRemove(g_slab_m, a); }
        else           { const ds_Array(sb_elem_c) a = { .buf = NULL, .length = length, .base = index }; ds_TSlabRemove(g_slab_c, a); }
    }
    else if (variant == SB_ALLOC)
    {
        ds_Free(ptr);
    }
    else if (variant == SB_CENTRAL)
    {
        sb_central_remove(g_central + kind, index, count);
    }
    else if (variant == SB_STEAL)
    {
        sb_steal_remove(kind, index, count);
    }
    else if (variant == SB_DEQUE)
    {
        sb_deque_remove(kind, index, count);
    }
    else if (variant == SB_MAGAZINE)
    {
        sb_mag_remove(kind, index, count);
    }
    else if (variant == SB_OLD)
    {
        sb_old_remove(kind, index, count);
    }
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
    else if (variant == SB_REALLOC)
    {
        p_new = ds_Realloc(p_old, (u64) old*size, (u64) count*size, 8);
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
            ds_TSlabAlloc(g_slab_m, SB_VM_SIZE, g_set_count);
            ds_TSlabAlloc(g_slab_c, SB_VM_SIZE, g_set_count);
        }
        else if (variant == SB_CENTRAL || variant == SB_STEAL || variant == SB_DEQUE || variant == SB_MAGAZINE
                 || variant == SB_OLD)
        {
            sb_central_alloc(g_central + 0, sb_elem_size[0]);
            sb_central_alloc(g_central + 1, sb_elem_size[1]);
            g_steal_threads = g_set_count;
            if (variant == SB_STEAL)
            {
                for (u32 kind = 0; kind < 2; ++kind)
                {
                    const u64 size = (u64) g_steal_threads*SB_COUNT_MAX*sizeof(struct sb_cstack);
                    g_steal[kind] = ds_Alloc(size, DS_CACHE_LINE);
                    for (u32 i = 0; i < g_steal_threads*SB_COUNT_MAX; ++i) { g_steal[kind][i].a_head = U32_MAX; }
                }
            }
            else if (variant == SB_OLD)
            {
                for (u32 kind = 0; kind < 2; ++kind)
                {
                    g_old[kind] = ds_Alloc(g_steal_threads*sizeof(struct sb_old_thread), DS_CACHE_LINE);
                    memset(g_old[kind], 0xff, g_steal_threads*sizeof(struct sb_old_thread));
                }
            }
            else if (variant == SB_MAGAZINE)
            {
                for (u32 kind = 0; kind < 2; ++kind)
                {
                    g_mag_thread[kind] = ds_Alloc(g_steal_threads*sizeof(struct sb_mag_thread), DS_CACHE_LINE);
                    for (u32 t = 0; t < g_steal_threads; ++t)
                    {
                        for (u32 count = 0; count < SB_COUNT_MAX; ++count)
                        {
                            g_mag_thread[kind][t].loaded[count] = (struct sb_mag_list) { U32_MAX, 0 };
                            g_mag_thread[kind][t].prev[count] = (struct sb_mag_list) { U32_MAX, 0 };
                        }
                    }
                    for (u32 count = 0; count < SB_COUNT_MAX; ++count) { g_depot[kind][count].a_head = U32_MAX; }
                }
            }
            else if (variant == SB_DEQUE)
            {
                for (u32 kind = 0; kind < 2; ++kind)
                {
                    const u64 size = (u64) g_steal_threads*SB_COUNT_MAX*sizeof(struct ds_WSDeque);
                    g_deque[kind] = ds_Alloc(size, DS_CACHE_LINE);
                    for (u32 t = 0; t < g_steal_threads; ++t)
                    {
                        for (u32 count = 0; count < SB_COUNT_MAX; ++count)
                        {
                            ds_WSDequeAlloc(g_deque[kind] + t*SB_COUNT_MAX + count, t, 64);
                        }
                    }
                }
            }
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
        else if (job->variant == SB_CENTRAL || job->variant == SB_STEAL || job->variant == SB_DEQUE
                 || job->variant == SB_MAGAZINE || job->variant == SB_OLD)
        {
            VMRelease(&g_central[0].vm);
            VMRelease(&g_central[1].vm);
            if (job->variant == SB_STEAL)
            {
                ds_Free(g_steal[0]);
                ds_Free(g_steal[1]);
                g_steal[0] = g_steal[1] = NULL;
            }
            else if (job->variant == SB_OLD)
            {
                ds_Free(g_old[0]);
                ds_Free(g_old[1]);
                g_old[0] = g_old[1] = NULL;
            }
            else if (job->variant == SB_MAGAZINE)
            {
                ds_Free(g_mag_thread[0]);
                ds_Free(g_mag_thread[1]);
                g_mag_thread[0] = g_mag_thread[1] = NULL;
            }
            else if (job->variant == SB_DEQUE)
            {
                for (u32 kind = 0; kind < 2; ++kind)
                {
                    for (u32 i = 0; i < g_steal_threads*SB_COUNT_MAX; ++i) { ds_WSDequeDealloc(g_deque[kind] + i); }
                    ds_Free(g_deque[kind]);
                    g_deque[kind] = NULL;
                }
            }
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
SB_INIT(old_fixed, SB_OLD, 0, 0)
SB_INIT(old_rotate, SB_OLD, 1, 0)
SB_INIT(old_fixed_pow2, SB_OLD, 0, 1)
SB_INIT(old_rotate_pow2, SB_OLD, 1, 1)
#if SB_EXTRA_VARIANTS
SB_INIT(central_fixed, SB_CENTRAL, 0, 0)
SB_INIT(central_rotate, SB_CENTRAL, 1, 0)
SB_INIT(steal_fixed, SB_STEAL, 0, 0)
SB_INIT(steal_rotate, SB_STEAL, 1, 0)
SB_INIT(deque_fixed, SB_DEQUE, 0, 0)
SB_INIT(deque_rotate, SB_DEQUE, 1, 0)
SB_INIT(magazine_fixed, SB_MAGAZINE, 0, 0)
SB_INIT(magazine_rotate, SB_MAGAZINE, 1, 0)
SB_INIT(alloc_fixed, SB_ALLOC, 0, 0)
SB_INIT(alloc_rotate, SB_ALLOC, 1, 0)
SB_INIT(alloc_fixed_pow2, SB_ALLOC, 0, 1)
SB_INIT(alloc_rotate_pow2, SB_ALLOC, 1, 1)
SB_INIT(realloc_fixed, SB_REALLOC, 0, 0)
SB_INIT(realloc_rotate, SB_REALLOC, 1, 0)
#endif

#define SB_ENTRY(name, id_str) { .id = id_str, .size = 1, .test = sb_test, .test_init = sb_init_##name, .test_reset = sb_reset, .test_free = sb_free }

static struct test_PerformanceParallel slab_parallel_test[] =
{
    SB_ENTRY(null_fixed, "MeshArraysNull"),
    SB_ENTRY(slab_fixed, "MeshArraysSlabFixed"),
    SB_ENTRY(slab_rotate, "MeshArraysSlabRotate"),
    SB_ENTRY(slab_fixed_pow2, "MeshArraysSlabFixedPow2"),
    SB_ENTRY(slab_rotate_pow2, "MeshArraysSlabRotatePow2"),
    SB_ENTRY(old_fixed, "MeshArraysOldSlabFixed"),
    SB_ENTRY(old_rotate, "MeshArraysOldSlabRotate"),
    SB_ENTRY(old_fixed_pow2, "MeshArraysOldSlabFixedPow2"),
    SB_ENTRY(old_rotate_pow2, "MeshArraysOldSlabRotatePow2"),
#if SB_EXTRA_VARIANTS
    SB_ENTRY(central_fixed, "MeshArraysCentralFixed"),
    SB_ENTRY(central_rotate, "MeshArraysCentralRotate"),
    SB_ENTRY(steal_fixed, "MeshArraysStealFixed"),
    SB_ENTRY(steal_rotate, "MeshArraysStealRotate"),
    SB_ENTRY(deque_fixed, "MeshArraysDequeFixed"),
    SB_ENTRY(deque_rotate, "MeshArraysDequeRotate"),
    SB_ENTRY(magazine_fixed, "MeshArraysMagazineFixed"),
    SB_ENTRY(magazine_rotate, "MeshArraysMagazineRotate"),
    SB_ENTRY(alloc_fixed, "MeshArraysAllocFixed"),
    SB_ENTRY(alloc_rotate, "MeshArraysAllocRotate"),
    SB_ENTRY(alloc_fixed_pow2, "MeshArraysAllocFixedPow2"),
    SB_ENTRY(alloc_rotate_pow2, "MeshArraysAllocRotatePow2"),
    SB_ENTRY(realloc_fixed, "MeshArraysReallocFixed"),
    SB_ENTRY(realloc_rotate, "MeshArraysReallocRotate"),
#endif
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

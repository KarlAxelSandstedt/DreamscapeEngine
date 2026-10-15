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

#include <string.h>

#include "ds_test.h"
#include "ds_serialize.h"
#include "ds_bitset.h"
#include "ds_hash_map.h"
#include "ds_list.h"
#include "queue.h"
#include "hierarchy_index.h"
#include "ds_string_database.h"

typedef struct test_SdbNode
{
    SDB_NODE;
    u32 value;
} test_SdbNode;
DEFINE_SDB(test_SdbNode);

typedef struct test_HINode
{
    HI_NODE;
    u32 value;
} test_HINode;
HI_DECLARE(test_HINode);
HI_DEFINE(test_HINode)

/* Every bit and the set-bit iterator of every block agree with the model. */
static u32 BitSetTestMatches(const struct ds_BitSet *set, const u8 *model)
{
    for (u64 k = 0; k < set->block_count; ++k)
    {
        u64 expected = 64*k;
        struct ds_BitBlock it = ds_BitBlockInit(set->bits[k], k);
        while (ds_BitBlockHasNext(&it))
        {
            const u64 peek = ds_BitBlockPeekNext(&it);
            const u64 bit = ds_BitBlockNext(&it);
            if (peek != bit || bit < expected || bit >= 64*(k+1) || !model[bit])
            {
                return 0;
            }
            for (; expected < bit; ++expected)
            {
                if (model[expected] || ds_BitSetGet(set, expected))
                {
                    return 0;
                }
            }
            if (!ds_BitSetGet(set, bit))
            {
                return 0;
            }
            expected = bit + 1;
        }

        for (; expected < 64*(k+1); ++expected)
        {
            if (model[expected] || ds_BitSetGet(set, expected))
            {
                return 0;
            }
        }
    }
    return 1;
}

/*
 * Arena (fixed) and heap (growable) sets: random sets against a model, the set-bit iterator, clear, growth
 * (old bits kept, new bits = clear_bit), serialize round trips, truncated streams.
 */
static struct test_Output BitSetTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const u64 page = g_mem_config->page_size;
    for (u32 heap = 0; heap <= 1; ++heap)
    {
        const u64 clear_bit = heap;
        struct ds_BitSet set = ds_BitSetAlloc((heap) ? NULL : env->mem_1, 1000, clear_bit, (heap) ? GROWABLE : NOT_GROWABLE);
        TEST_NOT_EQUAL(NULL, set.bits);
        TEST_TRUE(set.bit_count >= 1000);
        TEST_EQUAL(64*set.block_count, set.bit_count);

        /* covers the set after one growth by 100 bits (page-rounded) */

        u8 *model = ds_Alloc(set.bit_count + 200 + 8*page, 1);
        TEST_NOT_EQUAL(NULL, model);
        memset(model, (int) clear_bit, set.bit_count);
        TEST_TRUE(BitSetTestMatches(&set, model));

        for (u32 i = 0; i < 5000; ++i)
        {
            const u64 bit = RngU64Range(0, set.bit_count - 1);
            const u64 value = RngU64Range(0, 1);
            ds_BitSetSet(&set, bit, value);
            model[bit] = (u8) value;
        }
        TEST_TRUE(BitSetTestMatches(&set, model));

        if (heap)
        {
            const u64 old_bit_count = set.bit_count;
            const u64 new_bit = 1 - clear_bit;
            ds_BitSetIncreaseSize(&set, old_bit_count + 100, new_bit);
            TEST_TRUE(set.bit_count >= old_bit_count + 100);
            TEST_TRUE(set.bit_count <= old_bit_count + 200 + 8*page);
            memset(model + old_bit_count, (int) new_bit, set.bit_count - old_bit_count);
            TEST_TRUE(BitSetTestMatches(&set, model));
        }

        const u64 size = ds_BitSetSerializeSize(&set);
        struct ss ss = ss_Alloc(env->mem_1, size);
        TEST_NOT_EQUAL(NULL, ss.buf);
        ds_BitSetSerialize(&ss, &set);
        TEST_EQUAL(8*size, ss.bit_index);
        for (u32 read_heap = 0; read_heap <= 1; ++read_heap)
        {
            ss.bit_index = 0;
            struct ds_BitSet read;
            TEST_EQUAL(1, ds_BitSetTryDeserialize((read_heap) ? NULL : env->mem_2, &ss, &read, (read_heap) ? GROWABLE : NOT_GROWABLE));
            TEST_EQUAL(8*size, ss.bit_index);
            TEST_EQUAL(set.block_count, read.block_count);
            TEST_EQUAL(set.bit_count, read.bit_count);
            TEST_TRUE(BitSetTestMatches(&read, model));
            ds_BitSetDealloc(&read);
        }

        struct ss cut = ss_Buffered(ss.buf, size - 1);
        struct ds_BitSet read;
        TEST_EQUAL(0, ds_BitSetTryDeserialize(env->mem_2, &cut, &read, NOT_GROWABLE));
        TEST_EQUAL(0, cut.bit_index);
        TEST_EQUAL(NULL, read.bits);

        ds_BitSetClear(&set, 1);
        memset(model, 1, set.bit_count);
        TEST_TRUE(BitSetTestMatches(&set, model));

        ds_Free(model);
        ds_BitSetDealloc(&set);
    }

	return output;
}

#define HASH_MAP_TEST_MAX 4096

/* Every chain holds exactly the live indices of its bucket, each once. */
static u32 HashMapTestMatches(const struct ds_HashMap *map, const u8 *alive, const u32 *hash_of, const u32 alive_count, u8 *seen)
{
    memset(seen, 0, HASH_MAP_TEST_MAX);
    u32 found = 0;
    for (u32 b = 0; b < map->hash_len; ++b)
    {
        for (u32 i = ds_HashMapFirst(map, b); i != HASH_NULL; i = ds_HashMapNext(map, i))
        {
            if (i >= HASH_MAP_TEST_MAX || !alive[i] || seen[i] || (hash_of[i] & map->hash_mask) != b)
            {
                return 0;
            }
            seen[i] = 1;
            found += 1;
        }
    }
    return found == alive_count;
}

/*
 * Arena map (fixed) and heap map (growable past its index_len): random adds/removes with colliding hashes
 * against a model; a fixed map rejects indices >= index_len; flush; serialize round trips; corrupt streams.
 */
static struct test_Output HashMapTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u8 *alive = ArenaPush(env->mem_2, HASH_MAP_TEST_MAX);
    u8 *seen = ArenaPush(env->mem_2, HASH_MAP_TEST_MAX);
    u32 *hash_of = ArenaPush(env->mem_2, HASH_MAP_TEST_MAX*sizeof(u32));
    for (u32 growable = 0; growable <= 1; ++growable)
    {
        struct ds_HashMap map = (growable)
            ? ds_HashMapAlloc(NULL, 16, 16, GROWABLE)
            : ds_HashMapAlloc(env->mem_1, 64, 512, NOT_GROWABLE);
        TEST_NOT_EQUAL(NULL, map.hash);
        TEST_TRUE(PowerOfTwoCheck(map.hash_len) && map.hash_len <= map.index_len);
        const u32 index_max = (growable) ? HASH_MAP_TEST_MAX : map.index_len;
        TEST_TRUE(index_max <= HASH_MAP_TEST_MAX);

        if (!growable)
        {
            TEST_EQUAL(0, ds_HashMapAdd(&map, 7, map.index_len));
        }

        memset(alive, 0, HASH_MAP_TEST_MAX);
        u32 alive_count = 0;
        for (u32 i = 0; i < 3000; ++i)
        {
            const u64 op = RngU64Range(0, 2);
            if (op < 2 && alive_count < index_max / 2)
            {
                u32 index;
                do
                {
                    index = (u32) RngU64Range(0, index_max - 1);
                } while (alive[index]);
                /* few distinct hashes: long chains */
                const u32 hash = (u32) RngU64Range(0, 63) * 0x9e3779b1u;
                TEST_EQUAL(1, ds_HashMapAdd(&map, hash, index));
                alive[index] = 1;
                hash_of[index] = hash;
                alive_count += 1;
            }
            else if (alive_count)
            {
                u32 index;
                do
                {
                    index = (u32) RngU64Range(0, index_max - 1);
                } while (!alive[index]);
                ds_HashMapRemove(&map, hash_of[index], index);
                alive[index] = 0;
                alive_count -= 1;
            }

            if (i % 16 == 0)
            {
                TEST_TRUE(HashMapTestMatches(&map, alive, hash_of, alive_count, seen));
            }
        }
        TEST_TRUE(HashMapTestMatches(&map, alive, hash_of, alive_count, seen));
        if (growable)
        {
            TEST_TRUE(map.index_len >= HASH_MAP_TEST_MAX / 2);
        }

        const u64 size = ds_HashMapSerializeSize(&map);
        struct ss ss = ss_Alloc(env->mem_1, size);
        TEST_NOT_EQUAL(NULL, ss.buf);
        ds_HashMapSerialize(&ss, &map);
        TEST_EQUAL(8*size, ss.bit_index);
        for (u32 read_heap = 0; read_heap <= 1; ++read_heap)
        {
            ss.bit_index = 0;
            struct ds_HashMap read;
            TEST_EQUAL(1, ds_HashMapTryDeserialize((read_heap) ? NULL : env->mem_3, &ss, &read, (read_heap) ? GROWABLE : NOT_GROWABLE));
            TEST_EQUAL(8*size, ss.bit_index);
            TEST_EQUAL(map.hash_len, read.hash_len);
            TEST_EQUAL(map.index_len, read.index_len);
            TEST_TRUE(HashMapTestMatches(&read, alive, hash_of, alive_count, seen));
            ds_HashMapDealloc(&read);
        }

        /* truncated; hash_len not a power of two; hash_len > index_len */
        const u64 mem_left = env->mem_3->mem_left;
        struct ds_HashMap read;
        struct ss cut = ss_Buffered(ss.buf, size - 1);
        TEST_EQUAL(0, ds_HashMapTryDeserialize(env->mem_3, &cut, &read, NOT_GROWABLE));
        TEST_EQUAL(0, cut.bit_index);
        TEST_EQUAL(NULL, read.hash);

        const u32 corrupt_hash_len[] = { 0, map.hash_len + 1, 2*map.index_len };
        struct ss bad = ss_Alloc(env->mem_1, size);
        TEST_NOT_EQUAL(NULL, bad.buf);
        for (u32 c = 0; c < sizeof(corrupt_hash_len) / sizeof(corrupt_hash_len[0]); ++c)
        {
            memcpy(bad.buf, ss.buf, size);
            bad.bit_index = 0;
            ss_WriteU32Le(&bad, corrupt_hash_len[c]);
            bad.bit_index = 0;
            TEST_EQUAL(0, ds_HashMapTryDeserialize(env->mem_3, &bad, &read, NOT_GROWABLE));
            TEST_EQUAL(0, bad.bit_index);
            TEST_EQUAL(NULL, read.hash);
            TEST_EQUAL(mem_left, env->mem_3->mem_left);
        }

        ds_HashMapFlush(&map);
        memset(alive, 0, HASH_MAP_TEST_MAX);
        TEST_TRUE(HashMapTestMatches(&map, alive, hash_of, 0, seen));

        ds_HashMapDealloc(&map);
    }

	return output;
}

struct test_ListNode
{
    struct DLLNode  dll;
    LLNode          ll;
    u32             in_list;
};

#define LIST_TEST_MAX 256

/* Forward and backward walks of the list give the model's order. */
static u32 DLLTestMatches(const struct DLL *list, const struct test_ListNode *base, const u32 *order, const u32 count)
{
    if (list->count != count)
    {
        return 0;
    }

    u32 k = 0;
    for (i32 i = list->first; i != DLL_SENTINEL; i = base[i].dll.next)
    {
        if (k >= count || (u32) i != order[k++])
        {
            return 0;
        }
    }
    if (k != count)
    {
        return 0;
    }

    for (i32 i = list->last; i != DLL_SENTINEL; i = base[i].dll.prev)
    {
        if (k == 0 || (u32) i != order[--k])
        {
            return 0;
        }
    }
    return k == 0;
}

/* Random appends, prepends and removes against an order array; serialize round trip of the header. */
static struct test_Output DLLTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    /* index -1 is the stub */
    struct test_ListNode *base = (struct test_ListNode *) ArenaPushZero(env->mem_2, (LIST_TEST_MAX + 1)*sizeof(struct test_ListNode)) + 1;
    u32 *order = ArenaPush(env->mem_2, LIST_TEST_MAX*sizeof(u32));

    struct DLL list;
    DLLFlush(list);
    u32 count = 0;
    for (u32 i = 0; i < 4000; ++i)
    {
        const u64 op = RngU64Range(0, 2);
        if (op < 2 && count < LIST_TEST_MAX)
        {
            u32 index;
            do
            {
                index = (u32) RngU64Range(0, LIST_TEST_MAX - 1);
            } while (base[index].in_list);
            base[index].in_list = 1;

            if (op == 0)
            {
                DLLAppend(list, base, (i32) index, dll);
                order[count++] = index;
            }
            else
            {
                DLLPrepend(list, base, (i32) index, dll);
                memmove(order + 1, order, count*sizeof(u32));
                order[0] = index;
                count += 1;
            }
        }
        else if (count)
        {
            const u32 k = (u32) RngU64Range(0, count - 1);
            const u32 index = order[k];
            DLLRemove(list, base, (i32) index, dll);
            base[index].in_list = 0;
            memmove(order + k, order + k + 1, (count - k - 1)*sizeof(u32));
            count -= 1;
        }

        TEST_TRUE(DLLTestMatches(&list, base, order, count));
    }

    struct ss ss = ss_Alloc(env->mem_1, DLLSerializeSize(list));
    TEST_NOT_EQUAL(NULL, ss.buf);
    DLLSerialize(&ss, list);
    ss.bit_index = 0;
    struct DLL read;
    TEST_EQUAL(1, DLLTryDeserialize(&ss, read));
    TEST_TRUE(DLLTestMatches(&read, base, order, count));

    struct ss cut = ss_Buffered(ss.buf, DLLSerializeSize(list) - 1);
    TEST_EQUAL(0, DLLTryDeserialize(&cut, read));
    TEST_EQUAL(0, read.count);
    TEST_EQUAL(DLL_SENTINEL, read.first);
    TEST_EQUAL(DLL_SENTINEL, read.last);

	return output;
}

/* Appends and prepends give the model's order (forward walk); first/last/count; header round trip. */
static struct test_Output LLTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_ListNode *base = (struct test_ListNode *) ArenaPushZero(env->mem_2, (LIST_TEST_MAX + 1)*sizeof(struct test_ListNode)) + 1;
    /* middle[front, back) is the list: prepends grow it to the left, appends to the right */
    u32 *middle = ArenaPush(env->mem_2, 2*LIST_TEST_MAX*sizeof(u32));

    for (u32 round = 0; round < 20; ++round)
    {
        struct LL list;
        LLFlush(list);
        TEST_EQUAL(LL_SENTINEL, list.first);

        const u32 count = (u32) RngU64Range(0, LIST_TEST_MAX);
        u32 front = LIST_TEST_MAX;
        u32 back = LIST_TEST_MAX;
        for (u32 index = 0; index < count; ++index)
        {
            const u64 prepend = RngU64Range(0, 1);
            if (prepend)
            {
                LLPrepend(list, base, (i32) index, ll);
                middle[--front] = index;
            }
            else
            {
                LLAppend(list, base, (i32) index, ll);
                middle[back++] = index;
            }
        }

        TEST_EQUAL(count, list.count);
        u32 k = front;
        for (i32 i = list.first; i != LL_SENTINEL; i = base[i].ll)
        {
            TEST_TRUE(k < back);
            TEST_EQUAL((u32) i, middle[k]);
            k += 1;
        }
        TEST_EQUAL(back, k);
        if (count)
        {
            TEST_EQUAL((i32) middle[front], list.first);
            TEST_EQUAL((i32) middle[back - 1], list.last);
        }

        struct ss ss = ss_Alloc(env->mem_1, LLSerializeSize(list));
        LLSerialize(&ss, list);
        ss.bit_index = 0;
        struct LL read;
        TEST_EQUAL(1, LLTryDeserialize(&ss, read));
        TEST_EQUAL(list.count, read.count);
        TEST_EQUAL(list.first, read.first);
        TEST_EQUAL(list.last, read.last);
    }

	return output;
}

/*
 * Fixed arena queue and growable heap queue: random pushes and pops with tied priorities; every pop returns
 * a minimum of the model (peek agrees), and an (id, priority) pair that was pushed.
 */
static struct test_Output MinQueueFixedTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u32f32 *model = ArenaPush(env->mem_2, 1000*sizeof(u32f32));
    for (u32 growable = 0; growable <= 1; ++growable)
    {
        struct minQueueFixed queue = (growable)
            ? MinQueueFixedAlloc(NULL, 4, GROWABLE)
            : MinQueueFixedAlloc(env->mem_1, 1000, NOT_GROWABLE);
        TEST_NOT_EQUAL(NULL, queue.element);

        u32 count = 0;
        u32 next_id = 0;
        for (u32 i = 0; i < 6000; ++i)
        {
            const u64 op = RngU64Range(0, 2);
            if (op < 2 && count < 900)
            {
                const f32 priority = (f32) RngU64Range(0, 50);
                MinQueueFixedPush(&queue, next_id, priority);
                model[count].u = next_id;
                model[count].f = priority;
                count += 1;
                next_id += 1;
            }
            else if (count)
            {
                const u32f32 peek = MinQueueFixedPeek(&queue);
                const u32f32 pop = MinQueueFixedPop(&queue);
                TEST_EQUAL(peek.u, pop.u);
                u32 at = count;
                f32 min = F32_INFINITY;
                for (u32 k = 0; k < count; ++k)
                {
                    min = (model[k].f < min) ? model[k].f : min;
                    at = (model[k].u == pop.u) ? k : at;
                }
                TEST_TRUE(at < count);
                TEST_TRUE(model[at].f == pop.f && pop.f == min);
                model[at] = model[--count];
            }
            TEST_EQUAL(count, queue.count);
        }

        MinQueueFixedFlush(&queue);
        TEST_EQUAL(0, queue.count);
        MinQueueFixedDealloc(&queue);
    }

	return output;
}

#define MIN_QUEUE_TEST_MAX 4096

/*
 * Fixed heap queue kept within its initial length, and a growable heap queue that grows well past it:
 * random pushes, pops and priority decreases against a model by external index. A quarter of the
 * priorities are +inf (a pop must never drop a live element, springclean 12); decreases hit any live
 * object, also at object indices >= count after pops (springclean 11).
 */
static struct test_Output MinQueueTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u8 *alive = ArenaPush(env->mem_2, MIN_QUEUE_TEST_MAX);
    f32 *priority = ArenaPush(env->mem_2, MIN_QUEUE_TEST_MAX*sizeof(f32));
    u32 *object = ArenaPush(env->mem_2, MIN_QUEUE_TEST_MAX*sizeof(u32));

    for (u32 growable = 0; growable <= 1; ++growable)
    {
        struct minQueue queue = MinQueueAlloc(NULL, (growable) ? 4 : 64, growable);
        /* a growable queue reaches 1000 elements: past any page-rounded initial length */
        const u32 count_max = (growable) ? 1000 : 64;
        const u32 push_weight = (growable) ? 4 : 2;
        memset(alive, 0, MIN_QUEUE_TEST_MAX);
        u32 count = 0;
        u32 count_reached = 0;
        u32 next_external = 0;
        for (u32 i = 0; i < 4000 && next_external < MIN_QUEUE_TEST_MAX; ++i)
        {
            const u64 op = RngU64Range(0, push_weight + 2);
            if (op < push_weight && count < count_max)
            {
                const f32 p = (RngU64Range(0, 3) == 0) ? F32_INFINITY : (f32) RngU64Range(0, 100);
                const u32 e = next_external++;
                object[e] = MinQueuePush(&queue, p, e);
                priority[e] = p;
                alive[e] = 1;
                count += 1;
                count_reached = (count > count_reached) ? count : count_reached;
            }
            else if (op < push_weight + 2 && count)
            {
                f32 min = F32_INFINITY;
                for (u32 e = 0; e < next_external; ++e)
                {
                    min = (alive[e] && priority[e] < min) ? priority[e] : min;
                }
                TEST_TRUE(queue.elements[0].priority == min);
                const u32 e = MinQueuePop(&queue);
                TEST_TRUE(e < next_external && alive[e]);
                TEST_TRUE(priority[e] == min);
                alive[e] = 0;
                count -= 1;
            }
            else if (count)
            {
                u32 e;
                do
                {
                    e = (u32) RngU64Range(0, next_external - 1);
                } while (!alive[e]);

                const f32 p = (f32) RngU64Range(0, 100);
                MinQueueDecreasePriority(&queue, object[e], p);
                priority[e] = (p < priority[e]) ? p : priority[e];
            }
            TEST_EQUAL(count, queue.object_pool.count);
        }

        if (growable)
        {
            TEST_EQUAL(count_max, count_reached);
        }

        MinQueueFlush(&queue);
        TEST_EQUAL(0, queue.object_pool.count);
        MinQueueDealloc(&queue);
    }

	return output;
}

#define HI_TEST_MAX 64

/* Hierarchy model: parent[i] for every alive node (roots HI_ROOT and HI_ORPHAN have HI_NULL). */
struct hi_TestModel
{
    i32 parent[HI_TEST_MAX + 2];
    u8  alive[HI_TEST_MAX + 2];
    u32 alive_count;
};

static u32 HITestInSubtree(const struct hi_TestModel *model, i32 node, const i32 root)
{
    for (; node != HI_NULL; node = model->parent[node])
    {
        if (node == root) { return 1; }
    }
    return 0;
}

static i32 HITestRandomAlive(const struct hi_TestModel *model, const u32 min_index)
{
    for (;;)
    {
        const i32 i = (i32) RngU64Range(min_index, HI_TEST_MAX + 1);
        if (model->alive[i]) { return i; }
    }
}

/*
 * Every alive node's child list walks forward to hi_last in hi_child_count steps, each child links back
 * (hi_prev, hi_parent) and is the model's child; every non-root node is in exactly one list.
 */
static u32 HITestValid(const test_HINodeHI *hi, const struct hi_TestModel *model)
{
    u32 listed = 0;
    for (i32 p = 0; p < (i32) hi->pool.count_max; ++p)
    {
        if (!model->alive[p]) { continue; }

        const test_HINode *node = hi->pool.buf + p;
        if (p > HI_ORPHAN && node->hi_parent != model->parent[p]) { return 0; }

        u32 count = 0;
        i32 prev = HI_NULL;
        for (i32 c = node->hi_first; c != HI_NULL; c = hi->pool.buf[c].hi_next)
        {
            if (count == HI_TEST_MAX || c < 0 || c >= (i32) hi->pool.count_max || !model->alive[c]) { return 0; }
            if (hi->pool.buf[c].hi_parent != p || hi->pool.buf[c].hi_prev != prev || model->parent[c] != p) { return 0; }
            prev = c;
            count += 1;
        }
        if (node->hi_last != prev || node->hi_child_count != count) { return 0; }
        listed += count;
    }
    return listed + 2 == model->alive_count;
}

/*
 * Iterate root's sub-hierarchy (HIIInit/HIIAdvance until at == root), calling HIISkip at random non-root
 * nodes: every node of the sub-hierarchy without a skipped strict ancestor is visited once, nothing else.
 */
static u32 HITestIterate(const test_HINodeHI *hi, const struct hi_TestModel *model, const i32 root, const u32 skip)
{
    u8 visited[HI_TEST_MAX + 2] = { 0 };
    u8 skipped[HI_TEST_MAX + 2] = { 0 };
    u32 steps = 0;

    HII it;
    HIIInit(it, *hi, root);
    do
    {
        const i32 at = it.at;
        if (steps++ > HI_TEST_MAX + 2 || at < 0 || at >= HI_TEST_MAX + 2 || visited[at] || !HITestInSubtree(model, at, root))
        {
            return 0;
        }
        visited[at] = 1;
        if (skip && at != root && RngU64Range(0, 3) == 0)
        {
            skipped[at] = 1;
            HIISkip(it, *hi);
        }
        HIIAdvance(it, *hi);
    }
    while (it.at != root);

    for (i32 i = 0; i < HI_TEST_MAX + 2; ++i)
    {
        if (!model->alive[i] || !HITestInSubtree(model, i, root)) { continue; }
        u32 hidden = 0;
        /* strict ancestors below root */
        for (i32 a = model->parent[i]; i != root && a != root; a = model->parent[a])
        {
            hidden |= skipped[a];
        }
        if (visited[i] == hidden) { return 0; }
    }
    return 1;
}

static void HITestModelInit(struct hi_TestModel *model)
{
    memset(model, 0, sizeof(*model));
    model->parent[HI_ROOT] = HI_NULL;
    model->parent[HI_ORPHAN] = HI_NULL;
    model->alive[HI_ROOT] = 1;
    model->alive[HI_ORPHAN] = 1;
    model->alive_count = 2;
}

static i32 HITestAdd(test_HINodeHI *hi, struct hi_TestModel *model, const i32 parent)
{
    const struct slot slot = test_HINodeHIAdd(hi, (u32) parent);
    if (slot.address == NULL)
    {
        return HI_NULL;
    }
    model->parent[slot.index] = parent;
    model->alive[slot.index] = 1;
    model->alive_count += 1;
    return (i32) slot.index;
}

/* #35: a childless iteration root with a next sibling is iterated alone (HIIInit used to step to the sibling). */
static struct test_Output HierarchyIteratorSiblingRootTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct hi_TestModel model;
    HITestModelInit(&model);
    test_HINodeHI hi = test_HINodeHIAlloc(NULL, HI_TEST_MAX, NOT_GROWABLE);

    const i32 a = HITestAdd(&hi, &model, HI_ROOT);
    const i32 b = HITestAdd(&hi, &model, HI_ROOT);
    HITestAdd(&hi, &model, b);

    HII it;
    HIIInit(it, hi, a);
    TEST_EQUAL(a, it.at);
    TEST_EQUAL(a, it.next);
    HIIAdvance(it, hi);
    TEST_EQUAL(a, it.at);

    TEST_TRUE(HITestIterate(&hi, &model, a, 0));
    TEST_TRUE(HITestIterate(&hi, &model, b, 0));
    TEST_TRUE(HITestIterate(&hi, &model, HI_ROOT, 0));

    test_HINodeHIDealloc(&hi);
	return output;
}

/* #34: adopting a child that is not its parent's last leaves no link into its old siblings. */
static struct test_Output HierarchyAdoptMiddleChildTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct hi_TestModel model;
    HITestModelInit(&model);
    test_HINodeHI hi = test_HINodeHIAlloc(NULL, HI_TEST_MAX, NOT_GROWABLE);

    const i32 p = HITestAdd(&hi, &model, HI_ROOT);
    const i32 q = HITestAdd(&hi, &model, HI_ROOT);
    const i32 a = HITestAdd(&hi, &model, p);
    const i32 b = HITestAdd(&hi, &model, p);
    const i32 c = HITestAdd(&hi, &model, p);

    test_HINodeHIAdoptNode(&hi, (u32) a, (u32) q);
    model.parent[a] = q;
    TEST_TRUE(HITestValid(&hi, &model));
    TEST_EQUAL(HI_NULL, hi.pool.buf[a].hi_next);
    TEST_EQUAL(b, hi.pool.buf[p].hi_first);
    TEST_EQUAL(c, hi.pool.buf[p].hi_last);

    /* back into its own (old) parent while not its last child: used to link the sibling list into a cycle */
    test_HINodeHIAdoptNode(&hi, (u32) b, (u32) p);
    TEST_TRUE(HITestValid(&hi, &model));
    TEST_EQUAL(c, hi.pool.buf[p].hi_first);
    TEST_EQUAL(b, hi.pool.buf[p].hi_last);

    test_HINodeHIDealloc(&hi);
	return output;
}

/*
 * Random HIAdd / HIRemove / HIAdoptNode / HIAdoptNodeExclusive against a parent model; after every
 * operation the links are checked and a random sub-hierarchy is iterated (with random HIISkip).
 */
static struct test_Output HierarchyRandomTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct hi_TestModel model;
    HITestModelInit(&model);
    test_HINodeHI hi = test_HINodeHIAlloc(NULL, HI_TEST_MAX, NOT_GROWABLE);
    TEST_TRUE(hi.pool.buf != NULL);

    for (u32 op = 0; op < 20000; ++op)
    {
        const u64 kind = (model.alive_count == 2) ? 0 : RngU64Range(0, 9);
        if (kind <= 3)
        {
            HITestAdd(&hi, &model, HITestRandomAlive(&model, 0));
        }
        else if (kind == 4)
        {
            const i32 x = HITestRandomAlive(&model, HI_ORPHAN + 1);
            test_HINodeHIRemove(env->mem_1, &hi, (u32) x);
            for (i32 i = HI_ORPHAN + 1; i < HI_TEST_MAX + 2; ++i)
            {
                if (i != x && model.alive[i] && HITestInSubtree(&model, i, x))
                {
                    model.alive[i] = 0;
                    model.alive_count -= 1;
                }
            }
            model.alive[x] = 0;
            model.alive_count -= 1;
        }
        else if (kind <= 7)
        {
            const i32 x = HITestRandomAlive(&model, HI_ORPHAN + 1);
            i32 p;
            do { p = HITestRandomAlive(&model, 0); } while (HITestInSubtree(&model, p, x));
            test_HINodeHIAdoptNode(&hi, (u32) x, (u32) p);
            model.parent[x] = p;
        }
        else
        {
            const i32 x = HITestRandomAlive(&model, HI_ORPHAN + 1);
            i32 p;
            do { p = HITestRandomAlive(&model, 0); } while (p == x);
            test_HINodeHIAdoptNodeExclusive(&hi, (u32) x, (u32) p);
            for (i32 i = 0; i < HI_TEST_MAX + 2; ++i)
            {
                if (model.alive[i] && model.parent[i] == x)
                {
                    model.parent[i] = model.parent[x];
                }
            }
            model.parent[x] = p;
        }

        TEST_TRUE(HITestValid(&hi, &model));
        TEST_TRUE(HITestIterate(&hi, &model, HITestRandomAlive(&model, 0), (u32) (op & 1)));
    }

    test_HINodeHIDealloc(&hi);
	return output;
}

/*
 * String database (fixed size, on an arena): Add returns the new node's slot (it returned the failure slot);
 * empty, duplicate and over-capacity ids fail and leave the database as it was; aliased ids; Remove and
 * re-Add; the allocated list matches.
 */
static struct test_Output StringDatabaseTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    /* the stub and three nodes */
    test_SdbNodeSDB db;
    ds_SDBAlloc(env->mem_1, db, 4, NOT_GROWABLE);
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Inline("a")).index);

    const struct slot a = ds_SDBAdd(env->mem_1, db, Utf8Inline("a"));
    TEST_TRUE(a.address != NULL && a.index != U32_MAX && a.index != SDB_STUB);
    TEST_EQUAL(a.index, ds_SDBLookup(db, Utf8Inline("a")).index);
    TEST_TRUE(Utf8Equivalence(Utf8Inline("a"), db.pool.buf[a.index].id));
    TEST_EQUAL(0, db.pool.buf[a.index].reference_count);

    /* empty (the stub's) and duplicate ids fail */
    TEST_EQUAL(U32_MAX, ds_SDBAdd(env->mem_1, db, Utf8Empty()).index);
    TEST_EQUAL(U32_MAX, ds_SDBAdd(env->mem_1, db, Utf8Inline("a")).index);
    TEST_EQUAL(U32_MAX, ds_SDBAddAlias(db, Utf8Empty()).index);
    TEST_EQUAL(U32_MAX, ds_SDBAddAlias(db, Utf8Inline("a")).index);

    const struct slot b = ds_SDBAddAlias(db, Utf8Inline("b"));
    const struct slot c = ds_SDBAdd(env->mem_1, db, Utf8Inline("c"));
    TEST_TRUE(b.address != NULL && c.address != NULL);
    TEST_EQUAL(3, db.pool.count - 1);

    /* full: fails, and the database is unchanged */
    TEST_EQUAL(U32_MAX, ds_SDBAdd(env->mem_1, db, Utf8Inline("d")).index);
    TEST_EQUAL(U32_MAX, ds_SDBAddAlias(db, Utf8Inline("d")).index);
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Inline("d")).index);
    TEST_EQUAL(3, db.pool.count - 1);
    TEST_EQUAL(b.index, ds_SDBLookup(db, Utf8Inline("b")).index);
    TEST_EQUAL(c.index, ds_SDBLookup(db, Utf8Inline("c")).index);

    /* remove, then the freed slot takes a new id */
    ds_SDBRemove(db, Utf8Inline("b"));
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Inline("b")).index);
    TEST_EQUAL(2, db.pool.count - 1);
    const struct slot d = ds_SDBAdd(env->mem_1, db, Utf8Inline("d"));
    TEST_EQUAL(b.index, d.index);
    TEST_EQUAL(d.index, ds_SDBLookup(db, Utf8Inline("d")).index);

    /* removing the stub's id does nothing */
    ds_SDBRemove(db, Utf8Empty());
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Empty()).index);
    TEST_EQUAL(3, db.pool.count - 1);

	return output;
}

/* Reference / Dereference count found nodes only; the stub is a default callers reference freely. */
static struct test_Output StringDatabaseReferenceTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    test_SdbNodeSDB db;
    ds_SDBAlloc(NULL, db, 4, GROWABLE);
    const struct slot a = ds_SDBAdd(env->mem_1, db, Utf8Inline("a"));

    TEST_EQUAL(a.index, ds_SDBReference(db, Utf8Inline("a")).index);
    TEST_EQUAL(a.index, ds_SDBReference(db, Utf8Inline("a")).index);
    TEST_EQUAL(2, db.pool.buf[a.index].reference_count);
    ds_SDBDereference(db, a.index);
    TEST_EQUAL(1, db.pool.buf[a.index].reference_count);

    /* the stub: an unknown id and the empty id */
    TEST_EQUAL(SDB_STUB, ds_SDBReference(db, Utf8Inline("missing")).index);
    TEST_EQUAL(SDB_STUB, ds_SDBReference(db, Utf8Empty()).index);
    TEST_EQUAL(0, db.pool.buf[SDB_STUB].reference_count);
    ds_SDBDereference(db, SDB_STUB);
    ds_SDBDereference(db, SDB_STUB);
    TEST_EQUAL(0, db.pool.buf[SDB_STUB].reference_count);

    /* growable: many adds stay findable */
    for (u32 i = 0; i < 100; ++i)
    {
        TEST_TRUE(ds_SDBAdd(env->mem_1, db, Utf8Format(env->mem_1, "id_%u", i)).address != NULL);
    }
    for (u32 i = 0; i < 100; ++i)
    {
        const struct slot found = ds_SDBLookup(db, Utf8Format(env->mem_1, "id_%u", i));
        TEST_TRUE(found.index != SDB_STUB);
        TEST_EQUAL(Utf8Hash(db.pool.buf[found.index].id), db.pool.buf[found.index].id_hash);
    }

    /* flush: only a fresh stub is left, and ids can be added again */
    ds_SDBFlush(db);
    TEST_EQUAL(1, db.pool.count);
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Inline("a")).index);
    TEST_EQUAL(SDB_STUB, ds_SDBLookup(db, Utf8Inline("id_7")).index);
    TEST_EQUAL(0, db.pool.buf[SDB_STUB].id.len);
    TEST_TRUE(ds_SDBAdd(env->mem_1, db, Utf8Inline("a")).index != U32_MAX);

    ds_SDBDealloc(db);
	return output;
}

static struct test_Output (*container_tests[])(struct test_Environment *) =
{
    StringDatabaseTest,
    StringDatabaseReferenceTest,
    HierarchyIteratorSiblingRootTest,
    HierarchyAdoptMiddleChildTest,
    HierarchyRandomTest,
    BitSetTest,
    HashMapTest,
    DLLTest,
    LLTest,
    MinQueueFixedTest,
    MinQueueTest,
};

struct suite_Correctness m_container_suite =
{
	.id = "container",
	.unit_test = container_tests,
	.unit_test_count = sizeof(container_tests) / sizeof(container_tests[0]),
};

struct suite_Correctness *container_correctness_suite = &m_container_suite;

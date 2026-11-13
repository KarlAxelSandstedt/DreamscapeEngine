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
#include "ds_init.h"

/* Edges, and random n in (2^(k-1), 2^k] for every k. */
static struct test_Output PowerOfTwoTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    TEST_EQUAL(0, PowerOfTwoCheck(0));
    TEST_EQUAL(1, PowerOfTwoCheck(1));
    TEST_EQUAL(0, PowerOfTwoCheck(3));
    TEST_EQUAL(1, PowerOfTwoCheck((u64) 1 << 63));
    TEST_EQUAL(0, PowerOfTwoCheck(U64_MAX));
    TEST_EQUAL(1, PowerOfTwoCeil(0));
    TEST_EQUAL(1, PowerOfTwoCeil(1));
    TEST_EQUAL((u64) 1 << 63, PowerOfTwoCeil(((u64) 1 << 62) + 1));
    TEST_EQUAL((u64) 1 << 63, PowerOfTwoCeil((u64) 1 << 63));

    for (u32 k = 1; k < 64; ++k)
    {
        const u64 p = (u64) 1 << k;
        const u64 n = RngU64Range(p/2 + 1, p);
        TEST_EQUAL(p, PowerOfTwoCeil(n));
        TEST_EQUAL((u32) (n == p), PowerOfTwoCheck(n));
    }

	return output;
}

/* 
 * ds_Alloc/ds_Realloc/ds_Free: every alignment 1..4096 (both realloc paths: up to max_align_t, and above),
 * growing and shrinking keep the content and the alignment, size 0 and NULL.
 */
static struct test_Output HeapAllocTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    TEST_EQUAL(NULL, ds_Alloc(0, 8));
    ds_Free(NULL);
    TEST_EQUAL(NULL, ds_Realloc(NULL, 0, 0, 8));

    u8 *fresh = ds_Realloc(NULL, 0, 100, 64);
    TEST_NOT_EQUAL(NULL, fresh);
    TEST_EQUAL(0, (u64) fresh % 64);
    memset(fresh, 1, 100);
    ds_Free(fresh);

    for (u64 alignment = 1; alignment <= 4096; alignment <<= 1)
    {
        u64 size = 1 + RngU64Range(0, 100);
        u8 *addr = ds_Alloc(size, alignment);
        TEST_NOT_EQUAL(NULL, addr);
        TEST_TRUE(PowerOfTwoMultipleCheck((u64) addr, alignment));
        for (u64 i = 0; i < size; ++i)
        {
            addr[i] = (u8) (7*i + 1);
        }

        /* grow, grow, shrink, ... */
        for (u32 step = 0; step < 9; ++step)
        {
            const u64 new_size = (step % 3 == 2) ? size/2 + 1 : 3*size + 17;
            addr = ds_Realloc(addr, size, new_size, alignment);
            TEST_NOT_EQUAL(NULL, addr);
            TEST_TRUE(PowerOfTwoMultipleCheck((u64) addr, alignment));

            const u64 kept = (size < new_size) ? size : new_size;
            u32 same = 1;
            for (u64 i = 0; i < kept; ++i)
            {
                same &= (addr[i] == (u8) (7*i + 1));
            }
            TEST_TRUE(same);

            for (u64 i = kept; i < new_size; ++i)
            {
                addr[i] = (u8) (7*i + 1);
            }
            size = new_size;
        }

        TEST_EQUAL(NULL, ds_Realloc(addr, size, 0, alignment));
    }

	return output;
}

/* Random aligned pushes until full: exact padding, address and mem_left; a failed push changes nothing. */
static struct test_Output ArenaPushTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct arena ar = ArenaAlloc(env->mem_1, 4096);
    TEST_NOT_EQUAL(NULL, ar.stack_ptr);
    TEST_TRUE(PowerOfTwoMultipleCheck((u64) ar.stack_ptr, DS_CACHE_LINE));
    TEST_EQUAL(4096, ar.mem_left);
    const u8 *base = ar.stack_ptr;

    TEST_EQUAL(NULL, ArenaPush(&ar, 0));
    TEST_EQUAL(4096, ar.mem_left);

    for (u32 round = 0; round < 50; ++round)
    {
        ArenaFlush(&ar);
        TEST_EQUAL(base, ar.stack_ptr);
        TEST_EQUAL(4096, ar.mem_left);
        for (;;)
        {
            const u64 alignment = (u64) 1 << RngU64Range(0, 7);
            const u64 size = RngU64Range(1, 300);
            u8 *top = ar.stack_ptr;
            const u64 left = ar.mem_left;
            const u64 pad = PowerOfTwoMultipleCeil((u64) top, alignment) - (u64) top;
            u8 *addr = ArenaPushAligned(&ar, size, alignment);
            if (pad + size > left)
            {
                TEST_EQUAL(NULL, addr);
                TEST_EQUAL(top, ar.stack_ptr);
                TEST_EQUAL(left, ar.mem_left);
                break;
            }

            TEST_EQUAL(top + pad, addr);
            TEST_EQUAL(addr + size, ar.stack_ptr);
            TEST_EQUAL(left - pad - size, ar.mem_left);
            memset(addr, 0xab, size);
        }
    }

    /* zeroed and copied pushes over old content */
    ArenaFlush(&ar);
    u8 *old = ArenaPush(&ar, 256);
    memset(old, 0xab, 256);
    ArenaPopPacked(&ar, 256);
    const u8 *zero = ArenaPushZero(&ar, 256);
    TEST_EQUAL(old, zero);
    u32 all_zero = 1;
    for (u32 i = 0; i < 256; ++i)
    {
        all_zero &= (zero[i] == 0);
    }
    TEST_TRUE(all_zero);

    u8 src[100];
    for (u32 i = 0; i < sizeof(src); ++i)
    {
        src[i] = (u8) (3*i + 1);
    }
    const u8 *copy = ArenaPushMemcpy(&ar, src, sizeof(src));
    TEST_NOT_EQUAL(NULL, copy);
    TEST_EQUAL(0, memcmp(copy, src, sizeof(src)));

    /* a sub-arena that doesn't fit is empty and leaves its parent unchanged */
    const u64 parent_left = env->mem_1->mem_left;
    struct arena too_big = ArenaAlloc(env->mem_1, parent_left + 1);
    TEST_EQUAL(NULL, too_big.stack_ptr);
    TEST_EQUAL(0, too_big.mem_size);
    TEST_EQUAL(0, too_big.mem_left);
    TEST_EQUAL(parent_left, env->mem_1->mem_left);

	return output;
}

/* Nested records pop in LIFO order; ArenaRemoveRecord keeps memory; flush drops all records. */
static struct test_Output ArenaRecordTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct arena ar = ArenaAlloc(env->mem_1, 4096);
    TEST_NOT_EQUAL(NULL, ar.stack_ptr);

    ArenaPopRecord(&ar);
    TEST_EQUAL(4096, ar.mem_left);

    ArenaPush(&ar, 10);
    const u64 left_0 = ar.mem_left;
    ArenaPushRecord(&ar);
    ArenaPush(&ar, 20);
    const u64 left_1 = ar.mem_left;
    ArenaPushRecord(&ar);
    ArenaPush(&ar, 30);
    ArenaPushRecord(&ar);
    ArenaPush(&ar, 40);
    const u64 left_2 = ar.mem_left;
    ArenaRemoveRecord(&ar);
    TEST_EQUAL(left_2, ar.mem_left);
    ArenaPopRecord(&ar);
    TEST_EQUAL(left_1, ar.mem_left);
    ArenaPopRecord(&ar);
    TEST_EQUAL(left_0, ar.mem_left);
    TEST_EQUAL(NULL, ar.record);
    ArenaPopRecord(&ar);
    TEST_EQUAL(left_0, ar.mem_left);

    ArenaPushRecord(&ar);
    ArenaFlush(&ar);
    TEST_EQUAL(NULL, ar.record);
    TEST_EQUAL(4096, ar.mem_left);

    /* random pushes, records and pops against a stack of expected positions */
    u64 expected[32];
    u32 depth = 0;
    for (u32 i = 0; i < 4000; ++i)
    {
        const u64 op = RngU64Range(0, 2);
        if (op == 0 && depth < 32 && ar.mem_left >= 2*sizeof(struct arenaRecord))
        {
            expected[depth++] = ar.mem_left;
            ArenaPushRecord(&ar);
            TEST_TRUE(ar.mem_left < expected[depth-1]);
        }
        else if (op == 1 && depth)
        {
            ArenaPopRecord(&ar);
            TEST_EQUAL(expected[--depth], ar.mem_left);
        }
        else
        {
            const u64 size = RngU64Range(1, 64);
            ArenaPush(&ar, size);
        }
    }

	return output;
}

/* ArenaPushAlignedAll takes every whole slot after the padding; popping memPushed restores the arena. */
static struct test_Output ArenaPushAllTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct arena ar = ArenaAlloc(env->mem_1, 1000);
    TEST_NOT_EQUAL(NULL, ar.stack_ptr);
    ArenaPushPacked(&ar, 3);

    u8 *top = ar.stack_ptr;
    const u64 left = ar.mem_left;
    const u64 pad = (16 - ((u64) top & 15)) & 15;
    struct memArray array = ArenaPushAlignedAll(&ar, 24, 16);
    TEST_EQUAL((void *) (top + pad), array.addr);
    TEST_EQUAL((left - pad) / 24, array.len);
    TEST_EQUAL(pad + 24*array.len, array.memPushed);
    TEST_TRUE(ar.mem_left < 24);
    memset(array.addr, 0, 24*array.len);

    ArenaPopPacked(&ar, array.memPushed);
    TEST_EQUAL(left, ar.mem_left);
    TEST_EQUAL(top, ar.stack_ptr);

    struct memArray none = ArenaPushAlignedAll(&ar, left + 1, 1);
    TEST_EQUAL(NULL, none.addr);
    TEST_EQUAL(0, none.len);
    TEST_EQUAL(0, none.memPushed);
    TEST_EQUAL(left, ar.mem_left);

	return output;
}

static u32 CPoolTestMatches(const ds_CPool(u32) *pool, const u32 *model, const u32 count)
{
    if (pool->count != count)
    {
        return 0;
    }

    for (u32 i = 0; i < count; ++i)
    {
        if (pool->buf[i] != model[i])
        {
            return 0;
        }
    }
    return 1;
}

#define CPOOL_TEST_MAX 512

/*
 * Random push/pop/pop-batch/remove-and-swap/flush against an array model, for a fixed arena pool and a
 * growable heap pool starting empty. Pop batches get an expression as count (macro arguments).
 */
static struct test_Output CPoolTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u32 model[CPOOL_TEST_MAX];
    for (u32 growable = 0; growable <= 1; ++growable)
    {
        ds_CPool(u32) pool;
        const u32 length = (growable) ? 0 : 300;
        if (growable)
        {
            ds_CPoolAlloc(NULL, pool, length, GROWABLE);
        }
        else
        {
            ds_CPoolAlloc(env->mem_1, pool, length, NOT_GROWABLE);
        }
        TEST_EQUAL(length, pool.length);

        u32 count = 0;
        for (u32 i = 0; i < 5000; ++i)
        {
            const u64 op = RngU64Range(0, 19);
            if (op < 10)
            {
                if (count == CPOOL_TEST_MAX)
                {
                    continue;
                }

                const u32 value = (u32) RngU64();
                if (!growable && count == length)
                {
                    const struct slot slot = ds_CPoolPush(pool);
                    TEST_EQUAL(U32_MAX, slot.index);
                    /* writes to a full pool land in the stub */
                    ds_CPoolPushValue(pool, value);
                }
                else if (op < 5)
                {
                    const struct slot slot = ds_CPoolPush(pool);
                    TEST_EQUAL(count, slot.index);
                    TEST_EQUAL((void *) (pool.buf + count), slot.address);
                    *(u32 *) slot.address = value;
                    model[count++] = value;
                }
                else
                {
                    ds_CPoolPushValue(pool, value);
                    model[count++] = value;
                }
            }
            else if (op < 14 && count)
            {
                const u32 index = (u32) RngU64Range(0, count - 1);
                ds_CPoolRemoveAndSwap(pool, index);
                count -= 1;
                model[index] = model[count];
            }
            else if (op < 16 && count)
            {
                const u32 keep = (u32) RngU64Range(0, count);
                ds_CPoolPopBatch(pool, pool.count - keep);
                count = keep;
            }
            else if (op < 19 && count)
            {
                ds_CPoolPop(pool);
                count -= 1;
            }
            else if (op == 19)
            {
                ds_CPoolFlush(pool);
                count = 0;
            }

            TEST_TRUE(CPoolTestMatches(&pool, model, count));
            if (count)
            {
                TEST_EQUAL(model[count-1], ds_CPoolTop(pool));
                TEST_EQUAL(pool.buf + count - 1, ds_CPoolTopAddress(pool));
            }
        }

        ds_CPoolDealloc(pool);
        TEST_EQUAL(NULL, pool.buf);
        TEST_EQUAL(0, pool.count);
    }

    /* length 0, not growable (arena and heap): no block, not even the stub; the arena is untouched */
    for (u32 heap = 0; heap <= 1; ++heap)
    {
        const u64 mem_left = env->mem_1->mem_left;
        ds_CPool(u32) empty;
        ds_CPoolAlloc((heap) ? NULL : env->mem_1, empty, 0, NOT_GROWABLE);
        TEST_EQUAL(NULL, empty.buf);
        TEST_EQUAL(0, empty.length);
        TEST_EQUAL(0, empty.count);
        TEST_EQUAL(mem_left, env->mem_1->mem_left);
        ds_CPoolDealloc(empty);
    }

	return output;
}

/* Round trip into an arena (fixed) and the heap (growable); truncated or inconsistent streams fail cleanly. */
static struct test_Output CPoolSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    ds_CPool(u64) pool;
    ds_CPoolAlloc(NULL, pool, 0, GROWABLE);
    const u32 count = (u32) RngU64Range(1, 200);
    for (u32 i = 0; i < count; ++i)
    {
        const u64 value = RngU64();
        ds_CPoolPushValue(pool, value);
    }

    const u64 size = ds_CPoolSerializeSize(pool);
    struct SS ss = SSAlloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, ss.buf);
    ds_CPoolSerialize(&ss, pool);
    TEST_EQUAL(8*size, ss.bit_index);

    for (u32 growable = 0; growable <= 1; ++growable)
    {
        ss.bit_index = 0;
        ds_CPool(u64) read;
        struct arena *mem = (growable) ? NULL : env->mem_2;
        TEST_EQUAL(1, ds_CPoolTryDeserialize(mem, &ss, read, growable));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(pool.length, read.length);
        TEST_EQUAL(pool.count, read.count);
        TEST_EQUAL(growable, read.growable);
        TEST_EQUAL(0, memcmp(pool.buf, read.buf, count*sizeof(u64)));
        if (growable)
        {
            for (u32 i = 0; i < pool.length + 1; ++i)
            {
                ds_CPoolPushValue(read, (u64) i);
            }
            TEST_EQUAL(0, memcmp(pool.buf, read.buf, count*sizeof(u64)));
            TEST_EQUAL(count + pool.length + 1, read.count);
        }
        ds_CPoolDealloc(read);
    }

    const u64 mem_left = env->mem_2->mem_left;
    ds_CPool(u64) read;

    struct SS cut = SSBuffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_CPoolTryDeserialize(env->mem_2, &cut, read, NOT_GROWABLE));
    TEST_EQUAL(0, cut.bit_index);
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(0, read.count);
    TEST_EQUAL(mem_left, env->mem_2->mem_left);

    /* count > length */
    struct SS bad = SSAlloc(env->mem_1, 2*sizeof(u32) + 2*sizeof(u64));
    SSWriteU32Le(&bad, 1);
    SSWriteU32Le(&bad, 2);
    bad.bit_index = 0;
    TEST_EQUAL(0, ds_CPoolTryDeserialize(env->mem_2, &bad, read, NOT_GROWABLE));
    TEST_EQUAL(0, bad.bit_index);
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(mem_left, env->mem_2->mem_left);

    /* an empty length-0 pool round-trips unallocated */
    ds_CPool(u64) empty;
    ds_CPoolAlloc(env->mem_2, empty, 0, NOT_GROWABLE);
    struct SS empty_ss = SSAlloc(env->mem_1, ds_CPoolSerializeSize(empty));
    ds_CPoolSerialize(&empty_ss, empty);
    empty_ss.bit_index = 0;
    const u64 mem_left_empty = env->mem_2->mem_left;
    TEST_EQUAL(1, ds_CPoolTryDeserialize(env->mem_2, &empty_ss, read, NOT_GROWABLE));
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(0, read.length);
    TEST_EQUAL(0, read.count);
    TEST_EQUAL(mem_left_empty, env->mem_2->mem_left);
    TEST_EQUAL(8*ds_CPoolSerializeSize(empty), empty_ss.bit_index);

    ds_CPoolDealloc(pool);

	return output;
}

struct test_PoolNode
{
    u64 value;
    POOL_NODE;
    u32 pad;
};

DEFINE_POOL(test_PoolNode);

#define POOL_TEST_MAX 1024

/* Pool state matches the model: count, allocation bits of [0, count_max), values of allocated slots. */
static u32 PoolTestMatches(const ds_Pool(test_PoolNode) *pool, const u8 *alive, const u64 *value, const u32 count)
{
    if (pool->count != count || pool->count_max > POOL_TEST_MAX)
    {
        return 0;
    }

    for (u32 i = 0; i < pool->count_max; ++i)
    {
        const struct test_PoolNode *node = pool->buf + i;
        if ((u32) alive[i] != (u32) ds_PoolSlotAllocated(node) || (alive[i] && node->value != value[i]))
        {
            return 0;
        }
    }
    return 1;
}

/* Random adds and removes until full (fixed pools) or up to 400 slots (growable pool). */
static u32 PoolTestRandomOps(ds_Pool(test_PoolNode) *pool, u8 *alive, u64 *value, u32 *count, const u32 op_count)
{
    for (u32 i = 0; i < op_count; ++i)
    {
        const u64 op = RngU64Range(0, 2);
        if (op < 2)
        {
            if (*count == 400)
            {
                continue;
            }

            const u32 full = (pool->count == pool->length && !pool->growable);
            const struct slot slot = ds_PoolAdd(*pool);
            if (full)
            {
                if (slot.address != NULL || slot.index != POOL_NULL)
                {
                    return 0;
                }
                continue;
            }

            if (slot.address != (void *) (pool->buf + slot.index) || slot.index >= POOL_TEST_MAX || alive[slot.index])
            {
                return 0;
            }

            const u64 v = RngU64();
            ((struct test_PoolNode *) slot.address)->value = v;
            alive[slot.index] = 1;
            value[slot.index] = v;
            *count += 1;
        }
        else if (*count)
        {
            u32 index;
            do
            {
                index = (u32) RngU64Range(0, pool->count_max - 1);
            } while (!alive[index]);

            const u64 by_address = RngU64Range(0, 1);
            if (by_address)
            {
                ds_PoolRemoveAddress(*pool, pool->buf + index);
            }
            else
            {
                ds_PoolRemove(*pool, index);
            }
            alive[index] = 0;
            *count -= 1;
        }

        if (!PoolTestMatches(pool, alive, value, *count))
        {
            return 0;
        }
    }

    return 1;
}

/* Arena pool, fixed heap pool and growable heap pool against a model; flush restarts at index 0. */
static struct test_Output PoolTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u8 *alive = ArenaPush(env->mem_2, POOL_TEST_MAX);
    u64 *value = ArenaPush(env->mem_2, POOL_TEST_MAX*sizeof(u64));
    for (u32 variant = 0; variant < 3; ++variant)
    {
        ds_Pool(test_PoolNode) pool;
        const u32 allocated = (variant == 0)
            ? ds_PoolAlloc(env->mem_1, pool, 100, NOT_GROWABLE)
            : ds_PoolAlloc(NULL, pool, (variant == 1) ? 100 : 1, (variant == 2) ? GROWABLE : NOT_GROWABLE);
        TEST_EQUAL(1, allocated);
        TEST_NOT_EQUAL(NULL, pool.buf);
        TEST_TRUE((variant == 0) ? pool.length == 100 : pool.length >= ((variant == 1) ? 100u : 1u));

        memset(alive, 0, POOL_TEST_MAX);
        u32 count = 0;
        TEST_TRUE(PoolTestRandomOps(&pool, alive, value, &count, 4000));

        ds_PoolFlush(pool);
        TEST_EQUAL(0, pool.count);
        TEST_EQUAL(0, pool.count_max);
        TEST_EQUAL(0, ds_PoolAdd(pool).index);

        ds_PoolDealloc(pool);
        TEST_EQUAL(NULL, pool.buf);
        TEST_EQUAL(0, pool.length);
        TEST_EQUAL(0, pool.count_max);
    }

	return output;
}

/*
 * Round trip into an arena (fixed) and the heap (growable): slots, values and free-list order survive (the
 * next adds return the same slots). Truncated streams, bad headers, out-of-range and repeated indices fail and leave the
 * stream position and the arena unchanged.
 */
static struct test_Output PoolSerializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u8 *alive = ArenaPushZero(env->mem_2, POOL_TEST_MAX);
    u64 *value = ArenaPush(env->mem_2, POOL_TEST_MAX*sizeof(u64));
    u32 *free_order = ArenaPush(env->mem_2, POOL_TEST_MAX*sizeof(u32));

    ds_Pool(test_PoolNode) pool;
    ds_PoolAlloc(NULL, pool, 64, GROWABLE);
    u32 count = 0;
    TEST_TRUE(PoolTestRandomOps(&pool, alive, value, &count, 600));
    /* at least two free and two allocated slots (600 ops at 2/3 adds leave hundreds allocated) */
    for (u32 k = 0; k < 2; ++k)
    {
        u32 index = 0;
        while (!alive[index]) { index += 1; }
        ds_PoolRemove(pool, index);
        alive[index] = 0;
        count -= 1;
    }
    TEST_TRUE(count >= 2);
    TEST_TRUE(PoolTestMatches(&pool, alive, value, count));

    u32 free_count = 0;
    for (u32 i = pool.next_free; i != POOL_INDEX_MASK; i = pool.buf[i].pool_slot & POOL_INDEX_MASK)
    {
        free_order[free_count++] = i;
    }
    TEST_EQUAL(pool.count_max - pool.count, free_count);

    const u64 size = ds_PoolSerializeSize(pool);
    struct SS ss = SSAlloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, ss.buf);
    ds_PoolSerialize(&ss, pool);
    TEST_EQUAL(8*size, ss.bit_index);

    for (u32 heap = 0; heap <= 1; ++heap)
    {
        ss.bit_index = 0;
        ds_Pool(test_PoolNode) read;
        TEST_EQUAL(1, ds_PoolTryDeserialize((heap) ? NULL : env->mem_2, &ss, read, (heap) ? GROWABLE : NOT_GROWABLE));
        TEST_EQUAL(8*size, ss.bit_index);
        TEST_EQUAL(pool.count, read.count);
        TEST_EQUAL(pool.count_max, read.count_max);
        TEST_EQUAL(pool.next_free, read.next_free);
        TEST_TRUE(read.length >= pool.length);
        TEST_TRUE(PoolTestMatches(&read, alive, value, count));
        for (u32 i = 0; i < free_count; ++i)
        {
            TEST_EQUAL(free_order[i], ds_PoolAdd(read).index);
        }
        ds_PoolDealloc(read);
    }

    const u64 mem_left = env->mem_2->mem_left;
    ds_Pool(test_PoolNode) read;

    struct SS cut = SSBuffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_PoolTryDeserialize(env->mem_2, &cut, read, NOT_GROWABLE));
    TEST_EQUAL(0, cut.bit_index);
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(0, read.count);
    TEST_EQUAL(mem_left, env->mem_2->mem_left);

    /* byte offsets of the stream's words: header, then free indices, then slots with their index in pool_slot */
    const u64 pool_slot_offset = (u64) &(((struct test_PoolNode *) 0)->pool_slot);
    const u64 slot_0_index = 12 + 4*(u64) free_count + pool_slot_offset;
    const u64 slot_1_index = slot_0_index + sizeof(struct test_PoolNode);
    const u8 *slot_0_bytes = ss.buf + slot_0_index;
    const u32 slot_0_value = (u32) slot_0_bytes[0] | (u32) slot_0_bytes[1] << 8 | (u32) slot_0_bytes[2] << 16 | (u32) slot_0_bytes[3] << 24;
    const u64 corrupt_offset[] =
    {
        0,                  /* length = POOL_INDEX_MASK                     */
        0,                  /* length < count_max                           */
        4,                  /* count > count_max                            */
        12,                 /* free index == count_max                      */
        slot_0_index,       /* allocated index == count_max                 */
        16,                 /* second free index == first free index        */
        slot_0_index,       /* allocated index == a free index              */
        slot_1_index,       /* second allocated index == first allocated    */
    };
    const u32 corrupt_value[] =
    {
        POOL_INDEX_MASK,
        pool.count_max - 1,
        pool.count_max + 1,
        pool.count_max,
        pool.count_max,
        free_order[0],
        free_order[0],
        slot_0_value,
    };

    struct SS bad = SSAlloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, bad.buf);
    for (u32 i = 0; i < sizeof(corrupt_offset) / sizeof(corrupt_offset[0]); ++i)
    {
        for (u32 heap = 0; heap <= 1; ++heap)
        {
            memcpy(bad.buf, ss.buf, size);
            bad.bit_index = 8*corrupt_offset[i];
            SSWriteU32Le(&bad, corrupt_value[i]);
            bad.bit_index = 0;

            TEST_EQUAL(0, ds_PoolTryDeserialize((heap) ? NULL : env->mem_2, &bad, read, (heap) ? GROWABLE : NOT_GROWABLE));
            TEST_EQUAL(0, bad.bit_index);
            TEST_EQUAL(NULL, read.buf);
            TEST_EQUAL(0, read.heap_allocated);
            TEST_EQUAL(0, read.count);
            TEST_EQUAL(mem_left, env->mem_2->mem_left);
        }
    }

    ds_PoolDealloc(pool);

	return output;
}

#if 0 /* TPool: retired, rebuilt on magazines and TStack */
struct test_TNode
{
    TPOOL_NODE;
    u32 value;
};

DEFINE_TPOOL(test_TNode);

/*
 * Single thread: increments hand out 0, 1, 2, ... across doubling blocks at distinct addresses; removed slots
 * come back first (last removed first); flush restarts at index 0.
 */
static struct test_Output TPoolTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    ds_TPool(test_TNode) pool;
    ds_TPoolAlloc(pool, g_arch_config->logical_core_count, 3);
    TEST_EQUAL(4, pool.state.initial_length);

    /* blocks 4, 4, 8, ..., 512 hold 1024 slots */
    const u32 n = 1000;
    for (u32 i = 0; i < n; ++i)
    {
        const struct slot slot = ds_TPoolIncrement(pool);
        TEST_EQUAL(i, slot.index);
        TEST_EQUAL(slot.address, (void *) ds_TPoolAddress(pool, i));
        ((struct test_TNode *) slot.address)->value = i;
    }
    TEST_EQUAL(9, pool.state.block_count);

    u32 distinct = 1;
    for (u32 i = 0; i < n; ++i)
    {
        distinct &= (ds_TPoolAddress(pool, i)->value == i);
    }
    TEST_TRUE(distinct);

    ds_TPoolRemove(pool, 10);
    ds_TPoolRemove(pool, 500);
    TEST_EQUAL(500, ds_TPoolAdd(pool).index);
    TEST_EQUAL(10, ds_TPoolAdd(pool).index);
    TEST_EQUAL(n, ds_TPoolAdd(pool).index);

    ds_TPoolFlush(pool);
    TEST_EQUAL(0, ds_TPoolIncrement(pool).index);

    ds_TPoolDealloc(pool);

	return output;
}

struct test_PNode
{
    TPOOL_NODE;
    u32 owner;
    u32 index;
};

DEFINE_TPOOL(test_PNode);

#define TPOOL_PARALLEL_HELD     2048
#define TPOOL_PARALLEL_ROUNDS   64
#define TPOOL_PARALLEL_OWNERS   (1u << 20)

struct test_TPoolParallel
{
    ds_TPool(test_PNode)    pool;
    u32 *                   a_owner;    /* a_owner[index]: id of the job holding slot index, 0 if free */
    u32                     a_next_id;
    u32                     a_errors;
};

/* Release a held slot: it must still be ours, untouched; then to our free list or another thread's. */
static void TPoolParallelRelease(struct test_TPoolParallel *t, struct rng *rng, const u32 id, const u32 index)
{
    const struct test_PNode *node = ds_TPoolAddress(t->pool, index);
    u32 expected = id;
    if (node->owner != id || node->index != index || !AtomicCompareExchangeAcqRlx32(t->a_owner + index, &expected, 0))
    {
        AtomicAddFetchRlx32(&t->a_errors, 1);
        return;
    }

    if (RngLocalU64Range(rng, 0, 1))
    {
        ds_TPoolRemove(t->pool, index);
    }
    else
    {
        const u32 list = (u32) RngLocalU64Range(rng, 0, t->pool.state.free_list_count - 1);
        ds_TPoolFreeListPush(t->pool, list, index);
    }
}

/* One job: rounds of filling up to TPOOL_PARALLEL_HELD slots and releasing a random half; then release all. */
static void TPoolParallelJob(void *args)
{
    struct test_TPoolParallel *t = args;
    const u32 id = AtomicAddFetchRlx32(&t->a_next_id, 1);
    /* a job's draws depend on the seed and the job, not on the thread that happens to run it */
    struct rng rng;
    RngLocalInit(&rng, g_config->rng_seed[0] ^ ((u64) id << 32));
    u32 held[TPOOL_PARALLEL_HELD];
    u32 held_count = 0;

    for (u32 round = 0; round < TPOOL_PARALLEL_ROUNDS; ++round)
    {
        while (held_count < TPOOL_PARALLEL_HELD)
        {
            const struct slot slot = ds_TPoolAdd(t->pool);
            u32 expected = 0;
            if (slot.index >= TPOOL_PARALLEL_OWNERS
                || slot.address != (void *) ds_TPoolAddress(t->pool, slot.index)
                || !AtomicCompareExchangeAcqRlx32(t->a_owner + slot.index, &expected, id))
            {
                /* handed out twice, or out of the owner table: count it and stop this job */
                AtomicAddFetchRlx32(&t->a_errors, 1);
                return;
            }

            struct test_PNode *node = slot.address;
            node->owner = id;
            node->index = slot.index;
            held[held_count++] = slot.index;
        }

        for (u32 k = 0; k < TPOOL_PARALLEL_HELD / 2; ++k)
        {
            const u32 j = (u32) RngLocalU64Range(&rng, 0, held_count - 1);
            TPoolParallelRelease(t, &rng, id, held[j]);
            held[j] = held[--held_count];
        }
    }

    while (held_count)
    {
        TPoolParallelRelease(t, &rng, id, held[--held_count]);
    }
}

/*
Every scheduler worker at once: rounds of adds and releases (half onto the own free list, half onto random
other threads' lists, so slots move between threads and get stolen). Every slot handed out is claimed in an
owner table with a compare-exchange (fails if two jobs hold it) and keeps its owner and index while held; at
the end, every slot is free. Starts with 64 slots, so blocks are added while the jobs run.
*/
static struct test_Output TPoolParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_TPoolParallel t;
    ds_TPoolAlloc(t.pool, g_arch_config->logical_core_count, 64);
    t.a_owner = ds_Alloc(TPOOL_PARALLEL_OWNERS*sizeof(u32), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t.a_owner);
    memset(t.a_owner, 0, TPOOL_PARALLEL_OWNERS*sizeof(u32));
    AtomicStoreRel32(&t.a_next_id, 0);
    AtomicStoreRel32(&t.a_errors, 0);

    ds_TestRunParallel(TPoolParallelJob, &t);

    TEST_EQUAL(0, AtomicLoadAcq32(&t.a_errors));
    TEST_EQUAL(g_scheduler->worker_count, (u32) AtomicLoadAcq32(&t.a_next_id));
    TEST_TRUE((u32) AtomicLoadAcq32(&t.pool.state.a_count_max) <= TPOOL_PARALLEL_OWNERS);
    TEST_TRUE(t.pool.state.block_count > 1);
    u32 all_free = 1;
    for (u32 i = 0; i < TPOOL_PARALLEL_OWNERS; ++i)
    {
        all_free &= (t.a_owner[i] == 0);
    }
    TEST_TRUE(all_free);

    ds_Free(t.a_owner);
    ds_TPoolDealloc(t.pool);

	return output;
}

#define TSTACK_PARALLEL_NODES   16
#define TSTACK_PARALLEL_OPS     200000

/* One job: pop a slot from free list 0, claim it, release it, push it back; again and again. */
static void TStackParallelJob(void *args)
{
    struct test_TPoolParallel *t = args;
    const u32 id = AtomicAddFetchRlx32(&t->a_next_id, 1);
    for (u32 i = 0; i < TSTACK_PARALLEL_OPS; ++i)
    {
        const struct slot slot = ds_TPoolFreeListPop(t->pool, 0);
        if (!slot.address)
        {
            continue;
        }

        u32 expected = 0;
        if (slot.index >= TSTACK_PARALLEL_NODES || !AtomicCompareExchangeAcqRlx32(t->a_owner + slot.index, &expected, id))
        {
            AtomicAddFetchRlx32(&t->a_errors, 1);
            return;
        }

        struct test_PNode *node = slot.address;
        node->owner = id;
        expected = id;
        if (node->owner != id || !AtomicCompareExchangeAcqRlx32(t->a_owner + slot.index, &expected, 0))
        {
            AtomicAddFetchRlx32(&t->a_errors, 1);
            return;
        }
        ds_TPoolFreeListPush(t->pool, 0, slot.index);
    }
}

/*
Every scheduler worker hammers one free list holding TSTACK_PARALLEL_NODES slots (pop, claim, release,
push): few nodes and one contended head is where ABA corrupts a Treiber stack without the push tags. A slot
popped by two jobs at once fails its claim; at the end the list holds exactly the slots it started with.
*/
static struct test_Output TStackParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_TPoolParallel t;
    ds_TPoolAlloc(t.pool, g_arch_config->logical_core_count, TSTACK_PARALLEL_NODES);
    t.a_owner = ds_Alloc(TSTACK_PARALLEL_NODES*sizeof(u32), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t.a_owner);
    memset(t.a_owner, 0, TSTACK_PARALLEL_NODES*sizeof(u32));
    AtomicStoreRel32(&t.a_next_id, 0);
    AtomicStoreRel32(&t.a_errors, 0);
    for (u32 i = 0; i < TSTACK_PARALLEL_NODES; ++i)
    {
        ds_TPoolFreeListPush(t.pool, 0, ds_TPoolIncrement(t.pool).index);
    }

    ds_TestRunParallel(TStackParallelJob, &t);
    TEST_EQUAL(0, AtomicLoadAcq32(&t.a_errors));

    /* drain: exactly the starting slots, each once (bounded: a corrupted list may cycle) */
    u8 seen[TSTACK_PARALLEL_NODES] = { 0 };
    u32 popped = 0;
    for (u32 i = 0; i <= TSTACK_PARALLEL_NODES; ++i)
    {
        const struct slot slot = ds_TPoolFreeListPop(t.pool, 0);
        if (!slot.address)
        {
            break;
        }
        TEST_TRUE(slot.index < TSTACK_PARALLEL_NODES && !seen[slot.index]);
        seen[slot.index] = 1;
        popped += 1;
    }
    TEST_EQUAL(TSTACK_PARALLEL_NODES, popped);

    ds_Free(t.a_owner);
    ds_TPoolDealloc(t.pool);

	return output;
}

#endif

struct test_TSNode
{
    TREIBER_NODE;
    u32 owner;
};

/* TStack, one thread: LIFO order, empty pops, the debug mark after a pop, flush */
static struct test_Output TStackTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    struct ds_TStack stack;
    struct test_TSNode node[8];
    for (u32 i = 0; i < 8; ++i) { node[i].treiber_next = TSTACK_NULL; }
    ds_TStackFlush(&stack);

    u32 index = 0;
    ds_TStackTryPop(index, &stack, node);
    TEST_EQUAL(TSTACK_NULL, index);

    for (u32 i = 0; i < 8; ++i)
    {
        ds_TStackPush(&stack, node, i);
    }
    TEST_EQUAL(7, ds_TStackKeyIndex(stack.a_head));
    TEST_EQUAL(8, ds_TStackKeyTag(stack.a_head));
    for (u32 i = 8; i-- > 0;)
    {
        ds_TStackTryPop(index, &stack, node);
        TEST_EQUAL(i, index);
    }
    ds_TStackTryPop(index, &stack, node);
    TEST_EQUAL(TSTACK_NULL, index);
    /* a pop of an empty stack does not touch the head */
    TEST_EQUAL(16, ds_TStackKeyTag(stack.a_head));

    /* pushes in another order come back reversed; flush empties */
    ds_TStackPush(&stack, node, 3);
    ds_TStackPush(&stack, node, 5);
    ds_TStackPush(&stack, node, 1);
    ds_TStackTryPop(index, &stack, node);
    TEST_EQUAL(1, index);
    ds_TStackTryPop(index, &stack, node);
    TEST_EQUAL(5, index);
    ds_TStackFlush(&stack);
    ds_TStackTryPop(index, &stack, node);
    TEST_EQUAL(TSTACK_NULL, index);
    (void) env;
	return output;
}

#define TSTACK_PARALLEL_NODES   16
#ifndef TSTACK_PARALLEL_OPS
#define TSTACK_PARALLEL_OPS     200000
#endif

struct test_TStackParallel
{
    struct ds_TStack       stack;
    struct test_TSNode  node[TSTACK_PARALLEL_NODES];
    u32 *               a_owner;
    u32                 a_next_id;
    u32                 a_errors;
};

/* One job: pop a node, claim it, release it, push it back; again and again. */
static void TStackParallelJob(void *args)
{
    struct test_TStackParallel *t = args;
    const u32 id = AtomicAddFetchRlx32(&t->a_next_id, 1);
    for (u32 i = 0; i < TSTACK_PARALLEL_OPS; ++i)
    {
        u32 index;
        ds_TStackTryPop(index, &t->stack, t->node);
        if (index == TSTACK_NULL)
        {
            continue;
        }

        u32 expected = 0;
        if (index >= TSTACK_PARALLEL_NODES || !AtomicCompareExchangeAcqRlx32(t->a_owner + index, &expected, id))
        {
            AtomicAddFetchRlx32(&t->a_errors, 1);
            return;
        }

        t->node[index].owner = id;
        expected = id;
        if (t->node[index].owner != id || !AtomicCompareExchangeAcqRlx32(t->a_owner + index, &expected, 0))
        {
            AtomicAddFetchRlx32(&t->a_errors, 1);
            return;
        }
        ds_TStackPush(&t->stack, t->node, index);
    }
}

/*
Every scheduler worker hammers one stack holding TSTACK_PARALLEL_NODES nodes (pop, claim, release, push):
few nodes and one contended head is where ABA corrupts a Treiber stack without the tag. A node popped by
two jobs at once fails its claim; at the end the stack holds exactly the nodes it started with.
*/
static struct test_Output TStackParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_TStackParallel *t = ds_Alloc(sizeof(*t), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t);
    t->a_owner = ds_Alloc(TSTACK_PARALLEL_NODES*sizeof(u32), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t->a_owner);
    memset(t->a_owner, 0, TSTACK_PARALLEL_NODES*sizeof(u32));
    AtomicStoreRel32(&t->a_next_id, 0);
    AtomicStoreRel32(&t->a_errors, 0);
    ds_TStackFlush(&t->stack);
    for (u32 i = 0; i < TSTACK_PARALLEL_NODES; ++i)
    {
        t->node[i].treiber_next = TSTACK_NULL;
        ds_TStackPush(&t->stack, t->node, i);
    }

    ds_TestRunParallel(TStackParallelJob, t);
    TEST_EQUAL(0, AtomicLoadAcq32(&t->a_errors));

    /* drain: exactly the starting nodes, each once (bounded: a corrupted stack may cycle) */
    u8 seen[TSTACK_PARALLEL_NODES] = { 0 };
    u32 popped = 0;
    for (u32 i = 0; i <= TSTACK_PARALLEL_NODES; ++i)
    {
        u32 index;
        ds_TStackTryPop(index, &t->stack, t->node);
        if (index == TSTACK_NULL)
        {
            break;
        }
        TEST_TRUE(index < TSTACK_PARALLEL_NODES && !seen[index]);
        seen[index] = 1;
        popped += 1;
    }
    TEST_EQUAL(TSTACK_PARALLEL_NODES, popped);

    ds_Free(t->a_owner);
    ds_Free(t);
    (void) env;
	return output;
}

#define RING_TEST_SLOTS 64

/* the byte k of the allocation with the given id */
static u8 RingTestByte(const u32 id, const u64 k)
{
    return (u8) (id * 31 + k * 7 + 1);
}

/*
 * Ring: the second half mirrors the first in both directions (not on web); pushes that run past the end are
 * contiguous, their bytes showing at the start where mirrored; failed pushes leave the ring unchanged; a random FIFO
 * (RingPushEnd / RingPopStart) against a queue model, every byte checked through the slot (and through its
 * first-half address where mirrored); LIFO RingPopEnd; RingDealloc leaves the ring empty.
 */
static struct test_Output RingTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ring ring = RingAlloc(1);
    TEST_TRUE(ring.buf != NULL && ring.mem_total >= 1 && ring.mem_left == ring.mem_total && ring.offset == 0);
    const u64 total = ring.mem_total;

    for (u64 k = 0; k < total; ++k) { ring.buf[k] = RingTestByte(1, k); }
    for (u64 k = 0; k < total; ++k) { TEST_EQUAL(RingTestByte(1, k), ring.buf[total + k]); }
    for (u64 k = 0; k < total; ++k) { ring.buf[total + k] = RingTestByte(2, k); }
    for (u64 k = 0; k < total; ++k) { TEST_EQUAL(RingTestByte(2, k), ring.buf[k]); }

    /* failures leave the ring as it was */
    struct ringSlot slot = RingPushEnd(&ring, 0);
    TEST_TRUE(slot.address == NULL && slot.size == 0);
    slot = RingPushEnd(&ring, total + 1);
    TEST_TRUE(slot.address == NULL && slot.size == 0 && ring.mem_left == total && ring.offset == 0);
    slot = RingPushStart(&ring, total + 1);
    TEST_TRUE(slot.address == NULL && ring.mem_left == total && ring.offset == 0);

    /* a push across the end: contiguous, its tail at the start */
    slot = RingPushEnd(&ring, total - 16);
    TEST_TRUE(slot.address == ring.buf && ring.offset == total - 16);
    RingPopStart(&ring, total - 16);
    slot = RingPushEnd(&ring, 64);
    TEST_TRUE(slot.address == ring.buf + total - 16 && slot.size == 64 && ring.offset == 48);
    for (u64 k = 0; k < 64; ++k) { slot.address[k] = RingTestByte(3, k); }
    for (u64 k = 16; k < 64; ++k) { TEST_EQUAL(RingTestByte(3, k), ring.buf[k - 16]); }

    /* a push before the start wraps back over the end */
    RingFlush(&ring);
    slot = RingPushEnd(&ring, 32);
    TEST_TRUE(slot.address == ring.buf);
    slot = RingPushStart(&ring, 48);
    TEST_TRUE(slot.address == ring.buf + total - 48 && ring.mem_left == total - 80 && ring.offset == 32);

    /* LIFO */
    RingPopEnd(&ring, 32);
    TEST_TRUE(ring.offset == 0 && ring.mem_left == total - 48);
    RingFlush(&ring);
    TEST_TRUE(ring.offset == 0 && ring.mem_left == total);

    /* FIFO against a queue model */
    struct rng rng;
    RngLocalInit(&rng, env->seed);
    u8 *address[RING_TEST_SLOTS];
    u64 size[RING_TEST_SLOTS];
    u32 id[RING_TEST_SLOTS];
    u32 first = 0;
    u32 count = 0;
    u64 used = 0;
    for (u32 n = 0; n < 20000; ++n)
    {
        const u64 push_size = RngLocalU64Range(&rng, 1, total / 4);
        const u32 push = (count < RING_TEST_SLOTS) && (count == 0 || RngLocalU64Range(&rng, 0, 1));
        if (push)
        {
            slot = RingPushEnd(&ring, push_size);
            if (used + push_size > total)
            {
                TEST_TRUE(slot.address == NULL && ring.mem_left == total - used);
                continue;
            }
            TEST_TRUE(slot.address != NULL && slot.size == push_size);
            TEST_TRUE(ring.buf <= slot.address && slot.address < ring.buf + total);
            const u32 i = (first + count) % RING_TEST_SLOTS;
            address[i] = slot.address;
            size[i] = push_size;
            id[i] = n;
            for (u64 k = 0; k < push_size; ++k) { slot.address[k] = RingTestByte(n, k); }
            count += 1;
            used += push_size;
        }
        else
        {
            const u32 i = first;
            const u64 at = (u64) (address[i] - ring.buf);
            for (u64 k = 0; k < size[i]; ++k)
            {
                TEST_EQUAL(RingTestByte(id[i], k), address[i][k]);
                TEST_EQUAL(RingTestByte(id[i], k), ring.buf[(at + k) % total]);
            }
            RingPopStart(&ring, size[i]);
            first = (first + 1) % RING_TEST_SLOTS;
            count -= 1;
            used -= size[i];
        }
        TEST_EQUAL(total - used, ring.mem_left);
    }

    RingDealloc(&ring);
    TEST_TRUE(ring.buf == NULL && ring.mem_total == 0 && ring.mem_left == 0);

	return output;
}

/* ds_CPoolPushBatch: consecutive slots, growth over several doublings, a fixed pool's refusal leaves it unchanged */
static struct test_Output CPoolPushBatchTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    ds_CPool(u32) pool;
    ds_CPoolAlloc(NULL, pool, 0, GROWABLE);
    struct slot batch = ds_CPoolPushBatch(pool, 100);
    TEST_EQUAL(0, batch.index);
    TEST_EQUAL(100, pool.count);
    TEST_TRUE(pool.length >= 100);
    TEST_EQUAL(pool.buf, (u32 *) batch.address);
    for (u32 i = 0; i < 100; ++i) { pool.buf[i] = i; }

    batch = ds_CPoolPushBatch(pool, 1000);
    TEST_EQUAL(100, batch.index);
    TEST_EQUAL(1100, pool.count);
    TEST_EQUAL(pool.buf + 100, (u32 *) batch.address);
    for (u32 i = 0; i < 1000; ++i) { ((u32 *) batch.address)[i] = 100 + i; }
    for (u32 i = 0; i < 1100; ++i) { TEST_EQUAL(i, pool.buf[i]); }
    ds_CPoolDealloc(pool);

    ds_CPool(u32) fixed;
    ds_CPoolAlloc(env->mem_1, fixed, 8, NOT_GROWABLE);
    batch = ds_CPoolPushBatch(fixed, 5);
    TEST_EQUAL(0, batch.index);
    TEST_EQUAL(5, fixed.count);
    batch = ds_CPoolPushBatch(fixed, 4);
    TEST_EQUAL(U32_MAX, batch.index);
    TEST_EQUAL(5, fixed.count);
    batch = ds_CPoolPushBatch(fixed, 3);
    TEST_EQUAL(5, batch.index);
    TEST_EQUAL(8, fixed.count);

	return output;
}

struct slab_item { u64 a; u32 b; };
DEFINE_ARRAY_STRUCT(slab_item);
DEFINE_TSLAB(slab_item);

/* ds_TSlab: alloc (region, magazines; reserved or on an arena), flush, dealloc; the failure path leaves an empty slab */
static struct test_Output SlabAllocTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ds_TSlab(slab_item) slab;
    const u32 length = (u32) (((u64) 16 << 20) / sizeof(struct slab_item));
    TEST_EQUAL(1, ds_TSlabAlloc(NULL, slab, length, 3));
    TEST_NOT_EQUAL(NULL, slab.buf);
    TEST_EQUAL((void *) slab.shared.vm.buf, (void *) slab.buf);
    TEST_EQUAL(length, slab.shared.length);
    TEST_EQUAL(1, slab.shared.heap_allocated);
    TEST_EQUAL(0, slab.shared.a_count_max);
    TEST_EQUAL(3, slab.shared.magazine.thread_count);
    TEST_EQUAL(slab.shared.length, slab.shared.magazine.index_length);
    /* 16-byte items: 1024 per magazine, clamped to 64 down to class 8, then halving to 4 */
    TEST_EQUAL(64, slab.shared.magazine.mclass_length[0]);
    TEST_EQUAL(64, slab.shared.magazine.mclass_length[4]);
    TEST_EQUAL(32, slab.shared.magazine.mclass_length[5]);
    TEST_EQUAL(4, slab.shared.magazine.mclass_length[8]);
    TEST_EQUAL(4, slab.shared.magazine.mclass_length[30]);
    for (u32 c = 0; c < 31; ++c)
    {
        TEST_EQUAL(TSTACK_NULL, ds_TStackKeyIndex(slab.shared.magazine.mclass[c].stack_full.a_head));
        TEST_EQUAL(TSTACK_NULL, ds_TStackKeyIndex(slab.shared.magazine.mclass[c].stack_empty.a_head));
    }

    /* flush resets the carve count and the magazines */
    slab.shared.a_count_max = 7;
    slab.shared.magazine.thread[1].magazine[3].count[0] = 2;
    slab.shared.magazine.a_index_count = 5;
    ds_TSlabFlush(slab);
    TEST_EQUAL(0, slab.shared.a_count_max);
    TEST_EQUAL(0, slab.shared.magazine.thread[1].magazine[3].count[0]);
    TEST_EQUAL(0, slab.shared.magazine.a_index_count);

    ds_TSlabDealloc(slab);
    TEST_EQUAL(NULL, slab.buf);
    TEST_EQUAL(NULL, slab.shared.vm.buf);
    TEST_EQUAL(NULL, slab.shared.magazine.thread);
    TEST_EQUAL(0, slab.shared.magazine.thread_count);
    TEST_EQUAL(0, slab.shared.length);

    /* 0 slots fail and leave the slab empty */
    TEST_EQUAL(0, ds_TSlabAlloc(NULL, slab, 0, 1));
    TEST_EQUAL(NULL, slab.buf);
    TEST_EQUAL(NULL, slab.shared.magazine.thread);

    /* on an arena: every byte pushed up front, nothing released by dealloc; the failure path pushes stay for the caller's pop */
    ArenaPushRecord(env->mem_1);
    const u64 mem_left = env->mem_1->mem_left;
    TEST_EQUAL(1, ds_TSlabAlloc(env->mem_1, slab, 1024, 2));
    TEST_NOT_EQUAL(NULL, slab.buf);
    TEST_EQUAL(0, slab.shared.heap_allocated);
    TEST_EQUAL(1024, slab.shared.length);
    TEST_NOT_ZERO((u8 *) slab.buf >= env->mem_1->stack_ptr - (mem_left - env->mem_1->mem_left) && (u8 *) slab.buf < env->mem_1->stack_ptr);
    TEST_NOT_ZERO(mem_left - env->mem_1->mem_left >= 1024*sizeof(struct slab_item));
    ds_Array(slab_item) arr = ds_TSlabAdd(slab, 1024);
    TEST_EQUAL(1024, arr.length);
    TEST_EQUAL((void *) slab.buf, (void *) arr.buf);
    TEST_EQUAL(0, ds_TSlabAdd(slab, 1).length);
    ds_TSlabRemove(slab, arr);
    arr = ds_TSlabAdd(slab, 1024);
    TEST_EQUAL(0, arr.base);
    const u64 mem_left_before_dealloc = env->mem_1->mem_left;
    ds_TSlabDealloc(slab);
    TEST_EQUAL(mem_left_before_dealloc, env->mem_1->mem_left);
    TEST_EQUAL(NULL, slab.buf);

    struct arena small = ArenaAlloc(env->mem_1, 4*1024);
    TEST_EQUAL(0, ds_TSlabAlloc(&small, slab, 1024, 1));
    TEST_EQUAL(NULL, slab.buf);
    TEST_EQUAL(NULL, slab.shared.magazine.thread);
    ArenaPopRecord(env->mem_1);
	return output;
}

/* ds_TPool: single slots over the slab; the magazine hands a removed slot back, a full pool returns empty_slot */
static struct test_Output TPoolTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ds_TPool(slab_item) pool;
    TEST_EQUAL(1, ds_TPoolAlloc(NULL, pool, 8, 1));

    struct slot slot[8];
    for (u32 i = 0; i < 8; ++i)
    {
        slot[i] = ds_TPoolAdd(pool);
        TEST_EQUAL(i, slot[i].index);
        TEST_EQUAL((void *) (pool.buf + i), slot[i].address);
        ((struct slab_item *) slot[i].address)->a = i;
    }
    TEST_EQUAL(NULL, ds_TPoolAdd(pool).address);
    TEST_EQUAL(U32_MAX, ds_TPoolAdd(pool).index);

    /* removed slots come back before the region is touched again: 3 then 5, LIFO out of the magazine */
    ds_TPoolRemove(pool, 3);
    ds_TPoolRemove(pool, 5);
    TEST_EQUAL(5, ds_TPoolAdd(pool).index);
    TEST_EQUAL(3, ds_TPoolAdd(pool).index);
    TEST_EQUAL(NULL, ds_TPoolAdd(pool).address);

    ds_TPoolFlush(pool);
    TEST_EQUAL(0, pool.shared.a_count_max);
    TEST_EQUAL(0, ds_TPoolAdd(pool).index);

    ds_TPoolDealloc(pool);
    TEST_EQUAL(NULL, pool.buf);
    (void) env;
	return output;
}

/* ds_TSlabRealloc: the no-op band, growth with the copy, shrink with the first elements, release */
static struct test_Output TSlabReallocTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ds_TSlab(slab_item) slab;
    TEST_EQUAL(1, ds_TSlabAlloc(NULL, slab, (u32) (((u64) 16 << 20) / sizeof(struct slab_item)), 1));

    ds_Array(slab_item) arr = ds_TSlabAdd(slab, 3);
    TEST_EQUAL(4, arr.length);
    TEST_EQUAL((void *) (slab.buf + arr.base), (void *) arr.buf);
    for (u32 i = 0; i < 4; ++i) { arr.buf[i].a = 100 + i; }
    const u32 base_4 = arr.base;

    /* 3 and 4 fit the length-4 array, nothing moves */
    arr = ds_TSlabRealloc(slab, arr, 4);
    TEST_EQUAL(base_4, arr.base);
    arr = ds_TSlabRealloc(slab, arr, 3);
    TEST_EQUAL(base_4, arr.base);

    /* 5 needs 8: everything is copied, the old array goes back to the slab */
    arr = ds_TSlabRealloc(slab, arr, 5);
    TEST_EQUAL(8, arr.length);
    TEST_NOT_EQUAL(base_4, arr.base);
    for (u32 i = 0; i < 4; ++i) { TEST_EQUAL(100 + i, arr.buf[i].a); }
    const ds_Array(slab_item) again = ds_TSlabAdd(slab, 4);
    TEST_EQUAL(base_4, again.base);
    ds_TSlabRemove(slab, again);
    arr.buf[7].a = 107;

    /* 2 is at most half of 8: the first two elements move to a length-2 array */
    arr = ds_TSlabRealloc(slab, arr, 2);
    TEST_EQUAL(2, arr.length);
    TEST_EQUAL(100, arr.buf[0].a);
    TEST_EQUAL(101, arr.buf[1].a);

    /* 0 removes it */
    arr = ds_TSlabRealloc(slab, arr, 0);
    TEST_EQUAL(NULL, arr.buf);
    TEST_EQUAL(0, arr.length);
    TEST_EQUAL(U32_MAX, arr.base);

    ds_TSlabDealloc(slab);
    (void) env;
	return output;
}

/* ds_TSlabArrayTryDeserialize: the compact form of a grown array comes back at the rounded length, count 0, a cut stream, a full slab */
static struct test_Output TSlabArrayDeserializeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    ds_TSlab(slab_item) slab;
    TEST_EQUAL(1, ds_TSlabAlloc(NULL, slab, (u32) (((u64) 16 << 20) / sizeof(struct slab_item)), 1));

    /* 5 live elements in a length-8 array: written compact, restored as 5 in a length-8 array */
    ds_Array(slab_item) arr = ds_TSlabAdd(slab, 5);
    for (u32 i = 0; i < arr.length; ++i) { arr.buf[i].a = 200 + i; arr.buf[i].b = i; }
    const u32 live = 5;
    const u64 size = ds_ArrayCompactSerializeSize(arr, live);
    struct SS ss = SSAlloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, ss.buf);
    ds_ArrayCompactSerialize(&ss, arr, live);
    TEST_EQUAL(8*size, ss.bit_index);

    ss.bit_index = 0;
    ds_Array(slab_item) read;
    u32 count = U32_MAX;
    TEST_EQUAL(1, ds_TSlabArrayTryDeserialize(&ss, slab, read, &count));
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_EQUAL(live, count);
    TEST_EQUAL(8, read.length);
    TEST_NOT_EQUAL(arr.base, read.base);
    TEST_EQUAL((void *) (slab.buf + read.base), (void *) read.buf);
    TEST_EQUAL(0, memcmp(arr.buf, read.buf, live*sizeof(struct slab_item)));
    ds_TSlabRemove(slab, read);

    /* 3 live elements of the same array come back in a length-4 array: the restore shrinks */
    ss.bit_index = 0;
    ds_ArrayCompactSerialize(&ss, arr, 3);
    ss.bit_index = 0;
    TEST_EQUAL(1, ds_TSlabArrayTryDeserialize(&ss, slab, read, &count));
    TEST_EQUAL(3, count);
    TEST_EQUAL(4, read.length);
    ds_TSlabRemove(slab, read);

    /* count 0: a bare count, the empty array */
    ss.bit_index = 0;
    ds_ArrayCompactSerialize(&ss, arr, 0);
    ss.bit_index = 0;
    TEST_EQUAL(1, ds_TSlabArrayTryDeserialize(&ss, slab, read, &count));
    TEST_EQUAL(0, count);
    TEST_EQUAL(0, read.length);
    TEST_EQUAL(NULL, read.buf);

    /* a stream one byte short, and a corrupt count */
    ss.bit_index = 0;
    ds_ArrayCompactSerialize(&ss, arr, live);
    struct SS cut = SSBuffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_TSlabArrayTryDeserialize(&cut, slab, read, &count));
    TEST_EQUAL(0, cut.bit_index);
    TEST_EQUAL(0, count);
    TEST_EQUAL(0, read.length);
    struct SS corrupt = SSBuffered(ss.buf, size);
    SSWriteU32Le(&corrupt, live + 1);
    corrupt.bit_index = 0;
    TEST_EQUAL(0, ds_TSlabArrayTryDeserialize(&corrupt, slab, read, &count));
    TEST_EQUAL(0, corrupt.bit_index);

    /* a slab too small for the array: the stream stays, the array is empty */
    ds_TSlab(slab_item) tiny;
    TEST_EQUAL(1, ds_TSlabAlloc(NULL, tiny, 1, 1));
    const u32 full = tiny.shared.length;
    TEST_EQUAL(full, ds_TSlabAdd(tiny, full).length);
    ss.bit_index = 0;
    TEST_EQUAL(0, ds_TSlabArrayTryDeserialize(&ss, tiny, read, &count));
    TEST_EQUAL(0, ss.bit_index);
    TEST_EQUAL(0, read.length);
    ds_TSlabDealloc(tiny);

    ds_TSlabRemove(slab, arr);
    ds_TSlabDealloc(slab);
    return output;
}

struct test_TSlabItem { u32 owner; u32 serial; u32 pad[6]; };
DEFINE_ARRAY_STRUCT(test_TSlabItem);
DEFINE_TSLAB(test_TSlabItem);

/* an array handed to another worker to free */
struct test_TSlabHandoff
{
    TREIBER_NODE;
    u32 base;
    u32 length;
    u32 owner;
    u32 serial;
};

#define TSLAB_PARALLEL_HELD     256
#define TSLAB_PARALLEL_HANDOFF  1024
#ifndef TSLAB_PARALLEL_OPS
#define TSLAB_PARALLEL_OPS      200000
#endif

struct test_TSlabParallel
{
    ds_TSlab(test_TSlabItem)    slab;
    struct ds_TStack            handoff_free;
    struct ds_TStack            handoff_full;
    struct test_TSlabHandoff *  node;
    u32                         a_next_id;
    u32                         a_errors;
    u32                         a_live;
};

static void TSlabParallelFill(struct test_TSlabItem *item, const u32 length, const u32 owner, const u32 serial)
{
    for (u32 i = 0; i < length; ++i)
    {
        item[i].owner = owner;
        item[i].serial = serial;
    }
}

static u32 TSlabParallelCheck(const struct test_TSlabItem *item, const u32 length, const u32 owner, const u32 serial)
{
    u32 ok = 1;
    for (u32 i = 0; i < length; ++i)
    {
        ok &= (item[i].owner == owner) & (item[i].serial == serial);
    }
    return ok;
}

/*
One job: arrays of 1..8 items come and go in bursts (allocation-heavy, then free-heavy, by phase), every
array carries its owner and serial in every element and is checked before it is freed; some arrays are
handed over a shared stack to whichever worker takes them, so they are freed on another thread.
*/
static void TSlabParallelJob(void *args)
{
    struct test_TSlabParallel *t = args;
    const u32 id = AtomicAddFetchRlx32(&t->a_next_id, 1);
    struct rng rng;
    RngLocalInit(&rng, 0x75a6 + id);

    ds_Array(test_TSlabItem) held[TSLAB_PARALLEL_HELD];
    u32 serial_of[TSLAB_PARALLEL_HELD];
    u32 held_count = 0;
    u32 serial = 0;
    u32 errors = 0;

    for (u32 i = 0; i < TSLAB_PARALLEL_OPS && errors == 0; ++i)
    {
        /* phase 0 allocates three times as often as it frees, phase 1 the reverse */
        const u32 phase = (i / 2048) & 1;
        const u32 r = (u32) RngLocalU64Range(&rng, 0, 7);
        const u32 alloc = (phase == 0) ? (r < 6) : (r < 2);

        if (alloc && held_count < TSLAB_PARALLEL_HELD)
        {
            const u32 count = 1 + (u32) RngLocalU64Range(&rng, 0, 7);
            const ds_Array(test_TSlabItem) arr = ds_TSlabAdd(t->slab, count);
            if (arr.buf == NULL || arr.length < count || arr.length != PowerOfTwoCeil(count)
                || arr.buf != t->slab.buf + arr.base)
            {
                errors += 1;
                break;
            }
            serial += 1;
            TSlabParallelFill(arr.buf, arr.length, id, serial);
            AtomicAddFetchRlx32(&t->a_live, 1);
            held[held_count] = arr;
            serial_of[held_count] = serial;
            held_count += 1;
        }
        else if (held_count > 0)
        {
            const u32 pick = (u32) RngLocalU64Range(&rng, 0, held_count - 1);
            const ds_Array(test_TSlabItem) arr = held[pick];
            const u32 arr_serial = serial_of[pick];
            held_count -= 1;
            held[pick] = held[held_count];
            serial_of[pick] = serial_of[held_count];

            errors += !TSlabParallelCheck(arr.buf, arr.length, id, arr_serial);
            if (r & 1)
            {
                /* hand it over: another worker checks and frees it */
                u32 n;
                ds_TStackTryPop(n, &t->handoff_free, t->node);
                if (n != TSTACK_NULL)
                {
                    t->node[n].base = arr.base;
                    t->node[n].length = arr.length;
                    t->node[n].owner = id;
                    t->node[n].serial = arr_serial;
                    ds_TStackPush(&t->handoff_full, t->node, n);
                    continue;
                }
            }
            ds_TSlabRemove(t->slab, arr);
            AtomicAddFetchRlx32(&t->a_live, (u32) -1);
        }

        /* take an array handed over by anyone */
        if ((r & 3) == 0)
        {
            u32 n;
            ds_TStackTryPop(n, &t->handoff_full, t->node);
            if (n != TSTACK_NULL)
            {
                const struct test_TSlabHandoff h = t->node[n];
                ds_TStackPush(&t->handoff_free, t->node, n);
                const ds_Array(test_TSlabItem) arr = { .buf = t->slab.buf + h.base, .length = h.length, .base = h.base };
                errors += !TSlabParallelCheck(arr.buf, arr.length, h.owner, h.serial);
                ds_TSlabRemove(t->slab, arr);
                AtomicAddFetchRlx32(&t->a_live, (u32) -1);
            }
        }
    }

    while (held_count > 0)
    {
        held_count -= 1;
        errors += !TSlabParallelCheck(held[held_count].buf, held[held_count].length, id, serial_of[held_count]);
        ds_TSlabRemove(t->slab, held[held_count]);
        AtomicAddFetchRlx32(&t->a_live, (u32) -1);
    }

    AtomicAddFetchRlx32(&t->a_errors, errors);
}

/*
Every scheduler worker runs TSlabParallelJob on one slab; the bursts push whole magazines through the
depot and the handoffs free arrays on other threads than the allocating one. No element of a live array
may be overwritten (two arrays sharing slots), everything handed over is freed, and the slab holds nothing
live at the end.
*/
static struct test_Output TSlabParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_TSlabParallel *t = ds_Alloc(sizeof(*t), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t);
    TEST_EQUAL(1, ds_TSlabAlloc(NULL, t->slab, (u32) (((u64) 256 << 20) / sizeof(struct test_TSlabItem)), g_arch_config->logical_core_count));
    t->node = ds_Alloc(TSLAB_PARALLEL_HANDOFF*sizeof(struct test_TSlabHandoff), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, t->node);
    ds_TStackFlush(&t->handoff_free);
    ds_TStackFlush(&t->handoff_full);
    for (u32 i = 0; i < TSLAB_PARALLEL_HANDOFF; ++i)
    {
        t->node[i].treiber_next = TSTACK_NULL;
        ds_TStackPush(&t->handoff_free, t->node, i);
    }
    AtomicStoreRel32(&t->a_next_id, 0);
    AtomicStoreRel32(&t->a_errors, 0);
    AtomicStoreRel32(&t->a_live, 0);

    ds_TestRunParallel(TSlabParallelJob, t);
    TEST_EQUAL(0, AtomicLoadAcq32(&t->a_errors));

    /* the handoffs nobody took are checked and freed here */
    u32 taken = 0;
    for (;;)
    {
        u32 n;
        ds_TStackTryPop(n, &t->handoff_full, t->node);
        if (n == TSTACK_NULL)
        {
            break;
        }
        const struct test_TSlabHandoff h = t->node[n];
        const ds_Array(test_TSlabItem) arr = { .buf = t->slab.buf + h.base, .length = h.length, .base = h.base };
        TEST_TRUE(TSlabParallelCheck(arr.buf, arr.length, h.owner, h.serial));
        ds_TSlabRemove(t->slab, arr);
        AtomicAddFetchRlx32(&t->a_live, (u32) -1);
        taken += 1;
    }
    TEST_EQUAL(0, AtomicLoadAcq32(&t->a_live));
    TEST_TRUE(t->slab.shared.a_count_max <= t->slab.shared.length);

    /* every slot ever carved is free again: the depot and the local magazines hold them all */
    u32 free_count = 0;
    for (u32 c = 0; c < 31; ++c)
    {
        const u32 length = t->slab.shared.magazine.mclass_length[c];
        for (u32 w = 0; w < t->slab.shared.magazine.thread_count; ++w)
        {
            const struct ds_MagazineLocal *local = t->slab.shared.magazine.thread[w].magazine + c;
            free_count += (local->count[0] + local->count[1]) << c;
        }
        for (;;)
        {
            u32 m;
            ds_TStackTryPop(m, &t->slab.shared.magazine.mclass[c].stack_full, t->slab.shared.magazine.magazine);
            if (m == TSTACK_NULL)
            {
                break;
            }
            free_count += length << c;
        }
    }
    TEST_EQUAL(t->slab.shared.a_count_max, free_count);

    ds_Free(t->node);
    ds_TSlabDealloc(t->slab);
    ds_Free(t);
    (void) env;
	return output;
}

/* VM: a reservation, commits in slices, the no-op path inside the committed end, the cap at the reservation, release */
static struct test_Output VMTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    const u64 size = (u64) 64 << 20;
    struct VM vm = VMReserve(size);
    TEST_NOT_EQUAL(NULL, vm.buf);
    TEST_EQUAL(size, vm.reserved);
    TEST_EQUAL(0, vm.a_committed);

    /* one byte commits a whole slice, a multiple of the granularity */
    TEST_EQUAL(1, VMCommit(&vm, 1));
    TEST_TRUE(vm.a_committed >= 1 && vm.a_committed <= size && PowerOfTwoMultipleCheck(vm.a_committed, g_mem_config->vm_granularity));
    const u64 first = vm.a_committed;
    vm.buf[0] = 1;
    vm.buf[first - 1] = 2;
    TEST_EQUAL(1, vm.buf[0]);
    TEST_EQUAL(2, vm.buf[first - 1]);

    /* inside the committed end nothing moves */
    TEST_EQUAL(1, VMCommit(&vm, first));
    TEST_EQUAL(first, vm.a_committed);

    /* up to the last byte: the end is capped at the reservation */
    TEST_EQUAL(1, VMCommit(&vm, size));
    TEST_EQUAL(size, vm.a_committed);
    vm.buf[size - 1] = 3;
    TEST_EQUAL(3, vm.buf[size - 1]);

    /* an unrounded size rounds up to the granularity */
    struct VM odd = VMReserve(12345);
    TEST_NOT_EQUAL(NULL, odd.buf);
    TEST_TRUE(odd.reserved >= 12345 && PowerOfTwoMultipleCheck(odd.reserved, g_mem_config->vm_granularity));
    TEST_EQUAL(1, VMCommit(&odd, 12345));
    TEST_EQUAL(odd.reserved, odd.a_committed);
    odd.buf[12344] = 4;
    TEST_EQUAL(4, odd.buf[12344]);
    VMRelease(&odd);
    TEST_EQUAL(NULL, odd.buf);
    TEST_EQUAL(0, odd.reserved);

    VMRelease(&vm);
    TEST_EQUAL(NULL, vm.buf);
    (void) env;
	return output;
}

static struct test_Output (*allocator_tests[])(struct test_Environment *) =
{
    PowerOfTwoTest,
    HeapAllocTest,
    ArenaPushTest,
    ArenaRecordTest,
    ArenaPushAllTest,
    CPoolTest,
    CPoolSerializeTest,
    PoolTest,
    PoolSerializeTest,
    TStackTest,
    TStackParallelTest,
    RingTest,
    CPoolPushBatchTest,
    VMTest,
    SlabAllocTest,
    TPoolTest,
    TSlabReallocTest,
    TSlabArrayDeserializeTest,
    TSlabParallelTest,
};

struct suite_Correctness storage_allocator_correctness_suite =
{
	.id = "allocator",
	.unit_test = allocator_tests,
	.unit_test_count = sizeof(allocator_tests) / sizeof(allocator_tests[0]),
};

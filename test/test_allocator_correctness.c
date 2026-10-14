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
        TEST_EQUAL((n == p), PowerOfTwoCheck(n));
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
        TEST_EQUAL(0, (u64) addr % alignment);
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
            TEST_EQUAL(0, (u64) addr % alignment);

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
    TEST_EQUAL(0, (u64) ar.stack_ptr % DS_CACHE_LINE);
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
            const u64 pad = (alignment - ((u64) top & (alignment - 1))) & (alignment - 1);
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

    /* length 0, not growable (arena and heap): a stub exists, pushes report full and write into it */
    for (u32 heap = 0; heap <= 1; ++heap)
    {
        ds_CPool(u32) empty;
        ds_CPoolAlloc((heap) ? NULL : env->mem_1, empty, 0, NOT_GROWABLE);
        TEST_NOT_EQUAL(NULL, empty.buf);
        TEST_EQUAL(0, empty.length);
        TEST_EQUAL(U32_MAX, ds_CPoolPush(empty).index);
        ds_CPoolPushValue(empty, 7);
        const u32 src = 9;
        TEST_EQUAL(U32_MAX, ds_CPoolPushMemcpy(empty, &src).index);
        TEST_EQUAL(0, empty.count);
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
    struct ss ss = ss_Alloc(env->mem_1, size);
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

    struct ss cut = ss_Buffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_CPoolTryDeserialize(env->mem_2, &cut, read, NOT_GROWABLE));
    TEST_EQUAL(0, cut.bit_index);
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(0, read.count);
    TEST_EQUAL(mem_left, env->mem_2->mem_left);

    /* count > length */
    struct ss bad = ss_Alloc(env->mem_1, 2*sizeof(u32) + 2*sizeof(u64));
    ss_WriteU32Le(&bad, 1);
    ss_WriteU32Le(&bad, 2);
    bad.bit_index = 0;
    TEST_EQUAL(0, ds_CPoolTryDeserialize(env->mem_2, &bad, read, NOT_GROWABLE));
    TEST_EQUAL(0, bad.bit_index);
    TEST_EQUAL(NULL, read.buf);
    TEST_EQUAL(mem_left, env->mem_2->mem_left);

    /* an empty length-0 pool round-trips into a pool with a stub */
    ds_CPool(u64) empty;
    ds_CPoolAlloc(env->mem_2, empty, 0, NOT_GROWABLE);
    struct ss empty_ss = ss_Alloc(env->mem_1, ds_CPoolSerializeSize(empty));
    ds_CPoolSerialize(&empty_ss, empty);
    empty_ss.bit_index = 0;
    TEST_EQUAL(1, ds_CPoolTryDeserialize(env->mem_2, &empty_ss, read, NOT_GROWABLE));
    TEST_NOT_EQUAL(NULL, read.buf);
    TEST_EQUAL(0, read.length);
    TEST_EQUAL(U32_MAX, ds_CPoolPush(read).index);

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
        TEST_TRUE((variant == 0) ? pool.length == 100 : pool.length >= ((variant == 1) ? 100 : 1));

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
    struct ss ss = ss_Alloc(env->mem_1, size);
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

    struct ss cut = ss_Buffered(ss.buf, size - 1);
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

    struct ss bad = ss_Alloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, bad.buf);
    for (u32 i = 0; i < sizeof(corrupt_offset) / sizeof(corrupt_offset[0]); ++i)
    {
        for (u32 heap = 0; heap <= 1; ++heap)
        {
            memcpy(bad.buf, ss.buf, size);
            bad.bit_index = 8*corrupt_offset[i];
            ss_WriteU32Le(&bad, corrupt_value[i]);
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

struct test_TNode
{
    TPOOL_NODE;
    u32 value;
};

TPOOL_DECLARE(test_TNode)
TPOOL_DEFINE(test_TNode)

/*
 * Single thread: increments hand out 0, 1, 2, ... across doubling blocks at distinct addresses; removed slots
 * come back first (last removed first); flush restarts at index 0.
 */
static struct test_Output TPoolTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_TNodeTPool pool;
    test_TNodeTPoolAlloc(&pool, g_arch_config->logical_core_count, 3);
    TEST_EQUAL(4, pool.initial_length);

    /* blocks 4, 4, 8, ..., 512 hold 1024 slots */
    const u32 n = 1000;
    for (u32 i = 0; i < n; ++i)
    {
        const struct slot slot = test_TNodeTPoolIncrement(&pool);
        TEST_EQUAL(i, slot.index);
        TEST_EQUAL(slot.address, (void *) test_TNodeTPoolAddress(&pool, i));
        ((struct test_TNode *) slot.address)->value = i;
    }
    TEST_EQUAL(9, pool.block_count);

    u32 distinct = 1;
    for (u32 i = 0; i < n; ++i)
    {
        distinct &= (test_TNodeTPoolAddress(&pool, i)->value == i);
    }
    TEST_TRUE(distinct);

    test_TNodeTPoolRemove(&pool, 10);
    test_TNodeTPoolRemove(&pool, 500);
    TEST_EQUAL(500, test_TNodeTPoolAdd(&pool).index);
    TEST_EQUAL(10, test_TNodeTPoolAdd(&pool).index);
    TEST_EQUAL(n, test_TNodeTPoolAdd(&pool).index);

    test_TNodeTPoolFlush(&pool);
    TEST_EQUAL(0, test_TNodeTPoolIncrement(&pool).index);

    test_TNodeTPoolDealloc(&pool);

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
    TPoolTest,
};

struct suite_Correctness storage_allocator_correctness_suite =
{
	.id = "allocator",
	.unit_test = allocator_tests,
	.unit_test_count = sizeof(allocator_tests) / sizeof(allocator_tests[0]),
};

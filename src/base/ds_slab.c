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

ds_StaticAssert(offsetof(struct ds_TSlabShared, magazine) % DS_CACHE_LINE == 0, "");
ds_StaticAssert(offsetof(struct ds_TSlabShared, vm) % DS_CACHE_LINE == 0, "");

u32 ds_TSlabSharedAlloc(struct ds_TSlabShared *shared,
                       void **external,
                       const u32 thread_count,
                       const u64 slot_size,
                       const u64 vm_size)
{
	ds_Assert(slot_size > 0 && thread_count > 0);
	*shared = (struct ds_TSlabShared) { 0 };
	*external = NULL;

	shared->vm = VMReserve(vm_size);
	if (shared->vm.buf == NULL)
	{
        goto failure;
	}

	/* slot indices stay below DS_TSLAB_NULL */
	const u64 slots = shared->vm.reserved / slot_size;
	shared->length = (slots < U32_MAX) ? (u32) slots : U32_MAX - 1;

	/* an array takes at least one slot: the slot count bounds the arrays alive at once */
	if (!ds_MagazinePow2Alloc(NULL, &shared->magazine, thread_count, slot_size, shared->length))
	{
        goto failure;
	}

	*external = shared->vm.buf;
	return 1;

failure:
    ds_TSlabSharedDealloc(shared);
    return 0;
}

void ds_TSlabSharedDealloc(struct ds_TSlabShared *shared)
{
    ds_MagazinePow2Dealloc(&shared->magazine);
    if (shared->vm.buf) 
    { 
        VMRelease(&shared->vm); 
    }

    *shared = (struct ds_TSlabShared) { 0 };
}

void ds_TSlabSharedFlush(struct ds_TSlabShared *shared)
{
    ds_MagazinePow2Flush(&shared->magazine);
    shared->a_count_max = 0;
}

struct ds_ArrayUntyped ds_TSlabSharedAdd(struct ds_TSlabShared *shared, const u64 slot_size, const u32 count)
{
    ds_Assert(count > 0);
    struct ds_ArrayUntyped arr;
    arr.length = (u32) PowerOfTwoCeil(count);
    arr.base = ds_MagazinePow2TryPop(&shared->magazine, arr.length);
    if (arr.base == U32_MAX)
    {
        arr.base = AtomicFetchAddRlx32(&shared->a_count_max, arr.length);
        const u64 end = (u64) arr.base + arr.length;
        if (end > shared->length || !VMCommit(&shared->vm, end*slot_size))
        {
            return ds_ArrayEmpty();
        }
    }

    arr.buf = shared->vm.buf + arr.base*slot_size;
    UnpoisonAddress(arr.buf, arr.length*slot_size);
    return arr;
}

void ds_TSlabSharedRemove(struct ds_TSlabShared *shared, const u64 slot_size, const u32 base, const u32 length)
{
    ds_Assert(PowerOfTwoCheck(length));
    PoisonAddress(shared->vm.buf + base*slot_size, length*slot_size);
    ds_MagazinePow2Push(&shared->magazine, base, length);
}

struct ds_ArrayUntyped ds_TSlabSharedRealloc(struct ds_TSlabShared *shared, const u64 slot_size, const struct ds_ArrayUntyped array, const u32 count)
{
    ds_Assert(array.length);
    if (count == 0)
    {
        ds_TSlabSharedRemove(shared, slot_size, array.base, array.length);
        return ds_ArrayEmpty();
    }

    if ((array.length >> 1) < count && count <= array.length)
    {
        return array;
    }

    const struct ds_ArrayUntyped moved = ds_TSlabSharedAdd(shared, slot_size, count);
    if (moved.buf == NULL)
    {
        ds_TSlabOutOfMemoryInternal();
    }

    const u32 kept = (count < array.length) ? count : array.length;
    memcpy(moved.buf, array.buf, kept*slot_size);
    ds_TSlabSharedRemove(shared, slot_size, array.base, array.length);
    return moved;
}

u32 ds_TSlabSharedArrayTryDeserialize(struct SS *ss, struct ds_TSlabShared *shared, struct ds_ArrayUntyped *array, u32 *count, const u64 slot_size)
{
    *array = ds_ArrayEmpty();
    *count = 0;
    if (SSBytesLeft(ss) < sizeof(u32))
    {
        return 0;
    }

    const u64 bit_index = ss->bit_index;
    const u32 serialized_count = SSReadU32Le(ss);
    /* division instead of count*slot_size: a corrupt count must not overflow the check */
    if (serialized_count > SSBytesLeft(ss) / slot_size)
    {
        ss->bit_index = bit_index;
        return 0;
    }

    if (serialized_count > 0)
    {
        *array = ds_TSlabSharedAdd(shared, slot_size, serialized_count);
        if (array->buf == NULL)
        {
            ss->bit_index = bit_index;
            return 0;
        }
        SSRead8N(ss, (b8 *) array->buf, serialized_count*slot_size);
    }
    *count = serialized_count;
    return 1;
}

void ds_TSlabOutOfMemoryInternal(void)
{
    LogString(T_SYSTEM, S_FATAL, "Slab-Allocator OOM, exiting!");
    FatalCleanupAndExit();
}

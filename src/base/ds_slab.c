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

ds_StaticAssert(sizeof(struct ds_TSlabThread) == DS_CACHE_LINE, "");
ds_StaticAssert(sizeof(struct ds_TSlabShared) == 4*DS_CACHE_LINE, "");

u32 ds_TSlabSharedAlloc(struct ds_TSlabShared *shared,
                       void **external,
                       const u32 thread_count,
                       const u64 slot_size,
                       const u64 vm_size)
{
	ds_Assert(slot_size >= sizeof(u32) && thread_count > 0);
	*shared = (struct ds_TSlabShared) { 0 };
	*external = NULL;

	shared->vm = VMReserve(vm_size);
	if (shared->vm.buf == NULL)
	{
        goto failure;
	}

	shared->thread = ds_Alloc(thread_count*sizeof(struct ds_TSlabThread), DS_CACHE_LINE);
	if (shared->thread == NULL)
	{
        goto failure;
	}
	memset(shared->thread, 0, thread_count*sizeof(struct ds_TSlabThread));
	shared->thread_count = thread_count;

	for (u32 i = 0; i < thread_count; ++i)
	{
		struct ds_TSlabThread *t = shared->thread + i;
		ds_CPoolAlloc(NULL, t->free, DS_TSLAB_COUNT_DEFAULT, GROWABLE);
		if (t->free.buf == NULL)
		{
            goto failure;
		}

		const struct slot arr = ds_CPoolPushBatch(t->free, DS_TSLAB_COUNT_DEFAULT);
		memset(arr.address, 0xff, DS_TSLAB_COUNT_DEFAULT*sizeof(u32));
	}

	/* slot indices stay below DS_TSLAB_NULL */
	const u64 slots = shared->vm.reserved / slot_size;
	shared->length = (slots < U32_MAX) ? (u32) slots : U32_MAX - 1;
	*external = shared->vm.buf;
	return 1;

failure:
    ds_TSlabSharedDealloc(shared);
    return 0;
}

void ds_TSlabSharedDealloc(struct ds_TSlabShared *shared)
{
    if (shared->vm.buf) 
    { 
        VMRelease(&shared->vm); 
    }

    if (shared->thread)
    {
	    for (u32 i = 0; i < shared->thread_count; ++i)
	    {
	        ds_CPoolDealloc(shared->thread[i].free);
        }
        ds_Free(shared->thread);
    }

    *shared = (struct ds_TSlabShared) { 0 };
}

void ds_TSlabSharedFlush(struct ds_TSlabShared *shared)
{
    for (u32 i = 0; i < shared->thread_count; ++i)
	{
		struct ds_TSlabThread *t = shared->thread + i;
		memset(t->free.buf, 0xff, t->free.count*sizeof(u32));
        t->count_allocated = 0;
	}
    shared->a_count_max = 0;
}

void ds_TSlabOutOfMemoryInternal(void)
{
    LogString(T_SYSTEM, S_FATAL, "Slab-Allocator OOM, exiting!");
    FatalCleanupAndExit();
}

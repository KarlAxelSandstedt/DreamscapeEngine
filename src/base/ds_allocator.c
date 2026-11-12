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

#if __DS_PLATFORM__ == __DS_LINUX__

#define _GNU_SOURCE

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "ds_base.h"
#include "ds_serialize.h"

ds_StaticAssert(sizeof(struct VM) == 2*DS_CACHE_LINE, "");

struct memConfig g_mem_config_storage = { 0 };
struct memConfig *g_mem_config = &g_mem_config_storage;

void ds_MemApiShutdown(void)
{
}

void ds_MemApiInit(const u64 vm_commit_chunk_size)
{
	g_mem_config->page_size = ds_Pagesize();
	ds_Assert( PowerOfTwoCheck( g_mem_config->page_size ) );
#if __DS_PLATFORM__ == __DS_LINUX__
	g_mem_config->vm_granularity = g_mem_config->page_size;
#elif __DS_PLATFORM__ == __DS_WIN64__
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	g_mem_config->vm_granularity = info.dwAllocationGranularity;
#endif
	ds_Assert( PowerOfTwoCheck( g_mem_config->vm_granularity ) );
	const u64 floor = g_mem_config->vm_granularity;
	g_mem_config->vm_commit_chunk = PowerOfTwoCeil((vm_commit_chunk_size > floor) ? vm_commit_chunk_size : floor);
}

/* debug builds fill new memory: nothing may rely on it being zeroed */
#ifdef DS_DEBUG
	#define DS_ALLOC_DEBUG_FILL(addr, size)	memset((addr), 0xa5, (size))
#else
	#define DS_ALLOC_DEBUG_FILL(addr, size)
#endif

#if __DS_PLATFORM__ == __DS_LINUX__

#include <stddef.h>
#include <sys/mman.h>

void *ds_Alloc(const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (size == 0)
	{
		return NULL;
	}

	/* posix_memalign: the alignment must be a multiple of sizeof(void *) */
	const u64 alignment_used = (alignment < sizeof(void *)) ? sizeof(void *) : alignment;
	void *addr = NULL;
	if (posix_memalign(&addr, alignment_used, size) != 0)
	{
		return NULL;
	}

	DS_ALLOC_DEBUG_FILL(addr, size);
	return addr;
}

/* resize a non-NULL block to new_size > 0; NULL on failure */
static void *ds_ReallocPlatform(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	/* realloc keeps malloc's alignment (max_align_t); glibc moves large blocks with mremap, without a copy */
	if (alignment <= _Alignof(max_align_t))
	{
		return realloc(addr, new_size);
	}

	void *new_addr = ds_Alloc(new_size, alignment);
	if (new_addr)
	{
		memcpy(new_addr, addr, (old_size < new_size) ? old_size : new_size);
		free(addr);
	}
	return new_addr;
}

void ds_Free(void *addr)
{
	free(addr);
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <malloc.h>
#include <windows.h>

void *ds_Alloc(const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (size == 0)
	{
		return NULL;
	}

	void *addr = _aligned_malloc(size, alignment);
	if (addr)
	{
		DS_ALLOC_DEBUG_FILL(addr, size);
	}
	return addr;
}

/* resize a non-NULL block to new_size > 0; NULL on failure */
static void *ds_ReallocPlatform(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	return _aligned_realloc(addr, new_size, alignment);
}

void ds_Free(void *addr)
{
	_aligned_free(addr);
}

#else

#error

#endif

/*
TODO: a dedicated heap allocator. It would know its block sizes, so old_size could go (only the aligned
copy on Linux/web needs it).
*/
void *ds_Realloc(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (new_size == 0)
	{
		ds_Free(addr);
		return NULL;
	}

	void *new_addr = (addr)
		? ds_ReallocPlatform(addr, old_size, new_size, alignment)
		: ds_Alloc(new_size, alignment);
	if (new_addr == NULL)
	{
		/* a failed resize leaves addr intact: a shrink keeps the larger block, only growth is fatal */
		if (addr && new_size <= old_size)
		{
			return addr;
		}
		LogString(T_SYSTEM, S_FATAL, "Failed to grow a block in ds_Realloc, exiting.");
		FatalCleanupAndExit();
	}

	if (addr && old_size < new_size)
	{
		DS_ALLOC_DEBUG_FILL((u8 *) new_addr + old_size, new_size - old_size);
	}
	return new_addr;
}

struct VM VMReserve(const u64 size)
{
    if (!size)
    {
        return (struct VM) { 0 };
    }

    const u64 granularity = g_mem_config->vm_granularity;
    const u64 reserved = PowerOfTwoMultipleCeil(size, granularity);

#if __DS_PLATFORM__ == __DS_LINUX__

    void *buf = mmap(NULL, reserved, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (buf == MAP_FAILED)
    {
        LogSystemError(S_ERROR);
        return (struct VM) { 0 };
    }

#elif __DS_PLATFORM__ == __DS_WIN64__

    void *buf = VirtualAlloc(NULL, reserved, MEM_RESERVE, PAGE_NOACCESS);
    if (buf == NULL)
    {
        LogSystemError(S_ERROR);
        return (struct VM) { 0 };
    }

#endif
    return (struct VM) { .buf = buf, .reserved = reserved, .a_committed = 0 };
}

u32 VMCommitInternal(struct VM *vm, const u64 end)
{
    ds_Assert(vm->buf && end <= vm->reserved);
    u64 new_end = PowerOfTwoMultipleCeil(end, g_mem_config->vm_commit_chunk);
    if (new_end > vm->reserved)
    {
        new_end = vm->reserved;
    }

    u64 committed = AtomicLoadAcq64(&vm->a_committed);
    while (committed < new_end)
    {
#if __DS_PLATFORM__ == __DS_WIN64__
        if (VirtualAlloc(vm->buf + committed, new_end - committed, MEM_COMMIT, PAGE_READWRITE) == NULL)
        {
            LogSystemError(S_ERROR);
            return 0;
        }
#endif
        if (AtomicCompareExchangeAcqRelAcq64(&vm->a_committed, &committed, new_end))
        {
            break;
        }
    }
    return 1;
}

void VMRelease(struct VM *vm)
{
    if (vm->buf)
    {
#if __DS_PLATFORM__ == __DS_LINUX__
        if (munmap(vm->buf, vm->reserved) != 0)
        {
            LogSystemError(S_ERROR);
        }
#elif __DS_PLATFORM__ == __DS_WIN64__
        if (!VirtualFree(vm->buf, 0, MEM_RELEASE))
        {
            LogSystemError(S_ERROR);
        }
#endif
    }
    *vm = (struct VM) { 0 };
}

/****************************** ds_MagazinePow2 ******************************/

ds_StaticAssert(offsetof(struct ds_MagazinePow2, a_index_count) % DS_CACHE_LINE == 0, "");
ds_StaticAssert(offsetof(struct ds_MagazinePow2, a_magazine_count) % DS_CACHE_LINE == 0, "");
ds_StaticAssert(offsetof(struct ds_MagazinePow2, vm_index) % DS_CACHE_LINE == 0, "");

static u32 ds_MagazineClassLength(const u64 mclass, const u64 slot_size)
{
    ds_Assert(mclass < 31);
    const u64 array_length = (u64) 1 << mclass;
    const u64 array_size = slot_size*array_length;

    const u32 m_length = U32Clamp((u32) (DS_MAGAZINE_SIZE_HINT / array_size), 
                                  DS_MAGAZINE_LENGTH_MIN,
                                  DS_MAGAZINE_LENGTH_MAX);
    return m_length;
}

u32 ds_MagazinePow2Alloc(struct arena *mem,
                            struct ds_MagazinePow2 *storage,
                            const u32 thread_count,
                            const u64 slot_size,
                            const u32 max_array_count)
{
    *storage = (struct ds_MagazinePow2) { 0 };
    storage->thread_count = thread_count;

    u32 local_length = 0;
    for (u32 i = 0; i < 31; ++i)
    {
        storage->mclass_length[i] = ds_MagazineClassLength(i, slot_size);
        local_length += 2*storage->mclass_length[i];
    }

    const u64 max_magazine_count = (u64) max_array_count / DS_MAGAZINE_LENGTH_MIN;

    storage->index_length = max_array_count;
    storage->magazine_length = (u32) max_magazine_count;

    const u64 vm_index_size = (u64) max_array_count*sizeof(u32);
    const u64 vm_magazine_size = max_magazine_count*sizeof(struct ds_MagazineShared);
    if (mem)
    {
        storage->heap_allocated = 0;
        storage->thread = ArenaPushAligned(mem, thread_count*sizeof(struct ds_MagazineThread), DS_CACHE_LINE);
        storage->magazine = ArenaPushAligned(mem, vm_magazine_size, DS_CACHE_LINE);
        storage->index = ArenaPushAligned(mem, vm_index_size, DS_CACHE_LINE);

        storage->vm_magazine = (struct VM) { .buf = NULL, .reserved = vm_magazine_size, .a_committed = vm_magazine_size };
        storage->vm_index = (struct VM) { .buf = NULL, .reserved = vm_index_size, .a_committed = vm_index_size };
    }
    else
    {
        storage->heap_allocated = 1;
        storage->thread = ds_Alloc(thread_count*sizeof(struct ds_MagazineThread), DS_CACHE_LINE);
        storage->vm_index = VMReserve(vm_index_size);
        storage->vm_magazine = VMReserve(vm_magazine_size);
        storage->index = (u32 *) storage->vm_index.buf;
        storage->magazine = (struct ds_MagazineShared *) storage->vm_magazine.buf;
    }

    if (!storage->thread || !storage->magazine || !storage->index)
    {
        ds_MagazinePow2Dealloc(storage);
        return 0;
    }

    for (u32 i = 0; i < storage->thread_count; ++i)
    {
        struct ds_MagazineThread *thr = storage->thread + i;
        ds_CPoolAlloc(mem, thr->index_pool, local_length, NOT_GROWABLE);
        if (thr->index_pool.buf == NULL)
        {
            ds_MagazinePow2Dealloc(storage);
            return 0;
        }

        ds_CPoolPushBatch(thr->index_pool, local_length);
        u32 base = 0;
        for (u32 k = 0; k < 31; ++k)
        {
            thr->magazine[k].base[0] = base;
            base += storage->mclass_length[k];
            thr->magazine[k].base[1] = base;
            base += storage->mclass_length[k];
        }
    }
 
    ds_MagazinePow2Flush(storage);
    
    return 1;
}

void ds_MagazinePow2Dealloc(struct ds_MagazinePow2 *storage)
{
    if (storage->thread)
    {
        for (u32 i = 0; i < storage->thread_count; ++i)
        {
            ds_CPoolDealloc(storage->thread[i].index_pool);
        }
    }

    if (storage->heap_allocated)
    {
        ds_Free(storage->thread);
        VMRelease(&storage->vm_index);
        VMRelease(&storage->vm_magazine);
    }

    *storage = (struct ds_MagazinePow2) { 0 };
}

void ds_MagazinePow2Flush(struct ds_MagazinePow2 *storage)
{
    for (u32 i = 0; i < storage->thread_count; ++i)
    {
        struct ds_MagazineThread *thr = storage->thread + i;
        for (u32 j = 0; j < 31; ++j)
        {
            thr->magazine[j].cur = 0;
            thr->magazine[j].count[0] = 0;
            thr->magazine[j].count[1] = 0;
        }
    }

    for (u32 i = 0; i < 31; ++i)
    {
        ds_TStackFlush(&storage->mclass[i].stack_full);
        ds_TStackFlush(&storage->mclass[i].stack_empty);
    }

    storage->a_index_count = 0;
    storage->a_magazine_count = 0;
}

static u32 ds_MagazineThreadTryPush(struct ds_MagazineThread *t, const u32 mclass, const u32 mclass_length, const u32 index)
{
    struct ds_MagazineLocal *local = t->magazine + mclass;
    const u32 cur = local->cur;
    if (local->count[cur] == mclass_length)
    {
        return 0;
    }

    t->index_pool.buf[local->base[cur] + local->count[cur]] = index;
    local->count[cur] += 1;
    return 1;
}

void ds_MagazinePow2Push(struct ds_MagazinePow2 *storage, const u32 index, const u32 count)
{
    ds_Assert(PowerOfTwoCheck(count));

    const u32 mclass = Ctz32(count);
    const u32 mclass_length = storage->mclass_length[ mclass ];
    struct ds_MagazineThread *t = storage->thread + ds_ThreadSelfIndex();
    struct ds_MagazineLocal *local = t->magazine + mclass;
    if (ds_MagazineThreadTryPush(t, mclass, mclass_length, index))
    {
        return;
    }

    local->cur = 1 - local->cur;
    if (ds_MagazineThreadTryPush(t, mclass, mclass_length, index))
    {
        return;
    }

    struct ds_TStack *stack_full = &storage->mclass[ mclass ].stack_full;
    struct ds_TStack *stack_empty = &storage->mclass[ mclass ].stack_empty;

    /* Failed to save index to thread, both magazines full, try grab an empty one. */
    u32 m;
    ds_TStackTryPop(m, stack_empty, storage->magazine);
    if (m == TSTACK_NULL)
    {
        m = AtomicFetchAddRlx32(&storage->a_magazine_count, 1);
        const u32 base = AtomicFetchAddRlx32(&storage->a_index_count, mclass_length);
        /* m fits whenever base does: the node count is derived from the index count */
        if ((u64) base + mclass_length > storage->index_length)
        {
            LogString(T_SYSTEM, S_FATAL, "magazine storage: more arrays than max_array_count, exiting");
            FatalCleanupAndExit();
        }
        ds_Assert(m < storage->magazine_length);
        VMCommit(&storage->vm_magazine, ((u64) m + 1)*sizeof(struct ds_MagazineShared));
        VMCommit(&storage->vm_index, ((u64) base + mclass_length)*sizeof(u32));
        storage->magazine[m].treiber_next = TSTACK_NULL;
        storage->magazine[m].base = base;
    }

    struct ds_MagazineShared *shared = storage->magazine + m;
    ds_Assert(local->count[ local->cur ] == mclass_length);

    const u32 *local_index = t->index_pool.buf + local->base[ local->cur ];
    u32 *shared_index = storage->index + shared->base;
    memcpy(shared_index, local_index, mclass_length*sizeof(u32));

    ds_TStackPush(stack_full, storage->magazine, m);

    local->count[ local->cur ] = 1;
    t->index_pool.buf[local->base[ local->cur ]] = index;
}

static u32 ds_MagazineThreadTryPop(struct ds_MagazineThread *t, const u32 mclass)
{
    struct ds_MagazineLocal *local = t->magazine + mclass;
    const u32 cur = local->cur;
    if (local->count[cur] == 0)
    {
        return U32_MAX;
    }

    local->count[cur] -= 1;
    return t->index_pool.buf[local->base[cur] + local->count[cur]];
}

u32 ds_MagazinePow2TryPop(struct ds_MagazinePow2 *storage, const u32 count)
{
    ds_Assert(PowerOfTwoCheck(count));
    const u32 mclass = Ctz32(count);
    const u32 mclass_length = storage->mclass_length[ mclass ];

    struct ds_MagazineThread *t = storage->thread + ds_ThreadSelfIndex();
    struct ds_MagazineLocal *local = t->magazine + mclass;

    u32 index = ds_MagazineThreadTryPop(t, mclass);
    if (index != U32_MAX)
    {
        return index;
    }

    local->cur = 1 - local->cur;
    index = ds_MagazineThreadTryPop(t, mclass);
    if (index != U32_MAX)
    {
        return index;
    }

    struct ds_TStack *stack_full = &storage->mclass[ mclass ].stack_full;
    struct ds_TStack *stack_empty = &storage->mclass[ mclass ].stack_empty;

    /* Failed to save index to thread, both magazines empty, try grab full one. */
    u32 m;
    ds_TStackTryPop(m, stack_full, storage->magazine);
    if (m != TSTACK_NULL)
    {
        struct ds_MagazineShared *shared = storage->magazine + m;

        u32 *local_index = t->index_pool.buf + local->base[ local->cur ];
        const u32 *shared_index = storage->index + shared->base;
        memcpy(local_index, shared_index, mclass_length*sizeof(u32));

        ds_TStackPush(stack_empty, storage->magazine, m);

        local->count[ local->cur ] = mclass_length - 1;
        return local_index[ local->count[ local->cur ] ];
    }

    /* No full magazine exist, signal to caller that they have should allocate objects. */
    return U32_MAX;
}

#include "ds_arena.c"
#include "ds_ring.c"
#include "ds_compact_pool.c"
#include "ds_pool.c"
#include "ds_slab.c"

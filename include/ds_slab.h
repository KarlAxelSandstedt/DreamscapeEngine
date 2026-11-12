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
ds_slab.h
=========
TSlab: ds_TSlab(T), arrays of T from one reserved region, handles instead of pointers.
Not implemented yet.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_SLAB_H__
#define __DS_SLAB_H__

////TODO poison slab memory
////TODO compact api
////TODO mem api
////TODO add serialize api

/*
Slab API
========

::: Usage and Documentation :::

    struct item { ... };
    DEFINE_TSLAB(item);

    ds_TSlab(item) slab;
    ds_TSlabAlloc(slab, vm_size, thread_count);
    ds_TSlabFlush(slab);
    ds_TSlabDealloc(slab);


    // Reserve >= vm_size memory and initialize allocator.
    void ds_TSlabAlloc(slab, vm_size, thread_count)

    // Release any memory held by allocator
    void ds_TSlabDealloc(slab)

    // Flush allocator resources, reseting counts.
    void ds_TSlabFlush(slab)

    // On success return (ptr_base, index_base). On failure, (NULL, U32_MAX) is returned.
    struct slot ds_TSlabAdd(slab, count)

    // Remove slab at index.
    void ds_TSlabRemove(slab, index, count)

    // Remove slab at pointer.
    void ds_TSlabRemoveAddress(slab, ptr, count)
*/

#define DS_TSLAB_COUNT_DEFAULT   64
#define DS_TSLAB_NULL            U32_MAX

struct ds_Align(DS_CACHE_LINE) ds_TSlabThread
{
    ds_CPool(u32)   free;
    u32             count_allocated;
    u8              pad[DS_CACHE_LINE - sizeof(ds_CPool(u32)) - sizeof(u32)];
};

struct ds_Align(DS_CACHE_LINE) ds_TSlabShared
{
    u32                     a_count_max;
    u8                      pad[DS_CACHE_LINE - sizeof(u32)];
    struct VM               vm;
    u32                     length;
    u32                     thread_count;
    struct ds_TSlabThread *  thread;
};

#define ds_TSlab(T) ds_TSlab_ ## T

#define DEFINE_TSLAB(T)                                                                 \
typedef struct ds_TSlab(T)                                                              \
{                                                                                       \
    struct ds_TSlabShared    shared;                                                    \
    struct T *              buf;                                                        \
} ds_TSlab(T)

u32     ds_TSlabSharedAlloc(struct ds_TSlabShared *shared,
                           void **external,
                           const u32 thread_count,
                           const u64 slot_size,
                           const u64 vm_size);

void    ds_TSlabSharedDealloc(struct ds_TSlabShared *shared);

void    ds_TSlabSharedFlush(struct ds_TSlabShared *shared);

/* Internal: log and exit; the Log API is not visible here */
ds_NoReturn void ds_TSlabOutOfMemoryInternal(void);

/****************************** Slab inline definitions ******************************/

#define ds_TSlabAlloc(_slab_, _vm_size_, _thread_count_)                                \
    ds_TSlabSharedAlloc(&(_slab_).shared, (void **) &(_slab_).buf, (_thread_count_),    \
                       sizeof((_slab_).buf[0]), (_vm_size_))

#define ds_TSlabDealloc(_slab_)                                                         \
do                                                                                      \
{                                                                                       \
    ds_TSlabSharedDealloc(&(_slab_).shared);                                            \
    (_slab_).buf = NULL;                                                                \
} while (0)

#define ds_TSlabFlush(_slab_)      ds_TSlabSharedFlush(&(_slab_).shared)

#define ds_TSlabAdd(_slab_, _count_)                                                    \
    ds_TSlabSharedAdd(&(_slab_).shared, sizeof((_slab_).buf[0]), (_count_))

#define ds_TSlabRemove(_slab_, _index_, _count_)                                        \
    ds_TSlabSharedRemove(&(_slab_).shared, sizeof((_slab_).buf[0]), (_index_), (_count_))

#define ds_TSlabRemoveAddress(_slab_, _ptr_, _count_)                                   \
    ds_TSlabRemove(_slab_, (u32) ((_ptr_) - (_slab_).buf), (_count_))

static ds_ForceInline void ds_TSlabFreeExtend(struct ds_TSlabThread *thr, const u32 class)
{
    if (class >= thr->free.count)
    {
        const u32 push_count = class + 1 - thr->free.count;
        const struct slot arr = ds_CPoolPushBatch(thr->free, push_count);
        if (!arr.address)
        {
            ds_TSlabOutOfMemoryInternal();
        }
        memset(arr.address, 0xff, push_count*sizeof(u32));
    }
}

static ds_ForceInline struct slot ds_TSlabSharedAdd(struct ds_TSlabShared *shared,
                                                   const u64 slot_size,
                                                   const u32 count)
{
    struct slot arr = { .address = NULL, .index = DS_TSLAB_NULL }; 

    struct ds_TSlabThread *thr = shared->thread + ds_ThreadSelfIndex();
    ds_TSlabFreeExtend(thr, count);

    if (thr->free.buf[count] != U32_MAX)
    {
        arr.index = thr->free.buf[count];
        thr->free.buf[count] = *(u32 *) (shared->vm.buf + arr.index*slot_size);
    }
    else
    {
        arr.index = AtomicFetchAddRlx32(&shared->a_count_max, count);
        const u32 end = arr.index + count;
        if (end > shared->length || !VMCommit(&shared->vm, end*slot_size))
        {
            ds_TSlabOutOfMemoryInternal();
        }
    }

    arr.address = shared->vm.buf + arr.index*slot_size;
    thr->count_allocated += count;
    return arr;
}

static ds_ForceInline void ds_TSlabSharedRemove(struct ds_TSlabShared *shared,
                                               const u64 slot_size,
                                               const u32 index,
                                               const u32 count)
{
    ds_Assert(count > 0 && (u64) index + count <= AtomicLoadRlx32(&shared->a_count_max));
    struct ds_TSlabThread *t = shared->thread + ds_ThreadSelfIndex();
    ds_TSlabFreeExtend(t, count);

    *(u32 *) (shared->vm.buf + index*slot_size) = t->free.buf[count];
    t->free.buf[count] = index;
    t->count_allocated -= count;
}


#endif

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
Slab: ds_Slab(T), arrays of T from one reserved region, handles instead of pointers.
Not implemented yet.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_SLAB_H__
#define __DS_SLAB_H__


//* Return 1 on successful allocation, 0 on failure. */
//u32         ds_SlabAlloc(mem, intial_length, initial_class_count)
///* Free slab resources, if heap-allocated. */
//void        ds_SlabDealloc(pool)
///* Flush slab resources */
//void        ds_PoolFlush(pool)
///* On success return (ptr_base, index_base). On failure, (NULL, U32_MAX) is returned. */
//struct slot ds_SlabAdd(&slab, count);   
///* Remove slab at index. */
//struct      ds_SlabRemove(&slab, base_index);
///* Remove slab at pointer. */
//struct      ds_SlabRemoveAddress(&slab, ptr);
//
////TODO compact api
////TODO mem api
////TODO add serialize api

/*
Slab API
========
//TODO

::: Usage and Documentation :::

    struct item { ... };
    DEFINE_SLAB(item);

    ds_Slab(item) slab;
    ds_SlabAlloc(slab, vm_size, thread_count);
    ds_SlabFlush(slab);
    ds_SlabDealloc(slab);

    //TODO
    void *buf = 

The macros (slab is a ds_Slab(T) lvalue; arguments may be evaluated more than once):

    // Reserve >= vm_size memory and initialize allocator.
    void ds_SlabAlloc(slab, vm_size, thread_count)

    // Release any memory held by allocator
    void ds_SlabDealloc(slab)

    // Flush allocator resources, reseting counts.
    void ds_SlabFlush(slab)
*/

#define DS_SLAB_COUNT_DEFAULT   64
#define DS_SLAB_NULL            U32_MAX

struct ds_Align(DS_CACHE_LINE) ds_SlabThread
{
    ds_CPool(u32)   free;
    u32             count_allocated;
    u8              pad[DS_CACHE_LINE - sizeof(ds_CPool(u32)) - sizeof(u32)];
};

struct ds_Align(DS_CACHE_LINE) ds_SlabShared
{
    u32                     a_count_max;
    u8                      pad[DS_CACHE_LINE - sizeof(u32)];
    struct VM               vm;
    u32                     length;
    u32                     thread_count;
    struct ds_SlabThread *  thread;
};

#define ds_Slab(T) ds_Slab_ ## T

#define DEFINE_SLAB(T)                                                                  \
typedef struct ds_Slab(T)                                                               \
{                                                                                       \
    struct ds_SlabShared    shared;                                                     \
    struct T *              buf;                                                        \
} ds_Slab(T)

u32     ds_SlabSharedAlloc(struct ds_SlabShared *shared, 
                           const u32 thread_count, 
                           const u64 slot_size,
                           const u64 vm_size);

void    ds_SlabSharedDealloc(struct ds_SlabShared *shared);

void    ds_SlabSharedFlush(struct ds_SlabShared *shared);

/****************************** Slab inline definitions ******************************/

#define ds_SlabAlloc(slab, vm_size, thread_count)                                       \
    (ds_SlabSharedAlloc(&(slab).shared, (thread_count), sizeof((slab).buf[0]),           \
                        (vm_size))                                                      \
        ? ((slab).buf = (void *) (slab).shared.vm.buf, 1)                               \
        : ((slab).buf = NULL, 0))

#define ds_SlabDealloc(slab)                                                            \
do                                                                                      \
{                                                                                       \
    ds_SlabSharedDealloc(&(slab).shared);                                               \
    (slab).buf = NULL;                                                                  \
} while (0)

#define ds_SlabFlush(slab)      ds_SlabSharedFlush(&(slab).shared)

#endif

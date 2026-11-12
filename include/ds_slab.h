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
ds_TSlab: per-struct thread-safe slab alloactor.
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
    DEFINE_ARRAY_STRUCT(item);
    DEFINE_TSLAB(item);

    ds_TSlab(item) slab;
    ds_TSlabAlloc(slab, vm_size, thread_count);
    ds_TSlabFlush(slab);
    ds_TSlabDealloc(slab);


    // Master: Reserve >= vm_size memory and initialize allocator.
    void ds_TSlabAlloc(slab, vm_size, thread_count)

    // Master: Release any memory held by allocator
    void ds_TSlabDealloc(slab)

    // Master: Flush allocator resources, reseting counts.
    void ds_TSlabFlush(slab)

    // Thread-safe:
    // Returns ds_Array( T ) with length >= count (smallest power of two). 
    // On failure: { .length = 0, .buf = NULL, .base = U32_MAX }.
    ds_Array(T) ds_TSlabAdd(slab, count)

    // Thread-safe: Remove array.
    void ds_TSlabRemove(slab, array)

    // Thread-safe:
    // Returns reallocated array, or { .length = 0, .buf = NULL, .base = U32_MAX }
    // on count = 0. Removes any previous array.
    ds_Array(T) ds_TSlabRealloc(slab, array, count)
*/

#define DS_TSLAB_NULL            U32_MAX

struct ds_Align(DS_CACHE_LINE) ds_TSlabShared
{
    u32                     a_count_max;
    u8                      pad[DS_CACHE_LINE - sizeof(u32)];
    struct VM               vm;
    struct ds_MagazinePow2  magazine;
    u32                     length;
};

#define ds_TSlab(T) ds_TSlab_ ## T

#define DEFINE_TSLAB(T)                                                                 \
typedef struct ds_TSlab(T)                                                              \
{                                                                                       \
    struct ds_TSlabShared    shared;                                                    \
    struct T *              buf;                                                        \
    ds_Array(T) *           array_type;     /* never set: the type of ds_TSlabAdd */   \
} ds_TSlab(T)

u32     ds_TSlabSharedAlloc(struct ds_TSlabShared *shared,
                           void **external,
                           const u32 thread_count,
                           const u64 slot_size,
                           const u64 vm_size);

void    ds_TSlabSharedDealloc(struct ds_TSlabShared *shared);

void    ds_TSlabSharedFlush(struct ds_TSlabShared *shared);

struct ds_ArrayUntyped  ds_TSlabSharedAdd(struct ds_TSlabShared *shared,
                                          const u64 slot_size,
                                          const u32 count);

void    ds_TSlabSharedRemove(struct ds_TSlabShared *shared, const u64 slot_size,
                             const u32 base, const u32 length);

struct ds_ArrayUntyped  ds_TSlabSharedRealloc(struct ds_TSlabShared *shared,
                                              const u64 slot_size,
                                              const struct ds_ArrayUntyped array,
                                              const u32 count);

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
    ((union { struct ds_ArrayUntyped u; ds_TypeOf(*(_slab_).array_type) t; })           \
        { .u = ds_TSlabSharedAdd(&(_slab_).shared, sizeof((_slab_).buf[0]),             \
                                 (_count_)) }).t

#define ds_TSlabRemove(_slab_, _array_)                                                 \
    ds_TSlabSharedRemove(&(_slab_).shared, sizeof((_slab_).buf[0]), (_array_).base,     \
                         (_array_).length)

#define ds_TSlabRealloc(_slab_, _array_, _count_)                                       \
    ((union { struct ds_ArrayUntyped u; ds_TypeOf(*(_slab_).array_type) t; })           \
        { .u = ds_TSlabSharedRealloc(&(_slab_).shared, sizeof((_slab_).buf[0]),         \
                                     (struct ds_ArrayUntyped) { (_array_).buf,          \
                                                                (_array_).length,       \
                                                                (_array_).base },       \
                                     (_count_)) }).t

#endif

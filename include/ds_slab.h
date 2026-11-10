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
Slab: ds_Slab(T), arrays of T in power-of-two classes from one reserved region, handles instead of
pointers; TSlab, the thread-safe variant with per-thread free lists. Not implemented yet.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_SLAB_H__
#define __DS_SLAB_H__

#define SLAB_NODE   u32 slab_node;  // free-list, similar to Pool. On alloc, it stores the count

struct ds_SlabThread
{
    ds_CPool(u32)   class_pool_next;
    u32             count_free;         /* total number of slots free in lists, (so 64-slot slab gives += 64) */
};

//TODO Translate everything to macro
struct ds_Slab(T)
{
    SLAB_NODE;
    T * buf;
    FreeListArr?;
    u32 count_max;                      
    u32 length;
    u32 heap_allocated;
}

/* Return 1 on successful allocation, 0 on failure. */
u32         ds_SlabAlloc(mem, intial_length, initial_class_count)
/* Free slab resources, if heap-allocated. */
void        ds_SlabDealloc(pool)
/* Flush slab resources */
void        ds_PoolFlush(pool)
/* On success return (ptr_base, index_base). On failure, (NULL, U32_MAX) is returned. */
struct slot ds_SlabAdd(&slab, count);   
/* Remove slab at index. */
struct      ds_SlabRemove(&slab, base_index);
/* Remove slab at pointer. */
struct      ds_SlabRemoveAddress(&slab, ptr);

//TODO compact api
//TODO mem api
//TODO add serialize api

/*
//TODO: Design descisions:

1. Having multiple slabs disallows us to have thread-local free-lists, so any thread wishing to
access its freelists need to find its index, then thread_mem[index], at which it can access
free lists. Cost is obviously more indirection, while reward is master can re-organize lists
properly.

2. Can make growable instead of setting limit up-front by using a similar structure as TPool,
with a buffer of memory bases, >= in size, power-of-two, so we get easy lookup.

3. How do we store freelists in per-thread struct? A sorted array, or an array indexed by count?, the later seems damn easy and convinient
*/




#endif

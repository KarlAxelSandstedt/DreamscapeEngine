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
ds_queue.h
==========
Binary min-heaps of f32 priorities: the lowest priority is served first. Ties come out in an unspecified
order; priorities must not be NaN (the order is then unspecified, but no element is lost). +inf is fine.

Contents (in this order):
    minQueue        (priority, external index) elements with a handle per element, so a queued element's
                    priority can be lowered (MinQueueDecreasePriority).
    ds_MinQueue     (u32, priority) pairs without handles: push, pop, peek.

Both live in an arena (fixed length) or on the heap (fixed, or growable: the storage doubles when a push
finds the queue full). A push onto a full fixed queue is fatal. Not thread-safe.
*/

#ifndef __DS_QUEUE_H__
#define __DS_QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "ds_allocator.h"

/*
minQueue
--------
    struct minQueue queue = MinQueueAlloc(NULL, 64, GROWABLE);
    const u32 handle = MinQueuePush(&queue, cost, node);
    MinQueueDecreasePriority(&queue, handle, lower_cost);
    const u32 node = MinQueuePop(&queue);
    MinQueueDealloc(&queue);

elements[0, object_pool.count) is the heap; elements[0] is a minimum. A push returns the element's handle
(object index), valid until the element is popped or the queue flushed; a pop returns its external index.
*/

struct queueObject
{
	POOL_NODE;
	u32 	external_index;
	u32	    queue_index;
};
DEFINE_POOL(queueObject);

struct queueElement 
{
	f32	priority;
	u32 	object_index;
};

struct minQueue 
{
	ds_Pool(queueObject) 	object_pool;
	struct queueElement *	elements;
	u32			growable;
	u32			heap_allocated;	/* elements (and the pool) are ds_Alloc blocks */
};

/*
Return an empty queue of initial_length (> 0) elements on arena (fixed; growable must be 0) or on the heap
(arena == NULL). Fatal if the memory can't be allocated.
*/
struct minQueue	MinQueueAlloc(struct arena *arena, const u32 initial_length, const u32 growable);
/* Free a heap queue's memory (an arena queue's stays with the arena). */
void 		MinQueueDealloc(struct minQueue * const queue);
/* Queue external_index with priority and return its handle. Fatal if a fixed queue is full. */
u32 		MinQueuePush(struct minQueue * const queue, const f32 priority, const u32 external_index);
/* Remove a minimum and return its external index; the queue must not be empty. */
u32 		MinQueuePop(struct minQueue * const queue);
/* Lower the priority of the queued element object_index (a handle from MinQueuePush) to priority; no-op if
   priority is not lower. */
void 		MinQueueDecreasePriority(struct minQueue * const queue, const u32 object_index, const f32 priority);
/* Remove all elements; handles restart at 0. */
void 		MinQueueFlush(struct minQueue * const queue);

/*
ds_MinQueue
-----------
For when priorities never change: element[0, count) is the heap of (u = id, f = priority) pairs, element[0]
a minimum.
*/

struct ds_MinQueue 
{
	u32f32 *	element;
	u32 		count;
	u32		length;
	u32		growable;
	u32		heap_allocated;	/* element is a ds_Alloc block */
};

/*
Return an empty queue of initial_length elements on mem (fixed; growable must be 0) or on the heap
(mem == NULL). initial_length 0 gives the empty queue { 0 }, which can't grow. Fatal if the memory can't be
allocated.
*/
struct ds_MinQueue	ds_MinQueueAlloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* Return a fixed queue over all whole elements that fit in mem (possibly length 0). */
struct ds_MinQueue	ds_MinQueueAllocAll(struct arena *mem);
/* Free a heap queue's memory (an arena queue's stays with the arena). */
void 			ds_MinQueueDealloc(struct ds_MinQueue *queue);
/* Remove all elements. */
void			ds_MinQueueFlush(struct ds_MinQueue *queue);
/* Debug print of the heap array to Log. */
void 			ds_MinQueuePrint(FILE *Log, const struct ds_MinQueue *queue);
/* Queue (id, priority). Fatal if a fixed queue is full. */
void 			ds_MinQueuePush(struct ds_MinQueue *queue, const u32 id, const f32 priority);
/* Remove and return a minimum; the queue must not be empty. */
u32f32 			ds_MinQueuePop(struct ds_MinQueue *heap);
/* Return a minimum without removing it; the queue must not be empty. */
u32f32 			ds_MinQueuePeek(const struct ds_MinQueue *heap);

#ifdef __cplusplus
} 
#endif

#endif

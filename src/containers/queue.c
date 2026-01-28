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

#include <stdio.h>
#include <float.h>

#include "ds_base.h"
#include "queue.h"

/**
 * parent_index() - get queue index of parent.
 *
 * queue_index - index of child
 *
 * RETURNS: the index of the parent. If the function is called on the element first in the queue (index 0),
 * 	then an error value of U32_MAX is returned: 0/2 - ((0 AND 1) XOR 1) = 0 - (0 XOR 1) = U32_MAX.
 */
static u32 parent_index(const u32 queue_index)
{
	return queue_index / 2 - ((queue_index & 0x1) ^ 0x1);
}

/**
 * left_index() - get queue index of left child 
 *
 * queue_index - index of parent 
 */
static u32 left_index(const u32 queue_index)
{
	return (queue_index << 1) + 1;
}

/**
 * right_index() - get queue index of right child 
 *
 * queue_index - index of parent 
 */
static u32 right_index(const u32 queue_index)
{
	return (queue_index + 1) << 1;
}

/**
 * min_queue_change_elements() - Change two elements in the element array, and update their corresponding 
 * 	data' queue_index
 *
 * queue - The queue
 * i1 - queue index of element 1
 * i2 - queue index of element 2
 */
static void min_queue_change_elements(struct minQueue * const queue, const u32 i1, const u32 i2)
{
	/* Update data queue indices */
	struct queueObject *obj1 = PoolAddress(&queue->object_pool, queue->elements[i1].object_index);
	struct queueObject *obj2 = PoolAddress(&queue->object_pool, queue->elements[i2].object_index);
	obj1->queue_index = i2;
	obj2->queue_index = i1;

	/* Update priorities */
	const f32 tmp_priority = queue->elements[i1].priority;
	queue->elements[i1].priority = queue->elements[i2].priority;
	queue->elements[i2].priority = tmp_priority;

	/* update queue's data indices */
	const u32 tmp_index = queue->elements[i1].object_index;
	queue->elements[i1].object_index = queue->elements[i2].object_index;
	queue->elements[i2].object_index = tmp_index;
}

/**
 * min_queue_heapify_up() - Keep the queue coherent after a decrease of the queue_index's priority has been made.
 * 	A Decrease of the priority may destroy the coherency of the queue, as the parent should always be infront
 * 	of the children in the queue. decreasing the priority of a child may thus result in the child having a
 * 	lower priority than it's parent, so they have to be interchanged.
 *
 * queue - The queue
 * queue_index - The index of the queue element who just had it's priority decreased.
 */
static void min_queue_heapify_up(struct minQueue * const queue, u32 queue_index)
{
	u32 parent = parent_index(queue_index);

	/* Continue until parent's priority is smaller than child or the root has been reached */
	while (parent != U32_MAX && queue->elements[queue_index].priority < queue->elements[parent].priority) 
	{
		min_queue_change_elements(queue, queue_index, parent);
		queue_index = parent;
		parent = parent_index(queue_index); 
	}
}

static void recursion_done(struct minQueue * const queue, const u32 queue_index, const u32 small_priority_index);
static void recursive_call(struct minQueue * const queue, const u32 queue_index, const u32 small_priority_index);

void (*func[2])(struct minQueue * const, const u32, const u32) = { &recursion_done, &recursive_call };

/**
 * min_queue_heapify_down() - Keeps the queue coherent when the element at queue_index has had 
 * 	it's priority increased. Then the element may have a higher priority than it's children, breaking
 * 	the min-heap property.
 *
 * queue - The queue
 * queue_index - Index of the element whose priority has been increased
 */
static void min_queue_heapify_down(struct minQueue * const queue, const u32 queue_index)
{
	const u32 left = left_index(queue_index);
	const u32 right = right_index(queue_index);
	u32 smallest_priority_index = queue_index;

	if (left < queue->object_pool.count && queue->elements[left].priority < queue->elements[smallest_priority_index].priority)
		smallest_priority_index = left;
	
	if (right < queue->object_pool.count && queue->elements[right].priority < queue->elements[smallest_priority_index].priority)
		smallest_priority_index = right;
	
	/* Child had smaller priority */
	func[ ((u32) queue_index - smallest_priority_index) >> 31 ](queue, queue_index, smallest_priority_index);
}

static void recursion_done(struct minQueue * const queue, const u32 queue_index, const u32 small_priority_index)
{
	return;
}

static void recursive_call(struct minQueue * const queue, const u32 queue_index, const u32 smallest_priority_index)
{
		min_queue_change_elements(queue, queue_index, smallest_priority_index);
		/* Now child may break the min-heap property */
		min_queue_heapify_down(queue, smallest_priority_index);
}

struct minQueue MinQueueAlloc(struct arena *arena, const u32 initial_length, const u32 growable)
{
	ds_Assert(initial_length);
	ds_Assert(!arena || !growable);
	ds_StaticAssert(sizeof(u32f32) == 8, "");

	struct minQueue queue = { 0 };

	if (arena)
	{
		queue.object_pool = PoolAlloc(arena, initial_length, struct queueObject, !GROWABLE);
		queue.elements = ArenaPush(arena, initial_length * sizeof(struct queueElement));
	}
	else
	{
		queue.object_pool = PoolAlloc(NULL, initial_length, struct queueObject, !GROWABLE);
		queue.elements = ds_Alloc(&queue.mem_elements, queue.object_pool.length * sizeof(struct queueElement), queue.object_pool.mem_slot.huge_pages);
	}

	queue.growable = growable;

	if (queue.object_pool.length == 0 || queue.elements == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate min queue, exiting.");
		FatalCleanupAndExit();
	}
		
	return queue;
}

void MinQueueDealloc(struct minQueue * const queue)
{
	if (queue->mem_elements.address)
	{
		PoolDealloc(&queue->object_pool);
		ds_Free(&queue->mem_elements);
	}
}

u32 MinQueuePop(struct minQueue * const queue)
{
	ds_AssertString(queue->object_pool.count > 0, "Queue should have elements to extract\n");
	struct queueObject *obj = PoolAddress(&queue->object_pool, queue->elements[0].object_index);
	const u32 external_index = obj->external_index;
	queue->elements[0].priority = FLT_MAX;

	/* Keep the array of elements compact */
	min_queue_change_elements(queue, 0, queue->object_pool.count-1);
	/* Check coherence of the queue from the start */
	min_queue_heapify_down(queue, 0);

	PoolRemoveAddress(&queue->object_pool, obj);

	return external_index;
}

u32 MinQueuePush(struct minQueue * const queue, const f32 priority, const u32 external_index)
{
	const u32 old_length = queue->object_pool.length;
	const u32 queue_index = queue->object_pool.count;
	struct slot slot = PoolAdd(&queue->object_pool);
	if (old_length != queue->object_pool.length)
	{
		ds_Assert(queue->growable);
		queue->elements = ds_Realloc(&queue->mem_elements, queue->object_pool.length*sizeof(struct queueElement));
		if (queue->elements == NULL)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to reallocate min queue, exiting.");
			FatalCleanupAndExit();
		}
	}

	queue->elements[queue_index].priority = priority;
	queue->elements[queue_index].object_index = slot.index;

	struct queueObject *object = slot.address;
	object->external_index = external_index;
	object->queue_index = queue_index;

	min_queue_heapify_up(queue, queue_index);

	return slot.index;
}

void MinQueueDecreasePriority(struct minQueue * const queue, const u32 object_index, const f32 priority)
{
	ds_AssertString(object_index < queue->object_pool.count, "Queue index should be withing queue bounds");

	struct queueObject *obj = PoolAddress(&queue->object_pool, object_index);
	if (priority < queue->elements[obj->queue_index].priority) 
	{
		queue->elements[obj->queue_index].priority = priority;
		min_queue_heapify_up(queue, obj->queue_index);
	}
}

void MinQueueFlush(struct minQueue * const queue)
{
	PoolFlush(&queue->object_pool);
}

static void min_queue_fixed_heapify_up(struct minQueueFixed * const queue, u32 queue_index)
{
	u32 parent = parent_index(queue_index);

	/* Continue until parent's priority is smaller than child or the root has been reached */
	while (parent != U32_MAX && queue->element[queue_index].f < queue->element[parent].f) 
	{
		const u32f32 tuple = queue->element[queue_index];
		queue->element[queue_index] = queue->element[parent];
		queue->element[parent] = tuple; 
		queue_index = parent;
		parent = parent_index(queue_index); 
	}
}

static void queue_fixed_recursion_done(struct minQueueFixed * const queue, const u32 queue_index, const u32 small_priority_index);
static void queue_fixed_recursive_call(struct minQueueFixed * const queue, const u32 queue_index, const u32 small_priority_index);

void (*queue_fixed_func[2])(struct minQueueFixed * const, const u32, const u32) = { &queue_fixed_recursion_done, &queue_fixed_recursive_call };

static void min_queue_fixed_heapify_down(struct minQueueFixed * const queue, const u32 queue_index)
{
	const u32 left = left_index(queue_index);
	const u32 right = right_index(queue_index);
	u32 smallest_priority_index = queue_index;

	if (left < queue->count && queue->element[left].f < queue->element[smallest_priority_index].f)
		smallest_priority_index = left;
	
	if (right < queue->count && queue->element[right].f < queue->element[smallest_priority_index].f)
		smallest_priority_index = right;
	
	/* Child had smaller priority */
	queue_fixed_func[ ((u32) queue_index - smallest_priority_index) >> 31 ](queue, queue_index, smallest_priority_index);
}

static void queue_fixed_recursion_done(struct minQueueFixed * const queue, const u32 queue_index, const u32 small_priority_index)
{
	return;
}

static void queue_fixed_recursive_call(struct minQueueFixed * const queue, const u32 queue_index, const u32 smallest_priority_index)
{
	const u32f32 tuple = queue->element[queue_index];
	queue->element[queue_index] = queue->element[smallest_priority_index];
	queue->element[smallest_priority_index] = tuple;
	min_queue_fixed_heapify_down(queue, smallest_priority_index);
}


struct minQueueFixed MinQueueFixedAlloc(struct arena *mem, const u32 initial_length, const u32 growable)
{
	ds_Assert(!growable || !mem);
	if (!initial_length) { return (struct minQueueFixed) { 0 }; }

	struct minQueueFixed queue =
	{
		.count = 0,
		.growable = growable,
	};

	if (mem)
	{
		queue.element = ArenaPush(mem, initial_length*sizeof(u32f32));
		queue.length = initial_length;
	}
	else
	{
		queue.element = ds_Alloc(&queue.mem_element, initial_length*sizeof(u32f32), HUGE_PAGES);
		queue.length = queue.mem_element.size / sizeof(u32f32);	
	}

	if (queue.element == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate min_queue_fixed memory, exiting.");
		FatalCleanupAndExit();
	}

	return queue;
}

struct minQueueFixed MinQueueFixedAllocAll(struct arena *mem)
{
	ds_Assert(mem);

	struct minQueueFixed queue = { 0 };
	struct memArray arr = ArenaPushAlignedAll(mem, sizeof(u32f32), 4);
	queue.length = arr.len;
	queue.element = arr.addr;

	return queue;
}

void MinQueueFixedDealloc(struct minQueueFixed *queue)
{
	if (queue->mem_element.address)
	{
		ds_Free(&queue->mem_element);
	}
}

void MinQueueFixedFlush(struct minQueueFixed *queue)
{
	queue->count = 0;
}

void MinQueueFixedPrint(FILE *Log, const struct minQueueFixed *queue)
{
	fprintf(Log, "min queue_fixed %p: ", queue);
	fprintf(Log, "{ ");
	for (u32 i = 0; i < queue->count; ++i)
	{
		fprintf(Log, "(%u,%f), ", queue->element[i].u, queue->element[i].f);
	}
	fprintf(Log, "}\n");


}

void MinQueueFixedPush(struct minQueueFixed *queue, const u32 id, const f32 priority)
{
	if (queue->count == queue->length)
	{
		if (queue->growable)
		{
			queue->length *= 2;
			queue->element = ds_Realloc(&queue->mem_element, queue->length*sizeof(u32f32));
			if (queue->element == NULL)
			{
				LogString(T_SYSTEM, S_FATAL, "Failed to reallocate min_queue_fixed memory, exiting.");
				FatalCleanupAndExit();
			}
		}	
		else
		{
			return;
		}
	}

	const u32 i = queue->count;
	queue->count += 1;
	queue->element[i].f = priority;
	queue->element[i].u = id;
	min_queue_fixed_heapify_up(queue, i);
}

u32f32 MinQueueFixedPop(struct minQueueFixed *queue)
{
	ds_AssertString(queue->count > 0, "Heap should have elements to extract\n");
	queue->count -= 1;

	const u32f32 tuple = queue->element[0];
	queue->element[0] = queue->element[queue->count];
	min_queue_fixed_heapify_down(queue, 0);

	return tuple;
}

u32f32 	MinQueueFixedPeek(const struct minQueueFixed *queue)
{
	ds_AssertString(queue->count > 0, "Heap should have elements to extract\n");
	return queue->element[0];
}

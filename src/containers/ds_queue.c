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

#include "ds_base.h"
#include "ds_queue.h"


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
 * mqa_change_elements() - Change two elements in the element array, and update their corresponding 
 * 	data' queue_index
 *
 * queue - The queue
 * i1 - queue index of element 1
 * i2 - queue index of element 2
 */
static void mqa_change_elements(struct MQA * const queue, const u32 i1, const u32 i2)
{
	/* Update data queue indices */
	struct MQAObject *obj1 = queue->object_pool.buf + queue->elements[i1].object_index;
	struct MQAObject *obj2 = queue->object_pool.buf + queue->elements[i2].object_index;
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
 * mqa_heapify_up() - Keep the queue coherent after a decrease of the queue_index's priority has been made.
 * 	A Decrease of the priority may destroy the coherency of the queue, as the parent should always be infront
 * 	of the children in the queue. decreasing the priority of a child may thus result in the child having a
 * 	lower priority than it's parent, so they have to be interchanged.
 *
 * queue - The queue
 * queue_index - The index of the queue element who just had it's priority decreased.
 */
static void mqa_heapify_up(struct MQA * const queue, u32 queue_index)
{
	u32 parent = parent_index(queue_index);

	/* Continue until parent's priority is smaller than child or the root has been reached */
	while (parent != U32_MAX && queue->elements[queue_index].priority < queue->elements[parent].priority) 
	{
		mqa_change_elements(queue, queue_index, parent);
		queue_index = parent;
		parent = parent_index(queue_index); 
	}
}

static void mqa_recursion_done(struct MQA * const queue, const u32 queue_index, const u32 small_priority_index);
static void mqa_recursive_call(struct MQA * const queue, const u32 queue_index, const u32 small_priority_index);

static void (*mqa_heapify_down_step[2])(struct MQA * const, const u32, const u32) = { &mqa_recursion_done, &mqa_recursive_call };

/**
 * mqa_heapify_down() - Keeps the queue coherent when the element at queue_index has had 
 * 	it's priority increased. Then the element may have a higher priority than it's children, breaking
 * 	the min-heap property.
 *
 * queue - The queue
 * queue_index - Index of the element whose priority has been increased
 */
static void mqa_heapify_down(struct MQA * const queue, const u32 queue_index)
{
	const u32 left = left_index(queue_index);
	const u32 right = right_index(queue_index);
	u32 smallest_priority_index = queue_index;

	if (left < queue->object_pool.count && queue->elements[left].priority < queue->elements[smallest_priority_index].priority)
		smallest_priority_index = left;
	
	if (right < queue->object_pool.count && queue->elements[right].priority < queue->elements[smallest_priority_index].priority)
		smallest_priority_index = right;
	
	/* Child had smaller priority */
	mqa_heapify_down_step[ ((u32) queue_index - smallest_priority_index) >> 31 ](queue, queue_index, smallest_priority_index);
}

static void mqa_recursion_done(struct MQA * const queue, const u32 queue_index, const u32 small_priority_index)
{
	return;
}

static void mqa_recursive_call(struct MQA * const queue, const u32 queue_index, const u32 smallest_priority_index)
{
		mqa_change_elements(queue, queue_index, smallest_priority_index);
		/* Now child may break the min-heap property */
		mqa_heapify_down(queue, smallest_priority_index);
}

struct MQA MQAAlloc(struct arena *arena, const u32 initial_length, const u32 growable)
{
	ds_Assert(initial_length);
	ds_Assert(!arena || !growable);
	ds_StaticAssert(sizeof(u32f32) == 8, "");

	struct MQA queue = { 0 };

	if (arena)
	{
		ds_PoolAlloc(arena, queue.object_pool, initial_length, !GROWABLE);
		queue.elements = ArenaPush(arena, initial_length * sizeof(struct MQAElement));
	}
	else
	{
		ds_PoolAlloc(NULL, queue.object_pool, initial_length, growable);
		queue.elements = ds_Alloc(queue.object_pool.length * sizeof(struct MQAElement), sizeof(struct MQAElement));
		queue.heap_allocated = 1;
	}

	queue.growable = growable;

	if (queue.object_pool.length == 0 || queue.elements == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate min queue, exiting.");
		FatalCleanupAndExit();
	}
		
	return queue;
}

void MQADealloc(struct MQA * const queue)
{
	if (queue->heap_allocated)
	{
		ds_PoolDealloc(queue->object_pool);
		ds_Free(queue->elements);
		queue->heap_allocated = 0;
	}
}

u32 MQAPop(struct MQA * const queue)
{
	ds_AssertString(queue->object_pool.count > 0, "Queue should have elements to extract\n");
    const u32 index = queue->elements[0].object_index;
	struct MQAObject *obj = queue->object_pool.buf + index;
	const u32 external_index = obj->external_index;

	/*
	 * move the last element to the root and drop the popped one from the count before heapifying: it must
	 * not take part (a sentinel priority could tie with or lose to +inf and NaN priorities)
	 */
	mqa_change_elements(queue, 0, queue->object_pool.count-1);
	ds_PoolRemove(queue->object_pool, index);
	mqa_heapify_down(queue, 0);

	return external_index;
}

u32 MQAPush(struct MQA * const queue, const f32 priority, const u32 external_index)
{
	const u32 old_length = queue->object_pool.length;
	const u32 queue_index = queue->object_pool.count;
	struct slot slot = ds_PoolAdd(queue->object_pool);
	if (!slot.address)
	{
		LogString(T_SYSTEM, S_FATAL, "Non-growable min queue full in MQAPush, exiting.");
		FatalCleanupAndExit();
	}

	if (old_length != queue->object_pool.length)
	{
		ds_Assert(queue->growable);
		queue->elements = ds_Realloc(queue->elements, old_length*sizeof(struct MQAElement),
			queue->object_pool.length*sizeof(struct MQAElement), sizeof(struct MQAElement));
	}

	queue->elements[queue_index].priority = priority;
	queue->elements[queue_index].object_index = slot.index;

	struct MQAObject *object = slot.address;
	object->external_index = external_index;
	object->queue_index = queue_index;

	mqa_heapify_up(queue, queue_index);

	return slot.index;
}

void MQADecreasePriority(struct MQA * const queue, const u32 object_index, const f32 priority)
{
	/* object indices are pool slots: up to count_max, with holes left by pops */
	ds_AssertString(object_index < queue->object_pool.count_max && ds_PoolSlotAllocated(queue->object_pool.buf + object_index),
		"MQADecreasePriority: object_index is not in the queue");

	struct MQAObject *obj = queue->object_pool.buf + object_index;
	if (priority < queue->elements[obj->queue_index].priority) 
	{
		queue->elements[obj->queue_index].priority = priority;
		mqa_heapify_up(queue, obj->queue_index);
	}
}

void MQAFlush(struct MQA * const queue)
{
	ds_PoolFlush(queue->object_pool);
}

static void mq_heapify_up(struct MQ * const queue, u32 queue_index)
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

static void mq_recursion_done(struct MQ * const queue, const u32 queue_index, const u32 small_priority_index);
static void mq_recursive_call(struct MQ * const queue, const u32 queue_index, const u32 small_priority_index);

static void (*mq_heapify_down_step[2])(struct MQ * const, const u32, const u32) = { &mq_recursion_done, &mq_recursive_call };

static void mq_heapify_down(struct MQ * const queue, const u32 queue_index)
{
	const u32 left = left_index(queue_index);
	const u32 right = right_index(queue_index);
	u32 smallest_priority_index = queue_index;

	if (left < queue->count && queue->element[left].f < queue->element[smallest_priority_index].f)
		smallest_priority_index = left;
	
	if (right < queue->count && queue->element[right].f < queue->element[smallest_priority_index].f)
		smallest_priority_index = right;
	
	/* Child had smaller priority */
	mq_heapify_down_step[ ((u32) queue_index - smallest_priority_index) >> 31 ](queue, queue_index, smallest_priority_index);
}

static void mq_recursion_done(struct MQ * const queue, const u32 queue_index, const u32 small_priority_index)
{
	return;
}

static void mq_recursive_call(struct MQ * const queue, const u32 queue_index, const u32 smallest_priority_index)
{
	const u32f32 tuple = queue->element[queue_index];
	queue->element[queue_index] = queue->element[smallest_priority_index];
	queue->element[smallest_priority_index] = tuple;
	mq_heapify_down(queue, smallest_priority_index);
}


struct MQ MQAlloc(struct arena *mem, const u32 initial_length, const u32 growable)
{
	ds_Assert(!growable || !mem);
	if (!initial_length) { return (struct MQ) { 0 }; }

	struct MQ queue =
	{
		.count = 0,
		.growable = growable,
	};

	queue.element = (mem)
		? ArenaPush(mem, initial_length*sizeof(u32f32))
		: ds_Alloc(initial_length*sizeof(u32f32), sizeof(u32f32));
	queue.length = initial_length;
	queue.heap_allocated = (mem == NULL);

	if (queue.element == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate MQ memory, exiting.");
		FatalCleanupAndExit();
	}

	return queue;
}

struct MQ MQAllocAll(struct arena *mem)
{
	ds_Assert(mem);

	struct MQ queue = { 0 };
	struct memArray arr = ArenaPushAlignedAll(mem, sizeof(u32f32), 4);
	queue.length = arr.len;
	queue.element = arr.addr;

	return queue;
}

void MQDealloc(struct MQ *queue)
{
	if (queue->heap_allocated)
	{
		ds_Free(queue->element);
		queue->heap_allocated = 0;
	}
}

void MQFlush(struct MQ *queue)
{
	queue->count = 0;
}

void MQPrint(FILE *Log, const struct MQ *queue)
{
	fprintf(Log, "MQ %p: ", queue);
	fprintf(Log, "{ ");
	for (u32 i = 0; i < queue->count; ++i)
	{
		fprintf(Log, "(%u,%f), ", queue->element[i].u, queue->element[i].f);
	}
	fprintf(Log, "}\n");


}

void MQPush(struct MQ *queue, const u32 id, const f32 priority)
{
	if (queue->count == queue->length)
	{
		if (queue->growable)
		{
			queue->element = ds_Realloc(queue->element, queue->length*sizeof(u32f32), 2*queue->length*sizeof(u32f32), sizeof(u32f32));
			queue->length *= 2;
		}	
		else
		{
			/* as MQAPush: callers (bvh raycasts) must not silently lose elements */
			LogString(T_SYSTEM, S_FATAL, "Non-growable MQ full in MQPush, exiting.");
			FatalCleanupAndExit();
		}
	}

	const u32 i = queue->count;
	queue->count += 1;
	queue->element[i].f = priority;
	queue->element[i].u = id;
	mq_heapify_up(queue, i);
}

u32f32 MQPop(struct MQ *queue)
{
	ds_AssertString(queue->count > 0, "Heap should have elements to extract\n");
	queue->count -= 1;

	const u32f32 tuple = queue->element[0];
	queue->element[0] = queue->element[queue->count];
	mq_heapify_down(queue, 0);

	return tuple;
}

u32f32 	MQPeek(const struct MQ *queue)
{
	ds_AssertString(queue->count > 0, "Heap should have elements to extract\n");
	return queue->element[0];
}

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

#include "ds_base.h"
#include "hierarchy_index.h"

#include <string.h>

#define hi_ParentPtr(hi, index)		((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->parent_offset))
#define hi_NextPtr(hi, index)		((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->next_offset))
#define hi_PrevPtr(hi, index)		((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->prev_offset))
#define hi_FirstPtr(hi, index)		((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->first_offset))
#define hi_LastPtr(hi, index)		((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->last_offset))
#define hi_ChildCountPtr(hi, index)	((u32 *) ((hi)->pool.buf + (index)*(hi)->pool.slot_size + (hi)->child_count_offset))

struct hi hi_AllocInternal(struct arena *mem
			, const u32 length
			, const u64 data_size
			, const u32 growable
			, const u32 slot_allocation_offset	
			, const u32 parent_offset	
			, const u32 next_offset	
			, const u32 prev_offset	
			, const u32 first_offset	
			, const u32 last_offset	
			, const u32 child_count_offset)
{
	ds_Assert(length > 0);

	struct hi hi = 
	{  
		.parent_offset = parent_offset,
		.next_offset = next_offset,
		.prev_offset = prev_offset,
		.first_offset = first_offset,
		.last_offset = last_offset,
		.child_count_offset = child_count_offset,
	};
	hi.pool = PoolAllocInternal(mem, length, data_size, slot_allocation_offset, U64_MAX, growable);
	if (hi.pool.buf)
	{
		const u32 root_stub = (u32) PoolAdd(&hi.pool).index;
		*hi_ParentPtr(&hi, root_stub) = HI_NULL_INDEX;
		*hi_NextPtr(&hi, root_stub) = HI_NULL_INDEX;
		*hi_PrevPtr(&hi, root_stub) = HI_NULL_INDEX;
		*hi_FirstPtr(&hi, root_stub) = HI_NULL_INDEX;
		*hi_LastPtr(&hi, root_stub) = HI_NULL_INDEX;
		*hi_ChildCountPtr(&hi, root_stub) = 0;

		const u32 orphan_stub = (u32) PoolAdd(&hi.pool).index;
		*hi_ParentPtr(&hi, orphan_stub) = HI_NULL_INDEX;
		*hi_NextPtr(&hi, orphan_stub) = HI_NULL_INDEX;
		*hi_PrevPtr(&hi, orphan_stub) = HI_NULL_INDEX;
		*hi_FirstPtr(&hi, orphan_stub) = HI_NULL_INDEX;
		*hi_LastPtr(&hi, orphan_stub) = HI_NULL_INDEX;
		*hi_ChildCountPtr(&hi, orphan_stub) = 0;

		ds_Assert(root_stub == HI_ROOT_STUB_INDEX);
		ds_Assert(orphan_stub == HI_ORPHAN_STUB_INDEX);
	}

	return hi;
}

void hi_Dealloc(struct hi *hi)
{
	PoolDealloc(&hi->pool);
}
void hi_Flush(struct hi *hi)
{
	PoolFlush(&hi->pool);

	const u32 root_stub = PoolAdd(&hi->pool).index;
	*hi_ParentPtr(hi, root_stub) = HI_NULL_INDEX;
	*hi_NextPtr(hi, root_stub) = HI_NULL_INDEX;
	*hi_PrevPtr(hi, root_stub) = HI_NULL_INDEX;
	*hi_FirstPtr(hi, root_stub) = HI_NULL_INDEX;
	*hi_LastPtr(hi, root_stub) = HI_NULL_INDEX;
	*hi_ChildCountPtr(hi, root_stub) = 0;

	const u32 orphan_stub = PoolAdd(&hi->pool).index;
	*hi_ParentPtr(hi, orphan_stub) = HI_NULL_INDEX;
	*hi_NextPtr(hi, orphan_stub) = HI_NULL_INDEX;
	*hi_PrevPtr(hi, orphan_stub) = HI_NULL_INDEX;
	*hi_FirstPtr(hi, orphan_stub) = HI_NULL_INDEX;
	*hi_LastPtr(hi, orphan_stub) = HI_NULL_INDEX;
	*hi_ChildCountPtr(hi, orphan_stub) = 0;

	ds_Assert(root_stub == HI_ROOT_STUB_INDEX);
	ds_Assert(orphan_stub == HI_ORPHAN_STUB_INDEX);
}

struct slot hi_Add(struct hi *hi, const u32 parent_index)
{
	ds_Assert(parent_index <= hi->pool.count_max);

	struct slot new = PoolAdd(&hi->pool);
	if (new.index == U32_MAX)
	{
		return (struct slot) { .index = 0, .address = NULL };
	}

	u32 *parent_last = hi_LastPtr(hi, parent_index);
	*hi_ParentPtr(hi, new.index) = parent_index;
	*hi_PrevPtr(hi, new.index) = *parent_last;
	*hi_NextPtr(hi, new.index) = HI_NULL_INDEX;
	*hi_FirstPtr(hi, new.index) = HI_NULL_INDEX;
	*hi_LastPtr(hi, new.index) = HI_NULL_INDEX;
	*hi_ChildCountPtr(hi, new.index) = 0;

	*hi_ChildCountPtr(hi, parent_index) += 1;
	if (*parent_last)
	{
		ds_Assert(*hi_ParentPtr(hi, *parent_last) == parent_index);
		ds_Assert(hi_NextPtr(hi, *parent_last) == HI_NULL_INDEX);
		*hi_NextPtr(hi, *parent_last) = new.index;
		*hi_LastPtr(hi, parent_index) = new.index;
	}
	else
	{
		*hi_FirstPtr(hi, parent_index) = new.index;
		*parent_last = new.index;
	}

	return new;
}

static void internal_remove_recursive(struct hi *hi, const u32 root)
{
	const u32 first = *hi_FirstPtr(hi, root);
	const u32 next = *hi_NextPtr(hi, root);

	if (first)
	{
		internal_remove_recursive(hi, first);	
	}

	if (next)
	{
		internal_remove_recursive(hi, next);	
	}

	PoolRemove(&hi->pool, root);
}

static void internal_hierarchy_index_remove_sub_hierarchy_recursive(struct hi *hi, const u32 root)
{
	const u32 first = *hi_FirstPtr(hi, root);
	const u32 next = *hi_NextPtr(hi, root);

	if (first)
	{
		internal_remove_recursive(hi, first);	
	}

	if (next)
	{
		internal_remove_recursive(hi, next);	
	}
}

void hi_Remove(struct arena *tmp, struct hi *hi, const u32 node)
{
	ds_Assert(0 < node && node <= hi->pool.count_max);

	const u32 first = *hi_FirstPtr(hi, node);
	const u32 next = *hi_NextPtr(hi, node);
	const u32 prev = *hi_PrevPtr(hi, node);
	ArenaPushRecord(tmp);
	/* remove any nodes it the node's sub-hierarchy */
	if (first)
	{
		struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), sizeof(u32));
		u32 *stack = arr.addr;
		if (stack)
		{
			u32 sc = 1;
			stack[0] = first;
			while (sc--)
			{
				const u32 sub_index = stack[sc];
				const u32 sub_first = *hi_FirstPtr(hi, sub_index);
				const u32 sub_next = *hi_NextPtr(hi, sub_index);
				if (sub_first)
				{
					stack[sc++] = sub_first;
				}

				if (sub_next)
				{
					if (sc == arr.len)
					{
						LogString(T_SYSTEM, S_FATAL, "Stack OOM in hi_ApplyCustomFreeAndRemove");
						FatalCleanupAndExit();
					}
					stack[sc++] = sub_next;
				}

				PoolRemove(&hi->pool, sub_index);
			}
		}
		else
		{
			internal_hierarchy_index_remove_sub_hierarchy_recursive(hi, node);
		}
	}
	ArenaPopRecord(tmp);
	
	/* node is not a first or last child of its parent */
	if (prev && next)
	{
		*hi_NextPtr(hi, prev) = next;
		*hi_PrevPtr(hi, next) = prev;
	}
	else
	{
		const u32 parent = *hi_ParentPtr(hi, node);
		u32 *parent_first = hi_FirstPtr(hi, parent);
		u32 *parent_last = hi_LastPtr(hi, parent);

		*hi_ChildCountPtr(hi, parent) -= 1;
		/* node is an only child */
		if (*parent_first == *parent_last)
		{
			*parent_first = HI_NULL_INDEX;
			*parent_last = HI_NULL_INDEX;
		}
		else if (*parent_first == node)
		{
			*parent_first = next;
			*hi_PrevPtr(hi, next) = HI_NULL_INDEX;
			ds_Assert(*hi_ParentPtr(hi, next) == parent);
		}
		else
		{
			ds_Assert(*parent_last == node);
			*parent_last = prev;

			ds_Assert(*hi_NextPtr(hi, prev) == node);
			*hi_NextPtr(hi, prev) = HI_NULL_INDEX;
		}
	}

	PoolRemove(&hi->pool, node);
}

void hi_AdoptNodeExclusive(struct hi *hi, const u32 node, const u32 new_parent)
{
	ds_Assert(new_parent <= hi->pool.count_max);

	const u32 old_parent = *hi_ParentPtr(hi, node);
	const u32 next = *hi_NextPtr(hi, node);
	const u32 prev = *hi_PrevPtr(hi, node);
	const u32 first = *hi_FirstPtr(hi, node);
	const u32 last = *hi_LastPtr(hi, node);

	*hi_ChildCountPtr(hi, old_parent) += *hi_ChildCountPtr(hi, node) - 1;
	if (*hi_FirstPtr(hi, old_parent) == *hi_LastPtr(hi, old_parent))
	{
		*hi_PrevPtr(hi, next) = prev;
		*hi_NextPtr(hi, prev) = next;
		*hi_FirstPtr(hi, old_parent) = first;
		*hi_LastPtr(hi, old_parent) = last;
	}
	else if (*hi_FirstPtr(hi, old_parent) == node)
	{
		*hi_PrevPtr(hi, next) = last;
		if (first)
		{
			*hi_FirstPtr(hi, old_parent) = first;
			*hi_NextPtr(hi, last) = next;
		}
		else
		{
			*hi_FirstPtr(hi, old_parent) = next;
		}
	}
	else if (*hi_LastPtr(hi, old_parent) == node)
	{
		*hi_NextPtr(hi, prev) = first;	
		if (last)
		{
			*hi_LastPtr(hi, old_parent) = last;
			*hi_PrevPtr(hi, first) = prev;
		}
		else
		{
			*hi_LastPtr(hi, old_parent) = prev;
		}
	}
	else
	{
		if (first)
		{
			*hi_NextPtr(hi, prev) = first;
			*hi_PrevPtr(hi, next) = last;
			*hi_PrevPtr(hi, first) = prev;
			*hi_NextPtr(hi, last) = next;
		}
		else
		{
			*hi_PrevPtr(hi, next) = prev;
			*hi_NextPtr(hi, prev) = next;
		}
	}

	for (u32 child = first; child != HI_NULL_INDEX; child = *hi_NextPtr(hi, child))
	{
		*hi_ParentPtr(hi, child) = old_parent;
	}

	*hi_ChildCountPtr(hi, new_parent) += 1;
	*hi_ChildCountPtr(hi, node) = 0;

	const u32 old_last = *hi_LastPtr(hi, new_parent);
	*hi_ParentPtr(hi, node) = new_parent;
	*hi_PrevPtr(hi, node) = old_last;
	*hi_NextPtr(hi, node) = HI_NULL_INDEX;
	*hi_FirstPtr(hi, node) = HI_NULL_INDEX;
	*hi_LastPtr(hi, node) = HI_NULL_INDEX;

	if (old_last)
	{
		ds_Assert(*hi_ParentPtr(hi, old_last) == new_parent);
		ds_Assert(*hi_NextPtr(hi, old_last) == HI_NULL_INDEX);
		*hi_NextPtr(hi, old_last) = node;
		*hi_LastPtr(hi, new_parent) = node;
	}
	else
	{
		*hi_FirstPtr(hi, new_parent) = node;
		*hi_LastPtr(hi, new_parent) = node;
	}
}

void hi_AdoptNode(struct hi *hi, const u32 node, const u32 new_parent)
{
	ds_Assert(new_parent <= hi->pool.count_max);

	const u32 old_parent = *hi_ParentPtr(hi, node);
	const u32 next = *hi_NextPtr(hi, node);
	const u32 prev = *hi_PrevPtr(hi, node);

	*hi_ChildCountPtr(hi, old_parent) -= 1;
	*hi_PrevPtr(hi, next) = prev;
	*hi_NextPtr(hi, prev) = next;

	u32 *old_first = hi_FirstPtr(hi, old_parent);
	u32 *old_last = hi_LastPtr(hi, old_parent);
	if (*old_first == *old_last)
	{
		*old_first = HI_NULL_INDEX;
		*old_last = HI_NULL_INDEX;
	}
	else if (*old_first == node)
	{
		*old_first = next;
	}
	else if (*old_last == node)
	{
		*old_last = prev;
	}

	*hi_ChildCountPtr(hi, new_parent) += 1;

	
	u32 *new_first = hi_FirstPtr(hi, new_parent);
	u32 *new_last = hi_LastPtr(hi, new_parent);

	*hi_ParentPtr(hi, node) = new_parent;
	*hi_PrevPtr(hi, node) = *new_last;
	*hi_NextPtr(hi, node) = HI_NULL_INDEX;
	if (*new_last)
	{
		ds_Assert(*hi_ParentPtr(hi, *new_last) == new_parent);
		ds_Assert(*hi_NextPtr(hi, *new_last) == HI_NULL_INDEX);
		*hi_NextPtr(hi, *new_last) = node;
		*new_last = node;
	}
	else
	{
		*new_first = node;
		*new_last = node;
	}
}

void hi_ApplyCustomFreeAndRemove(struct arena *tmp, struct hi *hi, const u32 node, void (*custom_free)(const struct hi *hi, const u32 index, void *data), void *data)
{
	ds_Assert(0 < node && node <= hi->pool.count_max);

	ArenaPushRecord(tmp);
	const u32 first = *hi_FirstPtr(hi, node);
	/* remove any nodes it the node's sub-hierarchy */
	if (first)
	{
		struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), sizeof(u32));
		u32 *stack = arr.addr;
		if (stack)
		{
			u32 sc = 1;
			stack[0] = first;
			while (sc)
			{
				const u32 sub_node = stack[--sc];
				const u32 sub_first = *hi_FirstPtr(hi, sub_node);
				const u32 sub_next = *hi_NextPtr(hi, sub_node);
				if (sub_first)
				{
					stack[sc++] = sub_first;
				}

				if (sub_next)
				{
					if (sc == arr.len)
					{
						LogString(T_SYSTEM, S_FATAL, "Stack OOM in hi_ApplyCustomFreeAndRemove");
						FatalCleanupAndExit();
					}
					stack[sc++] = sub_next;
				}

				custom_free(hi, sub_node, data);
				PoolRemove(&hi->pool, sub_node);
			}
		}
		else
		{
			LogString(T_SYSTEM, S_FATAL, "Stack OOM in hi_ApplyCustomFreeAndRemove");
			FatalCleanupAndExit();
		}
	}
	ArenaPopRecord(tmp);
	
	const u32 parent = *hi_ParentPtr(hi, node);
	const u32 prev = *hi_PrevPtr(hi, node);
	const u32 next = *hi_NextPtr(hi, node);
	*hi_ChildCountPtr(hi, parent) -= 1;
	
	ds_Assert(next == HI_NULL_INDEX || *hi_PrevPtr(hi, next) == node);
	ds_Assert(prev == HI_NULL_INDEX || *hi_NextPtr(hi, prev) == node);
	ds_Assert(next == HI_NULL_INDEX || *hi_ParentPtr(hi, next) == parent);
	ds_Assert(prev == HI_NULL_INDEX || *hi_ParentPtr(hi, prev) == parent);

	*hi_NextPtr(hi, prev) = next;
	*hi_PrevPtr(hi, next) = prev;

	u32 *parent_first = hi_FirstPtr(hi, parent);
	u32 *parent_last = hi_LastPtr(hi, parent);

	/* node is an only child */
	if (*parent_first == *parent_last)
	{
		*parent_first = HI_NULL_INDEX;
		*parent_last = HI_NULL_INDEX;
	}
	else if (*parent_first == node)
	{
		*parent_first = next;
	}
	else if (*parent_last == node)
	{
		*parent_last = prev;
	}

	custom_free(hi, node, data);
	PoolRemove(&hi->pool, node);
}

void *hi_Address(const struct hi *hi, const u32 node)
{
	ds_Assert(node <= hi->pool.count_max);
	return PoolAddress(&hi->pool, node);
}

struct hiIterator hi_IteratorAlloc(struct arena *mem, struct hi *hi, const u32 root)
{
	ds_Assert(mem);
	ArenaPushRecord(mem);

	struct hiIterator it;
	it.hi = hi;

	struct memArray alloc = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));
	it.stack_len = alloc.len;
	it.stack = alloc.addr;

	if (it.stack == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Stack OOM in hi_IteratorAlloc");
		FatalCleanupAndExit();
	}

	ds_Assert(root != HI_NULL_INDEX);
	it.stack[0] = HI_NULL_INDEX;
	it.stack[1] = root;	
	it.count = 1;

	return it;
}

u32 hi_IteratorPeek(struct hiIterator *it)
{
	ds_Assert(it->count);
	return it->stack[it->count];
}

u32 hi_IteratorNextDf(struct hiIterator *it)
{
	ds_Assert(it->count);
	const u32 node = it->stack[it->count];

	u32 push[2];
	u64 push_count = 0;

	const u32 first = *hi_FirstPtr(it->hi, node);
	const u32 next = *hi_NextPtr(it->hi, node);

	if (next)
	{
		push[push_count++] = next;
	}
	
	if (first)
	{
		push[push_count++] = first;
	}

	if (push_count == 2)
	{
		/* count doesnt take initial stub into account {  (1 + it->count - 1 + push_count) } */
		if (it->count + push_count > it->stack_len)
		{
			LogString(T_SYSTEM, S_FATAL, "Stack OOM in hi_IteratorNextDf");
			FatalCleanupAndExit();
		}

		it->stack[it->count + 0] = push[0];
		it->stack[it->count + 1] = push[1];
	}
	else if (push_count == 1)
	{
		it->stack[it->count + 0] = push[0];
	}

	it->count = it->count + push_count -1;

	return node;
}

void hi_IteratorSkip(struct hiIterator *it)
{
	ds_Assert(it->count);
	const u32 node = it->stack[it->count];
	const u32 next = *hi_NextPtr(it->hi, node);
	if (next)
	{
		it->stack[it->count] = next;
	}
	else
	{
		it->count -= 1;
	}
}

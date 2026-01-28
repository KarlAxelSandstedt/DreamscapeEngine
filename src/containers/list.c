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

#include <string.h>

#include "ds_base.h"
#include "list.h"

struct ll ll_InitInternal(const u64 slot_size, const u64 slot_state_offset)
{
	struct ll ll =  
	{ 
		.count = 0,
	       	.first = LL_NULL, 
		.last = LL_NULL, 
		.slot_size = slot_size, 
		.slot_state_offset = slot_state_offset 
	};

	return ll;
}

void ll_Flush(struct ll *ll)
{
	ll->count = 0;
	ll->first = LL_NULL;
	ll->last = LL_NULL;
}

void ll_Append(struct ll *ll, void *array, const u32 index)
{
	ll->count += 1;
	u32 *first = (u32*) ((u8*) array + index*ll->slot_size + ll->slot_state_offset);
	*first = ll->first;
	ll->first = index;
	if (ll->last == LL_NULL)
	{
		ll->last = index;
	}
}

void ll_Prepend(struct ll *ll, void *array, const u32 index)
{
	ll->count += 1;
	if (ll->last == LL_NULL)
	{
		ll->first = index;
	}
	else
	{
		u32 *last = (u32*) ((u8*) array + ll->last*ll->slot_size + ll->slot_state_offset);
		*last = index;
	}
	ll->last = index;
	u32 *prev = (u32*) ((u8*) array + index*ll->slot_size + ll->slot_state_offset);
	*prev = LL_NULL;	
}

struct dll dll_InitInternal(const u64 slot_size, const u64 prev_offset, const u64 next_offset)
{
	struct dll dll =  
	{ 
		.count = 0,
	       	.first = DLL_NULL, 
		.last = DLL_NULL, 
		.slot_size = slot_size, 
		.prev_offset = prev_offset, 
		.next_offset = next_offset, 
	};

	return dll;
}

void dll_Flush(struct dll *dll)
{
	dll->count = 0;
	dll->first = DLL_NULL;
	dll->last = DLL_NULL;
}

void dll_Append(struct dll *dll, void *array, const u32 index)
{
	dll->count += 1;
	u32 *node_prev = (u32*) ((u8*) array + index*dll->slot_size + dll->prev_offset);
	u32 *node_next = (u32*) ((u8*) array + index*dll->slot_size + dll->next_offset);
	*node_prev = dll->last;	
	*node_next = DLL_NULL;	

	if (dll->last == DLL_NULL)
	{
		dll->first = index;
	}
	else
	{
		u32 *prev_next = (u32*) ((u8*) array + dll->last*dll->slot_size + dll->next_offset);
		*prev_next = index;
	}

	dll->last = index;
}

void dll_Prepend(struct dll *dll, void *array, const u32 index)
{
	dll->count += 1;
	u32 *node_prev = (u32*) ((u8*) array + index*dll->slot_size + dll->prev_offset);
	u32 *node_next = (u32*) ((u8*) array + index*dll->slot_size + dll->next_offset);
	*node_prev = DLL_NULL;	
	*node_next = dll->first;	

	if (dll->first == DLL_NULL)
	{
		dll->last = index;
	}
	else
	{
		u32 *next_prev = (u32*) ((u8*) array + dll->first*dll->slot_size + dll->prev_offset);
		*next_prev = index;
	}

	dll->first = index;
}

void dll_Remove(struct dll *dll, void *array, const u32 index)
{
	ds_Assert(dll->count);
	dll->count -= 1;

	u32 *node_prev = (u32*) ((u8*) array + index*dll->slot_size + dll->prev_offset);
	u32 *node_next = (u32*) ((u8*) array + index*dll->slot_size + dll->next_offset);

	u32 *prev_next = (u32*) ((u8*) array + (*node_prev)*dll->slot_size + dll->next_offset);
	u32 *next_prev = (u32*) ((u8*) array + (*node_next)*dll->slot_size + dll->prev_offset);

	if (*node_prev == DLL_NULL)
	{
		/* node was only node in list */
		if (*node_next == DLL_NULL)
		{
			dll->first = DLL_NULL;
			dll->last = DLL_NULL;
		}
		/* node is first  */
		else
		{
			*next_prev = DLL_NULL;
			dll->first = *node_next;
		}
	}
	else
	{
		/* node is last */
		if (*node_next == DLL_NULL)
		{
			*prev_next = DLL_NULL;
			dll->last = *node_prev;
		}
		/* node is inbetween  */
		else
		{
			*prev_next = *node_next;
			*next_prev = *node_prev;
		}
	}

	*node_prev = DLL_NOT_IN_LIST;
	*node_next = DLL_NOT_IN_LIST;
}

void dll_SlotSetNotInList(struct dll *dll, void *slot)
{
	u32 *node_prev = (u32*) ((u8*) slot + dll->prev_offset);
	u32 *node_next = (u32*) ((u8*) slot + dll->next_offset);

	*node_prev = DLL_NOT_IN_LIST;
	*node_next = DLL_NOT_IN_LIST;
}

struct nll nll_AllocInternal(struct arena *mem, 
		const u32 initial_length, 
		const u64 data_size, 
		const u64 pool_slot_offset, 
		const u64 next_offset, 
		const u64 prev_offset, 
		u32 (*index_in_prev_node)(struct nll *, void **, const void *, const u32),
	       	u32 (*index_in_next_node)(struct nll *, void **, const void *, const u32),
		const u32 growable)
{
	ds_Assert(!growable || !mem);
	ds_Assert(initial_length);
	
	struct nll net =
	{
		.index_in_prev_node = index_in_prev_node,
	       	.index_in_next_node = index_in_next_node,
		.next_offset = next_offset,
		.prev_offset = prev_offset,
	};

	net.pool = PoolAllocInternal(mem, initial_length, data_size, pool_slot_offset, U64_MAX, growable);
	if (!net.pool.length)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate net list");
		FatalCleanupAndExit();
	}

	struct slot slot = PoolAdd(&net.pool);
	u32 *next = (u32 *)((u8 *) slot.address + net.next_offset);
	u32 *prev = (u32 *)((u8 *) slot.address + net.prev_offset);
	next[0] = NLL_NULL;
	next[1] = NLL_NULL;
	prev[0] = NLL_NULL;
	prev[1] = NLL_NULL;
	ds_Assert(slot.index == NLL_NULL);

	return net;	
}

void nll_Dealloc(struct nll *net)
{
	if (net->heap_allocated)
	{
		PoolDealloc(&net->pool);
	}
}

void nll_Flush(struct nll *net)
{
	PoolFlush(&net->pool);
	struct slot slot = PoolAdd(&net->pool);
	u32 *next = (u32 *)((u8 *) slot.address + net->next_offset);
	u32 *prev = (u32 *)((u8 *) slot.address + net->prev_offset);
	next[0] = NLL_NULL;
	next[1] = NLL_NULL;
	prev[0] = NLL_NULL;
	prev[1] = NLL_NULL;
	ds_Assert(slot.index == NLL_NULL);
}

struct slot nll_Add(struct nll *net, void *data, const u32 next_0, const u32 next_1)
{
	struct slot slot = PoolAdd(&net->pool);

	/* copy over internal data required in index_in_prev_node function call */
	memcpy((u8*) data + net->pool.slot_allocation_offset, (u8*) slot.address + net->pool.slot_allocation_offset, sizeof(u32));
	memcpy(slot.address, data, net->pool.slot_size);

	u32 *next = (u32 *)((u8 *) slot.address + net->next_offset);
	u32 *prev = (u32 *)((u8 *) slot.address + net->prev_offset);

	next[0] = next_0;
	next[1] = next_1;
	prev[0] = NLL_NULL;
	prev[1] = NLL_NULL;

       	u8 *node_next_0;
       	u8 *node_next_1;

	const u32 index_next_0 = net->index_in_next_node(net, (void **) &node_next_0, slot.address, 0);
	const u32 index_next_1 = net->index_in_next_node(net, (void **) &node_next_1, slot.address, 1);

	prev = (u32 *)((u8 *) node_next_0 + net->prev_offset);
	ds_AssertString(next_0 == NLL_NULL || prev[index_next_0] == NLL_NULL, "either the next node must be the NULL NODE, indicating a list of size 1, or the previous head in the list which should have its previous node as the NULL NODE");
	prev[index_next_0] = slot.index;
	
	prev = (u32 *)((u8 *) node_next_1 + net->prev_offset);
	ds_AssertString(next_1 == NLL_NULL || prev[index_next_1] == NLL_NULL, "either the next node must be the NULL NODE, indicating a list of size 1, or the previous head in the list which should have its previous node as the NULL NODE");
	prev[index_next_1] = slot.index;

	u8 *tmp;
	ds_Assert(next_0 == NLL_NULL || net->index_in_prev_node(net, (void **) &tmp, node_next_0, index_next_0) == 0);
	ds_Assert(next_1 == NLL_NULL || net->index_in_prev_node(net, (void **) &tmp, node_next_1, index_next_1) == 1);

	return slot;
}

void nll_Remove(struct nll *net, const u32 index)
{
	u8 *node = PoolAddress(&net->pool, index);
	u32 *node_next = (u32 *)((u8 *) node + net->next_offset);
	u32 *node_prev = (u32 *)((u8 *) node + net->prev_offset);

	u8 *node_prev_0, *node_prev_1, *node_next_0, *node_next_1; 
	const u32 index_prev_0 = net->index_in_prev_node(net, (void **) &node_prev_0, node, 0);
	const u32 index_prev_1 = net->index_in_prev_node(net, (void **) &node_prev_1, node, 1);
	const u32 index_next_0 = net->index_in_next_node(net, (void **) &node_next_0, node, 0);
	const u32 index_next_1 = net->index_in_next_node(net, (void **) &node_next_1, node, 1);

	u32 *node_prev_0_next = (u32 *)((u8 *) node_prev_0 + net->next_offset) + index_prev_0;
	u32 *node_prev_1_next = (u32 *)((u8 *) node_prev_1 + net->next_offset) + index_prev_1;
	u32 *node_next_0_prev = (u32 *)((u8 *) node_next_0 + net->prev_offset) + index_next_0;
	u32 *node_next_1_prev = (u32 *)((u8 *) node_next_1 + net->prev_offset) + index_next_1;

	ds_Assert(node_prev[0] == NLL_NULL || *node_prev_0_next == index);
	ds_Assert(node_prev[1] == NLL_NULL || *node_prev_1_next == index);
	ds_Assert(node_next[0] == NLL_NULL || *node_next_0_prev == index);
	ds_Assert(node_next[1] == NLL_NULL || *node_next_1_prev == index);

	*node_prev_0_next = node_next[0];
	*node_prev_1_next = node_next[1];
	*node_next_0_prev = node_prev[0];
	*node_next_1_prev = node_prev[1];

	PoolRemove(&net->pool, index);
}

void *nll_Address(const struct nll *net, const u32 index)
{
	return PoolAddress(&net->pool, index);
}

u32 nll_Index(const struct nll *net, const void *address)
{
	return PoolIndex(&net->pool, address);
}

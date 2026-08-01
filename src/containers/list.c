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

void ll_Prepend(struct ll *ll, void *array, const u32 index)
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

void ll_Append(struct ll *ll, void *array, const u32 index)
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
	u32 *next = (u32*) ((u8*) array + index*ll->slot_size + ll->slot_state_offset);
	*next = LL_NULL;	
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

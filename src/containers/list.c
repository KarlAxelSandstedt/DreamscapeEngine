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

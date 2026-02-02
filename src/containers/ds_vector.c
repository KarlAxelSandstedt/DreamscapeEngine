/* ==========================================================================
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

#include <stdlib.h>
#include "ds_vector.h"
#include "vector.h"

DEFINE_STACK(u64);
DEFINE_STACK(u32);
DEFINE_STACK(f32);
DEFINE_STACK(ptr);
DEFINE_STACK(intv);

static const struct vector empty = { 0 };

struct vector VectorAlloc(struct arena *mem, const u64 blocksize, const u32 length, const u32 growable)
{
	ds_Assert(length && blocksize);

	struct vector v =
	{
		.blocksize = blocksize,
		.next = 0,
		.growable = growable,
	};

	if (mem)
	{
		v.length = length,
		v.data = ArenaPush(mem, blocksize*length);
	}
	else
	{
		const u64 size = ds_AllocSizeCeil(length*blocksize);
		v.length = size / blocksize;
		v.data = ds_Alloc(&v.mem_slot, size, HUGE_PAGES);
	}

	if (!v.data)
	{
		LogString(T_SYSTEM, S_ERROR, "Failed to allocate vector");
		v = empty;
	}

	return v;
}

void VectorDealloc(struct vector *v)
{
	if (v->mem_slot.address)
	{
		ds_Free(&v->mem_slot);
	}
}

struct slot VectorPush(struct vector *v)
{
	if (v->next >= v->length)
	{
		if (v->growable)
		{
			const u64 size = ds_AllocSizeCeil(2*v->mem_slot.size);
			v->length = size / v->blocksize;
			v->data = ds_Realloc(&v->mem_slot, size);
			if (!v->data)
			{
				LogString(T_SYSTEM, S_FATAL, "Failed to resize vector");
				FatalCleanupAndExit();
			}
		}
		else
		{
			return (struct slot) { .index = 0, .address = NULL };
		}
	}

	struct slot slot = { .address = VectorAddress(v, v->next) };
	slot.index = v->next;
	v->next += 1;
	return slot;
}

void VectorPop(struct vector *v)
{
	ds_Assert(v->next);
	v->next -= 1;
}

void *VectorAddress(const struct vector *v, const u32 index)
{
	return v->data + v->blocksize*index;
}

void VectorFlush(struct vector *v)
{
	v->next = 0;
}

struct stackVec3 stackVec3Alloc(struct arena *arena, const u32 length, const u32 growable)
{								
	ds_Assert(!arena || !growable);

	struct stackVec3 stack =				
	{							
		.next = 0,					
		.growable = growable,				
	};							

	if (arena)
	{
		stack.length = length;
		stack.arr = ArenaPush(arena, sizeof(stack.arr[0])*stack.length);
	}
	else
	{
		const u64 size = PowerOfTwoCeil( sizeof(stack.arr[0]) * ds_AllocSizeCeil(length) );
		stack.length = (u32) (size / sizeof(stack.arr[0]));

		stack.arr = (size >= 1024*1024) 
			? ds_Alloc(&stack.mem_slot, size, HUGE_PAGES)
			: ds_Alloc(&stack.mem_slot, size, NO_HUGE_PAGES);
	}

	if (length > 0 && !stack.arr)				
	{							
		FatalCleanupAndExit();				
	}							
	return stack;						
}

void stackVec3Free(struct stackVec3 *stack)
{						
	if (stack->mem_slot.address)				
	{					
		ds_Free(&stack->mem_slot);		
	}					
}

void stackVec3Push(struct stackVec3 *stack, const vec3 val)
{														
	if (stack->next >= stack->length)									
	{													
		if (stack->growable)										
		{												
			stack->arr = ds_Realloc(&stack->mem_slot, 2*stack->mem_slot.size);			
			stack->length = (u32) (stack->mem_slot.size / sizeof(stack->arr[0]));
			if (!stack->arr)									
			{											
				FatalCleanupAndExit();								
			}											
		}												
		else												
		{												
			FatalCleanupAndExit();									
		}												
	}													
	stack->arr[stack->next][0] = val[0];
	stack->arr[stack->next][1] = val[1];
	stack->arr[stack->next][2] = val[2];
	stack->next += 1;											
}

void stackVec3Set(struct stackVec3 *stack, const vec3 val)
{								
	ds_Assert(stack->next);					
	stack->arr[stack->next-1][0] = val[0];
	stack->arr[stack->next-1][1] = val[1];
	stack->arr[stack->next-1][2] = val[2];
}

void stackVec3Pop(struct stackVec3 *stack)
{								
	ds_Assert(stack->next);					
	stack->next -= 1;					
}

void stackVec3Flush(struct stackVec3 *stack)
{								
	stack->next = 0;					
}

void stackVec3Top(vec3 ret_val, const struct stackVec3 *stack)
{								
	ds_Assert(stack->next);				
	ret_val[0] = stack->arr[stack->next-1][0];
	ret_val[1] = stack->arr[stack->next-1][1];
	ret_val[2] = stack->arr[stack->next-1][2];
}

struct stackVec4 stackVec4Alloc(struct arena *arena, const u32 length, const u32 growable)
{								
	ds_Assert(!arena || !growable);

	struct stackVec4 stack =				
	{							
		.next = 0,					
		.growable = growable,				
	};							

	if (arena)
	{
		stack.length = length;
		stack.arr = ArenaPush(arena, sizeof(stack.arr[0])*stack.length);
	}
	else
	{
		const u64 size = PowerOfTwoCeil( ds_AllocSizeCeil( sizeof(stack.arr[0])*length ) );
		stack.length = (u32) (size / sizeof(stack.arr[0]));

		stack.arr = (size >= 1024*1024) 
			? ds_Alloc(&stack.mem_slot, size, HUGE_PAGES)
			: ds_Alloc(&stack.mem_slot, size, NO_HUGE_PAGES);
	}

	if (length > 0 && !stack.arr)				
	{							
		FatalCleanupAndExit();				
	}							
	return stack;						
}

void stackVec4Free(struct stackVec4 *stack)
{						
	if (stack->mem_slot.address)				
	{					
		ds_Free(&stack->mem_slot);		
	}					
}

void stackVec4Push(struct stackVec4 *stack, const vec4 val)
{														
	if (stack->next >= stack->length)									
	{													
		if (stack->growable)										
		{												
			stack->arr = ds_Realloc(&stack->mem_slot, 2*stack->mem_slot.size);			
			stack->length = (u32) (stack->mem_slot.size / sizeof(stack->arr[0]));
			if (!stack->arr)									
			{											
				FatalCleanupAndExit();								
			}											
		}												
		else												
		{												
			FatalCleanupAndExit();									
		}												
	}													
	stack->arr[stack->next][0] = val[0];
	stack->arr[stack->next][1] = val[1];
	stack->arr[stack->next][2] = val[2];
	stack->arr[stack->next][3] = val[3];
	stack->next += 1;											
}

void stackVec4Set(struct stackVec4 *stack, const vec4 val)
{								
	ds_Assert(stack->next);					
	stack->arr[stack->next-1][0] = val[0];
	stack->arr[stack->next-1][1] = val[1];
	stack->arr[stack->next-1][2] = val[2];
	stack->arr[stack->next-1][3] = val[3];
}

void stackVec4Pop(struct stackVec4 *stack)
{								
	ds_Assert(stack->next);					
	stack->next -= 1;					
}

void stackVec4Flush(struct stackVec4 *stack)
{								
	stack->next = 0;					
}

void stackVec4Top(vec4 ret_val, const struct stackVec4 *stack)
{								
	ds_Assert(stack->next);				
	ret_val[0] = stack->arr[stack->next-1][0];
	ret_val[1] = stack->arr[stack->next-1][1];
	ret_val[2] = stack->arr[stack->next-1][2];
	ret_val[3] = stack->arr[stack->next-1][3];
}

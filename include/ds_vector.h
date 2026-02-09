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

#ifndef __DS_VECTOR_H__
#define __DS_VECTOR_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"

/****************************************** general vector ******************************************/

/*
 * struct vector: Simple stack-based array, i.e. all of its contiguous memory up until data[next] is valid.
 */
struct vector
{
	u64		blocksize;	/* size of individual block 	*/
	u8 *		data;		/* memory address base 		*/
	u32 		length;		/* memory length (in blocks)	*/
	u32 		next;		/* next index to be pushed 	*/ 
	u32		growable;	/* Boolean: is memory growable  */
	struct memSlot 	mem_slot;	/* Optionally set if ds_Alloc	*/
};

/* allocate and initalize vector: If mem is defined, use arena allocator; given that growable == VECTOR_STATIC */
struct vector	VectorAlloc(struct arena *mem, const u64 blocksize, const u32 length, const u32 growable);
/* deallocate vector heap memory */
void		VectorDealloc(struct vector *v);
/* push block, return index and address on success, otherwise return (0, NULL) */
struct slot	VectorPush(struct vector *v);
/* pop block  */
void		VectorPop(struct vector *v);
/* return address of indexed block */
void *		VectorAddress(const struct vector *v, const u32 index);
/* pop all allocated blocks */
void		VectorFlush(struct vector *v);


/****************************************** FIXED TYPE STACK GENERATION ******************************************/

#define DECLARE_STACK_STRUCT(type)	\
typedef struct				\
{					\
	u32 		length;		\
	u32 		next;		\
	u32		growable;	\
	type *		arr;		\
	struct memSlot	mem_slot;	\
} stack_ ## type

#define DECLARE_STACK_ALLOC(type)	stack_ ## type stack_ ## type ## Alloc(struct arena *arena, const u32 length, const u32 growable)
#define DECLARE_STACK_FREE(type)	void stack_ ## type ## Free(stack_ ## type *stack)
#define DECLARE_STACK_PUSH(type)	void stack_ ## type ## Push(stack_ ## type *stack, const type val)
#define DECLARE_STACK_SET(type)		void stack_ ## type ## Set(stack_ ## type *stack, const type val)
#define DECLARE_STACK_POP(type)		type stack_ ## type ## Pop(stack_ ## type *stack)
#define DECLARE_STACK_TOP(type)		type stack_ ## type ## Top(stack_ ## type *stack)
#define DECLARE_STACK_FLUSH(type)	void stack_ ## type ## Flush(stack_ ## type *stack)

#define DECLARE_STACK(type)		\
	DECLARE_STACK_STRUCT(type);	\
	DECLARE_STACK_ALLOC(type);	\
	DECLARE_STACK_PUSH(type);	\
	DECLARE_STACK_POP(type);	\
	DECLARE_STACK_TOP(type);	\
	DECLARE_STACK_SET(type);	\
	DECLARE_STACK_FLUSH(type);	\
	DECLARE_STACK_FREE(type)

#define DEFINE_STACK_ALLOC(type)							\
DECLARE_STACK_ALLOC(type)								\
{											\
	ds_Assert(!arena || !growable);							\
	stack_ ## type stack =								\
	{										\
		.next = 0,								\
		.growable = growable,							\
	};										\
	if (arena)									\
	{										\
		stack.length = length;							\
		stack.arr = ArenaPush(arena, sizeof(stack.arr[0])*stack.length);	\
	}										\
	else										\
	{										\
		const u64 size = ds_AllocSizeCeil( sizeof(stack.arr[0])*length );	\
		stack.length = (u32) (size / sizeof(stack.arr[0]));			\
		stack.arr = (size >= 1024*1024) 					\
			? ds_Alloc(&stack.mem_slot, size, HUGE_PAGES)			\
			: ds_Alloc(&stack.mem_slot, size, NO_HUGE_PAGES);		\
	}										\
	PoisonAddress(stack.arr, stack.length*sizeof(stack.arr[0]));			\
	if (length > 0 && !stack.arr)							\
	{										\
		FatalCleanupAndExit();							\
	}										\
	return stack;									\
}

#define DEFINE_STACK_FREE(type)			\
DECLARE_STACK_FREE(type)			\
{						\
	if (stack->mem_slot.address)		\
	{					\
		ds_Free(&stack->mem_slot);	\
	}					\
}

#define DEFINE_STACK_PUSH(type)									\
DECLARE_STACK_PUSH(type)									\
{												\
	if (stack->next >= stack->length)							\
	{											\
		if (stack->growable)								\
		{										\
			stack->arr = ds_Realloc(&stack->mem_slot, 2*stack->mem_slot.size);	\
			PoisonAddress(stack->arr, stack->mem_slot.size);			\
			UnpoisonAddress(stack->arr, stack->length*sizeof(stack->arr[0]));	\
			stack->length = (u32) (stack->mem_slot.size / sizeof(stack->arr[0]));	\
			if (!stack->arr)							\
			{									\
				FatalCleanupAndExit();						\
			}									\
		}										\
		else										\
		{										\
			FatalCleanupAndExit();							\
		}										\
	}											\
	UnpoisonAddress(stack->arr + stack->next, sizeof(stack->arr[0]));			\
	stack->arr[stack->next] = val;								\
	stack->next += 1;									\
}

#define DEFINE_STACK_SET(type)			\
DECLARE_STACK_SET(type)				\
{						\
	ds_Assert(stack->next);			\
	stack->arr[stack->next-1] = val;	\
}

#define DEFINE_STACK_POP(type)						\
DECLARE_STACK_POP(type)							\
{									\
	ds_Assert(stack->next);						\
	stack->next -= 1;						\
	const type val = stack->arr[stack->next];			\
	PoisonAddress(stack->arr + stack->next, sizeof(stack->arr[0]));	\
	return val;							\
}

#define DEFINE_STACK_FLUSH(type)				\
DECLARE_STACK_FLUSH(type)					\
{								\
	PoisonAddress(stack->arr, stack->mem_slot.size);	\
	stack->next = 0;					\
}

#define DEFINE_STACK_TOP(type)			\
DECLARE_STACK_TOP(type)				\
{						\
	ds_Assert(stack->next);			\
	return stack->arr[stack->next-1];	\
}

#define DEFINE_STACK(type)		\
	DEFINE_STACK_ALLOC(type)	\
	DEFINE_STACK_PUSH(type)		\
	DEFINE_STACK_POP(type)		\
	DEFINE_STACK_TOP(type)		\
	DEFINE_STACK_SET(type)		\
	DEFINE_STACK_FLUSH(type)	\
	DEFINE_STACK_FREE(type)

typedef void * ptr;

DECLARE_STACK(u64);
DECLARE_STACK(u32);
DECLARE_STACK(f32);
DECLARE_STACK(ptr);
DECLARE_STACK(intv);

/*
Vector stacks
=============
*/

struct stackVec3
{						
	u32 		length;			
	u32 		next;			
	u32		growable;		
	vec3ptr		arr;			
	struct memSlot 	mem_slot;
};

struct stackVec3 	stackVec3Alloc(struct arena *arena, const u32 length, const u32 growable);
void 			stackVec3Free(struct stackVec3 *stack);
void 			stackVec3Push(struct stackVec3 *stack, const vec3 val);
void 			stackVec3Set(struct stackVec3 *stack, const vec3 val);
void 			stackVec3Pop(struct stackVec3 *stack);
void 			stackVec3Flush(struct stackVec3 *stack);
void 			stackVec3Top(vec3 ret_val, const struct stackVec3 *stack);

struct stackVec4
{						
	u32 	length;			
	u32 	next;			
	u32	growable;		
	vec4ptr	arr;			
	struct memSlot 	mem_slot;
};

struct stackVec4 	stackVec4Alloc(struct arena *arena, const u32 length, const u32 growable);
void 			stackVec4Free(struct stackVec4 *stack);
void 			stackVec4Push(struct stackVec4 *stack, const vec4 val);
void 			stackVec4Set(struct stackVec4 *stack, const vec4 val);
void 			stackVec4Pop(struct stackVec4 *stack);
void 			stackVec4Flush(struct stackVec4 *stack);
void 			stackVec4Top(vec4 ret_val, const struct stackVec4 *stack);

#ifdef __cplusplus
} 
#endif

#endif

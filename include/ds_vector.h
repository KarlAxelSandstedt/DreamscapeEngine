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

#define STACK_GROWABLE	1

#define DECLARE_STACK_STRUCT(type)	\
typedef struct				\
{					\
	u32 	length;			\
	u32 	next;			\
	u32	growable;		\
	type *	arr;			\
					\
} stack_ ## type

#define DECLARE_STACK_ALLOC(type)	stack_ ## type stack_ ## type ## _alloc(struct arena *arena, const u32 length, const u32 growable)
#define DECLARE_STACK_FREE(type)	void stack_ ## type ## _free(stack_ ## type *stack)
#define DECLARE_STACK_PUSH(type)	void stack_ ## type ## _push(stack_ ## type *stack, const type val)
#define DECLARE_STACK_SET(type)		void stack_ ## type ## _set(stack_ ## type *stack, const type val)
#define DECLARE_STACK_POP(type)		type stack_ ## type ## _pop(stack_ ## type *stack)
#define DECLARE_STACK_TOP(type)		type stack_ ## type ## _top(stack_ ## type *stack)
#define DECLARE_STACK_FLUSH(type)	void stack_ ## type ## _flush(stack_ ## type *stack)

#define DECLARE_STACK(type)		\
	DECLARE_STACK_STRUCT(type);	\
	DECLARE_STACK_ALLOC(type);	\
	DECLARE_STACK_PUSH(type);	\
	DECLARE_STACK_POP(type);	\
	DECLARE_STACK_TOP(type);	\
	DECLARE_STACK_SET(type);	\
	DECLARE_STACK_FLUSH(type);	\
	DECLARE_STACK_FREE(type)

#define DEFINE_STACK_ALLOC(type)									\
DECLARE_STACK_ALLOC(type)										\
{													\
	stack_ ## type stack =										\
	{												\
		.length = length,									\
		.next = 0,										\
		.growable = growable,									\
	};												\
	stack.arr = (arena)										\
		? ArenaPush(arena, sizeof(type)*length)							\
		: malloc(sizeof(type)*length);								\
	if (length > 0 && !stack.arr)									\
	{												\
		FatalCleanupAndExit();									\
	}												\
	return stack;											\
}

#define DEFINE_STACK_FREE(type)			\
DECLARE_STACK_FREE(type)			\
{						\
	if (stack->arr)				\
	{					\
		free(stack->arr);		\
	}					\
}

#define DEFINE_STACK_PUSH(type)									\
DECLARE_STACK_PUSH(type)									\
{												\
	if (stack->next >= stack->length)							\
	{											\
		if (stack->growable)								\
		{										\
			stack->length *= 2;							\
			stack->arr = realloc(stack->arr, stack->length*sizeof(stack->arr[0]));	\
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
	stack->arr[stack->next] = val;								\
	stack->next += 1;									\
}

#define DEFINE_STACK_SET(type)			\
DECLARE_STACK_SET(type)				\
{						\
	ds_Assert(stack->next);			\
	stack->arr[stack->next-1] = val;	\
}

#define DEFINE_STACK_POP(type)				\
DECLARE_STACK_POP(type)					\
{							\
	ds_Assert(stack->next);				\
	stack->next -= 1;				\
	const type val = stack->arr[stack->next];	\
	return val;					\
}

#define DEFINE_STACK_FLUSH(type)			\
DECLARE_STACK_FLUSH(type)				\
{							\
	stack->next = 0;				\
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

#define DECLARE_STACK_VEC_STRUCT(vectype)	\
typedef struct					\
{						\
	u32 		length;			\
	u32 		next;			\
	u32		growable;		\
	vectype ## ptr	arr;			\
						\
} stack_ ## vectype

#define DECLARE_STACK_VEC_ALLOC(vectype)	stack_ ## vectype stack_ ## vectype ## _alloc(struct arena *arena, const u32 length, const u32 growable)
#define DECLARE_STACK_VEC_FREE(vectype)		void stack_ ## vectype ## _free(stack_ ## vectype *stack)
#define DECLARE_STACK_VEC_PUSH(vectype)		void stack_ ## vectype ## _push(stack_ ## vectype *stack, const vectype val)
#define DECLARE_STACK_VEC_SET(vectype)		void stack_ ## vectype ## _set(stack_ ## vectype *stack, const vectype val)
#define DECLARE_STACK_VEC_POP(vectype)		void stack_ ## vectype ## _pop(stack_ ## vectype *stack)
#define DECLARE_STACK_VEC_FLUSH(vectype)	void stack_ ## vectype ## _flush(stack_ ## vectype *stack)
#define DECLARE_STACK_VEC_TOP(vectype)		void stack_ ## vectype ## _top(vectype ret_val, stack_ ## vectype *stack)

#define DECLARE_STACK_VEC(vectype)		\
	DECLARE_STACK_VEC_STRUCT(vectype);	\
	DECLARE_STACK_VEC_ALLOC(vectype);	\
	DECLARE_STACK_VEC_PUSH(vectype);	\
	DECLARE_STACK_VEC_POP(vectype);		\
	DECLARE_STACK_VEC_TOP(vectype);		\
	DECLARE_STACK_VEC_SET(vectype);		\
	DECLARE_STACK_VEC_FLUSH(vectype);	\
	DECLARE_STACK_VEC_FREE(vectype)

#define DEFINE_STACK_VEC_ALLOC(vectype)				\
DECLARE_STACK_VEC_ALLOC(vectype)				\
{								\
	stack_ ## vectype stack =				\
	{							\
		.length = length,				\
		.next = 0,					\
		.growable = growable,				\
	};							\
	stack.arr = (arena)					\
		? ArenaPush(arena, sizeof(vectype)*length)	\
		: malloc(sizeof(vectype)*length);		\
	if (length > 0 && !stack.arr)				\
	{							\
		FatalCleanupAndExit();				\
	}							\
	return stack;						\
}

#define DEFINE_STACK_VEC_FREE(vectype)		\
DECLARE_STACK_VEC_FREE(vectype)			\
{						\
	if (stack->arr)				\
	{					\
		free(stack->arr);		\
	}					\
}

#define DEFINE_STACK_VEC_PUSH(vectype)										\
DECLARE_STACK_VEC_PUSH(vectype)											\
{														\
	if (stack->next >= stack->length)									\
	{													\
		if (stack->growable)										\
		{												\
			stack->length *= 2;									\
			stack->arr = realloc(stack->arr, stack->length*sizeof(stack->arr[0]));			\
			if (!stack->arr)									\
			{											\
				FatalCleanupAndExit();								\
			}											\
		}												\
		else												\
		{												\
			FatalCleanupAndExit();									\
		}												\
	}													\
	vectype ## _copy(stack->arr[stack->next], val);								\
	stack->next += 1;											\
}

#define DEFINE_STACK_VEC_SET(vectype)				\
DECLARE_STACK_VEC_SET(vectype)					\
{								\
	ds_Assert(stack->next);				\
	vectype ## _copy(stack->arr[stack->next-1], val);	\
}

#define DEFINE_STACK_VEC_POP(vectype)				\
DECLARE_STACK_VEC_POP(vectype)					\
{								\
	ds_Assert(stack->next);				\
	stack->next -= 1;					\
}

#define DEFINE_STACK_VEC_FLUSH(vectype)				\
DECLARE_STACK_VEC_FLUSH(vectype)				\
{								\
	stack->next = 0;					\
}

#define DEFINE_STACK_VEC_TOP(vectype)				\
DECLARE_STACK_VEC_TOP(vectype)					\
{								\
	ds_Assert(stack->next);				\
	vectype ## _copy(ret_val, stack->arr[stack->next-1]);	\
}

#define DEFINE_STACK_VEC(vectype)		\
	DEFINE_STACK_VEC_ALLOC(vectype)		\
	DEFINE_STACK_VEC_PUSH(vectype)		\
	DEFINE_STACK_VEC_POP(vectype)		\
	DEFINE_STACK_VEC_TOP(vectype)		\
	DEFINE_STACK_VEC_SET(vectype)		\
	DEFINE_STACK_VEC_FLUSH(vectype)		\
	DEFINE_STACK_VEC_FREE(vectype)

typedef void * ptr;

DECLARE_STACK(u64);
DECLARE_STACK(u32);
DECLARE_STACK(f32);
DECLARE_STACK(ptr);
DECLARE_STACK(intv);
DECLARE_STACK_VEC(vec3);
DECLARE_STACK_VEC(vec4);

#ifdef __cplusplus
} 
#endif

#endif

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

#include "ds_vector.h"

DEFINE_STACK(u64);
DEFINE_STACK(u32);
DEFINE_STACK(f32);
DEFINE_STACK(ptr);
DEFINE_STACK(intv);

//DEFINE_STACK_VEC(vec3);
//DEFINE_STACK_VEC(vec4);

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


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

void ArenaPushRecord(struct arena *ar)
{
	const u64 rec_mem_left = ar->mem_left;
	struct arenaRecord *record = ArenaPush(ar, sizeof(struct arenaRecord));
	/* a missing record would make the matching ArenaPopRecord pop the outer record */
	if (!record)
	{
		LogString(T_SYSTEM, S_FATAL, "Arena full in ArenaPushRecord, exiting.");
		FatalCleanupAndExit();
	}

	record->prev = ar->record;
	record->rec_mem_left = rec_mem_left;
	ar->record = record;
}

void ArenaPopRecord(struct arena *ar)
{
	if (ar->record)
	{
		ds_Assert((u64) ar->record <= (u64) ar->stack_ptr);
		ds_Assert(ar->mem_left <= ar->record->rec_mem_left);
		const u64 rec_mem_left = ar->record->rec_mem_left;
		ar->record = ar->record->prev;
		ArenaPopPacked(ar, rec_mem_left - ar->mem_left);
	}
}

void ArenaRemoveRecord(struct arena *ar)
{
	if (ar->record)
	{
		ar->record = ar->record->prev;
	}
}

struct arena ArenaAlloc(struct arena *mem, const u64 size)
{
	struct arena ar =
	{
		.mem_size = 0,
		.mem_left = 0,
		.record = NULL,
	};

	ar.stack_ptr = (mem)
		? ArenaPushAligned(mem, size, DS_CACHE_LINE)
		: ds_Alloc(size, DS_CACHE_LINE);
	if (ar.stack_ptr)
	{
		ar.mem_size = size;
		ar.mem_left = size;
		ar.heap_allocated = (mem == NULL);
		PoisonAddress(ar.stack_ptr, ar.mem_left);
	}

	return ar;
}

void ArenaFree(struct arena *ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	UnpoisonAddress(ar->stack_ptr, ar->mem_size);
	if (ar->heap_allocated)
	{
		ds_Free(ar->stack_ptr);
	}
	ar->heap_allocated = 0;
	ar->mem_size = 0;
	ar->mem_left = 0;
	ar->stack_ptr = NULL;
	ar->record = NULL;
}

void ArenaFlush(struct arena* ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	ar->mem_left = ar->mem_size;
	ar->record = NULL;
	PoisonAddress(ar->stack_ptr, ar->mem_left);
}

void ArenaPopPacked(struct arena *ar, const u64 mem_to_pop)
{
	ds_AssertString(ar->mem_size - ar->mem_left >= mem_to_pop, "Trying to pop memory outside of arena");
	ar->stack_ptr -= mem_to_pop;
	ar->mem_left += mem_to_pop;
	PoisonAddress(ar->stack_ptr, mem_to_pop);
}

void *ArenaPushAligned(struct arena *ar, const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment) == 1);

	void* alloc_addr = NULL;
	if (size) 
	{ 
		/* bytes up to the next multiple of alignment */
		const u64 push_alignment = (0 - (u64) ar->stack_ptr) & (alignment - 1);
		if (ar->mem_left >= size + push_alignment)
		{
			UnpoisonAddress(ar->stack_ptr + push_alignment, size);
			alloc_addr = ar->stack_ptr + push_alignment;
			ar->mem_left -= size + push_alignment;
			ar->stack_ptr += size + push_alignment;
            ds_Assert((u64) alloc_addr % alignment == 0);
		}
	}

	return alloc_addr;
}


void *ArenaPushAlignedMemcpy(struct arena *ar, const void *copy, const u64 size, const u64 alignment)
{
	void *addr = ArenaPushAligned(ar, size, alignment);
	if (addr)
	{
		memcpy(addr, copy, size);
	}
	return addr;
}

void *ArenaPushAlignedZero(struct arena *ar, const u64 size, const u64 alignment)
{
	void *addr = ArenaPushAligned(ar, size, alignment);
	if (addr)
	{
		memset(addr, 0, size);
	}
	return addr;
}

struct memArray ArenaPushAlignedAll(struct arena *ar, const u64 slot_size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment) == 1 && slot_size > 0);

	struct memArray array = { .len = 0, .addr = NULL, .memPushed = 0 };
	/* bytes up to the next multiple of alignment */
	const u64 push_alignment = (0 - (u64) ar->stack_ptr) & (alignment - 1);
	if (push_alignment + slot_size <= ar->mem_left)
	{
		array.len = (ar->mem_left - push_alignment) / slot_size;
		array.addr = ar->stack_ptr + push_alignment;
		UnpoisonAddress(ar->stack_ptr + push_alignment, array.len * slot_size);
		array.memPushed = push_alignment + array.len * slot_size;
		ar->mem_left  -= push_alignment + array.len * slot_size;
		ar->stack_ptr += push_alignment + array.len * slot_size;
	}

	return array;
}

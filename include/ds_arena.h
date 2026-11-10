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

/*
ds_arena.h
==========
Arena: stack allocator (push, pop, records) on a heap block or inside another arena.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_ARENA_H__
#define __DS_ARENA_H__

/*
Arena API
---------
Stack allocator: pushes move a pointer forward, pops release the most recent pushes (LIFO).
*/

/*
ArenaPushAlignedAll result. Pop all of it with ArenaPopPacked(ar, array.memPushed); to keep the first N
slots, pop slot_size * (array.len - N).
*/
struct memArray
{
	void *	addr;		/* NULL if not even one slot fit */
	u64	len;		/* slots pushed */
	u64	memPushed;	/* bytes pushed, alignment padding included */
};

/* Internal: a record lives in its arena; records form a list from the latest to the oldest. */
struct arenaRecord
{
	struct arenaRecord *	prev;
	u64 			rec_mem_left;
};

struct arena 
{
	u8 * 			stack_ptr;	/* next free byte */
	u64 			mem_size;
	u64 			mem_left;
	struct arenaRecord *	record;		/* latest record, NULL == no record */
	u32			heap_allocated;	/* memory from ds_Alloc (else from a parent arena) */
};

/*
Return an arena of size bytes, cache-line aligned: a ds_Alloc block if mem == NULL, else pushed onto mem
(it is released when mem pops it, not by ArenaFree). On failure, the arena is { 0 }.
*/
struct arena ArenaAlloc(struct arena *mem, const u64 size);
/* Free a heap arena's block (no-op for an arena inside another) and leave the arena empty, size 0. */
void		ArenaFree(struct arena *ar);
/* Pop everything and drop all records. */
void		ArenaFlush(struct arena* ar);	

/* Record the current position. Fatal (FatalCleanupAndExit) if the arena has no room for the record. */
void		ArenaPushRecord(struct arena *ar);
/* Pop back to the latest record and remove it; no-op without records. */
void		ArenaPopRecord(struct arena *ar);
/* Remove the latest record without popping (the memory pushed since stays); no-op without records. */
void 		ArenaRemoveRecord(struct arena *ar);

/* Release the last mem_to_pop bytes pushed (alignment padding included); at most the bytes in use. */
void 		ArenaPopPacked(struct arena *ar, const u64 mem_to_pop);

/*
Return size bytes aligned to alignment (a power of two), or NULL (arena unchanged) if size == 0 or they
don't fit.
*/
void *		ArenaPushAligned(struct arena *ar, const u64 size, const u64 alignment);
/* ArenaPushAligned, with the bytes set to 0. */
void *		ArenaPushAlignedZero(struct arena *ar, const u64 size, const u64 alignment);
/* ArenaPushAligned, with size bytes copied from copy. */
void *		ArenaPushAlignedMemcpy(struct arena *ar, const void *copy, const u64 size, const u64 alignment);
/* Push every whole slot_size slot that fits after the alignment padding (see struct memArray). */
struct memArray	ArenaPushAlignedAll(struct arena *ar, const u64 slot_size, const u64 alignment);

/* Packed: no alignment. Default: DEFAULT_MEMORY_ALIGNMENT (8) bytes. */
#define		ArenaPushPacked(ar, size)		ArenaPushAligned(ar, size, 1)
#define		ArenaPushPackedZero(ar, size)		ArenaPushAlignedZero(ar, size, 1)
#define		ArenaPushPackedMemcpy(ar, copy, size)	ArenaPushAlignedMemcpy(ar, copy, size, 1)

#define		ArenaPush(ar, size)			ArenaPushAligned(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushZero(ar, size)			ArenaPushAlignedZero(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushMemcpy(ar, copy, size)		ArenaPushAlignedMemcpy(ar, copy, size, DEFAULT_MEMORY_ALIGNMENT)

#endif

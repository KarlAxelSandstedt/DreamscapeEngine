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

#ifndef __DS_BITSET_H__
#define __DS_BITSET_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

#define DS_BITSET_BLOCKSIZE         8
#define DS_BITSET_BLOCK_BITCOUNT    (8*DS_BITSET_BLOCKSIZE)	
	
struct ds_BitSet 
{
	u64 		        block_count;
	u64 		        bit_count;
	u64 * 		        bits;
	u32 		        growable;
	struct ds_MemSlot 	mem_slot;
};

/* Return a bitset with bit_count >= bit_count, with all bits cleared to clear_bit. On failure, an empty bitset is returned. */
struct ds_BitSet	ds_BitSetAlloc(struct arena *mem, const u64 bit_count, const u64 clear_bit, const u32 growable);
/* ds_BitSetAlloc without clearing the bits. */
struct ds_BitSet	ds_BitSetAllocEx(struct arena *mem, const u64 bit_count, const u32 growable);
/* Deallocate the set.  */
void 		        ds_BitSetDealloc(struct ds_BitSet *set);
/* Increase the set's size and set any newly allocated blocks with the clear bit. */
void 		        ds_BitSetIncreaseSize(struct ds_BitSet *set, const u64 bit_count, const u64 clear_bit);
/* Clear the set with the given bit value  */
void 		        ds_BitSetClear(struct ds_BitSet* set, const u64 clear_bit);
/* Return the required size when serializing the bitset. */
u64                 ds_BitSetSerializeSize(const struct ds_BitSet *set);
/* Serialize the bitset. WARNING: Assumes set fits in the stream. */
void                ds_BitSetSerialize(struct ss *ss, const struct ds_BitSet *set);
/* 
 * Returns 1 on success and 0 on failure. On failure, the set is set to empty. 
 * Failure may occur due to out-of-memory in mem, or the stream doesn't contain
 * a bitset. 
 */
u32                 ds_BitSetTryDeserialize(struct arena *mem, struct ds_BitSet *set, struct ss *ss, const u32 growable);

/* Return the bit value of the given bit. Indexing starts at 0.  */
static inline uint8_t ds_BitSetGet(const struct ds_BitSet* set, const u64 bit)
{
	ds_Assert(bit < set->bit_count);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	return (set->bits[block] >> block_bit) & 0x1;
}

/* Set the bit value of the given bit. Indexing starts at 0. */
static inline void ds_BitSetSet(struct ds_BitSet* set, const u64 bit, const u64 bit_value)
{
	ds_Assert(bit < set->bit_count && bit_value <= 1);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	/* Get all bits in block but set wanted bit to zero */
	const u64 mask = ~((u64) 0x1 << block_bit); 
	set->bits[block] = (set->bits[block] & mask) | (bit_value << block_bit);
}

struct ds_BitBlock
{
    u64 block;
    u64 bit;
};

/* Setup a block iterator that iterates over the given bit value.  */
static inline struct ds_BitBlock ds_BitBlockInit(const u64 block, const u64 block_index)
{
    const struct ds_BitBlock it =
    {
        .block = block,
        .bit = block_index*DS_BITSET_BLOCK_BITCOUNT,
    };
    return it;
}

/* Return true if the iterator has any more bits to iterate over */
static inline u64 ds_BitBlockHasNext(const struct ds_BitBlock *it)
{
    return it->block;
}

/* Advance the iterator */
static inline u64 ds_BitBlockNext(struct ds_BitBlock *it)
{
    ds_Assert(ds_BitBlockHasNext(it));

	const u64 tzc = Ctz64(it->block);
    const u64 bit = it->bit + tzc;
    it->bit += tzc + 1;
	it->block = (tzc < 63) 
		      ? it->block >> (tzc + 1)
		      : 0;

    return bit;
}

/* Peek the next bit in the iterator */
static inline u64 ds_BitBlockPeekNext(const struct ds_BitBlock *it)
{
    ds_Assert(ds_BitBlockHasNext(it));

	const u64 tzc = Ctz64(it->block);
    const u64 bit = it->bit + tzc;
    return bit;
}

#ifdef __cplusplus
} 
#endif

#endif

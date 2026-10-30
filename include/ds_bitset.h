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
ds_bitset.h
===========
Bit sets in u64 blocks, and an iterator over the set bits of one block.

Contents (in this order):
    ds_BitSet   bits[block_count]: get, set, clear all, grow (heap sets), serialize.
    ds_BitBlock the indices of the 1-bits of one block, lowest first.
*/

#ifndef __DS_BITSET_H__
#define __DS_BITSET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

/*
ds_BitSet
---------
    struct ds_BitSet set = ds_BitSetAlloc(NULL, 1000, 0, GROWABLE);
    ds_BitSetSet(&set, 17, 1);
    if (ds_BitSetGet(&set, 17)) { ... }
    ds_BitSetDealloc(&set);

Bit i is bit i % 64 of bits[i / 64]. bit_count is always block_count*64: sizes round up to whole blocks.
Not thread-safe.
*/

#define DS_BITSET_BLOCKSIZE         8
#define DS_BITSET_BLOCK_BITCOUNT    (8*DS_BITSET_BLOCKSIZE)

struct ds_BitSet
{
	u64 		        block_count;
	u64 		        bit_count;
	u64 * 		        bits;
	u32 		        growable;
	u32		            heap_allocated;	/* bits is a ds_Alloc block */
};

/*
Return a set of at least bit_count bits on mem (fixed; growable must be 0) or on the heap (mem == NULL),
every bit clear_bit (0 or 1). On failure, or for bit_count 0, an empty set (bits NULL, counts 0).
*/
struct ds_BitSet	ds_BitSetAlloc(struct arena *mem, const u64 bit_count, const u64 clear_bit, const u32 growable);
/* ds_BitSetAlloc with the bits uninitialized. */
struct ds_BitSet	ds_BitSetAllocEx(struct arena *mem, const u64 bit_count, const u32 growable);
/* Free a heap set's bits (an arena set's stay with the arena); the set becomes empty. */
void 		        ds_BitSetDealloc(struct ds_BitSet *set);
/*
Grow a growable set to at least bit_count (> set->bit_count) bits; the new bits are clear_bit, the old
ones kept. Fatal if the memory can't be allocated.
*/
void 		        ds_BitSetIncreaseSize(struct ds_BitSet *set, const u64 bit_count, const u64 clear_bit);
/* Set every bit to clear_bit (0 or 1). */
void 		        ds_BitSetClear(struct ds_BitSet* set, const u64 clear_bit);
/* Bytes ds_BitSetSerialize writes. */
u64                 ds_BitSetSerializeSize(const struct ds_BitSet *set);
/* Write the set to ss (at a byte boundary; it must fit). Format (LE): u64 block_count, the blocks. */
void                ds_BitSetSerialize(struct ss *ss, const struct ds_BitSet *set);
/*
Read a set from ss into mem (fixed) or the heap (mem == NULL; growable allowed). Returns 1 on success; 0
if the memory can't be allocated or ss holds no valid set (the set is empty, ss's position unchanged).
*/
u32                 ds_BitSetTryDeserialize(struct arena *mem, struct ss *ss, struct ds_BitSet *set, const u32 growable);
/* Return bit (< bit_count), 0 or 1. */
static inline u8    ds_BitSetGet(const struct ds_BitSet* set, const u64 bit);
/* Set bit (< bit_count) to bit_value (0 or 1). */
static inline void  ds_BitSetSet(struct ds_BitSet* set, const u64 bit, const u64 bit_value);

/*
ds_BitBlock
-----------
    for (u64 b = 0; b < set.block_count; ++b)
    {
        struct ds_BitBlock it = ds_BitBlockInit(set.bits[b], b);
        while (ds_BitBlockHasNext(&it))
        {
            const u64 bit = ds_BitBlockNext(&it);       (set index of a 1-bit, ascending)
            ...
        }
    }

The iterator visits the 1-bits of block; pass ~block for the 0-bits.
*/

struct ds_BitBlock
{
    u64 block;
    u64 bit;
};

/* Return an iterator over the 1-bits of block, the block_index-th block of a set (indices are set indices). */
static inline struct ds_BitBlock ds_BitBlockInit(const u64 block, const u64 block_index);
/* Nonzero if a 1-bit is left. */
static inline u64   ds_BitBlockHasNext(const struct ds_BitBlock *it);
/* Return the next 1-bit's set index and advance; a 1-bit must be left. */
static inline u64   ds_BitBlockNext(struct ds_BitBlock *it);
/* Return the next 1-bit's set index without advancing; a 1-bit must be left. */
static inline u64   ds_BitBlockPeekNext(const struct ds_BitBlock *it);

/******************************** ds_BitSet inline definitions ********************************/

static inline u8 ds_BitSetGet(const struct ds_BitSet* set, const u64 bit)
{
	ds_Assert(bit < set->bit_count);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	return (set->bits[block] >> block_bit) & 0x1;
}

static inline void ds_BitSetSet(struct ds_BitSet* set, const u64 bit, const u64 bit_value)
{
	ds_Assert(bit < set->bit_count && bit_value <= 1);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	/* Get all bits in block but set wanted bit to zero */
	const u64 mask = ~((u64) 0x1 << block_bit);
	set->bits[block] = (set->bits[block] & mask) | (bit_value << block_bit);
}

/******************************** ds_BitBlock inline definitions ********************************/

static inline struct ds_BitBlock ds_BitBlockInit(const u64 block, const u64 block_index)
{
    const struct ds_BitBlock it =
    {
        .block = block,
        .bit = block_index*DS_BITSET_BLOCK_BITCOUNT,
    };
    return it;
}

static inline u64 ds_BitBlockHasNext(const struct ds_BitBlock *it)
{
    return it->block;
}

static inline u64 ds_BitBlockNext(struct ds_BitBlock *it)
{
    ds_Assert(ds_BitBlockHasNext(it));

	const u64 tzc = Ctz64(it->block);
    const u64 bit = it->bit + tzc;
    it->bit += tzc + 1;
	/* a shift by 64 is undefined: the top bit was the last one */
	it->block = (tzc < 63)
		      ? it->block >> (tzc + 1)
		      : 0;

    return bit;
}

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

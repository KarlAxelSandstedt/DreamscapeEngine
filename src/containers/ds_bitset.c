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

#include "ds_base.h"
#include "ds_bitset.h"

/* bit indexing starts at 0, up to bit_count-1. */
const struct ds_BitSet set_empty = { 0 };

static void ds_BitsetStaticAssert(void)
{
    ds_StaticAssert(sizeof(((struct ds_BitSet *)0)->bits[0]) == DS_BITSET_BLOCKSIZE, "");
	ds_StaticAssert(DS_BITSET_BLOCK_BITCOUNT == 64, "must be a power of two");
}

struct ds_BitSet ds_BitSetAlloc(struct arena *mem, const u64 bit_count, const u64 clear_bit, const u32 growable)
{
	ds_AssertString(bit_count >= 1 && clear_bit <= 1, "invalid ds_BitSetAlloc bit count or clear bit value");
	ds_Assert(!(mem && growable));

	const u64 mod = (bit_count % DS_BITSET_BLOCK_BITCOUNT);
	const u64 bit_count_req = (mod) ? bit_count + (DS_BITSET_BLOCK_BITCOUNT - mod) : bit_count; 


	struct ds_BitSet set = 
	{ 
		.block_count = bit_count_req / DS_BITSET_BLOCK_BITCOUNT,
		.bit_count = bit_count_req,
		.growable = growable,
	};

	if (mem)
	{
		set.bits = ArenaPush(mem, set.block_count * sizeof(u64));
	}
	else
	{
		set.bits = ds_Alloc(&set.mem_slot, set.block_count * sizeof(u64), NO_HUGE_PAGES);
		set.block_count = set.mem_slot.size / sizeof(u64);
		set.bit_count = set.block_count * DS_BITSET_BLOCK_BITCOUNT;
	}

	if (set.bits == NULL)
	{
		return (struct ds_BitSet) { 0 };
	}

	for (u64 block = 0; block < set.block_count; ++block) 
	{
		set.bits[block] = U64_MAX * clear_bit;
	}

	return set;
}

void ds_BitSetDealloc(struct ds_BitSet *set)
{
	ds_Free(&set->mem_slot);
}

void ds_BitSetIncreaseSize(struct ds_BitSet *set, const u64 bit_count, const u64 clear_bit)
{
	ds_Assert(set->bit_count < bit_count);
	ds_Assert(set->growable);

	const u64 new_blocks_start = set->block_count;
	const u64 mod = (bit_count % DS_BITSET_BLOCK_BITCOUNT);

	u64 new_bit_count = (mod) ? bit_count + (DS_BITSET_BLOCK_BITCOUNT - mod) : bit_count; 
	u64 new_block_count = new_bit_count / DS_BITSET_BLOCK_BITCOUNT;

	set->bits = ds_Realloc(&set->mem_slot, new_block_count * sizeof(u64));
	set->block_count = set->mem_slot.size / sizeof(u64);
	set->bit_count = set->block_count * DS_BITSET_BLOCK_BITCOUNT;

	for (u64 i = new_blocks_start; i < set->block_count; ++i)
	{
		set->bits[i] = U64_MAX * clear_bit; 
	}
}

uint8_t ds_BitSetGet(const struct ds_BitSet* set, const u64 bit)
{
	ds_Assert(bit < set->bit_count);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	return (set->bits[block] >> block_bit) & 0x1;
}

void ds_BitSetSet(const struct ds_BitSet* set, const u64 bit, const u64 bit_value)
{
	ds_Assert(bit < set->bit_count && bit_value <= 1);

	const u64 block = bit / DS_BITSET_BLOCK_BITCOUNT;
	const u64 block_bit = bit % DS_BITSET_BLOCK_BITCOUNT;

	/* Get all bits in block but set wanted bit to zero */
	const u64 mask = ~((u64) 0x1 << block_bit); 
	set->bits[block] = (set->bits[block] & mask) | (bit_value << block_bit);
}

void ds_BitSetClear(struct ds_BitSet* set, const u64 clear_bit)
{
	for (u64 block = 0; block < set->block_count; ++block) 
	{
		set->bits[block] = U64_MAX * clear_bit;
	}
}

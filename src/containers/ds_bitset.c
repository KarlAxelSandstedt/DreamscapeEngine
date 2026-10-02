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

static void ds_BitsetStaticAssert(void)
{
	ds_StaticAssert(sizeof(((struct ds_BitSet *)0)->bits[0]) == DS_BITSET_BLOCKSIZE, "");
	ds_StaticAssert(DS_BITSET_BLOCK_BITCOUNT == 64, "");
}

struct ds_BitSet ds_BitSetAllocEx(struct arena *mem, const u64 bit_count, const u32 growable)
{
	ds_Assert(!(mem && growable));

	const u64 block_count = (bit_count + DS_BITSET_BLOCK_BITCOUNT - 1) / DS_BITSET_BLOCK_BITCOUNT;
	struct ds_BitSet set = 
	{ 
		.block_count = block_count,
		.bit_count = block_count * DS_BITSET_BLOCK_BITCOUNT,
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
		set.block_count = 0;
		set.bit_count = 0;
	}

	return set;
}

struct ds_BitSet ds_BitSetAlloc(struct arena *mem, const u64 bit_count, const u64 clear_bit, const u32 growable)
{
	struct ds_BitSet set = ds_BitSetAllocEx(mem, bit_count, growable);
    ds_BitSetClear(&set, clear_bit);
	return set;
}

void ds_BitSetDealloc(struct ds_BitSet *set)
{
	ds_Free(&set->mem_slot);
}

void ds_BitSetIncreaseSize(struct ds_BitSet *set, const u64 bit_count, const u64 clear_bit)
{
	ds_Assert(set->bit_count < bit_count && clear_bit <= 1);
	ds_Assert(set->growable);

	const u64 new_blocks_start = set->block_count;
	const u64 new_block_count = (bit_count + DS_BITSET_BLOCK_BITCOUNT - 1) / DS_BITSET_BLOCK_BITCOUNT;

	set->bits = ds_Realloc(&set->mem_slot, new_block_count * sizeof(u64));
	set->block_count = set->mem_slot.size / sizeof(u64);
	set->bit_count = set->block_count * DS_BITSET_BLOCK_BITCOUNT;

	for (u64 i = new_blocks_start; i < set->block_count; ++i)
	{
		set->bits[i] = U64_MAX * clear_bit;
	}
}

void ds_BitSetClear(struct ds_BitSet* set, const u64 clear_bit)
{
	ds_Assert(clear_bit <= 1);

	for (u64 block = 0; block < set->block_count; ++block) 
	{
		set->bits[block] = U64_MAX * clear_bit;
	}
}

u64 ds_BitSetSerializeSize(const struct ds_BitSet *set)
{
	return sizeof(u64) + set->block_count*sizeof(u64);
}

void ds_BitSetSerialize(struct ss *ss, const struct ds_BitSet *set)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(ds_BitSetSerializeSize(set) <= ss_BytesLeft(ss));

	ss_WriteU64Le(ss, set->block_count);
	ss_WriteU64LeN(ss, set->bits, set->block_count);
}

u32 ds_BitSetTryDeserialize(struct arena *mem, struct ds_BitSet *set, struct ss *ss, const u32 growable)
{
	ds_Assert(ss->bit_index % 8 == 0);

	*set = (struct ds_BitSet) { 0 };
	if (ss_BytesLeft(ss) < sizeof(u64))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u64 block_count = ss_ReadU64Le(ss);
	/* division instead of block_count*sizeof(u64): a corrupt block_count must not overflow the check */
	if (block_count > ss_BytesLeft(ss) / sizeof(u64))
	{
		ss->bit_index = bit_index;
		return 0;
	}

	*set = ds_BitSetAllocEx(mem, block_count*DS_BITSET_BLOCK_BITCOUNT, growable);
	if (set->bits == NULL && block_count)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	/* A heap allocation may round the capacity up; keep the serialized block count exact */
	set->block_count = block_count;
	set->bit_count = block_count*DS_BITSET_BLOCK_BITCOUNT;
	ss_ReadU64LeN(set->bits, ss, block_count);

	return 1;
}

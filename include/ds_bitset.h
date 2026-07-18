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
/* deallocate the set.  */
void 		        ds_BitSetDealloc(struct ds_BitSet *set);
/* increase the set's size and set any newly allocated blocks with the clear bit. */
void 		        ds_BitSetIncreaseSize(struct ds_BitSet *set, const u64 bit_count, const u64 clear_bit);
/* Clear the set with the given bit value  */
void 		        ds_BitSetClear(struct ds_BitSet* set, const u64 clear_bit);
/* return the bit value of the given bit  */
uint8_t 	        ds_BitSetGet(const struct ds_BitSet* set, const u64 bit);
/* set the bit value of the given bit */
void 		        ds_BitSetSet(const struct ds_BitSet* set, const u64 bit, const u64 bit_value);

#ifdef __cplusplus
} 
#endif

#endif

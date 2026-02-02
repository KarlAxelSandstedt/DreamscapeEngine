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

#include <stdlib.h>

#include <ds_base.h>

/*
	===== Derivation of Lookup table for bytes touched =====

input: 		bits_left_in_initial_byte, bit_count (to read or write)
output: 	bytes_touched

Case 1:
If bits_left_in_inital_byte <= bit_count  Then 

		ds_Assert(bit_count <= 64);
							
							bits:	  543 210
		(bit_count - bits_left_in_initial_byte) = 	  BBB bbb
		bytes_touched = 1;
		bytes_touched += BBB	
		bytes_touched += (bbb) ? 1 : 0	

Case 2:
If bit_count < bits_left_in_initial_byte Then 
							bits:	  543 210
		(bit_count - bits_left_in_initial_byte) =   111...111 bbb
										3 210
		where bbb + abs_diff(bit_count, bits_left_in_initial_byte) ==   1 000

							     abs_diff
		1 - 2	=== 111...111 111	(Check 111 + 001 	== 1 000)
		1 - 3	=== 111...111 110	(Check 110 + 010 	== 1 000)
		1 - 4	=== 111...111 101	(Check 101 + 011 	== 1 000)
		1 - 5	=== 111...111 100	(Check 100 + 100 	== 1 000)
		1 - 6	=== 111...111 011	(Check 011 + 101 	== 1 000)
		1 - 7	=== 111...111 010	(Check 010 + 110 	== 1 000)
		1 - 8	=== 111...111 001	(Check 001 + 111 	== 1 000)


Implementing Case1 we get:
If bits_left_in_inital_byte <= bit_count  Then 
							bits:	   543 210
		diff = (bit_count - bits_left_in_initial_byte) =   BBB bbb

		static lookup[8] = { 0, 1, 1, 1, 1, 1, 1, 1 };
		bytes_touched = (1 + (diff >> 3) + lookup[diff & 0x7])
			      = ((8 >> 3) + (diff >> 3) + lookup[diff & 0x7])
			      = ((8 >> 3) + (diff >> 3) + lookup[diff & 0x7])
			      = (((diff+8) >> 3) + lookup[diff & 0x7])
			      = (((diff+8) >> 3) + lookup[(diff+8) & 0x7])

Implementing Case2 we get:
If bits_left_in_inital_byte <= bit_count  Then 

		diff 	 = (bit_count - bits_left_in_initial_byte) 

			bits:	     543 210
			 =   111 ... 111 bbb

			bits:	     543 210
	=> 	diff + 8 =   000 ... 000 bbb

		where bbb > 0:

		We know that bytes_touched = 1 so the yields the correct result:

		static lookup[8] = { 0, 1, 1, 1, 1, 1, 1, 1 };
		bytes_touched = (((diff+8) >> 3) + lookup[(diff+8) & 0x7])
			      = (0 + lookup[bbb])
			      = (0 + 1)
			      = 1

We now see that we can unify case 1 and case 2 into a single unified execution path:

Execution Path:
		static lookup[8] = { 0, 1, 1, 1, 1, 1, 1, 1 };
		val  = (bit_count - bits_left_in_initial_byte) + 8;
		bytes_touched = ((val >> 3) + lookup[val & 0x7])
					

Now, since bit_count - bits_left_in_initial_byte + 8 lies between the range

	[ 1-8+8, 64-1+8 ] = [ 1, 71 ]
or,	[ 1-8, 64-1 ] = [ -7, 63 ]

we can setup a Execution path lookup table for all computations:

table[71] = { ((val >> 3) + lookup[val & 0x7]) : val = index+1 }
*bytes_touched_lookup = table + 7

Then:
	bytes_touched = bytes_touched_lookup[bit_count - bits_left_in_initial_byte];
		      = table[bit_count - bits_left_in_initial_byte + 7]
		      = ((bit_count - bits_left_in_initial_byte + 8) >> 3) + lookup[(bit_count - bits_left_in_initial_byte + 8) & 0x7]


	We may now choose between

		1 subtraction + 1 lookup
	vs. 	2 addition + 1 subtraction + 1 bit shift + 1 and + 1 lookup
*/

static const u32 bytes_touched_table[71] =
{
	       1, 1, 1, 1, 1, 1, 1,
	    1, 2, 2, 2, 2, 2, 2, 2,
	    2, 3, 3, 3, 3, 3, 3, 3,
	    3, 4, 4, 4, 4, 4, 4, 4,
	    4, 5, 5, 5, 5, 5, 5, 5,
	    5, 6, 6, 6, 6, 6, 6, 6,
	    6, 7, 7, 7, 7, 7, 7, 7,
	    7, 8, 8, 8, 8, 8, 8, 8,
	    8, 9, 9, 9, 9, 9, 9, 9,
};

/* aligned_table[stream->bit_index & 0x7] = byte_alignment */
static const u32 aligned_table[8] = { 1, 0, 0, 0, 0, 0, 0, 0, };

static const u32 *bytes_touched_lookup = bytes_touched_table + 7;

struct serialStream ss_Alloc(struct arena *mem, const u64 bufsize)
{
	struct serialStream ss = { 0 };

	u64 actual_bufsize;
	if (mem)
	{
		actual_bufsize = bufsize;
		ss.buf = ArenaPush(mem, actual_bufsize);
	}
	else
	{
		actual_bufsize = ds_AllocSizeCeil(bufsize);
		ss.buf = (actual_bufsize >= 1024*1024) 
			? ds_Alloc(&ss.mem_slot, actual_bufsize, HUGE_PAGES)
			: ds_Alloc(&ss.mem_slot, actual_bufsize, NO_HUGE_PAGES);
	}

	if (ss.buf)
	{
		ss.bit_count = actual_bufsize * 8;
		ss.bit_index = 0;	
	}

	return ss;
}

struct serialStream ss_Buffered(void *buf, const u64 bufsize)
{
	struct serialStream ss =
	{
		.bit_count = bufsize * 8,
		.bit_index = 0,
		.buf = buf,
	};

	return ss;
}

u64 ss_BytesLeft(const struct serialStream *ss)
{
	return (ss->bit_count - ss->bit_index) >> 3;
}

u64 ss_BitsLeft(const struct serialStream *ss)
{
	return ss->bit_count - ss->bit_index;
}

static inline b16 EndianShift16(const b16 be)
{
	const b16 shift = { .u =  (be.u << 8) | (be.u >> 8), };
	return shift; 
}

static inline b32 EndianShift32(const b32 le)
{
	const u32 mask1 = 0x00ff00ff;
	const u32 mask2 = 0xff00ff00;

	b32 shift = { .u = ((le.u & mask1) << 8) | ((le.u & mask2) >> 8) };
	shift.u = (shift.u << 16) | (shift.u >> 16);

	return shift; 
}

static inline b64 EndianShift64(const b64 le)
{
	const u64 mask1 = 0x0000ffff0000ffff;
	const u64 mask2 = 0xffff0000ffff0000;
	const u64 mask3 = 0x00ff00ff00ff00ff;
	const u64 mask4 = 0xff00ff00ff00ff00;

	/* 5 4 7 6 1 0 3 2 */
	b64 shift = { .u = ((le.u & mask1) << 16) | ((le.u & mask2) >> 16) };
	/* 4 5 6 7 0 1 2 3 */
	shift.u = ((shift.u & mask3) << 8) | ((shift.u & mask4) >> 8);
	/* 0 1 2 3 4 5 6 7 */
	shift.u = (shift.u << 32) | (shift.u >> 32);

	return shift; 
}

#if defined(DS_LITTLE_ENDIAN)
#define		NaToLe16(val)	(val)
#define		NaToLe32(val)	(val)
#define		NaToLe64(val)	(val)
#define		NaToBe16(val)	EndianShift16(val)	
#define		NaToBe32(val)	EndianShift32(val)	
#define		NaToBe64(val)	EndianShift64(val)	
#define		LeToNa16(val)	(val)
#define		LeToNa32(val)	(val)
#define		LeToNa64(val)	(val)
#define		BeToNa16(val)	EndianShift16(val)	
#define		BeToNa32(val)	EndianShift32(val)	
#define		BeToNa64(val)	EndianShift64(val)	
#elif defined(DS_BIG_ENDIAN)
#define		NaToLe16(val)	EndianShift16(val)	
#define		NaToLe32(val)	EndianShift32(val)	
#define		NaToLe64(val)	EndianShift64(val)	
#define		NaToBe16(val)	(val)
#define		NaToBe32(val)	(val)
#define		NaToBe64(val)	(val)
#define		LeToNa16(val)	EndianShift16(val)	
#define		LeToNa32(val)	EndianShift32(val)	
#define		LeToNa64(val)	EndianShift64(val)	
#define		BeToNa16(val)	(val)
#define		BeToNa32(val)	(val)
#define		BeToNa64(val)	(val)
#endif

void ss_Free(struct serialStream *ss)
{
	if (ss->mem_slot.address)
	{
		ds_Free(&ss->mem_slot);
	}
}

b8 ss_Read8(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	b8 val = { .u = ss->buf[offset], };
	return val;
}

void ss_Write8(struct serialStream *ss, const b8 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	*((b8 *) (ss->buf + offset)) = val;
}

b16 ss_Read16Le(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	b16 val = LeToNa16(*((b16 *) (ss->buf + offset)));
	return val;
}

void ss_Write16Le(struct serialStream *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	*((b16 *) (ss->buf + offset)) = NaToLe16(val);
}

b16 ss_Read16Be(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	b16 val = BeToNa16(*((b16 *) (ss->buf + offset)));
	return val;
}

void ss_Write16Be(struct serialStream *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	*((b16 *) (ss->buf + offset)) = NaToBe16(val);
}

b32 ss_Read32Le(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	b32 val = LeToNa32(*((b32 *) (ss->buf + offset)));
	return val;
}

void ss_Write32Le(struct serialStream *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	*((b32 *) (ss->buf + offset)) = NaToLe32(val);
}

b32 ss_Read32Be(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	b32 val = BeToNa32(*((b32 *) (ss->buf + offset)));
	return val;
}

void ss_Write32Be(struct serialStream *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	*((b32 *) (ss->buf + offset)) = NaToBe32(val);
}

b64 ss_Read64Le(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	b64 val = LeToNa64(*((b64 *) (ss->buf + offset)));
	return val;
}

void ss_Write64Le(struct serialStream *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	*((b64 *) (ss->buf + offset)) = NaToLe64(val);
}

b64 ss_Read64Be(struct serialStream *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	b64 val = BeToNa64(*((b64 *) (ss->buf + offset)));
	return val;
}

void ss_Write64Be(struct serialStream *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	*((b64 *) (ss->buf + offset)) = NaToBe64(val);
}

void ss_Read8N(b8 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 8*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8*len;
	const b8 *src = (b8 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = src[i];
	}
}

void ss_Write8N(struct serialStream *ss, const b8 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 8*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8*len;
	b8 *dst = (b8 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = buf[i];
	}
}

void ss_Read16LeN(b16 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	const b16 *src = (b16 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = LeToNa16(src[i]);
	}
}

void ss_Write16LeN(struct serialStream *ss, const b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	b16 *dst = (b16 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToLe16(buf[i]);
	}
}

void ss_Read16BeN(b16 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	const b16 *src = (b16 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = BeToNa16(src[i]);
	}
}

void ss_Write16BeN(struct serialStream *ss, const b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	b16 *dst = (b16 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToBe16(buf[i]);
	}
}

void ss_Read32LeN(b32 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	const b32 *src = (b32 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = LeToNa32(src[i]);
	}
}

void ss_Write32LeN(struct serialStream *ss, const b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	b32 *dst = (b32 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToLe32(buf[i]);
	}
}

void ss_Read32BeN(b32 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	const b32 *src = (b32 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = BeToNa32(src[i]);
	}
}

void ss_Write32BeN(struct serialStream *ss, const b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	b32 *dst = (b32 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToBe32(buf[i]);
	}
}

void ss_Read64LeN(b64 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	const b64 *src = (b64 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = LeToNa64(src[i]);
	}
}

void ss_Write64LeN(struct serialStream *ss, const b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	b64 *dst = (b64 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToLe64(buf[i]);
	}
}

void ss_Read64BeN(b64 *buf, struct serialStream *ss, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	const b64 *src = (b64 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		buf[i] = BeToNa64(src[i]);
	}
}

void ss_Write64BeN(struct serialStream *ss, const b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	b64 *dst = (b64 *) (ss->buf + offset);
	for (u64 i = 0; i < len; ++i)
	{
		dst[i] = NaToBe64(buf[i]);
	}
}

/*
 * internal_read/write_*_aligned is called when we are reading/writing from a byte aligned offset.
 * internal_read/write_*_straddling is called when we are reading/writing from a byte unaligned offset.
 *
 * For straddling methods, 
 * 	byte_lower_mask - upper part of stradling byte found in lower part of ptr[n]
 * 	byte_upper_mask - lower part of stradling byte found in upper part of tr[n+1]
 *
 * 	byte_lower_mask_size - size within [1,7] of lower mask
 */

#define STRADDLE_BYTE(byte, byte_lower_mask, byte_lower_mask_size)		\
	(									\
    	     ((ptr[byte] & byte_lower_mask) << (8 - byte_lower_mask_size)) 	\
           | ((ptr[byte+1] & ~byte_lower_mask) >> byte_lower_mask_size)		\
	)	

/* 
 * required right shift of last STRADDLE_BYTE read
 * bit_count-8 is in [-6, 8] (15 values) [  bit_count >= 2 since we read 2 bytes ] 
 *
 * Example: [internal_read_le_partial_3_straddling]
 *
 * We read two STRADDLE_BYTE, but we can only be sure of the first one being fully used.
 * at minimum, we may read 10 bits from 3 bytes (1 bit, 8 bits, 1 bit)  and at most 24 bits.
 * We access the shift lookup table as follows:
 *
 * 	  shift_lookup[bit_count - 8 * count(STRADDLE_BYTE)]
 * 	= shift_lookup[bit_count - 16]
 *
 * the reasoning of the table is as follows:
 *
 *	shift_lookup[10-16] = 6	(wants 2 bits in straddle byte 2 => shift 6)
 *	shift_lookup[11-16] = 5	(wants 3 bits in straddle byte 2 => shift 5)
 *	shift_lookup[12-16] = 4	(wants 4 bits in straddle byte 2 => shift 4)
 *	shift_lookup[13-16] = 3	(wants 5 bits in straddle byte 2 => shift 3)
 *
 *	shift_lookup[14-16] = 2	(wants 6 bits in straddle byte 2 => shift 2)
 *	shift_lookup[15-16] = 1	(wants 7 bits in straddle byte 2 => shift 1)
 *	shift_lookup[16-16] = 0	(wants 8 bits in straddle byte 2 => shift 0)
 *	shift_lookup[17-16] = 0 
 *
 *	shift_lookup[18-16] = 0
 *	shift_lookup[19-16] = 0
 *	shift_lookup[20-16] = 0
 *	shift_lookup[21-16] = 0
 *
 *	shift_lookup[22-16] = 0
 *	shift_lookup[23-16] = 0
 *	shift_lookup[24-16] = 0
 */

static const u32 shift_table[15] =  
{
	6,   5,   4,   3,   2,   1,   0,   0,
        0,   0,   0,   0,   0,   0,   0,    
};
static const u32 *shift_lookup = shift_table + 6;

/*
 * To understand the bit shifting of the last straddling byte, stare long enough at the
 * operations below...
 *
 * 	byte_lower_mask_size = 1, 
 *
 * 	6 => [xxxx xxxb] [bxxx xxxx]	(shift 1)
 * 	5 => [xxxx xxxb] [bbxx xxxx]    (shift 2)
 * 	4 => [xxxx xxxb] [bbbx xxxx] 	(shift 3)
 * 	3 => [xxxx xxxb] [bbbb xxxx]	(shift 4)
 * 	2 => [xxxx xxxb] [bbbb bxxx]    (shift 5)
 * 	1 => [xxxx xxxb] [bbbb bbxx] 	(shift 6)
 * 	0 => [xxxx xxxb] [bbbb bbbx]	(shift 7)
 *     -1 => [xxxx xxxb] [bbbb bbbb]    (shift 7)
 *
 * 	byte_lower_mask_size = 2, 
 *
 * 	5 => [xxxx xxbb] [bxxx xxxx]	(shift 1)
 * 	4 => [xxxx xxbb] [bbxx xxxx]    (shift 2)
 * 	3 => [xxxx xxbb] [bbbx xxxx] 	(shift 3)
 * 	2 => [xxxx xxbb] [bbbb xxxx]	(shift 4)
 * 	1 => [xxxx xxbb] [bbbb bxxx]    (shift 5)
 * 	0 => [xxxx xxbb] [bbbb bbxx] 	(shift 6)
 *     -1 => [xxxx xxbb] [bbbb bbbx]	(shift 6)
 *     -2 => [xxxx xxbb] [bbbb bbbb]    (shift 6)
 *
 * 	byte_lower_mask_size = 3, 
 *
 * 	4 => [xxxx xbbb] [bxxx xxxx]	(shift 1)
 * 	3 => [xxxx xbbb] [bbxx xxxx]    (shift 2)
 * 	2 => [xxxx xbbb] [bbbx xxxx] 	(shift 3)
 * 	1 => [xxxx xbbb] [bbbb xxxx]	(shift 4)
 * 	0 => [xxxx xbbb] [bbbb bxxx]    (shift 5)
 *     -1 => [xxxx xbbb] [bbbb bbxx] 	(shift 5)
 *     -2 => [xxxx xbbb] [bbbb bbbx]	(shift 5)
 *     -3 => [xxxx xbbb] [bbbb bbbb]    (shift 5)
 *
 *	byte_lower_mask_size = 1
 *	[1][new_lower_mask_size] = 
 *     	{ 
 *     	   7  7  6  5  4  3  2  1 	
 *     	}
 *
 *	byte_lower_mask_size = 2
 *	[2][new_lower_mask_size] = 
 *     	{ 
 *     	   6  6  6  5  4  3  2  1 	
 *     	}
 *
 *	byte_lower_mask_size = 3
 *	[3][new_lower_mask_size] = 
 *     	{ 
 *     	   5  5  5  5  4  3  2  1
 *     	}
 *
 *	byte_lower_mask_size = 4
 *	[4][new_lower_mask_size] = 
 *     	{ 
 *     	   4  4  4  4  4  3  2  1 	
 *     	}
 *
 *	byte_lower_mask_size = 5
 *	[5][new_lower_mask_size] = 
 *     	{ 
 *     	   3  3  3  3  3  3  2  1 	
 *     	}
 *
 *	byte_lower_mask_size = 6
 *	[6][new_lower_mask_size] = 
 *     	{ 
 *     	   2  2  2  2  2  2  2  1 	
 *     	}
 *
 *	byte_lower_mask_size = 7
 *	[7][new_lower_mask_size] = 
 *     	{ 
 *     	   1  1  1  1  1  1  1  1 	
 *     	}
 */                                  

/*	[byte_lower_mask_size][new_lower_mask_size]	*/
static const u32 lower_write_shift_lookup[8][8] =
{
	{ 0, 0, 0, 0, 0, 0, 0, 0, },
	{ 7, 7, 6, 5, 4, 3, 2, 1, },
	{ 6, 6, 6, 5, 4, 3, 2, 1, },
	{ 5, 5, 5, 5, 4, 3, 2, 1, },
	{ 4, 4, 4, 4, 4, 3, 2, 1, },
	{ 3, 3, 3, 3, 3, 3, 2, 1, },
	{ 2, 2, 2, 2, 2, 2, 2, 1, },
	{ 1, 1, 1, 1, 1, 1, 1, 1, },
};

static const u32 upper_write_shift_lookup[8][8] =
{
	{ 0, 0, 0, 0, 0, 0, 0, 0, },
	{ 1, 1, 2, 3, 4, 5, 6, 7, },
	{ 2, 2, 2, 3, 4, 5, 6, 7, },
	{ 3, 3, 3, 3, 4, 5, 6, 7, },
	{ 4, 4, 4, 4, 4, 5, 6, 7, },
	{ 5, 5, 5, 5, 5, 5, 6, 7, },
	{ 6, 6, 6, 6, 6, 6, 6, 7, },
	{ 7, 7, 7, 7, 7, 7, 7, 7, },
};
 
static u64 internal_read_1_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] >> (8-bit_count);
}

static u64 internal_read_2_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| (((u64) ptr[1] >> (16-bit_count)) << 8);
}

static u64 internal_read_3_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| (((u64) ptr[2] >> (24-bit_count)) << 16);
}

static u64 internal_read_4_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| (((u64) ptr[3] >> (32-bit_count)) << 24);
}

static u64 internal_read_5_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| (((u64) ptr[4] >> (40-bit_count)) << 32);
}

static u64 internal_read_6_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| ((u64) ptr[4] << 32) 
		| (((u64) ptr[5] >> (48-bit_count)) << 40);
}

static u64 internal_read_7_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| ((u64) ptr[4] << 32) 
		| ((u64) ptr[5] << 40) 
		| (((u64) ptr[6] >> (56-bit_count)) << 48);
}

static u64 internal_read_8_le_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| ((u64) ptr[4] << 32) 
		| ((u64) ptr[5] << 40) 
		| ((u64) ptr[6] << 48) 
		| (((u64) ptr[7] >> (64-bit_count)) << 56);
}

static void internal_write_1_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits << (8 - bit_count);
}

static void internal_write_2_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8) << (16 - bit_count);
}

static void internal_write_3_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16) << (24 - bit_count);
}

static void internal_write_4_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24) << (32 - bit_count);
}

static void internal_write_5_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32) << (40 - bit_count);
}

static void internal_write_6_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32);
	ptr[5] = (u8) (bits >> 40) << (48 - bit_count);
}

static void internal_write_7_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32);
	ptr[5] = (u8) (bits >> 40);
	ptr[6] = (u8) (bits >> 48) << (56 - bit_count);
}

static void internal_write_8_le_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32);
	ptr[5] = (u8) (bits >> 40);
	ptr[6] = (u8) (bits >> 48);
	ptr[7] = (u8) (bits >> 56) << (64 - bit_count);
}

static u64 internal_read_1_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	return (ptr[0] & byte_lower_mask) >> (byte_lower_mask_size - bit_count);
}

static u64 internal_read_2_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (8 + byte_lower_mask_size - (u32) bit_count);
	
	return    (STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-8]) 
		| ((u64) ((ptr[1] & byte_lower_mask) >> new_lower_mask_size) << 8);	
}

static u64 internal_read_3_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (16 + byte_lower_mask_size - (u32) bit_count);

	return           STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	        | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-16]) << 8)
		| ((u64)((ptr[2] & byte_lower_mask) >> new_lower_mask_size) << 16);
}

static u64 internal_read_4_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (24 + byte_lower_mask_size - (u32) bit_count);

	return          STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-24]) << 16)
	       | ((u64)((ptr[3] & byte_lower_mask) >> new_lower_mask_size) << 24);
}

static u64 internal_read_5_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (32 + byte_lower_mask_size - (u32) bit_count);

	return          STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-32]) << 24)
	       | ((u64)((ptr[4] & byte_lower_mask) >> new_lower_mask_size) << 32);
}

static u64 internal_read_6_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (40 + byte_lower_mask_size - (u32) bit_count);

          return        STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size)) << 24)
	       | ((u64)(STRADDLE_BYTE(4, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-40]) << 32)
	       | ((u64)((ptr[5] & byte_lower_mask) >> new_lower_mask_size) << 40);
}

static u64 internal_read_7_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (48 + byte_lower_mask_size - (u32) bit_count);

          return        STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size)) << 24)
	       | ((u64)(STRADDLE_BYTE(4, byte_lower_mask, byte_lower_mask_size)) << 32)
	       | ((u64)(STRADDLE_BYTE(5, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-48]) << 40)
	       | ((u64)((ptr[6] & byte_lower_mask) >> new_lower_mask_size) << 48);
}

static u64 internal_read_8_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (56 + byte_lower_mask_size - (u32) bit_count);

          return        STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size)) << 24)
	       | ((u64)(STRADDLE_BYTE(4, byte_lower_mask, byte_lower_mask_size)) << 32)
	       | ((u64)(STRADDLE_BYTE(5, byte_lower_mask, byte_lower_mask_size)) << 40)
	       | ((u64)(STRADDLE_BYTE(6, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-56]) << 48)
	       | ((u64)((ptr[7] & byte_lower_mask) >> new_lower_mask_size) << 56);
}

static u64 internal_read_9_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u8 byte_upper_mask = ~byte_lower_mask;
	const u32 new_lower_mask_size = (64 + byte_lower_mask_size - (u32) bit_count);

          return        STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size)) << 24)
	       | ((u64)(STRADDLE_BYTE(4, byte_lower_mask, byte_lower_mask_size)) << 32)
	       | ((u64)(STRADDLE_BYTE(5, byte_lower_mask, byte_lower_mask_size)) << 40)
	       | ((u64)(STRADDLE_BYTE(6, byte_lower_mask, byte_lower_mask_size)) << 48)
	       | ((u64)(STRADDLE_BYTE(7, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-64]) << 56);
}

static void internal_write_1_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	ptr[0] |= (u8) bits << (byte_lower_mask_size - bit_count);
}

static void internal_write_2_le_straddling(u8 *ptr,  const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (8 + byte_lower_mask_size - (u32) bit_count);

	ptr[0] |=  (u8)  bits >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size];
	ptr[1]  = ((u8)  bits << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        | ((u8) (bits >> 8) << new_lower_mask_size);
}

static void internal_write_3_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (16 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[2]  =  ((u8) (bits >> 8) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 16) << new_lower_mask_size);
}

static void internal_write_4_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{ 
	const u32 new_lower_mask_size = (24 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[3]  =  ((u8) (bits >> 16) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 24) << new_lower_mask_size);
}

static void internal_write_5_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{
	const u32 new_lower_mask_size = (32 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> byte_upper_mask_size);
	ptr[3]  =  ((u8) (bits >> 16) << byte_lower_mask_size)
	        |  ((u8) (bits >> 24) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[4]  =  ((u8) (bits >> 24) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 32) << new_lower_mask_size);
}

static void internal_write_6_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{
	const u32 new_lower_mask_size = (40 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> byte_upper_mask_size);
	ptr[3]  =  ((u8) (bits >> 16) << byte_lower_mask_size)
	        |  ((u8) (bits >> 24) >> byte_upper_mask_size);
	ptr[4]  =  ((u8) (bits >> 24) << byte_lower_mask_size)
	        |  ((u8) (bits >> 32) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[5]  =  ((u8) (bits >> 32) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 40) << new_lower_mask_size);
}

static void internal_write_7_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{
	const u32 new_lower_mask_size = (48 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> byte_upper_mask_size);
	ptr[3]  =  ((u8) (bits >> 16) << byte_lower_mask_size)
	        |  ((u8) (bits >> 24) >> byte_upper_mask_size);
	ptr[4]  =  ((u8) (bits >> 24) << byte_lower_mask_size)
	        |  ((u8) (bits >> 32) >> byte_upper_mask_size);
	ptr[5]  =  ((u8) (bits >> 32) << byte_lower_mask_size)
	        |  ((u8) (bits >> 40) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[6]  =  ((u8) (bits >> 40) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 48) << new_lower_mask_size);
}

static void internal_write_8_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{
	const u32 new_lower_mask_size = (56 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> byte_upper_mask_size);
	ptr[3]  =  ((u8) (bits >> 16) << byte_lower_mask_size)
	        |  ((u8) (bits >> 24) >> byte_upper_mask_size);
	ptr[4]  =  ((u8) (bits >> 24) << byte_lower_mask_size)
	        |  ((u8) (bits >> 32) >> byte_upper_mask_size);
	ptr[5]  =  ((u8) (bits >> 32) << byte_lower_mask_size)
	        |  ((u8) (bits >> 40) >> byte_upper_mask_size);
	ptr[6]  =  ((u8) (bits >> 40) << byte_lower_mask_size)
	        |  ((u8) (bits >> 48) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[7]  =  ((u8) (bits >> 48) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size])
	        |  ((u8) (bits >> 56) << new_lower_mask_size);
}

static void internal_write_9_le_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count) 
{
	const u32 new_lower_mask_size = (64 + byte_lower_mask_size - (u32) bit_count);
	const u32 byte_upper_mask_size = 8 - byte_lower_mask_size;

	ptr[0] |=   (u8)  bits >> byte_upper_mask_size;
	ptr[1]  =  ((u8)  bits << byte_lower_mask_size)
	        |  ((u8) (bits >> 8) >> byte_upper_mask_size);
	ptr[2]  =  ((u8) (bits >> 8) << byte_lower_mask_size)
	        |  ((u8) (bits >> 16) >> byte_upper_mask_size);
	ptr[3]  =  ((u8) (bits >> 16) << byte_lower_mask_size)
	        |  ((u8) (bits >> 24) >> byte_upper_mask_size);
	ptr[4]  =  ((u8) (bits >> 24) << byte_lower_mask_size)
	        |  ((u8) (bits >> 32) >> byte_upper_mask_size);
	ptr[5]  =  ((u8) (bits >> 32) << byte_lower_mask_size)
	        |  ((u8) (bits >> 40) >> byte_upper_mask_size);
	ptr[6]  =  ((u8) (bits >> 40) << byte_lower_mask_size)
	        |  ((u8) (bits >> 48) >> byte_upper_mask_size);
	ptr[7]  =  ((u8) (bits >> 48) << byte_lower_mask_size)
	        |  ((u8) (bits >> 56) >> lower_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
	ptr[8]  =  ((u8) (bits >> 56) << upper_write_shift_lookup[byte_lower_mask_size][new_lower_mask_size]);
}

///* 
// * internal_read_le_partial_table: function table to read a little endian value from stream
// *
// * USAGE:  table[aligned/straddling][bytes_touched] 
// */
//static u64 (*internal_read_le_partial_table[2][10])(const u8 *, const u8, const u32, const i32) = 
//{
//	{
//		NULL,
//		&internal_read_1_straddling,
//		&internal_read_2_le_straddling,
//		&internal_read_3_le_straddling,
//		&internal_read_4_le_straddling,
//		&internal_read_5_le_straddling,
//		&internal_read_6_le_straddling,
//		&internal_read_7_le_straddling,
//		&internal_read_8_le_straddling,
//		&internal_read_9_le_straddling,
//	},
//
//	{
//		NULL,
//		&internal_read_1_aligned,
//		&internal_read_2_le_aligned,
//		&internal_read_3_le_aligned,
//		&internal_read_4_le_aligned,
//		&internal_read_5_le_aligned,
//		&internal_read_6_le_aligned,
//		&internal_read_7_le_aligned,
//		&internal_read_8_le_aligned,
//		NULL,
//	},
//};
//
///* 
// * internal_write_le_partial_table: function table to write a Logical value (register) as little endian to stream
// *
// * USAGE:  table[aligned/straddling][bytes_touched] 
// */
//static void (*internal_write_le_partial_table[2][10])( u8 *, const u32, const u64, const u64) = 
//{
//	{
//		NULL,
//		&internal_write_1_straddling,
//		&internal_write_2_le_straddling,
//		&internal_write_3_le_straddling,
//		&internal_write_4_le_straddling,
//		&internal_write_5_le_straddling,
//		&internal_write_6_le_straddling,
//		&internal_write_7_le_straddling,
//		&internal_write_8_le_straddling,
//		&internal_write_9_le_straddling,
//	},
//
//	{
//		NULL,
//		&internal_write_1_aligned,
//		&internal_write_2_le_aligned,
//		&internal_write_3_le_aligned,
//		&internal_write_4_le_aligned,
//		&internal_write_5_le_aligned,
//		&internal_write_6_le_aligned,
//		&internal_write_7_le_aligned,
//		&internal_write_8_le_aligned,
//		NULL,
//	},
//};

void ss_WriteU64LePartial(struct serialStream *ss, const u64 val, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u64 masked_val = val & (U64_MAX >> (64 - bit_count));
	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & masked_val) == 0, "Expect upper unused bits to be 0 in value to serialize");

	u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u32 byte_upper_mask_size = ss->bit_index & 0x7;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;

	if (byte_upper_mask_size)
	{
		const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { internal_write_1_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 2: { internal_write_2_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 3: { internal_write_3_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 4: { internal_write_4_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 5: { internal_write_5_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 6: { internal_write_6_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 7: { internal_write_7_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 8: { internal_write_8_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 9: { internal_write_9_le_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
		}
	}
	else
	{
		const u32 bytes_touched = (bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { internal_write_1_aligned(ptr, 0, masked_val, bit_count); } break;
			case 2: { internal_write_2_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 3: { internal_write_3_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 4: { internal_write_4_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 5: { internal_write_5_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 6: { internal_write_6_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 7: { internal_write_7_le_aligned(ptr, 0, masked_val, bit_count); } break;
			case 8: { internal_write_8_le_aligned(ptr, 0, masked_val, bit_count); } break;
		}
	}
	ss->bit_index += bit_count;
}

u64 ss_ReadU64LePartial(struct serialStream *ss, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u64 byte_upper_mask_size = ss->bit_index & 0x7;
	const u8 byte_lower_mask = 0xff >> byte_upper_mask_size;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;
	const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;

	u64 val;
	if (byte_upper_mask_size)
	{
		const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { val = internal_read_1_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 2: { val = internal_read_2_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 3: { val = internal_read_3_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 4: { val = internal_read_4_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 5: { val = internal_read_5_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 6: { val = internal_read_6_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 7: { val = internal_read_7_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 8: { val = internal_read_8_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 9: { val = internal_read_9_le_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
		}
	}
	else
	{
		const u32 bytes_touched = (bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { val = internal_read_1_aligned(ptr, 0, 0, bit_count); } break;
			case 2: { val = internal_read_2_le_aligned(ptr, 0, 0, bit_count); } break;
			case 3: { val = internal_read_3_le_aligned(ptr, 0, 0, bit_count); } break;
			case 4: { val = internal_read_4_le_aligned(ptr, 0, 0, bit_count); } break;
			case 5: { val = internal_read_5_le_aligned(ptr, 0, 0, bit_count); } break;
			case 6: { val = internal_read_6_le_aligned(ptr, 0, 0, bit_count); } break;
			case 7: { val = internal_read_7_le_aligned(ptr, 0, 0, bit_count); } break;
			case 8: { val = internal_read_8_le_aligned(ptr, 0, 0, bit_count); } break;
		}
	}

	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & val) == 0, "Expect upper unused bits to be 0 in value to serialize");
	ss->bit_index += bit_count;
	return val;
}

static u64 internal_read_2_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] >> (16-bit_count));
}

static u64 internal_read_3_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] >> (24-bit_count));
}

static u64 internal_read_4_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] >> (32-bit_count));
}

static u64 internal_read_5_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] >> (40-bit_count));
}

static u64 internal_read_6_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] << (bit_count-40))
		| ((u64) ptr[5] >> (48-bit_count));
}

static u64 internal_read_7_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] << (bit_count-40))
		| ((u64) ptr[5] << (bit_count-48))
		| ((u64) ptr[6] >> (56-bit_count));
}

static u64 internal_read_8_be_aligned(const u8 *ptr, const u8 garbage_1, const u32 garbage_2, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] << (bit_count-40))
		| ((u64) ptr[5] << (bit_count-48))
		| ((u64) ptr[6] << (bit_count-56))
		| ((u64) ptr[7] >> (64-bit_count));
}

static void internal_write_2_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits << (16 - bit_count));
}

static void internal_write_3_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits << (24 - bit_count));
}

static void internal_write_4_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits << (32 - bit_count));
}

static void internal_write_5_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits << (40 - bit_count));
}

static void internal_write_6_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits >> (bit_count - 40));
	ptr[5] = (u8) (bits << (48 - bit_count));
}

static void internal_write_7_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits >> (bit_count - 40));
	ptr[5] = (u8) (bits >> (bit_count - 48));
	ptr[6] = (u8) (bits << (56 - bit_count));
}

static void internal_write_8_be_aligned(u8 *ptr, const u32 garbage_1, const u64 bits, const u64 bit_count) 
{
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits >> (bit_count - 40));
	ptr[5] = (u8) (bits >> (bit_count - 48));
	ptr[6] = (u8) (bits >> (bit_count - 56));
	ptr[7] = (u8) (bits << (64 - bit_count));
}

static u64 internal_read_2_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (8 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] >> new_lower_mask_size;
}

static u64 internal_read_3_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (16 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] >> new_lower_mask_size;
}

static u64 internal_read_4_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (24 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] >> new_lower_mask_size;
}

static u64 internal_read_5_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (32 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] << (bit_count - byte_lower_mask_size - 24)
		| (u64) ptr[4] >> new_lower_mask_size;
}

static u64 internal_read_6_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (40 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] << (bit_count - byte_lower_mask_size - 24)
		| (u64) ptr[4] << (bit_count - byte_lower_mask_size - 32)
		| (u64) ptr[5] >> new_lower_mask_size;
}

static u64 internal_read_7_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (48 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] << (bit_count - byte_lower_mask_size - 24)
		| (u64) ptr[4] << (bit_count - byte_lower_mask_size - 32)
		| (u64) ptr[5] << (bit_count - byte_lower_mask_size - 40)
		| (u64) ptr[6] >> new_lower_mask_size;
}

static u64 internal_read_8_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (56 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] << (bit_count - byte_lower_mask_size - 24)
		| (u64) ptr[4] << (bit_count - byte_lower_mask_size - 32)
		| (u64) ptr[5] << (bit_count - byte_lower_mask_size - 40)
		| (u64) ptr[6] << (bit_count - byte_lower_mask_size - 48)
		| (u64) ptr[7] >> new_lower_mask_size;
}

static u64 internal_read_9_be_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const u64 bit_count)
{
	const u32 new_lower_mask_size = (64 + byte_lower_mask_size - (u32) bit_count);
	return	  (u64) (byte_lower_mask & ptr[0]) << (bit_count - byte_lower_mask_size)
		| (u64) ptr[1] << (bit_count - byte_lower_mask_size - 8)
		| (u64) ptr[2] << (bit_count - byte_lower_mask_size - 16)
		| (u64) ptr[3] << (bit_count - byte_lower_mask_size - 24)
		| (u64) ptr[4] << (bit_count - byte_lower_mask_size - 32)
		| (u64) ptr[5] << (bit_count - byte_lower_mask_size - 40)
		| (u64) ptr[6] << (bit_count - byte_lower_mask_size - 48)
		| (u64) ptr[7] << (bit_count - byte_lower_mask_size - 56)
		| (u64) ptr[8] >> new_lower_mask_size;
}

static void internal_write_2_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (8 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_3_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (16 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_4_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (24 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_5_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (32 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 24));
	ptr[4]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_6_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (40 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 24));
	ptr[4]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 32));
	ptr[5]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_7_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (48 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 24));
	ptr[4]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 32));
	ptr[5]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 40));
	ptr[6]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_8_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (56 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 24));
	ptr[4]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 32));
	ptr[5]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 40));
	ptr[6]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 48));
	ptr[7]  = (u8) (bits << (new_lower_mask_size));
}

static void internal_write_9_be_straddling(u8 *ptr, const u32 byte_lower_mask_size, const u64 bits, const u64 bit_count)
{
	const u32 new_lower_mask_size = (64 + byte_lower_mask_size - (u32) bit_count);
	ptr[0] |= (u8) (bits >> (bit_count - byte_lower_mask_size));
	ptr[1]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 8));
	ptr[2]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 16));
	ptr[3]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 24));
	ptr[4]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 32));
	ptr[5]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 40));
	ptr[6]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 48));
	ptr[7]  = (u8) (bits >> (bit_count - byte_lower_mask_size - 56));
	ptr[8]  = (u8) (bits << (new_lower_mask_size));
}

u64 ss_ReadU64BePartial(struct serialStream *ss, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u64 byte_upper_mask_size = ss->bit_index & 0x7;
	const u8 byte_lower_mask = 0xff >> byte_upper_mask_size;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;
	const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;

	//fprintf(stderr, "byte lower mask size: %u\n", byte_lower_mask_size % 8);
	//fprintf(stderr, " ");
	//for (u32 i = 0; i < byte_upper_mask_size; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//fprintf(stderr, "V\n");
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[0] >> 7) & 0x1, (ptr[0] >> 6) & 0x1, (ptr[0] >> 5) & 0x1, (ptr[0] >> 4) & 0x1, (ptr[0] >> 3) & 0x1, (ptr[0] >> 2) & 0x1, (ptr[0] >> 1) & 0x1, (ptr[0] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[1] >> 7) & 0x1, (ptr[1] >> 6) & 0x1, (ptr[1] >> 5) & 0x1, (ptr[1] >> 4) & 0x1, (ptr[1] >> 3) & 0x1, (ptr[1] >> 2) & 0x1, (ptr[1] >> 1) & 0x1, (ptr[1] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[2] >> 7) & 0x1, (ptr[2] >> 6) & 0x1, (ptr[2] >> 5) & 0x1, (ptr[2] >> 4) & 0x1, (ptr[2] >> 3) & 0x1, (ptr[2] >> 2) & 0x1, (ptr[2] >> 1) & 0x1, (ptr[2] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[3] >> 7) & 0x1, (ptr[3] >> 6) & 0x1, (ptr[3] >> 5) & 0x1, (ptr[3] >> 4) & 0x1, (ptr[3] >> 3) & 0x1, (ptr[3] >> 2) & 0x1, (ptr[3] >> 1) & 0x1, (ptr[3] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[4] >> 7) & 0x1, (ptr[4] >> 6) & 0x1, (ptr[4] >> 5) & 0x1, (ptr[4] >> 4) & 0x1, (ptr[4] >> 3) & 0x1, (ptr[4] >> 2) & 0x1, (ptr[4] >> 1) & 0x1, (ptr[4] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[5] >> 7) & 0x1, (ptr[5] >> 6) & 0x1, (ptr[5] >> 5) & 0x1, (ptr[5] >> 4) & 0x1, (ptr[5] >> 3) & 0x1, (ptr[5] >> 2) & 0x1, (ptr[5] >> 1) & 0x1, (ptr[5] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[6] >> 7) & 0x1, (ptr[6] >> 6) & 0x1, (ptr[6] >> 5) & 0x1, (ptr[6] >> 4) & 0x1, (ptr[6] >> 3) & 0x1, (ptr[6] >> 2) & 0x1, (ptr[6] >> 1) & 0x1, (ptr[6] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[7] >> 7) & 0x1, (ptr[7] >> 6) & 0x1, (ptr[7] >> 5) & 0x1, (ptr[7] >> 4) & 0x1, (ptr[7] >> 3) & 0x1, (ptr[7] >> 2) & 0x1, (ptr[7] >> 1) & 0x1, (ptr[7] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u]\n\n", (ptr[8] >> 7) & 0x1, (ptr[8] >> 6) & 0x1, (ptr[8] >> 5) & 0x1, (ptr[8] >> 4) & 0x1, (ptr[8] >> 3) & 0x1, (ptr[8] >> 2) & 0x1, (ptr[8] >> 1) & 0x1, (ptr[8] >> 0) & 0x1);


	u64 val;
	if (byte_upper_mask_size)
	{
		const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { val = internal_read_1_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 2: { val = internal_read_2_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 3: { val = internal_read_3_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 4: { val = internal_read_4_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 5: { val = internal_read_5_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 6: { val = internal_read_6_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 7: { val = internal_read_7_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 8: { val = internal_read_8_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
			case 9: { val = internal_read_9_be_straddling(ptr, byte_lower_mask, byte_lower_mask_size, bit_count); } break;
		}
	}
	else
	{
		const u32 bytes_touched = (bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { val = internal_read_1_aligned(ptr, 0, 0, bit_count); } break;
			case 2: { val = internal_read_2_be_aligned(ptr, 0, 0, bit_count); } break;
			case 3: { val = internal_read_3_be_aligned(ptr, 0, 0, bit_count); } break;
			case 4: { val = internal_read_4_be_aligned(ptr, 0, 0, bit_count); } break;
			case 5: { val = internal_read_5_be_aligned(ptr, 0, 0, bit_count); } break;
			case 6: { val = internal_read_6_be_aligned(ptr, 0, 0, bit_count); } break;
			case 7: { val = internal_read_7_be_aligned(ptr, 0, 0, bit_count); } break;
			case 8: { val = internal_read_8_be_aligned(ptr, 0, 0, bit_count); } break;
		}
	}

	//fprintf(stderr, " ");
	//for (u32 i = 0; i < byte_upper_mask_size; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//for (u32 i = 0; i < bit_count; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//for (u32 i = 1; i < bytes_touched; ++i)
	//{
	//	fprintf(stderr, "   ");
	//}
        //
	//fprintf(stderr, "V\n");
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[0] >> 7) & 0x1, (ptr[0] >> 6) & 0x1, (ptr[0] >> 5) & 0x1, (ptr[0] >> 4) & 0x1, (ptr[0] >> 3) & 0x1, (ptr[0] >> 2) & 0x1, (ptr[0] >> 1) & 0x1, (ptr[0] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[1] >> 7) & 0x1, (ptr[1] >> 6) & 0x1, (ptr[1] >> 5) & 0x1, (ptr[1] >> 4) & 0x1, (ptr[1] >> 3) & 0x1, (ptr[1] >> 2) & 0x1, (ptr[1] >> 1) & 0x1, (ptr[1] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[2] >> 7) & 0x1, (ptr[2] >> 6) & 0x1, (ptr[2] >> 5) & 0x1, (ptr[2] >> 4) & 0x1, (ptr[2] >> 3) & 0x1, (ptr[2] >> 2) & 0x1, (ptr[2] >> 1) & 0x1, (ptr[2] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[3] >> 7) & 0x1, (ptr[3] >> 6) & 0x1, (ptr[3] >> 5) & 0x1, (ptr[3] >> 4) & 0x1, (ptr[3] >> 3) & 0x1, (ptr[3] >> 2) & 0x1, (ptr[3] >> 1) & 0x1, (ptr[3] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[4] >> 7) & 0x1, (ptr[4] >> 6) & 0x1, (ptr[4] >> 5) & 0x1, (ptr[4] >> 4) & 0x1, (ptr[4] >> 3) & 0x1, (ptr[4] >> 2) & 0x1, (ptr[4] >> 1) & 0x1, (ptr[4] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[5] >> 7) & 0x1, (ptr[5] >> 6) & 0x1, (ptr[5] >> 5) & 0x1, (ptr[5] >> 4) & 0x1, (ptr[5] >> 3) & 0x1, (ptr[5] >> 2) & 0x1, (ptr[5] >> 1) & 0x1, (ptr[5] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[6] >> 7) & 0x1, (ptr[6] >> 6) & 0x1, (ptr[6] >> 5) & 0x1, (ptr[6] >> 4) & 0x1, (ptr[6] >> 3) & 0x1, (ptr[6] >> 2) & 0x1, (ptr[6] >> 1) & 0x1, (ptr[6] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[7] >> 7) & 0x1, (ptr[7] >> 6) & 0x1, (ptr[7] >> 5) & 0x1, (ptr[7] >> 4) & 0x1, (ptr[7] >> 3) & 0x1, (ptr[7] >> 2) & 0x1, (ptr[7] >> 1) & 0x1, (ptr[7] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u]\n\n", (ptr[8] >> 7) & 0x1, (ptr[8] >> 6) & 0x1, (ptr[8] >> 5) & 0x1, (ptr[8] >> 4) & 0x1, (ptr[8] >> 3) & 0x1, (ptr[8] >> 2) & 0x1, (ptr[8] >> 1) & 0x1, (ptr[8] >> 0) & 0x1);

	//fprintf(stderr, "reading bits[%lu]: ", bit_count);
	//for (u32 i = 0; i < bit_count; ++i)
	//{
	//	fprintf(stderr, "%lu", (val >> (bit_count-1-i)) & 0x1);
	//}
	//fprintf(stderr, "\n\n");


	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & val) == 0, "Expect upper unused bits to be 0 in value to serialize");
	ss->bit_index += bit_count;
	return val;
}

void ss_WriteU64BePartial(struct serialStream *ss, const u64 val, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u64 masked_val = val & (U64_MAX >> (64 - bit_count));
	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & masked_val) == 0, "Expect upper unused bits to be 0 in value to serialize");

	u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u32 byte_upper_mask_size = ss->bit_index & 0x7;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;

	//fprintf(stderr, "writing bits[%lu]: ", bit_count);
	//for (u32 i = 0; i < bit_count; ++i)
	//{
	//	fprintf(stderr, "%lu", (masked_val >> (bit_count-1-i)) & 0x1);
	//}
	//fprintf(stderr, "\n");

	//fprintf(stderr, "byte lower mask size: %u\n", byte_lower_mask_size % 8);
	//fprintf(stderr, " ");
	//for (u32 i = 0; i < byte_upper_mask_size; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//fprintf(stderr, "V\n");
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[0] >> 7) & 0x1, (ptr[0] >> 6) & 0x1, (ptr[0] >> 5) & 0x1, (ptr[0] >> 4) & 0x1, (ptr[0] >> 3) & 0x1, (ptr[0] >> 2) & 0x1, (ptr[0] >> 1) & 0x1, (ptr[0] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[1] >> 7) & 0x1, (ptr[1] >> 6) & 0x1, (ptr[1] >> 5) & 0x1, (ptr[1] >> 4) & 0x1, (ptr[1] >> 3) & 0x1, (ptr[1] >> 2) & 0x1, (ptr[1] >> 1) & 0x1, (ptr[1] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[2] >> 7) & 0x1, (ptr[2] >> 6) & 0x1, (ptr[2] >> 5) & 0x1, (ptr[2] >> 4) & 0x1, (ptr[2] >> 3) & 0x1, (ptr[2] >> 2) & 0x1, (ptr[2] >> 1) & 0x1, (ptr[2] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[3] >> 7) & 0x1, (ptr[3] >> 6) & 0x1, (ptr[3] >> 5) & 0x1, (ptr[3] >> 4) & 0x1, (ptr[3] >> 3) & 0x1, (ptr[3] >> 2) & 0x1, (ptr[3] >> 1) & 0x1, (ptr[3] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[4] >> 7) & 0x1, (ptr[4] >> 6) & 0x1, (ptr[4] >> 5) & 0x1, (ptr[4] >> 4) & 0x1, (ptr[4] >> 3) & 0x1, (ptr[4] >> 2) & 0x1, (ptr[4] >> 1) & 0x1, (ptr[4] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[5] >> 7) & 0x1, (ptr[5] >> 6) & 0x1, (ptr[5] >> 5) & 0x1, (ptr[5] >> 4) & 0x1, (ptr[5] >> 3) & 0x1, (ptr[5] >> 2) & 0x1, (ptr[5] >> 1) & 0x1, (ptr[5] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[6] >> 7) & 0x1, (ptr[6] >> 6) & 0x1, (ptr[6] >> 5) & 0x1, (ptr[6] >> 4) & 0x1, (ptr[6] >> 3) & 0x1, (ptr[6] >> 2) & 0x1, (ptr[6] >> 1) & 0x1, (ptr[6] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[7] >> 7) & 0x1, (ptr[7] >> 6) & 0x1, (ptr[7] >> 5) & 0x1, (ptr[7] >> 4) & 0x1, (ptr[7] >> 3) & 0x1, (ptr[7] >> 2) & 0x1, (ptr[7] >> 1) & 0x1, (ptr[7] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u]\n\n", (ptr[8] >> 7) & 0x1, (ptr[8] >> 6) & 0x1, (ptr[8] >> 5) & 0x1, (ptr[8] >> 4) & 0x1, (ptr[8] >> 3) & 0x1, (ptr[8] >> 2) & 0x1, (ptr[8] >> 1) & 0x1, (ptr[8] >> 0) & 0x1);
	

	if (byte_upper_mask_size)
	{
		const u32 bytes_touched = (byte_upper_mask_size + bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { internal_write_1_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 2: { internal_write_2_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 3: { internal_write_3_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 4: { internal_write_4_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 5: { internal_write_5_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 6: { internal_write_6_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 7: { internal_write_7_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 8: { internal_write_8_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
			case 9: { internal_write_9_be_straddling(ptr, byte_lower_mask_size, masked_val, bit_count); } break;
		}
	}
	else
	{
		const u32 bytes_touched = (bit_count + 7) >> 3;
		switch (bytes_touched)
		{
			case 1: { internal_write_1_aligned(ptr, 0, masked_val, bit_count); } break;
			case 2: { internal_write_2_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 3: { internal_write_3_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 4: { internal_write_4_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 5: { internal_write_5_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 6: { internal_write_6_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 7: { internal_write_7_be_aligned(ptr, 0, masked_val, bit_count); } break;
			case 8: { internal_write_8_be_aligned(ptr, 0, masked_val, bit_count); } break;
		}
	}
	ss->bit_index += bit_count;

	//const u32 bytes_touched = (byte_upper_mask_size)
	//	? (byte_upper_mask_size + bit_count + 7) >> 3
	//	: (bit_count + 7) >> 3;
	//fprintf(stderr, " ");
	//for (u32 i = 0; i < byte_upper_mask_size; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//for (u32 i = 0; i < bit_count; ++i)
	//{
	//	fprintf(stderr, " ");
	//}
	//for (u32 i = 1; i < bytes_touched; ++i)
	//{
	//	fprintf(stderr, "   ");
	//}
	//fprintf(stderr, "V\n");
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[0] >> 7) & 0x1, (ptr[0] >> 6) & 0x1, (ptr[0] >> 5) & 0x1, (ptr[0] >> 4) & 0x1, (ptr[0] >> 3) & 0x1, (ptr[0] >> 2) & 0x1, (ptr[0] >> 1) & 0x1, (ptr[0] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] " , (ptr[1] >> 7) & 0x1, (ptr[1] >> 6) & 0x1, (ptr[1] >> 5) & 0x1, (ptr[1] >> 4) & 0x1, (ptr[1] >> 3) & 0x1, (ptr[1] >> 2) & 0x1, (ptr[1] >> 1) & 0x1, (ptr[1] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[2] >> 7) & 0x1, (ptr[2] >> 6) & 0x1, (ptr[2] >> 5) & 0x1, (ptr[2] >> 4) & 0x1, (ptr[2] >> 3) & 0x1, (ptr[2] >> 2) & 0x1, (ptr[2] >> 1) & 0x1, (ptr[2] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[3] >> 7) & 0x1, (ptr[3] >> 6) & 0x1, (ptr[3] >> 5) & 0x1, (ptr[3] >> 4) & 0x1, (ptr[3] >> 3) & 0x1, (ptr[3] >> 2) & 0x1, (ptr[3] >> 1) & 0x1, (ptr[3] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[4] >> 7) & 0x1, (ptr[4] >> 6) & 0x1, (ptr[4] >> 5) & 0x1, (ptr[4] >> 4) & 0x1, (ptr[4] >> 3) & 0x1, (ptr[4] >> 2) & 0x1, (ptr[4] >> 1) & 0x1, (ptr[4] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[5] >> 7) & 0x1, (ptr[5] >> 6) & 0x1, (ptr[5] >> 5) & 0x1, (ptr[5] >> 4) & 0x1, (ptr[5] >> 3) & 0x1, (ptr[5] >> 2) & 0x1, (ptr[5] >> 1) & 0x1, (ptr[5] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[6] >> 7) & 0x1, (ptr[6] >> 6) & 0x1, (ptr[6] >> 5) & 0x1, (ptr[6] >> 4) & 0x1, (ptr[6] >> 3) & 0x1, (ptr[6] >> 2) & 0x1, (ptr[6] >> 1) & 0x1, (ptr[6] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u] ", (ptr[7] >> 7) & 0x1, (ptr[7] >> 6) & 0x1, (ptr[7] >> 5) & 0x1, (ptr[7] >> 4) & 0x1, (ptr[7] >> 3) & 0x1, (ptr[7] >> 2) & 0x1, (ptr[7] >> 1) & 0x1, (ptr[7] >> 0) & 0x1);
	//fprintf(stderr, "[%u%u%u%u%u%u%u%u]\n\n", (ptr[8] >> 7) & 0x1, (ptr[8] >> 6) & 0x1, (ptr[8] >> 5) & 0x1, (ptr[8] >> 4) & 0x1, (ptr[8] >> 3) & 0x1, (ptr[8] >> 2) & 0x1, (ptr[8] >> 1) & 0x1, (ptr[8] >> 0) & 0x1);
}

void ss_WriteI64LePartial(struct serialStream *ss, const i64 val, const u64 bit_count)
{
	b64 trunc = { .i = val };
	const u64 mask = (0x7fffffffffffffff) >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	ss_WriteU64LePartial(ss, trunc.u, bit_count);
}

void ss_WriteI64BePartial(struct serialStream *ss, const i64 val, const u64 bit_count)
{
	b64 trunc = { .i = val };
	const u64 mask = (0x7fffffffffffffff) >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	ss_WriteU64BePartial(ss, trunc.u, bit_count);
}

i64 ss_ReadI64LePartial(struct serialStream *ss, const u64 bit_count)
{
	b64 trunc = { .u = ss_ReadU64LePartial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count-1);
	trunc.u |= ((0xffffffffffffffff * sign) << (bit_count-1));
	return trunc.i;
}

i64 ss_ReadI64BePartial(struct serialStream *ss, const u64 bit_count)
{
	b64 trunc = { .u = ss_ReadU64BePartial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count-1);
	trunc.u |= ((0xffffffffffffffff * sign) << (bit_count-1));
	return trunc.i;
}


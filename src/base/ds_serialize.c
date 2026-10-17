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

#include <ds_base.h>

/*
   ====== SERIALIZE_STREAM PARTIAL BYTE READING ======

 	  When we want to read n < 8 bits from a byte, we read from the "Logical high bit downwards, i.e. 
 	  		
    	  					         HEAD
    	 						  V
 	     		      | b7 b6 b5 b4 b3 b2 b1 b0 | b7 b6 b5 b4 b3 b2 b1 b0 |  
 	     		      | 	Byte k  	|  	Byte k+1	  |
 	 
 	  read(4) = 0 0 0 0 b7 b6 b5 b4;
 	 	    =>  			                     HEAD
 	 	             			              	      V
 	 	       	      | b7 b6 b5 b4 b3 b2 b1 b0 | b7 b6 b5 b4 b3 b2 b1 b0 | 
 	 	              | 	Byte k  	|  	Byte k+1          |
 	 
 	 
   ====== SERIALIZE_STREAM WRITING INTERNALS ======
   	
   	(1) FULL ALIGNED BYTES: We read/write a full primitive (b8, b16, b32, b64) to a byte aligned address:
  
   		Implementation Example:
			Write le16:	
				const b16 stored = NaToLe16(val);
				memcpy(ss->buf + offset, &stored, sizeof(stored));
        		Read: b16 val;
				memcpy(&val, ss->buf + offset, sizeof(val));
				val = LeToNa16(val);

		memcpy, not a (b16 *) cast: the stream only guarantees byte alignment.
  
		In the write example, from our register's perspective what happens is:
		
					         B1			       B0	
			Register  [ b15 b14 b13 b12 b11 b10 b9 b8 | b7 b6 b5 b4 b3 b2 b1 b0 ]
		
			=>	Register(LE)  [ b15 b14 b13 b12 b11 b10 b9 b8 | b7 b6 b5 b4 b3 b2 b1 b0 ]
			=>	Register(BE)  [ b8 b7 b6 b5 b4 b3 b2 b1 b0 | b15 b14 b13 b12 b11 b10 b9 ]	(manual invert byte order)
		
		 *buf[0,1] = register;
			=>	buf[0|1] = [ b8 b7 b6 b5 b4 b3 b2 b1 b0 | b15 b14 b13 b12 b11 b10 b9 ]	(LE)
			=>	buf[0|1] = [ b8 b7 b6 b5 b4 b3 b2 b1 b0 | b15 b14 b13 b12 b11 b10 b9 ]  (BE)
		
			I.e. LE_write(register) => invert Logical byte order
			I.e. BE_write(register) =>        Logical byte order

		similarly, in the read example, from our register's perspective what happens is:
		
				buf[0|1] = [ b8 b7 b6 b5 b4 b3 b2 b1 b0 | b15 b14 b13 b12 b11 b10 b9 ]

		 register = *buf[0|1];
			=>	register = [ b15 b14 b13 b12 b11 b10 b9 b8 | b7 b6 b5 b4 b3 b2 b1 b0 ] (LE)
			=>	register = [ b8 b7 b6 b5 b4 b3 b2 b1 b0 | b15 b14 b13 b12 b11 b10 b9 ] (BE)

		manual flipping of the BE case give the correct register.

   	(2) FULL UNALIGNED BITS: Exactly the same as the FULL ALIGNED BYTES case, except each byte (order depending or le/be write/read)
	    		       is possibly straddling the serialize stream bytes.

		Example be16 full write:
			       stream:	[ b7  b6  b5  b4  b3  b2  b1  b0 | b7  b6  b5  b4  b3 b2 b1 b0 | b7 b6 b5 b4 b3 b2 b1 b0 ]
			       b16: 			[ b15 b14 b13 b12  b11 b10 b9 b8 | b7 b6 b5 b4   b3 b2 b1 b0 ]


   	(3) PARTIAL BITS: We write a a number of bits not divisible by 8.  Here is where ambiguity may appear; suppose we write 12 bits
	using write16_partial_le. 
       
				  B1			      B0
		Register [ x x x x b11 b10 b9 b8 | b7 b6 b5 b4 b3 b2 b1 b0 ]

		Should the partial byte be stored at the start or at the end of the stream? i.e. should the state of the stream be 

			
	     Write happened here			Head after write 
		      V					      V
		[ ... b3 b2 b1 b0 | b11 b10 b9 b8 b7 b6 b5 b4 x x x .... ]	(partial byte at the beginning)
		[ ... b7 b6 b5 b4 b3 b2 b1 b0 | b11 b10 b9 b8 x x x .... ]	(partial byte at the end)

	It seems more reasonable to store the partial byte at the end, so we shall go with that convetion.

   ====== SERIALIZE_STREAM SIGNED INTEGER WRITING ======
	
	Suppose we want to write n bits of a signed integer value. It is reasonable to store the sign bit as the most significant bit
	and the rest of the n-1 bits in the usual order. Suppose and we wish to write a signed 16bit integer variable. Furthermore, 
	suppose we know the 10 most significat bits of our variable contains the sign bit(-64 <= variable <= 63). Then we can store the
	integer as follows:

	Integer :  [ S S S S S S S S | S S b5 b4 b3 b2 b1 b0 ]
	Register : [ x x x x x x x x | x S b5 b4 b3 b2 b1 b0 ]

	We can now write it in partial le/be fashion, and when we read it back, we may expand the sign in the register to retrieve
	the signed integer.

	Register : [ S S S S S S S S | S S b5 b4 b3 b2 b1 b0 ]

 */

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
 
static u64 internal_read_1_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] >> (8-bit_count);
}

static u64 internal_read_2_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| (((u64) ptr[1] >> (16-bit_count)) << 8);
}

static u64 internal_read_3_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| (((u64) ptr[2] >> (24-bit_count)) << 16);
}

static u64 internal_read_4_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| (((u64) ptr[3] >> (32-bit_count)) << 24);
}

static u64 internal_read_5_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| (((u64) ptr[4] >> (40-bit_count)) << 32);
}

static u64 internal_read_6_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| ((u64) ptr[4] << 32) 
		| (((u64) ptr[5] >> (48-bit_count)) << 40);
}

static u64 internal_read_7_le_aligned(const u8 *ptr, const i32 bit_count)
{
	return (u64) ptr[0] 
		| ((u64) ptr[1] << 8) 
		| ((u64) ptr[2] << 16) 
		| ((u64) ptr[3] << 24) 
		| ((u64) ptr[4] << 32) 
		| ((u64) ptr[5] << 40) 
		| (((u64) ptr[6] >> (56-bit_count)) << 48);
}

static u64 internal_read_8_le_aligned(const u8 *ptr, const i32 bit_count)
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

static void internal_write_1_aligned(u8 *ptr, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits << (8 - bit_count);
}

static void internal_write_2_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8) << (16 - bit_count);
}

static void internal_write_3_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16) << (24 - bit_count);
}

static void internal_write_4_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24) << (32 - bit_count);
}

static void internal_write_5_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32) << (40 - bit_count);
}

static void internal_write_6_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32);
	ptr[5] = (u8) (bits >> 40) << (48 - bit_count);
}

static void internal_write_7_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) bits;
	ptr[1] = (u8) (bits >> 8);
	ptr[2] = (u8) (bits >> 16);
	ptr[3] = (u8) (bits >> 24);
	ptr[4] = (u8) (bits >> 32);
	ptr[5] = (u8) (bits >> 40);
	ptr[6] = (u8) (bits >> 48) << (56 - bit_count);
}

static void internal_write_8_le_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
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
	const u32 new_lower_mask_size = (8 + byte_lower_mask_size - (u32) bit_count);
	
	return    (STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-8]) 
		| ((u64) ((ptr[1] & byte_lower_mask) >> new_lower_mask_size) << 8);	
}

static u64 internal_read_3_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u32 new_lower_mask_size = (16 + byte_lower_mask_size - (u32) bit_count);

	return           STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	        | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-16]) << 8)
		| ((u64)((ptr[2] & byte_lower_mask) >> new_lower_mask_size) << 16);
}

static u64 internal_read_4_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u32 new_lower_mask_size = (24 + byte_lower_mask_size - (u32) bit_count);

	return          STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-24]) << 16)
	       | ((u64)((ptr[3] & byte_lower_mask) >> new_lower_mask_size) << 24);
}

static u64 internal_read_5_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
	const u32 new_lower_mask_size = (32 + byte_lower_mask_size - (u32) bit_count);

	return          STRADDLE_BYTE(0, byte_lower_mask, byte_lower_mask_size) 
	       | ((u64)(STRADDLE_BYTE(1, byte_lower_mask, byte_lower_mask_size)) << 8)
	       | ((u64)(STRADDLE_BYTE(2, byte_lower_mask, byte_lower_mask_size)) << 16)
	       | ((u64)(STRADDLE_BYTE(3, byte_lower_mask, byte_lower_mask_size) >> shift_lookup[bit_count-32]) << 24)
	       | ((u64)((ptr[4] & byte_lower_mask) >> new_lower_mask_size) << 32);
}

static u64 internal_read_6_le_straddling(const u8 *ptr, const u8 byte_lower_mask, const u32 byte_lower_mask_size, const i32 bit_count)
{
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

void ss_WriteU64LePartial(struct ss *ss, const u64 val, const u64 bit_count)
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
			case 1: { internal_write_1_aligned(ptr, masked_val, bit_count); } break;
			case 2: { internal_write_2_le_aligned(ptr, masked_val, bit_count); } break;
			case 3: { internal_write_3_le_aligned(ptr, masked_val, bit_count); } break;
			case 4: { internal_write_4_le_aligned(ptr, masked_val, bit_count); } break;
			case 5: { internal_write_5_le_aligned(ptr, masked_val, bit_count); } break;
			case 6: { internal_write_6_le_aligned(ptr, masked_val, bit_count); } break;
			case 7: { internal_write_7_le_aligned(ptr, masked_val, bit_count); } break;
			case 8: { internal_write_8_le_aligned(ptr, masked_val, bit_count); } break;
		}
	}
	ss->bit_index += bit_count;
}

u64 ss_ReadU64LePartial(struct ss *ss, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u64 byte_upper_mask_size = ss->bit_index & 0x7;
	const u8 byte_lower_mask = 0xff >> byte_upper_mask_size;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;

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
			case 1: { val = internal_read_1_aligned(ptr, bit_count); } break;
			case 2: { val = internal_read_2_le_aligned(ptr, bit_count); } break;
			case 3: { val = internal_read_3_le_aligned(ptr, bit_count); } break;
			case 4: { val = internal_read_4_le_aligned(ptr, bit_count); } break;
			case 5: { val = internal_read_5_le_aligned(ptr, bit_count); } break;
			case 6: { val = internal_read_6_le_aligned(ptr, bit_count); } break;
			case 7: { val = internal_read_7_le_aligned(ptr, bit_count); } break;
			case 8: { val = internal_read_8_le_aligned(ptr, bit_count); } break;
		}
	}

	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & val) == 0, "Expect upper unused bits to be 0 in value to serialize");
	ss->bit_index += bit_count;
	return val;
}

static u64 internal_read_2_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] >> (16-bit_count));
}

static u64 internal_read_3_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] >> (24-bit_count));
}

static u64 internal_read_4_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] >> (32-bit_count));
}

static u64 internal_read_5_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] >> (40-bit_count));
}

static u64 internal_read_6_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] << (bit_count-40))
		| ((u64) ptr[5] >> (48-bit_count));
}

static u64 internal_read_7_be_aligned(const u8 *ptr, const u64 bit_count)
{
	return ((u64) ptr[0] << (bit_count-8))
		| ((u64) ptr[1] << (bit_count-16))
		| ((u64) ptr[2] << (bit_count-24))
		| ((u64) ptr[3] << (bit_count-32))
		| ((u64) ptr[4] << (bit_count-40))
		| ((u64) ptr[5] << (bit_count-48))
		| ((u64) ptr[6] >> (56-bit_count));
}

static u64 internal_read_8_be_aligned(const u8 *ptr, const u64 bit_count)
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

static void internal_write_2_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits << (16 - bit_count));
}

static void internal_write_3_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count)
{
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits << (24 - bit_count));
}

static void internal_write_4_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits << (32 - bit_count));
}

static void internal_write_5_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits << (40 - bit_count));
}

static void internal_write_6_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits >> (bit_count - 40));
	ptr[5] = (u8) (bits << (48 - bit_count));
}

static void internal_write_7_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
{ 
	ptr[0] = (u8) (bits >> (bit_count - 8));
	ptr[1] = (u8) (bits >> (bit_count - 16));
	ptr[2] = (u8) (bits >> (bit_count - 24));
	ptr[3] = (u8) (bits >> (bit_count - 32));
	ptr[4] = (u8) (bits >> (bit_count - 40));
	ptr[5] = (u8) (bits >> (bit_count - 48));
	ptr[6] = (u8) (bits << (56 - bit_count));
}

static void internal_write_8_be_aligned(u8 *ptr, const u64 bits, const u64 bit_count) 
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

u64 ss_ReadU64BePartial(struct ss *ss, const u64 bit_count)
{
	ds_Assert((ss->bit_index + bit_count) <= ss->bit_count);
	ds_Assert(1 <= bit_count && bit_count <= 64);

	const u8 *ptr = ss->buf + (ss->bit_index >> 3);
	const u64 byte_upper_mask_size = ss->bit_index & 0x7;
	const u8 byte_lower_mask = 0xff >> byte_upper_mask_size;
	const u32 byte_lower_mask_size = 8 - byte_upper_mask_size;

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
			case 1: { val = internal_read_1_aligned(ptr, bit_count); } break;
			case 2: { val = internal_read_2_be_aligned(ptr, bit_count); } break;
			case 3: { val = internal_read_3_be_aligned(ptr, bit_count); } break;
			case 4: { val = internal_read_4_be_aligned(ptr, bit_count); } break;
			case 5: { val = internal_read_5_be_aligned(ptr, bit_count); } break;
			case 6: { val = internal_read_6_be_aligned(ptr, bit_count); } break;
			case 7: { val = internal_read_7_be_aligned(ptr, bit_count); } break;
			case 8: { val = internal_read_8_be_aligned(ptr, bit_count); } break;
		}
	}

	ds_AssertString( ((~(U64_MAX >> (64 - bit_count))) & val) == 0, "Expect upper unused bits to be 0 in value to serialize");
	ss->bit_index += bit_count;
	return val;
}

void ss_WriteU64BePartial(struct ss *ss, const u64 val, const u64 bit_count)
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
			case 1: { internal_write_1_aligned(ptr, masked_val, bit_count); } break;
			case 2: { internal_write_2_be_aligned(ptr, masked_val, bit_count); } break;
			case 3: { internal_write_3_be_aligned(ptr, masked_val, bit_count); } break;
			case 4: { internal_write_4_be_aligned(ptr, masked_val, bit_count); } break;
			case 5: { internal_write_5_be_aligned(ptr, masked_val, bit_count); } break;
			case 6: { internal_write_6_be_aligned(ptr, masked_val, bit_count); } break;
			case 7: { internal_write_7_be_aligned(ptr, masked_val, bit_count); } break;
			case 8: { internal_write_8_be_aligned(ptr, masked_val, bit_count); } break;
		}
	}
	ss->bit_index += bit_count;
}

void ss_WriteI64LePartial(struct ss *ss, const i64 val, const u64 bit_count)
{
	b64 trunc = { .i = val };
	const u64 mask = (0x7fffffffffffffff) >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	ss_WriteU64LePartial(ss, trunc.u, bit_count);
}

void ss_WriteI64BePartial(struct ss *ss, const i64 val, const u64 bit_count)
{
	b64 trunc = { .i = val };
	const u64 mask = (0x7fffffffffffffff) >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	ss_WriteU64BePartial(ss, trunc.u, bit_count);
}

i64 ss_ReadI64LePartial(struct ss *ss, const u64 bit_count)
{
	b64 trunc = { .u = ss_ReadU64LePartial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count-1);
	trunc.u |= ((0xffffffffffffffff * sign) << (bit_count-1));
	return trunc.i;
}

i64 ss_ReadI64BePartial(struct ss *ss, const u64 bit_count)
{
	b64 trunc = { .u = ss_ReadU64BePartial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count-1);
	trunc.u |= ((0xffffffffffffffff * sign) << (bit_count-1));
	return trunc.i;
}


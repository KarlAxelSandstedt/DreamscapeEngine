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

#ifndef __DS_SERIALIZE_H__
#define __DS_SERIALIZE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

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
				((b16 *) (ss->buf + offset)) = na_to_le_16(val);
        		Read: b16 val = 
				le_to_na_16(*((b16 *) (ss->buf + offset)));
  
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
struct serialStream
{
	struct memSlot 	mem_slot;
	u64 		bit_index;	
	u64 		bit_count;
	u8 *		buf;
};

/* Allocates a stream on the arena if mem != NULL, otherwise heap allocation. On failure, we return stream = { 0 } */
struct serialStream	ss_Alloc(struct arena *mem, const u64 bufsize);
/* Initiates a stream that aliases the buf */
struct serialStream 	ss_Buffered(void *buf, const u64 bufsize);
/* free stream resources (when using malloc) */
void 			ss_Free(struct serialStream *ss);
/* return number of whole bytes left */
u64			ss_BytesLeft(const struct serialStream *ss);
/* return number of bits left */
u64			ss_BitsLeft(const struct serialStream *ss);


/*
 * TODO: CHECK / TRUNCATE b8 val upper bits to zero that are not to be written in functions calling ss_Write****
 * 	This simplifies Logic in write methods
 */

/*	read / write aligned byte(s): 
 *		unaligned read/writes are unhandled ERRORS! 
 *		buffer overruns are unhandled ERRORS! 
 */
b8 	ss_Read8(struct serialStream *ss);
void 	ss_Write8(struct serialStream *ss, const b8 val);
b16 	ss_Read16Le(struct serialStream *ss);
void 	ss_Write16Le(struct serialStream *ss, const b16 val);
b16 	ss_Read16Be(struct serialStream *ss);
void 	ss_Write16Be(struct serialStream *ss, const b16 val);
b32 	ss_Read32Le(struct serialStream *ss);
void 	ss_Write32Le(struct serialStream *ss, const b32 val);
b32 	ss_Read32Be(struct serialStream *ss);
void 	ss_Write32Be(struct serialStream *ss, const b32 val);
b64 	ss_Read64Le(struct serialStream *ss);
void 	ss_Write64Le(struct serialStream *ss, const b64 val);
b64 	ss_Read64Be(struct serialStream *ss);
void 	ss_Write64Be(struct serialStream *ss, const b64 val);

#define ss_ReadU8(ss)      	ss_Read8(ss).u
#define ss_WriteU8(ss, val)     ss_Write8(ss, (b8) { .u = val })
#define ss_ReadU16Le(ss)        ss_Read16Le(ss).u
#define ss_WriteU16Le(ss, val)  ss_Write16Le(ss, (b16) { .u = val })
#define ss_ReadU16Be(ss)        ss_Read16Be(ss).u
#define ss_WriteU16Be(ss, val)  ss_Write16Be(ss, (b16) { .u = val })
#define ss_ReadU32Le(ss)        ss_Read32Le(ss).u
#define ss_WriteU32Le(ss, val)  ss_Write32Le(ss, (b32) { .u = val })
#define ss_ReadU32Be(ss)        ss_Read32Be(ss).u
#define ss_WriteU32Be(ss, val)  ss_Write32Be(ss, (b32) { .u = val })
#define ss_ReadU64Le(ss)        ss_Read64Le(ss).u
#define ss_WriteU64Le(ss, val)  ss_Write64Le(ss, (b64) { .u = val })
#define ss_ReadU64Be(ss)        ss_Read64Be(ss).u
#define ss_WriteU64Be(ss, val)  ss_Write64Be(ss, (b64) { .u = val })

#define ss_ReadI8(ss)      	ss_Read8(ss).i
#define ss_WriteI8(ss, val)     ss_Write8(ss, (b8) { .i = val })
#define ss_ReadI16Le(ss)        ss_Read16Le(ss).i
#define ss_WriteI16Le(ss, val)  ss_Write16Le(ss, (b16) { .i = val })
#define ss_ReadI16Be(ss)        ss_Read16Be(ss).i
#define ss_WriteI16Be(ss, val)  ss_Write16Be(ss, (b16) { .i = val })
#define ss_ReadI32Le(ss)        ss_Read32Le(ss).i
#define ss_WriteI32Le(ss, val)  ss_Write32Le(ss, (b32) { .i = val })
#define ss_ReadI32Be(ss)        ss_Read32Be(ss).i
#define ss_WriteI32Be(ss, val)  ss_Write32Be(ss, (b32) { .i = val })
#define ss_ReadI64Le(ss)        ss_Read64Le(ss).i
#define ss_WriteI64Le(ss, val)  ss_Write64Le(ss, (b64) { .i = val })
#define ss_ReadI64Be(ss)        ss_Read64Be(ss).i
#define ss_WriteI64Be(ss, val)  ss_Write64Be(ss, (b64) { .i = val })

#define ss_ReadF32Le(ss)   	ss_Read32Le(ss).f
#define ss_WriteF32Le(ss, val)  ss_Write32Le(ss, (b32) { .f = val })
#define ss_ReadF32Be(ss)        ss_Read32Be(ss).f
#define ss_WriteF32Be(ss, val)  ss_Write32Be(ss, (b32) { .f = val })
#define ss_ReadF64Le(ss)        ss_Read64Le(ss).f
#define ss_WriteF64Le(ss, val)  ss_Write64Le(ss, (b64) { .f = val })
#define ss_ReadF64Be(ss)        ss_Read64Be(ss).f
#define ss_WriteF64Be(ss, val)  ss_Write64Be(ss, (b64) { .f = val })

/*	read / write aligned byte(s): 
 *		unaligned read/writes are unhandled ERRORS! 
 *		buffer overruns are unhandled ERRORS! 
 */
void	ss_Read8N(b8 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write8N(struct serialStream *ss, const b8 *buf, const u64 len);
void 	ss_Read16LeN(b16 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write16LeN(struct serialStream *ss, const b16 *buf, const u64 len);
void 	ss_Read16BeN(b16 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write16BeN(struct serialStream *ss, const b16 *buf, const u64 len);
void 	ss_Read32LeN(b32 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write32LeN(struct serialStream *ss, const b32 *buf, const u64 len);
void 	ss_Read32BeN(b32 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write32BeN(struct serialStream *ss, const b32 *buf, const u64 len);
void 	ss_Read64LeN(b64 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write64LeN(struct serialStream *ss, const b64 *buf, const u64 len);
void	ss_Read64BeN(b64 *buf, struct serialStream *ss, const u64 len);
void 	ss_Write64BeN(struct serialStream *ss, const b64 *buf, const u64 len);

#define ss_ReadU8N(buf, ss, len)		ss_Read8N((b8 *) (buf), ss, len)
#define ss_WritUe8N(ss, buf, len)		ss_Write8N(ss, (b8 *) (buf), len)
#define ss_ReadU16LeN(buf, ss, len) 		ss_Read16LeN((b16 *) (buf), ss, len)
#define ss_WriteU16LeN(ss, buf, len) 		ss_Write16LeN((ss, b16 *) (buf), len)
#define ss_ReadU16BeN(buf, ss, len) 		ss_Read16BeN((b16 *) (buf), ss, len)
#define ss_WriteU16BeN(ss, buf, len) 		ss_Write16BeN(ss, (b16 *) (buf), len)
#define ss_ReadU32LeN(buf, ss, len) 		ss_Read32LeN((b32 *) (buf), ss, len)
#define ss_WriteU32LeN(ss, buf, len) 		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadU32BeN(buf, ss, len) 		ss_Read32BeN((b32 *) (buf), ss, len)
#define ss_WriteU32BeN(ss, buf, len) 		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadU64LeN(buf, ss, len) 		ss_Read64LeN((b64 *) (buf), ss, len)
#define ss_WriteU64LeN(ss, buf, len) 		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadU64BeN(buf, ss, len)		ss_Read64BeN((b64 *) (buf), ss, len)
#define ss_WriteU64BeN(ss, buf, len) 		ss_Write64BeN(ss, (b64 *) (buf), len)

#define ss_ReadI8Array(buf, ss, len)		ss_Read8N((b8 *) (buf), ss, len)
#define ss_WriteI8Array(ss, buf, len)		ss_Write8N(ss, (b8 *) (buf), len)
#define ss_ReadI16LeN(buf, ss, len) 		ss_Read16LeN((b16 *) (buf), ss, len)
#define ss_WriteI16LeN(ss, buf, len)		ss_Write16LeN(ss, (b16 *) (buf), len)
#define ss_ReadI16BeN(buf, ss, len) 		ss_Read16BeN((b16 *) (buf), ss, len)
#define ss_WriteI16BeN(ss, buf, len)		ss_Write16BeN(ss, (b16 *) (buf), len)
#define ss_ReadI32LeN(buf, ss, len) 		ss_Read32LeN((b32 *) (buf), ss, len)
#define ss_WriteI32LeN(ss, buf, len)		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadI32BeN(buf, ss, len) 		ss_Read32BeN((b32 *) (buf), ss, len)
#define ss_WriteI32BeN(ss, buf, len)		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadI64LeN(buf, ss, len) 		ss_Read64LeN((b64 *) (buf), ss, len)
#define ss_WriteI64LeN(ss, buf, len)		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadI64BeN(buf, ss, len)		ss_Read64BeN((b64 *) (buf), ss, len)
#define ss_WriteI64BeN(ss, buf, len)		ss_Write64BeN(ss, (b64 *) (buf), len)

#define ss_ReadF32LeN(buf, ss, len) 		ss_Read32LeN((b32 *) (buf), ss, len)
#define ss_WriteF32LeN(ss, buf, len)		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadF32BeN(buf, ss, len) 		ss_Read32BeN((b32 *) (buf), ss, len)
#define ss_WriteF32BeN(ss, buf, len)		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadF64LeN(buf, ss, len) 		ss_Read64LeN((b64 *) (buf), ss, len)
#define ss_WriteF64LeN(ss, buf, len)		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadF64BeN(buf, ss, len)		ss_Read64BeN((b64 *) (buf), ss, len)
#define ss_WriteF64BeN(ss, buf, len)		ss_Write64BeN(ss, (b64 *) (buf), len)

/*	read / write bit(s): 
 *		buffer overruns are unhandled ERRORS! 
 */
void 	ss_WriteU64LePartial(struct serialStream *ss, const u64 val, const u64 bit_count);
#define ss_WriteU32LePartial(ss, val, bit_count)	ss_WriteU64LePartial(ss, val, bit_count)
#define ss_WriteU16LePartial(ss, val, bit_count)	ss_WriteU64LePartial(ss, val, bit_count)
void 	ss_WriteU64BePartial(struct serialStream *ss, const u64 val, const u64 bit_count);
#define ss_WriteU32BePartial(ss, val, bit_count)	ss_WriteU64BePartial(ss, val, bit_count)
#define ss_WriteU16BePartial(ss, val, bit_count)	ss_WriteU64BePartial(ss, val, bit_count)
#define ss_WriteU8Partial(ss, val, bit_count)		ss_WriteU64BePartial(ss, val, bit_count)

u64 	ss_ReadU64LePartial(struct serialStream *ss, const u64 bit_count);
#define ss_ReadU32LePartial(ss, bit_count)		((u32) ss_ReadU64LePartial(ss, bit_count));
#define ss_ReadU16LePartial(ss, bit_count)		((u16) ss_ReadU64LePartial(ss, bit_count));
u64 	ss_ReadU64BePartial(struct serialStream *ss, const u64 bit_count);
#define ss_ReadU32BePartial(ss, bit_count)		((u32) ss_ReadU64BePartial(ss, bit_count));
#define ss_ReadU16BePartial(ss, bit_count)		((u16) ss_ReadU64BePartial(ss, bit_count));
#define ss_ReadU8Partial(ss, bit_count)			((u8) ss_ReadU64BePartial(ss, bit_count));

void 	ss_WriteI64LePartial(struct serialStream *ss, const i64 val, const u64 bit_count);
#define ss_WriteI32LePartial(ss, val, bit_count)	ss_WriteI64LePartial(ss, val, bit_count)
#define ss_WriteI16LePartial(ss, val, bit_count)	ss_WriteI64LePartial(ss, val, bit_count)
void 	ss_WriteI64BePartial(struct serialStream *ss, const i64 val, const u64 bit_count);
#define ss_WriteI32BePartial(ss, val, bit_count)	ss_WriteI64BePartial(ss, val, bit_count)
#define ss_WriteI16BePartial(ss, val, bit_count)	ss_WriteI64BePartial(ss, val, bit_count)
#define ss_WriteI8Partial(ss, val, bit_count)		ss_WriteI64BePartial(ss, val, bit_count)

i64 	ss_ReadI64LePartial(struct serialStream *ss, const u64 bit_count);
#define ss_ReadI32LePartial(ss, bit_count)		((i32) ss_ReadI64LePartial(ss, bit_count));
#define ss_ReadI16LePartial(ss, bit_count)		((i16) ss_ReadI64LePartial(ss, bit_count));
i64 	ss_ReadI64BePartial(struct serialStream *ss, const u64 bit_count);
#define ss_ReadI32BePartial(ss, bit_count)		((i32) ss_ReadI64BePartial(ss, bit_count));
#define ss_ReadI16BePartial(ss, bit_count)		((i16) ss_ReadI64BePartial(ss, bit_count));
#define ss_ReadI8Partial(ss, bit_count)			((i8) ss_ReadI64BePartial(ss, bit_count));

#ifdef __cplusplus
} 
#endif

#endif

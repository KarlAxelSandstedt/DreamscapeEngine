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
ds_serialize.h
==============
Serial streams (struct ss): a buffer read or written front to back, one value at a time, at a bit position
(bit_index) that each call advances. Used for snapshots, container and pool serialization, the font and sprite
assets and the determinism test files.

Two kinds of access:
    Bytes       ss_Read/Write{8,16,32,64}{Le,Be}[N]: whole values at a byte boundary (asserted), in the
                stream's byte order Le or Be, independent of the machine's. N forms move an array.
    Partial     ss_Read/Write{U,I}64{Le,Be}Partial: a value of bit_count (1-64) bits at any bit position.
Both kinds may be mixed in one stream; a byte access after partial ones needs bit_index % 8 == 0.

Bit order: each byte is filled from its most significant bit down. A partial Be value is written most
significant bit first. A partial Le value is cut into 8-bit groups from its least significant end; the groups
go lowest first, each most significant bit first, and the last group holds the remaining 1-8 top bits. At a
byte boundary with bit_count a multiple of 8, both equal the byte functions of the same byte order.

Contracts (all asserted in debug only; with asserts off, a violation reads or writes out of bounds):
the access fits in the stream (bit_index + size <= bit_count), byte access at a byte boundary, partial
bit_count in [1, 64]. Writes go front to back: a partial write ORs into the first byte when it starts inside
it and zeroes the rest of its last byte, so writing over data already in the stream corrupts it.

The U/I/F macros wrap the b8/b16/b32/b64 functions for u, i and f values; the narrow partial macros wrap the
64-bit functions. Internals (bit layout derivations): ds_serialize.c.

Sections: stream, byte access, byte array access, partial access; macro definitions at the bottom.
*/

#ifndef __DS_SERIALIZE_H__
#define __DS_SERIALIZE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*************************** stream ***************************/

struct ss
{
	u32                 heap_allocated;	/* buf is a ds_Alloc block (ss_Alloc without arena) */
	u64 		        bit_index;	/* next bit to read or write, counted from buf[0] bit 7 */
	u64 		        bit_count;	/* stream size in bits: 8 * bufsize */
	u8 *		        buf;
};

/*
 * Allocates a stream of bufsize bytes on the arena if mem != NULL, otherwise heap allocation (ds_Alloc,
 * cache line aligned). The buffer is uninitialized and bit_index = 0. On failure (also bufsize = 0 on an
 * arena), we return stream = { 0 }
 */
struct ss   ss_Alloc(struct arena *mem, const u64 bufsize);
/* Initiates a stream that aliases the buf of bufsize bytes (in bytes, not bits); bit_index = 0, buf not owned */
struct ss   ss_Buffered(void *buf, const u64 bufsize);
/* free stream resources (the ds_Alloc buffer of a heap stream); no-op for arena and buffered streams; buf dangles */
void 	    ss_Free(struct ss *ss);
/* return number of whole bytes left: (bit_count - bit_index) / 8, rounded down */
u64		    ss_BytesLeft(const struct ss *ss);
/* return number of bits left */
u64			ss_BitsLeft(const struct ss *ss);

/*************************** byte access ***************************/

/*
 * TODO: CHECK / TRUNCATE b8 val upper bits to zero that are not to be written in functions calling ss_Write****
 * 	This simplifies Logic in write methods
 */

/*
 * read aligned byte(s): return the next 8/16/32/64 bits as a value stored in the stream's byte order (Le/Be),
 * and advance bit_index by the size. bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
b8 	    ss_Read8(struct ss *ss);
b16 	ss_Read16Le(struct ss *ss);
b16 	ss_Read16Be(struct ss *ss);
b32 	ss_Read32Le(struct ss *ss);
b32 	ss_Read32Be(struct ss *ss);
b64 	ss_Read64Le(struct ss *ss);
b64 	ss_Read64Be(struct ss *ss);

/*
 * write aligned byte(s): store val in the stream's byte order (Le/Be) at bit_index and advance it by the size.
 * bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
void 	ss_Write8(struct ss *ss, const b8 val);
void 	ss_Write16Le(struct ss *ss, const b16 val);
void 	ss_Write16Be(struct ss *ss, const b16 val);
void 	ss_Write32Le(struct ss *ss, const b32 val);
void 	ss_Write32Be(struct ss *ss, const b32 val);
void 	ss_Write64Le(struct ss *ss, const b64 val);
void 	ss_Write64Be(struct ss *ss, const b64 val);

/* return the next value as u (U8: 8 bits, no byte order), i or f (F: 32/64 only) */
//#define ss_Read{U,I}8(ss)
//#define ss_Read{U,I,F}{16,32,64}{Le,Be}(ss)
/* write val as u (U8: 8 bits, no byte order), i or f (F: 32/64 only) */
//#define ss_Write{U,I}8(ss, val)
//#define ss_Write{U,I,F}{16,32,64}{Le,Be}(ss, val)

/*************************** byte array access ***************************/

/*
 * read aligned byte(s): read len values into buf[0..len-1] as ss_Read8/16/32/64{Le,Be} would one by one.
 * bit_index % 8 == 0 and all len values fit in the stream (both asserted).
 */
void	ss_Read8N(b8 *buf, struct ss *ss, const u64 len);
void 	ss_Read16LeN(b16 *buf, struct ss *ss, const u64 len);
void 	ss_Read16BeN(b16 *buf, struct ss *ss, const u64 len);
void 	ss_Read32LeN(b32 *buf, struct ss *ss, const u64 len);
void 	ss_Read32BeN(b32 *buf, struct ss *ss, const u64 len);
void 	ss_Read64LeN(b64 *buf, struct ss *ss, const u64 len);
void	ss_Read64BeN(b64 *buf, struct ss *ss, const u64 len);

/*
 * write aligned byte(s): write buf[0..len-1] as ss_Write8/16/32/64{Le,Be} would one by one.
 * bit_index % 8 == 0 and all len values fit in the stream (both asserted).
 */
void 	ss_Write8N(struct ss *ss, const b8 *buf, const u64 len);
void 	ss_Write16LeN(struct ss *ss, const b16 *buf, const u64 len);
void 	ss_Write16BeN(struct ss *ss, const b16 *buf, const u64 len);
void 	ss_Write32LeN(struct ss *ss, const b32 *buf, const u64 len);
void 	ss_Write32BeN(struct ss *ss, const b32 *buf, const u64 len);
void 	ss_Write64LeN(struct ss *ss, const b64 *buf, const u64 len);
void 	ss_Write64BeN(struct ss *ss, const b64 *buf, const u64 len);

/* read len u/i/f values into buf (cast to the bN array; I8: ss_ReadI8Array) */
//#define ss_Read{U,I}8N(buf, ss, len)
//#define ss_Read{U,I,F}{16,32,64}{Le,Be}N(buf, ss, len)
/* write len u/i/f values from buf (U8: ss_WritUe8N; I8: ss_WriteI8Array) */
//#define ss_Write{U,I}8N(ss, buf, len)
//#define ss_Write{U,I,F}{16,32,64}{Le,Be}N(ss, buf, len)

/*************************** partial access ***************************/

/*
 * read / write bit(s): bit_count in [1, 64] and bit_index + bit_count <= stream bit_count (both asserted);
 * any bit_index. Le/Be: the bit order in the summary. bit_index advances by bit_count.
 */

/* write the low bit_count bits of val; higher bits of val are dropped (not checked) */
void 	ss_WriteU64LePartial(struct ss *ss, const u64 val, const u64 bit_count);
/* write the low bit_count bits of val; higher bits of val are dropped (not checked) */
void 	ss_WriteU64BePartial(struct ss *ss, const u64 val, const u64 bit_count);
/* return the next bit_count bits as an unsigned value (bits above bit_count are 0) */
u64 	ss_ReadU64LePartial(struct ss *ss, const u64 bit_count);
/* return the next bit_count bits as an unsigned value (bits above bit_count are 0) */
u64 	ss_ReadU64BePartial(struct ss *ss, const u64 bit_count);

/*
 * write val as a bit_count-bit two's complement integer (bit bit_count-1 is the sign). val must lie in
 * [-2^(bit_count-1), 2^(bit_count-1) - 1] (not checked; outside, the sign is kept and the rest truncated)
 */
void 	ss_WriteI64LePartial(struct ss *ss, const i64 val, const u64 bit_count);
/* as ss_WriteI64LePartial, Be bit order */
void 	ss_WriteI64BePartial(struct ss *ss, const i64 val, const u64 bit_count);
/* return the next bit_count bits as a two's complement integer, sign extended to 64 bits */
i64 	ss_ReadI64LePartial(struct ss *ss, const u64 bit_count);
/* as ss_ReadI64LePartial, Be bit order */
i64 	ss_ReadI64BePartial(struct ss *ss, const u64 bit_count);

/* the 64-bit partial functions under narrower names; the reads cast the result. 8: Be (one byte has no order) */
//#define ss_Write{U,I}{16,32}{Le,Be}Partial(ss, val, bit_count)
//#define ss_Write{U,I}8Partial(ss, val, bit_count)
//#define ss_Read{U,I}{16,32}{Le,Be}Partial(ss, bit_count)
//#define ss_Read{U,I}8Partial(ss, bit_count)

/*************************** macro definitions ***************************/

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

#define ss_ReadU8N(buf, ss, len)		ss_Read8N((b8 *) (buf), ss, len)
#define ss_WritUe8N(ss, buf, len)		ss_Write8N(ss, (b8 *) (buf), len)
#define ss_ReadU16LeN(buf, ss, len) 		ss_Read16LeN((b16 *) (buf), ss, len)
#define ss_WriteU16LeN(ss, buf, len) 		ss_Write16LeN(ss, (b16 *) (buf), len)
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

#define ss_WriteU32LePartial(ss, val, bit_count)	ss_WriteU64LePartial(ss, val, bit_count)
#define ss_WriteU16LePartial(ss, val, bit_count)	ss_WriteU64LePartial(ss, val, bit_count)
#define ss_WriteU32BePartial(ss, val, bit_count)	ss_WriteU64BePartial(ss, val, bit_count)
#define ss_WriteU16BePartial(ss, val, bit_count)	ss_WriteU64BePartial(ss, val, bit_count)
#define ss_WriteU8Partial(ss, val, bit_count)		ss_WriteU64BePartial(ss, val, bit_count)
#define ss_ReadU32LePartial(ss, bit_count)		((u32) ss_ReadU64LePartial(ss, bit_count))
#define ss_ReadU16LePartial(ss, bit_count)		((u16) ss_ReadU64LePartial(ss, bit_count))
#define ss_ReadU32BePartial(ss, bit_count)		((u32) ss_ReadU64BePartial(ss, bit_count))
#define ss_ReadU16BePartial(ss, bit_count)		((u16) ss_ReadU64BePartial(ss, bit_count))
#define ss_ReadU8Partial(ss, bit_count)			((u8) ss_ReadU64BePartial(ss, bit_count))
#define ss_WriteI32LePartial(ss, val, bit_count)	ss_WriteI64LePartial(ss, val, bit_count)
#define ss_WriteI16LePartial(ss, val, bit_count)	ss_WriteI64LePartial(ss, val, bit_count)
#define ss_WriteI32BePartial(ss, val, bit_count)	ss_WriteI64BePartial(ss, val, bit_count)
#define ss_WriteI16BePartial(ss, val, bit_count)	ss_WriteI64BePartial(ss, val, bit_count)
#define ss_WriteI8Partial(ss, val, bit_count)		ss_WriteI64BePartial(ss, val, bit_count)
#define ss_ReadI32LePartial(ss, bit_count)		((i32) ss_ReadI64LePartial(ss, bit_count))
#define ss_ReadI16LePartial(ss, bit_count)		((i16) ss_ReadI64LePartial(ss, bit_count))
#define ss_ReadI32BePartial(ss, bit_count)		((i32) ss_ReadI64BePartial(ss, bit_count))
#define ss_ReadI16BePartial(ss, bit_count)		((i16) ss_ReadI64BePartial(ss, bit_count))
#define ss_ReadI8Partial(ss, bit_count)			((i8) ss_ReadI64BePartial(ss, bit_count))

#ifdef __cplusplus
} 
#endif

#endif

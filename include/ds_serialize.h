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

Sections: stream, byte access, byte array access, partial access; the inline definitions of the first three
and the macro definitions at the bottom. Only the partial functions live in ds_serialize.c.
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
static ds_ForceInline struct ss   ss_Alloc(struct arena *mem, const u64 bufsize);
/* Initiates a stream that aliases the buf of bufsize bytes (in bytes, not bits); bit_index = 0, buf not owned */
static ds_ForceInline struct ss   ss_Buffered(void *buf, const u64 bufsize);
/* free stream resources (the ds_Alloc buffer of a heap stream); no-op for arena and buffered streams; buf dangles */
static ds_ForceInline void 	    ss_Free(struct ss *ss);
/* return number of whole bytes left: (bit_count - bit_index) / 8, rounded down */
static ds_ForceInline u64		    ss_BytesLeft(const struct ss *ss);
/* return number of bits left */
static ds_ForceInline u64			ss_BitsLeft(const struct ss *ss);

/*************************** byte access ***************************/

/*
 * TODO: CHECK / TRUNCATE b8 val upper bits to zero that are not to be written in functions calling ss_Write****
 * 	This simplifies Logic in write methods
 */

/*
 * read aligned byte(s): return the next 8/16/32/64 bits as a value stored in the stream's byte order (Le/Be),
 * and advance bit_index by the size. bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
static ds_ForceInline b8 	    ss_Read8(struct ss *ss);
static ds_ForceInline b16 	ss_Read16Le(struct ss *ss);
static ds_ForceInline b16 	ss_Read16Be(struct ss *ss);
static ds_ForceInline b32 	ss_Read32Le(struct ss *ss);
static ds_ForceInline b32 	ss_Read32Be(struct ss *ss);
static ds_ForceInline b64 	ss_Read64Le(struct ss *ss);
static ds_ForceInline b64 	ss_Read64Be(struct ss *ss);

/*
 * write aligned byte(s): store val in the stream's byte order (Le/Be) at bit_index and advance it by the size.
 * bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
static ds_ForceInline void 	ss_Write8(struct ss *ss, const b8 val);
static ds_ForceInline void 	ss_Write16Le(struct ss *ss, const b16 val);
static ds_ForceInline void 	ss_Write16Be(struct ss *ss, const b16 val);
static ds_ForceInline void 	ss_Write32Le(struct ss *ss, const b32 val);
static ds_ForceInline void 	ss_Write32Be(struct ss *ss, const b32 val);
static ds_ForceInline void 	ss_Write64Le(struct ss *ss, const b64 val);
static ds_ForceInline void 	ss_Write64Be(struct ss *ss, const b64 val);

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
static ds_ForceInline void	ss_Read8N(struct ss *ss, b8 *buf, const u64 len);
static ds_ForceInline void 	ss_Read16LeN(struct ss *ss, b16 *buf, const u64 len);
static ds_ForceInline void 	ss_Read16BeN(struct ss *ss, b16 *buf, const u64 len);
static ds_ForceInline void 	ss_Read32LeN(struct ss *ss, b32 *buf, const u64 len);
static ds_ForceInline void 	ss_Read32BeN(struct ss *ss, b32 *buf, const u64 len);
static ds_ForceInline void 	ss_Read64LeN(struct ss *ss, b64 *buf, const u64 len);
static ds_ForceInline void	ss_Read64BeN(struct ss *ss, b64 *buf, const u64 len);

/*
 * write aligned byte(s): write buf[0..len-1] as ss_Write8/16/32/64{Le,Be} would one by one.
 * bit_index % 8 == 0 and all len values fit in the stream (both asserted).
 */
static ds_ForceInline void 	ss_Write8N(struct ss *ss, const b8 *buf, const u64 len);
static ds_ForceInline void 	ss_Write16LeN(struct ss *ss, const b16 *buf, const u64 len);
static ds_ForceInline void 	ss_Write16BeN(struct ss *ss, const b16 *buf, const u64 len);
static ds_ForceInline void 	ss_Write32LeN(struct ss *ss, const b32 *buf, const u64 len);
static ds_ForceInline void 	ss_Write32BeN(struct ss *ss, const b32 *buf, const u64 len);
static ds_ForceInline void 	ss_Write64LeN(struct ss *ss, const b64 *buf, const u64 len);
static ds_ForceInline void 	ss_Write64BeN(struct ss *ss, const b64 *buf, const u64 len);

/* read len u/i/f values into buf (cast to the bN array) */
//#define ss_Read{U,I}8N(ss, buf, len)
//#define ss_Read{U,I,F}{16,32,64}{Le,Be}N(ss, buf, len)
/* write len u/i/f values from buf */
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

/*************************** stream: inline definitions ***************************/

static ds_ForceInline struct ss ss_Alloc(struct arena *mem, const u64 bufsize)
{
	struct ss ss = { 0 };

	ss.buf = (mem)
		? ArenaPush(mem, bufsize)
		: ds_Alloc(bufsize, DS_CACHE_LINE);
	if (ss.buf)
	{
		ss.bit_count = bufsize * 8;
		ss.bit_index = 0;
		ss.heap_allocated = (mem == NULL);
	}

	return ss;
}

static ds_ForceInline void ss_Free(struct ss *ss)
{
	if (ss->heap_allocated)
	{
		ds_Free(ss->buf);
		ss->heap_allocated = 0;
	}
}

static ds_ForceInline struct ss ss_Buffered(void *buf, const u64 bufsize)
{
	struct ss ss =
	{
		.bit_count = bufsize * 8,
		.bit_index = 0,
		.buf = buf,
	};

	return ss;
}

static ds_ForceInline u64 ss_BytesLeft(const struct ss *ss)
{
	return (ss->bit_count - ss->bit_index) >> 3;
}

static ds_ForceInline u64 ss_BitsLeft(const struct ss *ss)
{
	return ss->bit_count - ss->bit_index;
}

/*************************** byte access: inline definitions ***************************/

/* byte order conversion: Na native, Le/Be stream order */
static ds_ForceInline b16 EndianShift16(const b16 be)
{
	const b16 shift = { .u =  (be.u << 8) | (be.u >> 8), };
	return shift; 
}

static ds_ForceInline b32 EndianShift32(const b32 le)
{
	const u32 mask1 = 0x00ff00ff;
	const u32 mask2 = 0xff00ff00;

	b32 shift = { .u = ((le.u & mask1) << 8) | ((le.u & mask2) >> 8) };
	shift.u = (shift.u << 16) | (shift.u >> 16);

	return shift; 
}

static ds_ForceInline b64 EndianShift64(const b64 le)
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

static ds_ForceInline b8 ss_Read8(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	b8 val = { .u = ss->buf[offset], };
	return val;
}

static ds_ForceInline void ss_Write8(struct ss *ss, const b8 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	*((b8 *) (ss->buf + offset)) = val;
}

static ds_ForceInline b16 ss_Read16Le(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	b16 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = LeToNa16(val);
	return val;
}

static ds_ForceInline void ss_Write16Le(struct ss *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	const b16 stored = NaToLe16(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b16 ss_Read16Be(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	b16 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = BeToNa16(val);
	return val;
}

static ds_ForceInline void ss_Write16Be(struct ss *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	const b16 stored = NaToBe16(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b32 ss_Read32Le(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	b32 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = LeToNa32(val);
	return val;
}

static ds_ForceInline void ss_Write32Le(struct ss *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	const b32 stored = NaToLe32(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b32 ss_Read32Be(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	b32 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = BeToNa32(val);
	return val;
}

static ds_ForceInline void ss_Write32Be(struct ss *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	const b32 stored = NaToBe32(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b64 ss_Read64Le(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	b64 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = LeToNa64(val);
	return val;
}

static ds_ForceInline void ss_Write64Le(struct ss *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	const b64 stored = NaToLe64(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b64 ss_Read64Be(struct ss *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	b64 val;
	memcpy(&val, ss->buf + offset, sizeof(val));
	val = BeToNa64(val);
	return val;
}

static ds_ForceInline void ss_Write64Be(struct ss *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	const b64 stored = NaToBe64(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

/*************************** byte array access: inline definitions ***************************/

static ds_ForceInline void ss_Read8N(struct ss *ss, b8 *buf, const u64 len)
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

static ds_ForceInline void ss_Write8N(struct ss *ss, const b8 *buf, const u64 len)
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

static ds_ForceInline void ss_Read16LeN(struct ss *ss, b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b16 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = LeToNa16(val);
	}
}

static ds_ForceInline void ss_Write16LeN(struct ss *ss, const b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b16 stored = NaToLe16(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

static ds_ForceInline void ss_Read16BeN(struct ss *ss, b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b16 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = BeToNa16(val);
	}
}

static ds_ForceInline void ss_Write16BeN(struct ss *ss, const b16 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b16 stored = NaToBe16(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

static ds_ForceInline void ss_Read32LeN(struct ss *ss, b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b32 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = LeToNa32(val);
	}
}

static ds_ForceInline void ss_Write32LeN(struct ss *ss, const b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b32 stored = NaToLe32(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

static ds_ForceInline void ss_Read32BeN(struct ss *ss, b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b32 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = BeToNa32(val);
	}
}

static ds_ForceInline void ss_Write32BeN(struct ss *ss, const b32 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b32 stored = NaToBe32(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

static ds_ForceInline void ss_Read64LeN(struct ss *ss, b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b64 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = LeToNa64(val);
	}
}

static ds_ForceInline void ss_Write64LeN(struct ss *ss, const b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b64 stored = NaToLe64(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

static ds_ForceInline void ss_Read64BeN(struct ss *ss, b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	const u8 *src = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		b64 val;
		memcpy(&val, src + i*sizeof(val), sizeof(val));
		buf[i] = BeToNa64(val);
	}
}

static ds_ForceInline void ss_Write64BeN(struct ss *ss, const b64 *buf, const u64 len)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64*len) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64*len;
	u8 *dst = ss->buf + offset;
	for (u64 i = 0; i < len; ++i)
	{
		const b64 stored = NaToBe64(buf[i]);
		memcpy(dst + i*sizeof(stored), &stored, sizeof(stored));
	}
}

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

#define ss_ReadU8N(ss, buf, len)		ss_Read8N(ss, (b8 *) (buf), len)
#define ss_WriteU8N(ss, buf, len)		ss_Write8N(ss, (b8 *) (buf), len)
#define ss_ReadU16LeN(ss, buf, len) 		ss_Read16LeN(ss, (b16 *) (buf), len)
#define ss_WriteU16LeN(ss, buf, len) 		ss_Write16LeN(ss, (b16 *) (buf), len)
#define ss_ReadU16BeN(ss, buf, len) 		ss_Read16BeN(ss, (b16 *) (buf), len)
#define ss_WriteU16BeN(ss, buf, len) 		ss_Write16BeN(ss, (b16 *) (buf), len)
#define ss_ReadU32LeN(ss, buf, len) 		ss_Read32LeN(ss, (b32 *) (buf), len)
#define ss_WriteU32LeN(ss, buf, len) 		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadU32BeN(ss, buf, len) 		ss_Read32BeN(ss, (b32 *) (buf), len)
#define ss_WriteU32BeN(ss, buf, len) 		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadU64LeN(ss, buf, len) 		ss_Read64LeN(ss, (b64 *) (buf), len)
#define ss_WriteU64LeN(ss, buf, len) 		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadU64BeN(ss, buf, len)		ss_Read64BeN(ss, (b64 *) (buf), len)
#define ss_WriteU64BeN(ss, buf, len) 		ss_Write64BeN(ss, (b64 *) (buf), len)

#define ss_ReadI8N(ss, buf, len)		ss_Read8N(ss, (b8 *) (buf), len)
#define ss_WriteI8N(ss, buf, len)		ss_Write8N(ss, (b8 *) (buf), len)
#define ss_ReadI16LeN(ss, buf, len) 		ss_Read16LeN(ss, (b16 *) (buf), len)
#define ss_WriteI16LeN(ss, buf, len)		ss_Write16LeN(ss, (b16 *) (buf), len)
#define ss_ReadI16BeN(ss, buf, len) 		ss_Read16BeN(ss, (b16 *) (buf), len)
#define ss_WriteI16BeN(ss, buf, len)		ss_Write16BeN(ss, (b16 *) (buf), len)
#define ss_ReadI32LeN(ss, buf, len) 		ss_Read32LeN(ss, (b32 *) (buf), len)
#define ss_WriteI32LeN(ss, buf, len)		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadI32BeN(ss, buf, len) 		ss_Read32BeN(ss, (b32 *) (buf), len)
#define ss_WriteI32BeN(ss, buf, len)		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadI64LeN(ss, buf, len) 		ss_Read64LeN(ss, (b64 *) (buf), len)
#define ss_WriteI64LeN(ss, buf, len)		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadI64BeN(ss, buf, len)		ss_Read64BeN(ss, (b64 *) (buf), len)
#define ss_WriteI64BeN(ss, buf, len)		ss_Write64BeN(ss, (b64 *) (buf), len)

#define ss_ReadF32LeN(ss, buf, len) 		ss_Read32LeN(ss, (b32 *) (buf), len)
#define ss_WriteF32LeN(ss, buf, len)		ss_Write32LeN(ss, (b32 *) (buf), len)
#define ss_ReadF32BeN(ss, buf, len) 		ss_Read32BeN(ss, (b32 *) (buf), len)
#define ss_WriteF32BeN(ss, buf, len)		ss_Write32BeN(ss, (b32 *) (buf), len)
#define ss_ReadF64LeN(ss, buf, len) 		ss_Read64LeN(ss, (b64 *) (buf), len)
#define ss_WriteF64LeN(ss, buf, len)		ss_Write64LeN(ss, (b64 *) (buf), len)
#define ss_ReadF64BeN(ss, buf, len)		ss_Read64BeN(ss, (b64 *) (buf), len)
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

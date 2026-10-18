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
(bit_index) that each call advances.

Two kinds of access:
    Bytes       ss_Read/Write{8,16,32,64}{Le,Be}[N]: whole values at a byte boundary (asserted), in the
                stream's byte order Le or Be, independent of the machine's. N forms move an array.
    Partial     ss_Read/Write{U,I}{8,16,32,64}Partial: a value of bit_count (1-64) bits at any bit position.
Both kinds may be mixed in one stream; a byte access after partial ones needs bit_index % 8 == 0.

Bit order: 
    stream bit i  is bit i % 8 
                 of byte i / 8, 

and a partial value's bits go in least significant first, so at a byte boundary a value of 8*k bits has the
bytes of the Le byte functions. .

Contracts (all asserted in debug only; with asserts off, a violation reads or writes out of bounds):
the access fits in the stream (bit_index + size <= bit_count), byte access at a byte boundary, partial
bit_count in [1, 64]. Writes go front to back: a partial write zeroes the rest of the 8 bytes from the byte
holding bit_index (within the buffer), and takes the bits before bit_index in that byte from ss->acc, which only
partial writes maintain, so a partial write after reads or byte writes starts at a byte boundary. Writing over
data already in the stream corrupts it.

The U/I/F macros wrap the b8/b16/b32/b64 functions for u, i and f values; the narrow partial macros wrap the
64-bit functions.

Sections: stream, byte access, byte array access, partial access; their inline definitions and the macro
definitions at the bottom. Everything is inline; there is no ds_serialize.c.
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
	u64		        acc;		/* partial writer: the byte at bit_index as it last stored it */
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
 * read / write bit(s) in the bit order of the summary: bit_count in [1, 64] and bit_index + bit_count <= stream
 * bit_count (both asserted); any bit_index. bit_index advances by bit_count.
 */

/* return the next bit_count bits as an unsigned value (bits above bit_count are 0) */
static ds_ForceInline u64	ss_ReadU64Partial(struct ss *ss, const u64 bit_count);
/*
 * write the low bit_count bits of val (higher bits are dropped) and zero the rest of the 8 bytes from the byte
 * holding bit_index, within the buffer. The bits below bit_index in its byte come from ss->acc, which only
 * partial writes maintain: a partial write after reads or byte writes on the stream must start at a byte
 * boundary (writes go front to back anyway)
 */
static ds_ForceInline void	ss_WriteU64Partial(struct ss *ss, const u64 val, const u64 bit_count);
/*
 * write val as a bit_count-bit two's complement integer (bit bit_count-1 is the sign). val must lie in
 * [-2^(bit_count-1), 2^(bit_count-1) - 1] (not checked; outside, the sign is kept and the rest truncated)
 */
static ds_ForceInline void	ss_WriteI64Partial(struct ss *ss, const i64 val, const u64 bit_count);
/* return the next bit_count bits as a two's complement integer, sign extended to 64 bits */
static ds_ForceInline i64	ss_ReadI64Partial(struct ss *ss, const u64 bit_count);

/* the 64-bit partial functions under narrower names; the reads cast the result */
//#define ss_Write{U,I}{8,16,32}Partial(ss, val, bit_count)
//#define ss_Read{U,I}{8,16,32}Partial(ss, bit_count)

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

/*************************** partial access: inline definitions ***************************/

/* the low bit_count bits: a load instead of a shift by a variable count and its setup */
static const u64 ss_partial_mask[65] =
{
	0x0000000000000000, 0x0000000000000001, 0x0000000000000003, 0x0000000000000007,
	0x000000000000000f, 0x000000000000001f, 0x000000000000003f, 0x000000000000007f,
	0x00000000000000ff, 0x00000000000001ff, 0x00000000000003ff, 0x00000000000007ff,
	0x0000000000000fff, 0x0000000000001fff, 0x0000000000003fff, 0x0000000000007fff,
	0x000000000000ffff, 0x000000000001ffff, 0x000000000003ffff, 0x000000000007ffff,
	0x00000000000fffff, 0x00000000001fffff, 0x00000000003fffff, 0x00000000007fffff,
	0x0000000000ffffff, 0x0000000001ffffff, 0x0000000003ffffff, 0x0000000007ffffff,
	0x000000000fffffff, 0x000000001fffffff, 0x000000003fffffff, 0x000000007fffffff,
	0x00000000ffffffff, 0x00000001ffffffff, 0x00000003ffffffff, 0x00000007ffffffff,
	0x0000000fffffffff, 0x0000001fffffffff, 0x0000003fffffffff, 0x0000007fffffffff,
	0x000000ffffffffff, 0x000001ffffffffff, 0x000003ffffffffff, 0x000007ffffffffff,
	0x00000fffffffffff, 0x00001fffffffffff, 0x00003fffffffffff, 0x00007fffffffffff,
	0x0000ffffffffffff, 0x0001ffffffffffff, 0x0003ffffffffffff, 0x0007ffffffffffff,
	0x000fffffffffffff, 0x001fffffffffffff, 0x003fffffffffffff, 0x007fffffffffffff,
	0x00ffffffffffffff, 0x01ffffffffffffff, 0x03ffffffffffffff, 0x07ffffffffffffff,
	0x0fffffffffffffff, 0x1fffffffffffffff, 0x3fffffffffffffff, 0x7fffffffffffffff,
	0xffffffffffffffff,
};

/*
 * The window is the 8 stream bytes from the byte holding bit_index; its low n = bit_index % 8 bits are read
 * already, so it holds 57-64 unread bits and the value is a shift and a mask. Only a value of 58-64 bits that
 * starts inside a byte reaches past it, into the next byte. The window is loaded whole where 8 bytes remain
 * and byte by byte (zeros above) at the end of the buffer; the value fits, so the zeros are never read.
 */
static ds_ForceInline u64 ss_ReadU64Partial(struct ss *ss, const u64 bit_count)
{
	ds_Assert(1 <= bit_count && bit_count <= 64);
	ds_Assert(ss->bit_index + bit_count <= ss->bit_count);

	const u64 p = ss->bit_index >> 3;
	const u64 n = ss->bit_index & 7;
	const u64 bytes_left = (ss->bit_count >> 3) - p;
	b64 window = { .u = 0 };
	if (bytes_left >= 8)
	{
		memcpy(&window, ss->buf + p, 8);
	}
	else
	{
		memcpy(&window, ss->buf + p, bytes_left);
	}
	window = LeToNa64(window);

	const u64 mask = ss_partial_mask[bit_count];
	u64 val = (window.u >> n) & mask;
	if (n + bit_count > 64)
	{
		val |= (((u64) ss->buf[p + 8]) << (64 - n)) & mask;
	}
	ss->bit_index += bit_count;
	return val;
}

/*
 * The window is the 8 stream bytes from the byte holding bit_index: the n = bit_index % 8 bits written before
 * (from acc) and the value above them. It is stored whole where 8 bytes remain, else only the bytes the value
 * touches (the value fits). What is left of the window above the completed bytes is the next acc. A value of
 * 58-64 bits that starts inside a byte overflows the window: the overflow is the next byte and the next acc.
 */
static ds_ForceInline void ss_WriteU64Partial(struct ss *ss, const u64 val, const u64 bit_count)
{
	ds_Assert(1 <= bit_count && bit_count <= 64);
	ds_Assert(ss->bit_index + bit_count <= ss->bit_count);

	const u64 p = ss->bit_index >> 3;
	const u64 n = ss->bit_index & 7;
	const u64 bytes_left = (ss->bit_count >> 3) - p;
	const u64 masked = val & ss_partial_mask[bit_count];
	const u64 window = (ss->acc & (((u64) 1 << n) - 1)) | (masked << n);
	const u64 total = n + bit_count;

	const b64 stored = NaToLe64((b64) { .u = window });
	if (bytes_left >= 8)
	{
		memcpy(ss->buf + p, &stored, 8);
	}
	else
	{
		memcpy(ss->buf + p, &stored, (total + 7) >> 3);
	}

	ss->bit_index += bit_count;
	if (total <= 64)
	{
		const u64 done = total >> 3;
		ss->acc = (done < 8) ? window >> (8 * done) : 0;
	}
	else
	{
		const u64 overflow = masked >> (64 - n);
		ss->buf[p + 8] = (u8) overflow;
		ss->acc = overflow;
	}
}

static ds_ForceInline void ss_WriteI64Partial(struct ss *ss, const i64 val, const u64 bit_count)
{
	b64 trunc = { .i = val };
	const u64 mask = 0x7fffffffffffffff >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	ss_WriteU64Partial(ss, trunc.u, bit_count);
}

static ds_ForceInline i64 ss_ReadI64Partial(struct ss *ss, const u64 bit_count)
{
	b64 trunc = { .u = ss_ReadU64Partial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count - 1);
	trunc.u |= (0xffffffffffffffff * sign) << (bit_count - 1);
	return trunc.i;
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

#define ss_WriteU8Partial(ss, val, bit_count)		ss_WriteU64Partial(ss, val, bit_count)
#define ss_WriteU16Partial(ss, val, bit_count)		ss_WriteU64Partial(ss, val, bit_count)
#define ss_WriteU32Partial(ss, val, bit_count)		ss_WriteU64Partial(ss, val, bit_count)
#define ss_ReadU8Partial(ss, bit_count)			((u8) ss_ReadU64Partial(ss, bit_count))
#define ss_ReadU16Partial(ss, bit_count)		((u16) ss_ReadU64Partial(ss, bit_count))
#define ss_ReadU32Partial(ss, bit_count)		((u32) ss_ReadU64Partial(ss, bit_count))
#define ss_WriteI8Partial(ss, val, bit_count)		ss_WriteI64Partial(ss, val, bit_count)
#define ss_WriteI16Partial(ss, val, bit_count)		ss_WriteI64Partial(ss, val, bit_count)
#define ss_WriteI32Partial(ss, val, bit_count)		ss_WriteI64Partial(ss, val, bit_count)
#define ss_ReadI8Partial(ss, bit_count)			((i8) ss_ReadI64Partial(ss, bit_count))
#define ss_ReadI16Partial(ss, bit_count)		((i16) ss_ReadI64Partial(ss, bit_count))
#define ss_ReadI32Partial(ss, bit_count)		((i32) ss_ReadI64Partial(ss, bit_count))

#ifdef __cplusplus
} 
#endif

#endif

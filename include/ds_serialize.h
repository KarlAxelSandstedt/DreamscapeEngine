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
Serial streams (struct SS): a buffer read or written front to back, one value at a time, at a bit position
(bit_index) that each call advances.

Two kinds of access:
    Bytes       SSRead/Write{8,16,32,64}{Le,Be}[N]: whole values at a byte boundary (asserted), in the
                stream's byte order Le or Be, independent of the machine's. N forms move an array.
    Partial     SSRead/Write{U,I}{8,16,32,64}Partial: a value of bit_count bits at any bit position: 0-64 unsigned
                (0: nothing read or written), 1-64 signed.
Both kinds may be mixed in one stream; a byte access after partial ones needs bit_index % 8 == 0.
Type serializers follow suit: XSerialize (aligned) nests only in aligned ones, XSerializeBitPacked nests anywhere.

Bit order: 
    stream bit i  is bit i % 8 
                 of byte i / 8, 

and a partial value's bits go in least significant first, so at a byte boundary a value of 8*k bits has the
bytes of the Le byte functions. A signed partial value of n bits is its n-bit two's complement: the low n - 1
bits of the value, then the sign as the last bit (stream bit bit_index + n - 1); reading sign-extends it to 64
bits.

Contracts (all asserted in debug only; with asserts off, a violation reads or writes out of bounds):
the access fits in the stream (bit_index + size <= bit_count), byte access at a byte boundary, partial
bit_count in [1, 64]. Writes go front to back: a partial write zeroes the rest of the 8 bytes from the byte
holding bit_index (within the buffer), and takes the bits before bit_index in that byte from ss->acc, which only
partial writes maintain, so a partial write after reads or byte writes starts at a byte boundary. Writing over
data already in the stream corrupts it.

The U/I/F macros wrap the b8/b16/b32/b64 functions for u, i and f values; the narrow partial macros wrap the
64-bit functions.

Sections: stream, byte access, byte array access, partial access, array; their inline definitions and the
macro definitions at the bottom. Everything is inline; there is no ds_serialize.c.
*/

#ifndef __DS_SERIALIZE_H__
#define __DS_SERIALIZE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*************************** stream ***************************/

/* A constant view of bit_count bits of a stream's buf from bit_offset (not owned); SSSubstreamFromView streams it. */
struct SSView
{
	u8 *		buf;
	u64 		bit_offset;
	u64 		bit_count;
};

struct SS
{
	u32                 heap_allocated;	/* buf is a ds_Alloc block (SSAlloc without arena) */
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
static ds_ForceInline struct SS   SSAlloc(struct arena *mem, const u64 bufsize);
/* Initiates a stream that aliases the buf of bufsize bytes (in bytes, not bits); bit_index = 0, buf not owned */
static ds_ForceInline struct SS   SSBuffered(void *buf, const u64 bufsize);
/* free stream resources (the ds_Alloc buffer of a heap stream); no-op for arena and buffered streams; buf dangles */
static ds_ForceInline void 	    SSDealloc(struct SS *ss);
/* Flush stream */
static ds_ForceInline void 	    SSFlush(struct SS *ss);
/* Grow a heap stream's buffer to at least bufsize bytes (to max(twice its size, bufsize); ds_Realloc, fatal on failure). NO-OP if it already holds bufsize. */
static ds_ForceInline void 	    SSIncreaseSize(struct SS *ss, const u64 bufsize);
/* return number of whole bytes left: (bit_count - bit_index) / 8, rounded down */
static ds_ForceInline u64		    SSBytesLeft(const struct SS *ss);
/* return number of bits left */
static ds_ForceInline u64			SSBitsLeft(const struct SS *ss);
/* Initiates view over bit_count bits of ss's buf from bit_offset. Returns 0 if the range leaves ss (view = { 0 }). */
static ds_ForceInline u32         SSTryView(struct SSView *view, const struct SS *ss, const u64 bit_offset, const u64 bit_count);
/* Return the viewed stream: same buf, bit_index at bit_offset, bit_count at the view's end (SSBitsLeft == bit_count). */
static ds_ForceInline struct SS   SSSubstreamFromView(const struct SSView *view);


/*************************** byte access ***************************/

/*
 * read aligned byte(s): return the next 8/16/32/64 bits as a value stored in the stream's byte order (Le/Be),
 * and advance bit_index by the size. bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
static ds_ForceInline b8 	    SSRead8(struct SS *ss);
static ds_ForceInline b16 	SSRead16Le(struct SS *ss);
static ds_ForceInline b16 	SSRead16Be(struct SS *ss);
static ds_ForceInline b32 	SSRead32Le(struct SS *ss);
static ds_ForceInline b32 	SSRead32Be(struct SS *ss);
static ds_ForceInline b64 	SSRead64Le(struct SS *ss);
static ds_ForceInline b64 	SSRead64Be(struct SS *ss);

/*
 * write aligned byte(s): store val in the stream's byte order (Le/Be) at bit_index and advance it by the size.
 * bit_index % 8 == 0 and the value fits in the stream (both asserted).
 */
static ds_ForceInline void 	SSWrite8(struct SS *ss, const b8 val);
static ds_ForceInline void 	SSWrite16Le(struct SS *ss, const b16 val);
static ds_ForceInline void 	SSWrite16Be(struct SS *ss, const b16 val);
static ds_ForceInline void 	SSWrite32Le(struct SS *ss, const b32 val);
static ds_ForceInline void 	SSWrite32Be(struct SS *ss, const b32 val);
static ds_ForceInline void 	SSWrite64Le(struct SS *ss, const b64 val);
static ds_ForceInline void 	SSWrite64Be(struct SS *ss, const b64 val);

/* return the next value as u (U8: 8 bits, no byte order), i or f (F: 32/64 only) */
//#define SSRead{U,I}8(ss)
//#define SSRead{U,I,F}{16,32,64}{Le,Be}(ss)
/* write val as u (U8: 8 bits, no byte order), i or f (F: 32/64 only) */
//#define SSWrite{U,I}8(ss, val)
//#define SSWrite{U,I,F}{16,32,64}{Le,Be}(ss, val)

/*************************** byte array access ***************************/

/*
 * read aligned byte(s): read len values into buf[0..len-1] as SSRead8/16/32/64{Le,Be} would one by one.
 * bit_index % 8 == 0 and all len values fit in the stream (both asserted).
 */
static ds_ForceInline void	SSRead8N(struct SS *ss, b8 *buf, const u64 len);
static ds_ForceInline void 	SSRead16LeN(struct SS *ss, b16 *buf, const u64 len);
static ds_ForceInline void 	SSRead16BeN(struct SS *ss, b16 *buf, const u64 len);
static ds_ForceInline void 	SSRead32LeN(struct SS *ss, b32 *buf, const u64 len);
static ds_ForceInline void 	SSRead32BeN(struct SS *ss, b32 *buf, const u64 len);
static ds_ForceInline void 	SSRead64LeN(struct SS *ss, b64 *buf, const u64 len);
static ds_ForceInline void	SSRead64BeN(struct SS *ss, b64 *buf, const u64 len);

/*
 * write aligned byte(s): write buf[0..len-1] as SSWrite8/16/32/64{Le,Be} would one by one.
 * bit_index % 8 == 0 and all len values fit in the stream (both asserted).
 */
static ds_ForceInline void 	SSWrite8N(struct SS *ss, const b8 *buf, const u64 len);
static ds_ForceInline void 	SSWrite16LeN(struct SS *ss, const b16 *buf, const u64 len);
static ds_ForceInline void 	SSWrite16BeN(struct SS *ss, const b16 *buf, const u64 len);
static ds_ForceInline void 	SSWrite32LeN(struct SS *ss, const b32 *buf, const u64 len);
static ds_ForceInline void 	SSWrite32BeN(struct SS *ss, const b32 *buf, const u64 len);
static ds_ForceInline void 	SSWrite64LeN(struct SS *ss, const b64 *buf, const u64 len);
static ds_ForceInline void 	SSWrite64BeN(struct SS *ss, const b64 *buf, const u64 len);

/* read len u/i/f values into buf (cast to the bN array) */
//#define SSRead{U,I}8N(ss, buf, len)
//#define SSRead{U,I,F}{16,32,64}{Le,Be}N(ss, buf, len)
/* write len u/i/f values from buf */
//#define SSWrite{U,I}8N(ss, buf, len)
//#define SSWrite{U,I,F}{16,32,64}{Le,Be}N(ss, buf, len)

/*************************** partial access ***************************/

/*
 * read / write bit(s) in the bit order of the summary: bit_count in [1, 64] and bit_index + bit_count <= stream
 * bit_count (both asserted); any bit_index. bit_index advances by bit_count.
 */

/* return the next bit_count bits as an unsigned value (bits above bit_count are 0); 0 bits: 0, nothing read */
static ds_ForceInline u64	SSReadU64Partial(struct SS *ss, const u64 bit_count);
/*
 * write the low bit_count bits of val (higher bits are dropped) and zero the rest of the 8 bytes from the byte
 * holding bit_index, within the buffer. The bits below bit_index in its byte come from ss->acc, which only
 * partial writes maintain: a partial write after reads or byte writes on the stream must start at a byte
 * boundary (writes go front to back anyway). 0 bits: nothing written.
 */
static ds_ForceInline void	SSWriteU64Partial(struct SS *ss, const u64 val, const u64 bit_count);
/*
 * write val as a bit_count-bit two's complement integer (bit bit_count-1 is the sign). val must lie in
 * [-2^(bit_count-1), 2^(bit_count-1) - 1] (not checked; outside, the sign is kept and the rest truncated)
 */
static ds_ForceInline void	SSWriteI64Partial(struct SS *ss, const i64 val, const u64 bit_count);
/* return the next bit_count bits as a two's complement integer, sign extended to 64 bits */
static ds_ForceInline i64	SSReadI64Partial(struct SS *ss, const u64 bit_count);

/* the 64-bit partial functions under narrower names; the reads cast the result */
//#define SSWrite{U,I}{8,16,32}Partial(ss, val, bit_count)
//#define SSRead{U,I}{8,16,32}Partial(ss, bit_count)

/*************************** array ***************************/

/* WARNING: Platform-dependent. */
/* Bytes ds_ArrayCompactSerialize writes. */
#define ds_ArrayCompactSerializeSize(_array_, _count_)  (sizeof(u32) + (u64) (_count_)*sizeof((_array_).buf[0]))
/* Write the first count elements of the array to ss (at a byte boundary; it must fit). */
#define ds_ArrayCompactSerialize(_ss_, _array_, _count_)                                        \
	ds_ArraySerializeInternal((_ss_), (_array_).buf, (_count_), sizeof((_array_).buf[0]))
/* Bytes ds_ArraySerialize writes. */
#define ds_ArraySerializeSize(_array_)      ds_ArrayCompactSerializeSize((_array_), (_array_).length)
/* Write the array to ss (at a byte boundary; it must fit: ds_ArraySerializeSize). */
#define ds_ArraySerialize(_ss_, _array_)    ds_ArrayCompactSerialize((_ss_), (_array_), (_array_).length)
/*
 * Read an array from ss into mem or the heap (mem == NULL; free it with ds_Free), with the written length
 * and base U32_MAX. Returns 1 on success; 0 if mem is full or ss holds no valid array (the array is empty,
 * ss's position unchanged).
 */
#define ds_ArrayTryDeserialize(_mem_, _ss_, _array_)                                            \
	ds_ArrayTryDeserializeInternal((_mem_), (_ss_), (void **) &(_array_).buf, &(_array_).length, \
	                               &(_array_).base, sizeof((_array_).buf[0]))


/*************************** stream: inline definitions ***************************/

static ds_ForceInline struct SS SSAlloc(struct arena *mem, const u64 bufsize)
{
	struct SS ss = { 0 };

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

static ds_ForceInline void SSDealloc(struct SS *ss)
{
	if (ss->heap_allocated)
	{
		ds_Free(ss->buf);
		ss->heap_allocated = 0;
	}
}

static ds_ForceInline void SSFlush(struct SS *ss)
{
    ss->bit_index = 0;
}

static ds_ForceInline void SSIncreaseSize(struct SS *ss, const u64 bufsize)
{
	ds_Assert(ss->heap_allocated);
	const u64 old_size = ss->bit_count / 8;
	if (bufsize <= old_size)
	{
		return;
	}

	const u64 new_size = U64Max(2 * old_size, bufsize);
	ss->buf = ds_Realloc(ss->buf, old_size, new_size, DS_CACHE_LINE);
	ss->bit_count = 8 * new_size;
}

static ds_ForceInline struct SS SSBuffered(void *buf, const u64 bufsize)
{
	struct SS ss =
	{
		.bit_count = bufsize * 8,
		.bit_index = 0,
		.buf = buf,
	};

	return ss;
}

static ds_ForceInline u32 SSTryView(struct SSView *view, const struct SS *ss, const u64 bit_offset, const u64 bit_count)
{
	if (bit_offset + bit_count > ss->bit_count || bit_offset + bit_count < bit_offset)
	{
		*view = (struct SSView) { 0 };
		return 0;
	}

	*view = (struct SSView) { .buf = ss->buf, .bit_offset = bit_offset, .bit_count = bit_count };
	return 1;
}

static ds_ForceInline struct SS SSSubstreamFromView(const struct SSView *view)
{
	struct SS ss =
	{
		.bit_index = view->bit_offset,
		.bit_count = view->bit_offset + view->bit_count,
		.buf = view->buf,
	};
	return ss;
}

static ds_ForceInline u64 SSBytesLeft(const struct SS *ss)
{
	return (ss->bit_count - ss->bit_index) >> 3;
}

static ds_ForceInline u64 SSBitsLeft(const struct SS *ss)
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

static ds_ForceInline b8 SSRead8(struct SS *ss)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	b8 val = { .u = ss->buf[offset], };
	return val;
}

static ds_ForceInline void SSWrite8(struct SS *ss, const b8 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert(ss->bit_index + 8 <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 8;
	*((b8 *) (ss->buf + offset)) = val;
}

static ds_ForceInline b16 SSRead16Le(struct SS *ss)
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

static ds_ForceInline void SSWrite16Le(struct SS *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	const b16 stored = NaToLe16(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b16 SSRead16Be(struct SS *ss)
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

static ds_ForceInline void SSWrite16Be(struct SS *ss, const b16 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 16) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 16;
	const b16 stored = NaToBe16(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b32 SSRead32Le(struct SS *ss)
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

static ds_ForceInline void SSWrite32Le(struct SS *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	const b32 stored = NaToLe32(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b32 SSRead32Be(struct SS *ss)
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

static ds_ForceInline void SSWrite32Be(struct SS *ss, const b32 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 32) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 32;
	const b32 stored = NaToBe32(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b64 SSRead64Le(struct SS *ss)
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

static ds_ForceInline void SSWrite64Le(struct SS *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	const b64 stored = NaToLe64(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

static ds_ForceInline b64 SSRead64Be(struct SS *ss)
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

static ds_ForceInline void SSWrite64Be(struct SS *ss, const b64 val)
{
	ds_Assert((ss->bit_index & 0x7) == 0);
	ds_Assert((ss->bit_index + 64) <= ss->bit_count);

	const u64 offset = ss->bit_index >> 3;
	ss->bit_index += 64;
	const b64 stored = NaToBe64(val);
	memcpy(ss->buf + offset, &stored, sizeof(stored));
}

/*************************** byte array access: inline definitions ***************************/

static ds_ForceInline void SSRead8N(struct SS *ss, b8 *buf, const u64 len)
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

static ds_ForceInline void SSWrite8N(struct SS *ss, const b8 *buf, const u64 len)
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

static ds_ForceInline void SSRead16LeN(struct SS *ss, b16 *buf, const u64 len)
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

static ds_ForceInline void SSWrite16LeN(struct SS *ss, const b16 *buf, const u64 len)
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

static ds_ForceInline void SSRead16BeN(struct SS *ss, b16 *buf, const u64 len)
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

static ds_ForceInline void SSWrite16BeN(struct SS *ss, const b16 *buf, const u64 len)
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

static ds_ForceInline void SSRead32LeN(struct SS *ss, b32 *buf, const u64 len)
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

static ds_ForceInline void SSWrite32LeN(struct SS *ss, const b32 *buf, const u64 len)
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

static ds_ForceInline void SSRead32BeN(struct SS *ss, b32 *buf, const u64 len)
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

static ds_ForceInline void SSWrite32BeN(struct SS *ss, const b32 *buf, const u64 len)
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

static ds_ForceInline void SSRead64LeN(struct SS *ss, b64 *buf, const u64 len)
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

static ds_ForceInline void SSWrite64LeN(struct SS *ss, const b64 *buf, const u64 len)
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

static ds_ForceInline void SSRead64BeN(struct SS *ss, b64 *buf, const u64 len)
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

static ds_ForceInline void SSWrite64BeN(struct SS *ss, const b64 *buf, const u64 len)
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
static const u64 SSPartialMask[65] =
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
static ds_ForceInline u64 SSReadU64Partial(struct SS *ss, const u64 bit_count)
{
	ds_Assert(bit_count <= 64);
	ds_Assert(ss->bit_index + bit_count <= ss->bit_count);

	const u64 p = ss->bit_index >> 3;
	const u64 n = ss->bit_index & 7;
	const u64 bytes_left = ((ss->bit_count + 7) >> 3) - p;	/* a substream may end inside its last byte */
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

	const u64 mask = SSPartialMask[bit_count];
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
static ds_ForceInline void SSWriteU64Partial(struct SS *ss, const u64 val, const u64 bit_count)
{
	ds_Assert(bit_count <= 64);
	ds_Assert(ss->bit_index + bit_count <= ss->bit_count);

	const u64 p = ss->bit_index >> 3;
	const u64 n = ss->bit_index & 7;
	const u64 bytes_left = ((ss->bit_count + 7) >> 3) - p;	/* a substream may end inside its last byte */
	const u64 masked = val & SSPartialMask[bit_count];
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

static ds_ForceInline void SSWriteI64Partial(struct SS *ss, const i64 val, const u64 bit_count)
{
	ds_Assert(1 <= bit_count);
	b64 trunc = { .i = val };
	const u64 mask = 0x7fffffffffffffff >> (64 - bit_count);
	trunc.u = (trunc.u & mask) | ((trunc.u & 0x8000000000000000) >> (64 - bit_count));
	SSWriteU64Partial(ss, trunc.u, bit_count);
}

static ds_ForceInline i64 SSReadI64Partial(struct SS *ss, const u64 bit_count)
{
	ds_Assert(1 <= bit_count);
	b64 trunc = { .u = SSReadU64Partial(ss, bit_count) };
	const u64 sign = trunc.u >> (bit_count - 1);
	trunc.u |= (0xffffffffffffffff * sign) << (bit_count - 1);
	return trunc.i;
}

/*************************** array: inline definitions ***************************/

static ds_ForceInline void ds_ArraySerializeInternal(struct SS *ss, const void *buf, const u32 length, const u64 slot_size)
{
	ds_Assert(sizeof(u32) + length*slot_size <= SSBytesLeft(ss));
	SSWrite32Le(ss, (b32) { .u = length });
	SSWrite8N(ss, (const b8 *) buf, length*slot_size);
}

static ds_ForceInline u32 ds_ArrayTryDeserializeInternal(struct arena *mem, struct SS *ss, void **buf, u32 *length, u32 *base, const u64 slot_size)
{
	*buf = NULL;
	*length = 0;
	*base = U32_MAX;
	if (SSBytesLeft(ss) < sizeof(u32))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u32 serialized_length = SSRead32Le(ss).u;
	/* division instead of length*slot_size: a corrupt length must not overflow the check */
	if (serialized_length > SSBytesLeft(ss) / slot_size)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	if (serialized_length > 0)
	{
		*buf = (mem) ? ArenaPush(mem, serialized_length*slot_size) : ds_Alloc(serialized_length*slot_size, DEFAULT_MEMORY_ALIGNMENT);
		if (*buf == NULL)
		{
			ss->bit_index = bit_index;
			return 0;
		}
		SSRead8N(ss, (b8 *) *buf, serialized_length*slot_size);
	}
	*length = serialized_length;
	return 1;
}

/*************************** macro definitions ***************************/


#define SSReadU8(ss)      	SSRead8(ss).u
#define SSWriteU8(ss, val)     SSWrite8(ss, (b8) { .u = val })
#define SSReadU16Le(ss)        SSRead16Le(ss).u
#define SSWriteU16Le(ss, val)  SSWrite16Le(ss, (b16) { .u = val })
#define SSReadU16Be(ss)        SSRead16Be(ss).u
#define SSWriteU16Be(ss, val)  SSWrite16Be(ss, (b16) { .u = val })
#define SSReadU32Le(ss)        SSRead32Le(ss).u
#define SSWriteU32Le(ss, val)  SSWrite32Le(ss, (b32) { .u = val })
#define SSReadU32Be(ss)        SSRead32Be(ss).u
#define SSWriteU32Be(ss, val)  SSWrite32Be(ss, (b32) { .u = val })
#define SSReadU64Le(ss)        SSRead64Le(ss).u
#define SSWriteU64Le(ss, val)  SSWrite64Le(ss, (b64) { .u = val })
#define SSReadU64Be(ss)        SSRead64Be(ss).u
#define SSWriteU64Be(ss, val)  SSWrite64Be(ss, (b64) { .u = val })

#define SSReadI8(ss)      	SSRead8(ss).i
#define SSWriteI8(ss, val)     SSWrite8(ss, (b8) { .i = val })
#define SSReadI16Le(ss)        SSRead16Le(ss).i
#define SSWriteI16Le(ss, val)  SSWrite16Le(ss, (b16) { .i = val })
#define SSReadI16Be(ss)        SSRead16Be(ss).i
#define SSWriteI16Be(ss, val)  SSWrite16Be(ss, (b16) { .i = val })
#define SSReadI32Le(ss)        SSRead32Le(ss).i
#define SSWriteI32Le(ss, val)  SSWrite32Le(ss, (b32) { .i = val })
#define SSReadI32Be(ss)        SSRead32Be(ss).i
#define SSWriteI32Be(ss, val)  SSWrite32Be(ss, (b32) { .i = val })
#define SSReadI64Le(ss)        SSRead64Le(ss).i
#define SSWriteI64Le(ss, val)  SSWrite64Le(ss, (b64) { .i = val })
#define SSReadI64Be(ss)        SSRead64Be(ss).i
#define SSWriteI64Be(ss, val)  SSWrite64Be(ss, (b64) { .i = val })

#define SSReadF32Le(ss)   	SSRead32Le(ss).f
#define SSWriteF32Le(ss, val)  SSWrite32Le(ss, (b32) { .f = val })
#define SSReadF32Be(ss)        SSRead32Be(ss).f
#define SSWriteF32Be(ss, val)  SSWrite32Be(ss, (b32) { .f = val })
#define SSReadF64Le(ss)        SSRead64Le(ss).f
#define SSWriteF64Le(ss, val)  SSWrite64Le(ss, (b64) { .f = val })
#define SSReadF64Be(ss)        SSRead64Be(ss).f
#define SSWriteF64Be(ss, val)  SSWrite64Be(ss, (b64) { .f = val })

#define SSReadU8N(ss, buf, len)		SSRead8N(ss, (b8 *) (buf), len)
#define SSWriteU8N(ss, buf, len)		SSWrite8N(ss, (b8 *) (buf), len)
#define SSReadU16LeN(ss, buf, len) 		SSRead16LeN(ss, (b16 *) (buf), len)
#define SSWriteU16LeN(ss, buf, len) 		SSWrite16LeN(ss, (b16 *) (buf), len)
#define SSReadU16BeN(ss, buf, len) 		SSRead16BeN(ss, (b16 *) (buf), len)
#define SSWriteU16BeN(ss, buf, len) 		SSWrite16BeN(ss, (b16 *) (buf), len)
#define SSReadU32LeN(ss, buf, len) 		SSRead32LeN(ss, (b32 *) (buf), len)
#define SSWriteU32LeN(ss, buf, len) 		SSWrite32LeN(ss, (b32 *) (buf), len)
#define SSReadU32BeN(ss, buf, len) 		SSRead32BeN(ss, (b32 *) (buf), len)
#define SSWriteU32BeN(ss, buf, len) 		SSWrite32BeN(ss, (b32 *) (buf), len)
#define SSReadU64LeN(ss, buf, len) 		SSRead64LeN(ss, (b64 *) (buf), len)
#define SSWriteU64LeN(ss, buf, len) 		SSWrite64LeN(ss, (b64 *) (buf), len)
#define SSReadU64BeN(ss, buf, len)		SSRead64BeN(ss, (b64 *) (buf), len)
#define SSWriteU64BeN(ss, buf, len) 		SSWrite64BeN(ss, (b64 *) (buf), len)

#define SSReadI8N(ss, buf, len)		SSRead8N(ss, (b8 *) (buf), len)
#define SSWriteI8N(ss, buf, len)		SSWrite8N(ss, (b8 *) (buf), len)
#define SSReadI16LeN(ss, buf, len) 		SSRead16LeN(ss, (b16 *) (buf), len)
#define SSWriteI16LeN(ss, buf, len)		SSWrite16LeN(ss, (b16 *) (buf), len)
#define SSReadI16BeN(ss, buf, len) 		SSRead16BeN(ss, (b16 *) (buf), len)
#define SSWriteI16BeN(ss, buf, len)		SSWrite16BeN(ss, (b16 *) (buf), len)
#define SSReadI32LeN(ss, buf, len) 		SSRead32LeN(ss, (b32 *) (buf), len)
#define SSWriteI32LeN(ss, buf, len)		SSWrite32LeN(ss, (b32 *) (buf), len)
#define SSReadI32BeN(ss, buf, len) 		SSRead32BeN(ss, (b32 *) (buf), len)
#define SSWriteI32BeN(ss, buf, len)		SSWrite32BeN(ss, (b32 *) (buf), len)
#define SSReadI64LeN(ss, buf, len) 		SSRead64LeN(ss, (b64 *) (buf), len)
#define SSWriteI64LeN(ss, buf, len)		SSWrite64LeN(ss, (b64 *) (buf), len)
#define SSReadI64BeN(ss, buf, len)		SSRead64BeN(ss, (b64 *) (buf), len)
#define SSWriteI64BeN(ss, buf, len)		SSWrite64BeN(ss, (b64 *) (buf), len)

#define SSReadF32LeN(ss, buf, len) 		SSRead32LeN(ss, (b32 *) (buf), len)
#define SSWriteF32LeN(ss, buf, len)		SSWrite32LeN(ss, (b32 *) (buf), len)
#define SSReadF32BeN(ss, buf, len) 		SSRead32BeN(ss, (b32 *) (buf), len)
#define SSWriteF32BeN(ss, buf, len)		SSWrite32BeN(ss, (b32 *) (buf), len)
#define SSReadF64LeN(ss, buf, len) 		SSRead64LeN(ss, (b64 *) (buf), len)
#define SSWriteF64LeN(ss, buf, len)		SSWrite64LeN(ss, (b64 *) (buf), len)
#define SSReadF64BeN(ss, buf, len)		SSRead64BeN(ss, (b64 *) (buf), len)
#define SSWriteF64BeN(ss, buf, len)		SSWrite64BeN(ss, (b64 *) (buf), len)

#define SSWriteU8Partial(ss, val, bit_count)		SSWriteU64Partial(ss, val, bit_count)
#define SSWriteU16Partial(ss, val, bit_count)		SSWriteU64Partial(ss, val, bit_count)
#define SSWriteU32Partial(ss, val, bit_count)		SSWriteU64Partial(ss, val, bit_count)
#define SSReadU8Partial(ss, bit_count)			((u8) SSReadU64Partial(ss, bit_count))
#define SSReadU16Partial(ss, bit_count)		((u16) SSReadU64Partial(ss, bit_count))
#define SSReadU32Partial(ss, bit_count)		((u32) SSReadU64Partial(ss, bit_count))
#define SSWriteI8Partial(ss, val, bit_count)		SSWriteI64Partial(ss, val, bit_count)
#define SSWriteI16Partial(ss, val, bit_count)		SSWriteI64Partial(ss, val, bit_count)
#define SSWriteI32Partial(ss, val, bit_count)		SSWriteI64Partial(ss, val, bit_count)
#define SSReadI8Partial(ss, bit_count)			((i8) SSReadI64Partial(ss, bit_count))
#define SSReadI16Partial(ss, bit_count)		((i16) SSReadI64Partial(ss, bit_count))
#define SSReadI32Partial(ss, bit_count)		((i32) SSReadI64Partial(ss, bit_count))

#ifdef __cplusplus
} 
#endif

#endif

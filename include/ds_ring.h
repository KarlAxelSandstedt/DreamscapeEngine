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
ds_ring.h
=========
Ring: ring buffer whose end wraps through mirrored virtual memory (Linux, Windows).
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_RING_H__
#define __DS_RING_H__

/*
Ring API
--------
Ring buffer mapped twice back to back in virtual memory, so an allocation that runs past the end continues
at the start and is still contiguous. Web (no virtual memory): buf is a plain 2*mem_total block; every function
behaves the same, but the halves do not mirror each other, so reach a slot only through its own address.
*/

/*
The bytes in use are the mem_total - mem_left bytes ending at offset (wrapping), from the start (oldest)
to the end (newest).
*/
struct ring
{
	u64 mem_total;
	u64 mem_left;
	u64 offset;	/* end of the bytes in use: where RingPushEnd writes */
	u8 *buf;	/* buf[0, 2*mem_total): the second half mirrors the first (not on web) */
};

/* RingPushStart/RingPushEnd result: { NULL, 0 } on failure */
struct ringSlot
{
	u8 *	address;
	u64	size;
};

/* Return the empty ring (no memory). */
struct ring 	    RingEmpty(void);	
/*
Return a ring of at least mem_hint bytes (Linux, web: rounded up to whole pages; Windows: to a power of two of
at least the allocation granularity, 64 KiB), or the empty ring on failure.
*/
struct ring 	    RingAlloc(const u64 mem_hint);	
/* Unmap the ring and leave it empty. */
void 		        RingDealloc(struct ring *ring);			
/* Release all bytes and restart at offset 0. */
void		        RingFlush(struct ring *ring);
/* Push size bytes before the start (the end and offset stay); { NULL, 0 } if size == 0 or they don't fit. */
struct ringSlot 	RingPushStart(struct ring *ring, const u64 size);
/* Push size bytes at the end, advancing offset; { NULL, 0 } if size == 0 or they don't fit. */
struct ringSlot 	RingPushEnd(struct ring *ring, const u64 size);
/* Release the size oldest bytes (FIFO with RingPushEnd). */
void 		        RingPopStart(struct ring *ring, const u64 size); 
/* Release the size newest bytes, moving offset back (LIFO with RingPushEnd). */
void 		        RingPopEnd(struct ring *ring, const u64 size); 

#endif

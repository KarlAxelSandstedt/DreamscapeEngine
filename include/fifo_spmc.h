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

#ifndef __FIFO_SPMC_H__
#define __FIFO_SPMC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_semaphore.h"

/*
 * To be placed in user node structure 
 */
struct fifoSpmcEntry
{
	// PAD CACHELINE?
	u32 in_use;
	void *data;
};

/*
 * fifoSpmc - first in first out single producer multiple consumer queue.
 *
 * Invariant: 	(1) a_first always points to the next entry to be reserved 
 * 	     	(2) if able_for_reservation > 0, there must be equally many valid entries starting from a_first
 * 	     	(3) a_alloc always points to the next entry to be allocated
 * 	     	(4) Before entry[a_alloc] is allocated, the entry may not be in_use
 * 	     	(5) Before incrementing the semaphore, we must have allocated a new entry and incremented a_alloc
 */
struct fifoSpmc
{
	struct fifoSpmcEntry *entries;
	/* TODO CACHELINE */
	semaphore able_for_reservation; 	/* Master publishes work able for reservation */
	/* TODO CACHELINE */
	u32 a_first;				/* Consumer owned */
	/* TODO CACHELINE */
	u32 next_alloc;				/* Producer owned */
	/* TODO CACHELINE */
	u32 max_entry_count;			/* Must be power of 2, so that on overflow of tickers, 
						 * modulo count still is correct. It may not be U32_MAX. 
						 */
};

/*
 * Issue (1): consumer must be able to atomically get a valid entry AND update so that next valid entry is
 * 	      immediately visible.
 *
 * Solution(1): Reserving an entry is done via a semaphore, and the consumer can then determine which entry
 * 		is entitled to by atomically_icrementing an index. 
 *
 * Issue(2): How does the producer know which entries are free?
 *
 * Solution(2): 
 * 	Publishing: The producer picks the next entry to publish. Before publication, the entry is marked
 * 		    as in_use. If the entry is in_use when the producer first arrive, it should spin until
 * 		    it is no longer marked, or go to sleep. We should perhaps do an assertion here, as it
 * 		    should not happen.
 *
 * 	Releasing: Before the consumer is done with the entry, it should mark the entry as no longer in use,
 * 		   so that the producer may republish the entry.
 *
 * Every entry is marked with a in_use by the producer before publish. When a consumer is 
 * done with the entry, it removes the mark.
 */

struct fifoSpmc *	FifoSpmcInit(struct arena *mem_persistent, const u32 max_entry_count);
void 			FifoSpmcDestroy(struct fifoSpmc *q);
/* returns NULL on pop failure, otherwise pointer to producer memory */
void *			FifoSpmcPop(struct fifoSpmc *q);
/* returns 0 push failure, otherwise return 1 */
u32 			FifoSpmcTryPush(struct fifoSpmc *q, void *data);
/* spinlock push data */
void			FifoSpmcPush(struct fifoSpmc *q, void *data);
/* return the current number of slots ready to be pushed */
u32 			FifoSpmcPushableCount(const struct fifoSpmc *q);

#ifdef __cplusplus
} 
#endif

#endif

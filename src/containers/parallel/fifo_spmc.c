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

#include "ds_base.h"
#include "fifo_spmc.h"

struct fifoSpmc *FifoSpmcInit(struct arena *mem_persistent, const u32 max_entry_count)
{
	ds_Assert(max_entry_count > 0 && PowerOfTwoCheck(max_entry_count));
	struct fifoSpmc *q = NULL; 

	q = ArenaPush(mem_persistent, sizeof(struct fifoSpmc));
	q->max_entry_count = max_entry_count;
	q->entries = ArenaPush(mem_persistent, q->max_entry_count * sizeof(struct fifoSpmcEntry));
	for (u32 i = 0; i < q->max_entry_count; ++i)
	{
		q->entries[i].in_use = 0;
		q->entries[i].data = NULL;
	}

	q->next_alloc = 0;
	SemaphoreInit(&q->able_for_reservation, 0);
	AtomicStoreRel32(&q->a_first, 0);

	return q;
}

void FifoSpmcDestroy(struct fifoSpmc *q)
{
	SemaphoreDestroy(&q->able_for_reservation);
}

void *FifoSpmcPop(struct fifoSpmc *q)
{
	/* 
	 * Whatever index we get, we own. The operation only have to be atomic since the producer 
	 * makes sure not to not publish an index (through the semaphore) before having written
	 * everything out into the entry. 
	 */
	const u32 i = AtomicFetchAddRlx32(&q->a_first, 1) % q->max_entry_count;
	void *data = (void *) AtomicLoadAcq64(&q->entries[i].data);

	ds_AssertString(q->entries[i].in_use == 1, "If this happens, we have a race condition and in_use must be ATOMIC_AQUIRE.\n This seems unreasonable, because requiring aquire would mean that this CPU\n would not let the release CPU finish its local loads BEFORE its release \n which at this point has happened and finished.\n");

	/* 
	 * Local rw reorder barrier for above entries, releasing the barrier enforces the above loads to finish
	 * (become visible) before the producer may reallocate the entry. 
	 */
	AtomicStoreRel32(&q->entries[i].in_use, 0);

	return data;
}

u32 FifoSpmcPushableCount(const struct fifoSpmc *q)
{
	u32 count = 0;

	while (count < q->max_entry_count)
	{
		const u32 next = (q->next_alloc + count) % q->max_entry_count;
		/* acquire here is important, as we do not know what the caller may wish to do with the local data */
		if (AtomicLoadAcq32(&q->entries[next].in_use))
		{
			break;
		}

		count += 1;
	}

	return count;
}

u32 FifoSpmcTryPush(struct fifoSpmc *q, void *data)
{
	const u32 next = q->next_alloc % q->max_entry_count;
	//const u32 next = atomic_load(&q->alloc, ATOMIC_RELAXED) % q->max_entry_count;
	const u32 in_use = AtomicLoadRlx32(&q->entries[next].in_use);
	/* Local rw reorder barrier + if not in use, we can be sure that the entry can be changed */

	if (in_use) { return 0; }

	//atomic_store(&q->a_alloc, next + 1, ATOMIC_RELAXED);
	//atomic_store(&q->entries[next].in_use, 1, ATOMIC_RELAXED);
	///* Local rw reorder barrier + we force any worker popping here to see all writes   */
	//atomic_store(&q->entries[next].data, data, ATOMIC_RELEASE);
	
	q->next_alloc = next + 1;
	q->entries[next].in_use = 1;
	AtomicStoreRel64(&q->entries[next].data, data);
	/* ACQ_SEQ */
	SemaphorePost(&q->able_for_reservation);

	return 1;
}

void FifoSpmcPush(struct fifoSpmc *q, void *data)
{
	while (!FifoSpmcTryPush(q, data));
}

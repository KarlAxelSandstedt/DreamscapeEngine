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
#include "ds_hierarchy.h"

/*
 * The cold parts of the hierarchy; Add, Remove and the adopts are force-inlined in ds_hierarchy.h, so the
 * macros' node size and link offsets are constants there (out of line they were up to 2x slower: the
 * Hierarchy Performance suite).
 */

static void hi_InitFixed(const struct HIPool pool, const struct HILayout layout)
{
	const struct slot root = ds_PoolAddInternal(pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, *pool.growable, layout.slot_size, layout.pool_slot_offset);
	const struct slot orphan = ds_PoolAddInternal(pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, *pool.growable, layout.slot_size, layout.pool_slot_offset);
	ds_Assert(root.index == HI_ROOT && orphan.index == HI_ORPHAN);

	for (u32 i = HI_ROOT; i <= HI_ORPHAN; ++i)
	{
		i32 *n = HILinksInternal(pool, layout, i);
		n[HI_LINK_PARENT] = HI_NULL;
		n[HI_LINK_PREV] = HI_NULL;
		n[HI_LINK_NEXT] = HI_NULL;
		n[HI_LINK_FIRST] = HI_NULL;
		n[HI_LINK_LAST] = HI_NULL;
		n[HI_LINK_CHILD_COUNT] = 0;
	}
}

u32 HIAllocInternal(struct arena *mem, const struct HIPool pool, const struct HILayout layout, const u32 initial_length, const u32 growable)
{
	if (!ds_PoolAllocInternal(mem, pool.heap_allocated, pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, pool.growable, layout.slot_size, initial_length + 2, growable))
	{
		return 0;
	}

	hi_InitFixed(pool, layout);
	return 1;
}

void HIFlushInternal(const struct HIPool pool, const struct HILayout layout)
{
	/* ds_PoolFlush, untyped */
	*pool.count = 0;
	*pool.count_max = 0;
	*pool.next_free = POOL_INDEX_MASK;
	PoisonAddress(*pool.buf, layout.slot_size * *pool.length);

	hi_InitFixed(pool, layout);
}

void HIRemoveStackFatalInternal(void)
{
	LogString(T_SYSTEM, S_FATAL, "Stack OOM in HIRemove");
	FatalCleanupAndExit();
}

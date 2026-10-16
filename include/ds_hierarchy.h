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
ds_hierarchy.h
==============
A hierarchy (a tree with ordered children) of struct T nodes in a ds_Pool(T): nodes link to their parent,
siblings and first/last child by pool index, so indices are the handles. Two fixed nodes exist from Alloc
on: HI_ROOT (index 0) and HI_ORPHAN (index 1), each with no parent. Not thread-safe.
Sections: HI, the hierarchy (below), and HII, a depth-first iterator over a node's sub-hierarchy.

    struct node { HI_NODE; ... };
    DEFINE_HI(node);                                    ds_Pool(node) and the type HI(node) = { ds_Pool(node) pool; }

    HI(node) hi;
    HIAlloc(NULL, hi, 64, GROWABLE);
    const struct slot child = HIAdd(hi, HI_ROOT);
    HIRemove(tmp, hi, child.index);
    HIDealloc(hi);
*/

#ifndef __DS_HIERARCHY_H__
#define __DS_HIERARCHY_H__

#include "ds_allocator.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
HI
--
The macros: hi is a HI(T) lvalue, indices are u32, and arguments may be evaluated more than once. The
commented-out #defines are their declarations; the definitions are at the bottom.
*/

#define	HI_NULL	    -1
#define HI_ROOT	    0
#define HI_ORPHAN	1

/* the link fields, in this order (the implementation reads them as i32[6] from hi_parent) */
#define HI_NODE                 \
    i32         hi_parent;      \
    i32         hi_next;        \
    i32         hi_prev;        \
    i32         hi_first;       \
    i32         hi_last;        \
    u32         hi_child_count; \
    POOL_NODE

#define HI(T) HI_ ## T

#define DEFINE_HI(T)            \
DEFINE_POOL(T);                 \
typedef struct HI(T)            \
{                               \
    ds_Pool(T)  pool;           \
} HI(T)

/*
 * Set up the pool for initial_length nodes (plus the two fixed ones) in mem (fixed) or on the heap
 * (mem == NULL; growable allowed). Returns (u32) 1; 0 if the memory can't be allocated (hi.pool.buf NULL).
 */
//#define HIAlloc(mem, hi, initial_length, growable)

/* Free a heap hierarchy's memory. */
//#define HIDealloc(hi)

/* Remove every node but the two fixed ones. */
//#define HIFlush(hi)

/*
 * Add a node as parent's last child and return its struct slot { address, index }; its fields other than
 * the links are uninitialized. A full fixed pool: { NULL, HI_NULL (U32_MAX) }.
 */
//#define HIAdd(hi, parent)

/*
 * Remove index (> 1) and its whole sub-hierarchy. tmp holds a stack while the sub-hierarchy is walked
 * (fatal if it runs out).
 */
//#define HIRemove(tmp, hi, index)

/*
 * HIRemove that first calls custom_free(void *node, u32 index, void *data) on each removed node: the
 * sub-hierarchy depth first while it is being taken apart, index last, so custom_free must neither walk
 * nor change the hierarchy.
 */
//#define HIApplyCustomFreeAndRemove(tmp, hi, index, custom_free, data)

/*
 * Make index (> 1) new_parent's last child, with its sub-hierarchy. new_parent must not be in that
 * sub-hierarchy (debug assert: it would make a cycle).
 */
//#define HIAdoptNode(hi, index, new_parent)

/*
 * Make index (> 1) new_parent's last child alone: its children take its place among its old siblings, in
 * order. new_parent is any node but index.
 */
//#define HIAdoptNodeExclusive(hi, index, new_parent)

/*
HII
---
Iterator over a node and its entire sub-hierarchy in depth-first order (pre-order, root first), never its
siblings. The iteration is finished when at == root again; another HIIAdvance would start over.

    HIIInit(it, hi, root);
    do { visit(it.at); HIIAdvance(it, hi); } while (it.at != root);

To skip (or remove) the sub-hierarchy of it.at: HIISkip, then remove it, then HIIAdvance.

The macros: it is an HII lvalue, hi a T ## HI lvalue; arguments may be evaluated more than once.
*/

typedef struct HII
{
    i32 root;
    i32 at;
    i32 next;
} HII;

/*
 * Start at root: it.at = root; it.next = root's first child, or root if it has none (then the first
 * HIIAdvance finishes).
 */
//#define HIIInit(it, hi, root)

/* Move to it.next and set it.next to the node after it depth first (root when none is left). */
//#define HIIAdvance(it, hi)

/*
 * Set it.next to the node after it.at's sub-hierarchy depth first (root when none is left; at the root
 * itself: root). The sub-hierarchy may then be removed; it.at's links are not read again.
 */
//#define HIISkip(it, hi)

/******************************** HI inline definitions ********************************/

/* Internal: the pool's fields and where the links lie in T, for the untyped implementation. */
struct HIPool
{
	void ** buf;
	u32 *   length;
	u32 *   count;
	u32 *   count_max;
	u32 *   next_free;
	u32 *   growable;
	u32 *   heap_allocated;
};

struct HILayout
{
	u64 slot_size;
	u64 pool_slot_offset;
	u64 link_offset;
};

#define HIPoolFields(hi)                                                                                \
	((struct HIPool) { (void **) &(hi).pool.buf, &(hi).pool.length, &(hi).pool.count,                  \
	                   &(hi).pool.count_max, &(hi).pool.next_free, &(hi).pool.growable,                 \
	                   &(hi).pool.heap_allocated })

#define HILayout(hi)                                                                                    \
	((struct HILayout) { sizeof((hi).pool.buf[0]), ds_PoolSlotOffset((hi).pool),                       \
	                     (u64) &((1 ? NULL : (hi).pool.buf)->hi_parent) })

/* Internal: the implementation behind the macros. Add, Remove and the adopts are force-inlined (constant node size and offsets). */
u32         HIAllocInternal(struct arena *mem, const struct HIPool pool, const struct HILayout layout, const u32 initial_length, const u32 growable);
void        HIFlushInternal(const struct HIPool pool, const struct HILayout layout);
void        HIRemoveStackFatalInternal(void);
static ds_ForceInline i32 * HILinksInternal(const struct HIPool pool, const struct HILayout layout, const i64 index);
static ds_ForceInline struct slot HIAddInternal(const struct HIPool pool, const struct HILayout layout, const u32 parent);
static ds_ForceInline void HIRemoveInternal(struct arena *tmp, const struct HIPool pool, const struct HILayout layout, const u32 index, void (*custom_free)(void *node, const u32 index, void *data), void *data);
static ds_ForceInline void HIAdoptNodeInternal(const struct HIPool pool, const struct HILayout layout, const u32 index, const u32 new_parent);
static ds_ForceInline void HIAdoptNodeExclusiveInternal(const struct HIPool pool, const struct HILayout layout, const u32 index, const u32 new_parent);

#define HIAlloc(mem, hi, initial_length, growable)                                                      \
	HIAllocInternal((mem), HIPoolFields(hi), HILayout(hi), (initial_length), (growable))
#define HIDealloc(hi)                       ds_PoolDealloc((hi).pool)
#define HIFlush(hi)                         HIFlushInternal(HIPoolFields(hi), HILayout(hi))
#define HIAdd(hi, parent)                   HIAddInternal(HIPoolFields(hi), HILayout(hi), (parent))
#define HIRemove(tmp, hi, index)            HIRemoveInternal((tmp), HIPoolFields(hi), HILayout(hi), (index), NULL, NULL)
#define HIApplyCustomFreeAndRemove(tmp, hi, index, custom_free, data)                                   \
	HIRemoveInternal((tmp), HIPoolFields(hi), HILayout(hi), (index), (custom_free), (data))
#define HIAdoptNode(hi, index, new_parent)  HIAdoptNodeInternal(HIPoolFields(hi), HILayout(hi), (index), (new_parent))
#define HIAdoptNodeExclusive(hi, index, new_parent)                                                     \
	HIAdoptNodeExclusiveInternal(HIPoolFields(hi), HILayout(hi), (index), (new_parent))

/* a node's links: HI_NODE's six 32-bit fields from hi_parent on */
#define HI_LINK_PARENT      0
#define HI_LINK_NEXT        1
#define HI_LINK_PREV        2
#define HI_LINK_FIRST       3
#define HI_LINK_LAST        4
#define HI_LINK_CHILD_COUNT 5

/* index HI_NULL (-1) is the pool's stub slot (buf[-1]): a missing relative is written to, not branched on */
static ds_ForceInline i32 *HILinksInternal(const struct HIPool pool, const struct HILayout layout, const i64 index)
{
	return (i32 *) ((u8 *) *pool.buf + index*(i64) layout.slot_size + layout.link_offset);
}

static ds_ForceInline struct slot HIAddInternal(const struct HIPool pool, const struct HILayout layout, const u32 parent_index)
{
	ds_Assert(parent_index < *pool.count_max);

	/* add before taking addresses: a growable pool may move */
	const struct slot slot = ds_PoolAddInternal(pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, *pool.growable, layout.slot_size, layout.pool_slot_offset);
	if (slot.index == U32_MAX)
	{
		return (struct slot) { .index = HI_NULL, .address = NULL };
	}

	i32 *parent = HILinksInternal(pool, layout, parent_index);
	i32 *last = HILinksInternal(pool, layout, parent[HI_LINK_LAST]);
	i32 *node = HILinksInternal(pool, layout, slot.index);

	last[HI_LINK_NEXT] = (i32) slot.index;

	node[HI_LINK_PARENT] = (i32) parent_index;
	node[HI_LINK_NEXT] = HI_NULL;
	node[HI_LINK_PREV] = parent[HI_LINK_LAST];
	node[HI_LINK_FIRST] = HI_NULL;
	node[HI_LINK_LAST] = HI_NULL;
	node[HI_LINK_CHILD_COUNT] = 0;

	parent[HI_LINK_CHILD_COUNT] += 1;
	parent[HI_LINK_LAST] = (i32) slot.index;
	if (parent[HI_LINK_FIRST] == HI_NULL)
	{
		parent[HI_LINK_FIRST] = (i32) slot.index;
	}

	return slot;
}

static ds_ForceInline void HIRemoveInternal(struct arena *tmp, const struct HIPool pool, const struct HILayout layout, const u32 index, void (*custom_free)(void *node, const u32 index, void *data), void *data)
{
	/* removal never reallocates (nor may custom_free): one read of the buffer, which the stores below could alias */
	u8 *const base = (u8 *) *pool.buf;
	ds_Assert(1 < index && index < *pool.count_max);

	const i32 *node = ((i32 *) (base + (i64) (index)*(i64) layout.slot_size + layout.link_offset));
	const i32 parent = node[HI_LINK_PARENT];
	const i32 first = node[HI_LINK_FIRST];
	const i32 next = node[HI_LINK_NEXT];
	const i32 prev = node[HI_LINK_PREV];

	/* the sub-hierarchy: depth first through an explicit stack (pool slots stay where they are) */
	if (first != HI_NULL)
	{
		ArenaPushRecord(tmp);
		struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), sizeof(u32));
		u32 *stack = arr.addr;
		if (!stack)
		{
			HIRemoveStackFatalInternal();
		}

		u32 sc = 1;
		stack[0] = (u32) first;
		while (sc--)
		{
			const u32 sub_index = stack[sc];
			const i32 *sub = ((i32 *) (base + (i64) (sub_index)*(i64) layout.slot_size + layout.link_offset));
			const i32 sub_first = sub[HI_LINK_FIRST];
			const i32 sub_next = sub[HI_LINK_NEXT];
			if (sub_first != HI_NULL)
			{
				stack[sc++] = (u32) sub_first;
			}

			if (sub_next != HI_NULL)
			{
				if (sc == arr.len)
				{
					HIRemoveStackFatalInternal();
				}
				stack[sc++] = (u32) sub_next;
			}

			if (custom_free)
			{
				custom_free((u8 *) sub - layout.link_offset, sub_index, data);
			}
			ds_PoolRemoveInternal(base, pool.count, pool.next_free, *pool.count_max, sub_index, layout.slot_size, layout.pool_slot_offset);
		}
		ArenaPopRecord(tmp);
	}

	i32 *p = ((i32 *) (base + (i64) (parent)*(i64) layout.slot_size + layout.link_offset));
	p[HI_LINK_CHILD_COUNT] -= 1;
	if (p[HI_LINK_FIRST] == (i32) index)
	{
		p[HI_LINK_FIRST] = next;
	}

	if (p[HI_LINK_LAST] == (i32) index)
	{
		p[HI_LINK_LAST] = prev;
	}

	((i32 *) (base + (i64) (next)*(i64) layout.slot_size + layout.link_offset))[HI_LINK_PREV] = prev;
	((i32 *) (base + (i64) (prev)*(i64) layout.slot_size + layout.link_offset))[HI_LINK_NEXT] = next;

	if (custom_free)
	{
		custom_free((u8 *) node - layout.link_offset, index, data);
	}
	ds_PoolRemoveInternal(base, pool.count, pool.next_free, *pool.count_max, index, layout.slot_size, layout.pool_slot_offset);
}

static ds_ForceInline void HIAdoptNodeInternal(const struct HIPool pool, const struct HILayout layout, const u32 index, const u32 new_parent_index)
{
	ds_Assert(1 < index && index < *pool.count_max && new_parent_index < *pool.count_max);
#ifdef DS_ASSERT_DEBUG
	/* adopting into the node's own sub-hierarchy would make a parent cycle */
	for (i32 a = (i32) new_parent_index; a != HI_NULL; a = HILinksInternal(pool, layout, a)[HI_LINK_PARENT])
	{
		ds_AssertString(a != (i32) index, "new parent is inside the node's sub-hierarchy");
	}
#endif

	i32 *node = HILinksInternal(pool, layout, index);
	i32 *prev = HILinksInternal(pool, layout, node[HI_LINK_PREV]);
	i32 *next = HILinksInternal(pool, layout, node[HI_LINK_NEXT]);
	i32 *parent = HILinksInternal(pool, layout, node[HI_LINK_PARENT]);

	next[HI_LINK_PREV] = node[HI_LINK_PREV];
	prev[HI_LINK_NEXT] = node[HI_LINK_NEXT];

	parent[HI_LINK_CHILD_COUNT] -= 1;
	if (parent[HI_LINK_FIRST] == (i32) index)
	{
		parent[HI_LINK_FIRST] = node[HI_LINK_NEXT];
	}

	if (parent[HI_LINK_LAST] == (i32) index)
	{
		parent[HI_LINK_LAST] = node[HI_LINK_PREV];
	}

	parent = HILinksInternal(pool, layout, new_parent_index);
	i32 *last = HILinksInternal(pool, layout, parent[HI_LINK_LAST]);
	last[HI_LINK_NEXT] = (i32) index;
	node[HI_LINK_PREV] = parent[HI_LINK_LAST];
	node[HI_LINK_NEXT] = HI_NULL;
	node[HI_LINK_PARENT] = (i32) new_parent_index;
	parent[HI_LINK_LAST] = (i32) index;
	if (parent[HI_LINK_FIRST] == HI_NULL)
	{
		parent[HI_LINK_FIRST] = (i32) index;
	}
	parent[HI_LINK_CHILD_COUNT] += 1;
}

static ds_ForceInline void HIAdoptNodeExclusiveInternal(const struct HIPool pool, const struct HILayout layout, const u32 index, const u32 new_parent_index)
{
	ds_Assert(1 < index && index < *pool.count_max && new_parent_index < *pool.count_max);
	ds_Assert(new_parent_index != index);

	i32 *node = HILinksInternal(pool, layout, index);
	i32 *prev = HILinksInternal(pool, layout, node[HI_LINK_PREV]);
	i32 *next = HILinksInternal(pool, layout, node[HI_LINK_NEXT]);
	i32 *first = HILinksInternal(pool, layout, node[HI_LINK_FIRST]);
	i32 *last = HILinksInternal(pool, layout, node[HI_LINK_LAST]);
	i32 *parent = HILinksInternal(pool, layout, node[HI_LINK_PARENT]);

	parent[HI_LINK_CHILD_COUNT] += node[HI_LINK_CHILD_COUNT] - 1;

	if (node[HI_LINK_CHILD_COUNT])
	{
		if (parent[HI_LINK_FIRST] == (i32) index)
		{
			parent[HI_LINK_FIRST] = node[HI_LINK_FIRST];
		}

		if (parent[HI_LINK_LAST] == (i32) index)
		{
			parent[HI_LINK_LAST] = node[HI_LINK_LAST];
		}
		prev[HI_LINK_NEXT] = node[HI_LINK_FIRST];
		first[HI_LINK_PREV] = node[HI_LINK_PREV];

		next[HI_LINK_PREV] = node[HI_LINK_LAST];
		last[HI_LINK_NEXT] = node[HI_LINK_NEXT];

		/* last's next already continues into node's old siblings: stop at last */
		for (i32 *child = first;; child = HILinksInternal(pool, layout, child[HI_LINK_NEXT]))
		{
			child[HI_LINK_PARENT] = node[HI_LINK_PARENT];
			if (child == last)
			{
				break;
			}
		}
	}
	else
	{
		if (parent[HI_LINK_FIRST] == (i32) index)
		{
			parent[HI_LINK_FIRST] = node[HI_LINK_NEXT];
		}

		if (parent[HI_LINK_LAST] == (i32) index)
		{
			parent[HI_LINK_LAST] = node[HI_LINK_PREV];
		}
		prev[HI_LINK_NEXT] = node[HI_LINK_NEXT];
		next[HI_LINK_PREV] = node[HI_LINK_PREV];
	}

	parent = HILinksInternal(pool, layout, new_parent_index);
	last = HILinksInternal(pool, layout, parent[HI_LINK_LAST]);

	last[HI_LINK_NEXT] = (i32) index;
	node[HI_LINK_PARENT] = (i32) new_parent_index;
	node[HI_LINK_PREV] = parent[HI_LINK_LAST];
	node[HI_LINK_NEXT] = HI_NULL;
	node[HI_LINK_FIRST] = HI_NULL;
	node[HI_LINK_LAST] = HI_NULL;
	node[HI_LINK_CHILD_COUNT] = 0;

	parent[HI_LINK_CHILD_COUNT] += 1;
	parent[HI_LINK_LAST] = (i32) index;
	if (parent[HI_LINK_FIRST] == HI_NULL)
	{
		parent[HI_LINK_FIRST] = (i32) index;
	}
}

/******************************** HII definitions ********************************/

#define HIIInit(_hii_, _hi_, _root_)                                                \
do                                                                                  \
{                                                                                   \
    (_hii_).root = (i32) (_root_);                                                  \
    (_hii_).at = (i32) (_root_);                                                    \
                                                                                    \
    /* root's siblings are outside its sub-hierarchy: no children means finished */ \
    const i32 _first_ = (_hi_).pool.buf[(_hii_).at].hi_first;                       \
    (_hii_).next = (_first_ != HI_NULL) ? _first_ : (i32) (_root_);                 \
} while (0)

#define HIIAdvance(_hii_, _hi_)                                                                             \
do                                                                                                          \
{                                                                                                           \
    ds_Assert((_hii_).at != (_hii_).root                                                                    \
            || ((_hii_).next == (_hi_).pool.buf[(_hii_).root].hi_first                                      \
            || ((_hii_).next == (_hii_).root)));                                                            \
    (_hii_).at = (_hii_).next;                                                                              \
                                                                                                            \
    const i32 _first_ = (_hi_).pool.buf[(_hii_).next].hi_first;                                             \
    if (_first_ != HI_NULL)                                                                                 \
    {                                                                                                       \
        (_hii_).next = _first_;                                                                             \
        break;                                                                                              \
    }                                                                                                       \
                                                                                                            \
    HIISkip(_hii_, _hi_);                                                                                   \
}                                                                                                           \
while (0)

#define HIISkip(_hii_, _hi_)                                                                                \
do                                                                                                          \
{                                                                                                           \
    (_hii_).next = (_hii_).at;                                                                              \
    if ((_hii_).at == (_hii_).root)                                                                         \
    {                                                                                                       \
        break;                                                                                              \
    }                                                                                                       \
                                                                                                            \
    i32 _next_ = (_hi_).pool.buf[(_hii_).next].hi_next;                                                     \
    if (_next_ != HI_NULL)                                                                                  \
    {                                                                                                       \
        (_hii_).next = _next_;                                                                              \
        break;                                                                                              \
    }                                                                                                       \
                                                                                                            \
    do                                                                                                      \
    {                                                                                                       \
        (_hii_).next = (_hi_).pool.buf[(_hii_).next].hi_parent;                                             \
        if ((_hii_).next == (_hii_).root)                                                                   \
        {                                                                                                   \
            (_hii_).next = (_hii_).root;                                                                    \
            break;                                                                                          \
        }                                                                                                   \
                                                                                                            \
        _next_ = (_hi_).pool.buf[(_hii_).next].hi_next;                                                     \
        if (_next_ != HI_NULL)                                                                              \
        {                                                                                                   \
            (_hii_).next = _next_;                                                                          \
            break;                                                                                          \
        }                                                                                                   \
    } while (1);                                                                                            \
}                                                                                                           \
while (0)

#ifdef __cplusplus
}
#endif

#endif

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

#ifndef __HIERARCHY_INDEX_H__
#define __HIERARCHY_INDEX_H__

#include "ds_allocator.h"

#ifdef __cplusplus
extern "C" { 
#endif

#define	HI_NULL	    -1
#define HI_ROOT	    0
#define HI_ORPHAN	1

/* debug: node is no ancestor of (or equal to) new_parent; adopting into its own subtree makes a cycle */
#ifdef DS_ASSERT_DEBUG
#define HI_ASSERT_NOT_IN_SUBTREE(hi, node, new_parent)                                        \
    for (i32 _a_ = (i32) (new_parent); _a_ != HI_NULL; _a_ = (hi)->pool.buf[_a_].hi_parent)   \
    {                                                                                         \
        ds_AssertString(_a_ != (i32) (node), "new parent is inside the node's sub-hierarchy");\
    }
#else
#define HI_ASSERT_NOT_IN_SUBTREE(hi, node, new_parent)
#endif

#define HI_NODE                 \
    i32         hi_parent;      \
    i32         hi_next;        \
    i32         hi_prev;        \
    i32         hi_first;       \
    i32         hi_last;        \
    u32         hi_child_count; \
    POOL_NODE

#define DEFINE_HI_STRUCT(T)     \
typedef struct T ## HI          \
{                               \
    ds_Pool(T)  pool;           \
} T ## HI

#define HI_DECLARE(T)                           \
    DEFINE_POOL(T);                            \
    DEFINE_HI_STRUCT(T);                        \
    DECLARE_HI_ALLOC(T);                        \
    DECLARE_HI_DEALLOC(T);                      \
    DECLARE_HI_FLUSH(T);                        \
    DECLARE_HI_ADD(T);                          \
    DECLARE_HI_REMOVE(T);                       \
    DECLARE_HI_ADOPT_NODE_EXCLUSIVE(T);         \
    DECLARE_HI_ADOPT(T);                        \
    DECLARE_HI_APPLY_CUSTOM_FREE_AND_REMOVE(T)        

#define HI_DEFINE(T)                            \
    DEFINE_HI_ALLOC(T)                          \
    DEFINE_HI_DEALLOC(T)                        \
    DEFINE_HI_FLUSH(T)                          \
    DEFINE_HI_ADD(T)                            \
    DEFINE_HI_REMOVE(T)                         \
    DEFINE_HI_ADOPT_NODE_EXCLUSIVE(T)           \
    DEFINE_HI_ADOPT(T)                          \
    DEFINE_HI_APPLY_CUSTOM_FREE_AND_REMOVE(T)  


/* Alloc the hierarchy */
#define DECLARE_HI_ALLOC(T)                     \
T ## HI     T ## HIAlloc(struct arena *mem, const u32 initial_size, const u32 growable)

/* Dealloc the hierarchy */
#define DECLARE_HI_DEALLOC(T)                   \
void        T ## HIDealloc(T ## HI *hi)

/* Flush the hierarchy */
#define DECLARE_HI_FLUSH(T)                     \
void        T ## HIFlush(T ## HI *hi)

/*
 * Allocate a hierarchy node as parent_index's last child and return its slot; a full fixed pool
 * returns { .index = HI_NULL (U32_MAX), .address = NULL }.
 */
#define DECLARE_HI_ADD(T)                       \
struct slot T ## HIAdd(T ## HI *hi, const u32 parent_index)

/* Deallocate a hierarchy node and its whole sub-hierarchy */
#define DECLARE_HI_REMOVE(T)                    \
void        T ## HIRemove(struct arena *tmp, T ## HI *hi, const u32 index)

/*
 * node's children (and their subtrees) are adopted by node's parent, in node's place, and node
 * becomes new_parent_index's last child (any node but itself).
 */
#define DECLARE_HI_ADOPT_NODE_EXCLUSIVE(T)      \
void        T ## HIAdoptNodeExclusive(T ## HI *hi, const u32 index, const u32 new_parent_index)

/* node's subtree becomes new_parent_index's last child; new_parent_index must not be in that subtree (debug assert) */
#define DECLARE_HI_ADOPT(T)                     \
void        T ## HIAdoptNode(T ## HI *hi, const u32 index, const u32 new_parent_index)

/*
 * apply a custom free to and deallocate a hierarchy node and its whole sub-hierarchy;
 * the custom free takes in the index to remove. It runs on the sub-hierarchy first (depth-first,
 * while it is being taken apart), on index last: it must not walk the hierarchy.
 */
#define DECLARE_HI_APPLY_CUSTOM_FREE_AND_REMOVE(T)  \
void        T ## HIApplyCustomFreeAndRemove(struct arena *tmp, T ## HI *hi, const u32 index, void (*custom_free)(const T ## HI *hi, const u32, void *data), void *data)


#define DEFINE_HI_ALLOC(T)                                          \
DECLARE_HI_ALLOC(T)                                                 \
{                                                                   \
    const u64 size = initial_size + 2;                              \
	T ## HI hi = { 0 };                                             \
	ds_PoolAlloc(mem, hi.pool, size, growable);                  \
	if (hi.pool.buf)                                                \
	{                                                               \
		const u32 root_index = ds_PoolAdd(hi.pool).index;        \
		const u32 orphan_index = ds_PoolAdd(hi.pool).index;      \
                                                                    \
        T *root = hi.pool.buf + root_index;                         \
        T *orphan = hi.pool.buf + orphan_index;                     \
                                                                    \
        root->hi_parent = HI_NULL;                                  \
        root->hi_prev = HI_NULL;                                    \
        root->hi_next = HI_NULL;                                    \
        root->hi_first = HI_NULL;                                   \
        root->hi_last = HI_NULL;                                    \
        root->hi_child_count = 0;                                   \
                                                                    \
        orphan->hi_parent = HI_NULL;                                \
        orphan->hi_prev = HI_NULL;                                  \
        orphan->hi_next = HI_NULL;                                  \
        orphan->hi_first = HI_NULL;                                 \
        orphan->hi_last = HI_NULL;                                  \
        orphan->hi_child_count = 0;                                 \
                                                                    \
		ds_Assert(root_index == HI_ROOT);                           \
		ds_Assert(orphan_index == HI_ORPHAN);                       \
	}                                                               \
                                                                    \
	return hi;                                                      \
}

#define DEFINE_HI_DEALLOC(T)                                        \
DECLARE_HI_DEALLOC(T)                                               \
{                                                                   \
    ds_PoolDealloc(hi->pool);                                    \
}

#define DEFINE_HI_FLUSH(T)                                          \
DECLARE_HI_FLUSH(T)                                                 \
{                                                                   \
	ds_PoolFlush(hi->pool);                                      \
	const u32 root_index = ds_PoolAdd(hi->pool).index;           \
	const u32 orphan_index = ds_PoolAdd(hi->pool).index;         \
                                                                    \
    T *root = hi->pool.buf + root_index;                            \
    T *orphan = hi->pool.buf + orphan_index;                        \
                                                                    \
    root->hi_parent = HI_NULL;                                      \
    root->hi_prev = HI_NULL;                                        \
    root->hi_next = HI_NULL;                                        \
    root->hi_first = HI_NULL;                                       \
    root->hi_last = HI_NULL;                                        \
    root->hi_child_count = 0;                                       \
                                                                    \
    orphan->hi_parent = HI_NULL;                                    \
    orphan->hi_prev = HI_NULL;                                      \
    orphan->hi_next = HI_NULL;                                      \
    orphan->hi_first = HI_NULL;                                     \
    orphan->hi_last = HI_NULL;                                      \
    orphan->hi_child_count = 0;                                     \
                                                                    \
	ds_Assert(root_index == HI_ROOT);                               \
	ds_Assert(orphan_index == HI_ORPHAN);                           \
}

#define DEFINE_HI_ADD(T)                                            \
DECLARE_HI_ADD(T)                                                   \
{                                                                   \
	ds_Assert(parent_index < hi->pool.count_max);                   \
                                                                    \
	struct slot slot = ds_PoolAdd(hi->pool);                     \
	if (slot.index == U32_MAX)                                      \
	{                                                               \
		return (struct slot) { .index = HI_NULL, .address = NULL }; \
	}                                                               \
                                                                    \
    T *parent = hi->pool.buf + parent_index;                        \
    T *last = hi->pool.buf + parent->hi_last;                       \
    T *node = hi->pool.buf + slot.index;                            \
                                                                    \
    last->hi_next = (i32) slot.index;                               \
                                                                    \
    node->hi_parent = (i32) parent_index;                           \
    node->hi_next = HI_NULL;                                        \
    node->hi_prev = parent->hi_last;                                \
    node->hi_first = HI_NULL;                                       \
    node->hi_last = HI_NULL;                                        \
    node->hi_child_count = 0;                                       \
                                                                    \
    parent->hi_child_count += 1;                                    \
	parent->hi_last = slot.index;                                   \
    if (parent->hi_first == HI_NULL)                                \
    {                                                               \
        parent->hi_first = (i32) slot.index;                        \
	}                                                               \
                                                                    \
	return slot;                                                    \
}

#define DEFINE_HI_REMOVE(T)                                                         \
DECLARE_HI_REMOVE(T)                                                                \
{                                                                                   \
    ds_Assert(1 < index && index < hi->pool.count_max);                             \
                                                                                    \
    const i32 parent = hi->pool.buf[index].hi_parent;                               \
    const i32 first = hi->pool.buf[index].hi_first;                                 \
    const i32 next = hi->pool.buf[index].hi_next;                                   \
    const i32 prev = hi->pool.buf[index].hi_prev;                                   \
                                                                                    \
    /* remove any nodes it the node's sub-hierarchy */                              \
	if (first != HI_NULL)                                                           \
	{                                                                               \
        ArenaPushRecord(tmp);                                                       \
		struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), sizeof(u32));   \
		u32 *stack = arr.addr;                                                      \
		if (!stack)                                                                 \
		{                                                                           \
		    LogString(T_SYSTEM, S_FATAL, "Stack OOM in HIRemove");                  \
			FatalCleanupAndExit();                                                  \
        }                                                                           \
                                                                                    \
		u32 sc = 1;                                                                 \
		stack[0] = first;                                                           \
		while (sc--)                                                                \
		{                                                                           \
			const i32 sub_index = stack[sc];                                        \
			const i32 sub_first = hi->pool.buf[sub_index].hi_first;                 \
			const i32 sub_next = hi->pool.buf[sub_index].hi_next;                   \
			if (sub_first != HI_NULL)                                               \
			{                                                                       \
				stack[sc++] = (u32) sub_first;                                      \
			}                                                                       \
                                                                                    \
			if (sub_next != HI_NULL)                                                \
			{                                                                       \
				if (sc == arr.len)                                                  \
				{                                                                   \
					LogString(T_SYSTEM, S_FATAL, "Stack OOM in HiRemove");          \
					FatalCleanupAndExit();                                          \
				}                                                                   \
				stack[sc++] = (u32) sub_next;                                       \
			}                                                                       \
                                                                                    \
			ds_PoolRemove(hi->pool, sub_index);                                  \
		}                                                                           \
	    ArenaPopRecord(tmp);                                                        \
	}                                                                               \
                                                                                    \
    hi->pool.buf[parent].hi_child_count -= 1;                                       \
    if (hi->pool.buf[parent].hi_first == (i32) index)                               \
    {                                                                               \
        hi->pool.buf[parent].hi_first = next;                                       \
    }                                                                               \
                                                                                    \
    if (hi->pool.buf[parent].hi_last == (i32) index)                                \
    {                                                                               \
        hi->pool.buf[parent].hi_last = prev;                                        \
    }                                                                               \
                                                                                    \
    hi->pool.buf[next].hi_prev = prev;                                              \
    hi->pool.buf[prev].hi_next = next;                                              \
                                                                                    \
    ds_PoolRemove(hi->pool, index);                                              \
}

#define DEFINE_HI_APPLY_CUSTOM_FREE_AND_REMOVE(T)                                   \
DECLARE_HI_APPLY_CUSTOM_FREE_AND_REMOVE(T)                                          \
{                                                                                   \
    ds_Assert(1 < index && index < hi->pool.count_max);                             \
                                                                                    \
    const i32 parent = hi->pool.buf[index].hi_parent;                               \
    const i32 first = hi->pool.buf[index].hi_first;                                 \
    const i32 next = hi->pool.buf[index].hi_next;                                   \
    const i32 prev = hi->pool.buf[index].hi_prev;                                   \
                                                                                    \
    /* remove any nodes it the node's sub-hierarchy */                              \
	if (first != HI_NULL)                                                           \
	{                                                                               \
        ArenaPushRecord(tmp);                                                       \
		struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), sizeof(u32));   \
		u32 *stack = arr.addr;                                                      \
		if (!stack)                                                                 \
		{                                                                           \
		    LogString(T_SYSTEM, S_FATAL, "Stack OOM in HIRemove");                  \
			FatalCleanupAndExit();                                                  \
        }                                                                           \
                                                                                    \
		u32 sc = 1;                                                                 \
		stack[0] = first;                                                           \
		while (sc--)                                                                \
		{                                                                           \
			const i32 sub_index = stack[sc];                                        \
			const i32 sub_first = hi->pool.buf[sub_index].hi_first;                 \
			const i32 sub_next = hi->pool.buf[sub_index].hi_next;                   \
			if (sub_first != HI_NULL)                                               \
			{                                                                       \
				stack[sc++] = (u32) sub_first;                                      \
			}                                                                       \
                                                                                    \
			if (sub_next != HI_NULL)                                                \
			{                                                                       \
				if (sc == arr.len)                                                  \
				{                                                                   \
					LogString(T_SYSTEM, S_FATAL, "Stack OOM in HiRemove");          \
					FatalCleanupAndExit();                                          \
				}                                                                   \
				stack[sc++] = (u32) sub_next;                                       \
			}                                                                       \
                                                                                    \
		    custom_free(hi, (u32) sub_index, data);                                 \
			ds_PoolRemove(hi->pool, sub_index);                                  \
		}                                                                           \
	    ArenaPopRecord(tmp);                                                        \
	}                                                                               \
                                                                                    \
    hi->pool.buf[parent].hi_child_count -= 1;                                       \
    if (hi->pool.buf[parent].hi_first == (i32) index)                               \
    {                                                                               \
        hi->pool.buf[parent].hi_first = next;                                       \
    }                                                                               \
                                                                                    \
    if (hi->pool.buf[parent].hi_last == (i32) index)                                \
    {                                                                               \
        hi->pool.buf[parent].hi_last = prev;                                        \
    }                                                                               \
                                                                                    \
    hi->pool.buf[next].hi_prev = prev;                                              \
    hi->pool.buf[prev].hi_next = next;                                              \
                                                                                    \
	custom_free(hi, (u32) index, data);                                             \
    ds_PoolRemove(hi->pool, index);                                              \
}

#define DEFINE_HI_ADOPT_NODE_EXCLUSIVE(T)                                                               \
DECLARE_HI_ADOPT_NODE_EXCLUSIVE(T)                                                                      \
{                                                                                                       \
	ds_Assert(1 < index && index < hi->pool.count_max && new_parent_index < hi->pool.count_max);        \
	ds_Assert(new_parent_index != index);                                                               \
                                                                                                        \
    T *node = hi->pool.buf + index;                                                                     \
    T *prev = hi->pool.buf + node->hi_prev;                                                             \
    T *next = hi->pool.buf + node->hi_next;                                                             \
    T *first = hi->pool.buf + node->hi_first;                                                           \
    T *last = hi->pool.buf + node->hi_last;                                                             \
    T *parent = hi->pool.buf + node->hi_parent;                                                         \
                                                                                                        \
    parent->hi_child_count += node->hi_child_count - 1;                                                 \
                                                                                                        \
    if (node->hi_child_count)                                                                           \
    {                                                                                                   \
        if (parent->hi_first == (i32) index)                                                            \
        {                                                                                               \
            parent->hi_first = node->hi_first;                                                          \
        }                                                                                               \
                                                                                                        \
        if (parent->hi_last == (i32) index)                                                             \
        {                                                                                               \
            parent->hi_last = node->hi_last;                                                            \
        }                                                                                               \
        prev->hi_next = node->hi_first;                                                                 \
        first->hi_prev = node->hi_prev;                                                                 \
                                                                                                        \
        next->hi_prev = node->hi_last;                                                                  \
        last->hi_next = node->hi_next;                                                                  \
                                                                                                        \
        /* last->hi_next already continues into node's old siblings: stop at last */                  \
        for (T *child = first;; child = hi->pool.buf + child->hi_next)                                  \
        {                                                                                               \
            child->hi_parent = node->hi_parent;                                                         \
            if (child == last)                                                                          \
            {                                                                                           \
                break;                                                                                  \
            }                                                                                           \
        }                                                                                               \
    }                                                                                                   \
    else                                                                                                \
    {                                                                                                   \
        if (parent->hi_first == (i32) index)                                                            \
        {                                                                                               \
            parent->hi_first = node->hi_next;                                                           \
        }                                                                                               \
                                                                                                        \
        if (parent->hi_last == (i32) index)                                                             \
        {                                                                                               \
            parent->hi_last = node->hi_prev;                                                            \
        }                                                                                               \
        prev->hi_next = node->hi_next;                                                                  \
        next->hi_prev = node->hi_prev;                                                                  \
    }                                                                                                   \
                                                                                                        \
	                                                                                                    \
    parent = hi->pool.buf + new_parent_index;                                                           \
    last = hi->pool.buf + parent->hi_last;                                                              \
                                                                                                        \
    last->hi_next = (i32) index;                                                                        \
    node->hi_parent = new_parent_index;                                                                 \
    node->hi_prev = parent->hi_last;                                                                    \
    node->hi_next = HI_NULL;                                                                            \
    node->hi_first = HI_NULL;                                                                           \
    node->hi_last = HI_NULL;                                                                            \
	node->hi_child_count = 0;                                                                           \
                                                                                                        \
	parent->hi_child_count += 1;                                                                        \
    parent->hi_last = (i32) index;                                                                      \
    if (parent->hi_first == HI_NULL)                                                                    \
    {                                                                                                   \
        parent->hi_first = (i32) index;                                                                 \
    }                                                                                                   \
}

#define DEFINE_HI_ADOPT(T)                                                                              \
DECLARE_HI_ADOPT(T)                                                                                     \
{                                                                                                       \
    ds_Assert(1 < index && index < hi->pool.count_max && new_parent_index < hi->pool.count_max);        \
    HI_ASSERT_NOT_IN_SUBTREE(hi, index, new_parent_index)                                               \
                                                                                                        \
    T *node = hi->pool.buf + index;                                                                     \
    T *prev = hi->pool.buf + node->hi_prev;                                                             \
    T *next = hi->pool.buf + node->hi_next;                                                             \
    T *parent = hi->pool.buf + node->hi_parent;                                                         \
                                                                                                        \
    next->hi_prev = node->hi_prev;                                                                      \
    prev->hi_next = node->hi_next;                                                                      \
                                                                                                        \
    parent->hi_child_count -= 1;                                                                        \
    if (parent->hi_first == (i32) index)                                                                \
    {                                                                                                   \
        parent->hi_first = node->hi_next;                                                               \
    }                                                                                                   \
                                                                                                        \
    if (parent->hi_last == (i32) index)                                                                 \
    {                                                                                                   \
        parent->hi_last = node->hi_prev;                                                                \
    }                                                                                                   \
                                                                                                        \
    parent = hi->pool.buf + new_parent_index;                                                           \
    T *last = hi->pool.buf + parent->hi_last;                                                           \
    last->hi_next = (i32) index;                                                                        \
    node->hi_prev = parent->hi_last;                                                                    \
    node->hi_next = HI_NULL;                                                                            \
    node->hi_parent = new_parent_index;                                                                 \
    parent->hi_last = (i32) index;                                                                      \
    if (parent->hi_first == HI_NULL)                                                                    \
    {                                                                                                   \
        parent->hi_first = (i32) index;                                                                 \
    }                                                                                                   \
    parent->hi_child_count += 1;                                                                        \
}               

/*
 * Hierarchy index iterator: iterator for traversing a supplied node and it's entire sub-hierarchy in
 * depth-first order (pre-order, root first), never its siblings. The iteration is finished when
 * at == root again; another HIIAdvance would start over.
 *
 *     HIIInit(it, hi, root);
 *     do { visit(it.at); HIIAdvance(it, hi); } while (it.at != root);
 *
 * To skip (or remove) the sub-hierarchy of it.at: HIISkip, then remove it, then HIIAdvance.
 */

typedef struct HII
{
    i32 root;
    i32 at;
    i32 next;
} HII;

/* Setup hierarchy iterator at the given node root */
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

/* 
 * Advance the iterator in depth-first ordering. 
 */
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

/* 
 * Set it.next to the next depth-first index outside of subtree of it.at. If the iterator is at the root,
 * it.next is set to root indicating that the iterator is finished.
 */
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

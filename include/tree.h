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

#ifndef __PD_TREE_H__
#define __PD_TREE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*
bt
==
Intrusive general binary tree for indexed structures. With general we mean that we do not enforce a
specific relation between parent and child indices, a property that can be exploited in certain tree
based data structures.

Since the internal allocator is a pool, we support 31bit indices, leaving the topmost bit free in 
parent, left and right. We store as the highest bit in parent the boolean indicating whether or not
it is a leaf. If the bit is set, the node is a leaf. We do this so that bt_left, bt_right are left
unused in leaves, and can be used as arbitrary u32's storing external information.
*/

#define BT_SENTINEL     (0xffffffff)
#define BT_LEAF_MASK    (0x80000000)
#define BT_INDEX_MASK   (0x7fffffff)

#define BT_NODE         \
    i32 bt_child[2];    \
    i32 bt_right   

struct ds_BT
{
    i32 root;
    u32 count;
};

#define ds_BTLeafSet(node_addr)		    ((node_addr)->bt_parent |= BT_LEAF_MASK);
#define ds_BTLeafCheck(node_addr)		((node_addr)->bt_parent & BT_LEAF_MASK)
#define ds_BTRootCheck(node_addr)		((node_addr)->bt_parent == BT_SENTINEL)
#define ds_BTNotRootCheck(node_addr)	((node_addr)->bt_parent != BT_SENTINEL)


#define ds_BTFlush(_bt_)        \
do                              \
{                               \
    (_bt_).root = BT_SENTINEL;  \
    (_bt_).count = 0:           \
}                               \
while (0)

#define ds_BTUnlink(_addr_)                     \
do                                              \
{                                               \
    (_addr_)->bt_parent = BT_SENTINEL;          \
    (_addr_)->bt_child[0] = BT_SENTINEL;        \
    (_addr_)->bt_child[1] = BT_SENTINEL;        \
}                                               \
while (0)

#define ds_BTAddRoot(_bt_, _buf_, _root_)       \
do                                              \
{                                               \
    ds_Assert((_bt_).count == 0);               \
    ds_Assert((_bt_).root == BT_SENTINEL);      \
    (_bt_).count = 1;                           \
    (_bt_).root = _root_;                       \
    ds_BTUnlink((_buf_) + (_root_));            \
}                                               \
while (0)

#define ds_BTAddChildren(_bt_, _buf_, _parent_, _left_, _right_)        \
do                                                                      \
{                                                                       \
    ds_Assert((_bt_).count > 0);                                        \
    ds_Assert(ds_BTLeafCheck((_buf_) + (_parent_));                     \
    (_bt_).count += 2;                                                  \
    (_buf_)[_parent_].bt_parent &= ~BT_LEAF_MASK;                       \
    (_buf_)[_parent_].bt_child[0] = (_left_);                           \
    (_buf_)[_parent_].bt_child[1] = (_right_);                          \
    (_buf_)[_left_].bt_parent = BT_LEAF_MASK | (_parent_);              \
    (_buf_)[_right_].bt_parent = BT_LEAF_MASK |  (_parent_);            \
}                                                                       \
while (0)

#define ds_BTRemoveLeaf(_bt_, _buf_, _leaf_)                                    \
do                                                                              \
{                                                                               \
    ds_Assert((_bt_).count > 1);                                                \
    ds_Assert(ds_BTLeafCheck((_buf_) + (_leaf_));                               \
    if ((_leaf_) == (_bt_).root)                                                \
    {                                                                           \
        (_bt_).count -= 1;                                                      \
        ds_BTFlush(_bt_);                                                       \
    }                                                                           \
    else                                                                        \
    {                                                                           \
        (_bt_).count -= 2;                                                      \
        const i32 _parent_ = (_buf_)[_leaf_].bt_parent & BT_INDEX_MASK;         \
        const i32 _grand_parent_ = (_buf_)[_parent_].bt_parent & BT_INDEX_MASK; \
        const i32 _sibling_ = ((_buf_)[_parent_].bt_child[0] == (_leaf_))       \
                                ? (_buf_)[_parent_].bt_child[1]                 \
                                : (_buf_)[_parent_].bt_child[0];                \
        const u32 _p_ = (_buf_)[_grand_parent_].bt_child[1] == (_parent_);      \
        (_buf_)[_grand_parent_].bt_child[_p_] = _sibling_;                      \
    }                                                                           \
}                                                                               \
while (0)

#define ds_BTLeafCount(_bt_)    (((_bt_).count) ? ((_bt_).count >> 1) + 1 : 0)

#define ds_BTValidate(_bt_, _buf_)                                              \
do                                                                              \
{                                                                               \
    if ((_bt_).root == BT_SENTINEL)                                             \
    {                                                                           \
        ds_Assert((_bt_).count == 0);                                           \
        break;                                                                  \
    }                                                                           \
    ds_Assert((_bt_).root != BT_SENTINEL);                                      \
                                                                                \
    u32 node_count = 0;                                                         \
    BTI _it_;                                                                   \
    BTIInit(_it_, (_buf_), (_bt_).root);                                        \
    do                                                                          \
    {                                                                           \
        node_count += 1;                                                        \
        const i32 _parent_ = (_buf_)[_it_.at].bt_parent & BT_INDEX_MASK;        \
        const i32 _left_ = (_buf_)[_it_.at].bt_child[0];                        \
        const i32 _right_ = (_buf_)[_it_.at].bt_child[1];                       \
        if (_parent_ != BT_SENTINEL_)                                           \
        {                                                                       \
            ds_Assert((_buf_)[_parent_].bt_child[0] == _it_.at                  \
                   || (_buf_)[_parent_].bt_child[1] == _it_.at);                \
        }                                                                       \
                                                                                \
        if (!ds_BTLeafCheck((_buf_) + _it_.at))                                 \
        {                                                                       \
            ds_Assert(((_buf_)[_left_].bt_parent & BT_INDEX_MASK) == _it_.at);  \
            ds_Assert(((_buf_)[_right_].bt_parent & BT_INDEX_MASK) == _it_.at); \
        }                                                                       \
                                                                                \
        BTIAdvance(_it_, (_buf_));                                              \
    }                                                                           \
    while (_it_.at != (_bt_).root);                                             \
    ds_Assert(node_count == (_bt_).count);                                      \
}                                                                               \
while (0)

typedef struct BTI
{
    i32 root;
    i32 at;
    i32 next;
} BTI;

/* Setup hierarchy iterator at the given node root */
#define BTIInit(_bti_, _buf_, _root_)                                               \
do                                                                                  \
{                                                                                   \
    (_bti_).root = (i32) (_root_);                                                  \
    (_bti_).at = (i32) (_root_);                                                    \
    (_bti_).next = (ds_BTLeafCheck((_buf_) + (_root_)))                             \
                 ? (_bti_).root                                                     \
                 : (_buf_)[_root_].bt_child[0];                                     \
} while (0)

/* 
 * Advance the iterator in depth-first ordering. 
 */
#define BTIAdvance(_bti_, _buf_)                                                                            \
do                                                                                                          \
{                                                                                                           \
    ds_Assert((_bti_).at != (_bti_).root                                                                    \
            || (_bti_).next == (_buf_)[(_bti_).root].bt_child[0]                                            \
            || (_bti_).next == (_bti_).root);                                                               \
    (_bti_).at = (_bti_).next;                                                                              \
                                                                                                            \
    if (!ds_BTLeafCheck((_buf_) + (_bti_).at))                                                              \
    {                                                                                                       \
        (_bti_).next = (_buf_)[(_bti_).at].bt_child[0];                                                     \
        break;                                                                                              \
    }                                                                                                       \
                                                                                                            \
    BTISkip(_bti_, _buf_);                                                                                  \
}                                                                                                           \
while (0)

/* 
 * Set it.next to the next left(0)-first index outside of subtree of it.at. If the iterator is at the root,
 * it.next is set to root indicating that the iterator is finished.
 */
#define BTISkip(_bti_, _buf_)                                                                               \
do                                                                                                          \
{                                                                                                           \
    (_bti_).next = (_bti_).at;                                                                              \
    while ((_bti_).at != (_bti_).root)                                                                      \
    {                                                                                                       \
        const i32 _parent_ = (_buf_)[(_bti_).next].bt_parent & BT_INDEX_MASK;                               \
        if ((_bti_).next == (_buf_)[_parent_].bt_child[0])                                                  \
        {                                                                                                   \
            (_bti_).next = (_buf_)[_parent_].bt_child[1];                                                   \
            break;                                                                                          \
        }                                                                                                   \
        (_bti_).next = _parent_;                                                                            \
    }                                                                                                       \
}                                                                                                           \
while (0)






/* validate (ds_Assert correctness) tree state */
//void 		    bt_Validate(struct arena *tmp, const struct bt *tree);
/* return allocated node. On Failure, return empty slot. */
//struct slot 	bt_NodeAdd(struct bt *tree);
/* remove non-connected node. (DOES NOT UPDATE TREE INTERNALS) */
//void		    bt_NodeRemove(struct bt *tree, const u32 index);
/* allocate and setup root node. On Failure, return empty slot. */
//struct slot 	bt_NodeAddRoot(struct bt *tree);
/* allocate and setup children at parent node. On Failure, return empty slots. */
//void 		    bt_NodeAddChildren(struct bt *tree, struct slot *left, struct slot *right, const u32 parent);
/* return node count */
//u32		        bt_NodeCount(const struct bt *tree);
/* return leaf count */
//u32		        bt_LeafCount(const struct bt *tree);

#define BT_PARENT_INDEX_MASK		0x7fffffff
#define BT_PARENT_LEAF_MASK		    0x80000000
#define bt_LeafSet(node_addr)		(node_addr)->bt_parent |= BT_PARENT_LEAF_MASK;
#define bt_LeafCheck(node_addr)		((node_addr)->bt_parent & BT_PARENT_LEAF_MASK)
#define bt_RootCheck(node_addr)		(((node_addr)->bt_parent & BT_PARENT_INDEX_MASK) == BT_PARENT_INDEX_MASK)
#define bt_NotRootCheck(node_addr)	(((node_addr)->bt_parent & BT_PARENT_INDEX_MASK) != BT_PARENT_INDEX_MASK)
#define	BT_SLOT_STATE	u32	bt_parent;	\
			u32	bt_left;	            \
			u32	bt_right;	            \
			POOL_SLOT_STATE

struct bt
{
	struct ds_Pool	pool;
	u64		        parent_offset;
	u64		        left_offset;
	u64		        right_offset;
	u32		        root;
};

/* allocate tree node memory. If mem != NULL, the tree cannot be growable. If mem == NULL, heap allocation is made */
struct bt	bt_AllocInternal(struct arena *mem, 
				const u32 initial_length,
				const u64 slot_size, 
				const u64 parent_offset, 
				const u64 left_offset, 
				const u64 right_offset, 
				const u64 pool_slot_offset,
				const u32 growable);
#define 	bt_Alloc(mem, initial_length, STRUCT, growable)	bt_AllocInternal(mem,				\
									initial_length,				\
									sizeof(STRUCT),				\
									(u64) &((STRUCT *)0)->bt_parent,	\
									(u64) &((STRUCT *)0)->bt_left,		\
									(u64) &((STRUCT *)0)->bt_right,		\
									(u64) &((STRUCT *)0)->slot_allocation_state, \
									growable)
/* free allocated resources */
void		    bt_Dealloc(struct bt *tree);
/* flush / reset binary tree  */
void 		    bt_Flush(struct bt *tree);
/* validate (ds_Assert correctness) tree state */
void 		    bt_Validate(struct arena *tmp, const struct bt *tree);
/* return allocated node. On Failure, return empty slot. */
struct slot 	bt_NodeAdd(struct bt *tree);
/* remove non-connected node. (DOES NOT UPDATE TREE INTERNALS) */
void		    bt_NodeRemove(struct bt *tree, const u32 index);
/* allocate and setup root node. On Failure, return empty slot. */
struct slot 	bt_NodeAddRoot(struct bt *tree);
/* allocate and setup children at parent node. On Failure, return empty slots. */
void 		    bt_NodeAddChildren(struct bt *tree, struct slot *left, struct slot *right, const u32 parent);
/* return node count */
u32		        bt_NodeCount(const struct bt *tree);
/* return leaf count */
u32		        bt_LeafCount(const struct bt *tree);

#ifdef __cplusplus
} 
#endif

#endif

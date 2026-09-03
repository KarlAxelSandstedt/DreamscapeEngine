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
Intrusive general binary tree for indexed structures from ds_***Pool/ds_CPool. With general we mean
that we do not enforce a specific relation between parent and child indices, a property that can be
exploited in certain tree based data structures.

Since the external allocator is a pool, we support 31bit indices, leaving the topmost bit free in 
parent, left and right. We store as the highest bit in parent the boolean indicating whether or not
it is a leaf. If the bit is set, the node is a leaf. We do this so that bt_left, bt_right are left
unused in leaves, and can be used as arbitrary u32's storing external information.
*/

#define BT_INDEX_NULL   BT_INDEX_MASK 
#define BT_LEAF_MASK    0x80000000
#define BT_INDEX_MASK   0x7fffffff

#define BT_NODE         \
    u32 bt_child[2];    \
    u32 bt_parent  

struct ds_BT
{
    u32 root;
    u32 count;
};

#define ds_BTLeafSet(node_addr)		    ((node_addr)->bt_parent |= BT_LEAF_MASK) 
#define ds_BTLeafCheck(node_addr)		((node_addr)->bt_parent & BT_LEAF_MASK)
#define ds_BTRootCheck(node_addr)		(((node_addr)->bt_parent & BT_INDEX_MASK) == BT_INDEX_NULL)

#define ds_BTFlush(_bt_)        \
do                              \
{                               \
    (_bt_).root = BT_INDEX_NULL;\
    (_bt_).count = 0;           \
}                               \
while (0)

#define ds_BTAddRoot(_bt_, _buf_, _root_)                       \
do                                                              \
{                                                               \
    ds_Assert((_bt_).count == 0);                               \
    ds_Assert((_bt_).root == BT_INDEX_NULL);                    \
    (_bt_).count = 1;                                           \
    (_bt_).root = _root_;                                       \
    (_buf_)[_root_].bt_parent = BT_LEAF_MASK | BT_INDEX_NULL;   \
}                                                               \
while (0)

#define ds_BTAddChildren(_bt_, _buf_, _parent_, _left_, _right_)        \
do                                                                      \
{                                                                       \
    ds_Assert((_bt_).count > 0);                                        \
    ds_Assert(ds_BTLeafCheck((_buf_) + (_parent_)));                    \
    (_bt_).count += 2;                                                  \
    (_buf_)[_parent_].bt_parent &= BT_INDEX_MASK;                       \
    (_buf_)[_parent_].bt_child[0] = (_left_);                           \
    (_buf_)[_parent_].bt_child[1] = (_right_);                          \
    (_buf_)[_left_].bt_parent = BT_LEAF_MASK | (_parent_);              \
    (_buf_)[_right_].bt_parent = BT_LEAF_MASK |  (_parent_);            \
}                                                                       \
while (0)

#define ds_BTRemoveLeaf(_bt_, _buf_, _leaf_)                                        \
do                                                                                  \
{                                                                                   \
    ds_Assert((_bt_).count > 1);                                                    \
    ds_Assert(ds_BTLeafCheck((_buf_) + (_leaf_));                                   \
    if ((_leaf_) == (_bt_).root)                                                    \
    {                                                                               \
        (_bt_).count -= 1;                                                          \
        ds_BTFlush(_bt_);                                                           \
    }                                                                               \
    else                                                                            \
    {                                                                               \
        (_bt_).count -= 2;                                                          \
        const u32 _parent_ = (_buf_)[_leaf_].bt_parent & BT_INDEX_MASK;             \
        const u32 _sibling_ = ((_buf_)[_parent_].bt_child[0] == (_leaf_))           \
                                ? (_buf_)[_parent_].bt_child[1]                     \
                                : (_buf_)[_parent_].bt_child[0];                    \
        if (_parent_ == (_bt_).root)                                                \
        {                                                                           \
            (_bt_).root = _sibling_;                                                \
        }                                                                           \
        else                                                                        \
        {                                                                           \
            const u32 _grand_parent_ = (_buf_)[_parent_].bt_parent & BT_INDEX_MASK; \
            const u32 _p_ = (_buf_)[_grand_parent_].bt_child[1] == (_parent_);      \
            (_buf_)[_grand_parent_].bt_child[_p_] = _sibling_;                      \
        }                                                                           \
    }                                                                               \
}                                                                                   \
while (0)

#define ds_BTLeafCount(_bt_)    (((_bt_).count) ? ((_bt_).count >> 1) + 1 : 0)

#define ds_BTValidate(_bt_, _buf_)                                                          \
do                                                                                          \
{                                                                                           \
    if ((_bt_).root == BT_INDEX_NULL)                                                       \
    {                                                                                       \
        ds_AssertString((_bt_).count == 0, "(1)");                                          \
        break;                                                                              \
    }                                                                                       \
                                                                                            \
    u32 node_count = 0;                                                                     \
    BTI _it_;                                                                               \
    BTIInit(_it_, (_buf_), (_bt_).root);                                                    \
    do                                                                                      \
    {                                                                                       \
        node_count += 1;                                                                    \
        const u32 _parent_ = (_buf_)[_it_.at].bt_parent & BT_INDEX_MASK;                    \
        const u32 _left_ = (_buf_)[_it_.at].bt_child[0];                                    \
        const u32 _right_ = (_buf_)[_it_.at].bt_child[1];                                   \
        if (_parent_ != BT_INDEX_MASK)                                                      \
        {                                                                                   \
            ds_AssertString((_buf_)[_parent_].bt_child[0] == _it_.at                        \
                   || (_buf_)[_parent_].bt_child[1] == _it_.at, "(2)");                     \
        }                                                                                   \
                                                                                            \
        if (!ds_BTLeafCheck((_buf_) + _it_.at))                                             \
        {                                                                                   \
            ds_AssertString(((_buf_)[_left_].bt_parent & BT_INDEX_MASK) == _it_.at, "(3)"); \
            ds_AssertString(((_buf_)[_right_].bt_parent & BT_INDEX_MASK) == _it_.at, "(4)");\
        }                                                                                   \
                                                                                            \
        BTIAdvance(_it_, (_buf_));                                                          \
    }                                                                                       \
    while (_it_.at != (_bt_).root);                                                         \
    ds_AssertString(node_count == (_bt_).count, "(5)");                                     \
}                                                                                           \
while (0)

typedef struct BTI
{
    u32 root;
    u32 at;
    u32 next;
} BTI;

/* Setup hierarchy iterator at the given node root */
#define BTIInit(_bti_, _buf_, _root_)                                               \
do                                                                                  \
{                                                                                   \
    (_bti_).root = (_root_);                                                        \
    (_bti_).at = (_root_);                                                          \
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
    if (!ds_BTLeafCheck((_buf_) + (_bti_).next))                                                            \
    {                                                                                                       \
        (_bti_).next = (_buf_)[(_bti_).next].bt_child[0];                                                   \
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
    while ((_bti_).next != (_bti_).root)                                                                    \
    {                                                                                                       \
        const u32 _parent_ = (_buf_)[(_bti_).next].bt_parent & BT_INDEX_MASK;                               \
        if ((_bti_).next == (_buf_)[_parent_].bt_child[0])                                                  \
        {                                                                                                   \
            (_bti_).next = (_buf_)[_parent_].bt_child[1];                                                   \
            break;                                                                                          \
        }                                                                                                   \
        (_bti_).next = _parent_;                                                                            \
    }                                                                                                       \
}                                                                                                           \
while (0)

#ifdef __cplusplus
} 
#endif

#endif

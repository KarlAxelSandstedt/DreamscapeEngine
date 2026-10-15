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
ds_binary_tree.h
================
Intrusive binary tree over an indexed node array (normally a ds_Pool's buf), linked by 31-bit indices;
the BVHs (bvh.c) are built on it. Every internal node has exactly two children. Parent and child indices
need no particular order, which tree-based structures can exploit.

Contents (in this order):
    ds_BT   the tree: add a root, give a leaf two children, remove a leaf (with its parent), validate.
    BTI     iterators over a subtree: depth first (BTDF..., parents first) and left-right (BTLR...,
            children first).
*/

#ifndef __DS_BINARY_TREE_H__
#define __DS_BINARY_TREE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*
ds_BT
-----
    struct node { POOL_NODE; BT_NODE; ... };
    struct ds_BT bt;
    ds_BTFlush(bt);
    ds_BTAddRoot(bt, pool.buf, root);
    ds_BTAddChildren(bt, pool.buf, root, left, right);

bt_parent's top bit (BT_LEAF_MASK) marks a leaf, its other 31 bits are the parent index (BT_INDEX_NULL
at the root). The tree leaves a leaf's bt_child[] alone: callers may store anything there. count is the
number of nodes (odd, or 0). The macros don't check that an index is in the tree. Not thread-safe.

The macros (bt is a struct ds_BT lvalue, buf the node array, node_addr a node pointer; arguments may be
evaluated more than once):

    // Set the leaf bit; 1 if node_addr is a leaf; 1 if node_addr is the root (no parent).
    ds_BTLeafSet(node_addr)
    ds_BTLeafCheck(node_addr)
    ds_BTRootCheck(node_addr)

    // Make bt empty (also the initialization).
    ds_BTFlush(bt)

    // Make root the only node of the empty bt, a leaf.
    ds_BTAddRoot(bt, buf, root)

    // Make the leaf parent internal, with the new leaves left (bt_child[0]) and right (bt_child[1]).
    ds_BTAddChildren(bt, buf, parent, left, right)

    // Remove leaf: the root leaf empties bt; any other leaf goes with its parent, and the sibling (with
    // its subtree) takes the parent's place. The caller frees the removed nodes (leaf and parent).
    ds_BTRemoveLeaf(bt, buf, leaf)

    // Number of leaves: (count + 1) / 2.
    u32 ds_BTLeafCount(bt)

    // Debug check (asserts): every link matches its reverse link and count the reachable nodes.
    ds_BTValidate(bt, buf)
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
    (_bt_).root = (_root_);                                     \
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
    ds_Assert((_bt_).count > 0);                                                    \
    ds_Assert(ds_BTLeafCheck((_buf_) + (_leaf_)));                                  \
    if ((_leaf_) == (_bt_).root)                                                    \
    {                                                                               \
        ds_Assert((_bt_).count == 1);                                               \
        ds_BTFlush(_bt_);                                                           \
    }                                                                               \
    else                                                                            \
    {                                                                               \
        (_bt_).count -= 2;                                                          \
        const u32 _parent_ = (_buf_)[_leaf_].bt_parent & BT_INDEX_MASK;             \
        const u32 _sibling_ = ((_buf_)[_parent_].bt_child[0] == (_leaf_))           \
                                ? (_buf_)[_parent_].bt_child[1]                     \
                                : (_buf_)[_parent_].bt_child[0];                    \
        /* the sibling takes the parent's place; it keeps its leaf bit */           \
        const u32 _grand_parent_ = (_buf_)[_parent_].bt_parent & BT_INDEX_MASK;     \
        (_buf_)[_sibling_].bt_parent = ((_buf_)[_sibling_].bt_parent & BT_LEAF_MASK) | _grand_parent_; \
        if (_parent_ == (_bt_).root)                                                \
        {                                                                           \
            (_bt_).root = _sibling_;                                                \
        }                                                                           \
        else                                                                        \
        {                                                                           \
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
    BTDFInit(_it_, (_buf_), (_bt_).root);                                                   \
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
        BTDFAdvance(_it_, (_buf_));                                                         \
    }                                                                                       \
    while (_it_.at != (_bt_).root);                                                         \
    ds_AssertString(node_count == (_bt_).count, "(5)");                                     \
}                                                                                           \
while (0)

/*
BTI
---
Iterators over the subtree of root (any node). it.at is the current node.

    BTI it;
    BTDFInit(it, buf, root);
    do { ... it.at ...; BTDFAdvance(it, buf); } while (it.at != it.root);

    BTLRInit(it, buf, root);
    while (it.at != it.root_parent) { ... it.at ...; BTLRAdvance(it, buf); }

    // Depth first, parents before children, bt_child[0] first: starts at root; after the last node
    // (it.next == it.root), Advance returns it.at to root.
    BTDFInit(it, buf, root)
    BTDFAdvance(it, buf)

    // Set it.next past the subtree of it.at (to it.root if none is left), so Advance skips that subtree.
    BTDFSkip(it, buf)

    // Left-right (post-order), children before parents, bt_child[0] first: starts at the first leaf; the
    // walk ends when it.at reaches it.root_parent (root's parent; BT_INDEX_NULL for the tree's root).
    BTLRInit(it, buf, root)
    BTLRAdvance(it, buf)
*/

typedef struct BTI
{
    u32 root;
    u32 at;
    u32 next;
    u32 root_parent;
} BTI;

#define BTDFInit(_bti_, _buf_, _root_)                                              \
do                                                                                  \
{                                                                                   \
    (_bti_).root = (_root_);                                                        \
    (_bti_).at = (_root_);                                                          \
    (_bti_).next = (ds_BTLeafCheck((_buf_) + (_root_)))                             \
                 ? (_bti_).root                                                     \
                 : (_buf_)[_root_].bt_child[0];                                     \
} while (0)

#define BTDFAdvance(_bti_, _buf_)                                                                           \
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
    BTDFSkip(_bti_, _buf_);                                                                                 \
}                                                                                                           \
while (0)

#define BTDFSkip(_bti_, _buf_)                                                                              \
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

#define BTLRInit(_bti_, _buf_, _root_)                                              \
do                                                                                  \
{                                                                                   \
    (_bti_).root = (_root_);                                                        \
    (_bti_).at = (_root_);                                                          \
    (_bti_).root_parent = (_buf_)[(_bti_).root].bt_parent & BT_INDEX_MASK;          \
    (_bti_).next = (_bti_).root_parent;                                             \
    while (!ds_BTLeafCheck((_buf_) + (_bti_).at))                                   \
    {                                                                               \
        (_bti_).next = (_buf_)[(_bti_).at].bt_child[1];                             \
        (_bti_).at = (_buf_)[(_bti_).at].bt_child[0];                               \
    }                                                                               \
                                                                                    \
    /* root a leaf (no right subtree to visit): root_parent ends the walk */        \
    if ((_bti_).next == (_bti_).root_parent)                                        \
    {                                                                               \
        break;                                                                      \
    }                                                                               \
                                                                                    \
    while (!ds_BTLeafCheck((_buf_) + (_bti_).next))                                 \
    {                                                                               \
        (_bti_).next = (_buf_)[(_bti_).next].bt_child[0];                           \
    }                                                                               \
} while (0)

#define BTLRAdvance(_bti_, _buf_)                                                                           \
do                                                                                                          \
{                                                                                                           \
    ds_Assert((_bti_).at != (_bti_).root_parent);                                                           \
    (_bti_).at = (_bti_).next;                                                                              \
    if ((_bti_).at != (_bti_).root_parent)                                                                  \
    {                                                                                                       \
        const u32 _parent_ = (_buf_)[(_bti_).at].bt_parent & BT_INDEX_MASK;                                 \
        (_bti_).next = _parent_;                                                                            \
        if (_parent_ != (_bti_).root_parent && (_buf_)[_parent_].bt_child[0] == (_bti_).at)                 \
        {                                                                                                   \
            (_bti_).next = (_buf_)[_parent_].bt_child[1];                                                   \
            while (!ds_BTLeafCheck((_buf_) + (_bti_).next))                                                 \
            {                                                                                               \
                (_bti_).next = (_buf_)[(_bti_).next].bt_child[0];                                           \
            }                                                                                               \
        }                                                                                                   \
    }                                                                                                       \
}                                                                                                           \
while (0)

#ifdef __cplusplus
} 
#endif

#endif

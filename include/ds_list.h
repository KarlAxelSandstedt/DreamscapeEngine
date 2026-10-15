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
ds_list.h
=========
Intrusive singly (LL) and doubly (DLL) linked lists over an indexed node array, normally a ds_Pool's or
ds_CPool's buf: nodes are linked by index (i32), never by address, so the lists survive the array moving.

- base[-1] must be addressable: the macros write the link fields of the sentinel, LL_SENTINEL ==
  DLL_SENTINEL == DS_STUB_INDEX (-1), instead of branching. Pools and CPools have that stub slot.
- A list stores count, first and last (LL_SENTINEL when empty); indices are < 2^31. A node may be in
  several lists at once through different link fields (the ...Ex macros link across fields).
- No checks that a node is in, or not in, a list. Not thread-safe.
*/

#ifndef __DS_LIST_H__
#define __DS_LIST_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

/*
LL
--
    struct node { POOL_NODE; LLNode next; ... };
    struct LL list;
    LLFlush(list);
    LLAppend(list, pool.buf, slot.index, next);
    for (i32 i = list.first; i != LL_SENTINEL; i = pool.buf[i].next) { ... }

The macros (ll is a struct LL lvalue, base the node array, index a node's index, node the LLNode field;
arguments may be evaluated more than once):

    // Make ll empty (also the initialization).
    LLFlush(ll)

    // Append or prepend index (not in ll). Append writes base[last].node, base[LL_SENTINEL] when ll is empty.
    LLAppend(ll, base, index, node)
    LLPrepend(ll, base, index, node)

    // LLAppend for nodes whose link lies in different fields: base[ll.last].last_field = index, then
    // base[index].node = LL_SENTINEL.
    LLAppendEx(ll, base, index, last_field, node)

    // Bytes LLSerialize writes: 12.
    LLSerializeSize(ll)

    // Write ll (LE: u32 count, first, last; the nodes go with their pool). ss must have room.
    LLSerialize(ss, ll)

    // Read ll from ss. Returns 1, or 0 (ll empty, ss unchanged) if fewer than 12 bytes are left. The indices
    // are not checked against the pool.
    u32 LLTryDeserialize(ss, ll)
*/

#define LL_SENTINEL        DS_STUB_INDEX

struct LL
{
	u32 	count;
	i32 	first;
	i32 	last;
};

typedef i32 LLNode;


#define LLFlush(_ll_)                                                                                \
do                                                                                                      \
{                                                                                                       \
    (_ll_).count = 0;                                                                                   \
    (_ll_).first = LL_SENTINEL;                                                                         \
    (_ll_).last = LL_SENTINEL;                                                                          \
} while (0)

#define LLAppend(_ll_, _base_, _index_, _node_)    LLAppendEx(_ll_, _base_, _index_, _node_, _node_)
#define LLAppendEx(_ll_, _base_, _index_, _last_, _node_)                                            \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_ll_).last == LL_SENTINEL || (_base_)[(_ll_).last]._last_ == LL_SENTINEL);               \
    ds_Assert((u32) (_index_) < 0x80000000);                                                            \
                                                                                                        \
    (_ll_).count += 1;                                                                                  \
    (_base_)[(_ll_).last]._last_ = (_index_);                                                           \
    (_base_)[_index_]._node_ = LL_SENTINEL;                                                             \
                                                                                                        \
    (_ll_).last = (_index_);                                                                            \
    if ( (_ll_).first == LL_SENTINEL )                                                                  \
    {                                                                                                   \
        (_ll_).first = (_index_);                                                                       \
    }                                                                                                   \
} while (0)

#define LLPrepend(_ll_, _base_, _index_, _node_)                                                     \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((u32) (_index_) < 0x80000000);                                                            \
                                                                                                        \
    (_ll_).count += 1;                                                                                  \
    (_base_)[_index_]._node_ = (_ll_).first;                                                            \
    (_ll_).first = (_index_);                                                                           \
                                                                                                        \
    if ( (_ll_).last == LL_SENTINEL )                                                                   \
    {                                                                                                   \
        (_ll_).last = (_index_);                                                                        \
    }                                                                                                   \
} while (0)



#define LLSerializeSize(_ll_)    (3*sizeof(u32))

#define LLSerialize(_ss_, _ll_)                                                                         \
do                                                                                                      \
{                                                                                                       \
    ss_WriteU32Le((_ss_), (_ll_).count);                                                                \
    ss_WriteU32Le((_ss_), (u32) (_ll_).first);                                                          \
    ss_WriteU32Le((_ss_), (u32) (_ll_).last);                                                           \
} while (0)

#define LLTryDeserialize(_ss_, _ll_)                                                                    \
    ((ss_BytesLeft(_ss_) < LLSerializeSize(_ll_))                                                       \
        ? ((_ll_).count = 0, (_ll_).first = LL_SENTINEL, (_ll_).last = LL_SENTINEL, 0u)                 \
        : ((_ll_).count = ss_ReadU32Le(_ss_),                                                           \
           (_ll_).first = (i32) ss_ReadU32Le(_ss_),                                                     \
           (_ll_).last = (i32) ss_ReadU32Le(_ss_), 1u))

/*
DLL
---
    struct node { POOL_NODE; struct DLLNode dll; ... };
    struct DLL list;
    DLLFlush(list);
    DLLAppend(list, pool.buf, slot.index, dll);
    DLLRemove(list, pool.buf, slot.index, dll);

The macros (dll is a struct DLL lvalue, base the node array, index a node's index, node the struct DLLNode
field; arguments may be evaluated more than once):

    // Make dll empty (also the initialization).
    DLLFlush(dll)

    // Append or prepend index (not in dll). Writes base[DLL_SENTINEL] when dll is empty.
    DLLAppend(dll, base, index, node)
    DLLPrepend(dll, base, index, node)

    // Remove index (in dll); writes base[DLL_SENTINEL] at either end. The removed node's prev and next are
    // left as they were (stale).
    DLLRemove(dll, base, index, node)

    // For nodes whose links lie in different fields: the neighbour's link is last_field / first_field
    // (Append, Prepend), or prev_field / next_field (Remove: base[prev].prev_field.next and
    // base[next].next_field.prev); index's own link is node.
    DLLAppendEx(dll, base, index, last_field, node)
    DLLPrependEx(dll, base, index, node, first_field)
    DLLRemoveEx(dll, base, index, prev_field, node, next_field)

    // Bytes DLLSerialize writes: 12.
    DLLSerializeSize(dll)

    // Write dll (LE: u32 count, first, last; the nodes go with their pool). ss must have room.
    DLLSerialize(ss, dll)

    // Read dll from ss. Returns 1, or 0 (dll empty, ss unchanged) if fewer than 12 bytes are left. The
    // indices are not checked against the pool.
    u32 DLLTryDeserialize(ss, dll)
*/

#define DLL_SENTINEL        DS_STUB_INDEX

struct DLL
{
	u32 	count;
	i32 	first;
	i32 	last;
};

struct DLLNode
{
    i32 prev;
    i32 next;
};


#define DLLFlush(_dll_)                                                                              \
do                                                                                                      \
{                                                                                                       \
    (_dll_).count = 0;                                                                                  \
    (_dll_).first = DLL_SENTINEL;                                                                       \
    (_dll_).last = DLL_SENTINEL;                                                                        \
} while (0)

#define DLLAppend(_dll_, _base_, _index_, _node_)    DLLAppendEx(_dll_, _base_, _index_, _node_, _node_)
#define DLLAppendEx(_dll_, _base_, _index_, _last_, _node_)                                          \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_dll_).last == DLL_SENTINEL || (_base_)[(_dll_).last]._last_.next == DLL_SENTINEL);      \
    ds_Assert((u32) (_index_) < 0x80000000);                                                            \
                                                                                                        \
    (_dll_).count += 1;                                                                                 \
    (_base_)[(_dll_).last]._last_.next = (_index_);                                                     \
    (_base_)[_index_]._node_.prev = (_dll_).last;                                                       \
    (_base_)[_index_]._node_.next = DLL_SENTINEL;                                                       \
                                                                                                        \
    (_dll_).last = (_index_);                                                                           \
    if ( (_dll_).first == DLL_SENTINEL )                                                                \
    {                                                                                                   \
        (_dll_).first = (_index_);                                                                      \
    }                                                                                                   \
} while (0)

#define DLLPrepend(_dll_, _base_, _index_, _node_)    DLLPrependEx(_dll_, _base_, _index_, _node_, _node_)
#define DLLPrependEx(_dll_, _base_, _index_, _node_, _first_)                                        \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_dll_).first == DLL_SENTINEL || (_base_)[(_dll_).first]._first_.prev == DLL_SENTINEL);   \
    ds_Assert((u32) (_index_) < 0x80000000);                                                            \
                                                                                                        \
    (_dll_).count += 1;                                                                                 \
    (_base_)[(_dll_).first]._first_.prev = (_index_);                                                   \
    (_base_)[_index_]._node_.prev = DLL_SENTINEL;                                                       \
    (_base_)[_index_]._node_.next = (_dll_).first;                                                      \
                                                                                                        \
    (_dll_).first = (_index_);                                                                          \
    if ( (_dll_).last == DLL_SENTINEL )                                                                 \
    {                                                                                                   \
        (_dll_).last = (_index_);                                                                       \
    }                                                                                                   \
} while (0)

#define DLLRemove(_dll_, _base_, _index_, _node_) DLLRemoveEx(_dll_, _base_, _index_, _node_, _node_, _node_) 
#define DLLRemoveEx(_dll_, _base_, _index_, _prev_, _node_, _next_)                                  \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_dll_).count);                                                                           \
    (_dll_).count -= 1;                                                                                 \
    const i32 _p_ = (_base_)[_index_]._node_.prev;                                                      \
    const i32 _n_ = (_base_)[_index_]._node_.next;                                                      \
    (_base_)[_p_]._prev_.next = _n_;                                                                    \
    (_base_)[_n_]._next_.prev = _p_;                                                                    \
                                                                                                        \
    if ( _p_ == DLL_SENTINEL )                                                                          \
    {                                                                                                   \
        (_dll_).first = _n_;                                                                            \
    }                                                                                                   \
                                                                                                        \
    if ( _n_ == DLL_SENTINEL )                                                                          \
    {                                                                                                   \
        (_dll_).last = _p_;                                                                             \
    }                                                                                                   \
} while (0)


#define DLLSerializeSize(_dll_)    (3*sizeof(u32))

#define DLLSerialize(_ss_, _dll_)                                                                       \
do                                                                                                      \
{                                                                                                       \
    ss_WriteU32Le((_ss_), (_dll_).count);                                                               \
    ss_WriteU32Le((_ss_), (u32) (_dll_).first);                                                         \
    ss_WriteU32Le((_ss_), (u32) (_dll_).last);                                                          \
} while (0)

#define DLLTryDeserialize(_ss_, _dll_)                                                                  \
    ((ss_BytesLeft(_ss_) < DLLSerializeSize(_dll_))                                                     \
        ? ((_dll_).count = 0, (_dll_).first = DLL_SENTINEL, (_dll_).last = DLL_SENTINEL, 0u)            \
        : ((_dll_).count = ss_ReadU32Le(_ss_),                                                          \
           (_dll_).first = (i32) ss_ReadU32Le(_ss_),                                                    \
           (_dll_).last = (i32) ss_ReadU32Le(_ss_), 1u))

#ifdef __cplusplus
} 
#endif

#endif

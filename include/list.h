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

#ifndef __DS_LIST_H__
#define __DS_LIST_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*
ds_LL
======
Intrusive linked list for indexed structures meant for ds_Pool/ds_CPool. It expects a sentinel/stub at index -1
and can handle at most I32_MAX elements.

::: Usage :::
    //Initialize / Flush _ll_ to the empty state 
    ds_LLFlush(_ll_, _base_, _index_, _node_)                                                    

    // Append _index_ to _ll_ 
    ds_LLAppend(_ll_, _base_, _index_, _node_)                                                    
  
    // Append _index_ to _dll_ and setup _base_[_dll_.last]._last_ -> _base_[_index_]._node_
    // This macro exists for cases when we wish to use different next variables in the two
    // nodes. 
    ds_LLAppendEx(_ll_, _base_, _index_, _last_, _node_)                                                    
  
    // Prepend _index_ to _dll_
    ds_LLPrepend(_ll_, _base_, _index_, _node_)                                                    
*/

#define LL_SENTINEL        DS_STUB_INDEX

struct ds_LL
{
	u32 	count;
	i32 	first;
	i32 	last;
};

typedef i32 ds_LLNode;

#define ds_LLFlush(_ll_)                                                                                \
do                                                                                                      \
{                                                                                                       \
    (_ll_).count = 0;                                                                                   \
    (_ll_).first = LL_SENTINEL;                                                                         \
    (_ll_).last = LL_SENTINEL;                                                                          \
} while (0)

#define ds_LLAppend(_ll_, _base_, _index_, _node_)    ds_LLAppendEx(_ll_, _base_, _index_, _node_, _node_)
#define ds_LLAppendEx(_ll_, _base_, _index_, _last_, _node_)                                            \
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

#define ds_LLPrepend(_ll_, _base_, _index_, _node_)                                                     \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_index_) < 0x80000000);                                                                  \
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



/*
ds_DLL
======
Intrusive doubly linked list for indexed structures meant for ds_Pool/ds_CPool. It expects a sentinel/stub at index -1
and can handle at most I32_MAX elements.

::: Usage :::
  
    // Append _index_ to _dll_ 
    ds_DLLAppend(_dll_, _base_, _index_, _node_)                                                    
  
    // Append _index_ to _dll_ and setup _base_[_dll_.last]._last_ <-> _base_[_index_]._node_
    // This macro exists for cases when the list's nodes may alias different variables
    ds_DLLAppendEx(_dll_, _base_, _index_, _last_, _node_)                                                    

    // Prepend _index_ to _dll_ and set  _base_[_index_]._node_.prev/next
    ds_DLLPrepend(_dll_, _base_, _index_, _node_)                                                    
  
    // Prepend _index_ to _dll_ and setup _base_[_dll_.first]._first_ <-> _base_[_index_]._node_
    // This macro exists for cases when the list's nodes may alias different variables
    ds_DLLPrependEx(_dll_, _base_, _index_, _node_, _first_)                                        
    
    // Remove _index_ from _dll_ and set  _base_[_index_]._node_.prev to DLL_NOT_IN_LIST
    ds_DLLRemove(_dll_, _base_, _index_, _node_)                                                    
*/

#define DLL_SENTINEL        DS_STUB_INDEX

struct ds_DLL
{
	u32 	count;
	i32 	first;
	i32 	last;
};

struct ds_DLLNode
{
    i32 prev;
    i32 next;
};

#define ds_DLLFlush(_dll_)                                                                              \
do                                                                                                      \
{                                                                                                       \
    (_dll_).count = 0;                                                                                  \
    (_dll_).first = DLL_SENTINEL;                                                                       \
    (_dll_).last = DLL_SENTINEL;                                                                        \
} while (0)

#define ds_DLLAppend(_dll_, _base_, _index_, _node_)    ds_DLLAppendEx(_dll_, _base_, _index_, _node_, _node_)
#define ds_DLLAppendEx(_dll_, _base_, _index_, _last_, _node_)                                          \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_dll_).last == DLL_SENTINEL || (_base_)[(_dll_).last]._last_.next == DLL_SENTINEL);      \
    ds_Assert((_index_) < 0x80000000);                                                                  \
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

#define ds_DLLPrepend(_dll_, _base_, _index_, _node_)    ds_DLLAppendEx(_dll_, _base_, _index_, _node_, _node_)
#define ds_DLLPrependEx(_dll_, _base_, _index_, _node_, _first_)                                        \
do                                                                                                      \
{                                                                                                       \
    ds_Assert((_dll_).first == DLL_SENTINEL || (_base_)[(_dll_).first]._first_.prev == DLL_SENTINEL);   \
    ds_Assert((_index_) < 0x80000000);                                                                  \
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

#define ds_DLLRemove(_dll_, _base_, _index_, _node_) ds_DLLRemoveEx(_dll_, _base_, _index_, _node_, _node_, _node_) 
#define ds_DLLRemoveEx(_dll_, _base_, _index_, _prev_, _node_, _next_)                                  \
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


#ifdef __cplusplus
} 
#endif

#endif

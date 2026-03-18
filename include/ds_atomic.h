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

#ifndef __DREAMSCAPE_ATOMIC_H__
#define __DREAMSCAPE_ATOMIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_define.h"

//#define FORCE_SEQ_CST

#if __DS_COMPILER__ == __DS_GCC__ || __DS_COMPILER__ == __DS_CLANG__ || __DS_COMPILER__ == __DS_EMSCRIPTEN__

	#ifdef FORCE_SEQ_CST
		#define ATOMIC_RELAXED	__ATOMIC_SEQ_CST
		#define ATOMIC_ACQUIRE	__ATOMIC_SEQ_CST
		#define ATOMIC_RELEASE	__ATOMIC_SEQ_CST
        	#define ATOMIC_ACQ_REL  __ATOMIC_SEQ_CST
		#define ATOMIC_SEQ_CST	__ATOMIC_SEQ_CST
	#else
		#define ATOMIC_RELAXED	__ATOMIC_RELAXED
		#define ATOMIC_ACQUIRE	__ATOMIC_ACQUIRE
		#define ATOMIC_RELEASE	__ATOMIC_RELEASE
        	#define ATOMIC_ACQ_REL  __ATOMIC_ACQ_REL
		#define ATOMIC_SEQ_CST	__ATOMIC_SEQ_CST
	#endif
	
	#define AtomicFetchAddRlx32(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchAddAcq32(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_ACQUIRE)
	#define AtomicFetchAddRel32(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchAddSeqCst32(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_SEQ_CST)
	
	#define AtomicFetchAddRlx64(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchAddAcq64(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_ACQUIRE)
	#define AtomicFetchAddRel64(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchAddSeqCst64(fetch_addr, val)	__atomic_fetch_add(fetch_addr, val, ATOMIC_SEQ_CST)
	
	#define AtomicFetchSubRlx32(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchSubAcq32(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_ACQUIRE)
	#define AtomicFetchSubRel32(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchSubSeqCst32(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_SEQ_CST)
	                                                                       
	#define AtomicFetchSubRlx64(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchSubAcq64(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_ACQUIRE)
	#define AtomicFetchSubRel64(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED)
	#define AtomicFetchSubSeqCst64(fetch_addr, val)	__atomic_fetch_sub(fetch_addr, val, ATOMIC_SEQ_CST)
	
	#define AtomicCompareExchangeRlxRlx32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELAXED, ATOMIC_RELAXED)
	#define AtomicCompareExchangeAcqRlx32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_RELAXED)
	#define AtomicCompareExchangeAcqAcq32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeRelRlx32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_RELAXED)
	#define AtomicCompareExchangeRelAcq32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeAcqRelAcq32(dst_addr, cmp_addr, exch_val)	__atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeAcqRelAcq32(dst_addr, cmp_addr, exch_val)	__atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeSeqCst32(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_SEQ_CST)
	
	#define AtomicCompareExchangeRlxRlx64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELAXED, ATOMIC_RELAXED)
	#define AtomicCompareExchangeAcqRlx64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_RELAXED)
	#define AtomicCompareExchangeAcqAcq64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeRelRlx64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_RELAXED)
	#define AtomicCompareExchangeRelAcq64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeAcqRelAcq64(dst_addr, cmp_addr, exch_val)	__atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeAcqRelAcq64(dst_addr, cmp_addr, exch_val)	__atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE)
	#define AtomicCompareExchangeSeqCst64(dst_addr, cmp_addr, exch_val)	    __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_SEQ_CST)
	
	#define AtomicStoreRlx32(dst_addr, val)		__atomic_store_n(dst_addr, val, ATOMIC_RELAXED)
	#define AtomicStoreRel32(dst_addr, val)		__atomic_store_n(dst_addr, val, ATOMIC_RELEASE)
	#define AtomicStoreSeqCst32(dst_addr, val)	__atomic_store_n(dst_addr, val, ATOMIC_SEQ_CST)
	               
	#define AtomicStoreRlx64(dst_addr, val)		__atomic_store_n(dst_addr, val, ATOMIC_RELAXED)
	#define AtomicStoreRel64(dst_addr, val)		__atomic_store_n(dst_addr, val, ATOMIC_RELEASE)
	#define AtomicStoreSeqCst64(dst_addr, val)	__atomic_store_n(dst_addr, val, ATOMIC_SEQ_CST)
	
	#define AtomicAddFetchRlx32(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_RELAXED) 
	#define AtomicAddFetchAcq32(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_ACQUIRE) 
	#define AtomicAddFetchRel32(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_RELEASE) 
	#define AtomicAddFetchSeqCst32(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_SEQ_CST)
	      
	#define AtomicAddFetchRlx64(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_RELAXED) 
	#define AtomicAddFetchAcq64(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_ACQUIRE) 
	#define AtomicAddFetchRel64(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_RELEASE) 
	#define AtomicAddFetchSeqCst64(fetch_addr, val)	__atomic_add_fetch(fetch_addr, val, ATOMIC_SEQ_CST)
	     
	#define AtomicSubFetchRlx32(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_RELAXED) 
	#define AtomicSubFetchAcq32(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_ACQUIRE) 
	#define AtomicSubFetchRel32(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_RELEASE) 
	#define AtomicSubFetchSeqCst32(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_SEQ_CST)
	    
	#define AtomicSubFetchRlx64(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_RELAXED) 
	#define AtomicSubFetchAcq64(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_ACQUIRE) 
	#define AtomicSubFetchRel64(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_RELEASE) 
	#define AtomicSubFetchSeqCst64(fetch_addr, val)	__atomic_sub_fetch(fetch_addr, val, ATOMIC_SEQ_CST)
	
	#define AtomicLoadRlx32(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_RELAXED)
	#define AtomicLoadAcq32(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_ACQUIRE)
	#define AtomicLoadSeqCst32(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_SEQ_CST)
	
	#define AtomicLoadRlx64(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_RELAXED)
	#define AtomicLoadAcq64(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_ACQUIRE)
	#define AtomicLoadSeqCst64(fetch_addr)		__atomic_load_n(fetch_addr, ATOMIC_SEQ_CST)
	
	#define AtomicLoadToAddrRlx32(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_RELAXED)
	#define AtomicLoadToAddrAcq32(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_ACQUIRE)
	#define AtomicLoadToAddrSeqCst32(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_SEQ_CST)
	
	#define AtomicLoadToAddrRlx64(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_RELAXED)
	#define AtomicLoadToAddrAcq64(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_ACQUIRE)
	#define AtomicLoadToAddrSeqCst64(fetch_addr, dst_addr)	__atomic_load(fetch_addr, dst_addr, ATOMIC_SEQ_CST)
	
	#define AtomicStoreFromAddrRlx32(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_RELAXED)
	#define AtomicStoreFromAddrRel32(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_ACQUIRE)
	#define AtomicStoreFromAddrSeqCst32(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_SEQ_CST)
	                                                                         
	#define AtomicStoreFromAddrRlx64(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_RELAXED)
	#define AtomicStoreFromAddrRel64(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_ACQUIRE)
	#define AtomicStoreFromAddrSeqCst64(dst_addr, src_addr)	__atomic_store(dst_addr, src_addr, ATOMIC_SEQ_CST)
	
	/********************  Overflow Checking ********************/
	
	#define	U64AddReturnOverflow(dst_addr, src1, src2)		__builtin_add_overflow(src1, src2, dst_addr)
	#define	U64MulReturnOverflow(dst_addr, src1, src2)		__builtin_mul_overflow(src1, src2, dst_addr)
	
	/********************  Bit Manipulation ********************/
	
	/*** Some builtins (REQUIRES: (BMI, Bit Manipulation Instruction Set 1 >= year 2013)) ***/
	#if __DS_COMPILER__ == __DS_GCC__
		#define Clz32(x)	((u32) __builtin_clz(x))  	/* count leading zeroes, NOTE: if x == 0, undefined! */
		#define Clz64(x)	((u32) __builtin_clzl(x))	/* count leading zeroes long, NOTE: if x == 0, undefined! */
		#define Ctz32(x)	((u32) __builtin_ctz(x))  	/* count trailing zeroes, NOTE: if x == 0, undefined! */
		#define Ctz64(x)	((u32) __builtin_ctzl(x))	/* count trailing zeroes long, NOTE: if x == 0, undefined! */
	#else
		#define Clz32(x)	((u32) __builtin_clz(x))  	/* count leading zeroes, NOTE: if x == 0, undefined! */
		#define Clz64(x)	((u32) __builtin_clzll(x))	/* count leading zeroes long, NOTE: if x == 0, undefined! */
		#define Ctz32(x)	((u32) __builtin_ctz(x))  	/* count trailing zeroes, NOTE: if x == 0, undefined! */
		#define Ctz64(x)	((u32) __builtin_ctzll(x))	/* count trailing zeroes long, NOTE: if x == 0, undefined! */
	#endif

#elif __DS_COMPILER__ == __DS_MSVC__

	#include <intrin.h>
	#include <immintrin.h>
	
	/* returns signed integers  */
	#define AtomicFetchAddRlx32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val))
	#define AtomicFetchAddAcq32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val))
	#define AtomicFetchAddRel32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val))
	#define AtomicFetchAddSeqCst32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val))
	
	#define AtomicFetchAddRlx64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val))
	#define AtomicFetchAddAcq64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val))
	#define AtomicFetchAddRel64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val))
	#define AtomicFetchAddSeqCst64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val))
	
	#define AtomicFetchSubRlx32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), -(long) (val))
	#define AtomicFetchSubAcq32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), -(long) (val))
	#define AtomicFetchSubRel32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), -(long) (val))
	#define AtomicFetchSubSeqCst32(fetch_addr, val)	_InterlockedExchangeAdd((long volatile *) (fetch_addr), -(long) (val))
	
	#define AtomicFetchSubRlx64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), -(__int64) (val))
	#define AtomicFetchSubAcq64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), -(__int64) (val))
	#define AtomicFetchSubRel64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), -(__int64) (val))
	#define AtomicFetchSubSeqCst64(fetch_addr, val)	_InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), -(__int64) (val))
	
	__forceinline u32 ds_InterlockedCompareExchange(long volatile* dst_addr, long exch_val, long* cmp_addr)
	{
		const long initial_val = _InterlockedCompareExchange(dst_addr, exch_val, *cmp_addr);
		u32 success = 1;
		if (initial_val!= *cmp_addr)
		{
			success = 0;
			*cmp_addr = initial_val;
		}
		return success;
	}

	__forceinline u32 ds_InterlockedCompareExchange64(__int64 volatile* dst_addr, __int64 exch_val, __int64* cmp_addr)
	{
		const __int64 initial_val = _InterlockedCompareExchange64(dst_addr, exch_val, *cmp_addr);
		u32 success = 1;
		if (initial_val != *cmp_addr)
		{
			success = 0;
			*cmp_addr = initial_val;
		}
		return success;
	}

	/* if (dst == cmp) set dst to exch_val */
	#define AtomicCompareExchangeRlxRlx32(dst_addr, cmp_addr, exch_val)	ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeAcqRlx32(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeAcqAcq32(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeRelRlx32(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeRelAcq32(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeAcqRelAcq32(dst_addr, cmp_addr, exch_val)	ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
#define AtomicCompareExchangeSeqCst32(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr))
	                                                                                                    
	#define AtomicCompareExchangeRlxRlx64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeAcqRlx64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeAcqAcq64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeRelRlx64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeRelAcq64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeAcqRelAcq64(dst_addr, cmp_addr, exch_val)	ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	#define AtomicCompareExchangeSeqCst64(dst_addr, cmp_addr, exch_val)		ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr))
	
	#define AtomicStoreRlx32(dst_addr, val)		_InterlockedExchange((long volatile *) (dst_addr), (long) (val))
	#define AtomicStoreRel32(dst_addr, val)		_InterlockedExchange((long volatile *) (dst_addr), (long) (val))
	#define AtomicStoreSeqCst32(dst_addr, val)	_InterlockedExchange(dst_addr, val)
	               
	#define AtomicStoreRlx64(dst_addr, val)		_InterlockedExchange64((__int64 volatile *) (dst_addr), (__int64) (val))
	#define AtomicStoreRel64(dst_addr, val)		_InterlockedExchange64((__int64 volatile *) (dst_addr), (__int64) (val))
	#define AtomicStoreSeqCst64(dst_addr, val)	_InterlockedExchange64((__int64 volatile *) (dst_addr), (__int64) (val))
	
	#define AtomicAddFetchRlx32(fetch_addr, val)	(AtomicFetchAddRlx32(fetch_addr, val) + ((long) val))
	#define AtomicAddFetchAcq32(fetch_addr, val)	(AtomicFetchAddAcq32(fetch_addr, val) + ((long) val))
	#define AtomicAddFetchRel32(fetch_addr, val)	(AtomicFetchAddRel32(fetch_addr, val) + ((long) val))
	#define AtomicAddFetchSeqCst32(fetch_addr, val)	(AtomicFetchAddSeqCst32(fetch_addr, val) + ((long) val))
	                                                                                                         
	#define AtomicAddFetchRlx64(fetch_addr, val)	(AtomicFetchAddRlx64(fetch_addr, val) + ((__int64) val))
	#define AtomicAddFetchAcq64(fetch_addr, val)	(AtomicFetchAddAcq64(fetch_addr, val) + ((__int64) val))
	#define AtomicAddFetchRel64(fetch_addr, val)	(AtomicFetchAddRel64(fetch_addr, val) + ((__int64) val))
	#define AtomicAddFetchSeqCst64(fetch_addr, val)	(AtomicFetchAddSeqCst64(fetch_addr, val) + ((__int64) val))
	
	#define AtomicSubFetchRlx32(fetch_addr, val)	(AtomicFetchSubRlx32(fetch_addr, val) - ((long) val))
	#define AtomicSubFetchAcq32(fetch_addr, val)	(AtomicFetchSubAcq32(fetch_addr, val) - ((long) val))
	#define AtomicSubFetchRel32(fetch_addr, val)	(AtomicFetchSubRel32(fetch_addr, val) - ((long) val))
	#define AtomicSubFetchSeqCst32(fetch_addr, val)	(AtomicFetchSubSeqCst32(fetch_addr, val) - ((long) val))
	                                                                                                      
	#define AtomicSubFetchRlx64(fetch_addr, val)	(AtomicFetchSubRlx64(fetch_addr, val) - ((__int64) val))
	#define AtomicSubFetchAcq64(fetch_addr, val)	(AtomicFetchSubAcq64(fetch_addr, val) - ((__int64) val))
	#define AtomicSubFetchRel64(fetch_addr, val)	(AtomicFetchSubRel64(fetch_addr, val) - ((__int64) val) )
	#define AtomicSubFetchSeqCst64(fetch_addr, val)	(AtomicFetchSubSeqCst64(fetch_addr, val) - ((__int64) val))
	
	#define AtomicLoadRlx32(fetch_addr)		AtomicAddFetchRlx32(fetch_addr, 0)
	#define AtomicLoadAcq32(fetch_addr)		AtomicAddFetchAcq32(fetch_addr, 0)
	#define AtomicLoadSeqCst32(fetch_addr)		AtomicAddFetchSeqCst32(fetch_addr, 0)
	
	#define AtomicLoadRlx64(fetch_addr)		AtomicAddFetchRlx64(fetch_addr, 0)
	#define AtomicLoadAcq64(fetch_addr)		AtomicAddFetchAcq64(fetch_addr, 0)
	#define AtomicLoadSeqCst64(fetch_addr)		AtomicAddFetchSeqCst64(fetch_addr, 0)
	
	#define AtomicLoadToAddrRlx32(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchRlx32(fetch_addr, 0))
	#define AtomicLoadToAddrAcq32(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchAcq32(fetch_addr, 0))
	#define AtomicLoadToAddrSeqCst32(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchSeqCst32(fetch_addr, 0))
	
	#define AtomicLoadToAddrRlx64(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchRlx64(fetch_addr, 0))
	#define AtomicLoadToAddrAcq64(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchAcq64(fetch_addr, 0))
	#define AtomicLoadToAddrSeqCst64(fetch_addr, dst_addr)	(*(dst_addr) = AtomicAddFetchSeqCst64(fetch_addr, 0))
	
	#define AtomicStoreFromAddrRlx32(dst_addr, src_addr)	AtomicStoreRlx32(dst_addr, *(src_addr))
	#define AtomicStoreFromAddrRel32(dst_addr, src_addr)	AtomicStoreRel32(dst_addr, *(src_addr))
	#define AtomicStoreFromAddrSeqCst32(dst_addr, src_addr)	AtomicStoreSeqCst32(dst_addr, *(src_addr))
	               
	#define AtomicStoreFromAddrRlx64(dst_addr, src_addr)	AtomicStoreRlx64(dst_addr, *(src_addr))
	#define AtomicStoreFromAddrRel64(dst_addr, src_addr)	AtomicStoreRel64(dst_addr, *(src_addr))
	#define AtomicStoreFromAddrSeqCst64(dst_addr, src_addr)	AtomicStoreSeqCst64(dst_addr, *(src_addr))
	
	/******************************************** Overflow Checking ********************************************/
	
	#define U64AddReturnOverflow(dst_addr, src1, src2)	((u64) _addcarry_u64(0, src1, src2, dst_addr))
	
	__forceinline u64 U64MulReturnOverflow(u64 *dst, const u64 src1, const u64 src2)
	{
		u64 high;
		*dst = _umul128(src1, src2, &high);	
		return high;
	}
	
	/******************************************** Bit Manipulation *********************************************/
	
	
	/*** Some builtins (REQUIRES: (BMI, Bit Manipulation Instruction Set 1 >= year 2013)) ***/
	#define Clz32(x)	((u32) _lzcnt_u32(x))	/* count leading zeroes, NOTE: if x == 0, undefined! */
	#define Clz64(x)	((u32) _lzcnt_u64(x))	/* count leading zeroes long, NOTE: if x == 0, undefined! */
	#define Ctz32(x)	((u32) _tzcnt_u32(x))	/* count trailing zeroes, NOTE: if x == 0, undefined! */
	#define Ctz64(x)	((u32) _tzcnt_u64(x))	/* count trailing zeroes long, NOTE: if x == 0, undefined! */

#endif

#ifdef __cplusplus
} 
#endif

#endif

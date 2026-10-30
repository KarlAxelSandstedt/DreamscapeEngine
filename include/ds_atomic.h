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
ds_atomic.h
===========
Atomics, fences and a few bit and overflow intrinsics, mapped onto the GCC/clang builtins or the MSVC
intrinsics.

Contents (in this order):
    Fences      ds_MemoryFence<Ordering>.
    Atomics     Atomic<Op><Ordering>32/64 on u32/u64, pointers through the variant of their size (32 on wasm32).
    Overflow    U64AddReturnOverflow/U64MulReturnOverflow: nonzero on overflow; *dst gets the wrapped result.
    Bits        Clz/Ctz (leading/trailing zeroes), C (1-bits), Cz (0-bits), 32 and 64 bit.

Every atomic exists for each ordering in its name (Rlx, Acq, Rel, AcqRel, SeqCst; a compare-exchange names the
success, then the failure ordering) and for 32 and 64 bits (u32/u64; pointers through the variant of their size,
32 on wasm32). Results are u32/u64 on every compiler. Every operand pointer is checked at compile time to point at
exactly 4 or 8 bytes. MSVC (x64): RMW operations are lock-prefixed full barriers whatever the ordering; loads are
plain movs; Rlx/Rel stores are plain movs and SeqCst stores xchg. FORCE_SEQ_CST (GCC/clang) turns every ordering
into SeqCst, for ruling out ordering bugs. The macros are declared below, one comment per family; their
definitions follow per compiler.
*/

#ifndef __DREAMSCAPE_ATOMIC_H__
#define __DREAMSCAPE_ATOMIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_define.h"

#include <stdatomic.h>
/* a fence of the named ordering (atomic_thread_fence) */
#define ds_MemoryFenceAcquire atomic_thread_fence(memory_order_acquire)
#define ds_MemoryFenceRelease atomic_thread_fence(memory_order_release)
#define ds_MemoryFenceAcqRel  atomic_thread_fence(memory_order_acq_rel)
#define ds_MemoryFenceSeqCst  atomic_thread_fence(memory_order_seq_cst)

/*
Compile-time check that an operand is n bytes (no code): the MSVC intrinsics take any pointer, cast
to a long or __int64 pointer; the GCC builtins pick the size from the pointee, so a 32-bit macro on a u64 would
silently differ between compilers.
*/
#define DS_ATOMIC_SIZE_CHECK(p, n)	((void) sizeof(char[(sizeof(*(p)) == (n)) ? 1 : -1]))

//#define FORCE_SEQ_CST

/************************************ atomics *************************************/

/* return *fetch_addr */
//#define AtomicLoad{Rlx,Acq,SeqCst}{32,64}(fetch_addr)
/* *dst_addr = *fetch_addr, the load atomic */
//#define AtomicLoadToAddr{Rlx,Acq,SeqCst}{32,64}(fetch_addr, dst_addr)
/* *dst_addr = val; returns nothing */
//#define AtomicStore{Rlx,Rel,SeqCst}{32,64}(dst_addr, val)
/* *dst_addr = *src_addr, the store atomic; returns nothing */
//#define AtomicStoreFromAddr{Rlx,Rel,SeqCst}{32,64}(dst_addr, src_addr)
/* *fetch_addr += val (wrapping); return the old value */
//#define AtomicFetchAdd{Rlx,Acq,Rel,SeqCst}{32,64}(fetch_addr, val)
/* *fetch_addr += val (wrapping); return the new value */
//#define AtomicAddFetch{Rlx,Acq,Rel,AcqRel,SeqCst}{32,64}(fetch_addr, val)
/* *fetch_addr -= val (wrapping); return the old value */
//#define AtomicFetchSub{Rlx,Acq,Rel,SeqCst}{32,64}(fetch_addr, val)
/* *fetch_addr -= val (wrapping); return the new value */
//#define AtomicSubFetch{Rlx,Acq,Rel,SeqCst}{32,64}(fetch_addr, val)
/*
 * Strong compare-exchange: if *dst_addr == *cmp_addr, set *dst_addr = exch_val and return 1; else write the current
 * *dst_addr to *cmp_addr and return 0. *cmp_addr is read once, before the exchange.
 */
//#define AtomicCompareExchange{RlxRlx,AcqRlx,AcqAcq,RelRlx,AcqRelAcq,SeqCstRlx,SeqCst}{32,64}(dst_addr, cmp_addr, exch_val)

/************************************ overflow ************************************/

/* *dst_addr = src1 + src2 (wrapped); nonzero on overflow */
//#define U64AddReturnOverflow(dst_addr, src1, src2)
/* *dst_addr = src1 * src2 (wrapped); nonzero on overflow */
//#define U64MulReturnOverflow(dst_addr, src1, src2)

/************************************** bits **************************************/

/* the number of leading zero bits of x (u32); x == 0 is undefined. Compiles to lzcnt (the CPU baseline) */
//#define Clz{32,64}(x)
/* the number of trailing zero bits of x (u32); x == 0 is undefined. Compiles to tzcnt (the CPU baseline) */
//#define Ctz{32,64}(x)
/* the number of 1-bits of x (u32); popcnt */
//#define C{32,64}(x)
/* the number of 0-bits of x (u32) */
//#define Cz{32,64}(x)

/****************************** definitions per compiler ******************************/

#if __DS_COMPILER__ == __DS_GCC__ || __DS_COMPILER__ == __DS_CLANG__

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
	
	#define AtomicFetchAddRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicFetchAddAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_add(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicFetchAddRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_add(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicFetchAddSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_add(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicFetchAddRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_add(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicFetchAddAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_add(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicFetchAddRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_add(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicFetchAddSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_add(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicFetchSubRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicFetchSubAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_sub(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicFetchSubRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_sub(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicFetchSubSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_fetch_sub(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicFetchSubRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_sub(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicFetchSubAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_sub(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicFetchSubRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_sub(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicFetchSubSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_fetch_sub(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicCompareExchangeRlxRlx32(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELAXED, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqRlx32(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqAcq32(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_ACQUIRE))
	#define AtomicCompareExchangeRelRlx32(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqRelAcq32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE))
	#define AtomicCompareExchangeSeqCstRlx32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_RELAXED))
	#define AtomicCompareExchangeSeqCst32(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_SEQ_CST))
	#define AtomicCompareExchangeRlxRlx64(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELAXED, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqRlx64(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqAcq64(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQUIRE, ATOMIC_ACQUIRE))
	#define AtomicCompareExchangeRelRlx64(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_RELEASE, ATOMIC_RELAXED))
	#define AtomicCompareExchangeAcqRelAcq64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_ACQ_REL, ATOMIC_ACQUIRE))
	#define AtomicCompareExchangeSeqCstRlx64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_RELAXED))
	#define AtomicCompareExchangeSeqCst64(dst_addr, cmp_addr, exch_val)	    (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_compare_exchange_n(dst_addr, cmp_addr, exch_val, 0, ATOMIC_SEQ_CST, ATOMIC_SEQ_CST))
	#define AtomicStoreRlx32(dst_addr, val)		(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_store_n(dst_addr, val, ATOMIC_RELAXED))
	#define AtomicStoreRel32(dst_addr, val)		(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_store_n(dst_addr, val, ATOMIC_RELEASE))
	#define AtomicStoreSeqCst32(dst_addr, val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_store_n(dst_addr, val, ATOMIC_SEQ_CST))
	#define AtomicStoreRlx64(dst_addr, val)		(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_store_n(dst_addr, val, ATOMIC_RELAXED))
	#define AtomicStoreRel64(dst_addr, val)		(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_store_n(dst_addr, val, ATOMIC_RELEASE))
	#define AtomicStoreSeqCst64(dst_addr, val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_store_n(dst_addr, val, ATOMIC_SEQ_CST))
	#define AtomicAddFetchRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_add_fetch(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicAddFetchAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_add_fetch(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicAddFetchRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_add_fetch(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicAddFetchAcqRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_add_fetch(fetch_addr, val, ATOMIC_ACQ_REL))
	#define AtomicAddFetchSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_add_fetch(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicAddFetchRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_add_fetch(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicAddFetchAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_add_fetch(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicAddFetchRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_add_fetch(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicAddFetchAcqRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_add_fetch(fetch_addr, val, ATOMIC_ACQ_REL))
	#define AtomicAddFetchSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_add_fetch(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicSubFetchRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_sub_fetch(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicSubFetchAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_sub_fetch(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicSubFetchRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_sub_fetch(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicSubFetchSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_sub_fetch(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicSubFetchRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_sub_fetch(fetch_addr, val, ATOMIC_RELAXED))
	#define AtomicSubFetchAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_sub_fetch(fetch_addr, val, ATOMIC_ACQUIRE))
	#define AtomicSubFetchRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_sub_fetch(fetch_addr, val, ATOMIC_RELEASE))
	#define AtomicSubFetchSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_sub_fetch(fetch_addr, val, ATOMIC_SEQ_CST))
	#define AtomicLoadRlx32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_load_n(fetch_addr, ATOMIC_RELAXED))
	#define AtomicLoadAcq32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_load_n(fetch_addr, ATOMIC_ACQUIRE))
	#define AtomicLoadSeqCst32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), __atomic_load_n(fetch_addr, ATOMIC_SEQ_CST))
	#define AtomicLoadRlx64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_load_n(fetch_addr, ATOMIC_RELAXED))
	#define AtomicLoadAcq64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_load_n(fetch_addr, ATOMIC_ACQUIRE))
	#define AtomicLoadSeqCst64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), __atomic_load_n(fetch_addr, ATOMIC_SEQ_CST))
	#define AtomicLoadToAddrRlx32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_load(fetch_addr, dst_addr, ATOMIC_RELAXED))
	#define AtomicLoadToAddrAcq32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_load(fetch_addr, dst_addr, ATOMIC_ACQUIRE))
	#define AtomicLoadToAddrSeqCst32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __atomic_load(fetch_addr, dst_addr, ATOMIC_SEQ_CST))
	#define AtomicLoadToAddrRlx64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_load(fetch_addr, dst_addr, ATOMIC_RELAXED))
	#define AtomicLoadToAddrAcq64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_load(fetch_addr, dst_addr, ATOMIC_ACQUIRE))
	#define AtomicLoadToAddrSeqCst64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __atomic_load(fetch_addr, dst_addr, ATOMIC_SEQ_CST))
	#define AtomicStoreFromAddrRlx32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), DS_ATOMIC_SIZE_CHECK(src_addr, 4), __atomic_store(dst_addr, src_addr, ATOMIC_RELAXED))
	#define AtomicStoreFromAddrRel32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), DS_ATOMIC_SIZE_CHECK(src_addr, 4), __atomic_store(dst_addr, src_addr, ATOMIC_RELEASE))
	#define AtomicStoreFromAddrSeqCst32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), DS_ATOMIC_SIZE_CHECK(src_addr, 4), __atomic_store(dst_addr, src_addr, ATOMIC_SEQ_CST))
	#define AtomicStoreFromAddrRlx64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), DS_ATOMIC_SIZE_CHECK(src_addr, 8), __atomic_store(dst_addr, src_addr, ATOMIC_RELAXED))
	#define AtomicStoreFromAddrRel64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), DS_ATOMIC_SIZE_CHECK(src_addr, 8), __atomic_store(dst_addr, src_addr, ATOMIC_RELEASE))
	#define AtomicStoreFromAddrSeqCst64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), DS_ATOMIC_SIZE_CHECK(src_addr, 8), __atomic_store(dst_addr, src_addr, ATOMIC_SEQ_CST))

	/********************  Overflow Checking ********************/
	
	#define	U64AddReturnOverflow(dst_addr, src1, src2)		__builtin_add_overflow(src1, src2, dst_addr)
	#define	U64MulReturnOverflow(dst_addr, src1, src2)		__builtin_mul_overflow(src1, src2, dst_addr)
	
	/********************  Bit Manipulation ********************/
	
	/* lzcnt/tzcnt/popcnt through -mlzcnt -mbmi -mpopcnt (CMakeLists.txt); the ll builtins take 64 bits on every target */
	#define Clz32(x)	((u32) __builtin_clz((u32) (x)))
	#define Clz64(x)	((u32) __builtin_clzll((u64) (x)))
	#define Ctz32(x)	((u32) __builtin_ctz((u32) (x)))
	#define Ctz64(x)	((u32) __builtin_ctzll((u64) (x)))
	#define C32(x)		((u32) __builtin_popcount((u32) (x)))
	#define C64(x)		((u32) __builtin_popcountll((u64) (x)))
	#define Cz32(x)		((u32) __builtin_popcount(~(u32) (x)))
	#define Cz64(x)		((u32) __builtin_popcountll(~(u64) (x)))

#elif __DS_COMPILER__ == __DS_MSVC__

	#include <intrin.h>
	#include <immintrin.h>

	/* the load and store mappings below are the x64 ones (ARM64 needs __ldar/__stlr or _acq/_rel intrinsics) */
	#if !defined(_M_X64)
		#error "ds_atomic.h: MSVC atomics are implemented for x64 only"
	#endif

	/* results are u32/u64 (the intrinsics return long/__int64), as the GCC builtins give for u32/u64 operands */
	#define AtomicFetchAddRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicFetchAddAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicFetchAddRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicFetchAddSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (val)))

	#define AtomicFetchAddRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicFetchAddAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicFetchAddRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicFetchAddSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (val)))

	#define AtomicFetchSubRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicFetchSubAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicFetchSubRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicFetchSubSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) _InterlockedExchangeAdd((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))

	#define AtomicFetchSubRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicFetchSubAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicFetchSubRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicFetchSubSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) _InterlockedExchangeAdd64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))

	/* helpers behind the macros below, defined at the bottom */
	__forceinline u32 ds_InterlockedCompareExchange(long volatile* dst_addr, long exch_val, long* cmp_addr);
	__forceinline u32 ds_InterlockedCompareExchange64(__int64 volatile* dst_addr, __int64 exch_val, __int64* cmp_addr);
	__forceinline u32 ds_InterlockedAddFetch(long volatile *fetch_addr, const long val);
	__forceinline u64 ds_InterlockedAddFetch64(__int64 volatile *fetch_addr, const __int64 val);
	__forceinline u32 ds_LoadAcq32(const volatile long *fetch_addr);
	__forceinline u64 ds_LoadAcq64(const volatile __int64 *fetch_addr);
	__forceinline void ds_StoreRel32(volatile long *dst_addr, const long val);
	__forceinline void ds_StoreRel64(volatile __int64 *dst_addr, const __int64 val);

	/* if (dst == cmp) set dst to exch_val */
	#define AtomicCompareExchangeRlxRlx32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeAcqRlx32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeAcqAcq32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeRelRlx32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeAcqRelAcq32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeSeqCstRlx32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))
	#define AtomicCompareExchangeSeqCst32(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_InterlockedCompareExchange((long volatile *) (dst_addr), (long) (exch_val), (long*) (cmp_addr)))

	#define AtomicCompareExchangeRlxRlx64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeAcqRlx64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeAcqAcq64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeRelRlx64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeAcqRelAcq64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeSeqCstRlx64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))
	#define AtomicCompareExchangeSeqCst64(dst_addr, cmp_addr, exch_val)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_InterlockedCompareExchange64((__int64 volatile *) (dst_addr), (__int64) (exch_val), (__int64*) (cmp_addr)))

	/* stores return nothing, as the GCC builtins; only seq_cst needs xchg (the plain seq_cst loads rely on it) */
	#define AtomicStoreRlx32(dst_addr, val)		((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), __iso_volatile_store32((volatile __int32 *) (dst_addr), (__int32) (val))))
	#define AtomicStoreRel32(dst_addr, val)		((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), ds_StoreRel32((volatile long *) (dst_addr), (long) (val))))
	#define AtomicStoreSeqCst32(dst_addr, val)	((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 4), _InterlockedExchange((long volatile *) (dst_addr), (long) (val))))

	#define AtomicStoreRlx64(dst_addr, val)		((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), __iso_volatile_store64((volatile __int64 *) (dst_addr), (__int64) (val))))
	#define AtomicStoreRel64(dst_addr, val)		((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), ds_StoreRel64((volatile __int64 *) (dst_addr), (__int64) (val))))
	#define AtomicStoreSeqCst64(dst_addr, val)	((void) (DS_ATOMIC_SIZE_CHECK(dst_addr, 8), _InterlockedExchange64((__int64 volatile *) (dst_addr), (__int64) (val))))

	#define AtomicAddFetchRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicAddFetchAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicAddFetchRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicAddFetchAcqRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (val)))
	#define AtomicAddFetchSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (val)))

	#define AtomicAddFetchRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicAddFetchAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicAddFetchRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicAddFetchAcqRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (val)))
	#define AtomicAddFetchSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (val)))

	#define AtomicSubFetchRlx32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicSubFetchAcq32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicSubFetchRel32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))
	#define AtomicSubFetchSeqCst32(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_InterlockedAddFetch((long volatile *) (fetch_addr), (long) (0u - (u32) (val))))

	#define AtomicSubFetchRlx64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicSubFetchAcq64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicSubFetchRel64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))
	#define AtomicSubFetchSeqCst64(fetch_addr, val)	(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_InterlockedAddFetch64((__int64 volatile *) (fetch_addr), (__int64) (0ull - (u64) (val))))

	#define AtomicLoadRlx32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), (u32) __iso_volatile_load32((const volatile __int32 *) (fetch_addr)))
	#define AtomicLoadAcq32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_LoadAcq32((const volatile long *) (fetch_addr)))
	#define AtomicLoadSeqCst32(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 4), ds_LoadAcq32((const volatile long *) (fetch_addr)))

	#define AtomicLoadRlx64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), (u64) __iso_volatile_load64((const volatile __int64 *) (fetch_addr)))
	#define AtomicLoadAcq64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_LoadAcq64((const volatile __int64 *) (fetch_addr)))
	#define AtomicLoadSeqCst64(fetch_addr)		(DS_ATOMIC_SIZE_CHECK(fetch_addr, 8), ds_LoadAcq64((const volatile __int64 *) (fetch_addr)))

	#define AtomicLoadToAddrRlx32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), *(dst_addr) = AtomicLoadRlx32(fetch_addr))
	#define AtomicLoadToAddrAcq32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), *(dst_addr) = AtomicLoadAcq32(fetch_addr))
	#define AtomicLoadToAddrSeqCst32(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 4), *(dst_addr) = AtomicLoadSeqCst32(fetch_addr))

	#define AtomicLoadToAddrRlx64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), *(dst_addr) = AtomicLoadRlx64(fetch_addr))
	#define AtomicLoadToAddrAcq64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), *(dst_addr) = AtomicLoadAcq64(fetch_addr))
	#define AtomicLoadToAddrSeqCst64(fetch_addr, dst_addr)	(DS_ATOMIC_SIZE_CHECK(dst_addr, 8), *(dst_addr) = AtomicLoadSeqCst64(fetch_addr))

	#define AtomicStoreFromAddrRlx32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 4), AtomicStoreRlx32(dst_addr, *(src_addr)))
	#define AtomicStoreFromAddrRel32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 4), AtomicStoreRel32(dst_addr, *(src_addr)))
	#define AtomicStoreFromAddrSeqCst32(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 4), AtomicStoreSeqCst32(dst_addr, *(src_addr)))

	#define AtomicStoreFromAddrRlx64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 8), AtomicStoreRlx64(dst_addr, *(src_addr)))
	#define AtomicStoreFromAddrRel64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 8), AtomicStoreRel64(dst_addr, *(src_addr)))
	#define AtomicStoreFromAddrSeqCst64(dst_addr, src_addr)	(DS_ATOMIC_SIZE_CHECK(src_addr, 8), AtomicStoreSeqCst64(dst_addr, *(src_addr)))
	
	/******************************************** Overflow Checking ********************************************/
	
	#define U64AddReturnOverflow(dst_addr, src1, src2)	((u64) _addcarry_u64(0, src1, src2, dst_addr))
	
	__forceinline u64 U64MulReturnOverflow(u64 *dst, const u64 src1, const u64 src2);

	/******************************************** Bit Manipulation *********************************************/
	
	
	#define Clz32(x)	((u32) _lzcnt_u32((u32) (x)))
	#define Clz64(x)	((u32) _lzcnt_u64((u64) (x)))
	#define Ctz32(x)	((u32) _tzcnt_u32((u32) (x)))
	#define Ctz64(x)	((u32) _tzcnt_u64((u64) (x)))
	#define C32(x)		((u32) __popcnt((u32) (x)))
	#define C64(x)		((u32) __popcnt64((u64) (x)))
	#define Cz32(x)		((u32) __popcnt(~(u32) (x)))
	#define Cz64(x)		((u32) __popcnt64(~(u64) (x)))
#endif

/******************************************** MSVC inline definitions ********************************************/

#if __DS_COMPILER__ == __DS_MSVC__

	/*
	 * *cmp_addr is read once, before the exchange (as __atomic_compare_exchange_n does): it may be shared
	 * memory that another thread changes once the exchange succeeded (a Treiber stack push passes the
	 * pushed node's link), so reading it again afterwards could misreport success.
	 */
	__forceinline u32 ds_InterlockedCompareExchange(long volatile* dst_addr, long exch_val, long* cmp_addr)
	{
		const long expected = *cmp_addr;
		const long initial_val = _InterlockedCompareExchange(dst_addr, exch_val, expected);
		if (initial_val == expected)
		{
			return 1;
		}
		*cmp_addr = initial_val;
		return 0;
	}

	__forceinline u32 ds_InterlockedCompareExchange64(__int64 volatile* dst_addr, __int64 exch_val, __int64* cmp_addr)
	{
		const __int64 expected = *cmp_addr;
		const __int64 initial_val = _InterlockedCompareExchange64(dst_addr, exch_val, expected);
		if (initial_val == expected)
		{
			return 1;
		}
		*cmp_addr = initial_val;
		return 0;
	}

	/* functions, not macros: val is evaluated once; unsigned sum: wraps like the GCC builtins */
	__forceinline u32 ds_InterlockedAddFetch(long volatile *fetch_addr, const long val)
	{
		return (u32) _InterlockedExchangeAdd(fetch_addr, val) + (u32) val;
	}

	__forceinline u64 ds_InterlockedAddFetch64(__int64 volatile *fetch_addr, const __int64 val)
	{
		return (u64) _InterlockedExchangeAdd64(fetch_addr, val) + (u64) val;
	}

	/* x64: an aligned load is already an acquire load (and seq_cst, as seq_cst stores are xchg); the barrier keeps the compiler from moving later accesses before it */
	__forceinline u32 ds_LoadAcq32(const volatile long *fetch_addr)
	{
		const u32 val = (u32) __iso_volatile_load32((const volatile __int32 *) fetch_addr);
		_ReadWriteBarrier();
		return val;
	}

	__forceinline u64 ds_LoadAcq64(const volatile __int64 *fetch_addr)
	{
		const u64 val = (u64) __iso_volatile_load64(fetch_addr);
		_ReadWriteBarrier();
		return val;
	}

	/* x64: an aligned store is already a release store; the barrier keeps the compiler from moving earlier accesses after it */
	__forceinline void ds_StoreRel32(volatile long *dst_addr, const long val)
	{
		_ReadWriteBarrier();
		__iso_volatile_store32((volatile __int32 *) dst_addr, val);
	}

	__forceinline void ds_StoreRel64(volatile __int64 *dst_addr, const __int64 val)
	{
		_ReadWriteBarrier();
		__iso_volatile_store64(dst_addr, val);
	}

	__forceinline u64 U64MulReturnOverflow(u64 *dst, const u64 src1, const u64 src2)
	{
		u64 high;
		*dst = _umul128(src1, src2, &high);
		return high;
	}

#endif

#ifdef __cplusplus
} 
#endif

#endif

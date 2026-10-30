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
ds_arch.h
=========
The CPU and system the engine runs on: g_arch_config (set once by ds_ArchConfigInit, early in ds_Init; read
only afterwards), CPUID access and the baseline check (AGENTS.md: SSE2, POPCNT, LZCNT, BMI1).
*/

#ifndef __DREAMSCAPE_ARCH_H__
#define __DREAMSCAPE_ARCH_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_string.h"

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#include <sys/types.h>
typedef pid_t	pid;

#elif __DS_PLATFORM__ == __DS_WIN64__ 

#include <windows.h>
typedef DWORD	pid;

#endif

enum dsArchType
{
	ARCH_INTEL64,
	ARCH_AMD64,
};

struct dsArchConfig
{
	utf8 	vendor_string;		/* CPUID leaf 0: "GenuineIntel", "AuthenticAMD" */
	utf8	processor_string;	/* CPUID 0x80000002-4, e.g. "Intel(R) Core(TM) i7-7700HQ ..." */

	enum dsArchType 	type;
	u32		logical_core_count;	/* online logical cores (Windows: of the calling thread's processor group, <= 64) */
	pid		pid;

	u64		pagesize;	/* bytes */
	u64		cacheline;	/* bytes, L1 data cache line from CPUID (64 if not reported) */

	/* CPUID feature bits (1: supported); avx/avx2 do not check that the OS saves the AVX state */
	u32		sse : 1;
	u32		sse2 : 1;
	u32		sse3 : 1;
	u32		ssse3 : 1;
	u32		sse4_1 : 1;
	u32		sse4_2 : 1;
	u32		avx : 1;
	u32		avx2 : 1;
	u32		bmi1 : 1;	/* bit manipulation instructions: tzcnt (Ctz32/64), andn, blsr, ... */
	u32		popcnt : 1;	/* popcnt (C32/64, Cz32/64) */
	u32		lzcnt : 1;	/* lzcnt (Clz32/64); AMD calls it ABM */

	u32		Rdtsc : 1;	/* profiling timer support  */
	u32		Rdtscp : 1;	/* profiling timer support  */
	u32		tsc_invariant : 1;  /* tsc works as a wallclock timer, always ticking and at same frequency (in every power state) */
};

extern const struct dsArchConfig *g_arch_config;

/* return Logical core count (asks the OS; usable before ds_ArchConfigInit) */
u32  	ds_LogicalCoreCount(void);
/* return system pagesize (asks the OS; usable before ds_ArchConfigInit) */
u64	ds_Pagesize(void);
/* return process identifier */
pid	ds_Pid(void);
/* cpu x86 querying: CPUID leaf function; Linux: all zero for a leaf the CPU lacks; web: all zero */
void 	ds_Cpuid(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function);
/* ds_Cpuid with a subleaf (subfunction) */
void 	ds_CpuidEx(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function, const u32 subfunction);

/* 1 if the CPU meets the baseline (always on the web), else 0. CPUID only: no memory, no log, safe before any init. */
u32	ds_ArchBaselineCheck(void);
/*
 * sets up g_arch_config (strings on mem) and logs it. returns 1 on intrinsics requirements fullfilled, 0 otherwise
 * (an unsupported vendor or a CPU without the baseline; logged, ds_Init then exits).
 */
u32 	ds_ArchConfigInit(struct arena *mem);

#ifdef __cplusplus
} 
#endif

#endif

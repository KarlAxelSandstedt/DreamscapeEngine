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
ds_time.h
=========
Time since ds_TimeApiInit from the OS monotonic clock (consistent across threads), and conversions for the
x86 time-stamp counter (TSC) that profiling and the performance tests use. The TSC parts are x64 only: the
web has no Rdtsc, and its TscFrequency() is 0.
*/

#ifndef __DREAMSCAPE_TIMER_H__
#define __DREAMSCAPE_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
ds_Time
=======
The ds_Time** API returns time elapsed since the API's initialization, from the OS monotonic clock: consistent
across threads. The TSC (Rdtsc) is per core; its readings from different cores differ by g_tsc_skew.
*/

#if (__DS_PLATFORM__ == __DS_WEB__)

#elif (__DS_COMPILER__ == __DS_GCC__ || __DS_COMPILER__ == __DS_CLANG__)

	#include <x86intrin.h>
	/* the calling core's TSC */
	#define	Rdtsc()			__rdtsc()
	/* RDTSC once all earlier instructions have executed, and writes IA32_TSC_AUX (the OS's id of the core) to *core_addr. */
	#define	Rdtscp(core_addr)	__rdtscp(core_addr)

#elif (__DS_COMPILER__ == __DS_MSVC__)

	#include <intrin.h>
	/* the calling core's TSC */
	#define	Rdtsc()			__rdtsc()
	/* RDTSC once all earlier instructions have executed, and writes IA32_TSC_AUX (the OS's id of the core) to *core_addr. */
	#define	Rdtscp(core_addr)	__rdtscp(core_addr)

#endif

/*
 * Initialize API: the OS clock and TSC origins, the TSC frequency (a 100 ms busy wait against the OS clock) and
 * g_tsc_skew (a ping-pong between two threads pinned to core 0 and to each other core in turn; fatal if a thread
 * can't be pinned).
 */
void	ds_TimeApiInit(struct arena *persistent);
/* Return ns timing at the API's initialization (the OS clock's reading) */
u64	ds_TimeNsAtStart(void);				
/* Return seconds since initialization (truncated) */
u64	ds_TimeS(void);					
/* Return ms since initialization (truncated) */
u64	ds_TimeMs(void); 					
/* Return us since initialization (truncated) */
u64	ds_TimeUs(void); 					
/* Return ns since initialization */
u64	ds_TimeNs(void); 					
/* Return the OS clock's resolution in ns (Windows: one QPC tick, rounded up) */
u64	NsResolution(void);				

/* Return ns since initialization given cpu tsc value (>= the TSC at initialization) */
u64 	ds_TimeNsFromTsc(const u64 tsc);		
/* Return tsc value since initialization given PLATFORM ns value (as ds_TimeNsAtStart counts, not ds_TimeNs()) */
u64	ds_TimeTscFromNs(const u64 ns);			
/* transform a tsc tick count to corresponding ns (through TscFrequency, in f64) */
u64 	NsFromTsc(const u64 tsc);			
/* transform ns to corresponding tsc tick count (through TscFrequency, in f64) */
u64	TscFromNs(const u64 ns);			
/* transform tsc to ns, with an additional truth pair (ns, tsc) read at the same moment in order to reduce error: it grows with the distance to the pair */
u64	NsFromTscTruthSource(const u64 tsc, const u64 ns_truth, const u64 cc_truth); 
/* transform ns to tsc, with an additional truth pair (ns, tsc) read at the same moment in order to reduce error: it grows with the distance to the pair */
u64	TscFromNsTruthSource(const u64 ns, const u64 ns_truth, const u64 cc_truth);  
/* Return estimated tsc ticks per second, measured by ds_TimeApiInit (0 on the web) */
u64 	TscFrequency(void);				
/* Return estimated seconds given the tick count */
f64 	SFromTsc(const u64 ticks);			

/* g_tsc_skew[Logical_core_count]: estimated skew from core 0: core c's TSC minus core 0's, as the ping-pong measured it.
 * given a tsc value from core c, its corresponding tsc value on core 0 is t_0 = t_c - skew,
 * so in code we get 
 * 			tsc_c = Rdtscp(&core_c)
 * 			tsc_0 = tsc_c - g_tsc_skew[c];
 * The measurement includes the one-way latency between the cores (tens to hundreds of ticks): with synchronized
 * TSCs (modern CPUs) it is that latency, not skew. A negative value wraps (u64). Only logged at startup.
 */
extern u64 *	g_tsc_skew;

#ifdef __cplusplus
} 
#endif

#endif

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
Rdtsc(): the calling core's TSC. Rdtscp(core_addr): the same, once all earlier instructions have executed,
and writes IA32_TSC_AUX (the OS's id of the core) to *core_addr.
*/
#if (__DS_PLATFORM__ == __DS_WEB__)

#elif (__DS_COMPILER__ == __DS_GCC__ || __DS_COMPILER__ == __DS_CLANG__)

	#include <x86intrin.h>
	#define	Rdtsc()			__rdtsc()
	#define	Rdtscp(core_addr)	__rdtscp(core_addr)

#elif (__DS_COMPILER__ == __DS_MSVC__)

	#include <intrin.h>
	#define	Rdtsc()			__rdtsc()
	#define	Rdtscp(core_addr)	__rdtscp(core_addr)

#endif

/*
Start the clocks: the OS clock and TSC origins, the TSC frequency (a 100 ms busy wait against the OS clock)
and g_tsc_skew (a ping-pong between two threads pinned to core 0 and to each other core in turn; fatal if a
thread can't be pinned).
*/
void	ds_TimeApiInit(struct arena *persistent);
/* The OS clock reading (ns) at ds_TimeApiInit. */
u64	ds_TimeNsAtStart(void);
/* Time since ds_TimeApiInit, truncated to s, ms, us or ns. */
u64	ds_TimeS(void);
u64	ds_TimeMs(void);
u64	ds_TimeUs(void);
u64	ds_TimeNs(void);
/* The OS clock's resolution in ns (Windows: one QPC tick, rounded up). */
u64	NsResolution(void);

/* A TSC reading (>= the TSC at ds_TimeApiInit) as ns since ds_TimeApiInit. */
u64 	ds_TimeNsFromTsc(const u64 tsc);
/* An OS clock reading in ns (as ds_TimeNsAtStart counts, not ds_TimeNs) as TSC ticks since ds_TimeApiInit. */
u64	ds_TimeTscFromNs(const u64 ns);
/* A TSC tick count as ns, and ns as a tick count (through TscFrequency, in f64). */
u64 	NsFromTsc(const u64 tsc);
u64	TscFromNs(const u64 ns);
/*
The same for readings, through a pair (ns_truth, cc_truth) read at the same moment: the error grows with
the distance to that pair instead of the time since ds_TimeApiInit.
*/
u64	NsFromTscTruthSource(const u64 tsc, const u64 ns_truth, const u64 cc_truth);
u64	TscFromNsTruthSource(const u64 ns, const u64 ns_truth, const u64 cc_truth);
/* TSC ticks per second, measured by ds_TimeApiInit (0 on the web). */
u64 	TscFrequency(void);
/* A TSC tick count in seconds. */
f64 	SFromTsc(const u64 ticks);

/*
g_tsc_skew[c] (c < the logical core count): core c's TSC minus core 0's, as the ping-pong measured it, so
tsc_0 ~ tsc_c - g_tsc_skew[c]. The measurement includes the one-way latency between the cores (tens to
hundreds of ticks): with synchronized TSCs (modern CPUs) it is that latency, not skew. A negative value wraps
(u64). Only logged at startup.
*/
extern u64 *	g_tsc_skew;

#ifdef __cplusplus
}
#endif

#endif

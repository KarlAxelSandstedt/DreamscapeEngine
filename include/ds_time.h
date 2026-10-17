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

Sections: ds_Time (clock, TSC conversions, core skew), Perf blocks (PerfWallClock, PerfCycles: timing a code
block); inline definitions at the bottom.
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
	/* waits for all earlier instructions to complete; later ones don't start until it has */
	#define	Lfence()		_mm_lfence()
	/* performance counter index's value (Linux: enabled for user mode by PerfCyclesThreadInit) */
	#define	Rdpmc(index)		__rdpmc(index)

#elif (__DS_COMPILER__ == __DS_MSVC__)

	#include <intrin.h>
	/* the calling core's TSC */
	#define	Rdtsc()			__rdtsc()
	/* RDTSC once all earlier instructions have executed, and writes IA32_TSC_AUX (the OS's id of the core) to *core_addr. */
	#define	Rdtscp(core_addr)	__rdtscp(core_addr)
	/* waits for all earlier instructions to complete; later ones don't start until it has */
	#define	Lfence()		_mm_lfence()
	/* performance counter index's value (needs a driver on Windows: unused) */
	#define	Rdpmc(index)		__readpmc(index)

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

#ifdef DS_PERFORMANCE_MEASUREMENT_API

/*
Perf blocks
===========
Performance measuring blocks of code. 

    const u64 t = PerfWallClockBegin();
    {
        ... load inputs, the block, store results ...
    }
    const u64 block_ticks = PerfWallClockEnd(t);      

    const u64 c = PerfCyclesBegin();
    {
        ... load inputs, the block, store results ...
    }
    const u64 block_cycles = PerfCyclesEnd(c);      

    Note 1: Make fully sure that all operations to be measured are done WITHIN the measure zone: load the
    block's inputs and store its results inside it; the barriers order memory operations, not register work.

    Note 2: The block itself should cost 100 cycles or more: the measurement adds its own cost 
    (g_perf_*_overhead, tens of counts), which varies by a few counts from sample to sample, so
    a smaller block is measured to a few percent at best.

    Note 3: Within a measurement the block cannot overlap its neighbours, so it shows its latency,
    not the throughput it has inside a loop.


PerfWallClock: best available wall-clock timer for performance measurements.

PerfCycles: core cycles in user mode, independent of the core clock, following the thread
across cores. Linux only (perf_event_open: kernel.perf_event_paranoid <= 2), one counter
per thread opened by PerfCyclesThreadInit (ds_TimeApiInit for the main thread, the job
scheduler for its workers). 
*/

/* an empty pair's ticks / cycles on the main thread (ds_TimeApiInit); 0 if unavailable */
extern u64	g_perf_wall_clock_overhead;
extern u64	g_perf_cycles_overhead;

/* Open thread's hardware counter; no-op if already open or unavailable. */
void	    PerfCyclesThreadInit(void);
/* Close thread's hardware counter; no-op if already closed */
void	    PerfCyclesThreadShutdown(void);
/* Return 1 if the calling thread's counter is open */
u32	        PerfCyclesThreadAvailable(void);

/* Return ticks */
static ds_ForceInline u64	PerfWallClockBegin(void);
/* Return ticks since begin */
static ds_ForceInline u64	PerfWallClockEnd(const u64 begin);

/* Return core cycles, 0 when unavailable */
static ds_ForceInline u64	PerfCyclesBegin(void);
/* Return core cycles since begin; 0 where unavailable */
static ds_ForceInline u64	PerfCyclesEnd(const u64 begin);

/* Internal: the calling thread's counter index + 1 (0: unavailable), and the counter's width mask */
extern ds_ThreadLocal u32	tl_perf_cycles_index;
extern u64			        g_perf_cycles_mask;

#endif

/* TODO: We want to build a easy-to-use repetition tester that encloses a bit of code, repeatedly
 *       running it, and keeping track of best/total time spent in the function, and so on. When no
 *       improvement has been made to the time in quite some time, we end the repetition test. This works
 *       very well in serialized environments, but may require different APIs for multithreaded performance
 *       testing?
 *      
 * TODO: goes in skill
 *  () How to properly instrument code
 *      () lfence + rdtsc yields a wall-clock timer which properly times the
 *          code inside, however, wall-clock timings come with the issue of
 *          not being able to distinguish turbo-modes, and so on.
 *      () hardware cycle counting yields a true measurement of actual performance
 *          but it only works on linux, and perhaps the overhead is to large / uncontrollable
 *          to use for small instruction block measurements?
 *      () Ideally, our API should use the best possible measurement tools available,
 *         so linux, windows and wasm measure differently.
 *      () Whenever the agent runs on new hardware / setup, it should first verify the overhead
 *          and use the appropriate API.
 *
 *  () How to derive best-case speed-of-light comparisions?
 *          "A basic baseline we cannot possibly beat, for example
 *           can we estimate the time it takes to simply load all
 *           data we work on, and compare our timings against that?"
 *      () Estimate bytes read
 *      () Estimate cache lines touched
 *      () Instructions-per-bytes?
 *      () Estimate loads differently dependeing on volume?
 *          Can we assume L1/L2/L3/DRAM?
 */


/********************************** Perf blocks: inline definitions **********************************/

#ifdef DS_PERFORMANCE_MEASUREMENT_API

#if (__DS_PLATFORM__ == __DS_WEB__)

static ds_ForceInline u64 PerfWallClockBegin(void)
{
	return ds_TimeNs();
}

static ds_ForceInline u64 PerfWallClockEnd(const u64 begin)
{
	return ds_TimeNs() - begin;
}

#else

static ds_ForceInline u64 PerfWallClockBegin(void)
{
	ds_AcqRelCompilerBarrier;
	const u64 tsc = Rdtsc();
	Lfence();
	ds_AcqRelCompilerBarrier;
	return tsc;
}

static ds_ForceInline u64 PerfWallClockEnd(const u64 begin)
{
	ds_AcqRelCompilerBarrier;
	Lfence();
	const u64 tsc = Rdtsc();
	ds_AcqRelCompilerBarrier;
	return tsc - begin;
}

#endif

#if (__DS_PLATFORM__ == __DS_LINUX__)

static ds_ForceInline u64 PerfCyclesBegin(void)
{
	const u32 index = tl_perf_cycles_index;
	if (index == 0)
	{
		return 0;
	}
	ds_AcqRelCompilerBarrier;
	const u64 cycles = Rdpmc(index - 1);
	Lfence();
	ds_AcqRelCompilerBarrier;
	return cycles;
}

static ds_ForceInline u64 PerfCyclesEnd(const u64 begin)
{
	const u32 index = tl_perf_cycles_index;
	if (index == 0)
	{
		return 0;
	}
	ds_AcqRelCompilerBarrier;
	Lfence();
	const u64 cycles = Rdpmc(index - 1);
	ds_AcqRelCompilerBarrier;
	return (cycles - begin) & g_perf_cycles_mask;
}

#else

static ds_ForceInline u64 PerfCyclesBegin(void)
{
	return 0;
}

static ds_ForceInline u64 PerfCyclesEnd(const u64 begin)
{
	return 0;
}

#endif

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

#ifdef __cplusplus
} 
#endif

#endif

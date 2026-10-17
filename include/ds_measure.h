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
ds_measure.h
============
Performance measurement: timing a code block with the TSC (PerfWallClock), the core's cycle counter
(PerfCycles) or both (PerfBegin/PerfEnd). Only with DS_PERFORMANCE_MEASUREMENT_API. Inline definitions at the
bottom.
*/

#ifndef __DREAMSCAPE_MEASURE_H__
#define __DREAMSCAPE_MEASURE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_time.h"

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

PerfBegin/PerfEnd: both in one pair (a struct PerfStamp), one fence per side; the repetition tester's pair. The
tick window encloses the two cycle reads, so its tick cost is about three times the wall-clock pair's.
Each pair's cost on the main thread is measured by ds_MeasureApiInit: subtract it from a minimum, never from a
single sample.

PerfCycles: core cycles in user mode, independent of the core clock, following the thread
across cores. Linux only (perf_event_open: kernel.perf_event_paranoid <= 2), one counter
per thread opened by PerfCyclesThreadInit (ds_MeasureApiInit for the main thread, the job
scheduler for its workers). 
*/

/* an empty pair's ticks / cycles on the main thread (ds_MeasureApiInit); 0 if unavailable */
extern u64	g_perf_wall_clock_overhead;
extern u64	g_perf_cycles_overhead;
extern u64	g_perf_overhead_ticks;		/* PerfBegin/PerfEnd */
extern u64	g_perf_overhead_cycles;

struct PerfStamp
{
	u64	ticks;
	u64	cycles;
};

/* Open the main thread's counter and measure the pairs' costs; after ds_TimeApiInit */
void	ds_MeasureApiInit(void);

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

static ds_ForceInline struct PerfStamp	PerfBegin(void);
/* Return ticks and cycles since begin (cycles 0 where unavailable) */
static ds_ForceInline struct PerfStamp	PerfEnd(const struct PerfStamp begin);

/* Internal: the calling thread's counter index + 1 (0: unavailable), and the counter's width mask */
extern ds_ThreadLocal u32	tl_perf_cycles_index;
extern u64			        g_perf_cycles_mask;

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

/********************************** inline definitions **********************************/

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

#if (__DS_PLATFORM__ == __DS_LINUX__)

static ds_ForceInline struct PerfStamp PerfBegin(void)
{
	struct PerfStamp stamp;
	const u32 index = tl_perf_cycles_index;
	ds_AcqRelCompilerBarrier;
	stamp.ticks = Rdtsc();
	stamp.cycles = index ? Rdpmc(index - 1) : 0;
	Lfence();
	ds_AcqRelCompilerBarrier;
	return stamp;
}

static ds_ForceInline struct PerfStamp PerfEnd(const struct PerfStamp begin)
{
	struct PerfStamp stamp;
	const u32 index = tl_perf_cycles_index;
	ds_AcqRelCompilerBarrier;
	Lfence();
	stamp.cycles = index ? Rdpmc(index - 1) : 0;
	stamp.ticks = Rdtsc();
	ds_AcqRelCompilerBarrier;
	stamp.ticks -= begin.ticks;
	stamp.cycles = index ? (stamp.cycles - begin.cycles) & g_perf_cycles_mask : 0;
	return stamp;
}

#else

static ds_ForceInline struct PerfStamp PerfBegin(void)
{
	const struct PerfStamp stamp = { .ticks = PerfWallClockBegin(), .cycles = 0 };
	return stamp;
}

static ds_ForceInline struct PerfStamp PerfEnd(const struct PerfStamp begin)
{
	const struct PerfStamp stamp = { .ticks = PerfWallClockEnd(begin.ticks), .cycles = 0 };
	return stamp;
}

#endif

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

#ifdef __cplusplus
}
#endif

#endif

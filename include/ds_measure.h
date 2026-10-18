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
(PerfCycles) or both (PerfBegin/PerfEnd), and the repetition tester RT built on the last. Only with
DS_PERFORMANCE_MEASUREMENT_API. Sections: Perf blocks, RT; inline definitions at the bottom.
*/

#ifndef __DREAMSCAPE_MEASURE_H__
#define __DREAMSCAPE_MEASURE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
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

PerfWallClock: best available wall-clock timer for performance measurements (x64: the TSC; web: ns from the OS
clock).

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

/* a reading (PerfBegin) or the counts since one (PerfEnd) */
struct PerfStamp
{
	u64	ticks;
	u64	cycles;
};

/* Open the main thread's counter and measure the pairs' costs; after ds_TimeApiInit */
void	ds_MeasureApiInit(void);

/* Open the calling thread's hardware counter; no-op if already open, logged (S_NOTE) if unavailable */
void	    PerfCyclesThreadInit(void);
/* Close the calling thread's hardware counter; no-op if already closed */
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

/* Return ticks and core cycles (cycles 0 where unavailable) */
static ds_ForceInline struct PerfStamp	PerfBegin(void);
/* Return ticks and cycles since begin (cycles 0 where unavailable) */
static ds_ForceInline struct PerfStamp	PerfEnd(const struct PerfStamp begin);

/* Internal: the calling thread's counter index + 1 (0: unavailable), and the counter's width mask */
extern ds_ThreadLocal u32	tl_perf_cycles_index;
extern u64			        g_perf_cycles_mask;

/*
RT
==
Repetition tester: a block is run repeatedly, each repetition one or more RTBeginTime/RTEndTime regions, until no
faster repetition has appeared for retry_s seconds; the fastest, slowest and total repetition are kept in both
PerfBegin/PerfEnd counts. "Faster" is decided on wall ticks: the fastest repetition is the one that ran
uninterrupted at the highest clock, and its cycles are the block's cycles (a cycle minimum would prefer a low
clock for memory-bound blocks, which wait a fixed time). RTPrintStatistics subtracts the pairs' cost (regions x
g_perf_overhead_*) from the minimum only and prints cycles when the thread had a counter at RTWave; for a job
phase they are the master's own. A zeroed struct RT is a fresh tester; on an error it stays in RT_ERROR, the loop
ends and RTPrintStatistics prints the error. On the web ticks are ns.

    struct RT t = { 0 };
    RTWave(&t, bytes_per_repetition, 10.0);
    do
    {
        reset
        RTBeginTime(&t);
        code-block; 
        RTEndTime(&t);
    } while (RTTestingCheck(&t));
    RTPrintStatistics(&t, stdout);
*/

enum RTState
{
	RT_UNINITIALIZED = 0,
	RT_TESTING,
	RT_COMPLETED,
	RT_ERROR,
};

struct RT
{
	enum RTState		state;
	u32			cycles;		/* the thread had a counter at RTWave: cycle figures are printed */
	const char *		error;		/* the first error's message */
	u32			regions;	/* timed regions per repetition, from the first one */
	u32			enter_count;	/* the open repetition's regions */
	u32			exit_count;
	u64			bytes;		/* processed per repetition */
	u64			retry_ticks;	/* wall ticks without a new minimum before the wave completes */
	u64			retry_start;	/* wall ticks at the last new minimum */
	struct PerfStamp	begin;		/* the open region's start */
	struct PerfStamp	current;	/* the open repetition's sums */
	u64			count;		/* repetitions */
	struct PerfStamp	total;
	struct PerfStamp	min;		/* the fastest repetition: both of its counts */
	struct PerfStamp	max;		/* the slowest */
};

/* Start a wave (RT_UNINITIALIZED) or continue one (RT_COMPLETED: bytes must not change) */
void	RTWave(struct RT *t, const u64 bytes, const f64 retry_s);
/* Close the repetition: record it, restart the retry clock on a new minimum; return 1 to run another */
u32	RTTestingCheck(struct RT *t);
/* A timed region: all of a repetition's regions sum into it */
static ds_ForceInline void	RTBeginTime(struct RT *t);
static ds_ForceInline void	RTEndTime(struct RT *t);
/*
Print the fastest (overhead subtracted), slowest and average repetition: ms, GB/s and, with a counter, cycles,
cycles/B and the core clock during the fastest; the error in RT_ERROR
*/
void	RTPrintStatistics(const struct RT *t, FILE *file);

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

static ds_ForceInline void RTBeginTime(struct RT *t)
{
	t->enter_count += 1;
	t->begin = PerfBegin();
}

static ds_ForceInline void RTEndTime(struct RT *t)
{
	const struct PerfStamp d = PerfEnd(t->begin);
	t->current.ticks += d.ticks;
	t->current.cycles += d.cycles;
	t->exit_count += 1;
}

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

#ifdef __cplusplus
}
#endif

#endif

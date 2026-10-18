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

#include <stddef.h>
#include "ds_base.h"

#ifdef DS_PERFORMANCE_MEASUREMENT_API

u64 g_perf_wall_clock_overhead = 0;
u64 g_perf_cycles_overhead = 0;
u64 g_perf_overhead_ticks = 0;
u64 g_perf_overhead_cycles = 0;
ds_ThreadLocal u32 tl_perf_cycles_index = 0;
u64 g_perf_cycles_mask = 0;

#define PERF_OVERHEAD_SAMPLES 4096

#if __DS_PLATFORM__ == __DS_LINUX__

#include <sys/mman.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/*
PerfCycles: one perf event per thread (pid 0, cpu -1: the calling thread wherever it runs; the kernel saves and
restores the counter across context switches and migrations), read with rdpmc from user mode. The event's mmap
page tells whether user-mode reads are allowed (cap_user_rdpmc), which counter to read (index, 0 while the event
isn't scheduled on the core) and its width. index is cached at init: a single uninherited per-thread event keeps
its counter unless the kernel has to reschedule counters under contention with other perf users, which the
engine does not expect.
*/
static ds_ThreadLocal int tl_perf_cycles_fd = -1;
static ds_ThreadLocal struct perf_event_mmap_page *tl_perf_cycles_page = NULL;

void PerfCyclesThreadInit(void)
{
	if (tl_perf_cycles_index)
	{
		return;
	}

	struct perf_event_attr attr;
	memset(&attr, 0, sizeof(attr));
	attr.type = PERF_TYPE_HARDWARE;
	attr.size = sizeof(attr);
	attr.config = PERF_COUNT_HW_CPU_CYCLES;
	attr.exclude_kernel = 1;
	attr.exclude_hv = 1;

	const int fd = (int) syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0);
	if (fd < 0)
	{
		const char *reason = (errno == EACCES || errno == EPERM) ? "not permitted (kernel.perf_event_paranoid > 2?)"
				   : (errno == ENOENT || errno == ENODEV || errno == EOPNOTSUPP) ? "no cycle counter (VM?)"
				   : "perf_event_open failed";
		Log(T_SYSTEM, S_NOTE, "PerfCycles unavailable on thread %u: %s (errno %d)", ds_ThreadSelfIndex(), reason, errno);
		return;
	}

	struct perf_event_mmap_page *page = mmap(NULL, (size_t) sysconf(_SC_PAGESIZE), PROT_READ, MAP_SHARED, fd, 0);
	if (page == MAP_FAILED)
	{
		Log(T_SYSTEM, S_NOTE, "PerfCycles unavailable on thread %u: mmap failed (errno %d)", ds_ThreadSelfIndex(), errno);
		close(fd);
		return;
	}
	if (!page->cap_user_rdpmc || page->index == 0)
	{
		Log(T_SYSTEM, S_NOTE, "PerfCycles unavailable on thread %u: no user-mode rdpmc (cap %u, index %u)", ds_ThreadSelfIndex(), (u32) page->cap_user_rdpmc, page->index);
		munmap(page, (size_t) sysconf(_SC_PAGESIZE));
		close(fd);
		return;
	}

	tl_perf_cycles_fd = fd;
	tl_perf_cycles_page = page;
	tl_perf_cycles_index = page->index;
	g_perf_cycles_mask = (page->pmc_width >= 64) ? U64_MAX : (((u64) 1 << page->pmc_width) - 1);
}

void PerfCyclesThreadShutdown(void)
{
	if (tl_perf_cycles_index == 0)
	{
		return;
	}
	munmap(tl_perf_cycles_page, (size_t) sysconf(_SC_PAGESIZE));
	close(tl_perf_cycles_fd);
	tl_perf_cycles_page = NULL;
	tl_perf_cycles_fd = -1;
	tl_perf_cycles_index = 0;
}

u32 PerfCyclesThreadAvailable(void)
{
	return tl_perf_cycles_index != 0;
}

#else

/* no user-mode counter on Windows (QueryThreadCycleTime is TSC-based) or the web */
void PerfCyclesThreadInit(void) { }
void PerfCyclesThreadShutdown(void) { }
u32 PerfCyclesThreadAvailable(void) { return 0; }

#endif

/* the minimum of PERF_OVERHEAD_SAMPLES empty pairs, per pair */
void ds_MeasureApiInit(void)
{
	PerfCyclesThreadInit();

	u64 ticks = U64_MAX;
	for (u32 i = 0; i < PERF_OVERHEAD_SAMPLES; ++i)
	{
		const u64 begin = PerfWallClockBegin();
		const u64 t = PerfWallClockEnd(begin);
		ticks = (t < ticks) ? t : ticks;
	}
	g_perf_wall_clock_overhead = ticks;

	g_perf_cycles_overhead = 0;
	if (PerfCyclesThreadAvailable())
	{
		u64 cycles = U64_MAX;
		for (u32 i = 0; i < PERF_OVERHEAD_SAMPLES; ++i)
		{
			const u64 begin = PerfCyclesBegin();
			const u64 c = PerfCyclesEnd(begin);
			cycles = (c < cycles) ? c : cycles;
		}
		g_perf_cycles_overhead = cycles;
	}

	struct PerfStamp both = { .ticks = U64_MAX, .cycles = U64_MAX };
	for (u32 i = 0; i < PERF_OVERHEAD_SAMPLES; ++i)
	{
		const struct PerfStamp begin = PerfBegin();
		const struct PerfStamp d = PerfEnd(begin);
		both.ticks = (d.ticks < both.ticks) ? d.ticks : both.ticks;
		both.cycles = (d.cycles < both.cycles) ? d.cycles : both.cycles;
	}
	g_perf_overhead_ticks = both.ticks;
	g_perf_overhead_cycles = PerfCyclesThreadAvailable() ? both.cycles : 0;
}

/*
RT
==
*/

#if __DS_PLATFORM__ == __DS_WEB__

/* PerfWallClock ticks are ns on the web */
static u64 RTTicksPerSecond(void) { return NSEC_PER_SEC; }
#else
static u64 RTTicksPerSecond(void) { return TscFrequency(); }
#endif

static void RTError(struct RT *t, const char *msg)
{
	if (t->state != RT_ERROR)
	{
		t->state = RT_ERROR;
		t->error = msg;
		Log(T_SYSTEM, S_ERROR, "RT: %s", msg);
	}
}

void RTWave(struct RT *t, const u64 bytes, const f64 retry_s, const enum RTSelect select)
{
	const enum RTSelect resolved = (select == RT_SELECT_CYCLES && PerfCyclesThreadAvailable())
		? RT_SELECT_CYCLES
		: RT_SELECT_TICKS;

	switch (t->state)
	{
	    case RT_UNINITIALIZED:
        {
	    	t->bytes = bytes;
	    	t->select = resolved;
	    	break;
        }

	    case RT_COMPLETED:
        {
	    	if (bytes != t->bytes)
	    	{
	    		RTError(t, "bytes per repetition changed between waves");
	    		return;
	    	}
	    	if (resolved != t->select)
	    	{
	    		RTError(t, "selection changed between waves");
	    		return;
	    	}
	    	break;
        }

	    case RT_TESTING:
        {
	    	RTError(t, "wave started before the previous one completed");
	    	return;
        }

	    case RT_ERROR:
        {
	    	return;
        }
	}

	t->state = RT_TESTING;
	t->retry_ticks = (u64) (retry_s * (f64) RTTicksPerSecond());
	t->retry_start = PerfWallClockBegin();
	t->enter_count = 0;
	t->exit_count = 0;
	t->current.ticks = 0;
	t->current.cycles = 0;
}

u32 RTTestingCheck(struct RT *t)
{
	if (t->state != RT_TESTING)
	{
		return 0;
	}
	if (t->enter_count == 0)
	{
		RTError(t, "no timed region in the repetition");
		return 0;
	}
	if (t->enter_count != t->exit_count)
	{
		RTError(t, "a timed region was not closed");
		return 0;
	}
	if (t->count == 0)
	{
		t->regions = t->enter_count;
	}
	else if (t->regions != t->enter_count)
	{
		RTError(t, "timed region count changed between repetitions");
		return 0;
	}

	const struct PerfStamp current = t->current;
	const u64 now = PerfWallClockBegin();
	const u64 figure = (t->select == RT_SELECT_CYCLES) ? current.cycles : current.ticks;
	const u64 figure_min = (t->select == RT_SELECT_CYCLES) ? t->min.cycles : t->min.ticks;
	const u64 figure_max = (t->select == RT_SELECT_CYCLES) ? t->max.cycles : t->max.ticks;
	if (t->count == 0 || figure < figure_min)
	{
		t->min = current;
		t->retry_start = now;
	}
	if (t->count == 0 || figure_max < figure)
	{
		t->max = current;
	}
	t->total.ticks += current.ticks;
	t->total.cycles += current.cycles;
	t->count += 1;

	t->enter_count = 0;
	t->exit_count = 0;
	t->current.ticks = 0;
	t->current.cycles = 0;

	if (t->retry_ticks < now - t->retry_start)
	{
		t->state = RT_COMPLETED;
		return 0;
	}
	return 1;
}

static f64 RTSeconds(const u64 ticks)
{
	return (f64) ticks / (f64) RTTicksPerSecond();
}

/* GB/s (1024^3), 0 for a zero time */
static f64 RTThroughput(const u64 bytes, const f64 s)
{
	return (s > 0.0) ? (f64) bytes / s / (1024.0 * 1024.0 * 1024.0) : 0.0;
}

void RTPrintStatistics(const struct RT *t, FILE *file)
{
	if (t->state == RT_ERROR)
	{
		fprintf(file, "RT error: %s\n", t->error);
		return;
	}
	if (t->count == 0)
	{
		fprintf(file, "RT: no repetitions\n");
		return;
	}

	/* the pairs' own cost, once per region, out of the minimum only */
	const u64 overhead_ticks = t->regions * g_perf_overhead_ticks;
	const u64 overhead_cycles = t->regions * g_perf_overhead_cycles;
	const u64 ticks_min = (overhead_ticks < t->min.ticks) ? t->min.ticks - overhead_ticks : 0;
	const u64 cycles_min = (overhead_cycles < t->min.cycles) ? t->min.cycles - overhead_cycles : 0;

	const f64 s_min = RTSeconds(ticks_min);
	const f64 s_max = RTSeconds(t->max.ticks);
	const f64 s_avg = RTSeconds(t->total.ticks) / (f64) t->count;

	if (t->select == RT_SELECT_CYCLES)
	{
		const f64 ghz = (ticks_min) ? (f64) cycles_min / (f64) ticks_min * (f64) RTTicksPerSecond() / 1e9 : 0.0;
		fprintf(file, "min: [%.5fms] %.3fGB/s  %llu cycles  %.4f cycles/B  core %.2f GHz  (%llu repetitions, overhead subtracted)\n",
			s_min * 1e3, RTThroughput(t->bytes, s_min), (unsigned long long) cycles_min,
			(f64) cycles_min / (f64) t->bytes, ghz, (unsigned long long) t->count);
		fprintf(file, "max: [%.5fms] %.3fGB/s  %llu cycles\n",
			s_max * 1e3, RTThroughput(t->bytes, s_max), (unsigned long long) t->max.cycles);
		fprintf(file, "avg: [%.5fms] %.3fGB/s  %.0f cycles\n",
			s_avg * 1e3, RTThroughput(t->bytes, s_avg), (f64) t->total.cycles / (f64) t->count);
	}
	else
	{
		fprintf(file, "min: [%.5fms] %.3fGB/s  %.4f ticks/B  (%llu repetitions, overhead subtracted; by wall time)\n",
			s_min * 1e3, RTThroughput(t->bytes, s_min), (f64) ticks_min / (f64) t->bytes, (unsigned long long) t->count);
		fprintf(file, "max: [%.5fms] %.3fGB/s\n", s_max * 1e3, RTThroughput(t->bytes, s_max));
		fprintf(file, "avg: [%.5fms] %.3fGB/s\n", s_avg * 1e3, RTThroughput(t->bytes, s_avg));
	}
}

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

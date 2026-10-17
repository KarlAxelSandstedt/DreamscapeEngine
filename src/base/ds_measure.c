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

#endif /* DS_PERFORMANCE_MEASUREMENT_API */

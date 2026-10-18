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

#include <stdlib.h>
#include "ds_test.h"

/*
Memory figures per cache level for the speed-of-light floors (performance-measuring skill): sequential read,
sequential write, random lines with independent addresses, and a dependent chase, each over a footprint that
sits in L1, L2, L3 or DRAM. The results go to agent_notes/perf_machines.txt.
*/

#define MEM_LINE	64

struct mem_Input
{
	u64 *	buf;		/* footprint bytes */
	u64	count;		/* u64 elements */
	u64	lines;		/* MEM_LINE-byte lines */
	u32 *	index;		/* a permutation of the lines */
	u64	sum;		/* the test's result, stored inside the region */
};

static void *mem_init(const u64 footprint)
{
	struct mem_Input *in = malloc(sizeof(struct mem_Input));
	in->count = footprint / sizeof(u64);
	in->lines = footprint / MEM_LINE;
	in->buf = malloc(footprint);
	in->index = malloc(in->lines * sizeof(u32));
	for (u64 i = 0; i < in->count; ++i)
	{
		in->buf[i] = i;
	}
	for (u64 i = 0; i < in->lines; ++i)
	{
		in->index[i] = (u32) i;
	}
	/* Fisher-Yates: independent random lines */
	for (u64 i = in->lines - 1; i > 0; --i)
	{
		const u64 j = RngU64Range(0, i);
		const u32 t = in->index[i];
		in->index[i] = in->index[j];
		in->index[j] = t;
	}
	/* Sattolo: one cycle through all lines, each line's first word holds the next line */
	u32 *cycle = malloc(in->lines * sizeof(u32));
	for (u64 i = 0; i < in->lines; ++i)
	{
		cycle[i] = (u32) i;
	}
	for (u64 i = in->lines - 1; i > 0; --i)
	{
		const u64 j = RngU64Range(0, i - 1);
		const u32 t = cycle[i];
		cycle[i] = cycle[j];
		cycle[j] = t;
	}
	for (u64 i = 0; i < in->lines; ++i)
	{
		in->buf[(u64) cycle[i] * (MEM_LINE / sizeof(u64))] = cycle[(i + 1) % in->lines];
	}
	free(cycle);
	in->sum = 0;
	return in;
}

static void *mem_init_l1(void)	{ return mem_init(16 * 1024); }
static void *mem_init_l2(void)	{ return mem_init(128 * 1024); }
static void *mem_init_l3(void)	{ return mem_init(1024 * 1024); }
static void *mem_init_dram(void)	{ return mem_init(64 * 1024 * 1024); }

static void mem_free(void *args)
{
	struct mem_Input *in = args;
	free(in->index);
	free(in->buf);
	free(in);
}

static void mem_sequential_read(void *args)
{
	struct mem_Input *in = args;
	const u64 *buf = in->buf;
	u64 sum = 0;
	for (u64 i = 0; i < in->count; ++i)
	{
		sum += buf[i];
	}
	in->sum = sum;
}

static void mem_sequential_write(void *args)
{
	struct mem_Input *in = args;
	u64 *buf = in->buf;
	const u64 v = in->sum + 1;
	for (u64 i = 0; i < in->count; ++i)
	{
		buf[i] = v;
	}
	in->sum = v;
}

/* one word per line, lines in a random order known ahead: the misses overlap */
static void mem_random_lines(void *args)
{
	struct mem_Input *in = args;
	const u64 *buf = in->buf;
	const u32 *index = in->index;
	u64 sum = 0;
	for (u64 i = 0; i < in->lines; ++i)
	{
		sum += buf[(u64) index[i] * (MEM_LINE / sizeof(u64))];
	}
	in->sum = sum;
}

/* each line's address comes from the previous load: nothing overlaps */
static void mem_chase(void *args)
{
	struct mem_Input *in = args;
	const u64 *buf = in->buf;
	u64 next = 0;
	for (u64 i = 0; i < in->lines; ++i)
	{
		next = buf[next * (MEM_LINE / sizeof(u64))];
	}
	in->sum = next;
}

#define MEM_TESTS(level, footprint) \
	{ .id = "mem_sequential_read_" #level, .size = footprint, .test = &mem_sequential_read, .test_init = &mem_init_##level, .test_free = &mem_free }, \
	{ .id = "mem_sequential_write_" #level, .size = footprint, .test = &mem_sequential_write, .test_init = &mem_init_##level, .test_free = &mem_free }, \
	{ .id = "mem_random_lines_" #level, .size = footprint, .test = &mem_random_lines, .test_init = &mem_init_##level, .test_free = &mem_free }, \
	{ .id = "mem_chase_" #level, .size = footprint, .test = &mem_chase, .test_init = &mem_init_##level, .test_free = &mem_free }

struct test_PerformanceSerial memory_serial_test[] =
{
	MEM_TESTS(l1, 16 * 1024),
	MEM_TESTS(l2, 128 * 1024),
	MEM_TESTS(l3, 1024 * 1024),
	MEM_TESTS(dram, 64 * 1024 * 1024),
};

struct suite_Performance m_memory_performance_suite =
{
	.id = "Memory Performance",
	.serial_test = memory_serial_test,
	.serial_test_count = sizeof(memory_serial_test) / sizeof(memory_serial_test[0]),
	.parallel_test = NULL,
	.parallel_test_count = 0,
};

struct suite_Performance *memory_performance_suite = &m_memory_performance_suite;

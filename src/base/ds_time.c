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

#define _GNU_SOURCE

#include <stddef.h>
#include "ds_base.h"

u64 *g_tsc_skew = NULL;

struct tscTimer 
{
	u64 tsc_start;
	u64 tsc_freq;
};

/*
 * coarse timer for general use
 */
struct timer
{
	u64 ns_start;
	u64 tsc_start;

	u64 ns_resolution;	/* ns per tick */
};

static struct tscTimer g_precision_timer;
static struct timer g_timer;

#if __DS_PLATFORM__ == __DS_LINUX__

#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>

u64 ds_TimeNs(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (u64) NSEC_PER_SEC * ts.tv_sec + ts.tv_nsec - g_timer.ns_start;
}

struct ping_pong_data
{
	u32	a_lock;
	u32	a_iteration_test;
	u32	logical_core_count;
	u32	iterations;
	u64 *	tsc_reference;
	u64 *	tsc_iterator;
};

#define UNLOCKED_BY_REFERENCE 	1
#define UNLOCKED_BY_ITERATOR	2
void *PingPongReference(void *data_void)
{
	struct ping_pong_data *data = data_void;

	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0, &cpuset);	

	/* thread is allowed to run on intersection of cpu_set_t and actual system cpus. */
	if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to set thread affinity in TscEstimateSkew, exiting.");	
		FatalCleanupAndExit();
	}

	u32 c;
	g_tsc_skew[0] = 0;
	for (u32 core = 1; core < data->logical_core_count; ++core)
	{
		AtomicStoreRel32(&data->a_iteration_test, 1);

		for (u32 i = 0; i < data->iterations; ++i)
		{
			while (AtomicLoadAcq32(&data->a_lock) != UNLOCKED_BY_ITERATOR);
			data->tsc_reference[i] = Rdtscp(&c);
			AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_REFERENCE);
		}
		/* wait until last iteration of iterator is complete before calculating skew */
		while (AtomicLoadAcq32(&data->a_iteration_test) != 0);

		b64 skew = { .i = I64_MAX, };
		for (u32 i = 0; i < data->iterations; ++i)
		{
			const b64 it_skew = { .u = data->tsc_iterator[i] - data->tsc_reference[i] };
			if (it_skew.i < skew.i)
			{
				skew = it_skew;
			}
		}

		g_tsc_skew[core] = skew.u;
	}

	return NULL;
}

void *PingPongCoreIterator(void *data_void)
{
	struct ping_pong_data *data = data_void;

	u32 c;
	for (u32 core = 1; core < data->logical_core_count; ++core)
	{
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(core, &cpuset);	

		/* thread is allowed to run on intersection of cpu_set_t and actual system cpus. */
		if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to set thread affinity in TscEstimateSkew, exiting.");	
			FatalCleanupAndExit();
		
		}

		while (AtomicLoadAcq32(&data->a_iteration_test) != 1);

		AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_ITERATOR);

		for (u32 i = 0; i < data->iterations; ++i)
		{
			while (AtomicLoadAcq32(&data->a_lock) != UNLOCKED_BY_REFERENCE);
			data->tsc_iterator[i] = Rdtscp(&c);
			AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_ITERATOR);
		}

		AtomicStoreRel32(&data->a_lock, 0);
		AtomicStoreRel32(&data->a_iteration_test, 0);
	}

	return NULL;
}

/*
Ping-Pong calibration of core skew:

Skew Core: (c)	   		      Reference Core: (0)
      	    |	   		  		       |
=================================================================== ITERATION N
       	    |					       |
     [ RELEASE LOCK ] -------------------------> [ GAIN LOCK ]
       	    |					       |
            |					       V
            |                                         TSC() ----> t0_0
       	    |					       |
            V					       V
      [ GAIN LOCK ] <-------------------------- [ RELEASE LOCK ]
       	    |					       |
            V					       |
           TSC() --------------------------------------+--------> tc_1
       	    |					       |
=================================================================== ITERATION N+1
      	    |					       |

It follows that tc_1 = t0_0 + time_execution_instructions + extra + skew.
By running many iterations, we hope that extra goes to 0; so we estimate
the skew by min(tc_1 - t0_0).
 */
static void TscEstimateSkew(struct arena *persistent)
{	
	struct ping_pong_data data =
	{
		.logical_core_count = (u32) sysconf(_SC_NPROCESSORS_ONLN),
		.iterations = 100000,
		.a_lock = 0,
		.a_iteration_test = 0,
	};

	g_tsc_skew = ArenaPushZero(persistent, data.logical_core_count*sizeof(u64));
	ArenaPushRecord(persistent);
	data.tsc_reference = ArenaPush(persistent, data.iterations * sizeof(u64));
	data.tsc_iterator = ArenaPush(persistent, data.iterations * sizeof(u64));

	pthread_attr_t attr;
	if (pthread_attr_init(&attr) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	pthread_t thread1, thread2;
	if (pthread_create(&thread1, &attr, PingPongReference, (void *) &data) != 0
	    || pthread_create(&thread2, &attr, PingPongCoreIterator, (void *) &data) != 0)

	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	if (pthread_attr_destroy(&attr) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	void *garbage;
	pthread_join(thread1, &garbage);
	pthread_join(thread2, &garbage);
	ArenaPopRecord(persistent);
}

void ds_TimeApiInit(struct arena *persistent)
{
	struct timespec ts, tmp;
	clock_getres(CLOCK_MONOTONIC_RAW, &ts);
	g_timer.ns_resolution = ts.tv_nsec;
	ds_Assert(ts.tv_sec == 0);

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	u32 tmp2;
	/* instruction fence */
	g_precision_timer.tsc_start = __rdtscp(&tmp2);
	g_timer.ns_start = NSEC_PER_SEC * ts.tv_sec + ts.tv_nsec;
	g_timer.tsc_start = g_precision_timer.tsc_start;

	clock_gettime(CLOCK_MONOTONIC_RAW, &tmp);
	const u64 ms = 100;
	const u64 goal = (tmp.tv_sec) * NSEC_PER_SEC + tmp.tv_nsec + NSEC_PER_SEC / (1000/ms);
	while ((u64) (tmp.tv_sec * NSEC_PER_SEC + tmp.tv_nsec) < goal)
	{
		clock_gettime(CLOCK_MONOTONIC_RAW, &tmp);
	}
	u64 end = __rdtsc();
	g_precision_timer.tsc_freq = (1000/ms) * (end - g_precision_timer.tsc_start);

	TscEstimateSkew(persistent);
}

#elif __DS_PLATFORM__ == __DS_WEB__

#include <sys/time.h>

u64 ds_TimeNs(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (u64) NSEC_PER_SEC * ts.tv_sec + ts.tv_nsec - g_timer.ns_start;
}

void ds_TimeApiInit(struct arena *persistent)
{
	struct timespec ts;
	clock_getres(CLOCK_MONOTONIC, &ts);
	g_timer.ns_resolution = ts.tv_nsec;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	g_timer.ns_start = NSEC_PER_SEC * ts.tv_sec + ts.tv_nsec;

	g_timer.tsc_start = 0;
	g_precision_timer.tsc_start = 0;
	g_precision_timer.tsc_freq = 0;
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <windows.h>
#include <intrin.h>

u64 ds_TimeNs(void)
{
	LARGE_INTEGER ret_val;
	QueryPerformanceCounter(&ret_val);
	return (u64) ret_val.QuadPart * g_timer.ns_resolution - g_timer.ns_start;
}

struct ping_pong_data
{
	u32	a_lock;
	u32	a_iteration_test;
	u32	logical_core_count;
	u32	iterations;
	u64 *	tsc_reference;
	u64 *	tsc_iterator;
};

#define UNLOCKED_BY_REFERENCE 	1
#define UNLOCKED_BY_ITERATOR	2
static DWORD WINAPI PingPongReference(void *data_void)
{
	struct ping_pong_data *data = data_void;

	const HANDLE thread = GetCurrentThread();

	GROUP_AFFINITY affinity = { 0 };
	affinity.Group = 0;
	affinity.Mask = 0;
	
	if (!SetThreadGroupAffinity(thread, &affinity, NULL))
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to set thread affinity in TscEstimateSkew, exiting.");	
		FatalCleanupAndExit();
	}

	u32 c;
	g_tsc_skew[0] = 0;
	for (u32 core = 1; core < data->logical_core_count; ++core)
	{
		AtomicStoreRel32(&data->a_iteration_test, 1);

		for (u32 i = 0; i < data->iterations; ++i)
		{
			while (AtomicLoadAcq32(&data->a_lock) != UNLOCKED_BY_ITERATOR);
			data->tsc_reference[i] = Rdtscp(&c);
			AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_REFERENCE);
		}
		/* wait until last iteration of iterator is complete before calculating skew */
		while (AtomicLoadAcq32(&data->a_iteration_test) != 0);

		b64 skew = { .i = I64_MAX, };
		for (u32 i = 0; i < data->iterations; ++i)
		{
			const b64 it_skew = { .u = data->tsc_iterator[i] - data->tsc_reference[i] };
			if (it_skew.i < skew.i)
			{
				skew = it_skew;
			}
		}

		g_tsc_skew[core] = skew.u;
	}

	return 0;
}

static DWORD WINAPI PingPongCoreIterator(void *data_void)
{
	struct ping_pong_data *data = data_void;

	const HANDLE thread = GetCurrentThread();

	u32 c;
	for (u32 core = 1; core < data->logical_core_count; ++core)
	{
		GROUP_AFFINITY affinity = { 0 };
		affinity.Group = core / 64;
		affinity.Mask = (u64) 1 << (core % 64);
		
		if (!SetThreadGroupAffinity(thread, &affinity, NULL))
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to set thread affinity in TscEstimateSkew, exiting.");	
			FatalCleanupAndExit();
		}

		while (AtomicLoadAcq32(&data->a_iteration_test) != 1);

		AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_ITERATOR);

		for (u32 i = 0; i < data->iterations; ++i)
		{
			while (AtomicLoadAcq32(&data->a_lock) != UNLOCKED_BY_REFERENCE);
			data->tsc_iterator[i] = Rdtscp(&c);
			AtomicStoreRel32(&data->a_lock, UNLOCKED_BY_ITERATOR);
		}

		AtomicStoreRel32(&data->a_lock, 0);
		AtomicStoreRel32(&data->a_iteration_test, 0);
	}

	return 0;
}

/*
Ping-Pong calibration of core skew:

Skew Core: (c)	   		      Reference Core: (0)
      	    |	   		  		       |
=================================================================== ITERATION N
       	    |					       |
     [ RELEASE LOCK ] -------------------------> [ GAIN LOCK ]
       	    |					       |
            |					       V
            |                                         TSC() ----> t0_0
       	    |					       |
            V					       V
      [ GAIN LOCK ] <-------------------------- [ RELEASE LOCK ]
       	    |					       |
            V					       |
           TSC() --------------------------------------+--------> tc_1
       	    |					       |
=================================================================== ITERATION N+1
      	    |					       |

It follows that tc_1 = t0_0 + time_execution_instructions + extra + skew.
By running many iterations, we hope that extra goes to 0; so we estimate
the skew by min(tc_1 - t0_0).
 */
static void TscEstimateSkew(struct arena *persistent)
{	
	SYSTEM_INFO info;
	GetSystemInfo(&info);	

	struct ping_pong_data data =
	{
		.logical_core_count = (u32) info.dwNumberOfProcessors,
		.iterations = 100000,
		.a_lock = 0,
		.a_iteration_test = 0,
	};

	g_tsc_skew = ArenaPushZero(persistent, data.logical_core_count*sizeof(u64));
	ArenaPushRecord(persistent);
	data.tsc_reference = ArenaPush(persistent, data.iterations * sizeof(u64));
	data.tsc_iterator = ArenaPush(persistent, data.iterations * sizeof(u64));

  	LPSECURITY_ATTRIBUTES   lpThreadAttributes = NULL; 	/* default attributes */
  	DWORD                   dwCreationFlags = 0;		/* default creation flags */
	DWORD			dwStackSize = 0;
	HANDLE ref = CreateThread(lpThreadAttributes, dwStackSize, PingPongReference, (void *) &data, dwCreationFlags, NULL);
	HANDLE iter = CreateThread(lpThreadAttributes, dwStackSize, PingPongCoreIterator, (void *) &data, dwCreationFlags, NULL);
	if (!ref || !iter)
	{
		LogSystemError(S_FATAL);
		FatalCleanupAndExit();
	}

	u32 err = 0;
	const u32 res1 = WaitForSingleObject(ref, INFINITE);
	const u32 res2 = WaitForSingleObject(iter, INFINITE);
	if (res1 != WAIT_OBJECT_0 || res2 != WAIT_OBJECT_0)
	{
		LogSystemError(S_FATAL);
		FatalCleanupAndExit();
	}

	CloseHandle(ref);
	CloseHandle(iter);

	ArenaPopRecord(persistent);
}

void ds_TimeApiInit(struct arena *persistent)
{
	LARGE_INTEGER ret_val;
	QueryPerformanceCounter(&ret_val);
	u32 tmp;
	/* instruction fence */
	g_precision_timer.tsc_start = __rdtscp(&tmp);
	g_timer.ns_start = ret_val.QuadPart;
	QueryPerformanceFrequency(&ret_val);
	u64 freq = ret_val.QuadPart;

	g_timer.ns_resolution = NSEC_PER_SEC / freq;
	g_timer.ns_start *= g_timer.ns_resolution;
	g_timer.tsc_start = g_precision_timer.tsc_start;

	const u64 ms = 100;
	const u64 goal = g_timer.ns_start + NSEC_PER_SEC / (1000/ms);
	for (;;)
	{
		QueryPerformanceCounter(&ret_val);
		if (goal <= (u64) ret_val.QuadPart * g_timer.ns_resolution)
		{
			break;
		}
	}
	u64 end = __rdtsc();
	g_precision_timer.tsc_freq = (1000/ms) * (end-g_precision_timer.tsc_start);
	TscEstimateSkew(persistent);
}

#endif

u64 NsFromTsc(const u64 tsc)
{
	const u64 ns = NSEC_PER_SEC * SFromTsc(tsc);
	return ns;
}

u64 TscFromNs(const u64 ns)
{
	return ns * (f64) g_precision_timer.tsc_freq / NSEC_PER_SEC;
}

u64 ds_TimeNsFromTsc(const u64 tsc)
{
	ds_Assert(tsc >= g_timer.tsc_start);
	const u64 ns = NsFromTsc(tsc - g_timer.tsc_start);
	return ns;
}

u64 ds_TimeTscFromNs(const u64 ns)
{
	ds_Assert(ns >= g_timer.ns_start);
	const u64 cyc = TscFromNs(ns - g_timer.ns_start);
	return cyc;
}

u64 NsFromTscTruthSource(const u64 tsc, const u64 ns_truth, const u64 cc_truth)
{
	if (tsc >= cc_truth)
	{
		const u64 ns = NsFromTsc(tsc - cc_truth);
		return ns_truth + ns;
	}
	else
	{
		const u64 ns = NsFromTsc(cc_truth - tsc);
		return ns_truth - ns;
	}
}

u64 TscFromNsTruthSource(const u64 ns, const u64 ns_truth, const u64 cc_truth)
{
	if (ns >= ns_truth)
	{
		const u64 cc = TscFromNs(ns - ns_truth);
		return cc_truth + cc;
	}
	else
	{
		const u64 cc = TscFromNs(ns_truth - ns);
		return cc_truth - cc;
	}
}

u64 ds_TimeNsAtStart(void)
{
	return g_timer.ns_start;
}

u64 ds_TimeS(void)
{
	return ds_TimeNs() / NSEC_PER_SEC;
}

u64 ds_TimeMs(void)
{
	return ds_TimeNs() / NSEC_PER_MSEC;
}

u64 ds_TimeUs(void)
{
	return ds_TimeNs() / NSEC_PER_USEC;
}

u64 NsResolution(void)
{
	return g_timer.ns_resolution;
}

u64 TscFrequency(void)
{
	return g_precision_timer.tsc_freq;
}

f64 SFromTsc(const u64 ticks)
{
	return (f64) ticks / g_precision_timer.tsc_freq;
}

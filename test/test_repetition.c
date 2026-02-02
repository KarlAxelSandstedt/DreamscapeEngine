/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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
#include <string.h>

#include "test_local.h"

//#ifdef __linux__
//#include <linux/perf_event.h>
//#include <linux/hwBReakpoint.h>
//#include <sys/syscall.h>
//#include <sys/ioctl.h>
//#include <unistd.h>
//#include <errno.h>
//
//struct os_event_format
//{
//	u64 page_faults;
//	u64 branch_misses;
//	u64 frontend_stalled_cycles;
//	u64 backend_stalled_cycles;
//	u64 cycles;
//};
//
//static void os_get_counters(const struct repetition_tester *tester, const i32 group_fd, struct os_event_format *fm)
//{
//	struct 
//	{
//		u64 nr;
//		struct
//		{
//			u64 value;
//			u64 id;
//		} events[NUM_EVENTS];
//	} format;
//
//	while (read(group_fd, &format, sizeof(format)) == EOF)
//	{
//		fprintf(stdout, "Error %s:%d - Warning, EOF on page fault monitoring\n", __FILE__, __LINE__);
//	}
//	
//	static i32 pf = 0;
//	static i32 bm = 0;
//	static i32 fnt = 0;
//	static i32 bck = 0;
//	static i32 cyc = 0;
//
//	assert(format.nr == tester->event_count);
//	for (i32 i = 0; i < tester->event_count; ++i)
//	{
//		if (format.events[i].id == tester->pf_id)
//		{
//			fm->page_faults = format.events[i].value;
//			pf++;
//		}
//		else if (format.events[i].id == tester->bm_id)
//		{
//			fm->branch_misses = format.events[i].value;
//			bm++;
//		}
//		else if (format.events[i].id == tester->fnt_id)
//		{
//			fm->frontend_stalled_cycles = format.events[i].value;
//			fnt++;
//		}
//		else if (format.events[i].id == tester->bck_id)
//		{
//			fm->backend_stalled_cycles = format.events[i].value;
//			bck++;
//		}
//		else if (format.events[i].id == tester->cyc_id)
//		{
//			fm->cycles = format.events[i].value;
//			cyc++;
//		}
//		else
//		{
//			assert(0 && "Should not happen\n");
//		}
//	}
//
//	assert(pf <= 1 && bm <= 1 && fnt <= 1 && bck <= 1 && cyc <= 1);
//	pf = 0;
//	bm = 0;
//	fnt = 0;
//	bck = 0;
//	cyc = 0;
//}
//
//
//static void os_enable_counters(struct repetition_tester *tester)
//{
//	//ioctl(tester->pf_fd, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
//	ioctl(tester->pf_fd, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);
//}
//
//static void os_disable_counters(struct repetition_tester *tester)
//{
//	ioctl(tester->pf_fd, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
//}
//
//static void os_init_performance_events(struct repetition_tester *tester)
//{
//	const i32 pid = 0; /* this process on any cpu */
//	const i32 cpu = -1;
//	i32 group_fd = -1;		/* group leader for performance events,
//					   i.e. os makes sure all events in group
//					   are counted at for the same set of inst. */
//	const u32 flags = 0;
//	struct perf_event_attr pf_attr = { 0 };
//	{
//		pf_attr.type = PERF_TYPE_SOFTWARE;
//		pf_attr.config = PERF_COUNT_SW_PAGE_FAULTS;
//		pf_attr.size = sizeof(struct perf_event_attr);
//		//pf_attr.sample_period = PAGE_FAULT_SAMPLING_PERIOD;	/* event granularity of 1, (we get every page fault immediately!) */
//		pf_attr.disabled = 1;
//		pf_attr.read_format = PERF_FORMAT_ID | PERF_FORMAT_GROUP;
//	}
//
//	tester->pf_fd = syscall(SYS_perf_event_open, &pf_attr, pid, cpu, group_fd, flags);
//	group_fd = tester->pf_fd;
//	if (tester->pf_fd == -1)
//	{
//		if (errno == ENOENT)
//		{
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, "PERF_COUNT_SW_PAGE_FAULTS unsupported");
//			tester->pf_id = SIZE_MAX;
//		}
//		else
//		{
//			char buf[256];
//			strerror_r(errno, buf, sizeof(buf));
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, buf);
//			exit(1);
//		}
//	}
//	else
//	{
//		tester->event_count += 1;
//	    	ioctl(tester->pf_fd, PERF_EVENT_IOC_ID, &tester->pf_id);
//	}
//
//	struct perf_event_attr bm_attr = { 0 };
//	{
//		bm_attr.type = PERF_TYPE_HARDWARE;
//		bm_attr.config = PERF_COUNT_HW_BRANCH_MISSES;
//		bm_attr.size = sizeof(struct perf_event_attr);
//		//bm_attr.sample_period = BRANCH_MISSES_SAMPLING_PERIOD;
//		bm_attr.disabled = 1;
//		bm_attr.read_format = PERF_FORMAT_ID | PERF_FORMAT_GROUP;
//	}
//
//	tester->bm_fd = syscall(SYS_perf_event_open, &bm_attr, pid, cpu, group_fd, flags);
//	if (tester->bm_fd == -1)
//	{
//		if (errno == ENOENT)
//		{
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, "PERF_COUNT_HW_BRANCH_MISSES unsupported");
//			tester->bm_id = SIZE_MAX;
//		}
//		else
//		{
//			char buf[256];
//			strerror_r(errno, buf, sizeof(buf));
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, buf);
//			exit(1);
//		}
//	}
//	else
//	{
//		tester->event_count += 1;
//	    	ioctl(tester->bm_fd, PERF_EVENT_IOC_ID, &tester->bm_id);
//	}
//
//	struct perf_event_attr cyc_attr = { 0 };
//	{
//		cyc_attr.type = PERF_TYPE_HARDWARE;
//		cyc_attr.config = PERF_COUNT_HW_REF_CPU_CYCLES;
//		cyc_attr.size = sizeof(struct perf_event_attr);
//		//cyc_attr.sample_period = BRANCH_MISSES_SAMPLING_PERIOD;
//		cyc_attr.disabled = 1;
//		cyc_attr.read_format = PERF_FORMAT_ID | PERF_FORMAT_GROUP;
//	}
//
//	tester->cyc_fd = syscall(SYS_perf_event_open, &cyc_attr, pid, cpu, group_fd, flags);
//	if (tester->cyc_fd == -1)
//	{
//		if (errno == ENOENT)
//		{
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, "PERF_COUNT_HW_CPU_CYCLES unsupported");
//			tester->cyc_id = SIZE_MAX;
//		}
//		else
//		{
//			char buf[256];
//			strerror_r(errno, buf, sizeof(buf));
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, buf);
//			exit(1);
//		}
//	}
//	else
//	{
//		tester->event_count += 1;
//	    	ioctl(tester->cyc_fd, PERF_EVENT_IOC_ID, &tester->cyc_id);
//	}
//
//	struct perf_event_attr bck_attr = { 0 };
//	{
//		bck_attr.type = PERF_TYPE_HARDWARE;
//		bck_attr.config = PERF_COUNT_HW_STALLED_CYCLES_BACKEND;
//		bck_attr.size = sizeof(struct perf_event_attr);
//		//bck_attr.sample_period = BRANCH_MISSES_SAMPLING_PERIOD;
//		bck_attr.disabled = 1;
//		bck_attr.read_format = PERF_FORMAT_ID | PERF_FORMAT_GROUP;
//	}
//
//	tester->bck_fd = syscall(SYS_perf_event_open, &bck_attr, pid, cpu, group_fd, flags);
//	if (tester->bck_fd == -1)
//	{
//		if (errno == ENOENT)
//		{
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, "PERF_COUNT_HW_STALLED_CYCLES_BACKEND unsupported");
//			tester->bck_id = SIZE_MAX;
//		}
//		else
//		{
//			char buf[256];
//			strerror_r(errno, buf, sizeof(buf));
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, buf);
//			exit(1);
//		}
//	}
//	else
//	{
//		tester->event_count += 1;
//	    	ioctl(tester->bck_fd, PERF_EVENT_IOC_ID, &tester->bck_id);
//	}
//
//	struct perf_event_attr fnt_attr = { 0 };
//	{
//		fnt_attr.type = PERF_TYPE_HARDWARE;
//		fnt_attr.config = PERF_COUNT_HW_STALLED_CYCLES_FRONTEND;
//		fnt_attr.size = sizeof(struct perf_event_attr);
//		//fnt_attr.sample_period = BRANCH_MISSES_SAMPLING_PERIOD;
//		fnt_attr.disabled = 1;
//		fnt_attr.read_format = PERF_FORMAT_ID | PERF_FORMAT_GROUP;
//	}
//
//	tester->fnt_fd = syscall(SYS_perf_event_open, &fnt_attr, pid, cpu, group_fd, flags);
//	if (tester->fnt_fd == -1)
//	{
//		if (errno == ENOENT)
//		{
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, "PERF_COUNT_HW_STALLED_CYCLES_FRONTEND unsupported");
//			tester->fnt_id = SIZE_MAX;
//		}
//		else
//		{
//			char buf[256];
//			strerror_r(errno, buf, sizeof(buf));
//			fprintf(stderr, "%s:%u - %s\n", __FILE__, __LINE__, buf);
//			exit(1);
//		}
//	}
//	else
//	{
//		tester->event_count += 1;
//	    	ioctl(tester->fnt_fd, PERF_EVENT_IOC_ID, &tester->fnt_id);
//	}
//}
//#endif

void repetition_error(struct repetition_tester *tester, const char *file, const u32 line, const char *msg)
{
	fprintf(stderr, "Error %s:%d - %s\n", file, line, msg);
	tester->state = RT_ERROR;
}

i32 rt_is_testing(struct repetition_tester *tester)
{
	i32 status = 0;

	if (tester->state == RT_TESTING)
	{
		if (tester->enter_count)
		{
			if (tester->enter_count != tester->exit_count)
			{
				repetition_error(tester, __FILE__, __LINE__, "Timed regions in test in not enclosed properly");
			}

			if (tester->bytes_in_current_test != tester->bytes_to_process)
			{
				repetition_error(tester, __FILE__, __LINE__, "Proper amount of bytes not processed");
			}

			if (tester->state == RT_TESTING)
			{
				if (tester->tsc_iteration_max < tester->tsc_in_current_test)
				{
					tester->tsc_iteration_max = tester->tsc_in_current_test;
					tester->page_faults_max_time = tester->page_faults_in_current_test;
					tester->branch_misses_max_time = tester->branch_misses_in_current_test;
					tester->backend_stalled_cycles_max_time = tester->backend_stalled_cycles_in_current_test;
					tester->frontend_stalled_cycles_max_time = tester->frontend_stalled_cycles_in_current_test;
					tester->cycles_max_time = tester->cycles_in_current_test;
				}

				u64 tsc = rdtsc();
				if (tester->tsc_in_current_test < tester->tsc_iteration_min)
				{
					tester->tsc_start = tsc;
					tester->tsc_iteration_min = tester->tsc_in_current_test;
					tester->page_faults_min_time = tester->page_faults_in_current_test;
					tester->branch_misses_min_time = tester->branch_misses_in_current_test;
					tester->backend_stalled_cycles_min_time = tester->backend_stalled_cycles_in_current_test;
					tester->frontend_stalled_cycles_min_time = tester->frontend_stalled_cycles_in_current_test;
					tester->cycles_min_time = tester->cycles_in_current_test;
				}

				tester->bytes += tester->bytes_to_process;
				tester->time += tester->tsc_in_current_test;
				tester->page_faults += tester->page_faults_in_current_test;
				tester->branch_misses += tester->branch_misses_in_current_test;
				tester->backend_stalled_cycles += tester->backend_stalled_cycles_in_current_test;
				tester->frontend_stalled_cycles += tester->frontend_stalled_cycles_in_current_test;
				tester->cycles += tester->cycles_in_current_test;
				tester->enter_count = 0;
				tester->exit_count = 0;
				tester->test_count += 1;
				tester->tsc_in_current_test = 0;
				tester->bytes_in_current_test = tester->bytes_to_process;
				tester->page_faults_in_current_test = 0;
				tester->backend_stalled_cycles_in_current_test = 0;
				tester->frontend_stalled_cycles_in_current_test = 0;
				tester->cycles_in_current_test = 0;
				tester->branch_misses_in_current_test = 0;

				if (tester->tsc_retry_max < tsc - tester->tsc_start)
				{
					tester->state = RT_COMPLETED;
				}
				else
				{
					status = 1;
				}

			}
		}
		else
		{
			repetition_error(tester, __FILE__, __LINE__, "No timed region in test");
		}
	}	

	return status;
}

void rt_wave(struct repetition_tester *tester, const u64 bytes_to_process, const u64 tsc_freq, const u64 tsc_retry_max, const u32 print)
{
	if (tester->state == RT_UNINITIALIZED)
	{
		tester->bytes_to_process = bytes_to_process;
		tester->tsc_freq = tsc_freq;
		tester->print = print;
		tester->tsc_iteration_min = UINT64_MAX;
		//os_init_performance_events(tester);
	}
	else if (tester->state == RT_COMPLETED)
	{
		if (bytes_to_process != tester->bytes_to_process)
		{
			repetition_error(tester, __FILE__, __LINE__, "Expected bytes to process changed");
			return;
		}

		if (tsc_freq != tester->tsc_freq)
		{
			repetition_error(tester, __FILE__, __LINE__, "Expected cpu frequency changed");
			return;
		}
	}
		
	tester->state = RT_TESTING;
	tester->tsc_retry_max = tsc_retry_max;
	tester->tsc_start = rdtsc();
	tester->enter_count = 1;
	tester->exit_count = 1;
	tester->bytes_in_current_test = bytes_to_process;
	tester->tsc_in_current_test = 0;
	tester->page_faults_in_current_test = 0;
	tester->branch_misses_in_current_test = 0;
	tester->frontend_stalled_cycles_in_current_test = 0;
	tester->backend_stalled_cycles_in_current_test = 0;
	tester->cycles_in_current_test = 0;
}

void rt_begin_time(struct repetition_tester *tester)
{
	//os_enable_counters(tester);
	//struct os_event_format format = { 0 };
	//os_get_counters(tester, tester->pf_fd, &format);
	//tester->page_faults_in_current_test -= format.page_faults;
	//tester->branch_misses_in_current_test -= format.branch_misses;
	//tester->frontend_stalled_cycles_in_current_test -= format.frontend_stalled_cycles;
	//tester->backend_stalled_cycles_in_current_test -= format.backend_stalled_cycles;
	//tester->cycles_in_current_test -= format.cycles;
	tester->enter_count += 1;
	tester->tsc_in_current_test -= rdtsc();
}

void rt_end_time(struct repetition_tester *tester)
{
	tester->tsc_in_current_test += rdtsc();
	tester->exit_count += 1;
	//struct os_event_format format = { 0 };
	//os_get_counters(tester, tester->pf_fd, &format);
	//tester->page_faults_in_current_test += format.page_faults;
	//tester->branch_misses_in_current_test += format.branch_misses;
	//tester->frontend_stalled_cycles_in_current_test += format.frontend_stalled_cycles;
	//tester->backend_stalled_cycles_in_current_test += format.backend_stalled_cycles;
	//tester->cycles_in_current_test += format.cycles;
	//os_disable_counters(tester);
}

void rt_print_statistics(const struct repetition_tester *tester, FILE *file)
{
	const f64 ns_max = (f64) ns_from_tsc(tester->tsc_iteration_max);
	const f64 ns_min = (f64) ns_from_tsc(tester->tsc_iteration_min);
	const f64 ns_avg = (f64) ns_from_tsc(tester->time / tester->test_count);

	const f64 ms_max = ns_max / (1000.0f * 1000.0f); 
	const f64 ms_min = ns_min / (1000.0f * 1000.0f);
	const f64 ms_avg = ns_avg / (1000.0f * 1000.0f);	

	/* B/s */
	const f64 thr_max = ((f64) tester->bytes_to_process) / (ns_max / (1000.0f * 1000.0f * 1000.0f));
	const f64 thr_min = ((f64) tester->bytes_to_process) / (ns_min / (1000.0f * 1000.0f * 1000.0f)); 
	const f64 thr_avg = ((f64) tester->bytes_to_process) / (ns_avg / (1000.0f * 1000.0f * 1000.0f));

	fprintf(file, "min: [%.5fms] %.3fGB/s\n", ms_min, thr_min / (1024.0f * 1024.0f * 1024.0f));
	fprintf(file, "max: [%.5fms] %.3fGB/s\n", ms_max, thr_max / (1024.0f * 1024.0f * 1024.0f)); 
	fprintf(file, "avg: [%.5fms] %.3fGB/s\n", ms_avg, thr_avg / (1024.0f * 1024.0f * 1024.0f)); 
	fprintf(file, "min Cycles/B: [%.5fCyc/B]\n", (f64) tester->tsc_freq / thr_min); 

	//const f64 pf_mi = tester->page_faults_min_time;
	//const f64 pf_ma = tester->page_faults_max_time;
	//const f64 pf_av = (f64) tester->page_faults / tester->test_count;

	//const f64 kb_pf_mi = (f64) tester->bytes_to_process / (1024.0 * pf_mi);
	//const f64 kb_pf_ma = (f64) tester->bytes_to_process / (1024.0 * pf_ma);
	//const f64 kb_pf_av = (f64) tester->bytes_to_process / (1024.0 * pf_av);

	//const u64 bm_mi = tester->branch_misses_min_time;
	//const u64 bm_ma = tester->branch_misses_max_time;
	//const u64 bm_av = tester->branch_misses / tester->test_count;

	//const u64 fnt_mi = tester->frontend_stalled_cycles_min_time;
	//const u64 fnt_ma = tester->frontend_stalled_cycles_max_time;
	//const u64 fnt_av = tester->frontend_stalled_cycles / tester->test_count;

	//const u64 bck_mi = tester->backend_stalled_cycles_min_time;
	//const u64 bck_ma = tester->backend_stalled_cycles_max_time;
	//const u64 bck_av = tester->backend_stalled_cycles / tester->test_count;

	//f64 fnt_cyc_mi, fnt_cyc_ma, fnt_cyc_av, bck_cyc_mi, bck_cyc_ma, bck_cyc_av;
	//if (tester->cyc_fd != -1)
	//{
	//	fnt_cyc_mi = (f64) fnt_mi / tester->cycles_min_time;
	//	fnt_cyc_ma = (f64) fnt_ma / tester->cycles_max_time;
	//	fnt_cyc_av = (f64) fnt_av / tester->cycles;

	//	bck_cyc_mi = (f64) bck_mi / tester->cycles_min_time;
	//	bck_cyc_ma = (f64) bck_ma / tester->cycles_max_time;
	//	bck_cyc_av = (f64) bck_av / tester->cycles;
	//}
	//else
	//{
	//	fnt_cyc_mi = (f64) fnt_mi / tester->tsc_iteration_min;
	//	fnt_cyc_ma = (f64) fnt_ma / tester->tsc_iteration_max;
	//	fnt_cyc_av = (f64) fnt_av / tester->time;

	//	bck_cyc_mi = (f64) bck_mi / tester->tsc_iteration_min;
	//	bck_cyc_ma = (f64) bck_ma / tester->tsc_iteration_max;
	//	bck_cyc_av = (f64) bck_av / tester->time;              
	//}

	//fprintf(file, "min: [%.5fms] %.3fGB/s, PF: [%.4f] %.4fkB/PF, BM: %lu, FNT_S (%lu) [%.4f], BCK_S (%lu) [%.4f]\n", ms_ma, thr_mi, pf_mi, kb_pf_mi, bm_mi, fnt_mi, fnt_cyc_mi, bck_mi, bck_cyc_mi);
	//fprintf(file, "max: [%.5fms] %.3fGB/s, PF: [%.4f] %.4fkB/PF, BM: %lu, FNT_S (%lu) [%.4f], BCK_S (%lu) [%.4f]\n", ms_mi, thr_ma, pf_ma, kb_pf_ma, bm_ma, fnt_ma, fnt_cyc_ma, bck_ma, bck_cyc_ma); 
	//fprintf(file, "avg: [%.5fms] %.3fGB/s, PF: [%.4f] %.4fkB/PF, BM: %lu, FNT_S (%lu) [%.4f], BCK_S (%lu) [%.4f]\n", ms_av, thr_av, pf_av, kb_pf_av, bm_av, fnt_av, fnt_cyc_av, bck_av, bck_cyc_av); 
	//fprintf(file, "min Cycles/B: [%.5fCyc/B]\n", (f64) tester->tsc_freq / (thr_mi * 1024*1024*1024)); 
}

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

#ifndef __DREAMSCAPE_THREAD_H__
#define __DREAMSCAPE_THREAD_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

#if __DS_PLATFORM__ == __DS_LINUX__ ||__DS_PLATFORM__ == __DS_WEB__

#include <pthread.h>
typedef pid_t	tid;

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <windows.h>
typedef DWORD	tid;

#endif

typedef struct dsThread dsThread;
extern dsThreadLocal dsThread *g_tl_self;

/* Alloc and initiate master thread information; should only be called once! */
void 		ds_ThreadMasterInit(struct arena *mem);
/* Alloc and initiate thread. On success, return valid address. On failure, Fatally cleanup and exit */
dsThread *  ds_ThreadClone(struct arena *mem, void (*start)(dsThread *), void *args, const u64 stack_size);
/* Exit calling thread */
void		ds_ThreadExit(void);
/* Wait for given thread to finish execution */
void 		ds_ThreadWait(const dsThread *thr);
/* Get return value address */
void *		ds_ThreadReturnValue(const dsThread *thr);
/* Get return value size */
u64		    ds_ThreadReturnValueSize(const dsThread *thr);
/* Retrieve thread function arguments */
void *  	ds_ThreadArguments(const dsThread *thr);
/* Return thread id */
tid		    ds_ThreadTid(const dsThread *thr);
/* Return thread tid of caller */
tid 		ds_ThreadSelfTid(void);
/* Return index of thread (each created thread increments the global index counter) */
u32		    ds_ThreadIndex(const dsThread *thr);
/* Return index of caller */ 
u32		    ds_ThreadSelfIndex(void);

#ifdef __cplusplus
} 
#endif

#endif

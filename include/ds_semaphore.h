/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

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
ds_semaphore.h
==============
Counting semaphores for threads of one process (POSIX sem_t, Windows semaphore objects). Every OS error is
fatal, so the functions return only on success.
*/

#ifndef __DREAMSCAPE_SEMAPHORE_H__
#define __DREAMSCAPE_SEMAPHORE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_define.h"
#include "ds_types.h"

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

	#include <semaphore.h>
	typedef sem_t semaphore;

#elif __DS_PLATFORM__ == __DS_WIN64__

	#include <windows.h>
	typedef HANDLE semaphore;

#endif

/* Initialize sem (not yet initialized) to val (<= 2^31 - 1). */
void 	SemaphoreInit(semaphore *sem, const u32 val);
/* Destroy sem; no thread may be waiting on it. */
void 	SemaphoreDestroy(semaphore *sem);
/* Increment sem, waking one waiter if any. The count stays <= 2^31 - 1. */
void 	SemaphorePost(semaphore *sem);
/* Block until sem is positive, then decrement it. */
void 	SemaphoreWait(semaphore *sem);
/* Decrement sem and return 1 if it is positive; else return 0 at once (Linux: also 0 if a signal interrupts). */
u32 	SemaphoreTryWait(semaphore *sem);


#ifdef __cplusplus
} 
#endif

#endif

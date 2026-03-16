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

/* Initiate the semaphore with a given value: NOTE: initializing an already initiated semaphore is UB */
void 	SemaphoreInit(semaphore *sem, const u32 val); 
/* Destroy the given semaphore. NOTE: destroying a semaphore at which threads are waiting is UB */
void 	SemaphoreDestroy(semaphore *sem);
/* increment semaphore */
void 	SemaphorePost(semaphore *sem);	
/* return 1 on successful lock aquisition, 0 otherwise. */
u32 	SemaphoreWait(semaphore *sem);	
/* return 1 on successful lock aquisition, 0 otherwise. */
u32 	SemaphoreTryWait(semaphore *sem);	


#ifdef __cplusplus
} 
#endif

#endif

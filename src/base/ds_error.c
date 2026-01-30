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

#include "ds_base.h"

static u32 a_fatal_cleanup_initiated = 0;

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if __DS_PLATFORM__ == __DS_LINUX__
#include <execinfo.h>
#define STACKTRACE_BUFSIZE	128
#else
#include <emscripten/emscripten.h>
#endif

void FatalCleanupAndExit(void)
{
	u32 desired = 0;
	// TODO Somehow stop/force all threads to not continue / shut down program until cleanup is complete 
	
	if (AtomicCompareExchangeAcq32(&a_fatal_cleanup_initiated, &desired, 1))
	{

#if __DS_PLATFORM__ == __DS_LINUX__
		void *buf[STACKTRACE_BUFSIZE];
		char **strings;
		const i32 count = backtrace(buf, STACKTRACE_BUFSIZE);
		strings = backtrace_symbols(buf, count);
		if (!strings)
		{
			LogSystemError(S_FATAL);
		}
		else
		{
			fprintf(stderr, "================== STACKTACE ==================\n");
			for (i32 i = 0; i < count; ++i)
			{
				fprintf(stderr, "(%i) %s\n", i, strings[i]);
			}
			free(strings);
		}
#else
		const int len = emscripten_get_callstack(EM_LOG_C_STACK | EM_LOG_JS_STACK, 0, 0);
		char *callstack = malloc(len + 256);
		emscripten_get_callstack(EM_LOG_C_STACK | EM_LOG_JS_STACK, callstack, 0);
		{
			fprintf(stderr, "================== STACKTACE ==================\n%s", callstack);
		}
		free(callstack);
#endif
		LogShutdown();
		exit(1);
	}

	//TODO spin threads that di not acquire lock or exit or something...
}

utf8 Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 code)
{
	ds_Assert(bufsize > 0);
	utf8 err_str = 
	{
		.len = 0,
		.buf = buf,
		.size = bufsize,
	};

	const u32 status = strerror_r(code, (char *) buf, bufsize);
	if (status != 0)
	{
		if (status == EINVAL)
		{
			LogSystemErrorCode(S_ERROR, status);
		}
		else if (status == ERANGE)
		{
			ds_Assert(0 && "increase system error string buffer size!");
		}

		return Utf8Empty();
	}

	err_str.len = strnlen((char *) err_str.buf, ERROR_BUFSIZE);
	if (err_str.len == ERROR_BUFSIZE)
	{
		Log(T_SYSTEM, S_ERROR, "strnlen failed to determine string length in %s, most likely due to no null-termination? Fix.", __func__);
		return Utf8Empty();
	}

	return err_str;
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include "ds_string.h"

#include <minidumpapiset.h>

void FatalCleanupAndExit()
{
	u32 desired = 0;
	// TODO Somehow stop/force all threads to not continue / shut down program until cleanup is complete 
	
	if (AtomicCompareExchangeAcq32(&a_fatal_cleanup_initiated, &desired, 1))
	{
    		SYSTEMTIME local_time;
    		GetLocalTime(&local_time);

		struct arena tmp = ArenaAlloc1MB();
		const utf8 utf8_filename = Utf8Format(&tmp, "%s_%s_latest.dmp"
				, DS_EXECUTABLE_CSTR
				, DS_VERSION_CSTR);
		//const utf8 utf8_filename = Utf8Format(&tmp, "%s_%s_%u%u%u_%u%u%u.dmp", 
    		//           "engine_sandbox",
		//	   "0_1", 
    		//           local_time.wYear,
		//	   local_time.wMonth,
		//	   local_time.wDay, 
    		//           local_time.wHour, 
		//	   local_time.wMinute,
		//	   local_time.wSecond);
    		const char *filename = CstrUtf8(&tmp, utf8_filename);
		struct file dump = FileNull();
		if (FileTryCreateAtCwd(&tmp, &dump, filename, FILE_TRUNCATE) == FS_SUCCESS)
		{
			if (!MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), dump.handle, MiniDumpWithFullMemory, NULL, NULL, NULL))
			{
				LogSystemError(S_ERROR);
			}
			
			FileClose(&dump);
		}
		ArenaFree1MB(&tmp);

		LogShutdown();
		exit(0);
	}

	//TODO spin threads that di not acquire lock or exit or something...
}

utf8 Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 garbage)
{
	utf8 err_str =
	{
		.buf = buf,
		.size = bufsize,
		.len = 0,
	};

	const DWORD code = GetLastError();
	err_str.len  = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM |FORMAT_MESSAGE_IGNORE_INSERTS,
					 NULL,
					 code,
					 0,
					 buf,
					 (DWORD) bufsize,
					 NULL);

	return (err_str.len > 0) ? err_str : Utf8Empty();
}

#endif

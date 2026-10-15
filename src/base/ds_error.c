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

#include <stdio.h>

#include "ds_base.h"
#include "ds_platform.h"

#ifdef DS_ASSERT_DEBUG
void ds_AssertFailed(const char *file, const u32 line, const char *func, const char *format, ...)
{
	if (format == NULL)
	{
		Log(T_ASSERT, S_FATAL, "ds_Assertion failed at %s:%u in function %s", file, line, func);
		fflush(stdout);
		return;
	}

	u8 buf[LOG_MAX_MESSAGE_SIZE];
	u64 reqsize;
	va_list args;
	va_start(args, format);
	const utf8 msg = Utf8FormatBufferedVariadic(&reqsize, buf, LOG_MAX_MESSAGE_SIZE, format, args);
	va_end(args);
	Log(T_ASSERT, S_FATAL, "ds_Assertion failed at %s:%u in function %s - %k", file, line, func, &msg);
	/* the caller's Breakpoint kills the process without a debugger: get the message out first */
	fflush(stdout);
}
#endif

static u32 a_fatal_cleanup_initiated = 0;
/* set on the thread doing the cleanup: a fatal error during the cleanup itself exits at once */
static ds_ThreadLocal u32 tl_fatal_cleanup_running = 0;

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
	if (tl_fatal_cleanup_running)
	{
		exit(1);
	}

	u32 desired = 0;
	if (AtomicCompareExchangeAcqRlx32(&a_fatal_cleanup_initiated, &desired, 1))
	{
		tl_fatal_cleanup_running = 1;
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
			fprintf(stderr, "================== STACKTRACE ==================\n");
			for (i32 i = 0; i < count; ++i)
			{
				fprintf(stderr, "(%i) %s\n", i, strings[i]);
			}
			free(strings);
		}
#else
		/* the first call returns the bytes needed (with the '\0'); the stack may grow by the second call */
		const int len = emscripten_get_callstack(EM_LOG_C_STACK | EM_LOG_JS_STACK, NULL, 0) + 256;
		char *callstack = malloc(len);
		if (callstack)
		{
			emscripten_get_callstack(EM_LOG_C_STACK | EM_LOG_JS_STACK, callstack, len);
			fprintf(stderr, "================== STACKTRACE ==================\n%s", callstack);
			free(callstack);
		}
#endif
		LogShutdown();
		/* break only now: without a debugger the breakpoint kills the process, unflushed output is lost */
		fflush(stdout);
		Breakpoint(1);
		exit(1);
	}

	/* another thread is cleaning up and will exit the process: never return to the failed caller */
	for (;;)
	{
		ds_ThreadYield();
	}
}

#if __DS_PLATFORM__ == __DS_LINUX__
utf8 Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 code)
{
	ds_Assert(bufsize > 0);

    char tmpbuf[ERROR_BUFSIZE];
	const char *str = strerror_r(code, (char *) tmpbuf, ERROR_BUFSIZE);
    const utf8 err_str = Utf8CstrBuffered(buf, bufsize, str);
	if (err_str.len == 0)
	{
		Log(T_SYSTEM, S_ERROR, "%s: no text for error code %u (or it does not fit %u bytes)", __func__, code, bufsize);
	}

	return err_str;
}
#else
utf8 Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 code)
{
    /* Note: Emscripten seems to comply with XSI, so need separate function here */
	ds_Assert(bufsize > 0);

    char tmpbuf[ERROR_BUFSIZE];
    char *str = tmpbuf;
	if (strerror_r(code, (char *) tmpbuf, ERROR_BUFSIZE) != 0)
	{
        str = "";
		Log(T_SYSTEM, S_ERROR, "%s: strerror_r has no text for error code %u", __func__, code);
	}

    const utf8 err_str = Utf8CstrBuffered(buf, bufsize, str);

	return err_str;
}
#endif

#elif __DS_PLATFORM__ == __DS_WIN64__

#include "ds_string.h"

#include <minidumpapiset.h>

void FatalCleanupAndExit()
{
	if (tl_fatal_cleanup_running)
	{
		exit(1);
	}

	u32 desired = 0;
	if (AtomicCompareExchangeAcqRlx32(&a_fatal_cleanup_initiated, &desired, 1))
	{
		tl_fatal_cleanup_running = 1;

		/*
		 * a compile-time name opened with CreateFileA (process working directory): no scratch arena and no
		 * engine state, so the dump is written even with the scratch arenas exhausted or before any init
		 */
		HANDLE dump = CreateFileA(DS_EXECUTABLE_CSTR "_" DS_VERSION_CSTR "_latest.dmp", GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
		if (dump == INVALID_HANDLE_VALUE)
		{
			LogSystemError(S_ERROR);
		}
		else
		{
			if (!MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), dump, MiniDumpWithDataSegs | MiniDumpWithIndirectlyReferencedMemory, NULL, NULL, NULL))
			{
				LogSystemError(S_ERROR);
			}
			CloseHandle(dump);
		}

		LogShutdown();
		/* without a debugger the breakpoint kills the process, unflushed output is lost */
		fflush(stdout);
		Breakpoint(1);
		exit(1);
	}

	/* another thread is cleaning up and will exit the process: never return to the failed caller */
	for (;;)
	{
		ds_ThreadYield();
	}
}

utf8 Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 code)
{
	ds_Assert(bufsize > 0);

	utf8 err_str =
	{
		.buf = buf,
		.size = 0,
		.len = 0,
	};

	/* system messages are ASCII: byte count == codepoint count */
	err_str.len  = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM |FORMAT_MESSAGE_IGNORE_INSERTS,
					 NULL,
					 (DWORD) code,
					 0,
					 (char *) buf,
					 (DWORD) bufsize,
					 NULL);

	/* drop the trailing "\r\n" */
	while (err_str.len > 0 && (buf[err_str.len - 1] == '\n' || buf[err_str.len - 1] == '\r'))
	{
		err_str.len -= 1;
	}
	err_str.size = err_str.len;

	return (err_str.len > 0) ? err_str : Utf8Empty();
}

#endif

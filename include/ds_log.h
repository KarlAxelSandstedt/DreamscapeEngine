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

/*
ds_log.h
========
Thread-safe logging: each message goes to stdout at once and to the log file in batches. API turned
on/off by DS_LOG macro.

Description:
    Log(system, severity, format, ...) formats with the Utf8Format rules (ds_string.h: %s, %k for a utf8
    pointer, %i/%u/%li/%lu, %f, %p) and prints the line

        [seconds.mmm s] System severity - Thread <index>: <text>

Usage:
    Log(T_PHYSICS, S_WARNING, "body %u: %k", index, &name);
    LogString(T_SYSTEM, S_NOTE, "plain text");              (no format arguments)
    LogSystemError(S_ERROR);                                (text of the last system error: errno, Windows GetLastError())
    LogSystemErrorCode(S_ERROR, code);                      (text of a given errno / Windows error code)

Internals:
    A ticket factory hands out the LOG_MAX_MESSAGES slots as a ring. A thread that finds no free slot becomes
    the disk writer (a_writing_to_disk) and writes the completed slots in order until it reaches one still
    being formatted.
*/

#ifndef __DREAMSCAPE_LOG_H__
#define __DREAMSCAPE_LOG_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdarg.h>

#include "ds_allocator.h"
#include "ds_error.h"

#define LOG_MAX_MESSAGES		512
#define LOG_MAX_MESSAGE_SIZE 	512
#define ERROR_BUFSIZE			512				

/* 
 * Setup Log memory and open filepath (truncate on existence). If filepath is NULL, no file is
 * created. 
 */
void 	LogInit(struct arena *mem, const char *filepath);
/* 
 * Write all pending messages to the file and close it; later messages are dropped.
 */
void 	LogShutdown(void);
/*
 * LogWriteMessage() - Generate a formatted string from the input string and the following arguments
 * and write to the Logger.
 */
void 	LogWriteMessage(const enum system_id system, const enum severity_id severity, const char *format, ... );

#ifdef DS_LOG

/* must not have side-effects */
#define LogString(system, severity, msg, ...)		LogWriteMessage(system, severity, msg)
#define Log(system, severity, msg, ...)			    LogWriteMessage(system, severity, msg, __VA_ARGS__)

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

	#define LogSystemError(severity)		_LogSystemErrorCode(severity, errno, __func__, __FILE__, __LINE__)
	#define LogSystemErrorCode(severity, code)	_LogSystemErrorCode(severity, code, __func__, __FILE__, __LINE__)
	#define _LogSystemErrorCode(severity, code, func, file, line)						\
	{													\
		u8 _err_buf[ERROR_BUFSIZE];									\
		const utf8 _err_str = Utf8SystemErrorCodeStringBuffered(_err_buf, ERROR_BUFSIZE, code);	\
		Log(T_SYSTEM, severity, "At %s:%u in function %s - %k", file, line, func, &_err_str);		\
	}

#elif __DS_PLATFORM__ == __DS_WIN64__

	/* GetLastError() is evaluated at the call, before anything can overwrite it */
	#define LogSystemError(severity)		_LogSystemErrorCode(severity, GetLastError(), __func__, __FILE__, __LINE__)
	#define LogSystemErrorCode(severity, code)	_LogSystemErrorCode(severity, code, __func__, __FILE__, __LINE__)
	#define _LogSystemErrorCode(severity, code, func, file, line)						\
	{													\
		u8 _err_buf[ERROR_BUFSIZE];									\
		const utf8 _err_str = Utf8SystemErrorCodeStringBuffered(_err_buf, ERROR_BUFSIZE, code);	\
		Log(T_SYSTEM, severity, "At %s:%u in function %s - %k", file, line, func, &_err_str);		\
	}

#endif

#else

#define LogString(system, severity, msg, ...)
#define Log(system, severity, msg, ...)

#define LogSystemError(severity)		
#define LogSystemErrorCode(severity, code)

#endif

#ifdef __cplusplus
} 
#endif

#endif

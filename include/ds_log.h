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

#ifndef __DREAMSCAPE_LOG_H__
#define __DREAMSCAPE_LOG_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdarg.h>

#include "ds_allocator.h"
#include "ds_error.h"

#define LOG_MAX_MESSAGES		512
#define LOG_MAX_MESSAGE_SIZE 		512
#define ERROR_BUFSIZE			512				

void 	LogInit(struct arena *mem, const char *filepath);
void 	LogShutdown(void);


/**
 * LogWriteMessage() - Generate a formatted string from the input string and the following arguments and write to the Logger.
 */
void 	LogWriteMessage(const enum system_id system, const enum severity_id severity, const char *format, ... );

#ifdef DS_LOG

#define LogString(system, severity, msg, ...)		LogWriteMessage(system, severity, msg)
#define Log(system, severity, msg, ...)			    LogWriteMessage(system, severity, msg, __VA_ARGS__)

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

	#define ERROR_BUFSIZE	512
	#define LogSystemError(severity)		_LogSystemErrorCode(severity, errno, __func__, __FILE__, __LINE__)
	#define LogSystemErrorCode(severity, code)	_LogSystemErrorCode(severity, code, __func__, __FILE__, __LINE__)
	#define _LogSystemErrorCode(severity, code, func, file, line)						\
	{													\
		u8 _err_buf[ERROR_BUFSIZE];									\
		const utf8 _err_str = Utf8SystemErrorCodeStringBuffered(_err_buf, ERROR_BUFSIZE, code);	\
		Log(T_SYSTEM, severity, "At %s:%u in function %s - %k", file, line, func, &_err_str);		\
	}

#elif __DS_PLATFORM__ == __DS_WIN64__

	#define LogSystemError(severity)		_LogSystemError(severity, __func__, __FILE__, __LINE__)
	#define _LogSystemError(severity, func, file, line)							\
	{													\
		u8 _err_buf[ERROR_BUFSIZE];									\
		const utf8 _err_str = Utf8SystemErrorCodeStringBuffered(_err_buf, ERROR_BUFSIZE, 0);	\
		Log(T_SYSTEM, severity, "At %s:%u in function %s - %k\n", file, line, func, &_err_str);		\
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

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
ds_error.h
==========
Fatal-error exit and system error messages.

Usage:
    Log(T_SYSTEM, S_FATAL, "out of memory in %s", __func__);
    FatalCleanupAndExit();
*/

#ifndef __DREAMSCAPE_ERROR_H__
#define __DREAMSCAPE_ERROR_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <errno.h>

#include "ds_string.h"

/*
Never returns. The first caller writes a crash report (Linux: stack trace on stderr; web: call stack on
stderr; Windows: minidump <executable>_<version>_latest.dmp in the working directory: stacks, globals and the
memory they point to, not the whole heap), shuts down the log (LogShutdown) and ends the process:
Breakpoint (a debugger stops there), then exit(1). Other
threads calling it meanwhile wait for that exit; a fatal error during the cleanup exits at once. Safe
before any engine init.
*/
void 	FatalCleanupAndExit(void);

/* Write the text of a system error code into buf and return it (UTF-8, in English where the system has English
 * messages); empty on failure or when it does not fit. Thread-safe. */
utf8	Utf8SystemErrorCodeStringBuffered(u8 *buf, const u32 bufsize, const u32 code);


#ifdef __cplusplus
} 
#endif

#endif

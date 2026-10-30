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
ds_assert.h
===========
Debug assertions, usable in every header (this one needs only ds_define.h and ds_types.h). With
DS_DEBUG off (no DS_ASSERT_DEBUG) they compile to nothing: no side effects or error handling inside.

    ds_Assert(condition)
    ds_AssertString(condition, cstr)                    (also logs cstr)
    ds_AssertMessage(condition, format, ...)            (also logs the formatted message, Utf8Format rules)

A failed assertion logs the file, line, function (and message), stops in a debugger (Breakpoint) at the
failing line, and calls FatalCleanupAndExit.
*/

#ifndef __DS_ASSERT_H__
#define __DS_ASSERT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_define.h"
#include "ds_types.h"

#ifdef DS_ASSERT_DEBUG

/* Internal: log a failed assertion; format may be NULL (no message). */
void	ds_AssertFailed(const char *file, const u32 line, const char *func, const char *format, ...);
/* ds_error.h: declared here too so that asserts need no later header */
void 	FatalCleanupAndExit(void);

#define ds_Assert(ds_Assertion)                                                                         \
	if (ds_Assertion) { }                                                                               \
	else                                                                                                \
	{                                                                                                   \
		ds_AssertFailed(__FILE__, __LINE__, __func__, NULL);                                            \
		Breakpoint(1);                                                                                  \
		FatalCleanupAndExit();                                                                          \
	}

#define ds_AssertString(ds_Assertion, cstr)                                                             \
	if (ds_Assertion) { }                                                                               \
	else                                                                                                \
	{                                                                                                   \
		ds_AssertFailed(__FILE__, __LINE__, __func__, "%s", cstr);                                      \
		Breakpoint(1);                                                                                  \
		FatalCleanupAndExit();                                                                          \
	}

#define ds_AssertMessage(ds_Assertion, msg, ...)                                                        \
	if (ds_Assertion) { }                                                                               \
	else                                                                                                \
	{                                                                                                   \
		ds_AssertFailed(__FILE__, __LINE__, __func__, msg, __VA_ARGS__);                                \
		Breakpoint(1);                                                                                  \
		FatalCleanupAndExit();                                                                          \
	}

#else

#define ds_Assert(ds_Assertion)
#define ds_AssertString(ds_Assertion, str)
#define ds_AssertMessage(ds_Assertion, msg, ...)

#endif

#ifdef __cplusplus
}
#endif

#endif

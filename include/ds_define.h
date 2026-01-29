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

#ifndef __DREAMSCAPE_DEFINE_H__
#define __DREAMSCAPE_DEFINE_H__

#ifdef __cplusplus
extern "C" { 
#endif

/*** system definitions ***/
#define __DS_SDL3__	0

#define __DS_WIN64__	1
#define __DS_LINUX__	2
#define __DS_WEB__	3

#define __DS_GCC__ 	4
#define __DS_MSVC__ 	5
#define __DS_CLANG__ 	6

#if defined(__linux__)
	#define __DS_PLATFORM__ 	__DS_LINUX__
#elif defined(__EMSCRIPTEN__)
	#define __DS_PLATFORM__ 	__DS_WEB__
#elif defined(_WIN64)
	#define __DS_PLATFORM__ 	__DS_WIN64__
#endif

#define __GAPI__ __DS_SDL3__

#define DS_CACHE_LINE_UB		64

#if defined(__EMSCRIPTEN__)

	#define __DS_COMPILER__ __DS_EMSCRIPTEN__ 
	#define DS_LITTLE_ENDIAN
	#define dsThreadLocal __thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))
	#undef DS_PROFILE

#elif defined(__clang__)

	#define __DS_COMPILER__	__DS_CLANG__
	#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
		#define DS_LITTLE_ENDIAN
	#elif (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
		#define DS_BIG_ENDIAN
	#endif
	#define dsThreadLocal	__thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))

#elif defined(__GNUC__)

	#define __DS_COMPILER__	__DS_GCC__
	#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
		#define DS_LITTLE_ENDIAN
	#elif (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
		#define DS_BIG_ENDIAN
	#endif
	#define dsThreadLocal	__thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))

#elif defined(_MSC_VER)

	#define __DS_COMPILER__	__MSVC__
	#define DS_LITTLE_ENDIAN
	#define dsThreadLocal	__declspec(thread)
	#define ds_StaticAssert(ds_Assertion, str)	static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __declspec(align(alignment)) 

#endif

#ifdef DS_DEBUG
	#define DS_PHYSICS_DEBUG	/* Physics debug events, physics debug rendering	*/
	#define DS_ASSERT_DEBUG 	/* Asserts on 						*/
	#define DS_GL_DEBUG		/* graphics debugging */

	#if __DS_PLATFORM__ == __DS_LINUX__

		#ifdef __x86_64__
			#define Breakpoint(condition) if (!(condition)) { } else { asm ("int3; nop"); }
		#endif 
	
	#elif __DS_PLATFORM__ == __DS_WEB__

		void emscripten_debugger(void);
		#define Breakpoint(condition) if (!(condition)) { } else { emscripten_debugger(); }

	#elif __DS_PLATFORM__ == __DS_WIN64__

		#ifdef _M_X64
			#define Breakpoint(condition) if (!(condition)) { } else { __debugbreak(); }
		#endif 
	#endif
#else
	#define Breakpoint(condition)
#endif

#ifdef __cplusplus
} 
#endif

#endif

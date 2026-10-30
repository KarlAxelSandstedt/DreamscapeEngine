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
ds_define.h
===========
Platform, compiler and build-mode definitions that every engine header builds on (it includes only the
compiler's intrinsics headers). An unsupported platform, CPU or compiler is an #error.

    __DS_PLATFORM__     __DS_LINUX__, __DS_WIN64__ or __DS_WEB__ (Emscripten).
    __DS_COMPILER__     __DS_GCC__, __DS_CLANG__ (emcc too) or __DS_MSVC__.
    __DS_X64__          64-bit x86 (Linux and Windows; the web is wasm).
    DS_LITTLE_ENDIAN    or DS_BIG_ENDIAN (GCC/Clang byte-order macros; MSVC and the web are little endian).
    DS_CACHE_LINE       64 bytes. DS_THREAD_COUNT_MAX: most threads the engine indexes, main thread included.
    ds_ThreadLocal, ds_StaticAssert(condition, cstr), ds_Align(bytes), ds_ForceInline, ds_NoReturn: portable keywords.
    ds_AcqRelCompilerBarrier   compiler-only barrier, no CPU fence (thread ordering: ds_atomic.h).
    ds_CpuPause(count)  count pause instructions, a spin-wait hint; nothing on the web.
    Breakpoint(condition)   DS_DEBUG builds: if condition holds, stop in a debugger (without one, Linux and
                        Windows end the process; the web does nothing). Otherwise compiled out: no side
                        effects or error handling inside.
    DS_DEBUG            also defines DS_ASSERT_DEBUG (ds_assert.h), DS_PHYSICS_DEBUG and DS_GL_DEBUG.
                        The web build undefines DS_PROFILE (no Tracy there).
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
    #if defined(__x86_64__)
        #define __DS_X64__
    #else 
        #error "ds_define.h: Linux is supported on x86-64 only"
    #endif
#elif defined(__EMSCRIPTEN__)
	#define __DS_PLATFORM__ 	__DS_WEB__
#elif defined(_WIN64)
	#define __DS_PLATFORM__ 	__DS_WIN64__
    #if defined(_M_X64)
        #define __DS_X64__
    #else 
        #error "ds_define.h: Windows is supported on x64 only"
    #endif
#else
    #error "ds_define.h: unsupported platform (Linux, Windows or Emscripten)"
#endif

#define __GAPI__ __DS_SDL3__

#define DS_CACHE_LINE		64

#if defined(__EMSCRIPTEN__)

	#define __DS_COMPILER__ __DS_CLANG__	/* emcc is clang; __DS_PLATFORM__ == __DS_WEB__ tells the web build apart */
	#define DS_LITTLE_ENDIAN
	#define ds_ThreadLocal __thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))
	#define ds_ForceInline inline __attribute__((always_inline))
	#define ds_NoReturn __attribute__((noreturn))
    #define ds_AcqRelCompilerBarrier __asm__ __volatile__ ("" ::: "memory")
	#undef DS_PROFILE	/* no Tracy on the web */

#elif defined(__clang__)

	#define __DS_COMPILER__	__DS_CLANG__
	#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
		#define DS_LITTLE_ENDIAN
	#elif (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
		#define DS_BIG_ENDIAN
	#endif
	#define ds_ThreadLocal	__thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))
	#define ds_ForceInline inline __attribute__((always_inline))
	#define ds_NoReturn __attribute__((noreturn))
    #define ds_AcqRelCompilerBarrier __asm__ __volatile__ ("" ::: "memory")

#elif defined(__GNUC__)

	#define __DS_COMPILER__	__DS_GCC__
	#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
		#define DS_LITTLE_ENDIAN
	#elif (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
		#define DS_BIG_ENDIAN
	#endif
	#define ds_ThreadLocal	__thread
	#define ds_StaticAssert(ds_Assertion, str)	_Static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __attribute__((aligned(alignment)))
	#define ds_ForceInline inline __attribute__((always_inline))
	#define ds_NoReturn __attribute__((noreturn))
    #define ds_AcqRelCompilerBarrier __asm__ __volatile__ ("" ::: "memory")

#elif defined(_MSC_VER)

	#define __DS_COMPILER__	__DS_MSVC__
	#define DS_LITTLE_ENDIAN
	#define ds_ThreadLocal	__declspec(thread)
	#define ds_StaticAssert(ds_Assertion, str)	static_assert(ds_Assertion, str)
	#define ds_Align(alignment) __declspec(align(alignment)) 
	#define ds_ForceInline __forceinline
	#define ds_NoReturn __declspec(noreturn)
    #define ds_AcqRelCompilerBarrier atomic_signal_fence(memory_order_acq_rel)

#else

	#error "ds_define.h: unsupported compiler (GCC, Clang, MSVC)"

#endif

/* Most threads the engine creates and indexes (thread index < DS_THREAD_COUNT_MAX), main thread included */
#define DS_THREAD_COUNT_MAX	64

#ifdef DS_DEBUG
	#define DS_PHYSICS_DEBUG	/* Physics pipeline validation every tick		*/
	#define DS_ASSERT_DEBUG 	/* Asserts on 						*/
	#define DS_GL_DEBUG		/* graphics debugging */
#endif

#if __DS_PLATFORM__ == __DS_LINUX__

	#ifdef __DS_X64__
        #include <immintrin.h>	/* _mm_pause */
		#ifdef DS_DEBUG
			#define Breakpoint(condition) if (!(condition)) { } else { asm ("int3; nop"); }
		#else
			#define Breakpoint(condition)
		#endif
        #define ds_CpuPause(__cpu_pause_count)                                                          \
        do                                                                                              \
        {                                                                                               \
            for (u32 __cpu_pause_index = 0; __cpu_pause_index < (__cpu_pause_count); ++__cpu_pause_index) \
            {                                                                                           \
                _mm_pause();                                                                            \
            }                                                                                           \
        } while (0)
	#endif

#elif __DS_PLATFORM__ == __DS_WEB__

	void emscripten_debugger(void);
	#ifdef DS_DEBUG
		#define Breakpoint(condition) if (!(condition)) { } else { emscripten_debugger(); }
	#else
		#define Breakpoint(condition)
	#endif
    #define ds_CpuPause(count)	((void) 0)

#elif __DS_PLATFORM__ == __DS_WIN64__

	#ifdef __DS_X64__
        #include <intrin.h>
		#ifdef DS_DEBUG
			#define Breakpoint(condition) if (!(condition)) { } else { __debugbreak(); }
		#else
			#define Breakpoint(condition)
		#endif
        #define ds_CpuPause(__cpu_pause_count)                                                          \
        do                                                                                              \
        {                                                                                               \
            for (u32 __cpu_pause_index = 0; __cpu_pause_index < (__cpu_pause_count); ++__cpu_pause_index) \
            {                                                                                           \
                _mm_pause();                                                                            \
            }                                                                                           \
        } while (0)
	#endif 
#endif


#ifdef __cplusplus
} 
#endif

#endif

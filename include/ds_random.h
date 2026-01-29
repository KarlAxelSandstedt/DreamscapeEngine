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

#ifndef __DS_RANDOM_H__
#define __DS_RANDOM_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_types.h"

/*************************** THREAD-SAFE RNG API ***************************/

/* push current thread local rng state */
void 	RngPushState(void);
/* pop old rng state */
void	RngPopState(void);
/* gen [0, U64_MAX] random number */
u64 	RngU64(void);
/* gen [min, max] random number */
u64 	RngU64Range(const u64 min, const u64 max);
/* gen [0.0f, 1.0f] random number */
f32 	RngF32Normalized(void);
/* gen [min, max] random number */
f32 	RngF32Range(const f32 min, const f32 max);

/*************************** internal rng initiation ***************************/

/*
 *	xoshiro256** (David Blackman, Sebastiano Vigna)
 */
/* Call once on main thread before calling thread_init_rng_local on each thread */ 
void 	Xoshiro256Init(const u64 seed[4]);
/* Call once on thread to initate thread local xoshiro256** rng sequence  */ 
void	ThreadXoshiro256InitSequence(void);
/* NOTE: THREAD UNSAFE!!! Exposed for testing purposes. next rng on global rng */ 
u64 	TestXoshiro256Next(void);

#ifdef __cplusplus
} 
#endif

#endif

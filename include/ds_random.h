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

#include "ds_common.h"

/*************************** THREAD-SAFE RNG API ***************************/

/* push current thread local rng state */
void 	rng_push_state(void);
/* pop old rng state */
void	rng_pop_state(void);
/* gen [0, U64_MAX] random number */
u64 	rng_u64(void);
/* gen [min, max] random number */
u64 	rng_u64_range(const u64 min, const u64 max);
/* gen [0.0f, 1.0f] random number */
f32 	rng_f32_normalized(void);
/* gen [min, max] random number */
f32 	rng_f32_range(const f32 min, const f32 max);

/*************************** internal rng initiation ***************************/

/*
 *	xoshiro256** (David Blackman, Sebastiano Vigna)
 */
/* Call once on main thread before calling thread_init_rng_local on each thread */ 
void 	g_xoshiro_256_init(const u64 seed[4]);
/* Call once on thread to initate thread local xoshiro256** rng sequence  */ 
void	thread_xoshiro_256_init_sequence(void);
/* NOTE: THREAD UNSAFE!!! Exposed for testing purposes. next rng on global rng */ 
u64 	g_xoshiro_256_next(void);

#ifdef __cplusplus
} 
#endif

#endif

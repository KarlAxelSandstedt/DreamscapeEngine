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

/*************************** LOCAL RNG API ***************************/

/*
 * A generator with its own state (xoshiro256**), independent of the thread's: the same seed gives the same numbers on
 * every thread and platform, and using it leaves the thread's numbers unchanged. One user at a time (not thread-safe).
 */
struct rng
{
	u64 s[4];
};

/* Seed rng with any value (0 included); the state is SplitMix64's first four outputs from seed */
void	RngLocalInit(struct rng *rng, const u64 seed);
/* gen [0, U64_MAX] random number */
u64 	RngLocalU64(struct rng *rng);
/* gen [min, max] random number */
u64 	RngLocalU64Range(struct rng *rng, const u64 min, const u64 max);

/*************************** internal rng initialization ***************************/

/*
 *	xoshiro256** (David Blackman, Sebastiano Vigna)
 */
/* Call once on the main thread before any other thread starts; seeds it with stream 0 */ 
void 	Xoshiro256Init(const u64 seed[4]);
/*
 * Seed the calling thread with stream `stream`: the seed advanced by stream * 2^128 numbers, so the
 * streams don't overlap. Threads pass their thread index, which makes each thread's numbers depend
 * only on the seed and the index, not on the order threads start in.
 */ 
void	ThreadXoshiro256InitSequence(const u32 stream);

/* NOTE: THREAD UNSAFE!!! Exposed for testing purposes. Next number of the base state (stream 0), which
 * it advances: streams seeded afterwards start from the advanced state. */ 
u64 	TestXoshiro256Next(void);

#ifdef __cplusplus
} 
#endif

#endif

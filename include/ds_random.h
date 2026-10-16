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
#include "ds_define.h"
#include "ds_assert.h"

/*************************** THREAD-SAFE RNG API ***************************/

/* push current thread local rng state */
static ds_ForceInline void RngPushState(void);
/* pop old rng state */
static ds_ForceInline void RngPopState(void);
/* gen [0, U64_MAX] random number */
static ds_ForceInline u64 RngU64(void);
/* gen [min, max] random number */
static ds_ForceInline u64 RngU64Range(const u64 min, const u64 max);
/* gen [0.0f, 1.0f] random number */
static ds_ForceInline f32 RngF32Normalized(void);
/* gen [min, max] random number */
static ds_ForceInline f32 RngF32Range(const f32 min, const f32 max);

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
static ds_ForceInline void RngLocalInit(struct rng *rng, const u64 seed);
/* gen [0, U64_MAX] random number */
static ds_ForceInline u64 RngLocalU64(struct rng *rng);
/* gen [min, max] random number */
static ds_ForceInline u64 RngLocalU64Range(struct rng *rng, const u64 min, const u64 max);

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

/* the thread generator's state and its pushed copy (ds_random.c), used by the inline functions below */
extern ds_ThreadLocal u64 tl_xoshiro_256[4];
extern ds_ThreadLocal u64 tl_pushed_state[4];

/*************************** xoshiro256**: inline definitions ***************************/

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
static ds_ForceInline u64 Xoshiro256Rotl(const u64 x, const i32 k)
{
	return (x << k) | (x >> (64 - k));
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org); return the next number and advance s */
static ds_ForceInline u64 Xoshiro256Next(u64 s[4])
{
	const u64 result = Xoshiro256Rotl(s[1] * 5, 7) * 9;
	const u64 t = s[1] << 17;
	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];
	s[2] ^= t;
	s[3] = Xoshiro256Rotl(s[3], 45);
	return result;
}

/*************************** THREAD-SAFE RNG API: inline definitions ***************************/

static ds_ForceInline void RngPushState(void)
{
	tl_pushed_state[0] = tl_xoshiro_256[0];
	tl_pushed_state[1] = tl_xoshiro_256[1];
	tl_pushed_state[2] = tl_xoshiro_256[2];
	tl_pushed_state[3] = tl_xoshiro_256[3];
}

static ds_ForceInline void RngPopState(void)
{
	tl_xoshiro_256[0] = tl_pushed_state[0];
	tl_xoshiro_256[1] = tl_pushed_state[1];
	tl_xoshiro_256[2] = tl_pushed_state[2];
	tl_xoshiro_256[3] = tl_pushed_state[3];
}

static ds_ForceInline u64 RngU64(void)
{
	return Xoshiro256Next(tl_xoshiro_256);
}

static ds_ForceInline u64 RngU64Range(const u64 min, const u64 max)
{
	ds_Assert(min <= max);
	const u64 r = RngU64();
	const u64 interval = max-min+1;
	return (interval == 0) ? r : (r % interval) + min;
}

static ds_ForceInline f32 RngF32Normalized(void)
{
	return (f32) RngU64() / (f32) U64_MAX;
}

static ds_ForceInline f32 RngF32Range(const f32 min, const f32 max)
{
	ds_Assert(min <= max);
	return RngF32Normalized() * (max-min) + min;
}

/*************************** LOCAL RNG API: inline definitions ***************************/

/* SplitMix64 (Sebastiano Vigna, 2015): the seeding the xoshiro authors recommend */
static ds_ForceInline void RngLocalInit(struct rng *rng, const u64 seed)
{
	u64 x = seed;
	for (u32 i = 0; i < 4; ++i)
	{
		x += 0x9e3779b97f4a7c15;
		u64 z = x;
		z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
		z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
		rng->s[i] = z ^ (z >> 31);
	}
}

static ds_ForceInline u64 RngLocalU64(struct rng *rng)
{
	return Xoshiro256Next(rng->s);
}

static ds_ForceInline u64 RngLocalU64Range(struct rng *rng, const u64 min, const u64 max)
{
	ds_Assert(min <= max);
	const u64 r = RngLocalU64(rng);
	const u64 interval = max-min+1;
	return (interval == 0) ? r : (r % interval) + min;
}

#ifdef __cplusplus
} 
#endif

#endif

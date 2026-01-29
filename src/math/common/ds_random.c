#include "ds_random.h"
#include "sys_public.h"

/* xoshiro_256** */
u64 g_xoshiro_256[4];
u32 a_g_xoshiro_256_lock = 0;

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
static u64 rotl(const u64 x, i32 k) 
{
	return (x << k) | (x >> (64 - k));
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
u64 g_xoshiro_256_next(void) 
{
	const u64 result = rotl(g_xoshiro_256[1] * 5, 7) * 9;

	const u64 t = g_xoshiro_256[1] << 17;

	g_xoshiro_256[2] ^= g_xoshiro_256[0];
	g_xoshiro_256[3] ^= g_xoshiro_256[1];
	g_xoshiro_256[1] ^= g_xoshiro_256[2];
	g_xoshiro_256[0] ^= g_xoshiro_256[3];

	g_xoshiro_256[2] ^= t;

	g_xoshiro_256[3] = rotl(g_xoshiro_256[3], 45);

	return result;
}

void g_xoshiro_256_init(const u64 seed[4])
{
	g_xoshiro_256[0] = seed[0];
	g_xoshiro_256[1] = seed[1];
	g_xoshiro_256[2] = seed[2];
	g_xoshiro_256[3] = seed[3];
}

dsThreadLocal u64 thread_xoshiro_256[4];
dsThreadLocal u64 thread_pushed_state[4];

void rng_push_state(void)
{
	thread_pushed_state[0] = thread_xoshiro_256[0];
	thread_pushed_state[1] = thread_xoshiro_256[1];
	thread_pushed_state[2] = thread_xoshiro_256[2];
	thread_pushed_state[3] = thread_xoshiro_256[3];
}

void rng_pop_state(void)
{
	thread_xoshiro_256[0] = thread_pushed_state[0];
	thread_xoshiro_256[1] = thread_pushed_state[1];
	thread_xoshiro_256[2] = thread_pushed_state[2];
	thread_xoshiro_256[3] = thread_pushed_state[3];
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
inline u64 rng_u64(void)
{
	const u64 result = rotl(thread_xoshiro_256[1] * 5, 7) * 9;

	const u64 t = thread_xoshiro_256[1] << 17;

	thread_xoshiro_256[2] ^= thread_xoshiro_256[0];
	thread_xoshiro_256[3] ^= thread_xoshiro_256[1];
	thread_xoshiro_256[1] ^= thread_xoshiro_256[2];
	thread_xoshiro_256[0] ^= thread_xoshiro_256[3];

	thread_xoshiro_256[2] ^= t;

	thread_xoshiro_256[3] = rotl(thread_xoshiro_256[3], 45);

	return result;
}

u64 rng_u64_range(const u64 min, const u64 max)
{
	ds_Assert(min <= max);
	const u64 r = rng_u64();
	const u64 interval = max-min+1;
	return (interval == 0) ? r : (r % interval) + min;
}

f32 rng_f32_normalized(void)
{
	return (f32) rng_u64() / (f32) U64_MAX;
}

f32 rng_f32_range(const f32 min, const f32 max)
{
	ds_Assert(min <= max);
	return rng_f32_normalized() * (max-min) + min;
}


/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
void g_xoshiro_256_jump(void) 
{
	static const u64 JUMP[] = { 0x180ec6d33cfd0aba, 0xd5a61266f0c9392c, 0xa9582618e03fc9aa, 0x39abdc4529b1661c };

	u64 s0 = 0;
	u64 s1 = 0;
	u64 s2 = 0;
	u64 s3 = 0;
	for(u64 i = 0; i < sizeof(JUMP) / sizeof(JUMP[0]); i++)
		for(int b = 0; b < 64; b++) {
			if (JUMP[i] & UINT64_C(1) << b) {
				s0 ^= g_xoshiro_256[0];
				s1 ^= g_xoshiro_256[1];
				s2 ^= g_xoshiro_256[2];
				s3 ^= g_xoshiro_256[3];
			}
			g_xoshiro_256_next();	
		}
		
	g_xoshiro_256[0] = s0;
	g_xoshiro_256[1] = s1;
	g_xoshiro_256[2] = s2;
	g_xoshiro_256[3] = s3;
}

void thread_xoshiro_256_init_sequence(void)
{
	u32 a_wanted_lock_state;
	AtomicStoreRel32(&a_wanted_lock_state, 0);
	while (!AtomicCompareExchangeSeqCst32(&a_g_xoshiro_256_lock, &a_wanted_lock_state, 1))
	{
		AtomicStoreRel32(&a_wanted_lock_state, 0);
	}

	thread_xoshiro_256[0] = g_xoshiro_256[0];
	thread_xoshiro_256[1] = g_xoshiro_256[1];
	thread_xoshiro_256[2] = g_xoshiro_256[2];
	thread_xoshiro_256[3] = g_xoshiro_256[3];
	g_xoshiro_256_jump();

	AtomicStoreSeqCst32(&a_g_xoshiro_256_lock, 0);
}


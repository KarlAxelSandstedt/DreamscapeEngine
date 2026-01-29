#include "ds_base.h"

dsThreadLocal u64 tl_xoshiro_256[4];
dsThreadLocal u64 tl_pushed_state[4];

/* xoshiro_256** */
u64 g_xoshiro_256[4];
u32 a_g_xoshiro_256_lock = 0;

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
static u64 Rotl(const u64 x, i32 k) 
{
	return (x << k) | (x >> (64 - k));
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
u64 TestXoshiro256Next(void) 
{
	const u64 result = Rotl(g_xoshiro_256[1] * 5, 7) * 9;

	const u64 t = g_xoshiro_256[1] << 17;

	g_xoshiro_256[2] ^= g_xoshiro_256[0];
	g_xoshiro_256[3] ^= g_xoshiro_256[1];
	g_xoshiro_256[1] ^= g_xoshiro_256[2];
	g_xoshiro_256[0] ^= g_xoshiro_256[3];

	g_xoshiro_256[2] ^= t;

	g_xoshiro_256[3] = Rotl(g_xoshiro_256[3], 45);

	return result;
}

void Xoshiro256Init(const u64 seed[4])
{
	g_xoshiro_256[0] = seed[0];
	g_xoshiro_256[1] = seed[1];
	g_xoshiro_256[2] = seed[2];
	g_xoshiro_256[3] = seed[3];
}

void RngPushState(void)
{
	tl_pushed_state[0] = tl_xoshiro_256[0];
	tl_pushed_state[1] = tl_xoshiro_256[1];
	tl_pushed_state[2] = tl_xoshiro_256[2];
	tl_pushed_state[3] = tl_xoshiro_256[3];
}

void RngPopState(void)
{
	tl_xoshiro_256[0] = tl_pushed_state[0];
	tl_xoshiro_256[1] = tl_pushed_state[1];
	tl_xoshiro_256[2] = tl_pushed_state[2];
	tl_xoshiro_256[3] = tl_pushed_state[3];
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org) */
inline u64 RngU64(void)
{
	const u64 result = Rotl(tl_xoshiro_256[1] * 5, 7) * 9;

	const u64 t = tl_xoshiro_256[1] << 17;

	tl_xoshiro_256[2] ^= tl_xoshiro_256[0];
	tl_xoshiro_256[3] ^= tl_xoshiro_256[1];
	tl_xoshiro_256[1] ^= tl_xoshiro_256[2];
	tl_xoshiro_256[0] ^= tl_xoshiro_256[3];

	tl_xoshiro_256[2] ^= t;

	tl_xoshiro_256[3] = Rotl(tl_xoshiro_256[3], 45);

	return result;
}

u64 RngU64Range(const u64 min, const u64 max)
{
	ds_Assert(min <= max);
	const u64 r = RngU64();
	const u64 interval = max-min+1;
	return (interval == 0) ? r : (r % interval) + min;
}

f32 RngF32Normalized(void)
{
	return (f32) RngU64() / (f32) U64_MAX;
}

f32 RngF32Range(const f32 min, const f32 max)
{
	ds_Assert(min <= max);
	return RngF32Normalized() * (max-min) + min;
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
			TestXoshiro256Next();	
		}
		
	g_xoshiro_256[0] = s0;
	g_xoshiro_256[1] = s1;
	g_xoshiro_256[2] = s2;
	g_xoshiro_256[3] = s3;
}

void ThreadXoshiro256InitSequence(void)
{
	u32 a_wanted_lock_state;
	AtomicStoreRel32(&a_wanted_lock_state, 0);
	while (!AtomicCompareExchangeSeqCst32(&a_g_xoshiro_256_lock, &a_wanted_lock_state, 1))
	{
		AtomicStoreRel32(&a_wanted_lock_state, 0);
	}

	tl_xoshiro_256[0] = g_xoshiro_256[0];
	tl_xoshiro_256[1] = g_xoshiro_256[1];
	tl_xoshiro_256[2] = g_xoshiro_256[2];
	tl_xoshiro_256[3] = g_xoshiro_256[3];
	g_xoshiro_256_jump();

	AtomicStoreSeqCst32(&a_g_xoshiro_256_lock, 0);
}


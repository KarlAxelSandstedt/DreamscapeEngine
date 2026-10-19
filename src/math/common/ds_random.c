#include "ds_base.h"
#include "ds_random.h"

ds_ThreadLocal u64 tl_xoshiro_256[4];

/* xoshiro_256** base state: stream 0; set by Xoshiro256Init before any other thread starts */
u64 g_xoshiro_256[4];

u64 TestXoshiro256Next(void) 
{
	return Xoshiro256Next(g_xoshiro_256);
}

void Xoshiro256Init(const u64 seed[4])
{
	g_xoshiro_256[0] = seed[0];
	g_xoshiro_256[1] = seed[1];
	g_xoshiro_256[2] = seed[2];
	g_xoshiro_256[3] = seed[3];
	ThreadXoshiro256InitSequence(0);
}

/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org): advance s by 2^128 calls */
static void Xoshiro256Jump(u64 s[4])
{
	static const u64 JUMP[] = { 0x180ec6d33cfd0aba, 0xd5a61266f0c9392c, 0xa9582618e03fc9aa, 0x39abdc4529b1661c };

	u64 s0 = 0;
	u64 s1 = 0;
	u64 s2 = 0;
	u64 s3 = 0;
	for(u64 i = 0; i < sizeof(JUMP) / sizeof(JUMP[0]); i++)
		for(int b = 0; b < 64; b++) {
			if (JUMP[i] & UINT64_C(1) << b) {
				s0 ^= s[0];
				s1 ^= s[1];
				s2 ^= s[2];
				s3 ^= s[3];
			}
			Xoshiro256Next(s);
		}
		
	s[0] = s0;
	s[1] = s1;
	s[2] = s2;
	s[3] = s3;
}

void ThreadXoshiro256InitSequence(const u32 stream)
{
	tl_xoshiro_256[0] = g_xoshiro_256[0];
	tl_xoshiro_256[1] = g_xoshiro_256[1];
	tl_xoshiro_256[2] = g_xoshiro_256[2];
	tl_xoshiro_256[3] = g_xoshiro_256[3];
	for (u32 i = 0; i < stream; ++i)
	{
		Xoshiro256Jump(tl_xoshiro_256);
	}
}

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

#define XXH_INLINE_ALL
#include "test_local.h"
#include "xxhash.h"

#define ARRAY_TEST_SIZE 	(1024*1024)
#define STRUCT_TEST_HASH_COUNT	(1024*1024)

/*
  NO INLINE: 
0000000000010df0 <xxHash3_struct_stress_test>:
   10df0:       55                      push   rbp
   10df1:       48 89 fd                mov    rbp,rdi
   10df4:       53                      push   rbx
   10df5:       bb 00 00 10 00          mov    ebx,0x100000
   10dfa:       48 83 ec 08             sub    rsp,0x8
   10dfe:       66 90                   xchg   ax,ax
   10e00:       be 10 00 00 00          mov    esi,0x10
   10e05:       48 89 ef                mov    rdi,rbp
   10e08:       e8 d3 68 14 00          call   1576e0 <XXH3_64bits>
   10e0d:       48 89 c2                mov    rdx,rax
   10e10:       8b 05 56 22 1a 00       mov    eax,DWORD PTR [rip+0x1a2256]        # 1b306c <g_sum>
   10e16:       01 d0                   add    eax,edx
   10e18:       89 05 4e 22 1a 00       mov    DWORD PTR [rip+0x1a224e],eax        # 1b306c <g_sum>
   10e1e:       83 eb 01                sub    ebx,0x1
   10e21:       75 dd                   jne    10e00 <xxHash3_struct_stress_test+0x10>
   10e23:       48 83 c4 08             add    rsp,0x8
   10e27:       5b                      pop    rbx
   10e28:       5d                      pop    rbp
   10e29:       c3                      ret

:::::::::: Running peformance suite xxHash Performance ::::::::::
       ::: xxHash3_128bits_struct_test ::: 
min: [2.92478ms] 5.342GB/s
max: [5.54229ms] 2.819GB/s
avg: [3.00328ms] 5.203GB/s
min Cycles/B: [0.67863Cyc/B]
        ::: xxHash3_64bits_struct_test ::: 
min: [1.69541ms] 9.216GB/s
max: [3.39532ms] 4.602GB/s
avg: [1.78181ms] 8.769GB/s
min Cycles/B: [0.39338Cyc/B]
       ::: xxHash3_128bits_array_test ::: 
min: [0.02894ms] 33.741GB/s
max: [0.17344ms] 5.631GB/s
avg: [0.03182ms] 30.690GB/s
min Cycles/B: [0.10745Cyc/B]
        ::: xxHash3_64bits_array_test ::: 
min: [0.02920ms] 33.439GB/s
max: [0.24233ms] 4.030GB/s
avg: [0.03253ms] 30.024GB/s
min Cycles/B: [0.10842Cyc/B]

  INLINE: 

00000000000112c0 <xxHash3_struct_stress_test>:
   112c0:       48 b8 b9 39 42 ea 7b    movabs rax,0x6782737bea4239b9
   112c7:       73 82 67 
   112ca:       48 33 07                xor    rax,QWORD PTR [rdi]
   112cd:       48 b9 3a 52 96 09 3b    movabs rcx,0xaf56bc3b0996523a
   112d4:       bc 56 af 
   112d7:       48 33 4f 08             xor    rcx,QWORD PTR [rdi+0x8]
   112db:       48 89 c6                mov    rsi,rax
   112de:       48 f7 e1                mul    rcx
   112e1:       48 0f ce                bswap  rsi
   112e4:       48 31 c2                xor    rdx,rax
   112e7:       48 8d 54 11 10          lea    rdx,[rcx+rdx*1+0x10]
   112ec:       48 01 f2                add    rdx,rsi
   112ef:       48 89 d0                mov    rax,rdx
   112f2:       48 c1 e8 25             shr    rax,0x25
   112f6:       48 31 d0                xor    rax,rdx
   112f9:       48 ba f9 79 37 9e 91    movabs rdx,0x165667919e3779f9
   11300:       67 56 16 
   11303:       48 0f af c2             imul   rax,rdx
   11307:       ba 00 00 10 00          mov    edx,0x100000
   1130c:       48 89 c1                mov    rcx,rax
   1130f:       48 c1 e9 20             shr    rcx,0x20
   11313:       31 c1                   xor    ecx,eax
   11315:       66 66 2e 0f 1f 84 00    data16 cs nop WORD PTR [rax+rax*1+0x0]
   1131c:       00 00 00 00 
   11320:       8b 05 46 bd 19 00       mov    eax,DWORD PTR [rip+0x19bd46]        # 1ad06c <g_sum>
   11326:       01 c8                   add    eax,ecx
   11328:       89 05 3e bd 19 00       mov    DWORD PTR [rip+0x19bd3e],eax        # 1ad06c <g_sum>
   1132e:       83 ea 01                sub    edx,0x1
   11331:       75 ed                   jne    11320 <xxHash3_struct_stress_test+0x60>
   11333:       c3                      ret


:::::::::: Running peformance suite xxHash Performance ::::::::::
        ::: xxHash3_128bits_struct_test ::: 
min: [0.23608ms] 66.187GB/s
max: [0.56992ms] 27.416GB/s
avg: [0.24384ms] 64.079GB/s
min Cycles/B: [0.05478Cyc/B]
        ::: xxHash3_64bits_struct_test ::: 
min: [0.23608ms] 66.186GB/s
max: [0.58252ms] 26.823GB/s
avg: [0.24243ms] 64.453GB/s
min Cycles/B: [0.05478Cyc/B]
       ::: xxHash3_128bits_array_test ::: 
min: [0.02912ms] 33.532GB/s
max: [0.19466ms] 5.017GB/s
avg: [0.03218ms] 30.350GB/s
min Cycles/B: [0.10812Cyc/B]
        ::: xxHash3_64bits_array_test ::: 
min: [0.02902ms] 33.648GB/s
max: [0.19872ms] 4.914GB/s
avg: [0.03191ms] 30.605GB/s
min Cycles/B: [0.10775Cyc/B]
      
*/

struct test_key 
{
	u32 a;
	u32 b;
	u32 c;
	u32 d;
};

struct hash_input
{
	struct test_key	key;
	void *		buf;
	u64		size;
	XXH32_hash_t	seed32;
	XXH64_hash_t	seed64;
	u32 		sum;
};

volatile u32 g_sum = 0;

void *hash_stress_init(void)
{
	struct hash_input *input = malloc(sizeof(struct hash_input));
	input->size = ARRAY_TEST_SIZE;
	input->buf = malloc(input->size);
	input->seed32 = (u32) RngU64();
	input->seed64 = RngU64();
	for (u32 i = 0; i < input->size; ++i)
	{
		((u8 *) input->buf)[i] = (u8) RngU64();
	}
	input->key.a = (u32) RngU64();
	input->key.b = (u32) RngU64();
	input->key.c = (u32) RngU64();
	input->key.d = (u32) RngU64();
	g_sum = 0;

	return input;
}

void hash_stress_free(void *args)
{
	struct hash_input *input = args;
	free(input->buf);
	free(input);
}

void xxHash32_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH32(input->buf, input->size, input->seed32);
}

void xxHash64_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH64(input->buf, input->size, input->seed64);
}

void xxHash128_withSeed_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH3_128bits_withSeed(input->buf, input->size, input->seed64).low64;
}

void xxHash3_withSeed_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH3_64bits_withSeed(input->buf, input->size, input->seed64);
}

void xxHash128_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH3_128bits(input->buf, input->size).low64;
}

void xxHash3_array_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	g_sum += (u32) XXH3_64bits(input->buf, input->size);
}

void xxHash32_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH32(&input->key, sizeof(struct test_key), input->seed32);
	}
}

void xxHash64_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH64(&input->key, sizeof(struct test_key), input->seed64);
	}
}

void xxHash128_withSeed_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH3_128bits_withSeed(&input->key, sizeof(struct test_key), input->seed64).low64;
	}
}

void xxHash3_withSeed_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH3_64bits_withSeed(&input->key, sizeof(struct test_key), input->seed64);
	}
}

void xxHash128_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH3_128bits(&input->key, sizeof(struct test_key)).low64;
	}
}

void xxHash3_struct_stress_test(void *void_input)
{
	struct hash_input *input = void_input;
	for (u32 i = 0; i < STRUCT_TEST_HASH_COUNT; ++i)
	{
		g_sum += (u32) XXH3_64bits(&input->key, sizeof(struct test_key));
	}
}

struct serial_test hash_serial_test[] =
{
	{
		.id = "xxHash32_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash32_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash64_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash64_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_128bits_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash128_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_64bits_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash3_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_128bits_withSeed_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash128_withSeed_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_64bits_withSeed_struct_test",
		.size = STRUCT_TEST_HASH_COUNT*sizeof(struct test_key),
		.test = &xxHash3_withSeed_struct_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash32_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash32_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash64_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash64_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_128bits_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash128_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_64bits_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash3_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_128bits_withSeed_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash128_withSeed_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},

	{
		.id = "xxHash3_64bits_withSeed_array_test",
		.size = ARRAY_TEST_SIZE,
		.test = &xxHash3_withSeed_array_stress_test,
		.test_init = &hash_stress_init,
		.test_reset = NULL,
		.test_free = &hash_stress_free,
	},
};

struct performance_suite storage_performance_hash_suite =
{
	.id = "xxHash Performance",
	.parallel_test = NULL,
	.parallel_test_count = 0, 
	.serial_test = hash_serial_test,
	.serial_test_count = sizeof(hash_serial_test) / sizeof(hash_serial_test[0]),
};

struct performance_suite *hash_performance_suite = &storage_performance_hash_suite;


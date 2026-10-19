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

#include <float.h>

#include "ds_test.h"
#include "ticket_factory.h"
#include "ds_graphics.h"
#include "ds_renderer.h"
#include "ds_platform.h"

static u32 BitTestClz64(const u64 x)
{
    u32 n = 0;
    for (u64 bit = (u64) 1 << 63; bit && !(x & bit); bit >>= 1) { n += 1; }
    return n;
}

static u32 BitTestCtz64(const u64 x)
{
    u32 n = 0;
    for (u64 bit = 1; bit && !(x & bit); bit <<= 1) { n += 1; }
    return n;
}

static u32 BitTestOnes64(const u64 x)
{
    u32 n = 0;
    for (u64 bit = 1; bit; bit <<= 1) { n += !!(x & bit); }
    return n;
}

/* Clz/Ctz/C/Cz (32 and 64 bit) against bit loops: single bits, edges, random values of every bit length. */
static struct test_Output BitCountTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    /* 64 values of every bit length: the top bit, random lower bits, and a random number of low zero bits */
    for (u32 i = 0; i < 64*64; ++i)
    {
        const u32 length = i / 64 + 1;
        const u64 top = (u64) 1 << (length - 1);
        const u64 low = RngU64() & (top - 1);
        const u64 low_zeros = RngU64Range(0, length - 1);
        const u64 x = top | ((low >> low_zeros) << low_zeros);
        TEST_EQUAL(BitTestClz64(x), Clz64(x));
        TEST_EQUAL(BitTestCtz64(x), Ctz64(x));
        TEST_EQUAL(BitTestOnes64(x), C64(x));
        TEST_EQUAL(64 - BitTestOnes64(x), Cz64(x));

        if (length <= 32)
        {
            const u32 x32 = (u32) x;
            TEST_EQUAL(BitTestClz64(x32) - 32, Clz32(x32));
            TEST_EQUAL(BitTestCtz64(x32), Ctz32(x32));
            TEST_EQUAL(BitTestOnes64(x32), C32(x32));
            TEST_EQUAL(32 - BitTestOnes64(x32), Cz32(x32));
        }
    }

    TEST_EQUAL(0, Clz64(U64_MAX));
    TEST_EQUAL(63, Clz64(1));
    TEST_EQUAL(63, Ctz64((u64) 1 << 63));
    TEST_EQUAL(0, Clz32(U32_MAX));
    TEST_EQUAL(31, Clz32(1));
    TEST_EQUAL(31, Ctz32(0x80000000u));
    TEST_EQUAL(32, C32(U32_MAX));
    TEST_EQUAL(0, Cz32(U32_MAX));
    TEST_EQUAL(64, Cz64(0));

	return output;
}

/* U64AddReturnOverflow / U64MulReturnOverflow: nonzero exactly on overflow; the low bits are the wrapped result. */
static struct test_Output OverflowTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u64 dst;
    TEST_FALSE(U64AddReturnOverflow(&dst, 2, 3));
    TEST_EQUAL(5, dst);
    TEST_TRUE(U64AddReturnOverflow(&dst, U64_MAX, 1));
    TEST_EQUAL(0, dst);
    TEST_FALSE(U64AddReturnOverflow(&dst, U64_MAX - 7, 7));
    TEST_EQUAL(U64_MAX, dst);

    TEST_FALSE(U64MulReturnOverflow(&dst, 3, 5));
    TEST_EQUAL(15, dst);
    TEST_TRUE(U64MulReturnOverflow(&dst, (u64) 1 << 32, (u64) 1 << 32));
    TEST_EQUAL(0, dst);
    TEST_FALSE(U64MulReturnOverflow(&dst, U64_MAX, 1));
    TEST_EQUAL(U64_MAX, dst);
    TEST_FALSE(U64MulReturnOverflow(&dst, 0, U64_MAX));
    TEST_EQUAL(0, dst);

    for (u32 i = 0; i < 1000; ++i)
    {
        const u64 a = RngU64() >> RngU64Range(0, 63);
        const u64 b = RngU64() >> RngU64Range(0, 63);
        TEST_EQUAL((a + b < a), !!U64AddReturnOverflow(&dst, a, b));
        TEST_EQUAL(a + b, dst);
        const u32 overflow = (a != 0 && b > U64_MAX / a);
        TEST_EQUAL(overflow, !!U64MulReturnOverflow(&dst, a, b));
        TEST_EQUAL(a * b, dst);
    }

	return output;
}

/*
 * Single thread: return values of every atomic family (old value for FetchX, new value for XFetch,
 * success + updated comparand for CompareExchange), values above 2^31 / 2^63 included (MSVC returns signed).
 */
static struct test_Output AtomicSingleThreadTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    u32 a32 = 0x80000005u;
    TEST_EQUAL(0x80000005u, (u32) AtomicFetchAddRlx32(&a32, 3));
    TEST_EQUAL(0x80000008u, a32);
    TEST_EQUAL(0x80000008u, (u32) AtomicFetchSubRel32(&a32, 8));
    TEST_EQUAL(0x80000000u, a32);
    TEST_EQUAL(0x80000010u, (u32) AtomicAddFetchAcqRel32(&a32, 16));
    TEST_EQUAL(0x8000000fu, (u32) AtomicSubFetchSeqCst32(&a32, 1));
    TEST_EQUAL(0x8000000fu, (u32) AtomicLoadAcq32(&a32));
    AtomicStoreRel32(&a32, U32_MAX);
    TEST_EQUAL(U32_MAX, (u32) AtomicLoadRlx32(&a32));
    TEST_EQUAL(0, (u32) AtomicAddFetchRlx32(&a32, 1));

    /* results are u32/u64 on every compiler: no sign extension of values >= 2^31 into 64 bits */
    a32 = 0x80000001u;
    const u64 widened_load = AtomicLoadAcq32(&a32);
    const u64 widened_fetch = AtomicFetchAddRlx32(&a32, 1);
    const u64 widened_add_fetch = AtomicAddFetchRlx32(&a32, 1);
    TEST_EQUAL(0x80000001ull, widened_load);
    TEST_EQUAL(0x80000001ull, widened_fetch);
    TEST_EQUAL(0x80000003ull, widened_add_fetch);
    TEST_EQUAL(0x80000003ull / 512, AtomicLoadRlx32(&a32) / 512);

    /* subtrahends >= 2^31 / 2^63 (MSVC negated them as signed: -(long) 0x80000000 overflows) */
    TEST_EQUAL(0x80000003u, (u32) AtomicFetchSubRlx32(&a32, 0x80000000u));
    TEST_EQUAL(3, a32);
    TEST_EQUAL(4, (u32) AtomicSubFetchRlx32(&a32, U32_MAX));
    TEST_EQUAL(0x80000004u, (u32) AtomicSubFetchAcq32(&a32, 0x80000000u));
    u64 sub64 = 5;
    TEST_EQUAL(5, (u64) AtomicFetchSubRlx64(&sub64, 0x8000000000000000ull));
    TEST_EQUAL(0x8000000000000005ull, sub64);
    TEST_EQUAL(0x8000000000000006ull, (u64) AtomicSubFetchRel64(&sub64, U64_MAX));
    TEST_EQUAL(6, (u64) AtomicSubFetchSeqCst64(&sub64, 0x8000000000000000ull));
    AtomicStoreRlx32(&a32, 0);

    u32 cmp32 = 1;
    TEST_FALSE(AtomicCompareExchangeAcqRlx32(&a32, &cmp32, 7));
    TEST_EQUAL(0, cmp32);
    TEST_EQUAL(0, a32);
    TEST_TRUE(AtomicCompareExchangeAcqRlx32(&a32, &cmp32, 0xfffffff0u));
    TEST_EQUAL(0xfffffff0u, a32);

    u64 a64 = 0x8000000000000005ull;
    TEST_EQUAL(0x8000000000000005ull, (u64) AtomicFetchAddRlx64(&a64, 3));
    TEST_EQUAL(0x8000000000000008ull, a64);
    TEST_EQUAL(0x8000000000000008ull, (u64) AtomicFetchSubAcq64(&a64, 0x100000000ull));
    TEST_EQUAL(0x7fffffff00000008ull, a64);
    TEST_EQUAL(0x8000000000000008ull, (u64) AtomicAddFetchRel64(&a64, 0x100000000ull));
    TEST_EQUAL(0x8000000000000007ull, (u64) AtomicSubFetchRlx64(&a64, 1));
    AtomicStoreSeqCst64(&a64, U64_MAX);
    TEST_EQUAL(U64_MAX, (u64) AtomicLoadSeqCst64(&a64));

    u64 cmp64 = 0;
    TEST_FALSE(AtomicCompareExchangeRelRlx64(&a64, &cmp64, 1));
    TEST_EQUAL(U64_MAX, cmp64);
    TEST_TRUE(AtomicCompareExchangeAcqRelAcq64(&a64, &cmp64, 0x123456789abcdef0ull));
    TEST_EQUAL(0x123456789abcdef0ull, (u64) AtomicLoadAcq64(&a64));

    u64 to_addr = 0;
    AtomicLoadToAddrAcq64(&a64, &to_addr);
    TEST_EQUAL(0x123456789abcdef0ull, to_addr);
    u64 from_addr = 42;
    AtomicStoreFromAddrRel64(&a64, &from_addr);
    TEST_EQUAL(42, a64);

	return output;
}

/* Tickets are handed out in order up to max_tickets; returning tickets frees as many; a closed factory says so. */
static struct test_Output TicketFactoryTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ticketFactory tf;
    TicketFactoryInit(&tf, 4);

    u32 ticket;
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_EQUAL(1, TicketFactoryTryGetTicket(&ticket, &tf));
        TEST_EQUAL(i, ticket);
    }
    TEST_EQUAL(0, TicketFactoryTryGetTicket(&ticket, &tf));

    TicketFactoryReturnTickets(&tf, 3);
    for (u32 i = 4; i < 7; ++i)
    {
        TEST_EQUAL(i, TicketFactoryGetTicket(&tf));
    }
    TEST_EQUAL(0, TicketFactoryTryGetTicket(&ticket, &tf));

    TicketFactoryReturnTickets(&tf, 4);
    TEST_EQUAL(7, TicketFactoryGetTicket(&tf));

    AtomicStoreRel32(&tf.a_open, 0);
    TEST_EQUAL(TICKET_FACTORY_CLOSED, TicketFactoryTryGetTicket(&ticket, &tf));
    /* closed: GetTicket ends its wait and says so (it returned an unwritten ticket) */
    TEST_EQUAL(TICKET_FACTORY_CLOSED, TicketFactoryGetTicket(&tf));

    TicketFactoryDestroy(&tf);

	return output;
}

/*
 * Counting past 2^16 (Windows used to cap its semaphores at 65535: the 65536th post was fatal there), Wait on a
 * positive count, TryWait on zero; Init with a large value.
 */
static struct test_Output SemaphoreTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    semaphore sem;
    SemaphoreInit(&sem, 0);
    TEST_EQUAL(0, SemaphoreTryWait(&sem));
    const u32 count = 70000;
    for (u32 i = 0; i < count; ++i)
    {
        SemaphorePost(&sem);
    }
    SemaphoreWait(&sem);
    u32 taken = 1;
    while (SemaphoreTryWait(&sem))
    {
        taken += 1;
    }
    TEST_EQUAL(count, taken);
    SemaphoreDestroy(&sem);

    SemaphoreInit(&sem, count);
    TEST_EQUAL(1, SemaphoreTryWait(&sem));
    SemaphoreDestroy(&sem);

	return output;
}

/* constant expressions: usable in static initializers */
static const i64 g_limits_min[4] = { I8_MIN, I16_MIN, I32_MIN, I64_MIN };

/*
 * ds_types.h limits against stdint.h (springclean 19: I8/I16/I32_MIN were positive), their types, and the
 * layout the math unions promise (members alias buf, matrices column-major, no padding).
 */
static struct test_Output TypesTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    TEST_TRUE(I8_MIN == INT8_MIN && I16_MIN == INT16_MIN && I32_MIN == INT32_MIN && I64_MIN == INT64_MIN);
    TEST_TRUE(I8_MAX == INT8_MAX && I16_MAX == INT16_MAX && I32_MAX == INT32_MAX && I64_MAX == INT64_MAX);
    TEST_TRUE(U8_MAX == UINT8_MAX && U16_MAX == UINT16_MAX && U32_MAX == UINT32_MAX && U64_MAX == UINT64_MAX);
    TEST_TRUE(sizeof(I8_MIN) == 1 && sizeof(I16_MIN) == 2 && sizeof(I32_MIN) == 4 && sizeof(I64_MIN) == 8);
    TEST_TRUE(g_limits_min[0] == -128 && g_limits_min[1] == -32768 && g_limits_min[2] == -2147483647 - 1);
    TEST_TRUE(g_limits_min[3] == INT64_MIN);

    TEST_TRUE(sizeof(v2) == 8 && sizeof(v3) == 12 && sizeof(v4) == 16 && sizeof(q) == 16 && sizeof(v3f64) == 24);
    TEST_TRUE(sizeof(m2) == 16 && sizeof(m3) == 36 && sizeof(m4) == 64);
    const v3 v = V3(1.0f, 2.0f, 3.0f);
    TEST_TRUE(v.buf[0] == 1.0f && v.buf[1] == 2.0f && v.buf[2] == 3.0f);
    const m3 m = M3(1.0f, 2.0f, 3.0f,
                    4.0f, 5.0f, 6.0f,
                    7.0f, 8.0f, 9.0f);
    TEST_TRUE(m.a21 == 2.0f && m.a12 == 4.0f && m.col[2].z == 9.0f && m.buf[1*3 + 0] == m.a12);

	return output;
}

static f32 FloatTestBits(const u32 bits)
{
    const union ieee32 val = { .bits = bits };
    return val.f;
}

static u32 FloatTestBitsOf(const f32 f)
{
    const union ieee32 val = { .f = f };
    return val.bits;
}

static f64 FloatTestF64(const u64 bits)
{
    f64 x;
    memcpy(&x, &bits, sizeof(x));
    return x;
}

/* the next f32 above f by stepping its bits (f neither NaN nor +inf); independent of nextafterf */
static f32 FloatTestNextUp(const f32 f)
{
    const u32 bits = FloatTestBitsOf(f);
    if ((bits & ~F32_SIGN_MASK) == 0) { return FloatTestBits(1); }
    return FloatTestBits((bits & F32_SIGN_MASK) ? bits - 1 : bits + 1);
}

static f32 FloatTestNextDown(const f32 f)
{
    return -FloatTestNextUp(-f);
}

/* x not NaN: F64DemoteDownwards(x) is the largest f32 <= x, F64DemoteUpwards(x) the smallest f32 >= x */
static u32 FloatTestDemoteCheck(const f64 x)
{
    const f32 d = F64DemoteDownwards(x);
    const f32 u = F64DemoteUpwards(x);
    return (f64) d <= x && x < (f64) FloatTestNextUp(d)
        && (f64) FloatTestNextDown(u) < x && x <= (f64) u;
}

/*
 * ds_float.h: the constants against float.h; bit access and classification of every exponent (zero, low,
 * full and random significands, both signs) against comparisons; min, max, clamp and round, NaN included;
 * F64DemoteDownwards/Upwards on edges, random doubles over and beyond the f32 range, f32 values and the
 * midpoints between neighbouring f32 values (ties for the cast).
 */
static struct test_Output FloatTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    /* FLT_TRUE_MIN without C11's float.h: 2^-126 * 2^-23, exact */
    const f32 true_min = FLT_MIN * FLT_EPSILON;
    TEST_TRUE(F32_MAX_POSITIVE_NORMAL == FLT_MAX && F32_MIN_NEGATIVE_NORMAL == -FLT_MAX);
    TEST_TRUE(F32_MIN_POSITIVE_NORMAL == FLT_MIN && F32_MAX_NEGATIVE_NORMAL == -FLT_MIN);
    TEST_TRUE(F32_MIN_POSITIVE_SUBNORMAL == true_min && F32_MAX_NEGATIVE_SUBNORMAL == -true_min);
    TEST_TRUE(F32_MAX_POSITIVE_SUBNORMAL == FLT_MIN - true_min && F32_MIN_NEGATIVE_SUBNORMAL == -(FLT_MIN - true_min));
    TEST_TRUE(F32_EPSILON == FLT_EPSILON);
    TEST_TRUE(F32_SIGNIFICAND_LENGTH + 1 == FLT_MANT_DIG);
    TEST_TRUE(F32_BIAS == FLT_MAX_EXP - 1 && F32_MAX_EXPONENT == FLT_MAX_EXP - 1 && F32_MIN_EXPONENT == FLT_MIN_EXP - 1);
    TEST_TRUE(F32_PI == (f32) 3.14159265358979323846 && F32_PI2 == 2.0f * F32_PI);
    TEST_TRUE(F32_INFINITY > FLT_MAX && F32Inf(1) < -FLT_MAX);
    TEST_EQUAL(0, FloatTestBitsOf(F32Zero(0)));
    TEST_EQUAL(F32_SIGN_MASK, FloatTestBitsOf(F32Zero(1)));
    TEST_TRUE(F32TestNan(F32Nan()));

    struct rng rng;
    RngLocalInit(&rng, env->seed);

    for (u32 sign = 0; sign <= 1; ++sign)
    {
        for (u32 exponent = 0; exponent <= 255; ++exponent)
        {
            for (u32 k = 0; k < 4; ++k)
            {
                const u32 significand = (k == 0) ? 0
                    : (k == 1) ? 1
                    : (k == 2) ? F32_SIGNIFICAND_MASK
                    : (u32) RngLocalU64Range(&rng, 1, F32_SIGNIFICAND_MASK);
                const u32 bits = (sign << 31) | (exponent << 23) | significand;
                const f32 f = FloatTestBits(bits);

                const u32 nan = (f != f);
                const u32 inf = !nan && (f > FLT_MAX || f < -FLT_MAX);
                const u32 zero = (f == 0.0f);
                const u32 subnormal = !zero && -FLT_MIN < f && f < FLT_MIN;
                const u32 normal = !nan && !inf && !zero && !subnormal;
                const enum ieee_type type = nan ? IEEE_NAN
                    : inf ? IEEE_INF
                    : zero ? IEEE_ZERO
                    : subnormal ? IEEE_SUBNORMAL
                    : IEEE_NORMAL;

                TEST_EQUAL(bits, FloatTestBitsOf(F32Construct(sign, exponent, significand)));
                TEST_EQUAL(sign, F32SignBit(f));
                TEST_EQUAL(exponent, F32ExponentBits(f));
                TEST_EQUAL(significand, F32MantissaBits(f));
                TEST_EQUAL(nan, F32TestNan(f));
                TEST_EQUAL(!nan && !inf, F32FiniteCheck(f));
                TEST_EQUAL(inf && !sign, F32TestPositiveInf(f));
                TEST_EQUAL(inf && sign, F32TestNegativeInf(f));
                TEST_EQUAL(zero && !sign, F32TestPositiveZero(f));
                TEST_EQUAL(zero && sign, F32TestNegativeZero(f));
                TEST_EQUAL(normal, F32TestNormal(f));
                TEST_EQUAL(subnormal, F32TestSubnormal(f));
                TEST_EQUAL(type, F32Classify(f));
                TEST_EQUAL(bits & ~F32_SIGN_MASK, FloatTestBitsOf(F32Abs(f)));
                TEST_TRUE(F32Sign(f) == (sign ? -1.0f : 1.0f));
            }
        }
    }

    const f32 nan = F32Nan();
    TEST_TRUE(F32Max(1.0f, 2.0f) == 2.0f && F32Max(2.0f, 1.0f) == 2.0f);
    TEST_TRUE(F32Min(1.0f, 2.0f) == 1.0f && F32Min(2.0f, 1.0f) == 1.0f);
    TEST_TRUE(F32Clamp(-1.0f, 0.0f, 1.0f) == 0.0f && F32Clamp(2.0f, 0.0f, 1.0f) == 1.0f && F32Clamp(0.25f, 0.0f, 1.0f) == 0.25f);
    /* a NaN first argument gives the second, a NaN second argument is returned; Clamp passes NaN through */
    TEST_TRUE(F32Max(nan, 1.0f) == 1.0f && F32TestNan(F32Max(1.0f, nan)));
    TEST_TRUE(F32Min(nan, 1.0f) == 1.0f && F32TestNan(F32Min(1.0f, nan)));
    TEST_TRUE(F32TestNan(F32Clamp(nan, 0.0f, 1.0f)));
    /* ties to even */
    TEST_TRUE(F32Round(0.5f) == 0.0f && F32Round(1.5f) == 2.0f && F32Round(2.5f) == 2.0f && F32Round(-2.5f) == -2.0f);
    TEST_EQUAL(F32_SIGN_MASK, FloatTestBitsOf(F32Round(-0.5f)));

    /* edges: zeros, below the subnormals, the subnormal/normal border, around FLT_MAX and its rounding to inf */
    const f64 edge[] =
    {
        0.0, -0.0, 0x1p-150, -0x1p-150, 0x1.8p-150, 0x1p-200, -0x1p-200, 0x1p-1074,
        (f64) FLT_MIN, (f64) FLT_MIN - 0x1p-151, (f64) FLT_MIN + 0x1p-151, (f64) true_min,
        (f64) FLT_MAX, (f64) FLT_MAX + 0x1p103, -((f64) FLT_MAX + 0x1p103), (f64) FLT_MAX + 0x1p102,
        0x1p128, -0x1p128, 1e300, -1e300, 1.0, -1.0, 0.1, -0.1, 1.0 + 0x1p-24, 1.0 - 0x1p-25,
    };
    for (u32 i = 0; i < sizeof(edge) / sizeof(edge[0]); ++i)
    {
        TEST_TRUE(FloatTestDemoteCheck(edge[i]));
    }
    TEST_EQUAL(F32_SIGN_MASK, FloatTestBitsOf(F64DemoteDownwards(-0.0)));
    TEST_EQUAL(F32_SIGN_MASK, FloatTestBitsOf(F64DemoteUpwards(-0.0)));
    TEST_EQUAL(F32_SIGN_MASK, FloatTestBitsOf(F64DemoteUpwards(-0x1p-200)));
    TEST_TRUE(F32TestPositiveInf(F64DemoteDownwards((f64) F32_INFINITY)) && F32TestPositiveInf(F64DemoteUpwards((f64) F32_INFINITY)));
    TEST_TRUE(F32TestNegativeInf(F64DemoteDownwards((f64) F32Inf(1))) && F32TestNegativeInf(F64DemoteUpwards((f64) F32Inf(1))));
    TEST_TRUE(F32TestNan(F64DemoteDownwards((f64) nan)) && F32TestNan(F64DemoteUpwards((f64) nan)));

    for (u32 i = 0; i < 100000; ++i)
    {
        /* |x| in [2^-160, 2^140): below the f32 subnormals, the f32 range and beyond FLT_MAX */
        const u64 x_sign = RngLocalU64(&rng) & ((u64) 1 << 63);
        const u64 x_exponent = RngLocalU64Range(&rng, 1023 - 160, 1023 + 139);
        const u64 x_significand = RngLocalU64(&rng) >> 12;
        TEST_TRUE(FloatTestDemoteCheck(FloatTestF64(x_sign | (x_exponent << 52) | x_significand)));

        /* a nonzero finite f32 f below FLT_MAX in magnitude: f is exact, the midpoint above it a tie */
        const u32 f_sign = (u32) RngLocalU64Range(&rng, 0, 1) << 31;
        const u32 f_magnitude = (u32) RngLocalU64Range(&rng, 1, 0x7f7ffffe);
        const f32 f = FloatTestBits(f_sign | f_magnitude);
        const f32 f_up = FloatTestNextUp(f);
        const f64 midpoint = ((f64) f + (f64) f_up) / 2.0;
        TEST_EQUAL(FloatTestBitsOf(f), FloatTestBitsOf(F64DemoteDownwards((f64) f)));
        TEST_EQUAL(FloatTestBitsOf(f), FloatTestBitsOf(F64DemoteUpwards((f64) f)));
        TEST_EQUAL(FloatTestBitsOf(f), FloatTestBitsOf(F64DemoteDownwards(midpoint)));
        TEST_EQUAL(FloatTestBitsOf(f_up), FloatTestBitsOf(F64DemoteUpwards(midpoint)));
    }

	return output;
}

/* uniform in [min, max) from the top 24 bits: the same value on every compiler */
static f32 VectorTestF32(struct rng *rng, const f32 min, const f32 max)
{
    return min + (max - min) * ((f32) (RngLocalU64(rng) >> 40) * 0x1p-24f);
}

/* components drawn one statement at a time (argument evaluation order is unspecified) */
static v3 VectorTestV3(struct rng *rng, const f32 min, const f32 max)
{
    const f32 x = VectorTestF32(rng, min, max);
    const f32 y = VectorTestF32(rng, min, max);
    const f32 z = VectorTestF32(rng, min, max);
    return V3(x, y, z);
}

static u32 VectorTestNear(const v3 a, const v3 b, const f32 tol)
{
    return F32Abs(a.x - b.x) <= tol && F32Abs(a.y - b.y) <= tol && F32Abs(a.z - b.z) <= tol;
}

/*
 * ds_vector.h contracts: V3Dot sums left to right; V3Normalize gives unit length; V3CreateBasis gives an
 * orthonormal, left-handed (n1 x n2 = -n3) basis, on the axes, at both sides of its switch |n3.z| = 1/sqrt(2)
 * and for random normals; V3Support returns the lowest index among the maxima (ties made by repeated points,
 * dir = 0 included); V3Interpolate is exact at alpha 1 and 0; V3ParallelCheck around its threshold, antiparallel
 * and zero vectors; V3RotateY turns x toward -z; the V3F64 demotions bracket their input.
 */
static struct test_Output VectorTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct rng rng;
    RngLocalInit(&rng, env->seed);

    for (u32 i = 0; i < 10000; ++i)
    {
        const v3 a = VectorTestV3(&rng, -1000.0f, 1000.0f);
        const v3 b = VectorTestV3(&rng, -1000.0f, 1000.0f);
        TEST_EQUAL(FloatTestBitsOf((a.x*b.x + a.y*b.y) + a.z*b.z), FloatTestBitsOf(V3Dot(a, b)));

        TEST_TRUE(V3Interpolate(a, b, 1.0f).x == a.x && V3Interpolate(a, b, 1.0f).y == a.y && V3Interpolate(a, b, 1.0f).z == a.z);
        TEST_TRUE(V3Interpolate(a, b, 0.0f).x == b.x && V3Interpolate(a, b, 0.0f).y == b.y && V3Interpolate(a, b, 0.0f).z == b.z);

        if (V3Length(a) > 1e-3f)
        {
            TEST_TRUE(F32Abs(V3Length(V3Normalize(a)) - 1.0f) <= 1e-6f);
        }
    }

    /* the axes, both sides of the switch at |z| = 1/sqrt(2), then random normals */
    const f32 r = 1.0f / F32Sqrt(2.0f);
    const v3 normal[] =
    {
        V3(1.0f, 0.0f, 0.0f), V3(-1.0f, 0.0f, 0.0f), V3(0.0f, 1.0f, 0.0f), V3(0.0f, -1.0f, 0.0f),
        V3(0.0f, 0.0f, 1.0f), V3(0.0f, 0.0f, -1.0f),
        V3Normalize(V3(r, 0.0f, r * 0.999f)), V3Normalize(V3(r, 0.0f, r * 1.001f)),
        V3Normalize(V3(0.0f, -r, -r * 0.999f)), V3Normalize(V3(0.0f, -r, -r * 1.001f)),
    };
    const u32 normal_count = sizeof(normal) / sizeof(normal[0]);
    for (u32 i = 0; i < normal_count + 10000; ++i)
    {
        v3 n3;
        if (i < normal_count)
        {
            n3 = normal[i];
        }
        else
        {
            const v3 v = VectorTestV3(&rng, -1.0f, 1.0f);
            if (V3Length(v) < 0.1f) { continue; }
            n3 = V3Normalize(v);
        }

        v3 n1, n2;
        V3CreateBasis(&n1, &n2, n3);
        TEST_TRUE(F32Abs(V3Length(n1) - 1.0f) <= 1e-5f && F32Abs(V3Length(n2) - 1.0f) <= 1e-5f);
        TEST_TRUE(F32Abs(V3Dot(n1, n2)) <= 1e-5f && F32Abs(V3Dot(n1, n3)) <= 1e-5f && F32Abs(V3Dot(n2, n3)) <= 1e-5f);
        TEST_TRUE(VectorTestNear(V3Cross(n1, n2), V3Negate(n3), 1e-5f));
    }

    for (u32 i = 0; i < 1000; ++i)
    {
        v3 v[32];
        const u32 count = (u32) RngLocalU64Range(&rng, 1, 32);
        for (u32 k = 0; k < count; ++k)
        {
            v[k] = VectorTestV3(&rng, -1.0f, 1.0f);
        }
        /* a repeated point makes a tie, which the lower index wins */
        const u32 from = (u32) RngLocalU64Range(&rng, 0, count - 1);
        const u32 to = (u32) RngLocalU64Range(&rng, 0, count - 1);
        v[to] = v[from];

        const v3 dir = (i % 10 == 0) ? V3Zero() : VectorTestV3(&rng, -1.0f, 1.0f);
        const u32 best = V3Support(v, count, dir);
        TEST_TRUE(best < count);
        const f32 best_dot = V3Dot(v[best], dir);
        for (u32 k = 0; k < count; ++k)
        {
            TEST_TRUE(V3Dot(v[k], dir) <= best_dot);
            TEST_TRUE(k >= best || V3Dot(v[k], dir) < best_dot);
        }
    }

    /* b at angle theta from a, both lengths arbitrary; the threshold is 0.5 degrees */
    const f32 eps = V3ParallelCheckEpsilon(0.5f);
    const v3 a = V3Scale(V3Normalize(V3(1.0f, 2.0f, 3.0f)), 7.0f);
    const v3 p = V3Normalize(V3Cross(a, V3(0.0f, 0.0f, 1.0f)));
    const f32 degrees[] = { 0.0f, 0.25f, 0.45f, 0.55f, 1.0f, 90.0f };
    const u32 parallel[] = { 1, 1, 1, 0, 0, 0 };
    for (u32 i = 0; i < sizeof(degrees) / sizeof(degrees[0]); ++i)
    {
        const f32 theta = degrees[i] * F32_PI / 180.0f;
        const v3 b = V3Scale(V3Add(V3Scale(V3Normalize(a), F32Cos(theta)), V3Scale(p, F32Sin(theta))), 0.3f);
        TEST_EQUAL(parallel[i], V3ParallelCheck(a, b, eps));
        TEST_EQUAL(parallel[i], V3ParallelCheck(a, V3Negate(b), eps));
    }
    TEST_EQUAL(0, V3ParallelCheck(a, V3Zero(), eps));
    TEST_EQUAL(0, V3ParallelCheck(V3Zero(), V3Zero(), eps));

    TEST_TRUE(VectorTestNear(V3RotateY(V3(1.0f, 0.0f, 0.0f), F32_PI / 2.0f), V3(0.0f, 0.0f, -1.0f), 1e-6f));
    TEST_TRUE(VectorTestNear(V3RotateY(V3(0.0f, 0.0f, 1.0f), F32_PI / 2.0f), V3(1.0f, 0.0f, 0.0f), 1e-6f));

    const v3f64 w = V3F64(0.1, -1e-30, 3e9 + 0.3);
    const v3 w_down = V3F64DemoteDownwards(w);
    const v3 w_up = V3F64DemoteUpwards(w);
    const v3 w_near = V3F64Demote(w);
    for (u32 k = 0; k < 3; ++k)
    {
        TEST_TRUE((f64) w_down.buf[k] <= w.buf[k] && w.buf[k] <= (f64) w_up.buf[k]);
        TEST_TRUE(w_down.buf[k] <= w_near.buf[k] && w_near.buf[k] <= w_up.buf[k]);
    }

	return output;
}

static u32 QuaternionTestEqual(const q a, const q b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

static u32 QuaternionTestNear(const q a, const q b, const f32 tol)
{
    return F32Abs(a.x - b.x) <= tol && F32Abs(a.y - b.y) <= tol && F32Abs(a.z - b.z) <= tol && F32Abs(a.w - b.w) <= tol;
}

/* a random unit quaternion, components drawn one statement at a time */
static q QuaternionTestUnit(struct rng *rng)
{
    while (1)
    {
        const v3 v = VectorTestV3(rng, -1.0f, 1.0f);
        const f32 w = VectorTestF32(rng, -1.0f, 1.0f);
        const q a = Q(v.x, v.y, v.z, w);
        if (QNorm(a) >= 0.1f)
        {
            return QNormalize(a);
        }
    }
}

/*
 * ds_quaternion.h conventions: the Hamilton table (ij = k, ...); QUnitAxisAngle by the right-hand rule and against
 * Rodrigues' formula for random axes, angles and vectors; QAxisAngle normalizes its axis; QMul(a, b) rotates by b
 * first; QConj and QInverse invert; QYUnitRotate equals QUnitRotate on (0, y, 0); QNormalize and QNorm.
 */
static struct test_Output QuaternionTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const q one = QIdentity();
    const q i = Q(1.0f, 0.0f, 0.0f, 0.0f);
    const q j = Q(0.0f, 1.0f, 0.0f, 0.0f);
    const q k = Q(0.0f, 0.0f, 1.0f, 0.0f);
    const q minus_one = Q(0.0f, 0.0f, 0.0f, -1.0f);
    TEST_TRUE(QuaternionTestEqual(QMul(i, i), minus_one) && QuaternionTestEqual(QMul(j, j), minus_one) && QuaternionTestEqual(QMul(k, k), minus_one));
    TEST_TRUE(QuaternionTestEqual(QMul(i, j), k) && QuaternionTestEqual(QMul(j, i), QScale(k, -1.0f)));
    TEST_TRUE(QuaternionTestEqual(QMul(j, k), i) && QuaternionTestEqual(QMul(k, j), QScale(i, -1.0f)));
    TEST_TRUE(QuaternionTestEqual(QMul(k, i), j) && QuaternionTestEqual(QMul(i, k), QScale(j, -1.0f)));
    TEST_TRUE(QuaternionTestEqual(QMul(one, i), i) && QuaternionTestEqual(QMul(k, one), k));

    /* right-hand rule: a quarter turn about z takes x to y, about x takes y to z, about y takes z to x */
    const v3 x_axis = V3(1.0f, 0.0f, 0.0f);
    const v3 y_axis = V3(0.0f, 1.0f, 0.0f);
    const v3 z_axis = V3(0.0f, 0.0f, 1.0f);
    TEST_TRUE(VectorTestNear(QUnitRotate(QUnitAxisAngle(z_axis, F32_PI / 2.0f), x_axis), y_axis, 1e-6f));
    TEST_TRUE(VectorTestNear(QUnitRotate(QUnitAxisAngle(x_axis, F32_PI / 2.0f), y_axis), z_axis, 1e-6f));
    TEST_TRUE(VectorTestNear(QUnitRotate(QUnitAxisAngle(y_axis, F32_PI / 2.0f), z_axis), x_axis, 1e-6f));

    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 10000; ++n)
    {
        const v3 axis = VectorTestV3(&rng, -1.0f, 1.0f);
        const f32 theta = VectorTestF32(&rng, -F32_PI, F32_PI);
        const v3 v = VectorTestV3(&rng, -1.0f, 1.0f);
        const f32 s = VectorTestF32(&rng, 0.1f, 10.0f);
        const f32 y = VectorTestF32(&rng, -10.0f, 10.0f);
        const q a = QuaternionTestUnit(&rng);
        const q b = QuaternionTestUnit(&rng);
        if (V3Length(axis) < 0.1f) { continue; }
        const v3 u = V3Normalize(axis);

        /* Rodrigues: v cos t + (u x v) sin t + u (u.v)(1 - cos t) */
        const f32 c = F32Cos(theta);
        const f32 sn = F32Sin(theta);
        const v3 rodrigues = V3Add(V3Add(V3Scale(v, c), V3Scale(V3Cross(u, v), sn)), V3Scale(u, V3Dot(u, v) * (1.0f - c)));
        const q r = QUnitAxisAngle(u, theta);
        TEST_TRUE(VectorTestNear(QUnitRotate(r, v), rodrigues, 1e-5f));
        TEST_TRUE(QuaternionTestNear(QAxisAngle(V3Scale(u, s), theta), r, 1e-6f));

        TEST_TRUE(VectorTestNear(QUnitRotate(QMul(a, b), v), QUnitRotate(a, QUnitRotate(b, v)), 1e-5f));
        TEST_TRUE(QuaternionTestNear(QMul(a, QConj(a)), one, 1e-6f));
        const q c3 = QScale(a, 3.0f);
        TEST_TRUE(QuaternionTestNear(QMul(c3, QInverse(c3)), one, 1e-6f));
        TEST_TRUE(F32Abs(QNorm(c3) - 3.0f) <= 1e-5f && F32Abs(QNorm(QNormalize(c3)) - 1.0f) <= 1e-6f);

        const v3 rotated = QUnitRotate(a, V3(0.0f, y, 0.0f));
        const v3 rotated_y = QYUnitRotate(a, y);
        TEST_TRUE(rotated.x == rotated_y.x && rotated.y == rotated_y.y && rotated.z == rotated_y.z);
    }

	return output;
}

/* determinant of the n x n column-major buf by Gaussian elimination with partial pivoting, in f64 */
static f64 MatrixTestDeterminant(const f32 *buf, const u32 n)
{
    f64 a[4][4];
    for (u32 i = 0; i < n; ++i)
    {
        for (u32 j = 0; j < n; ++j)
        {
            a[i][j] = (f64) buf[j*n + i];
        }
    }

    f64 det = 1.0;
    for (u32 c = 0; c < n; ++c)
    {
        u32 pivot = c;
        for (u32 r = c + 1; r < n; ++r)
        {
            const f64 m_r = (a[r][c] < 0.0) ? -a[r][c] : a[r][c];
            const f64 m_p = (a[pivot][c] < 0.0) ? -a[pivot][c] : a[pivot][c];
            if (m_r > m_p) { pivot = r; }
        }
        if (a[pivot][c] == 0.0) { return 0.0; }
        if (pivot != c)
        {
            for (u32 j = 0; j < n; ++j) { const f64 t = a[c][j]; a[c][j] = a[pivot][j]; a[pivot][j] = t; }
            det = -det;
        }
        det *= a[c][c];
        for (u32 r = c + 1; r < n; ++r)
        {
            const f64 f = a[r][c] / a[c][c];
            for (u32 j = c; j < n; ++j) { a[r][j] -= f * a[c][j]; }
        }
    }
    return det;
}

/* n x n column-major entries in [-1, 1), plus diag on the diagonal (diagonally dominant for diag > n) */
static void MatrixTestRandom(f32 *buf, struct rng *rng, const u32 n, const f32 diag)
{
    for (u32 j = 0; j < n; ++j)
    {
        for (u32 i = 0; i < n; ++i)
        {
            const f32 e = VectorTestF32(rng, -1.0f, 1.0f);
            buf[j*n + i] = (i == j) ? e + diag : e;
        }
    }
}

/* max |(a b)_ij - I_ij| for n x n column-major a, b */
static f32 MatrixTestIdentityError(const f32 *a, const f32 *b, const u32 n)
{
    f32 error = 0.0f;
    for (u32 i = 0; i < n; ++i)
    {
        for (u32 j = 0; j < n; ++j)
        {
            f32 sum = 0.0f;
            for (u32 k = 0; k < n; ++k) { sum += a[k*n + i] * b[j*n + k]; }
            error = F32Max(error, F32Abs(sum - ((i == j) ? 1.0f : 0.0f)));
        }
    }
    return error;
}

/*
 * ds_matrix.h/.c: Rows is the transpose of Columns; V3M3Mul(v, m) equals M3V3Mul(m^T, v) bit for bit; M3Mul and
 * M3V3Mul against f64 products; OuterProduct; AbsMin/AbsMax; M3QUnit against QUnitRotate and M4QUnit's 3x3 block;
 * the 2x2, 3x3 and 4x4 inverses: determinant against f64 elimination, a inv = I, and exactly singular integer
 * matrices returning determinant 0.
 */
static struct test_Output MatrixTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 2000; ++n)
    {
        m3 a, b;
        MatrixTestRandom(a.buf, &rng, 3, 0.0f);
        MatrixTestRandom(b.buf, &rng, 3, 0.0f);
        const v3 v = VectorTestV3(&rng, -1.0f, 1.0f);
        const v3 w = VectorTestV3(&rng, -1.0f, 1.0f);

        const m3 rows = M3Rows(a.col[0], a.col[1], a.col[2]);
        const m3 at = M3Transpose(a);
        for (u32 k = 0; k < 9; ++k) { TEST_TRUE(rows.buf[k] == at.buf[k]); }
        TEST_TRUE(rows.a12 == a.a21 && rows.a13 == a.a31 && rows.a23 == a.a32);

        const v3 row_product = V3M3Mul(v, a);
        const v3 column_product = M3V3Mul(at, v);
        TEST_EQUAL(FloatTestBitsOf(row_product.x), FloatTestBitsOf(column_product.x));
        TEST_EQUAL(FloatTestBitsOf(row_product.y), FloatTestBitsOf(column_product.y));
        TEST_EQUAL(FloatTestBitsOf(row_product.z), FloatTestBitsOf(column_product.z));

        const v3 av = M3V3Mul(a, v);
        const m3 ab = M3Mul(a, b);
        for (u32 i = 0; i < 3; ++i)
        {
            f64 ref_av = 0.0;
            for (u32 k = 0; k < 3; ++k) { ref_av += (f64) a.buf[k*3 + i] * (f64) v.buf[k]; }
            TEST_TRUE(F32Abs(av.buf[i] - (f32) ref_av) <= 1e-6f);
            for (u32 j = 0; j < 3; ++j)
            {
                f64 ref_ab = 0.0;
                for (u32 k = 0; k < 3; ++k) { ref_ab += (f64) a.buf[k*3 + i] * (f64) b.buf[j*3 + k]; }
                TEST_TRUE(F32Abs(ab.buf[j*3 + i] - (f32) ref_ab) <= 1e-6f);
            }
        }
        TEST_TRUE(VectorTestNear(M3V3Mul(M3OuterProduct(v, w), a.col[0]), V3Scale(v, V3Dot(w, a.col[0])), 4e-6f));

        f32 abs_min = F32Abs(a.buf[0]);
        f32 abs_max = F32Abs(a.buf[0]);
        for (u32 k = 1; k < 9; ++k)
        {
            abs_min = F32Min(abs_min, F32Abs(a.buf[k]));
            abs_max = F32Max(abs_max, F32Abs(a.buf[k]));
        }
        TEST_TRUE(M3AbsMin(a) == abs_min && M3AbsMax(a) == abs_max);

        const q r = QuaternionTestUnit(&rng);
        const m3 rot = M3QUnit(r);
        const m4 rot4 = M4QUnit(r);
        TEST_TRUE(VectorTestNear(M3V3Mul(rot, v), QUnitRotate(r, v), 4e-6f));
        for (u32 j = 0; j < 3; ++j)
        {
            TEST_TRUE(rot4.col[j].x == rot.col[j].x && rot4.col[j].y == rot.col[j].y && rot4.col[j].z == rot.col[j].z);
            TEST_TRUE(rot4.col[j].w == 0.0f && rot4.col[3].buf[j] == 0.0f);
        }
        TEST_TRUE(rot4.a44 == 1.0f);

        /* inverses of well-conditioned (diagonally dominant) matrices */
        m2 c2, inv2;
        m3 c3, inv3;
        m4 c4, inv4;
        MatrixTestRandom(c2.buf, &rng, 2, 3.0f);
        MatrixTestRandom(c3.buf, &rng, 3, 4.0f);
        MatrixTestRandom(c4.buf, &rng, 4, 5.0f);
        const f32 det2 = M2Inverse(&inv2, c2);
        const f32 det3 = M3Inverse(&inv3, c3);
        const f32 det4 = M4Inverse(&inv4, c4);
        const f64 ref2 = MatrixTestDeterminant(c2.buf, 2);
        const f64 ref3 = MatrixTestDeterminant(c3.buf, 3);
        const f64 ref4 = MatrixTestDeterminant(c4.buf, 4);
        TEST_TRUE(F32Abs(det2 - (f32) ref2) <= 1e-5f * F32Abs((f32) ref2));
        TEST_TRUE(F32Abs(det3 - (f32) ref3) <= 1e-5f * F32Abs((f32) ref3));
        TEST_TRUE(F32Abs(det4 - (f32) ref4) <= 1e-5f * F32Abs((f32) ref4));
        TEST_TRUE(MatrixTestIdentityError(c2.buf, inv2.buf, 2) <= 1e-5f && MatrixTestIdentityError(inv2.buf, c2.buf, 2) <= 1e-5f);
        TEST_TRUE(MatrixTestIdentityError(c3.buf, inv3.buf, 3) <= 1e-5f && MatrixTestIdentityError(inv3.buf, c3.buf, 3) <= 1e-5f);
        TEST_TRUE(MatrixTestIdentityError(c4.buf, inv4.buf, 4) <= 1e-5f && MatrixTestIdentityError(inv4.buf, c4.buf, 4) <= 1e-5f);
    }

    /* exactly singular: a column (or row) is a combination of others; small integers keep every product exact */
    m2 inv2;
    m3 inv3;
    m4 inv4;
    TEST_TRUE(M2Inverse(&inv2, M2(1.0f, 2.0f, 2.0f, 4.0f)) == 0.0f);
    TEST_TRUE(M3Inverse(&inv3, M3(1.0f, 2.0f, 3.0f,  4.0f, 5.0f, 6.0f,  5.0f, 7.0f, 9.0f)) == 0.0f);
    TEST_TRUE(M4Inverse(&inv4, M4(1.0f, 2.0f, 3.0f, 4.0f,  2.0f, 0.0f, 1.0f, 1.0f,  3.0f, 2.0f, 4.0f, 5.0f,  0.0f, 1.0f, 0.0f, 2.0f)) == 0.0f);

	return output;
}

/* m (p, 1) */
static v4 TransformTestApply(const m4 m, const v3 p)
{
    return M4V4Mul(m, V4(p.x, p.y, p.z, 1.0f));
}

/*
 * transform.h conventions: M4Perspective's near, far and side planes land on clip -1/+1; M4View takes position to
 * the origin and forward/up/left to -z/+y/-x; M3SequentialRotation is R(axis_1) R(axis_2); M3Rotation takes any
 * axis length; V3RotateCenter keeps the distance to the center.
 */
static struct test_Output TransformTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 aspect = 16.0f / 9.0f;
    const f32 fov_x = F32_PI / 2.5f;
    const f32 z_near = 0.1f;
    const f32 z_far = 100.0f;
    const f32 t = F32Tan(fov_x / 2.0f);
    const m4 projection = M4Perspective(aspect, fov_x, z_near, z_far);
    const v4 p_near = TransformTestApply(projection, V3(0.0f, 0.0f, -z_near));
    const v4 p_far = TransformTestApply(projection, V3(0.0f, 0.0f, -z_far));
    const v4 p_side = TransformTestApply(projection, V3(t * 7.0f, 0.0f, -7.0f));
    const v4 p_top = TransformTestApply(projection, V3(0.0f, -t * 7.0f / aspect, -7.0f));
    TEST_TRUE(F32Abs(p_near.z / p_near.w + 1.0f) <= 1e-5f && F32Abs(p_far.z / p_far.w - 1.0f) <= 1e-5f);
    TEST_TRUE(F32Abs(p_side.x / p_side.w - 1.0f) <= 1e-5f && F32Abs(p_top.y / p_top.w + 1.0f) <= 1e-5f);

    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 2000; ++n)
    {
        const v3 position = VectorTestV3(&rng, -100.0f, 100.0f);
        const v3 offset = VectorTestV3(&rng, -10.0f, 10.0f);
        const v3 axis_1 = VectorTestV3(&rng, -1.0f, 1.0f);
        const v3 axis_2 = VectorTestV3(&rng, -1.0f, 1.0f);
        const f32 angle_1 = VectorTestF32(&rng, -F32_PI, F32_PI);
        const f32 angle_2 = VectorTestF32(&rng, -F32_PI, F32_PI);
        const f32 scale = VectorTestF32(&rng, 0.5f, 4.0f);
        const q r = QuaternionTestUnit(&rng);
        if (V3Length(axis_1) < 0.1f || V3Length(axis_2) < 0.1f) { continue; }

        /* a right-handed camera frame: left = up x forward */
        const v3 forward = QUnitRotate(r, V3(1.0f, 0.0f, 0.0f));
        const v3 up = QUnitRotate(r, V3(0.0f, 1.0f, 0.0f));
        const v3 left = QUnitRotate(r, V3(0.0f, 0.0f, -1.0f));
        const m4 view = M4View(position, left, up, forward);
        const v4 o = TransformTestApply(view, position);
        const v4 f = TransformTestApply(view, V3Add(position, forward));
        const v4 u = TransformTestApply(view, V3Add(position, up));
        const v4 l = TransformTestApply(view, V3Add(position, left));
        TEST_TRUE(VectorTestNear(V3(o.x, o.y, o.z), V3Zero(), 1e-4f) && o.w == 1.0f);
        TEST_TRUE(VectorTestNear(V3(f.x, f.y, f.z), V3(0.0f, 0.0f, -1.0f), 1e-4f));
        TEST_TRUE(VectorTestNear(V3(u.x, u.y, u.z), V3(0.0f, 1.0f, 0.0f), 1e-4f));
        TEST_TRUE(VectorTestNear(V3(l.x, l.y, l.z), V3(-1.0f, 0.0f, 0.0f), 1e-4f));

        const m3 sequential = M3SequentialRotation(axis_1, angle_1, axis_2, angle_2);
        const m3 product = M3Mul(M3Rotation(axis_1, angle_1), M3Rotation(axis_2, angle_2));
        for (u32 k = 0; k < 9; ++k) { TEST_TRUE(F32Abs(sequential.buf[k] - product.buf[k]) <= 1e-5f); }
        const m3 rotation = M3Rotation(axis_1, angle_1);
        const m3 rotation_scaled = M3Rotation(V3Scale(axis_1, scale), angle_1);
        for (u32 k = 0; k < 9; ++k) { TEST_TRUE(F32Abs(rotation.buf[k] - rotation_scaled.buf[k]) <= 1e-6f); }
        const v3 rotated = V3RotateCenter(rotation, position, V3Add(position, offset));
        TEST_TRUE(F32Abs(V3Distance(rotated, position) - V3Length(offset)) <= 1e-4f);
    }

	return output;
}

/* a world position about 10^7 m from the origin on every axis (f32's spacing there is 1 m) */
static v3f64 PoseTestFar(struct rng *rng)
{
    const v3 r = VectorTestV3(rng, -1.0f, 1.0f);
    return V3F64(1.0e7 + 1000.0 * r.x, -2.0e7 + 1000.0 * r.y, 3.0e7 + 1000.0 * r.z);
}

/*
 * ds_math.h poses: world positions about 10^7 m from the origin keep f32 precision relative to the frames'
 * separation (a point 1 m from a frame maps back within 1e-6 m; f32 positions there would be off by up to 0.5 m);
 * a world relative pose composes back to the target and maps points as going through the world; the same for
 * local poses; ds_TransformPromote is exact; the identities map points to themselves.
 */
static struct test_Output PoseTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 2000; ++n)
    {
        const q reference_rotation = QuaternionTestUnit(&rng);
        const v3f64 reference_position = PoseTestFar(&rng);
        const q target_rotation = QuaternionTestUnit(&rng);
        const v3 offset = VectorTestV3(&rng, -10.0f, 10.0f);
        const v3 p = VectorTestV3(&rng, -1.0f, 1.0f);
        const v3 local_position = VectorTestV3(&rng, -10.0f, 10.0f);

        const ds_TransformWorld reference = { .rotation = reference_rotation, .position = reference_position };
        const ds_TransformWorld target = { .rotation = target_rotation, .position = V3F64Add(reference_position, V3Promote(offset)) };

        const v3 back = ds_TransformWorldPointToLocal(&reference, ds_TransformWorldPointToWorld(&reference, p));
        TEST_TRUE(VectorTestNear(back, p, 1e-6f));

        const ds_Transform relative = ds_TransformWorldRelative(&reference, &target);
        const ds_TransformWorld composed = ds_TransformWorldChild(&reference, &relative);
        TEST_TRUE(QuaternionTestNear(composed.rotation, target.rotation, 1e-6f));
        TEST_TRUE(VectorTestNear(V3F64Demote(V3F64Sub(composed.position, target.position)), V3Zero(), 1e-5f));

        const v3 via_world = ds_TransformWorldPointToLocal(&reference, ds_TransformWorldPointToWorld(&target, p));
        const v3 via_relative = V3Add(QUnitRotate(relative.rotation, p), relative.position);
        TEST_TRUE(VectorTestNear(via_world, via_relative, 1e-5f));

        const ds_Transform a = { .rotation = reference_rotation, .position = local_position };
        const ds_Transform b = { .rotation = target_rotation, .position = V3Add(local_position, offset) };
        const ds_Transform local_relative = ds_TransformRelative(&a, &b);
        const v3 local_via_parent = ds_TransformPointToLocal(&a, V3Add(QUnitRotate(b.rotation, p), b.position));
        TEST_TRUE(VectorTestNear(local_via_parent, V3Add(QUnitRotate(local_relative.rotation, p), local_relative.position), 1e-5f));

        const ds_TransformWorld promoted = ds_TransformPromote(&a);
        TEST_TRUE(QuaternionTestEqual(promoted.rotation, a.rotation));
        TEST_TRUE(promoted.position.x == (f64) a.position.x && promoted.position.y == (f64) a.position.y && promoted.position.z == (f64) a.position.z);

        const ds_Transform identity = ds_TransformIdentity();
        const ds_TransformWorld world_identity = ds_TransformWorldIdentity();
        const v3 p_identity = ds_TransformPointToLocal(&identity, p);
        const v3 p_world_identity = ds_TransformWorldPointToLocal(&world_identity, V3Promote(p));
        TEST_TRUE(p_identity.x == p.x && p_identity.y == p.y && p_identity.z == p.z);
        TEST_TRUE(p_world_identity.x == p.x && p_world_identity.y == p.y && p_world_identity.z == p.z);
    }

	return output;
}

/*
 * r_CameraInit: the camera looks along direction (pitch within r_CameraUpdateAngles' clamp), its frame is
 * orthonormal with left = up x forward, and r_CameraUpdateAxes on its angles gives the same frame (the frame and
 * the angles agree). Straight down: yaw 0, pitch clamped. yaw = pitch = 0 looks along +z.
 */
static struct test_Output CameraTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 pitch_max = F32_PI / 2.0f - 0.50f;
    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 2000; ++n)
    {
        const v3 d = VectorTestV3(&rng, -1.0f, 1.0f);
        const f32 len = V3Length(d);
        if (len < 0.1f || F32Abs(d.y) > len * F32Sin(pitch_max) * 0.999f) { continue; }

        struct r_Camera cam = r_CameraInit(V3F64(1.0, 2.0, 3.0), d, 0.1f, 100.0f, 1.5f, F32_PI / 2.0f);
        TEST_TRUE(VectorTestNear(cam.forward, V3Scale(d, 1.0f / len), 1e-5f));
        TEST_TRUE(VectorTestNear(cam.left, V3Cross(cam.up, cam.forward), 1e-5f));
        TEST_TRUE(F32Abs(V3Dot(cam.up, cam.forward)) <= 1e-5f && F32Abs(V3Length(cam.up) - 1.0f) <= 1e-5f);

        const v3 forward = cam.forward;
        const v3 left = cam.left;
        const v3 up = cam.up;
        r_CameraUpdateAxes(&cam);
        TEST_TRUE(VectorTestNear(cam.forward, forward, 1e-6f) && VectorTestNear(cam.left, left, 1e-6f) && VectorTestNear(cam.up, up, 1e-6f));
    }

    const struct r_Camera down = r_CameraInit(V3F64(0.0, 0.0, 0.0), V3(0.0f, -2.0f, 0.0f), 0.1f, 100.0f, 1.5f, F32_PI / 2.0f);
    TEST_TRUE(down.yaw == 0.0f && down.pitch == pitch_max);
    const struct r_Camera ahead = r_CameraInit(V3F64(0.0, 0.0, 0.0), V3(0.0f, 0.0f, 3.0f), 0.1f, 100.0f, 1.5f, F32_PI / 2.0f);
    TEST_TRUE(ahead.yaw == 0.0f && ahead.pitch == 0.0f && VectorTestNear(ahead.forward, V3(0.0f, 0.0f, 1.0f), 1e-6f));

	return output;
}

/*
 * CstrDsKeycode / CstrButton: the keys around the old misordering (springclean #135: PLUS and MINUS were listed
 * after ENTER) by name, and every key and button has a name of its own (no NULL from an entry left out).
 */
static struct test_Output InputNameTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct { enum dsKeycode key; const char *name; } pair[] =
    {
        { DS_ENTER, "DS_ENTER" }, { DS_F1, "DS_F1" }, { DS_F12, "DS_F12" }, { DS_TAB, "DS_TAB" },
        { DS_RIGHT, "DS_RIGHT" }, { DS_DELETE, "DS_DELETE" }, { DS_PLUS, "DS_PLUS" }, { DS_MINUS, "DS_MINUS" },
        { DS_HOME, "DS_HOME" }, { DS_0, "DS_0" }, { DS_A, "DS_A" }, { DS_Z, "DS_Z" }, { DS_NO_SYMBOL, "DS_NO_SYMBOL" },
    };
    for (u32 i = 0; i < sizeof(pair) / sizeof(pair[0]); ++i)
    {
        TEST_TRUE(CstrDsKeycode(pair[i].key) != NULL && strcmp(CstrDsKeycode(pair[i].key), pair[i].name) == 0);
    }

    for (u32 a = 0; a < DS_KEY_COUNT; ++a)
    {
        TEST_TRUE(CstrDsKeycode((enum dsKeycode) a) != NULL);
        for (u32 b = 0; b < a; ++b)
        {
            TEST_TRUE(strcmp(CstrDsKeycode((enum dsKeycode) a), CstrDsKeycode((enum dsKeycode) b)) != 0);
        }
    }
    for (u32 a = 0; a < MOUSE_BUTTON_COUNT; ++a)
    {
        TEST_TRUE(CstrButton((enum mouseButton) a) != NULL);
    }
    TEST_TRUE(strcmp(CstrButton(MOUSE_BUTTON_SCROLL), "MOUSE_BUTTON_SCROLL") == 0);

	return output;
}

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <fcntl.h>

#define FS_TEST_DIR "ds_filesystem_test"

/* remove what FilesystemTest creates (also the leftovers of an interrupted run); C library, test only */
static void FilesystemTestCleanup(void)
{
    const char *entry[] = { FS_TEST_DIR "/a.txt", FS_TEST_DIR "/dangling", FS_TEST_DIR "/sub" };
    for (u32 i = 0; i < sizeof(entry) / sizeof(entry[0]); ++i)
    {
        unlink(entry[i]);
        rmdir(entry[i]);
    }
    rmdir(FS_TEST_DIR);
}

/*
 * The file API in a scratch directory under the working directory: create (and refuse an existing name without
 * truncate), write, append, write at an offset, dump. Directories report FILE_DIRECTORY and refuse regular files
 * (FS_TYPE_INVALID, springclean #142); a listing with a dangling symbolic link succeeds and lists it as
 * FILE_UNRECOGNIZED (#141). A /sys file whose fstat size (a page) exceeds its contents dumps its contents (#140:
 * the read loop hung). CwdGet on an arena too small for the path returns the empty path (#143).
 */
static struct test_Output FilesystemTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    struct arena *mem = env->mem_1;

    FilesystemTestCleanup();
    struct file dir = FileNull();
    TEST_EQUAL(FS_SUCCESS, DirectoryTryCreateAtCwd(mem, &dir, FS_TEST_DIR));
    TEST_EQUAL(FILE_DIRECTORY, dir.type);

    struct file file = FileNull();
    TEST_EQUAL(FS_SUCCESS, FileTryCreate(mem, &file, "a.txt", &dir, 0));
    TEST_EQUAL(FILE_REGULAR, file.type);
    TEST_EQUAL(5, FileWriteAppend(&file, (const u8 *) "hello", 5));
    TEST_EQUAL(6, FileWriteAppend(&file, (const u8 *) " world", 6));
    TEST_EQUAL(1, FileWriteOffset(&file, (const u8 *) "J", 1, 0));
    FileClose(&file);

    struct file again = FileNull();
    TEST_EQUAL(FS_ALREADY_EXISTS, FileTryCreate(mem, &again, "a.txt", &dir, 0));

    const struct dsBuffer dump = FileDump(mem, "a.txt", &dir);
    TEST_TRUE(dump.data != NULL && dump.size == 11 && memcmp(dump.data, "Jello world", 11) == 0);

    struct file not_dir = FileNull();
    TEST_EQUAL(FS_TYPE_INVALID, DirectoryTryOpen(mem, &not_dir, "a.txt", &dir));

    struct file sub = FileNull();
    TEST_EQUAL(FS_SUCCESS, DirectoryTryCreate(mem, &sub, "sub", &dir));
    TEST_EQUAL(FILE_DIRECTORY, sub.type);
    FileClose(&sub);
    TEST_EQUAL(0, symlinkat("does_not_exist", dir.handle, "dangling"));

    ds_CPool(file) entries;
    ds_CPoolAlloc(NULL, entries, 8, GROWABLE);
    TEST_EQUAL(FS_SUCCESS, DirectoryPushEntries(mem, &entries, &dir));
    TEST_EQUAL(FILE_HANDLE_INVALID, dir.handle);
    u32 found = 0;
    for (u32 i = 0; i < entries.count; ++i)
    {
        const struct file *e = entries.buf + i;
        if (Utf8Equivalence(e->path, Utf8Inline("a.txt")))         { found |= 1; TEST_EQUAL(FILE_REGULAR, e->type); }
        else if (Utf8Equivalence(e->path, Utf8Inline("sub")))      { found |= 2; TEST_EQUAL(FILE_DIRECTORY, e->type); }
        else if (Utf8Equivalence(e->path, Utf8Inline("dangling"))) { found |= 4; TEST_EQUAL(FILE_UNRECOGNIZED, e->type); }
    }
    ds_CPoolDealloc(entries);
    TEST_EQUAL(7, found);

    /* a pseudo-file: fstat says a page, read() ends after its few bytes */
    const char *pseudo = "/sys/devices/system/cpu/online";
    if (access(pseudo, R_OK) == 0)
    {
        const struct dsBuffer online = FileDumpAtCwd(mem, pseudo);
        TEST_TRUE(online.data != NULL && 0 < online.size && online.size < 4096);
    }

    struct arena small = ArenaAlloc(NULL, 64);
    TEST_TRUE(small.stack_ptr != NULL);
    const utf8 cwd_small = CwdGet(&small);
    ArenaFree(&small);
    TEST_EQUAL(0, cwd_small.size);
    const utf8 cwd = CwdGet(mem);
    TEST_TRUE(cwd.size > 0 && cwd.buf[0] == '/');

    FilesystemTestCleanup();

	return output;
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <shlwapi.h>

#define FS_TEST_DIR "ds_filesystem_test"
/* U+00E5 U+00E4 U+00F6 as UTF-8 and UTF-16, escaped: MSVC reads the source in the ANSI code page */
#define FS_TEST_NON_ASCII "\xc3\xa5\xc3\xa4\xc3\xb6"
#define FS_TEST_NON_ASCII_W L"\x00e5\x00e4\x00f6"

/* remove what FilesystemTest creates (also the leftovers of an interrupted run); OS API, test only */
static void FilesystemTestCleanup(void)
{
    DeleteFileW(L"ds_filesystem_test\\a.txt");
    RemoveDirectoryW(L"ds_filesystem_test\\dangling");
    DeleteFileW(L"ds_filesystem_test\\dangling");
    RemoveDirectoryW(L"ds_filesystem_test\\sub");
    RemoveDirectoryW(L"ds_filesystem_test\\" FS_TEST_NON_ASCII_W);
    RemoveDirectoryW(L"ds_filesystem_test");
}

/*
 * The Windows twin of the Linux FilesystemTest, plus UTF-8 names: a non-ASCII directory name round-trips through
 * DirectoryTryCreate and DirectoryPushEntries. A dangling symbolic link (only if this account may create one:
 * developer mode or admin) is listed as FILE_UNRECOGNIZED. Checks for a springclean finding name it.
 */
static struct test_Output FilesystemTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    struct arena *mem = env->mem_1;

    FilesystemTestCleanup();
    struct file dir = FileNull();
    TEST_EQUAL(FS_SUCCESS, DirectoryTryCreateAtCwd(mem, &dir, FS_TEST_DIR));
    TEST_EQUAL(FILE_DIRECTORY, dir.type);

    struct file file = FileNull();
    TEST_EQUAL(FS_SUCCESS, FileTryCreate(mem, &file, "a.txt", &dir, 0));
    TEST_EQUAL(FILE_REGULAR, file.type);
    TEST_EQUAL(5, FileWriteAppend(&file, (const u8 *) "hello", 5));
    TEST_EQUAL(6, FileWriteAppend(&file, (const u8 *) " world", 6));
    TEST_EQUAL(1, FileWriteOffset(&file, (const u8 *) "J", 1, 0));
    FileClose(&file);

    struct file again = FileNull();
    TEST_EQUAL(FS_ALREADY_EXISTS, FileTryCreate(mem, &again, "a.txt", &dir, 0));

    const struct dsBuffer dump = FileDump(mem, "a.txt", &dir);
    TEST_TRUE(dump.data != NULL && dump.size == 11 && memcmp(dump.data, "Jello world", 11) == 0);

    /* #163: a regular file is no directory */
    struct file not_dir = FileNull();
    const enum fsError not_dir_err = DirectoryTryOpen(mem, &not_dir, "a.txt", &dir);
    if (not_dir.handle != FILE_HANDLE_INVALID) { FileClose(&not_dir); }
    TEST_EQUAL(FS_TYPE_INVALID, not_dir_err);

    struct file sub = FileNull();
    TEST_EQUAL(FS_SUCCESS, DirectoryTryCreate(mem, &sub, "sub", &dir));
    TEST_EQUAL(FILE_DIRECTORY, sub.type);
    FileClose(&sub);

    struct file non_ascii = FileNull();
    TEST_EQUAL(FS_SUCCESS, DirectoryTryCreate(mem, &non_ascii, FS_TEST_NON_ASCII, &dir));
    FileClose(&non_ascii);

    const u32 have_link = CreateSymbolicLinkW(L"ds_filesystem_test\\dangling", L"does_not_exist",
            SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE) != 0;

    const utf8 non_ascii_name = { .buf = (u8 *) FS_TEST_NON_ASCII, .size = sizeof(FS_TEST_NON_ASCII) - 1, .len = 3 };
    ds_CPool(file) entries;
    ds_CPoolAlloc(NULL, entries, 8, GROWABLE);
    TEST_EQUAL(FS_SUCCESS, DirectoryPushEntries(mem, &entries, &dir));
    TEST_EQUAL(FILE_HANDLE_INVALID, dir.handle);
    u32 found = 0;
    for (u32 i = 0; i < entries.count; ++i)
    {
        const struct file *e = entries.buf + i;
        if (Utf8Equivalence(e->path, Utf8Inline("a.txt")))                  { found |= 1; TEST_EQUAL(FILE_REGULAR, e->type); }
        else if (Utf8Equivalence(e->path, Utf8Inline("sub")))               { found |= 2; TEST_EQUAL(FILE_DIRECTORY, e->type); }
        else if (Utf8Equivalence(e->path, non_ascii_name))                 { found |= 4; TEST_EQUAL(FILE_DIRECTORY, e->type); }
        else if (Utf8Equivalence(e->path, Utf8Inline("dangling")))          { found |= 8; TEST_EQUAL(FILE_UNRECOGNIZED, e->type); }
    }
    ds_CPoolDealloc(entries);
    TEST_EQUAL((have_link) ? 15 : 7, found);

    struct arena tiny = ArenaAlloc(NULL, 4);
    TEST_TRUE(tiny.stack_ptr != NULL);
    const utf8 cwd_small = CwdGet(&tiny);
    ArenaFree(&tiny);
    TEST_EQUAL(0, cwd_small.size);
    const utf8 cwd = CwdGet(mem);
    TEST_TRUE(cwd.size > 0 && !PathIsRelativeA(CstrUtf8(mem, cwd)));

    /* #164: the process cwd inside the non-ASCII directory, CwdGet returns it in UTF-8 */
    WCHAR w_cwd[MAX_PATH];
    TEST_TRUE(GetCurrentDirectoryW(MAX_PATH, w_cwd) != 0);
    TEST_TRUE(SetCurrentDirectoryW(L"ds_filesystem_test\\" FS_TEST_NON_ASCII_W) != 0);
    const utf8 cwd_non_ascii = CwdGet(mem);
    SetCurrentDirectoryW(w_cwd);
    const u32 suffix = sizeof(FS_TEST_NON_ASCII) - 1;
    TEST_TRUE(cwd_non_ascii.size >= suffix
            && memcmp(cwd_non_ascii.buf + cwd_non_ascii.size - suffix, FS_TEST_NON_ASCII, suffix) == 0);

    FilesystemTestCleanup();

	return output;
}

#endif

/*
 * Without a root window, ds_WindowLookup finds nothing and ds_DeallocTaggedWindows frees nothing, neither
 * walking the hierarchy from HI_NULL. The editor's root window (if any) is hidden during the calls.
 */
static struct test_Output WindowNoRootTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const i32 root = g_process_root_window;
    g_process_root_window = HI_NULL;
    const struct slot slot = ds_WindowLookup(0);
    ds_DeallocTaggedWindows();
    const i32 root_after = g_process_root_window;
    g_process_root_window = root;

    TEST_EQUAL(U32_MAX, slot.index);
    TEST_TRUE(slot.address == NULL);
    TEST_EQUAL(HI_NULL, root_after);

	return output;
}

#ifdef DS_PERFORMANCE_MEASUREMENT_API
/*
PerfBegin / PerfEnd on a block of known cost: 1000 steps of x = x * x + c, a dependent chain of one
imul (3 cycles) and one add (1 cycle) per step, so ~4000 core cycles; the minimum over many samples, minus the
pair's overhead, must be 3-6 cycles per step on PerfCycles (an exact, user-only counter), and the wall clock's
ticks per cycle must lie in the core clock's range (TSC / core: 0.3 .. 3). Without PerfCycles only the wall
clock is checked, against the cycle range scaled by that ratio. Numbers printed for the record.
*/
static u64 g_perf_block_sink;

static struct test_Output PerfBlockTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u32 steps = 1000;
	const u32 samples = 4096;
	u64 ticks = U64_MAX;
	u64 cycles = U64_MAX;
	for (u32 i = 0; i < samples; ++i)
	{
		/* input loaded and result stored inside the block: the pair only orders memory operations, so a
		 * register-only chain would be hoisted out of it (it was) */
		g_perf_block_sink = RngU64() | 1;
		const struct PerfStamp begin = PerfBegin();
		u64 x = g_perf_block_sink;
		for (u32 k = 0; k < steps; ++k)
		{
			x = x * x + 0x9E3779B97F4A7C15ull;
		}
		g_perf_block_sink = x;
		const struct PerfStamp d = PerfEnd(begin);
		ticks = (d.ticks < ticks) ? d.ticks : ticks;
		cycles = (d.cycles < cycles) ? d.cycles : cycles;
	}

	fprintf(stdout, "\t%s: empty pair %llu ticks, %llu cycles; %u-step chain min %llu ticks, %llu cycles, before subtracting (PerfCycles %s)\n", __func__,
		(unsigned long long) g_perf_overhead_ticks, (unsigned long long) g_perf_overhead_cycles,
		steps, (unsigned long long) ticks, (unsigned long long) cycles, PerfCyclesThreadAvailable() ? "available" : "unavailable");

	TEST_NOT_ZERO(g_perf_overhead_ticks);
	TEST_NOT_ZERO(ticks > g_perf_overhead_ticks);
	ticks -= g_perf_overhead_ticks;

	if (PerfCyclesThreadAvailable())
	{
		TEST_NOT_ZERO(g_perf_overhead_cycles);
		TEST_NOT_ZERO(cycles > g_perf_overhead_cycles);
		cycles -= g_perf_overhead_cycles;
		TEST_NOT_ZERO(3 * steps <= cycles && cycles <= 6 * steps);
		TEST_NOT_ZERO(3 * ticks >= cycles && ticks <= 3 * cycles);
	}
	else
	{
		TEST_NOT_ZERO(steps <= ticks && ticks <= 18 * steps);
	}

	return output;
}
#endif

static struct test_Output (*base_tests[])(struct test_Environment *) =
{
#ifdef DS_PERFORMANCE_MEASUREMENT_API
    PerfBlockTest,
#endif
    TypesTest,
    FloatTest,
    VectorTest,
    QuaternionTest,
    MatrixTest,
    TransformTest,
    PoseTest,
    CameraTest,
    InputNameTest,
#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__ || __DS_PLATFORM__ == __DS_WIN64__
    FilesystemTest,
#endif
    BitCountTest,
    OverflowTest,
    AtomicSingleThreadTest,
    TicketFactoryTest,
    SemaphoreTest,
    WindowNoRootTest,
};

struct suite_Correctness m_base_suite =
{
	.id = "base",
	.unit_test = base_tests,
	.unit_test_count = sizeof(base_tests) / sizeof(base_tests[0]),
};

struct suite_Correctness *base_correctness_suite = &m_base_suite;

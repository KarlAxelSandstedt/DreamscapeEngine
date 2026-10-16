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

static struct test_Output (*base_tests[])(struct test_Environment *) =
{
    TypesTest,
    FloatTest,
    VectorTest,
    QuaternionTest,
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

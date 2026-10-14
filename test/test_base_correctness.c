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

#include "ds_test.h"
#include "ticket_factory.h"

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
    const u64 from_addr = 42;
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

    TicketFactoryDestroy(&tf);

	return output;
}

static struct test_Output (*base_tests[])(struct test_Environment *) =
{
    BitCountTest,
    OverflowTest,
    AtomicSingleThreadTest,
    TicketFactoryTest,
};

struct suite_Correctness m_base_suite =
{
	.id = "base",
	.unit_test = base_tests,
	.unit_test_count = sizeof(base_tests) / sizeof(base_tests[0]),
};

struct suite_Correctness *base_correctness_suite = &m_base_suite;

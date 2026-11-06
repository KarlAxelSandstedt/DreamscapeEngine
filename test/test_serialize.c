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
#include "ds_compact_size.h"

enum SSType
{
	SS_WRITE8,
	SS_WRITE16_LE,
	SS_WRITE32_LE,
	SS_WRITE64_LE,
	SS_WRITE16_BE,
	SS_WRITE32_BE,
	SS_WRITE64_BE,
	SS_COUNT
};

const u64 ss_type_size[SS_COUNT] = { 1, 2, 4, 8, 2, 4, 8 };

static struct test_Output ss_randomized_aligned(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct SS ss_1 = SSAlloc(NULL, size);
	struct SS ss_2 = ss_1;
	struct SS *ss_in = &ss_1;
	struct SS *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 size_left = ss_in->bit_count / 8;
	while (size_left)
	{
		enum SSType type = (enum SSType) RngU64Range(SS_WRITE8, SS_COUNT-1);
		if (size_left < ss_type_size[type])
		{
			continue;
		}
		size_left -= ss_type_size[type];

		switch (type)
		{
			case SS_WRITE8:
			{
				const b8 expected = { .u = (u8) RngU64Range(0, U8_MAX), };
				SSWrite8(ss_in, expected);
				const b8 actual = SSRead8(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_LE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				SSWrite16Le(ss_in, expected);
				const b16 actual = SSRead16Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_BE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				SSWrite16Be(ss_in, expected);
				const b16 actual = SSRead16Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_LE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				SSWrite32Le(ss_in, expected);
				const b32 actual = SSRead32Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_BE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				SSWrite32Be(ss_in, expected);
				const b32 actual = SSRead32Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_LE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				SSWrite64Le(ss_in, expected);
				const b64 actual = SSRead64Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_BE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				SSWrite64Be(ss_in, expected);
				const b64 actual = SSRead64Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;
		}
	}

	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	SSDealloc(&ss_1);

	return output;
}

static struct test_Output ss_randomized_aligned_array(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct SS ss_1 = SSAlloc(NULL, size);
	struct SS ss_2 = ss_1;
	struct SS *ss_in = &ss_1;
	struct SS *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);
#define MAX_COUNT 8
	b8 arr8_write[MAX_COUNT];
	b8 arr8_read[MAX_COUNT];
	b16 arr16_write[MAX_COUNT];
	b16 arr16_read[MAX_COUNT];
	b32 arr32_write[MAX_COUNT];
	b32 arr32_read[MAX_COUNT];
	b64 arr64_write[MAX_COUNT];
	b64 arr64_read[MAX_COUNT];

	u64 size_left = ss_in->bit_count / 8;
	while (size_left)
	{
		u64 count = RngU64Range(1, MAX_COUNT);
		count = (count > MAX_COUNT) ? MAX_COUNT : count;	/* the bound where GCC can see it (-Wstringop-overflow) */
		enum SSType type = (enum SSType) RngU64Range(SS_WRITE8, SS_COUNT-1);
		if (size_left < count*ss_type_size[type])
		{
			continue;
		}
		size_left -= count*ss_type_size[type];

		switch (type)
		{
			case SS_WRITE8:
			{
				for (u32 i = 0; i < count; ++i)
				{
					arr8_write[i].u = (u8) RngU64Range(0, U8_MAX);
				}
				SSWrite8N(ss_in, arr8_write, count);
				SSRead8N(ss_out, arr8_read, count);
				for (u32 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr8_write[i].u, arr8_read[i].u);
				}
			} break;

			case SS_WRITE16_LE:
			{
				for (u32 i = 0; i < count; ++i)
				{
					arr16_write[i].u = (u16) RngU64Range(0, U16_MAX);
				}
				SSWrite16LeN(ss_in, arr16_write, count);
				SSRead16LeN(ss_out, arr16_read, count);
				for (u32 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr16_write[i].u, arr16_read[i].u);
				}
			} break;

			case SS_WRITE16_BE:
			{
				for (u32 i = 0; i < count; ++i)
				{
					arr16_write[i].u = (u16) RngU64Range(0, U16_MAX);
				}
				SSWrite16BeN(ss_in, arr16_write, count);
				SSRead16BeN(ss_out, arr16_read, count);
				for (u32 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr16_write[i].u, arr16_read[i].u);
				}
			} break;

			case SS_WRITE32_LE:
			{
				for (u32 i = 0; i < count; ++i)
				{
					arr32_write[i].u = (u32) RngU64Range(0, U32_MAX);
				}
				SSWrite32LeN(ss_in, arr32_write, count);
				SSRead32LeN(ss_out, arr32_read, count);
				for (u32 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr32_write[i].u, arr32_read[i].u);
				}
			} break;

			case SS_WRITE32_BE:
			{
				for (u32 i = 0; i < count; ++i)
				{
					arr32_write[i].u = (u32) RngU64Range(0, U32_MAX);
				}
				SSWrite32BeN(ss_in, arr32_write, count);
				SSRead32BeN(ss_out, arr32_read, count);
				for (u32 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr32_write[i].u, arr32_read[i].u);
				}
			} break;

			case SS_WRITE64_LE:
			{
				for (u64 i = 0; i < count; ++i)
				{
					arr64_write[i].u = (u64) RngU64Range(0, U64_MAX);
				}
				SSWrite64LeN(ss_in, arr64_write, count);
				SSRead64LeN(ss_out, arr64_read, count);
				for (u64 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr64_write[i].u, arr64_read[i].u);
				}
			} break;

			case SS_WRITE64_BE:
			{
				for (u64 i = 0; i < count; ++i)
				{
					arr64_write[i].u = (u64) RngU64Range(0, U64_MAX);
				}
				SSWrite64BeN(ss_in, arr64_write, count);
				SSRead64BeN(ss_out, arr64_read, count);
				for (u64 i = 0; i < count; ++i)
				{
					TEST_EQUAL(arr64_write[i].u, arr64_read[i].u);
				}
			} break;
		}
	}

	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_in->bit_index = 0;
	ss_out->bit_index = 0;

	SSDealloc(&ss_1);

	return output;
}

static struct test_Output ss_randomized_partial(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct SS ss_1 = SSAlloc(NULL, size);
	struct SS ss_2 = ss_1;
	struct SS *ss_in = &ss_1;
	struct SS *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		enum SSType type = (enum SSType) RngU64Range(SS_WRITE8, SS_COUNT-1);
		const u64 bit_count = RngU64Range(1, 8*ss_type_size[type]);
		if (bits_left < bit_count)
		{
			continue;
		}
		bits_left -= bit_count;

		/* a 0-bit write or read is nothing: the position stays, the bits around it are untouched */
		if (RngU64Range(0, 3) == 0)
		{
			const u64 index_in = ss_in->bit_index;
			const u64 index_out = ss_out->bit_index;
			SSWriteU64Partial(ss_in, RngU64Range(0, U64_MAX), 0);
			TEST_EQUAL(index_in, ss_in->bit_index);
			TEST_EQUAL(0, SSReadU64Partial(ss_out, 0));
			TEST_EQUAL(index_out, ss_out->bit_index);
		}

		switch (type)
		{
			case SS_WRITE8:
			{
				const u8 expected = (u8) RngU64Range(0, U8_MAX) >> (8-bit_count);
				SSWriteU8Partial(ss_in, expected, bit_count);
				const u8 actual = SSReadU8Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_LE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				SSWriteU16Partial(ss_in, expected, bit_count);
				const u16 actual = SSReadU16Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_BE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				SSWriteU16Partial(ss_in, expected, bit_count);
				const u16 actual = SSReadU16Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_LE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				SSWriteU32Partial(ss_in, expected, bit_count);
				const u32 actual = SSReadU32Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_BE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				SSWriteU32Partial(ss_in, expected, bit_count);
				const u32 actual = SSReadU32Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_LE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				SSWriteU64Partial(ss_in, expected, bit_count);
				const u64 actual = SSReadU64Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_BE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				SSWriteU64Partial(ss_in, expected, bit_count);
				const u64 actual = SSReadU64Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;
		}
	}
	
	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_in->bit_index = 0;
	ss_out->bit_index = 0;

	SSDealloc(&ss_1);

	return output;
}

struct ss_sequence_entry
{
	enum SSType 	type;
	u32		bit_count;
	union
	{
		u64	expected;
		i64	expected_signed;
	};
};

static struct test_Output ss_randomized_sequence_partial(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	u32 seq_len = 0;
	struct ss_sequence_entry *seq = malloc(sizeof(struct ss_sequence_entry) * size * 8);
	struct SS ss_1 = SSAlloc(NULL, size);
	struct SS ss_2 = ss_1;
	struct SS *ss_in = &ss_1;
	struct SS *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = (enum SSType) RngU64Range(SS_WRITE8, SS_COUNT-1);
		seq[seq_len].bit_count = (u32) RngU64Range(1, 8*ss_type_size[seq[seq_len].type]);
		seq[seq_len].expected = RngU64Range(0, U64_MAX) >> (64-seq[seq_len].bit_count);
		if (bits_left < seq[seq_len].bit_count)
		{
			continue;
		}
		bits_left -= seq[seq_len].bit_count;

		switch (seq[seq_len].type)
		{
			case SS_WRITE8:     { SSWriteU8Partial(ss_in,  (u8) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { SSWriteU16Partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { SSWriteU16Partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { SSWriteU32Partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { SSWriteU32Partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { SSWriteU64Partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { SSWriteU64Partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		u64 actual = 0;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = SSReadU8Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = SSReadU16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = SSReadU16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = SSReadU32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = SSReadU32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = SSReadU64Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = SSReadU64Partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected);
	}

	free(seq);
	SSDealloc(&ss_1);

	return output;
}

static struct test_Output ss_randomized_sequence_partial_signed(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	u32 seq_len = 0;
	struct ss_sequence_entry *seq = malloc(sizeof(struct ss_sequence_entry) * size * 8);
	struct SS ss_1 = SSAlloc(NULL, size);
	struct SS ss_2 = ss_1;
	struct SS *ss_in = &ss_1;
	struct SS *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = (enum SSType) RngU64Range(SS_WRITE8, SS_COUNT-1);
		seq[seq_len].bit_count = (u32) RngU64Range(1, 8*ss_type_size[seq[seq_len].type]);
		seq[seq_len].expected = RngU64Range(0, U64_MAX) >> (64-seq[seq_len].bit_count);
		const u64 sign = seq[seq_len].expected >> (seq[seq_len].bit_count-1);
		seq[seq_len].expected |= (U64_MAX << (seq[seq_len].bit_count-1)) * sign;

		if (bits_left < seq[seq_len].bit_count)
		{
			continue;
		}
		bits_left -= seq[seq_len].bit_count;

		switch (seq[seq_len].type)
		{
			case SS_WRITE8:     { SSWriteI8Partial(ss_in,  seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { SSWriteI16Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { SSWriteI16Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { SSWriteI32Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { SSWriteI32Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { SSWriteI64Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { SSWriteI64Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		i64 actual = 0;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = SSReadI8Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = SSReadI16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = SSReadI16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = SSReadI32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = SSReadI32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = SSReadI64Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = SSReadI64Partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected_signed);
	}

	free(seq);
	SSDealloc(&ss_1);

	return output;
}

/*
Partial write/read performance: n values of 1-32 bits, widths and values pregenerated in init (the RNG stays
out of the region); the read's sum is stored inside the region. Random widths defeat the branch predictor,
the schema repeats 16 widths as a fixed message layout would. Each init also writes the stream once, so the
read tests have one to read.
*/
struct ss_partial_input
{
	struct SS	ss_w;
	struct SS	ss_r;	/* the same buffer, its own position */
	u64		n;
	u8 *		bits;
	u32 *		values;
	u64		sum;
};

#define   ss_partial_size    ((u64) 4*256*1024)

static void ss_partial_write(void *args)
{
	struct ss_partial_input *in = args;
	struct SS *ss = &in->ss_w;
	for (u64 i = 0; i < in->n; ++i)
	{
		SSWriteU32Partial(ss, in->values[i], in->bits[i]);
	}
}

static void ss_partial_read(void *args)
{
	struct ss_partial_input *in = args;
	struct SS *ss = &in->ss_r;
	u64 sum = 0;
	for (u64 i = 0; i < in->n; ++i)
	{
		sum += SSReadU32Partial(ss, in->bits[i]);
	}
	in->sum = sum;
}

static void ss_partial_write_read(void *args)
{
	struct ss_partial_input *in = args;
	u64 sum = 0;
	for (u64 i = 0; i < in->n; ++i)
	{
		SSWriteU32Partial(&in->ss_w, in->values[i], in->bits[i]);
		sum += SSReadU32Partial(&in->ss_r, in->bits[i]);
	}
	in->sum = sum;
}

static void *ss_partial_init(const u32 schema)
{
	static const u8 widths[16] = { 1, 32, 5, 12, 8, 3, 16, 24, 7, 1, 9, 32, 4, 20, 11, 2 };
	struct ss_partial_input *in = malloc(sizeof(struct ss_partial_input));
	in->n = ss_partial_size / 4;
	in->ss_w = SSAlloc(NULL, ss_partial_size);
	in->ss_r = in->ss_w;
	in->bits = malloc(in->n);
	in->values = malloc(in->n * sizeof(u32));
	for (u64 i = 0; i < in->n; ++i)
	{
		in->bits[i] = schema ? widths[i & 15] : (u8) RngU64Range(1, 32);
		in->values[i] = (u32) (RngU64Range(0, U32_MAX) >> (32 - in->bits[i]));
	}
	in->sum = 0;
	ss_partial_write(in);
	return in;
}

static void *ss_partial_init_random(void)	{ return ss_partial_init(0); }
static void *ss_partial_init_schema(void)	{ return ss_partial_init(1); }

static void ss_partial_reset(void *args)
{
	struct ss_partial_input *in = args;
	in->ss_w.bit_index = 0;
	in->ss_r.bit_index = 0;
}

static void ss_partial_free(void *args)
{
	struct ss_partial_input *in = args;
	free(in->values);
	free(in->bits);
	SSDealloc(&in->ss_w);
	free(in);
}


/*
SSReadU64Partial against a bit-by-bit reference of the format (stream bit i = bit i % 8 of byte i / 8, least
significant first): random widths over a random buffer down to its last bit, aligned widths against the Le byte
reads, and one hand-computed pair.
*/
static struct test_Output ss_read_partial_reference(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	{
		u8 two[2] = { 0xB5, 0x63 };	/* 1011 0101, 0110 0011 */
		struct SS ss = SSBuffered(two, 2);
		TEST_EQUAL(5u, SSReadU64Partial(&ss, 3));	/* bits 0-2 of 0xB5: 101 */
		TEST_EQUAL(118u, SSReadU64Partial(&ss, 7));	/* bits 3-7 of 0xB5 (10110) then bits 0-1 of 0x63 (11) */
		TEST_EQUAL(24u, SSReadU64Partial(&ss, 6));	/* bits 2-7 of 0x63: 0,0,0,1,1,0 = 8 + 16 */
	}

	const u64 size = 4096;
	u8 *buf = malloc(size);
	for (u64 i = 0; i < size; ++i)
	{
		buf[i] = (u8) RngU64Range(0, U8_MAX);
	}

	struct SS ss = SSBuffered(buf, size);
	struct SS aligned = ss;
	while (ss.bit_index < ss.bit_count)
	{
		const u64 bits_left = ss.bit_count - ss.bit_index;
		const u64 bit_count = (bits_left < 64) ? RngU64Range(1, bits_left) : RngU64Range(1, 64);
		const u64 pos = ss.bit_index;

		u64 expected = 0;
		for (u64 k = 0; k < bit_count; ++k)
		{
			const u64 bit = (buf[(pos + k) >> 3] >> ((pos + k) & 7)) & 1;
			expected |= bit << k;
		}
		const u64 actual = SSReadU64Partial(&ss, bit_count);
		TEST_EQUAL(expected, actual);
		TEST_EQUAL(pos + bit_count, ss.bit_index);

		/* at a byte boundary, 8k bits are the Le byte functions' bytes */
		if ((pos & 7) == 0 && (bit_count == 8 || bit_count == 16 || bit_count == 32 || bit_count == 64))
		{
			aligned.bit_index = pos;
			const u64 le = (bit_count == 8) ? SSReadU8(&aligned)
				     : (bit_count == 16) ? SSReadU16Le(&aligned)
				     : (bit_count == 32) ? SSReadU32Le(&aligned)
				     : SSReadU64Le(&aligned);
			TEST_EQUAL(le, actual);
		}
	}

	free(buf);
	return output;
}


/*
SSWriteU64Partial against a bit-by-bit reference writer: random widths and values to the last bit of a buffer,
byte writes mixed in at byte boundaries, the buffers compared, and the stream read back with SSReadU64Partial.
*/
static struct test_Output ss_write_partial_reference(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 4096;
	const u64 n_max = 2 * size;
	u8 *buf = malloc(size);
	u8 *reference = malloc(size);
	u8 *bits = malloc(n_max);
	u64 *values = malloc(n_max * sizeof(u64));
	memset(buf, 0xA5, size);	/* the writer must not depend on the buffer's old contents */
	memset(reference, 0, size);

	struct SS ss = SSBuffered(buf, size);
	u64 n = 0;
	while (ss.bit_index < ss.bit_count)
	{
		const u64 pos = ss.bit_index;
		const u64 bits_left = ss.bit_count - pos;
		u64 bit_count = (bits_left < 64) ? RngU64Range(1, bits_left) : RngU64Range(1, 64);
		const u64 val = RngU64();
		const u64 masked = val & (U64_MAX >> (64 - bit_count));

		/* every fourth aligned value of a byte size goes through the Le byte functions */
		if ((pos & 7) == 0 && (bit_count & 7) == 0 && bit_count <= 32 && bits_left >= 32 && (n & 3) == 0)
		{
			bit_count = 32;
			SSWriteU32Le(&ss, (u32) val);
		}
		else
		{
			SSWriteU64Partial(&ss, val, bit_count);
		}
		TEST_EQUAL(pos + bit_count, ss.bit_index);

		const u64 stored = val & (U64_MAX >> (64 - bit_count));
		for (u64 k = 0; k < bit_count; ++k)
		{
			reference[(pos + k) >> 3] |= (u8) (((stored >> k) & 1) << ((pos + k) & 7));
		}
		bits[n] = (u8) bit_count;
		values[n] = stored;
		n += 1;
	}
	TEST_EQUAL(0, memcmp(buf, reference, size));

	struct SS in = SSBuffered(buf, size);
	for (u64 i = 0; i < n; ++i)
	{
		TEST_EQUAL(values[i], SSReadU64Partial(&in, bits[i]));
	}

	free(values);
	free(bits);
	free(reference);
	free(buf);
	return output;
}

/* a view streams the parent's buffer at its own bit range and refuses a range past the end */
static struct test_Output ss_view(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	u8 buf[4] = { 0xB5, 0x63, 0x0F, 0xA0 };	/* 1011 0101, 0110 0011, 0000 1111, 1010 0000 */
	struct SS ss = SSBuffered(buf, 4);
	struct SSView view;

	TEST_EQUAL(1, SSTryView(&view, &ss, 3, 7));		/* bits 3-9: 10110 then 11 */
	TEST_TRUE(view.buf == buf && view.bit_offset == 3 && view.bit_count == 7);
	struct SS sub = SSSubstreamFromView(&view);
	TEST_TRUE(sub.buf == buf && sub.bit_index == 3 && sub.bit_count == 10 && SSBitsLeft(&sub) == 7);
	TEST_EQUAL(118u, SSReadU64Partial(&sub, 7));
	TEST_EQUAL(0u, SSBitsLeft(&sub));

	TEST_EQUAL(1, SSTryView(&view, &ss, 8, 24));		/* byte aligned: byte access works */
	sub = SSSubstreamFromView(&view);
	TEST_TRUE(sub.buf == buf && sub.bit_index == 8 && SSBytesLeft(&sub) == 3);
	TEST_EQUAL(0x63u, SSReadU8(&sub));

	TEST_EQUAL(0, SSTryView(&view, &ss, 30, 3));		/* past the end */
	TEST_TRUE(view.buf == NULL && view.bit_count == 0);
	TEST_EQUAL(0, SSTryView(&view, &ss, 33, 0));
	TEST_EQUAL(1, SSTryView(&view, &ss, 32, 0));		/* empty at the end */
	sub = SSSubstreamFromView(&view);
	TEST_EQUAL(0u, SSBitsLeft(&sub));

	return output;
}

/* SSIncreaseSize: grows to max(twice the size, the request) keeping bytes and position; a request that fits is a no-op */
static struct test_Output ss_increase_size(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	struct SS ss = SSAlloc(NULL, 8);
	for (u32 i = 0; i < 4; ++i)
	{
		SSWriteU16Le(&ss, (u16) (0x1000 + i));
	}

	SSIncreaseSize(&ss, 9);			/* doubles: 16 */
	TEST_TRUE(ss.heap_allocated && ss.bit_count == 128 && ss.bit_index == 64);
	SSIncreaseSize(&ss, 100);		/* more than double: 100 */
	TEST_TRUE(ss.bit_count == 800 && ss.bit_index == 64);
	SSIncreaseSize(&ss, 50);		/* fits: no-op */
	TEST_TRUE(ss.bit_count == 800 && ss.bit_index == 64);

	SSWriteU16Le(&ss, 0x2000);
	ss.bit_index = 0;
	for (u32 i = 0; i < 4; ++i)
	{
		TEST_EQUAL(0x1000 + i, SSReadU16Le(&ss));
	}
	TEST_EQUAL(0x2000u, SSReadU16Le(&ss));

	SSDealloc(&ss);
	return output;
}

/*
 * CompactSize: every size boundary round trips at its size (252/253, 65535/65536, 2^32 - 1 / 2^32, U64_MAX), reading
 * advances by exactly the size; a truncated stream and each longer-than-shortest encoding are refused with the
 * stream position as before.
 */
static struct test_Output ss_compact_size(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 values[] = { 0, 1, 252, 253, 254, 255, 256, U16_MAX, (u64) U16_MAX + 1, U32_MAX, (u64) U32_MAX + 1, U64_MAX };
	const u64 sizes[]  = { 1, 1,   1,   3,   3,   3,   3,       3,                 5,       5,                 9,       9 };
	u8 buf[32];
	for (u32 i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
	{
		struct SS ss = SSBuffered(buf, 32);
		TEST_EQUAL(sizes[i], ds_CompactSizeSerializeSize(values[i]));
		ds_CompactSizeSerialize(&ss, values[i]);
		TEST_EQUAL(8*sizes[i], ss.bit_index);

		ss.bit_index = 0;
		u64 read = 0;
		TEST_EQUAL(1, ds_CompactSizeTryDeserialize(&ss, &read));
		TEST_EQUAL(values[i], read);
		TEST_EQUAL(8*sizes[i], ss.bit_index);

		/* truncated: one byte short */
		struct SS cut = SSBuffered(buf, sizes[i] - 1);
		TEST_EQUAL(0, ds_CompactSizeTryDeserialize(&cut, &read));
		TEST_EQUAL(0, cut.bit_index);
	}

	/* not the shortest encoding: 252 as u16, 65535 as u32, 2^32 - 1 as u64 */
	struct SS ss = SSBuffered(buf, 32);
	SSWriteU8(&ss, 0xFD); SSWriteU16Le(&ss, 252);
	SSWriteU8(&ss, 0xFE); SSWriteU32Le(&ss, U16_MAX);
	SSWriteU8(&ss, 0xFF); SSWriteU64Le(&ss, U32_MAX);
	ss.bit_index = 0;
	u64 read = 0;
	TEST_EQUAL(0, ds_CompactSizeTryDeserialize(&ss, &read));
	TEST_EQUAL(0, ss.bit_index);
	ss.bit_index = 8*3;
	TEST_EQUAL(0, ds_CompactSizeTryDeserialize(&ss, &read));
	TEST_EQUAL(8*3, ss.bit_index);
	ss.bit_index = 8*8;
	TEST_EQUAL(0, ds_CompactSizeTryDeserialize(&ss, &read));
	TEST_EQUAL(8*8, ss.bit_index);

	return output;
}

/*
 * CompactSize fuzz (fixed seed): random values encoded then mutated (bit flips, truncation, a random marker byte,
 * random bytes); a read that succeeds consumed exactly the value's size and re-encodes to the same bytes, a read
 * that fails leaves the position as it was. Never an assert.
 */
static struct test_Output ss_compact_size_fuzz(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	struct rng rng;
	RngLocalInit(&rng, 0x5ea1ab1e);
	for (u32 n = 0; n < 20000; ++n)
	{
		u8 buf[16] = { 0 };
		struct SS ss = SSBuffered(buf, 16);
		const u32 shift = (u32) RngLocalU64Range(&rng, 0, 63);
		const u64 value = RngLocalU64(&rng) >> shift;
		ds_CompactSizeSerialize(&ss, value);
		u64 len = ss.bit_index / 8;

		switch (RngLocalU64Range(&rng, 0, 3))
		{
			case 0: { buf[RngLocalU64Range(&rng, 0, len - 1)] ^= (u8) (1u << RngLocalU64Range(&rng, 0, 7)); } break;
			case 1: { len = RngLocalU64Range(&rng, 0, len); } break;
			case 2: { buf[0] = (u8) RngLocalU64Range(&rng, 0, 255); } break;
			default: { for (u32 i = 0; i < 16; ++i) { buf[i] = (u8) RngLocalU64Range(&rng, 0, 255); } len = RngLocalU64Range(&rng, 0, 16); } break;
		}

		struct SS read = SSBuffered(buf, len);
		read.bit_index = 0;
		u64 decoded = 0;
		if (ds_CompactSizeTryDeserialize(&read, &decoded))
		{
			const u64 size = ds_CompactSizeSerializeSize(decoded);
			TEST_EQUAL(8*size, read.bit_index);
			u8 again[16] = { 0 };
			struct SS re = SSBuffered(again, 16);
			ds_CompactSizeSerialize(&re, decoded);
			for (u32 i = 0; i < size; ++i)
			{
				TEST_EQUAL(buf[i], again[i]);
			}
		}
		else
		{
			TEST_EQUAL(0, read.bit_index);
		}
	}

	return output;
}

struct test_CorrectnessRepetition repetition_test[] =
{
	{ .test =  &ss_view, .count = 1, },
	{ .test =  &ss_increase_size, .count = 1, },
	{ .test =  &ss_compact_size, .count = 1, },
	{ .test =  &ss_compact_size_fuzz, .count = 1, },
	{ .test =  &ss_read_partial_reference, .count = 20, },
	{ .test =  &ss_write_partial_reference, .count = 20, },
	{ .test =  &ss_randomized_aligned, .count = 100, },
	{ .test =  &ss_randomized_aligned_array, .count = 100, },
	{ .test =  &ss_randomized_partial, .count = 100, },
	{ .test =  &ss_randomized_sequence_partial, .count = 100, },
	{ .test =  &ss_randomized_sequence_partial_signed, .count = 100, },
};

struct suite_Correctness m_serialize_suite =
{
	.id = "Serialize",
	.repetition_test = repetition_test,
	.repetition_test_count = sizeof(repetition_test) / sizeof(repetition_test[0]),
};

struct suite_Correctness *serialize_correctness_suite = &m_serialize_suite;

struct test_PerformanceSerial serialize_serial_test[] =
{
	{ .id = "ss_write_u32_partial_random", .size = ss_partial_size, .test = &ss_partial_write, .test_init = &ss_partial_init_random, .test_reset = &ss_partial_reset, .test_free = &ss_partial_free },
	{ .id = "ss_read_u32_partial_random", .size = ss_partial_size, .test = &ss_partial_read, .test_init = &ss_partial_init_random, .test_reset = &ss_partial_reset, .test_free = &ss_partial_free },
	{ .id = "ss_write_read_u32_partial", .size = ss_partial_size, .test = &ss_partial_write_read, .test_init = &ss_partial_init_random, .test_reset = &ss_partial_reset, .test_free = &ss_partial_free },
	{ .id = "ss_write_u32_partial_schema", .size = ss_partial_size, .test = &ss_partial_write, .test_init = &ss_partial_init_schema, .test_reset = &ss_partial_reset, .test_free = &ss_partial_free },
	{ .id = "ss_read_u32_partial_schema", .size = ss_partial_size, .test = &ss_partial_read, .test_init = &ss_partial_init_schema, .test_reset = &ss_partial_reset, .test_free = &ss_partial_free },
};

struct suite_Performance storage_performance_serialize_suite =
{
	.id = "Serialize Performance",
	.serial_test = serialize_serial_test,
	.serial_test_count = sizeof(serialize_serial_test) / sizeof(serialize_serial_test[0]),
};

struct suite_Performance *serialize_performance_suite = &storage_performance_serialize_suite;


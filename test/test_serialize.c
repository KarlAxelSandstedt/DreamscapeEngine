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

enum ss_Type
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
	struct ss ss_1 = ss_Alloc(NULL, size);
	struct ss ss_2 = ss_1;
	struct ss *ss_in = &ss_1;
	struct ss *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 size_left = ss_in->bit_count / 8;
	while (size_left)
	{
		enum ss_Type type = (enum ss_Type) RngU64Range(SS_WRITE8, SS_COUNT-1);
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
				ss_Write8(ss_in, expected);
				const b8 actual = ss_Read8(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_LE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				ss_Write16Le(ss_in, expected);
				const b16 actual = ss_Read16Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_BE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				ss_Write16Be(ss_in, expected);
				const b16 actual = ss_Read16Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_LE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				ss_Write32Le(ss_in, expected);
				const b32 actual = ss_Read32Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_BE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				ss_Write32Be(ss_in, expected);
				const b32 actual = ss_Read32Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_LE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				ss_Write64Le(ss_in, expected);
				const b64 actual = ss_Read64Le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_BE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				ss_Write64Be(ss_in, expected);
				const b64 actual = ss_Read64Be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;
		}
	}

	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_Free(&ss_1);

	return output;
}

static struct test_Output ss_randomized_aligned_array(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct ss ss_1 = ss_Alloc(NULL, size);
	struct ss ss_2 = ss_1;
	struct ss *ss_in = &ss_1;
	struct ss *ss_out = &ss_2;
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
		enum ss_Type type = (enum ss_Type) RngU64Range(SS_WRITE8, SS_COUNT-1);
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
				ss_Write8N(ss_in, arr8_write, count);
				ss_Read8N(ss_out, arr8_read, count);
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
				ss_Write16LeN(ss_in, arr16_write, count);
				ss_Read16LeN(ss_out, arr16_read, count);
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
				ss_Write16BeN(ss_in, arr16_write, count);
				ss_Read16BeN(ss_out, arr16_read, count);
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
				ss_Write32LeN(ss_in, arr32_write, count);
				ss_Read32LeN(ss_out, arr32_read, count);
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
				ss_Write32BeN(ss_in, arr32_write, count);
				ss_Read32BeN(ss_out, arr32_read, count);
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
				ss_Write64LeN(ss_in, arr64_write, count);
				ss_Read64LeN(ss_out, arr64_read, count);
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
				ss_Write64BeN(ss_in, arr64_write, count);
				ss_Read64BeN(ss_out, arr64_read, count);
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

	ss_Free(&ss_1);

	return output;
}

static struct test_Output ss_randomized_partial(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct ss ss_1 = ss_Alloc(NULL, size);
	struct ss ss_2 = ss_1;
	struct ss *ss_in = &ss_1;
	struct ss *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		enum ss_Type type = (enum ss_Type) RngU64Range(SS_WRITE8, SS_COUNT-1);
		const u64 bit_count = RngU64Range(1, 8*ss_type_size[type]);
		if (bits_left < bit_count)
		{
			continue;
		}
		bits_left -= bit_count;

		switch (type)
		{
			case SS_WRITE8:
			{
				const u8 expected = (u8) RngU64Range(0, U8_MAX) >> (8-bit_count);
				ss_WriteU8Partial(ss_in, expected, bit_count);
				const u8 actual = ss_ReadU8Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_LE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				ss_WriteU16Partial(ss_in, expected, bit_count);
				const u16 actual = ss_ReadU16Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_BE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				ss_WriteU16Partial(ss_in, expected, bit_count);
				const u16 actual = ss_ReadU16Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_LE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				ss_WriteU32Partial(ss_in, expected, bit_count);
				const u32 actual = ss_ReadU32Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_BE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				ss_WriteU32Partial(ss_in, expected, bit_count);
				const u32 actual = ss_ReadU32Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_LE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				ss_WriteU64Partial(ss_in, expected, bit_count);
				const u64 actual = ss_ReadU64Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_BE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				ss_WriteU64Partial(ss_in, expected, bit_count);
				const u64 actual = ss_ReadU64Partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;
		}
	}
	
	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_in->bit_index = 0;
	ss_out->bit_index = 0;

	ss_Free(&ss_1);

	return output;
}

struct ss_sequence_entry
{
	enum ss_Type 	type;
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
	struct ss ss_1 = ss_Alloc(NULL, size);
	struct ss ss_2 = ss_1;
	struct ss *ss_in = &ss_1;
	struct ss *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = (enum ss_Type) RngU64Range(SS_WRITE8, SS_COUNT-1);
		seq[seq_len].bit_count = (u32) RngU64Range(1, 8*ss_type_size[seq[seq_len].type]);
		seq[seq_len].expected = RngU64Range(0, U64_MAX) >> (64-seq[seq_len].bit_count);
		if (bits_left < seq[seq_len].bit_count)
		{
			continue;
		}
		bits_left -= seq[seq_len].bit_count;

		switch (seq[seq_len].type)
		{
			case SS_WRITE8:     { ss_WriteU8Partial(ss_in,  (u8) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { ss_WriteU16Partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { ss_WriteU16Partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { ss_WriteU32Partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { ss_WriteU32Partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { ss_WriteU64Partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { ss_WriteU64Partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		u64 actual = 0;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = ss_ReadU8Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = ss_ReadU16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = ss_ReadU16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = ss_ReadU32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = ss_ReadU32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = ss_ReadU64Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = ss_ReadU64Partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected);
	}

	free(seq);
	ss_Free(&ss_1);

	return output;
}

static struct test_Output ss_randomized_sequence_partial_signed(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	u32 seq_len = 0;
	struct ss_sequence_entry *seq = malloc(sizeof(struct ss_sequence_entry) * size * 8);
	struct ss ss_1 = ss_Alloc(NULL, size);
	struct ss ss_2 = ss_1;
	struct ss *ss_in = &ss_1;
	struct ss *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = (enum ss_Type) RngU64Range(SS_WRITE8, SS_COUNT-1);
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
			case SS_WRITE8:     { ss_WriteI8Partial(ss_in,  seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { ss_WriteI16Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { ss_WriteI16Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { ss_WriteI32Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { ss_WriteI32Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { ss_WriteI64Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { ss_WriteI64Partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		i64 actual = 0;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = ss_ReadI8Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = ss_ReadI16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = ss_ReadI16Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = ss_ReadI32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = ss_ReadI32Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = ss_ReadI64Partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = ss_ReadI64Partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected_signed);
	}

	free(seq);
	ss_Free(&ss_1);

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
	struct ss	ss_w;
	struct ss	ss_r;	/* the same buffer, its own position */
	u64		n;
	u8 *		bits;
	u32 *		values;
	u64		sum;
};

#define   ss_partial_size    ((u64) 4*256*1024)

static void ss_partial_write(void *args)
{
	struct ss_partial_input *in = args;
	struct ss *ss = &in->ss_w;
	for (u64 i = 0; i < in->n; ++i)
	{
		ss_WriteU32Partial(ss, in->values[i], in->bits[i]);
	}
}

static void ss_partial_read(void *args)
{
	struct ss_partial_input *in = args;
	struct ss *ss = &in->ss_r;
	u64 sum = 0;
	for (u64 i = 0; i < in->n; ++i)
	{
		sum += ss_ReadU32Partial(ss, in->bits[i]);
	}
	in->sum = sum;
}

static void ss_partial_write_read(void *args)
{
	struct ss_partial_input *in = args;
	u64 sum = 0;
	for (u64 i = 0; i < in->n; ++i)
	{
		ss_WriteU32Partial(&in->ss_w, in->values[i], in->bits[i]);
		sum += ss_ReadU32Partial(&in->ss_r, in->bits[i]);
	}
	in->sum = sum;
}

static void *ss_partial_init(const u32 schema)
{
	static const u8 widths[16] = { 1, 32, 5, 12, 8, 3, 16, 24, 7, 1, 9, 32, 4, 20, 11, 2 };
	struct ss_partial_input *in = malloc(sizeof(struct ss_partial_input));
	in->n = ss_partial_size / 4;
	in->ss_w = ss_Alloc(NULL, ss_partial_size);
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
	ss_Free(&in->ss_w);
	free(in);
}


/*
ss_ReadU64Partial against a bit-by-bit reference of the format (stream bit i = bit i % 8 of byte i / 8, least
significant first): random widths over a random buffer down to its last bit, aligned widths against the Le byte
reads, and one hand-computed pair.
*/
static struct test_Output ss_read_partial_reference(void)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	{
		u8 two[2] = { 0xB5, 0x63 };	/* 1011 0101, 0110 0011 */
		struct ss ss = ss_Buffered(two, 2);
		TEST_EQUAL(5u, ss_ReadU64Partial(&ss, 3));	/* bits 0-2 of 0xB5: 101 */
		TEST_EQUAL(118u, ss_ReadU64Partial(&ss, 7));	/* bits 3-7 of 0xB5 (10110) then bits 0-1 of 0x63 (11) */
		TEST_EQUAL(24u, ss_ReadU64Partial(&ss, 6));	/* bits 2-7 of 0x63: 0,0,0,1,1,0 = 8 + 16 */
	}

	const u64 size = 4096;
	u8 *buf = malloc(size);
	for (u64 i = 0; i < size; ++i)
	{
		buf[i] = (u8) RngU64Range(0, U8_MAX);
	}

	struct ss ss = ss_Buffered(buf, size);
	struct ss aligned = ss;
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
		const u64 actual = ss_ReadU64Partial(&ss, bit_count);
		TEST_EQUAL(expected, actual);
		TEST_EQUAL(pos + bit_count, ss.bit_index);

		/* at a byte boundary, 8k bits are the Le byte functions' bytes */
		if ((pos & 7) == 0 && (bit_count == 8 || bit_count == 16 || bit_count == 32 || bit_count == 64))
		{
			aligned.bit_index = pos;
			const u64 le = (bit_count == 8) ? ss_ReadU8(&aligned)
				     : (bit_count == 16) ? ss_ReadU16Le(&aligned)
				     : (bit_count == 32) ? ss_ReadU32Le(&aligned)
				     : ss_ReadU64Le(&aligned);
			TEST_EQUAL(le, actual);
		}
	}

	free(buf);
	return output;
}


/*
ss_WriteU64Partial against a bit-by-bit reference writer: random widths and values to the last bit of a buffer,
byte writes mixed in at byte boundaries, the buffers compared, and the stream read back with ss_ReadU64Partial.
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

	struct ss ss = ss_Buffered(buf, size);
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
			ss_WriteU32Le(&ss, (u32) val);
		}
		else
		{
			ss_WriteU64Partial(&ss, val, bit_count);
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

	struct ss in = ss_Buffered(buf, size);
	for (u64 i = 0; i < n; ++i)
	{
		TEST_EQUAL(values[i], ss_ReadU64Partial(&in, bits[i]));
	}

	free(values);
	free(bits);
	free(reference);
	free(buf);
	return output;
}

struct test_CorrectnessRepetition repetition_test[] =
{
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


/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#include <string.h>

#include "test_local.h"
#include "serialize.h"

enum ss_type
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

static struct test_output ss_randomized_aligned(void)
{
	struct test_output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct serialize_stream ss_1 = ss_alloc(NULL, size);
	struct serialize_stream ss_2 = ss_1;
	struct serialize_stream *ss_in = &ss_1;
	struct serialize_stream *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 size_left = ss_in->bit_count / 8;
	while (size_left)
	{
		enum ss_type type = RngU64Range(SS_WRITE8, SS_COUNT-1);
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
				ss_write8(ss_in, expected);
				const b8 actual = ss_read8(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_LE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				ss_write16_le(ss_in, expected);
				const b16 actual = ss_read16_le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE16_BE:
			{
				const b16 expected = { .u = (u16) RngU64Range(0, U16_MAX), };
				ss_write16_be(ss_in, expected);
				const b16 actual = ss_read16_be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_LE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				ss_write32_le(ss_in, expected);
				const b32 actual = ss_read32_le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE32_BE:
			{
				const b32 expected = { .u = (u32) RngU64Range(0, U32_MAX), };
				ss_write32_be(ss_in, expected);
				const b32 actual = ss_read32_be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_LE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				ss_write64_le(ss_in, expected);
				const b64 actual = ss_read64_le(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;

			case SS_WRITE64_BE:
			{
				const b64 expected = { .u = (u64) RngU64Range(0, U64_MAX), };
				ss_write64_be(ss_in, expected);
				const b64 actual = ss_read64_be(ss_out);
				TEST_EQUAL(expected.u, actual.u);
			} break;
		}
	}

	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_free(&ss_1);

	return output;
}

static struct test_output ss_randomized_aligned_array(void)
{
	struct test_output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct serialize_stream ss_1 = ss_alloc(NULL, size);
	struct serialize_stream ss_2 = ss_1;
	struct serialize_stream *ss_in = &ss_1;
	struct serialize_stream *ss_out = &ss_2;
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
		enum ss_type type = RngU64Range(SS_WRITE8, SS_COUNT-1);
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
				ss_write8_array(ss_in, arr8_write, count);
				ss_read8_array(arr8_read, ss_out, count);
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
				ss_write16_le_array(ss_in, arr16_write, count);
				ss_read16_le_array(arr16_read, ss_out, count);
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
				ss_write16_be_array(ss_in, arr16_write, count);
				ss_read16_be_array(arr16_read, ss_out, count);
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
				ss_write32_le_array(ss_in, arr32_write, count);
				ss_read32_le_array(arr32_read, ss_out, count);
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
				ss_write32_be_array(ss_in, arr32_write, count);
				ss_read32_be_array(arr32_read, ss_out, count);
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
				ss_write64_le_array(ss_in, arr64_write, count);
				ss_read64_le_array(arr64_read, ss_out, count);
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
				ss_write64_be_array(ss_in, arr64_write, count);
				ss_read64_be_array(arr64_read, ss_out, count);
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

	ss_free(&ss_1);

	return output;
}

static struct test_output ss_randomized_partial(void)
{
	struct test_output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	struct serialize_stream ss_1 = ss_alloc(NULL, size);
	struct serialize_stream ss_2 = ss_1;
	struct serialize_stream *ss_in = &ss_1;
	struct serialize_stream *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		enum ss_type type = RngU64Range(SS_WRITE8, SS_COUNT-1);
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
				ss_write_u8_partial(ss_in, expected, bit_count);
				const u8 actual = ss_read_u8_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_LE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				ss_write_u16_le_partial(ss_in, expected, bit_count);
				const u16 actual = ss_read_u16_le_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE16_BE:
			{
				const u16 expected = (u16) RngU64Range(0, U16_MAX) >> (16-bit_count);
				ss_write_u16_be_partial(ss_in, expected, bit_count);
				const u16 actual = ss_read_u16_be_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_LE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				ss_write_u32_le_partial(ss_in, expected, bit_count);
				const u32 actual = ss_read_u32_le_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE32_BE:
			{
				const u32 expected = (u32) RngU64Range(0, U32_MAX) >> (32-bit_count);
				ss_write_u32_be_partial(ss_in, expected, bit_count);
				const u32 actual = ss_read_u32_be_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_LE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				ss_write_u64_le_partial(ss_in, expected, bit_count);
				const u64 actual = ss_read_u64_le_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;

			case SS_WRITE64_BE:
			{
				const u64 expected = (u64) RngU64Range(0, U64_MAX) >> (64-bit_count);
				ss_write_u64_be_partial(ss_in, expected, bit_count);
				const u64 actual = ss_read_u64_be_partial(ss_out, bit_count);
				TEST_EQUAL(expected, actual);
			} break;
		}
	}
	
	TEST_EQUAL(ss_in->bit_index, ss_in->bit_count);
	TEST_EQUAL(ss_out->bit_index, ss_out->bit_count);

	ss_in->bit_index = 0;
	ss_out->bit_index = 0;

	ss_free(&ss_1);

	return output;
}

struct ss_sequence_entry
{
	enum ss_type 	type;
	u32		bit_count;
	union
	{
		u64	expected;
		i64	expected_signed;
	};
};

static struct test_output ss_randomized_sequence_partial(void)
{
	struct test_output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	u32 seq_len = 0;
	struct ss_sequence_entry *seq = malloc(sizeof(struct ss_sequence_entry) * size * 8);
	struct serialize_stream ss_1 = ss_alloc(NULL, size);
	struct serialize_stream ss_2 = ss_1;
	struct serialize_stream *ss_in = &ss_1;
	struct serialize_stream *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = RngU64Range(SS_WRITE8, SS_COUNT-1);
		seq[seq_len].bit_count = RngU64Range(1, 8*ss_type_size[seq[seq_len].type]);
		seq[seq_len].expected = RngU64Range(0, U64_MAX) >> (64-seq[seq_len].bit_count);
		if (bits_left < seq[seq_len].bit_count)
		{
			continue;
		}
		bits_left -= seq[seq_len].bit_count;

		switch (seq[seq_len].type)
		{
			case SS_WRITE8: { ss_write_u8_partial(ss_in,  (u8) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { ss_write_u16_le_partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { ss_write_u16_be_partial(ss_in, (u16) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { ss_write_u32_le_partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { ss_write_u32_be_partial(ss_in, (u32) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { ss_write_u64_le_partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { ss_write_u64_be_partial(ss_in, (u64) seq[seq_len].expected, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		u64 actual;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = ss_read_u8_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = ss_read_u16_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = ss_read_u16_be_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = ss_read_u32_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = ss_read_u32_be_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = ss_read_u64_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = ss_read_u64_be_partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected);
	}

	free(seq);
	ss_free(&ss_1);

	return output;
}

static struct test_output ss_randomized_sequence_partial_signed(void)
{
	struct test_output output = { .success = 1, .id = __func__ };

	const u64 size = 1*1024*1024;
	u32 seq_len = 0;
	struct ss_sequence_entry *seq = malloc(sizeof(struct ss_sequence_entry) * size * 8);
	struct serialize_stream ss_1 = ss_alloc(NULL, size);
	struct serialize_stream ss_2 = ss_1;
	struct serialize_stream *ss_in = &ss_1;
	struct serialize_stream *ss_out = &ss_2;
	memset(ss_1.buf, 0, size);

	u64 bits_left = ss_in->bit_count;
	while (bits_left)
	{
		seq[seq_len].type = RngU64Range(SS_WRITE8, SS_COUNT-1);
		seq[seq_len].bit_count = RngU64Range(1, 8*ss_type_size[seq[seq_len].type]);
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
			case SS_WRITE8: { ss_write_i8_partial(ss_in,  seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_LE: { ss_write_i16_le_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE16_BE: { ss_write_i16_be_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_LE: { ss_write_i32_le_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE32_BE: { ss_write_i32_be_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_LE: { ss_write_i64_le_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
			case SS_WRITE64_BE: { ss_write_i64_be_partial(ss_in, seq[seq_len].expected_signed, seq[seq_len].bit_count); } break;
		}
		seq_len += 1;
	}

	for (u32 i = 0; i < seq_len; ++i)
	{
		i64 actual;
		switch (seq[i].type)
		{
			case SS_WRITE8:     { actual = ss_read_i8_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_LE: { actual = ss_read_i16_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE16_BE: { actual = ss_read_i16_be_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_LE: { actual = ss_read_i32_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE32_BE: { actual = ss_read_i32_be_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_LE: { actual = ss_read_i64_le_partial(ss_out, seq[i].bit_count); } break;
			case SS_WRITE64_BE: { actual = ss_read_i64_be_partial(ss_out, seq[i].bit_count); } break;
		}

		TEST_EQUAL(actual, seq[i].expected_signed);
	}

	free(seq);
	ss_free(&ss_1);

	return output;
}

struct ss_write_read_u32_partial_input 
{
	struct serialize_stream ss_1;
	struct serialize_stream ss_2;
	u64 n;
};

const u64 ss_write_read_u32_partial_size = 4*256*1024;

void *ss_write_read_u32_partial_init(void)
{
	struct ss_write_read_u32_partial_input *input = malloc(sizeof(struct ss_write_read_u32_partial_input));
	input->n = ss_write_read_u32_partial_size / 4;
	input->ss_1 = ss_alloc(NULL, ss_write_read_u32_partial_size);
	input->ss_2 = input->ss_1;

	return input;
}

void ss_write_read_u32_partial_reset(void *args)
{
	struct ss_write_read_u32_partial_input *input = args;
	input->ss_1.bit_index = 0;
	input->ss_2.bit_index = 0;
}

void ss_write_read_u32_partial_free(void *args)
{
	struct ss_write_read_u32_partial_input *input = args;
	free(input->ss_1.buf);
	free(input);
}

static void ss_write_read_u32_partial(void *args)
{
	struct ss_write_read_u32_partial_input *input = args;
	struct serialize_stream *ss_in = &input->ss_1;
	struct serialize_stream *ss_out = &input->ss_2;

	for (u32 i = 0; i < input->n; ++i)
	{
		const u64 bit_count = RngU64Range(1, 32);		
		const u64 expected = RngU64Range(0, U32_MAX) >> (32-bit_count);
		ss_write_u32_le_partial(ss_in, expected, bit_count);
		const u32 actual = ss_read_u32_le_partial(ss_out, bit_count);
	}
}

struct repetition_test repetition_test[] =
{
	{ .test =  &ss_randomized_aligned, .count = 100, },
	{ .test =  &ss_randomized_aligned_array, .count = 100, },
	{ .test =  &ss_randomized_partial, .count = 100, },
	{ .test =  &ss_randomized_sequence_partial, .count = 100, },
	{ .test =  &ss_randomized_sequence_partial_signed, .count = 100, },
};

struct suite m_serialize_suite =
{
	.id = "Serialize",
	.repetition_test = repetition_test,
	.repetition_test_count = sizeof(repetition_test) / sizeof(repetition_test[0]),
};

struct suite *serialize_suite = &m_serialize_suite;

struct serial_test serialize_serial_test[] =
{
	{ 
		.id = "ss_write_read_u32_partial", 
		.size = ss_write_read_u32_partial_size,
		.test = &ss_write_read_u32_partial,
		.test_init = &ss_write_read_u32_partial_init,
		.test_reset = &ss_write_read_u32_partial_reset,
		.test_free = &ss_write_read_u32_partial_free,
	},
};

struct performance_suite storage_performance_serialize_suite =
{
	.id = "Serialize Performance",
	.serial_test = serialize_serial_test,
	.serial_test_count = sizeof(serialize_serial_test) / sizeof(serialize_serial_test[0]),
};

struct performance_suite *serialize_performance_suite = &storage_performance_serialize_suite;


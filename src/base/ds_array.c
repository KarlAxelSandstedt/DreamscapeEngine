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

#include "ds_serialize.h"

void ds_ArraySerializeInternal(struct SS *ss, const void *buf, const u32 length, const u64 slot_size)
{
	ds_Assert(sizeof(u32) + length*slot_size <= SSBytesLeft(ss));
	SSWrite32Le(ss, (b32) { .u = length });
	SSWrite8N(ss, (const b8 *) buf, length*slot_size);
}

u32 ds_ArrayTryDeserializeInternal(struct arena *mem, struct SS *ss, void **buf, u32 *length, u32 *base, const u64 slot_size)
{
	*buf = NULL;
	*length = 0;
	*base = U32_MAX;
	if (SSBytesLeft(ss) < sizeof(u32))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u32 serialized_length = SSRead32Le(ss).u;
	/* division instead of length*slot_size: a corrupt length must not overflow the check */
	if (serialized_length > SSBytesLeft(ss) / slot_size)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	if (serialized_length > 0)
	{
		*buf = (mem) ? ArenaPush(mem, serialized_length*slot_size) : ds_Alloc(serialized_length*slot_size, DEFAULT_MEMORY_ALIGNMENT);
		if (*buf == NULL)
		{
			ss->bit_index = bit_index;
			return 0;
		}
		SSRead8N(ss, (b8 *) *buf, serialized_length*slot_size);
	}
	*length = serialized_length;
	return 1;
}

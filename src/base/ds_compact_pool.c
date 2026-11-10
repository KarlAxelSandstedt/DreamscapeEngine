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

u32 ds_CPoolAllocInternal(struct arena *mem, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable)
{
	ds_Assert(!alloc_growable || !mem);
	*buf = NULL;
	*length = 0;
	*count = 0;
	*growable = alloc_growable;
	*heap_allocated = 0;
	/* length 0: no block, not even the stub (a growable pool allocates on its first push) */
	if (alloc_length == 0)
	{
		return 1;
	}

	const u64 size = slot_size*((u64) alloc_length + 1);
	void *addr = (mem)
		? ArenaPushAligned(mem, size, DS_CACHE_LINE)
		: ds_Alloc(size, DS_CACHE_LINE);

	ds_Assert((u64) addr % DS_CACHE_LINE == 0);
	if (addr == NULL)
	{
		return 0;
	}

	*buf = (u8 *) addr + slot_size;
	*length = alloc_length;
	*heap_allocated = (mem == NULL);
	PoisonAddress(*buf, size - slot_size);
	return 1;
}

void ds_CPoolSerializeInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u64 slot_size)
{
	ds_Assert(2*sizeof(u32) + count*slot_size <= SSBytesLeft(ss));

	SSWriteU32Le(ss, length);
	SSWriteU32Le(ss, count);
	SSWrite8N(ss, (const b8 *) buf, count*slot_size);
}

u32 ds_CPoolTryDeserializeInternal(struct arena *mem, struct SS *ss, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_growable)
{
	*buf = NULL;
	*length = 0;
	*count = 0;
	*growable = alloc_growable;
	*heap_allocated = 0;
	if (SSBytesLeft(ss) < 2*sizeof(u32))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u32 serialized_length = SSReadU32Le(ss);
	const u32 serialized_count = SSReadU32Le(ss);
	/* division instead of count*slot_size: a corrupt count must not overflow the check */
	if (serialized_count > serialized_length || serialized_count > SSBytesLeft(ss) / slot_size)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	if (!ds_CPoolAllocInternal(mem, buf, length, count, growable, heap_allocated, slot_size, serialized_length, alloc_growable))
	{
		ss->bit_index = bit_index;
		return 0;
	}

	UnpoisonAddress(*buf, serialized_count*slot_size);
	SSRead8N(ss, (b8 *) *buf, serialized_count*slot_size);
	*count = serialized_count;
	return 1;
}

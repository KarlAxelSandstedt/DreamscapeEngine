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

/*
ds_array.h
==========
ds_Array(T) is an array of T with its length and base: { T *buf; u32 length; u32 base; }.
base is used by allocators, or as a value holding the base index at which the array starts
in a larger array.


::: Usage ::::

    Empty Array: { .buf = NULL, .length = 0, .base = U32_MAX }.

    DEFINE_ARRAY_STRUCT(item);
    ds_Array(item) items = ds_TSlabAdd(slab, 3);
    for (u32 i = 0; i < items.length; ++i) { ... items.buf[i] ... }


::: Documentation ::::

    // Return allocated array (Optionally on arena) with copied data. On failure return empty.
    // ds_ArrayCopy(mem, array)

    // WARNING: Platform-dependent (the elements' bytes).
    // Bytes ds_ArrayCompactSerialize writes.
    // ds_ArrayCompactSerializeSize(array, count)

    // Write the first count elements of the array to ss (at a byte boundary; it must fit).
    // ds_ArrayCompactSerialize(ss, array, count)

    // Bytes ds_ArraySerialize writes.
    // ds_ArraySerializeSize(array)

    // Write the array to ss (at a byte boundary; it must fit: ds_ArraySerializeSize).
    // ds_ArraySerialize(ss, array)

    // Read an array from ss into mem or the heap (mem == NULL; free it with ds_Free), with
    // the written length and base U32_MAX. Returns 1 on success; 0 if mem is full or ss
    // holds no valid array (the array is empty, ss's position unchanged).
    // ds_ArrayTryDeserialize(mem, ss, array)

*/

#ifndef __DS_ARRAY_H__
#define __DS_ARRAY_H__

#define ds_Array(T) ds_Array_ ## T

#define DEFINE_ARRAY_NATIVE(T)      \
typedef struct ds_Array(T)          \
{                                   \
	T *         buf;                \
	u32         length;             \
	u32         base;               \
} ds_Array(T)

#define DEFINE_ARRAY_STRUCT(T)      \
typedef struct ds_Array(T)          \
{                                   \
	struct T *  buf;                \
	u32         length;             \
	u32         base;               \
} ds_Array(T)

struct ds_ArrayUntyped
{
	void *      buf;
	u32         length;
	u32         base;
};

#define ds_ArrayEmpty()	((struct ds_ArrayUntyped) { .buf = NULL, .length = 0, .base = U32_MAX })

DEFINE_ARRAY_NATIVE(u8);
DEFINE_ARRAY_NATIVE(u16);
DEFINE_ARRAY_NATIVE(u32);
DEFINE_ARRAY_NATIVE(u64);
DEFINE_ARRAY_NATIVE(i8);
DEFINE_ARRAY_NATIVE(i16);
DEFINE_ARRAY_NATIVE(i32);
DEFINE_ARRAY_NATIVE(i64);
DEFINE_ARRAY_NATIVE(f32);
DEFINE_ARRAY_NATIVE(f64);
DEFINE_ARRAY_NATIVE(v2);
DEFINE_ARRAY_NATIVE(v3);
DEFINE_ARRAY_NATIVE(v4);
DEFINE_ARRAY_STRUCT(intv);

/* Internal: see ds_ArrayCopy. */
static ds_ForceInline struct ds_ArrayUntyped ds_ArrayCopyInternal(struct arena *mem, const void *buf, const u32 length, const u64 slot_size);

struct SS;

/* Internal: see ds_ArrayCompactSerialize. */
void ds_ArraySerializeInternal(struct SS *ss, const void *buf, const u32 length, const u64 slot_size);
/* Internal: see ds_ArrayTryDeserialize. */
u32  ds_ArrayTryDeserializeInternal(struct arena *mem, struct SS *ss, void **buf, u32 *length, u32 *base, const u64 slot_size);

/*************************** array: inline definitions ***************************/

#define ds_ArrayCopy(_mem_, _array_)                                                        \
    ((union { struct ds_ArrayUntyped u; ds_TypeOf(_array_) t; })                            \
        { .u = ds_ArrayCopyInternal((_mem_), (_array_).buf, (_array_).length,               \
                                    sizeof((_array_).buf[0])) }).t

static ds_ForceInline struct ds_ArrayUntyped ds_ArrayCopyInternal(struct arena *mem, const void *buf, const u32 length, const u64 slot_size)
{
    const u64 size = (u64) length*slot_size;
    void *copy = (mem)
               ? ArenaPush(mem, size)
               : ds_Alloc(size, DEFAULT_MEMORY_ALIGNMENT);
    if (copy == NULL)
    {
        return ds_ArrayEmpty();
    }

    memcpy(copy, buf, size);
    return (struct ds_ArrayUntyped) { .buf = copy, .length = length, .base = U32_MAX };
}

#define ds_ArrayCompactSerializeSize(_array_, _count_)  (sizeof(u32) + (u64) (_count_)*sizeof((_array_).buf[0]))
#define ds_ArrayCompactSerialize(_ss_, _array_, _count_)                                        \
	ds_ArraySerializeInternal((_ss_), (_array_).buf, (_count_), sizeof((_array_).buf[0]))
#define ds_ArraySerializeSize(_array_)      ds_ArrayCompactSerializeSize((_array_), (_array_).length)
#define ds_ArraySerialize(_ss_, _array_)    ds_ArrayCompactSerialize((_ss_), (_array_), (_array_).length)
#define ds_ArrayTryDeserialize(_mem_, _ss_, _array_)                                            \
	ds_ArrayTryDeserializeInternal((_mem_), (_ss_), (void **) &(_array_).buf, &(_array_).length, \
	                               &(_array_).base, sizeof((_array_).buf[0]))

#endif

/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt

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
ds_compact_size.h
=================
Bitcoin's CompactSize: an unsigned integer in 1, 3, 5 or 9 bytes serialized in n little-endian.
*/

#ifndef __DS_COMPACT_SIZE_H__
#define __DS_COMPACT_SIZE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"
#include "ds_serialize.h"

/* Return the required size when serializing val. */
static ds_ForceInline u64   ds_CompactSizeSerializeSize(const u64 val);
/* Serialize val. WARNING: Assumes val fits in the stream. */
static ds_ForceInline void  ds_CompactSizeSerialize(struct SS *ss, const u64 val);
/* Returns 1 on success. On failure return 0 (bad stream or not the shortest encoding) and reset the stream position. */
static ds_ForceInline u32   ds_CompactSizeTryDeserialize(struct SS *ss, u64 *val);


static ds_ForceInline u64 ds_CompactSizeSerializeSize(const u64 val)
{
    return (val < 253) 
            ? 1 
            : (val <= U16_MAX) ? 3 
            : (val <= U32_MAX) ? 5 : 9;
}

static ds_ForceInline void ds_CompactSizeSerialize(struct SS *ss, const u64 val)
{
    ds_Assert(ds_CompactSizeSerializeSize(val) <= SSBytesLeft(ss));

    if (val < 253)
    {
        SSWriteU8(ss, (u8) val);
    }
    else if (val <= U16_MAX)
    {
        SSWriteU8(ss, 0xFD);
        SSWriteU16Le(ss, (u16) val);
    }
    else if (val <= U32_MAX)
    {
        SSWriteU8(ss, 0xFE);
        SSWriteU32Le(ss, (u32) val);
    }
    else
    {
        SSWriteU8(ss, 0xFF);
        SSWriteU64Le(ss, val);
    }
}

static ds_ForceInline u32 ds_CompactSizeTryDeserialize(struct SS *ss, u64 *val)
{
    const u64 bit_index = ss->bit_index;
    if (SSBytesLeft(ss) < 1)
    {
        return 0;
    }

    const u8 first = SSReadU8(ss);
    const u64 size = (first < 253) 
                   ? 1 
                   : (first == 0xFD) ? 3 
                   : (first == 0xFE) ? 5 
                   : 9;
    if (SSBytesLeft(ss) < size - 1)
    {
        ss->bit_index = bit_index;
        return 0;
    }

    switch (size)
    {
        case 1: { *val = first;             } break;
        case 3: { *val = SSReadU16Le(ss);   } break;
        case 5: { *val = SSReadU32Le(ss);   } break;
        default: { *val = SSReadU64Le(ss);  } break;
    }

    if (ds_CompactSizeSerializeSize(*val) != size)
    {
        ss->bit_index = bit_index;
        return 0;
    }

    return 1;
}

#ifdef __cplusplus
}
#endif

#endif

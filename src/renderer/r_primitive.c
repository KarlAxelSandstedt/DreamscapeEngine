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

#include "r_local.h"

void r_PrimitiveWorldBufferAlloc(struct arena *mem, struct r_PrimitiveWorldBuffer *buffer, const u32 length, const u32 growable)
{
    ds_CPoolAlloc(mem, buffer->segment, length, growable);
    ds_CPoolAlloc(mem, buffer->point, length, growable);
    ds_CPoolAlloc(mem, buffer->sphere, length, growable);
    ds_CPoolAlloc(mem, buffer->capsule, length, growable);
    ds_CPoolAlloc(mem, buffer->box, length, growable);
    ds_CPoolAlloc(mem, buffer->transform, length, growable);
    if (!mem && !(buffer->segment.buf && buffer->point.buf && buffer->sphere.buf && buffer->capsule.buf
                  && buffer->box.buf && buffer->transform.buf))
    {
        LogString(T_RENDERER, S_FATAL, "primitive buffer: heap allocation failed");
        FatalCleanupAndExit();
    }
}

void r_PrimitiveWorldBufferFree(struct r_PrimitiveWorldBuffer *buffer)
{
    ds_CPoolDealloc(buffer->segment);
    ds_CPoolDealloc(buffer->point);
    ds_CPoolDealloc(buffer->sphere);
    ds_CPoolDealloc(buffer->capsule);
    ds_CPoolDealloc(buffer->box);
    ds_CPoolDealloc(buffer->transform);
}

void r_PrimitiveWorldBufferFlush(struct r_PrimitiveWorldBuffer *buffer)
{
    ds_CPoolFlush(buffer->segment);
    ds_CPoolFlush(buffer->point);
    ds_CPoolFlush(buffer->sphere);
    ds_CPoolFlush(buffer->capsule);
    ds_CPoolFlush(buffer->box);
    ds_CPoolFlush(buffer->transform);
}

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

#include "csg.h"
#include "sys_public.h"
#include "Log.h"

/*
global command identifiers
==========================
*/

struct csg csg_alloc(void)
{
	struct csg csg;

	csg.brush_db = strdb_Alloc(NULL, 32, 32, struct csgBRush, GROWABLE);
	csg.instance_pool = PoolAlloc(NULL, 32, struct csg_instance, GROWABLE);
	csg.node_pool = PoolAlloc(NULL, 32, struct csg_instance, GROWABLE);
	csg.frame = ArenaAlloc(1024*1024);
	csg.brush_marked_list = dll_Init(struct csgBRush);
	csg.instance_marked_list = dll_Init(struct csg_instance);
	csg.instance_non_marked_list = dll_Init(struct csg_instance);
	//csg.dcel_allocator = dcel_allocator_alloc(32, 32);

	struct csgBRush *stubBRush = strdb_Address(&csg.brush_db, STRING_DATABASE_STUB_INDEX);
	stubBRush->primitive = CSG_PRIMITIVE_BOX;
	stubBRush->dcel = DcelBoxStub();
	stubBRush->flags = CSG_CONSTANT;
	stubBRush->delta = NULL;
	stubBRush->cache = ui_NodeCacheNull();

	DcelAssertTopology(&stubBRush->dcel);

	return csg;
}

void csg_dealloc(struct csg *csg)
{
	strdb_Dealloc(&csg->brush_db);
	PoolDealloc(&csg->instance_pool);
	PoolDealloc(&csg->node_pool);
	ArenaFree(&csg->frame);
	//dcel_allocator_dealloc(csg->dcel_allocator);
}

void csg_flush(struct csg *csg)
{
	strdb_Flush(&csg->brush_db);
	PoolFlush(&csg->instance_pool);
	PoolFlush(&csg->node_pool);
	ArenaFlush(&csg->frame);
	dll_Flush(&csg->brush_marked_list);
	dll_Flush(&csg->instance_marked_list);
	dll_Flush(&csg->instance_non_marked_list);
	//dcel_allocator_flush(csg->dcel_allocator);
}

void csg_serialize(struct serialStream *ss, const struct csg *csg)
{

}

struct csg csg_deserialize(struct arena *mem, struct serialStream *ss, const u32 growable)
{
	ds_Assert(!mem || !growable);
}

static void csg_apply_delta(struct csg *csg)
{

}

static void csg_remove_marked_structs(struct csg *csg)
{
	struct csgBRush *brush = NULL;
	for (u32 i = csg->brush_marked_list.first; i != DLL_NULL; i = dll_Next(brush))
	{
		brush = strdb_Address(&csg->brush_db, i);
		if ((brush->flags & CSG_CONSTANT) || brush->reference_count)
		{
			brush->flags &= ~CSG_MARKED_FOR_REMOVAL;
			dll_Remove(&csg->brush_marked_list, csg->brush_db.pool.buf, i);
			continue;
		}

		utf8 id = brush->id;
		strdb_Remove(&csg->brush_db, id);
		ThreadFree256B(id.buf);
	}

	dll_Flush(&csg->brush_marked_list);
	dll_Flush(&csg->instance_marked_list);
}

void csg_main(struct csg *csg)
{
	/* (1) Apply deltas */
	csg_apply_delta(csg);

	/* (2) Safe to flush frame now */
	ArenaFlush(&csg->frame);

	/* (3) Remove markged csg structs */
	csg_remove_marked_structs(csg);
}

struct slot csgBRush_add(struct csg *csg, const utf8 id)
{
	if (id.size > 256)
	{
		Log(T_CSG, S_WARNING, "Failed to create csgBRush, id %k requires size > 256B.", &id);
		return empty_slot; 
	}

	void *buf = ThreadAlloc256B();
	const utf8 heap_id = Utf8CopyBuffered(buf, 256, id);
	struct slot slot = strdb_AddAndAlias(&csg->brush_db, heap_id);
	if (!slot.address)
	{
		Log(T_CSG, S_WARNING, "Failed to create csgBRush, brush with id %k already exist.", &id);
		ThreadFree256B(buf);
	}
	else
	{
		struct csgBRush *brush = slot.address;
		brush->primitive = CSG_PRIMITIVE_BOX;
		brush->dcel = DcelBoxStub();
		brush->flags = CSG_FLAG_NONE;
		brush->delta = NULL;

		brush->cache = ui_NodeCacheNull();
	}

	return slot;
}

void csgBRush_mark_for_removal(struct csg *csg, const utf8 id)
{
	struct slot slot = strdb_Lookup(&csg->brush_db, id);
	struct csgBRush *brush = slot.address;
	if (brush && !(brush->flags & (CSG_CONSTANT | CSG_MARKED_FOR_REMOVAL)))
	{
		brush->flags |= CSG_MARKED_FOR_REMOVAL;
		dll_Append(&csg->brush_marked_list, csg->brush_db.pool.buf, slot.index);
	}
}

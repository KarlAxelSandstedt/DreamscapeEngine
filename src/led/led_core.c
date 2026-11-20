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

#include "led_local.h"
#include "ds_random.h"


struct slot led_NodeLookupId(struct led *led, const utf8 id)
{
	struct slot slot = empty_slot;
	const u32 hash = Utf8Hash(id);
	for (u32 i = ds_HashMapFirst(&led->node_map, hash); i != HASH_NULL; i = ds_HashMapNext(&led->node_map, i))
	{
		struct led_Node *node = led->node_hierarchy.pool.buf + i;
		if (Utf8Equivalence(id, node->id))
		{
			slot.index = i;
			slot.address = node;
			break;
		}
	}

	return slot;
}

struct slot led_NodeLookup(struct led *led, const ds_Id id)
{
    if (ds_IdNullCheck(id))
    {
        return empty_slot;        
    }

    struct slot slot;
    slot.index = ds_IdIndex(id);
    if (slot.index >= led->node_hierarchy.pool.count_max)
    {
        return empty_slot;
    }

    slot.address = led->node_hierarchy.pool.buf + slot.index;
    struct led_Node *node = slot.address;
    return ds_IdEqual(node->tagged_id, id)
        ? slot 
        : empty_slot;
}

static ds_Id led_NodeInitalize(struct led_Node *node, const u32 node_index, const u64 flags)
{
    node->tagged_id = ds_IdNextGeneration(ds_IdInit(node_index, ds_IdGeneration(node->tagged_id)));
	node->flags = flags;
	node->body_prefab = SDB_STUB;
    node->shape_prefab = SDB_STUB;
    node->proxy = PROXY3D_NULL;
    node->transform = ds_TransformWorldIdentity();
    node->color = V4(0.9f, 0.9f, 0.9f, 1.0f);

    return node->tagged_id;
}

static ds_Id led_NodeAnonymousAdd(struct led *led, const ds_Id parent_id)
{
    struct led_Node *node = led_NodeLookup(led, parent_id).address;
    ds_Assert(ds_IdNullCheck(parent_id) || node);

    u32 parent = LED_NODE_ROOT;
    if (!ds_IdNullCheck(parent_id))
    {
        parent = ds_IdIndex(parent_id);
    }

    struct slot slot = HIAdd(led->node_hierarchy, parent);
    return led_NodeInitalize(slot.address, slot.index, LED_ANONYMOUS);
}

ds_Id led_NodeAdd(struct led *led, const utf8 id, const utf8 parent_id)
{
    if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: id must not be empty");
        return ds_IdNull();
	} 

	if (led_NodeLookupId(led, id).address != NULL) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: node with given id already exist");
        return ds_IdNull();
	}

    u32 parent = LED_NODE_ROOT;
    const struct slot slot_parent = led_NodeLookupId(led, parent_id);
    if (slot_parent.address)
    {
        parent = slot_parent.index;
    }
    else if (parent_id.len)
    {
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: parent with given id does not exist");
        return ds_IdNull();
    }

    struct slot slot = HIAdd(led->node_hierarchy, parent);
    if (!slot.address)
    {
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: node hierarchy is full");
        return ds_IdNull();
    }
    struct led_Node *node = slot.address;
	node->id = Utf8Copy(&led->mem_persistent, id);
	if (!node->id.len)
	{
        HIRemove(&led->frame, led->node_hierarchy, slot.index);
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: persistent memory is full");
        return ds_IdNull();
	}
	const u32 hash = Utf8Hash(node->id);
	ds_HashMapAdd(&led->node_map, hash, slot.index);

    return led_NodeInitalize(node, slot.index, LED_FLAG_NONE);
}

static void led_NodeRemoveResources(void *node_void, const u32 index, void *led_void)
{
    struct led *led = led_void;
    struct led_Node *node = node_void;

	ds_SDBDereference(led->body_prefab_db, node->body_prefab);
    node->body_prefab = SDB_STUB;

    ds_SDBDereference(led->shape_prefab_db, node->shape_prefab);
    node->shape_prefab = SDB_STUB;

	r_Proxy3dDealloc(&led->frame, node->proxy);
    node->proxy = PROXY3D_NULL;
		
    if ((node->flags & LED_ANONYMOUS) == 0)
    {
        const u32 hash = Utf8Hash(node->id);
        ds_HashMapRemove(&led->node_map, hash, index);
    }
}

void led_NodeRemoveId(struct led *led, const utf8 id)
{
	struct slot slot = led_NodeLookupId(led, id);
	struct led_Node *node = slot.address;
    if (node)
    {
        HIApplyCustomFreeAndRemove(&led->frame, led->node_hierarchy, slot.index, &led_NodeRemoveResources, led);
    }
}

void led_NodeRemove(struct led *led, const ds_Id id)
{
    struct led_Node *node = led_NodeLookup(led, id).address;
    if (node && ds_IdEqual(node->tagged_id, id))
    {
        HIApplyCustomFreeAndRemove(&led->frame, led->node_hierarchy, ds_IdIndex(id), &led_NodeRemoveResources, led);
    }
}

void led_NodeSetPositionId(struct led *led, const utf8 id, const v3f64 position)
{
	struct slot slot = led_NodeLookupId(led, id);
	struct led_Node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %k, node not found.", &id);
	}
	else
	{
		node->transform.position = position;
	}
}

void led_NodeSetPosition(struct led *led, const ds_Id id, const v3f64 position)
{
	struct led_Node *node = led_NodeLookup(led, id).address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %lu, node not found.", id);
	}
	else
	{
		node->transform.position = position;
	}
}

void led_NodeSetColor(struct led *led, const ds_Id id, const v4 color, const f32 blend)
{
	struct led_Node *node = led_NodeLookup(led, id).address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set color of led node %lu, node not found.", id);
	}
	else
	{
		node->color = color;
        node->blend = blend;
	}
}

void led_NodeSetColorId(struct led *led, const utf8 id, const v4 color, const f32 blend)
{
	struct slot slot = led_NodeLookupId(led, id);
	struct led_Node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set color of led node %k, node not found.", &id);
	}
	else
	{
		node->color = color;
        node->blend = blend;
	}
}

static void led_NodeDetachRigidBodyPrefabInternal(struct led *led, struct led_Node *node)
{
    if (node->flags & LED_BODY_PREFAB)
    {
	    ds_SDBDereference(led->body_prefab_db, node->body_prefab);
        struct led_Node *child = NULL;
        for (i32 i = node->hi_first; i != HI_NULL; )
        {
            child = led->node_hierarchy.pool.buf + i;
            const i32 next = child->hi_next;
            if (child->flags & LED_SHAPE_PREFAB)
            {
                HIApplyCustomFreeAndRemove(&led->frame, led->node_hierarchy, i, &led_NodeRemoveResources, led);
            }
            i = next;
        }

	    r_Proxy3dDealloc(&led->frame, node->proxy);

        node->body_prefab = SDB_STUB;
        node->proxy = PROXY3D_NULL;
    }
}

static void led_NodeAttachRigidBodyPrefabInternal(struct led *led, const u32 node_index, const utf8 prefab)
{
    struct slot slot = ds_SDBReference(led->body_prefab_db, prefab);
	if (slot.index == SDB_STUB)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, prefab not found.", &prefab);
        return;
	}

    struct led_Node *node = led->node_hierarchy.pool.buf + node_index;
    led_NodeDetachRigidBodyPrefabInternal(led, node); 

    const struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
    struct r_Proxy3d_config config =
	{
		.parent = PROXY3D_ROOT,
		.linear_velocity = V3(0.0f, 0.0f, 0.0f),
		.angular_velocity = V3(0.0f, 0.0f, 0.0f),
		.mesh = Utf8Inline(""),
		.ns_time = 1000 * host->sim.us_time,
        .blend = node->blend,
	};
    config.color = node->color;
	config.position = node->transform.position;
	config.rotation = node->transform.rotation;

    node->flags |= LED_BODY_PREFAB;
	node->body_prefab = slot.index;
    node->proxy = r_Proxy3dAlloc(&config);

    struct r_Proxy3d *proxy = r_Proxy3dAddress(node->proxy);
    proxy->flags &= ~PROXY3D_DRAW;

    const struct ds_BodyPrefab *body = slot.address;
    const struct ds_ShapePrefabInstance *instance = NULL;
    for (i32 i = body->shape_list.first; i != DLL_SENTINEL; i = instance->body_shape.next)
    {
        instance = led->shape_prefab_instance_pool.buf + i;
        const struct ds_ShapePrefab *shape = led->shape_prefab_db.pool.buf + instance->shape_prefab;
        const struct r_Mesh *render_mesh = led->render_mesh_db.pool.buf + shape->render_mesh;

        const ds_Id child_id = led_NodeAnonymousAdd(led, node->tagged_id);
        node = led->node_hierarchy.pool.buf + node_index;
        struct led_Node *child = led_NodeLookup(led, child_id).address;

        child->transform = ds_TransformPromote(&instance->t_local);
        child->flags |= LED_SHAPE_PREFAB;
        child->shape_prefab = ds_SDBReference(led->shape_prefab_db, shape->id).index;
        child->color = node->color;
        child->blend = node->blend;

		config.mesh = render_mesh->id;
        config.parent = node->proxy;
		config.position = child->transform.position;
		config.rotation = child->transform.rotation;
		child->proxy = r_Proxy3dAlloc(&config);
    }
}

void led_NodeAttachRigidBodyPrefabId(struct led *led, const utf8 id, const utf8 prefab)
{
	struct slot slot = led_NodeLookupId(led, id);
	if (!slot.address)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node not found.", &id);
	}
	else
	{
        led_NodeAttachRigidBodyPrefabInternal(led, slot.index, prefab);
	}
}

void led_NodeAttachRigidBodyPrefab(struct led *led, const ds_Id id, const utf8 prefab)
{
	const struct slot slot = led_NodeLookup(led, id);
	if (!slot.address)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %lu, node not found.", id);
	}
	else
	{
        led_NodeAttachRigidBodyPrefabInternal(led, slot.index, prefab);
	}
}

void led_NodeDetachRigidBodyPrefabId(struct led *led, const utf8 id)
{
	struct slot slot = led_NodeLookupId(led, id);
	if (!slot.address)
	{
		Log(T_LED, S_WARNING, "Failed to attach body prefab to led node %k, node not found.", &id);
	}
	else
	{
        led_NodeDetachRigidBodyPrefabInternal(led, slot.address);
	}
}

void led_NodeDetachRigidBodyPrefab(struct led *led, const ds_Id id)
{
	struct led_Node *node = led_NodeLookup(led, id).address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to detach body prefab from led node %lu, node not found.", id);
	}
	else
	{
        led_NodeDetachRigidBodyPrefabInternal(led, node);
	}
}

static struct slot led_CollisionShapeAdd(struct led *led, const struct c_Shape *shape)
{
	const struct slot slot = c_ShapeDBAdd(&led->cs_db, shape->id, shape);
	if (slot.index != SDB_STUB)
	{
		DLLAppend(led->cs_shape_list, led->cs_db.sdb.pool.buf, slot.index, led_shape_node);
	}
	return slot;
}

struct slot led_CollisionShapeDefaultAdd(struct led *led, const utf8 id)
{
    const v3 hw = V3(0.5f, 0.5f, 0.5f);
	return led_CollisionBoxAdd(led, id, hw);	
}

struct slot led_CollisionBoxAdd(struct led *led, const utf8 id, const v3 hw)
{
	//TODO Need to come up with a memory allocation strategy for dcels
    struct slot slot = empty_slot;
	if (hw.x > 0.0f && hw.y > 0.0f && hw.z > 0.0f)
	{
		struct c_Shape shape =
		{
			.id = id, 
			.type = C_SHAPE_CONVEX_HULL,
			.hull = DcelBox(&led->mem_persistent, hw), 
		};

		slot = led_CollisionShapeAdd(led, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision box: bad parameters");
	}

    return slot;
}

struct slot led_CollisionDcelAdd(struct led *led, const utf8 id, struct dcel *dcel)
{
    struct slot slot = empty_slot;
	if (dcel->v_count)
	{
		struct c_Shape shape =
		{
			.id = id,
			.type = C_SHAPE_CONVEX_HULL,
			.hull = *dcel, 
		};

		slot = led_CollisionShapeAdd(led, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision dcel: bad parameters");
	}

    return slot;
}

struct slot led_CollisionMeshAdd(struct led *led, const utf8 id, struct mesh *mesh)
{
    struct slot slot = empty_slot;
	if (mesh->v_count && mesh->bvh.bt.count)
	{
		struct c_Shape shape =
		{
			.id = id,
			.type = C_SHAPE_TRI_MESH,
			.mesh = *mesh, 
		};

		slot = led_CollisionShapeAdd(led, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision tri_mesh: bad parameters");
	}

    return slot;
}

struct slot led_CollisionSphereAdd(struct led *led, const utf8 id, const f32 radius)
{
    struct slot slot = empty_slot;
	struct c_Shape shape =
	{
		.id = id,
		.type = C_SHAPE_SPHERE,
		.sphere = { .radius = radius },
	};

	if (shape.sphere.radius > 0.0f)
	{
		slot = led_CollisionShapeAdd(led, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision sphere: bad parameters");
	}

    return slot;
}

struct slot led_CollisionCapsuleAdd(struct led *led, const utf8 id, const f32 radius, const f32 half_height)
{
    struct slot slot = empty_slot;
	struct c_Shape shape =
	{
		.id = id,
		.type = C_SHAPE_CAPSULE,
		.capsule = 
		{ 
			.radius = radius,
			.half_height = half_height,
		},
	};

	if (shape.capsule.radius > 0.0f && shape.capsule.half_height > 0.0f)
	{
		slot = led_CollisionShapeAdd(led, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision capsule: bad parameters");
	}

    return slot;
}

void led_CollisionShapeRemove(struct led *led, const utf8 id)
{
	struct slot slot = led_CollisionShapeLookup(led, id);
	struct c_Shape *shape = slot.address;
	if (slot.index != SDB_STUB && shape->reference_count == 0)
	{
		void *buf = shape->id.buf;
		DLLRemove(led->cs_shape_list, led->cs_db.sdb.pool.buf, slot.index, led_shape_node);
		c_ShapeDBRemove(&led->cs_db, id);
	}
}

struct slot led_CollisionShapeLookup(struct led *led, const utf8 id)
{
	return c_ShapeDBLookup(&led->cs_db, id);
}

struct slot led_ShapePrefabAdd(struct led *led, const utf8 id, const utf8 cshape, const f32 density, const f32 restitution, const f32 friction)
{
	struct slot slot = empty_slot;	
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate ds_ShapePrefab: prefab->id must not be empty");
	} 
	else if (ds_SDBLookup(led->shape_prefab_db, id).index != SDB_STUB) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate ds_ShapePrefab: prefab with given id already exist");
	}
	else
	{ 
		u8 buf[PREFAB_BUFSIZE];
		const utf8 copy = Utf8CopyBuffered(buf, PREFAB_BUFSIZE, id);	
		if (!copy.len)
		{
			Log(T_LED, S_WARNING, "Failed to allocate ds_ShapePrefab: prefab->id size must be <= %luB", PREFAB_BUFSIZE);
		}
		else
		{
			struct slot ref = c_ShapeDBReference(&led->cs_db, cshape);
			if (ref.index == SDB_STUB)
			{
				LogString(T_LED, S_WARNING, "In ds_ShapePrefab: shape not found, stub_shape chosen");
			}
			slot = ds_SDBAdd(&led->mem_persistent, led->shape_prefab_db, copy);

			struct ds_ShapePrefab *prefab = slot.address;
			prefab->cshape = ref.index;
            prefab->render_mesh = SDB_STUB;
			prefab->restitution = restitution;
			prefab->friction = friction;
			prefab->density = density;
		}
	}

	return slot;
}

void led_ShapePrefabRemove(struct led *led, const utf8 id)
{
	struct slot slot = led_ShapePrefabLookup(led, id);
	struct ds_ShapePrefab *prefab = slot.address;
	if (slot.index != SDB_STUB && prefab->reference_count == 0)
	{
		c_ShapeDBDereference(&led->cs_db, prefab->cshape);
        ds_SDBDereference(led->render_mesh_db, prefab->render_mesh);
		ds_SDBRemove(led->shape_prefab_db, id);
	}	
}

struct slot led_ShapePrefabLookup(struct led *led, const utf8 id)
{
	return ds_SDBLookup(led->shape_prefab_db, id);
}

void led_ShapePrefabAttachRenderMesh(struct led *led, const utf8 id, const utf8 render_mesh)
{
    struct slot slot_shape = led_ShapePrefabLookup(led, id);
    struct slot slot_mesh = led_RenderMeshLookup(led, render_mesh);
    if (slot_shape.index == SDB_STUB) 
	{
		Log(T_LED, S_WARNING, "Failed to attach render mesh to shape: shape %k does not exist.", &id);
	} 
    else if (slot_mesh.index == SDB_STUB) 
	{
		Log(T_LED, S_WARNING, "Failed to attach render mesh to shape: mesh %k does not exist.", &render_mesh);
	}
    else
	{ 
        led_ShapePrefabDetachRenderMesh(led, id);
        struct ds_ShapePrefab *shape = slot_shape.address;
        shape->render_mesh = ds_SDBReference(led->render_mesh_db, render_mesh).index;
    }
}

void led_ShapePrefabDetachRenderMesh(struct led *led, const utf8 id)
{
    struct slot slot = led_ShapePrefabLookup(led, id);
    if (slot.index != SDB_STUB)
    {
        struct ds_ShapePrefab *shape = slot.address;
        ds_SDBDereference(led->render_mesh_db, shape->render_mesh);
    }
}

struct slot led_RigidBodyPrefabAdd(struct led *led, const utf8 id, const u32 dynamic)
{
	struct slot slot = empty_slot;	
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate body_prefab: prefab->id must not be empty");
	} 
	else if (ds_SDBLookup(led->body_prefab_db, id).index != SDB_STUB) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate body_prefab: prefab with given id already exist");
	}
	else
	{ 
		u8 buf[PREFAB_BUFSIZE];
		const utf8 copy = Utf8CopyBuffered(buf, PREFAB_BUFSIZE, id);	
		if (!copy.len)
		{
			Log(T_LED, S_WARNING, "Failed to allocate body_prefab: prefab->id size must be <= %luB", PREFAB_BUFSIZE);
		}
		else
		{
			struct ds_BodyPrefab *prefab = ds_SDBAdd(&led->mem_persistent, led->body_prefab_db, copy).address;
            DLLFlush(prefab->shape_list);
			prefab->dynamic = dynamic;
        }
	}

	return slot;
}

static void led_ShapePrefabInstanceRemove(struct led *led, const u32 i)
{
    const struct ds_ShapePrefabInstance *instance = led->shape_prefab_instance_pool.buf + i;
    ds_SDBDereference(led->shape_prefab_db, instance->shape_prefab);
    ds_PoolRemove(led->shape_prefab_instance_pool, i);
}

void led_RigidBodyPrefabRemove(struct led *led, const utf8 id)
{
	struct slot slot = led_RigidBodyPrefabLookup(led, id);
	struct ds_BodyPrefab *prefab = slot.address;
	if (slot.index != SDB_STUB && prefab->reference_count == 0)
	{
        for (i32 i = prefab->shape_list.first; i != DLL_SENTINEL; )
        {
            const i32 next = led->shape_prefab_instance_pool.buf[i].body_shape.next;
            led_ShapePrefabInstanceRemove(led, i);
            i = next;
        }

		void *buf = prefab->id.buf;
		ds_SDBRemove(led->body_prefab_db, id);
	}	
}

struct slot led_RigidBodyPrefabLookup(struct led *led, const utf8 id)
{
	return ds_SDBLookup(led->body_prefab_db, id);
}

void led_RigidBodyPrefabAttachShape(struct led *led, const utf8 rb_id, const utf8 shape_id, const utf8 local_shape_id, const ds_Transform *t_local)
{
    struct ds_BodyPrefab *body_prefab = led_RigidBodyPrefabLookup(led, rb_id).address;
    struct ds_ShapePrefab *shape_prefab = led_ShapePrefabLookup(led, shape_id).address;
    if (!body_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to ds_BodyPrefab %k, prefab doesn't exist", &rb_id);
    } 
    else if (!shape_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to ds_BodyPrefab %k, ds_ShapePrefab %k does not exist.", &rb_id, &shape_id);
    }
    else if (led_RigidBodyPrefabLookupShape(led, rb_id, local_shape_id).address != NULL)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to to ds_BodyPrefab %k, a shape instance with local id %k already exists in the body.", &rb_id, &local_shape_id);
    }
    else
    {
        struct slot slot = ds_PoolAdd(led->shape_prefab_instance_pool);
        struct ds_ShapePrefabInstance *instance = slot.address;

        DLLPrepend(body_prefab->shape_list, led->shape_prefab_instance_pool.buf, slot.index, body_shape);
        instance->id = Utf8Copy(&led->mem_persistent, local_shape_id);
        instance->shape_prefab = ds_SDBReference(led->shape_prefab_db, shape_id).index;
        instance->t_local = *t_local;
    }
}

void led_RigidBodyPrefabDetachShape(struct led *led, const utf8 rb_id, const utf8 local_shape_id)
{
    struct ds_BodyPrefab *body_prefab = led_RigidBodyPrefabLookup(led, rb_id).address;
    struct slot slot = led_RigidBodyPrefabLookupShape(led, rb_id, local_shape_id);
    struct ds_ShapePrefabInstance *instance = slot.address;
    if (!body_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to deattach shape to ds_BodyPrefab %k, prefab doesn't exist", &rb_id);
    } 
    else if (!instance)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to to ds_BodyPrefab %k, a shape instance with local id %k does not exist in the body.", &rb_id, &local_shape_id);
    }
    else
    {
        DLLRemove(body_prefab->shape_list, led->shape_prefab_instance_pool.buf, slot.index, body_shape);
        led_ShapePrefabInstanceRemove(led, slot.index);
    }
}

struct slot led_RigidBodyPrefabLookupShape(struct led *led, const utf8 rb_id, const utf8 local_shape_id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct slot prefab_slot = led_RigidBodyPrefabLookup(led, rb_id);
    if (prefab_slot.index != SDB_STUB)
    {
        const struct ds_BodyPrefab *prefab = prefab_slot.address;
        struct ds_ShapePrefabInstance *instance = NULL;
        for (i32 i = prefab->shape_list.first; i != DLL_SENTINEL; i = instance->body_shape.next)
        {
            instance = led->shape_prefab_instance_pool.buf + i;
            if (Utf8Equivalence(instance->id, local_shape_id))
            {
                slot.index = i;
                slot.address = instance;
                break;
            }
        }
    }

    return slot;
}

struct slot led_RenderMeshAdd(struct led *led, const utf8 id, const utf8 shape)
{
	struct slot slot = empty_slot;
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate render mesh: id must not be empty");
	} 
	else if (ds_SDBLookup(led->render_mesh_db, id).index != SDB_STUB) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate render mesh: mesh with given id already exist");
	}
	else
	{
		slot = ds_SDBAdd(&led->mem_persistent, led->render_mesh_db, id);
		struct r_Mesh *mesh = slot.address;

		struct slot ref = c_ShapeDBLookup(&led->cs_db, shape);
		if (ref.index == SDB_STUB)
		{
			LogString(T_LED, S_WARNING, "In render_mesh_add: shape not found, stub chosen");
		}

	    //TODO Need to come up with a memory allocation strategy for r_Meshes
		const struct c_Shape *s = ref.address;
		switch (s->type)
		{
			case C_SHAPE_SPHERE: { r_MeshSphere(&led->mem_persistent, mesh, s->sphere.radius, 12); } break;
			case C_SHAPE_CAPSULE: { r_MeshCapsule(&led->mem_persistent, mesh, s->capsule.half_height, s->capsule.radius, 16); } break;
			case C_SHAPE_CONVEX_HULL: { r_MeshHull(&led->mem_persistent, mesh, &s->hull); } break;
			case C_SHAPE_TRI_MESH: { r_MeshTriMesh(&led->mem_persistent, mesh, &s->mesh); } break;
		}
	}

	return slot;
}

void led_RenderMeshRemove(struct led *led, const utf8 id)
{
	struct slot slot = ds_SDBLookup(led->render_mesh_db, id);
	struct r_Mesh *mesh = slot.address;
	if (slot.index != SDB_STUB && mesh->reference_count == 0)
	{
		void *buf = mesh->id.buf;
		ds_SDBRemove(led->render_mesh_db, id);
	}
}

struct slot led_RenderMeshLookup(struct led *led, const utf8 id)
{
	return ds_SDBLookup(led->render_mesh_db, id);
}

static struct mesh TriMeshPerlinNoise(struct arena *mem_persistent, const u32 n, const f32 width)
{
	ds_Assert(PowerOfTwoCheck(n) && n >= 32);

	struct arena *tmp = ArenaPushScratch();

	struct mesh mesh = 
	{
		.v_count = (n-1)*(n-1),
		.tri_count = 2*(n-2)*(n-2),
	};
	mesh.v = ArenaPush(mem_persistent, mesh.v_count*sizeof(v3));
	mesh.tri = ArenaPush(mem_persistent, mesh.tri_count*sizeof(v3u32));
	//TODO move out functino to TriMeshPerlinNoise method
	//TODO return stub mesh
	ds_Assert(mesh.v && mesh.tri);

	const f32 unit = width / n;

#define OCTAVES	6
	v2 *grad[OCTAVES];
	for (u32 o = 0; o < OCTAVES; ++o)
	{
		const u32 on = (n >> o) + 1;
		grad[o] = ArenaPush(tmp, on*on*sizeof(v2));
		for (u32 x = 0; x < on; ++x)
		{
			for (u32 z = 0; z < on; ++z)
			{
				const f32 angle = RngF32Range(0.0f, F32_PI2);
				grad[o][x*on + z] = V2(F32Cos(angle), F32Sin(angle));
			}
		}
	}

	const v3 offset = V3(-unit * (n >> 1),
			     -20.0f,
			     -unit * (n >> 1) + 30.0f);

	for (u32 x = 0; x < n-1; ++x)
	{
		for (u32 z = 0; z < n-1; ++z)
		{
			v3 *p = mesh.v + x*(n-1) + z;
			*p = V3((0.5f + x) * unit, 0.0f, (0.5f + z) * unit);

			f32 amplitude = 1.0f / (1 << OCTAVES);
			for (u32 i = 0; i < OCTAVES; ++i)
			{
				const u32 on = (n >> i) + 1;
				const u32 x_low = x - (x % (1 << i));
				const u32 z_low = z - (z % (1 << i));

				const u32 x_high = x_low + (1 << i);
				const u32 z_high = z_low + (1 << i);
	
				const v2 bl_diff = V2(p->x - x_low * unit, p->z - z_low * unit);
				const v2 tl_diff = V2(p->x - x_low * unit, p->z - z_high * unit);
				const v2 br_diff = V2(p->x - x_high * unit, p->z - z_low * unit);
				const v2 tr_diff = V2(p->x - x_high * unit, p->z - z_high * unit);

				const u32 xg_low = x_low >> i;
				const u32 xg_high = xg_low + 1;
				const u32 zg_low = z_low >> i;
				const u32 zg_high = zg_low + 1;

				ds_Assert(xg_low*on  + zg_low  < on*on);
				ds_Assert(xg_high*on + zg_low  < on*on);
				ds_Assert(xg_low*on  + zg_high < on*on);
				ds_Assert(xg_high*on + zg_high < on*on);

				const f32 bl_dot = V2Dot(bl_diff, grad[i][xg_low*on  + zg_low]);
				const f32 br_dot = V2Dot(br_diff, grad[i][xg_high*on + zg_low]);
				const f32 tl_dot = V2Dot(tl_diff, grad[i][xg_low*on  + zg_high]);
				const f32 tr_dot = V2Dot(tr_diff, grad[i][xg_high*on + zg_high]);

				const v2 low = V2(x_low * unit, z_low * unit);
				const v2 high = V2(x_high * unit, z_high * unit);
				const v2 t = V2((p->x - low.x) / (high.x - low.x),
						(p->z - low.y) / (high.y - low.y));

				const v2 smoothstep = V2(6*t.x*t.x*t.x*t.x*t.x - 15*t.x*t.x*t.x*t.x + 10*t.x*t.x*t.x,
							 6*t.y*t.y*t.y*t.y*t.y - 15*t.y*t.y*t.y*t.y + 10*t.y*t.y*t.y);

				const f32 vb = bl_dot*(1.0f - smoothstep.x) + br_dot*smoothstep.x;
				const f32 vt = tl_dot*(1.0f - smoothstep.x) + tr_dot*smoothstep.x;
				const f32 perlin = vb*(1.0f - smoothstep.y) + vt*smoothstep.y;

				p->y += perlin * amplitude;
				amplitude *= 2.0f;
			}

			*p = V3Add(*p, offset);
		}
	}

	for (u32 x = 0; x < n-2; ++x)
	{
		for (u32 z = 0; z < n-2; ++z)
		{
			mesh.tri[2*(x*(n-2) + z) + 0] = V3U32(x*(n-1) + z, x*(n-1) + z+1, (x+1)*(n-1) + z);
			mesh.tri[2*(x*(n-2) + z) + 1] = V3U32((x+1)*(n-1) + z, x*(n-1) + z+1, (x+1)*(n-1) + z+1);
		}
	}

	ArenaPopScratch();

	/* centered on its box: not required, but f32 vertices are most precise near the origin */
	const struct aabb bbox = MeshBbox(&mesh);
	const v3 local_origin = V3Scale(V3Add(bbox.min, bbox.max), -0.5f);
	for (u32 i = 0; i < mesh.v_count; ++i)
	{
		mesh.v[i] = V3Add(mesh.v[i], local_origin);
	}

	return mesh;
}

static void led_DistanceJointAdd(struct led *led, const struct ds_DistanceJointPrefab *prefab, const ds_Id id0, const ds_Transform *t0, const ds_Id id1, const ds_Transform *t1, const f32 distance)
{
    struct led_Joint *joint = ds_CPoolPush(led->joint_pool).address;
    joint->prefab = *prefab;
    joint->id[0] = id0;
    joint->id[1] = id1;
    joint->local_frame[0] = *t0;
    joint->local_frame[1] = *t1;
}

#define ROPE_COUNT 10
/* setup scenes: a layout position (relative to led->scene_origin) in world */
static v3f64 led_ScenePosition(const struct led *led, const v3 layout)
{
    return V3F64Add(led->scene_origin, V3Promote(layout));
}

void led_RopeSetup(struct led *led)
{
    ds_CPoolFlush(led->joint_pool);

	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    utf8 id;
    ds_Id tagged_id;

    const f32 rope_radius = 0.125f;
    const f32 rope_half_height = 0.125f;
    const u32 floor_count = 0;

	const f32 floor_friction = 0.8f;
	const f32 capsule_friction = 0.1f;

	const f32 alpha1 = 0.7f;
	const f32 alpha2 = 0.5f;
	const v4 floor_color = V4(0.8f, 0.6f, 0.6f, alpha2);
	const v4 capsule_color = V4(0.1f, 0.4f, 0.8f, alpha2);

	const f32 ramp_width = 10.0f;
	const f32 ramp_length = 60.0f;
	const f32 ramp_height = 34.0f;
	const u32 v_count = 6;
	v3 ramp_vertices[6] = 
	{
		V3(0.0f, ramp_height, -ramp_length),
		V3(ramp_width, ramp_height, -ramp_length),
		V3(0.0f, 0.0f, -ramp_length),
		V3(ramp_width, 0.0f, -ramp_length),
		V3(0.0f, 0.0f, 0.0f),
		V3(ramp_width, 0.0f, 0.0f),
	};


	id = Utf8Cstr(sys_win->ui->mem_frame, "c_ceil");
    const v3 ceil_hw = V3(10.0f, 0.25f, 0.25f);
    led_CollisionBoxAdd(led, id, ceil_hw);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
    const v3 floor_hw = V3(10.0f, 0.5f, 10.0f);
    led_CollisionBoxAdd(led, id, floor_hw);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
    led_CollisionCapsuleAdd(led, id, rope_radius, rope_half_height);

    const u32 rb_static = 0;
    const u32 rb_dynamic = 1;
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_ceil"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_floor"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_capsule"), rb_dynamic);

    led_RenderMeshAdd(led, Utf8Inline("rm_ceil"), Utf8Inline("c_ceil"));
    led_RenderMeshAdd(led, Utf8Inline("rm_floor"), Utf8Inline("c_floor"));
    led_RenderMeshAdd(led, Utf8Inline("rm_capsule"), Utf8Inline("c_capsule"));

    const f32 density = 1.0f;
    const f32 restitution = 0.0f;
    led_ShapePrefabAdd(led, Utf8Inline("s_ceil"), Utf8Inline("c_ceil"), density, restitution, floor_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_floor"), Utf8Inline("c_floor"), density, restitution, floor_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_capsule"), Utf8Inline("c_capsule"), density, restitution, capsule_friction);

    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_ceil"), Utf8Inline("rm_ceil")); 
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_floor"), Utf8Inline("rm_floor")); 
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_capsule"), Utf8Inline("rm_capsule"));

    ds_Transform transform = ds_TransformIdentity();
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_ceil"), Utf8Inline("s_ceil"), Utf8Inline("l_s_ceilr"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_floor"), Utf8Inline("s_floor"), Utf8Inline("l_s_floor"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_capsule"), Utf8Inline("s_capsule"), Utf8Inline("l_s_capsule"), &transform);

    ds_Id rope_id[ROPE_COUNT + 1];
    ds_Transform rope_t[ROPE_COUNT + 1];

    v3 ceil_transform = V3(5.0f, 3.0f, 15.0f);
    v3 rope_base = V3(5.0f, 2.75f - ceil_hw.y, 15.0f);

	id = Utf8Format(sys_win->ui->mem_frame, "led_ceil");
    rope_id[0] = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, rope_id[0], led_ScenePosition(led, ceil_transform));
    led_NodeAttachRigidBodyPrefab(led, rope_id[0], Utf8Inline("rb_ceil"));
    led_NodeSetColor(led, rope_id[0], floor_color, 1.0f);
    rope_t[0] = ds_TransformIdentity();
    rope_t[0].position = V3Sub(ceil_transform, rope_base);

    struct ds_DistanceJointPrefab prefab;
    const f32 distance = 0.0f;
    ds_DistanceJointPrefab(&prefab, distance, ds_BreakageOptionNone());
	for (u32 i = 0; i < ROPE_COUNT; ++i)
	{	
		v3 translation;
		translation = rope_base;
		translation.y -= i*2.0f*(rope_half_height+rope_radius);
        rope_t[i+1] = ds_TransformIdentity();
        rope_t[i+1].position = V3(0.0f, rope_half_height+rope_radius, 0.0f);

		id = Utf8Format(sys_win->ui->mem_frame, "rope_%u", i);
        rope_id[i+1] = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, rope_id[i+1], led_ScenePosition(led, translation));
        led_NodeAttachRigidBodyPrefab(led, rope_id[i+1], Utf8Inline("rb_capsule"));
        led_NodeSetColor(led, rope_id[i+1], capsule_color, 1.0f);
        led_DistanceJointAdd(led, &prefab, rope_id[i], rope_t + i, rope_id[i+1], rope_t + i + 1, distance);
        rope_t[i+1].position = V3(0.0f, -rope_half_height+rope_radius, 0.0f);
	}

	v3 floor_translation = V3(0.0f, -ramp_width/2.0f - 1.0f, ramp_length / 2.0f -ramp_width/2.0f);
    for (u32 i = 0; i < floor_count; ++i)
    {
        const f32 fi = -(f32) floor_count/2.0f + i;
        for (u32 j = 0; j < floor_count; ++j)
        {
            const f32 fj = -(f32) floor_count/2.0f + j;
            const v3 floor_offset = V3(
                floor_translation.x + fi*floor_hw.x*2.0f,
                floor_translation.y,
                floor_translation.z + fj*floor_hw.z*2.0f);
		    id = Utf8Format(sys_win->ui->mem_frame, "led_floor_%u_%u", i, j);
            tagged_id = led_NodeAdd(led, id, Utf8Empty());
            led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, floor_offset));
            led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_floor"));
            led_NodeSetColor(led, tagged_id, floor_color, 1.0f);
        }
    }
}

/* joint showcase: one node with a rigid body prefab at a scene position */
static ds_Id led_JointSceneNodeAdd(struct led *led, const char *name, const u32 index, const v3 position, const utf8 prefab, const v4 color)
{
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    const utf8 id = Utf8Format(sys_win->ui->mem_frame, "%s_%u", name, index);
    const ds_Id node = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, node, led_ScenePosition(led, position));
    led_NodeAttachRigidBodyPrefab(led, node, prefab);
    led_NodeSetColor(led, node, color, 1.0f);
    return node;
}

/* joint showcase: a joint between two nodes with the frames at their centres of mass */
static void led_JointSceneJointAdd(struct led *led, const struct ds_DistanceJointPrefab *prefab, const ds_Id node0, const ds_Id node1)
{
    const ds_Transform frame = ds_TransformIdentity();
    led_DistanceJointAdd(led, prefab, node0, &frame, node1, &frame, prefab->target);
}

/* joint showcase: a joint from a point on the ceiling bar (bar node, bar-local anchor) to a node's centre of mass */
static void led_JointSceneHangAdd(struct led *led, const struct ds_DistanceJointPrefab *prefab, const ds_Id bar, const v3 bar_anchor, const ds_Id node)
{
    ds_Transform frame_bar = ds_TransformIdentity();
    frame_bar.position = bar_anchor;
    const ds_Transform frame_node = ds_TransformIdentity();
    led_DistanceJointAdd(led, prefab, bar, &frame_bar, node, &frame_node, prefab->target);
}

/*
 * Joint showcase: every distance joint configuration several times, hanging from a static bar (y = 8) over a
 * floor (top at y = 0). Row 1 (z = -5): rigid joints (purple) and ranges (green). Row 2 (z = -10): springs
 * (orange) and range springs (blue). The camera starts at (5, 5, 5) looking at the origin.
 */
void led_JointSetup(struct led *led)
{
    ds_CPoolFlush(led->joint_pool);

	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    utf8 id;

    const f32 box_hw = 0.25f;
    /* the links weigh about as much as a box: a chain carrying a much heavier body stretches under PGS */
    const f32 capsule_radius = 0.2f;
    const f32 capsule_half_height = 0.2f;
    const f32 link_length = 2.0f * (capsule_half_height + capsule_radius);
    const f32 link_gap = 0.05f;
    const f32 bar_y = 8.0f;
    const v3 bar_hw = V3(16.0f, 0.25f, 0.25f);
    const v3 floor_hw = V3(20.0f, 0.5f, 10.0f);
    const f32 row1_z = -5.0f;
    const f32 row2_z = -10.0f;
    const f32 row3_z = -15.0f;

	const f32 alpha = 0.6f;
	const v4 static_color = V4(0.8f, 0.6f, 0.6f, alpha);
	const v4 rigid_color = V4(154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f, alpha);
	const v4 range_color = V4(54.0f/256.0f, 183.0f/256.0f, 122.0f/256.0f, alpha);
	const v4 spring_color = V4(254.0f/256.0f, 181.0f/256.0f, 82.0f/256.0f, alpha);
	const v4 range_spring_color = V4(0.1f, 0.4f, 0.8f, alpha);
	const v4 breakable_color = V4(0.9f, 0.35f, 0.3f, alpha);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_bar");
    led_CollisionBoxAdd(led, id, bar_hw);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
    led_CollisionBoxAdd(led, id, floor_hw);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_post");
    led_CollisionBoxAdd(led, id, V3(0.25f, 1.5f, 0.25f));
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
    led_CollisionBoxAdd(led, id, V3(box_hw, box_hw, box_hw));
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
    led_CollisionCapsuleAdd(led, id, capsule_radius, capsule_half_height);

    const u32 rb_static = 0;
    const u32 rb_dynamic = 1;
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_bar"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_floor"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_post"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_box"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_capsule"), rb_dynamic);

    led_RenderMeshAdd(led, Utf8Inline("rm_bar"), Utf8Inline("c_bar"));
    led_RenderMeshAdd(led, Utf8Inline("rm_floor"), Utf8Inline("c_floor"));
    led_RenderMeshAdd(led, Utf8Inline("rm_post"), Utf8Inline("c_post"));
    led_RenderMeshAdd(led, Utf8Inline("rm_box"), Utf8Inline("c_box"));
    led_RenderMeshAdd(led, Utf8Inline("rm_capsule"), Utf8Inline("c_capsule"));

    const f32 density = 1.0f;
    const f32 restitution = 0.0f;
    const f32 friction = 0.5f;
    led_ShapePrefabAdd(led, Utf8Inline("s_bar"), Utf8Inline("c_bar"), density, restitution, friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_floor"), Utf8Inline("c_floor"), density, restitution, friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_post"), Utf8Inline("c_post"), density, restitution, friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_box"), Utf8Inline("c_box"), density, restitution, friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_capsule"), Utf8Inline("c_capsule"), density, restitution, friction);

    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_bar"), Utf8Inline("rm_bar"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_floor"), Utf8Inline("rm_floor"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_post"), Utf8Inline("rm_post"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_box"), Utf8Inline("rm_box"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_capsule"), Utf8Inline("rm_capsule"));

    const ds_Transform identity = ds_TransformIdentity();
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_bar"), Utf8Inline("s_bar"), Utf8Inline("l_s_bar"), &identity);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_floor"), Utf8Inline("s_floor"), Utf8Inline("l_s_floor"), &identity);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_post"), Utf8Inline("s_post"), Utf8Inline("l_s_post"), &identity);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_box"), Utf8Inline("s_box"), Utf8Inline("l_s_box"), &identity);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_capsule"), Utf8Inline("s_capsule"), Utf8Inline("l_s_capsule"), &identity);

    /* statics: the floor and one bar per row; a hang point on a bar is (x, -bar_hw.y, 0) in bar space */
    led_JointSceneNodeAdd(led, "floor", 0, V3(0.0f, -floor_hw.y, 0.5f * (row1_z + row3_z)), Utf8Inline("rb_floor"), static_color);
    const ds_Id bar1 = led_JointSceneNodeAdd(led, "bar", 1, V3(0.0f, bar_y, row1_z), Utf8Inline("rb_bar"), static_color);
    const ds_Id bar2 = led_JointSceneNodeAdd(led, "bar", 2, V3(0.0f, bar_y, row2_z), Utf8Inline("rb_bar"), static_color);
    const ds_Id bar3 = led_JointSceneNodeAdd(led, "bar", 3, V3(0.0f, bar_y, row3_z), Utf8Inline("rb_bar"), static_color);
    const f32 hang_y = bar_y - bar_hw.y;
    struct ds_DistanceJointPrefab prefab;
    u32 n = 0;

    /* --- row 1: rigid --- */

    /* pendulum: length 2, released 30 degrees off the vertical */
    {
        const f32 x = -14.0f;
        const f32 L = 2.0f;
        const f32 angle = 30.0f * F32_PI / 180.0f;
        const ds_Id bob = led_JointSceneNodeAdd(led, "pendulum", n++, V3(x + L * F32Sin(angle), hang_y - L * F32Cos(angle), row1_z), Utf8Inline("rb_box"), rigid_color);
        ds_DistanceJointPrefab(&prefab, L, ds_BreakageOptionNone());
        led_JointSceneHangAdd(led, &prefab, bar1, V3(x, -bar_hw.y, 0.0f), bob);
    }

    /* rope: 6 capsule links joined end to end with a weight hung 1 m to the side of the last link */
    {
        const f32 x = -11.0f;
        const u32 link_count = 6;
        ds_Id prev = bar1;
        ds_Transform frame_prev = ds_TransformIdentity();
        frame_prev.position = V3(x, -bar_hw.y, 0.0f);
        ds_DistanceJointPrefab(&prefab, link_gap, ds_BreakageOptionNone());
        f32 top = hang_y - link_gap;
        for (u32 i = 0; i < link_count; ++i)
        {
            const ds_Id link = led_JointSceneNodeAdd(led, "rope", i, V3(x, top - 0.5f * link_length, row1_z), Utf8Inline("rb_capsule"), rigid_color);
            ds_Transform frame_link = ds_TransformIdentity();
            frame_link.position = V3(0.0f, 0.5f * link_length, 0.0f);
            led_DistanceJointAdd(led, &prefab, prev, &frame_prev, link, &frame_link, prefab.target);
            prev = link;
            frame_prev.position = V3(0.0f, -0.5f * link_length, 0.0f);
            top -= link_length + link_gap;
        }
        const f32 weight_distance = 1.0f;
        const ds_Id weight = led_JointSceneNodeAdd(led, "rope_weight", 0, V3(x + weight_distance, top, row1_z), Utf8Inline("rb_box"), rigid_color);
        ds_DistanceJointPrefab(&prefab, weight_distance, ds_BreakageOptionNone());
        const ds_Transform frame_weight = ds_TransformIdentity();
        led_DistanceJointAdd(led, &prefab, prev, &frame_prev, weight, &frame_weight, prefab.target);
    }

    /* double pendulum: the second link starts horizontal */
    {
        const f32 x = -8.0f;
        const f32 L = 1.5f;
        const ds_Id upper = led_JointSceneNodeAdd(led, "double_pendulum", 0, V3(x, hang_y - L, row1_z), Utf8Inline("rb_box"), rigid_color);
        const ds_Id lower = led_JointSceneNodeAdd(led, "double_pendulum", 1, V3(x + L, hang_y - L, row1_z), Utf8Inline("rb_box"), rigid_color);
        ds_DistanceJointPrefab(&prefab, L, ds_BreakageOptionNone());
        led_JointSceneHangAdd(led, &prefab, bar1, V3(x, -bar_hw.y, 0.0f), upper);
        led_JointSceneJointAdd(led, &prefab, upper, lower);
    }

    /* dumbbell: two boxes 1.5 m apart on a rigid link, dropped onto the floor */
    {
        const f32 x = -5.0f;
        const f32 L = 1.5f;
        const ds_Id a = led_JointSceneNodeAdd(led, "dumbbell", 0, V3(x - 0.5f * L, 5.0f, row1_z), Utf8Inline("rb_box"), rigid_color);
        const ds_Id b = led_JointSceneNodeAdd(led, "dumbbell", 1, V3(x + 0.5f * L, 4.0f, row1_z), Utf8Inline("rb_box"), rigid_color);
        ds_DistanceJointPrefab(&prefab, F32Sqrt(L * L + 1.0f), ds_BreakageOptionNone());
        led_JointSceneJointAdd(led, &prefab, a, b);
    }

    /* bridge: 7 boxes on rigid links between two posts, straight at first, sags into a chain curve */
    {
        const f32 x0 = 7.0f;
        const f32 span = 8.0f;
        const u32 box_count = 7;
        const f32 pitch = span / (f32) (box_count + 1);
        const f32 y = 3.0f;
        const ds_Id post0 = led_JointSceneNodeAdd(led, "post", 0, V3(x0, 1.5f, row1_z), Utf8Inline("rb_post"), static_color);
        const ds_Id post1 = led_JointSceneNodeAdd(led, "post", 1, V3(x0 + span, 1.5f, row1_z), Utf8Inline("rb_post"), static_color);
        ds_DistanceJointPrefab(&prefab, pitch, ds_BreakageOptionNone());
        ds_Id prev = post0;
        ds_Transform frame_prev = ds_TransformIdentity();
        frame_prev.position = V3(0.0f, y - 1.5f, 0.0f);
        const ds_Transform frame_box = ds_TransformIdentity();
        for (u32 i = 0; i < box_count; ++i)
        {
            const ds_Id box = led_JointSceneNodeAdd(led, "bridge", i, V3(x0 + pitch * (f32) (i + 1), y, row1_z), Utf8Inline("rb_box"), rigid_color);
            led_DistanceJointAdd(led, &prefab, prev, &frame_prev, box, &frame_box, prefab.target);
            prev = box;
            frame_prev = frame_box;
        }
        ds_Transform frame_post1 = ds_TransformIdentity();
        frame_post1.position = V3(0.0f, y - 1.5f, 0.0f);
        led_DistanceJointAdd(led, &prefab, prev, &frame_prev, post1, &frame_post1, prefab.target);
    }

    /* --- row 1: ranges --- */

    /* slack rope: starts at the low bound, falls to the high bound and stops there */
    {
        const f32 x = -2.0f;
        const intv range = Intv(1.0f, 3.0f);
        const ds_Id box = led_JointSceneNodeAdd(led, "slack", 0, V3(x, hang_y - range.low, row1_z), Utf8Inline("rb_box"), range_color);
        ds_DistanceJointPrefabRange(&prefab, range, ds_BreakageOptionNone());
        led_JointSceneHangAdd(led, &prefab, bar1, V3(x, -bar_hw.y, 0.0f), box);
    }

    /* slack chain: 5 boxes stacked close, each link [0.6, 1.2]; the chain unfolds link by link */
    {
        const f32 x = 1.0f;
        const u32 box_count = 5;
        const intv range = Intv(0.6f, 1.2f);
        ds_DistanceJointPrefabRange(&prefab, range, ds_BreakageOptionNone());
        ds_Id prev = led_JointSceneNodeAdd(led, "slack_chain", 0, V3(x, hang_y - range.low, row1_z), Utf8Inline("rb_box"), range_color);
        led_JointSceneHangAdd(led, &prefab, bar1, V3(x, -bar_hw.y, 0.0f), prev);
        for (u32 i = 1; i < box_count; ++i)
        {
            const ds_Id box = led_JointSceneNodeAdd(led, "slack_chain", i, V3(x, hang_y - range.low * (f32) (i + 1), row1_z), Utf8Inline("rb_box"), range_color);
            led_JointSceneJointAdd(led, &prefab, prev, box);
            prev = box;
        }
    }

    /* tether: two boxes dropped from different heights, free until the tether [0.5, 2.5] goes taut */
    {
        const f32 x = 4.0f;
        const ds_Id a = led_JointSceneNodeAdd(led, "tether", 0, V3(x - 0.5f, 6.0f, row1_z), Utf8Inline("rb_box"), range_color);
        const ds_Id b = led_JointSceneNodeAdd(led, "tether", 1, V3(x + 0.5f, 3.0f, row1_z), Utf8Inline("rb_box"), range_color);
        ds_DistanceJointPrefabRange(&prefab, Intv(0.5f, 2.5f), ds_BreakageOptionNone());
        led_JointSceneJointAdd(led, &prefab, a, b);
    }

    /* --- row 2: springs, all released at the target so gravity starts them --- */

    /* stiffness: 0.5, 1 and 2 Hz at damping ratio 0.1 */
    {
        const f32 hertz[3] = { 0.5f, 1.0f, 2.0f };
        const f32 L = 2.0f;
        for (u32 i = 0; i < 3; ++i)
        {
            const f32 x = -14.0f + 2.5f * (f32) i;
            const ds_Id box = led_JointSceneNodeAdd(led, "spring_hertz", i, V3(x, hang_y - L, row2_z), Utf8Inline("rb_box"), spring_color);
            ds_DistanceJointPrefabSpring(&prefab, L, hertz[i], 0.1f, ds_BreakageOptionNone());
            led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), box);
        }
    }

    /* damping: ratios 0 (never settles), 0.5, 1 (no overshoot) and 2 (creeps) at 1 Hz */
    {
        const f32 damping_ratio[4] = { 0.0f, 0.5f, 1.0f, 2.0f };
        const f32 L = 2.0f;
        for (u32 i = 0; i < 4; ++i)
        {
            const f32 x = -6.5f + 2.0f * (f32) i;
            const ds_Id box = led_JointSceneNodeAdd(led, "spring_damping", i, V3(x, hang_y - L, row2_z), Utf8Inline("rb_box"), spring_color);
            ds_DistanceJointPrefabSpring(&prefab, L, 1.0f, damping_ratio[i], ds_BreakageOptionNone());
            led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), box);
        }
    }

    /* spring chain: three boxes in series, 1 Hz, damping ratio 0.2 */
    {
        const f32 x = 2.0f;
        const f32 L = 1.5f;
        ds_DistanceJointPrefabSpring(&prefab, L, 1.0f, 0.2f, ds_BreakageOptionNone());
        ds_Id prev = led_JointSceneNodeAdd(led, "spring_chain", 0, V3(x, hang_y - L, row2_z), Utf8Inline("rb_box"), spring_color);
        led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), prev);
        for (u32 i = 1; i < 3; ++i)
        {
            const ds_Id box = led_JointSceneNodeAdd(led, "spring_chain", i, V3(x, hang_y - L * (f32) (i + 1), row2_z), Utf8Inline("rb_box"), spring_color);
            led_JointSceneJointAdd(led, &prefab, prev, box);
            prev = box;
        }
    }

    /* lift: a box on the floor, spring target 2 m below the bar at 0.3 Hz; the stretch exceeds its weight, so it rises */
    {
        const f32 x = 4.5f;
        const ds_Id box = led_JointSceneNodeAdd(led, "spring_lift", 0, V3(x, box_hw, row2_z), Utf8Inline("rb_box"), spring_color);
        ds_DistanceJointPrefabSpring(&prefab, 2.0f, 0.3f, 0.1f, ds_BreakageOptionNone());
        led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), box);
    }

    /* --- row 2: range springs --- */

    /* suspension with end stops [1.5, 2.5]: 0.5 Hz rests on the lower stop, 2 Hz rests inside and oscillates */
    {
        const f32 hertz[2] = { 0.5f, 2.0f };
        const intv range = Intv(1.5f, 2.5f);
        for (u32 i = 0; i < 2; ++i)
        {
            const f32 x = 7.0f + 2.5f * (f32) i;
            ds_DistanceJointPrefabRangeSpring(&prefab, range, hertz[i], 0.2f, ds_BreakageOptionNone());
            const ds_Id box = led_JointSceneNodeAdd(led, "range_spring", i, V3(x, hang_y - prefab.target, row2_z), Utf8Inline("rb_box"), range_spring_color);
            led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), box);
        }
    }

    /* undamped spring [1, 3] released at the low bound: it swings into the upper stop every period */
    {
        const f32 x = 12.0f;
        const intv range = Intv(1.0f, 3.0f);
        ds_DistanceJointPrefabRangeSpring(&prefab, range, 1.0f, 0.0f, ds_BreakageOptionNone());
        const ds_Id box = led_JointSceneNodeAdd(led, "range_spring_bounce", 0, V3(x, hang_y - range.low, row2_z), Utf8Inline("rb_box"), range_spring_color);
        led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), box);
    }

    /* --- row 3: breakable, thresholds in box weights; the grab pulls with up to LED_GRAB_FORCE_SCALE weights --- */

    /* hangers: the rod breaks at 2, 4, 8 and 16 weights, the last never; pull a box down or sideways */
    {
        const f32 L = 2.0f;
        const f32 box_weight = 8.0f * box_hw * box_hw * box_hw * density * V3Length(g_solver_config->gravity);
        const f32 strength[5] = { 2.0f, 4.0f, 8.0f, 16.0f, F32_INFINITY };
        for (u32 i = 0; i < 5; ++i)
        {
            const f32 x = -14.0f + 3.0f * (f32) i;
            const ds_Id box = led_JointSceneNodeAdd(led, "breakable", i, V3(x, hang_y - L, row3_z), Utf8Inline("rb_box"), breakable_color);
            ds_DistanceJointPrefab(&prefab, L, ds_BreakageOption(strength[i] * box_weight, F32_INFINITY));
            led_JointSceneHangAdd(led, &prefab, bar3, V3(x, -bar_hw.y, 0.0f), box);
        }
    }

    /* chain: four boxes, links 8, 5, 12 and 24 weights from the top (link k carries 4 - k boxes); pulled at the bottom the second link goes first */
    {
        const f32 x = 2.0f;
        const f32 L = 1.5f;
        const f32 box_weight = 8.0f * box_hw * box_hw * box_hw * density * V3Length(g_solver_config->gravity);
        const f32 strength[4] = { 8.0f, 5.0f, 12.0f, 24.0f };
        ds_DistanceJointPrefab(&prefab, L, ds_BreakageOption(strength[0] * box_weight, F32_INFINITY));
        ds_Id prev = led_JointSceneNodeAdd(led, "breakable_chain", 0, V3(x, hang_y - L, row3_z), Utf8Inline("rb_box"), breakable_color);
        led_JointSceneHangAdd(led, &prefab, bar3, V3(x, -bar_hw.y, 0.0f), prev);
        for (u32 i = 1; i < 4; ++i)
        {
            const ds_Id box = led_JointSceneNodeAdd(led, "breakable_chain", i, V3(x, hang_y - L * (f32) (i + 1), row3_z), Utf8Inline("rb_box"), breakable_color);
            ds_DistanceJointPrefab(&prefab, L, ds_BreakageOption(strength[i] * box_weight, F32_INFINITY));
            led_JointSceneJointAdd(led, &prefab, prev, box);
            prev = box;
        }
    }

    /* tug: a box slung between two posts on rods of 3 and 9 weights, hanging in a shallow V (each rod carries ~1.4 weights); pulled sideways the weak rod goes */
    {
        const f32 x = 10.0f;
        const f32 span = 6.0f;
        const f32 rod = 3.2f;
        const f32 box_weight = 8.0f * box_hw * box_hw * box_hw * density * V3Length(g_solver_config->gravity);
        const f32 y = 3.0f;
        const f32 sag = F32Sqrt(rod * rod - 0.25f * span * span);
        const ds_Id post0 = led_JointSceneNodeAdd(led, "breakable_post", 0, V3(x - 0.5f * span, 1.5f, row3_z), Utf8Inline("rb_post"), static_color);
        const ds_Id post1 = led_JointSceneNodeAdd(led, "breakable_post", 1, V3(x + 0.5f * span, 1.5f, row3_z), Utf8Inline("rb_post"), static_color);
        const ds_Id box = led_JointSceneNodeAdd(led, "breakable_tug", 0, V3(x, y - sag, row3_z), Utf8Inline("rb_box"), breakable_color);
        ds_Transform frame_post = ds_TransformIdentity();
        frame_post.position = V3(0.0f, y - 1.5f, 0.0f);
        const ds_Transform frame_box = ds_TransformIdentity();
        ds_DistanceJointPrefab(&prefab, rod, ds_BreakageOption(3.0f * box_weight, F32_INFINITY));
        led_DistanceJointAdd(led, &prefab, post0, &frame_post, box, &frame_box, prefab.target);
        ds_DistanceJointPrefab(&prefab, rod, ds_BreakageOption(9.0f * box_weight, F32_INFINITY));
        led_DistanceJointAdd(led, &prefab, box, &frame_box, post1, &frame_post, prefab.target);
    }

    /* range spring chain: three boxes, links [0.8, 1.5], 1.5 Hz, damping ratio 0.3 */
    {
        const f32 x = 14.5f;
        const intv range = Intv(0.8f, 1.5f);
        ds_DistanceJointPrefabRangeSpring(&prefab, range, 1.5f, 0.3f, ds_BreakageOptionNone());
        ds_Id prev = led_JointSceneNodeAdd(led, "range_spring_chain", 0, V3(x, hang_y - prefab.target, row2_z), Utf8Inline("rb_box"), range_spring_color);
        led_JointSceneHangAdd(led, &prefab, bar2, V3(x, -bar_hw.y, 0.0f), prev);
        for (u32 i = 1; i < 3; ++i)
        {
            const ds_Id box = led_JointSceneNodeAdd(led, "range_spring_chain", i, V3(x, hang_y - prefab.target * (f32) (i + 1), row2_z), Utf8Inline("rb_box"), range_spring_color);
            led_JointSceneJointAdd(led, &prefab, prev, box);
            prev = box;
        }
    }
}

enum led_MeshKind
{
    LED_MESH_WAVES,     /* a rolling field */
    LED_MESH_BOWL,      /* a paraboloid, 5 m deep at the rim */
    LED_MESH_STEPS      /* 2.5 m treads, 0.5 m risers, climbing along x */
};

/* a heightfield of n x n vertices over width x width metres, centred on the origin, y = f(x, z) by kind */
static struct mesh TriMeshHeightfield(struct arena *mem, const u32 n, const f32 width, const enum led_MeshKind kind)
{
	struct mesh mesh = { .v_count = n*n, .tri_count = 2*(n-1)*(n-1) };
	mesh.v = ArenaPush(mem, mesh.v_count*sizeof(v3));
	mesh.tri = ArenaPush(mem, mesh.tri_count*sizeof(v3u32));
	ds_Assert(mesh.v && mesh.tri);

	const f32 unit = width / (f32) (n - 1);
	for (u32 x = 0; x < n; ++x)
	{
		for (u32 z = 0; z < n; ++z)
		{
			const f32 px = -0.5f * width + x*unit;
			const f32 pz = -0.5f * width + z*unit;
			f32 py = 0.0f;
			switch (kind)
			{
				case LED_MESH_WAVES: { py = 0.6f * F32Sin(0.8f * px) * F32Cos(0.8f * pz); } break;
				case LED_MESH_BOWL:  { py = 0.05f * (px*px + pz*pz); } break;
				case LED_MESH_STEPS: { py = 0.5f * (f32) (u32) ((px + 0.5f * width) / 2.5f); } break;
			}
			mesh.v[x*n + z] = V3(px, py, pz);
		}
	}

	/* CCW about +y, as TriMeshPerlinNoise */
	for (u32 x = 0; x < n-1; ++x)
	{
		for (u32 z = 0; z < n-1; ++z)
		{
			mesh.tri[2*(x*(n-1) + z) + 0] = V3U32(x*n + z, x*n + z+1, (x+1)*n + z);
			mesh.tri[2*(x*(n-1) + z) + 1] = V3U32((x+1)*n + z, x*n + z+1, (x+1)*n + z+1);
		}
	}
	return mesh;
}

/* a tri mesh collision shape from a heightfield, its BVH built with 8 bins */
static void led_MeshSceneShapeAdd(struct led *led, const char *id, const enum led_MeshKind kind)
{
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
	struct mesh *mesh = ArenaPush(&led->mem_persistent, sizeof(struct mesh));
	*mesh = TriMeshHeightfield(&led->mem_persistent, 32, 20.0f, kind);
	MeshConstruct(&led->mem_persistent, mesh, 8);
	led_CollisionMeshAdd(led, Utf8Cstr(sys_win->ui->mem_frame, id), mesh);
}

/* tri mesh shapes: three heightfields side by side with boxes and spheres dropped on them */
void led_MeshSetup(struct led *led)
{
    ds_CPoolFlush(led->joint_pool);
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);

	const f32 alpha = 0.6f;
	const v4 static_color = V4(0.8f, 0.6f, 0.6f, alpha);
	const v4 dynamic_color = V4(154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f, alpha);

    led_CollisionBoxAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_box"), V3(0.5f, 0.5f, 0.5f));
    led_CollisionSphereAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_sphere"), 0.5f);
	led_MeshSceneShapeAdd(led, "c_waves", LED_MESH_WAVES);
	led_MeshSceneShapeAdd(led, "c_bowl", LED_MESH_BOWL);
	led_MeshSceneShapeAdd(led, "c_steps", LED_MESH_STEPS);

    const u32 rb_static = 0;
    const u32 rb_dynamic = 1;
    const char *shape[5] = { "box", "sphere", "waves", "bowl", "steps" };
    const u32 dynamic[5] = { rb_dynamic, rb_dynamic, rb_static, rb_static, rb_static };
    const ds_Transform identity = ds_TransformIdentity();
    for (u32 i = 0; i < 5; ++i)
    {
        const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_%s", shape[i]);
        const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_%s", shape[i]);
        const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_%s", shape[i]);
        const utf8 rb = Utf8Format(sys_win->ui->mem_frame, "rb_%s", shape[i]);
        const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_%s", shape[i]);
        led_RigidBodyPrefabAdd(led, rb, dynamic[i]);
        led_RenderMeshAdd(led, rm, c);
        led_ShapePrefabAdd(led, sp, c, 1.0f, 0.0f, 0.5f);
        led_ShapePrefabAttachRenderMesh(led, sp, rm);
        led_RigidBodyPrefabAttachShape(led, rb, sp, l, &identity);
    }

    /* heavy box and sphere: density 50, so their contact forces are 50 times the others' */
    const char *heavy[2] = { "box", "sphere" };
    for (u32 i = 0; i < 2; ++i)
    {
        const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_%s", heavy[i]);
        const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_%s", heavy[i]);
        const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_heavy_%s", heavy[i]);
        const utf8 rb = Utf8Format(sys_win->ui->mem_frame, "rb_heavy_%s", heavy[i]);
        const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_heavy_%s", heavy[i]);
        led_RigidBodyPrefabAdd(led, rb, rb_dynamic);
        led_ShapePrefabAdd(led, sp, c, 50.0f, 0.0f, 0.5f);
        led_ShapePrefabAttachRenderMesh(led, sp, rm);
        led_RigidBodyPrefabAttachShape(led, rb, sp, l, &identity);
    }

    const f32 mesh_x[3] = { -25.0f, 0.0f, 25.0f };
    const char *mesh_rb[3] = { "rb_waves", "rb_bowl", "rb_steps" };
    for (u32 m = 0; m < 3; ++m)
    {
        led_JointSceneNodeAdd(led, "mesh", m, V3(mesh_x[m], 0.0f, 0.0f), Utf8Cstr(sys_win->ui->mem_frame, mesh_rb[m]), static_color);
        for (u32 i = 0; i < 4; ++i)
        {
            for (u32 j = 0; j < 4; ++j)
            {
                const v3 position = V3(mesh_x[m] - 6.0f + 4.0f * i, 8.0f + 1.5f * (f32) ((i + j) & 1), -6.0f + 4.0f * j);
                const char *rb = ((i + j) & 1) ? "rb_sphere" : "rb_box";
                led_JointSceneNodeAdd(led, "drop", m*16 + i*4 + j, position, Utf8Cstr(sys_win->ui->mem_frame, rb), dynamic_color);
            }
        }

        /* a stack of three heavy boxes in one back corner, a heavy sphere in the other */
        for (u32 k = 0; k < 3; ++k)
        {
            led_JointSceneNodeAdd(led, "heavy", m*4 + k, V3(mesh_x[m] - 8.0f, 10.0f + 1.2f * (f32) k, 8.0f), Utf8Inline("rb_heavy_box"), dynamic_color);
        }
        led_JointSceneNodeAdd(led, "heavy", m*4 + 3, V3(mesh_x[m] + 8.0f, 10.0f, 8.0f), Utf8Inline("rb_heavy_sphere"), dynamic_color);
    }

    /* one mesh shape instanced four times, each turned and tilted differently, in a row behind */
    for (u32 k = 0; k < 4; ++k)
    {
        const v3 position = V3(-37.5f + 25.0f * k, 0.0f, 30.0f);
        const ds_Id node = led_JointSceneNodeAdd(led, "waves", k, position, Utf8Inline("rb_waves"), static_color);
        struct led_Node *led_node = led->node_hierarchy.pool.buf + ds_IdIndex(node);
        led_node->transform.rotation = QAxisAngle(V3(0.3f, 1.0f, 0.2f * (f32) k), F32_PI * (f32) k / 5.0f);     /* QAxisAngle normalizes */
        for (u32 i = 0; i < 4; ++i)
        {
            const v3 drop = V3(position.x - 3.0f + 2.0f * i, 9.0f, position.z + 2.0f * (f32) i - 3.0f);
            led_JointSceneNodeAdd(led, "waves_drop", k*4 + i, drop, Utf8Inline("rb_box"), dynamic_color);
        }
    }
}

/* TODO(Remove/Refactor) the game's mover capsule */
#define LED_MOVER_HALF_HEIGHT   0.5f
#define LED_MOVER_RADIUS        0.3f

/* dungeon: one static box with its own collision shape and prefabs, turned by yaw about +y */
static void led_DungeonBoxAdd(struct led *led, const u32 index, const v3 center, const v3 hw, const f32 yaw, const v4 color)
{
    struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_dungeon_%u", index);
    const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_dungeon_%u", index);
    const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_dungeon_%u", index);
    const utf8 rb = Utf8Format(sys_win->ui->mem_frame, "rb_dungeon_%u", index);
    const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_dungeon_%u", index);
    const ds_Transform identity = ds_TransformIdentity();
    led_CollisionBoxAdd(led, c, hw);
    led_RigidBodyPrefabAdd(led, rb, 0);
    led_RenderMeshAdd(led, rm, c);
    led_ShapePrefabAdd(led, sp, c, 1.0f, 0.0f, 0.5f);
    led_ShapePrefabAttachRenderMesh(led, sp, rm);
    led_RigidBodyPrefabAttachShape(led, rb, sp, l, &identity);
    const ds_Id node = led_JointSceneNodeAdd(led, "dungeon", index, center, rb, color);
    struct led_Node *led_node = led->node_hierarchy.pool.buf + ds_IdIndex(node);
    led_node->transform.rotation = QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), yaw);
}

/*
 * Mover test map, floor top at y = 0: a 60 x 60 m hall (walls 12 m, open top) with a 20 degree ramp and stairs
 * (0.2 m risers) up to a 1.6 m platform, a 30 degree V corner, and boxes, spheres and capsules to push. Two ways
 * up to a gallery along the north wall (9 m), whose parapet has a gap to push its boxes and sphere through: in the
 * north-west corner a spiral ramp turning twice around a pillar, in the north-east a rocky ridge turning twice
 * around a column, a gap away from it; each ends at a bridge onto the gallery. Static shapes are all convex.
 */
void led_DungeonSetup(struct led *led)
{
    ds_CPoolFlush(led->joint_pool);
    struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);

    const f32 alpha = 0.6f;
    const v4 static_color = V4(0.8f, 0.6f, 0.6f, alpha);
    const v4 spiral_color = V4(165.0f/256.0f, 172.0f/256.0f, 243.0f/256.0f, alpha);
    const v4 box_color = V4(154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f, alpha);
    const v4 big_box_color = V4(54.0f/256.0f, 183.0f/256.0f, 122.0f/256.0f, alpha);
    const v4 sphere_color = V4(254.0f/256.0f, 181.0f/256.0f, 82.0f/256.0f, alpha);
    const v4 capsule_color = V4(0.1f, 0.4f, 0.8f, alpha);
    const v4 rock_color = V4(0.55f, 0.5f, 0.45f, alpha);
    const ds_Transform identity = ds_TransformIdentity();

    /*
     * floor, walls, platform, gallery (x in [-23.1, 30], z in [-30, -26], top 9), its parapet (open at the bridges
     * and at x in [4, 6]), the spiral's bridge and the ridge's bridge
     */
    const u32 hall_count = 12;
    const v3 hall_center[12] =
    {
        V3(0.0f, -0.5f, 0.0f),
        V3(0.0f, 6.0f, -30.25f),
        V3(0.0f, 6.0f, 30.25f),
        V3(-30.25f, 6.0f, 0.0f),
        V3(30.25f, 6.0f, 0.0f),
        V3(8.0f, 0.8f, 0.0f),
        V3(3.45f, 8.75f, -28.0f),
        V3(-7.5f, 9.5f, -26.1f),
        V3(14.9f, 9.5f, -26.1f),
        V3(28.3f, 9.5f, -26.1f),
        V3(-21.25f, 8.75f, -25.5f),
        V3(25.2f, 8.75f, -24.25f),
    };
    const v3 hall_hw[12] =
    {
        V3(32.0f, 0.5f, 32.0f),
        V3(30.5f, 6.0f, 0.25f),
        V3(30.5f, 6.0f, 0.25f),
        V3(0.25f, 6.0f, 30.5f),
        V3(0.25f, 6.0f, 30.5f),
        V3(1.5f, 0.8f, 1.5f),
        V3(26.55f, 0.25f, 2.0f),
        V3(11.5f, 0.5f, 0.1f),
        V3(8.9f, 0.5f, 0.1f),
        V3(1.7f, 0.5f, 0.1f),
        V3(1.85f, 0.25f, 1.0f),
        V3(1.0f, 0.25f, 2.25f),
    };
    for (u32 i = 0; i < hall_count; ++i)
    {
        led_DungeonBoxAdd(led, i, hall_center[i], hall_hw[i], 0.0f, static_color);
    }

    /* seven steps down from the platform's east side (x = 9.5), 0.4 m treads */
    for (u32 i = 0; i < 7; ++i)
    {
        const f32 height = 1.4f - 0.2f * (f32) i;
        led_DungeonBoxAdd(led, hall_count + i, V3(9.7f + 0.4f * (f32) i, 0.5f * height, 0.0f), V3(0.2f, 0.5f * height, 1.5f), 0.0f, static_color);
    }

    /* V corner: two walls from the apex, opening towards -x */
    const f32 v_half_angle = F32_PI / 12.0f;
    const v3 v_apex = V3(13.5f, 1.0f, 10.0f);
    const v3 v_hw = V3(3.0f, 1.0f, 0.1f);
    for (u32 i = 0; i < 2; ++i)
    {
        const f32 side = (i == 0) ? -1.0f : 1.0f;
        const v3 center = V3(v_apex.x - v_hw.x * F32Cos(v_half_angle), v_apex.y, v_apex.z + side * v_hw.x * F32Sin(v_half_angle));
        led_DungeonBoxAdd(led, hall_count + 7 + i, center, v_hw, side * v_half_angle, static_color);
    }

    /* the ramp: a wedge from x = 2.1 up to the platform's west side (x = 6.5) */
    const v3 ramp_v[6] =
    {
        V3(-2.2f, 0.0f, -1.5f),
        V3(-2.2f, 0.0f, 1.5f),
        V3(2.2f, 0.0f, -1.5f),
        V3(2.2f, 0.0f, 1.5f),
        V3(2.2f, 1.6f, -1.5f),
        V3(2.2f, 1.6f, 1.5f),
    };
    struct dcel ramp = DcelConvexHull(&led->mem_persistent, ramp_v, 6);
    led_CollisionDcelAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_ramp"), &ramp);

    /*
     * Spiral segment, in the frame of the spiral's axis: the slab and its outer railing between the angles 0 and
     * segment_angle about +y (the point at angle a and radius r is (r cos a, y, -r sin a)), rising segment_rise.
     * Segment k is this turned by k * segment_angle and raised by k * segment_rise.
     */
    const v3 spiral_center = V3(-24.5f, 0.0f, -24.5f);
    const u32 spiral_segments = 48;
    const f32 segment_angle = F32_PI / 12.0f;
    const f32 segment_rise = 0.1875f;
    const f32 spiral_inner = 1.4f;
    const f32 spiral_outer = 5.1f;
    const f32 slab_thickness = 0.3f;
    const f32 rail_inner = spiral_outer - 0.2f;
    const f32 rail_height = 1.0f;
    v3 slab_v[8];
    v3 rail_v[8];
    for (u32 e = 0; e < 2; ++e)
    {
        const f32 c = F32Cos(segment_angle * (f32) e);
        const f32 s = -F32Sin(segment_angle * (f32) e);
        const f32 top = segment_rise * (f32) e;
        const f32 bottom = top - slab_thickness;
        slab_v[4*e + 0] = V3(spiral_inner * c, top, spiral_inner * s);
        slab_v[4*e + 1] = V3(spiral_outer * c, top, spiral_outer * s);
        slab_v[4*e + 2] = V3(spiral_inner * c, bottom, spiral_inner * s);
        slab_v[4*e + 3] = V3(spiral_outer * c, bottom, spiral_outer * s);
        rail_v[4*e + 0] = V3(rail_inner * c, top + rail_height, rail_inner * s);
        rail_v[4*e + 1] = V3(spiral_outer * c, top + rail_height, spiral_outer * s);
        rail_v[4*e + 2] = V3(rail_inner * c, bottom, rail_inner * s);
        rail_v[4*e + 3] = V3(spiral_outer * c, bottom, spiral_outer * s);
    }
    struct dcel slab = DcelConvexHull(&led->mem_persistent, slab_v, 8);
    struct dcel rail = DcelConvexHull(&led->mem_persistent, rail_v, 8);
    led_CollisionDcelAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_spiral_slab"), &slab);
    led_CollisionDcelAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_spiral_rail"), &rail);

    led_CollisionCapsuleAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_pillar"), 1.5f, 6.0f);
    led_CollisionCapsuleAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_column"), 1.2f, 6.0f);
    led_CollisionBoxAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_box"), V3(0.5f, 0.5f, 0.5f));
    led_CollisionBoxAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_big_box"), V3(1.0f, 1.0f, 1.0f));
    led_CollisionSphereAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_sphere"), 0.5f);
    led_CollisionCapsuleAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_capsule"), 0.3f, 0.6f);
    led_CollisionSphereAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_heavy_sphere"), 0.75f);

    const char *shape[8] = { "ramp", "pillar", "column", "box", "big_box", "sphere", "capsule", "heavy_sphere" };
    const u32 dynamic[8] = { 0, 0, 0, 1, 1, 1, 1, 1 };
    const f32 density[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 10.0f };
    for (u32 i = 0; i < 8; ++i)
    {
        const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_%s", shape[i]);
        const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_%s", shape[i]);
        const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_%s", shape[i]);
        const utf8 rb = Utf8Format(sys_win->ui->mem_frame, "rb_%s", shape[i]);
        const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_%s", shape[i]);
        led_RigidBodyPrefabAdd(led, rb, dynamic[i]);
        led_RenderMeshAdd(led, rm, c);
        led_ShapePrefabAdd(led, sp, c, density[i], 0.0f, 0.5f);
        led_ShapePrefabAttachRenderMesh(led, sp, rm);
        led_RigidBodyPrefabAttachShape(led, rb, sp, l, &identity);
    }

    /* one static body per spiral segment: slab and railing */
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_spiral"), 0);
    const char *spiral_part[2] = { "slab", "rail" };
    for (u32 i = 0; i < 2; ++i)
    {
        const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_spiral_%s", spiral_part[i]);
        const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_spiral_%s", spiral_part[i]);
        const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_spiral_%s", spiral_part[i]);
        const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_spiral_%s", spiral_part[i]);
        led_RenderMeshAdd(led, rm, c);
        led_ShapePrefabAdd(led, sp, c, 1.0f, 0.0f, 0.5f);
        led_ShapePrefabAttachRenderMesh(led, sp, rm);
        led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_spiral"), sp, l, &identity);
    }

    led_JointSceneNodeAdd(led, "ramp", 0, V3(4.3f, 0.0f, 0.0f), Utf8Inline("rb_ramp"), static_color);
    led_JointSceneNodeAdd(led, "pillar", 0, V3(spiral_center.x, 5.0f, spiral_center.z), Utf8Inline("rb_pillar"), spiral_color);
    for (u32 k = 0; k < spiral_segments; ++k)
    {
        const v3 position = V3(spiral_center.x, spiral_center.y + segment_rise * (f32) k, spiral_center.z);
        const ds_Id node = led_JointSceneNodeAdd(led, "spiral", k, position, Utf8Inline("rb_spiral"), spiral_color);
        struct led_Node *led_node = led->node_hierarchy.pool.buf + ds_IdIndex(node);
        led_node->transform.rotation = QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), segment_angle * (f32) k);
    }

    /*
     * Ridge, in the frame of its column's axis (angles as the spiral's): edge k is the ridge's cross-section at the
     * angle k * ridge_angle, a triangle (top inner, top outer, bottom middle) jittered from a fixed seed. Segment k is the hull of edges k and k + 1, so neighbours share their edge. The
     * first and last edges are not jittered: the ridge starts on the floor and ends at its bridge.
     */
    const v3 ridge_center = V3(22.0f, 0.0f, -22.0f);
    const u32 ridge_segments = 32;
    const f32 ridge_angle = F32_PI / 8.0f;
    const f32 ridge_rise = 0.28125f;
    f32 ridge_inner[33];
    f32 ridge_outer[33];
    f32 ridge_top[33];
    f32 ridge_bottom[33];
    struct rng rng;
    RngLocalInit(&rng, 7);
    for (u32 k = 0; k <= ridge_segments; ++k)
    {
        const f32 amplitude = (k == 0 || k == ridge_segments) ? 0.0f : 1.0f;
        const f32 jitter_inner = (f32) RngLocalU64Range(&rng, 0, 1000) / 1000.0f - 0.5f;
        const f32 jitter_outer = (f32) RngLocalU64Range(&rng, 0, 1000) / 1000.0f - 0.5f;
        const f32 jitter_top = (f32) RngLocalU64Range(&rng, 0, 1000) / 1000.0f - 0.5f;
        const f32 jitter_depth = (f32) RngLocalU64Range(&rng, 0, 1000) / 1000.0f - 0.5f;
        ridge_inner[k] = 2.2f + 0.4f * amplitude * jitter_inner;
        ridge_outer[k] = 4.2f + 0.4f * amplitude * jitter_outer;
        ridge_top[k] = ridge_rise * (f32) k + 0.12f * amplitude * jitter_top;
        ridge_bottom[k] = ridge_top[k] - 0.7f - 0.4f * amplitude * jitter_depth;
    }

    led_JointSceneNodeAdd(led, "column", 0, V3(ridge_center.x, 6.0f, ridge_center.z), Utf8Inline("rb_column"), rock_color);
    for (u32 k = 0; k < ridge_segments; ++k)
    {
        v3 rock_v[6];
        for (u32 e = 0; e < 2; ++e)
        {
            const u32 edge = k + e;
            const f32 c = F32Cos(ridge_angle * (f32) edge);
            const f32 s = -F32Sin(ridge_angle * (f32) edge);
            const f32 middle = 0.5f * (ridge_inner[edge] + ridge_outer[edge]);
            rock_v[3*e + 0] = V3(ridge_inner[edge] * c, ridge_top[edge], ridge_inner[edge] * s);
            rock_v[3*e + 1] = V3(ridge_outer[edge] * c, ridge_top[edge], ridge_outer[edge] * s);
            rock_v[3*e + 2] = V3(middle * c, ridge_bottom[edge], middle * s);
        }

        struct dcel rock = DcelConvexHull(&led->mem_persistent, rock_v, 6);
        const utf8 c = Utf8Format(sys_win->ui->mem_frame, "c_ridge_%u", k);
        const utf8 rm = Utf8Format(sys_win->ui->mem_frame, "rm_ridge_%u", k);
        const utf8 sp = Utf8Format(sys_win->ui->mem_frame, "s_ridge_%u", k);
        const utf8 rb = Utf8Format(sys_win->ui->mem_frame, "rb_ridge_%u", k);
        const utf8 l = Utf8Format(sys_win->ui->mem_frame, "l_s_ridge_%u", k);
        led_CollisionDcelAdd(led, c, &rock);
        led_RigidBodyPrefabAdd(led, rb, 0);
        led_RenderMeshAdd(led, rm, c);
        led_ShapePrefabAdd(led, sp, c, 1.0f, 0.0f, 0.5f);
        led_ShapePrefabAttachRenderMesh(led, sp, rm);
        led_RigidBodyPrefabAttachShape(led, rb, sp, l, &identity);
        led_JointSceneNodeAdd(led, "ridge", k, ridge_center, rb, rock_color);
    }

    /* hall: a pyramid of ten boxes, big boxes along the west wall, a row of spheres, capsules lying along z */
    u32 n = 0;
    for (u32 layer = 0; layer < 4; ++layer)
    {
        for (u32 j = 0; j + layer < 4; ++j)
        {
            led_JointSceneNodeAdd(led, "box", n, V3(-2.0f + 0.5f * (f32) layer + (f32) j, 0.5f + (f32) layer, 4.0f), Utf8Inline("rb_box"), box_color);
            n += 1;
        }
    }

    for (u32 i = 0; i < 3; ++i)
    {
        led_JointSceneNodeAdd(led, "big_box", i, V3(-27.0f, 1.0f, 4.0f * (f32) i), Utf8Inline("rb_big_box"), big_box_color);
    }

    for (u32 i = 0; i < 5; ++i)
    {
        led_JointSceneNodeAdd(led, "sphere", i, V3(-3.0f + 2.0f * (f32) i, 0.5f, 7.0f), Utf8Inline("rb_sphere"), sphere_color);
        const ds_Id node = led_JointSceneNodeAdd(led, "capsule", i, V3(-10.0f + 2.0f * (f32) i, 0.3f, 11.0f), Utf8Inline("rb_capsule"), capsule_color);
        struct led_Node *led_node = led->node_hierarchy.pool.buf + ds_IdIndex(node);
        led_node->transform.rotation = QUnitAxisAngle(V3(1.0f, 0.0f, 0.0f), 0.5f * F32_PI);
    }

    /* gallery: a pyramid of six boxes and a heavy sphere */
    n = 0;
    for (u32 layer = 0; layer < 3; ++layer)
    {
        for (u32 j = 0; j + layer < 3; ++j)
        {
            led_JointSceneNodeAdd(led, "gallery_box", n, V3(9.0f + 0.5f * (f32) layer + (f32) j, 9.5f + (f32) layer, -28.0f), Utf8Inline("rb_box"), box_color);
            n += 1;
        }
    }
    led_JointSceneNodeAdd(led, "heavy_sphere", 0, V3(2.0f, 9.75f, -28.0f), Utf8Inline("rb_heavy_sphere"), sphere_color);

    /* the game's mover: spawned at the south end by led_EngineInit, drawn as a capsule proxy */
    led_CollisionCapsuleAdd(led, Utf8Cstr(sys_win->ui->mem_frame, "c_mover"), LED_MOVER_RADIUS, LED_MOVER_HALF_HEIGHT);
    led_RenderMeshAdd(led, Utf8Inline("rm_mover"), Utf8Inline("c_mover"));
    led->mover_spawn_enabled = 1;
    led->mover_spawn = led_ScenePosition(led, V3(0.0f, 1.0f, 12.0f));
    const struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
    const struct r_Proxy3d_config mover_config =
    {
        .ns_time = 1000 * host->sim.us_time,
        .parent = PROXY3D_ROOT,
        .position = led->mover_spawn,
        .rotation = QIdentity(),
        .linear_velocity = V3Zero(),
        .angular_velocity = V3Zero(),
        .color = V4(0.95f, 0.85f, 0.2f, alpha),
        .blend = 1.0f,
        .mesh = Utf8Inline("rm_mover"),
    };
    led->mover_proxy = r_Proxy3dAlloc(&mover_config);
}

void led_WallSmashSimulationSetup(struct led *led)
{
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    ds_CPoolFlush(led->joint_pool);
    utf8 id;
    ds_Id tagged_id;

#define dsphere_v_count 	30
    //const u32 floor_count = 10;
	//const u32 dsphere_count = 40;
	//const u32 capsule_count = 20;
	//const u32 tower1_count = 2;
	//const u32 tower2_count = 4;
	//const u32 tower1_box_count = 40;
	//const u32 tower2_box_count = 10;
    //const u32 multibox_count = 5;
    //const u32 incr_count = 10;
   
    const u32 floor_count = 10;
	const u32 dsphere_count = 0;
	//const u32 dsphere_count = 120;
	//const u32 capsule_count = 400;
	const u32 capsule_count = 0;
	const u32 tower1_count = 0;
	const u32 tower2_count = 0;
	const u32 tower1_box_count = 0;
	const u32 tower2_box_count = 0;
    const u32 multibox_count = 0;
	const u32 pyramid_layers = 30;
	const u32 pyramid_count = 1;
	//const u32 pyramid_layers = 0;
	//const u32 pyramid_count = 0;
    const u32 incr_count = 0;
	const u32 pyramid_size = pyramid_layers*(pyramid_layers+1) / 2;

	/* Setup rigid bodies */
	const f32 box_friction = 1.0f;
	const f32 ramp_friction = 0.1f;
	const f32 sphere_friction = 0.1f;
	const f32 floor_friction = 0.8f;
	const f32 capsule_friction = 0.1f;

	const f32 alpha1 = 0.7f;
	const f32 alpha2 = 0.5f;
	const v4 dsphere_color = V4(244.0f/256.0f, 0.1f, 0.4f, alpha2);
	const v4 tower1_color = V4(154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f, alpha1);
	const v4 tower2_color = V4(54.0f/256.0f, 183.0f/256.0f, 122.0f/256.0f, alpha2);
	const v4 pyramid_color = V4(254.0f/256.0f, 181.0f/256.0f, 82.0f/256.0f, alpha2);
	const v4 floor_color = V4(0.8f, 0.6f, 0.6f, alpha2);
	const v4 ramp_color = V4(165.0f/256.0f, 172.0f/256.0f, 243.0f/256.0f, alpha2);
	const v4 sphere_color = V4(0.2f, 0.9f, 0.5f, alpha1);
	const v4 capsule_color = V4(0.1f, 0.4f, 0.8f, alpha2);
	const v4 multibox_color = V4(0.2f, 0.3f, 0.6f, alpha2);
	const v4 multidsphere_color = V4(0.9f, 0.6f, 0.1f, alpha2);
	const v4 mesh_color = V4(0.5f, 0.7f, 0.7f, 0.7f);

	const f32 box_side = 1.0f;

	const f32 ramp_width = 10.0f;
	const f32 ramp_length = 60.0f;
	const f32 ramp_height = 34.0f;
	const u32 v_count = 6;
	v3 ramp_vertices[6] = 
	{
		V3(0.0f, ramp_height, -ramp_length),
		V3(ramp_width, ramp_height, -ramp_length),
		V3(0.0f, 0.0f, -ramp_length),
		V3(ramp_width, 0.0f, -ramp_length),
		V3(0.0f, 0.0f, 0.0f),
		V3(ramp_width, 0.0f, 0.0f),
	};

	v3 dsphere_vertices[dsphere_v_count];
	const f32 phi = F32_PI * (3.0f - F32Sqrt(5.0f));
	for (u32 i = 0; i < dsphere_v_count; ++i)
	{
		const f32 y = 1.0 - i*2.0f/(dsphere_v_count-1);
		dsphere_vertices[i] = V3(F32Cos(i*phi)*F32Sqrt(1 - y*y), y, F32Sin(i*phi)*F32Sqrt(1 - y*y));
	}

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
    const v3 floor_hw = V3(10.0f, 0.5f, 10.0f);
    led_CollisionBoxAdd(led, id, floor_hw);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
    led_CollisionCapsuleAdd(led, id, 0.5f, 1.0f);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
    //const v3 box_hw = V3(box_side / 2.0f, box_side, box_side / 2.0f);
    const v3 box_hw = V3(box_side / 2.0f, box_side / 2.0f, box_side / 2.0f);
    led_CollisionBoxAdd(led, id, box_hw);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_bar");
    led_CollisionBoxAdd(led, id, V3(6.0f, 0.25f, 0.25f));

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_sphere");
    led_CollisionSphereAdd(led, id, 2.0f);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_s_sphere");
    led_CollisionSphereAdd(led, id, 0.5f);

	struct dcel *c_ramp = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_ramp = DcelConvexHull(&sys_win->mem_persistent, ramp_vertices, 6);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_ramp");
    led_CollisionDcelAdd(led, id, c_ramp);

	struct dcel *c_dsphere = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_dsphere = DcelConvexHull(&sys_win->mem_persistent, dsphere_vertices, dsphere_v_count);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_dsphere");
    led_CollisionDcelAdd(led, id, c_dsphere);

	struct mesh *mesh = ArenaPush(&led->mem_persistent, sizeof(struct mesh));
	*mesh = TriMeshPerlinNoise(&led->mem_persistent, 64, 200.0f);
	f32 best_cost = F32_INFINITY;
	u32 best_bin_count = 8;
	for (u32 i = 8; i < 16; ++i)
	{
		ArenaPushRecord(&led->mem_persistent);
		MeshConstruct(&led->mem_persistent, mesh, i);
		const f32 cost = BVHCost(&mesh->bvh);
		if (cost < best_cost)
		{
			best_cost = cost;
			best_bin_count = i;
		}
		ArenaPopRecord(&led->mem_persistent);
	}
	MeshConstruct(&led->mem_persistent, mesh, best_bin_count);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_mesh");
    led_CollisionMeshAdd(led, id, mesh);

    const u32 rb_static = 0;
    const u32 rb_dynamic = 1;
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_mesh"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_floor"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_ramp"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_bar"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_box"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_multibox"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_capsule"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_sphere"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_s_sphere"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_dsphere"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_multidsphere"), rb_dynamic);

    led_RenderMeshAdd(led, Utf8Inline("rm_mesh"), Utf8Inline("c_mesh"));
    led_RenderMeshAdd(led, Utf8Inline("rm_floor"), Utf8Inline("c_floor"));
    led_RenderMeshAdd(led, Utf8Inline("rm_ramp"), Utf8Inline("c_ramp"));
    led_RenderMeshAdd(led, Utf8Inline("rm_bar"), Utf8Inline("c_bar"));
    led_RenderMeshAdd(led, Utf8Inline("rm_capsule"), Utf8Inline("c_capsule"));
    led_RenderMeshAdd(led, Utf8Inline("rm_box"), Utf8Inline("c_box"));
    led_RenderMeshAdd(led, Utf8Inline("rm_sphere"), Utf8Inline("c_sphere"));
    led_RenderMeshAdd(led, Utf8Inline("rm_s_sphere"), Utf8Inline("c_s_sphere"));
    led_RenderMeshAdd(led, Utf8Inline("rm_dsphere"), Utf8Inline("c_dsphere"));

    const f32 density = 1.0f;
    const f32 restitution = 0.0f;
    led_ShapePrefabAdd(led, Utf8Inline("s_mesh"), Utf8Inline("c_mesh"), density, restitution, floor_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_floor"), Utf8Inline("c_floor"), density, restitution, floor_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_box"), Utf8Inline("c_box"), density, restitution, box_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_capsule"), Utf8Inline("c_capsule"), density, restitution, capsule_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_sphere"), Utf8Inline("c_sphere"), 100.0f, restitution, sphere_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_s_sphere"), Utf8Inline("c_s_sphere"), 100.0f, restitution, sphere_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_dsphere"), Utf8Inline("c_dsphere"), density, restitution, box_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_ramp"), Utf8Inline("c_ramp"), density, restitution, ramp_friction);
    led_ShapePrefabAdd(led, Utf8Inline("s_bar"), Utf8Inline("c_bar"), density, restitution, floor_friction);

    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_mesh"), Utf8Inline("rm_mesh"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_floor"), Utf8Inline("rm_floor")); 
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_box"), Utf8Inline("rm_box"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_capsule"), Utf8Inline("rm_capsule"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_sphere"), Utf8Inline("rm_sphere"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_s_sphere"), Utf8Inline("rm_s_sphere"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_dsphere"), Utf8Inline("rm_dsphere"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_ramp"), Utf8Inline("rm_ramp"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_bar"), Utf8Inline("rm_bar"));

    ds_Transform transform = ds_TransformIdentity();
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_mesh"), Utf8Inline("s_mesh"), Utf8Inline("l_s_mesh"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_floor"), Utf8Inline("s_floor"), Utf8Inline("l_s_floor"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_box"), Utf8Inline("s_box"), Utf8Inline("l_s_box"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_capsule"), Utf8Inline("s_capsule"), Utf8Inline("l_s_capsule"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_sphere"), Utf8Inline("s_sphere"), Utf8Inline("l_s_sphere"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_s_sphere"), Utf8Inline("s_s_sphere"), Utf8Inline("l_s_s_sphere"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_dsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_ramp"), Utf8Inline("s_ramp"), Utf8Inline("l_s_ramp"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_bar"), Utf8Inline("s_bar"), Utf8Inline("l_s_bar"), &transform);

    transform.position = V3(0.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box0"), &transform);
    transform.position = V3(2.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box1"), &transform);
    transform.position = V3(0.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box2"), &transform);
    transform.position = V3(2.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box3"), &transform);
    transform.position = V3(0.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box4"), &transform);
    transform.position = V3(2.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box5"), &transform);
    transform.position = V3(0.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box6"), &transform);
    transform.position = V3(2.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box7"), &transform);

    transform.position = V3(0.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere0"), &transform);
    transform.position = V3(2.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere1"), &transform);
    transform.position = V3(0.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere2"), &transform);
    transform.position = V3(2.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere3"), &transform);
    transform.position = V3(0.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere4"), &transform);
    transform.position = V3(2.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere5"), &transform);
    transform.position = V3(0.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere6"), &transform);
    transform.position = V3(2.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multidsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere7"), &transform);

	v3 floor_translation = V3(0.0f, -ramp_width/2.0f - 1.0f, ramp_length / 2.0f -ramp_width/2.0f);
    //const v3 mesh_translation = V3(0.0f, -25.0f, 0.0f);
    
    const v3 mesh_translation = V3(0.0f, -25.0f, 0.0f);

	const v3 sphere_translation = V3(-0.5, 0.5f + ramp_height, -ramp_length);
	const v3 box_translation = V3(-0.5f, 0.0f, -0.5f);
	const v3 ramp_translation = V3(-5.0f, -5.0f, -15.0f);
	const v3 box_base_translation = V3(0.0f, floor_translation.y + 1.0f, floor_translation.z / 2.0f);
	const v3 dsphere_base_translation = V3(-15.0f, floor_translation.y + 1.0f, floor_translation.z / 2.0f + 20.0f);
    const v3 multibox_base_translation = V3(0.0f, floor_translation.y + 1.0f, floor_translation.z / 2.0f -10.0f);
    const v3 multidsphere_base_translation = V3(0.0f, floor_translation.y + 1.0f, floor_translation.z / 2.0f -14.0f);

    floor_translation.y -= 35.0f;

    id = Utf8Cstr(sys_win->ui->mem_frame, "led_mesh");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, mesh_translation));
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_mesh"));
    led_NodeSetColor(led, tagged_id, mesh_color, 1.0f);
    struct led_Node *led_mesh = led->node_hierarchy.pool.buf + ds_IdIndex(tagged_id);
	const v3 axis = V3(0.6f, 1.0f, 0.6f);	/* QAxisAngle normalizes */
	const f32 angle = F32_PI / 16.0f;
	led_mesh->transform.rotation = QAxisAngle(axis, angle);

    for (u32 i = 0; i < floor_count; ++i)
    {
        const f32 fi = -(f32) floor_count/2.0f + i;
        for (u32 j = 0; j < floor_count; ++j)
        {
            const f32 fj = -(f32) floor_count/2.0f + j;
            const v3 floor_offset = V3(
                floor_translation.x + fi*floor_hw.x*2.0f,
                floor_translation.y,
                floor_translation.z + fj*floor_hw.z*2.0f);
		    id = Utf8Format(sys_win->ui->mem_frame, "led_floor_%u_%u", i, j);
            tagged_id = led_NodeAdd(led, id, Utf8Empty());
            led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, floor_offset));
            led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_floor"));
            led_NodeSetColor(led, tagged_id, floor_color, 1.0f);
        }
    }
	
    id = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, ramp_translation));
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_ramp"));
    led_NodeSetColor(led, tagged_id, ramp_color, 1.0f);

    //id = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
    //tagged_id = led_NodeAdd(led, id, Utf8Empty());
    //led_NodeSetPosition(led, tagged_id, sphere_translation);
    //led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_sphere"));
    //led_NodeSetColor(led, tagged_id, sphere_color, 1.0f);

	for (u32 i = 0; i < multibox_count; ++i)
    {
        v3 translation;
		translation = multibox_base_translation;
		translation.x += 0.0f;
		translation.y += i*2.5f;
		translation.z += 0.0f;

		id = Utf8Format(sys_win->ui->mem_frame, "multibox_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_multibox"));
        led_NodeSetColor(led, tagged_id, multibox_color, 1.0f);
    }

	for (u32 i = 0; i < multibox_count; ++i)
    {
        v3 translation;
		translation = multidsphere_base_translation;
		translation.x += 0.0f;
		translation.y += i*4.0f;
		translation.z += 0.0f;

		id = Utf8Format(sys_win->ui->mem_frame, "multidsphere_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_multidsphere"));
        led_NodeSetColor(led, tagged_id, multidsphere_color, 1.0f);
    }

	for (u32 i = 0; i < capsule_count; ++i)
	{	
		v3 translation;
		translation = dsphere_base_translation;
		translation.x += (10.0f - 38.0f * (f32) i / capsule_count) * F32Cos(i * F32_PI*37.0f/197.0f);
		translation.y += 25.0f + (f32) i / 2.0f;
		translation.z += (10.0f - 38.0f * (f32) i / capsule_count) * F32Sin(i * F32_PI*37.0f/197.0f);

		id = Utf8Format(sys_win->ui->mem_frame, "capsule_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_capsule"));
        led_NodeSetColor(led, tagged_id, capsule_color, 1.0f);
	}

	for (u32 i = 0; i < dsphere_count; ++i)
	{	
		v3 translation;
		translation = dsphere_base_translation;
		translation.x += (10.0f - 38.0f * (f32) i / dsphere_count) * F32Cos(i * F32_PI*37.0f/197.0f);
		translation.y += 5.0f + (f32) i / 2.0f;
		translation.z += (10.0f - 38.0f * (f32) i / dsphere_count) * F32Sin(i * F32_PI*37.0f/197.0f);

		id = Utf8Format(sys_win->ui->mem_frame, "dsphere_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_dsphere"));
        led_NodeSetColor(led, tagged_id, dsphere_color, 1.0f);
	}

	for (u32 k = 0; k < pyramid_count; ++k)
	{
		for (u32 i = 0; i < pyramid_layers; ++i)
		{
			const f32 local_y = i * box_hw.y*4.0f;
			for (u32 j = 0; j < pyramid_layers-i; ++j)
			{
				const f32 local_x = j -(pyramid_layers-i-1) * box_side / 2.0f;
				v3 translation;
				translation = box_base_translation;
				translation.x += local_x;
				translation.y += local_y;
				translation.z += 10.0f * k;

				id = Utf8Format(sys_win->ui->mem_frame, "pyramid_%u_%u_%u", i, j, k);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                //led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_capsule"));
                //led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_s_sphere"));
                led_NodeSetColor(led, tagged_id, pyramid_color, 1.0f);
			}
		}
	}

	/*
	 * Joint stage: distance joints in the determinism run, boxes only so everything settles and sleeps. From
	 * a static bar 9 m over the floor: a rigid rope of 6 links with a weight below, a pendulum released at
	 * 30 degrees that swings down onto the floor, a spring with its weight dropped 1 m below rest, and a
	 * slack range rope whose weight drops onto the high bound just above the floor.
	 */
	const u32 joint_stage = 1;
	if (joint_stage)
	{
		const v4 joint_color = V4(154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f, alpha1);
		const f32 floor_top = floor_translation.y + floor_hw.y;
		const v3 bar_hw = V3(6.0f, 0.25f, 0.25f);
		const v3 bar_translation = V3(80.0f, floor_top + 9.0f, 100.0f);
		const f32 hang_y = bar_translation.y - bar_hw.y;
		struct ds_DistanceJointPrefab prefab;
		ds_Transform frame_bar = ds_TransformIdentity();
		ds_Transform frame_node = ds_TransformIdentity();

		id = Utf8Cstr(sys_win->ui->mem_frame, "joint_bar");
		const ds_Id bar = led_NodeAdd(led, id, Utf8Empty());
		led_NodeSetPosition(led, bar, led_ScenePosition(led, bar_translation));
		led_NodeAttachRigidBodyPrefab(led, bar, Utf8Inline("rb_bar"));
		led_NodeSetColor(led, bar, floor_color, 1.0f);

		/* rope: 6 links and a weight, 0.1 m rigid gaps, hanging straight */
		{
			const f32 x = bar_translation.x - 4.0f;
			const f32 gap = 0.1f;
			ds_DistanceJointPrefab(&prefab, gap, ds_BreakageOptionNone());
			ds_Id prev = bar;
			frame_bar.position = V3(-4.0f, -bar_hw.y, 0.0f);
			ds_Transform frame_prev = frame_bar;
			f32 top = hang_y - gap;
			for (u32 i = 0; i < 7; ++i)
			{
				id = Utf8Format(sys_win->ui->mem_frame, "joint_rope_%u", i);
				const ds_Id link = led_NodeAdd(led, id, Utf8Empty());
				led_NodeSetPosition(led, link, led_ScenePosition(led, V3(x, top - box_hw.y, bar_translation.z)));
				led_NodeAttachRigidBodyPrefab(led, link, Utf8Inline("rb_box"));
				led_NodeSetColor(led, link, joint_color, 1.0f);
				frame_node.position = V3(0.0f, box_hw.y, 0.0f);
				led_DistanceJointAdd(led, &prefab, prev, &frame_prev, link, &frame_node, prefab.target);
				prev = link;
				frame_prev = ds_TransformIdentity();
				frame_prev.position = V3(0.0f, -box_hw.y, 0.0f);
				top -= 2.0f * box_hw.y + gap;
			}
		}

		/* pendulum: a rod reaching the floor, released at 30 degrees; the weight swings down and slides to rest */
		{
			const f32 x = bar_translation.x - 1.0f;
			const f32 L = hang_y - (floor_top + box_hw.y);
			const f32 angle = 30.0f * F32_PI / 180.0f;
			id = Utf8Cstr(sys_win->ui->mem_frame, "joint_pendulum");
			const ds_Id bob = led_NodeAdd(led, id, Utf8Empty());
			led_NodeSetPosition(led, bob, led_ScenePosition(led, V3(x + L * F32Sin(angle), hang_y - L * F32Cos(angle), bar_translation.z)));
			led_NodeAttachRigidBodyPrefab(led, bob, Utf8Inline("rb_box"));
			led_NodeSetColor(led, bob, joint_color, 1.0f);
			ds_DistanceJointPrefab(&prefab, L, ds_BreakageOptionNone());
			frame_bar.position = V3(-1.0f, -bar_hw.y, 0.0f);
			frame_node = ds_TransformIdentity();
			led_DistanceJointAdd(led, &prefab, bar, &frame_bar, bob, &frame_node, prefab.target);
		}

		/* spring: rest length 2, 2 Hz critically damped, the weight starts 1 m below rest */
		{
			const f32 x = bar_translation.x + 1.0f;
			id = Utf8Cstr(sys_win->ui->mem_frame, "joint_spring");
			const ds_Id weight = led_NodeAdd(led, id, Utf8Empty());
			led_NodeSetPosition(led, weight, led_ScenePosition(led, V3(x, hang_y - 3.0f, bar_translation.z)));
			led_NodeAttachRigidBodyPrefab(led, weight, Utf8Inline("rb_box"));
			led_NodeSetColor(led, weight, joint_color, 1.0f);
			ds_DistanceJointPrefabSpring(&prefab, 2.0f, 2.0f, 1.0f, ds_BreakageOptionNone());
			frame_bar.position = V3(1.0f, -bar_hw.y, 0.0f);
			led_DistanceJointAdd(led, &prefab, bar, &frame_bar, weight, &frame_node, prefab.target);
		}

		/* slack range [1, 8]: the weight drops from 6 m and hangs on the high bound 0.25 m over the floor */
		{
			const f32 x = bar_translation.x + 4.0f;
			id = Utf8Cstr(sys_win->ui->mem_frame, "joint_range");
			const ds_Id weight = led_NodeAdd(led, id, Utf8Empty());
			led_NodeSetPosition(led, weight, led_ScenePosition(led, V3(x, hang_y - 6.0f, bar_translation.z)));
			led_NodeAttachRigidBodyPrefab(led, weight, Utf8Inline("rb_box"));
			led_NodeSetColor(led, weight, joint_color, 1.0f);
			ds_DistanceJointPrefabRange(&prefab, Intv(1.0f, 8.0f), ds_BreakageOptionNone());
			frame_bar.position = V3(4.0f, -bar_hw.y, 0.0f);
			led_DistanceJointAdd(led, &prefab, bar, &frame_bar, weight, &frame_node, prefab.target);
		}
	}

	for (u32 k = 0; k < tower1_count; ++k)
	{
		for (u32 j = 0; j < tower1_count; ++j)
		{
			for (u32 i = 0; i < tower1_box_count; ++i)
			{
				v3 translation;
				translation = box_base_translation;
				translation.z += 15.0f + 2.0f*k;
				translation.y += (f32) i * box_hw.y * 2.10f;
				translation.x += 15.0f + 2.0f*j;

				id = Utf8Format(sys_win->ui->mem_frame, "tower1_%u_%u_%u", i, j, k);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                led_NodeSetColor(led, tagged_id, tower1_color, 1.0f);
			}
		}
	}

	//for (u32 k = 0; k < tower2_count; ++k)
    //{
        for (u32 j = 0; j < incr_count; ++j)
        {
    		for (u32 i = 0; i <= j; ++i)
			{
				v3 translation;
				translation = box_base_translation;
				translation.z -= 15.0f + 2.0f*0;
				translation.y += (f32) i * box_hw.y * 2.10f;
				translation.x -= 15.0f + 1.5f*j;
			
				id = Utf8Format(sys_win->ui->mem_frame, "tower2_%u_%u_%u", i, j, 0);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, led_ScenePosition(led, translation));
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                led_NodeSetColor(led, tagged_id, tower2_color, 1.0f);
			}
        }
    //}
   
	//for (u32 k = 0; k < tower2_count; ++k)
	//{
	//	for (u32 j = 0; j < tower2_count; ++j)
	//	{
	//		for (u32 i = 0; i < tower2_box_count; ++i)
	//		{
	//			v3 translation;
	//			Vec3Copy(translation, box_base_translation);
	//			translation[2] += 15.0f + 2.0f*k;
	//			translation[1] += (f32) i * box_aabb.hw[1] * 2.10f;
	//			translation[0] -= 15.0f + 2.0f*j;
	//		
	//			id = Utf8Format(sys_win->ui->mem_frame, "tower2_%u_%u_%u", i, j, k);
    //            tagged_id = led_NodeAdd(led, id, Utf8Empty());
    //            led_NodeSetPosition(led, tagged_id, translation);
    //            led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
    //            led_NodeSetColor(led, tagged_id, tower2_color, 1.0f);
	//		}
	//	}
	//}
}

void led_Compile(struct led *led)
{
}

void led_Refresh(struct led *led)
{
    HII it; 
    HIIInit(it, led->node_hierarchy, LED_NODE_ROOT);
    HIIAdvance(it, led->node_hierarchy);
	while (it.at != LED_NODE_ROOT)
	{
		struct led_Node *node = led->node_hierarchy.pool.buf + it.at;
        if (node->flags & LED_BODY_PREFAB)
        {
            const struct ds_BodyPrefab *prefab = led->body_prefab_db.pool.buf + node->body_prefab;
            led_NodeAttachRigidBodyPrefab(led, node->tagged_id, prefab->id);
            HIISkip(it, led->node_hierarchy);
        }
            
        HIIAdvance(it, led->node_hierarchy);
	}
}

void led_Run(struct led *led)
{
	led->pending_engine_initalized = 1;
	led->pending_engine_running = 1;
	led->pending_engine_paused = 0;
}

void led_Pause(struct led *led)
{
	led->pending_engine_paused = 1;
	led->pending_engine_running = 0;


}

void led_Stop(struct led *led)
{
	led->pending_engine_initalized = 0;
	led->pending_engine_running = 0;
	led->pending_engine_paused = 0;
}

u32	cmd_led_compile;
u32	cmd_led_run;
u32	cmd_led_pause;
u32	cmd_led_stop;

void led_CompileCmd(void)
{
    led_Compile(g_editor);
}

void led_RunCmd(void)
{
    led_Run(g_editor);
}

void led_PauseCmd(void)
{
    led_Pause(g_editor);
}

void led_StopCmd(void)
{
    led_Stop(g_editor);
}

void led_CoreInitCommands(void)
{
	cmd_led_compile = CmdFunctionRegister(Utf8Inline("led_Compile"), 0, &led_CompileCmd).index;
	cmd_led_run = CmdFunctionRegister(Utf8Inline("led_Run"), 0, &led_RunCmd).index;
	cmd_led_pause = CmdFunctionRegister(Utf8Inline("led_Pause"), 0, &led_PauseCmd).index;
	cmd_led_stop = CmdFunctionRegister(Utf8Inline("led_Stop"), 0, &led_StopCmd).index;
}

/* index U32_MAX: a body without an editor node (the pipeline's mouse anchor), nothing to colour */
static void led_NodeColorProxies(struct led *led, const u32 index, const v4 color)
{
    if (index == U32_MAX)
    {
        return;
    }
    const struct led_Node *node = led->node_hierarchy.pool.buf + index;
    const struct led_Node *child = NULL;
    for (i32 i = node->hi_first; i != HI_NULL; i = child->hi_next)
    {
        child = led->node_hierarchy.pool.buf + i;
        if (child->flags & LED_SHAPE_PREFAB)
        {
            struct r_Proxy3d *proxy = r_Proxy3dAddress(child->proxy);
	        proxy->color = color;
        }
    }
}

static void led_NodeDrawProxies(struct led *led, const u32 index)
{
    const struct led_Node *node = led->node_hierarchy.pool.buf + index;
    const struct led_Node *child = NULL;
    for (i32 i = node->hi_first; i != HI_NULL; i = child->hi_next)
    {
        child = led->node_hierarchy.pool.buf + i;
        if (child->flags & LED_SHAPE_PREFAB)
        {
            struct r_Proxy3d *proxy = r_Proxy3dAddress(child->proxy);
            proxy->flags |= PROXY3D_DRAW;
        }
    }
}

static void led_NodeDontDrawProxies(struct led *led, const u32 index)
{
    const struct led_Node *node = led->node_hierarchy.pool.buf + index;
    const struct led_Node *child = NULL;
    for (i32 i = node->hi_first; i != HI_NULL; i = child->hi_next)
    {
        child = led->node_hierarchy.pool.buf + i;
        if (child->flags & LED_SHAPE_PREFAB)
        {
            struct r_Proxy3d *proxy = r_Proxy3dAddress(child->proxy);
            proxy->flags &= ~PROXY3D_DRAW;
        }
    }
}

static void led_ColorIsland(struct led *led, const ds_IslandId id, const v4 color)
{
	struct ds_Island *is = ds_IslandLookup(led->physics, id).address;
    if (!is)
    {
        return;
    }

	const struct ds_Body *body;
	for (u32 i = is->body_list.first; (i32) i != DLL_SENTINEL; i = body->island_body.next)
	{
		body = led->physics->body_pool.buf + i;
        led_NodeColorProxies(led, body->entity, color);
	}
}

static void led_EngineRun(struct led *led, const u32 physics_ticked)
{
	if (led->pending_body_color_mode != led->body_color_mode)
	{
		switch (led->pending_body_color_mode)
		{
			case RB_COLOR_MODE_BODY: 
			{ 
                for (u64 bi = 0; bi < led->physics->body_usage_set.block_count; ++bi)
                {
                    struct ds_BitBlock it = ds_BitBlockInit(led->physics->body_usage_set.bits[bi], bi);
                    while (ds_BitBlockHasNext(&it))
                    {
                        const struct ds_Body *body = led->physics->body_pool.buf + ds_BitBlockNext(&it);
                        if (body->entity == U32_MAX)
                        {
                            continue;
                        }
                        const struct led_Node *node = led->node_hierarchy.pool.buf + body->entity;
                        led_NodeColorProxies(led, body->entity, node->color);
                    }
                }
			} break;

			case RB_COLOR_MODE_COLLISION: 
			{ 
                v4 color;
                for (u64 bi = 0; bi < led->physics->body_usage_set.block_count; ++bi)
                {
                    struct ds_BitBlock it = ds_BitBlockInit(led->physics->body_usage_set.bits[bi], bi);
                    while (ds_BitBlockHasNext(&it))
                    {
				        const struct ds_Body *body = led->physics->body_pool.buf + ds_BitBlockNext(&it);
                        if (body->entity == U32_MAX)
                        {
                            continue;
                        }
                        const struct led_Node *node = led->node_hierarchy.pool.buf + body->entity;
					    if (ds_BodyDynamicCheck(body))
					    {
                            const struct ds_Island *island = led->physics->island_pool.buf + body->island;
					    	color = (island->contact_list.count)
					    		? led->collision_color
					    		: node->color;
					    }
					    else
					    {
					    	color = led->static_color;
					    }
					    led_NodeColorProxies(led, body->entity, color);
                    }
				}
			} break;

			case RB_COLOR_MODE_SLEEP: 
			{ 
                for (u64 bi = 0; bi < led->physics->body_usage_set.block_count; ++bi)
                {
                    struct ds_BitBlock it = ds_BitBlockInit(led->physics->body_usage_set.bits[bi], bi);
                    while (ds_BitBlockHasNext(&it))
                    {
				        const struct ds_Body *body = led->physics->body_pool.buf + ds_BitBlockNext(&it);
                        if (!ds_BodyDynamicCheck(body))
					    {						
                            led_NodeColorProxies(led, body->entity, led->static_color);
					    }
					    else
					    {
					    	(body->set < SOLVER_SET_SLEEPING_FIRST)
                                ? led_NodeColorProxies(led, body->entity,  led->awake_color)
                                : led_NodeColorProxies(led, body->entity,  led->sleep_color);
					    }
                    }
				}
			} break;

			case RB_COLOR_MODE_ISLAND: 
			{ 
                for (u64 bi = 0; bi < led->physics->body_usage_set.block_count; ++bi)
                {
                    struct ds_BitBlock it = ds_BitBlockInit(led->physics->body_usage_set.bits[bi], bi);
                    while (ds_BitBlockHasNext(&it))
                    {
				        const struct ds_Body *body = led->physics->body_pool.buf + ds_BitBlockNext(&it);
                        if (body->entity == U32_MAX)
                        {
                            continue;
                        }
                        const struct led_Node *node = led->node_hierarchy.pool.buf + body->entity;
					    if (ds_BodyStaticCheck(body))
					    {
                            led_NodeColorProxies(led, body->entity, led->static_color);
					    }
					    else
					    {
					    	const struct ds_Island *is = led->physics->island_pool.buf + body->island;
                            led_NodeColorProxies(led, body->entity, is->color);
					    }
                    }
                }
			} break;
		}
	}
	led->body_color_mode = led->pending_body_color_mode;

	while (led->physics->event_list.first != DLL_SENTINEL)
	{
        const u32 ei = led->physics->event_list.first;
	    struct ds_PhysicsEvent *event = led->physics->event_pool.buf + ei;
		switch (event->type)
		{
			case PHYSICS_EVENT_CONTACT_NEW:
			{
				if (led->body_color_mode == RB_COLOR_MODE_COLLISION)
				{
                    const struct ds_Contact *c = ds_ContactLookup(led->physics, event->contact).address;
                    if (c)
                    {
                        const struct ds_Body *body[2] =
                        {
		                    led->physics->body_pool.buf + c->body[0],
		                    led->physics->body_pool.buf + c->body[1],
                        };

					    if (ds_BodyDynamicCheck(body[0]))
					    {
                            led_NodeColorProxies(led, body[0]->entity, led->collision_color);
					    }
					    if (ds_BodyDynamicCheck(body[1]))
					    {
                            led_NodeColorProxies(led, body[1]->entity, led->collision_color);
					    }
                    }
				}
			} break;

			case PHYSICS_EVENT_CONTACT_REMOVED:
			{
				if (led->body_color_mode == RB_COLOR_MODE_COLLISION)
				{
					const struct ds_Body *body1 = ds_BodyLookup(led->physics, event->contact_removed_bodies[0]).address;
					const struct ds_Body *body2 = ds_BodyLookup(led->physics, event->contact_removed_bodies[1]).address;
                    if (body1 && body2)
                    {
                        const struct led_Node *node1 = led->node_hierarchy.pool.buf + body1->entity;
                        const struct led_Node *node2 = led->node_hierarchy.pool.buf + body2->entity;

					    if (ds_BodyDynamicCheck(body1))
					    {
                            const struct ds_Island *is = led->physics->island_pool.buf + body1->island;
					    	if (is->contact_list.count == 0)
					    	{
                                led_NodeColorProxies(led, body1->entity, node1->color);
					    	}
					    }
					    else
					    {
                            led_NodeColorProxies(led, body1->entity, led->static_color);
					    }
		
					    if (ds_BodyDynamicCheck(body2))
					    {
                            const struct ds_Island *is = led->physics->island_pool.buf + body2->island;
					    	if (is->contact_list.count == 0)
					    	{
                                led_NodeColorProxies(led, body2->entity, node2->color);
					    	}
					    }
					    else
					    {
                            led_NodeColorProxies(led, body2->entity, led->static_color);
					    }
                    }
				}
			} break;

			case PHYSICS_EVENT_ISLAND_NEW:
			{
				struct ds_Island *is = ds_IslandLookup(led->physics, event->island).address;
                if (is)
				{
					/* one draw per statement: a call's argument order is the compiler's */
					const f32 r = RngF32Normalized();
					const f32 g = RngF32Normalized();
					const f32 b = RngF32Normalized();
					is->color = V4(r, g, b, 0.7f);
					if (led->body_color_mode == RB_COLOR_MODE_ISLAND)
					{
						led_ColorIsland(led, event->island, is->color);
					}
					else if (led->body_color_mode == RB_COLOR_MODE_SLEEP)
					{
						led_ColorIsland(led, event->island, led->awake_color);
					}
				}
			} break;

			case PHYSICS_EVENT_ISLAND_AWAKE:
			{
				if (led->body_color_mode == RB_COLOR_MODE_SLEEP)
				{
					led_ColorIsland(led, event->island, led->awake_color);
				}
			} break;

			case PHYSICS_EVENT_ISLAND_ASLEEP:
			{
				if (led->body_color_mode == RB_COLOR_MODE_SLEEP)
				{
					led_ColorIsland(led, event->island, led->sleep_color);
				}

                struct ds_Island *is = ds_IslandLookup(led->physics, event->island).address;
                if (is)
                {
                    struct ds_SolverSet *set = led->physics->solver_set_pool.buf + is->set;
                    for (u32 i = 0; i < set->body_sim_pool.count; ++i)
                    {
                        const struct ds_BodySim *sim = set->body_sim_pool.buf + i;
	                	const struct ds_Body *body = led->physics->body_pool.buf + sim->body;
                        const struct led_Node *node = led->node_hierarchy.pool.buf + body->entity;

                        const v3 linear_velocity = V3Zero();
                        const v3 angular_velocity = V3Zero();
                        const u64 ns = led->physics->frames_completed*led->physics->ns_tick;

	                	r_Proxy3dLinearSpeculationSet(sim->world.position
	                			, sim->world.rotation
	                			, linear_velocity
	                			, angular_velocity
	                			, ns 
	                			, node->proxy);
                    }
                }
			} break;

			case PHYSICS_EVENT_ISLAND_EXPANDED:
            {
            } break;

			case PHYSICS_EVENT_BODY_NEW:
			{
			} break;

			case PHYSICS_EVENT_BODY_REMOVED:
			{
                led_NodeDontDrawProxies(led, event->entity);
			} break;

			case PHYSICS_EVENT_JOINT_BREAKING:
			{
				ds_JointRemove(led->physics, event->joint);
			} break;
		}

        DLLRemove(led->physics->event_list, led->physics->event_pool.buf, ei, node);
	}

    if (physics_ticked)
    {
        struct ds_SolverSet *active = led->physics->solver_set_pool.buf + SOLVER_SET_ACTIVE;
        for (u32 i = 0; i < active->body_sim_pool.count; ++i)
        {
            const struct ds_BodySim *sim = active->body_sim_pool.buf + i;
	    	const struct ds_Body *body = led->physics->body_pool.buf + sim->body;
            const struct ds_BodyCompute *compute = active->body_compute_pool.buf + body->sim;
            const struct led_Node *node = led->node_hierarchy.pool.buf + body->entity;
            const u64 ns = led->physics->frames_completed*led->physics->ns_tick;

	    	r_Proxy3dLinearSpeculationSet(sim->world.position
	    			, sim->world.rotation
	    			, compute->linear_velocity
	    			, compute->angular_velocity
	    			, ns
	    			, node->proxy);
            if (led->body_color_mode == RB_COLOR_MODE_ISLAND)
            {
                led_NodeColorProxies(led, body->entity, led->physics->island_pool.buf[body->island].color);
            }
        }

        const struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
        if (led->mover_spawn_enabled && host->sim.game.spawned)
        {
            const struct ds_GeometricMover *mover = &host->sim.game.mover;
            const u64 ns = led->physics->frames_completed*led->physics->ns_tick;
            r_Proxy3dLinearSpeculationSet(mover->position, QIdentity(), mover->velocity, V3Zero(), ns, led->mover_proxy);
        }
    }
        
	
	ds_PoolFlush(led->physics->event_pool);

    //ds_DynamicsPrintUsage(led->physics);
}

static void led_EngineFlush(struct led *led)
{
	ds_SimHostReset(&led->srv, led->sim);
	ds_DynamicsConfigPush(led->physics);
	led->grab.active = 0;
    const struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
    const u64 ns = 1000 * host->sim.us_time;

    HII it; 
    HIIInit(it, led->node_hierarchy, LED_NODE_ROOT);
    HIIAdvance(it, led->node_hierarchy);
	while (it.at != LED_NODE_ROOT)
	{
		struct led_Node *node = led->node_hierarchy.pool.buf + it.at;
        if (node->flags & LED_PROXY3D)
        {
            r_Proxy3dLinearSpeculationSet(node->transform.position
					, node->transform.rotation
					, V3Zero()
					, V3Zero()
					, ns
					, node->proxy);
		    struct r_Proxy3d *proxy = r_Proxy3dAddress(node->proxy);

            if (node->flags & LED_SHAPE_PREFAB)
            {
                proxy->flags |= PROXY3D_DRAW;
            }
		    proxy->color = node->color;
            proxy->blend = node->blend;
        }
        HIIAdvance(it, led->node_hierarchy);
	}

    if (led->mover_spawn_enabled)
    {
        r_Proxy3dLinearSpeculationSet(led->mover_spawn, QIdentity(), V3Zero(), V3Zero(), ns, led->mover_proxy);
    }
}

static void led_EngineInit(struct led *led)
{
	//TODO move this into engine flush
	ds_SimHostReset(&led->srv, led->sim);
	ds_DynamicsConfigPush(led->physics);

    /* the scene as the first frame's commands: bodies with their shapes, then the joints */
    ds_CPool(ds_SimCmd) cmds;
    ds_CPoolAlloc(NULL, cmds, 1024, GROWABLE);

    HII it; 
    HIIInit(it, led->node_hierarchy, LED_NODE_ROOT);
    HIIAdvance(it, led->node_hierarchy);
	while (it.at != LED_NODE_ROOT)
	{
		struct led_Node *node = led->node_hierarchy.pool.buf + it.at;
        if (node->flags & LED_BODY_PREFAB)
        {
            const struct ds_BodyPrefab *body_prefab = led->body_prefab_db.pool.buf + node->body_prefab;
            const struct ds_SimCmd body_add = { .type = SIM_CMD_BODY_ADD, .body_add = { .prefab = body_prefab->id, .t_world = node->transform, .entity = it.at } };
            ds_CPoolPushValue(cmds, body_add);
    
            //TODO mass properties should be calculated on AttachShape....
            struct ds_ShapePrefabInstance *instance = NULL;
            struct ds_ShapePrefab *shape_prefab = NULL;
            for (i32 j = body_prefab->shape_list.first; j != DLL_SENTINEL; j = instance->body_shape.next)
            {
                instance = led->shape_prefab_instance_pool.buf + j;
                shape_prefab = led->shape_prefab_db.pool.buf + instance->shape_prefab;
                const struct ds_SimCmd shape_add = { .type = SIM_CMD_SHAPE_ADD, .shape_add = { .prefab = shape_prefab->id, .t_local = instance->t_local, .entity = it.at } };
                ds_CPoolPushValue(cmds, shape_add);
            }
        }
        HIIAdvance(it, led->node_hierarchy);
	}

    for (u32 i = 0; i < led->joint_pool.count; ++i)
    {
        const struct led_Joint *joint = led->joint_pool.buf + i;
        const struct slot node0 = led_NodeLookup(led, joint->id[0]);
        const struct slot node1 = led_NodeLookup(led, joint->id[1]);
        if (!node0.address || !node1.address)
        {
		    LogString(T_LED, S_WARNING, "Failed to lookup joint attached node");
            continue;
        }

        const struct ds_DistanceJointPrefab *prefab = &joint->prefab;
        struct ds_SimCmd joint_add = { .type = SIM_CMD_JOINT_ADD };
        joint_add.joint_add = (struct ds_JointAddCmd)
        {
            .collide_connected = prefab->collide_connected,
            .target = prefab->target,
            .range = prefab->range,
            .enable_spring = prefab->enable_spring,
            .hertz = prefab->hertz,
            .damping_ratio = prefab->damping_ratio,
            .breakage = prefab->breakage,
            .entity = { node0.index, node1.index },
            .local_frame = { joint->local_frame[0], joint->local_frame[1] },
        };
        ds_CPoolPushValue(cmds, joint_add);
    }

    if (led->mover_spawn_enabled)
    {
        const struct ds_SimCmd spawn = { .type = SIM_CMD_MOVER_SPAWN, .mover_spawn = { .position = led->mover_spawn, .capsule = { .half_height = LED_MOVER_HALF_HEIGHT, .radius = LED_MOVER_RADIUS } } };
        ds_CPoolPushValue(cmds, spawn);
    }

    const struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
    if (host->record_mode != SIM_MODE_REPLAY)
    {
        ds_CPoolPushValue(led->srv.payload_pool, ds_SimCmdPayloadPushAll(&led->srv.frame, led->sim, &cmds));
    }
    ds_CPoolDealloc(cmds);
}

void led_Core(struct led *led)
{
	static u32 once = 1;
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
	if (once && sys_win)
	{
		once = 0;
#ifdef DS_TEST_DETERMINISM
		/* the determinism run's scene: every body sleeps, the run ends */
		led_WallSmashSimulationSetup(led);
#else
		led_DungeonSetup(led);
		//led_JointSetup(led);
		//led_MeshSetup(led);
		//led_WallSmashSimulationSetup(led);
		//led_RopeSetup(led);
#endif
        /* the camera starts at the same place relative to the scene */
        led->cam.position = V3F64Add(led->scene_origin, led->cam.position);
        led_Refresh(led);
        led_Run(led);
	}

	if (led->engine_initalized && !led->pending_engine_initalized)
	{
		led_EngineFlush(led);
        led_Refresh(led);
	}

	if (!led->engine_initalized && led->pending_engine_initalized)
	{
		led_EngineInit(led);
	}
	led->engine_initalized = led->pending_engine_initalized;
	led->engine_running = led->pending_engine_running;
	led->engine_paused = led->pending_engine_paused;

	/* the server's frame: the simulation's clock runs while the engine does, its commands always */
	struct ds_SimHost *host = ds_SimHostLookup(&led->srv, led->sim).address;
	host->speed = (led->engine_running) ? 1.0 : 0.0;
	const u64 physics_frames_before = led->physics->frames_completed;
	if (led->cmds.count)
	{
		ds_CPoolPushValue(led->srv.payload_pool, ds_SimCmdPayloadPushAll(&led->srv.frame, led->sim, &led->cmds));
		ds_CPoolFlush(led->cmds);
	}
	ds_ServerTick(&led->srv, led->ns_delta);
	ds_DynamicsConfigPush(led->physics);    /* the tick popped it; the editor's queries until the next tick read it */
	if (led->engine_running)
	{
		led->ns_engine_running += led->ns_delta;
		led_EngineRun(led, led->physics->frames_completed != physics_frames_before);
	}
}

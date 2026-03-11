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
		struct led_Node *node = hi_Address(&led->node_hierarchy, i);
		if (Utf8Equivalence(id, node->id))
		{
			slot.index = i;
			slot.address = node;
			break;
		}
	}

	return slot;
}

struct led_Node *led_NodeLookup(struct led *led, const ds_Id id)
{
    if (id == DS_ID_NULL)
    {
        return NULL;        
    }

    struct led_Node *node = hi_Address(&led->node_hierarchy, ds_IdIndex(id));
    return (node->tagged_id = id)
        ? node
        : NULL;
}

static ds_Id led_NodeInitalize(struct led_Node *node, const u32 node_index, const u64 flags)
{
    node->tagged_id = ((node->tagged_id & DS_ID_TAG_MASK) + DS_ID_TAG_INCREMENT) | node_index; 
	node->flags = flags;
	node->body_prefab = STRING_DATABASE_STUB_INDEX;
    node->shape_prefab = STRING_DATABASE_STUB_INDEX;
    node->proxy = PROXY3D_NULL;
    node->transform = ds_TransformIdentity();
    Vec4Set(node->color, 0.9f, 0.9f, 0.9f, 1.0f);

    return node->tagged_id;
}

static ds_Id led_NodeAnonymousAdd(struct led *led, const ds_Id parent_id)
{
    struct led_Node *node = led_NodeLookup(led, parent_id);
    ds_Assert(parent_id == DS_ID_NULL || node);

    u32 parent = HI_ROOT_STUB_INDEX;
    if (parent_id != DS_ID_NULL)
    {
        parent = ds_IdIndex(parent_id);
    }

    struct slot slot = hi_Add(&led->node_hierarchy, parent);
    return led_NodeInitalize(slot.address, slot.index, LED_ANONYMOUS);
}

ds_Id led_NodeAdd(struct led *led, const utf8 id, const utf8 parent_id)
{
    if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: id must not be empty");
        return DS_ID_NULL;
	} 

	if (led_NodeLookupId(led, id).address != NULL) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: node with given id already exist");
        return DS_ID_NULL;
	}

    u32 parent = HI_ROOT_STUB_INDEX;
    const struct slot slot_parent = led_NodeLookupId(led, parent_id);
    if (slot_parent.address)
    {
        parent = slot_parent.index;
    }
    else if (parent_id.len)
    {
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: parent with given id does not exist");
        return DS_ID_NULL;
    }

    struct slot slot = hi_Add(&led->node_hierarchy, parent);
    struct led_Node *node = slot.address;
	node->id = Utf8CopyBuffered(node->id_buf, LED_NODE_ID_SIZE, id);	
	if (!node->id.len)
	{
        hi_Remove(&led->frame, &led->node_hierarchy, slot.index);
		Log(T_LED, S_WARNING, "Failed to allocate led_node: id size must be <= %luB", LED_NODE_ID_SIZE);
        return DS_ID_NULL;
	} 
	const u32 hash = Utf8Hash(node->id);
	ds_HashMapAdd(&led->node_map, hash, slot.index);

    return led_NodeInitalize(node, slot.index, LED_FLAG_NONE);
}

static void led_NodeRemoveResources(const struct hi *hi, const u32 index, void *led_void)
{
    struct led *led = led_void;
    struct led_Node *node = hi_Address(hi, index);

	strdb_Dereference(&led->body_prefab_db, node->body_prefab);
    node->body_prefab = STRING_DATABASE_STUB_INDEX;

    strdb_Dereference(&led->shape_prefab_db, node->shape_prefab);
    node->shape_prefab = STRING_DATABASE_STUB_INDEX;

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
        hi_ApplyCustomFreeAndRemove(&led->frame, &led->node_hierarchy, slot.index, &led_NodeRemoveResources, led);
    }
}

void led_NodeRemove(struct led *led, const ds_Id id)
{
    struct led_Node *node = led_NodeLookup(led, id);
    if (node && node->tagged_id == id)
    {
        hi_ApplyCustomFreeAndRemove(&led->frame, &led->node_hierarchy, ds_IdIndex(id), &led_NodeRemoveResources, led);
    }
}

void led_NodeSetPositionId(struct led *led, const utf8 id, const vec3 position)
{
	struct slot slot = led_NodeLookupId(led, id);
	struct led_Node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %k, node not found.", &id);
	}
	else
	{
		Vec3Copy(node->transform.position, position);
	}
}

void led_NodeSetPosition(struct led *led, const ds_Id id, const vec3 position)
{
	struct led_Node *node = led_NodeLookup(led, id);
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %lu, node not found.", id);
	}
	else
	{
		Vec3Copy(node->transform.position, position);
	}
}

void led_NodeSetColor(struct led *led, const ds_Id id, const vec4 color, const f32 blend)
{
	struct led_Node *node = led_NodeLookup(led, id);
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set color of led node %lu, node not found.", id);
	}
	else
	{
		Vec4Copy(node->color, color);
        node->blend = blend;
	}
}

void led_NodeSetColorId(struct led *led, const utf8 id, const vec4 color, const f32 blend)
{
	struct slot slot = led_NodeLookupId(led, id);
	struct led_Node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set color of led node %k, node not found.", &id);
	}
	else
	{
		Vec4Copy(node->color, color);
        node->blend = blend;
	}
}

static void led_NodeDetachRigidBodyPrefabInternal(struct led *led, struct led_Node *node)
{
    if (node->flags & LED_BODY_PREFAB)
    {
	    strdb_Dereference(&led->body_prefab_db, node->body_prefab);
        for (u32 i = node->hi_first; i != HI_NULL_INDEX; )
        {
            struct led_Node *child = hi_Address(&led->node_hierarchy, i);
            const u32 next = child->hi_next;
            if (child->flags & LED_SHAPE_PREFAB)
            {
                hi_ApplyCustomFreeAndRemove(&led->frame, &led->node_hierarchy, i, &led_NodeRemoveResources, led);
            }
            i = next;
        }

	    r_Proxy3dDealloc(&led->frame, node->proxy);

        node->body_prefab = STRING_DATABASE_STUB_INDEX;
        node->proxy = PROXY3D_NULL;
    }
}

static void led_NodeAttachRigidBodyPrefabInternal(struct led *led, struct led_Node *node, const utf8 prefab)
{
    struct slot slot = strdb_Reference(&led->body_prefab_db, prefab);
	if (slot.index == STRING_DATABASE_STUB_INDEX)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, prefab not found.", &prefab);
        return;
	}

    led_NodeDetachRigidBodyPrefabInternal(led, node); 

    struct r_Proxy3d_config config =
	{
		.parent = PROXY3D_ROOT,
		.linear_velocity = { 0.0f, 0.0f, 0.0f },
		.angular_velocity = { 0.0f, 0.0f, 0.0f },
		.mesh = Utf8Inline(""),
		.ns_time = led->ns,
        .blend = node->blend,
	};
    Vec4Copy(config.color, node->color);
	Vec3Copy(config.position, node->transform.position);
	QuatCopy(config.rotation, node->transform.rotation);

    node->flags |= LED_BODY_PREFAB;
	node->body_prefab = slot.index;
    node->proxy = r_Proxy3dAlloc(&config);

    struct r_Proxy3d *proxy = r_Proxy3dAddress(node->proxy);
    proxy->flags &= ~PROXY3D_DRAW;

    const struct ds_RigidBodyPrefab *body = slot.address;
    const struct ds_ShapePrefabInstance *instance = NULL;
    for (u32 i = body->shape_list.first; i != DLL_NULL; i = instance->dll_next)
    {
        instance = ds_PoolAddress(&led->shape_prefab_instance_pool, i);
        const struct ds_ShapePrefab *shape = strdb_Address(&led->shape_prefab_db, instance->shape_prefab);
        const struct r_Mesh *render_mesh = strdb_Address(&led->render_mesh_db, shape->render_mesh);

        const ds_Id child_id = led_NodeAnonymousAdd(led, node->tagged_id);
        struct led_Node *child = led_NodeLookup(led, child_id);

        child->transform = instance->t_local;
        child->flags |= LED_SHAPE_PREFAB;
        child->shape_prefab = strdb_Reference(&led->shape_prefab_db, shape->id).index;

		config.mesh = render_mesh->id;
        config.parent = node->proxy;
		Vec3Copy(config.position, child->transform.position);
		QuatCopy(config.rotation, child->transform.rotation);
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
        led_NodeAttachRigidBodyPrefabInternal(led, slot.address, prefab);
	}
}

void led_NodeAttachRigidBodyPrefab(struct led *led, const ds_Id id, const utf8 prefab)
{
	struct led_Node *node = led_NodeLookup(led, id);
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %lu, node not found.", id);
	}
	else
	{
        led_NodeAttachRigidBodyPrefabInternal(led, node, prefab);
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
	struct led_Node *node = led_NodeLookup(led, id);
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to detach body prefab from led node %lu, node not found.", id);
	}
	else
	{
        led_NodeDetachRigidBodyPrefabInternal(led, node);
	}
}

//static void led_NodeColorProxies(struct led *led, const u32 index, const vec4 color)
//{
//    const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, index);
//    const struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//    struct ds_ShapePrefabInstance *instance = NULL;
//    for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//    {
//        instance = ds_PoolAddress(&led->shape_prefab_instance_pool, j);
//	    struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->proxy);
//	    Vec4Copy(proxy->color, color);
//    }
//}
//
//static void led_NodeDrawProxies(struct led *led, const u32 index)
//{
//    const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, index);
//    const struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//    struct ds_ShapePrefabInstance *instance = NULL;
//    for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//    {
//        instance = ds_PoolAddress(&led->shape_prefab_instance_pool, j);
//	    struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->proxy);
//        proxy->flags |= PROXY3D_DRAW;
//    }
//}
//
//static void led_NodeDontDrawProxies(struct led *led, const u32 index)
//{
//    const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, index);
//    const struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//    struct ds_ShapePrefabInstance *instance = NULL;
//    for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//    {
//        instance = ds_PoolAddress(&led->shape_prefab_instance_pool, j);
//	    struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->proxy);
//        proxy->flags &= ~PROXY3D_DRAW;
//    }
//}

static struct slot led_CollisionShapeAdd(struct led *led, const struct c_Shape *shape)
{
	struct slot slot = empty_slot;
	if (!shape->id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision shape: shape->id must not be empty");
	} 
	else if (strdb_Lookup(&led->cs_db, shape->id).index != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision shape: shape with given id already exist");
	}
	else
	{ 
		u8 buf[C_SHAPE_ID_SIZE];
		const utf8 copy = Utf8CopyBuffered(buf, C_SHAPE_ID_SIZE, shape->id);	
		if (!copy.len)
		{
			Log(T_LED, S_WARNING, "Failed to allocate collision shape: shape->id size must be <= %luB", C_SHAPE_ID_SIZE);
		}
		else
		{
			slot = strdb_AddAndAlias(&led->cs_db, copy);
			struct c_Shape *new_shape = slot.address;
            new_shape->id = Utf8CopyBuffered(new_shape->id_buf, C_SHAPE_ID_SIZE, copy);
			new_shape->type = shape->type;
			switch (shape->type)
			{
				case C_SHAPE_SPHERE: 
				{ 
					new_shape->sphere = shape->sphere; 
				} break;

				case C_SHAPE_CAPSULE: 
				{ 
					new_shape->capsule = shape->capsule; 
				} break;

				case C_SHAPE_CONVEX_HULL: 
				{ 
					new_shape->hull = shape->hull; 
				} break;

				case C_SHAPE_TRI_MESH: 
				{ 
					new_shape->mesh_bvh = shape->mesh_bvh; 
				} break;
			};

			if (shape->type != C_SHAPE_TRI_MESH)
			{
				c_ShapeUpdateMassProperties(new_shape);
			}
		}
	}

	return slot;
}

struct slot led_CollisionShapeDefaultAdd(struct led *led, const utf8 id)
{
    const vec3 hw = { 0.5f, 0.5f, 0.5f };
	return led_CollisionBoxAdd(led, id, hw);	
}

struct slot led_CollisionBoxAdd(struct led *led, const utf8 id, const vec3 hw)
{
	//TODO Need to come up with a memory allocation strategy for dcels
    struct slot slot = empty_slot;
	if (hw[0] > 0.0f && hw[1] > 0.0f && hw[2] > 0.0f)
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

struct slot led_CollisionTriMeshBvhAdd(struct led *led, const utf8 id, struct triMeshBvh *mesh_bvh)
{
    struct slot slot = empty_slot;
	if (mesh_bvh->mesh->v_count && bt_NodeCount(&mesh_bvh->bvh.tree))
	{
		struct c_Shape shape =
		{
			.id = id,
			.type = C_SHAPE_TRI_MESH,
			.mesh_bvh = *mesh_bvh, 
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
	if (slot.index != STRING_DATABASE_STUB_INDEX && shape->reference_count == 0)
	{
		void *buf = shape->id.buf;
		strdb_Remove(&led->cs_db, id);
	}
}

struct slot led_CollisionShapeLookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->cs_db, id);
}

struct slot led_ShapePrefabAdd(struct led *led, const utf8 id, const utf8 cshape, const f32 density, const f32 restitution, const f32 friction, const f32 margin)
{
	struct slot slot = empty_slot;	
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate ds_ShapePrefab: prefab->id must not be empty");
	} 
	else if (strdb_Lookup(&led->shape_prefab_db, id).index != STRING_DATABASE_STUB_INDEX) 
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
			struct slot ref = strdb_Reference(&led->cs_db, cshape);
			if (ref.index == STRING_DATABASE_STUB_INDEX)
			{
				LogString(T_LED, S_WARNING, "In ds_ShapePrefab: shape not found, stub_shape chosen");
			}
			slot = strdb_AddAndAlias(&led->shape_prefab_db, copy);

			struct ds_ShapePrefab *prefab = slot.address;
            prefab->id = Utf8CopyBuffered(prefab->id_buf, PREFAB_BUFSIZE, copy);
			prefab->cshape = ref.index;
            prefab->render_mesh = STRING_DATABASE_STUB_INDEX;
			prefab->restitution = restitution;
			prefab->friction = friction;
			prefab->density = density;
            prefab->margin = margin;
		}
	}

	return slot;
}

void led_ShapePrefabRemove(struct led *led, const utf8 id)
{
	struct slot slot = led_ShapePrefabLookup(led, id);
	struct ds_ShapePrefab *prefab = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && prefab->reference_count == 0)
	{
		strdb_Dereference(&led->cs_db, prefab->cshape);
        strdb_Dereference(&led->render_mesh_db, prefab->render_mesh);
		strdb_Remove(&led->shape_prefab_db, id);
	}	
}

struct slot led_ShapePrefabLookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->shape_prefab_db, id);
}

void led_ShapePrefabAttachRenderMesh(struct led *led, const utf8 id, const utf8 render_mesh)
{
    struct slot slot_shape = led_ShapePrefabLookup(led, id);
    struct slot slot_mesh = led_RenderMeshLookup(led, render_mesh);
    if (slot_shape.index == STRING_DATABASE_STUB_INDEX) 
	{
		Log(T_LED, S_WARNING, "Failed to attach render mesh to shape: shape %k does not exist.", &id);
	} 
    else if (slot_mesh.index == STRING_DATABASE_STUB_INDEX) 
	{
		Log(T_LED, S_WARNING, "Failed to attach render mesh to shape: mesh %k does not exist.", &render_mesh);
	}
    else
	{ 
        led_ShapePrefabDetachRenderMesh(led, id);
        struct ds_ShapePrefab *shape = slot_shape.address;
        shape->render_mesh = strdb_Reference(&led->render_mesh_db, render_mesh).index;
    }
}

void led_ShapePrefabDetachRenderMesh(struct led *led, const utf8 id)
{
    struct slot slot = led_ShapePrefabLookup(led, id);
    if (slot.index != STRING_DATABASE_STUB_INDEX)
    {
        struct ds_ShapePrefab *shape = slot.address;
        strdb_Dereference(&led->render_mesh_db, shape->render_mesh);
    }
}

struct slot led_RigidBodyPrefabAdd(struct led *led, const utf8 id, const u32 dynamic)
{
	struct slot slot = empty_slot;	
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate body_prefab: prefab->id must not be empty");
	} 
	else if (strdb_Lookup(&led->body_prefab_db, id).index != STRING_DATABASE_STUB_INDEX) 
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
			struct ds_RigidBodyPrefab *prefab = strdb_AddAndAlias(&led->body_prefab_db, copy).address;
            prefab->id = Utf8CopyBuffered(prefab->id_buf, PREFAB_BUFSIZE, copy);
            prefab->shape_list = dll_Init(struct ds_ShapePrefabInstance);
			prefab->dynamic = dynamic;
        }
	}

	return slot;
}

static void led_ShapePrefabInstanceRemove(struct led *led, const u32 i)
{
    const struct ds_ShapePrefabInstance *instance = ds_PoolAddress(&led->shape_prefab_instance_pool, i);
    strdb_Dereference(&led->shape_prefab_db, instance->shape_prefab);
    ds_PoolRemove(&led->shape_prefab_instance_pool, i);
}

void led_RigidBodyPrefabRemove(struct led *led, const utf8 id)
{
	struct slot slot = led_RigidBodyPrefabLookup(led, id);
	struct ds_RigidBodyPrefab *prefab = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && prefab->reference_count == 0)
	{
        for (u32 i = prefab->shape_list.first; i != DLL_NULL; )
        {
            const struct ds_ShapePrefabInstance *instance = ds_PoolAddress(&led->shape_prefab_instance_pool, i);
            i = instance->dll_next;
            led_ShapePrefabInstanceRemove(led, i);
        }

		void *buf = prefab->id.buf;
		strdb_Remove(&led->body_prefab_db, id);
	}	
}

struct slot led_RigidBodyPrefabLookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->body_prefab_db, id);
}

void led_RigidBodyPrefabAttachShape(struct led *led, const utf8 rb_id, const utf8 shape_id, const utf8 local_shape_id, const ds_Transform *t_local)
{
    struct ds_RigidBodyPrefab *body_prefab = led_RigidBodyPrefabLookup(led, rb_id).address;
    struct ds_ShapePrefab *shape_prefab = led_ShapePrefabLookup(led, shape_id).address;
    const u64 id_reqsize = Utf8SizeRequired(local_shape_id);
    if (!body_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to ds_RigidBodyPrefab %k, prefab doesn't exist", &rb_id);
    } 
    else if (!shape_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to ds_RigidBodyPrefab %k, ds_ShapePrefab %k does not exist.", &rb_id, &shape_id);
    }
    else if (led_RigidBodyPrefabLookupShape(led, rb_id, local_shape_id).address != NULL)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to to ds_RigidBodyPrefab %k, a shape instance with local id %k already exists in the body.", &rb_id, &local_shape_id);
    }
    else if (PREFAB_BUFSIZE < id_reqsize)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to to ds_RigidBodyPrefab %k, identifier buffer size is %luB but requires %luB.", &rb_id, PREFAB_BUFSIZE, id_reqsize);
    }
    else
    {
        struct slot slot = ds_PoolAdd(&led->shape_prefab_instance_pool);
        struct ds_ShapePrefabInstance *instance = slot.address;

        dll_Prepend(&body_prefab->shape_list, led->shape_prefab_instance_pool.buf, slot.index);
        instance->id = Utf8CopyBuffered(instance->id_buf, PREFAB_BUFSIZE, local_shape_id);
        instance->shape_prefab = strdb_Reference(&led->shape_prefab_db, shape_id).index;
        instance->t_local = *t_local;
    }
}

void led_RigidBodyPrefabDetachShape(struct led *led, const utf8 rb_id, const utf8 local_shape_id)
{
    struct ds_RigidBodyPrefab *body_prefab = led_RigidBodyPrefabLookup(led, rb_id).address;
    struct slot slot = led_RigidBodyPrefabLookupShape(led, rb_id, local_shape_id);
    struct ds_ShapePrefabInstance *instance = slot.address;
    if (!body_prefab)
    {
		Log(T_LED, S_WARNING, "Failed to deattach shape to ds_RigidBodyPrefab %k, prefab doesn't exist", &rb_id);
    } 
    else if (!instance)
    {
		Log(T_LED, S_WARNING, "Failed to attach shape to to ds_RigidBodyPrefab %k, a shape instance with local id %k does not exist in the body.", &rb_id, &local_shape_id);
    }
    else
    {
        strdb_Dereference(&led->shape_prefab_db, instance->shape_prefab);
        dll_Remove(&body_prefab->shape_list, led->shape_prefab_instance_pool.buf, slot.index);
    }
}

struct slot led_RigidBodyPrefabLookupShape(struct led *led, const utf8 rb_id, const utf8 local_shape_id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct slot prefab_slot = led_RigidBodyPrefabLookup(led, rb_id);
    if (prefab_slot.index != STRING_DATABASE_STUB_INDEX)
    {
        const struct ds_RigidBodyPrefab *prefab = prefab_slot.address;
        struct ds_ShapePrefabInstance *instance = NULL;
        for (u32 i = prefab->shape_list.first; i != DLL_NULL; i = instance->dll_next)
        {
            instance = ds_PoolAddress(&led->shape_prefab_instance_pool, i);
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
	else if (strdb_Lookup(&led->render_mesh_db, id).index != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate render mesh: mesh with given id already exist");
	}
	else
	{ 
		u8 buf[R_MESH_BUFSIZE];
		const utf8 copy = Utf8CopyBuffered(buf, R_MESH_BUFSIZE, id);	
		if (!copy.len)
		{
			Log(T_LED, S_WARNING, "Failed to allocate render mesh: id size must be <= %luB", R_MESH_BUFSIZE);
		}
		else
		{
			slot = strdb_AddAndAlias(&led->render_mesh_db, copy);
			struct r_Mesh *mesh = slot.address;
            mesh->id = Utf8CopyBuffered(mesh->id_buf, R_MESH_BUFSIZE, copy);

			struct slot ref = strdb_Lookup(&led->cs_db, shape);
			if (ref.index == STRING_DATABASE_STUB_INDEX)
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
				case C_SHAPE_TRI_MESH: { r_MeshTriMesh(&led->mem_persistent, mesh, s->mesh_bvh.mesh); } break;
			}
		}
	}

	return slot;
}

void led_RenderMeshRemove(struct led *led, const utf8 id)
{
	struct slot slot = strdb_Lookup(&led->render_mesh_db, id);
	struct r_Mesh *mesh = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && mesh->reference_count == 0)
	{
		void *buf = mesh->id.buf;
		strdb_Remove(&led->render_mesh_db, id);
	}
}

struct slot led_RenderMeshLookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->render_mesh_db, id);
}

static struct triMesh TriMeshPerlinNoise(struct arena *mem_persistent, const u32 n, const f32 width)
{
	ds_Assert(PowerOfTwoCheck(n) && n >= 32);

	struct arena tmp = ArenaAlloc1MB();

	struct triMesh mesh = 
	{
		.v_count = (n-1)*(n-1),
		.tri_count = 2*(n-2)*(n-2),
	};
	mesh.v = ArenaPush(mem_persistent, mesh.v_count*sizeof(vec3));
	mesh.tri = ArenaPush(mem_persistent, mesh.tri_count*sizeof(vec3u32));
	//TODO move out functino to TriMeshPerlinNoise method
	//TODO return stub mesh
	ds_Assert(mesh.v && mesh.tri);

	const f32 unit = width / n;

#define OCTAVES	6
	vec2ptr grad[OCTAVES];
	for (u32 o = 0; o < OCTAVES; ++o)
	{
		const u32 on = (n >> o) + 1;
		grad[o] = ArenaPush(&tmp, on*on*sizeof(vec2));
		for (u32 x = 0; x < on; ++x)
		{
			for (u32 z = 0; z < on; ++z)
			{
				const f32 angle = RngF32Range(0.0f, F32_PI2);
				grad[o][x*on + z][0] = f32_cos(angle);
				grad[o][x*on + z][1] = f32_sin(angle);
			}
		}
	}

	const vec3 offset =
	{
		-unit * (n >> 1),
		-20.0f,
		-unit * (n >> 1) + 30.0f,
	};

	for (u32 x = 0; x < n-1; ++x)
	{
		for (u32 z = 0; z < n-1; ++z)
		{
			mesh.v[x*(n-1) + z][0] = (0.5f + x) * unit;
			mesh.v[x*(n-1) + z][1] = 0.0f;
			mesh.v[x*(n-1) + z][2] = (0.5f + z) * unit;

			f32 amplitude = 1.0f / (1 << OCTAVES);
			for (u32 i = 0; i < OCTAVES; ++i)
			{
				const u32 on = (n >> i) + 1;
				const u32 x_low = x - (x % (1 << i));
				const u32 z_low = z - (z % (1 << i));

				const u32 x_high = x_low + (1 << i);
				const u32 z_high = z_low + (1 << i);
	
				const vec2 bl_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_low * unit, 	
					mesh.v[x*(n-1) + z][2] - z_low * unit, 	
				};

				const vec2 tl_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_low * unit, 	
					mesh.v[x*(n-1) + z][2] - z_high * unit, 	
				};

				const vec2 br_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_high * unit, 	
					mesh.v[x*(n-1) + z][2] - z_low * unit, 	
				};

				const vec2 tr_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_high * unit, 	
					mesh.v[x*(n-1) + z][2] - z_high * unit, 	
				};

				//const f32 bl_dot = Vec2Dot(bl_diff, grad[i][(x_low >> i)*on + (z_low >> i)]);
				//const f32 br_dot = Vec2Dot(br_diff, grad[i][(x_high >> i)*on + (z_low >> i)]);
				//const f32 tl_dot = Vec2Dot(tl_diff, grad[i][(x_low >> i)*on + (z_high >> i)]);
				//const f32 tr_dot = Vec2Dot(tr_diff, grad[i][(x_high >> i)*on + (z_high >> i)]);
				
				const u32 xg_low = x_low >> i;
				const u32 xg_high = xg_low + 1;
				const u32 zg_low = z_low >> i;
				const u32 zg_high = zg_low + 1;

				ds_Assert(xg_low*on  + zg_low  < on*on);
				ds_Assert(xg_high*on + zg_low  < on*on);
				ds_Assert(xg_low*on  + zg_high < on*on);
				ds_Assert(xg_high*on + zg_high < on*on);

				const f32 bl_dot = Vec2Dot(bl_diff, grad[i][xg_low*on  + zg_low]);
				const f32 br_dot = Vec2Dot(br_diff, grad[i][xg_high*on + zg_low]);
				const f32 tl_dot = Vec2Dot(tl_diff, grad[i][xg_low*on  + zg_high]);
				const f32 tr_dot = Vec2Dot(tr_diff, grad[i][xg_high*on + zg_high]);

				//fprintf(stderr, "(%u, %u), (%u, %u)\n",
				//	(x_low >> i),
				//	(x_high >> i),
				//	(z_low >> i),
				//	(z_high >> i));

				const vec2 low = 
				{
					x_low * unit,
					z_low * unit,
				};

				const vec2 high = 
				{
					x_high * unit,
					z_high * unit,
				};

				const vec2 t = 
				{
					(mesh.v[x*(n-1) + z][0] - low[0]) / (high[0] - low[0]),
					(mesh.v[x*(n-1) + z][2] - low[1]) / (high[1] - low[1]),
				};

				const vec2 smoothstep =
				{
					6*t[0]*t[0]*t[0]*t[0]*t[0] - 15*t[0]*t[0]*t[0]*t[0] + 10*t[0]*t[0]*t[0],
					6*t[1]*t[1]*t[1]*t[1]*t[1] - 15*t[1]*t[1]*t[1]*t[1] + 10*t[1]*t[1]*t[1],
				};

				//if (i == OCTAVES-4)
				//{
				//fprintf(stderr, "(%u, %u) -> (%u, %u), (%u, %u)\n", x, z, x_low, z_low, x_high, z_high);
				////fprintf(stderr, "%f, %f\n", low[0], low[1]);
				//fprintf(stderr, "%f, %f\n", t[0], t[1]);
				////fprintf(stderr, "%f, %f\n", smoothstep[0], smoothstep[1]);
				//}

				const f32 vb = bl_dot*(1.0f - smoothstep[0]) + br_dot*smoothstep[0];
				const f32 vt = tl_dot*(1.0f - smoothstep[0]) + tr_dot*smoothstep[0];
				const f32 perlin = vb*(1.0f - smoothstep[1]) + vt*smoothstep[1];

				mesh.v[x*(n-1) + z][1] += perlin * amplitude;
				amplitude *= 2.0f;
			}

			mesh.v[x*(n-1) + z][0] += offset[0];
			mesh.v[x*(n-1) + z][1] += offset[1];
			mesh.v[x*(n-1) + z][2] += offset[2];
		}
	}

	for (u32 x = 0; x < n-2; ++x)
	{
		for (u32 z = 0; z < n-2; ++z)
		{
			mesh.tri[2*(x*(n-2) + z) + 0][0] = x*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 0][1] = x*(n-1) + z+1;
			mesh.tri[2*(x*(n-2) + z) + 0][2] = (x+1)*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 1][0] = (x+1)*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 1][1] = x*(n-1) + z+1;
			mesh.tri[2*(x*(n-2) + z) + 1][2] = (x+1)*(n-1) + z+1;
		}
	}

	ArenaFree1MB(&tmp);

	struct aabb bbox = TriMeshBbox(&mesh);
	vec3 local_origin;
	Vec3Scale(local_origin, bbox.center, -1.0f);
	for (u32 i = 0; i < mesh.v_count; ++i)
	{
		Vec3Translate(mesh.v[i], local_origin);
	}

	return mesh;
}

void led_WallSmashSimulationSetup(struct led *led)
{
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
    utf8 id;
    ds_Id tagged_id;

#define dsphere_v_count 	30
	const u32 dsphere_count = 40;
	const u32 capsule_count = 20;
	const u32 tower1_count = 2;
	const u32 tower2_count = 4;
	const u32 tower1_box_count = 40;
	const u32 tower2_box_count = 10;
    const u32 multibox_count = 5;
	const u32 pyramid_layers = 15;
	const u32 pyramid_count = 3;
	const u32 bodies = tower1_box_count + tower2_box_count + 3 + pyramid_layers*(pyramid_layers+1) / 2;

	/* Setup rigid bodies */
	const f32 box_friction = 0.8f;
	const f32 ramp_friction = 0.1f;
	const f32 sphere_friction = 0.1f;
	const f32 floor_friction = 0.8f;
	const f32 capsule_friction = 0.8f;

	const f32 alpha1 = 0.7f;
	const f32 alpha2 = 0.5f;
	const vec4 dsphere_color = { 244.0f/256.0f, 0.1f, 0.4f, alpha2 };
	const vec4 tower1_color = { 154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f,alpha1 };
	const vec4 tower2_color = { 54.0f/256.0f, 183.0f/256.0f, 122.0f/256.0f, alpha2 };
	const vec4 pyramid_color = { 254.0f/256.0f, 181.0f/256.0f, 82.0f/256.0f,alpha2 };
	const vec4 floor_color = { 0.8f, 0.6f, 0.6f,                            alpha2 };
	const vec4 ramp_color = { 165.0f/256.0f, 242.0f/256.0f, 243.0f/256.0f,  alpha2 };
	const vec4 sphere_color = { 0.2f, 0.9f, 0.5f,                             alpha1 };
	const vec4 capsule_color = { 0.1f, 0.4f, 0.8f, 				alpha2 };
	const vec4 multibox_color = { 0.2f, 0.3f, 0.6f, 				alpha2 };
	const vec4 map_color = { 0.5f, 0.5f, 0.8f, 0.7f };

	const f32 box_side = 1.0f;
	struct aabb box_aabb = { .center = { 0.0f, 0.0f, 0.0f }, .hw = { box_side / 2.0f, box_side / 4.0f, box_side / 2.0f} };

	const f32 ramp_width = 10.0f;
	const f32 ramp_length = 60.0f;
	const f32 ramp_height = 34.0f;
	const u32 v_count = 6;
	vec3 ramp_vertices[6] = 
	{
		{0.0f, 		    ramp_height,	-ramp_length},
		{ramp_width, 	ramp_height, 	-ramp_length},
		{0.0f, 		    0.0f, 		    -ramp_length},
		{ramp_width, 	0.0f, 		    -ramp_length},
		{0.0f, 		    0.0f, 		    0.0f},
		{ramp_width, 	0.0f, 		    0.0f},
	};

	vec3 dsphere_vertices[dsphere_v_count];
	const f32 phi = F32_PI * (3.0f - f32_sqrt(5.0f));
	for (u32 i = 0; i < dsphere_v_count; ++i)
	{
		const f32 y = 1.0 - i*2.0f/(dsphere_v_count-1);
		Vec3Set(dsphere_vertices[i]
				, f32_cos(i*phi)*f32_sqrt(1 - y*y)
				, y
				, f32_sin(i*phi)*f32_sqrt(1 - y*y));
	}

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
    const vec3 ramp_hw = { 8.0f * ramp_width, 0.5f, ramp_length };
    led_CollisionBoxAdd(led, id, ramp_hw);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
    led_CollisionCapsuleAdd(led, id, 0.5f, 1.0f);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
    const vec3 box_hw = { box_side / 2.0f, box_side / 4.0f, box_side / 2.0f };
    led_CollisionBoxAdd(led, id, box_hw);

	id = Utf8Cstr(sys_win->ui->mem_frame, "c_sphere");
    led_CollisionSphereAdd(led, id, 2.0f);

	struct dcel *c_ramp = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_ramp = DcelConvexHull(&sys_win->mem_persistent, ramp_vertices, 6, F32_EPSILON * 100.0f);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_ramp");
    led_CollisionDcelAdd(led, id, c_ramp);

	struct dcel *c_dsphere = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_dsphere = DcelConvexHull(&sys_win->mem_persistent, dsphere_vertices, dsphere_v_count, F32_EPSILON * 100.0f);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_dsphere");
    led_CollisionDcelAdd(led, id, c_dsphere);

	struct triMesh *map = ArenaPush(&led->mem_persistent, sizeof(struct triMesh));
	*map = TriMeshPerlinNoise(&led->mem_persistent, 64, 100.0f);
	struct triMeshBvh *mesh_bvh = ArenaPush(&led->mem_persistent, sizeof(struct triMeshBvh));
	f32 best_cost = F32_INFINITY;
	u32 best_bin_count = 8;
	for (u32 i = 8; i < 16; ++i)
	{
		ArenaPushRecord(&led->mem_persistent);
		*mesh_bvh = TriMeshBvhConstruct(&led->mem_persistent, map, i);
		const f32 cost = BvhCost(&mesh_bvh->bvh);
		if (cost < best_cost)
		{
			best_cost = cost;
			best_bin_count = i;
		}
		ArenaPopRecord(&led->mem_persistent);
	}
	*mesh_bvh = TriMeshBvhConstruct(&led->mem_persistent, map, best_bin_count);
	id = Utf8Cstr(sys_win->ui->mem_frame, "c_map");
    led_CollisionTriMeshBvhAdd(led, id, mesh_bvh);

    const u32 rb_static = 0;
    const u32 rb_dynamic = 1;
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_map"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_floor"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_ramp"), rb_static);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_box"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_multibox"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_capsule"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_sphere"), rb_dynamic);
    led_RigidBodyPrefabAdd(led, Utf8Inline("rb_dsphere"), rb_dynamic);

    led_RenderMeshAdd(led, Utf8Inline("rm_map"), Utf8Inline("c_map"));
    led_RenderMeshAdd(led, Utf8Inline("rm_floor"), Utf8Inline("c_floor"));
    led_RenderMeshAdd(led, Utf8Inline("rm_ramp"), Utf8Inline("c_ramp"));
    led_RenderMeshAdd(led, Utf8Inline("rm_capsule"), Utf8Inline("c_capsule"));
    led_RenderMeshAdd(led, Utf8Inline("rm_box"), Utf8Inline("c_box"));
    led_RenderMeshAdd(led, Utf8Inline("rm_sphere"), Utf8Inline("c_sphere"));
    led_RenderMeshAdd(led, Utf8Inline("rm_dsphere"), Utf8Inline("c_dsphere"));

    const f32 density = 1.0f;
    const f32 restitution = 0.0f;
    const f32 margin = 0.5f;
    led_ShapePrefabAdd(led, Utf8Inline("s_map"), Utf8Inline("c_map"), density, restitution, floor_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_floor"), Utf8Inline("c_floor"), density, restitution, floor_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_box"), Utf8Inline("c_box"), density, restitution, box_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_capsule"), Utf8Inline("c_capsule"), density, restitution, capsule_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_sphere"), Utf8Inline("c_sphere"), 100.0f, restitution, sphere_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_dsphere"), Utf8Inline("c_dsphere"), density, restitution, box_friction, margin);
    led_ShapePrefabAdd(led, Utf8Inline("s_ramp"), Utf8Inline("c_ramp"), density, restitution, ramp_friction, margin);

    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_map"), Utf8Inline("rm_map"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_floor"), Utf8Inline("rm_floor")); 
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_box"), Utf8Inline("rm_box"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_capsule"), Utf8Inline("rm_capsule"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_sphere"), Utf8Inline("rm_sphere"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_dsphere"), Utf8Inline("rm_dsphere"));
    led_ShapePrefabAttachRenderMesh(led, Utf8Inline("s_ramp"), Utf8Inline("rm_ramp"));

    ds_Transform transform = ds_TransformIdentity();
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_map"), Utf8Inline("s_map"), Utf8Inline("l_s_map"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_floor"), Utf8Inline("s_floor"), Utf8Inline("l_s_floor"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_box"), Utf8Inline("s_box"), Utf8Inline("l_s_box"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_capsule"), Utf8Inline("s_capsule"), Utf8Inline("l_s_capsule"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_sphere"), Utf8Inline("s_sphere"), Utf8Inline("l_s_sphere"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_dsphere"), Utf8Inline("s_dsphere"), Utf8Inline("l_s_dsphere"), &transform);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_ramp"), Utf8Inline("s_ramp"), Utf8Inline("l_s_ramp"), &transform);

    Vec3Set(transform.position, 0.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box0"), &transform);
    Vec3Set(transform.position, 2.0f, 0.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box1"), &transform);
    Vec3Set(transform.position, 0.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box2"), &transform);
    Vec3Set(transform.position, 2.0f, 0.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box3"), &transform);
    Vec3Set(transform.position, 0.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box4"), &transform);
    Vec3Set(transform.position, 2.0f, 1.0f, 0.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box5"), &transform);
    Vec3Set(transform.position, 0.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box6"), &transform);
    Vec3Set(transform.position, 2.0f, 1.0f, 2.0f);
    led_RigidBodyPrefabAttachShape(led, Utf8Inline("rb_multibox"), Utf8Inline("s_box"), Utf8Inline("l_s_box7"), &transform);

	const vec3 sphere_translation = { -0.5, 0.5f + ramp_height, -ramp_length };
	const vec3 box_translation =  {-0.5f, 0.0f, -0.5f};
	const vec3 ramp_translation = {0.0f , ramp_width, -ramp_length};
	const vec3 floor_translation = { 0.0f, -ramp_width/2.0f - 1.0f, ramp_length / 2.0f -ramp_width/2.0f};
	const vec3 box_base_translation = { 0.0f, floor_translation[1] + 1.0f, floor_translation[2] / 2.0f};
	const vec3 dsphere_base_translation = { -15.0f, floor_translation[1] + 1.0f, floor_translation[2] / 2.0f + 20.0f};
    const vec3 multibox_base_translation = { 0.0f, floor_translation[1] + 1.0f, floor_translation[2] / 2.0f -10.0f };
    const vec3 map_translation = { 0.0f, -25.0f, 0.0f };

    id = Utf8Cstr(sys_win->ui->mem_frame, "led_floor");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, floor_translation);
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_floor"));
    led_NodeSetColor(led, tagged_id, floor_color, 1.0f);
	
    id = Utf8Cstr(sys_win->ui->mem_frame, "led_map");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, map_translation);
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_map"));
    led_NodeSetColor(led, tagged_id, map_color, 1.0f);

    id = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, ramp_translation);
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_ramp"));
    led_NodeSetColor(led, tagged_id, ramp_color, 1.0f);

    id = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
    tagged_id = led_NodeAdd(led, id, Utf8Empty());
    led_NodeSetPosition(led, tagged_id, sphere_translation);
    led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_sphere"));
    led_NodeSetColor(led, tagged_id, sphere_color, 1.0f);

	for (u32 i = 0; i < multibox_count; ++i)
    {
        vec3 translation;
		Vec3Copy(translation, multibox_base_translation);
		translation[0] += 0.0f;
		translation[1] += i*2.5f;
		translation[2] += 0.0f;

		id = Utf8Format(sys_win->ui->mem_frame, "multibox_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, translation);
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_multibox"));
        led_NodeSetColor(led, tagged_id, multibox_color, 1.0f);
    }
	
	for (u32 i = 0; i < capsule_count; ++i)
	{	
		vec3 translation;
		Vec3Copy(translation, dsphere_base_translation);
		translation[0] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_cos(i * F32_PI*37.0f/197.0f);
		translation[1] += 25.0f + (f32) i / 2.0f;
		translation[2] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_sin(i * F32_PI*37.0f/197.0f);

		id = Utf8Format(sys_win->ui->mem_frame, "capsule_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, translation);
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_capsule"));
        led_NodeSetColor(led, tagged_id, capsule_color, 1.0f);
	}

	for (u32 i = 0; i < dsphere_count; ++i)
	{	
		vec3 translation;
		Vec3Copy(translation, dsphere_base_translation);
		translation[0] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_cos(i * F32_PI*37.0f/197.0f);
		translation[1] += 5.0f + (f32) i / 2.0f;
		translation[2] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_sin(i * F32_PI*37.0f/197.0f);

		id = Utf8Format(sys_win->ui->mem_frame, "dsphere_%u", i);
        tagged_id = led_NodeAdd(led, id, Utf8Empty());
        led_NodeSetPosition(led, tagged_id, translation);
        led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_dsphere"));
        led_NodeSetColor(led, tagged_id, dsphere_color, 1.0f);
	}

	for (u32 k = 0; k < pyramid_count; ++k)
	{
		for (u32 i = 0; i < pyramid_layers; ++i)
		{
			const f32 local_y = i * box_side;
			for (u32 j = 0; j < pyramid_layers-i; ++j)
			{
				const f32 local_x = j -(pyramid_layers-i-1) * box_side / 2.0f;
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[0] += local_x;
				translation[1] += local_y;
				translation[2] += 10.0f * k;

				id = Utf8Format(sys_win->ui->mem_frame, "pyramid_%u_%u_%u", i, j, k);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, translation);
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                led_NodeSetColor(led, tagged_id, pyramid_color, 1.0f);
			}
		}
	}

	for (u32 k = 0; k < tower1_count; ++k)
	{
		for (u32 j = 0; j < tower1_count; ++j)
		{
			for (u32 i = 0; i < tower1_box_count; ++i)
			{
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[2] += 15.0f + 2.0f*k;
				translation[1] += (f32) i * box_aabb.hw[1] * 2.10f;
				translation[0] += 15.0f + 2.0f*j;

				id = Utf8Format(sys_win->ui->mem_frame, "tower1_%u_%u_%u", i, j, k);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, translation);
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                led_NodeSetColor(led, tagged_id, tower1_color, 1.0f);
			}
		}
	}

	for (u32 k = 0; k < tower2_count; ++k)
	{
		for (u32 j = 0; j < tower2_count; ++j)
		{
			for (u32 i = 0; i < tower2_box_count; ++i)
			{
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[2] += 15.0f + 2.0f*k;
				translation[1] += (f32) i * box_aabb.hw[1] * 2.10f;
				translation[0] -= 15.0f + 2.0f*j;
			
				id = Utf8Format(sys_win->ui->mem_frame, "tower2_%u_%u_%u", i, j, k);
                tagged_id = led_NodeAdd(led, id, Utf8Empty());
                led_NodeSetPosition(led, tagged_id, translation);
                led_NodeAttachRigidBodyPrefab(led, tagged_id, Utf8Inline("rb_box"));
                led_NodeSetColor(led, tagged_id, tower2_color, 1.0f);
			}
		}
	}
}

void cmd_led_compile(void)
{
	return led_Compile(g_editor);
}

void cmd_led_run(void)
{
	return led_Run(g_editor);
}

void cmd_led_pause(void)
{
	return led_Pause(g_editor);
}

void cmd_led_stop(void)
{
	return led_Stop(g_editor);
}

void led_Compile(struct led *led)
{
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

static void led_EngineFlush(struct led *led)
{
//	PhysicsPipelineFlush(&led->physics);
//	struct led_Node *node = NULL;
//	for (u32 i = led->node_non_marked_list.first; i != DLL_NULL; i = dll_Next(node))
//	{
//		node = ds_PoolAddress(&led->node_hierarchy, i);
//        struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//        struct ds_ShapePrefabInstance *instance = NULL;
//        for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//        {
//            instance = ds_PoolAddress(&led->shape_prefab_instance_pool, j);
//            r_Proxy3dLinearSpeculationSet(node->position
//					, node->rotation
//					, (vec3) { 0 } 
//					, (vec3) { 0 } 
//					, led->ns
//					, instance->proxy);
//		    struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->proxy);
//            proxy->flags |= PROXY3D_DRAW;
//		    Vec4Copy(proxy->color, node->color);
//        }
//	}
}

static void led_EngineInit(struct led *led)
{
//	//TODO move this into engine flush
//	PhysicsPipelineFlush(&led->physics);		
//	led->physics.ns_start = led->ns;
//	led->physics.ns_elapsed = -led->ns_delta;
//	led->ns_engine_paused = 0;
//
//	struct led_Node *node = NULL;
//	for (u32 i = led->node_non_marked_list.first; i != DLL_NULL; i = dll_Next(node))
//	{
//		node = ds_GPoolAddress(&led->node_hierarchy, i);
//		if (node->flags & LED_PHYSICS)
//		{
//			struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//			if (Utf8Equivalence(prefab->id, Utf8Inline("rb_map")))
//			{
//                struct ds_ShapePrefabInstance *instance = ds_PoolAddress(&led->shape_prefab_instance_pool, prefab->shape_list.first);
//				vec3 axis = { 0.6f, 1.0f, 0.6f };
//				Vec3ScaleSelf(axis, 1.0f / f32_sqrt(Vec3Length(axis)));
//				const f32 angle = F32_PI / 16.0f;
//				QuatAxisAngle(node->rotation, axis, angle);
//				vec3 linear_velocity = { 0.0f, 0.0f, 0.0f};
//				vec3 angular_velocity = { 0.0f, 0.0f, 0.0f};
//				r_Proxy3dLinearSpeculationSet(node->position
//						, node->rotation
//						, linear_velocity
//						, angular_velocity
//						, led->ns 
//						, instance->proxy);
//			}
//            ds_Transform t;
//            Vec3Set(t.position, 0.0f, 0.0f, 0.0f);
//            QuatAxisAngle(t.rotation, Vec3Inline(0.0f, 1.0f, 0.0f), 0.0f);
//	    	const ds_RigidBodyId body = ds_RigidBodyAdd(&led->physics, prefab, node->position, node->rotation, i);
//    
//            struct ds_ShapePrefabInstance *instance = NULL;
//            struct ds_ShapePrefab *shape_prefab = NULL;
//            for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//            {
//                instance = ds_PoolAddress(&led->shape_prefab_instance_pool, j);
//                shape_prefab = strdb_Address(&led->shape_prefab_db, instance->shape_prefab);
//                instance->shape = ds_ShapeAdd(&led->physics, shape_prefab, &instance->t_local, body);
//            }
//
//            ds_RigidBodyUpdateMassProperties(&led->physics, body);
//		}
//	}
}

static void led_EngineColorBodies(struct led *led, const u32 island, const vec4 color)
{
//	struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, island);
//	const struct ds_RigidBody *body;
//	for (u32 i = is->body_list.first; i != DLL_NULL; i = body->dll2_next)
//	{
//		body = ds_PoolAddress(&led->physics.body_pool, i);
//		const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, body->entity);
//        const struct ds_RigidBodyPrefab *prefab = strdb_Address(&led->body_prefab_db, node->body_prefab);
//        struct ds_ShapePrefabInstance *instance = NULL;
//        for (u32 j = prefab->shape_list.first; j != DLL_NULL; j = instance->dll_next)
//        {
//		    struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->proxy);
//		    Vec4Copy(proxy->color, node->color);
//        }
//	}
}

static void led_engine_run(struct led *led)
{
//	led->physics.ns_elapsed += led->ns_delta;
//
//	//const u64 game_frames_to_run = (game->ns_elapsed - (game->frames_completed * game->ns_tick)) / game->ns_tick;
//	const u64 physics_frames_to_run = (led->physics.ns_elapsed - (led->physics.frames_completed * led->physics.ns_tick)) / led->physics.ns_tick;
//
//	//u64 ns_next_game_frame = game->frames_completed * game->ns_tick;
//	u64 ns_next_physics_frame = (led->physics.frames_completed+1) * led->physics.ns_tick;
//
//	for (u64 i = 0; i < /* game_frames_to_run + */ physics_frames_to_run; ++i)
//	{
//		//if (ns_next_game_frame <= ns_next_physics_frame)
//		//{
//		//	ArenaFlush(&game->frame);
//		//	game_tick(game);
//		//	game->frames_completed += 1;
//		//	ns_next_game_frame += game->ns_tick;
//		//}
//		//else
//		//{
//			PhysicsPipelineTick(&led->physics);
//			ns_next_physics_frame += led->physics.ns_tick;
//		//}
//	}
//
//	if (led->physics.pending_body_color_mode != led->physics.body_color_mode)
//	{
//		switch (led->physics.pending_body_color_mode)
//		{
//			case RB_COLOR_MODE_BODY: 
//			{ 
//				const struct ds_RigidBody *body = NULL;
//				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
//				{
//					body = ds_PoolAddress(&led->physics.body_pool, i);
//                    const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, body->entity);
//                    led_NodeColorProxies(led, body->entity, node->color);
//				}
//			} break;
//
//			case RB_COLOR_MODE_COLLISION: 
//			{ 
//				const struct ds_RigidBody *body = NULL;
//				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
//				{
//					body = ds_PoolAddress(&led->physics.body_pool, i);
//                    //TODO;
//					//const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, body->entity);
//					//struct r_Proxy3d *proxy = r_Proxy3dAddress(node->proxy);
//					//if (RB_IS_DYNAMIC(body))
//					//{
//					//	(body->contact_first == NLL_NULL)
//					//		? Vec4Copy(proxy->color, node->color)
//					//		: Vec4Copy(proxy->color, led->physics.collision_color);
//					//}
//					//else
//					//{
//					//	Vec4Copy(proxy->color, led->physics.static_color);
//					//}
//				}
//			} break;
//
//			case RB_COLOR_MODE_SLEEP: 
//			{ 
//				const struct ds_RigidBody *body = NULL;
//				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
//				{
//					body = ds_PoolAddress(&led->physics.body_pool, i);
//                    if (!RB_IS_DYNAMIC(body))
//					{						
//                        led_NodeColorProxies(led, body->entity, led->physics.static_color);
//					}
//					else
//					{
//						(RB_IS_AWAKE(body))
//                            ? led_NodeColorProxies(led, body->entity,  led->physics.awake_color)
//                            : led_NodeColorProxies(led, body->entity,  led->physics.sleep_color);
//					}
//				}
//			} break;
//
//			case RB_COLOR_MODE_ISLAND: 
//			{ 
//				const struct ds_RigidBody *body = NULL;
//				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
//				{
//					body = ds_PoolAddress(&led->physics.body_pool, i);
//					const struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, body->entity);
//					if (body->island_index == ISLAND_STATIC)
//					{
//                        led_NodeColorProxies(led, body->entity, led->physics.static_color);
//					}
//					else
//					{
//						const struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, body->island_index);
//                        led_NodeColorProxies(led, body->entity, is->color);
//					}
//				}
//			} break;
//		}
//	}
//	led->physics.body_color_mode = led->physics.pending_body_color_mode;
//
//	struct physicsEvent *event = NULL;
//	for (u32 i = led->physics.event_list.first; i != DLL_NULL; )
//	{
//		event = ds_PoolAddress(&led->physics.event_pool, i);
//		const u32 next = dll_Next(event);
//		switch (event->type)
//		{
//			case PHYSICS_EVENT_CONTACT_NEW:
//			{
//				if (led->physics.body_color_mode == RB_COLOR_MODE_COLLISION)
//				{
//                    const struct ds_Contact *c = ds_ContactLookup(&led->physics, event->contact).address;
//                    if (c)
//                    {
//					    const struct ds_RigidBody *body1 = ds_PoolAddress(&led->physics.body_pool, c->key.body0);
//					    const struct ds_RigidBody *body2 = ds_PoolAddress(&led->physics.body_pool, c->key.body1);
//					    if (RB_IS_DYNAMIC(body1))
//					    {
//                            led_NodeColorProxies(led, body1->entity, led->physics.collision_color);
//					    }
//					    if (RB_IS_DYNAMIC(body2))
//					    {
//                            led_NodeColorProxies(led, body2->entity, led->physics.collision_color);
//					    }
//                    }
//				}
//			} break;
//
//			case PHYSICS_EVENT_CONTACT_REMOVED:
//			{
//				if (led->physics.body_color_mode == RB_COLOR_MODE_COLLISION)
//				{
//					const struct ds_RigidBody *body1 = ds_RigidBodyLookup(&led->physics, event->contact_removed_bodies[0]).address;
//					const struct ds_RigidBody *body2 = ds_RigidBodyLookup(&led->physics, event->contact_removed_bodies[1]).address;
//                    if (body1 && body2)
//                    {
//					    const struct led_Node *node1 = ds_PoolAddress(&led->node_hierarchy, body1->entity);
//					    const struct led_Node *node2 = ds_PoolAddress(&led->node_hierarchy, body2->entity);
//
//
//					    if (RB_IS_DYNAMIC(body1))
//					    {
//                            const struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, body1->island_index);
//					    	if (is->contact_list.count == 0)
//					    	{
//                                led_NodeColorProxies(led, body1->entity, node1->color);
//					    	}
//					    }
//					    else
//					    {
//                            led_NodeColorProxies(led, body1->entity, led->physics.static_color);
//					    }
//		
//					    if (RB_IS_DYNAMIC(body2))
//					    {
//                            const struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, body2->island_index);
//					    	if (is->contact_list.count)
//					    	{
//                                led_NodeColorProxies(led, body2->entity, node2->color);
//					    	}
//					    }
//					    else
//					    {
//                            led_NodeColorProxies(led, body2->entity, led->physics.static_color);
//					    }
//                    }
//				}
//			} break;
//
//#ifdef DS_PHYSICS_DEBUG
//			case PHYSICS_EVENT_ISLAND_NEW:
//			{
//				struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, event->island);
//				if (PoolSlotAllocated(is))
//				{
//					Vec4Set(is->color, 
//							RngF32Normalized(), 
//							RngF32Normalized(), 
//							RngF32Normalized(), 
//							0.7f);
//					if (led->physics.body_color_mode == RB_COLOR_MODE_ISLAND)
//					{
//						led_EngineColorBodies(led, event->island, is->color);
//					}
//					else if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
//					{
//						led_EngineColorBodies(led, event->island, led->physics.awake_color);
//					}
//				}
//			} break;
//
//			case PHYSICS_EVENT_ISLAND_EXPANDED:
//			{
//				if (led->physics.body_color_mode == RB_COLOR_MODE_ISLAND)
//				{
//					const struct ds_Island *is = ds_PoolAddress(&led->physics.is_db.island_pool, event->island);
//					if (PoolSlotAllocated(is))
//					{
//						led_EngineColorBodies(led, event->island, is->color);
//					}
//				}
//			} break;
//#endif
//
//			case PHYSICS_EVENT_ISLAND_REMOVED:
//			{
//			} break;
//
//			case PHYSICS_EVENT_ISLAND_AWAKE:
//			{
//				if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
//				{
//					led_EngineColorBodies(led, event->island, led->physics.awake_color);
//				}
//			} break;
//
//			case PHYSICS_EVENT_ISLAND_ASLEEP:
//			{
//				if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
//				{
//					led_EngineColorBodies(led, event->island, led->physics.sleep_color);
//				}
//			} break;
//			
//			case PHYSICS_EVENT_BODY_NEW:
//			{
//			} break;
//
//			case PHYSICS_EVENT_BODY_REMOVED:
//			{
//                led_NodeDontDrawProxies(led, event->entity);
//			} break;
//
//			case PHYSICS_EVENT_BODY_ORIENTATION:
//			{
//				const struct ds_RigidBody *body = ds_PoolAddress(&led->physics.body_pool, event->body);
//				struct led_Node *node = ds_PoolAddress(&led->node_hierarchy, body->entity);
//
//				vec3 linear_velocity;
//				Vec3Scale(linear_velocity, body->linear_momentum, 1.0f / body->mass);
//				r_Proxy3dLinearSpeculationSet(body->t_world.position
//						, body->t_world.rotation
//						, linear_velocity
//						, body->angular_velocity
//						, event->ns
//						, node->proxy);
//			} break;
//		}
//		ds_PoolRemove(&led->physics.event_pool, i);
//		i = next;
//	}
//	
//	dll_Flush(&led->physics.event_list);
}

void led_CoreInitCommands(void)
{
    //TODO
}

void led_Core(struct led *led)
{
	static u32 once = 1;
	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
	if (once && sys_win)
	{
		once = 0;
		led_WallSmashSimulationSetup(led);
	}

	if (led->engine_initalized && !led->pending_engine_initalized)
	{
		led_EngineFlush(led);
	}

	if (!led->engine_initalized && led->pending_engine_initalized)
	{
		led_EngineInit(led);
	}
	led->engine_initalized = led->pending_engine_initalized;
	led->engine_running = led->pending_engine_running;
	led->engine_paused = led->pending_engine_paused;
	if (led->engine_running)
	{
		led->ns_engine_running += led->ns_delta;
		led_engine_run(led);
	}

	if (led->engine_paused)
	{
		led->ns_engine_paused += led->ns_delta;
	}
}

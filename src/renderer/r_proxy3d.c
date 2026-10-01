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

HI_DEFINE(r_Proxy3d);

void r_Proxy3dBufferLocalLayoutSet(void)
{
	ds_glEnableVertexAttribArray(3);
	ds_glEnableVertexAttribArray(4);

	ds_glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, L_PROXY3D_STRIDE, (void *) L_PROXY3D_POSITION_OFFSET);
	ds_glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, L_PROXY3D_STRIDE, (void *) L_PROXY3D_NORMAL_OFFSET);
}

void r_Proxy3dBufferSharedLayoutSet(void)
{

	ds_glEnableVertexAttribArray(0);
	ds_glEnableVertexAttribArray(1);
	ds_glEnableVertexAttribArray(2);

	ds_glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, S_PROXY3D_STRIDE, (void *) S_PROXY3D_TRANSLATION_BLEND_OFFSET);
	ds_glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, S_PROXY3D_STRIDE, (void *) S_PROXY3D_ROTATION_OFFSET);
	ds_glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, S_PROXY3D_STRIDE, (void *) S_PROXY3D_COLOR_OFFSET);

	ds_glVertexAttribDivisor(0, 1);
	ds_glVertexAttribDivisor(1, 1);
	ds_glVertexAttribDivisor(2, 1);
}

void r_Proxy3dLinearSpeculationSet(const v3 position, const q rotation, const v3 linear_velocity, const v3 angular_velocity, const u64 ns_time, const u32 proxy_index)
{
	struct r_Proxy3d *proxy = r_Proxy3dAddress(proxy_index);

	proxy->flags &= ~(PROXY3D_SPECULATE_FLAGS | PROXY3D_MOVING);
	proxy->flags |= PROXY3D_SPECULATE_LINEAR;
	proxy->ns_at_update = ns_time;
	proxy->position = position;
	proxy->rotation = rotation;
	proxy->spec_position = position;
	proxy->spec_rotation = rotation;
	proxy->linear.linear_velocity = linear_velocity;
	proxy->linear.angular_velocity = angular_velocity;
	if (V3Dot(linear_velocity, linear_velocity) + V3Dot(angular_velocity, angular_velocity) > 0.0f)
	{
		proxy->flags |= PROXY3D_MOVING;
	}
}

u32 r_Proxy3dAlloc(const struct r_Proxy3d_config *config)
{
	struct slot slot = r_Proxy3dHIAdd(&g_r_core->proxy3d_hierarchy, config->parent);
	struct r_Proxy3d *proxy = slot.address;
	proxy->flags = (config->parent != g_r_core->proxy3d_root)
		? PROXY3D_DRAW | PROXY3D_RELATIVE
		: PROXY3D_DRAW;

	proxy->mesh = r_MeshSDBReference(g_r_core->mesh_database, config->mesh).index;
	proxy->color = config->color;
	proxy->blend = config->blend;

	r_Proxy3dLinearSpeculationSet(config->position, config->rotation, config->linear_velocity, config->angular_velocity, config->ns_time, slot.index);

	return slot.index;
}

void r_Proxy3dDealloc(struct arena *tmp, const u32 proxy_index)
{
    if (proxy_index != PROXY3D_NULL)
    {
	    struct r_Proxy3d *proxy = r_Proxy3dAddress(proxy_index);
	    r_MeshSDBDereference(g_r_core->mesh_database, proxy->mesh);
	    r_Proxy3dHIRemove(tmp, &g_r_core->proxy3d_hierarchy, proxy_index);
    }
}

struct r_Proxy3d *r_Proxy3dAddress(const u32 proxy)
{
	return g_r_core->proxy3d_hierarchy.pool.buf + proxy;
}

/* Calculate the speculative movement of the proxy locally, i.e., the position of the proxy not counting any position type effects */
static void r_InternalProxy3dLocalSpeculativeOrientation(struct r_Proxy3d *proxy, const u64 ns_time)
{
	const f32 timestep = (f32) (ns_time - proxy->ns_at_update) / NSEC_PER_SEC;

	switch (proxy->flags & PROXY3D_SPECULATE_FLAGS)
	{
		case PROXY3D_SPECULATE_LINEAR:
		{
			proxy->spec_position = V3AddScaled(proxy->position, proxy->linear.linear_velocity, timestep);

			const q a_vel_quat = Q(proxy->linear.angular_velocity.x, 
					       proxy->linear.angular_velocity.y, 
					       proxy->linear.angular_velocity.z,
					       0.0f);
			const q rot_delta = QScale(QMul(a_vel_quat, proxy->rotation), timestep / 2.0f);
			proxy->spec_rotation = QNormalize(QAdd(proxy->rotation, rot_delta));
		} break;
		
		default:
		{
			proxy->spec_position = proxy->position;	
			proxy->spec_rotation = proxy->rotation;	
		} break;
	}	
}

void r_Proxy3dHierarchySpeculate(struct arena *mem, const u64 ns_time)
{
	ArenaPushRecord(mem);

    HII it;
    HIIInit(it, g_r_core->proxy3d_hierarchy, g_r_core->proxy3d_root);
	// skip root stub 
    HIIAdvance(it, g_r_core->proxy3d_hierarchy);
	while (it.at != (i32) g_r_core->proxy3d_root)
	{
		const u32 index = it.at;
        HIIAdvance(it, g_r_core->proxy3d_hierarchy);

		struct r_Proxy3d *proxy = r_Proxy3dAddress(index);
		if (proxy->flags & PROXY3D_MOVING)
		{
			r_InternalProxy3dLocalSpeculativeOrientation(proxy, ns_time);
		}

		if (proxy->hi_parent != (i32) g_r_core->proxy3d_root)
		{
			const struct r_Proxy3d *parent = r_Proxy3dAddress(proxy->hi_parent);
			if ((proxy->flags & PROXY3D_MOVING) == 0)
			{
				proxy->spec_position = proxy->position;	
				proxy->spec_rotation = proxy->rotation;	
			}

            proxy->spec_position = V3Add(M3V3Mul(M3Q(parent->spec_rotation), proxy->spec_position), parent->spec_position);
            proxy->spec_rotation = QNormalize(QMul(parent->spec_rotation, proxy->spec_rotation));
		}
	}
	ArenaPopRecord(mem);
}

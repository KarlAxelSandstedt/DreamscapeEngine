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

#include "r_local.h"

void r_proxy3d_buffer_local_layout_setter(void)
{
	ds_glEnableVertexAttribArray(3);
	ds_glEnableVertexAttribArray(4);

	ds_glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, L_PROXY3D_STRIDE, (void *) L_PROXY3D_POSITION_OFFSET);
	ds_glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, L_PROXY3D_STRIDE, (void *) L_PROXY3D_NORMAL_OFFSET);
}

void r_proxy3d_buffer_shared_layout_setter(void)
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

void r_proxy3d_set_linear_speculation(const vec3 position, const quat rotation, const vec3 linear_velocity, const vec3 angular_velocity, const u64 ns_time, const u32 proxy_index)
{
	struct r_proxy3d *proxy = r_proxy3d_address(proxy_index);

	proxy->flags &= ~(PROXY3D_SPECULATE_FLAGS | PROXY3D_MOVING);
	proxy->flags |= PROXY3D_SPECULATE_LINEAR;
	proxy->ns_at_update = ns_time;
	Vec3Copy(proxy->position, position);
	QuatCopy(proxy->rotation, rotation);
	Vec3Copy(proxy->spec_position, position);
	QuatCopy(proxy->spec_rotation, rotation);
	Vec3Copy(proxy->linear.linear_velocity, linear_velocity);
	Vec3Copy(proxy->linear.angular_velocity, angular_velocity);
	if (Vec3Dot(linear_velocity, linear_velocity) + Vec3Dot(angular_velocity, angular_velocity) > 0.0f)
	{
		proxy->flags |= PROXY3D_MOVING;
	}
}

u32 r_proxy3d_alloc(const struct r_proxy3d_config *config)
{
	struct slot slot = hi_Add(g_r_core->proxy3d_hierarchy, config->parent);
	struct r_proxy3d *proxy = slot.address;
	proxy->flags = (config->parent != g_r_core->proxy3d_root)
		? PROXY3D_RELATIVE
		: 0;

	proxy->mesh = strdb_Reference(g_r_core->mesh_database, config->mesh).index;
	Vec4Copy(proxy->color, config->color);
	proxy->blend = config->blend;

	r_proxy3d_set_linear_speculation(config->position, config->rotation, config->linear_velocity, config->angular_velocity, config->ns_time, slot.index);

	return slot.index;
}
void r_proxy3d_dealloc(struct arena *tmp, const u32 proxy_index)
{
	struct r_proxy3d *proxy = r_proxy3d_address(proxy_index);
	strdb_Dereference(g_r_core->mesh_database, proxy->mesh);
	hi_Remove(tmp, g_r_core->proxy3d_hierarchy, proxy_index);
}

struct r_proxy3d *r_proxy3d_address(const u32 proxy)
{
	return hi_Address(g_r_core->proxy3d_hierarchy, proxy);
}


/* Calculate the speculative movement of the proxy locally, i.e., the position of the proxy not counting any position type effects */
static void internal_r_proxy3d_local_speculative_orientation(struct r_proxy3d *proxy, const u64 ns_time)
{
	const f32 timestep = (f32) (ns_time - proxy->ns_at_update) / NSEC_PER_SEC;

	switch (proxy->flags & PROXY3D_SPECULATE_FLAGS)
	{
		case PROXY3D_SPECULATE_LINEAR:
		{
			proxy->spec_position[0] = proxy->position[0] + proxy->linear.linear_velocity[0] * timestep;
			proxy->spec_position[1] = proxy->position[1] + proxy->linear.linear_velocity[1] * timestep;
			proxy->spec_position[2] = proxy->position[2] + proxy->linear.linear_velocity[1] * timestep;

			quat a_vel_quat, rot_delta;
			QuatSet(a_vel_quat, 
					proxy->linear.angular_velocity[0], 
					proxy->linear.angular_velocity[1], 
					proxy->linear.angular_velocity[2],
				      	0.0f);
			QuatMul(rot_delta, a_vel_quat, proxy->rotation);
			QuatScale(rot_delta, timestep / 2.0f);
			QuatAdd(proxy->spec_rotation, proxy->rotation, rot_delta);
			QuatNormalize(proxy->spec_rotation);	
		} break;
		
		default:
		{
			Vec3Copy(proxy->spec_position, proxy->position);	
			QuatCopy(proxy->spec_rotation, proxy->rotation);	
		} break;
	}	
}

void r_proxy3d_hierarchy_speculate(struct arena *mem, const u64 ns_time)
{
	struct hiIterator it = hi_IteratorAlloc(mem, g_r_core->proxy3d_hierarchy, g_r_core->proxy3d_root);
	// skip root stub 
	hi_IteratorNextDf(&it);
	while (it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		struct r_proxy3d *proxy = r_proxy3d_address(index);
		if (proxy->flags & PROXY3D_MOVING)
		{
			internal_r_proxy3d_local_speculative_orientation(proxy, ns_time);
		}

		if (proxy->header.parent != g_r_core->proxy3d_root)
		{
			const struct r_proxy3d *parent = r_proxy3d_address(proxy->header.parent);
			if ((proxy->flags & PROXY3D_MOVING) == 0)
			{
				Vec3Copy(proxy->spec_position, proxy->position);	
				QuatCopy(proxy->spec_rotation, proxy->rotation);	
			}

			Vec3Translate(proxy->spec_position, parent->spec_position);
			quat tmp;
			QuatCopy(tmp, proxy->spec_rotation);
			QuatMul(proxy->spec_rotation, tmp, parent->spec_rotation);
		}
	}
	hi_IteratorRelease(&it);
}

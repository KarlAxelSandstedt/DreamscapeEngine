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
#include "ds_renderer.h"
#include "ds_asset.h"
#include "transform.h"
#include "ds_led.h"

static struct r_Mesh *DebugContactManifoldSegmentsMesh(struct arena *mem, const struct ds_RigidBodyPipeline *pipeline)
{
	const struct contactManifold *cm = pipeline->cm;
	const u32 cm_count = pipeline->cm_count;

	ArenaPushRecord(mem);

	const u32 vertex_count = 2*pipeline->cm_count;
	struct r_Mesh *mesh = NULL;
 	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count*L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		ArenaPopRecord(mem);
		goto end;
	}
	ArenaRemoveRecord(mem);

	mesh = tmp;
	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = vertex_count; 
	mesh->vertex_data = vertex_data;
	mesh->local_stride = L_COLOR_STRIDE;

	for (u32 i = 0; i < cm_count; ++i)
	{
		vec3 n0, n1;
		switch (cm[i].v_count)
		{
			case 1:
			{
				Vec3Copy(n0, cm[i].v[0]);
				Vec3Add(n1, n0, cm[i].n);
			} break;

			case 2:
			{
				Vec3Interpolate(n0, cm[i].v[0], cm[i].v[1], 0.5f);
				Vec3Add(n1, n0, cm[i].n);
			} break;

			case 3:
			{
				Vec3Scale(n0, cm[i].v[0], 1.0f/3.0f);
				Vec3TranslateScaled(n0, cm[i].v[1], 1.0f/3.0f);
				Vec3TranslateScaled(n0, cm[i].v[2], 1.0f/3.0f);
				Vec3Add(n1, n0, cm[i].n);
			} break;

			case 4:
			{
				Vec3Scale(n0, cm[i].v[0], 1.0f/4.0f);
				Vec3TranslateScaled(n0, cm[i].v[1], 1.0f/4.0f);
				Vec3TranslateScaled(n0, cm[i].v[2], 1.0f/4.0f);
				Vec3TranslateScaled(n0, cm[i].v[3], 1.0f/4.0f);
				Vec3Add(n1, n0, cm[i].n);
			} break;

			default:
			{
				continue;
			} break;
		}

		Vec3Copy((f32 *) vertex_data +  0, n0);
		Vec4Copy((f32 *) vertex_data +  3, pipeline->manifold_color);
		Vec3Copy((f32 *) vertex_data +  7, n1);
		Vec4Copy((f32 *) vertex_data + 10, pipeline->manifold_color);
		vertex_data += 2*(sizeof(vec3) + sizeof(vec4));
	}
end:
	return mesh;

}

static struct r_Mesh *DebugContactManifoldTrianglesMesh(struct arena *mem, const struct ds_RigidBodyPipeline *pipeline)
{
	const struct contactManifold *cm = pipeline->cm;
	const u32 cm_count = pipeline->cm_count;

	ArenaPushRecord(mem);

	u32 vertex_count = 6*pipeline->cm_count;

	struct r_Mesh *mesh = NULL;
 	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count*L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		ArenaPopRecord(mem);
		goto end;
	}
	ArenaRemoveRecord(mem);

	mesh = tmp;
	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = 0; 
	mesh->vertex_data = vertex_data;
	mesh->local_stride = L_COLOR_STRIDE;

	vec3 v[4];
	u32 cm_triangles = 0;
	u32 cm_planes = 0;
	for (u32 i = 0; i < cm_count; ++i)
	{
		switch (cm[i].v_count)
		{
			case 3:
			{
				Vec3Copy(v[0], cm[i].v[0]);
				Vec3Copy(v[1], cm[i].v[1]);
				Vec3Copy(v[2], cm[i].v[2]);
				Vec3TranslateScaled(v[0], cm[i].n, 0.005f);
				Vec3TranslateScaled(v[1], cm[i].n, 0.005f);
				Vec3TranslateScaled(v[2], cm[i].n, 0.005f);

				Vec3Copy((f32 *) vertex_data +  0, v[0]);
				Vec4Copy((f32 *) vertex_data +  3, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data +  7, v[1]);
				Vec4Copy((f32 *) vertex_data + 10, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data + 14, v[2]);
				Vec4Copy((f32 *) vertex_data + 17, pipeline->manifold_color);
				vertex_data += 3*(sizeof(vec3) + sizeof(vec4));
				mesh->vertex_count += 3; 
			} break;

			case 4:
			{
				Vec3Copy(v[0], cm[i].v[0]);
				Vec3Copy(v[1], cm[i].v[1]);
				Vec3Copy(v[2], cm[i].v[2]);
				Vec3Copy(v[3], cm[i].v[3]);
				Vec3TranslateScaled(v[0], cm[i].n, 0.005f);
				Vec3TranslateScaled(v[1], cm[i].n, 0.005f);
				Vec3TranslateScaled(v[2], cm[i].n, 0.005f);
				Vec3TranslateScaled(v[3], cm[i].n, 0.005f);

				Vec3Copy((f32 *) vertex_data +  0, v[0]);
				Vec4Copy((f32 *) vertex_data +  3, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data +  7, v[1]);
				Vec4Copy((f32 *) vertex_data + 10, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data + 14, v[2]);
				Vec4Copy((f32 *) vertex_data + 17, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data + 21, v[0]);
				Vec4Copy((f32 *) vertex_data + 24, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data + 28, v[2]);
				Vec4Copy((f32 *) vertex_data + 31, pipeline->manifold_color);
				Vec3Copy((f32 *) vertex_data + 35, v[3]);
				Vec4Copy((f32 *) vertex_data + 38, pipeline->manifold_color);
				vertex_data += 6*(sizeof(vec3) + sizeof(vec4));
				mesh->vertex_count += 6; 
			} break;

			default:
			{
				continue;
			} break;
		}
	}
end:
	return mesh;
}

static struct r_Mesh *DebugLinesMesh(struct arena *mem, const struct ds_RigidBodyPipeline *pipeline)
{
	ArenaPushRecord(mem);

	u32 vertex_count = 0;
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		vertex_count += 2*pipeline->debug[i].stack_segment.next;
	}

	struct r_Mesh *mesh = NULL;
 	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count*L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		ArenaPopRecord(mem);
		goto end;
	}
	ArenaRemoveRecord(mem);

	mesh = tmp;
	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = vertex_count; 
	mesh->vertex_data = vertex_data;
	mesh->local_stride = L_COLOR_STRIDE;

	u64 mem_left = mesh->vertex_count * L_COLOR_STRIDE;

	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		for (u32 j = 0; j < pipeline->debug[i].stack_segment.next; ++j)
		{
			Vec3Copy((f32 *) vertex_data +  0, pipeline->debug[i].stack_segment.arr[j].segment.p0);
			Vec4Copy((f32 *) vertex_data +  3, pipeline->debug[i].stack_segment.arr[j].color);
			Vec3Copy((f32 *) vertex_data +  7, pipeline->debug[i].stack_segment.arr[j].segment.p1);
			Vec4Copy((f32 *) vertex_data + 10, pipeline->debug[i].stack_segment.arr[j].color);
			vertex_data += 2*(sizeof(vec3) + sizeof(vec4));
			mem_left -= 2*(sizeof(vec3) + sizeof(vec4));
		}
	}
	ds_Assert(mem_left == 0);
end:
	return mesh;
}

static struct r_Mesh *BoundingBoxesMesh(struct arena *mem, const struct ds_RigidBodyPipeline *pipeline, const vec4 color)
{
	ArenaPushRecord(mem);
	const u32 vertex_count = 3*8*pipeline->body_pool.count;
	struct r_Mesh *mesh = NULL;
 	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count * L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		ArenaPopRecord(mem);
		goto end;
	}
	ArenaRemoveRecord(mem);

	mesh = tmp;
	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = vertex_count; 
	mesh->vertex_data = vertex_data;
	mesh->local_stride = L_COLOR_STRIDE;

	u64 mem_left = mesh->vertex_count * L_COLOR_STRIDE;
	struct ds_RigidBody *body = NULL;
	for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
	{
		body = PoolAddress(&pipeline->body_pool, i);
        struct ds_Shape *shape = NULL;
        for (u32 j = body->shape_list.first; j != DLL_NULL; j = shape->dll_next)
        {
            shape = PoolAddress(&pipeline->shape_pool, j);
            const struct aabb bbox = ds_ShapeWorldBbox(pipeline, shape);
		    const u64 bytes_written = AabbPushLinesBuffered(vertex_data, mem_left, &bbox, color);
		    vertex_data += bytes_written;
		    mem_left -= bytes_written;
        }
	}
	ds_Assert(mem_left == 0);
end:
	return mesh;
}

static struct r_Mesh *bvh_Mesh(struct arena *mem, const struct bvh *bvh, const vec3 translation, const quat rotation, const vec4 color)
{
	mat3 rot;
	Mat3Quat(rot, rotation);

	ArenaPushRecord(mem);
	const u32 vertex_count = 3*8*bvh->tree.pool.count;
 	struct r_Mesh *mesh = NULL;
	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count * L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		goto end;
	}
	ArenaRemoveRecord(mem);

	mesh = tmp;
	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = vertex_count; 
	mesh->vertex_data = vertex_data;
	mesh->local_stride = L_COLOR_STRIDE;

	ArenaPushRecord(mem);
	struct memArray arr = ArenaPushAlignedAll(mem, sizeof(u32), 4); 
	u32 *stack = arr.addr;

	u32 i = bvh->tree.root;
	u32 sc = U32_MAX;


	const struct bvhNode *nodes = (struct bvhNode *) bvh->tree.pool.buf;
	u64 mem_left = mesh->vertex_count * L_COLOR_STRIDE;
	while (i != U32_MAX)
	{
		const u64 bytes_written = AabbTransformPushLinesBuffered(vertex_data, mem_left, &nodes[i].bbox, translation, rot, color);
		vertex_data += bytes_written;
		mem_left -= bytes_written;

		if (!bt_LeafCheck(nodes + i))
		{
			sc += 1;
			if (sc == arr.len)
			{
				goto end;	
			}
			stack[sc] = nodes[i].bt_right;
			i = nodes[i].bt_left;
		}
		else if (sc != U32_MAX)
		{
			i = stack[sc--];
		}
		else
		{
			i = U32_MAX;
		}
	}
	ds_Assert(mem_left == 0);
end:
	ArenaPopRecord(mem);
	return mesh;
}

static void r_EditorDraw(const struct led *led)
{
	ProfZone;

	//{
	//	struct slot slot = strdb_Lookup(&led->render_mesh_db, Utf8Inline("rm_map"));
	//	ds_Assert(slot.index != STRING_DATABASE_STUB_INDEX);
	//	if (slot.index != STRING_DATABASE_STUB_INDEX)
	//	{
	//		const u64 material = r_MaterialConstruct(PROGRAM_LIGHTNING, slot.index, TEXTURE_NONE);
	//		const u64 depth = 0x7fffff;
	//		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, 0, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
	//		struct r_Instance *instance = r_InstanceAddNonCached(cmd);
	//		instance->type = R_INSTANCE_MESH;
	//		instance->mesh = slot.address;
	//	}
	//}

	const u32 depth_exponent = 1 + f32_exponent_bits(led->cam.fz_far);
	ds_Assert(depth_exponent >= 23);

	r_Proxy3dHierarchySpeculate(&g_r_core->frame, led->ns - led->ns_engine_paused);

	ArenaPushRecord(&g_r_core->frame);
	struct hiIterator it = hi_IteratorAlloc(&g_r_core->frame, &g_r_core->proxy3d_hierarchy, PROXY3D_ROOT);
	// skip root stub 
	hi_IteratorNextDf(&it);
	while (it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		struct r_Proxy3d *proxy = r_Proxy3dAddress(index);

		const f32 dist = Vec3Distance(proxy->spec_position, led->cam.position);
		const u32 unit_exponent = f32_exponent_bits(dist);
		const u64 depth = (unit_exponent <= depth_exponent && unit_exponent > (depth_exponent - 23))
			? (0x00800000 | f32_mantissa_bits(dist)) >> (depth_exponent - unit_exponent + 1)
			: 0;

		const u64 transparency = (proxy->color[3] == 1.0f)
			? R_CMD_TRANSPARENCY_OPAQUE
			: R_CMD_TRANSPARENCY_ADDITIVE;

		const u64 material = r_MaterialConstruct(PROGRAM_PROXY3D, proxy->mesh, TEXTURE_NONE);
		const struct r_Mesh *r_mesh = strdb_Address(&led->render_mesh_db, proxy->mesh);
		const u64 command = (r_mesh->index_data)
			? r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_INSTANCED, R_CMD_ELEMENTS)
			: r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_INSTANCED, R_CMD_ARRAYS);
		
		r_InstanceAdd(index, command);
	}
	ArenaPopRecord(&g_r_core->frame);

	if (led->physics.draw_dbvh)
	{
		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		quat rotation;
		const vec3 translation = { 0 };
		vec3 axis = { 0.0f, 1.0f, 0.0f };
		const f32 angle = 0.0f;
		QuatAxisAngle(rotation, axis, angle);
		struct r_Mesh *mesh = bvh_Mesh(&g_r_core->frame, &led->physics.shape_bvh, translation, rotation, led->physics.dbvh_color);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}
	}

	if (led->physics.draw_sbvh)
	{

		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct ds_RigidBody *body = NULL;
		for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
		{
			body = PoolAddress(&led->physics.body_pool, i);
			if (body->shape_type != C_SHAPE_TRI_MESH)
			{
				continue;
			}

			const struct c_Shape *shape = strdb_Address(led->physics.cshape_db, body->shape_handle);
			struct r_Mesh *mesh = bvh_Mesh(&g_r_core->frame, &shape->mesh_bvh.bvh, body->position, body->rotation, led->physics.sbvh_color);
			if (mesh)
			{
				struct r_Instance *instance = r_InstanceAddNonCached(cmd);
				instance->type = R_INSTANCE_MESH;
				instance->mesh = mesh;
			}
		}
	}

	if (led->physics.draw_bounding_box)
	{
		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct r_Mesh *mesh = BoundingBoxesMesh(&g_r_core->frame, &led->physics, led->physics.bounding_box_color);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}
	}

	if (led->physics.draw_lines)
	{
		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct r_Mesh *mesh = DebugLinesMesh(&g_r_core->frame, &led->physics);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}
	}

	if (led->physics.draw_manifold)
	{

		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;

		const u64 cmd1 = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct r_Mesh *mesh = DebugContactManifoldTrianglesMesh(&g_r_core->frame, &led->physics);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd1);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}

		const u64 cmd2 = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		mesh = DebugContactManifoldSegmentsMesh(&g_r_core->frame, &led->physics);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd2);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}

	}

	ProfZoneEnd;
}

static void r_InternalProxy3dUniforms(const struct led *led, const u32 window)
{
	mat4 perspective, view;
	const struct r_Camera *cam = &led->cam;
	mat4Perspective(perspective, cam->aspect_ratio, cam->fov_x, cam->fz_near, cam->fz_far);
	mat4View(view, cam->position, cam->left, cam->up, cam->forward);
	
	ds_glUseProgram(g_r_core->program[PROGRAM_PROXY3D].gl_program);
	GLint aspect_ratio_addr, view_addr, perspective_addr, light_position_addr;
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "perspective");
	light_position_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "light_position");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniform3f(light_position_addr, cam->position[0], cam->position[1], cam->position[2]);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, (f32 *) perspective);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, (f32 *) view);

	ds_glUseProgram(g_r_core->program[PROGRAM_LIGHTNING].gl_program);
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "perspective");
	light_position_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "light_position");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniform3f(light_position_addr, cam->position[0], cam->position[1], cam->position[2]);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, (f32 *) perspective);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, (f32 *) view);
	
	
	ds_glUseProgram(g_r_core->program[PROGRAM_COLOR].gl_program);
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "perspective");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, (f32 *) perspective);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, (f32 *) view);
}

static void r_InternalUiUniforms(const u32 window)
{
	vec2u32 resolution;
	ds_WindowSize(resolution, window);

	ds_glUseProgram(g_r_core->program[PROGRAM_UI].gl_program);
	GLint resolution_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_UI].gl_program, "resolution");
	ds_glUniform2f(resolution_addr, (f32) resolution[0], (f32) resolution[1]);
}

static void r_SceneRender(const struct led *led, const u32 window)
{
	ProfZone;

	struct ds_Window *sys_win = ds_WindowAddress(window);
	ds_glViewport(0, 0, (i32) sys_win->size[0], (i32) sys_win->size[1]); 

	ds_glClearColor(0.08f, 0.08f, 0.08f, 1.0f);
	ds_glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (struct r_Bucket *b = sys_win->r_scene->frame_bucket_list; b; b = b->next)
	{
		ProfZoneNamed("render bucket");
		switch (b->screen_layer)
		{
			case R_CMD_SCREEN_LAYER_GAME:
			{
				ds_glEnableDepthTesting();
			} break;

			case R_CMD_SCREEN_LAYER_HUD:
			{
				ds_glDisableDepthTesting();
			} break;

			default:
			{
				ds_AssertString(0, "unimplemented");
			} break;
		}

		switch (b->transparency)
		{
			case R_CMD_TRANSPARENCY_OPAQUE:
			{
				ds_glDisableBlending();
			} break;

			case R_CMD_TRANSPARENCY_ADDITIVE:
			{
				ds_glEnableBlending();
				ds_glBlendEquation(GL_FUNC_ADD);
			} break;

			case R_CMD_TRANSPARENCY_SUBTRACTIVE:
			{
				ds_glEnableBlending();
				ds_glBlendEquation(GL_FUNC_SUBTRACT);
			} break;

			default: 
			{ 
				 ds_AssertString(0, "unexpected transparency setting"); 
			} break;
		}
		
		const u32 program = MATERIAL_PROGRAM_GET(b->material);
		ds_glUseProgram(g_r_core->program[program].gl_program);

		const u32 mesh = MATERIAL_MESH_GET(b->material);
		const u32 texture = MATERIAL_TEXTURE_GET(b->material);	
		switch (program)
		{
			case PROGRAM_UI:
			{
				u32 tx_index = 0;
				ds_glActiveTexture(GL_TEXTURE0 + tx_index);
				//TODO setup compile time arrays as with g_r_core->program[program].gl_program (?????)
				ds_glBindTexture(GL_TEXTURE_2D, g_r_core->texture[texture].handle);
				const i32 texture_addr = ds_glGetUniformLocation(g_r_core->program[program].gl_program, "texture");
				ds_glUniform1i(texture_addr, tx_index);
				ds_glViewport(0, 0, (i32) sys_win->size[0], (i32) sys_win->size[1]); 
			} break;

			case PROGRAM_LIGHTNING:
			case PROGRAM_COLOR:
			case PROGRAM_PROXY3D:
			{
				ds_glViewport(led->viewport_position[0]
					       , led->viewport_position[1]
					       , led->viewport_size[0]
					       , led->viewport_size[1]
					       ); 
			} break;
		}

		GLenum mode;
		switch (b->primitive)
		{
			case R_CMD_PRIMITIVE_LINE: { mode = GL_LINES; } break;
			case R_CMD_PRIMITIVE_TRIANGLE: { mode = GL_TRIANGLES; } break;
			default: { ds_AssertString(0, "Unexpected draw primitive"); } break;
		}

		u32 vao;
		ds_glGenVertexArrays(1, &vao);
		ds_glBindVertexArray(vao);
		for (u32 i = 0; i < b->buffer_count; ++i)
		{	
			struct r_Buffer *buf = b->buffer_array[i];
			ds_glGenBuffers(1, &buf->local_vbo);
			ds_glBindBuffer(GL_ARRAY_BUFFER, buf->local_vbo);
			ds_glBufferData(GL_ARRAY_BUFFER, buf->local_size, buf->local_data, GL_STATIC_DRAW);
			g_r_core->program[program].buffer_local_layout_setter();

			if (!b->elements)
			{
					//fprintf(stderr, "\t\tDrawing Array: buf[%u], vbuf[%lu]\n", 
					//		i,
					//		buf->local_size);

				if (!b->instanced)
				{
					ds_glDrawArrays(mode, 0, buf->local_size / g_r_core->program[program].local_stride);
				}
				else
				{
					ds_glGenBuffers(1, &buf->shared_vbo);
					ds_glBindBuffer(GL_ARRAY_BUFFER, buf->shared_vbo);
					ds_glBufferData(GL_ARRAY_BUFFER, buf->shared_size, buf->shared_data, GL_STATIC_DRAW);
					g_r_core->program[program].buffer_shared_layout_setter();

					ds_glDrawArraysInstanced(mode, 0, buf->local_size / g_r_core->program[program].local_stride, buf->instance_count);
					ds_glDeleteBuffers(1, &buf->shared_vbo);

				}
			}
			else
			{
				ds_glGenBuffers(1, &buf->ebo);
				ds_glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf->ebo);
				ds_glBufferData(GL_ELEMENT_ARRAY_BUFFER, buf->index_count * sizeof(u32), buf->index_data, GL_STATIC_DRAW);
				if (!b->instanced)
				{
					//fprintf(stderr, "\t\tDrawing Regular: buf[%u], vbuf[%lu], index_count: %u\n", 
					//		i,
					//		buf->local_size,
					//		buf->index_count);

					ds_glDrawElements(mode, buf->index_count, GL_UNSIGNED_INT, 0);
				}
				else
				{
					//fprintf(stderr, "\t\tDrawing Instanced: buf[%u], sbuf[%lu], vbuf[%lu], index_count: %u, instace_count: %u\n", 
					//		i,
					//		buf->shared_size,
					//		buf->local_size,
					//		buf->index_count,
					//		buf->instance_count);

					ds_glGenBuffers(1, &buf->shared_vbo);
					ds_glBindBuffer(GL_ARRAY_BUFFER, buf->shared_vbo);
					ds_glBufferData(GL_ARRAY_BUFFER, buf->shared_size, buf->shared_data, GL_STATIC_DRAW);
					g_r_core->program[program].buffer_shared_layout_setter();

					ds_glDrawElementsInstanced(mode, buf->index_count, GL_UNSIGNED_INT, 0, buf->instance_count);
					ds_glDeleteBuffers(1, &buf->shared_vbo);
				}	
			}

			ds_glDeleteBuffers(1, &buf->local_vbo);
			ds_glDeleteBuffers(1, &buf->ebo);
		}

		ds_glDeleteVertexArrays(1, &vao);
		ProfZoneEnd;
	}

	ds_WindowSwapGlBuffers(window);
	GL_STATE_ASSERT;
	ProfZoneEnd;
}

void r_EditorMain(const struct led *led)
{
	g_r_core->ns_elapsed = led->ns;
	if (g_r_core->ns_tick)
	{
		const u64 frames_elapsed_since_last_draw = (g_r_core->ns_elapsed - (g_r_core->frames_elapsed * g_r_core->ns_tick)) / g_r_core->ns_tick;
		if (frames_elapsed_since_last_draw)
		{
			ProfZoneNamed("render frame");
			ArenaFlush(&g_r_core->frame);
			g_r_core->frames_elapsed += frames_elapsed_since_last_draw;

			//fprintf(stderr, "led ns: %lu\n", led->ns);
			//fprintf(stderr, "r   ns: %lu\n", g_r_core->ns_elapsed);
			//fprintf(stderr, "p   ns: %lu\n", led->physics.ns_start + led->physics.ns_elapsed);
			//fprintf(stderr, "p f ns: %lu\n", led->physics.frames_completed * led->physics.ns_tick);
			//fprintf(stderr, "spec:   %lu\n", led->ns - led->ns_engine_paused);

			struct ds_Window *win = NULL;

			struct arena tmp = ArenaAlloc1MB();
			struct hiIterator it = hi_IteratorAlloc(&tmp, g_window_hierarchy, g_process_root_window);
			while (it.count)
			{
				const u32 window = hi_IteratorNextDf(&it);
				win = ds_WindowAddress(window);
				if (!win->tagged_for_destruction)
				{
					ds_WindowSetCurrentGlContext(window);
					ds_WindowSetGlobal(window);

					r_SceneSetGlobal(win->r_scene);
					r_SceneFrameBegin();
					{
						r_UiDraw(win->ui);
						r_InternalUiUniforms(window);
						if (window == g_process_root_window)
						{
							r_EditorDraw(led);
							r_InternalProxy3dUniforms(led, window);
						}
					}
					r_SceneFrameEnd();
					r_SceneRender(led, window);
				}
			}
			ArenaFree1MB(&tmp);

			/* NOTE: main context must be set in the case of creating new contexts sharing state. */
			ds_WindowSetCurrentGlContext(g_process_root_window);

			ProfZoneEnd;
		}
	}
	else
	{
		ds_Assert(0);
		g_r_core->frames_elapsed += 1;
	}
}

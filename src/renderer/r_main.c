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


/*
 * The first vertex_count vertices (L_COLOR layout) hold positions in a frame located at position (world); move
 * them into the view's frame (camera-relative): one f64 subtraction per vertex.
 */
static void ColorVerticesToView(void *vertex_data, const u64 vertex_count, const v3f64 position, const v3f64 origin)
{
	const v3f64 translation = V3F64Sub(position, origin);
	u8 *vertex = vertex_data;
	for (u64 i = 0; i < vertex_count; ++i)
	{
		v3 p;
		memcpy(&p, vertex, sizeof(v3));
		p = V3F64Demote(V3F64Add(translation, V3Promote(p)));
		memcpy(vertex, &p, sizeof(v3));
		vertex += L_COLOR_STRIDE;
	}
}

/*
 * Write the 12 edges of box as 24 line vertices (L_COLOR layout: position, color), the corners rotated by rotation.
 * Return the bytes written: 24*L_COLOR_STRIDE, or 0 if bufsize is smaller.
 */
static u64 AabbTransformPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const m3 rotation, const v4 color)
{
	const u64 bytes_written = 24*L_COLOR_STRIDE;
	if (bufsize < bytes_written)
	{
		return 0;
	}

	/* corner i takes the max on the axes whose bit is set in i; an edge joins corners one bit apart */
	v3 corner[8];
	for (u32 i = 0; i < 8; ++i)
	{
		const v3 c = V3((i & 1) ? box->max.x : box->min.x, (i & 2) ? box->max.y : box->min.y, (i & 4) ? box->max.z : box->min.z);
		corner[i] = M3V3Mul(rotation, c);
	}

	static const u8 edge[12][2] = { {0,1}, {2,3}, {4,5}, {6,7}, {0,2}, {1,3}, {4,6}, {5,7}, {0,4}, {1,5}, {2,6}, {3,7} };
	u8 *vertex = buf;
	for (u32 e = 0; e < 12; ++e)
	{
		for (u32 k = 0; k < 2; ++k)
		{
			memcpy(vertex, corner + edge[e][k], sizeof(v3));
			memcpy(vertex + sizeof(v3), &color, sizeof(v4));
			vertex += L_COLOR_STRIDE;
		}
	}

	return bytes_written;
}

static u64 AabbPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const v4 color)
{
	return AabbTransformPushLinesBuffered(buf, bufsize, box, M3Identity(), color);
}

static struct r_Mesh *BoundingBoxesMesh(struct arena *mem, const struct ds_Dynamics *pipeline, const v3f64 origin, const v4 color)
{
	ArenaPushRecord(mem);
	const u32 vertex_count = 24*pipeline->shape_pool.count;
	struct r_Mesh *mesh = NULL;
 	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count * L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		ArenaPopRecord(mem);
		Log(T_RENDERER, S_ERROR, "bounding boxes: %u vertices do not fit the renderer's frame arena, drawn as the stub box", vertex_count);
		mesh = &g_r_core->mesh_stub_color;
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
    for (u64 bi = 0; bi < pipeline->body_usage_set.block_count; ++bi)
    {
        struct ds_BitBlock it = ds_BitBlockInit(pipeline->body_usage_set.bits[bi], bi);
        while (ds_BitBlockHasNext(&it))
        {
            const struct ds_Body *body = pipeline->body_pool.buf + ds_BitBlockNext(&it);
            struct ds_Shape *shape = NULL;
            for (i32 j = body->shape_list.first; (i32) j != DLL_SENTINEL; j = shape->body_shape.next)
            {
                shape = pipeline->shape_pool.buf + j;
                const struct aabb bbox = ds_ShapeWorldBbox(pipeline, shape);
		        const u64 bytes_written = AabbPushLinesBuffered(vertex_data, mem_left, &bbox, color);
		        vertex_data += bytes_written;
		        mem_left -= bytes_written;
            }
        }
    }
	ColorVerticesToView(mesh->vertex_data, (mesh->vertex_count*L_COLOR_STRIDE - mem_left) / L_COLOR_STRIDE, V3F64(0.0, 0.0, 0.0), origin);

	ds_Assert(mem_left == 0);
end:
	return mesh;
}

static struct r_Mesh *BVHMesh(struct arena *mem, const struct BVH *bvh, const v3f64 position, const q rotation, const v3f64 origin, const v4 color)
{
	const m3 rot = M3QUnit(rotation);

	ArenaPushRecord(mem);
	const u32 vertex_count = 3*8*bvh->pool.count;
 	struct r_Mesh *mesh = NULL;
	struct r_Mesh *tmp = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex_data = ArenaPush(mem, vertex_count * L_COLOR_STRIDE);
	if (!tmp || !vertex_data) 
	{ 
		Log(T_RENDERER, S_ERROR, "bvh: %u vertices do not fit the renderer's frame arena, drawn as the stub box", vertex_count);
		mesh = &g_r_core->mesh_stub_color;
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
	u32 sc = 1;
    stack[0] = bvh->bt.root;

	const struct BVHNode *nodes = bvh->pool.buf;
	u64 mem_left = vertex_count*L_COLOR_STRIDE;
	while (sc--)
	{
	    u32 i = stack[sc];
		const u64 bytes_written = AabbTransformPushLinesBuffered(vertex_data, mem_left, &nodes[i].bbox, rot, color);

        ds_Assert(bytes_written == 24*L_COLOR_STRIDE);
		vertex_data += bytes_written;
		mem_left -= bytes_written;

		if (!BTLeafCheck(nodes + i))
		{
			if (sc+2 > arr.len)
			{
				goto end;	
			}
			stack[sc++] = nodes[i].bt_child[0];
			stack[sc++] = nodes[i].bt_child[1];
		}
	}
	ColorVerticesToView(mesh->vertex_data, (mesh->vertex_count*L_COLOR_STRIDE - mem_left) / L_COLOR_STRIDE, position, origin);
	ds_Assert(mem_left == 0);
end:
	ArenaPopRecord(mem);
	return mesh;
}

/*
 * HSL to rgb. The hue walks the colour wheel: red at 0, green at 1/3, blue at 2/3, back to red at 1. Lightness
 * is the mean of the brightest and darkest channel, 0 black, 1 white, 1/2 the pure colour; saturation is how far
 * the channels spread about that mean, 0 grey. So "lighter" is a larger lightness, "paler" a smaller saturation.
 *
 * The spread (chroma) is the largest the lightness allows times the saturation:
 *     c = (1 - |2 l - 1|) s
 * The wheel has six sectors of 60 degrees. In each, one channel is at the top of the spread, one at the bottom,
 * and the third ramps linearly between them:
 *     h' = 6 h                    sector = floor(h'), f = h' - sector
 *     x = c (1 - |(h' mod 2) - 1|)       the ramping channel, up on even sectors, down on odd ones
 *     sector 0: (c, x, 0)  1: (x, c, 0)  2: (0, c, x)  3: (0, x, c)  4: (x, 0, c)  5: (c, 0, x)
 * Finally everything is lifted so the mean lands on the lightness:
 *     m = l - c / 2,  rgb = (r, g, b) + m
 */
static v3 r_InternalHsl(const f32 hue, const f32 saturation, const f32 lightness)
{
	const f32 c = (1.0f - F32Abs(2.0f * lightness - 1.0f)) * saturation;
	const f32 h6 = hue * 6.0f;
	const u32 sector = (u32) h6 % 6;
	const f32 x = c * (1.0f - F32Abs((h6 - 2.0f * (f32) (sector / 2)) - 1.0f));     /* h' mod 2 */
	const f32 m = lightness - 0.5f * c;
	v3 rgb;
	switch (sector)
	{
		case 0:  rgb = V3(c, x, 0.0f); break;
		case 1:  rgb = V3(x, c, 0.0f); break;
		case 2:  rgb = V3(0.0f, c, x); break;
		case 3:  rgb = V3(0.0f, x, c); break;
		case 4:  rgb = V3(x, 0.0f, c); break;
		default: rgb = V3(c, 0.0f, x); break;
	}
	return V3(rgb.x + m, rgb.y + m, rgb.z + m);
}

/*
 * A colour from an id, stable while the id is. Not a hash: hashed hues cluster, and the eye tells only some
 * fifteen hues apart at one saturation and lightness. The golden ratio sequence
 *     hue_i = frac(i * (sqrt(5) - 1) / 2)
 * puts every next id 137.5 degrees further round the wheel, the spacing that stays as even as possible for any
 * count of ids. A second such sequence (the plastic ratio) moves the lightness by up to +-0.08, so ids that land
 * on neighbouring hues still differ.
 */
static v4 r_InternalIdColor(const u32 id, const f32 saturation, const f32 lightness, const f32 alpha)
{
	const f32 golden = 0.6180339887f;
	const f32 plastic = 0.7548776662f;
	const f32 hue_turn = (f32) id * golden;
	const f32 light_turn = (f32) id * plastic;
	const f32 hue = hue_turn - (f32) (u32) hue_turn;
	const f32 light = lightness + 0.16f * ((light_turn - (f32) (u32) light_turn) - 0.5f);
	const v3 rgb = r_InternalHsl(hue, saturation, F32Clamp(light, 0.0f, 1.0f));
	return V4(rgb.x, rgb.y, rgb.z, alpha);
}

/* 1 if any of the shape's contacts has a manifold point */
static u32 r_InternalShapeTouching(const struct ds_Dynamics *pipeline, const u32 shape_index)
{
	const struct ds_Shape *shape = pipeline->shape_pool.buf + shape_index;
	for (i32 ci = shape->contact_list.first; ci != DLL_SENTINEL; )
	{
		const struct ds_Contact *contact = pipeline->contact_pool.buf + ci;
		const u32 side = (contact->key.shape[0] == shape_index) ? 0 : 1;
		ci = contact->shape_contact[side].next;
		for (u32 m = 0; m < contact->narrowphase.manifold_count; ++m)
		{
			if (contact->narrowphase.manifold[m].point_count)
			{
				return 1;
			}
		}
	}
	return 0;
}

/* the colour of a body's shape under the config's mode: derived from the body's state, nothing stored */
static v4 r_InternalShapeColor(const struct ds_Dynamics *pipeline, const struct ds_DynamicsDrawConfig *config, const struct ds_Body *body, const u32 shape_index)
{
	if (config->shape_color_mode != DRAW_COLOR_UNIFORM && ds_BodyStaticCheck(body))
	{
		return config->color_static;
	}

	switch (config->shape_color_mode)
	{
		case DRAW_COLOR_RANDOM:
		{
			return r_InternalIdColor((u32) (body - pipeline->body_pool.buf), config->shape_saturation, config->shape_lightness, config->shape_alpha);
		}

		case DRAW_COLOR_ISLAND:
		{
			return r_InternalIdColor(body->island, config->shape_saturation, config->shape_lightness, config->shape_alpha);
		}

		case DRAW_COLOR_SLEEP:
		{
			return (body->set >= SOLVER_SET_SLEEPING_FIRST) ? config->color_sleeping : config->color_awake;
		}

		case DRAW_COLOR_BODY_TYPE:
		{
			return ds_BodyKinematicCheck(body) ? config->color_kinematic : config->color_dynamic;
		}

		case DRAW_COLOR_VELOCITY:
		{
			/* a sleeping body rests; an active one has its compute state */
			f32 energy = 0.0f;
			if (body->set == SOLVER_SET_ACTIVE)
			{
				const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
				const struct ds_BodyCompute *compute = set->body_compute_pool.buf + body->sim;
				energy = 0.5f * body->mass * V3Dot(compute->linear_velocity, compute->linear_velocity);
			}
			const f32 t = F32Clamp(energy / config->energy_max, 0.0f, 1.0f);
			return V4Add(V4Scale(config->color_velocity_low, 1.0f - t), V4Scale(config->color_velocity_high, t));
		}

		case DRAW_COLOR_TOUCHING:
		{
			return r_InternalShapeTouching(pipeline, shape_index) ? config->color_touching : config->color_not_touching;
		}

		default: { return config->color_shape; }
	}
}

/*
 * draw_shapes: every shape as an instanced raw mesh (built on first sight) at the tick's pose, bodies sharing a
 * c_Shape in one draw (the batch number is the shape's slot); the colour by the config's mode
 */
static void r_InternalDrawShapes(const struct led *led)
{
	ProfZone;
	const struct ds_Dynamics *pipeline = &led->physics;
	const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
	const u32 depth_exponent = 1 + F32ExponentBits(led->cam.fz_far);
	for (u64 bi = 0; bi < pipeline->body_usage_set.block_count; ++bi)
	{
		struct ds_BitBlock it = ds_BitBlockInit(pipeline->body_usage_set.bits[bi], bi);
		while (ds_BitBlockHasNext(&it))
		{
			const struct ds_Body *body = pipeline->body_pool.buf + ds_BitBlockNext(&it);
			const struct ds_Shape *shape = NULL;
			for (i32 si = body->shape_list.first; si != DLL_SENTINEL; si = shape->body_shape.next)
			{
				shape = pipeline->shape_pool.buf + si;
				const v4 color = r_InternalShapeColor(pipeline, config, body, (u32) si);
				struct c_Shape *cshape = c_ShapeDBShape(pipeline->cshape_db, shape->cshape_handle);
				const struct r_Mesh *mesh = r_MeshShape(cshape, config->refinement);
				if (!mesh)
				{
					continue;
				}

				ds_TransformWorld transform;
				ds_ShapeWorldTransform(&transform, pipeline, shape);

				const f32 dist = V3Length(V3F64Demote(V3F64Sub(transform.position, led->cam.position)));
				const u32 unit_exponent = F32ExponentBits(dist);
				const u64 depth = (unit_exponent <= depth_exponent && unit_exponent > (depth_exponent - 23))
					? (0x00800000 | F32MantissaBits(dist)) >> (depth_exponent - unit_exponent + 1)
					: 0;
				const u64 transparency = (color.w == 1.0f) ? R_CMD_TRANSPARENCY_OPAQUE : R_CMD_TRANSPARENCY_ADDITIVE;
				const u64 material = r_MaterialConstructRaw(PROGRAM_PROXY3D, shape->cshape_handle, TEXTURE_NONE);
				const u64 elements = (mesh->index_data) ? R_CMD_ELEMENTS : R_CMD_ARRAYS;
				const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_INSTANCED, elements);

				struct r_Raw3d *raw = ArenaPush(g_scene->mem_frame, sizeof(struct r_Raw3d));
				if (!raw)
				{
					Log(T_RENDERER, S_ERROR, "shape %k: no frame memory for its instance, not drawn", &cshape->id);
					continue;
				}

				*raw = (struct r_Raw3d) { .mesh = mesh, .position = transform.position, .rotation = transform.rotation, .color = color, .blend = 1.0f };
				struct r_Instance *instance = r_InstanceAddNonCached(cmd);
				instance->type = R_INSTANCE_RAW3D;
				instance->raw3d = raw;

				/* a tri mesh's BVH: its lines, built once per shape, instanced with the body */
				const struct r_Mesh *bvh_mesh = (config->draw_mesh_bvh) ? r_MeshShapeBvh(cshape) : NULL;
				if (bvh_mesh)
				{
					const u64 transparency_bvh = (config->color_mesh_bvh.w == 1.0f) ? R_CMD_TRANSPARENCY_OPAQUE : R_CMD_TRANSPARENCY_ADDITIVE;
					const u64 cmd_bvh = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency_bvh, material, R_CMD_PRIMITIVE_LINE, R_CMD_INSTANCED, R_CMD_ELEMENTS);
					struct r_Raw3d *raw_bvh = ArenaPush(g_scene->mem_frame, sizeof(struct r_Raw3d));
					if (!raw_bvh)
					{
						Log(T_RENDERER, S_ERROR, "shape %k: no frame memory for its BVH instance, not drawn", &cshape->id);
						continue;
					}

					*raw_bvh = (struct r_Raw3d) { .mesh = bvh_mesh, .position = transform.position, .rotation = transform.rotation, .color = config->color_mesh_bvh, .blend = 1.0f };
					struct r_Instance *instance_bvh = r_InstanceAddNonCached(cmd_bvh);
					instance_bvh->type = R_INSTANCE_RAW3D;
					instance_bvh->raw3d = raw_bvh;
				}
			}
		}
	}
	ProfZoneEnd;
}

/*
 * The segments and points pushed during the tick (ds_DynamicsDraw: the pipeline's and every worker's) as one line
 * mesh and one point mesh on the renderer's frame arena, one non-cached instance each; skipped when there is
 * nothing to draw or the mesh does not fit. A force is a segment of force_scale metres per newton in its colour;
 * a point is one GL point of the config's point_size pixels. Positions are written relative to the view origin.
 */
/* an L_COLOR mesh of vertex_count vertices on the renderer's frame arena, NULL (logged) if it does not fit */
static struct r_Mesh *ColorMeshFrame(const u64 vertex_count, const char *what)
{
	struct arena *mem = &g_r_core->frame;
	ArenaPushRecord(mem);
	struct r_Mesh *mesh = ArenaPush(mem, sizeof(struct r_Mesh));
	u8 *vertex = ArenaPush(mem, vertex_count * L_COLOR_STRIDE);
	if (!mesh || !vertex)
	{
		ArenaPopRecord(mem);
		Log(T_RENDERER, S_ERROR, "%s: %lu vertices do not fit the renderer's frame arena, not drawn", what, vertex_count);
		return NULL;
	}
	ArenaRemoveRecord(mem);

	mesh->index_count = 0;
	mesh->index_max_used = 0;
	mesh->index_data = NULL;
	mesh->vertex_count = (u32) vertex_count;
	mesh->vertex_data = vertex;
	mesh->local_stride = L_COLOR_STRIDE;
	return mesh;
}

/* the HUD layer draws after the game layer without depth testing, before the UI (smaller depths sort later) */
static void ColorMeshDraw(const struct r_Mesh *mesh, const u64 primitive, const u32 on_top)
{
	const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
	const u64 depth = 0x7fffff;
	const u64 layer = (on_top) ? R_CMD_SCREEN_LAYER_HUD : R_CMD_SCREEN_LAYER_GAME;
	const u64 cmd = r_CommandKey(layer, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, primitive, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
	struct r_Instance *instance = r_InstanceAddNonCached(cmd);
	instance->type = R_INSTANCE_MESH;
	instance->mesh = (struct r_Mesh *) mesh;
}

static u8 *LineVertexWrite(u8 *vertex, const v3 p, const v4 color)
{
	memcpy(vertex, &p, sizeof(v3));
	memcpy(vertex + L_COLOR_OFFSET, &color, sizeof(v4));
	return vertex + L_COLOR_STRIDE;
}

static u8 *SegmentWrite(u8 *vertex, const v3 p0, const v3 p1, const v4 color)
{
	vertex = LineVertexWrite(vertex, p0, color);
	return LineVertexWrite(vertex, p1, color);
}

static v3 ViewRelative(const v3f64 p, const v3f64 origin)
{
	return V3F64Demote(V3F64Sub(p, origin));
}

static void r_InternalDrawSegments(const struct led *led)
{
	ProfZone;

	const struct ds_Dynamics *pipeline = &led->physics;
	const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
	const u32 source_count = pipeline->worker_count + 1;

	u64 segment_vertex_count = 0;
	u64 point_vertex_count = 0;
	for (u32 i = 0; i < source_count; ++i)
	{
		const struct ds_DynamicsDraw *draw = (i < pipeline->worker_count) ? &pipeline->worker[i].draw : &pipeline->draw;
		segment_vertex_count += 2*draw->segment.count + 2*draw->force.count;
		point_vertex_count += draw->point.count;
	}

	struct r_Mesh *segments = (segment_vertex_count) ? ColorMeshFrame(segment_vertex_count, "segments") : NULL;
	struct r_Mesh *points = (point_vertex_count) ? ColorMeshFrame(point_vertex_count, "points") : NULL;
	u8 *segment = (segments) ? segments->vertex_data : NULL;
	u8 *point = (points) ? points->vertex_data : NULL;
	const v3f64 origin = g_scene->view_origin;
	for (u32 i = 0; i < source_count; ++i)
	{
		const struct ds_DynamicsDraw *draw = (i < pipeline->worker_count) ? &pipeline->worker[i].draw : &pipeline->draw;

		if (segment)
		{
			for (u32 k = 0; k < draw->segment.count; ++k)
			{
				const struct r_SegmentWorld *s = draw->segment.buf + k;
				segment = SegmentWrite(segment, ViewRelative(s->p0, origin), ViewRelative(s->p1, origin), s->color);
			}

			for (u32 k = 0; k < draw->force.count; ++k)
			{
				const struct ds_ForceDraw *s = draw->force.buf + k;
				const v3 p = ViewRelative(s->origin, origin);
				segment = SegmentWrite(segment, p, V3AddScaled(p, s->force, config->force_scale), s->color);
			}
		}

		if (point)
		{
			for (u32 k = 0; k < draw->point.count; ++k)
			{
				const struct r_PointWorld *s = draw->point.buf + k;
				point = LineVertexWrite(point, ViewRelative(s->p, origin), s->color);
			}
		}
	}

	if (segments)
	{
		ds_Assert((u64) (segment - (u8 *) segments->vertex_data) == segment_vertex_count * L_COLOR_STRIDE);
		ColorMeshDraw(segments, R_CMD_PRIMITIVE_LINE, config->draw_on_top);
	}

	if (points)
	{
		ds_Assert((u64) (point - (u8 *) points->vertex_data) == point_vertex_count * L_COLOR_STRIDE);
		ColorMeshDraw(points, R_CMD_PRIMITIVE_POINT, config->draw_on_top);
	}
	ProfZoneEnd;
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

	const u32 depth_exponent = 1 + F32ExponentBits(led->cam.fz_far);
	ds_Assert(depth_exponent >= 23);

	g_scene->view_origin = led->cam.position;

	r_Proxy3dHierarchySpeculate(&g_r_core->frame, led->ns - led->ns_engine_paused);

	/* the proxies: off while the physics shapes are drawn in their place (z-fighting) */
	//    HII it;
	//    HIIInit(it, g_r_core->proxy3d_hierarchy, PROXY3D_ROOT);
	//	// skip root stub 
	//    HIIAdvance(it, g_r_core->proxy3d_hierarchy);
	//	while (it.at != PROXY3D_ROOT)
	//	{
	//		const u32 index = it.at;
	//        HIIAdvance(it, g_r_core->proxy3d_hierarchy);
	//		struct r_Proxy3d *proxy = r_Proxy3dAddress(index);
	//        if ((proxy->flags & PROXY3D_DRAW) == 0)
	//        {
	//            continue;
	//        }
	//
	//		const f32 dist = V3Length(V3F64Demote(V3F64Sub(proxy->spec_position, led->cam.position)));
	//		const u32 unit_exponent = F32ExponentBits(dist);
	//		const u64 depth = (unit_exponent <= depth_exponent && unit_exponent > (depth_exponent - 23))
	//			? (0x00800000 | F32MantissaBits(dist)) >> (depth_exponent - unit_exponent + 1)
	//			: 0;
	//
	//		const u64 transparency = (proxy->color.w == 1.0f)
	//			? R_CMD_TRANSPARENCY_OPAQUE
	//			: R_CMD_TRANSPARENCY_ADDITIVE;
	//
	//		const u64 material = r_MaterialConstruct(PROGRAM_PROXY3D, proxy->mesh, TEXTURE_NONE);
	//		const struct r_Mesh *r_mesh = led->render_mesh_db.pool.buf + proxy->mesh;
	//		const u64 command = (r_mesh->index_data)
	//			? r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_INSTANCED, R_CMD_ELEMENTS)
	//			: r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_INSTANCED, R_CMD_ARRAYS);
	//
	//		r_InstanceAdd(index, command);
	//	}

	/* the proxy trees, in world space: rebuilt every frame */
	const struct ds_DynamicsDrawConfig *draw_config = &led->physics.draw_config;
	const struct BVH *tree[2] = { &led->physics.dynamic_bvh, &led->physics.static_bvh };
	const u32 draw_tree[2] = { draw_config->draw_dbvh, draw_config->draw_sbvh };
	const v4 tree_color[2] = { draw_config->color_dbvh, draw_config->color_sbvh };
	for (u32 t = 0; t < 2; ++t)
	{
		if (!draw_tree[t])
		{
			continue;
		}

		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct r_Mesh *mesh = BVHMesh(&g_r_core->frame, tree[t], V3F64(0.0, 0.0, 0.0), QIdentity(), g_scene->view_origin, tree_color[t]);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}
	}

	if (led->physics.draw_config.draw_shapes)
	{
		r_InternalDrawShapes(led);
	}

	r_InternalDrawSegments(led);

	if (draw_config->draw_bounds)
	{
		const u64 material = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
		const u64 depth = 0x7fffff;
		const u64 cmd = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, depth, R_CMD_TRANSPARENCY_ADDITIVE, material, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
		struct r_Mesh *mesh = BoundingBoxesMesh(&g_r_core->frame, &led->physics, g_scene->view_origin, draw_config->color_bounds);
		if (mesh)
		{
			struct r_Instance *instance = r_InstanceAddNonCached(cmd);
			instance->type = R_INSTANCE_MESH;
			instance->mesh = mesh;
		}
	}
	ProfZoneEnd;
}

static void r_InternalProxy3dUniforms(const struct led *led, const u32 window)
{
	const struct r_Camera *cam = &led->cam;
	const m4 perspective = M4Perspective(cam->aspect_ratio, cam->fov_x, cam->fz_near, cam->fz_far);
	/* 3d draw data is uploaded relative to the camera (g_scene->view_origin): the view only rotates, and the light (at the camera) is at the origin */
	const m4 view = M4View(V3Zero(), cam->left, cam->up, cam->forward);
	
	ds_glUseProgram(g_r_core->program[PROGRAM_PROXY3D].gl_program);
	GLint aspect_ratio_addr, view_addr, perspective_addr, light_position_addr;
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "perspective");
	light_position_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_PROXY3D].gl_program, "light_position");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniform3f(light_position_addr, 0.0f, 0.0f, 0.0f);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, perspective.buf);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, view.buf);

	ds_glUseProgram(g_r_core->program[PROGRAM_LIGHTNING].gl_program);
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "perspective");
	light_position_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_LIGHTNING].gl_program, "light_position");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniform3f(light_position_addr, 0.0f, 0.0f, 0.0f);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, perspective.buf);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, view.buf);
	
	
	ds_glUseProgram(g_r_core->program[PROGRAM_COLOR].gl_program);
	aspect_ratio_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "aspect_ratio");
	view_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "view");
	perspective_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "perspective");
	const GLint point_size_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_COLOR].gl_program, "point_size");
	ds_glUniform1f(aspect_ratio_addr, (f32) cam->aspect_ratio);
	ds_glUniformMatrix4fv(perspective_addr, 1, GL_FALSE, perspective.buf);
	ds_glUniformMatrix4fv(view_addr, 1, GL_FALSE, view.buf);
	ds_glUniform1f(point_size_addr, led->physics.draw_config.point_size);
}

static void r_InternalUiUniforms(const u32 window)
{
	const v2u32 resolution = ds_WindowSize(window);

	ds_glUseProgram(g_r_core->program[PROGRAM_UI].gl_program);
	GLint resolution_addr = ds_glGetUniformLocation(g_r_core->program[PROGRAM_UI].gl_program, "resolution");
	ds_glUniform2f(resolution_addr, (f32) resolution.x, (f32) resolution.y);
}

static void r_SceneRender(const struct led *led, const u32 window)
{
	ProfZone;

	struct ds_Window *sys_win = ds_WindowAddress(window);
	ds_glViewport(0, 0, (i32) sys_win->size.x, (i32) sys_win->size.y); 

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
				const i32 texture_addr = ds_glGetUniformLocation(g_r_core->program[program].gl_program, "texture_atlas");
				ds_glUniform1i(texture_addr, tx_index);
				ds_glViewport(0, 0, (i32) sys_win->size.x, (i32) sys_win->size.y); 
			} break;

			case PROGRAM_LIGHTNING:
			case PROGRAM_COLOR:
			case PROGRAM_PROXY3D:
			{
				ds_glViewport(led->viewport_position.x
					       , led->viewport_position.y
					       , led->viewport_size.x
					       , led->viewport_size.y
					       ); 
			} break;
		}

		GLenum mode;
		switch (b->primitive)
		{
			case R_CMD_PRIMITIVE_LINE: { mode = GL_LINES; } break;
			case R_CMD_PRIMITIVE_TRIANGLE: { mode = GL_TRIANGLES; } break;
			case R_CMD_PRIMITIVE_POINT: { mode = GL_POINTS; } break;
			default:
			{
				/* no mode to draw with: skip the bucket */
				ds_AssertString(0, "Unexpected draw primitive");
				ProfZoneEnd;
				continue;
			}
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
					ds_glDrawArrays(mode, 0, (GLsizei) (buf->local_size / g_r_core->program[program].local_stride));
				}
				else
				{
					ds_glGenBuffers(1, &buf->shared_vbo);
					ds_glBindBuffer(GL_ARRAY_BUFFER, buf->shared_vbo);
					ds_glBufferData(GL_ARRAY_BUFFER, buf->shared_size, buf->shared_data, GL_STATIC_DRAW);
					g_r_core->program[program].buffer_shared_layout_setter();

					ds_glDrawArraysInstanced(mode, 0, (GLsizei) (buf->local_size / g_r_core->program[program].local_stride), buf->instance_count);
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
			//fprintf(stderr, "p   ns: %lu\n", led->ns_physics_start + led->ns_physics_elapsed);
			//fprintf(stderr, "p f ns: %lu\n", led->physics.frames_completed * led->physics.ns_tick);
			//fprintf(stderr, "spec:   %lu\n", led->ns - led->ns_engine_paused);

			struct ds_Window *win = NULL;

            HII it;
            HIIInit(it, *g_window_hierarchy, g_process_root_window);
            do
			{
				const i32 window = it.at;
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
                HIIAdvance(it, *g_window_hierarchy);
			}
            while (it.at != g_process_root_window);

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

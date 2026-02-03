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

#include <stdlib.h>
#include <string.h>

#include "r_local.h"
#define XXH_INLINE_ALL
#include "xxhash.h"

struct r_Scene *g_scene = NULL;

struct r_Scene *r_SceneAlloc(void)
{
	struct r_Scene *scene = malloc(sizeof(struct r_Scene));
	scene->mem_frame_arr[0] = ArenaAlloc(64*1024*1024);
	scene->mem_frame_arr[1] = ArenaAlloc(64*1024*1024);
	scene->mem_frame = scene->mem_frame_arr + 0;
	scene->frame = 0;

	scene->proxy3d_to_instance_map = HashMapAlloc(NULL, 4096, 4096, GROWABLE);
	scene->instance_pool = PoolAlloc(NULL, 4096, struct r_Instance, GROWABLE);

	scene->instance_new_list = ll_Init(struct r_Instance);

	scene->cmd_cache = NULL;
	scene->cmd_cache_count = 0;
	scene->cmd_frame = NULL;
	scene->cmd_frame_count = 0;

	return scene;
}

void r_SceneDealloc(struct r_Scene *scene)
{
	PoolDealloc(&scene->instance_pool);
	HashMapFree(&scene->proxy3d_to_instance_map);
	ArenaFree(scene->mem_frame_arr + 0),
	ArenaFree(scene->mem_frame_arr + 1),
	free(scene);
}

void r_SceneSetGlobal(struct r_Scene *scene)
{
	g_scene = scene;
}

void r_SceneFrameBegin(void)
{
	g_scene->frame += 1;
	g_scene->mem_frame = g_scene->mem_frame_arr + (g_scene->frame & 0x1);	
	
	ll_Flush(&g_scene->instance_new_list);
	g_scene->cmd_cache = g_scene->cmd_frame;
	g_scene->cmd_cache_count = g_scene->cmd_frame_count;
	g_scene->cmd_frame = NULL;
	g_scene->cmd_frame_count = 0;
	g_scene->frame_bucket_list = NULL;

	ArenaFlush(g_scene->mem_frame);
}

/* merge unit [left, mid-1], [mid, right-1] => tmp => copy back to unit */
static void r_InternalCommandMerge(struct r_Command *r_cmd, struct r_Command *tmp, const u32 left, const u32 mid, const u32 right)
{
	u32 l = left;
	u32 r = mid;
	const u32 count = right - left;

	for (u32 i = left; i < right; ++i)
	{
		if (r < right && (l >= mid || r_cmd[r].key > r_cmd[l].key))
		{
			tmp[i] = r_cmd[r];
			r += 1;
		}
		else
		{
			tmp[i] = r_cmd[l];
			l += 1;
		}
	}

	memcpy(r_cmd + left, tmp + left, count * sizeof(struct r_Command));
}

#ifdef DS_DEBUG

void r_SceneAssertCommandSorted(void)
{
	u32 sorted = 1;
	for (u32 i = 1; i < g_scene->cmd_frame_count; ++i)
	{
		//fprintf(stderr, "depth: %lu\n", R_CMD_DEPTH_GET(g_scene->cmd_frame[i-1].key));
		if (g_scene->cmd_frame[i-1].key < g_scene->cmd_frame[i].key)
		{
			//fprintf(stderr, "depth: %lu\n", R_CMD_DEPTH_GET(g_scene->cmd_frame[i].key));
			sorted = 0;	
			break;
		}
	}

	ds_AssertString(sorted, "r_scene ds_Assertion failed: draw commands not sorted");
}

void r_SceneAssertInstanceCommandBijection(void)
{
	for (u32 i = 0; i < g_scene->cmd_frame_count; ++i)
	{
		struct r_Command *cmd = g_scene->cmd_frame + i;
		struct r_Instance *instance = PoolAddress(&g_scene->instance_pool, cmd->instance);
		ds_Assert(PoolSlotAllocated(instance));
		ds_Assert(((u64) instance->cmd - (u64) cmd) == 0);
	}
}

#define R_SCENE_ASSERT_CMD_SORTED		r_SceneAssertCommandSorted()
#define	R_SCENE_ASSERT_INSTANCE_CMD_BIJECTION	r_SceneAssertInstanceCommandBijection()

#else

#define R_SCENE_ASSERT_CMD_SORTED		
#define	R_SCENE_ASSERT_INSTANCE_CMD_BIJECTION

#endif

static void r_scene_sort_commands_and_prune_instances(void)
{
	ProfZone;

	g_scene->cmd_frame = ArenaPush(g_scene->mem_frame, g_scene->cmd_frame_count * sizeof(struct r_Command));
	ArenaPushRecord(g_scene->mem_frame);
	struct r_Command *cmd_new = ArenaPush(g_scene->mem_frame, g_scene->instance_new_list.count*sizeof(struct r_Command));
	struct r_Command *cmd_tmp = ArenaPush(g_scene->mem_frame, g_scene->instance_new_list.count*sizeof(struct r_Command));
	struct r_Instance *instance = (struct r_Instance *) g_scene->instance_pool.buf;
	u32 index = g_scene->instance_new_list.first;
	for (u32 i = 0; i < g_scene->instance_new_list.count; ++i)
	{
		cmd_new[i] = *instance[index].cmd;
		index = instance[index].ll_next;
	}

	/* Sort newly added commands */
	for (u32 width = 2; width/2 < g_scene->instance_new_list.count; width *= 2)
	{
		u32 i = 0;
		for (; i + width <= g_scene->instance_new_list.count; i += width)
		{
			//fprintf(stderr, "merging       : [%u, %u] and [%u, %u]\n", i, i + width/2 - 1, i + width/2, i + width - 1);
			r_InternalCommandMerge(cmd_new, cmd_tmp, i, i + width/2, i + width);
		}

		if (i + width/2 < g_scene->instance_new_list.count)
		{
			r_InternalCommandMerge(cmd_new, cmd_tmp, i, i + width/2, g_scene->instance_new_list.count);
			//fprintf(stderr, "merging (last): [%u, %u] and [%u, %u]\n", i, i + width/2 - 1, i + width/2, add_count-1);
		}
	}

	/* (3) sort key_cache with new keys, remove any untouched instances */
	u32 cache_i = 0;
	u32 new_i = 0;
	for (u32 i = 0; i < g_scene->cmd_frame_count; ++i)
	{
		for (; cache_i < g_scene->cmd_cache_count; cache_i++) 
		{
			const u32 index = g_scene->cmd_cache[cache_i].instance;
			struct r_Instance *cached_instance = PoolAddress(&g_scene->instance_pool, index);
			if (cached_instance->frame_last_touched != g_scene->frame)
			{
				if (cached_instance->type == R_INSTANCE_PROXY3D)
				{
					HashMapRemove(&g_scene->proxy3d_to_instance_map, cached_instance->unit, index);
				}
				PoolRemove(&g_scene->instance_pool, index);
				g_scene->cmd_cache[cache_i].allocated = 0;
			}

			if (g_scene->cmd_cache[cache_i].allocated)
			{
				break;
			}
		}

		if (cache_i >= g_scene->cmd_cache_count) 
		{
			g_scene->cmd_frame[i] = cmd_new[new_i++];
		}
		else if (new_i >= g_scene->instance_new_list.count || g_scene->cmd_cache[cache_i].key >= cmd_new[new_i].key)
		{
			g_scene->cmd_frame[i] = g_scene->cmd_cache[cache_i++];
		}
		else
		{
			g_scene->cmd_frame[i] = cmd_new[new_i++];
		}

		struct r_Instance *instance = PoolAddress(&g_scene->instance_pool, g_scene->cmd_frame[i].instance);
		instance->cmd = g_scene->cmd_frame + i;
	}

	/* (4) remove any last untouched instances */
	for (; cache_i < g_scene->cmd_cache_count; cache_i++) 
	{
		const u32 index = g_scene->cmd_cache[cache_i].instance;
		struct r_Instance *cached_instance = PoolAddress(&g_scene->instance_pool, index);
		if (cached_instance->frame_last_touched != g_scene->frame)
		{
			if (cached_instance->type == R_INSTANCE_PROXY3D)
			{
				HashMapRemove(&g_scene->proxy3d_to_instance_map, cached_instance->unit, index);
			}
			PoolRemove(&g_scene->instance_pool, index);
			g_scene->cmd_cache[cache_i].allocated = 0;
		}
	}
	ArenaPopRecord(g_scene->mem_frame);
	R_SCENE_ASSERT_CMD_SORTED;
	R_SCENE_ASSERT_INSTANCE_CMD_BIJECTION;

	ProfZoneEnd;
}

void r_BufferConstructorReset(struct r_BufferConstructor *constructor)
{
	constructor->count = 0;
	constructor->first = NULL;
	constructor->last = NULL;
}

void r_BufferConstructorBufferAlloc(struct r_BufferConstructor *constructor, const u32 c_new_l)
{
	struct r_Buffer *buf = ArenaPush(g_scene->mem_frame, sizeof(struct r_Buffer));
	buf->next = NULL;
	buf->c_l = c_new_l;
	buf->local_size = 0;
	buf->shared_size = 0;
	buf->index_count = 0;
	buf->instance_count = 0;

	if (constructor->count == 0)
	{
		constructor->first = buf;
	}
	else
	{
		constructor->last->next = buf;
		constructor->last->c_h = c_new_l-1;
	}

	constructor->last = buf;
	constructor->count += 1;
}

void r_BufferConstructorBufferAddSize(struct r_BufferConstructor *constructor, const u64 local_size, const u64 shared_size, const u32 instance_count, const u32 index_count)
{
	ds_Assert(constructor->count);
	constructor->last->local_size += local_size;
	constructor->last->shared_size += shared_size;
	constructor->last->instance_count += instance_count;
	constructor->last->index_count += index_count;
}

struct r_Buffer **r_BufferConstructorFinish(struct r_BufferConstructor *constructor, const u32 c_h)
{
	struct r_Buffer **array = NULL;
	if (constructor->count)
	{
		array = ArenaPush(g_scene->mem_frame, constructor->count * sizeof(struct r_Buffer *));
		u32 i = 0;
		for (struct r_Buffer *buf = constructor->first; buf; buf = buf->next)
		{
			array[i++] = buf;
		}
		ds_Assert(i == constructor->count);
		array[i-1]->c_h = c_h;
	}

	return array;
}

static struct r_Bucket bucket_stub = 
{  
	.c_l = U32_MAX,
	.c_h = U32_MAX,
	.buffer_count = 0,
	.buffer_array = NULL,
};

void r_SceneBucketListGenerate(void)
{
	ProfZone;

	struct r_Bucket *start = &bucket_stub;
	struct r_Bucket *b = start;
	b->next = NULL;
	u32 begin_new_bucket = 1;

	struct r_BufferConstructor buf_constructor = { 0 };

	for (u32 i = 0; i < g_scene->cmd_frame_count; ++i)
	{
		struct r_Command *cmd = g_scene->cmd_frame + i;
		struct r_Instance *instance = PoolAddress(&g_scene->instance_pool, cmd->instance);

		/* TODO: can just compare u64 masked keys here... */
		if (b->transparency != R_CMD_TRANSPARENCY_GET(cmd->key)
				|| b->material != R_CMD_MATERIAL_GET(cmd->key)
				|| b->screen_layer != R_CMD_SCREEN_LAYER_GET(cmd->key)
				|| b->primitive != R_CMD_PRIMITIVE_GET(cmd->key)
				|| b->instanced != R_CMD_INSTANCED_GET(cmd->key))
		{
			begin_new_bucket = 1;
		}

		if (begin_new_bucket)
		{
			b->buffer_count = buf_constructor.count;
			b->buffer_array = r_BufferConstructorFinish(&buf_constructor, i-1);
			r_BufferConstructorReset(&buf_constructor);
			r_BufferConstructorBufferAlloc(&buf_constructor, i);

			begin_new_bucket = 0;
			b->c_h = i-1;
			b->next = ArenaPush(g_scene->mem_frame, sizeof(struct r_Bucket));
			ds_Assert(b->next);
			b = b->next;
			b->next = NULL;
			b->c_l = i;
			b->screen_layer = R_CMD_SCREEN_LAYER_GET(cmd->key);
			b->transparency = R_CMD_TRANSPARENCY_GET(cmd->key);
			b->material = R_CMD_MATERIAL_GET(cmd->key);
			b->primitive = R_CMD_PRIMITIVE_GET(cmd->key);
			b->instanced = R_CMD_INSTANCED_GET(cmd->key);
			b->elements = R_CMD_ELEMENTS_GET(cmd->key);
		}

		switch (instance->type)
		{ 
			case R_INSTANCE_UI: 
			{
				buf_constructor.last->index_count = 6;
				buf_constructor.last->local_size = 0;
				r_BufferConstructorBufferAddSize(&buf_constructor,
						  0,
					       	  instance->ui_bucket->count*S_UI_STRIDE, 
						  instance->ui_bucket->count,
						  0);	
			} break;

			case R_INSTANCE_PROXY3D:
			{
				const struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->unit);
				const struct r_Mesh *mesh = strdb_Address(g_r_core->mesh_database, proxy->mesh);
				buf_constructor.last->index_count = mesh->index_count;
				buf_constructor.last->local_size = mesh->vertex_count * L_PROXY3D_STRIDE;
				r_BufferConstructorBufferAddSize(&buf_constructor, 
						0,
						S_PROXY3D_STRIDE,
						1,
					       	0);
				
			} break;

			case R_INSTANCE_MESH:
			{
				r_BufferConstructorBufferAddSize(&buf_constructor,
						instance->mesh->vertex_count * instance->mesh->local_stride,
						0,
						0,
						0);
				ds_AssertMessage(buf_constructor.last->local_size <= 10000000, "ID: %k", &instance->mesh->id);
			} break;

			default:
			{
				ds_AssertString(0, "unexpected r_instance type in generate_bucket\n");
			} break;
		}
	}
	b->buffer_count = buf_constructor.count;
	b->buffer_array = r_BufferConstructorFinish(&buf_constructor, g_scene->cmd_frame_count-1);
	b->c_h = g_scene->cmd_frame_count-1;

	ProfZoneEnd;

	g_scene->frame_bucket_list = start->next;
}

static void r_scene_bucket_generate_draw_data(struct r_Bucket *b)
{
	ProfZone;

	const vec4 zero4 = { 0.0f, 0.0f, 0.0f, 0.0f };
	const vec3 zero3 = { 0.0f, 0.0f, 0.0f };

	const struct r_Command *r_cmd = g_scene->cmd_frame + b->c_l;
	const struct r_Instance *instance = PoolAddress(&g_scene->instance_pool, r_cmd->instance);

	for (u32 bi = 0; bi < b->buffer_count; bi++)
	{	
		struct r_Buffer *buf = b->buffer_array[bi];
		switch (instance->type)
		{
			case R_INSTANCE_UI:
			{
				buf->shared_data = ArenaPush(g_scene->mem_frame, buf->shared_size);
				buf->local_data = ArenaPush(g_scene->mem_frame, buf->local_size);
				buf->index_data = ArenaPush(g_scene->mem_frame, buf->index_count * sizeof(u32));

				u8 *shared_data = buf->shared_data;
				u8 *local_data = buf->local_data;

				buf->index_data[0] = 0;
				buf->index_data[1] = 1;
				buf->index_data[2] = 2;
				buf->index_data[3] = 0;
				buf->index_data[4] = 2;
				buf->index_data[5] = 3;

				for (u32 i = buf->c_l; i <= buf->c_h; ++i)
				{
					r_cmd = g_scene->cmd_frame + i;
					instance = PoolAddress(&g_scene->instance_pool, r_cmd->instance);

					const struct ui_DrawBucket *ui_b = instance->ui_bucket;
					struct ui_DrawNode *draw_node = ui_b->list;
					if (ui_CmdLayerGet(ui_b->cmd) == UI_CMD_LAYER_TEXT)
					{
						for (u32 i = 0; i < ui_b->count; )
						{
							const struct ui_Node *n = hi_Address(&g_ui->node_hierarchy, draw_node->index);
							draw_node = draw_node->next;
							const vec4 visible_rect =
							{
								(n->pixel_visible[AXIS_2_X].high + n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high + n->pixel_visible[AXIS_2_Y].low) / 2.0f,
								(n->pixel_visible[AXIS_2_X].high - n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high - n->pixel_visible[AXIS_2_Y].low) / 2.0f,
							};

							vec2 global_offset;
							switch (n->text_align_x)
							{
								case ALIGN_X_CENTER: 
								{ 
									global_offset[0] = n->pixel_position[0] + (n->pixel_size[0] - n->layout_text->width) / 2.0f; 
								} break;

								case ALIGN_LEFT: 
								{ 
									global_offset[0] = n->pixel_position[0] + n->text_pad[0]; 
								} break;

								case ALIGN_RIGHT: 
								{ 
									global_offset[0] = n->pixel_position[0] + n->pixel_size[0] - n->text_pad[0] - n->layout_text->width; 
								} break;
							}	

							switch (n->text_align_y)
							{
								case ALIGN_Y_CENTER: 
								{ 
									global_offset[1] = n->pixel_position[1] + (n->pixel_size[1] + n->font->linespace*n->layout_text->line_count) / 2.0f; 
								} break;

								case ALIGN_TOP: 
								{ 
									global_offset[1] = n->pixel_position[1] + n->pixel_size[1] - n->text_pad[1]; 
								} break;

								case ALIGN_BOTTOM: 
								{ 
									global_offset[1] = n->pixel_position[1] + n->font->linespace*n->layout_text->line_count + n->text_pad[1]; 
								} break;
							}

							global_offset[0] = f32_round(global_offset[0]);
							global_offset[1] = f32_round(global_offset[1]);

							struct textLine *line = n->layout_text->line;
							for (u32 l = 0; l < n->layout_text->line_count; ++l, line = line->next)
							{
								vec2 global_baseline =
								{
									global_offset[0],
									global_offset[1] - n->font->ascent - l*n->font->linespace,
								};
									
								i += line->glyph_count;
								for (u32 t = 0; t < line->glyph_count; ++t)
								{
									const struct fontGlyph *glyph = GlyphLookup(n->font, line->glyph[t].codepoint);
									const vec2 local_offset = 
									{ 
										global_baseline[0] + (f32) glyph->bearing[0] + line->glyph[t].x,
										global_baseline[1] + (f32) glyph->bearing[1],
									};

									const vec4 glyph_rect =
									{
										(2*local_offset[0] + (f32) glyph->size[0]) / 2.0f,
										(2*local_offset[1] - (f32) glyph->size[1]) / 2.0f,
										(f32) glyph->size[0] / 2.0f,
										(f32) glyph->size[1] / 2.0f,
									};	

									const vec4 uv_rect = 
									{
										(glyph->tr[0] + glyph->bl[0]) / 2.0f,
										(glyph->tr[1] + glyph->bl[1]) / 2.0f,
										(glyph->tr[0] - glyph->bl[0]) / 2.0f,
										(glyph->tr[1] - glyph->bl[1]) / 2.0f,
									};

									memcpy(shared_data + S_NODE_RECT_OFFSET, glyph_rect, sizeof(vec4));
									memcpy(shared_data + S_VISIBLE_RECT_OFFSET, visible_rect, sizeof(vec4));
									memcpy(shared_data + S_UV_RECT_OFFSET, uv_rect, sizeof(vec4));
									memcpy(shared_data + S_BACKGROUND_COLOR_OFFSET, zero4, sizeof(vec4));
									memcpy(shared_data + S_BORDER_COLOR_OFFSET, zero4, sizeof(vec4));
									memcpy(shared_data + S_SPRITE_COLOR_OFFSET, n->sprite_color, sizeof(vec4));
									memcpy(shared_data + S_EXTRA_OFFSET, zero3, sizeof(vec3));
									memset(shared_data + S_GRADIENT_COLOR_BR_OFFSET, 0, 4*sizeof(vec4));
									shared_data += S_UI_STRIDE;
								}
							}
						}
					}
					else if (ui_CmdLayerGet(ui_b->cmd) == UI_CMD_LAYER_TEXT_SELECTION)
					{
						for (u32 i = 0; i < ui_b->count; ++i)
						{
							const struct ui_TextSelection *sel = g_ui->frame_stack_text_selection.arr + draw_node->index;
							const struct ui_Node *n = sel->node;
							draw_node = draw_node->next;

							vec2 global_offset;
							switch (n->text_align_x)
							{
								case ALIGN_X_CENTER: 
								{ 
									global_offset[0] = n->pixel_position[0] + (n->pixel_size[0] - n->layout_text->width) / 2.0f; 
								} break;

								case ALIGN_LEFT: 
								{ 
									global_offset[0] = n->pixel_position[0] + n->text_pad[0]; 
								} break;

								case ALIGN_RIGHT: 
								{ 
									global_offset[0] = n->pixel_position[0] + n->pixel_size[0] - n->text_pad[0] - n->layout_text->width; 
								} break;
							}	

							switch (n->text_align_y)
							{
								case ALIGN_Y_CENTER: 
								{ 
									global_offset[1] = n->pixel_position[1] + (n->pixel_size[1] + n->font->linespace*n->layout_text->line_count) / 2.0f; 
								} break;

								case ALIGN_TOP: 
								{ 
									global_offset[1] = n->pixel_position[1] + n->pixel_size[1] - n->text_pad[1]; 
								} break;

								case ALIGN_BOTTOM: 
								{ 
									global_offset[1] = n->pixel_position[1] + n->font->linespace*n->layout_text->line_count + n->text_pad[1]; 
								} break;
							}

							global_offset[0] = f32_round(global_offset[0]);
							global_offset[1] = f32_round(global_offset[1]);

							struct textLine *line = sel->layout->line;
							ds_Assert(sel->layout->line_count == 1);
							ds_Assert(sel->high <= line->glyph_count + 1);

							const struct fontGlyph *glyph = GlyphLookup(n->font, (u32) ' ');
							f32 height = n->font->linespace;
							f32 width = glyph->advance;
							if (sel->low != sel->high)
							{
								width += line->glyph[sel->high-1].x - line->glyph[sel->low].x;	
							}

							if (0 < sel->low && sel->low <= line->glyph_count)
							{
								const struct fontGlyph *end_glyph = GlyphLookup(n->font, line->glyph[sel->low-1].codepoint);
								global_offset[0] += line->glyph[sel->low-1].x + end_glyph->advance;
							}

							const vec4 highlight_rect =
							{
								(2*global_offset[0] + width) / 2.0f,
								(2*global_offset[1] - height) / 2.0f,
								width / 2.0f,
								height / 2.0f,
							};	

							const vec4 visible_rect =
							{
								(n->pixel_visible[AXIS_2_X].high + n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high + n->pixel_visible[AXIS_2_Y].low) / 2.0f,
								(n->pixel_visible[AXIS_2_X].high - n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high - n->pixel_visible[AXIS_2_Y].low) / 2.0f,
							};
	
							const struct sprite *spr = g_sprite + n->sprite;
							const vec4 uv_rect = 
							{
								(spr->tr[0] + spr->bl[0]) / 2.0f,
								(spr->tr[1] + spr->bl[1]) / 2.0f,
								(spr->tr[0] - spr->bl[0]) / 2.0f,
								(spr->tr[1] - spr->bl[1]) / 2.0f,
							};

							memcpy(shared_data + S_NODE_RECT_OFFSET, highlight_rect, sizeof(vec4));
							memcpy(shared_data + S_VISIBLE_RECT_OFFSET, visible_rect, sizeof(vec4));
							memcpy(shared_data + S_UV_RECT_OFFSET, uv_rect, sizeof(vec4));
							memcpy(shared_data + S_BACKGROUND_COLOR_OFFSET, sel->color, sizeof(vec4));
							memcpy(shared_data + S_BORDER_COLOR_OFFSET, zero4, sizeof(vec4));
							memcpy(shared_data + S_SPRITE_COLOR_OFFSET, zero4, sizeof(vec4));
							memcpy(shared_data + S_EXTRA_OFFSET, zero3, sizeof(vec3));
							memset(shared_data + S_GRADIENT_COLOR_BR_OFFSET, 0, 4*sizeof(vec4));
							shared_data += S_UI_STRIDE;
						}
					}
					else
					{
						for (u32 i = 0; i < ui_b->count; ++i)
						{
							const struct ui_Node *n = hi_Address(&g_ui->node_hierarchy, draw_node->index);
							draw_node = draw_node->next;
							const struct sprite *spr = g_sprite + n->sprite;
							const vec4 node_rect =
							{
								n->pixel_position[0] + n->pixel_size[0] / 2.0f,
								n->pixel_position[1] + n->pixel_size[1] / 2.0f,
								n->pixel_size[0] / 2.0f,
								n->pixel_size[1] / 2.0f,
							};

							const vec4 visible_rect =
							{
								(n->pixel_visible[AXIS_2_X].high + n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high + n->pixel_visible[AXIS_2_Y].low) / 2.0f,
								(n->pixel_visible[AXIS_2_X].high - n->pixel_visible[AXIS_2_X].low) / 2.0f,
								(n->pixel_visible[AXIS_2_Y].high - n->pixel_visible[AXIS_2_Y].low) / 2.0f,
							};

							const vec4 uv_rect = 
							{
								(spr->tr[0] + spr->bl[0]) / 2.0f,
								(spr->tr[1] + spr->bl[1]) / 2.0f,
								(spr->tr[0] - spr->bl[0]) / 2.0f,
								(spr->tr[1] - spr->bl[1]) / 2.0f,
							};


							const vec3 extra = { n->border_size, n->corner_radius, n->edge_softness };
							memcpy(shared_data + S_NODE_RECT_OFFSET, node_rect, sizeof(vec4));
							memcpy(shared_data + S_VISIBLE_RECT_OFFSET, visible_rect, sizeof(vec4));
							memcpy(shared_data + S_UV_RECT_OFFSET, uv_rect, sizeof(vec4));
							memcpy(shared_data + S_BACKGROUND_COLOR_OFFSET, n->background_color, sizeof(vec4));
							memcpy(shared_data + S_BORDER_COLOR_OFFSET, n->border_color, sizeof(vec4));
							memcpy(shared_data + S_SPRITE_COLOR_OFFSET, n->sprite_color, sizeof(vec4));
							memcpy(shared_data + S_EXTRA_OFFSET, extra, sizeof(vec3));
							memcpy(shared_data + S_GRADIENT_COLOR_BR_OFFSET, n->gradient_color, 4*sizeof(vec4));
							shared_data += S_UI_STRIDE;
						}
					}
				}
			} break;

			case R_INSTANCE_PROXY3D:
			{
				const struct r_Proxy3d *proxy = r_Proxy3dAddress(instance->unit);
				const struct r_Mesh *mesh = strdb_Address(g_r_core->mesh_database, proxy->mesh);
				buf->shared_data = ArenaPush(g_scene->mem_frame, buf->shared_size);
				buf->local_data = mesh->vertex_data;
				buf->index_data = mesh->index_data;

				u8 *shared_data = buf->shared_data;
				for (u32 i = buf->c_l; i <= buf->c_h; ++i)
				{
					r_cmd = g_scene->cmd_frame + i;
					instance = PoolAddress(&g_scene->instance_pool, r_cmd->instance);
					proxy = r_Proxy3dAddress(instance->unit);

					memcpy(shared_data + S_PROXY3D_TRANSLATION_BLEND_OFFSET, proxy->spec_position, sizeof(vec3));
					memcpy(shared_data + S_PROXY3D_TRANSLATION_BLEND_OFFSET + sizeof(vec3), &proxy->blend, sizeof(f32));
					memcpy(shared_data + S_PROXY3D_ROTATION_OFFSET, proxy->spec_rotation, sizeof(quat));
					memcpy(shared_data + S_PROXY3D_COLOR_OFFSET, proxy->color, sizeof(vec4));
					shared_data += S_PROXY3D_STRIDE;
				}
			} break;

			case R_INSTANCE_MESH:
			{
				buf->shared_data = NULL;
				buf->index_data = NULL;
				buf->local_data = ArenaPush(g_scene->mem_frame, buf->local_size);
				u8 *local_data = buf->local_data;
				for (u32 i = buf->c_l; i <= buf->c_h; ++i)
				{
					r_cmd = g_scene->cmd_frame + i;
					instance = PoolAddress(&g_scene->instance_pool, r_cmd->instance);
					memcpy(local_data, instance->mesh->vertex_data, instance->mesh->vertex_count * instance->mesh->local_stride);
					local_data += instance->mesh->vertex_count * instance->mesh->local_stride;
				}
			} break;

			default:
			{
				ds_AssertString(0, "Unimplemented instance type in draw call generation");
			} break;
		}
	}

	ProfZoneEnd;
}

void r_SceneFrameEnd(void)
{
	ProfZone;

	r_scene_sort_commands_and_prune_instances();
	r_SceneBucketListGenerate();
	for (struct r_Bucket *b = g_scene->frame_bucket_list; b; b = b->next)
	{
		r_scene_bucket_generate_draw_data(b);
	}
	ProfZoneEnd;
}

struct r_Instance *r_InstanceAdd(const u32 unit, const u64 cmd)
{
	struct r_Instance *instance = NULL;
	const u32 hash = (u32) XXH3_64bits(&unit, sizeof(u32));

	u32 index = HashMapFirst(&g_scene->proxy3d_to_instance_map, hash);
	for (; index != HASH_NULL; index = HashMapNext(&g_scene->proxy3d_to_instance_map, index))
	{
		instance = PoolAddress(&g_scene->instance_pool, index);
		if (instance->unit == unit)
		{
			break;
		}
	}

	if (instance == NULL)
	{
		struct slot slot = PoolAdd(&g_scene->instance_pool);
		HashMapAdd(&g_scene->proxy3d_to_instance_map, hash, slot.index);
		ll_Prepend(&g_scene->instance_new_list, g_scene->instance_pool.buf, slot.index);

		instance = PoolAddress(&g_scene->instance_pool, slot.index);
		instance->unit = unit;
		instance->cmd = ArenaPush(g_scene->mem_frame, sizeof(struct r_Command));
		instance->cmd->key = cmd;
		instance->cmd->instance = slot.index;
		instance->cmd->allocated = 1;
	}
	else if (instance->cmd->key != cmd)
	{
		ll_Prepend(&g_scene->instance_new_list, g_scene->instance_pool.buf, index);

		instance->cmd->allocated = 0;
		instance->cmd = ArenaPush(g_scene->mem_frame, sizeof(struct r_Command));
		instance->cmd->key = cmd;
		instance->cmd->instance = index;
		instance->cmd->allocated = 1;
	}

	instance->frame_last_touched = g_scene->frame;
	instance->type = R_INSTANCE_PROXY3D;
	g_scene->cmd_frame_count += 1;
	
	return instance;
}

struct r_Instance *r_InstanceAddNonCached(const u64 cmd)
{
	struct slot slot = PoolAdd(&g_scene->instance_pool);
	ll_Prepend(&g_scene->instance_new_list, g_scene->instance_pool.buf, slot.index);

	struct r_Instance *instance = PoolAddress(&g_scene->instance_pool, slot.index);
	instance->cmd = ArenaPush(g_scene->mem_frame, sizeof(struct r_Command));
	instance->cmd->key = cmd;
	instance->cmd->instance = slot.index;
	instance->cmd->allocated = 1;
	instance->frame_last_touched = g_scene->frame;
	g_scene->cmd_frame_count += 1;
	
	return instance;
}

u64 r_MaterialConstruct(const u64 program, const u64 mesh, const u64 texture)
{
	ds_Assert(program <= (MATERIAL_PROGRAM_MASK >> MATERIAL_PROGRAM_LOW_BIT));
	ds_Assert(texture <= (MATERIAL_TEXTURE_MASK >> MATERIAL_TEXTURE_LOW_BIT));

	return (program << MATERIAL_PROGRAM_LOW_BIT) | (mesh << MATERIAL_MESH_LOW_BIT) | (texture << MATERIAL_TEXTURE_LOW_BIT);
}

u64 r_CommandKey(const u64 screen, const u64 depth, const u64 transparency, const u64 material, const u64 primitive, const u64 instanced, const u64 elements)
{
	ds_Assert(screen <= (((u64) 1 << R_CMD_SCREEN_LAYER_BITS) - (u64) 1));
	ds_Assert(depth <= (((u64) 1 << R_CMD_DEPTH_BITS) - (u64) 1));
	ds_Assert(transparency <= (((u64) 1 << R_CMD_TRANSPARENCY_BITS) - (u64) 1));
	ds_Assert(material <= (((u64) 1 << R_CMD_MATERIAL_BITS) - (u64) 1));
	ds_Assert(primitive <= (((u64) 1 << R_CMD_PRIMITIVE_BITS) - (u64) 1));
	ds_Assert(instanced <= (((u64) 1 << R_CMD_INSTANCED_BITS) - (u64) 1));
	ds_Assert(elements <= (((u64) 1 << R_CMD_ELEMENTS_BITS) - (u64) 1));

	return ((u64) screen << R_CMD_SCREEN_LAYER_LOW_BIT)		
	      	| ((u64) depth << R_CMD_DEPTH_LOW_BIT)			
	       	| ((u64) transparency << R_CMD_TRANSPARENCY_LOW_BIT)	
	       	| ((u64) material << R_CMD_MATERIAL_LOW_BIT)		
		| ((u64) primitive << R_CMD_PRIMITIVE_LOW_BIT)
		| ((u64) instanced << R_CMD_INSTANCED_LOW_BIT)
		| ((u64) elements << R_CMD_ELEMENTS_LOW_BIT);
}

const char *screen_str_table[1 << R_CMD_SCREEN_LAYER_BITS] =
{
	"SCREEN_LAYER_HUD",
	"SCREEN_LAYER_GAME",
};

const char *transparency_str_table[1 << R_CMD_TRANSPARENCY_BITS] =
{
	"TRANSPARENCY_NORMAL",
	"TRANSPARENCY_SUBTRACTIVE",
	"TRANSPARENCY_ADDITIVE",
	"TRANSPARENCY_OPAQUE",
};

const char *primitive_str_table[1 << R_CMD_PRIMITIVE_BITS] =
{
	"PRIMITIVE_TRIANGLE",
	"PRIMITIVE_LINE",
};

const char *instanced_str_table[1 << R_CMD_INSTANCED_BITS] =
{
	"NON_INSTANCED",
	"INSTANCED",
};

const char *elements_str_table[1 << R_CMD_ELEMENTS_BITS] =
{
	"ARRAYS",
	"ELEMENTS",
};

void r_CommandKeyPrint(const u64 key)
{
	const char *screen_str = screen_str_table[R_CMD_SCREEN_LAYER_GET(key)];
	const char *transparency_str = transparency_str_table[R_CMD_TRANSPARENCY_GET(key)];
	const char *primitive_str = primitive_str_table[R_CMD_PRIMITIVE_GET(key)];
	const char *instanced_str = instanced_str_table[R_CMD_INSTANCED_GET(key)];
	const char *elements_str = elements_str_table[R_CMD_ELEMENTS_GET(key)];

	fprintf(stderr, "render command key:\n");
	fprintf(stderr, "\tscreen: %s\n", screen_str);
	fprintf(stderr, "\tdepth: %lu\n", R_CMD_DEPTH_GET(key));
	fprintf(stderr, "\ttransparency: %s\n", transparency_str);
	fprintf(stderr, "\tmaterial: %lu\n", R_CMD_MATERIAL_GET(key));
	fprintf(stderr, "\tprimitive: %s\n", primitive_str);
	fprintf(stderr, "\tinstanced: %s\n", instanced_str);
	fprintf(stderr, "\tlayout: %s\n", elements_str);
}

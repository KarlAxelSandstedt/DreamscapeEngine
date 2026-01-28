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
#include "ui_public.h"

void r_ui_draw(struct ui *ui)
{
	const vec4 zero4 = { 0.0f, 0.0f, 0.0f, 0.0f };
	const vec3 zero3 = { 0.0f, 0.0f, 0.0f };

	struct ui_draw_bucket *b = PoolAddress(&ui->bucket_pool, ui->bucket_list.first);
	for (u32 i = dll_Next(b); i != DLL_NULL; i = dll_Next(b)) 
	{
		b = PoolAddress(&ui->bucket_pool, i);
		/* we reverse depth since in ui, larger depths goes infront, but in renderer lower depths drawn last */
		const u64 depth = ((1 << R_CMD_DEPTH_BITS) - 1) - (UI_CMD_DEPTH_GET(b->cmd) << UI_CMD_LAYER_BITS);
		const u64 transparency = (UI_CMD_DEPTH_GET(b->cmd) == UI_CMD_LAYER_TEXT_SELECTION)
			? R_CMD_TRANSPARENCY_SUBTRACTIVE
			: R_CMD_TRANSPARENCY_ADDITIVE;
		struct r_instance *instance = r_instance_add_non_cached(
				r_command_key(
					R_CMD_SCREEN_LAYER_HUD,
					depth + UI_CMD_LAYER_GET(b->cmd),	
					R_CMD_TRANSPARENCY_ADDITIVE,
					r_material_construct(PROGRAM_UI, MESH_NONE, UI_CMD_TEXTURE_GET(b->cmd)),
					R_CMD_PRIMITIVE_TRIANGLE,
					R_CMD_INSTANCED,
					R_CMD_ELEMENTS));
		instance->type = R_INSTANCE_UI;
		instance->ui_bucket = b;

		if (UI_CMD_LAYER_GET(b->cmd) == UI_CMD_LAYER_TEXT)
		{
			u64 total_glyph_count = 0;
			struct ui_draw_node *draw_node = instance->ui_bucket->list;
			for (u32 i = 0; i < instance->ui_bucket->count; ++i)
			{
				const struct ui_node *n = hi_Address(ui->node_hierarchy, draw_node->index);
				draw_node = draw_node->next;
				struct textLine *line = n->layout_text->line;
				for (u32 l = 0; l < n->layout_text->line_count; ++l)
				{
					total_glyph_count += line->glyph_count;
 					line = line->next;
				}
			}
			instance->ui_bucket->count = total_glyph_count;
		}
	}
}

void r_ui_buffer_shared_layout_setter(void)
{
	ds_glEnableVertexAttribArray(0);
	ds_glEnableVertexAttribArray(1);
	ds_glEnableVertexAttribArray(2);
	ds_glEnableVertexAttribArray(3);
	ds_glEnableVertexAttribArray(4);
	ds_glEnableVertexAttribArray(5);
	ds_glEnableVertexAttribArray(6);
	ds_glEnableVertexAttribArray(7);
	ds_glEnableVertexAttribArray(8);
	ds_glEnableVertexAttribArray(9);
	ds_glEnableVertexAttribArray(10);

	ds_glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_NODE_RECT_OFFSET);
	ds_glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_VISIBLE_RECT_OFFSET);
	ds_glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_UV_RECT_OFFSET);
	ds_glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_BACKGROUND_COLOR_OFFSET);
	ds_glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_BORDER_COLOR_OFFSET);
	ds_glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_SPRITE_COLOR_OFFSET);
	ds_glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_EXTRA_OFFSET);
	ds_glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_GRADIENT_COLOR_BR_OFFSET);
	ds_glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_GRADIENT_COLOR_TR_OFFSET);
	ds_glVertexAttribPointer(9, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_GRADIENT_COLOR_TL_OFFSET);
	ds_glVertexAttribPointer(10, 4, GL_FLOAT, GL_FALSE, S_UI_STRIDE, (void *)S_GRADIENT_COLOR_BL_OFFSET);

	ds_glVertexAttribDivisor(0, 1);
	ds_glVertexAttribDivisor(1, 1);
	ds_glVertexAttribDivisor(2, 1);
	ds_glVertexAttribDivisor(3, 1);
	ds_glVertexAttribDivisor(4, 1);
	ds_glVertexAttribDivisor(5, 1);
	ds_glVertexAttribDivisor(6, 1);
	ds_glVertexAttribDivisor(7, 1);
	ds_glVertexAttribDivisor(8, 1);
	ds_glVertexAttribDivisor(9, 1);
	ds_glVertexAttribDivisor(10, 1);
}

void r_ui_buffer_local_layout_setter(void)
{
}

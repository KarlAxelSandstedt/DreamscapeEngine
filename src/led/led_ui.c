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

#include "led_local.h"
#include "sys_public.h"

static void led_project_menu_ui(struct led *led, const struct ui_Visual *visual)
{
	struct led_project_menu *menu = &led->project_menu;

	system_window_set_global(menu->window);
	CmdQueueExecute();

	struct system_window *win = system_window_address(menu->window);
	ui_FrameBegin(win->size, visual);

	ui_TextAlignX(ALIGN_LEFT)
	ui_ChildLayoutAxis(AXIS_2_Y)
	ui_Parent(ui_NodeAllocF(UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "###window_%u", menu->window).index)
	ui_Flags(UI_DRAW_ROUNDED_CORNERS | UI_TEXT_ALLOW_OVERFLOW)
	ui_ChildLayoutAxis(AXIS_2_X)
	ui_Height(ui_SizePixel(32.0f, 1.0f))
	{
		ui_Pad();

		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###cur_fld_row").index)
		ui_Width(ui_SizePixel(20.0f, 1.0f))
		{
			ui_Sprite(SPRITE_LED_FOLDER)
			ui_NodeAllocF(UI_DRAW_SPRITE, "###cur_fld_spr");

			ui_Pad();

			ui_Width(ui_SizeText(F32_INFINITY, 0.0f))
			ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BORDER, "%k###cur_fld_path", &led->root_folder.path);

			ui_PadFill();
			ui_Pad();

			ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
			if (ui_ButtonF(UI_DRAW_TEXT | UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_DRAW_ROUNDED_CORNERS, "Refresh###ref") & UI_INTER_LEFT_CLICK)
			{
				menu->projects_folder_refresh = 1;
			}

			ui_Pad();
		}

		ui_Pad();

		const u32 file_count = menu->dir_nav.files.next;
		ui_Height(ui_SizePixel(20.0f, 1.0f))
		ui_list(&menu->dir_list, "###p", &menu->dir_list)
		for (u32 f = 0; f < file_count; ++f)
		{
			const struct file *file = VectorAddress(&menu->dir_nav.files, f);
			const enum spriteId spr = (file->type == FILE_DIRECTORY)
				? SPRITE_LED_FOLDER
				: SPRITE_LED_FILE;

			struct slot entry = ui_ListEntryAllocF(&menu->dir_list, "###%p_%u", &menu->dir_list, f);
			if (entry.address)
			ui_Parent(entry.index)
			{
				ui_Pad();

				ui_Sprite(spr)
				ui_Width(ui_SizePixel(20.0f, 1.0f))
				ui_NodeAllocNonHashed(UI_DRAW_BORDER | UI_DRAW_SPRITE);

				ui_Pad();

				ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
				ui_NodeAllocF(UI_DRAW_TEXT, "%k##%u", &file->path, f);
			}
		}
		ui_Pad();

		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "search_bar_row").index)
		{
			ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
			ui_NodeAllocF(UI_DRAW_TEXT, "search:##bar");
			
			const utf8 tmp = Utf8Inline("Text Window (TODO)");
			ui_Width(ui_SizeText(F32_INFINITY, 0.0f))
			ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BORDER, "%k##_bar", &tmp);
		}
		
		ui_Pad();

		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "buttons_row").index)
		ui_Width(ui_SizeText(F32_INFINITY, 0.0f))
		{
			ui_Pad();

			if ((ui_ButtonF(UI_DRAW_TEXT | UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_DRAW_ROUNDED_CORNERS, "New Project") & UI_INTER_LEFT_CLICK) && menu->popup_new_project.state == UI_POPUP_STATE_NULL)
			{
				ui_PopupUtf8Input(&menu->popup_new_project, &menu->utf8_new_project, &menu->input_line_new_project, Utf8Inline("Please enter the new project's name"), Utf8Inline("New Project:"), "New Project", visual);
			} 
			else if (menu->popup_new_project.state == UI_POPUP_STATE_PENDING_VERIFICATION)
			{
				ui_PopupTryDestroyAndSetToNull(&menu->popup_new_project_extra);
				menu->popup_new_project.state = UI_POPUP_STATE_COMPLETED;

				const char *error_string = NULL;

				if (menu->utf8_new_project.len == 0 || !led_filename_valid(menu->utf8_new_project))
				{
					error_string = "Invalid project name!";
				}
				else
				{
					struct system_window *project_window = system_window_address(led->window);
					enum fsError err;

					const char *cstr_project_name = CstrUtf8(g_ui->mem_frame, menu->utf8_new_project);
					err = DirectoryTryCreate(&project_window->mem_persistent, &led->project.folder, cstr_project_name, &led->root_folder);
					if (err != FS_SUCCESS)
					{
						switch (err)
						{
							case FS_ALREADY_EXISTS: { error_string = "Project already exists!"; } break;
							default: { error_string = "Unexpected error in creating project folder!"; } break;
						}
					}
					else
					{
						err = FileTryCreate(&project_window->mem_persistent, &led->project.file, cstr_project_name, &led->project.folder, !FILE_TRUNCATE);
						if (err != FS_SUCCESS)
						{
							switch (err)
							{
								default: { error_string = "Unexpected error in creating main project file!"; } break;
							}
						}
					}
				}
				
				if (error_string)
				{
					ui_PopupUtf8Display(&menu->popup_new_project_extra, Utf8Cstr(g_ui->mem_frame, error_string), "Error Message", visual);
					menu->popup_new_project.state = UI_POPUP_STATE_RUNNING;
				}
			}

			ui_Pad();

			if (ui_ButtonF(UI_DRAW_TEXT | UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_DRAW_ROUNDED_CORNERS, "Load") & UI_INTER_LEFT_CLICK)
			{
				fprintf(stderr, "Load!\n");
			}

			ui_Pad();

			if (ui_ButtonF(UI_DRAW_TEXT | UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_DRAW_ROUNDED_CORNERS, "Delete") & UI_INTER_LEFT_CLICK)
			{
				fprintf(stderr, "Delete!\n");
			}
		}
	}

	DsWindowEventHandler(win);
	ui_FrameEnd();
}

static void led_ui_test(struct led *led, const struct ui_Visual *visual)
{
	system_window_set_global(led->window);
	CmdQueueExecute();

	struct system_window *win = system_window_address(led->window);
	ui_FrameBegin(win->size, visual);

	ui_TextAlignX(ALIGN_LEFT)
	ui_ChildLayoutAxis(AXIS_2_Y)
	ui_Parent(ui_NodeAllocF(UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "###window_%u", led->window).index)
	ui_Flags(UI_DRAW_ROUNDED_CORNERS | UI_TEXT_ALLOW_OVERFLOW)
	ui_ChildLayoutAxis(AXIS_2_X)
	ui_Height(ui_SizePixel(32.0f, 1.0f))
	/* Testbed for ui features*/
	{
		win->cmd_console->visible = 1;
		ui_Width(ui_SizePerc(1.0f))
		if (win->cmd_console->visible)
		{
			ui_CmdConsoleF(win->cmd_console, "###console_%p", win->ui);
		};


		for (u32 r = 0; r < 5; ++r)
		{
			ui_Height(ui_SizePerc(0.1f))
			ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", r).index)
			{
				ui_SpriteColor(Vec4Inline(0.4f, 0.15f, 0.75f, 0.7f))
				ui_Sprite(SPRITE_LED_FOLDER)
				ui_BackgroundColor(Vec4Inline(204.0f/256.0f, 48.0f/256.0f, 110.0f/256.0f, 0.7f))
				ui_IntvViewableX(intv_inline(100.0f, 200.0f))	
				for (u32 i = 0; i <= 10; ++i)
				{
					ui_Width(ui_SizeUnit(intv_inline(95.0f + i*10.0f, 105.0f + i*10.0f)))
					ui_Height(ui_SizePerc(1.0f))
					(i % 2) 
						? ui_NodeAllocF(UI_DRAW_BACKGROUND | UI_DRAW_GRADIENT | UI_DRAW_BORDER | UI_DRAW_ROUNDED_CORNERS, "###box_%u_%u", r, i)
						: ui_NodeAllocF(UI_DRAW_SPRITE, "###box_%u_%u", r, i);
				}
			}
		}
	
		ui_Height(ui_SizePerc(0.1f))
		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", 6).index)
		{
			ui_Height(ui_SizePerc(1.0f))
			{
				for (u32 i = 0; i < 8; ++i)
				{
					ui_Width(ui_SizePixel(400, 1.0f / (2 << i)))
					ui_BackgroundColor(Vec4Inline((214.0f - i*30.0f)/256.0f, (48.0f + i*30.0f)/256.0f, (44.0f + i*30.0f)/256.0f, 0.7f))
					ui_NodeAllocF(UI_DRAW_BACKGROUND, "###box_%u_%u", 6, i);
				}

				ui_Width(ui_SizePixel(400, 1.0f / (2 << 8)))
				ui_BackgroundColor(Vec4Inline((204.0f- 8*20.0f)/256.0f, (48.0f + 8*20.0f)/256.0f, (110.0f + 8*10.0f)/256.0f, 0.7f))
				ui_NodeAllocF(UI_DRAW_BACKGROUND, "###box_%u_%u", 6, 8);
			}
		}
		
		ui_Height(ui_SizePerc(0.1f))
		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", 5).index)
		{
			ui_Height(ui_SizePerc(1.0f))
			ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", 7).index)
			{
				ui_Width(ui_SizePixel(80, 1.0f))
				ui_Height(ui_SizePixel(80, 1.0f))
				ui_FixedX(220.0f)
				ui_FixedY(220.0f)
				ui_BackgroundColor(Vec4Inline(0.1f, 0.3f, 0.6f, 0.7f))
				ui_NodeAllocF(UI_DRAW_BACKGROUND, "###box_%u_%u", 7, 0);
			}
		}

		ui_Height(ui_SizePerc(0.1f))
		ui_SpriteColor(Vec4Inline(1.0f, 1.0f, 1.0f, 1.0f))
		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", 8).index)
		{
			ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
			ui_Height(ui_SizePerc(1.0f))
			ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "text centering!###box_%u_%u", 8, 0);

			ui_Flags(UI_TEXT_ALLOW_OVERFLOW)
			ui_Width(ui_SizePixel(110.0f, 1.0f))
			ui_Height(ui_SizePerc(1.0f))
			{
				ui_TextAlignX(ALIGN_LEFT)
				ui_TextAlignY(ALIGN_BOTTOM)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "LB###box_%u_%u", 8, 1);
				ui_TextAlignX(ALIGN_LEFT)
				ui_TextAlignY(ALIGN_Y_CENTER)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "LC###box_%u_%u", 8, 2);
				ui_TextAlignX(ALIGN_LEFT)
				ui_TextAlignY(ALIGN_TOP)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "LT###box_%u_%u", 8, 3);
				ui_TextAlignX(ALIGN_X_CENTER)
				ui_TextAlignY(ALIGN_BOTTOM)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "CB###box_%u_%u", 8, 4);
				ui_TextAlignX(ALIGN_X_CENTER)
				ui_TextAlignY(ALIGN_Y_CENTER)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "CC###box_%u_%u", 8, 5);
				ui_TextAlignX(ALIGN_X_CENTER)
				ui_TextAlignY(ALIGN_TOP)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "CT###box_%u_%u", 8, 6);
				ui_TextAlignX(ALIGN_RIGHT)
				ui_TextAlignY(ALIGN_BOTTOM)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "RB###box_%u_%u", 8, 7);
				ui_TextAlignX(ALIGN_RIGHT)
				ui_TextAlignY(ALIGN_Y_CENTER)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "RC###box_%u_%u", 8, 8);
				ui_TextAlignX(ALIGN_RIGHT)
				ui_TextAlignY(ALIGN_TOP)
				ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "RT###box_%u_%u", 8, 9);
			}
		}

		ui_Height(ui_SizePerc(0.1f))
		ui_SpriteColor(Vec4Inline(1.0f, 1.0f, 1.0f, 1.0f))
		ui_Font(FONT_DEFAULT_SMALL)
		ui_Parent(ui_NodeAllocF(UI_FLAG_NONE, "###row_%u", 9).index)
		{
			ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
			ui_Height(ui_SizePerc(1.0f))
			ui_BackgroundColor(Vec4Inline(0.2f, 0.2, 0.4f, 0.7f))
			ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "###box_%u_%u", 9, 0);

			ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
			ui_Height(ui_SizePerc(1.0f))
			ui_BackgroundColor(Vec4Inline(0.2f, 0.2, 0.4f, 0.7f))
			ui_NodeAllocF(UI_DRAW_TEXT | UI_DRAW_BACKGROUND | UI_DRAW_BORDER, "awd###box_%u_%u", 9, 1);
		}
	}

	DsWindowEventHandler(win);
	ui_FrameEnd();
}

static void led_input_handler(struct led *led, struct ui_Node *viewport)
{
	Vec4Set(viewport->border_color, 0.9f, 0.9f, 0.9f, 1.0f);
	struct system_window *sys_win = system_window_address(led->window);

	for (u32 i = sys_win->ui->event_list.first; i != DLL_NULL; )
	{
		struct dsEvent *event = PoolAddress(&sys_win->ui->event_pool, i);
		const u32 next = dll_Next(event);
		u32 event_consumed = 1;
		if (event->type == DS_KEY_PRESSED)
		{
			switch (event->scancode)
			{
				case DS_MINUS:
				{
					led->ns_delta_modifier *= 0.8f;
				} break;

				case DS_PLUS:
				{
					led->ns_delta_modifier *= 1.25f;
				} break;

				case DS_R:
				{
					led->ns_delta_modifier *= 1.0f;
				} break;

				default:
				{
					event_consumed = 0;
				} break;
			}
		}

		if (event_consumed)
		{
			dll_Remove(&sys_win->ui->event_list, sys_win->ui->event_pool.buf, i);
			PoolRemove(&sys_win->ui->event_pool, i);
		}
		i = next;
	}

	if (sys_win->ui->inter.key_pressed[DS_W])
	{
	    	led->cam_forward_velocity += 9.0f; 
	} 

	if (sys_win->ui->inter.key_pressed[DS_S])
	{
	    	led->cam_forward_velocity -= 9.0f; 
	} 

	if (sys_win->ui->inter.key_pressed[DS_A])
	{
		led->cam_left_velocity += 9.0f; 
	} 

	if (sys_win->ui->inter.key_pressed[DS_D])
	{
	    	led->cam_left_velocity -= 9.0f; 
	} 

	r_camera_update_angles(&led->cam, -sys_win->ui->inter.cursor_delta[0] / 300.0f, -sys_win->ui->inter.cursor_delta[1] / 300.0f);
	r_camera_update_axes(&led->cam);

	sys_win->ui->inter.cursor_delta[0] = 0.0f;
	sys_win->ui->inter.cursor_delta[1] = 0.0f;
}

static void led_ui(struct led *led, const struct ui_Visual *visual)
{
	system_window_set_global(led->window);
	CmdQueueExecute();

	struct system_window *win = system_window_address(led->window);
	ui_FrameBegin(win->size, visual);

	static u32 count = 0;
	static u32 once = 1;

	if (once)
	{
		once = 0;

		led->node_ui_list = ui_ListInit(AXIS_2_Y, 256.0f, 24.0f, UI_SELECTION_MULTI); 
		led->node_selected_ui_list = ui_ListInit(AXIS_2_Y, 512.0f, 24.0f + 3*24.0f + 12.0f, UI_SELECTION_NONE);
		led->cs_list = ui_ListInit(AXIS_2_Y, 200.0f, 24.0f, UI_SELECTION_UNIQUE);
		led->cs_mesh_menu = ui_DropdownMenuInit(150.0f, Vec2Inline(110.0f, 24.0f), UI_DROPDOWN_BELOW);

		led->rb_prefab_list = ui_ListInit(AXIS_2_Y, 200.0f, 24.0f, UI_SELECTION_UNIQUE);
		led->rb_prefab_mesh_menu = ui_DropdownMenuInit(150.0f, Vec2Inline(110.0f, 24.0f), UI_DROPDOWN_ABOVE);
		
		led->rb_color_mode_menu = ui_DropdownMenuInit(120.0f, Vec2Inline(196.0f, 24.0f), UI_DROPDOWN_BELOW);
	}

	ui_TextAlignX(ALIGN_LEFT)
	ui_TextAlignY(ALIGN_BOTTOM)
	ui_ChildLayoutAxis(AXIS_2_X)
	ui_Parent(ui_NodeAllocF(UI_DRAW_BORDER, "###window_%u", led->window).index)
	{
		ui_ChildLayoutAxis(AXIS_2_Y)
		ui_Parent(ui_NodeAllocNonHashed(0).index)
		ui_Width(ui_SizePerc(1.0f))
		{
			ui_Height(ui_SizePixel(32.0f, 1.0f))
			ui_ChildLayoutAxis(AXIS_2_X)
			ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BACKGROUND | UI_DRAW_BORDER).index)
			{
				ui_PadFill();

				ui_BackgroundColor(Vec4Inline(0.0f, 0.125f, 0.125f, 1.0f))
				ui_Flags(UI_DRAW_BACKGROUND)
				{
					struct ui_Node *button;
					ui_Width(ui_SizePixel(32.0f, 1.0f))
					ui_Flags(UI_DRAW_SPRITE)
					ui_BackgroundColor(Vec4Inline(0.5f, 0.5f, 0.5f, 0.5f))
					ui_SpriteColor(Vec4Inline(0.0f, 0.0f, 0.0f, 0.1f))
					ui_Sprite(SPRITE_LED_PLAY)
					if (ui_ButtonF(UI_DRAW_BACKGROUND | UI_DRAW_SPRITE, "###play") & UI_INTER_LEFT_CLICK)
					{
						//ui_BackgroundColor(Vec4Inline(0.0f, 0.5f, 0.5f, 0.5f))
						CmdSubmitFormat(g_ui->mem_frame, "led_compile");
						CmdSubmitFormat(g_ui->mem_frame, "led_run");
					}
					
					ui_Pad();

					ui_Width(ui_SizePixel(32.0f, 1.0f))
					ui_Flags(UI_DRAW_SPRITE)
					ui_SpriteColor(Vec4Inline(0.0f, 0.0f, 0.0f, 0.1f))
					ui_Sprite(SPRITE_LED_PAUSE)
					if (ui_ButtonF(UI_DRAW_SPRITE, "###pause") & UI_INTER_LEFT_CLICK)
					{
						CmdSubmitFormat(g_ui->mem_frame, "led_pause");
					}

					ui_Pad();

					ui_Width(ui_SizePixel(32.0f, 1.0f))
					ui_Flags(UI_DRAW_SPRITE)
					ui_SpriteColor(Vec4Inline(0.0f, 0.0f, 0.0f, 0.1f))
					ui_Sprite(SPRITE_LED_STOP)
					if (ui_ButtonF(UI_DRAW_SPRITE, "###stop") & UI_INTER_LEFT_CLICK)
					{
						CmdSubmitFormat(g_ui->mem_frame, "led_stop");
					}
				}

				ui_PadFill();
			}

			ui_Height(ui_SizePerc(1.0f))
			ui_TextAlignY(ALIGN_TOP)
			{
				struct slot slot;
				const utf32 external_text = Utf32Cstr(g_ui->mem_frame, "Viewport");

				slot = ui_NodeAlloc(UI_DRAW_BORDER | UI_INTER_FLAGS, &led->viewport_id);
				if (slot.index != HI_ORPHAN_STUB_INDEX)
				ui_Parent(slot.index)
				{
					struct ui_Node *node = slot.address;
					if (node->inter & UI_INTER_HOVER)
					{
						vec3 dir; 
						const vec2 cursor_viewport_position =
						{
							g_ui->inter.cursor_position[0] - node->pixel_position[0],
							g_ui->inter.cursor_position[1] - node->pixel_position[1],
						};
						window_space_to_world_space(dir, cursor_viewport_position, node->pixel_size, &led->cam);
						Vec3TranslateScaled(dir, led->cam.position, -1.0f);
						Vec3ScaleSelf(dir, 1.0f / Vec3Length(dir));
						const struct ray ray = RayConstruct(led->cam.position, dir);
						const u32f32 hit = PhysicsPipelineRaycastParameter(g_ui->mem_frame, &led->physics, &ray);
						if (hit.f < F32_INFINITY)
						{
							const struct rigidBody *body = PoolAddress(&led->physics.body_pool, hit.u);	
							const struct led_node *entity = PoolAddress(&led->node_pool, body->entity);
							const char *body_id = CstrUtf8(g_ui->mem_frame, entity->id);


							ui_FixedX(g_ui->inter.cursor_position[0])
							ui_FixedY(g_ui->inter.cursor_position[1])
							ui_Width(ui_SizeText(128.0f, 1.0f))
							ui_Height(ui_SizePixel(24.0f, 1.0f))
							ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW | UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_SKIP_HOVER_SEARCH, "%k##%u", &entity->id, body->entity);
						}
					}

					if (node->inter & UI_INTER_FOCUS)
					{	
						const vec2 pos = 
						{
							node->pixel_position[0],
							node->pixel_position[1] + node->pixel_size[1],
						};
						cursor_set_rect(win, pos, node->pixel_size);
						led_input_handler(led, node);
					}

					if (node->inter & UI_INTER_FOCUS_IN)
					{
						cursor_lock(win);	
					}

					if (node->inter & UI_INTER_FOCUS_OUT)
					{
						cursor_unlock(win);	
					}
				}
			}

			u32 shape_selected = U32_MAX;
			ui_Height(ui_SizePixel(192.0f, 1.0f))
			ui_ChildLayoutAxis(AXIS_2_X)
			ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BACKGROUND | UI_DRAW_BORDER).index)
			ui_Height(ui_SizePerc(1.0f))
			{
				ui_Pad();

				ui_Width(ui_SizePixel(226.0f, 1.0f))
				ui_ChildLayoutAxis(AXIS_2_Y)
				ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
				ui_Height(ui_SizePixel(24.0f, 1.0f))
				ui_Width(ui_SizePixel(218.0f, 1.0f))
				{
					ui_Pad();

					utf8 new_shape_id;
					new_shape_id = ui_FieldUtf8F("Add Collision Shape...###new_shape");
					if (new_shape_id.len)
					{
						Utf8DebugPrint(new_shape_id);
						g_queue->cmd_exec->arg[0].utf8 = new_shape_id;
						CmdSubmitFormat(g_ui->mem_frame, "collision_shape_add \"%k\"", &new_shape_id);
					}

					ui_Pad();

					const struct collisionShape *shape;
					ui_list(&led->cs_list, "###%p", &led->cs_list)
					for (u32 i = led->cs_db.allocated_dll.first; i != DLL_NULL; i = strdb_Next(shape))
					{
						shape = strdb_Address(&led->cs_db, i);
						struct slot entry = ui_ListEntryAllocF(&led->cs_list, "###%p_%u", &led->cs_list, i);
						if (entry.index)
						ui_Parent(entry.index)
						{
							if (entry.index == led->cs_list.last_selected)
							{
								shape_selected = i;
							}
							ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "%k##%u", &shape->id, i);
						}
					}

					ui_PadFill();
				}

				ui_Width(ui_SizePixel(192.0f, 1.0f))
				ui_ChildLayoutAxis(AXIS_2_X)
				ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BORDER).index)
				if (led->cs_list.last_selection_happened == g_ui->frame)
				{
					ui_Pad();

					ui_ChildLayoutAxis(AXIS_2_Y)
					ui_Width(ui_SizePixel(180.0f, 1.0f))
					ui_Parent(ui_NodeAllocNonHashed(0).index)
					{
						ui_Pad();

						struct collisionShape *shape = strdb_Address(&led->cs_db, shape_selected);
						ui_Height(ui_SizePixel(24.0f, 1.0f))
						ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW | UI_DRAW_BORDER, "%k##shape_selected", &shape->id);

						ui_Pad();

						switch (shape->type)
						{
							case COLLISION_SHAPE_SPHERE:
							{
								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "type: SPHERE");

								ui_Pad();

								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_ChildLayoutAxis(AXIS_2_X)
								ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
								{
									ui_Width(ui_SizePixel(64.0f, 1.0f))
									ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "radius: ###sph_rad");

									ui_Width(ui_SizePerc(1.0f))
									shape->sphere.radius = ui_FieldF32F(shape->sphere.radius, intv_inline(0.0125f, 100.0f), "%f###sph_rad_in", shape->sphere.radius);
								}

							} break;

							case COLLISION_SHAPE_CAPSULE:
							{
								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "type: CAPSULE");

								ui_Pad();

								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_ChildLayoutAxis(AXIS_2_X)
								ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
								{
									ui_Width(ui_SizePixel(64.0f, 1.0f))
									ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "height: ###cap_height");
									ui_Width(ui_SizePerc(1.0f))
									shape->capsule.half_height = ui_FieldF32F(shape->capsule.half_height, intv_inline(0.0125f, 100.0f), "%f###cap_height_in", shape->capsule.half_height);
									//ui_ChildLayoutAxis(AXIS_2_Y)
									//ui_Height(ui_SizePerc(1.0f))
									//ui_Width(ui_SizePerc(1.0f))
									//ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
									//{
									//	shape->capsule.radius = ui_FieldF32F(shape->capsule.radius, intv_inline(0.0125f, 100.0f), "%f###cap_rad", shape->capsule.radius);
									//	shape->capsule.half_height = ui_FieldF32F(shape->capsule.half_height, intv_inline(0.0125f, 100.0f), "%f###cap_height", shape->capsule.half_height);

									//}
								}

								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_ChildLayoutAxis(AXIS_2_X)
								ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
								{
									ui_Width(ui_SizePixel(64.0f, 1.0f))
									ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "radius: ###cap_rad");
									ui_Width(ui_SizePerc(1.0f))
									shape->capsule.radius = ui_FieldF32F(shape->capsule.radius, intv_inline(0.0125f, 100.0f), "%f###cap_rad_in", shape->capsule.radius);
								}

							} break;

							case COLLISION_SHAPE_CONVEX_HULL:
							{
								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "type: CONVEX HULL");
							} break;
			
							case COLLISION_SHAPE_TRI_MESH:
							{
								ui_Height(ui_SizePixel(24.0f, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "type: TRIANGLE MESH");
							} break;

						}

						ui_PadFill();
					}

					ui_Pad();
				}

				ui_Pad();

				u32 prefab_selected = U32_MAX;
				ui_Width(ui_SizePixel(226.0f, 1.0f))
				ui_ChildLayoutAxis(AXIS_2_Y)
				ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BORDER).index)
				ui_Height(ui_SizePixel(24.0f, 1.0f))
				ui_Width(ui_SizePixel(218.0f, 1.0f))
				{
					ui_Pad();

					utf8 new_prefab_id;
					new_prefab_id = ui_FieldUtf8F("Add Rigid BodyBody Prefab...###new_prefab");
					if (new_prefab_id.len)
					{
						g_queue->cmd_exec->arg[0].utf8 = new_prefab_id;
						CmdSubmitFormat(g_ui->mem_frame, "rigid_body_prefab_add \"%k\" \"c_box\" 1.0 0.0 0.0 0", &new_prefab_id);
					}

					ui_Pad();

					const struct rigidBodyPrefab *prefab;
					ui_list(&led->rb_prefab_list, "###%p", &led->rb_prefab_list)
					for (u32 i = led->rb_prefab_db.allocated_dll.first; i != DLL_NULL; i = strdb_Next(prefab))
					{
						prefab = strdb_Address(&led->rb_prefab_db, i);
						struct slot entry = ui_ListEntryAllocF(&led->rb_prefab_list, "###%p_%u", &led->rb_prefab_list, i);
						if (entry.index)
						ui_Parent(entry.index)
						{
							if (entry.index == led->rb_prefab_list.last_selected)
							{
								prefab_selected = i;
							}
							ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "%k##%u", &prefab->id, i);
						}
					}

					ui_PadFill();
				}

				ui_Width(ui_SizePixel(256.0f, 1.0f))
				ui_ChildLayoutAxis(AXIS_2_X)
				ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BORDER).index)
				if (led->rb_prefab_list.last_selection_happened == g_ui->frame)
				{
					ui_Pad();

					ui_ChildLayoutAxis(AXIS_2_Y)
					ui_Width(ui_SizePixel(240.0f, 1.0f))
					ui_Parent(ui_NodeAllocNonHashed(0).index)
					{
						ui_Pad();

						struct rigidBodyPrefab *prefab = strdb_Address(&led->rb_prefab_db, prefab_selected);
						struct collisionShape *shape = NULL;
						ui_Height(ui_SizePixel(24.0f, 1.0f))
						ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW | UI_DRAW_BORDER, "%k##prefab_selected", &prefab->id);

						ui_Pad();

						const f32 density_prev = prefab->density;
						const u32 shape_prev = prefab->shape;

						ui_Height(ui_SizePixel(24.0f, 1.0f))
						ui_ChildLayoutAxis(AXIS_2_X)
						{
							ui_Parent(ui_NodeAllocNonHashed(0).index)
							{
								ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT, "density: ");
				
								ui_PadFill();

								ui_Flags(UI_DRAW_BORDER)
								ui_Width(ui_SizePixel(110.0f, 1.0f))
								prefab->density = ui_FieldF32F(prefab->density, intv_inline(0.00125f, 1000000.0f), "%f###s_density", prefab->density);
							}
							
							ui_Pad();

							ui_Parent(ui_NodeAllocNonHashed(0).index)
							{
								ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT, "restitution: ");
				
								ui_PadFill();

								ui_Flags(UI_DRAW_BORDER)
								ui_Width(ui_SizePixel(110.0f, 1.0f))
								prefab->restitution = ui_FieldF32F(prefab->restitution, intv_inline(0.0f, 1.0f), "%f###s_restitution", prefab->restitution);
							}
							
							ui_Pad();

							ui_Parent(ui_NodeAllocNonHashed(0).index)
							{
								ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT, "friction: ");
				
								ui_PadFill();

								ui_Flags(UI_DRAW_BORDER)
								ui_Width(ui_SizePixel(110.0f, 1.0f))
								prefab->friction = ui_FieldF32F(prefab->friction, intv_inline(0.0f, 1.0f), "%f###s_friction", prefab->friction);
							}
							
							ui_Pad();

							ui_Parent(ui_NodeAllocNonHashed(0).index)
							{
								ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT, "dynamic: ");
				
								ui_PadFill();

								ui_Flags(UI_DRAW_BORDER)
								ui_Width(ui_SizePixel(110.0f, 1.0f))
								prefab->dynamic = (u32) ui_FieldU64F(prefab->dynamic, intvu64_inline(0, 1), "%u###s_dynamic", prefab->dynamic);
							}

							ui_Pad();

							ui_Parent(ui_NodeAllocNonHashed(0).index)
							{
								ui_Width(ui_SizeText(F32_INFINITY, 1.0f))
								ui_NodeAllocF(UI_DRAW_TEXT, "shape: ");
				
								ui_PadFill();

								shape = strdb_Address(&led->cs_db, prefab->shape);
								ui_Width(ui_SizePixel(110.0f, 1.0f))
								if (ui_DropdownMenuF(&led->rb_prefab_mesh_menu, "%k###%p_sel", &shape->id, &led->rb_prefab_mesh_menu))
								{
									ui_DropdownMenuPush(&led->rb_prefab_mesh_menu);

									const struct collisionShape *s;
									for (u32 i = led->cs_db.allocated_dll.first; i != DLL_NULL; i = strdb_Next(s))
									{
										s = strdb_Address(&led->cs_db, i);
										struct ui_Node *drop;
										ui_Flags(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW)
										drop = ui_DropdownMenuEntryF(&led->rb_prefab_mesh_menu, "%k##%p_%u", &s->id, &led->rb_prefab_mesh_menu, i).address;
										if (drop->inter & UI_INTER_SELECT)
										{
											strdb_Dereference(&led->cs_db, prefab->shape);
											prefab->shape = strdb_Reference(&led->cs_db, s->id).index;
										}
									}

									ui_DropdownMenuPop(&led->rb_prefab_mesh_menu);
								}
							}
						}

						if (prefab->density != density_prev || prefab->shape != shape_prev)
						{
							PrefabStaticsSetup(prefab, shape, prefab->density);
						}

						ui_PadFill();
					}

					ui_Pad();
				}

				ui_ChildLayoutAxis(AXIS_2_Y)
				ui_Width(ui_SizePixel(230.0f, 1.0f))
				ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BACKGROUND).index)
				ui_Flags(UI_DRAW_ROUNDED_CORNERS | UI_TEXT_ALLOW_OVERFLOW)
				{
					if (ui_DropdownMenuF(&led->rb_color_mode_menu, "%s###%p_color_mode", body_color_mode_str[led->physics.body_color_mode], &led->rb_color_mode_menu))
					{
						ui_DropdownMenuPush(&led->rb_color_mode_menu);

						for (u32 i = 0; i < RB_COLOR_MODE_COUNT; ++i)
						{
							struct ui_Node *drop;
							ui_Flags(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW)
							drop = ui_DropdownMenuEntryF(&led->rb_color_mode_menu, "%s###%p_%u", 
									body_color_mode_str[i], 
									&led->rb_color_mode_menu, 
									i).address;
							if (drop->inter & UI_INTER_SELECT)
							{
								led->physics.pending_body_color_mode = i;
							}
						}

						ui_DropdownMenuPop(&led->rb_color_mode_menu);
					}

					ui_Pad(); 

					/* TODO: Make checkbox one-lines */
					/* TODO: Make center one-lines */
					struct ui_Node *node;
					ui_Height(ui_SizePixel(24.0f, 1.0f))
					ui_ChildLayoutAxis(AXIS_2_X)
					{
						ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
						{
							ui_Pad();
							ui_Width(ui_SizePixel(24.0f, 1.0f))
							node = ui_NodeAllocF(UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_INTER_LEFT_CLICK, "###draw_0").address;
							ui_Pad();
							ui_NodeAllocF(UI_DRAW_TEXT, "draw DBVT");
							if (node->inter & UI_INTER_LEFT_CLICK)
							{
								led->physics.draw_dbvh = !led->physics.draw_dbvh;
							}

							if (led->physics.draw_dbvh)
							{
								Vec4Set(node->background_color, 0.9f, 0.9f, 0.9f, 1.0f);
							}

							if (node->inter & UI_INTER_HOVER)
							{
								Vec4Set(node->background_color, 0.3f, 0.3f, 0.4f, 1.0f);
							}
						}

						ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
						{
							ui_Pad();
							ui_Width(ui_SizePixel(24.0f, 1.0f))
							node = ui_NodeAllocF(UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_INTER_LEFT_CLICK, "###draw_1").address;
							ui_Pad();
							ui_NodeAllocF(UI_DRAW_TEXT, "draw SBVT");
							if (node->inter & UI_INTER_LEFT_CLICK)
							{
								led->physics.draw_sbvh = !led->physics.draw_sbvh;
							}

							if (led->physics.draw_sbvh)
							{
								Vec4Set(node->background_color, 0.9f, 0.9f, 0.9f, 1.0f);
							}

							if (node->inter & UI_INTER_HOVER)
							{
								Vec4Set(node->background_color, 0.3f, 0.3f, 0.4f, 1.0f);
							}
						}

						ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
						{
							ui_Pad();
							ui_Width(ui_SizePixel(24.0f, 1.0f))
							node = ui_NodeAllocF(UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_INTER_LEFT_CLICK, "###draw_2").address;
							ui_Pad();
							ui_NodeAllocF(UI_DRAW_TEXT, "draw bounding boxes");
							if (node->inter & UI_INTER_LEFT_CLICK)
							{
								led->physics.draw_bounding_box = !led->physics.draw_bounding_box;
							}

							if (led->physics.draw_bounding_box)
							{
								Vec4Set(node->background_color, 0.9f, 0.9f, 0.9f, 1.0f);
							}

							if (node->inter & UI_INTER_HOVER)
							{
								Vec4Set(node->background_color, 0.3f, 0.3f, 0.4f, 1.0f);
							}
						}

						ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
						{
							ui_Pad();
							ui_Width(ui_SizePixel(24.0f, 1.0f))
							node = ui_NodeAllocF(UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_INTER_LEFT_CLICK, "###draw_3").address;
							ui_Pad();
							ui_NodeAllocF(UI_DRAW_TEXT, "draw collision manifolds");
							if (node->inter & UI_INTER_LEFT_CLICK)
							{
								led->physics.draw_manifold = !led->physics.draw_manifold;
							}

							if (led->physics.draw_manifold)
							{
								Vec4Set(node->background_color, 0.9f, 0.9f, 0.9f, 1.0f);
							}

							if (node->inter & UI_INTER_HOVER)
							{
								Vec4Set(node->background_color, 0.3f, 0.3f, 0.4f, 1.0f);
							}
						}

						ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
						{
							ui_Pad();
							ui_Width(ui_SizePixel(24.0f, 1.0f))
							node = ui_NodeAllocF(UI_DRAW_BORDER | UI_DRAW_BACKGROUND | UI_INTER_LEFT_CLICK, "###draw_4").address;
							ui_Pad();
							ui_NodeAllocF(UI_DRAW_TEXT, "draw debug lines");
							if (node->inter & UI_INTER_LEFT_CLICK)
							{
								led->physics.draw_lines = !led->physics.draw_lines;
							}

							if (led->physics.draw_lines)
							{
								Vec4Set(node->background_color, 0.9f, 0.9f, 0.9f, 1.0f);
							}

							if (node->inter & UI_INTER_HOVER)
							{
								Vec4Set(node->background_color, 0.3f, 0.3f, 0.4f, 1.0f);
							}
						}
					}


				
					//struct led_node *node = NULL;
					//ui_Height(ui_SizePixel(256.0f, 1.0f))
					//ui_list(&led->node_ui_list, "###%p", &led->node_ui_list)
					//for (u32 i = led->node_non_marked_list.first; i != DLL_NULL; i = dll_Next(node))
					//{
					//	node = GPoolAddress(&led->node_pool, i);
					//	ui_ChildLayoutAxis(AXIS_2_X)
					//	node->cache = ui_ListEntryAllocCached(&led->node_ui_list, 
					//			       	node->id,
					//				node->cache);

					//	if (node->cache.frame_node)
					//	ui_Parent(node->cache.index)
					//	{
					//		ui_Pad(); 

					//		ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "%k##%u", &node->id, i);
					//	}

					//	struct ui_Node *ui_node = node->cache.frame_node;
					//	if (ui_node->inter & UI_INTER_SELECT)
					//	{
					//		if (!dll2_InList(node))
					//		{
					//			dll_Append(&led->node_selected_list, led->node_pool.buf, i);
					//		}
					//	}
					//	else
					//	{
					//		if (dll2_InList(node))
					//		{
					//			dll_Remove(&led->node_selected_list, led->node_pool.buf, i);
					//		}
					//	}
					//}

					//ui_list(&led->node_selected_ui_list, "###%p", &led->node_selected_ui_list)
					//for (u32 i = led->node_selected_list.first; i != DLL_NULL; i = dll2_Next(node))
					//{
					//	node = GPoolAddress(&led->node_pool, i);
					//	ui_ChildLayoutAxis(AXIS_2_Y)
					//	ui_Parent(ui_ListEntryAllocF(&led->node_selected_ui_list, "###%p_%u", &led->node_selected_ui_list, i).index)
					//	{
					//		ui_Height(ui_SizePixel(24.0f, 1.0f))
					//		ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "%k##sel_%u", &node->id, i);
					//		ui_Height(ui_SizePixel(3*24.0f + 12.0f, 1.0f))
					//		ui_ChildLayoutAxis(AXIS_2_X)
					//		ui_Parent(ui_NodeAllocNonHashed(UI_DRAW_BORDER).index)
					//		ui_ChildLayoutAxis(AXIS_2_Y)
					//		{
					//			ui_TextAlignY(ALIGN_TOP)	
					//			ui_Width(ui_SizePixel(128.0f, 0.0f))
					//			ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
					//			ui_Height(ui_SizePixel(24.0f, 1.0f))
					//			{
					//				ui_PadPixel(6.0f);

					//				ui_Height(ui_SizePixel(3*24.0f, 1.0f))
					//				ui_NodeAllocF(UI_DRAW_TEXT | UI_TEXT_ALLOW_OVERFLOW, "position##%u", i);
					//				ui_PadPixel(6.0f);
					//			}

					//			ui_Parent(ui_NodeAllocNonHashed(UI_FLAG_NONE).index)
					//			ui_Height(ui_SizePixel(24.0f, 1.0f))
					//			{
					//				ui_PadPixel(6.0f);
					//				
					//				vec3 p;
					//				Vec3Copy(p, node->position);
					//				node->position[0] = ui_FieldF32F(node->position[0], intv_inline(-1000000.0f, 1000000.0f), "%f###field_x_%u", node->position[0], i);
					//				node->position[1] = ui_FieldF32F(node->position[1], intv_inline(-1000000.0f, 1000000.0f), "%f###field_y_%u", node->position[1], i);
					//				node->position[2] = ui_FieldF32F(node->position[2], intv_inline(-1000000.0f, 1000000.0f), "%f###field_z_%u", node->position[2], i);

					//				if (p[0] != node->position[0] || p[1] != node->position[1] || p[2] != node->position[2])
					//				{
					//					vec3 zero = { 0 };
					//					r_proxy3d_set_linear_speculation(node->position
					//							, node->rotation
					//							, zero 
					//							, zero 
					//							, led->ns
					//							, node->proxy);
					//	
					//				}


					//				ui_PadPixel(6.0f);
					//			}

					//			ui_PadFill();
					//		}
					//	}
					//}

					ui_PadFill();
				}
			}

			win->cmd_console->visible = 1;
			ui_Height(ui_SizePixel(32.0f, 1.0f))
			if (win->cmd_console->visible)
			{
				ui_CmdConsoleF(win->cmd_console, "###console_%p", win->ui);
			};
		}	
	}

	DsWindowEventHandler(win);
	ui_FrameEnd();

	struct ui_Node *node = ui_NodeLookup(&led->viewport_id).address;
	led->viewport_position[0] = node->pixel_position[0];
	led->viewport_position[1] = node->pixel_position[1];
	led->viewport_size[0] = node->pixel_size[0];
	led->viewport_size[1] = node->pixel_size[1];

	const f32 delta = (f32) led->ns_delta / NSEC_PER_SEC;
	led->cam.position[0] += delta * (led->cam_left_velocity * led->cam.left[0] + led->cam_forward_velocity * led->cam.forward[0]);
	led->cam.position[1] += delta * (led->cam_left_velocity * led->cam.left[1] + led->cam_forward_velocity * led->cam.forward[1]);
	led->cam.position[2] += delta * (led->cam_left_velocity * led->cam.left[2] + led->cam_forward_velocity * led->cam.forward[2]);
	led->cam.aspect_ratio =  (f32) led->viewport_size[0] / led->viewport_size[1];

	led->cam_left_velocity = 0.0f;
	led->cam_forward_velocity = 0.0f;

	if (win->tagged_for_destruction) 
	{
		led->running = 0;
	}
}

void led_ui_main(struct led *led)
{
	ProfZone;

	const vec4 bg = { 0.0625f, 0.0625f, 0.0625f, 1.0f };
	const vec4 br = { 0.0f, 0.15f, 0.25f, 1.0f };
	const vec4 gr[BOX_CORNER_COUNT] = 
	{
		{0.0f, 0.15f, 0.8f, 0.8f },
		{0.0f, 0.7f, 0.25f, 0.8f },
		{0.0f, 0.7f, 0.25f, 0.8f },
		{0.0f, 0.15f, 0.8f, 0.8f },
	};
	const vec4 sp = { 0.9f, 0.9f, 0.9f, 1.0f };

	const f32 pad = 8.0f;
	const f32 edge_softness = 0.0f;
	const f32 corner_radius = 3.0f;
	const f32 border_size = 1.0f;	
	const f32 text_pad_x = 4.0f;
	const f32 text_pad_y = 4.0f;

	const struct ui_Visual visual = ui_VisualInit(bg, br, gr, sp, pad, edge_softness, corner_radius, border_size, FONT_DEFAULT_SMALL, ALIGN_X_CENTER, ALIGN_Y_CENTER, text_pad_x, text_pad_y);

	//led_ui_test(led, &visual);
	led_ui(led, &visual);

	if (led->project_menu.window)
	{
		led_project_menu_ui(led, &visual);
	}

	ProfZoneEnd;
}

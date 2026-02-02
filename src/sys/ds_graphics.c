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

#include "ds_graphics.h"
#include "ui_public.h"
#include "sys_local.h"
//#include "r_public.h"

struct hi g_window_hierarchy_storage = { 0 };
struct hi *g_window_hierarchy = &g_window_hierarchy_storage;
u32 g_window = HI_NULL_INDEX;
u32 g_process_root_window = HI_NULL_INDEX;

static void system_window_free_resources(struct system_window *sys_win)
{
	//gl_state_free(sys_win->gl_state);
	ui_dealloc(sys_win->ui);
	//r_scene_free(sys_win->r_scene);
	CmdQueueDealloc(&sys_win->cmd_queue);
	ArenaFree1MB(&sys_win->mem_persistent);
	NativeWindowDestroy(sys_win->native);
}

u32 system_window_alloc(const char *title, const vec2u32 position, const vec2u32 size, const u32 parent)
{
	struct slot slot = hi_Add(g_window_hierarchy, parent);
	ds_Assert(parent != HI_ROOT_STUB_INDEX || slot.index == 2);

	struct system_window *sys_win = slot.address;

	sys_win->mem_persistent = ArenaAlloc1MB();
	sys_win->native = NativeWindowCreate(&sys_win->mem_persistent, (const char *) title, position, size);

	sys_win->ui = ui_alloc();
	//sys_win->r_scene = r_scene_alloc();
	sys_win->cmd_queue = CmdQueueAlloc();
	sys_win->cmd_console = ArenaPushZero(&sys_win->mem_persistent, sizeof(struct cmd_console));
	sys_win->cmd_console->prompt = ui_text_input_alloc(&sys_win->mem_persistent, 256);
	sys_win->tagged_for_destruction = 0;
	sys_win->text_input_mode = 0;
	
	//if (slot.index == 2)
	//{
	//	/* root window */
	//	NativeWindowGlSetCurrent(sys_win->native);
	//	sys_win->gl_state = gl_state_alloc();
	//	gl_state_set_current(sys_win->gl_state);
	//}
	//else
	//{
	//	/* set context before we initalize gl function pointers ***POSSIBLY*** Local to the new context on 
	//	 * som platforms */
	//	NativeWindowGlSetCurrent(sys_win->native);
	//	sys_win->gl_state = gl_state_alloc();

	//	struct system_window *root = hi_Address(g_window_hierarchy, g_process_root_window);
	//	NativeWindowGlSetCurrent(root->native);
	//}

	system_window_config_update(slot.index);

	return slot.index;
}

void system_window_tag_sub_hierarchy_for_destruction(const u32 root)
{
	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, g_window_hierarchy, root);
	while (it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		struct system_window *sys_win = hi_Address(g_window_hierarchy, index);
		sys_win->tagged_for_destruction = 1;
	}
	ArenaFree1MB(&tmp);
}



static void func_system_window_free(const struct hi *hi, const u32 index, void *data)
{
	struct system_window *win = hi_Address(hi, index);
	system_window_free_resources(win);
}

void system_free_tagged_windows(void)
{
	struct arena tmp1 = ArenaAlloc1MB();
	struct arena tmp2 = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp1, g_window_hierarchy, g_process_root_window);
	while (it.count)
	{
		const u32 index = hi_IteratorPeek(&it);
		struct system_window *sys_win = hi_Address(g_window_hierarchy, index);
		if (sys_win->tagged_for_destruction)
		{
			hi_IteratorSkip(&it);
			hi_ApplyCustomFreeAndRemove(&tmp2, g_window_hierarchy, index, func_system_window_free, NULL);

		}
		else
		{
			hi_IteratorNextDf(&it);
		}
	}
	ArenaFree1MB(&tmp1);
	ArenaFree1MB(&tmp2);
}

struct slot system_window_lookup(const u64 native_handle)
{
	struct system_window *win = NULL;
	u32 index = U32_MAX;

	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, g_window_hierarchy, g_process_root_window);
	while (it.count)
	{
		const u32 win_index = hi_IteratorNextDf(&it);
		struct system_window *sys_win = hi_Address(g_window_hierarchy, win_index);
		if (NativeWindowGetNativeHandle(sys_win->native) == native_handle)
		{
			win = sys_win;
			index = win_index;
			break;
		}
	}
	ArenaFree1MB(&tmp);

	return (struct slot) { .index = index, .address = win };
}

u32 system_process_root_window_alloc(const char *title, const vec2u32 position, const vec2u32 size)
{
	ds_Assert(g_process_root_window == HI_NULL_INDEX);
	g_process_root_window = system_window_alloc(title, position, size, HI_ROOT_STUB_INDEX);
	ds_Assert(g_process_root_window == 2);
	return g_process_root_window;
}

void system_window_config_update(const u32 window)
{
	struct system_window *sys_win = hi_Address(g_window_hierarchy, window);
	NativeWindowConfigUpdate(sys_win->position, sys_win->size, sys_win->native);
}

void system_window_size(vec2u32 size, const u32 window)
{
	struct system_window *sys_win = hi_Address(g_window_hierarchy, window);
	size[0] = sys_win->size[0];
	size[1] = sys_win->size[1];
}

struct system_window *system_window_address(const u32 index)
{
	return PoolAddress(&g_window_hierarchy->pool, index);
}

u32 system_window_index(const struct system_window *win)
{
	return PoolIndex(&g_window_hierarchy->pool, win);
}

void system_window_set_current_gl_context(const u32 window)
{
	struct system_window *sys_win = system_window_address(window);
	NativeWindowGlSetCurrent(sys_win->native);
	//gl_state_set_current(sys_win->gl_state);
}

void system_window_swap_gl_buffers(const u32 window)
{
	struct system_window *sys_win = system_window_address(window);
	NativeWindowGlSwapBuffers(sys_win->native);
}

void system_window_set_global(const u32 index)
{
	g_window = index;
	struct system_window *sys_win = hi_Address(g_window_hierarchy, index);
	ui_set(sys_win->ui);
	CmdQueueSet(&sys_win->cmd_queue);
}

void system_graphics_init(void)
{
#if __GAPI__ == __DS_SDL3__
	sdl3_WrapperInit();
#endif
	g_window_hierarchy_storage = hi_Alloc(NULL, 8, struct system_window, GROWABLE);
	
	//gl_state_list_alloc();
}

void system_graphics_destroy(void)
{
	struct arena tmp = ArenaAlloc1MB();
	hi_ApplyCustomFreeAndRemove(&tmp, g_window_hierarchy, g_process_root_window, func_system_window_free, NULL);
	ArenaFree1MB(&tmp);

	//gl_state_list_free();
	hi_Dealloc(g_window_hierarchy);
}

void system_window_text_input_mode_enable(void)
{
	struct system_window *sys_win = hi_Address(g_window_hierarchy, g_window);
	if (EnterTextInputMode(sys_win->native))
	{
		sys_win->text_input_mode = 1;
	}
	else
	{
		sys_win->text_input_mode = 0;
	}
}

void system_window_text_input_mode_disable(void)
{
	struct system_window *sys_win = hi_Address(g_window_hierarchy, g_window);
	if (ExitTextInputMode(sys_win->native))
	{
		sys_win->text_input_mode = 0;
	}
	else
	{
		sys_win->text_input_mode = 1;
	}
}

u32 cursor_is_locked(struct system_window *sys_win)
{
	return NativeCursorLockedCheck(sys_win->native);
}

u32 cursor_lock(struct system_window *sys_win)
{
	return NativeCursorLock(sys_win->native);
}

u32 cursor_unlock(struct system_window *sys_win)
{
	cursor_unset_rect(sys_win);
	return NativeCursorUnlock(sys_win->native);
}

u32 cursor_is_visible(struct system_window *sys_win)
{
	return NativeCursorVisibleCheck(sys_win->native);
}

void cursor_show(struct system_window *sys_win)
{
	NativeCursorShow(sys_win->native);
}

void cursor_hide(struct system_window *sys_win)
{
	NativeCursorHide(sys_win->native);
}

void cursor_set_rect(struct system_window *sys_win, const vec2 sys_position, const vec2 size)
{
	vec2 nat_pos;
	WindowPositionEngineToNative(nat_pos, sys_win->native, sys_position);
	NativeCursorSetRectangle(sys_win->native, nat_pos, size);
}

void cursor_unset_rect(struct system_window *sys_win)
{
	NativeCursorUnsetRectangle(sys_win->native);
}

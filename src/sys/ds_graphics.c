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
#include "r_public.h"

struct hi g_window_hierarchy_storage = { 0 };
struct hi *g_window_hierarchy = &g_window_hierarchy_storage;
u32 g_window = HI_NULL_INDEX;
u32 g_process_root_window = HI_NULL_INDEX;

static void ds_WindowDealloc(struct ds_Window *sys_win)
{
	gl_StateDealloc(sys_win->gl_state);
	CmdQueueDealloc(&sys_win->cmd_queue);
	r_SceneDealloc(sys_win->r_scene);
	ui_Dealloc(sys_win->ui);
	NativeWindowDestroy(sys_win->native);
	ArenaFree1MB(&sys_win->mem_persistent);
}

u32 ds_WindowAlloc(const char *title, const vec2u32 position, const vec2u32 size, const u32 parent)
{
	struct slot slot = hi_Add(g_window_hierarchy, parent);
	ds_Assert(parent != HI_ROOT_STUB_INDEX || slot.index == 2);

	struct ds_Window *sys_win = slot.address;

	sys_win->mem_persistent = ArenaAlloc1MB();
	sys_win->native = NativeWindowCreate(&sys_win->mem_persistent, (const char *) title, position, size);

	sys_win->ui = ui_Alloc();
	sys_win->r_scene = r_SceneAlloc();
	sys_win->cmd_queue = CmdQueueAlloc();
	sys_win->cmd_console = ArenaPushZero(&sys_win->mem_persistent, sizeof(struct ui_CmdConsole));
	sys_win->cmd_console->prompt = ui_TextInputAlloc(&sys_win->mem_persistent, 256);
	sys_win->tagged_for_destruction = 0;
	sys_win->text_input_mode = 0;
	
	NativeWindowGlSetCurrent(sys_win->native);
	sys_win->gl_state = gl_StateAlloc();
	if (slot.index == 2)
	{
		/* root window */
		gl_StateSetCurrent(sys_win->gl_state);
	}
	else
	{
		/* set context before we initalize gl function pointers ***POSSIBLY*** Local to the new context on 
		 * some platforms */
		struct ds_Window *root = hi_Address(g_window_hierarchy, g_process_root_window);
		NativeWindowGlSetCurrent(root->native);
	}

	ds_WindowConfigUpdate(slot.index);

	return slot.index;
}

void ds_WindowTagSubHierarchyForDestruction(const u32 root)
{
	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, g_window_hierarchy, root);
	while (it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		struct ds_Window *sys_win = hi_Address(g_window_hierarchy, index);
		sys_win->tagged_for_destruction = 1;
	}
	ArenaFree1MB(&tmp);
}

static void ds_InternalWindowDealloc(const struct hi *hi, const u32 index, void *data)
{
	struct ds_Window *win = hi_Address(hi, index);
	ds_WindowDealloc(win);
}

void ds_DeallocTaggedWindows(void)
{
	struct arena tmp1 = ArenaAlloc1MB();
	struct arena tmp2 = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp1, g_window_hierarchy, g_process_root_window);
	while (it.count)
	{
		const u32 index = hi_IteratorPeek(&it);
		struct ds_Window *sys_win = hi_Address(g_window_hierarchy, index);
		if (sys_win->tagged_for_destruction)
		{
			hi_IteratorSkip(&it);
			hi_ApplyCustomFreeAndRemove(&tmp2, g_window_hierarchy, index, ds_InternalWindowDealloc, NULL);

		}
		else
		{
			hi_IteratorNextDf(&it);
		}
	}
	ArenaFree1MB(&tmp1);
	ArenaFree1MB(&tmp2);
}

struct slot ds_WindowLookup(const u64 native_handle)
{
	struct ds_Window *win = NULL;
	u32 index = U32_MAX;

	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, g_window_hierarchy, g_process_root_window);
	while (it.count)
	{
		const u32 win_index = hi_IteratorNextDf(&it);
		struct ds_Window *sys_win = hi_Address(g_window_hierarchy, win_index);
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

u32 ds_RootWindowAlloc(const char *title, const vec2u32 position, const vec2u32 size)
{
	ds_Assert(g_process_root_window == HI_NULL_INDEX);
	g_process_root_window = ds_WindowAlloc(title, position, size, HI_ROOT_STUB_INDEX);
	ds_Assert(g_process_root_window == 2);
	return g_process_root_window;
}

void ds_WindowConfigUpdate(const u32 window)
{
	struct ds_Window *sys_win = hi_Address(g_window_hierarchy, window);
	NativeWindowConfigUpdate(sys_win->position, sys_win->size, sys_win->native);
}

void ds_WindowSize(vec2u32 size, const u32 window)
{
	struct ds_Window *sys_win = hi_Address(g_window_hierarchy, window);
	size[0] = sys_win->size[0];
	size[1] = sys_win->size[1];
}

struct ds_Window *ds_WindowAddress(const u32 index)
{
	return PoolAddress(&g_window_hierarchy->pool, index);
}

u32 ds_WindowIndex(const struct ds_Window *win)
{
	return PoolIndex(&g_window_hierarchy->pool, win);
}

void ds_WindowSetCurrentGlContext(const u32 window)
{
	struct ds_Window *sys_win = ds_WindowAddress(window);
	NativeWindowGlSetCurrent(sys_win->native);
	gl_StateSetCurrent(sys_win->gl_state);
}

void ds_WindowSwapGlBuffers(const u32 window)
{
	struct ds_Window *sys_win = ds_WindowAddress(window);
	NativeWindowGlSwapBuffers(sys_win->native);
}

void ds_WindowSetGlobal(const u32 index)
{
	g_window = index;
	struct ds_Window *sys_win = hi_Address(g_window_hierarchy, index);
	ui_Set(sys_win->ui);
	CmdQueueSet(&sys_win->cmd_queue);
}

void ds_GraphicsApiInit(void)
{
#if __GAPI__ == __DS_SDL3__
	sdl3_WrapperInit();
#endif
	ds_CmdApiInit();
	ds_UiApiInit();
	g_window_hierarchy_storage = hi_Alloc(NULL, 8, struct ds_Window, GROWABLE);
	
	gl_StatePoolAlloc();
}

void ds_GraphicsApiShutdown(void)
{
	struct arena tmp = ArenaAlloc1MB();
	hi_ApplyCustomFreeAndRemove(&tmp, g_window_hierarchy, g_process_root_window, ds_InternalWindowDealloc, NULL);
	ArenaFree1MB(&tmp);

	gl_StatePoolDealloc();
	hi_Dealloc(g_window_hierarchy);
	ds_CmdApiShutdown();
}

void ds_WindowTextInputModeEnable(void)
{
	struct ds_Window *sys_win = hi_Address(g_window_hierarchy, g_window);
	if (EnterTextInputMode(sys_win->native))
	{
		sys_win->text_input_mode = 1;
	}
	else
	{
		sys_win->text_input_mode = 0;
	}
}

void ds_WindowTextInputModeDisable(void)
{
	struct ds_Window *sys_win = hi_Address(g_window_hierarchy, g_window);
	if (ExitTextInputMode(sys_win->native))
	{
		sys_win->text_input_mode = 0;
	}
	else
	{
		sys_win->text_input_mode = 1;
	}
}

u32 ds_CursorLockedCheck(struct ds_Window *sys_win)
{
	return NativeCursorLockedCheck(sys_win->native);
}

u32 ds_CursorLock(struct ds_Window *sys_win)
{
	return NativeCursorLock(sys_win->native);
}

u32 ds_CursorUnlock(struct ds_Window *sys_win)
{
	ds_CursorUnsetRectangle(sys_win);
	return NativeCursorUnlock(sys_win->native);
}

u32 ds_CursorVisibleCheck(struct ds_Window *sys_win)
{
	return NativeCursorVisibleCheck(sys_win->native);
}

void ds_CursorShow(struct ds_Window *sys_win)
{
	NativeCursorShow(sys_win->native);
}

void ds_CursorHide(struct ds_Window *sys_win)
{
	NativeCursorHide(sys_win->native);
}

void ds_CursorSetRectangle(struct ds_Window *sys_win, const vec2 sys_position, const vec2 size)
{
	vec2 nat_pos;
	WindowPositionEngineToNative(nat_pos, sys_win->native, sys_position);
	NativeCursorSetRectangle(sys_win->native, nat_pos, size);
}

void ds_CursorUnsetRectangle(struct ds_Window *sys_win)
{
	NativeCursorUnsetRectangle(sys_win->native);
}

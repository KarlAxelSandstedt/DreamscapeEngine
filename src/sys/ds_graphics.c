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
#include "ds_ui.h"
#include "sys_local.h"
#include "ds_renderer.h"

HI_DEFINE(ds_Window);

struct ds_WindowHI g_window_hierarchy_storage = { 0 };
struct ds_WindowHI *g_window_hierarchy = &g_window_hierarchy_storage;
i32 g_window = HI_NULL;
i32 g_process_root_window = HI_NULL;

static void ds_WindowDealloc(struct ds_Window *sys_win)
{
	gl_StateDealloc(sys_win->gl_state);
	CmdQueueDealloc(&sys_win->cmd_queue);
	r_SceneDealloc(sys_win->r_scene);
	ui_Dealloc(sys_win->ui);
	NativeWindowDestroy(sys_win->native);
	ArenaFree(&sys_win->mem_persistent);
}

u32 ds_WindowAlloc(const char *title, const vec2u32 position, const vec2u32 size, const u32 parent)
{
	struct slot slot = ds_WindowHIAdd(g_window_hierarchy, parent);
	ds_Assert(parent != HI_ROOT || slot.index == 2);

	struct ds_Window *sys_win = slot.address;

	sys_win->mem_persistent = ArenaAlloc(NULL, 1024*1024);
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
		struct ds_Window *root = g_window_hierarchy->pool.buf + g_process_root_window;
		NativeWindowGlSetCurrent(root->native);
	}

	ds_WindowConfigUpdate(slot.index);

	return slot.index;
}

void ds_WindowTagSubHierarchyForDestruction(const u32 root)
{
    HII it;
    HIIInit(it, *g_window_hierarchy, root);
    do
	{
		const u32 index = it.at;
        HIIAdvance(it, *g_window_hierarchy);
		struct ds_Window *sys_win = g_window_hierarchy->pool.buf + index;
		sys_win->tagged_for_destruction = 1;
	}
    while (it.at != (i32) root);
}

static void ds_InternalWindowDealloc(const struct ds_WindowHI *hi, const u32 index, void *data)
{
	struct ds_Window *win = hi->pool.buf + index;
	ds_WindowDealloc(win);
}

void ds_DeallocTaggedWindows(void)
{
	struct arena *tmp = ArenaPushScratch();

    HII it;
    HIIInit(it, *g_window_hierarchy, g_process_root_window);
    do
	{
        const u32 index = it.at;
		struct ds_Window *sys_win = g_window_hierarchy->pool.buf + index;
		if (sys_win->tagged_for_destruction)
		{
            HIISkip(it, *g_window_hierarchy);
			ds_WindowHIApplyCustomFreeAndRemove(tmp, g_window_hierarchy, index, ds_InternalWindowDealloc, NULL);

		}
        HIIAdvance(it, *g_window_hierarchy);
	}
    while (it.at != g_process_root_window);

	ArenaPopScratch();
}

struct slot ds_WindowLookup(const u64 native_handle)
{
    struct slot slot = { .index = U32_MAX, .address = NULL };

    HII it;
    HIIInit(it, *g_window_hierarchy, g_process_root_window);
    do
	{
        struct ds_Window *sys_win = g_window_hierarchy->pool.buf + it.at;
		if (NativeWindowGetNativeHandle(sys_win->native) == native_handle)
		{
			slot.address = sys_win;
			slot.index = it.at;
			break;
		}
        HIIAdvance(it, *g_window_hierarchy);
	}
    while (it.at != g_process_root_window);

	return slot;
}

u32 ds_RootWindowAlloc(const char *title, const vec2u32 position, const vec2u32 size)
{
	ds_Assert(g_process_root_window == HI_NULL);
	g_process_root_window = ds_WindowAlloc(title, position, size, HI_ROOT);
	ds_Assert(g_process_root_window == 2);
	return g_process_root_window;
}

void ds_WindowConfigUpdate(const u32 window)
{
	struct ds_Window *sys_win = g_window_hierarchy->pool.buf + window;
	NativeWindowConfigUpdate(sys_win->position, sys_win->size, sys_win->native);
}

void ds_WindowSize(vec2u32 size, const u32 window)
{
	struct ds_Window *sys_win = g_window_hierarchy->pool.buf + window;
	size[0] = sys_win->size[0];
	size[1] = sys_win->size[1];
}

struct ds_Window *ds_WindowAddress(const u32 index)
{
	return g_window_hierarchy->pool.buf + index;
}

u32 ds_WindowIndex(const struct ds_Window *win)
{
	return ds_WindowPoolIndex(&g_window_hierarchy->pool, win);
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
	struct ds_Window *sys_win = g_window_hierarchy->pool.buf + index;
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
	g_window_hierarchy_storage = ds_WindowHIAlloc(NULL, 8, GROWABLE);
	
	gl_StateMemAlloc();
}

void ds_GraphicsApiShutdown(void)
{
	struct arena *tmp = ArenaPushScratch();
	ds_WindowHIApplyCustomFreeAndRemove(tmp, g_window_hierarchy, g_process_root_window, ds_InternalWindowDealloc, NULL);
    ArenaPopScratch();

	gl_StateMemDealloc();
	ds_WindowHIDealloc(g_window_hierarchy);
	ds_CmdApiShutdown();
}

void ds_WindowTextInputModeEnable(void)
{
	struct ds_Window *sys_win = g_window_hierarchy->pool.buf + g_window;
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
	struct ds_Window *sys_win = g_window_hierarchy->pool.buf + g_window;
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

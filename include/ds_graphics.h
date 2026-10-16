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

/*
ds_graphics.h
=============
The process's windows. A ds_Window owns a native window with its own GL context (sharing the root window's
objects), a UI, a command queue and console, a render scene and a 1 MB persistent arena. Windows form a
hierarchy (ds_hierarchy.h): the process root window is HI_ROOT's only child, every other window a child of
the window it was opened from, and freeing a window frees its sub-hierarchy. Windows are referred to by
index. Not thread-safe (main thread).
*/

#ifndef __DS_GRAPHICS_H__
#define __DS_GRAPHICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_base.h"
#include "ds_hierarchy.h"
#include "cmd.h"

/*
 * system window coordinate system:
 *
 *  (0,Y) ------------------------- (X,Y)
 *    |				      |
 *    |				      |
 *    |				      |
 *    |				      |
 *    |				      |
 *  (0,0) ------------------------- (X,0)
 *
 *  Since we are using a right handed coordinate system to describe the world, and the camera
 *  looks down the +Z axis, an increase in X or Y in the screen space means an "increase" from
 *  the camera's perspective as well. We must ensure that the underlying platform events that
 *  contains window coordinates are translated into this format.
 *
 *		A (Y)
 *		|
 *		|	(X)
 *		|------->
 *	       /
 *            /
 *	     V (Z)
 */

struct r_Scene;
struct nativeWindow;

/* Set up the window hierarchy and the GL state memory (and SDL). ds_Init calls it unless headless. */
void 	ds_GraphicsApiInit(void);
/* Free every window, the GL state memory and the hierarchy (and quit SDL). ds_Shutdown calls it unless headless. */
void 	ds_GraphicsApiShutdown(void);

/*
 * g_window_hierarchy: all windows. g_process_root_window: the root window's index; HI_NULL before
 * ds_RootWindowAlloc and after the root window is freed. g_window: the current window (ds_WindowSetGlobal).
 */
extern struct HI(ds_Window) *	g_window_hierarchy;
extern i32 		g_process_root_window;
extern i32 		g_window;

typedef struct ds_Window
{
	HI_NODE;
	struct nativeWindow *	native;			/* native graphics handle */
	struct ui *		ui;			/* local ui */
	struct cmdQueue 	cmd_queue;		/* local command queue */
	struct ui_CmdConsole *	cmd_console;		/* console */
	struct r_Scene *	r_scene;
	struct arena 		mem_persistent;		/* persistent 1MB arena, freed with the window */

	u32			tagged_for_destruction; /* If tagged, freed by the next ds_DeallocTaggedWindows */
	u32			text_input_mode;	/* If on, window is receiving text input events */
	v2u32			position;		/* native position, as of the last ds_WindowConfigUpdate */
	v2u32			size;			/* size in pixels, as of the last ds_WindowConfigUpdate */

	u32			gl_state;		/* the window's gl_State index (r_gl.c) */
} ds_Window;
DEFINE_HI(ds_Window);

/*
 * Open a window as parent's last child and return its index (fatal on failure). parent is a window; only
 * the root window goes under HI_ROOT (ds_RootWindowAlloc). position is the window's top-left corner on
 * screen (native coordinates; ignored on the web and where the OS refuses it). The root window's GL context
 * is current afterwards.
 */
u32 			ds_WindowAlloc(const char *title, const v2u32 position, const v2u32 size, const u32 parent);
/* Open the process root window (once): it becomes g_process_root_window, its GL context and state current. */
u32 			ds_RootWindowAlloc(const char *title, const v2u32 position, const v2u32 size);
/* Handle the debug keys among sys_win's UI events: L cursor lock, F10 cursor visibility, F11 fullscreen, F12 border, C close. */
void 			ds_WindowEventHandler(struct ds_Window *sys_win);
/*
 * Tag window root and its sub-hierarchy for destruction. The windows live until the next
 * ds_DeallocTaggedWindows; whoever holds their indices checks tagged_for_destruction and lets go.
 */
void 			ds_WindowTagSubHierarchyForDestruction(const u32 root);
/*
 * Free every tagged window with its sub-hierarchy (call at the start of a frame). Freeing the root
 * window frees all windows and sets g_process_root_window and g_window to HI_NULL.
 */
void			ds_DeallocTaggedWindows(void);
/* A window's index. */
u32			ds_WindowIndex(const struct ds_Window *sys_win);
/* The window at index. */
struct ds_Window *	ds_WindowAddress(const u32 index);
/*
 * The window whose native handle (SDL: its SDL_Window pointer) is native_handle: { address, index };
 * none (also without a root window): { NULL, U32_MAX }.
 */
struct slot		ds_WindowLookup(const u64 native_handle);
/* Make window current: g_window, and its UI (ui_Set) and command queue (CmdQueueSet). */
void			ds_WindowSetGlobal(const u32 window);
/* Start or stop text input events for g_window; its text_input_mode says whether they are on afterwards. */
void 			ds_WindowTextInputModeEnable(void);
void 			ds_WindowTextInputModeDisable(void);
/* Make window's GL context and GL state current. */
void			ds_WindowSetCurrentGlContext(const u32 window);
/* Present window's back buffer. */
void 			ds_WindowSwapGlBuffers(const u32 window);
/* Read window's native position and size into its ds_Window. */
void			ds_WindowConfigUpdate(const u32 window);
/* window's size as of the last ds_WindowConfigUpdate. */
v2u32			ds_WindowSize(const u32 window);


/*
 * Confine the cursor to the rectangle of the given size whose top-left corner is sys_position, in window
 * coordinates (above); ds_CursorUnsetRectangle releases it.
 */
void 			ds_CursorSetRectangle(struct ds_Window *sys_win, const v2 sys_position, const v2 size);
void 			ds_CursorUnsetRectangle(struct ds_Window *sys_win);
/* 1 if the cursor is locked to sys_win (hidden, reporting only motion), 0 otherwise. */
u32  			ds_CursorLockedCheck(struct ds_Window *sys_win);
/* Lock the cursor to sys_win; 1 on success, 0 otherwise. */
u32 			ds_CursorLock(struct ds_Window *sys_win);
/* Release the cursor rectangle and unlock the cursor; 1 on success, 0 otherwise. */
u32 			ds_CursorUnlock(struct ds_Window *sys_win);
/* 1 if the cursor is visible, 0 otherwise. */
u32 			ds_CursorVisibleCheck(struct ds_Window *sys_win);
void 			ds_CursorShow(struct ds_Window *sys_win);
void 			ds_CursorHide(struct ds_Window *sys_win);

#ifdef __cplusplus
}
#endif

#endif

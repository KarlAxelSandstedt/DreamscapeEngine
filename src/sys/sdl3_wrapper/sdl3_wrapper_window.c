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

#include "sdl3_wrapper_local.h"

struct nativeWindow
{
	SDL_Window *	sdl_win;
	SDL_GLContext	gl_context;	
};

/* GLOBAL FUNCTION POINTERS */
struct nativeWindow *	(*NativeWindowCreate)(struct arena *mem, const char *title, const vec2u32 position, const vec2u32 size);
void 			(*NativeWindowDestroy)(struct nativeWindow *native);

u64 			(*NativeWindowGetNativeHandle)(const struct nativeWindow *native);

void 			(*NativeWindowGlSetCurrent)(struct nativeWindow *native);
void 			(*NativeWindowGlSwapBuffers)(struct nativeWindow *native);

void 			(*NativeWindowConfigUpdate)(vec2u32 position, vec2u32 size, struct nativeWindow *native);
void 			(*NativeWindowFullscreen)(struct nativeWindow *native);
void 			(*NativeWindowWindowed)(struct nativeWindow *native);
void 			(*NativeWindowBordered)(struct nativeWindow *native);
void 			(*NativeWindowBorderless)(struct nativeWindow *native);
u32 			(*NativeWindowFullscreenCheck)(const struct nativeWindow *native);
u32 			(*NativeWindowBorderedCheck)(const struct nativeWindow *native);

void			(*NativeCursorShow)(struct nativeWindow *native);
void			(*NativeCursorHide)(struct nativeWindow *native);
u32 			(*NativeCursorLockedCheck)(struct nativeWindow *native);
u32 			(*NativeCursorVisibleCheck)(struct nativeWindow *native);
u32 			(*NativeCursorLock)(struct nativeWindow *native);
u32 			(*NativeCursorUnlock)(struct nativeWindow *native);
void 			(*NativeCursorSetRectangle)(struct nativeWindow *native, const vec2 nat_position, const vec2 size);
void 			(*NativeCursorUnsetRectangle)(struct nativeWindow *native);

void 			(*ScreenPositionNativeToEngine)(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos);
void 			(*ScreenPositionEngineToNative)(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos);
void 			(*WindowPositionNativeToEngine)(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos);
void 			(*WindowPositionEngineToNative)(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos);

utf8 			(*Utf8GetClipboard)(struct arena *mem);
void 			(*CstrSetClipboard)(const char *str);

u32 			(*EnterTextInputMode)(struct nativeWindow *native);
u32 			(*ExitTextInputMode)(struct nativeWindow *native);
u32 			(*KeyModifiers)(void);

u32 			(*EventConsume)(struct dsEvent *event);

void 			(*GlFunctionsInit)(struct gl_functions *func);

static void sdl3_NativeWindowGlSetCurrent(struct nativeWindow *native)
{
	if (!SDL_GL_MakeCurrent(native->sdl_win, native->gl_context))
	{
		LogString(T_RENDERER, S_ERROR, SDL_GetError());
	}	
}

static void sdl3_NativeWindowGlSwapBuffers(struct nativeWindow *native)
{
	if (!SDL_GL_SwapWindow(native->sdl_win))
	{
		LogString(T_RENDERER, S_WARNING, SDL_GetError());
	}
}

static u64 sdl3_NativeWindowGetNativeHandle(const struct nativeWindow *native)
{
	return (u64) native->sdl_win;
}

static void sdl3_NativeCursorShow(struct nativeWindow *native)
{
	if (!SDL_ShowCursor())
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static void sdl3_NativeCursorHide(struct nativeWindow *native)
{
	if (!SDL_HideCursor())
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static u32 sdl3_NativeCursorLock(struct nativeWindow *native)
{
	u32 lock = 1;
	if (!SDL_SetWindowRelativeMouseMode(native->sdl_win, 1))
	{
		lock = 0;
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}

	return lock;
}

static u32 sdl3_NativeCursorUnlock(struct nativeWindow *native)
{
	u32 lock = 0;
	if (!SDL_SetWindowRelativeMouseMode(native->sdl_win, 0))
	{
		lock = 1;
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
	
	return lock;
}

void sdl3_NativeCursorSetRectangle(struct nativeWindow *native, const vec2 nat_position, const vec2 size)
{
	const SDL_Rect rect = 
	{ 
		.x = nat_position[0], 
		.y = nat_position[1], 
		.w = size[0], 
		.h = size[1] 
	};

	if (!SDL_SetWindowMouseRect(native->sdl_win, &rect))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

void sdl3_NativeCursorUnsetRectangle(struct nativeWindow *native)
{
	if (!SDL_SetWindowMouseRect(native->sdl_win, NULL))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static u32 sdl3_NativeCursorVisibleCheck(struct nativeWindow *native)
{
	return (SDL_CursorVisible()) ? 1 : 0;
}

static u32 sdl3_NativeCursorLockedCheck(struct nativeWindow *native)
{
	return SDL_GetWindowRelativeMouseMode(native->sdl_win);
}

static void sdl3_NativeWindowConfigUpdate(vec2u32 position, vec2u32 size, struct nativeWindow *native)
{
	int w, h;
	if (!SDL_GetWindowSize(native->sdl_win, &w, &h))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	int x = (int) position[0];
       	int y = (int) position[1];
	if (!SDL_GetWindowPosition(native->sdl_win, &x, &y))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}

	size[0] = (u32) w;
	size[1] = (u32) h;
	position[0] = (u32) x;
	position[1] = (u32) y;
}

static void sdl3_NativeWindowFullscreen(struct nativeWindow *native)
{
	if (!SDL_SetWindowFullscreen(native->sdl_win, 1))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static void sdl3_NativeWindowWindowed(struct nativeWindow *native)
{
	if (!SDL_SetWindowFullscreen(native->sdl_win, 0))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static void sdl3_NativeWindowBordered(struct nativeWindow *native)
{
	if (!SDL_SetWindowBordered(native->sdl_win, 1))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static void sdl3_NativeWindowBorderless(struct nativeWindow *native)
{
	if (!SDL_SetWindowBordered(native->sdl_win, 0))
	{
		LogString(T_SYSTEM, S_WARNING, SDL_GetError());
	}
}

static u32 sdl3_NativeWindowFullscreenCheck(const struct nativeWindow *native)
{
	return (SDL_GetWindowFlags(native->sdl_win) & SDL_WINDOW_FULLSCREEN) ? 1 : 0;
}

static u32 sdl3_NativeWindowBorderedCheck(const struct nativeWindow *native)
{
	return (SDL_GetWindowFlags(native->sdl_win) & SDL_WINDOW_BORDERLESS) ? 0 : 1;
}

void sdl3_ScreenPositionNativeToEngine(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos)
{
       ds_AssertMessage(0, "#implement %s\n", __func__);
}

void sdl3_ScreenPositionEngineToNative(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos)
{
       ds_AssertMessage(0, "#implement %s\n", __func__);
}

void sdl3_WindowPositionNativeToEngine(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos)
{
	int w, h;
	if (!SDL_GetWindowSize(native->sdl_win, &w, &h))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	sys_pos[0] = nat_pos[0];
	sys_pos[1] = h - 1.0f - nat_pos[1];
}

void sdl3_WindowPositionEngineToNative(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos)
{
	int w, h;
	if (!SDL_GetWindowSize(native->sdl_win, &w, &h))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	nat_pos[0] = sys_pos[0];
	nat_pos[1] = h - 1.0f - sys_pos[1];
}

static void sdl3_DestroyGlContext(struct nativeWindow *native)
{
	if (!SDL_GL_DestroyContext(native->gl_context))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}
}

static void sdl3_CreateGlContext(struct nativeWindow *native)
{
	native->gl_context = SDL_GL_CreateContext(native->sdl_win);
	if (native->gl_context == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	/* turn off vsync for context (dont block on SWAP until window refresh (or something...) */
	if (!SDL_GL_SetSwapInterval(0))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	static u64 once = 1;
	if (once)
	{
		once = 0;
		SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
	}
}

static struct nativeWindow *sdl3_NativeWindowCreate(struct arena *mem, const char *title, const vec2u32 position, const vec2u32 size)
{
	struct nativeWindow *native = ArenaPush(mem, sizeof(struct nativeWindow));
	native->sdl_win = SDL_CreateWindow(title, (i32) size[0], (i32) size[1], SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
	if (native->sdl_win == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	sdl3_CreateGlContext(native);
	return native;
}

static void sdl3_NativeWindowDestroy(struct nativeWindow *native)
{
	sdl3_DestroyGlContext(native);
	SDL_DestroyWindow(native->sdl_win);
}

u32 sdl3_EnterTextInputMode(struct nativeWindow *native)
{
	u32 success = 1;
	if (!SDL_TextInputActive(native->sdl_win) && !SDL_StartTextInput(native->sdl_win))
	{
		LogString(T_SYSTEM, S_ERROR, SDL_GetError());
		success = 0;
	}

	return success;
}

u32 sdl3_ExitTextInputMode(struct nativeWindow *native)
{
	u32 success = 1;
	if (SDL_TextInputActive(native->sdl_win) && !SDL_StopTextInput(native->sdl_win))
	{
		LogString(T_SYSTEM, S_ERROR, SDL_GetError());
		success = 0;
	}

	return success;
}

utf8 sdl3_Utf8GetClipboard(struct arena *mem)
{
	utf8 ret = Utf8Empty();
	if (SDL_HasClipboardText())
	{	
		utf8 utf8_null =  { .buf = (u8*) SDL_GetClipboardText() };
		if (utf8_null.buf)
		{
			u32 len = 0;
			u64 size = 0;
			while (Utf8ReadCodepoint(&size, &utf8_null, size))
			{
				len += 1;
			}
			/* skip null */
			size -= 1;

			u8 *buf = ArenaPushMemcpy(mem, utf8_null.buf, size);
			if (buf)
			{
				ret = (utf8) { .buf = buf, .len = len, .size = (u32) size };
			}
			free (utf8_null.buf);
		}
		else
		{
			LogString(T_SYSTEM, S_ERROR, SDL_GetError());
		}
	}

	return ret;
}

void sdl3_CstrSetClipboard(const char *str)
{
	if (!SDL_SetClipboardText(str))
	{
		LogString(T_SYSTEM, S_ERROR, SDL_GetError());
	}
}

void sdl3_WrapperInit(void)
{
	if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WIN64__
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	if (!SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1)
		|| !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3)
		|| !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3)
		)
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	i32 major, minor;
	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &major);
	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &minor);
	if (major < 3 || minor < 3)
	{
		LogString(T_SYSTEM, S_FATAL, "Requires GL 3.3 or greater, exiting\n");
		FatalCleanupAndExit();
	}
#elif __DS_PLATFORM__ == __DS_WEB__
	SDL_SetHint(SDL_HINT_OPENGL_ES_DRIVER, "1");
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
	if (!SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1)
		|| !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3)
		|| !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0)
		)
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}

	i32 major, minor;
	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &major);
	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &minor);
	if (major < 3)
	{
		LogString(T_SYSTEM, S_FATAL, "Requires GLES 3.0 or greater, exiting\n");
		FatalCleanupAndExit();
	}
#endif
	/* Must be done after initalizing the video driver but before creating any opengl windows */
	if (!SDL_GL_LoadLibrary(NULL))
	{
		LogString(T_SYSTEM, S_FATAL, SDL_GetError());
		FatalCleanupAndExit();
	}
	NativeWindowCreate = &sdl3_NativeWindowCreate;
	NativeWindowDestroy = &sdl3_NativeWindowDestroy;
	NativeWindowGetNativeHandle = &sdl3_NativeWindowGetNativeHandle;
	NativeWindowGlSetCurrent = sdl3_NativeWindowGlSetCurrent;
	NativeWindowGlSwapBuffers = &sdl3_NativeWindowGlSwapBuffers;
	NativeWindowConfigUpdate = &sdl3_NativeWindowConfigUpdate;
	NativeWindowFullscreen = &sdl3_NativeWindowFullscreen;
	NativeWindowWindowed = &sdl3_NativeWindowWindowed;
	NativeWindowBordered = &sdl3_NativeWindowBordered;
	NativeWindowBorderless = &sdl3_NativeWindowBorderless;
	NativeWindowFullscreenCheck = &sdl3_NativeWindowFullscreenCheck;
	NativeWindowBorderedCheck = &sdl3_NativeWindowBorderedCheck;

	NativeCursorShow = &sdl3_NativeCursorShow;
	NativeCursorHide = &sdl3_NativeCursorHide;
	NativeCursorVisibleCheck = &sdl3_NativeCursorVisibleCheck;
	NativeCursorLockedCheck = &sdl3_NativeCursorLockedCheck;
	NativeCursorLock = &sdl3_NativeCursorLock;
	NativeCursorUnlock = &sdl3_NativeCursorUnlock;
 	NativeCursorSetRectangle = &sdl3_NativeCursorSetRectangle;
 	NativeCursorUnsetRectangle = &sdl3_NativeCursorUnsetRectangle;

	ScreenPositionNativeToEngine = &sdl3_ScreenPositionNativeToEngine;
	ScreenPositionEngineToNative = &sdl3_ScreenPositionEngineToNative;
	WindowPositionNativeToEngine = &sdl3_WindowPositionNativeToEngine;
	WindowPositionEngineToNative = &sdl3_WindowPositionEngineToNative;

	Utf8GetClipboard = sdl3_Utf8GetClipboard;
	CstrSetClipboard = sdl3_CstrSetClipboard;

	EnterTextInputMode = &sdl3_EnterTextInputMode;
	ExitTextInputMode = &sdl3_ExitTextInputMode;
	KeyModifiers = &sdl3_KeyModifiers;

	EventConsume = &sdl3_EventConsume;

	GlFunctionsInit = &sdl3_GlFunctionsInit;
}

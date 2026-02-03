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

#ifndef __DS_SDL3_WRAPPER_LOCAL_H__
#define __DS_SDL3_WRAPPER_LOCAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "SDL3/SDL.h"

#undef LogWriteMessage

#include "ds_platform.h"
#include "sdl3_wrapper_public.h"

void			sdl3_GlFunctionsInit(struct gl_Functions *func);
u32 			sdl3_EventConsume(struct dsEvent *event);
u32 			sdl3_KeyModifiers(void);
enum mouseButton	sdl3_DsMouseButton(const u8 mouse_button);
enum dsKeycode		sdl3_DsKeycode(const SDL_Keycode sdl_key);
enum dsKeycode 	sdl3_DsScancode(const SDL_Scancode sdl_key);

#ifdef __cplusplus
}
#endif

#endif

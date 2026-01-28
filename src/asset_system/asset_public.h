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

#ifndef __ASSET_PUBLIC_H__
#define __ASSET_PUBLIC_H__

/*
 * TODO: currently we build ssff's by loading png's containing collections of sprites in a row. This is 
 * 	 the wanted behaviour pngs constructed when animating, but for the purpose of creating a "static"
 * 	 sprite sheet, say for a level editor, this is not the absolute best. Instead, we actually want 
 * 	 to just hardcode each sprite's position in the sheet, as their positions are unlikely to change.
 */
enum r_program_id
{
	PROGRAM_PROXY3D,
	PROGRAM_UI,
	PROGRAM_COLOR,
	PROGRAM_LIGHTNING,
	PROGRAM_COUNT
};

enum r_texture_id
{
	TEXTURE_STUB,
	TEXTURE_NONE,
	TEXTURE_FONT_DEFAULT_SMALL,
	TEXTURE_FONT_DEFAULT_MEDIUM,
	TEXTURE_LED,
	TEXTURE_DYNAMIC,
	TEXTURE_COUNT
};

enum sprite_id
{
	SPRITE_NONE,

	/* LED SPRITES */
	SPRITE_LED_REFRESH_BUTTON,
	SPRITE_LED_REFRESH_BUTTON_HIGHLIGHT,
	SPRITE_LED_REFRESH_BUTTON_PRESSED,
	SPRITE_LED_FOLDER,
	SPRITE_LED_FILE,
	SPRITE_LED_PLAY,
	SPRITE_LED_PAUSE,
	SPRITE_LED_STOP,

	SPRITE_SORCERER_IDLE_1,
	SPRITE_SORCERER_IDLE_2,
	SPRITE_SORCERER_CAST_TRANSITION_1,
	SPRITE_SORCERER_STAND_CAST_1,
	SPRITE_SORCERER_STAND_CAST_2,
	SPRITE_SORCERER_STAND_CAST_3,
	SPRITE_SORCERER_STAND_CAST_4,
	SPRITE_SORCERER_STAND_CAST_5,
	SPRITE_SORCERER_WALK_CAST_1,
	SPRITE_SORCERER_WALK_CAST_2,
	SPRITE_SORCERER_WALK_CAST_3,
	SPRITE_SORCERER_WALK_CAST_4,
	SPRITE_SORCERER_WALK_CAST_5,
	SPRITE_SORCERER_RUN_CAST_1,
	SPRITE_SORCERER_RUN_CAST_2,
	SPRITE_SORCERER_RUN_CAST_3,
	SPRITE_SORCERER_RUN_CAST_4,
	SPRITE_SORCERER_RUN_CAST_5,
	SPRITE_COUNT
};

/* sprite sheet material id */
enum animation_id
{
	ANIMATION_SORCERER_IDLE,
	ANIMATION_SORCERER_CAST_TRANSITION,
	ANIMATION_SORCERER_STAND_CAST,
	ANIMATION_SORCERER_WALK_CAST,
	ANIMATION_SORCERER_RUN_CAST,
	ANIMATION_COUNT
};

/* sprite sheet material id */
enum ssff_id
{
	SSFF_NONE_ID = 0,
	SSFF_DYNAMIC_ID,
	SSFF_LED_ID,
	SSFF_COUNT
};

enum font_id
{
	FONT_NONE,
	FONT_DEFAULT_SMALL,
	FONT_DEFAULT_MEDIUM,
	FONT_COUNT
};

#include "sys_public.h"

/***************************** GLOBAL SPRITE ARRAY *****************************/

struct sprite
{
	enum ssff_id	ssff_id;	/* sprite sheet identifer	*/
	vec2u32		pixel_size;	/* size in pixels		*/
	vec2		bl;		/* lower-left uv coordinate 	*/
	vec2		tr;		/* upper-right uv coordinate	*/
};

extern struct sprite *	g_sprite;

/***************************** PNG ASSET DEFINITIONS AND GLOBALS *****************************/

#ifdef	DS_DEV

struct asset_png
{
	const char *	filepath;	/* relative file path */
	u32 		width;		/* pixel width */
	u32 		height;		/* pixel height */
	u32		sprite_width;	/* hardcoded sprite width for each png component */
	u32		valid;		/* is the asset valid? */
	file_handle	handle;		/* set to FILE_HANDLE_INVALID if not loaded */ 
};	

#endif 

/***************************** SSFF ASSET DEFINITIONS AND GLOBALS *****************************/

struct asset_ssff
{
	const char *		filepath;	/* relative file path */
	u32			loaded;		/* is the asset loaded? */
	const struct ssff_header *ssff;		/* loaded ssff header  */
	/* if loaded and valid */
	u32			width;
	u32			height;
	void *			pixel;		/* pixel opengl texture data 			*/
	struct sprite *		sprite_info;	/* sprite information is order of sprite generation */
	u32 			count;		/* uv[count] 					*/
	enum r_texture_id	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	u32			png_count;	/* number of png sources that this ssff is constructed from */
	struct asset_png *	png;		/* png sources  */
#endif
}; 

/* Return valid to use asset_ssff. If request fails, the returned asset is a dummy with dummy pixel parameters */
struct asset_ssff *	asset_database_request_ssff(struct arena *tmp, const enum ssff_id id);
/* return texture id of sprite */
enum r_texture_id	asset_database_sprite_get_texture_id(const enum sprite_id sprite);

/***************************** TTF ASSET DEFINITIONS AND GLOBALS *****************************/

#ifdef	DS_DEV

struct asset_ttf
{
	const char *		filepath;	/* relative file path */
	u32			valid;		/* is the asset valid? */
	file_handle		handle;		/* set to FILE_HANDLE_INVALID if not loaded */ 
};	

#endif 

/***************************** SSFF ASSET DEFINITIONS AND GLOBALS *****************************/

struct font_glyph
{
	vec2i32		size;		/* glyph size 			*/
	vec2i32		bearing;	/* glyph offset from baseline 	*/
	u32		advance;	/* pen position advancement (px)*/
	u32		codepoint;	/* utf32 codepoint 		*/
	vec2		bl;		/* lower-left uv coordinate 	*/
	vec2		tr;		/* upper-right uv coordinate	*/
};

struct font
{
	u64 			size;			/* sizeof(header) + sizeof(data[]) */
	f32			ascent;			/* max distance from baseline to the highest coordinate used to place an outline point */
	f32			descent;		/* min distance (is negative) from baseline to the lowest coordinate used to place an outline point */
	f32			linespace;		/* baseline-to-baseline offset ( > = 0.0f)  */
	struct hashMap *	codepoint_to_glyph_map;	/* map codepoint -> glyph. If codepoint not found, return "box" glyph" */

	struct font_glyph *	glyph;			/* glyphs in font; glyph[0] represents glyphs not found */
	u32			glyph_count;
	u32			glyph_unknown_index;	/* unknown glyph to use when encountering unmapped codepoint */

	u32			pixmap_width;
	u32			pixmap_height;
	void *			pixmap;			/* pixmap  */

	u8			data[];
};

struct asset_font
{
	const char *		filepath;	/* relative file path */
	u32			loaded;		/* is the asset loaded? */
	const struct font *	font;		/* loaded ssff header  */
	const u32 		pixel_glyph_height;
	/* if loaded and valid */
	enum r_texture_id	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	struct asset_ttf *	ttf;		/* ttf source  */
#endif
}; 

/* Return valid to use asset_ssff. If request fails, the returned asset is a dummy with dummy pixel parameters */
struct asset_font *asset_database_request_font(struct arena *tmp, const enum font_id id);
/* return glyph metrics of the corresponding codepoint. */
const struct font_glyph *glyph_lookup(const struct font *font, const u32 codepoint);

/******************** ASSET DATABASE ********************/

struct asset_database
{
	struct asset_ssff **ssff;	/* immutable ssff array, indexable with SSFF_**_ID */
	struct asset_font **font;	/* immutable ssff array, indexable with FONT_**_ID */
};

extern struct asset_database *g_asset_db;

/* Full flush of asset database; all assets will be reloaded (and rebuilt if DS_DEV) on next request */
void 	asset_database_flush_full(void);

/******************** asset_init.c ********************/

void asset_database_init(struct arena *mem_persistent);
void asset_database_cleanup(void);

#endif

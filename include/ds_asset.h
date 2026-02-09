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

#ifndef __DS_ASSET_PUBLIC_H__
#define __DS_ASSET_PUBLIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_platform.h"
#include "ds_font.h"

/*

 * TODO: currently we build ssff's by loading png's containing collections of sprites in a row. This is 
 * 	 the wanted behaviour pngs constructed when animating, but for the purpose of creating a "static"
 * 	 sprite sheet, say for a level editor, this is not the absolute best. Instead, we actually want 
 * 	 to just hardcode each sprite's position in the sheet, as their positions are unlikely to change.
 */
enum rProgramId
{
	PROGRAM_PROXY3D,
	PROGRAM_UI,
	PROGRAM_COLOR,
	PROGRAM_LIGHTNING,
	PROGRAM_COUNT
};

enum rTextureId
{
	TEXTURE_STUB,
	TEXTURE_NONE,
	TEXTURE_FONT_DEFAULT_SMALL,
	TEXTURE_FONT_DEFAULT_MEDIUM,
	TEXTURE_LED,
	TEXTURE_DYNAMIC,
	TEXTURE_COUNT
};

enum spriteId
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
enum animationId
{
	ANIMATION_SORCERER_IDLE,
	ANIMATION_SORCERER_CAST_TRANSITION,
	ANIMATION_SORCERER_STAND_CAST,
	ANIMATION_SORCERER_WALK_CAST,
	ANIMATION_SORCERER_RUN_CAST,
	ANIMATION_COUNT
};

/* sprite sheet material id */
enum ssffId
{
	SSFF_NONE_ID = 0,
	SSFF_DYNAMIC_ID,
	SSFF_LED_ID,
	SSFF_COUNT
};

enum fontId
{
	FONT_NONE,
	FONT_DEFAULT_SMALL,
	FONT_DEFAULT_MEDIUM,
	FONT_COUNT
};

/***************************** GLOBAL SPRITE ARRAY *****************************/

struct sprite
{
	enum ssffId	ssff_id;	/* sprite sheet identifer	*/
	vec2u32		pixel_size;	/* size in pixels		*/
	vec2		bl;		/* lower-left uv coordinate 	*/
	vec2		tr;		/* upper-right uv coordinate	*/
};

extern struct sprite *	g_sprite;

/***************************** PNG ASSET DEFINITIONS AND GLOBALS *****************************/

#ifdef	DS_DEV

struct assetPng
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

struct assetSsff
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
	enum rTextureId	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	u32			png_count;	/* number of png sources that this ssff is constructed from */
	struct assetPng *	png;		/* png sources  */
#endif
}; 

/* Return valid to use asset_ssff. If request fails, the returned asset is a dummy with dummy pixel parameters */
struct assetSsff *	AssetRequestSsff(struct arena *tmp, const enum ssffId id);
/* return texture id of sprite */
enum rTextureId		AssetSpriteGetTextureId(const enum spriteId sprite);

/***************************** TTF ASSET DEFINITIONS AND GLOBALS *****************************/

#ifdef	DS_DEV

struct assetTtf
{
	const char *		filepath;	/* relative file path */
	u32			valid;		/* is the asset valid? */
	file_handle		handle;		/* set to FILE_HANDLE_INVALID if not loaded */ 
};	

#endif 

/***************************** SSFF ASSET DEFINITIONS AND GLOBALS *****************************/

struct fontGlyph
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
	struct hashMap 		codepoint_to_glyph_map;	/* map codepoint -> glyph. If codepoint not found, return "box" glyph" */

	struct fontGlyph *	glyph;			/* glyphs in font; glyph[0] represents glyphs not found */
	u32			glyph_count;
	u32			glyph_unknown_index;	/* unknown glyph to use when encountering unmapped codepoint */

	u32			pixmap_width;
	u32			pixmap_height;
	void *			pixmap;			/* pixmap  */

	u8			data[];
};

struct assetFont
{
	const char *		filepath;	/* relative file path */
	u32			loaded;		/* is the asset loaded? */
	const struct font *	font;		/* loaded ssff header  */
	const u32 		pixel_glyph_height;
	/* if loaded and valid */
	enum rTextureId	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	struct assetTtf *	ttf;		/* ttf source  */
#endif
}; 

/* Return valid to use asset_ssff. If request fails, the returned asset is a dummy with dummy pixel parameters */
struct assetFont *	AssetRequestFont(struct arena *tmp, const enum fontId id);
/* return glyph metrics of the corresponding codepoint. */
const struct fontGlyph *GlyphLookup(const struct font *font, const u32 codepoint);

/******************** ASSET DATABASE ********************/

struct assetDatabase
{
	struct assetSsff **ssff;	/* immutable ssff array, indexable with SSFF_**_ID */
	struct assetFont **font;	/* immutable ssff array, indexable with FONT_**_ID */
};

extern struct assetDatabase *g_asset_db;

/* Full flush of asset database; all assets will be reloaded (and rebuilt if DS_DEV) on next request */
void 	AssetFlush(void);

/******************** asset_init.c ********************/

void AssetInit(struct arena *mem_persistent);
void AssetShutdown(void);

#ifdef __cplusplus
} 
#endif

#endif

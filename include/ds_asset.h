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
Assets: sprite sheets (.ssff, assets/sprites) and fonts (.kasfnt, assets/fonts), each a hardcoded entry of the
asset database (asset_init.c lists them with their file paths, texture ids and, with DS_DEV, their sources under
asset_components/). File paths are relative to the working directory: run from a build directory.

A request (AssetRequestSsff, AssetRequestFont) returns the database's own entry, loaded on the first request
and kept until AssetFlush. The entries are static; the caller never frees them. The asset system runs on the
main thread only.

DS_DEV (CMakeLists.txt: every build type but Release): the first request of a process rebuilds the asset
from its sources (PNG sheets with stb_image, TTF fonts with freetype) and rewrites the file under assets/,
then loads it. A missing source is fatal. Without DS_DEV the files are only read.

A missing, unreadable or corrupt file is logged (S_ERROR) and the entry is loaded as a stub until the next
flush: a transparent texture for a sheet (its sprites keep SPRITE_NONE's sheet and so draw nothing), a font
with one empty glyph for a font (layouts run and show nothing).

Sections: ids, the sprite table, sheets, fonts, the database, init.
*/

/********************************** ids **********************************/

/* the renderer's shader programs; here because the texture ids below are its texture units */
enum rProgramId
{
	PROGRAM_PROXY3D,
	PROGRAM_UI,
	PROGRAM_COLOR,
	PROGRAM_LIGHTNING,
	PROGRAM_COUNT
};

/* the renderer's texture units; each loaded asset has one */
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

/*
 * Sprites, in the order of their sheet's PNG cells (left to right); the sheet's parameter setter in
 * asset_init.c hardcodes that order. A new sprite goes into its sheet's run and into the setter.
 */
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

/* animations of the dynamic sheet (no user yet) */
enum animationId
{
	ANIMATION_SORCERER_IDLE,
	ANIMATION_SORCERER_CAST_TRANSITION,
	ANIMATION_SORCERER_STAND_CAST,
	ANIMATION_SORCERER_WALK_CAST,
	ANIMATION_SORCERER_RUN_CAST,
	ANIMATION_COUNT
};

/* sprite sheets; SSFF_NONE_ID is the one-pixel transparent sheet every sprite belongs to before its sheet is loaded */
enum ssffId
{
	SSFF_NONE_ID = 0,
	SSFF_DYNAMIC_ID,
	SSFF_LED_ID,
	SSFF_COUNT
};

/* fonts; FONT_NONE has no entry (NULL in the database) */
enum fontId
{
	FONT_NONE,
	FONT_DEFAULT_SMALL,
	FONT_DEFAULT_MEDIUM,
	FONT_COUNT
};

/***************************** the sprite table *****************************/

struct sprite
{
	enum ssffId	ssff_id;	/* sprite sheet identifer	*/
	v2u32		pixel_size;	/* size in pixels		*/
	v2		bl;		/* lower-left uv coordinate 	*/
	v2		tr;		/* upper-right uv coordinate	*/
};

/*
 * g_sprite[SPRITE_COUNT], indexed by enum spriteId. An entry is set by its sheet's request (uv in the
 * sheet's texture) and stays through flushes; before that it is zero: SSFF_NONE_ID, no size. Read-only
 * outside the asset system.
 */
extern struct sprite *	g_sprite;

/***************************** sprite sheets *****************************/

#ifdef	DS_DEV

/* a PNG source of a sheet: a row of equally wide cells, one sprite each, cell order = sprite order */
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

struct assetSsff
{
	const char *		filepath;	/* relative file path */
	u32			loaded;		/* is the asset loaded? */
	/* if loaded and valid */
	u32			width;		/* texture size, hardcoded; the sheet's collections stack as rows in it */
	u32			height;
	void *			pixel;		/* width x height RGBA8 texture, heap (ds_Alloc), freed by the next flush */
	enum rTextureId	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	u32			png_count;	/* number of png sources that this ssff is constructed from */
	struct assetPng *	png;		/* png sources  */
#endif
};

/*
 * Return the sheet's database entry, loaded (a stub if its file cannot be used: summary). tmp is scratch for
 * the request, used inside a record: the sheet file and the sprite table (tens of KB); with DS_DEV the PNG
 * build runs on it too (the decoded PNGs are heap). The sheet's g_sprite entries are set on a
 * successful load.
 */
struct assetSsff *	AssetRequestSsff(struct arena *tmp, const enum ssffId id);
/* return the texture id of the sprite's sheet: TEXTURE_NONE until the sheet's request */
enum rTextureId		AssetSpriteGetTextureId(const enum spriteId sprite);

/***************************** fonts *****************************/

#ifdef	DS_DEV

/* a TTF source of a font */
struct assetTtf
{
	const char *		filepath;	/* relative file path */
	u32			valid;		/* is the asset valid? */
	file_handle		handle;		/* set to FILE_HANDLE_INVALID if not loaded */
};

#endif

struct fontGlyph
{
	v2i32		size;		/* glyph size 			*/
	v2i32		bearing;	/* glyph offset from baseline 	*/
	u32		advance;	/* pen position advancement (px)*/
	u32		codepoint;	/* utf32 codepoint 		*/
	v2		bl;		/* lower-left uv coordinate 	*/
	v2		tr;		/* upper-right uv coordinate	*/
};

/* a loaded font: glyph metrics and uv in its alpha pixmap, the pixmap, and the codepoint map (asset_local.h: file format) */
struct font
{
	u64 			size;			/* sizeof(header) + sizeof(data[]) */
	f32			ascent;			/* max distance from baseline to the highest coordinate used to place an outline point */
	f32			descent;		/* min distance (is negative) from baseline to the lowest coordinate used to place an outline point */
	f32			linespace;		/* baseline-to-baseline offset ( > = 0.0f)  */
	struct ds_HashMap 		codepoint_to_glyph_map;	/* map codepoint -> glyph. If codepoint not found, return "box" glyph" */

	struct fontGlyph *	glyph;			/* glyphs in font; glyph[0] represents glyphs not found */
	u32			glyph_count;
	u32			glyph_unknown_index;	/* unknown glyph to use when encountering unmapped codepoint */

	u32			pixmap_width;
	u32			pixmap_height;
	void *			pixmap;			/* pixmap_width x pixmap_height alpha bytes, bottom row first */

	u8			data[];
};

struct assetFont
{
	const char *		filepath;	/* relative file path */
	u32			loaded;		/* is the asset loaded? */
	const struct font *	font;		/* the loaded font (heap), freed by the next flush */
	const u32 		pixel_glyph_height;	/* glyph height the font is rasterized at (DS_DEV) and the stub's line height */
	/* if loaded and valid */
	enum rTextureId	texture_id;	/* texture id to use in draw command pipeline   */
#ifdef	DS_DEV
	u32			valid;		/* is the asset valid? (if not, we must rebuilt it) */
	struct assetTtf *	ttf;		/* ttf source  */
#endif
};

/*
 * Return the font's database entry, loaded (a stub if its file cannot be used: summary). tmp as for
 * AssetRequestSsff (the file is mapped, not dumped); with DS_DEV the TTF build renders every glyph and the
 * pixmap onto it (hundreds of KB). id != FONT_NONE.
 */
struct assetFont *	AssetRequestFont(struct arena *tmp, const enum fontId id);
/* return the glyph of the codepoint, or the font's unknown glyph */
const struct fontGlyph *GlyphLookup(const struct font *font, const u32 codepoint);

/******************** the database ********************/

struct assetDatabase
{
	struct assetSsff **ssff;	/* immutable array, indexable with SSFF_**_ID */
	struct assetFont **font;	/* immutable array, indexable with FONT_**_ID; [FONT_NONE] is NULL */
};

extern struct assetDatabase *g_asset_db;

/*
 * Unload every sheet and font (their heap data is freed: no pointer to a loaded texture or font survives);
 * the next request loads, and with DS_DEV rebuilds, the asset again. g_sprite keeps its entries.
 */
void 	AssetFlush(void);

/******************** asset_init.c ********************/

/* ds_Init/ds_Shutdown call these: the database's tables on mem_persistent, freetype with DS_DEV */
void AssetInit(struct arena *mem_persistent);
void AssetShutdown(void);

#ifdef __cplusplus
}
#endif

#endif

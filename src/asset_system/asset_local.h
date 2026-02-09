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

#ifndef __DS_ASSET_LOCAL_H__
#define __DS_ASSET_LOCAL_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdio.h>
#include <stdlib.h>

#include "ds_base.h"
#include "ds_asset.h"

/***************************** SPIRTE SHEET FILE FORMAT *****************************/

/*
 * Sprite Sheet File Format (.ssff): Fully compact, no padding 
 *
 * 	ssff_header
 * 	collection[0]
 * 	...
 * 	collection[N1]
 * 	color_table[0]
 * 	collection[0].sprite[0]
 * 	...
 * 	collection[0].sprite[collection[0].color_count-1]
 * 	...
 * 	...
 * 	...
 * 	...
 * 	color_table[N1]
 * 	collection[N1].sprite[0]
 * 	...
 * 	collection[N1].sprite[collection[N1].color_count-1];
 * 	pixel_data[]	
 */

/*
 * ssff_header: .ssff header. The mapping between sprite collections <-> ssff file is immutable.
 * 	  Furthermore, the local spirte ordering of each sprite collection in the file is immutable.
 */
struct ssff_header
{
	u64 		size;			/* sizeof(ssff) + sizeof(data[]) */
	u32		collection_count;	/* number of collections */
	u32		collection_offset;	/* file offest to collection[collection_count] */
	u8		data[];
};

/*
 * ssff_collection: collection of local sprites, think sorcererHero. Each pixel usages bit_depth number of bits. The
 * 	bits index into the collections local color table to determine rgba color. 
 */
struct ssff_collection
{
	u32		color_count;		/* number of colors used in collection */
	u32		color_offset;		/* file offset to color[color_count] */
	u32		bit_depth;		/* number of bits per pixel */
	u32 		sprite_count;		/* sprite_count */
	u32		sprite_offset;		/* sprite file offset */
	u32		width;			/* sprite width sum  */
	u32		height;			/* max sprite height */
};

/*
 * ssffSprite - local sprite within a ssff_collection; indexable according to the ssff_collection's hardcoded
 * 	identififer. For example, collection[sorcerer_collection_id].sprite[SORCERER_WALK_1]. pixel coordinates
 * 	follows the following rule: x0 < x1, y0 < y1 and 
 *
 * 	(x0,y0) --------------------------- (x1, y0) 
 * 	   |					|
 * 	   |					|
 * 	   |					|
 * 	   |					|
 * 	   |					|
 * 	(x0,y1) --------------------------- (x1, y1) 
 */
struct ssffSprite
{
	u32 	x0;		
	u32 	x1;
	u32 	y0;
	u32 	y1;
	u32	pixel_offset;		/* file offest to pixel data, stored left-right, top-down */
};

struct ssffTextureReturn
{
	void *		pixel;	/* pixel opengl texture data 				*/
	struct sprite *	sprite;	/* sprite information is order of sprite generation 	*/
	u32 		count;	/* uv[count] 						*/
};

#ifdef	DS_DEV
/* build a ssff file header and save it to disk. replace clip color with { 0, 0, 0, 0 } color */
void				SsffBuild(struct arena *mem, const u32 ssff_id);
/* save ssff to disk  */
void 				SsffSave(const struct assetSsff *asset, const struct ssff_header *ssff);
#endif
/* heap allocate and load ssff from disk on success, return NULL on failure */
const struct ssff_header *	SsffLoad(struct assetSsff *asset);
/* heap allocate and construct texture with given width and height from ssff data. push, in order of generation, texture coordinates onto arena, and return values. */
struct ssffTextureReturn 	SsffTexture(struct arena *mem, const struct ssff_header *ssff, const u32 width, const u32 height);
/* verbosely print ssff contents */
void				SsffDebugPrint(FILE *out, const struct ssff_header *ssff);

/***************************** asset_font.c *****************************/

/* font file format:
	
	{
		size			: u64 (be)  // size of header + data[] 
		ascent			: f32 (be)
		descent         	: f32 (be)
		linespace       	: f32 (be)
		pixmap_width		: u32 (be)
		pixmap_height		: u32 (be)
		glyph_unknown_index  	: u32 (be)
		glyph_count 		: u32 (be)
	}	
	glyph[glyph_count] 
	{
		vec2i32	size;		; i32 (be)	
		vec2i32	bearing;	; i32 (be)	
		u32	advance;	; u32 (be)
		u32	codepoint;	; u32 (be)
		vec2	bl;		; f32 (be)
		vec2	tr;		; f32 (be)

	}

	codepoint_to_glyph_map		;  [serialized];

	pixmap[width*height]		; u8		// bl -> tp pixel sequence
 */

#ifdef	DS_DEV
/* initalize freetype library resources */
void				InternalFreetypeInit(void);
/* release freetype library resources */
void				InternalFreetypeFree(void);
/* build a font file header and save it to disk. */
void				FontBuild(struct arena *mem, const u32 font_id);
/* save font to disk  */
void 				FontSerialize(const struct assetFont *asset, const struct font *font);
#endif
/* heap allocate and load font from disk on success, return NULL on failure */
const struct font *		FontDeserialize(struct assetFont *asset);
/* debug print .kasfnt file to console */
void 				FontDebugPrint(FILE *out, const struct font *font);

/***************************** asset_init.c *****************************/

/* set parameters of hardcoded order of sprites in ssff */
void 	DynamicSsffSetSpriteParameters(struct assetSsff *dynamic_ssff, const struct ssffTextureReturn *param);
void 	LedSsffSetSpriteParameters(struct assetSsff *led_ssff, const struct ssffTextureReturn *param);

#ifdef __cplusplus
} 
#endif

#endif

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

#ifdef	DS_DEV
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#undef Log
#endif

#include <inttypes.h>

#include "asset_local.h"

#ifdef DS_DEV

/*
 * Handling sprite-to-ssff mappings:
 * 
 * Option 1: 
 * 	For each collection of sprites, say SorcererHero_SpriteSheet, we have a local ordering of sprites;
 * 	The ordering goes from left-right, top-down. For example, we may have the ordering
 *
 * 		"SORCERER_WALK_1" => "SORCERER_WALK_2"
 *
 * 	This identification must be hardcoded, so if we wish to change the order of sprites, or add/del sprites,
 * 	we must recompile with the new identifcation scheme.
 *
 * 	With the requirements of LOCAL identification done, we must also have a GLOBAL identification between
 * 	sprite and ssff. This is partly due to us wanting to be able to reload and regenerate .ssff files if an
 * 	asset_component has been updated.
 *
 * 	Suppose we want to grab the sprite SORCERER_WALK_1 when doing some drawing. In the code, we should simply
 * 	be able to do something like
 * 			
 * 			r_entity_alloc(..., material = SORCERER_WALK_1);
 *
 * 	In the backend, we must
 * 			
 * 			1. Use global identifier to identify which .ssff is needed
 * 			2. If not loaded, load
 * 			   Else if handle invalid, Reload
 *			3. Identify which sprite in .ssff is the one requested
 *			4. Get sprite data 
 *
 *	In order to determine how to implement the backed, we first determine what constants we have.
 *
 *	1. the sprite <-> .ssff mapping is immutable and determined at compile time.
 *	2. the sprite identifier is constant (either just unique id or unique (material, local_id)
 *
 * 	Assume for now that all materials are some sprite collection.
 * 
 * 	MATERIAL_SORCERER_ID = 1	//HARDCODED ID
 * 	SSFF_ID = 0 			//HARDCODED ID
 * 	SORCERER_WALK_1 = 0		//HARDCODED ID
 * 	SORCERER_WALK_2 = 1		//HARDCODED ID
 *
 * #if defined(DS_DEV)
 * 	material[MATERIAL_SORCERER_ID].source = DS_COMPILE_TIME_STRING(filepath);	//HARDCODED
 * #endif
 * 	material[MATERIAL_SORCERER_ID].ssff = SSFF_ID			// HARDCODED MAPPING
 *	material[MATERIAL_SORCERER_ID].collection_id = index;		// SET ON STARTUP (less error-prone, tedious).
 *
 * 	ssff[SSFF_ID].collection[index].id == MATERIAL_SORCERER_ID;	// HARDCODED MAPPING
 * 	ssff[SSFF_ID].collection[index].sprite[SORCERER_WALK_1] = ...;  // DYNAMIC
 * 	ssff[SSFF_ID].collection[index].sprite[SORCERER_WALK_2] = ...;  // DYNAMIC
 *
 *	SsffBuild(SSFF_ID) -> Rebuild ssff completely; if first build, set collection_id for each used material.
 */

/* return 0 for a cell of clip color only: it is stored as its first pixel (one transparent pixel) */
static u32 internal_get_sprite_parameters(struct arena *mem, struct ssffSprite *sprite, u32 *color_count, u32 *color, const u8 *pixel, const u32 pixels_per_line, const u32 sprite_width, const u32 sprite_height, const u32 sprite_index)
{
	//TODO  How to handle?
	const u32 clip_color = 0x0;

	u32 min_x = (sprite_index+1) * sprite_width;
	u32 max_x = sprite_index * sprite_width;
	u32 min_y = sprite_height;
	u32 max_y = 0;

	// NOTE these do not follow the file pixel layout, but are instead bottom left -> top right 
	for (u32 y = 0; y  < sprite_height; ++y)
	{
		for (u32 x = sprite_index * sprite_width; x < (sprite_index+1) * sprite_width; ++x)
		{
			const u32 px_color = ((u32) pixel[4*(y*pixels_per_line + x) + 0] << 24) 
			          	   | ((u32) pixel[4*(y*pixels_per_line + x) + 1] << 16) 
			          	   | ((u32) pixel[4*(y*pixels_per_line + x) + 2] <<  8) 
			          	   | ((u32) pixel[4*(y*pixels_per_line + x) + 3] <<  0);

			//TODO
			//if (px_color != clip_color)
			//{
				u32 in_table = 0;
				for (u32 k = 0; k < *color_count; ++k)
				{
					if (px_color == color[k])
					{
						in_table = 1;
						break;	
					}
				}

				if (!in_table)
				{
					ArenaPushPackedMemcpy(mem, &px_color, sizeof(u32));
					*color_count += 1;
				}

				if (px_color != clip_color)
				{
					if (x < min_x)
					{
						min_x = x;
					}
					if (x > max_x)
					{
						max_x = x;
					}
					if (y < min_y)
					{
						min_y = y;
					}
					if (y > max_y)
					{
						max_y = y;
					}
				}
			//}
		}
	}

	const u32 empty = (min_x > max_x);
	if (empty)
	{
		min_x = max_x = sprite_index * sprite_width;
		min_y = max_y = 0;
	}

	sprite->x0 = min_x;
	sprite->x1 = max_x;
	sprite->y0 = min_y;
	sprite->y1 = max_y;

	//fprintf(stderr, "sprite: [%u,%u] x [%u,%u]\n"
	//		, sprite->x0
	//		, sprite->x1
	//		, sprite->y0
	//		, sprite->y1
	//		);
	return !empty;
}

/*
 * heap-allocate and return generated sprite sheet on success, otherwise return NULL;
 * sprite_count - number of sprites in png
 * sprite_id	- array of unique string identifiers of each sprite, in order left->right, top->down in png
 */
void SsffBuild(struct arena *mem, const u32 ssff_id)
{
	struct assetSsff *info = g_asset_db->ssff[ssff_id];

	ArenaPushRecord(mem);
	
	void **pixel = ArenaPush(mem, info->png_count * sizeof(void *));
	u32 *png_width = ArenaPush(mem, info->png_count * sizeof(u32));
	u32 *png_height = ArenaPush(mem, info->png_count * sizeof(u32));
	u32 *color_count = ArenaPush(mem, info->png_count * sizeof(u32));
	u32 **color = ArenaPush(mem, info->png_count * sizeof(u32 *));
	u32 *sprite_count = ArenaPush(mem, info->png_count * sizeof(u32)); 
	struct ssffSprite **sprite = ArenaPush(mem, info->png_count * sizeof(struct ssffSprite *));

	for (u32 i = 0; i < info->png_count; ++i)
	{
		i32 comp;	/* the file's channels; stb converts to the 4 requested */
		pixel[i] = stbi_load((char *) info->png[i].filepath, (i32 *) png_width + i, (i32 *) png_height + i, &comp, 4);
		if (!pixel[i])
		{
			Log(T_ASSET, S_FATAL, "%s: %s", info->png[i].filepath, stbi_failure_reason());
			FatalCleanupAndExit();
		}
		if (png_width[i] % info->png[i].sprite_width != 0)
		{
			Log(T_ASSET, S_FATAL, "%s: width %u is not a multiple of the sprite width %u", info->png[i].filepath, png_width[i], info->png[i].sprite_width);
			FatalCleanupAndExit();
		}
		sprite_count[i] = png_width[i] / info->png[i].sprite_width;
		sprite[i] = ArenaPushPacked(mem, sprite_count[i] * sizeof(struct ssffSprite));
		color[i] = (void *) mem->stack_ptr;
		color_count[i] = 0;

		for (u32 s = 0; s < sprite_count[i]; ++s)
		{
			if (!internal_get_sprite_parameters(mem, sprite[i] + s, color_count + i, color[i], pixel[i], png_width[i], info->png[i].sprite_width, png_height[i], s))
			{
				Log(T_ASSET, S_WARNING, "%s: sprite %u is clip color only, stored as one transparent pixel", info->png[i].filepath, s);
			}
		}
	}

	struct ssff_header *header = ArenaPush(mem, sizeof(struct ssff_header));
	header->size = sizeof(struct ssff_header); 
	header->collection_count = info->png_count;
	header->collection_offset = sizeof(struct ssff_header);

	header->size += header->collection_count * sizeof(struct ssff_collection);
	struct ssff_collection *c = ArenaPushPacked(mem, header->collection_count * sizeof(struct ssff_collection));
	for (u32 i = 0; i < header->collection_count; ++i)
	{
		c[i].color_count = color_count[i];
		c[i].sprite_count = sprite_count[i];

		/* bits for the indices 0 .. color_count-1 */
		ds_Assert(c[i].color_count);
		c[i].bit_depth = (c[i].color_count > 1) ? 32 - Clz32(c[i].color_count - 1) : 0;

		header->size += c[i].color_count * sizeof(u32) + c[i].sprite_count * sizeof(struct ssffSprite);
		u32 *color_table = ArenaPushPacked(mem, c[i].color_count * sizeof(u32));
		struct ssffSprite *spr = ArenaPushPacked(mem, c[i].sprite_count * sizeof(struct ssffSprite)); 
		c[i].color_offset = (u32) ((u64) color_table - (u64) header);
		c[i].sprite_offset = (u32) ((u64) spr - (u64) header);

		for (u32 j = 0; j < c[i].color_count; ++j)
		{
			color_table[j] = color[i][j];
		}

		c[i].height = 0;
		c[i].width = 0;
		for (u32 j = 0; j < c[i].sprite_count; ++j)
		{
			spr[j] = sprite[i][j];
			c[i].width += spr[j].x1 - spr[j].x0 + 1;
			const u32 sprite_height = spr[j].y1 - spr[j].y0 + 1;
			if (c[i].height < sprite_height)
			{
				c[i].height = sprite_height;
			}
		}
	}

	for (u32 i = 0; i < header->collection_count; ++i)
	{
		for (u32 s = 0; s < c[i].sprite_count; ++s)
		{
			const u32 width = sprite[i][s].x1 - sprite[i][s].x0 + 1;
			const u32 height = sprite[i][s].y1 - sprite[i][s].y0 + 1;
			header->size += c[i].bit_depth*width*height / 8 + 1;
			void *sprite_pixel = ArenaPushPacked(mem, c[i].bit_depth*width*height / 8 + 1);
			u32 *color_table = (u32 *) (((u8 *) header) + c[i].color_offset);
			struct ssffSprite *spr = (struct ssffSprite *) (((u8 *) header) + c[i].sprite_offset);
			spr[s].pixel_offset = (u32) ((u64) sprite_pixel - (u64) header);

			struct ss stream = ss_Buffered(sprite_pixel, c[i].bit_depth*width*height / 8 + 1);	/* the block's bytes */
			
			for (u32 y = sprite[i][s].y0; y <= sprite[i][s].y1; ++y)
			{
				for (u32 x = sprite[i][s].x0; x <= sprite[i][s].x1; ++x)
				{
					const u32 color = ((u32) ((u8 *) pixel[i])[4*(y*png_width[i] + x) + 0] << 24)
							| ((u32) ((u8 *) pixel[i])[4*(y*png_width[i] + x) + 1] << 16)
							| ((u32) ((u8 *) pixel[i])[4*(y*png_width[i] + x) + 2] <<  8)
							| ((u32) ((u8 *) pixel[i])[4*(y*png_width[i] + x) + 3] <<  0);

					u32 ci = U32_MAX;
					for (u32 k = 0; k < color_count[i]; ++k)
					{
						if (color == color_table[k])
						{
							ci = k;
							break;	
						}
					}
					ds_Assert(ci != U32_MAX);	/* the table was built from these pixels */
					ss_WriteU32Partial(&stream, ci, c[i].bit_depth);
				}
			}

			sprite[i][s].x1 = sprite[i][s].x1 - sprite[i][s].x0;
			sprite[i][s].x0 = 0;
			sprite[i][s].y1 = sprite[i][s].y1 - sprite[i][s].y0;
			sprite[i][s].y0 = 0;
		}
	}

	SsffSave(info, header);
	for (u32 i = 0; i < info->png_count; ++i)
	{
		stbi_image_free(pixel[i]);
	}
	
	ArenaPopRecord(mem);
}

void SsffSave(const struct assetSsff *asset, const struct ssff_header *header)
{
	struct arena *tmp = ArenaPushScratch();

	struct file file = FileNull();
	if (FileTryCreateAtCwd(tmp, &file, asset->filepath, FILE_TRUNCATE) != FS_SUCCESS)
	{
		LogString(T_ASSET, S_FATAL, "Failed to create .ssff file handle");
		FatalCleanupAndExit();
	}

	FileWriteAppend(&file, (u8 *) header, header->size);
	FileClose(&file);

    ArenaPopScratch();
}

#endif

/* Every table and pixel block the sheet points at lies inside its size bytes */
static u32 SsffCheck(const struct ssff_header *ssff, const u64 size)
{
	if (size < sizeof(struct ssff_header) || ssff->size != size
		|| ssff->collection_offset > size
		|| (u64) ssff->collection_count * sizeof(struct ssff_collection) > size - ssff->collection_offset)
	{
		return 0;
	}

	const struct ssff_collection *c = (const struct ssff_collection *) (((const u8 *) ssff) + ssff->collection_offset);
	for (u32 i = 0; i < ssff->collection_count; ++i)
	{
		if (c[i].bit_depth > 32
			|| c[i].color_offset > size || (u64) c[i].color_count * sizeof(u32) > size - c[i].color_offset
			|| c[i].sprite_offset > size || (u64) c[i].sprite_count * sizeof(struct ssffSprite) > size - c[i].sprite_offset)
		{
			return 0;
		}

		const struct ssffSprite *sprite = (const struct ssffSprite *) (((const u8 *) ssff) + c[i].sprite_offset);
		for (u32 j = 0; j < c[i].sprite_count; ++j)
		{
			const u64 width = (u64) sprite[j].x1 - sprite[j].x0 + 1;
			const u64 height = (u64) sprite[j].y1 - sprite[j].y0 + 1;
			if (sprite[j].x1 < sprite[j].x0 || sprite[j].y1 < sprite[j].y0
				|| sprite[j].pixel_offset > size || c[i].bit_depth*width*height / 8 + 1 > size - sprite[j].pixel_offset)
			{
				return 0;
			}
		}
	}

	return 1;
}

const struct ssff_header *SsffLoad(struct arena *mem, const struct assetSsff *asset)
{
	const struct dsBuffer buf = FileDumpAtCwd(mem, asset->filepath);
	const struct ssff_header *ssff = (const struct ssff_header *) buf.data;
	if (!ssff)
	{
		Log(T_ASSET, S_ERROR, "%s: cannot read the sprite sheet", asset->filepath);
		return NULL;
	}
	if (!SsffCheck(ssff, buf.size))
	{
		Log(T_ASSET, S_ERROR, "%s: the sprite sheet is corrupt (a table or pixel block lies outside the file)", asset->filepath);
		return NULL;
	}
	return ssff;
}

struct ssffTextureReturn SsffTexture(struct arena *mem, const struct ssff_header *ssff, const u32 width, const u32 height)
{
	/* NOTE: BAD: if we add/subtract half widths to uv coordinates, then, as those coordinates gets interpolated
	 * in the fragment shader, we will get half the wanted pixels to represent borders of the sprite. */
	//const vec2 pixel_hw = 
	//{
	//	1.0f / (2.0f * width),
	//	1.0f / (2.0f * height),
	//};
	/* the collections stack as rows of the texture: each must fit in the row, the rows in the height */
	u64 height_required = 0;
	for (u32 i = 0; i < ssff->collection_count; ++i)
	{
		const struct ssff_collection *c = (struct ssff_collection *)(((u8 *) ssff) + ssff->collection_offset) + i;
		const struct ssffSprite *sprite = (struct ssffSprite *)(((u8 *) ssff) + c->sprite_offset);
		u64 row_width = 0;
		u64 row_height = 0;
		for (u32 j = 0; j < c->sprite_count; ++j)
		{
			row_width += (u64) sprite[j].x1 - sprite[j].x0 + 1;
			const u64 sprite_height = (u64) sprite[j].y1 - sprite[j].y0 + 1;
			row_height = (row_height < sprite_height) ? sprite_height : row_height;
		}
		height_required += c->height;
		if (row_width > width || row_height > c->height || height_required > height)
		{
			Log(T_ASSET, S_ERROR, "sprite sheet collection %u (%llu x %llu, rows to %llu) does not fit the %u x %u texture",
				i, (long long unsigned int) row_width, (long long unsigned int) row_height,
				(long long unsigned int) height_required, width, height);
			return (struct ssffTextureReturn) { 0 };
		}
	}

	u8 *pixel = ds_Alloc(width*height*sizeof(u32), sizeof(u32));
	if (!pixel)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a sprite sheet texture");
		FatalCleanupAndExit();
	}
	memset(pixel, 0, width*height*sizeof(u32));	/* outside the sprites: transparent, not garbage (the mipmaps sample it) */
	struct ssffTextureReturn ret = 
	{
		.pixel = pixel,
		.sprite = (struct sprite *) mem->stack_ptr,
		.count = 0,
	};

	for (u32 i = 0; i < ssff->collection_count; ++i)
	{
		const struct ssff_collection *c = (struct ssff_collection *)(((u8 *) ssff) + ssff->collection_offset) + i;
		const u32 *color_table = (u32 *)(((u8 *) ssff) + c->color_offset);
		const struct ssffSprite *sprite = (struct ssffSprite *)(((u8 *) ssff) + c->sprite_offset);
		ArenaPushPacked(mem, c->sprite_count * sizeof(struct sprite));
		u32 x_offset = 0;
		u32 y_offset = 0;
		for (u32 j = 0; j < c->sprite_count; ++j)
		{
			const u32 sprite_width = sprite[j].x1 - sprite[j].x0 + 1;
			const u32 sprite_height = sprite[j].y1 - sprite[j].y0 + 1;
			void *sprite_pixel = (((u8 *) ssff) + sprite[j].pixel_offset);
			struct ss stream = ss_Buffered(sprite_pixel, c->bit_depth*sprite_width*sprite_height / 8 + 1);	/* the block's bytes */
			for (u32 y = 0; y < sprite_height; ++y)
			{
				for (u32 x = 0; x < sprite_width; ++x)
				{
					const u32 ci = ss_ReadU32Partial(&stream, c->bit_depth);
					const u32 color = (ci < c->color_count) ? color_table[ci] : 0;	/* corrupt index: transparent */
					pixel[4*((y_offset + y)*width + (x_offset + x)) + 0] = (u8) (color >> 24);
					pixel[4*((y_offset + y)*width + (x_offset + x)) + 1] = (u8) (color >> 16);
					pixel[4*((y_offset + y)*width + (x_offset + x)) + 2] = (u8) (color >>  8);
					pixel[4*((y_offset + y)*width + (x_offset + x)) + 3] = (u8) (color >>  0);
				}
			}

			//Vec2Set(ret.uv[ret.count + j].bl, 
			//		pixel_hw[0] + (f32) x_offset / width, 
			//		pixel_hw[1] + (f32) (y_offset + sprite_height - 1) / height);
			//Vec2Set(ret.uv[ret.count + j].tr,
			//		ret.uv[ret.count + j].bl[0] + (f32) (sprite_width-1) / width,
			//		pixel_hw[1] + (f32) y_offset / height);
			ret.sprite[ret.count + j].bl = V2((f32) x_offset / width, 
							   (f32) (y_offset + sprite_height) / height);
			ret.sprite[ret.count + j].tr = V2(ret.sprite[ret.count + j].bl.x + (f32) sprite_width / width,
							   (f32) y_offset / height);
			ret.sprite[ret.count + j].pixel_size = V2U32(sprite_width, sprite_height);
			x_offset += sprite_width;
		}

		y_offset += c->height;
		ret.count += c->sprite_count;
	}

	return ret;
}

void SsffDebugPrint(FILE *out, const struct ssff_header *ssff)
{
	fprintf(out, "ssff[%p]\n{\n", ssff);

	fprintf(out, "\theader[0]\n\t{\n");
	fprintf(out, "\t\t.size = %" PRIu64 "\n", ssff->size);
	fprintf(out, "\t\t.collection_count = %u\n", ssff->collection_count);
	fprintf(out, "\t\t.collection_offset = %u\n", ssff->collection_offset);
	fprintf(out, "\t}\n");
	
	fprintf(out, "\tcollection_array[%u]\n\t{\n", ssff->collection_offset);
	const struct ssff_collection *c = (const struct ssff_collection *) (((u8 *) ssff) + ssff->collection_offset);
	for (u32 i = 0; i < ssff->collection_count; ++i)
	{
		fprintf(out, "\t\tcollection[%u]\n\t\t{\n", i);
		fprintf(out, "\t\t\t.color_count = %u\n", c[i].color_count);
		fprintf(out, "\t\t\t.color_offset = %u\n", c[i].color_offset);
		fprintf(out, "\t\t\t.bit_depth = %u\n", c[i].bit_depth);
		fprintf(out, "\t\t\t.sprite_count = %u\n", c[i].sprite_count);
		fprintf(out, "\t\t\t.sprite_offset = %u\n", c[i].sprite_offset);
		fprintf(out, "\t\t\t.width = %u\n", c[i].width);
		fprintf(out, "\t\t\t.height = %u\n", c[i].height);
		fprintf(out, "\t\t}\n");
	}
	fprintf(out, "\t}\n");
	for (u32 i = 0; i < ssff->collection_count; ++i)
	{
		const u32 *color = (u32 *) (((u8 *) ssff) + c[i].color_offset);
		const struct ssffSprite *sprite = (const struct ssffSprite *) (((u8 *) ssff) + c[i].sprite_offset);
		fprintf(out, "\tcolor_table[%u]\n\t{\n", c[i].color_offset);
		for (u32 j = 0; j < c[i].color_count; ++j)
		{
			fprintf(out, "\t\tcolor[%u] = { %u, %u, %u, %u }\n"
				, j
				, (color[j] >> 24) & 0xff
				, (color[j] >> 16) & 0xff
				, (color[j] >>  8) & 0xff
				, (color[j] >>  0) & 0xff);
		}
		fprintf(out, "\t}\n");
		fprintf(out, "\tsprite_table[%u]\n\t{\n", c[i].sprite_offset);
		for (u32 j = 0; j < c[i].sprite_count; ++j)
		{
			fprintf(out, "\t\tsprite[%u]\n\t\t{\n", j);
			fprintf(out, "\t\t\t.x0 = %u\n", sprite[j].x0);
			fprintf(out, "\t\t\t.x1 = %u\n", sprite[j].x1);
			fprintf(out, "\t\t\t.y0 = %u\n", sprite[j].y0);
			fprintf(out, "\t\t\t.y1 = %u\n", sprite[j].y1);
			fprintf(out, "\t\t}\n");
		}
		fprintf(out, "\t}\n");
	}
}

struct assetSsff *AssetRequestSsff(struct arena *tmp, const enum ssffId id)
{
	ArenaPushRecord(tmp);
	struct assetSsff *asset = g_asset_db->ssff[id];
#ifdef DS_DEV
	if (!asset->valid)
	{
		if (asset->loaded)
		{
			ds_Free(asset->pixel);
			asset->pixel = NULL;
		}

		SsffBuild(tmp, id);
		asset->valid = 1;
		asset->loaded = 0;
	}
#endif
	if (!asset->loaded)
	{
		const struct ssff_header *ssff = SsffLoad(tmp, asset);
		const struct ssffTextureReturn ret = (ssff)
			? SsffTexture(tmp, ssff, asset->width, asset->height)
			: (struct ssffTextureReturn) { 0 };
		if (ret.pixel)
		{
			switch (id)
			{
				case SSFF_DYNAMIC_ID: { DynamicSsffSetSpriteParameters(asset, &ret); } break;
				case SSFF_LED_ID: { LedSsffSetSpriteParameters(asset, &ret); } break;
				default: { ds_AssertString(0, "unhandled sprite sheet parameter setting"); } break;
			}
		}
		else
		{
			/* stub: a transparent texture; the sheet's sprites keep their last parameters (none: SPRITE_NONE's sheet) */
			Log(T_ASSET, S_ERROR, "%s: the sprite sheet is replaced by a transparent texture until the next flush", asset->filepath);
			asset->pixel = ds_Alloc(asset->width*asset->height*sizeof(u32), sizeof(u32));
			if (!asset->pixel)
			{
				LogString(T_ASSET, S_FATAL, "Failed to allocate a sprite sheet texture");
				FatalCleanupAndExit();
			}
			memset(asset->pixel, 0, asset->width*asset->height*sizeof(u32));
		}
		asset->loaded = 1;
	}

	ArenaPopRecord(tmp);
	return asset;
}

enum rTextureId AssetSpriteGetTextureId(const enum spriteId sprite)
{
	return g_asset_db->ssff[g_sprite[sprite].ssff_id]->texture_id;
}

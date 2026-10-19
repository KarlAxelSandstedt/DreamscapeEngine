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

#include "asset_local.h"

#ifdef	DS_DEV

#include "ft2build.h"
#include FT_FREETYPE_H

FT_Library g_ft_library = { 0 };

void InternalFreetypeInit(void)
{
	if (FT_Init_FreeType(&g_ft_library))
	{
		LogString(T_ASSET, S_FATAL, "Failed to initiate freetype2 library");
		FatalCleanupAndExit();
	}
}

void InternalFreetypeFree(void)
{
	FT_Done_FreeType(g_ft_library);
}

DEFINE_CPOOL_STRUCT(fontGlyph);
void FontBuild(struct arena *mem, const u32 id)
{
	struct assetFont *asset = g_asset_db->font[id];

	ArenaPushRecord(mem);

	FT_Face	face;
	const u32 face_index = 0;
	u32 error = FT_New_Face(g_ft_library, (const char *) asset->ttf->filepath, face_index, &face);
	if (error)
	{
		LogString(T_ASSET, S_FATAL, "Failed to initiate freetype face");
		FatalCleanupAndExit();
	}

	error = FT_Set_Pixel_Sizes(face, 0, asset->pixel_glyph_height);
	if (error)
	{
		LogString(T_ASSET, S_FATAL, "Failed to set freetype pixel size");
		FatalCleanupAndExit();
	}

	i32 total_glyph_width = 0;
	i32 glyph_width_max = 0;	/* the atlas packs rows glyph_height_max apart: a glyph may be taller than the pixel size */
	i32 glyph_height_max = 0;
	const u32 glyph_max_count = 4096;
    ds_CPool(voidptr) stack_pixels;
    ds_CPool(fontGlyph) stack_glyph;
    ds_CPoolAlloc(mem, stack_pixels, glyph_max_count, NOT_GROWABLE);
    ds_CPoolAlloc(mem, stack_glyph, glyph_max_count, NOT_GROWABLE);
	
	/* the not-found glyph (.notdef, the box), rendered like the others */
	error = FT_Load_Char(face, 0, FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL);
	if (error)
	{
		LogString(T_ASSET, S_FATAL, "Failed to load the not-found glyph");
		FatalCleanupAndExit();
	}

	total_glyph_width += face->glyph->bitmap.width;
	glyph_width_max = (i32) face->glyph->bitmap.width;
	glyph_height_max = (i32) face->glyph->bitmap.rows;
	u8 *pixels = ArenaPushMemcpy(mem, face->glyph->bitmap.buffer, face->glyph->bitmap.width * face->glyph->bitmap.rows);
	ds_CPoolPushValue(stack_pixels, pixels);
	const u32 glyph_unknown_index = stack_glyph.count;
	ds_CPoolPushValue(stack_glyph, ((struct fontGlyph)
	{
		.size = V2I32((i32) face->glyph->bitmap.width, (i32) face->glyph->bitmap.rows),
		.bearing = V2I32(face->glyph->bitmap_left, face->glyph->bitmap_top),
		.advance = (i32) face->glyph->advance.x >> 6,
		.codepoint = 0,
	}));

	/* load ascii, Latin-1 Suplement, Latin Extend A, Latin Extend B  */
	for (u32 c = 0x1; c <= 0x024f; ++c)
	{
		/* load embeded bitmap if found, otherwise load native image of glyph and 
		 * render anti-aliased coverage bitmap (8bit pixmap) */
		//error = FT_Load_Char(face, c, FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT | FT_LOAD_TARGET_NORMAL | FT_RENDER_MODE_NORMAL);
		error = FT_Load_Char(face, c, FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL);
		if (error)
		{
			LogString(T_ASSET, S_ERROR, "Failed to load glyph");
			continue;
		}

		total_glyph_width += face->glyph->bitmap.width;
		glyph_width_max = ((i32) face->glyph->bitmap.width > glyph_width_max) ? (i32) face->glyph->bitmap.width : glyph_width_max;
		glyph_height_max = ((i32) face->glyph->bitmap.rows > glyph_height_max) ? (i32) face->glyph->bitmap.rows : glyph_height_max;
		pixels = ArenaPushMemcpy(mem, face->glyph->bitmap.buffer, face->glyph->bitmap.width * face->glyph->bitmap.rows);
		ds_CPoolPushValue(stack_pixels, pixels);
		ds_CPoolPushValue(stack_glyph, ((struct fontGlyph)
			{
				.size = V2I32((i32) face->glyph->bitmap.width, (i32) face->glyph->bitmap.rows),
				.bearing = V2I32(face->glyph->bitmap_left, face->glyph->bitmap_top),
				.advance = (i32) face->glyph->advance.x >> 6,
				.codepoint = c,
			}));
	}

	/* Greek and Coptic */
	for (u32 c = 0x0370; c <= 0x03ff; ++c)
	{
		/* load embeded bitmap if found, otherwise load native image of glyph and 
		 * render anti-aliased coverage bitmap (8bit pixmap) */
		//error = FT_Load_Char(face, c, FT_LOAD_RENDER | FT_LOAD_FORCE_AUTOHINT | FT_LOAD_TARGET_NORMAL | FT_RENDER_MODE_NORMAL);
		error = FT_Load_Char(face, c, FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL | FT_RENDER_MODE_NORMAL);
		if (error)
		{
			LogString(T_ASSET, S_ERROR, "Failed to load glyph");
			continue;
		}

		total_glyph_width += face->glyph->bitmap.width;
		glyph_width_max = ((i32) face->glyph->bitmap.width > glyph_width_max) ? (i32) face->glyph->bitmap.width : glyph_width_max;
		glyph_height_max = ((i32) face->glyph->bitmap.rows > glyph_height_max) ? (i32) face->glyph->bitmap.rows : glyph_height_max;
		pixels = ArenaPushMemcpy(mem, face->glyph->bitmap.buffer, face->glyph->bitmap.width * face->glyph->bitmap.rows);
		ds_CPoolPushValue(stack_pixels, pixels);
		ds_CPoolPushValue(stack_glyph, ((struct fontGlyph)
			{
				.size = V2I32((i32) face->glyph->bitmap.width, (i32) face->glyph->bitmap.rows),
				.bearing = V2I32(face->glyph->bitmap_left, face->glyph->bitmap_top),
				.advance = (i32) face->glyph->advance.x >> 6,
				.codepoint = c,
			}));
	}

	const u32 hash_len = (u32) PowerOfTwoCeil(stack_glyph.count);
	struct font *font = ArenaPush(mem, sizeof(struct font));
	font->codepoint_to_glyph_map = ds_HashMapAlloc(mem, hash_len, hash_len, !GROWABLE);
	font->glyph_count = stack_glyph.count;
	font->glyph_unknown_index = glyph_unknown_index;
	font->glyph = stack_glyph.buf;

	font->ascent = (f32) face->size->metrics.ascender / 64;
	font->descent = (face->size->metrics.descender > 0.0f)
			? (f32) -face->size->metrics.descender / 64
			: (f32) face->size->metrics.descender / 64;
	font->linespace = (f32) face->size->metrics.height / 64;

	font->size = 0;
	font->pixmap_width = (u32) PowerOfTwoCeil(asset->pixel_glyph_height);
	font->pixmap_height = 0;
	font->pixmap = NULL;

	/* find the minimum square fitting all bitmaps: rows glyph_height_max apart, each losing at most one glyph width */
	const u32 row_height = (u32) glyph_height_max;
	while (font->pixmap_width < (u32) glyph_width_max || font->pixmap_width < row_height)
	{
		font->pixmap_width *= 2;
	}
	while (1)
	{
		/* rows needed if we allowed partial fitting of bitmaps; */
		const u32 clipped_rows_required = (total_glyph_width % font->pixmap_width)
				? 1 + (total_glyph_width / font->pixmap_width)
				: (total_glyph_width / font->pixmap_width);

		const u32 total_glyph_width_padded = total_glyph_width + clipped_rows_required*glyph_width_max;
		/* for every clipped row, pad an additional glyph to make sure we get a correct upper bound */
		const u32 rows_required = (total_glyph_width_padded % font->pixmap_width)
				? 1 + (total_glyph_width_padded / font->pixmap_width)
				: (total_glyph_width_padded / font->pixmap_width);

		const u32 pixmap_height_required = rows_required * row_height;
		if (pixmap_height_required <= font->pixmap_width)
		{
			 break;
		}
		font->pixmap_width *= 2;
	}

	font->pixmap_height = font->pixmap_width;
	font->pixmap = ArenaPush(mem, font->pixmap_width * font->pixmap_height);
	font->size = sizeof(u64) + 3*sizeof(f32) + 4*sizeof(u32)
		+ stack_glyph.count * (2*sizeof(v2i32) + 2*sizeof(u32) + 2*sizeof(v2))
		+ sizeof(u32) + hash_len*sizeof(u32)
		+ sizeof(u32) + hash_len*sizeof(u32)
		+ font->pixmap_width * font->pixmap_height;
	memset(font->pixmap, 0, font->pixmap_width * font->pixmap_height);

	const f32 pixel_halfsize = 1.0f / (2.0f*font->pixmap_width);
	v2u32 offset = V2U32(0, 0);
	for (u32 i = 0; i < stack_glyph.count; ++i)
	{
		u8 *alpha = font->pixmap;

		struct fontGlyph *g = stack_glyph.buf + i;
		ds_HashMapAdd(&font->codepoint_to_glyph_map, g->codepoint, i);
		pixels = stack_pixels.buf[i];
		if (offset.x + g->size.x > font->pixmap_width)
		{
			offset.x = 0;		
			offset.y += row_height;
		}
		ds_Assert(offset.y + g->size.y <= font->pixmap_height);

		for (i32 y = 0; y < g->size.y; ++y)
		{
			for (i32 x = 0; x < g->size.x; ++x)
			{
				ds_Assert(offset.y + y < font->pixmap_height);
				ds_Assert(offset.x + x < font->pixmap_width);
				alpha[(offset.y + g->size.y - 1 - y)*font->pixmap_width + (offset.x + x)] = pixels[y*g->size.x + x];
			}
		}

		g->bl.x = 2.0f*offset.x * pixel_halfsize; 
		g->tr.x = 2.0f*(offset.x + g->size.x) * pixel_halfsize; 
		g->bl.y = 2.0f*offset.y * pixel_halfsize; 
		g->tr.y = 2.0f*(offset.y + g->size.y) * pixel_halfsize; 

		offset.x += g->size.x;
	}

	if (FT_HAS_KERNING(face))
	{
		ds_AssertString(0, "Font supports kerning, but we do not!\n");
	}

	FontSerialize(asset, font);

	FT_Done_Face(face);
	ArenaPopRecord(mem);
}

void FontSerialize(const struct assetFont *asset, const struct font *font)
{
	struct arena *tmp = ArenaPushScratch();

	struct file file = FileNull();
	if (FileTryCreateAtCwd(tmp, &file, asset->filepath, FILE_TRUNCATE) != FS_SUCCESS)
	{
		LogString(T_ASSET, S_FATAL, "Failed to create .font file");
		FatalCleanupAndExit();
	}

	FileSetSize(&file, font->size);
	void *buf = FileMemoryMapPartial(&file, font->size, 0, FS_PROT_READ | FS_PROT_WRITE, FS_MAP_SHARED);
	struct ss ss = ss_Buffered(buf, font->size);

	ss_WriteU64Be(&ss, font->size);
	ss_WriteF32Be(&ss, font->ascent);
	ss_WriteF32Be(&ss, font->descent);
	ss_WriteF32Be(&ss, font->linespace);
	ss_WriteU32Be(&ss, font->pixmap_width);
	ss_WriteU32Be(&ss, font->pixmap_height);
	ss_WriteU32Be(&ss, font->glyph_unknown_index);
	ss_WriteU32Be(&ss, font->glyph_count);

	for (u32 i = 0; i < font->glyph_count; ++i)
	{
		ss_WriteI32Be(&ss, font->glyph[i].size.x);	
		ss_WriteI32Be(&ss, font->glyph[i].size.y);	
		ss_WriteI32Be(&ss, font->glyph[i].bearing.x);	
		ss_WriteI32Be(&ss, font->glyph[i].bearing.y);	
		ss_WriteU32Be(&ss, font->glyph[i].advance);	
		ss_WriteU32Be(&ss, font->glyph[i].codepoint);	
		ss_WriteF32Be(&ss, font->glyph[i].bl.x);
		ss_WriteF32Be(&ss, font->glyph[i].bl.y);
		ss_WriteF32Be(&ss, font->glyph[i].tr.x);
		ss_WriteF32Be(&ss, font->glyph[i].tr.y);
	}

	ds_HashMapSerialize(&ss, &font->codepoint_to_glyph_map);
	ss_Write8N(&ss, font->pixmap, font->pixmap_height * font->pixmap_width);

	FileMemoryUnmap(buf, font->size);
	FileClose(&file);

    ArenaPopScratch();
}

#endif

/* free a deserialized or stub font (heap: ds_Alloc and a heap hash map) */
static void InternalFontFree(struct font *font)
{
	if (font)
	{
		ds_HashMapDealloc(&font->codepoint_to_glyph_map);
		ds_Free(font->pixmap);
		ds_Free(font->glyph);
		ds_Free(font);
	}
}

/*
 * The whole file is checked against its counts before anything is read from it, so a short or corrupt file
 * gives NULL, never a partial font or a read outside the mapping.
 */
const struct font *FontDeserialize(const struct assetFont *asset)
{
	struct arena *tmp = ArenaPushScratch();
	struct file file = FileNull();
	struct font *font = NULL;
	void *buf = NULL;
	u64 size = 0;
	u32 ok = 0;

	FileTryOpenAtCwd(tmp, &file, asset->filepath, FILE_READ);
	if (file.handle == FILE_HANDLE_INVALID)
	{
		Log(T_ASSET, S_ERROR, "%s: cannot open the font", asset->filepath);
		goto end;
	}
	buf = FileMemoryMap(&size, &file, FS_PROT_READ, FS_MAP_SHARED);
	if (!buf)
	{
		Log(T_ASSET, S_ERROR, "%s: cannot map the font", asset->filepath);
		goto end;
	}

	struct ss ss = ss_Buffered(buf, size);
	const u64 header_size = sizeof(u64) + 3*sizeof(f32) + 4*sizeof(u32);
	if (ss_BytesLeft(&ss) < header_size || ss_ReadU64Be(&ss) > size)
	{
		goto corrupt;
	}

	font = ds_Alloc(sizeof(struct font), sizeof(u64));
	if (!font)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a font");
		FatalCleanupAndExit();
	}
	*font = (struct font) { .size = size };
	font->ascent = ss_ReadF32Be(&ss);
	font->descent = ss_ReadF32Be(&ss);
	font->linespace = ss_ReadF32Be(&ss);
	font->pixmap_width = ss_ReadU32Be(&ss);
	font->pixmap_height = ss_ReadU32Be(&ss);
	font->glyph_unknown_index = ss_ReadU32Be(&ss);
	font->glyph_count = ss_ReadU32Be(&ss);
	/* each serialized glyph: 4 i32, 2 u32, 4 f32 */
	if (font->glyph_count == 0 || font->glyph_unknown_index >= font->glyph_count
		|| ss_BytesLeft(&ss) < (u64) font->glyph_count * 10*sizeof(u32)
		|| font->pixmap_width == 0 || font->pixmap_height == 0)
	{
		goto corrupt;
	}

	font->glyph = ds_Alloc(font->glyph_count * sizeof(struct fontGlyph), sizeof(u32));
	if (!font->glyph)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a font's glyphs");
		FatalCleanupAndExit();
	}
	for (u32 i = 0; i < font->glyph_count; ++i)
	{
		font->glyph[i].size.x = ss_ReadI32Be(&ss);
		font->glyph[i].size.y = ss_ReadI32Be(&ss);
		font->glyph[i].bearing.x = ss_ReadI32Be(&ss);
		font->glyph[i].bearing.y = ss_ReadI32Be(&ss);
		font->glyph[i].advance = ss_ReadU32Be(&ss);
		font->glyph[i].codepoint = ss_ReadU32Be(&ss);
		font->glyph[i].bl.x = ss_ReadF32Be(&ss);
		font->glyph[i].bl.y = ss_ReadF32Be(&ss);
		font->glyph[i].tr.x = ss_ReadF32Be(&ss);
		font->glyph[i].tr.y = ss_ReadF32Be(&ss);
	}

	if (!ds_HashMapTryDeserialize(NULL, &ss, &font->codepoint_to_glyph_map, NOT_GROWABLE)
		|| ss_BytesLeft(&ss) < (u64) font->pixmap_width * font->pixmap_height)
	{
		goto corrupt;
	}
	font->pixmap = ds_Alloc((u64) font->pixmap_width * font->pixmap_height, sizeof(u8));
	if (!font->pixmap)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a font's pixmap");
		FatalCleanupAndExit();
	}
	ss_Read8N(&ss, font->pixmap, font->pixmap_height * font->pixmap_width);
	ok = 1;
	goto end;

corrupt:
	Log(T_ASSET, S_ERROR, "%s: the font file is corrupt (shorter than its counts claim)", asset->filepath);
end:
	if (!ok)
	{
		InternalFontFree(font);
		font = NULL;
	}
	if (buf)
	{
		FileMemoryUnmap(buf, size);
	}
	if (file.handle != FILE_HANDLE_INVALID)
	{
		FileClose(&file);
	}
	ArenaPopScratch();
	return font;
}

void FontFree(struct assetFont *asset)
{
	InternalFontFree((struct font *) asset->font);
	asset->font = NULL;
	asset->loaded = 0;
}

/*
 * A font without glyphs: every codepoint maps to one empty glyph half a glyph height wide, the pixmap is one
 * transparent pixel, so layouts and draws run and show nothing.
 */
static const struct font *FontStub(const struct assetFont *asset)
{
	struct font *font = ds_Alloc(sizeof(struct font), sizeof(u64));
	struct fontGlyph *glyph = ds_Alloc(sizeof(struct fontGlyph), sizeof(u32));
	u8 *pixmap = ds_Alloc(1, sizeof(u8));
	if (!font || !glyph || !pixmap)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a stub font");
		FatalCleanupAndExit();
	}

	const f32 height = (f32) asset->pixel_glyph_height;
	*glyph = (struct fontGlyph) { .advance = asset->pixel_glyph_height / 2 };
	*pixmap = 0;
	*font = (struct font)
	{
		.ascent = height,
		.descent = 0.0f,
		.linespace = height,
		.codepoint_to_glyph_map = ds_HashMapAlloc(NULL, 1, 1, NOT_GROWABLE),
		.glyph = glyph,
		.glyph_count = 1,
		.glyph_unknown_index = 0,
		.pixmap_width = 1,
		.pixmap_height = 1,
		.pixmap = pixmap,
	};
	if (!font->codepoint_to_glyph_map.hash)
	{
		LogString(T_ASSET, S_FATAL, "Failed to allocate a stub font's map");
		FatalCleanupAndExit();
	}
	return font;
}

void FontDebugPrint(FILE *out, const struct font *font)
{
	fprintf(out, "font:\n{\n");
	fprintf(out, "\tpixmap_width: %u\n", font->pixmap_width);
	fprintf(out, "\tpixmap_height: %u\n", font->pixmap_height);
	fprintf(out, "\tglyph_count: %u\n", font->glyph_count);

	for (u32 i = 0; i < font->glyph_count; ++i)
	{
		fprintf(out, "\tglyph[%u]:\n", i);
		fprintf(out, "\t{\n");
		fprintf(out, "\t\tsize:    { %i, %i }\n", font->glyph[i].size.x, font->glyph[i].size.y);
		fprintf(out, "\t\tbearing: { %i, %i }\n", font->glyph[i].bearing.x, font->glyph[i].bearing.y);
		fprintf(out, "\t\tbl: 	   { %f, %f }\n", font->glyph[i].bl.x, font->glyph[i].bl.y);
		fprintf(out, "\t\ttr: 	   { %f, %f }\n", font->glyph[i].tr.x, font->glyph[i].tr.y);
		fprintf(out, "\t\tadvance:   %u\n", font->glyph[i].advance);
		fprintf(out, "\t\tcodepoint: %u\n", font->glyph[i].codepoint);
		fprintf(out, "\t}\n");
	}
	fprintf(out, "}\n");
}

struct assetFont *AssetRequestFont(struct arena *tmp, const enum fontId id)
{
	ArenaPushRecord(tmp);
	struct assetFont *asset = g_asset_db->font[id];
#ifdef DS_DEV
	if (!asset->valid)
	{
		if (asset->loaded)
		{
			FontFree(asset);
		}

		FontBuild(tmp, id);
		asset->valid = 1;
		asset->loaded = 0;
	}
#endif
	if (!asset->loaded)
	{
		asset->font = FontDeserialize(asset);
		if (!asset->font)
		{
			Log(T_ASSET, S_ERROR, "%s: the font is replaced by a stub without glyphs", asset->filepath);
			asset->font = FontStub(asset);
		}
		asset->loaded = 1;
	}

	ArenaPopRecord(tmp);

	return asset;
}

const struct fontGlyph *GlyphLookup(const struct font *font, const u32 codepoint)
{
	const struct fontGlyph *g;
	u32 index = ds_HashMapFirst(&font->codepoint_to_glyph_map, codepoint);	
	for (; index != HASH_NULL; index = ds_HashMapNext(&font->codepoint_to_glyph_map, index))
	{
		g = font->glyph + index;
		if (g->codepoint == codepoint)
		{
			break;
		} 
	}

	if (index == HASH_NULL)
	{
		g = font->glyph + font->glyph_unknown_index;	
	}

	return g;
}

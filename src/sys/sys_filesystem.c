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

#include "hash_map.h"
#include "ds_vector.h"
#include "ds_platform.h"

struct file FileNull(void)
{
	return (struct file) { .handle = FILE_HANDLE_INVALID, .path = Utf8Empty(), .type = FILE_NONE };
}

struct directoryNavigator DirectoryNavigatorAlloc(const u32 initial_memory_string_size, const u32 hash_size, const u32 initial_hash_index_size)
{
	struct directoryNavigator dn =
	{
		.path = Utf8Empty(),
		.relative_path_to_file_map = HashMapAlloc(NULL, hash_size, initial_hash_index_size, GROWABLE),
		.mem_string = ArenaAlloc(initial_memory_string_size),
		.files = VectorAlloc(NULL, sizeof(struct file), initial_hash_index_size, GROWABLE),
	};

	return dn;
}

void DirectoryNavigatorDealloc(struct directoryNavigator *dn)
{
	ArenaFree(&dn->mem_string);
	HashMapFree(&dn->relative_path_to_file_map);
	VectorDealloc(&dn->files);
}

void DirectoryNavigatorFlush(struct directoryNavigator *dn)
{
	ArenaFlush(&dn->mem_string);
	HashMapFlush(&dn->relative_path_to_file_map);
	VectorFlush(&dn->files);
}

u32 DirectoryNavigatorLookupSubstring(struct arena *mem, u32 **index, struct directoryNavigator *dn, const utf8 substring)
{
	ArenaPushRecord(&dn->mem_string);

	struct kmpSubstring kmp_substring = Utf8LookupSubstringInit(&dn->mem_string, substring);
	*index = (u32 *) mem->stack_ptr;
	u32 count = 0;

	for (u32 i = 0; i < dn->files.next; ++i)
	{
		const struct file *file = VectorAddress(&dn->files, i);
		if (Utf8LookupSubstring(&kmp_substring, file->path))
		{
			ArenaPushPackedMemcpy(mem, &i, sizeof(i));
			count += 1;
		}
	}

	ArenaPopRecord(&dn->mem_string);
	return count;
}

u32 DirectoryNavigatorLookup(const struct directoryNavigator *dn, const utf8 filename)
{
	const u32 key = Utf8Hash(filename);
	u32 index = HASH_NULL;
	for (u32 i = HashMapFirst(&dn->relative_path_to_file_map, key); i != HASH_NULL; i = HashMapNext(&dn->relative_path_to_file_map, i))
	{
		const struct file *file = VectorAddress(&dn->files, i);
		if (Utf8Equivalence(filename, file->path))
		{
			index = i;
			break;
		}
	}

	return index;
}

enum fsError DirectoryNavigatorEnterAndAliasPath(struct directoryNavigator *dn, const utf8 path)
{
	DirectoryNavigatorFlush(dn);

	struct file dir = FileNull();
	const enum fsError ret = DirectoryTryOpenAtCwd(&dn->mem_string, &dir, CstrUtf8(&dn->mem_string, path));
	if (ret == FS_SUCCESS)
	{
		dn->path = path;
		DirectoryPushEntries(&dn->mem_string, &dn->files, &dir);
		for (u32 i = 0; i < dn->files.next; ++i)
		{
			const struct file *entry = VectorAddress(&dn->files, i);
			const u32 key = Utf8Hash(entry->path);
			HashMapAdd(&dn->relative_path_to_file_map, key, i);
		}
	}

	return ret;
}

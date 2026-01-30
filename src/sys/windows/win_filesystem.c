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

#include <stdio.h>
#include "win_local.h"
#include "sys_public.h"

#include <windows.h>
#include <pathcch.h>
#include <shlwapi.h>
#include <shlobj_core.h>

u32 			(*SystemAdminCheck)(void);

u32			(*Utf8PathRelativeCheck)(utf8 path);
u32			(*CstrPathRelativeCheck)(const char *path);

enum fsError 		(*FileTryCreate)(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 truncate);
enum fsError 		(*FileTryCreateAtCwd)(struct arena *mem, struct file *file, const char *filename, const u32 truncate);
enum fsError 		(*FileTryOpen)(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 writeable);
enum fsError 		(*FileTryOpenAtCwd)(struct arena *mem, struct file *file, const char *filename, const u32 writeable);
enum fsError 		(*FileTryOpen_absolute)(struct arena *mem, struct file *file, const char *filename);

enum fsError 		(*DirectoryTryCreate)(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
enum fsError 		(*DirectoryTryCreateAtCwd)(struct arena *mem, struct file *dir, const char *filename);
enum fsError 		(*DirectoryTryOpen)(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
enum fsError 		(*DirectoryTryOpenAtCwd)(struct arena *mem, struct file *dir, const char *filename);
enum fsError		(*DirectoryPushEntries)(struct arena *mem, struct vector *vec, struct file *dir);

u64 			(*FileWriteOffset)(const struct file *file, const u8 *buf, const u64 bufsize, const u64 offset);
u64 			(*FileWriteAppend)(const struct file *file, const u8 *buf, const u64 size);
void 			(*FileSync)(const struct file *file);
void 			(*FileClose)(struct file *file);

void *			(*FileMemoryMap)(u64 *size, const struct file *file, const u32 prot, const u32 flags);
void *			(*FileMemoryMapPartial)(const struct file *file, const u64 length, const u64 offset, const u32 prot, const u32 flags);
void 			(*FileMemoryUnmap)(void *addr, const u64 length);
void 			(*FileMemorySyncUnmap)(void *addr, const u64 length);

utf8			(*CwdGet)(struct arena *mem);
enum fsError		(*CwdSet)(struct arena *mem, const char *path);

struct dsBuffer 	(*FileDump)(struct arena *mem, const char *path, const struct file *dir);
struct dsBuffer 	(*FileDumpAtCwd)(struct arena *mem, const char *path);

enum fsError		(*FileStatusPath)(file_status *status, const char *path, const struct file *dir);
enum fsError		(*FileStatusFile)(file_status *status, const struct file *file);
enum fileType		(*FileStatusGetType)(const file_status *status);

void			(*FileStatusDebugPrint)(const file_status *stat);

u32			(*FileSetSize)(const struct file *file, const u64 size);

static enum fsError w_absolute_path_from_relative_path_and_directory(WCHAR w_absolute_path[MAX_PATH], const char *relative_path, const struct file *dir)
{
	WCHAR w_directory_path[MAX_PATH];
	WCHAR w_relative_path[MAX_PATH];

	if (!GetFinalPathNameByHandleW(dir->handle, w_directory_path, MAX_PATH, FILE_NAME_NORMALIZED | VOLUME_NAME_DOS))
	{
		LogSystemError(S_ERROR);
		return FS_HANDLE_INVALID;
	}

	const int len_relative = MultiByteToWideChar(CP_UTF8, 0, relative_path, -1, w_relative_path, MAX_PATH);
	if (!len_relative)
	{
		LogSystemError(S_ERROR);	
		return FS_HANDLE_INVALID;
	}

	if (PathCchCombine(w_absolute_path, MAX_PATH, w_directory_path, w_relative_path) != S_OK)
	{
		return FS_HANDLE_INVALID;
	}

	return FS_SUCCESS;
}

u32 win_system_user_is_admin(void)
{
	return IsUserAnAdmin() ? 1 : 0;
}

u32 win_utf8_path_is_relative(const utf8 path)
{
	const u32 req_size = (u32) Utf8SizeRequired(path);
	WCHAR w_path[MAX_PATH];
	const int len_relative = MultiByteToWideChar(CP_UTF8, 0, (char *) path.buf, req_size, w_path, MAX_PATH);
	u32 relative = 0;
	if (!len_relative)
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		w_path[len_relative] = '\0';
		relative = (u32) PathIsRelativeW(w_path);
	}

	return relative;
}


u32 win_cstr_path_is_relative(const char *path)
{
	return (u32) PathIsRelativeA(path);
}

enum fsError win_file_try_create(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 truncate)
{
	ds_Assert(file->handle == FILE_HANDLE_INVALID);
	file->handle = FILE_HANDLE_INVALID;
		
	enum fsError err = FS_SUCCESS;
	if (!CstrPathRelativeCheck(filename))
	{
		err = FS_PATH_INVALID;
	}
	else
	{
		WCHAR w_absolute_path[MAX_PATH];
		if (w_absolute_path_from_relative_path_and_directory(w_absolute_path, filename, dir) == FS_SUCCESS)

		{
			const DWORD creationDisposition = (truncate)
				? CREATE_ALWAYS
				: CREATE_NEW;

			file->handle = CreateFileW(w_absolute_path
				, GENERIC_READ | GENERIC_WRITE
				, FILE_SHARE_READ
				, NULL
				, creationDisposition
				, FILE_ATTRIBUTE_NORMAL
				, NULL);

			if (file->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_ALREADY_EXISTS: { err = FS_ALREADY_EXISTS; } break;
					case ERROR_FILE_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					default: { err = FS_ERROR_UNSPECIFIED; } break;
				}

				LogSystemError(S_ERROR);	
			}
		}
	}

	if (err == FS_SUCCESS)
	{
		file->path = Utf8Cstr(mem, filename);
		file->type = FILE_REGULAR;
	}

	return err;
}

enum fsError win_file_try_create_at_cwd(struct arena *mem, struct file *file, const char *filename, const u32 truncate)
{
	return win_file_try_create(mem, file, filename, &g_sys_env->cwd, truncate);
}

enum fsError win_file_try_open(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 writeable)
{
	ds_Assert(file->handle == FILE_HANDLE_INVALID);
	file->handle = FILE_HANDLE_INVALID;
		
	enum fsError err = FS_SUCCESS;
	if (!CstrPathRelativeCheck(filename))
	{
		err = FS_PATH_INVALID;
	}
	else
	{
		WCHAR w_absolute_path[MAX_PATH];
		if (w_absolute_path_from_relative_path_and_directory(w_absolute_path, filename, dir) == FS_SUCCESS)
		{
			const DWORD desiredAccess = (writeable)
				? GENERIC_READ | GENERIC_WRITE
				: GENERIC_READ;

			file->handle = CreateFileW(w_absolute_path
				, desiredAccess 
				, FILE_SHARE_READ
				, NULL
				, OPEN_EXISTING
				, FILE_ATTRIBUTE_NORMAL
				, NULL);

			if (file->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_ALREADY_EXISTS: { err = FS_ALREADY_EXISTS; } break;
					case ERROR_FILE_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					default: { err = FS_ERROR_UNSPECIFIED; } break;
				}

				LogSystemError(S_ERROR);	
			}
		}
	}

	if (err == FS_SUCCESS)
	{
		file->path = Utf8Cstr(mem, filename);
		file->type = FILE_DIRECTORY;
	}

	return err;
}

enum fsError win_file_try_open_at_cwd(struct arena *mem, struct file *file, const char *filename, const u32 writeable)
{
	return win_file_try_open(mem, file, filename, &g_sys_env->cwd, writeable);
}

enum fsError win_directory_try_create(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir)
{
	ds_Assert(dir->handle == FILE_HANDLE_INVALID);
	dir->handle = FILE_HANDLE_INVALID;
		
	enum fsError err = FS_SUCCESS;
	if (!CstrPathRelativeCheck(filename))
	{
		err = FS_PATH_INVALID;
	}
	else
	{
		WCHAR w_absolute_path[MAX_PATH];
		if (w_absolute_path_from_relative_path_and_directory(w_absolute_path, filename, parent_dir) == FS_SUCCESS)
		{
			if (CreateDirectoryW(w_absolute_path, NULL))
			{
				dir->handle = CreateFileW(w_absolute_path
					, GENERIC_READ | FILE_LIST_DIRECTORY
					, FILE_SHARE_READ
					, NULL
					, OPEN_EXISTING
					, FILE_FLAG_BACKUP_SEMANTICS
					, NULL);

			}
			else
			{
				switch (GetLastError())
				{
					case ERROR_ALREADY_EXISTS: { err = FS_ALREADY_EXISTS; } break;
					case ERROR_FILE_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					default: { err = FS_ERROR_UNSPECIFIED; } break;
				}

				LogSystemError(S_ERROR);	
			}
		}
	}

	if (err == FS_SUCCESS)
	{
		dir->path = Utf8Cstr(mem, filename);
		dir->type = FILE_DIRECTORY;
	}

	return err;
}

enum fsError win_directory_try_create_at_cwd(struct arena *mem, struct file *dir, const char *filename)
{
	return win_directory_try_create(mem, dir, filename, &g_sys_env->cwd);
}

enum fsError win_directory_try_open(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir)
{
	ds_Assert(dir->handle == FILE_HANDLE_INVALID);
	dir->handle = FILE_HANDLE_INVALID;
		
	enum fsError err = FS_SUCCESS;
	if (!CstrPathRelativeCheck(filename))
	{
		err = FS_PATH_INVALID;
	}
	else
	{
		WCHAR w_absolute_path[MAX_PATH];
		if (w_absolute_path_from_relative_path_and_directory(w_absolute_path, filename, parent_dir) == FS_SUCCESS)
		{
			dir->handle = CreateFileW(w_absolute_path
				, GENERIC_READ | FILE_LIST_DIRECTORY
				, FILE_SHARE_READ
				, NULL
				, OPEN_EXISTING
				, FILE_FLAG_BACKUP_SEMANTICS
				, NULL);

			if (dir->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_FILE_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					default: { err = FS_ERROR_UNSPECIFIED; } break;
				}

				LogSystemError(S_ERROR);	
			}
		}
	}

	if (err == FS_SUCCESS)
	{
		dir->path = Utf8Cstr(mem, filename);
		dir->type = FILE_DIRECTORY;
	}

	return err;
}

enum fsError win_directory_try_open_at_cwd(struct arena *mem, struct file *dir, const char *filename)
{
	return win_directory_try_open(mem, dir, filename, &g_sys_env->cwd); 
}

struct dsBuffer win_file_dump(struct arena *mem, const char *path, const struct file *dir)
{
	struct file file = FileNull();
	struct dsBuffer buf = { 0 };
	if (!FileTryOpen(mem, &file, path, dir, FILE_READ) == FS_SUCCESS)
	{
		return buf;
	}

	LARGE_INTEGER size;
	if (!GetFileSizeEx(file.handle, &size))
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		buf.data = ArenaPush(mem, size.QuadPart);
		if (buf.data)
		{
			DWORD bytes_read;
			buf.size = size.QuadPart;
			ds_Assert(size.QuadPart <= U32_MAX);
			if (!ReadFile(file.handle, buf.data, (u32) buf.size, &bytes_read, NULL))
			{
				LogSystemError(S_ERROR);
				buf.data = NULL;
				buf.size = 0;
			}
			else
			{
				ds_Assert(bytes_read == buf.size);
			}
		}
	}
	
	FileClose(&file);

	return buf;
}

struct dsBuffer win_file_dump_at_cwd(struct arena *mem, const char *path)
{
	return win_file_dump(mem, path, &g_sys_env->cwd);
}

u32 win_file_set_size(const struct file *file, const u64 size)
{
	u32 success = 1;
	LARGE_INTEGER l_size = { .QuadPart = size };
	if (!SetFilePointerEx(file->handle, l_size, NULL, FILE_BEGIN) || !SetEndOfFile(file->handle))
	{
		LogSystemError(S_ERROR);
		success = 0;	
	}

	return success;
}

void win_file_close(struct file *file)
{
	if (!CloseHandle(file->handle))
	{
		LogSystemError(S_ERROR);
	}
	file->handle = FILE_HANDLE_INVALID;
}


u64 win_file_write_offset(const struct file *file, const u8 *buf, const u64 size, const u64 offset)
{
	LARGE_INTEGER l_size;
	if (!GetFileSizeEx(file->handle, &l_size))
	{
		LogSystemError(S_ERROR);
		return 0;
	}

	if ((u64) l_size.QuadPart < offset + size)
	{
		win_file_set_size(file, offset + size);
	}

	LARGE_INTEGER l_offset = { .QuadPart = offset };
	if (!SetFilePointerEx(file->handle, l_offset, NULL, FILE_BEGIN))
	{
		LogSystemError(S_ERROR);
		return 0;
	}

	DWORD bytes_written;
	/* bytes_written are zeroed inside, before error checking */
	if (!WriteFile(file->handle, buf, (DWORD) size, &bytes_written, NULL))
	{
		LogSystemError(S_ERROR);
	}

	/* Seems like no memory maps will see our writes if we do not sync... TODO: Fix sane api */
	FileSync(file);
	ds_AssertMessage(bytes_written == size, "bytes_written = %u, size = %u\n", bytes_written, size); 
	return bytes_written;

}

u64 win_file_write_append(const struct file *file, const u8 *buf, const u64 size)
{
	LARGE_INTEGER distance_to_move = { .QuadPart = 0 };
	if (!SetFilePointerEx(file->handle, distance_to_move, NULL, FILE_END))
	{
		LogSystemError(S_ERROR);
		return 0;
	}

	DWORD bytes_written;
	/* bytes_written are zeroed inside, before error checking */
	if (!WriteFile(file->handle, buf, (DWORD) size, &bytes_written, NULL))
	{
		LogSystemError(S_ERROR);
	}

	/* Seems like no memory maps will see our writes if we do not sync... TODO: Fix sane api */
	FileSync(file);

	ds_AssertMessage(bytes_written == size, "bytes_written = %u, size = %u\n", bytes_written, size); 

	return bytes_written;
}


void win_file_sync(const struct file *file)
{
	if (!FlushFileBuffers(file->handle))
	{
		LogSystemError(S_ERROR);
	}
}


void *win_file_memory_map(u64 *size, const struct file *file, const u32 prot, const u32 garbage)
{ 
	LARGE_INTEGER l_size;
	void *map = NULL;
	*size = 0;
	if (!GetFileSizeEx(file->handle, &l_size))
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		*size = l_size.QuadPart;
		map = FileMemoryMapPartial(file, 0, 0, prot, garbage);
	}

	return map;
}

void *win_file_memory_map_partial(const struct file *file, const u64 length, const u64 offset, const u32 prot, const u32 garbage)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	DWORD new_size_low = 0;
	DWORD new_size_high = 0;
	if (length)
	{
		LARGE_INTEGER l_size;
		if (!GetFileSizeEx(file->handle, &l_size))
		{
			LogSystemError(S_ERROR);
			return NULL;
		}
		else if ((u64) l_size.QuadPart < offset + length)
		{
			new_size_high = (u32) ((offset + length) >> 32);
			new_size_low = (u32) (offset + length);
		}
	}
	void *map = NULL;
	const DWORD map_prot = ((prot & (FILE_MAP_READ | FILE_MAP_WRITE)) == (FILE_MAP_READ | FILE_MAP_WRITE))
		? PAGE_READWRITE
		: PAGE_READONLY;
	HANDLE handle = CreateFileMappingA(file->handle, NULL, map_prot, new_size_high, new_size_low, NULL);
	if (handle == NULL)
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		const DWORD mod = offset % info.dwAllocationGranularity;
		const DWORD offset_high = (u32) ((offset-mod) >> 32);
		const DWORD offset_low = (u32) (offset-mod);
		map = MapViewOfFileEx(handle, prot, offset_high, offset_low, length + mod, NULL);

		if (map == NULL)
		{
			LogSystemError(S_ERROR);
		}
		else
		{
			map = ((u8 *) map) + mod;
		}

		if (CloseHandle(handle) == 0)
		{
			LogSystemError(S_ERROR);
		}
	}

	return map;
}

void win_file_memory_unmap(void *addr, const u64 length)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	const u64 mod = (u64) addr % info.dwAllocationGranularity;

	if (UnmapViewOfFile(((u8 *) addr) - mod) == 0)
	{
		LogSystemError(S_ERROR);
	}
}

void win_file_memory_sync_unmap(void *addr, const u64 length)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	const u64 mod = (u64) addr % info.dwAllocationGranularity;

	if (FlushViewOfFile(((u8 *) addr) - mod, length + mod) == 0)
	{	
		LogSystemError(S_ERROR);
	}

	if (UnmapViewOfFile(((u8 *) addr) - mod) == 0)
	{
		LogSystemError(S_ERROR);
	}
}

utf8 win_cwd_get(struct arena *mem)
{
	utf8 str = Utf8Empty();

	const DWORD req_size = GetCurrentDirectory(0, NULL);
	void *buf = ArenaPush(mem, req_size);
	if (buf)
	{
		const DWORD len = GetCurrentDirectory(req_size, buf);
		if (len)
		{
			str.buf = buf;
			str.len = (u32) len;
			str.size = (u32) req_size;
		}
		else
		{
			LogSystemError(S_ERROR);
		}
	}
	

	return str;
}

enum fsError win_cwd_set(struct arena *mem, const char *path)
{
	enum fsError err = FS_SUCCESS;
	HANDLE handle = CreateFileA((char *) path
			, GENERIC_READ
			, FILE_SHARE_READ | FILE_SHARE_WRITE
			, NULL
			, OPEN_EXISTING
			, FILE_FLAG_BACKUP_SEMANTICS
			, NULL);
	
	if (handle == INVALID_HANDLE_VALUE)
	{
		g_sys_env->cwd.handle = FILE_HANDLE_INVALID;
		err = FS_ERROR_UNSPECIFIED;
		LogSystemError(S_ERROR);
	}
	else
	{
		if (!SetCurrentDirectory((char *) path))
		{
			/* We shouldn't use the underlying system cwd; instead, all *_at_cwd operations
			 * go through g_sys_env on windows, so a WARNING is sufficient. */
			err = FS_ERROR_UNSPECIFIED;
			LogSystemError(S_WARNING);
		}

		g_sys_env->cwd.handle = handle;
		g_sys_env->cwd.type = FILE_DIRECTORY;
		g_sys_env->cwd.path = win_cwd_get(mem);	
	}

	return err;
}

enum fsError win_directory_push_entries(struct arena *mem, struct vector *vec, struct file *dir)
{
	WCHAR w_path[MAX_PATH];
	if (w_absolute_path_from_relative_path_and_directory(w_path, "*", dir) != FS_SUCCESS)
	{
		return FS_ERROR_UNSPECIFIED;
	}

	WIN32_FIND_DATAW file_info;
	HANDLE handle = FindFirstFileW(w_path, &file_info);
	if (handle == INVALID_HANDLE_VALUE)
	{
		LogSystemError(S_ERROR);
		return FS_ERROR_UNSPECIFIED;
	}
	

	ArenaPushRecord(mem);
	const u32 vec_record = vec->next;

	enum fsError ret = FS_SUCCESS;
	file_status status;
	char cstr_filename[4*MAX_PATH];
	do
	{
		struct file *file = VectorPush(vec).address;

		int cstr_len = WideCharToMultiByte(CP_UTF8, 0, file_info.cFileName, -1, cstr_filename, sizeof(cstr_filename), NULL, NULL);
		if (cstr_len == 0)
		{
			LogSystemError(S_ERROR);
			ret = FS_ERROR_UNSPECIFIED;
			break;
		}

		file->path = Utf8Cstr(mem, cstr_filename);
		if (FileStatusPath(&status, cstr_filename, dir) != FS_SUCCESS)
		{
			ret = FS_ERROR_UNSPECIFIED;
			break;
		}

		file->type = FileStatusGetType(&status);


	} while (FindNextFileW(handle, &file_info) != 0);

	if (ret != FS_SUCCESS)
	{
		ArenaPopRecord(mem);
		vec->next = vec_record;
	}

	FindClose(handle);
	FileClose(dir);
	*dir = FileNull();
	return ret;
}

enum fsError win_file_status_file(file_status *status, const struct file *file)
{
	WCHAR w_path[MAX_PATH];
	if (!GetFinalPathNameByHandleW(file->handle, w_path, MAX_PATH, FILE_NAME_NORMALIZED | VOLUME_NAME_DOS))
	{
		LogSystemError(S_ERROR);
		return FS_ERROR_UNSPECIFIED;
	}

	enum fsError err = FS_SUCCESS;
	if (!GetFileAttributesExW(w_path, GetFileExInfoStandard, status))
	{
		LogSystemError(S_ERROR);
		err = FS_ERROR_UNSPECIFIED;
	}

	return err;
}

enum fsError win_file_status_path(file_status *status, const char *path, const struct file *dir)
{	
	WCHAR w_path[MAX_PATH];
	enum fsError err = w_absolute_path_from_relative_path_and_directory(w_path, path, dir);
	if (err == FS_SUCCESS)
	{
		if (!GetFileAttributesExW(w_path, GetFileExInfoStandard, status))
		{
			LogSystemError(S_ERROR);
			err = FS_ERROR_UNSPECIFIED;
		}
	}

	return err;
}

enum fileType win_file_status_type(const file_status *status)
{
	enum fileType type;

	if (status->dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
	{
		type = FILE_DIRECTORY;
	}
	else if (status->dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT)
	{
		/* reparse point  */
		type = FILE_UNRECOGNIZED;

	}
	else if (status->dwFileAttributes & FILE_ATTRIBUTE_DEVICE)
	{
		/* device */
		type = FILE_UNRECOGNIZED;
	}
	else
	{
		type = FILE_REGULAR;
	}

	return type;
}

#include <time.h>

// Helper function to convert FILETIME to a human-readable string
void FileTimeToString(FILETIME ft, char* buffer, size_t bufferSize) 
{
	SYSTEMTIME st;
	if (!FileTimeToSystemTime(&ft, &st)) 
	{
		snprintf(buffer, bufferSize, "Error converting FILETIME: %d", GetLastError());
		return;
	}
	
	// Format as YYYY-MM-DD HH:MM:SS
	snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
	         st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
}

void win_file_status_debug_print(const file_status *stat)
{
	fprintf(stderr, "\nFile Attributes:\n");
	DWORD attrs = stat->dwFileAttributes;
	if (attrs == INVALID_FILE_ATTRIBUTES) {
		fprintf(stderr, "  Invalid attributes (error).\n");
	} else {
		fprintf(stderr, "  %-30s: %s\n", "Is Directory", (attrs & FILE_ATTRIBUTE_DIRECTORY) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Is Regular File", 
		       !(attrs & (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT | FILE_ATTRIBUTE_DEVICE)) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Read-Only", (attrs & FILE_ATTRIBUTE_READONLY) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Hidden", (attrs & FILE_ATTRIBUTE_HIDDEN) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "System File", (attrs & FILE_ATTRIBUTE_SYSTEM) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Archive", (attrs & FILE_ATTRIBUTE_ARCHIVE) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Temporary", (attrs & FILE_ATTRIBUTE_TEMPORARY) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Compressed", (attrs & FILE_ATTRIBUTE_COMPRESSED) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Encrypted", (attrs & FILE_ATTRIBUTE_ENCRYPTED) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Reparse Point (e.g., Symlink)", 
		       (attrs & FILE_ATTRIBUTE_REPARSE_POINT) ? "Yes" : "No");
		fprintf(stderr, "  %-30s: %s\n", "Device", (attrs & FILE_ATTRIBUTE_DEVICE) ? "Yes" : "No");
	}
	
	// Print File Size
	ULONGLONG fileSize = ((ULONGLONG)stat->nFileSizeHigh << 32) | stat->nFileSizeLow;
	fprintf(stderr, "\nFile Size:\n");
	fprintf(stderr, "  %-30s: %llu bytes\n", "Size", fileSize);
	fprintf(stderr, "  %-30s: %.2f KB\n", "Size (KB)", fileSize / 1024.0);
	fprintf(stderr, "  %-30s: %.2f MB\n", "Size (MB)", fileSize / (1024.0 * 1024.0));
	
	// Print Timestamps
	char timeBuffer[64];
	fprintf(stderr, "\nTimestamps:\n");
	
	FileTimeToString(stat->ftCreationTime, timeBuffer, sizeof(timeBuffer));
	fprintf(stderr, "  %-30s: %s\n", "Creation Time", timeBuffer);
	
	FileTimeToString(stat->ftLastAccessTime, timeBuffer, sizeof(timeBuffer));
	fprintf(stderr, "  %-30s: %s\n", "Last Access Time", timeBuffer);
	
	FileTimeToString(stat->ftLastWriteTime, timeBuffer, sizeof(timeBuffer));
	fprintf(stderr, "  %-30s: %s\n", "Last Write Time", timeBuffer);
	
	fprintf(stderr, "\n================================\n");
}

void filesystem_init_func_ptrs(void)
{
	SystemAdminCheck = &win_system_user_is_admin;

	Utf8PathRelativeCheck = &win_utf8_path_is_relative;
	CstrPathRelativeCheck = &win_cstr_path_is_relative;

	FileTryCreate = &win_file_try_create;
	FileTryCreateAtCwd = &win_file_try_create_at_cwd;
	FileTryOpen = &win_file_try_open;
	FileTryOpenAtCwd = &win_file_try_open_at_cwd;

	DirectoryTryCreate = &win_directory_try_create;
	DirectoryTryCreateAtCwd = &win_directory_try_create_at_cwd;
	DirectoryTryOpen = &win_directory_try_open;
	DirectoryTryOpenAtCwd = &win_directory_try_open_at_cwd;
	DirectoryPushEntries = &win_directory_push_entries;

	CwdGet = &win_cwd_get;
	CwdSet = &win_cwd_set;

	FileDump = &win_file_dump;
	FileDumpAtCwd = &win_file_dump_at_cwd;

	FileStatusFile = &win_file_status_file;
        FileStatusPath = &win_file_status_path;
	FileStatusDebugPrint = &win_file_status_debug_print;
	FileStatusGetType = win_file_status_type;

	FileClose = &win_file_close;

	FileWriteOffset = &win_file_write_offset;
	FileWriteAppend = &win_file_write_append;
	FileSync = &win_file_sync;

	FileMemoryMap = &win_file_memory_map;
	FileMemoryMapPartial = &win_file_memory_map_partial;
	FileMemoryUnmap = &win_file_memory_unmap;
	FileMemorySyncUnmap = &win_file_memory_sync_unmap;

	FileSetSize = &win_file_set_size;
}

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

#include <windows.h>
#include <pathcch.h>
#include <shlwapi.h>
#include <shlobj_core.h>
#include <stdio.h>

#include "ds_base.h"
#include "ds_platform.h"

/*
 * dir == NULL: relative to the process' current working directory (Linux's AT_FDCWD), so *AtCwd
 * calls work before ds_PlatformApiInit sets g_sys_env->cwd (ds_Init reads its config before that).
 */
static enum fsError WAbsolutePathFromRelativePathAndDirectory(WCHAR w_absolute_path[MAX_PATH], const char *relative_path, const struct file *dir)
{
	WCHAR w_directory_path[MAX_PATH];
	WCHAR w_relative_path[MAX_PATH];

	const int len_relative = MultiByteToWideChar(CP_UTF8, 0, relative_path, -1, w_relative_path, MAX_PATH);
	if (!len_relative)
	{
		LogSystemError(S_ERROR);
		return FS_PATH_INVALID;
	}

	if (dir == NULL)
	{
		const DWORD len_absolute = GetFullPathNameW(w_relative_path, MAX_PATH, w_absolute_path, NULL);
		if (len_absolute == 0)
		{
			LogSystemError(S_ERROR);
			return FS_PATH_INVALID;
		}

		/* the path did not fit: len_absolute is the size required */
		return (len_absolute < MAX_PATH) ? FS_SUCCESS : FS_PATH_INVALID;
	}

	const DWORD len_directory = GetFinalPathNameByHandleW(dir->handle, w_directory_path, MAX_PATH, FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
	if (len_directory == 0)
	{
		LogSystemError(S_ERROR);
		return FS_HANDLE_INVALID;
	}

	/* the path did not fit: len_directory is the size required, and nothing was written */
	if (MAX_PATH <= len_directory)
	{
		return FS_PATH_INVALID;
	}

	if (PathCchCombine(w_absolute_path, MAX_PATH, w_directory_path, w_relative_path) != S_OK)
	{
		return FS_PATH_INVALID;
	}

	return FS_SUCCESS;
}

u32 SystemAdminCheck(void)
{
	return IsUserAnAdmin() ? 1 : 0;
}

u32 Utf8PathRelativeCheck(const utf8 path)
{
	/* as on Linux; MultiByteToWideChar fails on 0 bytes */
	if (path.size == 0)
	{
		return 1;
	}

	const u32 req_size = (u32) Utf8SizeRequired(path);
	/* + 1: the terminator after MAX_PATH converted units */
	WCHAR w_path[MAX_PATH + 1];
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


u32 CstrPathRelativeCheck(const char *path)
{
	return (u32) PathIsRelativeA(path);
}

enum fsError FileTryCreate(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 truncate)
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
		err = WAbsolutePathFromRelativePathAndDirectory(w_absolute_path, filename, dir);
		if (err == FS_SUCCESS)
		{
			const DWORD creationDisposition = (truncate)
				? CREATE_ALWAYS
				: CREATE_NEW;

			file->handle = CreateFileW(w_absolute_path
				, GENERIC_READ | GENERIC_WRITE
				, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE
				, NULL
				, creationDisposition
				, FILE_ATTRIBUTE_NORMAL
				, NULL);

			if (file->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					/* CREATE_NEW fails with ERROR_FILE_EXISTS */
					case ERROR_FILE_EXISTS:
					case ERROR_ALREADY_EXISTS: { err = FS_ALREADY_EXISTS; } break;
					case ERROR_FILE_NOT_FOUND:
					case ERROR_PATH_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_SHARING_VIOLATION:
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					case ERROR_DIRECTORY: { err = FS_FILE_IS_NOT_DIRECTORY; } break;
					default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
				}
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

enum fsError FileTryCreateAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 truncate)
{
	return FileTryCreate(mem, file, filename, NULL, truncate);
}

enum fsError FileTryOpen(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 writeable)
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
		err = WAbsolutePathFromRelativePathAndDirectory(w_absolute_path, filename, dir);
		if (err == FS_SUCCESS)
		{
			const DWORD desiredAccess = (writeable)
				? GENERIC_READ | GENERIC_WRITE
				: GENERIC_READ;

			file->handle = CreateFileW(w_absolute_path
				, desiredAccess
				, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE
				, NULL
				, OPEN_EXISTING
				, FILE_ATTRIBUTE_NORMAL
				, NULL);

			if (file->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_FILE_NOT_FOUND:
					case ERROR_PATH_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_SHARING_VIOLATION:
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					case ERROR_DIRECTORY: { err = FS_FILE_IS_NOT_DIRECTORY; } break;
					default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
				}
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

enum fsError FileTryOpenAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 writeable)
{
	return FileTryOpen(mem, file, filename, NULL, writeable);
}

enum fsError DirectoryTryCreate(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir)
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
		err = WAbsolutePathFromRelativePathAndDirectory(w_absolute_path, filename, parent_dir);
		if (err == FS_SUCCESS)
		{
			if (CreateDirectoryW(w_absolute_path, NULL))
			{
				dir->handle = CreateFileW(w_absolute_path
					, GENERIC_READ | FILE_LIST_DIRECTORY
					, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE
					, NULL
					, OPEN_EXISTING
					, FILE_FLAG_BACKUP_SEMANTICS
					, NULL);
			}

			if (dir->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_ALREADY_EXISTS: { err = FS_ALREADY_EXISTS; } break;
					case ERROR_FILE_NOT_FOUND:
					case ERROR_PATH_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_SHARING_VIOLATION:
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					case ERROR_DIRECTORY: { err = FS_FILE_IS_NOT_DIRECTORY; } break;
					default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
				}
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

enum fsError DirectoryTryCreateAtCwd(struct arena *mem, struct file *dir, const char *filename)
{
	return DirectoryTryCreate(mem, dir, filename, NULL);
}

enum fsError DirectoryTryOpen(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir)
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
		err = WAbsolutePathFromRelativePathAndDirectory(w_absolute_path, filename, parent_dir);
		if (err == FS_SUCCESS)
		{
			dir->handle = CreateFileW(w_absolute_path
				, GENERIC_READ | FILE_LIST_DIRECTORY
				, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE
				, NULL
				, OPEN_EXISTING
				, FILE_FLAG_BACKUP_SEMANTICS
				, NULL);

			if (dir->handle == FILE_HANDLE_INVALID)
			{
				switch (GetLastError())
				{
					case ERROR_FILE_NOT_FOUND:
					case ERROR_PATH_NOT_FOUND: { err = FS_PATH_INVALID; } break;
					case ERROR_SHARING_VIOLATION:
					case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
					case ERROR_DIRECTORY: { err = FS_FILE_IS_NOT_DIRECTORY; } break;
					default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
				}
			}
			else
			{
				/* FILE_FLAG_BACKUP_SEMANTICS opens regular files too */
				BY_HANDLE_FILE_INFORMATION info;
				if (!GetFileInformationByHandle(dir->handle, &info))
				{
					err = FS_ERROR_UNSPECIFIED;
					LogSystemError(S_ERROR);
				}
				else if (!(info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
				{
					err = FS_TYPE_INVALID;
				}

				if (err != FS_SUCCESS)
				{
					FileClose(dir);
				}
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

enum fsError DirectoryTryOpenAtCwd(struct arena *mem, struct file *dir, const char *filename)
{
	return DirectoryTryOpen(mem, dir, filename, NULL); 
}

/* ReadFile and WriteFile take a DWORD size */
#define FILE_IO_CHUNK_MAX	(1u << 30)

struct dsBuffer FileDump(struct arena *mem, const char *path, const struct file *dir)
{
	/* FileTryOpen copies the path onto mem: pop it, only the contents stay */
	const struct arena record = *mem;
	struct file file = FileNull();
	if (FileTryOpen(mem, &file, path, dir, FILE_READ) != FS_SUCCESS)
	{
		return ds_buffer_empty;
	}
	*mem = record;

	struct dsBuffer buf = ds_buffer_empty;
	LARGE_INTEGER size;
	if (!GetFileSizeEx(file.handle, &size))
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		buf.data = ArenaPush(mem, (u64) size.QuadPart);
		buf.size = (buf.data) ? (u64) size.QuadPart : 0;
		u64 bytes_read = 0;
		while (bytes_read < buf.size)
		{
			const DWORD chunk = (DWORD) ((buf.size - bytes_read < FILE_IO_CHUNK_MAX) ? buf.size - bytes_read : FILE_IO_CHUNK_MAX);
			DWORD count;
			if (!ReadFile(file.handle, buf.data + bytes_read, chunk, &count, NULL))
			{
				LogSystemError(S_ERROR);
				buf = ds_buffer_empty;
				*mem = record;
				break;
			}

			/* end of file before the size: the file shrank meanwhile, keep what was read */
			if (count == 0)
			{
				buf.size = bytes_read;
				break;
			}
			bytes_read += count;
		}
		buf.mem_left = buf.size;
	}
	
	FileClose(&file);

	return buf;
}

struct dsBuffer FileDumpAtCwd(struct arena *mem, const char *path)
{
	return FileDump(mem, path, NULL);
}

u32 FileSetSize(const struct file *file, const u64 size)
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

void FileClose(struct file *file)
{
	if (!CloseHandle(file->handle))
	{
		LogSystemError(S_ERROR);
	}
	file->handle = FILE_HANDLE_INVALID;
}


/* as on Linux: no sync (FileSync), writing past the end extends the file */
u64 FileWriteOffset(const struct file *file, const u8 *buf, const u64 size, const u64 offset)
{
	if (!buf || size == 0) { return 0; }

	LARGE_INTEGER l_offset = { .QuadPart = offset };
	if (!SetFilePointerEx(file->handle, l_offset, NULL, FILE_BEGIN))
	{
		LogSystemError(S_ERROR);
		return 0;
	}

	u64 total = 0;
	while (total < size)
	{
		const DWORD chunk = (DWORD) ((size - total < FILE_IO_CHUNK_MAX) ? size - total : FILE_IO_CHUNK_MAX);
		DWORD count;
		if (!WriteFile(file->handle, buf + total, chunk, &count, NULL))
		{
			LogSystemError(S_ERROR);
			break;
		}

		total += count;
	}

	return total;
}

u64 FileWriteAppend(const struct file *file, const u8 *buf, const u64 size)
{
	if (!buf || size == 0) { return 0; }

	LARGE_INTEGER distance_to_move = { .QuadPart = 0 };
	if (!SetFilePointerEx(file->handle, distance_to_move, NULL, FILE_END))
	{
		LogSystemError(S_ERROR);
		return 0;
	}

	u64 total = 0;
	while (total < size)
	{
		const DWORD chunk = (DWORD) ((size - total < FILE_IO_CHUNK_MAX) ? size - total : FILE_IO_CHUNK_MAX);
		DWORD count;
		if (!WriteFile(file->handle, buf + total, chunk, &count, NULL))
		{
			LogSystemError(S_ERROR);
			break;
		}

		total += count;
	}

	return total;
}


void FileSync(const struct file *file)
{
	if (!FlushFileBuffers(file->handle))
	{
		LogSystemError(S_ERROR);
	}
}


void *FileMemoryMap(u64 *size, const struct file *file, const u32 prot, const u32 flags)
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
		map = FileMemoryMapPartial(file, 0, 0, prot, flags);
	}

	return map;
}

void *FileMemoryMapPartial(const struct file *file, const u64 length, const u64 offset, const u32 prot, const u32 flags)
{
	/* an execute view needs a file opened for execution, and a view without access fails */
	if ((prot & FS_PROT_EXECUTE) || prot == FS_PROT_NONE)
	{
		LogString(T_SYSTEM, S_ERROR, "FileMemoryMapPartial: FS_PROT_EXECUTE and FS_PROT_NONE are not supported on Windows");
		return NULL;
	}

	/* the section's protection and the view's access; a private view copies on write */
	const u32 write = (prot & FS_PROT_WRITE) != 0;
	const u32 copy_on_write = write && (flags & FS_MAP_PRIVATE);
	const DWORD protection = (!write) ? PAGE_READONLY : (copy_on_write) ? PAGE_WRITECOPY : PAGE_READWRITE;
	const DWORD access = (!write) ? FILE_MAP_READ : (copy_on_write) ? FILE_MAP_COPY : FILE_MAP_WRITE;

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
			/*
			 * a shared writeable section grows the file itself; a read-only or private one can't, so the file
			 * grows first then, as on Linux (SetEndOfFile fails while another view of the file is mapped)
			 */
			if (protection == PAGE_READWRITE)
			{
				new_size_high = (u32) ((offset + length) >> 32);
				new_size_low = (u32) (offset + length);
			}
			else if (!FileSetSize(file, offset + length))
			{
				return NULL;
			}
		}
	}
	void *map = NULL;
	HANDLE handle = CreateFileMappingA(file->handle, NULL, protection, new_size_high, new_size_low, NULL);
	if (handle == NULL)
	{
		LogSystemError(S_ERROR);
	}
	else
	{
		const DWORD mod = offset % info.dwAllocationGranularity;
		const DWORD offset_high = (u32) ((offset-mod) >> 32);
		const DWORD offset_low = (u32) (offset-mod);
		map = MapViewOfFileEx(handle, access, offset_high, offset_low, length + mod, NULL);

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

void FileMemoryUnmap(void *addr, const u64 length)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	const u64 mod = (u64) addr % info.dwAllocationGranularity;

	if (UnmapViewOfFile(((u8 *) addr) - mod) == 0)
	{
		LogSystemError(S_ERROR);
	}
}

void FileMemorySyncUnmap(void *addr, const u64 length)
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

utf8 CwdGet(struct arena *mem)
{
	utf8 str = Utf8Empty();

	/* the W form: the A form returns the ANSI code page, not UTF-8 */
	WCHAR w_path[MAX_PATH];
	const DWORD w_len = GetCurrentDirectoryW(MAX_PATH, w_path);
	if (w_len == 0)
	{
		LogSystemError(S_ERROR);
		return str;
	}

	/* the path did not fit: w_len is the size required */
	if (MAX_PATH <= w_len)
	{
		LogString(T_SYSTEM, S_ERROR, "CwdGet: the working directory is longer than MAX_PATH");
		return str;
	}

	const int size = WideCharToMultiByte(CP_UTF8, 0, w_path, (int) w_len, NULL, 0, NULL, NULL);
	if (size == 0)
	{
		LogSystemError(S_ERROR);
		return str;
	}

	/* terminated, as GetCurrentDirectory left it */
	u8 *buf = ArenaPush(mem, (u64) size + 1);
	if (buf)
	{
		WideCharToMultiByte(CP_UTF8, 0, w_path, (int) w_len, (char *) buf, size, NULL, NULL);
		buf[size] = '\0';
		str = (utf8) { .buf = buf, .size = (u32) size, .len = 0 };
		for (u64 offset = 0; offset < str.size; str.len += 1)
		{
			Utf8ReadCodepoint(&offset, &str, offset);
		}
	}

	return str;
}

enum fsError CwdSet(struct arena *mem, const char *path)
{
	WCHAR w_path[MAX_PATH];
	if (!MultiByteToWideChar(CP_UTF8, 0, path, -1, w_path, MAX_PATH))
	{
		LogSystemError(S_ERROR);
		return FS_PATH_INVALID;
	}

	HANDLE handle = CreateFileW(w_path
			, GENERIC_READ
			, FILE_SHARE_READ | FILE_SHARE_WRITE
			, NULL
			, OPEN_EXISTING
			, FILE_FLAG_BACKUP_SEMANTICS
			, NULL);

	enum fsError err = FS_SUCCESS;
	if (handle == INVALID_HANDLE_VALUE)
	{
		switch (GetLastError())
		{
			case ERROR_FILE_NOT_FOUND:
			case ERROR_PATH_NOT_FOUND: { err = FS_PATH_INVALID; } break;
			case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
			default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
		}
		return err;
	}

	/* FILE_FLAG_BACKUP_SEMANTICS opens regular files too */
	BY_HANDLE_FILE_INFORMATION info;
	if (!GetFileInformationByHandle(handle, &info))
	{
		err = FS_ERROR_UNSPECIFIED;
		LogSystemError(S_ERROR);
	}
	else if (!(info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
	{
		err = FS_PATH_INVALID;
	}
	else if (!SetCurrentDirectoryW(w_path))
	{
		switch (GetLastError())
		{
			case ERROR_ACCESS_DENIED: { err = FS_PERMISSION_DENIED; } break;
			default: { err = FS_ERROR_UNSPECIFIED; LogSystemError(S_ERROR); } break;
		}
	}

	/* g_sys_env->cwd changes only on success */
	if (err != FS_SUCCESS)
	{
		if (!CloseHandle(handle))
		{
			LogSystemError(S_ERROR);
		}
		return err;
	}

	g_sys_env->cwd.handle = handle;
	g_sys_env->cwd.type = FILE_DIRECTORY;
	g_sys_env->cwd.path = CwdGet(mem);

	return FS_SUCCESS;
}

enum fsError DirectoryPushEntries(struct arena *mem, ds_CPool(file) *vec, struct file *dir)
{
	WCHAR w_path[MAX_PATH];
	if (WAbsolutePathFromRelativePathAndDirectory(w_path, "*", dir) != FS_SUCCESS)
	{
		FileClose(dir);
		return FS_ERROR_UNSPECIFIED;
	}

	WIN32_FIND_DATAW file_info;
	HANDLE handle = FindFirstFileW(w_path, &file_info);
	if (handle == INVALID_HANDLE_VALUE)
	{
		LogSystemError(S_ERROR);
		FileClose(dir);
		return FS_ERROR_UNSPECIFIED;
	}
	

	ArenaPushRecord(mem);
	const u32 vec_record = vec->count;

	enum fsError ret = FS_SUCCESS;
	char cstr_filename[4*MAX_PATH];
	do
	{
		struct file *file = ds_CPoolPush(*vec).address;

		int cstr_len = WideCharToMultiByte(CP_UTF8, 0, file_info.cFileName, -1, cstr_filename, sizeof(cstr_filename), NULL, NULL);
		if (cstr_len == 0)
		{
			LogSystemError(S_ERROR);
			ret = FS_ERROR_UNSPECIFIED;
			break;
		}

		file->path = Utf8Cstr(mem, cstr_filename);
		if (file->path.len == 0)
		{
			ret = FS_BUFFER_TO_SMALL;
			break;
		}

		/* the listing's own attributes: no second lookup that can fail (pagefile.sys, an entry removed meanwhile) */
		const file_status status = { .dwFileAttributes = file_info.dwFileAttributes };
		file->type = FileStatusGetType(&status);
	} while (FindNextFileW(handle, &file_info) != 0);

	if (ret == FS_SUCCESS && GetLastError() != ERROR_NO_MORE_FILES)
	{
		LogSystemError(S_ERROR);
		ret = FS_ERROR_UNSPECIFIED;
	}

	if (ret != FS_SUCCESS)
	{
		ArenaPopRecord(mem);
        ds_CPoolPopBatch(*vec, vec->count - vec_record);
	}

	FindClose(handle);
	FileClose(dir);
	*dir = FileNull();
	return ret;
}

enum fsError FileStatusFile(file_status *status, const struct file *file)
{
	WCHAR w_path[MAX_PATH];
	const DWORD len = GetFinalPathNameByHandleW(file->handle, w_path, MAX_PATH, FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
	if (len == 0)
	{
		LogSystemError(S_ERROR);
		return FS_ERROR_UNSPECIFIED;
	}

	/* the path did not fit: len is the size required, and nothing was written */
	if (MAX_PATH <= len)
	{
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

enum fsError FileStatusPath(file_status *status, const char *path, const struct file *dir)
{	
	WCHAR w_path[MAX_PATH];
	enum fsError err = WAbsolutePathFromRelativePathAndDirectory(w_path, path, dir);
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

enum fileType FileStatusGetType(const file_status *status)
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

void FileTimeToString(FILETIME ft, char* buffer, size_t bufferSize) 
{
	SYSTEMTIME st;
	if (!FileTimeToSystemTime(&ft, &st)) 
	{
		snprintf(buffer, bufferSize, "Error converting FILETIME: %lu", GetLastError());
		return;
	}
	
	// Format as YYYY-MM-DD HH:MM:SS
	snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
	         st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
}

void FileStatusDebugPrint(const file_status *stat)
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

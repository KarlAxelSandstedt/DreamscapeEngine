/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

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

#ifndef __DS_PLATFORM_H__
#define __DS_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#include "hash_map.h"
#include "ds_vector.h"

/************************************************************************/
/* 				Platform FileIO 			*/
/************************************************************************/

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

	#include <sys/stat.h>
	#include <sys/mman.h>
	
	typedef struct stat	file_status;
	typedef int 		file_handle;
	
	#define FILE_HANDLE_INVALID 	-1
	
	#define FS_PROT_READ         PROT_READ
	#define FS_PROT_WRITE        PROT_WRITE
	#define FS_PROT_EXECUTE      PROT_EXEC
	#define FS_PROT_NONE         PROT_NONE
	
	#define FS_MAP_SHARED        MAP_SHARED
	#define FS_MAP_PRIVATE       MAP_PRIVATE
	
	
	#define FILE_READ	0
	#define FILE_WRITE	(1 << 0)
	#define FILE_TRUNCATE	(1 << 1)

#elif 

	#include <windows.h>
	
	typedef WIN32_FILE_ATTRIBUTE_DATA	file_status;
	typedef HANDLE 				file_handle;
	
	#define FILE_HANDLE_INVALID	INVALID_HANDLE_VALUE
	
	#define FS_PROT_READ         	FILE_MAP_READ
	#define FS_PROT_WRITE		FILE_MAP_WRITE
	#define FS_PROT_EXECUTE      	FILE_MAP_EXECUTE
	#define FS_PROT_NONE         	0
	
	#define FS_MAP_SHARED        	0
	#define FS_MAP_PRIVATE       	0

#endif

enum fsError
{
	FS_SUCCESS,
	FS_BUFFER_TO_SMALL,
	FS_ALREADY_EXISTS,
	FS_HANDLE_INVALID,
	FS_FILE_IS_NOT_DIRECTORY,
	FS_DIRECTORY_NOT_EMPTY,
	FS_PERMISSION_DENIED,
	FS_TYPE_INVALID,
	FS_PATH_INVALID,
	FS_ERROR_UNSPECIFIED,
	FS_COUNT
};

enum fileType
{
	FILE_NONE,
	FILE_REGULAR,
	FILE_DIRECTORY,
	FILE_UNRECOGNIZED,
	FILE_COUNT
};

struct file
{
	file_handle		handle;	/* WARNING: not necessarily opened 		*/
	enum fileType		type;	/* file type 					*/
	utf8			path;	/* context dependent: relative or absolute 	*/
};

struct file FileNull(void);

/************************************************************************/
/* 		filesystem navigation and manipulaiton			*/
/************************************************************************/

/****************************************** path operations  ****************************************/

/* returns 1 if the path is relative, otherwise 0.  */
u32		CstrPathRelativeCheck(const char *path);
u32		Utf8PathRelativeCheck(const utf8 path);

/**************************** file opening, creating, closing and dumping *************************/

/* try close file if it is open and set to FileNull  */
void 		FileClose(struct file *file);
/* Try create and open a file at the given directory; If the file already exist, error is returned. */
enum fsError 	FileTryCreate(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 truncate);
/* Try create and open a file at the cwd; If the file already exist, error is returned. */
enum fsError 	FileTryCreateAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 truncate);
/* Try open a file at the given directory; If the file does not exist, error is returned. */
enum fsError 	FileTryOpen(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 writeable);
/* Try open a file at the cwd; If the file does not exist, error is returned. */
enum fsError 	FileTryOpenAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 writeable);

/* On success, return filled buffer. On failure, set buffer to empty buffer */
struct dsBuffer FileDump(struct arena *mem, const char *path, const struct file *dir);
struct dsBuffer FileDumpAtCwd(struct arena *mem, const char *path);

/*********************************** file writing and  memory mapping *********************************/

/* return number of bytes written  */
u64 		FileWriteOffset(const struct file *file, const u8 *buf, const u64 bufsize, const u64 file_offset);
/* return number of bytes written  */
u64 		FileWriteAppend(const struct file *file, const u8 *buf, const u64 bufsize);
/* flush kernel io buffers -> up to hardware to actually store flushed io. NOTE: EXTREMELY SLOW OPERATION  */
void 		FileSync(const struct file *file);
/* returns 1 on successful size change, 0 or failure */
u32		FileSetSize(const struct file *file, const u64 size);

/* return memory mapped address of file and size of file, or NULL on failure. */
void *		FileMemoryMap(u64 *size, const struct file *file, const u32 prot, const u32 flags);
/* return memory mapped address of file, or NULL on failure   */
void *		FileMemoryMapPartial(const struct file *file, const u64 length, const u64 offset, const u32 prot, const u32 flags);
/* unmap memory mapping */
void 		FileMemoryUnmap(void *addr, const u64 length);
/* sync mmap before unmapping.  NOTE: EXTREMELY SLOW OPERATION */
void 		FileMemorySyncUnmap(void *addr, const u64 length);

/***************************** directory creation, reading and navigation *****************************/

/* Try create and open a directory at the given directory; If the file already exist, error is returned. */
enum fsError 	DirectoryTryCreate(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
/* Try create and open a directory at the cwd; If the file already exist, error is returned. */
enum fsError 	DirectoryTryCreateAtCwd(struct arena *mem, struct file *dir, const char *filename);
/* Try open a directory at the given directory; If the file does not exist, error is returned. */
enum fsError 	DirectoryTryOpen(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
/* Try open a directory at the cwd; If the file does not exist, error is returned. */
enum fsError 	DirectoryTryOpenAtCwd(struct arena *mem, struct file *dir, const char *filename);
/* push directory file paths and states AND CLOSE DIRECTORY! 
 *
 * RETURNS:
 * 	DS_FS_SUCCESS on success,
 * 	DS_FS_BUFFER_TO_SMALL on out-of-memory,
 *	DS_FS_UNSPECIFIED on errors regarding opening and reading the directory.
 */
enum fsError	DirectoryPushEntries(struct arena *mem, struct vector *vec, struct file *dir);


/*
 * directory navigator: navigation utility for reading and navigating current directory contents.
 */
struct directoryNavigator
{
	utf8		path;				/* directory path  		*/ 
	struct hashMap 	relative_path_to_file_map;	/* relative_path -> file index 	*/
	struct arena	mem_string;			/* path memory			*/
	struct vector	files;				/* file information 		*/
};

/* allocate initial memory */
struct directoryNavigator 	DirectoryNavigatorAlloc(const u32 initial_memory_string_size, const u32 hash_size, const u32 initial_hash_index_size);
/* deallocate  memory */
void				DirectoryNavigatorDealloc(struct directoryNavigator *dn);	
/* flush memory and reset data structure  */
void				DirectoryNavigatorFlush(struct directoryNavigator *dn);
/* returns number of paths containing substring. *index is set to u32[count] containing the matched indices.  */
u32 				DirectoryNavigatorLookupSubstring(struct arena *mem, u32 **index, struct directoryNavigator *dn, const utf8 substring);
/* returns file index, or if no file found, return HASH_NULL (=U32_MAX) */
u32				DirectoryNavigatorLookup(const struct directoryNavigator *dn, const utf8 filename);
/* enter given folder and update the directory_navigator state. 
 * WARNING: aliases input path.
 * RETURNS:
 * 	DS_FS_SUCCESS (= 0) on success,
 * 	DS_FS_TYPE_INVALID if specified file is not a directory,
 * 	DS_FS_PATH_INVALID if the given file does not exist,
 * 	DS_FS_PERMISSION_DENIED if user to permitted.
 */
enum fsError			DirectoryNavigatorEnterAndAliasPath(struct directoryNavigator *dn, const utf8 path);

/**************************************** file status operations ****************************************/

/* on success, set status, on error (ret_value != FS_SUCCESS), status becomes undefined */
void		FileStatusDebugPrint(const file_status *stat);
/* return file type from file status */
enum fileType	FileStatusGetType(const file_status *status);
/* Return the file status of a given file. On error ...
 * TODO: RETURNS:.. */
enum fsError	FileStatusFile(file_status *status, const struct file *file);
/* Return the file status of a given filepath. On error ...
 * TODO: RETURNS:.. */
enum fsError	FileStatusPath(file_status *status, const char *path, const struct file *dir);

/************************************* process directory operations *************************************/

/* Return the absolute path of the current working directory; string is set to empty on error. */
utf8		CwdGet(struct arena *mem);

/* Set g_sys_env->cwd and update process' internal current working directory.
 *
 * RETURNS:
 *	DS_FS_SUCCESS on success,
 *	DS_FS_PATH_INVALID if given file does not exist,
 * 	DS_FS_TYPE_INVALID if given file is not a normal directory,
 *	DS_FS_PERMISSION_DENIED if bad permissions.
 *	DS_FS_UNSPECIFIED on unexpected error.
 */
enum fsError	CwdSet(struct arena *mem, const char *path);



/************************************************************************/
/* 				System Environment			*/
/************************************************************************/

struct dsSysEnv
{
	struct file	cwd;			/* current working directory, SHOULD ONLY BE SET ONCE 	*/
	u32 		user_privileged;	/* 1 if privileged user 			 	*/ 
};

extern struct dsSysEnv *g_sys_env;

/* returns 1 if user running the process had root/administrator privileges */
u32 	SystemAdminCheck(void);

/* allocate utf8 on arena */
utf8 	Utf8GetClipboard(struct arena *mem);
void 	CstrSetClipboard(const char *utf8);

#endif

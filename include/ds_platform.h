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

/*
ds_platform.h
=============
The engine's interface to the operating system (Linux, Windows, web), implemented in src/sys: platform startup,
system randomness, files and directories, the process environment (working directory, clipboard), keyboard and
mouse codes, and system events. The graphics side (windows, GL) is in ds_graphics.h. Platform layer: its
implementation may call the C library and OS APIs directly.

Functions that can fail return enum fsError (FS_SUCCESS on success) unless their comment says otherwise. Paths are
UTF-8, relative to a directory (struct file) or to the working directory (the ...AtCwd forms).
*/

#ifndef __DS_PLATFORM_H__
#define __DS_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_base.h"
#include "ds_types.h"
#include "ds_hash_map.h"
#include "ds_list.h"

/************************************************************************/
/* 			    Platform Initialization			*/
/************************************************************************/

/*
 * Initiate/cleanup system resources: the working directory (g_sys_env->cwd; fatal if it can't be opened), the clock
 * report, and the job scheduler with thread_count workers and the given per-thread arena sizes. Called by ds_Init and
 * ds_Shutdown (ds_init.h), not by programs.
 */
void 	ds_PlatformApiInit(struct arena *mem, const u64 framesize, const u64 scratchsize, const u32 scratch_count, const u32 thread_count);
void 	ds_PlatformApiShutdown(void);


/************************************************************************/
/* 				Platform Randomness			*/
/************************************************************************/

/* 
 * Write random bytes into buf[size] using platform API. On linux, this is done by 
 * getrandom; on the Web we read from /dev/urandom; on Windows, BCryptGenRandom (system RNG).
 * Fatal if the system source fails.
 */
void	RngSystem(void *buf, const u64 size);


/************************************************************************/
/* 				Platform FileIO 			*/
/************************************************************************/

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

	#include <sys/stat.h>
	#include <sys/mman.h>
	
	typedef struct stat	file_status;
	typedef int 		file_handle;
	
	#define FILE_HANDLE_INVALID 	(-1)
	
	#define FS_PROT_READ         PROT_READ
	#define FS_PROT_WRITE        PROT_WRITE
	#define FS_PROT_EXECUTE      PROT_EXEC
	#define FS_PROT_NONE         PROT_NONE
	
	#define FS_MAP_SHARED        MAP_SHARED
	#define FS_MAP_PRIVATE       MAP_PRIVATE

#elif __DS_PLATFORM__ == __DS_WIN64__

	#include <windows.h>
	
	typedef WIN32_FILE_ATTRIBUTE_DATA	file_status;
	typedef HANDLE 				        file_handle;
	
	#define FILE_HANDLE_INVALID	    INVALID_HANDLE_VALUE
	
	#define FS_PROT_READ         	FILE_MAP_READ
	#define FS_PROT_WRITE		    FILE_MAP_WRITE
	#define FS_PROT_EXECUTE      	FILE_MAP_EXECUTE
	#define FS_PROT_NONE         	0
	
	#define FS_MAP_SHARED        	0
	#define FS_MAP_PRIVATE       	1

#endif

#define FILE_FLAGS_NONE	0
#define FILE_READ	    0
#define FILE_WRITE	    (1 << 0)
#define FILE_TRUNCATE	(1 << 1)


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
	enum fileType	type;	/* file type 					*/
	utf8			path;	/* context dependent: relative or absolute 	*/
};
DEFINE_CPOOL_STRUCT(file);

/* the closed file: handle FILE_HANDLE_INVALID, empty path, FILE_NONE */
struct file FileNull(void);

/****************************************** path operations  ****************************************/

/* returns 1 if the path is relative, otherwise 0 (Linux, web: it starts with '/'; Windows: PathIsRelativeA). */
u32		CstrPathRelativeCheck(const char *path);
u32		Utf8PathRelativeCheck(const utf8 path);

/**************************** file opening, creating, closing and dumping *************************/

/*
 * Paths are relative to dir (or the cwd): FileTryCreate, FileTryOpen, DirectoryTryCreate and DirectoryTryOpen
 * return FS_PATH_INVALID for an absolute path. Platforms differ: FileDump accepts absolute paths on Linux and web
 * (Windows goes through FileTryOpen); FileStatusPath rejects them on Linux. On Windows, the joined absolute path (dir's
 * in its "\\?\" form, then the relative one) must fit MAX_PATH (260 UTF-16 units with the terminator), else
 * FS_PATH_INVALID.
 *
 * The Try functions below expect *file closed (FILE_HANDLE_INVALID; asserted), copy the path onto mem and set the
 * type from the opened file. Errors besides those named: FS_PATH_INVALID (nothing at the path), FS_PERMISSION_DENIED,
 * FS_FILE_IS_NOT_DIRECTORY (dir or a path component is not a directory), FS_ERROR_UNSPECIFIED.
 */

/* try close file if it is open and set to FileNull  */
void 		FileClose(struct file *file);
/* Try create and open a file at the given directory, for reading and writing; If the file already exists, it is truncated if truncate is set, otherwise FS_ALREADY_EXISTS is returned. */
enum fsError 	FileTryCreate(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 truncate);
/* Try create and open a file at the cwd; If the file already exists, it is truncated if truncate is set, otherwise FS_ALREADY_EXISTS is returned. */
enum fsError 	FileTryCreateAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 truncate);
/* Try open a file at the given directory, read-only unless writeable; If the file does not exist, FS_PATH_INVALID is returned; a directory gives FS_TYPE_INVALID (DirectoryTryOpen opens those). */
enum fsError 	FileTryOpen(struct arena *mem, struct file *file, const char *filename, const struct file *dir, const u32 writeable);
/* Try open a file at the cwd; If the file does not exist, FS_PATH_INVALID is returned; a directory gives FS_TYPE_INVALID. */
enum fsError 	FileTryOpenAtCwd(struct arena *mem, struct file *file, const char *filename, const u32 writeable);

/*
 * On success, return filled buffer (on mem). On failure, set buffer to empty buffer (mem unchanged); an empty file
 * also gives the empty buffer. A file shorter than its size claims (it shrank meanwhile, pseudo-files) gives what
 * was read.
 */
struct dsBuffer FileDump(struct arena *mem, const char *path, const struct file *dir);
struct dsBuffer FileDumpAtCwd(struct arena *mem, const char *path);

/*********************************** file writing and  memory mapping *********************************/

/* write buf at file_offset; return number of bytes written (fewer on an error, which is logged; 0 if buf is empty) */
u64 		FileWriteOffset(const struct file *file, const u8 *buf, const u64 bufsize, const u64 file_offset);
/* write buf at the end of the file; return number of bytes written (fewer on an error, which is logged; 0 if buf is empty) */
u64 		FileWriteAppend(const struct file *file, const u8 *buf, const u64 bufsize);
/* flush kernel io buffers -> up to hardware to actually store flushed io. NOTE: EXTREMELY SLOW OPERATION  */
void 		FileSync(const struct file *file);
/* returns 1 on successful size change, 0 on failure */
u32		FileSetSize(const struct file *file, const u64 size);

/*
 * prot: FS_PROT_READ, or FS_PROT_READ | FS_PROT_WRITE (file opened writeable). flags: FS_MAP_SHARED (writes reach the
 * file) or FS_MAP_PRIVATE (copy-on-write: writes stay in the mapping). Windows doesn't support FS_PROT_EXECUTE or
 * FS_PROT_NONE: the mapping fails (NULL).
 */

/* return memory mapped address of the whole file and set *size to the file's size; NULL on failure. */
void *		FileMemoryMap(u64 *size, const struct file *file, const u32 prot, const u32 flags);
/* return memory mapped address of file[offset, offset + length), growing the file first if it is shorter; any offset (aligned internally on every platform); NULL on failure */
void *		FileMemoryMapPartial(const struct file *file, const u64 length, const u64 offset, const u32 prot, const u32 flags);
/* unmap memory mapping */
void 		FileMemoryUnmap(void *addr, const u64 length);
/*
 * sync mmap before unmapping.  NOTE: EXTREMELY SLOW OPERATION
 * Linux waits until the changes are written back (msync MS_SYNC); Windows only starts writing them (FlushViewOfFile):
 * FileSync on the file afterwards waits for storage.
 */
void 		FileMemorySyncUnmap(void *addr, const u64 length);

/***************************** directory creation, reading and navigation *****************************/

/* Try create and open a directory at the given directory; If the file already exist, FS_ALREADY_EXISTS is returned. */
enum fsError 	DirectoryTryCreate(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
/* Try create and open a directory at the cwd; If the file already exist, FS_ALREADY_EXISTS is returned. */
enum fsError 	DirectoryTryCreateAtCwd(struct arena *mem, struct file *dir, const char *filename);
/* Try open a directory at the given directory; FS_PATH_INVALID if it does not exist, FS_TYPE_INVALID if it is not a directory. */
enum fsError 	DirectoryTryOpen(struct arena *mem, struct file *dir, const char *filename, const struct file *parent_dir);
/* Try open a directory at the cwd; FS_PATH_INVALID if it does not exist, FS_TYPE_INVALID if it is not a directory. */
enum fsError 	DirectoryTryOpenAtCwd(struct arena *mem, struct file *dir, const char *filename);
/* push directory file paths and states AND CLOSE DIRECTORY (on every path)! Every entry, "." and ".." included,
 * gets its name (on mem) and type; a link has its target's type, and entries whose status can't be read (dangling
 * links) are FILE_UNRECOGNIZED.
 *
 * RETURNS:
 * 	FS_SUCCESS on success,
 * 	FS_BUFFER_TO_SMALL on out-of-memory,
 *	FS_ERROR_UNSPECIFIED on errors regarding opening and reading the directory.
 * On failure, vec and mem are as before.
 */
enum fsError	DirectoryPushEntries(struct arena *mem, ds_CPool(file) *vec, struct file *dir);


/*
 * directory navigator: navigation utility for reading and navigating current directory contents.
 */
struct directoryNavigator
{
	utf8		path;				/* directory path  		*/ 
	struct ds_HashMap 	relative_path_to_file_map;	/* relative_path -> file index 	*/
	struct arena	mem_string;			/* path memory			*/
	ds_CPool(file)	files;				/* file information 		*/
};

/* allocate initial memory (heap) */
struct directoryNavigator 	DirectoryNavigatorAlloc(const u32 initial_memory_string_size, const u32 hash_size, const u32 initial_hash_index_size);
/* deallocate  memory */
void				DirectoryNavigatorDealloc(struct directoryNavigator *dn);	
/* flush memory and reset data structure  */
void				DirectoryNavigatorFlush(struct directoryNavigator *dn);
/* returns number of paths containing substring. *index is set to u32[count] (on mem) containing the matched indices.  */
u32 				DirectoryNavigatorLookupSubstring(struct arena *mem, u32 **index, struct directoryNavigator *dn, const utf8 substring);
/* returns file index, or if no file found, return HASH_NULL (=U32_MAX) */
u32				DirectoryNavigatorLookup(const struct directoryNavigator *dn, const utf8 filename);
/* enter given folder (relative to the cwd) and update the directory_navigator state. 
 * WARNING: aliases input path (no copy: path must outlive the navigator's use of it).
 * RETURNS:
 * 	FS_SUCCESS (= 0) on success,
 * 	FS_TYPE_INVALID if specified file is not a directory,
 * 	FS_PATH_INVALID if the given file does not exist,
 * 	FS_PERMISSION_DENIED if user not permitted.
 */
enum fsError			DirectoryNavigatorEnterAndAliasPath(struct directoryNavigator *dn, const utf8 path);

/**************************************** file status operations ****************************************/

/* print the file status to stderr (debugging) */
void		FileStatusDebugPrint(const file_status *stat);
/* return file type from file status */
enum fileType	FileStatusGetType(const file_status *status);
/* Fill the file status of a given open file (opened through a link: its target). Returns FS_SUCCESS or
 * FS_ERROR_UNSPECIFIED (status undefined then). */
enum fsError	FileStatusFile(file_status *status, const struct file *file);
/* Fill the file status of a given filepath relative to dir, links followed (a dangling link fails). Returns FS_SUCCESS,
 * FS_ERROR_UNSPECIFIED (status undefined then), or on Linux FS_PATH_INVALID for an absolute path. */
enum fsError	FileStatusPath(file_status *status, const char *path, const struct file *dir);

/************************************* process directory operations *************************************/

/* Return the absolute path of the current working directory (on mem); string is set to empty on error, also when mem is too small. */
utf8		CwdGet(struct arena *mem);

/* Set g_sys_env->cwd and update process' internal current working directory. On failure, both stay as they were.
 *
 * RETURNS:
 *	FS_SUCCESS on success,
 *	FS_PATH_INVALID if given file does not exist or is not a directory,
 *	FS_PERMISSION_DENIED if bad permissions.
 *	FS_ERROR_UNSPECIFIED on unexpected error.
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
u32 		SystemAdminCheck(void);

/* allocate utf8 on arena; set by the graphics backend (graphics only, not headless) */
extern utf8 	(*Utf8GetClipboard)(struct arena *mem);
/* set the clipboard's text; set by the graphics backend (graphics only, not headless) */
extern void 	(*CstrSetClipboard)(const char *utf8);


/************************************************************************/
/* 			system mouse/keyboard handling 			*/
/************************************************************************/

#define	KEY_MOD_NONE	0
#define	KEY_MOD_LSHIFT	(1 << 0)
#define	KEY_MOD_RSHIFT	(1 << 1)
#define	KEY_MOD_LCTRL	(1 << 2)
#define	KEY_MOD_RCTRL	(1 << 3)
#define	KEY_MOD_LALT	(1 << 4)
#define	KEY_MOD_RALT	(1 << 5)	/* Alt Gr? 		*/
#define	KEY_MOD_LGUI	(1 << 6)	/* Left windows-key?	*/
#define	KEY_MOD_RGUI	(1 << 7)	/* Right windows-key? 	*/
#define KEY_MOD_NUM	(1 << 8)	/* Num-lock  		*/ 
#define KEY_MOD_CAPS	(1 << 9)	
#define KEY_MOD_ALTGR	(1 << 10)	
#define KEY_MOD_SCROLL	(1 << 11)	/* Scroll-lock */

#define KEY_MOD_SHIFT	(KEY_MOD_LSHIFT | KEY_MOD_RSHIFT)
#define KEY_MOD_CTRL	(KEY_MOD_LCTRL | KEY_MOD_RCTRL)
#define KEY_MOD_ALT	(KEY_MOD_LALT | KEY_MOD_RALT)
#define KEY_MOD_GUI	(KEY_MOD_LGUI | KEY_MOD_RGUI)

enum dsKeycode
{
	DS_SHIFT,
	DS_CTRL,
	DS_SPACE,
	DS_BACKSPACE,
	DS_ESCAPE,
	DS_ENTER,
	DS_F1,
	DS_F2,
	DS_F3,
	DS_F4,
	DS_F5,
	DS_F6,
	DS_F7,
	DS_F8,
	DS_F9,
	DS_F10,
	DS_F11,
	DS_F12,
	DS_TAB,
	DS_UP,
	DS_DOWN,
	DS_LEFT,
	DS_RIGHT,
	DS_DELETE,
	DS_PLUS,
	DS_MINUS,
	DS_HOME,
	DS_END,
	DS_0,
	DS_1,
	DS_2,
	DS_3,
	DS_4,
	DS_5,
	DS_6,
	DS_7,
	DS_8,
	DS_9,
	DS_A,
	DS_B,  
	DS_C, 
	DS_D, 
	DS_E, 
	DS_F, 
	DS_G, 
	DS_H, 
	DS_I, 
	DS_J, 
	DS_K, 
	DS_L, 
	DS_M, 
	DS_N, 
	DS_O, 
	DS_P, 
	DS_Q, 
	DS_R, 
	DS_S, 
	DS_T, 
	DS_U, 
	DS_V, 
	DS_W, 
	DS_X, 
	DS_Y, 
	DS_Z, 
	DS_NO_SYMBOL,
	DS_KEY_COUNT
};

enum mouseButton
{
	MOUSE_BUTTON_LEFT,
	MOUSE_BUTTON_RIGHT,
	MOUSE_BUTTON_SCROLL,
	MOUSE_BUTTON_NONMAPPED,
	MOUSE_BUTTON_COUNT
};

enum mouseScroll
{
	MOUSE_SCROLL_UP,
	MOUSE_SCROLL_DOWN,
	MOUSE_SCROLL_COUNT
};

/* the current KEY_MOD_* bits; set by the graphics backend (graphics only, not headless) */
extern u32	(*KeyModifiers)(void);
	
/* the enumerator's name (static string, for debugging) */
const char *	CstrDsKeycode(const enum dsKeycode key);
const char *	CstrButton(const enum mouseButton button);


/************************************************************************/
/* 			      system events 				*/
/************************************************************************/

enum ds_EventType 
{
	DS_SCROLL,
	DS_KEY_PRESSED,
	DS_KEY_RELEASED,
	DS_BUTTON_PRESSED,
	DS_BUTTON_RELEASED,
	DS_CURSOR_POSITION,
	DS_TEXT_INPUT,
	DS_WINDOW_CLOSE,
	DS_WINDOW_CURSOR_ENTER,
	DS_WINDOW_CURSOR_LEAVE,
	DS_WINDOW_FOCUS_IN,
	DS_WINDOW_FOCUS_OUT,
	DS_WINDOW_EXPOSE,
	DS_WINDOW_CONFIG,
	DS_WINDOW_MINIMIZE,
	DS_NO_EVENT,
};

struct ds_Event 
{
    POOL_NODE;
    struct DLLNode   node;
	u64			native_handle;	/* window handle 			*/
	u64			ns_timestamp;	/* external event time; NOT OUR CLOCK 	*/
	enum ds_EventType 	type;

	/* Input key */
	enum dsKeycode keycode; 	
	enum dsKeycode scancode;

	/* Input Mouse button */
	enum mouseButton button; 

	/* mouse scrolling */
	struct 
	{
		enum mouseScroll direction;
		u32 count;
	} scroll;

	v2 native_cursor_window_position;	/* native window coordinate space cursor position */
	v2 native_cursor_window_delta;	/* native window coordinate space cursor delta */

	utf8	utf8;
};
DEFINE_POOL(ds_Event);

/* process native window events and update corresponding system window states (graphics only; once per frame) */
void	ds_ProcessEvents(void);

#ifdef __cplusplus
}
#endif


#endif

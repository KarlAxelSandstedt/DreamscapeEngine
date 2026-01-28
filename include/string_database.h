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

#ifndef __STRING_DATABASE_H__
#define __STRING_DATABASE_H__

#include "hash_map.h"
#include "list.h"

#define STRING_DATABASE_STUB_INDEX	0

/*
 * internal database struct state - Place inside of any structure to be stored within the database.
 */
#define STRING_DATABASE_SLOT_STATE									\
	utf8 				id;			/* identifier of database object */	\
	u32				reference_count;	/* Number of references to slot  */	\
	DLL3_SLOT_STATE;					/* allocated list state	         */	\
	POOL_SLOT_STATE						/* pool slot internal state      */

#define strdb_Next(structure_addr)	dll3_Next(structure_addr)
#define strdb_Prev(structure_addr)	dll3_Prev(structure_addr)
#define strdb_InList(structure_addr)	dll3_InList(structure_addr)

/*
 *	1. id aliasing: on deallocation, do nothing with identifier, 
 *		(up to user to free, BUT WE MUST ENSURE LIFETIME ATLEAST AS LONG AS DATABASE)
 *	2. malloc copy: on deallocation, free identifier 
 *	3. arena copy:  on deallocation, do nothing with identifier, (up to user to free)
 */
struct strdb
{
	struct hashMap 	hash;
	struct pool	pool;
	struct dll	allocated_dll;
	u64		id_offset;		/* id offset within db structure 	    */
	u64		reference_count_offset; /* ref_count offset within db structure     */
	u64		allocated_prev_offset;	/* (dll) previous allocated index offset     */
	u64		allocated_next_offset;	/* (dll) next allocated index offset 	     */
};

/* allocate and return database with entries of data_size (simply sizeof(struct)). 
 * If growable, allows the database to increase size when required. */
struct strdb	strdb_AllocInternal(struct arena *mem, const u32 hash_size, const u32 index_size, const u64 data_size, const u64 id_offset, const u64 reference_count_offset, const u64 allocated_prev_offset, const u64 allocated_next_offset, const u64 pool_state_offset, const u32 growable);
#define 		strdb_Alloc(mem, hash_size, index_size, STRUCT, growable)			\
			strdb_AllocInternal(mem,							\
				       		       hash_size,					\
						       index_size, 					\
						       sizeof(STRUCT),					\
						       ((u64)&((STRUCT *)0)->id),			\
						       ((u64)&((STRUCT *)0)->reference_count),		\
						       ((u64)&((STRUCT *)0)->dll3_prev),		\
						       ((u64)&((STRUCT *)0)->dll3_next),		\
						       ((u64)&((STRUCT *)0)->slot_allocation_state),	\
						       growable)
/* free the database. NOTE that none of the database id strings are freed as they are either aliases or arena memory. */
void		strdb_Dealloc(struct strdb *db);
/* flush or reset the string database */
void		strdb_Flush(struct strdb *db);
/* allocate a new database node with the given identifier and return its index (handle). 
   The id will be copied onto the arena. On failure, the stub slot (0, NULL) is returned. 
   the reference count is set to 0. */
struct slot	strdb_Add(struct arena *mem_db_lifetime, struct strdb *db, const utf8 id);
/* allocate a new database node with the given identifier and return its index. 
   The id will alias the given string's content. On failure, the stub slot (0, NULL) is returned. 
   the reference count is set to 0. */
struct slot	strdb_AddAndAlias(struct strdb *db, const utf8 id);
/* remove the identifier's corresponding database node if found and update database state, otherwise do nothing. */
void		strdb_Remove(struct strdb *db, const utf8 id);
/* Lookup the identifer in the database. If it exist, return its slot. Otherwise return (0, NULL). */
struct slot	strdb_Lookup(const struct strdb *db, const utf8 id);
/* Return the corresponding address of the index. */
void *		strdb_Address(const struct strdb *db, const u32 handle);
/* Return the result of the lookup operation. furthermore, if the returned slot is not (0, NULL), increment the corresponding node's reference count.  */
struct slot 	strdb_Reference(struct strdb *db, const utf8 id);
/* Lookup the handle in the database. If it exist, decrement the corresponding node's reference count. */
void		strdb_Dereference(struct strdb *db, const u32 handle);

#endif

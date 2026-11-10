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

struct ring RingEmpty()
{
	return (struct ring) { .mem_total = 0, .mem_left = 0, .offset = 0, .buf = NULL };
}

#if __DS_PLATFORM__ == __DS_LINUX__

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

struct ring RingAlloc(const u64 mem_hint)
{
	ds_Assert(mem_hint);

	/* whole pages (page_size is a power of two) */
	const u64 page = g_mem_config->page_size;
	const u64 mem_total = PowerOfTwoMultipleCeil(mem_hint, page);

	/* one memory object mapped twice (shared): writes through either view show in both */
	const int fd = memfd_create("ds_ring", MFD_CLOEXEC);
	if (fd == -1)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to create the ring's memory: %s", strerror(errno));
		return RingEmpty();
	}

	if (ftruncate(fd, (off_t) mem_total) == -1)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to size the ring's memory: %s", strerror(errno));
		close(fd);
		return RingEmpty();
	}

	/* reserve both halves, then map the object over each */
	u8 *buf = mmap(NULL, 2*mem_total, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (buf == MAP_FAILED)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to reserve the ring's address range: %s", strerror(errno));
		close(fd);
		return RingEmpty();
	}

	if (mmap(buf, mem_total, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, 0) == MAP_FAILED
		|| mmap(buf + mem_total, mem_total, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, 0) == MAP_FAILED)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to map the ring's views: %s", strerror(errno));
		munmap(buf, 2*mem_total);
		close(fd);
		return RingEmpty();
	}

	/* the mappings keep the object alive */
	close(fd);

	return (struct ring) { .mem_total = mem_total, .mem_left = mem_total, .offset = 0, .buf = buf };
}

void RingDealloc(struct ring *ring)
{
	if (munmap(ring->buf, 2*ring->mem_total) == -1)
	{
		Log(T_SYSTEM, S_ERROR, "%s:%i - %s", __FILE__, __LINE__, strerror(errno));
	}
	*ring = RingEmpty();
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <memoryapi.h>

struct ring RingAlloc(const u64 mem_hint)
{
	ds_Assert(mem_hint);

	SYSTEM_INFO info;
	GetSystemInfo(&info);

	u64 bufsize = PowerOfTwoCeil(mem_hint);
	if (bufsize < info.dwAllocationGranularity)
	{
		bufsize = info.dwAllocationGranularity;
	}
	u8 *alloc = VirtualAlloc2(NULL, NULL, 2*bufsize, MEM_RESERVE | MEM_RESERVE_PLACEHOLDER, PAGE_NOACCESS, NULL, 0);
	if (alloc == NULL)
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	/* split into two placeholders, [alloc, alloc + bufsize) and [alloc + bufsize, alloc + 2*bufsize) */
	if (!VirtualFree(alloc, bufsize, MEM_RELEASE | MEM_PRESERVE_PLACEHOLDER))
	{
		LogSystemError(S_ERROR);
		VirtualFree(alloc, 0, MEM_RELEASE);
		return RingEmpty();
	}

	/* fails with NULL, not INVALID_HANDLE_VALUE */
	HANDLE map = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, (DWORD) (bufsize >> 32), (DWORD) ((u32) bufsize), NULL);
	if (map == NULL)
	{
		LogSystemError(S_ERROR);
		goto release_placeholders;
	}

	u8 *buf = MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0);
	if (buf == NULL)
	{
		LogSystemError(S_ERROR);
		goto close_map;
	}

	if (MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc + bufsize, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0) == NULL)
	{
		LogSystemError(S_ERROR);
		UnmapViewOfFile(buf);
		CloseHandle(map);
		VirtualFree(alloc + bufsize, 0, MEM_RELEASE);
		return RingEmpty();
	}

	/* the views keep the mapping alive */
	CloseHandle(map);

	return (struct ring) { .mem_total = bufsize, .mem_left = bufsize, .offset = 0, .buf = buf };

close_map:
	CloseHandle(map);
release_placeholders:
	VirtualFree(alloc, 0, MEM_RELEASE);
	VirtualFree(alloc + bufsize, 0, MEM_RELEASE);
	return RingEmpty();
}

void RingDealloc(struct ring *ring)
{
	if (!UnmapViewOfFile(ring->buf))
	{
		LogSystemError(S_ERROR);
	}
	if (!UnmapViewOfFile(ring->buf + ring->mem_total))
	{
		LogSystemError(S_ERROR);
	}
	*ring = RingEmpty();
}

#endif

void RingFlush(struct ring *ring)
{
	ring->mem_left = ring->mem_total;
	ring->offset = 0;
}

struct ringSlot RingPushStart(struct ring *ring, const u64 size)
{
	struct ringSlot buf = { 0 };
	if (size && size <= ring->mem_left)
	{
		ring->mem_left -= size;
		buf.address = ring->buf + ((ring->offset + ring->mem_left) % ring->mem_total);
		buf.size = size;
	}

	return buf;
}

struct ringSlot RingPushEnd(struct ring *ring, const u64 size)
{
	struct ringSlot buf = { 0 };
	if (size && size <= ring->mem_left)
	{
		buf.address = ring->buf + ring->offset;
		buf.size = size;
		ring->mem_left -= size;
		ring->offset = (ring->offset + size) % ring->mem_total;
	}

	return buf;
}

void RingPopStart(struct ring *ring, const u64 size)
{
	ds_Assert(size + ring->mem_left <= ring->mem_total);
	ring->mem_left += size;
}

void RingPopEnd(struct ring *ring, const u64 size)
{
	ds_Assert(size + ring->mem_left <= ring->mem_total);
	ring->mem_left += size;
	ring->offset = (ring->mem_total + ring->offset - size) % ring->mem_total;
}

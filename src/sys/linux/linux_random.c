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
#include <stdlib.h>
#include "ds_base.h"

#if __DS_PLATFORM__ == __DS_LINUX__

#include <sys/random.h>

void RngSystem(void *buf, const u64 size)
{
	u64 bytes_left = size;
	while (bytes_left)
	{
		const u64 offset = size - bytes_left;
		const ssize_t bytes_read = getrandom(((u8*) buf) + offset, bytes_left, 0);
		if (bytes_read == -1)
		{
			LogSystemError(S_FATAL);
			FatalCleanupAndExit();
		}
		bytes_left -= (u64) bytes_read;
	}
}

#elif __DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <fcntl.h>

void RngSystem(void *buf, const u64 size)
{
	int fd = open("/dev/urandom", O_RDONLY);
	if (fd == -1)
	{
		LogSystemError(S_FATAL);
		FatalCleanupAndExit();
	}

	u64 bytes_left = size;
	while (bytes_left)
	{
		const u64 offset = size - bytes_left;
		const ssize_t bytes_read = read(fd, ((u8*) buf) + offset, bytes_left);
		if (bytes_read == -1)
		{
			LogSystemError(S_FATAL);
			FatalCleanupAndExit();
		}
		bytes_left -= (u64) bytes_read;
	}

	close(fd);
}

#endif

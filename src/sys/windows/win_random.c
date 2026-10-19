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
#include <windows.h>
#include <bcrypt.h>

#include "ds_base.h"



void RngSystem(void *buf, const u64 size)
{
	/* as on Linux; BCryptGenRandom fails on a NULL buffer even at size 0 */
	if (size == 0)
	{
		return;
	}

	BCRYPT_ALG_HANDLE hAlgorithm;
	LPCWSTR pszAlgId = BCRYPT_RNG_ALGORITHM;
	LPCWSTR pszImplementation = NULL;
	ULONG dwFlags = 0;
	NTSTATUS status = BCryptOpenAlgorithmProvider(&hAlgorithm, pszAlgId, pszImplementation, dwFlags);
	if (!BCRYPT_SUCCESS(status))
	{
		Log(T_SYSTEM, S_FATAL, "Couldn't open algorithm provider: %08x\n", status);
		FatalCleanupAndExit();
	}

	/* BCryptGenRandom takes a ULONG size */
	u64 bytes_left = size;
	while (bytes_left)
	{
		const u64 offset = size - bytes_left;
		const ULONG chunk = (ULONG) ((bytes_left < U32_MAX) ? bytes_left : U32_MAX);
		status = BCryptGenRandom(hAlgorithm, ((u8 *) buf) + offset, chunk, 0);
		if (!BCRYPT_SUCCESS(status))
		{
			Log(T_SYSTEM, S_FATAL, "Couldn't generate BCrypt randomness: %08x\n", status);
			FatalCleanupAndExit();
		}
		bytes_left -= chunk;
	}

	status = BCryptCloseAlgorithmProvider(hAlgorithm, 0);
	if (!BCRYPT_SUCCESS(status))
	{
		Log(T_SYSTEM, S_FATAL, "Couldn't close algorithm provider: %08x\n", status);
		FatalCleanupAndExit();
	}
}

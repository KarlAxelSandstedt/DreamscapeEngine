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

#include "led_local.h"

u32 led_FilenameValid(const utf8 filename)
{
	u32 valid = 1;
	for (u32 i = 0; i < filename.len; ++i)
	{
		if (WordbreakCheck(filename.buf[i]))
		{
			valid = 0;
			break;
		}
	}

	return valid; 
}

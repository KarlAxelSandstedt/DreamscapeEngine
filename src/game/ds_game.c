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

#include "ds_base.h"
#include "ds_game.h"

void ds_GameAlloc(struct ds_Game *game)
{
}

void ds_GameDealloc(struct ds_Game *game)
{
}

void ds_GameFlush(struct ds_Game *game)
{
}

void ds_GameTick(struct ds_Game *game, struct ds_Dynamics *dynamics, const u64 ns_tick)
{
}

u64 ds_GameHash(const struct ds_Game *game)
{
    return 0;
}

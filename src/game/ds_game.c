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
    game->unit_map = ds_HashMapAlloc(NULL, 1024, 1024, GROWABLE);
    ds_CPoolAlloc(NULL, game->joint_unit_pool, 8, GROWABLE);
}

void ds_GameDealloc(struct ds_Game *game)
{
    ds_HashMapDealloc(&game->unit_map);
    ds_CPoolDealloc(game->joint_unit_pool);
}

void ds_GameFlush(struct ds_Game *game)
{
    game->spawned = 0;
    game->mover = (struct ds_GeometricMover) { 0 };
    game->throttle = V2(0.0f, 0.0f);
    game->yaw = 0.0f;
    ds_HashMapFlush(&game->unit_map);
    ds_CPoolFlush(game->joint_unit_pool);
}

void ds_GameTick(struct ds_Game *game, struct ds_Dynamics *dynamics, const u64 ns_tick)
{
}

u64 ds_GameHash(const struct ds_Game *game)
{
    return 0;
}

/* TODO(Remove/Refactor) */
ds_BodyId ds_GameBodyByUnit(const struct ds_Game *game, const struct ds_Dynamics *dynamics, const u32 unit)
{
    for (u32 i = ds_HashMapFirst(&game->unit_map, unit); i != HASH_NULL; i = ds_HashMapNext(&game->unit_map, i))
    {
        const struct ds_Body *body = dynamics->body_pool.buf + i;
        if (ds_PoolSlotAllocated(body) && body->entity == unit)
        {
            return ds_BodyIdInit(i, body->generation);
        }
    }

    return ds_BodyIdNull();
}

/* TODO(Remove/Refactor) */
ds_JointId ds_GameJointByUnit(const struct ds_Game *game, const struct ds_Dynamics *dynamics, const u32 joint_unit)
{
    for (u32 i = 0; i < game->joint_unit_pool.count; ++i)
    {
        const u64 entry = game->joint_unit_pool.buf[i];
        if ((u32) (entry >> 32) == joint_unit)
        {
            const struct ds_Joint *joint = dynamics->joint_pool.buf + (u32) entry;
            return (ds_PoolSlotAllocated(joint)) ? joint->id : ds_JointIdNull();
        }
    }

    return ds_JointIdNull();
}

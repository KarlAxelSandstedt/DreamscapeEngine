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
ds_game.h
=========
*/

#ifndef __DS_GAME_H__
#define __DS_GAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"
#include "ds_dynamics.h"

/*
ds_Game
=======
*/
struct ds_Game
{
    /* TODO(Remove/Refactor) temporary state below */
    u32                 spawned;
    struct ds_GeometricMover mover;
    v2                  throttle;
    f32                 yaw;
    u32                 jump;
    u32                 on_ground;
    f32                 pogo_velocity;
    struct ds_HashMap   entity_map;         /* TODO(Remove/Refactor) entity -> body pool index, for the commands and the host */
    ds_CPool(u64)       joint_entity_pool;  /* TODO(Remove/Refactor) joint entity << 32 | joint pool index */
};

void    ds_GameAlloc(struct ds_Game *game);
void    ds_GameDealloc(struct ds_Game *game);
void    ds_GameFlush(struct ds_Game *game);
void    ds_GameTick(struct ds_Game *game, struct ds_Dynamics *dynamics, const u64 ns_tick);
u64     ds_GameHash(const struct ds_Game *game);

/* TODO(Remove/Refactor) Return the body added for entity (SIM_CMD_BODY_ADD), or the null id. */
ds_BodyId  ds_GameBodyByEntity(const struct ds_Game *game, const struct ds_Dynamics *dynamics, const u32 entity);
/* TODO(Remove/Refactor) Return the joint added for the joint entity (SIM_CMD_MOUSE_JOINT_ADD), or the null id. */
ds_JointId ds_GameJointByEntity(const struct ds_Game *game, const struct ds_Dynamics *dynamics, const u32 joint_entity);

#ifdef __cplusplus
}
#endif

#endif

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
#include "xxhash.h"

/* TODO(Remove/Refactor) the mover's tuning, box3d's GeometricMoverDef: speeds in m/s, rates in 1/s, gravity in m/s^2 */
#define DS_GAME_MOVER_MAX_SPEED     6.0f
#define DS_GAME_MOVER_MIN_SPEED     0.01f
#define DS_GAME_MOVER_STOP_SPEED    1.0f
#define DS_GAME_MOVER_ACCELERATE    30.0f
#define DS_GAME_MOVER_FRICTION      4.0f
#define DS_GAME_MOVER_GRAVITY       15.0f
#define DS_GAME_MOVER_JUMP_SPEED    5.0f
#define DS_GAME_MOVER_POGO_REST     0.35f
#define DS_GAME_MOVER_POGO_HERTZ    4.0f
#define DS_GAME_MOVER_POGO_DAMPING  0.7f

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
    game->jump = 0;
    game->on_ground = 0;
    game->pogo_velocity = 0.0f;
    ds_HashMapFlush(&game->unit_map);
    ds_CPoolFlush(game->joint_unit_pool);
}

void ds_GameTick(struct ds_Game *game, struct ds_Dynamics *dynamics, const u64 ns_tick)
{
    if (!game->spawned)
    {
        return;
    }

    const f32 h = (f32) ((f64) ns_tick / 1000000000.0);
    v3 v = game->mover.velocity;

    /* a jump only leaves the ground; the request is used up either way */
    if (game->jump && game->on_ground)
    {
        v.y = DS_GAME_MOVER_JUMP_SPEED;
        game->on_ground = 0;
    }
    game->jump = 0;

    /* friction, horizontal only: proportional above the stop speed, constant below it */
    const f32 speed = V3Length(v);
    if (speed < DS_GAME_MOVER_MIN_SPEED)
    {
        v.x = 0.0f;
        v.z = 0.0f;
    }
    else
    {
        const f32 drop = F32Max(speed, DS_GAME_MOVER_STOP_SPEED) * DS_GAME_MOVER_FRICTION * h;
        const f32 ratio = F32Max(0.0f, speed - drop) / speed;
        v.x *= ratio;
        v.z *= ratio;
    }

    if (game->on_ground)
    {
        v.y = 0.0f;
    }

    /* accelerate toward the throttle (x right, y forward) in the frame of the yaw */
    const f32 c = F32Cos(game->yaw);
    const f32 s = F32Sin(game->yaw);
    const v3 forward = V3(s, 0.0f, c);
    const v3 right = V3(-c, 0.0f, s);
    const v3 wish = V3Scale(V3Add(V3Scale(right, game->throttle.x), V3Scale(forward, game->throttle.y)), DS_GAME_MOVER_MAX_SPEED);
    const f32 wish_length = V3Length(wish);
    if (wish_length > 0.0f)
    {
        const v3 direction = V3Scale(wish, 1.0f / wish_length);
        const f32 wish_speed = F32Min(wish_length, DS_GAME_MOVER_MAX_SPEED);
        const f32 add = wish_speed - V3Dot(v, direction);
        if (add > 0.0f)
        {
            v = V3AddScaled(v, direction, F32Min(DS_GAME_MOVER_ACCELERATE * DS_GAME_MOVER_MAX_SPEED * h, add));
        }
    }

    v.y -= DS_GAME_MOVER_GRAVITY * h;

    /*
     * pogo: a ray down from the lower core point; on a hit within rest + radius a spring-damper holds that point
     * the rest length above the ground; off while moving up, so a jump is not pulled back
     */
    const f32 ray_length = DS_GAME_MOVER_POGO_REST + game->mover.capsule.radius;
    const struct rayWorld ray = { .origin = V3F64Sub(game->mover.position, V3F64(0.0, (f64) game->mover.capsule.half_height, 0.0)), .dir = V3(0.0f, -1.0f, 0.0f) };
    const u32f32 hit = ds_DynamicsRayCastParameter(dynamics, &ray);
    if (hit.u == U32_MAX || hit.f > ray_length || v.y > 0.0f)
    {
        game->on_ground = 0;
        game->pogo_velocity = 0.0f;
    }
    else
    {
        game->on_ground = 1;
        const f32 offset = hit.f - DS_GAME_MOVER_POGO_REST;
        const f32 omega = 2.0f * F32_PI * DS_GAME_MOVER_POGO_HERTZ;
        const f32 omega_h = omega * h;
        game->pogo_velocity = (game->pogo_velocity - omega * omega_h * offset) / (1.0f + 2.0f * DS_GAME_MOVER_POGO_DAMPING * omega_h + omega_h * omega_h);
    }

    game->mover.velocity = v;
    ds_DynamicsGeometricMover(dynamics, &game->mover, V3Scale(V3Add(v, V3(0.0f, game->pogo_velocity, 0.0f)), h));
}

u64 ds_GameHash(const struct ds_Game *game)
{
    if (!game->spawned)
    {
        return 0;
    }

    const struct ds_GeometricMover *mover = &game->mover;
    const u64 words[16] =
    {
        game->spawned,
        (b64) { .f = mover->position.x }.u,
        (b64) { .f = mover->position.y }.u,
        (b64) { .f = mover->position.z }.u,
        (b32) { .f = mover->velocity.x }.u,
        (b32) { .f = mover->velocity.y }.u,
        (b32) { .f = mover->velocity.z }.u,
        (b32) { .f = mover->capsule.half_height }.u,
        (b32) { .f = mover->capsule.radius }.u,
        mover->apply_impulse,
        (b32) { .f = game->throttle.x }.u,
        (b32) { .f = game->throttle.y }.u,
        (b32) { .f = game->yaw }.u,
        game->jump,
        game->on_ground,
        (b32) { .f = game->pogo_velocity }.u,
    };
    return XXH3_64bits(words, sizeof(words));
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

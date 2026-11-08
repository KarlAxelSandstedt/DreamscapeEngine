/*
sim_cmd_tmp.c
=============
TODO(Remove/Refactor): the temporary commands (everything but ConfigLoad), included by sim_cmd.c. They name
bodies and joints by author-chosen entities (ds_Sim.entity_map, joint_entity_pool) until the id scheme is decided.
*/

/* a prefab id: non-empty and within the database's buffer */
static u32 ds_SimCmdPrefabIdCheck(const utf8 id)
{
    return id.size >= 1 && id.size < PREFAB_BUFSIZE;
}

/* a prefab id: 8 bits of size, then its bytes */
#define DS_SIM_CMD_ID_BITS(id)      (8 + 8*(id).size)

/* the id's bytes go on mem; 0 if the stream is short or mem is full */
static u32 ds_SimCmdTryReadId(struct arena *mem, struct SS *ss, utf8 *id)
{
    if (SSBitsLeft(ss) < 8)
    {
        return 0;
    }

    const u32 size = (u32) SSReadU64Partial(ss, 8);
    if (SSBitsLeft(ss) < 8*size)
    {
        return 0;
    }

    u8 *buf = ArenaPush(mem, (size) ? size : 1);
    if (buf == NULL)
    {
        return 0;
    }

    for (u32 i = 0; i < size; ++i)
    {
        buf[i] = (u8) SSReadU64Partial(ss, 8);
    }
    *id = (utf8) { .buf = buf, .size = size, .len = size };
    return 1;
}

/******** BodyAdd ********/

static u32 ds_BodyAddCmdFormVerify(const struct ds_BodyAddCmd *cmd)
{
    return ds_SimCmdPrefabIdCheck(cmd->prefab) && ds_TransformWorldFiniteCheck(&cmd->t_world);
}

static u32 ds_BodyAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_BodyAddCmd *cmd)
{
    return ds_SDBLookup(*sim->dynamics.external->body_prefab_db, cmd->prefab).index != SDB_STUB
        && ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity));
}

static u32 ds_BodyAddCmdTryExecute(struct ds_Sim *sim, const struct ds_BodyAddCmd *cmd)
{
    if (!ds_BodyAddCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_BodyAddCmdTryExecute: rejected, unknown prefab %.*s or entity %u taken", cmd->prefab.size, cmd->prefab.buf, cmd->entity);
        return 0;
    }

    const struct ds_BodyPrefab *prefab = ds_SDBLookup(*sim->dynamics.external->body_prefab_db, cmd->prefab).address;
    const ds_BodyId body = ds_BodyAdd(&sim->dynamics, prefab, &cmd->t_world, cmd->entity);
    if (ds_BodyIdNullCheck(body))
    {
        Log(T_SYSTEM, S_ERROR, "ds_BodyAddCmdTryExecute: no body for entity %u", cmd->entity);
        return 0;
    }
    ds_HashMapAdd(&sim->entity_map, cmd->entity, ds_BodyIdIndex(body));
    return 1;
}

static u64 ds_BodyAddCmdSerializeBitPackedSize(const struct ds_BodyAddCmd *cmd)
{
    return DS_SIM_CMD_ID_BITS(cmd->prefab) + ds_TransformWorldSerializeBitPackedSize(&cmd->t_world) + 32;
}

static void ds_BodyAddCmdSerializeBitPacked(struct SS *ss, const struct ds_BodyAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->prefab.size, 8);
    for (u32 i = 0; i < cmd->prefab.size; ++i)
    {
        SSWriteU64Partial(ss, cmd->prefab.buf[i], 8);
    }
    ds_TransformWorldSerializeBitPacked(ss, &cmd->t_world);
    SSWriteU64Partial(ss, cmd->entity, 32);
}

static u32 ds_BodyAddCmdTryDeserializeBitPacked(struct arena *mem, struct SS *ss, struct ds_BodyAddCmd *cmd)
{
    if (!ds_SimCmdTryReadId(mem, ss, &cmd->prefab) || SSBitsLeft(ss) < ds_TransformWorldSerializeBitPackedSize(&cmd->t_world) + 32)
    {
        return 0;
    }

    ds_TransformWorldDeserializeBitPacked(ss, &cmd->t_world);
    cmd->entity = (u32) SSReadU64Partial(ss, 32);
    return 1;
}

/******** ShapeAdd ********/

static u32 ds_ShapeAddCmdFormVerify(const struct ds_ShapeAddCmd *cmd)
{
    return ds_SimCmdPrefabIdCheck(cmd->prefab) && ds_TransformFiniteCheck(&cmd->t_local);
}

static u32 ds_ShapeAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_ShapeAddCmd *cmd)
{
    return ds_SDBLookup(*sim->dynamics.external->shape_prefab_db, cmd->prefab).index != SDB_STUB
        && !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity));
}

static u32 ds_ShapeAddCmdTryExecute(struct ds_Sim *sim, const struct ds_ShapeAddCmd *cmd)
{
    if (!ds_ShapeAddCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_ShapeAddCmdTryExecute: rejected, unknown prefab %.*s or no body for entity %u", cmd->prefab.size, cmd->prefab.buf, cmd->entity);
        return 0;
    }

    const struct ds_ShapePrefab *prefab = ds_SDBLookup(*sim->dynamics.external->shape_prefab_db, cmd->prefab).address;
    ds_ShapeAdd(&sim->dynamics, prefab, &cmd->t_local, ds_SimBodyByEntity(sim, cmd->entity));
    return 1;
}

static u64 ds_ShapeAddCmdSerializeBitPackedSize(const struct ds_ShapeAddCmd *cmd)
{
    return DS_SIM_CMD_ID_BITS(cmd->prefab) + ds_TransformSerializeBitPackedSize(&cmd->t_local) + 32;
}

static void ds_ShapeAddCmdSerializeBitPacked(struct SS *ss, const struct ds_ShapeAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->prefab.size, 8);
    for (u32 i = 0; i < cmd->prefab.size; ++i)
    {
        SSWriteU64Partial(ss, cmd->prefab.buf[i], 8);
    }
    ds_TransformSerializeBitPacked(ss, &cmd->t_local);
    SSWriteU64Partial(ss, cmd->entity, 32);
}

static u32 ds_ShapeAddCmdTryDeserializeBitPacked(struct arena *mem, struct SS *ss, struct ds_ShapeAddCmd *cmd)
{
    if (!ds_SimCmdTryReadId(mem, ss, &cmd->prefab) || SSBitsLeft(ss) < ds_TransformSerializeBitPackedSize(&cmd->t_local) + 32)
    {
        return 0;
    }

    ds_TransformDeserializeBitPacked(ss, &cmd->t_local);
    cmd->entity = (u32) SSReadU64Partial(ss, 32);
    return 1;
}

/******** JointAdd ********/

#define DS_SIM_CMD_JOINT_ADD_BITS   (32 + 32 + 2*32 + 32 + 2*32 + 2*32 + 2*32 + 2*8*28)

static u32 ds_JointAddCmdFormVerify(const struct ds_JointAddCmd *cmd)
{
    return F32FiniteCheck(cmd->target) && F32FiniteCheck(cmd->range.low) && F32FiniteCheck(cmd->range.high)
        && F32FiniteCheck(cmd->hertz) && F32FiniteCheck(cmd->damping_ratio)
        && !F32TestNan(cmd->breakage.force_threshold) && !F32TestNan(cmd->breakage.torque_threshold)   /* infinity: unbreakable */
        && cmd->range.low <= cmd->target && cmd->target <= cmd->range.high && cmd->hertz >= 0.0f && cmd->damping_ratio >= 0.0f
        && cmd->entity[0] != cmd->entity[1]
        && ds_TransformFiniteCheck(cmd->local_frame + 0) && ds_TransformFiniteCheck(cmd->local_frame + 1);
}

static u32 ds_JointAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_JointAddCmd *cmd)
{
    return !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity[0]))
        && !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity[1]));
}

static u32 ds_JointAddCmdTryExecute(struct ds_Sim *sim, const struct ds_JointAddCmd *cmd)
{
    if (!ds_JointAddCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_JointAddCmdTryExecute: rejected, no body for entity %u or %u", cmd->entity[0], cmd->entity[1]);
        return 0;
    }

    struct ds_DistanceJointPrefab prefab;
    if (cmd->enable_spring)
    {
        ds_DistanceJointPrefabRangeSpring(&prefab, cmd->range, cmd->hertz, cmd->damping_ratio, cmd->breakage);
    }
    else
    {
        ds_DistanceJointPrefabRange(&prefab, cmd->range, cmd->breakage);
    }
    prefab.target = F32Clamp(cmd->target, prefab.range.low, prefab.range.high);
    prefab.collide_connected = cmd->collide_connected;
    ds_DistanceJointAdd(&sim->dynamics, &prefab, ds_SimBodyByEntity(sim, cmd->entity[0]), cmd->local_frame + 0, ds_SimBodyByEntity(sim, cmd->entity[1]), cmd->local_frame + 1);
    return 1;
}

static u64 ds_JointAddCmdSerializeBitPackedSize(const struct ds_JointAddCmd *cmd)
{
    return DS_SIM_CMD_JOINT_ADD_BITS;
}

static void ds_JointAddCmdSerializeBitPacked(struct SS *ss, const struct ds_JointAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->collide_connected, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->target }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->range.low }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->range.high }.u, 32);
    SSWriteU64Partial(ss, cmd->enable_spring, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->hertz }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->damping_ratio }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->breakage.force_threshold }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->breakage.torque_threshold }.u, 32);
    SSWriteU64Partial(ss, cmd->entity[0], 32);
    SSWriteU64Partial(ss, cmd->entity[1], 32);
    ds_TransformSerializeBitPacked(ss, cmd->local_frame + 0);
    ds_TransformSerializeBitPacked(ss, cmd->local_frame + 1);
}

static u32 ds_JointAddCmdTryDeserializeBitPacked(struct SS *ss, struct ds_JointAddCmd *cmd)
{
    if (SSBitsLeft(ss) < DS_SIM_CMD_JOINT_ADD_BITS)
    {
        return 0;
    }

    cmd->collide_connected = (u32) SSReadU64Partial(ss, 32);
    cmd->target = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->range.low = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->range.high = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->enable_spring = (u32) SSReadU64Partial(ss, 32);
    cmd->hertz = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->damping_ratio = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->breakage.force_threshold = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->breakage.torque_threshold = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->entity[0] = (u32) SSReadU64Partial(ss, 32);
    cmd->entity[1] = (u32) SSReadU64Partial(ss, 32);
    ds_TransformDeserializeBitPacked(ss, cmd->local_frame + 0);
    ds_TransformDeserializeBitPacked(ss, cmd->local_frame + 1);
    return 1;
}

/******** MouseJointAdd ********/

static u32 ds_MouseJointAddCmdFormVerify(const struct ds_MouseJointAddCmd *cmd)
{
    return V3F64FiniteCheck(cmd->target)
        && F32FiniteCheck(cmd->hertz) && F32FiniteCheck(cmd->damping_ratio) && F32FiniteCheck(cmd->force_scale) && F32FiniteCheck(cmd->torque_scale)
        && cmd->hertz >= 0.0f && cmd->damping_ratio >= 0.0f && cmd->force_scale >= 0.0f && cmd->torque_scale >= 0.0f;
}

static u32 ds_MouseJointAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_MouseJointAddCmd *cmd)
{
    const struct ds_Body *body = ds_BodyLookup(&sim->dynamics, ds_SimBodyByEntity(sim, cmd->entity)).address;
    return body && ds_BodyDynamicCheck(body) && ds_JointIdNullCheck(ds_SimJointByEntity(sim, cmd->joint_entity));
}

static u32 ds_MouseJointAddCmdTryExecute(struct ds_Sim *sim, const struct ds_MouseJointAddCmd *cmd)
{
    if (!ds_MouseJointAddCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_MouseJointAddCmdTryExecute: rejected, entity %u has no dynamic body or joint entity %u is taken", cmd->entity, cmd->joint_entity);
        return 0;
    }

    const ds_BodyId id = ds_SimBodyByEntity(sim, cmd->entity);
    const struct ds_Body *body = ds_BodyLookup(&sim->dynamics, id).address;
    const struct ds_BodySim *bsim = sim->dynamics.solver_set_pool.buf[body->set].body_sim_pool.buf + body->sim;
    ds_Transform local_frame = ds_TransformIdentity();
    local_frame.position = ds_TransformWorldPointToLocal(&bsim->world, cmd->target);

    struct ds_MouseJointPrefab prefab;
    ds_MouseJointPrefabFromBody(&prefab, &sim->dynamics, id, cmd->target, cmd->hertz, cmd->damping_ratio, cmd->force_scale, cmd->torque_scale, ds_BreakageOptionNone());
    const ds_JointId joint = ds_MouseJointAdd(&sim->dynamics, &prefab, id, &local_frame);
    if (ds_JointIdNullCheck(joint))
    {
        Log(T_SYSTEM, S_ERROR, "ds_MouseJointAddCmdTryExecute: no joint for entity %u", cmd->joint_entity);
        return 0;
    }
    ds_CPoolPushValue(sim->joint_entity_pool, ((u64) cmd->joint_entity << 32) | ds_JointIdIndex(joint));
    return 1;
}

#define DS_SIM_CMD_MOUSE_JOINT_ADD_BITS (32 + 32 + 3*64 + 4*32)

static u64 ds_MouseJointAddCmdSerializeBitPackedSize(const struct ds_MouseJointAddCmd *cmd)
{
    return DS_SIM_CMD_MOUSE_JOINT_ADD_BITS;
}

static void ds_MouseJointAddCmdSerializeBitPacked(struct SS *ss, const struct ds_MouseJointAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->joint_entity, 32);
    SSWriteU64Partial(ss, cmd->entity, 32);
    V3F64SerializeBitPacked(ss, cmd->target);
    SSWriteU64Partial(ss, (b32) { .f = cmd->hertz }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->damping_ratio }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->force_scale }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->torque_scale }.u, 32);
}

static u32 ds_MouseJointAddCmdTryDeserializeBitPacked(struct SS *ss, struct ds_MouseJointAddCmd *cmd)
{
    if (SSBitsLeft(ss) < DS_SIM_CMD_MOUSE_JOINT_ADD_BITS)
    {
        return 0;
    }

    cmd->joint_entity = (u32) SSReadU64Partial(ss, 32);
    cmd->entity = (u32) SSReadU64Partial(ss, 32);
    V3F64DeserializeBitPacked(ss, &cmd->target);
    cmd->hertz = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->damping_ratio = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->force_scale = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->torque_scale = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    return 1;
}

/******** MouseJointTargetSet ********/

static u32 ds_MouseJointTargetSetCmdFormVerify(const struct ds_MouseJointTargetSetCmd *cmd)
{
    return V3F64FiniteCheck(cmd->target);
}

static u32 ds_MouseJointTargetSetCmdDataVerify(const struct ds_Sim *sim, const struct ds_MouseJointTargetSetCmd *cmd)
{
    const struct ds_Joint *joint = ds_JointLookup(&sim->dynamics, ds_SimJointByEntity(sim, cmd->joint_entity)).address;
    return joint && joint->type == DS_JOINT_TYPE_MOUSE;
}

static u32 ds_MouseJointTargetSetCmdTryExecute(struct ds_Sim *sim, const struct ds_MouseJointTargetSetCmd *cmd)
{
    if (!ds_MouseJointTargetSetCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_MouseJointTargetSetCmdTryExecute: rejected, joint entity %u is no mouse joint", cmd->joint_entity);
        return 0;
    }

    ds_MouseJointTargetSet(&sim->dynamics, ds_SimJointByEntity(sim, cmd->joint_entity), cmd->target);
    return 1;
}

#define DS_SIM_CMD_MOUSE_JOINT_TARGET_SET_BITS (32 + 3*64)

static u64 ds_MouseJointTargetSetCmdSerializeBitPackedSize(const struct ds_MouseJointTargetSetCmd *cmd)
{
    return DS_SIM_CMD_MOUSE_JOINT_TARGET_SET_BITS;
}

static void ds_MouseJointTargetSetCmdSerializeBitPacked(struct SS *ss, const struct ds_MouseJointTargetSetCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->joint_entity, 32);
    V3F64SerializeBitPacked(ss, cmd->target);
}

static u32 ds_MouseJointTargetSetCmdTryDeserializeBitPacked(struct SS *ss, struct ds_MouseJointTargetSetCmd *cmd)
{
    if (SSBitsLeft(ss) < DS_SIM_CMD_MOUSE_JOINT_TARGET_SET_BITS)
    {
        return 0;
    }

    cmd->joint_entity = (u32) SSReadU64Partial(ss, 32);
    V3F64DeserializeBitPacked(ss, &cmd->target);
    return 1;
}

/******** JointRemove ********/

static u32 ds_JointRemoveCmdFormVerify(const struct ds_JointRemoveCmd *cmd)
{
    return 1;
}

static u32 ds_JointRemoveCmdDataVerify(const struct ds_Sim *sim, const struct ds_JointRemoveCmd *cmd)
{
    return !ds_JointIdNullCheck(ds_SimJointByEntity(sim, cmd->joint_entity));
}

static u32 ds_JointRemoveCmdTryExecute(struct ds_Sim *sim, const struct ds_JointRemoveCmd *cmd)
{
    if (!ds_JointRemoveCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_JointRemoveCmdTryExecute: rejected, no joint for joint entity %u", cmd->joint_entity);
        return 0;
    }

    const ds_JointId joint = ds_SimJointByEntity(sim, cmd->joint_entity);
    for (u32 i = 0; i < sim->joint_entity_pool.count; ++i)
    {
        if ((u32) (sim->joint_entity_pool.buf[i] >> 32) == cmd->joint_entity)
        {
            ds_CPoolRemoveAndSwap(sim->joint_entity_pool, i);
            break;
        }
    }
    ds_JointRemove(&sim->dynamics, joint);
    return 1;
}

static u64 ds_JointRemoveCmdSerializeBitPackedSize(const struct ds_JointRemoveCmd *cmd)
{
    return 32;
}

static void ds_JointRemoveCmdSerializeBitPacked(struct SS *ss, const struct ds_JointRemoveCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->joint_entity, 32);
}

static u32 ds_JointRemoveCmdTryDeserializeBitPacked(struct SS *ss, struct ds_JointRemoveCmd *cmd)
{
    if (SSBitsLeft(ss) < 32)
    {
        return 0;
    }

    cmd->joint_entity = (u32) SSReadU64Partial(ss, 32);
    return 1;
}


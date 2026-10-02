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
Serialization and Recording API TODO(Under Construction)
===============================
Used for serialiing/deserializing the physics state.

Data-structure serialization plans:
-----------------------------------
    Basic containers (each needs XSerializeSize / XSerialize / XDeserialize):

        Slot pools (POOL_DECLARE, ds_allocator.h)                       NEW
            ds_Body, ds_Shape, ds_Joint, ds_Contact, ds_Island,
            ds_SolverSet, ds_PhysicsEvent, bvhNode.
            Probably macro-generated like the rest of the pool API.

        CPools (ds_CPool(T), ds_allocator.h)                            NEW
            dirty_shape_query; 6 inside each ds_SolverSet, 3 inside
            each ds_CGraphColor.

        BitSets (ds_bitset.h)                                           NEW
            5 in ds_Dynamics, leaf_set/internal_set in each bvh,
            body_bitset in each ds_CGraphColor.

        HashMap (ds_hash_map.h)                                         UPDATE
            contact_map. ds_HashMapSerialize exists, but: no Size
            function; it silently writes nothing if the data doesn't fit
            (should assume it fits + ds_Assert); Deserialize allocates a
            new map (need: deserialize into an existing map).

        DLL (ds_list.h)                                                 TRIVIAL
            Header only {count, first, last}; the nodes live inside pool
            elements and are restored with them.

        minQueue (bvh cost_queue)                                       NONE?
            Only used during BVH insertion and empty between ticks
            (bvh.c:233-238). Verify; then only its capacity matters.

    Composite structures:
        bvh (dynamic_bvh, static_bvh): bt {root, count}, leaf_set,
            internal_set, bvhNode pool, cost_queue.
        ds_CGraph: CG_COLOR_COUNT x { body_bitset, 3 CPools }.
        ds_SolverSet: pool elements OWN 6 CPools, so the solver set pool
            needs per-element serialization; copying the struct is not
            enough.

    Pointers to non-pool memory (cannot be serialized as raw bytes):
        ds_Contact.narrowphase (c_ContactResult): manifold, cache, tri and
            tri_manifold point into the thread frame arenas (alive for 2
            frames, double buffered), or the heap for sleeping contacts.
            Serialize by value and re-point on restore. => The frame
            arenas ARE state, at least for contacts.
        ds_ContactCompute.cc/ccache (in solver set and color CPools):
            pointers; verify whether they are rebuilt every tick.



Design Issues:
-------------
    1. Determinism: When we serialize, say, a non-compact pool, how do we deal with
       the sparseness? The smartest thing to do would probably be to simply have each
       body stay on exactly its assigned slot. Any indices held to it will the hold 
       after deserialization. But how do we deal with the free-list? Should we be allowed
       to change its order? Probably not, as this allocated slots in different order on
       runs. If determinism is dependent on the order of allocations, we run into issues.

    2. How do we handle errors when serializing?
        We don't. We assume it data fits. Ideally, any size checks should be done before the
        Serialize call. This gives us two API calls, for each struct / data type

        // ds_Assert size requirements, and assume the data fits.
        ds_DataSerialize(&stream, & const data) 
        // Pure function, return the size requirement.
        ds_DataSerializeSize(& const data) 

    3. How do we handle errors when deserializing?
        /TODO   (trusted vs. untrused), ERRORS vs. FATAL is on a per-system basis.


Example usage: repeat a tick on identical input.
------------------------------------------------

    {
        ArenaPushRecord(mem);

        const u64 size = ds_DynamicsSerializeSize(pipeline);
        struct ss snap = ss_Alloc(mem, size);
        if (snap.buf == NULL)
        {
            LogString(T_PHYSICS, S_FATAL, "Failed to allocate physics snapshot");
            FatalCleanupAndExit();
        }
        ds_DynamicsSerialize(&snap, pipeline);

        while (measuring)
        {
            snap.bit_index = 0;                         // rewind
            ds_DynamicsDeserialize(pipeline, &snap);    // restore
            ds_DynamicsTick(pipeline);
        }

        ArenaPopRecord(mem);
    }
 */

u64 ds_DynamicsSerializeSize(const struct ds_Dynamics *pipeline)
{
}

void ds_DynamicsSerialize(struct ss *ss, const struct ds_Dynamics *pipeline)
{
}

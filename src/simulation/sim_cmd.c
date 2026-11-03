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
sim_cmd.c
=========
The external commands: every state change an external source (a user, the editor, the network) imposes on the
simulation. Commands arrive in ds_Sim.external_cmd_buf, are stamped as one batch per frame, recorded, and executed
at the batch's slot before any system tick of that frame.

The first command, ConfigLoad: a serialized ds_SimConfig. It is the seed batch at time 0 (the record's head
re-applied) and, later, the host's apply of its pending edits; executing it derives the config (every value clamped
into its range) and applies it to the systems (ds_DynamicsConfigUpdatePending for the dynamics). The alloc-time
values (seed, tick periods, capacities) are ignored after time 0. TODO: implement.
*/

#include "ds_base.h"
#include "ds_simulation.h"

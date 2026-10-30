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

#define XXH_INLINE_ALL
#include "xxhash.h"

#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"
#include "ds_quaternion.h"

#include "ds_dynamics.h"
#include "ds_snapshot.c"
#include "ds_pipeline.c"
#include "ds_metrics.c"
#include "ds_contact.c"
#include "ds_solver.c"
#include "ds_island.c"
#include "ds_shape.c"
#include "ds_body.c"
#include "ds_numerics.c"
#include "ds_joint.c"
#include "ds_constraint_graph.c"
#include "ds_solver_set.c"

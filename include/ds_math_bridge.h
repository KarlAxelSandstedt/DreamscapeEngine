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

#ifndef __DS_MATH_BRIDGE_H__
#define __DS_MATH_BRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_define.h"
#include "ds_types.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_matrix.h"

/*
 * TEMPORARY migration helpers: load/store data that still uses the old vec3/quat/mat3 types
 * (e.g. ds_Transform, c_Manifold, c_Shape, solverConfig). Delete this header once the migration is complete.
 */
static ds_ForceInline v3 V3Load(const vec3 a)
{
	return V3(a[0], a[1], a[2]);
}

static ds_ForceInline void V3Store(vec3 dst, const v3 a)
{
	dst[0] = a.x;
	dst[1] = a.y;
	dst[2] = a.z;
}

static ds_ForceInline v4 V4Load(const vec4 a)
{
	return V4(a[0], a[1], a[2], a[3]);
}

static ds_ForceInline q QLoad(const quat a)
{
	return Q(a[0], a[1], a[2], a[3]);
}

static ds_ForceInline void QStore(quat dst, const q a)
{
	dst[0] = a.x;
	dst[1] = a.y;
	dst[2] = a.z;
	dst[3] = a.w;
}

static ds_ForceInline m3 M3Load(mat3 a)
{
	return M3Columns(V3Load(a[0]), V3Load(a[1]), V3Load(a[2]));
}

static ds_ForceInline void M3Store(mat3 dst, const m3 a)
{
	V3Store(dst[0], a.col[0]);
	V3Store(dst[1], a.col[1]);
	V3Store(dst[2], a.col[2]);
}

#ifdef __cplusplus
}
#endif

#endif

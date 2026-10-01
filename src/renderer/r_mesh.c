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

#include <string.h>

#include "r_local.h"

SDB_DEFINE(r_Mesh);

f32 stub_vertices[] =
{
		 0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 
		-0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		-0.5f,  0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		 0.5f,  0.5f, -0.5f, 0.0f, 0.0f, -1.0f,
		-0.5f, -0.5f,  0.5f, 0.0f, 0.0f,  1.0f,
		 0.5f, -0.5f,  0.5f, 0.0f, 0.0f,  1.0f,
		 0.5f,  0.5f,  0.5f, 0.0f, 0.0f,  1.0f,
		-0.5f,  0.5f,  0.5f, 0.0f, 0.0f,  1.0f,

		 0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 
		-0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f,
		-0.5f,  0.5f, -0.5f, 0.0f,  1.0f, 0.0f,
		 0.5f,  0.5f, -0.5f, 0.0f,  1.0f, 0.0f,
		-0.5f, -0.5f,  0.5f, 0.0f, -1.0f, 0.0f,
		 0.5f, -0.5f,  0.5f, 0.0f, -1.0f, 0.0f,
		 0.5f,  0.5f,  0.5f, 0.0f,  1.0f, 0.0f,
		-0.5f,  0.5f,  0.5f, 0.0f,  1.0f, 0.0f,

		 0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f, 
		-0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f,
		-0.5f,  0.5f, -0.5f, -1.0f, 0.0f, 0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f,  0.5f, -1.0f, 0.0f, 0.0f,
		 0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
		-0.5f,  0.5f,  0.5f, -1.0f, 0.0f, 0.0f,
};

u32 stub_indices[] = 
{
	0, 1, 2, 0, 2, 3,
	6, 7, 4, 6, 4, 5,
	8 + 3, 8 + 2, 8 + 7, 8 + 3, 8 + 7, 8 + 6,
	8 + 5, 8 + 4, 8 + 1, 8 + 5, 8 + 1, 8 + 0,
	16 + 5, 16 + 0, 16 + 3, 16 + 5, 16 + 3, 16 + 6,
	16 + 1, 16 + 4, 16 + 7, 16 + 1, 16 + 7, 16 + 2,
};

void r_MeshStubBox(struct r_Mesh *mesh_stub)
{
	mesh_stub->index_max_used = 16 + 7;
	mesh_stub->index_count = sizeof(stub_indices) / sizeof(stub_indices[0]); 
	mesh_stub->index_data = stub_indices;
	mesh_stub->vertex_count = sizeof(stub_vertices) / sizeof(stub_vertices[0]);
	mesh_stub->vertex_data = stub_vertices;
	mesh_stub->local_stride = sizeof(stub_vertices[0]);
}

static void r_InternalMeshSphere(u32 *b_i, u8 *vertex_data, u32 *index_data, const f32 radius, const v3 translation, const u32 refinement)
{
	const u32 points_per_strip = 2 * refinement;
	const u32 num_strips = refinement;
	const f32 inc_angle = F32_PI / refinement;

	v3 vertex = V3Add(V3(0.0f, radius, 0.0f), translation);
	v3 n = V3(0.0f, 1.0f, 0.0f);

	u64 offset = 0;
	const u64 p_offset = 0;
	/* const u64 c_offset = sizeof(v3); */
	const u64 n_offset = sizeof(v3) /* + sizeof(v4)*/;

	memcpy(vertex_data + offset + p_offset, &vertex, sizeof(v3));
	/* memcpy(vertex_data + offset + c_offset, &color, sizeof(v4)); */
	memcpy(vertex_data + offset + n_offset, &n, sizeof(v3));
	offset += sizeof(v3) /* + sizeof(v4) */ + sizeof(v3);

	for (u32 i = 1; i < num_strips; ++i)
	{
		const f32 k = inc_angle * i;
		for (u32 j = 0; j < points_per_strip; ++j)
		{
			const f32 t = inc_angle * j;
			vertex = V3Scale(V3Normalize(V3(F32Cos(t), 0.0f, -F32Sin(t))), F32Sin(k));
			vertex.y = F32Cos(k);
			n = V3Normalize(vertex);
			vertex = V3Add(V3Scale(vertex, radius), translation);

			memcpy(vertex_data + offset + p_offset, &vertex, sizeof(v3));
			//memcpy(vertex_data + offset + c_offset, &color, sizeof(v4));
			memcpy(vertex_data + offset + n_offset, &n, sizeof(v3));
			offset += sizeof(v3) /*+ sizeof(v4)*/ + sizeof(v3);
		}
	}

	vertex = V3Add(V3(0.0f, -radius, 0.0f), translation);
	n = V3Normalize(vertex);
	memcpy(vertex_data + offset + p_offset, &vertex, sizeof(v3));
	//memcpy(vertex_data + offset + c_offset, &color, sizeof(v4));
	memcpy(vertex_data + offset + n_offset, &n, sizeof(v3));
	offset += sizeof(v3) /*+ sizeof(v4) */ + sizeof(v3);

	*b_i = 0;	
	u32 k = 0;

	for (u32 i = 0; i < points_per_strip; ++i)
	{
		index_data[k++] = *b_i + 1 + ((i + 1) % points_per_strip);
		index_data[k++] = *b_i;
		index_data[k++] = *b_i + i + 1;
	}

	*b_i += 1;
	for (u32 i = 1; i < (num_strips-1); ++i)
	{
		*b_i += points_per_strip;
		for (u32 j = 0; j < points_per_strip; ++j)
		{
			index_data[k++] = *b_i + ((j + 1) % points_per_strip);
			index_data[k++] = *b_i + j - points_per_strip;
			index_data[k++] = *b_i + j;
			index_data[k++] = *b_i + ((j + 1) % points_per_strip);
			index_data[k++] = *b_i + ((j + 1) % points_per_strip) - points_per_strip;
			index_data[k++] = *b_i + j - points_per_strip;
		}
	}

	for (u32 i = 0; i < points_per_strip; ++i)
	{
		index_data[k++] = *b_i + points_per_strip;
		index_data[k++] = *b_i + ((i + 1) % points_per_strip);
		index_data[k++] = *b_i + i;
	}

	*b_i += points_per_strip;
}

/* const_circle_points - number of vertices on single circle of sphere */
void r_MeshSphere(struct arena *mem, struct r_Mesh *mesh, const f32 radius, const u32 refinement)
{
	ds_Assert(refinement >= 3);

	const u32 points_per_strip = 2 * refinement;
	const u32 num_strips = refinement;
	const f32 inc_angle = F32_PI / refinement;

	const u32 vertex_count = 2 + (num_strips - 1) * points_per_strip;
	const u64 vertex_size = sizeof(v3) + /*sizeof(v4) */+ sizeof(v3);
	u8 *vertex_data = ArenaPush(mem, vertex_count * vertex_size);

	const u32 index_count = 2 * 3 * points_per_strip + (num_strips-1-1) * points_per_strip * 6;
	u32 *index_data = ArenaPush(mem, index_count * sizeof(u32));

	u32 max_used = 0;
	const v3 translation = V3Zero();
	r_InternalMeshSphere(&max_used, vertex_data, index_data, radius, translation, refinement);

	mesh->index_max_used = max_used;
	mesh->index_count = index_count;
	mesh->index_data = index_data;
	mesh->vertex_count = vertex_count;
	mesh->vertex_data = (void *) vertex_data;
	mesh->local_stride = vertex_size;
}

void r_MeshCapsule(struct arena *mem, struct r_Mesh *mesh, const f32 half_height, const f32 radius, const u32 refinement)
{
	ds_Assert(refinement >= 2);
	ds_Assert(half_height > 0.0f && radius > 0.0f);

	const u32 n_long_slice = 2*refinement;
	const u32 n_lat_cap_slice = refinement;
	//TODO
	const u32 n_lat_cyl_slice = refinement;
	
	struct memArray arr = ArenaPushAlignedAll(mem, sizeof(v3), 4);
	v3 *v = arr.addr;

	//TODO
	const u32 n = 2*n_lat_cap_slice*n_long_slice + n_lat_cyl_slice*n_long_slice + 2;
	if (arr.len < n)
	{
		ArenaPopPacked(mem, arr.memPushed);
		r_MeshStubBox(mesh);
		return;
	}
	
	u32 vi = 0;
	v[vi++] = V3(0.0f, -half_height, 0.0f);
	v[vi++] = V3(0.0f, half_height, 0.0f);

	for (u32 i = 0; i < n_lat_cap_slice; ++i)
	{
		const f32 theta = (i + 1)*(F32_PI / 2.0f) / n_lat_cap_slice;
		const f32 ring_radius = radius * F32Sin(theta);
		const f32 y = -half_height - radius * F32Cos(theta);
		for (u32 j = 0; j < n_long_slice; ++j)
		{
			const f32 phi = j * 2.0f * F32_PI / n_long_slice;
			v[vi++] = V3(ring_radius * F32Cos(phi),
				     y,
				     ring_radius * F32Sin(phi));

			v[vi++] = V3(ring_radius * F32Cos(phi),
				     -y,
				     ring_radius * F32Sin(phi));
		}
	}

	for (u32 i = 0; i < n_lat_cyl_slice; ++i)
	{
		const f32 y = -half_height + i*half_height / n_lat_cyl_slice;
		for (u32 j = 0; j < n_long_slice; ++j)
		{
			const f32 phi = j * 2.0f * F32_PI / n_long_slice;
			v[vi++] = V3(radius * F32Cos(phi),
				     y,
				     radius * F32Sin(phi));
		}
	}

	ds_Assert(vi == n);
	ArenaPopPacked(mem, (arr.len - vi) * sizeof(v3));

	struct arena *tmp = ArenaPushScratch();
	struct dcel dcel = DcelConvexHull(tmp, v, vi, 100.0f * F32_EPSILON);
	r_MeshHull(mem, mesh, &dcel);
    ArenaPopScratch();
}

void r_MeshHull(struct arena *mem, struct r_Mesh *mesh, const struct dcel *hull)
{
	mesh->vertex_data = mem->stack_ptr;
	mesh->vertex_count = 0;
	mesh->local_stride = sizeof(v3) + sizeof(v3);

	for (u32 fi = 0; fi < hull->f_count; ++fi)
	{
		struct dcelFace *f = hull->f + fi;
		struct dcelEdge *e0 = hull->e + f->first;
		struct dcelEdge *e1 = hull->e + f->first + 1;
		struct dcelEdge *e2 = hull->e + f->first + 2;
		const v3 normal = TriCcwNormal(hull->v[e0->origin],
				hull->v[e1->origin],
				hull->v[e2->origin]);

		const v3 p0 = hull->v[e0->origin]; 
		const v3 p1 = hull->v[e1->origin];
		v3 p2 = hull->v[e2->origin];

		ArenaPushPackedMemcpy(mem, &p0, sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		ArenaPushPackedMemcpy(mem, &p1, sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		ArenaPushPackedMemcpy(mem, &p2, sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		mesh->vertex_count += 3;

		const u32 tri_count = f->count - 2;
		for (u32 ti = 1; ti < tri_count; ++ti)
		{
			mesh->vertex_count += 1;
			e2 = hull->e + f->first + ti + 2;
                	p2 = hull->v[e2->origin];

			ArenaPushPackedMemcpy(mem, &p2, sizeof(v3));
			//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
			ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		}
	}

	mesh->index_data = (u32 *) mem->stack_ptr;
	mesh->index_count = 0;
	u32 m_i = 0;
	for (u32 fi = 0; fi < hull->f_count; ++fi)
	{
		u32 indices[3] = { m_i + 0, m_i + 1, m_i + 2 };
		u32 offset = 3;
		ArenaPushPackedMemcpy(mem, indices, sizeof(indices));
		mesh->index_count += 3;

		struct dcelFace *f = hull->f + fi;
		const u32 tri_count = f->count - 2;
		for (u32 ti = 1; ti < tri_count; ++ti)
		{
			indices[0] = m_i;
			indices[1] = m_i + ti + 1;
			indices[2] = m_i + ti + 2;

			ArenaPushPackedMemcpy(mem, indices, sizeof(indices));
			offset += 1;
			mesh->index_count += 3;
		}

		ds_Assert((u64) m_i + offset <= U32_MAX)
		m_i += offset;
	}

	mesh->index_max_used = m_i - 1;
}

void r_MeshTriMesh(struct arena *mem, struct r_Mesh *mesh, const struct triMesh *tri_mesh)
{
	mesh->vertex_count = 3*tri_mesh->tri_count;
	mesh->vertex_data = mem->stack_ptr;
	mesh->index_count = 0;
	mesh->index_data = NULL;
	mesh->local_stride = sizeof(v3) /* + sizeof(v4) */ + sizeof(v3);

	const v4 color = V4(0.5f, 0.5f, 0.8f, 0.7f);

	for (u32 t = 0; t < tri_mesh->tri_count; ++t)
	{
		const v3 normal = TriCcwNormal(tri_mesh->v[tri_mesh->tri[t].buf[0]],
				tri_mesh->v[tri_mesh->tri[t].buf[1]],
				tri_mesh->v[tri_mesh->tri[t].buf[2]]);

		ArenaPushPackedMemcpy(mem, &tri_mesh->v[tri_mesh->tri[t].buf[0]], sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		ArenaPushPackedMemcpy(mem, &tri_mesh->v[tri_mesh->tri[t].buf[1]], sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
		ArenaPushPackedMemcpy(mem, &tri_mesh->v[tri_mesh->tri[t].buf[2]], sizeof(v3));
		//ArenaPushPackedMemcpy(mem, &color, sizeof(v4));
		ArenaPushPackedMemcpy(mem, &normal, sizeof(v3));
	}

	mesh->index_max_used = 0;
	//mesh->index_max_used = mesh->index_count-1;
}

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
	mesh_stub->vertex_count = sizeof(stub_vertices) / (6*sizeof(f32));	/* position and normal per vertex */
	mesh_stub->vertex_data = stub_vertices;
	mesh_stub->local_stride = 6*sizeof(f32);
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
	n = V3(0.0f, -1.0f, 0.0f);
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

	/*
	 * r_MeshSphere's rings split at the equator: the upper hemisphere moved up by half_height, the lower one down, and
	 * the cylinder between the two equator rings. Rings run from the top pole down (refinement per hemisphere, the
	 * last upper and the first lower one on the equator); the normals are the sphere's, radial on the cylinder.
	 */
	const u32 points_per_ring = 2 * refinement;
	const u32 ring_count = 2 * refinement;
	const f32 polar_step = F32_PI / (2.0f * refinement);
	const f32 azimuth_step = F32_PI / refinement;

	const u32 vertex_count = 2 + ring_count * points_per_ring;
	const u64 vertex_size = sizeof(v3) + sizeof(v3);
	u8 *vertex_data = ArenaPush(mem, vertex_count * vertex_size);

	const u32 index_count = 2 * 3 * points_per_ring + (ring_count - 1) * points_per_ring * 6;
	u32 *index_data = ArenaPush(mem, index_count * sizeof(u32));

	u64 offset = 0;
	v3 vertex = V3(0.0f, half_height + radius, 0.0f);
	v3 n = V3(0.0f, 1.0f, 0.0f);
	memcpy(vertex_data + offset, &vertex, sizeof(v3));
	memcpy(vertex_data + offset + sizeof(v3), &n, sizeof(v3));
	offset += vertex_size;

	for (u32 i = 0; i < ring_count; ++i)
	{
		const u32 upper = (i < refinement);
		const f32 polar = polar_step * ((upper) ? i + 1 : i);
		const f32 y = (upper) ? half_height : -half_height;
		for (u32 j = 0; j < points_per_ring; ++j)
		{
			const f32 t = azimuth_step * j;
			n = V3(F32Sin(polar) * F32Cos(t), F32Cos(polar), -F32Sin(polar) * F32Sin(t));
			vertex = V3Add(V3Scale(n, radius), V3(0.0f, y, 0.0f));
			memcpy(vertex_data + offset, &vertex, sizeof(v3));
			memcpy(vertex_data + offset + sizeof(v3), &n, sizeof(v3));
			offset += vertex_size;
		}
	}

	vertex = V3(0.0f, -half_height - radius, 0.0f);
	n = V3(0.0f, -1.0f, 0.0f);
	memcpy(vertex_data + offset, &vertex, sizeof(v3));
	memcpy(vertex_data + offset + sizeof(v3), &n, sizeof(v3));

	/* r_InternalMeshSphere's triangles: the top fan, a band of quads between consecutive rings, the bottom fan */
	u32 k = 0;
	for (u32 i = 0; i < points_per_ring; ++i)
	{
		index_data[k++] = 1 + ((i + 1) % points_per_ring);
		index_data[k++] = 0;
		index_data[k++] = 1 + i;
	}

	u32 ring = 1;
	for (u32 i = 1; i < ring_count; ++i)
	{
		ring += points_per_ring;
		for (u32 j = 0; j < points_per_ring; ++j)
		{
			index_data[k++] = ring + ((j + 1) % points_per_ring);
			index_data[k++] = ring + j - points_per_ring;
			index_data[k++] = ring + j;
			index_data[k++] = ring + ((j + 1) % points_per_ring);
			index_data[k++] = ring + ((j + 1) % points_per_ring) - points_per_ring;
			index_data[k++] = ring + j - points_per_ring;
		}
	}

	for (u32 i = 0; i < points_per_ring; ++i)
	{
		index_data[k++] = ring + points_per_ring;
		index_data[k++] = ring + ((i + 1) % points_per_ring);
		index_data[k++] = ring + i;
	}
	ds_Assert(k == index_count);

	mesh->index_max_used = vertex_count - 1;
	mesh->index_count = index_count;
	mesh->index_data = index_data;
	mesh->vertex_count = vertex_count;
	mesh->vertex_data = (void *) vertex_data;
	mesh->local_stride = vertex_size;
}

void r_MeshHull(struct arena *mem, struct r_Mesh *mesh, const struct dcel *hull)
{
	mesh->vertex_data = mem->stack_ptr;
	mesh->vertex_count = 0;
	mesh->local_stride = sizeof(v3) + sizeof(v3);

	for (u32 fi = 0; fi < hull->f_count; ++fi)
	{
		const struct dcelFace *f = hull->f + fi;
		const struct dcelEdge *e0 = hull->e + f->first;
		const struct dcelEdge *e1 = hull->e + f->first + 1;
		const struct dcelEdge *e2 = hull->e + f->first + 2;
		const v3 normal = hull->plane[fi].normal;

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

		const struct dcelFace *f = hull->f + fi;
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
		/* a degenerate triangle gets the zero normal (drawn unlit) */
		v3 normal;
		TriCcwNormalTryCompute(&normal, tri_mesh->v[tri_mesh->tri[t].buf[0]], tri_mesh->v[tri_mesh->tri[t].buf[1]], tri_mesh->v[tri_mesh->tri[t].buf[2]]);

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

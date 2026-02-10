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

#include "dynamics.h"

void ds_RigidBodyUpdateLocalFrame(struct ds_RigidBodyPipeline *pipeline, const u32 body, const ds_Transform t_apply_to_local)
{
	//TODO
	ds_Assert(0);
}

void ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const u32 body_index)
{
	ArenaPushRecord(&pipeline->frame);

	struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, body_index);
	ds_Assert(PoolSlotAllocated(body));

	vec3 tmp;
	mat3 rot_world, rot_local, rot_local_inv, tmp1, tmp2, tmp3;
	Mat3Quat(rot_world, body->t_world.rotation);

	body->mass = 0.0f;
	Vec3Set(body->local_center_of_mass, 0.0f, 0.0f, 0.0f);
	Mat3Set(body->inertia_tensor, 
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f);

	f32 *mass = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(f32));
	vec3ptr center_of_mass = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(vec3));
	mat3ptr inertia_tensor = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(mat3));

	struct ds_Shape *shape = NULL;
	u32 s = body->shape_list.first;
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		shape = PoolAddress(&pipeline->shape_pool, s);
		s = shape->dll_next;
		const struct collisionShape *cshape = strdb_Address(pipeline->shape_db, shape->cshape_handle);

		mass[i] = shape->density * cshape->volume;
		body->mass += mass[i];

		/* R, R^-1*/
		Mat3Quat(rot_local, shape->t_local.rotation);
		Mat3Transpose(rot_local_inv, rot_local);

		/* center_of_mass_Shape[i] = R*shape_center_of_mass + pos */
		Vec3Copy(tmp, cshape->center_of_mass);
		Mat3VecMul(center_of_mass[i], rot_local, tmp);
		Vec3Translate(center_of_mass[i], shape->t_local.position);
		Vec3TranslateScaled(body->local_center_of_mass, center_of_mass[i], mass[i]);

		/* I_Shape(i) = R * Shape_Inertia * R^-1 */
		Mat3Scale(tmp1, cshape->inertia_tensor, shape->density);
		Mat3Mul(tmp2, rot_local, tmp1);
		Mat3Mul(inertia_tensor[i], tmp2, rot_local_inv);

	}

	Vec3ScaleSelf(body->local_center_of_mass, 1.0f / body->mass);

	/* 
	 * d(i) = center_of_mass_Shape(i) - center_of_mass_Body
	 * I_Body = sum { I_Shape(i) + mass_Shape(i) * (Identity*DOT(d(i),d(i) - OUTER(d(i),d(i)))) } 
	 */
	vec3 d;
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		Vec3Sub(d, center_of_mass[i], body->local_center_of_mass);

		Mat3Identity(tmp1);
		Mat3ScaleSelf(tmp1, mass[i]*Vec3Dot(d, d));

		Mat3OuterProduct(tmp2, d, d);
		Mat3ScaleSelf(tmp2, mass[i]);

		Mat3AddSelf(body->inertia_tensor, inertia_tensor[i]);
		Mat3AddSelf(body->inertia_tensor, tmp1);
		Mat3SubSelf(body->inertia_tensor, tmp2);
	}

	ArenaPopRecord(&pipeline->frame);
}

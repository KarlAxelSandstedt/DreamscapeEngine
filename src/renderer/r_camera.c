/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#include "r_local.h"
#include "transform.h"

void r_camera2d_transform(mat3 W_to_AS, const vec2 view_center, const f32 view_height, const f32 view_aspect_ratio)
{
	const f32 view_width = view_height * view_aspect_ratio;
	/* world-to-camera, gles2 expects column-major */
	mat3 W_to_C =
	{
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ -view_center[0], -view_center[1], 1.0f },
	};

	/* camera-to-aspect_screen, gles2 expects column-major */
	mat3 C_to_AS =
	{
		{ 2.0f/view_width, 0.0f, 0.0f },
		{ 0.0f, 2.0f/view_height, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
	};

	Mat3Mul(W_to_AS, C_to_AS, W_to_C);
}

void r_camera_debug_print(const struct r_camera *cam)
{
	fprintf(stderr, "POS: (%f, %f, %f)\n", cam->position[0], cam->position[1], cam->position[2]);
	fprintf(stderr, "RIGHT: (%f, %f, %f)\n", cam->left[0], cam->left[1], cam->left[2]);
	fprintf(stderr, "UP: (%f, %f, %f)\n", cam->up[0], cam->up[1], cam->up[2]);
	fprintf(stderr, "DIR: (%f, %f, %f)\n", cam->forward[0], cam->forward[1], cam->forward[2]);
	fprintf(stderr, "ASPECT, FOV_X, FZ_NEAR, FZ_FAR: (%f, %f, %f, %f)\n", cam->aspect_ratio, cam->fov_x, cam->fz_near, cam->fz_far);
	fprintf(stderr, "YAW, PITCH: (%f, %f)\n", cam->yaw, cam->pitch);
}

struct r_camera r_camera_init(const vec3 position, const vec3 direction, const f32 fz_near, const f32 fz_far, const f32 aspect_ratio, const f32 fov_x)
{
	ds_Assert(fov_x > 0.0f && fov_x < F32_PI);
	ds_Assert(fz_near > 0.0f);
	ds_Assert(fz_far > fz_near);
	ds_Assert(aspect_ratio > 0);
	ds_Assert(Vec3Length(direction) > 0.0f);
	
	struct r_camera cam = 
	{
		.yaw = 0.0f,
		.pitch = 0.0f,
		.fz_near = fz_near,
		.fz_far = fz_far,
		.aspect_ratio = aspect_ratio,
		.fov_x = fov_x,
	};

	Vec3Copy(cam.position, position);

	const vec3 direction_xz = { direction[0], 0.0f, direction[2] };
	const f32 direction_xz_len = Vec3Length(direction_xz);
	if (direction_xz_len < 0.001f)
	{
		if (direction[1] > 0.0f)
		{
			Vec3Set(cam.up, 0.0f, 0.0f, -1.0f);
			Vec3Set(cam.forward, 0.0f, 1.0f, 0.0f);
			Vec3Set(cam.left, 1.0f, 0.0f, 0.0f);
		}
		else
		{
			Vec3Set(cam.up, 0.0f, 0.0f, 1.0f);
			Vec3Set(cam.forward, 0.0f, -1.0f, 0.0f);
			Vec3Set(cam.left, 1.0f, 0.0f, 0.0f);
		}
	}
	else
	{
		const vec3 x = { 1.0f, 0.0f, 0.0f };
		const vec3 y = { 0.0f, 1.0f, 0.0f };
		const vec3 z = { 0.0f, 0.0f, 1.0f };

		quat q1, q2;
		const f32 angle1 = (direction[0] < 0.0f) 
			? -f32_acos(direction[2] / (direction_xz_len))
			:  f32_acos(direction[2] / (direction_xz_len));

		mat3 rot, rot1, rot2;
		QuatUnitAxisAngle(q1, y, angle1);
		Mat3Quat(rot1, q1);

		const f32 angle2 = (direction[1] > 0.0f)
			? -f32_acos(direction_xz_len*direction_xz_len / (direction_xz_len * Vec3Length(direction)))
			:  f32_acos(direction_xz_len*direction_xz_len / (direction_xz_len * Vec3Length(direction)));
		vec3 v;
		Mat3VecMul(v, rot1, x);
		QuatAxisAngle(q2, v, angle2);
		Mat3Quat(rot2, q2);

		Mat3Mul(rot, rot2, rot1);

		Mat3VecMul(cam.forward, rot, z);
		Mat3VecMul(cam.left, rot, x);
		Mat3VecMul(cam.up, rot, y);
	}

	return cam;
}

void r_camera_construct(struct r_camera *cam,
		const vec3 position,
	       	const vec3 left,
	       	const vec3 up,
	       	const vec3 forward,
		const f32 yaw,
		const f32 pitch,
	       	const f32 fz_near,
	       	const f32 fz_far,
	       	const f32 aspect_ratio,
	       	const f32 fov_x)
{
	ds_Assert(fov_x > 0.0f && fov_x < F32_PI);
	ds_Assert(fz_near > 0.0f);
	ds_Assert(fz_far > fz_near);
	ds_Assert(aspect_ratio > 0);
	
	Vec3Copy(cam->position, position);
	Vec3Copy(cam->left, left);
	Vec3Copy(cam->up, up);
	Vec3Copy(cam->forward, forward);
	cam->yaw = yaw;
	cam->pitch = pitch;
	cam->fz_near = fz_near;
	cam->fz_far = fz_far;
	cam->aspect_ratio = aspect_ratio;
	cam->fov_x = fov_x;
}

void r_camera_update_axes(struct r_camera *cam)
{
	vec3 left = {1.0f, 0.0f, 0.0f};
	vec3 up = {0.0f, 1.0f, 0.0f};
	vec3 forward = {0.0f, 0.0f, 1.0f};

	mat3 rot;
	mat3SequentialRotation(rot, up, cam->yaw, left, cam->pitch);

	Mat3VecMul(cam->left, rot, left);
	Mat3VecMul(cam->up, rot, up);
	Mat3VecMul(cam->forward, rot, forward);
}

void r_camera_update_angles(struct r_camera *cam, const f32 yaw_delta, const f32 pitch_delta)
{
	cam->yaw += yaw_delta;
	if (cam->yaw >= F32_PI)
	{
		cam->yaw -= F32_PI2;
	}
	else if (cam->yaw <= -F32_PI)
	{
		cam->yaw += F32_PI2;
	}

	if (cam->pitch + pitch_delta > F32_PI / 2.0f - 0.50f)
	{
		cam->pitch = F32_PI / 2.0f - 0.50f;
	}
	else if (cam->pitch + pitch_delta < 0.50f - F32_PI / 2.0f)
	{
		cam->pitch = 0.50f - F32_PI / 2.0f;
	}
	else 
	{
		cam->pitch += pitch_delta;
	}
}

void frustum_projection_plane_sides(f32 *width, f32 *height, const f32 plane_distance, const f32 fov_x, const f32 aspect_ratio)
{
	*width = 2.0f * plane_distance * f32_tan(fov_x / 2.0f);
	*height = *width / aspect_ratio;
}

void frustum_projection_plane_camera_space(vec3 bottom_left, vec3 upper_right, const struct r_camera *cam)
{
	f32 frustum_width, frustum_height;
	frustum_projection_plane_sides(&frustum_width, &frustum_height, cam->fz_near, cam->fov_x, cam->aspect_ratio);
	Vec3Set(bottom_left, frustum_width / 2.0f, -frustum_height / 2.0f, cam->fz_near);
	Vec3Set(upper_right, -frustum_width / 2.0f, frustum_height / 2.0f, cam->fz_near);
}

void frustum_projection_plane_world_space(vec3 bottom_left, vec3 upper_right, const struct r_camera *cam)
{
	f32 frustum_width, frustum_height;
	frustum_projection_plane_sides(&frustum_width, &frustum_height, cam->fz_near, cam->fov_x, cam->aspect_ratio);

	vec3 v;
	vec3 left = {1.0f, 0.0f, 0.0f};
	vec3 up = {0.0f, 1.0f, 0.0f};
	mat3 rot;
	mat3SequentialRotation(rot, up, cam->yaw, left, cam->pitch);

	Vec3Set(v, frustum_width / 2.0f, -frustum_height / 2.0f, cam->fz_near);
	Mat3VecMul(bottom_left, rot, v);
	Vec3Translate(bottom_left, cam->position);

	v[0] -= frustum_width;
	v[1] += frustum_height;
	Mat3VecMul(upper_right, rot, v);
	Vec3Translate(upper_right, cam->position);
}

void window_space_to_world_space(vec3 world_pixel, const vec2 pixel, const vec2 win_size, const struct r_camera * cam)
{
	mat3 rot;
	vec3 bl, tr, camera_pixel;

	vec3 left = {1.0f, 0.0f, 0.0f};
	vec3 up = {0.0f, 1.0f, 0.0f};
	mat3SequentialRotation(rot, up, cam->yaw, left, cam->pitch);

	const vec3 alphas = { 1.0f - ((f32) pixel[0]) / win_size[0], 1.0f - ((f32) pixel[1]) / win_size[1], 1.0f };	
	frustum_projection_plane_camera_space(bl, tr, cam);
	Vec3InterpolatePiecewise(camera_pixel, bl, tr, alphas);	
	Mat3VecMul(world_pixel, rot, camera_pixel);
	Vec3Translate(world_pixel, cam->position);
}

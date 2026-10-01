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

#include "r_local.h"

m3 r_Camera2dTransform(const v2 view_center, const f32 view_height, const f32 view_aspect_ratio)
{
	const f32 view_width = view_height * view_aspect_ratio;
	/* world-to-camera, gles2 expects column-major */
	const m3 W_to_C = M3(1.0f, 0.0f, 0.0f,
			     0.0f, 1.0f, 0.0f,
			     -view_center.x, -view_center.y, 1.0f);

	/* camera-to-aspect_screen, gles2 expects column-major */
	const m3 C_to_AS = M3(2.0f/view_width, 0.0f, 0.0f,
			      0.0f, 2.0f/view_height, 0.0f,
			      0.0f, 0.0f, 1.0f);

	return M3Mul(C_to_AS, W_to_C);
}

void r_CameraDebugPrint(const struct r_Camera *cam)
{
	fprintf(stderr, "POS: (%f, %f, %f)\n", cam->position.x, cam->position.y, cam->position.z);
	fprintf(stderr, "RIGHT: (%f, %f, %f)\n", cam->left.x, cam->left.y, cam->left.z);
	fprintf(stderr, "UP: (%f, %f, %f)\n", cam->up.x, cam->up.y, cam->up.z);
	fprintf(stderr, "DIR: (%f, %f, %f)\n", cam->forward.x, cam->forward.y, cam->forward.z);
	fprintf(stderr, "ASPECT, FOV_X, FZ_NEAR, FZ_FAR: (%f, %f, %f, %f)\n", cam->aspect_ratio, cam->fov_x, cam->fz_near, cam->fz_far);
	fprintf(stderr, "YAW, PITCH: (%f, %f)\n", cam->yaw, cam->pitch);
}

struct r_Camera r_CameraInit(const v3 position, const v3 direction, const f32 fz_near, const f32 fz_far, const f32 aspect_ratio, const f32 fov_x)
{
	ds_Assert(fov_x > 0.0f && fov_x < F32_PI);
	ds_Assert(fz_near > 0.0f);
	ds_Assert(fz_far > fz_near);
	ds_Assert(aspect_ratio > 0);
	ds_Assert(V3Length(direction) > 0.0f);
	
	struct r_Camera cam = 
	{
		.position = position,
		.yaw = 0.0f,
		.pitch = 0.0f,
		.fz_near = fz_near,
		.fz_far = fz_far,
		.aspect_ratio = aspect_ratio,
		.fov_x = fov_x,
	};

	const v3 direction_xz = V3(direction.x, 0.0f, direction.z);
	const f32 direction_xz_len = V3Length(direction_xz);
	if (direction_xz_len < 0.001f)
	{
		if (direction.y > 0.0f)
		{
			cam.up = V3(0.0f, 0.0f, -1.0f);
			cam.forward = V3(0.0f, 1.0f, 0.0f);
			cam.left = V3(1.0f, 0.0f, 0.0f);
		}
		else
		{
			cam.up = V3(0.0f, 0.0f, 1.0f);
			cam.forward = V3(0.0f, -1.0f, 0.0f);
			cam.left = V3(1.0f, 0.0f, 0.0f);
		}
	}
	else
	{
		const v3 x = V3(1.0f, 0.0f, 0.0f);
		const v3 y = V3(0.0f, 1.0f, 0.0f);
		const v3 z = V3(0.0f, 0.0f, 1.0f);

		const f32 angle1 = (direction.x < 0.0f) 
			? -F32Acos(direction.z / (direction_xz_len))
			:  F32Acos(direction.z / (direction_xz_len));
		const m3 rot1 = M3Q(QUnitAxisAngle(y, angle1));

		const f32 angle2 = (direction.y > 0.0f)
			? -F32Acos(direction_xz_len*direction_xz_len / (direction_xz_len * V3Length(direction)))
			:  F32Acos(direction_xz_len*direction_xz_len / (direction_xz_len * V3Length(direction)));
		const v3 v = M3V3Mul(rot1, x);
		const m3 rot2 = M3Q(QAxisAngle(v, angle2));

		const m3 rot = M3Mul(rot2, rot1);

		cam.forward = M3V3Mul(rot, z);
		cam.left = M3V3Mul(rot, x);
		cam.up = M3V3Mul(rot, y);
	}

	return cam;
}

void r_CameraConstruct(struct r_Camera *cam,
		const v3 position,
	       	const v3 left,
	       	const v3 up,
	       	const v3 forward,
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
	
	cam->position = position;
	cam->left = left;
	cam->up = up;
	cam->forward = forward;
	cam->yaw = yaw;
	cam->pitch = pitch;
	cam->fz_near = fz_near;
	cam->fz_far = fz_far;
	cam->aspect_ratio = aspect_ratio;
	cam->fov_x = fov_x;
}

void r_CameraUpdateAxes(struct r_Camera *cam)
{
	const v3 left = V3(1.0f, 0.0f, 0.0f);
	const v3 up = V3(0.0f, 1.0f, 0.0f);
	const v3 forward = V3(0.0f, 0.0f, 1.0f);

	const m3 rot = M3SequentialRotation(up, cam->yaw, left, cam->pitch);

	cam->left = M3V3Mul(rot, left);
	cam->up = M3V3Mul(rot, up);
	cam->forward = M3V3Mul(rot, forward);
}

void r_CameraUpdateAngles(struct r_Camera *cam, const f32 yaw_delta, const f32 pitch_delta)
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

void FrustumProjectionPlaneSides(f32 *width, f32 *height, const f32 plane_distance, const f32 fov_x, const f32 aspect_ratio)
{
	*width = 2.0f * plane_distance * F32Tan(fov_x / 2.0f);
	*height = *width / aspect_ratio;
}

void FrustumProjectionPlaneCameraSpace(v3 *bottom_left, v3 *upper_right, const struct r_Camera *cam)
{
	f32 frustum_width, frustum_height;
	FrustumProjectionPlaneSides(&frustum_width, &frustum_height, cam->fz_near, cam->fov_x, cam->aspect_ratio);
	*bottom_left = V3(frustum_width / 2.0f, -frustum_height / 2.0f, cam->fz_near);
	*upper_right = V3(-frustum_width / 2.0f, frustum_height / 2.0f, cam->fz_near);
}

void FrustumProjectionPlaneWorldSpace(v3 *bottom_left, v3 *upper_right, const struct r_Camera *cam)
{
	f32 frustum_width, frustum_height;
	FrustumProjectionPlaneSides(&frustum_width, &frustum_height, cam->fz_near, cam->fov_x, cam->aspect_ratio);

	const v3 left = V3(1.0f, 0.0f, 0.0f);
	const v3 up = V3(0.0f, 1.0f, 0.0f);
	const m3 rot = M3SequentialRotation(up, cam->yaw, left, cam->pitch);

	v3 v = V3(frustum_width / 2.0f, -frustum_height / 2.0f, cam->fz_near);
	*bottom_left = V3Add(M3V3Mul(rot, v), cam->position);

	v.x -= frustum_width;
	v.y += frustum_height;
	*upper_right = V3Add(M3V3Mul(rot, v), cam->position);
}

v3 WindowSpaceToWorldSpace(const v2 pixel, const v2 win_size, const struct r_Camera * cam)
{
	const v3 left = V3(1.0f, 0.0f, 0.0f);
	const v3 up = V3(0.0f, 1.0f, 0.0f);
	const m3 rot = M3SequentialRotation(up, cam->yaw, left, cam->pitch);

	const v3 alphas = V3(1.0f - pixel.x / win_size.x, 1.0f - pixel.y / win_size.y, 1.0f);	
	v3 bl, tr;
	FrustumProjectionPlaneCameraSpace(&bl, &tr, cam);
	const v3 camera_pixel = V3InterpolatePiecewise(bl, tr, alphas);	
	return V3Add(M3V3Mul(rot, camera_pixel), cam->position);
}

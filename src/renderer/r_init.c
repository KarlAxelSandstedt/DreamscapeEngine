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
#include "r_public.h"

#include <string.h>

//TODO MOVE into asset_manager
#if __DS_PLATFORM__ == __DS_WIN64__ || __DS_PLATFORM__ ==  __DS_LINUX__
#define vertex_ui		"../assets/shaders/ui.vert"
#define fragment_ui		"../assets/shaders/ui.frag"
#define vertex_proxy3d		"../assets/shaders/proxy3d.vert"
#define fragment_proxy3d	"../assets/shaders/proxy3d.frag"
#define vertex_color		"../assets/shaders/color.vert"
#define fragment_color		"../assets/shaders/color.frag"
#define vertex_lightning	"../assets/shaders/lightning.vert"
#define fragment_lightning	"../assets/shaders/lightning.frag"
#elif __DS_PLATFORM__ == __DS_WEB__
#define vertex_ui		"../assets/shaders/gles_ui.vert"
#define fragment_ui		"../assets/shaders/gles_ui.frag"
#define vertex_proxy3d		"../assets/shaders/gles_proxy3d.vert"
#define fragment_proxy3d	"../assets/shaders/gles_proxy3d.frag"
#define vertex_color		"../assets/shaders/gles_color.vert"
#define fragment_color		"../assets/shaders/gles_color.frag"
#define vertex_lightning	"../assets/shaders/gles_lightning.vert"
#define fragment_lightning	"../assets/shaders/gles_lightning.frag"
#endif

static void shader_source_and_compile(GLuint shader, const char *filepath)
{
	FILE *file = fopen(filepath, "r");

	char buf[4096];
	memset(buf, 0, sizeof(buf));
	fseek(file, 0, SEEK_END);
	long int size = ftell(file);
	fseek(file, 0, SEEK_SET);
	fread(buf, 1, size, file);

	const GLchar *buf_ptr = buf;
	ds_glShaderSource(shader, 1, &buf_ptr, 0);

	fclose(file);

	ds_glCompileShader(shader);	

	GLint compiled;
	ds_glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled == GL_FALSE)
	{
		GLsizei len = 0;
		ds_glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
		ds_glGetShaderInfoLog(shader, len, &len, buf);
		Log(T_RENDERER, S_FATAL, "Failed to compile %s, %s", filepath, buf);
		FatalCleanupAndExit(ds_ThreadSelfTid());
	}
}

void r_compile_shader(u32 *prg, const char *v_filepath, const char *f_filepath)
{
	GLuint v_sh = ds_glCreateShader(GL_VERTEX_SHADER);
	GLuint f_sh = ds_glCreateShader(GL_FRAGMENT_SHADER);

	shader_source_and_compile(v_sh, v_filepath);
	shader_source_and_compile(f_sh, f_filepath);

	*prg = ds_glCreateProgram();

	ds_glAttachShader(*prg, v_sh);
	ds_glAttachShader(*prg, f_sh);

	ds_glLinkProgram(*prg);
	GLint success;
	ds_glGetProgramiv(*prg, GL_LINK_STATUS, &success);
	if (!success)
	{
		char buf[4096];
		GLsizei len = 0;
		ds_glGetProgramiv(*prg, GL_INFO_LOG_LENGTH, &len);
		ds_glGetProgramInfoLog(*prg, len, &len, buf);
		Log(T_RENDERER, S_FATAL, "Failed to Link program: %s", buf);
		FatalCleanupAndExit(ds_ThreadSelfTid());
	}

	ds_glDetachShader(*prg, v_sh);
	ds_glDetachShader(*prg, f_sh);

	ds_glDeleteShader(v_sh);
	ds_glDeleteShader(f_sh);
}

void r_color_buffer_layout_setter(void)
{
	ds_glEnableVertexAttribArray(0);
	ds_glEnableVertexAttribArray(1);

	const u64 stride = sizeof(vec3) + sizeof(vec4);

	ds_glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, (GLsizei)  stride, 0);
	ds_glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, (GLsizei)  stride, (void *)(sizeof(vec3)));
}

void r_lightning_buffer_layout_setter(void)
{
	ds_glEnableVertexAttribArray(0);
	ds_glEnableVertexAttribArray(1);
	ds_glEnableVertexAttribArray(2);

	const u64 stride = 2*sizeof(vec3) + sizeof(vec4);

	ds_glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, (GLsizei)  stride, 0);
	ds_glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, (GLsizei)  stride, (void *)(sizeof(vec3)));
	ds_glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, (GLsizei)  stride, (void *)(sizeof(vec3) + sizeof(vec4)));
}

void r_init(struct arena *mem_persistent, const u64 ns_tick, const u64 frame_size, const u64 core_unit_count, struct strdb *mesh_database)
{
	g_r_core->frames_elapsed = 0;	
	g_r_core->ns_elapsed = 0;	
	g_r_core->ns_tick = ns_tick;	

	r_compile_shader(&g_r_core->program[PROGRAM_UI].gl_program, vertex_ui, fragment_ui);
	g_r_core->program[PROGRAM_UI].shared_stride = S_UI_STRIDE;
	g_r_core->program[PROGRAM_UI].local_stride = L_UI_STRIDE;
	g_r_core->program[PROGRAM_UI].buffer_shared_layout_setter = r_ui_buffer_shared_layout_setter;
	g_r_core->program[PROGRAM_UI].buffer_local_layout_setter = r_ui_buffer_local_layout_setter;

	r_compile_shader(&g_r_core->program[PROGRAM_PROXY3D].gl_program, vertex_proxy3d, fragment_proxy3d);
	g_r_core->program[PROGRAM_PROXY3D].shared_stride = S_PROXY3D_STRIDE;
	g_r_core->program[PROGRAM_PROXY3D].local_stride = L_PROXY3D_STRIDE;
	g_r_core->program[PROGRAM_PROXY3D].buffer_shared_layout_setter = r_proxy3d_buffer_shared_layout_setter;
	g_r_core->program[PROGRAM_PROXY3D].buffer_local_layout_setter = r_proxy3d_buffer_local_layout_setter;

	r_compile_shader(&g_r_core->program[PROGRAM_COLOR].gl_program, vertex_color, fragment_color);
	g_r_core->program[PROGRAM_COLOR].shared_stride = S_COLOR_STRIDE;
	g_r_core->program[PROGRAM_COLOR].local_stride = L_COLOR_STRIDE;
	g_r_core->program[PROGRAM_COLOR].buffer_shared_layout_setter = NULL;
	g_r_core->program[PROGRAM_COLOR].buffer_local_layout_setter = r_color_buffer_layout_setter;

	r_compile_shader(&g_r_core->program[PROGRAM_LIGHTNING].gl_program, vertex_lightning, fragment_lightning);
	g_r_core->program[PROGRAM_LIGHTNING].shared_stride = S_LIGHTNING_STRIDE;
	g_r_core->program[PROGRAM_LIGHTNING].local_stride = L_LIGHTNING_STRIDE;
	g_r_core->program[PROGRAM_LIGHTNING].buffer_shared_layout_setter = NULL;
	g_r_core->program[PROGRAM_LIGHTNING].buffer_local_layout_setter = r_lightning_buffer_layout_setter;

	g_r_core->frame = ArenaAlloc(frame_size); 
	if (g_r_core->frame.mem_size == 0)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate renderer frame, exiting.");
		FatalCleanupAndExit(0);
	}

	g_r_core->proxy3d_hierarchy = hi_Alloc(NULL, core_unit_count, sizeof(struct r_proxy3d), GROWABLE);
	if (g_r_core->proxy3d_hierarchy == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate r_core unit hierarchy, exiting.");
		FatalCleanupAndExit(ds_ThreadSelfTid());
	}

	struct slot slot3d = hi_Add(g_r_core->proxy3d_hierarchy, HI_NULL_INDEX);
	g_r_core->proxy3d_root = slot3d.index;
	ds_Assert(g_r_core->proxy3d_root == PROXY3D_ROOT);
	struct r_proxy3d *stub3d = slot3d.address;
	Vec3Set(stub3d->position, 0.0f, 0.0f, 0.0f);
	Vec3Set(stub3d->spec_position, 0.0f, 0.0f, 0.0f);
	const vec3 axis = { 0.0f, 1.0f, 0.0f };
	unit_axis_angle_to_quaternion(stub3d->rotation, axis, 0.0f);
	quat_copy(stub3d->spec_rotation, stub3d->rotation);
	Vec3Set(stub3d->linear.linear_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(stub3d->linear.angular_velocity, 0.0f, 0.0f, 0.0f);
	stub3d->flags = 0;

	g_r_core->mesh_database = mesh_database; 
	struct r_mesh *stub = strdb_Address(g_r_core->mesh_database, STRING_DATABASE_STUB_INDEX);
	r_mesh_set_stub_box(stub);





	g_r_core->texture[TEXTURE_STUB].handle = 0;

	ArenaPushRecord(mem_persistent);
	const struct asset_font *a_f = asset_database_request_font(&g_r_core->frame, FONT_DEFAULT_SMALL);
	u32 w = a_f->font->pixmap_width;
	u32 h = a_f->font->pixmap_height;
	u8 *pixel8 = a_f->font->pixmap;
	u32 *pixel32 = ArenaPush(mem_persistent, w*h*sizeof(u32));
	for (u32 i = 0; i < w*h; ++i)
	{
		pixel32[i] = ((u32) pixel8[i] << 24) + 0xffffff;
	}
	ds_glGenTextures(1, &g_r_core->texture[TEXTURE_FONT_DEFAULT_SMALL].handle);
	ds_glActiveTexture(GL_TEXTURE0 + 0);
	ds_glBindTexture(GL_TEXTURE_2D, g_r_core->texture[TEXTURE_FONT_DEFAULT_SMALL].handle);
	ds_glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixel32);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	ds_glGenerateMipmap(GL_TEXTURE_2D);

	a_f = asset_database_request_font(&g_r_core->frame, FONT_DEFAULT_MEDIUM);
	w = a_f->font->pixmap_width;
	h = a_f->font->pixmap_height;
	pixel8 = a_f->font->pixmap;
	pixel32 = ArenaPush(mem_persistent, w*h*sizeof(u32));
	for (u32 i = 0; i < w*h; ++i)
	{
		pixel32[i] = ((u32) pixel8[i] << 24) + 0xffffff;
	}
	ds_glGenTextures(1, &g_r_core->texture[TEXTURE_FONT_DEFAULT_MEDIUM].handle);
	ds_glActiveTexture(GL_TEXTURE0 + 1);
	ds_glBindTexture(GL_TEXTURE_2D, g_r_core->texture[TEXTURE_FONT_DEFAULT_MEDIUM].handle);
	ds_glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixel32);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	ds_glGenerateMipmap(GL_TEXTURE_2D);

	ArenaPopRecord(mem_persistent);

	struct asset_ssff *asset = asset_database_request_ssff(&g_r_core->frame, SSFF_LED_ID);
	ds_glGenTextures(1, &g_r_core->texture[TEXTURE_LED].handle);
	ds_glActiveTexture(GL_TEXTURE0 + 2);
	ds_glBindTexture(GL_TEXTURE_2D, g_r_core->texture[TEXTURE_LED].handle);
	ds_glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, asset->width, asset->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, asset->pixel);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	ds_glGenerateMipmap(GL_TEXTURE_2D);

	asset = asset_database_request_ssff(&g_r_core->frame, SSFF_NONE_ID);
	ds_glGenTextures(1, &g_r_core->texture[TEXTURE_NONE].handle);
	ds_glActiveTexture(GL_TEXTURE0 + 3);
	ds_glBindTexture(GL_TEXTURE_2D, g_r_core->texture[TEXTURE_NONE].handle);
	ds_glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, asset->width, asset->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, asset->pixel);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	ds_glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	ds_glGenerateMipmap(GL_TEXTURE_2D);
}

void r_core_flush(void)
{
	g_r_core->frames_elapsed = 0;	
	g_r_core->ns_elapsed = 0;	

	hi_Flush(g_r_core->proxy3d_hierarchy);
	struct slot slot3d = hi_Add(g_r_core->proxy3d_hierarchy, HI_NULL_INDEX);
	g_r_core->proxy3d_root = slot3d.index;
	ds_Assert(g_r_core->proxy3d_root == PROXY3D_ROOT);
	struct r_proxy3d *stub3d = slot3d.address;
	Vec3Set(stub3d->position, 0.0f, 0.0f, 0.0f);
	Vec3Set(stub3d->spec_position, 0.0f, 0.0f, 0.0f);
	const vec3 axis = { 0.0f, 1.0f, 0.0f };
	unit_axis_angle_to_quaternion(stub3d->rotation, axis, 0.0f);
	quat_copy(stub3d->spec_rotation, stub3d->rotation);
	Vec3Set(stub3d->linear.linear_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(stub3d->linear.angular_velocity, 0.0f, 0.0f, 0.0f);
	stub3d->flags = 0;

	GPoolFlush(&g_r_core->unit_pool);
}

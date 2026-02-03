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

#ifndef __R_LOCAL_H__
#define __R_LOCAL_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_math.h"
#include "r_public.h"
#include "bit_vector.h"
#include "ui_public.h"
#include "asset_public.h"
#include "ds_gl.h"

#define L_POSITION_OFFSET	0
#define L_COLOR_OFFSET		(sizeof(vec3))
#define L_NORMAL_OFFSET		(sizeof(vec3) + sizeof(vec4))

#define S_COLOR_STRIDE		0
#define L_COLOR_STRIDE		(sizeof(vec3) + sizeof(vec4))

#define S_LIGHTNING_STRIDE	0
#define L_LIGHTNING_STRIDE	(2*sizeof(vec3) + sizeof(vec4))

/********************************************************
 *			r_ui.c				*
 ********************************************************/

#define S_NODE_RECT_OFFSET		(0*sizeof(vec4))
#define S_VISIBLE_RECT_OFFSET		(1*sizeof(vec4))
#define S_UV_RECT_OFFSET		(2*sizeof(vec4))
#define S_BACKGROUND_COLOR_OFFSET	(3*sizeof(vec4))
#define S_BORDER_COLOR_OFFSET 		(4*sizeof(vec4))
#define S_SPRITE_COLOR_OFFSET 		(5*sizeof(vec4))
#define S_EXTRA_OFFSET			(6*sizeof(vec4))
#define S_GRADIENT_COLOR_BR_OFFSET	(6*sizeof(vec4) + sizeof(vec3))
#define S_GRADIENT_COLOR_TR_OFFSET	(7*sizeof(vec4) + sizeof(vec3))
#define S_GRADIENT_COLOR_TL_OFFSET	(8*sizeof(vec4) + sizeof(vec3))
#define S_GRADIENT_COLOR_BL_OFFSET	(9*sizeof(vec4) + sizeof(vec3))
#define S_UI_STRIDE 			(10*sizeof(vec4) + sizeof(vec3))

#define L_UI_STRIDE 			(0)

void r_UiDraw(struct ui *ui);
/* ui program gl buffer shared instace data layout setter  */
void r_UiBufferSharedLayoutSetter(void);
/* ui program gl buffer local vertex layout setter  */
void r_UiBufferLocalLayoutSetter(void);

/********************************************************
 *			r_Init.c			*
 ********************************************************/

//TODO getting/loading shaders should be done through asset_system later 
/* compile shader program */
void 	r_ShaderCompile(u32 *prg, const char *v_filepath, const char *f_filepath);

/********************************************************
 *			r_camera.c			*
 ********************************************************/

/********************************************************
 *			r_core.c			*
 ********************************************************/

/*
 * r_program - gl program related info. Indexable by r_program_id and initalized at startup
 */
struct r_Program
{
	u32	gl_program;				/* opengl program id */
	u64	shared_stride;
	u64	local_stride;
	void	(* buffer_shared_layout_setter)(void);	/* opengl buffer shared (instanced) layout setter */
	void	(* buffer_local_layout_setter)(void);	/* opengl buffer local layout setter */
};

/*
 * r_program - gl texture related info. Indexable by r_texture_id and initalized at startup
 */
struct r_Texture
{
	GLuint	handle;	
};

 /*
 * r_core - core render state; 
 */
struct r_Core
{
	u64 			frames_elapsed;		/* frames elapsed or drawn */
	u64 			ns_elapsed;		/* process time (ns) */
	u64 			ns_tick;		/* ns per render frame; if set to 0, we redraw on each r_main
							   entry */
	/* TODO: tmp */
	struct r_Camera		cam;	

	struct arena 		frame;

	struct r_Program	program[PROGRAM_COUNT];
	struct r_Texture	texture[TEXTURE_COUNT];

	struct pool		unit_pool;

	struct strdb *mesh_database;		/* mesh storage (external) */

	struct hi 		proxy3d_hierarchy;	/* proxy3d storage */
	u32			proxy3d_root;
};
extern struct r_Core *g_r_core;

/* Reset / flush renderer core memory */
void	r_CoreFlush(void);

/********************************************************
 *			r_proxy3d.c			*
 ********************************************************/

#define S_PROXY3D_TRANSLATION_BLEND_OFFSET	(0)
#define S_PROXY3D_ROTATION_OFFSET		(1*sizeof(vec4))
#define S_PROXY3D_COLOR_OFFSET			(2*sizeof(vec4))
#define S_PROXY3D_STRIDE			(3*sizeof(vec4))

#define L_PROXY3D_POSITION_OFFSET		(0)
#define L_PROXY3D_NORMAL_OFFSET			(1*sizeof(vec3))
#define L_PROXY3D_STRIDE			(2*sizeof(vec3))

/* proxy3d opengl buffer local layout setter */
void 	r_Proxy3dBufferLocalLayoutSet(void);
/* proxy3d opengl buffer shared layout setter */
void 	r_Proxy3dBufferSharedLayoutSet(void);
/* generate speculative positions */
void 	r_Proxy3dHierarchySpeculate(struct arena *mem, const u64 ns_time);

/*************************** opengl context state ****************************/

struct gl_Limits
{
	/* shader limits */
	GLuint 			max_tx_units_vertex;
	GLuint 			max_tx_units_fragment;
	GLuint 			max_vertex_attributes;
	GLuint			max_varying_vectors;

	/* textures limits */
	GLsizei			max_2d_tx_size;
	GLsizei			max_cube_map_tx_size;

	/* texture units */
	GLuint 			tx_unit_count;

	/* buffers */
	GLuint			max_element_index;
};

extern struct gl_Limits *g_gl_limits;

struct gl_State
{
	POOL_SLOT_STATE;

	/* texture units */
	GLenum 			tx_unit_active;
	struct gl_TextureUnit *tx_unit;

	/* depth testing */
	u32			depth;

	/* program */
	u32 			program;

	/* culling */
	u32			cull_face;
	GLenum			cull_mode;
	GLenum			face_front;

	/* blending  */
	u32			blend;
	GLenum			eq_rgb;
	GLenum			eq_a;
	GLenum			func_s_rgb;
	GLenum			func_s_a;
	GLenum			func_d_rgb;
	GLenum			func_d_a;

	struct gl_Functions	func;
};

struct gl_TextureUnitBinding
{
	u32 			context;
	GLuint 			tx_unit;

	DLL_SLOT_STATE;
	POOL_SLOT_STATE;
};

struct gl_Texture
{
	POOL_SLOT_STATE;
	GLuint 		name;
	struct dll 	binding_list;

	GLenum 		target;
	GLint 		wrap_s;
	GLint 		wrap_t;
	GLint 		min_filter;
	GLint 		mag_filter;

	GLint 		level;
	GLint		internalformat;
	GLsizei		width;
	GLsizei		height;
	GLenum		format;
	GLenum		type;
};

struct gl_TextureUnit
{
	u32	binding;
	u32	gl_tx_2d_index;		/* index 0 is reserved for no texture (as opengl expects)  */
	u32	gl_tx_cube_map_index;	/* index 0 is reserved for no texture (as opengl expects)  */
};

GLboolean	ds_glIsEnabled(GLenum cap);
void 		ds_glGetIntegerv(GLenum pname, GLint *data);
const GLubyte * ds_glGetString(GLenum name);
void		ds_glGetTexParameterfv(GLenum target, GLenum pname, GLfloat *params);
void		ds_glGetTexParameteriv(GLenum target, GLenum pname, GLint *params);

void		ds_glClear(GLbitfield mask);
void		ds_glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);

void		ds_glViewport(GLint x, GLint y, GLsizei width, GLsizei height);
void		ds_glPolygonMode(GLenum face, GLenum mode);

void 		ds_glGenBuffers(GLsizei n, GLuint *buffers);
void 		ds_glBindBuffer(GLenum target, GLuint buffer);
void 		ds_glBufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
void 		ds_glBufferSubData(GLenum target, GLintptr offset, GLsizeiptr size,  const void *data);
void 		ds_glDeleteBuffers(GLsizei n, const GLuint *buffers);

GLuint 		ds_glCreateProgram(void);
void 		ds_glLinkProgram(GLuint program);
void 		ds_glUseProgram(GLuint program);
void 		ds_glDeleteProgram(GLuint program);
void		ds_glGetProgramiv(GLuint program, GLenum pname, GLint *params);
void		ds_glGetProgramInfoLog(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog);

void 		ds_glDrawArrays(GLenum mode, GLint first, GLsizei count);
void 		ds_glDrawElements(GLenum mode, GLsizei count, GLenum type, const void *indices);
void 		ds_glDrawArraysInstanced(GLenum mode, GLint first, GLint count, GLsizei primcount);
void 		ds_glDrawElementsInstanced(GLenum mode, GLsizei count, GLenum type, const void *indices, GLsizei primcount);

void 		ds_glGenVertexArrays(GLsizei n, GLuint *arrays);
void 		ds_glDeleteVertexArrays(GLsizei n, const GLuint *arrays);
void 		ds_glBindVertexArray(GLuint array);

void 		ds_glEnableVertexAttribArray(GLuint index);
void 		ds_glDisableVertexAttribArray(GLuint index);
void 		ds_glVertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer);
void 		ds_glVertexAttribIPointer(GLuint index, GLint size, GLenum type, GLsizei stride, const void *pointer);
void 		ds_glVertexAttribLPointer(GLuint index, GLint size, GLenum type, GLsizei stride, const void *pointer);
void		ds_glVertexAttribDivisor(GLuint index, GLuint divisor);

GLuint		ds_glGetUniformLocation(GLuint program, const GLchar *name);
void		ds_glUniform1f(GLint location, GLfloat v0);
void		ds_glUniform2f(GLint location, GLfloat v0, GLfloat v1);
void		ds_glUniform3f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
void		ds_glUniform4f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3);
void		ds_glUniform1i(GLint location, GLint v0);
void		ds_glUniform2i(GLint location, GLint v0, GLint v1);
void		ds_glUniform3i(GLint location, GLint v0, GLint v1, GLint v2);
void		ds_glUniform4i(GLint location, GLint v0, GLint v1, GLint v2, GLint v3);
void		ds_glUniform1ui(GLint location, GLuint v0);
void		ds_glUniform2ui(GLint location, GLuint v0, GLuint v1);
void		ds_glUniform3ui(GLint location, GLuint v0, GLuint v1, GLuint v2);
void		ds_glUniform4ui(GLint location, GLuint v0, GLuint v1, GLuint v2, GLuint v3);
void		ds_glUniform1fv(GLint location, GLsizei count, const GLfloat *value);
void		ds_glUniform2fv(GLint location, GLsizei count, const GLfloat *value);
void		ds_glUniform3fv(GLint location, GLsizei count, const GLfloat *value);
void		ds_glUniform4fv(GLint location, GLsizei count, const GLfloat *value);
void		ds_glUniform1iv(GLint location, GLsizei count, const GLint *value);
void		ds_glUniform2iv(GLint location, GLsizei count, const GLint *value);
void		ds_glUniform3iv(GLint location, GLsizei count, const GLint *value);
void		ds_glUniform4iv(GLint location, GLsizei count, const GLint *value);
void		ds_glUniform1uiv(GLint location, GLsizei count, const GLuint *value);
void		ds_glUniform2uiv(GLint location, GLsizei count, const GLuint *value);
void		ds_glUniform3uiv(GLint location, GLsizei count, const GLuint *value);
void		ds_glUniform4uiv(GLint location, GLsizei count, const GLuint *value);
void		ds_glUniformMatrix2fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
void		ds_glUniformMatrix3fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
void		ds_glUniformMatrix4fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);

/* gles2	 wrappers with state checks to reduce / get statistics on calls */
void 		ds_glEnableBlending(void);
void 		ds_glDisableBlending(void);
void 		ds_glBlendEquation(const GLenum eq);
void 		ds_glBlendEquationSeparate(const GLenum eq_rgb, const GLenum eq_a);
void 		ds_glBlendFunc(const GLenum sfactor, const GLenum dfactor);
void 		ds_glBlendFuncSeparate(const GLenum srcRGB, const GLenum dstRGB, const GLenum srcAlpha, const GLenum dstAlpha);

void 		ds_glEnableFaceCulling(void);
void 		ds_glDisableFaceCulling(void);
void 		ds_glCullFace(const GLenum mode);
void 		ds_glFrontFace(const GLenum mode);

void 		ds_glEnableDepthTesting(void);
void 		ds_glDisableDepthTesting(void);

/* Note: These functions work exactly how gles2 expects them too, but in reality we are getting back and using 
 * texture indices, which under the hood gets translated to actual opengl texture names. This is done so that 
 * we can do some internal book-keeping state of buffers and opengl.
 */
void 	ds_glGenTextures(const GLsizei count, GLuint tx[]);
void 	ds_glDeleteTextures(const GLsizei count, const GLuint tx[]);
void 	ds_glBindTexture(const GLenum target, const GLuint tx);
void 	ds_glTexImage2D(const GLenum target,
		      const GLint level,
		      const GLint internalformat,
		      const GLsizei width,
		      const GLsizei height,
		      const GLint border,
		      const GLenum format,
		      const GLenum type,
		      const void *data);
void 	ds_glTexParameteri(const GLenum target, const GLenum pname, const GLint param);
void 	ds_glTexParameterf(const GLenum target, const GLenum pname, const GLfloat param);
void 	ds_glTexParameteriv(const GLenum target, const GLenum pname, const GLint *params);
void 	ds_glTexParameterfv(const GLenum target, const GLenum pname, const GLfloat *params);
void 	ds_glActiveTexture(const GLenum tx_unit);
void	ds_glGenerateMipmap(GLenum target);

void	ds_glGetShaderiv(GLuint shader, GLenum pname, GLint *params);
void	ds_glGetShaderInfoLog(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
GLuint 	ds_glCreateShader(GLenum type);
void 	ds_glShaderSource(GLuint shader, GLsizei count, const GLchar **string, const GLint *length);
void 	ds_glCompileShader(GLuint shader);
void 	ds_glAttachShader(GLuint program, GLuint shader);
void 	ds_glDetachShader(GLuint program, GLuint shader);
void 	ds_glDeleteShader(GLuint shader);

#ifdef DS_GL_DEBUG

void 	gl_StateAssert(void);

#define GL_STATE_ASSERT		gl_StateAssert();

#else

#define GL_STATE_ASSERT	

#endif

#ifdef __cplusplus
} 
#endif

#endif

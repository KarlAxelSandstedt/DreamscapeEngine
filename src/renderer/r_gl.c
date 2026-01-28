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

#include <stdlib.h>

#include "r_local.h"
#include "sys_public.h"

struct gl_limits gl_limits_storage = { 0 };
struct gl_limits *g_gl_limits = &gl_limits_storage;

struct array_list_intrusive *g_gl_state_list = NULL;
u32 g_gl_state = U32_MAX;
struct pool g_binding_pool = { 0 };

u32 			tx_in_use = 0;	/* number of texture names currently allocated (glGen*, glDel*) */
struct array_list *	tx_list = NULL;

#ifdef DS_GL_DEBUG

static void gl_state_assert_blending(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	//TODO assert blending equations
	//TODO assert blending funcs 
	GLint eq_rgb, eq_a, func_s_rgb, func_s_a, func_d_rgb, func_d_a;
	gl_state->func.glGetIntegerv(GL_BLEND_EQUATION_RGB,   &eq_rgb);
	gl_state->func.glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &eq_a);
	gl_state->func.glGetIntegerv(GL_BLEND_SRC_RGB,   &func_s_rgb);
	gl_state->func.glGetIntegerv(GL_BLEND_SRC_ALPHA, &func_s_a);
	gl_state->func.glGetIntegerv(GL_BLEND_DST_RGB,   &func_d_rgb);
	gl_state->func.glGetIntegerv(GL_BLEND_DST_ALPHA, &func_d_a);

	ds_Assert(gl_state->blend == gl_state->func.glIsEnabled(GL_BLEND));
	ds_Assert((GLint) gl_state->eq_rgb == eq_rgb);
	ds_Assert((GLint) gl_state->eq_a == eq_a);
	ds_Assert((GLint) gl_state->func_s_rgb == func_s_rgb);
	ds_Assert((GLint) gl_state->func_s_a == func_s_a);
	ds_Assert((GLint) gl_state->func_d_rgb == func_d_rgb);
	ds_Assert((GLint) gl_state->func_d_a == func_d_a);
}

static void gl_state_assert_culling(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	GLint cull_mode, face_front;
	gl_state->func.glGetIntegerv(GL_CULL_FACE_MODE, &cull_mode);
	gl_state->func.glGetIntegerv(GL_FRONT_FACE, &face_front);

	ds_Assert(gl_state->cull_face == gl_state->func.glIsEnabled(GL_CULL_FACE));
	ds_Assert((GLint) gl_state->cull_mode == cull_mode);
	ds_Assert((GLint) gl_state->face_front == face_front);
}

static void gl_state_assert_texture_unit(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	GLint tx_unit_active;
	gl_state->func.glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint *) &tx_unit_active);
	ds_Assert((GLint) gl_state->tx_unit_active == (tx_unit_active - GL_TEXTURE0));

	for (GLuint i = 0; i < g_gl_limits->tx_unit_count; ++i)
	{
		const GLuint tx1 = gl_state->tx_unit[i].gl_tx_2d_index;
		const GLuint tx2 = gl_state->tx_unit[i].gl_tx_cube_map_index;
		const GLuint txi = (tx1) ? tx1 : tx2;
		struct gl_texture *tx;
		if (txi)
		{
			tx = array_list_address(tx_list, txi);
			struct texture_unit_binding *binding = NULL;
			for (u32 k = tx->binding_list.first; k != DLL_NULL; k = binding->dll_next)
			{
 				binding = PoolAddress(&g_binding_pool, k);
				ds_Assert(binding->header.allocated);
				if (binding->context == g_gl_state)
				{
					ds_Assert(binding->tx_unit == i);
					break;
				}
			}
		}

		ds_glActiveTexture(GL_TEXTURE0 + i);

		GLint tx_2d, tx_cube, wrap_s, wrap_t, mag_filter, min_filter;
		if (tx1 != 0)
		{
			tx = array_list_address(tx_list, tx1);
			gl_state->func.glGetIntegerv(GL_TEXTURE_BINDING_2D, &tx_2d);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, &mag_filter);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, &min_filter);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, &wrap_s);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, &wrap_t);

			ds_Assert((GLint) tx->name == tx_2d);
			ds_Assert(tx->wrap_s == wrap_s);
			ds_Assert(tx->wrap_t == wrap_t);
			ds_Assert(tx->mag_filter == mag_filter);
			ds_Assert(tx->min_filter == min_filter);
		}

		if (tx2 != 0)
		{
			tx = array_list_address(tx_list, tx2);
			gl_state->func.glGetIntegerv(GL_TEXTURE_BINDING_CUBE_MAP, &tx_cube);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, &mag_filter);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, &min_filter);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, &wrap_s);
			gl_state->func.glGetTexParameteriv(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, &wrap_t);

			ds_Assert((GLint) tx->name == tx_cube);
			ds_Assert(tx->wrap_s == wrap_s);
			ds_Assert(tx->wrap_t == wrap_t);
			ds_Assert(tx->mag_filter == mag_filter);
			ds_Assert(tx->min_filter == min_filter);
		}
	}

	gl_state->func.glActiveTexture(tx_unit_active);
}

void gl_state_assert(void)
{
	gl_state_assert_blending();
	gl_state_assert_culling();
	gl_state_assert_texture_unit();
}

#endif

static void ds_glEnable(GLenum cap)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glEnable(cap);
}

static void ds_glDisable(GLenum cap)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDisable(cap);
}

void ds_glGetTexParameterfv(GLenum target, GLenum pname, GLfloat *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetTexParameterfv(target, pname, params);
}

void ds_glGetTexParameteriv(GLenum target, GLenum pname, GLint *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetTexParameteriv(target, pname, params);
}

void ds_glGetIntegerv(GLenum pname, GLint *data)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetIntegerv(pname, data);
}

const GLubyte *ds_glGetString(GLenum name)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	return gl_state->func.glGetString(name);
}

void ds_glClear(GLbitfield mask)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glClear(mask);
}

void ds_glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glClearColor(red, green, blue, alpha);
}

void ds_glViewport(GLint x, GLint y, GLsizei width, GLsizei height)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glViewport(x, y, width, height);
}

void ds_glGenBuffers(GLsizei n, GLuint *buffers)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGenBuffers(n, buffers);
}

void ds_glBindBuffer(GLenum target, GLuint buffer)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glBindBuffer(target, buffer);
}

void ds_glBufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glBufferData(target, size, data, usage);
}

void ds_glBufferSubData(GLenum target, GLintptr offset, GLsizeiptr size,  const void *data)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glBufferSubData(target, offset, size, data);
}

void ds_glDeleteBuffers(GLsizei n, const GLuint *buffers)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDeleteBuffers(n, buffers);
}

void ds_glDrawArrays(GLenum mode, GLint first, GLsizei count)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDrawArrays(mode, first, count);
}

void ds_glDrawElements(GLenum mode, GLsizei count, GLenum type, const void *indices)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDrawElements(mode, count, type, indices);
}

void ds_glDrawArraysInstanced(GLenum mode, GLint first, GLint count, GLsizei primcount)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDrawArraysInstanced(mode, first, count, primcount);
}

void ds_glDrawElementsInstanced(GLenum mode, GLsizei count, GLenum type, const void *indices, GLsizei primcount)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDrawElementsInstanced(mode, count, type, indices, primcount);
}

void ds_glGenVertexArrays(GLsizei n, GLuint *arrays)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGenVertexArrays(n, arrays);
}

void ds_glDeleteVertexArrays(GLsizei n, const GLuint *arrays)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDeleteVertexArrays(n, arrays);
}

void ds_glBindVertexArray(GLuint array)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glBindVertexArray(array);
}

void ds_glEnableVertexAttribArray(GLuint index)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glEnableVertexAttribArray(index);
}

void ds_glDisableVertexAttribArray(GLuint index)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDisableVertexAttribArray(index);
}

void ds_glVertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glVertexAttribPointer(index, size, type, normalized, stride, pointer);
}

void ds_glVertexAttribIPointer(GLuint index, GLint size, GLenum type, GLsizei stride, const void *pointer)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glVertexAttribIPointer(index, size, type, stride, pointer);
}

void ds_glVertexAttribDivisor(GLuint index, GLuint divisor)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glVertexAttribDivisor(index, divisor);
}

GLuint ds_glGetUniformLocation(GLuint program, const GLchar *name)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	return gl_state->func.glGetUniformLocation(program, name);
}

void ds_glUniform1f(GLint location, GLfloat v0)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1f(location, v0);
}

void ds_glUniform2f(GLint location, GLfloat v0, GLfloat v1)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2f(location, v0, v1);
}

void ds_glUniform3f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3f(location, v0, v1, v2);
}

void ds_glUniform4f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4f(location, v0, v1, v2, v3);
}

void ds_glUniform1i(GLint location, GLint v0)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1i(location, v0);
}

void ds_glUniform2i(GLint location, GLint v0, GLint v1)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2i(location, v0, v1);
}

void ds_glUniform3i(GLint location, GLint v0, GLint v1, GLint v2)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3i(location, v0, v1, v2);
}

void ds_glUniform4i(GLint location, GLint v0, GLint v1, GLint v2, GLint v3)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4i(location, v0, v1, v2, v3);
}

void ds_glUniform1ui(GLint location, GLuint v0)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1ui(location, v0);
}

void ds_glUniform2ui(GLint location, GLuint v0, GLuint v1)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2ui(location, v0, v1);
}

void ds_glUniform3ui(GLint location, GLuint v0, GLuint v1, GLuint v2)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3ui(location, v0, v1, v2);
}

void ds_glUniform4ui(GLint location, GLuint v0, GLuint v1, GLuint v2, GLuint v3)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4ui(location, v0, v1, v2, v3);
}

void ds_glUniform1fv(GLint location, GLsizei count, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1fv(location, count, value);
}

void ds_glUniform2fv(GLint location, GLsizei count, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2fv(location, count, value);
}

void ds_glUniform3fv(GLint location, GLsizei count, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3fv(location, count, value);
}

void ds_glUniform4fv(GLint location, GLsizei count, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4fv(location, count, value);
}

void ds_glUniform1iv(GLint location, GLsizei count, const GLint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1iv(location, count, value);
}

void ds_glUniform2iv(GLint location, GLsizei count, const GLint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2iv(location, count, value);
}

void ds_glUniform3iv(GLint location, GLsizei count, const GLint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3iv(location, count, value);
}

void ds_glUniform4iv(GLint location, GLsizei count, const GLint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4iv(location, count, value);
}

void ds_glUniform1uiv(GLint location, GLsizei count, const GLuint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform1uiv(location, count, value);
}

void ds_glUniform2uiv(GLint location, GLsizei count, const GLuint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform2uiv(location, count, value);
}

void ds_glUniform3uiv(GLint location, GLsizei count, const GLuint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform3uiv(location, count, value);
}

void ds_glUniform4uiv(GLint location, GLsizei count, const GLuint *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniform4uiv(location, count, value);
}

void ds_glUniformMatrix2fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniformMatrix2fv(location, count, transpose, value);
}

void ds_glUniformMatrix3fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniformMatrix3fv(location, count, transpose, value);
}

void ds_glUniformMatrix4fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glUniformMatrix4fv(location, count, transpose, value);
}




static struct gl_texture *internal_tx_unit_get_texture_target(const GLenum target)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx;
	const GLenum unit = gl_state->tx_unit_active;
	if (target == GL_TEXTURE_2D)
	{
		tx = array_list_address(tx_list, gl_state->tx_unit[unit].gl_tx_2d_index);
	}
	else
	{
		ds_Assert(target == GL_TEXTURE_CUBE_MAP);
		tx = array_list_address(tx_list, gl_state->tx_unit[unit].gl_tx_cube_map_index);
	}

	return tx;
}

GLboolean ds_glIsEnabled(GLenum cap)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	return gl_state->func.glIsEnabled(cap);
}

void ds_glGetShaderiv(GLuint shader, GLenum pname, GLint *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetShaderiv(shader, pname, params);
}

void ds_glGetShaderInfoLog(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetShaderInfoLog(shader, bufSize, length, infoLog);
}

GLuint ds_glCreateShader(GLenum type)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	return gl_state->func.glCreateShader(type);
}

void ds_glShaderSource(GLuint shader, GLsizei count, const GLchar **string, const GLint *length)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glShaderSource(shader, count, string, length);
}

void ds_glCompileShader(GLuint shader)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glCompileShader(shader);
}

void ds_glAttachShader(GLuint program, GLuint shader)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glAttachShader(program, shader);
}

void ds_glDetachShader(GLuint program, GLuint shader)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDetachShader(program, shader);
}

void ds_glDeleteShader(GLuint shader)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glDeleteShader(shader);
}

void ds_glDisableBlending(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->blend)
	{
		gl_state->func.glDisable(GL_BLEND);
		gl_state->blend = 0;
	}
}

void ds_glEnableBlending(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (!gl_state->blend)
	{
		gl_state->func.glEnable(GL_BLEND);
		gl_state->blend = 1;
	}
}

void ds_glBlendEquation(const GLenum eq)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->eq_rgb != eq || gl_state->eq_a != eq)
	{
		gl_state->func.glBlendEquation(eq);
		gl_state->eq_rgb = eq;
		gl_state->eq_a = eq;
	}
}

void ds_glBlendEquationSeparate(const GLenum eq_rgb, const GLenum eq_a)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->eq_rgb != eq_rgb || gl_state->eq_a != eq_a)
	{
		gl_state->func.glBlendEquationSeparate(eq_rgb, eq_a);
		gl_state->eq_rgb = eq_rgb;
		gl_state->eq_a = eq_a;
	}
}

void ds_glBlendFunc(const GLenum sfactor, const GLenum dfactor)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->func_s_rgb != sfactor
			|| gl_state->func_s_a != sfactor
			|| gl_state->func_d_rgb != dfactor
			|| gl_state->func_d_a != dfactor)
	{
		gl_state->func.glBlendFunc(sfactor, dfactor);
		gl_state->func_s_rgb = sfactor;
		gl_state->func_s_a = sfactor;
		gl_state->func_d_rgb = dfactor;
		gl_state->func_d_a = dfactor;
	}
}

void ds_glBlendFuncSeparate(const GLenum srcRGB, const GLenum dstRGB, const GLenum srcAlpha, const GLenum dstAlpha)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->func_s_rgb != srcRGB
			|| gl_state->func_s_a != srcAlpha
			|| gl_state->func_d_rgb != dstRGB
			|| gl_state->func_d_a != dstAlpha)
	{
		gl_state->func.glBlendFuncSeparate(srcRGB, dstRGB, srcAlpha, dstAlpha);
		gl_state->func_s_rgb = srcRGB;
		gl_state->func_s_a = srcAlpha;
		gl_state->func_d_rgb = dstRGB;
		gl_state->func_d_a = dstAlpha;
	}
}

void ds_glEnableFaceCulling(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (!gl_state->cull_face)
	{
		gl_state->cull_face = 1;
		gl_state->func.glEnable(GL_CULL_FACE);
	}
}

void ds_glDisableFaceCulling(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->cull_face)
	{
		gl_state->cull_face = 0;
		gl_state->func.glDisable(GL_CULL_FACE);
	}
}

void ds_glCullFace(const GLenum mode)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->cull_mode != mode)
	{
		gl_state->cull_mode = mode;
		gl_state->func.glCullFace(mode);
	}
}

void ds_glFrontFace(const GLenum mode)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->face_front != mode)
	{
		gl_state->face_front = mode;
		gl_state->func.glFrontFace(mode);
	}
}

void ds_glActiveTexture(const GLenum tx_unit)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	const GLuint i = (tx_unit - GL_TEXTURE0);
	if (gl_state->tx_unit_active != i)
	{
		gl_state->tx_unit_active = i;
		gl_state->func.glActiveTexture(tx_unit);
	}
}

void ds_glGenerateMipmap(GLenum target)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGenerateMipmap(target);
}

void ds_glGenTextures(const GLsizei count, GLuint tx[])
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	GLsizei i = 0;
	for (; i < count; ++i)
	{
		tx[i] = (u32) array_list_reserve_index(tx_list);
		struct gl_texture *tx_ptr = array_list_address(tx_list, tx[i]);
		gl_state->func.glGenTextures(1, &tx_ptr->name);
		if (tx_ptr->name == 0)
		{
			array_list_remove_index(tx_list, tx[i]);
			LogString(T_RENDERER, S_ERROR, "GL internal; glGenTexture failed");
			break;
		}

		tx_in_use += 1;
		tx_ptr->binding_list = dll_Init(struct texture_unit_binding);
		tx_ptr->wrap_s = GL_REPEAT;
		tx_ptr->wrap_t = GL_REPEAT;
		tx_ptr->mag_filter = GL_LINEAR;
		tx_ptr->min_filter = GL_NEAREST_MIPMAP_LINEAR;
	}
}

void ds_glDeleteTextures(const GLsizei count, const GLuint tx[])
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	for (GLsizei i = 0; i < count; ++i)
	{
		if (tx[i] > 0)
		{
			struct gl_texture *tx_ptr = array_list_address(tx_list, tx[i]);
			struct texture_unit_binding *binding;
			for (u32 k = tx_ptr->binding_list.first; k != DLL_NULL; k = dll_Next(binding))
			{
				binding = PoolAddress(&g_binding_pool, k);
				struct gl_state *local_state = array_list_intrusive_address(g_gl_state_list, binding->context);
				ds_Assert(binding->tx_unit < g_gl_limits->tx_unit_count);
				if (local_state->tx_unit[binding->tx_unit].gl_tx_2d_index == tx[i])
				{
					local_state->tx_unit[binding->tx_unit].gl_tx_2d_index = 0;
				}
				else	
				{
					ds_Assert(local_state->tx_unit[binding->tx_unit].gl_tx_cube_map_index == tx[i]);
					local_state->tx_unit[binding->tx_unit].gl_tx_cube_map_index = 0;
				}
				local_state->tx_unit[binding->tx_unit].binding = DLL_NULL;
			}
			tx_in_use -= 1;
			gl_state->func.glDeleteTextures(1, &tx_ptr->name);
			array_list_remove_index(tx_list, tx[i]);
		}
	}
}

void ds_glBindTexture(const GLenum target, const GLuint tx)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture_unit *unit = gl_state->tx_unit + gl_state->tx_unit_active;

	GLuint prev_tx;
	if (target == GL_TEXTURE_2D)
	{
		ds_AssertString(tx == 0 || unit->gl_tx_cube_map_index != tx, "While it is may be possible  \
								to bind a texture to several *different* type of \
								targets, you are probably doing something fucky  \
								wucky by mistake.\n");
		prev_tx = unit->gl_tx_2d_index;
		unit->gl_tx_2d_index = tx;
	}
	else
	{
		ds_Assert(target == GL_TEXTURE_CUBE_MAP);
		ds_AssertString(tx == 0 || unit->gl_tx_2d_index != tx,       "While it is may be possible \
								to bind a texture to several *different* type of \
								targets, you are probably doing something fucky  \
								wucky by mistake.\n");
		prev_tx = unit->gl_tx_cube_map_index;
		unit->gl_tx_cube_map_index = tx;
	}

	if (prev_tx == tx)
	{
		return;
	}

	if (prev_tx)
	{
		struct gl_texture *texture = array_list_address(tx_list, prev_tx);
		if (texture->binding_list.first == unit->binding)
		{
			struct texture_unit_binding *binding = PoolAddress(&g_binding_pool, unit->binding);
			texture->binding_list.first = binding->dll_next;
		}
		//fprintf(stderr, "context; %p\t DEL binding (UNIT,TEXTURE) : (%u,%u)\n", g_gl_state, gl_state->tx_unit_active, texture->name);
		dll_Remove(&texture->binding_list, g_binding_pool.buf, unit->binding);
		PoolRemove(&g_binding_pool, unit->binding);
		unit->binding = DLL_NULL;
	}

	if (tx)
	{
		struct gl_texture *texture = array_list_address(tx_list, tx);
		struct texture_unit_binding *binding;
		for (u32 i = texture->binding_list.first; i != DLL_NULL; i = binding->dll_next)
		{
			binding = PoolAddress(&g_binding_pool, i);
			if (binding->context == g_gl_state)
			{
				/* if binding->tx_unit == gl_state->tx_unit_active, then tx already bound to current unit, so prev_tx == tx */
				//f (binding->tx_unit != gl_state->tx_unit_active)
				//{
					if (texture->binding_list.first == i)
					{
						texture->binding_list.first = binding->dll_next;
					}

					if (target == GL_TEXTURE_2D)
					{
						gl_state->tx_unit[binding->tx_unit].gl_tx_2d_index = 0;
					}
					else
					{
						gl_state->tx_unit[binding->tx_unit].gl_tx_cube_map_index = 0;
					}
					gl_state->tx_unit[binding->tx_unit].binding = DLL_NULL;

					dll_Remove(&texture->binding_list, g_binding_pool.buf, i);
					PoolRemove(&g_binding_pool, i);
					//fprintf(stderr, "context; %p\t DEL binding (UNIT,TEXTURE) : (%u,%u)\n", g_gl_state, binding->tx_unit, texture->name);
				//}
				break;
			}
		}

		struct slot slot;
		slot = PoolAdd(&g_binding_pool);
	 	dll_Prepend(&texture->binding_list, g_binding_pool.buf, slot.index);
		
		binding = slot.address;
		binding->context = g_gl_state;
		binding->tx_unit = gl_state->tx_unit_active;

		texture->binding_list.first = slot.index;

		unit->binding = slot.index;
		gl_state->func.glBindTexture(target, texture->name);
		//fprintf(stderr, "context; %p\t NEW binding (UNIT,TEXTURE) : (%u,%u)\n", g_gl_state, gl_state->tx_unit_active, texture->name);
	}
	else
	{
		gl_state->func.glBindTexture(target, 0);
		//fprintf(stderr, "context; %p\t binding (UNIT,TEXTURE) : (%u,%u)\n", g_gl_state, gl_state->tx_unit_active, 0);
	}

	//Breakpoint(1);
}

void ds_glTexImage2D(const GLenum target,
		      const GLint level,
		      const GLint internalformat,
		      const GLsizei width,
		      const GLsizei height,
		      const GLint border,
		      const GLenum format,
		      const GLenum type,
		      const void *data)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx = internal_tx_unit_get_texture_target(target);

	tx->level = level;
	tx->internalformat = format;
	tx->width = width;
	tx->height = height;
	tx->format = format;
	tx->type = type;
	gl_state->func.glTexImage2D(target, level, internalformat, width, height, border, format, type, data);

	GLsizei max_size;
	if (target == GL_TEXTURE_2D)
	{
		max_size = g_gl_limits->max_2d_tx_size;
	}
	else
	{
		ds_Assert(target == GL_TEXTURE_CUBE_MAP);
		max_size = g_gl_limits->max_cube_map_tx_size;
	}

	if (tx->width < 0 || max_size < tx->width || tx->height < 0 || max_size < tx->height)
	{
		Log(T_RENDERER, S_ERROR, 
				"(glTexImage2D) (width, height) =  (%i, %i) must both be in range [0, %i]", 
				tx->width, tx->height, max_size);
		tx->width = 0;
		tx->height = 0;
	}
}

void ds_glTexParameteri(const GLenum target, const GLenum pname, const GLint param)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx = internal_tx_unit_get_texture_target(target);

	switch (pname)
	{
		case GL_TEXTURE_MAG_FILTER: { tx->mag_filter = param; } break;
		case GL_TEXTURE_MIN_FILTER: { tx->min_filter = param; } break;
		case GL_TEXTURE_WRAP_S:	    { tx->wrap_s = param; } break;
		case GL_TEXTURE_WRAP_T:	    { tx->wrap_t = param; } break;
		default: 		    { ds_Assert(0); }; break;
	}

	gl_state->func.glTexParameteri(target, pname, param);
}

void ds_glTexParameterf(const GLenum target, const GLenum pname, const GLfloat param)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx = internal_tx_unit_get_texture_target(target);

	switch (pname)
	{
		case GL_TEXTURE_MAG_FILTER: { tx->mag_filter = param; } break;
		case GL_TEXTURE_MIN_FILTER: { tx->min_filter = param; } break;
		case GL_TEXTURE_WRAP_S:	    { tx->wrap_s = param; } break;
		case GL_TEXTURE_WRAP_T:	    { tx->wrap_t = param; } break;
		default: 		    { ds_Assert(0); }; break;
	}

	gl_state->func.glTexParameterf(target, pname, param);
}

void ds_glTexParameteriv(const GLenum target, const GLenum pname, const GLint *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx = internal_tx_unit_get_texture_target(target);

	switch (pname)
	{
		default: 		    	{ ds_Assert(0); } break;
	}

	gl_state->func.glTexParameteriv(target, pname, params);
}

void ds_glTexParameterfv(const GLenum target, const GLenum pname, const GLfloat *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	struct gl_texture *tx = internal_tx_unit_get_texture_target(target);

	switch (pname)
	{
		default: 		    	{ ds_Assert(0); } break;
	}

	gl_state->func.glTexParameterfv(target, pname, params);
}

void ds_glEnableDepthTesting(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (!gl_state->depth)
	{
		gl_state->func.glEnable(GL_DEPTH_TEST);
		gl_state->depth = 1;
	}
}

void ds_glDisableDepthTesting(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (gl_state->depth)
	{
		gl_state->func.glDisable(GL_DEPTH_TEST);
		gl_state->depth = 0;
	}
}

GLuint ds_glCreateProgram(void)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	return gl_state->func.glCreateProgram();
}

void ds_glLinkProgram(GLuint program)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glLinkProgram(program);
}

void ds_glUseProgram(GLuint program)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (program != gl_state->program)
	{
		gl_state->program = program;
		gl_state->func.glUseProgram(program);
	}
}

void ds_glDeleteProgram(GLuint program)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	if (program == gl_state->program)
	{
		gl_state->program = 0;
	}

	gl_state->func.glDeleteProgram(program);
}

void ds_glGetProgramiv(GLuint program, GLenum pname, GLint *params)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetProgramiv(program, pname, params);
}

void ds_glGetProgramInfoLog(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, g_gl_state);
	gl_state->func.glGetProgramInfoLog(program, bufSize, length, infoLog);
}

u32 gl_state_alloc(void)
{
	const u32 g_gl_state_prev = g_gl_state;
	const u32 gl_state_index = array_list_intrusive_reserve_index(g_gl_state_list);
	g_gl_state = gl_state_index;
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, gl_state_index);
	gl_functions_init(&gl_state->func);

	static u32 once = 1;
	if (once)
	{
		once = 0;

		gl_state->func.glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, (GLint *) &g_gl_limits->tx_unit_count);
		gl_state->func.glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, (GLint *) &g_gl_limits->max_tx_units_fragment);
		gl_state->func.glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, (GLint *) &g_gl_limits->max_tx_units_vertex);
		gl_state->func.glGetIntegerv(GL_MAX_TEXTURE_SIZE, (GLint *) &g_gl_limits->max_2d_tx_size);
		gl_state->func.glGetIntegerv(GL_MAX_CUBE_MAP_TEXTURE_SIZE, (GLint *) &g_gl_limits->max_cube_map_tx_size);
		gl_state->func.glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, (GLint *) &g_gl_limits->max_vertex_attributes);
		gl_state->func.glGetIntegerv(GL_MAX_VARYING_VECTORS, (GLint *) &g_gl_limits->max_varying_vectors);
		gl_state->func.glGetIntegerv(GL_MAX_ELEMENT_INDEX, (GLint *) &g_gl_limits->max_element_index);
	
		Log(T_RENDERER, S_NOTE, "               \n\
			        GL Vendor           - %s\n\
				GL Renderer         - %s\n\
				GL Version          - %s\n\
				GL Shading Language - %s"
				, gl_state->func.glGetString(GL_VENDOR)
				, gl_state->func.glGetString(GL_RENDERER)
				, gl_state->func.glGetString(GL_VERSION)
				, gl_state->func.glGetString(GL_SHADING_LANGUAGE_VERSION));

		Log(T_RENDERER, S_NOTE, "opengl limits:\
				\n\tGL_MAX_COMBINED_TEXTURE_IMAGE_UNITS: %u\
				\n\tGL_MAX_TEXTURE_IMAGE_UNITS:          %u\
				\n\tGL_MAX_VERTEX_TEXTURE_IMAGE_UNITS:   %u\
				\n\tGL_MAX_TEXTURE_SIZE:                 %u\
				\n\tGL_MAX_CUBE_MAP_TEXTURE_SIZE:        %u\
				\n\tGL_MAX_VERTEX_ATTRIBS:               %u\
				\n\tGL_MAX_VARYING_VECTORS:              %u\
				\n\tGL_MAX_ELEMENT_INDEX:                %u\
				"
			,g_gl_limits->tx_unit_count
			,g_gl_limits->max_tx_units_fragment
			,g_gl_limits->max_tx_units_vertex
			,(u32) g_gl_limits->max_2d_tx_size
			,(u32) g_gl_limits->max_cube_map_tx_size
			,g_gl_limits->max_vertex_attributes
			,g_gl_limits->max_varying_vectors
			,g_gl_limits->max_element_index
			);
	}

	gl_state->blend = U32_MAX;
	gl_state->eq_rgb = GL_INVALID_ENUM;
	gl_state->eq_a = GL_INVALID_ENUM;
	gl_state->func_s_rgb = GL_INVALID_ENUM;
	gl_state->func_s_a = GL_INVALID_ENUM;
	gl_state->func_d_rgb = GL_INVALID_ENUM;
	gl_state->func_d_a = GL_INVALID_ENUM;
	ds_glDisableBlending();
	ds_glBlendEquation(GL_FUNC_ADD);
	ds_glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	gl_state->program = U32_MAX;
	ds_glUseProgram(0);

	gl_state->cull_face = 0;
	gl_state->cull_mode = GL_INVALID_ENUM;
	gl_state->face_front = GL_INVALID_ENUM;
	ds_glEnableFaceCulling();
	ds_glFrontFace(GL_CCW);
	ds_glCullFace(GL_BACK);

	gl_state->depth = 0;
	ds_glEnableDepthTesting();

	gl_state->tx_unit_active = g_gl_limits->tx_unit_count;
	ds_glActiveTexture(GL_TEXTURE0 + 0);
	ds_Assert(gl_state->tx_unit_active == 0);

	gl_state->tx_unit = malloc(g_gl_limits->tx_unit_count * sizeof(struct gl_texture_unit));
	for (GLuint i = 0; i < g_gl_limits->tx_unit_count; ++i)
	{
		gl_state->tx_unit[i].binding = DLL_NULL;
		gl_state->tx_unit[i].gl_tx_2d_index = 0;
		gl_state->tx_unit[i].gl_tx_cube_map_index = 0;
	}
	GL_STATE_ASSERT;

	g_gl_state = g_gl_state_prev;
	return gl_state_index;
}

void gl_state_free(const u32 gl_state_index)
{
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, gl_state_index);
	for (u32 i = 0; i < g_gl_limits->tx_unit_count; ++i)
	{
		if (gl_state->tx_unit[i].binding != DLL_NULL)
		{
			const GLuint tx1 = gl_state->tx_unit[i].gl_tx_2d_index;
			const GLuint tx2 = gl_state->tx_unit[i].gl_tx_cube_map_index;
			const GLuint tx = (tx1) ? tx1 : tx2;
			struct gl_texture *texture = array_list_address(tx_list, tx);

			if (texture->binding_list.first == gl_state->tx_unit[i].binding)
			{
				struct texture_unit_binding *binding = PoolAddress(&g_binding_pool, texture->binding_list.first);
				texture->binding_list.first = binding->dll_next;
			}
			dll_Remove(&texture->binding_list, g_binding_pool.buf, gl_state->tx_unit[i].binding);
			PoolRemove(&g_binding_pool, gl_state->tx_unit[i].binding);
		}
	}

	free(gl_state->tx_unit);
	array_list_intrusive_remove(g_gl_state_list, gl_state);

	if (g_gl_state == gl_state_index)
	{
		g_gl_state = U32_MAX;
	}
}

void gl_state_set_current(const u32 gl_state_index)
{
	g_gl_state = gl_state_index;
	struct gl_state *gl_state = array_list_intrusive_address(g_gl_state_list, gl_state_index);
}

void gl_state_list_alloc(void)
{
	tx_list = array_list_alloc(NULL, 256, sizeof(struct gl_texture), ARRAY_LIST_GROWABLE);
	const GLuint stub_index = array_list_reserve_index(tx_list);
	ds_AssertString(stub_index == 0, "Reserve first index for stub, that we we can return 0 of *Texture calls to indicate error"); 
	g_gl_state_list = array_list_intrusive_alloc(NULL, 8, sizeof(struct gl_state), ARRAY_LIST_GROWABLE);
	g_binding_pool = PoolAlloc(NULL, 192, struct texture_unit_binding, GROWABLE);
}

void gl_state_list_free(void)
{
	array_list_free(tx_list);
	array_list_intrusive_free(g_gl_state_list);
	PoolDealloc(&g_binding_pool);
}

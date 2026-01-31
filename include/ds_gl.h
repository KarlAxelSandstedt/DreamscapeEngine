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

#ifndef __DS_GL_H__
#define __DS_GL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"

#if __DS_PLATFORM__ == __DS_WIN64__ || __DS_PLATFORM__ == __DS_LINUX__
#include "GL/glcorearb.h"
#include "GL/glext.h"
#elif __DS_PLATFORM__ == __DS_WEB__
#include "GLES3/gl3.h"
#define APIENTRY	GL_APIENTRY
#endif


typedef void 		(APIENTRY *type_glGetIntegerv)(GLenum pname, GLint *data);
typedef const GLubyte * (APIENTRY *type_glGetString)(GLenum);
typedef void 		(APIENTRY *type_glGetTexParameterfv)(GLenum target, GLenum pname, GLfloat *params);
typedef void 		(APIENTRY *type_glGetTexParameteriv)(GLenum target, GLenum pname, GLint *params);
typedef void		(APIENTRY *type_glViewport)(GLint, GLint, GLsizei, GLsizei);
typedef void 		(APIENTRY *type_glClearColor)(GLclampf, GLclampf, GLclampf, GLclampf);
typedef void 		(APIENTRY *type_glClear)(GLbitfield);
typedef void 		(APIENTRY *type_glEnable)(GLenum);
typedef void 		(APIENTRY *type_glDisable)(GLenum);
typedef void		(APIENTRY *type_glCullFace)(GLenum);
typedef void		(APIENTRY *type_glFrontFace)(GLenum);
typedef void		(APIENTRY *type_DEBUGPROC)(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *, const void *);
typedef void 		(APIENTRY *type_glDebugMessageCallback)(type_DEBUGPROC, void *);
typedef void 		(APIENTRY *type_glGenBuffers)(GLsizei, GLuint *);
typedef void 		(APIENTRY *type_glBindBuffer)(GLenum, GLuint);
typedef void 		(APIENTRY *type_glBufferData)(GLenum, GLsizeiptr, const void *, GLenum);
typedef void 		(APIENTRY *type_glBufferSubData)(GLenum, GLintptr, GLsizeiptr,  const void *);
typedef void 		(APIENTRY *type_glDeleteBuffers)(GLsizei, const GLuint *);
typedef void 		(APIENTRY *type_glGenVertexArrays)(GLsizei, GLuint *);
typedef void 		(APIENTRY *type_glBindVertexArray)(GLuint array);
typedef void 		(APIENTRY *type_glDeleteVertexArrays)(GLsizei, const GLuint *arrays);
typedef void 		(APIENTRY *type_glEnableVertexAttribArray)(GLuint);
typedef void 		(APIENTRY *type_glDisableVertexAttribArray)(GLuint);
typedef void 		(APIENTRY *type_glVertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void *);
typedef void 		(APIENTRY *type_glVertexAttribIPointer)(GLuint, GLint, GLenum, GLsizei, const void *);
//typedef void 		(APIENTRY *type_glVertexAttribLPointer)(GLuint, GLint, GLenum, GLsizei, const void *);
typedef void		(APIENTRY *type_glVertexAttribDivisor)(GLuint index, GLuint divisor);
typedef GLuint 		(APIENTRY *type_glCreateShader)(GLenum);
typedef void 		(APIENTRY *type_glShaderSource)(GLuint, GLsizei, const GLchar **, const GLint *);
typedef void 		(APIENTRY *type_glCompileShader)(GLuint);
typedef void 		(APIENTRY *type_glAttachShader)(GLuint, GLuint);
typedef void 		(APIENTRY *type_glDetachShader)(GLuint, GLuint);
typedef void 		(APIENTRY *type_glDeleteShader)(GLuint);
typedef GLuint 		(APIENTRY *type_glCreateProgram)(void);
typedef void 		(APIENTRY *type_glLinkProgram)(GLuint);
typedef void 		(APIENTRY *type_glUseProgram)(GLuint);
typedef void 		(APIENTRY *type_glDeleteProgram)(GLuint);
typedef void 		(APIENTRY *type_glGetProgramiv)(GLuint program, GLenum pname, GLint *params);
typedef void 		(APIENTRY *type_glGetProgramInfoLog)(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
typedef void 		(APIENTRY *type_glDrawArrays)(GLenum, GLint, GLsizei);
typedef void 		(APIENTRY *type_glDrawElements)(GLenum, GLsizei, GLenum, const void *);
typedef void 		(APIENTRY *type_glDrawArraysInstanced)(GLenum mode, GLint first, GLint count, GLsizei primcount);
typedef void 		(APIENTRY *type_glDrawElementsInstanced)(GLenum mode, GLsizei count, GLenum type, const void *indices, GLsizei primcount);
typedef GLuint		(APIENTRY *type_glGetUniformLocation)(GLuint, const GLchar *);
typedef void		(APIENTRY *type_glUniform1f)(GLint, GLfloat);
typedef void		(APIENTRY *type_glUniform2f)(GLint, GLfloat, GLfloat);
typedef void		(APIENTRY *type_glUniform3f)(GLint, GLfloat, GLfloat, GLfloat);
typedef void		(APIENTRY *type_glUniform4f)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
typedef void		(APIENTRY *type_glUniform1i)(GLint, GLint);
typedef void		(APIENTRY *type_glUniform2i)(GLint, GLint, GLint);
typedef void		(APIENTRY *type_glUniform3i)(GLint, GLint, GLint, GLint);
typedef void		(APIENTRY *type_glUniform4i)(GLint, GLint, GLint, GLint, GLint);
typedef void		(APIENTRY *type_glUniform1ui)(GLint, GLuint);
typedef void		(APIENTRY *type_glUniform2ui)(GLint, GLuint, GLuint);
typedef void		(APIENTRY *type_glUniform3ui)(GLint, GLuint, GLuint, GLuint);
typedef void		(APIENTRY *type_glUniform4ui)(GLint, GLuint, GLuint, GLuint, GLuint);
typedef void		(APIENTRY *type_glUniform1fv)(GLint, GLsizei, const GLfloat *);
typedef void		(APIENTRY *type_glUniform2fv)(GLint, GLsizei, const GLfloat *);
typedef void		(APIENTRY *type_glUniform3fv)(GLint, GLsizei, const GLfloat *);
typedef void		(APIENTRY *type_glUniform4fv)(GLint, GLsizei, const GLfloat *);
typedef void		(APIENTRY *type_glUniform1iv)(GLint, GLsizei, const GLint *);
typedef void		(APIENTRY *type_glUniform2iv)(GLint, GLsizei, const GLint *);
typedef void		(APIENTRY *type_glUniform3iv)(GLint, GLsizei, const GLint *);
typedef void		(APIENTRY *type_glUniform4iv)(GLint, GLsizei, const GLint *);
typedef void		(APIENTRY *type_glUniform1uiv)(GLint, GLsizei, const GLuint *);
typedef void		(APIENTRY *type_glUniform2uiv)(GLint, GLsizei, const GLuint *);
typedef void		(APIENTRY *type_glUniform3uiv)(GLint, GLsizei, const GLuint *);
typedef void		(APIENTRY *type_glUniform4uiv)(GLint, GLsizei, const GLuint *);
typedef void		(APIENTRY *type_glUniformMatrix2fv)(GLint, GLsizei, GLboolean, const GLfloat *);
typedef void		(APIENTRY *type_glUniformMatrix3fv)(GLint, GLsizei, GLboolean, const GLfloat *);
typedef void		(APIENTRY *type_glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat *);
typedef void		(APIENTRY *type_glGenTextures)(GLsizei, GLuint *);
typedef void		(APIENTRY *type_glBindTexture)(GLenum, GLuint);
typedef void		(APIENTRY *type_glDeleteTextures)(GLsizei, GLuint *);
typedef void		(APIENTRY *type_glTexParameteri)(GLenum, GLenum, GLint);
typedef void		(APIENTRY *type_glTexParameterf)(GLenum, GLenum, GLfloat);
typedef void		(APIENTRY *type_glTexParameteriv)(GLenum, GLenum, const GLint *params);
typedef void		(APIENTRY *type_glTexParameterfv)(GLenum, GLenum, const GLfloat *params);
typedef void		(APIENTRY *type_glTexImage2D)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *);
typedef void		(APIENTRY *type_glActiveTexture)(GLenum);
typedef void		(APIENTRY *type_glGenerateMipmap)(GLenum);
typedef void		(APIENTRY *type_glGetShaderiv)(GLuint, GLenum, GLint *);
typedef void		(APIENTRY *type_glGetShaderInfoLog)(GLuint, GLsizei, GLsizei *, GLchar *);

typedef void 		(APIENTRY *type_glBlendEquation)(GLenum mode);
typedef void 		(APIENTRY *type_glBlendFunc)(GLenum sfactor, GLenum dfactor);
typedef void 		(APIENTRY *type_glBlendFuncSeparate)(GLenum sfactorRGB, GLenum dfactorRGB, GLenum sfactorAlpha, GLenum dfactorAlpha);
typedef void 		(APIENTRY *type_glBlendEquationSeparate)(GLenum modeRGB, GLenum modeAlpha);
typedef GLboolean 	(APIENTRY *type_glIsEnabled)(GLenum cap);

struct gl_functions
{
	type_glGetIntegerv		glGetIntegerv;
	type_glGetString		glGetString;
	type_glGetTexParameterfv	glGetTexParameterfv;
	type_glGetTexParameteriv        glGetTexParameteriv;
	type_glViewport			glViewport;
	type_glClearColor		glClearColor;
	type_glClear			glClear;
	type_glEnable			glEnable;
	type_glDisable			glDisable;
	type_glCullFace			glCullFace;
	type_glFrontFace		glFrontFace;
	type_glDebugMessageCallback	glDebugMessageCallback;
	type_glGenBuffers		glGenBuffers;
	type_glBindBuffer		glBindBuffer;
	type_glBufferData		glBufferData;
	type_glBufferSubData		glBufferSubData;
	type_glDeleteBuffers		glDeleteBuffers;
	type_glGenVertexArrays		glGenVertexArrays;
	type_glBindVertexArray		glBindVertexArray;
	type_glDeleteVertexArrays	glDeleteVertexArrays;
	type_glEnableVertexAttribArray	glEnableVertexAttribArray;
	type_glDisableVertexAttribArray	glDisableVertexAttribArray;
	type_glVertexAttribPointer	glVertexAttribPointer;
	type_glVertexAttribIPointer	glVertexAttribIPointer;
	type_glVertexAttribDivisor	glVertexAttribDivisor;
	type_glCreateShader		glCreateShader;
	type_glShaderSource		glShaderSource;
	type_glCompileShader		glCompileShader;
	type_glAttachShader		glAttachShader;
	type_glDetachShader		glDetachShader;
	type_glDeleteShader		glDeleteShader;
	type_glCreateProgram		glCreateProgram;
	type_glLinkProgram		glLinkProgram;
	type_glUseProgram		glUseProgram;
	type_glDeleteProgram		glDeleteProgram;
	type_glGetProgramiv		glGetProgramiv;
	type_glGetProgramInfoLog	glGetProgramInfoLog;
	type_glDrawArrays		glDrawArrays;
	type_glDrawElements		glDrawElements;
	type_glGetUniformLocation	glGetUniformLocation;
	type_glUniform1f		glUniform1f;
	type_glUniform2f		glUniform2f;
	type_glUniform3f		glUniform3f;
	type_glUniform4f		glUniform4f;
	type_glUniform1i		glUniform1i;
	type_glUniform2i		glUniform2i;
	type_glUniform3i		glUniform3i;
	type_glUniform4i		glUniform4i;
	type_glUniform1ui		glUniform1ui;
	type_glUniform2ui		glUniform2ui;
	type_glUniform3ui		glUniform3ui;
	type_glUniform4ui		glUniform4ui;
	type_glUniform1fv		glUniform1fv;
	type_glUniform2fv		glUniform2fv;
	type_glUniform3fv		glUniform3fv;
	type_glUniform4fv		glUniform4fv;
	type_glUniform1iv		glUniform1iv;
	type_glUniform2iv		glUniform2iv;
	type_glUniform3iv		glUniform3iv;
	type_glUniform4iv		glUniform4iv;
	type_glUniform1uiv		glUniform1uiv;
	type_glUniform2uiv		glUniform2uiv;
	type_glUniform3uiv		glUniform3uiv;
	type_glUniform4uiv		glUniform4uiv;
	type_glUniformMatrix2fv		glUniformMatrix2fv;
	type_glUniformMatrix3fv		glUniformMatrix3fv;
	type_glUniformMatrix4fv		glUniformMatrix4fv;
	type_glGenTextures		glGenTextures;
	type_glBindTexture		glBindTexture;
	type_glDeleteTextures		glDeleteTextures;
	type_glTexParameteri		glTexParameteri;
	type_glTexParameterf		glTexParameterf;
	type_glTexParameteriv		glTexParameteriv;
	type_glTexParameterfv		glTexParameterfv;
	type_glTexImage2D		glTexImage2D;
	type_glActiveTexture		glActiveTexture;
	type_glGenerateMipmap		glGenerateMipmap;
	type_glGetShaderiv		glGetShaderiv;
	type_glGetShaderInfoLog		glGetShaderInfoLog;
	type_glBlendEquation		glBlendEquation;
	type_glBlendFunc		glBlendFunc;
	type_glBlendFuncSeparate	glBlendFuncSeparate;
	type_glBlendEquationSeparate 	glBlendEquationSeparate;
	type_glIsEnabled 		glIsEnabled;
	type_glDrawArraysInstanced	glDrawArraysInstanced;
	type_glDrawElementsInstanced    glDrawElementsInstanced;
};

/* init gl function pointers (for single context on some windows implementations) */
extern void 	(*GlFunctionsInit)(struct gl_functions *func);

#ifdef __cplusplus
}
#endif

 
#endif

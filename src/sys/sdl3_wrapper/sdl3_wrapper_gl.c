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

#include "sdl3_wrapper_local.h"
#include "ds_gl.h"

#ifdef GL_DEBUG

void GLAPIENTRY GlDebugMessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *user_param)
{
	char *src_str = "";
	switch (source)
	{
		case GL_DEBUG_SOURCE_API:		src_str = "API"; break;
		case GL_DEBUG_SOURCE_WINDOW_SYSTEM:	src_str = "Window System"; break;
		case GL_DEBUG_SOURCE_SHADER_COMPILER:	src_str = "Shader Compiler"; break;
		case GL_DEBUG_SOURCE_THIRD_PARTY:	src_str = "Third Party"; break;
		case GL_DEBUG_SOURCE_APPLICATION:	src_str = "Application"; break;
		case GL_DEBUG_SOURCE_OTHER:		src_str = "Other"; break;
	}

	char *type_str = "";
	switch (type)
	{
		case GL_DEBUG_TYPE_ERROR: 		type_str = "Error"; break;
		case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:	type_str = "Deprecated Behavior"; break;
		case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: 	type_str = "Undefined Behavior"; break;
		case GL_DEBUG_TYPE_PORTABILITY:        	type_str = "Portability"; break;
		case GL_DEBUG_TYPE_PERFORMANCE:        	type_str = "Performance"; break;
		case GL_DEBUG_TYPE_MARKER: 	       	type_str = "Marker"; break;
		case GL_DEBUG_TYPE_PUSH_GROUP: 	       	type_str = "Push Group"; break;
		case GL_DEBUG_TYPE_POP_GROUP: 	       	type_str = "Pop Group"; break;
		case GL_DEBUG_TYPE_OTHER: 	       	type_str = "Other"; break;
	}

	char *severity_str = "";
	u32 sev = 0;
	switch (severity)
	{
		//case GL_DEBUG_SEVERITY_NOTIFICATION:	severity_str = "Severity : Notification"; break;
		case GL_DEBUG_SEVERITY_NOTIFICATION:	{ sev = S_SUCCESS; severity_str = "Severity : Notification"; } break;
		case GL_DEBUG_SEVERITY_LOW:	    	{ sev = S_ERROR; severity_str = "Severity : Low"; } break;
		case GL_DEBUG_SEVERITY_MEDIUM:	    	{ sev = S_ERROR; severity_str = "Severity : Medium"; } break;
		case GL_DEBUG_SEVERITY_HIGH:	    	{ sev = S_ERROR; severity_str = "Severity : High"; } break;
	}

	LOG_MESSAGE(T_RENDERER, sev, 0, "opengl debug message [%s, %s, %s ] - %s\n", src_str, type_str, severity_str, message);
}

void GlDebugInit(void)
{
	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(&GlDebugMessageCallback, NULL);
}

#endif

SDL_FunctionPointer LoadProc(const char *proc)
{
	SDL_FunctionPointer fp = SDL_GL_GetProcAddress(proc); 				
	if (fp == NULL) 									
	{											
		Log(T_SYSTEM, S_ERROR, 0, "Failed to load %s: %s\n", proc, SDL_GetError());	
	}
	return fp;
}

/* NOTE: some platforms may not return NULL on invalid functions pointers; so checking
for NULL is not a reliable way to check for errors. Since we are targeting 
a subset of opengl 3.3 (webgl 2), perhaps we can assume that it is supported...
We should check the GL version of the context and verify that the loaded functions
are supported. 
 */
#define	LOAD_PROC(gl_fp)	(type_ ## gl_fp) LoadProc(#gl_fp) 								
void sdl3_GlFunctionsInit(struct gl_Functions *func)
{	
	func->glGetIntegerv = LOAD_PROC(glGetIntegerv);
	func->glGetString = LOAD_PROC(glGetString);
	func->glGetTexParameterfv = LOAD_PROC(glGetTexParameterfv);
	func->glGetTexParameteriv = LOAD_PROC(glGetTexParameteriv);
#ifdef GL_DEBUG
	func->glDebugMessageCallback = LOAD_PROC(glDebugMessageCallback);
#endif
	func->glGenBuffers = LOAD_PROC(glGenBuffers);
	func->glBindBuffer = LOAD_PROC(glBindBuffer);
	func->glBufferData = LOAD_PROC(glBufferData);
	func->glBufferSubData = LOAD_PROC(glBufferSubData);
	func->glDeleteBuffers = LOAD_PROC(glDeleteBuffers);
	func->glDrawElements = LOAD_PROC(glDrawElements);
	func->glDrawArrays = LOAD_PROC(glDrawArrays);
	func->glDrawArraysInstanced = LOAD_PROC(glDrawArraysInstanced);
	func->glDrawElementsInstanced  = LOAD_PROC(glDrawElementsInstanced);
	func->glGenVertexArrays = LOAD_PROC(glGenVertexArrays);
	func->glBindVertexArray = LOAD_PROC(glBindVertexArray);
	func->glDeleteVertexArrays = LOAD_PROC(glDeleteVertexArrays);
	func->glEnableVertexAttribArray = LOAD_PROC(glEnableVertexAttribArray);
	func->glDisableVertexAttribArray = LOAD_PROC(glDisableVertexAttribArray);
	func->glVertexAttribPointer = LOAD_PROC(glVertexAttribPointer);
	func->glVertexAttribIPointer = LOAD_PROC(glVertexAttribIPointer);
	func->glVertexAttribDivisor = LOAD_PROC(glVertexAttribDivisor);
	func->glCreateShader = LOAD_PROC(glCreateShader);
	func->glShaderSource = LOAD_PROC(glShaderSource);
	func->glCompileShader = LOAD_PROC(glCompileShader);
	func->glAttachShader = LOAD_PROC(glAttachShader);
	func->glDetachShader = LOAD_PROC(glDetachShader);
	func->glDeleteShader = LOAD_PROC(glDeleteShader);
	func->glCreateProgram = LOAD_PROC(glCreateProgram);
	func->glLinkProgram = LOAD_PROC(glLinkProgram);
	func->glUseProgram = LOAD_PROC(glUseProgram);
	func->glDeleteProgram = LOAD_PROC(glDeleteProgram);
	func->glClearColor = LOAD_PROC(glClearColor);
	func->glClear = LOAD_PROC(glClear);
	func->glEnable = LOAD_PROC(glEnable);
	func->glDisable = LOAD_PROC(glDisable);
	func->glGetUniformLocation = LOAD_PROC(glGetUniformLocation);
	func->glUniform1f = LOAD_PROC(glUniform1f);
	func->glUniform2f = LOAD_PROC(glUniform2f);
	func->glUniform3f = LOAD_PROC(glUniform3f);
	func->glUniform4f = LOAD_PROC(glUniform4f);
	func->glUniform1i = LOAD_PROC(glUniform1i);
	func->glUniform2i = LOAD_PROC(glUniform2i);
	func->glUniform3i = LOAD_PROC(glUniform3i);
	func->glUniform4i = LOAD_PROC(glUniform4i);
	func->glUniform1ui = LOAD_PROC(glUniform1ui);
	func->glUniform2ui = LOAD_PROC(glUniform2ui);
	func->glUniform3ui = LOAD_PROC(glUniform3ui);
	func->glUniform4ui = LOAD_PROC(glUniform4ui);
	func->glUniform1fv = LOAD_PROC(glUniform1fv);
	func->glUniform2fv = LOAD_PROC(glUniform2fv);
	func->glUniform3fv = LOAD_PROC(glUniform3fv);
	func->glUniform4fv = LOAD_PROC(glUniform4fv);
	func->glUniform1iv = LOAD_PROC(glUniform1iv);
	func->glUniform2iv = LOAD_PROC(glUniform2iv);
	func->glUniform3iv = LOAD_PROC(glUniform3iv);
	func->glUniform4iv = LOAD_PROC(glUniform4iv);
	func->glUniform1uiv = LOAD_PROC(glUniform1uiv);
	func->glUniform2uiv = LOAD_PROC(glUniform2uiv);
	func->glUniform3uiv = LOAD_PROC(glUniform3uiv);
	func->glUniform4uiv = LOAD_PROC(glUniform4uiv);
	func->glUniformMatrix2fv = LOAD_PROC(glUniformMatrix2fv);
	func->glUniformMatrix3fv = LOAD_PROC(glUniformMatrix3fv);
	func->glUniformMatrix4fv = LOAD_PROC(glUniformMatrix4fv);
	func->glGenTextures = LOAD_PROC(glGenTextures);
	func->glBindTexture = LOAD_PROC(glBindTexture);
	func->glDeleteTextures = LOAD_PROC(glDeleteTextures);
	func->glTexParameteri = LOAD_PROC(glTexParameteri);
	func->glTexParameterf = LOAD_PROC(glTexParameterf);
	func->glTexParameteriv = LOAD_PROC(glTexParameteriv);
	func->glTexParameterfv = LOAD_PROC(glTexParameterfv);
	func->glTexImage2D = LOAD_PROC(glTexImage2D);
	func->glActiveTexture = LOAD_PROC(glActiveTexture);
	func->glGenerateMipmap = LOAD_PROC(glGenerateMipmap);
	func->glViewport = LOAD_PROC(glViewport);
	func->glGetShaderiv = LOAD_PROC(glGetShaderiv);
	func->glGetShaderInfoLog = LOAD_PROC(glGetShaderInfoLog);
	func->glCullFace = LOAD_PROC(glCullFace);
	func->glFrontFace = LOAD_PROC(glFrontFace);
	func->glBlendEquation =  LOAD_PROC(glBlendEquation);
	func->glBlendFunc = LOAD_PROC(glBlendFunc);
	func->glBlendFuncSeparate = LOAD_PROC(glBlendFuncSeparate);
	func->glBlendEquationSeparate = LOAD_PROC(glBlendEquationSeparate);
	func->glIsEnabled = LOAD_PROC(glIsEnabled);
	func->glGetProgramiv = LOAD_PROC(glGetProgramiv);
	func->glGetProgramInfoLog = LOAD_PROC(glGetProgramInfoLog);
#if GL_DEBUG
	GlDebugInit();
#endif
}

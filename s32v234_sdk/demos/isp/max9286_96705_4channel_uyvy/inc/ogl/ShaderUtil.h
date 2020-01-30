#ifndef _SHADER_UTIL_H__
#define _SHADER_UTIL_H__
#include "GLES2/gl2.h"
int loadShaders(const char * vShaderFName, const char * pShaderFName, GLuint & shaderProgram);
int compileShader(const char * FName,GLuint ShaderNum);
#endif // _SHADER_UTIL_H__

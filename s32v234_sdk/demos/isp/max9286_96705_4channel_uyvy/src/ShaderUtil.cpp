#include "ShaderUtil.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char* loadFileToStr(const char* filename)
{
	FILE* fptr = fopen(filename, "r");
	char* shaderSource = NULL;
    if (fptr == NULL)
    {
        fprintf(stderr,"couldn't find file: %s\n", filename);
		goto End;
    }
    int length;
    fseek(fptr, 0, SEEK_END);
    length = ftell(fptr);
    fseek(fptr, 0 ,SEEK_SET);

    shaderSource = (char*)malloc(sizeof (char) * (length + 1));
	memset(shaderSource, 0, sizeof(char) * (length + 1));

    if (shaderSource == NULL)
    {
        fprintf(stderr, "Out of memory.\n");
		goto End;
    }
	if (!fread(shaderSource, 1, length, fptr))
	{
		goto End;
	}
End:
	fclose(fptr);
	return shaderSource;
}


int loadShaders(const char * vShaderFName, const char * pShaderFName, GLuint & shaderProgram)
{
    GLuint vertShaderNum = glCreateShader(GL_VERTEX_SHADER);
    GLuint pixelShaderNum = glCreateShader(GL_FRAGMENT_SHADER);
	if (vertShaderNum == 0 || pixelShaderNum == 0)
	{
		fprintf(stderr, "create shader failed: %d\n", glGetError());
	}
    if (compileShader(vShaderFName, vertShaderNum) != 0)
    {
        fprintf(stderr,"vshader compile failed: %s\n", vShaderFName);
        return -1;
    }

    if (compileShader(pShaderFName, pixelShaderNum) != 0)
    {
        fprintf(stderr,"fshader compile failed: %s\n", pShaderFName);
        return -2;
    }
    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram , vertShaderNum);
    glAttachShader(shaderProgram , pixelShaderNum);

    glLinkProgram(shaderProgram );
    // Check if linking succeeded.
    GLint linked = false;
    glGetProgramiv(shaderProgram , GL_LINK_STATUS, &linked);
    if (!linked)
    {
        // Retrieve error buffer size.
        GLint errorBufSize, errorLength;
        glGetShaderiv(shaderProgram , GL_INFO_LOG_LENGTH, &errorBufSize);
        char * infoLog = (char*)malloc(errorBufSize * sizeof (char) + 1);
        if (!infoLog)
        {
            // Retrieve error.
            glGetProgramInfoLog(shaderProgram , errorBufSize, &errorLength, infoLog);
            infoLog[errorBufSize + 1] = '\0';
            fprintf(stderr, "%s", infoLog);
            free(infoLog);
        }
        fprintf(stderr,"shader program link failed\n");
        return -3;
    }
    glDeleteShader(vertShaderNum);
    glDeleteShader(pixelShaderNum);
    return (shaderProgram == 0);
}
int compileShader(const char * FName, GLuint ShaderNum)
{
    FILE * fptr = NULL;
    fptr = fopen(FName, "rb");
    if (fptr == NULL)
    {
        fprintf(stderr,"find vshader error date::0808\n");
        return -1;
    }
    int length;
    fseek(fptr, 0, SEEK_END);
    length = ftell(fptr);
    fseek(fptr, 0 ,SEEK_SET);

    char * shaderSource = (char*)malloc(sizeof (char) * length);
    if (shaderSource == NULL)
    {
        fprintf(stderr, "Out of memory.\n");
        return -2;
    }

    size_t successful_read = fread(shaderSource, length, 1, fptr);
    if (!successful_read)
    {
        fprintf(stderr,"failed to read shader source file\n");
        return -3;
    }
	//printf("Source: %s", shaderSource);
    glShaderSource(ShaderNum, 1, (const char**)&shaderSource, &length);
    glCompileShader(ShaderNum);

    free(shaderSource);
    fclose(fptr);

    GLint compiled = 0;
    glGetShaderiv(ShaderNum, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        GLint errorBufSize, errorLength;
		errorBufSize = errorLength = 0;
        glGetShaderiv(ShaderNum, GL_INFO_LOG_LENGTH, &errorBufSize);

        char * infoLog = (char*)malloc(errorBufSize * sizeof(char) + 2);
        if (infoLog)
        {
            // Retrieve error.
            glGetShaderInfoLog(ShaderNum, errorBufSize, &errorLength, infoLog);
            infoLog[errorBufSize + 1] = '\0';
			printf("info log size = %d\n", errorBufSize);
            fprintf(stderr,"Compile error: %s\n",infoLog);
            free(infoLog);
        }
        return -4;
    }
    return 0;
}

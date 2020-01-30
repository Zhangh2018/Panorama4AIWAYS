#include <GLES2/gl2.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui.hpp>


#include "CVOGLBridge.h"
#include "ShaderUtil.h"



static inline int oglType2cvType(int format, int type)
{
	int numChannels = 0;
	switch (format)
	{
	case GL_ALPHA:
	case GL_LUMINANCE:
		numChannels = 1;
		break;
	case GL_LUMINANCE_ALPHA:
		numChannels = 2;
		break;
	case GL_RGB:
		numChannels = 3;
		break;
	case GL_RGBA:
		numChannels = 4;
		break;
	default:
		break;
	}

	if (numChannels == 0 || type != GL_UNSIGNED_BYTE)
	{
		return -1;
	}
	return CV_8UC(numChannels);
}



cv::Mat fbo2Mat(GLuint fbo, int x, int y, int width, int height, int format, int type)
{
	GLint backup = 0;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &backup);

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	cv::Mat img = screen2Mat(x, y, width, height, format, type);
	glBindFramebuffer(GL_FRAMEBUFFER, backup);

	return img;
}


cv::Mat screen2Mat(int x, int y, int width, int height, int format, int type)
{
	int cvtype = oglType2cvType(format, type);
	if (cvtype == -1)
	{
		return cv::Mat{};
	}

	cv::Mat img{ cv::Size{ width, height}, cvtype};
	glReadPixels(x, y, width, height, format, type, img.data);

	return img;
}


void renderImage(
	GLuint tex,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment,
    glm::mat4 bevTransformMatrix)
{
 static bool Flag_cal = false;

 	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

//    GLuint VertexArrayID;
//    glGenVertexArrays(1, &VertexArrayID);
//    glBindVertexArray(VertexArrayID);


//  static GLuint TextureID = 0;


    static const GLfloat g_vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            -1.0f, 1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            -1.0f, 1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
    };

    static const GLfloat g_uv_buffer_data[] = {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 0.0f,

            0.0f, 1.0f,
            1.0f, 0.0f,
            1.0f, 1.0f,
    };

    static GLuint vertexbuffer;
    static GLuint uvbuffer;
    static GLuint TextureID;
    static GLuint program;
    static GLuint transformMatrixHandle;

    if(!Flag_cal)
    {

        if(loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program)!= 0)
            printf("return error : renderImage()!!!!!!!!!!!!!!!!!!!\n");
        else
            printf("return success : renderImage()!!!!!!!!!!!!!!!!!!!\n");

        TextureID  = glGetUniformLocation(program, "uTexture");

        glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

        glGenBuffers(1, &uvbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

        transformMatrixHandle = glGetUniformLocation(program, "uTransformMatrix");
//        glGenBuffers(1, &uvbuffer);
//        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//        glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    }

    Flag_cal = true;


    // Clear the screen
//    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(program);
//
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(TextureID, 3);

    glUniformMatrix4fv(transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
            0,                  // attribut1e 0. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
    );

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glVertexAttribPointer(
            1,                  // attribut1e 0. No particular reason for 0, but must match the layout in the shader.
            2,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
    );

    glDrawArrays(GL_TRIANGLES, 0, 2 * 3);

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);
/**
	// rendering image to fill the whole view
	GLfloat vertices[] = {
		-1.0f, 1.0f, 0.0f,  // Position 0
		0.0f,  0.0f,        // TexCoord 0
		-1.0f, -1.0f, 0.0f,  // Position 1
		0.0f,  1.0f,        // TexCoord 1
		1.0f, -1.0f, 0.0f,  // Position 2
		1.0f,  1.0f,        // TexCoord 2
		1.0f,  1.0f, 0.0f,  // Position 3
		1.0f,  0.0f         // TexCoord 3
	};

	GLushort indices[] =
		{ 0, 1, 2, 0, 2, 3 };

	GLuint program;
	loadShaders("shader/image.vert", "shader/image.frag", program);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	GLuint samplerLoc = glGetUniformLocation(program, "uTexture");
	GLuint posLoc = glGetAttribLocation(program, "aPosition");
	GLuint texCoordLoc = glGetAttribLocation(program, "aTexCoord");

	GLint viewport[4] = {0};
	glGetIntegerv(GL_VIEWPORT, viewport);
	printf("viewport = (%d, %d, %d, %d)\n", viewport[0], viewport[1], viewport[2], viewport[3]);

	//glViewport(x, y, viewWidth, viewHeight);
	//glClear(GL_COLOR_BUFFER_BIT);

	GLuint vbo[3] = {0};
	glGenBuffers(3, vbo);


	glUseProgram(program);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(samplerLoc, 0);

	glVertexAttribPointer(posLoc, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), vertices);
	glVertexAttribPointer(texCoordLoc, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), &vertices[3]);
	glEnableVertexAttribArray(posLoc);
	glEnableVertexAttribArray(texCoordLoc);
	glEnableVertexAttribArray(samplerLoc);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);

	glDisableVertexAttribArray(posLoc);
	glDisableVertexAttribArray(texCoordLoc);
	glDisableVertexAttribArray(samplerLoc);

	glDeleteProgram(program);
*/
}

void renderImage(
	void* data, int imgWidth, int imgHeight,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment)
{
	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

	GLuint tex;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, format, imgWidth, imgHeight, 0, format, type, data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	renderImage(tex, x, y, viewWidth, viewHeight, format, type, alignment);

	//glDeleteTextures(1, &tex);
}

void renderMat(const cv::Mat& mat, int x, int y, int width, int height, int format, int type)
{
	renderImage(mat.data, mat.size().width, mat.size().height, x, y, width, height, format, type, 1);
}

void renderImage2(
	GLuint tex,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment)
{
    static GLfloat posVertices[6*3] = {
				0.0,-1.0,0.0,
				0.0,0.0,0.0,
				1.0,0.0,0.0,
				0.0,-1.0,0.0,
				1.0,-1.0,0.0,
				1.0,0.0,0.0};

    static GLfloat texVertices[6*2] = {
				0.0,1.0,
				0.0,0.0,
				1.0,0.0,
				0.0,1.0,
				1.0,1.0,
				1.0,0.0};
    static unsigned short indices[6] = {0,1,2,3,4,5};

	GLuint program;
	loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program);

	GLuint samplerLoc = glGetUniformLocation(program, "uTexture");
	GLuint posLoc = glGetAttribLocation(program, "aPosition");
	GLuint texCoordLoc = glGetAttribLocation(program, "aTexCoord");

	GLint viewport[4] = {0};
	glGetIntegerv(GL_VIEWPORT, viewport);
	printf("viewport = (%d, %d, %d, %d)\n", viewport[0], viewport[1], viewport[2], viewport[3]);
	//glViewport(x, y, viewWidth, viewHeight);
	//glClear(GL_COLOR_BUFFER_BIT);

	GLuint m_vbo[3] = {0,0,0};
	glGenBuffers(3, m_vbo);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*3,posVertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*2,texVertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)*6, indices, GL_STATIC_DRAW);

	glUseProgram(program);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(samplerLoc, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glVertexAttribPointer(posLoc, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 3, 0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glVertexAttribPointer(texCoordLoc, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 2, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);

	glEnableVertexAttribArray(posLoc);
	glEnableVertexAttribArray(texCoordLoc);
	glEnableVertexAttribArray(samplerLoc);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(posLoc);
	glDisableVertexAttribArray(texCoordLoc);
	glDisableVertexAttribArray(samplerLoc);
}


void renderImage2(
	void* data, int imgWidth, int imgHeight,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment)
{
	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

	GLuint tex;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, format, imgWidth, imgHeight, 0, format, type, data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	renderImage2(tex, x, y, viewWidth, viewHeight, format, type, alignment);

	glDeleteTextures(1, &tex);
}

void renderMat2(const cv::Mat& mat, int x, int y, int width, int height, int format, int type)
{
	renderImage2(mat.data, mat.size().width, mat.size().height, x, y, width, height, format, type, 1);
}


void renderImage3(
	GLuint tex,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment)
{
	static GLfloat posVertices[] = {
		-1.0, -1.0, 0.0,
		-1.0, 1.0, 0.0,
		1.0, 1.0, 0.0,

		-1.0, -1.0, 0.0,
		1.0, -1.0, 0.0,
		1.0, 1.0, 0.0
	};

	static GLfloat texVertices[] = {
		0.0, 0.0,
		0.0, 1.0,
		1.0, 1.0,

		0.0, 0.0,
		1.0, 0.0,
		1.0, 1.0
	};

	GLushort indices[] = {0,1,2,3,4,5};

	GLuint program;

	loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program);
	GLuint samplerLoc = glGetUniformLocation(program, "uTexture");
	GLuint posLoc = glGetAttribLocation(program, "aPosition");
	GLuint texCoordLoc = glGetAttribLocation(program, "aTexCoord");

	GLuint vbo[3] = {0};
	glGenBuffers(3, vbo);

	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 3, posVertices, GL_STATIC_DRAW);
	//glBuffer
}


cv::Mat texture2Mat(GLuint tex, int x, int y, int width, int height, int format, int type)
{
	GLuint fbo, rbo;
	glGenFramebuffers(1, &fbo);
	glGenRenderbuffers(1, &rbo);

	GLuint dstTex;
	glGenTextures(1, &dstTex);
	glBindTexture(GL_TEXTURE_2D, dstTex);
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, type, NULL);

	glBindRenderbuffer(GL_RENDERBUFFER,  rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);

	GLint backupFBO = 0;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &backupFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTex, 0);

	auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		printf("framebuffer failed: ");
		switch (status)
		{
		case GL_FRAMEBUFFER_UNSUPPORTED:
			printf("unsupported\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
			printf("incomplete attachment\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
			printf("incomplete dimensions\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
			printf("incomplete missing attchment\n");
			break;
		default:
			printf("unknown error: %d (0x%x)\n", status, status);
			break;
		}
	}

	renderImage(tex, x, y, width, height, format, type);

	cv::Mat img{ cv::Size{width, height}, oglType2cvType(format, type) };
	glReadPixels(x, y, width, height, format, type, img.data);

	glBindFramebuffer(GL_FRAMEBUFFER, backupFBO);
	glDeleteFramebuffers(1, &fbo);
	glDeleteRenderbuffers(1, &rbo);
	glDeleteTextures(1, &dstTex);

	return img;
}

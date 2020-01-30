#pragma once

#include <GLES2/gl2.h>
#include <opencv2/core.hpp>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

extern cv::Mat fbo2Mat(GLuint fbo, int x, int y, int width, int height, int format = GL_RGBA, int type = GL_UNSIGNED_BYTE);
extern cv::Mat screen2Mat(int x, int y, int width, int height, int format = GL_RGBA, int type = GL_UNSIGNED_BYTE);
extern cv::Mat texture2Mat(GLuint tex, int x, int y, int width, int height, int format = GL_RGBA, int type = GL_UNSIGNED_BYTE);


extern void renderMat(
	const cv::Mat& mat, int x, int y, int width, int height,
	int format = GL_RGBA, int type = GL_UNSIGNED_BYTE);


extern void renderImage(
	void* data, int imgWidth, int imgHeight,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment = 1);

extern void renderImage(
	GLuint tex,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment = 1,
    glm::mat4 bevTransformMatrix = glm::mat4(1.0));

extern void renderMat2(
	const cv::Mat& mat, int x, int y, int width, int height,
	int format = GL_RGBA, int type = GL_UNSIGNED_BYTE);


extern void renderImage2(
	void* data, int imgWidth, int imgHeight,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment = 1);

extern void renderImage2(
	GLuint tex,
	int x, int y, int viewWidth, int viewHeight,
	int format, int type,
	int alignment = 1);

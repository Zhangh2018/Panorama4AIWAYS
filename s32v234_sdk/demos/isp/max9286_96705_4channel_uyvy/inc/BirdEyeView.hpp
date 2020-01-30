#pragma once

#include <memory>
#include <array>

#include <opencv2/core.hpp>
#include <GLES2/gl2.h>
#include <glm/glm.hpp>
#include "LUT.h"

class View : public std::enable_shared_from_this<View>
{
public:
	virtual ~View() = 0;
};

class NonOverlappedBEV : public virtual View
{
public:
	virtual ~NonOverlappedBEV() override;
	NonOverlappedBEV() = default;
	NonOverlappedBEV(std::shared_ptr<CalibLUT> lut);
	void draw(const GLuint* textures, const float* delta, const glm::mat4& transform = glm::mat4(1.0));

public:
	void generateVertices();

	static GLuint program;
	static GLint positionAttr;
	static GLint texCoordAttr;
	static GLint transformMatrixUnif;
	static GLint deltaUnif;
	static GLint samplerUnif;

	GLuint positionVBO[4];
	GLuint texCoordVBO[4];


	std::array<int, 4> vertexRows, vertexCols;
	std::array<int, 4> numPrimitive, numVertex;

	std::array<cv::Rect, 4> rois;

	std::shared_ptr<CalibLUT> lut;
};

class OverlappedBEV : public virtual View
{
public:
	virtual ~OverlappedBEV() override;
	OverlappedBEV() = default;
	OverlappedBEV(std::shared_ptr<CalibLUT> lut);

	void draw(const GLuint* textures, const float* delta, const glm::mat4& transform = glm::mat4(1.0));

public:
	void generateVertices();

	static GLuint program;
	static GLint positionAttr;
	static GLint texCoordAttr1, texCoordAttr2;
	static GLint weightAttr;
	static GLint transformMatrixUnif;
	static GLint deltaUnif1, deltaUnif2;
	static GLint samplerUnif1, samplerUnif2;

	GLuint positionVBO[4];
	GLuint texCoordVBO1[4], texCoordVBO2[4];
	GLuint weightVBO[4];

	std::array<int, 4> vertexRows, vertexCols;
	std::array<int, 4> numPrimitive, numVertex;

	std::shared_ptr<CalibLUT> lut;

	std::array<cv::Rect, 4> rois;

	static const int cams[4][2];
};

class BirdEyeView : public virtual View
{
public:
	virtual ~BirdEyeView() override {}

	BirdEyeView() = default;
	BirdEyeView(std::shared_ptr<CalibLUT> lut)
		: ovBEV(lut), novBEV(lut)
	{
	}

	void draw(const GLuint* textures, const float* delta, const glm::mat4& transform = glm::mat4(1.0))
	{
		novBEV.draw(textures, delta, transform);
		ovBEV.draw(textures, delta, transform);
	}
private:
	OverlappedBEV ovBEV;
	NonOverlappedBEV novBEV;
};
#pragma once

#include <memory>

#include <GLES2/gl2.h>
#include <glm/glm.hpp>
#include "LUT.h"

#include "const_value.h"

class DistanceLineView
{
public:
	DistanceLineView() = default;
	DistanceLineView( std::shared_ptr<CalibLUT> lut, const char* configPath);

	~DistanceLineView();

	void draw(int camId, const glm::mat4& transform = glm::mat4(1.0));

private:
	void generateVertices();
	void generateFrontRearVertices();
	void generateLeftRightVertices();
	void loadConfig(const char* configPath);

	static GLuint program;
	static GLint positionAttr;
	static GLint colorAttr;
	static GLint colorUnif;
	static GLint transformMatrixUnif;

	GLuint positionVBO[const_value::CAMERANUM];
	GLuint indexVBO[const_value::CAMERANUM];

	std::shared_ptr<CalibLUT> lut;

	float frontRearDistance, leftRightDistance,
		frontRearLineLength, leftRightLineLength;
	glm::vec4 lineColor;
	float lineWidth;
};

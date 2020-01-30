#include "DistanceLine.hpp"
#include "const_value.h"
#include "Quad.hpp"

#include "OGLUtil.hpp"
#include "ShaderUtil.h"

#include <glm/ext.hpp>
GLuint DistanceLineView::program;
GLint DistanceLineView::positionAttr,
	DistanceLineView::colorAttr,
	DistanceLineView::colorUnif,
	DistanceLineView::transformMatrixUnif;

DistanceLineView::DistanceLineView(
	std::shared_ptr<CalibLUT> lut,
	const char* configPath)
	: lut(lut)
{
	loadConfig(configPath);
	glGenBuffers(const_value::CAMERANUM, positionVBO);
	glGenBuffers(const_value::CAMERANUM, indexVBO);
}

DistanceLineView::~DistanceLineView()
{
	glDeleteBuffers(const_value::CAMERANUM, positionVBO);
	glDeleteBuffers(const_value::CAMERANUM, indexVBO);
}

void DistanceLineView::loadConfig(const char* configPath)
{
	FILE* fp = fopen(configPath, "r");
	if (fp == nullptr)
	{
		throw std::runtime_error("couldn't open the config of DistanceLineView");
	}
	fscanf(fp, "frontRearDistance: %f\n", &frontRearDistance);
	fscanf(fp, "leftRightDistance: %f\n", &leftRightDistance);
	fscanf(fp, "frontRearLineLength: %f\n", &frontRearLineLength);
	fscanf(fp, "leftRightLineLength: %f\n", &leftRightLineLength);
	fscanf(fp, "lineColor: [%f, %f, %f, %f]\n", &lineColor.r, &lineColor.g, &lineColor.b, &lineColor.a);
	fscanf(fp, "lineWidth: %f\n", &lineWidth);

	// 几个距离长度读入的都是以毫米计数，需要转换为像素计数
	frontRearDistance /= lut->header.mm_per_pixel_h;
	leftRightDistance /= lut->header.mm_per_pixel_w;
	frontRearLineLength /= lut->header.mm_per_pixel_w;
	leftRightLineLength /= lut->header.mm_per_pixel_h;

	leftRightDistance *= 2;
	leftRightLineLength *= 2;

	fclose(fp);
}

void worldCoord2ImageCoord(std::shared_ptr<CalibLUT> lut, float& x, float& y)
{
	x /= lut->header.mm_per_pixel_w;
	y /= lut->header.mm_per_pixel_h;
}

void DistanceLineView::generateFrontRearVertices()
{
	const auto& carRect = lut->header.car_Icon_Rect;
	auto bevWidth = lut->header.bev_img_width;
	auto bevHeight = lut->header.bev_img_height;

	Quad front{
		// 距离线和车框居中对齐
		carRect.x + (1.0f * carRect.width / 2.0f) - (frontRearLineLength / 2.0f),

		// 矩形左上角，减去 lineWidth 使得矩形下边与车框距离刚好为 frontRearDistance
		carRect.y - frontRearDistance - lineWidth,
		frontRearLineLength,
		lineWidth
	};

	auto frontPositions = front.vertex();
	normalize2DVertices(frontPositions, bevWidth, bevHeight);
	glBindBuffer(GL_ARRAY_BUFFER, positionVBO[const_value::FRONT_CAM]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * frontPositions.size(),frontPositions.data(), GL_STATIC_DRAW);

	Quad rear{
		carRect.x + (1.0f * carRect.width / 2.0f) - (frontRearLineLength / 2.0f),
		// 车尾的左上角刚好跟车框的距离为 frontRearDistance
		carRect.y + frontRearDistance,
		frontRearLineLength,
		lineWidth
	};

	auto rearPositions = rear.vertex();
	normalize2DVertices(rearPositions, bevWidth, bevHeight);
	glBindBuffer(GL_ARRAY_BUFFER, positionVBO[const_value::REAR_CAM]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * rearPositions.size(), rearPositions.data(), GL_STATIC_DRAW);

	std::vector<GLushort> indices = { 0, 1, 2, 0, 2, 3 };
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[const_value::FRONT_CAM]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);

	indexVBO[const_value::REAR_CAM] = indexVBO[const_value::FRONT_CAM];
}

void DistanceLineView::generateLeftRightVertices()
{
	float svWidth = 1110;
	float svHeight = 810;

	Quad left{
		svWidth - leftRightDistance - lineWidth,
		200,
		lineWidth,
		leftRightLineLength
	};
	Quad leftL{
		svWidth - leftRightDistance,
		200,
		80,
		lineWidth
	};
	auto leftPositions = left.vertex();
	auto leftLPositions = leftL.vertex();
	std::copy(leftLPositions.begin(), leftLPositions.end(), std::back_inserter(leftPositions));
	normalize2DVertices(leftPositions, svWidth, svHeight);
	glBindBuffer(GL_ARRAY_BUFFER, positionVBO[const_value::LEFT_CAM]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * leftPositions.size(), leftPositions.data(), GL_STATIC_DRAW);

	Quad right {
		leftRightDistance,
		200,
		lineWidth,
		leftRightLineLength
	};
	Quad rightL{
		leftRightDistance - 80,
		200,
		80,
		lineWidth
	};
	auto rightPositions = right.vertex();
	auto rightLPositions = rightL.vertex();
	std::copy(rightLPositions.begin(), rightLPositions.end(), std::back_inserter(rightPositions));

	normalize2DVertices(rightPositions, svWidth, svHeight);
	glBindBuffer(GL_ARRAY_BUFFER, positionVBO[const_value::RIGHT_CAM]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * rightPositions.size(), rightPositions.data(), GL_STATIC_DRAW);

	std::vector<GLushort> indices = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[const_value::LEFT_CAM]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);

	indexVBO[const_value::RIGHT_CAM] = indexVBO[const_value::LEFT_CAM];
}

void DistanceLineView::generateVertices()
{
	generateFrontRearVertices();
	generateLeftRightVertices();
}

void DistanceLineView::draw(int camId, const glm::mat4& transform)
{
	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("shader/line.vert", "shader/line.frag", program) != 0)
		{
			throw std::runtime_error("couldn't load line.vert/line.frag");
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		colorAttr = glGetAttribLocation(program, "aColor");
		colorUnif = glGetUniformLocation(program, "uColor");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
		generateVertices();
		isProgramLoad = true;
	}

	glUseProgram(program);
	glUniform4fv(colorUnif, 1, glm::value_ptr(lineColor));
	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO[camId]);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(positionAttr);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[camId]);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	int vertexCount = (camId < 2 ? 6 : 12);
	glDrawElements(GL_TRIANGLES, vertexCount, GL_UNSIGNED_SHORT, nullptr);

	glDisable(GL_BLEND);
	glDisableVertexAttribArray(positionAttr);
}

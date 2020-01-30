#pragma once

#include <opencv2/core.hpp>
#include <GLES2/gl2.h>
#include <glm/glm.hpp>

class PlaneExpandView
{
public:
	PlaneExpandView() = default;
	PlaneExpandView(int camId, const char* configPath, const char* vehicleConfigPath);

	void draw(GLuint texture, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

private:
	void loadConfig(const char* configPath);
	void loadVehicleConfig(const char* configPath);
	void generateLUT();
	void generateFrontRearLUT();
	void generateLeftRightLUT();
	void generateVertices();

	int camId;
	cv::Mat_<cv::Point> lut;
	cv::Mat_<cv::Point> world2PEV;
	int srcWidth, srcHeight;
	int resultWidth, resultHeight;
	float translateX, translateY;
	float fovH, fovV;

	float worldWidth, worldHeight;
	float carBodyWidth;
	float vehicleWidth;

	static GLuint program;
	static GLint positionAttr, texCoordAttr;
	static GLint transformMatrixUnif, samplerUnif;

	int vertexCount;
	GLuint positionVBO, texCoordVBO;

	static int vertexRows;
	static int vertexCols;
};

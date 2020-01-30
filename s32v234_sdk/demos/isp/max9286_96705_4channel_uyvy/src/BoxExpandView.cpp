#include <iostream>
#include <cstdio>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <GLES2/gl2.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <fstream>
#include <iomanip>
#include "BoxExpandView.hpp"
#include <numeric>

#include "toml11/toml.hpp"

#include "camera_model.h"

#include "ShaderUtil.h"

GLuint BoxExpandView::program;
GLint BoxExpandView::positionAttr;
GLint BoxExpandView::texCoordAttr;
GLint BoxExpandView::transformMatrixUnif;
GLint BoxExpandView::samplerUnif;

BoxExpandView::BoxExpandView(int camId, const char* configPath)
	: camId(camId)
{
	loadConfig(configPath);
	srcWidth = Camera_Model[camId]->camera_model_int.width;
	srcHeight = Camera_Model[camId]->camera_model_int.height;
}

BoxExpandView::~BoxExpandView()
{
	glDeleteBuffers(1, &positionVBO);
	glDeleteBuffers(1, &texCoordVBO);
}

cv::Mat_<cv::Point2i>  BoxExpandView::generateBoxExpandLUT()
{
	float isFrontCam = (camId == 0 ? 1 : -1);

	float focalLen = 1000;  // 虚拟焦距，怎么取值我也不知道

	float boxFoldAngle = (fovHCenter + fovHSide) * 0.5;
	float boxSideWidth = (resultWidth - boxCenterWidth) * 0.5;

	// 计算各方向上每像素代表的长度值
	float mm_per_pixel_h_center = focalLen * std::tan(fovHCenter * 0.5) * 2 / boxCenterWidth;
	float mm_per_pixel_h_side = focalLen / std::cos(fovHCenter * 0.5) * std::sin(fovHSide * 0.5) * 2 / boxSideWidth;
	float mm_per_pixel_v = focalLen * std::tan(fovV * 0.5) * 2 / resultHeight;

	cv::Vec3d invPose;
	for (int i = 0; i < 3; ++i)
	{
		invPose[i] = Camera_Model[camId]->camera_model_ext.inv_pose[i + 3];
	}
	cv::Matx33d R(Camera_Model[camId]->camera_model_ext.R);

	//相机在世界坐标系中的位置
	cv::Vec3d pose = R * invPose;
	pose *= -1;

	float initWorldCoordLeftX = pose[0] - isFrontCam
	                                       * (mm_per_pixel_h_center * boxCenterWidth * 0.5
	                                          + mm_per_pixel_h_side * boxSideWidth * std::cos(boxFoldAngle));
	float initWorldCoordLeftY = pose[1] + isFrontCam * (focalLen
	                                                     - mm_per_pixel_h_side * boxSideWidth * std::sin(boxFoldAngle));
	float initWorldCoordLeftZ = pose[2] + mm_per_pixel_v
	                                       * resultHeight * 0.5 + translationY;

	float initWorldCoordCenterX = pose[0] - isFrontCam
	                                         * mm_per_pixel_h_center * boxCenterWidth * 0.5;
	float initWorldCoordCenterY = pose[1] + isFrontCam * focalLen;
	float initWorldCoordCenterZ = pose[2]
	                               + mm_per_pixel_v * resultHeight * 0.5 + translationY;

	float initWorldCoordRightX = pose[0]
	                              + isFrontCam * mm_per_pixel_h_center * boxCenterWidth * 0.5;
	float initWorldCoordRightY = pose[1] + isFrontCam * focalLen;
	float initWorldCoordRightZ = pose[2] + mm_per_pixel_v * resultHeight * 0.5 + translationY;

	cv::Mat_<cv::Point2i> lut(cv::Size(resultWidth, resultHeight));

	for (int i = 0; i < resultHeight; i++)
	{
		for (int j = 0; j < resultWidth; j++)
		{
			double worldCoord[3];
			if (j < boxSideWidth)
			{
				worldCoord[0] = initWorldCoordLeftX
				                + isFrontCam * j * mm_per_pixel_h_side * std::cos(boxFoldAngle);
				worldCoord[1] = initWorldCoordLeftY
				                + isFrontCam * j * mm_per_pixel_h_side * std::sin(boxFoldAngle);
				worldCoord[2] = initWorldCoordLeftZ
				                - i * mm_per_pixel_v;
			}
			else if (j <= boxSideWidth + boxCenterWidth)
			{
				worldCoord[0] = initWorldCoordCenterX
				                + isFrontCam * (j - boxSideWidth) * mm_per_pixel_h_center;
				worldCoord[1] = initWorldCoordCenterY;
				worldCoord[2] = initWorldCoordCenterZ - i * mm_per_pixel_v;
			}
			else
			{
				worldCoord[0] = initWorldCoordRightX + isFrontCam
				                                       * (j - boxSideWidth - boxCenterWidth)
				                                       * mm_per_pixel_h_side * std::cos(boxFoldAngle);
				worldCoord[1] = initWorldCoordRightY - isFrontCam
				                                       * (j - boxSideWidth - boxCenterWidth)
				                                       * mm_per_pixel_h_side * std::sin(boxFoldAngle);
				worldCoord[2] = initWorldCoordRightZ - i * mm_per_pixel_v;
			}
			double camRay[3];
			World_Ray_To_Cam_Ray(camRay, worldCoord, &Camera_Model[camId]->camera_model_ext);

			double imagePoint[3];
			Cam_Ray_To_Image_Point(imagePoint, camRay, &Camera_Model[camId]->camera_model_int);

			lut(i, j) = cv::Point2i(imagePoint[1], imagePoint[0]);
		}
	}
	return lut;
}


void BoxExpandView::generateVertices(const cv::Mat_<cv::Point2i>& lut)
{
	int vertexRow = 50;
	int vertexCol = 50;
	vertexCount = vertexRow * vertexCol * 6;

	std::vector<GLfloat> positions(vertexCount * 2);
	std::vector<GLfloat> texCoords(vertexCount * 2);
	std::vector<GLushort> indices(vertexCount);

	float posStartX = -1.0, posStartY = 1.0;   // 左上角
	float posStepX = 2.0 / vertexCol,
		posStepY = 2.0 / vertexRow;
	float posOffset[][2] = {
		{0,        0},
		{posStepX, 0},
		{posStepX, -posStepY},
		{0,        -posStepY},
		{posStepX, -posStepY},
		{0,        0}
	};

	float texStepX = 1.0 * resultHeight / vertexRow,
		texStepY = 1.0 * resultWidth / vertexCol;
	float texOffset[][2] = {
		{0,            0},
		{0,            texStepY - 1},
		{texStepX - 1, texStepY - 1},
		{texStepX - 1, 0},
		{texStepX - 1, texStepY - 1},
		{0,            0}
	};

	for (int i = 0; i < vertexRow; ++i)
	{
		float posY = posStartY - i * posStepY;
		float texY = i * texStepX;

		for (int j = 0; j < vertexCol; ++j)
		{
			float posX = posStartX + j * posStepX;
			float texX = j * texStepY;

			// 一个矩形用2个三角形表示，共6个顶点
			for (int k = 0; k < 6; ++k)
			{
				int idx = ((i * vertexCol + j) * 6 + k) * 2;
				positions[idx] = posX + posOffset[k][0];
				positions[idx + 1] = posY + posOffset[k][1];

				int lutY = texY + texOffset[k][0];
				int lutX = texX + texOffset[k][1];
				auto p = lut(lutY, lutX);

				texCoords[idx] = 1.0 * p.x / srcWidth;
				texCoords[idx + 1] = 1.0 * p.y / srcHeight;
			}
		}
	}

	glGenBuffers(1, &positionVBO);
	glGenBuffers(1, &texCoordVBO);

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);
}


void BoxExpandView::draw(GLuint textures, int x, int y, int width, int height, const glm::mat4& transform)
{
	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/image.vert", "/data/opengl_new/shader/image.frag", program) != 0)
		{
			throw std::runtime_error("couldn't load image.vert/image.frag");
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		texCoordAttr = glGetAttribLocation(program, "aTexCoord");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
		samplerUnif = glGetUniformLocation(program, "uTexture");
		isProgramLoad = true;
	}

	if (!isVerticesGenerated)
	{
		auto lut = generateBoxExpandLUT();
		generateVertices(lut);
		isVerticesGenerated = true;
	}

	glUseProgram(program);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures);
	glUniform1i(samplerUnif, 0);

	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0)));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(positionAttr);

	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glVertexAttribPointer(texCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(texCoordAttr);

	glDrawArrays(GL_TRIANGLES, 0, vertexCount);

	glDisableVertexAttribArray(positionAttr);
	glDisableVertexAttribArray(texCoordAttr);
}


void BoxExpandView::loadConfig(const char* path)
{
	const auto config = toml::parse(path);
	const toml::value& table = toml::find<toml::table>(config, "box_expand");
	resultWidth = toml::find<int>(table, "result_image_width");
	resultHeight = toml::find<int>(table, "result_image_height");
	boxCenterWidth = toml::find<int>(table, "box_center_width");
	translationY = toml::find<float>(table, "translate_y");
	fovHCenter = toml::find<float>(table, "fov_center_h");
	fovHSide = toml::find<float>(table, "fov_side_h");
	fovV = toml::find<float>(table, "fov_v");

	fovHCenter = fovHCenter * M_PI / 180.0;
	fovHSide = fovHSide * M_PI / 180.0;
	fovV = fovV * M_PI / 180.0;
}

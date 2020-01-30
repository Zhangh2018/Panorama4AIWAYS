#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <glm/ext.hpp>

#include "toml11/toml.hpp"
#include "camera_model.h"
#include "PlaneExpandView.hpp"
#include "CoordSystem.hpp"

#include "ShaderUtil.h"

#include "utils/utils.hpp"
#include "const_value.h"

int PlaneExpandView::vertexRows = 100,
	PlaneExpandView::vertexCols = 100;
GLuint PlaneExpandView::program = 0;
GLint PlaneExpandView::positionAttr = -1,
	PlaneExpandView::texCoordAttr = -1,
	PlaneExpandView::transformMatrixUnif = -1,
	PlaneExpandView::samplerUnif = -1;

PlaneExpandView::PlaneExpandView(int camId, const char* configPath, const char* vehicleConfigPath)
	: camId(camId), srcWidth(1280), srcHeight(720)
{
	loadConfig(configPath);
	loadVehicleConfig(vehicleConfigPath);
	generateLUT();
	generateVertices();
}

void PlaneExpandView::loadConfig(const char* configPath)
{
	const auto config = toml::parse(configPath);
	const toml::value resultImage = toml::find<toml::table>(config, "result_image");
	resultWidth = toml::find<int>(resultImage, "width");
	resultHeight = toml::find<int>(resultImage, "height");

	const toml::value planeExpand = toml::find<toml::table>(config, "plane_expand");
	translateY = toml::find<float>(planeExpand, "translate_y");
	switch (camId)
	{
	case const_value::FRONT_CAM:
	case const_value::REAR_CAM:
		translateX = toml::find<float>(planeExpand, "translate_x");
		fovH = toml::find<float>(planeExpand, "fov_h");
		fovV = toml::find<float>(planeExpand, "fov_v");

		fovH = glm::radians(fovH);
		fovV = glm::radians(fovV);
		break;
	case const_value::LEFT_CAM:
	case const_value::RIGHT_CAM:
		worldWidth = toml::find<float>(planeExpand, "world_width");
		worldHeight = toml::find<float>(planeExpand, "world_height");
		carBodyWidth = toml::find<float>(planeExpand, "car_body_width");
		break;
	default:
		break;
	}
}

void PlaneExpandView::loadVehicleConfig(const char* configPath)
{
	const auto config = toml::parse(configPath);
	vehicleWidth = toml::find<float>(config, "vehicle_width");
}

void PlaneExpandView::generateLUT()
{
	switch (camId)
	{
	case const_value::FRONT_CAM:
	case const_value::REAR_CAM:
		generateFrontRearLUT();
		break;
	case const_value::LEFT_CAM:
	case const_value::RIGHT_CAM:
		generateLeftRightLUT();
		break;
	default:
		break;
	}
}

void PlaneExpandView::generateFrontRearLUT()
{
	float focalLength = 1000;
	float mmPerPixelH = focalLength * std::tan(fovH * 0.5) * 2 / resultWidth;
	float mmPerPixelV = focalLength * std::tan(fovV * 0.5) * 2 / resultHeight;

	cv::Vec3d invPose;
	for (int i = 0; i < 3; ++i)
	{
		invPose[i] = Camera_Model[camId]->camera_model_ext.inv_pose[i + 3];
	}
	cv::Matx33d R(Camera_Model[camId]->camera_model_ext.R);
	cv::Vec3d pose = R * invPose;
	pose *= -1;

	float pitch = (camId == 0 ? (2 * M_PI) : (3 * M_PI)) / 180.0;
	float is_front_cam = (camId == 0 ? 1 : -1);

	float init_world_coord_x = pose[0] - is_front_cam
	                                     * mmPerPixelH * resultWidth * 0.5 + translateX;
	float init_world_coord_y = pose[1] + is_front_cam
	                                     * (focalLength * cos(pitch)
	                                        + mmPerPixelV * resultHeight * 0.5 * sin(pitch));
	float init_world_coord_z = pose[2] - focalLength * sin(pitch)
	                           + mmPerPixelV * resultHeight * 0.5 * cos(pitch) + translateY;

	float stepX = mmPerPixelH;
	float stepY = mmPerPixelV * sin(pitch);
	float stepZ = mmPerPixelV * cos(pitch);

	lut = cv::Mat_<cv::Point>::zeros(cv::Size(resultWidth, resultHeight));
	for (int y = 0; y < resultHeight; ++y)
	{
		for (int x = 0; x < resultWidth; ++x)
		{
			double world[3] = {0};
			world[0] = init_world_coord_x + is_front_cam * x * stepX;
			world[1] = init_world_coord_y - is_front_cam * y * stepY;
			world[2] = init_world_coord_z - y * stepZ;

			double imagePoint[2] = {0};
			World_Ray_To_Image_Point(imagePoint, world, Camera_Model[camId]);
			lut(y, x) = cv::Point(imagePoint[1], imagePoint[0]);
		}
	}
}

void PlaneExpandView::generateLeftRightLUT()
{
	float dx = worldWidth / resultWidth, dy = worldHeight / resultHeight;
	float world_init_x, world_init_y;

	if (camId == const_value::LEFT_CAM)
	{
		world_init_x = -(worldWidth + (vehicleWidth * 0.5 - carBodyWidth * dx));
		world_init_y = worldHeight * 0.5 - translateY;
	}
	if (camId == const_value::RIGHT_CAM)
	{
		world_init_x = vehicleWidth * 0.5 - carBodyWidth * dx;
		world_init_y = worldHeight * 0.5 - translateY;
	}

	lut = cv::Mat_<cv::Point>::zeros(cv::Size(resultWidth, resultHeight));
	for (int y = 0; y < resultHeight; ++y)
	{
		for (int x = 0; x < resultWidth; ++x)
		{
			double world_coord[3] = {
				world_init_x + x * dx,
				world_init_y - y * dy,
				0
			};
			double image_point[2] = {0};
			World_Ray_To_Image_Point(image_point, world_coord, Camera_Model[camId]);
			lut(y, x) = cv::Point(image_point[1], image_point[0]);
		}
	}
}

void PlaneExpandView::generateVertices()
{
	glGenBuffers(1, &positionVBO);
	glGenBuffers(1, &texCoordVBO);
	vertexCount = vertexRows * vertexCols * 6;

	std::vector<GLfloat> positions(vertexCount * 2);
	std::vector<GLfloat> texCoords(vertexCount * 2);

	float stepX = 1.0 * resultWidth / vertexCols,
		stepY = 1.0 * resultHeight / vertexRows;

	float offset[][2] = {
		{0, 0},
		{stepX, 0},
		{stepX, stepY},
		{0, stepY},
		{stepX, stepY},
		{0, 0}
	};

#ifdef PANORAMA_DUMP_VERTEX
	std::ofstream ofs{ "pe_vertex.txt" };
#endif

	for (int i = 0; i < vertexRows; ++i)
	{
		float y = i * stepY;
		for (int j = 0; j < vertexCols; ++j)
		{
			float x = j * stepX;
			for (int k = 0; k < 6; ++k)
			{
				int idx = (i * vertexCols + j) * 12 + k * 2;

				int lutX = x + offset[k][0];
				int lutY = y + offset[k][1];
				lutX = clamp<int>(lutX, 0, resultWidth - 1);
				lutY = clamp<int>(lutY, 0, resultHeight - 1);
				positions[idx] = x + offset[k][0];
				positions[idx + 1] = y + offset[k][1];

				std::tie(positions[idx], positions[idx + 1]) = cv2GLNDCoord<float>(
					x + offset[k][0],
					y + offset[k][1],
					resultWidth,
					resultHeight);

				const auto& coord = lut(lutY, lutX);
				texCoords[idx] = 1.0 * coord.x / srcWidth;
				texCoords[idx + 1] = 1.0 * coord.y / srcHeight;
#ifdef PANORAMA_DUMP_VERTEX
				ofs << std::fixed << std::setprecision(6) 
					<< positions[idx] << '\t'
					<< positions[idx + 1] << '\t'
					<< texCoords[idx] << '\t'
					<< texCoords[idx + 1] << std::endl;
#endif
			}
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);
}

void PlaneExpandView::draw(GLuint texture, int x, int y, int width, int height, const glm::mat4& transform)
{
	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/single_view_ng.vert", "/data/opengl_new/shader/single_view_ng.frag", program) == 0)
		{
			positionAttr = glGetAttribLocation(program, "aPosition");
			texCoordAttr = glGetAttribLocation(program, "aTexCoord");

			transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
			samplerUnif = glGetUniformLocation(program, "uTexture");
			isProgramLoad = true;
		}
	}

	glUseProgram(program);

	glBindTexture(GL_TEXTURE_2D, texture);
	glActiveTexture(GL_TEXTURE0);
	glUniform1i(samplerUnif, 0);

	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	
	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glVertexAttribPointer(texCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

	glEnableVertexAttribArray(positionAttr);
	glEnableVertexAttribArray(texCoordAttr);

	glDrawArrays(GL_TRIANGLES, 0, vertexCount);

	glDisableVertexAttribArray(positionAttr);
	glDisableVertexAttribArray(texCoordAttr);
}
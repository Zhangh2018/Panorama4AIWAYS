#include "BirdEyeView.hpp"

#include <iomanip>
#include <opencv2/highgui.hpp>
#include "const_value.h"
#include <glm/ext.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <iostream>

#include <ShaderUtil.h>

using namespace panorama;

GLuint NonOverlappedBEV::program;
GLint NonOverlappedBEV::positionAttr;
GLint NonOverlappedBEV::texCoordAttr;
GLint NonOverlappedBEV::transformMatrixUnif;
GLint NonOverlappedBEV::deltaUnif;
GLint NonOverlappedBEV::samplerUnif;

template<class T>
inline T clamp(T v, T min, T max)
{
	return (v < min ? min : (v > max ? max : v));
}

inline std::tuple<GLfloat, GLfloat> normalize2DVertex(float x, float y, float width, float height)
{
	GLfloat gx = x / (width / 2.0) - 1.0;
	GLfloat gy = 1.0 - y / (height / 2.0);
	return std::tuple<GLfloat, GLfloat>(gx, gy);
}

inline void normalize2DVertices(std::vector<GLfloat>& positions, float width, float height)
{
	for (int i = 0; i < positions.size() / 2; ++i)
	{
		auto& x = positions[i * 2];
		auto& y = positions[i * 2 + 1];

		std::tie(x, y) = normalize2DVertex(x, y, width, height);
	}
}

inline std::tuple<GLfloat, GLfloat> normalizeTexCoord(float x, float y, float width, float height)
{
	return std::tuple<GLfloat, GLfloat>(x / width, 1.0 - y / height);
}

inline void normalizeTexCoords(std::vector<GLfloat>& texCoords, float width, float height)
{
	for (int i = 0; i < texCoords.size() / 2; ++i)
	{
		auto& x = texCoords[i * 2];
		auto& y = texCoords[i * 2 + 1];
		std::tie(x, y) = normalizeTexCoord(x, y, width, height);
	}
}


View::~View()
{
}

NonOverlappedBEV::~NonOverlappedBEV()
{
	glDeleteBuffers(4, positionVBO);
	glDeleteBuffers(4, texCoordVBO);
}

NonOverlappedBEV::NonOverlappedBEV(std::shared_ptr<CalibLUT> lut)
	: lut(lut)
{
	const cv::Rect& carRect = lut->header.car_Icon_Rect;

	int frontX = carRect.x;
	int frontY = 0;
	int frontWidth = carRect.width;
	int frontHeight = carRect.y;

	int rearX = frontX;
	int rearY = carRect.br().y + 1;
	int rearWidth = carRect.width;
	int rearHeight = lut->header.bev_img_height - carRect.br().y - 2;

	int leftX = 0;
	int leftY = carRect.y;
	int leftWidth = carRect.x;
	int leftHeight = carRect.height + 1;

	int rightX = carRect.br().x + 1;
	int rightY = leftY;
	int rightWidth = lut->header.bev_img_width - carRect.br().x - 2;
	int rightHeight = carRect.height + 1;

	rois = {
		cv::Rect(frontX, frontY, frontWidth, frontHeight),
		cv::Rect(rearX, rearY, rearWidth, rearHeight),
		cv::Rect(leftX, leftY, leftWidth, leftHeight),
		cv::Rect(rightX, rightY, rightWidth, rightHeight)
	};



	vertexRows = { 20, 20, 60, 60 };
	vertexCols = { 30, 30, 20, 20 };
}

void NonOverlappedBEV::generateVertices()
{
	int bevWidth = lut->header.bev_img_width;
	int bevHeight = lut->header.bev_img_height;
	int srcWidth = lut->header.src_img_width;
	int srcHeight = lut->header.src_img_height;

	glGenBuffers(4, positionVBO);
	glGenBuffers(4, texCoordVBO);

	for (int camId = 0; camId < 4; ++camId)
	{
		// 当前相机视图在 2D 拼接图上对应的 ROI
		auto roi = rois[camId];

		float vertexRow = vertexRows[camId],
			vertexCol = vertexCols[camId];

		// 将视图划分为 vertexRow * vertexCol 个矩形，每个矩形再划分为 2 个三角形
		int primitiveCount = vertexRow * vertexCol * 2;

		// 每个三角形三个顶点
		int vertexCount = primitiveCount * 3;

		numPrimitive[camId] = primitiveCount;
		numVertex[camId] = vertexCount;

		std::vector<GLfloat> positions(vertexCount * 2);
		std::vector<GLfloat> texCoords(vertexCount * 2);

		// setup step and offset for position vertex
		float posStartX = roi.x, posStartY = roi.y;
		float posStepX = 1.0 * roi.width / vertexCol,
			posStepY = 1.0 * roi.height / vertexRow;

		float posOffset[][2] = {
			{ 0, 0 },
			{ posStepX, 0 },
			{ posStepX, posStepY },
			{ 0, posStepY },
			{ posStepX, posStepY },
			{ 0, 0 }
		};


		//setup step and offset for texcoord
		float texStepX = 1.0 * roi.height / vertexRow,
			texStepY = 1.0 * roi.width / vertexCol;
		float texOffset[][2] = {
			{0,            0},
			{0,            texStepY - 1},
			{texStepX - 1, texStepY - 1},
			{texStepX - 1, 0},
			{texStepX - 1, texStepY - 1},
			{0,            0}
		};

		auto lutROI = lut->operator()(roi);

		cv::Mat tex = cv::Mat::zeros(cv::Size(roi.width, roi.height), CV_8UC1);
		cv::Mat pos = cv::Mat::zeros(cv::Size(bevWidth, bevHeight), CV_8UC1);
		std::string filename[]= {"front", "rear", "left", "right"};
		std::ofstream lutTxt(filename[camId] + "_pos.txt");

		for (int i = 0; i < vertexRow; ++i)
		{
			float posY = posStartY + i * posStepY;
			float texY =  i * texStepX;

			for (int j = 0; j < vertexCol; ++j)
			{
				float posX = posStartX + j * posStepX;
				float texX =  j * texStepY;

				for (int k = 0; k < 6; ++k)
				{
					int idx = (i * vertexCol + j) * 12 + k * 2;
					positions[idx] = posX + posOffset[k][0];
					positions[idx + 1] = posY + posOffset[k][1];


					if (positions[idx] >= bevWidth)
					{
						std::cout << camId << " x overflow" << std::endl;
					}
					if (positions[idx + 1] >= bevHeight)
					{
						std::cout << camId << " y overflow" << std::endl;
					}

					pos.at<uint8_t>(positions[idx + 1], positions[idx]) = 255;

					int lutX = texX + texOffset[k][1];
					int lutY = texY + texOffset[k][0];
					auto p = lutROI(lutY, lutX);
					TacPoint tp = p.point_pos1;

					tex.at<uint8_t>(lutY, lutX) = 255;
					texCoords[idx] = 1.0 * tp.x / srcWidth;
					texCoords[idx + 1] = 1.0 * tp.y / srcHeight;
				}
			}
		}
//		cv::imshow(std::string("tex") + std::to_string(camId), tex);
//		cv::imshow(std::string("pos") + std::to_string(camId), pos);
//		cv::waitKey(0);

		normalize2DVertices(positions, bevWidth, bevHeight);
		//normalizeTexCoords(texCoords, srcWidth, srcHeight);

		glBindBuffer(GL_ARRAY_BUFFER, positionVBO[camId]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO[camId]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);
	}
}


void NonOverlappedBEV::draw(const GLuint* textures, const float* delta, const glm::mat4& transform)
{
	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/unmerge2d.vert", "/data/opengl_new/shader/unmerge2d.frag", program) != 0)
		{
			throw std::runtime_error("couldn't not load unmerge2d.vert/unmerge2d.frag");
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		texCoordAttr = glGetAttribLocation(program, "aTexCoord");
		deltaUnif = glGetUniformLocation(program, "uDelta");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
		samplerUnif = glGetUniformLocation(program, "my_Sampler");

		generateVertices();
		isProgramLoad = true;
	}

	for (int camId = 0; camId < 4; ++camId)
	{
		glUseProgram(program);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textures[camId]);
		glUniform1i(samplerUnif, 0);

		glUniform1f(deltaUnif, delta[camId]);
		glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));

		glBindBuffer(GL_ARRAY_BUFFER, positionVBO[camId]);
		glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO[camId]);
		glVertexAttribPointer(texCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

		glEnableVertexAttribArray(positionAttr);
		glEnableVertexAttribArray(texCoordAttr);

		glDrawArrays(GL_TRIANGLES, 0, numVertex[camId]);

		glDisableVertexAttribArray(positionAttr);
		glDisableVertexAttribArray(texCoordAttr);
	}
}


//
GLuint OverlappedBEV::program;
GLint OverlappedBEV::positionAttr;
GLint OverlappedBEV::texCoordAttr1, OverlappedBEV::texCoordAttr2;
GLint OverlappedBEV::weightAttr;
GLint OverlappedBEV::transformMatrixUnif;
GLint OverlappedBEV::deltaUnif1, OverlappedBEV::deltaUnif2;
GLint OverlappedBEV::samplerUnif1, OverlappedBEV::samplerUnif2;

const int OverlappedBEV::cams[4][2] = {
	{ const_value::FRONT_CAM, const_value::LEFT_CAM },
	{ const_value::FRONT_CAM, const_value::RIGHT_CAM },
	{ const_value::REAR_CAM, const_value::LEFT_CAM },
	{ const_value::REAR_CAM, const_value::RIGHT_CAM }
};
OverlappedBEV::~OverlappedBEV()
{
	glDeleteBuffers(4, positionVBO);
	glDeleteBuffers(4, texCoordVBO1);
	glDeleteBuffers(4, texCoordVBO2);
	glDeleteBuffers(4, weightVBO);
}

OverlappedBEV::OverlappedBEV(std::shared_ptr<CalibLUT> lut)
	: lut(lut)
{
	const cv::Rect& carRect = lut->header.car_Icon_Rect;
	auto bevWidth = lut->header.bev_img_width;
	auto bevHeight = lut->header.bev_img_height;

	int flX = 0;
	int flY = 0;
	int flWidth = carRect.x;
	int flHeight = carRect.y;

	int frX = carRect.br().x;
	int frY = 0;
	int frWidth = bevWidth - carRect.br().x - 1;
	int frHeight = carRect.y;

	int rlX = 0;
	int rlY = carRect.br().y + 1;
	int rlWidth = carRect.x;
	int rlHeight = bevHeight - carRect.br().y - 2;

	int rrX = frX;
	int rrY = rlY;
	int rrWidth = frWidth;
	int rrHeight = rlHeight;

	rois = {
		cv::Rect(flX, flY, flWidth, flHeight),
		cv::Rect(frX, frY, frWidth, frHeight),
		cv::Rect(rlX, rlY, rlWidth, rlHeight),
		cv::Rect(rrX, rrY, rrWidth, rrHeight)
	};

	vertexRows = {100, 100, 100, 100 };
	vertexCols = {100, 100, 100, 100 };
}

void OverlappedBEV::generateVertices()
{
	int bevWidth = lut->header.bev_img_width,
		bevHeight = lut->header.bev_img_height;
	int srcWidth = lut->header.src_img_width,
		srcHeight = lut->header.src_img_height;

	glGenBuffers(4, positionVBO);
	glGenBuffers(4, texCoordVBO1);
	glGenBuffers(4, texCoordVBO2);
	glGenBuffers(4, weightVBO);

	for (int r = 0; r < const_value::CAMERANUM; ++r)
	{
		// 当前相机视图在 2D 拼接图上对应的 ROI
		auto roi = rois[r];

		float vertexRow = vertexRows[r],
			vertexCol = vertexCols[r];

		// 将视图划分为 vertexRow * vertexCol 个矩形，每个矩形再划分为 2 个三角形
		int primitiveCount = vertexRow * vertexCol * 2;

		// 每个三角形三个顶点
		int vertexCount = primitiveCount * 3;

		numPrimitive[r] = primitiveCount;
		numVertex[r] = vertexCount;

		std::vector<GLfloat> positions(vertexCount * 2);
		std::vector<GLfloat> texCoords1(vertexCount * 2);
		std::vector<GLfloat> texCoords2(vertexCount * 2);
		std::vector<GLfloat> weights(vertexCount);

		// setup step and offset for position vertex
		float posStartX = roi.x, posStartY = roi.y;
		float posStepX = 1.0 * roi.width / vertexCol,
			posStepY = 1.0 * roi.height / vertexRow;

		float posOffset[][2] = {
			{ 0, 0 },
			{ posStepX, 0 },
			{ posStepX, posStepY },
			{ 0, posStepY },
			{ posStepX, posStepY },
			{ 0, 0 }
		};


		//setup step and offset for texcoord
		float texStepX = 1.0 * roi.height / vertexRow,
			texStepY = 1.0 * roi.width / vertexCol;
		float texOffset[][2] = {
			{0,            0},
			{0,            texStepY - 1},
			{texStepX - 1, texStepY - 1},
			{texStepX - 1, 0},
			{texStepX - 1, texStepY - 1},
			{0,            0}
		};

		auto lutROI = lut->operator()(roi);

		cv::Mat pos = cv::Mat::zeros(cv::Size(bevWidth, bevHeight), CV_8UC1);
		cv::Mat wg = cv::Mat::zeros(cv::Size(bevWidth, bevHeight), CV_8UC1);
		cv::Mat tex1 = cv::Mat::zeros(cv::Size(srcWidth, srcHeight), CV_8UC1);
		cv::Mat tex2 = cv::Mat::zeros(cv::Size(srcWidth, srcHeight), CV_8UC1);

#ifdef PANORAMA_DUMP_VERTEX
		std::ofstream vertexDump{ std::string("vertex") + std::to_string(r) + std::string(".txt") };
		std::ofstream posDump{ std::string("pos") + std::to_string(r) + std::string(".txt") };
#endif

		for (int i = 0; i < vertexRow; ++i)
		{
			float posY = posStartY + i * posStepY;
			float texY = i * texStepX;

			for (int j = 0; j < vertexCol; ++j)
			{
				float posX = posStartX + j * posStepX;
				float texX = j * texStepY;

				for (int k = 0; k < 6; ++k)
				{
					int idx = (i * vertexCol + j) * 12 + k * 2;
//					positions[idx] = clamp<float>(posX + posOffset[k][0], 0, bevWidth);
//					positions[idx + 1] = clamp<float>(posY + posOffset[k][1], 0, bevWidth);
					positions[idx] = posX + posOffset[k][0];
					positions[idx + 1] = posY + posOffset[k][1];

#ifdef PANORAMA_DUMP_VERTEX
					if (positions[idx] >= bevWidth)
					{
						std::cout << r << " x overflow" << std::endl;
					}
					if (positions[idx + 1] >= bevHeight)
					{
						std::cout << r << " y overflow" << std::endl;
					}
#endif

					pos.at<uint8_t>(positions[idx + 1], positions[idx]) = 255;

					int lutX = texX + texOffset[k][1];
					int lutY = texY + texOffset[k][0];
					auto p = lutROI(lutY, lutX);

					texCoords1[idx] = 1.0 * p.point_pos1.x / srcWidth;
					texCoords1[idx + 1] = 1.0 * p.point_pos1.y / srcHeight;
					tex1.at<uint8_t>(p.point_pos1.y, p.point_pos1.x) = 255;

					texCoords2[idx] = 1.0 * p.point_pos2.x / srcWidth;
					texCoords2[idx + 1] = 1.0 * p.point_pos2.y / srcHeight;
					tex2.at<uint8_t>(p.point_pos2.y, p.point_pos2.x) = 255;

					weights[idx / 2] = 1.0 * p.wt_fusion / 255.0;

#if 0
					vertexDump << std::fixed << std::setprecision(6) << positions[idx] << "\t"
						<< positions[idx + 1] << "\t"
						<< (float)p.point_pos1.x << "\t"
						<< (float)p.point_pos1.y << "\t"
						<< (float)p.point_pos2.x << "\t"
						<< (float)p.point_pos2.y << "\t"
						<< weights[idx] << std::endl;
#endif
					wg.at<uint8_t>(positions[idx + 1], positions[idx]) = p.wt_fusion;
				}
			}
		}

//		cv::imshow(std::string("pos") + std::to_string(r), pos);
//		cv::imshow(std::string("tex1") + std::to_string(r), tex1);
//		cv::imshow(std::string("tex2") + std::to_string(r), tex2);
//		cv::imshow(std::string("weight") + std::to_string(r), wg);
//		cv::waitKey(0);
		//cv::imwrite(std::string("pos") + std::to_string(r) + ".png", pos);

		normalize2DVertices(positions, bevWidth, bevHeight);

#ifdef PANORAMA_DUMP_VERTEX
		for (int i = 0; i < positions.size(); i += 2)
		{
			posDump << std::fixed << std::setprecision(6) << positions[i] << "\t" << positions[i + 1] << std::endl;
		}
#endif
		glBindBuffer(GL_ARRAY_BUFFER, positionVBO[r]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO1[r]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords1.size(), texCoords1.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO2[r]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * texCoords2.size(), texCoords2.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, weightVBO[r]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * weights.size(), weights.data(), GL_STATIC_DRAW);
	}
}

void OverlappedBEV::draw(const GLuint* textures, const float* delta, const glm::mat4& transform)
{
	static bool isProgramLoad = false;
	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/merge2d.vert", "/data/opengl_new/shader/merge2d.frag", program) != 0)
		{
			throw std::runtime_error("couldn't load merge2d.vert/merge2d.frag");
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		texCoordAttr1 = glGetAttribLocation(program, "aTexCoord1");
		texCoordAttr2 = glGetAttribLocation(program, "aTexCoord2");
		weightAttr = glGetAttribLocation(program, "aWeight");

		samplerUnif1 = glGetUniformLocation(program, "uTexture1");
		samplerUnif2 = glGetUniformLocation(program, "uTexture2");
		deltaUnif1 = glGetUniformLocation(program, "uDelta1");
		deltaUnif2 = glGetUniformLocation(program, "uDelta2");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
		generateVertices();
		isProgramLoad = true;
	}

	for (int i = 0; i < const_value::CAMERANUM; ++i)
	{
		glUseProgram(program);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textures[ cams[i][0] ]);
		glUniform1i(samplerUnif1, 0);

		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, textures[ cams[i][1] ]);
		glUniform1i(samplerUnif2, 1);

		glUniform1f(deltaUnif1, delta[ cams[i][0] ]);
		glUniform1f(deltaUnif2, delta[ cams[i][1] ]);
		glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));

		glBindBuffer(GL_ARRAY_BUFFER, positionVBO[i]);
		glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO1[i]);
		glVertexAttribPointer(texCoordAttr1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO2[i]);
		glVertexAttribPointer(texCoordAttr2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

		glBindBuffer(GL_ARRAY_BUFFER, weightVBO[i]);
		glVertexAttribPointer(weightAttr, 1, GL_FLOAT, GL_FALSE, 0, nullptr);

		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO[i]);

		glEnableVertexAttribArray(positionAttr);
		glEnableVertexAttribArray(texCoordAttr1);
		glEnableVertexAttribArray(texCoordAttr2);
		glEnableVertexAttribArray(weightAttr);

		glDrawArrays(GL_TRIANGLES, 0, numVertex[i]);
		//glDrawElements(GL_TRIANGLES, numVertex[i], GL_UNSIGNED_SHORT, nullptr);

		glDisableVertexAttribArray(positionAttr);
		glDisableVertexAttribArray(texCoordAttr1);
		glDisableVertexAttribArray(texCoordAttr2);
		glDisableVertexAttribArray(weightAttr);
	}
}
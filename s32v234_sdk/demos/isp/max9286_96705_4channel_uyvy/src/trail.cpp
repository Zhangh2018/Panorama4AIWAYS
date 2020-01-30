#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <iterator>
#include <numeric>

#define _USE_MATH_DEFINES
#include <cmath>

#include <GLES2/gl2.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <ogl/ShaderUtil.h>

#include "ShaderUtil.h"

constexpr float svWidth = 1110;
constexpr float svHeight = 1080;

constexpr float bevWidth = 810;
constexpr float bevHeight = 1080;

constexpr float winWidth = 1920;
constexpr float winHeight = 1080;

constexpr float carX = 311;
constexpr float carY = 291;
constexpr float carWidth = 188;
constexpr float carHeight = 508;

constexpr float carBrX = carX + carWidth;
constexpr float carBrY = carY + carHeight;

constexpr float carCX = 405;
constexpr float carCY = 540;

constexpr int numSegments = 1000;

constexpr float minAngle = M_PI / 180.0 * -31.0;
constexpr float maxAngle = M_PI / 180.0 * 31.0;

double perspective[3][3] = {
	{ 1.386558080884066, -1.87507964238002, -7.181025497483866 },
	{ -2.074729277268261e-15, -0.4402219710204447, 237.8039872572193 },
	{ -3.767602549387128e-18, -0.003391223923543303, 1 }
};

template<class T>
static inline bool isNegative(T v)
{
	return std::signbit(v);
}

template<class T>
static inline bool isPositive(T v)
{
	return !isNegative(v);
}


template<class T>
T clamp(T v, T min, T max)
{
	return (v < min ? min : (v > max ? max : v));
}

static inline std::tuple<GLfloat, GLfloat> normalize2DVertex(float x, float y, float width, float height)
{
	GLfloat gx = x / (width / 2.0) - 1.0;
	GLfloat gy = 1.0 - y / (height / 2.0);
	return std::tuple<GLfloat, GLfloat>(gx, gy);
}

static inline void normalize2DVertices(std::vector<float>& positions, float width, float height)
{
	for (int i = 0; i < positions.size() / 2; ++i)
	{
		auto& x = positions[i * 2];
		auto& y = positions[i * 2 + 1];

		std::tie(x, y) = normalize2DVertex(x, y, width, height);
	}
}


static inline void applyPerspectiveTransform(std::vector<GLfloat>& pos)
{
	double m[3][3] = {
		{ 1.386558080884066, -1.87507964238002, -7.181025497483866 },
		{ -2.074729277268261e-15, -0.4402219710204447, 237.8039872572193 },
		{ -3.767602549387128e-18, -0.003391223923543303, 1 }
	};
	for (int i = 0; i < pos.size() / 2; ++i)
	{
		auto& x = pos[i * 2];
		auto& y = pos[i * 2 + 1];
		double tx = (m[0][0] * x + m[0][1] * y + m[0][2]);
		double ty = (m[1][0] * x + m[1][1] * y + m[1][2]);

		double t = m[2][0] * x + m[2][1] * y + m[2][2];

		x = tx / t;
		y = ty / t;
	}
}

void genericDraw(const std::vector<GLfloat>& positions, const std::vector<GLushort>& indices, GLenum primitiveType)
{
	static GLuint program = 0;
	static GLint positionAttr = 0;
	static GLint transformMatrixUnif = 0;
	static GLint perspectiveMatrixUnif = 0;
	static bool isProgramLoad = false;

	if (!isProgramLoad)
	{
		if (loadShaders("/data/opengl_new/shader/line.vert", "/data/opengl_new/shader/line.frag", program) != 0)
		{
			return;
		}
		positionAttr = glGetAttribLocation(program, "aPosition");
		transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
		perspectiveMatrixUnif = glGetUniformLocation(program, "uPersepctiveMatrix");

		isProgramLoad = true;
	}

	GLuint positionVBO = 0;
	GLuint indexVBO = 0;

	glGenBuffers(1, &positionVBO);
	glGenBuffers(1, &indexVBO);

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * positions.size(), positions.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);

	glUseProgram(program);
	glm::mat4 transformMatrix = glm::mat4(1.0);
	glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transformMatrix));

	glm::mat4 perspectiveMatrix = glm::mat4(1.0);
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			perspectiveMatrix[j][i] = perspective[i][j];
		}
	}
	glUniformMatrix3fv(perspectiveMatrixUnif, 1, GL_FALSE, glm::value_ptr(perspectiveMatrix));

	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexVBO);

	glEnableVertexAttribArray(positionAttr);

	glDrawElements(primitiveType, indices.size(), GL_UNSIGNED_SHORT, 0);

	glDisableVertexAttribArray(positionAttr);

	glDeleteBuffers(1, &positionVBO);
	glDeleteBuffers(1, &indexVBO);
}


void drawPolyline(const std::vector<float>& points, float viewWidth, float viewHeight)
{
	std::vector<GLfloat> positions;
	std::vector<GLushort> indices;

	positions.resize(points.size());
	indices.resize(points.size() / 2);

	std::copy(points.cbegin(), points.cend(), std::back_inserter(positions));
	std::iota(indices.begin(), indices.end(), 0); // [0, 1, 2, ..., N]

	normalize2DVertices(positions, viewWidth, viewHeight);

	genericDraw(positions, indices, GL_LINE_STRIP);
}

void drawRect(float x, float y, float width, float height, float viewWidth, float viewHeight)
{
	std::vector<GLfloat> positions(8, 0.0);
	std::vector<GLushort> indices(4, 0);

	positions[0] = x;
	positions[1] = y;

	positions[2] = x + width;
	positions[3] = y;

	positions[4] = x + width;
	positions[5] = y + height;

	positions[6] = x;
	positions[7] = y + height;

	std::iota(indices.begin(), indices.end(), 0);

	normalize2DVertices(positions, viewWidth, viewHeight);

	genericDraw(positions, indices, GL_LINE_LOOP);
}

void generateCircle(int segments, float cx, float cy, float r, std::vector<GLfloat>& positions, std::vector<GLushort>& indices)
{
	if (positions.size() < segments * 2)
	{
		positions.resize(segments * 2);
	}
	if (indices.size() < segments)
	{
		indices.resize(segments);
	}

	float theta = 2 * M_PI / (float)segments;

	float x = r;
	float y = 0;

	float cosTheta = std::cos(theta);
	float sinTheta = std::sin(theta);

	for (int i = 0; i < segments; ++i)
	{
		positions[i * 2] = x + cx;
		positions[i * 2 + 1] = y + cy;
		indices[i] = i;

		float nextX = cosTheta * x - sinTheta * y;
		float nextY = sinTheta * x + cosTheta * y;

		x = nextX;
		y = nextY;
	}
}

void drawCircle(float cx, float cy, float r, float viewWidth, float viewHeight)
{
	std::vector<GLfloat> positions;
	std::vector<GLushort> indices;

	generateCircle(numSegments, cx, cy, r, positions, indices);
	normalize2DVertices(positions, viewWidth, viewHeight);

	genericDraw(positions, indices, GL_LINE_LOOP);
}

void generateArc(int segments, float cx, float cy, float r, float startAngle, float arcAngle,
				 std::vector<GLfloat>& positions, std::vector<GLushort>& indices)
{

	if (positions.size() < segments * 2)
	{
		positions.resize(segments * 2);
	}
	if (indices.size() < segments)
	{
		indices.resize(segments);
	}

	float theta = arcAngle / float(segments - 1);
	float tangetial_factor = tanf(theta);
	float radial_factor = cosf(theta);

	float x = r * cosf(startAngle);
	float y = r * sinf(startAngle);

	for(int i = 0; i < segments; ++i)
	{
		positions[i * 2] = x + cx;
		positions[i * 2 + 1] = y + cy;
		indices[i] = i;

		float tx = -y;
		float ty = x;

		x += tx * tangetial_factor;
		y += ty * tangetial_factor;

		x *= radial_factor;
		y *= radial_factor;
	}
}


void drawArc(float cx, float cy, float r, float startAngle, float arcAngle, float viewWidth, float viewHeight,
			 bool applyTransform = false)
{
	std::vector<GLfloat> positions;
	std::vector<GLushort> indices;

	float startAngleBKP = startAngle;

	if (isPositive(startAngleBKP))
	{
		startAngle *= -1;
		arcAngle *= -1;
	}

	generateArc(numSegments, cx, cy, r, startAngle, arcAngle, positions, indices);

	if (applyTransform)
	{
		applyPerspectiveTransform(positions);
	}

	normalize2DVertices(positions, viewWidth, viewHeight);

	if (isPositive(startAngleBKP))
	{
		for (int i = 0; i < numSegments; ++i)
		{
			positions[i * 2] = -positions[i * 2];
		}
	}

	genericDraw(positions, indices, GL_LINE_STRIP);
}

void generateArc2(int segments, float cx, float cy, float r, float startAngle, float arcAngle, float width,
				  std::vector<GLfloat>& positions, std::vector<GLushort>& indices)
{
	std::vector<GLfloat> outerPos, innerPos;
	std::vector<GLushort> outerIdx, innerIdx;

	float outerR = r;
	float innerR = r - width;

	generateArc(segments, cx, cy, outerR, startAngle, arcAngle, outerPos, outerIdx);
	generateArc(segments, cx, cy, innerR, startAngle, arcAngle, innerPos, innerIdx);

	positions.resize(segments * 4);
	indices.resize(segments * 2);
	for (int i = 0; i < numSegments; ++i)
	{
		positions[i * 4] = outerPos[i * 2];
		positions[i * 4 + 1] = outerPos[i * 2 + 1];
		positions[i * 4 + 2] = innerPos[i * 2];
		positions[i * 4 + 3] = innerPos[i * 2 + 1];

		indices[i * 2] = i * 2;
		indices[i * 2 + 1] = i * 2 + 1;
	}
}

void drawArc2(float cx, float cy, float r, float startAngle, float arcAngle, float width, float viewWidth, float viewHeight,
			  bool applyTransform = false)
{
	std::vector<GLfloat> positions;
	std::vector<GLushort> indices;

	float startAngleBKP = startAngle;

	if (isPositive(startAngleBKP))
	{
		startAngle *= -1;
		arcAngle *= -1;
	}

	generateArc2(numSegments, cx, cy, r, startAngle, arcAngle, width, positions, indices);
	if (applyTransform)
	{
		applyPerspectiveTransform(positions);
	}
	normalize2DVertices(positions, viewWidth, viewHeight);

	if (isPositive(startAngleBKP))
	{
		for (int i = 0; i < numSegments; ++i)
		{
			positions[i * 4] = -positions[i * 4];
			positions[i * 4 + 2] = -positions[i * 4 + 2];
		}
	}

	genericDraw(positions, indices, GL_TRIANGLE_STRIP);
}

void drawTrail(float steeringAngle, float arcLen, float viewWidth, float viewHeight, bool applyTransform = false)
{
	float sign = std::copysign(1.0, steeringAngle);

	float r = std::abs(carHeight / sin(steeringAngle));
	float cy = carBrY;

	float cx = carX - cos(steeringAngle) * r;

	float startAngle = steeringAngle;
	float arcAngle = sign * arcLen / r;

	float r2 = r + std::abs(cos(steeringAngle) * carWidth);
	float startAngle2 = sign * std::asin(carHeight / r2);
	float arcAngle2 = sign * arcLen / r2;

	static bool printed = false;
	if (!printed)
	{
		std::cout << "cx = " << cx << std::endl
				  << "cy = " << cy << std::endl
				  << "r = " << r << std::endl
				  << "startAngle = " << startAngle << std::endl
				  << "arcAngle = " << arcAngle << std::endl
				  << "r2 = " << r2 << std::endl
				  << "startAngle2 = " << startAngle2 << std::endl;
		printed = true;
	}


//	drawArc(cx, cy, r, startAngle, arcAngle);
//	drawArc(cx, cy, r2, startAngle2, arcAngle2);

	drawArc2(cx, cy, r, startAngle, arcAngle, 5, viewWidth, viewHeight, applyTransform);
	drawArc2(cx, cy, r2, startAngle2, arcAngle2, 5, viewWidth, viewHeight, applyTransform);
}

void drawTrailInBEV(float steeringAngle, float arcLen)
{
	drawTrail(steeringAngle, arcLen, bevWidth, bevHeight, false);
}


void drawTrailInSingleView(float steeringAngle, float arcLen)
{
	drawTrail(steeringAngle, arcLen, svWidth, svHeight, true);
}

float steeringStep = M_PI / 180.0;
float steeringAngle = M_PI / 180.0 * 31.0;

#if 0
void drawFunc(ESContext* ctx)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);


	//BEV

	glViewport(0, 0, bevWidth, bevHeight);
	drawRect(0, 0, bevWidth, bevHeight, bevWidth, bevHeight);
	drawRect(carX, carY, carWidth, carHeight, bevWidth, bevHeight);
	drawTrailInBEV(steeringAngle, 200);

	//SV
	glViewport(810, 1080 - svHeight, svWidth, svHeight);
	drawRect(0, 0, svWidth, svHeight, svWidth, svHeight);
	drawTrailInSingleView(steeringAngle, 200);
	glViewport(0, 0, winWidth, winHeight);
}

void keyFunc(ESContext* ctx, unsigned char ch, int asdf, int hjkl)
{
	switch (ch)
	{
	case 'q':
		exit(0);
		break;
	case 'a':
		std::cout << "turn left: ";
		steeringAngle = clamp(steeringAngle - steeringStep, minAngle, maxAngle);
		break;
	case 'd':
		std::cout << "turn right: ";
		steeringAngle = clamp(steeringAngle + steeringStep, minAngle, maxAngle);
		break;
	default:
		break;
	}

	std::cout << steeringAngle << std::endl;
}


int main()
{
	ESContext ctx;
	esInitContext(&ctx);

	esCreateWindow(&ctx, "trail", winWidth, winHeight, ES_WINDOW_RGB);

	esRegisterDrawFunc(&ctx, drawFunc);
	esRegisterKeyFunc(&ctx, keyFunc);

	esMainLoop(&ctx);

	return 0;
}
#endif

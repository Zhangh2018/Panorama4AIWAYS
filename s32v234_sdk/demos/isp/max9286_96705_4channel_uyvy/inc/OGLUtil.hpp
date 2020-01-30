#pragma once

#include <tuple>
#include <vector>
#include <GLES2/gl2.h>

static inline std::tuple<GLfloat, GLfloat> normalize2DVertex(float x, float y, float width, float height)
{
	GLfloat gx = x / (width / 2.0) - 1.0;
	GLfloat gy = 1.0 - y / (height / 2.0);
	return std::tuple<GLfloat, GLfloat>(gx, gy);
}

static inline void normalize2DVertices(std::vector<GLfloat>& positions, float width, float height)
{
	for (int i = 0; i < positions.size() / 2; ++i)
	{
		auto& x = positions[i * 2];
		auto& y = positions[i * 2 + 1];

		std::tie(x, y) = normalize2DVertex(x, y, width, height);
	}
}

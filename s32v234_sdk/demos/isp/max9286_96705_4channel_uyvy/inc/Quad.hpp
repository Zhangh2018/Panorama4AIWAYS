#pragma once

#include <GLES2/gl2.h>
#include <opencv2/core.hpp>

struct Quad
{
	cv::Point2f points[4];
#if 0
	struct
	{
		cv::Point2f topLeft;
		cv::Point2f topRight;
		cv::Point2f bottomLeft;
		cv::Point2f bottomRight;
	};
#else
	inline cv::Point2f& topLeft() { return points[0]; }
	inline cv::Point2f& topRight() { return points[1]; }
	inline cv::Point2f& bottomLeft() { return points[2]; }
	inline cv::Point2f& bottomRight() { return points[3]; }

	inline const cv::Point2f& topLeft() const { return points[0]; }
	inline const cv::Point2f& topRight() const { return points[1]; }
	inline const cv::Point2f& bottomLeft() const { return points[2]; }
	inline const cv::Point2f& bottomRight() const { return points[3]; }
#endif

	Quad() = default;

	/**
	 * construct a rect
	 * @param x 左上角
	 * @param y
	 * @param width
	 * @param height
	 */
	Quad(float x, float y, float width, float height)
	{
		topLeft().x = x;
		topLeft().y = y;

		topRight().x = x + width;
		topRight().y = y;

		bottomLeft().x = x;
		bottomLeft().y = y + height;

		bottomRight().x = x + width;
		bottomRight().y = y + height;
	}

	std::vector<GLfloat> vertex()
	{
		std::vector<GLfloat> positions;
		int index[] = {0, 1, 3, 2};
		for (int i = 0; i < 4; ++i)
		{
			const auto& p = points[ index[i] ];
			positions.push_back(p.x);
			positions.push_back(p.y);
		}
		return positions;
	}
};
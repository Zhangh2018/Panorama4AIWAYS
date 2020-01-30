#pragma once

#include <tuple>
#include <opencv2/core.hpp>
#include "Quad.hpp"

/**
 * cv2NGLCoord, ngl2CVCoord 系列函数用于 OpenCV 与 OpenGL NDC 顶点坐标系间的转换。
 *
 * OpenCV 坐标系以屏幕左上角为坐标原点，x 轴向右为正，y 轴向下为正
 * normalized OpenGL 坐标系以屏幕中心为坐标原点，x 轴向右为正，y 轴向上为正，normalize 后坐标范围为 [-1, 1]
 */
template<class T>
static inline std::tuple<T, T> cv2GLNDCoord(const T& x, const T& y, const T& width, const T& height)
{
	T gx = x / (width / T(2)) - T(1);
	T gy = T(1) - y / (height / T(2));
	return std::tuple<T, T>(gx, gy);
}

template<class T>
static inline std::tuple<T, T> glndc2CVCoord(const T& x, const T& y, const T& width, const T& height)
{
	T vx = (x + T(1)) * (width / T(2));
	T vy = (T(1) - y) * (height / T(2));
	return std::tuple<T, T>(vx, vy);
}

template<class T>
static inline cv::Point_<T> glndc2CVCoord(const cv::Point_<T>& p, const T& width, const T& height)
{
	cv::Point_<T> vp;
	std::tie(vp.x, vp.y) =  glndc2CVCoord(p.x, p.y, width, height);
	return vp;
}

template<class T>
static inline cv::Point_<T> cv2GLNDCoord(const cv::Point_<T>& p, const T& width, const T& height)
{
	cv::Point_<T> gp;
	std::tie(gp.x, gp.y) = cv2GLNDCoord(p.x, p.y, width, height);
	return gp;
}

static inline Quad glndc2CVCoord(const Quad& quad, float width, float height)
{
	Quad q = quad;
	for (int i = 0; i < 4; ++i)
	{
		q.points[i] = glndc2CVCoord(q.points[i], width, height);
	}
	return q;
}

static inline Quad cv2GLNDCoord(const Quad& quad, float width, float height)
{
	Quad q = quad;
	for (int i = 0; i < 4; ++i)
	{
		q.points[i] = cv2GLNDCoord(q.points[i], width, height);
	}
	return q;
}

template<class T>
static inline std::tuple<T, T> transformCVCoordNDC(const T& x, const T& y, float width, float height, const glm::mat4& transform)
{
	auto glCoord = cv2GLNDCoord(x, y, width, height);
	auto glV = glm::vec4(std::get<0>(glCoord), std::get<1>(glCoord), 1.0, 1.0);
	auto transV = transform * glV;
	return glndc2CVCoord(transV[0], transV[1], width, height);
}


template<class T>
static inline cv::Point_<T> transformCVCoordNDC(const cv::Point_<T>& p, float width, float height, const glm::mat4& transform)
{
	auto transT = transformCVCoordNDC(p.x, p.y, width, height, transform);
	return cv::Point_<T>(std::get<0>(transT), std::get<1>(transT));
}

static inline Quad transformCVCoordNDC(const Quad& quad, float width, float height, const glm::mat4& transform)
{
	auto glQ = quad;
	for (int i = 0; i < 4; ++i)
	{
		glQ.points[i] = transformCVCoordNDC(glQ.points[i], width, height, transform);
	}
	return glQ;
}


/**
 * OpenCV 坐标系与 OpenGL 坐标系的转换，其中 OpenGL 并非平常的 NDC 坐标，
 * 而只是将坐标系原点移到画面中心，OpenGL 坐标系的范围为 [-width / 2, width / 2], [-height / 2, height / 2]
 */
template<class T>
static inline std::tuple<T, T> cv2GLCoord(const T& x, const T& y, const T& width, const T& height)
{
	T gx = x - (width / T(2));
	T gy = (height / T(2)) - y;
	return std::tuple<T, T>(gx, gy);
}

template<class T>
static inline std::tuple<T, T> gl2CVCoord(const T& x, const T& y, const T& width, const T& height)
{
	T vx = x + (width / T(2));
	T vy = (height / T(2)) - y;
	return std::tuple<T, T>(vx, vy);
}

template<class T>
static inline cv::Point_<T> gl2CVCoord(const cv::Point_<T>& p, const T& width, const T& height)
{
	cv::Point_<T> vp;
	std::tie(vp.x, vp.y) =  gl2CVCoord(p.x, p.y, width, height);
	return vp;
}

template<class T>
static inline cv::Point_<T> cv2GLCoord(const cv::Point_<T>& p, const T& width, const T& height)
{
	cv::Point_<T> gp;
	std::tie(gp.x, gp.y) = cv2GLCoord(p.x, p.y, width, height);
	return gp;
}

static inline Quad gl2CVCoord(const Quad& quad, float width, float height)
{
	Quad q = quad;
	for (int i = 0; i < 4; ++i)
	{
		q.points[i] = gl2CVCoord(q.points[i], width, height);
	}
	return q;
}

static inline Quad cv2GLCoord(const Quad& quad, float width, float height)
{
	Quad q = quad;
	for (int i = 0; i < 4; ++i)
	{
		q.points[i] = cv2GLCoord(q.points[i], width, height);
	}
	return q;
}

template<class T>
static inline std::tuple<T, T> transformCVCoord(const T& x, const T& y, float width, float height, const glm::mat4& transform)
{
	auto glCoord = cv2GLCoord(x, y, width, height);
	auto glV = glm::vec4(std::get<0>(glCoord), std::get<1>(glCoord), 0.0, 1.0);
	auto transV = transform * glV;
	return gl2CVCoord(transV[0], transV[1], width, height);
}

template<class T>
static inline cv::Point_<T> transformCVCoord(const cv::Point_<T>& p, float width, float height, const glm::mat4& transform)
{
	auto transT = transformCVCoord(p.x, p.y, width, height, transform);
	return cv::Point_<T>(std::get<0>(transT), std::get<1>(transT));
}

static inline Quad transformCVCoord(const Quad& quad, float width, float height, const glm::mat4& transform)
{
	auto glQ = quad;
	for (int i = 0; i < 4; ++i)
	{
		glQ.points[i] = transformCVCoord(glQ.points[i], width, height, transform);
	}
	return glQ;
}

static inline cv::Rect transformCVCoord(const cv::Rect& rect, float width, float height, const glm::mat4& transform)
{
	Quad q(rect.x, rect.y, rect.width, rect.height);
	Quad transQ = transformCVCoord(q, width, height, transform);
	return cv::Rect(transQ.topLeft().x,
		transQ.topRight().y,
		transQ.topRight().x - transQ.topLeft().x,
		transQ.bottomLeft().y - transQ.topLeft().y);
}

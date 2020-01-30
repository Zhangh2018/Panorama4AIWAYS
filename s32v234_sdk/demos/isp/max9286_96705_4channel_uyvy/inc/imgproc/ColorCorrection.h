#pragma once

#include <type_traits>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

template<class T>
static inline std::vector<T> genGammaLUT(double gamma, T min, T max, T step = 1)
{
	static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

	std::vector<T> lut;
	for (T i = min; i <= max; i += step)
	{
		lut[i] = std::pow(i, gamma);
	}
	return lut;
}

//template<class TPixel>
//typename std::enable_if<std::is_arithmetic<TPixel>::value, cv::Mat_<TPixel>>
//gammaCorrection(const cv::Mat_<TPixel>& src, double gamma = 1.0)
//{
//	cv::Mat_<TPixel> dst = cv::Mat_<TPixel>::zeros(src.size());
//	auto lut = genGammaLUT(gamma, std::numeric_limits<TPixel>::min(), std::numeric_limits<TPixel>::max());
//	src.forEach(
//		[&](TPixel& pixel, const int* position)
//		{
//			int x = position[0];
//			int y = position[1];
//			dst(y, x) = lut[src(y, x)];
//		}
//	);
//
//	return dst;
//}

template<class TPixel>
cv::Mat_<TPixel>
gammaCorrection(const cv::Mat_<TPixel>& src, double gamma = 1.0)
{
	int nChannels = src.channels();
	cv::Mat_<TPixel> dst = cv::Mat_<TPixel>::zeros(src.size());

	using ValueType = typename cv::DataType<TPixel>::value_type ;
	auto lut = genGammaLUT<ValueType>(gamma, std::numeric_limits<ValueType>::min(), std::numeric_limits<ValueType>::max());
	src.forEach(
		[&](TPixel& pixel, const int* position)
		{
			int x = position[0];
			int y = position[1];
			for (int c = 0; c < nChannels; ++c)
			{
				dst(y, x)[c] = lut[src(y, x)[0]];
			}
		}
	);

	return dst;
}


template<class TPixel>
cv::Vec3d computeGain(const cv::Mat_<TPixel>& a, const cv::Mat_<TPixel>& b, double gamma)
{
	auto ga = gammaCorrection<TPixel>(a, gamma);
	auto gb = gammaCorrection<TPixel>(b, gamma);

	auto gainS = cv::sum(ga) / cv::sum(gb);
	cv::Vec3d gain;
	gain = gainS[0];
	gain = gainS[1];
	gain = gainS[2];
	return gain;
}


template<class TPixel>
cv::Mat_<TPixel> colorCorrection(cv::Mat_<TPixel>* imgs, int count)
{
	cv::Vec3d gains[4];
	cv::Vec3d gainSum;
	cv::Vec3d gain2Sum;

	gains[0][0] = gains[0][1] = gains[0][2] = 1;
	for (int i = 1; i < 4; ++i)
	{
		gains[i] = computeGain(imgs[i - 1], imgs[i], 1.5);
		gainSum += gains[i];
		gain2Sum += gains[i].mul(gains[i]);
	}

	//gains[0][0] = cv::Vec3d::ones();
	cv::Vec3d globalGain;
	cv::divide(gainSum, gain2Sum, globalGain);

	for (int i = 1; i < 4; ++i)
	{
		imgs[i] = globalGain * gains[i] * imgs[i];
	}
}

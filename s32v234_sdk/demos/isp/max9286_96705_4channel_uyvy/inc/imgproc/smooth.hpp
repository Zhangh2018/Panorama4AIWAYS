/***********************************************************************
 *
 * Mario Created on   : 2019-04-19 14:11
 * Filename      : smooth.hpp
 * Function      : 亮度均衡实现代码
 *
 *************************************************************************/

#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <math.h>
#include "LUT.h"
#include <vector>
#include "LuminanceEqualize.hpp"
#include <numeric>
#include <functional>

#include <chrono>
#include <toml11/toml.hpp>

class ExposureCompensation
{
private:

	// 融合像素灰度阈值(防止在看到有高度的物体时,两个相机看到的图像不一致的情况)
	int intensityThreshold = 30;

	// 滞后滤波系数
	float lagAlpha = 0.4;

	// 亮度均衡gains阈值(防止产生过度均衡效果)
	float minGain = 0.7;
	float maxGain = 1.3;

	// 前一帧的gains
	cv::Mat_<double> preGains{4, 1};

public:
	ExposureCompensation(const ExposureCompensation&) = default;

	/**
	 * @param intensityThreshold 融合像素灰度阈值
	 * @param lagAlpha 滞后滤波系数
	 * @param minGain 亮度均衡gains阈值
	 * @param maxGain 亮度均衡gains阈值
	 */
	ExposureCompensation(float intensityThreshold = 30, float lagAlpha = 0.4, float minGain = 0.7, float maxGain = 1.3)
		: intensityThreshold(intensityThreshold), lagAlpha(lagAlpha), minGain(minGain), maxGain(maxGain)
	{
	}

	/**
	 * 从配置文件初始化
	 * @param configPath
	 */
	ExposureCompensation(const char* configPath)
	{
		const auto config = toml::parse(configPath);
		intensityThreshold = toml::find<int>(config, "intensity_threshold");
		lagAlpha = toml::find<float>(config, "lag_alpha");
		minGain = toml::find<float>(config, "min_gain");
		maxGain = toml::find<float>(config, "max_gain");
	}

	/**
	 * @brief 遍历求每一矩形像素块融合区域的灰度值
	 *
	 * @param images                    输入的4张BGR图像
	 * @param sum1                      第一个摄像头在矩形像素块中的灰度总和
	 * @param sum2                      第二个摄像头在矩形像素块中的灰度总和
	 * @param rowstart                  这四个参数定义了矩形像素块
	 * @param rowend
	 * @param colstart
	 * @param colend
	 * @param birdviewWidth             俯视图的宽度(用于遍历)
	 * @param lut                       lut表
	 * @param mergePixelNum             这一矩形像素块中有多少个点是融合点
	 */
	static void iterationSumBGR(cv::Mat *images,
	                     double& sum1,
	                     double& sum2,
	                     const int rowstart,
	                     const int rowend,
	                     const int colstart,
	                     const int colend,
	                     const int birdviewWidth,
	                     const int intensityThreshold,
	                     const CalibLUT& lut,
	                     int& mergePixelNum)
	{

		sum1 = 0;
		sum2 = 0;
		mergePixelNum = 0;
		int pic1, pic2, x_pos, y_pos;

		// 灰度值 = 0.114 * B + 0.587 * G + 0.299 * R
		const float bgr2y[] = {0.114, 0.587, 0.299};

		// 遍历矩形框:([rowstart, rowend],[colstart,colend])内的像素点
		for (int i = rowstart; i < rowend; ++i)
			for (int j = colstart; j < colend; ++j)
			{

				int idx = i * birdviewWidth + j;

				// 用来判断是不是融合区域 TODO: CHECK
				if ((lut.table[idx].wt_fusion != 0) && (lut.table[idx].wt_fusion != 255))
				{

					// pic1是前视图或后视图, 所以对于左下角情况, pic1=1, pic2=3;
					pic1 = lut.table[idx].pic1;
					pic2 = lut.table[idx].pic2;

					x_pos = lut.table[idx].point_pos1.x;
					y_pos = lut.table[idx].point_pos1.y;
					auto p = images[pic1].at<cv::Vec3b>(y_pos, x_pos);
					double intensity1 = 0;
					for (int i = 0; i < 3; ++i)
					{
						// 灰度
						intensity1 += p[i] * bgr2y[i];
					}

					x_pos = lut.table[idx].point_pos2.x;
					y_pos = lut.table[idx].point_pos2.y;
					p = images[pic2].at<cv::Vec3b>(y_pos, x_pos);
					double intensity2 = 0;
					for (int i = 0; i < 3; ++i)
					{
						intensity2 += p[i] * bgr2y[i];
					}

					if (abs(intensity1 - intensity2) < intensityThreshold)
					{
						// 融合区域计数
						mergePixelNum++;
						// 计算第一个矩形像素块的灰度总和
						sum1 += intensity1;
						// 计算第二个矩形像素块的灰度总和
						sum2 += intensity2;
					}
				}
			}
	}

	/**
	 * @brief 遍历求每一矩形像素块融合区域的灰度值[UYVY格式]
	 * 实现与BGR类似,徐伟修改
	 *
	 * @param images                    输入的4张UYVY图像
	 * @param sum1                      第一个摄像头在矩形像素块中的灰度总和
	 * @param sum2                      第二个摄像头在矩形像素块中的灰度总和
	 * @param rowstart                  这四个参数定义了矩形像素块
	 * @param rowend
	 * @param colstart
	 * @param colend
	 * @param birdviewWidth             俯视图的宽度(用于遍历)
	 * @param lut                       lut表
	 * @param mergePixelNum             这一矩形像素块中有多少个点是融合点
	 */
	static void iterationSumUYVY(cv::Mat *images,
	                      double& sum1,
	                      double& sum2,
	                      const int rowstart,
	                      const int rowend,
	                      const int colstart,
	                      const int colend,
	                      const int birdviewWidth,
	                      const int intensityThreshold,
	                      const CalibLUT& lut,
	                      int& mergePixelNum)
	{

		sum1 = 0;
		sum2 = 0;
		mergePixelNum = 0;
		int pic1, pic2, x_pos, y_pos;

		const float bgr2y[] = {0.114, 0.587, 0.299};

		for (int i = rowstart; i < rowend; ++i)
			for (int j = colstart; j < colend; ++j)
			{

				int idx = i * birdviewWidth + j;

				if ((lut.table[idx].wt_fusion != 0) && (lut.table[idx].wt_fusion != 255))
				{

					mergePixelNum++;
					pic1 = lut.table[idx].pic1;
					pic2 = lut.table[idx].pic2;

					x_pos = lut.table[idx].point_pos1.x;
					y_pos = lut.table[idx].point_pos1.y;

					auto p = images[pic1].at<cv::Vec2b>(y_pos, x_pos);
					int intensity1 = p[1];

					x_pos = lut.table[idx].point_pos2.x;
					y_pos = lut.table[idx].point_pos2.y;

					p = images[pic2].at<cv::Vec2b>(y_pos, x_pos);
					int intensity2 = p[1];

					if (abs(intensity1 - intensity2) < intensityThreshold)
					{
						// 融合区域计数
						mergePixelNum++;
						// 计算第一个矩形像素块的灰度总和
						sum1 += intensity1;
						// 计算第二个矩形像素块的灰度总和
						sum2 += intensity2;
					}
				}
			}
	}


	/**
 * @brief 遍历求每一矩形像素块融合区域的灰度值[YUYV格式]
 * 实现与BGR类似,徐伟修改
 *
 * @param images                    输入的4张YUYV图像
 * @param sum1                      第一个摄像头在矩形像素块中的灰度总和
 * @param sum2                      第二个摄像头在矩形像素块中的灰度总和
 * @param rowstart                  这四个参数定义了矩形像素块
 * @param rowend
 * @param colstart
 * @param colend
 * @param birdviewWidth             俯视图的宽度(用于遍历)
 * @param lut                       lut表
 * @param mergePixelNum             这一矩形像素块中有多少个点是融合点
 */
	static void iterationSumYUYV(cv::Mat *images,
	                             double& sum1,
	                             double& sum2,
	                             const int rowstart,
	                             const int rowend,
	                             const int colstart,
	                             const int colend,
	                             const int birdviewWidth,
	                             const int intensityThreshold,
	                             const CalibLUT& lut,
	                             int& mergePixelNum)
	{

		sum1 = 0;
		sum2 = 0;
		mergePixelNum = 0;
		int pic1, pic2, x_pos, y_pos;

		const float bgr2y[] = {0.114, 0.587, 0.299};

		for (int i = rowstart; i < rowend; ++i)
			for (int j = colstart; j < colend; ++j)
			{

				int idx = i * birdviewWidth + j;

				if ((lut.table[idx].wt_fusion != 0) && (lut.table[idx].wt_fusion != 255))
				{

					mergePixelNum++;
					pic1 = lut.table[idx].pic1;
					pic2 = lut.table[idx].pic2;

					x_pos = lut.table[idx].point_pos1.x;
					y_pos = lut.table[idx].point_pos1.y;

					auto p = images[pic1].at<cv::Vec2b>(y_pos, x_pos);
					int intensity1 = p[0];

					x_pos = lut.table[idx].point_pos2.x;
					y_pos = lut.table[idx].point_pos2.y;

					p = images[pic2].at<cv::Vec2b>(y_pos, x_pos);
					int intensity2 = p[0];

					if (abs(intensity1 - intensity2) < intensityThreshold)
					{
						// 融合区域计数
						mergePixelNum++;
						// 计算第一个矩形像素块的灰度总和
						sum1 += intensity1;
						// 计算第二个矩形像素块的灰度总和
						sum2 += intensity2;
					}
				}
			}
	}


	/**
	 * @brief 亮度均衡代码具体实现
	 *
	 * @tparam TPixel
	 * @tparam IterationSumFunc
	 * @param images                4张图像[Input]
	 * @param lut                   Lut表[Input]
	 * @param gains                 增益[Output]
	 * @param iterationSum          图像遍历函数
	 */
	template<class IterationSumFunc>
	void calcExposureGain(cv::Mat *images, const CalibLUT& lut, float *gains, IterationSumFunc iterationSum)
	{

		// 表示融合区域长宽 [305, 313]
		int mergeHeight = lut.header.car_Icon_Rect.y;
		int mergeWidth = lut.header.car_Icon_Rect.x;

		// 表示原图长宽 [1280, 720]
		int srcWidth = lut.header.src_img_width;
		int srcHeight = lut.header.src_img_height;

		// 表示俯视图长宽 [810, 1080]
		int birdviewWidth = lut.header.bev_img_width;
		int birdviewHeight = lut.header.bev_img_height;

		// 表示待拼接的图像数量
		const int num_images = 4;

		// ** IMPORTANT **:
		// N: 4*4
		//  表示两幅图像间重叠部分的像素数量
		cv::Mat_<int> N(num_images, num_images);
		N.setTo(0);

		// ** IMPORTANT **:
		// I: 4*4
		//  表示两幅图像间重叠部分的(第一个元素所代表的图像的)亮度和
		cv::Mat_<double> I(num_images, num_images);
		I.setTo(0);

		// ************ 计算I与N ************
		auto t1 = std::chrono::high_resolution_clock::now();
		double sum1;
		double sum2;
		int mergePixelNum;

		/**
		区域划分:
				 0
			 ---------
			 |       |
			 |       |
		 2   |       |   3
			 |       |
			 |       |
			 ---------
				 1
		*/

		// 左上角 对应0,2区域
		iterationSum(images, sum1, sum2, mergeHeight - 90, mergeHeight - 40, mergeWidth - 50, mergeWidth, birdviewWidth,
		             intensityThreshold, lut, mergePixelNum);
		N(0, 2) = N(2, 0) = mergePixelNum;
		if (N(0, 2) != 0)
		{
			I(0, 2) = sum1 / N(0, 2);
			I(2, 0) = sum2 / N(2, 0);
		}

		// 右下角
		iterationSum(images, sum1, sum2, mergeHeight - 90, mergeHeight - 40, birdviewWidth - mergeWidth,
		             birdviewWidth - mergeWidth + 50, birdviewWidth, intensityThreshold, lut, mergePixelNum);
		N(0, 3) = N(3, 0) = mergePixelNum;
		if (N(0, 3) != 0)
		{
			I(0, 3) = sum1 / N(0, 3);
			I(3, 0) = sum2 / N(3, 0);
		}

		// 左下角
		iterationSum(images, sum1, sum2, birdviewHeight - mergeHeight + 10, birdviewHeight - mergeHeight + 60,
		             mergeWidth - 50, mergeWidth, birdviewWidth, intensityThreshold, lut, mergePixelNum);
		N(1, 2) = N(2, 1) = mergePixelNum;
		if (N(1, 2) != 0)
		{
			I(1, 2) = sum1 / N(1, 2);
			I(2, 1) = sum2 / N(2, 1);
		}

		// 右下角
		iterationSum(images, sum1, sum2, birdviewHeight - mergeHeight + 10, birdviewHeight - mergeHeight + 60,
		             birdviewWidth - mergeWidth, birdviewWidth - mergeWidth + 50, birdviewWidth, intensityThreshold,
		             lut, mergePixelNum);
		N(1, 3) = N(3, 1) = mergePixelNum;
		if (N(1, 3) != 0)
		{
			I(1, 3) = sum1 / N(1, 3);
			I(3, 1) = sum2 / N(3, 1);
		}

		// ************ 最小二乘 ************
		double alpha = 0.01;    //表示σN的平方的倒数
		double beta = 100;    //表示σg的平方的倒数

		cv::Mat_<double> A(num_images, num_images);
		A.setTo(0);
		cv::Mat_<double> b(num_images, 1);
		b.setTo(0);

		for (int i = 0; i < num_images; ++i)
		{
			for (int j = 0; j < num_images; ++j)
			{
				b(i, 0) += beta * N(i, j);
				A(i, i) += beta * N(i, j);
				if (j == i) continue;
				A(i, i) += 2 * alpha * I(i, j) * I(i, j) * N(i, j);
				A(i, j) -= 2 * alpha * I(i, j) * I(j, i) * N(i, j);
			}
		}

		cv::Mat_<double> gains_;
		cv::solve(A, b, gains_);

		auto t2 = std::chrono::high_resolution_clock::now();
		auto timeUsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		#if 0
		std::cout << " !!!! allExposureComposition solve time cost = " << timeUsed << " ms. " << std::endl;
		std::cout << "!!! A: " << A << std::endl;
		std::cout << "!!! b: " << b << std::endl;
		std::cout << "!!! N: " << N << std::endl;
		std::cout << "!!! I: " << I << std::endl;
		std::cout << "!!!gains = " << gains_ << std::endl;
		#endif
		// 滞后gains
		for (int i = 0; i < 4; ++i)
		{

			// 阈值操作
			gains_(i, 0) = (gains_(i, 0) > maxGain) ? maxGain : gains_(i, 0);
			gains_(i, 0) = (gains_(i, 0) < minGain) ? minGain : gains_(i, 0);

			gains[i] = lagAlpha * preGains(i, 0) +
			           (1 - lagAlpha) * gains_(i, 0);

			preGains(i, 0) = gains[i];
		}
	}


	/**
	 * @brief 从BGR图片计算亮度均衡增益gains接口
	 *
	 * @tparam TPixel
	 * @param images        输入4张图片[Input]
	 * @param lut           Lut表[Input]
	 * @param gains         亮度均衡增益[Output]
	 */
	inline void calcExposureGainBGR(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain(images, lut, gains, iterationSumBGR);
	}

	/**
	 * @brief 从UYVY图片计算亮度均衡增益gains接口
	 *
	 * @tparam TPixel
	 * @param images        输入4张图片[Input]
	 * @param lut           Lut表[Input]
	 * @param gains         亮度均衡增益[Output]
	 */
	inline void calcExposureGainUYVY(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain(images, lut, gains, iterationSumUYVY);
	}

	inline void calcExposureGainYUYV(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain(images, lut, gains, iterationSumYUYV);
	}
};
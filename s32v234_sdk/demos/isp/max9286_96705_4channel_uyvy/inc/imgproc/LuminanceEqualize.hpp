#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/Timer.hpp"
#include "utils/utils.hpp"

template<class TPixel>
void LumEqual(cv::Mat_<TPixel>* images, const CalibLUT& lut)
{
	int FRONT = 0, REAR = 1, LEFT = 2, RIGHT = 3;

	int srcWidth = lut.header.src_img_width;
	int srcHeight = lut.header.src_img_height;

	int birdviewWidth = lut.header.bev_img_width;
	int birdviewHeight = lut.header.bev_img_height;

	float	Ysum[4][4] = { 0 };
	int		totalSize[4][4] = { 0 };
	float	Yavg[4][4] = { 0 }, Ydiff[4] = { 0 };
	int		step = srcWidth * 3;
	int		tSize = 0;

	int offset = 260;


	cv::Mat_<TPixel> Lab[4];
	for (int i = 0; i < 4; ++i)
	{
		cv::cvtColor(images[i], Lab[i], cv::COLOR_BGR2Lab);
	}

	for (int y = offset; y < birdviewHeight - offset + 10; y++)
		for (int x = offset; x < birdviewWidth - offset; x++)
		{
			int idx = y * birdviewWidth + x;
			if ((lut.table[idx].wt_fusion != 0) && (lut.table[idx].wt_fusion != 255) && lut.table[idx].pic1 != lut.table[idx].pic2)
			{
				int pic1 = lut.table[idx].pic1;
				int pic2 = lut.table[idx].pic2;

				int y_pos = lut.table[idx].point_pos1.y;
				int x_pos = lut.table[idx].point_pos1.x;
				Ysum[pic1][pic2] += Lab[pic1](y_pos,x_pos)[0];
				totalSize[pic1][pic2]++;

				y_pos = lut.table[idx].point_pos2.y;
				x_pos = lut.table[idx].point_pos2.x;
				Ysum[pic2][pic1] += Lab[pic2](y_pos,x_pos)[0];
				totalSize[pic2][pic1]++;
			}
		}
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			if (totalSize[i][j] != 0)
				Yavg[i][j] = Ysum[i][j] / totalSize[i][j];

	Ydiff[LEFT]  = Yavg[FRONT][LEFT]  - Yavg[LEFT][FRONT];
	Ydiff[RIGHT] = Yavg[FRONT][RIGHT] - Yavg[RIGHT][FRONT];
	Ydiff[REAR]  = Yavg[LEFT][REAR]   - Yavg[REAR][LEFT] + Ydiff[LEFT];

	for (int k = 1; k < 4; k++)
	{

		for (int j = 0; j < srcHeight; j++)
		{

			for (int i = 0; i < srcWidth; i++)
			{
#if 1
				Lab[k](j,i)[0] = clamp<float>((float)Lab[k](j,i)[0] + Ydiff[k], 0.0, 255.0);
#else
				for (int ch = 0; ch < 3; ch++)
				{
					images[k](j, i)[ch] = clamp<float>(images[k](j, i)[ch] + Ydiff[k], 0.0, 255.0);
				}
#endif
			}
		}
	}


	printf("Ydiff = [%f, %f, %f, %f]\n", Ydiff[0], Ydiff[1], Ydiff[2], Ydiff[3]);

	for (int i = 0; i < 4; ++i)
	{
		cv::cvtColor(Lab[i], images[i], cv::COLOR_Lab2BGR);
	}
}


template<class TPixel>
void calcLumDelta(const cv::Mat_<TPixel>* images, const CalibLUT& lut, float* delta)
{
	int FRONT = 0, REAR = 1, LEFT = 2, RIGHT = 3;

	int srcWidth = lut.header.src_img_width;
	int srcHeight = lut.header.src_img_height;

	int birdviewWidth = lut.header.bev_img_width;
	int birdviewHeight = lut.header.bev_img_height;

	float	Ysum[4][4] = { 0 };
	int		totalSize[4][4] = { 0 };
	float	Yavg[4][4] = { 0 };
	int		step = srcWidth * 3;
	int		tSize = 0;


	int offset = 260;


	// cv::Mat_<TPixel> YUV[4];
	// Timer<> timer;
	// timer.start();

	// for (int i = 0; i < 4; ++i)
	// {
	// 	cv::cvtColor(images[i], YUV[i], cv::COLOR_BGR2YUV);
	// }
	// timer.stop();
	// std::cout << "BGR -> YUV: " << timer.get() << std::endl;

	const float BGR2Y[3] = {0.114f, 0.587f, 0.299f};


	for (int y = offset; y < birdviewHeight - offset + 10; y++)
		for (int x = offset; x < birdviewWidth - offset; x++)
		{
			int idx = y * birdviewWidth + x;
			if ((lut.table[idx].wt_fusion != 0) && (lut.table[idx].wt_fusion != 255) && lut.table[idx].pic1 != lut.table[idx].pic2)
			{
				int pic1 = lut.table[idx].pic1;
				int pic2 = lut.table[idx].pic2;

				int y_pos = lut.table[idx].point_pos1.y;
				int x_pos = lut.table[idx].point_pos1.x;
				auto& p1 = images[pic1](y_pos, x_pos);
				Ysum[pic1][pic2] += p1[0] * BGR2Y[0] + p1[1] * BGR2Y[1] + p1[2] * BGR2Y[2];
				totalSize[pic1][pic2]++;

				y_pos = lut.table[idx].point_pos2.y;
				x_pos = lut.table[idx].point_pos2.x;
				auto& p2 = images[pic2](y_pos, x_pos);
				Ysum[pic2][pic1] += p2[0] * BGR2Y[0] + p2[1] * BGR2Y[1] + p2[2] * BGR2Y[2];
				totalSize[pic2][pic1]++;
			}
		}


	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			if (totalSize[i][j] != 0)
			{
				Yavg[i][j] = Ysum[i][j] / totalSize[i][j];
			}

	delta[LEFT]  = Yavg[FRONT][LEFT]  - Yavg[LEFT][FRONT];
	delta[RIGHT] = Yavg[FRONT][RIGHT] - Yavg[RIGHT][FRONT];
	delta[REAR]  = Yavg[LEFT][REAR]   - Yavg[REAR][LEFT] + delta[LEFT];
}

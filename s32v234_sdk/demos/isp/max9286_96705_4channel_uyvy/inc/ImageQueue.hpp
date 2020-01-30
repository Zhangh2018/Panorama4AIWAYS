#pragma once

#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "utils/Timer.hpp"

struct Frame
{
	cv::Mat_<cv::Vec3b> images[4];
	int counter;

	Frame(int n) : counter(n)
	{
		//images = std::vector<cv::Mat>(n);
	}

	~Frame()
	{
	 	for (auto& m : images)
	 	{
	 		m.release();
	 	}
	}
	Frame(const Frame& other)
	{
		for (int i =0; i < 4; ++i)
		{
			images[i] = other.images[i];
		}
		counter = other.counter;
	}

	inline Frame& operator=(const Frame& other)
	{
		for (int i =0; i < 4; ++i)
		{
			images[i] = other.images[i];
		}
		counter = other.counter;
		return *this;
	}
};


class ImageQueue
{
public:

	ImageQueue(const char* pathPattern, int numFrames, int numCams, int maxSize)
		: numFrames(numFrames), numCams(numCams), maxSize(maxSize), frameCounter(1), curFrame(0)
	{
		strcpy(pattern, pathPattern);
		daemon = std::move(std::thread(&ImageQueue::prefetch, this));
	}

	void prefetch()
	{
		while (frameCounter <= numFrames)
		{
			std::unique_lock<std::mutex> lock{ mutex };
			int n = maxSize - queue.size();
			if (n > 0)
			{
				while (n--)
				{
				LoopBegin:
					Frame frame{ numCams };
					//frame.images = std::vector<cv::Mat>(numCams);
					for (int j = 0; j < numCams; ++j)
					{
						char filename[1024] = {0};
						sprintf(filename, pattern, frameCounter, j);
						cv::Mat_<cv::Vec3b> img = cv::imread(filename, cv::IMREAD_COLOR);
						if (img.size().width == 0 || img.size().height == 0)
						{
							++frameCounter;
							goto LoopBegin;
						}
						//[0 1 2 3]
						//[1 3 0 2]
						//[2 0 3 1]
						static const int camMapping[] = { 2, 0, 3, 1 };
						//printf("%d --> %d --> %s\n", j, camMapping[j], filename);

						frame.images[camMapping[j]] = img;
						//frame.images.push_back(img);
					}
					++frameCounter;
					queue.push_back(frame);
				}
				//printf("[prefetch]: queue size = %d\n", queue.size());
				cv1.notify_one();
			}
			cv2.wait_for(lock, std::chrono::milliseconds(10));
		}
	}

	cv::Mat pop(int cam)
	{
		static int pop_counter = 0;
		std::unique_lock<std::mutex> lock{ mutex };

		if (curFrame.counter <= 0)
		{
			if (queue.empty())
			{
				cv2.notify_one();
				cv1.wait(lock);
				//printf("[pop]: queue size = %d\n", queue.size());
			}
			printf("pop %d\n", pop_counter++);
			curFrame = queue.front();
			queue.pop_front();
		}
		curFrame.counter--;
		return curFrame.images[cam];
	}

	bool isEmpty()
	{
		std::unique_lock<std::mutex> lock{ mutex };
		return (frameCounter > numFrames && queue.empty());
	}

private:
	char pattern[1024];
	int frameCounter;
	int numFrames;
	int maxSize;
	int numCams;

	Frame curFrame;

	std::thread daemon;
	std::deque<Frame> queue;

	std::mutex mutex;
	std::condition_variable cv1, cv2;
};

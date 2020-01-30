# pragma once

#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
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


class VideoFrameQueue
{
public:

	VideoFrameQueue(const char* filename, int numFrames, int numCams, int maxSize)
		: numFrames(numFrames), numCams(numCams), maxSize(maxSize), frameCounter(1), curFrame(0)
	{
		capture = cv::VideoCapture{ filename };
		capture.set(cv::CAP_PROP_BUFFERSIZE, 10);
		daemon = std::move(std::thread(&VideoFrameQueue::prefetch, this));
	}

	void prefetch()
	{
		while (frameCounter <= numFrames)
		{
			std::unique_lock<std::mutex> lock{ mutex };
			int n = maxSize - queue.size();
			if (n > 0)
			{
				Timer<> timer;

				while (n--)
				{
					cv::Mat f;
					timer.start();
					capture >> f;
					timer.stop();

					static const cv::Rect ROIs[] = {
						cv::Rect{0, 0, 1280, 720},
						cv::Rect{1280, 0, 1280, 720},
						cv::Rect{0, 720, 1280, 720},
						cv::Rect{1280, 720, 1280, 720}
					};

					Frame frame{ numCams };
					for (int j = 0; j < numCams; ++j)
					{
						cv::Mat roi = (f(ROIs[j])).clone();

						static const int camMapping[] = {0, 2, 3, 1};
						frame.images[camMapping[j]] = roi.clone();
					}
					++frameCounter;
					queue.push_back(frame);
				}

				cv1.notify_one();
				isFull = true;
				//printf("[prefetch]: queue size = %d\n", queue.size());
			}
			cv2.wait_for(lock, std::chrono::milliseconds(10));
			if (mutex.try_lock())
			{
				mutex.unlock();
			}
		}
	}

	cv::Mat pop(int cam)
	{
		Timer<> timer;
		timer.start();

		std::unique_lock<std::mutex> lock{ mutex };

		timer.stop();

		if (curFrame.counter-- <= 0)
		{
			if (queue.empty())
			{

				Timer<> timer;
				timer.start();

				cv2.notify_one();
				cv1.wait(lock);

				timer.stop();

				//printf("[pop]: queue size = %d\n", queue.size());
			}

			timer.start();
			curFrame = queue.front();
			timer.stop();

			timer.start();
			queue.pop_front();
			timer.stop();

			isFull = false;
		}
		return curFrame.images[cam];
	}

	bool isEmpty()
	{
		std::unique_lock<std::mutex> lock{ mutex };
		return (frameCounter > numFrames && queue.empty());
	}

private:
	bool isFull = false;
	int frameCounter;
	int numFrames;
	int maxSize;
	int numCams;

	Frame curFrame;

	cv::VideoCapture capture;

	std::thread daemon;
	std::deque<Frame> queue;

	std::mutex mutex;
	std::condition_variable cv1, cv2;
};

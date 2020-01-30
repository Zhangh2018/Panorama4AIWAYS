#pragma once

#include <GLES2/gl2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/types.h>
#include <sys/stat.h>


class Dumper
{
public:

	cv::Mat dump(int width, int height, int x = 0, int y = 0)
	{
		cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
		glReadPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE, img.data);
		cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
		return img;
	}

	cv::Mat dumpFBO(GLuint fbo, int width, int height, int x = 0, int y = 0)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		cv::Mat img = dump(width, height, x, y);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		return img;
	}
};

class FileDumper : Dumper
{
public:

	FileDumper(const char* directory = "dump", const char* pathPattern = "%s/%04d.png")
		: frameCounter(1), Dumper()
	{
		dir = new char[strlen(directory) + 1];
		strcpy(dir, directory);

		pattern = new char[strlen(pathPattern) + 1];
		strcpy(pattern, pathPattern);

		mkdir(dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	}

	~FileDumper()
	{
		delete[] dir;
		delete[] pattern;
	}

	void dump(int width, int height, int x = 0, int y = 0)
	{
		cv::Mat img = Dumper::dump(width, height, x, y);
		char filename[1024 + 1] = {0};
		sprintf(filename, pattern, dir, frameCounter++);
		cv::imwrite(filename, img);
	}

	void dumpFBO(GLuint fbo, int width, int height, int x = 0, int y = 0)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		dump(width, height, x, y);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

private:
	size_t frameCounter;
	char* pattern;
	char* dir;
};

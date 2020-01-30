/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
/**
* \file image_video_io.cpp
* \brief Unitary interface implementation for reading either image series or video sequences
* \author Anca Dima
* \version
* \date 11-February-2014
****************************************************************************/

#include "io_image_video.h"
#include <iostream>
#include <stdio.h>


#ifdef __STANDALONE__
#include <printf.h>
#endif

using namespace cv;

CReadVideoSeqData::CReadVideoSeqData(std::string& inputPath, std::string& fileExt, int minIdx, int maxIdx)
: CReadSeqData(inputPath, fileExt, minIdx, maxIdx)
, m_frame_size(cvSize(0, 0))
{
	m_input_video = new cv::VideoCapture(cv::String(inputPath+m_fileExt));//cvCaptureFromFile((m_inputPath + m_fileExt).c_str());

	if (m_input_video == NULL)
	{
		m_minIdx = m_maxIdx = m_crtIdx = 0;

		/* Either the video didn't exist OR it uses a codec OpenCV doesn't support. */
		std::cerr << "Error: Couldn't open video : " << (m_inputPath + m_fileExt).c_str() <<  std::endl;
		return;
	}

	/* This is a hack.  If we don't call this first then getting capture
	* properties (below) won't work right.  This is an OpenCV bug.  We
	* ignore the return value here.  But it's actually a video frame.
	*/
	//cvQueryFrame(m_input_video);
	bool ok = m_input_video->grab();
	m_frame_size.width = (int) m_input_video->get(CAP_PROP_FRAME_WIDTH);
	m_frame_size.height = (int)m_input_video->get(CAP_PROP_FRAME_HEIGHT);

	/* Read the video's frame size out of the AVI. */
	//m_frame_size.height = (int)cvGetCaptureProperty(m_input_video, CV_CAP_PROP_FRAME_HEIGHT);
	//m_frame_size.width = (int)cvGetCaptureProperty(m_input_video, CV_CAP_PROP_FRAME_WIDTH);

	/* Go to the end of the AVI (ie: the fraction is "1") */
	//cvSetCaptureProperty(m_input_video, CV_CAP_PROP_POS_AVI_RATIO, 1.);
	m_input_video->set(CAP_PROP_POS_AVI_RATIO, 1.0);

	/* Now that we're at the end, read the AVI position in frames */
	//m_maxIdx = (int)cvGetCaptureProperty(m_input_video, CV_CAP_PROP_POS_FRAMES) - 1;
	m_maxIdx = (int)m_input_video->get(CAP_PROP_POS_FRAMES) - 1;

	/* Return to the beginning */
	//cvSetCaptureProperty(m_input_video, CV_CAP_PROP_POS_FRAMES, 0.);
	m_input_video->set(CAP_PROP_POS_FRAMES, 0.);
}

int CReadVideoSeqData::GetNextFrame(Mat& nextFrame, int idxToGet)
{

	if (m_input_video == NULL)
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		* doesn't support.
		*/
		std::cerr << "Error: Couldn't open video : " << (m_inputPath + m_fileExt).c_str() <<  std::endl;
		return -1;
	}

	// fetch either the next image or the wanted index
	m_crtIdx = idxToGet < 0 ? m_crtIdx : idxToGet;

	//static IplImage *frame = NULL;
	static Mat frame;

	/* Go to the frame we want. */
	//cvSetCaptureProperty(m_input_video, CV_CAP_PROP_POS_FRAMES, m_crtIdx);
	m_input_video->set(CAP_PROP_POS_FRAMES, (double) m_crtIdx);
	++m_crtIdx;

	/* Get the next frame of the video.
	* IMPORTANT!  cvQueryFrame() always returns a pointer to the _same_ memory location!!
	* So successive calls:
	* frame1 = cvQueryFrame();
	* frame2 = cvQueryFrame();
	* frame3 = cvQueryFrame();
	* will result in (frame1 == frame2 && frame2 == frame3) being true.
	* The solution is to make a copy of the cvQueryFrame() output.
	*/
	//frame = cvQueryFrame(m_input_video);
	bool ok = m_input_video->grab();
	m_input_video->retrieve(frame);

	if (frame.empty())
	{
		/* Why did we get a NULL frame?  We shouldn't be at the end. */
		std::cerr << "Error: The end came sooner than we thought." << std::endl;
		return -1;
	}

	/* Convert whatever the AVI image format is into OpenCV's preferred format.
	*/
	nextFrame = frame.clone(); //cv::cvarrToMat(&frame);
	
	return m_crtIdx;

}

int CReadImgSeqData::GetNextFrame(Mat& nextFrame, int idxToGet)
{
	char nrStr[20];
	if (m_crtIdx > m_maxIdx) {
		//End of data
		return -1;
	}
	m_crtIdx = idxToGet < 0 ? m_crtIdx : idxToGet;

	static long endOfFilePos = 0;


	sprintf(nrStr, "%08d", m_crtIdx);
	std::string stringNr(nrStr);
	m_filePath = m_inputPath + stringNr + m_fileExt;
	
#ifndef __STANDALONE__ // if no file system is present (i.e. bare metal environment), this code cannot be compiled

	FILE *f = fopen(m_filePath.c_str(), "r");
	
	if (f == NULL || allFiles != NULL) { // the image stack does not have the format: <imgName>_00.<ext>, thus search in the directory for *.<ext>
		if (m_crtIdx == 0 && allFiles == NULL) {
			// CAUTION this is only working in Windows!
			if (m_crtIdx == 0) {
				//system("pwd");
#ifdef _WIN32
				std::string cmd = "dir /B /O:N " + m_inputPath + "*" + m_fileExt + " > tmp.txt";
#else
				std::string cmd = "ls " + m_inputPath + "*" + m_fileExt + " > tmp.txt; echo \"\
\">>tmp.txt";				
				//std::string cmd = "ls " + m_inputPath + "*" + m_fileExt + " > tmp.txt";
#endif
				system(cmd.c_str());
			}

			allFiles = fopen("tmp.txt", "rt");
						
			if (allFiles == NULL) {
				std::cerr << "Error could not find the specified path: " << m_inputPath + "*" + m_fileExt << endl;
				return -1;
			}

			fseek(allFiles, 0, SEEK_END);
			endOfFilePos = ftell(allFiles);

			// rewind
			fseek(allFiles, 0, SEEK_SET);
			long crtPos = ftell(allFiles);

			const int maxLineLen = 512;
			char tmpName[maxLineLen];
			/*NO preamble any more
			fgets(tmpName, maxLineLen, allFiles); // jump over the first five lines
			fgets(tmpName, maxLineLen, allFiles); // jump over the first five lines
			fgets(tmpName, maxLineLen, allFiles); // jump over the first five lines
			fgets(tmpName, maxLineLen, allFiles); // jump over the first five lines
			fgets(tmpName, maxLineLen, allFiles); // jump over the first five lines
			if (feof(allFiles)) {
				fclose(allFiles);
			}
			
			char tmp1[50], tmp2[50], tmp3[50], tmp4[50];*/
			// now get the first file name
			//fscanf(allFiles, "%s %s %s %s %s\n", tmp1, tmp2, tmp3, tmp4, &tmpName); 
			fscanf(allFiles, "%s\n", (char*) &tmpName);
			if (feof(allFiles)) {
				fclose(allFiles);
				allFiles = NULL;
				string s(tmpName);
				if (s.length() <= m_fileExt.length()) {
					std::cerr << "Error list of files is empty: tmp.txt" << endl;
					return -1;
				}
			}

			int lastBackslash = m_inputPath.rfind("\\");
			if (lastBackslash > 0) {
				m_inputPath = m_inputPath.substr(0, lastBackslash);				
			}
			else {
				lastBackslash = m_inputPath.rfind("/");
				if (lastBackslash > 0) {
					m_inputPath = m_inputPath.substr(0, lastBackslash);
				}
			}

			//int idx = 0;
			//do {
				m_fileName = tmpName;
#ifdef _WIN32
				m_filePath = m_inputPath + "\\" + std::string(tmpName);
#else
				m_filePath = std::string(tmpName);
#endif
			//} while (idx < m_crtIdx || feof(allFiles));
		}
		else {
			if (allFiles == NULL) {
				return -1;
			}
			char tmpName[512];

			// now get the next file name until end of the list
			long crtPos = ftell(allFiles);
			if (crtPos == endOfFilePos ) {
			//if (feof(allFiles)) {
				fclose(allFiles);
				allFiles = NULL;
				return -1;
			}
			fscanf(allFiles, "%s\n", (char*) &tmpName);
			m_fileName = tmpName;

			if (m_fileName.size() > 90)
			{
				fclose(allFiles);
				allFiles = NULL;
				return -1;
			}
#ifdef _WIN32
			m_filePath = m_inputPath + "\\" + std::string(tmpName);
#else
			m_filePath = std::string(tmpName);
#endif

		}
		if ((f=fopen(m_filePath.c_str(), "r")) == NULL) {
			std::cerr << "Failed to open file: " << m_filePath << endl;
			return -1;
		}
		else {
			fclose(f);
		}

	}
	else {
		fclose(f);
	}
#endif
	
	
	++m_crtIdx;
	nextFrame = imread(m_filePath, m_cvReadFlag);
	if (!nextFrame.data)
	{
		std::cout << "Failed to open the image with OpenCV: " << m_filePath << std::endl;
		return -1;
	}
	return m_crtIdx - 1;
}

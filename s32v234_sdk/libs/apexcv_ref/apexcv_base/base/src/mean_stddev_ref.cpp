/******************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2018 NXP
* All Rights Reserved
*
*******************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
/*!****************************************************************************
*  \file mean_std_ref.cpp
*  \brief Reference code for mean and standard deviation kernels
******************************************************************************/

#include "mean_stddev_ref.h"
#include <math.h>
#include <stdio.h>

void mean_stddev_08u_ref(vsdk::SMat& aSrc, float& aMean, float& aStd)
{
	double fSum = 0;
	double fSqDiffSum = 0;
	uint64_t nNumel = aSrc.rows * aSrc.cols;
	double fMean = 0;
	float _std = 0;

	for (int x = 0; x < aSrc.cols; x++)
	{
		for (int y = 0; y < aSrc.rows; y++)
		{
			double a = (double)aSrc.at<uint8_t>(y, x);
			fSum += a;
		}
	}

	fMean = fSum/nNumel;

	for (int x = 0; x < aSrc.cols; x++)
	{
		for (int y = 0; y < aSrc.rows; y++)
		{
			double a = (double)aSrc.at<uint8_t>(y, x);
			fSqDiffSum += (a - fMean)*(a - fMean);
		}
	}

	aMean = (float)fMean;
	aStd = (float)sqrt(fSqDiffSum/nNumel);
}

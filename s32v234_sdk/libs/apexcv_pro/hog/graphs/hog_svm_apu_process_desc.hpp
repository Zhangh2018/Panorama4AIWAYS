/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2018 NXP
* All Rights Reserved
*
******************************************************************************
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
****************************************************************************/

/*!*********************************************************************************
*  \file
*  \brief ACF process description for tobject detection based on precomputed block's HOG
*  and SVM decision function.
***********************************************************************************/
#ifndef HOGSVMPROCESSDESC_HPP
#define HOGSVMPROCESSDESC_HPP


#include "acf_process_desc_apu.hpp"
#include "hog_svm_graph.hpp"


/*!*********************************************************************************
*  \brief ACF process description for multiplication of HOG descriptor and object's SVM
*  decision function in order to get the detection scores.
*  \see UG-10267-03 ACF User Guide, Section 3.4
***********************************************************************************/
class hog_svm_process_desc : public ACF_Process_Desc_APU
{
public:
  virtual ~hog_svm_process_desc() {}

/*!*********************************************************************************
*  \brief Create the ACF process description.
*
*  In this function we
*     - initialize the process and give it a unique identifier.
*     - set the input chunk size
***********************************************************************************/
  void Create()
  {
    Initialize(mHogSvmGraph, "HOG_SVM");

  }

  /*! \ref hog_graph "ACF graph" of the HOG object detector. */
  hog_svm_graph mHogSvmGraph;
};


#endif /* HOGSVMPROCESSDESC_HPP */


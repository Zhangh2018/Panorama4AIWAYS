/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright 2017 NXP
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

/*****************************************************************************
 *  @file apex_fast.h
 * @brief Apex Host wrapper for fast corner detector algorithm
 ******************************************************************************/

#ifndef APEX_FAST9_H
#define APEX_FAST9_H

#include <sumat.hpp>

#include <apexcv_error_codes.hpp>
#ifdef APEX2_EMULATE
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include <acf_process_apu.h>
#endif

//#define FAST_MAX_LIST_ELEMENTS 8192

namespace apex_fast9
{
/*==================================================================================================
*                                             ENUMS
==================================================================================================*/

/****************************************************************************
 * @brief The available FAST ACF processes.
 ***************************************************************************/
enum FAST9_PROCESS_TYPE_E
{
  FAST9_NONE        = 0, /*<@brief No ACF process. */
  FAST9_PROCESS_IMG = 1, /*<@brief ACF process that runs FAST and outputs an
                             image where pixel values can be either
                             0 (non-corner) or 1 (corner). */
  FAST9_PROCESS_IMG_NMS  /*<@brief FAST_PROCESS_IMG that also implements
                             non maximum supression. */
};

/***************************************************************************
 * @brief APEX FAST Corner Detection class which creates an APEX Process.
 *
 * @details  Fast9 is the host-ACF interface for creating, initializing,
 *  executing and releasing the APEX graph which detects Fast9 corners.
 ****************************************************************************/
class Fast9
{
public:
  /***************************************************************************
   * @brief Default constructor.
   ****************************************************************************/
  Fast9();

  /***************************************************************************
   * @brief Destructor.
   ****************************************************************************/
  ~Fast9();

  /***************************************************************************
   * @brief Initializes the process parameters and allocates ACF resources.
   * @param[in]  aSrc         source image.
   * @param[in]  acThreshold  threshold for the FAST algorithm.
   * @param[in]  acNonMaxSupp enable or not non maximum suppression.
   * @param[in]  aRoi Region  of interest where to search for corners.
   * @param[out] aDst Output  buffer. Depending on the value of acOutIsList
   *             it can be:
   *                  - an image where pixel values are:
   *                      - 0 for non-corner
   *                      - 8bit score for the corner
   * @return Error code for the initialization - see apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT Initialize(vsdk::SUMat &aDst,
                               vsdk::SUMat &aSrc,
                               const int   acThreshold,
                               const bool  acNonMaxSupp,
                               vsdk::Rect &aRoi);

  /***************************************************************************
   * @brief Launches the process for the FAST algorithm and waits for
   *completion.
   * @return Error code for the execution - see apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT Process();

  /***************************************************************************
   * @brief Launches the process for the FAST algorithm and immediately
   *returns, allowing for other tasks to be run on the host. To wait for process
   *completion call ProcessWait(). Make sure that every call to
   *ProcessNoPolling() is paired with a call to ProcessWait().
   * @return Error code for the execution - see apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT ProcessNoPolling();

  /***************************************************************************
   * @brief Waits for a FAST process previously launched with
   *ProcessNoPolling() to finish.
   * @return Error code for the execution - see apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT ProcessWait();

  /***************************************************************************
   * @brief Reinitializes the ACF process graph connections.
   *  Used for reconnecting the input and output buffers. This operation is
   *  needed only if the connected Input/Outputs are changed. This operation is
   *  not needed if only the data values change, without any change of pointers
   *  or sizes.
   * @return Error code for ACF graph process reinitialization - see
   *  apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT ReconnectIO(vsdk::SUMat &aDst,
                                vsdk::SUMat &aSrc,
                                const int   acThreshold,
                                const bool  acNms);

  /***************************************************************************
   * @brief SelectApuConfiguration
   *  Selects APU and APEX configuration.
   * @return Error code for updating the APU configuration - see
   *apexcv_error_codes.hpp
   ****************************************************************************/
  APEXCV_LIB_RESULT SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                           int32_t     aApexId);

  /***************************************************************************
   * @brief Returns the number of features detected by FAST
   *  If the output is not a list the latency is very high !
   ****************************************************************************/
  int GetNrOfFeatures();

private:
  vsdk::SUMat *mpOut; /*< @brief Reference to the output image buffer */

  unsigned int mInit; /*< @brief Indicates if it has been initialized or not */
  bool         mNonMaxSupression; /*< @brief Indicates if NMS is on or off */
  int          mProcessType;      /*< @brief Indicates the process type used */
  int          mImgWidth;         /*< @brief Image width */
  int          mImgHeight;        /*< @brief Image height */

  ACF_APU_CFG mApuConfig; /*< @brief APU configuration for the ACF process */
  int32_t mApexId; /*< @brief APEX core to be used for running the process */

  vsdk::SUMat mThreshold; /*< @brief Contains the minimal threshold for corner
                            detection */

  void *mpProcess; /*< @brief Pointer to FAST processes */
};
} /* namespace apexc_fast9 */

#endif /* APEX_FAST9_H */

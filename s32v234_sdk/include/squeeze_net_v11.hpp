/***********************************************************************************
*
*  NXP Confidential Proprietary
*
*  Copyright (c) 2015 NXP Semiconductor;
*  All Rights Reserved
*
***********************************************************************************
*
*  THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
*  THE POSSIBILITY OF SUCH DAMAGE.
*
**********************************************************************************/

/*!*********************************************************************************
*  @file squeeze_v11_net.hpp
*  @brief ApexCV SqueezeNet V1.1.
**********************************************************************************/

#ifndef __SQUEEZE_V11_NET_HPP__
#define __SQUEEZE_V11_NET_HPP__

#define APEXCV_SQUEEZENET_V11_WEIGHT_MODEL_BYTES 1235574

#include <vector>
#include <string>
#include <cstdint>
#include <apexcv_host_base_class.h>
#include "apexdnn_sn_public.hpp"

#define WEIGHT_BW 64

#define APEXCV_SQUEEZENET_V11_COLLECT_STATISTICS 0

//bw: block width;  ih: image height; bd: block depth;
//bh: block height; bs: block stride; ts: tile stride;


namespace apexcv {

	/*!*********************************************************************************
	*   \brief ApexCV SqueezeNet V1.1
	*
	*  apex::SqueezeV10Net is the Host-ACF interface for creating, initializing,
	*  executing and releasing the SqueezeV11Net on Apex.
	***********************************************************************************/

	class SqueezeV11Net: public ApexcvHostBaseBaseClass {

	public:

		/*!*********************************************************************************
		*   \brief Default Constructor.
		*
		*  Allocate resources for the ACF processes.
		***********************************************************************************/
		SqueezeV11Net();

		/*!*********************************************************************************
		*  \brief Destructor.
		*   Release resources for the ACF processes.
		*********************************************************************************/
		~SqueezeV11Net();

		/*!*********************************************************************************
		*   \brief Initialization
		*
		*  Initializes the intermediate buffers needed for the process, and initializes the
		*  ACF processes.
		* ***********************************************************************************/
		int Initialize(int8_t *pWeight                      /*!< 8-bit buffer, 1449984 bytes to hold the SqueezeNet V1.0 quantized weights coefficients */
			);

		/*!*********************************************************************************
		*  \brief Process
		*  \return Error code for the ACF execution (zero on success).
		***********************************************************************************/
		int Process(uint16_t* pTop5                      /*!< 16-bit buffer, 10 elements as [label index, score] x 5 to hold Top-5 classes */
			);

		/*!*********************************************************************************
		*  \brief SelectApuConfiguration
		*  Select APU and APEX configuration
		*  \return Error code .
		***********************************************************************************/
		int SelectApuConfiguration(ACF_APU_CFG lApuConfig   /*!< APU configuration */,
			int32_t lApexId          /*!< APEX configuration */
			);

		apexdnnTensorDescriptor* net_input_tensor_;
		apexdnnTensorDescriptor* net_output_tensor_;


	private:
		apexdnnNet* net_;
		apexdnnWorkSpace* workspace_;

		ACF_APU_CFG mApuConfig;
		int32_t     mApexId;
		int32_t     mNumOfApuProcesses;
	};

} /* namespace apexcv */

#endif /* __SQUEEZE_V1.1_NET_HPP__ */

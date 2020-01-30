#ifndef _ACF_PROCESS_APU_ELTMULCRED_MODULE
#define _ACF_PROCESS_APU_ELTMULCRED_MODULE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <ELTMULCRED_MODULE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_ELTMULCRED_MODULE[] = {{1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_ELTMULCRED_MODULE[] = {{0, 0}};

static acf_scenario gScenarioArray_ELTMULCRED_MODULE[] = {
{0, 0, 0, 272, 0, gScenarioBufferData0_ELTMULCRED_MODULE, 96, gScenarioKernelData0_ELTMULCRED_MODULE, 4}
};

static acf_scenario_list gScenarioList_ELTMULCRED_MODULE = {
1, //number of scenarios
gScenarioArray_ELTMULCRED_MODULE};
//**************************************************************

class ELTMULCRED_MODULE : public ACF_Process_APU
{

public:
   ELTMULCRED_MODULE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("ELTMULCRED_MODULE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("ELTMULCRED_MODULE",
                                        ELTMULCRED_MODULE_LOAD_SEGMENTS,
                                        ELTMULCRED_MODULE_LOAD_PMEM, ELTMULCRED_MODULE_LOAD_PMEM_SIZE,
                                        ELTMULCRED_MODULE_LOAD_DMEM, ELTMULCRED_MODULE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(ELTMULCRED_MODULE_APEX_LOG_BUFFER, ELTMULCRED_MODULE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("ELTMULCRED_MODULE_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("ELTMULCRED_MODULE_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("ELTMULCRED_MODULE_INPUT_BIAS", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("ELTMULCRED_MODULE_INPUT_PARAMS", icp::DATATYPE_32S, 1, 1, 32, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("ELTMULCRED_MODULE_OUTPUT", icp::DATATYPE_08S, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_ELTMULCRED_MODULE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_ELTMULCRED_MODULE

#ifndef _ACF_PROCESS_APU_CONV3X3MPS1_MODULE
#define _ACF_PROCESS_APU_CONV3X3MPS1_MODULE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CONV3X3MPS1_MODULE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CONV3X3MPS1_MODULE[] = {{2, 4, 3, 0, {1, 1, 0, 0}}, {12, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {64, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CONV3X3MPS1_MODULE[] = {{1, 1}};

static acf_scenario gScenarioArray_CONV3X3MPS1_MODULE[] = {
{0, 0, 76, 464, 2, gScenarioBufferData0_CONV3X3MPS1_MODULE, 112, gScenarioKernelData0_CONV3X3MPS1_MODULE, 4}
};

static acf_scenario_list gScenarioList_CONV3X3MPS1_MODULE = {
1, //number of scenarios
gScenarioArray_CONV3X3MPS1_MODULE};
//**************************************************************

class CONV3X3MPS1_MODULE : public ACF_Process_APU
{

public:
   CONV3X3MPS1_MODULE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CONV3X3MPS1_MODULE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CONV3X3MPS1_MODULE",
                                        CONV3X3MPS1_MODULE_LOAD_SEGMENTS,
                                        CONV3X3MPS1_MODULE_LOAD_PMEM, CONV3X3MPS1_MODULE_LOAD_PMEM_SIZE,
                                        CONV3X3MPS1_MODULE_LOAD_DMEM, CONV3X3MPS1_MODULE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CONV3X3MPS1_MODULE_APEX_LOG_BUFFER, CONV3X3MPS1_MODULE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("CONV3X3MPS1_MODULE_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 2, 4, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("CONV3X3MPS1_MODULE_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 1, 0, 0, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("CONV3X3MPS1_MODULE_INPUT_PARAMS", icp::DATATYPE_32S, 1, 1, 32, 1, 0, 1, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("CONV3X3MPS1_MODULE_OUTPUT_S1", icp::DATATYPE_08S, 1, 1, 2, 1, 1, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CONV3X3MPS1_MODULE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CONV3X3MPS1_MODULE

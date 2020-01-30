#ifndef _ACF_PROCESS_APU_E1E3MP_MODULE
#define _ACF_PROCESS_APU_E1E3MP_MODULE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <E1E3MP_MODULE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_E1E3MP_MODULE[] = {{2, 4, 3, 0, {1, 1, 0, 0}}, {12, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {64, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_E1E3MP_MODULE[] = {{1, 0}};

static acf_scenario gScenarioArray_E1E3MP_MODULE[] = {
{0, 0, 80, 464, 1, gScenarioBufferData0_E1E3MP_MODULE, 128, gScenarioKernelData0_E1E3MP_MODULE, 4}
};

static acf_scenario_list gScenarioList_E1E3MP_MODULE = {
1, //number of scenarios
gScenarioArray_E1E3MP_MODULE};
//**************************************************************

class E1E3MP_MODULE : public ACF_Process_APU
{

public:
   E1E3MP_MODULE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("E1E3MP_MODULE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("E1E3MP_MODULE",
                                        E1E3MP_MODULE_LOAD_SEGMENTS,
                                        E1E3MP_MODULE_LOAD_PMEM, E1E3MP_MODULE_LOAD_PMEM_SIZE,
                                        E1E3MP_MODULE_LOAD_DMEM, E1E3MP_MODULE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(E1E3MP_MODULE_APEX_LOG_BUFFER, E1E3MP_MODULE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("E1E3MP_MODULE_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 2, 4, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("E1E3MP_MODULE_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 1, 0, 0, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("E1E3MP_MODULE_INPUT_PARAMS", icp::DATATYPE_32S, 1, 1, 32, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("E1E3MP_MODULE_OUTPUT_MP1", icp::DATATYPE_08S, 1, 1, 2, 1, 1, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("E1E3MP_MODULE_OUTPUT_MP3", icp::DATATYPE_08S, 1, 1, 2, 1, 1, 0, 0, 0, 1, 0, 5, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_E1E3MP_MODULE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_E1E3MP_MODULE

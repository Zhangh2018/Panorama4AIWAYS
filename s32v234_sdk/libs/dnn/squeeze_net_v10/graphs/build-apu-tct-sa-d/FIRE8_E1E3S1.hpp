#ifndef _ACF_PROCESS_APU_FIRE8_E1E3S1
#define _ACF_PROCESS_APU_FIRE8_E1E3S1

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE8_E1E3S1_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE8_E1E3S1[] = {{1360, 3, 3, 0, {1, 1, 0, 0}}, {28, 1, 1, 1, {0, 0, 0, 0}}, {84, 1, 1, 1, {0, 0, 0, 0}}, {84, 1, 1, 1, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {64, 31, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE8_E1E3S1[] = {{1, 0}};

static acf_scenario gScenarioArray_FIRE8_E1E3S1[] = {
{0, 0, 2432, 20416, 1, gScenarioBufferData0_FIRE8_E1E3S1, 112, gScenarioKernelData0_FIRE8_E1E3S1, 4}
};

static acf_scenario_list gScenarioList_FIRE8_E1E3S1 = {
1, //number of scenarios
gScenarioArray_FIRE8_E1E3S1};
//**************************************************************

class FIRE8_E1E3S1 : public ACF_Process_APU
{

public:
   FIRE8_E1E3S1(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE8_E1E3S1");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE8_E1E3S1",
                                        FIRE8_E1E3S1_LOAD_SEGMENTS,
                                        FIRE8_E1E3S1_LOAD_PMEM, FIRE8_E1E3S1_LOAD_PMEM_SIZE,
                                        FIRE8_E1E3S1_LOAD_DMEM, FIRE8_E1E3S1_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE8_E1E3S1_APEX_LOG_BUFFER, FIRE8_E1E3S1_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE8_E1E3S1_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 1360, 3, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("FIRE8_E1E3S1_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 31, 0, 0, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("FIRE8_E1E3S1_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("FIRE8_E1E3S1_OUTPUT_S1", icp::DATATYPE_08S, 1, 1, 28, 3, 1, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE8_E1E3S1);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE8_E1E3S1

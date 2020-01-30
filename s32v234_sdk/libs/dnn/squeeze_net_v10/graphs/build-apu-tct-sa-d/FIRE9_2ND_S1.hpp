#ifndef _ACF_PROCESS_APU_FIRE9_2ND_S1
#define _ACF_PROCESS_APU_FIRE9_2ND_S1

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE9_2ND_S1_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE9_2ND_S1[] = {{7184, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 1, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {64, 9, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE9_2ND_S1[] = {{0, 0}};

static acf_scenario gScenarioArray_FIRE9_2ND_S1[] = {
{0, 0, 660, 14384, 0, gScenarioBufferData0_FIRE9_2ND_S1, 80, gScenarioKernelData0_FIRE9_2ND_S1, 4}
};

static acf_scenario_list gScenarioList_FIRE9_2ND_S1 = {
1, //number of scenarios
gScenarioArray_FIRE9_2ND_S1};
//**************************************************************

class FIRE9_2ND_S1 : public ACF_Process_APU
{

public:
   FIRE9_2ND_S1(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE9_2ND_S1");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE9_2ND_S1",
                                        FIRE9_2ND_S1_LOAD_SEGMENTS,
                                        FIRE9_2ND_S1_LOAD_PMEM, FIRE9_2ND_S1_LOAD_PMEM_SIZE,
                                        FIRE9_2ND_S1_LOAD_DMEM, FIRE9_2ND_S1_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE9_2ND_S1_APEX_LOG_BUFFER, FIRE9_2ND_S1_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE9_2ND_S1_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 7184, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("FIRE9_2ND_S1_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 9, 0, 0, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("FIRE9_2ND_S1_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("FIRE9_2ND_S1_OUTPUT_S1", icp::DATATYPE_08S, 1, 1, 14, 1, 1, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE9_2ND_S1);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE9_2ND_S1

#ifndef _ACF_PROCESS_APU_FIRE2_CONV1MPS1
#define _ACF_PROCESS_APU_FIRE2_CONV1MPS1

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE2_CONV1MPS1_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE2_CONV1MPS1[] = {{700, 12, 3, 0, {2, 3, 0, 0}}, {112, 1, 1, 1, {0, 0, 0, 0}}, {9, 1, 1, 1, {0, 0, 0, 0}}, {222, 3, 1, 1, {0, 0, 0, 0}}, {112, 1, 1, 1, {0, 0, 0, 0}}, {56, 3, 3, 2, {0, 2, 0, 0}}, {64, 7, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE2_CONV1MPS1[] = {{1, 1}};

static acf_scenario gScenarioArray_FIRE2_CONV1MPS1[] = {
{0, 0, 2292, 35968, 2, gScenarioBufferData0_FIRE2_CONV1MPS1, 128, gScenarioKernelData0_FIRE2_CONV1MPS1, 4}
};

static acf_scenario_list gScenarioList_FIRE2_CONV1MPS1 = {
1, //number of scenarios
gScenarioArray_FIRE2_CONV1MPS1};
//**************************************************************

class FIRE2_CONV1MPS1 : public ACF_Process_APU
{

public:
   FIRE2_CONV1MPS1(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE2_CONV1MPS1");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE2_CONV1MPS1",
                                        FIRE2_CONV1MPS1_LOAD_SEGMENTS,
                                        FIRE2_CONV1MPS1_LOAD_PMEM, FIRE2_CONV1MPS1_LOAD_PMEM_SIZE,
                                        FIRE2_CONV1MPS1_LOAD_DMEM, FIRE2_CONV1MPS1_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE2_CONV1MPS1_APEX_LOG_BUFFER, FIRE2_CONV1MPS1_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE2_CONV1MPS1_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 700, 12, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("FIRE2_CONV1MPS1_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 7, 0, 0, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("FIRE2_CONV1MPS1_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("FIRE2_CONV1MPS1_OUTPUT_S1", icp::DATATYPE_08S, 1, 1, 56, 3, 1, 0, 0, 0, 1, 0, 5, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE2_CONV1MPS1);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE2_CONV1MPS1

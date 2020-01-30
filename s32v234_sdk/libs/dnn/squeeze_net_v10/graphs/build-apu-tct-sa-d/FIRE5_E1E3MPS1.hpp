#ifndef _ACF_PROCESS_APU_FIRE5_E1E3MPS1
#define _ACF_PROCESS_APU_FIRE5_E1E3MPS1

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE5_E1E3MPS1_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE5_E1E3MPS1[] = {{1808, 4, 3, 0, {1, 1, 0, 0}}, {56, 1, 1, 1, {0, 0, 0, 0}}, {9, 1, 1, 1, {0, 0, 0, 0}}, {112, 3, 1, 1, {0, 0, 0, 0}}, {112, 3, 1, 1, {0, 0, 0, 0}}, {56, 1, 1, 1, {0, 0, 0, 0}}, {56, 1, 1, 1, {0, 0, 0, 0}}, {28, 2, 3, 2, {0, 1, 0, 0}}, {64, 16, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE5_E1E3MPS1[] = {{1, 1}};

static acf_scenario gScenarioArray_FIRE5_E1E3MPS1[] = {
{0, 0, 2228, 32608, 2, gScenarioBufferData0_FIRE5_E1E3MPS1, 160, gScenarioKernelData0_FIRE5_E1E3MPS1, 4}
};

static acf_scenario_list gScenarioList_FIRE5_E1E3MPS1 = {
1, //number of scenarios
gScenarioArray_FIRE5_E1E3MPS1};
//**************************************************************

class FIRE5_E1E3MPS1 : public ACF_Process_APU
{

public:
   FIRE5_E1E3MPS1(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE5_E1E3MPS1");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE5_E1E3MPS1",
                                        FIRE5_E1E3MPS1_LOAD_SEGMENTS,
                                        FIRE5_E1E3MPS1_LOAD_PMEM, FIRE5_E1E3MPS1_LOAD_PMEM_SIZE,
                                        FIRE5_E1E3MPS1_LOAD_DMEM, FIRE5_E1E3MPS1_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE5_E1E3MPS1_APEX_LOG_BUFFER, FIRE5_E1E3MPS1_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE5_E1E3MPS1_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 1808, 4, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("FIRE5_E1E3MPS1_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 16, 0, 0, 1, 0, 1, 0, 8, 0, 0, 0, 0, 0);
         AddPort("FIRE5_E1E3MPS1_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("FIRE5_E1E3MPS1_OUTPUT_S1", icp::DATATYPE_08S, 1, 1, 28, 2, 1, 0, 0, 0, 1, 0, 7, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE5_E1E3MPS1);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE5_E1E3MPS1

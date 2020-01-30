#ifndef _ACF_PROCESS_APU_FIRE10_2ND_CONV10AP_BOTTOM
#define _ACF_PROCESS_APU_FIRE10_2ND_CONV10AP_BOTTOM

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE10_2ND_CONV10AP_BOTTOM_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE10_2ND_CONV10AP_BOTTOM[] = {{7168, 7, 1, 0, {0, 0, 0, 0}}, {14, 1, 1, 0, {0, 0, 0, 0}}, {14, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {64, 9, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE10_2ND_CONV10AP_BOTTOM[] = {{0, 0}};

static acf_scenario gScenarioArray_FIRE10_2ND_CONV10AP_BOTTOM[] = {
{0, 0, 1232, 50192, 0, gScenarioBufferData0_FIRE10_2ND_CONV10AP_BOTTOM, 112, gScenarioKernelData0_FIRE10_2ND_CONV10AP_BOTTOM, 4}
};

static acf_scenario_list gScenarioList_FIRE10_2ND_CONV10AP_BOTTOM = {
1, //number of scenarios
gScenarioArray_FIRE10_2ND_CONV10AP_BOTTOM};
//**************************************************************

class FIRE10_2ND_CONV10AP_BOTTOM : public ACF_Process_APU
{

public:
   FIRE10_2ND_CONV10AP_BOTTOM(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE10_2ND_CONV10AP_BOTTOM");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE10_2ND_CONV10AP_BOTTOM",
                                        FIRE10_2ND_CONV10AP_BOTTOM_LOAD_SEGMENTS,
                                        FIRE10_2ND_CONV10AP_BOTTOM_LOAD_PMEM, FIRE10_2ND_CONV10AP_BOTTOM_LOAD_PMEM_SIZE,
                                        FIRE10_2ND_CONV10AP_BOTTOM_LOAD_DMEM, FIRE10_2ND_CONV10AP_BOTTOM_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE10_2ND_CONV10AP_BOTTOM_APEX_LOG_BUFFER, FIRE10_2ND_CONV10AP_BOTTOM_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE10_2ND_CONV10AP_BOTTOM_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 9, 0, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("FIRE10_2ND_CONV10AP_BOTTOM_INPUT_ACCM", icp::DATATYPE_16S, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("FIRE10_2ND_CONV10AP_BOTTOM_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 7168, 7, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("FIRE10_2ND_CONV10AP_BOTTOM_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("FIRE10_2ND_CONV10AP_BOTTOM_OUTPUT_AP", icp::DATATYPE_16S, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE10_2ND_CONV10AP_BOTTOM);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE10_2ND_CONV10AP_BOTTOM

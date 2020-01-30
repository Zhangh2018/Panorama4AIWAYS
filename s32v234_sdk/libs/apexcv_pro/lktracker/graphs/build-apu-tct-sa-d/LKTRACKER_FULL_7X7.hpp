#ifndef _ACF_PROCESS_APU_LKTRACKER_FULL_7X7
#define _ACF_PROCESS_APU_LKTRACKER_FULL_7X7

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LKTRACKER_FULL_7X7_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LKTRACKER_FULL_7X7[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 9, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LKTRACKER_FULL_7X7[] = {{0, 0}, {0, 0}, {0, 0}};

static acf_scenario gScenarioArray_LKTRACKER_FULL_7X7[] = {
{0, 0, 1636, 16, 0, gScenarioBufferData0_LKTRACKER_FULL_7X7, 176, gScenarioKernelData0_LKTRACKER_FULL_7X7, 12}
};

static acf_scenario_list gScenarioList_LKTRACKER_FULL_7X7 = {
1, //number of scenarios
gScenarioArray_LKTRACKER_FULL_7X7};
//**************************************************************

class LKTRACKER_FULL_7X7 : public ACF_Process_APU
{

public:
   LKTRACKER_FULL_7X7(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LKTRACKER_FULL_7X7");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LKTRACKER_FULL_7X7",
                                        LKTRACKER_FULL_7X7_LOAD_SEGMENTS,
                                        LKTRACKER_FULL_7X7_LOAD_PMEM, LKTRACKER_FULL_7X7_LOAD_PMEM_SIZE,
                                        LKTRACKER_FULL_7X7_LOAD_DMEM, LKTRACKER_FULL_7X7_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LKTRACKER_FULL_7X7_APEX_LOG_BUFFER, LKTRACKER_FULL_7X7_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_TEMPLATE", icp::DATATYPE_08U, 1, 1, 12, 10, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_PATCH", icp::DATATYPE_08U, 1, 1, 20, 20, 0, 0, 0, 1, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("INPUT_COOR_XY0", icp::DATATYPE_32S, 4, 1, 1, 1, 0, 0, 0, 0, 1, 0, 8, 0, 0, 0, 0, 0);
         AddPort("INPUT_COOR_XY1", icp::DATATYPE_32S, 4, 1, 1, 1, 0, 0, 0, 0, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("PARAMS", icp::DATATYPE_08S, 1, 1, 12, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_COOR_XY1", icp::DATATYPE_32S, 4, 1, 1, 1, 1, 0, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LKTRACKER_FULL_7X7);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LKTRACKER_FULL_7X7

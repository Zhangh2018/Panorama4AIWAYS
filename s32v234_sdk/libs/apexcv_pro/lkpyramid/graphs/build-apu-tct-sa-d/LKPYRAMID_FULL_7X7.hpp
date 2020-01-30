#ifndef _ACF_PROCESS_APU_LKPYRAMID_FULL_7X7
#define _ACF_PROCESS_APU_LKPYRAMID_FULL_7X7

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LKPYRAMID_FULL_7X7_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LKPYRAMID_FULL_7X7[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 9, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {8, 7, 1, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LKPYRAMID_FULL_7X7[] = {{0, 0}, {0, 0}, {0, 0}};

static acf_scenario gScenarioArray_LKPYRAMID_FULL_7X7[] = {
{0, 0, 1692, 16, 0, gScenarioBufferData0_LKPYRAMID_FULL_7X7, 240, gScenarioKernelData0_LKPYRAMID_FULL_7X7, 12}
};

static acf_scenario_list gScenarioList_LKPYRAMID_FULL_7X7 = {
1, //number of scenarios
gScenarioArray_LKPYRAMID_FULL_7X7};
//**************************************************************

class LKPYRAMID_FULL_7X7 : public ACF_Process_APU
{

public:
   LKPYRAMID_FULL_7X7(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LKPYRAMID_FULL_7X7");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LKPYRAMID_FULL_7X7",
                                        LKPYRAMID_FULL_7X7_LOAD_SEGMENTS,
                                        LKPYRAMID_FULL_7X7_LOAD_PMEM, LKPYRAMID_FULL_7X7_LOAD_PMEM_SIZE,
                                        LKPYRAMID_FULL_7X7_LOAD_DMEM, LKPYRAMID_FULL_7X7_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LKPYRAMID_FULL_7X7_APEX_LOG_BUFFER, LKPYRAMID_FULL_7X7_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_TEMPLATE", icp::DATATYPE_08U, 1, 1, 12, 10, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_PATCH", icp::DATATYPE_08U, 1, 1, 20, 20, 0, 0, 0, 1, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("INPUT_COOR_XY0", icp::DATATYPE_32S, 4, 1, 1, 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("INPUT_COOR_XY1", icp::DATATYPE_32S, 4, 1, 1, 1, 0, 0, 0, 0, 1, 0, 12, 0, 0, 0, 0, 0);
         AddPort("INPUT_OFFSET_XY1", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 13, 0, 0, 0, 0, 0);
         AddPort("TEMPL_COOR_NEXT", icp::DATATYPE_32S, 4, 1, 1, 1, 0, 0, 0, 0, 1, 0, 14, 0, 0, 0, 0, 0);
         AddPort("PARAMS", icp::DATATYPE_08S, 1, 1, 8, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_COOR_XY1", icp::DATATYPE_32S, 4, 1, 1, 1, 1, 0, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_OFFSET_XY1", icp::DATATYPE_32S, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_TEMPL_OFFSET_NEXT", icp::DATATYPE_32S, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 5, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LKPYRAMID_FULL_7X7);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LKPYRAMID_FULL_7X7

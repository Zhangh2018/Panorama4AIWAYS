#ifndef _ACF_PROCESS_APU_REMAP
#define _ACF_PROCESS_APU_REMAP

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <REMAP_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_REMAP[] = {{8, 9, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_REMAP[] = {{0, 0}};

static acf_scenario gScenarioArray_REMAP[] = {
{0, 0, 1376, 0, 0, gScenarioBufferData0_REMAP, 64, gScenarioKernelData0_REMAP, 4}
};

static acf_scenario_list gScenarioList_REMAP = {
1, //number of scenarios
gScenarioArray_REMAP};
//**************************************************************

class REMAP : public ACF_Process_APU
{

public:
   REMAP(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("REMAP");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("REMAP",
                                        REMAP_LOAD_SEGMENTS,
                                        REMAP_LOAD_PMEM, REMAP_LOAD_PMEM_SIZE,
                                        REMAP_LOAD_DMEM, REMAP_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(REMAP_APEX_LOG_BUFFER, REMAP_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("SRC_IMG", icp::DATATYPE_32U, 1, 1, 8, 9, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("LOCAL_OFFSET", icp::DATATYPE_16U, 1, 1, 10, 5, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("DELTA", icp::DATATYPE_08U, 2, 1, 10, 5, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("DST_IMG", icp::DATATYPE_32U, 1, 1, 10, 5, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_REMAP);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_REMAP

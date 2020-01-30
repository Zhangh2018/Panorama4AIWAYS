#ifndef _ACF_PROCESS_APU_APU_ORB_RBRIEF
#define _ACF_PROCESS_APU_APU_ORB_RBRIEF

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <APU_ORB_RBRIEF_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_APU_ORB_RBRIEF[] = {{40, 40, 2, 0, {0, 0, 0, 0}}, {64, 1, 1, 0, {0, 0, 0, 0}}, {4096, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {2048, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_APU_ORB_RBRIEF[] = {{0, 0}};

static acf_scenario gScenarioArray_APU_ORB_RBRIEF[] = {
{0, 0, 3268, 10288, 0, gScenarioBufferData0_APU_ORB_RBRIEF, 128, gScenarioKernelData0_APU_ORB_RBRIEF, 4}
};

static acf_scenario_list gScenarioList_APU_ORB_RBRIEF = {
1, //number of scenarios
gScenarioArray_APU_ORB_RBRIEF};
//**************************************************************

class APU_ORB_RBRIEF : public ACF_Process_APU
{

public:
   APU_ORB_RBRIEF(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("APU_ORB_RBRIEF");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("APU_ORB_RBRIEF",
                                        APU_ORB_RBRIEF_LOAD_SEGMENTS,
                                        APU_ORB_RBRIEF_LOAD_PMEM, APU_ORB_RBRIEF_LOAD_PMEM_SIZE,
                                        APU_ORB_RBRIEF_LOAD_DMEM, APU_ORB_RBRIEF_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(APU_ORB_RBRIEF_APEX_LOG_BUFFER, APU_ORB_RBRIEF_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 40, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("ORIENTATION", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("BIT_PATTERN", icp::DATATYPE_08S, 1, 1, 2048, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("DESCR_SIZE_B", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("PATCH_SIZE", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("COUNT_DESCR", icp::DATATYPE_32U, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 64, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_APU_ORB_RBRIEF);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_APU_ORB_RBRIEF

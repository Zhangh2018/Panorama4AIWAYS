#ifndef _ACF_PROCESS_APU_LBP_EXTRACT_DESCRIPTOR
#define _ACF_PROCESS_APU_LBP_EXTRACT_DESCRIPTOR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LBP_EXTRACT_DESCRIPTOR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LBP_EXTRACT_DESCRIPTOR[] = {{32, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LBP_EXTRACT_DESCRIPTOR[] = {{0, 0}};

static acf_scenario gScenarioArray_LBP_EXTRACT_DESCRIPTOR[] = {
{0, 0, 2560, 16, 0, gScenarioBufferData0_LBP_EXTRACT_DESCRIPTOR, 48, gScenarioKernelData0_LBP_EXTRACT_DESCRIPTOR, 4}
};

static acf_scenario_list gScenarioList_LBP_EXTRACT_DESCRIPTOR = {
1, //number of scenarios
gScenarioArray_LBP_EXTRACT_DESCRIPTOR};
//**************************************************************

class LBP_EXTRACT_DESCRIPTOR : public ACF_Process_APU
{

public:
   LBP_EXTRACT_DESCRIPTOR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LBP_EXTRACT_DESCRIPTOR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LBP_EXTRACT_DESCRIPTOR",
                                        LBP_EXTRACT_DESCRIPTOR_LOAD_SEGMENTS,
                                        LBP_EXTRACT_DESCRIPTOR_LOAD_PMEM, LBP_EXTRACT_DESCRIPTOR_LOAD_PMEM_SIZE,
                                        LBP_EXTRACT_DESCRIPTOR_LOAD_DMEM, LBP_EXTRACT_DESCRIPTOR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LBP_EXTRACT_DESCRIPTOR_APEX_LOG_BUFFER, LBP_EXTRACT_DESCRIPTOR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 32, 32, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("PROCESS_SIZE", icp::DATATYPE_16U, 2, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 128, 2, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LBP_EXTRACT_DESCRIPTOR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LBP_EXTRACT_DESCRIPTOR

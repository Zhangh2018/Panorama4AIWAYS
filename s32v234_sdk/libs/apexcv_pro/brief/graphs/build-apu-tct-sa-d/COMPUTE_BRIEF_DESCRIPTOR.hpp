#ifndef _ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR
#define _ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <COMPUTE_BRIEF_DESCRIPTOR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_COMPUTE_BRIEF_DESCRIPTOR[] = {{36, 36, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {64, 1, 1, 0, {0, 0, 0, 0}}, {4096, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2048, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_COMPUTE_BRIEF_DESCRIPTOR[] = {{0, 0}};

static acf_scenario gScenarioArray_COMPUTE_BRIEF_DESCRIPTOR[] = {
{0, 0, 2656, 16512, 0, gScenarioBufferData0_COMPUTE_BRIEF_DESCRIPTOR, 192, gScenarioKernelData0_COMPUTE_BRIEF_DESCRIPTOR, 4}
};

static acf_scenario_list gScenarioList_COMPUTE_BRIEF_DESCRIPTOR = {
1, //number of scenarios
gScenarioArray_COMPUTE_BRIEF_DESCRIPTOR};
//**************************************************************

class COMPUTE_BRIEF_DESCRIPTOR : public ACF_Process_APU
{

public:
   COMPUTE_BRIEF_DESCRIPTOR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("COMPUTE_BRIEF_DESCRIPTOR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("COMPUTE_BRIEF_DESCRIPTOR",
                                        COMPUTE_BRIEF_DESCRIPTOR_LOAD_SEGMENTS,
                                        COMPUTE_BRIEF_DESCRIPTOR_LOAD_PMEM, COMPUTE_BRIEF_DESCRIPTOR_LOAD_PMEM_SIZE,
                                        COMPUTE_BRIEF_DESCRIPTOR_LOAD_DMEM, COMPUTE_BRIEF_DESCRIPTOR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(COMPUTE_BRIEF_DESCRIPTOR_APEX_LOG_BUFFER, COMPUTE_BRIEF_DESCRIPTOR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 36, 36, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("FILTER_TYPE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("SMPL_PACKET", icp::DATATYPE_32U, 1, 1, 2048, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("NR_PACKETS_UL", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("NR_PACKETS_UR", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("NR_PACKETS_LL", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 8, 0, 0, 0, 0, 0);
         AddPort("NR_PACKETS_LR", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("PATCH_SIZE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("DESC_SIZE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("COUNT", icp::DATATYPE_32U, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0);
         AddPort("DESCRIPTOR_OUT", icp::DATATYPE_08U, 1, 1, 64, 1, 1, 0, 0, 0, 1, 1, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_COMPUTE_BRIEF_DESCRIPTOR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR

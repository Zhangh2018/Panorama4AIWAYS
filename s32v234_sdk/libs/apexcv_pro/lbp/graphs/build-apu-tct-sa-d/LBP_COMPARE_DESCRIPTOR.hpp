#ifndef _ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR
#define _ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LBP_COMPARE_DESCRIPTOR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LBP_COMPARE_DESCRIPTOR[] = {{1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_LBP_COMPARE_DESCRIPTOR[] = {{1, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_LBP_COMPARE_DESCRIPTOR[] = {{1, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 3, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_LBP_COMPARE_DESCRIPTOR[] = {{1, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 4, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_LBP_COMPARE_DESCRIPTOR[] = {{1, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 5, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LBP_COMPARE_DESCRIPTOR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_LBP_COMPARE_DESCRIPTOR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_LBP_COMPARE_DESCRIPTOR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_LBP_COMPARE_DESCRIPTOR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_LBP_COMPARE_DESCRIPTOR[] = {{0, 0}};

static acf_scenario gScenarioArray_LBP_COMPARE_DESCRIPTOR[] = {
{1, 1, 768, 48, 0, gScenarioBufferData0_LBP_COMPARE_DESCRIPTOR, 64, gScenarioKernelData0_LBP_COMPARE_DESCRIPTOR, 4},
{1, 2, 1536, 48, 0, gScenarioBufferData1_LBP_COMPARE_DESCRIPTOR, 64, gScenarioKernelData1_LBP_COMPARE_DESCRIPTOR, 4},
{1, 3, 2304, 48, 0, gScenarioBufferData2_LBP_COMPARE_DESCRIPTOR, 64, gScenarioKernelData2_LBP_COMPARE_DESCRIPTOR, 4},
{1, 4, 3072, 48, 0, gScenarioBufferData3_LBP_COMPARE_DESCRIPTOR, 64, gScenarioKernelData3_LBP_COMPARE_DESCRIPTOR, 4},
{1, 5, 3840, 48, 0, gScenarioBufferData4_LBP_COMPARE_DESCRIPTOR, 64, gScenarioKernelData4_LBP_COMPARE_DESCRIPTOR, 4}
};

static acf_scenario_list gScenarioList_LBP_COMPARE_DESCRIPTOR = {
5, //number of scenarios
gScenarioArray_LBP_COMPARE_DESCRIPTOR};
//**************************************************************

class LBP_COMPARE_DESCRIPTOR : public ACF_Process_APU
{

public:
   LBP_COMPARE_DESCRIPTOR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LBP_COMPARE_DESCRIPTOR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LBP_COMPARE_DESCRIPTOR",
                                        LBP_COMPARE_DESCRIPTOR_LOAD_SEGMENTS,
                                        LBP_COMPARE_DESCRIPTOR_LOAD_PMEM, LBP_COMPARE_DESCRIPTOR_LOAD_PMEM_SIZE,
                                        LBP_COMPARE_DESCRIPTOR_LOAD_DMEM, LBP_COMPARE_DESCRIPTOR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LBP_COMPARE_DESCRIPTOR_APEX_LOG_BUFFER, LBP_COMPARE_DESCRIPTOR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_MODEL", icp::DATATYPE_08U, 128, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_TEST", icp::DATATYPE_08U, 128, 2, 1, 1, 0, 0, 1, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("DISTANCE", icp::DATATYPE_32S, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("CLOSEST_ID", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LBP_COMPARE_DESCRIPTOR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR

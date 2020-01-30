#ifndef _ACF_PROCESS_APU_AGGCF_HIST_TI3
#define _ACF_PROCESS_APU_AGGCF_HIST_TI3

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_HIST_TI3_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_HIST_TI3[] = {{6, 6, 4, 0, {3, 0, 0, 3}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {6, 6, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_HIST_TI3[] = {{12, 6, 4, 0, {3, 0, 0, 3}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {12, 6, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_HIST_TI3[] = {{18, 6, 4, 0, {3, 0, 0, 3}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {18, 6, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_HIST_TI3[] = {{24, 6, 4, 0, {3, 0, 0, 3}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {24, 6, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_HIST_TI3[] = {{6, 12, 4, 0, {3, 0, 0, 3}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {6, 12, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_HIST_TI3[] = {{12, 12, 4, 0, {3, 0, 0, 3}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {12, 12, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_HIST_TI3[] = {{6, 18, 4, 0, {3, 0, 0, 3}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {6, 18, 4, 0, {3, 0, 0, 3}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_HIST_TI3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_HIST_TI3[] = {{1, 0}};

static acf_scenario gScenarioArray_AGGCF_HIST_TI3[] = {
{1, 1, 1328, 528, 1, gScenarioBufferData0_AGGCF_HIST_TI3, 224, gScenarioKernelData0_AGGCF_HIST_TI3, 4},
{2, 1, 2072, 528, 1, gScenarioBufferData1_AGGCF_HIST_TI3, 224, gScenarioKernelData1_AGGCF_HIST_TI3, 4},
{3, 1, 2816, 528, 1, gScenarioBufferData2_AGGCF_HIST_TI3, 224, gScenarioKernelData2_AGGCF_HIST_TI3, 4},
{4, 1, 3560, 528, 1, gScenarioBufferData3_AGGCF_HIST_TI3, 224, gScenarioKernelData3_AGGCF_HIST_TI3, 4},
{1, 2, 2288, 528, 1, gScenarioBufferData4_AGGCF_HIST_TI3, 224, gScenarioKernelData4_AGGCF_HIST_TI3, 4},
{2, 2, 3704, 528, 1, gScenarioBufferData5_AGGCF_HIST_TI3, 224, gScenarioKernelData5_AGGCF_HIST_TI3, 4},
{1, 3, 3248, 528, 1, gScenarioBufferData6_AGGCF_HIST_TI3, 224, gScenarioKernelData6_AGGCF_HIST_TI3, 4}
};

static acf_scenario_list gScenarioList_AGGCF_HIST_TI3 = {
7, //number of scenarios
gScenarioArray_AGGCF_HIST_TI3};
//**************************************************************

class AGGCF_HIST_TI3 : public ACF_Process_APU
{

public:
   AGGCF_HIST_TI3(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_HIST_TI3");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_HIST_TI3",
                                        AGGCF_HIST_TI3_LOAD_SEGMENTS,
                                        AGGCF_HIST_TI3_LOAD_PMEM, AGGCF_HIST_TI3_LOAD_PMEM_SIZE,
                                        AGGCF_HIST_TI3_LOAD_DMEM, AGGCF_HIST_TI3_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_HIST_TI3_APEX_LOG_BUFFER, AGGCF_HIST_TI3_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUTM", icp::DATATYPE_16U, 1, 1, 6, 6, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0);
         AddPort("INPUTO", icp::DATATYPE_16U, 1, 1, 6, 6, 0, 0, 0, 0, 0, 0, 11, 3, 0, 0, 3, 0);
         AddPort("OFFSET", icp::DATATYPE_32S, 1, 1, 128, 1, 0, 1, 1, 0, 1, 0, 12, 0, 0, 0, 0, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 4, 1, 0, 1, 1, 0, 1, 0, 13, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH1", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH2", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH3", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH4", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH5", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH6", icp::DATATYPE_16U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_HIST_TI3);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_HIST_TI3

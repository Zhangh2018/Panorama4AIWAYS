#ifndef _ACF_PROCESS_APU_PYRAMID_MULTI_DOWN
#define _ACF_PROCESS_APU_PYRAMID_MULTI_DOWN

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <PYRAMID_MULTI_DOWN_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_PYRAMID_MULTI_DOWN[] = {{8, 4, 2, 0, {0, 0, 2, 2}}, {8, 4, 4, 0, {2, 2, 0, 0}}, {4, 2, 2, 1, {0, 0, 2, 2}}, {4, 2, 4, 1, {2, 2, 0, 0}}, {2, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_PYRAMID_MULTI_DOWN[] = {{16, 4, 2, 0, {0, 0, 2, 2}}, {16, 4, 4, 0, {2, 2, 0, 0}}, {8, 2, 2, 1, {0, 0, 2, 2}}, {8, 2, 4, 1, {2, 2, 0, 0}}, {4, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_PYRAMID_MULTI_DOWN[] = {{24, 4, 2, 0, {0, 0, 2, 2}}, {24, 4, 4, 0, {2, 2, 0, 0}}, {12, 2, 2, 1, {0, 0, 2, 2}}, {12, 2, 4, 1, {2, 2, 0, 0}}, {6, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_PYRAMID_MULTI_DOWN[] = {{32, 4, 2, 0, {0, 0, 2, 2}}, {32, 4, 4, 0, {2, 2, 0, 0}}, {16, 2, 2, 1, {0, 0, 2, 2}}, {16, 2, 4, 1, {2, 2, 0, 0}}, {8, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_PYRAMID_MULTI_DOWN[] = {{40, 4, 2, 0, {0, 0, 2, 2}}, {40, 4, 4, 0, {2, 2, 0, 0}}, {20, 2, 2, 1, {0, 0, 2, 2}}, {20, 2, 4, 1, {2, 2, 0, 0}}, {10, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_PYRAMID_MULTI_DOWN[] = {{48, 4, 2, 0, {0, 0, 2, 2}}, {48, 4, 4, 0, {2, 2, 0, 0}}, {24, 2, 2, 1, {0, 0, 2, 2}}, {24, 2, 4, 1, {2, 2, 0, 0}}, {12, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_PYRAMID_MULTI_DOWN[] = {{56, 4, 2, 0, {0, 0, 2, 2}}, {56, 4, 4, 0, {2, 2, 0, 0}}, {28, 2, 2, 1, {0, 0, 2, 2}}, {28, 2, 4, 1, {2, 2, 0, 0}}, {14, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_PYRAMID_MULTI_DOWN[] = {{8, 8, 2, 0, {0, 0, 2, 2}}, {8, 8, 4, 0, {2, 2, 0, 0}}, {4, 4, 2, 1, {0, 0, 2, 2}}, {4, 4, 4, 1, {2, 2, 0, 0}}, {2, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_PYRAMID_MULTI_DOWN[] = {{16, 8, 2, 0, {0, 0, 2, 2}}, {16, 8, 4, 0, {2, 2, 0, 0}}, {8, 4, 2, 1, {0, 0, 2, 2}}, {8, 4, 4, 1, {2, 2, 0, 0}}, {4, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_PYRAMID_MULTI_DOWN[] = {{24, 8, 2, 0, {0, 0, 2, 2}}, {24, 8, 4, 0, {2, 2, 0, 0}}, {12, 4, 2, 1, {0, 0, 2, 2}}, {12, 4, 4, 1, {2, 2, 0, 0}}, {6, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_PYRAMID_MULTI_DOWN[] = {{32, 8, 2, 0, {0, 0, 2, 2}}, {32, 8, 4, 0, {2, 2, 0, 0}}, {16, 4, 2, 1, {0, 0, 2, 2}}, {16, 4, 4, 1, {2, 2, 0, 0}}, {8, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_PYRAMID_MULTI_DOWN[] = {{8, 12, 2, 0, {0, 0, 2, 2}}, {8, 12, 4, 0, {2, 2, 0, 0}}, {4, 6, 2, 1, {0, 0, 2, 2}}, {4, 6, 4, 1, {2, 2, 0, 0}}, {2, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_PYRAMID_MULTI_DOWN[] = {{16, 12, 2, 0, {0, 0, 2, 2}}, {16, 12, 4, 0, {2, 2, 0, 0}}, {8, 6, 2, 1, {0, 0, 2, 2}}, {8, 6, 4, 1, {2, 2, 0, 0}}, {4, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_PYRAMID_MULTI_DOWN[] = {{8, 16, 2, 0, {0, 0, 2, 2}}, {8, 16, 4, 0, {2, 2, 0, 0}}, {4, 8, 2, 1, {0, 0, 2, 2}}, {4, 8, 4, 1, {2, 2, 0, 0}}, {2, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_PYRAMID_MULTI_DOWN[] = {{16, 16, 2, 0, {0, 0, 2, 2}}, {16, 16, 4, 0, {2, 2, 0, 0}}, {8, 8, 2, 1, {0, 0, 2, 2}}, {8, 8, 4, 1, {2, 2, 0, 0}}, {4, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_PYRAMID_MULTI_DOWN[] = {{8, 20, 2, 0, {0, 0, 2, 2}}, {8, 20, 4, 0, {2, 2, 0, 0}}, {4, 10, 2, 1, {0, 0, 2, 2}}, {4, 10, 4, 1, {2, 2, 0, 0}}, {2, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_PYRAMID_MULTI_DOWN[] = {{8, 24, 2, 0, {0, 0, 2, 2}}, {8, 24, 4, 0, {2, 2, 0, 0}}, {4, 12, 2, 1, {0, 0, 2, 2}}, {4, 12, 4, 1, {2, 2, 0, 0}}, {2, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_PYRAMID_MULTI_DOWN[] = {{8, 32, 2, 0, {0, 0, 2, 2}}, {8, 32, 4, 0, {2, 2, 0, 0}}, {4, 16, 2, 1, {0, 0, 2, 2}}, {4, 16, 4, 1, {2, 2, 0, 0}}, {2, 8, 2, 2, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_PYRAMID_MULTI_DOWN[] = {{0, 0}, {1, 0}, {1, 0}, {2, 0}};

static acf_scenario gScenarioArray_PYRAMID_MULTI_DOWN[] = {
{2, 1, 548, 0, 2, gScenarioBufferData0_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData0_PYRAMID_MULTI_DOWN, 16},
{4, 1, 1048, 0, 2, gScenarioBufferData1_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData1_PYRAMID_MULTI_DOWN, 16},
{6, 1, 1548, 0, 2, gScenarioBufferData2_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData2_PYRAMID_MULTI_DOWN, 16},
{8, 1, 2048, 0, 2, gScenarioBufferData3_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData3_PYRAMID_MULTI_DOWN, 16},
{10, 1, 2548, 0, 2, gScenarioBufferData4_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData4_PYRAMID_MULTI_DOWN, 16},
{12, 1, 3048, 0, 2, gScenarioBufferData5_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData5_PYRAMID_MULTI_DOWN, 16},
{14, 1, 3548, 0, 2, gScenarioBufferData6_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData6_PYRAMID_MULTI_DOWN, 16},
{2, 2, 1000, 0, 2, gScenarioBufferData7_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData7_PYRAMID_MULTI_DOWN, 16},
{4, 2, 1904, 0, 2, gScenarioBufferData8_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData8_PYRAMID_MULTI_DOWN, 16},
{6, 2, 2808, 0, 2, gScenarioBufferData9_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData9_PYRAMID_MULTI_DOWN, 16},
{8, 2, 3712, 0, 2, gScenarioBufferData10_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData10_PYRAMID_MULTI_DOWN, 16},
{2, 3, 1452, 0, 2, gScenarioBufferData11_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData11_PYRAMID_MULTI_DOWN, 16},
{4, 3, 2760, 0, 2, gScenarioBufferData12_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData12_PYRAMID_MULTI_DOWN, 16},
{2, 4, 1904, 0, 2, gScenarioBufferData13_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData13_PYRAMID_MULTI_DOWN, 16},
{4, 4, 3616, 0, 2, gScenarioBufferData14_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData14_PYRAMID_MULTI_DOWN, 16},
{2, 5, 2356, 0, 2, gScenarioBufferData15_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData15_PYRAMID_MULTI_DOWN, 16},
{2, 6, 2808, 0, 2, gScenarioBufferData16_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData16_PYRAMID_MULTI_DOWN, 16},
{2, 8, 3712, 0, 2, gScenarioBufferData17_PYRAMID_MULTI_DOWN, 80, gScenarioKernelData17_PYRAMID_MULTI_DOWN, 16}
};

static acf_scenario_list gScenarioList_PYRAMID_MULTI_DOWN = {
18, //number of scenarios
gScenarioArray_PYRAMID_MULTI_DOWN};
//**************************************************************

class PYRAMID_MULTI_DOWN : public ACF_Process_APU
{

public:
   PYRAMID_MULTI_DOWN(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("PYRAMID_MULTI_DOWN");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("PYRAMID_MULTI_DOWN",
                                        PYRAMID_MULTI_DOWN_LOAD_SEGMENTS,
                                        PYRAMID_MULTI_DOWN_LOAD_PMEM, PYRAMID_MULTI_DOWN_LOAD_PMEM_SIZE,
                                        PYRAMID_MULTI_DOWN_LOAD_DMEM, PYRAMID_MULTI_DOWN_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(PYRAMID_MULTI_DOWN_APEX_LOG_BUFFER, PYRAMID_MULTI_DOWN_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 8, 4, 0, 0, 0, 0, 0, 0, 0, 6, 6, 6, 6, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_PYRAMID_MULTI_DOWN);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_PYRAMID_MULTI_DOWN

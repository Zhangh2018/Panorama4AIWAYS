#ifndef _ACF_PROCESS_APU_AGGCF_SCALE_DOWN8
#define _ACF_PROCESS_APU_AGGCF_SCALE_DOWN8

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_SCALE_DOWN8_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_SCALE_DOWN8[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_SCALE_DOWN8[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_SCALE_DOWN8[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 1, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_SCALE_DOWN8[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_SCALE_DOWN8[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 1, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_SCALE_DOWN8[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_SCALE_DOWN8[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 1, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_SCALE_DOWN8[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_SCALE_DOWN8[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 1, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_SCALE_DOWN8[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_SCALE_DOWN8[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 1, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_SCALE_DOWN8[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_SCALE_DOWN8[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 1, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_SCALE_DOWN8[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_SCALE_DOWN8[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 1, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_SCALE_DOWN8[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_SCALE_DOWN8[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 1, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_SCALE_DOWN8[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_SCALE_DOWN8[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 1, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_SCALE_DOWN8[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_SCALE_DOWN8[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {3, 32, 1, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_SCALE_DOWN8[] = {{8, 40, 2, 0, {0, 0, 0, 0}}, {1, 40, 1, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_SCALE_DOWN8[] = {{16, 40, 2, 0, {0, 0, 0, 0}}, {2, 40, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_SCALE_DOWN8[] = {{8, 48, 2, 0, {0, 0, 0, 0}}, {1, 48, 1, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_SCALE_DOWN8[] = {{16, 48, 2, 0, {0, 0, 0, 0}}, {2, 48, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_SCALE_DOWN8[] = {{8, 64, 2, 0, {0, 0, 0, 0}}, {1, 64, 1, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_SCALE_DOWN8[] = {{8, 80, 2, 0, {0, 0, 0, 0}}, {1, 80, 1, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_SCALE_DOWN8[] = {{8, 96, 2, 0, {0, 0, 0, 0}}, {1, 96, 1, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_SCALE_DOWN8[] = {{0, 0}, {0, 0}};

static acf_scenario gScenarioArray_AGGCF_SCALE_DOWN8[] = {
{1, 1, 276, 0, 0, gScenarioBufferData0_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData0_AGGCF_SCALE_DOWN8, 8},
{2, 1, 552, 0, 0, gScenarioBufferData1_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData1_AGGCF_SCALE_DOWN8, 8},
{3, 1, 828, 0, 0, gScenarioBufferData2_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData2_AGGCF_SCALE_DOWN8, 8},
{4, 1, 1104, 0, 0, gScenarioBufferData3_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData3_AGGCF_SCALE_DOWN8, 8},
{5, 1, 1380, 0, 0, gScenarioBufferData4_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData4_AGGCF_SCALE_DOWN8, 8},
{6, 1, 1656, 0, 0, gScenarioBufferData5_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData5_AGGCF_SCALE_DOWN8, 8},
{7, 1, 1932, 0, 0, gScenarioBufferData6_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData6_AGGCF_SCALE_DOWN8, 8},
{8, 1, 2208, 0, 0, gScenarioBufferData7_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData7_AGGCF_SCALE_DOWN8, 8},
{1, 2, 552, 0, 0, gScenarioBufferData8_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData8_AGGCF_SCALE_DOWN8, 8},
{2, 2, 1104, 0, 0, gScenarioBufferData9_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData9_AGGCF_SCALE_DOWN8, 8},
{3, 2, 1656, 0, 0, gScenarioBufferData10_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData10_AGGCF_SCALE_DOWN8, 8},
{4, 2, 2208, 0, 0, gScenarioBufferData11_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData11_AGGCF_SCALE_DOWN8, 8},
{5, 2, 2760, 0, 0, gScenarioBufferData12_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData12_AGGCF_SCALE_DOWN8, 8},
{6, 2, 3312, 0, 0, gScenarioBufferData13_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData13_AGGCF_SCALE_DOWN8, 8},
{1, 3, 828, 0, 0, gScenarioBufferData14_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData14_AGGCF_SCALE_DOWN8, 8},
{2, 3, 1656, 0, 0, gScenarioBufferData15_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData15_AGGCF_SCALE_DOWN8, 8},
{3, 3, 2484, 0, 0, gScenarioBufferData16_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData16_AGGCF_SCALE_DOWN8, 8},
{4, 3, 3312, 0, 0, gScenarioBufferData17_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData17_AGGCF_SCALE_DOWN8, 8},
{1, 4, 1104, 0, 0, gScenarioBufferData18_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData18_AGGCF_SCALE_DOWN8, 8},
{2, 4, 2208, 0, 0, gScenarioBufferData19_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData19_AGGCF_SCALE_DOWN8, 8},
{3, 4, 3312, 0, 0, gScenarioBufferData20_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData20_AGGCF_SCALE_DOWN8, 8},
{1, 5, 1380, 0, 0, gScenarioBufferData21_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData21_AGGCF_SCALE_DOWN8, 8},
{2, 5, 2760, 0, 0, gScenarioBufferData22_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData22_AGGCF_SCALE_DOWN8, 8},
{1, 6, 1656, 0, 0, gScenarioBufferData23_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData23_AGGCF_SCALE_DOWN8, 8},
{2, 6, 3312, 0, 0, gScenarioBufferData24_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData24_AGGCF_SCALE_DOWN8, 8},
{1, 8, 2208, 0, 0, gScenarioBufferData25_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData25_AGGCF_SCALE_DOWN8, 8},
{1, 10, 2760, 0, 0, gScenarioBufferData26_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData26_AGGCF_SCALE_DOWN8, 8},
{1, 12, 3312, 0, 0, gScenarioBufferData27_AGGCF_SCALE_DOWN8, 48, gScenarioKernelData27_AGGCF_SCALE_DOWN8, 8}
};

static acf_scenario_list gScenarioList_AGGCF_SCALE_DOWN8 = {
28, //number of scenarios
gScenarioArray_AGGCF_SCALE_DOWN8};
//**************************************************************

class AGGCF_SCALE_DOWN8 : public ACF_Process_APU
{

public:
   AGGCF_SCALE_DOWN8(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_SCALE_DOWN8");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_SCALE_DOWN8",
                                        AGGCF_SCALE_DOWN8_LOAD_SEGMENTS,
                                        AGGCF_SCALE_DOWN8_LOAD_PMEM, AGGCF_SCALE_DOWN8_LOAD_PMEM_SIZE,
                                        AGGCF_SCALE_DOWN8_LOAD_DMEM, AGGCF_SCALE_DOWN8_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_SCALE_DOWN8_APEX_LOG_BUFFER, AGGCF_SCALE_DOWN8_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("SRC", icp::DATATYPE_16U, 1, 1, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("DST", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_SCALE_DOWN8);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_SCALE_DOWN8

#ifndef _ACF_PROCESS_APU_BLOCKMATCHING
#define _ACF_PROCESS_APU_BLOCKMATCHING

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <BLOCKMATCHING_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_BLOCKMATCHING[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {28, 28, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_BLOCKMATCHING[] = {{0, 0}};

static acf_scenario gScenarioArray_BLOCKMATCHING[] = {
{0, 0, 2092, 0, 0, gScenarioBufferData0_BLOCKMATCHING, 64, gScenarioKernelData0_BLOCKMATCHING, 4}
};

static acf_scenario_list gScenarioList_BLOCKMATCHING = {
1, //number of scenarios
gScenarioArray_BLOCKMATCHING};
//**************************************************************

class BLOCKMATCHING : public ACF_Process_APU
{

public:
   BLOCKMATCHING(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("BLOCKMATCHING");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("BLOCKMATCHING",
                                        BLOCKMATCHING_LOAD_SEGMENTS,
                                        BLOCKMATCHING_LOAD_PMEM, BLOCKMATCHING_LOAD_PMEM_SIZE,
                                        BLOCKMATCHING_LOAD_DMEM, BLOCKMATCHING_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(BLOCKMATCHING_APEX_LOG_BUFFER, BLOCKMATCHING_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_TEMPLATE", icp::DATATYPE_08U, 1, 1, 16, 16, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_WINDOW", icp::DATATYPE_08U, 1, 1, 28, 28, 0, 0, 0, 1, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_SCORE", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_POINTS", icp::DATATYPE_16S, 2, 1, 1, 1, 1, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_BLOCKMATCHING);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_BLOCKMATCHING

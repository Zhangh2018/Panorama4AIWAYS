#ifndef _ACF_PROCESS_APU_FIRE9_1ST_E1E3MP
#define _ACF_PROCESS_APU_FIRE9_1ST_E1E3MP

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FIRE9_1ST_E1E3MP_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FIRE9_1ST_E1E3MP[] = {{1808, 2, 3, 0, {1, 1, 0, 0}}, {9, 1, 1, 1, {0, 0, 0, 0}}, {112, 3, 1, 1, {0, 0, 0, 0}}, {112, 3, 1, 1, {0, 0, 0, 0}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {64, 45, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FIRE9_1ST_E1E3MP[] = {{1, 1}};

static acf_scenario gScenarioArray_FIRE9_1ST_E1E3MP[] = {
{0, 0, 3776, 21760, 2, gScenarioBufferData0_FIRE9_1ST_E1E3MP, 128, gScenarioKernelData0_FIRE9_1ST_E1E3MP, 4}
};

static acf_scenario_list gScenarioList_FIRE9_1ST_E1E3MP = {
1, //number of scenarios
gScenarioArray_FIRE9_1ST_E1E3MP};
//**************************************************************

class FIRE9_1ST_E1E3MP : public ACF_Process_APU
{

public:
   FIRE9_1ST_E1E3MP(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FIRE9_1ST_E1E3MP");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FIRE9_1ST_E1E3MP",
                                        FIRE9_1ST_E1E3MP_LOAD_SEGMENTS,
                                        FIRE9_1ST_E1E3MP_LOAD_PMEM, FIRE9_1ST_E1E3MP_LOAD_PMEM_SIZE,
                                        FIRE9_1ST_E1E3MP_LOAD_DMEM, FIRE9_1ST_E1E3MP_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FIRE9_1ST_E1E3MP_APEX_LOG_BUFFER, FIRE9_1ST_E1E3MP_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FIRE9_1ST_E1E3MP_INPUT_IMAGE", icp::DATATYPE_08S, 1, 1, 1808, 2, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
         AddPort("FIRE9_1ST_E1E3MP_INPUT_WEIGHT", icp::DATATYPE_08S, 1, 1, 64, 45, 0, 0, 1, 0, 1, 0, 6, 0, 0, 0, 0, 0);
         AddPort("FIRE9_1ST_E1E3MP_INPUT_PARAMS", icp::DATATYPE_08S, 1, 1, 16, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("FIRE9_1ST_E1E3MP_OUTPUT_MP1", icp::DATATYPE_08S, 1, 1, 56, 1, 1, 0, 0, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("FIRE9_1ST_E1E3MP_OUTPUT_MP3", icp::DATATYPE_08S, 1, 1, 56, 1, 1, 0, 0, 0, 1, 0, 5, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FIRE9_1ST_E1E3MP);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FIRE9_1ST_E1E3MP

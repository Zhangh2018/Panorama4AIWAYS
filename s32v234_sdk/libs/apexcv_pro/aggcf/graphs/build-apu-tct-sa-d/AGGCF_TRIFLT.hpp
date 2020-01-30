#ifndef _ACF_PROCESS_APU_AGGCF_TRIFLT
#define _ACF_PROCESS_APU_AGGCF_TRIFLT

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_TRIFLT_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_TRIFLT[] = {{10, 8, 4, 0, {7, 7, 7, 7}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {12, 24, 1, 1, {0, 0, 0, 0}}, {12, 1, 1, 1, {0, 0, 0, 0}}, {12, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_TRIFLT[] = {{1, 0}};

static acf_scenario gScenarioArray_AGGCF_TRIFLT[] = {
{1, 1, 3776, 16, 1, gScenarioBufferData0_AGGCF_TRIFLT, 96, gScenarioKernelData0_AGGCF_TRIFLT, 4}
};

static acf_scenario_list gScenarioList_AGGCF_TRIFLT = {
1, //number of scenarios
gScenarioArray_AGGCF_TRIFLT};
//**************************************************************

class AGGCF_TRIFLT : public ACF_Process_APU
{

public:
   AGGCF_TRIFLT(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_TRIFLT");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_TRIFLT",
                                        AGGCF_TRIFLT_LOAD_SEGMENTS,
                                        AGGCF_TRIFLT_LOAD_PMEM, AGGCF_TRIFLT_LOAD_PMEM_SIZE,
                                        AGGCF_TRIFLT_LOAD_DMEM, AGGCF_TRIFLT_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_TRIFLT_APEX_LOG_BUFFER, AGGCF_TRIFLT_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_16U, 1, 1, 10, 8, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_16U, 1, 1, 10, 8, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_TRIFLT);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_TRIFLT

#ifndef _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_EXTRACT
#define _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_EXTRACT

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <GFTT_B7N5_COMPUTE_EXTRACT_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_GFTT_B7N5_COMPUTE_EXTRACT[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_GFTT_B7N5_COMPUTE_EXTRACT[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_GFTT_B7N5_COMPUTE_EXTRACT[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_GFTT_B7N5_COMPUTE_EXTRACT[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_GFTT_B7N5_COMPUTE_EXTRACT[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_GFTT_B7N5_COMPUTE_EXTRACT[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_GFTT_B7N5_COMPUTE_EXTRACT[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_GFTT_B7N5_COMPUTE_EXTRACT[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_GFTT_B7N5_COMPUTE_EXTRACT[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_GFTT_B7N5_COMPUTE_EXTRACT[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_GFTT_B7N5_COMPUTE_EXTRACT[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_GFTT_B7N5_COMPUTE_EXTRACT[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_GFTT_B7N5_COMPUTE_EXTRACT[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_GFTT_B7N5_COMPUTE_EXTRACT[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_GFTT_B7N5_COMPUTE_EXTRACT[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_GFTT_B7N5_COMPUTE_EXTRACT[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_GFTT_B7N5_COMPUTE_EXTRACT[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_GFTT_B7N5_COMPUTE_EXTRACT[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_GFTT_B7N5_COMPUTE_EXTRACT[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_GFTT_B7N5_COMPUTE_EXTRACT[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 3, 3, {0, 3, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_GFTT_B7N5_COMPUTE_EXTRACT[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 3, 3, {0, 3, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_GFTT_B7N5_COMPUTE_EXTRACT[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_GFTT_B7N5_COMPUTE_EXTRACT[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 1, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_GFTT_B7N5_COMPUTE_EXTRACT[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 3, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_GFTT_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};

static acf_scenario gScenarioArray_GFTT_B7N5_COMPUTE_EXTRACT[] = {
{1, 1, 3300, 65600, 6, gScenarioBufferData0_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData0_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{2, 1, 3340, 65600, 6, gScenarioBufferData1_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData1_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{3, 1, 3380, 65600, 6, gScenarioBufferData2_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData2_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{4, 1, 3420, 65600, 6, gScenarioBufferData3_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData3_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{5, 1, 3460, 65600, 6, gScenarioBufferData4_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData4_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{6, 1, 3500, 65600, 6, gScenarioBufferData5_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData5_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{7, 1, 3540, 65600, 6, gScenarioBufferData6_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData6_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{8, 1, 3580, 65600, 6, gScenarioBufferData7_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData7_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{9, 1, 3620, 65600, 6, gScenarioBufferData8_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData8_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{10, 1, 3660, 65600, 6, gScenarioBufferData9_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData9_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{11, 1, 3700, 65600, 6, gScenarioBufferData10_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData10_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{12, 1, 3740, 65600, 6, gScenarioBufferData11_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData11_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{13, 1, 3780, 65600, 6, gScenarioBufferData12_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData12_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{14, 1, 3820, 65600, 6, gScenarioBufferData13_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData13_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 2, 3372, 65600, 4, gScenarioBufferData14_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData14_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{2, 2, 3468, 65600, 4, gScenarioBufferData15_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData15_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{3, 2, 3564, 65600, 4, gScenarioBufferData16_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData16_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{4, 2, 3660, 65600, 4, gScenarioBufferData17_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData17_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{5, 2, 3756, 65600, 4, gScenarioBufferData18_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData18_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 3, 3428, 65600, 3, gScenarioBufferData19_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData19_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{2, 3, 3564, 65600, 3, gScenarioBufferData20_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData20_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{3, 3, 3700, 65600, 3, gScenarioBufferData21_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData21_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{4, 3, 3836, 65600, 3, gScenarioBufferData22_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData22_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 4, 3500, 65600, 3, gScenarioBufferData23_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData23_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{2, 4, 3692, 65600, 3, gScenarioBufferData24_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData24_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 5, 3492, 65600, 2, gScenarioBufferData25_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData25_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{2, 5, 3660, 65600, 2, gScenarioBufferData26_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData26_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{3, 5, 3828, 65600, 2, gScenarioBufferData27_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData27_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 6, 3596, 65600, 2, gScenarioBufferData28_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData28_GFTT_B7N5_COMPUTE_EXTRACT, 12},
{1, 8, 3724, 65600, 2, gScenarioBufferData29_GFTT_B7N5_COMPUTE_EXTRACT, 320, gScenarioKernelData29_GFTT_B7N5_COMPUTE_EXTRACT, 12}
};

static acf_scenario_list gScenarioList_GFTT_B7N5_COMPUTE_EXTRACT = {
30, //number of scenarios
gScenarioArray_GFTT_B7N5_COMPUTE_EXTRACT};
//**************************************************************

class GFTT_B7N5_COMPUTE_EXTRACT : public ACF_Process_APU
{

public:
   GFTT_B7N5_COMPUTE_EXTRACT(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("GFTT_B7N5_COMPUTE_EXTRACT");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("GFTT_B7N5_COMPUTE_EXTRACT",
                                        GFTT_B7N5_COMPUTE_EXTRACT_LOAD_SEGMENTS,
                                        GFTT_B7N5_COMPUTE_EXTRACT_LOAD_PMEM, GFTT_B7N5_COMPUTE_EXTRACT_LOAD_PMEM_SIZE,
                                        GFTT_B7N5_COMPUTE_EXTRACT_LOAD_DMEM, GFTT_B7N5_COMPUTE_EXTRACT_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(GFTT_B7N5_COMPUTE_EXTRACT_APEX_LOG_BUFFER, GFTT_B7N5_COMPUTE_EXTRACT_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 6, 1, 0, 1, 1, 0, 1, 0, 19, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_CORNERLIST", icp::DATATYPE_32S, 1, 1, 8192, 1, 1, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_COUNT", icp::DATATYPE_32S, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_GFTT_B7N5_COMPUTE_EXTRACT);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_EXTRACT

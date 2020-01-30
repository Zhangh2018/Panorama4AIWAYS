#ifndef _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE
#define _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <GFTT_B7N5_COMPUTE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_GFTT_B7N5_COMPUTE[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_GFTT_B7N5_COMPUTE[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_GFTT_B7N5_COMPUTE[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_GFTT_B7N5_COMPUTE[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_GFTT_B7N5_COMPUTE[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_GFTT_B7N5_COMPUTE[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_GFTT_B7N5_COMPUTE[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_GFTT_B7N5_COMPUTE[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_GFTT_B7N5_COMPUTE[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_GFTT_B7N5_COMPUTE[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_GFTT_B7N5_COMPUTE[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_GFTT_B7N5_COMPUTE[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_GFTT_B7N5_COMPUTE[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_GFTT_B7N5_COMPUTE[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_GFTT_B7N5_COMPUTE[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_GFTT_B7N5_COMPUTE[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_GFTT_B7N5_COMPUTE[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_GFTT_B7N5_COMPUTE[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_GFTT_B7N5_COMPUTE[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_GFTT_B7N5_COMPUTE[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_GFTT_B7N5_COMPUTE[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_GFTT_B7N5_COMPUTE[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_GFTT_B7N5_COMPUTE[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_GFTT_B7N5_COMPUTE[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_GFTT_B7N5_COMPUTE[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_GFTT_B7N5_COMPUTE[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_GFTT_B7N5_COMPUTE[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_GFTT_B7N5_COMPUTE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_GFTT_B7N5_COMPUTE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_GFTT_B7N5_COMPUTE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_GFTT_B7N5_COMPUTE[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_GFTT_B7N5_COMPUTE[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_GFTT_B7N5_COMPUTE[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_GFTT_B7N5_COMPUTE[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_GFTT_B7N5_COMPUTE[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 3, 2, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_GFTT_B7N5_COMPUTE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_GFTT_B7N5_COMPUTE[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 3, 2, {0, 5, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData1_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData2_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData3_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData4_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData5_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData6_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData7_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData8_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData9_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData10_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData11_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData12_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData13_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData14_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData15_GFTT_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData16_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData17_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData18_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData19_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData20_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData21_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData22_GFTT_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData23_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData24_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData25_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData26_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData27_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData28_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData29_GFTT_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData30_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData31_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData32_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData33_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData34_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData35_GFTT_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData36_GFTT_B7N5_COMPUTE[] = {{1, 1}};

static acf_scenario gScenarioArray_GFTT_B7N5_COMPUTE[] = {
{1, 1, 3180, 32, 6, gScenarioBufferData0_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData0_GFTT_B7N5_COMPUTE, 4},
{2, 1, 3220, 32, 6, gScenarioBufferData1_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData1_GFTT_B7N5_COMPUTE, 4},
{3, 1, 3260, 32, 6, gScenarioBufferData2_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData2_GFTT_B7N5_COMPUTE, 4},
{4, 1, 3300, 32, 6, gScenarioBufferData3_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData3_GFTT_B7N5_COMPUTE, 4},
{5, 1, 3340, 32, 6, gScenarioBufferData4_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData4_GFTT_B7N5_COMPUTE, 4},
{6, 1, 3380, 32, 6, gScenarioBufferData5_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData5_GFTT_B7N5_COMPUTE, 4},
{7, 1, 3420, 32, 6, gScenarioBufferData6_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData6_GFTT_B7N5_COMPUTE, 4},
{8, 1, 3460, 32, 6, gScenarioBufferData7_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData7_GFTT_B7N5_COMPUTE, 4},
{9, 1, 3500, 32, 6, gScenarioBufferData8_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData8_GFTT_B7N5_COMPUTE, 4},
{10, 1, 3540, 32, 6, gScenarioBufferData9_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData9_GFTT_B7N5_COMPUTE, 4},
{11, 1, 3580, 32, 6, gScenarioBufferData10_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData10_GFTT_B7N5_COMPUTE, 4},
{12, 1, 3620, 32, 6, gScenarioBufferData11_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData11_GFTT_B7N5_COMPUTE, 4},
{13, 1, 3660, 32, 6, gScenarioBufferData12_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData12_GFTT_B7N5_COMPUTE, 4},
{14, 1, 3700, 32, 6, gScenarioBufferData13_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData13_GFTT_B7N5_COMPUTE, 4},
{15, 1, 3740, 32, 6, gScenarioBufferData14_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData14_GFTT_B7N5_COMPUTE, 4},
{16, 1, 3780, 32, 6, gScenarioBufferData15_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData15_GFTT_B7N5_COMPUTE, 4},
{1, 2, 3252, 32, 4, gScenarioBufferData16_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData16_GFTT_B7N5_COMPUTE, 4},
{2, 2, 3348, 32, 4, gScenarioBufferData17_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData17_GFTT_B7N5_COMPUTE, 4},
{3, 2, 3444, 32, 4, gScenarioBufferData18_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData18_GFTT_B7N5_COMPUTE, 4},
{4, 2, 3540, 32, 4, gScenarioBufferData19_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData19_GFTT_B7N5_COMPUTE, 4},
{5, 2, 3636, 32, 4, gScenarioBufferData20_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData20_GFTT_B7N5_COMPUTE, 4},
{6, 2, 3732, 32, 4, gScenarioBufferData21_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData21_GFTT_B7N5_COMPUTE, 4},
{7, 2, 3828, 32, 4, gScenarioBufferData22_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData22_GFTT_B7N5_COMPUTE, 4},
{1, 3, 3308, 32, 3, gScenarioBufferData23_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData23_GFTT_B7N5_COMPUTE, 4},
{2, 3, 3444, 32, 3, gScenarioBufferData24_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData24_GFTT_B7N5_COMPUTE, 4},
{3, 3, 3580, 32, 3, gScenarioBufferData25_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData25_GFTT_B7N5_COMPUTE, 4},
{4, 3, 3716, 32, 3, gScenarioBufferData26_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData26_GFTT_B7N5_COMPUTE, 4},
{1, 4, 3380, 32, 3, gScenarioBufferData27_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData27_GFTT_B7N5_COMPUTE, 4},
{2, 4, 3572, 32, 3, gScenarioBufferData28_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData28_GFTT_B7N5_COMPUTE, 4},
{3, 4, 3764, 32, 3, gScenarioBufferData29_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData29_GFTT_B7N5_COMPUTE, 4},
{1, 5, 3372, 32, 2, gScenarioBufferData30_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData30_GFTT_B7N5_COMPUTE, 4},
{2, 5, 3540, 32, 2, gScenarioBufferData31_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData31_GFTT_B7N5_COMPUTE, 4},
{3, 5, 3708, 32, 2, gScenarioBufferData32_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData32_GFTT_B7N5_COMPUTE, 4},
{1, 6, 3476, 32, 2, gScenarioBufferData33_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData33_GFTT_B7N5_COMPUTE, 4},
{2, 6, 3732, 32, 2, gScenarioBufferData34_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData34_GFTT_B7N5_COMPUTE, 4},
{1, 8, 3604, 32, 2, gScenarioBufferData35_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData35_GFTT_B7N5_COMPUTE, 4},
{1, 10, 3732, 32, 2, gScenarioBufferData36_GFTT_B7N5_COMPUTE, 192, gScenarioKernelData36_GFTT_B7N5_COMPUTE, 4}
};

static acf_scenario_list gScenarioList_GFTT_B7N5_COMPUTE = {
37, //number of scenarios
gScenarioArray_GFTT_B7N5_COMPUTE};
//**************************************************************

class GFTT_B7N5_COMPUTE : public ACF_Process_APU
{

public:
   GFTT_B7N5_COMPUTE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("GFTT_B7N5_COMPUTE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("GFTT_B7N5_COMPUTE",
                                        GFTT_B7N5_COMPUTE_LOAD_SEGMENTS,
                                        GFTT_B7N5_COMPUTE_LOAD_PMEM, GFTT_B7N5_COMPUTE_LOAD_PMEM_SIZE,
                                        GFTT_B7N5_COMPUTE_LOAD_DMEM, GFTT_B7N5_COMPUTE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(GFTT_B7N5_COMPUTE_APEX_LOG_BUFFER, GFTT_B7N5_COMPUTE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 6, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_GFTT_B7N5_COMPUTE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_GFTT_B7N5_COMPUTE

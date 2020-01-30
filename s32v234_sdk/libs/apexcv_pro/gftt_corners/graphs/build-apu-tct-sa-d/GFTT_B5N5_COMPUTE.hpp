#ifndef _ACF_PROCESS_APU_GFTT_B5N5_COMPUTE
#define _ACF_PROCESS_APU_GFTT_B5N5_COMPUTE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <GFTT_B5N5_COMPUTE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_GFTT_B5N5_COMPUTE[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_GFTT_B5N5_COMPUTE[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_GFTT_B5N5_COMPUTE[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_GFTT_B5N5_COMPUTE[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_GFTT_B5N5_COMPUTE[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_GFTT_B5N5_COMPUTE[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_GFTT_B5N5_COMPUTE[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_GFTT_B5N5_COMPUTE[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_GFTT_B5N5_COMPUTE[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_GFTT_B5N5_COMPUTE[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_GFTT_B5N5_COMPUTE[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_GFTT_B5N5_COMPUTE[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_GFTT_B5N5_COMPUTE[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_GFTT_B5N5_COMPUTE[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_GFTT_B5N5_COMPUTE[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_GFTT_B5N5_COMPUTE[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_GFTT_B5N5_COMPUTE[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_GFTT_B5N5_COMPUTE[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_GFTT_B5N5_COMPUTE[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_GFTT_B5N5_COMPUTE[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_GFTT_B5N5_COMPUTE[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_GFTT_B5N5_COMPUTE[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_GFTT_B5N5_COMPUTE[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_GFTT_B5N5_COMPUTE[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_GFTT_B5N5_COMPUTE[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_GFTT_B5N5_COMPUTE[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_GFTT_B5N5_COMPUTE[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_GFTT_B5N5_COMPUTE[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_GFTT_B5N5_COMPUTE[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_GFTT_B5N5_COMPUTE[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_GFTT_B5N5_COMPUTE[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_GFTT_B5N5_COMPUTE[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_GFTT_B5N5_COMPUTE[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_GFTT_B5N5_COMPUTE[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_GFTT_B5N5_COMPUTE[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_GFTT_B5N5_COMPUTE[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_GFTT_B5N5_COMPUTE[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_GFTT_B5N5_COMPUTE[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_GFTT_B5N5_COMPUTE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_GFTT_B5N5_COMPUTE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_GFTT_B5N5_COMPUTE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_GFTT_B5N5_COMPUTE[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_GFTT_B5N5_COMPUTE[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_GFTT_B5N5_COMPUTE[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_GFTT_B5N5_COMPUTE[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_GFTT_B5N5_COMPUTE[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_GFTT_B5N5_COMPUTE[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_GFTT_B5N5_COMPUTE[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_GFTT_B5N5_COMPUTE[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_GFTT_B5N5_COMPUTE[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_GFTT_B5N5_COMPUTE[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_GFTT_B5N5_COMPUTE[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_GFTT_B5N5_COMPUTE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 4, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_GFTT_B5N5_COMPUTE[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 3, 2, {0, 4, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_GFTT_B5N5_COMPUTE[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 3, 2, {0, 6, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_GFTT_B5N5_COMPUTE[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 3, 2, {0, 6, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_GFTT_B5N5_COMPUTE[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 3, 2, {0, 8, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_GFTT_B5N5_COMPUTE[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 3, 2, {0, 10, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_GFTT_B5N5_COMPUTE[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 3, 2, {0, 12, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData1_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData2_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData3_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData4_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData5_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData6_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData7_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData8_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData9_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData10_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData11_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData12_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData13_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData14_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData15_GFTT_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData16_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData17_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData18_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData19_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData20_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData21_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData22_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData23_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData24_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData25_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData26_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData27_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData28_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData29_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData30_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData31_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData32_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData33_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData34_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData35_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData36_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData37_GFTT_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData38_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData39_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData40_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData41_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData42_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData43_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData44_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData45_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData46_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData47_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData48_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData49_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData50_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData51_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData52_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData53_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData54_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData55_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData56_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData57_GFTT_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData58_GFTT_B5N5_COMPUTE[] = {{1, 1}};

static acf_scenario gScenarioArray_GFTT_B5N5_COMPUTE[] = {
{1, 1, 2772, 32, 5, gScenarioBufferData0_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData0_GFTT_B5N5_COMPUTE, 4},
{2, 1, 2812, 32, 5, gScenarioBufferData1_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData1_GFTT_B5N5_COMPUTE, 4},
{3, 1, 2852, 32, 5, gScenarioBufferData2_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData2_GFTT_B5N5_COMPUTE, 4},
{4, 1, 2892, 32, 5, gScenarioBufferData3_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData3_GFTT_B5N5_COMPUTE, 4},
{5, 1, 2932, 32, 5, gScenarioBufferData4_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData4_GFTT_B5N5_COMPUTE, 4},
{6, 1, 2972, 32, 5, gScenarioBufferData5_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData5_GFTT_B5N5_COMPUTE, 4},
{7, 1, 3012, 32, 5, gScenarioBufferData6_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData6_GFTT_B5N5_COMPUTE, 4},
{8, 1, 3052, 32, 5, gScenarioBufferData7_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData7_GFTT_B5N5_COMPUTE, 4},
{9, 1, 3092, 32, 5, gScenarioBufferData8_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData8_GFTT_B5N5_COMPUTE, 4},
{10, 1, 3132, 32, 5, gScenarioBufferData9_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData9_GFTT_B5N5_COMPUTE, 4},
{11, 1, 3172, 32, 5, gScenarioBufferData10_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData10_GFTT_B5N5_COMPUTE, 4},
{12, 1, 3212, 32, 5, gScenarioBufferData11_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData11_GFTT_B5N5_COMPUTE, 4},
{13, 1, 3252, 32, 5, gScenarioBufferData12_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData12_GFTT_B5N5_COMPUTE, 4},
{14, 1, 3292, 32, 5, gScenarioBufferData13_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData13_GFTT_B5N5_COMPUTE, 4},
{15, 1, 3332, 32, 5, gScenarioBufferData14_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData14_GFTT_B5N5_COMPUTE, 4},
{16, 1, 3372, 32, 5, gScenarioBufferData15_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData15_GFTT_B5N5_COMPUTE, 4},
{1, 2, 2820, 32, 3, gScenarioBufferData16_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData16_GFTT_B5N5_COMPUTE, 4},
{2, 2, 2892, 32, 3, gScenarioBufferData17_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData17_GFTT_B5N5_COMPUTE, 4},
{3, 2, 2964, 32, 3, gScenarioBufferData18_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData18_GFTT_B5N5_COMPUTE, 4},
{4, 2, 3036, 32, 3, gScenarioBufferData19_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData19_GFTT_B5N5_COMPUTE, 4},
{5, 2, 3108, 32, 3, gScenarioBufferData20_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData20_GFTT_B5N5_COMPUTE, 4},
{6, 2, 3180, 32, 3, gScenarioBufferData21_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData21_GFTT_B5N5_COMPUTE, 4},
{7, 2, 3252, 32, 3, gScenarioBufferData22_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData22_GFTT_B5N5_COMPUTE, 4},
{8, 2, 3324, 32, 3, gScenarioBufferData23_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData23_GFTT_B5N5_COMPUTE, 4},
{9, 2, 3396, 32, 3, gScenarioBufferData24_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData24_GFTT_B5N5_COMPUTE, 4},
{10, 2, 3468, 32, 3, gScenarioBufferData25_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData25_GFTT_B5N5_COMPUTE, 4},
{11, 2, 3540, 32, 3, gScenarioBufferData26_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData26_GFTT_B5N5_COMPUTE, 4},
{12, 2, 3612, 32, 3, gScenarioBufferData27_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData27_GFTT_B5N5_COMPUTE, 4},
{13, 2, 3684, 32, 3, gScenarioBufferData28_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData28_GFTT_B5N5_COMPUTE, 4},
{14, 2, 3756, 32, 3, gScenarioBufferData29_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData29_GFTT_B5N5_COMPUTE, 4},
{15, 2, 3828, 32, 3, gScenarioBufferData30_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData30_GFTT_B5N5_COMPUTE, 4},
{1, 3, 2908, 32, 3, gScenarioBufferData31_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData31_GFTT_B5N5_COMPUTE, 4},
{2, 3, 3052, 32, 3, gScenarioBufferData32_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData32_GFTT_B5N5_COMPUTE, 4},
{3, 3, 3196, 32, 3, gScenarioBufferData33_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData33_GFTT_B5N5_COMPUTE, 4},
{4, 3, 3340, 32, 3, gScenarioBufferData34_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData34_GFTT_B5N5_COMPUTE, 4},
{5, 3, 3484, 32, 3, gScenarioBufferData35_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData35_GFTT_B5N5_COMPUTE, 4},
{6, 3, 3628, 32, 3, gScenarioBufferData36_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData36_GFTT_B5N5_COMPUTE, 4},
{7, 3, 3772, 32, 3, gScenarioBufferData37_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData37_GFTT_B5N5_COMPUTE, 4},
{1, 4, 2916, 32, 2, gScenarioBufferData38_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData38_GFTT_B5N5_COMPUTE, 4},
{2, 4, 3052, 32, 2, gScenarioBufferData39_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData39_GFTT_B5N5_COMPUTE, 4},
{3, 4, 3188, 32, 2, gScenarioBufferData40_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData40_GFTT_B5N5_COMPUTE, 4},
{4, 4, 3324, 32, 2, gScenarioBufferData41_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData41_GFTT_B5N5_COMPUTE, 4},
{5, 4, 3460, 32, 2, gScenarioBufferData42_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData42_GFTT_B5N5_COMPUTE, 4},
{6, 4, 3596, 32, 2, gScenarioBufferData43_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData43_GFTT_B5N5_COMPUTE, 4},
{7, 4, 3732, 32, 2, gScenarioBufferData44_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData44_GFTT_B5N5_COMPUTE, 4},
{1, 5, 3012, 32, 2, gScenarioBufferData45_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData45_GFTT_B5N5_COMPUTE, 4},
{2, 5, 3228, 32, 2, gScenarioBufferData46_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData46_GFTT_B5N5_COMPUTE, 4},
{3, 5, 3444, 32, 2, gScenarioBufferData47_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData47_GFTT_B5N5_COMPUTE, 4},
{4, 5, 3660, 32, 2, gScenarioBufferData48_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData48_GFTT_B5N5_COMPUTE, 4},
{1, 6, 3076, 32, 2, gScenarioBufferData49_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData49_GFTT_B5N5_COMPUTE, 4},
{2, 6, 3340, 32, 2, gScenarioBufferData50_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData50_GFTT_B5N5_COMPUTE, 4},
{3, 6, 3604, 32, 2, gScenarioBufferData51_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData51_GFTT_B5N5_COMPUTE, 4},
{1, 8, 3204, 32, 2, gScenarioBufferData52_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData52_GFTT_B5N5_COMPUTE, 4},
{2, 8, 3564, 32, 2, gScenarioBufferData53_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData53_GFTT_B5N5_COMPUTE, 4},
{1, 10, 3332, 32, 2, gScenarioBufferData54_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData54_GFTT_B5N5_COMPUTE, 4},
{2, 10, 3788, 32, 2, gScenarioBufferData55_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData55_GFTT_B5N5_COMPUTE, 4},
{1, 12, 3460, 32, 2, gScenarioBufferData56_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData56_GFTT_B5N5_COMPUTE, 4},
{1, 14, 3588, 32, 2, gScenarioBufferData57_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData57_GFTT_B5N5_COMPUTE, 4},
{1, 16, 3716, 32, 2, gScenarioBufferData58_GFTT_B5N5_COMPUTE, 192, gScenarioKernelData58_GFTT_B5N5_COMPUTE, 4}
};

static acf_scenario_list gScenarioList_GFTT_B5N5_COMPUTE = {
59, //number of scenarios
gScenarioArray_GFTT_B5N5_COMPUTE};
//**************************************************************

class GFTT_B5N5_COMPUTE : public ACF_Process_APU
{

public:
   GFTT_B5N5_COMPUTE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("GFTT_B5N5_COMPUTE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("GFTT_B5N5_COMPUTE",
                                        GFTT_B5N5_COMPUTE_LOAD_SEGMENTS,
                                        GFTT_B5N5_COMPUTE_LOAD_PMEM, GFTT_B5N5_COMPUTE_LOAD_PMEM_SIZE,
                                        GFTT_B5N5_COMPUTE_LOAD_DMEM, GFTT_B5N5_COMPUTE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(GFTT_B5N5_COMPUTE_APEX_LOG_BUFFER, GFTT_B5N5_COMPUTE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 6, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_GFTT_B5N5_COMPUTE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_GFTT_B5N5_COMPUTE

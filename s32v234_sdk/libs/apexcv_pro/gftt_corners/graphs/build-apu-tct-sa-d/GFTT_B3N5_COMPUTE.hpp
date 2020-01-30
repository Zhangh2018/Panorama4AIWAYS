#ifndef _ACF_PROCESS_APU_GFTT_B3N5_COMPUTE
#define _ACF_PROCESS_APU_GFTT_B3N5_COMPUTE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <GFTT_B3N5_COMPUTE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_GFTT_B3N5_COMPUTE[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_GFTT_B3N5_COMPUTE[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_GFTT_B3N5_COMPUTE[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_GFTT_B3N5_COMPUTE[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_GFTT_B3N5_COMPUTE[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_GFTT_B3N5_COMPUTE[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_GFTT_B3N5_COMPUTE[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_GFTT_B3N5_COMPUTE[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_GFTT_B3N5_COMPUTE[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_GFTT_B3N5_COMPUTE[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_GFTT_B3N5_COMPUTE[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_GFTT_B3N5_COMPUTE[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_GFTT_B3N5_COMPUTE[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_GFTT_B3N5_COMPUTE[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_GFTT_B3N5_COMPUTE[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_GFTT_B3N5_COMPUTE[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 4, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_GFTT_B3N5_COMPUTE[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_GFTT_B3N5_COMPUTE[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_GFTT_B3N5_COMPUTE[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_GFTT_B3N5_COMPUTE[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_GFTT_B3N5_COMPUTE[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_GFTT_B3N5_COMPUTE[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_GFTT_B3N5_COMPUTE[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_GFTT_B3N5_COMPUTE[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_GFTT_B3N5_COMPUTE[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_GFTT_B3N5_COMPUTE[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_GFTT_B3N5_COMPUTE[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_GFTT_B3N5_COMPUTE[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_GFTT_B3N5_COMPUTE[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_GFTT_B3N5_COMPUTE[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_GFTT_B3N5_COMPUTE[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 3, 3, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_GFTT_B3N5_COMPUTE[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_GFTT_B3N5_COMPUTE[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_GFTT_B3N5_COMPUTE[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_GFTT_B3N5_COMPUTE[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_GFTT_B3N5_COMPUTE[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_GFTT_B3N5_COMPUTE[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_GFTT_B3N5_COMPUTE[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_GFTT_B3N5_COMPUTE[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_GFTT_B3N5_COMPUTE[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_GFTT_B3N5_COMPUTE[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_GFTT_B3N5_COMPUTE[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_GFTT_B3N5_COMPUTE[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_GFTT_B3N5_COMPUTE[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_GFTT_B3N5_COMPUTE[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_GFTT_B3N5_COMPUTE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_GFTT_B3N5_COMPUTE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_GFTT_B3N5_COMPUTE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_GFTT_B3N5_COMPUTE[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_GFTT_B3N5_COMPUTE[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_GFTT_B3N5_COMPUTE[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_GFTT_B3N5_COMPUTE[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_GFTT_B3N5_COMPUTE[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 3, 2, {0, 1, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_GFTT_B3N5_COMPUTE[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_GFTT_B3N5_COMPUTE[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_GFTT_B3N5_COMPUTE[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_GFTT_B3N5_COMPUTE[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_GFTT_B3N5_COMPUTE[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_GFTT_B3N5_COMPUTE[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 3, 2, {0, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_GFTT_B3N5_COMPUTE[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 3, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_GFTT_B3N5_COMPUTE[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 3, 2, {0, 3, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_GFTT_B3N5_COMPUTE[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 3, 2, {0, 3, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_GFTT_B3N5_COMPUTE[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 3, 2, {0, 3, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_GFTT_B3N5_COMPUTE[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 3, 2, {0, 3, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_GFTT_B3N5_COMPUTE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 5, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_GFTT_B3N5_COMPUTE[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 3, 2, {0, 5, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_GFTT_B3N5_COMPUTE[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 3, 2, {0, 5, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_GFTT_B3N5_COMPUTE[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 3, 2, {0, 7, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_GFTT_B3N5_COMPUTE[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 3, 2, {0, 7, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_GFTT_B3N5_COMPUTE[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 3, 2, {0, 9, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_GFTT_B3N5_COMPUTE[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 3, 2, {0, 9, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_GFTT_B3N5_COMPUTE[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 3, 2, {0, 11, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_GFTT_B3N5_COMPUTE[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 3, 2, {0, 13, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_GFTT_B3N5_COMPUTE[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 3, 2, {0, 15, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_GFTT_B3N5_COMPUTE[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 3, 2, {0, 17, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_GFTT_B3N5_COMPUTE[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 3, 2, {0, 19, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_GFTT_B3N5_COMPUTE[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 3, 2, {0, 21, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {34, 1, 1, 1, {0, 0, 0, 0}}, {1, 1, 1, 1, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData1_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData2_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData3_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData4_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData5_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData6_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData7_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData8_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData9_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData10_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData11_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData12_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData13_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData14_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData15_GFTT_B3N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData16_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData17_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData18_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData19_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData20_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData21_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData22_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData23_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData24_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData25_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData26_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData27_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData28_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData29_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData30_GFTT_B3N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData31_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData32_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData33_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData34_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData35_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData36_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData37_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData38_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData39_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData40_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData41_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData42_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData43_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData44_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData45_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData46_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData47_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData48_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData49_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData50_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData51_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData52_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData53_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData54_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData55_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData56_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData57_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData58_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData59_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData60_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData61_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData62_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData63_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData64_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData65_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData66_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData67_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData68_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData69_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData70_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData71_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData72_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData73_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData74_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData75_GFTT_B3N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData76_GFTT_B3N5_COMPUTE[] = {{1, 1}};

static acf_scenario gScenarioArray_GFTT_B3N5_COMPUTE[] = {
{1, 1, 2364, 32, 4, gScenarioBufferData0_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData0_GFTT_B3N5_COMPUTE, 4},
{2, 1, 2404, 32, 4, gScenarioBufferData1_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData1_GFTT_B3N5_COMPUTE, 4},
{3, 1, 2444, 32, 4, gScenarioBufferData2_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData2_GFTT_B3N5_COMPUTE, 4},
{4, 1, 2484, 32, 4, gScenarioBufferData3_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData3_GFTT_B3N5_COMPUTE, 4},
{5, 1, 2524, 32, 4, gScenarioBufferData4_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData4_GFTT_B3N5_COMPUTE, 4},
{6, 1, 2564, 32, 4, gScenarioBufferData5_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData5_GFTT_B3N5_COMPUTE, 4},
{7, 1, 2604, 32, 4, gScenarioBufferData6_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData6_GFTT_B3N5_COMPUTE, 4},
{8, 1, 2644, 32, 4, gScenarioBufferData7_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData7_GFTT_B3N5_COMPUTE, 4},
{9, 1, 2684, 32, 4, gScenarioBufferData8_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData8_GFTT_B3N5_COMPUTE, 4},
{10, 1, 2724, 32, 4, gScenarioBufferData9_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData9_GFTT_B3N5_COMPUTE, 4},
{11, 1, 2764, 32, 4, gScenarioBufferData10_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData10_GFTT_B3N5_COMPUTE, 4},
{12, 1, 2804, 32, 4, gScenarioBufferData11_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData11_GFTT_B3N5_COMPUTE, 4},
{13, 1, 2844, 32, 4, gScenarioBufferData12_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData12_GFTT_B3N5_COMPUTE, 4},
{14, 1, 2884, 32, 4, gScenarioBufferData13_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData13_GFTT_B3N5_COMPUTE, 4},
{15, 1, 2924, 32, 4, gScenarioBufferData14_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData14_GFTT_B3N5_COMPUTE, 4},
{16, 1, 2964, 32, 4, gScenarioBufferData15_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData15_GFTT_B3N5_COMPUTE, 4},
{1, 2, 2436, 32, 3, gScenarioBufferData16_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData16_GFTT_B3N5_COMPUTE, 4},
{2, 2, 2532, 32, 3, gScenarioBufferData17_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData17_GFTT_B3N5_COMPUTE, 4},
{3, 2, 2628, 32, 3, gScenarioBufferData18_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData18_GFTT_B3N5_COMPUTE, 4},
{4, 2, 2724, 32, 3, gScenarioBufferData19_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData19_GFTT_B3N5_COMPUTE, 4},
{5, 2, 2820, 32, 3, gScenarioBufferData20_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData20_GFTT_B3N5_COMPUTE, 4},
{6, 2, 2916, 32, 3, gScenarioBufferData21_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData21_GFTT_B3N5_COMPUTE, 4},
{7, 2, 3012, 32, 3, gScenarioBufferData22_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData22_GFTT_B3N5_COMPUTE, 4},
{8, 2, 3108, 32, 3, gScenarioBufferData23_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData23_GFTT_B3N5_COMPUTE, 4},
{9, 2, 3204, 32, 3, gScenarioBufferData24_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData24_GFTT_B3N5_COMPUTE, 4},
{10, 2, 3300, 32, 3, gScenarioBufferData25_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData25_GFTT_B3N5_COMPUTE, 4},
{11, 2, 3396, 32, 3, gScenarioBufferData26_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData26_GFTT_B3N5_COMPUTE, 4},
{12, 2, 3492, 32, 3, gScenarioBufferData27_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData27_GFTT_B3N5_COMPUTE, 4},
{13, 2, 3588, 32, 3, gScenarioBufferData28_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData28_GFTT_B3N5_COMPUTE, 4},
{14, 2, 3684, 32, 3, gScenarioBufferData29_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData29_GFTT_B3N5_COMPUTE, 4},
{15, 2, 3780, 32, 3, gScenarioBufferData30_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData30_GFTT_B3N5_COMPUTE, 4},
{1, 3, 2460, 32, 2, gScenarioBufferData31_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData31_GFTT_B3N5_COMPUTE, 4},
{2, 3, 2564, 32, 2, gScenarioBufferData32_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData32_GFTT_B3N5_COMPUTE, 4},
{3, 3, 2668, 32, 2, gScenarioBufferData33_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData33_GFTT_B3N5_COMPUTE, 4},
{4, 3, 2772, 32, 2, gScenarioBufferData34_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData34_GFTT_B3N5_COMPUTE, 4},
{5, 3, 2876, 32, 2, gScenarioBufferData35_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData35_GFTT_B3N5_COMPUTE, 4},
{6, 3, 2980, 32, 2, gScenarioBufferData36_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData36_GFTT_B3N5_COMPUTE, 4},
{7, 3, 3084, 32, 2, gScenarioBufferData37_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData37_GFTT_B3N5_COMPUTE, 4},
{8, 3, 3188, 32, 2, gScenarioBufferData38_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData38_GFTT_B3N5_COMPUTE, 4},
{9, 3, 3292, 32, 2, gScenarioBufferData39_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData39_GFTT_B3N5_COMPUTE, 4},
{10, 3, 3396, 32, 2, gScenarioBufferData40_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData40_GFTT_B3N5_COMPUTE, 4},
{11, 3, 3500, 32, 2, gScenarioBufferData41_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData41_GFTT_B3N5_COMPUTE, 4},
{12, 3, 3604, 32, 2, gScenarioBufferData42_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData42_GFTT_B3N5_COMPUTE, 4},
{13, 3, 3708, 32, 2, gScenarioBufferData43_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData43_GFTT_B3N5_COMPUTE, 4},
{14, 3, 3812, 32, 2, gScenarioBufferData44_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData44_GFTT_B3N5_COMPUTE, 4},
{1, 4, 2548, 32, 2, gScenarioBufferData45_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData45_GFTT_B3N5_COMPUTE, 4},
{2, 4, 2724, 32, 2, gScenarioBufferData46_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData46_GFTT_B3N5_COMPUTE, 4},
{3, 4, 2900, 32, 2, gScenarioBufferData47_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData47_GFTT_B3N5_COMPUTE, 4},
{4, 4, 3076, 32, 2, gScenarioBufferData48_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData48_GFTT_B3N5_COMPUTE, 4},
{5, 4, 3252, 32, 2, gScenarioBufferData49_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData49_GFTT_B3N5_COMPUTE, 4},
{6, 4, 3428, 32, 2, gScenarioBufferData50_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData50_GFTT_B3N5_COMPUTE, 4},
{7, 4, 3604, 32, 2, gScenarioBufferData51_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData51_GFTT_B3N5_COMPUTE, 4},
{8, 4, 3780, 32, 2, gScenarioBufferData52_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData52_GFTT_B3N5_COMPUTE, 4},
{1, 5, 2612, 32, 2, gScenarioBufferData53_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData53_GFTT_B3N5_COMPUTE, 4},
{2, 5, 2836, 32, 2, gScenarioBufferData54_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData54_GFTT_B3N5_COMPUTE, 4},
{3, 5, 3060, 32, 2, gScenarioBufferData55_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData55_GFTT_B3N5_COMPUTE, 4},
{4, 5, 3284, 32, 2, gScenarioBufferData56_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData56_GFTT_B3N5_COMPUTE, 4},
{5, 5, 3508, 32, 2, gScenarioBufferData57_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData57_GFTT_B3N5_COMPUTE, 4},
{6, 5, 3732, 32, 2, gScenarioBufferData58_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData58_GFTT_B3N5_COMPUTE, 4},
{1, 6, 2676, 32, 2, gScenarioBufferData59_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData59_GFTT_B3N5_COMPUTE, 4},
{2, 6, 2948, 32, 2, gScenarioBufferData60_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData60_GFTT_B3N5_COMPUTE, 4},
{3, 6, 3220, 32, 2, gScenarioBufferData61_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData61_GFTT_B3N5_COMPUTE, 4},
{4, 6, 3492, 32, 2, gScenarioBufferData62_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData62_GFTT_B3N5_COMPUTE, 4},
{5, 6, 3764, 32, 2, gScenarioBufferData63_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData63_GFTT_B3N5_COMPUTE, 4},
{1, 8, 2804, 32, 2, gScenarioBufferData64_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData64_GFTT_B3N5_COMPUTE, 4},
{2, 8, 3172, 32, 2, gScenarioBufferData65_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData65_GFTT_B3N5_COMPUTE, 4},
{3, 8, 3540, 32, 2, gScenarioBufferData66_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData66_GFTT_B3N5_COMPUTE, 4},
{1, 10, 2932, 32, 2, gScenarioBufferData67_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData67_GFTT_B3N5_COMPUTE, 4},
{2, 10, 3396, 32, 2, gScenarioBufferData68_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData68_GFTT_B3N5_COMPUTE, 4},
{1, 12, 3060, 32, 2, gScenarioBufferData69_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData69_GFTT_B3N5_COMPUTE, 4},
{2, 12, 3620, 32, 2, gScenarioBufferData70_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData70_GFTT_B3N5_COMPUTE, 4},
{1, 14, 3188, 32, 2, gScenarioBufferData71_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData71_GFTT_B3N5_COMPUTE, 4},
{1, 16, 3316, 32, 2, gScenarioBufferData72_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData72_GFTT_B3N5_COMPUTE, 4},
{1, 18, 3444, 32, 2, gScenarioBufferData73_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData73_GFTT_B3N5_COMPUTE, 4},
{1, 20, 3572, 32, 2, gScenarioBufferData74_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData74_GFTT_B3N5_COMPUTE, 4},
{1, 22, 3700, 32, 2, gScenarioBufferData75_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData75_GFTT_B3N5_COMPUTE, 4},
{1, 24, 3828, 32, 2, gScenarioBufferData76_GFTT_B3N5_COMPUTE, 192, gScenarioKernelData76_GFTT_B3N5_COMPUTE, 4}
};

static acf_scenario_list gScenarioList_GFTT_B3N5_COMPUTE = {
77, //number of scenarios
gScenarioArray_GFTT_B3N5_COMPUTE};
//**************************************************************

class GFTT_B3N5_COMPUTE : public ACF_Process_APU
{

public:
   GFTT_B3N5_COMPUTE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("GFTT_B3N5_COMPUTE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("GFTT_B3N5_COMPUTE",
                                        GFTT_B3N5_COMPUTE_LOAD_SEGMENTS,
                                        GFTT_B3N5_COMPUTE_LOAD_PMEM, GFTT_B3N5_COMPUTE_LOAD_PMEM_SIZE,
                                        GFTT_B3N5_COMPUTE_LOAD_DMEM, GFTT_B3N5_COMPUTE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(GFTT_B3N5_COMPUTE_APEX_LOG_BUFFER, GFTT_B3N5_COMPUTE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 6, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_GFTT_B3N5_COMPUTE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_GFTT_B3N5_COMPUTE

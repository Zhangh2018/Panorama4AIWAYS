#ifndef _ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE
#define _ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HARRIS_B7N5_COMPUTE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HARRIS_B7N5_COMPUTE[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HARRIS_B7N5_COMPUTE[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HARRIS_B7N5_COMPUTE[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HARRIS_B7N5_COMPUTE[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HARRIS_B7N5_COMPUTE[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HARRIS_B7N5_COMPUTE[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HARRIS_B7N5_COMPUTE[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HARRIS_B7N5_COMPUTE[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HARRIS_B7N5_COMPUTE[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HARRIS_B7N5_COMPUTE[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HARRIS_B7N5_COMPUTE[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HARRIS_B7N5_COMPUTE[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HARRIS_B7N5_COMPUTE[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HARRIS_B7N5_COMPUTE[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HARRIS_B7N5_COMPUTE[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HARRIS_B7N5_COMPUTE[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HARRIS_B7N5_COMPUTE[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HARRIS_B7N5_COMPUTE[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HARRIS_B7N5_COMPUTE[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HARRIS_B7N5_COMPUTE[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HARRIS_B7N5_COMPUTE[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HARRIS_B7N5_COMPUTE[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HARRIS_B7N5_COMPUTE[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 3, 4, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HARRIS_B7N5_COMPUTE[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HARRIS_B7N5_COMPUTE[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HARRIS_B7N5_COMPUTE[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HARRIS_B7N5_COMPUTE[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HARRIS_B7N5_COMPUTE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HARRIS_B7N5_COMPUTE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HARRIS_B7N5_COMPUTE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 3, 3, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HARRIS_B7N5_COMPUTE[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HARRIS_B7N5_COMPUTE[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HARRIS_B7N5_COMPUTE[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HARRIS_B7N5_COMPUTE[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HARRIS_B7N5_COMPUTE[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 3, 2, {0, 1, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HARRIS_B7N5_COMPUTE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 3, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HARRIS_B7N5_COMPUTE[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 3, 2, {0, 5, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData1_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData2_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData3_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData4_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData5_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData6_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData7_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData8_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData9_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData10_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData11_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData12_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData13_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData14_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData15_HARRIS_B7N5_COMPUTE[] = {{1, 5}};
static acf_scenario_kernel_data gScenarioKernelData16_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData17_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData18_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData19_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData20_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData21_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData22_HARRIS_B7N5_COMPUTE[] = {{1, 3}};
static acf_scenario_kernel_data gScenarioKernelData23_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData24_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData25_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData26_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData27_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData28_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData29_HARRIS_B7N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData30_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData31_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData32_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData33_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData34_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData35_HARRIS_B7N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData36_HARRIS_B7N5_COMPUTE[] = {{1, 1}};

static acf_scenario gScenarioArray_HARRIS_B7N5_COMPUTE[] = {
{1, 1, 3176, 32, 6, gScenarioBufferData0_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData0_HARRIS_B7N5_COMPUTE, 4},
{2, 1, 3216, 32, 6, gScenarioBufferData1_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData1_HARRIS_B7N5_COMPUTE, 4},
{3, 1, 3256, 32, 6, gScenarioBufferData2_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData2_HARRIS_B7N5_COMPUTE, 4},
{4, 1, 3296, 32, 6, gScenarioBufferData3_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData3_HARRIS_B7N5_COMPUTE, 4},
{5, 1, 3336, 32, 6, gScenarioBufferData4_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData4_HARRIS_B7N5_COMPUTE, 4},
{6, 1, 3376, 32, 6, gScenarioBufferData5_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData5_HARRIS_B7N5_COMPUTE, 4},
{7, 1, 3416, 32, 6, gScenarioBufferData6_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData6_HARRIS_B7N5_COMPUTE, 4},
{8, 1, 3456, 32, 6, gScenarioBufferData7_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData7_HARRIS_B7N5_COMPUTE, 4},
{9, 1, 3496, 32, 6, gScenarioBufferData8_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData8_HARRIS_B7N5_COMPUTE, 4},
{10, 1, 3536, 32, 6, gScenarioBufferData9_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData9_HARRIS_B7N5_COMPUTE, 4},
{11, 1, 3576, 32, 6, gScenarioBufferData10_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData10_HARRIS_B7N5_COMPUTE, 4},
{12, 1, 3616, 32, 6, gScenarioBufferData11_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData11_HARRIS_B7N5_COMPUTE, 4},
{13, 1, 3656, 32, 6, gScenarioBufferData12_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData12_HARRIS_B7N5_COMPUTE, 4},
{14, 1, 3696, 32, 6, gScenarioBufferData13_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData13_HARRIS_B7N5_COMPUTE, 4},
{15, 1, 3736, 32, 6, gScenarioBufferData14_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData14_HARRIS_B7N5_COMPUTE, 4},
{16, 1, 3776, 32, 6, gScenarioBufferData15_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData15_HARRIS_B7N5_COMPUTE, 4},
{1, 2, 3248, 32, 4, gScenarioBufferData16_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData16_HARRIS_B7N5_COMPUTE, 4},
{2, 2, 3344, 32, 4, gScenarioBufferData17_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData17_HARRIS_B7N5_COMPUTE, 4},
{3, 2, 3440, 32, 4, gScenarioBufferData18_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData18_HARRIS_B7N5_COMPUTE, 4},
{4, 2, 3536, 32, 4, gScenarioBufferData19_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData19_HARRIS_B7N5_COMPUTE, 4},
{5, 2, 3632, 32, 4, gScenarioBufferData20_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData20_HARRIS_B7N5_COMPUTE, 4},
{6, 2, 3728, 32, 4, gScenarioBufferData21_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData21_HARRIS_B7N5_COMPUTE, 4},
{7, 2, 3824, 32, 4, gScenarioBufferData22_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData22_HARRIS_B7N5_COMPUTE, 4},
{1, 3, 3304, 32, 3, gScenarioBufferData23_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData23_HARRIS_B7N5_COMPUTE, 4},
{2, 3, 3440, 32, 3, gScenarioBufferData24_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData24_HARRIS_B7N5_COMPUTE, 4},
{3, 3, 3576, 32, 3, gScenarioBufferData25_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData25_HARRIS_B7N5_COMPUTE, 4},
{4, 3, 3712, 32, 3, gScenarioBufferData26_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData26_HARRIS_B7N5_COMPUTE, 4},
{1, 4, 3376, 32, 3, gScenarioBufferData27_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData27_HARRIS_B7N5_COMPUTE, 4},
{2, 4, 3568, 32, 3, gScenarioBufferData28_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData28_HARRIS_B7N5_COMPUTE, 4},
{3, 4, 3760, 32, 3, gScenarioBufferData29_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData29_HARRIS_B7N5_COMPUTE, 4},
{1, 5, 3368, 32, 2, gScenarioBufferData30_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData30_HARRIS_B7N5_COMPUTE, 4},
{2, 5, 3536, 32, 2, gScenarioBufferData31_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData31_HARRIS_B7N5_COMPUTE, 4},
{3, 5, 3704, 32, 2, gScenarioBufferData32_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData32_HARRIS_B7N5_COMPUTE, 4},
{1, 6, 3472, 32, 2, gScenarioBufferData33_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData33_HARRIS_B7N5_COMPUTE, 4},
{2, 6, 3728, 32, 2, gScenarioBufferData34_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData34_HARRIS_B7N5_COMPUTE, 4},
{1, 8, 3600, 32, 2, gScenarioBufferData35_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData35_HARRIS_B7N5_COMPUTE, 4},
{1, 10, 3728, 32, 2, gScenarioBufferData36_HARRIS_B7N5_COMPUTE, 176, gScenarioKernelData36_HARRIS_B7N5_COMPUTE, 4}
};

static acf_scenario_list gScenarioList_HARRIS_B7N5_COMPUTE = {
37, //number of scenarios
gScenarioArray_HARRIS_B7N5_COMPUTE};
//**************************************************************

class HARRIS_B7N5_COMPUTE : public ACF_Process_APU
{

public:
   HARRIS_B7N5_COMPUTE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HARRIS_B7N5_COMPUTE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HARRIS_B7N5_COMPUTE",
                                        HARRIS_B7N5_COMPUTE_LOAD_SEGMENTS,
                                        HARRIS_B7N5_COMPUTE_LOAD_PMEM, HARRIS_B7N5_COMPUTE_LOAD_PMEM_SIZE,
                                        HARRIS_B7N5_COMPUTE_LOAD_DMEM, HARRIS_B7N5_COMPUTE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HARRIS_B7N5_COMPUTE_APEX_LOG_BUFFER, HARRIS_B7N5_COMPUTE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 7, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HARRIS_B7N5_COMPUTE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE

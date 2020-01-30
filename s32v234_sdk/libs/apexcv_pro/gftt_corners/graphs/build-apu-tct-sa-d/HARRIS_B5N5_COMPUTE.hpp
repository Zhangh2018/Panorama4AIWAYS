#ifndef _ACF_PROCESS_APU_HARRIS_B5N5_COMPUTE
#define _ACF_PROCESS_APU_HARRIS_B5N5_COMPUTE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HARRIS_B5N5_COMPUTE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HARRIS_B5N5_COMPUTE[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HARRIS_B5N5_COMPUTE[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HARRIS_B5N5_COMPUTE[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HARRIS_B5N5_COMPUTE[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HARRIS_B5N5_COMPUTE[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HARRIS_B5N5_COMPUTE[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HARRIS_B5N5_COMPUTE[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HARRIS_B5N5_COMPUTE[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HARRIS_B5N5_COMPUTE[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HARRIS_B5N5_COMPUTE[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HARRIS_B5N5_COMPUTE[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HARRIS_B5N5_COMPUTE[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HARRIS_B5N5_COMPUTE[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HARRIS_B5N5_COMPUTE[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HARRIS_B5N5_COMPUTE[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HARRIS_B5N5_COMPUTE[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 5, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HARRIS_B5N5_COMPUTE[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HARRIS_B5N5_COMPUTE[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HARRIS_B5N5_COMPUTE[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HARRIS_B5N5_COMPUTE[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HARRIS_B5N5_COMPUTE[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HARRIS_B5N5_COMPUTE[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HARRIS_B5N5_COMPUTE[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HARRIS_B5N5_COMPUTE[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HARRIS_B5N5_COMPUTE[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HARRIS_B5N5_COMPUTE[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HARRIS_B5N5_COMPUTE[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HARRIS_B5N5_COMPUTE[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HARRIS_B5N5_COMPUTE[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HARRIS_B5N5_COMPUTE[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HARRIS_B5N5_COMPUTE[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 3, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HARRIS_B5N5_COMPUTE[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HARRIS_B5N5_COMPUTE[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HARRIS_B5N5_COMPUTE[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HARRIS_B5N5_COMPUTE[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HARRIS_B5N5_COMPUTE[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HARRIS_B5N5_COMPUTE[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_HARRIS_B5N5_COMPUTE[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 3, 3, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_HARRIS_B5N5_COMPUTE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_HARRIS_B5N5_COMPUTE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_HARRIS_B5N5_COMPUTE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_HARRIS_B5N5_COMPUTE[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_HARRIS_B5N5_COMPUTE[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_HARRIS_B5N5_COMPUTE[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_HARRIS_B5N5_COMPUTE[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 2, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_HARRIS_B5N5_COMPUTE[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_HARRIS_B5N5_COMPUTE[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_HARRIS_B5N5_COMPUTE[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_HARRIS_B5N5_COMPUTE[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 3, 2, {0, 1, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_HARRIS_B5N5_COMPUTE[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_HARRIS_B5N5_COMPUTE[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_HARRIS_B5N5_COMPUTE[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 3, 2, {0, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_HARRIS_B5N5_COMPUTE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 4, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_HARRIS_B5N5_COMPUTE[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 3, 2, {0, 4, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_HARRIS_B5N5_COMPUTE[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 3, 2, {0, 6, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_HARRIS_B5N5_COMPUTE[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 3, 2, {0, 6, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_HARRIS_B5N5_COMPUTE[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 3, 2, {0, 8, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_HARRIS_B5N5_COMPUTE[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 3, 2, {0, 10, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_HARRIS_B5N5_COMPUTE[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 3, 2, {0, 12, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_HARRIS_B5N5_COMPUTE[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 3, 2, {0, 14, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {32, 6, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {36, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData1_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData2_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData3_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData4_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData5_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData6_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData7_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData8_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData9_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData10_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData11_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData12_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData13_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData14_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData15_HARRIS_B5N5_COMPUTE[] = {{1, 4}};
static acf_scenario_kernel_data gScenarioKernelData16_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData17_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData18_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData19_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData20_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData21_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData22_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData23_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData24_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData25_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData26_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData27_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData28_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData29_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData30_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData31_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData32_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData33_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData34_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData35_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData36_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData37_HARRIS_B5N5_COMPUTE[] = {{1, 2}};
static acf_scenario_kernel_data gScenarioKernelData38_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData39_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData40_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData41_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData42_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData43_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData44_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData45_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData46_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData47_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData48_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData49_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData50_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData51_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData52_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData53_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData54_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData55_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData56_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData57_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData58_HARRIS_B5N5_COMPUTE[] = {{1, 1}};
static acf_scenario_kernel_data gScenarioKernelData59_HARRIS_B5N5_COMPUTE[] = {{1, 1}};

static acf_scenario gScenarioArray_HARRIS_B5N5_COMPUTE[] = {
{1, 1, 2768, 32, 5, gScenarioBufferData0_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData0_HARRIS_B5N5_COMPUTE, 4},
{2, 1, 2808, 32, 5, gScenarioBufferData1_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData1_HARRIS_B5N5_COMPUTE, 4},
{3, 1, 2848, 32, 5, gScenarioBufferData2_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData2_HARRIS_B5N5_COMPUTE, 4},
{4, 1, 2888, 32, 5, gScenarioBufferData3_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData3_HARRIS_B5N5_COMPUTE, 4},
{5, 1, 2928, 32, 5, gScenarioBufferData4_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData4_HARRIS_B5N5_COMPUTE, 4},
{6, 1, 2968, 32, 5, gScenarioBufferData5_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData5_HARRIS_B5N5_COMPUTE, 4},
{7, 1, 3008, 32, 5, gScenarioBufferData6_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData6_HARRIS_B5N5_COMPUTE, 4},
{8, 1, 3048, 32, 5, gScenarioBufferData7_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData7_HARRIS_B5N5_COMPUTE, 4},
{9, 1, 3088, 32, 5, gScenarioBufferData8_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData8_HARRIS_B5N5_COMPUTE, 4},
{10, 1, 3128, 32, 5, gScenarioBufferData9_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData9_HARRIS_B5N5_COMPUTE, 4},
{11, 1, 3168, 32, 5, gScenarioBufferData10_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData10_HARRIS_B5N5_COMPUTE, 4},
{12, 1, 3208, 32, 5, gScenarioBufferData11_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData11_HARRIS_B5N5_COMPUTE, 4},
{13, 1, 3248, 32, 5, gScenarioBufferData12_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData12_HARRIS_B5N5_COMPUTE, 4},
{14, 1, 3288, 32, 5, gScenarioBufferData13_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData13_HARRIS_B5N5_COMPUTE, 4},
{15, 1, 3328, 32, 5, gScenarioBufferData14_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData14_HARRIS_B5N5_COMPUTE, 4},
{16, 1, 3368, 32, 5, gScenarioBufferData15_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData15_HARRIS_B5N5_COMPUTE, 4},
{1, 2, 2816, 32, 3, gScenarioBufferData16_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData16_HARRIS_B5N5_COMPUTE, 4},
{2, 2, 2888, 32, 3, gScenarioBufferData17_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData17_HARRIS_B5N5_COMPUTE, 4},
{3, 2, 2960, 32, 3, gScenarioBufferData18_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData18_HARRIS_B5N5_COMPUTE, 4},
{4, 2, 3032, 32, 3, gScenarioBufferData19_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData19_HARRIS_B5N5_COMPUTE, 4},
{5, 2, 3104, 32, 3, gScenarioBufferData20_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData20_HARRIS_B5N5_COMPUTE, 4},
{6, 2, 3176, 32, 3, gScenarioBufferData21_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData21_HARRIS_B5N5_COMPUTE, 4},
{7, 2, 3248, 32, 3, gScenarioBufferData22_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData22_HARRIS_B5N5_COMPUTE, 4},
{8, 2, 3320, 32, 3, gScenarioBufferData23_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData23_HARRIS_B5N5_COMPUTE, 4},
{9, 2, 3392, 32, 3, gScenarioBufferData24_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData24_HARRIS_B5N5_COMPUTE, 4},
{10, 2, 3464, 32, 3, gScenarioBufferData25_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData25_HARRIS_B5N5_COMPUTE, 4},
{11, 2, 3536, 32, 3, gScenarioBufferData26_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData26_HARRIS_B5N5_COMPUTE, 4},
{12, 2, 3608, 32, 3, gScenarioBufferData27_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData27_HARRIS_B5N5_COMPUTE, 4},
{13, 2, 3680, 32, 3, gScenarioBufferData28_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData28_HARRIS_B5N5_COMPUTE, 4},
{14, 2, 3752, 32, 3, gScenarioBufferData29_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData29_HARRIS_B5N5_COMPUTE, 4},
{15, 2, 3824, 32, 3, gScenarioBufferData30_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData30_HARRIS_B5N5_COMPUTE, 4},
{1, 3, 2904, 32, 3, gScenarioBufferData31_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData31_HARRIS_B5N5_COMPUTE, 4},
{2, 3, 3048, 32, 3, gScenarioBufferData32_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData32_HARRIS_B5N5_COMPUTE, 4},
{3, 3, 3192, 32, 3, gScenarioBufferData33_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData33_HARRIS_B5N5_COMPUTE, 4},
{4, 3, 3336, 32, 3, gScenarioBufferData34_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData34_HARRIS_B5N5_COMPUTE, 4},
{5, 3, 3480, 32, 3, gScenarioBufferData35_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData35_HARRIS_B5N5_COMPUTE, 4},
{6, 3, 3624, 32, 3, gScenarioBufferData36_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData36_HARRIS_B5N5_COMPUTE, 4},
{7, 3, 3768, 32, 3, gScenarioBufferData37_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData37_HARRIS_B5N5_COMPUTE, 4},
{1, 4, 2912, 32, 2, gScenarioBufferData38_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData38_HARRIS_B5N5_COMPUTE, 4},
{2, 4, 3048, 32, 2, gScenarioBufferData39_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData39_HARRIS_B5N5_COMPUTE, 4},
{3, 4, 3184, 32, 2, gScenarioBufferData40_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData40_HARRIS_B5N5_COMPUTE, 4},
{4, 4, 3320, 32, 2, gScenarioBufferData41_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData41_HARRIS_B5N5_COMPUTE, 4},
{5, 4, 3456, 32, 2, gScenarioBufferData42_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData42_HARRIS_B5N5_COMPUTE, 4},
{6, 4, 3592, 32, 2, gScenarioBufferData43_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData43_HARRIS_B5N5_COMPUTE, 4},
{7, 4, 3728, 32, 2, gScenarioBufferData44_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData44_HARRIS_B5N5_COMPUTE, 4},
{1, 5, 3008, 32, 2, gScenarioBufferData45_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData45_HARRIS_B5N5_COMPUTE, 4},
{2, 5, 3224, 32, 2, gScenarioBufferData46_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData46_HARRIS_B5N5_COMPUTE, 4},
{3, 5, 3440, 32, 2, gScenarioBufferData47_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData47_HARRIS_B5N5_COMPUTE, 4},
{4, 5, 3656, 32, 2, gScenarioBufferData48_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData48_HARRIS_B5N5_COMPUTE, 4},
{1, 6, 3072, 32, 2, gScenarioBufferData49_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData49_HARRIS_B5N5_COMPUTE, 4},
{2, 6, 3336, 32, 2, gScenarioBufferData50_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData50_HARRIS_B5N5_COMPUTE, 4},
{3, 6, 3600, 32, 2, gScenarioBufferData51_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData51_HARRIS_B5N5_COMPUTE, 4},
{1, 8, 3200, 32, 2, gScenarioBufferData52_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData52_HARRIS_B5N5_COMPUTE, 4},
{2, 8, 3560, 32, 2, gScenarioBufferData53_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData53_HARRIS_B5N5_COMPUTE, 4},
{1, 10, 3328, 32, 2, gScenarioBufferData54_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData54_HARRIS_B5N5_COMPUTE, 4},
{2, 10, 3784, 32, 2, gScenarioBufferData55_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData55_HARRIS_B5N5_COMPUTE, 4},
{1, 12, 3456, 32, 2, gScenarioBufferData56_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData56_HARRIS_B5N5_COMPUTE, 4},
{1, 14, 3584, 32, 2, gScenarioBufferData57_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData57_HARRIS_B5N5_COMPUTE, 4},
{1, 16, 3712, 32, 2, gScenarioBufferData58_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData58_HARRIS_B5N5_COMPUTE, 4},
{1, 18, 3840, 32, 2, gScenarioBufferData59_HARRIS_B5N5_COMPUTE, 176, gScenarioKernelData59_HARRIS_B5N5_COMPUTE, 4}
};

static acf_scenario_list gScenarioList_HARRIS_B5N5_COMPUTE = {
60, //number of scenarios
gScenarioArray_HARRIS_B5N5_COMPUTE};
//**************************************************************

class HARRIS_B5N5_COMPUTE : public ACF_Process_APU
{

public:
   HARRIS_B5N5_COMPUTE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HARRIS_B5N5_COMPUTE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HARRIS_B5N5_COMPUTE",
                                        HARRIS_B5N5_COMPUTE_LOAD_SEGMENTS,
                                        HARRIS_B5N5_COMPUTE_LOAD_PMEM, HARRIS_B5N5_COMPUTE_LOAD_PMEM_SIZE,
                                        HARRIS_B5N5_COMPUTE_LOAD_DMEM, HARRIS_B5N5_COMPUTE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HARRIS_B5N5_COMPUTE_APEX_LOG_BUFFER, HARRIS_B5N5_COMPUTE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 7, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HARRIS_B5N5_COMPUTE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HARRIS_B5N5_COMPUTE

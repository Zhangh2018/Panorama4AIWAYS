#ifndef _ACF_PROCESS_APU_THRESHOLD_RANGE_16U
#define _ACF_PROCESS_APU_THRESHOLD_RANGE_16U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <THRESHOLD_RANGE_16U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_THRESHOLD_RANGE_16U[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_THRESHOLD_RANGE_16U[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_THRESHOLD_RANGE_16U[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_THRESHOLD_RANGE_16U[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_THRESHOLD_RANGE_16U[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_THRESHOLD_RANGE_16U[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_THRESHOLD_RANGE_16U[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_THRESHOLD_RANGE_16U[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_THRESHOLD_RANGE_16U[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_THRESHOLD_RANGE_16U[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_THRESHOLD_RANGE_16U[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_THRESHOLD_RANGE_16U[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_THRESHOLD_RANGE_16U[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_THRESHOLD_RANGE_16U[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_THRESHOLD_RANGE_16U[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_THRESHOLD_RANGE_16U[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_THRESHOLD_RANGE_16U[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_THRESHOLD_RANGE_16U[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_THRESHOLD_RANGE_16U[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_THRESHOLD_RANGE_16U[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_THRESHOLD_RANGE_16U[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_THRESHOLD_RANGE_16U[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_THRESHOLD_RANGE_16U[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_THRESHOLD_RANGE_16U[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_THRESHOLD_RANGE_16U[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_THRESHOLD_RANGE_16U[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_THRESHOLD_RANGE_16U[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_THRESHOLD_RANGE_16U[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_THRESHOLD_RANGE_16U[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_THRESHOLD_RANGE_16U[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_THRESHOLD_RANGE_16U[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_THRESHOLD_RANGE_16U[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_THRESHOLD_RANGE_16U[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_THRESHOLD_RANGE_16U[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_THRESHOLD_RANGE_16U[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_THRESHOLD_RANGE_16U[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_THRESHOLD_RANGE_16U[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_THRESHOLD_RANGE_16U[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_THRESHOLD_RANGE_16U[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_THRESHOLD_RANGE_16U[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_THRESHOLD_RANGE_16U[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_THRESHOLD_RANGE_16U[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_THRESHOLD_RANGE_16U[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_THRESHOLD_RANGE_16U[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_THRESHOLD_RANGE_16U[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_THRESHOLD_RANGE_16U[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_THRESHOLD_RANGE_16U[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_THRESHOLD_RANGE_16U[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_THRESHOLD_RANGE_16U[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_THRESHOLD_RANGE_16U[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_THRESHOLD_RANGE_16U[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_THRESHOLD_RANGE_16U[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_THRESHOLD_RANGE_16U[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_THRESHOLD_RANGE_16U[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_THRESHOLD_RANGE_16U[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_THRESHOLD_RANGE_16U[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_THRESHOLD_RANGE_16U[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_THRESHOLD_RANGE_16U[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_THRESHOLD_RANGE_16U[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_THRESHOLD_RANGE_16U[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_THRESHOLD_RANGE_16U[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_THRESHOLD_RANGE_16U[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_THRESHOLD_RANGE_16U[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_THRESHOLD_RANGE_16U[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_THRESHOLD_RANGE_16U[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_THRESHOLD_RANGE_16U[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_THRESHOLD_RANGE_16U[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_THRESHOLD_RANGE_16U[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_THRESHOLD_RANGE_16U[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_THRESHOLD_RANGE_16U[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_THRESHOLD_RANGE_16U[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_THRESHOLD_RANGE_16U[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_THRESHOLD_RANGE_16U[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_THRESHOLD_RANGE_16U[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_THRESHOLD_RANGE_16U[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_THRESHOLD_RANGE_16U[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_THRESHOLD_RANGE_16U[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_THRESHOLD_RANGE_16U[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_THRESHOLD_RANGE_16U[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_THRESHOLD_RANGE_16U[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_THRESHOLD_RANGE_16U[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_THRESHOLD_RANGE_16U[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_THRESHOLD_RANGE_16U[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_THRESHOLD_RANGE_16U[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_THRESHOLD_RANGE_16U[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_THRESHOLD_RANGE_16U[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_THRESHOLD_RANGE_16U[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_THRESHOLD_RANGE_16U[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_THRESHOLD_RANGE_16U[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_THRESHOLD_RANGE_16U[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_THRESHOLD_RANGE_16U[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_THRESHOLD_RANGE_16U[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_THRESHOLD_RANGE_16U[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_THRESHOLD_RANGE_16U[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_THRESHOLD_RANGE_16U[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_THRESHOLD_RANGE_16U[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_THRESHOLD_RANGE_16U[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_THRESHOLD_RANGE_16U[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_THRESHOLD_RANGE_16U[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_THRESHOLD_RANGE_16U[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_THRESHOLD_RANGE_16U[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_THRESHOLD_RANGE_16U[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_THRESHOLD_RANGE_16U[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_THRESHOLD_RANGE_16U[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_THRESHOLD_RANGE_16U[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_THRESHOLD_RANGE_16U[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_THRESHOLD_RANGE_16U[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_THRESHOLD_RANGE_16U[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_THRESHOLD_RANGE_16U[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_THRESHOLD_RANGE_16U[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_THRESHOLD_RANGE_16U[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_THRESHOLD_RANGE_16U[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_THRESHOLD_RANGE_16U[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_THRESHOLD_RANGE_16U[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_THRESHOLD_RANGE_16U[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_THRESHOLD_RANGE_16U[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_THRESHOLD_RANGE_16U[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_THRESHOLD_RANGE_16U[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_THRESHOLD_RANGE_16U[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_THRESHOLD_RANGE_16U[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_THRESHOLD_RANGE_16U[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_THRESHOLD_RANGE_16U[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_THRESHOLD_RANGE_16U[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_THRESHOLD_RANGE_16U[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_THRESHOLD_RANGE_16U[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_THRESHOLD_RANGE_16U[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_THRESHOLD_RANGE_16U[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_THRESHOLD_RANGE_16U[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_THRESHOLD_RANGE_16U[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_THRESHOLD_RANGE_16U[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_THRESHOLD_RANGE_16U[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_THRESHOLD_RANGE_16U[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_THRESHOLD_RANGE_16U[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_THRESHOLD_RANGE_16U[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_THRESHOLD_RANGE_16U[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_THRESHOLD_RANGE_16U[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_THRESHOLD_RANGE_16U[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_THRESHOLD_RANGE_16U[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_THRESHOLD_RANGE_16U[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_THRESHOLD_RANGE_16U[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_THRESHOLD_RANGE_16U[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_THRESHOLD_RANGE_16U[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_THRESHOLD_RANGE_16U[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_THRESHOLD_RANGE_16U[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_THRESHOLD_RANGE_16U[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_THRESHOLD_RANGE_16U[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_THRESHOLD_RANGE_16U[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_THRESHOLD_RANGE_16U[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_THRESHOLD_RANGE_16U[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_THRESHOLD_RANGE_16U[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_THRESHOLD_RANGE_16U[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_THRESHOLD_RANGE_16U[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_THRESHOLD_RANGE_16U[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_THRESHOLD_RANGE_16U[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_THRESHOLD_RANGE_16U[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_THRESHOLD_RANGE_16U[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_THRESHOLD_RANGE_16U[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_THRESHOLD_RANGE_16U[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_THRESHOLD_RANGE_16U[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_THRESHOLD_RANGE_16U[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_THRESHOLD_RANGE_16U[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_THRESHOLD_RANGE_16U[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_THRESHOLD_RANGE_16U[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_THRESHOLD_RANGE_16U[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_THRESHOLD_RANGE_16U[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_THRESHOLD_RANGE_16U[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_THRESHOLD_RANGE_16U[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_THRESHOLD_RANGE_16U[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_THRESHOLD_RANGE_16U[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_THRESHOLD_RANGE_16U[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_THRESHOLD_RANGE_16U[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_THRESHOLD_RANGE_16U[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_THRESHOLD_RANGE_16U[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_THRESHOLD_RANGE_16U[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_THRESHOLD_RANGE_16U[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_THRESHOLD_RANGE_16U[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_THRESHOLD_RANGE_16U[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_THRESHOLD_RANGE_16U[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_THRESHOLD_RANGE_16U[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_THRESHOLD_RANGE_16U[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_THRESHOLD_RANGE_16U[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_THRESHOLD_RANGE_16U[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_THRESHOLD_RANGE_16U[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_THRESHOLD_RANGE_16U[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_THRESHOLD_RANGE_16U[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_THRESHOLD_RANGE_16U[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_THRESHOLD_RANGE_16U[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_THRESHOLD_RANGE_16U[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_THRESHOLD_RANGE_16U[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_THRESHOLD_RANGE_16U[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_THRESHOLD_RANGE_16U[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_THRESHOLD_RANGE_16U[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_THRESHOLD_RANGE_16U[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_THRESHOLD_RANGE_16U[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_THRESHOLD_RANGE_16U[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_THRESHOLD_RANGE_16U[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_THRESHOLD_RANGE_16U[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_THRESHOLD_RANGE_16U[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_THRESHOLD_RANGE_16U[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_THRESHOLD_RANGE_16U[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_THRESHOLD_RANGE_16U[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_THRESHOLD_RANGE_16U[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_THRESHOLD_RANGE_16U[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_THRESHOLD_RANGE_16U[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_THRESHOLD_RANGE_16U[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_THRESHOLD_RANGE_16U[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_THRESHOLD_RANGE_16U[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_THRESHOLD_RANGE_16U[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_THRESHOLD_RANGE_16U[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_THRESHOLD_RANGE_16U[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_THRESHOLD_RANGE_16U[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_THRESHOLD_RANGE_16U[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_THRESHOLD_RANGE_16U[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_THRESHOLD_RANGE_16U[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_THRESHOLD_RANGE_16U[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_THRESHOLD_RANGE_16U[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_THRESHOLD_RANGE_16U[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_THRESHOLD_RANGE_16U[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_THRESHOLD_RANGE_16U[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_THRESHOLD_RANGE_16U[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_THRESHOLD_RANGE_16U[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_THRESHOLD_RANGE_16U[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_THRESHOLD_RANGE_16U[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_THRESHOLD_RANGE_16U[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_THRESHOLD_RANGE_16U[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_THRESHOLD_RANGE_16U[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_THRESHOLD_RANGE_16U[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_THRESHOLD_RANGE_16U[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_THRESHOLD_RANGE_16U[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_THRESHOLD_RANGE_16U[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_THRESHOLD_RANGE_16U[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_THRESHOLD_RANGE_16U[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_THRESHOLD_RANGE_16U[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_THRESHOLD_RANGE_16U[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_THRESHOLD_RANGE_16U[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_THRESHOLD_RANGE_16U[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_THRESHOLD_RANGE_16U[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_THRESHOLD_RANGE_16U[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_THRESHOLD_RANGE_16U[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_THRESHOLD_RANGE_16U[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_THRESHOLD_RANGE_16U[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_THRESHOLD_RANGE_16U[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_THRESHOLD_RANGE_16U[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_THRESHOLD_RANGE_16U[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_THRESHOLD_RANGE_16U[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_THRESHOLD_RANGE_16U[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_THRESHOLD_RANGE_16U[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_THRESHOLD_RANGE_16U[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_THRESHOLD_RANGE_16U[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_THRESHOLD_RANGE_16U[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_THRESHOLD_RANGE_16U[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_THRESHOLD_RANGE_16U[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_THRESHOLD_RANGE_16U[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_THRESHOLD_RANGE_16U[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_THRESHOLD_RANGE_16U[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_THRESHOLD_RANGE_16U[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_THRESHOLD_RANGE_16U[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_THRESHOLD_RANGE_16U[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_THRESHOLD_RANGE_16U[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_THRESHOLD_RANGE_16U[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_THRESHOLD_RANGE_16U[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_THRESHOLD_RANGE_16U[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_THRESHOLD_RANGE_16U[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_THRESHOLD_RANGE_16U[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_THRESHOLD_RANGE_16U[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_THRESHOLD_RANGE_16U[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_THRESHOLD_RANGE_16U[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_THRESHOLD_RANGE_16U[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_THRESHOLD_RANGE_16U[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_THRESHOLD_RANGE_16U[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_THRESHOLD_RANGE_16U[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_THRESHOLD_RANGE_16U[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_THRESHOLD_RANGE_16U[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_THRESHOLD_RANGE_16U[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_THRESHOLD_RANGE_16U[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_THRESHOLD_RANGE_16U[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_THRESHOLD_RANGE_16U[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_THRESHOLD_RANGE_16U[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_THRESHOLD_RANGE_16U[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_THRESHOLD_RANGE_16U[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_THRESHOLD_RANGE_16U[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_THRESHOLD_RANGE_16U[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_THRESHOLD_RANGE_16U[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_THRESHOLD_RANGE_16U[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_THRESHOLD_RANGE_16U[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_THRESHOLD_RANGE_16U[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_THRESHOLD_RANGE_16U[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_THRESHOLD_RANGE_16U[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_THRESHOLD_RANGE_16U[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_THRESHOLD_RANGE_16U[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_THRESHOLD_RANGE_16U[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_THRESHOLD_RANGE_16U[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_THRESHOLD_RANGE_16U[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_THRESHOLD_RANGE_16U[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_THRESHOLD_RANGE_16U[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_THRESHOLD_RANGE_16U[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_THRESHOLD_RANGE_16U[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_THRESHOLD_RANGE_16U[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_THRESHOLD_RANGE_16U[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_THRESHOLD_RANGE_16U[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_THRESHOLD_RANGE_16U[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_THRESHOLD_RANGE_16U[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_THRESHOLD_RANGE_16U[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_THRESHOLD_RANGE_16U[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_THRESHOLD_RANGE_16U[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_THRESHOLD_RANGE_16U[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_THRESHOLD_RANGE_16U[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_THRESHOLD_RANGE_16U[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_THRESHOLD_RANGE_16U[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_THRESHOLD_RANGE_16U[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_THRESHOLD_RANGE_16U[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_THRESHOLD_RANGE_16U[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_THRESHOLD_RANGE_16U[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_THRESHOLD_RANGE_16U[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_THRESHOLD_RANGE_16U[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_THRESHOLD_RANGE_16U[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_THRESHOLD_RANGE_16U[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_THRESHOLD_RANGE_16U[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_THRESHOLD_RANGE_16U[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_THRESHOLD_RANGE_16U[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_THRESHOLD_RANGE_16U[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_THRESHOLD_RANGE_16U[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_THRESHOLD_RANGE_16U[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_THRESHOLD_RANGE_16U[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_THRESHOLD_RANGE_16U[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_THRESHOLD_RANGE_16U[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_THRESHOLD_RANGE_16U[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_THRESHOLD_RANGE_16U[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_THRESHOLD_RANGE_16U[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_THRESHOLD_RANGE_16U[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_THRESHOLD_RANGE_16U[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_THRESHOLD_RANGE_16U[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_THRESHOLD_RANGE_16U[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_THRESHOLD_RANGE_16U[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_THRESHOLD_RANGE_16U[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_THRESHOLD_RANGE_16U[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_THRESHOLD_RANGE_16U[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_THRESHOLD_RANGE_16U[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_THRESHOLD_RANGE_16U[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_THRESHOLD_RANGE_16U[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_THRESHOLD_RANGE_16U[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_THRESHOLD_RANGE_16U[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_THRESHOLD_RANGE_16U[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_THRESHOLD_RANGE_16U[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_THRESHOLD_RANGE_16U[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_THRESHOLD_RANGE_16U[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_THRESHOLD_RANGE_16U[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_THRESHOLD_RANGE_16U[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_THRESHOLD_RANGE_16U[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_THRESHOLD_RANGE_16U[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_THRESHOLD_RANGE_16U[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_THRESHOLD_RANGE_16U[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_THRESHOLD_RANGE_16U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_THRESHOLD_RANGE_16U[] = {{0, 0}};

static acf_scenario gScenarioArray_THRESHOLD_RANGE_16U[] = {
{2, 1, 12, 32, 0, gScenarioBufferData0_THRESHOLD_RANGE_16U, 64, gScenarioKernelData0_THRESHOLD_RANGE_16U, 4},
{4, 1, 24, 32, 0, gScenarioBufferData1_THRESHOLD_RANGE_16U, 64, gScenarioKernelData1_THRESHOLD_RANGE_16U, 4},
{6, 1, 36, 32, 0, gScenarioBufferData2_THRESHOLD_RANGE_16U, 64, gScenarioKernelData2_THRESHOLD_RANGE_16U, 4},
{8, 1, 48, 32, 0, gScenarioBufferData3_THRESHOLD_RANGE_16U, 64, gScenarioKernelData3_THRESHOLD_RANGE_16U, 4},
{10, 1, 60, 32, 0, gScenarioBufferData4_THRESHOLD_RANGE_16U, 64, gScenarioKernelData4_THRESHOLD_RANGE_16U, 4},
{12, 1, 72, 32, 0, gScenarioBufferData5_THRESHOLD_RANGE_16U, 64, gScenarioKernelData5_THRESHOLD_RANGE_16U, 4},
{14, 1, 84, 32, 0, gScenarioBufferData6_THRESHOLD_RANGE_16U, 64, gScenarioKernelData6_THRESHOLD_RANGE_16U, 4},
{16, 1, 96, 32, 0, gScenarioBufferData7_THRESHOLD_RANGE_16U, 64, gScenarioKernelData7_THRESHOLD_RANGE_16U, 4},
{18, 1, 108, 32, 0, gScenarioBufferData8_THRESHOLD_RANGE_16U, 64, gScenarioKernelData8_THRESHOLD_RANGE_16U, 4},
{20, 1, 120, 32, 0, gScenarioBufferData9_THRESHOLD_RANGE_16U, 64, gScenarioKernelData9_THRESHOLD_RANGE_16U, 4},
{22, 1, 132, 32, 0, gScenarioBufferData10_THRESHOLD_RANGE_16U, 64, gScenarioKernelData10_THRESHOLD_RANGE_16U, 4},
{24, 1, 144, 32, 0, gScenarioBufferData11_THRESHOLD_RANGE_16U, 64, gScenarioKernelData11_THRESHOLD_RANGE_16U, 4},
{26, 1, 156, 32, 0, gScenarioBufferData12_THRESHOLD_RANGE_16U, 64, gScenarioKernelData12_THRESHOLD_RANGE_16U, 4},
{28, 1, 168, 32, 0, gScenarioBufferData13_THRESHOLD_RANGE_16U, 64, gScenarioKernelData13_THRESHOLD_RANGE_16U, 4},
{30, 1, 180, 32, 0, gScenarioBufferData14_THRESHOLD_RANGE_16U, 64, gScenarioKernelData14_THRESHOLD_RANGE_16U, 4},
{32, 1, 192, 32, 0, gScenarioBufferData15_THRESHOLD_RANGE_16U, 64, gScenarioKernelData15_THRESHOLD_RANGE_16U, 4},
{36, 1, 216, 32, 0, gScenarioBufferData16_THRESHOLD_RANGE_16U, 64, gScenarioKernelData16_THRESHOLD_RANGE_16U, 4},
{40, 1, 240, 32, 0, gScenarioBufferData17_THRESHOLD_RANGE_16U, 64, gScenarioKernelData17_THRESHOLD_RANGE_16U, 4},
{44, 1, 264, 32, 0, gScenarioBufferData18_THRESHOLD_RANGE_16U, 64, gScenarioKernelData18_THRESHOLD_RANGE_16U, 4},
{48, 1, 288, 32, 0, gScenarioBufferData19_THRESHOLD_RANGE_16U, 64, gScenarioKernelData19_THRESHOLD_RANGE_16U, 4},
{52, 1, 312, 32, 0, gScenarioBufferData20_THRESHOLD_RANGE_16U, 64, gScenarioKernelData20_THRESHOLD_RANGE_16U, 4},
{56, 1, 336, 32, 0, gScenarioBufferData21_THRESHOLD_RANGE_16U, 64, gScenarioKernelData21_THRESHOLD_RANGE_16U, 4},
{60, 1, 360, 32, 0, gScenarioBufferData22_THRESHOLD_RANGE_16U, 64, gScenarioKernelData22_THRESHOLD_RANGE_16U, 4},
{64, 1, 384, 32, 0, gScenarioBufferData23_THRESHOLD_RANGE_16U, 64, gScenarioKernelData23_THRESHOLD_RANGE_16U, 4},
{2, 2, 24, 32, 0, gScenarioBufferData24_THRESHOLD_RANGE_16U, 64, gScenarioKernelData24_THRESHOLD_RANGE_16U, 4},
{4, 2, 48, 32, 0, gScenarioBufferData25_THRESHOLD_RANGE_16U, 64, gScenarioKernelData25_THRESHOLD_RANGE_16U, 4},
{6, 2, 72, 32, 0, gScenarioBufferData26_THRESHOLD_RANGE_16U, 64, gScenarioKernelData26_THRESHOLD_RANGE_16U, 4},
{8, 2, 96, 32, 0, gScenarioBufferData27_THRESHOLD_RANGE_16U, 64, gScenarioKernelData27_THRESHOLD_RANGE_16U, 4},
{10, 2, 120, 32, 0, gScenarioBufferData28_THRESHOLD_RANGE_16U, 64, gScenarioKernelData28_THRESHOLD_RANGE_16U, 4},
{12, 2, 144, 32, 0, gScenarioBufferData29_THRESHOLD_RANGE_16U, 64, gScenarioKernelData29_THRESHOLD_RANGE_16U, 4},
{14, 2, 168, 32, 0, gScenarioBufferData30_THRESHOLD_RANGE_16U, 64, gScenarioKernelData30_THRESHOLD_RANGE_16U, 4},
{16, 2, 192, 32, 0, gScenarioBufferData31_THRESHOLD_RANGE_16U, 64, gScenarioKernelData31_THRESHOLD_RANGE_16U, 4},
{18, 2, 216, 32, 0, gScenarioBufferData32_THRESHOLD_RANGE_16U, 64, gScenarioKernelData32_THRESHOLD_RANGE_16U, 4},
{20, 2, 240, 32, 0, gScenarioBufferData33_THRESHOLD_RANGE_16U, 64, gScenarioKernelData33_THRESHOLD_RANGE_16U, 4},
{22, 2, 264, 32, 0, gScenarioBufferData34_THRESHOLD_RANGE_16U, 64, gScenarioKernelData34_THRESHOLD_RANGE_16U, 4},
{24, 2, 288, 32, 0, gScenarioBufferData35_THRESHOLD_RANGE_16U, 64, gScenarioKernelData35_THRESHOLD_RANGE_16U, 4},
{26, 2, 312, 32, 0, gScenarioBufferData36_THRESHOLD_RANGE_16U, 64, gScenarioKernelData36_THRESHOLD_RANGE_16U, 4},
{28, 2, 336, 32, 0, gScenarioBufferData37_THRESHOLD_RANGE_16U, 64, gScenarioKernelData37_THRESHOLD_RANGE_16U, 4},
{30, 2, 360, 32, 0, gScenarioBufferData38_THRESHOLD_RANGE_16U, 64, gScenarioKernelData38_THRESHOLD_RANGE_16U, 4},
{32, 2, 384, 32, 0, gScenarioBufferData39_THRESHOLD_RANGE_16U, 64, gScenarioKernelData39_THRESHOLD_RANGE_16U, 4},
{36, 2, 432, 32, 0, gScenarioBufferData40_THRESHOLD_RANGE_16U, 64, gScenarioKernelData40_THRESHOLD_RANGE_16U, 4},
{40, 2, 480, 32, 0, gScenarioBufferData41_THRESHOLD_RANGE_16U, 64, gScenarioKernelData41_THRESHOLD_RANGE_16U, 4},
{44, 2, 528, 32, 0, gScenarioBufferData42_THRESHOLD_RANGE_16U, 64, gScenarioKernelData42_THRESHOLD_RANGE_16U, 4},
{48, 2, 576, 32, 0, gScenarioBufferData43_THRESHOLD_RANGE_16U, 64, gScenarioKernelData43_THRESHOLD_RANGE_16U, 4},
{52, 2, 624, 32, 0, gScenarioBufferData44_THRESHOLD_RANGE_16U, 64, gScenarioKernelData44_THRESHOLD_RANGE_16U, 4},
{56, 2, 672, 32, 0, gScenarioBufferData45_THRESHOLD_RANGE_16U, 64, gScenarioKernelData45_THRESHOLD_RANGE_16U, 4},
{60, 2, 720, 32, 0, gScenarioBufferData46_THRESHOLD_RANGE_16U, 64, gScenarioKernelData46_THRESHOLD_RANGE_16U, 4},
{64, 2, 768, 32, 0, gScenarioBufferData47_THRESHOLD_RANGE_16U, 64, gScenarioKernelData47_THRESHOLD_RANGE_16U, 4},
{2, 3, 36, 32, 0, gScenarioBufferData48_THRESHOLD_RANGE_16U, 64, gScenarioKernelData48_THRESHOLD_RANGE_16U, 4},
{4, 3, 72, 32, 0, gScenarioBufferData49_THRESHOLD_RANGE_16U, 64, gScenarioKernelData49_THRESHOLD_RANGE_16U, 4},
{6, 3, 108, 32, 0, gScenarioBufferData50_THRESHOLD_RANGE_16U, 64, gScenarioKernelData50_THRESHOLD_RANGE_16U, 4},
{8, 3, 144, 32, 0, gScenarioBufferData51_THRESHOLD_RANGE_16U, 64, gScenarioKernelData51_THRESHOLD_RANGE_16U, 4},
{10, 3, 180, 32, 0, gScenarioBufferData52_THRESHOLD_RANGE_16U, 64, gScenarioKernelData52_THRESHOLD_RANGE_16U, 4},
{12, 3, 216, 32, 0, gScenarioBufferData53_THRESHOLD_RANGE_16U, 64, gScenarioKernelData53_THRESHOLD_RANGE_16U, 4},
{14, 3, 252, 32, 0, gScenarioBufferData54_THRESHOLD_RANGE_16U, 64, gScenarioKernelData54_THRESHOLD_RANGE_16U, 4},
{16, 3, 288, 32, 0, gScenarioBufferData55_THRESHOLD_RANGE_16U, 64, gScenarioKernelData55_THRESHOLD_RANGE_16U, 4},
{18, 3, 324, 32, 0, gScenarioBufferData56_THRESHOLD_RANGE_16U, 64, gScenarioKernelData56_THRESHOLD_RANGE_16U, 4},
{20, 3, 360, 32, 0, gScenarioBufferData57_THRESHOLD_RANGE_16U, 64, gScenarioKernelData57_THRESHOLD_RANGE_16U, 4},
{22, 3, 396, 32, 0, gScenarioBufferData58_THRESHOLD_RANGE_16U, 64, gScenarioKernelData58_THRESHOLD_RANGE_16U, 4},
{24, 3, 432, 32, 0, gScenarioBufferData59_THRESHOLD_RANGE_16U, 64, gScenarioKernelData59_THRESHOLD_RANGE_16U, 4},
{26, 3, 468, 32, 0, gScenarioBufferData60_THRESHOLD_RANGE_16U, 64, gScenarioKernelData60_THRESHOLD_RANGE_16U, 4},
{28, 3, 504, 32, 0, gScenarioBufferData61_THRESHOLD_RANGE_16U, 64, gScenarioKernelData61_THRESHOLD_RANGE_16U, 4},
{30, 3, 540, 32, 0, gScenarioBufferData62_THRESHOLD_RANGE_16U, 64, gScenarioKernelData62_THRESHOLD_RANGE_16U, 4},
{32, 3, 576, 32, 0, gScenarioBufferData63_THRESHOLD_RANGE_16U, 64, gScenarioKernelData63_THRESHOLD_RANGE_16U, 4},
{36, 3, 648, 32, 0, gScenarioBufferData64_THRESHOLD_RANGE_16U, 64, gScenarioKernelData64_THRESHOLD_RANGE_16U, 4},
{40, 3, 720, 32, 0, gScenarioBufferData65_THRESHOLD_RANGE_16U, 64, gScenarioKernelData65_THRESHOLD_RANGE_16U, 4},
{44, 3, 792, 32, 0, gScenarioBufferData66_THRESHOLD_RANGE_16U, 64, gScenarioKernelData66_THRESHOLD_RANGE_16U, 4},
{48, 3, 864, 32, 0, gScenarioBufferData67_THRESHOLD_RANGE_16U, 64, gScenarioKernelData67_THRESHOLD_RANGE_16U, 4},
{52, 3, 936, 32, 0, gScenarioBufferData68_THRESHOLD_RANGE_16U, 64, gScenarioKernelData68_THRESHOLD_RANGE_16U, 4},
{56, 3, 1008, 32, 0, gScenarioBufferData69_THRESHOLD_RANGE_16U, 64, gScenarioKernelData69_THRESHOLD_RANGE_16U, 4},
{60, 3, 1080, 32, 0, gScenarioBufferData70_THRESHOLD_RANGE_16U, 64, gScenarioKernelData70_THRESHOLD_RANGE_16U, 4},
{64, 3, 1152, 32, 0, gScenarioBufferData71_THRESHOLD_RANGE_16U, 64, gScenarioKernelData71_THRESHOLD_RANGE_16U, 4},
{2, 4, 48, 32, 0, gScenarioBufferData72_THRESHOLD_RANGE_16U, 64, gScenarioKernelData72_THRESHOLD_RANGE_16U, 4},
{4, 4, 96, 32, 0, gScenarioBufferData73_THRESHOLD_RANGE_16U, 64, gScenarioKernelData73_THRESHOLD_RANGE_16U, 4},
{6, 4, 144, 32, 0, gScenarioBufferData74_THRESHOLD_RANGE_16U, 64, gScenarioKernelData74_THRESHOLD_RANGE_16U, 4},
{8, 4, 192, 32, 0, gScenarioBufferData75_THRESHOLD_RANGE_16U, 64, gScenarioKernelData75_THRESHOLD_RANGE_16U, 4},
{10, 4, 240, 32, 0, gScenarioBufferData76_THRESHOLD_RANGE_16U, 64, gScenarioKernelData76_THRESHOLD_RANGE_16U, 4},
{12, 4, 288, 32, 0, gScenarioBufferData77_THRESHOLD_RANGE_16U, 64, gScenarioKernelData77_THRESHOLD_RANGE_16U, 4},
{14, 4, 336, 32, 0, gScenarioBufferData78_THRESHOLD_RANGE_16U, 64, gScenarioKernelData78_THRESHOLD_RANGE_16U, 4},
{16, 4, 384, 32, 0, gScenarioBufferData79_THRESHOLD_RANGE_16U, 64, gScenarioKernelData79_THRESHOLD_RANGE_16U, 4},
{18, 4, 432, 32, 0, gScenarioBufferData80_THRESHOLD_RANGE_16U, 64, gScenarioKernelData80_THRESHOLD_RANGE_16U, 4},
{20, 4, 480, 32, 0, gScenarioBufferData81_THRESHOLD_RANGE_16U, 64, gScenarioKernelData81_THRESHOLD_RANGE_16U, 4},
{22, 4, 528, 32, 0, gScenarioBufferData82_THRESHOLD_RANGE_16U, 64, gScenarioKernelData82_THRESHOLD_RANGE_16U, 4},
{24, 4, 576, 32, 0, gScenarioBufferData83_THRESHOLD_RANGE_16U, 64, gScenarioKernelData83_THRESHOLD_RANGE_16U, 4},
{26, 4, 624, 32, 0, gScenarioBufferData84_THRESHOLD_RANGE_16U, 64, gScenarioKernelData84_THRESHOLD_RANGE_16U, 4},
{28, 4, 672, 32, 0, gScenarioBufferData85_THRESHOLD_RANGE_16U, 64, gScenarioKernelData85_THRESHOLD_RANGE_16U, 4},
{30, 4, 720, 32, 0, gScenarioBufferData86_THRESHOLD_RANGE_16U, 64, gScenarioKernelData86_THRESHOLD_RANGE_16U, 4},
{32, 4, 768, 32, 0, gScenarioBufferData87_THRESHOLD_RANGE_16U, 64, gScenarioKernelData87_THRESHOLD_RANGE_16U, 4},
{36, 4, 864, 32, 0, gScenarioBufferData88_THRESHOLD_RANGE_16U, 64, gScenarioKernelData88_THRESHOLD_RANGE_16U, 4},
{40, 4, 960, 32, 0, gScenarioBufferData89_THRESHOLD_RANGE_16U, 64, gScenarioKernelData89_THRESHOLD_RANGE_16U, 4},
{44, 4, 1056, 32, 0, gScenarioBufferData90_THRESHOLD_RANGE_16U, 64, gScenarioKernelData90_THRESHOLD_RANGE_16U, 4},
{48, 4, 1152, 32, 0, gScenarioBufferData91_THRESHOLD_RANGE_16U, 64, gScenarioKernelData91_THRESHOLD_RANGE_16U, 4},
{52, 4, 1248, 32, 0, gScenarioBufferData92_THRESHOLD_RANGE_16U, 64, gScenarioKernelData92_THRESHOLD_RANGE_16U, 4},
{56, 4, 1344, 32, 0, gScenarioBufferData93_THRESHOLD_RANGE_16U, 64, gScenarioKernelData93_THRESHOLD_RANGE_16U, 4},
{60, 4, 1440, 32, 0, gScenarioBufferData94_THRESHOLD_RANGE_16U, 64, gScenarioKernelData94_THRESHOLD_RANGE_16U, 4},
{64, 4, 1536, 32, 0, gScenarioBufferData95_THRESHOLD_RANGE_16U, 64, gScenarioKernelData95_THRESHOLD_RANGE_16U, 4},
{2, 5, 60, 32, 0, gScenarioBufferData96_THRESHOLD_RANGE_16U, 64, gScenarioKernelData96_THRESHOLD_RANGE_16U, 4},
{4, 5, 120, 32, 0, gScenarioBufferData97_THRESHOLD_RANGE_16U, 64, gScenarioKernelData97_THRESHOLD_RANGE_16U, 4},
{6, 5, 180, 32, 0, gScenarioBufferData98_THRESHOLD_RANGE_16U, 64, gScenarioKernelData98_THRESHOLD_RANGE_16U, 4},
{8, 5, 240, 32, 0, gScenarioBufferData99_THRESHOLD_RANGE_16U, 64, gScenarioKernelData99_THRESHOLD_RANGE_16U, 4},
{10, 5, 300, 32, 0, gScenarioBufferData100_THRESHOLD_RANGE_16U, 64, gScenarioKernelData100_THRESHOLD_RANGE_16U, 4},
{12, 5, 360, 32, 0, gScenarioBufferData101_THRESHOLD_RANGE_16U, 64, gScenarioKernelData101_THRESHOLD_RANGE_16U, 4},
{14, 5, 420, 32, 0, gScenarioBufferData102_THRESHOLD_RANGE_16U, 64, gScenarioKernelData102_THRESHOLD_RANGE_16U, 4},
{16, 5, 480, 32, 0, gScenarioBufferData103_THRESHOLD_RANGE_16U, 64, gScenarioKernelData103_THRESHOLD_RANGE_16U, 4},
{18, 5, 540, 32, 0, gScenarioBufferData104_THRESHOLD_RANGE_16U, 64, gScenarioKernelData104_THRESHOLD_RANGE_16U, 4},
{20, 5, 600, 32, 0, gScenarioBufferData105_THRESHOLD_RANGE_16U, 64, gScenarioKernelData105_THRESHOLD_RANGE_16U, 4},
{22, 5, 660, 32, 0, gScenarioBufferData106_THRESHOLD_RANGE_16U, 64, gScenarioKernelData106_THRESHOLD_RANGE_16U, 4},
{24, 5, 720, 32, 0, gScenarioBufferData107_THRESHOLD_RANGE_16U, 64, gScenarioKernelData107_THRESHOLD_RANGE_16U, 4},
{26, 5, 780, 32, 0, gScenarioBufferData108_THRESHOLD_RANGE_16U, 64, gScenarioKernelData108_THRESHOLD_RANGE_16U, 4},
{28, 5, 840, 32, 0, gScenarioBufferData109_THRESHOLD_RANGE_16U, 64, gScenarioKernelData109_THRESHOLD_RANGE_16U, 4},
{30, 5, 900, 32, 0, gScenarioBufferData110_THRESHOLD_RANGE_16U, 64, gScenarioKernelData110_THRESHOLD_RANGE_16U, 4},
{32, 5, 960, 32, 0, gScenarioBufferData111_THRESHOLD_RANGE_16U, 64, gScenarioKernelData111_THRESHOLD_RANGE_16U, 4},
{36, 5, 1080, 32, 0, gScenarioBufferData112_THRESHOLD_RANGE_16U, 64, gScenarioKernelData112_THRESHOLD_RANGE_16U, 4},
{40, 5, 1200, 32, 0, gScenarioBufferData113_THRESHOLD_RANGE_16U, 64, gScenarioKernelData113_THRESHOLD_RANGE_16U, 4},
{44, 5, 1320, 32, 0, gScenarioBufferData114_THRESHOLD_RANGE_16U, 64, gScenarioKernelData114_THRESHOLD_RANGE_16U, 4},
{48, 5, 1440, 32, 0, gScenarioBufferData115_THRESHOLD_RANGE_16U, 64, gScenarioKernelData115_THRESHOLD_RANGE_16U, 4},
{52, 5, 1560, 32, 0, gScenarioBufferData116_THRESHOLD_RANGE_16U, 64, gScenarioKernelData116_THRESHOLD_RANGE_16U, 4},
{56, 5, 1680, 32, 0, gScenarioBufferData117_THRESHOLD_RANGE_16U, 64, gScenarioKernelData117_THRESHOLD_RANGE_16U, 4},
{60, 5, 1800, 32, 0, gScenarioBufferData118_THRESHOLD_RANGE_16U, 64, gScenarioKernelData118_THRESHOLD_RANGE_16U, 4},
{64, 5, 1920, 32, 0, gScenarioBufferData119_THRESHOLD_RANGE_16U, 64, gScenarioKernelData119_THRESHOLD_RANGE_16U, 4},
{2, 6, 72, 32, 0, gScenarioBufferData120_THRESHOLD_RANGE_16U, 64, gScenarioKernelData120_THRESHOLD_RANGE_16U, 4},
{4, 6, 144, 32, 0, gScenarioBufferData121_THRESHOLD_RANGE_16U, 64, gScenarioKernelData121_THRESHOLD_RANGE_16U, 4},
{6, 6, 216, 32, 0, gScenarioBufferData122_THRESHOLD_RANGE_16U, 64, gScenarioKernelData122_THRESHOLD_RANGE_16U, 4},
{8, 6, 288, 32, 0, gScenarioBufferData123_THRESHOLD_RANGE_16U, 64, gScenarioKernelData123_THRESHOLD_RANGE_16U, 4},
{10, 6, 360, 32, 0, gScenarioBufferData124_THRESHOLD_RANGE_16U, 64, gScenarioKernelData124_THRESHOLD_RANGE_16U, 4},
{12, 6, 432, 32, 0, gScenarioBufferData125_THRESHOLD_RANGE_16U, 64, gScenarioKernelData125_THRESHOLD_RANGE_16U, 4},
{14, 6, 504, 32, 0, gScenarioBufferData126_THRESHOLD_RANGE_16U, 64, gScenarioKernelData126_THRESHOLD_RANGE_16U, 4},
{16, 6, 576, 32, 0, gScenarioBufferData127_THRESHOLD_RANGE_16U, 64, gScenarioKernelData127_THRESHOLD_RANGE_16U, 4},
{18, 6, 648, 32, 0, gScenarioBufferData128_THRESHOLD_RANGE_16U, 64, gScenarioKernelData128_THRESHOLD_RANGE_16U, 4},
{20, 6, 720, 32, 0, gScenarioBufferData129_THRESHOLD_RANGE_16U, 64, gScenarioKernelData129_THRESHOLD_RANGE_16U, 4},
{22, 6, 792, 32, 0, gScenarioBufferData130_THRESHOLD_RANGE_16U, 64, gScenarioKernelData130_THRESHOLD_RANGE_16U, 4},
{24, 6, 864, 32, 0, gScenarioBufferData131_THRESHOLD_RANGE_16U, 64, gScenarioKernelData131_THRESHOLD_RANGE_16U, 4},
{26, 6, 936, 32, 0, gScenarioBufferData132_THRESHOLD_RANGE_16U, 64, gScenarioKernelData132_THRESHOLD_RANGE_16U, 4},
{28, 6, 1008, 32, 0, gScenarioBufferData133_THRESHOLD_RANGE_16U, 64, gScenarioKernelData133_THRESHOLD_RANGE_16U, 4},
{30, 6, 1080, 32, 0, gScenarioBufferData134_THRESHOLD_RANGE_16U, 64, gScenarioKernelData134_THRESHOLD_RANGE_16U, 4},
{32, 6, 1152, 32, 0, gScenarioBufferData135_THRESHOLD_RANGE_16U, 64, gScenarioKernelData135_THRESHOLD_RANGE_16U, 4},
{36, 6, 1296, 32, 0, gScenarioBufferData136_THRESHOLD_RANGE_16U, 64, gScenarioKernelData136_THRESHOLD_RANGE_16U, 4},
{40, 6, 1440, 32, 0, gScenarioBufferData137_THRESHOLD_RANGE_16U, 64, gScenarioKernelData137_THRESHOLD_RANGE_16U, 4},
{44, 6, 1584, 32, 0, gScenarioBufferData138_THRESHOLD_RANGE_16U, 64, gScenarioKernelData138_THRESHOLD_RANGE_16U, 4},
{48, 6, 1728, 32, 0, gScenarioBufferData139_THRESHOLD_RANGE_16U, 64, gScenarioKernelData139_THRESHOLD_RANGE_16U, 4},
{52, 6, 1872, 32, 0, gScenarioBufferData140_THRESHOLD_RANGE_16U, 64, gScenarioKernelData140_THRESHOLD_RANGE_16U, 4},
{56, 6, 2016, 32, 0, gScenarioBufferData141_THRESHOLD_RANGE_16U, 64, gScenarioKernelData141_THRESHOLD_RANGE_16U, 4},
{60, 6, 2160, 32, 0, gScenarioBufferData142_THRESHOLD_RANGE_16U, 64, gScenarioKernelData142_THRESHOLD_RANGE_16U, 4},
{64, 6, 2304, 32, 0, gScenarioBufferData143_THRESHOLD_RANGE_16U, 64, gScenarioKernelData143_THRESHOLD_RANGE_16U, 4},
{2, 8, 96, 32, 0, gScenarioBufferData144_THRESHOLD_RANGE_16U, 64, gScenarioKernelData144_THRESHOLD_RANGE_16U, 4},
{4, 8, 192, 32, 0, gScenarioBufferData145_THRESHOLD_RANGE_16U, 64, gScenarioKernelData145_THRESHOLD_RANGE_16U, 4},
{6, 8, 288, 32, 0, gScenarioBufferData146_THRESHOLD_RANGE_16U, 64, gScenarioKernelData146_THRESHOLD_RANGE_16U, 4},
{8, 8, 384, 32, 0, gScenarioBufferData147_THRESHOLD_RANGE_16U, 64, gScenarioKernelData147_THRESHOLD_RANGE_16U, 4},
{10, 8, 480, 32, 0, gScenarioBufferData148_THRESHOLD_RANGE_16U, 64, gScenarioKernelData148_THRESHOLD_RANGE_16U, 4},
{12, 8, 576, 32, 0, gScenarioBufferData149_THRESHOLD_RANGE_16U, 64, gScenarioKernelData149_THRESHOLD_RANGE_16U, 4},
{14, 8, 672, 32, 0, gScenarioBufferData150_THRESHOLD_RANGE_16U, 64, gScenarioKernelData150_THRESHOLD_RANGE_16U, 4},
{16, 8, 768, 32, 0, gScenarioBufferData151_THRESHOLD_RANGE_16U, 64, gScenarioKernelData151_THRESHOLD_RANGE_16U, 4},
{18, 8, 864, 32, 0, gScenarioBufferData152_THRESHOLD_RANGE_16U, 64, gScenarioKernelData152_THRESHOLD_RANGE_16U, 4},
{20, 8, 960, 32, 0, gScenarioBufferData153_THRESHOLD_RANGE_16U, 64, gScenarioKernelData153_THRESHOLD_RANGE_16U, 4},
{22, 8, 1056, 32, 0, gScenarioBufferData154_THRESHOLD_RANGE_16U, 64, gScenarioKernelData154_THRESHOLD_RANGE_16U, 4},
{24, 8, 1152, 32, 0, gScenarioBufferData155_THRESHOLD_RANGE_16U, 64, gScenarioKernelData155_THRESHOLD_RANGE_16U, 4},
{26, 8, 1248, 32, 0, gScenarioBufferData156_THRESHOLD_RANGE_16U, 64, gScenarioKernelData156_THRESHOLD_RANGE_16U, 4},
{28, 8, 1344, 32, 0, gScenarioBufferData157_THRESHOLD_RANGE_16U, 64, gScenarioKernelData157_THRESHOLD_RANGE_16U, 4},
{30, 8, 1440, 32, 0, gScenarioBufferData158_THRESHOLD_RANGE_16U, 64, gScenarioKernelData158_THRESHOLD_RANGE_16U, 4},
{32, 8, 1536, 32, 0, gScenarioBufferData159_THRESHOLD_RANGE_16U, 64, gScenarioKernelData159_THRESHOLD_RANGE_16U, 4},
{36, 8, 1728, 32, 0, gScenarioBufferData160_THRESHOLD_RANGE_16U, 64, gScenarioKernelData160_THRESHOLD_RANGE_16U, 4},
{40, 8, 1920, 32, 0, gScenarioBufferData161_THRESHOLD_RANGE_16U, 64, gScenarioKernelData161_THRESHOLD_RANGE_16U, 4},
{44, 8, 2112, 32, 0, gScenarioBufferData162_THRESHOLD_RANGE_16U, 64, gScenarioKernelData162_THRESHOLD_RANGE_16U, 4},
{48, 8, 2304, 32, 0, gScenarioBufferData163_THRESHOLD_RANGE_16U, 64, gScenarioKernelData163_THRESHOLD_RANGE_16U, 4},
{52, 8, 2496, 32, 0, gScenarioBufferData164_THRESHOLD_RANGE_16U, 64, gScenarioKernelData164_THRESHOLD_RANGE_16U, 4},
{56, 8, 2688, 32, 0, gScenarioBufferData165_THRESHOLD_RANGE_16U, 64, gScenarioKernelData165_THRESHOLD_RANGE_16U, 4},
{60, 8, 2880, 32, 0, gScenarioBufferData166_THRESHOLD_RANGE_16U, 64, gScenarioKernelData166_THRESHOLD_RANGE_16U, 4},
{64, 8, 3072, 32, 0, gScenarioBufferData167_THRESHOLD_RANGE_16U, 64, gScenarioKernelData167_THRESHOLD_RANGE_16U, 4},
{2, 10, 120, 32, 0, gScenarioBufferData168_THRESHOLD_RANGE_16U, 64, gScenarioKernelData168_THRESHOLD_RANGE_16U, 4},
{4, 10, 240, 32, 0, gScenarioBufferData169_THRESHOLD_RANGE_16U, 64, gScenarioKernelData169_THRESHOLD_RANGE_16U, 4},
{6, 10, 360, 32, 0, gScenarioBufferData170_THRESHOLD_RANGE_16U, 64, gScenarioKernelData170_THRESHOLD_RANGE_16U, 4},
{8, 10, 480, 32, 0, gScenarioBufferData171_THRESHOLD_RANGE_16U, 64, gScenarioKernelData171_THRESHOLD_RANGE_16U, 4},
{10, 10, 600, 32, 0, gScenarioBufferData172_THRESHOLD_RANGE_16U, 64, gScenarioKernelData172_THRESHOLD_RANGE_16U, 4},
{12, 10, 720, 32, 0, gScenarioBufferData173_THRESHOLD_RANGE_16U, 64, gScenarioKernelData173_THRESHOLD_RANGE_16U, 4},
{14, 10, 840, 32, 0, gScenarioBufferData174_THRESHOLD_RANGE_16U, 64, gScenarioKernelData174_THRESHOLD_RANGE_16U, 4},
{16, 10, 960, 32, 0, gScenarioBufferData175_THRESHOLD_RANGE_16U, 64, gScenarioKernelData175_THRESHOLD_RANGE_16U, 4},
{18, 10, 1080, 32, 0, gScenarioBufferData176_THRESHOLD_RANGE_16U, 64, gScenarioKernelData176_THRESHOLD_RANGE_16U, 4},
{20, 10, 1200, 32, 0, gScenarioBufferData177_THRESHOLD_RANGE_16U, 64, gScenarioKernelData177_THRESHOLD_RANGE_16U, 4},
{22, 10, 1320, 32, 0, gScenarioBufferData178_THRESHOLD_RANGE_16U, 64, gScenarioKernelData178_THRESHOLD_RANGE_16U, 4},
{24, 10, 1440, 32, 0, gScenarioBufferData179_THRESHOLD_RANGE_16U, 64, gScenarioKernelData179_THRESHOLD_RANGE_16U, 4},
{26, 10, 1560, 32, 0, gScenarioBufferData180_THRESHOLD_RANGE_16U, 64, gScenarioKernelData180_THRESHOLD_RANGE_16U, 4},
{28, 10, 1680, 32, 0, gScenarioBufferData181_THRESHOLD_RANGE_16U, 64, gScenarioKernelData181_THRESHOLD_RANGE_16U, 4},
{30, 10, 1800, 32, 0, gScenarioBufferData182_THRESHOLD_RANGE_16U, 64, gScenarioKernelData182_THRESHOLD_RANGE_16U, 4},
{32, 10, 1920, 32, 0, gScenarioBufferData183_THRESHOLD_RANGE_16U, 64, gScenarioKernelData183_THRESHOLD_RANGE_16U, 4},
{36, 10, 2160, 32, 0, gScenarioBufferData184_THRESHOLD_RANGE_16U, 64, gScenarioKernelData184_THRESHOLD_RANGE_16U, 4},
{40, 10, 2400, 32, 0, gScenarioBufferData185_THRESHOLD_RANGE_16U, 64, gScenarioKernelData185_THRESHOLD_RANGE_16U, 4},
{44, 10, 2640, 32, 0, gScenarioBufferData186_THRESHOLD_RANGE_16U, 64, gScenarioKernelData186_THRESHOLD_RANGE_16U, 4},
{48, 10, 2880, 32, 0, gScenarioBufferData187_THRESHOLD_RANGE_16U, 64, gScenarioKernelData187_THRESHOLD_RANGE_16U, 4},
{52, 10, 3120, 32, 0, gScenarioBufferData188_THRESHOLD_RANGE_16U, 64, gScenarioKernelData188_THRESHOLD_RANGE_16U, 4},
{56, 10, 3360, 32, 0, gScenarioBufferData189_THRESHOLD_RANGE_16U, 64, gScenarioKernelData189_THRESHOLD_RANGE_16U, 4},
{60, 10, 3600, 32, 0, gScenarioBufferData190_THRESHOLD_RANGE_16U, 64, gScenarioKernelData190_THRESHOLD_RANGE_16U, 4},
{64, 10, 3840, 32, 0, gScenarioBufferData191_THRESHOLD_RANGE_16U, 64, gScenarioKernelData191_THRESHOLD_RANGE_16U, 4},
{2, 12, 144, 32, 0, gScenarioBufferData192_THRESHOLD_RANGE_16U, 64, gScenarioKernelData192_THRESHOLD_RANGE_16U, 4},
{4, 12, 288, 32, 0, gScenarioBufferData193_THRESHOLD_RANGE_16U, 64, gScenarioKernelData193_THRESHOLD_RANGE_16U, 4},
{6, 12, 432, 32, 0, gScenarioBufferData194_THRESHOLD_RANGE_16U, 64, gScenarioKernelData194_THRESHOLD_RANGE_16U, 4},
{8, 12, 576, 32, 0, gScenarioBufferData195_THRESHOLD_RANGE_16U, 64, gScenarioKernelData195_THRESHOLD_RANGE_16U, 4},
{10, 12, 720, 32, 0, gScenarioBufferData196_THRESHOLD_RANGE_16U, 64, gScenarioKernelData196_THRESHOLD_RANGE_16U, 4},
{12, 12, 864, 32, 0, gScenarioBufferData197_THRESHOLD_RANGE_16U, 64, gScenarioKernelData197_THRESHOLD_RANGE_16U, 4},
{14, 12, 1008, 32, 0, gScenarioBufferData198_THRESHOLD_RANGE_16U, 64, gScenarioKernelData198_THRESHOLD_RANGE_16U, 4},
{16, 12, 1152, 32, 0, gScenarioBufferData199_THRESHOLD_RANGE_16U, 64, gScenarioKernelData199_THRESHOLD_RANGE_16U, 4},
{18, 12, 1296, 32, 0, gScenarioBufferData200_THRESHOLD_RANGE_16U, 64, gScenarioKernelData200_THRESHOLD_RANGE_16U, 4},
{20, 12, 1440, 32, 0, gScenarioBufferData201_THRESHOLD_RANGE_16U, 64, gScenarioKernelData201_THRESHOLD_RANGE_16U, 4},
{22, 12, 1584, 32, 0, gScenarioBufferData202_THRESHOLD_RANGE_16U, 64, gScenarioKernelData202_THRESHOLD_RANGE_16U, 4},
{24, 12, 1728, 32, 0, gScenarioBufferData203_THRESHOLD_RANGE_16U, 64, gScenarioKernelData203_THRESHOLD_RANGE_16U, 4},
{26, 12, 1872, 32, 0, gScenarioBufferData204_THRESHOLD_RANGE_16U, 64, gScenarioKernelData204_THRESHOLD_RANGE_16U, 4},
{28, 12, 2016, 32, 0, gScenarioBufferData205_THRESHOLD_RANGE_16U, 64, gScenarioKernelData205_THRESHOLD_RANGE_16U, 4},
{30, 12, 2160, 32, 0, gScenarioBufferData206_THRESHOLD_RANGE_16U, 64, gScenarioKernelData206_THRESHOLD_RANGE_16U, 4},
{32, 12, 2304, 32, 0, gScenarioBufferData207_THRESHOLD_RANGE_16U, 64, gScenarioKernelData207_THRESHOLD_RANGE_16U, 4},
{36, 12, 2592, 32, 0, gScenarioBufferData208_THRESHOLD_RANGE_16U, 64, gScenarioKernelData208_THRESHOLD_RANGE_16U, 4},
{40, 12, 2880, 32, 0, gScenarioBufferData209_THRESHOLD_RANGE_16U, 64, gScenarioKernelData209_THRESHOLD_RANGE_16U, 4},
{44, 12, 3168, 32, 0, gScenarioBufferData210_THRESHOLD_RANGE_16U, 64, gScenarioKernelData210_THRESHOLD_RANGE_16U, 4},
{48, 12, 3456, 32, 0, gScenarioBufferData211_THRESHOLD_RANGE_16U, 64, gScenarioKernelData211_THRESHOLD_RANGE_16U, 4},
{52, 12, 3744, 32, 0, gScenarioBufferData212_THRESHOLD_RANGE_16U, 64, gScenarioKernelData212_THRESHOLD_RANGE_16U, 4},
{2, 14, 168, 32, 0, gScenarioBufferData213_THRESHOLD_RANGE_16U, 64, gScenarioKernelData213_THRESHOLD_RANGE_16U, 4},
{4, 14, 336, 32, 0, gScenarioBufferData214_THRESHOLD_RANGE_16U, 64, gScenarioKernelData214_THRESHOLD_RANGE_16U, 4},
{6, 14, 504, 32, 0, gScenarioBufferData215_THRESHOLD_RANGE_16U, 64, gScenarioKernelData215_THRESHOLD_RANGE_16U, 4},
{8, 14, 672, 32, 0, gScenarioBufferData216_THRESHOLD_RANGE_16U, 64, gScenarioKernelData216_THRESHOLD_RANGE_16U, 4},
{10, 14, 840, 32, 0, gScenarioBufferData217_THRESHOLD_RANGE_16U, 64, gScenarioKernelData217_THRESHOLD_RANGE_16U, 4},
{12, 14, 1008, 32, 0, gScenarioBufferData218_THRESHOLD_RANGE_16U, 64, gScenarioKernelData218_THRESHOLD_RANGE_16U, 4},
{14, 14, 1176, 32, 0, gScenarioBufferData219_THRESHOLD_RANGE_16U, 64, gScenarioKernelData219_THRESHOLD_RANGE_16U, 4},
{16, 14, 1344, 32, 0, gScenarioBufferData220_THRESHOLD_RANGE_16U, 64, gScenarioKernelData220_THRESHOLD_RANGE_16U, 4},
{18, 14, 1512, 32, 0, gScenarioBufferData221_THRESHOLD_RANGE_16U, 64, gScenarioKernelData221_THRESHOLD_RANGE_16U, 4},
{20, 14, 1680, 32, 0, gScenarioBufferData222_THRESHOLD_RANGE_16U, 64, gScenarioKernelData222_THRESHOLD_RANGE_16U, 4},
{22, 14, 1848, 32, 0, gScenarioBufferData223_THRESHOLD_RANGE_16U, 64, gScenarioKernelData223_THRESHOLD_RANGE_16U, 4},
{24, 14, 2016, 32, 0, gScenarioBufferData224_THRESHOLD_RANGE_16U, 64, gScenarioKernelData224_THRESHOLD_RANGE_16U, 4},
{26, 14, 2184, 32, 0, gScenarioBufferData225_THRESHOLD_RANGE_16U, 64, gScenarioKernelData225_THRESHOLD_RANGE_16U, 4},
{28, 14, 2352, 32, 0, gScenarioBufferData226_THRESHOLD_RANGE_16U, 64, gScenarioKernelData226_THRESHOLD_RANGE_16U, 4},
{30, 14, 2520, 32, 0, gScenarioBufferData227_THRESHOLD_RANGE_16U, 64, gScenarioKernelData227_THRESHOLD_RANGE_16U, 4},
{32, 14, 2688, 32, 0, gScenarioBufferData228_THRESHOLD_RANGE_16U, 64, gScenarioKernelData228_THRESHOLD_RANGE_16U, 4},
{36, 14, 3024, 32, 0, gScenarioBufferData229_THRESHOLD_RANGE_16U, 64, gScenarioKernelData229_THRESHOLD_RANGE_16U, 4},
{40, 14, 3360, 32, 0, gScenarioBufferData230_THRESHOLD_RANGE_16U, 64, gScenarioKernelData230_THRESHOLD_RANGE_16U, 4},
{44, 14, 3696, 32, 0, gScenarioBufferData231_THRESHOLD_RANGE_16U, 64, gScenarioKernelData231_THRESHOLD_RANGE_16U, 4},
{2, 16, 192, 32, 0, gScenarioBufferData232_THRESHOLD_RANGE_16U, 64, gScenarioKernelData232_THRESHOLD_RANGE_16U, 4},
{4, 16, 384, 32, 0, gScenarioBufferData233_THRESHOLD_RANGE_16U, 64, gScenarioKernelData233_THRESHOLD_RANGE_16U, 4},
{6, 16, 576, 32, 0, gScenarioBufferData234_THRESHOLD_RANGE_16U, 64, gScenarioKernelData234_THRESHOLD_RANGE_16U, 4},
{8, 16, 768, 32, 0, gScenarioBufferData235_THRESHOLD_RANGE_16U, 64, gScenarioKernelData235_THRESHOLD_RANGE_16U, 4},
{10, 16, 960, 32, 0, gScenarioBufferData236_THRESHOLD_RANGE_16U, 64, gScenarioKernelData236_THRESHOLD_RANGE_16U, 4},
{12, 16, 1152, 32, 0, gScenarioBufferData237_THRESHOLD_RANGE_16U, 64, gScenarioKernelData237_THRESHOLD_RANGE_16U, 4},
{14, 16, 1344, 32, 0, gScenarioBufferData238_THRESHOLD_RANGE_16U, 64, gScenarioKernelData238_THRESHOLD_RANGE_16U, 4},
{16, 16, 1536, 32, 0, gScenarioBufferData239_THRESHOLD_RANGE_16U, 64, gScenarioKernelData239_THRESHOLD_RANGE_16U, 4},
{18, 16, 1728, 32, 0, gScenarioBufferData240_THRESHOLD_RANGE_16U, 64, gScenarioKernelData240_THRESHOLD_RANGE_16U, 4},
{20, 16, 1920, 32, 0, gScenarioBufferData241_THRESHOLD_RANGE_16U, 64, gScenarioKernelData241_THRESHOLD_RANGE_16U, 4},
{22, 16, 2112, 32, 0, gScenarioBufferData242_THRESHOLD_RANGE_16U, 64, gScenarioKernelData242_THRESHOLD_RANGE_16U, 4},
{24, 16, 2304, 32, 0, gScenarioBufferData243_THRESHOLD_RANGE_16U, 64, gScenarioKernelData243_THRESHOLD_RANGE_16U, 4},
{26, 16, 2496, 32, 0, gScenarioBufferData244_THRESHOLD_RANGE_16U, 64, gScenarioKernelData244_THRESHOLD_RANGE_16U, 4},
{28, 16, 2688, 32, 0, gScenarioBufferData245_THRESHOLD_RANGE_16U, 64, gScenarioKernelData245_THRESHOLD_RANGE_16U, 4},
{30, 16, 2880, 32, 0, gScenarioBufferData246_THRESHOLD_RANGE_16U, 64, gScenarioKernelData246_THRESHOLD_RANGE_16U, 4},
{32, 16, 3072, 32, 0, gScenarioBufferData247_THRESHOLD_RANGE_16U, 64, gScenarioKernelData247_THRESHOLD_RANGE_16U, 4},
{36, 16, 3456, 32, 0, gScenarioBufferData248_THRESHOLD_RANGE_16U, 64, gScenarioKernelData248_THRESHOLD_RANGE_16U, 4},
{40, 16, 3840, 32, 0, gScenarioBufferData249_THRESHOLD_RANGE_16U, 64, gScenarioKernelData249_THRESHOLD_RANGE_16U, 4},
{2, 18, 216, 32, 0, gScenarioBufferData250_THRESHOLD_RANGE_16U, 64, gScenarioKernelData250_THRESHOLD_RANGE_16U, 4},
{4, 18, 432, 32, 0, gScenarioBufferData251_THRESHOLD_RANGE_16U, 64, gScenarioKernelData251_THRESHOLD_RANGE_16U, 4},
{6, 18, 648, 32, 0, gScenarioBufferData252_THRESHOLD_RANGE_16U, 64, gScenarioKernelData252_THRESHOLD_RANGE_16U, 4},
{8, 18, 864, 32, 0, gScenarioBufferData253_THRESHOLD_RANGE_16U, 64, gScenarioKernelData253_THRESHOLD_RANGE_16U, 4},
{10, 18, 1080, 32, 0, gScenarioBufferData254_THRESHOLD_RANGE_16U, 64, gScenarioKernelData254_THRESHOLD_RANGE_16U, 4},
{12, 18, 1296, 32, 0, gScenarioBufferData255_THRESHOLD_RANGE_16U, 64, gScenarioKernelData255_THRESHOLD_RANGE_16U, 4},
{14, 18, 1512, 32, 0, gScenarioBufferData256_THRESHOLD_RANGE_16U, 64, gScenarioKernelData256_THRESHOLD_RANGE_16U, 4},
{16, 18, 1728, 32, 0, gScenarioBufferData257_THRESHOLD_RANGE_16U, 64, gScenarioKernelData257_THRESHOLD_RANGE_16U, 4},
{18, 18, 1944, 32, 0, gScenarioBufferData258_THRESHOLD_RANGE_16U, 64, gScenarioKernelData258_THRESHOLD_RANGE_16U, 4},
{20, 18, 2160, 32, 0, gScenarioBufferData259_THRESHOLD_RANGE_16U, 64, gScenarioKernelData259_THRESHOLD_RANGE_16U, 4},
{22, 18, 2376, 32, 0, gScenarioBufferData260_THRESHOLD_RANGE_16U, 64, gScenarioKernelData260_THRESHOLD_RANGE_16U, 4},
{24, 18, 2592, 32, 0, gScenarioBufferData261_THRESHOLD_RANGE_16U, 64, gScenarioKernelData261_THRESHOLD_RANGE_16U, 4},
{26, 18, 2808, 32, 0, gScenarioBufferData262_THRESHOLD_RANGE_16U, 64, gScenarioKernelData262_THRESHOLD_RANGE_16U, 4},
{28, 18, 3024, 32, 0, gScenarioBufferData263_THRESHOLD_RANGE_16U, 64, gScenarioKernelData263_THRESHOLD_RANGE_16U, 4},
{30, 18, 3240, 32, 0, gScenarioBufferData264_THRESHOLD_RANGE_16U, 64, gScenarioKernelData264_THRESHOLD_RANGE_16U, 4},
{32, 18, 3456, 32, 0, gScenarioBufferData265_THRESHOLD_RANGE_16U, 64, gScenarioKernelData265_THRESHOLD_RANGE_16U, 4},
{2, 20, 240, 32, 0, gScenarioBufferData266_THRESHOLD_RANGE_16U, 64, gScenarioKernelData266_THRESHOLD_RANGE_16U, 4},
{4, 20, 480, 32, 0, gScenarioBufferData267_THRESHOLD_RANGE_16U, 64, gScenarioKernelData267_THRESHOLD_RANGE_16U, 4},
{6, 20, 720, 32, 0, gScenarioBufferData268_THRESHOLD_RANGE_16U, 64, gScenarioKernelData268_THRESHOLD_RANGE_16U, 4},
{8, 20, 960, 32, 0, gScenarioBufferData269_THRESHOLD_RANGE_16U, 64, gScenarioKernelData269_THRESHOLD_RANGE_16U, 4},
{10, 20, 1200, 32, 0, gScenarioBufferData270_THRESHOLD_RANGE_16U, 64, gScenarioKernelData270_THRESHOLD_RANGE_16U, 4},
{12, 20, 1440, 32, 0, gScenarioBufferData271_THRESHOLD_RANGE_16U, 64, gScenarioKernelData271_THRESHOLD_RANGE_16U, 4},
{14, 20, 1680, 32, 0, gScenarioBufferData272_THRESHOLD_RANGE_16U, 64, gScenarioKernelData272_THRESHOLD_RANGE_16U, 4},
{16, 20, 1920, 32, 0, gScenarioBufferData273_THRESHOLD_RANGE_16U, 64, gScenarioKernelData273_THRESHOLD_RANGE_16U, 4},
{18, 20, 2160, 32, 0, gScenarioBufferData274_THRESHOLD_RANGE_16U, 64, gScenarioKernelData274_THRESHOLD_RANGE_16U, 4},
{20, 20, 2400, 32, 0, gScenarioBufferData275_THRESHOLD_RANGE_16U, 64, gScenarioKernelData275_THRESHOLD_RANGE_16U, 4},
{22, 20, 2640, 32, 0, gScenarioBufferData276_THRESHOLD_RANGE_16U, 64, gScenarioKernelData276_THRESHOLD_RANGE_16U, 4},
{24, 20, 2880, 32, 0, gScenarioBufferData277_THRESHOLD_RANGE_16U, 64, gScenarioKernelData277_THRESHOLD_RANGE_16U, 4},
{26, 20, 3120, 32, 0, gScenarioBufferData278_THRESHOLD_RANGE_16U, 64, gScenarioKernelData278_THRESHOLD_RANGE_16U, 4},
{28, 20, 3360, 32, 0, gScenarioBufferData279_THRESHOLD_RANGE_16U, 64, gScenarioKernelData279_THRESHOLD_RANGE_16U, 4},
{30, 20, 3600, 32, 0, gScenarioBufferData280_THRESHOLD_RANGE_16U, 64, gScenarioKernelData280_THRESHOLD_RANGE_16U, 4},
{32, 20, 3840, 32, 0, gScenarioBufferData281_THRESHOLD_RANGE_16U, 64, gScenarioKernelData281_THRESHOLD_RANGE_16U, 4},
{2, 22, 264, 32, 0, gScenarioBufferData282_THRESHOLD_RANGE_16U, 64, gScenarioKernelData282_THRESHOLD_RANGE_16U, 4},
{4, 22, 528, 32, 0, gScenarioBufferData283_THRESHOLD_RANGE_16U, 64, gScenarioKernelData283_THRESHOLD_RANGE_16U, 4},
{6, 22, 792, 32, 0, gScenarioBufferData284_THRESHOLD_RANGE_16U, 64, gScenarioKernelData284_THRESHOLD_RANGE_16U, 4},
{8, 22, 1056, 32, 0, gScenarioBufferData285_THRESHOLD_RANGE_16U, 64, gScenarioKernelData285_THRESHOLD_RANGE_16U, 4},
{10, 22, 1320, 32, 0, gScenarioBufferData286_THRESHOLD_RANGE_16U, 64, gScenarioKernelData286_THRESHOLD_RANGE_16U, 4},
{12, 22, 1584, 32, 0, gScenarioBufferData287_THRESHOLD_RANGE_16U, 64, gScenarioKernelData287_THRESHOLD_RANGE_16U, 4},
{14, 22, 1848, 32, 0, gScenarioBufferData288_THRESHOLD_RANGE_16U, 64, gScenarioKernelData288_THRESHOLD_RANGE_16U, 4},
{16, 22, 2112, 32, 0, gScenarioBufferData289_THRESHOLD_RANGE_16U, 64, gScenarioKernelData289_THRESHOLD_RANGE_16U, 4},
{18, 22, 2376, 32, 0, gScenarioBufferData290_THRESHOLD_RANGE_16U, 64, gScenarioKernelData290_THRESHOLD_RANGE_16U, 4},
{20, 22, 2640, 32, 0, gScenarioBufferData291_THRESHOLD_RANGE_16U, 64, gScenarioKernelData291_THRESHOLD_RANGE_16U, 4},
{22, 22, 2904, 32, 0, gScenarioBufferData292_THRESHOLD_RANGE_16U, 64, gScenarioKernelData292_THRESHOLD_RANGE_16U, 4},
{24, 22, 3168, 32, 0, gScenarioBufferData293_THRESHOLD_RANGE_16U, 64, gScenarioKernelData293_THRESHOLD_RANGE_16U, 4},
{26, 22, 3432, 32, 0, gScenarioBufferData294_THRESHOLD_RANGE_16U, 64, gScenarioKernelData294_THRESHOLD_RANGE_16U, 4},
{28, 22, 3696, 32, 0, gScenarioBufferData295_THRESHOLD_RANGE_16U, 64, gScenarioKernelData295_THRESHOLD_RANGE_16U, 4},
{2, 24, 288, 32, 0, gScenarioBufferData296_THRESHOLD_RANGE_16U, 64, gScenarioKernelData296_THRESHOLD_RANGE_16U, 4},
{4, 24, 576, 32, 0, gScenarioBufferData297_THRESHOLD_RANGE_16U, 64, gScenarioKernelData297_THRESHOLD_RANGE_16U, 4},
{6, 24, 864, 32, 0, gScenarioBufferData298_THRESHOLD_RANGE_16U, 64, gScenarioKernelData298_THRESHOLD_RANGE_16U, 4},
{8, 24, 1152, 32, 0, gScenarioBufferData299_THRESHOLD_RANGE_16U, 64, gScenarioKernelData299_THRESHOLD_RANGE_16U, 4},
{10, 24, 1440, 32, 0, gScenarioBufferData300_THRESHOLD_RANGE_16U, 64, gScenarioKernelData300_THRESHOLD_RANGE_16U, 4},
{12, 24, 1728, 32, 0, gScenarioBufferData301_THRESHOLD_RANGE_16U, 64, gScenarioKernelData301_THRESHOLD_RANGE_16U, 4},
{14, 24, 2016, 32, 0, gScenarioBufferData302_THRESHOLD_RANGE_16U, 64, gScenarioKernelData302_THRESHOLD_RANGE_16U, 4},
{16, 24, 2304, 32, 0, gScenarioBufferData303_THRESHOLD_RANGE_16U, 64, gScenarioKernelData303_THRESHOLD_RANGE_16U, 4},
{18, 24, 2592, 32, 0, gScenarioBufferData304_THRESHOLD_RANGE_16U, 64, gScenarioKernelData304_THRESHOLD_RANGE_16U, 4},
{20, 24, 2880, 32, 0, gScenarioBufferData305_THRESHOLD_RANGE_16U, 64, gScenarioKernelData305_THRESHOLD_RANGE_16U, 4},
{22, 24, 3168, 32, 0, gScenarioBufferData306_THRESHOLD_RANGE_16U, 64, gScenarioKernelData306_THRESHOLD_RANGE_16U, 4},
{24, 24, 3456, 32, 0, gScenarioBufferData307_THRESHOLD_RANGE_16U, 64, gScenarioKernelData307_THRESHOLD_RANGE_16U, 4},
{26, 24, 3744, 32, 0, gScenarioBufferData308_THRESHOLD_RANGE_16U, 64, gScenarioKernelData308_THRESHOLD_RANGE_16U, 4},
{2, 26, 312, 32, 0, gScenarioBufferData309_THRESHOLD_RANGE_16U, 64, gScenarioKernelData309_THRESHOLD_RANGE_16U, 4},
{4, 26, 624, 32, 0, gScenarioBufferData310_THRESHOLD_RANGE_16U, 64, gScenarioKernelData310_THRESHOLD_RANGE_16U, 4},
{6, 26, 936, 32, 0, gScenarioBufferData311_THRESHOLD_RANGE_16U, 64, gScenarioKernelData311_THRESHOLD_RANGE_16U, 4},
{8, 26, 1248, 32, 0, gScenarioBufferData312_THRESHOLD_RANGE_16U, 64, gScenarioKernelData312_THRESHOLD_RANGE_16U, 4},
{10, 26, 1560, 32, 0, gScenarioBufferData313_THRESHOLD_RANGE_16U, 64, gScenarioKernelData313_THRESHOLD_RANGE_16U, 4},
{12, 26, 1872, 32, 0, gScenarioBufferData314_THRESHOLD_RANGE_16U, 64, gScenarioKernelData314_THRESHOLD_RANGE_16U, 4},
{14, 26, 2184, 32, 0, gScenarioBufferData315_THRESHOLD_RANGE_16U, 64, gScenarioKernelData315_THRESHOLD_RANGE_16U, 4},
{16, 26, 2496, 32, 0, gScenarioBufferData316_THRESHOLD_RANGE_16U, 64, gScenarioKernelData316_THRESHOLD_RANGE_16U, 4},
{18, 26, 2808, 32, 0, gScenarioBufferData317_THRESHOLD_RANGE_16U, 64, gScenarioKernelData317_THRESHOLD_RANGE_16U, 4},
{20, 26, 3120, 32, 0, gScenarioBufferData318_THRESHOLD_RANGE_16U, 64, gScenarioKernelData318_THRESHOLD_RANGE_16U, 4},
{22, 26, 3432, 32, 0, gScenarioBufferData319_THRESHOLD_RANGE_16U, 64, gScenarioKernelData319_THRESHOLD_RANGE_16U, 4},
{24, 26, 3744, 32, 0, gScenarioBufferData320_THRESHOLD_RANGE_16U, 64, gScenarioKernelData320_THRESHOLD_RANGE_16U, 4},
{2, 28, 336, 32, 0, gScenarioBufferData321_THRESHOLD_RANGE_16U, 64, gScenarioKernelData321_THRESHOLD_RANGE_16U, 4},
{4, 28, 672, 32, 0, gScenarioBufferData322_THRESHOLD_RANGE_16U, 64, gScenarioKernelData322_THRESHOLD_RANGE_16U, 4},
{6, 28, 1008, 32, 0, gScenarioBufferData323_THRESHOLD_RANGE_16U, 64, gScenarioKernelData323_THRESHOLD_RANGE_16U, 4},
{8, 28, 1344, 32, 0, gScenarioBufferData324_THRESHOLD_RANGE_16U, 64, gScenarioKernelData324_THRESHOLD_RANGE_16U, 4},
{10, 28, 1680, 32, 0, gScenarioBufferData325_THRESHOLD_RANGE_16U, 64, gScenarioKernelData325_THRESHOLD_RANGE_16U, 4},
{12, 28, 2016, 32, 0, gScenarioBufferData326_THRESHOLD_RANGE_16U, 64, gScenarioKernelData326_THRESHOLD_RANGE_16U, 4},
{14, 28, 2352, 32, 0, gScenarioBufferData327_THRESHOLD_RANGE_16U, 64, gScenarioKernelData327_THRESHOLD_RANGE_16U, 4},
{16, 28, 2688, 32, 0, gScenarioBufferData328_THRESHOLD_RANGE_16U, 64, gScenarioKernelData328_THRESHOLD_RANGE_16U, 4},
{18, 28, 3024, 32, 0, gScenarioBufferData329_THRESHOLD_RANGE_16U, 64, gScenarioKernelData329_THRESHOLD_RANGE_16U, 4},
{20, 28, 3360, 32, 0, gScenarioBufferData330_THRESHOLD_RANGE_16U, 64, gScenarioKernelData330_THRESHOLD_RANGE_16U, 4},
{22, 28, 3696, 32, 0, gScenarioBufferData331_THRESHOLD_RANGE_16U, 64, gScenarioKernelData331_THRESHOLD_RANGE_16U, 4},
{2, 30, 360, 32, 0, gScenarioBufferData332_THRESHOLD_RANGE_16U, 64, gScenarioKernelData332_THRESHOLD_RANGE_16U, 4},
{4, 30, 720, 32, 0, gScenarioBufferData333_THRESHOLD_RANGE_16U, 64, gScenarioKernelData333_THRESHOLD_RANGE_16U, 4},
{6, 30, 1080, 32, 0, gScenarioBufferData334_THRESHOLD_RANGE_16U, 64, gScenarioKernelData334_THRESHOLD_RANGE_16U, 4},
{8, 30, 1440, 32, 0, gScenarioBufferData335_THRESHOLD_RANGE_16U, 64, gScenarioKernelData335_THRESHOLD_RANGE_16U, 4},
{10, 30, 1800, 32, 0, gScenarioBufferData336_THRESHOLD_RANGE_16U, 64, gScenarioKernelData336_THRESHOLD_RANGE_16U, 4},
{12, 30, 2160, 32, 0, gScenarioBufferData337_THRESHOLD_RANGE_16U, 64, gScenarioKernelData337_THRESHOLD_RANGE_16U, 4},
{14, 30, 2520, 32, 0, gScenarioBufferData338_THRESHOLD_RANGE_16U, 64, gScenarioKernelData338_THRESHOLD_RANGE_16U, 4},
{16, 30, 2880, 32, 0, gScenarioBufferData339_THRESHOLD_RANGE_16U, 64, gScenarioKernelData339_THRESHOLD_RANGE_16U, 4},
{18, 30, 3240, 32, 0, gScenarioBufferData340_THRESHOLD_RANGE_16U, 64, gScenarioKernelData340_THRESHOLD_RANGE_16U, 4},
{20, 30, 3600, 32, 0, gScenarioBufferData341_THRESHOLD_RANGE_16U, 64, gScenarioKernelData341_THRESHOLD_RANGE_16U, 4},
{2, 32, 384, 32, 0, gScenarioBufferData342_THRESHOLD_RANGE_16U, 64, gScenarioKernelData342_THRESHOLD_RANGE_16U, 4},
{4, 32, 768, 32, 0, gScenarioBufferData343_THRESHOLD_RANGE_16U, 64, gScenarioKernelData343_THRESHOLD_RANGE_16U, 4},
{6, 32, 1152, 32, 0, gScenarioBufferData344_THRESHOLD_RANGE_16U, 64, gScenarioKernelData344_THRESHOLD_RANGE_16U, 4},
{8, 32, 1536, 32, 0, gScenarioBufferData345_THRESHOLD_RANGE_16U, 64, gScenarioKernelData345_THRESHOLD_RANGE_16U, 4},
{10, 32, 1920, 32, 0, gScenarioBufferData346_THRESHOLD_RANGE_16U, 64, gScenarioKernelData346_THRESHOLD_RANGE_16U, 4},
{12, 32, 2304, 32, 0, gScenarioBufferData347_THRESHOLD_RANGE_16U, 64, gScenarioKernelData347_THRESHOLD_RANGE_16U, 4},
{14, 32, 2688, 32, 0, gScenarioBufferData348_THRESHOLD_RANGE_16U, 64, gScenarioKernelData348_THRESHOLD_RANGE_16U, 4},
{16, 32, 3072, 32, 0, gScenarioBufferData349_THRESHOLD_RANGE_16U, 64, gScenarioKernelData349_THRESHOLD_RANGE_16U, 4},
{18, 32, 3456, 32, 0, gScenarioBufferData350_THRESHOLD_RANGE_16U, 64, gScenarioKernelData350_THRESHOLD_RANGE_16U, 4},
{20, 32, 3840, 32, 0, gScenarioBufferData351_THRESHOLD_RANGE_16U, 64, gScenarioKernelData351_THRESHOLD_RANGE_16U, 4}
};

static acf_scenario_list gScenarioList_THRESHOLD_RANGE_16U = {
352, //number of scenarios
gScenarioArray_THRESHOLD_RANGE_16U};
//**************************************************************

class THRESHOLD_RANGE_16U : public ACF_Process_APU
{

public:
   THRESHOLD_RANGE_16U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("THRESHOLD_RANGE_16U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("THRESHOLD_RANGE_16U",
                                        THRESHOLD_RANGE_16U_LOAD_SEGMENTS,
                                        THRESHOLD_RANGE_16U_LOAD_PMEM, THRESHOLD_RANGE_16U_LOAD_PMEM_SIZE,
                                        THRESHOLD_RANGE_16U_LOAD_DMEM, THRESHOLD_RANGE_16U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(THRESHOLD_RANGE_16U_APEX_LOG_BUFFER, THRESHOLD_RANGE_16U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_16U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_16U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_THRESHOLD_RANGE_16U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_THRESHOLD_RANGE_16U

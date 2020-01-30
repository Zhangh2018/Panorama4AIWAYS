#ifndef _ACF_PROCESS_APU_THRESHOLD_RANGE_08U
#define _ACF_PROCESS_APU_THRESHOLD_RANGE_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <THRESHOLD_RANGE_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_THRESHOLD_RANGE_08U[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_THRESHOLD_RANGE_08U[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_THRESHOLD_RANGE_08U[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_THRESHOLD_RANGE_08U[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_THRESHOLD_RANGE_08U[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_THRESHOLD_RANGE_08U[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_THRESHOLD_RANGE_08U[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_THRESHOLD_RANGE_08U[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_THRESHOLD_RANGE_08U[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_THRESHOLD_RANGE_08U[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_THRESHOLD_RANGE_08U[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_THRESHOLD_RANGE_08U[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_THRESHOLD_RANGE_08U[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_THRESHOLD_RANGE_08U[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_THRESHOLD_RANGE_08U[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_THRESHOLD_RANGE_08U[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_THRESHOLD_RANGE_08U[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_THRESHOLD_RANGE_08U[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_THRESHOLD_RANGE_08U[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_THRESHOLD_RANGE_08U[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_THRESHOLD_RANGE_08U[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_THRESHOLD_RANGE_08U[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_THRESHOLD_RANGE_08U[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_THRESHOLD_RANGE_08U[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_THRESHOLD_RANGE_08U[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_THRESHOLD_RANGE_08U[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_THRESHOLD_RANGE_08U[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_THRESHOLD_RANGE_08U[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_THRESHOLD_RANGE_08U[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_THRESHOLD_RANGE_08U[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_THRESHOLD_RANGE_08U[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_THRESHOLD_RANGE_08U[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_THRESHOLD_RANGE_08U[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_THRESHOLD_RANGE_08U[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_THRESHOLD_RANGE_08U[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_THRESHOLD_RANGE_08U[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_THRESHOLD_RANGE_08U[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_THRESHOLD_RANGE_08U[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_THRESHOLD_RANGE_08U[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_THRESHOLD_RANGE_08U[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_THRESHOLD_RANGE_08U[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_THRESHOLD_RANGE_08U[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_THRESHOLD_RANGE_08U[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_THRESHOLD_RANGE_08U[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_THRESHOLD_RANGE_08U[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_THRESHOLD_RANGE_08U[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_THRESHOLD_RANGE_08U[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_THRESHOLD_RANGE_08U[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_THRESHOLD_RANGE_08U[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_THRESHOLD_RANGE_08U[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_THRESHOLD_RANGE_08U[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_THRESHOLD_RANGE_08U[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_THRESHOLD_RANGE_08U[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_THRESHOLD_RANGE_08U[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_THRESHOLD_RANGE_08U[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_THRESHOLD_RANGE_08U[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_THRESHOLD_RANGE_08U[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_THRESHOLD_RANGE_08U[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_THRESHOLD_RANGE_08U[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_THRESHOLD_RANGE_08U[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_THRESHOLD_RANGE_08U[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_THRESHOLD_RANGE_08U[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_THRESHOLD_RANGE_08U[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_THRESHOLD_RANGE_08U[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_THRESHOLD_RANGE_08U[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_THRESHOLD_RANGE_08U[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_THRESHOLD_RANGE_08U[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_THRESHOLD_RANGE_08U[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_THRESHOLD_RANGE_08U[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_THRESHOLD_RANGE_08U[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_THRESHOLD_RANGE_08U[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_THRESHOLD_RANGE_08U[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_THRESHOLD_RANGE_08U[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_THRESHOLD_RANGE_08U[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_THRESHOLD_RANGE_08U[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_THRESHOLD_RANGE_08U[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_THRESHOLD_RANGE_08U[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_THRESHOLD_RANGE_08U[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_THRESHOLD_RANGE_08U[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_THRESHOLD_RANGE_08U[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_THRESHOLD_RANGE_08U[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_THRESHOLD_RANGE_08U[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_THRESHOLD_RANGE_08U[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_THRESHOLD_RANGE_08U[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_THRESHOLD_RANGE_08U[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_THRESHOLD_RANGE_08U[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_THRESHOLD_RANGE_08U[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_THRESHOLD_RANGE_08U[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_THRESHOLD_RANGE_08U[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_THRESHOLD_RANGE_08U[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_THRESHOLD_RANGE_08U[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_THRESHOLD_RANGE_08U[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_THRESHOLD_RANGE_08U[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_THRESHOLD_RANGE_08U[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_THRESHOLD_RANGE_08U[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_THRESHOLD_RANGE_08U[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_THRESHOLD_RANGE_08U[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_THRESHOLD_RANGE_08U[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_THRESHOLD_RANGE_08U[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_THRESHOLD_RANGE_08U[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_THRESHOLD_RANGE_08U[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_THRESHOLD_RANGE_08U[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_THRESHOLD_RANGE_08U[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_THRESHOLD_RANGE_08U[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_THRESHOLD_RANGE_08U[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_THRESHOLD_RANGE_08U[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_THRESHOLD_RANGE_08U[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_THRESHOLD_RANGE_08U[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_THRESHOLD_RANGE_08U[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_THRESHOLD_RANGE_08U[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_THRESHOLD_RANGE_08U[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_THRESHOLD_RANGE_08U[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_THRESHOLD_RANGE_08U[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_THRESHOLD_RANGE_08U[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_THRESHOLD_RANGE_08U[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_THRESHOLD_RANGE_08U[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_THRESHOLD_RANGE_08U[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_THRESHOLD_RANGE_08U[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_THRESHOLD_RANGE_08U[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_THRESHOLD_RANGE_08U[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_THRESHOLD_RANGE_08U[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_THRESHOLD_RANGE_08U[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_THRESHOLD_RANGE_08U[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_THRESHOLD_RANGE_08U[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_THRESHOLD_RANGE_08U[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_THRESHOLD_RANGE_08U[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_THRESHOLD_RANGE_08U[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_THRESHOLD_RANGE_08U[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_THRESHOLD_RANGE_08U[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_THRESHOLD_RANGE_08U[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_THRESHOLD_RANGE_08U[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_THRESHOLD_RANGE_08U[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_THRESHOLD_RANGE_08U[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_THRESHOLD_RANGE_08U[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_THRESHOLD_RANGE_08U[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_THRESHOLD_RANGE_08U[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_THRESHOLD_RANGE_08U[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_THRESHOLD_RANGE_08U[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_THRESHOLD_RANGE_08U[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_THRESHOLD_RANGE_08U[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_THRESHOLD_RANGE_08U[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_THRESHOLD_RANGE_08U[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_THRESHOLD_RANGE_08U[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_THRESHOLD_RANGE_08U[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_THRESHOLD_RANGE_08U[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_THRESHOLD_RANGE_08U[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_THRESHOLD_RANGE_08U[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_THRESHOLD_RANGE_08U[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_THRESHOLD_RANGE_08U[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_THRESHOLD_RANGE_08U[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_THRESHOLD_RANGE_08U[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_THRESHOLD_RANGE_08U[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_THRESHOLD_RANGE_08U[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_THRESHOLD_RANGE_08U[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_THRESHOLD_RANGE_08U[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_THRESHOLD_RANGE_08U[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_THRESHOLD_RANGE_08U[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_THRESHOLD_RANGE_08U[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_THRESHOLD_RANGE_08U[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_THRESHOLD_RANGE_08U[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_THRESHOLD_RANGE_08U[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_THRESHOLD_RANGE_08U[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_THRESHOLD_RANGE_08U[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_THRESHOLD_RANGE_08U[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_THRESHOLD_RANGE_08U[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_THRESHOLD_RANGE_08U[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_THRESHOLD_RANGE_08U[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_THRESHOLD_RANGE_08U[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_THRESHOLD_RANGE_08U[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_THRESHOLD_RANGE_08U[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_THRESHOLD_RANGE_08U[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_THRESHOLD_RANGE_08U[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_THRESHOLD_RANGE_08U[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_THRESHOLD_RANGE_08U[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_THRESHOLD_RANGE_08U[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_THRESHOLD_RANGE_08U[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_THRESHOLD_RANGE_08U[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_THRESHOLD_RANGE_08U[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_THRESHOLD_RANGE_08U[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_THRESHOLD_RANGE_08U[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_THRESHOLD_RANGE_08U[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_THRESHOLD_RANGE_08U[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_THRESHOLD_RANGE_08U[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_THRESHOLD_RANGE_08U[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_THRESHOLD_RANGE_08U[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_THRESHOLD_RANGE_08U[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_THRESHOLD_RANGE_08U[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_THRESHOLD_RANGE_08U[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_THRESHOLD_RANGE_08U[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_THRESHOLD_RANGE_08U[] = {{112, 6, 2, 0, {0, 0, 0, 0}}, {112, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_THRESHOLD_RANGE_08U[] = {{120, 6, 2, 0, {0, 0, 0, 0}}, {120, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_THRESHOLD_RANGE_08U[] = {{128, 6, 2, 0, {0, 0, 0, 0}}, {128, 6, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_THRESHOLD_RANGE_08U[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_THRESHOLD_RANGE_08U[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_THRESHOLD_RANGE_08U[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_THRESHOLD_RANGE_08U[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_THRESHOLD_RANGE_08U[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_THRESHOLD_RANGE_08U[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_THRESHOLD_RANGE_08U[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_THRESHOLD_RANGE_08U[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_THRESHOLD_RANGE_08U[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_THRESHOLD_RANGE_08U[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_THRESHOLD_RANGE_08U[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_THRESHOLD_RANGE_08U[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_THRESHOLD_RANGE_08U[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_THRESHOLD_RANGE_08U[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_THRESHOLD_RANGE_08U[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_THRESHOLD_RANGE_08U[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_THRESHOLD_RANGE_08U[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_THRESHOLD_RANGE_08U[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_THRESHOLD_RANGE_08U[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_THRESHOLD_RANGE_08U[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_THRESHOLD_RANGE_08U[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_THRESHOLD_RANGE_08U[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_THRESHOLD_RANGE_08U[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_THRESHOLD_RANGE_08U[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_THRESHOLD_RANGE_08U[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_THRESHOLD_RANGE_08U[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_THRESHOLD_RANGE_08U[] = {{88, 8, 2, 0, {0, 0, 0, 0}}, {88, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_THRESHOLD_RANGE_08U[] = {{96, 8, 2, 0, {0, 0, 0, 0}}, {96, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_THRESHOLD_RANGE_08U[] = {{104, 8, 2, 0, {0, 0, 0, 0}}, {104, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_THRESHOLD_RANGE_08U[] = {{112, 8, 2, 0, {0, 0, 0, 0}}, {112, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_THRESHOLD_RANGE_08U[] = {{120, 8, 2, 0, {0, 0, 0, 0}}, {120, 8, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_THRESHOLD_RANGE_08U[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_THRESHOLD_RANGE_08U[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_THRESHOLD_RANGE_08U[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_THRESHOLD_RANGE_08U[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_THRESHOLD_RANGE_08U[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_THRESHOLD_RANGE_08U[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_THRESHOLD_RANGE_08U[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_THRESHOLD_RANGE_08U[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_THRESHOLD_RANGE_08U[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_THRESHOLD_RANGE_08U[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_THRESHOLD_RANGE_08U[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_THRESHOLD_RANGE_08U[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_THRESHOLD_RANGE_08U[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_THRESHOLD_RANGE_08U[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_THRESHOLD_RANGE_08U[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_THRESHOLD_RANGE_08U[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_THRESHOLD_RANGE_08U[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_THRESHOLD_RANGE_08U[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_THRESHOLD_RANGE_08U[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_THRESHOLD_RANGE_08U[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_THRESHOLD_RANGE_08U[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_THRESHOLD_RANGE_08U[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_THRESHOLD_RANGE_08U[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_THRESHOLD_RANGE_08U[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_THRESHOLD_RANGE_08U[] = {{72, 10, 2, 0, {0, 0, 0, 0}}, {72, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_THRESHOLD_RANGE_08U[] = {{80, 10, 2, 0, {0, 0, 0, 0}}, {80, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_THRESHOLD_RANGE_08U[] = {{88, 10, 2, 0, {0, 0, 0, 0}}, {88, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_THRESHOLD_RANGE_08U[] = {{96, 10, 2, 0, {0, 0, 0, 0}}, {96, 10, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_THRESHOLD_RANGE_08U[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_THRESHOLD_RANGE_08U[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_THRESHOLD_RANGE_08U[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_THRESHOLD_RANGE_08U[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_THRESHOLD_RANGE_08U[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_THRESHOLD_RANGE_08U[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_THRESHOLD_RANGE_08U[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_THRESHOLD_RANGE_08U[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_THRESHOLD_RANGE_08U[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_THRESHOLD_RANGE_08U[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_THRESHOLD_RANGE_08U[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_THRESHOLD_RANGE_08U[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_THRESHOLD_RANGE_08U[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_THRESHOLD_RANGE_08U[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_THRESHOLD_RANGE_08U[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_THRESHOLD_RANGE_08U[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_THRESHOLD_RANGE_08U[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_THRESHOLD_RANGE_08U[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_THRESHOLD_RANGE_08U[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_THRESHOLD_RANGE_08U[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_THRESHOLD_RANGE_08U[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_THRESHOLD_RANGE_08U[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {56, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_THRESHOLD_RANGE_08U[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {60, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_THRESHOLD_RANGE_08U[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {64, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_THRESHOLD_RANGE_08U[] = {{72, 12, 2, 0, {0, 0, 0, 0}}, {72, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_THRESHOLD_RANGE_08U[] = {{80, 12, 2, 0, {0, 0, 0, 0}}, {80, 12, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_THRESHOLD_RANGE_08U[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_THRESHOLD_RANGE_08U[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_THRESHOLD_RANGE_08U[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_THRESHOLD_RANGE_08U[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_THRESHOLD_RANGE_08U[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_THRESHOLD_RANGE_08U[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_THRESHOLD_RANGE_08U[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_THRESHOLD_RANGE_08U[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_THRESHOLD_RANGE_08U[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_THRESHOLD_RANGE_08U[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_THRESHOLD_RANGE_08U[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_THRESHOLD_RANGE_08U[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_THRESHOLD_RANGE_08U[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_THRESHOLD_RANGE_08U[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_THRESHOLD_RANGE_08U[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_THRESHOLD_RANGE_08U[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_THRESHOLD_RANGE_08U[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_THRESHOLD_RANGE_08U[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_THRESHOLD_RANGE_08U[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_THRESHOLD_RANGE_08U[] = {{48, 14, 2, 0, {0, 0, 0, 0}}, {48, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_THRESHOLD_RANGE_08U[] = {{52, 14, 2, 0, {0, 0, 0, 0}}, {52, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_THRESHOLD_RANGE_08U[] = {{56, 14, 2, 0, {0, 0, 0, 0}}, {56, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_THRESHOLD_RANGE_08U[] = {{60, 14, 2, 0, {0, 0, 0, 0}}, {60, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_THRESHOLD_RANGE_08U[] = {{64, 14, 2, 0, {0, 0, 0, 0}}, {64, 14, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_THRESHOLD_RANGE_08U[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_THRESHOLD_RANGE_08U[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_THRESHOLD_RANGE_08U[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_THRESHOLD_RANGE_08U[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_THRESHOLD_RANGE_08U[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_THRESHOLD_RANGE_08U[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_THRESHOLD_RANGE_08U[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_THRESHOLD_RANGE_08U[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_THRESHOLD_RANGE_08U[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_THRESHOLD_RANGE_08U[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_THRESHOLD_RANGE_08U[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_THRESHOLD_RANGE_08U[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_THRESHOLD_RANGE_08U[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_THRESHOLD_RANGE_08U[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_THRESHOLD_RANGE_08U[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_THRESHOLD_RANGE_08U[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_THRESHOLD_RANGE_08U[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_THRESHOLD_RANGE_08U[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_THRESHOLD_RANGE_08U[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {44, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_THRESHOLD_RANGE_08U[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {48, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_THRESHOLD_RANGE_08U[] = {{52, 16, 2, 0, {0, 0, 0, 0}}, {52, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_THRESHOLD_RANGE_08U[] = {{56, 16, 2, 0, {0, 0, 0, 0}}, {56, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_THRESHOLD_RANGE_08U[] = {{60, 16, 2, 0, {0, 0, 0, 0}}, {60, 16, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_THRESHOLD_RANGE_08U[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_THRESHOLD_RANGE_08U[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_THRESHOLD_RANGE_08U[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_THRESHOLD_RANGE_08U[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_THRESHOLD_RANGE_08U[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_THRESHOLD_RANGE_08U[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_THRESHOLD_RANGE_08U[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_THRESHOLD_RANGE_08U[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_THRESHOLD_RANGE_08U[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_THRESHOLD_RANGE_08U[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_THRESHOLD_RANGE_08U[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_THRESHOLD_RANGE_08U[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_THRESHOLD_RANGE_08U[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_THRESHOLD_RANGE_08U[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_THRESHOLD_RANGE_08U[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_THRESHOLD_RANGE_08U[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_THRESHOLD_RANGE_08U[] = {{36, 18, 2, 0, {0, 0, 0, 0}}, {36, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_THRESHOLD_RANGE_08U[] = {{40, 18, 2, 0, {0, 0, 0, 0}}, {40, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_THRESHOLD_RANGE_08U[] = {{44, 18, 2, 0, {0, 0, 0, 0}}, {44, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_THRESHOLD_RANGE_08U[] = {{48, 18, 2, 0, {0, 0, 0, 0}}, {48, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_THRESHOLD_RANGE_08U[] = {{52, 18, 2, 0, {0, 0, 0, 0}}, {52, 18, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_THRESHOLD_RANGE_08U[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_THRESHOLD_RANGE_08U[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_THRESHOLD_RANGE_08U[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_THRESHOLD_RANGE_08U[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_THRESHOLD_RANGE_08U[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_THRESHOLD_RANGE_08U[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_THRESHOLD_RANGE_08U[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_THRESHOLD_RANGE_08U[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_THRESHOLD_RANGE_08U[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_THRESHOLD_RANGE_08U[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_THRESHOLD_RANGE_08U[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_THRESHOLD_RANGE_08U[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_THRESHOLD_RANGE_08U[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_THRESHOLD_RANGE_08U[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_THRESHOLD_RANGE_08U[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_THRESHOLD_RANGE_08U[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_THRESHOLD_RANGE_08U[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {36, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_THRESHOLD_RANGE_08U[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {40, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_THRESHOLD_RANGE_08U[] = {{44, 20, 2, 0, {0, 0, 0, 0}}, {44, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_THRESHOLD_RANGE_08U[] = {{48, 20, 2, 0, {0, 0, 0, 0}}, {48, 20, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_THRESHOLD_RANGE_08U[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_THRESHOLD_RANGE_08U[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_THRESHOLD_RANGE_08U[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_THRESHOLD_RANGE_08U[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_THRESHOLD_RANGE_08U[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_THRESHOLD_RANGE_08U[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_THRESHOLD_RANGE_08U[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_THRESHOLD_RANGE_08U[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_THRESHOLD_RANGE_08U[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_THRESHOLD_RANGE_08U[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_THRESHOLD_RANGE_08U[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_THRESHOLD_RANGE_08U[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_THRESHOLD_RANGE_08U[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_THRESHOLD_RANGE_08U[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_THRESHOLD_RANGE_08U[] = {{30, 22, 2, 0, {0, 0, 0, 0}}, {30, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_THRESHOLD_RANGE_08U[] = {{32, 22, 2, 0, {0, 0, 0, 0}}, {32, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_THRESHOLD_RANGE_08U[] = {{36, 22, 2, 0, {0, 0, 0, 0}}, {36, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_THRESHOLD_RANGE_08U[] = {{40, 22, 2, 0, {0, 0, 0, 0}}, {40, 22, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_THRESHOLD_RANGE_08U[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_THRESHOLD_RANGE_08U[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_THRESHOLD_RANGE_08U[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_THRESHOLD_RANGE_08U[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_THRESHOLD_RANGE_08U[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_THRESHOLD_RANGE_08U[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_THRESHOLD_RANGE_08U[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_THRESHOLD_RANGE_08U[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_THRESHOLD_RANGE_08U[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_THRESHOLD_RANGE_08U[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_THRESHOLD_RANGE_08U[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_THRESHOLD_RANGE_08U[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_THRESHOLD_RANGE_08U[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_THRESHOLD_RANGE_08U[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {28, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_THRESHOLD_RANGE_08U[] = {{30, 24, 2, 0, {0, 0, 0, 0}}, {30, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_THRESHOLD_RANGE_08U[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {32, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData399_THRESHOLD_RANGE_08U[] = {{36, 24, 2, 0, {0, 0, 0, 0}}, {36, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData400_THRESHOLD_RANGE_08U[] = {{40, 24, 2, 0, {0, 0, 0, 0}}, {40, 24, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData401_THRESHOLD_RANGE_08U[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData402_THRESHOLD_RANGE_08U[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData403_THRESHOLD_RANGE_08U[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData404_THRESHOLD_RANGE_08U[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData405_THRESHOLD_RANGE_08U[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData406_THRESHOLD_RANGE_08U[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData407_THRESHOLD_RANGE_08U[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData408_THRESHOLD_RANGE_08U[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData409_THRESHOLD_RANGE_08U[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData410_THRESHOLD_RANGE_08U[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData411_THRESHOLD_RANGE_08U[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData412_THRESHOLD_RANGE_08U[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData413_THRESHOLD_RANGE_08U[] = {{26, 26, 2, 0, {0, 0, 0, 0}}, {26, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData414_THRESHOLD_RANGE_08U[] = {{28, 26, 2, 0, {0, 0, 0, 0}}, {28, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData415_THRESHOLD_RANGE_08U[] = {{30, 26, 2, 0, {0, 0, 0, 0}}, {30, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData416_THRESHOLD_RANGE_08U[] = {{32, 26, 2, 0, {0, 0, 0, 0}}, {32, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData417_THRESHOLD_RANGE_08U[] = {{36, 26, 2, 0, {0, 0, 0, 0}}, {36, 26, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData418_THRESHOLD_RANGE_08U[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData419_THRESHOLD_RANGE_08U[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData420_THRESHOLD_RANGE_08U[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData421_THRESHOLD_RANGE_08U[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData422_THRESHOLD_RANGE_08U[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData423_THRESHOLD_RANGE_08U[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData424_THRESHOLD_RANGE_08U[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData425_THRESHOLD_RANGE_08U[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData426_THRESHOLD_RANGE_08U[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData427_THRESHOLD_RANGE_08U[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData428_THRESHOLD_RANGE_08U[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData429_THRESHOLD_RANGE_08U[] = {{24, 28, 2, 0, {0, 0, 0, 0}}, {24, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData430_THRESHOLD_RANGE_08U[] = {{26, 28, 2, 0, {0, 0, 0, 0}}, {26, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData431_THRESHOLD_RANGE_08U[] = {{28, 28, 2, 0, {0, 0, 0, 0}}, {28, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData432_THRESHOLD_RANGE_08U[] = {{30, 28, 2, 0, {0, 0, 0, 0}}, {30, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData433_THRESHOLD_RANGE_08U[] = {{32, 28, 2, 0, {0, 0, 0, 0}}, {32, 28, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData434_THRESHOLD_RANGE_08U[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData435_THRESHOLD_RANGE_08U[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData436_THRESHOLD_RANGE_08U[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData437_THRESHOLD_RANGE_08U[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData438_THRESHOLD_RANGE_08U[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData439_THRESHOLD_RANGE_08U[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData440_THRESHOLD_RANGE_08U[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData441_THRESHOLD_RANGE_08U[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData442_THRESHOLD_RANGE_08U[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData443_THRESHOLD_RANGE_08U[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData444_THRESHOLD_RANGE_08U[] = {{22, 30, 2, 0, {0, 0, 0, 0}}, {22, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData445_THRESHOLD_RANGE_08U[] = {{24, 30, 2, 0, {0, 0, 0, 0}}, {24, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData446_THRESHOLD_RANGE_08U[] = {{26, 30, 2, 0, {0, 0, 0, 0}}, {26, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData447_THRESHOLD_RANGE_08U[] = {{28, 30, 2, 0, {0, 0, 0, 0}}, {28, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData448_THRESHOLD_RANGE_08U[] = {{30, 30, 2, 0, {0, 0, 0, 0}}, {30, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData449_THRESHOLD_RANGE_08U[] = {{32, 30, 2, 0, {0, 0, 0, 0}}, {32, 30, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData450_THRESHOLD_RANGE_08U[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData451_THRESHOLD_RANGE_08U[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData452_THRESHOLD_RANGE_08U[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData453_THRESHOLD_RANGE_08U[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData454_THRESHOLD_RANGE_08U[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData455_THRESHOLD_RANGE_08U[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData456_THRESHOLD_RANGE_08U[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData457_THRESHOLD_RANGE_08U[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData458_THRESHOLD_RANGE_08U[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData459_THRESHOLD_RANGE_08U[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData460_THRESHOLD_RANGE_08U[] = {{22, 32, 2, 0, {0, 0, 0, 0}}, {22, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData461_THRESHOLD_RANGE_08U[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {24, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData462_THRESHOLD_RANGE_08U[] = {{26, 32, 2, 0, {0, 0, 0, 0}}, {26, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData463_THRESHOLD_RANGE_08U[] = {{28, 32, 2, 0, {0, 0, 0, 0}}, {28, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData464_THRESHOLD_RANGE_08U[] = {{30, 32, 2, 0, {0, 0, 0, 0}}, {30, 32, 2, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData399_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData400_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData401_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData402_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData403_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData404_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData405_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData406_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData407_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData408_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData409_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData410_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData411_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData412_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData413_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData414_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData415_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData416_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData417_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData418_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData419_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData420_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData421_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData422_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData423_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData424_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData425_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData426_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData427_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData428_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData429_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData430_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData431_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData432_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData433_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData434_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData435_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData436_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData437_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData438_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData439_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData440_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData441_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData442_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData443_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData444_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData445_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData446_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData447_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData448_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData449_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData450_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData451_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData452_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData453_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData454_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData455_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData456_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData457_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData458_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData459_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData460_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData461_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData462_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData463_THRESHOLD_RANGE_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData464_THRESHOLD_RANGE_08U[] = {{0, 0}};

static acf_scenario gScenarioArray_THRESHOLD_RANGE_08U[] = {
{2, 1, 8, 32, 0, gScenarioBufferData0_THRESHOLD_RANGE_08U, 64, gScenarioKernelData0_THRESHOLD_RANGE_08U, 4},
{4, 1, 16, 32, 0, gScenarioBufferData1_THRESHOLD_RANGE_08U, 64, gScenarioKernelData1_THRESHOLD_RANGE_08U, 4},
{6, 1, 24, 32, 0, gScenarioBufferData2_THRESHOLD_RANGE_08U, 64, gScenarioKernelData2_THRESHOLD_RANGE_08U, 4},
{8, 1, 32, 32, 0, gScenarioBufferData3_THRESHOLD_RANGE_08U, 64, gScenarioKernelData3_THRESHOLD_RANGE_08U, 4},
{10, 1, 40, 32, 0, gScenarioBufferData4_THRESHOLD_RANGE_08U, 64, gScenarioKernelData4_THRESHOLD_RANGE_08U, 4},
{12, 1, 48, 32, 0, gScenarioBufferData5_THRESHOLD_RANGE_08U, 64, gScenarioKernelData5_THRESHOLD_RANGE_08U, 4},
{14, 1, 56, 32, 0, gScenarioBufferData6_THRESHOLD_RANGE_08U, 64, gScenarioKernelData6_THRESHOLD_RANGE_08U, 4},
{16, 1, 64, 32, 0, gScenarioBufferData7_THRESHOLD_RANGE_08U, 64, gScenarioKernelData7_THRESHOLD_RANGE_08U, 4},
{18, 1, 72, 32, 0, gScenarioBufferData8_THRESHOLD_RANGE_08U, 64, gScenarioKernelData8_THRESHOLD_RANGE_08U, 4},
{20, 1, 80, 32, 0, gScenarioBufferData9_THRESHOLD_RANGE_08U, 64, gScenarioKernelData9_THRESHOLD_RANGE_08U, 4},
{22, 1, 88, 32, 0, gScenarioBufferData10_THRESHOLD_RANGE_08U, 64, gScenarioKernelData10_THRESHOLD_RANGE_08U, 4},
{24, 1, 96, 32, 0, gScenarioBufferData11_THRESHOLD_RANGE_08U, 64, gScenarioKernelData11_THRESHOLD_RANGE_08U, 4},
{26, 1, 104, 32, 0, gScenarioBufferData12_THRESHOLD_RANGE_08U, 64, gScenarioKernelData12_THRESHOLD_RANGE_08U, 4},
{28, 1, 112, 32, 0, gScenarioBufferData13_THRESHOLD_RANGE_08U, 64, gScenarioKernelData13_THRESHOLD_RANGE_08U, 4},
{30, 1, 120, 32, 0, gScenarioBufferData14_THRESHOLD_RANGE_08U, 64, gScenarioKernelData14_THRESHOLD_RANGE_08U, 4},
{32, 1, 128, 32, 0, gScenarioBufferData15_THRESHOLD_RANGE_08U, 64, gScenarioKernelData15_THRESHOLD_RANGE_08U, 4},
{36, 1, 144, 32, 0, gScenarioBufferData16_THRESHOLD_RANGE_08U, 64, gScenarioKernelData16_THRESHOLD_RANGE_08U, 4},
{40, 1, 160, 32, 0, gScenarioBufferData17_THRESHOLD_RANGE_08U, 64, gScenarioKernelData17_THRESHOLD_RANGE_08U, 4},
{44, 1, 176, 32, 0, gScenarioBufferData18_THRESHOLD_RANGE_08U, 64, gScenarioKernelData18_THRESHOLD_RANGE_08U, 4},
{48, 1, 192, 32, 0, gScenarioBufferData19_THRESHOLD_RANGE_08U, 64, gScenarioKernelData19_THRESHOLD_RANGE_08U, 4},
{52, 1, 208, 32, 0, gScenarioBufferData20_THRESHOLD_RANGE_08U, 64, gScenarioKernelData20_THRESHOLD_RANGE_08U, 4},
{56, 1, 224, 32, 0, gScenarioBufferData21_THRESHOLD_RANGE_08U, 64, gScenarioKernelData21_THRESHOLD_RANGE_08U, 4},
{60, 1, 240, 32, 0, gScenarioBufferData22_THRESHOLD_RANGE_08U, 64, gScenarioKernelData22_THRESHOLD_RANGE_08U, 4},
{64, 1, 256, 32, 0, gScenarioBufferData23_THRESHOLD_RANGE_08U, 64, gScenarioKernelData23_THRESHOLD_RANGE_08U, 4},
{72, 1, 288, 32, 0, gScenarioBufferData24_THRESHOLD_RANGE_08U, 64, gScenarioKernelData24_THRESHOLD_RANGE_08U, 4},
{80, 1, 320, 32, 0, gScenarioBufferData25_THRESHOLD_RANGE_08U, 64, gScenarioKernelData25_THRESHOLD_RANGE_08U, 4},
{88, 1, 352, 32, 0, gScenarioBufferData26_THRESHOLD_RANGE_08U, 64, gScenarioKernelData26_THRESHOLD_RANGE_08U, 4},
{96, 1, 384, 32, 0, gScenarioBufferData27_THRESHOLD_RANGE_08U, 64, gScenarioKernelData27_THRESHOLD_RANGE_08U, 4},
{104, 1, 416, 32, 0, gScenarioBufferData28_THRESHOLD_RANGE_08U, 64, gScenarioKernelData28_THRESHOLD_RANGE_08U, 4},
{112, 1, 448, 32, 0, gScenarioBufferData29_THRESHOLD_RANGE_08U, 64, gScenarioKernelData29_THRESHOLD_RANGE_08U, 4},
{120, 1, 480, 32, 0, gScenarioBufferData30_THRESHOLD_RANGE_08U, 64, gScenarioKernelData30_THRESHOLD_RANGE_08U, 4},
{128, 1, 512, 32, 0, gScenarioBufferData31_THRESHOLD_RANGE_08U, 64, gScenarioKernelData31_THRESHOLD_RANGE_08U, 4},
{2, 2, 16, 32, 0, gScenarioBufferData32_THRESHOLD_RANGE_08U, 64, gScenarioKernelData32_THRESHOLD_RANGE_08U, 4},
{4, 2, 32, 32, 0, gScenarioBufferData33_THRESHOLD_RANGE_08U, 64, gScenarioKernelData33_THRESHOLD_RANGE_08U, 4},
{6, 2, 48, 32, 0, gScenarioBufferData34_THRESHOLD_RANGE_08U, 64, gScenarioKernelData34_THRESHOLD_RANGE_08U, 4},
{8, 2, 64, 32, 0, gScenarioBufferData35_THRESHOLD_RANGE_08U, 64, gScenarioKernelData35_THRESHOLD_RANGE_08U, 4},
{10, 2, 80, 32, 0, gScenarioBufferData36_THRESHOLD_RANGE_08U, 64, gScenarioKernelData36_THRESHOLD_RANGE_08U, 4},
{12, 2, 96, 32, 0, gScenarioBufferData37_THRESHOLD_RANGE_08U, 64, gScenarioKernelData37_THRESHOLD_RANGE_08U, 4},
{14, 2, 112, 32, 0, gScenarioBufferData38_THRESHOLD_RANGE_08U, 64, gScenarioKernelData38_THRESHOLD_RANGE_08U, 4},
{16, 2, 128, 32, 0, gScenarioBufferData39_THRESHOLD_RANGE_08U, 64, gScenarioKernelData39_THRESHOLD_RANGE_08U, 4},
{18, 2, 144, 32, 0, gScenarioBufferData40_THRESHOLD_RANGE_08U, 64, gScenarioKernelData40_THRESHOLD_RANGE_08U, 4},
{20, 2, 160, 32, 0, gScenarioBufferData41_THRESHOLD_RANGE_08U, 64, gScenarioKernelData41_THRESHOLD_RANGE_08U, 4},
{22, 2, 176, 32, 0, gScenarioBufferData42_THRESHOLD_RANGE_08U, 64, gScenarioKernelData42_THRESHOLD_RANGE_08U, 4},
{24, 2, 192, 32, 0, gScenarioBufferData43_THRESHOLD_RANGE_08U, 64, gScenarioKernelData43_THRESHOLD_RANGE_08U, 4},
{26, 2, 208, 32, 0, gScenarioBufferData44_THRESHOLD_RANGE_08U, 64, gScenarioKernelData44_THRESHOLD_RANGE_08U, 4},
{28, 2, 224, 32, 0, gScenarioBufferData45_THRESHOLD_RANGE_08U, 64, gScenarioKernelData45_THRESHOLD_RANGE_08U, 4},
{30, 2, 240, 32, 0, gScenarioBufferData46_THRESHOLD_RANGE_08U, 64, gScenarioKernelData46_THRESHOLD_RANGE_08U, 4},
{32, 2, 256, 32, 0, gScenarioBufferData47_THRESHOLD_RANGE_08U, 64, gScenarioKernelData47_THRESHOLD_RANGE_08U, 4},
{36, 2, 288, 32, 0, gScenarioBufferData48_THRESHOLD_RANGE_08U, 64, gScenarioKernelData48_THRESHOLD_RANGE_08U, 4},
{40, 2, 320, 32, 0, gScenarioBufferData49_THRESHOLD_RANGE_08U, 64, gScenarioKernelData49_THRESHOLD_RANGE_08U, 4},
{44, 2, 352, 32, 0, gScenarioBufferData50_THRESHOLD_RANGE_08U, 64, gScenarioKernelData50_THRESHOLD_RANGE_08U, 4},
{48, 2, 384, 32, 0, gScenarioBufferData51_THRESHOLD_RANGE_08U, 64, gScenarioKernelData51_THRESHOLD_RANGE_08U, 4},
{52, 2, 416, 32, 0, gScenarioBufferData52_THRESHOLD_RANGE_08U, 64, gScenarioKernelData52_THRESHOLD_RANGE_08U, 4},
{56, 2, 448, 32, 0, gScenarioBufferData53_THRESHOLD_RANGE_08U, 64, gScenarioKernelData53_THRESHOLD_RANGE_08U, 4},
{60, 2, 480, 32, 0, gScenarioBufferData54_THRESHOLD_RANGE_08U, 64, gScenarioKernelData54_THRESHOLD_RANGE_08U, 4},
{64, 2, 512, 32, 0, gScenarioBufferData55_THRESHOLD_RANGE_08U, 64, gScenarioKernelData55_THRESHOLD_RANGE_08U, 4},
{72, 2, 576, 32, 0, gScenarioBufferData56_THRESHOLD_RANGE_08U, 64, gScenarioKernelData56_THRESHOLD_RANGE_08U, 4},
{80, 2, 640, 32, 0, gScenarioBufferData57_THRESHOLD_RANGE_08U, 64, gScenarioKernelData57_THRESHOLD_RANGE_08U, 4},
{88, 2, 704, 32, 0, gScenarioBufferData58_THRESHOLD_RANGE_08U, 64, gScenarioKernelData58_THRESHOLD_RANGE_08U, 4},
{96, 2, 768, 32, 0, gScenarioBufferData59_THRESHOLD_RANGE_08U, 64, gScenarioKernelData59_THRESHOLD_RANGE_08U, 4},
{104, 2, 832, 32, 0, gScenarioBufferData60_THRESHOLD_RANGE_08U, 64, gScenarioKernelData60_THRESHOLD_RANGE_08U, 4},
{112, 2, 896, 32, 0, gScenarioBufferData61_THRESHOLD_RANGE_08U, 64, gScenarioKernelData61_THRESHOLD_RANGE_08U, 4},
{120, 2, 960, 32, 0, gScenarioBufferData62_THRESHOLD_RANGE_08U, 64, gScenarioKernelData62_THRESHOLD_RANGE_08U, 4},
{128, 2, 1024, 32, 0, gScenarioBufferData63_THRESHOLD_RANGE_08U, 64, gScenarioKernelData63_THRESHOLD_RANGE_08U, 4},
{2, 3, 24, 32, 0, gScenarioBufferData64_THRESHOLD_RANGE_08U, 64, gScenarioKernelData64_THRESHOLD_RANGE_08U, 4},
{4, 3, 48, 32, 0, gScenarioBufferData65_THRESHOLD_RANGE_08U, 64, gScenarioKernelData65_THRESHOLD_RANGE_08U, 4},
{6, 3, 72, 32, 0, gScenarioBufferData66_THRESHOLD_RANGE_08U, 64, gScenarioKernelData66_THRESHOLD_RANGE_08U, 4},
{8, 3, 96, 32, 0, gScenarioBufferData67_THRESHOLD_RANGE_08U, 64, gScenarioKernelData67_THRESHOLD_RANGE_08U, 4},
{10, 3, 120, 32, 0, gScenarioBufferData68_THRESHOLD_RANGE_08U, 64, gScenarioKernelData68_THRESHOLD_RANGE_08U, 4},
{12, 3, 144, 32, 0, gScenarioBufferData69_THRESHOLD_RANGE_08U, 64, gScenarioKernelData69_THRESHOLD_RANGE_08U, 4},
{14, 3, 168, 32, 0, gScenarioBufferData70_THRESHOLD_RANGE_08U, 64, gScenarioKernelData70_THRESHOLD_RANGE_08U, 4},
{16, 3, 192, 32, 0, gScenarioBufferData71_THRESHOLD_RANGE_08U, 64, gScenarioKernelData71_THRESHOLD_RANGE_08U, 4},
{18, 3, 216, 32, 0, gScenarioBufferData72_THRESHOLD_RANGE_08U, 64, gScenarioKernelData72_THRESHOLD_RANGE_08U, 4},
{20, 3, 240, 32, 0, gScenarioBufferData73_THRESHOLD_RANGE_08U, 64, gScenarioKernelData73_THRESHOLD_RANGE_08U, 4},
{22, 3, 264, 32, 0, gScenarioBufferData74_THRESHOLD_RANGE_08U, 64, gScenarioKernelData74_THRESHOLD_RANGE_08U, 4},
{24, 3, 288, 32, 0, gScenarioBufferData75_THRESHOLD_RANGE_08U, 64, gScenarioKernelData75_THRESHOLD_RANGE_08U, 4},
{26, 3, 312, 32, 0, gScenarioBufferData76_THRESHOLD_RANGE_08U, 64, gScenarioKernelData76_THRESHOLD_RANGE_08U, 4},
{28, 3, 336, 32, 0, gScenarioBufferData77_THRESHOLD_RANGE_08U, 64, gScenarioKernelData77_THRESHOLD_RANGE_08U, 4},
{30, 3, 360, 32, 0, gScenarioBufferData78_THRESHOLD_RANGE_08U, 64, gScenarioKernelData78_THRESHOLD_RANGE_08U, 4},
{32, 3, 384, 32, 0, gScenarioBufferData79_THRESHOLD_RANGE_08U, 64, gScenarioKernelData79_THRESHOLD_RANGE_08U, 4},
{36, 3, 432, 32, 0, gScenarioBufferData80_THRESHOLD_RANGE_08U, 64, gScenarioKernelData80_THRESHOLD_RANGE_08U, 4},
{40, 3, 480, 32, 0, gScenarioBufferData81_THRESHOLD_RANGE_08U, 64, gScenarioKernelData81_THRESHOLD_RANGE_08U, 4},
{44, 3, 528, 32, 0, gScenarioBufferData82_THRESHOLD_RANGE_08U, 64, gScenarioKernelData82_THRESHOLD_RANGE_08U, 4},
{48, 3, 576, 32, 0, gScenarioBufferData83_THRESHOLD_RANGE_08U, 64, gScenarioKernelData83_THRESHOLD_RANGE_08U, 4},
{52, 3, 624, 32, 0, gScenarioBufferData84_THRESHOLD_RANGE_08U, 64, gScenarioKernelData84_THRESHOLD_RANGE_08U, 4},
{56, 3, 672, 32, 0, gScenarioBufferData85_THRESHOLD_RANGE_08U, 64, gScenarioKernelData85_THRESHOLD_RANGE_08U, 4},
{60, 3, 720, 32, 0, gScenarioBufferData86_THRESHOLD_RANGE_08U, 64, gScenarioKernelData86_THRESHOLD_RANGE_08U, 4},
{64, 3, 768, 32, 0, gScenarioBufferData87_THRESHOLD_RANGE_08U, 64, gScenarioKernelData87_THRESHOLD_RANGE_08U, 4},
{72, 3, 864, 32, 0, gScenarioBufferData88_THRESHOLD_RANGE_08U, 64, gScenarioKernelData88_THRESHOLD_RANGE_08U, 4},
{80, 3, 960, 32, 0, gScenarioBufferData89_THRESHOLD_RANGE_08U, 64, gScenarioKernelData89_THRESHOLD_RANGE_08U, 4},
{88, 3, 1056, 32, 0, gScenarioBufferData90_THRESHOLD_RANGE_08U, 64, gScenarioKernelData90_THRESHOLD_RANGE_08U, 4},
{96, 3, 1152, 32, 0, gScenarioBufferData91_THRESHOLD_RANGE_08U, 64, gScenarioKernelData91_THRESHOLD_RANGE_08U, 4},
{104, 3, 1248, 32, 0, gScenarioBufferData92_THRESHOLD_RANGE_08U, 64, gScenarioKernelData92_THRESHOLD_RANGE_08U, 4},
{112, 3, 1344, 32, 0, gScenarioBufferData93_THRESHOLD_RANGE_08U, 64, gScenarioKernelData93_THRESHOLD_RANGE_08U, 4},
{120, 3, 1440, 32, 0, gScenarioBufferData94_THRESHOLD_RANGE_08U, 64, gScenarioKernelData94_THRESHOLD_RANGE_08U, 4},
{128, 3, 1536, 32, 0, gScenarioBufferData95_THRESHOLD_RANGE_08U, 64, gScenarioKernelData95_THRESHOLD_RANGE_08U, 4},
{2, 4, 32, 32, 0, gScenarioBufferData96_THRESHOLD_RANGE_08U, 64, gScenarioKernelData96_THRESHOLD_RANGE_08U, 4},
{4, 4, 64, 32, 0, gScenarioBufferData97_THRESHOLD_RANGE_08U, 64, gScenarioKernelData97_THRESHOLD_RANGE_08U, 4},
{6, 4, 96, 32, 0, gScenarioBufferData98_THRESHOLD_RANGE_08U, 64, gScenarioKernelData98_THRESHOLD_RANGE_08U, 4},
{8, 4, 128, 32, 0, gScenarioBufferData99_THRESHOLD_RANGE_08U, 64, gScenarioKernelData99_THRESHOLD_RANGE_08U, 4},
{10, 4, 160, 32, 0, gScenarioBufferData100_THRESHOLD_RANGE_08U, 64, gScenarioKernelData100_THRESHOLD_RANGE_08U, 4},
{12, 4, 192, 32, 0, gScenarioBufferData101_THRESHOLD_RANGE_08U, 64, gScenarioKernelData101_THRESHOLD_RANGE_08U, 4},
{14, 4, 224, 32, 0, gScenarioBufferData102_THRESHOLD_RANGE_08U, 64, gScenarioKernelData102_THRESHOLD_RANGE_08U, 4},
{16, 4, 256, 32, 0, gScenarioBufferData103_THRESHOLD_RANGE_08U, 64, gScenarioKernelData103_THRESHOLD_RANGE_08U, 4},
{18, 4, 288, 32, 0, gScenarioBufferData104_THRESHOLD_RANGE_08U, 64, gScenarioKernelData104_THRESHOLD_RANGE_08U, 4},
{20, 4, 320, 32, 0, gScenarioBufferData105_THRESHOLD_RANGE_08U, 64, gScenarioKernelData105_THRESHOLD_RANGE_08U, 4},
{22, 4, 352, 32, 0, gScenarioBufferData106_THRESHOLD_RANGE_08U, 64, gScenarioKernelData106_THRESHOLD_RANGE_08U, 4},
{24, 4, 384, 32, 0, gScenarioBufferData107_THRESHOLD_RANGE_08U, 64, gScenarioKernelData107_THRESHOLD_RANGE_08U, 4},
{26, 4, 416, 32, 0, gScenarioBufferData108_THRESHOLD_RANGE_08U, 64, gScenarioKernelData108_THRESHOLD_RANGE_08U, 4},
{28, 4, 448, 32, 0, gScenarioBufferData109_THRESHOLD_RANGE_08U, 64, gScenarioKernelData109_THRESHOLD_RANGE_08U, 4},
{30, 4, 480, 32, 0, gScenarioBufferData110_THRESHOLD_RANGE_08U, 64, gScenarioKernelData110_THRESHOLD_RANGE_08U, 4},
{32, 4, 512, 32, 0, gScenarioBufferData111_THRESHOLD_RANGE_08U, 64, gScenarioKernelData111_THRESHOLD_RANGE_08U, 4},
{36, 4, 576, 32, 0, gScenarioBufferData112_THRESHOLD_RANGE_08U, 64, gScenarioKernelData112_THRESHOLD_RANGE_08U, 4},
{40, 4, 640, 32, 0, gScenarioBufferData113_THRESHOLD_RANGE_08U, 64, gScenarioKernelData113_THRESHOLD_RANGE_08U, 4},
{44, 4, 704, 32, 0, gScenarioBufferData114_THRESHOLD_RANGE_08U, 64, gScenarioKernelData114_THRESHOLD_RANGE_08U, 4},
{48, 4, 768, 32, 0, gScenarioBufferData115_THRESHOLD_RANGE_08U, 64, gScenarioKernelData115_THRESHOLD_RANGE_08U, 4},
{52, 4, 832, 32, 0, gScenarioBufferData116_THRESHOLD_RANGE_08U, 64, gScenarioKernelData116_THRESHOLD_RANGE_08U, 4},
{56, 4, 896, 32, 0, gScenarioBufferData117_THRESHOLD_RANGE_08U, 64, gScenarioKernelData117_THRESHOLD_RANGE_08U, 4},
{60, 4, 960, 32, 0, gScenarioBufferData118_THRESHOLD_RANGE_08U, 64, gScenarioKernelData118_THRESHOLD_RANGE_08U, 4},
{64, 4, 1024, 32, 0, gScenarioBufferData119_THRESHOLD_RANGE_08U, 64, gScenarioKernelData119_THRESHOLD_RANGE_08U, 4},
{72, 4, 1152, 32, 0, gScenarioBufferData120_THRESHOLD_RANGE_08U, 64, gScenarioKernelData120_THRESHOLD_RANGE_08U, 4},
{80, 4, 1280, 32, 0, gScenarioBufferData121_THRESHOLD_RANGE_08U, 64, gScenarioKernelData121_THRESHOLD_RANGE_08U, 4},
{88, 4, 1408, 32, 0, gScenarioBufferData122_THRESHOLD_RANGE_08U, 64, gScenarioKernelData122_THRESHOLD_RANGE_08U, 4},
{96, 4, 1536, 32, 0, gScenarioBufferData123_THRESHOLD_RANGE_08U, 64, gScenarioKernelData123_THRESHOLD_RANGE_08U, 4},
{104, 4, 1664, 32, 0, gScenarioBufferData124_THRESHOLD_RANGE_08U, 64, gScenarioKernelData124_THRESHOLD_RANGE_08U, 4},
{112, 4, 1792, 32, 0, gScenarioBufferData125_THRESHOLD_RANGE_08U, 64, gScenarioKernelData125_THRESHOLD_RANGE_08U, 4},
{120, 4, 1920, 32, 0, gScenarioBufferData126_THRESHOLD_RANGE_08U, 64, gScenarioKernelData126_THRESHOLD_RANGE_08U, 4},
{128, 4, 2048, 32, 0, gScenarioBufferData127_THRESHOLD_RANGE_08U, 64, gScenarioKernelData127_THRESHOLD_RANGE_08U, 4},
{2, 5, 40, 32, 0, gScenarioBufferData128_THRESHOLD_RANGE_08U, 64, gScenarioKernelData128_THRESHOLD_RANGE_08U, 4},
{4, 5, 80, 32, 0, gScenarioBufferData129_THRESHOLD_RANGE_08U, 64, gScenarioKernelData129_THRESHOLD_RANGE_08U, 4},
{6, 5, 120, 32, 0, gScenarioBufferData130_THRESHOLD_RANGE_08U, 64, gScenarioKernelData130_THRESHOLD_RANGE_08U, 4},
{8, 5, 160, 32, 0, gScenarioBufferData131_THRESHOLD_RANGE_08U, 64, gScenarioKernelData131_THRESHOLD_RANGE_08U, 4},
{10, 5, 200, 32, 0, gScenarioBufferData132_THRESHOLD_RANGE_08U, 64, gScenarioKernelData132_THRESHOLD_RANGE_08U, 4},
{12, 5, 240, 32, 0, gScenarioBufferData133_THRESHOLD_RANGE_08U, 64, gScenarioKernelData133_THRESHOLD_RANGE_08U, 4},
{14, 5, 280, 32, 0, gScenarioBufferData134_THRESHOLD_RANGE_08U, 64, gScenarioKernelData134_THRESHOLD_RANGE_08U, 4},
{16, 5, 320, 32, 0, gScenarioBufferData135_THRESHOLD_RANGE_08U, 64, gScenarioKernelData135_THRESHOLD_RANGE_08U, 4},
{18, 5, 360, 32, 0, gScenarioBufferData136_THRESHOLD_RANGE_08U, 64, gScenarioKernelData136_THRESHOLD_RANGE_08U, 4},
{20, 5, 400, 32, 0, gScenarioBufferData137_THRESHOLD_RANGE_08U, 64, gScenarioKernelData137_THRESHOLD_RANGE_08U, 4},
{22, 5, 440, 32, 0, gScenarioBufferData138_THRESHOLD_RANGE_08U, 64, gScenarioKernelData138_THRESHOLD_RANGE_08U, 4},
{24, 5, 480, 32, 0, gScenarioBufferData139_THRESHOLD_RANGE_08U, 64, gScenarioKernelData139_THRESHOLD_RANGE_08U, 4},
{26, 5, 520, 32, 0, gScenarioBufferData140_THRESHOLD_RANGE_08U, 64, gScenarioKernelData140_THRESHOLD_RANGE_08U, 4},
{28, 5, 560, 32, 0, gScenarioBufferData141_THRESHOLD_RANGE_08U, 64, gScenarioKernelData141_THRESHOLD_RANGE_08U, 4},
{30, 5, 600, 32, 0, gScenarioBufferData142_THRESHOLD_RANGE_08U, 64, gScenarioKernelData142_THRESHOLD_RANGE_08U, 4},
{32, 5, 640, 32, 0, gScenarioBufferData143_THRESHOLD_RANGE_08U, 64, gScenarioKernelData143_THRESHOLD_RANGE_08U, 4},
{36, 5, 720, 32, 0, gScenarioBufferData144_THRESHOLD_RANGE_08U, 64, gScenarioKernelData144_THRESHOLD_RANGE_08U, 4},
{40, 5, 800, 32, 0, gScenarioBufferData145_THRESHOLD_RANGE_08U, 64, gScenarioKernelData145_THRESHOLD_RANGE_08U, 4},
{44, 5, 880, 32, 0, gScenarioBufferData146_THRESHOLD_RANGE_08U, 64, gScenarioKernelData146_THRESHOLD_RANGE_08U, 4},
{48, 5, 960, 32, 0, gScenarioBufferData147_THRESHOLD_RANGE_08U, 64, gScenarioKernelData147_THRESHOLD_RANGE_08U, 4},
{52, 5, 1040, 32, 0, gScenarioBufferData148_THRESHOLD_RANGE_08U, 64, gScenarioKernelData148_THRESHOLD_RANGE_08U, 4},
{56, 5, 1120, 32, 0, gScenarioBufferData149_THRESHOLD_RANGE_08U, 64, gScenarioKernelData149_THRESHOLD_RANGE_08U, 4},
{60, 5, 1200, 32, 0, gScenarioBufferData150_THRESHOLD_RANGE_08U, 64, gScenarioKernelData150_THRESHOLD_RANGE_08U, 4},
{64, 5, 1280, 32, 0, gScenarioBufferData151_THRESHOLD_RANGE_08U, 64, gScenarioKernelData151_THRESHOLD_RANGE_08U, 4},
{72, 5, 1440, 32, 0, gScenarioBufferData152_THRESHOLD_RANGE_08U, 64, gScenarioKernelData152_THRESHOLD_RANGE_08U, 4},
{80, 5, 1600, 32, 0, gScenarioBufferData153_THRESHOLD_RANGE_08U, 64, gScenarioKernelData153_THRESHOLD_RANGE_08U, 4},
{88, 5, 1760, 32, 0, gScenarioBufferData154_THRESHOLD_RANGE_08U, 64, gScenarioKernelData154_THRESHOLD_RANGE_08U, 4},
{96, 5, 1920, 32, 0, gScenarioBufferData155_THRESHOLD_RANGE_08U, 64, gScenarioKernelData155_THRESHOLD_RANGE_08U, 4},
{104, 5, 2080, 32, 0, gScenarioBufferData156_THRESHOLD_RANGE_08U, 64, gScenarioKernelData156_THRESHOLD_RANGE_08U, 4},
{112, 5, 2240, 32, 0, gScenarioBufferData157_THRESHOLD_RANGE_08U, 64, gScenarioKernelData157_THRESHOLD_RANGE_08U, 4},
{120, 5, 2400, 32, 0, gScenarioBufferData158_THRESHOLD_RANGE_08U, 64, gScenarioKernelData158_THRESHOLD_RANGE_08U, 4},
{128, 5, 2560, 32, 0, gScenarioBufferData159_THRESHOLD_RANGE_08U, 64, gScenarioKernelData159_THRESHOLD_RANGE_08U, 4},
{2, 6, 48, 32, 0, gScenarioBufferData160_THRESHOLD_RANGE_08U, 64, gScenarioKernelData160_THRESHOLD_RANGE_08U, 4},
{4, 6, 96, 32, 0, gScenarioBufferData161_THRESHOLD_RANGE_08U, 64, gScenarioKernelData161_THRESHOLD_RANGE_08U, 4},
{6, 6, 144, 32, 0, gScenarioBufferData162_THRESHOLD_RANGE_08U, 64, gScenarioKernelData162_THRESHOLD_RANGE_08U, 4},
{8, 6, 192, 32, 0, gScenarioBufferData163_THRESHOLD_RANGE_08U, 64, gScenarioKernelData163_THRESHOLD_RANGE_08U, 4},
{10, 6, 240, 32, 0, gScenarioBufferData164_THRESHOLD_RANGE_08U, 64, gScenarioKernelData164_THRESHOLD_RANGE_08U, 4},
{12, 6, 288, 32, 0, gScenarioBufferData165_THRESHOLD_RANGE_08U, 64, gScenarioKernelData165_THRESHOLD_RANGE_08U, 4},
{14, 6, 336, 32, 0, gScenarioBufferData166_THRESHOLD_RANGE_08U, 64, gScenarioKernelData166_THRESHOLD_RANGE_08U, 4},
{16, 6, 384, 32, 0, gScenarioBufferData167_THRESHOLD_RANGE_08U, 64, gScenarioKernelData167_THRESHOLD_RANGE_08U, 4},
{18, 6, 432, 32, 0, gScenarioBufferData168_THRESHOLD_RANGE_08U, 64, gScenarioKernelData168_THRESHOLD_RANGE_08U, 4},
{20, 6, 480, 32, 0, gScenarioBufferData169_THRESHOLD_RANGE_08U, 64, gScenarioKernelData169_THRESHOLD_RANGE_08U, 4},
{22, 6, 528, 32, 0, gScenarioBufferData170_THRESHOLD_RANGE_08U, 64, gScenarioKernelData170_THRESHOLD_RANGE_08U, 4},
{24, 6, 576, 32, 0, gScenarioBufferData171_THRESHOLD_RANGE_08U, 64, gScenarioKernelData171_THRESHOLD_RANGE_08U, 4},
{26, 6, 624, 32, 0, gScenarioBufferData172_THRESHOLD_RANGE_08U, 64, gScenarioKernelData172_THRESHOLD_RANGE_08U, 4},
{28, 6, 672, 32, 0, gScenarioBufferData173_THRESHOLD_RANGE_08U, 64, gScenarioKernelData173_THRESHOLD_RANGE_08U, 4},
{30, 6, 720, 32, 0, gScenarioBufferData174_THRESHOLD_RANGE_08U, 64, gScenarioKernelData174_THRESHOLD_RANGE_08U, 4},
{32, 6, 768, 32, 0, gScenarioBufferData175_THRESHOLD_RANGE_08U, 64, gScenarioKernelData175_THRESHOLD_RANGE_08U, 4},
{36, 6, 864, 32, 0, gScenarioBufferData176_THRESHOLD_RANGE_08U, 64, gScenarioKernelData176_THRESHOLD_RANGE_08U, 4},
{40, 6, 960, 32, 0, gScenarioBufferData177_THRESHOLD_RANGE_08U, 64, gScenarioKernelData177_THRESHOLD_RANGE_08U, 4},
{44, 6, 1056, 32, 0, gScenarioBufferData178_THRESHOLD_RANGE_08U, 64, gScenarioKernelData178_THRESHOLD_RANGE_08U, 4},
{48, 6, 1152, 32, 0, gScenarioBufferData179_THRESHOLD_RANGE_08U, 64, gScenarioKernelData179_THRESHOLD_RANGE_08U, 4},
{52, 6, 1248, 32, 0, gScenarioBufferData180_THRESHOLD_RANGE_08U, 64, gScenarioKernelData180_THRESHOLD_RANGE_08U, 4},
{56, 6, 1344, 32, 0, gScenarioBufferData181_THRESHOLD_RANGE_08U, 64, gScenarioKernelData181_THRESHOLD_RANGE_08U, 4},
{60, 6, 1440, 32, 0, gScenarioBufferData182_THRESHOLD_RANGE_08U, 64, gScenarioKernelData182_THRESHOLD_RANGE_08U, 4},
{64, 6, 1536, 32, 0, gScenarioBufferData183_THRESHOLD_RANGE_08U, 64, gScenarioKernelData183_THRESHOLD_RANGE_08U, 4},
{72, 6, 1728, 32, 0, gScenarioBufferData184_THRESHOLD_RANGE_08U, 64, gScenarioKernelData184_THRESHOLD_RANGE_08U, 4},
{80, 6, 1920, 32, 0, gScenarioBufferData185_THRESHOLD_RANGE_08U, 64, gScenarioKernelData185_THRESHOLD_RANGE_08U, 4},
{88, 6, 2112, 32, 0, gScenarioBufferData186_THRESHOLD_RANGE_08U, 64, gScenarioKernelData186_THRESHOLD_RANGE_08U, 4},
{96, 6, 2304, 32, 0, gScenarioBufferData187_THRESHOLD_RANGE_08U, 64, gScenarioKernelData187_THRESHOLD_RANGE_08U, 4},
{104, 6, 2496, 32, 0, gScenarioBufferData188_THRESHOLD_RANGE_08U, 64, gScenarioKernelData188_THRESHOLD_RANGE_08U, 4},
{112, 6, 2688, 32, 0, gScenarioBufferData189_THRESHOLD_RANGE_08U, 64, gScenarioKernelData189_THRESHOLD_RANGE_08U, 4},
{120, 6, 2880, 32, 0, gScenarioBufferData190_THRESHOLD_RANGE_08U, 64, gScenarioKernelData190_THRESHOLD_RANGE_08U, 4},
{128, 6, 3072, 32, 0, gScenarioBufferData191_THRESHOLD_RANGE_08U, 64, gScenarioKernelData191_THRESHOLD_RANGE_08U, 4},
{2, 8, 64, 32, 0, gScenarioBufferData192_THRESHOLD_RANGE_08U, 64, gScenarioKernelData192_THRESHOLD_RANGE_08U, 4},
{4, 8, 128, 32, 0, gScenarioBufferData193_THRESHOLD_RANGE_08U, 64, gScenarioKernelData193_THRESHOLD_RANGE_08U, 4},
{6, 8, 192, 32, 0, gScenarioBufferData194_THRESHOLD_RANGE_08U, 64, gScenarioKernelData194_THRESHOLD_RANGE_08U, 4},
{8, 8, 256, 32, 0, gScenarioBufferData195_THRESHOLD_RANGE_08U, 64, gScenarioKernelData195_THRESHOLD_RANGE_08U, 4},
{10, 8, 320, 32, 0, gScenarioBufferData196_THRESHOLD_RANGE_08U, 64, gScenarioKernelData196_THRESHOLD_RANGE_08U, 4},
{12, 8, 384, 32, 0, gScenarioBufferData197_THRESHOLD_RANGE_08U, 64, gScenarioKernelData197_THRESHOLD_RANGE_08U, 4},
{14, 8, 448, 32, 0, gScenarioBufferData198_THRESHOLD_RANGE_08U, 64, gScenarioKernelData198_THRESHOLD_RANGE_08U, 4},
{16, 8, 512, 32, 0, gScenarioBufferData199_THRESHOLD_RANGE_08U, 64, gScenarioKernelData199_THRESHOLD_RANGE_08U, 4},
{18, 8, 576, 32, 0, gScenarioBufferData200_THRESHOLD_RANGE_08U, 64, gScenarioKernelData200_THRESHOLD_RANGE_08U, 4},
{20, 8, 640, 32, 0, gScenarioBufferData201_THRESHOLD_RANGE_08U, 64, gScenarioKernelData201_THRESHOLD_RANGE_08U, 4},
{22, 8, 704, 32, 0, gScenarioBufferData202_THRESHOLD_RANGE_08U, 64, gScenarioKernelData202_THRESHOLD_RANGE_08U, 4},
{24, 8, 768, 32, 0, gScenarioBufferData203_THRESHOLD_RANGE_08U, 64, gScenarioKernelData203_THRESHOLD_RANGE_08U, 4},
{26, 8, 832, 32, 0, gScenarioBufferData204_THRESHOLD_RANGE_08U, 64, gScenarioKernelData204_THRESHOLD_RANGE_08U, 4},
{28, 8, 896, 32, 0, gScenarioBufferData205_THRESHOLD_RANGE_08U, 64, gScenarioKernelData205_THRESHOLD_RANGE_08U, 4},
{30, 8, 960, 32, 0, gScenarioBufferData206_THRESHOLD_RANGE_08U, 64, gScenarioKernelData206_THRESHOLD_RANGE_08U, 4},
{32, 8, 1024, 32, 0, gScenarioBufferData207_THRESHOLD_RANGE_08U, 64, gScenarioKernelData207_THRESHOLD_RANGE_08U, 4},
{36, 8, 1152, 32, 0, gScenarioBufferData208_THRESHOLD_RANGE_08U, 64, gScenarioKernelData208_THRESHOLD_RANGE_08U, 4},
{40, 8, 1280, 32, 0, gScenarioBufferData209_THRESHOLD_RANGE_08U, 64, gScenarioKernelData209_THRESHOLD_RANGE_08U, 4},
{44, 8, 1408, 32, 0, gScenarioBufferData210_THRESHOLD_RANGE_08U, 64, gScenarioKernelData210_THRESHOLD_RANGE_08U, 4},
{48, 8, 1536, 32, 0, gScenarioBufferData211_THRESHOLD_RANGE_08U, 64, gScenarioKernelData211_THRESHOLD_RANGE_08U, 4},
{52, 8, 1664, 32, 0, gScenarioBufferData212_THRESHOLD_RANGE_08U, 64, gScenarioKernelData212_THRESHOLD_RANGE_08U, 4},
{56, 8, 1792, 32, 0, gScenarioBufferData213_THRESHOLD_RANGE_08U, 64, gScenarioKernelData213_THRESHOLD_RANGE_08U, 4},
{60, 8, 1920, 32, 0, gScenarioBufferData214_THRESHOLD_RANGE_08U, 64, gScenarioKernelData214_THRESHOLD_RANGE_08U, 4},
{64, 8, 2048, 32, 0, gScenarioBufferData215_THRESHOLD_RANGE_08U, 64, gScenarioKernelData215_THRESHOLD_RANGE_08U, 4},
{72, 8, 2304, 32, 0, gScenarioBufferData216_THRESHOLD_RANGE_08U, 64, gScenarioKernelData216_THRESHOLD_RANGE_08U, 4},
{80, 8, 2560, 32, 0, gScenarioBufferData217_THRESHOLD_RANGE_08U, 64, gScenarioKernelData217_THRESHOLD_RANGE_08U, 4},
{88, 8, 2816, 32, 0, gScenarioBufferData218_THRESHOLD_RANGE_08U, 64, gScenarioKernelData218_THRESHOLD_RANGE_08U, 4},
{96, 8, 3072, 32, 0, gScenarioBufferData219_THRESHOLD_RANGE_08U, 64, gScenarioKernelData219_THRESHOLD_RANGE_08U, 4},
{104, 8, 3328, 32, 0, gScenarioBufferData220_THRESHOLD_RANGE_08U, 64, gScenarioKernelData220_THRESHOLD_RANGE_08U, 4},
{112, 8, 3584, 32, 0, gScenarioBufferData221_THRESHOLD_RANGE_08U, 64, gScenarioKernelData221_THRESHOLD_RANGE_08U, 4},
{120, 8, 3840, 32, 0, gScenarioBufferData222_THRESHOLD_RANGE_08U, 64, gScenarioKernelData222_THRESHOLD_RANGE_08U, 4},
{2, 10, 80, 32, 0, gScenarioBufferData223_THRESHOLD_RANGE_08U, 64, gScenarioKernelData223_THRESHOLD_RANGE_08U, 4},
{4, 10, 160, 32, 0, gScenarioBufferData224_THRESHOLD_RANGE_08U, 64, gScenarioKernelData224_THRESHOLD_RANGE_08U, 4},
{6, 10, 240, 32, 0, gScenarioBufferData225_THRESHOLD_RANGE_08U, 64, gScenarioKernelData225_THRESHOLD_RANGE_08U, 4},
{8, 10, 320, 32, 0, gScenarioBufferData226_THRESHOLD_RANGE_08U, 64, gScenarioKernelData226_THRESHOLD_RANGE_08U, 4},
{10, 10, 400, 32, 0, gScenarioBufferData227_THRESHOLD_RANGE_08U, 64, gScenarioKernelData227_THRESHOLD_RANGE_08U, 4},
{12, 10, 480, 32, 0, gScenarioBufferData228_THRESHOLD_RANGE_08U, 64, gScenarioKernelData228_THRESHOLD_RANGE_08U, 4},
{14, 10, 560, 32, 0, gScenarioBufferData229_THRESHOLD_RANGE_08U, 64, gScenarioKernelData229_THRESHOLD_RANGE_08U, 4},
{16, 10, 640, 32, 0, gScenarioBufferData230_THRESHOLD_RANGE_08U, 64, gScenarioKernelData230_THRESHOLD_RANGE_08U, 4},
{18, 10, 720, 32, 0, gScenarioBufferData231_THRESHOLD_RANGE_08U, 64, gScenarioKernelData231_THRESHOLD_RANGE_08U, 4},
{20, 10, 800, 32, 0, gScenarioBufferData232_THRESHOLD_RANGE_08U, 64, gScenarioKernelData232_THRESHOLD_RANGE_08U, 4},
{22, 10, 880, 32, 0, gScenarioBufferData233_THRESHOLD_RANGE_08U, 64, gScenarioKernelData233_THRESHOLD_RANGE_08U, 4},
{24, 10, 960, 32, 0, gScenarioBufferData234_THRESHOLD_RANGE_08U, 64, gScenarioKernelData234_THRESHOLD_RANGE_08U, 4},
{26, 10, 1040, 32, 0, gScenarioBufferData235_THRESHOLD_RANGE_08U, 64, gScenarioKernelData235_THRESHOLD_RANGE_08U, 4},
{28, 10, 1120, 32, 0, gScenarioBufferData236_THRESHOLD_RANGE_08U, 64, gScenarioKernelData236_THRESHOLD_RANGE_08U, 4},
{30, 10, 1200, 32, 0, gScenarioBufferData237_THRESHOLD_RANGE_08U, 64, gScenarioKernelData237_THRESHOLD_RANGE_08U, 4},
{32, 10, 1280, 32, 0, gScenarioBufferData238_THRESHOLD_RANGE_08U, 64, gScenarioKernelData238_THRESHOLD_RANGE_08U, 4},
{36, 10, 1440, 32, 0, gScenarioBufferData239_THRESHOLD_RANGE_08U, 64, gScenarioKernelData239_THRESHOLD_RANGE_08U, 4},
{40, 10, 1600, 32, 0, gScenarioBufferData240_THRESHOLD_RANGE_08U, 64, gScenarioKernelData240_THRESHOLD_RANGE_08U, 4},
{44, 10, 1760, 32, 0, gScenarioBufferData241_THRESHOLD_RANGE_08U, 64, gScenarioKernelData241_THRESHOLD_RANGE_08U, 4},
{48, 10, 1920, 32, 0, gScenarioBufferData242_THRESHOLD_RANGE_08U, 64, gScenarioKernelData242_THRESHOLD_RANGE_08U, 4},
{52, 10, 2080, 32, 0, gScenarioBufferData243_THRESHOLD_RANGE_08U, 64, gScenarioKernelData243_THRESHOLD_RANGE_08U, 4},
{56, 10, 2240, 32, 0, gScenarioBufferData244_THRESHOLD_RANGE_08U, 64, gScenarioKernelData244_THRESHOLD_RANGE_08U, 4},
{60, 10, 2400, 32, 0, gScenarioBufferData245_THRESHOLD_RANGE_08U, 64, gScenarioKernelData245_THRESHOLD_RANGE_08U, 4},
{64, 10, 2560, 32, 0, gScenarioBufferData246_THRESHOLD_RANGE_08U, 64, gScenarioKernelData246_THRESHOLD_RANGE_08U, 4},
{72, 10, 2880, 32, 0, gScenarioBufferData247_THRESHOLD_RANGE_08U, 64, gScenarioKernelData247_THRESHOLD_RANGE_08U, 4},
{80, 10, 3200, 32, 0, gScenarioBufferData248_THRESHOLD_RANGE_08U, 64, gScenarioKernelData248_THRESHOLD_RANGE_08U, 4},
{88, 10, 3520, 32, 0, gScenarioBufferData249_THRESHOLD_RANGE_08U, 64, gScenarioKernelData249_THRESHOLD_RANGE_08U, 4},
{96, 10, 3840, 32, 0, gScenarioBufferData250_THRESHOLD_RANGE_08U, 64, gScenarioKernelData250_THRESHOLD_RANGE_08U, 4},
{2, 12, 96, 32, 0, gScenarioBufferData251_THRESHOLD_RANGE_08U, 64, gScenarioKernelData251_THRESHOLD_RANGE_08U, 4},
{4, 12, 192, 32, 0, gScenarioBufferData252_THRESHOLD_RANGE_08U, 64, gScenarioKernelData252_THRESHOLD_RANGE_08U, 4},
{6, 12, 288, 32, 0, gScenarioBufferData253_THRESHOLD_RANGE_08U, 64, gScenarioKernelData253_THRESHOLD_RANGE_08U, 4},
{8, 12, 384, 32, 0, gScenarioBufferData254_THRESHOLD_RANGE_08U, 64, gScenarioKernelData254_THRESHOLD_RANGE_08U, 4},
{10, 12, 480, 32, 0, gScenarioBufferData255_THRESHOLD_RANGE_08U, 64, gScenarioKernelData255_THRESHOLD_RANGE_08U, 4},
{12, 12, 576, 32, 0, gScenarioBufferData256_THRESHOLD_RANGE_08U, 64, gScenarioKernelData256_THRESHOLD_RANGE_08U, 4},
{14, 12, 672, 32, 0, gScenarioBufferData257_THRESHOLD_RANGE_08U, 64, gScenarioKernelData257_THRESHOLD_RANGE_08U, 4},
{16, 12, 768, 32, 0, gScenarioBufferData258_THRESHOLD_RANGE_08U, 64, gScenarioKernelData258_THRESHOLD_RANGE_08U, 4},
{18, 12, 864, 32, 0, gScenarioBufferData259_THRESHOLD_RANGE_08U, 64, gScenarioKernelData259_THRESHOLD_RANGE_08U, 4},
{20, 12, 960, 32, 0, gScenarioBufferData260_THRESHOLD_RANGE_08U, 64, gScenarioKernelData260_THRESHOLD_RANGE_08U, 4},
{22, 12, 1056, 32, 0, gScenarioBufferData261_THRESHOLD_RANGE_08U, 64, gScenarioKernelData261_THRESHOLD_RANGE_08U, 4},
{24, 12, 1152, 32, 0, gScenarioBufferData262_THRESHOLD_RANGE_08U, 64, gScenarioKernelData262_THRESHOLD_RANGE_08U, 4},
{26, 12, 1248, 32, 0, gScenarioBufferData263_THRESHOLD_RANGE_08U, 64, gScenarioKernelData263_THRESHOLD_RANGE_08U, 4},
{28, 12, 1344, 32, 0, gScenarioBufferData264_THRESHOLD_RANGE_08U, 64, gScenarioKernelData264_THRESHOLD_RANGE_08U, 4},
{30, 12, 1440, 32, 0, gScenarioBufferData265_THRESHOLD_RANGE_08U, 64, gScenarioKernelData265_THRESHOLD_RANGE_08U, 4},
{32, 12, 1536, 32, 0, gScenarioBufferData266_THRESHOLD_RANGE_08U, 64, gScenarioKernelData266_THRESHOLD_RANGE_08U, 4},
{36, 12, 1728, 32, 0, gScenarioBufferData267_THRESHOLD_RANGE_08U, 64, gScenarioKernelData267_THRESHOLD_RANGE_08U, 4},
{40, 12, 1920, 32, 0, gScenarioBufferData268_THRESHOLD_RANGE_08U, 64, gScenarioKernelData268_THRESHOLD_RANGE_08U, 4},
{44, 12, 2112, 32, 0, gScenarioBufferData269_THRESHOLD_RANGE_08U, 64, gScenarioKernelData269_THRESHOLD_RANGE_08U, 4},
{48, 12, 2304, 32, 0, gScenarioBufferData270_THRESHOLD_RANGE_08U, 64, gScenarioKernelData270_THRESHOLD_RANGE_08U, 4},
{52, 12, 2496, 32, 0, gScenarioBufferData271_THRESHOLD_RANGE_08U, 64, gScenarioKernelData271_THRESHOLD_RANGE_08U, 4},
{56, 12, 2688, 32, 0, gScenarioBufferData272_THRESHOLD_RANGE_08U, 64, gScenarioKernelData272_THRESHOLD_RANGE_08U, 4},
{60, 12, 2880, 32, 0, gScenarioBufferData273_THRESHOLD_RANGE_08U, 64, gScenarioKernelData273_THRESHOLD_RANGE_08U, 4},
{64, 12, 3072, 32, 0, gScenarioBufferData274_THRESHOLD_RANGE_08U, 64, gScenarioKernelData274_THRESHOLD_RANGE_08U, 4},
{72, 12, 3456, 32, 0, gScenarioBufferData275_THRESHOLD_RANGE_08U, 64, gScenarioKernelData275_THRESHOLD_RANGE_08U, 4},
{80, 12, 3840, 32, 0, gScenarioBufferData276_THRESHOLD_RANGE_08U, 64, gScenarioKernelData276_THRESHOLD_RANGE_08U, 4},
{2, 14, 112, 32, 0, gScenarioBufferData277_THRESHOLD_RANGE_08U, 64, gScenarioKernelData277_THRESHOLD_RANGE_08U, 4},
{4, 14, 224, 32, 0, gScenarioBufferData278_THRESHOLD_RANGE_08U, 64, gScenarioKernelData278_THRESHOLD_RANGE_08U, 4},
{6, 14, 336, 32, 0, gScenarioBufferData279_THRESHOLD_RANGE_08U, 64, gScenarioKernelData279_THRESHOLD_RANGE_08U, 4},
{8, 14, 448, 32, 0, gScenarioBufferData280_THRESHOLD_RANGE_08U, 64, gScenarioKernelData280_THRESHOLD_RANGE_08U, 4},
{10, 14, 560, 32, 0, gScenarioBufferData281_THRESHOLD_RANGE_08U, 64, gScenarioKernelData281_THRESHOLD_RANGE_08U, 4},
{12, 14, 672, 32, 0, gScenarioBufferData282_THRESHOLD_RANGE_08U, 64, gScenarioKernelData282_THRESHOLD_RANGE_08U, 4},
{14, 14, 784, 32, 0, gScenarioBufferData283_THRESHOLD_RANGE_08U, 64, gScenarioKernelData283_THRESHOLD_RANGE_08U, 4},
{16, 14, 896, 32, 0, gScenarioBufferData284_THRESHOLD_RANGE_08U, 64, gScenarioKernelData284_THRESHOLD_RANGE_08U, 4},
{18, 14, 1008, 32, 0, gScenarioBufferData285_THRESHOLD_RANGE_08U, 64, gScenarioKernelData285_THRESHOLD_RANGE_08U, 4},
{20, 14, 1120, 32, 0, gScenarioBufferData286_THRESHOLD_RANGE_08U, 64, gScenarioKernelData286_THRESHOLD_RANGE_08U, 4},
{22, 14, 1232, 32, 0, gScenarioBufferData287_THRESHOLD_RANGE_08U, 64, gScenarioKernelData287_THRESHOLD_RANGE_08U, 4},
{24, 14, 1344, 32, 0, gScenarioBufferData288_THRESHOLD_RANGE_08U, 64, gScenarioKernelData288_THRESHOLD_RANGE_08U, 4},
{26, 14, 1456, 32, 0, gScenarioBufferData289_THRESHOLD_RANGE_08U, 64, gScenarioKernelData289_THRESHOLD_RANGE_08U, 4},
{28, 14, 1568, 32, 0, gScenarioBufferData290_THRESHOLD_RANGE_08U, 64, gScenarioKernelData290_THRESHOLD_RANGE_08U, 4},
{30, 14, 1680, 32, 0, gScenarioBufferData291_THRESHOLD_RANGE_08U, 64, gScenarioKernelData291_THRESHOLD_RANGE_08U, 4},
{32, 14, 1792, 32, 0, gScenarioBufferData292_THRESHOLD_RANGE_08U, 64, gScenarioKernelData292_THRESHOLD_RANGE_08U, 4},
{36, 14, 2016, 32, 0, gScenarioBufferData293_THRESHOLD_RANGE_08U, 64, gScenarioKernelData293_THRESHOLD_RANGE_08U, 4},
{40, 14, 2240, 32, 0, gScenarioBufferData294_THRESHOLD_RANGE_08U, 64, gScenarioKernelData294_THRESHOLD_RANGE_08U, 4},
{44, 14, 2464, 32, 0, gScenarioBufferData295_THRESHOLD_RANGE_08U, 64, gScenarioKernelData295_THRESHOLD_RANGE_08U, 4},
{48, 14, 2688, 32, 0, gScenarioBufferData296_THRESHOLD_RANGE_08U, 64, gScenarioKernelData296_THRESHOLD_RANGE_08U, 4},
{52, 14, 2912, 32, 0, gScenarioBufferData297_THRESHOLD_RANGE_08U, 64, gScenarioKernelData297_THRESHOLD_RANGE_08U, 4},
{56, 14, 3136, 32, 0, gScenarioBufferData298_THRESHOLD_RANGE_08U, 64, gScenarioKernelData298_THRESHOLD_RANGE_08U, 4},
{60, 14, 3360, 32, 0, gScenarioBufferData299_THRESHOLD_RANGE_08U, 64, gScenarioKernelData299_THRESHOLD_RANGE_08U, 4},
{64, 14, 3584, 32, 0, gScenarioBufferData300_THRESHOLD_RANGE_08U, 64, gScenarioKernelData300_THRESHOLD_RANGE_08U, 4},
{2, 16, 128, 32, 0, gScenarioBufferData301_THRESHOLD_RANGE_08U, 64, gScenarioKernelData301_THRESHOLD_RANGE_08U, 4},
{4, 16, 256, 32, 0, gScenarioBufferData302_THRESHOLD_RANGE_08U, 64, gScenarioKernelData302_THRESHOLD_RANGE_08U, 4},
{6, 16, 384, 32, 0, gScenarioBufferData303_THRESHOLD_RANGE_08U, 64, gScenarioKernelData303_THRESHOLD_RANGE_08U, 4},
{8, 16, 512, 32, 0, gScenarioBufferData304_THRESHOLD_RANGE_08U, 64, gScenarioKernelData304_THRESHOLD_RANGE_08U, 4},
{10, 16, 640, 32, 0, gScenarioBufferData305_THRESHOLD_RANGE_08U, 64, gScenarioKernelData305_THRESHOLD_RANGE_08U, 4},
{12, 16, 768, 32, 0, gScenarioBufferData306_THRESHOLD_RANGE_08U, 64, gScenarioKernelData306_THRESHOLD_RANGE_08U, 4},
{14, 16, 896, 32, 0, gScenarioBufferData307_THRESHOLD_RANGE_08U, 64, gScenarioKernelData307_THRESHOLD_RANGE_08U, 4},
{16, 16, 1024, 32, 0, gScenarioBufferData308_THRESHOLD_RANGE_08U, 64, gScenarioKernelData308_THRESHOLD_RANGE_08U, 4},
{18, 16, 1152, 32, 0, gScenarioBufferData309_THRESHOLD_RANGE_08U, 64, gScenarioKernelData309_THRESHOLD_RANGE_08U, 4},
{20, 16, 1280, 32, 0, gScenarioBufferData310_THRESHOLD_RANGE_08U, 64, gScenarioKernelData310_THRESHOLD_RANGE_08U, 4},
{22, 16, 1408, 32, 0, gScenarioBufferData311_THRESHOLD_RANGE_08U, 64, gScenarioKernelData311_THRESHOLD_RANGE_08U, 4},
{24, 16, 1536, 32, 0, gScenarioBufferData312_THRESHOLD_RANGE_08U, 64, gScenarioKernelData312_THRESHOLD_RANGE_08U, 4},
{26, 16, 1664, 32, 0, gScenarioBufferData313_THRESHOLD_RANGE_08U, 64, gScenarioKernelData313_THRESHOLD_RANGE_08U, 4},
{28, 16, 1792, 32, 0, gScenarioBufferData314_THRESHOLD_RANGE_08U, 64, gScenarioKernelData314_THRESHOLD_RANGE_08U, 4},
{30, 16, 1920, 32, 0, gScenarioBufferData315_THRESHOLD_RANGE_08U, 64, gScenarioKernelData315_THRESHOLD_RANGE_08U, 4},
{32, 16, 2048, 32, 0, gScenarioBufferData316_THRESHOLD_RANGE_08U, 64, gScenarioKernelData316_THRESHOLD_RANGE_08U, 4},
{36, 16, 2304, 32, 0, gScenarioBufferData317_THRESHOLD_RANGE_08U, 64, gScenarioKernelData317_THRESHOLD_RANGE_08U, 4},
{40, 16, 2560, 32, 0, gScenarioBufferData318_THRESHOLD_RANGE_08U, 64, gScenarioKernelData318_THRESHOLD_RANGE_08U, 4},
{44, 16, 2816, 32, 0, gScenarioBufferData319_THRESHOLD_RANGE_08U, 64, gScenarioKernelData319_THRESHOLD_RANGE_08U, 4},
{48, 16, 3072, 32, 0, gScenarioBufferData320_THRESHOLD_RANGE_08U, 64, gScenarioKernelData320_THRESHOLD_RANGE_08U, 4},
{52, 16, 3328, 32, 0, gScenarioBufferData321_THRESHOLD_RANGE_08U, 64, gScenarioKernelData321_THRESHOLD_RANGE_08U, 4},
{56, 16, 3584, 32, 0, gScenarioBufferData322_THRESHOLD_RANGE_08U, 64, gScenarioKernelData322_THRESHOLD_RANGE_08U, 4},
{60, 16, 3840, 32, 0, gScenarioBufferData323_THRESHOLD_RANGE_08U, 64, gScenarioKernelData323_THRESHOLD_RANGE_08U, 4},
{2, 18, 144, 32, 0, gScenarioBufferData324_THRESHOLD_RANGE_08U, 64, gScenarioKernelData324_THRESHOLD_RANGE_08U, 4},
{4, 18, 288, 32, 0, gScenarioBufferData325_THRESHOLD_RANGE_08U, 64, gScenarioKernelData325_THRESHOLD_RANGE_08U, 4},
{6, 18, 432, 32, 0, gScenarioBufferData326_THRESHOLD_RANGE_08U, 64, gScenarioKernelData326_THRESHOLD_RANGE_08U, 4},
{8, 18, 576, 32, 0, gScenarioBufferData327_THRESHOLD_RANGE_08U, 64, gScenarioKernelData327_THRESHOLD_RANGE_08U, 4},
{10, 18, 720, 32, 0, gScenarioBufferData328_THRESHOLD_RANGE_08U, 64, gScenarioKernelData328_THRESHOLD_RANGE_08U, 4},
{12, 18, 864, 32, 0, gScenarioBufferData329_THRESHOLD_RANGE_08U, 64, gScenarioKernelData329_THRESHOLD_RANGE_08U, 4},
{14, 18, 1008, 32, 0, gScenarioBufferData330_THRESHOLD_RANGE_08U, 64, gScenarioKernelData330_THRESHOLD_RANGE_08U, 4},
{16, 18, 1152, 32, 0, gScenarioBufferData331_THRESHOLD_RANGE_08U, 64, gScenarioKernelData331_THRESHOLD_RANGE_08U, 4},
{18, 18, 1296, 32, 0, gScenarioBufferData332_THRESHOLD_RANGE_08U, 64, gScenarioKernelData332_THRESHOLD_RANGE_08U, 4},
{20, 18, 1440, 32, 0, gScenarioBufferData333_THRESHOLD_RANGE_08U, 64, gScenarioKernelData333_THRESHOLD_RANGE_08U, 4},
{22, 18, 1584, 32, 0, gScenarioBufferData334_THRESHOLD_RANGE_08U, 64, gScenarioKernelData334_THRESHOLD_RANGE_08U, 4},
{24, 18, 1728, 32, 0, gScenarioBufferData335_THRESHOLD_RANGE_08U, 64, gScenarioKernelData335_THRESHOLD_RANGE_08U, 4},
{26, 18, 1872, 32, 0, gScenarioBufferData336_THRESHOLD_RANGE_08U, 64, gScenarioKernelData336_THRESHOLD_RANGE_08U, 4},
{28, 18, 2016, 32, 0, gScenarioBufferData337_THRESHOLD_RANGE_08U, 64, gScenarioKernelData337_THRESHOLD_RANGE_08U, 4},
{30, 18, 2160, 32, 0, gScenarioBufferData338_THRESHOLD_RANGE_08U, 64, gScenarioKernelData338_THRESHOLD_RANGE_08U, 4},
{32, 18, 2304, 32, 0, gScenarioBufferData339_THRESHOLD_RANGE_08U, 64, gScenarioKernelData339_THRESHOLD_RANGE_08U, 4},
{36, 18, 2592, 32, 0, gScenarioBufferData340_THRESHOLD_RANGE_08U, 64, gScenarioKernelData340_THRESHOLD_RANGE_08U, 4},
{40, 18, 2880, 32, 0, gScenarioBufferData341_THRESHOLD_RANGE_08U, 64, gScenarioKernelData341_THRESHOLD_RANGE_08U, 4},
{44, 18, 3168, 32, 0, gScenarioBufferData342_THRESHOLD_RANGE_08U, 64, gScenarioKernelData342_THRESHOLD_RANGE_08U, 4},
{48, 18, 3456, 32, 0, gScenarioBufferData343_THRESHOLD_RANGE_08U, 64, gScenarioKernelData343_THRESHOLD_RANGE_08U, 4},
{52, 18, 3744, 32, 0, gScenarioBufferData344_THRESHOLD_RANGE_08U, 64, gScenarioKernelData344_THRESHOLD_RANGE_08U, 4},
{2, 20, 160, 32, 0, gScenarioBufferData345_THRESHOLD_RANGE_08U, 64, gScenarioKernelData345_THRESHOLD_RANGE_08U, 4},
{4, 20, 320, 32, 0, gScenarioBufferData346_THRESHOLD_RANGE_08U, 64, gScenarioKernelData346_THRESHOLD_RANGE_08U, 4},
{6, 20, 480, 32, 0, gScenarioBufferData347_THRESHOLD_RANGE_08U, 64, gScenarioKernelData347_THRESHOLD_RANGE_08U, 4},
{8, 20, 640, 32, 0, gScenarioBufferData348_THRESHOLD_RANGE_08U, 64, gScenarioKernelData348_THRESHOLD_RANGE_08U, 4},
{10, 20, 800, 32, 0, gScenarioBufferData349_THRESHOLD_RANGE_08U, 64, gScenarioKernelData349_THRESHOLD_RANGE_08U, 4},
{12, 20, 960, 32, 0, gScenarioBufferData350_THRESHOLD_RANGE_08U, 64, gScenarioKernelData350_THRESHOLD_RANGE_08U, 4},
{14, 20, 1120, 32, 0, gScenarioBufferData351_THRESHOLD_RANGE_08U, 64, gScenarioKernelData351_THRESHOLD_RANGE_08U, 4},
{16, 20, 1280, 32, 0, gScenarioBufferData352_THRESHOLD_RANGE_08U, 64, gScenarioKernelData352_THRESHOLD_RANGE_08U, 4},
{18, 20, 1440, 32, 0, gScenarioBufferData353_THRESHOLD_RANGE_08U, 64, gScenarioKernelData353_THRESHOLD_RANGE_08U, 4},
{20, 20, 1600, 32, 0, gScenarioBufferData354_THRESHOLD_RANGE_08U, 64, gScenarioKernelData354_THRESHOLD_RANGE_08U, 4},
{22, 20, 1760, 32, 0, gScenarioBufferData355_THRESHOLD_RANGE_08U, 64, gScenarioKernelData355_THRESHOLD_RANGE_08U, 4},
{24, 20, 1920, 32, 0, gScenarioBufferData356_THRESHOLD_RANGE_08U, 64, gScenarioKernelData356_THRESHOLD_RANGE_08U, 4},
{26, 20, 2080, 32, 0, gScenarioBufferData357_THRESHOLD_RANGE_08U, 64, gScenarioKernelData357_THRESHOLD_RANGE_08U, 4},
{28, 20, 2240, 32, 0, gScenarioBufferData358_THRESHOLD_RANGE_08U, 64, gScenarioKernelData358_THRESHOLD_RANGE_08U, 4},
{30, 20, 2400, 32, 0, gScenarioBufferData359_THRESHOLD_RANGE_08U, 64, gScenarioKernelData359_THRESHOLD_RANGE_08U, 4},
{32, 20, 2560, 32, 0, gScenarioBufferData360_THRESHOLD_RANGE_08U, 64, gScenarioKernelData360_THRESHOLD_RANGE_08U, 4},
{36, 20, 2880, 32, 0, gScenarioBufferData361_THRESHOLD_RANGE_08U, 64, gScenarioKernelData361_THRESHOLD_RANGE_08U, 4},
{40, 20, 3200, 32, 0, gScenarioBufferData362_THRESHOLD_RANGE_08U, 64, gScenarioKernelData362_THRESHOLD_RANGE_08U, 4},
{44, 20, 3520, 32, 0, gScenarioBufferData363_THRESHOLD_RANGE_08U, 64, gScenarioKernelData363_THRESHOLD_RANGE_08U, 4},
{48, 20, 3840, 32, 0, gScenarioBufferData364_THRESHOLD_RANGE_08U, 64, gScenarioKernelData364_THRESHOLD_RANGE_08U, 4},
{2, 22, 176, 32, 0, gScenarioBufferData365_THRESHOLD_RANGE_08U, 64, gScenarioKernelData365_THRESHOLD_RANGE_08U, 4},
{4, 22, 352, 32, 0, gScenarioBufferData366_THRESHOLD_RANGE_08U, 64, gScenarioKernelData366_THRESHOLD_RANGE_08U, 4},
{6, 22, 528, 32, 0, gScenarioBufferData367_THRESHOLD_RANGE_08U, 64, gScenarioKernelData367_THRESHOLD_RANGE_08U, 4},
{8, 22, 704, 32, 0, gScenarioBufferData368_THRESHOLD_RANGE_08U, 64, gScenarioKernelData368_THRESHOLD_RANGE_08U, 4},
{10, 22, 880, 32, 0, gScenarioBufferData369_THRESHOLD_RANGE_08U, 64, gScenarioKernelData369_THRESHOLD_RANGE_08U, 4},
{12, 22, 1056, 32, 0, gScenarioBufferData370_THRESHOLD_RANGE_08U, 64, gScenarioKernelData370_THRESHOLD_RANGE_08U, 4},
{14, 22, 1232, 32, 0, gScenarioBufferData371_THRESHOLD_RANGE_08U, 64, gScenarioKernelData371_THRESHOLD_RANGE_08U, 4},
{16, 22, 1408, 32, 0, gScenarioBufferData372_THRESHOLD_RANGE_08U, 64, gScenarioKernelData372_THRESHOLD_RANGE_08U, 4},
{18, 22, 1584, 32, 0, gScenarioBufferData373_THRESHOLD_RANGE_08U, 64, gScenarioKernelData373_THRESHOLD_RANGE_08U, 4},
{20, 22, 1760, 32, 0, gScenarioBufferData374_THRESHOLD_RANGE_08U, 64, gScenarioKernelData374_THRESHOLD_RANGE_08U, 4},
{22, 22, 1936, 32, 0, gScenarioBufferData375_THRESHOLD_RANGE_08U, 64, gScenarioKernelData375_THRESHOLD_RANGE_08U, 4},
{24, 22, 2112, 32, 0, gScenarioBufferData376_THRESHOLD_RANGE_08U, 64, gScenarioKernelData376_THRESHOLD_RANGE_08U, 4},
{26, 22, 2288, 32, 0, gScenarioBufferData377_THRESHOLD_RANGE_08U, 64, gScenarioKernelData377_THRESHOLD_RANGE_08U, 4},
{28, 22, 2464, 32, 0, gScenarioBufferData378_THRESHOLD_RANGE_08U, 64, gScenarioKernelData378_THRESHOLD_RANGE_08U, 4},
{30, 22, 2640, 32, 0, gScenarioBufferData379_THRESHOLD_RANGE_08U, 64, gScenarioKernelData379_THRESHOLD_RANGE_08U, 4},
{32, 22, 2816, 32, 0, gScenarioBufferData380_THRESHOLD_RANGE_08U, 64, gScenarioKernelData380_THRESHOLD_RANGE_08U, 4},
{36, 22, 3168, 32, 0, gScenarioBufferData381_THRESHOLD_RANGE_08U, 64, gScenarioKernelData381_THRESHOLD_RANGE_08U, 4},
{40, 22, 3520, 32, 0, gScenarioBufferData382_THRESHOLD_RANGE_08U, 64, gScenarioKernelData382_THRESHOLD_RANGE_08U, 4},
{2, 24, 192, 32, 0, gScenarioBufferData383_THRESHOLD_RANGE_08U, 64, gScenarioKernelData383_THRESHOLD_RANGE_08U, 4},
{4, 24, 384, 32, 0, gScenarioBufferData384_THRESHOLD_RANGE_08U, 64, gScenarioKernelData384_THRESHOLD_RANGE_08U, 4},
{6, 24, 576, 32, 0, gScenarioBufferData385_THRESHOLD_RANGE_08U, 64, gScenarioKernelData385_THRESHOLD_RANGE_08U, 4},
{8, 24, 768, 32, 0, gScenarioBufferData386_THRESHOLD_RANGE_08U, 64, gScenarioKernelData386_THRESHOLD_RANGE_08U, 4},
{10, 24, 960, 32, 0, gScenarioBufferData387_THRESHOLD_RANGE_08U, 64, gScenarioKernelData387_THRESHOLD_RANGE_08U, 4},
{12, 24, 1152, 32, 0, gScenarioBufferData388_THRESHOLD_RANGE_08U, 64, gScenarioKernelData388_THRESHOLD_RANGE_08U, 4},
{14, 24, 1344, 32, 0, gScenarioBufferData389_THRESHOLD_RANGE_08U, 64, gScenarioKernelData389_THRESHOLD_RANGE_08U, 4},
{16, 24, 1536, 32, 0, gScenarioBufferData390_THRESHOLD_RANGE_08U, 64, gScenarioKernelData390_THRESHOLD_RANGE_08U, 4},
{18, 24, 1728, 32, 0, gScenarioBufferData391_THRESHOLD_RANGE_08U, 64, gScenarioKernelData391_THRESHOLD_RANGE_08U, 4},
{20, 24, 1920, 32, 0, gScenarioBufferData392_THRESHOLD_RANGE_08U, 64, gScenarioKernelData392_THRESHOLD_RANGE_08U, 4},
{22, 24, 2112, 32, 0, gScenarioBufferData393_THRESHOLD_RANGE_08U, 64, gScenarioKernelData393_THRESHOLD_RANGE_08U, 4},
{24, 24, 2304, 32, 0, gScenarioBufferData394_THRESHOLD_RANGE_08U, 64, gScenarioKernelData394_THRESHOLD_RANGE_08U, 4},
{26, 24, 2496, 32, 0, gScenarioBufferData395_THRESHOLD_RANGE_08U, 64, gScenarioKernelData395_THRESHOLD_RANGE_08U, 4},
{28, 24, 2688, 32, 0, gScenarioBufferData396_THRESHOLD_RANGE_08U, 64, gScenarioKernelData396_THRESHOLD_RANGE_08U, 4},
{30, 24, 2880, 32, 0, gScenarioBufferData397_THRESHOLD_RANGE_08U, 64, gScenarioKernelData397_THRESHOLD_RANGE_08U, 4},
{32, 24, 3072, 32, 0, gScenarioBufferData398_THRESHOLD_RANGE_08U, 64, gScenarioKernelData398_THRESHOLD_RANGE_08U, 4},
{36, 24, 3456, 32, 0, gScenarioBufferData399_THRESHOLD_RANGE_08U, 64, gScenarioKernelData399_THRESHOLD_RANGE_08U, 4},
{40, 24, 3840, 32, 0, gScenarioBufferData400_THRESHOLD_RANGE_08U, 64, gScenarioKernelData400_THRESHOLD_RANGE_08U, 4},
{2, 26, 208, 32, 0, gScenarioBufferData401_THRESHOLD_RANGE_08U, 64, gScenarioKernelData401_THRESHOLD_RANGE_08U, 4},
{4, 26, 416, 32, 0, gScenarioBufferData402_THRESHOLD_RANGE_08U, 64, gScenarioKernelData402_THRESHOLD_RANGE_08U, 4},
{6, 26, 624, 32, 0, gScenarioBufferData403_THRESHOLD_RANGE_08U, 64, gScenarioKernelData403_THRESHOLD_RANGE_08U, 4},
{8, 26, 832, 32, 0, gScenarioBufferData404_THRESHOLD_RANGE_08U, 64, gScenarioKernelData404_THRESHOLD_RANGE_08U, 4},
{10, 26, 1040, 32, 0, gScenarioBufferData405_THRESHOLD_RANGE_08U, 64, gScenarioKernelData405_THRESHOLD_RANGE_08U, 4},
{12, 26, 1248, 32, 0, gScenarioBufferData406_THRESHOLD_RANGE_08U, 64, gScenarioKernelData406_THRESHOLD_RANGE_08U, 4},
{14, 26, 1456, 32, 0, gScenarioBufferData407_THRESHOLD_RANGE_08U, 64, gScenarioKernelData407_THRESHOLD_RANGE_08U, 4},
{16, 26, 1664, 32, 0, gScenarioBufferData408_THRESHOLD_RANGE_08U, 64, gScenarioKernelData408_THRESHOLD_RANGE_08U, 4},
{18, 26, 1872, 32, 0, gScenarioBufferData409_THRESHOLD_RANGE_08U, 64, gScenarioKernelData409_THRESHOLD_RANGE_08U, 4},
{20, 26, 2080, 32, 0, gScenarioBufferData410_THRESHOLD_RANGE_08U, 64, gScenarioKernelData410_THRESHOLD_RANGE_08U, 4},
{22, 26, 2288, 32, 0, gScenarioBufferData411_THRESHOLD_RANGE_08U, 64, gScenarioKernelData411_THRESHOLD_RANGE_08U, 4},
{24, 26, 2496, 32, 0, gScenarioBufferData412_THRESHOLD_RANGE_08U, 64, gScenarioKernelData412_THRESHOLD_RANGE_08U, 4},
{26, 26, 2704, 32, 0, gScenarioBufferData413_THRESHOLD_RANGE_08U, 64, gScenarioKernelData413_THRESHOLD_RANGE_08U, 4},
{28, 26, 2912, 32, 0, gScenarioBufferData414_THRESHOLD_RANGE_08U, 64, gScenarioKernelData414_THRESHOLD_RANGE_08U, 4},
{30, 26, 3120, 32, 0, gScenarioBufferData415_THRESHOLD_RANGE_08U, 64, gScenarioKernelData415_THRESHOLD_RANGE_08U, 4},
{32, 26, 3328, 32, 0, gScenarioBufferData416_THRESHOLD_RANGE_08U, 64, gScenarioKernelData416_THRESHOLD_RANGE_08U, 4},
{36, 26, 3744, 32, 0, gScenarioBufferData417_THRESHOLD_RANGE_08U, 64, gScenarioKernelData417_THRESHOLD_RANGE_08U, 4},
{2, 28, 224, 32, 0, gScenarioBufferData418_THRESHOLD_RANGE_08U, 64, gScenarioKernelData418_THRESHOLD_RANGE_08U, 4},
{4, 28, 448, 32, 0, gScenarioBufferData419_THRESHOLD_RANGE_08U, 64, gScenarioKernelData419_THRESHOLD_RANGE_08U, 4},
{6, 28, 672, 32, 0, gScenarioBufferData420_THRESHOLD_RANGE_08U, 64, gScenarioKernelData420_THRESHOLD_RANGE_08U, 4},
{8, 28, 896, 32, 0, gScenarioBufferData421_THRESHOLD_RANGE_08U, 64, gScenarioKernelData421_THRESHOLD_RANGE_08U, 4},
{10, 28, 1120, 32, 0, gScenarioBufferData422_THRESHOLD_RANGE_08U, 64, gScenarioKernelData422_THRESHOLD_RANGE_08U, 4},
{12, 28, 1344, 32, 0, gScenarioBufferData423_THRESHOLD_RANGE_08U, 64, gScenarioKernelData423_THRESHOLD_RANGE_08U, 4},
{14, 28, 1568, 32, 0, gScenarioBufferData424_THRESHOLD_RANGE_08U, 64, gScenarioKernelData424_THRESHOLD_RANGE_08U, 4},
{16, 28, 1792, 32, 0, gScenarioBufferData425_THRESHOLD_RANGE_08U, 64, gScenarioKernelData425_THRESHOLD_RANGE_08U, 4},
{18, 28, 2016, 32, 0, gScenarioBufferData426_THRESHOLD_RANGE_08U, 64, gScenarioKernelData426_THRESHOLD_RANGE_08U, 4},
{20, 28, 2240, 32, 0, gScenarioBufferData427_THRESHOLD_RANGE_08U, 64, gScenarioKernelData427_THRESHOLD_RANGE_08U, 4},
{22, 28, 2464, 32, 0, gScenarioBufferData428_THRESHOLD_RANGE_08U, 64, gScenarioKernelData428_THRESHOLD_RANGE_08U, 4},
{24, 28, 2688, 32, 0, gScenarioBufferData429_THRESHOLD_RANGE_08U, 64, gScenarioKernelData429_THRESHOLD_RANGE_08U, 4},
{26, 28, 2912, 32, 0, gScenarioBufferData430_THRESHOLD_RANGE_08U, 64, gScenarioKernelData430_THRESHOLD_RANGE_08U, 4},
{28, 28, 3136, 32, 0, gScenarioBufferData431_THRESHOLD_RANGE_08U, 64, gScenarioKernelData431_THRESHOLD_RANGE_08U, 4},
{30, 28, 3360, 32, 0, gScenarioBufferData432_THRESHOLD_RANGE_08U, 64, gScenarioKernelData432_THRESHOLD_RANGE_08U, 4},
{32, 28, 3584, 32, 0, gScenarioBufferData433_THRESHOLD_RANGE_08U, 64, gScenarioKernelData433_THRESHOLD_RANGE_08U, 4},
{2, 30, 240, 32, 0, gScenarioBufferData434_THRESHOLD_RANGE_08U, 64, gScenarioKernelData434_THRESHOLD_RANGE_08U, 4},
{4, 30, 480, 32, 0, gScenarioBufferData435_THRESHOLD_RANGE_08U, 64, gScenarioKernelData435_THRESHOLD_RANGE_08U, 4},
{6, 30, 720, 32, 0, gScenarioBufferData436_THRESHOLD_RANGE_08U, 64, gScenarioKernelData436_THRESHOLD_RANGE_08U, 4},
{8, 30, 960, 32, 0, gScenarioBufferData437_THRESHOLD_RANGE_08U, 64, gScenarioKernelData437_THRESHOLD_RANGE_08U, 4},
{10, 30, 1200, 32, 0, gScenarioBufferData438_THRESHOLD_RANGE_08U, 64, gScenarioKernelData438_THRESHOLD_RANGE_08U, 4},
{12, 30, 1440, 32, 0, gScenarioBufferData439_THRESHOLD_RANGE_08U, 64, gScenarioKernelData439_THRESHOLD_RANGE_08U, 4},
{14, 30, 1680, 32, 0, gScenarioBufferData440_THRESHOLD_RANGE_08U, 64, gScenarioKernelData440_THRESHOLD_RANGE_08U, 4},
{16, 30, 1920, 32, 0, gScenarioBufferData441_THRESHOLD_RANGE_08U, 64, gScenarioKernelData441_THRESHOLD_RANGE_08U, 4},
{18, 30, 2160, 32, 0, gScenarioBufferData442_THRESHOLD_RANGE_08U, 64, gScenarioKernelData442_THRESHOLD_RANGE_08U, 4},
{20, 30, 2400, 32, 0, gScenarioBufferData443_THRESHOLD_RANGE_08U, 64, gScenarioKernelData443_THRESHOLD_RANGE_08U, 4},
{22, 30, 2640, 32, 0, gScenarioBufferData444_THRESHOLD_RANGE_08U, 64, gScenarioKernelData444_THRESHOLD_RANGE_08U, 4},
{24, 30, 2880, 32, 0, gScenarioBufferData445_THRESHOLD_RANGE_08U, 64, gScenarioKernelData445_THRESHOLD_RANGE_08U, 4},
{26, 30, 3120, 32, 0, gScenarioBufferData446_THRESHOLD_RANGE_08U, 64, gScenarioKernelData446_THRESHOLD_RANGE_08U, 4},
{28, 30, 3360, 32, 0, gScenarioBufferData447_THRESHOLD_RANGE_08U, 64, gScenarioKernelData447_THRESHOLD_RANGE_08U, 4},
{30, 30, 3600, 32, 0, gScenarioBufferData448_THRESHOLD_RANGE_08U, 64, gScenarioKernelData448_THRESHOLD_RANGE_08U, 4},
{32, 30, 3840, 32, 0, gScenarioBufferData449_THRESHOLD_RANGE_08U, 64, gScenarioKernelData449_THRESHOLD_RANGE_08U, 4},
{2, 32, 256, 32, 0, gScenarioBufferData450_THRESHOLD_RANGE_08U, 64, gScenarioKernelData450_THRESHOLD_RANGE_08U, 4},
{4, 32, 512, 32, 0, gScenarioBufferData451_THRESHOLD_RANGE_08U, 64, gScenarioKernelData451_THRESHOLD_RANGE_08U, 4},
{6, 32, 768, 32, 0, gScenarioBufferData452_THRESHOLD_RANGE_08U, 64, gScenarioKernelData452_THRESHOLD_RANGE_08U, 4},
{8, 32, 1024, 32, 0, gScenarioBufferData453_THRESHOLD_RANGE_08U, 64, gScenarioKernelData453_THRESHOLD_RANGE_08U, 4},
{10, 32, 1280, 32, 0, gScenarioBufferData454_THRESHOLD_RANGE_08U, 64, gScenarioKernelData454_THRESHOLD_RANGE_08U, 4},
{12, 32, 1536, 32, 0, gScenarioBufferData455_THRESHOLD_RANGE_08U, 64, gScenarioKernelData455_THRESHOLD_RANGE_08U, 4},
{14, 32, 1792, 32, 0, gScenarioBufferData456_THRESHOLD_RANGE_08U, 64, gScenarioKernelData456_THRESHOLD_RANGE_08U, 4},
{16, 32, 2048, 32, 0, gScenarioBufferData457_THRESHOLD_RANGE_08U, 64, gScenarioKernelData457_THRESHOLD_RANGE_08U, 4},
{18, 32, 2304, 32, 0, gScenarioBufferData458_THRESHOLD_RANGE_08U, 64, gScenarioKernelData458_THRESHOLD_RANGE_08U, 4},
{20, 32, 2560, 32, 0, gScenarioBufferData459_THRESHOLD_RANGE_08U, 64, gScenarioKernelData459_THRESHOLD_RANGE_08U, 4},
{22, 32, 2816, 32, 0, gScenarioBufferData460_THRESHOLD_RANGE_08U, 64, gScenarioKernelData460_THRESHOLD_RANGE_08U, 4},
{24, 32, 3072, 32, 0, gScenarioBufferData461_THRESHOLD_RANGE_08U, 64, gScenarioKernelData461_THRESHOLD_RANGE_08U, 4},
{26, 32, 3328, 32, 0, gScenarioBufferData462_THRESHOLD_RANGE_08U, 64, gScenarioKernelData462_THRESHOLD_RANGE_08U, 4},
{28, 32, 3584, 32, 0, gScenarioBufferData463_THRESHOLD_RANGE_08U, 64, gScenarioKernelData463_THRESHOLD_RANGE_08U, 4},
{30, 32, 3840, 32, 0, gScenarioBufferData464_THRESHOLD_RANGE_08U, 64, gScenarioKernelData464_THRESHOLD_RANGE_08U, 4}
};

static acf_scenario_list gScenarioList_THRESHOLD_RANGE_08U = {
465, //number of scenarios
gScenarioArray_THRESHOLD_RANGE_08U};
//**************************************************************

class THRESHOLD_RANGE_08U : public ACF_Process_APU
{

public:
   THRESHOLD_RANGE_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("THRESHOLD_RANGE_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("THRESHOLD_RANGE_08U",
                                        THRESHOLD_RANGE_08U_LOAD_SEGMENTS,
                                        THRESHOLD_RANGE_08U_LOAD_PMEM, THRESHOLD_RANGE_08U_LOAD_PMEM_SIZE,
                                        THRESHOLD_RANGE_08U_LOAD_DMEM, THRESHOLD_RANGE_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(THRESHOLD_RANGE_08U_APEX_LOG_BUFFER, THRESHOLD_RANGE_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_THRESHOLD_RANGE_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_THRESHOLD_RANGE_08U

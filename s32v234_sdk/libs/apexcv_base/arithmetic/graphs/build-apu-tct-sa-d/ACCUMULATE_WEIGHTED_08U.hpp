#ifndef _ACF_PROCESS_APU_ACCUMULATE_WEIGHTED_08U
#define _ACF_PROCESS_APU_ACCUMULATE_WEIGHTED_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <ACCUMULATE_WEIGHTED_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_ACCUMULATE_WEIGHTED_08U[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_ACCUMULATE_WEIGHTED_08U[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_ACCUMULATE_WEIGHTED_08U[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_ACCUMULATE_WEIGHTED_08U[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_ACCUMULATE_WEIGHTED_08U[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_ACCUMULATE_WEIGHTED_08U[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_ACCUMULATE_WEIGHTED_08U[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_ACCUMULATE_WEIGHTED_08U[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_ACCUMULATE_WEIGHTED_08U[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_ACCUMULATE_WEIGHTED_08U[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_ACCUMULATE_WEIGHTED_08U[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_ACCUMULATE_WEIGHTED_08U[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_ACCUMULATE_WEIGHTED_08U[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_ACCUMULATE_WEIGHTED_08U[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_ACCUMULATE_WEIGHTED_08U[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_ACCUMULATE_WEIGHTED_08U[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_ACCUMULATE_WEIGHTED_08U[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_ACCUMULATE_WEIGHTED_08U[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_ACCUMULATE_WEIGHTED_08U[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_ACCUMULATE_WEIGHTED_08U[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_ACCUMULATE_WEIGHTED_08U[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_ACCUMULATE_WEIGHTED_08U[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_ACCUMULATE_WEIGHTED_08U[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_ACCUMULATE_WEIGHTED_08U[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_ACCUMULATE_WEIGHTED_08U[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_ACCUMULATE_WEIGHTED_08U[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_ACCUMULATE_WEIGHTED_08U[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_ACCUMULATE_WEIGHTED_08U[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_ACCUMULATE_WEIGHTED_08U[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_ACCUMULATE_WEIGHTED_08U[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_ACCUMULATE_WEIGHTED_08U[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_ACCUMULATE_WEIGHTED_08U[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_ACCUMULATE_WEIGHTED_08U[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_ACCUMULATE_WEIGHTED_08U[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_ACCUMULATE_WEIGHTED_08U[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_ACCUMULATE_WEIGHTED_08U[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_ACCUMULATE_WEIGHTED_08U[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_ACCUMULATE_WEIGHTED_08U[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_ACCUMULATE_WEIGHTED_08U[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_ACCUMULATE_WEIGHTED_08U[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_ACCUMULATE_WEIGHTED_08U[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_ACCUMULATE_WEIGHTED_08U[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_ACCUMULATE_WEIGHTED_08U[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_ACCUMULATE_WEIGHTED_08U[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_ACCUMULATE_WEIGHTED_08U[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_ACCUMULATE_WEIGHTED_08U[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_ACCUMULATE_WEIGHTED_08U[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_ACCUMULATE_WEIGHTED_08U[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_ACCUMULATE_WEIGHTED_08U[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_ACCUMULATE_WEIGHTED_08U[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_ACCUMULATE_WEIGHTED_08U[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_ACCUMULATE_WEIGHTED_08U[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_ACCUMULATE_WEIGHTED_08U[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_ACCUMULATE_WEIGHTED_08U[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_ACCUMULATE_WEIGHTED_08U[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_ACCUMULATE_WEIGHTED_08U[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_ACCUMULATE_WEIGHTED_08U[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_ACCUMULATE_WEIGHTED_08U[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_ACCUMULATE_WEIGHTED_08U[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_ACCUMULATE_WEIGHTED_08U[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_ACCUMULATE_WEIGHTED_08U[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_ACCUMULATE_WEIGHTED_08U[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_ACCUMULATE_WEIGHTED_08U[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_ACCUMULATE_WEIGHTED_08U[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_ACCUMULATE_WEIGHTED_08U[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_ACCUMULATE_WEIGHTED_08U[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_ACCUMULATE_WEIGHTED_08U[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_ACCUMULATE_WEIGHTED_08U[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_ACCUMULATE_WEIGHTED_08U[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_ACCUMULATE_WEIGHTED_08U[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_ACCUMULATE_WEIGHTED_08U[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_ACCUMULATE_WEIGHTED_08U[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_ACCUMULATE_WEIGHTED_08U[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_ACCUMULATE_WEIGHTED_08U[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_ACCUMULATE_WEIGHTED_08U[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_ACCUMULATE_WEIGHTED_08U[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_ACCUMULATE_WEIGHTED_08U[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_ACCUMULATE_WEIGHTED_08U[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_ACCUMULATE_WEIGHTED_08U[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_ACCUMULATE_WEIGHTED_08U[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_ACCUMULATE_WEIGHTED_08U[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_ACCUMULATE_WEIGHTED_08U[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_ACCUMULATE_WEIGHTED_08U[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_ACCUMULATE_WEIGHTED_08U[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_ACCUMULATE_WEIGHTED_08U[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_ACCUMULATE_WEIGHTED_08U[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_ACCUMULATE_WEIGHTED_08U[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_ACCUMULATE_WEIGHTED_08U[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_ACCUMULATE_WEIGHTED_08U[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_ACCUMULATE_WEIGHTED_08U[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_ACCUMULATE_WEIGHTED_08U[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_ACCUMULATE_WEIGHTED_08U[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_ACCUMULATE_WEIGHTED_08U[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_ACCUMULATE_WEIGHTED_08U[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_ACCUMULATE_WEIGHTED_08U[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_ACCUMULATE_WEIGHTED_08U[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_ACCUMULATE_WEIGHTED_08U[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_ACCUMULATE_WEIGHTED_08U[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_ACCUMULATE_WEIGHTED_08U[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_ACCUMULATE_WEIGHTED_08U[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_ACCUMULATE_WEIGHTED_08U[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_ACCUMULATE_WEIGHTED_08U[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_ACCUMULATE_WEIGHTED_08U[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_ACCUMULATE_WEIGHTED_08U[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_ACCUMULATE_WEIGHTED_08U[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_ACCUMULATE_WEIGHTED_08U[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_ACCUMULATE_WEIGHTED_08U[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_ACCUMULATE_WEIGHTED_08U[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_ACCUMULATE_WEIGHTED_08U[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_ACCUMULATE_WEIGHTED_08U[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_ACCUMULATE_WEIGHTED_08U[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_ACCUMULATE_WEIGHTED_08U[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_ACCUMULATE_WEIGHTED_08U[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_ACCUMULATE_WEIGHTED_08U[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_ACCUMULATE_WEIGHTED_08U[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_ACCUMULATE_WEIGHTED_08U[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_ACCUMULATE_WEIGHTED_08U[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_ACCUMULATE_WEIGHTED_08U[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_ACCUMULATE_WEIGHTED_08U[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_ACCUMULATE_WEIGHTED_08U[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_ACCUMULATE_WEIGHTED_08U[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_ACCUMULATE_WEIGHTED_08U[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_ACCUMULATE_WEIGHTED_08U[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_ACCUMULATE_WEIGHTED_08U[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_ACCUMULATE_WEIGHTED_08U[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_ACCUMULATE_WEIGHTED_08U[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_ACCUMULATE_WEIGHTED_08U[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_ACCUMULATE_WEIGHTED_08U[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_ACCUMULATE_WEIGHTED_08U[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_ACCUMULATE_WEIGHTED_08U[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_ACCUMULATE_WEIGHTED_08U[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_ACCUMULATE_WEIGHTED_08U[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_ACCUMULATE_WEIGHTED_08U[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_ACCUMULATE_WEIGHTED_08U[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_ACCUMULATE_WEIGHTED_08U[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_ACCUMULATE_WEIGHTED_08U[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_ACCUMULATE_WEIGHTED_08U[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_ACCUMULATE_WEIGHTED_08U[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_ACCUMULATE_WEIGHTED_08U[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_ACCUMULATE_WEIGHTED_08U[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_ACCUMULATE_WEIGHTED_08U[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_ACCUMULATE_WEIGHTED_08U[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_ACCUMULATE_WEIGHTED_08U[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_ACCUMULATE_WEIGHTED_08U[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_ACCUMULATE_WEIGHTED_08U[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_ACCUMULATE_WEIGHTED_08U[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_ACCUMULATE_WEIGHTED_08U[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_ACCUMULATE_WEIGHTED_08U[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_ACCUMULATE_WEIGHTED_08U[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_ACCUMULATE_WEIGHTED_08U[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_ACCUMULATE_WEIGHTED_08U[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_ACCUMULATE_WEIGHTED_08U[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_ACCUMULATE_WEIGHTED_08U[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_ACCUMULATE_WEIGHTED_08U[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_ACCUMULATE_WEIGHTED_08U[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_ACCUMULATE_WEIGHTED_08U[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_ACCUMULATE_WEIGHTED_08U[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_ACCUMULATE_WEIGHTED_08U[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_ACCUMULATE_WEIGHTED_08U[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_ACCUMULATE_WEIGHTED_08U[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_ACCUMULATE_WEIGHTED_08U[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_ACCUMULATE_WEIGHTED_08U[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_ACCUMULATE_WEIGHTED_08U[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_ACCUMULATE_WEIGHTED_08U[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_ACCUMULATE_WEIGHTED_08U[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_ACCUMULATE_WEIGHTED_08U[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_ACCUMULATE_WEIGHTED_08U[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_ACCUMULATE_WEIGHTED_08U[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_ACCUMULATE_WEIGHTED_08U[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_ACCUMULATE_WEIGHTED_08U[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_ACCUMULATE_WEIGHTED_08U[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_ACCUMULATE_WEIGHTED_08U[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_ACCUMULATE_WEIGHTED_08U[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_ACCUMULATE_WEIGHTED_08U[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_ACCUMULATE_WEIGHTED_08U[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_ACCUMULATE_WEIGHTED_08U[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_ACCUMULATE_WEIGHTED_08U[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_ACCUMULATE_WEIGHTED_08U[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_ACCUMULATE_WEIGHTED_08U[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_ACCUMULATE_WEIGHTED_08U[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_ACCUMULATE_WEIGHTED_08U[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_ACCUMULATE_WEIGHTED_08U[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_ACCUMULATE_WEIGHTED_08U[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_ACCUMULATE_WEIGHTED_08U[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_ACCUMULATE_WEIGHTED_08U[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_ACCUMULATE_WEIGHTED_08U[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_ACCUMULATE_WEIGHTED_08U[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_ACCUMULATE_WEIGHTED_08U[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_ACCUMULATE_WEIGHTED_08U[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_ACCUMULATE_WEIGHTED_08U[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_ACCUMULATE_WEIGHTED_08U[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_ACCUMULATE_WEIGHTED_08U[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_ACCUMULATE_WEIGHTED_08U[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_ACCUMULATE_WEIGHTED_08U[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_ACCUMULATE_WEIGHTED_08U[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_ACCUMULATE_WEIGHTED_08U[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_ACCUMULATE_WEIGHTED_08U[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_ACCUMULATE_WEIGHTED_08U[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_ACCUMULATE_WEIGHTED_08U[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_ACCUMULATE_WEIGHTED_08U[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_ACCUMULATE_WEIGHTED_08U[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_ACCUMULATE_WEIGHTED_08U[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_ACCUMULATE_WEIGHTED_08U[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_ACCUMULATE_WEIGHTED_08U[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_ACCUMULATE_WEIGHTED_08U[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_ACCUMULATE_WEIGHTED_08U[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_ACCUMULATE_WEIGHTED_08U[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_ACCUMULATE_WEIGHTED_08U[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_ACCUMULATE_WEIGHTED_08U[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_ACCUMULATE_WEIGHTED_08U[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_ACCUMULATE_WEIGHTED_08U[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_ACCUMULATE_WEIGHTED_08U[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_ACCUMULATE_WEIGHTED_08U[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_ACCUMULATE_WEIGHTED_08U[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_ACCUMULATE_WEIGHTED_08U[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_ACCUMULATE_WEIGHTED_08U[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_ACCUMULATE_WEIGHTED_08U[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_ACCUMULATE_WEIGHTED_08U[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_ACCUMULATE_WEIGHTED_08U[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_ACCUMULATE_WEIGHTED_08U[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_ACCUMULATE_WEIGHTED_08U[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_ACCUMULATE_WEIGHTED_08U[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_ACCUMULATE_WEIGHTED_08U[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_ACCUMULATE_WEIGHTED_08U[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_ACCUMULATE_WEIGHTED_08U[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_ACCUMULATE_WEIGHTED_08U[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_ACCUMULATE_WEIGHTED_08U[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_ACCUMULATE_WEIGHTED_08U[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_ACCUMULATE_WEIGHTED_08U[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_ACCUMULATE_WEIGHTED_08U[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_ACCUMULATE_WEIGHTED_08U[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_ACCUMULATE_WEIGHTED_08U[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_ACCUMULATE_WEIGHTED_08U[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_ACCUMULATE_WEIGHTED_08U[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_ACCUMULATE_WEIGHTED_08U[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_ACCUMULATE_WEIGHTED_08U[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_ACCUMULATE_WEIGHTED_08U[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_ACCUMULATE_WEIGHTED_08U[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_ACCUMULATE_WEIGHTED_08U[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_ACCUMULATE_WEIGHTED_08U[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_ACCUMULATE_WEIGHTED_08U[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_ACCUMULATE_WEIGHTED_08U[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_ACCUMULATE_WEIGHTED_08U[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_ACCUMULATE_WEIGHTED_08U[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_ACCUMULATE_WEIGHTED_08U[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_ACCUMULATE_WEIGHTED_08U[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_ACCUMULATE_WEIGHTED_08U[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_ACCUMULATE_WEIGHTED_08U[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_ACCUMULATE_WEIGHTED_08U[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_ACCUMULATE_WEIGHTED_08U[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_ACCUMULATE_WEIGHTED_08U[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_ACCUMULATE_WEIGHTED_08U[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_ACCUMULATE_WEIGHTED_08U[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_ACCUMULATE_WEIGHTED_08U[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_ACCUMULATE_WEIGHTED_08U[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_ACCUMULATE_WEIGHTED_08U[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_ACCUMULATE_WEIGHTED_08U[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_ACCUMULATE_WEIGHTED_08U[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_ACCUMULATE_WEIGHTED_08U[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_ACCUMULATE_WEIGHTED_08U[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_ACCUMULATE_WEIGHTED_08U[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_ACCUMULATE_WEIGHTED_08U[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_ACCUMULATE_WEIGHTED_08U[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_ACCUMULATE_WEIGHTED_08U[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_ACCUMULATE_WEIGHTED_08U[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_ACCUMULATE_WEIGHTED_08U[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_ACCUMULATE_WEIGHTED_08U[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_ACCUMULATE_WEIGHTED_08U[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_ACCUMULATE_WEIGHTED_08U[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_ACCUMULATE_WEIGHTED_08U[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_ACCUMULATE_WEIGHTED_08U[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_ACCUMULATE_WEIGHTED_08U[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_ACCUMULATE_WEIGHTED_08U[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_ACCUMULATE_WEIGHTED_08U[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_ACCUMULATE_WEIGHTED_08U[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_ACCUMULATE_WEIGHTED_08U[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_ACCUMULATE_WEIGHTED_08U[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_ACCUMULATE_WEIGHTED_08U[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_ACCUMULATE_WEIGHTED_08U[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_ACCUMULATE_WEIGHTED_08U[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_ACCUMULATE_WEIGHTED_08U[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_ACCUMULATE_WEIGHTED_08U[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_ACCUMULATE_WEIGHTED_08U[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_ACCUMULATE_WEIGHTED_08U[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_ACCUMULATE_WEIGHTED_08U[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_ACCUMULATE_WEIGHTED_08U[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_ACCUMULATE_WEIGHTED_08U[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_ACCUMULATE_WEIGHTED_08U[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_ACCUMULATE_WEIGHTED_08U[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_ACCUMULATE_WEIGHTED_08U[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_ACCUMULATE_WEIGHTED_08U[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_ACCUMULATE_WEIGHTED_08U[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_ACCUMULATE_WEIGHTED_08U[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_ACCUMULATE_WEIGHTED_08U[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_ACCUMULATE_WEIGHTED_08U[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_ACCUMULATE_WEIGHTED_08U[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_ACCUMULATE_WEIGHTED_08U[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_ACCUMULATE_WEIGHTED_08U[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_ACCUMULATE_WEIGHTED_08U[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_ACCUMULATE_WEIGHTED_08U[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_ACCUMULATE_WEIGHTED_08U[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_ACCUMULATE_WEIGHTED_08U[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_ACCUMULATE_WEIGHTED_08U[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_ACCUMULATE_WEIGHTED_08U[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_ACCUMULATE_WEIGHTED_08U[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_ACCUMULATE_WEIGHTED_08U[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_ACCUMULATE_WEIGHTED_08U[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_ACCUMULATE_WEIGHTED_08U[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_ACCUMULATE_WEIGHTED_08U[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_ACCUMULATE_WEIGHTED_08U[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_ACCUMULATE_WEIGHTED_08U[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_ACCUMULATE_WEIGHTED_08U[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_ACCUMULATE_WEIGHTED_08U[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_ACCUMULATE_WEIGHTED_08U[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_ACCUMULATE_WEIGHTED_08U[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_ACCUMULATE_WEIGHTED_08U[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_ACCUMULATE_WEIGHTED_08U[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_ACCUMULATE_WEIGHTED_08U[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_ACCUMULATE_WEIGHTED_08U[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_ACCUMULATE_WEIGHTED_08U[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_ACCUMULATE_WEIGHTED_08U[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_ACCUMULATE_WEIGHTED_08U[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_ACCUMULATE_WEIGHTED_08U[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_ACCUMULATE_WEIGHTED_08U[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_ACCUMULATE_WEIGHTED_08U[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_ACCUMULATE_WEIGHTED_08U[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_ACCUMULATE_WEIGHTED_08U[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_ACCUMULATE_WEIGHTED_08U[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_ACCUMULATE_WEIGHTED_08U[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_ACCUMULATE_WEIGHTED_08U[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_ACCUMULATE_WEIGHTED_08U[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_ACCUMULATE_WEIGHTED_08U[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_ACCUMULATE_WEIGHTED_08U[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_ACCUMULATE_WEIGHTED_08U[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_ACCUMULATE_WEIGHTED_08U[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_ACCUMULATE_WEIGHTED_08U[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_ACCUMULATE_WEIGHTED_08U[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_ACCUMULATE_WEIGHTED_08U[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_ACCUMULATE_WEIGHTED_08U[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_ACCUMULATE_WEIGHTED_08U[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_ACCUMULATE_WEIGHTED_08U[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_ACCUMULATE_WEIGHTED_08U[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_ACCUMULATE_WEIGHTED_08U[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_ACCUMULATE_WEIGHTED_08U[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_ACCUMULATE_WEIGHTED_08U[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_ACCUMULATE_WEIGHTED_08U[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_ACCUMULATE_WEIGHTED_08U[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_ACCUMULATE_WEIGHTED_08U[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_ACCUMULATE_WEIGHTED_08U[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_ACCUMULATE_WEIGHTED_08U[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_ACCUMULATE_WEIGHTED_08U[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_ACCUMULATE_WEIGHTED_08U[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_ACCUMULATE_WEIGHTED_08U[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_ACCUMULATE_WEIGHTED_08U[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_ACCUMULATE_WEIGHTED_08U[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_ACCUMULATE_WEIGHTED_08U[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_ACCUMULATE_WEIGHTED_08U[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_ACCUMULATE_WEIGHTED_08U[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_ACCUMULATE_WEIGHTED_08U[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_ACCUMULATE_WEIGHTED_08U[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_ACCUMULATE_WEIGHTED_08U[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_ACCUMULATE_WEIGHTED_08U[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_ACCUMULATE_WEIGHTED_08U[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_ACCUMULATE_WEIGHTED_08U[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_ACCUMULATE_WEIGHTED_08U[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_ACCUMULATE_WEIGHTED_08U[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_ACCUMULATE_WEIGHTED_08U[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_ACCUMULATE_WEIGHTED_08U[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_ACCUMULATE_WEIGHTED_08U[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_ACCUMULATE_WEIGHTED_08U[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_ACCUMULATE_WEIGHTED_08U[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_ACCUMULATE_WEIGHTED_08U[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_ACCUMULATE_WEIGHTED_08U[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_ACCUMULATE_WEIGHTED_08U[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_ACCUMULATE_WEIGHTED_08U[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_ACCUMULATE_WEIGHTED_08U[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_ACCUMULATE_WEIGHTED_08U[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_ACCUMULATE_WEIGHTED_08U[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_ACCUMULATE_WEIGHTED_08U[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_ACCUMULATE_WEIGHTED_08U[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_ACCUMULATE_WEIGHTED_08U[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_ACCUMULATE_WEIGHTED_08U[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_ACCUMULATE_WEIGHTED_08U[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_ACCUMULATE_WEIGHTED_08U[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_ACCUMULATE_WEIGHTED_08U[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_ACCUMULATE_WEIGHTED_08U[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_ACCUMULATE_WEIGHTED_08U[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_ACCUMULATE_WEIGHTED_08U[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_ACCUMULATE_WEIGHTED_08U[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_ACCUMULATE_WEIGHTED_08U[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_ACCUMULATE_WEIGHTED_08U[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_ACCUMULATE_WEIGHTED_08U[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_ACCUMULATE_WEIGHTED_08U[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_ACCUMULATE_WEIGHTED_08U[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_ACCUMULATE_WEIGHTED_08U[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_ACCUMULATE_WEIGHTED_08U[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_ACCUMULATE_WEIGHTED_08U[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_ACCUMULATE_WEIGHTED_08U[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_ACCUMULATE_WEIGHTED_08U[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_ACCUMULATE_WEIGHTED_08U[] = {{0, 0}};

static acf_scenario gScenarioArray_ACCUMULATE_WEIGHTED_08U[] = {
{2, 1, 12, 16, 0, gScenarioBufferData0_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData0_ACCUMULATE_WEIGHTED_08U, 4},
{4, 1, 24, 16, 0, gScenarioBufferData1_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData1_ACCUMULATE_WEIGHTED_08U, 4},
{6, 1, 36, 16, 0, gScenarioBufferData2_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData2_ACCUMULATE_WEIGHTED_08U, 4},
{8, 1, 48, 16, 0, gScenarioBufferData3_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData3_ACCUMULATE_WEIGHTED_08U, 4},
{10, 1, 60, 16, 0, gScenarioBufferData4_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData4_ACCUMULATE_WEIGHTED_08U, 4},
{12, 1, 72, 16, 0, gScenarioBufferData5_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData5_ACCUMULATE_WEIGHTED_08U, 4},
{14, 1, 84, 16, 0, gScenarioBufferData6_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData6_ACCUMULATE_WEIGHTED_08U, 4},
{16, 1, 96, 16, 0, gScenarioBufferData7_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData7_ACCUMULATE_WEIGHTED_08U, 4},
{18, 1, 108, 16, 0, gScenarioBufferData8_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData8_ACCUMULATE_WEIGHTED_08U, 4},
{20, 1, 120, 16, 0, gScenarioBufferData9_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData9_ACCUMULATE_WEIGHTED_08U, 4},
{22, 1, 132, 16, 0, gScenarioBufferData10_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData10_ACCUMULATE_WEIGHTED_08U, 4},
{24, 1, 144, 16, 0, gScenarioBufferData11_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData11_ACCUMULATE_WEIGHTED_08U, 4},
{26, 1, 156, 16, 0, gScenarioBufferData12_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData12_ACCUMULATE_WEIGHTED_08U, 4},
{28, 1, 168, 16, 0, gScenarioBufferData13_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData13_ACCUMULATE_WEIGHTED_08U, 4},
{30, 1, 180, 16, 0, gScenarioBufferData14_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData14_ACCUMULATE_WEIGHTED_08U, 4},
{32, 1, 192, 16, 0, gScenarioBufferData15_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData15_ACCUMULATE_WEIGHTED_08U, 4},
{36, 1, 216, 16, 0, gScenarioBufferData16_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData16_ACCUMULATE_WEIGHTED_08U, 4},
{40, 1, 240, 16, 0, gScenarioBufferData17_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData17_ACCUMULATE_WEIGHTED_08U, 4},
{44, 1, 264, 16, 0, gScenarioBufferData18_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData18_ACCUMULATE_WEIGHTED_08U, 4},
{48, 1, 288, 16, 0, gScenarioBufferData19_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData19_ACCUMULATE_WEIGHTED_08U, 4},
{52, 1, 312, 16, 0, gScenarioBufferData20_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData20_ACCUMULATE_WEIGHTED_08U, 4},
{56, 1, 336, 16, 0, gScenarioBufferData21_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData21_ACCUMULATE_WEIGHTED_08U, 4},
{60, 1, 360, 16, 0, gScenarioBufferData22_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData22_ACCUMULATE_WEIGHTED_08U, 4},
{64, 1, 384, 16, 0, gScenarioBufferData23_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData23_ACCUMULATE_WEIGHTED_08U, 4},
{72, 1, 432, 16, 0, gScenarioBufferData24_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData24_ACCUMULATE_WEIGHTED_08U, 4},
{80, 1, 480, 16, 0, gScenarioBufferData25_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData25_ACCUMULATE_WEIGHTED_08U, 4},
{88, 1, 528, 16, 0, gScenarioBufferData26_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData26_ACCUMULATE_WEIGHTED_08U, 4},
{96, 1, 576, 16, 0, gScenarioBufferData27_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData27_ACCUMULATE_WEIGHTED_08U, 4},
{104, 1, 624, 16, 0, gScenarioBufferData28_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData28_ACCUMULATE_WEIGHTED_08U, 4},
{112, 1, 672, 16, 0, gScenarioBufferData29_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData29_ACCUMULATE_WEIGHTED_08U, 4},
{120, 1, 720, 16, 0, gScenarioBufferData30_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData30_ACCUMULATE_WEIGHTED_08U, 4},
{128, 1, 768, 16, 0, gScenarioBufferData31_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData31_ACCUMULATE_WEIGHTED_08U, 4},
{2, 2, 24, 16, 0, gScenarioBufferData32_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData32_ACCUMULATE_WEIGHTED_08U, 4},
{4, 2, 48, 16, 0, gScenarioBufferData33_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData33_ACCUMULATE_WEIGHTED_08U, 4},
{6, 2, 72, 16, 0, gScenarioBufferData34_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData34_ACCUMULATE_WEIGHTED_08U, 4},
{8, 2, 96, 16, 0, gScenarioBufferData35_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData35_ACCUMULATE_WEIGHTED_08U, 4},
{10, 2, 120, 16, 0, gScenarioBufferData36_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData36_ACCUMULATE_WEIGHTED_08U, 4},
{12, 2, 144, 16, 0, gScenarioBufferData37_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData37_ACCUMULATE_WEIGHTED_08U, 4},
{14, 2, 168, 16, 0, gScenarioBufferData38_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData38_ACCUMULATE_WEIGHTED_08U, 4},
{16, 2, 192, 16, 0, gScenarioBufferData39_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData39_ACCUMULATE_WEIGHTED_08U, 4},
{18, 2, 216, 16, 0, gScenarioBufferData40_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData40_ACCUMULATE_WEIGHTED_08U, 4},
{20, 2, 240, 16, 0, gScenarioBufferData41_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData41_ACCUMULATE_WEIGHTED_08U, 4},
{22, 2, 264, 16, 0, gScenarioBufferData42_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData42_ACCUMULATE_WEIGHTED_08U, 4},
{24, 2, 288, 16, 0, gScenarioBufferData43_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData43_ACCUMULATE_WEIGHTED_08U, 4},
{26, 2, 312, 16, 0, gScenarioBufferData44_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData44_ACCUMULATE_WEIGHTED_08U, 4},
{28, 2, 336, 16, 0, gScenarioBufferData45_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData45_ACCUMULATE_WEIGHTED_08U, 4},
{30, 2, 360, 16, 0, gScenarioBufferData46_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData46_ACCUMULATE_WEIGHTED_08U, 4},
{32, 2, 384, 16, 0, gScenarioBufferData47_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData47_ACCUMULATE_WEIGHTED_08U, 4},
{36, 2, 432, 16, 0, gScenarioBufferData48_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData48_ACCUMULATE_WEIGHTED_08U, 4},
{40, 2, 480, 16, 0, gScenarioBufferData49_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData49_ACCUMULATE_WEIGHTED_08U, 4},
{44, 2, 528, 16, 0, gScenarioBufferData50_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData50_ACCUMULATE_WEIGHTED_08U, 4},
{48, 2, 576, 16, 0, gScenarioBufferData51_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData51_ACCUMULATE_WEIGHTED_08U, 4},
{52, 2, 624, 16, 0, gScenarioBufferData52_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData52_ACCUMULATE_WEIGHTED_08U, 4},
{56, 2, 672, 16, 0, gScenarioBufferData53_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData53_ACCUMULATE_WEIGHTED_08U, 4},
{60, 2, 720, 16, 0, gScenarioBufferData54_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData54_ACCUMULATE_WEIGHTED_08U, 4},
{64, 2, 768, 16, 0, gScenarioBufferData55_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData55_ACCUMULATE_WEIGHTED_08U, 4},
{72, 2, 864, 16, 0, gScenarioBufferData56_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData56_ACCUMULATE_WEIGHTED_08U, 4},
{80, 2, 960, 16, 0, gScenarioBufferData57_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData57_ACCUMULATE_WEIGHTED_08U, 4},
{88, 2, 1056, 16, 0, gScenarioBufferData58_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData58_ACCUMULATE_WEIGHTED_08U, 4},
{96, 2, 1152, 16, 0, gScenarioBufferData59_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData59_ACCUMULATE_WEIGHTED_08U, 4},
{104, 2, 1248, 16, 0, gScenarioBufferData60_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData60_ACCUMULATE_WEIGHTED_08U, 4},
{112, 2, 1344, 16, 0, gScenarioBufferData61_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData61_ACCUMULATE_WEIGHTED_08U, 4},
{120, 2, 1440, 16, 0, gScenarioBufferData62_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData62_ACCUMULATE_WEIGHTED_08U, 4},
{128, 2, 1536, 16, 0, gScenarioBufferData63_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData63_ACCUMULATE_WEIGHTED_08U, 4},
{2, 3, 36, 16, 0, gScenarioBufferData64_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData64_ACCUMULATE_WEIGHTED_08U, 4},
{4, 3, 72, 16, 0, gScenarioBufferData65_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData65_ACCUMULATE_WEIGHTED_08U, 4},
{6, 3, 108, 16, 0, gScenarioBufferData66_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData66_ACCUMULATE_WEIGHTED_08U, 4},
{8, 3, 144, 16, 0, gScenarioBufferData67_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData67_ACCUMULATE_WEIGHTED_08U, 4},
{10, 3, 180, 16, 0, gScenarioBufferData68_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData68_ACCUMULATE_WEIGHTED_08U, 4},
{12, 3, 216, 16, 0, gScenarioBufferData69_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData69_ACCUMULATE_WEIGHTED_08U, 4},
{14, 3, 252, 16, 0, gScenarioBufferData70_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData70_ACCUMULATE_WEIGHTED_08U, 4},
{16, 3, 288, 16, 0, gScenarioBufferData71_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData71_ACCUMULATE_WEIGHTED_08U, 4},
{18, 3, 324, 16, 0, gScenarioBufferData72_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData72_ACCUMULATE_WEIGHTED_08U, 4},
{20, 3, 360, 16, 0, gScenarioBufferData73_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData73_ACCUMULATE_WEIGHTED_08U, 4},
{22, 3, 396, 16, 0, gScenarioBufferData74_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData74_ACCUMULATE_WEIGHTED_08U, 4},
{24, 3, 432, 16, 0, gScenarioBufferData75_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData75_ACCUMULATE_WEIGHTED_08U, 4},
{26, 3, 468, 16, 0, gScenarioBufferData76_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData76_ACCUMULATE_WEIGHTED_08U, 4},
{28, 3, 504, 16, 0, gScenarioBufferData77_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData77_ACCUMULATE_WEIGHTED_08U, 4},
{30, 3, 540, 16, 0, gScenarioBufferData78_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData78_ACCUMULATE_WEIGHTED_08U, 4},
{32, 3, 576, 16, 0, gScenarioBufferData79_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData79_ACCUMULATE_WEIGHTED_08U, 4},
{36, 3, 648, 16, 0, gScenarioBufferData80_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData80_ACCUMULATE_WEIGHTED_08U, 4},
{40, 3, 720, 16, 0, gScenarioBufferData81_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData81_ACCUMULATE_WEIGHTED_08U, 4},
{44, 3, 792, 16, 0, gScenarioBufferData82_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData82_ACCUMULATE_WEIGHTED_08U, 4},
{48, 3, 864, 16, 0, gScenarioBufferData83_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData83_ACCUMULATE_WEIGHTED_08U, 4},
{52, 3, 936, 16, 0, gScenarioBufferData84_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData84_ACCUMULATE_WEIGHTED_08U, 4},
{56, 3, 1008, 16, 0, gScenarioBufferData85_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData85_ACCUMULATE_WEIGHTED_08U, 4},
{60, 3, 1080, 16, 0, gScenarioBufferData86_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData86_ACCUMULATE_WEIGHTED_08U, 4},
{64, 3, 1152, 16, 0, gScenarioBufferData87_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData87_ACCUMULATE_WEIGHTED_08U, 4},
{72, 3, 1296, 16, 0, gScenarioBufferData88_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData88_ACCUMULATE_WEIGHTED_08U, 4},
{80, 3, 1440, 16, 0, gScenarioBufferData89_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData89_ACCUMULATE_WEIGHTED_08U, 4},
{88, 3, 1584, 16, 0, gScenarioBufferData90_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData90_ACCUMULATE_WEIGHTED_08U, 4},
{96, 3, 1728, 16, 0, gScenarioBufferData91_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData91_ACCUMULATE_WEIGHTED_08U, 4},
{104, 3, 1872, 16, 0, gScenarioBufferData92_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData92_ACCUMULATE_WEIGHTED_08U, 4},
{112, 3, 2016, 16, 0, gScenarioBufferData93_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData93_ACCUMULATE_WEIGHTED_08U, 4},
{120, 3, 2160, 16, 0, gScenarioBufferData94_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData94_ACCUMULATE_WEIGHTED_08U, 4},
{128, 3, 2304, 16, 0, gScenarioBufferData95_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData95_ACCUMULATE_WEIGHTED_08U, 4},
{2, 4, 48, 16, 0, gScenarioBufferData96_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData96_ACCUMULATE_WEIGHTED_08U, 4},
{4, 4, 96, 16, 0, gScenarioBufferData97_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData97_ACCUMULATE_WEIGHTED_08U, 4},
{6, 4, 144, 16, 0, gScenarioBufferData98_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData98_ACCUMULATE_WEIGHTED_08U, 4},
{8, 4, 192, 16, 0, gScenarioBufferData99_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData99_ACCUMULATE_WEIGHTED_08U, 4},
{10, 4, 240, 16, 0, gScenarioBufferData100_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData100_ACCUMULATE_WEIGHTED_08U, 4},
{12, 4, 288, 16, 0, gScenarioBufferData101_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData101_ACCUMULATE_WEIGHTED_08U, 4},
{14, 4, 336, 16, 0, gScenarioBufferData102_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData102_ACCUMULATE_WEIGHTED_08U, 4},
{16, 4, 384, 16, 0, gScenarioBufferData103_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData103_ACCUMULATE_WEIGHTED_08U, 4},
{18, 4, 432, 16, 0, gScenarioBufferData104_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData104_ACCUMULATE_WEIGHTED_08U, 4},
{20, 4, 480, 16, 0, gScenarioBufferData105_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData105_ACCUMULATE_WEIGHTED_08U, 4},
{22, 4, 528, 16, 0, gScenarioBufferData106_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData106_ACCUMULATE_WEIGHTED_08U, 4},
{24, 4, 576, 16, 0, gScenarioBufferData107_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData107_ACCUMULATE_WEIGHTED_08U, 4},
{26, 4, 624, 16, 0, gScenarioBufferData108_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData108_ACCUMULATE_WEIGHTED_08U, 4},
{28, 4, 672, 16, 0, gScenarioBufferData109_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData109_ACCUMULATE_WEIGHTED_08U, 4},
{30, 4, 720, 16, 0, gScenarioBufferData110_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData110_ACCUMULATE_WEIGHTED_08U, 4},
{32, 4, 768, 16, 0, gScenarioBufferData111_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData111_ACCUMULATE_WEIGHTED_08U, 4},
{36, 4, 864, 16, 0, gScenarioBufferData112_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData112_ACCUMULATE_WEIGHTED_08U, 4},
{40, 4, 960, 16, 0, gScenarioBufferData113_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData113_ACCUMULATE_WEIGHTED_08U, 4},
{44, 4, 1056, 16, 0, gScenarioBufferData114_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData114_ACCUMULATE_WEIGHTED_08U, 4},
{48, 4, 1152, 16, 0, gScenarioBufferData115_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData115_ACCUMULATE_WEIGHTED_08U, 4},
{52, 4, 1248, 16, 0, gScenarioBufferData116_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData116_ACCUMULATE_WEIGHTED_08U, 4},
{56, 4, 1344, 16, 0, gScenarioBufferData117_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData117_ACCUMULATE_WEIGHTED_08U, 4},
{60, 4, 1440, 16, 0, gScenarioBufferData118_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData118_ACCUMULATE_WEIGHTED_08U, 4},
{64, 4, 1536, 16, 0, gScenarioBufferData119_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData119_ACCUMULATE_WEIGHTED_08U, 4},
{72, 4, 1728, 16, 0, gScenarioBufferData120_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData120_ACCUMULATE_WEIGHTED_08U, 4},
{80, 4, 1920, 16, 0, gScenarioBufferData121_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData121_ACCUMULATE_WEIGHTED_08U, 4},
{88, 4, 2112, 16, 0, gScenarioBufferData122_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData122_ACCUMULATE_WEIGHTED_08U, 4},
{96, 4, 2304, 16, 0, gScenarioBufferData123_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData123_ACCUMULATE_WEIGHTED_08U, 4},
{104, 4, 2496, 16, 0, gScenarioBufferData124_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData124_ACCUMULATE_WEIGHTED_08U, 4},
{112, 4, 2688, 16, 0, gScenarioBufferData125_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData125_ACCUMULATE_WEIGHTED_08U, 4},
{120, 4, 2880, 16, 0, gScenarioBufferData126_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData126_ACCUMULATE_WEIGHTED_08U, 4},
{128, 4, 3072, 16, 0, gScenarioBufferData127_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData127_ACCUMULATE_WEIGHTED_08U, 4},
{2, 5, 60, 16, 0, gScenarioBufferData128_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData128_ACCUMULATE_WEIGHTED_08U, 4},
{4, 5, 120, 16, 0, gScenarioBufferData129_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData129_ACCUMULATE_WEIGHTED_08U, 4},
{6, 5, 180, 16, 0, gScenarioBufferData130_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData130_ACCUMULATE_WEIGHTED_08U, 4},
{8, 5, 240, 16, 0, gScenarioBufferData131_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData131_ACCUMULATE_WEIGHTED_08U, 4},
{10, 5, 300, 16, 0, gScenarioBufferData132_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData132_ACCUMULATE_WEIGHTED_08U, 4},
{12, 5, 360, 16, 0, gScenarioBufferData133_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData133_ACCUMULATE_WEIGHTED_08U, 4},
{14, 5, 420, 16, 0, gScenarioBufferData134_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData134_ACCUMULATE_WEIGHTED_08U, 4},
{16, 5, 480, 16, 0, gScenarioBufferData135_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData135_ACCUMULATE_WEIGHTED_08U, 4},
{18, 5, 540, 16, 0, gScenarioBufferData136_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData136_ACCUMULATE_WEIGHTED_08U, 4},
{20, 5, 600, 16, 0, gScenarioBufferData137_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData137_ACCUMULATE_WEIGHTED_08U, 4},
{22, 5, 660, 16, 0, gScenarioBufferData138_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData138_ACCUMULATE_WEIGHTED_08U, 4},
{24, 5, 720, 16, 0, gScenarioBufferData139_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData139_ACCUMULATE_WEIGHTED_08U, 4},
{26, 5, 780, 16, 0, gScenarioBufferData140_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData140_ACCUMULATE_WEIGHTED_08U, 4},
{28, 5, 840, 16, 0, gScenarioBufferData141_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData141_ACCUMULATE_WEIGHTED_08U, 4},
{30, 5, 900, 16, 0, gScenarioBufferData142_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData142_ACCUMULATE_WEIGHTED_08U, 4},
{32, 5, 960, 16, 0, gScenarioBufferData143_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData143_ACCUMULATE_WEIGHTED_08U, 4},
{36, 5, 1080, 16, 0, gScenarioBufferData144_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData144_ACCUMULATE_WEIGHTED_08U, 4},
{40, 5, 1200, 16, 0, gScenarioBufferData145_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData145_ACCUMULATE_WEIGHTED_08U, 4},
{44, 5, 1320, 16, 0, gScenarioBufferData146_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData146_ACCUMULATE_WEIGHTED_08U, 4},
{48, 5, 1440, 16, 0, gScenarioBufferData147_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData147_ACCUMULATE_WEIGHTED_08U, 4},
{52, 5, 1560, 16, 0, gScenarioBufferData148_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData148_ACCUMULATE_WEIGHTED_08U, 4},
{56, 5, 1680, 16, 0, gScenarioBufferData149_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData149_ACCUMULATE_WEIGHTED_08U, 4},
{60, 5, 1800, 16, 0, gScenarioBufferData150_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData150_ACCUMULATE_WEIGHTED_08U, 4},
{64, 5, 1920, 16, 0, gScenarioBufferData151_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData151_ACCUMULATE_WEIGHTED_08U, 4},
{72, 5, 2160, 16, 0, gScenarioBufferData152_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData152_ACCUMULATE_WEIGHTED_08U, 4},
{80, 5, 2400, 16, 0, gScenarioBufferData153_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData153_ACCUMULATE_WEIGHTED_08U, 4},
{88, 5, 2640, 16, 0, gScenarioBufferData154_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData154_ACCUMULATE_WEIGHTED_08U, 4},
{96, 5, 2880, 16, 0, gScenarioBufferData155_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData155_ACCUMULATE_WEIGHTED_08U, 4},
{104, 5, 3120, 16, 0, gScenarioBufferData156_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData156_ACCUMULATE_WEIGHTED_08U, 4},
{112, 5, 3360, 16, 0, gScenarioBufferData157_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData157_ACCUMULATE_WEIGHTED_08U, 4},
{120, 5, 3600, 16, 0, gScenarioBufferData158_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData158_ACCUMULATE_WEIGHTED_08U, 4},
{128, 5, 3840, 16, 0, gScenarioBufferData159_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData159_ACCUMULATE_WEIGHTED_08U, 4},
{2, 6, 72, 16, 0, gScenarioBufferData160_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData160_ACCUMULATE_WEIGHTED_08U, 4},
{4, 6, 144, 16, 0, gScenarioBufferData161_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData161_ACCUMULATE_WEIGHTED_08U, 4},
{6, 6, 216, 16, 0, gScenarioBufferData162_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData162_ACCUMULATE_WEIGHTED_08U, 4},
{8, 6, 288, 16, 0, gScenarioBufferData163_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData163_ACCUMULATE_WEIGHTED_08U, 4},
{10, 6, 360, 16, 0, gScenarioBufferData164_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData164_ACCUMULATE_WEIGHTED_08U, 4},
{12, 6, 432, 16, 0, gScenarioBufferData165_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData165_ACCUMULATE_WEIGHTED_08U, 4},
{14, 6, 504, 16, 0, gScenarioBufferData166_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData166_ACCUMULATE_WEIGHTED_08U, 4},
{16, 6, 576, 16, 0, gScenarioBufferData167_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData167_ACCUMULATE_WEIGHTED_08U, 4},
{18, 6, 648, 16, 0, gScenarioBufferData168_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData168_ACCUMULATE_WEIGHTED_08U, 4},
{20, 6, 720, 16, 0, gScenarioBufferData169_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData169_ACCUMULATE_WEIGHTED_08U, 4},
{22, 6, 792, 16, 0, gScenarioBufferData170_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData170_ACCUMULATE_WEIGHTED_08U, 4},
{24, 6, 864, 16, 0, gScenarioBufferData171_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData171_ACCUMULATE_WEIGHTED_08U, 4},
{26, 6, 936, 16, 0, gScenarioBufferData172_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData172_ACCUMULATE_WEIGHTED_08U, 4},
{28, 6, 1008, 16, 0, gScenarioBufferData173_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData173_ACCUMULATE_WEIGHTED_08U, 4},
{30, 6, 1080, 16, 0, gScenarioBufferData174_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData174_ACCUMULATE_WEIGHTED_08U, 4},
{32, 6, 1152, 16, 0, gScenarioBufferData175_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData175_ACCUMULATE_WEIGHTED_08U, 4},
{36, 6, 1296, 16, 0, gScenarioBufferData176_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData176_ACCUMULATE_WEIGHTED_08U, 4},
{40, 6, 1440, 16, 0, gScenarioBufferData177_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData177_ACCUMULATE_WEIGHTED_08U, 4},
{44, 6, 1584, 16, 0, gScenarioBufferData178_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData178_ACCUMULATE_WEIGHTED_08U, 4},
{48, 6, 1728, 16, 0, gScenarioBufferData179_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData179_ACCUMULATE_WEIGHTED_08U, 4},
{52, 6, 1872, 16, 0, gScenarioBufferData180_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData180_ACCUMULATE_WEIGHTED_08U, 4},
{56, 6, 2016, 16, 0, gScenarioBufferData181_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData181_ACCUMULATE_WEIGHTED_08U, 4},
{60, 6, 2160, 16, 0, gScenarioBufferData182_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData182_ACCUMULATE_WEIGHTED_08U, 4},
{64, 6, 2304, 16, 0, gScenarioBufferData183_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData183_ACCUMULATE_WEIGHTED_08U, 4},
{72, 6, 2592, 16, 0, gScenarioBufferData184_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData184_ACCUMULATE_WEIGHTED_08U, 4},
{80, 6, 2880, 16, 0, gScenarioBufferData185_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData185_ACCUMULATE_WEIGHTED_08U, 4},
{88, 6, 3168, 16, 0, gScenarioBufferData186_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData186_ACCUMULATE_WEIGHTED_08U, 4},
{96, 6, 3456, 16, 0, gScenarioBufferData187_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData187_ACCUMULATE_WEIGHTED_08U, 4},
{104, 6, 3744, 16, 0, gScenarioBufferData188_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData188_ACCUMULATE_WEIGHTED_08U, 4},
{2, 8, 96, 16, 0, gScenarioBufferData189_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData189_ACCUMULATE_WEIGHTED_08U, 4},
{4, 8, 192, 16, 0, gScenarioBufferData190_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData190_ACCUMULATE_WEIGHTED_08U, 4},
{6, 8, 288, 16, 0, gScenarioBufferData191_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData191_ACCUMULATE_WEIGHTED_08U, 4},
{8, 8, 384, 16, 0, gScenarioBufferData192_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData192_ACCUMULATE_WEIGHTED_08U, 4},
{10, 8, 480, 16, 0, gScenarioBufferData193_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData193_ACCUMULATE_WEIGHTED_08U, 4},
{12, 8, 576, 16, 0, gScenarioBufferData194_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData194_ACCUMULATE_WEIGHTED_08U, 4},
{14, 8, 672, 16, 0, gScenarioBufferData195_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData195_ACCUMULATE_WEIGHTED_08U, 4},
{16, 8, 768, 16, 0, gScenarioBufferData196_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData196_ACCUMULATE_WEIGHTED_08U, 4},
{18, 8, 864, 16, 0, gScenarioBufferData197_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData197_ACCUMULATE_WEIGHTED_08U, 4},
{20, 8, 960, 16, 0, gScenarioBufferData198_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData198_ACCUMULATE_WEIGHTED_08U, 4},
{22, 8, 1056, 16, 0, gScenarioBufferData199_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData199_ACCUMULATE_WEIGHTED_08U, 4},
{24, 8, 1152, 16, 0, gScenarioBufferData200_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData200_ACCUMULATE_WEIGHTED_08U, 4},
{26, 8, 1248, 16, 0, gScenarioBufferData201_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData201_ACCUMULATE_WEIGHTED_08U, 4},
{28, 8, 1344, 16, 0, gScenarioBufferData202_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData202_ACCUMULATE_WEIGHTED_08U, 4},
{30, 8, 1440, 16, 0, gScenarioBufferData203_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData203_ACCUMULATE_WEIGHTED_08U, 4},
{32, 8, 1536, 16, 0, gScenarioBufferData204_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData204_ACCUMULATE_WEIGHTED_08U, 4},
{36, 8, 1728, 16, 0, gScenarioBufferData205_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData205_ACCUMULATE_WEIGHTED_08U, 4},
{40, 8, 1920, 16, 0, gScenarioBufferData206_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData206_ACCUMULATE_WEIGHTED_08U, 4},
{44, 8, 2112, 16, 0, gScenarioBufferData207_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData207_ACCUMULATE_WEIGHTED_08U, 4},
{48, 8, 2304, 16, 0, gScenarioBufferData208_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData208_ACCUMULATE_WEIGHTED_08U, 4},
{52, 8, 2496, 16, 0, gScenarioBufferData209_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData209_ACCUMULATE_WEIGHTED_08U, 4},
{56, 8, 2688, 16, 0, gScenarioBufferData210_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData210_ACCUMULATE_WEIGHTED_08U, 4},
{60, 8, 2880, 16, 0, gScenarioBufferData211_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData211_ACCUMULATE_WEIGHTED_08U, 4},
{64, 8, 3072, 16, 0, gScenarioBufferData212_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData212_ACCUMULATE_WEIGHTED_08U, 4},
{72, 8, 3456, 16, 0, gScenarioBufferData213_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData213_ACCUMULATE_WEIGHTED_08U, 4},
{80, 8, 3840, 16, 0, gScenarioBufferData214_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData214_ACCUMULATE_WEIGHTED_08U, 4},
{2, 10, 120, 16, 0, gScenarioBufferData215_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData215_ACCUMULATE_WEIGHTED_08U, 4},
{4, 10, 240, 16, 0, gScenarioBufferData216_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData216_ACCUMULATE_WEIGHTED_08U, 4},
{6, 10, 360, 16, 0, gScenarioBufferData217_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData217_ACCUMULATE_WEIGHTED_08U, 4},
{8, 10, 480, 16, 0, gScenarioBufferData218_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData218_ACCUMULATE_WEIGHTED_08U, 4},
{10, 10, 600, 16, 0, gScenarioBufferData219_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData219_ACCUMULATE_WEIGHTED_08U, 4},
{12, 10, 720, 16, 0, gScenarioBufferData220_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData220_ACCUMULATE_WEIGHTED_08U, 4},
{14, 10, 840, 16, 0, gScenarioBufferData221_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData221_ACCUMULATE_WEIGHTED_08U, 4},
{16, 10, 960, 16, 0, gScenarioBufferData222_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData222_ACCUMULATE_WEIGHTED_08U, 4},
{18, 10, 1080, 16, 0, gScenarioBufferData223_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData223_ACCUMULATE_WEIGHTED_08U, 4},
{20, 10, 1200, 16, 0, gScenarioBufferData224_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData224_ACCUMULATE_WEIGHTED_08U, 4},
{22, 10, 1320, 16, 0, gScenarioBufferData225_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData225_ACCUMULATE_WEIGHTED_08U, 4},
{24, 10, 1440, 16, 0, gScenarioBufferData226_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData226_ACCUMULATE_WEIGHTED_08U, 4},
{26, 10, 1560, 16, 0, gScenarioBufferData227_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData227_ACCUMULATE_WEIGHTED_08U, 4},
{28, 10, 1680, 16, 0, gScenarioBufferData228_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData228_ACCUMULATE_WEIGHTED_08U, 4},
{30, 10, 1800, 16, 0, gScenarioBufferData229_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData229_ACCUMULATE_WEIGHTED_08U, 4},
{32, 10, 1920, 16, 0, gScenarioBufferData230_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData230_ACCUMULATE_WEIGHTED_08U, 4},
{36, 10, 2160, 16, 0, gScenarioBufferData231_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData231_ACCUMULATE_WEIGHTED_08U, 4},
{40, 10, 2400, 16, 0, gScenarioBufferData232_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData232_ACCUMULATE_WEIGHTED_08U, 4},
{44, 10, 2640, 16, 0, gScenarioBufferData233_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData233_ACCUMULATE_WEIGHTED_08U, 4},
{48, 10, 2880, 16, 0, gScenarioBufferData234_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData234_ACCUMULATE_WEIGHTED_08U, 4},
{52, 10, 3120, 16, 0, gScenarioBufferData235_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData235_ACCUMULATE_WEIGHTED_08U, 4},
{56, 10, 3360, 16, 0, gScenarioBufferData236_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData236_ACCUMULATE_WEIGHTED_08U, 4},
{60, 10, 3600, 16, 0, gScenarioBufferData237_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData237_ACCUMULATE_WEIGHTED_08U, 4},
{64, 10, 3840, 16, 0, gScenarioBufferData238_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData238_ACCUMULATE_WEIGHTED_08U, 4},
{2, 12, 144, 16, 0, gScenarioBufferData239_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData239_ACCUMULATE_WEIGHTED_08U, 4},
{4, 12, 288, 16, 0, gScenarioBufferData240_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData240_ACCUMULATE_WEIGHTED_08U, 4},
{6, 12, 432, 16, 0, gScenarioBufferData241_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData241_ACCUMULATE_WEIGHTED_08U, 4},
{8, 12, 576, 16, 0, gScenarioBufferData242_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData242_ACCUMULATE_WEIGHTED_08U, 4},
{10, 12, 720, 16, 0, gScenarioBufferData243_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData243_ACCUMULATE_WEIGHTED_08U, 4},
{12, 12, 864, 16, 0, gScenarioBufferData244_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData244_ACCUMULATE_WEIGHTED_08U, 4},
{14, 12, 1008, 16, 0, gScenarioBufferData245_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData245_ACCUMULATE_WEIGHTED_08U, 4},
{16, 12, 1152, 16, 0, gScenarioBufferData246_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData246_ACCUMULATE_WEIGHTED_08U, 4},
{18, 12, 1296, 16, 0, gScenarioBufferData247_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData247_ACCUMULATE_WEIGHTED_08U, 4},
{20, 12, 1440, 16, 0, gScenarioBufferData248_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData248_ACCUMULATE_WEIGHTED_08U, 4},
{22, 12, 1584, 16, 0, gScenarioBufferData249_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData249_ACCUMULATE_WEIGHTED_08U, 4},
{24, 12, 1728, 16, 0, gScenarioBufferData250_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData250_ACCUMULATE_WEIGHTED_08U, 4},
{26, 12, 1872, 16, 0, gScenarioBufferData251_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData251_ACCUMULATE_WEIGHTED_08U, 4},
{28, 12, 2016, 16, 0, gScenarioBufferData252_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData252_ACCUMULATE_WEIGHTED_08U, 4},
{30, 12, 2160, 16, 0, gScenarioBufferData253_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData253_ACCUMULATE_WEIGHTED_08U, 4},
{32, 12, 2304, 16, 0, gScenarioBufferData254_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData254_ACCUMULATE_WEIGHTED_08U, 4},
{36, 12, 2592, 16, 0, gScenarioBufferData255_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData255_ACCUMULATE_WEIGHTED_08U, 4},
{40, 12, 2880, 16, 0, gScenarioBufferData256_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData256_ACCUMULATE_WEIGHTED_08U, 4},
{44, 12, 3168, 16, 0, gScenarioBufferData257_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData257_ACCUMULATE_WEIGHTED_08U, 4},
{48, 12, 3456, 16, 0, gScenarioBufferData258_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData258_ACCUMULATE_WEIGHTED_08U, 4},
{52, 12, 3744, 16, 0, gScenarioBufferData259_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData259_ACCUMULATE_WEIGHTED_08U, 4},
{2, 14, 168, 16, 0, gScenarioBufferData260_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData260_ACCUMULATE_WEIGHTED_08U, 4},
{4, 14, 336, 16, 0, gScenarioBufferData261_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData261_ACCUMULATE_WEIGHTED_08U, 4},
{6, 14, 504, 16, 0, gScenarioBufferData262_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData262_ACCUMULATE_WEIGHTED_08U, 4},
{8, 14, 672, 16, 0, gScenarioBufferData263_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData263_ACCUMULATE_WEIGHTED_08U, 4},
{10, 14, 840, 16, 0, gScenarioBufferData264_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData264_ACCUMULATE_WEIGHTED_08U, 4},
{12, 14, 1008, 16, 0, gScenarioBufferData265_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData265_ACCUMULATE_WEIGHTED_08U, 4},
{14, 14, 1176, 16, 0, gScenarioBufferData266_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData266_ACCUMULATE_WEIGHTED_08U, 4},
{16, 14, 1344, 16, 0, gScenarioBufferData267_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData267_ACCUMULATE_WEIGHTED_08U, 4},
{18, 14, 1512, 16, 0, gScenarioBufferData268_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData268_ACCUMULATE_WEIGHTED_08U, 4},
{20, 14, 1680, 16, 0, gScenarioBufferData269_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData269_ACCUMULATE_WEIGHTED_08U, 4},
{22, 14, 1848, 16, 0, gScenarioBufferData270_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData270_ACCUMULATE_WEIGHTED_08U, 4},
{24, 14, 2016, 16, 0, gScenarioBufferData271_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData271_ACCUMULATE_WEIGHTED_08U, 4},
{26, 14, 2184, 16, 0, gScenarioBufferData272_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData272_ACCUMULATE_WEIGHTED_08U, 4},
{28, 14, 2352, 16, 0, gScenarioBufferData273_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData273_ACCUMULATE_WEIGHTED_08U, 4},
{30, 14, 2520, 16, 0, gScenarioBufferData274_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData274_ACCUMULATE_WEIGHTED_08U, 4},
{32, 14, 2688, 16, 0, gScenarioBufferData275_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData275_ACCUMULATE_WEIGHTED_08U, 4},
{36, 14, 3024, 16, 0, gScenarioBufferData276_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData276_ACCUMULATE_WEIGHTED_08U, 4},
{40, 14, 3360, 16, 0, gScenarioBufferData277_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData277_ACCUMULATE_WEIGHTED_08U, 4},
{44, 14, 3696, 16, 0, gScenarioBufferData278_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData278_ACCUMULATE_WEIGHTED_08U, 4},
{2, 16, 192, 16, 0, gScenarioBufferData279_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData279_ACCUMULATE_WEIGHTED_08U, 4},
{4, 16, 384, 16, 0, gScenarioBufferData280_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData280_ACCUMULATE_WEIGHTED_08U, 4},
{6, 16, 576, 16, 0, gScenarioBufferData281_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData281_ACCUMULATE_WEIGHTED_08U, 4},
{8, 16, 768, 16, 0, gScenarioBufferData282_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData282_ACCUMULATE_WEIGHTED_08U, 4},
{10, 16, 960, 16, 0, gScenarioBufferData283_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData283_ACCUMULATE_WEIGHTED_08U, 4},
{12, 16, 1152, 16, 0, gScenarioBufferData284_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData284_ACCUMULATE_WEIGHTED_08U, 4},
{14, 16, 1344, 16, 0, gScenarioBufferData285_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData285_ACCUMULATE_WEIGHTED_08U, 4},
{16, 16, 1536, 16, 0, gScenarioBufferData286_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData286_ACCUMULATE_WEIGHTED_08U, 4},
{18, 16, 1728, 16, 0, gScenarioBufferData287_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData287_ACCUMULATE_WEIGHTED_08U, 4},
{20, 16, 1920, 16, 0, gScenarioBufferData288_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData288_ACCUMULATE_WEIGHTED_08U, 4},
{22, 16, 2112, 16, 0, gScenarioBufferData289_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData289_ACCUMULATE_WEIGHTED_08U, 4},
{24, 16, 2304, 16, 0, gScenarioBufferData290_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData290_ACCUMULATE_WEIGHTED_08U, 4},
{26, 16, 2496, 16, 0, gScenarioBufferData291_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData291_ACCUMULATE_WEIGHTED_08U, 4},
{28, 16, 2688, 16, 0, gScenarioBufferData292_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData292_ACCUMULATE_WEIGHTED_08U, 4},
{30, 16, 2880, 16, 0, gScenarioBufferData293_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData293_ACCUMULATE_WEIGHTED_08U, 4},
{32, 16, 3072, 16, 0, gScenarioBufferData294_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData294_ACCUMULATE_WEIGHTED_08U, 4},
{36, 16, 3456, 16, 0, gScenarioBufferData295_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData295_ACCUMULATE_WEIGHTED_08U, 4},
{40, 16, 3840, 16, 0, gScenarioBufferData296_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData296_ACCUMULATE_WEIGHTED_08U, 4},
{2, 18, 216, 16, 0, gScenarioBufferData297_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData297_ACCUMULATE_WEIGHTED_08U, 4},
{4, 18, 432, 16, 0, gScenarioBufferData298_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData298_ACCUMULATE_WEIGHTED_08U, 4},
{6, 18, 648, 16, 0, gScenarioBufferData299_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData299_ACCUMULATE_WEIGHTED_08U, 4},
{8, 18, 864, 16, 0, gScenarioBufferData300_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData300_ACCUMULATE_WEIGHTED_08U, 4},
{10, 18, 1080, 16, 0, gScenarioBufferData301_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData301_ACCUMULATE_WEIGHTED_08U, 4},
{12, 18, 1296, 16, 0, gScenarioBufferData302_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData302_ACCUMULATE_WEIGHTED_08U, 4},
{14, 18, 1512, 16, 0, gScenarioBufferData303_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData303_ACCUMULATE_WEIGHTED_08U, 4},
{16, 18, 1728, 16, 0, gScenarioBufferData304_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData304_ACCUMULATE_WEIGHTED_08U, 4},
{18, 18, 1944, 16, 0, gScenarioBufferData305_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData305_ACCUMULATE_WEIGHTED_08U, 4},
{20, 18, 2160, 16, 0, gScenarioBufferData306_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData306_ACCUMULATE_WEIGHTED_08U, 4},
{22, 18, 2376, 16, 0, gScenarioBufferData307_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData307_ACCUMULATE_WEIGHTED_08U, 4},
{24, 18, 2592, 16, 0, gScenarioBufferData308_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData308_ACCUMULATE_WEIGHTED_08U, 4},
{26, 18, 2808, 16, 0, gScenarioBufferData309_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData309_ACCUMULATE_WEIGHTED_08U, 4},
{28, 18, 3024, 16, 0, gScenarioBufferData310_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData310_ACCUMULATE_WEIGHTED_08U, 4},
{30, 18, 3240, 16, 0, gScenarioBufferData311_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData311_ACCUMULATE_WEIGHTED_08U, 4},
{32, 18, 3456, 16, 0, gScenarioBufferData312_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData312_ACCUMULATE_WEIGHTED_08U, 4},
{2, 20, 240, 16, 0, gScenarioBufferData313_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData313_ACCUMULATE_WEIGHTED_08U, 4},
{4, 20, 480, 16, 0, gScenarioBufferData314_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData314_ACCUMULATE_WEIGHTED_08U, 4},
{6, 20, 720, 16, 0, gScenarioBufferData315_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData315_ACCUMULATE_WEIGHTED_08U, 4},
{8, 20, 960, 16, 0, gScenarioBufferData316_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData316_ACCUMULATE_WEIGHTED_08U, 4},
{10, 20, 1200, 16, 0, gScenarioBufferData317_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData317_ACCUMULATE_WEIGHTED_08U, 4},
{12, 20, 1440, 16, 0, gScenarioBufferData318_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData318_ACCUMULATE_WEIGHTED_08U, 4},
{14, 20, 1680, 16, 0, gScenarioBufferData319_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData319_ACCUMULATE_WEIGHTED_08U, 4},
{16, 20, 1920, 16, 0, gScenarioBufferData320_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData320_ACCUMULATE_WEIGHTED_08U, 4},
{18, 20, 2160, 16, 0, gScenarioBufferData321_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData321_ACCUMULATE_WEIGHTED_08U, 4},
{20, 20, 2400, 16, 0, gScenarioBufferData322_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData322_ACCUMULATE_WEIGHTED_08U, 4},
{22, 20, 2640, 16, 0, gScenarioBufferData323_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData323_ACCUMULATE_WEIGHTED_08U, 4},
{24, 20, 2880, 16, 0, gScenarioBufferData324_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData324_ACCUMULATE_WEIGHTED_08U, 4},
{26, 20, 3120, 16, 0, gScenarioBufferData325_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData325_ACCUMULATE_WEIGHTED_08U, 4},
{28, 20, 3360, 16, 0, gScenarioBufferData326_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData326_ACCUMULATE_WEIGHTED_08U, 4},
{30, 20, 3600, 16, 0, gScenarioBufferData327_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData327_ACCUMULATE_WEIGHTED_08U, 4},
{32, 20, 3840, 16, 0, gScenarioBufferData328_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData328_ACCUMULATE_WEIGHTED_08U, 4},
{2, 22, 264, 16, 0, gScenarioBufferData329_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData329_ACCUMULATE_WEIGHTED_08U, 4},
{4, 22, 528, 16, 0, gScenarioBufferData330_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData330_ACCUMULATE_WEIGHTED_08U, 4},
{6, 22, 792, 16, 0, gScenarioBufferData331_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData331_ACCUMULATE_WEIGHTED_08U, 4},
{8, 22, 1056, 16, 0, gScenarioBufferData332_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData332_ACCUMULATE_WEIGHTED_08U, 4},
{10, 22, 1320, 16, 0, gScenarioBufferData333_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData333_ACCUMULATE_WEIGHTED_08U, 4},
{12, 22, 1584, 16, 0, gScenarioBufferData334_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData334_ACCUMULATE_WEIGHTED_08U, 4},
{14, 22, 1848, 16, 0, gScenarioBufferData335_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData335_ACCUMULATE_WEIGHTED_08U, 4},
{16, 22, 2112, 16, 0, gScenarioBufferData336_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData336_ACCUMULATE_WEIGHTED_08U, 4},
{18, 22, 2376, 16, 0, gScenarioBufferData337_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData337_ACCUMULATE_WEIGHTED_08U, 4},
{20, 22, 2640, 16, 0, gScenarioBufferData338_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData338_ACCUMULATE_WEIGHTED_08U, 4},
{22, 22, 2904, 16, 0, gScenarioBufferData339_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData339_ACCUMULATE_WEIGHTED_08U, 4},
{24, 22, 3168, 16, 0, gScenarioBufferData340_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData340_ACCUMULATE_WEIGHTED_08U, 4},
{26, 22, 3432, 16, 0, gScenarioBufferData341_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData341_ACCUMULATE_WEIGHTED_08U, 4},
{28, 22, 3696, 16, 0, gScenarioBufferData342_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData342_ACCUMULATE_WEIGHTED_08U, 4},
{2, 24, 288, 16, 0, gScenarioBufferData343_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData343_ACCUMULATE_WEIGHTED_08U, 4},
{4, 24, 576, 16, 0, gScenarioBufferData344_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData344_ACCUMULATE_WEIGHTED_08U, 4},
{6, 24, 864, 16, 0, gScenarioBufferData345_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData345_ACCUMULATE_WEIGHTED_08U, 4},
{8, 24, 1152, 16, 0, gScenarioBufferData346_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData346_ACCUMULATE_WEIGHTED_08U, 4},
{10, 24, 1440, 16, 0, gScenarioBufferData347_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData347_ACCUMULATE_WEIGHTED_08U, 4},
{12, 24, 1728, 16, 0, gScenarioBufferData348_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData348_ACCUMULATE_WEIGHTED_08U, 4},
{14, 24, 2016, 16, 0, gScenarioBufferData349_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData349_ACCUMULATE_WEIGHTED_08U, 4},
{16, 24, 2304, 16, 0, gScenarioBufferData350_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData350_ACCUMULATE_WEIGHTED_08U, 4},
{18, 24, 2592, 16, 0, gScenarioBufferData351_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData351_ACCUMULATE_WEIGHTED_08U, 4},
{20, 24, 2880, 16, 0, gScenarioBufferData352_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData352_ACCUMULATE_WEIGHTED_08U, 4},
{22, 24, 3168, 16, 0, gScenarioBufferData353_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData353_ACCUMULATE_WEIGHTED_08U, 4},
{24, 24, 3456, 16, 0, gScenarioBufferData354_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData354_ACCUMULATE_WEIGHTED_08U, 4},
{26, 24, 3744, 16, 0, gScenarioBufferData355_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData355_ACCUMULATE_WEIGHTED_08U, 4},
{2, 26, 312, 16, 0, gScenarioBufferData356_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData356_ACCUMULATE_WEIGHTED_08U, 4},
{4, 26, 624, 16, 0, gScenarioBufferData357_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData357_ACCUMULATE_WEIGHTED_08U, 4},
{6, 26, 936, 16, 0, gScenarioBufferData358_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData358_ACCUMULATE_WEIGHTED_08U, 4},
{8, 26, 1248, 16, 0, gScenarioBufferData359_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData359_ACCUMULATE_WEIGHTED_08U, 4},
{10, 26, 1560, 16, 0, gScenarioBufferData360_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData360_ACCUMULATE_WEIGHTED_08U, 4},
{12, 26, 1872, 16, 0, gScenarioBufferData361_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData361_ACCUMULATE_WEIGHTED_08U, 4},
{14, 26, 2184, 16, 0, gScenarioBufferData362_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData362_ACCUMULATE_WEIGHTED_08U, 4},
{16, 26, 2496, 16, 0, gScenarioBufferData363_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData363_ACCUMULATE_WEIGHTED_08U, 4},
{18, 26, 2808, 16, 0, gScenarioBufferData364_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData364_ACCUMULATE_WEIGHTED_08U, 4},
{20, 26, 3120, 16, 0, gScenarioBufferData365_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData365_ACCUMULATE_WEIGHTED_08U, 4},
{22, 26, 3432, 16, 0, gScenarioBufferData366_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData366_ACCUMULATE_WEIGHTED_08U, 4},
{24, 26, 3744, 16, 0, gScenarioBufferData367_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData367_ACCUMULATE_WEIGHTED_08U, 4},
{2, 28, 336, 16, 0, gScenarioBufferData368_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData368_ACCUMULATE_WEIGHTED_08U, 4},
{4, 28, 672, 16, 0, gScenarioBufferData369_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData369_ACCUMULATE_WEIGHTED_08U, 4},
{6, 28, 1008, 16, 0, gScenarioBufferData370_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData370_ACCUMULATE_WEIGHTED_08U, 4},
{8, 28, 1344, 16, 0, gScenarioBufferData371_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData371_ACCUMULATE_WEIGHTED_08U, 4},
{10, 28, 1680, 16, 0, gScenarioBufferData372_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData372_ACCUMULATE_WEIGHTED_08U, 4},
{12, 28, 2016, 16, 0, gScenarioBufferData373_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData373_ACCUMULATE_WEIGHTED_08U, 4},
{14, 28, 2352, 16, 0, gScenarioBufferData374_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData374_ACCUMULATE_WEIGHTED_08U, 4},
{16, 28, 2688, 16, 0, gScenarioBufferData375_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData375_ACCUMULATE_WEIGHTED_08U, 4},
{18, 28, 3024, 16, 0, gScenarioBufferData376_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData376_ACCUMULATE_WEIGHTED_08U, 4},
{20, 28, 3360, 16, 0, gScenarioBufferData377_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData377_ACCUMULATE_WEIGHTED_08U, 4},
{22, 28, 3696, 16, 0, gScenarioBufferData378_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData378_ACCUMULATE_WEIGHTED_08U, 4},
{2, 30, 360, 16, 0, gScenarioBufferData379_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData379_ACCUMULATE_WEIGHTED_08U, 4},
{4, 30, 720, 16, 0, gScenarioBufferData380_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData380_ACCUMULATE_WEIGHTED_08U, 4},
{6, 30, 1080, 16, 0, gScenarioBufferData381_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData381_ACCUMULATE_WEIGHTED_08U, 4},
{8, 30, 1440, 16, 0, gScenarioBufferData382_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData382_ACCUMULATE_WEIGHTED_08U, 4},
{10, 30, 1800, 16, 0, gScenarioBufferData383_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData383_ACCUMULATE_WEIGHTED_08U, 4},
{12, 30, 2160, 16, 0, gScenarioBufferData384_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData384_ACCUMULATE_WEIGHTED_08U, 4},
{14, 30, 2520, 16, 0, gScenarioBufferData385_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData385_ACCUMULATE_WEIGHTED_08U, 4},
{16, 30, 2880, 16, 0, gScenarioBufferData386_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData386_ACCUMULATE_WEIGHTED_08U, 4},
{18, 30, 3240, 16, 0, gScenarioBufferData387_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData387_ACCUMULATE_WEIGHTED_08U, 4},
{20, 30, 3600, 16, 0, gScenarioBufferData388_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData388_ACCUMULATE_WEIGHTED_08U, 4},
{2, 32, 384, 16, 0, gScenarioBufferData389_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData389_ACCUMULATE_WEIGHTED_08U, 4},
{4, 32, 768, 16, 0, gScenarioBufferData390_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData390_ACCUMULATE_WEIGHTED_08U, 4},
{6, 32, 1152, 16, 0, gScenarioBufferData391_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData391_ACCUMULATE_WEIGHTED_08U, 4},
{8, 32, 1536, 16, 0, gScenarioBufferData392_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData392_ACCUMULATE_WEIGHTED_08U, 4},
{10, 32, 1920, 16, 0, gScenarioBufferData393_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData393_ACCUMULATE_WEIGHTED_08U, 4},
{12, 32, 2304, 16, 0, gScenarioBufferData394_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData394_ACCUMULATE_WEIGHTED_08U, 4},
{14, 32, 2688, 16, 0, gScenarioBufferData395_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData395_ACCUMULATE_WEIGHTED_08U, 4},
{16, 32, 3072, 16, 0, gScenarioBufferData396_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData396_ACCUMULATE_WEIGHTED_08U, 4},
{18, 32, 3456, 16, 0, gScenarioBufferData397_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData397_ACCUMULATE_WEIGHTED_08U, 4},
{20, 32, 3840, 16, 0, gScenarioBufferData398_ACCUMULATE_WEIGHTED_08U, 64, gScenarioKernelData398_ACCUMULATE_WEIGHTED_08U, 4}
};

static acf_scenario_list gScenarioList_ACCUMULATE_WEIGHTED_08U = {
399, //number of scenarios
gScenarioArray_ACCUMULATE_WEIGHTED_08U};
//**************************************************************

class ACCUMULATE_WEIGHTED_08U : public ACF_Process_APU
{

public:
   ACCUMULATE_WEIGHTED_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("ACCUMULATE_WEIGHTED_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("ACCUMULATE_WEIGHTED_08U",
                                        ACCUMULATE_WEIGHTED_08U_LOAD_SEGMENTS,
                                        ACCUMULATE_WEIGHTED_08U_LOAD_PMEM, ACCUMULATE_WEIGHTED_08U_LOAD_PMEM_SIZE,
                                        ACCUMULATE_WEIGHTED_08U_LOAD_DMEM, ACCUMULATE_WEIGHTED_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(ACCUMULATE_WEIGHTED_08U_APEX_LOG_BUFFER, ACCUMULATE_WEIGHTED_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("ALPHA", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_ACCUMULATE_WEIGHTED_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_ACCUMULATE_WEIGHTED_08U

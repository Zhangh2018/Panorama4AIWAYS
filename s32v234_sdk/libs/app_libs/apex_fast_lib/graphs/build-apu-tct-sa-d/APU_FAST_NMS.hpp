#ifndef _ACF_PROCESS_APU_APU_FAST_NMS
#define _ACF_PROCESS_APU_APU_FAST_NMS

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <APU_FAST_NMS_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_APU_FAST_NMS[] = {{4, 1, 10, 0, {3, 3, 4, 4}}, {4, 1, 4, 3, {1, 1, 2, 2}}, {4, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_APU_FAST_NMS[] = {{6, 1, 10, 0, {3, 3, 4, 4}}, {6, 1, 4, 3, {1, 1, 2, 2}}, {6, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_APU_FAST_NMS[] = {{8, 1, 10, 0, {3, 3, 4, 4}}, {8, 1, 4, 3, {1, 1, 2, 2}}, {8, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_APU_FAST_NMS[] = {{10, 1, 10, 0, {3, 3, 4, 4}}, {10, 1, 4, 3, {1, 1, 2, 2}}, {10, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_APU_FAST_NMS[] = {{12, 1, 10, 0, {3, 3, 4, 4}}, {12, 1, 4, 3, {1, 1, 2, 2}}, {12, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_APU_FAST_NMS[] = {{14, 1, 10, 0, {3, 3, 4, 4}}, {14, 1, 4, 3, {1, 1, 2, 2}}, {14, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_APU_FAST_NMS[] = {{16, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 4, 3, {1, 1, 2, 2}}, {16, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_APU_FAST_NMS[] = {{18, 1, 10, 0, {3, 3, 4, 4}}, {18, 1, 4, 3, {1, 1, 2, 2}}, {18, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_APU_FAST_NMS[] = {{20, 1, 10, 0, {3, 3, 4, 4}}, {20, 1, 4, 3, {1, 1, 2, 2}}, {20, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_APU_FAST_NMS[] = {{22, 1, 10, 0, {3, 3, 4, 4}}, {22, 1, 4, 3, {1, 1, 2, 2}}, {22, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_APU_FAST_NMS[] = {{24, 1, 10, 0, {3, 3, 4, 4}}, {24, 1, 4, 3, {1, 1, 2, 2}}, {24, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_APU_FAST_NMS[] = {{26, 1, 10, 0, {3, 3, 4, 4}}, {26, 1, 4, 3, {1, 1, 2, 2}}, {26, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_APU_FAST_NMS[] = {{28, 1, 10, 0, {3, 3, 4, 4}}, {28, 1, 4, 3, {1, 1, 2, 2}}, {28, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_APU_FAST_NMS[] = {{30, 1, 10, 0, {3, 3, 4, 4}}, {30, 1, 4, 3, {1, 1, 2, 2}}, {30, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_APU_FAST_NMS[] = {{32, 1, 10, 0, {3, 3, 4, 4}}, {32, 1, 4, 3, {1, 1, 2, 2}}, {32, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_APU_FAST_NMS[] = {{36, 1, 10, 0, {3, 3, 4, 4}}, {36, 1, 4, 3, {1, 1, 2, 2}}, {36, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_APU_FAST_NMS[] = {{40, 1, 10, 0, {3, 3, 4, 4}}, {40, 1, 4, 3, {1, 1, 2, 2}}, {40, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_APU_FAST_NMS[] = {{44, 1, 10, 0, {3, 3, 4, 4}}, {44, 1, 4, 3, {1, 1, 2, 2}}, {44, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_APU_FAST_NMS[] = {{48, 1, 10, 0, {3, 3, 4, 4}}, {48, 1, 4, 3, {1, 1, 2, 2}}, {48, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_APU_FAST_NMS[] = {{52, 1, 10, 0, {3, 3, 4, 4}}, {52, 1, 4, 3, {1, 1, 2, 2}}, {52, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_APU_FAST_NMS[] = {{56, 1, 10, 0, {3, 3, 4, 4}}, {56, 1, 4, 3, {1, 1, 2, 2}}, {56, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_APU_FAST_NMS[] = {{60, 1, 10, 0, {3, 3, 4, 4}}, {60, 1, 4, 3, {1, 1, 2, 2}}, {60, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_APU_FAST_NMS[] = {{64, 1, 10, 0, {3, 3, 4, 4}}, {64, 1, 4, 3, {1, 1, 2, 2}}, {64, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_APU_FAST_NMS[] = {{72, 1, 10, 0, {3, 3, 4, 4}}, {72, 1, 4, 3, {1, 1, 2, 2}}, {72, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_APU_FAST_NMS[] = {{80, 1, 10, 0, {3, 3, 4, 4}}, {80, 1, 4, 3, {1, 1, 2, 2}}, {80, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_APU_FAST_NMS[] = {{88, 1, 10, 0, {3, 3, 4, 4}}, {88, 1, 4, 3, {1, 1, 2, 2}}, {88, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_APU_FAST_NMS[] = {{96, 1, 10, 0, {3, 3, 4, 4}}, {96, 1, 4, 3, {1, 1, 2, 2}}, {96, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_APU_FAST_NMS[] = {{104, 1, 10, 0, {3, 3, 4, 4}}, {104, 1, 4, 3, {1, 1, 2, 2}}, {104, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_APU_FAST_NMS[] = {{112, 1, 10, 0, {3, 3, 4, 4}}, {112, 1, 4, 3, {1, 1, 2, 2}}, {112, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_APU_FAST_NMS[] = {{120, 1, 10, 0, {3, 3, 4, 4}}, {120, 1, 4, 3, {1, 1, 2, 2}}, {120, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_APU_FAST_NMS[] = {{128, 1, 10, 0, {3, 3, 4, 4}}, {128, 1, 4, 3, {1, 1, 2, 2}}, {128, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_APU_FAST_NMS[] = {{4, 2, 7, 0, {3, 3, 4, 4}}, {4, 2, 4, 2, {1, 1, 2, 2}}, {4, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_APU_FAST_NMS[] = {{6, 2, 7, 0, {3, 3, 4, 4}}, {6, 2, 4, 2, {1, 1, 2, 2}}, {6, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_APU_FAST_NMS[] = {{8, 2, 7, 0, {3, 3, 4, 4}}, {8, 2, 4, 2, {1, 1, 2, 2}}, {8, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_APU_FAST_NMS[] = {{10, 2, 7, 0, {3, 3, 4, 4}}, {10, 2, 4, 2, {1, 1, 2, 2}}, {10, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_APU_FAST_NMS[] = {{12, 2, 7, 0, {3, 3, 4, 4}}, {12, 2, 4, 2, {1, 1, 2, 2}}, {12, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_APU_FAST_NMS[] = {{14, 2, 7, 0, {3, 3, 4, 4}}, {14, 2, 4, 2, {1, 1, 2, 2}}, {14, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_APU_FAST_NMS[] = {{16, 2, 7, 0, {3, 3, 4, 4}}, {16, 2, 4, 2, {1, 1, 2, 2}}, {16, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_APU_FAST_NMS[] = {{18, 2, 7, 0, {3, 3, 4, 4}}, {18, 2, 4, 2, {1, 1, 2, 2}}, {18, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_APU_FAST_NMS[] = {{20, 2, 7, 0, {3, 3, 4, 4}}, {20, 2, 4, 2, {1, 1, 2, 2}}, {20, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_APU_FAST_NMS[] = {{22, 2, 7, 0, {3, 3, 4, 4}}, {22, 2, 4, 2, {1, 1, 2, 2}}, {22, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_APU_FAST_NMS[] = {{24, 2, 7, 0, {3, 3, 4, 4}}, {24, 2, 4, 2, {1, 1, 2, 2}}, {24, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_APU_FAST_NMS[] = {{26, 2, 7, 0, {3, 3, 4, 4}}, {26, 2, 4, 2, {1, 1, 2, 2}}, {26, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_APU_FAST_NMS[] = {{28, 2, 7, 0, {3, 3, 4, 4}}, {28, 2, 4, 2, {1, 1, 2, 2}}, {28, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_APU_FAST_NMS[] = {{30, 2, 7, 0, {3, 3, 4, 4}}, {30, 2, 4, 2, {1, 1, 2, 2}}, {30, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_APU_FAST_NMS[] = {{32, 2, 7, 0, {3, 3, 4, 4}}, {32, 2, 4, 2, {1, 1, 2, 2}}, {32, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_APU_FAST_NMS[] = {{36, 2, 7, 0, {3, 3, 4, 4}}, {36, 2, 4, 2, {1, 1, 2, 2}}, {36, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_APU_FAST_NMS[] = {{40, 2, 7, 0, {3, 3, 4, 4}}, {40, 2, 4, 2, {1, 1, 2, 2}}, {40, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_APU_FAST_NMS[] = {{44, 2, 7, 0, {3, 3, 4, 4}}, {44, 2, 4, 2, {1, 1, 2, 2}}, {44, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_APU_FAST_NMS[] = {{48, 2, 7, 0, {3, 3, 4, 4}}, {48, 2, 4, 2, {1, 1, 2, 2}}, {48, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_APU_FAST_NMS[] = {{52, 2, 7, 0, {3, 3, 4, 4}}, {52, 2, 4, 2, {1, 1, 2, 2}}, {52, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_APU_FAST_NMS[] = {{56, 2, 7, 0, {3, 3, 4, 4}}, {56, 2, 4, 2, {1, 1, 2, 2}}, {56, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_APU_FAST_NMS[] = {{60, 2, 7, 0, {3, 3, 4, 4}}, {60, 2, 4, 2, {1, 1, 2, 2}}, {60, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_APU_FAST_NMS[] = {{64, 2, 7, 0, {3, 3, 4, 4}}, {64, 2, 4, 2, {1, 1, 2, 2}}, {64, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_APU_FAST_NMS[] = {{72, 2, 7, 0, {3, 3, 4, 4}}, {72, 2, 4, 2, {1, 1, 2, 2}}, {72, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_APU_FAST_NMS[] = {{80, 2, 7, 0, {3, 3, 4, 4}}, {80, 2, 4, 2, {1, 1, 2, 2}}, {80, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_APU_FAST_NMS[] = {{88, 2, 7, 0, {3, 3, 4, 4}}, {88, 2, 4, 2, {1, 1, 2, 2}}, {88, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_APU_FAST_NMS[] = {{96, 2, 7, 0, {3, 3, 4, 4}}, {96, 2, 4, 2, {1, 1, 2, 2}}, {96, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_APU_FAST_NMS[] = {{104, 2, 7, 0, {3, 3, 4, 4}}, {104, 2, 4, 2, {1, 1, 2, 2}}, {104, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_APU_FAST_NMS[] = {{4, 3, 4, 0, {3, 3, 4, 4}}, {4, 3, 4, 1, {1, 1, 2, 2}}, {4, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_APU_FAST_NMS[] = {{6, 3, 4, 0, {3, 3, 4, 4}}, {6, 3, 4, 1, {1, 1, 2, 2}}, {6, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_APU_FAST_NMS[] = {{8, 3, 4, 0, {3, 3, 4, 4}}, {8, 3, 4, 1, {1, 1, 2, 2}}, {8, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_APU_FAST_NMS[] = {{10, 3, 4, 0, {3, 3, 4, 4}}, {10, 3, 4, 1, {1, 1, 2, 2}}, {10, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_APU_FAST_NMS[] = {{12, 3, 4, 0, {3, 3, 4, 4}}, {12, 3, 4, 1, {1, 1, 2, 2}}, {12, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_APU_FAST_NMS[] = {{14, 3, 4, 0, {3, 3, 4, 4}}, {14, 3, 4, 1, {1, 1, 2, 2}}, {14, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_APU_FAST_NMS[] = {{16, 3, 4, 0, {3, 3, 4, 4}}, {16, 3, 4, 1, {1, 1, 2, 2}}, {16, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_APU_FAST_NMS[] = {{18, 3, 4, 0, {3, 3, 4, 4}}, {18, 3, 4, 1, {1, 1, 2, 2}}, {18, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_APU_FAST_NMS[] = {{20, 3, 4, 0, {3, 3, 4, 4}}, {20, 3, 4, 1, {1, 1, 2, 2}}, {20, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_APU_FAST_NMS[] = {{22, 3, 4, 0, {3, 3, 4, 4}}, {22, 3, 4, 1, {1, 1, 2, 2}}, {22, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_APU_FAST_NMS[] = {{24, 3, 4, 0, {3, 3, 4, 4}}, {24, 3, 4, 1, {1, 1, 2, 2}}, {24, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_APU_FAST_NMS[] = {{26, 3, 4, 0, {3, 3, 4, 4}}, {26, 3, 4, 1, {1, 1, 2, 2}}, {26, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_APU_FAST_NMS[] = {{28, 3, 4, 0, {3, 3, 4, 4}}, {28, 3, 4, 1, {1, 1, 2, 2}}, {28, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_APU_FAST_NMS[] = {{30, 3, 4, 0, {3, 3, 4, 4}}, {30, 3, 4, 1, {1, 1, 2, 2}}, {30, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_APU_FAST_NMS[] = {{32, 3, 4, 0, {3, 3, 4, 4}}, {32, 3, 4, 1, {1, 1, 2, 2}}, {32, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_APU_FAST_NMS[] = {{36, 3, 4, 0, {3, 3, 4, 4}}, {36, 3, 4, 1, {1, 1, 2, 2}}, {36, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_APU_FAST_NMS[] = {{40, 3, 4, 0, {3, 3, 4, 4}}, {40, 3, 4, 1, {1, 1, 2, 2}}, {40, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_APU_FAST_NMS[] = {{44, 3, 4, 0, {3, 3, 4, 4}}, {44, 3, 4, 1, {1, 1, 2, 2}}, {44, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_APU_FAST_NMS[] = {{48, 3, 4, 0, {3, 3, 4, 4}}, {48, 3, 4, 1, {1, 1, 2, 2}}, {48, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_APU_FAST_NMS[] = {{52, 3, 4, 0, {3, 3, 4, 4}}, {52, 3, 4, 1, {1, 1, 2, 2}}, {52, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_APU_FAST_NMS[] = {{56, 3, 4, 0, {3, 3, 4, 4}}, {56, 3, 4, 1, {1, 1, 2, 2}}, {56, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_APU_FAST_NMS[] = {{60, 3, 4, 0, {3, 3, 4, 4}}, {60, 3, 4, 1, {1, 1, 2, 2}}, {60, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_APU_FAST_NMS[] = {{64, 3, 4, 0, {3, 3, 4, 4}}, {64, 3, 4, 1, {1, 1, 2, 2}}, {64, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_APU_FAST_NMS[] = {{72, 3, 4, 0, {3, 3, 4, 4}}, {72, 3, 4, 1, {1, 1, 2, 2}}, {72, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_APU_FAST_NMS[] = {{80, 3, 4, 0, {3, 3, 4, 4}}, {80, 3, 4, 1, {1, 1, 2, 2}}, {80, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_APU_FAST_NMS[] = {{88, 3, 4, 0, {3, 3, 4, 4}}, {88, 3, 4, 1, {1, 1, 2, 2}}, {88, 3, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_APU_FAST_NMS[] = {{4, 4, 4, 0, {3, 3, 4, 4}}, {4, 4, 4, 1, {1, 1, 2, 2}}, {4, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_APU_FAST_NMS[] = {{6, 4, 4, 0, {3, 3, 4, 4}}, {6, 4, 4, 1, {1, 1, 2, 2}}, {6, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_APU_FAST_NMS[] = {{8, 4, 4, 0, {3, 3, 4, 4}}, {8, 4, 4, 1, {1, 1, 2, 2}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_APU_FAST_NMS[] = {{10, 4, 4, 0, {3, 3, 4, 4}}, {10, 4, 4, 1, {1, 1, 2, 2}}, {10, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_APU_FAST_NMS[] = {{12, 4, 4, 0, {3, 3, 4, 4}}, {12, 4, 4, 1, {1, 1, 2, 2}}, {12, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_APU_FAST_NMS[] = {{14, 4, 4, 0, {3, 3, 4, 4}}, {14, 4, 4, 1, {1, 1, 2, 2}}, {14, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_APU_FAST_NMS[] = {{16, 4, 4, 0, {3, 3, 4, 4}}, {16, 4, 4, 1, {1, 1, 2, 2}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_APU_FAST_NMS[] = {{18, 4, 4, 0, {3, 3, 4, 4}}, {18, 4, 4, 1, {1, 1, 2, 2}}, {18, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_APU_FAST_NMS[] = {{20, 4, 4, 0, {3, 3, 4, 4}}, {20, 4, 4, 1, {1, 1, 2, 2}}, {20, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_APU_FAST_NMS[] = {{22, 4, 4, 0, {3, 3, 4, 4}}, {22, 4, 4, 1, {1, 1, 2, 2}}, {22, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_APU_FAST_NMS[] = {{24, 4, 4, 0, {3, 3, 4, 4}}, {24, 4, 4, 1, {1, 1, 2, 2}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_APU_FAST_NMS[] = {{26, 4, 4, 0, {3, 3, 4, 4}}, {26, 4, 4, 1, {1, 1, 2, 2}}, {26, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_APU_FAST_NMS[] = {{28, 4, 4, 0, {3, 3, 4, 4}}, {28, 4, 4, 1, {1, 1, 2, 2}}, {28, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_APU_FAST_NMS[] = {{30, 4, 4, 0, {3, 3, 4, 4}}, {30, 4, 4, 1, {1, 1, 2, 2}}, {30, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_APU_FAST_NMS[] = {{32, 4, 4, 0, {3, 3, 4, 4}}, {32, 4, 4, 1, {1, 1, 2, 2}}, {32, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_APU_FAST_NMS[] = {{36, 4, 4, 0, {3, 3, 4, 4}}, {36, 4, 4, 1, {1, 1, 2, 2}}, {36, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_APU_FAST_NMS[] = {{40, 4, 4, 0, {3, 3, 4, 4}}, {40, 4, 4, 1, {1, 1, 2, 2}}, {40, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_APU_FAST_NMS[] = {{44, 4, 4, 0, {3, 3, 4, 4}}, {44, 4, 4, 1, {1, 1, 2, 2}}, {44, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_APU_FAST_NMS[] = {{48, 4, 4, 0, {3, 3, 4, 4}}, {48, 4, 4, 1, {1, 1, 2, 2}}, {48, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_APU_FAST_NMS[] = {{52, 4, 4, 0, {3, 3, 4, 4}}, {52, 4, 4, 1, {1, 1, 2, 2}}, {52, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_APU_FAST_NMS[] = {{56, 4, 4, 0, {3, 3, 4, 4}}, {56, 4, 4, 1, {1, 1, 2, 2}}, {56, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_APU_FAST_NMS[] = {{60, 4, 4, 0, {3, 3, 4, 4}}, {60, 4, 4, 1, {1, 1, 2, 2}}, {60, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_APU_FAST_NMS[] = {{64, 4, 4, 0, {3, 3, 4, 4}}, {64, 4, 4, 1, {1, 1, 2, 2}}, {64, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_APU_FAST_NMS[] = {{72, 4, 4, 0, {3, 3, 4, 4}}, {72, 4, 4, 1, {1, 1, 2, 2}}, {72, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_APU_FAST_NMS[] = {{4, 5, 4, 0, {3, 3, 4, 4}}, {4, 5, 4, 1, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_APU_FAST_NMS[] = {{6, 5, 4, 0, {3, 3, 4, 4}}, {6, 5, 4, 1, {1, 1, 2, 2}}, {6, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_APU_FAST_NMS[] = {{8, 5, 4, 0, {3, 3, 4, 4}}, {8, 5, 4, 1, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_APU_FAST_NMS[] = {{10, 5, 4, 0, {3, 3, 4, 4}}, {10, 5, 4, 1, {1, 1, 2, 2}}, {10, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_APU_FAST_NMS[] = {{12, 5, 4, 0, {3, 3, 4, 4}}, {12, 5, 4, 1, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_APU_FAST_NMS[] = {{14, 5, 4, 0, {3, 3, 4, 4}}, {14, 5, 4, 1, {1, 1, 2, 2}}, {14, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_APU_FAST_NMS[] = {{16, 5, 4, 0, {3, 3, 4, 4}}, {16, 5, 4, 1, {1, 1, 2, 2}}, {16, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_APU_FAST_NMS[] = {{18, 5, 4, 0, {3, 3, 4, 4}}, {18, 5, 4, 1, {1, 1, 2, 2}}, {18, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_APU_FAST_NMS[] = {{20, 5, 4, 0, {3, 3, 4, 4}}, {20, 5, 4, 1, {1, 1, 2, 2}}, {20, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_APU_FAST_NMS[] = {{22, 5, 4, 0, {3, 3, 4, 4}}, {22, 5, 4, 1, {1, 1, 2, 2}}, {22, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_APU_FAST_NMS[] = {{24, 5, 4, 0, {3, 3, 4, 4}}, {24, 5, 4, 1, {1, 1, 2, 2}}, {24, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_APU_FAST_NMS[] = {{26, 5, 4, 0, {3, 3, 4, 4}}, {26, 5, 4, 1, {1, 1, 2, 2}}, {26, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_APU_FAST_NMS[] = {{28, 5, 4, 0, {3, 3, 4, 4}}, {28, 5, 4, 1, {1, 1, 2, 2}}, {28, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_APU_FAST_NMS[] = {{30, 5, 4, 0, {3, 3, 4, 4}}, {30, 5, 4, 1, {1, 1, 2, 2}}, {30, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_APU_FAST_NMS[] = {{32, 5, 4, 0, {3, 3, 4, 4}}, {32, 5, 4, 1, {1, 1, 2, 2}}, {32, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_APU_FAST_NMS[] = {{36, 5, 4, 0, {3, 3, 4, 4}}, {36, 5, 4, 1, {1, 1, 2, 2}}, {36, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_APU_FAST_NMS[] = {{40, 5, 4, 0, {3, 3, 4, 4}}, {40, 5, 4, 1, {1, 1, 2, 2}}, {40, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_APU_FAST_NMS[] = {{44, 5, 4, 0, {3, 3, 4, 4}}, {44, 5, 4, 1, {1, 1, 2, 2}}, {44, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_APU_FAST_NMS[] = {{48, 5, 4, 0, {3, 3, 4, 4}}, {48, 5, 4, 1, {1, 1, 2, 2}}, {48, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_APU_FAST_NMS[] = {{52, 5, 4, 0, {3, 3, 4, 4}}, {52, 5, 4, 1, {1, 1, 2, 2}}, {52, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_APU_FAST_NMS[] = {{56, 5, 4, 0, {3, 3, 4, 4}}, {56, 5, 4, 1, {1, 1, 2, 2}}, {56, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_APU_FAST_NMS[] = {{60, 5, 4, 0, {3, 3, 4, 4}}, {60, 5, 4, 1, {1, 1, 2, 2}}, {60, 5, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_APU_FAST_NMS[] = {{4, 6, 4, 0, {3, 3, 4, 4}}, {4, 6, 4, 1, {1, 1, 2, 2}}, {4, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_APU_FAST_NMS[] = {{6, 6, 4, 0, {3, 3, 4, 4}}, {6, 6, 4, 1, {1, 1, 2, 2}}, {6, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_APU_FAST_NMS[] = {{8, 6, 4, 0, {3, 3, 4, 4}}, {8, 6, 4, 1, {1, 1, 2, 2}}, {8, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_APU_FAST_NMS[] = {{10, 6, 4, 0, {3, 3, 4, 4}}, {10, 6, 4, 1, {1, 1, 2, 2}}, {10, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_APU_FAST_NMS[] = {{12, 6, 4, 0, {3, 3, 4, 4}}, {12, 6, 4, 1, {1, 1, 2, 2}}, {12, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_APU_FAST_NMS[] = {{14, 6, 4, 0, {3, 3, 4, 4}}, {14, 6, 4, 1, {1, 1, 2, 2}}, {14, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_APU_FAST_NMS[] = {{16, 6, 4, 0, {3, 3, 4, 4}}, {16, 6, 4, 1, {1, 1, 2, 2}}, {16, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_APU_FAST_NMS[] = {{18, 6, 4, 0, {3, 3, 4, 4}}, {18, 6, 4, 1, {1, 1, 2, 2}}, {18, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_APU_FAST_NMS[] = {{20, 6, 4, 0, {3, 3, 4, 4}}, {20, 6, 4, 1, {1, 1, 2, 2}}, {20, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_APU_FAST_NMS[] = {{22, 6, 4, 0, {3, 3, 4, 4}}, {22, 6, 4, 1, {1, 1, 2, 2}}, {22, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_APU_FAST_NMS[] = {{24, 6, 4, 0, {3, 3, 4, 4}}, {24, 6, 4, 1, {1, 1, 2, 2}}, {24, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_APU_FAST_NMS[] = {{26, 6, 4, 0, {3, 3, 4, 4}}, {26, 6, 4, 1, {1, 1, 2, 2}}, {26, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_APU_FAST_NMS[] = {{28, 6, 4, 0, {3, 3, 4, 4}}, {28, 6, 4, 1, {1, 1, 2, 2}}, {28, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_APU_FAST_NMS[] = {{30, 6, 4, 0, {3, 3, 4, 4}}, {30, 6, 4, 1, {1, 1, 2, 2}}, {30, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_APU_FAST_NMS[] = {{32, 6, 4, 0, {3, 3, 4, 4}}, {32, 6, 4, 1, {1, 1, 2, 2}}, {32, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_APU_FAST_NMS[] = {{36, 6, 4, 0, {3, 3, 4, 4}}, {36, 6, 4, 1, {1, 1, 2, 2}}, {36, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_APU_FAST_NMS[] = {{40, 6, 4, 0, {3, 3, 4, 4}}, {40, 6, 4, 1, {1, 1, 2, 2}}, {40, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_APU_FAST_NMS[] = {{44, 6, 4, 0, {3, 3, 4, 4}}, {44, 6, 4, 1, {1, 1, 2, 2}}, {44, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_APU_FAST_NMS[] = {{48, 6, 4, 0, {3, 3, 4, 4}}, {48, 6, 4, 1, {1, 1, 2, 2}}, {48, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_APU_FAST_NMS[] = {{4, 8, 4, 0, {3, 3, 4, 4}}, {4, 8, 4, 1, {1, 1, 2, 2}}, {4, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_APU_FAST_NMS[] = {{6, 8, 4, 0, {3, 3, 4, 4}}, {6, 8, 4, 1, {1, 1, 2, 2}}, {6, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_APU_FAST_NMS[] = {{8, 8, 4, 0, {3, 3, 4, 4}}, {8, 8, 4, 1, {1, 1, 2, 2}}, {8, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_APU_FAST_NMS[] = {{10, 8, 4, 0, {3, 3, 4, 4}}, {10, 8, 4, 1, {1, 1, 2, 2}}, {10, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_APU_FAST_NMS[] = {{12, 8, 4, 0, {3, 3, 4, 4}}, {12, 8, 4, 1, {1, 1, 2, 2}}, {12, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_APU_FAST_NMS[] = {{14, 8, 4, 0, {3, 3, 4, 4}}, {14, 8, 4, 1, {1, 1, 2, 2}}, {14, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_APU_FAST_NMS[] = {{16, 8, 4, 0, {3, 3, 4, 4}}, {16, 8, 4, 1, {1, 1, 2, 2}}, {16, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_APU_FAST_NMS[] = {{18, 8, 4, 0, {3, 3, 4, 4}}, {18, 8, 4, 1, {1, 1, 2, 2}}, {18, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_APU_FAST_NMS[] = {{20, 8, 4, 0, {3, 3, 4, 4}}, {20, 8, 4, 1, {1, 1, 2, 2}}, {20, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_APU_FAST_NMS[] = {{22, 8, 4, 0, {3, 3, 4, 4}}, {22, 8, 4, 1, {1, 1, 2, 2}}, {22, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_APU_FAST_NMS[] = {{24, 8, 4, 0, {3, 3, 4, 4}}, {24, 8, 4, 1, {1, 1, 2, 2}}, {24, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_APU_FAST_NMS[] = {{26, 8, 4, 0, {3, 3, 4, 4}}, {26, 8, 4, 1, {1, 1, 2, 2}}, {26, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_APU_FAST_NMS[] = {{28, 8, 4, 0, {3, 3, 4, 4}}, {28, 8, 4, 1, {1, 1, 2, 2}}, {28, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_APU_FAST_NMS[] = {{30, 8, 4, 0, {3, 3, 4, 4}}, {30, 8, 4, 1, {1, 1, 2, 2}}, {30, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_APU_FAST_NMS[] = {{32, 8, 4, 0, {3, 3, 4, 4}}, {32, 8, 4, 1, {1, 1, 2, 2}}, {32, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_APU_FAST_NMS[] = {{36, 8, 4, 0, {3, 3, 4, 4}}, {36, 8, 4, 1, {1, 1, 2, 2}}, {36, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_APU_FAST_NMS[] = {{4, 10, 4, 0, {3, 3, 4, 4}}, {4, 10, 4, 1, {1, 1, 2, 2}}, {4, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_APU_FAST_NMS[] = {{6, 10, 4, 0, {3, 3, 4, 4}}, {6, 10, 4, 1, {1, 1, 2, 2}}, {6, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_APU_FAST_NMS[] = {{8, 10, 4, 0, {3, 3, 4, 4}}, {8, 10, 4, 1, {1, 1, 2, 2}}, {8, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_APU_FAST_NMS[] = {{10, 10, 4, 0, {3, 3, 4, 4}}, {10, 10, 4, 1, {1, 1, 2, 2}}, {10, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_APU_FAST_NMS[] = {{12, 10, 4, 0, {3, 3, 4, 4}}, {12, 10, 4, 1, {1, 1, 2, 2}}, {12, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_APU_FAST_NMS[] = {{14, 10, 4, 0, {3, 3, 4, 4}}, {14, 10, 4, 1, {1, 1, 2, 2}}, {14, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_APU_FAST_NMS[] = {{16, 10, 4, 0, {3, 3, 4, 4}}, {16, 10, 4, 1, {1, 1, 2, 2}}, {16, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_APU_FAST_NMS[] = {{18, 10, 4, 0, {3, 3, 4, 4}}, {18, 10, 4, 1, {1, 1, 2, 2}}, {18, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_APU_FAST_NMS[] = {{20, 10, 4, 0, {3, 3, 4, 4}}, {20, 10, 4, 1, {1, 1, 2, 2}}, {20, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_APU_FAST_NMS[] = {{22, 10, 4, 0, {3, 3, 4, 4}}, {22, 10, 4, 1, {1, 1, 2, 2}}, {22, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_APU_FAST_NMS[] = {{24, 10, 4, 0, {3, 3, 4, 4}}, {24, 10, 4, 1, {1, 1, 2, 2}}, {24, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_APU_FAST_NMS[] = {{26, 10, 4, 0, {3, 3, 4, 4}}, {26, 10, 4, 1, {1, 1, 2, 2}}, {26, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_APU_FAST_NMS[] = {{28, 10, 4, 0, {3, 3, 4, 4}}, {28, 10, 4, 1, {1, 1, 2, 2}}, {28, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_APU_FAST_NMS[] = {{30, 10, 4, 0, {3, 3, 4, 4}}, {30, 10, 4, 1, {1, 1, 2, 2}}, {30, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_APU_FAST_NMS[] = {{4, 12, 4, 0, {3, 3, 4, 4}}, {4, 12, 4, 1, {1, 1, 2, 2}}, {4, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_APU_FAST_NMS[] = {{6, 12, 4, 0, {3, 3, 4, 4}}, {6, 12, 4, 1, {1, 1, 2, 2}}, {6, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_APU_FAST_NMS[] = {{8, 12, 4, 0, {3, 3, 4, 4}}, {8, 12, 4, 1, {1, 1, 2, 2}}, {8, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_APU_FAST_NMS[] = {{10, 12, 4, 0, {3, 3, 4, 4}}, {10, 12, 4, 1, {1, 1, 2, 2}}, {10, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_APU_FAST_NMS[] = {{12, 12, 4, 0, {3, 3, 4, 4}}, {12, 12, 4, 1, {1, 1, 2, 2}}, {12, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_APU_FAST_NMS[] = {{14, 12, 4, 0, {3, 3, 4, 4}}, {14, 12, 4, 1, {1, 1, 2, 2}}, {14, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_APU_FAST_NMS[] = {{16, 12, 4, 0, {3, 3, 4, 4}}, {16, 12, 4, 1, {1, 1, 2, 2}}, {16, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_APU_FAST_NMS[] = {{18, 12, 4, 0, {3, 3, 4, 4}}, {18, 12, 4, 1, {1, 1, 2, 2}}, {18, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_APU_FAST_NMS[] = {{20, 12, 4, 0, {3, 3, 4, 4}}, {20, 12, 4, 1, {1, 1, 2, 2}}, {20, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_APU_FAST_NMS[] = {{22, 12, 4, 0, {3, 3, 4, 4}}, {22, 12, 4, 1, {1, 1, 2, 2}}, {22, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_APU_FAST_NMS[] = {{24, 12, 4, 0, {3, 3, 4, 4}}, {24, 12, 4, 1, {1, 1, 2, 2}}, {24, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_APU_FAST_NMS[] = {{4, 14, 4, 0, {3, 3, 4, 4}}, {4, 14, 4, 1, {1, 1, 2, 2}}, {4, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_APU_FAST_NMS[] = {{6, 14, 4, 0, {3, 3, 4, 4}}, {6, 14, 4, 1, {1, 1, 2, 2}}, {6, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_APU_FAST_NMS[] = {{8, 14, 4, 0, {3, 3, 4, 4}}, {8, 14, 4, 1, {1, 1, 2, 2}}, {8, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_APU_FAST_NMS[] = {{10, 14, 4, 0, {3, 3, 4, 4}}, {10, 14, 4, 1, {1, 1, 2, 2}}, {10, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_APU_FAST_NMS[] = {{12, 14, 4, 0, {3, 3, 4, 4}}, {12, 14, 4, 1, {1, 1, 2, 2}}, {12, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_APU_FAST_NMS[] = {{14, 14, 4, 0, {3, 3, 4, 4}}, {14, 14, 4, 1, {1, 1, 2, 2}}, {14, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_APU_FAST_NMS[] = {{16, 14, 4, 0, {3, 3, 4, 4}}, {16, 14, 4, 1, {1, 1, 2, 2}}, {16, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_APU_FAST_NMS[] = {{18, 14, 4, 0, {3, 3, 4, 4}}, {18, 14, 4, 1, {1, 1, 2, 2}}, {18, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_APU_FAST_NMS[] = {{20, 14, 4, 0, {3, 3, 4, 4}}, {20, 14, 4, 1, {1, 1, 2, 2}}, {20, 14, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_APU_FAST_NMS[] = {{4, 16, 4, 0, {3, 3, 4, 4}}, {4, 16, 4, 1, {1, 1, 2, 2}}, {4, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_APU_FAST_NMS[] = {{6, 16, 4, 0, {3, 3, 4, 4}}, {6, 16, 4, 1, {1, 1, 2, 2}}, {6, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_APU_FAST_NMS[] = {{8, 16, 4, 0, {3, 3, 4, 4}}, {8, 16, 4, 1, {1, 1, 2, 2}}, {8, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_APU_FAST_NMS[] = {{10, 16, 4, 0, {3, 3, 4, 4}}, {10, 16, 4, 1, {1, 1, 2, 2}}, {10, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_APU_FAST_NMS[] = {{12, 16, 4, 0, {3, 3, 4, 4}}, {12, 16, 4, 1, {1, 1, 2, 2}}, {12, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_APU_FAST_NMS[] = {{14, 16, 4, 0, {3, 3, 4, 4}}, {14, 16, 4, 1, {1, 1, 2, 2}}, {14, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_APU_FAST_NMS[] = {{16, 16, 4, 0, {3, 3, 4, 4}}, {16, 16, 4, 1, {1, 1, 2, 2}}, {16, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_APU_FAST_NMS[] = {{4, 18, 4, 0, {3, 3, 4, 4}}, {4, 18, 4, 1, {1, 1, 2, 2}}, {4, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_APU_FAST_NMS[] = {{6, 18, 4, 0, {3, 3, 4, 4}}, {6, 18, 4, 1, {1, 1, 2, 2}}, {6, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_APU_FAST_NMS[] = {{8, 18, 4, 0, {3, 3, 4, 4}}, {8, 18, 4, 1, {1, 1, 2, 2}}, {8, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_APU_FAST_NMS[] = {{10, 18, 4, 0, {3, 3, 4, 4}}, {10, 18, 4, 1, {1, 1, 2, 2}}, {10, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_APU_FAST_NMS[] = {{12, 18, 4, 0, {3, 3, 4, 4}}, {12, 18, 4, 1, {1, 1, 2, 2}}, {12, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_APU_FAST_NMS[] = {{14, 18, 4, 0, {3, 3, 4, 4}}, {14, 18, 4, 1, {1, 1, 2, 2}}, {14, 18, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_APU_FAST_NMS[] = {{4, 20, 4, 0, {3, 3, 4, 4}}, {4, 20, 4, 1, {1, 1, 2, 2}}, {4, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_APU_FAST_NMS[] = {{6, 20, 4, 0, {3, 3, 4, 4}}, {6, 20, 4, 1, {1, 1, 2, 2}}, {6, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_APU_FAST_NMS[] = {{8, 20, 4, 0, {3, 3, 4, 4}}, {8, 20, 4, 1, {1, 1, 2, 2}}, {8, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_APU_FAST_NMS[] = {{10, 20, 4, 0, {3, 3, 4, 4}}, {10, 20, 4, 1, {1, 1, 2, 2}}, {10, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_APU_FAST_NMS[] = {{12, 20, 4, 0, {3, 3, 4, 4}}, {12, 20, 4, 1, {1, 1, 2, 2}}, {12, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_APU_FAST_NMS[] = {{4, 22, 4, 0, {3, 3, 4, 4}}, {4, 22, 4, 1, {1, 1, 2, 2}}, {4, 22, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_APU_FAST_NMS[] = {{6, 22, 4, 0, {3, 3, 4, 4}}, {6, 22, 4, 1, {1, 1, 2, 2}}, {6, 22, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_APU_FAST_NMS[] = {{8, 22, 4, 0, {3, 3, 4, 4}}, {8, 22, 4, 1, {1, 1, 2, 2}}, {8, 22, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_APU_FAST_NMS[] = {{10, 22, 4, 0, {3, 3, 4, 4}}, {10, 22, 4, 1, {1, 1, 2, 2}}, {10, 22, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_APU_FAST_NMS[] = {{4, 24, 4, 0, {3, 3, 4, 4}}, {4, 24, 4, 1, {1, 1, 2, 2}}, {4, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_APU_FAST_NMS[] = {{6, 24, 4, 0, {3, 3, 4, 4}}, {6, 24, 4, 1, {1, 1, 2, 2}}, {6, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_APU_FAST_NMS[] = {{8, 24, 4, 0, {3, 3, 4, 4}}, {8, 24, 4, 1, {1, 1, 2, 2}}, {8, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_APU_FAST_NMS[] = {{10, 24, 4, 0, {3, 3, 4, 4}}, {10, 24, 4, 1, {1, 1, 2, 2}}, {10, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_APU_FAST_NMS[] = {{4, 26, 4, 0, {3, 3, 4, 4}}, {4, 26, 4, 1, {1, 1, 2, 2}}, {4, 26, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_APU_FAST_NMS[] = {{6, 26, 4, 0, {3, 3, 4, 4}}, {6, 26, 4, 1, {1, 1, 2, 2}}, {6, 26, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_APU_FAST_NMS[] = {{8, 26, 4, 0, {3, 3, 4, 4}}, {8, 26, 4, 1, {1, 1, 2, 2}}, {8, 26, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_APU_FAST_NMS[] = {{4, 28, 4, 0, {3, 3, 4, 4}}, {4, 28, 4, 1, {1, 1, 2, 2}}, {4, 28, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_APU_FAST_NMS[] = {{6, 28, 4, 0, {3, 3, 4, 4}}, {6, 28, 4, 1, {1, 1, 2, 2}}, {6, 28, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_APU_FAST_NMS[] = {{8, 28, 4, 0, {3, 3, 4, 4}}, {8, 28, 4, 1, {1, 1, 2, 2}}, {8, 28, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_APU_FAST_NMS[] = {{4, 30, 4, 0, {3, 3, 4, 4}}, {4, 30, 4, 1, {1, 1, 2, 2}}, {4, 30, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_APU_FAST_NMS[] = {{6, 30, 4, 0, {3, 3, 4, 4}}, {6, 30, 4, 1, {1, 1, 2, 2}}, {6, 30, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_APU_FAST_NMS[] = {{4, 32, 4, 0, {3, 3, 4, 4}}, {4, 32, 4, 1, {1, 1, 2, 2}}, {4, 32, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_APU_FAST_NMS[] = {{6, 32, 4, 0, {3, 3, 4, 4}}, {6, 32, 4, 1, {1, 1, 2, 2}}, {6, 32, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_APU_FAST_NMS[] = {{3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_APU_FAST_NMS[] = {{2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_APU_FAST_NMS[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_APU_FAST_NMS[] = {{1, 0}, {2, 0}};

static acf_scenario gScenarioArray_APU_FAST_NMS[] = {
{4, 1, 248, 16, 4, gScenarioBufferData0_APU_FAST_NMS, 64, gScenarioKernelData0_APU_FAST_NMS, 8},
{6, 1, 296, 16, 4, gScenarioBufferData1_APU_FAST_NMS, 64, gScenarioKernelData1_APU_FAST_NMS, 8},
{8, 1, 344, 16, 4, gScenarioBufferData2_APU_FAST_NMS, 64, gScenarioKernelData2_APU_FAST_NMS, 8},
{10, 1, 392, 16, 4, gScenarioBufferData3_APU_FAST_NMS, 64, gScenarioKernelData3_APU_FAST_NMS, 8},
{12, 1, 440, 16, 4, gScenarioBufferData4_APU_FAST_NMS, 64, gScenarioKernelData4_APU_FAST_NMS, 8},
{14, 1, 488, 16, 4, gScenarioBufferData5_APU_FAST_NMS, 64, gScenarioKernelData5_APU_FAST_NMS, 8},
{16, 1, 536, 16, 4, gScenarioBufferData6_APU_FAST_NMS, 64, gScenarioKernelData6_APU_FAST_NMS, 8},
{18, 1, 584, 16, 4, gScenarioBufferData7_APU_FAST_NMS, 64, gScenarioKernelData7_APU_FAST_NMS, 8},
{20, 1, 632, 16, 4, gScenarioBufferData8_APU_FAST_NMS, 64, gScenarioKernelData8_APU_FAST_NMS, 8},
{22, 1, 680, 16, 4, gScenarioBufferData9_APU_FAST_NMS, 64, gScenarioKernelData9_APU_FAST_NMS, 8},
{24, 1, 728, 16, 4, gScenarioBufferData10_APU_FAST_NMS, 64, gScenarioKernelData10_APU_FAST_NMS, 8},
{26, 1, 776, 16, 4, gScenarioBufferData11_APU_FAST_NMS, 64, gScenarioKernelData11_APU_FAST_NMS, 8},
{28, 1, 824, 16, 4, gScenarioBufferData12_APU_FAST_NMS, 64, gScenarioKernelData12_APU_FAST_NMS, 8},
{30, 1, 872, 16, 4, gScenarioBufferData13_APU_FAST_NMS, 64, gScenarioKernelData13_APU_FAST_NMS, 8},
{32, 1, 920, 16, 4, gScenarioBufferData14_APU_FAST_NMS, 64, gScenarioKernelData14_APU_FAST_NMS, 8},
{36, 1, 1016, 16, 4, gScenarioBufferData15_APU_FAST_NMS, 64, gScenarioKernelData15_APU_FAST_NMS, 8},
{40, 1, 1112, 16, 4, gScenarioBufferData16_APU_FAST_NMS, 64, gScenarioKernelData16_APU_FAST_NMS, 8},
{44, 1, 1208, 16, 4, gScenarioBufferData17_APU_FAST_NMS, 64, gScenarioKernelData17_APU_FAST_NMS, 8},
{48, 1, 1304, 16, 4, gScenarioBufferData18_APU_FAST_NMS, 64, gScenarioKernelData18_APU_FAST_NMS, 8},
{52, 1, 1400, 16, 4, gScenarioBufferData19_APU_FAST_NMS, 64, gScenarioKernelData19_APU_FAST_NMS, 8},
{56, 1, 1496, 16, 4, gScenarioBufferData20_APU_FAST_NMS, 64, gScenarioKernelData20_APU_FAST_NMS, 8},
{60, 1, 1592, 16, 4, gScenarioBufferData21_APU_FAST_NMS, 64, gScenarioKernelData21_APU_FAST_NMS, 8},
{64, 1, 1688, 16, 4, gScenarioBufferData22_APU_FAST_NMS, 64, gScenarioKernelData22_APU_FAST_NMS, 8},
{72, 1, 1880, 16, 4, gScenarioBufferData23_APU_FAST_NMS, 64, gScenarioKernelData23_APU_FAST_NMS, 8},
{80, 1, 2072, 16, 4, gScenarioBufferData24_APU_FAST_NMS, 64, gScenarioKernelData24_APU_FAST_NMS, 8},
{88, 1, 2264, 16, 4, gScenarioBufferData25_APU_FAST_NMS, 64, gScenarioKernelData25_APU_FAST_NMS, 8},
{96, 1, 2456, 16, 4, gScenarioBufferData26_APU_FAST_NMS, 64, gScenarioKernelData26_APU_FAST_NMS, 8},
{104, 1, 2648, 16, 4, gScenarioBufferData27_APU_FAST_NMS, 64, gScenarioKernelData27_APU_FAST_NMS, 8},
{112, 1, 2840, 16, 4, gScenarioBufferData28_APU_FAST_NMS, 64, gScenarioKernelData28_APU_FAST_NMS, 8},
{120, 1, 3032, 16, 4, gScenarioBufferData29_APU_FAST_NMS, 64, gScenarioKernelData29_APU_FAST_NMS, 8},
{128, 1, 3224, 16, 4, gScenarioBufferData30_APU_FAST_NMS, 64, gScenarioKernelData30_APU_FAST_NMS, 8},
{4, 2, 336, 16, 3, gScenarioBufferData31_APU_FAST_NMS, 64, gScenarioKernelData31_APU_FAST_NMS, 8},
{6, 2, 404, 16, 3, gScenarioBufferData32_APU_FAST_NMS, 64, gScenarioKernelData32_APU_FAST_NMS, 8},
{8, 2, 472, 16, 3, gScenarioBufferData33_APU_FAST_NMS, 64, gScenarioKernelData33_APU_FAST_NMS, 8},
{10, 2, 540, 16, 3, gScenarioBufferData34_APU_FAST_NMS, 64, gScenarioKernelData34_APU_FAST_NMS, 8},
{12, 2, 608, 16, 3, gScenarioBufferData35_APU_FAST_NMS, 64, gScenarioKernelData35_APU_FAST_NMS, 8},
{14, 2, 676, 16, 3, gScenarioBufferData36_APU_FAST_NMS, 64, gScenarioKernelData36_APU_FAST_NMS, 8},
{16, 2, 744, 16, 3, gScenarioBufferData37_APU_FAST_NMS, 64, gScenarioKernelData37_APU_FAST_NMS, 8},
{18, 2, 812, 16, 3, gScenarioBufferData38_APU_FAST_NMS, 64, gScenarioKernelData38_APU_FAST_NMS, 8},
{20, 2, 880, 16, 3, gScenarioBufferData39_APU_FAST_NMS, 64, gScenarioKernelData39_APU_FAST_NMS, 8},
{22, 2, 948, 16, 3, gScenarioBufferData40_APU_FAST_NMS, 64, gScenarioKernelData40_APU_FAST_NMS, 8},
{24, 2, 1016, 16, 3, gScenarioBufferData41_APU_FAST_NMS, 64, gScenarioKernelData41_APU_FAST_NMS, 8},
{26, 2, 1084, 16, 3, gScenarioBufferData42_APU_FAST_NMS, 64, gScenarioKernelData42_APU_FAST_NMS, 8},
{28, 2, 1152, 16, 3, gScenarioBufferData43_APU_FAST_NMS, 64, gScenarioKernelData43_APU_FAST_NMS, 8},
{30, 2, 1220, 16, 3, gScenarioBufferData44_APU_FAST_NMS, 64, gScenarioKernelData44_APU_FAST_NMS, 8},
{32, 2, 1288, 16, 3, gScenarioBufferData45_APU_FAST_NMS, 64, gScenarioKernelData45_APU_FAST_NMS, 8},
{36, 2, 1424, 16, 3, gScenarioBufferData46_APU_FAST_NMS, 64, gScenarioKernelData46_APU_FAST_NMS, 8},
{40, 2, 1560, 16, 3, gScenarioBufferData47_APU_FAST_NMS, 64, gScenarioKernelData47_APU_FAST_NMS, 8},
{44, 2, 1696, 16, 3, gScenarioBufferData48_APU_FAST_NMS, 64, gScenarioKernelData48_APU_FAST_NMS, 8},
{48, 2, 1832, 16, 3, gScenarioBufferData49_APU_FAST_NMS, 64, gScenarioKernelData49_APU_FAST_NMS, 8},
{52, 2, 1968, 16, 3, gScenarioBufferData50_APU_FAST_NMS, 64, gScenarioKernelData50_APU_FAST_NMS, 8},
{56, 2, 2104, 16, 3, gScenarioBufferData51_APU_FAST_NMS, 64, gScenarioKernelData51_APU_FAST_NMS, 8},
{60, 2, 2240, 16, 3, gScenarioBufferData52_APU_FAST_NMS, 64, gScenarioKernelData52_APU_FAST_NMS, 8},
{64, 2, 2376, 16, 3, gScenarioBufferData53_APU_FAST_NMS, 64, gScenarioKernelData53_APU_FAST_NMS, 8},
{72, 2, 2648, 16, 3, gScenarioBufferData54_APU_FAST_NMS, 64, gScenarioKernelData54_APU_FAST_NMS, 8},
{80, 2, 2920, 16, 3, gScenarioBufferData55_APU_FAST_NMS, 64, gScenarioKernelData55_APU_FAST_NMS, 8},
{88, 2, 3192, 16, 3, gScenarioBufferData56_APU_FAST_NMS, 64, gScenarioKernelData56_APU_FAST_NMS, 8},
{96, 2, 3464, 16, 3, gScenarioBufferData57_APU_FAST_NMS, 64, gScenarioKernelData57_APU_FAST_NMS, 8},
{104, 2, 3736, 16, 3, gScenarioBufferData58_APU_FAST_NMS, 64, gScenarioKernelData58_APU_FAST_NMS, 8},
{4, 3, 352, 16, 2, gScenarioBufferData59_APU_FAST_NMS, 64, gScenarioKernelData59_APU_FAST_NMS, 8},
{6, 3, 428, 16, 2, gScenarioBufferData60_APU_FAST_NMS, 64, gScenarioKernelData60_APU_FAST_NMS, 8},
{8, 3, 504, 16, 2, gScenarioBufferData61_APU_FAST_NMS, 64, gScenarioKernelData61_APU_FAST_NMS, 8},
{10, 3, 580, 16, 2, gScenarioBufferData62_APU_FAST_NMS, 64, gScenarioKernelData62_APU_FAST_NMS, 8},
{12, 3, 656, 16, 2, gScenarioBufferData63_APU_FAST_NMS, 64, gScenarioKernelData63_APU_FAST_NMS, 8},
{14, 3, 732, 16, 2, gScenarioBufferData64_APU_FAST_NMS, 64, gScenarioKernelData64_APU_FAST_NMS, 8},
{16, 3, 808, 16, 2, gScenarioBufferData65_APU_FAST_NMS, 64, gScenarioKernelData65_APU_FAST_NMS, 8},
{18, 3, 884, 16, 2, gScenarioBufferData66_APU_FAST_NMS, 64, gScenarioKernelData66_APU_FAST_NMS, 8},
{20, 3, 960, 16, 2, gScenarioBufferData67_APU_FAST_NMS, 64, gScenarioKernelData67_APU_FAST_NMS, 8},
{22, 3, 1036, 16, 2, gScenarioBufferData68_APU_FAST_NMS, 64, gScenarioKernelData68_APU_FAST_NMS, 8},
{24, 3, 1112, 16, 2, gScenarioBufferData69_APU_FAST_NMS, 64, gScenarioKernelData69_APU_FAST_NMS, 8},
{26, 3, 1188, 16, 2, gScenarioBufferData70_APU_FAST_NMS, 64, gScenarioKernelData70_APU_FAST_NMS, 8},
{28, 3, 1264, 16, 2, gScenarioBufferData71_APU_FAST_NMS, 64, gScenarioKernelData71_APU_FAST_NMS, 8},
{30, 3, 1340, 16, 2, gScenarioBufferData72_APU_FAST_NMS, 64, gScenarioKernelData72_APU_FAST_NMS, 8},
{32, 3, 1416, 16, 2, gScenarioBufferData73_APU_FAST_NMS, 64, gScenarioKernelData73_APU_FAST_NMS, 8},
{36, 3, 1568, 16, 2, gScenarioBufferData74_APU_FAST_NMS, 64, gScenarioKernelData74_APU_FAST_NMS, 8},
{40, 3, 1720, 16, 2, gScenarioBufferData75_APU_FAST_NMS, 64, gScenarioKernelData75_APU_FAST_NMS, 8},
{44, 3, 1872, 16, 2, gScenarioBufferData76_APU_FAST_NMS, 64, gScenarioKernelData76_APU_FAST_NMS, 8},
{48, 3, 2024, 16, 2, gScenarioBufferData77_APU_FAST_NMS, 64, gScenarioKernelData77_APU_FAST_NMS, 8},
{52, 3, 2176, 16, 2, gScenarioBufferData78_APU_FAST_NMS, 64, gScenarioKernelData78_APU_FAST_NMS, 8},
{56, 3, 2328, 16, 2, gScenarioBufferData79_APU_FAST_NMS, 64, gScenarioKernelData79_APU_FAST_NMS, 8},
{60, 3, 2480, 16, 2, gScenarioBufferData80_APU_FAST_NMS, 64, gScenarioKernelData80_APU_FAST_NMS, 8},
{64, 3, 2632, 16, 2, gScenarioBufferData81_APU_FAST_NMS, 64, gScenarioKernelData81_APU_FAST_NMS, 8},
{72, 3, 2936, 16, 2, gScenarioBufferData82_APU_FAST_NMS, 64, gScenarioKernelData82_APU_FAST_NMS, 8},
{80, 3, 3240, 16, 2, gScenarioBufferData83_APU_FAST_NMS, 64, gScenarioKernelData83_APU_FAST_NMS, 8},
{88, 3, 3544, 16, 2, gScenarioBufferData84_APU_FAST_NMS, 64, gScenarioKernelData84_APU_FAST_NMS, 8},
{4, 4, 440, 16, 2, gScenarioBufferData85_APU_FAST_NMS, 64, gScenarioKernelData85_APU_FAST_NMS, 8},
{6, 4, 536, 16, 2, gScenarioBufferData86_APU_FAST_NMS, 64, gScenarioKernelData86_APU_FAST_NMS, 8},
{8, 4, 632, 16, 2, gScenarioBufferData87_APU_FAST_NMS, 64, gScenarioKernelData87_APU_FAST_NMS, 8},
{10, 4, 728, 16, 2, gScenarioBufferData88_APU_FAST_NMS, 64, gScenarioKernelData88_APU_FAST_NMS, 8},
{12, 4, 824, 16, 2, gScenarioBufferData89_APU_FAST_NMS, 64, gScenarioKernelData89_APU_FAST_NMS, 8},
{14, 4, 920, 16, 2, gScenarioBufferData90_APU_FAST_NMS, 64, gScenarioKernelData90_APU_FAST_NMS, 8},
{16, 4, 1016, 16, 2, gScenarioBufferData91_APU_FAST_NMS, 64, gScenarioKernelData91_APU_FAST_NMS, 8},
{18, 4, 1112, 16, 2, gScenarioBufferData92_APU_FAST_NMS, 64, gScenarioKernelData92_APU_FAST_NMS, 8},
{20, 4, 1208, 16, 2, gScenarioBufferData93_APU_FAST_NMS, 64, gScenarioKernelData93_APU_FAST_NMS, 8},
{22, 4, 1304, 16, 2, gScenarioBufferData94_APU_FAST_NMS, 64, gScenarioKernelData94_APU_FAST_NMS, 8},
{24, 4, 1400, 16, 2, gScenarioBufferData95_APU_FAST_NMS, 64, gScenarioKernelData95_APU_FAST_NMS, 8},
{26, 4, 1496, 16, 2, gScenarioBufferData96_APU_FAST_NMS, 64, gScenarioKernelData96_APU_FAST_NMS, 8},
{28, 4, 1592, 16, 2, gScenarioBufferData97_APU_FAST_NMS, 64, gScenarioKernelData97_APU_FAST_NMS, 8},
{30, 4, 1688, 16, 2, gScenarioBufferData98_APU_FAST_NMS, 64, gScenarioKernelData98_APU_FAST_NMS, 8},
{32, 4, 1784, 16, 2, gScenarioBufferData99_APU_FAST_NMS, 64, gScenarioKernelData99_APU_FAST_NMS, 8},
{36, 4, 1976, 16, 2, gScenarioBufferData100_APU_FAST_NMS, 64, gScenarioKernelData100_APU_FAST_NMS, 8},
{40, 4, 2168, 16, 2, gScenarioBufferData101_APU_FAST_NMS, 64, gScenarioKernelData101_APU_FAST_NMS, 8},
{44, 4, 2360, 16, 2, gScenarioBufferData102_APU_FAST_NMS, 64, gScenarioKernelData102_APU_FAST_NMS, 8},
{48, 4, 2552, 16, 2, gScenarioBufferData103_APU_FAST_NMS, 64, gScenarioKernelData103_APU_FAST_NMS, 8},
{52, 4, 2744, 16, 2, gScenarioBufferData104_APU_FAST_NMS, 64, gScenarioKernelData104_APU_FAST_NMS, 8},
{56, 4, 2936, 16, 2, gScenarioBufferData105_APU_FAST_NMS, 64, gScenarioKernelData105_APU_FAST_NMS, 8},
{60, 4, 3128, 16, 2, gScenarioBufferData106_APU_FAST_NMS, 64, gScenarioKernelData106_APU_FAST_NMS, 8},
{64, 4, 3320, 16, 2, gScenarioBufferData107_APU_FAST_NMS, 64, gScenarioKernelData107_APU_FAST_NMS, 8},
{72, 4, 3704, 16, 2, gScenarioBufferData108_APU_FAST_NMS, 64, gScenarioKernelData108_APU_FAST_NMS, 8},
{4, 5, 528, 16, 2, gScenarioBufferData109_APU_FAST_NMS, 64, gScenarioKernelData109_APU_FAST_NMS, 8},
{6, 5, 644, 16, 2, gScenarioBufferData110_APU_FAST_NMS, 64, gScenarioKernelData110_APU_FAST_NMS, 8},
{8, 5, 760, 16, 2, gScenarioBufferData111_APU_FAST_NMS, 64, gScenarioKernelData111_APU_FAST_NMS, 8},
{10, 5, 876, 16, 2, gScenarioBufferData112_APU_FAST_NMS, 64, gScenarioKernelData112_APU_FAST_NMS, 8},
{12, 5, 992, 16, 2, gScenarioBufferData113_APU_FAST_NMS, 64, gScenarioKernelData113_APU_FAST_NMS, 8},
{14, 5, 1108, 16, 2, gScenarioBufferData114_APU_FAST_NMS, 64, gScenarioKernelData114_APU_FAST_NMS, 8},
{16, 5, 1224, 16, 2, gScenarioBufferData115_APU_FAST_NMS, 64, gScenarioKernelData115_APU_FAST_NMS, 8},
{18, 5, 1340, 16, 2, gScenarioBufferData116_APU_FAST_NMS, 64, gScenarioKernelData116_APU_FAST_NMS, 8},
{20, 5, 1456, 16, 2, gScenarioBufferData117_APU_FAST_NMS, 64, gScenarioKernelData117_APU_FAST_NMS, 8},
{22, 5, 1572, 16, 2, gScenarioBufferData118_APU_FAST_NMS, 64, gScenarioKernelData118_APU_FAST_NMS, 8},
{24, 5, 1688, 16, 2, gScenarioBufferData119_APU_FAST_NMS, 64, gScenarioKernelData119_APU_FAST_NMS, 8},
{26, 5, 1804, 16, 2, gScenarioBufferData120_APU_FAST_NMS, 64, gScenarioKernelData120_APU_FAST_NMS, 8},
{28, 5, 1920, 16, 2, gScenarioBufferData121_APU_FAST_NMS, 64, gScenarioKernelData121_APU_FAST_NMS, 8},
{30, 5, 2036, 16, 2, gScenarioBufferData122_APU_FAST_NMS, 64, gScenarioKernelData122_APU_FAST_NMS, 8},
{32, 5, 2152, 16, 2, gScenarioBufferData123_APU_FAST_NMS, 64, gScenarioKernelData123_APU_FAST_NMS, 8},
{36, 5, 2384, 16, 2, gScenarioBufferData124_APU_FAST_NMS, 64, gScenarioKernelData124_APU_FAST_NMS, 8},
{40, 5, 2616, 16, 2, gScenarioBufferData125_APU_FAST_NMS, 64, gScenarioKernelData125_APU_FAST_NMS, 8},
{44, 5, 2848, 16, 2, gScenarioBufferData126_APU_FAST_NMS, 64, gScenarioKernelData126_APU_FAST_NMS, 8},
{48, 5, 3080, 16, 2, gScenarioBufferData127_APU_FAST_NMS, 64, gScenarioKernelData127_APU_FAST_NMS, 8},
{52, 5, 3312, 16, 2, gScenarioBufferData128_APU_FAST_NMS, 64, gScenarioKernelData128_APU_FAST_NMS, 8},
{56, 5, 3544, 16, 2, gScenarioBufferData129_APU_FAST_NMS, 64, gScenarioKernelData129_APU_FAST_NMS, 8},
{60, 5, 3776, 16, 2, gScenarioBufferData130_APU_FAST_NMS, 64, gScenarioKernelData130_APU_FAST_NMS, 8},
{4, 6, 616, 16, 2, gScenarioBufferData131_APU_FAST_NMS, 64, gScenarioKernelData131_APU_FAST_NMS, 8},
{6, 6, 752, 16, 2, gScenarioBufferData132_APU_FAST_NMS, 64, gScenarioKernelData132_APU_FAST_NMS, 8},
{8, 6, 888, 16, 2, gScenarioBufferData133_APU_FAST_NMS, 64, gScenarioKernelData133_APU_FAST_NMS, 8},
{10, 6, 1024, 16, 2, gScenarioBufferData134_APU_FAST_NMS, 64, gScenarioKernelData134_APU_FAST_NMS, 8},
{12, 6, 1160, 16, 2, gScenarioBufferData135_APU_FAST_NMS, 64, gScenarioKernelData135_APU_FAST_NMS, 8},
{14, 6, 1296, 16, 2, gScenarioBufferData136_APU_FAST_NMS, 64, gScenarioKernelData136_APU_FAST_NMS, 8},
{16, 6, 1432, 16, 2, gScenarioBufferData137_APU_FAST_NMS, 64, gScenarioKernelData137_APU_FAST_NMS, 8},
{18, 6, 1568, 16, 2, gScenarioBufferData138_APU_FAST_NMS, 64, gScenarioKernelData138_APU_FAST_NMS, 8},
{20, 6, 1704, 16, 2, gScenarioBufferData139_APU_FAST_NMS, 64, gScenarioKernelData139_APU_FAST_NMS, 8},
{22, 6, 1840, 16, 2, gScenarioBufferData140_APU_FAST_NMS, 64, gScenarioKernelData140_APU_FAST_NMS, 8},
{24, 6, 1976, 16, 2, gScenarioBufferData141_APU_FAST_NMS, 64, gScenarioKernelData141_APU_FAST_NMS, 8},
{26, 6, 2112, 16, 2, gScenarioBufferData142_APU_FAST_NMS, 64, gScenarioKernelData142_APU_FAST_NMS, 8},
{28, 6, 2248, 16, 2, gScenarioBufferData143_APU_FAST_NMS, 64, gScenarioKernelData143_APU_FAST_NMS, 8},
{30, 6, 2384, 16, 2, gScenarioBufferData144_APU_FAST_NMS, 64, gScenarioKernelData144_APU_FAST_NMS, 8},
{32, 6, 2520, 16, 2, gScenarioBufferData145_APU_FAST_NMS, 64, gScenarioKernelData145_APU_FAST_NMS, 8},
{36, 6, 2792, 16, 2, gScenarioBufferData146_APU_FAST_NMS, 64, gScenarioKernelData146_APU_FAST_NMS, 8},
{40, 6, 3064, 16, 2, gScenarioBufferData147_APU_FAST_NMS, 64, gScenarioKernelData147_APU_FAST_NMS, 8},
{44, 6, 3336, 16, 2, gScenarioBufferData148_APU_FAST_NMS, 64, gScenarioKernelData148_APU_FAST_NMS, 8},
{48, 6, 3608, 16, 2, gScenarioBufferData149_APU_FAST_NMS, 64, gScenarioKernelData149_APU_FAST_NMS, 8},
{4, 8, 792, 16, 2, gScenarioBufferData150_APU_FAST_NMS, 64, gScenarioKernelData150_APU_FAST_NMS, 8},
{6, 8, 968, 16, 2, gScenarioBufferData151_APU_FAST_NMS, 64, gScenarioKernelData151_APU_FAST_NMS, 8},
{8, 8, 1144, 16, 2, gScenarioBufferData152_APU_FAST_NMS, 64, gScenarioKernelData152_APU_FAST_NMS, 8},
{10, 8, 1320, 16, 2, gScenarioBufferData153_APU_FAST_NMS, 64, gScenarioKernelData153_APU_FAST_NMS, 8},
{12, 8, 1496, 16, 2, gScenarioBufferData154_APU_FAST_NMS, 64, gScenarioKernelData154_APU_FAST_NMS, 8},
{14, 8, 1672, 16, 2, gScenarioBufferData155_APU_FAST_NMS, 64, gScenarioKernelData155_APU_FAST_NMS, 8},
{16, 8, 1848, 16, 2, gScenarioBufferData156_APU_FAST_NMS, 64, gScenarioKernelData156_APU_FAST_NMS, 8},
{18, 8, 2024, 16, 2, gScenarioBufferData157_APU_FAST_NMS, 64, gScenarioKernelData157_APU_FAST_NMS, 8},
{20, 8, 2200, 16, 2, gScenarioBufferData158_APU_FAST_NMS, 64, gScenarioKernelData158_APU_FAST_NMS, 8},
{22, 8, 2376, 16, 2, gScenarioBufferData159_APU_FAST_NMS, 64, gScenarioKernelData159_APU_FAST_NMS, 8},
{24, 8, 2552, 16, 2, gScenarioBufferData160_APU_FAST_NMS, 64, gScenarioKernelData160_APU_FAST_NMS, 8},
{26, 8, 2728, 16, 2, gScenarioBufferData161_APU_FAST_NMS, 64, gScenarioKernelData161_APU_FAST_NMS, 8},
{28, 8, 2904, 16, 2, gScenarioBufferData162_APU_FAST_NMS, 64, gScenarioKernelData162_APU_FAST_NMS, 8},
{30, 8, 3080, 16, 2, gScenarioBufferData163_APU_FAST_NMS, 64, gScenarioKernelData163_APU_FAST_NMS, 8},
{32, 8, 3256, 16, 2, gScenarioBufferData164_APU_FAST_NMS, 64, gScenarioKernelData164_APU_FAST_NMS, 8},
{36, 8, 3608, 16, 2, gScenarioBufferData165_APU_FAST_NMS, 64, gScenarioKernelData165_APU_FAST_NMS, 8},
{4, 10, 968, 16, 2, gScenarioBufferData166_APU_FAST_NMS, 64, gScenarioKernelData166_APU_FAST_NMS, 8},
{6, 10, 1184, 16, 2, gScenarioBufferData167_APU_FAST_NMS, 64, gScenarioKernelData167_APU_FAST_NMS, 8},
{8, 10, 1400, 16, 2, gScenarioBufferData168_APU_FAST_NMS, 64, gScenarioKernelData168_APU_FAST_NMS, 8},
{10, 10, 1616, 16, 2, gScenarioBufferData169_APU_FAST_NMS, 64, gScenarioKernelData169_APU_FAST_NMS, 8},
{12, 10, 1832, 16, 2, gScenarioBufferData170_APU_FAST_NMS, 64, gScenarioKernelData170_APU_FAST_NMS, 8},
{14, 10, 2048, 16, 2, gScenarioBufferData171_APU_FAST_NMS, 64, gScenarioKernelData171_APU_FAST_NMS, 8},
{16, 10, 2264, 16, 2, gScenarioBufferData172_APU_FAST_NMS, 64, gScenarioKernelData172_APU_FAST_NMS, 8},
{18, 10, 2480, 16, 2, gScenarioBufferData173_APU_FAST_NMS, 64, gScenarioKernelData173_APU_FAST_NMS, 8},
{20, 10, 2696, 16, 2, gScenarioBufferData174_APU_FAST_NMS, 64, gScenarioKernelData174_APU_FAST_NMS, 8},
{22, 10, 2912, 16, 2, gScenarioBufferData175_APU_FAST_NMS, 64, gScenarioKernelData175_APU_FAST_NMS, 8},
{24, 10, 3128, 16, 2, gScenarioBufferData176_APU_FAST_NMS, 64, gScenarioKernelData176_APU_FAST_NMS, 8},
{26, 10, 3344, 16, 2, gScenarioBufferData177_APU_FAST_NMS, 64, gScenarioKernelData177_APU_FAST_NMS, 8},
{28, 10, 3560, 16, 2, gScenarioBufferData178_APU_FAST_NMS, 64, gScenarioKernelData178_APU_FAST_NMS, 8},
{30, 10, 3776, 16, 2, gScenarioBufferData179_APU_FAST_NMS, 64, gScenarioKernelData179_APU_FAST_NMS, 8},
{4, 12, 1144, 16, 2, gScenarioBufferData180_APU_FAST_NMS, 64, gScenarioKernelData180_APU_FAST_NMS, 8},
{6, 12, 1400, 16, 2, gScenarioBufferData181_APU_FAST_NMS, 64, gScenarioKernelData181_APU_FAST_NMS, 8},
{8, 12, 1656, 16, 2, gScenarioBufferData182_APU_FAST_NMS, 64, gScenarioKernelData182_APU_FAST_NMS, 8},
{10, 12, 1912, 16, 2, gScenarioBufferData183_APU_FAST_NMS, 64, gScenarioKernelData183_APU_FAST_NMS, 8},
{12, 12, 2168, 16, 2, gScenarioBufferData184_APU_FAST_NMS, 64, gScenarioKernelData184_APU_FAST_NMS, 8},
{14, 12, 2424, 16, 2, gScenarioBufferData185_APU_FAST_NMS, 64, gScenarioKernelData185_APU_FAST_NMS, 8},
{16, 12, 2680, 16, 2, gScenarioBufferData186_APU_FAST_NMS, 64, gScenarioKernelData186_APU_FAST_NMS, 8},
{18, 12, 2936, 16, 2, gScenarioBufferData187_APU_FAST_NMS, 64, gScenarioKernelData187_APU_FAST_NMS, 8},
{20, 12, 3192, 16, 2, gScenarioBufferData188_APU_FAST_NMS, 64, gScenarioKernelData188_APU_FAST_NMS, 8},
{22, 12, 3448, 16, 2, gScenarioBufferData189_APU_FAST_NMS, 64, gScenarioKernelData189_APU_FAST_NMS, 8},
{24, 12, 3704, 16, 2, gScenarioBufferData190_APU_FAST_NMS, 64, gScenarioKernelData190_APU_FAST_NMS, 8},
{4, 14, 1320, 16, 2, gScenarioBufferData191_APU_FAST_NMS, 64, gScenarioKernelData191_APU_FAST_NMS, 8},
{6, 14, 1616, 16, 2, gScenarioBufferData192_APU_FAST_NMS, 64, gScenarioKernelData192_APU_FAST_NMS, 8},
{8, 14, 1912, 16, 2, gScenarioBufferData193_APU_FAST_NMS, 64, gScenarioKernelData193_APU_FAST_NMS, 8},
{10, 14, 2208, 16, 2, gScenarioBufferData194_APU_FAST_NMS, 64, gScenarioKernelData194_APU_FAST_NMS, 8},
{12, 14, 2504, 16, 2, gScenarioBufferData195_APU_FAST_NMS, 64, gScenarioKernelData195_APU_FAST_NMS, 8},
{14, 14, 2800, 16, 2, gScenarioBufferData196_APU_FAST_NMS, 64, gScenarioKernelData196_APU_FAST_NMS, 8},
{16, 14, 3096, 16, 2, gScenarioBufferData197_APU_FAST_NMS, 64, gScenarioKernelData197_APU_FAST_NMS, 8},
{18, 14, 3392, 16, 2, gScenarioBufferData198_APU_FAST_NMS, 64, gScenarioKernelData198_APU_FAST_NMS, 8},
{20, 14, 3688, 16, 2, gScenarioBufferData199_APU_FAST_NMS, 64, gScenarioKernelData199_APU_FAST_NMS, 8},
{4, 16, 1496, 16, 2, gScenarioBufferData200_APU_FAST_NMS, 64, gScenarioKernelData200_APU_FAST_NMS, 8},
{6, 16, 1832, 16, 2, gScenarioBufferData201_APU_FAST_NMS, 64, gScenarioKernelData201_APU_FAST_NMS, 8},
{8, 16, 2168, 16, 2, gScenarioBufferData202_APU_FAST_NMS, 64, gScenarioKernelData202_APU_FAST_NMS, 8},
{10, 16, 2504, 16, 2, gScenarioBufferData203_APU_FAST_NMS, 64, gScenarioKernelData203_APU_FAST_NMS, 8},
{12, 16, 2840, 16, 2, gScenarioBufferData204_APU_FAST_NMS, 64, gScenarioKernelData204_APU_FAST_NMS, 8},
{14, 16, 3176, 16, 2, gScenarioBufferData205_APU_FAST_NMS, 64, gScenarioKernelData205_APU_FAST_NMS, 8},
{16, 16, 3512, 16, 2, gScenarioBufferData206_APU_FAST_NMS, 64, gScenarioKernelData206_APU_FAST_NMS, 8},
{4, 18, 1672, 16, 2, gScenarioBufferData207_APU_FAST_NMS, 64, gScenarioKernelData207_APU_FAST_NMS, 8},
{6, 18, 2048, 16, 2, gScenarioBufferData208_APU_FAST_NMS, 64, gScenarioKernelData208_APU_FAST_NMS, 8},
{8, 18, 2424, 16, 2, gScenarioBufferData209_APU_FAST_NMS, 64, gScenarioKernelData209_APU_FAST_NMS, 8},
{10, 18, 2800, 16, 2, gScenarioBufferData210_APU_FAST_NMS, 64, gScenarioKernelData210_APU_FAST_NMS, 8},
{12, 18, 3176, 16, 2, gScenarioBufferData211_APU_FAST_NMS, 64, gScenarioKernelData211_APU_FAST_NMS, 8},
{14, 18, 3552, 16, 2, gScenarioBufferData212_APU_FAST_NMS, 64, gScenarioKernelData212_APU_FAST_NMS, 8},
{4, 20, 1848, 16, 2, gScenarioBufferData213_APU_FAST_NMS, 64, gScenarioKernelData213_APU_FAST_NMS, 8},
{6, 20, 2264, 16, 2, gScenarioBufferData214_APU_FAST_NMS, 64, gScenarioKernelData214_APU_FAST_NMS, 8},
{8, 20, 2680, 16, 2, gScenarioBufferData215_APU_FAST_NMS, 64, gScenarioKernelData215_APU_FAST_NMS, 8},
{10, 20, 3096, 16, 2, gScenarioBufferData216_APU_FAST_NMS, 64, gScenarioKernelData216_APU_FAST_NMS, 8},
{12, 20, 3512, 16, 2, gScenarioBufferData217_APU_FAST_NMS, 64, gScenarioKernelData217_APU_FAST_NMS, 8},
{4, 22, 2024, 16, 2, gScenarioBufferData218_APU_FAST_NMS, 64, gScenarioKernelData218_APU_FAST_NMS, 8},
{6, 22, 2480, 16, 2, gScenarioBufferData219_APU_FAST_NMS, 64, gScenarioKernelData219_APU_FAST_NMS, 8},
{8, 22, 2936, 16, 2, gScenarioBufferData220_APU_FAST_NMS, 64, gScenarioKernelData220_APU_FAST_NMS, 8},
{10, 22, 3392, 16, 2, gScenarioBufferData221_APU_FAST_NMS, 64, gScenarioKernelData221_APU_FAST_NMS, 8},
{4, 24, 2200, 16, 2, gScenarioBufferData222_APU_FAST_NMS, 64, gScenarioKernelData222_APU_FAST_NMS, 8},
{6, 24, 2696, 16, 2, gScenarioBufferData223_APU_FAST_NMS, 64, gScenarioKernelData223_APU_FAST_NMS, 8},
{8, 24, 3192, 16, 2, gScenarioBufferData224_APU_FAST_NMS, 64, gScenarioKernelData224_APU_FAST_NMS, 8},
{10, 24, 3688, 16, 2, gScenarioBufferData225_APU_FAST_NMS, 64, gScenarioKernelData225_APU_FAST_NMS, 8},
{4, 26, 2376, 16, 2, gScenarioBufferData226_APU_FAST_NMS, 64, gScenarioKernelData226_APU_FAST_NMS, 8},
{6, 26, 2912, 16, 2, gScenarioBufferData227_APU_FAST_NMS, 64, gScenarioKernelData227_APU_FAST_NMS, 8},
{8, 26, 3448, 16, 2, gScenarioBufferData228_APU_FAST_NMS, 64, gScenarioKernelData228_APU_FAST_NMS, 8},
{4, 28, 2552, 16, 2, gScenarioBufferData229_APU_FAST_NMS, 64, gScenarioKernelData229_APU_FAST_NMS, 8},
{6, 28, 3128, 16, 2, gScenarioBufferData230_APU_FAST_NMS, 64, gScenarioKernelData230_APU_FAST_NMS, 8},
{8, 28, 3704, 16, 2, gScenarioBufferData231_APU_FAST_NMS, 64, gScenarioKernelData231_APU_FAST_NMS, 8},
{4, 30, 2728, 16, 2, gScenarioBufferData232_APU_FAST_NMS, 64, gScenarioKernelData232_APU_FAST_NMS, 8},
{6, 30, 3344, 16, 2, gScenarioBufferData233_APU_FAST_NMS, 64, gScenarioKernelData233_APU_FAST_NMS, 8},
{4, 32, 2904, 16, 2, gScenarioBufferData234_APU_FAST_NMS, 64, gScenarioKernelData234_APU_FAST_NMS, 8},
{6, 32, 3560, 16, 2, gScenarioBufferData235_APU_FAST_NMS, 64, gScenarioKernelData235_APU_FAST_NMS, 8}
};

static acf_scenario_list gScenarioList_APU_FAST_NMS = {
236, //number of scenarios
gScenarioArray_APU_FAST_NMS};
//**************************************************************

class APU_FAST_NMS : public ACF_Process_APU
{

public:
   APU_FAST_NMS(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("APU_FAST_NMS");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("APU_FAST_NMS",
                                        APU_FAST_NMS_LOAD_SEGMENTS,
                                        APU_FAST_NMS_LOAD_PMEM, APU_FAST_NMS_LOAD_PMEM_SIZE,
                                        APU_FAST_NMS_LOAD_DMEM, APU_FAST_NMS_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(APU_FAST_NMS_APEX_LOG_BUFFER, APU_FAST_NMS_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("FAST_GR_IN_IMG", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0);
         AddPort("FAST_GR_IN_THR", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("FAST_GR_OUT_IMG", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_APU_FAST_NMS);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_APU_FAST_NMS

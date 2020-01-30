#ifndef _ACF_PROCESS_APU_BOX_9X9_08U
#define _ACF_PROCESS_APU_BOX_9X9_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <BOX_9X9_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_BOX_9X9_08U[] = {{4, 1, 13, 0, {4, 4, 0, 0}}, {4, 1, 1, 4, {0, 0, 4, 4}}, {4, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_BOX_9X9_08U[] = {{6, 1, 13, 0, {4, 4, 0, 0}}, {6, 1, 1, 4, {0, 0, 4, 4}}, {6, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_BOX_9X9_08U[] = {{8, 1, 13, 0, {4, 4, 0, 0}}, {8, 1, 1, 4, {0, 0, 4, 4}}, {8, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_BOX_9X9_08U[] = {{10, 1, 13, 0, {4, 4, 0, 0}}, {10, 1, 1, 4, {0, 0, 4, 4}}, {10, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_BOX_9X9_08U[] = {{12, 1, 13, 0, {4, 4, 0, 0}}, {12, 1, 1, 4, {0, 0, 4, 4}}, {12, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_BOX_9X9_08U[] = {{14, 1, 13, 0, {4, 4, 0, 0}}, {14, 1, 1, 4, {0, 0, 4, 4}}, {14, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_BOX_9X9_08U[] = {{16, 1, 13, 0, {4, 4, 0, 0}}, {16, 1, 1, 4, {0, 0, 4, 4}}, {16, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_BOX_9X9_08U[] = {{18, 1, 13, 0, {4, 4, 0, 0}}, {18, 1, 1, 4, {0, 0, 4, 4}}, {18, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_BOX_9X9_08U[] = {{20, 1, 13, 0, {4, 4, 0, 0}}, {20, 1, 1, 4, {0, 0, 4, 4}}, {20, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_BOX_9X9_08U[] = {{22, 1, 13, 0, {4, 4, 0, 0}}, {22, 1, 1, 4, {0, 0, 4, 4}}, {22, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_BOX_9X9_08U[] = {{24, 1, 13, 0, {4, 4, 0, 0}}, {24, 1, 1, 4, {0, 0, 4, 4}}, {24, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_BOX_9X9_08U[] = {{26, 1, 13, 0, {4, 4, 0, 0}}, {26, 1, 1, 4, {0, 0, 4, 4}}, {26, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_BOX_9X9_08U[] = {{28, 1, 13, 0, {4, 4, 0, 0}}, {28, 1, 1, 4, {0, 0, 4, 4}}, {28, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_BOX_9X9_08U[] = {{30, 1, 13, 0, {4, 4, 0, 0}}, {30, 1, 1, 4, {0, 0, 4, 4}}, {30, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_BOX_9X9_08U[] = {{32, 1, 13, 0, {4, 4, 0, 0}}, {32, 1, 1, 4, {0, 0, 4, 4}}, {32, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_BOX_9X9_08U[] = {{36, 1, 13, 0, {4, 4, 0, 0}}, {36, 1, 1, 4, {0, 0, 4, 4}}, {36, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_BOX_9X9_08U[] = {{40, 1, 13, 0, {4, 4, 0, 0}}, {40, 1, 1, 4, {0, 0, 4, 4}}, {40, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_BOX_9X9_08U[] = {{44, 1, 13, 0, {4, 4, 0, 0}}, {44, 1, 1, 4, {0, 0, 4, 4}}, {44, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_BOX_9X9_08U[] = {{48, 1, 13, 0, {4, 4, 0, 0}}, {48, 1, 1, 4, {0, 0, 4, 4}}, {48, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_BOX_9X9_08U[] = {{52, 1, 13, 0, {4, 4, 0, 0}}, {52, 1, 1, 4, {0, 0, 4, 4}}, {52, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_BOX_9X9_08U[] = {{56, 1, 13, 0, {4, 4, 0, 0}}, {56, 1, 1, 4, {0, 0, 4, 4}}, {56, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_BOX_9X9_08U[] = {{60, 1, 13, 0, {4, 4, 0, 0}}, {60, 1, 1, 4, {0, 0, 4, 4}}, {60, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_BOX_9X9_08U[] = {{64, 1, 13, 0, {4, 4, 0, 0}}, {64, 1, 1, 4, {0, 0, 4, 4}}, {64, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_BOX_9X9_08U[] = {{72, 1, 13, 0, {4, 4, 0, 0}}, {72, 1, 1, 4, {0, 0, 4, 4}}, {72, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_BOX_9X9_08U[] = {{80, 1, 13, 0, {4, 4, 0, 0}}, {80, 1, 1, 4, {0, 0, 4, 4}}, {80, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_BOX_9X9_08U[] = {{88, 1, 13, 0, {4, 4, 0, 0}}, {88, 1, 1, 4, {0, 0, 4, 4}}, {88, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_BOX_9X9_08U[] = {{96, 1, 13, 0, {4, 4, 0, 0}}, {96, 1, 1, 4, {0, 0, 4, 4}}, {96, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_BOX_9X9_08U[] = {{104, 1, 13, 0, {4, 4, 0, 0}}, {104, 1, 1, 4, {0, 0, 4, 4}}, {104, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_BOX_9X9_08U[] = {{112, 1, 13, 0, {4, 4, 0, 0}}, {112, 1, 1, 4, {0, 0, 4, 4}}, {112, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_BOX_9X9_08U[] = {{120, 1, 13, 0, {4, 4, 0, 0}}, {120, 1, 1, 4, {0, 0, 4, 4}}, {120, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_BOX_9X9_08U[] = {{128, 1, 13, 0, {4, 4, 0, 0}}, {128, 1, 1, 4, {0, 0, 4, 4}}, {128, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_BOX_9X9_08U[] = {{4, 2, 7, 0, {4, 4, 0, 0}}, {4, 2, 1, 2, {0, 0, 4, 4}}, {4, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_BOX_9X9_08U[] = {{6, 2, 7, 0, {4, 4, 0, 0}}, {6, 2, 1, 2, {0, 0, 4, 4}}, {6, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_BOX_9X9_08U[] = {{8, 2, 7, 0, {4, 4, 0, 0}}, {8, 2, 1, 2, {0, 0, 4, 4}}, {8, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_BOX_9X9_08U[] = {{10, 2, 7, 0, {4, 4, 0, 0}}, {10, 2, 1, 2, {0, 0, 4, 4}}, {10, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_BOX_9X9_08U[] = {{12, 2, 7, 0, {4, 4, 0, 0}}, {12, 2, 1, 2, {0, 0, 4, 4}}, {12, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_BOX_9X9_08U[] = {{14, 2, 7, 0, {4, 4, 0, 0}}, {14, 2, 1, 2, {0, 0, 4, 4}}, {14, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_BOX_9X9_08U[] = {{16, 2, 7, 0, {4, 4, 0, 0}}, {16, 2, 1, 2, {0, 0, 4, 4}}, {16, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_BOX_9X9_08U[] = {{18, 2, 7, 0, {4, 4, 0, 0}}, {18, 2, 1, 2, {0, 0, 4, 4}}, {18, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_BOX_9X9_08U[] = {{20, 2, 7, 0, {4, 4, 0, 0}}, {20, 2, 1, 2, {0, 0, 4, 4}}, {20, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_BOX_9X9_08U[] = {{22, 2, 7, 0, {4, 4, 0, 0}}, {22, 2, 1, 2, {0, 0, 4, 4}}, {22, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_BOX_9X9_08U[] = {{24, 2, 7, 0, {4, 4, 0, 0}}, {24, 2, 1, 2, {0, 0, 4, 4}}, {24, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_BOX_9X9_08U[] = {{26, 2, 7, 0, {4, 4, 0, 0}}, {26, 2, 1, 2, {0, 0, 4, 4}}, {26, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_BOX_9X9_08U[] = {{28, 2, 7, 0, {4, 4, 0, 0}}, {28, 2, 1, 2, {0, 0, 4, 4}}, {28, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_BOX_9X9_08U[] = {{30, 2, 7, 0, {4, 4, 0, 0}}, {30, 2, 1, 2, {0, 0, 4, 4}}, {30, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_BOX_9X9_08U[] = {{32, 2, 7, 0, {4, 4, 0, 0}}, {32, 2, 1, 2, {0, 0, 4, 4}}, {32, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_BOX_9X9_08U[] = {{36, 2, 7, 0, {4, 4, 0, 0}}, {36, 2, 1, 2, {0, 0, 4, 4}}, {36, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_BOX_9X9_08U[] = {{40, 2, 7, 0, {4, 4, 0, 0}}, {40, 2, 1, 2, {0, 0, 4, 4}}, {40, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_BOX_9X9_08U[] = {{44, 2, 7, 0, {4, 4, 0, 0}}, {44, 2, 1, 2, {0, 0, 4, 4}}, {44, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_BOX_9X9_08U[] = {{48, 2, 7, 0, {4, 4, 0, 0}}, {48, 2, 1, 2, {0, 0, 4, 4}}, {48, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_BOX_9X9_08U[] = {{52, 2, 7, 0, {4, 4, 0, 0}}, {52, 2, 1, 2, {0, 0, 4, 4}}, {52, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_BOX_9X9_08U[] = {{56, 2, 7, 0, {4, 4, 0, 0}}, {56, 2, 1, 2, {0, 0, 4, 4}}, {56, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_BOX_9X9_08U[] = {{60, 2, 7, 0, {4, 4, 0, 0}}, {60, 2, 1, 2, {0, 0, 4, 4}}, {60, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_BOX_9X9_08U[] = {{64, 2, 7, 0, {4, 4, 0, 0}}, {64, 2, 1, 2, {0, 0, 4, 4}}, {64, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_BOX_9X9_08U[] = {{72, 2, 7, 0, {4, 4, 0, 0}}, {72, 2, 1, 2, {0, 0, 4, 4}}, {72, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_BOX_9X9_08U[] = {{80, 2, 7, 0, {4, 4, 0, 0}}, {80, 2, 1, 2, {0, 0, 4, 4}}, {80, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_BOX_9X9_08U[] = {{88, 2, 7, 0, {4, 4, 0, 0}}, {88, 2, 1, 2, {0, 0, 4, 4}}, {88, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_BOX_9X9_08U[] = {{96, 2, 7, 0, {4, 4, 0, 0}}, {96, 2, 1, 2, {0, 0, 4, 4}}, {96, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_BOX_9X9_08U[] = {{104, 2, 7, 0, {4, 4, 0, 0}}, {104, 2, 1, 2, {0, 0, 4, 4}}, {104, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_BOX_9X9_08U[] = {{112, 2, 7, 0, {4, 4, 0, 0}}, {112, 2, 1, 2, {0, 0, 4, 4}}, {112, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_BOX_9X9_08U[] = {{120, 2, 7, 0, {4, 4, 0, 0}}, {120, 2, 1, 2, {0, 0, 4, 4}}, {120, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_BOX_9X9_08U[] = {{128, 2, 7, 0, {4, 4, 0, 0}}, {128, 2, 1, 2, {0, 0, 4, 4}}, {128, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_BOX_9X9_08U[] = {{4, 3, 7, 0, {4, 4, 0, 0}}, {4, 3, 1, 2, {0, 0, 4, 4}}, {4, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_BOX_9X9_08U[] = {{6, 3, 7, 0, {4, 4, 0, 0}}, {6, 3, 1, 2, {0, 0, 4, 4}}, {6, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_BOX_9X9_08U[] = {{8, 3, 7, 0, {4, 4, 0, 0}}, {8, 3, 1, 2, {0, 0, 4, 4}}, {8, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_BOX_9X9_08U[] = {{10, 3, 7, 0, {4, 4, 0, 0}}, {10, 3, 1, 2, {0, 0, 4, 4}}, {10, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_BOX_9X9_08U[] = {{12, 3, 7, 0, {4, 4, 0, 0}}, {12, 3, 1, 2, {0, 0, 4, 4}}, {12, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_BOX_9X9_08U[] = {{14, 3, 7, 0, {4, 4, 0, 0}}, {14, 3, 1, 2, {0, 0, 4, 4}}, {14, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_BOX_9X9_08U[] = {{16, 3, 7, 0, {4, 4, 0, 0}}, {16, 3, 1, 2, {0, 0, 4, 4}}, {16, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_BOX_9X9_08U[] = {{18, 3, 7, 0, {4, 4, 0, 0}}, {18, 3, 1, 2, {0, 0, 4, 4}}, {18, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_BOX_9X9_08U[] = {{20, 3, 7, 0, {4, 4, 0, 0}}, {20, 3, 1, 2, {0, 0, 4, 4}}, {20, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_BOX_9X9_08U[] = {{22, 3, 7, 0, {4, 4, 0, 0}}, {22, 3, 1, 2, {0, 0, 4, 4}}, {22, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_BOX_9X9_08U[] = {{24, 3, 7, 0, {4, 4, 0, 0}}, {24, 3, 1, 2, {0, 0, 4, 4}}, {24, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_BOX_9X9_08U[] = {{26, 3, 7, 0, {4, 4, 0, 0}}, {26, 3, 1, 2, {0, 0, 4, 4}}, {26, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_BOX_9X9_08U[] = {{28, 3, 7, 0, {4, 4, 0, 0}}, {28, 3, 1, 2, {0, 0, 4, 4}}, {28, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_BOX_9X9_08U[] = {{30, 3, 7, 0, {4, 4, 0, 0}}, {30, 3, 1, 2, {0, 0, 4, 4}}, {30, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_BOX_9X9_08U[] = {{32, 3, 7, 0, {4, 4, 0, 0}}, {32, 3, 1, 2, {0, 0, 4, 4}}, {32, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_BOX_9X9_08U[] = {{36, 3, 7, 0, {4, 4, 0, 0}}, {36, 3, 1, 2, {0, 0, 4, 4}}, {36, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_BOX_9X9_08U[] = {{40, 3, 7, 0, {4, 4, 0, 0}}, {40, 3, 1, 2, {0, 0, 4, 4}}, {40, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_BOX_9X9_08U[] = {{44, 3, 7, 0, {4, 4, 0, 0}}, {44, 3, 1, 2, {0, 0, 4, 4}}, {44, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_BOX_9X9_08U[] = {{48, 3, 7, 0, {4, 4, 0, 0}}, {48, 3, 1, 2, {0, 0, 4, 4}}, {48, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_BOX_9X9_08U[] = {{52, 3, 7, 0, {4, 4, 0, 0}}, {52, 3, 1, 2, {0, 0, 4, 4}}, {52, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_BOX_9X9_08U[] = {{56, 3, 7, 0, {4, 4, 0, 0}}, {56, 3, 1, 2, {0, 0, 4, 4}}, {56, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_BOX_9X9_08U[] = {{60, 3, 7, 0, {4, 4, 0, 0}}, {60, 3, 1, 2, {0, 0, 4, 4}}, {60, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_BOX_9X9_08U[] = {{64, 3, 7, 0, {4, 4, 0, 0}}, {64, 3, 1, 2, {0, 0, 4, 4}}, {64, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_BOX_9X9_08U[] = {{72, 3, 7, 0, {4, 4, 0, 0}}, {72, 3, 1, 2, {0, 0, 4, 4}}, {72, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_BOX_9X9_08U[] = {{80, 3, 7, 0, {4, 4, 0, 0}}, {80, 3, 1, 2, {0, 0, 4, 4}}, {80, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_BOX_9X9_08U[] = {{88, 3, 7, 0, {4, 4, 0, 0}}, {88, 3, 1, 2, {0, 0, 4, 4}}, {88, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_BOX_9X9_08U[] = {{96, 3, 7, 0, {4, 4, 0, 0}}, {96, 3, 1, 2, {0, 0, 4, 4}}, {96, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_BOX_9X9_08U[] = {{4, 4, 4, 0, {4, 4, 0, 0}}, {4, 4, 1, 1, {0, 0, 4, 4}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_BOX_9X9_08U[] = {{6, 4, 4, 0, {4, 4, 0, 0}}, {6, 4, 1, 1, {0, 0, 4, 4}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_BOX_9X9_08U[] = {{8, 4, 4, 0, {4, 4, 0, 0}}, {8, 4, 1, 1, {0, 0, 4, 4}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_BOX_9X9_08U[] = {{10, 4, 4, 0, {4, 4, 0, 0}}, {10, 4, 1, 1, {0, 0, 4, 4}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_BOX_9X9_08U[] = {{12, 4, 4, 0, {4, 4, 0, 0}}, {12, 4, 1, 1, {0, 0, 4, 4}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_BOX_9X9_08U[] = {{14, 4, 4, 0, {4, 4, 0, 0}}, {14, 4, 1, 1, {0, 0, 4, 4}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_BOX_9X9_08U[] = {{16, 4, 4, 0, {4, 4, 0, 0}}, {16, 4, 1, 1, {0, 0, 4, 4}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_BOX_9X9_08U[] = {{18, 4, 4, 0, {4, 4, 0, 0}}, {18, 4, 1, 1, {0, 0, 4, 4}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_BOX_9X9_08U[] = {{20, 4, 4, 0, {4, 4, 0, 0}}, {20, 4, 1, 1, {0, 0, 4, 4}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_BOX_9X9_08U[] = {{22, 4, 4, 0, {4, 4, 0, 0}}, {22, 4, 1, 1, {0, 0, 4, 4}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_BOX_9X9_08U[] = {{24, 4, 4, 0, {4, 4, 0, 0}}, {24, 4, 1, 1, {0, 0, 4, 4}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_BOX_9X9_08U[] = {{26, 4, 4, 0, {4, 4, 0, 0}}, {26, 4, 1, 1, {0, 0, 4, 4}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_BOX_9X9_08U[] = {{28, 4, 4, 0, {4, 4, 0, 0}}, {28, 4, 1, 1, {0, 0, 4, 4}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_BOX_9X9_08U[] = {{30, 4, 4, 0, {4, 4, 0, 0}}, {30, 4, 1, 1, {0, 0, 4, 4}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_BOX_9X9_08U[] = {{32, 4, 4, 0, {4, 4, 0, 0}}, {32, 4, 1, 1, {0, 0, 4, 4}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_BOX_9X9_08U[] = {{36, 4, 4, 0, {4, 4, 0, 0}}, {36, 4, 1, 1, {0, 0, 4, 4}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_BOX_9X9_08U[] = {{40, 4, 4, 0, {4, 4, 0, 0}}, {40, 4, 1, 1, {0, 0, 4, 4}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_BOX_9X9_08U[] = {{44, 4, 4, 0, {4, 4, 0, 0}}, {44, 4, 1, 1, {0, 0, 4, 4}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_BOX_9X9_08U[] = {{48, 4, 4, 0, {4, 4, 0, 0}}, {48, 4, 1, 1, {0, 0, 4, 4}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_BOX_9X9_08U[] = {{52, 4, 4, 0, {4, 4, 0, 0}}, {52, 4, 1, 1, {0, 0, 4, 4}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_BOX_9X9_08U[] = {{56, 4, 4, 0, {4, 4, 0, 0}}, {56, 4, 1, 1, {0, 0, 4, 4}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_BOX_9X9_08U[] = {{60, 4, 4, 0, {4, 4, 0, 0}}, {60, 4, 1, 1, {0, 0, 4, 4}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_BOX_9X9_08U[] = {{64, 4, 4, 0, {4, 4, 0, 0}}, {64, 4, 1, 1, {0, 0, 4, 4}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_BOX_9X9_08U[] = {{72, 4, 4, 0, {4, 4, 0, 0}}, {72, 4, 1, 1, {0, 0, 4, 4}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_BOX_9X9_08U[] = {{80, 4, 4, 0, {4, 4, 0, 0}}, {80, 4, 1, 1, {0, 0, 4, 4}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_BOX_9X9_08U[] = {{88, 4, 4, 0, {4, 4, 0, 0}}, {88, 4, 1, 1, {0, 0, 4, 4}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_BOX_9X9_08U[] = {{96, 4, 4, 0, {4, 4, 0, 0}}, {96, 4, 1, 1, {0, 0, 4, 4}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_BOX_9X9_08U[] = {{104, 4, 4, 0, {4, 4, 0, 0}}, {104, 4, 1, 1, {0, 0, 4, 4}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_BOX_9X9_08U[] = {{4, 5, 4, 0, {4, 4, 0, 0}}, {4, 5, 1, 1, {0, 0, 4, 4}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_BOX_9X9_08U[] = {{6, 5, 4, 0, {4, 4, 0, 0}}, {6, 5, 1, 1, {0, 0, 4, 4}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_BOX_9X9_08U[] = {{8, 5, 4, 0, {4, 4, 0, 0}}, {8, 5, 1, 1, {0, 0, 4, 4}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_BOX_9X9_08U[] = {{10, 5, 4, 0, {4, 4, 0, 0}}, {10, 5, 1, 1, {0, 0, 4, 4}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_BOX_9X9_08U[] = {{12, 5, 4, 0, {4, 4, 0, 0}}, {12, 5, 1, 1, {0, 0, 4, 4}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_BOX_9X9_08U[] = {{14, 5, 4, 0, {4, 4, 0, 0}}, {14, 5, 1, 1, {0, 0, 4, 4}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_BOX_9X9_08U[] = {{16, 5, 4, 0, {4, 4, 0, 0}}, {16, 5, 1, 1, {0, 0, 4, 4}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_BOX_9X9_08U[] = {{18, 5, 4, 0, {4, 4, 0, 0}}, {18, 5, 1, 1, {0, 0, 4, 4}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_BOX_9X9_08U[] = {{20, 5, 4, 0, {4, 4, 0, 0}}, {20, 5, 1, 1, {0, 0, 4, 4}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_BOX_9X9_08U[] = {{22, 5, 4, 0, {4, 4, 0, 0}}, {22, 5, 1, 1, {0, 0, 4, 4}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_BOX_9X9_08U[] = {{24, 5, 4, 0, {4, 4, 0, 0}}, {24, 5, 1, 1, {0, 0, 4, 4}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_BOX_9X9_08U[] = {{26, 5, 4, 0, {4, 4, 0, 0}}, {26, 5, 1, 1, {0, 0, 4, 4}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_BOX_9X9_08U[] = {{28, 5, 4, 0, {4, 4, 0, 0}}, {28, 5, 1, 1, {0, 0, 4, 4}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_BOX_9X9_08U[] = {{30, 5, 4, 0, {4, 4, 0, 0}}, {30, 5, 1, 1, {0, 0, 4, 4}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_BOX_9X9_08U[] = {{32, 5, 4, 0, {4, 4, 0, 0}}, {32, 5, 1, 1, {0, 0, 4, 4}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_BOX_9X9_08U[] = {{36, 5, 4, 0, {4, 4, 0, 0}}, {36, 5, 1, 1, {0, 0, 4, 4}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_BOX_9X9_08U[] = {{40, 5, 4, 0, {4, 4, 0, 0}}, {40, 5, 1, 1, {0, 0, 4, 4}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_BOX_9X9_08U[] = {{44, 5, 4, 0, {4, 4, 0, 0}}, {44, 5, 1, 1, {0, 0, 4, 4}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_BOX_9X9_08U[] = {{48, 5, 4, 0, {4, 4, 0, 0}}, {48, 5, 1, 1, {0, 0, 4, 4}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_BOX_9X9_08U[] = {{52, 5, 4, 0, {4, 4, 0, 0}}, {52, 5, 1, 1, {0, 0, 4, 4}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_BOX_9X9_08U[] = {{56, 5, 4, 0, {4, 4, 0, 0}}, {56, 5, 1, 1, {0, 0, 4, 4}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_BOX_9X9_08U[] = {{60, 5, 4, 0, {4, 4, 0, 0}}, {60, 5, 1, 1, {0, 0, 4, 4}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_BOX_9X9_08U[] = {{64, 5, 4, 0, {4, 4, 0, 0}}, {64, 5, 1, 1, {0, 0, 4, 4}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_BOX_9X9_08U[] = {{72, 5, 4, 0, {4, 4, 0, 0}}, {72, 5, 1, 1, {0, 0, 4, 4}}, {72, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_BOX_9X9_08U[] = {{80, 5, 4, 0, {4, 4, 0, 0}}, {80, 5, 1, 1, {0, 0, 4, 4}}, {80, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_BOX_9X9_08U[] = {{88, 5, 4, 0, {4, 4, 0, 0}}, {88, 5, 1, 1, {0, 0, 4, 4}}, {88, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_BOX_9X9_08U[] = {{4, 6, 4, 0, {4, 4, 0, 0}}, {4, 6, 1, 1, {0, 0, 4, 4}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_BOX_9X9_08U[] = {{6, 6, 4, 0, {4, 4, 0, 0}}, {6, 6, 1, 1, {0, 0, 4, 4}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_BOX_9X9_08U[] = {{8, 6, 4, 0, {4, 4, 0, 0}}, {8, 6, 1, 1, {0, 0, 4, 4}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_BOX_9X9_08U[] = {{10, 6, 4, 0, {4, 4, 0, 0}}, {10, 6, 1, 1, {0, 0, 4, 4}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_BOX_9X9_08U[] = {{12, 6, 4, 0, {4, 4, 0, 0}}, {12, 6, 1, 1, {0, 0, 4, 4}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_BOX_9X9_08U[] = {{14, 6, 4, 0, {4, 4, 0, 0}}, {14, 6, 1, 1, {0, 0, 4, 4}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_BOX_9X9_08U[] = {{16, 6, 4, 0, {4, 4, 0, 0}}, {16, 6, 1, 1, {0, 0, 4, 4}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_BOX_9X9_08U[] = {{18, 6, 4, 0, {4, 4, 0, 0}}, {18, 6, 1, 1, {0, 0, 4, 4}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_BOX_9X9_08U[] = {{20, 6, 4, 0, {4, 4, 0, 0}}, {20, 6, 1, 1, {0, 0, 4, 4}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_BOX_9X9_08U[] = {{22, 6, 4, 0, {4, 4, 0, 0}}, {22, 6, 1, 1, {0, 0, 4, 4}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_BOX_9X9_08U[] = {{24, 6, 4, 0, {4, 4, 0, 0}}, {24, 6, 1, 1, {0, 0, 4, 4}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_BOX_9X9_08U[] = {{26, 6, 4, 0, {4, 4, 0, 0}}, {26, 6, 1, 1, {0, 0, 4, 4}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_BOX_9X9_08U[] = {{28, 6, 4, 0, {4, 4, 0, 0}}, {28, 6, 1, 1, {0, 0, 4, 4}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_BOX_9X9_08U[] = {{30, 6, 4, 0, {4, 4, 0, 0}}, {30, 6, 1, 1, {0, 0, 4, 4}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_BOX_9X9_08U[] = {{32, 6, 4, 0, {4, 4, 0, 0}}, {32, 6, 1, 1, {0, 0, 4, 4}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_BOX_9X9_08U[] = {{36, 6, 4, 0, {4, 4, 0, 0}}, {36, 6, 1, 1, {0, 0, 4, 4}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_BOX_9X9_08U[] = {{40, 6, 4, 0, {4, 4, 0, 0}}, {40, 6, 1, 1, {0, 0, 4, 4}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_BOX_9X9_08U[] = {{44, 6, 4, 0, {4, 4, 0, 0}}, {44, 6, 1, 1, {0, 0, 4, 4}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_BOX_9X9_08U[] = {{48, 6, 4, 0, {4, 4, 0, 0}}, {48, 6, 1, 1, {0, 0, 4, 4}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_BOX_9X9_08U[] = {{52, 6, 4, 0, {4, 4, 0, 0}}, {52, 6, 1, 1, {0, 0, 4, 4}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_BOX_9X9_08U[] = {{56, 6, 4, 0, {4, 4, 0, 0}}, {56, 6, 1, 1, {0, 0, 4, 4}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_BOX_9X9_08U[] = {{60, 6, 4, 0, {4, 4, 0, 0}}, {60, 6, 1, 1, {0, 0, 4, 4}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_BOX_9X9_08U[] = {{64, 6, 4, 0, {4, 4, 0, 0}}, {64, 6, 1, 1, {0, 0, 4, 4}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_BOX_9X9_08U[] = {{72, 6, 4, 0, {4, 4, 0, 0}}, {72, 6, 1, 1, {0, 0, 4, 4}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_BOX_9X9_08U[] = {{4, 8, 4, 0, {4, 4, 0, 0}}, {4, 8, 1, 1, {0, 0, 4, 4}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_BOX_9X9_08U[] = {{6, 8, 4, 0, {4, 4, 0, 0}}, {6, 8, 1, 1, {0, 0, 4, 4}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_BOX_9X9_08U[] = {{8, 8, 4, 0, {4, 4, 0, 0}}, {8, 8, 1, 1, {0, 0, 4, 4}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_BOX_9X9_08U[] = {{10, 8, 4, 0, {4, 4, 0, 0}}, {10, 8, 1, 1, {0, 0, 4, 4}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_BOX_9X9_08U[] = {{12, 8, 4, 0, {4, 4, 0, 0}}, {12, 8, 1, 1, {0, 0, 4, 4}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_BOX_9X9_08U[] = {{14, 8, 4, 0, {4, 4, 0, 0}}, {14, 8, 1, 1, {0, 0, 4, 4}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_BOX_9X9_08U[] = {{16, 8, 4, 0, {4, 4, 0, 0}}, {16, 8, 1, 1, {0, 0, 4, 4}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_BOX_9X9_08U[] = {{18, 8, 4, 0, {4, 4, 0, 0}}, {18, 8, 1, 1, {0, 0, 4, 4}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_BOX_9X9_08U[] = {{20, 8, 4, 0, {4, 4, 0, 0}}, {20, 8, 1, 1, {0, 0, 4, 4}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_BOX_9X9_08U[] = {{22, 8, 4, 0, {4, 4, 0, 0}}, {22, 8, 1, 1, {0, 0, 4, 4}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_BOX_9X9_08U[] = {{24, 8, 4, 0, {4, 4, 0, 0}}, {24, 8, 1, 1, {0, 0, 4, 4}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_BOX_9X9_08U[] = {{26, 8, 4, 0, {4, 4, 0, 0}}, {26, 8, 1, 1, {0, 0, 4, 4}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_BOX_9X9_08U[] = {{28, 8, 4, 0, {4, 4, 0, 0}}, {28, 8, 1, 1, {0, 0, 4, 4}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_BOX_9X9_08U[] = {{30, 8, 4, 0, {4, 4, 0, 0}}, {30, 8, 1, 1, {0, 0, 4, 4}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_BOX_9X9_08U[] = {{32, 8, 4, 0, {4, 4, 0, 0}}, {32, 8, 1, 1, {0, 0, 4, 4}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_BOX_9X9_08U[] = {{36, 8, 4, 0, {4, 4, 0, 0}}, {36, 8, 1, 1, {0, 0, 4, 4}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_BOX_9X9_08U[] = {{40, 8, 4, 0, {4, 4, 0, 0}}, {40, 8, 1, 1, {0, 0, 4, 4}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_BOX_9X9_08U[] = {{44, 8, 4, 0, {4, 4, 0, 0}}, {44, 8, 1, 1, {0, 0, 4, 4}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_BOX_9X9_08U[] = {{48, 8, 4, 0, {4, 4, 0, 0}}, {48, 8, 1, 1, {0, 0, 4, 4}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_BOX_9X9_08U[] = {{52, 8, 4, 0, {4, 4, 0, 0}}, {52, 8, 1, 1, {0, 0, 4, 4}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_BOX_9X9_08U[] = {{56, 8, 4, 0, {4, 4, 0, 0}}, {56, 8, 1, 1, {0, 0, 4, 4}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_BOX_9X9_08U[] = {{4, 10, 4, 0, {4, 4, 0, 0}}, {4, 10, 1, 1, {0, 0, 4, 4}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_BOX_9X9_08U[] = {{6, 10, 4, 0, {4, 4, 0, 0}}, {6, 10, 1, 1, {0, 0, 4, 4}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_BOX_9X9_08U[] = {{8, 10, 4, 0, {4, 4, 0, 0}}, {8, 10, 1, 1, {0, 0, 4, 4}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_BOX_9X9_08U[] = {{10, 10, 4, 0, {4, 4, 0, 0}}, {10, 10, 1, 1, {0, 0, 4, 4}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_BOX_9X9_08U[] = {{12, 10, 4, 0, {4, 4, 0, 0}}, {12, 10, 1, 1, {0, 0, 4, 4}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_BOX_9X9_08U[] = {{14, 10, 4, 0, {4, 4, 0, 0}}, {14, 10, 1, 1, {0, 0, 4, 4}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_BOX_9X9_08U[] = {{16, 10, 4, 0, {4, 4, 0, 0}}, {16, 10, 1, 1, {0, 0, 4, 4}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_BOX_9X9_08U[] = {{18, 10, 4, 0, {4, 4, 0, 0}}, {18, 10, 1, 1, {0, 0, 4, 4}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_BOX_9X9_08U[] = {{20, 10, 4, 0, {4, 4, 0, 0}}, {20, 10, 1, 1, {0, 0, 4, 4}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_BOX_9X9_08U[] = {{22, 10, 4, 0, {4, 4, 0, 0}}, {22, 10, 1, 1, {0, 0, 4, 4}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_BOX_9X9_08U[] = {{24, 10, 4, 0, {4, 4, 0, 0}}, {24, 10, 1, 1, {0, 0, 4, 4}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_BOX_9X9_08U[] = {{26, 10, 4, 0, {4, 4, 0, 0}}, {26, 10, 1, 1, {0, 0, 4, 4}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_BOX_9X9_08U[] = {{28, 10, 4, 0, {4, 4, 0, 0}}, {28, 10, 1, 1, {0, 0, 4, 4}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_BOX_9X9_08U[] = {{30, 10, 4, 0, {4, 4, 0, 0}}, {30, 10, 1, 1, {0, 0, 4, 4}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_BOX_9X9_08U[] = {{32, 10, 4, 0, {4, 4, 0, 0}}, {32, 10, 1, 1, {0, 0, 4, 4}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_BOX_9X9_08U[] = {{36, 10, 4, 0, {4, 4, 0, 0}}, {36, 10, 1, 1, {0, 0, 4, 4}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_BOX_9X9_08U[] = {{40, 10, 4, 0, {4, 4, 0, 0}}, {40, 10, 1, 1, {0, 0, 4, 4}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_BOX_9X9_08U[] = {{44, 10, 4, 0, {4, 4, 0, 0}}, {44, 10, 1, 1, {0, 0, 4, 4}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_BOX_9X9_08U[] = {{48, 10, 4, 0, {4, 4, 0, 0}}, {48, 10, 1, 1, {0, 0, 4, 4}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_BOX_9X9_08U[] = {{4, 12, 4, 0, {4, 4, 0, 0}}, {4, 12, 1, 1, {0, 0, 4, 4}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_BOX_9X9_08U[] = {{6, 12, 4, 0, {4, 4, 0, 0}}, {6, 12, 1, 1, {0, 0, 4, 4}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_BOX_9X9_08U[] = {{8, 12, 4, 0, {4, 4, 0, 0}}, {8, 12, 1, 1, {0, 0, 4, 4}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_BOX_9X9_08U[] = {{10, 12, 4, 0, {4, 4, 0, 0}}, {10, 12, 1, 1, {0, 0, 4, 4}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_BOX_9X9_08U[] = {{12, 12, 4, 0, {4, 4, 0, 0}}, {12, 12, 1, 1, {0, 0, 4, 4}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_BOX_9X9_08U[] = {{14, 12, 4, 0, {4, 4, 0, 0}}, {14, 12, 1, 1, {0, 0, 4, 4}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_BOX_9X9_08U[] = {{16, 12, 4, 0, {4, 4, 0, 0}}, {16, 12, 1, 1, {0, 0, 4, 4}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_BOX_9X9_08U[] = {{18, 12, 4, 0, {4, 4, 0, 0}}, {18, 12, 1, 1, {0, 0, 4, 4}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_BOX_9X9_08U[] = {{20, 12, 4, 0, {4, 4, 0, 0}}, {20, 12, 1, 1, {0, 0, 4, 4}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_BOX_9X9_08U[] = {{22, 12, 4, 0, {4, 4, 0, 0}}, {22, 12, 1, 1, {0, 0, 4, 4}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_BOX_9X9_08U[] = {{24, 12, 4, 0, {4, 4, 0, 0}}, {24, 12, 1, 1, {0, 0, 4, 4}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_BOX_9X9_08U[] = {{26, 12, 4, 0, {4, 4, 0, 0}}, {26, 12, 1, 1, {0, 0, 4, 4}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_BOX_9X9_08U[] = {{28, 12, 4, 0, {4, 4, 0, 0}}, {28, 12, 1, 1, {0, 0, 4, 4}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_BOX_9X9_08U[] = {{30, 12, 4, 0, {4, 4, 0, 0}}, {30, 12, 1, 1, {0, 0, 4, 4}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_BOX_9X9_08U[] = {{32, 12, 4, 0, {4, 4, 0, 0}}, {32, 12, 1, 1, {0, 0, 4, 4}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_BOX_9X9_08U[] = {{36, 12, 4, 0, {4, 4, 0, 0}}, {36, 12, 1, 1, {0, 0, 4, 4}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_BOX_9X9_08U[] = {{40, 12, 4, 0, {4, 4, 0, 0}}, {40, 12, 1, 1, {0, 0, 4, 4}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_BOX_9X9_08U[] = {{4, 14, 4, 0, {4, 4, 0, 0}}, {4, 14, 1, 1, {0, 0, 4, 4}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_BOX_9X9_08U[] = {{6, 14, 4, 0, {4, 4, 0, 0}}, {6, 14, 1, 1, {0, 0, 4, 4}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_BOX_9X9_08U[] = {{8, 14, 4, 0, {4, 4, 0, 0}}, {8, 14, 1, 1, {0, 0, 4, 4}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_BOX_9X9_08U[] = {{10, 14, 4, 0, {4, 4, 0, 0}}, {10, 14, 1, 1, {0, 0, 4, 4}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_BOX_9X9_08U[] = {{12, 14, 4, 0, {4, 4, 0, 0}}, {12, 14, 1, 1, {0, 0, 4, 4}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_BOX_9X9_08U[] = {{14, 14, 4, 0, {4, 4, 0, 0}}, {14, 14, 1, 1, {0, 0, 4, 4}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_BOX_9X9_08U[] = {{16, 14, 4, 0, {4, 4, 0, 0}}, {16, 14, 1, 1, {0, 0, 4, 4}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_BOX_9X9_08U[] = {{18, 14, 4, 0, {4, 4, 0, 0}}, {18, 14, 1, 1, {0, 0, 4, 4}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_BOX_9X9_08U[] = {{20, 14, 4, 0, {4, 4, 0, 0}}, {20, 14, 1, 1, {0, 0, 4, 4}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_BOX_9X9_08U[] = {{22, 14, 4, 0, {4, 4, 0, 0}}, {22, 14, 1, 1, {0, 0, 4, 4}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_BOX_9X9_08U[] = {{24, 14, 4, 0, {4, 4, 0, 0}}, {24, 14, 1, 1, {0, 0, 4, 4}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_BOX_9X9_08U[] = {{26, 14, 4, 0, {4, 4, 0, 0}}, {26, 14, 1, 1, {0, 0, 4, 4}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_BOX_9X9_08U[] = {{28, 14, 4, 0, {4, 4, 0, 0}}, {28, 14, 1, 1, {0, 0, 4, 4}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_BOX_9X9_08U[] = {{30, 14, 4, 0, {4, 4, 0, 0}}, {30, 14, 1, 1, {0, 0, 4, 4}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_BOX_9X9_08U[] = {{32, 14, 4, 0, {4, 4, 0, 0}}, {32, 14, 1, 1, {0, 0, 4, 4}}, {32, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_BOX_9X9_08U[] = {{4, 16, 4, 0, {4, 4, 0, 0}}, {4, 16, 1, 1, {0, 0, 4, 4}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_BOX_9X9_08U[] = {{6, 16, 4, 0, {4, 4, 0, 0}}, {6, 16, 1, 1, {0, 0, 4, 4}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_BOX_9X9_08U[] = {{8, 16, 4, 0, {4, 4, 0, 0}}, {8, 16, 1, 1, {0, 0, 4, 4}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_BOX_9X9_08U[] = {{10, 16, 4, 0, {4, 4, 0, 0}}, {10, 16, 1, 1, {0, 0, 4, 4}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_BOX_9X9_08U[] = {{12, 16, 4, 0, {4, 4, 0, 0}}, {12, 16, 1, 1, {0, 0, 4, 4}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_BOX_9X9_08U[] = {{14, 16, 4, 0, {4, 4, 0, 0}}, {14, 16, 1, 1, {0, 0, 4, 4}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_BOX_9X9_08U[] = {{16, 16, 4, 0, {4, 4, 0, 0}}, {16, 16, 1, 1, {0, 0, 4, 4}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_BOX_9X9_08U[] = {{18, 16, 4, 0, {4, 4, 0, 0}}, {18, 16, 1, 1, {0, 0, 4, 4}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_BOX_9X9_08U[] = {{20, 16, 4, 0, {4, 4, 0, 0}}, {20, 16, 1, 1, {0, 0, 4, 4}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_BOX_9X9_08U[] = {{22, 16, 4, 0, {4, 4, 0, 0}}, {22, 16, 1, 1, {0, 0, 4, 4}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_BOX_9X9_08U[] = {{24, 16, 4, 0, {4, 4, 0, 0}}, {24, 16, 1, 1, {0, 0, 4, 4}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_BOX_9X9_08U[] = {{26, 16, 4, 0, {4, 4, 0, 0}}, {26, 16, 1, 1, {0, 0, 4, 4}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_BOX_9X9_08U[] = {{28, 16, 4, 0, {4, 4, 0, 0}}, {28, 16, 1, 1, {0, 0, 4, 4}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_BOX_9X9_08U[] = {{30, 16, 4, 0, {4, 4, 0, 0}}, {30, 16, 1, 1, {0, 0, 4, 4}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_BOX_9X9_08U[] = {{4, 18, 4, 0, {4, 4, 0, 0}}, {4, 18, 1, 1, {0, 0, 4, 4}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_BOX_9X9_08U[] = {{6, 18, 4, 0, {4, 4, 0, 0}}, {6, 18, 1, 1, {0, 0, 4, 4}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_BOX_9X9_08U[] = {{8, 18, 4, 0, {4, 4, 0, 0}}, {8, 18, 1, 1, {0, 0, 4, 4}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_BOX_9X9_08U[] = {{10, 18, 4, 0, {4, 4, 0, 0}}, {10, 18, 1, 1, {0, 0, 4, 4}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_BOX_9X9_08U[] = {{12, 18, 4, 0, {4, 4, 0, 0}}, {12, 18, 1, 1, {0, 0, 4, 4}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_BOX_9X9_08U[] = {{14, 18, 4, 0, {4, 4, 0, 0}}, {14, 18, 1, 1, {0, 0, 4, 4}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_BOX_9X9_08U[] = {{16, 18, 4, 0, {4, 4, 0, 0}}, {16, 18, 1, 1, {0, 0, 4, 4}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_BOX_9X9_08U[] = {{18, 18, 4, 0, {4, 4, 0, 0}}, {18, 18, 1, 1, {0, 0, 4, 4}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_BOX_9X9_08U[] = {{20, 18, 4, 0, {4, 4, 0, 0}}, {20, 18, 1, 1, {0, 0, 4, 4}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_BOX_9X9_08U[] = {{22, 18, 4, 0, {4, 4, 0, 0}}, {22, 18, 1, 1, {0, 0, 4, 4}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_BOX_9X9_08U[] = {{24, 18, 4, 0, {4, 4, 0, 0}}, {24, 18, 1, 1, {0, 0, 4, 4}}, {24, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_BOX_9X9_08U[] = {{26, 18, 4, 0, {4, 4, 0, 0}}, {26, 18, 1, 1, {0, 0, 4, 4}}, {26, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_BOX_9X9_08U[] = {{4, 20, 4, 0, {4, 4, 0, 0}}, {4, 20, 1, 1, {0, 0, 4, 4}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_BOX_9X9_08U[] = {{6, 20, 4, 0, {4, 4, 0, 0}}, {6, 20, 1, 1, {0, 0, 4, 4}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_BOX_9X9_08U[] = {{8, 20, 4, 0, {4, 4, 0, 0}}, {8, 20, 1, 1, {0, 0, 4, 4}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_BOX_9X9_08U[] = {{10, 20, 4, 0, {4, 4, 0, 0}}, {10, 20, 1, 1, {0, 0, 4, 4}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_BOX_9X9_08U[] = {{12, 20, 4, 0, {4, 4, 0, 0}}, {12, 20, 1, 1, {0, 0, 4, 4}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_BOX_9X9_08U[] = {{14, 20, 4, 0, {4, 4, 0, 0}}, {14, 20, 1, 1, {0, 0, 4, 4}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_BOX_9X9_08U[] = {{16, 20, 4, 0, {4, 4, 0, 0}}, {16, 20, 1, 1, {0, 0, 4, 4}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_BOX_9X9_08U[] = {{18, 20, 4, 0, {4, 4, 0, 0}}, {18, 20, 1, 1, {0, 0, 4, 4}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_BOX_9X9_08U[] = {{20, 20, 4, 0, {4, 4, 0, 0}}, {20, 20, 1, 1, {0, 0, 4, 4}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_BOX_9X9_08U[] = {{22, 20, 4, 0, {4, 4, 0, 0}}, {22, 20, 1, 1, {0, 0, 4, 4}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_BOX_9X9_08U[] = {{24, 20, 4, 0, {4, 4, 0, 0}}, {24, 20, 1, 1, {0, 0, 4, 4}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_BOX_9X9_08U[] = {{4, 22, 4, 0, {4, 4, 0, 0}}, {4, 22, 1, 1, {0, 0, 4, 4}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_BOX_9X9_08U[] = {{6, 22, 4, 0, {4, 4, 0, 0}}, {6, 22, 1, 1, {0, 0, 4, 4}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_BOX_9X9_08U[] = {{8, 22, 4, 0, {4, 4, 0, 0}}, {8, 22, 1, 1, {0, 0, 4, 4}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_BOX_9X9_08U[] = {{10, 22, 4, 0, {4, 4, 0, 0}}, {10, 22, 1, 1, {0, 0, 4, 4}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_BOX_9X9_08U[] = {{12, 22, 4, 0, {4, 4, 0, 0}}, {12, 22, 1, 1, {0, 0, 4, 4}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_BOX_9X9_08U[] = {{14, 22, 4, 0, {4, 4, 0, 0}}, {14, 22, 1, 1, {0, 0, 4, 4}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_BOX_9X9_08U[] = {{16, 22, 4, 0, {4, 4, 0, 0}}, {16, 22, 1, 1, {0, 0, 4, 4}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_BOX_9X9_08U[] = {{18, 22, 4, 0, {4, 4, 0, 0}}, {18, 22, 1, 1, {0, 0, 4, 4}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_BOX_9X9_08U[] = {{20, 22, 4, 0, {4, 4, 0, 0}}, {20, 22, 1, 1, {0, 0, 4, 4}}, {20, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_BOX_9X9_08U[] = {{22, 22, 4, 0, {4, 4, 0, 0}}, {22, 22, 1, 1, {0, 0, 4, 4}}, {22, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_BOX_9X9_08U[] = {{4, 24, 4, 0, {4, 4, 0, 0}}, {4, 24, 1, 1, {0, 0, 4, 4}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_BOX_9X9_08U[] = {{6, 24, 4, 0, {4, 4, 0, 0}}, {6, 24, 1, 1, {0, 0, 4, 4}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_BOX_9X9_08U[] = {{8, 24, 4, 0, {4, 4, 0, 0}}, {8, 24, 1, 1, {0, 0, 4, 4}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_BOX_9X9_08U[] = {{10, 24, 4, 0, {4, 4, 0, 0}}, {10, 24, 1, 1, {0, 0, 4, 4}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_BOX_9X9_08U[] = {{12, 24, 4, 0, {4, 4, 0, 0}}, {12, 24, 1, 1, {0, 0, 4, 4}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_BOX_9X9_08U[] = {{14, 24, 4, 0, {4, 4, 0, 0}}, {14, 24, 1, 1, {0, 0, 4, 4}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_BOX_9X9_08U[] = {{16, 24, 4, 0, {4, 4, 0, 0}}, {16, 24, 1, 1, {0, 0, 4, 4}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_BOX_9X9_08U[] = {{18, 24, 4, 0, {4, 4, 0, 0}}, {18, 24, 1, 1, {0, 0, 4, 4}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_BOX_9X9_08U[] = {{20, 24, 4, 0, {4, 4, 0, 0}}, {20, 24, 1, 1, {0, 0, 4, 4}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_BOX_9X9_08U[] = {{4, 26, 4, 0, {4, 4, 0, 0}}, {4, 26, 1, 1, {0, 0, 4, 4}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_BOX_9X9_08U[] = {{6, 26, 4, 0, {4, 4, 0, 0}}, {6, 26, 1, 1, {0, 0, 4, 4}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_BOX_9X9_08U[] = {{8, 26, 4, 0, {4, 4, 0, 0}}, {8, 26, 1, 1, {0, 0, 4, 4}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_BOX_9X9_08U[] = {{10, 26, 4, 0, {4, 4, 0, 0}}, {10, 26, 1, 1, {0, 0, 4, 4}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_BOX_9X9_08U[] = {{12, 26, 4, 0, {4, 4, 0, 0}}, {12, 26, 1, 1, {0, 0, 4, 4}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_BOX_9X9_08U[] = {{14, 26, 4, 0, {4, 4, 0, 0}}, {14, 26, 1, 1, {0, 0, 4, 4}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_BOX_9X9_08U[] = {{16, 26, 4, 0, {4, 4, 0, 0}}, {16, 26, 1, 1, {0, 0, 4, 4}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_BOX_9X9_08U[] = {{18, 26, 4, 0, {4, 4, 0, 0}}, {18, 26, 1, 1, {0, 0, 4, 4}}, {18, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_BOX_9X9_08U[] = {{4, 28, 4, 0, {4, 4, 0, 0}}, {4, 28, 1, 1, {0, 0, 4, 4}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_BOX_9X9_08U[] = {{6, 28, 4, 0, {4, 4, 0, 0}}, {6, 28, 1, 1, {0, 0, 4, 4}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_BOX_9X9_08U[] = {{8, 28, 4, 0, {4, 4, 0, 0}}, {8, 28, 1, 1, {0, 0, 4, 4}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_BOX_9X9_08U[] = {{10, 28, 4, 0, {4, 4, 0, 0}}, {10, 28, 1, 1, {0, 0, 4, 4}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_BOX_9X9_08U[] = {{12, 28, 4, 0, {4, 4, 0, 0}}, {12, 28, 1, 1, {0, 0, 4, 4}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_BOX_9X9_08U[] = {{14, 28, 4, 0, {4, 4, 0, 0}}, {14, 28, 1, 1, {0, 0, 4, 4}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_BOX_9X9_08U[] = {{16, 28, 4, 0, {4, 4, 0, 0}}, {16, 28, 1, 1, {0, 0, 4, 4}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_BOX_9X9_08U[] = {{4, 30, 4, 0, {4, 4, 0, 0}}, {4, 30, 1, 1, {0, 0, 4, 4}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_BOX_9X9_08U[] = {{6, 30, 4, 0, {4, 4, 0, 0}}, {6, 30, 1, 1, {0, 0, 4, 4}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_BOX_9X9_08U[] = {{8, 30, 4, 0, {4, 4, 0, 0}}, {8, 30, 1, 1, {0, 0, 4, 4}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_BOX_9X9_08U[] = {{10, 30, 4, 0, {4, 4, 0, 0}}, {10, 30, 1, 1, {0, 0, 4, 4}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_BOX_9X9_08U[] = {{12, 30, 4, 0, {4, 4, 0, 0}}, {12, 30, 1, 1, {0, 0, 4, 4}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_BOX_9X9_08U[] = {{14, 30, 4, 0, {4, 4, 0, 0}}, {14, 30, 1, 1, {0, 0, 4, 4}}, {14, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_BOX_9X9_08U[] = {{16, 30, 4, 0, {4, 4, 0, 0}}, {16, 30, 1, 1, {0, 0, 4, 4}}, {16, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_BOX_9X9_08U[] = {{4, 32, 4, 0, {4, 4, 0, 0}}, {4, 32, 1, 1, {0, 0, 4, 4}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_BOX_9X9_08U[] = {{6, 32, 4, 0, {4, 4, 0, 0}}, {6, 32, 1, 1, {0, 0, 4, 4}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_BOX_9X9_08U[] = {{8, 32, 4, 0, {4, 4, 0, 0}}, {8, 32, 1, 1, {0, 0, 4, 4}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_BOX_9X9_08U[] = {{10, 32, 4, 0, {4, 4, 0, 0}}, {10, 32, 1, 1, {0, 0, 4, 4}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_BOX_9X9_08U[] = {{12, 32, 4, 0, {4, 4, 0, 0}}, {12, 32, 1, 1, {0, 0, 4, 4}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_BOX_9X9_08U[] = {{14, 32, 4, 0, {4, 4, 0, 0}}, {14, 32, 1, 1, {0, 0, 4, 4}}, {14, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_BOX_9X9_08U[] = {{4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_BOX_9X9_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_BOX_9X9_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_BOX_9X9_08U[] = {{1, 0}, {1, 0}};

static acf_scenario gScenarioArray_BOX_9X9_08U[] = {
{4, 1, 104, 0, 4, gScenarioBufferData0_BOX_9X9_08U, 48, gScenarioKernelData0_BOX_9X9_08U, 8},
{6, 1, 156, 0, 4, gScenarioBufferData1_BOX_9X9_08U, 48, gScenarioKernelData1_BOX_9X9_08U, 8},
{8, 1, 200, 0, 4, gScenarioBufferData2_BOX_9X9_08U, 48, gScenarioKernelData2_BOX_9X9_08U, 8},
{10, 1, 252, 0, 4, gScenarioBufferData3_BOX_9X9_08U, 48, gScenarioKernelData3_BOX_9X9_08U, 8},
{12, 1, 296, 0, 4, gScenarioBufferData4_BOX_9X9_08U, 48, gScenarioKernelData4_BOX_9X9_08U, 8},
{14, 1, 348, 0, 4, gScenarioBufferData5_BOX_9X9_08U, 48, gScenarioKernelData5_BOX_9X9_08U, 8},
{16, 1, 392, 0, 4, gScenarioBufferData6_BOX_9X9_08U, 48, gScenarioKernelData6_BOX_9X9_08U, 8},
{18, 1, 444, 0, 4, gScenarioBufferData7_BOX_9X9_08U, 48, gScenarioKernelData7_BOX_9X9_08U, 8},
{20, 1, 488, 0, 4, gScenarioBufferData8_BOX_9X9_08U, 48, gScenarioKernelData8_BOX_9X9_08U, 8},
{22, 1, 540, 0, 4, gScenarioBufferData9_BOX_9X9_08U, 48, gScenarioKernelData9_BOX_9X9_08U, 8},
{24, 1, 584, 0, 4, gScenarioBufferData10_BOX_9X9_08U, 48, gScenarioKernelData10_BOX_9X9_08U, 8},
{26, 1, 636, 0, 4, gScenarioBufferData11_BOX_9X9_08U, 48, gScenarioKernelData11_BOX_9X9_08U, 8},
{28, 1, 680, 0, 4, gScenarioBufferData12_BOX_9X9_08U, 48, gScenarioKernelData12_BOX_9X9_08U, 8},
{30, 1, 732, 0, 4, gScenarioBufferData13_BOX_9X9_08U, 48, gScenarioKernelData13_BOX_9X9_08U, 8},
{32, 1, 776, 0, 4, gScenarioBufferData14_BOX_9X9_08U, 48, gScenarioKernelData14_BOX_9X9_08U, 8},
{36, 1, 872, 0, 4, gScenarioBufferData15_BOX_9X9_08U, 48, gScenarioKernelData15_BOX_9X9_08U, 8},
{40, 1, 968, 0, 4, gScenarioBufferData16_BOX_9X9_08U, 48, gScenarioKernelData16_BOX_9X9_08U, 8},
{44, 1, 1064, 0, 4, gScenarioBufferData17_BOX_9X9_08U, 48, gScenarioKernelData17_BOX_9X9_08U, 8},
{48, 1, 1160, 0, 4, gScenarioBufferData18_BOX_9X9_08U, 48, gScenarioKernelData18_BOX_9X9_08U, 8},
{52, 1, 1256, 0, 4, gScenarioBufferData19_BOX_9X9_08U, 48, gScenarioKernelData19_BOX_9X9_08U, 8},
{56, 1, 1352, 0, 4, gScenarioBufferData20_BOX_9X9_08U, 48, gScenarioKernelData20_BOX_9X9_08U, 8},
{60, 1, 1448, 0, 4, gScenarioBufferData21_BOX_9X9_08U, 48, gScenarioKernelData21_BOX_9X9_08U, 8},
{64, 1, 1544, 0, 4, gScenarioBufferData22_BOX_9X9_08U, 48, gScenarioKernelData22_BOX_9X9_08U, 8},
{72, 1, 1736, 0, 4, gScenarioBufferData23_BOX_9X9_08U, 48, gScenarioKernelData23_BOX_9X9_08U, 8},
{80, 1, 1928, 0, 4, gScenarioBufferData24_BOX_9X9_08U, 48, gScenarioKernelData24_BOX_9X9_08U, 8},
{88, 1, 2120, 0, 4, gScenarioBufferData25_BOX_9X9_08U, 48, gScenarioKernelData25_BOX_9X9_08U, 8},
{96, 1, 2312, 0, 4, gScenarioBufferData26_BOX_9X9_08U, 48, gScenarioKernelData26_BOX_9X9_08U, 8},
{104, 1, 2504, 0, 4, gScenarioBufferData27_BOX_9X9_08U, 48, gScenarioKernelData27_BOX_9X9_08U, 8},
{112, 1, 2696, 0, 4, gScenarioBufferData28_BOX_9X9_08U, 48, gScenarioKernelData28_BOX_9X9_08U, 8},
{120, 1, 2888, 0, 4, gScenarioBufferData29_BOX_9X9_08U, 48, gScenarioKernelData29_BOX_9X9_08U, 8},
{128, 1, 3080, 0, 4, gScenarioBufferData30_BOX_9X9_08U, 48, gScenarioKernelData30_BOX_9X9_08U, 8},
{4, 2, 128, 0, 2, gScenarioBufferData31_BOX_9X9_08U, 48, gScenarioKernelData31_BOX_9X9_08U, 8},
{6, 2, 184, 0, 2, gScenarioBufferData32_BOX_9X9_08U, 48, gScenarioKernelData32_BOX_9X9_08U, 8},
{8, 2, 240, 0, 2, gScenarioBufferData33_BOX_9X9_08U, 48, gScenarioKernelData33_BOX_9X9_08U, 8},
{10, 2, 296, 0, 2, gScenarioBufferData34_BOX_9X9_08U, 48, gScenarioKernelData34_BOX_9X9_08U, 8},
{12, 2, 352, 0, 2, gScenarioBufferData35_BOX_9X9_08U, 48, gScenarioKernelData35_BOX_9X9_08U, 8},
{14, 2, 408, 0, 2, gScenarioBufferData36_BOX_9X9_08U, 48, gScenarioKernelData36_BOX_9X9_08U, 8},
{16, 2, 464, 0, 2, gScenarioBufferData37_BOX_9X9_08U, 48, gScenarioKernelData37_BOX_9X9_08U, 8},
{18, 2, 520, 0, 2, gScenarioBufferData38_BOX_9X9_08U, 48, gScenarioKernelData38_BOX_9X9_08U, 8},
{20, 2, 576, 0, 2, gScenarioBufferData39_BOX_9X9_08U, 48, gScenarioKernelData39_BOX_9X9_08U, 8},
{22, 2, 632, 0, 2, gScenarioBufferData40_BOX_9X9_08U, 48, gScenarioKernelData40_BOX_9X9_08U, 8},
{24, 2, 688, 0, 2, gScenarioBufferData41_BOX_9X9_08U, 48, gScenarioKernelData41_BOX_9X9_08U, 8},
{26, 2, 744, 0, 2, gScenarioBufferData42_BOX_9X9_08U, 48, gScenarioKernelData42_BOX_9X9_08U, 8},
{28, 2, 800, 0, 2, gScenarioBufferData43_BOX_9X9_08U, 48, gScenarioKernelData43_BOX_9X9_08U, 8},
{30, 2, 856, 0, 2, gScenarioBufferData44_BOX_9X9_08U, 48, gScenarioKernelData44_BOX_9X9_08U, 8},
{32, 2, 912, 0, 2, gScenarioBufferData45_BOX_9X9_08U, 48, gScenarioKernelData45_BOX_9X9_08U, 8},
{36, 2, 1024, 0, 2, gScenarioBufferData46_BOX_9X9_08U, 48, gScenarioKernelData46_BOX_9X9_08U, 8},
{40, 2, 1136, 0, 2, gScenarioBufferData47_BOX_9X9_08U, 48, gScenarioKernelData47_BOX_9X9_08U, 8},
{44, 2, 1248, 0, 2, gScenarioBufferData48_BOX_9X9_08U, 48, gScenarioKernelData48_BOX_9X9_08U, 8},
{48, 2, 1360, 0, 2, gScenarioBufferData49_BOX_9X9_08U, 48, gScenarioKernelData49_BOX_9X9_08U, 8},
{52, 2, 1472, 0, 2, gScenarioBufferData50_BOX_9X9_08U, 48, gScenarioKernelData50_BOX_9X9_08U, 8},
{56, 2, 1584, 0, 2, gScenarioBufferData51_BOX_9X9_08U, 48, gScenarioKernelData51_BOX_9X9_08U, 8},
{60, 2, 1696, 0, 2, gScenarioBufferData52_BOX_9X9_08U, 48, gScenarioKernelData52_BOX_9X9_08U, 8},
{64, 2, 1808, 0, 2, gScenarioBufferData53_BOX_9X9_08U, 48, gScenarioKernelData53_BOX_9X9_08U, 8},
{72, 2, 2032, 0, 2, gScenarioBufferData54_BOX_9X9_08U, 48, gScenarioKernelData54_BOX_9X9_08U, 8},
{80, 2, 2256, 0, 2, gScenarioBufferData55_BOX_9X9_08U, 48, gScenarioKernelData55_BOX_9X9_08U, 8},
{88, 2, 2480, 0, 2, gScenarioBufferData56_BOX_9X9_08U, 48, gScenarioKernelData56_BOX_9X9_08U, 8},
{96, 2, 2704, 0, 2, gScenarioBufferData57_BOX_9X9_08U, 48, gScenarioKernelData57_BOX_9X9_08U, 8},
{104, 2, 2928, 0, 2, gScenarioBufferData58_BOX_9X9_08U, 48, gScenarioKernelData58_BOX_9X9_08U, 8},
{112, 2, 3152, 0, 2, gScenarioBufferData59_BOX_9X9_08U, 48, gScenarioKernelData59_BOX_9X9_08U, 8},
{120, 2, 3376, 0, 2, gScenarioBufferData60_BOX_9X9_08U, 48, gScenarioKernelData60_BOX_9X9_08U, 8},
{128, 2, 3600, 0, 2, gScenarioBufferData61_BOX_9X9_08U, 48, gScenarioKernelData61_BOX_9X9_08U, 8},
{4, 3, 176, 0, 2, gScenarioBufferData62_BOX_9X9_08U, 48, gScenarioKernelData62_BOX_9X9_08U, 8},
{6, 3, 256, 0, 2, gScenarioBufferData63_BOX_9X9_08U, 48, gScenarioKernelData63_BOX_9X9_08U, 8},
{8, 3, 328, 0, 2, gScenarioBufferData64_BOX_9X9_08U, 48, gScenarioKernelData64_BOX_9X9_08U, 8},
{10, 3, 408, 0, 2, gScenarioBufferData65_BOX_9X9_08U, 48, gScenarioKernelData65_BOX_9X9_08U, 8},
{12, 3, 480, 0, 2, gScenarioBufferData66_BOX_9X9_08U, 48, gScenarioKernelData66_BOX_9X9_08U, 8},
{14, 3, 560, 0, 2, gScenarioBufferData67_BOX_9X9_08U, 48, gScenarioKernelData67_BOX_9X9_08U, 8},
{16, 3, 632, 0, 2, gScenarioBufferData68_BOX_9X9_08U, 48, gScenarioKernelData68_BOX_9X9_08U, 8},
{18, 3, 712, 0, 2, gScenarioBufferData69_BOX_9X9_08U, 48, gScenarioKernelData69_BOX_9X9_08U, 8},
{20, 3, 784, 0, 2, gScenarioBufferData70_BOX_9X9_08U, 48, gScenarioKernelData70_BOX_9X9_08U, 8},
{22, 3, 864, 0, 2, gScenarioBufferData71_BOX_9X9_08U, 48, gScenarioKernelData71_BOX_9X9_08U, 8},
{24, 3, 936, 0, 2, gScenarioBufferData72_BOX_9X9_08U, 48, gScenarioKernelData72_BOX_9X9_08U, 8},
{26, 3, 1016, 0, 2, gScenarioBufferData73_BOX_9X9_08U, 48, gScenarioKernelData73_BOX_9X9_08U, 8},
{28, 3, 1088, 0, 2, gScenarioBufferData74_BOX_9X9_08U, 48, gScenarioKernelData74_BOX_9X9_08U, 8},
{30, 3, 1168, 0, 2, gScenarioBufferData75_BOX_9X9_08U, 48, gScenarioKernelData75_BOX_9X9_08U, 8},
{32, 3, 1240, 0, 2, gScenarioBufferData76_BOX_9X9_08U, 48, gScenarioKernelData76_BOX_9X9_08U, 8},
{36, 3, 1392, 0, 2, gScenarioBufferData77_BOX_9X9_08U, 48, gScenarioKernelData77_BOX_9X9_08U, 8},
{40, 3, 1544, 0, 2, gScenarioBufferData78_BOX_9X9_08U, 48, gScenarioKernelData78_BOX_9X9_08U, 8},
{44, 3, 1696, 0, 2, gScenarioBufferData79_BOX_9X9_08U, 48, gScenarioKernelData79_BOX_9X9_08U, 8},
{48, 3, 1848, 0, 2, gScenarioBufferData80_BOX_9X9_08U, 48, gScenarioKernelData80_BOX_9X9_08U, 8},
{52, 3, 2000, 0, 2, gScenarioBufferData81_BOX_9X9_08U, 48, gScenarioKernelData81_BOX_9X9_08U, 8},
{56, 3, 2152, 0, 2, gScenarioBufferData82_BOX_9X9_08U, 48, gScenarioKernelData82_BOX_9X9_08U, 8},
{60, 3, 2304, 0, 2, gScenarioBufferData83_BOX_9X9_08U, 48, gScenarioKernelData83_BOX_9X9_08U, 8},
{64, 3, 2456, 0, 2, gScenarioBufferData84_BOX_9X9_08U, 48, gScenarioKernelData84_BOX_9X9_08U, 8},
{72, 3, 2760, 0, 2, gScenarioBufferData85_BOX_9X9_08U, 48, gScenarioKernelData85_BOX_9X9_08U, 8},
{80, 3, 3064, 0, 2, gScenarioBufferData86_BOX_9X9_08U, 48, gScenarioKernelData86_BOX_9X9_08U, 8},
{88, 3, 3368, 0, 2, gScenarioBufferData87_BOX_9X9_08U, 48, gScenarioKernelData87_BOX_9X9_08U, 8},
{96, 3, 3672, 0, 2, gScenarioBufferData88_BOX_9X9_08U, 48, gScenarioKernelData88_BOX_9X9_08U, 8},
{4, 4, 176, 0, 1, gScenarioBufferData89_BOX_9X9_08U, 48, gScenarioKernelData89_BOX_9X9_08U, 8},
{6, 4, 248, 0, 1, gScenarioBufferData90_BOX_9X9_08U, 48, gScenarioKernelData90_BOX_9X9_08U, 8},
{8, 4, 320, 0, 1, gScenarioBufferData91_BOX_9X9_08U, 48, gScenarioKernelData91_BOX_9X9_08U, 8},
{10, 4, 392, 0, 1, gScenarioBufferData92_BOX_9X9_08U, 48, gScenarioKernelData92_BOX_9X9_08U, 8},
{12, 4, 464, 0, 1, gScenarioBufferData93_BOX_9X9_08U, 48, gScenarioKernelData93_BOX_9X9_08U, 8},
{14, 4, 536, 0, 1, gScenarioBufferData94_BOX_9X9_08U, 48, gScenarioKernelData94_BOX_9X9_08U, 8},
{16, 4, 608, 0, 1, gScenarioBufferData95_BOX_9X9_08U, 48, gScenarioKernelData95_BOX_9X9_08U, 8},
{18, 4, 680, 0, 1, gScenarioBufferData96_BOX_9X9_08U, 48, gScenarioKernelData96_BOX_9X9_08U, 8},
{20, 4, 752, 0, 1, gScenarioBufferData97_BOX_9X9_08U, 48, gScenarioKernelData97_BOX_9X9_08U, 8},
{22, 4, 824, 0, 1, gScenarioBufferData98_BOX_9X9_08U, 48, gScenarioKernelData98_BOX_9X9_08U, 8},
{24, 4, 896, 0, 1, gScenarioBufferData99_BOX_9X9_08U, 48, gScenarioKernelData99_BOX_9X9_08U, 8},
{26, 4, 968, 0, 1, gScenarioBufferData100_BOX_9X9_08U, 48, gScenarioKernelData100_BOX_9X9_08U, 8},
{28, 4, 1040, 0, 1, gScenarioBufferData101_BOX_9X9_08U, 48, gScenarioKernelData101_BOX_9X9_08U, 8},
{30, 4, 1112, 0, 1, gScenarioBufferData102_BOX_9X9_08U, 48, gScenarioKernelData102_BOX_9X9_08U, 8},
{32, 4, 1184, 0, 1, gScenarioBufferData103_BOX_9X9_08U, 48, gScenarioKernelData103_BOX_9X9_08U, 8},
{36, 4, 1328, 0, 1, gScenarioBufferData104_BOX_9X9_08U, 48, gScenarioKernelData104_BOX_9X9_08U, 8},
{40, 4, 1472, 0, 1, gScenarioBufferData105_BOX_9X9_08U, 48, gScenarioKernelData105_BOX_9X9_08U, 8},
{44, 4, 1616, 0, 1, gScenarioBufferData106_BOX_9X9_08U, 48, gScenarioKernelData106_BOX_9X9_08U, 8},
{48, 4, 1760, 0, 1, gScenarioBufferData107_BOX_9X9_08U, 48, gScenarioKernelData107_BOX_9X9_08U, 8},
{52, 4, 1904, 0, 1, gScenarioBufferData108_BOX_9X9_08U, 48, gScenarioKernelData108_BOX_9X9_08U, 8},
{56, 4, 2048, 0, 1, gScenarioBufferData109_BOX_9X9_08U, 48, gScenarioKernelData109_BOX_9X9_08U, 8},
{60, 4, 2192, 0, 1, gScenarioBufferData110_BOX_9X9_08U, 48, gScenarioKernelData110_BOX_9X9_08U, 8},
{64, 4, 2336, 0, 1, gScenarioBufferData111_BOX_9X9_08U, 48, gScenarioKernelData111_BOX_9X9_08U, 8},
{72, 4, 2624, 0, 1, gScenarioBufferData112_BOX_9X9_08U, 48, gScenarioKernelData112_BOX_9X9_08U, 8},
{80, 4, 2912, 0, 1, gScenarioBufferData113_BOX_9X9_08U, 48, gScenarioKernelData113_BOX_9X9_08U, 8},
{88, 4, 3200, 0, 1, gScenarioBufferData114_BOX_9X9_08U, 48, gScenarioKernelData114_BOX_9X9_08U, 8},
{96, 4, 3488, 0, 1, gScenarioBufferData115_BOX_9X9_08U, 48, gScenarioKernelData115_BOX_9X9_08U, 8},
{104, 4, 3776, 0, 1, gScenarioBufferData116_BOX_9X9_08U, 48, gScenarioKernelData116_BOX_9X9_08U, 8},
{4, 5, 212, 0, 1, gScenarioBufferData117_BOX_9X9_08U, 48, gScenarioKernelData117_BOX_9X9_08U, 8},
{6, 5, 300, 0, 1, gScenarioBufferData118_BOX_9X9_08U, 48, gScenarioKernelData118_BOX_9X9_08U, 8},
{8, 5, 384, 0, 1, gScenarioBufferData119_BOX_9X9_08U, 48, gScenarioKernelData119_BOX_9X9_08U, 8},
{10, 5, 472, 0, 1, gScenarioBufferData120_BOX_9X9_08U, 48, gScenarioKernelData120_BOX_9X9_08U, 8},
{12, 5, 556, 0, 1, gScenarioBufferData121_BOX_9X9_08U, 48, gScenarioKernelData121_BOX_9X9_08U, 8},
{14, 5, 644, 0, 1, gScenarioBufferData122_BOX_9X9_08U, 48, gScenarioKernelData122_BOX_9X9_08U, 8},
{16, 5, 728, 0, 1, gScenarioBufferData123_BOX_9X9_08U, 48, gScenarioKernelData123_BOX_9X9_08U, 8},
{18, 5, 816, 0, 1, gScenarioBufferData124_BOX_9X9_08U, 48, gScenarioKernelData124_BOX_9X9_08U, 8},
{20, 5, 900, 0, 1, gScenarioBufferData125_BOX_9X9_08U, 48, gScenarioKernelData125_BOX_9X9_08U, 8},
{22, 5, 988, 0, 1, gScenarioBufferData126_BOX_9X9_08U, 48, gScenarioKernelData126_BOX_9X9_08U, 8},
{24, 5, 1072, 0, 1, gScenarioBufferData127_BOX_9X9_08U, 48, gScenarioKernelData127_BOX_9X9_08U, 8},
{26, 5, 1160, 0, 1, gScenarioBufferData128_BOX_9X9_08U, 48, gScenarioKernelData128_BOX_9X9_08U, 8},
{28, 5, 1244, 0, 1, gScenarioBufferData129_BOX_9X9_08U, 48, gScenarioKernelData129_BOX_9X9_08U, 8},
{30, 5, 1332, 0, 1, gScenarioBufferData130_BOX_9X9_08U, 48, gScenarioKernelData130_BOX_9X9_08U, 8},
{32, 5, 1416, 0, 1, gScenarioBufferData131_BOX_9X9_08U, 48, gScenarioKernelData131_BOX_9X9_08U, 8},
{36, 5, 1588, 0, 1, gScenarioBufferData132_BOX_9X9_08U, 48, gScenarioKernelData132_BOX_9X9_08U, 8},
{40, 5, 1760, 0, 1, gScenarioBufferData133_BOX_9X9_08U, 48, gScenarioKernelData133_BOX_9X9_08U, 8},
{44, 5, 1932, 0, 1, gScenarioBufferData134_BOX_9X9_08U, 48, gScenarioKernelData134_BOX_9X9_08U, 8},
{48, 5, 2104, 0, 1, gScenarioBufferData135_BOX_9X9_08U, 48, gScenarioKernelData135_BOX_9X9_08U, 8},
{52, 5, 2276, 0, 1, gScenarioBufferData136_BOX_9X9_08U, 48, gScenarioKernelData136_BOX_9X9_08U, 8},
{56, 5, 2448, 0, 1, gScenarioBufferData137_BOX_9X9_08U, 48, gScenarioKernelData137_BOX_9X9_08U, 8},
{60, 5, 2620, 0, 1, gScenarioBufferData138_BOX_9X9_08U, 48, gScenarioKernelData138_BOX_9X9_08U, 8},
{64, 5, 2792, 0, 1, gScenarioBufferData139_BOX_9X9_08U, 48, gScenarioKernelData139_BOX_9X9_08U, 8},
{72, 5, 3136, 0, 1, gScenarioBufferData140_BOX_9X9_08U, 48, gScenarioKernelData140_BOX_9X9_08U, 8},
{80, 5, 3480, 0, 1, gScenarioBufferData141_BOX_9X9_08U, 48, gScenarioKernelData141_BOX_9X9_08U, 8},
{88, 5, 3824, 0, 1, gScenarioBufferData142_BOX_9X9_08U, 48, gScenarioKernelData142_BOX_9X9_08U, 8},
{4, 6, 248, 0, 1, gScenarioBufferData143_BOX_9X9_08U, 48, gScenarioKernelData143_BOX_9X9_08U, 8},
{6, 6, 348, 0, 1, gScenarioBufferData144_BOX_9X9_08U, 48, gScenarioKernelData144_BOX_9X9_08U, 8},
{8, 6, 448, 0, 1, gScenarioBufferData145_BOX_9X9_08U, 48, gScenarioKernelData145_BOX_9X9_08U, 8},
{10, 6, 548, 0, 1, gScenarioBufferData146_BOX_9X9_08U, 48, gScenarioKernelData146_BOX_9X9_08U, 8},
{12, 6, 648, 0, 1, gScenarioBufferData147_BOX_9X9_08U, 48, gScenarioKernelData147_BOX_9X9_08U, 8},
{14, 6, 748, 0, 1, gScenarioBufferData148_BOX_9X9_08U, 48, gScenarioKernelData148_BOX_9X9_08U, 8},
{16, 6, 848, 0, 1, gScenarioBufferData149_BOX_9X9_08U, 48, gScenarioKernelData149_BOX_9X9_08U, 8},
{18, 6, 948, 0, 1, gScenarioBufferData150_BOX_9X9_08U, 48, gScenarioKernelData150_BOX_9X9_08U, 8},
{20, 6, 1048, 0, 1, gScenarioBufferData151_BOX_9X9_08U, 48, gScenarioKernelData151_BOX_9X9_08U, 8},
{22, 6, 1148, 0, 1, gScenarioBufferData152_BOX_9X9_08U, 48, gScenarioKernelData152_BOX_9X9_08U, 8},
{24, 6, 1248, 0, 1, gScenarioBufferData153_BOX_9X9_08U, 48, gScenarioKernelData153_BOX_9X9_08U, 8},
{26, 6, 1348, 0, 1, gScenarioBufferData154_BOX_9X9_08U, 48, gScenarioKernelData154_BOX_9X9_08U, 8},
{28, 6, 1448, 0, 1, gScenarioBufferData155_BOX_9X9_08U, 48, gScenarioKernelData155_BOX_9X9_08U, 8},
{30, 6, 1548, 0, 1, gScenarioBufferData156_BOX_9X9_08U, 48, gScenarioKernelData156_BOX_9X9_08U, 8},
{32, 6, 1648, 0, 1, gScenarioBufferData157_BOX_9X9_08U, 48, gScenarioKernelData157_BOX_9X9_08U, 8},
{36, 6, 1848, 0, 1, gScenarioBufferData158_BOX_9X9_08U, 48, gScenarioKernelData158_BOX_9X9_08U, 8},
{40, 6, 2048, 0, 1, gScenarioBufferData159_BOX_9X9_08U, 48, gScenarioKernelData159_BOX_9X9_08U, 8},
{44, 6, 2248, 0, 1, gScenarioBufferData160_BOX_9X9_08U, 48, gScenarioKernelData160_BOX_9X9_08U, 8},
{48, 6, 2448, 0, 1, gScenarioBufferData161_BOX_9X9_08U, 48, gScenarioKernelData161_BOX_9X9_08U, 8},
{52, 6, 2648, 0, 1, gScenarioBufferData162_BOX_9X9_08U, 48, gScenarioKernelData162_BOX_9X9_08U, 8},
{56, 6, 2848, 0, 1, gScenarioBufferData163_BOX_9X9_08U, 48, gScenarioKernelData163_BOX_9X9_08U, 8},
{60, 6, 3048, 0, 1, gScenarioBufferData164_BOX_9X9_08U, 48, gScenarioKernelData164_BOX_9X9_08U, 8},
{64, 6, 3248, 0, 1, gScenarioBufferData165_BOX_9X9_08U, 48, gScenarioKernelData165_BOX_9X9_08U, 8},
{72, 6, 3648, 0, 1, gScenarioBufferData166_BOX_9X9_08U, 48, gScenarioKernelData166_BOX_9X9_08U, 8},
{4, 8, 320, 0, 1, gScenarioBufferData167_BOX_9X9_08U, 48, gScenarioKernelData167_BOX_9X9_08U, 8},
{6, 8, 448, 0, 1, gScenarioBufferData168_BOX_9X9_08U, 48, gScenarioKernelData168_BOX_9X9_08U, 8},
{8, 8, 576, 0, 1, gScenarioBufferData169_BOX_9X9_08U, 48, gScenarioKernelData169_BOX_9X9_08U, 8},
{10, 8, 704, 0, 1, gScenarioBufferData170_BOX_9X9_08U, 48, gScenarioKernelData170_BOX_9X9_08U, 8},
{12, 8, 832, 0, 1, gScenarioBufferData171_BOX_9X9_08U, 48, gScenarioKernelData171_BOX_9X9_08U, 8},
{14, 8, 960, 0, 1, gScenarioBufferData172_BOX_9X9_08U, 48, gScenarioKernelData172_BOX_9X9_08U, 8},
{16, 8, 1088, 0, 1, gScenarioBufferData173_BOX_9X9_08U, 48, gScenarioKernelData173_BOX_9X9_08U, 8},
{18, 8, 1216, 0, 1, gScenarioBufferData174_BOX_9X9_08U, 48, gScenarioKernelData174_BOX_9X9_08U, 8},
{20, 8, 1344, 0, 1, gScenarioBufferData175_BOX_9X9_08U, 48, gScenarioKernelData175_BOX_9X9_08U, 8},
{22, 8, 1472, 0, 1, gScenarioBufferData176_BOX_9X9_08U, 48, gScenarioKernelData176_BOX_9X9_08U, 8},
{24, 8, 1600, 0, 1, gScenarioBufferData177_BOX_9X9_08U, 48, gScenarioKernelData177_BOX_9X9_08U, 8},
{26, 8, 1728, 0, 1, gScenarioBufferData178_BOX_9X9_08U, 48, gScenarioKernelData178_BOX_9X9_08U, 8},
{28, 8, 1856, 0, 1, gScenarioBufferData179_BOX_9X9_08U, 48, gScenarioKernelData179_BOX_9X9_08U, 8},
{30, 8, 1984, 0, 1, gScenarioBufferData180_BOX_9X9_08U, 48, gScenarioKernelData180_BOX_9X9_08U, 8},
{32, 8, 2112, 0, 1, gScenarioBufferData181_BOX_9X9_08U, 48, gScenarioKernelData181_BOX_9X9_08U, 8},
{36, 8, 2368, 0, 1, gScenarioBufferData182_BOX_9X9_08U, 48, gScenarioKernelData182_BOX_9X9_08U, 8},
{40, 8, 2624, 0, 1, gScenarioBufferData183_BOX_9X9_08U, 48, gScenarioKernelData183_BOX_9X9_08U, 8},
{44, 8, 2880, 0, 1, gScenarioBufferData184_BOX_9X9_08U, 48, gScenarioKernelData184_BOX_9X9_08U, 8},
{48, 8, 3136, 0, 1, gScenarioBufferData185_BOX_9X9_08U, 48, gScenarioKernelData185_BOX_9X9_08U, 8},
{52, 8, 3392, 0, 1, gScenarioBufferData186_BOX_9X9_08U, 48, gScenarioKernelData186_BOX_9X9_08U, 8},
{56, 8, 3648, 0, 1, gScenarioBufferData187_BOX_9X9_08U, 48, gScenarioKernelData187_BOX_9X9_08U, 8},
{4, 10, 392, 0, 1, gScenarioBufferData188_BOX_9X9_08U, 48, gScenarioKernelData188_BOX_9X9_08U, 8},
{6, 10, 548, 0, 1, gScenarioBufferData189_BOX_9X9_08U, 48, gScenarioKernelData189_BOX_9X9_08U, 8},
{8, 10, 704, 0, 1, gScenarioBufferData190_BOX_9X9_08U, 48, gScenarioKernelData190_BOX_9X9_08U, 8},
{10, 10, 860, 0, 1, gScenarioBufferData191_BOX_9X9_08U, 48, gScenarioKernelData191_BOX_9X9_08U, 8},
{12, 10, 1016, 0, 1, gScenarioBufferData192_BOX_9X9_08U, 48, gScenarioKernelData192_BOX_9X9_08U, 8},
{14, 10, 1172, 0, 1, gScenarioBufferData193_BOX_9X9_08U, 48, gScenarioKernelData193_BOX_9X9_08U, 8},
{16, 10, 1328, 0, 1, gScenarioBufferData194_BOX_9X9_08U, 48, gScenarioKernelData194_BOX_9X9_08U, 8},
{18, 10, 1484, 0, 1, gScenarioBufferData195_BOX_9X9_08U, 48, gScenarioKernelData195_BOX_9X9_08U, 8},
{20, 10, 1640, 0, 1, gScenarioBufferData196_BOX_9X9_08U, 48, gScenarioKernelData196_BOX_9X9_08U, 8},
{22, 10, 1796, 0, 1, gScenarioBufferData197_BOX_9X9_08U, 48, gScenarioKernelData197_BOX_9X9_08U, 8},
{24, 10, 1952, 0, 1, gScenarioBufferData198_BOX_9X9_08U, 48, gScenarioKernelData198_BOX_9X9_08U, 8},
{26, 10, 2108, 0, 1, gScenarioBufferData199_BOX_9X9_08U, 48, gScenarioKernelData199_BOX_9X9_08U, 8},
{28, 10, 2264, 0, 1, gScenarioBufferData200_BOX_9X9_08U, 48, gScenarioKernelData200_BOX_9X9_08U, 8},
{30, 10, 2420, 0, 1, gScenarioBufferData201_BOX_9X9_08U, 48, gScenarioKernelData201_BOX_9X9_08U, 8},
{32, 10, 2576, 0, 1, gScenarioBufferData202_BOX_9X9_08U, 48, gScenarioKernelData202_BOX_9X9_08U, 8},
{36, 10, 2888, 0, 1, gScenarioBufferData203_BOX_9X9_08U, 48, gScenarioKernelData203_BOX_9X9_08U, 8},
{40, 10, 3200, 0, 1, gScenarioBufferData204_BOX_9X9_08U, 48, gScenarioKernelData204_BOX_9X9_08U, 8},
{44, 10, 3512, 0, 1, gScenarioBufferData205_BOX_9X9_08U, 48, gScenarioKernelData205_BOX_9X9_08U, 8},
{48, 10, 3824, 0, 1, gScenarioBufferData206_BOX_9X9_08U, 48, gScenarioKernelData206_BOX_9X9_08U, 8},
{4, 12, 464, 0, 1, gScenarioBufferData207_BOX_9X9_08U, 48, gScenarioKernelData207_BOX_9X9_08U, 8},
{6, 12, 648, 0, 1, gScenarioBufferData208_BOX_9X9_08U, 48, gScenarioKernelData208_BOX_9X9_08U, 8},
{8, 12, 832, 0, 1, gScenarioBufferData209_BOX_9X9_08U, 48, gScenarioKernelData209_BOX_9X9_08U, 8},
{10, 12, 1016, 0, 1, gScenarioBufferData210_BOX_9X9_08U, 48, gScenarioKernelData210_BOX_9X9_08U, 8},
{12, 12, 1200, 0, 1, gScenarioBufferData211_BOX_9X9_08U, 48, gScenarioKernelData211_BOX_9X9_08U, 8},
{14, 12, 1384, 0, 1, gScenarioBufferData212_BOX_9X9_08U, 48, gScenarioKernelData212_BOX_9X9_08U, 8},
{16, 12, 1568, 0, 1, gScenarioBufferData213_BOX_9X9_08U, 48, gScenarioKernelData213_BOX_9X9_08U, 8},
{18, 12, 1752, 0, 1, gScenarioBufferData214_BOX_9X9_08U, 48, gScenarioKernelData214_BOX_9X9_08U, 8},
{20, 12, 1936, 0, 1, gScenarioBufferData215_BOX_9X9_08U, 48, gScenarioKernelData215_BOX_9X9_08U, 8},
{22, 12, 2120, 0, 1, gScenarioBufferData216_BOX_9X9_08U, 48, gScenarioKernelData216_BOX_9X9_08U, 8},
{24, 12, 2304, 0, 1, gScenarioBufferData217_BOX_9X9_08U, 48, gScenarioKernelData217_BOX_9X9_08U, 8},
{26, 12, 2488, 0, 1, gScenarioBufferData218_BOX_9X9_08U, 48, gScenarioKernelData218_BOX_9X9_08U, 8},
{28, 12, 2672, 0, 1, gScenarioBufferData219_BOX_9X9_08U, 48, gScenarioKernelData219_BOX_9X9_08U, 8},
{30, 12, 2856, 0, 1, gScenarioBufferData220_BOX_9X9_08U, 48, gScenarioKernelData220_BOX_9X9_08U, 8},
{32, 12, 3040, 0, 1, gScenarioBufferData221_BOX_9X9_08U, 48, gScenarioKernelData221_BOX_9X9_08U, 8},
{36, 12, 3408, 0, 1, gScenarioBufferData222_BOX_9X9_08U, 48, gScenarioKernelData222_BOX_9X9_08U, 8},
{40, 12, 3776, 0, 1, gScenarioBufferData223_BOX_9X9_08U, 48, gScenarioKernelData223_BOX_9X9_08U, 8},
{4, 14, 536, 0, 1, gScenarioBufferData224_BOX_9X9_08U, 48, gScenarioKernelData224_BOX_9X9_08U, 8},
{6, 14, 748, 0, 1, gScenarioBufferData225_BOX_9X9_08U, 48, gScenarioKernelData225_BOX_9X9_08U, 8},
{8, 14, 960, 0, 1, gScenarioBufferData226_BOX_9X9_08U, 48, gScenarioKernelData226_BOX_9X9_08U, 8},
{10, 14, 1172, 0, 1, gScenarioBufferData227_BOX_9X9_08U, 48, gScenarioKernelData227_BOX_9X9_08U, 8},
{12, 14, 1384, 0, 1, gScenarioBufferData228_BOX_9X9_08U, 48, gScenarioKernelData228_BOX_9X9_08U, 8},
{14, 14, 1596, 0, 1, gScenarioBufferData229_BOX_9X9_08U, 48, gScenarioKernelData229_BOX_9X9_08U, 8},
{16, 14, 1808, 0, 1, gScenarioBufferData230_BOX_9X9_08U, 48, gScenarioKernelData230_BOX_9X9_08U, 8},
{18, 14, 2020, 0, 1, gScenarioBufferData231_BOX_9X9_08U, 48, gScenarioKernelData231_BOX_9X9_08U, 8},
{20, 14, 2232, 0, 1, gScenarioBufferData232_BOX_9X9_08U, 48, gScenarioKernelData232_BOX_9X9_08U, 8},
{22, 14, 2444, 0, 1, gScenarioBufferData233_BOX_9X9_08U, 48, gScenarioKernelData233_BOX_9X9_08U, 8},
{24, 14, 2656, 0, 1, gScenarioBufferData234_BOX_9X9_08U, 48, gScenarioKernelData234_BOX_9X9_08U, 8},
{26, 14, 2868, 0, 1, gScenarioBufferData235_BOX_9X9_08U, 48, gScenarioKernelData235_BOX_9X9_08U, 8},
{28, 14, 3080, 0, 1, gScenarioBufferData236_BOX_9X9_08U, 48, gScenarioKernelData236_BOX_9X9_08U, 8},
{30, 14, 3292, 0, 1, gScenarioBufferData237_BOX_9X9_08U, 48, gScenarioKernelData237_BOX_9X9_08U, 8},
{32, 14, 3504, 0, 1, gScenarioBufferData238_BOX_9X9_08U, 48, gScenarioKernelData238_BOX_9X9_08U, 8},
{4, 16, 608, 0, 1, gScenarioBufferData239_BOX_9X9_08U, 48, gScenarioKernelData239_BOX_9X9_08U, 8},
{6, 16, 848, 0, 1, gScenarioBufferData240_BOX_9X9_08U, 48, gScenarioKernelData240_BOX_9X9_08U, 8},
{8, 16, 1088, 0, 1, gScenarioBufferData241_BOX_9X9_08U, 48, gScenarioKernelData241_BOX_9X9_08U, 8},
{10, 16, 1328, 0, 1, gScenarioBufferData242_BOX_9X9_08U, 48, gScenarioKernelData242_BOX_9X9_08U, 8},
{12, 16, 1568, 0, 1, gScenarioBufferData243_BOX_9X9_08U, 48, gScenarioKernelData243_BOX_9X9_08U, 8},
{14, 16, 1808, 0, 1, gScenarioBufferData244_BOX_9X9_08U, 48, gScenarioKernelData244_BOX_9X9_08U, 8},
{16, 16, 2048, 0, 1, gScenarioBufferData245_BOX_9X9_08U, 48, gScenarioKernelData245_BOX_9X9_08U, 8},
{18, 16, 2288, 0, 1, gScenarioBufferData246_BOX_9X9_08U, 48, gScenarioKernelData246_BOX_9X9_08U, 8},
{20, 16, 2528, 0, 1, gScenarioBufferData247_BOX_9X9_08U, 48, gScenarioKernelData247_BOX_9X9_08U, 8},
{22, 16, 2768, 0, 1, gScenarioBufferData248_BOX_9X9_08U, 48, gScenarioKernelData248_BOX_9X9_08U, 8},
{24, 16, 3008, 0, 1, gScenarioBufferData249_BOX_9X9_08U, 48, gScenarioKernelData249_BOX_9X9_08U, 8},
{26, 16, 3248, 0, 1, gScenarioBufferData250_BOX_9X9_08U, 48, gScenarioKernelData250_BOX_9X9_08U, 8},
{28, 16, 3488, 0, 1, gScenarioBufferData251_BOX_9X9_08U, 48, gScenarioKernelData251_BOX_9X9_08U, 8},
{30, 16, 3728, 0, 1, gScenarioBufferData252_BOX_9X9_08U, 48, gScenarioKernelData252_BOX_9X9_08U, 8},
{4, 18, 680, 0, 1, gScenarioBufferData253_BOX_9X9_08U, 48, gScenarioKernelData253_BOX_9X9_08U, 8},
{6, 18, 948, 0, 1, gScenarioBufferData254_BOX_9X9_08U, 48, gScenarioKernelData254_BOX_9X9_08U, 8},
{8, 18, 1216, 0, 1, gScenarioBufferData255_BOX_9X9_08U, 48, gScenarioKernelData255_BOX_9X9_08U, 8},
{10, 18, 1484, 0, 1, gScenarioBufferData256_BOX_9X9_08U, 48, gScenarioKernelData256_BOX_9X9_08U, 8},
{12, 18, 1752, 0, 1, gScenarioBufferData257_BOX_9X9_08U, 48, gScenarioKernelData257_BOX_9X9_08U, 8},
{14, 18, 2020, 0, 1, gScenarioBufferData258_BOX_9X9_08U, 48, gScenarioKernelData258_BOX_9X9_08U, 8},
{16, 18, 2288, 0, 1, gScenarioBufferData259_BOX_9X9_08U, 48, gScenarioKernelData259_BOX_9X9_08U, 8},
{18, 18, 2556, 0, 1, gScenarioBufferData260_BOX_9X9_08U, 48, gScenarioKernelData260_BOX_9X9_08U, 8},
{20, 18, 2824, 0, 1, gScenarioBufferData261_BOX_9X9_08U, 48, gScenarioKernelData261_BOX_9X9_08U, 8},
{22, 18, 3092, 0, 1, gScenarioBufferData262_BOX_9X9_08U, 48, gScenarioKernelData262_BOX_9X9_08U, 8},
{24, 18, 3360, 0, 1, gScenarioBufferData263_BOX_9X9_08U, 48, gScenarioKernelData263_BOX_9X9_08U, 8},
{26, 18, 3628, 0, 1, gScenarioBufferData264_BOX_9X9_08U, 48, gScenarioKernelData264_BOX_9X9_08U, 8},
{4, 20, 752, 0, 1, gScenarioBufferData265_BOX_9X9_08U, 48, gScenarioKernelData265_BOX_9X9_08U, 8},
{6, 20, 1048, 0, 1, gScenarioBufferData266_BOX_9X9_08U, 48, gScenarioKernelData266_BOX_9X9_08U, 8},
{8, 20, 1344, 0, 1, gScenarioBufferData267_BOX_9X9_08U, 48, gScenarioKernelData267_BOX_9X9_08U, 8},
{10, 20, 1640, 0, 1, gScenarioBufferData268_BOX_9X9_08U, 48, gScenarioKernelData268_BOX_9X9_08U, 8},
{12, 20, 1936, 0, 1, gScenarioBufferData269_BOX_9X9_08U, 48, gScenarioKernelData269_BOX_9X9_08U, 8},
{14, 20, 2232, 0, 1, gScenarioBufferData270_BOX_9X9_08U, 48, gScenarioKernelData270_BOX_9X9_08U, 8},
{16, 20, 2528, 0, 1, gScenarioBufferData271_BOX_9X9_08U, 48, gScenarioKernelData271_BOX_9X9_08U, 8},
{18, 20, 2824, 0, 1, gScenarioBufferData272_BOX_9X9_08U, 48, gScenarioKernelData272_BOX_9X9_08U, 8},
{20, 20, 3120, 0, 1, gScenarioBufferData273_BOX_9X9_08U, 48, gScenarioKernelData273_BOX_9X9_08U, 8},
{22, 20, 3416, 0, 1, gScenarioBufferData274_BOX_9X9_08U, 48, gScenarioKernelData274_BOX_9X9_08U, 8},
{24, 20, 3712, 0, 1, gScenarioBufferData275_BOX_9X9_08U, 48, gScenarioKernelData275_BOX_9X9_08U, 8},
{4, 22, 824, 0, 1, gScenarioBufferData276_BOX_9X9_08U, 48, gScenarioKernelData276_BOX_9X9_08U, 8},
{6, 22, 1148, 0, 1, gScenarioBufferData277_BOX_9X9_08U, 48, gScenarioKernelData277_BOX_9X9_08U, 8},
{8, 22, 1472, 0, 1, gScenarioBufferData278_BOX_9X9_08U, 48, gScenarioKernelData278_BOX_9X9_08U, 8},
{10, 22, 1796, 0, 1, gScenarioBufferData279_BOX_9X9_08U, 48, gScenarioKernelData279_BOX_9X9_08U, 8},
{12, 22, 2120, 0, 1, gScenarioBufferData280_BOX_9X9_08U, 48, gScenarioKernelData280_BOX_9X9_08U, 8},
{14, 22, 2444, 0, 1, gScenarioBufferData281_BOX_9X9_08U, 48, gScenarioKernelData281_BOX_9X9_08U, 8},
{16, 22, 2768, 0, 1, gScenarioBufferData282_BOX_9X9_08U, 48, gScenarioKernelData282_BOX_9X9_08U, 8},
{18, 22, 3092, 0, 1, gScenarioBufferData283_BOX_9X9_08U, 48, gScenarioKernelData283_BOX_9X9_08U, 8},
{20, 22, 3416, 0, 1, gScenarioBufferData284_BOX_9X9_08U, 48, gScenarioKernelData284_BOX_9X9_08U, 8},
{22, 22, 3740, 0, 1, gScenarioBufferData285_BOX_9X9_08U, 48, gScenarioKernelData285_BOX_9X9_08U, 8},
{4, 24, 896, 0, 1, gScenarioBufferData286_BOX_9X9_08U, 48, gScenarioKernelData286_BOX_9X9_08U, 8},
{6, 24, 1248, 0, 1, gScenarioBufferData287_BOX_9X9_08U, 48, gScenarioKernelData287_BOX_9X9_08U, 8},
{8, 24, 1600, 0, 1, gScenarioBufferData288_BOX_9X9_08U, 48, gScenarioKernelData288_BOX_9X9_08U, 8},
{10, 24, 1952, 0, 1, gScenarioBufferData289_BOX_9X9_08U, 48, gScenarioKernelData289_BOX_9X9_08U, 8},
{12, 24, 2304, 0, 1, gScenarioBufferData290_BOX_9X9_08U, 48, gScenarioKernelData290_BOX_9X9_08U, 8},
{14, 24, 2656, 0, 1, gScenarioBufferData291_BOX_9X9_08U, 48, gScenarioKernelData291_BOX_9X9_08U, 8},
{16, 24, 3008, 0, 1, gScenarioBufferData292_BOX_9X9_08U, 48, gScenarioKernelData292_BOX_9X9_08U, 8},
{18, 24, 3360, 0, 1, gScenarioBufferData293_BOX_9X9_08U, 48, gScenarioKernelData293_BOX_9X9_08U, 8},
{20, 24, 3712, 0, 1, gScenarioBufferData294_BOX_9X9_08U, 48, gScenarioKernelData294_BOX_9X9_08U, 8},
{4, 26, 968, 0, 1, gScenarioBufferData295_BOX_9X9_08U, 48, gScenarioKernelData295_BOX_9X9_08U, 8},
{6, 26, 1348, 0, 1, gScenarioBufferData296_BOX_9X9_08U, 48, gScenarioKernelData296_BOX_9X9_08U, 8},
{8, 26, 1728, 0, 1, gScenarioBufferData297_BOX_9X9_08U, 48, gScenarioKernelData297_BOX_9X9_08U, 8},
{10, 26, 2108, 0, 1, gScenarioBufferData298_BOX_9X9_08U, 48, gScenarioKernelData298_BOX_9X9_08U, 8},
{12, 26, 2488, 0, 1, gScenarioBufferData299_BOX_9X9_08U, 48, gScenarioKernelData299_BOX_9X9_08U, 8},
{14, 26, 2868, 0, 1, gScenarioBufferData300_BOX_9X9_08U, 48, gScenarioKernelData300_BOX_9X9_08U, 8},
{16, 26, 3248, 0, 1, gScenarioBufferData301_BOX_9X9_08U, 48, gScenarioKernelData301_BOX_9X9_08U, 8},
{18, 26, 3628, 0, 1, gScenarioBufferData302_BOX_9X9_08U, 48, gScenarioKernelData302_BOX_9X9_08U, 8},
{4, 28, 1040, 0, 1, gScenarioBufferData303_BOX_9X9_08U, 48, gScenarioKernelData303_BOX_9X9_08U, 8},
{6, 28, 1448, 0, 1, gScenarioBufferData304_BOX_9X9_08U, 48, gScenarioKernelData304_BOX_9X9_08U, 8},
{8, 28, 1856, 0, 1, gScenarioBufferData305_BOX_9X9_08U, 48, gScenarioKernelData305_BOX_9X9_08U, 8},
{10, 28, 2264, 0, 1, gScenarioBufferData306_BOX_9X9_08U, 48, gScenarioKernelData306_BOX_9X9_08U, 8},
{12, 28, 2672, 0, 1, gScenarioBufferData307_BOX_9X9_08U, 48, gScenarioKernelData307_BOX_9X9_08U, 8},
{14, 28, 3080, 0, 1, gScenarioBufferData308_BOX_9X9_08U, 48, gScenarioKernelData308_BOX_9X9_08U, 8},
{16, 28, 3488, 0, 1, gScenarioBufferData309_BOX_9X9_08U, 48, gScenarioKernelData309_BOX_9X9_08U, 8},
{4, 30, 1112, 0, 1, gScenarioBufferData310_BOX_9X9_08U, 48, gScenarioKernelData310_BOX_9X9_08U, 8},
{6, 30, 1548, 0, 1, gScenarioBufferData311_BOX_9X9_08U, 48, gScenarioKernelData311_BOX_9X9_08U, 8},
{8, 30, 1984, 0, 1, gScenarioBufferData312_BOX_9X9_08U, 48, gScenarioKernelData312_BOX_9X9_08U, 8},
{10, 30, 2420, 0, 1, gScenarioBufferData313_BOX_9X9_08U, 48, gScenarioKernelData313_BOX_9X9_08U, 8},
{12, 30, 2856, 0, 1, gScenarioBufferData314_BOX_9X9_08U, 48, gScenarioKernelData314_BOX_9X9_08U, 8},
{14, 30, 3292, 0, 1, gScenarioBufferData315_BOX_9X9_08U, 48, gScenarioKernelData315_BOX_9X9_08U, 8},
{16, 30, 3728, 0, 1, gScenarioBufferData316_BOX_9X9_08U, 48, gScenarioKernelData316_BOX_9X9_08U, 8},
{4, 32, 1184, 0, 1, gScenarioBufferData317_BOX_9X9_08U, 48, gScenarioKernelData317_BOX_9X9_08U, 8},
{6, 32, 1648, 0, 1, gScenarioBufferData318_BOX_9X9_08U, 48, gScenarioKernelData318_BOX_9X9_08U, 8},
{8, 32, 2112, 0, 1, gScenarioBufferData319_BOX_9X9_08U, 48, gScenarioKernelData319_BOX_9X9_08U, 8},
{10, 32, 2576, 0, 1, gScenarioBufferData320_BOX_9X9_08U, 48, gScenarioKernelData320_BOX_9X9_08U, 8},
{12, 32, 3040, 0, 1, gScenarioBufferData321_BOX_9X9_08U, 48, gScenarioKernelData321_BOX_9X9_08U, 8},
{14, 32, 3504, 0, 1, gScenarioBufferData322_BOX_9X9_08U, 48, gScenarioKernelData322_BOX_9X9_08U, 8}
};

static acf_scenario_list gScenarioList_BOX_9X9_08U = {
323, //number of scenarios
gScenarioArray_BOX_9X9_08U};
//**************************************************************

class BOX_9X9_08U : public ACF_Process_APU
{

public:
   BOX_9X9_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("BOX_9X9_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("BOX_9X9_08U",
                                        BOX_9X9_08U_LOAD_SEGMENTS,
                                        BOX_9X9_08U_LOAD_PMEM, BOX_9X9_08U_LOAD_PMEM_SIZE,
                                        BOX_9X9_08U_LOAD_DMEM, BOX_9X9_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(BOX_9X9_08U_APEX_LOG_BUFFER, BOX_9X9_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_BOX_9X9_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_BOX_9X9_08U

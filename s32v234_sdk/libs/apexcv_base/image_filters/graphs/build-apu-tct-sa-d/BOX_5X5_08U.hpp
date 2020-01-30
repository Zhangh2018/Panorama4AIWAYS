#ifndef _ACF_PROCESS_APU_BOX_5X5_08U
#define _ACF_PROCESS_APU_BOX_5X5_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <BOX_5X5_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_BOX_5X5_08U[] = {{2, 1, 7, 0, {2, 2, 0, 0}}, {2, 1, 1, 2, {0, 0, 2, 2}}, {2, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_BOX_5X5_08U[] = {{4, 1, 7, 0, {2, 2, 0, 0}}, {4, 1, 1, 2, {0, 0, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_BOX_5X5_08U[] = {{6, 1, 7, 0, {2, 2, 0, 0}}, {6, 1, 1, 2, {0, 0, 2, 2}}, {6, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_BOX_5X5_08U[] = {{8, 1, 7, 0, {2, 2, 0, 0}}, {8, 1, 1, 2, {0, 0, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_BOX_5X5_08U[] = {{10, 1, 7, 0, {2, 2, 0, 0}}, {10, 1, 1, 2, {0, 0, 2, 2}}, {10, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_BOX_5X5_08U[] = {{12, 1, 7, 0, {2, 2, 0, 0}}, {12, 1, 1, 2, {0, 0, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_BOX_5X5_08U[] = {{14, 1, 7, 0, {2, 2, 0, 0}}, {14, 1, 1, 2, {0, 0, 2, 2}}, {14, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_BOX_5X5_08U[] = {{16, 1, 7, 0, {2, 2, 0, 0}}, {16, 1, 1, 2, {0, 0, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_BOX_5X5_08U[] = {{18, 1, 7, 0, {2, 2, 0, 0}}, {18, 1, 1, 2, {0, 0, 2, 2}}, {18, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_BOX_5X5_08U[] = {{20, 1, 7, 0, {2, 2, 0, 0}}, {20, 1, 1, 2, {0, 0, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_BOX_5X5_08U[] = {{22, 1, 7, 0, {2, 2, 0, 0}}, {22, 1, 1, 2, {0, 0, 2, 2}}, {22, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_BOX_5X5_08U[] = {{24, 1, 7, 0, {2, 2, 0, 0}}, {24, 1, 1, 2, {0, 0, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_BOX_5X5_08U[] = {{26, 1, 7, 0, {2, 2, 0, 0}}, {26, 1, 1, 2, {0, 0, 2, 2}}, {26, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_BOX_5X5_08U[] = {{28, 1, 7, 0, {2, 2, 0, 0}}, {28, 1, 1, 2, {0, 0, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_BOX_5X5_08U[] = {{30, 1, 7, 0, {2, 2, 0, 0}}, {30, 1, 1, 2, {0, 0, 2, 2}}, {30, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_BOX_5X5_08U[] = {{32, 1, 7, 0, {2, 2, 0, 0}}, {32, 1, 1, 2, {0, 0, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_BOX_5X5_08U[] = {{36, 1, 7, 0, {2, 2, 0, 0}}, {36, 1, 1, 2, {0, 0, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_BOX_5X5_08U[] = {{40, 1, 7, 0, {2, 2, 0, 0}}, {40, 1, 1, 2, {0, 0, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_BOX_5X5_08U[] = {{44, 1, 7, 0, {2, 2, 0, 0}}, {44, 1, 1, 2, {0, 0, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_BOX_5X5_08U[] = {{48, 1, 7, 0, {2, 2, 0, 0}}, {48, 1, 1, 2, {0, 0, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_BOX_5X5_08U[] = {{52, 1, 7, 0, {2, 2, 0, 0}}, {52, 1, 1, 2, {0, 0, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_BOX_5X5_08U[] = {{56, 1, 7, 0, {2, 2, 0, 0}}, {56, 1, 1, 2, {0, 0, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_BOX_5X5_08U[] = {{60, 1, 7, 0, {2, 2, 0, 0}}, {60, 1, 1, 2, {0, 0, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_BOX_5X5_08U[] = {{64, 1, 7, 0, {2, 2, 0, 0}}, {64, 1, 1, 2, {0, 0, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_BOX_5X5_08U[] = {{72, 1, 7, 0, {2, 2, 0, 0}}, {72, 1, 1, 2, {0, 0, 2, 2}}, {72, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_BOX_5X5_08U[] = {{80, 1, 7, 0, {2, 2, 0, 0}}, {80, 1, 1, 2, {0, 0, 2, 2}}, {80, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_BOX_5X5_08U[] = {{88, 1, 7, 0, {2, 2, 0, 0}}, {88, 1, 1, 2, {0, 0, 2, 2}}, {88, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_BOX_5X5_08U[] = {{96, 1, 7, 0, {2, 2, 0, 0}}, {96, 1, 1, 2, {0, 0, 2, 2}}, {96, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_BOX_5X5_08U[] = {{104, 1, 7, 0, {2, 2, 0, 0}}, {104, 1, 1, 2, {0, 0, 2, 2}}, {104, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_BOX_5X5_08U[] = {{112, 1, 7, 0, {2, 2, 0, 0}}, {112, 1, 1, 2, {0, 0, 2, 2}}, {112, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_BOX_5X5_08U[] = {{120, 1, 7, 0, {2, 2, 0, 0}}, {120, 1, 1, 2, {0, 0, 2, 2}}, {120, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_BOX_5X5_08U[] = {{128, 1, 7, 0, {2, 2, 0, 0}}, {128, 1, 1, 2, {0, 0, 2, 2}}, {128, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_BOX_5X5_08U[] = {{2, 2, 4, 0, {2, 2, 0, 0}}, {2, 2, 1, 1, {0, 0, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_BOX_5X5_08U[] = {{4, 2, 4, 0, {2, 2, 0, 0}}, {4, 2, 1, 1, {0, 0, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_BOX_5X5_08U[] = {{6, 2, 4, 0, {2, 2, 0, 0}}, {6, 2, 1, 1, {0, 0, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_BOX_5X5_08U[] = {{8, 2, 4, 0, {2, 2, 0, 0}}, {8, 2, 1, 1, {0, 0, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_BOX_5X5_08U[] = {{10, 2, 4, 0, {2, 2, 0, 0}}, {10, 2, 1, 1, {0, 0, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_BOX_5X5_08U[] = {{12, 2, 4, 0, {2, 2, 0, 0}}, {12, 2, 1, 1, {0, 0, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_BOX_5X5_08U[] = {{14, 2, 4, 0, {2, 2, 0, 0}}, {14, 2, 1, 1, {0, 0, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_BOX_5X5_08U[] = {{16, 2, 4, 0, {2, 2, 0, 0}}, {16, 2, 1, 1, {0, 0, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_BOX_5X5_08U[] = {{18, 2, 4, 0, {2, 2, 0, 0}}, {18, 2, 1, 1, {0, 0, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_BOX_5X5_08U[] = {{20, 2, 4, 0, {2, 2, 0, 0}}, {20, 2, 1, 1, {0, 0, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_BOX_5X5_08U[] = {{22, 2, 4, 0, {2, 2, 0, 0}}, {22, 2, 1, 1, {0, 0, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_BOX_5X5_08U[] = {{24, 2, 4, 0, {2, 2, 0, 0}}, {24, 2, 1, 1, {0, 0, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_BOX_5X5_08U[] = {{26, 2, 4, 0, {2, 2, 0, 0}}, {26, 2, 1, 1, {0, 0, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_BOX_5X5_08U[] = {{28, 2, 4, 0, {2, 2, 0, 0}}, {28, 2, 1, 1, {0, 0, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_BOX_5X5_08U[] = {{30, 2, 4, 0, {2, 2, 0, 0}}, {30, 2, 1, 1, {0, 0, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_BOX_5X5_08U[] = {{32, 2, 4, 0, {2, 2, 0, 0}}, {32, 2, 1, 1, {0, 0, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_BOX_5X5_08U[] = {{36, 2, 4, 0, {2, 2, 0, 0}}, {36, 2, 1, 1, {0, 0, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_BOX_5X5_08U[] = {{40, 2, 4, 0, {2, 2, 0, 0}}, {40, 2, 1, 1, {0, 0, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_BOX_5X5_08U[] = {{44, 2, 4, 0, {2, 2, 0, 0}}, {44, 2, 1, 1, {0, 0, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_BOX_5X5_08U[] = {{48, 2, 4, 0, {2, 2, 0, 0}}, {48, 2, 1, 1, {0, 0, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_BOX_5X5_08U[] = {{52, 2, 4, 0, {2, 2, 0, 0}}, {52, 2, 1, 1, {0, 0, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_BOX_5X5_08U[] = {{56, 2, 4, 0, {2, 2, 0, 0}}, {56, 2, 1, 1, {0, 0, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_BOX_5X5_08U[] = {{60, 2, 4, 0, {2, 2, 0, 0}}, {60, 2, 1, 1, {0, 0, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_BOX_5X5_08U[] = {{64, 2, 4, 0, {2, 2, 0, 0}}, {64, 2, 1, 1, {0, 0, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_BOX_5X5_08U[] = {{72, 2, 4, 0, {2, 2, 0, 0}}, {72, 2, 1, 1, {0, 0, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_BOX_5X5_08U[] = {{80, 2, 4, 0, {2, 2, 0, 0}}, {80, 2, 1, 1, {0, 0, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_BOX_5X5_08U[] = {{88, 2, 4, 0, {2, 2, 0, 0}}, {88, 2, 1, 1, {0, 0, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_BOX_5X5_08U[] = {{96, 2, 4, 0, {2, 2, 0, 0}}, {96, 2, 1, 1, {0, 0, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_BOX_5X5_08U[] = {{104, 2, 4, 0, {2, 2, 0, 0}}, {104, 2, 1, 1, {0, 0, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_BOX_5X5_08U[] = {{112, 2, 4, 0, {2, 2, 0, 0}}, {112, 2, 1, 1, {0, 0, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_BOX_5X5_08U[] = {{120, 2, 4, 0, {2, 2, 0, 0}}, {120, 2, 1, 1, {0, 0, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_BOX_5X5_08U[] = {{128, 2, 4, 0, {2, 2, 0, 0}}, {128, 2, 1, 1, {0, 0, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_BOX_5X5_08U[] = {{2, 3, 4, 0, {2, 2, 0, 0}}, {2, 3, 1, 1, {0, 0, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_BOX_5X5_08U[] = {{4, 3, 4, 0, {2, 2, 0, 0}}, {4, 3, 1, 1, {0, 0, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_BOX_5X5_08U[] = {{6, 3, 4, 0, {2, 2, 0, 0}}, {6, 3, 1, 1, {0, 0, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_BOX_5X5_08U[] = {{8, 3, 4, 0, {2, 2, 0, 0}}, {8, 3, 1, 1, {0, 0, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_BOX_5X5_08U[] = {{10, 3, 4, 0, {2, 2, 0, 0}}, {10, 3, 1, 1, {0, 0, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_BOX_5X5_08U[] = {{12, 3, 4, 0, {2, 2, 0, 0}}, {12, 3, 1, 1, {0, 0, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_BOX_5X5_08U[] = {{14, 3, 4, 0, {2, 2, 0, 0}}, {14, 3, 1, 1, {0, 0, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_BOX_5X5_08U[] = {{16, 3, 4, 0, {2, 2, 0, 0}}, {16, 3, 1, 1, {0, 0, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_BOX_5X5_08U[] = {{18, 3, 4, 0, {2, 2, 0, 0}}, {18, 3, 1, 1, {0, 0, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_BOX_5X5_08U[] = {{20, 3, 4, 0, {2, 2, 0, 0}}, {20, 3, 1, 1, {0, 0, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_BOX_5X5_08U[] = {{22, 3, 4, 0, {2, 2, 0, 0}}, {22, 3, 1, 1, {0, 0, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_BOX_5X5_08U[] = {{24, 3, 4, 0, {2, 2, 0, 0}}, {24, 3, 1, 1, {0, 0, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_BOX_5X5_08U[] = {{26, 3, 4, 0, {2, 2, 0, 0}}, {26, 3, 1, 1, {0, 0, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_BOX_5X5_08U[] = {{28, 3, 4, 0, {2, 2, 0, 0}}, {28, 3, 1, 1, {0, 0, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_BOX_5X5_08U[] = {{30, 3, 4, 0, {2, 2, 0, 0}}, {30, 3, 1, 1, {0, 0, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_BOX_5X5_08U[] = {{32, 3, 4, 0, {2, 2, 0, 0}}, {32, 3, 1, 1, {0, 0, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_BOX_5X5_08U[] = {{36, 3, 4, 0, {2, 2, 0, 0}}, {36, 3, 1, 1, {0, 0, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_BOX_5X5_08U[] = {{40, 3, 4, 0, {2, 2, 0, 0}}, {40, 3, 1, 1, {0, 0, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_BOX_5X5_08U[] = {{44, 3, 4, 0, {2, 2, 0, 0}}, {44, 3, 1, 1, {0, 0, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_BOX_5X5_08U[] = {{48, 3, 4, 0, {2, 2, 0, 0}}, {48, 3, 1, 1, {0, 0, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_BOX_5X5_08U[] = {{52, 3, 4, 0, {2, 2, 0, 0}}, {52, 3, 1, 1, {0, 0, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_BOX_5X5_08U[] = {{56, 3, 4, 0, {2, 2, 0, 0}}, {56, 3, 1, 1, {0, 0, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_BOX_5X5_08U[] = {{60, 3, 4, 0, {2, 2, 0, 0}}, {60, 3, 1, 1, {0, 0, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_BOX_5X5_08U[] = {{64, 3, 4, 0, {2, 2, 0, 0}}, {64, 3, 1, 1, {0, 0, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_BOX_5X5_08U[] = {{72, 3, 4, 0, {2, 2, 0, 0}}, {72, 3, 1, 1, {0, 0, 2, 2}}, {72, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_BOX_5X5_08U[] = {{80, 3, 4, 0, {2, 2, 0, 0}}, {80, 3, 1, 1, {0, 0, 2, 2}}, {80, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_BOX_5X5_08U[] = {{88, 3, 4, 0, {2, 2, 0, 0}}, {88, 3, 1, 1, {0, 0, 2, 2}}, {88, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_BOX_5X5_08U[] = {{96, 3, 4, 0, {2, 2, 0, 0}}, {96, 3, 1, 1, {0, 0, 2, 2}}, {96, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_BOX_5X5_08U[] = {{104, 3, 4, 0, {2, 2, 0, 0}}, {104, 3, 1, 1, {0, 0, 2, 2}}, {104, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_BOX_5X5_08U[] = {{112, 3, 4, 0, {2, 2, 0, 0}}, {112, 3, 1, 1, {0, 0, 2, 2}}, {112, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_BOX_5X5_08U[] = {{120, 3, 4, 0, {2, 2, 0, 0}}, {120, 3, 1, 1, {0, 0, 2, 2}}, {120, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_BOX_5X5_08U[] = {{128, 3, 4, 0, {2, 2, 0, 0}}, {128, 3, 1, 1, {0, 0, 2, 2}}, {128, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_BOX_5X5_08U[] = {{2, 4, 4, 0, {2, 2, 0, 0}}, {2, 4, 1, 1, {0, 0, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_BOX_5X5_08U[] = {{4, 4, 4, 0, {2, 2, 0, 0}}, {4, 4, 1, 1, {0, 0, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_BOX_5X5_08U[] = {{6, 4, 4, 0, {2, 2, 0, 0}}, {6, 4, 1, 1, {0, 0, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_BOX_5X5_08U[] = {{8, 4, 4, 0, {2, 2, 0, 0}}, {8, 4, 1, 1, {0, 0, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_BOX_5X5_08U[] = {{10, 4, 4, 0, {2, 2, 0, 0}}, {10, 4, 1, 1, {0, 0, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_BOX_5X5_08U[] = {{12, 4, 4, 0, {2, 2, 0, 0}}, {12, 4, 1, 1, {0, 0, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_BOX_5X5_08U[] = {{14, 4, 4, 0, {2, 2, 0, 0}}, {14, 4, 1, 1, {0, 0, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_BOX_5X5_08U[] = {{16, 4, 4, 0, {2, 2, 0, 0}}, {16, 4, 1, 1, {0, 0, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_BOX_5X5_08U[] = {{18, 4, 4, 0, {2, 2, 0, 0}}, {18, 4, 1, 1, {0, 0, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_BOX_5X5_08U[] = {{20, 4, 4, 0, {2, 2, 0, 0}}, {20, 4, 1, 1, {0, 0, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_BOX_5X5_08U[] = {{22, 4, 4, 0, {2, 2, 0, 0}}, {22, 4, 1, 1, {0, 0, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_BOX_5X5_08U[] = {{24, 4, 4, 0, {2, 2, 0, 0}}, {24, 4, 1, 1, {0, 0, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_BOX_5X5_08U[] = {{26, 4, 4, 0, {2, 2, 0, 0}}, {26, 4, 1, 1, {0, 0, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_BOX_5X5_08U[] = {{28, 4, 4, 0, {2, 2, 0, 0}}, {28, 4, 1, 1, {0, 0, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_BOX_5X5_08U[] = {{30, 4, 4, 0, {2, 2, 0, 0}}, {30, 4, 1, 1, {0, 0, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_BOX_5X5_08U[] = {{32, 4, 4, 0, {2, 2, 0, 0}}, {32, 4, 1, 1, {0, 0, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_BOX_5X5_08U[] = {{36, 4, 4, 0, {2, 2, 0, 0}}, {36, 4, 1, 1, {0, 0, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_BOX_5X5_08U[] = {{40, 4, 4, 0, {2, 2, 0, 0}}, {40, 4, 1, 1, {0, 0, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_BOX_5X5_08U[] = {{44, 4, 4, 0, {2, 2, 0, 0}}, {44, 4, 1, 1, {0, 0, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_BOX_5X5_08U[] = {{48, 4, 4, 0, {2, 2, 0, 0}}, {48, 4, 1, 1, {0, 0, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_BOX_5X5_08U[] = {{52, 4, 4, 0, {2, 2, 0, 0}}, {52, 4, 1, 1, {0, 0, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_BOX_5X5_08U[] = {{56, 4, 4, 0, {2, 2, 0, 0}}, {56, 4, 1, 1, {0, 0, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_BOX_5X5_08U[] = {{60, 4, 4, 0, {2, 2, 0, 0}}, {60, 4, 1, 1, {0, 0, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_BOX_5X5_08U[] = {{64, 4, 4, 0, {2, 2, 0, 0}}, {64, 4, 1, 1, {0, 0, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_BOX_5X5_08U[] = {{72, 4, 4, 0, {2, 2, 0, 0}}, {72, 4, 1, 1, {0, 0, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_BOX_5X5_08U[] = {{80, 4, 4, 0, {2, 2, 0, 0}}, {80, 4, 1, 1, {0, 0, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_BOX_5X5_08U[] = {{88, 4, 4, 0, {2, 2, 0, 0}}, {88, 4, 1, 1, {0, 0, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_BOX_5X5_08U[] = {{96, 4, 4, 0, {2, 2, 0, 0}}, {96, 4, 1, 1, {0, 0, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_BOX_5X5_08U[] = {{104, 4, 4, 0, {2, 2, 0, 0}}, {104, 4, 1, 1, {0, 0, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_BOX_5X5_08U[] = {{112, 4, 4, 0, {2, 2, 0, 0}}, {112, 4, 1, 1, {0, 0, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_BOX_5X5_08U[] = {{2, 5, 4, 0, {2, 2, 0, 0}}, {2, 5, 1, 1, {0, 0, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_BOX_5X5_08U[] = {{4, 5, 4, 0, {2, 2, 0, 0}}, {4, 5, 1, 1, {0, 0, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_BOX_5X5_08U[] = {{6, 5, 4, 0, {2, 2, 0, 0}}, {6, 5, 1, 1, {0, 0, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_BOX_5X5_08U[] = {{8, 5, 4, 0, {2, 2, 0, 0}}, {8, 5, 1, 1, {0, 0, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_BOX_5X5_08U[] = {{10, 5, 4, 0, {2, 2, 0, 0}}, {10, 5, 1, 1, {0, 0, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_BOX_5X5_08U[] = {{12, 5, 4, 0, {2, 2, 0, 0}}, {12, 5, 1, 1, {0, 0, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_BOX_5X5_08U[] = {{14, 5, 4, 0, {2, 2, 0, 0}}, {14, 5, 1, 1, {0, 0, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_BOX_5X5_08U[] = {{16, 5, 4, 0, {2, 2, 0, 0}}, {16, 5, 1, 1, {0, 0, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_BOX_5X5_08U[] = {{18, 5, 4, 0, {2, 2, 0, 0}}, {18, 5, 1, 1, {0, 0, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_BOX_5X5_08U[] = {{20, 5, 4, 0, {2, 2, 0, 0}}, {20, 5, 1, 1, {0, 0, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_BOX_5X5_08U[] = {{22, 5, 4, 0, {2, 2, 0, 0}}, {22, 5, 1, 1, {0, 0, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_BOX_5X5_08U[] = {{24, 5, 4, 0, {2, 2, 0, 0}}, {24, 5, 1, 1, {0, 0, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_BOX_5X5_08U[] = {{26, 5, 4, 0, {2, 2, 0, 0}}, {26, 5, 1, 1, {0, 0, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_BOX_5X5_08U[] = {{28, 5, 4, 0, {2, 2, 0, 0}}, {28, 5, 1, 1, {0, 0, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_BOX_5X5_08U[] = {{30, 5, 4, 0, {2, 2, 0, 0}}, {30, 5, 1, 1, {0, 0, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_BOX_5X5_08U[] = {{32, 5, 4, 0, {2, 2, 0, 0}}, {32, 5, 1, 1, {0, 0, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_BOX_5X5_08U[] = {{36, 5, 4, 0, {2, 2, 0, 0}}, {36, 5, 1, 1, {0, 0, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_BOX_5X5_08U[] = {{40, 5, 4, 0, {2, 2, 0, 0}}, {40, 5, 1, 1, {0, 0, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_BOX_5X5_08U[] = {{44, 5, 4, 0, {2, 2, 0, 0}}, {44, 5, 1, 1, {0, 0, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_BOX_5X5_08U[] = {{48, 5, 4, 0, {2, 2, 0, 0}}, {48, 5, 1, 1, {0, 0, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_BOX_5X5_08U[] = {{52, 5, 4, 0, {2, 2, 0, 0}}, {52, 5, 1, 1, {0, 0, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_BOX_5X5_08U[] = {{56, 5, 4, 0, {2, 2, 0, 0}}, {56, 5, 1, 1, {0, 0, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_BOX_5X5_08U[] = {{60, 5, 4, 0, {2, 2, 0, 0}}, {60, 5, 1, 1, {0, 0, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_BOX_5X5_08U[] = {{64, 5, 4, 0, {2, 2, 0, 0}}, {64, 5, 1, 1, {0, 0, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_BOX_5X5_08U[] = {{72, 5, 4, 0, {2, 2, 0, 0}}, {72, 5, 1, 1, {0, 0, 2, 2}}, {72, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_BOX_5X5_08U[] = {{80, 5, 4, 0, {2, 2, 0, 0}}, {80, 5, 1, 1, {0, 0, 2, 2}}, {80, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_BOX_5X5_08U[] = {{88, 5, 4, 0, {2, 2, 0, 0}}, {88, 5, 1, 1, {0, 0, 2, 2}}, {88, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_BOX_5X5_08U[] = {{96, 5, 4, 0, {2, 2, 0, 0}}, {96, 5, 1, 1, {0, 0, 2, 2}}, {96, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_BOX_5X5_08U[] = {{2, 6, 4, 0, {2, 2, 0, 0}}, {2, 6, 1, 1, {0, 0, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_BOX_5X5_08U[] = {{4, 6, 4, 0, {2, 2, 0, 0}}, {4, 6, 1, 1, {0, 0, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_BOX_5X5_08U[] = {{6, 6, 4, 0, {2, 2, 0, 0}}, {6, 6, 1, 1, {0, 0, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_BOX_5X5_08U[] = {{8, 6, 4, 0, {2, 2, 0, 0}}, {8, 6, 1, 1, {0, 0, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_BOX_5X5_08U[] = {{10, 6, 4, 0, {2, 2, 0, 0}}, {10, 6, 1, 1, {0, 0, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_BOX_5X5_08U[] = {{12, 6, 4, 0, {2, 2, 0, 0}}, {12, 6, 1, 1, {0, 0, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_BOX_5X5_08U[] = {{14, 6, 4, 0, {2, 2, 0, 0}}, {14, 6, 1, 1, {0, 0, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_BOX_5X5_08U[] = {{16, 6, 4, 0, {2, 2, 0, 0}}, {16, 6, 1, 1, {0, 0, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_BOX_5X5_08U[] = {{18, 6, 4, 0, {2, 2, 0, 0}}, {18, 6, 1, 1, {0, 0, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_BOX_5X5_08U[] = {{20, 6, 4, 0, {2, 2, 0, 0}}, {20, 6, 1, 1, {0, 0, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_BOX_5X5_08U[] = {{22, 6, 4, 0, {2, 2, 0, 0}}, {22, 6, 1, 1, {0, 0, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_BOX_5X5_08U[] = {{24, 6, 4, 0, {2, 2, 0, 0}}, {24, 6, 1, 1, {0, 0, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_BOX_5X5_08U[] = {{26, 6, 4, 0, {2, 2, 0, 0}}, {26, 6, 1, 1, {0, 0, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_BOX_5X5_08U[] = {{28, 6, 4, 0, {2, 2, 0, 0}}, {28, 6, 1, 1, {0, 0, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_BOX_5X5_08U[] = {{30, 6, 4, 0, {2, 2, 0, 0}}, {30, 6, 1, 1, {0, 0, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_BOX_5X5_08U[] = {{32, 6, 4, 0, {2, 2, 0, 0}}, {32, 6, 1, 1, {0, 0, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_BOX_5X5_08U[] = {{36, 6, 4, 0, {2, 2, 0, 0}}, {36, 6, 1, 1, {0, 0, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_BOX_5X5_08U[] = {{40, 6, 4, 0, {2, 2, 0, 0}}, {40, 6, 1, 1, {0, 0, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_BOX_5X5_08U[] = {{44, 6, 4, 0, {2, 2, 0, 0}}, {44, 6, 1, 1, {0, 0, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_BOX_5X5_08U[] = {{48, 6, 4, 0, {2, 2, 0, 0}}, {48, 6, 1, 1, {0, 0, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_BOX_5X5_08U[] = {{52, 6, 4, 0, {2, 2, 0, 0}}, {52, 6, 1, 1, {0, 0, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_BOX_5X5_08U[] = {{56, 6, 4, 0, {2, 2, 0, 0}}, {56, 6, 1, 1, {0, 0, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_BOX_5X5_08U[] = {{60, 6, 4, 0, {2, 2, 0, 0}}, {60, 6, 1, 1, {0, 0, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_BOX_5X5_08U[] = {{64, 6, 4, 0, {2, 2, 0, 0}}, {64, 6, 1, 1, {0, 0, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_BOX_5X5_08U[] = {{72, 6, 4, 0, {2, 2, 0, 0}}, {72, 6, 1, 1, {0, 0, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_BOX_5X5_08U[] = {{80, 6, 4, 0, {2, 2, 0, 0}}, {80, 6, 1, 1, {0, 0, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_BOX_5X5_08U[] = {{2, 8, 4, 0, {2, 2, 0, 0}}, {2, 8, 1, 1, {0, 0, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_BOX_5X5_08U[] = {{4, 8, 4, 0, {2, 2, 0, 0}}, {4, 8, 1, 1, {0, 0, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_BOX_5X5_08U[] = {{6, 8, 4, 0, {2, 2, 0, 0}}, {6, 8, 1, 1, {0, 0, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_BOX_5X5_08U[] = {{8, 8, 4, 0, {2, 2, 0, 0}}, {8, 8, 1, 1, {0, 0, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_BOX_5X5_08U[] = {{10, 8, 4, 0, {2, 2, 0, 0}}, {10, 8, 1, 1, {0, 0, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_BOX_5X5_08U[] = {{12, 8, 4, 0, {2, 2, 0, 0}}, {12, 8, 1, 1, {0, 0, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_BOX_5X5_08U[] = {{14, 8, 4, 0, {2, 2, 0, 0}}, {14, 8, 1, 1, {0, 0, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_BOX_5X5_08U[] = {{16, 8, 4, 0, {2, 2, 0, 0}}, {16, 8, 1, 1, {0, 0, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_BOX_5X5_08U[] = {{18, 8, 4, 0, {2, 2, 0, 0}}, {18, 8, 1, 1, {0, 0, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_BOX_5X5_08U[] = {{20, 8, 4, 0, {2, 2, 0, 0}}, {20, 8, 1, 1, {0, 0, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_BOX_5X5_08U[] = {{22, 8, 4, 0, {2, 2, 0, 0}}, {22, 8, 1, 1, {0, 0, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_BOX_5X5_08U[] = {{24, 8, 4, 0, {2, 2, 0, 0}}, {24, 8, 1, 1, {0, 0, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_BOX_5X5_08U[] = {{26, 8, 4, 0, {2, 2, 0, 0}}, {26, 8, 1, 1, {0, 0, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_BOX_5X5_08U[] = {{28, 8, 4, 0, {2, 2, 0, 0}}, {28, 8, 1, 1, {0, 0, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_BOX_5X5_08U[] = {{30, 8, 4, 0, {2, 2, 0, 0}}, {30, 8, 1, 1, {0, 0, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_BOX_5X5_08U[] = {{32, 8, 4, 0, {2, 2, 0, 0}}, {32, 8, 1, 1, {0, 0, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_BOX_5X5_08U[] = {{36, 8, 4, 0, {2, 2, 0, 0}}, {36, 8, 1, 1, {0, 0, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_BOX_5X5_08U[] = {{40, 8, 4, 0, {2, 2, 0, 0}}, {40, 8, 1, 1, {0, 0, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_BOX_5X5_08U[] = {{44, 8, 4, 0, {2, 2, 0, 0}}, {44, 8, 1, 1, {0, 0, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_BOX_5X5_08U[] = {{48, 8, 4, 0, {2, 2, 0, 0}}, {48, 8, 1, 1, {0, 0, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_BOX_5X5_08U[] = {{52, 8, 4, 0, {2, 2, 0, 0}}, {52, 8, 1, 1, {0, 0, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_BOX_5X5_08U[] = {{56, 8, 4, 0, {2, 2, 0, 0}}, {56, 8, 1, 1, {0, 0, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_BOX_5X5_08U[] = {{60, 8, 4, 0, {2, 2, 0, 0}}, {60, 8, 1, 1, {0, 0, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_BOX_5X5_08U[] = {{2, 10, 4, 0, {2, 2, 0, 0}}, {2, 10, 1, 1, {0, 0, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_BOX_5X5_08U[] = {{4, 10, 4, 0, {2, 2, 0, 0}}, {4, 10, 1, 1, {0, 0, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_BOX_5X5_08U[] = {{6, 10, 4, 0, {2, 2, 0, 0}}, {6, 10, 1, 1, {0, 0, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_BOX_5X5_08U[] = {{8, 10, 4, 0, {2, 2, 0, 0}}, {8, 10, 1, 1, {0, 0, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_BOX_5X5_08U[] = {{10, 10, 4, 0, {2, 2, 0, 0}}, {10, 10, 1, 1, {0, 0, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_BOX_5X5_08U[] = {{12, 10, 4, 0, {2, 2, 0, 0}}, {12, 10, 1, 1, {0, 0, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_BOX_5X5_08U[] = {{14, 10, 4, 0, {2, 2, 0, 0}}, {14, 10, 1, 1, {0, 0, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_BOX_5X5_08U[] = {{16, 10, 4, 0, {2, 2, 0, 0}}, {16, 10, 1, 1, {0, 0, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_BOX_5X5_08U[] = {{18, 10, 4, 0, {2, 2, 0, 0}}, {18, 10, 1, 1, {0, 0, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_BOX_5X5_08U[] = {{20, 10, 4, 0, {2, 2, 0, 0}}, {20, 10, 1, 1, {0, 0, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_BOX_5X5_08U[] = {{22, 10, 4, 0, {2, 2, 0, 0}}, {22, 10, 1, 1, {0, 0, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_BOX_5X5_08U[] = {{24, 10, 4, 0, {2, 2, 0, 0}}, {24, 10, 1, 1, {0, 0, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_BOX_5X5_08U[] = {{26, 10, 4, 0, {2, 2, 0, 0}}, {26, 10, 1, 1, {0, 0, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_BOX_5X5_08U[] = {{28, 10, 4, 0, {2, 2, 0, 0}}, {28, 10, 1, 1, {0, 0, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_BOX_5X5_08U[] = {{30, 10, 4, 0, {2, 2, 0, 0}}, {30, 10, 1, 1, {0, 0, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_BOX_5X5_08U[] = {{32, 10, 4, 0, {2, 2, 0, 0}}, {32, 10, 1, 1, {0, 0, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_BOX_5X5_08U[] = {{36, 10, 4, 0, {2, 2, 0, 0}}, {36, 10, 1, 1, {0, 0, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_BOX_5X5_08U[] = {{40, 10, 4, 0, {2, 2, 0, 0}}, {40, 10, 1, 1, {0, 0, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_BOX_5X5_08U[] = {{44, 10, 4, 0, {2, 2, 0, 0}}, {44, 10, 1, 1, {0, 0, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_BOX_5X5_08U[] = {{48, 10, 4, 0, {2, 2, 0, 0}}, {48, 10, 1, 1, {0, 0, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_BOX_5X5_08U[] = {{2, 12, 4, 0, {2, 2, 0, 0}}, {2, 12, 1, 1, {0, 0, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_BOX_5X5_08U[] = {{4, 12, 4, 0, {2, 2, 0, 0}}, {4, 12, 1, 1, {0, 0, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_BOX_5X5_08U[] = {{6, 12, 4, 0, {2, 2, 0, 0}}, {6, 12, 1, 1, {0, 0, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_BOX_5X5_08U[] = {{8, 12, 4, 0, {2, 2, 0, 0}}, {8, 12, 1, 1, {0, 0, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_BOX_5X5_08U[] = {{10, 12, 4, 0, {2, 2, 0, 0}}, {10, 12, 1, 1, {0, 0, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_BOX_5X5_08U[] = {{12, 12, 4, 0, {2, 2, 0, 0}}, {12, 12, 1, 1, {0, 0, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_BOX_5X5_08U[] = {{14, 12, 4, 0, {2, 2, 0, 0}}, {14, 12, 1, 1, {0, 0, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_BOX_5X5_08U[] = {{16, 12, 4, 0, {2, 2, 0, 0}}, {16, 12, 1, 1, {0, 0, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_BOX_5X5_08U[] = {{18, 12, 4, 0, {2, 2, 0, 0}}, {18, 12, 1, 1, {0, 0, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_BOX_5X5_08U[] = {{20, 12, 4, 0, {2, 2, 0, 0}}, {20, 12, 1, 1, {0, 0, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_BOX_5X5_08U[] = {{22, 12, 4, 0, {2, 2, 0, 0}}, {22, 12, 1, 1, {0, 0, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_BOX_5X5_08U[] = {{24, 12, 4, 0, {2, 2, 0, 0}}, {24, 12, 1, 1, {0, 0, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_BOX_5X5_08U[] = {{26, 12, 4, 0, {2, 2, 0, 0}}, {26, 12, 1, 1, {0, 0, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_BOX_5X5_08U[] = {{28, 12, 4, 0, {2, 2, 0, 0}}, {28, 12, 1, 1, {0, 0, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_BOX_5X5_08U[] = {{30, 12, 4, 0, {2, 2, 0, 0}}, {30, 12, 1, 1, {0, 0, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_BOX_5X5_08U[] = {{32, 12, 4, 0, {2, 2, 0, 0}}, {32, 12, 1, 1, {0, 0, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_BOX_5X5_08U[] = {{36, 12, 4, 0, {2, 2, 0, 0}}, {36, 12, 1, 1, {0, 0, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_BOX_5X5_08U[] = {{40, 12, 4, 0, {2, 2, 0, 0}}, {40, 12, 1, 1, {0, 0, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_BOX_5X5_08U[] = {{2, 14, 4, 0, {2, 2, 0, 0}}, {2, 14, 1, 1, {0, 0, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_BOX_5X5_08U[] = {{4, 14, 4, 0, {2, 2, 0, 0}}, {4, 14, 1, 1, {0, 0, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_BOX_5X5_08U[] = {{6, 14, 4, 0, {2, 2, 0, 0}}, {6, 14, 1, 1, {0, 0, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_BOX_5X5_08U[] = {{8, 14, 4, 0, {2, 2, 0, 0}}, {8, 14, 1, 1, {0, 0, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_BOX_5X5_08U[] = {{10, 14, 4, 0, {2, 2, 0, 0}}, {10, 14, 1, 1, {0, 0, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_BOX_5X5_08U[] = {{12, 14, 4, 0, {2, 2, 0, 0}}, {12, 14, 1, 1, {0, 0, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_BOX_5X5_08U[] = {{14, 14, 4, 0, {2, 2, 0, 0}}, {14, 14, 1, 1, {0, 0, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_BOX_5X5_08U[] = {{16, 14, 4, 0, {2, 2, 0, 0}}, {16, 14, 1, 1, {0, 0, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_BOX_5X5_08U[] = {{18, 14, 4, 0, {2, 2, 0, 0}}, {18, 14, 1, 1, {0, 0, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_BOX_5X5_08U[] = {{20, 14, 4, 0, {2, 2, 0, 0}}, {20, 14, 1, 1, {0, 0, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_BOX_5X5_08U[] = {{22, 14, 4, 0, {2, 2, 0, 0}}, {22, 14, 1, 1, {0, 0, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_BOX_5X5_08U[] = {{24, 14, 4, 0, {2, 2, 0, 0}}, {24, 14, 1, 1, {0, 0, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_BOX_5X5_08U[] = {{26, 14, 4, 0, {2, 2, 0, 0}}, {26, 14, 1, 1, {0, 0, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_BOX_5X5_08U[] = {{28, 14, 4, 0, {2, 2, 0, 0}}, {28, 14, 1, 1, {0, 0, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_BOX_5X5_08U[] = {{30, 14, 4, 0, {2, 2, 0, 0}}, {30, 14, 1, 1, {0, 0, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_BOX_5X5_08U[] = {{32, 14, 4, 0, {2, 2, 0, 0}}, {32, 14, 1, 1, {0, 0, 2, 2}}, {32, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_BOX_5X5_08U[] = {{36, 14, 4, 0, {2, 2, 0, 0}}, {36, 14, 1, 1, {0, 0, 2, 2}}, {36, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_BOX_5X5_08U[] = {{2, 16, 4, 0, {2, 2, 0, 0}}, {2, 16, 1, 1, {0, 0, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_BOX_5X5_08U[] = {{4, 16, 4, 0, {2, 2, 0, 0}}, {4, 16, 1, 1, {0, 0, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_BOX_5X5_08U[] = {{6, 16, 4, 0, {2, 2, 0, 0}}, {6, 16, 1, 1, {0, 0, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_BOX_5X5_08U[] = {{8, 16, 4, 0, {2, 2, 0, 0}}, {8, 16, 1, 1, {0, 0, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_BOX_5X5_08U[] = {{10, 16, 4, 0, {2, 2, 0, 0}}, {10, 16, 1, 1, {0, 0, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_BOX_5X5_08U[] = {{12, 16, 4, 0, {2, 2, 0, 0}}, {12, 16, 1, 1, {0, 0, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_BOX_5X5_08U[] = {{14, 16, 4, 0, {2, 2, 0, 0}}, {14, 16, 1, 1, {0, 0, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_BOX_5X5_08U[] = {{16, 16, 4, 0, {2, 2, 0, 0}}, {16, 16, 1, 1, {0, 0, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_BOX_5X5_08U[] = {{18, 16, 4, 0, {2, 2, 0, 0}}, {18, 16, 1, 1, {0, 0, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_BOX_5X5_08U[] = {{20, 16, 4, 0, {2, 2, 0, 0}}, {20, 16, 1, 1, {0, 0, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_BOX_5X5_08U[] = {{22, 16, 4, 0, {2, 2, 0, 0}}, {22, 16, 1, 1, {0, 0, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_BOX_5X5_08U[] = {{24, 16, 4, 0, {2, 2, 0, 0}}, {24, 16, 1, 1, {0, 0, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_BOX_5X5_08U[] = {{26, 16, 4, 0, {2, 2, 0, 0}}, {26, 16, 1, 1, {0, 0, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_BOX_5X5_08U[] = {{28, 16, 4, 0, {2, 2, 0, 0}}, {28, 16, 1, 1, {0, 0, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_BOX_5X5_08U[] = {{30, 16, 4, 0, {2, 2, 0, 0}}, {30, 16, 1, 1, {0, 0, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_BOX_5X5_08U[] = {{32, 16, 4, 0, {2, 2, 0, 0}}, {32, 16, 1, 1, {0, 0, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_BOX_5X5_08U[] = {{2, 18, 4, 0, {2, 2, 0, 0}}, {2, 18, 1, 1, {0, 0, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_BOX_5X5_08U[] = {{4, 18, 4, 0, {2, 2, 0, 0}}, {4, 18, 1, 1, {0, 0, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_BOX_5X5_08U[] = {{6, 18, 4, 0, {2, 2, 0, 0}}, {6, 18, 1, 1, {0, 0, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_BOX_5X5_08U[] = {{8, 18, 4, 0, {2, 2, 0, 0}}, {8, 18, 1, 1, {0, 0, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_BOX_5X5_08U[] = {{10, 18, 4, 0, {2, 2, 0, 0}}, {10, 18, 1, 1, {0, 0, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_BOX_5X5_08U[] = {{12, 18, 4, 0, {2, 2, 0, 0}}, {12, 18, 1, 1, {0, 0, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_BOX_5X5_08U[] = {{14, 18, 4, 0, {2, 2, 0, 0}}, {14, 18, 1, 1, {0, 0, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_BOX_5X5_08U[] = {{16, 18, 4, 0, {2, 2, 0, 0}}, {16, 18, 1, 1, {0, 0, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_BOX_5X5_08U[] = {{18, 18, 4, 0, {2, 2, 0, 0}}, {18, 18, 1, 1, {0, 0, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_BOX_5X5_08U[] = {{20, 18, 4, 0, {2, 2, 0, 0}}, {20, 18, 1, 1, {0, 0, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_BOX_5X5_08U[] = {{22, 18, 4, 0, {2, 2, 0, 0}}, {22, 18, 1, 1, {0, 0, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_BOX_5X5_08U[] = {{24, 18, 4, 0, {2, 2, 0, 0}}, {24, 18, 1, 1, {0, 0, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_BOX_5X5_08U[] = {{26, 18, 4, 0, {2, 2, 0, 0}}, {26, 18, 1, 1, {0, 0, 2, 2}}, {26, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_BOX_5X5_08U[] = {{28, 18, 4, 0, {2, 2, 0, 0}}, {28, 18, 1, 1, {0, 0, 2, 2}}, {28, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_BOX_5X5_08U[] = {{2, 20, 4, 0, {2, 2, 0, 0}}, {2, 20, 1, 1, {0, 0, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_BOX_5X5_08U[] = {{4, 20, 4, 0, {2, 2, 0, 0}}, {4, 20, 1, 1, {0, 0, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_BOX_5X5_08U[] = {{6, 20, 4, 0, {2, 2, 0, 0}}, {6, 20, 1, 1, {0, 0, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_BOX_5X5_08U[] = {{8, 20, 4, 0, {2, 2, 0, 0}}, {8, 20, 1, 1, {0, 0, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_BOX_5X5_08U[] = {{10, 20, 4, 0, {2, 2, 0, 0}}, {10, 20, 1, 1, {0, 0, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_BOX_5X5_08U[] = {{12, 20, 4, 0, {2, 2, 0, 0}}, {12, 20, 1, 1, {0, 0, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_BOX_5X5_08U[] = {{14, 20, 4, 0, {2, 2, 0, 0}}, {14, 20, 1, 1, {0, 0, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_BOX_5X5_08U[] = {{16, 20, 4, 0, {2, 2, 0, 0}}, {16, 20, 1, 1, {0, 0, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_BOX_5X5_08U[] = {{18, 20, 4, 0, {2, 2, 0, 0}}, {18, 20, 1, 1, {0, 0, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_BOX_5X5_08U[] = {{20, 20, 4, 0, {2, 2, 0, 0}}, {20, 20, 1, 1, {0, 0, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_BOX_5X5_08U[] = {{22, 20, 4, 0, {2, 2, 0, 0}}, {22, 20, 1, 1, {0, 0, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_BOX_5X5_08U[] = {{24, 20, 4, 0, {2, 2, 0, 0}}, {24, 20, 1, 1, {0, 0, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_BOX_5X5_08U[] = {{26, 20, 4, 0, {2, 2, 0, 0}}, {26, 20, 1, 1, {0, 0, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_BOX_5X5_08U[] = {{2, 22, 4, 0, {2, 2, 0, 0}}, {2, 22, 1, 1, {0, 0, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_BOX_5X5_08U[] = {{4, 22, 4, 0, {2, 2, 0, 0}}, {4, 22, 1, 1, {0, 0, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_BOX_5X5_08U[] = {{6, 22, 4, 0, {2, 2, 0, 0}}, {6, 22, 1, 1, {0, 0, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_BOX_5X5_08U[] = {{8, 22, 4, 0, {2, 2, 0, 0}}, {8, 22, 1, 1, {0, 0, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_BOX_5X5_08U[] = {{10, 22, 4, 0, {2, 2, 0, 0}}, {10, 22, 1, 1, {0, 0, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_BOX_5X5_08U[] = {{12, 22, 4, 0, {2, 2, 0, 0}}, {12, 22, 1, 1, {0, 0, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_BOX_5X5_08U[] = {{14, 22, 4, 0, {2, 2, 0, 0}}, {14, 22, 1, 1, {0, 0, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_BOX_5X5_08U[] = {{16, 22, 4, 0, {2, 2, 0, 0}}, {16, 22, 1, 1, {0, 0, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_BOX_5X5_08U[] = {{18, 22, 4, 0, {2, 2, 0, 0}}, {18, 22, 1, 1, {0, 0, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_BOX_5X5_08U[] = {{20, 22, 4, 0, {2, 2, 0, 0}}, {20, 22, 1, 1, {0, 0, 2, 2}}, {20, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_BOX_5X5_08U[] = {{22, 22, 4, 0, {2, 2, 0, 0}}, {22, 22, 1, 1, {0, 0, 2, 2}}, {22, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_BOX_5X5_08U[] = {{2, 24, 4, 0, {2, 2, 0, 0}}, {2, 24, 1, 1, {0, 0, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_BOX_5X5_08U[] = {{4, 24, 4, 0, {2, 2, 0, 0}}, {4, 24, 1, 1, {0, 0, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_BOX_5X5_08U[] = {{6, 24, 4, 0, {2, 2, 0, 0}}, {6, 24, 1, 1, {0, 0, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_BOX_5X5_08U[] = {{8, 24, 4, 0, {2, 2, 0, 0}}, {8, 24, 1, 1, {0, 0, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_BOX_5X5_08U[] = {{10, 24, 4, 0, {2, 2, 0, 0}}, {10, 24, 1, 1, {0, 0, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_BOX_5X5_08U[] = {{12, 24, 4, 0, {2, 2, 0, 0}}, {12, 24, 1, 1, {0, 0, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_BOX_5X5_08U[] = {{14, 24, 4, 0, {2, 2, 0, 0}}, {14, 24, 1, 1, {0, 0, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_BOX_5X5_08U[] = {{16, 24, 4, 0, {2, 2, 0, 0}}, {16, 24, 1, 1, {0, 0, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_BOX_5X5_08U[] = {{18, 24, 4, 0, {2, 2, 0, 0}}, {18, 24, 1, 1, {0, 0, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_BOX_5X5_08U[] = {{20, 24, 4, 0, {2, 2, 0, 0}}, {20, 24, 1, 1, {0, 0, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_BOX_5X5_08U[] = {{2, 26, 4, 0, {2, 2, 0, 0}}, {2, 26, 1, 1, {0, 0, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_BOX_5X5_08U[] = {{4, 26, 4, 0, {2, 2, 0, 0}}, {4, 26, 1, 1, {0, 0, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_BOX_5X5_08U[] = {{6, 26, 4, 0, {2, 2, 0, 0}}, {6, 26, 1, 1, {0, 0, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_BOX_5X5_08U[] = {{8, 26, 4, 0, {2, 2, 0, 0}}, {8, 26, 1, 1, {0, 0, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_BOX_5X5_08U[] = {{10, 26, 4, 0, {2, 2, 0, 0}}, {10, 26, 1, 1, {0, 0, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_BOX_5X5_08U[] = {{12, 26, 4, 0, {2, 2, 0, 0}}, {12, 26, 1, 1, {0, 0, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_BOX_5X5_08U[] = {{14, 26, 4, 0, {2, 2, 0, 0}}, {14, 26, 1, 1, {0, 0, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_BOX_5X5_08U[] = {{16, 26, 4, 0, {2, 2, 0, 0}}, {16, 26, 1, 1, {0, 0, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_BOX_5X5_08U[] = {{18, 26, 4, 0, {2, 2, 0, 0}}, {18, 26, 1, 1, {0, 0, 2, 2}}, {18, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_BOX_5X5_08U[] = {{20, 26, 4, 0, {2, 2, 0, 0}}, {20, 26, 1, 1, {0, 0, 2, 2}}, {20, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_BOX_5X5_08U[] = {{2, 28, 4, 0, {2, 2, 0, 0}}, {2, 28, 1, 1, {0, 0, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_BOX_5X5_08U[] = {{4, 28, 4, 0, {2, 2, 0, 0}}, {4, 28, 1, 1, {0, 0, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_BOX_5X5_08U[] = {{6, 28, 4, 0, {2, 2, 0, 0}}, {6, 28, 1, 1, {0, 0, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_BOX_5X5_08U[] = {{8, 28, 4, 0, {2, 2, 0, 0}}, {8, 28, 1, 1, {0, 0, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_BOX_5X5_08U[] = {{10, 28, 4, 0, {2, 2, 0, 0}}, {10, 28, 1, 1, {0, 0, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_BOX_5X5_08U[] = {{12, 28, 4, 0, {2, 2, 0, 0}}, {12, 28, 1, 1, {0, 0, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_BOX_5X5_08U[] = {{14, 28, 4, 0, {2, 2, 0, 0}}, {14, 28, 1, 1, {0, 0, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_BOX_5X5_08U[] = {{16, 28, 4, 0, {2, 2, 0, 0}}, {16, 28, 1, 1, {0, 0, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_BOX_5X5_08U[] = {{18, 28, 4, 0, {2, 2, 0, 0}}, {18, 28, 1, 1, {0, 0, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_BOX_5X5_08U[] = {{2, 30, 4, 0, {2, 2, 0, 0}}, {2, 30, 1, 1, {0, 0, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_BOX_5X5_08U[] = {{4, 30, 4, 0, {2, 2, 0, 0}}, {4, 30, 1, 1, {0, 0, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_BOX_5X5_08U[] = {{6, 30, 4, 0, {2, 2, 0, 0}}, {6, 30, 1, 1, {0, 0, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_BOX_5X5_08U[] = {{8, 30, 4, 0, {2, 2, 0, 0}}, {8, 30, 1, 1, {0, 0, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_BOX_5X5_08U[] = {{10, 30, 4, 0, {2, 2, 0, 0}}, {10, 30, 1, 1, {0, 0, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_BOX_5X5_08U[] = {{12, 30, 4, 0, {2, 2, 0, 0}}, {12, 30, 1, 1, {0, 0, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_BOX_5X5_08U[] = {{14, 30, 4, 0, {2, 2, 0, 0}}, {14, 30, 1, 1, {0, 0, 2, 2}}, {14, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_BOX_5X5_08U[] = {{16, 30, 4, 0, {2, 2, 0, 0}}, {16, 30, 1, 1, {0, 0, 2, 2}}, {16, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_BOX_5X5_08U[] = {{2, 32, 4, 0, {2, 2, 0, 0}}, {2, 32, 1, 1, {0, 0, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_BOX_5X5_08U[] = {{4, 32, 4, 0, {2, 2, 0, 0}}, {4, 32, 1, 1, {0, 0, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_BOX_5X5_08U[] = {{6, 32, 4, 0, {2, 2, 0, 0}}, {6, 32, 1, 1, {0, 0, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_BOX_5X5_08U[] = {{8, 32, 4, 0, {2, 2, 0, 0}}, {8, 32, 1, 1, {0, 0, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_BOX_5X5_08U[] = {{10, 32, 4, 0, {2, 2, 0, 0}}, {10, 32, 1, 1, {0, 0, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_BOX_5X5_08U[] = {{12, 32, 4, 0, {2, 2, 0, 0}}, {12, 32, 1, 1, {0, 0, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_BOX_5X5_08U[] = {{14, 32, 4, 0, {2, 2, 0, 0}}, {14, 32, 1, 1, {0, 0, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_BOX_5X5_08U[] = {{16, 32, 4, 0, {2, 2, 0, 0}}, {16, 32, 1, 1, {0, 0, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_BOX_5X5_08U[] = {{2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_BOX_5X5_08U[] = {{1, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_BOX_5X5_08U[] = {{1, 0}, {1, 0}};

static acf_scenario gScenarioArray_BOX_5X5_08U[] = {
{2, 1, 36, 0, 2, gScenarioBufferData0_BOX_5X5_08U, 48, gScenarioKernelData0_BOX_5X5_08U, 8},
{4, 1, 60, 0, 2, gScenarioBufferData1_BOX_5X5_08U, 48, gScenarioKernelData1_BOX_5X5_08U, 8},
{6, 1, 92, 0, 2, gScenarioBufferData2_BOX_5X5_08U, 48, gScenarioKernelData2_BOX_5X5_08U, 8},
{8, 1, 116, 0, 2, gScenarioBufferData3_BOX_5X5_08U, 48, gScenarioKernelData3_BOX_5X5_08U, 8},
{10, 1, 148, 0, 2, gScenarioBufferData4_BOX_5X5_08U, 48, gScenarioKernelData4_BOX_5X5_08U, 8},
{12, 1, 172, 0, 2, gScenarioBufferData5_BOX_5X5_08U, 48, gScenarioKernelData5_BOX_5X5_08U, 8},
{14, 1, 204, 0, 2, gScenarioBufferData6_BOX_5X5_08U, 48, gScenarioKernelData6_BOX_5X5_08U, 8},
{16, 1, 228, 0, 2, gScenarioBufferData7_BOX_5X5_08U, 48, gScenarioKernelData7_BOX_5X5_08U, 8},
{18, 1, 260, 0, 2, gScenarioBufferData8_BOX_5X5_08U, 48, gScenarioKernelData8_BOX_5X5_08U, 8},
{20, 1, 284, 0, 2, gScenarioBufferData9_BOX_5X5_08U, 48, gScenarioKernelData9_BOX_5X5_08U, 8},
{22, 1, 316, 0, 2, gScenarioBufferData10_BOX_5X5_08U, 48, gScenarioKernelData10_BOX_5X5_08U, 8},
{24, 1, 340, 0, 2, gScenarioBufferData11_BOX_5X5_08U, 48, gScenarioKernelData11_BOX_5X5_08U, 8},
{26, 1, 372, 0, 2, gScenarioBufferData12_BOX_5X5_08U, 48, gScenarioKernelData12_BOX_5X5_08U, 8},
{28, 1, 396, 0, 2, gScenarioBufferData13_BOX_5X5_08U, 48, gScenarioKernelData13_BOX_5X5_08U, 8},
{30, 1, 428, 0, 2, gScenarioBufferData14_BOX_5X5_08U, 48, gScenarioKernelData14_BOX_5X5_08U, 8},
{32, 1, 452, 0, 2, gScenarioBufferData15_BOX_5X5_08U, 48, gScenarioKernelData15_BOX_5X5_08U, 8},
{36, 1, 508, 0, 2, gScenarioBufferData16_BOX_5X5_08U, 48, gScenarioKernelData16_BOX_5X5_08U, 8},
{40, 1, 564, 0, 2, gScenarioBufferData17_BOX_5X5_08U, 48, gScenarioKernelData17_BOX_5X5_08U, 8},
{44, 1, 620, 0, 2, gScenarioBufferData18_BOX_5X5_08U, 48, gScenarioKernelData18_BOX_5X5_08U, 8},
{48, 1, 676, 0, 2, gScenarioBufferData19_BOX_5X5_08U, 48, gScenarioKernelData19_BOX_5X5_08U, 8},
{52, 1, 732, 0, 2, gScenarioBufferData20_BOX_5X5_08U, 48, gScenarioKernelData20_BOX_5X5_08U, 8},
{56, 1, 788, 0, 2, gScenarioBufferData21_BOX_5X5_08U, 48, gScenarioKernelData21_BOX_5X5_08U, 8},
{60, 1, 844, 0, 2, gScenarioBufferData22_BOX_5X5_08U, 48, gScenarioKernelData22_BOX_5X5_08U, 8},
{64, 1, 900, 0, 2, gScenarioBufferData23_BOX_5X5_08U, 48, gScenarioKernelData23_BOX_5X5_08U, 8},
{72, 1, 1012, 0, 2, gScenarioBufferData24_BOX_5X5_08U, 48, gScenarioKernelData24_BOX_5X5_08U, 8},
{80, 1, 1124, 0, 2, gScenarioBufferData25_BOX_5X5_08U, 48, gScenarioKernelData25_BOX_5X5_08U, 8},
{88, 1, 1236, 0, 2, gScenarioBufferData26_BOX_5X5_08U, 48, gScenarioKernelData26_BOX_5X5_08U, 8},
{96, 1, 1348, 0, 2, gScenarioBufferData27_BOX_5X5_08U, 48, gScenarioKernelData27_BOX_5X5_08U, 8},
{104, 1, 1460, 0, 2, gScenarioBufferData28_BOX_5X5_08U, 48, gScenarioKernelData28_BOX_5X5_08U, 8},
{112, 1, 1572, 0, 2, gScenarioBufferData29_BOX_5X5_08U, 48, gScenarioKernelData29_BOX_5X5_08U, 8},
{120, 1, 1684, 0, 2, gScenarioBufferData30_BOX_5X5_08U, 48, gScenarioKernelData30_BOX_5X5_08U, 8},
{128, 1, 1796, 0, 2, gScenarioBufferData31_BOX_5X5_08U, 48, gScenarioKernelData31_BOX_5X5_08U, 8},
{2, 2, 44, 0, 1, gScenarioBufferData32_BOX_5X5_08U, 48, gScenarioKernelData32_BOX_5X5_08U, 8},
{4, 2, 80, 0, 1, gScenarioBufferData33_BOX_5X5_08U, 48, gScenarioKernelData33_BOX_5X5_08U, 8},
{6, 2, 116, 0, 1, gScenarioBufferData34_BOX_5X5_08U, 48, gScenarioKernelData34_BOX_5X5_08U, 8},
{8, 2, 152, 0, 1, gScenarioBufferData35_BOX_5X5_08U, 48, gScenarioKernelData35_BOX_5X5_08U, 8},
{10, 2, 188, 0, 1, gScenarioBufferData36_BOX_5X5_08U, 48, gScenarioKernelData36_BOX_5X5_08U, 8},
{12, 2, 224, 0, 1, gScenarioBufferData37_BOX_5X5_08U, 48, gScenarioKernelData37_BOX_5X5_08U, 8},
{14, 2, 260, 0, 1, gScenarioBufferData38_BOX_5X5_08U, 48, gScenarioKernelData38_BOX_5X5_08U, 8},
{16, 2, 296, 0, 1, gScenarioBufferData39_BOX_5X5_08U, 48, gScenarioKernelData39_BOX_5X5_08U, 8},
{18, 2, 332, 0, 1, gScenarioBufferData40_BOX_5X5_08U, 48, gScenarioKernelData40_BOX_5X5_08U, 8},
{20, 2, 368, 0, 1, gScenarioBufferData41_BOX_5X5_08U, 48, gScenarioKernelData41_BOX_5X5_08U, 8},
{22, 2, 404, 0, 1, gScenarioBufferData42_BOX_5X5_08U, 48, gScenarioKernelData42_BOX_5X5_08U, 8},
{24, 2, 440, 0, 1, gScenarioBufferData43_BOX_5X5_08U, 48, gScenarioKernelData43_BOX_5X5_08U, 8},
{26, 2, 476, 0, 1, gScenarioBufferData44_BOX_5X5_08U, 48, gScenarioKernelData44_BOX_5X5_08U, 8},
{28, 2, 512, 0, 1, gScenarioBufferData45_BOX_5X5_08U, 48, gScenarioKernelData45_BOX_5X5_08U, 8},
{30, 2, 548, 0, 1, gScenarioBufferData46_BOX_5X5_08U, 48, gScenarioKernelData46_BOX_5X5_08U, 8},
{32, 2, 584, 0, 1, gScenarioBufferData47_BOX_5X5_08U, 48, gScenarioKernelData47_BOX_5X5_08U, 8},
{36, 2, 656, 0, 1, gScenarioBufferData48_BOX_5X5_08U, 48, gScenarioKernelData48_BOX_5X5_08U, 8},
{40, 2, 728, 0, 1, gScenarioBufferData49_BOX_5X5_08U, 48, gScenarioKernelData49_BOX_5X5_08U, 8},
{44, 2, 800, 0, 1, gScenarioBufferData50_BOX_5X5_08U, 48, gScenarioKernelData50_BOX_5X5_08U, 8},
{48, 2, 872, 0, 1, gScenarioBufferData51_BOX_5X5_08U, 48, gScenarioKernelData51_BOX_5X5_08U, 8},
{52, 2, 944, 0, 1, gScenarioBufferData52_BOX_5X5_08U, 48, gScenarioKernelData52_BOX_5X5_08U, 8},
{56, 2, 1016, 0, 1, gScenarioBufferData53_BOX_5X5_08U, 48, gScenarioKernelData53_BOX_5X5_08U, 8},
{60, 2, 1088, 0, 1, gScenarioBufferData54_BOX_5X5_08U, 48, gScenarioKernelData54_BOX_5X5_08U, 8},
{64, 2, 1160, 0, 1, gScenarioBufferData55_BOX_5X5_08U, 48, gScenarioKernelData55_BOX_5X5_08U, 8},
{72, 2, 1304, 0, 1, gScenarioBufferData56_BOX_5X5_08U, 48, gScenarioKernelData56_BOX_5X5_08U, 8},
{80, 2, 1448, 0, 1, gScenarioBufferData57_BOX_5X5_08U, 48, gScenarioKernelData57_BOX_5X5_08U, 8},
{88, 2, 1592, 0, 1, gScenarioBufferData58_BOX_5X5_08U, 48, gScenarioKernelData58_BOX_5X5_08U, 8},
{96, 2, 1736, 0, 1, gScenarioBufferData59_BOX_5X5_08U, 48, gScenarioKernelData59_BOX_5X5_08U, 8},
{104, 2, 1880, 0, 1, gScenarioBufferData60_BOX_5X5_08U, 48, gScenarioKernelData60_BOX_5X5_08U, 8},
{112, 2, 2024, 0, 1, gScenarioBufferData61_BOX_5X5_08U, 48, gScenarioKernelData61_BOX_5X5_08U, 8},
{120, 2, 2168, 0, 1, gScenarioBufferData62_BOX_5X5_08U, 48, gScenarioKernelData62_BOX_5X5_08U, 8},
{128, 2, 2312, 0, 1, gScenarioBufferData63_BOX_5X5_08U, 48, gScenarioKernelData63_BOX_5X5_08U, 8},
{2, 3, 64, 0, 1, gScenarioBufferData64_BOX_5X5_08U, 48, gScenarioKernelData64_BOX_5X5_08U, 8},
{4, 3, 112, 0, 1, gScenarioBufferData65_BOX_5X5_08U, 48, gScenarioKernelData65_BOX_5X5_08U, 8},
{6, 3, 164, 0, 1, gScenarioBufferData66_BOX_5X5_08U, 48, gScenarioKernelData66_BOX_5X5_08U, 8},
{8, 3, 212, 0, 1, gScenarioBufferData67_BOX_5X5_08U, 48, gScenarioKernelData67_BOX_5X5_08U, 8},
{10, 3, 264, 0, 1, gScenarioBufferData68_BOX_5X5_08U, 48, gScenarioKernelData68_BOX_5X5_08U, 8},
{12, 3, 312, 0, 1, gScenarioBufferData69_BOX_5X5_08U, 48, gScenarioKernelData69_BOX_5X5_08U, 8},
{14, 3, 364, 0, 1, gScenarioBufferData70_BOX_5X5_08U, 48, gScenarioKernelData70_BOX_5X5_08U, 8},
{16, 3, 412, 0, 1, gScenarioBufferData71_BOX_5X5_08U, 48, gScenarioKernelData71_BOX_5X5_08U, 8},
{18, 3, 464, 0, 1, gScenarioBufferData72_BOX_5X5_08U, 48, gScenarioKernelData72_BOX_5X5_08U, 8},
{20, 3, 512, 0, 1, gScenarioBufferData73_BOX_5X5_08U, 48, gScenarioKernelData73_BOX_5X5_08U, 8},
{22, 3, 564, 0, 1, gScenarioBufferData74_BOX_5X5_08U, 48, gScenarioKernelData74_BOX_5X5_08U, 8},
{24, 3, 612, 0, 1, gScenarioBufferData75_BOX_5X5_08U, 48, gScenarioKernelData75_BOX_5X5_08U, 8},
{26, 3, 664, 0, 1, gScenarioBufferData76_BOX_5X5_08U, 48, gScenarioKernelData76_BOX_5X5_08U, 8},
{28, 3, 712, 0, 1, gScenarioBufferData77_BOX_5X5_08U, 48, gScenarioKernelData77_BOX_5X5_08U, 8},
{30, 3, 764, 0, 1, gScenarioBufferData78_BOX_5X5_08U, 48, gScenarioKernelData78_BOX_5X5_08U, 8},
{32, 3, 812, 0, 1, gScenarioBufferData79_BOX_5X5_08U, 48, gScenarioKernelData79_BOX_5X5_08U, 8},
{36, 3, 912, 0, 1, gScenarioBufferData80_BOX_5X5_08U, 48, gScenarioKernelData80_BOX_5X5_08U, 8},
{40, 3, 1012, 0, 1, gScenarioBufferData81_BOX_5X5_08U, 48, gScenarioKernelData81_BOX_5X5_08U, 8},
{44, 3, 1112, 0, 1, gScenarioBufferData82_BOX_5X5_08U, 48, gScenarioKernelData82_BOX_5X5_08U, 8},
{48, 3, 1212, 0, 1, gScenarioBufferData83_BOX_5X5_08U, 48, gScenarioKernelData83_BOX_5X5_08U, 8},
{52, 3, 1312, 0, 1, gScenarioBufferData84_BOX_5X5_08U, 48, gScenarioKernelData84_BOX_5X5_08U, 8},
{56, 3, 1412, 0, 1, gScenarioBufferData85_BOX_5X5_08U, 48, gScenarioKernelData85_BOX_5X5_08U, 8},
{60, 3, 1512, 0, 1, gScenarioBufferData86_BOX_5X5_08U, 48, gScenarioKernelData86_BOX_5X5_08U, 8},
{64, 3, 1612, 0, 1, gScenarioBufferData87_BOX_5X5_08U, 48, gScenarioKernelData87_BOX_5X5_08U, 8},
{72, 3, 1812, 0, 1, gScenarioBufferData88_BOX_5X5_08U, 48, gScenarioKernelData88_BOX_5X5_08U, 8},
{80, 3, 2012, 0, 1, gScenarioBufferData89_BOX_5X5_08U, 48, gScenarioKernelData89_BOX_5X5_08U, 8},
{88, 3, 2212, 0, 1, gScenarioBufferData90_BOX_5X5_08U, 48, gScenarioKernelData90_BOX_5X5_08U, 8},
{96, 3, 2412, 0, 1, gScenarioBufferData91_BOX_5X5_08U, 48, gScenarioKernelData91_BOX_5X5_08U, 8},
{104, 3, 2612, 0, 1, gScenarioBufferData92_BOX_5X5_08U, 48, gScenarioKernelData92_BOX_5X5_08U, 8},
{112, 3, 2812, 0, 1, gScenarioBufferData93_BOX_5X5_08U, 48, gScenarioKernelData93_BOX_5X5_08U, 8},
{120, 3, 3012, 0, 1, gScenarioBufferData94_BOX_5X5_08U, 48, gScenarioKernelData94_BOX_5X5_08U, 8},
{128, 3, 3212, 0, 1, gScenarioBufferData95_BOX_5X5_08U, 48, gScenarioKernelData95_BOX_5X5_08U, 8},
{2, 4, 80, 0, 1, gScenarioBufferData96_BOX_5X5_08U, 48, gScenarioKernelData96_BOX_5X5_08U, 8},
{4, 4, 144, 0, 1, gScenarioBufferData97_BOX_5X5_08U, 48, gScenarioKernelData97_BOX_5X5_08U, 8},
{6, 4, 208, 0, 1, gScenarioBufferData98_BOX_5X5_08U, 48, gScenarioKernelData98_BOX_5X5_08U, 8},
{8, 4, 272, 0, 1, gScenarioBufferData99_BOX_5X5_08U, 48, gScenarioKernelData99_BOX_5X5_08U, 8},
{10, 4, 336, 0, 1, gScenarioBufferData100_BOX_5X5_08U, 48, gScenarioKernelData100_BOX_5X5_08U, 8},
{12, 4, 400, 0, 1, gScenarioBufferData101_BOX_5X5_08U, 48, gScenarioKernelData101_BOX_5X5_08U, 8},
{14, 4, 464, 0, 1, gScenarioBufferData102_BOX_5X5_08U, 48, gScenarioKernelData102_BOX_5X5_08U, 8},
{16, 4, 528, 0, 1, gScenarioBufferData103_BOX_5X5_08U, 48, gScenarioKernelData103_BOX_5X5_08U, 8},
{18, 4, 592, 0, 1, gScenarioBufferData104_BOX_5X5_08U, 48, gScenarioKernelData104_BOX_5X5_08U, 8},
{20, 4, 656, 0, 1, gScenarioBufferData105_BOX_5X5_08U, 48, gScenarioKernelData105_BOX_5X5_08U, 8},
{22, 4, 720, 0, 1, gScenarioBufferData106_BOX_5X5_08U, 48, gScenarioKernelData106_BOX_5X5_08U, 8},
{24, 4, 784, 0, 1, gScenarioBufferData107_BOX_5X5_08U, 48, gScenarioKernelData107_BOX_5X5_08U, 8},
{26, 4, 848, 0, 1, gScenarioBufferData108_BOX_5X5_08U, 48, gScenarioKernelData108_BOX_5X5_08U, 8},
{28, 4, 912, 0, 1, gScenarioBufferData109_BOX_5X5_08U, 48, gScenarioKernelData109_BOX_5X5_08U, 8},
{30, 4, 976, 0, 1, gScenarioBufferData110_BOX_5X5_08U, 48, gScenarioKernelData110_BOX_5X5_08U, 8},
{32, 4, 1040, 0, 1, gScenarioBufferData111_BOX_5X5_08U, 48, gScenarioKernelData111_BOX_5X5_08U, 8},
{36, 4, 1168, 0, 1, gScenarioBufferData112_BOX_5X5_08U, 48, gScenarioKernelData112_BOX_5X5_08U, 8},
{40, 4, 1296, 0, 1, gScenarioBufferData113_BOX_5X5_08U, 48, gScenarioKernelData113_BOX_5X5_08U, 8},
{44, 4, 1424, 0, 1, gScenarioBufferData114_BOX_5X5_08U, 48, gScenarioKernelData114_BOX_5X5_08U, 8},
{48, 4, 1552, 0, 1, gScenarioBufferData115_BOX_5X5_08U, 48, gScenarioKernelData115_BOX_5X5_08U, 8},
{52, 4, 1680, 0, 1, gScenarioBufferData116_BOX_5X5_08U, 48, gScenarioKernelData116_BOX_5X5_08U, 8},
{56, 4, 1808, 0, 1, gScenarioBufferData117_BOX_5X5_08U, 48, gScenarioKernelData117_BOX_5X5_08U, 8},
{60, 4, 1936, 0, 1, gScenarioBufferData118_BOX_5X5_08U, 48, gScenarioKernelData118_BOX_5X5_08U, 8},
{64, 4, 2064, 0, 1, gScenarioBufferData119_BOX_5X5_08U, 48, gScenarioKernelData119_BOX_5X5_08U, 8},
{72, 4, 2320, 0, 1, gScenarioBufferData120_BOX_5X5_08U, 48, gScenarioKernelData120_BOX_5X5_08U, 8},
{80, 4, 2576, 0, 1, gScenarioBufferData121_BOX_5X5_08U, 48, gScenarioKernelData121_BOX_5X5_08U, 8},
{88, 4, 2832, 0, 1, gScenarioBufferData122_BOX_5X5_08U, 48, gScenarioKernelData122_BOX_5X5_08U, 8},
{96, 4, 3088, 0, 1, gScenarioBufferData123_BOX_5X5_08U, 48, gScenarioKernelData123_BOX_5X5_08U, 8},
{104, 4, 3344, 0, 1, gScenarioBufferData124_BOX_5X5_08U, 48, gScenarioKernelData124_BOX_5X5_08U, 8},
{112, 4, 3600, 0, 1, gScenarioBufferData125_BOX_5X5_08U, 48, gScenarioKernelData125_BOX_5X5_08U, 8},
{2, 5, 100, 0, 1, gScenarioBufferData126_BOX_5X5_08U, 48, gScenarioKernelData126_BOX_5X5_08U, 8},
{4, 5, 176, 0, 1, gScenarioBufferData127_BOX_5X5_08U, 48, gScenarioKernelData127_BOX_5X5_08U, 8},
{6, 5, 256, 0, 1, gScenarioBufferData128_BOX_5X5_08U, 48, gScenarioKernelData128_BOX_5X5_08U, 8},
{8, 5, 332, 0, 1, gScenarioBufferData129_BOX_5X5_08U, 48, gScenarioKernelData129_BOX_5X5_08U, 8},
{10, 5, 412, 0, 1, gScenarioBufferData130_BOX_5X5_08U, 48, gScenarioKernelData130_BOX_5X5_08U, 8},
{12, 5, 488, 0, 1, gScenarioBufferData131_BOX_5X5_08U, 48, gScenarioKernelData131_BOX_5X5_08U, 8},
{14, 5, 568, 0, 1, gScenarioBufferData132_BOX_5X5_08U, 48, gScenarioKernelData132_BOX_5X5_08U, 8},
{16, 5, 644, 0, 1, gScenarioBufferData133_BOX_5X5_08U, 48, gScenarioKernelData133_BOX_5X5_08U, 8},
{18, 5, 724, 0, 1, gScenarioBufferData134_BOX_5X5_08U, 48, gScenarioKernelData134_BOX_5X5_08U, 8},
{20, 5, 800, 0, 1, gScenarioBufferData135_BOX_5X5_08U, 48, gScenarioKernelData135_BOX_5X5_08U, 8},
{22, 5, 880, 0, 1, gScenarioBufferData136_BOX_5X5_08U, 48, gScenarioKernelData136_BOX_5X5_08U, 8},
{24, 5, 956, 0, 1, gScenarioBufferData137_BOX_5X5_08U, 48, gScenarioKernelData137_BOX_5X5_08U, 8},
{26, 5, 1036, 0, 1, gScenarioBufferData138_BOX_5X5_08U, 48, gScenarioKernelData138_BOX_5X5_08U, 8},
{28, 5, 1112, 0, 1, gScenarioBufferData139_BOX_5X5_08U, 48, gScenarioKernelData139_BOX_5X5_08U, 8},
{30, 5, 1192, 0, 1, gScenarioBufferData140_BOX_5X5_08U, 48, gScenarioKernelData140_BOX_5X5_08U, 8},
{32, 5, 1268, 0, 1, gScenarioBufferData141_BOX_5X5_08U, 48, gScenarioKernelData141_BOX_5X5_08U, 8},
{36, 5, 1424, 0, 1, gScenarioBufferData142_BOX_5X5_08U, 48, gScenarioKernelData142_BOX_5X5_08U, 8},
{40, 5, 1580, 0, 1, gScenarioBufferData143_BOX_5X5_08U, 48, gScenarioKernelData143_BOX_5X5_08U, 8},
{44, 5, 1736, 0, 1, gScenarioBufferData144_BOX_5X5_08U, 48, gScenarioKernelData144_BOX_5X5_08U, 8},
{48, 5, 1892, 0, 1, gScenarioBufferData145_BOX_5X5_08U, 48, gScenarioKernelData145_BOX_5X5_08U, 8},
{52, 5, 2048, 0, 1, gScenarioBufferData146_BOX_5X5_08U, 48, gScenarioKernelData146_BOX_5X5_08U, 8},
{56, 5, 2204, 0, 1, gScenarioBufferData147_BOX_5X5_08U, 48, gScenarioKernelData147_BOX_5X5_08U, 8},
{60, 5, 2360, 0, 1, gScenarioBufferData148_BOX_5X5_08U, 48, gScenarioKernelData148_BOX_5X5_08U, 8},
{64, 5, 2516, 0, 1, gScenarioBufferData149_BOX_5X5_08U, 48, gScenarioKernelData149_BOX_5X5_08U, 8},
{72, 5, 2828, 0, 1, gScenarioBufferData150_BOX_5X5_08U, 48, gScenarioKernelData150_BOX_5X5_08U, 8},
{80, 5, 3140, 0, 1, gScenarioBufferData151_BOX_5X5_08U, 48, gScenarioKernelData151_BOX_5X5_08U, 8},
{88, 5, 3452, 0, 1, gScenarioBufferData152_BOX_5X5_08U, 48, gScenarioKernelData152_BOX_5X5_08U, 8},
{96, 5, 3764, 0, 1, gScenarioBufferData153_BOX_5X5_08U, 48, gScenarioKernelData153_BOX_5X5_08U, 8},
{2, 6, 116, 0, 1, gScenarioBufferData154_BOX_5X5_08U, 48, gScenarioKernelData154_BOX_5X5_08U, 8},
{4, 6, 208, 0, 1, gScenarioBufferData155_BOX_5X5_08U, 48, gScenarioKernelData155_BOX_5X5_08U, 8},
{6, 6, 300, 0, 1, gScenarioBufferData156_BOX_5X5_08U, 48, gScenarioKernelData156_BOX_5X5_08U, 8},
{8, 6, 392, 0, 1, gScenarioBufferData157_BOX_5X5_08U, 48, gScenarioKernelData157_BOX_5X5_08U, 8},
{10, 6, 484, 0, 1, gScenarioBufferData158_BOX_5X5_08U, 48, gScenarioKernelData158_BOX_5X5_08U, 8},
{12, 6, 576, 0, 1, gScenarioBufferData159_BOX_5X5_08U, 48, gScenarioKernelData159_BOX_5X5_08U, 8},
{14, 6, 668, 0, 1, gScenarioBufferData160_BOX_5X5_08U, 48, gScenarioKernelData160_BOX_5X5_08U, 8},
{16, 6, 760, 0, 1, gScenarioBufferData161_BOX_5X5_08U, 48, gScenarioKernelData161_BOX_5X5_08U, 8},
{18, 6, 852, 0, 1, gScenarioBufferData162_BOX_5X5_08U, 48, gScenarioKernelData162_BOX_5X5_08U, 8},
{20, 6, 944, 0, 1, gScenarioBufferData163_BOX_5X5_08U, 48, gScenarioKernelData163_BOX_5X5_08U, 8},
{22, 6, 1036, 0, 1, gScenarioBufferData164_BOX_5X5_08U, 48, gScenarioKernelData164_BOX_5X5_08U, 8},
{24, 6, 1128, 0, 1, gScenarioBufferData165_BOX_5X5_08U, 48, gScenarioKernelData165_BOX_5X5_08U, 8},
{26, 6, 1220, 0, 1, gScenarioBufferData166_BOX_5X5_08U, 48, gScenarioKernelData166_BOX_5X5_08U, 8},
{28, 6, 1312, 0, 1, gScenarioBufferData167_BOX_5X5_08U, 48, gScenarioKernelData167_BOX_5X5_08U, 8},
{30, 6, 1404, 0, 1, gScenarioBufferData168_BOX_5X5_08U, 48, gScenarioKernelData168_BOX_5X5_08U, 8},
{32, 6, 1496, 0, 1, gScenarioBufferData169_BOX_5X5_08U, 48, gScenarioKernelData169_BOX_5X5_08U, 8},
{36, 6, 1680, 0, 1, gScenarioBufferData170_BOX_5X5_08U, 48, gScenarioKernelData170_BOX_5X5_08U, 8},
{40, 6, 1864, 0, 1, gScenarioBufferData171_BOX_5X5_08U, 48, gScenarioKernelData171_BOX_5X5_08U, 8},
{44, 6, 2048, 0, 1, gScenarioBufferData172_BOX_5X5_08U, 48, gScenarioKernelData172_BOX_5X5_08U, 8},
{48, 6, 2232, 0, 1, gScenarioBufferData173_BOX_5X5_08U, 48, gScenarioKernelData173_BOX_5X5_08U, 8},
{52, 6, 2416, 0, 1, gScenarioBufferData174_BOX_5X5_08U, 48, gScenarioKernelData174_BOX_5X5_08U, 8},
{56, 6, 2600, 0, 1, gScenarioBufferData175_BOX_5X5_08U, 48, gScenarioKernelData175_BOX_5X5_08U, 8},
{60, 6, 2784, 0, 1, gScenarioBufferData176_BOX_5X5_08U, 48, gScenarioKernelData176_BOX_5X5_08U, 8},
{64, 6, 2968, 0, 1, gScenarioBufferData177_BOX_5X5_08U, 48, gScenarioKernelData177_BOX_5X5_08U, 8},
{72, 6, 3336, 0, 1, gScenarioBufferData178_BOX_5X5_08U, 48, gScenarioKernelData178_BOX_5X5_08U, 8},
{80, 6, 3704, 0, 1, gScenarioBufferData179_BOX_5X5_08U, 48, gScenarioKernelData179_BOX_5X5_08U, 8},
{2, 8, 152, 0, 1, gScenarioBufferData180_BOX_5X5_08U, 48, gScenarioKernelData180_BOX_5X5_08U, 8},
{4, 8, 272, 0, 1, gScenarioBufferData181_BOX_5X5_08U, 48, gScenarioKernelData181_BOX_5X5_08U, 8},
{6, 8, 392, 0, 1, gScenarioBufferData182_BOX_5X5_08U, 48, gScenarioKernelData182_BOX_5X5_08U, 8},
{8, 8, 512, 0, 1, gScenarioBufferData183_BOX_5X5_08U, 48, gScenarioKernelData183_BOX_5X5_08U, 8},
{10, 8, 632, 0, 1, gScenarioBufferData184_BOX_5X5_08U, 48, gScenarioKernelData184_BOX_5X5_08U, 8},
{12, 8, 752, 0, 1, gScenarioBufferData185_BOX_5X5_08U, 48, gScenarioKernelData185_BOX_5X5_08U, 8},
{14, 8, 872, 0, 1, gScenarioBufferData186_BOX_5X5_08U, 48, gScenarioKernelData186_BOX_5X5_08U, 8},
{16, 8, 992, 0, 1, gScenarioBufferData187_BOX_5X5_08U, 48, gScenarioKernelData187_BOX_5X5_08U, 8},
{18, 8, 1112, 0, 1, gScenarioBufferData188_BOX_5X5_08U, 48, gScenarioKernelData188_BOX_5X5_08U, 8},
{20, 8, 1232, 0, 1, gScenarioBufferData189_BOX_5X5_08U, 48, gScenarioKernelData189_BOX_5X5_08U, 8},
{22, 8, 1352, 0, 1, gScenarioBufferData190_BOX_5X5_08U, 48, gScenarioKernelData190_BOX_5X5_08U, 8},
{24, 8, 1472, 0, 1, gScenarioBufferData191_BOX_5X5_08U, 48, gScenarioKernelData191_BOX_5X5_08U, 8},
{26, 8, 1592, 0, 1, gScenarioBufferData192_BOX_5X5_08U, 48, gScenarioKernelData192_BOX_5X5_08U, 8},
{28, 8, 1712, 0, 1, gScenarioBufferData193_BOX_5X5_08U, 48, gScenarioKernelData193_BOX_5X5_08U, 8},
{30, 8, 1832, 0, 1, gScenarioBufferData194_BOX_5X5_08U, 48, gScenarioKernelData194_BOX_5X5_08U, 8},
{32, 8, 1952, 0, 1, gScenarioBufferData195_BOX_5X5_08U, 48, gScenarioKernelData195_BOX_5X5_08U, 8},
{36, 8, 2192, 0, 1, gScenarioBufferData196_BOX_5X5_08U, 48, gScenarioKernelData196_BOX_5X5_08U, 8},
{40, 8, 2432, 0, 1, gScenarioBufferData197_BOX_5X5_08U, 48, gScenarioKernelData197_BOX_5X5_08U, 8},
{44, 8, 2672, 0, 1, gScenarioBufferData198_BOX_5X5_08U, 48, gScenarioKernelData198_BOX_5X5_08U, 8},
{48, 8, 2912, 0, 1, gScenarioBufferData199_BOX_5X5_08U, 48, gScenarioKernelData199_BOX_5X5_08U, 8},
{52, 8, 3152, 0, 1, gScenarioBufferData200_BOX_5X5_08U, 48, gScenarioKernelData200_BOX_5X5_08U, 8},
{56, 8, 3392, 0, 1, gScenarioBufferData201_BOX_5X5_08U, 48, gScenarioKernelData201_BOX_5X5_08U, 8},
{60, 8, 3632, 0, 1, gScenarioBufferData202_BOX_5X5_08U, 48, gScenarioKernelData202_BOX_5X5_08U, 8},
{2, 10, 188, 0, 1, gScenarioBufferData203_BOX_5X5_08U, 48, gScenarioKernelData203_BOX_5X5_08U, 8},
{4, 10, 336, 0, 1, gScenarioBufferData204_BOX_5X5_08U, 48, gScenarioKernelData204_BOX_5X5_08U, 8},
{6, 10, 484, 0, 1, gScenarioBufferData205_BOX_5X5_08U, 48, gScenarioKernelData205_BOX_5X5_08U, 8},
{8, 10, 632, 0, 1, gScenarioBufferData206_BOX_5X5_08U, 48, gScenarioKernelData206_BOX_5X5_08U, 8},
{10, 10, 780, 0, 1, gScenarioBufferData207_BOX_5X5_08U, 48, gScenarioKernelData207_BOX_5X5_08U, 8},
{12, 10, 928, 0, 1, gScenarioBufferData208_BOX_5X5_08U, 48, gScenarioKernelData208_BOX_5X5_08U, 8},
{14, 10, 1076, 0, 1, gScenarioBufferData209_BOX_5X5_08U, 48, gScenarioKernelData209_BOX_5X5_08U, 8},
{16, 10, 1224, 0, 1, gScenarioBufferData210_BOX_5X5_08U, 48, gScenarioKernelData210_BOX_5X5_08U, 8},
{18, 10, 1372, 0, 1, gScenarioBufferData211_BOX_5X5_08U, 48, gScenarioKernelData211_BOX_5X5_08U, 8},
{20, 10, 1520, 0, 1, gScenarioBufferData212_BOX_5X5_08U, 48, gScenarioKernelData212_BOX_5X5_08U, 8},
{22, 10, 1668, 0, 1, gScenarioBufferData213_BOX_5X5_08U, 48, gScenarioKernelData213_BOX_5X5_08U, 8},
{24, 10, 1816, 0, 1, gScenarioBufferData214_BOX_5X5_08U, 48, gScenarioKernelData214_BOX_5X5_08U, 8},
{26, 10, 1964, 0, 1, gScenarioBufferData215_BOX_5X5_08U, 48, gScenarioKernelData215_BOX_5X5_08U, 8},
{28, 10, 2112, 0, 1, gScenarioBufferData216_BOX_5X5_08U, 48, gScenarioKernelData216_BOX_5X5_08U, 8},
{30, 10, 2260, 0, 1, gScenarioBufferData217_BOX_5X5_08U, 48, gScenarioKernelData217_BOX_5X5_08U, 8},
{32, 10, 2408, 0, 1, gScenarioBufferData218_BOX_5X5_08U, 48, gScenarioKernelData218_BOX_5X5_08U, 8},
{36, 10, 2704, 0, 1, gScenarioBufferData219_BOX_5X5_08U, 48, gScenarioKernelData219_BOX_5X5_08U, 8},
{40, 10, 3000, 0, 1, gScenarioBufferData220_BOX_5X5_08U, 48, gScenarioKernelData220_BOX_5X5_08U, 8},
{44, 10, 3296, 0, 1, gScenarioBufferData221_BOX_5X5_08U, 48, gScenarioKernelData221_BOX_5X5_08U, 8},
{48, 10, 3592, 0, 1, gScenarioBufferData222_BOX_5X5_08U, 48, gScenarioKernelData222_BOX_5X5_08U, 8},
{2, 12, 224, 0, 1, gScenarioBufferData223_BOX_5X5_08U, 48, gScenarioKernelData223_BOX_5X5_08U, 8},
{4, 12, 400, 0, 1, gScenarioBufferData224_BOX_5X5_08U, 48, gScenarioKernelData224_BOX_5X5_08U, 8},
{6, 12, 576, 0, 1, gScenarioBufferData225_BOX_5X5_08U, 48, gScenarioKernelData225_BOX_5X5_08U, 8},
{8, 12, 752, 0, 1, gScenarioBufferData226_BOX_5X5_08U, 48, gScenarioKernelData226_BOX_5X5_08U, 8},
{10, 12, 928, 0, 1, gScenarioBufferData227_BOX_5X5_08U, 48, gScenarioKernelData227_BOX_5X5_08U, 8},
{12, 12, 1104, 0, 1, gScenarioBufferData228_BOX_5X5_08U, 48, gScenarioKernelData228_BOX_5X5_08U, 8},
{14, 12, 1280, 0, 1, gScenarioBufferData229_BOX_5X5_08U, 48, gScenarioKernelData229_BOX_5X5_08U, 8},
{16, 12, 1456, 0, 1, gScenarioBufferData230_BOX_5X5_08U, 48, gScenarioKernelData230_BOX_5X5_08U, 8},
{18, 12, 1632, 0, 1, gScenarioBufferData231_BOX_5X5_08U, 48, gScenarioKernelData231_BOX_5X5_08U, 8},
{20, 12, 1808, 0, 1, gScenarioBufferData232_BOX_5X5_08U, 48, gScenarioKernelData232_BOX_5X5_08U, 8},
{22, 12, 1984, 0, 1, gScenarioBufferData233_BOX_5X5_08U, 48, gScenarioKernelData233_BOX_5X5_08U, 8},
{24, 12, 2160, 0, 1, gScenarioBufferData234_BOX_5X5_08U, 48, gScenarioKernelData234_BOX_5X5_08U, 8},
{26, 12, 2336, 0, 1, gScenarioBufferData235_BOX_5X5_08U, 48, gScenarioKernelData235_BOX_5X5_08U, 8},
{28, 12, 2512, 0, 1, gScenarioBufferData236_BOX_5X5_08U, 48, gScenarioKernelData236_BOX_5X5_08U, 8},
{30, 12, 2688, 0, 1, gScenarioBufferData237_BOX_5X5_08U, 48, gScenarioKernelData237_BOX_5X5_08U, 8},
{32, 12, 2864, 0, 1, gScenarioBufferData238_BOX_5X5_08U, 48, gScenarioKernelData238_BOX_5X5_08U, 8},
{36, 12, 3216, 0, 1, gScenarioBufferData239_BOX_5X5_08U, 48, gScenarioKernelData239_BOX_5X5_08U, 8},
{40, 12, 3568, 0, 1, gScenarioBufferData240_BOX_5X5_08U, 48, gScenarioKernelData240_BOX_5X5_08U, 8},
{2, 14, 260, 0, 1, gScenarioBufferData241_BOX_5X5_08U, 48, gScenarioKernelData241_BOX_5X5_08U, 8},
{4, 14, 464, 0, 1, gScenarioBufferData242_BOX_5X5_08U, 48, gScenarioKernelData242_BOX_5X5_08U, 8},
{6, 14, 668, 0, 1, gScenarioBufferData243_BOX_5X5_08U, 48, gScenarioKernelData243_BOX_5X5_08U, 8},
{8, 14, 872, 0, 1, gScenarioBufferData244_BOX_5X5_08U, 48, gScenarioKernelData244_BOX_5X5_08U, 8},
{10, 14, 1076, 0, 1, gScenarioBufferData245_BOX_5X5_08U, 48, gScenarioKernelData245_BOX_5X5_08U, 8},
{12, 14, 1280, 0, 1, gScenarioBufferData246_BOX_5X5_08U, 48, gScenarioKernelData246_BOX_5X5_08U, 8},
{14, 14, 1484, 0, 1, gScenarioBufferData247_BOX_5X5_08U, 48, gScenarioKernelData247_BOX_5X5_08U, 8},
{16, 14, 1688, 0, 1, gScenarioBufferData248_BOX_5X5_08U, 48, gScenarioKernelData248_BOX_5X5_08U, 8},
{18, 14, 1892, 0, 1, gScenarioBufferData249_BOX_5X5_08U, 48, gScenarioKernelData249_BOX_5X5_08U, 8},
{20, 14, 2096, 0, 1, gScenarioBufferData250_BOX_5X5_08U, 48, gScenarioKernelData250_BOX_5X5_08U, 8},
{22, 14, 2300, 0, 1, gScenarioBufferData251_BOX_5X5_08U, 48, gScenarioKernelData251_BOX_5X5_08U, 8},
{24, 14, 2504, 0, 1, gScenarioBufferData252_BOX_5X5_08U, 48, gScenarioKernelData252_BOX_5X5_08U, 8},
{26, 14, 2708, 0, 1, gScenarioBufferData253_BOX_5X5_08U, 48, gScenarioKernelData253_BOX_5X5_08U, 8},
{28, 14, 2912, 0, 1, gScenarioBufferData254_BOX_5X5_08U, 48, gScenarioKernelData254_BOX_5X5_08U, 8},
{30, 14, 3116, 0, 1, gScenarioBufferData255_BOX_5X5_08U, 48, gScenarioKernelData255_BOX_5X5_08U, 8},
{32, 14, 3320, 0, 1, gScenarioBufferData256_BOX_5X5_08U, 48, gScenarioKernelData256_BOX_5X5_08U, 8},
{36, 14, 3728, 0, 1, gScenarioBufferData257_BOX_5X5_08U, 48, gScenarioKernelData257_BOX_5X5_08U, 8},
{2, 16, 296, 0, 1, gScenarioBufferData258_BOX_5X5_08U, 48, gScenarioKernelData258_BOX_5X5_08U, 8},
{4, 16, 528, 0, 1, gScenarioBufferData259_BOX_5X5_08U, 48, gScenarioKernelData259_BOX_5X5_08U, 8},
{6, 16, 760, 0, 1, gScenarioBufferData260_BOX_5X5_08U, 48, gScenarioKernelData260_BOX_5X5_08U, 8},
{8, 16, 992, 0, 1, gScenarioBufferData261_BOX_5X5_08U, 48, gScenarioKernelData261_BOX_5X5_08U, 8},
{10, 16, 1224, 0, 1, gScenarioBufferData262_BOX_5X5_08U, 48, gScenarioKernelData262_BOX_5X5_08U, 8},
{12, 16, 1456, 0, 1, gScenarioBufferData263_BOX_5X5_08U, 48, gScenarioKernelData263_BOX_5X5_08U, 8},
{14, 16, 1688, 0, 1, gScenarioBufferData264_BOX_5X5_08U, 48, gScenarioKernelData264_BOX_5X5_08U, 8},
{16, 16, 1920, 0, 1, gScenarioBufferData265_BOX_5X5_08U, 48, gScenarioKernelData265_BOX_5X5_08U, 8},
{18, 16, 2152, 0, 1, gScenarioBufferData266_BOX_5X5_08U, 48, gScenarioKernelData266_BOX_5X5_08U, 8},
{20, 16, 2384, 0, 1, gScenarioBufferData267_BOX_5X5_08U, 48, gScenarioKernelData267_BOX_5X5_08U, 8},
{22, 16, 2616, 0, 1, gScenarioBufferData268_BOX_5X5_08U, 48, gScenarioKernelData268_BOX_5X5_08U, 8},
{24, 16, 2848, 0, 1, gScenarioBufferData269_BOX_5X5_08U, 48, gScenarioKernelData269_BOX_5X5_08U, 8},
{26, 16, 3080, 0, 1, gScenarioBufferData270_BOX_5X5_08U, 48, gScenarioKernelData270_BOX_5X5_08U, 8},
{28, 16, 3312, 0, 1, gScenarioBufferData271_BOX_5X5_08U, 48, gScenarioKernelData271_BOX_5X5_08U, 8},
{30, 16, 3544, 0, 1, gScenarioBufferData272_BOX_5X5_08U, 48, gScenarioKernelData272_BOX_5X5_08U, 8},
{32, 16, 3776, 0, 1, gScenarioBufferData273_BOX_5X5_08U, 48, gScenarioKernelData273_BOX_5X5_08U, 8},
{2, 18, 332, 0, 1, gScenarioBufferData274_BOX_5X5_08U, 48, gScenarioKernelData274_BOX_5X5_08U, 8},
{4, 18, 592, 0, 1, gScenarioBufferData275_BOX_5X5_08U, 48, gScenarioKernelData275_BOX_5X5_08U, 8},
{6, 18, 852, 0, 1, gScenarioBufferData276_BOX_5X5_08U, 48, gScenarioKernelData276_BOX_5X5_08U, 8},
{8, 18, 1112, 0, 1, gScenarioBufferData277_BOX_5X5_08U, 48, gScenarioKernelData277_BOX_5X5_08U, 8},
{10, 18, 1372, 0, 1, gScenarioBufferData278_BOX_5X5_08U, 48, gScenarioKernelData278_BOX_5X5_08U, 8},
{12, 18, 1632, 0, 1, gScenarioBufferData279_BOX_5X5_08U, 48, gScenarioKernelData279_BOX_5X5_08U, 8},
{14, 18, 1892, 0, 1, gScenarioBufferData280_BOX_5X5_08U, 48, gScenarioKernelData280_BOX_5X5_08U, 8},
{16, 18, 2152, 0, 1, gScenarioBufferData281_BOX_5X5_08U, 48, gScenarioKernelData281_BOX_5X5_08U, 8},
{18, 18, 2412, 0, 1, gScenarioBufferData282_BOX_5X5_08U, 48, gScenarioKernelData282_BOX_5X5_08U, 8},
{20, 18, 2672, 0, 1, gScenarioBufferData283_BOX_5X5_08U, 48, gScenarioKernelData283_BOX_5X5_08U, 8},
{22, 18, 2932, 0, 1, gScenarioBufferData284_BOX_5X5_08U, 48, gScenarioKernelData284_BOX_5X5_08U, 8},
{24, 18, 3192, 0, 1, gScenarioBufferData285_BOX_5X5_08U, 48, gScenarioKernelData285_BOX_5X5_08U, 8},
{26, 18, 3452, 0, 1, gScenarioBufferData286_BOX_5X5_08U, 48, gScenarioKernelData286_BOX_5X5_08U, 8},
{28, 18, 3712, 0, 1, gScenarioBufferData287_BOX_5X5_08U, 48, gScenarioKernelData287_BOX_5X5_08U, 8},
{2, 20, 368, 0, 1, gScenarioBufferData288_BOX_5X5_08U, 48, gScenarioKernelData288_BOX_5X5_08U, 8},
{4, 20, 656, 0, 1, gScenarioBufferData289_BOX_5X5_08U, 48, gScenarioKernelData289_BOX_5X5_08U, 8},
{6, 20, 944, 0, 1, gScenarioBufferData290_BOX_5X5_08U, 48, gScenarioKernelData290_BOX_5X5_08U, 8},
{8, 20, 1232, 0, 1, gScenarioBufferData291_BOX_5X5_08U, 48, gScenarioKernelData291_BOX_5X5_08U, 8},
{10, 20, 1520, 0, 1, gScenarioBufferData292_BOX_5X5_08U, 48, gScenarioKernelData292_BOX_5X5_08U, 8},
{12, 20, 1808, 0, 1, gScenarioBufferData293_BOX_5X5_08U, 48, gScenarioKernelData293_BOX_5X5_08U, 8},
{14, 20, 2096, 0, 1, gScenarioBufferData294_BOX_5X5_08U, 48, gScenarioKernelData294_BOX_5X5_08U, 8},
{16, 20, 2384, 0, 1, gScenarioBufferData295_BOX_5X5_08U, 48, gScenarioKernelData295_BOX_5X5_08U, 8},
{18, 20, 2672, 0, 1, gScenarioBufferData296_BOX_5X5_08U, 48, gScenarioKernelData296_BOX_5X5_08U, 8},
{20, 20, 2960, 0, 1, gScenarioBufferData297_BOX_5X5_08U, 48, gScenarioKernelData297_BOX_5X5_08U, 8},
{22, 20, 3248, 0, 1, gScenarioBufferData298_BOX_5X5_08U, 48, gScenarioKernelData298_BOX_5X5_08U, 8},
{24, 20, 3536, 0, 1, gScenarioBufferData299_BOX_5X5_08U, 48, gScenarioKernelData299_BOX_5X5_08U, 8},
{26, 20, 3824, 0, 1, gScenarioBufferData300_BOX_5X5_08U, 48, gScenarioKernelData300_BOX_5X5_08U, 8},
{2, 22, 404, 0, 1, gScenarioBufferData301_BOX_5X5_08U, 48, gScenarioKernelData301_BOX_5X5_08U, 8},
{4, 22, 720, 0, 1, gScenarioBufferData302_BOX_5X5_08U, 48, gScenarioKernelData302_BOX_5X5_08U, 8},
{6, 22, 1036, 0, 1, gScenarioBufferData303_BOX_5X5_08U, 48, gScenarioKernelData303_BOX_5X5_08U, 8},
{8, 22, 1352, 0, 1, gScenarioBufferData304_BOX_5X5_08U, 48, gScenarioKernelData304_BOX_5X5_08U, 8},
{10, 22, 1668, 0, 1, gScenarioBufferData305_BOX_5X5_08U, 48, gScenarioKernelData305_BOX_5X5_08U, 8},
{12, 22, 1984, 0, 1, gScenarioBufferData306_BOX_5X5_08U, 48, gScenarioKernelData306_BOX_5X5_08U, 8},
{14, 22, 2300, 0, 1, gScenarioBufferData307_BOX_5X5_08U, 48, gScenarioKernelData307_BOX_5X5_08U, 8},
{16, 22, 2616, 0, 1, gScenarioBufferData308_BOX_5X5_08U, 48, gScenarioKernelData308_BOX_5X5_08U, 8},
{18, 22, 2932, 0, 1, gScenarioBufferData309_BOX_5X5_08U, 48, gScenarioKernelData309_BOX_5X5_08U, 8},
{20, 22, 3248, 0, 1, gScenarioBufferData310_BOX_5X5_08U, 48, gScenarioKernelData310_BOX_5X5_08U, 8},
{22, 22, 3564, 0, 1, gScenarioBufferData311_BOX_5X5_08U, 48, gScenarioKernelData311_BOX_5X5_08U, 8},
{2, 24, 440, 0, 1, gScenarioBufferData312_BOX_5X5_08U, 48, gScenarioKernelData312_BOX_5X5_08U, 8},
{4, 24, 784, 0, 1, gScenarioBufferData313_BOX_5X5_08U, 48, gScenarioKernelData313_BOX_5X5_08U, 8},
{6, 24, 1128, 0, 1, gScenarioBufferData314_BOX_5X5_08U, 48, gScenarioKernelData314_BOX_5X5_08U, 8},
{8, 24, 1472, 0, 1, gScenarioBufferData315_BOX_5X5_08U, 48, gScenarioKernelData315_BOX_5X5_08U, 8},
{10, 24, 1816, 0, 1, gScenarioBufferData316_BOX_5X5_08U, 48, gScenarioKernelData316_BOX_5X5_08U, 8},
{12, 24, 2160, 0, 1, gScenarioBufferData317_BOX_5X5_08U, 48, gScenarioKernelData317_BOX_5X5_08U, 8},
{14, 24, 2504, 0, 1, gScenarioBufferData318_BOX_5X5_08U, 48, gScenarioKernelData318_BOX_5X5_08U, 8},
{16, 24, 2848, 0, 1, gScenarioBufferData319_BOX_5X5_08U, 48, gScenarioKernelData319_BOX_5X5_08U, 8},
{18, 24, 3192, 0, 1, gScenarioBufferData320_BOX_5X5_08U, 48, gScenarioKernelData320_BOX_5X5_08U, 8},
{20, 24, 3536, 0, 1, gScenarioBufferData321_BOX_5X5_08U, 48, gScenarioKernelData321_BOX_5X5_08U, 8},
{2, 26, 476, 0, 1, gScenarioBufferData322_BOX_5X5_08U, 48, gScenarioKernelData322_BOX_5X5_08U, 8},
{4, 26, 848, 0, 1, gScenarioBufferData323_BOX_5X5_08U, 48, gScenarioKernelData323_BOX_5X5_08U, 8},
{6, 26, 1220, 0, 1, gScenarioBufferData324_BOX_5X5_08U, 48, gScenarioKernelData324_BOX_5X5_08U, 8},
{8, 26, 1592, 0, 1, gScenarioBufferData325_BOX_5X5_08U, 48, gScenarioKernelData325_BOX_5X5_08U, 8},
{10, 26, 1964, 0, 1, gScenarioBufferData326_BOX_5X5_08U, 48, gScenarioKernelData326_BOX_5X5_08U, 8},
{12, 26, 2336, 0, 1, gScenarioBufferData327_BOX_5X5_08U, 48, gScenarioKernelData327_BOX_5X5_08U, 8},
{14, 26, 2708, 0, 1, gScenarioBufferData328_BOX_5X5_08U, 48, gScenarioKernelData328_BOX_5X5_08U, 8},
{16, 26, 3080, 0, 1, gScenarioBufferData329_BOX_5X5_08U, 48, gScenarioKernelData329_BOX_5X5_08U, 8},
{18, 26, 3452, 0, 1, gScenarioBufferData330_BOX_5X5_08U, 48, gScenarioKernelData330_BOX_5X5_08U, 8},
{20, 26, 3824, 0, 1, gScenarioBufferData331_BOX_5X5_08U, 48, gScenarioKernelData331_BOX_5X5_08U, 8},
{2, 28, 512, 0, 1, gScenarioBufferData332_BOX_5X5_08U, 48, gScenarioKernelData332_BOX_5X5_08U, 8},
{4, 28, 912, 0, 1, gScenarioBufferData333_BOX_5X5_08U, 48, gScenarioKernelData333_BOX_5X5_08U, 8},
{6, 28, 1312, 0, 1, gScenarioBufferData334_BOX_5X5_08U, 48, gScenarioKernelData334_BOX_5X5_08U, 8},
{8, 28, 1712, 0, 1, gScenarioBufferData335_BOX_5X5_08U, 48, gScenarioKernelData335_BOX_5X5_08U, 8},
{10, 28, 2112, 0, 1, gScenarioBufferData336_BOX_5X5_08U, 48, gScenarioKernelData336_BOX_5X5_08U, 8},
{12, 28, 2512, 0, 1, gScenarioBufferData337_BOX_5X5_08U, 48, gScenarioKernelData337_BOX_5X5_08U, 8},
{14, 28, 2912, 0, 1, gScenarioBufferData338_BOX_5X5_08U, 48, gScenarioKernelData338_BOX_5X5_08U, 8},
{16, 28, 3312, 0, 1, gScenarioBufferData339_BOX_5X5_08U, 48, gScenarioKernelData339_BOX_5X5_08U, 8},
{18, 28, 3712, 0, 1, gScenarioBufferData340_BOX_5X5_08U, 48, gScenarioKernelData340_BOX_5X5_08U, 8},
{2, 30, 548, 0, 1, gScenarioBufferData341_BOX_5X5_08U, 48, gScenarioKernelData341_BOX_5X5_08U, 8},
{4, 30, 976, 0, 1, gScenarioBufferData342_BOX_5X5_08U, 48, gScenarioKernelData342_BOX_5X5_08U, 8},
{6, 30, 1404, 0, 1, gScenarioBufferData343_BOX_5X5_08U, 48, gScenarioKernelData343_BOX_5X5_08U, 8},
{8, 30, 1832, 0, 1, gScenarioBufferData344_BOX_5X5_08U, 48, gScenarioKernelData344_BOX_5X5_08U, 8},
{10, 30, 2260, 0, 1, gScenarioBufferData345_BOX_5X5_08U, 48, gScenarioKernelData345_BOX_5X5_08U, 8},
{12, 30, 2688, 0, 1, gScenarioBufferData346_BOX_5X5_08U, 48, gScenarioKernelData346_BOX_5X5_08U, 8},
{14, 30, 3116, 0, 1, gScenarioBufferData347_BOX_5X5_08U, 48, gScenarioKernelData347_BOX_5X5_08U, 8},
{16, 30, 3544, 0, 1, gScenarioBufferData348_BOX_5X5_08U, 48, gScenarioKernelData348_BOX_5X5_08U, 8},
{2, 32, 584, 0, 1, gScenarioBufferData349_BOX_5X5_08U, 48, gScenarioKernelData349_BOX_5X5_08U, 8},
{4, 32, 1040, 0, 1, gScenarioBufferData350_BOX_5X5_08U, 48, gScenarioKernelData350_BOX_5X5_08U, 8},
{6, 32, 1496, 0, 1, gScenarioBufferData351_BOX_5X5_08U, 48, gScenarioKernelData351_BOX_5X5_08U, 8},
{8, 32, 1952, 0, 1, gScenarioBufferData352_BOX_5X5_08U, 48, gScenarioKernelData352_BOX_5X5_08U, 8},
{10, 32, 2408, 0, 1, gScenarioBufferData353_BOX_5X5_08U, 48, gScenarioKernelData353_BOX_5X5_08U, 8},
{12, 32, 2864, 0, 1, gScenarioBufferData354_BOX_5X5_08U, 48, gScenarioKernelData354_BOX_5X5_08U, 8},
{14, 32, 3320, 0, 1, gScenarioBufferData355_BOX_5X5_08U, 48, gScenarioKernelData355_BOX_5X5_08U, 8},
{16, 32, 3776, 0, 1, gScenarioBufferData356_BOX_5X5_08U, 48, gScenarioKernelData356_BOX_5X5_08U, 8}
};

static acf_scenario_list gScenarioList_BOX_5X5_08U = {
357, //number of scenarios
gScenarioArray_BOX_5X5_08U};
//**************************************************************

class BOX_5X5_08U : public ACF_Process_APU
{

public:
   BOX_5X5_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("BOX_5X5_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("BOX_5X5_08U",
                                        BOX_5X5_08U_LOAD_SEGMENTS,
                                        BOX_5X5_08U_LOAD_PMEM, BOX_5X5_08U_LOAD_PMEM_SIZE,
                                        BOX_5X5_08U_LOAD_DMEM, BOX_5X5_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(BOX_5X5_08U_APEX_LOG_BUFFER, BOX_5X5_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_BOX_5X5_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_BOX_5X5_08U

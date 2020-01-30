#ifndef _ACF_PROCESS_APU_SCHARR_3X3_08U
#define _ACF_PROCESS_APU_SCHARR_3X3_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <SCHARR_3X3_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_SCHARR_3X3_08U[] = {{2, 1, 4, 0, {1, 1, 2, 2}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_SCHARR_3X3_08U[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_SCHARR_3X3_08U[] = {{6, 1, 4, 0, {1, 1, 2, 2}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_SCHARR_3X3_08U[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_SCHARR_3X3_08U[] = {{10, 1, 4, 0, {1, 1, 2, 2}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_SCHARR_3X3_08U[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_SCHARR_3X3_08U[] = {{14, 1, 4, 0, {1, 1, 2, 2}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_SCHARR_3X3_08U[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_SCHARR_3X3_08U[] = {{18, 1, 4, 0, {1, 1, 2, 2}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_SCHARR_3X3_08U[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_SCHARR_3X3_08U[] = {{22, 1, 4, 0, {1, 1, 2, 2}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_SCHARR_3X3_08U[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_SCHARR_3X3_08U[] = {{26, 1, 4, 0, {1, 1, 2, 2}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_SCHARR_3X3_08U[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_SCHARR_3X3_08U[] = {{30, 1, 4, 0, {1, 1, 2, 2}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_SCHARR_3X3_08U[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_SCHARR_3X3_08U[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_SCHARR_3X3_08U[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_SCHARR_3X3_08U[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_SCHARR_3X3_08U[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_SCHARR_3X3_08U[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_SCHARR_3X3_08U[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_SCHARR_3X3_08U[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_SCHARR_3X3_08U[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_SCHARR_3X3_08U[] = {{72, 1, 4, 0, {1, 1, 2, 2}}, {72, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_SCHARR_3X3_08U[] = {{80, 1, 4, 0, {1, 1, 2, 2}}, {80, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_SCHARR_3X3_08U[] = {{88, 1, 4, 0, {1, 1, 2, 2}}, {88, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_SCHARR_3X3_08U[] = {{96, 1, 4, 0, {1, 1, 2, 2}}, {96, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_SCHARR_3X3_08U[] = {{104, 1, 4, 0, {1, 1, 2, 2}}, {104, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_SCHARR_3X3_08U[] = {{112, 1, 4, 0, {1, 1, 2, 2}}, {112, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_SCHARR_3X3_08U[] = {{120, 1, 4, 0, {1, 1, 2, 2}}, {120, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_SCHARR_3X3_08U[] = {{128, 1, 4, 0, {1, 1, 2, 2}}, {128, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_SCHARR_3X3_08U[] = {{2, 2, 4, 0, {1, 1, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_SCHARR_3X3_08U[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_SCHARR_3X3_08U[] = {{6, 2, 4, 0, {1, 1, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_SCHARR_3X3_08U[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_SCHARR_3X3_08U[] = {{10, 2, 4, 0, {1, 1, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_SCHARR_3X3_08U[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_SCHARR_3X3_08U[] = {{14, 2, 4, 0, {1, 1, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_SCHARR_3X3_08U[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_SCHARR_3X3_08U[] = {{18, 2, 4, 0, {1, 1, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_SCHARR_3X3_08U[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_SCHARR_3X3_08U[] = {{22, 2, 4, 0, {1, 1, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_SCHARR_3X3_08U[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_SCHARR_3X3_08U[] = {{26, 2, 4, 0, {1, 1, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_SCHARR_3X3_08U[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_SCHARR_3X3_08U[] = {{30, 2, 4, 0, {1, 1, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_SCHARR_3X3_08U[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_SCHARR_3X3_08U[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_SCHARR_3X3_08U[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_SCHARR_3X3_08U[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_SCHARR_3X3_08U[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_SCHARR_3X3_08U[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_SCHARR_3X3_08U[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_SCHARR_3X3_08U[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_SCHARR_3X3_08U[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_SCHARR_3X3_08U[] = {{72, 2, 4, 0, {1, 1, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_SCHARR_3X3_08U[] = {{80, 2, 4, 0, {1, 1, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_SCHARR_3X3_08U[] = {{88, 2, 4, 0, {1, 1, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_SCHARR_3X3_08U[] = {{96, 2, 4, 0, {1, 1, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_SCHARR_3X3_08U[] = {{104, 2, 4, 0, {1, 1, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_SCHARR_3X3_08U[] = {{112, 2, 4, 0, {1, 1, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_SCHARR_3X3_08U[] = {{120, 2, 4, 0, {1, 1, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_SCHARR_3X3_08U[] = {{128, 2, 4, 0, {1, 1, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_SCHARR_3X3_08U[] = {{2, 3, 4, 0, {1, 1, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_SCHARR_3X3_08U[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_SCHARR_3X3_08U[] = {{6, 3, 4, 0, {1, 1, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_SCHARR_3X3_08U[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_SCHARR_3X3_08U[] = {{10, 3, 4, 0, {1, 1, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_SCHARR_3X3_08U[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_SCHARR_3X3_08U[] = {{14, 3, 4, 0, {1, 1, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_SCHARR_3X3_08U[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_SCHARR_3X3_08U[] = {{18, 3, 4, 0, {1, 1, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_SCHARR_3X3_08U[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_SCHARR_3X3_08U[] = {{22, 3, 4, 0, {1, 1, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_SCHARR_3X3_08U[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_SCHARR_3X3_08U[] = {{26, 3, 4, 0, {1, 1, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_SCHARR_3X3_08U[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_SCHARR_3X3_08U[] = {{30, 3, 4, 0, {1, 1, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_SCHARR_3X3_08U[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_SCHARR_3X3_08U[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_SCHARR_3X3_08U[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_SCHARR_3X3_08U[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_SCHARR_3X3_08U[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_SCHARR_3X3_08U[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_SCHARR_3X3_08U[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_SCHARR_3X3_08U[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_SCHARR_3X3_08U[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_SCHARR_3X3_08U[] = {{72, 3, 4, 0, {1, 1, 2, 2}}, {72, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_SCHARR_3X3_08U[] = {{80, 3, 4, 0, {1, 1, 2, 2}}, {80, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_SCHARR_3X3_08U[] = {{88, 3, 4, 0, {1, 1, 2, 2}}, {88, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_SCHARR_3X3_08U[] = {{96, 3, 4, 0, {1, 1, 2, 2}}, {96, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_SCHARR_3X3_08U[] = {{104, 3, 4, 0, {1, 1, 2, 2}}, {104, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_SCHARR_3X3_08U[] = {{112, 3, 4, 0, {1, 1, 2, 2}}, {112, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_SCHARR_3X3_08U[] = {{120, 3, 4, 0, {1, 1, 2, 2}}, {120, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_SCHARR_3X3_08U[] = {{128, 3, 4, 0, {1, 1, 2, 2}}, {128, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_SCHARR_3X3_08U[] = {{2, 4, 4, 0, {1, 1, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_SCHARR_3X3_08U[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_SCHARR_3X3_08U[] = {{6, 4, 4, 0, {1, 1, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_SCHARR_3X3_08U[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_SCHARR_3X3_08U[] = {{10, 4, 4, 0, {1, 1, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_SCHARR_3X3_08U[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_SCHARR_3X3_08U[] = {{14, 4, 4, 0, {1, 1, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_SCHARR_3X3_08U[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_SCHARR_3X3_08U[] = {{18, 4, 4, 0, {1, 1, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_SCHARR_3X3_08U[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_SCHARR_3X3_08U[] = {{22, 4, 4, 0, {1, 1, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_SCHARR_3X3_08U[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_SCHARR_3X3_08U[] = {{26, 4, 4, 0, {1, 1, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_SCHARR_3X3_08U[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_SCHARR_3X3_08U[] = {{30, 4, 4, 0, {1, 1, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_SCHARR_3X3_08U[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_SCHARR_3X3_08U[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_SCHARR_3X3_08U[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_SCHARR_3X3_08U[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_SCHARR_3X3_08U[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_SCHARR_3X3_08U[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_SCHARR_3X3_08U[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_SCHARR_3X3_08U[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_SCHARR_3X3_08U[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_SCHARR_3X3_08U[] = {{72, 4, 4, 0, {1, 1, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_SCHARR_3X3_08U[] = {{80, 4, 4, 0, {1, 1, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_SCHARR_3X3_08U[] = {{88, 4, 4, 0, {1, 1, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_SCHARR_3X3_08U[] = {{96, 4, 4, 0, {1, 1, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_SCHARR_3X3_08U[] = {{104, 4, 4, 0, {1, 1, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_SCHARR_3X3_08U[] = {{112, 4, 4, 0, {1, 1, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_SCHARR_3X3_08U[] = {{120, 4, 4, 0, {1, 1, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_SCHARR_3X3_08U[] = {{128, 4, 4, 0, {1, 1, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_SCHARR_3X3_08U[] = {{2, 5, 4, 0, {1, 1, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_SCHARR_3X3_08U[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_SCHARR_3X3_08U[] = {{6, 5, 4, 0, {1, 1, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_SCHARR_3X3_08U[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_SCHARR_3X3_08U[] = {{10, 5, 4, 0, {1, 1, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_SCHARR_3X3_08U[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_SCHARR_3X3_08U[] = {{14, 5, 4, 0, {1, 1, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_SCHARR_3X3_08U[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_SCHARR_3X3_08U[] = {{18, 5, 4, 0, {1, 1, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_SCHARR_3X3_08U[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_SCHARR_3X3_08U[] = {{22, 5, 4, 0, {1, 1, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_SCHARR_3X3_08U[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_SCHARR_3X3_08U[] = {{26, 5, 4, 0, {1, 1, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_SCHARR_3X3_08U[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_SCHARR_3X3_08U[] = {{30, 5, 4, 0, {1, 1, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_SCHARR_3X3_08U[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_SCHARR_3X3_08U[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_SCHARR_3X3_08U[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_SCHARR_3X3_08U[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_SCHARR_3X3_08U[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_SCHARR_3X3_08U[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_SCHARR_3X3_08U[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_SCHARR_3X3_08U[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_SCHARR_3X3_08U[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_SCHARR_3X3_08U[] = {{72, 5, 4, 0, {1, 1, 2, 2}}, {72, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_SCHARR_3X3_08U[] = {{80, 5, 4, 0, {1, 1, 2, 2}}, {80, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_SCHARR_3X3_08U[] = {{88, 5, 4, 0, {1, 1, 2, 2}}, {88, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_SCHARR_3X3_08U[] = {{96, 5, 4, 0, {1, 1, 2, 2}}, {96, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_SCHARR_3X3_08U[] = {{104, 5, 4, 0, {1, 1, 2, 2}}, {104, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_SCHARR_3X3_08U[] = {{112, 5, 4, 0, {1, 1, 2, 2}}, {112, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_SCHARR_3X3_08U[] = {{2, 6, 4, 0, {1, 1, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_SCHARR_3X3_08U[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_SCHARR_3X3_08U[] = {{6, 6, 4, 0, {1, 1, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_SCHARR_3X3_08U[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_SCHARR_3X3_08U[] = {{10, 6, 4, 0, {1, 1, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_SCHARR_3X3_08U[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_SCHARR_3X3_08U[] = {{14, 6, 4, 0, {1, 1, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_SCHARR_3X3_08U[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_SCHARR_3X3_08U[] = {{18, 6, 4, 0, {1, 1, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_SCHARR_3X3_08U[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_SCHARR_3X3_08U[] = {{22, 6, 4, 0, {1, 1, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_SCHARR_3X3_08U[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_SCHARR_3X3_08U[] = {{26, 6, 4, 0, {1, 1, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_SCHARR_3X3_08U[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_SCHARR_3X3_08U[] = {{30, 6, 4, 0, {1, 1, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_SCHARR_3X3_08U[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_SCHARR_3X3_08U[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_SCHARR_3X3_08U[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_SCHARR_3X3_08U[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_SCHARR_3X3_08U[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_SCHARR_3X3_08U[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_SCHARR_3X3_08U[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_SCHARR_3X3_08U[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_SCHARR_3X3_08U[] = {{64, 6, 4, 0, {1, 1, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_SCHARR_3X3_08U[] = {{72, 6, 4, 0, {1, 1, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_SCHARR_3X3_08U[] = {{80, 6, 4, 0, {1, 1, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_SCHARR_3X3_08U[] = {{88, 6, 4, 0, {1, 1, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_SCHARR_3X3_08U[] = {{96, 6, 4, 0, {1, 1, 2, 2}}, {96, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_SCHARR_3X3_08U[] = {{2, 8, 4, 0, {1, 1, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_SCHARR_3X3_08U[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_SCHARR_3X3_08U[] = {{6, 8, 4, 0, {1, 1, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_SCHARR_3X3_08U[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_SCHARR_3X3_08U[] = {{10, 8, 4, 0, {1, 1, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_SCHARR_3X3_08U[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_SCHARR_3X3_08U[] = {{14, 8, 4, 0, {1, 1, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_SCHARR_3X3_08U[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_SCHARR_3X3_08U[] = {{18, 8, 4, 0, {1, 1, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_SCHARR_3X3_08U[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_SCHARR_3X3_08U[] = {{22, 8, 4, 0, {1, 1, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_SCHARR_3X3_08U[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_SCHARR_3X3_08U[] = {{26, 8, 4, 0, {1, 1, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_SCHARR_3X3_08U[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_SCHARR_3X3_08U[] = {{30, 8, 4, 0, {1, 1, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_SCHARR_3X3_08U[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_SCHARR_3X3_08U[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_SCHARR_3X3_08U[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_SCHARR_3X3_08U[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_SCHARR_3X3_08U[] = {{48, 8, 4, 0, {1, 1, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_SCHARR_3X3_08U[] = {{52, 8, 4, 0, {1, 1, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_SCHARR_3X3_08U[] = {{56, 8, 4, 0, {1, 1, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_SCHARR_3X3_08U[] = {{60, 8, 4, 0, {1, 1, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_SCHARR_3X3_08U[] = {{64, 8, 4, 0, {1, 1, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_SCHARR_3X3_08U[] = {{72, 8, 4, 0, {1, 1, 2, 2}}, {72, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_SCHARR_3X3_08U[] = {{2, 10, 4, 0, {1, 1, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_SCHARR_3X3_08U[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_SCHARR_3X3_08U[] = {{6, 10, 4, 0, {1, 1, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_SCHARR_3X3_08U[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_SCHARR_3X3_08U[] = {{10, 10, 4, 0, {1, 1, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_SCHARR_3X3_08U[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_SCHARR_3X3_08U[] = {{14, 10, 4, 0, {1, 1, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_SCHARR_3X3_08U[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_SCHARR_3X3_08U[] = {{18, 10, 4, 0, {1, 1, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_SCHARR_3X3_08U[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_SCHARR_3X3_08U[] = {{22, 10, 4, 0, {1, 1, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_SCHARR_3X3_08U[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_SCHARR_3X3_08U[] = {{26, 10, 4, 0, {1, 1, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_SCHARR_3X3_08U[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_SCHARR_3X3_08U[] = {{30, 10, 4, 0, {1, 1, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_SCHARR_3X3_08U[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_SCHARR_3X3_08U[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_SCHARR_3X3_08U[] = {{40, 10, 4, 0, {1, 1, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_SCHARR_3X3_08U[] = {{44, 10, 4, 0, {1, 1, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_SCHARR_3X3_08U[] = {{48, 10, 4, 0, {1, 1, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_SCHARR_3X3_08U[] = {{52, 10, 4, 0, {1, 1, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_SCHARR_3X3_08U[] = {{56, 10, 4, 0, {1, 1, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_SCHARR_3X3_08U[] = {{2, 12, 4, 0, {1, 1, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_SCHARR_3X3_08U[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_SCHARR_3X3_08U[] = {{6, 12, 4, 0, {1, 1, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_SCHARR_3X3_08U[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_SCHARR_3X3_08U[] = {{10, 12, 4, 0, {1, 1, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_SCHARR_3X3_08U[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_SCHARR_3X3_08U[] = {{14, 12, 4, 0, {1, 1, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_SCHARR_3X3_08U[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_SCHARR_3X3_08U[] = {{18, 12, 4, 0, {1, 1, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_SCHARR_3X3_08U[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_SCHARR_3X3_08U[] = {{22, 12, 4, 0, {1, 1, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_SCHARR_3X3_08U[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_SCHARR_3X3_08U[] = {{26, 12, 4, 0, {1, 1, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_SCHARR_3X3_08U[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_SCHARR_3X3_08U[] = {{30, 12, 4, 0, {1, 1, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_SCHARR_3X3_08U[] = {{32, 12, 4, 0, {1, 1, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_SCHARR_3X3_08U[] = {{36, 12, 4, 0, {1, 1, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_SCHARR_3X3_08U[] = {{40, 12, 4, 0, {1, 1, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_SCHARR_3X3_08U[] = {{44, 12, 4, 0, {1, 1, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_SCHARR_3X3_08U[] = {{48, 12, 4, 0, {1, 1, 2, 2}}, {48, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_SCHARR_3X3_08U[] = {{2, 14, 4, 0, {1, 1, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_SCHARR_3X3_08U[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_SCHARR_3X3_08U[] = {{6, 14, 4, 0, {1, 1, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_SCHARR_3X3_08U[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_SCHARR_3X3_08U[] = {{10, 14, 4, 0, {1, 1, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_SCHARR_3X3_08U[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_SCHARR_3X3_08U[] = {{14, 14, 4, 0, {1, 1, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_SCHARR_3X3_08U[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_SCHARR_3X3_08U[] = {{18, 14, 4, 0, {1, 1, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_SCHARR_3X3_08U[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_SCHARR_3X3_08U[] = {{22, 14, 4, 0, {1, 1, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_SCHARR_3X3_08U[] = {{24, 14, 4, 0, {1, 1, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_SCHARR_3X3_08U[] = {{26, 14, 4, 0, {1, 1, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_SCHARR_3X3_08U[] = {{28, 14, 4, 0, {1, 1, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_SCHARR_3X3_08U[] = {{30, 14, 4, 0, {1, 1, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_SCHARR_3X3_08U[] = {{32, 14, 4, 0, {1, 1, 2, 2}}, {32, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_SCHARR_3X3_08U[] = {{36, 14, 4, 0, {1, 1, 2, 2}}, {36, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_SCHARR_3X3_08U[] = {{40, 14, 4, 0, {1, 1, 2, 2}}, {40, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_SCHARR_3X3_08U[] = {{2, 16, 4, 0, {1, 1, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_SCHARR_3X3_08U[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_SCHARR_3X3_08U[] = {{6, 16, 4, 0, {1, 1, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_SCHARR_3X3_08U[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_SCHARR_3X3_08U[] = {{10, 16, 4, 0, {1, 1, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_SCHARR_3X3_08U[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_SCHARR_3X3_08U[] = {{14, 16, 4, 0, {1, 1, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_SCHARR_3X3_08U[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_SCHARR_3X3_08U[] = {{18, 16, 4, 0, {1, 1, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_SCHARR_3X3_08U[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_SCHARR_3X3_08U[] = {{22, 16, 4, 0, {1, 1, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_SCHARR_3X3_08U[] = {{24, 16, 4, 0, {1, 1, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_SCHARR_3X3_08U[] = {{26, 16, 4, 0, {1, 1, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_SCHARR_3X3_08U[] = {{28, 16, 4, 0, {1, 1, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_SCHARR_3X3_08U[] = {{30, 16, 4, 0, {1, 1, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_SCHARR_3X3_08U[] = {{32, 16, 4, 0, {1, 1, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_SCHARR_3X3_08U[] = {{36, 16, 4, 0, {1, 1, 2, 2}}, {36, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_SCHARR_3X3_08U[] = {{2, 18, 4, 0, {1, 1, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_SCHARR_3X3_08U[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_SCHARR_3X3_08U[] = {{6, 18, 4, 0, {1, 1, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_SCHARR_3X3_08U[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_SCHARR_3X3_08U[] = {{10, 18, 4, 0, {1, 1, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_SCHARR_3X3_08U[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_SCHARR_3X3_08U[] = {{14, 18, 4, 0, {1, 1, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_SCHARR_3X3_08U[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_SCHARR_3X3_08U[] = {{18, 18, 4, 0, {1, 1, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_SCHARR_3X3_08U[] = {{20, 18, 4, 0, {1, 1, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_SCHARR_3X3_08U[] = {{22, 18, 4, 0, {1, 1, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_SCHARR_3X3_08U[] = {{24, 18, 4, 0, {1, 1, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_SCHARR_3X3_08U[] = {{26, 18, 4, 0, {1, 1, 2, 2}}, {26, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_SCHARR_3X3_08U[] = {{28, 18, 4, 0, {1, 1, 2, 2}}, {28, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_SCHARR_3X3_08U[] = {{30, 18, 4, 0, {1, 1, 2, 2}}, {30, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_SCHARR_3X3_08U[] = {{32, 18, 4, 0, {1, 1, 2, 2}}, {32, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_SCHARR_3X3_08U[] = {{2, 20, 4, 0, {1, 1, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_SCHARR_3X3_08U[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_SCHARR_3X3_08U[] = {{6, 20, 4, 0, {1, 1, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_SCHARR_3X3_08U[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_SCHARR_3X3_08U[] = {{10, 20, 4, 0, {1, 1, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_SCHARR_3X3_08U[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_SCHARR_3X3_08U[] = {{14, 20, 4, 0, {1, 1, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_SCHARR_3X3_08U[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_SCHARR_3X3_08U[] = {{18, 20, 4, 0, {1, 1, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_SCHARR_3X3_08U[] = {{20, 20, 4, 0, {1, 1, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_SCHARR_3X3_08U[] = {{22, 20, 4, 0, {1, 1, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_SCHARR_3X3_08U[] = {{24, 20, 4, 0, {1, 1, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_SCHARR_3X3_08U[] = {{26, 20, 4, 0, {1, 1, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_SCHARR_3X3_08U[] = {{28, 20, 4, 0, {1, 1, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_SCHARR_3X3_08U[] = {{2, 22, 4, 0, {1, 1, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_SCHARR_3X3_08U[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_SCHARR_3X3_08U[] = {{6, 22, 4, 0, {1, 1, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_SCHARR_3X3_08U[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_SCHARR_3X3_08U[] = {{10, 22, 4, 0, {1, 1, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_SCHARR_3X3_08U[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_SCHARR_3X3_08U[] = {{14, 22, 4, 0, {1, 1, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_SCHARR_3X3_08U[] = {{16, 22, 4, 0, {1, 1, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_SCHARR_3X3_08U[] = {{18, 22, 4, 0, {1, 1, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_SCHARR_3X3_08U[] = {{20, 22, 4, 0, {1, 1, 2, 2}}, {20, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_SCHARR_3X3_08U[] = {{22, 22, 4, 0, {1, 1, 2, 2}}, {22, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_SCHARR_3X3_08U[] = {{24, 22, 4, 0, {1, 1, 2, 2}}, {24, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_SCHARR_3X3_08U[] = {{2, 24, 4, 0, {1, 1, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_SCHARR_3X3_08U[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_SCHARR_3X3_08U[] = {{6, 24, 4, 0, {1, 1, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_SCHARR_3X3_08U[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_SCHARR_3X3_08U[] = {{10, 24, 4, 0, {1, 1, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_SCHARR_3X3_08U[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_SCHARR_3X3_08U[] = {{14, 24, 4, 0, {1, 1, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_SCHARR_3X3_08U[] = {{16, 24, 4, 0, {1, 1, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_SCHARR_3X3_08U[] = {{18, 24, 4, 0, {1, 1, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_SCHARR_3X3_08U[] = {{20, 24, 4, 0, {1, 1, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_SCHARR_3X3_08U[] = {{22, 24, 4, 0, {1, 1, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_SCHARR_3X3_08U[] = {{2, 26, 4, 0, {1, 1, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_SCHARR_3X3_08U[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_SCHARR_3X3_08U[] = {{6, 26, 4, 0, {1, 1, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_SCHARR_3X3_08U[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_SCHARR_3X3_08U[] = {{10, 26, 4, 0, {1, 1, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_SCHARR_3X3_08U[] = {{12, 26, 4, 0, {1, 1, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_SCHARR_3X3_08U[] = {{14, 26, 4, 0, {1, 1, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_SCHARR_3X3_08U[] = {{16, 26, 4, 0, {1, 1, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_SCHARR_3X3_08U[] = {{18, 26, 4, 0, {1, 1, 2, 2}}, {18, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_SCHARR_3X3_08U[] = {{20, 26, 4, 0, {1, 1, 2, 2}}, {20, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_SCHARR_3X3_08U[] = {{2, 28, 4, 0, {1, 1, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_SCHARR_3X3_08U[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_SCHARR_3X3_08U[] = {{6, 28, 4, 0, {1, 1, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_SCHARR_3X3_08U[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_SCHARR_3X3_08U[] = {{10, 28, 4, 0, {1, 1, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_SCHARR_3X3_08U[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_SCHARR_3X3_08U[] = {{14, 28, 4, 0, {1, 1, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_SCHARR_3X3_08U[] = {{16, 28, 4, 0, {1, 1, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_SCHARR_3X3_08U[] = {{18, 28, 4, 0, {1, 1, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_SCHARR_3X3_08U[] = {{2, 30, 4, 0, {1, 1, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_SCHARR_3X3_08U[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_SCHARR_3X3_08U[] = {{6, 30, 4, 0, {1, 1, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_SCHARR_3X3_08U[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_SCHARR_3X3_08U[] = {{10, 30, 4, 0, {1, 1, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_SCHARR_3X3_08U[] = {{12, 30, 4, 0, {1, 1, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_SCHARR_3X3_08U[] = {{14, 30, 4, 0, {1, 1, 2, 2}}, {14, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_SCHARR_3X3_08U[] = {{16, 30, 4, 0, {1, 1, 2, 2}}, {16, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_SCHARR_3X3_08U[] = {{18, 30, 4, 0, {1, 1, 2, 2}}, {18, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_SCHARR_3X3_08U[] = {{2, 32, 4, 0, {1, 1, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_SCHARR_3X3_08U[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_SCHARR_3X3_08U[] = {{6, 32, 4, 0, {1, 1, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_SCHARR_3X3_08U[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_SCHARR_3X3_08U[] = {{10, 32, 4, 0, {1, 1, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_SCHARR_3X3_08U[] = {{12, 32, 4, 0, {1, 1, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_SCHARR_3X3_08U[] = {{14, 32, 4, 0, {1, 1, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_SCHARR_3X3_08U[] = {{16, 32, 4, 0, {1, 1, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_SCHARR_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_SCHARR_3X3_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_SCHARR_3X3_08U[] = {
{2, 1, 40, 0, 1, gScenarioBufferData0_SCHARR_3X3_08U, 32, gScenarioKernelData0_SCHARR_3X3_08U, 4},
{4, 1, 56, 0, 1, gScenarioBufferData1_SCHARR_3X3_08U, 32, gScenarioKernelData1_SCHARR_3X3_08U, 4},
{6, 1, 72, 0, 1, gScenarioBufferData2_SCHARR_3X3_08U, 32, gScenarioKernelData2_SCHARR_3X3_08U, 4},
{8, 1, 88, 0, 1, gScenarioBufferData3_SCHARR_3X3_08U, 32, gScenarioKernelData3_SCHARR_3X3_08U, 4},
{10, 1, 104, 0, 1, gScenarioBufferData4_SCHARR_3X3_08U, 32, gScenarioKernelData4_SCHARR_3X3_08U, 4},
{12, 1, 120, 0, 1, gScenarioBufferData5_SCHARR_3X3_08U, 32, gScenarioKernelData5_SCHARR_3X3_08U, 4},
{14, 1, 136, 0, 1, gScenarioBufferData6_SCHARR_3X3_08U, 32, gScenarioKernelData6_SCHARR_3X3_08U, 4},
{16, 1, 152, 0, 1, gScenarioBufferData7_SCHARR_3X3_08U, 32, gScenarioKernelData7_SCHARR_3X3_08U, 4},
{18, 1, 168, 0, 1, gScenarioBufferData8_SCHARR_3X3_08U, 32, gScenarioKernelData8_SCHARR_3X3_08U, 4},
{20, 1, 184, 0, 1, gScenarioBufferData9_SCHARR_3X3_08U, 32, gScenarioKernelData9_SCHARR_3X3_08U, 4},
{22, 1, 200, 0, 1, gScenarioBufferData10_SCHARR_3X3_08U, 32, gScenarioKernelData10_SCHARR_3X3_08U, 4},
{24, 1, 216, 0, 1, gScenarioBufferData11_SCHARR_3X3_08U, 32, gScenarioKernelData11_SCHARR_3X3_08U, 4},
{26, 1, 232, 0, 1, gScenarioBufferData12_SCHARR_3X3_08U, 32, gScenarioKernelData12_SCHARR_3X3_08U, 4},
{28, 1, 248, 0, 1, gScenarioBufferData13_SCHARR_3X3_08U, 32, gScenarioKernelData13_SCHARR_3X3_08U, 4},
{30, 1, 264, 0, 1, gScenarioBufferData14_SCHARR_3X3_08U, 32, gScenarioKernelData14_SCHARR_3X3_08U, 4},
{32, 1, 280, 0, 1, gScenarioBufferData15_SCHARR_3X3_08U, 32, gScenarioKernelData15_SCHARR_3X3_08U, 4},
{36, 1, 312, 0, 1, gScenarioBufferData16_SCHARR_3X3_08U, 32, gScenarioKernelData16_SCHARR_3X3_08U, 4},
{40, 1, 344, 0, 1, gScenarioBufferData17_SCHARR_3X3_08U, 32, gScenarioKernelData17_SCHARR_3X3_08U, 4},
{44, 1, 376, 0, 1, gScenarioBufferData18_SCHARR_3X3_08U, 32, gScenarioKernelData18_SCHARR_3X3_08U, 4},
{48, 1, 408, 0, 1, gScenarioBufferData19_SCHARR_3X3_08U, 32, gScenarioKernelData19_SCHARR_3X3_08U, 4},
{52, 1, 440, 0, 1, gScenarioBufferData20_SCHARR_3X3_08U, 32, gScenarioKernelData20_SCHARR_3X3_08U, 4},
{56, 1, 472, 0, 1, gScenarioBufferData21_SCHARR_3X3_08U, 32, gScenarioKernelData21_SCHARR_3X3_08U, 4},
{60, 1, 504, 0, 1, gScenarioBufferData22_SCHARR_3X3_08U, 32, gScenarioKernelData22_SCHARR_3X3_08U, 4},
{64, 1, 536, 0, 1, gScenarioBufferData23_SCHARR_3X3_08U, 32, gScenarioKernelData23_SCHARR_3X3_08U, 4},
{72, 1, 600, 0, 1, gScenarioBufferData24_SCHARR_3X3_08U, 32, gScenarioKernelData24_SCHARR_3X3_08U, 4},
{80, 1, 664, 0, 1, gScenarioBufferData25_SCHARR_3X3_08U, 32, gScenarioKernelData25_SCHARR_3X3_08U, 4},
{88, 1, 728, 0, 1, gScenarioBufferData26_SCHARR_3X3_08U, 32, gScenarioKernelData26_SCHARR_3X3_08U, 4},
{96, 1, 792, 0, 1, gScenarioBufferData27_SCHARR_3X3_08U, 32, gScenarioKernelData27_SCHARR_3X3_08U, 4},
{104, 1, 856, 0, 1, gScenarioBufferData28_SCHARR_3X3_08U, 32, gScenarioKernelData28_SCHARR_3X3_08U, 4},
{112, 1, 920, 0, 1, gScenarioBufferData29_SCHARR_3X3_08U, 32, gScenarioKernelData29_SCHARR_3X3_08U, 4},
{120, 1, 984, 0, 1, gScenarioBufferData30_SCHARR_3X3_08U, 32, gScenarioKernelData30_SCHARR_3X3_08U, 4},
{128, 1, 1048, 0, 1, gScenarioBufferData31_SCHARR_3X3_08U, 32, gScenarioKernelData31_SCHARR_3X3_08U, 4},
{2, 2, 68, 0, 1, gScenarioBufferData32_SCHARR_3X3_08U, 32, gScenarioKernelData32_SCHARR_3X3_08U, 4},
{4, 2, 96, 0, 1, gScenarioBufferData33_SCHARR_3X3_08U, 32, gScenarioKernelData33_SCHARR_3X3_08U, 4},
{6, 2, 124, 0, 1, gScenarioBufferData34_SCHARR_3X3_08U, 32, gScenarioKernelData34_SCHARR_3X3_08U, 4},
{8, 2, 152, 0, 1, gScenarioBufferData35_SCHARR_3X3_08U, 32, gScenarioKernelData35_SCHARR_3X3_08U, 4},
{10, 2, 180, 0, 1, gScenarioBufferData36_SCHARR_3X3_08U, 32, gScenarioKernelData36_SCHARR_3X3_08U, 4},
{12, 2, 208, 0, 1, gScenarioBufferData37_SCHARR_3X3_08U, 32, gScenarioKernelData37_SCHARR_3X3_08U, 4},
{14, 2, 236, 0, 1, gScenarioBufferData38_SCHARR_3X3_08U, 32, gScenarioKernelData38_SCHARR_3X3_08U, 4},
{16, 2, 264, 0, 1, gScenarioBufferData39_SCHARR_3X3_08U, 32, gScenarioKernelData39_SCHARR_3X3_08U, 4},
{18, 2, 292, 0, 1, gScenarioBufferData40_SCHARR_3X3_08U, 32, gScenarioKernelData40_SCHARR_3X3_08U, 4},
{20, 2, 320, 0, 1, gScenarioBufferData41_SCHARR_3X3_08U, 32, gScenarioKernelData41_SCHARR_3X3_08U, 4},
{22, 2, 348, 0, 1, gScenarioBufferData42_SCHARR_3X3_08U, 32, gScenarioKernelData42_SCHARR_3X3_08U, 4},
{24, 2, 376, 0, 1, gScenarioBufferData43_SCHARR_3X3_08U, 32, gScenarioKernelData43_SCHARR_3X3_08U, 4},
{26, 2, 404, 0, 1, gScenarioBufferData44_SCHARR_3X3_08U, 32, gScenarioKernelData44_SCHARR_3X3_08U, 4},
{28, 2, 432, 0, 1, gScenarioBufferData45_SCHARR_3X3_08U, 32, gScenarioKernelData45_SCHARR_3X3_08U, 4},
{30, 2, 460, 0, 1, gScenarioBufferData46_SCHARR_3X3_08U, 32, gScenarioKernelData46_SCHARR_3X3_08U, 4},
{32, 2, 488, 0, 1, gScenarioBufferData47_SCHARR_3X3_08U, 32, gScenarioKernelData47_SCHARR_3X3_08U, 4},
{36, 2, 544, 0, 1, gScenarioBufferData48_SCHARR_3X3_08U, 32, gScenarioKernelData48_SCHARR_3X3_08U, 4},
{40, 2, 600, 0, 1, gScenarioBufferData49_SCHARR_3X3_08U, 32, gScenarioKernelData49_SCHARR_3X3_08U, 4},
{44, 2, 656, 0, 1, gScenarioBufferData50_SCHARR_3X3_08U, 32, gScenarioKernelData50_SCHARR_3X3_08U, 4},
{48, 2, 712, 0, 1, gScenarioBufferData51_SCHARR_3X3_08U, 32, gScenarioKernelData51_SCHARR_3X3_08U, 4},
{52, 2, 768, 0, 1, gScenarioBufferData52_SCHARR_3X3_08U, 32, gScenarioKernelData52_SCHARR_3X3_08U, 4},
{56, 2, 824, 0, 1, gScenarioBufferData53_SCHARR_3X3_08U, 32, gScenarioKernelData53_SCHARR_3X3_08U, 4},
{60, 2, 880, 0, 1, gScenarioBufferData54_SCHARR_3X3_08U, 32, gScenarioKernelData54_SCHARR_3X3_08U, 4},
{64, 2, 936, 0, 1, gScenarioBufferData55_SCHARR_3X3_08U, 32, gScenarioKernelData55_SCHARR_3X3_08U, 4},
{72, 2, 1048, 0, 1, gScenarioBufferData56_SCHARR_3X3_08U, 32, gScenarioKernelData56_SCHARR_3X3_08U, 4},
{80, 2, 1160, 0, 1, gScenarioBufferData57_SCHARR_3X3_08U, 32, gScenarioKernelData57_SCHARR_3X3_08U, 4},
{88, 2, 1272, 0, 1, gScenarioBufferData58_SCHARR_3X3_08U, 32, gScenarioKernelData58_SCHARR_3X3_08U, 4},
{96, 2, 1384, 0, 1, gScenarioBufferData59_SCHARR_3X3_08U, 32, gScenarioKernelData59_SCHARR_3X3_08U, 4},
{104, 2, 1496, 0, 1, gScenarioBufferData60_SCHARR_3X3_08U, 32, gScenarioKernelData60_SCHARR_3X3_08U, 4},
{112, 2, 1608, 0, 1, gScenarioBufferData61_SCHARR_3X3_08U, 32, gScenarioKernelData61_SCHARR_3X3_08U, 4},
{120, 2, 1720, 0, 1, gScenarioBufferData62_SCHARR_3X3_08U, 32, gScenarioKernelData62_SCHARR_3X3_08U, 4},
{128, 2, 1832, 0, 1, gScenarioBufferData63_SCHARR_3X3_08U, 32, gScenarioKernelData63_SCHARR_3X3_08U, 4},
{2, 3, 96, 0, 1, gScenarioBufferData64_SCHARR_3X3_08U, 32, gScenarioKernelData64_SCHARR_3X3_08U, 4},
{4, 3, 136, 0, 1, gScenarioBufferData65_SCHARR_3X3_08U, 32, gScenarioKernelData65_SCHARR_3X3_08U, 4},
{6, 3, 176, 0, 1, gScenarioBufferData66_SCHARR_3X3_08U, 32, gScenarioKernelData66_SCHARR_3X3_08U, 4},
{8, 3, 216, 0, 1, gScenarioBufferData67_SCHARR_3X3_08U, 32, gScenarioKernelData67_SCHARR_3X3_08U, 4},
{10, 3, 256, 0, 1, gScenarioBufferData68_SCHARR_3X3_08U, 32, gScenarioKernelData68_SCHARR_3X3_08U, 4},
{12, 3, 296, 0, 1, gScenarioBufferData69_SCHARR_3X3_08U, 32, gScenarioKernelData69_SCHARR_3X3_08U, 4},
{14, 3, 336, 0, 1, gScenarioBufferData70_SCHARR_3X3_08U, 32, gScenarioKernelData70_SCHARR_3X3_08U, 4},
{16, 3, 376, 0, 1, gScenarioBufferData71_SCHARR_3X3_08U, 32, gScenarioKernelData71_SCHARR_3X3_08U, 4},
{18, 3, 416, 0, 1, gScenarioBufferData72_SCHARR_3X3_08U, 32, gScenarioKernelData72_SCHARR_3X3_08U, 4},
{20, 3, 456, 0, 1, gScenarioBufferData73_SCHARR_3X3_08U, 32, gScenarioKernelData73_SCHARR_3X3_08U, 4},
{22, 3, 496, 0, 1, gScenarioBufferData74_SCHARR_3X3_08U, 32, gScenarioKernelData74_SCHARR_3X3_08U, 4},
{24, 3, 536, 0, 1, gScenarioBufferData75_SCHARR_3X3_08U, 32, gScenarioKernelData75_SCHARR_3X3_08U, 4},
{26, 3, 576, 0, 1, gScenarioBufferData76_SCHARR_3X3_08U, 32, gScenarioKernelData76_SCHARR_3X3_08U, 4},
{28, 3, 616, 0, 1, gScenarioBufferData77_SCHARR_3X3_08U, 32, gScenarioKernelData77_SCHARR_3X3_08U, 4},
{30, 3, 656, 0, 1, gScenarioBufferData78_SCHARR_3X3_08U, 32, gScenarioKernelData78_SCHARR_3X3_08U, 4},
{32, 3, 696, 0, 1, gScenarioBufferData79_SCHARR_3X3_08U, 32, gScenarioKernelData79_SCHARR_3X3_08U, 4},
{36, 3, 776, 0, 1, gScenarioBufferData80_SCHARR_3X3_08U, 32, gScenarioKernelData80_SCHARR_3X3_08U, 4},
{40, 3, 856, 0, 1, gScenarioBufferData81_SCHARR_3X3_08U, 32, gScenarioKernelData81_SCHARR_3X3_08U, 4},
{44, 3, 936, 0, 1, gScenarioBufferData82_SCHARR_3X3_08U, 32, gScenarioKernelData82_SCHARR_3X3_08U, 4},
{48, 3, 1016, 0, 1, gScenarioBufferData83_SCHARR_3X3_08U, 32, gScenarioKernelData83_SCHARR_3X3_08U, 4},
{52, 3, 1096, 0, 1, gScenarioBufferData84_SCHARR_3X3_08U, 32, gScenarioKernelData84_SCHARR_3X3_08U, 4},
{56, 3, 1176, 0, 1, gScenarioBufferData85_SCHARR_3X3_08U, 32, gScenarioKernelData85_SCHARR_3X3_08U, 4},
{60, 3, 1256, 0, 1, gScenarioBufferData86_SCHARR_3X3_08U, 32, gScenarioKernelData86_SCHARR_3X3_08U, 4},
{64, 3, 1336, 0, 1, gScenarioBufferData87_SCHARR_3X3_08U, 32, gScenarioKernelData87_SCHARR_3X3_08U, 4},
{72, 3, 1496, 0, 1, gScenarioBufferData88_SCHARR_3X3_08U, 32, gScenarioKernelData88_SCHARR_3X3_08U, 4},
{80, 3, 1656, 0, 1, gScenarioBufferData89_SCHARR_3X3_08U, 32, gScenarioKernelData89_SCHARR_3X3_08U, 4},
{88, 3, 1816, 0, 1, gScenarioBufferData90_SCHARR_3X3_08U, 32, gScenarioKernelData90_SCHARR_3X3_08U, 4},
{96, 3, 1976, 0, 1, gScenarioBufferData91_SCHARR_3X3_08U, 32, gScenarioKernelData91_SCHARR_3X3_08U, 4},
{104, 3, 2136, 0, 1, gScenarioBufferData92_SCHARR_3X3_08U, 32, gScenarioKernelData92_SCHARR_3X3_08U, 4},
{112, 3, 2296, 0, 1, gScenarioBufferData93_SCHARR_3X3_08U, 32, gScenarioKernelData93_SCHARR_3X3_08U, 4},
{120, 3, 2456, 0, 1, gScenarioBufferData94_SCHARR_3X3_08U, 32, gScenarioKernelData94_SCHARR_3X3_08U, 4},
{128, 3, 2616, 0, 1, gScenarioBufferData95_SCHARR_3X3_08U, 32, gScenarioKernelData95_SCHARR_3X3_08U, 4},
{2, 4, 124, 0, 1, gScenarioBufferData96_SCHARR_3X3_08U, 32, gScenarioKernelData96_SCHARR_3X3_08U, 4},
{4, 4, 176, 0, 1, gScenarioBufferData97_SCHARR_3X3_08U, 32, gScenarioKernelData97_SCHARR_3X3_08U, 4},
{6, 4, 228, 0, 1, gScenarioBufferData98_SCHARR_3X3_08U, 32, gScenarioKernelData98_SCHARR_3X3_08U, 4},
{8, 4, 280, 0, 1, gScenarioBufferData99_SCHARR_3X3_08U, 32, gScenarioKernelData99_SCHARR_3X3_08U, 4},
{10, 4, 332, 0, 1, gScenarioBufferData100_SCHARR_3X3_08U, 32, gScenarioKernelData100_SCHARR_3X3_08U, 4},
{12, 4, 384, 0, 1, gScenarioBufferData101_SCHARR_3X3_08U, 32, gScenarioKernelData101_SCHARR_3X3_08U, 4},
{14, 4, 436, 0, 1, gScenarioBufferData102_SCHARR_3X3_08U, 32, gScenarioKernelData102_SCHARR_3X3_08U, 4},
{16, 4, 488, 0, 1, gScenarioBufferData103_SCHARR_3X3_08U, 32, gScenarioKernelData103_SCHARR_3X3_08U, 4},
{18, 4, 540, 0, 1, gScenarioBufferData104_SCHARR_3X3_08U, 32, gScenarioKernelData104_SCHARR_3X3_08U, 4},
{20, 4, 592, 0, 1, gScenarioBufferData105_SCHARR_3X3_08U, 32, gScenarioKernelData105_SCHARR_3X3_08U, 4},
{22, 4, 644, 0, 1, gScenarioBufferData106_SCHARR_3X3_08U, 32, gScenarioKernelData106_SCHARR_3X3_08U, 4},
{24, 4, 696, 0, 1, gScenarioBufferData107_SCHARR_3X3_08U, 32, gScenarioKernelData107_SCHARR_3X3_08U, 4},
{26, 4, 748, 0, 1, gScenarioBufferData108_SCHARR_3X3_08U, 32, gScenarioKernelData108_SCHARR_3X3_08U, 4},
{28, 4, 800, 0, 1, gScenarioBufferData109_SCHARR_3X3_08U, 32, gScenarioKernelData109_SCHARR_3X3_08U, 4},
{30, 4, 852, 0, 1, gScenarioBufferData110_SCHARR_3X3_08U, 32, gScenarioKernelData110_SCHARR_3X3_08U, 4},
{32, 4, 904, 0, 1, gScenarioBufferData111_SCHARR_3X3_08U, 32, gScenarioKernelData111_SCHARR_3X3_08U, 4},
{36, 4, 1008, 0, 1, gScenarioBufferData112_SCHARR_3X3_08U, 32, gScenarioKernelData112_SCHARR_3X3_08U, 4},
{40, 4, 1112, 0, 1, gScenarioBufferData113_SCHARR_3X3_08U, 32, gScenarioKernelData113_SCHARR_3X3_08U, 4},
{44, 4, 1216, 0, 1, gScenarioBufferData114_SCHARR_3X3_08U, 32, gScenarioKernelData114_SCHARR_3X3_08U, 4},
{48, 4, 1320, 0, 1, gScenarioBufferData115_SCHARR_3X3_08U, 32, gScenarioKernelData115_SCHARR_3X3_08U, 4},
{52, 4, 1424, 0, 1, gScenarioBufferData116_SCHARR_3X3_08U, 32, gScenarioKernelData116_SCHARR_3X3_08U, 4},
{56, 4, 1528, 0, 1, gScenarioBufferData117_SCHARR_3X3_08U, 32, gScenarioKernelData117_SCHARR_3X3_08U, 4},
{60, 4, 1632, 0, 1, gScenarioBufferData118_SCHARR_3X3_08U, 32, gScenarioKernelData118_SCHARR_3X3_08U, 4},
{64, 4, 1736, 0, 1, gScenarioBufferData119_SCHARR_3X3_08U, 32, gScenarioKernelData119_SCHARR_3X3_08U, 4},
{72, 4, 1944, 0, 1, gScenarioBufferData120_SCHARR_3X3_08U, 32, gScenarioKernelData120_SCHARR_3X3_08U, 4},
{80, 4, 2152, 0, 1, gScenarioBufferData121_SCHARR_3X3_08U, 32, gScenarioKernelData121_SCHARR_3X3_08U, 4},
{88, 4, 2360, 0, 1, gScenarioBufferData122_SCHARR_3X3_08U, 32, gScenarioKernelData122_SCHARR_3X3_08U, 4},
{96, 4, 2568, 0, 1, gScenarioBufferData123_SCHARR_3X3_08U, 32, gScenarioKernelData123_SCHARR_3X3_08U, 4},
{104, 4, 2776, 0, 1, gScenarioBufferData124_SCHARR_3X3_08U, 32, gScenarioKernelData124_SCHARR_3X3_08U, 4},
{112, 4, 2984, 0, 1, gScenarioBufferData125_SCHARR_3X3_08U, 32, gScenarioKernelData125_SCHARR_3X3_08U, 4},
{120, 4, 3192, 0, 1, gScenarioBufferData126_SCHARR_3X3_08U, 32, gScenarioKernelData126_SCHARR_3X3_08U, 4},
{128, 4, 3400, 0, 1, gScenarioBufferData127_SCHARR_3X3_08U, 32, gScenarioKernelData127_SCHARR_3X3_08U, 4},
{2, 5, 152, 0, 1, gScenarioBufferData128_SCHARR_3X3_08U, 32, gScenarioKernelData128_SCHARR_3X3_08U, 4},
{4, 5, 216, 0, 1, gScenarioBufferData129_SCHARR_3X3_08U, 32, gScenarioKernelData129_SCHARR_3X3_08U, 4},
{6, 5, 280, 0, 1, gScenarioBufferData130_SCHARR_3X3_08U, 32, gScenarioKernelData130_SCHARR_3X3_08U, 4},
{8, 5, 344, 0, 1, gScenarioBufferData131_SCHARR_3X3_08U, 32, gScenarioKernelData131_SCHARR_3X3_08U, 4},
{10, 5, 408, 0, 1, gScenarioBufferData132_SCHARR_3X3_08U, 32, gScenarioKernelData132_SCHARR_3X3_08U, 4},
{12, 5, 472, 0, 1, gScenarioBufferData133_SCHARR_3X3_08U, 32, gScenarioKernelData133_SCHARR_3X3_08U, 4},
{14, 5, 536, 0, 1, gScenarioBufferData134_SCHARR_3X3_08U, 32, gScenarioKernelData134_SCHARR_3X3_08U, 4},
{16, 5, 600, 0, 1, gScenarioBufferData135_SCHARR_3X3_08U, 32, gScenarioKernelData135_SCHARR_3X3_08U, 4},
{18, 5, 664, 0, 1, gScenarioBufferData136_SCHARR_3X3_08U, 32, gScenarioKernelData136_SCHARR_3X3_08U, 4},
{20, 5, 728, 0, 1, gScenarioBufferData137_SCHARR_3X3_08U, 32, gScenarioKernelData137_SCHARR_3X3_08U, 4},
{22, 5, 792, 0, 1, gScenarioBufferData138_SCHARR_3X3_08U, 32, gScenarioKernelData138_SCHARR_3X3_08U, 4},
{24, 5, 856, 0, 1, gScenarioBufferData139_SCHARR_3X3_08U, 32, gScenarioKernelData139_SCHARR_3X3_08U, 4},
{26, 5, 920, 0, 1, gScenarioBufferData140_SCHARR_3X3_08U, 32, gScenarioKernelData140_SCHARR_3X3_08U, 4},
{28, 5, 984, 0, 1, gScenarioBufferData141_SCHARR_3X3_08U, 32, gScenarioKernelData141_SCHARR_3X3_08U, 4},
{30, 5, 1048, 0, 1, gScenarioBufferData142_SCHARR_3X3_08U, 32, gScenarioKernelData142_SCHARR_3X3_08U, 4},
{32, 5, 1112, 0, 1, gScenarioBufferData143_SCHARR_3X3_08U, 32, gScenarioKernelData143_SCHARR_3X3_08U, 4},
{36, 5, 1240, 0, 1, gScenarioBufferData144_SCHARR_3X3_08U, 32, gScenarioKernelData144_SCHARR_3X3_08U, 4},
{40, 5, 1368, 0, 1, gScenarioBufferData145_SCHARR_3X3_08U, 32, gScenarioKernelData145_SCHARR_3X3_08U, 4},
{44, 5, 1496, 0, 1, gScenarioBufferData146_SCHARR_3X3_08U, 32, gScenarioKernelData146_SCHARR_3X3_08U, 4},
{48, 5, 1624, 0, 1, gScenarioBufferData147_SCHARR_3X3_08U, 32, gScenarioKernelData147_SCHARR_3X3_08U, 4},
{52, 5, 1752, 0, 1, gScenarioBufferData148_SCHARR_3X3_08U, 32, gScenarioKernelData148_SCHARR_3X3_08U, 4},
{56, 5, 1880, 0, 1, gScenarioBufferData149_SCHARR_3X3_08U, 32, gScenarioKernelData149_SCHARR_3X3_08U, 4},
{60, 5, 2008, 0, 1, gScenarioBufferData150_SCHARR_3X3_08U, 32, gScenarioKernelData150_SCHARR_3X3_08U, 4},
{64, 5, 2136, 0, 1, gScenarioBufferData151_SCHARR_3X3_08U, 32, gScenarioKernelData151_SCHARR_3X3_08U, 4},
{72, 5, 2392, 0, 1, gScenarioBufferData152_SCHARR_3X3_08U, 32, gScenarioKernelData152_SCHARR_3X3_08U, 4},
{80, 5, 2648, 0, 1, gScenarioBufferData153_SCHARR_3X3_08U, 32, gScenarioKernelData153_SCHARR_3X3_08U, 4},
{88, 5, 2904, 0, 1, gScenarioBufferData154_SCHARR_3X3_08U, 32, gScenarioKernelData154_SCHARR_3X3_08U, 4},
{96, 5, 3160, 0, 1, gScenarioBufferData155_SCHARR_3X3_08U, 32, gScenarioKernelData155_SCHARR_3X3_08U, 4},
{104, 5, 3416, 0, 1, gScenarioBufferData156_SCHARR_3X3_08U, 32, gScenarioKernelData156_SCHARR_3X3_08U, 4},
{112, 5, 3672, 0, 1, gScenarioBufferData157_SCHARR_3X3_08U, 32, gScenarioKernelData157_SCHARR_3X3_08U, 4},
{2, 6, 180, 0, 1, gScenarioBufferData158_SCHARR_3X3_08U, 32, gScenarioKernelData158_SCHARR_3X3_08U, 4},
{4, 6, 256, 0, 1, gScenarioBufferData159_SCHARR_3X3_08U, 32, gScenarioKernelData159_SCHARR_3X3_08U, 4},
{6, 6, 332, 0, 1, gScenarioBufferData160_SCHARR_3X3_08U, 32, gScenarioKernelData160_SCHARR_3X3_08U, 4},
{8, 6, 408, 0, 1, gScenarioBufferData161_SCHARR_3X3_08U, 32, gScenarioKernelData161_SCHARR_3X3_08U, 4},
{10, 6, 484, 0, 1, gScenarioBufferData162_SCHARR_3X3_08U, 32, gScenarioKernelData162_SCHARR_3X3_08U, 4},
{12, 6, 560, 0, 1, gScenarioBufferData163_SCHARR_3X3_08U, 32, gScenarioKernelData163_SCHARR_3X3_08U, 4},
{14, 6, 636, 0, 1, gScenarioBufferData164_SCHARR_3X3_08U, 32, gScenarioKernelData164_SCHARR_3X3_08U, 4},
{16, 6, 712, 0, 1, gScenarioBufferData165_SCHARR_3X3_08U, 32, gScenarioKernelData165_SCHARR_3X3_08U, 4},
{18, 6, 788, 0, 1, gScenarioBufferData166_SCHARR_3X3_08U, 32, gScenarioKernelData166_SCHARR_3X3_08U, 4},
{20, 6, 864, 0, 1, gScenarioBufferData167_SCHARR_3X3_08U, 32, gScenarioKernelData167_SCHARR_3X3_08U, 4},
{22, 6, 940, 0, 1, gScenarioBufferData168_SCHARR_3X3_08U, 32, gScenarioKernelData168_SCHARR_3X3_08U, 4},
{24, 6, 1016, 0, 1, gScenarioBufferData169_SCHARR_3X3_08U, 32, gScenarioKernelData169_SCHARR_3X3_08U, 4},
{26, 6, 1092, 0, 1, gScenarioBufferData170_SCHARR_3X3_08U, 32, gScenarioKernelData170_SCHARR_3X3_08U, 4},
{28, 6, 1168, 0, 1, gScenarioBufferData171_SCHARR_3X3_08U, 32, gScenarioKernelData171_SCHARR_3X3_08U, 4},
{30, 6, 1244, 0, 1, gScenarioBufferData172_SCHARR_3X3_08U, 32, gScenarioKernelData172_SCHARR_3X3_08U, 4},
{32, 6, 1320, 0, 1, gScenarioBufferData173_SCHARR_3X3_08U, 32, gScenarioKernelData173_SCHARR_3X3_08U, 4},
{36, 6, 1472, 0, 1, gScenarioBufferData174_SCHARR_3X3_08U, 32, gScenarioKernelData174_SCHARR_3X3_08U, 4},
{40, 6, 1624, 0, 1, gScenarioBufferData175_SCHARR_3X3_08U, 32, gScenarioKernelData175_SCHARR_3X3_08U, 4},
{44, 6, 1776, 0, 1, gScenarioBufferData176_SCHARR_3X3_08U, 32, gScenarioKernelData176_SCHARR_3X3_08U, 4},
{48, 6, 1928, 0, 1, gScenarioBufferData177_SCHARR_3X3_08U, 32, gScenarioKernelData177_SCHARR_3X3_08U, 4},
{52, 6, 2080, 0, 1, gScenarioBufferData178_SCHARR_3X3_08U, 32, gScenarioKernelData178_SCHARR_3X3_08U, 4},
{56, 6, 2232, 0, 1, gScenarioBufferData179_SCHARR_3X3_08U, 32, gScenarioKernelData179_SCHARR_3X3_08U, 4},
{60, 6, 2384, 0, 1, gScenarioBufferData180_SCHARR_3X3_08U, 32, gScenarioKernelData180_SCHARR_3X3_08U, 4},
{64, 6, 2536, 0, 1, gScenarioBufferData181_SCHARR_3X3_08U, 32, gScenarioKernelData181_SCHARR_3X3_08U, 4},
{72, 6, 2840, 0, 1, gScenarioBufferData182_SCHARR_3X3_08U, 32, gScenarioKernelData182_SCHARR_3X3_08U, 4},
{80, 6, 3144, 0, 1, gScenarioBufferData183_SCHARR_3X3_08U, 32, gScenarioKernelData183_SCHARR_3X3_08U, 4},
{88, 6, 3448, 0, 1, gScenarioBufferData184_SCHARR_3X3_08U, 32, gScenarioKernelData184_SCHARR_3X3_08U, 4},
{96, 6, 3752, 0, 1, gScenarioBufferData185_SCHARR_3X3_08U, 32, gScenarioKernelData185_SCHARR_3X3_08U, 4},
{2, 8, 236, 0, 1, gScenarioBufferData186_SCHARR_3X3_08U, 32, gScenarioKernelData186_SCHARR_3X3_08U, 4},
{4, 8, 336, 0, 1, gScenarioBufferData187_SCHARR_3X3_08U, 32, gScenarioKernelData187_SCHARR_3X3_08U, 4},
{6, 8, 436, 0, 1, gScenarioBufferData188_SCHARR_3X3_08U, 32, gScenarioKernelData188_SCHARR_3X3_08U, 4},
{8, 8, 536, 0, 1, gScenarioBufferData189_SCHARR_3X3_08U, 32, gScenarioKernelData189_SCHARR_3X3_08U, 4},
{10, 8, 636, 0, 1, gScenarioBufferData190_SCHARR_3X3_08U, 32, gScenarioKernelData190_SCHARR_3X3_08U, 4},
{12, 8, 736, 0, 1, gScenarioBufferData191_SCHARR_3X3_08U, 32, gScenarioKernelData191_SCHARR_3X3_08U, 4},
{14, 8, 836, 0, 1, gScenarioBufferData192_SCHARR_3X3_08U, 32, gScenarioKernelData192_SCHARR_3X3_08U, 4},
{16, 8, 936, 0, 1, gScenarioBufferData193_SCHARR_3X3_08U, 32, gScenarioKernelData193_SCHARR_3X3_08U, 4},
{18, 8, 1036, 0, 1, gScenarioBufferData194_SCHARR_3X3_08U, 32, gScenarioKernelData194_SCHARR_3X3_08U, 4},
{20, 8, 1136, 0, 1, gScenarioBufferData195_SCHARR_3X3_08U, 32, gScenarioKernelData195_SCHARR_3X3_08U, 4},
{22, 8, 1236, 0, 1, gScenarioBufferData196_SCHARR_3X3_08U, 32, gScenarioKernelData196_SCHARR_3X3_08U, 4},
{24, 8, 1336, 0, 1, gScenarioBufferData197_SCHARR_3X3_08U, 32, gScenarioKernelData197_SCHARR_3X3_08U, 4},
{26, 8, 1436, 0, 1, gScenarioBufferData198_SCHARR_3X3_08U, 32, gScenarioKernelData198_SCHARR_3X3_08U, 4},
{28, 8, 1536, 0, 1, gScenarioBufferData199_SCHARR_3X3_08U, 32, gScenarioKernelData199_SCHARR_3X3_08U, 4},
{30, 8, 1636, 0, 1, gScenarioBufferData200_SCHARR_3X3_08U, 32, gScenarioKernelData200_SCHARR_3X3_08U, 4},
{32, 8, 1736, 0, 1, gScenarioBufferData201_SCHARR_3X3_08U, 32, gScenarioKernelData201_SCHARR_3X3_08U, 4},
{36, 8, 1936, 0, 1, gScenarioBufferData202_SCHARR_3X3_08U, 32, gScenarioKernelData202_SCHARR_3X3_08U, 4},
{40, 8, 2136, 0, 1, gScenarioBufferData203_SCHARR_3X3_08U, 32, gScenarioKernelData203_SCHARR_3X3_08U, 4},
{44, 8, 2336, 0, 1, gScenarioBufferData204_SCHARR_3X3_08U, 32, gScenarioKernelData204_SCHARR_3X3_08U, 4},
{48, 8, 2536, 0, 1, gScenarioBufferData205_SCHARR_3X3_08U, 32, gScenarioKernelData205_SCHARR_3X3_08U, 4},
{52, 8, 2736, 0, 1, gScenarioBufferData206_SCHARR_3X3_08U, 32, gScenarioKernelData206_SCHARR_3X3_08U, 4},
{56, 8, 2936, 0, 1, gScenarioBufferData207_SCHARR_3X3_08U, 32, gScenarioKernelData207_SCHARR_3X3_08U, 4},
{60, 8, 3136, 0, 1, gScenarioBufferData208_SCHARR_3X3_08U, 32, gScenarioKernelData208_SCHARR_3X3_08U, 4},
{64, 8, 3336, 0, 1, gScenarioBufferData209_SCHARR_3X3_08U, 32, gScenarioKernelData209_SCHARR_3X3_08U, 4},
{72, 8, 3736, 0, 1, gScenarioBufferData210_SCHARR_3X3_08U, 32, gScenarioKernelData210_SCHARR_3X3_08U, 4},
{2, 10, 292, 0, 1, gScenarioBufferData211_SCHARR_3X3_08U, 32, gScenarioKernelData211_SCHARR_3X3_08U, 4},
{4, 10, 416, 0, 1, gScenarioBufferData212_SCHARR_3X3_08U, 32, gScenarioKernelData212_SCHARR_3X3_08U, 4},
{6, 10, 540, 0, 1, gScenarioBufferData213_SCHARR_3X3_08U, 32, gScenarioKernelData213_SCHARR_3X3_08U, 4},
{8, 10, 664, 0, 1, gScenarioBufferData214_SCHARR_3X3_08U, 32, gScenarioKernelData214_SCHARR_3X3_08U, 4},
{10, 10, 788, 0, 1, gScenarioBufferData215_SCHARR_3X3_08U, 32, gScenarioKernelData215_SCHARR_3X3_08U, 4},
{12, 10, 912, 0, 1, gScenarioBufferData216_SCHARR_3X3_08U, 32, gScenarioKernelData216_SCHARR_3X3_08U, 4},
{14, 10, 1036, 0, 1, gScenarioBufferData217_SCHARR_3X3_08U, 32, gScenarioKernelData217_SCHARR_3X3_08U, 4},
{16, 10, 1160, 0, 1, gScenarioBufferData218_SCHARR_3X3_08U, 32, gScenarioKernelData218_SCHARR_3X3_08U, 4},
{18, 10, 1284, 0, 1, gScenarioBufferData219_SCHARR_3X3_08U, 32, gScenarioKernelData219_SCHARR_3X3_08U, 4},
{20, 10, 1408, 0, 1, gScenarioBufferData220_SCHARR_3X3_08U, 32, gScenarioKernelData220_SCHARR_3X3_08U, 4},
{22, 10, 1532, 0, 1, gScenarioBufferData221_SCHARR_3X3_08U, 32, gScenarioKernelData221_SCHARR_3X3_08U, 4},
{24, 10, 1656, 0, 1, gScenarioBufferData222_SCHARR_3X3_08U, 32, gScenarioKernelData222_SCHARR_3X3_08U, 4},
{26, 10, 1780, 0, 1, gScenarioBufferData223_SCHARR_3X3_08U, 32, gScenarioKernelData223_SCHARR_3X3_08U, 4},
{28, 10, 1904, 0, 1, gScenarioBufferData224_SCHARR_3X3_08U, 32, gScenarioKernelData224_SCHARR_3X3_08U, 4},
{30, 10, 2028, 0, 1, gScenarioBufferData225_SCHARR_3X3_08U, 32, gScenarioKernelData225_SCHARR_3X3_08U, 4},
{32, 10, 2152, 0, 1, gScenarioBufferData226_SCHARR_3X3_08U, 32, gScenarioKernelData226_SCHARR_3X3_08U, 4},
{36, 10, 2400, 0, 1, gScenarioBufferData227_SCHARR_3X3_08U, 32, gScenarioKernelData227_SCHARR_3X3_08U, 4},
{40, 10, 2648, 0, 1, gScenarioBufferData228_SCHARR_3X3_08U, 32, gScenarioKernelData228_SCHARR_3X3_08U, 4},
{44, 10, 2896, 0, 1, gScenarioBufferData229_SCHARR_3X3_08U, 32, gScenarioKernelData229_SCHARR_3X3_08U, 4},
{48, 10, 3144, 0, 1, gScenarioBufferData230_SCHARR_3X3_08U, 32, gScenarioKernelData230_SCHARR_3X3_08U, 4},
{52, 10, 3392, 0, 1, gScenarioBufferData231_SCHARR_3X3_08U, 32, gScenarioKernelData231_SCHARR_3X3_08U, 4},
{56, 10, 3640, 0, 1, gScenarioBufferData232_SCHARR_3X3_08U, 32, gScenarioKernelData232_SCHARR_3X3_08U, 4},
{2, 12, 348, 0, 1, gScenarioBufferData233_SCHARR_3X3_08U, 32, gScenarioKernelData233_SCHARR_3X3_08U, 4},
{4, 12, 496, 0, 1, gScenarioBufferData234_SCHARR_3X3_08U, 32, gScenarioKernelData234_SCHARR_3X3_08U, 4},
{6, 12, 644, 0, 1, gScenarioBufferData235_SCHARR_3X3_08U, 32, gScenarioKernelData235_SCHARR_3X3_08U, 4},
{8, 12, 792, 0, 1, gScenarioBufferData236_SCHARR_3X3_08U, 32, gScenarioKernelData236_SCHARR_3X3_08U, 4},
{10, 12, 940, 0, 1, gScenarioBufferData237_SCHARR_3X3_08U, 32, gScenarioKernelData237_SCHARR_3X3_08U, 4},
{12, 12, 1088, 0, 1, gScenarioBufferData238_SCHARR_3X3_08U, 32, gScenarioKernelData238_SCHARR_3X3_08U, 4},
{14, 12, 1236, 0, 1, gScenarioBufferData239_SCHARR_3X3_08U, 32, gScenarioKernelData239_SCHARR_3X3_08U, 4},
{16, 12, 1384, 0, 1, gScenarioBufferData240_SCHARR_3X3_08U, 32, gScenarioKernelData240_SCHARR_3X3_08U, 4},
{18, 12, 1532, 0, 1, gScenarioBufferData241_SCHARR_3X3_08U, 32, gScenarioKernelData241_SCHARR_3X3_08U, 4},
{20, 12, 1680, 0, 1, gScenarioBufferData242_SCHARR_3X3_08U, 32, gScenarioKernelData242_SCHARR_3X3_08U, 4},
{22, 12, 1828, 0, 1, gScenarioBufferData243_SCHARR_3X3_08U, 32, gScenarioKernelData243_SCHARR_3X3_08U, 4},
{24, 12, 1976, 0, 1, gScenarioBufferData244_SCHARR_3X3_08U, 32, gScenarioKernelData244_SCHARR_3X3_08U, 4},
{26, 12, 2124, 0, 1, gScenarioBufferData245_SCHARR_3X3_08U, 32, gScenarioKernelData245_SCHARR_3X3_08U, 4},
{28, 12, 2272, 0, 1, gScenarioBufferData246_SCHARR_3X3_08U, 32, gScenarioKernelData246_SCHARR_3X3_08U, 4},
{30, 12, 2420, 0, 1, gScenarioBufferData247_SCHARR_3X3_08U, 32, gScenarioKernelData247_SCHARR_3X3_08U, 4},
{32, 12, 2568, 0, 1, gScenarioBufferData248_SCHARR_3X3_08U, 32, gScenarioKernelData248_SCHARR_3X3_08U, 4},
{36, 12, 2864, 0, 1, gScenarioBufferData249_SCHARR_3X3_08U, 32, gScenarioKernelData249_SCHARR_3X3_08U, 4},
{40, 12, 3160, 0, 1, gScenarioBufferData250_SCHARR_3X3_08U, 32, gScenarioKernelData250_SCHARR_3X3_08U, 4},
{44, 12, 3456, 0, 1, gScenarioBufferData251_SCHARR_3X3_08U, 32, gScenarioKernelData251_SCHARR_3X3_08U, 4},
{48, 12, 3752, 0, 1, gScenarioBufferData252_SCHARR_3X3_08U, 32, gScenarioKernelData252_SCHARR_3X3_08U, 4},
{2, 14, 404, 0, 1, gScenarioBufferData253_SCHARR_3X3_08U, 32, gScenarioKernelData253_SCHARR_3X3_08U, 4},
{4, 14, 576, 0, 1, gScenarioBufferData254_SCHARR_3X3_08U, 32, gScenarioKernelData254_SCHARR_3X3_08U, 4},
{6, 14, 748, 0, 1, gScenarioBufferData255_SCHARR_3X3_08U, 32, gScenarioKernelData255_SCHARR_3X3_08U, 4},
{8, 14, 920, 0, 1, gScenarioBufferData256_SCHARR_3X3_08U, 32, gScenarioKernelData256_SCHARR_3X3_08U, 4},
{10, 14, 1092, 0, 1, gScenarioBufferData257_SCHARR_3X3_08U, 32, gScenarioKernelData257_SCHARR_3X3_08U, 4},
{12, 14, 1264, 0, 1, gScenarioBufferData258_SCHARR_3X3_08U, 32, gScenarioKernelData258_SCHARR_3X3_08U, 4},
{14, 14, 1436, 0, 1, gScenarioBufferData259_SCHARR_3X3_08U, 32, gScenarioKernelData259_SCHARR_3X3_08U, 4},
{16, 14, 1608, 0, 1, gScenarioBufferData260_SCHARR_3X3_08U, 32, gScenarioKernelData260_SCHARR_3X3_08U, 4},
{18, 14, 1780, 0, 1, gScenarioBufferData261_SCHARR_3X3_08U, 32, gScenarioKernelData261_SCHARR_3X3_08U, 4},
{20, 14, 1952, 0, 1, gScenarioBufferData262_SCHARR_3X3_08U, 32, gScenarioKernelData262_SCHARR_3X3_08U, 4},
{22, 14, 2124, 0, 1, gScenarioBufferData263_SCHARR_3X3_08U, 32, gScenarioKernelData263_SCHARR_3X3_08U, 4},
{24, 14, 2296, 0, 1, gScenarioBufferData264_SCHARR_3X3_08U, 32, gScenarioKernelData264_SCHARR_3X3_08U, 4},
{26, 14, 2468, 0, 1, gScenarioBufferData265_SCHARR_3X3_08U, 32, gScenarioKernelData265_SCHARR_3X3_08U, 4},
{28, 14, 2640, 0, 1, gScenarioBufferData266_SCHARR_3X3_08U, 32, gScenarioKernelData266_SCHARR_3X3_08U, 4},
{30, 14, 2812, 0, 1, gScenarioBufferData267_SCHARR_3X3_08U, 32, gScenarioKernelData267_SCHARR_3X3_08U, 4},
{32, 14, 2984, 0, 1, gScenarioBufferData268_SCHARR_3X3_08U, 32, gScenarioKernelData268_SCHARR_3X3_08U, 4},
{36, 14, 3328, 0, 1, gScenarioBufferData269_SCHARR_3X3_08U, 32, gScenarioKernelData269_SCHARR_3X3_08U, 4},
{40, 14, 3672, 0, 1, gScenarioBufferData270_SCHARR_3X3_08U, 32, gScenarioKernelData270_SCHARR_3X3_08U, 4},
{2, 16, 460, 0, 1, gScenarioBufferData271_SCHARR_3X3_08U, 32, gScenarioKernelData271_SCHARR_3X3_08U, 4},
{4, 16, 656, 0, 1, gScenarioBufferData272_SCHARR_3X3_08U, 32, gScenarioKernelData272_SCHARR_3X3_08U, 4},
{6, 16, 852, 0, 1, gScenarioBufferData273_SCHARR_3X3_08U, 32, gScenarioKernelData273_SCHARR_3X3_08U, 4},
{8, 16, 1048, 0, 1, gScenarioBufferData274_SCHARR_3X3_08U, 32, gScenarioKernelData274_SCHARR_3X3_08U, 4},
{10, 16, 1244, 0, 1, gScenarioBufferData275_SCHARR_3X3_08U, 32, gScenarioKernelData275_SCHARR_3X3_08U, 4},
{12, 16, 1440, 0, 1, gScenarioBufferData276_SCHARR_3X3_08U, 32, gScenarioKernelData276_SCHARR_3X3_08U, 4},
{14, 16, 1636, 0, 1, gScenarioBufferData277_SCHARR_3X3_08U, 32, gScenarioKernelData277_SCHARR_3X3_08U, 4},
{16, 16, 1832, 0, 1, gScenarioBufferData278_SCHARR_3X3_08U, 32, gScenarioKernelData278_SCHARR_3X3_08U, 4},
{18, 16, 2028, 0, 1, gScenarioBufferData279_SCHARR_3X3_08U, 32, gScenarioKernelData279_SCHARR_3X3_08U, 4},
{20, 16, 2224, 0, 1, gScenarioBufferData280_SCHARR_3X3_08U, 32, gScenarioKernelData280_SCHARR_3X3_08U, 4},
{22, 16, 2420, 0, 1, gScenarioBufferData281_SCHARR_3X3_08U, 32, gScenarioKernelData281_SCHARR_3X3_08U, 4},
{24, 16, 2616, 0, 1, gScenarioBufferData282_SCHARR_3X3_08U, 32, gScenarioKernelData282_SCHARR_3X3_08U, 4},
{26, 16, 2812, 0, 1, gScenarioBufferData283_SCHARR_3X3_08U, 32, gScenarioKernelData283_SCHARR_3X3_08U, 4},
{28, 16, 3008, 0, 1, gScenarioBufferData284_SCHARR_3X3_08U, 32, gScenarioKernelData284_SCHARR_3X3_08U, 4},
{30, 16, 3204, 0, 1, gScenarioBufferData285_SCHARR_3X3_08U, 32, gScenarioKernelData285_SCHARR_3X3_08U, 4},
{32, 16, 3400, 0, 1, gScenarioBufferData286_SCHARR_3X3_08U, 32, gScenarioKernelData286_SCHARR_3X3_08U, 4},
{36, 16, 3792, 0, 1, gScenarioBufferData287_SCHARR_3X3_08U, 32, gScenarioKernelData287_SCHARR_3X3_08U, 4},
{2, 18, 516, 0, 1, gScenarioBufferData288_SCHARR_3X3_08U, 32, gScenarioKernelData288_SCHARR_3X3_08U, 4},
{4, 18, 736, 0, 1, gScenarioBufferData289_SCHARR_3X3_08U, 32, gScenarioKernelData289_SCHARR_3X3_08U, 4},
{6, 18, 956, 0, 1, gScenarioBufferData290_SCHARR_3X3_08U, 32, gScenarioKernelData290_SCHARR_3X3_08U, 4},
{8, 18, 1176, 0, 1, gScenarioBufferData291_SCHARR_3X3_08U, 32, gScenarioKernelData291_SCHARR_3X3_08U, 4},
{10, 18, 1396, 0, 1, gScenarioBufferData292_SCHARR_3X3_08U, 32, gScenarioKernelData292_SCHARR_3X3_08U, 4},
{12, 18, 1616, 0, 1, gScenarioBufferData293_SCHARR_3X3_08U, 32, gScenarioKernelData293_SCHARR_3X3_08U, 4},
{14, 18, 1836, 0, 1, gScenarioBufferData294_SCHARR_3X3_08U, 32, gScenarioKernelData294_SCHARR_3X3_08U, 4},
{16, 18, 2056, 0, 1, gScenarioBufferData295_SCHARR_3X3_08U, 32, gScenarioKernelData295_SCHARR_3X3_08U, 4},
{18, 18, 2276, 0, 1, gScenarioBufferData296_SCHARR_3X3_08U, 32, gScenarioKernelData296_SCHARR_3X3_08U, 4},
{20, 18, 2496, 0, 1, gScenarioBufferData297_SCHARR_3X3_08U, 32, gScenarioKernelData297_SCHARR_3X3_08U, 4},
{22, 18, 2716, 0, 1, gScenarioBufferData298_SCHARR_3X3_08U, 32, gScenarioKernelData298_SCHARR_3X3_08U, 4},
{24, 18, 2936, 0, 1, gScenarioBufferData299_SCHARR_3X3_08U, 32, gScenarioKernelData299_SCHARR_3X3_08U, 4},
{26, 18, 3156, 0, 1, gScenarioBufferData300_SCHARR_3X3_08U, 32, gScenarioKernelData300_SCHARR_3X3_08U, 4},
{28, 18, 3376, 0, 1, gScenarioBufferData301_SCHARR_3X3_08U, 32, gScenarioKernelData301_SCHARR_3X3_08U, 4},
{30, 18, 3596, 0, 1, gScenarioBufferData302_SCHARR_3X3_08U, 32, gScenarioKernelData302_SCHARR_3X3_08U, 4},
{32, 18, 3816, 0, 1, gScenarioBufferData303_SCHARR_3X3_08U, 32, gScenarioKernelData303_SCHARR_3X3_08U, 4},
{2, 20, 572, 0, 1, gScenarioBufferData304_SCHARR_3X3_08U, 32, gScenarioKernelData304_SCHARR_3X3_08U, 4},
{4, 20, 816, 0, 1, gScenarioBufferData305_SCHARR_3X3_08U, 32, gScenarioKernelData305_SCHARR_3X3_08U, 4},
{6, 20, 1060, 0, 1, gScenarioBufferData306_SCHARR_3X3_08U, 32, gScenarioKernelData306_SCHARR_3X3_08U, 4},
{8, 20, 1304, 0, 1, gScenarioBufferData307_SCHARR_3X3_08U, 32, gScenarioKernelData307_SCHARR_3X3_08U, 4},
{10, 20, 1548, 0, 1, gScenarioBufferData308_SCHARR_3X3_08U, 32, gScenarioKernelData308_SCHARR_3X3_08U, 4},
{12, 20, 1792, 0, 1, gScenarioBufferData309_SCHARR_3X3_08U, 32, gScenarioKernelData309_SCHARR_3X3_08U, 4},
{14, 20, 2036, 0, 1, gScenarioBufferData310_SCHARR_3X3_08U, 32, gScenarioKernelData310_SCHARR_3X3_08U, 4},
{16, 20, 2280, 0, 1, gScenarioBufferData311_SCHARR_3X3_08U, 32, gScenarioKernelData311_SCHARR_3X3_08U, 4},
{18, 20, 2524, 0, 1, gScenarioBufferData312_SCHARR_3X3_08U, 32, gScenarioKernelData312_SCHARR_3X3_08U, 4},
{20, 20, 2768, 0, 1, gScenarioBufferData313_SCHARR_3X3_08U, 32, gScenarioKernelData313_SCHARR_3X3_08U, 4},
{22, 20, 3012, 0, 1, gScenarioBufferData314_SCHARR_3X3_08U, 32, gScenarioKernelData314_SCHARR_3X3_08U, 4},
{24, 20, 3256, 0, 1, gScenarioBufferData315_SCHARR_3X3_08U, 32, gScenarioKernelData315_SCHARR_3X3_08U, 4},
{26, 20, 3500, 0, 1, gScenarioBufferData316_SCHARR_3X3_08U, 32, gScenarioKernelData316_SCHARR_3X3_08U, 4},
{28, 20, 3744, 0, 1, gScenarioBufferData317_SCHARR_3X3_08U, 32, gScenarioKernelData317_SCHARR_3X3_08U, 4},
{2, 22, 628, 0, 1, gScenarioBufferData318_SCHARR_3X3_08U, 32, gScenarioKernelData318_SCHARR_3X3_08U, 4},
{4, 22, 896, 0, 1, gScenarioBufferData319_SCHARR_3X3_08U, 32, gScenarioKernelData319_SCHARR_3X3_08U, 4},
{6, 22, 1164, 0, 1, gScenarioBufferData320_SCHARR_3X3_08U, 32, gScenarioKernelData320_SCHARR_3X3_08U, 4},
{8, 22, 1432, 0, 1, gScenarioBufferData321_SCHARR_3X3_08U, 32, gScenarioKernelData321_SCHARR_3X3_08U, 4},
{10, 22, 1700, 0, 1, gScenarioBufferData322_SCHARR_3X3_08U, 32, gScenarioKernelData322_SCHARR_3X3_08U, 4},
{12, 22, 1968, 0, 1, gScenarioBufferData323_SCHARR_3X3_08U, 32, gScenarioKernelData323_SCHARR_3X3_08U, 4},
{14, 22, 2236, 0, 1, gScenarioBufferData324_SCHARR_3X3_08U, 32, gScenarioKernelData324_SCHARR_3X3_08U, 4},
{16, 22, 2504, 0, 1, gScenarioBufferData325_SCHARR_3X3_08U, 32, gScenarioKernelData325_SCHARR_3X3_08U, 4},
{18, 22, 2772, 0, 1, gScenarioBufferData326_SCHARR_3X3_08U, 32, gScenarioKernelData326_SCHARR_3X3_08U, 4},
{20, 22, 3040, 0, 1, gScenarioBufferData327_SCHARR_3X3_08U, 32, gScenarioKernelData327_SCHARR_3X3_08U, 4},
{22, 22, 3308, 0, 1, gScenarioBufferData328_SCHARR_3X3_08U, 32, gScenarioKernelData328_SCHARR_3X3_08U, 4},
{24, 22, 3576, 0, 1, gScenarioBufferData329_SCHARR_3X3_08U, 32, gScenarioKernelData329_SCHARR_3X3_08U, 4},
{2, 24, 684, 0, 1, gScenarioBufferData330_SCHARR_3X3_08U, 32, gScenarioKernelData330_SCHARR_3X3_08U, 4},
{4, 24, 976, 0, 1, gScenarioBufferData331_SCHARR_3X3_08U, 32, gScenarioKernelData331_SCHARR_3X3_08U, 4},
{6, 24, 1268, 0, 1, gScenarioBufferData332_SCHARR_3X3_08U, 32, gScenarioKernelData332_SCHARR_3X3_08U, 4},
{8, 24, 1560, 0, 1, gScenarioBufferData333_SCHARR_3X3_08U, 32, gScenarioKernelData333_SCHARR_3X3_08U, 4},
{10, 24, 1852, 0, 1, gScenarioBufferData334_SCHARR_3X3_08U, 32, gScenarioKernelData334_SCHARR_3X3_08U, 4},
{12, 24, 2144, 0, 1, gScenarioBufferData335_SCHARR_3X3_08U, 32, gScenarioKernelData335_SCHARR_3X3_08U, 4},
{14, 24, 2436, 0, 1, gScenarioBufferData336_SCHARR_3X3_08U, 32, gScenarioKernelData336_SCHARR_3X3_08U, 4},
{16, 24, 2728, 0, 1, gScenarioBufferData337_SCHARR_3X3_08U, 32, gScenarioKernelData337_SCHARR_3X3_08U, 4},
{18, 24, 3020, 0, 1, gScenarioBufferData338_SCHARR_3X3_08U, 32, gScenarioKernelData338_SCHARR_3X3_08U, 4},
{20, 24, 3312, 0, 1, gScenarioBufferData339_SCHARR_3X3_08U, 32, gScenarioKernelData339_SCHARR_3X3_08U, 4},
{22, 24, 3604, 0, 1, gScenarioBufferData340_SCHARR_3X3_08U, 32, gScenarioKernelData340_SCHARR_3X3_08U, 4},
{2, 26, 740, 0, 1, gScenarioBufferData341_SCHARR_3X3_08U, 32, gScenarioKernelData341_SCHARR_3X3_08U, 4},
{4, 26, 1056, 0, 1, gScenarioBufferData342_SCHARR_3X3_08U, 32, gScenarioKernelData342_SCHARR_3X3_08U, 4},
{6, 26, 1372, 0, 1, gScenarioBufferData343_SCHARR_3X3_08U, 32, gScenarioKernelData343_SCHARR_3X3_08U, 4},
{8, 26, 1688, 0, 1, gScenarioBufferData344_SCHARR_3X3_08U, 32, gScenarioKernelData344_SCHARR_3X3_08U, 4},
{10, 26, 2004, 0, 1, gScenarioBufferData345_SCHARR_3X3_08U, 32, gScenarioKernelData345_SCHARR_3X3_08U, 4},
{12, 26, 2320, 0, 1, gScenarioBufferData346_SCHARR_3X3_08U, 32, gScenarioKernelData346_SCHARR_3X3_08U, 4},
{14, 26, 2636, 0, 1, gScenarioBufferData347_SCHARR_3X3_08U, 32, gScenarioKernelData347_SCHARR_3X3_08U, 4},
{16, 26, 2952, 0, 1, gScenarioBufferData348_SCHARR_3X3_08U, 32, gScenarioKernelData348_SCHARR_3X3_08U, 4},
{18, 26, 3268, 0, 1, gScenarioBufferData349_SCHARR_3X3_08U, 32, gScenarioKernelData349_SCHARR_3X3_08U, 4},
{20, 26, 3584, 0, 1, gScenarioBufferData350_SCHARR_3X3_08U, 32, gScenarioKernelData350_SCHARR_3X3_08U, 4},
{2, 28, 796, 0, 1, gScenarioBufferData351_SCHARR_3X3_08U, 32, gScenarioKernelData351_SCHARR_3X3_08U, 4},
{4, 28, 1136, 0, 1, gScenarioBufferData352_SCHARR_3X3_08U, 32, gScenarioKernelData352_SCHARR_3X3_08U, 4},
{6, 28, 1476, 0, 1, gScenarioBufferData353_SCHARR_3X3_08U, 32, gScenarioKernelData353_SCHARR_3X3_08U, 4},
{8, 28, 1816, 0, 1, gScenarioBufferData354_SCHARR_3X3_08U, 32, gScenarioKernelData354_SCHARR_3X3_08U, 4},
{10, 28, 2156, 0, 1, gScenarioBufferData355_SCHARR_3X3_08U, 32, gScenarioKernelData355_SCHARR_3X3_08U, 4},
{12, 28, 2496, 0, 1, gScenarioBufferData356_SCHARR_3X3_08U, 32, gScenarioKernelData356_SCHARR_3X3_08U, 4},
{14, 28, 2836, 0, 1, gScenarioBufferData357_SCHARR_3X3_08U, 32, gScenarioKernelData357_SCHARR_3X3_08U, 4},
{16, 28, 3176, 0, 1, gScenarioBufferData358_SCHARR_3X3_08U, 32, gScenarioKernelData358_SCHARR_3X3_08U, 4},
{18, 28, 3516, 0, 1, gScenarioBufferData359_SCHARR_3X3_08U, 32, gScenarioKernelData359_SCHARR_3X3_08U, 4},
{2, 30, 852, 0, 1, gScenarioBufferData360_SCHARR_3X3_08U, 32, gScenarioKernelData360_SCHARR_3X3_08U, 4},
{4, 30, 1216, 0, 1, gScenarioBufferData361_SCHARR_3X3_08U, 32, gScenarioKernelData361_SCHARR_3X3_08U, 4},
{6, 30, 1580, 0, 1, gScenarioBufferData362_SCHARR_3X3_08U, 32, gScenarioKernelData362_SCHARR_3X3_08U, 4},
{8, 30, 1944, 0, 1, gScenarioBufferData363_SCHARR_3X3_08U, 32, gScenarioKernelData363_SCHARR_3X3_08U, 4},
{10, 30, 2308, 0, 1, gScenarioBufferData364_SCHARR_3X3_08U, 32, gScenarioKernelData364_SCHARR_3X3_08U, 4},
{12, 30, 2672, 0, 1, gScenarioBufferData365_SCHARR_3X3_08U, 32, gScenarioKernelData365_SCHARR_3X3_08U, 4},
{14, 30, 3036, 0, 1, gScenarioBufferData366_SCHARR_3X3_08U, 32, gScenarioKernelData366_SCHARR_3X3_08U, 4},
{16, 30, 3400, 0, 1, gScenarioBufferData367_SCHARR_3X3_08U, 32, gScenarioKernelData367_SCHARR_3X3_08U, 4},
{18, 30, 3764, 0, 1, gScenarioBufferData368_SCHARR_3X3_08U, 32, gScenarioKernelData368_SCHARR_3X3_08U, 4},
{2, 32, 908, 0, 1, gScenarioBufferData369_SCHARR_3X3_08U, 32, gScenarioKernelData369_SCHARR_3X3_08U, 4},
{4, 32, 1296, 0, 1, gScenarioBufferData370_SCHARR_3X3_08U, 32, gScenarioKernelData370_SCHARR_3X3_08U, 4},
{6, 32, 1684, 0, 1, gScenarioBufferData371_SCHARR_3X3_08U, 32, gScenarioKernelData371_SCHARR_3X3_08U, 4},
{8, 32, 2072, 0, 1, gScenarioBufferData372_SCHARR_3X3_08U, 32, gScenarioKernelData372_SCHARR_3X3_08U, 4},
{10, 32, 2460, 0, 1, gScenarioBufferData373_SCHARR_3X3_08U, 32, gScenarioKernelData373_SCHARR_3X3_08U, 4},
{12, 32, 2848, 0, 1, gScenarioBufferData374_SCHARR_3X3_08U, 32, gScenarioKernelData374_SCHARR_3X3_08U, 4},
{14, 32, 3236, 0, 1, gScenarioBufferData375_SCHARR_3X3_08U, 32, gScenarioKernelData375_SCHARR_3X3_08U, 4},
{16, 32, 3624, 0, 1, gScenarioBufferData376_SCHARR_3X3_08U, 32, gScenarioKernelData376_SCHARR_3X3_08U, 4}
};

static acf_scenario_list gScenarioList_SCHARR_3X3_08U = {
377, //number of scenarios
gScenarioArray_SCHARR_3X3_08U};
//**************************************************************

class SCHARR_3X3_08U : public ACF_Process_APU
{

public:
   SCHARR_3X3_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("SCHARR_3X3_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("SCHARR_3X3_08U",
                                        SCHARR_3X3_08U_LOAD_SEGMENTS,
                                        SCHARR_3X3_08U_LOAD_PMEM, SCHARR_3X3_08U_LOAD_PMEM_SIZE,
                                        SCHARR_3X3_08U_LOAD_DMEM, SCHARR_3X3_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(SCHARR_3X3_08U_APEX_LOG_BUFFER, SCHARR_3X3_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_SCHARR_3X3_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_SCHARR_3X3_08U

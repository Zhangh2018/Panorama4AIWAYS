#ifndef _ACF_PROCESS_APU_MEDIAN_5X5_08U
#define _ACF_PROCESS_APU_MEDIAN_5X5_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MEDIAN_5X5_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MEDIAN_5X5_08U[] = {{2, 2, 4, 0, {2, 2, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MEDIAN_5X5_08U[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MEDIAN_5X5_08U[] = {{6, 2, 4, 0, {2, 2, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MEDIAN_5X5_08U[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MEDIAN_5X5_08U[] = {{10, 2, 4, 0, {2, 2, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MEDIAN_5X5_08U[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MEDIAN_5X5_08U[] = {{14, 2, 4, 0, {2, 2, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MEDIAN_5X5_08U[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MEDIAN_5X5_08U[] = {{18, 2, 4, 0, {2, 2, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MEDIAN_5X5_08U[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MEDIAN_5X5_08U[] = {{22, 2, 4, 0, {2, 2, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MEDIAN_5X5_08U[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MEDIAN_5X5_08U[] = {{26, 2, 4, 0, {2, 2, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MEDIAN_5X5_08U[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MEDIAN_5X5_08U[] = {{30, 2, 4, 0, {2, 2, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MEDIAN_5X5_08U[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MEDIAN_5X5_08U[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MEDIAN_5X5_08U[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MEDIAN_5X5_08U[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MEDIAN_5X5_08U[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MEDIAN_5X5_08U[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MEDIAN_5X5_08U[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MEDIAN_5X5_08U[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MEDIAN_5X5_08U[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MEDIAN_5X5_08U[] = {{72, 2, 4, 0, {2, 2, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MEDIAN_5X5_08U[] = {{80, 2, 4, 0, {2, 2, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MEDIAN_5X5_08U[] = {{88, 2, 4, 0, {2, 2, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MEDIAN_5X5_08U[] = {{96, 2, 4, 0, {2, 2, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MEDIAN_5X5_08U[] = {{104, 2, 4, 0, {2, 2, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MEDIAN_5X5_08U[] = {{112, 2, 4, 0, {2, 2, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MEDIAN_5X5_08U[] = {{120, 2, 4, 0, {2, 2, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MEDIAN_5X5_08U[] = {{128, 2, 4, 0, {2, 2, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MEDIAN_5X5_08U[] = {{2, 4, 4, 0, {2, 2, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MEDIAN_5X5_08U[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MEDIAN_5X5_08U[] = {{6, 4, 4, 0, {2, 2, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MEDIAN_5X5_08U[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MEDIAN_5X5_08U[] = {{10, 4, 4, 0, {2, 2, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MEDIAN_5X5_08U[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MEDIAN_5X5_08U[] = {{14, 4, 4, 0, {2, 2, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MEDIAN_5X5_08U[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MEDIAN_5X5_08U[] = {{18, 4, 4, 0, {2, 2, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MEDIAN_5X5_08U[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MEDIAN_5X5_08U[] = {{22, 4, 4, 0, {2, 2, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MEDIAN_5X5_08U[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MEDIAN_5X5_08U[] = {{26, 4, 4, 0, {2, 2, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MEDIAN_5X5_08U[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MEDIAN_5X5_08U[] = {{30, 4, 4, 0, {2, 2, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MEDIAN_5X5_08U[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MEDIAN_5X5_08U[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MEDIAN_5X5_08U[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MEDIAN_5X5_08U[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MEDIAN_5X5_08U[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MEDIAN_5X5_08U[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MEDIAN_5X5_08U[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MEDIAN_5X5_08U[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MEDIAN_5X5_08U[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MEDIAN_5X5_08U[] = {{72, 4, 4, 0, {2, 2, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MEDIAN_5X5_08U[] = {{80, 4, 4, 0, {2, 2, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MEDIAN_5X5_08U[] = {{88, 4, 4, 0, {2, 2, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MEDIAN_5X5_08U[] = {{96, 4, 4, 0, {2, 2, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MEDIAN_5X5_08U[] = {{104, 4, 4, 0, {2, 2, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MEDIAN_5X5_08U[] = {{112, 4, 4, 0, {2, 2, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MEDIAN_5X5_08U[] = {{120, 4, 4, 0, {2, 2, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MEDIAN_5X5_08U[] = {{128, 4, 4, 0, {2, 2, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MEDIAN_5X5_08U[] = {{2, 6, 4, 0, {2, 2, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MEDIAN_5X5_08U[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MEDIAN_5X5_08U[] = {{6, 6, 4, 0, {2, 2, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MEDIAN_5X5_08U[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MEDIAN_5X5_08U[] = {{10, 6, 4, 0, {2, 2, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MEDIAN_5X5_08U[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MEDIAN_5X5_08U[] = {{14, 6, 4, 0, {2, 2, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MEDIAN_5X5_08U[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MEDIAN_5X5_08U[] = {{18, 6, 4, 0, {2, 2, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MEDIAN_5X5_08U[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MEDIAN_5X5_08U[] = {{22, 6, 4, 0, {2, 2, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MEDIAN_5X5_08U[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MEDIAN_5X5_08U[] = {{26, 6, 4, 0, {2, 2, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MEDIAN_5X5_08U[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MEDIAN_5X5_08U[] = {{30, 6, 4, 0, {2, 2, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MEDIAN_5X5_08U[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MEDIAN_5X5_08U[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MEDIAN_5X5_08U[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MEDIAN_5X5_08U[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MEDIAN_5X5_08U[] = {{48, 6, 4, 0, {2, 2, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MEDIAN_5X5_08U[] = {{52, 6, 4, 0, {2, 2, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MEDIAN_5X5_08U[] = {{56, 6, 4, 0, {2, 2, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MEDIAN_5X5_08U[] = {{60, 6, 4, 0, {2, 2, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MEDIAN_5X5_08U[] = {{64, 6, 4, 0, {2, 2, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MEDIAN_5X5_08U[] = {{72, 6, 4, 0, {2, 2, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MEDIAN_5X5_08U[] = {{80, 6, 4, 0, {2, 2, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MEDIAN_5X5_08U[] = {{88, 6, 4, 0, {2, 2, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MEDIAN_5X5_08U[] = {{2, 8, 4, 0, {2, 2, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MEDIAN_5X5_08U[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MEDIAN_5X5_08U[] = {{6, 8, 4, 0, {2, 2, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MEDIAN_5X5_08U[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MEDIAN_5X5_08U[] = {{10, 8, 4, 0, {2, 2, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MEDIAN_5X5_08U[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MEDIAN_5X5_08U[] = {{14, 8, 4, 0, {2, 2, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MEDIAN_5X5_08U[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MEDIAN_5X5_08U[] = {{18, 8, 4, 0, {2, 2, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MEDIAN_5X5_08U[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MEDIAN_5X5_08U[] = {{22, 8, 4, 0, {2, 2, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MEDIAN_5X5_08U[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MEDIAN_5X5_08U[] = {{26, 8, 4, 0, {2, 2, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MEDIAN_5X5_08U[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MEDIAN_5X5_08U[] = {{30, 8, 4, 0, {2, 2, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MEDIAN_5X5_08U[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MEDIAN_5X5_08U[] = {{36, 8, 4, 0, {2, 2, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MEDIAN_5X5_08U[] = {{40, 8, 4, 0, {2, 2, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MEDIAN_5X5_08U[] = {{44, 8, 4, 0, {2, 2, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MEDIAN_5X5_08U[] = {{48, 8, 4, 0, {2, 2, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MEDIAN_5X5_08U[] = {{52, 8, 4, 0, {2, 2, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MEDIAN_5X5_08U[] = {{56, 8, 4, 0, {2, 2, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MEDIAN_5X5_08U[] = {{60, 8, 4, 0, {2, 2, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MEDIAN_5X5_08U[] = {{64, 8, 4, 0, {2, 2, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MEDIAN_5X5_08U[] = {{2, 10, 4, 0, {2, 2, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MEDIAN_5X5_08U[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MEDIAN_5X5_08U[] = {{6, 10, 4, 0, {2, 2, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MEDIAN_5X5_08U[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MEDIAN_5X5_08U[] = {{10, 10, 4, 0, {2, 2, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MEDIAN_5X5_08U[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MEDIAN_5X5_08U[] = {{14, 10, 4, 0, {2, 2, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MEDIAN_5X5_08U[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MEDIAN_5X5_08U[] = {{18, 10, 4, 0, {2, 2, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MEDIAN_5X5_08U[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MEDIAN_5X5_08U[] = {{22, 10, 4, 0, {2, 2, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MEDIAN_5X5_08U[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MEDIAN_5X5_08U[] = {{26, 10, 4, 0, {2, 2, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MEDIAN_5X5_08U[] = {{28, 10, 4, 0, {2, 2, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MEDIAN_5X5_08U[] = {{30, 10, 4, 0, {2, 2, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MEDIAN_5X5_08U[] = {{32, 10, 4, 0, {2, 2, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MEDIAN_5X5_08U[] = {{36, 10, 4, 0, {2, 2, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MEDIAN_5X5_08U[] = {{40, 10, 4, 0, {2, 2, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MEDIAN_5X5_08U[] = {{44, 10, 4, 0, {2, 2, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MEDIAN_5X5_08U[] = {{48, 10, 4, 0, {2, 2, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MEDIAN_5X5_08U[] = {{52, 10, 4, 0, {2, 2, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MEDIAN_5X5_08U[] = {{56, 10, 4, 0, {2, 2, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MEDIAN_5X5_08U[] = {{2, 12, 4, 0, {2, 2, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MEDIAN_5X5_08U[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MEDIAN_5X5_08U[] = {{6, 12, 4, 0, {2, 2, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MEDIAN_5X5_08U[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MEDIAN_5X5_08U[] = {{10, 12, 4, 0, {2, 2, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MEDIAN_5X5_08U[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MEDIAN_5X5_08U[] = {{14, 12, 4, 0, {2, 2, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MEDIAN_5X5_08U[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MEDIAN_5X5_08U[] = {{18, 12, 4, 0, {2, 2, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MEDIAN_5X5_08U[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MEDIAN_5X5_08U[] = {{22, 12, 4, 0, {2, 2, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MEDIAN_5X5_08U[] = {{24, 12, 4, 0, {2, 2, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MEDIAN_5X5_08U[] = {{26, 12, 4, 0, {2, 2, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MEDIAN_5X5_08U[] = {{28, 12, 4, 0, {2, 2, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MEDIAN_5X5_08U[] = {{30, 12, 4, 0, {2, 2, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MEDIAN_5X5_08U[] = {{32, 12, 4, 0, {2, 2, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MEDIAN_5X5_08U[] = {{36, 12, 4, 0, {2, 2, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MEDIAN_5X5_08U[] = {{40, 12, 4, 0, {2, 2, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MEDIAN_5X5_08U[] = {{44, 12, 4, 0, {2, 2, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MEDIAN_5X5_08U[] = {{2, 16, 4, 0, {2, 2, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MEDIAN_5X5_08U[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MEDIAN_5X5_08U[] = {{6, 16, 4, 0, {2, 2, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MEDIAN_5X5_08U[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MEDIAN_5X5_08U[] = {{10, 16, 4, 0, {2, 2, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MEDIAN_5X5_08U[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MEDIAN_5X5_08U[] = {{14, 16, 4, 0, {2, 2, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MEDIAN_5X5_08U[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MEDIAN_5X5_08U[] = {{18, 16, 4, 0, {2, 2, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MEDIAN_5X5_08U[] = {{20, 16, 4, 0, {2, 2, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MEDIAN_5X5_08U[] = {{22, 16, 4, 0, {2, 2, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MEDIAN_5X5_08U[] = {{24, 16, 4, 0, {2, 2, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MEDIAN_5X5_08U[] = {{26, 16, 4, 0, {2, 2, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MEDIAN_5X5_08U[] = {{28, 16, 4, 0, {2, 2, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MEDIAN_5X5_08U[] = {{30, 16, 4, 0, {2, 2, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MEDIAN_5X5_08U[] = {{32, 16, 4, 0, {2, 2, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MEDIAN_5X5_08U[] = {{2, 20, 4, 0, {2, 2, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MEDIAN_5X5_08U[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MEDIAN_5X5_08U[] = {{6, 20, 4, 0, {2, 2, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MEDIAN_5X5_08U[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MEDIAN_5X5_08U[] = {{10, 20, 4, 0, {2, 2, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MEDIAN_5X5_08U[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MEDIAN_5X5_08U[] = {{14, 20, 4, 0, {2, 2, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MEDIAN_5X5_08U[] = {{16, 20, 4, 0, {2, 2, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MEDIAN_5X5_08U[] = {{18, 20, 4, 0, {2, 2, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MEDIAN_5X5_08U[] = {{20, 20, 4, 0, {2, 2, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MEDIAN_5X5_08U[] = {{22, 20, 4, 0, {2, 2, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MEDIAN_5X5_08U[] = {{24, 20, 4, 0, {2, 2, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MEDIAN_5X5_08U[] = {{26, 20, 4, 0, {2, 2, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MEDIAN_5X5_08U[] = {{28, 20, 4, 0, {2, 2, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MEDIAN_5X5_08U[] = {{2, 24, 4, 0, {2, 2, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MEDIAN_5X5_08U[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MEDIAN_5X5_08U[] = {{6, 24, 4, 0, {2, 2, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MEDIAN_5X5_08U[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MEDIAN_5X5_08U[] = {{10, 24, 4, 0, {2, 2, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MEDIAN_5X5_08U[] = {{12, 24, 4, 0, {2, 2, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MEDIAN_5X5_08U[] = {{14, 24, 4, 0, {2, 2, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MEDIAN_5X5_08U[] = {{16, 24, 4, 0, {2, 2, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MEDIAN_5X5_08U[] = {{18, 24, 4, 0, {2, 2, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MEDIAN_5X5_08U[] = {{20, 24, 4, 0, {2, 2, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MEDIAN_5X5_08U[] = {{22, 24, 4, 0, {2, 2, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MEDIAN_5X5_08U[] = {{2, 28, 4, 0, {2, 2, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MEDIAN_5X5_08U[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MEDIAN_5X5_08U[] = {{6, 28, 4, 0, {2, 2, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MEDIAN_5X5_08U[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MEDIAN_5X5_08U[] = {{10, 28, 4, 0, {2, 2, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MEDIAN_5X5_08U[] = {{12, 28, 4, 0, {2, 2, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MEDIAN_5X5_08U[] = {{14, 28, 4, 0, {2, 2, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MEDIAN_5X5_08U[] = {{16, 28, 4, 0, {2, 2, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MEDIAN_5X5_08U[] = {{18, 28, 4, 0, {2, 2, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MEDIAN_5X5_08U[] = {{2, 32, 4, 0, {2, 2, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MEDIAN_5X5_08U[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MEDIAN_5X5_08U[] = {{6, 32, 4, 0, {2, 2, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MEDIAN_5X5_08U[] = {{8, 32, 4, 0, {2, 2, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MEDIAN_5X5_08U[] = {{10, 32, 4, 0, {2, 2, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MEDIAN_5X5_08U[] = {{12, 32, 4, 0, {2, 2, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MEDIAN_5X5_08U[] = {{14, 32, 4, 0, {2, 2, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MEDIAN_5X5_08U[] = {{16, 32, 4, 0, {2, 2, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MEDIAN_5X5_08U[] = {{2, 36, 4, 0, {2, 2, 2, 2}}, {2, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MEDIAN_5X5_08U[] = {{4, 36, 4, 0, {2, 2, 2, 2}}, {4, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MEDIAN_5X5_08U[] = {{6, 36, 4, 0, {2, 2, 2, 2}}, {6, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MEDIAN_5X5_08U[] = {{8, 36, 4, 0, {2, 2, 2, 2}}, {8, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MEDIAN_5X5_08U[] = {{10, 36, 4, 0, {2, 2, 2, 2}}, {10, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MEDIAN_5X5_08U[] = {{12, 36, 4, 0, {2, 2, 2, 2}}, {12, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MEDIAN_5X5_08U[] = {{14, 36, 4, 0, {2, 2, 2, 2}}, {14, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MEDIAN_5X5_08U[] = {{2, 40, 4, 0, {2, 2, 2, 2}}, {2, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MEDIAN_5X5_08U[] = {{4, 40, 4, 0, {2, 2, 2, 2}}, {4, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MEDIAN_5X5_08U[] = {{6, 40, 4, 0, {2, 2, 2, 2}}, {6, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MEDIAN_5X5_08U[] = {{8, 40, 4, 0, {2, 2, 2, 2}}, {8, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MEDIAN_5X5_08U[] = {{10, 40, 4, 0, {2, 2, 2, 2}}, {10, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MEDIAN_5X5_08U[] = {{12, 40, 4, 0, {2, 2, 2, 2}}, {12, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MEDIAN_5X5_08U[] = {{2, 44, 4, 0, {2, 2, 2, 2}}, {2, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MEDIAN_5X5_08U[] = {{4, 44, 4, 0, {2, 2, 2, 2}}, {4, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MEDIAN_5X5_08U[] = {{6, 44, 4, 0, {2, 2, 2, 2}}, {6, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MEDIAN_5X5_08U[] = {{8, 44, 4, 0, {2, 2, 2, 2}}, {8, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MEDIAN_5X5_08U[] = {{10, 44, 4, 0, {2, 2, 2, 2}}, {10, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MEDIAN_5X5_08U[] = {{2, 48, 4, 0, {2, 2, 2, 2}}, {2, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MEDIAN_5X5_08U[] = {{4, 48, 4, 0, {2, 2, 2, 2}}, {4, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MEDIAN_5X5_08U[] = {{6, 48, 4, 0, {2, 2, 2, 2}}, {6, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MEDIAN_5X5_08U[] = {{8, 48, 4, 0, {2, 2, 2, 2}}, {8, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MEDIAN_5X5_08U[] = {{10, 48, 4, 0, {2, 2, 2, 2}}, {10, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MEDIAN_5X5_08U[] = {{2, 52, 4, 0, {2, 2, 2, 2}}, {2, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MEDIAN_5X5_08U[] = {{4, 52, 4, 0, {2, 2, 2, 2}}, {4, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MEDIAN_5X5_08U[] = {{6, 52, 4, 0, {2, 2, 2, 2}}, {6, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MEDIAN_5X5_08U[] = {{8, 52, 4, 0, {2, 2, 2, 2}}, {8, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MEDIAN_5X5_08U[] = {{2, 56, 4, 0, {2, 2, 2, 2}}, {2, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MEDIAN_5X5_08U[] = {{4, 56, 4, 0, {2, 2, 2, 2}}, {4, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MEDIAN_5X5_08U[] = {{6, 56, 4, 0, {2, 2, 2, 2}}, {6, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MEDIAN_5X5_08U[] = {{8, 56, 4, 0, {2, 2, 2, 2}}, {8, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MEDIAN_5X5_08U[] = {{2, 60, 4, 0, {2, 2, 2, 2}}, {2, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MEDIAN_5X5_08U[] = {{4, 60, 4, 0, {2, 2, 2, 2}}, {4, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MEDIAN_5X5_08U[] = {{6, 60, 4, 0, {2, 2, 2, 2}}, {6, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MEDIAN_5X5_08U[] = {{2, 64, 4, 0, {2, 2, 2, 2}}, {2, 64, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MEDIAN_5X5_08U[] = {{4, 64, 4, 0, {2, 2, 2, 2}}, {4, 64, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MEDIAN_5X5_08U[] = {{6, 64, 4, 0, {2, 2, 2, 2}}, {6, 64, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MEDIAN_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MEDIAN_5X5_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_MEDIAN_5X5_08U[] = {
{1, 1, 80, 0, 1, gScenarioBufferData0_MEDIAN_5X5_08U, 32, gScenarioKernelData0_MEDIAN_5X5_08U, 4},
{2, 1, 112, 0, 1, gScenarioBufferData1_MEDIAN_5X5_08U, 32, gScenarioKernelData1_MEDIAN_5X5_08U, 4},
{3, 1, 144, 0, 1, gScenarioBufferData2_MEDIAN_5X5_08U, 32, gScenarioKernelData2_MEDIAN_5X5_08U, 4},
{4, 1, 176, 0, 1, gScenarioBufferData3_MEDIAN_5X5_08U, 32, gScenarioKernelData3_MEDIAN_5X5_08U, 4},
{5, 1, 208, 0, 1, gScenarioBufferData4_MEDIAN_5X5_08U, 32, gScenarioKernelData4_MEDIAN_5X5_08U, 4},
{6, 1, 240, 0, 1, gScenarioBufferData5_MEDIAN_5X5_08U, 32, gScenarioKernelData5_MEDIAN_5X5_08U, 4},
{7, 1, 272, 0, 1, gScenarioBufferData6_MEDIAN_5X5_08U, 32, gScenarioKernelData6_MEDIAN_5X5_08U, 4},
{8, 1, 304, 0, 1, gScenarioBufferData7_MEDIAN_5X5_08U, 32, gScenarioKernelData7_MEDIAN_5X5_08U, 4},
{9, 1, 336, 0, 1, gScenarioBufferData8_MEDIAN_5X5_08U, 32, gScenarioKernelData8_MEDIAN_5X5_08U, 4},
{10, 1, 368, 0, 1, gScenarioBufferData9_MEDIAN_5X5_08U, 32, gScenarioKernelData9_MEDIAN_5X5_08U, 4},
{11, 1, 400, 0, 1, gScenarioBufferData10_MEDIAN_5X5_08U, 32, gScenarioKernelData10_MEDIAN_5X5_08U, 4},
{12, 1, 432, 0, 1, gScenarioBufferData11_MEDIAN_5X5_08U, 32, gScenarioKernelData11_MEDIAN_5X5_08U, 4},
{13, 1, 464, 0, 1, gScenarioBufferData12_MEDIAN_5X5_08U, 32, gScenarioKernelData12_MEDIAN_5X5_08U, 4},
{14, 1, 496, 0, 1, gScenarioBufferData13_MEDIAN_5X5_08U, 32, gScenarioKernelData13_MEDIAN_5X5_08U, 4},
{15, 1, 528, 0, 1, gScenarioBufferData14_MEDIAN_5X5_08U, 32, gScenarioKernelData14_MEDIAN_5X5_08U, 4},
{16, 1, 560, 0, 1, gScenarioBufferData15_MEDIAN_5X5_08U, 32, gScenarioKernelData15_MEDIAN_5X5_08U, 4},
{18, 1, 624, 0, 1, gScenarioBufferData16_MEDIAN_5X5_08U, 32, gScenarioKernelData16_MEDIAN_5X5_08U, 4},
{20, 1, 688, 0, 1, gScenarioBufferData17_MEDIAN_5X5_08U, 32, gScenarioKernelData17_MEDIAN_5X5_08U, 4},
{22, 1, 752, 0, 1, gScenarioBufferData18_MEDIAN_5X5_08U, 32, gScenarioKernelData18_MEDIAN_5X5_08U, 4},
{24, 1, 816, 0, 1, gScenarioBufferData19_MEDIAN_5X5_08U, 32, gScenarioKernelData19_MEDIAN_5X5_08U, 4},
{26, 1, 880, 0, 1, gScenarioBufferData20_MEDIAN_5X5_08U, 32, gScenarioKernelData20_MEDIAN_5X5_08U, 4},
{28, 1, 944, 0, 1, gScenarioBufferData21_MEDIAN_5X5_08U, 32, gScenarioKernelData21_MEDIAN_5X5_08U, 4},
{30, 1, 1008, 0, 1, gScenarioBufferData22_MEDIAN_5X5_08U, 32, gScenarioKernelData22_MEDIAN_5X5_08U, 4},
{32, 1, 1072, 0, 1, gScenarioBufferData23_MEDIAN_5X5_08U, 32, gScenarioKernelData23_MEDIAN_5X5_08U, 4},
{36, 1, 1200, 0, 1, gScenarioBufferData24_MEDIAN_5X5_08U, 32, gScenarioKernelData24_MEDIAN_5X5_08U, 4},
{40, 1, 1328, 0, 1, gScenarioBufferData25_MEDIAN_5X5_08U, 32, gScenarioKernelData25_MEDIAN_5X5_08U, 4},
{44, 1, 1456, 0, 1, gScenarioBufferData26_MEDIAN_5X5_08U, 32, gScenarioKernelData26_MEDIAN_5X5_08U, 4},
{48, 1, 1584, 0, 1, gScenarioBufferData27_MEDIAN_5X5_08U, 32, gScenarioKernelData27_MEDIAN_5X5_08U, 4},
{52, 1, 1712, 0, 1, gScenarioBufferData28_MEDIAN_5X5_08U, 32, gScenarioKernelData28_MEDIAN_5X5_08U, 4},
{56, 1, 1840, 0, 1, gScenarioBufferData29_MEDIAN_5X5_08U, 32, gScenarioKernelData29_MEDIAN_5X5_08U, 4},
{60, 1, 1968, 0, 1, gScenarioBufferData30_MEDIAN_5X5_08U, 32, gScenarioKernelData30_MEDIAN_5X5_08U, 4},
{64, 1, 2096, 0, 1, gScenarioBufferData31_MEDIAN_5X5_08U, 32, gScenarioKernelData31_MEDIAN_5X5_08U, 4},
{1, 2, 136, 0, 1, gScenarioBufferData32_MEDIAN_5X5_08U, 32, gScenarioKernelData32_MEDIAN_5X5_08U, 4},
{2, 2, 192, 0, 1, gScenarioBufferData33_MEDIAN_5X5_08U, 32, gScenarioKernelData33_MEDIAN_5X5_08U, 4},
{3, 2, 248, 0, 1, gScenarioBufferData34_MEDIAN_5X5_08U, 32, gScenarioKernelData34_MEDIAN_5X5_08U, 4},
{4, 2, 304, 0, 1, gScenarioBufferData35_MEDIAN_5X5_08U, 32, gScenarioKernelData35_MEDIAN_5X5_08U, 4},
{5, 2, 360, 0, 1, gScenarioBufferData36_MEDIAN_5X5_08U, 32, gScenarioKernelData36_MEDIAN_5X5_08U, 4},
{6, 2, 416, 0, 1, gScenarioBufferData37_MEDIAN_5X5_08U, 32, gScenarioKernelData37_MEDIAN_5X5_08U, 4},
{7, 2, 472, 0, 1, gScenarioBufferData38_MEDIAN_5X5_08U, 32, gScenarioKernelData38_MEDIAN_5X5_08U, 4},
{8, 2, 528, 0, 1, gScenarioBufferData39_MEDIAN_5X5_08U, 32, gScenarioKernelData39_MEDIAN_5X5_08U, 4},
{9, 2, 584, 0, 1, gScenarioBufferData40_MEDIAN_5X5_08U, 32, gScenarioKernelData40_MEDIAN_5X5_08U, 4},
{10, 2, 640, 0, 1, gScenarioBufferData41_MEDIAN_5X5_08U, 32, gScenarioKernelData41_MEDIAN_5X5_08U, 4},
{11, 2, 696, 0, 1, gScenarioBufferData42_MEDIAN_5X5_08U, 32, gScenarioKernelData42_MEDIAN_5X5_08U, 4},
{12, 2, 752, 0, 1, gScenarioBufferData43_MEDIAN_5X5_08U, 32, gScenarioKernelData43_MEDIAN_5X5_08U, 4},
{13, 2, 808, 0, 1, gScenarioBufferData44_MEDIAN_5X5_08U, 32, gScenarioKernelData44_MEDIAN_5X5_08U, 4},
{14, 2, 864, 0, 1, gScenarioBufferData45_MEDIAN_5X5_08U, 32, gScenarioKernelData45_MEDIAN_5X5_08U, 4},
{15, 2, 920, 0, 1, gScenarioBufferData46_MEDIAN_5X5_08U, 32, gScenarioKernelData46_MEDIAN_5X5_08U, 4},
{16, 2, 976, 0, 1, gScenarioBufferData47_MEDIAN_5X5_08U, 32, gScenarioKernelData47_MEDIAN_5X5_08U, 4},
{18, 2, 1088, 0, 1, gScenarioBufferData48_MEDIAN_5X5_08U, 32, gScenarioKernelData48_MEDIAN_5X5_08U, 4},
{20, 2, 1200, 0, 1, gScenarioBufferData49_MEDIAN_5X5_08U, 32, gScenarioKernelData49_MEDIAN_5X5_08U, 4},
{22, 2, 1312, 0, 1, gScenarioBufferData50_MEDIAN_5X5_08U, 32, gScenarioKernelData50_MEDIAN_5X5_08U, 4},
{24, 2, 1424, 0, 1, gScenarioBufferData51_MEDIAN_5X5_08U, 32, gScenarioKernelData51_MEDIAN_5X5_08U, 4},
{26, 2, 1536, 0, 1, gScenarioBufferData52_MEDIAN_5X5_08U, 32, gScenarioKernelData52_MEDIAN_5X5_08U, 4},
{28, 2, 1648, 0, 1, gScenarioBufferData53_MEDIAN_5X5_08U, 32, gScenarioKernelData53_MEDIAN_5X5_08U, 4},
{30, 2, 1760, 0, 1, gScenarioBufferData54_MEDIAN_5X5_08U, 32, gScenarioKernelData54_MEDIAN_5X5_08U, 4},
{32, 2, 1872, 0, 1, gScenarioBufferData55_MEDIAN_5X5_08U, 32, gScenarioKernelData55_MEDIAN_5X5_08U, 4},
{36, 2, 2096, 0, 1, gScenarioBufferData56_MEDIAN_5X5_08U, 32, gScenarioKernelData56_MEDIAN_5X5_08U, 4},
{40, 2, 2320, 0, 1, gScenarioBufferData57_MEDIAN_5X5_08U, 32, gScenarioKernelData57_MEDIAN_5X5_08U, 4},
{44, 2, 2544, 0, 1, gScenarioBufferData58_MEDIAN_5X5_08U, 32, gScenarioKernelData58_MEDIAN_5X5_08U, 4},
{48, 2, 2768, 0, 1, gScenarioBufferData59_MEDIAN_5X5_08U, 32, gScenarioKernelData59_MEDIAN_5X5_08U, 4},
{52, 2, 2992, 0, 1, gScenarioBufferData60_MEDIAN_5X5_08U, 32, gScenarioKernelData60_MEDIAN_5X5_08U, 4},
{56, 2, 3216, 0, 1, gScenarioBufferData61_MEDIAN_5X5_08U, 32, gScenarioKernelData61_MEDIAN_5X5_08U, 4},
{60, 2, 3440, 0, 1, gScenarioBufferData62_MEDIAN_5X5_08U, 32, gScenarioKernelData62_MEDIAN_5X5_08U, 4},
{64, 2, 3664, 0, 1, gScenarioBufferData63_MEDIAN_5X5_08U, 32, gScenarioKernelData63_MEDIAN_5X5_08U, 4},
{1, 3, 192, 0, 1, gScenarioBufferData64_MEDIAN_5X5_08U, 32, gScenarioKernelData64_MEDIAN_5X5_08U, 4},
{2, 3, 272, 0, 1, gScenarioBufferData65_MEDIAN_5X5_08U, 32, gScenarioKernelData65_MEDIAN_5X5_08U, 4},
{3, 3, 352, 0, 1, gScenarioBufferData66_MEDIAN_5X5_08U, 32, gScenarioKernelData66_MEDIAN_5X5_08U, 4},
{4, 3, 432, 0, 1, gScenarioBufferData67_MEDIAN_5X5_08U, 32, gScenarioKernelData67_MEDIAN_5X5_08U, 4},
{5, 3, 512, 0, 1, gScenarioBufferData68_MEDIAN_5X5_08U, 32, gScenarioKernelData68_MEDIAN_5X5_08U, 4},
{6, 3, 592, 0, 1, gScenarioBufferData69_MEDIAN_5X5_08U, 32, gScenarioKernelData69_MEDIAN_5X5_08U, 4},
{7, 3, 672, 0, 1, gScenarioBufferData70_MEDIAN_5X5_08U, 32, gScenarioKernelData70_MEDIAN_5X5_08U, 4},
{8, 3, 752, 0, 1, gScenarioBufferData71_MEDIAN_5X5_08U, 32, gScenarioKernelData71_MEDIAN_5X5_08U, 4},
{9, 3, 832, 0, 1, gScenarioBufferData72_MEDIAN_5X5_08U, 32, gScenarioKernelData72_MEDIAN_5X5_08U, 4},
{10, 3, 912, 0, 1, gScenarioBufferData73_MEDIAN_5X5_08U, 32, gScenarioKernelData73_MEDIAN_5X5_08U, 4},
{11, 3, 992, 0, 1, gScenarioBufferData74_MEDIAN_5X5_08U, 32, gScenarioKernelData74_MEDIAN_5X5_08U, 4},
{12, 3, 1072, 0, 1, gScenarioBufferData75_MEDIAN_5X5_08U, 32, gScenarioKernelData75_MEDIAN_5X5_08U, 4},
{13, 3, 1152, 0, 1, gScenarioBufferData76_MEDIAN_5X5_08U, 32, gScenarioKernelData76_MEDIAN_5X5_08U, 4},
{14, 3, 1232, 0, 1, gScenarioBufferData77_MEDIAN_5X5_08U, 32, gScenarioKernelData77_MEDIAN_5X5_08U, 4},
{15, 3, 1312, 0, 1, gScenarioBufferData78_MEDIAN_5X5_08U, 32, gScenarioKernelData78_MEDIAN_5X5_08U, 4},
{16, 3, 1392, 0, 1, gScenarioBufferData79_MEDIAN_5X5_08U, 32, gScenarioKernelData79_MEDIAN_5X5_08U, 4},
{18, 3, 1552, 0, 1, gScenarioBufferData80_MEDIAN_5X5_08U, 32, gScenarioKernelData80_MEDIAN_5X5_08U, 4},
{20, 3, 1712, 0, 1, gScenarioBufferData81_MEDIAN_5X5_08U, 32, gScenarioKernelData81_MEDIAN_5X5_08U, 4},
{22, 3, 1872, 0, 1, gScenarioBufferData82_MEDIAN_5X5_08U, 32, gScenarioKernelData82_MEDIAN_5X5_08U, 4},
{24, 3, 2032, 0, 1, gScenarioBufferData83_MEDIAN_5X5_08U, 32, gScenarioKernelData83_MEDIAN_5X5_08U, 4},
{26, 3, 2192, 0, 1, gScenarioBufferData84_MEDIAN_5X5_08U, 32, gScenarioKernelData84_MEDIAN_5X5_08U, 4},
{28, 3, 2352, 0, 1, gScenarioBufferData85_MEDIAN_5X5_08U, 32, gScenarioKernelData85_MEDIAN_5X5_08U, 4},
{30, 3, 2512, 0, 1, gScenarioBufferData86_MEDIAN_5X5_08U, 32, gScenarioKernelData86_MEDIAN_5X5_08U, 4},
{32, 3, 2672, 0, 1, gScenarioBufferData87_MEDIAN_5X5_08U, 32, gScenarioKernelData87_MEDIAN_5X5_08U, 4},
{36, 3, 2992, 0, 1, gScenarioBufferData88_MEDIAN_5X5_08U, 32, gScenarioKernelData88_MEDIAN_5X5_08U, 4},
{40, 3, 3312, 0, 1, gScenarioBufferData89_MEDIAN_5X5_08U, 32, gScenarioKernelData89_MEDIAN_5X5_08U, 4},
{44, 3, 3632, 0, 1, gScenarioBufferData90_MEDIAN_5X5_08U, 32, gScenarioKernelData90_MEDIAN_5X5_08U, 4},
{1, 4, 248, 0, 1, gScenarioBufferData91_MEDIAN_5X5_08U, 32, gScenarioKernelData91_MEDIAN_5X5_08U, 4},
{2, 4, 352, 0, 1, gScenarioBufferData92_MEDIAN_5X5_08U, 32, gScenarioKernelData92_MEDIAN_5X5_08U, 4},
{3, 4, 456, 0, 1, gScenarioBufferData93_MEDIAN_5X5_08U, 32, gScenarioKernelData93_MEDIAN_5X5_08U, 4},
{4, 4, 560, 0, 1, gScenarioBufferData94_MEDIAN_5X5_08U, 32, gScenarioKernelData94_MEDIAN_5X5_08U, 4},
{5, 4, 664, 0, 1, gScenarioBufferData95_MEDIAN_5X5_08U, 32, gScenarioKernelData95_MEDIAN_5X5_08U, 4},
{6, 4, 768, 0, 1, gScenarioBufferData96_MEDIAN_5X5_08U, 32, gScenarioKernelData96_MEDIAN_5X5_08U, 4},
{7, 4, 872, 0, 1, gScenarioBufferData97_MEDIAN_5X5_08U, 32, gScenarioKernelData97_MEDIAN_5X5_08U, 4},
{8, 4, 976, 0, 1, gScenarioBufferData98_MEDIAN_5X5_08U, 32, gScenarioKernelData98_MEDIAN_5X5_08U, 4},
{9, 4, 1080, 0, 1, gScenarioBufferData99_MEDIAN_5X5_08U, 32, gScenarioKernelData99_MEDIAN_5X5_08U, 4},
{10, 4, 1184, 0, 1, gScenarioBufferData100_MEDIAN_5X5_08U, 32, gScenarioKernelData100_MEDIAN_5X5_08U, 4},
{11, 4, 1288, 0, 1, gScenarioBufferData101_MEDIAN_5X5_08U, 32, gScenarioKernelData101_MEDIAN_5X5_08U, 4},
{12, 4, 1392, 0, 1, gScenarioBufferData102_MEDIAN_5X5_08U, 32, gScenarioKernelData102_MEDIAN_5X5_08U, 4},
{13, 4, 1496, 0, 1, gScenarioBufferData103_MEDIAN_5X5_08U, 32, gScenarioKernelData103_MEDIAN_5X5_08U, 4},
{14, 4, 1600, 0, 1, gScenarioBufferData104_MEDIAN_5X5_08U, 32, gScenarioKernelData104_MEDIAN_5X5_08U, 4},
{15, 4, 1704, 0, 1, gScenarioBufferData105_MEDIAN_5X5_08U, 32, gScenarioKernelData105_MEDIAN_5X5_08U, 4},
{16, 4, 1808, 0, 1, gScenarioBufferData106_MEDIAN_5X5_08U, 32, gScenarioKernelData106_MEDIAN_5X5_08U, 4},
{18, 4, 2016, 0, 1, gScenarioBufferData107_MEDIAN_5X5_08U, 32, gScenarioKernelData107_MEDIAN_5X5_08U, 4},
{20, 4, 2224, 0, 1, gScenarioBufferData108_MEDIAN_5X5_08U, 32, gScenarioKernelData108_MEDIAN_5X5_08U, 4},
{22, 4, 2432, 0, 1, gScenarioBufferData109_MEDIAN_5X5_08U, 32, gScenarioKernelData109_MEDIAN_5X5_08U, 4},
{24, 4, 2640, 0, 1, gScenarioBufferData110_MEDIAN_5X5_08U, 32, gScenarioKernelData110_MEDIAN_5X5_08U, 4},
{26, 4, 2848, 0, 1, gScenarioBufferData111_MEDIAN_5X5_08U, 32, gScenarioKernelData111_MEDIAN_5X5_08U, 4},
{28, 4, 3056, 0, 1, gScenarioBufferData112_MEDIAN_5X5_08U, 32, gScenarioKernelData112_MEDIAN_5X5_08U, 4},
{30, 4, 3264, 0, 1, gScenarioBufferData113_MEDIAN_5X5_08U, 32, gScenarioKernelData113_MEDIAN_5X5_08U, 4},
{32, 4, 3472, 0, 1, gScenarioBufferData114_MEDIAN_5X5_08U, 32, gScenarioKernelData114_MEDIAN_5X5_08U, 4},
{1, 5, 304, 0, 1, gScenarioBufferData115_MEDIAN_5X5_08U, 32, gScenarioKernelData115_MEDIAN_5X5_08U, 4},
{2, 5, 432, 0, 1, gScenarioBufferData116_MEDIAN_5X5_08U, 32, gScenarioKernelData116_MEDIAN_5X5_08U, 4},
{3, 5, 560, 0, 1, gScenarioBufferData117_MEDIAN_5X5_08U, 32, gScenarioKernelData117_MEDIAN_5X5_08U, 4},
{4, 5, 688, 0, 1, gScenarioBufferData118_MEDIAN_5X5_08U, 32, gScenarioKernelData118_MEDIAN_5X5_08U, 4},
{5, 5, 816, 0, 1, gScenarioBufferData119_MEDIAN_5X5_08U, 32, gScenarioKernelData119_MEDIAN_5X5_08U, 4},
{6, 5, 944, 0, 1, gScenarioBufferData120_MEDIAN_5X5_08U, 32, gScenarioKernelData120_MEDIAN_5X5_08U, 4},
{7, 5, 1072, 0, 1, gScenarioBufferData121_MEDIAN_5X5_08U, 32, gScenarioKernelData121_MEDIAN_5X5_08U, 4},
{8, 5, 1200, 0, 1, gScenarioBufferData122_MEDIAN_5X5_08U, 32, gScenarioKernelData122_MEDIAN_5X5_08U, 4},
{9, 5, 1328, 0, 1, gScenarioBufferData123_MEDIAN_5X5_08U, 32, gScenarioKernelData123_MEDIAN_5X5_08U, 4},
{10, 5, 1456, 0, 1, gScenarioBufferData124_MEDIAN_5X5_08U, 32, gScenarioKernelData124_MEDIAN_5X5_08U, 4},
{11, 5, 1584, 0, 1, gScenarioBufferData125_MEDIAN_5X5_08U, 32, gScenarioKernelData125_MEDIAN_5X5_08U, 4},
{12, 5, 1712, 0, 1, gScenarioBufferData126_MEDIAN_5X5_08U, 32, gScenarioKernelData126_MEDIAN_5X5_08U, 4},
{13, 5, 1840, 0, 1, gScenarioBufferData127_MEDIAN_5X5_08U, 32, gScenarioKernelData127_MEDIAN_5X5_08U, 4},
{14, 5, 1968, 0, 1, gScenarioBufferData128_MEDIAN_5X5_08U, 32, gScenarioKernelData128_MEDIAN_5X5_08U, 4},
{15, 5, 2096, 0, 1, gScenarioBufferData129_MEDIAN_5X5_08U, 32, gScenarioKernelData129_MEDIAN_5X5_08U, 4},
{16, 5, 2224, 0, 1, gScenarioBufferData130_MEDIAN_5X5_08U, 32, gScenarioKernelData130_MEDIAN_5X5_08U, 4},
{18, 5, 2480, 0, 1, gScenarioBufferData131_MEDIAN_5X5_08U, 32, gScenarioKernelData131_MEDIAN_5X5_08U, 4},
{20, 5, 2736, 0, 1, gScenarioBufferData132_MEDIAN_5X5_08U, 32, gScenarioKernelData132_MEDIAN_5X5_08U, 4},
{22, 5, 2992, 0, 1, gScenarioBufferData133_MEDIAN_5X5_08U, 32, gScenarioKernelData133_MEDIAN_5X5_08U, 4},
{24, 5, 3248, 0, 1, gScenarioBufferData134_MEDIAN_5X5_08U, 32, gScenarioKernelData134_MEDIAN_5X5_08U, 4},
{26, 5, 3504, 0, 1, gScenarioBufferData135_MEDIAN_5X5_08U, 32, gScenarioKernelData135_MEDIAN_5X5_08U, 4},
{28, 5, 3760, 0, 1, gScenarioBufferData136_MEDIAN_5X5_08U, 32, gScenarioKernelData136_MEDIAN_5X5_08U, 4},
{1, 6, 360, 0, 1, gScenarioBufferData137_MEDIAN_5X5_08U, 32, gScenarioKernelData137_MEDIAN_5X5_08U, 4},
{2, 6, 512, 0, 1, gScenarioBufferData138_MEDIAN_5X5_08U, 32, gScenarioKernelData138_MEDIAN_5X5_08U, 4},
{3, 6, 664, 0, 1, gScenarioBufferData139_MEDIAN_5X5_08U, 32, gScenarioKernelData139_MEDIAN_5X5_08U, 4},
{4, 6, 816, 0, 1, gScenarioBufferData140_MEDIAN_5X5_08U, 32, gScenarioKernelData140_MEDIAN_5X5_08U, 4},
{5, 6, 968, 0, 1, gScenarioBufferData141_MEDIAN_5X5_08U, 32, gScenarioKernelData141_MEDIAN_5X5_08U, 4},
{6, 6, 1120, 0, 1, gScenarioBufferData142_MEDIAN_5X5_08U, 32, gScenarioKernelData142_MEDIAN_5X5_08U, 4},
{7, 6, 1272, 0, 1, gScenarioBufferData143_MEDIAN_5X5_08U, 32, gScenarioKernelData143_MEDIAN_5X5_08U, 4},
{8, 6, 1424, 0, 1, gScenarioBufferData144_MEDIAN_5X5_08U, 32, gScenarioKernelData144_MEDIAN_5X5_08U, 4},
{9, 6, 1576, 0, 1, gScenarioBufferData145_MEDIAN_5X5_08U, 32, gScenarioKernelData145_MEDIAN_5X5_08U, 4},
{10, 6, 1728, 0, 1, gScenarioBufferData146_MEDIAN_5X5_08U, 32, gScenarioKernelData146_MEDIAN_5X5_08U, 4},
{11, 6, 1880, 0, 1, gScenarioBufferData147_MEDIAN_5X5_08U, 32, gScenarioKernelData147_MEDIAN_5X5_08U, 4},
{12, 6, 2032, 0, 1, gScenarioBufferData148_MEDIAN_5X5_08U, 32, gScenarioKernelData148_MEDIAN_5X5_08U, 4},
{13, 6, 2184, 0, 1, gScenarioBufferData149_MEDIAN_5X5_08U, 32, gScenarioKernelData149_MEDIAN_5X5_08U, 4},
{14, 6, 2336, 0, 1, gScenarioBufferData150_MEDIAN_5X5_08U, 32, gScenarioKernelData150_MEDIAN_5X5_08U, 4},
{15, 6, 2488, 0, 1, gScenarioBufferData151_MEDIAN_5X5_08U, 32, gScenarioKernelData151_MEDIAN_5X5_08U, 4},
{16, 6, 2640, 0, 1, gScenarioBufferData152_MEDIAN_5X5_08U, 32, gScenarioKernelData152_MEDIAN_5X5_08U, 4},
{18, 6, 2944, 0, 1, gScenarioBufferData153_MEDIAN_5X5_08U, 32, gScenarioKernelData153_MEDIAN_5X5_08U, 4},
{20, 6, 3248, 0, 1, gScenarioBufferData154_MEDIAN_5X5_08U, 32, gScenarioKernelData154_MEDIAN_5X5_08U, 4},
{22, 6, 3552, 0, 1, gScenarioBufferData155_MEDIAN_5X5_08U, 32, gScenarioKernelData155_MEDIAN_5X5_08U, 4},
{1, 8, 472, 0, 1, gScenarioBufferData156_MEDIAN_5X5_08U, 32, gScenarioKernelData156_MEDIAN_5X5_08U, 4},
{2, 8, 672, 0, 1, gScenarioBufferData157_MEDIAN_5X5_08U, 32, gScenarioKernelData157_MEDIAN_5X5_08U, 4},
{3, 8, 872, 0, 1, gScenarioBufferData158_MEDIAN_5X5_08U, 32, gScenarioKernelData158_MEDIAN_5X5_08U, 4},
{4, 8, 1072, 0, 1, gScenarioBufferData159_MEDIAN_5X5_08U, 32, gScenarioKernelData159_MEDIAN_5X5_08U, 4},
{5, 8, 1272, 0, 1, gScenarioBufferData160_MEDIAN_5X5_08U, 32, gScenarioKernelData160_MEDIAN_5X5_08U, 4},
{6, 8, 1472, 0, 1, gScenarioBufferData161_MEDIAN_5X5_08U, 32, gScenarioKernelData161_MEDIAN_5X5_08U, 4},
{7, 8, 1672, 0, 1, gScenarioBufferData162_MEDIAN_5X5_08U, 32, gScenarioKernelData162_MEDIAN_5X5_08U, 4},
{8, 8, 1872, 0, 1, gScenarioBufferData163_MEDIAN_5X5_08U, 32, gScenarioKernelData163_MEDIAN_5X5_08U, 4},
{9, 8, 2072, 0, 1, gScenarioBufferData164_MEDIAN_5X5_08U, 32, gScenarioKernelData164_MEDIAN_5X5_08U, 4},
{10, 8, 2272, 0, 1, gScenarioBufferData165_MEDIAN_5X5_08U, 32, gScenarioKernelData165_MEDIAN_5X5_08U, 4},
{11, 8, 2472, 0, 1, gScenarioBufferData166_MEDIAN_5X5_08U, 32, gScenarioKernelData166_MEDIAN_5X5_08U, 4},
{12, 8, 2672, 0, 1, gScenarioBufferData167_MEDIAN_5X5_08U, 32, gScenarioKernelData167_MEDIAN_5X5_08U, 4},
{13, 8, 2872, 0, 1, gScenarioBufferData168_MEDIAN_5X5_08U, 32, gScenarioKernelData168_MEDIAN_5X5_08U, 4},
{14, 8, 3072, 0, 1, gScenarioBufferData169_MEDIAN_5X5_08U, 32, gScenarioKernelData169_MEDIAN_5X5_08U, 4},
{15, 8, 3272, 0, 1, gScenarioBufferData170_MEDIAN_5X5_08U, 32, gScenarioKernelData170_MEDIAN_5X5_08U, 4},
{16, 8, 3472, 0, 1, gScenarioBufferData171_MEDIAN_5X5_08U, 32, gScenarioKernelData171_MEDIAN_5X5_08U, 4},
{1, 10, 584, 0, 1, gScenarioBufferData172_MEDIAN_5X5_08U, 32, gScenarioKernelData172_MEDIAN_5X5_08U, 4},
{2, 10, 832, 0, 1, gScenarioBufferData173_MEDIAN_5X5_08U, 32, gScenarioKernelData173_MEDIAN_5X5_08U, 4},
{3, 10, 1080, 0, 1, gScenarioBufferData174_MEDIAN_5X5_08U, 32, gScenarioKernelData174_MEDIAN_5X5_08U, 4},
{4, 10, 1328, 0, 1, gScenarioBufferData175_MEDIAN_5X5_08U, 32, gScenarioKernelData175_MEDIAN_5X5_08U, 4},
{5, 10, 1576, 0, 1, gScenarioBufferData176_MEDIAN_5X5_08U, 32, gScenarioKernelData176_MEDIAN_5X5_08U, 4},
{6, 10, 1824, 0, 1, gScenarioBufferData177_MEDIAN_5X5_08U, 32, gScenarioKernelData177_MEDIAN_5X5_08U, 4},
{7, 10, 2072, 0, 1, gScenarioBufferData178_MEDIAN_5X5_08U, 32, gScenarioKernelData178_MEDIAN_5X5_08U, 4},
{8, 10, 2320, 0, 1, gScenarioBufferData179_MEDIAN_5X5_08U, 32, gScenarioKernelData179_MEDIAN_5X5_08U, 4},
{9, 10, 2568, 0, 1, gScenarioBufferData180_MEDIAN_5X5_08U, 32, gScenarioKernelData180_MEDIAN_5X5_08U, 4},
{10, 10, 2816, 0, 1, gScenarioBufferData181_MEDIAN_5X5_08U, 32, gScenarioKernelData181_MEDIAN_5X5_08U, 4},
{11, 10, 3064, 0, 1, gScenarioBufferData182_MEDIAN_5X5_08U, 32, gScenarioKernelData182_MEDIAN_5X5_08U, 4},
{12, 10, 3312, 0, 1, gScenarioBufferData183_MEDIAN_5X5_08U, 32, gScenarioKernelData183_MEDIAN_5X5_08U, 4},
{13, 10, 3560, 0, 1, gScenarioBufferData184_MEDIAN_5X5_08U, 32, gScenarioKernelData184_MEDIAN_5X5_08U, 4},
{14, 10, 3808, 0, 1, gScenarioBufferData185_MEDIAN_5X5_08U, 32, gScenarioKernelData185_MEDIAN_5X5_08U, 4},
{1, 12, 696, 0, 1, gScenarioBufferData186_MEDIAN_5X5_08U, 32, gScenarioKernelData186_MEDIAN_5X5_08U, 4},
{2, 12, 992, 0, 1, gScenarioBufferData187_MEDIAN_5X5_08U, 32, gScenarioKernelData187_MEDIAN_5X5_08U, 4},
{3, 12, 1288, 0, 1, gScenarioBufferData188_MEDIAN_5X5_08U, 32, gScenarioKernelData188_MEDIAN_5X5_08U, 4},
{4, 12, 1584, 0, 1, gScenarioBufferData189_MEDIAN_5X5_08U, 32, gScenarioKernelData189_MEDIAN_5X5_08U, 4},
{5, 12, 1880, 0, 1, gScenarioBufferData190_MEDIAN_5X5_08U, 32, gScenarioKernelData190_MEDIAN_5X5_08U, 4},
{6, 12, 2176, 0, 1, gScenarioBufferData191_MEDIAN_5X5_08U, 32, gScenarioKernelData191_MEDIAN_5X5_08U, 4},
{7, 12, 2472, 0, 1, gScenarioBufferData192_MEDIAN_5X5_08U, 32, gScenarioKernelData192_MEDIAN_5X5_08U, 4},
{8, 12, 2768, 0, 1, gScenarioBufferData193_MEDIAN_5X5_08U, 32, gScenarioKernelData193_MEDIAN_5X5_08U, 4},
{9, 12, 3064, 0, 1, gScenarioBufferData194_MEDIAN_5X5_08U, 32, gScenarioKernelData194_MEDIAN_5X5_08U, 4},
{10, 12, 3360, 0, 1, gScenarioBufferData195_MEDIAN_5X5_08U, 32, gScenarioKernelData195_MEDIAN_5X5_08U, 4},
{11, 12, 3656, 0, 1, gScenarioBufferData196_MEDIAN_5X5_08U, 32, gScenarioKernelData196_MEDIAN_5X5_08U, 4},
{1, 14, 808, 0, 1, gScenarioBufferData197_MEDIAN_5X5_08U, 32, gScenarioKernelData197_MEDIAN_5X5_08U, 4},
{2, 14, 1152, 0, 1, gScenarioBufferData198_MEDIAN_5X5_08U, 32, gScenarioKernelData198_MEDIAN_5X5_08U, 4},
{3, 14, 1496, 0, 1, gScenarioBufferData199_MEDIAN_5X5_08U, 32, gScenarioKernelData199_MEDIAN_5X5_08U, 4},
{4, 14, 1840, 0, 1, gScenarioBufferData200_MEDIAN_5X5_08U, 32, gScenarioKernelData200_MEDIAN_5X5_08U, 4},
{5, 14, 2184, 0, 1, gScenarioBufferData201_MEDIAN_5X5_08U, 32, gScenarioKernelData201_MEDIAN_5X5_08U, 4},
{6, 14, 2528, 0, 1, gScenarioBufferData202_MEDIAN_5X5_08U, 32, gScenarioKernelData202_MEDIAN_5X5_08U, 4},
{7, 14, 2872, 0, 1, gScenarioBufferData203_MEDIAN_5X5_08U, 32, gScenarioKernelData203_MEDIAN_5X5_08U, 4},
{8, 14, 3216, 0, 1, gScenarioBufferData204_MEDIAN_5X5_08U, 32, gScenarioKernelData204_MEDIAN_5X5_08U, 4},
{9, 14, 3560, 0, 1, gScenarioBufferData205_MEDIAN_5X5_08U, 32, gScenarioKernelData205_MEDIAN_5X5_08U, 4},
{1, 16, 920, 0, 1, gScenarioBufferData206_MEDIAN_5X5_08U, 32, gScenarioKernelData206_MEDIAN_5X5_08U, 4},
{2, 16, 1312, 0, 1, gScenarioBufferData207_MEDIAN_5X5_08U, 32, gScenarioKernelData207_MEDIAN_5X5_08U, 4},
{3, 16, 1704, 0, 1, gScenarioBufferData208_MEDIAN_5X5_08U, 32, gScenarioKernelData208_MEDIAN_5X5_08U, 4},
{4, 16, 2096, 0, 1, gScenarioBufferData209_MEDIAN_5X5_08U, 32, gScenarioKernelData209_MEDIAN_5X5_08U, 4},
{5, 16, 2488, 0, 1, gScenarioBufferData210_MEDIAN_5X5_08U, 32, gScenarioKernelData210_MEDIAN_5X5_08U, 4},
{6, 16, 2880, 0, 1, gScenarioBufferData211_MEDIAN_5X5_08U, 32, gScenarioKernelData211_MEDIAN_5X5_08U, 4},
{7, 16, 3272, 0, 1, gScenarioBufferData212_MEDIAN_5X5_08U, 32, gScenarioKernelData212_MEDIAN_5X5_08U, 4},
{8, 16, 3664, 0, 1, gScenarioBufferData213_MEDIAN_5X5_08U, 32, gScenarioKernelData213_MEDIAN_5X5_08U, 4},
{1, 18, 1032, 0, 1, gScenarioBufferData214_MEDIAN_5X5_08U, 32, gScenarioKernelData214_MEDIAN_5X5_08U, 4},
{2, 18, 1472, 0, 1, gScenarioBufferData215_MEDIAN_5X5_08U, 32, gScenarioKernelData215_MEDIAN_5X5_08U, 4},
{3, 18, 1912, 0, 1, gScenarioBufferData216_MEDIAN_5X5_08U, 32, gScenarioKernelData216_MEDIAN_5X5_08U, 4},
{4, 18, 2352, 0, 1, gScenarioBufferData217_MEDIAN_5X5_08U, 32, gScenarioKernelData217_MEDIAN_5X5_08U, 4},
{5, 18, 2792, 0, 1, gScenarioBufferData218_MEDIAN_5X5_08U, 32, gScenarioKernelData218_MEDIAN_5X5_08U, 4},
{6, 18, 3232, 0, 1, gScenarioBufferData219_MEDIAN_5X5_08U, 32, gScenarioKernelData219_MEDIAN_5X5_08U, 4},
{7, 18, 3672, 0, 1, gScenarioBufferData220_MEDIAN_5X5_08U, 32, gScenarioKernelData220_MEDIAN_5X5_08U, 4},
{1, 20, 1144, 0, 1, gScenarioBufferData221_MEDIAN_5X5_08U, 32, gScenarioKernelData221_MEDIAN_5X5_08U, 4},
{2, 20, 1632, 0, 1, gScenarioBufferData222_MEDIAN_5X5_08U, 32, gScenarioKernelData222_MEDIAN_5X5_08U, 4},
{3, 20, 2120, 0, 1, gScenarioBufferData223_MEDIAN_5X5_08U, 32, gScenarioKernelData223_MEDIAN_5X5_08U, 4},
{4, 20, 2608, 0, 1, gScenarioBufferData224_MEDIAN_5X5_08U, 32, gScenarioKernelData224_MEDIAN_5X5_08U, 4},
{5, 20, 3096, 0, 1, gScenarioBufferData225_MEDIAN_5X5_08U, 32, gScenarioKernelData225_MEDIAN_5X5_08U, 4},
{6, 20, 3584, 0, 1, gScenarioBufferData226_MEDIAN_5X5_08U, 32, gScenarioKernelData226_MEDIAN_5X5_08U, 4},
{1, 22, 1256, 0, 1, gScenarioBufferData227_MEDIAN_5X5_08U, 32, gScenarioKernelData227_MEDIAN_5X5_08U, 4},
{2, 22, 1792, 0, 1, gScenarioBufferData228_MEDIAN_5X5_08U, 32, gScenarioKernelData228_MEDIAN_5X5_08U, 4},
{3, 22, 2328, 0, 1, gScenarioBufferData229_MEDIAN_5X5_08U, 32, gScenarioKernelData229_MEDIAN_5X5_08U, 4},
{4, 22, 2864, 0, 1, gScenarioBufferData230_MEDIAN_5X5_08U, 32, gScenarioKernelData230_MEDIAN_5X5_08U, 4},
{5, 22, 3400, 0, 1, gScenarioBufferData231_MEDIAN_5X5_08U, 32, gScenarioKernelData231_MEDIAN_5X5_08U, 4},
{1, 24, 1368, 0, 1, gScenarioBufferData232_MEDIAN_5X5_08U, 32, gScenarioKernelData232_MEDIAN_5X5_08U, 4},
{2, 24, 1952, 0, 1, gScenarioBufferData233_MEDIAN_5X5_08U, 32, gScenarioKernelData233_MEDIAN_5X5_08U, 4},
{3, 24, 2536, 0, 1, gScenarioBufferData234_MEDIAN_5X5_08U, 32, gScenarioKernelData234_MEDIAN_5X5_08U, 4},
{4, 24, 3120, 0, 1, gScenarioBufferData235_MEDIAN_5X5_08U, 32, gScenarioKernelData235_MEDIAN_5X5_08U, 4},
{5, 24, 3704, 0, 1, gScenarioBufferData236_MEDIAN_5X5_08U, 32, gScenarioKernelData236_MEDIAN_5X5_08U, 4},
{1, 26, 1480, 0, 1, gScenarioBufferData237_MEDIAN_5X5_08U, 32, gScenarioKernelData237_MEDIAN_5X5_08U, 4},
{2, 26, 2112, 0, 1, gScenarioBufferData238_MEDIAN_5X5_08U, 32, gScenarioKernelData238_MEDIAN_5X5_08U, 4},
{3, 26, 2744, 0, 1, gScenarioBufferData239_MEDIAN_5X5_08U, 32, gScenarioKernelData239_MEDIAN_5X5_08U, 4},
{4, 26, 3376, 0, 1, gScenarioBufferData240_MEDIAN_5X5_08U, 32, gScenarioKernelData240_MEDIAN_5X5_08U, 4},
{1, 28, 1592, 0, 1, gScenarioBufferData241_MEDIAN_5X5_08U, 32, gScenarioKernelData241_MEDIAN_5X5_08U, 4},
{2, 28, 2272, 0, 1, gScenarioBufferData242_MEDIAN_5X5_08U, 32, gScenarioKernelData242_MEDIAN_5X5_08U, 4},
{3, 28, 2952, 0, 1, gScenarioBufferData243_MEDIAN_5X5_08U, 32, gScenarioKernelData243_MEDIAN_5X5_08U, 4},
{4, 28, 3632, 0, 1, gScenarioBufferData244_MEDIAN_5X5_08U, 32, gScenarioKernelData244_MEDIAN_5X5_08U, 4},
{1, 30, 1704, 0, 1, gScenarioBufferData245_MEDIAN_5X5_08U, 32, gScenarioKernelData245_MEDIAN_5X5_08U, 4},
{2, 30, 2432, 0, 1, gScenarioBufferData246_MEDIAN_5X5_08U, 32, gScenarioKernelData246_MEDIAN_5X5_08U, 4},
{3, 30, 3160, 0, 1, gScenarioBufferData247_MEDIAN_5X5_08U, 32, gScenarioKernelData247_MEDIAN_5X5_08U, 4},
{1, 32, 1816, 0, 1, gScenarioBufferData248_MEDIAN_5X5_08U, 32, gScenarioKernelData248_MEDIAN_5X5_08U, 4},
{2, 32, 2592, 0, 1, gScenarioBufferData249_MEDIAN_5X5_08U, 32, gScenarioKernelData249_MEDIAN_5X5_08U, 4},
{3, 32, 3368, 0, 1, gScenarioBufferData250_MEDIAN_5X5_08U, 32, gScenarioKernelData250_MEDIAN_5X5_08U, 4}
};

static acf_scenario_list gScenarioList_MEDIAN_5X5_08U = {
251, //number of scenarios
gScenarioArray_MEDIAN_5X5_08U};
//**************************************************************

class MEDIAN_5X5_08U : public ACF_Process_APU
{

public:
   MEDIAN_5X5_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MEDIAN_5X5_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MEDIAN_5X5_08U",
                                        MEDIAN_5X5_08U_LOAD_SEGMENTS,
                                        MEDIAN_5X5_08U_LOAD_PMEM, MEDIAN_5X5_08U_LOAD_PMEM_SIZE,
                                        MEDIAN_5X5_08U_LOAD_DMEM, MEDIAN_5X5_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MEDIAN_5X5_08U_APEX_LOG_BUFFER, MEDIAN_5X5_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MEDIAN_5X5_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MEDIAN_5X5_08U

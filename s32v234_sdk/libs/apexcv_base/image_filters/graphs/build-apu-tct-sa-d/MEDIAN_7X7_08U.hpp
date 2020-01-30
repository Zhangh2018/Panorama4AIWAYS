#ifndef _ACF_PROCESS_APU_MEDIAN_7X7_08U
#define _ACF_PROCESS_APU_MEDIAN_7X7_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MEDIAN_7X7_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MEDIAN_7X7_08U[] = {{4, 2, 7, 0, {3, 3, 4, 4}}, {4, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MEDIAN_7X7_08U[] = {{6, 2, 7, 0, {3, 3, 4, 4}}, {6, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MEDIAN_7X7_08U[] = {{8, 2, 7, 0, {3, 3, 4, 4}}, {8, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MEDIAN_7X7_08U[] = {{10, 2, 7, 0, {3, 3, 4, 4}}, {10, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MEDIAN_7X7_08U[] = {{12, 2, 7, 0, {3, 3, 4, 4}}, {12, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MEDIAN_7X7_08U[] = {{14, 2, 7, 0, {3, 3, 4, 4}}, {14, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MEDIAN_7X7_08U[] = {{16, 2, 7, 0, {3, 3, 4, 4}}, {16, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MEDIAN_7X7_08U[] = {{18, 2, 7, 0, {3, 3, 4, 4}}, {18, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MEDIAN_7X7_08U[] = {{20, 2, 7, 0, {3, 3, 4, 4}}, {20, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MEDIAN_7X7_08U[] = {{22, 2, 7, 0, {3, 3, 4, 4}}, {22, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MEDIAN_7X7_08U[] = {{24, 2, 7, 0, {3, 3, 4, 4}}, {24, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MEDIAN_7X7_08U[] = {{26, 2, 7, 0, {3, 3, 4, 4}}, {26, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MEDIAN_7X7_08U[] = {{28, 2, 7, 0, {3, 3, 4, 4}}, {28, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MEDIAN_7X7_08U[] = {{30, 2, 7, 0, {3, 3, 4, 4}}, {30, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MEDIAN_7X7_08U[] = {{32, 2, 7, 0, {3, 3, 4, 4}}, {32, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MEDIAN_7X7_08U[] = {{36, 2, 7, 0, {3, 3, 4, 4}}, {36, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MEDIAN_7X7_08U[] = {{40, 2, 7, 0, {3, 3, 4, 4}}, {40, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MEDIAN_7X7_08U[] = {{44, 2, 7, 0, {3, 3, 4, 4}}, {44, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MEDIAN_7X7_08U[] = {{48, 2, 7, 0, {3, 3, 4, 4}}, {48, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MEDIAN_7X7_08U[] = {{52, 2, 7, 0, {3, 3, 4, 4}}, {52, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MEDIAN_7X7_08U[] = {{56, 2, 7, 0, {3, 3, 4, 4}}, {56, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MEDIAN_7X7_08U[] = {{60, 2, 7, 0, {3, 3, 4, 4}}, {60, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MEDIAN_7X7_08U[] = {{64, 2, 7, 0, {3, 3, 4, 4}}, {64, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MEDIAN_7X7_08U[] = {{72, 2, 7, 0, {3, 3, 4, 4}}, {72, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MEDIAN_7X7_08U[] = {{80, 2, 7, 0, {3, 3, 4, 4}}, {80, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MEDIAN_7X7_08U[] = {{88, 2, 7, 0, {3, 3, 4, 4}}, {88, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MEDIAN_7X7_08U[] = {{96, 2, 7, 0, {3, 3, 4, 4}}, {96, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MEDIAN_7X7_08U[] = {{104, 2, 7, 0, {3, 3, 4, 4}}, {104, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MEDIAN_7X7_08U[] = {{112, 2, 7, 0, {3, 3, 4, 4}}, {112, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MEDIAN_7X7_08U[] = {{120, 2, 7, 0, {3, 3, 4, 4}}, {120, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MEDIAN_7X7_08U[] = {{128, 2, 7, 0, {3, 3, 4, 4}}, {128, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MEDIAN_7X7_08U[] = {{4, 4, 4, 0, {3, 3, 4, 4}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MEDIAN_7X7_08U[] = {{6, 4, 4, 0, {3, 3, 4, 4}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MEDIAN_7X7_08U[] = {{8, 4, 4, 0, {3, 3, 4, 4}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MEDIAN_7X7_08U[] = {{10, 4, 4, 0, {3, 3, 4, 4}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MEDIAN_7X7_08U[] = {{12, 4, 4, 0, {3, 3, 4, 4}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MEDIAN_7X7_08U[] = {{14, 4, 4, 0, {3, 3, 4, 4}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MEDIAN_7X7_08U[] = {{16, 4, 4, 0, {3, 3, 4, 4}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MEDIAN_7X7_08U[] = {{18, 4, 4, 0, {3, 3, 4, 4}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MEDIAN_7X7_08U[] = {{20, 4, 4, 0, {3, 3, 4, 4}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MEDIAN_7X7_08U[] = {{22, 4, 4, 0, {3, 3, 4, 4}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MEDIAN_7X7_08U[] = {{24, 4, 4, 0, {3, 3, 4, 4}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MEDIAN_7X7_08U[] = {{26, 4, 4, 0, {3, 3, 4, 4}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MEDIAN_7X7_08U[] = {{28, 4, 4, 0, {3, 3, 4, 4}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MEDIAN_7X7_08U[] = {{30, 4, 4, 0, {3, 3, 4, 4}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MEDIAN_7X7_08U[] = {{32, 4, 4, 0, {3, 3, 4, 4}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MEDIAN_7X7_08U[] = {{36, 4, 4, 0, {3, 3, 4, 4}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MEDIAN_7X7_08U[] = {{40, 4, 4, 0, {3, 3, 4, 4}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MEDIAN_7X7_08U[] = {{44, 4, 4, 0, {3, 3, 4, 4}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MEDIAN_7X7_08U[] = {{48, 4, 4, 0, {3, 3, 4, 4}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MEDIAN_7X7_08U[] = {{52, 4, 4, 0, {3, 3, 4, 4}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MEDIAN_7X7_08U[] = {{56, 4, 4, 0, {3, 3, 4, 4}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MEDIAN_7X7_08U[] = {{60, 4, 4, 0, {3, 3, 4, 4}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MEDIAN_7X7_08U[] = {{64, 4, 4, 0, {3, 3, 4, 4}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MEDIAN_7X7_08U[] = {{72, 4, 4, 0, {3, 3, 4, 4}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MEDIAN_7X7_08U[] = {{80, 4, 4, 0, {3, 3, 4, 4}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MEDIAN_7X7_08U[] = {{88, 4, 4, 0, {3, 3, 4, 4}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MEDIAN_7X7_08U[] = {{96, 4, 4, 0, {3, 3, 4, 4}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MEDIAN_7X7_08U[] = {{104, 4, 4, 0, {3, 3, 4, 4}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MEDIAN_7X7_08U[] = {{112, 4, 4, 0, {3, 3, 4, 4}}, {112, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MEDIAN_7X7_08U[] = {{120, 4, 4, 0, {3, 3, 4, 4}}, {120, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MEDIAN_7X7_08U[] = {{4, 6, 4, 0, {3, 3, 4, 4}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MEDIAN_7X7_08U[] = {{6, 6, 4, 0, {3, 3, 4, 4}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MEDIAN_7X7_08U[] = {{8, 6, 4, 0, {3, 3, 4, 4}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MEDIAN_7X7_08U[] = {{10, 6, 4, 0, {3, 3, 4, 4}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MEDIAN_7X7_08U[] = {{12, 6, 4, 0, {3, 3, 4, 4}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MEDIAN_7X7_08U[] = {{14, 6, 4, 0, {3, 3, 4, 4}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MEDIAN_7X7_08U[] = {{16, 6, 4, 0, {3, 3, 4, 4}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MEDIAN_7X7_08U[] = {{18, 6, 4, 0, {3, 3, 4, 4}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MEDIAN_7X7_08U[] = {{20, 6, 4, 0, {3, 3, 4, 4}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MEDIAN_7X7_08U[] = {{22, 6, 4, 0, {3, 3, 4, 4}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MEDIAN_7X7_08U[] = {{24, 6, 4, 0, {3, 3, 4, 4}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MEDIAN_7X7_08U[] = {{26, 6, 4, 0, {3, 3, 4, 4}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MEDIAN_7X7_08U[] = {{28, 6, 4, 0, {3, 3, 4, 4}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MEDIAN_7X7_08U[] = {{30, 6, 4, 0, {3, 3, 4, 4}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MEDIAN_7X7_08U[] = {{32, 6, 4, 0, {3, 3, 4, 4}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MEDIAN_7X7_08U[] = {{36, 6, 4, 0, {3, 3, 4, 4}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MEDIAN_7X7_08U[] = {{40, 6, 4, 0, {3, 3, 4, 4}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MEDIAN_7X7_08U[] = {{44, 6, 4, 0, {3, 3, 4, 4}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MEDIAN_7X7_08U[] = {{48, 6, 4, 0, {3, 3, 4, 4}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MEDIAN_7X7_08U[] = {{52, 6, 4, 0, {3, 3, 4, 4}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MEDIAN_7X7_08U[] = {{56, 6, 4, 0, {3, 3, 4, 4}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MEDIAN_7X7_08U[] = {{60, 6, 4, 0, {3, 3, 4, 4}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MEDIAN_7X7_08U[] = {{64, 6, 4, 0, {3, 3, 4, 4}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MEDIAN_7X7_08U[] = {{72, 6, 4, 0, {3, 3, 4, 4}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MEDIAN_7X7_08U[] = {{80, 6, 4, 0, {3, 3, 4, 4}}, {80, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MEDIAN_7X7_08U[] = {{4, 8, 4, 0, {3, 3, 4, 4}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MEDIAN_7X7_08U[] = {{6, 8, 4, 0, {3, 3, 4, 4}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MEDIAN_7X7_08U[] = {{8, 8, 4, 0, {3, 3, 4, 4}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MEDIAN_7X7_08U[] = {{10, 8, 4, 0, {3, 3, 4, 4}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MEDIAN_7X7_08U[] = {{12, 8, 4, 0, {3, 3, 4, 4}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MEDIAN_7X7_08U[] = {{14, 8, 4, 0, {3, 3, 4, 4}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MEDIAN_7X7_08U[] = {{16, 8, 4, 0, {3, 3, 4, 4}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MEDIAN_7X7_08U[] = {{18, 8, 4, 0, {3, 3, 4, 4}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MEDIAN_7X7_08U[] = {{20, 8, 4, 0, {3, 3, 4, 4}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MEDIAN_7X7_08U[] = {{22, 8, 4, 0, {3, 3, 4, 4}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MEDIAN_7X7_08U[] = {{24, 8, 4, 0, {3, 3, 4, 4}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MEDIAN_7X7_08U[] = {{26, 8, 4, 0, {3, 3, 4, 4}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MEDIAN_7X7_08U[] = {{28, 8, 4, 0, {3, 3, 4, 4}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MEDIAN_7X7_08U[] = {{30, 8, 4, 0, {3, 3, 4, 4}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MEDIAN_7X7_08U[] = {{32, 8, 4, 0, {3, 3, 4, 4}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MEDIAN_7X7_08U[] = {{36, 8, 4, 0, {3, 3, 4, 4}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MEDIAN_7X7_08U[] = {{40, 8, 4, 0, {3, 3, 4, 4}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MEDIAN_7X7_08U[] = {{44, 8, 4, 0, {3, 3, 4, 4}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MEDIAN_7X7_08U[] = {{48, 8, 4, 0, {3, 3, 4, 4}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MEDIAN_7X7_08U[] = {{52, 8, 4, 0, {3, 3, 4, 4}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MEDIAN_7X7_08U[] = {{56, 8, 4, 0, {3, 3, 4, 4}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MEDIAN_7X7_08U[] = {{60, 8, 4, 0, {3, 3, 4, 4}}, {60, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MEDIAN_7X7_08U[] = {{64, 8, 4, 0, {3, 3, 4, 4}}, {64, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MEDIAN_7X7_08U[] = {{4, 10, 4, 0, {3, 3, 4, 4}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MEDIAN_7X7_08U[] = {{6, 10, 4, 0, {3, 3, 4, 4}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MEDIAN_7X7_08U[] = {{8, 10, 4, 0, {3, 3, 4, 4}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MEDIAN_7X7_08U[] = {{10, 10, 4, 0, {3, 3, 4, 4}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MEDIAN_7X7_08U[] = {{12, 10, 4, 0, {3, 3, 4, 4}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MEDIAN_7X7_08U[] = {{14, 10, 4, 0, {3, 3, 4, 4}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MEDIAN_7X7_08U[] = {{16, 10, 4, 0, {3, 3, 4, 4}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MEDIAN_7X7_08U[] = {{18, 10, 4, 0, {3, 3, 4, 4}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MEDIAN_7X7_08U[] = {{20, 10, 4, 0, {3, 3, 4, 4}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MEDIAN_7X7_08U[] = {{22, 10, 4, 0, {3, 3, 4, 4}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MEDIAN_7X7_08U[] = {{24, 10, 4, 0, {3, 3, 4, 4}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MEDIAN_7X7_08U[] = {{26, 10, 4, 0, {3, 3, 4, 4}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MEDIAN_7X7_08U[] = {{28, 10, 4, 0, {3, 3, 4, 4}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MEDIAN_7X7_08U[] = {{30, 10, 4, 0, {3, 3, 4, 4}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MEDIAN_7X7_08U[] = {{32, 10, 4, 0, {3, 3, 4, 4}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MEDIAN_7X7_08U[] = {{36, 10, 4, 0, {3, 3, 4, 4}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MEDIAN_7X7_08U[] = {{40, 10, 4, 0, {3, 3, 4, 4}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MEDIAN_7X7_08U[] = {{44, 10, 4, 0, {3, 3, 4, 4}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MEDIAN_7X7_08U[] = {{48, 10, 4, 0, {3, 3, 4, 4}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MEDIAN_7X7_08U[] = {{52, 10, 4, 0, {3, 3, 4, 4}}, {52, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MEDIAN_7X7_08U[] = {{4, 12, 4, 0, {3, 3, 4, 4}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MEDIAN_7X7_08U[] = {{6, 12, 4, 0, {3, 3, 4, 4}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MEDIAN_7X7_08U[] = {{8, 12, 4, 0, {3, 3, 4, 4}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MEDIAN_7X7_08U[] = {{10, 12, 4, 0, {3, 3, 4, 4}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MEDIAN_7X7_08U[] = {{12, 12, 4, 0, {3, 3, 4, 4}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MEDIAN_7X7_08U[] = {{14, 12, 4, 0, {3, 3, 4, 4}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MEDIAN_7X7_08U[] = {{16, 12, 4, 0, {3, 3, 4, 4}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MEDIAN_7X7_08U[] = {{18, 12, 4, 0, {3, 3, 4, 4}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MEDIAN_7X7_08U[] = {{20, 12, 4, 0, {3, 3, 4, 4}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MEDIAN_7X7_08U[] = {{22, 12, 4, 0, {3, 3, 4, 4}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MEDIAN_7X7_08U[] = {{24, 12, 4, 0, {3, 3, 4, 4}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MEDIAN_7X7_08U[] = {{26, 12, 4, 0, {3, 3, 4, 4}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MEDIAN_7X7_08U[] = {{28, 12, 4, 0, {3, 3, 4, 4}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MEDIAN_7X7_08U[] = {{30, 12, 4, 0, {3, 3, 4, 4}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MEDIAN_7X7_08U[] = {{32, 12, 4, 0, {3, 3, 4, 4}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MEDIAN_7X7_08U[] = {{36, 12, 4, 0, {3, 3, 4, 4}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MEDIAN_7X7_08U[] = {{40, 12, 4, 0, {3, 3, 4, 4}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MEDIAN_7X7_08U[] = {{4, 16, 4, 0, {3, 3, 4, 4}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MEDIAN_7X7_08U[] = {{6, 16, 4, 0, {3, 3, 4, 4}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MEDIAN_7X7_08U[] = {{8, 16, 4, 0, {3, 3, 4, 4}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MEDIAN_7X7_08U[] = {{10, 16, 4, 0, {3, 3, 4, 4}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MEDIAN_7X7_08U[] = {{12, 16, 4, 0, {3, 3, 4, 4}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MEDIAN_7X7_08U[] = {{14, 16, 4, 0, {3, 3, 4, 4}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MEDIAN_7X7_08U[] = {{16, 16, 4, 0, {3, 3, 4, 4}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MEDIAN_7X7_08U[] = {{18, 16, 4, 0, {3, 3, 4, 4}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MEDIAN_7X7_08U[] = {{20, 16, 4, 0, {3, 3, 4, 4}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MEDIAN_7X7_08U[] = {{22, 16, 4, 0, {3, 3, 4, 4}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MEDIAN_7X7_08U[] = {{24, 16, 4, 0, {3, 3, 4, 4}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MEDIAN_7X7_08U[] = {{26, 16, 4, 0, {3, 3, 4, 4}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MEDIAN_7X7_08U[] = {{28, 16, 4, 0, {3, 3, 4, 4}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MEDIAN_7X7_08U[] = {{30, 16, 4, 0, {3, 3, 4, 4}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MEDIAN_7X7_08U[] = {{32, 16, 4, 0, {3, 3, 4, 4}}, {32, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MEDIAN_7X7_08U[] = {{4, 20, 4, 0, {3, 3, 4, 4}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MEDIAN_7X7_08U[] = {{6, 20, 4, 0, {3, 3, 4, 4}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MEDIAN_7X7_08U[] = {{8, 20, 4, 0, {3, 3, 4, 4}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MEDIAN_7X7_08U[] = {{10, 20, 4, 0, {3, 3, 4, 4}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MEDIAN_7X7_08U[] = {{12, 20, 4, 0, {3, 3, 4, 4}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MEDIAN_7X7_08U[] = {{14, 20, 4, 0, {3, 3, 4, 4}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MEDIAN_7X7_08U[] = {{16, 20, 4, 0, {3, 3, 4, 4}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MEDIAN_7X7_08U[] = {{18, 20, 4, 0, {3, 3, 4, 4}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MEDIAN_7X7_08U[] = {{20, 20, 4, 0, {3, 3, 4, 4}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MEDIAN_7X7_08U[] = {{22, 20, 4, 0, {3, 3, 4, 4}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MEDIAN_7X7_08U[] = {{24, 20, 4, 0, {3, 3, 4, 4}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MEDIAN_7X7_08U[] = {{4, 24, 4, 0, {3, 3, 4, 4}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MEDIAN_7X7_08U[] = {{6, 24, 4, 0, {3, 3, 4, 4}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MEDIAN_7X7_08U[] = {{8, 24, 4, 0, {3, 3, 4, 4}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MEDIAN_7X7_08U[] = {{10, 24, 4, 0, {3, 3, 4, 4}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MEDIAN_7X7_08U[] = {{12, 24, 4, 0, {3, 3, 4, 4}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MEDIAN_7X7_08U[] = {{14, 24, 4, 0, {3, 3, 4, 4}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MEDIAN_7X7_08U[] = {{16, 24, 4, 0, {3, 3, 4, 4}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MEDIAN_7X7_08U[] = {{18, 24, 4, 0, {3, 3, 4, 4}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MEDIAN_7X7_08U[] = {{20, 24, 4, 0, {3, 3, 4, 4}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MEDIAN_7X7_08U[] = {{4, 28, 4, 0, {3, 3, 4, 4}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MEDIAN_7X7_08U[] = {{6, 28, 4, 0, {3, 3, 4, 4}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MEDIAN_7X7_08U[] = {{8, 28, 4, 0, {3, 3, 4, 4}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MEDIAN_7X7_08U[] = {{10, 28, 4, 0, {3, 3, 4, 4}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MEDIAN_7X7_08U[] = {{12, 28, 4, 0, {3, 3, 4, 4}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MEDIAN_7X7_08U[] = {{14, 28, 4, 0, {3, 3, 4, 4}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MEDIAN_7X7_08U[] = {{16, 28, 4, 0, {3, 3, 4, 4}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MEDIAN_7X7_08U[] = {{4, 32, 4, 0, {3, 3, 4, 4}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MEDIAN_7X7_08U[] = {{6, 32, 4, 0, {3, 3, 4, 4}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MEDIAN_7X7_08U[] = {{8, 32, 4, 0, {3, 3, 4, 4}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MEDIAN_7X7_08U[] = {{10, 32, 4, 0, {3, 3, 4, 4}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MEDIAN_7X7_08U[] = {{12, 32, 4, 0, {3, 3, 4, 4}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MEDIAN_7X7_08U[] = {{4, 36, 4, 0, {3, 3, 4, 4}}, {4, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MEDIAN_7X7_08U[] = {{6, 36, 4, 0, {3, 3, 4, 4}}, {6, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MEDIAN_7X7_08U[] = {{8, 36, 4, 0, {3, 3, 4, 4}}, {8, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MEDIAN_7X7_08U[] = {{10, 36, 4, 0, {3, 3, 4, 4}}, {10, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MEDIAN_7X7_08U[] = {{4, 40, 4, 0, {3, 3, 4, 4}}, {4, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MEDIAN_7X7_08U[] = {{6, 40, 4, 0, {3, 3, 4, 4}}, {6, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MEDIAN_7X7_08U[] = {{8, 40, 4, 0, {3, 3, 4, 4}}, {8, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MEDIAN_7X7_08U[] = {{10, 40, 4, 0, {3, 3, 4, 4}}, {10, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MEDIAN_7X7_08U[] = {{4, 44, 4, 0, {3, 3, 4, 4}}, {4, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MEDIAN_7X7_08U[] = {{6, 44, 4, 0, {3, 3, 4, 4}}, {6, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MEDIAN_7X7_08U[] = {{8, 44, 4, 0, {3, 3, 4, 4}}, {8, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MEDIAN_7X7_08U[] = {{4, 48, 4, 0, {3, 3, 4, 4}}, {4, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MEDIAN_7X7_08U[] = {{6, 48, 4, 0, {3, 3, 4, 4}}, {6, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MEDIAN_7X7_08U[] = {{4, 52, 4, 0, {3, 3, 4, 4}}, {4, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MEDIAN_7X7_08U[] = {{6, 52, 4, 0, {3, 3, 4, 4}}, {6, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MEDIAN_7X7_08U[] = {{4, 56, 4, 0, {3, 3, 4, 4}}, {4, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MEDIAN_7X7_08U[] = {{4, 60, 4, 0, {3, 3, 4, 4}}, {4, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MEDIAN_7X7_08U[] = {{4, 64, 4, 0, {3, 3, 4, 4}}, {4, 64, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MEDIAN_7X7_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MEDIAN_7X7_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MEDIAN_7X7_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_MEDIAN_7X7_08U[] = {
{2, 1, 256, 0, 2, gScenarioBufferData0_MEDIAN_7X7_08U, 32, gScenarioKernelData0_MEDIAN_7X7_08U, 4},
{3, 1, 304, 0, 2, gScenarioBufferData1_MEDIAN_7X7_08U, 32, gScenarioKernelData1_MEDIAN_7X7_08U, 4},
{4, 1, 352, 0, 2, gScenarioBufferData2_MEDIAN_7X7_08U, 32, gScenarioKernelData2_MEDIAN_7X7_08U, 4},
{5, 1, 400, 0, 2, gScenarioBufferData3_MEDIAN_7X7_08U, 32, gScenarioKernelData3_MEDIAN_7X7_08U, 4},
{6, 1, 448, 0, 2, gScenarioBufferData4_MEDIAN_7X7_08U, 32, gScenarioKernelData4_MEDIAN_7X7_08U, 4},
{7, 1, 496, 0, 2, gScenarioBufferData5_MEDIAN_7X7_08U, 32, gScenarioKernelData5_MEDIAN_7X7_08U, 4},
{8, 1, 544, 0, 2, gScenarioBufferData6_MEDIAN_7X7_08U, 32, gScenarioKernelData6_MEDIAN_7X7_08U, 4},
{9, 1, 592, 0, 2, gScenarioBufferData7_MEDIAN_7X7_08U, 32, gScenarioKernelData7_MEDIAN_7X7_08U, 4},
{10, 1, 640, 0, 2, gScenarioBufferData8_MEDIAN_7X7_08U, 32, gScenarioKernelData8_MEDIAN_7X7_08U, 4},
{11, 1, 688, 0, 2, gScenarioBufferData9_MEDIAN_7X7_08U, 32, gScenarioKernelData9_MEDIAN_7X7_08U, 4},
{12, 1, 736, 0, 2, gScenarioBufferData10_MEDIAN_7X7_08U, 32, gScenarioKernelData10_MEDIAN_7X7_08U, 4},
{13, 1, 784, 0, 2, gScenarioBufferData11_MEDIAN_7X7_08U, 32, gScenarioKernelData11_MEDIAN_7X7_08U, 4},
{14, 1, 832, 0, 2, gScenarioBufferData12_MEDIAN_7X7_08U, 32, gScenarioKernelData12_MEDIAN_7X7_08U, 4},
{15, 1, 880, 0, 2, gScenarioBufferData13_MEDIAN_7X7_08U, 32, gScenarioKernelData13_MEDIAN_7X7_08U, 4},
{16, 1, 928, 0, 2, gScenarioBufferData14_MEDIAN_7X7_08U, 32, gScenarioKernelData14_MEDIAN_7X7_08U, 4},
{18, 1, 1024, 0, 2, gScenarioBufferData15_MEDIAN_7X7_08U, 32, gScenarioKernelData15_MEDIAN_7X7_08U, 4},
{20, 1, 1120, 0, 2, gScenarioBufferData16_MEDIAN_7X7_08U, 32, gScenarioKernelData16_MEDIAN_7X7_08U, 4},
{22, 1, 1216, 0, 2, gScenarioBufferData17_MEDIAN_7X7_08U, 32, gScenarioKernelData17_MEDIAN_7X7_08U, 4},
{24, 1, 1312, 0, 2, gScenarioBufferData18_MEDIAN_7X7_08U, 32, gScenarioKernelData18_MEDIAN_7X7_08U, 4},
{26, 1, 1408, 0, 2, gScenarioBufferData19_MEDIAN_7X7_08U, 32, gScenarioKernelData19_MEDIAN_7X7_08U, 4},
{28, 1, 1504, 0, 2, gScenarioBufferData20_MEDIAN_7X7_08U, 32, gScenarioKernelData20_MEDIAN_7X7_08U, 4},
{30, 1, 1600, 0, 2, gScenarioBufferData21_MEDIAN_7X7_08U, 32, gScenarioKernelData21_MEDIAN_7X7_08U, 4},
{32, 1, 1696, 0, 2, gScenarioBufferData22_MEDIAN_7X7_08U, 32, gScenarioKernelData22_MEDIAN_7X7_08U, 4},
{36, 1, 1888, 0, 2, gScenarioBufferData23_MEDIAN_7X7_08U, 32, gScenarioKernelData23_MEDIAN_7X7_08U, 4},
{40, 1, 2080, 0, 2, gScenarioBufferData24_MEDIAN_7X7_08U, 32, gScenarioKernelData24_MEDIAN_7X7_08U, 4},
{44, 1, 2272, 0, 2, gScenarioBufferData25_MEDIAN_7X7_08U, 32, gScenarioKernelData25_MEDIAN_7X7_08U, 4},
{48, 1, 2464, 0, 2, gScenarioBufferData26_MEDIAN_7X7_08U, 32, gScenarioKernelData26_MEDIAN_7X7_08U, 4},
{52, 1, 2656, 0, 2, gScenarioBufferData27_MEDIAN_7X7_08U, 32, gScenarioKernelData27_MEDIAN_7X7_08U, 4},
{56, 1, 2848, 0, 2, gScenarioBufferData28_MEDIAN_7X7_08U, 32, gScenarioKernelData28_MEDIAN_7X7_08U, 4},
{60, 1, 3040, 0, 2, gScenarioBufferData29_MEDIAN_7X7_08U, 32, gScenarioKernelData29_MEDIAN_7X7_08U, 4},
{64, 1, 3232, 0, 2, gScenarioBufferData30_MEDIAN_7X7_08U, 32, gScenarioKernelData30_MEDIAN_7X7_08U, 4},
{2, 2, 296, 0, 1, gScenarioBufferData31_MEDIAN_7X7_08U, 32, gScenarioKernelData31_MEDIAN_7X7_08U, 4},
{3, 2, 356, 0, 1, gScenarioBufferData32_MEDIAN_7X7_08U, 32, gScenarioKernelData32_MEDIAN_7X7_08U, 4},
{4, 2, 416, 0, 1, gScenarioBufferData33_MEDIAN_7X7_08U, 32, gScenarioKernelData33_MEDIAN_7X7_08U, 4},
{5, 2, 476, 0, 1, gScenarioBufferData34_MEDIAN_7X7_08U, 32, gScenarioKernelData34_MEDIAN_7X7_08U, 4},
{6, 2, 536, 0, 1, gScenarioBufferData35_MEDIAN_7X7_08U, 32, gScenarioKernelData35_MEDIAN_7X7_08U, 4},
{7, 2, 596, 0, 1, gScenarioBufferData36_MEDIAN_7X7_08U, 32, gScenarioKernelData36_MEDIAN_7X7_08U, 4},
{8, 2, 656, 0, 1, gScenarioBufferData37_MEDIAN_7X7_08U, 32, gScenarioKernelData37_MEDIAN_7X7_08U, 4},
{9, 2, 716, 0, 1, gScenarioBufferData38_MEDIAN_7X7_08U, 32, gScenarioKernelData38_MEDIAN_7X7_08U, 4},
{10, 2, 776, 0, 1, gScenarioBufferData39_MEDIAN_7X7_08U, 32, gScenarioKernelData39_MEDIAN_7X7_08U, 4},
{11, 2, 836, 0, 1, gScenarioBufferData40_MEDIAN_7X7_08U, 32, gScenarioKernelData40_MEDIAN_7X7_08U, 4},
{12, 2, 896, 0, 1, gScenarioBufferData41_MEDIAN_7X7_08U, 32, gScenarioKernelData41_MEDIAN_7X7_08U, 4},
{13, 2, 956, 0, 1, gScenarioBufferData42_MEDIAN_7X7_08U, 32, gScenarioKernelData42_MEDIAN_7X7_08U, 4},
{14, 2, 1016, 0, 1, gScenarioBufferData43_MEDIAN_7X7_08U, 32, gScenarioKernelData43_MEDIAN_7X7_08U, 4},
{15, 2, 1076, 0, 1, gScenarioBufferData44_MEDIAN_7X7_08U, 32, gScenarioKernelData44_MEDIAN_7X7_08U, 4},
{16, 2, 1136, 0, 1, gScenarioBufferData45_MEDIAN_7X7_08U, 32, gScenarioKernelData45_MEDIAN_7X7_08U, 4},
{18, 2, 1256, 0, 1, gScenarioBufferData46_MEDIAN_7X7_08U, 32, gScenarioKernelData46_MEDIAN_7X7_08U, 4},
{20, 2, 1376, 0, 1, gScenarioBufferData47_MEDIAN_7X7_08U, 32, gScenarioKernelData47_MEDIAN_7X7_08U, 4},
{22, 2, 1496, 0, 1, gScenarioBufferData48_MEDIAN_7X7_08U, 32, gScenarioKernelData48_MEDIAN_7X7_08U, 4},
{24, 2, 1616, 0, 1, gScenarioBufferData49_MEDIAN_7X7_08U, 32, gScenarioKernelData49_MEDIAN_7X7_08U, 4},
{26, 2, 1736, 0, 1, gScenarioBufferData50_MEDIAN_7X7_08U, 32, gScenarioKernelData50_MEDIAN_7X7_08U, 4},
{28, 2, 1856, 0, 1, gScenarioBufferData51_MEDIAN_7X7_08U, 32, gScenarioKernelData51_MEDIAN_7X7_08U, 4},
{30, 2, 1976, 0, 1, gScenarioBufferData52_MEDIAN_7X7_08U, 32, gScenarioKernelData52_MEDIAN_7X7_08U, 4},
{32, 2, 2096, 0, 1, gScenarioBufferData53_MEDIAN_7X7_08U, 32, gScenarioKernelData53_MEDIAN_7X7_08U, 4},
{36, 2, 2336, 0, 1, gScenarioBufferData54_MEDIAN_7X7_08U, 32, gScenarioKernelData54_MEDIAN_7X7_08U, 4},
{40, 2, 2576, 0, 1, gScenarioBufferData55_MEDIAN_7X7_08U, 32, gScenarioKernelData55_MEDIAN_7X7_08U, 4},
{44, 2, 2816, 0, 1, gScenarioBufferData56_MEDIAN_7X7_08U, 32, gScenarioKernelData56_MEDIAN_7X7_08U, 4},
{48, 2, 3056, 0, 1, gScenarioBufferData57_MEDIAN_7X7_08U, 32, gScenarioKernelData57_MEDIAN_7X7_08U, 4},
{52, 2, 3296, 0, 1, gScenarioBufferData58_MEDIAN_7X7_08U, 32, gScenarioKernelData58_MEDIAN_7X7_08U, 4},
{56, 2, 3536, 0, 1, gScenarioBufferData59_MEDIAN_7X7_08U, 32, gScenarioKernelData59_MEDIAN_7X7_08U, 4},
{60, 2, 3776, 0, 1, gScenarioBufferData60_MEDIAN_7X7_08U, 32, gScenarioKernelData60_MEDIAN_7X7_08U, 4},
{2, 3, 408, 0, 1, gScenarioBufferData61_MEDIAN_7X7_08U, 32, gScenarioKernelData61_MEDIAN_7X7_08U, 4},
{3, 3, 492, 0, 1, gScenarioBufferData62_MEDIAN_7X7_08U, 32, gScenarioKernelData62_MEDIAN_7X7_08U, 4},
{4, 3, 576, 0, 1, gScenarioBufferData63_MEDIAN_7X7_08U, 32, gScenarioKernelData63_MEDIAN_7X7_08U, 4},
{5, 3, 660, 0, 1, gScenarioBufferData64_MEDIAN_7X7_08U, 32, gScenarioKernelData64_MEDIAN_7X7_08U, 4},
{6, 3, 744, 0, 1, gScenarioBufferData65_MEDIAN_7X7_08U, 32, gScenarioKernelData65_MEDIAN_7X7_08U, 4},
{7, 3, 828, 0, 1, gScenarioBufferData66_MEDIAN_7X7_08U, 32, gScenarioKernelData66_MEDIAN_7X7_08U, 4},
{8, 3, 912, 0, 1, gScenarioBufferData67_MEDIAN_7X7_08U, 32, gScenarioKernelData67_MEDIAN_7X7_08U, 4},
{9, 3, 996, 0, 1, gScenarioBufferData68_MEDIAN_7X7_08U, 32, gScenarioKernelData68_MEDIAN_7X7_08U, 4},
{10, 3, 1080, 0, 1, gScenarioBufferData69_MEDIAN_7X7_08U, 32, gScenarioKernelData69_MEDIAN_7X7_08U, 4},
{11, 3, 1164, 0, 1, gScenarioBufferData70_MEDIAN_7X7_08U, 32, gScenarioKernelData70_MEDIAN_7X7_08U, 4},
{12, 3, 1248, 0, 1, gScenarioBufferData71_MEDIAN_7X7_08U, 32, gScenarioKernelData71_MEDIAN_7X7_08U, 4},
{13, 3, 1332, 0, 1, gScenarioBufferData72_MEDIAN_7X7_08U, 32, gScenarioKernelData72_MEDIAN_7X7_08U, 4},
{14, 3, 1416, 0, 1, gScenarioBufferData73_MEDIAN_7X7_08U, 32, gScenarioKernelData73_MEDIAN_7X7_08U, 4},
{15, 3, 1500, 0, 1, gScenarioBufferData74_MEDIAN_7X7_08U, 32, gScenarioKernelData74_MEDIAN_7X7_08U, 4},
{16, 3, 1584, 0, 1, gScenarioBufferData75_MEDIAN_7X7_08U, 32, gScenarioKernelData75_MEDIAN_7X7_08U, 4},
{18, 3, 1752, 0, 1, gScenarioBufferData76_MEDIAN_7X7_08U, 32, gScenarioKernelData76_MEDIAN_7X7_08U, 4},
{20, 3, 1920, 0, 1, gScenarioBufferData77_MEDIAN_7X7_08U, 32, gScenarioKernelData77_MEDIAN_7X7_08U, 4},
{22, 3, 2088, 0, 1, gScenarioBufferData78_MEDIAN_7X7_08U, 32, gScenarioKernelData78_MEDIAN_7X7_08U, 4},
{24, 3, 2256, 0, 1, gScenarioBufferData79_MEDIAN_7X7_08U, 32, gScenarioKernelData79_MEDIAN_7X7_08U, 4},
{26, 3, 2424, 0, 1, gScenarioBufferData80_MEDIAN_7X7_08U, 32, gScenarioKernelData80_MEDIAN_7X7_08U, 4},
{28, 3, 2592, 0, 1, gScenarioBufferData81_MEDIAN_7X7_08U, 32, gScenarioKernelData81_MEDIAN_7X7_08U, 4},
{30, 3, 2760, 0, 1, gScenarioBufferData82_MEDIAN_7X7_08U, 32, gScenarioKernelData82_MEDIAN_7X7_08U, 4},
{32, 3, 2928, 0, 1, gScenarioBufferData83_MEDIAN_7X7_08U, 32, gScenarioKernelData83_MEDIAN_7X7_08U, 4},
{36, 3, 3264, 0, 1, gScenarioBufferData84_MEDIAN_7X7_08U, 32, gScenarioKernelData84_MEDIAN_7X7_08U, 4},
{40, 3, 3600, 0, 1, gScenarioBufferData85_MEDIAN_7X7_08U, 32, gScenarioKernelData85_MEDIAN_7X7_08U, 4},
{2, 4, 520, 0, 1, gScenarioBufferData86_MEDIAN_7X7_08U, 32, gScenarioKernelData86_MEDIAN_7X7_08U, 4},
{3, 4, 628, 0, 1, gScenarioBufferData87_MEDIAN_7X7_08U, 32, gScenarioKernelData87_MEDIAN_7X7_08U, 4},
{4, 4, 736, 0, 1, gScenarioBufferData88_MEDIAN_7X7_08U, 32, gScenarioKernelData88_MEDIAN_7X7_08U, 4},
{5, 4, 844, 0, 1, gScenarioBufferData89_MEDIAN_7X7_08U, 32, gScenarioKernelData89_MEDIAN_7X7_08U, 4},
{6, 4, 952, 0, 1, gScenarioBufferData90_MEDIAN_7X7_08U, 32, gScenarioKernelData90_MEDIAN_7X7_08U, 4},
{7, 4, 1060, 0, 1, gScenarioBufferData91_MEDIAN_7X7_08U, 32, gScenarioKernelData91_MEDIAN_7X7_08U, 4},
{8, 4, 1168, 0, 1, gScenarioBufferData92_MEDIAN_7X7_08U, 32, gScenarioKernelData92_MEDIAN_7X7_08U, 4},
{9, 4, 1276, 0, 1, gScenarioBufferData93_MEDIAN_7X7_08U, 32, gScenarioKernelData93_MEDIAN_7X7_08U, 4},
{10, 4, 1384, 0, 1, gScenarioBufferData94_MEDIAN_7X7_08U, 32, gScenarioKernelData94_MEDIAN_7X7_08U, 4},
{11, 4, 1492, 0, 1, gScenarioBufferData95_MEDIAN_7X7_08U, 32, gScenarioKernelData95_MEDIAN_7X7_08U, 4},
{12, 4, 1600, 0, 1, gScenarioBufferData96_MEDIAN_7X7_08U, 32, gScenarioKernelData96_MEDIAN_7X7_08U, 4},
{13, 4, 1708, 0, 1, gScenarioBufferData97_MEDIAN_7X7_08U, 32, gScenarioKernelData97_MEDIAN_7X7_08U, 4},
{14, 4, 1816, 0, 1, gScenarioBufferData98_MEDIAN_7X7_08U, 32, gScenarioKernelData98_MEDIAN_7X7_08U, 4},
{15, 4, 1924, 0, 1, gScenarioBufferData99_MEDIAN_7X7_08U, 32, gScenarioKernelData99_MEDIAN_7X7_08U, 4},
{16, 4, 2032, 0, 1, gScenarioBufferData100_MEDIAN_7X7_08U, 32, gScenarioKernelData100_MEDIAN_7X7_08U, 4},
{18, 4, 2248, 0, 1, gScenarioBufferData101_MEDIAN_7X7_08U, 32, gScenarioKernelData101_MEDIAN_7X7_08U, 4},
{20, 4, 2464, 0, 1, gScenarioBufferData102_MEDIAN_7X7_08U, 32, gScenarioKernelData102_MEDIAN_7X7_08U, 4},
{22, 4, 2680, 0, 1, gScenarioBufferData103_MEDIAN_7X7_08U, 32, gScenarioKernelData103_MEDIAN_7X7_08U, 4},
{24, 4, 2896, 0, 1, gScenarioBufferData104_MEDIAN_7X7_08U, 32, gScenarioKernelData104_MEDIAN_7X7_08U, 4},
{26, 4, 3112, 0, 1, gScenarioBufferData105_MEDIAN_7X7_08U, 32, gScenarioKernelData105_MEDIAN_7X7_08U, 4},
{28, 4, 3328, 0, 1, gScenarioBufferData106_MEDIAN_7X7_08U, 32, gScenarioKernelData106_MEDIAN_7X7_08U, 4},
{30, 4, 3544, 0, 1, gScenarioBufferData107_MEDIAN_7X7_08U, 32, gScenarioKernelData107_MEDIAN_7X7_08U, 4},
{32, 4, 3760, 0, 1, gScenarioBufferData108_MEDIAN_7X7_08U, 32, gScenarioKernelData108_MEDIAN_7X7_08U, 4},
{2, 5, 632, 0, 1, gScenarioBufferData109_MEDIAN_7X7_08U, 32, gScenarioKernelData109_MEDIAN_7X7_08U, 4},
{3, 5, 764, 0, 1, gScenarioBufferData110_MEDIAN_7X7_08U, 32, gScenarioKernelData110_MEDIAN_7X7_08U, 4},
{4, 5, 896, 0, 1, gScenarioBufferData111_MEDIAN_7X7_08U, 32, gScenarioKernelData111_MEDIAN_7X7_08U, 4},
{5, 5, 1028, 0, 1, gScenarioBufferData112_MEDIAN_7X7_08U, 32, gScenarioKernelData112_MEDIAN_7X7_08U, 4},
{6, 5, 1160, 0, 1, gScenarioBufferData113_MEDIAN_7X7_08U, 32, gScenarioKernelData113_MEDIAN_7X7_08U, 4},
{7, 5, 1292, 0, 1, gScenarioBufferData114_MEDIAN_7X7_08U, 32, gScenarioKernelData114_MEDIAN_7X7_08U, 4},
{8, 5, 1424, 0, 1, gScenarioBufferData115_MEDIAN_7X7_08U, 32, gScenarioKernelData115_MEDIAN_7X7_08U, 4},
{9, 5, 1556, 0, 1, gScenarioBufferData116_MEDIAN_7X7_08U, 32, gScenarioKernelData116_MEDIAN_7X7_08U, 4},
{10, 5, 1688, 0, 1, gScenarioBufferData117_MEDIAN_7X7_08U, 32, gScenarioKernelData117_MEDIAN_7X7_08U, 4},
{11, 5, 1820, 0, 1, gScenarioBufferData118_MEDIAN_7X7_08U, 32, gScenarioKernelData118_MEDIAN_7X7_08U, 4},
{12, 5, 1952, 0, 1, gScenarioBufferData119_MEDIAN_7X7_08U, 32, gScenarioKernelData119_MEDIAN_7X7_08U, 4},
{13, 5, 2084, 0, 1, gScenarioBufferData120_MEDIAN_7X7_08U, 32, gScenarioKernelData120_MEDIAN_7X7_08U, 4},
{14, 5, 2216, 0, 1, gScenarioBufferData121_MEDIAN_7X7_08U, 32, gScenarioKernelData121_MEDIAN_7X7_08U, 4},
{15, 5, 2348, 0, 1, gScenarioBufferData122_MEDIAN_7X7_08U, 32, gScenarioKernelData122_MEDIAN_7X7_08U, 4},
{16, 5, 2480, 0, 1, gScenarioBufferData123_MEDIAN_7X7_08U, 32, gScenarioKernelData123_MEDIAN_7X7_08U, 4},
{18, 5, 2744, 0, 1, gScenarioBufferData124_MEDIAN_7X7_08U, 32, gScenarioKernelData124_MEDIAN_7X7_08U, 4},
{20, 5, 3008, 0, 1, gScenarioBufferData125_MEDIAN_7X7_08U, 32, gScenarioKernelData125_MEDIAN_7X7_08U, 4},
{22, 5, 3272, 0, 1, gScenarioBufferData126_MEDIAN_7X7_08U, 32, gScenarioKernelData126_MEDIAN_7X7_08U, 4},
{24, 5, 3536, 0, 1, gScenarioBufferData127_MEDIAN_7X7_08U, 32, gScenarioKernelData127_MEDIAN_7X7_08U, 4},
{26, 5, 3800, 0, 1, gScenarioBufferData128_MEDIAN_7X7_08U, 32, gScenarioKernelData128_MEDIAN_7X7_08U, 4},
{2, 6, 744, 0, 1, gScenarioBufferData129_MEDIAN_7X7_08U, 32, gScenarioKernelData129_MEDIAN_7X7_08U, 4},
{3, 6, 900, 0, 1, gScenarioBufferData130_MEDIAN_7X7_08U, 32, gScenarioKernelData130_MEDIAN_7X7_08U, 4},
{4, 6, 1056, 0, 1, gScenarioBufferData131_MEDIAN_7X7_08U, 32, gScenarioKernelData131_MEDIAN_7X7_08U, 4},
{5, 6, 1212, 0, 1, gScenarioBufferData132_MEDIAN_7X7_08U, 32, gScenarioKernelData132_MEDIAN_7X7_08U, 4},
{6, 6, 1368, 0, 1, gScenarioBufferData133_MEDIAN_7X7_08U, 32, gScenarioKernelData133_MEDIAN_7X7_08U, 4},
{7, 6, 1524, 0, 1, gScenarioBufferData134_MEDIAN_7X7_08U, 32, gScenarioKernelData134_MEDIAN_7X7_08U, 4},
{8, 6, 1680, 0, 1, gScenarioBufferData135_MEDIAN_7X7_08U, 32, gScenarioKernelData135_MEDIAN_7X7_08U, 4},
{9, 6, 1836, 0, 1, gScenarioBufferData136_MEDIAN_7X7_08U, 32, gScenarioKernelData136_MEDIAN_7X7_08U, 4},
{10, 6, 1992, 0, 1, gScenarioBufferData137_MEDIAN_7X7_08U, 32, gScenarioKernelData137_MEDIAN_7X7_08U, 4},
{11, 6, 2148, 0, 1, gScenarioBufferData138_MEDIAN_7X7_08U, 32, gScenarioKernelData138_MEDIAN_7X7_08U, 4},
{12, 6, 2304, 0, 1, gScenarioBufferData139_MEDIAN_7X7_08U, 32, gScenarioKernelData139_MEDIAN_7X7_08U, 4},
{13, 6, 2460, 0, 1, gScenarioBufferData140_MEDIAN_7X7_08U, 32, gScenarioKernelData140_MEDIAN_7X7_08U, 4},
{14, 6, 2616, 0, 1, gScenarioBufferData141_MEDIAN_7X7_08U, 32, gScenarioKernelData141_MEDIAN_7X7_08U, 4},
{15, 6, 2772, 0, 1, gScenarioBufferData142_MEDIAN_7X7_08U, 32, gScenarioKernelData142_MEDIAN_7X7_08U, 4},
{16, 6, 2928, 0, 1, gScenarioBufferData143_MEDIAN_7X7_08U, 32, gScenarioKernelData143_MEDIAN_7X7_08U, 4},
{18, 6, 3240, 0, 1, gScenarioBufferData144_MEDIAN_7X7_08U, 32, gScenarioKernelData144_MEDIAN_7X7_08U, 4},
{20, 6, 3552, 0, 1, gScenarioBufferData145_MEDIAN_7X7_08U, 32, gScenarioKernelData145_MEDIAN_7X7_08U, 4},
{2, 8, 968, 0, 1, gScenarioBufferData146_MEDIAN_7X7_08U, 32, gScenarioKernelData146_MEDIAN_7X7_08U, 4},
{3, 8, 1172, 0, 1, gScenarioBufferData147_MEDIAN_7X7_08U, 32, gScenarioKernelData147_MEDIAN_7X7_08U, 4},
{4, 8, 1376, 0, 1, gScenarioBufferData148_MEDIAN_7X7_08U, 32, gScenarioKernelData148_MEDIAN_7X7_08U, 4},
{5, 8, 1580, 0, 1, gScenarioBufferData149_MEDIAN_7X7_08U, 32, gScenarioKernelData149_MEDIAN_7X7_08U, 4},
{6, 8, 1784, 0, 1, gScenarioBufferData150_MEDIAN_7X7_08U, 32, gScenarioKernelData150_MEDIAN_7X7_08U, 4},
{7, 8, 1988, 0, 1, gScenarioBufferData151_MEDIAN_7X7_08U, 32, gScenarioKernelData151_MEDIAN_7X7_08U, 4},
{8, 8, 2192, 0, 1, gScenarioBufferData152_MEDIAN_7X7_08U, 32, gScenarioKernelData152_MEDIAN_7X7_08U, 4},
{9, 8, 2396, 0, 1, gScenarioBufferData153_MEDIAN_7X7_08U, 32, gScenarioKernelData153_MEDIAN_7X7_08U, 4},
{10, 8, 2600, 0, 1, gScenarioBufferData154_MEDIAN_7X7_08U, 32, gScenarioKernelData154_MEDIAN_7X7_08U, 4},
{11, 8, 2804, 0, 1, gScenarioBufferData155_MEDIAN_7X7_08U, 32, gScenarioKernelData155_MEDIAN_7X7_08U, 4},
{12, 8, 3008, 0, 1, gScenarioBufferData156_MEDIAN_7X7_08U, 32, gScenarioKernelData156_MEDIAN_7X7_08U, 4},
{13, 8, 3212, 0, 1, gScenarioBufferData157_MEDIAN_7X7_08U, 32, gScenarioKernelData157_MEDIAN_7X7_08U, 4},
{14, 8, 3416, 0, 1, gScenarioBufferData158_MEDIAN_7X7_08U, 32, gScenarioKernelData158_MEDIAN_7X7_08U, 4},
{15, 8, 3620, 0, 1, gScenarioBufferData159_MEDIAN_7X7_08U, 32, gScenarioKernelData159_MEDIAN_7X7_08U, 4},
{16, 8, 3824, 0, 1, gScenarioBufferData160_MEDIAN_7X7_08U, 32, gScenarioKernelData160_MEDIAN_7X7_08U, 4},
{2, 10, 1192, 0, 1, gScenarioBufferData161_MEDIAN_7X7_08U, 32, gScenarioKernelData161_MEDIAN_7X7_08U, 4},
{3, 10, 1444, 0, 1, gScenarioBufferData162_MEDIAN_7X7_08U, 32, gScenarioKernelData162_MEDIAN_7X7_08U, 4},
{4, 10, 1696, 0, 1, gScenarioBufferData163_MEDIAN_7X7_08U, 32, gScenarioKernelData163_MEDIAN_7X7_08U, 4},
{5, 10, 1948, 0, 1, gScenarioBufferData164_MEDIAN_7X7_08U, 32, gScenarioKernelData164_MEDIAN_7X7_08U, 4},
{6, 10, 2200, 0, 1, gScenarioBufferData165_MEDIAN_7X7_08U, 32, gScenarioKernelData165_MEDIAN_7X7_08U, 4},
{7, 10, 2452, 0, 1, gScenarioBufferData166_MEDIAN_7X7_08U, 32, gScenarioKernelData166_MEDIAN_7X7_08U, 4},
{8, 10, 2704, 0, 1, gScenarioBufferData167_MEDIAN_7X7_08U, 32, gScenarioKernelData167_MEDIAN_7X7_08U, 4},
{9, 10, 2956, 0, 1, gScenarioBufferData168_MEDIAN_7X7_08U, 32, gScenarioKernelData168_MEDIAN_7X7_08U, 4},
{10, 10, 3208, 0, 1, gScenarioBufferData169_MEDIAN_7X7_08U, 32, gScenarioKernelData169_MEDIAN_7X7_08U, 4},
{11, 10, 3460, 0, 1, gScenarioBufferData170_MEDIAN_7X7_08U, 32, gScenarioKernelData170_MEDIAN_7X7_08U, 4},
{12, 10, 3712, 0, 1, gScenarioBufferData171_MEDIAN_7X7_08U, 32, gScenarioKernelData171_MEDIAN_7X7_08U, 4},
{2, 12, 1416, 0, 1, gScenarioBufferData172_MEDIAN_7X7_08U, 32, gScenarioKernelData172_MEDIAN_7X7_08U, 4},
{3, 12, 1716, 0, 1, gScenarioBufferData173_MEDIAN_7X7_08U, 32, gScenarioKernelData173_MEDIAN_7X7_08U, 4},
{4, 12, 2016, 0, 1, gScenarioBufferData174_MEDIAN_7X7_08U, 32, gScenarioKernelData174_MEDIAN_7X7_08U, 4},
{5, 12, 2316, 0, 1, gScenarioBufferData175_MEDIAN_7X7_08U, 32, gScenarioKernelData175_MEDIAN_7X7_08U, 4},
{6, 12, 2616, 0, 1, gScenarioBufferData176_MEDIAN_7X7_08U, 32, gScenarioKernelData176_MEDIAN_7X7_08U, 4},
{7, 12, 2916, 0, 1, gScenarioBufferData177_MEDIAN_7X7_08U, 32, gScenarioKernelData177_MEDIAN_7X7_08U, 4},
{8, 12, 3216, 0, 1, gScenarioBufferData178_MEDIAN_7X7_08U, 32, gScenarioKernelData178_MEDIAN_7X7_08U, 4},
{9, 12, 3516, 0, 1, gScenarioBufferData179_MEDIAN_7X7_08U, 32, gScenarioKernelData179_MEDIAN_7X7_08U, 4},
{10, 12, 3816, 0, 1, gScenarioBufferData180_MEDIAN_7X7_08U, 32, gScenarioKernelData180_MEDIAN_7X7_08U, 4},
{2, 14, 1640, 0, 1, gScenarioBufferData181_MEDIAN_7X7_08U, 32, gScenarioKernelData181_MEDIAN_7X7_08U, 4},
{3, 14, 1988, 0, 1, gScenarioBufferData182_MEDIAN_7X7_08U, 32, gScenarioKernelData182_MEDIAN_7X7_08U, 4},
{4, 14, 2336, 0, 1, gScenarioBufferData183_MEDIAN_7X7_08U, 32, gScenarioKernelData183_MEDIAN_7X7_08U, 4},
{5, 14, 2684, 0, 1, gScenarioBufferData184_MEDIAN_7X7_08U, 32, gScenarioKernelData184_MEDIAN_7X7_08U, 4},
{6, 14, 3032, 0, 1, gScenarioBufferData185_MEDIAN_7X7_08U, 32, gScenarioKernelData185_MEDIAN_7X7_08U, 4},
{7, 14, 3380, 0, 1, gScenarioBufferData186_MEDIAN_7X7_08U, 32, gScenarioKernelData186_MEDIAN_7X7_08U, 4},
{8, 14, 3728, 0, 1, gScenarioBufferData187_MEDIAN_7X7_08U, 32, gScenarioKernelData187_MEDIAN_7X7_08U, 4},
{2, 16, 1864, 0, 1, gScenarioBufferData188_MEDIAN_7X7_08U, 32, gScenarioKernelData188_MEDIAN_7X7_08U, 4},
{3, 16, 2260, 0, 1, gScenarioBufferData189_MEDIAN_7X7_08U, 32, gScenarioKernelData189_MEDIAN_7X7_08U, 4},
{4, 16, 2656, 0, 1, gScenarioBufferData190_MEDIAN_7X7_08U, 32, gScenarioKernelData190_MEDIAN_7X7_08U, 4},
{5, 16, 3052, 0, 1, gScenarioBufferData191_MEDIAN_7X7_08U, 32, gScenarioKernelData191_MEDIAN_7X7_08U, 4},
{6, 16, 3448, 0, 1, gScenarioBufferData192_MEDIAN_7X7_08U, 32, gScenarioKernelData192_MEDIAN_7X7_08U, 4},
{2, 18, 2088, 0, 1, gScenarioBufferData193_MEDIAN_7X7_08U, 32, gScenarioKernelData193_MEDIAN_7X7_08U, 4},
{3, 18, 2532, 0, 1, gScenarioBufferData194_MEDIAN_7X7_08U, 32, gScenarioKernelData194_MEDIAN_7X7_08U, 4},
{4, 18, 2976, 0, 1, gScenarioBufferData195_MEDIAN_7X7_08U, 32, gScenarioKernelData195_MEDIAN_7X7_08U, 4},
{5, 18, 3420, 0, 1, gScenarioBufferData196_MEDIAN_7X7_08U, 32, gScenarioKernelData196_MEDIAN_7X7_08U, 4},
{2, 20, 2312, 0, 1, gScenarioBufferData197_MEDIAN_7X7_08U, 32, gScenarioKernelData197_MEDIAN_7X7_08U, 4},
{3, 20, 2804, 0, 1, gScenarioBufferData198_MEDIAN_7X7_08U, 32, gScenarioKernelData198_MEDIAN_7X7_08U, 4},
{4, 20, 3296, 0, 1, gScenarioBufferData199_MEDIAN_7X7_08U, 32, gScenarioKernelData199_MEDIAN_7X7_08U, 4},
{5, 20, 3788, 0, 1, gScenarioBufferData200_MEDIAN_7X7_08U, 32, gScenarioKernelData200_MEDIAN_7X7_08U, 4},
{2, 22, 2536, 0, 1, gScenarioBufferData201_MEDIAN_7X7_08U, 32, gScenarioKernelData201_MEDIAN_7X7_08U, 4},
{3, 22, 3076, 0, 1, gScenarioBufferData202_MEDIAN_7X7_08U, 32, gScenarioKernelData202_MEDIAN_7X7_08U, 4},
{4, 22, 3616, 0, 1, gScenarioBufferData203_MEDIAN_7X7_08U, 32, gScenarioKernelData203_MEDIAN_7X7_08U, 4},
{2, 24, 2760, 0, 1, gScenarioBufferData204_MEDIAN_7X7_08U, 32, gScenarioKernelData204_MEDIAN_7X7_08U, 4},
{3, 24, 3348, 0, 1, gScenarioBufferData205_MEDIAN_7X7_08U, 32, gScenarioKernelData205_MEDIAN_7X7_08U, 4},
{2, 26, 2984, 0, 1, gScenarioBufferData206_MEDIAN_7X7_08U, 32, gScenarioKernelData206_MEDIAN_7X7_08U, 4},
{3, 26, 3620, 0, 1, gScenarioBufferData207_MEDIAN_7X7_08U, 32, gScenarioKernelData207_MEDIAN_7X7_08U, 4},
{2, 28, 3208, 0, 1, gScenarioBufferData208_MEDIAN_7X7_08U, 32, gScenarioKernelData208_MEDIAN_7X7_08U, 4},
{2, 30, 3432, 0, 1, gScenarioBufferData209_MEDIAN_7X7_08U, 32, gScenarioKernelData209_MEDIAN_7X7_08U, 4},
{2, 32, 3656, 0, 1, gScenarioBufferData210_MEDIAN_7X7_08U, 32, gScenarioKernelData210_MEDIAN_7X7_08U, 4}
};

static acf_scenario_list gScenarioList_MEDIAN_7X7_08U = {
211, //number of scenarios
gScenarioArray_MEDIAN_7X7_08U};
//**************************************************************

class MEDIAN_7X7_08U : public ACF_Process_APU
{

public:
   MEDIAN_7X7_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MEDIAN_7X7_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MEDIAN_7X7_08U",
                                        MEDIAN_7X7_08U_LOAD_SEGMENTS,
                                        MEDIAN_7X7_08U_LOAD_PMEM, MEDIAN_7X7_08U_LOAD_PMEM_SIZE,
                                        MEDIAN_7X7_08U_LOAD_DMEM, MEDIAN_7X7_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MEDIAN_7X7_08U_APEX_LOG_BUFFER, MEDIAN_7X7_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 4, 2, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MEDIAN_7X7_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MEDIAN_7X7_08U

#ifndef _ACF_PROCESS_APU_MEDIAN_3X3_08U
#define _ACF_PROCESS_APU_MEDIAN_3X3_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MEDIAN_3X3_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MEDIAN_3X3_08U[] = {{2, 2, 4, 0, {1, 1, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MEDIAN_3X3_08U[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MEDIAN_3X3_08U[] = {{6, 2, 4, 0, {1, 1, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MEDIAN_3X3_08U[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MEDIAN_3X3_08U[] = {{10, 2, 4, 0, {1, 1, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MEDIAN_3X3_08U[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MEDIAN_3X3_08U[] = {{14, 2, 4, 0, {1, 1, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MEDIAN_3X3_08U[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MEDIAN_3X3_08U[] = {{18, 2, 4, 0, {1, 1, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MEDIAN_3X3_08U[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MEDIAN_3X3_08U[] = {{22, 2, 4, 0, {1, 1, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MEDIAN_3X3_08U[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MEDIAN_3X3_08U[] = {{26, 2, 4, 0, {1, 1, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MEDIAN_3X3_08U[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MEDIAN_3X3_08U[] = {{30, 2, 4, 0, {1, 1, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MEDIAN_3X3_08U[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MEDIAN_3X3_08U[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MEDIAN_3X3_08U[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MEDIAN_3X3_08U[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MEDIAN_3X3_08U[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MEDIAN_3X3_08U[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MEDIAN_3X3_08U[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MEDIAN_3X3_08U[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MEDIAN_3X3_08U[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MEDIAN_3X3_08U[] = {{72, 2, 4, 0, {1, 1, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MEDIAN_3X3_08U[] = {{80, 2, 4, 0, {1, 1, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MEDIAN_3X3_08U[] = {{88, 2, 4, 0, {1, 1, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MEDIAN_3X3_08U[] = {{96, 2, 4, 0, {1, 1, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MEDIAN_3X3_08U[] = {{104, 2, 4, 0, {1, 1, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MEDIAN_3X3_08U[] = {{112, 2, 4, 0, {1, 1, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MEDIAN_3X3_08U[] = {{120, 2, 4, 0, {1, 1, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MEDIAN_3X3_08U[] = {{128, 2, 4, 0, {1, 1, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MEDIAN_3X3_08U[] = {{2, 4, 4, 0, {1, 1, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MEDIAN_3X3_08U[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MEDIAN_3X3_08U[] = {{6, 4, 4, 0, {1, 1, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MEDIAN_3X3_08U[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MEDIAN_3X3_08U[] = {{10, 4, 4, 0, {1, 1, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MEDIAN_3X3_08U[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MEDIAN_3X3_08U[] = {{14, 4, 4, 0, {1, 1, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MEDIAN_3X3_08U[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MEDIAN_3X3_08U[] = {{18, 4, 4, 0, {1, 1, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MEDIAN_3X3_08U[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MEDIAN_3X3_08U[] = {{22, 4, 4, 0, {1, 1, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MEDIAN_3X3_08U[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MEDIAN_3X3_08U[] = {{26, 4, 4, 0, {1, 1, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MEDIAN_3X3_08U[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MEDIAN_3X3_08U[] = {{30, 4, 4, 0, {1, 1, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MEDIAN_3X3_08U[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MEDIAN_3X3_08U[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MEDIAN_3X3_08U[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MEDIAN_3X3_08U[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MEDIAN_3X3_08U[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MEDIAN_3X3_08U[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MEDIAN_3X3_08U[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MEDIAN_3X3_08U[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MEDIAN_3X3_08U[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MEDIAN_3X3_08U[] = {{72, 4, 4, 0, {1, 1, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MEDIAN_3X3_08U[] = {{80, 4, 4, 0, {1, 1, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MEDIAN_3X3_08U[] = {{88, 4, 4, 0, {1, 1, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MEDIAN_3X3_08U[] = {{96, 4, 4, 0, {1, 1, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MEDIAN_3X3_08U[] = {{104, 4, 4, 0, {1, 1, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MEDIAN_3X3_08U[] = {{112, 4, 4, 0, {1, 1, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MEDIAN_3X3_08U[] = {{120, 4, 4, 0, {1, 1, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MEDIAN_3X3_08U[] = {{128, 4, 4, 0, {1, 1, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MEDIAN_3X3_08U[] = {{2, 6, 4, 0, {1, 1, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MEDIAN_3X3_08U[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MEDIAN_3X3_08U[] = {{6, 6, 4, 0, {1, 1, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MEDIAN_3X3_08U[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MEDIAN_3X3_08U[] = {{10, 6, 4, 0, {1, 1, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MEDIAN_3X3_08U[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MEDIAN_3X3_08U[] = {{14, 6, 4, 0, {1, 1, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MEDIAN_3X3_08U[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MEDIAN_3X3_08U[] = {{18, 6, 4, 0, {1, 1, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MEDIAN_3X3_08U[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MEDIAN_3X3_08U[] = {{22, 6, 4, 0, {1, 1, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MEDIAN_3X3_08U[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MEDIAN_3X3_08U[] = {{26, 6, 4, 0, {1, 1, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MEDIAN_3X3_08U[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MEDIAN_3X3_08U[] = {{30, 6, 4, 0, {1, 1, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MEDIAN_3X3_08U[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MEDIAN_3X3_08U[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MEDIAN_3X3_08U[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MEDIAN_3X3_08U[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MEDIAN_3X3_08U[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MEDIAN_3X3_08U[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MEDIAN_3X3_08U[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MEDIAN_3X3_08U[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MEDIAN_3X3_08U[] = {{64, 6, 4, 0, {1, 1, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MEDIAN_3X3_08U[] = {{72, 6, 4, 0, {1, 1, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MEDIAN_3X3_08U[] = {{80, 6, 4, 0, {1, 1, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MEDIAN_3X3_08U[] = {{88, 6, 4, 0, {1, 1, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MEDIAN_3X3_08U[] = {{96, 6, 4, 0, {1, 1, 2, 2}}, {96, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MEDIAN_3X3_08U[] = {{2, 8, 4, 0, {1, 1, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MEDIAN_3X3_08U[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MEDIAN_3X3_08U[] = {{6, 8, 4, 0, {1, 1, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MEDIAN_3X3_08U[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MEDIAN_3X3_08U[] = {{10, 8, 4, 0, {1, 1, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MEDIAN_3X3_08U[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MEDIAN_3X3_08U[] = {{14, 8, 4, 0, {1, 1, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MEDIAN_3X3_08U[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MEDIAN_3X3_08U[] = {{18, 8, 4, 0, {1, 1, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MEDIAN_3X3_08U[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MEDIAN_3X3_08U[] = {{22, 8, 4, 0, {1, 1, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MEDIAN_3X3_08U[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MEDIAN_3X3_08U[] = {{26, 8, 4, 0, {1, 1, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MEDIAN_3X3_08U[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MEDIAN_3X3_08U[] = {{30, 8, 4, 0, {1, 1, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MEDIAN_3X3_08U[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MEDIAN_3X3_08U[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MEDIAN_3X3_08U[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MEDIAN_3X3_08U[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MEDIAN_3X3_08U[] = {{48, 8, 4, 0, {1, 1, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MEDIAN_3X3_08U[] = {{52, 8, 4, 0, {1, 1, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MEDIAN_3X3_08U[] = {{56, 8, 4, 0, {1, 1, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MEDIAN_3X3_08U[] = {{60, 8, 4, 0, {1, 1, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MEDIAN_3X3_08U[] = {{64, 8, 4, 0, {1, 1, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MEDIAN_3X3_08U[] = {{72, 8, 4, 0, {1, 1, 2, 2}}, {72, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MEDIAN_3X3_08U[] = {{2, 10, 4, 0, {1, 1, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MEDIAN_3X3_08U[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MEDIAN_3X3_08U[] = {{6, 10, 4, 0, {1, 1, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MEDIAN_3X3_08U[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MEDIAN_3X3_08U[] = {{10, 10, 4, 0, {1, 1, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MEDIAN_3X3_08U[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MEDIAN_3X3_08U[] = {{14, 10, 4, 0, {1, 1, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MEDIAN_3X3_08U[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MEDIAN_3X3_08U[] = {{18, 10, 4, 0, {1, 1, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MEDIAN_3X3_08U[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MEDIAN_3X3_08U[] = {{22, 10, 4, 0, {1, 1, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MEDIAN_3X3_08U[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MEDIAN_3X3_08U[] = {{26, 10, 4, 0, {1, 1, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MEDIAN_3X3_08U[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MEDIAN_3X3_08U[] = {{30, 10, 4, 0, {1, 1, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MEDIAN_3X3_08U[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MEDIAN_3X3_08U[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MEDIAN_3X3_08U[] = {{40, 10, 4, 0, {1, 1, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MEDIAN_3X3_08U[] = {{44, 10, 4, 0, {1, 1, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MEDIAN_3X3_08U[] = {{48, 10, 4, 0, {1, 1, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MEDIAN_3X3_08U[] = {{52, 10, 4, 0, {1, 1, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MEDIAN_3X3_08U[] = {{56, 10, 4, 0, {1, 1, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MEDIAN_3X3_08U[] = {{2, 12, 4, 0, {1, 1, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MEDIAN_3X3_08U[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MEDIAN_3X3_08U[] = {{6, 12, 4, 0, {1, 1, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MEDIAN_3X3_08U[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MEDIAN_3X3_08U[] = {{10, 12, 4, 0, {1, 1, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MEDIAN_3X3_08U[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MEDIAN_3X3_08U[] = {{14, 12, 4, 0, {1, 1, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MEDIAN_3X3_08U[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MEDIAN_3X3_08U[] = {{18, 12, 4, 0, {1, 1, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MEDIAN_3X3_08U[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MEDIAN_3X3_08U[] = {{22, 12, 4, 0, {1, 1, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MEDIAN_3X3_08U[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MEDIAN_3X3_08U[] = {{26, 12, 4, 0, {1, 1, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MEDIAN_3X3_08U[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MEDIAN_3X3_08U[] = {{30, 12, 4, 0, {1, 1, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MEDIAN_3X3_08U[] = {{32, 12, 4, 0, {1, 1, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MEDIAN_3X3_08U[] = {{36, 12, 4, 0, {1, 1, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MEDIAN_3X3_08U[] = {{40, 12, 4, 0, {1, 1, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MEDIAN_3X3_08U[] = {{44, 12, 4, 0, {1, 1, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MEDIAN_3X3_08U[] = {{48, 12, 4, 0, {1, 1, 2, 2}}, {48, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MEDIAN_3X3_08U[] = {{2, 16, 4, 0, {1, 1, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MEDIAN_3X3_08U[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MEDIAN_3X3_08U[] = {{6, 16, 4, 0, {1, 1, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MEDIAN_3X3_08U[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MEDIAN_3X3_08U[] = {{10, 16, 4, 0, {1, 1, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MEDIAN_3X3_08U[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MEDIAN_3X3_08U[] = {{14, 16, 4, 0, {1, 1, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MEDIAN_3X3_08U[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MEDIAN_3X3_08U[] = {{18, 16, 4, 0, {1, 1, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MEDIAN_3X3_08U[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MEDIAN_3X3_08U[] = {{22, 16, 4, 0, {1, 1, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MEDIAN_3X3_08U[] = {{24, 16, 4, 0, {1, 1, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MEDIAN_3X3_08U[] = {{26, 16, 4, 0, {1, 1, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MEDIAN_3X3_08U[] = {{28, 16, 4, 0, {1, 1, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MEDIAN_3X3_08U[] = {{30, 16, 4, 0, {1, 1, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MEDIAN_3X3_08U[] = {{32, 16, 4, 0, {1, 1, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MEDIAN_3X3_08U[] = {{36, 16, 4, 0, {1, 1, 2, 2}}, {36, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MEDIAN_3X3_08U[] = {{2, 20, 4, 0, {1, 1, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MEDIAN_3X3_08U[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MEDIAN_3X3_08U[] = {{6, 20, 4, 0, {1, 1, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MEDIAN_3X3_08U[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MEDIAN_3X3_08U[] = {{10, 20, 4, 0, {1, 1, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MEDIAN_3X3_08U[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MEDIAN_3X3_08U[] = {{14, 20, 4, 0, {1, 1, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MEDIAN_3X3_08U[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MEDIAN_3X3_08U[] = {{18, 20, 4, 0, {1, 1, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MEDIAN_3X3_08U[] = {{20, 20, 4, 0, {1, 1, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MEDIAN_3X3_08U[] = {{22, 20, 4, 0, {1, 1, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MEDIAN_3X3_08U[] = {{24, 20, 4, 0, {1, 1, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MEDIAN_3X3_08U[] = {{26, 20, 4, 0, {1, 1, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MEDIAN_3X3_08U[] = {{28, 20, 4, 0, {1, 1, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MEDIAN_3X3_08U[] = {{2, 24, 4, 0, {1, 1, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MEDIAN_3X3_08U[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MEDIAN_3X3_08U[] = {{6, 24, 4, 0, {1, 1, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MEDIAN_3X3_08U[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MEDIAN_3X3_08U[] = {{10, 24, 4, 0, {1, 1, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MEDIAN_3X3_08U[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MEDIAN_3X3_08U[] = {{14, 24, 4, 0, {1, 1, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MEDIAN_3X3_08U[] = {{16, 24, 4, 0, {1, 1, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MEDIAN_3X3_08U[] = {{18, 24, 4, 0, {1, 1, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MEDIAN_3X3_08U[] = {{20, 24, 4, 0, {1, 1, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MEDIAN_3X3_08U[] = {{22, 24, 4, 0, {1, 1, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MEDIAN_3X3_08U[] = {{2, 28, 4, 0, {1, 1, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MEDIAN_3X3_08U[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MEDIAN_3X3_08U[] = {{6, 28, 4, 0, {1, 1, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MEDIAN_3X3_08U[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MEDIAN_3X3_08U[] = {{10, 28, 4, 0, {1, 1, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MEDIAN_3X3_08U[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MEDIAN_3X3_08U[] = {{14, 28, 4, 0, {1, 1, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MEDIAN_3X3_08U[] = {{16, 28, 4, 0, {1, 1, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MEDIAN_3X3_08U[] = {{18, 28, 4, 0, {1, 1, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MEDIAN_3X3_08U[] = {{2, 32, 4, 0, {1, 1, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MEDIAN_3X3_08U[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MEDIAN_3X3_08U[] = {{6, 32, 4, 0, {1, 1, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MEDIAN_3X3_08U[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MEDIAN_3X3_08U[] = {{10, 32, 4, 0, {1, 1, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MEDIAN_3X3_08U[] = {{12, 32, 4, 0, {1, 1, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MEDIAN_3X3_08U[] = {{14, 32, 4, 0, {1, 1, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MEDIAN_3X3_08U[] = {{16, 32, 4, 0, {1, 1, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MEDIAN_3X3_08U[] = {{2, 36, 4, 0, {1, 1, 2, 2}}, {2, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MEDIAN_3X3_08U[] = {{4, 36, 4, 0, {1, 1, 2, 2}}, {4, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MEDIAN_3X3_08U[] = {{6, 36, 4, 0, {1, 1, 2, 2}}, {6, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MEDIAN_3X3_08U[] = {{8, 36, 4, 0, {1, 1, 2, 2}}, {8, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MEDIAN_3X3_08U[] = {{10, 36, 4, 0, {1, 1, 2, 2}}, {10, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MEDIAN_3X3_08U[] = {{12, 36, 4, 0, {1, 1, 2, 2}}, {12, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MEDIAN_3X3_08U[] = {{14, 36, 4, 0, {1, 1, 2, 2}}, {14, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MEDIAN_3X3_08U[] = {{2, 40, 4, 0, {1, 1, 2, 2}}, {2, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MEDIAN_3X3_08U[] = {{4, 40, 4, 0, {1, 1, 2, 2}}, {4, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MEDIAN_3X3_08U[] = {{6, 40, 4, 0, {1, 1, 2, 2}}, {6, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MEDIAN_3X3_08U[] = {{8, 40, 4, 0, {1, 1, 2, 2}}, {8, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MEDIAN_3X3_08U[] = {{10, 40, 4, 0, {1, 1, 2, 2}}, {10, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MEDIAN_3X3_08U[] = {{12, 40, 4, 0, {1, 1, 2, 2}}, {12, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MEDIAN_3X3_08U[] = {{2, 44, 4, 0, {1, 1, 2, 2}}, {2, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MEDIAN_3X3_08U[] = {{4, 44, 4, 0, {1, 1, 2, 2}}, {4, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MEDIAN_3X3_08U[] = {{6, 44, 4, 0, {1, 1, 2, 2}}, {6, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MEDIAN_3X3_08U[] = {{8, 44, 4, 0, {1, 1, 2, 2}}, {8, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MEDIAN_3X3_08U[] = {{10, 44, 4, 0, {1, 1, 2, 2}}, {10, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MEDIAN_3X3_08U[] = {{2, 48, 4, 0, {1, 1, 2, 2}}, {2, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MEDIAN_3X3_08U[] = {{4, 48, 4, 0, {1, 1, 2, 2}}, {4, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MEDIAN_3X3_08U[] = {{6, 48, 4, 0, {1, 1, 2, 2}}, {6, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MEDIAN_3X3_08U[] = {{8, 48, 4, 0, {1, 1, 2, 2}}, {8, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MEDIAN_3X3_08U[] = {{10, 48, 4, 0, {1, 1, 2, 2}}, {10, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MEDIAN_3X3_08U[] = {{2, 52, 4, 0, {1, 1, 2, 2}}, {2, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MEDIAN_3X3_08U[] = {{4, 52, 4, 0, {1, 1, 2, 2}}, {4, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MEDIAN_3X3_08U[] = {{6, 52, 4, 0, {1, 1, 2, 2}}, {6, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MEDIAN_3X3_08U[] = {{8, 52, 4, 0, {1, 1, 2, 2}}, {8, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MEDIAN_3X3_08U[] = {{2, 56, 4, 0, {1, 1, 2, 2}}, {2, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MEDIAN_3X3_08U[] = {{4, 56, 4, 0, {1, 1, 2, 2}}, {4, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MEDIAN_3X3_08U[] = {{6, 56, 4, 0, {1, 1, 2, 2}}, {6, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MEDIAN_3X3_08U[] = {{8, 56, 4, 0, {1, 1, 2, 2}}, {8, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MEDIAN_3X3_08U[] = {{2, 60, 4, 0, {1, 1, 2, 2}}, {2, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MEDIAN_3X3_08U[] = {{4, 60, 4, 0, {1, 1, 2, 2}}, {4, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_MEDIAN_3X3_08U[] = {{6, 60, 4, 0, {1, 1, 2, 2}}, {6, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_MEDIAN_3X3_08U[] = {{2, 64, 4, 0, {1, 1, 2, 2}}, {2, 64, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_MEDIAN_3X3_08U[] = {{4, 64, 4, 0, {1, 1, 2, 2}}, {4, 64, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_MEDIAN_3X3_08U[] = {{6, 64, 4, 0, {1, 1, 2, 2}}, {6, 64, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_MEDIAN_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_MEDIAN_3X3_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_MEDIAN_3X3_08U[] = {
{2, 1, 68, 0, 1, gScenarioBufferData0_MEDIAN_3X3_08U, 32, gScenarioKernelData0_MEDIAN_3X3_08U, 4},
{4, 1, 96, 0, 1, gScenarioBufferData1_MEDIAN_3X3_08U, 32, gScenarioKernelData1_MEDIAN_3X3_08U, 4},
{6, 1, 124, 0, 1, gScenarioBufferData2_MEDIAN_3X3_08U, 32, gScenarioKernelData2_MEDIAN_3X3_08U, 4},
{8, 1, 152, 0, 1, gScenarioBufferData3_MEDIAN_3X3_08U, 32, gScenarioKernelData3_MEDIAN_3X3_08U, 4},
{10, 1, 180, 0, 1, gScenarioBufferData4_MEDIAN_3X3_08U, 32, gScenarioKernelData4_MEDIAN_3X3_08U, 4},
{12, 1, 208, 0, 1, gScenarioBufferData5_MEDIAN_3X3_08U, 32, gScenarioKernelData5_MEDIAN_3X3_08U, 4},
{14, 1, 236, 0, 1, gScenarioBufferData6_MEDIAN_3X3_08U, 32, gScenarioKernelData6_MEDIAN_3X3_08U, 4},
{16, 1, 264, 0, 1, gScenarioBufferData7_MEDIAN_3X3_08U, 32, gScenarioKernelData7_MEDIAN_3X3_08U, 4},
{18, 1, 292, 0, 1, gScenarioBufferData8_MEDIAN_3X3_08U, 32, gScenarioKernelData8_MEDIAN_3X3_08U, 4},
{20, 1, 320, 0, 1, gScenarioBufferData9_MEDIAN_3X3_08U, 32, gScenarioKernelData9_MEDIAN_3X3_08U, 4},
{22, 1, 348, 0, 1, gScenarioBufferData10_MEDIAN_3X3_08U, 32, gScenarioKernelData10_MEDIAN_3X3_08U, 4},
{24, 1, 376, 0, 1, gScenarioBufferData11_MEDIAN_3X3_08U, 32, gScenarioKernelData11_MEDIAN_3X3_08U, 4},
{26, 1, 404, 0, 1, gScenarioBufferData12_MEDIAN_3X3_08U, 32, gScenarioKernelData12_MEDIAN_3X3_08U, 4},
{28, 1, 432, 0, 1, gScenarioBufferData13_MEDIAN_3X3_08U, 32, gScenarioKernelData13_MEDIAN_3X3_08U, 4},
{30, 1, 460, 0, 1, gScenarioBufferData14_MEDIAN_3X3_08U, 32, gScenarioKernelData14_MEDIAN_3X3_08U, 4},
{32, 1, 488, 0, 1, gScenarioBufferData15_MEDIAN_3X3_08U, 32, gScenarioKernelData15_MEDIAN_3X3_08U, 4},
{36, 1, 544, 0, 1, gScenarioBufferData16_MEDIAN_3X3_08U, 32, gScenarioKernelData16_MEDIAN_3X3_08U, 4},
{40, 1, 600, 0, 1, gScenarioBufferData17_MEDIAN_3X3_08U, 32, gScenarioKernelData17_MEDIAN_3X3_08U, 4},
{44, 1, 656, 0, 1, gScenarioBufferData18_MEDIAN_3X3_08U, 32, gScenarioKernelData18_MEDIAN_3X3_08U, 4},
{48, 1, 712, 0, 1, gScenarioBufferData19_MEDIAN_3X3_08U, 32, gScenarioKernelData19_MEDIAN_3X3_08U, 4},
{52, 1, 768, 0, 1, gScenarioBufferData20_MEDIAN_3X3_08U, 32, gScenarioKernelData20_MEDIAN_3X3_08U, 4},
{56, 1, 824, 0, 1, gScenarioBufferData21_MEDIAN_3X3_08U, 32, gScenarioKernelData21_MEDIAN_3X3_08U, 4},
{60, 1, 880, 0, 1, gScenarioBufferData22_MEDIAN_3X3_08U, 32, gScenarioKernelData22_MEDIAN_3X3_08U, 4},
{64, 1, 936, 0, 1, gScenarioBufferData23_MEDIAN_3X3_08U, 32, gScenarioKernelData23_MEDIAN_3X3_08U, 4},
{72, 1, 1048, 0, 1, gScenarioBufferData24_MEDIAN_3X3_08U, 32, gScenarioKernelData24_MEDIAN_3X3_08U, 4},
{80, 1, 1160, 0, 1, gScenarioBufferData25_MEDIAN_3X3_08U, 32, gScenarioKernelData25_MEDIAN_3X3_08U, 4},
{88, 1, 1272, 0, 1, gScenarioBufferData26_MEDIAN_3X3_08U, 32, gScenarioKernelData26_MEDIAN_3X3_08U, 4},
{96, 1, 1384, 0, 1, gScenarioBufferData27_MEDIAN_3X3_08U, 32, gScenarioKernelData27_MEDIAN_3X3_08U, 4},
{104, 1, 1496, 0, 1, gScenarioBufferData28_MEDIAN_3X3_08U, 32, gScenarioKernelData28_MEDIAN_3X3_08U, 4},
{112, 1, 1608, 0, 1, gScenarioBufferData29_MEDIAN_3X3_08U, 32, gScenarioKernelData29_MEDIAN_3X3_08U, 4},
{120, 1, 1720, 0, 1, gScenarioBufferData30_MEDIAN_3X3_08U, 32, gScenarioKernelData30_MEDIAN_3X3_08U, 4},
{128, 1, 1832, 0, 1, gScenarioBufferData31_MEDIAN_3X3_08U, 32, gScenarioKernelData31_MEDIAN_3X3_08U, 4},
{2, 2, 124, 0, 1, gScenarioBufferData32_MEDIAN_3X3_08U, 32, gScenarioKernelData32_MEDIAN_3X3_08U, 4},
{4, 2, 176, 0, 1, gScenarioBufferData33_MEDIAN_3X3_08U, 32, gScenarioKernelData33_MEDIAN_3X3_08U, 4},
{6, 2, 228, 0, 1, gScenarioBufferData34_MEDIAN_3X3_08U, 32, gScenarioKernelData34_MEDIAN_3X3_08U, 4},
{8, 2, 280, 0, 1, gScenarioBufferData35_MEDIAN_3X3_08U, 32, gScenarioKernelData35_MEDIAN_3X3_08U, 4},
{10, 2, 332, 0, 1, gScenarioBufferData36_MEDIAN_3X3_08U, 32, gScenarioKernelData36_MEDIAN_3X3_08U, 4},
{12, 2, 384, 0, 1, gScenarioBufferData37_MEDIAN_3X3_08U, 32, gScenarioKernelData37_MEDIAN_3X3_08U, 4},
{14, 2, 436, 0, 1, gScenarioBufferData38_MEDIAN_3X3_08U, 32, gScenarioKernelData38_MEDIAN_3X3_08U, 4},
{16, 2, 488, 0, 1, gScenarioBufferData39_MEDIAN_3X3_08U, 32, gScenarioKernelData39_MEDIAN_3X3_08U, 4},
{18, 2, 540, 0, 1, gScenarioBufferData40_MEDIAN_3X3_08U, 32, gScenarioKernelData40_MEDIAN_3X3_08U, 4},
{20, 2, 592, 0, 1, gScenarioBufferData41_MEDIAN_3X3_08U, 32, gScenarioKernelData41_MEDIAN_3X3_08U, 4},
{22, 2, 644, 0, 1, gScenarioBufferData42_MEDIAN_3X3_08U, 32, gScenarioKernelData42_MEDIAN_3X3_08U, 4},
{24, 2, 696, 0, 1, gScenarioBufferData43_MEDIAN_3X3_08U, 32, gScenarioKernelData43_MEDIAN_3X3_08U, 4},
{26, 2, 748, 0, 1, gScenarioBufferData44_MEDIAN_3X3_08U, 32, gScenarioKernelData44_MEDIAN_3X3_08U, 4},
{28, 2, 800, 0, 1, gScenarioBufferData45_MEDIAN_3X3_08U, 32, gScenarioKernelData45_MEDIAN_3X3_08U, 4},
{30, 2, 852, 0, 1, gScenarioBufferData46_MEDIAN_3X3_08U, 32, gScenarioKernelData46_MEDIAN_3X3_08U, 4},
{32, 2, 904, 0, 1, gScenarioBufferData47_MEDIAN_3X3_08U, 32, gScenarioKernelData47_MEDIAN_3X3_08U, 4},
{36, 2, 1008, 0, 1, gScenarioBufferData48_MEDIAN_3X3_08U, 32, gScenarioKernelData48_MEDIAN_3X3_08U, 4},
{40, 2, 1112, 0, 1, gScenarioBufferData49_MEDIAN_3X3_08U, 32, gScenarioKernelData49_MEDIAN_3X3_08U, 4},
{44, 2, 1216, 0, 1, gScenarioBufferData50_MEDIAN_3X3_08U, 32, gScenarioKernelData50_MEDIAN_3X3_08U, 4},
{48, 2, 1320, 0, 1, gScenarioBufferData51_MEDIAN_3X3_08U, 32, gScenarioKernelData51_MEDIAN_3X3_08U, 4},
{52, 2, 1424, 0, 1, gScenarioBufferData52_MEDIAN_3X3_08U, 32, gScenarioKernelData52_MEDIAN_3X3_08U, 4},
{56, 2, 1528, 0, 1, gScenarioBufferData53_MEDIAN_3X3_08U, 32, gScenarioKernelData53_MEDIAN_3X3_08U, 4},
{60, 2, 1632, 0, 1, gScenarioBufferData54_MEDIAN_3X3_08U, 32, gScenarioKernelData54_MEDIAN_3X3_08U, 4},
{64, 2, 1736, 0, 1, gScenarioBufferData55_MEDIAN_3X3_08U, 32, gScenarioKernelData55_MEDIAN_3X3_08U, 4},
{72, 2, 1944, 0, 1, gScenarioBufferData56_MEDIAN_3X3_08U, 32, gScenarioKernelData56_MEDIAN_3X3_08U, 4},
{80, 2, 2152, 0, 1, gScenarioBufferData57_MEDIAN_3X3_08U, 32, gScenarioKernelData57_MEDIAN_3X3_08U, 4},
{88, 2, 2360, 0, 1, gScenarioBufferData58_MEDIAN_3X3_08U, 32, gScenarioKernelData58_MEDIAN_3X3_08U, 4},
{96, 2, 2568, 0, 1, gScenarioBufferData59_MEDIAN_3X3_08U, 32, gScenarioKernelData59_MEDIAN_3X3_08U, 4},
{104, 2, 2776, 0, 1, gScenarioBufferData60_MEDIAN_3X3_08U, 32, gScenarioKernelData60_MEDIAN_3X3_08U, 4},
{112, 2, 2984, 0, 1, gScenarioBufferData61_MEDIAN_3X3_08U, 32, gScenarioKernelData61_MEDIAN_3X3_08U, 4},
{120, 2, 3192, 0, 1, gScenarioBufferData62_MEDIAN_3X3_08U, 32, gScenarioKernelData62_MEDIAN_3X3_08U, 4},
{128, 2, 3400, 0, 1, gScenarioBufferData63_MEDIAN_3X3_08U, 32, gScenarioKernelData63_MEDIAN_3X3_08U, 4},
{2, 3, 180, 0, 1, gScenarioBufferData64_MEDIAN_3X3_08U, 32, gScenarioKernelData64_MEDIAN_3X3_08U, 4},
{4, 3, 256, 0, 1, gScenarioBufferData65_MEDIAN_3X3_08U, 32, gScenarioKernelData65_MEDIAN_3X3_08U, 4},
{6, 3, 332, 0, 1, gScenarioBufferData66_MEDIAN_3X3_08U, 32, gScenarioKernelData66_MEDIAN_3X3_08U, 4},
{8, 3, 408, 0, 1, gScenarioBufferData67_MEDIAN_3X3_08U, 32, gScenarioKernelData67_MEDIAN_3X3_08U, 4},
{10, 3, 484, 0, 1, gScenarioBufferData68_MEDIAN_3X3_08U, 32, gScenarioKernelData68_MEDIAN_3X3_08U, 4},
{12, 3, 560, 0, 1, gScenarioBufferData69_MEDIAN_3X3_08U, 32, gScenarioKernelData69_MEDIAN_3X3_08U, 4},
{14, 3, 636, 0, 1, gScenarioBufferData70_MEDIAN_3X3_08U, 32, gScenarioKernelData70_MEDIAN_3X3_08U, 4},
{16, 3, 712, 0, 1, gScenarioBufferData71_MEDIAN_3X3_08U, 32, gScenarioKernelData71_MEDIAN_3X3_08U, 4},
{18, 3, 788, 0, 1, gScenarioBufferData72_MEDIAN_3X3_08U, 32, gScenarioKernelData72_MEDIAN_3X3_08U, 4},
{20, 3, 864, 0, 1, gScenarioBufferData73_MEDIAN_3X3_08U, 32, gScenarioKernelData73_MEDIAN_3X3_08U, 4},
{22, 3, 940, 0, 1, gScenarioBufferData74_MEDIAN_3X3_08U, 32, gScenarioKernelData74_MEDIAN_3X3_08U, 4},
{24, 3, 1016, 0, 1, gScenarioBufferData75_MEDIAN_3X3_08U, 32, gScenarioKernelData75_MEDIAN_3X3_08U, 4},
{26, 3, 1092, 0, 1, gScenarioBufferData76_MEDIAN_3X3_08U, 32, gScenarioKernelData76_MEDIAN_3X3_08U, 4},
{28, 3, 1168, 0, 1, gScenarioBufferData77_MEDIAN_3X3_08U, 32, gScenarioKernelData77_MEDIAN_3X3_08U, 4},
{30, 3, 1244, 0, 1, gScenarioBufferData78_MEDIAN_3X3_08U, 32, gScenarioKernelData78_MEDIAN_3X3_08U, 4},
{32, 3, 1320, 0, 1, gScenarioBufferData79_MEDIAN_3X3_08U, 32, gScenarioKernelData79_MEDIAN_3X3_08U, 4},
{36, 3, 1472, 0, 1, gScenarioBufferData80_MEDIAN_3X3_08U, 32, gScenarioKernelData80_MEDIAN_3X3_08U, 4},
{40, 3, 1624, 0, 1, gScenarioBufferData81_MEDIAN_3X3_08U, 32, gScenarioKernelData81_MEDIAN_3X3_08U, 4},
{44, 3, 1776, 0, 1, gScenarioBufferData82_MEDIAN_3X3_08U, 32, gScenarioKernelData82_MEDIAN_3X3_08U, 4},
{48, 3, 1928, 0, 1, gScenarioBufferData83_MEDIAN_3X3_08U, 32, gScenarioKernelData83_MEDIAN_3X3_08U, 4},
{52, 3, 2080, 0, 1, gScenarioBufferData84_MEDIAN_3X3_08U, 32, gScenarioKernelData84_MEDIAN_3X3_08U, 4},
{56, 3, 2232, 0, 1, gScenarioBufferData85_MEDIAN_3X3_08U, 32, gScenarioKernelData85_MEDIAN_3X3_08U, 4},
{60, 3, 2384, 0, 1, gScenarioBufferData86_MEDIAN_3X3_08U, 32, gScenarioKernelData86_MEDIAN_3X3_08U, 4},
{64, 3, 2536, 0, 1, gScenarioBufferData87_MEDIAN_3X3_08U, 32, gScenarioKernelData87_MEDIAN_3X3_08U, 4},
{72, 3, 2840, 0, 1, gScenarioBufferData88_MEDIAN_3X3_08U, 32, gScenarioKernelData88_MEDIAN_3X3_08U, 4},
{80, 3, 3144, 0, 1, gScenarioBufferData89_MEDIAN_3X3_08U, 32, gScenarioKernelData89_MEDIAN_3X3_08U, 4},
{88, 3, 3448, 0, 1, gScenarioBufferData90_MEDIAN_3X3_08U, 32, gScenarioKernelData90_MEDIAN_3X3_08U, 4},
{96, 3, 3752, 0, 1, gScenarioBufferData91_MEDIAN_3X3_08U, 32, gScenarioKernelData91_MEDIAN_3X3_08U, 4},
{2, 4, 236, 0, 1, gScenarioBufferData92_MEDIAN_3X3_08U, 32, gScenarioKernelData92_MEDIAN_3X3_08U, 4},
{4, 4, 336, 0, 1, gScenarioBufferData93_MEDIAN_3X3_08U, 32, gScenarioKernelData93_MEDIAN_3X3_08U, 4},
{6, 4, 436, 0, 1, gScenarioBufferData94_MEDIAN_3X3_08U, 32, gScenarioKernelData94_MEDIAN_3X3_08U, 4},
{8, 4, 536, 0, 1, gScenarioBufferData95_MEDIAN_3X3_08U, 32, gScenarioKernelData95_MEDIAN_3X3_08U, 4},
{10, 4, 636, 0, 1, gScenarioBufferData96_MEDIAN_3X3_08U, 32, gScenarioKernelData96_MEDIAN_3X3_08U, 4},
{12, 4, 736, 0, 1, gScenarioBufferData97_MEDIAN_3X3_08U, 32, gScenarioKernelData97_MEDIAN_3X3_08U, 4},
{14, 4, 836, 0, 1, gScenarioBufferData98_MEDIAN_3X3_08U, 32, gScenarioKernelData98_MEDIAN_3X3_08U, 4},
{16, 4, 936, 0, 1, gScenarioBufferData99_MEDIAN_3X3_08U, 32, gScenarioKernelData99_MEDIAN_3X3_08U, 4},
{18, 4, 1036, 0, 1, gScenarioBufferData100_MEDIAN_3X3_08U, 32, gScenarioKernelData100_MEDIAN_3X3_08U, 4},
{20, 4, 1136, 0, 1, gScenarioBufferData101_MEDIAN_3X3_08U, 32, gScenarioKernelData101_MEDIAN_3X3_08U, 4},
{22, 4, 1236, 0, 1, gScenarioBufferData102_MEDIAN_3X3_08U, 32, gScenarioKernelData102_MEDIAN_3X3_08U, 4},
{24, 4, 1336, 0, 1, gScenarioBufferData103_MEDIAN_3X3_08U, 32, gScenarioKernelData103_MEDIAN_3X3_08U, 4},
{26, 4, 1436, 0, 1, gScenarioBufferData104_MEDIAN_3X3_08U, 32, gScenarioKernelData104_MEDIAN_3X3_08U, 4},
{28, 4, 1536, 0, 1, gScenarioBufferData105_MEDIAN_3X3_08U, 32, gScenarioKernelData105_MEDIAN_3X3_08U, 4},
{30, 4, 1636, 0, 1, gScenarioBufferData106_MEDIAN_3X3_08U, 32, gScenarioKernelData106_MEDIAN_3X3_08U, 4},
{32, 4, 1736, 0, 1, gScenarioBufferData107_MEDIAN_3X3_08U, 32, gScenarioKernelData107_MEDIAN_3X3_08U, 4},
{36, 4, 1936, 0, 1, gScenarioBufferData108_MEDIAN_3X3_08U, 32, gScenarioKernelData108_MEDIAN_3X3_08U, 4},
{40, 4, 2136, 0, 1, gScenarioBufferData109_MEDIAN_3X3_08U, 32, gScenarioKernelData109_MEDIAN_3X3_08U, 4},
{44, 4, 2336, 0, 1, gScenarioBufferData110_MEDIAN_3X3_08U, 32, gScenarioKernelData110_MEDIAN_3X3_08U, 4},
{48, 4, 2536, 0, 1, gScenarioBufferData111_MEDIAN_3X3_08U, 32, gScenarioKernelData111_MEDIAN_3X3_08U, 4},
{52, 4, 2736, 0, 1, gScenarioBufferData112_MEDIAN_3X3_08U, 32, gScenarioKernelData112_MEDIAN_3X3_08U, 4},
{56, 4, 2936, 0, 1, gScenarioBufferData113_MEDIAN_3X3_08U, 32, gScenarioKernelData113_MEDIAN_3X3_08U, 4},
{60, 4, 3136, 0, 1, gScenarioBufferData114_MEDIAN_3X3_08U, 32, gScenarioKernelData114_MEDIAN_3X3_08U, 4},
{64, 4, 3336, 0, 1, gScenarioBufferData115_MEDIAN_3X3_08U, 32, gScenarioKernelData115_MEDIAN_3X3_08U, 4},
{72, 4, 3736, 0, 1, gScenarioBufferData116_MEDIAN_3X3_08U, 32, gScenarioKernelData116_MEDIAN_3X3_08U, 4},
{2, 5, 292, 0, 1, gScenarioBufferData117_MEDIAN_3X3_08U, 32, gScenarioKernelData117_MEDIAN_3X3_08U, 4},
{4, 5, 416, 0, 1, gScenarioBufferData118_MEDIAN_3X3_08U, 32, gScenarioKernelData118_MEDIAN_3X3_08U, 4},
{6, 5, 540, 0, 1, gScenarioBufferData119_MEDIAN_3X3_08U, 32, gScenarioKernelData119_MEDIAN_3X3_08U, 4},
{8, 5, 664, 0, 1, gScenarioBufferData120_MEDIAN_3X3_08U, 32, gScenarioKernelData120_MEDIAN_3X3_08U, 4},
{10, 5, 788, 0, 1, gScenarioBufferData121_MEDIAN_3X3_08U, 32, gScenarioKernelData121_MEDIAN_3X3_08U, 4},
{12, 5, 912, 0, 1, gScenarioBufferData122_MEDIAN_3X3_08U, 32, gScenarioKernelData122_MEDIAN_3X3_08U, 4},
{14, 5, 1036, 0, 1, gScenarioBufferData123_MEDIAN_3X3_08U, 32, gScenarioKernelData123_MEDIAN_3X3_08U, 4},
{16, 5, 1160, 0, 1, gScenarioBufferData124_MEDIAN_3X3_08U, 32, gScenarioKernelData124_MEDIAN_3X3_08U, 4},
{18, 5, 1284, 0, 1, gScenarioBufferData125_MEDIAN_3X3_08U, 32, gScenarioKernelData125_MEDIAN_3X3_08U, 4},
{20, 5, 1408, 0, 1, gScenarioBufferData126_MEDIAN_3X3_08U, 32, gScenarioKernelData126_MEDIAN_3X3_08U, 4},
{22, 5, 1532, 0, 1, gScenarioBufferData127_MEDIAN_3X3_08U, 32, gScenarioKernelData127_MEDIAN_3X3_08U, 4},
{24, 5, 1656, 0, 1, gScenarioBufferData128_MEDIAN_3X3_08U, 32, gScenarioKernelData128_MEDIAN_3X3_08U, 4},
{26, 5, 1780, 0, 1, gScenarioBufferData129_MEDIAN_3X3_08U, 32, gScenarioKernelData129_MEDIAN_3X3_08U, 4},
{28, 5, 1904, 0, 1, gScenarioBufferData130_MEDIAN_3X3_08U, 32, gScenarioKernelData130_MEDIAN_3X3_08U, 4},
{30, 5, 2028, 0, 1, gScenarioBufferData131_MEDIAN_3X3_08U, 32, gScenarioKernelData131_MEDIAN_3X3_08U, 4},
{32, 5, 2152, 0, 1, gScenarioBufferData132_MEDIAN_3X3_08U, 32, gScenarioKernelData132_MEDIAN_3X3_08U, 4},
{36, 5, 2400, 0, 1, gScenarioBufferData133_MEDIAN_3X3_08U, 32, gScenarioKernelData133_MEDIAN_3X3_08U, 4},
{40, 5, 2648, 0, 1, gScenarioBufferData134_MEDIAN_3X3_08U, 32, gScenarioKernelData134_MEDIAN_3X3_08U, 4},
{44, 5, 2896, 0, 1, gScenarioBufferData135_MEDIAN_3X3_08U, 32, gScenarioKernelData135_MEDIAN_3X3_08U, 4},
{48, 5, 3144, 0, 1, gScenarioBufferData136_MEDIAN_3X3_08U, 32, gScenarioKernelData136_MEDIAN_3X3_08U, 4},
{52, 5, 3392, 0, 1, gScenarioBufferData137_MEDIAN_3X3_08U, 32, gScenarioKernelData137_MEDIAN_3X3_08U, 4},
{56, 5, 3640, 0, 1, gScenarioBufferData138_MEDIAN_3X3_08U, 32, gScenarioKernelData138_MEDIAN_3X3_08U, 4},
{2, 6, 348, 0, 1, gScenarioBufferData139_MEDIAN_3X3_08U, 32, gScenarioKernelData139_MEDIAN_3X3_08U, 4},
{4, 6, 496, 0, 1, gScenarioBufferData140_MEDIAN_3X3_08U, 32, gScenarioKernelData140_MEDIAN_3X3_08U, 4},
{6, 6, 644, 0, 1, gScenarioBufferData141_MEDIAN_3X3_08U, 32, gScenarioKernelData141_MEDIAN_3X3_08U, 4},
{8, 6, 792, 0, 1, gScenarioBufferData142_MEDIAN_3X3_08U, 32, gScenarioKernelData142_MEDIAN_3X3_08U, 4},
{10, 6, 940, 0, 1, gScenarioBufferData143_MEDIAN_3X3_08U, 32, gScenarioKernelData143_MEDIAN_3X3_08U, 4},
{12, 6, 1088, 0, 1, gScenarioBufferData144_MEDIAN_3X3_08U, 32, gScenarioKernelData144_MEDIAN_3X3_08U, 4},
{14, 6, 1236, 0, 1, gScenarioBufferData145_MEDIAN_3X3_08U, 32, gScenarioKernelData145_MEDIAN_3X3_08U, 4},
{16, 6, 1384, 0, 1, gScenarioBufferData146_MEDIAN_3X3_08U, 32, gScenarioKernelData146_MEDIAN_3X3_08U, 4},
{18, 6, 1532, 0, 1, gScenarioBufferData147_MEDIAN_3X3_08U, 32, gScenarioKernelData147_MEDIAN_3X3_08U, 4},
{20, 6, 1680, 0, 1, gScenarioBufferData148_MEDIAN_3X3_08U, 32, gScenarioKernelData148_MEDIAN_3X3_08U, 4},
{22, 6, 1828, 0, 1, gScenarioBufferData149_MEDIAN_3X3_08U, 32, gScenarioKernelData149_MEDIAN_3X3_08U, 4},
{24, 6, 1976, 0, 1, gScenarioBufferData150_MEDIAN_3X3_08U, 32, gScenarioKernelData150_MEDIAN_3X3_08U, 4},
{26, 6, 2124, 0, 1, gScenarioBufferData151_MEDIAN_3X3_08U, 32, gScenarioKernelData151_MEDIAN_3X3_08U, 4},
{28, 6, 2272, 0, 1, gScenarioBufferData152_MEDIAN_3X3_08U, 32, gScenarioKernelData152_MEDIAN_3X3_08U, 4},
{30, 6, 2420, 0, 1, gScenarioBufferData153_MEDIAN_3X3_08U, 32, gScenarioKernelData153_MEDIAN_3X3_08U, 4},
{32, 6, 2568, 0, 1, gScenarioBufferData154_MEDIAN_3X3_08U, 32, gScenarioKernelData154_MEDIAN_3X3_08U, 4},
{36, 6, 2864, 0, 1, gScenarioBufferData155_MEDIAN_3X3_08U, 32, gScenarioKernelData155_MEDIAN_3X3_08U, 4},
{40, 6, 3160, 0, 1, gScenarioBufferData156_MEDIAN_3X3_08U, 32, gScenarioKernelData156_MEDIAN_3X3_08U, 4},
{44, 6, 3456, 0, 1, gScenarioBufferData157_MEDIAN_3X3_08U, 32, gScenarioKernelData157_MEDIAN_3X3_08U, 4},
{48, 6, 3752, 0, 1, gScenarioBufferData158_MEDIAN_3X3_08U, 32, gScenarioKernelData158_MEDIAN_3X3_08U, 4},
{2, 8, 460, 0, 1, gScenarioBufferData159_MEDIAN_3X3_08U, 32, gScenarioKernelData159_MEDIAN_3X3_08U, 4},
{4, 8, 656, 0, 1, gScenarioBufferData160_MEDIAN_3X3_08U, 32, gScenarioKernelData160_MEDIAN_3X3_08U, 4},
{6, 8, 852, 0, 1, gScenarioBufferData161_MEDIAN_3X3_08U, 32, gScenarioKernelData161_MEDIAN_3X3_08U, 4},
{8, 8, 1048, 0, 1, gScenarioBufferData162_MEDIAN_3X3_08U, 32, gScenarioKernelData162_MEDIAN_3X3_08U, 4},
{10, 8, 1244, 0, 1, gScenarioBufferData163_MEDIAN_3X3_08U, 32, gScenarioKernelData163_MEDIAN_3X3_08U, 4},
{12, 8, 1440, 0, 1, gScenarioBufferData164_MEDIAN_3X3_08U, 32, gScenarioKernelData164_MEDIAN_3X3_08U, 4},
{14, 8, 1636, 0, 1, gScenarioBufferData165_MEDIAN_3X3_08U, 32, gScenarioKernelData165_MEDIAN_3X3_08U, 4},
{16, 8, 1832, 0, 1, gScenarioBufferData166_MEDIAN_3X3_08U, 32, gScenarioKernelData166_MEDIAN_3X3_08U, 4},
{18, 8, 2028, 0, 1, gScenarioBufferData167_MEDIAN_3X3_08U, 32, gScenarioKernelData167_MEDIAN_3X3_08U, 4},
{20, 8, 2224, 0, 1, gScenarioBufferData168_MEDIAN_3X3_08U, 32, gScenarioKernelData168_MEDIAN_3X3_08U, 4},
{22, 8, 2420, 0, 1, gScenarioBufferData169_MEDIAN_3X3_08U, 32, gScenarioKernelData169_MEDIAN_3X3_08U, 4},
{24, 8, 2616, 0, 1, gScenarioBufferData170_MEDIAN_3X3_08U, 32, gScenarioKernelData170_MEDIAN_3X3_08U, 4},
{26, 8, 2812, 0, 1, gScenarioBufferData171_MEDIAN_3X3_08U, 32, gScenarioKernelData171_MEDIAN_3X3_08U, 4},
{28, 8, 3008, 0, 1, gScenarioBufferData172_MEDIAN_3X3_08U, 32, gScenarioKernelData172_MEDIAN_3X3_08U, 4},
{30, 8, 3204, 0, 1, gScenarioBufferData173_MEDIAN_3X3_08U, 32, gScenarioKernelData173_MEDIAN_3X3_08U, 4},
{32, 8, 3400, 0, 1, gScenarioBufferData174_MEDIAN_3X3_08U, 32, gScenarioKernelData174_MEDIAN_3X3_08U, 4},
{36, 8, 3792, 0, 1, gScenarioBufferData175_MEDIAN_3X3_08U, 32, gScenarioKernelData175_MEDIAN_3X3_08U, 4},
{2, 10, 572, 0, 1, gScenarioBufferData176_MEDIAN_3X3_08U, 32, gScenarioKernelData176_MEDIAN_3X3_08U, 4},
{4, 10, 816, 0, 1, gScenarioBufferData177_MEDIAN_3X3_08U, 32, gScenarioKernelData177_MEDIAN_3X3_08U, 4},
{6, 10, 1060, 0, 1, gScenarioBufferData178_MEDIAN_3X3_08U, 32, gScenarioKernelData178_MEDIAN_3X3_08U, 4},
{8, 10, 1304, 0, 1, gScenarioBufferData179_MEDIAN_3X3_08U, 32, gScenarioKernelData179_MEDIAN_3X3_08U, 4},
{10, 10, 1548, 0, 1, gScenarioBufferData180_MEDIAN_3X3_08U, 32, gScenarioKernelData180_MEDIAN_3X3_08U, 4},
{12, 10, 1792, 0, 1, gScenarioBufferData181_MEDIAN_3X3_08U, 32, gScenarioKernelData181_MEDIAN_3X3_08U, 4},
{14, 10, 2036, 0, 1, gScenarioBufferData182_MEDIAN_3X3_08U, 32, gScenarioKernelData182_MEDIAN_3X3_08U, 4},
{16, 10, 2280, 0, 1, gScenarioBufferData183_MEDIAN_3X3_08U, 32, gScenarioKernelData183_MEDIAN_3X3_08U, 4},
{18, 10, 2524, 0, 1, gScenarioBufferData184_MEDIAN_3X3_08U, 32, gScenarioKernelData184_MEDIAN_3X3_08U, 4},
{20, 10, 2768, 0, 1, gScenarioBufferData185_MEDIAN_3X3_08U, 32, gScenarioKernelData185_MEDIAN_3X3_08U, 4},
{22, 10, 3012, 0, 1, gScenarioBufferData186_MEDIAN_3X3_08U, 32, gScenarioKernelData186_MEDIAN_3X3_08U, 4},
{24, 10, 3256, 0, 1, gScenarioBufferData187_MEDIAN_3X3_08U, 32, gScenarioKernelData187_MEDIAN_3X3_08U, 4},
{26, 10, 3500, 0, 1, gScenarioBufferData188_MEDIAN_3X3_08U, 32, gScenarioKernelData188_MEDIAN_3X3_08U, 4},
{28, 10, 3744, 0, 1, gScenarioBufferData189_MEDIAN_3X3_08U, 32, gScenarioKernelData189_MEDIAN_3X3_08U, 4},
{2, 12, 684, 0, 1, gScenarioBufferData190_MEDIAN_3X3_08U, 32, gScenarioKernelData190_MEDIAN_3X3_08U, 4},
{4, 12, 976, 0, 1, gScenarioBufferData191_MEDIAN_3X3_08U, 32, gScenarioKernelData191_MEDIAN_3X3_08U, 4},
{6, 12, 1268, 0, 1, gScenarioBufferData192_MEDIAN_3X3_08U, 32, gScenarioKernelData192_MEDIAN_3X3_08U, 4},
{8, 12, 1560, 0, 1, gScenarioBufferData193_MEDIAN_3X3_08U, 32, gScenarioKernelData193_MEDIAN_3X3_08U, 4},
{10, 12, 1852, 0, 1, gScenarioBufferData194_MEDIAN_3X3_08U, 32, gScenarioKernelData194_MEDIAN_3X3_08U, 4},
{12, 12, 2144, 0, 1, gScenarioBufferData195_MEDIAN_3X3_08U, 32, gScenarioKernelData195_MEDIAN_3X3_08U, 4},
{14, 12, 2436, 0, 1, gScenarioBufferData196_MEDIAN_3X3_08U, 32, gScenarioKernelData196_MEDIAN_3X3_08U, 4},
{16, 12, 2728, 0, 1, gScenarioBufferData197_MEDIAN_3X3_08U, 32, gScenarioKernelData197_MEDIAN_3X3_08U, 4},
{18, 12, 3020, 0, 1, gScenarioBufferData198_MEDIAN_3X3_08U, 32, gScenarioKernelData198_MEDIAN_3X3_08U, 4},
{20, 12, 3312, 0, 1, gScenarioBufferData199_MEDIAN_3X3_08U, 32, gScenarioKernelData199_MEDIAN_3X3_08U, 4},
{22, 12, 3604, 0, 1, gScenarioBufferData200_MEDIAN_3X3_08U, 32, gScenarioKernelData200_MEDIAN_3X3_08U, 4},
{2, 14, 796, 0, 1, gScenarioBufferData201_MEDIAN_3X3_08U, 32, gScenarioKernelData201_MEDIAN_3X3_08U, 4},
{4, 14, 1136, 0, 1, gScenarioBufferData202_MEDIAN_3X3_08U, 32, gScenarioKernelData202_MEDIAN_3X3_08U, 4},
{6, 14, 1476, 0, 1, gScenarioBufferData203_MEDIAN_3X3_08U, 32, gScenarioKernelData203_MEDIAN_3X3_08U, 4},
{8, 14, 1816, 0, 1, gScenarioBufferData204_MEDIAN_3X3_08U, 32, gScenarioKernelData204_MEDIAN_3X3_08U, 4},
{10, 14, 2156, 0, 1, gScenarioBufferData205_MEDIAN_3X3_08U, 32, gScenarioKernelData205_MEDIAN_3X3_08U, 4},
{12, 14, 2496, 0, 1, gScenarioBufferData206_MEDIAN_3X3_08U, 32, gScenarioKernelData206_MEDIAN_3X3_08U, 4},
{14, 14, 2836, 0, 1, gScenarioBufferData207_MEDIAN_3X3_08U, 32, gScenarioKernelData207_MEDIAN_3X3_08U, 4},
{16, 14, 3176, 0, 1, gScenarioBufferData208_MEDIAN_3X3_08U, 32, gScenarioKernelData208_MEDIAN_3X3_08U, 4},
{18, 14, 3516, 0, 1, gScenarioBufferData209_MEDIAN_3X3_08U, 32, gScenarioKernelData209_MEDIAN_3X3_08U, 4},
{2, 16, 908, 0, 1, gScenarioBufferData210_MEDIAN_3X3_08U, 32, gScenarioKernelData210_MEDIAN_3X3_08U, 4},
{4, 16, 1296, 0, 1, gScenarioBufferData211_MEDIAN_3X3_08U, 32, gScenarioKernelData211_MEDIAN_3X3_08U, 4},
{6, 16, 1684, 0, 1, gScenarioBufferData212_MEDIAN_3X3_08U, 32, gScenarioKernelData212_MEDIAN_3X3_08U, 4},
{8, 16, 2072, 0, 1, gScenarioBufferData213_MEDIAN_3X3_08U, 32, gScenarioKernelData213_MEDIAN_3X3_08U, 4},
{10, 16, 2460, 0, 1, gScenarioBufferData214_MEDIAN_3X3_08U, 32, gScenarioKernelData214_MEDIAN_3X3_08U, 4},
{12, 16, 2848, 0, 1, gScenarioBufferData215_MEDIAN_3X3_08U, 32, gScenarioKernelData215_MEDIAN_3X3_08U, 4},
{14, 16, 3236, 0, 1, gScenarioBufferData216_MEDIAN_3X3_08U, 32, gScenarioKernelData216_MEDIAN_3X3_08U, 4},
{16, 16, 3624, 0, 1, gScenarioBufferData217_MEDIAN_3X3_08U, 32, gScenarioKernelData217_MEDIAN_3X3_08U, 4},
{2, 18, 1020, 0, 1, gScenarioBufferData218_MEDIAN_3X3_08U, 32, gScenarioKernelData218_MEDIAN_3X3_08U, 4},
{4, 18, 1456, 0, 1, gScenarioBufferData219_MEDIAN_3X3_08U, 32, gScenarioKernelData219_MEDIAN_3X3_08U, 4},
{6, 18, 1892, 0, 1, gScenarioBufferData220_MEDIAN_3X3_08U, 32, gScenarioKernelData220_MEDIAN_3X3_08U, 4},
{8, 18, 2328, 0, 1, gScenarioBufferData221_MEDIAN_3X3_08U, 32, gScenarioKernelData221_MEDIAN_3X3_08U, 4},
{10, 18, 2764, 0, 1, gScenarioBufferData222_MEDIAN_3X3_08U, 32, gScenarioKernelData222_MEDIAN_3X3_08U, 4},
{12, 18, 3200, 0, 1, gScenarioBufferData223_MEDIAN_3X3_08U, 32, gScenarioKernelData223_MEDIAN_3X3_08U, 4},
{14, 18, 3636, 0, 1, gScenarioBufferData224_MEDIAN_3X3_08U, 32, gScenarioKernelData224_MEDIAN_3X3_08U, 4},
{2, 20, 1132, 0, 1, gScenarioBufferData225_MEDIAN_3X3_08U, 32, gScenarioKernelData225_MEDIAN_3X3_08U, 4},
{4, 20, 1616, 0, 1, gScenarioBufferData226_MEDIAN_3X3_08U, 32, gScenarioKernelData226_MEDIAN_3X3_08U, 4},
{6, 20, 2100, 0, 1, gScenarioBufferData227_MEDIAN_3X3_08U, 32, gScenarioKernelData227_MEDIAN_3X3_08U, 4},
{8, 20, 2584, 0, 1, gScenarioBufferData228_MEDIAN_3X3_08U, 32, gScenarioKernelData228_MEDIAN_3X3_08U, 4},
{10, 20, 3068, 0, 1, gScenarioBufferData229_MEDIAN_3X3_08U, 32, gScenarioKernelData229_MEDIAN_3X3_08U, 4},
{12, 20, 3552, 0, 1, gScenarioBufferData230_MEDIAN_3X3_08U, 32, gScenarioKernelData230_MEDIAN_3X3_08U, 4},
{2, 22, 1244, 0, 1, gScenarioBufferData231_MEDIAN_3X3_08U, 32, gScenarioKernelData231_MEDIAN_3X3_08U, 4},
{4, 22, 1776, 0, 1, gScenarioBufferData232_MEDIAN_3X3_08U, 32, gScenarioKernelData232_MEDIAN_3X3_08U, 4},
{6, 22, 2308, 0, 1, gScenarioBufferData233_MEDIAN_3X3_08U, 32, gScenarioKernelData233_MEDIAN_3X3_08U, 4},
{8, 22, 2840, 0, 1, gScenarioBufferData234_MEDIAN_3X3_08U, 32, gScenarioKernelData234_MEDIAN_3X3_08U, 4},
{10, 22, 3372, 0, 1, gScenarioBufferData235_MEDIAN_3X3_08U, 32, gScenarioKernelData235_MEDIAN_3X3_08U, 4},
{2, 24, 1356, 0, 1, gScenarioBufferData236_MEDIAN_3X3_08U, 32, gScenarioKernelData236_MEDIAN_3X3_08U, 4},
{4, 24, 1936, 0, 1, gScenarioBufferData237_MEDIAN_3X3_08U, 32, gScenarioKernelData237_MEDIAN_3X3_08U, 4},
{6, 24, 2516, 0, 1, gScenarioBufferData238_MEDIAN_3X3_08U, 32, gScenarioKernelData238_MEDIAN_3X3_08U, 4},
{8, 24, 3096, 0, 1, gScenarioBufferData239_MEDIAN_3X3_08U, 32, gScenarioKernelData239_MEDIAN_3X3_08U, 4},
{10, 24, 3676, 0, 1, gScenarioBufferData240_MEDIAN_3X3_08U, 32, gScenarioKernelData240_MEDIAN_3X3_08U, 4},
{2, 26, 1468, 0, 1, gScenarioBufferData241_MEDIAN_3X3_08U, 32, gScenarioKernelData241_MEDIAN_3X3_08U, 4},
{4, 26, 2096, 0, 1, gScenarioBufferData242_MEDIAN_3X3_08U, 32, gScenarioKernelData242_MEDIAN_3X3_08U, 4},
{6, 26, 2724, 0, 1, gScenarioBufferData243_MEDIAN_3X3_08U, 32, gScenarioKernelData243_MEDIAN_3X3_08U, 4},
{8, 26, 3352, 0, 1, gScenarioBufferData244_MEDIAN_3X3_08U, 32, gScenarioKernelData244_MEDIAN_3X3_08U, 4},
{2, 28, 1580, 0, 1, gScenarioBufferData245_MEDIAN_3X3_08U, 32, gScenarioKernelData245_MEDIAN_3X3_08U, 4},
{4, 28, 2256, 0, 1, gScenarioBufferData246_MEDIAN_3X3_08U, 32, gScenarioKernelData246_MEDIAN_3X3_08U, 4},
{6, 28, 2932, 0, 1, gScenarioBufferData247_MEDIAN_3X3_08U, 32, gScenarioKernelData247_MEDIAN_3X3_08U, 4},
{8, 28, 3608, 0, 1, gScenarioBufferData248_MEDIAN_3X3_08U, 32, gScenarioKernelData248_MEDIAN_3X3_08U, 4},
{2, 30, 1692, 0, 1, gScenarioBufferData249_MEDIAN_3X3_08U, 32, gScenarioKernelData249_MEDIAN_3X3_08U, 4},
{4, 30, 2416, 0, 1, gScenarioBufferData250_MEDIAN_3X3_08U, 32, gScenarioKernelData250_MEDIAN_3X3_08U, 4},
{6, 30, 3140, 0, 1, gScenarioBufferData251_MEDIAN_3X3_08U, 32, gScenarioKernelData251_MEDIAN_3X3_08U, 4},
{2, 32, 1804, 0, 1, gScenarioBufferData252_MEDIAN_3X3_08U, 32, gScenarioKernelData252_MEDIAN_3X3_08U, 4},
{4, 32, 2576, 0, 1, gScenarioBufferData253_MEDIAN_3X3_08U, 32, gScenarioKernelData253_MEDIAN_3X3_08U, 4},
{6, 32, 3348, 0, 1, gScenarioBufferData254_MEDIAN_3X3_08U, 32, gScenarioKernelData254_MEDIAN_3X3_08U, 4}
};

static acf_scenario_list gScenarioList_MEDIAN_3X3_08U = {
255, //number of scenarios
gScenarioArray_MEDIAN_3X3_08U};
//**************************************************************

class MEDIAN_3X3_08U : public ACF_Process_APU
{

public:
   MEDIAN_3X3_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MEDIAN_3X3_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MEDIAN_3X3_08U",
                                        MEDIAN_3X3_08U_LOAD_SEGMENTS,
                                        MEDIAN_3X3_08U_LOAD_PMEM, MEDIAN_3X3_08U_LOAD_PMEM_SIZE,
                                        MEDIAN_3X3_08U_LOAD_DMEM, MEDIAN_3X3_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MEDIAN_3X3_08U_APEX_LOG_BUFFER, MEDIAN_3X3_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MEDIAN_3X3_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MEDIAN_3X3_08U

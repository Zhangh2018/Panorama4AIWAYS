#ifndef _ACF_PROCESS_APU_SCHARR_Y_5X5_08U
#define _ACF_PROCESS_APU_SCHARR_Y_5X5_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <SCHARR_Y_5X5_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_SCHARR_Y_5X5_08U[] = {{2, 1, 7, 0, {2, 2, 2, 2}}, {2, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_SCHARR_Y_5X5_08U[] = {{4, 1, 7, 0, {2, 2, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_SCHARR_Y_5X5_08U[] = {{6, 1, 7, 0, {2, 2, 2, 2}}, {6, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_SCHARR_Y_5X5_08U[] = {{8, 1, 7, 0, {2, 2, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_SCHARR_Y_5X5_08U[] = {{10, 1, 7, 0, {2, 2, 2, 2}}, {10, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_SCHARR_Y_5X5_08U[] = {{12, 1, 7, 0, {2, 2, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_SCHARR_Y_5X5_08U[] = {{14, 1, 7, 0, {2, 2, 2, 2}}, {14, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_SCHARR_Y_5X5_08U[] = {{16, 1, 7, 0, {2, 2, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_SCHARR_Y_5X5_08U[] = {{18, 1, 7, 0, {2, 2, 2, 2}}, {18, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_SCHARR_Y_5X5_08U[] = {{20, 1, 7, 0, {2, 2, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_SCHARR_Y_5X5_08U[] = {{22, 1, 7, 0, {2, 2, 2, 2}}, {22, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_SCHARR_Y_5X5_08U[] = {{24, 1, 7, 0, {2, 2, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_SCHARR_Y_5X5_08U[] = {{26, 1, 7, 0, {2, 2, 2, 2}}, {26, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_SCHARR_Y_5X5_08U[] = {{28, 1, 7, 0, {2, 2, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_SCHARR_Y_5X5_08U[] = {{30, 1, 7, 0, {2, 2, 2, 2}}, {30, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_SCHARR_Y_5X5_08U[] = {{32, 1, 7, 0, {2, 2, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_SCHARR_Y_5X5_08U[] = {{36, 1, 7, 0, {2, 2, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_SCHARR_Y_5X5_08U[] = {{40, 1, 7, 0, {2, 2, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_SCHARR_Y_5X5_08U[] = {{44, 1, 7, 0, {2, 2, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_SCHARR_Y_5X5_08U[] = {{48, 1, 7, 0, {2, 2, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_SCHARR_Y_5X5_08U[] = {{52, 1, 7, 0, {2, 2, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_SCHARR_Y_5X5_08U[] = {{56, 1, 7, 0, {2, 2, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_SCHARR_Y_5X5_08U[] = {{60, 1, 7, 0, {2, 2, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_SCHARR_Y_5X5_08U[] = {{64, 1, 7, 0, {2, 2, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_SCHARR_Y_5X5_08U[] = {{2, 2, 4, 0, {2, 2, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_SCHARR_Y_5X5_08U[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_SCHARR_Y_5X5_08U[] = {{6, 2, 4, 0, {2, 2, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_SCHARR_Y_5X5_08U[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_SCHARR_Y_5X5_08U[] = {{10, 2, 4, 0, {2, 2, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_SCHARR_Y_5X5_08U[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_SCHARR_Y_5X5_08U[] = {{14, 2, 4, 0, {2, 2, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_SCHARR_Y_5X5_08U[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_SCHARR_Y_5X5_08U[] = {{18, 2, 4, 0, {2, 2, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_SCHARR_Y_5X5_08U[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_SCHARR_Y_5X5_08U[] = {{22, 2, 4, 0, {2, 2, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_SCHARR_Y_5X5_08U[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_SCHARR_Y_5X5_08U[] = {{26, 2, 4, 0, {2, 2, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_SCHARR_Y_5X5_08U[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_SCHARR_Y_5X5_08U[] = {{30, 2, 4, 0, {2, 2, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_SCHARR_Y_5X5_08U[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_SCHARR_Y_5X5_08U[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_SCHARR_Y_5X5_08U[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_SCHARR_Y_5X5_08U[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_SCHARR_Y_5X5_08U[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_SCHARR_Y_5X5_08U[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_SCHARR_Y_5X5_08U[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_SCHARR_Y_5X5_08U[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_SCHARR_Y_5X5_08U[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_SCHARR_Y_5X5_08U[] = {{2, 3, 4, 0, {2, 2, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_SCHARR_Y_5X5_08U[] = {{4, 3, 4, 0, {2, 2, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_SCHARR_Y_5X5_08U[] = {{6, 3, 4, 0, {2, 2, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_SCHARR_Y_5X5_08U[] = {{8, 3, 4, 0, {2, 2, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_SCHARR_Y_5X5_08U[] = {{10, 3, 4, 0, {2, 2, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_SCHARR_Y_5X5_08U[] = {{12, 3, 4, 0, {2, 2, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_SCHARR_Y_5X5_08U[] = {{14, 3, 4, 0, {2, 2, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_SCHARR_Y_5X5_08U[] = {{16, 3, 4, 0, {2, 2, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_SCHARR_Y_5X5_08U[] = {{18, 3, 4, 0, {2, 2, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_SCHARR_Y_5X5_08U[] = {{20, 3, 4, 0, {2, 2, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_SCHARR_Y_5X5_08U[] = {{22, 3, 4, 0, {2, 2, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_SCHARR_Y_5X5_08U[] = {{24, 3, 4, 0, {2, 2, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_SCHARR_Y_5X5_08U[] = {{26, 3, 4, 0, {2, 2, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_SCHARR_Y_5X5_08U[] = {{28, 3, 4, 0, {2, 2, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_SCHARR_Y_5X5_08U[] = {{30, 3, 4, 0, {2, 2, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_SCHARR_Y_5X5_08U[] = {{32, 3, 4, 0, {2, 2, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_SCHARR_Y_5X5_08U[] = {{36, 3, 4, 0, {2, 2, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_SCHARR_Y_5X5_08U[] = {{40, 3, 4, 0, {2, 2, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_SCHARR_Y_5X5_08U[] = {{44, 3, 4, 0, {2, 2, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_SCHARR_Y_5X5_08U[] = {{48, 3, 4, 0, {2, 2, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_SCHARR_Y_5X5_08U[] = {{52, 3, 4, 0, {2, 2, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_SCHARR_Y_5X5_08U[] = {{56, 3, 4, 0, {2, 2, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_SCHARR_Y_5X5_08U[] = {{60, 3, 4, 0, {2, 2, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_SCHARR_Y_5X5_08U[] = {{64, 3, 4, 0, {2, 2, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_SCHARR_Y_5X5_08U[] = {{2, 4, 4, 0, {2, 2, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_SCHARR_Y_5X5_08U[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_SCHARR_Y_5X5_08U[] = {{6, 4, 4, 0, {2, 2, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_SCHARR_Y_5X5_08U[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_SCHARR_Y_5X5_08U[] = {{10, 4, 4, 0, {2, 2, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_SCHARR_Y_5X5_08U[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_SCHARR_Y_5X5_08U[] = {{14, 4, 4, 0, {2, 2, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_SCHARR_Y_5X5_08U[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_SCHARR_Y_5X5_08U[] = {{18, 4, 4, 0, {2, 2, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_SCHARR_Y_5X5_08U[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_SCHARR_Y_5X5_08U[] = {{22, 4, 4, 0, {2, 2, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_SCHARR_Y_5X5_08U[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_SCHARR_Y_5X5_08U[] = {{26, 4, 4, 0, {2, 2, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_SCHARR_Y_5X5_08U[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_SCHARR_Y_5X5_08U[] = {{30, 4, 4, 0, {2, 2, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_SCHARR_Y_5X5_08U[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_SCHARR_Y_5X5_08U[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_SCHARR_Y_5X5_08U[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_SCHARR_Y_5X5_08U[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_SCHARR_Y_5X5_08U[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_SCHARR_Y_5X5_08U[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_SCHARR_Y_5X5_08U[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_SCHARR_Y_5X5_08U[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_SCHARR_Y_5X5_08U[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_SCHARR_Y_5X5_08U[] = {{2, 5, 4, 0, {2, 2, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_SCHARR_Y_5X5_08U[] = {{4, 5, 4, 0, {2, 2, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_SCHARR_Y_5X5_08U[] = {{6, 5, 4, 0, {2, 2, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_SCHARR_Y_5X5_08U[] = {{8, 5, 4, 0, {2, 2, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_SCHARR_Y_5X5_08U[] = {{10, 5, 4, 0, {2, 2, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_SCHARR_Y_5X5_08U[] = {{12, 5, 4, 0, {2, 2, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_SCHARR_Y_5X5_08U[] = {{14, 5, 4, 0, {2, 2, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_SCHARR_Y_5X5_08U[] = {{16, 5, 4, 0, {2, 2, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_SCHARR_Y_5X5_08U[] = {{18, 5, 4, 0, {2, 2, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_SCHARR_Y_5X5_08U[] = {{20, 5, 4, 0, {2, 2, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_SCHARR_Y_5X5_08U[] = {{22, 5, 4, 0, {2, 2, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_SCHARR_Y_5X5_08U[] = {{24, 5, 4, 0, {2, 2, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_SCHARR_Y_5X5_08U[] = {{26, 5, 4, 0, {2, 2, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_SCHARR_Y_5X5_08U[] = {{28, 5, 4, 0, {2, 2, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_SCHARR_Y_5X5_08U[] = {{30, 5, 4, 0, {2, 2, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_SCHARR_Y_5X5_08U[] = {{32, 5, 4, 0, {2, 2, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_SCHARR_Y_5X5_08U[] = {{36, 5, 4, 0, {2, 2, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_SCHARR_Y_5X5_08U[] = {{40, 5, 4, 0, {2, 2, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_SCHARR_Y_5X5_08U[] = {{44, 5, 4, 0, {2, 2, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_SCHARR_Y_5X5_08U[] = {{48, 5, 4, 0, {2, 2, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_SCHARR_Y_5X5_08U[] = {{52, 5, 4, 0, {2, 2, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_SCHARR_Y_5X5_08U[] = {{56, 5, 4, 0, {2, 2, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_SCHARR_Y_5X5_08U[] = {{60, 5, 4, 0, {2, 2, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_SCHARR_Y_5X5_08U[] = {{64, 5, 4, 0, {2, 2, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_SCHARR_Y_5X5_08U[] = {{2, 6, 4, 0, {2, 2, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_SCHARR_Y_5X5_08U[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_SCHARR_Y_5X5_08U[] = {{6, 6, 4, 0, {2, 2, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_SCHARR_Y_5X5_08U[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_SCHARR_Y_5X5_08U[] = {{10, 6, 4, 0, {2, 2, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_SCHARR_Y_5X5_08U[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_SCHARR_Y_5X5_08U[] = {{14, 6, 4, 0, {2, 2, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_SCHARR_Y_5X5_08U[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_SCHARR_Y_5X5_08U[] = {{18, 6, 4, 0, {2, 2, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_SCHARR_Y_5X5_08U[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_SCHARR_Y_5X5_08U[] = {{22, 6, 4, 0, {2, 2, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_SCHARR_Y_5X5_08U[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_SCHARR_Y_5X5_08U[] = {{26, 6, 4, 0, {2, 2, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_SCHARR_Y_5X5_08U[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_SCHARR_Y_5X5_08U[] = {{30, 6, 4, 0, {2, 2, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_SCHARR_Y_5X5_08U[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_SCHARR_Y_5X5_08U[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_SCHARR_Y_5X5_08U[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_SCHARR_Y_5X5_08U[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_SCHARR_Y_5X5_08U[] = {{48, 6, 4, 0, {2, 2, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_SCHARR_Y_5X5_08U[] = {{52, 6, 4, 0, {2, 2, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_SCHARR_Y_5X5_08U[] = {{56, 6, 4, 0, {2, 2, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_SCHARR_Y_5X5_08U[] = {{60, 6, 4, 0, {2, 2, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_SCHARR_Y_5X5_08U[] = {{64, 6, 4, 0, {2, 2, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_SCHARR_Y_5X5_08U[] = {{2, 8, 4, 0, {2, 2, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_SCHARR_Y_5X5_08U[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_SCHARR_Y_5X5_08U[] = {{6, 8, 4, 0, {2, 2, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_SCHARR_Y_5X5_08U[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_SCHARR_Y_5X5_08U[] = {{10, 8, 4, 0, {2, 2, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_SCHARR_Y_5X5_08U[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_SCHARR_Y_5X5_08U[] = {{14, 8, 4, 0, {2, 2, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_SCHARR_Y_5X5_08U[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_SCHARR_Y_5X5_08U[] = {{18, 8, 4, 0, {2, 2, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_SCHARR_Y_5X5_08U[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_SCHARR_Y_5X5_08U[] = {{22, 8, 4, 0, {2, 2, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_SCHARR_Y_5X5_08U[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_SCHARR_Y_5X5_08U[] = {{26, 8, 4, 0, {2, 2, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_SCHARR_Y_5X5_08U[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_SCHARR_Y_5X5_08U[] = {{30, 8, 4, 0, {2, 2, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_SCHARR_Y_5X5_08U[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_SCHARR_Y_5X5_08U[] = {{36, 8, 4, 0, {2, 2, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_SCHARR_Y_5X5_08U[] = {{40, 8, 4, 0, {2, 2, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_SCHARR_Y_5X5_08U[] = {{44, 8, 4, 0, {2, 2, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_SCHARR_Y_5X5_08U[] = {{48, 8, 4, 0, {2, 2, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_SCHARR_Y_5X5_08U[] = {{52, 8, 4, 0, {2, 2, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_SCHARR_Y_5X5_08U[] = {{2, 10, 4, 0, {2, 2, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_SCHARR_Y_5X5_08U[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_SCHARR_Y_5X5_08U[] = {{6, 10, 4, 0, {2, 2, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_SCHARR_Y_5X5_08U[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_SCHARR_Y_5X5_08U[] = {{10, 10, 4, 0, {2, 2, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_SCHARR_Y_5X5_08U[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_SCHARR_Y_5X5_08U[] = {{14, 10, 4, 0, {2, 2, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_SCHARR_Y_5X5_08U[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_SCHARR_Y_5X5_08U[] = {{18, 10, 4, 0, {2, 2, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_SCHARR_Y_5X5_08U[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_SCHARR_Y_5X5_08U[] = {{22, 10, 4, 0, {2, 2, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_SCHARR_Y_5X5_08U[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_SCHARR_Y_5X5_08U[] = {{26, 10, 4, 0, {2, 2, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_SCHARR_Y_5X5_08U[] = {{28, 10, 4, 0, {2, 2, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_SCHARR_Y_5X5_08U[] = {{30, 10, 4, 0, {2, 2, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_SCHARR_Y_5X5_08U[] = {{32, 10, 4, 0, {2, 2, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_SCHARR_Y_5X5_08U[] = {{36, 10, 4, 0, {2, 2, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_SCHARR_Y_5X5_08U[] = {{40, 10, 4, 0, {2, 2, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_SCHARR_Y_5X5_08U[] = {{2, 12, 4, 0, {2, 2, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_SCHARR_Y_5X5_08U[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_SCHARR_Y_5X5_08U[] = {{6, 12, 4, 0, {2, 2, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_SCHARR_Y_5X5_08U[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_SCHARR_Y_5X5_08U[] = {{10, 12, 4, 0, {2, 2, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_SCHARR_Y_5X5_08U[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_SCHARR_Y_5X5_08U[] = {{14, 12, 4, 0, {2, 2, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_SCHARR_Y_5X5_08U[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_SCHARR_Y_5X5_08U[] = {{18, 12, 4, 0, {2, 2, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_SCHARR_Y_5X5_08U[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_SCHARR_Y_5X5_08U[] = {{22, 12, 4, 0, {2, 2, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_SCHARR_Y_5X5_08U[] = {{24, 12, 4, 0, {2, 2, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_SCHARR_Y_5X5_08U[] = {{26, 12, 4, 0, {2, 2, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_SCHARR_Y_5X5_08U[] = {{28, 12, 4, 0, {2, 2, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_SCHARR_Y_5X5_08U[] = {{30, 12, 4, 0, {2, 2, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_SCHARR_Y_5X5_08U[] = {{32, 12, 4, 0, {2, 2, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_SCHARR_Y_5X5_08U[] = {{36, 12, 4, 0, {2, 2, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_SCHARR_Y_5X5_08U[] = {{2, 14, 4, 0, {2, 2, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_SCHARR_Y_5X5_08U[] = {{4, 14, 4, 0, {2, 2, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_SCHARR_Y_5X5_08U[] = {{6, 14, 4, 0, {2, 2, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_SCHARR_Y_5X5_08U[] = {{8, 14, 4, 0, {2, 2, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_SCHARR_Y_5X5_08U[] = {{10, 14, 4, 0, {2, 2, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_SCHARR_Y_5X5_08U[] = {{12, 14, 4, 0, {2, 2, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_SCHARR_Y_5X5_08U[] = {{14, 14, 4, 0, {2, 2, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_SCHARR_Y_5X5_08U[] = {{16, 14, 4, 0, {2, 2, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_SCHARR_Y_5X5_08U[] = {{18, 14, 4, 0, {2, 2, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_SCHARR_Y_5X5_08U[] = {{20, 14, 4, 0, {2, 2, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_SCHARR_Y_5X5_08U[] = {{22, 14, 4, 0, {2, 2, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_SCHARR_Y_5X5_08U[] = {{24, 14, 4, 0, {2, 2, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_SCHARR_Y_5X5_08U[] = {{26, 14, 4, 0, {2, 2, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_SCHARR_Y_5X5_08U[] = {{28, 14, 4, 0, {2, 2, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_SCHARR_Y_5X5_08U[] = {{30, 14, 4, 0, {2, 2, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_SCHARR_Y_5X5_08U[] = {{2, 16, 4, 0, {2, 2, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_SCHARR_Y_5X5_08U[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_SCHARR_Y_5X5_08U[] = {{6, 16, 4, 0, {2, 2, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_SCHARR_Y_5X5_08U[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_SCHARR_Y_5X5_08U[] = {{10, 16, 4, 0, {2, 2, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_SCHARR_Y_5X5_08U[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_SCHARR_Y_5X5_08U[] = {{14, 16, 4, 0, {2, 2, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_SCHARR_Y_5X5_08U[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_SCHARR_Y_5X5_08U[] = {{18, 16, 4, 0, {2, 2, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_SCHARR_Y_5X5_08U[] = {{20, 16, 4, 0, {2, 2, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_SCHARR_Y_5X5_08U[] = {{22, 16, 4, 0, {2, 2, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_SCHARR_Y_5X5_08U[] = {{24, 16, 4, 0, {2, 2, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_SCHARR_Y_5X5_08U[] = {{26, 16, 4, 0, {2, 2, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_SCHARR_Y_5X5_08U[] = {{2, 18, 4, 0, {2, 2, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_SCHARR_Y_5X5_08U[] = {{4, 18, 4, 0, {2, 2, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_SCHARR_Y_5X5_08U[] = {{6, 18, 4, 0, {2, 2, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_SCHARR_Y_5X5_08U[] = {{8, 18, 4, 0, {2, 2, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_SCHARR_Y_5X5_08U[] = {{10, 18, 4, 0, {2, 2, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_SCHARR_Y_5X5_08U[] = {{12, 18, 4, 0, {2, 2, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_SCHARR_Y_5X5_08U[] = {{14, 18, 4, 0, {2, 2, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_SCHARR_Y_5X5_08U[] = {{16, 18, 4, 0, {2, 2, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_SCHARR_Y_5X5_08U[] = {{18, 18, 4, 0, {2, 2, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_SCHARR_Y_5X5_08U[] = {{20, 18, 4, 0, {2, 2, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_SCHARR_Y_5X5_08U[] = {{22, 18, 4, 0, {2, 2, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_SCHARR_Y_5X5_08U[] = {{2, 20, 4, 0, {2, 2, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_SCHARR_Y_5X5_08U[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_SCHARR_Y_5X5_08U[] = {{6, 20, 4, 0, {2, 2, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_SCHARR_Y_5X5_08U[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_SCHARR_Y_5X5_08U[] = {{10, 20, 4, 0, {2, 2, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_SCHARR_Y_5X5_08U[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_SCHARR_Y_5X5_08U[] = {{14, 20, 4, 0, {2, 2, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_SCHARR_Y_5X5_08U[] = {{16, 20, 4, 0, {2, 2, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_SCHARR_Y_5X5_08U[] = {{18, 20, 4, 0, {2, 2, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_SCHARR_Y_5X5_08U[] = {{20, 20, 4, 0, {2, 2, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_SCHARR_Y_5X5_08U[] = {{2, 22, 4, 0, {2, 2, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_SCHARR_Y_5X5_08U[] = {{4, 22, 4, 0, {2, 2, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_SCHARR_Y_5X5_08U[] = {{6, 22, 4, 0, {2, 2, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_SCHARR_Y_5X5_08U[] = {{8, 22, 4, 0, {2, 2, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_SCHARR_Y_5X5_08U[] = {{10, 22, 4, 0, {2, 2, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_SCHARR_Y_5X5_08U[] = {{12, 22, 4, 0, {2, 2, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_SCHARR_Y_5X5_08U[] = {{14, 22, 4, 0, {2, 2, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_SCHARR_Y_5X5_08U[] = {{16, 22, 4, 0, {2, 2, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_SCHARR_Y_5X5_08U[] = {{18, 22, 4, 0, {2, 2, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_SCHARR_Y_5X5_08U[] = {{2, 24, 4, 0, {2, 2, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_SCHARR_Y_5X5_08U[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_SCHARR_Y_5X5_08U[] = {{6, 24, 4, 0, {2, 2, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_SCHARR_Y_5X5_08U[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_SCHARR_Y_5X5_08U[] = {{10, 24, 4, 0, {2, 2, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_SCHARR_Y_5X5_08U[] = {{12, 24, 4, 0, {2, 2, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_SCHARR_Y_5X5_08U[] = {{14, 24, 4, 0, {2, 2, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_SCHARR_Y_5X5_08U[] = {{16, 24, 4, 0, {2, 2, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_SCHARR_Y_5X5_08U[] = {{2, 26, 4, 0, {2, 2, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_SCHARR_Y_5X5_08U[] = {{4, 26, 4, 0, {2, 2, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_SCHARR_Y_5X5_08U[] = {{6, 26, 4, 0, {2, 2, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_SCHARR_Y_5X5_08U[] = {{8, 26, 4, 0, {2, 2, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_SCHARR_Y_5X5_08U[] = {{10, 26, 4, 0, {2, 2, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_SCHARR_Y_5X5_08U[] = {{12, 26, 4, 0, {2, 2, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_SCHARR_Y_5X5_08U[] = {{14, 26, 4, 0, {2, 2, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_SCHARR_Y_5X5_08U[] = {{16, 26, 4, 0, {2, 2, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_SCHARR_Y_5X5_08U[] = {{2, 28, 4, 0, {2, 2, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_SCHARR_Y_5X5_08U[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_SCHARR_Y_5X5_08U[] = {{6, 28, 4, 0, {2, 2, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_SCHARR_Y_5X5_08U[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_SCHARR_Y_5X5_08U[] = {{10, 28, 4, 0, {2, 2, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_SCHARR_Y_5X5_08U[] = {{12, 28, 4, 0, {2, 2, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_SCHARR_Y_5X5_08U[] = {{14, 28, 4, 0, {2, 2, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_SCHARR_Y_5X5_08U[] = {{2, 30, 4, 0, {2, 2, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_SCHARR_Y_5X5_08U[] = {{4, 30, 4, 0, {2, 2, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_SCHARR_Y_5X5_08U[] = {{6, 30, 4, 0, {2, 2, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_SCHARR_Y_5X5_08U[] = {{8, 30, 4, 0, {2, 2, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_SCHARR_Y_5X5_08U[] = {{10, 30, 4, 0, {2, 2, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_SCHARR_Y_5X5_08U[] = {{12, 30, 4, 0, {2, 2, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_SCHARR_Y_5X5_08U[] = {{2, 32, 4, 0, {2, 2, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_SCHARR_Y_5X5_08U[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_SCHARR_Y_5X5_08U[] = {{6, 32, 4, 0, {2, 2, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_SCHARR_Y_5X5_08U[] = {{8, 32, 4, 0, {2, 2, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_SCHARR_Y_5X5_08U[] = {{10, 32, 4, 0, {2, 2, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_SCHARR_Y_5X5_08U[] = {{12, 32, 4, 0, {2, 2, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_SCHARR_Y_5X5_08U[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_SCHARR_Y_5X5_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_SCHARR_Y_5X5_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_SCHARR_Y_5X5_08U[] = {
{2, 1, 76, 0, 2, gScenarioBufferData0_SCHARR_Y_5X5_08U, 32, gScenarioKernelData0_SCHARR_Y_5X5_08U, 4},
{4, 1, 104, 0, 2, gScenarioBufferData1_SCHARR_Y_5X5_08U, 32, gScenarioKernelData1_SCHARR_Y_5X5_08U, 4},
{6, 1, 136, 0, 2, gScenarioBufferData2_SCHARR_Y_5X5_08U, 32, gScenarioKernelData2_SCHARR_Y_5X5_08U, 4},
{8, 1, 164, 0, 2, gScenarioBufferData3_SCHARR_Y_5X5_08U, 32, gScenarioKernelData3_SCHARR_Y_5X5_08U, 4},
{10, 1, 196, 0, 2, gScenarioBufferData4_SCHARR_Y_5X5_08U, 32, gScenarioKernelData4_SCHARR_Y_5X5_08U, 4},
{12, 1, 224, 0, 2, gScenarioBufferData5_SCHARR_Y_5X5_08U, 32, gScenarioKernelData5_SCHARR_Y_5X5_08U, 4},
{14, 1, 256, 0, 2, gScenarioBufferData6_SCHARR_Y_5X5_08U, 32, gScenarioKernelData6_SCHARR_Y_5X5_08U, 4},
{16, 1, 284, 0, 2, gScenarioBufferData7_SCHARR_Y_5X5_08U, 32, gScenarioKernelData7_SCHARR_Y_5X5_08U, 4},
{18, 1, 316, 0, 2, gScenarioBufferData8_SCHARR_Y_5X5_08U, 32, gScenarioKernelData8_SCHARR_Y_5X5_08U, 4},
{20, 1, 344, 0, 2, gScenarioBufferData9_SCHARR_Y_5X5_08U, 32, gScenarioKernelData9_SCHARR_Y_5X5_08U, 4},
{22, 1, 376, 0, 2, gScenarioBufferData10_SCHARR_Y_5X5_08U, 32, gScenarioKernelData10_SCHARR_Y_5X5_08U, 4},
{24, 1, 404, 0, 2, gScenarioBufferData11_SCHARR_Y_5X5_08U, 32, gScenarioKernelData11_SCHARR_Y_5X5_08U, 4},
{26, 1, 436, 0, 2, gScenarioBufferData12_SCHARR_Y_5X5_08U, 32, gScenarioKernelData12_SCHARR_Y_5X5_08U, 4},
{28, 1, 464, 0, 2, gScenarioBufferData13_SCHARR_Y_5X5_08U, 32, gScenarioKernelData13_SCHARR_Y_5X5_08U, 4},
{30, 1, 496, 0, 2, gScenarioBufferData14_SCHARR_Y_5X5_08U, 32, gScenarioKernelData14_SCHARR_Y_5X5_08U, 4},
{32, 1, 524, 0, 2, gScenarioBufferData15_SCHARR_Y_5X5_08U, 32, gScenarioKernelData15_SCHARR_Y_5X5_08U, 4},
{36, 1, 584, 0, 2, gScenarioBufferData16_SCHARR_Y_5X5_08U, 32, gScenarioKernelData16_SCHARR_Y_5X5_08U, 4},
{40, 1, 644, 0, 2, gScenarioBufferData17_SCHARR_Y_5X5_08U, 32, gScenarioKernelData17_SCHARR_Y_5X5_08U, 4},
{44, 1, 704, 0, 2, gScenarioBufferData18_SCHARR_Y_5X5_08U, 32, gScenarioKernelData18_SCHARR_Y_5X5_08U, 4},
{48, 1, 764, 0, 2, gScenarioBufferData19_SCHARR_Y_5X5_08U, 32, gScenarioKernelData19_SCHARR_Y_5X5_08U, 4},
{52, 1, 824, 0, 2, gScenarioBufferData20_SCHARR_Y_5X5_08U, 32, gScenarioKernelData20_SCHARR_Y_5X5_08U, 4},
{56, 1, 884, 0, 2, gScenarioBufferData21_SCHARR_Y_5X5_08U, 32, gScenarioKernelData21_SCHARR_Y_5X5_08U, 4},
{60, 1, 944, 0, 2, gScenarioBufferData22_SCHARR_Y_5X5_08U, 32, gScenarioKernelData22_SCHARR_Y_5X5_08U, 4},
{64, 1, 1004, 0, 2, gScenarioBufferData23_SCHARR_Y_5X5_08U, 32, gScenarioKernelData23_SCHARR_Y_5X5_08U, 4},
{2, 2, 88, 0, 1, gScenarioBufferData24_SCHARR_Y_5X5_08U, 32, gScenarioKernelData24_SCHARR_Y_5X5_08U, 4},
{4, 2, 128, 0, 1, gScenarioBufferData25_SCHARR_Y_5X5_08U, 32, gScenarioKernelData25_SCHARR_Y_5X5_08U, 4},
{6, 2, 168, 0, 1, gScenarioBufferData26_SCHARR_Y_5X5_08U, 32, gScenarioKernelData26_SCHARR_Y_5X5_08U, 4},
{8, 2, 208, 0, 1, gScenarioBufferData27_SCHARR_Y_5X5_08U, 32, gScenarioKernelData27_SCHARR_Y_5X5_08U, 4},
{10, 2, 248, 0, 1, gScenarioBufferData28_SCHARR_Y_5X5_08U, 32, gScenarioKernelData28_SCHARR_Y_5X5_08U, 4},
{12, 2, 288, 0, 1, gScenarioBufferData29_SCHARR_Y_5X5_08U, 32, gScenarioKernelData29_SCHARR_Y_5X5_08U, 4},
{14, 2, 328, 0, 1, gScenarioBufferData30_SCHARR_Y_5X5_08U, 32, gScenarioKernelData30_SCHARR_Y_5X5_08U, 4},
{16, 2, 368, 0, 1, gScenarioBufferData31_SCHARR_Y_5X5_08U, 32, gScenarioKernelData31_SCHARR_Y_5X5_08U, 4},
{18, 2, 408, 0, 1, gScenarioBufferData32_SCHARR_Y_5X5_08U, 32, gScenarioKernelData32_SCHARR_Y_5X5_08U, 4},
{20, 2, 448, 0, 1, gScenarioBufferData33_SCHARR_Y_5X5_08U, 32, gScenarioKernelData33_SCHARR_Y_5X5_08U, 4},
{22, 2, 488, 0, 1, gScenarioBufferData34_SCHARR_Y_5X5_08U, 32, gScenarioKernelData34_SCHARR_Y_5X5_08U, 4},
{24, 2, 528, 0, 1, gScenarioBufferData35_SCHARR_Y_5X5_08U, 32, gScenarioKernelData35_SCHARR_Y_5X5_08U, 4},
{26, 2, 568, 0, 1, gScenarioBufferData36_SCHARR_Y_5X5_08U, 32, gScenarioKernelData36_SCHARR_Y_5X5_08U, 4},
{28, 2, 608, 0, 1, gScenarioBufferData37_SCHARR_Y_5X5_08U, 32, gScenarioKernelData37_SCHARR_Y_5X5_08U, 4},
{30, 2, 648, 0, 1, gScenarioBufferData38_SCHARR_Y_5X5_08U, 32, gScenarioKernelData38_SCHARR_Y_5X5_08U, 4},
{32, 2, 688, 0, 1, gScenarioBufferData39_SCHARR_Y_5X5_08U, 32, gScenarioKernelData39_SCHARR_Y_5X5_08U, 4},
{36, 2, 768, 0, 1, gScenarioBufferData40_SCHARR_Y_5X5_08U, 32, gScenarioKernelData40_SCHARR_Y_5X5_08U, 4},
{40, 2, 848, 0, 1, gScenarioBufferData41_SCHARR_Y_5X5_08U, 32, gScenarioKernelData41_SCHARR_Y_5X5_08U, 4},
{44, 2, 928, 0, 1, gScenarioBufferData42_SCHARR_Y_5X5_08U, 32, gScenarioKernelData42_SCHARR_Y_5X5_08U, 4},
{48, 2, 1008, 0, 1, gScenarioBufferData43_SCHARR_Y_5X5_08U, 32, gScenarioKernelData43_SCHARR_Y_5X5_08U, 4},
{52, 2, 1088, 0, 1, gScenarioBufferData44_SCHARR_Y_5X5_08U, 32, gScenarioKernelData44_SCHARR_Y_5X5_08U, 4},
{56, 2, 1168, 0, 1, gScenarioBufferData45_SCHARR_Y_5X5_08U, 32, gScenarioKernelData45_SCHARR_Y_5X5_08U, 4},
{60, 2, 1248, 0, 1, gScenarioBufferData46_SCHARR_Y_5X5_08U, 32, gScenarioKernelData46_SCHARR_Y_5X5_08U, 4},
{64, 2, 1328, 0, 1, gScenarioBufferData47_SCHARR_Y_5X5_08U, 32, gScenarioKernelData47_SCHARR_Y_5X5_08U, 4},
{2, 3, 120, 0, 1, gScenarioBufferData48_SCHARR_Y_5X5_08U, 32, gScenarioKernelData48_SCHARR_Y_5X5_08U, 4},
{4, 3, 176, 0, 1, gScenarioBufferData49_SCHARR_Y_5X5_08U, 32, gScenarioKernelData49_SCHARR_Y_5X5_08U, 4},
{6, 3, 232, 0, 1, gScenarioBufferData50_SCHARR_Y_5X5_08U, 32, gScenarioKernelData50_SCHARR_Y_5X5_08U, 4},
{8, 3, 288, 0, 1, gScenarioBufferData51_SCHARR_Y_5X5_08U, 32, gScenarioKernelData51_SCHARR_Y_5X5_08U, 4},
{10, 3, 344, 0, 1, gScenarioBufferData52_SCHARR_Y_5X5_08U, 32, gScenarioKernelData52_SCHARR_Y_5X5_08U, 4},
{12, 3, 400, 0, 1, gScenarioBufferData53_SCHARR_Y_5X5_08U, 32, gScenarioKernelData53_SCHARR_Y_5X5_08U, 4},
{14, 3, 456, 0, 1, gScenarioBufferData54_SCHARR_Y_5X5_08U, 32, gScenarioKernelData54_SCHARR_Y_5X5_08U, 4},
{16, 3, 512, 0, 1, gScenarioBufferData55_SCHARR_Y_5X5_08U, 32, gScenarioKernelData55_SCHARR_Y_5X5_08U, 4},
{18, 3, 568, 0, 1, gScenarioBufferData56_SCHARR_Y_5X5_08U, 32, gScenarioKernelData56_SCHARR_Y_5X5_08U, 4},
{20, 3, 624, 0, 1, gScenarioBufferData57_SCHARR_Y_5X5_08U, 32, gScenarioKernelData57_SCHARR_Y_5X5_08U, 4},
{22, 3, 680, 0, 1, gScenarioBufferData58_SCHARR_Y_5X5_08U, 32, gScenarioKernelData58_SCHARR_Y_5X5_08U, 4},
{24, 3, 736, 0, 1, gScenarioBufferData59_SCHARR_Y_5X5_08U, 32, gScenarioKernelData59_SCHARR_Y_5X5_08U, 4},
{26, 3, 792, 0, 1, gScenarioBufferData60_SCHARR_Y_5X5_08U, 32, gScenarioKernelData60_SCHARR_Y_5X5_08U, 4},
{28, 3, 848, 0, 1, gScenarioBufferData61_SCHARR_Y_5X5_08U, 32, gScenarioKernelData61_SCHARR_Y_5X5_08U, 4},
{30, 3, 904, 0, 1, gScenarioBufferData62_SCHARR_Y_5X5_08U, 32, gScenarioKernelData62_SCHARR_Y_5X5_08U, 4},
{32, 3, 960, 0, 1, gScenarioBufferData63_SCHARR_Y_5X5_08U, 32, gScenarioKernelData63_SCHARR_Y_5X5_08U, 4},
{36, 3, 1072, 0, 1, gScenarioBufferData64_SCHARR_Y_5X5_08U, 32, gScenarioKernelData64_SCHARR_Y_5X5_08U, 4},
{40, 3, 1184, 0, 1, gScenarioBufferData65_SCHARR_Y_5X5_08U, 32, gScenarioKernelData65_SCHARR_Y_5X5_08U, 4},
{44, 3, 1296, 0, 1, gScenarioBufferData66_SCHARR_Y_5X5_08U, 32, gScenarioKernelData66_SCHARR_Y_5X5_08U, 4},
{48, 3, 1408, 0, 1, gScenarioBufferData67_SCHARR_Y_5X5_08U, 32, gScenarioKernelData67_SCHARR_Y_5X5_08U, 4},
{52, 3, 1520, 0, 1, gScenarioBufferData68_SCHARR_Y_5X5_08U, 32, gScenarioKernelData68_SCHARR_Y_5X5_08U, 4},
{56, 3, 1632, 0, 1, gScenarioBufferData69_SCHARR_Y_5X5_08U, 32, gScenarioKernelData69_SCHARR_Y_5X5_08U, 4},
{60, 3, 1744, 0, 1, gScenarioBufferData70_SCHARR_Y_5X5_08U, 32, gScenarioKernelData70_SCHARR_Y_5X5_08U, 4},
{64, 3, 1856, 0, 1, gScenarioBufferData71_SCHARR_Y_5X5_08U, 32, gScenarioKernelData71_SCHARR_Y_5X5_08U, 4},
{2, 4, 152, 0, 1, gScenarioBufferData72_SCHARR_Y_5X5_08U, 32, gScenarioKernelData72_SCHARR_Y_5X5_08U, 4},
{4, 4, 224, 0, 1, gScenarioBufferData73_SCHARR_Y_5X5_08U, 32, gScenarioKernelData73_SCHARR_Y_5X5_08U, 4},
{6, 4, 296, 0, 1, gScenarioBufferData74_SCHARR_Y_5X5_08U, 32, gScenarioKernelData74_SCHARR_Y_5X5_08U, 4},
{8, 4, 368, 0, 1, gScenarioBufferData75_SCHARR_Y_5X5_08U, 32, gScenarioKernelData75_SCHARR_Y_5X5_08U, 4},
{10, 4, 440, 0, 1, gScenarioBufferData76_SCHARR_Y_5X5_08U, 32, gScenarioKernelData76_SCHARR_Y_5X5_08U, 4},
{12, 4, 512, 0, 1, gScenarioBufferData77_SCHARR_Y_5X5_08U, 32, gScenarioKernelData77_SCHARR_Y_5X5_08U, 4},
{14, 4, 584, 0, 1, gScenarioBufferData78_SCHARR_Y_5X5_08U, 32, gScenarioKernelData78_SCHARR_Y_5X5_08U, 4},
{16, 4, 656, 0, 1, gScenarioBufferData79_SCHARR_Y_5X5_08U, 32, gScenarioKernelData79_SCHARR_Y_5X5_08U, 4},
{18, 4, 728, 0, 1, gScenarioBufferData80_SCHARR_Y_5X5_08U, 32, gScenarioKernelData80_SCHARR_Y_5X5_08U, 4},
{20, 4, 800, 0, 1, gScenarioBufferData81_SCHARR_Y_5X5_08U, 32, gScenarioKernelData81_SCHARR_Y_5X5_08U, 4},
{22, 4, 872, 0, 1, gScenarioBufferData82_SCHARR_Y_5X5_08U, 32, gScenarioKernelData82_SCHARR_Y_5X5_08U, 4},
{24, 4, 944, 0, 1, gScenarioBufferData83_SCHARR_Y_5X5_08U, 32, gScenarioKernelData83_SCHARR_Y_5X5_08U, 4},
{26, 4, 1016, 0, 1, gScenarioBufferData84_SCHARR_Y_5X5_08U, 32, gScenarioKernelData84_SCHARR_Y_5X5_08U, 4},
{28, 4, 1088, 0, 1, gScenarioBufferData85_SCHARR_Y_5X5_08U, 32, gScenarioKernelData85_SCHARR_Y_5X5_08U, 4},
{30, 4, 1160, 0, 1, gScenarioBufferData86_SCHARR_Y_5X5_08U, 32, gScenarioKernelData86_SCHARR_Y_5X5_08U, 4},
{32, 4, 1232, 0, 1, gScenarioBufferData87_SCHARR_Y_5X5_08U, 32, gScenarioKernelData87_SCHARR_Y_5X5_08U, 4},
{36, 4, 1376, 0, 1, gScenarioBufferData88_SCHARR_Y_5X5_08U, 32, gScenarioKernelData88_SCHARR_Y_5X5_08U, 4},
{40, 4, 1520, 0, 1, gScenarioBufferData89_SCHARR_Y_5X5_08U, 32, gScenarioKernelData89_SCHARR_Y_5X5_08U, 4},
{44, 4, 1664, 0, 1, gScenarioBufferData90_SCHARR_Y_5X5_08U, 32, gScenarioKernelData90_SCHARR_Y_5X5_08U, 4},
{48, 4, 1808, 0, 1, gScenarioBufferData91_SCHARR_Y_5X5_08U, 32, gScenarioKernelData91_SCHARR_Y_5X5_08U, 4},
{52, 4, 1952, 0, 1, gScenarioBufferData92_SCHARR_Y_5X5_08U, 32, gScenarioKernelData92_SCHARR_Y_5X5_08U, 4},
{56, 4, 2096, 0, 1, gScenarioBufferData93_SCHARR_Y_5X5_08U, 32, gScenarioKernelData93_SCHARR_Y_5X5_08U, 4},
{60, 4, 2240, 0, 1, gScenarioBufferData94_SCHARR_Y_5X5_08U, 32, gScenarioKernelData94_SCHARR_Y_5X5_08U, 4},
{64, 4, 2384, 0, 1, gScenarioBufferData95_SCHARR_Y_5X5_08U, 32, gScenarioKernelData95_SCHARR_Y_5X5_08U, 4},
{2, 5, 184, 0, 1, gScenarioBufferData96_SCHARR_Y_5X5_08U, 32, gScenarioKernelData96_SCHARR_Y_5X5_08U, 4},
{4, 5, 272, 0, 1, gScenarioBufferData97_SCHARR_Y_5X5_08U, 32, gScenarioKernelData97_SCHARR_Y_5X5_08U, 4},
{6, 5, 360, 0, 1, gScenarioBufferData98_SCHARR_Y_5X5_08U, 32, gScenarioKernelData98_SCHARR_Y_5X5_08U, 4},
{8, 5, 448, 0, 1, gScenarioBufferData99_SCHARR_Y_5X5_08U, 32, gScenarioKernelData99_SCHARR_Y_5X5_08U, 4},
{10, 5, 536, 0, 1, gScenarioBufferData100_SCHARR_Y_5X5_08U, 32, gScenarioKernelData100_SCHARR_Y_5X5_08U, 4},
{12, 5, 624, 0, 1, gScenarioBufferData101_SCHARR_Y_5X5_08U, 32, gScenarioKernelData101_SCHARR_Y_5X5_08U, 4},
{14, 5, 712, 0, 1, gScenarioBufferData102_SCHARR_Y_5X5_08U, 32, gScenarioKernelData102_SCHARR_Y_5X5_08U, 4},
{16, 5, 800, 0, 1, gScenarioBufferData103_SCHARR_Y_5X5_08U, 32, gScenarioKernelData103_SCHARR_Y_5X5_08U, 4},
{18, 5, 888, 0, 1, gScenarioBufferData104_SCHARR_Y_5X5_08U, 32, gScenarioKernelData104_SCHARR_Y_5X5_08U, 4},
{20, 5, 976, 0, 1, gScenarioBufferData105_SCHARR_Y_5X5_08U, 32, gScenarioKernelData105_SCHARR_Y_5X5_08U, 4},
{22, 5, 1064, 0, 1, gScenarioBufferData106_SCHARR_Y_5X5_08U, 32, gScenarioKernelData106_SCHARR_Y_5X5_08U, 4},
{24, 5, 1152, 0, 1, gScenarioBufferData107_SCHARR_Y_5X5_08U, 32, gScenarioKernelData107_SCHARR_Y_5X5_08U, 4},
{26, 5, 1240, 0, 1, gScenarioBufferData108_SCHARR_Y_5X5_08U, 32, gScenarioKernelData108_SCHARR_Y_5X5_08U, 4},
{28, 5, 1328, 0, 1, gScenarioBufferData109_SCHARR_Y_5X5_08U, 32, gScenarioKernelData109_SCHARR_Y_5X5_08U, 4},
{30, 5, 1416, 0, 1, gScenarioBufferData110_SCHARR_Y_5X5_08U, 32, gScenarioKernelData110_SCHARR_Y_5X5_08U, 4},
{32, 5, 1504, 0, 1, gScenarioBufferData111_SCHARR_Y_5X5_08U, 32, gScenarioKernelData111_SCHARR_Y_5X5_08U, 4},
{36, 5, 1680, 0, 1, gScenarioBufferData112_SCHARR_Y_5X5_08U, 32, gScenarioKernelData112_SCHARR_Y_5X5_08U, 4},
{40, 5, 1856, 0, 1, gScenarioBufferData113_SCHARR_Y_5X5_08U, 32, gScenarioKernelData113_SCHARR_Y_5X5_08U, 4},
{44, 5, 2032, 0, 1, gScenarioBufferData114_SCHARR_Y_5X5_08U, 32, gScenarioKernelData114_SCHARR_Y_5X5_08U, 4},
{48, 5, 2208, 0, 1, gScenarioBufferData115_SCHARR_Y_5X5_08U, 32, gScenarioKernelData115_SCHARR_Y_5X5_08U, 4},
{52, 5, 2384, 0, 1, gScenarioBufferData116_SCHARR_Y_5X5_08U, 32, gScenarioKernelData116_SCHARR_Y_5X5_08U, 4},
{56, 5, 2560, 0, 1, gScenarioBufferData117_SCHARR_Y_5X5_08U, 32, gScenarioKernelData117_SCHARR_Y_5X5_08U, 4},
{60, 5, 2736, 0, 1, gScenarioBufferData118_SCHARR_Y_5X5_08U, 32, gScenarioKernelData118_SCHARR_Y_5X5_08U, 4},
{64, 5, 2912, 0, 1, gScenarioBufferData119_SCHARR_Y_5X5_08U, 32, gScenarioKernelData119_SCHARR_Y_5X5_08U, 4},
{2, 6, 216, 0, 1, gScenarioBufferData120_SCHARR_Y_5X5_08U, 32, gScenarioKernelData120_SCHARR_Y_5X5_08U, 4},
{4, 6, 320, 0, 1, gScenarioBufferData121_SCHARR_Y_5X5_08U, 32, gScenarioKernelData121_SCHARR_Y_5X5_08U, 4},
{6, 6, 424, 0, 1, gScenarioBufferData122_SCHARR_Y_5X5_08U, 32, gScenarioKernelData122_SCHARR_Y_5X5_08U, 4},
{8, 6, 528, 0, 1, gScenarioBufferData123_SCHARR_Y_5X5_08U, 32, gScenarioKernelData123_SCHARR_Y_5X5_08U, 4},
{10, 6, 632, 0, 1, gScenarioBufferData124_SCHARR_Y_5X5_08U, 32, gScenarioKernelData124_SCHARR_Y_5X5_08U, 4},
{12, 6, 736, 0, 1, gScenarioBufferData125_SCHARR_Y_5X5_08U, 32, gScenarioKernelData125_SCHARR_Y_5X5_08U, 4},
{14, 6, 840, 0, 1, gScenarioBufferData126_SCHARR_Y_5X5_08U, 32, gScenarioKernelData126_SCHARR_Y_5X5_08U, 4},
{16, 6, 944, 0, 1, gScenarioBufferData127_SCHARR_Y_5X5_08U, 32, gScenarioKernelData127_SCHARR_Y_5X5_08U, 4},
{18, 6, 1048, 0, 1, gScenarioBufferData128_SCHARR_Y_5X5_08U, 32, gScenarioKernelData128_SCHARR_Y_5X5_08U, 4},
{20, 6, 1152, 0, 1, gScenarioBufferData129_SCHARR_Y_5X5_08U, 32, gScenarioKernelData129_SCHARR_Y_5X5_08U, 4},
{22, 6, 1256, 0, 1, gScenarioBufferData130_SCHARR_Y_5X5_08U, 32, gScenarioKernelData130_SCHARR_Y_5X5_08U, 4},
{24, 6, 1360, 0, 1, gScenarioBufferData131_SCHARR_Y_5X5_08U, 32, gScenarioKernelData131_SCHARR_Y_5X5_08U, 4},
{26, 6, 1464, 0, 1, gScenarioBufferData132_SCHARR_Y_5X5_08U, 32, gScenarioKernelData132_SCHARR_Y_5X5_08U, 4},
{28, 6, 1568, 0, 1, gScenarioBufferData133_SCHARR_Y_5X5_08U, 32, gScenarioKernelData133_SCHARR_Y_5X5_08U, 4},
{30, 6, 1672, 0, 1, gScenarioBufferData134_SCHARR_Y_5X5_08U, 32, gScenarioKernelData134_SCHARR_Y_5X5_08U, 4},
{32, 6, 1776, 0, 1, gScenarioBufferData135_SCHARR_Y_5X5_08U, 32, gScenarioKernelData135_SCHARR_Y_5X5_08U, 4},
{36, 6, 1984, 0, 1, gScenarioBufferData136_SCHARR_Y_5X5_08U, 32, gScenarioKernelData136_SCHARR_Y_5X5_08U, 4},
{40, 6, 2192, 0, 1, gScenarioBufferData137_SCHARR_Y_5X5_08U, 32, gScenarioKernelData137_SCHARR_Y_5X5_08U, 4},
{44, 6, 2400, 0, 1, gScenarioBufferData138_SCHARR_Y_5X5_08U, 32, gScenarioKernelData138_SCHARR_Y_5X5_08U, 4},
{48, 6, 2608, 0, 1, gScenarioBufferData139_SCHARR_Y_5X5_08U, 32, gScenarioKernelData139_SCHARR_Y_5X5_08U, 4},
{52, 6, 2816, 0, 1, gScenarioBufferData140_SCHARR_Y_5X5_08U, 32, gScenarioKernelData140_SCHARR_Y_5X5_08U, 4},
{56, 6, 3024, 0, 1, gScenarioBufferData141_SCHARR_Y_5X5_08U, 32, gScenarioKernelData141_SCHARR_Y_5X5_08U, 4},
{60, 6, 3232, 0, 1, gScenarioBufferData142_SCHARR_Y_5X5_08U, 32, gScenarioKernelData142_SCHARR_Y_5X5_08U, 4},
{64, 6, 3440, 0, 1, gScenarioBufferData143_SCHARR_Y_5X5_08U, 32, gScenarioKernelData143_SCHARR_Y_5X5_08U, 4},
{2, 8, 280, 0, 1, gScenarioBufferData144_SCHARR_Y_5X5_08U, 32, gScenarioKernelData144_SCHARR_Y_5X5_08U, 4},
{4, 8, 416, 0, 1, gScenarioBufferData145_SCHARR_Y_5X5_08U, 32, gScenarioKernelData145_SCHARR_Y_5X5_08U, 4},
{6, 8, 552, 0, 1, gScenarioBufferData146_SCHARR_Y_5X5_08U, 32, gScenarioKernelData146_SCHARR_Y_5X5_08U, 4},
{8, 8, 688, 0, 1, gScenarioBufferData147_SCHARR_Y_5X5_08U, 32, gScenarioKernelData147_SCHARR_Y_5X5_08U, 4},
{10, 8, 824, 0, 1, gScenarioBufferData148_SCHARR_Y_5X5_08U, 32, gScenarioKernelData148_SCHARR_Y_5X5_08U, 4},
{12, 8, 960, 0, 1, gScenarioBufferData149_SCHARR_Y_5X5_08U, 32, gScenarioKernelData149_SCHARR_Y_5X5_08U, 4},
{14, 8, 1096, 0, 1, gScenarioBufferData150_SCHARR_Y_5X5_08U, 32, gScenarioKernelData150_SCHARR_Y_5X5_08U, 4},
{16, 8, 1232, 0, 1, gScenarioBufferData151_SCHARR_Y_5X5_08U, 32, gScenarioKernelData151_SCHARR_Y_5X5_08U, 4},
{18, 8, 1368, 0, 1, gScenarioBufferData152_SCHARR_Y_5X5_08U, 32, gScenarioKernelData152_SCHARR_Y_5X5_08U, 4},
{20, 8, 1504, 0, 1, gScenarioBufferData153_SCHARR_Y_5X5_08U, 32, gScenarioKernelData153_SCHARR_Y_5X5_08U, 4},
{22, 8, 1640, 0, 1, gScenarioBufferData154_SCHARR_Y_5X5_08U, 32, gScenarioKernelData154_SCHARR_Y_5X5_08U, 4},
{24, 8, 1776, 0, 1, gScenarioBufferData155_SCHARR_Y_5X5_08U, 32, gScenarioKernelData155_SCHARR_Y_5X5_08U, 4},
{26, 8, 1912, 0, 1, gScenarioBufferData156_SCHARR_Y_5X5_08U, 32, gScenarioKernelData156_SCHARR_Y_5X5_08U, 4},
{28, 8, 2048, 0, 1, gScenarioBufferData157_SCHARR_Y_5X5_08U, 32, gScenarioKernelData157_SCHARR_Y_5X5_08U, 4},
{30, 8, 2184, 0, 1, gScenarioBufferData158_SCHARR_Y_5X5_08U, 32, gScenarioKernelData158_SCHARR_Y_5X5_08U, 4},
{32, 8, 2320, 0, 1, gScenarioBufferData159_SCHARR_Y_5X5_08U, 32, gScenarioKernelData159_SCHARR_Y_5X5_08U, 4},
{36, 8, 2592, 0, 1, gScenarioBufferData160_SCHARR_Y_5X5_08U, 32, gScenarioKernelData160_SCHARR_Y_5X5_08U, 4},
{40, 8, 2864, 0, 1, gScenarioBufferData161_SCHARR_Y_5X5_08U, 32, gScenarioKernelData161_SCHARR_Y_5X5_08U, 4},
{44, 8, 3136, 0, 1, gScenarioBufferData162_SCHARR_Y_5X5_08U, 32, gScenarioKernelData162_SCHARR_Y_5X5_08U, 4},
{48, 8, 3408, 0, 1, gScenarioBufferData163_SCHARR_Y_5X5_08U, 32, gScenarioKernelData163_SCHARR_Y_5X5_08U, 4},
{52, 8, 3680, 0, 1, gScenarioBufferData164_SCHARR_Y_5X5_08U, 32, gScenarioKernelData164_SCHARR_Y_5X5_08U, 4},
{2, 10, 344, 0, 1, gScenarioBufferData165_SCHARR_Y_5X5_08U, 32, gScenarioKernelData165_SCHARR_Y_5X5_08U, 4},
{4, 10, 512, 0, 1, gScenarioBufferData166_SCHARR_Y_5X5_08U, 32, gScenarioKernelData166_SCHARR_Y_5X5_08U, 4},
{6, 10, 680, 0, 1, gScenarioBufferData167_SCHARR_Y_5X5_08U, 32, gScenarioKernelData167_SCHARR_Y_5X5_08U, 4},
{8, 10, 848, 0, 1, gScenarioBufferData168_SCHARR_Y_5X5_08U, 32, gScenarioKernelData168_SCHARR_Y_5X5_08U, 4},
{10, 10, 1016, 0, 1, gScenarioBufferData169_SCHARR_Y_5X5_08U, 32, gScenarioKernelData169_SCHARR_Y_5X5_08U, 4},
{12, 10, 1184, 0, 1, gScenarioBufferData170_SCHARR_Y_5X5_08U, 32, gScenarioKernelData170_SCHARR_Y_5X5_08U, 4},
{14, 10, 1352, 0, 1, gScenarioBufferData171_SCHARR_Y_5X5_08U, 32, gScenarioKernelData171_SCHARR_Y_5X5_08U, 4},
{16, 10, 1520, 0, 1, gScenarioBufferData172_SCHARR_Y_5X5_08U, 32, gScenarioKernelData172_SCHARR_Y_5X5_08U, 4},
{18, 10, 1688, 0, 1, gScenarioBufferData173_SCHARR_Y_5X5_08U, 32, gScenarioKernelData173_SCHARR_Y_5X5_08U, 4},
{20, 10, 1856, 0, 1, gScenarioBufferData174_SCHARR_Y_5X5_08U, 32, gScenarioKernelData174_SCHARR_Y_5X5_08U, 4},
{22, 10, 2024, 0, 1, gScenarioBufferData175_SCHARR_Y_5X5_08U, 32, gScenarioKernelData175_SCHARR_Y_5X5_08U, 4},
{24, 10, 2192, 0, 1, gScenarioBufferData176_SCHARR_Y_5X5_08U, 32, gScenarioKernelData176_SCHARR_Y_5X5_08U, 4},
{26, 10, 2360, 0, 1, gScenarioBufferData177_SCHARR_Y_5X5_08U, 32, gScenarioKernelData177_SCHARR_Y_5X5_08U, 4},
{28, 10, 2528, 0, 1, gScenarioBufferData178_SCHARR_Y_5X5_08U, 32, gScenarioKernelData178_SCHARR_Y_5X5_08U, 4},
{30, 10, 2696, 0, 1, gScenarioBufferData179_SCHARR_Y_5X5_08U, 32, gScenarioKernelData179_SCHARR_Y_5X5_08U, 4},
{32, 10, 2864, 0, 1, gScenarioBufferData180_SCHARR_Y_5X5_08U, 32, gScenarioKernelData180_SCHARR_Y_5X5_08U, 4},
{36, 10, 3200, 0, 1, gScenarioBufferData181_SCHARR_Y_5X5_08U, 32, gScenarioKernelData181_SCHARR_Y_5X5_08U, 4},
{40, 10, 3536, 0, 1, gScenarioBufferData182_SCHARR_Y_5X5_08U, 32, gScenarioKernelData182_SCHARR_Y_5X5_08U, 4},
{2, 12, 408, 0, 1, gScenarioBufferData183_SCHARR_Y_5X5_08U, 32, gScenarioKernelData183_SCHARR_Y_5X5_08U, 4},
{4, 12, 608, 0, 1, gScenarioBufferData184_SCHARR_Y_5X5_08U, 32, gScenarioKernelData184_SCHARR_Y_5X5_08U, 4},
{6, 12, 808, 0, 1, gScenarioBufferData185_SCHARR_Y_5X5_08U, 32, gScenarioKernelData185_SCHARR_Y_5X5_08U, 4},
{8, 12, 1008, 0, 1, gScenarioBufferData186_SCHARR_Y_5X5_08U, 32, gScenarioKernelData186_SCHARR_Y_5X5_08U, 4},
{10, 12, 1208, 0, 1, gScenarioBufferData187_SCHARR_Y_5X5_08U, 32, gScenarioKernelData187_SCHARR_Y_5X5_08U, 4},
{12, 12, 1408, 0, 1, gScenarioBufferData188_SCHARR_Y_5X5_08U, 32, gScenarioKernelData188_SCHARR_Y_5X5_08U, 4},
{14, 12, 1608, 0, 1, gScenarioBufferData189_SCHARR_Y_5X5_08U, 32, gScenarioKernelData189_SCHARR_Y_5X5_08U, 4},
{16, 12, 1808, 0, 1, gScenarioBufferData190_SCHARR_Y_5X5_08U, 32, gScenarioKernelData190_SCHARR_Y_5X5_08U, 4},
{18, 12, 2008, 0, 1, gScenarioBufferData191_SCHARR_Y_5X5_08U, 32, gScenarioKernelData191_SCHARR_Y_5X5_08U, 4},
{20, 12, 2208, 0, 1, gScenarioBufferData192_SCHARR_Y_5X5_08U, 32, gScenarioKernelData192_SCHARR_Y_5X5_08U, 4},
{22, 12, 2408, 0, 1, gScenarioBufferData193_SCHARR_Y_5X5_08U, 32, gScenarioKernelData193_SCHARR_Y_5X5_08U, 4},
{24, 12, 2608, 0, 1, gScenarioBufferData194_SCHARR_Y_5X5_08U, 32, gScenarioKernelData194_SCHARR_Y_5X5_08U, 4},
{26, 12, 2808, 0, 1, gScenarioBufferData195_SCHARR_Y_5X5_08U, 32, gScenarioKernelData195_SCHARR_Y_5X5_08U, 4},
{28, 12, 3008, 0, 1, gScenarioBufferData196_SCHARR_Y_5X5_08U, 32, gScenarioKernelData196_SCHARR_Y_5X5_08U, 4},
{30, 12, 3208, 0, 1, gScenarioBufferData197_SCHARR_Y_5X5_08U, 32, gScenarioKernelData197_SCHARR_Y_5X5_08U, 4},
{32, 12, 3408, 0, 1, gScenarioBufferData198_SCHARR_Y_5X5_08U, 32, gScenarioKernelData198_SCHARR_Y_5X5_08U, 4},
{36, 12, 3808, 0, 1, gScenarioBufferData199_SCHARR_Y_5X5_08U, 32, gScenarioKernelData199_SCHARR_Y_5X5_08U, 4},
{2, 14, 472, 0, 1, gScenarioBufferData200_SCHARR_Y_5X5_08U, 32, gScenarioKernelData200_SCHARR_Y_5X5_08U, 4},
{4, 14, 704, 0, 1, gScenarioBufferData201_SCHARR_Y_5X5_08U, 32, gScenarioKernelData201_SCHARR_Y_5X5_08U, 4},
{6, 14, 936, 0, 1, gScenarioBufferData202_SCHARR_Y_5X5_08U, 32, gScenarioKernelData202_SCHARR_Y_5X5_08U, 4},
{8, 14, 1168, 0, 1, gScenarioBufferData203_SCHARR_Y_5X5_08U, 32, gScenarioKernelData203_SCHARR_Y_5X5_08U, 4},
{10, 14, 1400, 0, 1, gScenarioBufferData204_SCHARR_Y_5X5_08U, 32, gScenarioKernelData204_SCHARR_Y_5X5_08U, 4},
{12, 14, 1632, 0, 1, gScenarioBufferData205_SCHARR_Y_5X5_08U, 32, gScenarioKernelData205_SCHARR_Y_5X5_08U, 4},
{14, 14, 1864, 0, 1, gScenarioBufferData206_SCHARR_Y_5X5_08U, 32, gScenarioKernelData206_SCHARR_Y_5X5_08U, 4},
{16, 14, 2096, 0, 1, gScenarioBufferData207_SCHARR_Y_5X5_08U, 32, gScenarioKernelData207_SCHARR_Y_5X5_08U, 4},
{18, 14, 2328, 0, 1, gScenarioBufferData208_SCHARR_Y_5X5_08U, 32, gScenarioKernelData208_SCHARR_Y_5X5_08U, 4},
{20, 14, 2560, 0, 1, gScenarioBufferData209_SCHARR_Y_5X5_08U, 32, gScenarioKernelData209_SCHARR_Y_5X5_08U, 4},
{22, 14, 2792, 0, 1, gScenarioBufferData210_SCHARR_Y_5X5_08U, 32, gScenarioKernelData210_SCHARR_Y_5X5_08U, 4},
{24, 14, 3024, 0, 1, gScenarioBufferData211_SCHARR_Y_5X5_08U, 32, gScenarioKernelData211_SCHARR_Y_5X5_08U, 4},
{26, 14, 3256, 0, 1, gScenarioBufferData212_SCHARR_Y_5X5_08U, 32, gScenarioKernelData212_SCHARR_Y_5X5_08U, 4},
{28, 14, 3488, 0, 1, gScenarioBufferData213_SCHARR_Y_5X5_08U, 32, gScenarioKernelData213_SCHARR_Y_5X5_08U, 4},
{30, 14, 3720, 0, 1, gScenarioBufferData214_SCHARR_Y_5X5_08U, 32, gScenarioKernelData214_SCHARR_Y_5X5_08U, 4},
{2, 16, 536, 0, 1, gScenarioBufferData215_SCHARR_Y_5X5_08U, 32, gScenarioKernelData215_SCHARR_Y_5X5_08U, 4},
{4, 16, 800, 0, 1, gScenarioBufferData216_SCHARR_Y_5X5_08U, 32, gScenarioKernelData216_SCHARR_Y_5X5_08U, 4},
{6, 16, 1064, 0, 1, gScenarioBufferData217_SCHARR_Y_5X5_08U, 32, gScenarioKernelData217_SCHARR_Y_5X5_08U, 4},
{8, 16, 1328, 0, 1, gScenarioBufferData218_SCHARR_Y_5X5_08U, 32, gScenarioKernelData218_SCHARR_Y_5X5_08U, 4},
{10, 16, 1592, 0, 1, gScenarioBufferData219_SCHARR_Y_5X5_08U, 32, gScenarioKernelData219_SCHARR_Y_5X5_08U, 4},
{12, 16, 1856, 0, 1, gScenarioBufferData220_SCHARR_Y_5X5_08U, 32, gScenarioKernelData220_SCHARR_Y_5X5_08U, 4},
{14, 16, 2120, 0, 1, gScenarioBufferData221_SCHARR_Y_5X5_08U, 32, gScenarioKernelData221_SCHARR_Y_5X5_08U, 4},
{16, 16, 2384, 0, 1, gScenarioBufferData222_SCHARR_Y_5X5_08U, 32, gScenarioKernelData222_SCHARR_Y_5X5_08U, 4},
{18, 16, 2648, 0, 1, gScenarioBufferData223_SCHARR_Y_5X5_08U, 32, gScenarioKernelData223_SCHARR_Y_5X5_08U, 4},
{20, 16, 2912, 0, 1, gScenarioBufferData224_SCHARR_Y_5X5_08U, 32, gScenarioKernelData224_SCHARR_Y_5X5_08U, 4},
{22, 16, 3176, 0, 1, gScenarioBufferData225_SCHARR_Y_5X5_08U, 32, gScenarioKernelData225_SCHARR_Y_5X5_08U, 4},
{24, 16, 3440, 0, 1, gScenarioBufferData226_SCHARR_Y_5X5_08U, 32, gScenarioKernelData226_SCHARR_Y_5X5_08U, 4},
{26, 16, 3704, 0, 1, gScenarioBufferData227_SCHARR_Y_5X5_08U, 32, gScenarioKernelData227_SCHARR_Y_5X5_08U, 4},
{2, 18, 600, 0, 1, gScenarioBufferData228_SCHARR_Y_5X5_08U, 32, gScenarioKernelData228_SCHARR_Y_5X5_08U, 4},
{4, 18, 896, 0, 1, gScenarioBufferData229_SCHARR_Y_5X5_08U, 32, gScenarioKernelData229_SCHARR_Y_5X5_08U, 4},
{6, 18, 1192, 0, 1, gScenarioBufferData230_SCHARR_Y_5X5_08U, 32, gScenarioKernelData230_SCHARR_Y_5X5_08U, 4},
{8, 18, 1488, 0, 1, gScenarioBufferData231_SCHARR_Y_5X5_08U, 32, gScenarioKernelData231_SCHARR_Y_5X5_08U, 4},
{10, 18, 1784, 0, 1, gScenarioBufferData232_SCHARR_Y_5X5_08U, 32, gScenarioKernelData232_SCHARR_Y_5X5_08U, 4},
{12, 18, 2080, 0, 1, gScenarioBufferData233_SCHARR_Y_5X5_08U, 32, gScenarioKernelData233_SCHARR_Y_5X5_08U, 4},
{14, 18, 2376, 0, 1, gScenarioBufferData234_SCHARR_Y_5X5_08U, 32, gScenarioKernelData234_SCHARR_Y_5X5_08U, 4},
{16, 18, 2672, 0, 1, gScenarioBufferData235_SCHARR_Y_5X5_08U, 32, gScenarioKernelData235_SCHARR_Y_5X5_08U, 4},
{18, 18, 2968, 0, 1, gScenarioBufferData236_SCHARR_Y_5X5_08U, 32, gScenarioKernelData236_SCHARR_Y_5X5_08U, 4},
{20, 18, 3264, 0, 1, gScenarioBufferData237_SCHARR_Y_5X5_08U, 32, gScenarioKernelData237_SCHARR_Y_5X5_08U, 4},
{22, 18, 3560, 0, 1, gScenarioBufferData238_SCHARR_Y_5X5_08U, 32, gScenarioKernelData238_SCHARR_Y_5X5_08U, 4},
{2, 20, 664, 0, 1, gScenarioBufferData239_SCHARR_Y_5X5_08U, 32, gScenarioKernelData239_SCHARR_Y_5X5_08U, 4},
{4, 20, 992, 0, 1, gScenarioBufferData240_SCHARR_Y_5X5_08U, 32, gScenarioKernelData240_SCHARR_Y_5X5_08U, 4},
{6, 20, 1320, 0, 1, gScenarioBufferData241_SCHARR_Y_5X5_08U, 32, gScenarioKernelData241_SCHARR_Y_5X5_08U, 4},
{8, 20, 1648, 0, 1, gScenarioBufferData242_SCHARR_Y_5X5_08U, 32, gScenarioKernelData242_SCHARR_Y_5X5_08U, 4},
{10, 20, 1976, 0, 1, gScenarioBufferData243_SCHARR_Y_5X5_08U, 32, gScenarioKernelData243_SCHARR_Y_5X5_08U, 4},
{12, 20, 2304, 0, 1, gScenarioBufferData244_SCHARR_Y_5X5_08U, 32, gScenarioKernelData244_SCHARR_Y_5X5_08U, 4},
{14, 20, 2632, 0, 1, gScenarioBufferData245_SCHARR_Y_5X5_08U, 32, gScenarioKernelData245_SCHARR_Y_5X5_08U, 4},
{16, 20, 2960, 0, 1, gScenarioBufferData246_SCHARR_Y_5X5_08U, 32, gScenarioKernelData246_SCHARR_Y_5X5_08U, 4},
{18, 20, 3288, 0, 1, gScenarioBufferData247_SCHARR_Y_5X5_08U, 32, gScenarioKernelData247_SCHARR_Y_5X5_08U, 4},
{20, 20, 3616, 0, 1, gScenarioBufferData248_SCHARR_Y_5X5_08U, 32, gScenarioKernelData248_SCHARR_Y_5X5_08U, 4},
{2, 22, 728, 0, 1, gScenarioBufferData249_SCHARR_Y_5X5_08U, 32, gScenarioKernelData249_SCHARR_Y_5X5_08U, 4},
{4, 22, 1088, 0, 1, gScenarioBufferData250_SCHARR_Y_5X5_08U, 32, gScenarioKernelData250_SCHARR_Y_5X5_08U, 4},
{6, 22, 1448, 0, 1, gScenarioBufferData251_SCHARR_Y_5X5_08U, 32, gScenarioKernelData251_SCHARR_Y_5X5_08U, 4},
{8, 22, 1808, 0, 1, gScenarioBufferData252_SCHARR_Y_5X5_08U, 32, gScenarioKernelData252_SCHARR_Y_5X5_08U, 4},
{10, 22, 2168, 0, 1, gScenarioBufferData253_SCHARR_Y_5X5_08U, 32, gScenarioKernelData253_SCHARR_Y_5X5_08U, 4},
{12, 22, 2528, 0, 1, gScenarioBufferData254_SCHARR_Y_5X5_08U, 32, gScenarioKernelData254_SCHARR_Y_5X5_08U, 4},
{14, 22, 2888, 0, 1, gScenarioBufferData255_SCHARR_Y_5X5_08U, 32, gScenarioKernelData255_SCHARR_Y_5X5_08U, 4},
{16, 22, 3248, 0, 1, gScenarioBufferData256_SCHARR_Y_5X5_08U, 32, gScenarioKernelData256_SCHARR_Y_5X5_08U, 4},
{18, 22, 3608, 0, 1, gScenarioBufferData257_SCHARR_Y_5X5_08U, 32, gScenarioKernelData257_SCHARR_Y_5X5_08U, 4},
{2, 24, 792, 0, 1, gScenarioBufferData258_SCHARR_Y_5X5_08U, 32, gScenarioKernelData258_SCHARR_Y_5X5_08U, 4},
{4, 24, 1184, 0, 1, gScenarioBufferData259_SCHARR_Y_5X5_08U, 32, gScenarioKernelData259_SCHARR_Y_5X5_08U, 4},
{6, 24, 1576, 0, 1, gScenarioBufferData260_SCHARR_Y_5X5_08U, 32, gScenarioKernelData260_SCHARR_Y_5X5_08U, 4},
{8, 24, 1968, 0, 1, gScenarioBufferData261_SCHARR_Y_5X5_08U, 32, gScenarioKernelData261_SCHARR_Y_5X5_08U, 4},
{10, 24, 2360, 0, 1, gScenarioBufferData262_SCHARR_Y_5X5_08U, 32, gScenarioKernelData262_SCHARR_Y_5X5_08U, 4},
{12, 24, 2752, 0, 1, gScenarioBufferData263_SCHARR_Y_5X5_08U, 32, gScenarioKernelData263_SCHARR_Y_5X5_08U, 4},
{14, 24, 3144, 0, 1, gScenarioBufferData264_SCHARR_Y_5X5_08U, 32, gScenarioKernelData264_SCHARR_Y_5X5_08U, 4},
{16, 24, 3536, 0, 1, gScenarioBufferData265_SCHARR_Y_5X5_08U, 32, gScenarioKernelData265_SCHARR_Y_5X5_08U, 4},
{2, 26, 856, 0, 1, gScenarioBufferData266_SCHARR_Y_5X5_08U, 32, gScenarioKernelData266_SCHARR_Y_5X5_08U, 4},
{4, 26, 1280, 0, 1, gScenarioBufferData267_SCHARR_Y_5X5_08U, 32, gScenarioKernelData267_SCHARR_Y_5X5_08U, 4},
{6, 26, 1704, 0, 1, gScenarioBufferData268_SCHARR_Y_5X5_08U, 32, gScenarioKernelData268_SCHARR_Y_5X5_08U, 4},
{8, 26, 2128, 0, 1, gScenarioBufferData269_SCHARR_Y_5X5_08U, 32, gScenarioKernelData269_SCHARR_Y_5X5_08U, 4},
{10, 26, 2552, 0, 1, gScenarioBufferData270_SCHARR_Y_5X5_08U, 32, gScenarioKernelData270_SCHARR_Y_5X5_08U, 4},
{12, 26, 2976, 0, 1, gScenarioBufferData271_SCHARR_Y_5X5_08U, 32, gScenarioKernelData271_SCHARR_Y_5X5_08U, 4},
{14, 26, 3400, 0, 1, gScenarioBufferData272_SCHARR_Y_5X5_08U, 32, gScenarioKernelData272_SCHARR_Y_5X5_08U, 4},
{16, 26, 3824, 0, 1, gScenarioBufferData273_SCHARR_Y_5X5_08U, 32, gScenarioKernelData273_SCHARR_Y_5X5_08U, 4},
{2, 28, 920, 0, 1, gScenarioBufferData274_SCHARR_Y_5X5_08U, 32, gScenarioKernelData274_SCHARR_Y_5X5_08U, 4},
{4, 28, 1376, 0, 1, gScenarioBufferData275_SCHARR_Y_5X5_08U, 32, gScenarioKernelData275_SCHARR_Y_5X5_08U, 4},
{6, 28, 1832, 0, 1, gScenarioBufferData276_SCHARR_Y_5X5_08U, 32, gScenarioKernelData276_SCHARR_Y_5X5_08U, 4},
{8, 28, 2288, 0, 1, gScenarioBufferData277_SCHARR_Y_5X5_08U, 32, gScenarioKernelData277_SCHARR_Y_5X5_08U, 4},
{10, 28, 2744, 0, 1, gScenarioBufferData278_SCHARR_Y_5X5_08U, 32, gScenarioKernelData278_SCHARR_Y_5X5_08U, 4},
{12, 28, 3200, 0, 1, gScenarioBufferData279_SCHARR_Y_5X5_08U, 32, gScenarioKernelData279_SCHARR_Y_5X5_08U, 4},
{14, 28, 3656, 0, 1, gScenarioBufferData280_SCHARR_Y_5X5_08U, 32, gScenarioKernelData280_SCHARR_Y_5X5_08U, 4},
{2, 30, 984, 0, 1, gScenarioBufferData281_SCHARR_Y_5X5_08U, 32, gScenarioKernelData281_SCHARR_Y_5X5_08U, 4},
{4, 30, 1472, 0, 1, gScenarioBufferData282_SCHARR_Y_5X5_08U, 32, gScenarioKernelData282_SCHARR_Y_5X5_08U, 4},
{6, 30, 1960, 0, 1, gScenarioBufferData283_SCHARR_Y_5X5_08U, 32, gScenarioKernelData283_SCHARR_Y_5X5_08U, 4},
{8, 30, 2448, 0, 1, gScenarioBufferData284_SCHARR_Y_5X5_08U, 32, gScenarioKernelData284_SCHARR_Y_5X5_08U, 4},
{10, 30, 2936, 0, 1, gScenarioBufferData285_SCHARR_Y_5X5_08U, 32, gScenarioKernelData285_SCHARR_Y_5X5_08U, 4},
{12, 30, 3424, 0, 1, gScenarioBufferData286_SCHARR_Y_5X5_08U, 32, gScenarioKernelData286_SCHARR_Y_5X5_08U, 4},
{2, 32, 1048, 0, 1, gScenarioBufferData287_SCHARR_Y_5X5_08U, 32, gScenarioKernelData287_SCHARR_Y_5X5_08U, 4},
{4, 32, 1568, 0, 1, gScenarioBufferData288_SCHARR_Y_5X5_08U, 32, gScenarioKernelData288_SCHARR_Y_5X5_08U, 4},
{6, 32, 2088, 0, 1, gScenarioBufferData289_SCHARR_Y_5X5_08U, 32, gScenarioKernelData289_SCHARR_Y_5X5_08U, 4},
{8, 32, 2608, 0, 1, gScenarioBufferData290_SCHARR_Y_5X5_08U, 32, gScenarioKernelData290_SCHARR_Y_5X5_08U, 4},
{10, 32, 3128, 0, 1, gScenarioBufferData291_SCHARR_Y_5X5_08U, 32, gScenarioKernelData291_SCHARR_Y_5X5_08U, 4},
{12, 32, 3648, 0, 1, gScenarioBufferData292_SCHARR_Y_5X5_08U, 32, gScenarioKernelData292_SCHARR_Y_5X5_08U, 4}
};

static acf_scenario_list gScenarioList_SCHARR_Y_5X5_08U = {
293, //number of scenarios
gScenarioArray_SCHARR_Y_5X5_08U};
//**************************************************************

class SCHARR_Y_5X5_08U : public ACF_Process_APU
{

public:
   SCHARR_Y_5X5_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("SCHARR_Y_5X5_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("SCHARR_Y_5X5_08U",
                                        SCHARR_Y_5X5_08U_LOAD_SEGMENTS,
                                        SCHARR_Y_5X5_08U_LOAD_PMEM, SCHARR_Y_5X5_08U_LOAD_PMEM_SIZE,
                                        SCHARR_Y_5X5_08U_LOAD_DMEM, SCHARR_Y_5X5_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(SCHARR_Y_5X5_08U_APEX_LOG_BUFFER, SCHARR_Y_5X5_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_SCHARR_Y_5X5_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_SCHARR_Y_5X5_08U

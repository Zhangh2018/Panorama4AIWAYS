#ifndef _ACF_PROCESS_APU_PREWITT_X_3X3
#define _ACF_PROCESS_APU_PREWITT_X_3X3

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <PREWITT_X_3X3_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_PREWITT_X_3X3[] = {{2, 1, 4, 0, {1, 1, 2, 2}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_PREWITT_X_3X3[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_PREWITT_X_3X3[] = {{6, 1, 4, 0, {1, 1, 2, 2}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_PREWITT_X_3X3[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_PREWITT_X_3X3[] = {{10, 1, 4, 0, {1, 1, 2, 2}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_PREWITT_X_3X3[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_PREWITT_X_3X3[] = {{14, 1, 4, 0, {1, 1, 2, 2}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_PREWITT_X_3X3[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_PREWITT_X_3X3[] = {{18, 1, 4, 0, {1, 1, 2, 2}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_PREWITT_X_3X3[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_PREWITT_X_3X3[] = {{22, 1, 4, 0, {1, 1, 2, 2}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_PREWITT_X_3X3[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_PREWITT_X_3X3[] = {{26, 1, 4, 0, {1, 1, 2, 2}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_PREWITT_X_3X3[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_PREWITT_X_3X3[] = {{30, 1, 4, 0, {1, 1, 2, 2}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_PREWITT_X_3X3[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_PREWITT_X_3X3[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_PREWITT_X_3X3[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_PREWITT_X_3X3[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_PREWITT_X_3X3[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_PREWITT_X_3X3[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_PREWITT_X_3X3[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_PREWITT_X_3X3[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_PREWITT_X_3X3[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_PREWITT_X_3X3[] = {{2, 2, 4, 0, {1, 1, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_PREWITT_X_3X3[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_PREWITT_X_3X3[] = {{6, 2, 4, 0, {1, 1, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_PREWITT_X_3X3[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_PREWITT_X_3X3[] = {{10, 2, 4, 0, {1, 1, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_PREWITT_X_3X3[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_PREWITT_X_3X3[] = {{14, 2, 4, 0, {1, 1, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_PREWITT_X_3X3[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_PREWITT_X_3X3[] = {{18, 2, 4, 0, {1, 1, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_PREWITT_X_3X3[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_PREWITT_X_3X3[] = {{22, 2, 4, 0, {1, 1, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_PREWITT_X_3X3[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_PREWITT_X_3X3[] = {{26, 2, 4, 0, {1, 1, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_PREWITT_X_3X3[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_PREWITT_X_3X3[] = {{30, 2, 4, 0, {1, 1, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_PREWITT_X_3X3[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_PREWITT_X_3X3[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_PREWITT_X_3X3[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_PREWITT_X_3X3[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_PREWITT_X_3X3[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_PREWITT_X_3X3[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_PREWITT_X_3X3[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_PREWITT_X_3X3[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_PREWITT_X_3X3[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_PREWITT_X_3X3[] = {{2, 3, 4, 0, {1, 1, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_PREWITT_X_3X3[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_PREWITT_X_3X3[] = {{6, 3, 4, 0, {1, 1, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_PREWITT_X_3X3[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_PREWITT_X_3X3[] = {{10, 3, 4, 0, {1, 1, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_PREWITT_X_3X3[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_PREWITT_X_3X3[] = {{14, 3, 4, 0, {1, 1, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_PREWITT_X_3X3[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_PREWITT_X_3X3[] = {{18, 3, 4, 0, {1, 1, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_PREWITT_X_3X3[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_PREWITT_X_3X3[] = {{22, 3, 4, 0, {1, 1, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_PREWITT_X_3X3[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_PREWITT_X_3X3[] = {{26, 3, 4, 0, {1, 1, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_PREWITT_X_3X3[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_PREWITT_X_3X3[] = {{30, 3, 4, 0, {1, 1, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_PREWITT_X_3X3[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_PREWITT_X_3X3[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_PREWITT_X_3X3[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_PREWITT_X_3X3[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_PREWITT_X_3X3[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_PREWITT_X_3X3[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_PREWITT_X_3X3[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_PREWITT_X_3X3[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_PREWITT_X_3X3[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_PREWITT_X_3X3[] = {{2, 4, 4, 0, {1, 1, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_PREWITT_X_3X3[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_PREWITT_X_3X3[] = {{6, 4, 4, 0, {1, 1, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_PREWITT_X_3X3[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_PREWITT_X_3X3[] = {{10, 4, 4, 0, {1, 1, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_PREWITT_X_3X3[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_PREWITT_X_3X3[] = {{14, 4, 4, 0, {1, 1, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_PREWITT_X_3X3[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_PREWITT_X_3X3[] = {{18, 4, 4, 0, {1, 1, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_PREWITT_X_3X3[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_PREWITT_X_3X3[] = {{22, 4, 4, 0, {1, 1, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_PREWITT_X_3X3[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_PREWITT_X_3X3[] = {{26, 4, 4, 0, {1, 1, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_PREWITT_X_3X3[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_PREWITT_X_3X3[] = {{30, 4, 4, 0, {1, 1, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_PREWITT_X_3X3[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_PREWITT_X_3X3[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_PREWITT_X_3X3[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_PREWITT_X_3X3[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_PREWITT_X_3X3[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_PREWITT_X_3X3[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_PREWITT_X_3X3[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_PREWITT_X_3X3[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_PREWITT_X_3X3[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_PREWITT_X_3X3[] = {{2, 5, 4, 0, {1, 1, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_PREWITT_X_3X3[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_PREWITT_X_3X3[] = {{6, 5, 4, 0, {1, 1, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_PREWITT_X_3X3[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_PREWITT_X_3X3[] = {{10, 5, 4, 0, {1, 1, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_PREWITT_X_3X3[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_PREWITT_X_3X3[] = {{14, 5, 4, 0, {1, 1, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_PREWITT_X_3X3[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_PREWITT_X_3X3[] = {{18, 5, 4, 0, {1, 1, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_PREWITT_X_3X3[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_PREWITT_X_3X3[] = {{22, 5, 4, 0, {1, 1, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_PREWITT_X_3X3[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_PREWITT_X_3X3[] = {{26, 5, 4, 0, {1, 1, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_PREWITT_X_3X3[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_PREWITT_X_3X3[] = {{30, 5, 4, 0, {1, 1, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_PREWITT_X_3X3[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_PREWITT_X_3X3[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_PREWITT_X_3X3[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_PREWITT_X_3X3[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_PREWITT_X_3X3[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_PREWITT_X_3X3[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_PREWITT_X_3X3[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_PREWITT_X_3X3[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_PREWITT_X_3X3[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_PREWITT_X_3X3[] = {{2, 6, 4, 0, {1, 1, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_PREWITT_X_3X3[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_PREWITT_X_3X3[] = {{6, 6, 4, 0, {1, 1, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_PREWITT_X_3X3[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_PREWITT_X_3X3[] = {{10, 6, 4, 0, {1, 1, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_PREWITT_X_3X3[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_PREWITT_X_3X3[] = {{14, 6, 4, 0, {1, 1, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_PREWITT_X_3X3[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_PREWITT_X_3X3[] = {{18, 6, 4, 0, {1, 1, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_PREWITT_X_3X3[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_PREWITT_X_3X3[] = {{22, 6, 4, 0, {1, 1, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_PREWITT_X_3X3[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_PREWITT_X_3X3[] = {{26, 6, 4, 0, {1, 1, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_PREWITT_X_3X3[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_PREWITT_X_3X3[] = {{30, 6, 4, 0, {1, 1, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_PREWITT_X_3X3[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_PREWITT_X_3X3[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_PREWITT_X_3X3[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_PREWITT_X_3X3[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_PREWITT_X_3X3[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_PREWITT_X_3X3[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_PREWITT_X_3X3[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_PREWITT_X_3X3[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_PREWITT_X_3X3[] = {{64, 6, 4, 0, {1, 1, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_PREWITT_X_3X3[] = {{2, 8, 4, 0, {1, 1, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_PREWITT_X_3X3[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_PREWITT_X_3X3[] = {{6, 8, 4, 0, {1, 1, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_PREWITT_X_3X3[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_PREWITT_X_3X3[] = {{10, 8, 4, 0, {1, 1, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_PREWITT_X_3X3[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_PREWITT_X_3X3[] = {{14, 8, 4, 0, {1, 1, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_PREWITT_X_3X3[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_PREWITT_X_3X3[] = {{18, 8, 4, 0, {1, 1, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_PREWITT_X_3X3[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_PREWITT_X_3X3[] = {{22, 8, 4, 0, {1, 1, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_PREWITT_X_3X3[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_PREWITT_X_3X3[] = {{26, 8, 4, 0, {1, 1, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_PREWITT_X_3X3[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_PREWITT_X_3X3[] = {{30, 8, 4, 0, {1, 1, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_PREWITT_X_3X3[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_PREWITT_X_3X3[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_PREWITT_X_3X3[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_PREWITT_X_3X3[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_PREWITT_X_3X3[] = {{48, 8, 4, 0, {1, 1, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_PREWITT_X_3X3[] = {{52, 8, 4, 0, {1, 1, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_PREWITT_X_3X3[] = {{56, 8, 4, 0, {1, 1, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_PREWITT_X_3X3[] = {{2, 10, 4, 0, {1, 1, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_PREWITT_X_3X3[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_PREWITT_X_3X3[] = {{6, 10, 4, 0, {1, 1, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_PREWITT_X_3X3[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_PREWITT_X_3X3[] = {{10, 10, 4, 0, {1, 1, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_PREWITT_X_3X3[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_PREWITT_X_3X3[] = {{14, 10, 4, 0, {1, 1, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_PREWITT_X_3X3[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_PREWITT_X_3X3[] = {{18, 10, 4, 0, {1, 1, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_PREWITT_X_3X3[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_PREWITT_X_3X3[] = {{22, 10, 4, 0, {1, 1, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_PREWITT_X_3X3[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_PREWITT_X_3X3[] = {{26, 10, 4, 0, {1, 1, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_PREWITT_X_3X3[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_PREWITT_X_3X3[] = {{30, 10, 4, 0, {1, 1, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_PREWITT_X_3X3[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_PREWITT_X_3X3[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_PREWITT_X_3X3[] = {{40, 10, 4, 0, {1, 1, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_PREWITT_X_3X3[] = {{44, 10, 4, 0, {1, 1, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_PREWITT_X_3X3[] = {{2, 12, 4, 0, {1, 1, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_PREWITT_X_3X3[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_PREWITT_X_3X3[] = {{6, 12, 4, 0, {1, 1, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_PREWITT_X_3X3[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_PREWITT_X_3X3[] = {{10, 12, 4, 0, {1, 1, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_PREWITT_X_3X3[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_PREWITT_X_3X3[] = {{14, 12, 4, 0, {1, 1, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_PREWITT_X_3X3[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_PREWITT_X_3X3[] = {{18, 12, 4, 0, {1, 1, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_PREWITT_X_3X3[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_PREWITT_X_3X3[] = {{22, 12, 4, 0, {1, 1, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_PREWITT_X_3X3[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_PREWITT_X_3X3[] = {{26, 12, 4, 0, {1, 1, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_PREWITT_X_3X3[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_PREWITT_X_3X3[] = {{30, 12, 4, 0, {1, 1, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_PREWITT_X_3X3[] = {{32, 12, 4, 0, {1, 1, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_PREWITT_X_3X3[] = {{36, 12, 4, 0, {1, 1, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_PREWITT_X_3X3[] = {{2, 14, 4, 0, {1, 1, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_PREWITT_X_3X3[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_PREWITT_X_3X3[] = {{6, 14, 4, 0, {1, 1, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_PREWITT_X_3X3[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_PREWITT_X_3X3[] = {{10, 14, 4, 0, {1, 1, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_PREWITT_X_3X3[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_PREWITT_X_3X3[] = {{14, 14, 4, 0, {1, 1, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_PREWITT_X_3X3[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_PREWITT_X_3X3[] = {{18, 14, 4, 0, {1, 1, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_PREWITT_X_3X3[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_PREWITT_X_3X3[] = {{22, 14, 4, 0, {1, 1, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_PREWITT_X_3X3[] = {{24, 14, 4, 0, {1, 1, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_PREWITT_X_3X3[] = {{26, 14, 4, 0, {1, 1, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_PREWITT_X_3X3[] = {{28, 14, 4, 0, {1, 1, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_PREWITT_X_3X3[] = {{30, 14, 4, 0, {1, 1, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_PREWITT_X_3X3[] = {{2, 16, 4, 0, {1, 1, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_PREWITT_X_3X3[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_PREWITT_X_3X3[] = {{6, 16, 4, 0, {1, 1, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_PREWITT_X_3X3[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_PREWITT_X_3X3[] = {{10, 16, 4, 0, {1, 1, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_PREWITT_X_3X3[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_PREWITT_X_3X3[] = {{14, 16, 4, 0, {1, 1, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_PREWITT_X_3X3[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_PREWITT_X_3X3[] = {{18, 16, 4, 0, {1, 1, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_PREWITT_X_3X3[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_PREWITT_X_3X3[] = {{22, 16, 4, 0, {1, 1, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_PREWITT_X_3X3[] = {{24, 16, 4, 0, {1, 1, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_PREWITT_X_3X3[] = {{26, 16, 4, 0, {1, 1, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_PREWITT_X_3X3[] = {{2, 18, 4, 0, {1, 1, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_PREWITT_X_3X3[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_PREWITT_X_3X3[] = {{6, 18, 4, 0, {1, 1, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_PREWITT_X_3X3[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_PREWITT_X_3X3[] = {{10, 18, 4, 0, {1, 1, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_PREWITT_X_3X3[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_PREWITT_X_3X3[] = {{14, 18, 4, 0, {1, 1, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_PREWITT_X_3X3[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_PREWITT_X_3X3[] = {{18, 18, 4, 0, {1, 1, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_PREWITT_X_3X3[] = {{20, 18, 4, 0, {1, 1, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_PREWITT_X_3X3[] = {{22, 18, 4, 0, {1, 1, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_PREWITT_X_3X3[] = {{24, 18, 4, 0, {1, 1, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_PREWITT_X_3X3[] = {{2, 20, 4, 0, {1, 1, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_PREWITT_X_3X3[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_PREWITT_X_3X3[] = {{6, 20, 4, 0, {1, 1, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_PREWITT_X_3X3[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_PREWITT_X_3X3[] = {{10, 20, 4, 0, {1, 1, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_PREWITT_X_3X3[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_PREWITT_X_3X3[] = {{14, 20, 4, 0, {1, 1, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_PREWITT_X_3X3[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_PREWITT_X_3X3[] = {{18, 20, 4, 0, {1, 1, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_PREWITT_X_3X3[] = {{20, 20, 4, 0, {1, 1, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_PREWITT_X_3X3[] = {{2, 22, 4, 0, {1, 1, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_PREWITT_X_3X3[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_PREWITT_X_3X3[] = {{6, 22, 4, 0, {1, 1, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_PREWITT_X_3X3[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_PREWITT_X_3X3[] = {{10, 22, 4, 0, {1, 1, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_PREWITT_X_3X3[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_PREWITT_X_3X3[] = {{14, 22, 4, 0, {1, 1, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_PREWITT_X_3X3[] = {{16, 22, 4, 0, {1, 1, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_PREWITT_X_3X3[] = {{18, 22, 4, 0, {1, 1, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_PREWITT_X_3X3[] = {{2, 24, 4, 0, {1, 1, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_PREWITT_X_3X3[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_PREWITT_X_3X3[] = {{6, 24, 4, 0, {1, 1, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_PREWITT_X_3X3[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_PREWITT_X_3X3[] = {{10, 24, 4, 0, {1, 1, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_PREWITT_X_3X3[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_PREWITT_X_3X3[] = {{14, 24, 4, 0, {1, 1, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_PREWITT_X_3X3[] = {{16, 24, 4, 0, {1, 1, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_PREWITT_X_3X3[] = {{2, 26, 4, 0, {1, 1, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_PREWITT_X_3X3[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_PREWITT_X_3X3[] = {{6, 26, 4, 0, {1, 1, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_PREWITT_X_3X3[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_PREWITT_X_3X3[] = {{10, 26, 4, 0, {1, 1, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_PREWITT_X_3X3[] = {{12, 26, 4, 0, {1, 1, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_PREWITT_X_3X3[] = {{14, 26, 4, 0, {1, 1, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_PREWITT_X_3X3[] = {{16, 26, 4, 0, {1, 1, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_PREWITT_X_3X3[] = {{2, 28, 4, 0, {1, 1, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_PREWITT_X_3X3[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_PREWITT_X_3X3[] = {{6, 28, 4, 0, {1, 1, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_PREWITT_X_3X3[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_PREWITT_X_3X3[] = {{10, 28, 4, 0, {1, 1, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_PREWITT_X_3X3[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_PREWITT_X_3X3[] = {{14, 28, 4, 0, {1, 1, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_PREWITT_X_3X3[] = {{2, 30, 4, 0, {1, 1, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_PREWITT_X_3X3[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_PREWITT_X_3X3[] = {{6, 30, 4, 0, {1, 1, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_PREWITT_X_3X3[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_PREWITT_X_3X3[] = {{10, 30, 4, 0, {1, 1, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_PREWITT_X_3X3[] = {{12, 30, 4, 0, {1, 1, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_PREWITT_X_3X3[] = {{2, 32, 4, 0, {1, 1, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_PREWITT_X_3X3[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_PREWITT_X_3X3[] = {{6, 32, 4, 0, {1, 1, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_PREWITT_X_3X3[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_PREWITT_X_3X3[] = {{10, 32, 4, 0, {1, 1, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_PREWITT_X_3X3[] = {{12, 32, 4, 0, {1, 1, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_PREWITT_X_3X3[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_PREWITT_X_3X3[] = {{1, 0}};

static acf_scenario gScenarioArray_PREWITT_X_3X3[] = {
{2, 1, 44, 0, 1, gScenarioBufferData0_PREWITT_X_3X3, 32, gScenarioKernelData0_PREWITT_X_3X3, 4},
{4, 1, 64, 0, 1, gScenarioBufferData1_PREWITT_X_3X3, 32, gScenarioKernelData1_PREWITT_X_3X3, 4},
{6, 1, 84, 0, 1, gScenarioBufferData2_PREWITT_X_3X3, 32, gScenarioKernelData2_PREWITT_X_3X3, 4},
{8, 1, 104, 0, 1, gScenarioBufferData3_PREWITT_X_3X3, 32, gScenarioKernelData3_PREWITT_X_3X3, 4},
{10, 1, 124, 0, 1, gScenarioBufferData4_PREWITT_X_3X3, 32, gScenarioKernelData4_PREWITT_X_3X3, 4},
{12, 1, 144, 0, 1, gScenarioBufferData5_PREWITT_X_3X3, 32, gScenarioKernelData5_PREWITT_X_3X3, 4},
{14, 1, 164, 0, 1, gScenarioBufferData6_PREWITT_X_3X3, 32, gScenarioKernelData6_PREWITT_X_3X3, 4},
{16, 1, 184, 0, 1, gScenarioBufferData7_PREWITT_X_3X3, 32, gScenarioKernelData7_PREWITT_X_3X3, 4},
{18, 1, 204, 0, 1, gScenarioBufferData8_PREWITT_X_3X3, 32, gScenarioKernelData8_PREWITT_X_3X3, 4},
{20, 1, 224, 0, 1, gScenarioBufferData9_PREWITT_X_3X3, 32, gScenarioKernelData9_PREWITT_X_3X3, 4},
{22, 1, 244, 0, 1, gScenarioBufferData10_PREWITT_X_3X3, 32, gScenarioKernelData10_PREWITT_X_3X3, 4},
{24, 1, 264, 0, 1, gScenarioBufferData11_PREWITT_X_3X3, 32, gScenarioKernelData11_PREWITT_X_3X3, 4},
{26, 1, 284, 0, 1, gScenarioBufferData12_PREWITT_X_3X3, 32, gScenarioKernelData12_PREWITT_X_3X3, 4},
{28, 1, 304, 0, 1, gScenarioBufferData13_PREWITT_X_3X3, 32, gScenarioKernelData13_PREWITT_X_3X3, 4},
{30, 1, 324, 0, 1, gScenarioBufferData14_PREWITT_X_3X3, 32, gScenarioKernelData14_PREWITT_X_3X3, 4},
{32, 1, 344, 0, 1, gScenarioBufferData15_PREWITT_X_3X3, 32, gScenarioKernelData15_PREWITT_X_3X3, 4},
{36, 1, 384, 0, 1, gScenarioBufferData16_PREWITT_X_3X3, 32, gScenarioKernelData16_PREWITT_X_3X3, 4},
{40, 1, 424, 0, 1, gScenarioBufferData17_PREWITT_X_3X3, 32, gScenarioKernelData17_PREWITT_X_3X3, 4},
{44, 1, 464, 0, 1, gScenarioBufferData18_PREWITT_X_3X3, 32, gScenarioKernelData18_PREWITT_X_3X3, 4},
{48, 1, 504, 0, 1, gScenarioBufferData19_PREWITT_X_3X3, 32, gScenarioKernelData19_PREWITT_X_3X3, 4},
{52, 1, 544, 0, 1, gScenarioBufferData20_PREWITT_X_3X3, 32, gScenarioKernelData20_PREWITT_X_3X3, 4},
{56, 1, 584, 0, 1, gScenarioBufferData21_PREWITT_X_3X3, 32, gScenarioKernelData21_PREWITT_X_3X3, 4},
{60, 1, 624, 0, 1, gScenarioBufferData22_PREWITT_X_3X3, 32, gScenarioKernelData22_PREWITT_X_3X3, 4},
{64, 1, 664, 0, 1, gScenarioBufferData23_PREWITT_X_3X3, 32, gScenarioKernelData23_PREWITT_X_3X3, 4},
{2, 2, 76, 0, 1, gScenarioBufferData24_PREWITT_X_3X3, 32, gScenarioKernelData24_PREWITT_X_3X3, 4},
{4, 2, 112, 0, 1, gScenarioBufferData25_PREWITT_X_3X3, 32, gScenarioKernelData25_PREWITT_X_3X3, 4},
{6, 2, 148, 0, 1, gScenarioBufferData26_PREWITT_X_3X3, 32, gScenarioKernelData26_PREWITT_X_3X3, 4},
{8, 2, 184, 0, 1, gScenarioBufferData27_PREWITT_X_3X3, 32, gScenarioKernelData27_PREWITT_X_3X3, 4},
{10, 2, 220, 0, 1, gScenarioBufferData28_PREWITT_X_3X3, 32, gScenarioKernelData28_PREWITT_X_3X3, 4},
{12, 2, 256, 0, 1, gScenarioBufferData29_PREWITT_X_3X3, 32, gScenarioKernelData29_PREWITT_X_3X3, 4},
{14, 2, 292, 0, 1, gScenarioBufferData30_PREWITT_X_3X3, 32, gScenarioKernelData30_PREWITT_X_3X3, 4},
{16, 2, 328, 0, 1, gScenarioBufferData31_PREWITT_X_3X3, 32, gScenarioKernelData31_PREWITT_X_3X3, 4},
{18, 2, 364, 0, 1, gScenarioBufferData32_PREWITT_X_3X3, 32, gScenarioKernelData32_PREWITT_X_3X3, 4},
{20, 2, 400, 0, 1, gScenarioBufferData33_PREWITT_X_3X3, 32, gScenarioKernelData33_PREWITT_X_3X3, 4},
{22, 2, 436, 0, 1, gScenarioBufferData34_PREWITT_X_3X3, 32, gScenarioKernelData34_PREWITT_X_3X3, 4},
{24, 2, 472, 0, 1, gScenarioBufferData35_PREWITT_X_3X3, 32, gScenarioKernelData35_PREWITT_X_3X3, 4},
{26, 2, 508, 0, 1, gScenarioBufferData36_PREWITT_X_3X3, 32, gScenarioKernelData36_PREWITT_X_3X3, 4},
{28, 2, 544, 0, 1, gScenarioBufferData37_PREWITT_X_3X3, 32, gScenarioKernelData37_PREWITT_X_3X3, 4},
{30, 2, 580, 0, 1, gScenarioBufferData38_PREWITT_X_3X3, 32, gScenarioKernelData38_PREWITT_X_3X3, 4},
{32, 2, 616, 0, 1, gScenarioBufferData39_PREWITT_X_3X3, 32, gScenarioKernelData39_PREWITT_X_3X3, 4},
{36, 2, 688, 0, 1, gScenarioBufferData40_PREWITT_X_3X3, 32, gScenarioKernelData40_PREWITT_X_3X3, 4},
{40, 2, 760, 0, 1, gScenarioBufferData41_PREWITT_X_3X3, 32, gScenarioKernelData41_PREWITT_X_3X3, 4},
{44, 2, 832, 0, 1, gScenarioBufferData42_PREWITT_X_3X3, 32, gScenarioKernelData42_PREWITT_X_3X3, 4},
{48, 2, 904, 0, 1, gScenarioBufferData43_PREWITT_X_3X3, 32, gScenarioKernelData43_PREWITT_X_3X3, 4},
{52, 2, 976, 0, 1, gScenarioBufferData44_PREWITT_X_3X3, 32, gScenarioKernelData44_PREWITT_X_3X3, 4},
{56, 2, 1048, 0, 1, gScenarioBufferData45_PREWITT_X_3X3, 32, gScenarioKernelData45_PREWITT_X_3X3, 4},
{60, 2, 1120, 0, 1, gScenarioBufferData46_PREWITT_X_3X3, 32, gScenarioKernelData46_PREWITT_X_3X3, 4},
{64, 2, 1192, 0, 1, gScenarioBufferData47_PREWITT_X_3X3, 32, gScenarioKernelData47_PREWITT_X_3X3, 4},
{2, 3, 108, 0, 1, gScenarioBufferData48_PREWITT_X_3X3, 32, gScenarioKernelData48_PREWITT_X_3X3, 4},
{4, 3, 160, 0, 1, gScenarioBufferData49_PREWITT_X_3X3, 32, gScenarioKernelData49_PREWITT_X_3X3, 4},
{6, 3, 212, 0, 1, gScenarioBufferData50_PREWITT_X_3X3, 32, gScenarioKernelData50_PREWITT_X_3X3, 4},
{8, 3, 264, 0, 1, gScenarioBufferData51_PREWITT_X_3X3, 32, gScenarioKernelData51_PREWITT_X_3X3, 4},
{10, 3, 316, 0, 1, gScenarioBufferData52_PREWITT_X_3X3, 32, gScenarioKernelData52_PREWITT_X_3X3, 4},
{12, 3, 368, 0, 1, gScenarioBufferData53_PREWITT_X_3X3, 32, gScenarioKernelData53_PREWITT_X_3X3, 4},
{14, 3, 420, 0, 1, gScenarioBufferData54_PREWITT_X_3X3, 32, gScenarioKernelData54_PREWITT_X_3X3, 4},
{16, 3, 472, 0, 1, gScenarioBufferData55_PREWITT_X_3X3, 32, gScenarioKernelData55_PREWITT_X_3X3, 4},
{18, 3, 524, 0, 1, gScenarioBufferData56_PREWITT_X_3X3, 32, gScenarioKernelData56_PREWITT_X_3X3, 4},
{20, 3, 576, 0, 1, gScenarioBufferData57_PREWITT_X_3X3, 32, gScenarioKernelData57_PREWITT_X_3X3, 4},
{22, 3, 628, 0, 1, gScenarioBufferData58_PREWITT_X_3X3, 32, gScenarioKernelData58_PREWITT_X_3X3, 4},
{24, 3, 680, 0, 1, gScenarioBufferData59_PREWITT_X_3X3, 32, gScenarioKernelData59_PREWITT_X_3X3, 4},
{26, 3, 732, 0, 1, gScenarioBufferData60_PREWITT_X_3X3, 32, gScenarioKernelData60_PREWITT_X_3X3, 4},
{28, 3, 784, 0, 1, gScenarioBufferData61_PREWITT_X_3X3, 32, gScenarioKernelData61_PREWITT_X_3X3, 4},
{30, 3, 836, 0, 1, gScenarioBufferData62_PREWITT_X_3X3, 32, gScenarioKernelData62_PREWITT_X_3X3, 4},
{32, 3, 888, 0, 1, gScenarioBufferData63_PREWITT_X_3X3, 32, gScenarioKernelData63_PREWITT_X_3X3, 4},
{36, 3, 992, 0, 1, gScenarioBufferData64_PREWITT_X_3X3, 32, gScenarioKernelData64_PREWITT_X_3X3, 4},
{40, 3, 1096, 0, 1, gScenarioBufferData65_PREWITT_X_3X3, 32, gScenarioKernelData65_PREWITT_X_3X3, 4},
{44, 3, 1200, 0, 1, gScenarioBufferData66_PREWITT_X_3X3, 32, gScenarioKernelData66_PREWITT_X_3X3, 4},
{48, 3, 1304, 0, 1, gScenarioBufferData67_PREWITT_X_3X3, 32, gScenarioKernelData67_PREWITT_X_3X3, 4},
{52, 3, 1408, 0, 1, gScenarioBufferData68_PREWITT_X_3X3, 32, gScenarioKernelData68_PREWITT_X_3X3, 4},
{56, 3, 1512, 0, 1, gScenarioBufferData69_PREWITT_X_3X3, 32, gScenarioKernelData69_PREWITT_X_3X3, 4},
{60, 3, 1616, 0, 1, gScenarioBufferData70_PREWITT_X_3X3, 32, gScenarioKernelData70_PREWITT_X_3X3, 4},
{64, 3, 1720, 0, 1, gScenarioBufferData71_PREWITT_X_3X3, 32, gScenarioKernelData71_PREWITT_X_3X3, 4},
{2, 4, 140, 0, 1, gScenarioBufferData72_PREWITT_X_3X3, 32, gScenarioKernelData72_PREWITT_X_3X3, 4},
{4, 4, 208, 0, 1, gScenarioBufferData73_PREWITT_X_3X3, 32, gScenarioKernelData73_PREWITT_X_3X3, 4},
{6, 4, 276, 0, 1, gScenarioBufferData74_PREWITT_X_3X3, 32, gScenarioKernelData74_PREWITT_X_3X3, 4},
{8, 4, 344, 0, 1, gScenarioBufferData75_PREWITT_X_3X3, 32, gScenarioKernelData75_PREWITT_X_3X3, 4},
{10, 4, 412, 0, 1, gScenarioBufferData76_PREWITT_X_3X3, 32, gScenarioKernelData76_PREWITT_X_3X3, 4},
{12, 4, 480, 0, 1, gScenarioBufferData77_PREWITT_X_3X3, 32, gScenarioKernelData77_PREWITT_X_3X3, 4},
{14, 4, 548, 0, 1, gScenarioBufferData78_PREWITT_X_3X3, 32, gScenarioKernelData78_PREWITT_X_3X3, 4},
{16, 4, 616, 0, 1, gScenarioBufferData79_PREWITT_X_3X3, 32, gScenarioKernelData79_PREWITT_X_3X3, 4},
{18, 4, 684, 0, 1, gScenarioBufferData80_PREWITT_X_3X3, 32, gScenarioKernelData80_PREWITT_X_3X3, 4},
{20, 4, 752, 0, 1, gScenarioBufferData81_PREWITT_X_3X3, 32, gScenarioKernelData81_PREWITT_X_3X3, 4},
{22, 4, 820, 0, 1, gScenarioBufferData82_PREWITT_X_3X3, 32, gScenarioKernelData82_PREWITT_X_3X3, 4},
{24, 4, 888, 0, 1, gScenarioBufferData83_PREWITT_X_3X3, 32, gScenarioKernelData83_PREWITT_X_3X3, 4},
{26, 4, 956, 0, 1, gScenarioBufferData84_PREWITT_X_3X3, 32, gScenarioKernelData84_PREWITT_X_3X3, 4},
{28, 4, 1024, 0, 1, gScenarioBufferData85_PREWITT_X_3X3, 32, gScenarioKernelData85_PREWITT_X_3X3, 4},
{30, 4, 1092, 0, 1, gScenarioBufferData86_PREWITT_X_3X3, 32, gScenarioKernelData86_PREWITT_X_3X3, 4},
{32, 4, 1160, 0, 1, gScenarioBufferData87_PREWITT_X_3X3, 32, gScenarioKernelData87_PREWITT_X_3X3, 4},
{36, 4, 1296, 0, 1, gScenarioBufferData88_PREWITT_X_3X3, 32, gScenarioKernelData88_PREWITT_X_3X3, 4},
{40, 4, 1432, 0, 1, gScenarioBufferData89_PREWITT_X_3X3, 32, gScenarioKernelData89_PREWITT_X_3X3, 4},
{44, 4, 1568, 0, 1, gScenarioBufferData90_PREWITT_X_3X3, 32, gScenarioKernelData90_PREWITT_X_3X3, 4},
{48, 4, 1704, 0, 1, gScenarioBufferData91_PREWITT_X_3X3, 32, gScenarioKernelData91_PREWITT_X_3X3, 4},
{52, 4, 1840, 0, 1, gScenarioBufferData92_PREWITT_X_3X3, 32, gScenarioKernelData92_PREWITT_X_3X3, 4},
{56, 4, 1976, 0, 1, gScenarioBufferData93_PREWITT_X_3X3, 32, gScenarioKernelData93_PREWITT_X_3X3, 4},
{60, 4, 2112, 0, 1, gScenarioBufferData94_PREWITT_X_3X3, 32, gScenarioKernelData94_PREWITT_X_3X3, 4},
{64, 4, 2248, 0, 1, gScenarioBufferData95_PREWITT_X_3X3, 32, gScenarioKernelData95_PREWITT_X_3X3, 4},
{2, 5, 172, 0, 1, gScenarioBufferData96_PREWITT_X_3X3, 32, gScenarioKernelData96_PREWITT_X_3X3, 4},
{4, 5, 256, 0, 1, gScenarioBufferData97_PREWITT_X_3X3, 32, gScenarioKernelData97_PREWITT_X_3X3, 4},
{6, 5, 340, 0, 1, gScenarioBufferData98_PREWITT_X_3X3, 32, gScenarioKernelData98_PREWITT_X_3X3, 4},
{8, 5, 424, 0, 1, gScenarioBufferData99_PREWITT_X_3X3, 32, gScenarioKernelData99_PREWITT_X_3X3, 4},
{10, 5, 508, 0, 1, gScenarioBufferData100_PREWITT_X_3X3, 32, gScenarioKernelData100_PREWITT_X_3X3, 4},
{12, 5, 592, 0, 1, gScenarioBufferData101_PREWITT_X_3X3, 32, gScenarioKernelData101_PREWITT_X_3X3, 4},
{14, 5, 676, 0, 1, gScenarioBufferData102_PREWITT_X_3X3, 32, gScenarioKernelData102_PREWITT_X_3X3, 4},
{16, 5, 760, 0, 1, gScenarioBufferData103_PREWITT_X_3X3, 32, gScenarioKernelData103_PREWITT_X_3X3, 4},
{18, 5, 844, 0, 1, gScenarioBufferData104_PREWITT_X_3X3, 32, gScenarioKernelData104_PREWITT_X_3X3, 4},
{20, 5, 928, 0, 1, gScenarioBufferData105_PREWITT_X_3X3, 32, gScenarioKernelData105_PREWITT_X_3X3, 4},
{22, 5, 1012, 0, 1, gScenarioBufferData106_PREWITT_X_3X3, 32, gScenarioKernelData106_PREWITT_X_3X3, 4},
{24, 5, 1096, 0, 1, gScenarioBufferData107_PREWITT_X_3X3, 32, gScenarioKernelData107_PREWITT_X_3X3, 4},
{26, 5, 1180, 0, 1, gScenarioBufferData108_PREWITT_X_3X3, 32, gScenarioKernelData108_PREWITT_X_3X3, 4},
{28, 5, 1264, 0, 1, gScenarioBufferData109_PREWITT_X_3X3, 32, gScenarioKernelData109_PREWITT_X_3X3, 4},
{30, 5, 1348, 0, 1, gScenarioBufferData110_PREWITT_X_3X3, 32, gScenarioKernelData110_PREWITT_X_3X3, 4},
{32, 5, 1432, 0, 1, gScenarioBufferData111_PREWITT_X_3X3, 32, gScenarioKernelData111_PREWITT_X_3X3, 4},
{36, 5, 1600, 0, 1, gScenarioBufferData112_PREWITT_X_3X3, 32, gScenarioKernelData112_PREWITT_X_3X3, 4},
{40, 5, 1768, 0, 1, gScenarioBufferData113_PREWITT_X_3X3, 32, gScenarioKernelData113_PREWITT_X_3X3, 4},
{44, 5, 1936, 0, 1, gScenarioBufferData114_PREWITT_X_3X3, 32, gScenarioKernelData114_PREWITT_X_3X3, 4},
{48, 5, 2104, 0, 1, gScenarioBufferData115_PREWITT_X_3X3, 32, gScenarioKernelData115_PREWITT_X_3X3, 4},
{52, 5, 2272, 0, 1, gScenarioBufferData116_PREWITT_X_3X3, 32, gScenarioKernelData116_PREWITT_X_3X3, 4},
{56, 5, 2440, 0, 1, gScenarioBufferData117_PREWITT_X_3X3, 32, gScenarioKernelData117_PREWITT_X_3X3, 4},
{60, 5, 2608, 0, 1, gScenarioBufferData118_PREWITT_X_3X3, 32, gScenarioKernelData118_PREWITT_X_3X3, 4},
{64, 5, 2776, 0, 1, gScenarioBufferData119_PREWITT_X_3X3, 32, gScenarioKernelData119_PREWITT_X_3X3, 4},
{2, 6, 204, 0, 1, gScenarioBufferData120_PREWITT_X_3X3, 32, gScenarioKernelData120_PREWITT_X_3X3, 4},
{4, 6, 304, 0, 1, gScenarioBufferData121_PREWITT_X_3X3, 32, gScenarioKernelData121_PREWITT_X_3X3, 4},
{6, 6, 404, 0, 1, gScenarioBufferData122_PREWITT_X_3X3, 32, gScenarioKernelData122_PREWITT_X_3X3, 4},
{8, 6, 504, 0, 1, gScenarioBufferData123_PREWITT_X_3X3, 32, gScenarioKernelData123_PREWITT_X_3X3, 4},
{10, 6, 604, 0, 1, gScenarioBufferData124_PREWITT_X_3X3, 32, gScenarioKernelData124_PREWITT_X_3X3, 4},
{12, 6, 704, 0, 1, gScenarioBufferData125_PREWITT_X_3X3, 32, gScenarioKernelData125_PREWITT_X_3X3, 4},
{14, 6, 804, 0, 1, gScenarioBufferData126_PREWITT_X_3X3, 32, gScenarioKernelData126_PREWITT_X_3X3, 4},
{16, 6, 904, 0, 1, gScenarioBufferData127_PREWITT_X_3X3, 32, gScenarioKernelData127_PREWITT_X_3X3, 4},
{18, 6, 1004, 0, 1, gScenarioBufferData128_PREWITT_X_3X3, 32, gScenarioKernelData128_PREWITT_X_3X3, 4},
{20, 6, 1104, 0, 1, gScenarioBufferData129_PREWITT_X_3X3, 32, gScenarioKernelData129_PREWITT_X_3X3, 4},
{22, 6, 1204, 0, 1, gScenarioBufferData130_PREWITT_X_3X3, 32, gScenarioKernelData130_PREWITT_X_3X3, 4},
{24, 6, 1304, 0, 1, gScenarioBufferData131_PREWITT_X_3X3, 32, gScenarioKernelData131_PREWITT_X_3X3, 4},
{26, 6, 1404, 0, 1, gScenarioBufferData132_PREWITT_X_3X3, 32, gScenarioKernelData132_PREWITT_X_3X3, 4},
{28, 6, 1504, 0, 1, gScenarioBufferData133_PREWITT_X_3X3, 32, gScenarioKernelData133_PREWITT_X_3X3, 4},
{30, 6, 1604, 0, 1, gScenarioBufferData134_PREWITT_X_3X3, 32, gScenarioKernelData134_PREWITT_X_3X3, 4},
{32, 6, 1704, 0, 1, gScenarioBufferData135_PREWITT_X_3X3, 32, gScenarioKernelData135_PREWITT_X_3X3, 4},
{36, 6, 1904, 0, 1, gScenarioBufferData136_PREWITT_X_3X3, 32, gScenarioKernelData136_PREWITT_X_3X3, 4},
{40, 6, 2104, 0, 1, gScenarioBufferData137_PREWITT_X_3X3, 32, gScenarioKernelData137_PREWITT_X_3X3, 4},
{44, 6, 2304, 0, 1, gScenarioBufferData138_PREWITT_X_3X3, 32, gScenarioKernelData138_PREWITT_X_3X3, 4},
{48, 6, 2504, 0, 1, gScenarioBufferData139_PREWITT_X_3X3, 32, gScenarioKernelData139_PREWITT_X_3X3, 4},
{52, 6, 2704, 0, 1, gScenarioBufferData140_PREWITT_X_3X3, 32, gScenarioKernelData140_PREWITT_X_3X3, 4},
{56, 6, 2904, 0, 1, gScenarioBufferData141_PREWITT_X_3X3, 32, gScenarioKernelData141_PREWITT_X_3X3, 4},
{60, 6, 3104, 0, 1, gScenarioBufferData142_PREWITT_X_3X3, 32, gScenarioKernelData142_PREWITT_X_3X3, 4},
{64, 6, 3304, 0, 1, gScenarioBufferData143_PREWITT_X_3X3, 32, gScenarioKernelData143_PREWITT_X_3X3, 4},
{2, 8, 268, 0, 1, gScenarioBufferData144_PREWITT_X_3X3, 32, gScenarioKernelData144_PREWITT_X_3X3, 4},
{4, 8, 400, 0, 1, gScenarioBufferData145_PREWITT_X_3X3, 32, gScenarioKernelData145_PREWITT_X_3X3, 4},
{6, 8, 532, 0, 1, gScenarioBufferData146_PREWITT_X_3X3, 32, gScenarioKernelData146_PREWITT_X_3X3, 4},
{8, 8, 664, 0, 1, gScenarioBufferData147_PREWITT_X_3X3, 32, gScenarioKernelData147_PREWITT_X_3X3, 4},
{10, 8, 796, 0, 1, gScenarioBufferData148_PREWITT_X_3X3, 32, gScenarioKernelData148_PREWITT_X_3X3, 4},
{12, 8, 928, 0, 1, gScenarioBufferData149_PREWITT_X_3X3, 32, gScenarioKernelData149_PREWITT_X_3X3, 4},
{14, 8, 1060, 0, 1, gScenarioBufferData150_PREWITT_X_3X3, 32, gScenarioKernelData150_PREWITT_X_3X3, 4},
{16, 8, 1192, 0, 1, gScenarioBufferData151_PREWITT_X_3X3, 32, gScenarioKernelData151_PREWITT_X_3X3, 4},
{18, 8, 1324, 0, 1, gScenarioBufferData152_PREWITT_X_3X3, 32, gScenarioKernelData152_PREWITT_X_3X3, 4},
{20, 8, 1456, 0, 1, gScenarioBufferData153_PREWITT_X_3X3, 32, gScenarioKernelData153_PREWITT_X_3X3, 4},
{22, 8, 1588, 0, 1, gScenarioBufferData154_PREWITT_X_3X3, 32, gScenarioKernelData154_PREWITT_X_3X3, 4},
{24, 8, 1720, 0, 1, gScenarioBufferData155_PREWITT_X_3X3, 32, gScenarioKernelData155_PREWITT_X_3X3, 4},
{26, 8, 1852, 0, 1, gScenarioBufferData156_PREWITT_X_3X3, 32, gScenarioKernelData156_PREWITT_X_3X3, 4},
{28, 8, 1984, 0, 1, gScenarioBufferData157_PREWITT_X_3X3, 32, gScenarioKernelData157_PREWITT_X_3X3, 4},
{30, 8, 2116, 0, 1, gScenarioBufferData158_PREWITT_X_3X3, 32, gScenarioKernelData158_PREWITT_X_3X3, 4},
{32, 8, 2248, 0, 1, gScenarioBufferData159_PREWITT_X_3X3, 32, gScenarioKernelData159_PREWITT_X_3X3, 4},
{36, 8, 2512, 0, 1, gScenarioBufferData160_PREWITT_X_3X3, 32, gScenarioKernelData160_PREWITT_X_3X3, 4},
{40, 8, 2776, 0, 1, gScenarioBufferData161_PREWITT_X_3X3, 32, gScenarioKernelData161_PREWITT_X_3X3, 4},
{44, 8, 3040, 0, 1, gScenarioBufferData162_PREWITT_X_3X3, 32, gScenarioKernelData162_PREWITT_X_3X3, 4},
{48, 8, 3304, 0, 1, gScenarioBufferData163_PREWITT_X_3X3, 32, gScenarioKernelData163_PREWITT_X_3X3, 4},
{52, 8, 3568, 0, 1, gScenarioBufferData164_PREWITT_X_3X3, 32, gScenarioKernelData164_PREWITT_X_3X3, 4},
{56, 8, 3832, 0, 1, gScenarioBufferData165_PREWITT_X_3X3, 32, gScenarioKernelData165_PREWITT_X_3X3, 4},
{2, 10, 332, 0, 1, gScenarioBufferData166_PREWITT_X_3X3, 32, gScenarioKernelData166_PREWITT_X_3X3, 4},
{4, 10, 496, 0, 1, gScenarioBufferData167_PREWITT_X_3X3, 32, gScenarioKernelData167_PREWITT_X_3X3, 4},
{6, 10, 660, 0, 1, gScenarioBufferData168_PREWITT_X_3X3, 32, gScenarioKernelData168_PREWITT_X_3X3, 4},
{8, 10, 824, 0, 1, gScenarioBufferData169_PREWITT_X_3X3, 32, gScenarioKernelData169_PREWITT_X_3X3, 4},
{10, 10, 988, 0, 1, gScenarioBufferData170_PREWITT_X_3X3, 32, gScenarioKernelData170_PREWITT_X_3X3, 4},
{12, 10, 1152, 0, 1, gScenarioBufferData171_PREWITT_X_3X3, 32, gScenarioKernelData171_PREWITT_X_3X3, 4},
{14, 10, 1316, 0, 1, gScenarioBufferData172_PREWITT_X_3X3, 32, gScenarioKernelData172_PREWITT_X_3X3, 4},
{16, 10, 1480, 0, 1, gScenarioBufferData173_PREWITT_X_3X3, 32, gScenarioKernelData173_PREWITT_X_3X3, 4},
{18, 10, 1644, 0, 1, gScenarioBufferData174_PREWITT_X_3X3, 32, gScenarioKernelData174_PREWITT_X_3X3, 4},
{20, 10, 1808, 0, 1, gScenarioBufferData175_PREWITT_X_3X3, 32, gScenarioKernelData175_PREWITT_X_3X3, 4},
{22, 10, 1972, 0, 1, gScenarioBufferData176_PREWITT_X_3X3, 32, gScenarioKernelData176_PREWITT_X_3X3, 4},
{24, 10, 2136, 0, 1, gScenarioBufferData177_PREWITT_X_3X3, 32, gScenarioKernelData177_PREWITT_X_3X3, 4},
{26, 10, 2300, 0, 1, gScenarioBufferData178_PREWITT_X_3X3, 32, gScenarioKernelData178_PREWITT_X_3X3, 4},
{28, 10, 2464, 0, 1, gScenarioBufferData179_PREWITT_X_3X3, 32, gScenarioKernelData179_PREWITT_X_3X3, 4},
{30, 10, 2628, 0, 1, gScenarioBufferData180_PREWITT_X_3X3, 32, gScenarioKernelData180_PREWITT_X_3X3, 4},
{32, 10, 2792, 0, 1, gScenarioBufferData181_PREWITT_X_3X3, 32, gScenarioKernelData181_PREWITT_X_3X3, 4},
{36, 10, 3120, 0, 1, gScenarioBufferData182_PREWITT_X_3X3, 32, gScenarioKernelData182_PREWITT_X_3X3, 4},
{40, 10, 3448, 0, 1, gScenarioBufferData183_PREWITT_X_3X3, 32, gScenarioKernelData183_PREWITT_X_3X3, 4},
{44, 10, 3776, 0, 1, gScenarioBufferData184_PREWITT_X_3X3, 32, gScenarioKernelData184_PREWITT_X_3X3, 4},
{2, 12, 396, 0, 1, gScenarioBufferData185_PREWITT_X_3X3, 32, gScenarioKernelData185_PREWITT_X_3X3, 4},
{4, 12, 592, 0, 1, gScenarioBufferData186_PREWITT_X_3X3, 32, gScenarioKernelData186_PREWITT_X_3X3, 4},
{6, 12, 788, 0, 1, gScenarioBufferData187_PREWITT_X_3X3, 32, gScenarioKernelData187_PREWITT_X_3X3, 4},
{8, 12, 984, 0, 1, gScenarioBufferData188_PREWITT_X_3X3, 32, gScenarioKernelData188_PREWITT_X_3X3, 4},
{10, 12, 1180, 0, 1, gScenarioBufferData189_PREWITT_X_3X3, 32, gScenarioKernelData189_PREWITT_X_3X3, 4},
{12, 12, 1376, 0, 1, gScenarioBufferData190_PREWITT_X_3X3, 32, gScenarioKernelData190_PREWITT_X_3X3, 4},
{14, 12, 1572, 0, 1, gScenarioBufferData191_PREWITT_X_3X3, 32, gScenarioKernelData191_PREWITT_X_3X3, 4},
{16, 12, 1768, 0, 1, gScenarioBufferData192_PREWITT_X_3X3, 32, gScenarioKernelData192_PREWITT_X_3X3, 4},
{18, 12, 1964, 0, 1, gScenarioBufferData193_PREWITT_X_3X3, 32, gScenarioKernelData193_PREWITT_X_3X3, 4},
{20, 12, 2160, 0, 1, gScenarioBufferData194_PREWITT_X_3X3, 32, gScenarioKernelData194_PREWITT_X_3X3, 4},
{22, 12, 2356, 0, 1, gScenarioBufferData195_PREWITT_X_3X3, 32, gScenarioKernelData195_PREWITT_X_3X3, 4},
{24, 12, 2552, 0, 1, gScenarioBufferData196_PREWITT_X_3X3, 32, gScenarioKernelData196_PREWITT_X_3X3, 4},
{26, 12, 2748, 0, 1, gScenarioBufferData197_PREWITT_X_3X3, 32, gScenarioKernelData197_PREWITT_X_3X3, 4},
{28, 12, 2944, 0, 1, gScenarioBufferData198_PREWITT_X_3X3, 32, gScenarioKernelData198_PREWITT_X_3X3, 4},
{30, 12, 3140, 0, 1, gScenarioBufferData199_PREWITT_X_3X3, 32, gScenarioKernelData199_PREWITT_X_3X3, 4},
{32, 12, 3336, 0, 1, gScenarioBufferData200_PREWITT_X_3X3, 32, gScenarioKernelData200_PREWITT_X_3X3, 4},
{36, 12, 3728, 0, 1, gScenarioBufferData201_PREWITT_X_3X3, 32, gScenarioKernelData201_PREWITT_X_3X3, 4},
{2, 14, 460, 0, 1, gScenarioBufferData202_PREWITT_X_3X3, 32, gScenarioKernelData202_PREWITT_X_3X3, 4},
{4, 14, 688, 0, 1, gScenarioBufferData203_PREWITT_X_3X3, 32, gScenarioKernelData203_PREWITT_X_3X3, 4},
{6, 14, 916, 0, 1, gScenarioBufferData204_PREWITT_X_3X3, 32, gScenarioKernelData204_PREWITT_X_3X3, 4},
{8, 14, 1144, 0, 1, gScenarioBufferData205_PREWITT_X_3X3, 32, gScenarioKernelData205_PREWITT_X_3X3, 4},
{10, 14, 1372, 0, 1, gScenarioBufferData206_PREWITT_X_3X3, 32, gScenarioKernelData206_PREWITT_X_3X3, 4},
{12, 14, 1600, 0, 1, gScenarioBufferData207_PREWITT_X_3X3, 32, gScenarioKernelData207_PREWITT_X_3X3, 4},
{14, 14, 1828, 0, 1, gScenarioBufferData208_PREWITT_X_3X3, 32, gScenarioKernelData208_PREWITT_X_3X3, 4},
{16, 14, 2056, 0, 1, gScenarioBufferData209_PREWITT_X_3X3, 32, gScenarioKernelData209_PREWITT_X_3X3, 4},
{18, 14, 2284, 0, 1, gScenarioBufferData210_PREWITT_X_3X3, 32, gScenarioKernelData210_PREWITT_X_3X3, 4},
{20, 14, 2512, 0, 1, gScenarioBufferData211_PREWITT_X_3X3, 32, gScenarioKernelData211_PREWITT_X_3X3, 4},
{22, 14, 2740, 0, 1, gScenarioBufferData212_PREWITT_X_3X3, 32, gScenarioKernelData212_PREWITT_X_3X3, 4},
{24, 14, 2968, 0, 1, gScenarioBufferData213_PREWITT_X_3X3, 32, gScenarioKernelData213_PREWITT_X_3X3, 4},
{26, 14, 3196, 0, 1, gScenarioBufferData214_PREWITT_X_3X3, 32, gScenarioKernelData214_PREWITT_X_3X3, 4},
{28, 14, 3424, 0, 1, gScenarioBufferData215_PREWITT_X_3X3, 32, gScenarioKernelData215_PREWITT_X_3X3, 4},
{30, 14, 3652, 0, 1, gScenarioBufferData216_PREWITT_X_3X3, 32, gScenarioKernelData216_PREWITT_X_3X3, 4},
{2, 16, 524, 0, 1, gScenarioBufferData217_PREWITT_X_3X3, 32, gScenarioKernelData217_PREWITT_X_3X3, 4},
{4, 16, 784, 0, 1, gScenarioBufferData218_PREWITT_X_3X3, 32, gScenarioKernelData218_PREWITT_X_3X3, 4},
{6, 16, 1044, 0, 1, gScenarioBufferData219_PREWITT_X_3X3, 32, gScenarioKernelData219_PREWITT_X_3X3, 4},
{8, 16, 1304, 0, 1, gScenarioBufferData220_PREWITT_X_3X3, 32, gScenarioKernelData220_PREWITT_X_3X3, 4},
{10, 16, 1564, 0, 1, gScenarioBufferData221_PREWITT_X_3X3, 32, gScenarioKernelData221_PREWITT_X_3X3, 4},
{12, 16, 1824, 0, 1, gScenarioBufferData222_PREWITT_X_3X3, 32, gScenarioKernelData222_PREWITT_X_3X3, 4},
{14, 16, 2084, 0, 1, gScenarioBufferData223_PREWITT_X_3X3, 32, gScenarioKernelData223_PREWITT_X_3X3, 4},
{16, 16, 2344, 0, 1, gScenarioBufferData224_PREWITT_X_3X3, 32, gScenarioKernelData224_PREWITT_X_3X3, 4},
{18, 16, 2604, 0, 1, gScenarioBufferData225_PREWITT_X_3X3, 32, gScenarioKernelData225_PREWITT_X_3X3, 4},
{20, 16, 2864, 0, 1, gScenarioBufferData226_PREWITT_X_3X3, 32, gScenarioKernelData226_PREWITT_X_3X3, 4},
{22, 16, 3124, 0, 1, gScenarioBufferData227_PREWITT_X_3X3, 32, gScenarioKernelData227_PREWITT_X_3X3, 4},
{24, 16, 3384, 0, 1, gScenarioBufferData228_PREWITT_X_3X3, 32, gScenarioKernelData228_PREWITT_X_3X3, 4},
{26, 16, 3644, 0, 1, gScenarioBufferData229_PREWITT_X_3X3, 32, gScenarioKernelData229_PREWITT_X_3X3, 4},
{2, 18, 588, 0, 1, gScenarioBufferData230_PREWITT_X_3X3, 32, gScenarioKernelData230_PREWITT_X_3X3, 4},
{4, 18, 880, 0, 1, gScenarioBufferData231_PREWITT_X_3X3, 32, gScenarioKernelData231_PREWITT_X_3X3, 4},
{6, 18, 1172, 0, 1, gScenarioBufferData232_PREWITT_X_3X3, 32, gScenarioKernelData232_PREWITT_X_3X3, 4},
{8, 18, 1464, 0, 1, gScenarioBufferData233_PREWITT_X_3X3, 32, gScenarioKernelData233_PREWITT_X_3X3, 4},
{10, 18, 1756, 0, 1, gScenarioBufferData234_PREWITT_X_3X3, 32, gScenarioKernelData234_PREWITT_X_3X3, 4},
{12, 18, 2048, 0, 1, gScenarioBufferData235_PREWITT_X_3X3, 32, gScenarioKernelData235_PREWITT_X_3X3, 4},
{14, 18, 2340, 0, 1, gScenarioBufferData236_PREWITT_X_3X3, 32, gScenarioKernelData236_PREWITT_X_3X3, 4},
{16, 18, 2632, 0, 1, gScenarioBufferData237_PREWITT_X_3X3, 32, gScenarioKernelData237_PREWITT_X_3X3, 4},
{18, 18, 2924, 0, 1, gScenarioBufferData238_PREWITT_X_3X3, 32, gScenarioKernelData238_PREWITT_X_3X3, 4},
{20, 18, 3216, 0, 1, gScenarioBufferData239_PREWITT_X_3X3, 32, gScenarioKernelData239_PREWITT_X_3X3, 4},
{22, 18, 3508, 0, 1, gScenarioBufferData240_PREWITT_X_3X3, 32, gScenarioKernelData240_PREWITT_X_3X3, 4},
{24, 18, 3800, 0, 1, gScenarioBufferData241_PREWITT_X_3X3, 32, gScenarioKernelData241_PREWITT_X_3X3, 4},
{2, 20, 652, 0, 1, gScenarioBufferData242_PREWITT_X_3X3, 32, gScenarioKernelData242_PREWITT_X_3X3, 4},
{4, 20, 976, 0, 1, gScenarioBufferData243_PREWITT_X_3X3, 32, gScenarioKernelData243_PREWITT_X_3X3, 4},
{6, 20, 1300, 0, 1, gScenarioBufferData244_PREWITT_X_3X3, 32, gScenarioKernelData244_PREWITT_X_3X3, 4},
{8, 20, 1624, 0, 1, gScenarioBufferData245_PREWITT_X_3X3, 32, gScenarioKernelData245_PREWITT_X_3X3, 4},
{10, 20, 1948, 0, 1, gScenarioBufferData246_PREWITT_X_3X3, 32, gScenarioKernelData246_PREWITT_X_3X3, 4},
{12, 20, 2272, 0, 1, gScenarioBufferData247_PREWITT_X_3X3, 32, gScenarioKernelData247_PREWITT_X_3X3, 4},
{14, 20, 2596, 0, 1, gScenarioBufferData248_PREWITT_X_3X3, 32, gScenarioKernelData248_PREWITT_X_3X3, 4},
{16, 20, 2920, 0, 1, gScenarioBufferData249_PREWITT_X_3X3, 32, gScenarioKernelData249_PREWITT_X_3X3, 4},
{18, 20, 3244, 0, 1, gScenarioBufferData250_PREWITT_X_3X3, 32, gScenarioKernelData250_PREWITT_X_3X3, 4},
{20, 20, 3568, 0, 1, gScenarioBufferData251_PREWITT_X_3X3, 32, gScenarioKernelData251_PREWITT_X_3X3, 4},
{2, 22, 716, 0, 1, gScenarioBufferData252_PREWITT_X_3X3, 32, gScenarioKernelData252_PREWITT_X_3X3, 4},
{4, 22, 1072, 0, 1, gScenarioBufferData253_PREWITT_X_3X3, 32, gScenarioKernelData253_PREWITT_X_3X3, 4},
{6, 22, 1428, 0, 1, gScenarioBufferData254_PREWITT_X_3X3, 32, gScenarioKernelData254_PREWITT_X_3X3, 4},
{8, 22, 1784, 0, 1, gScenarioBufferData255_PREWITT_X_3X3, 32, gScenarioKernelData255_PREWITT_X_3X3, 4},
{10, 22, 2140, 0, 1, gScenarioBufferData256_PREWITT_X_3X3, 32, gScenarioKernelData256_PREWITT_X_3X3, 4},
{12, 22, 2496, 0, 1, gScenarioBufferData257_PREWITT_X_3X3, 32, gScenarioKernelData257_PREWITT_X_3X3, 4},
{14, 22, 2852, 0, 1, gScenarioBufferData258_PREWITT_X_3X3, 32, gScenarioKernelData258_PREWITT_X_3X3, 4},
{16, 22, 3208, 0, 1, gScenarioBufferData259_PREWITT_X_3X3, 32, gScenarioKernelData259_PREWITT_X_3X3, 4},
{18, 22, 3564, 0, 1, gScenarioBufferData260_PREWITT_X_3X3, 32, gScenarioKernelData260_PREWITT_X_3X3, 4},
{2, 24, 780, 0, 1, gScenarioBufferData261_PREWITT_X_3X3, 32, gScenarioKernelData261_PREWITT_X_3X3, 4},
{4, 24, 1168, 0, 1, gScenarioBufferData262_PREWITT_X_3X3, 32, gScenarioKernelData262_PREWITT_X_3X3, 4},
{6, 24, 1556, 0, 1, gScenarioBufferData263_PREWITT_X_3X3, 32, gScenarioKernelData263_PREWITT_X_3X3, 4},
{8, 24, 1944, 0, 1, gScenarioBufferData264_PREWITT_X_3X3, 32, gScenarioKernelData264_PREWITT_X_3X3, 4},
{10, 24, 2332, 0, 1, gScenarioBufferData265_PREWITT_X_3X3, 32, gScenarioKernelData265_PREWITT_X_3X3, 4},
{12, 24, 2720, 0, 1, gScenarioBufferData266_PREWITT_X_3X3, 32, gScenarioKernelData266_PREWITT_X_3X3, 4},
{14, 24, 3108, 0, 1, gScenarioBufferData267_PREWITT_X_3X3, 32, gScenarioKernelData267_PREWITT_X_3X3, 4},
{16, 24, 3496, 0, 1, gScenarioBufferData268_PREWITT_X_3X3, 32, gScenarioKernelData268_PREWITT_X_3X3, 4},
{2, 26, 844, 0, 1, gScenarioBufferData269_PREWITT_X_3X3, 32, gScenarioKernelData269_PREWITT_X_3X3, 4},
{4, 26, 1264, 0, 1, gScenarioBufferData270_PREWITT_X_3X3, 32, gScenarioKernelData270_PREWITT_X_3X3, 4},
{6, 26, 1684, 0, 1, gScenarioBufferData271_PREWITT_X_3X3, 32, gScenarioKernelData271_PREWITT_X_3X3, 4},
{8, 26, 2104, 0, 1, gScenarioBufferData272_PREWITT_X_3X3, 32, gScenarioKernelData272_PREWITT_X_3X3, 4},
{10, 26, 2524, 0, 1, gScenarioBufferData273_PREWITT_X_3X3, 32, gScenarioKernelData273_PREWITT_X_3X3, 4},
{12, 26, 2944, 0, 1, gScenarioBufferData274_PREWITT_X_3X3, 32, gScenarioKernelData274_PREWITT_X_3X3, 4},
{14, 26, 3364, 0, 1, gScenarioBufferData275_PREWITT_X_3X3, 32, gScenarioKernelData275_PREWITT_X_3X3, 4},
{16, 26, 3784, 0, 1, gScenarioBufferData276_PREWITT_X_3X3, 32, gScenarioKernelData276_PREWITT_X_3X3, 4},
{2, 28, 908, 0, 1, gScenarioBufferData277_PREWITT_X_3X3, 32, gScenarioKernelData277_PREWITT_X_3X3, 4},
{4, 28, 1360, 0, 1, gScenarioBufferData278_PREWITT_X_3X3, 32, gScenarioKernelData278_PREWITT_X_3X3, 4},
{6, 28, 1812, 0, 1, gScenarioBufferData279_PREWITT_X_3X3, 32, gScenarioKernelData279_PREWITT_X_3X3, 4},
{8, 28, 2264, 0, 1, gScenarioBufferData280_PREWITT_X_3X3, 32, gScenarioKernelData280_PREWITT_X_3X3, 4},
{10, 28, 2716, 0, 1, gScenarioBufferData281_PREWITT_X_3X3, 32, gScenarioKernelData281_PREWITT_X_3X3, 4},
{12, 28, 3168, 0, 1, gScenarioBufferData282_PREWITT_X_3X3, 32, gScenarioKernelData282_PREWITT_X_3X3, 4},
{14, 28, 3620, 0, 1, gScenarioBufferData283_PREWITT_X_3X3, 32, gScenarioKernelData283_PREWITT_X_3X3, 4},
{2, 30, 972, 0, 1, gScenarioBufferData284_PREWITT_X_3X3, 32, gScenarioKernelData284_PREWITT_X_3X3, 4},
{4, 30, 1456, 0, 1, gScenarioBufferData285_PREWITT_X_3X3, 32, gScenarioKernelData285_PREWITT_X_3X3, 4},
{6, 30, 1940, 0, 1, gScenarioBufferData286_PREWITT_X_3X3, 32, gScenarioKernelData286_PREWITT_X_3X3, 4},
{8, 30, 2424, 0, 1, gScenarioBufferData287_PREWITT_X_3X3, 32, gScenarioKernelData287_PREWITT_X_3X3, 4},
{10, 30, 2908, 0, 1, gScenarioBufferData288_PREWITT_X_3X3, 32, gScenarioKernelData288_PREWITT_X_3X3, 4},
{12, 30, 3392, 0, 1, gScenarioBufferData289_PREWITT_X_3X3, 32, gScenarioKernelData289_PREWITT_X_3X3, 4},
{2, 32, 1036, 0, 1, gScenarioBufferData290_PREWITT_X_3X3, 32, gScenarioKernelData290_PREWITT_X_3X3, 4},
{4, 32, 1552, 0, 1, gScenarioBufferData291_PREWITT_X_3X3, 32, gScenarioKernelData291_PREWITT_X_3X3, 4},
{6, 32, 2068, 0, 1, gScenarioBufferData292_PREWITT_X_3X3, 32, gScenarioKernelData292_PREWITT_X_3X3, 4},
{8, 32, 2584, 0, 1, gScenarioBufferData293_PREWITT_X_3X3, 32, gScenarioKernelData293_PREWITT_X_3X3, 4},
{10, 32, 3100, 0, 1, gScenarioBufferData294_PREWITT_X_3X3, 32, gScenarioKernelData294_PREWITT_X_3X3, 4},
{12, 32, 3616, 0, 1, gScenarioBufferData295_PREWITT_X_3X3, 32, gScenarioKernelData295_PREWITT_X_3X3, 4}
};

static acf_scenario_list gScenarioList_PREWITT_X_3X3 = {
296, //number of scenarios
gScenarioArray_PREWITT_X_3X3};
//**************************************************************

class PREWITT_X_3X3 : public ACF_Process_APU
{

public:
   PREWITT_X_3X3(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("PREWITT_X_3X3");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("PREWITT_X_3X3",
                                        PREWITT_X_3X3_LOAD_SEGMENTS,
                                        PREWITT_X_3X3_LOAD_PMEM, PREWITT_X_3X3_LOAD_PMEM_SIZE,
                                        PREWITT_X_3X3_LOAD_DMEM, PREWITT_X_3X3_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(PREWITT_X_3X3_APEX_LOG_BUFFER, PREWITT_X_3X3_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_PREWITT_X_3X3);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_PREWITT_X_3X3

#ifndef _ACF_PROCESS_APU_CONVOLVE_5X5_16S
#define _ACF_PROCESS_APU_CONVOLVE_5X5_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CONVOLVE_5X5_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CONVOLVE_5X5_16S[] = {{2, 1, 7, 0, {2, 2, 2, 2}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CONVOLVE_5X5_16S[] = {{4, 1, 7, 0, {2, 2, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CONVOLVE_5X5_16S[] = {{6, 1, 7, 0, {2, 2, 2, 2}}, {6, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CONVOLVE_5X5_16S[] = {{8, 1, 7, 0, {2, 2, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CONVOLVE_5X5_16S[] = {{10, 1, 7, 0, {2, 2, 2, 2}}, {10, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CONVOLVE_5X5_16S[] = {{12, 1, 7, 0, {2, 2, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CONVOLVE_5X5_16S[] = {{14, 1, 7, 0, {2, 2, 2, 2}}, {14, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CONVOLVE_5X5_16S[] = {{16, 1, 7, 0, {2, 2, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CONVOLVE_5X5_16S[] = {{18, 1, 7, 0, {2, 2, 2, 2}}, {18, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CONVOLVE_5X5_16S[] = {{20, 1, 7, 0, {2, 2, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CONVOLVE_5X5_16S[] = {{22, 1, 7, 0, {2, 2, 2, 2}}, {22, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CONVOLVE_5X5_16S[] = {{24, 1, 7, 0, {2, 2, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CONVOLVE_5X5_16S[] = {{26, 1, 7, 0, {2, 2, 2, 2}}, {26, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CONVOLVE_5X5_16S[] = {{28, 1, 7, 0, {2, 2, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CONVOLVE_5X5_16S[] = {{30, 1, 7, 0, {2, 2, 2, 2}}, {30, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CONVOLVE_5X5_16S[] = {{32, 1, 7, 0, {2, 2, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CONVOLVE_5X5_16S[] = {{36, 1, 7, 0, {2, 2, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CONVOLVE_5X5_16S[] = {{40, 1, 7, 0, {2, 2, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CONVOLVE_5X5_16S[] = {{44, 1, 7, 0, {2, 2, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CONVOLVE_5X5_16S[] = {{48, 1, 7, 0, {2, 2, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CONVOLVE_5X5_16S[] = {{52, 1, 7, 0, {2, 2, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CONVOLVE_5X5_16S[] = {{56, 1, 7, 0, {2, 2, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CONVOLVE_5X5_16S[] = {{60, 1, 7, 0, {2, 2, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CONVOLVE_5X5_16S[] = {{64, 1, 7, 0, {2, 2, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CONVOLVE_5X5_16S[] = {{2, 2, 4, 0, {2, 2, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CONVOLVE_5X5_16S[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CONVOLVE_5X5_16S[] = {{6, 2, 4, 0, {2, 2, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CONVOLVE_5X5_16S[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CONVOLVE_5X5_16S[] = {{10, 2, 4, 0, {2, 2, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CONVOLVE_5X5_16S[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CONVOLVE_5X5_16S[] = {{14, 2, 4, 0, {2, 2, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CONVOLVE_5X5_16S[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CONVOLVE_5X5_16S[] = {{18, 2, 4, 0, {2, 2, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CONVOLVE_5X5_16S[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CONVOLVE_5X5_16S[] = {{22, 2, 4, 0, {2, 2, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CONVOLVE_5X5_16S[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CONVOLVE_5X5_16S[] = {{26, 2, 4, 0, {2, 2, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CONVOLVE_5X5_16S[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CONVOLVE_5X5_16S[] = {{30, 2, 4, 0, {2, 2, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CONVOLVE_5X5_16S[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CONVOLVE_5X5_16S[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CONVOLVE_5X5_16S[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CONVOLVE_5X5_16S[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CONVOLVE_5X5_16S[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CONVOLVE_5X5_16S[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CONVOLVE_5X5_16S[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CONVOLVE_5X5_16S[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CONVOLVE_5X5_16S[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CONVOLVE_5X5_16S[] = {{2, 3, 4, 0, {2, 2, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CONVOLVE_5X5_16S[] = {{4, 3, 4, 0, {2, 2, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CONVOLVE_5X5_16S[] = {{6, 3, 4, 0, {2, 2, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CONVOLVE_5X5_16S[] = {{8, 3, 4, 0, {2, 2, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CONVOLVE_5X5_16S[] = {{10, 3, 4, 0, {2, 2, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CONVOLVE_5X5_16S[] = {{12, 3, 4, 0, {2, 2, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CONVOLVE_5X5_16S[] = {{14, 3, 4, 0, {2, 2, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CONVOLVE_5X5_16S[] = {{16, 3, 4, 0, {2, 2, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CONVOLVE_5X5_16S[] = {{18, 3, 4, 0, {2, 2, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CONVOLVE_5X5_16S[] = {{20, 3, 4, 0, {2, 2, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CONVOLVE_5X5_16S[] = {{22, 3, 4, 0, {2, 2, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CONVOLVE_5X5_16S[] = {{24, 3, 4, 0, {2, 2, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CONVOLVE_5X5_16S[] = {{26, 3, 4, 0, {2, 2, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CONVOLVE_5X5_16S[] = {{28, 3, 4, 0, {2, 2, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CONVOLVE_5X5_16S[] = {{30, 3, 4, 0, {2, 2, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CONVOLVE_5X5_16S[] = {{32, 3, 4, 0, {2, 2, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CONVOLVE_5X5_16S[] = {{36, 3, 4, 0, {2, 2, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CONVOLVE_5X5_16S[] = {{40, 3, 4, 0, {2, 2, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CONVOLVE_5X5_16S[] = {{44, 3, 4, 0, {2, 2, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CONVOLVE_5X5_16S[] = {{48, 3, 4, 0, {2, 2, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CONVOLVE_5X5_16S[] = {{52, 3, 4, 0, {2, 2, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CONVOLVE_5X5_16S[] = {{56, 3, 4, 0, {2, 2, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CONVOLVE_5X5_16S[] = {{60, 3, 4, 0, {2, 2, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CONVOLVE_5X5_16S[] = {{64, 3, 4, 0, {2, 2, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CONVOLVE_5X5_16S[] = {{2, 4, 4, 0, {2, 2, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CONVOLVE_5X5_16S[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CONVOLVE_5X5_16S[] = {{6, 4, 4, 0, {2, 2, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CONVOLVE_5X5_16S[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CONVOLVE_5X5_16S[] = {{10, 4, 4, 0, {2, 2, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CONVOLVE_5X5_16S[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CONVOLVE_5X5_16S[] = {{14, 4, 4, 0, {2, 2, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CONVOLVE_5X5_16S[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CONVOLVE_5X5_16S[] = {{18, 4, 4, 0, {2, 2, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CONVOLVE_5X5_16S[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CONVOLVE_5X5_16S[] = {{22, 4, 4, 0, {2, 2, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CONVOLVE_5X5_16S[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CONVOLVE_5X5_16S[] = {{26, 4, 4, 0, {2, 2, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CONVOLVE_5X5_16S[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CONVOLVE_5X5_16S[] = {{30, 4, 4, 0, {2, 2, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CONVOLVE_5X5_16S[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CONVOLVE_5X5_16S[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CONVOLVE_5X5_16S[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CONVOLVE_5X5_16S[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CONVOLVE_5X5_16S[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CONVOLVE_5X5_16S[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CONVOLVE_5X5_16S[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CONVOLVE_5X5_16S[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CONVOLVE_5X5_16S[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CONVOLVE_5X5_16S[] = {{2, 5, 4, 0, {2, 2, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CONVOLVE_5X5_16S[] = {{4, 5, 4, 0, {2, 2, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CONVOLVE_5X5_16S[] = {{6, 5, 4, 0, {2, 2, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CONVOLVE_5X5_16S[] = {{8, 5, 4, 0, {2, 2, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CONVOLVE_5X5_16S[] = {{10, 5, 4, 0, {2, 2, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CONVOLVE_5X5_16S[] = {{12, 5, 4, 0, {2, 2, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CONVOLVE_5X5_16S[] = {{14, 5, 4, 0, {2, 2, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CONVOLVE_5X5_16S[] = {{16, 5, 4, 0, {2, 2, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_CONVOLVE_5X5_16S[] = {{18, 5, 4, 0, {2, 2, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_CONVOLVE_5X5_16S[] = {{20, 5, 4, 0, {2, 2, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_CONVOLVE_5X5_16S[] = {{22, 5, 4, 0, {2, 2, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_CONVOLVE_5X5_16S[] = {{24, 5, 4, 0, {2, 2, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_CONVOLVE_5X5_16S[] = {{26, 5, 4, 0, {2, 2, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_CONVOLVE_5X5_16S[] = {{28, 5, 4, 0, {2, 2, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_CONVOLVE_5X5_16S[] = {{30, 5, 4, 0, {2, 2, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_CONVOLVE_5X5_16S[] = {{32, 5, 4, 0, {2, 2, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_CONVOLVE_5X5_16S[] = {{36, 5, 4, 0, {2, 2, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_CONVOLVE_5X5_16S[] = {{40, 5, 4, 0, {2, 2, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_CONVOLVE_5X5_16S[] = {{44, 5, 4, 0, {2, 2, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_CONVOLVE_5X5_16S[] = {{48, 5, 4, 0, {2, 2, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_CONVOLVE_5X5_16S[] = {{52, 5, 4, 0, {2, 2, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_CONVOLVE_5X5_16S[] = {{56, 5, 4, 0, {2, 2, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_CONVOLVE_5X5_16S[] = {{60, 5, 4, 0, {2, 2, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_CONVOLVE_5X5_16S[] = {{64, 5, 4, 0, {2, 2, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_CONVOLVE_5X5_16S[] = {{2, 6, 4, 0, {2, 2, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_CONVOLVE_5X5_16S[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_CONVOLVE_5X5_16S[] = {{6, 6, 4, 0, {2, 2, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_CONVOLVE_5X5_16S[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_CONVOLVE_5X5_16S[] = {{10, 6, 4, 0, {2, 2, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_CONVOLVE_5X5_16S[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_CONVOLVE_5X5_16S[] = {{14, 6, 4, 0, {2, 2, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_CONVOLVE_5X5_16S[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_CONVOLVE_5X5_16S[] = {{18, 6, 4, 0, {2, 2, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_CONVOLVE_5X5_16S[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_CONVOLVE_5X5_16S[] = {{22, 6, 4, 0, {2, 2, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_CONVOLVE_5X5_16S[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_CONVOLVE_5X5_16S[] = {{26, 6, 4, 0, {2, 2, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_CONVOLVE_5X5_16S[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_CONVOLVE_5X5_16S[] = {{30, 6, 4, 0, {2, 2, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_CONVOLVE_5X5_16S[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_CONVOLVE_5X5_16S[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_CONVOLVE_5X5_16S[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_CONVOLVE_5X5_16S[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_CONVOLVE_5X5_16S[] = {{48, 6, 4, 0, {2, 2, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_CONVOLVE_5X5_16S[] = {{52, 6, 4, 0, {2, 2, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_CONVOLVE_5X5_16S[] = {{56, 6, 4, 0, {2, 2, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_CONVOLVE_5X5_16S[] = {{60, 6, 4, 0, {2, 2, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_CONVOLVE_5X5_16S[] = {{64, 6, 4, 0, {2, 2, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_CONVOLVE_5X5_16S[] = {{2, 8, 4, 0, {2, 2, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_CONVOLVE_5X5_16S[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_CONVOLVE_5X5_16S[] = {{6, 8, 4, 0, {2, 2, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_CONVOLVE_5X5_16S[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_CONVOLVE_5X5_16S[] = {{10, 8, 4, 0, {2, 2, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_CONVOLVE_5X5_16S[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_CONVOLVE_5X5_16S[] = {{14, 8, 4, 0, {2, 2, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_CONVOLVE_5X5_16S[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_CONVOLVE_5X5_16S[] = {{18, 8, 4, 0, {2, 2, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_CONVOLVE_5X5_16S[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_CONVOLVE_5X5_16S[] = {{22, 8, 4, 0, {2, 2, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_CONVOLVE_5X5_16S[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_CONVOLVE_5X5_16S[] = {{26, 8, 4, 0, {2, 2, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_CONVOLVE_5X5_16S[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_CONVOLVE_5X5_16S[] = {{30, 8, 4, 0, {2, 2, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_CONVOLVE_5X5_16S[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_CONVOLVE_5X5_16S[] = {{36, 8, 4, 0, {2, 2, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_CONVOLVE_5X5_16S[] = {{40, 8, 4, 0, {2, 2, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_CONVOLVE_5X5_16S[] = {{44, 8, 4, 0, {2, 2, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_CONVOLVE_5X5_16S[] = {{48, 8, 4, 0, {2, 2, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_CONVOLVE_5X5_16S[] = {{52, 8, 4, 0, {2, 2, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_CONVOLVE_5X5_16S[] = {{2, 10, 4, 0, {2, 2, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_CONVOLVE_5X5_16S[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_CONVOLVE_5X5_16S[] = {{6, 10, 4, 0, {2, 2, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_CONVOLVE_5X5_16S[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_CONVOLVE_5X5_16S[] = {{10, 10, 4, 0, {2, 2, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_CONVOLVE_5X5_16S[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_CONVOLVE_5X5_16S[] = {{14, 10, 4, 0, {2, 2, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_CONVOLVE_5X5_16S[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_CONVOLVE_5X5_16S[] = {{18, 10, 4, 0, {2, 2, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_CONVOLVE_5X5_16S[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_CONVOLVE_5X5_16S[] = {{22, 10, 4, 0, {2, 2, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_CONVOLVE_5X5_16S[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_CONVOLVE_5X5_16S[] = {{26, 10, 4, 0, {2, 2, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_CONVOLVE_5X5_16S[] = {{28, 10, 4, 0, {2, 2, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_CONVOLVE_5X5_16S[] = {{30, 10, 4, 0, {2, 2, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_CONVOLVE_5X5_16S[] = {{32, 10, 4, 0, {2, 2, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_CONVOLVE_5X5_16S[] = {{36, 10, 4, 0, {2, 2, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_CONVOLVE_5X5_16S[] = {{40, 10, 4, 0, {2, 2, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_CONVOLVE_5X5_16S[] = {{2, 12, 4, 0, {2, 2, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_CONVOLVE_5X5_16S[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_CONVOLVE_5X5_16S[] = {{6, 12, 4, 0, {2, 2, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_CONVOLVE_5X5_16S[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_CONVOLVE_5X5_16S[] = {{10, 12, 4, 0, {2, 2, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_CONVOLVE_5X5_16S[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_CONVOLVE_5X5_16S[] = {{14, 12, 4, 0, {2, 2, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_CONVOLVE_5X5_16S[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_CONVOLVE_5X5_16S[] = {{18, 12, 4, 0, {2, 2, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_CONVOLVE_5X5_16S[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_CONVOLVE_5X5_16S[] = {{22, 12, 4, 0, {2, 2, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_CONVOLVE_5X5_16S[] = {{24, 12, 4, 0, {2, 2, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_CONVOLVE_5X5_16S[] = {{26, 12, 4, 0, {2, 2, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_CONVOLVE_5X5_16S[] = {{28, 12, 4, 0, {2, 2, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_CONVOLVE_5X5_16S[] = {{30, 12, 4, 0, {2, 2, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_CONVOLVE_5X5_16S[] = {{32, 12, 4, 0, {2, 2, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_CONVOLVE_5X5_16S[] = {{36, 12, 4, 0, {2, 2, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_CONVOLVE_5X5_16S[] = {{2, 14, 4, 0, {2, 2, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_CONVOLVE_5X5_16S[] = {{4, 14, 4, 0, {2, 2, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_CONVOLVE_5X5_16S[] = {{6, 14, 4, 0, {2, 2, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_CONVOLVE_5X5_16S[] = {{8, 14, 4, 0, {2, 2, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_CONVOLVE_5X5_16S[] = {{10, 14, 4, 0, {2, 2, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_CONVOLVE_5X5_16S[] = {{12, 14, 4, 0, {2, 2, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_CONVOLVE_5X5_16S[] = {{14, 14, 4, 0, {2, 2, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_CONVOLVE_5X5_16S[] = {{16, 14, 4, 0, {2, 2, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_CONVOLVE_5X5_16S[] = {{18, 14, 4, 0, {2, 2, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_CONVOLVE_5X5_16S[] = {{20, 14, 4, 0, {2, 2, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_CONVOLVE_5X5_16S[] = {{22, 14, 4, 0, {2, 2, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_CONVOLVE_5X5_16S[] = {{24, 14, 4, 0, {2, 2, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_CONVOLVE_5X5_16S[] = {{26, 14, 4, 0, {2, 2, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_CONVOLVE_5X5_16S[] = {{28, 14, 4, 0, {2, 2, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_CONVOLVE_5X5_16S[] = {{30, 14, 4, 0, {2, 2, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_CONVOLVE_5X5_16S[] = {{2, 16, 4, 0, {2, 2, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_CONVOLVE_5X5_16S[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_CONVOLVE_5X5_16S[] = {{6, 16, 4, 0, {2, 2, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_CONVOLVE_5X5_16S[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_CONVOLVE_5X5_16S[] = {{10, 16, 4, 0, {2, 2, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_CONVOLVE_5X5_16S[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_CONVOLVE_5X5_16S[] = {{14, 16, 4, 0, {2, 2, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_CONVOLVE_5X5_16S[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_CONVOLVE_5X5_16S[] = {{18, 16, 4, 0, {2, 2, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_CONVOLVE_5X5_16S[] = {{20, 16, 4, 0, {2, 2, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_CONVOLVE_5X5_16S[] = {{22, 16, 4, 0, {2, 2, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_CONVOLVE_5X5_16S[] = {{24, 16, 4, 0, {2, 2, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_CONVOLVE_5X5_16S[] = {{26, 16, 4, 0, {2, 2, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_CONVOLVE_5X5_16S[] = {{2, 18, 4, 0, {2, 2, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_CONVOLVE_5X5_16S[] = {{4, 18, 4, 0, {2, 2, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_CONVOLVE_5X5_16S[] = {{6, 18, 4, 0, {2, 2, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_CONVOLVE_5X5_16S[] = {{8, 18, 4, 0, {2, 2, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_CONVOLVE_5X5_16S[] = {{10, 18, 4, 0, {2, 2, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_CONVOLVE_5X5_16S[] = {{12, 18, 4, 0, {2, 2, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_CONVOLVE_5X5_16S[] = {{14, 18, 4, 0, {2, 2, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_CONVOLVE_5X5_16S[] = {{16, 18, 4, 0, {2, 2, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_CONVOLVE_5X5_16S[] = {{18, 18, 4, 0, {2, 2, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_CONVOLVE_5X5_16S[] = {{20, 18, 4, 0, {2, 2, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_CONVOLVE_5X5_16S[] = {{22, 18, 4, 0, {2, 2, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_CONVOLVE_5X5_16S[] = {{2, 20, 4, 0, {2, 2, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_CONVOLVE_5X5_16S[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_CONVOLVE_5X5_16S[] = {{6, 20, 4, 0, {2, 2, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_CONVOLVE_5X5_16S[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_CONVOLVE_5X5_16S[] = {{10, 20, 4, 0, {2, 2, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_CONVOLVE_5X5_16S[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_CONVOLVE_5X5_16S[] = {{14, 20, 4, 0, {2, 2, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_CONVOLVE_5X5_16S[] = {{16, 20, 4, 0, {2, 2, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_CONVOLVE_5X5_16S[] = {{18, 20, 4, 0, {2, 2, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_CONVOLVE_5X5_16S[] = {{20, 20, 4, 0, {2, 2, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_CONVOLVE_5X5_16S[] = {{2, 22, 4, 0, {2, 2, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_CONVOLVE_5X5_16S[] = {{4, 22, 4, 0, {2, 2, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_CONVOLVE_5X5_16S[] = {{6, 22, 4, 0, {2, 2, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_CONVOLVE_5X5_16S[] = {{8, 22, 4, 0, {2, 2, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_CONVOLVE_5X5_16S[] = {{10, 22, 4, 0, {2, 2, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_CONVOLVE_5X5_16S[] = {{12, 22, 4, 0, {2, 2, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_CONVOLVE_5X5_16S[] = {{14, 22, 4, 0, {2, 2, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_CONVOLVE_5X5_16S[] = {{16, 22, 4, 0, {2, 2, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_CONVOLVE_5X5_16S[] = {{18, 22, 4, 0, {2, 2, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_CONVOLVE_5X5_16S[] = {{2, 24, 4, 0, {2, 2, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_CONVOLVE_5X5_16S[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_CONVOLVE_5X5_16S[] = {{6, 24, 4, 0, {2, 2, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_CONVOLVE_5X5_16S[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_CONVOLVE_5X5_16S[] = {{10, 24, 4, 0, {2, 2, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_CONVOLVE_5X5_16S[] = {{12, 24, 4, 0, {2, 2, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_CONVOLVE_5X5_16S[] = {{14, 24, 4, 0, {2, 2, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_CONVOLVE_5X5_16S[] = {{16, 24, 4, 0, {2, 2, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_CONVOLVE_5X5_16S[] = {{2, 26, 4, 0, {2, 2, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_CONVOLVE_5X5_16S[] = {{4, 26, 4, 0, {2, 2, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_CONVOLVE_5X5_16S[] = {{6, 26, 4, 0, {2, 2, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_CONVOLVE_5X5_16S[] = {{8, 26, 4, 0, {2, 2, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_CONVOLVE_5X5_16S[] = {{10, 26, 4, 0, {2, 2, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_CONVOLVE_5X5_16S[] = {{12, 26, 4, 0, {2, 2, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_CONVOLVE_5X5_16S[] = {{14, 26, 4, 0, {2, 2, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_CONVOLVE_5X5_16S[] = {{16, 26, 4, 0, {2, 2, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_CONVOLVE_5X5_16S[] = {{2, 28, 4, 0, {2, 2, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_CONVOLVE_5X5_16S[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_CONVOLVE_5X5_16S[] = {{6, 28, 4, 0, {2, 2, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_CONVOLVE_5X5_16S[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_CONVOLVE_5X5_16S[] = {{10, 28, 4, 0, {2, 2, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_CONVOLVE_5X5_16S[] = {{12, 28, 4, 0, {2, 2, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_CONVOLVE_5X5_16S[] = {{14, 28, 4, 0, {2, 2, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_CONVOLVE_5X5_16S[] = {{2, 30, 4, 0, {2, 2, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_CONVOLVE_5X5_16S[] = {{4, 30, 4, 0, {2, 2, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_CONVOLVE_5X5_16S[] = {{6, 30, 4, 0, {2, 2, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_CONVOLVE_5X5_16S[] = {{8, 30, 4, 0, {2, 2, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_CONVOLVE_5X5_16S[] = {{10, 30, 4, 0, {2, 2, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_CONVOLVE_5X5_16S[] = {{12, 30, 4, 0, {2, 2, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_CONVOLVE_5X5_16S[] = {{2, 32, 4, 0, {2, 2, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_CONVOLVE_5X5_16S[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_CONVOLVE_5X5_16S[] = {{6, 32, 4, 0, {2, 2, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_CONVOLVE_5X5_16S[] = {{8, 32, 4, 0, {2, 2, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_CONVOLVE_5X5_16S[] = {{10, 32, 4, 0, {2, 2, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_CONVOLVE_5X5_16S[] = {{12, 32, 4, 0, {2, 2, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {25, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CONVOLVE_5X5_16S[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_CONVOLVE_5X5_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_CONVOLVE_5X5_16S[] = {{1, 0}};

static acf_scenario gScenarioArray_CONVOLVE_5X5_16S[] = {
{2, 1, 76, 32, 2, gScenarioBufferData0_CONVOLVE_5X5_16S, 48, gScenarioKernelData0_CONVOLVE_5X5_16S, 4},
{4, 1, 104, 32, 2, gScenarioBufferData1_CONVOLVE_5X5_16S, 48, gScenarioKernelData1_CONVOLVE_5X5_16S, 4},
{6, 1, 136, 32, 2, gScenarioBufferData2_CONVOLVE_5X5_16S, 48, gScenarioKernelData2_CONVOLVE_5X5_16S, 4},
{8, 1, 164, 32, 2, gScenarioBufferData3_CONVOLVE_5X5_16S, 48, gScenarioKernelData3_CONVOLVE_5X5_16S, 4},
{10, 1, 196, 32, 2, gScenarioBufferData4_CONVOLVE_5X5_16S, 48, gScenarioKernelData4_CONVOLVE_5X5_16S, 4},
{12, 1, 224, 32, 2, gScenarioBufferData5_CONVOLVE_5X5_16S, 48, gScenarioKernelData5_CONVOLVE_5X5_16S, 4},
{14, 1, 256, 32, 2, gScenarioBufferData6_CONVOLVE_5X5_16S, 48, gScenarioKernelData6_CONVOLVE_5X5_16S, 4},
{16, 1, 284, 32, 2, gScenarioBufferData7_CONVOLVE_5X5_16S, 48, gScenarioKernelData7_CONVOLVE_5X5_16S, 4},
{18, 1, 316, 32, 2, gScenarioBufferData8_CONVOLVE_5X5_16S, 48, gScenarioKernelData8_CONVOLVE_5X5_16S, 4},
{20, 1, 344, 32, 2, gScenarioBufferData9_CONVOLVE_5X5_16S, 48, gScenarioKernelData9_CONVOLVE_5X5_16S, 4},
{22, 1, 376, 32, 2, gScenarioBufferData10_CONVOLVE_5X5_16S, 48, gScenarioKernelData10_CONVOLVE_5X5_16S, 4},
{24, 1, 404, 32, 2, gScenarioBufferData11_CONVOLVE_5X5_16S, 48, gScenarioKernelData11_CONVOLVE_5X5_16S, 4},
{26, 1, 436, 32, 2, gScenarioBufferData12_CONVOLVE_5X5_16S, 48, gScenarioKernelData12_CONVOLVE_5X5_16S, 4},
{28, 1, 464, 32, 2, gScenarioBufferData13_CONVOLVE_5X5_16S, 48, gScenarioKernelData13_CONVOLVE_5X5_16S, 4},
{30, 1, 496, 32, 2, gScenarioBufferData14_CONVOLVE_5X5_16S, 48, gScenarioKernelData14_CONVOLVE_5X5_16S, 4},
{32, 1, 524, 32, 2, gScenarioBufferData15_CONVOLVE_5X5_16S, 48, gScenarioKernelData15_CONVOLVE_5X5_16S, 4},
{36, 1, 584, 32, 2, gScenarioBufferData16_CONVOLVE_5X5_16S, 48, gScenarioKernelData16_CONVOLVE_5X5_16S, 4},
{40, 1, 644, 32, 2, gScenarioBufferData17_CONVOLVE_5X5_16S, 48, gScenarioKernelData17_CONVOLVE_5X5_16S, 4},
{44, 1, 704, 32, 2, gScenarioBufferData18_CONVOLVE_5X5_16S, 48, gScenarioKernelData18_CONVOLVE_5X5_16S, 4},
{48, 1, 764, 32, 2, gScenarioBufferData19_CONVOLVE_5X5_16S, 48, gScenarioKernelData19_CONVOLVE_5X5_16S, 4},
{52, 1, 824, 32, 2, gScenarioBufferData20_CONVOLVE_5X5_16S, 48, gScenarioKernelData20_CONVOLVE_5X5_16S, 4},
{56, 1, 884, 32, 2, gScenarioBufferData21_CONVOLVE_5X5_16S, 48, gScenarioKernelData21_CONVOLVE_5X5_16S, 4},
{60, 1, 944, 32, 2, gScenarioBufferData22_CONVOLVE_5X5_16S, 48, gScenarioKernelData22_CONVOLVE_5X5_16S, 4},
{64, 1, 1004, 32, 2, gScenarioBufferData23_CONVOLVE_5X5_16S, 48, gScenarioKernelData23_CONVOLVE_5X5_16S, 4},
{2, 2, 88, 32, 1, gScenarioBufferData24_CONVOLVE_5X5_16S, 48, gScenarioKernelData24_CONVOLVE_5X5_16S, 4},
{4, 2, 128, 32, 1, gScenarioBufferData25_CONVOLVE_5X5_16S, 48, gScenarioKernelData25_CONVOLVE_5X5_16S, 4},
{6, 2, 168, 32, 1, gScenarioBufferData26_CONVOLVE_5X5_16S, 48, gScenarioKernelData26_CONVOLVE_5X5_16S, 4},
{8, 2, 208, 32, 1, gScenarioBufferData27_CONVOLVE_5X5_16S, 48, gScenarioKernelData27_CONVOLVE_5X5_16S, 4},
{10, 2, 248, 32, 1, gScenarioBufferData28_CONVOLVE_5X5_16S, 48, gScenarioKernelData28_CONVOLVE_5X5_16S, 4},
{12, 2, 288, 32, 1, gScenarioBufferData29_CONVOLVE_5X5_16S, 48, gScenarioKernelData29_CONVOLVE_5X5_16S, 4},
{14, 2, 328, 32, 1, gScenarioBufferData30_CONVOLVE_5X5_16S, 48, gScenarioKernelData30_CONVOLVE_5X5_16S, 4},
{16, 2, 368, 32, 1, gScenarioBufferData31_CONVOLVE_5X5_16S, 48, gScenarioKernelData31_CONVOLVE_5X5_16S, 4},
{18, 2, 408, 32, 1, gScenarioBufferData32_CONVOLVE_5X5_16S, 48, gScenarioKernelData32_CONVOLVE_5X5_16S, 4},
{20, 2, 448, 32, 1, gScenarioBufferData33_CONVOLVE_5X5_16S, 48, gScenarioKernelData33_CONVOLVE_5X5_16S, 4},
{22, 2, 488, 32, 1, gScenarioBufferData34_CONVOLVE_5X5_16S, 48, gScenarioKernelData34_CONVOLVE_5X5_16S, 4},
{24, 2, 528, 32, 1, gScenarioBufferData35_CONVOLVE_5X5_16S, 48, gScenarioKernelData35_CONVOLVE_5X5_16S, 4},
{26, 2, 568, 32, 1, gScenarioBufferData36_CONVOLVE_5X5_16S, 48, gScenarioKernelData36_CONVOLVE_5X5_16S, 4},
{28, 2, 608, 32, 1, gScenarioBufferData37_CONVOLVE_5X5_16S, 48, gScenarioKernelData37_CONVOLVE_5X5_16S, 4},
{30, 2, 648, 32, 1, gScenarioBufferData38_CONVOLVE_5X5_16S, 48, gScenarioKernelData38_CONVOLVE_5X5_16S, 4},
{32, 2, 688, 32, 1, gScenarioBufferData39_CONVOLVE_5X5_16S, 48, gScenarioKernelData39_CONVOLVE_5X5_16S, 4},
{36, 2, 768, 32, 1, gScenarioBufferData40_CONVOLVE_5X5_16S, 48, gScenarioKernelData40_CONVOLVE_5X5_16S, 4},
{40, 2, 848, 32, 1, gScenarioBufferData41_CONVOLVE_5X5_16S, 48, gScenarioKernelData41_CONVOLVE_5X5_16S, 4},
{44, 2, 928, 32, 1, gScenarioBufferData42_CONVOLVE_5X5_16S, 48, gScenarioKernelData42_CONVOLVE_5X5_16S, 4},
{48, 2, 1008, 32, 1, gScenarioBufferData43_CONVOLVE_5X5_16S, 48, gScenarioKernelData43_CONVOLVE_5X5_16S, 4},
{52, 2, 1088, 32, 1, gScenarioBufferData44_CONVOLVE_5X5_16S, 48, gScenarioKernelData44_CONVOLVE_5X5_16S, 4},
{56, 2, 1168, 32, 1, gScenarioBufferData45_CONVOLVE_5X5_16S, 48, gScenarioKernelData45_CONVOLVE_5X5_16S, 4},
{60, 2, 1248, 32, 1, gScenarioBufferData46_CONVOLVE_5X5_16S, 48, gScenarioKernelData46_CONVOLVE_5X5_16S, 4},
{64, 2, 1328, 32, 1, gScenarioBufferData47_CONVOLVE_5X5_16S, 48, gScenarioKernelData47_CONVOLVE_5X5_16S, 4},
{2, 3, 120, 32, 1, gScenarioBufferData48_CONVOLVE_5X5_16S, 48, gScenarioKernelData48_CONVOLVE_5X5_16S, 4},
{4, 3, 176, 32, 1, gScenarioBufferData49_CONVOLVE_5X5_16S, 48, gScenarioKernelData49_CONVOLVE_5X5_16S, 4},
{6, 3, 232, 32, 1, gScenarioBufferData50_CONVOLVE_5X5_16S, 48, gScenarioKernelData50_CONVOLVE_5X5_16S, 4},
{8, 3, 288, 32, 1, gScenarioBufferData51_CONVOLVE_5X5_16S, 48, gScenarioKernelData51_CONVOLVE_5X5_16S, 4},
{10, 3, 344, 32, 1, gScenarioBufferData52_CONVOLVE_5X5_16S, 48, gScenarioKernelData52_CONVOLVE_5X5_16S, 4},
{12, 3, 400, 32, 1, gScenarioBufferData53_CONVOLVE_5X5_16S, 48, gScenarioKernelData53_CONVOLVE_5X5_16S, 4},
{14, 3, 456, 32, 1, gScenarioBufferData54_CONVOLVE_5X5_16S, 48, gScenarioKernelData54_CONVOLVE_5X5_16S, 4},
{16, 3, 512, 32, 1, gScenarioBufferData55_CONVOLVE_5X5_16S, 48, gScenarioKernelData55_CONVOLVE_5X5_16S, 4},
{18, 3, 568, 32, 1, gScenarioBufferData56_CONVOLVE_5X5_16S, 48, gScenarioKernelData56_CONVOLVE_5X5_16S, 4},
{20, 3, 624, 32, 1, gScenarioBufferData57_CONVOLVE_5X5_16S, 48, gScenarioKernelData57_CONVOLVE_5X5_16S, 4},
{22, 3, 680, 32, 1, gScenarioBufferData58_CONVOLVE_5X5_16S, 48, gScenarioKernelData58_CONVOLVE_5X5_16S, 4},
{24, 3, 736, 32, 1, gScenarioBufferData59_CONVOLVE_5X5_16S, 48, gScenarioKernelData59_CONVOLVE_5X5_16S, 4},
{26, 3, 792, 32, 1, gScenarioBufferData60_CONVOLVE_5X5_16S, 48, gScenarioKernelData60_CONVOLVE_5X5_16S, 4},
{28, 3, 848, 32, 1, gScenarioBufferData61_CONVOLVE_5X5_16S, 48, gScenarioKernelData61_CONVOLVE_5X5_16S, 4},
{30, 3, 904, 32, 1, gScenarioBufferData62_CONVOLVE_5X5_16S, 48, gScenarioKernelData62_CONVOLVE_5X5_16S, 4},
{32, 3, 960, 32, 1, gScenarioBufferData63_CONVOLVE_5X5_16S, 48, gScenarioKernelData63_CONVOLVE_5X5_16S, 4},
{36, 3, 1072, 32, 1, gScenarioBufferData64_CONVOLVE_5X5_16S, 48, gScenarioKernelData64_CONVOLVE_5X5_16S, 4},
{40, 3, 1184, 32, 1, gScenarioBufferData65_CONVOLVE_5X5_16S, 48, gScenarioKernelData65_CONVOLVE_5X5_16S, 4},
{44, 3, 1296, 32, 1, gScenarioBufferData66_CONVOLVE_5X5_16S, 48, gScenarioKernelData66_CONVOLVE_5X5_16S, 4},
{48, 3, 1408, 32, 1, gScenarioBufferData67_CONVOLVE_5X5_16S, 48, gScenarioKernelData67_CONVOLVE_5X5_16S, 4},
{52, 3, 1520, 32, 1, gScenarioBufferData68_CONVOLVE_5X5_16S, 48, gScenarioKernelData68_CONVOLVE_5X5_16S, 4},
{56, 3, 1632, 32, 1, gScenarioBufferData69_CONVOLVE_5X5_16S, 48, gScenarioKernelData69_CONVOLVE_5X5_16S, 4},
{60, 3, 1744, 32, 1, gScenarioBufferData70_CONVOLVE_5X5_16S, 48, gScenarioKernelData70_CONVOLVE_5X5_16S, 4},
{64, 3, 1856, 32, 1, gScenarioBufferData71_CONVOLVE_5X5_16S, 48, gScenarioKernelData71_CONVOLVE_5X5_16S, 4},
{2, 4, 152, 32, 1, gScenarioBufferData72_CONVOLVE_5X5_16S, 48, gScenarioKernelData72_CONVOLVE_5X5_16S, 4},
{4, 4, 224, 32, 1, gScenarioBufferData73_CONVOLVE_5X5_16S, 48, gScenarioKernelData73_CONVOLVE_5X5_16S, 4},
{6, 4, 296, 32, 1, gScenarioBufferData74_CONVOLVE_5X5_16S, 48, gScenarioKernelData74_CONVOLVE_5X5_16S, 4},
{8, 4, 368, 32, 1, gScenarioBufferData75_CONVOLVE_5X5_16S, 48, gScenarioKernelData75_CONVOLVE_5X5_16S, 4},
{10, 4, 440, 32, 1, gScenarioBufferData76_CONVOLVE_5X5_16S, 48, gScenarioKernelData76_CONVOLVE_5X5_16S, 4},
{12, 4, 512, 32, 1, gScenarioBufferData77_CONVOLVE_5X5_16S, 48, gScenarioKernelData77_CONVOLVE_5X5_16S, 4},
{14, 4, 584, 32, 1, gScenarioBufferData78_CONVOLVE_5X5_16S, 48, gScenarioKernelData78_CONVOLVE_5X5_16S, 4},
{16, 4, 656, 32, 1, gScenarioBufferData79_CONVOLVE_5X5_16S, 48, gScenarioKernelData79_CONVOLVE_5X5_16S, 4},
{18, 4, 728, 32, 1, gScenarioBufferData80_CONVOLVE_5X5_16S, 48, gScenarioKernelData80_CONVOLVE_5X5_16S, 4},
{20, 4, 800, 32, 1, gScenarioBufferData81_CONVOLVE_5X5_16S, 48, gScenarioKernelData81_CONVOLVE_5X5_16S, 4},
{22, 4, 872, 32, 1, gScenarioBufferData82_CONVOLVE_5X5_16S, 48, gScenarioKernelData82_CONVOLVE_5X5_16S, 4},
{24, 4, 944, 32, 1, gScenarioBufferData83_CONVOLVE_5X5_16S, 48, gScenarioKernelData83_CONVOLVE_5X5_16S, 4},
{26, 4, 1016, 32, 1, gScenarioBufferData84_CONVOLVE_5X5_16S, 48, gScenarioKernelData84_CONVOLVE_5X5_16S, 4},
{28, 4, 1088, 32, 1, gScenarioBufferData85_CONVOLVE_5X5_16S, 48, gScenarioKernelData85_CONVOLVE_5X5_16S, 4},
{30, 4, 1160, 32, 1, gScenarioBufferData86_CONVOLVE_5X5_16S, 48, gScenarioKernelData86_CONVOLVE_5X5_16S, 4},
{32, 4, 1232, 32, 1, gScenarioBufferData87_CONVOLVE_5X5_16S, 48, gScenarioKernelData87_CONVOLVE_5X5_16S, 4},
{36, 4, 1376, 32, 1, gScenarioBufferData88_CONVOLVE_5X5_16S, 48, gScenarioKernelData88_CONVOLVE_5X5_16S, 4},
{40, 4, 1520, 32, 1, gScenarioBufferData89_CONVOLVE_5X5_16S, 48, gScenarioKernelData89_CONVOLVE_5X5_16S, 4},
{44, 4, 1664, 32, 1, gScenarioBufferData90_CONVOLVE_5X5_16S, 48, gScenarioKernelData90_CONVOLVE_5X5_16S, 4},
{48, 4, 1808, 32, 1, gScenarioBufferData91_CONVOLVE_5X5_16S, 48, gScenarioKernelData91_CONVOLVE_5X5_16S, 4},
{52, 4, 1952, 32, 1, gScenarioBufferData92_CONVOLVE_5X5_16S, 48, gScenarioKernelData92_CONVOLVE_5X5_16S, 4},
{56, 4, 2096, 32, 1, gScenarioBufferData93_CONVOLVE_5X5_16S, 48, gScenarioKernelData93_CONVOLVE_5X5_16S, 4},
{60, 4, 2240, 32, 1, gScenarioBufferData94_CONVOLVE_5X5_16S, 48, gScenarioKernelData94_CONVOLVE_5X5_16S, 4},
{64, 4, 2384, 32, 1, gScenarioBufferData95_CONVOLVE_5X5_16S, 48, gScenarioKernelData95_CONVOLVE_5X5_16S, 4},
{2, 5, 184, 32, 1, gScenarioBufferData96_CONVOLVE_5X5_16S, 48, gScenarioKernelData96_CONVOLVE_5X5_16S, 4},
{4, 5, 272, 32, 1, gScenarioBufferData97_CONVOLVE_5X5_16S, 48, gScenarioKernelData97_CONVOLVE_5X5_16S, 4},
{6, 5, 360, 32, 1, gScenarioBufferData98_CONVOLVE_5X5_16S, 48, gScenarioKernelData98_CONVOLVE_5X5_16S, 4},
{8, 5, 448, 32, 1, gScenarioBufferData99_CONVOLVE_5X5_16S, 48, gScenarioKernelData99_CONVOLVE_5X5_16S, 4},
{10, 5, 536, 32, 1, gScenarioBufferData100_CONVOLVE_5X5_16S, 48, gScenarioKernelData100_CONVOLVE_5X5_16S, 4},
{12, 5, 624, 32, 1, gScenarioBufferData101_CONVOLVE_5X5_16S, 48, gScenarioKernelData101_CONVOLVE_5X5_16S, 4},
{14, 5, 712, 32, 1, gScenarioBufferData102_CONVOLVE_5X5_16S, 48, gScenarioKernelData102_CONVOLVE_5X5_16S, 4},
{16, 5, 800, 32, 1, gScenarioBufferData103_CONVOLVE_5X5_16S, 48, gScenarioKernelData103_CONVOLVE_5X5_16S, 4},
{18, 5, 888, 32, 1, gScenarioBufferData104_CONVOLVE_5X5_16S, 48, gScenarioKernelData104_CONVOLVE_5X5_16S, 4},
{20, 5, 976, 32, 1, gScenarioBufferData105_CONVOLVE_5X5_16S, 48, gScenarioKernelData105_CONVOLVE_5X5_16S, 4},
{22, 5, 1064, 32, 1, gScenarioBufferData106_CONVOLVE_5X5_16S, 48, gScenarioKernelData106_CONVOLVE_5X5_16S, 4},
{24, 5, 1152, 32, 1, gScenarioBufferData107_CONVOLVE_5X5_16S, 48, gScenarioKernelData107_CONVOLVE_5X5_16S, 4},
{26, 5, 1240, 32, 1, gScenarioBufferData108_CONVOLVE_5X5_16S, 48, gScenarioKernelData108_CONVOLVE_5X5_16S, 4},
{28, 5, 1328, 32, 1, gScenarioBufferData109_CONVOLVE_5X5_16S, 48, gScenarioKernelData109_CONVOLVE_5X5_16S, 4},
{30, 5, 1416, 32, 1, gScenarioBufferData110_CONVOLVE_5X5_16S, 48, gScenarioKernelData110_CONVOLVE_5X5_16S, 4},
{32, 5, 1504, 32, 1, gScenarioBufferData111_CONVOLVE_5X5_16S, 48, gScenarioKernelData111_CONVOLVE_5X5_16S, 4},
{36, 5, 1680, 32, 1, gScenarioBufferData112_CONVOLVE_5X5_16S, 48, gScenarioKernelData112_CONVOLVE_5X5_16S, 4},
{40, 5, 1856, 32, 1, gScenarioBufferData113_CONVOLVE_5X5_16S, 48, gScenarioKernelData113_CONVOLVE_5X5_16S, 4},
{44, 5, 2032, 32, 1, gScenarioBufferData114_CONVOLVE_5X5_16S, 48, gScenarioKernelData114_CONVOLVE_5X5_16S, 4},
{48, 5, 2208, 32, 1, gScenarioBufferData115_CONVOLVE_5X5_16S, 48, gScenarioKernelData115_CONVOLVE_5X5_16S, 4},
{52, 5, 2384, 32, 1, gScenarioBufferData116_CONVOLVE_5X5_16S, 48, gScenarioKernelData116_CONVOLVE_5X5_16S, 4},
{56, 5, 2560, 32, 1, gScenarioBufferData117_CONVOLVE_5X5_16S, 48, gScenarioKernelData117_CONVOLVE_5X5_16S, 4},
{60, 5, 2736, 32, 1, gScenarioBufferData118_CONVOLVE_5X5_16S, 48, gScenarioKernelData118_CONVOLVE_5X5_16S, 4},
{64, 5, 2912, 32, 1, gScenarioBufferData119_CONVOLVE_5X5_16S, 48, gScenarioKernelData119_CONVOLVE_5X5_16S, 4},
{2, 6, 216, 32, 1, gScenarioBufferData120_CONVOLVE_5X5_16S, 48, gScenarioKernelData120_CONVOLVE_5X5_16S, 4},
{4, 6, 320, 32, 1, gScenarioBufferData121_CONVOLVE_5X5_16S, 48, gScenarioKernelData121_CONVOLVE_5X5_16S, 4},
{6, 6, 424, 32, 1, gScenarioBufferData122_CONVOLVE_5X5_16S, 48, gScenarioKernelData122_CONVOLVE_5X5_16S, 4},
{8, 6, 528, 32, 1, gScenarioBufferData123_CONVOLVE_5X5_16S, 48, gScenarioKernelData123_CONVOLVE_5X5_16S, 4},
{10, 6, 632, 32, 1, gScenarioBufferData124_CONVOLVE_5X5_16S, 48, gScenarioKernelData124_CONVOLVE_5X5_16S, 4},
{12, 6, 736, 32, 1, gScenarioBufferData125_CONVOLVE_5X5_16S, 48, gScenarioKernelData125_CONVOLVE_5X5_16S, 4},
{14, 6, 840, 32, 1, gScenarioBufferData126_CONVOLVE_5X5_16S, 48, gScenarioKernelData126_CONVOLVE_5X5_16S, 4},
{16, 6, 944, 32, 1, gScenarioBufferData127_CONVOLVE_5X5_16S, 48, gScenarioKernelData127_CONVOLVE_5X5_16S, 4},
{18, 6, 1048, 32, 1, gScenarioBufferData128_CONVOLVE_5X5_16S, 48, gScenarioKernelData128_CONVOLVE_5X5_16S, 4},
{20, 6, 1152, 32, 1, gScenarioBufferData129_CONVOLVE_5X5_16S, 48, gScenarioKernelData129_CONVOLVE_5X5_16S, 4},
{22, 6, 1256, 32, 1, gScenarioBufferData130_CONVOLVE_5X5_16S, 48, gScenarioKernelData130_CONVOLVE_5X5_16S, 4},
{24, 6, 1360, 32, 1, gScenarioBufferData131_CONVOLVE_5X5_16S, 48, gScenarioKernelData131_CONVOLVE_5X5_16S, 4},
{26, 6, 1464, 32, 1, gScenarioBufferData132_CONVOLVE_5X5_16S, 48, gScenarioKernelData132_CONVOLVE_5X5_16S, 4},
{28, 6, 1568, 32, 1, gScenarioBufferData133_CONVOLVE_5X5_16S, 48, gScenarioKernelData133_CONVOLVE_5X5_16S, 4},
{30, 6, 1672, 32, 1, gScenarioBufferData134_CONVOLVE_5X5_16S, 48, gScenarioKernelData134_CONVOLVE_5X5_16S, 4},
{32, 6, 1776, 32, 1, gScenarioBufferData135_CONVOLVE_5X5_16S, 48, gScenarioKernelData135_CONVOLVE_5X5_16S, 4},
{36, 6, 1984, 32, 1, gScenarioBufferData136_CONVOLVE_5X5_16S, 48, gScenarioKernelData136_CONVOLVE_5X5_16S, 4},
{40, 6, 2192, 32, 1, gScenarioBufferData137_CONVOLVE_5X5_16S, 48, gScenarioKernelData137_CONVOLVE_5X5_16S, 4},
{44, 6, 2400, 32, 1, gScenarioBufferData138_CONVOLVE_5X5_16S, 48, gScenarioKernelData138_CONVOLVE_5X5_16S, 4},
{48, 6, 2608, 32, 1, gScenarioBufferData139_CONVOLVE_5X5_16S, 48, gScenarioKernelData139_CONVOLVE_5X5_16S, 4},
{52, 6, 2816, 32, 1, gScenarioBufferData140_CONVOLVE_5X5_16S, 48, gScenarioKernelData140_CONVOLVE_5X5_16S, 4},
{56, 6, 3024, 32, 1, gScenarioBufferData141_CONVOLVE_5X5_16S, 48, gScenarioKernelData141_CONVOLVE_5X5_16S, 4},
{60, 6, 3232, 32, 1, gScenarioBufferData142_CONVOLVE_5X5_16S, 48, gScenarioKernelData142_CONVOLVE_5X5_16S, 4},
{64, 6, 3440, 32, 1, gScenarioBufferData143_CONVOLVE_5X5_16S, 48, gScenarioKernelData143_CONVOLVE_5X5_16S, 4},
{2, 8, 280, 32, 1, gScenarioBufferData144_CONVOLVE_5X5_16S, 48, gScenarioKernelData144_CONVOLVE_5X5_16S, 4},
{4, 8, 416, 32, 1, gScenarioBufferData145_CONVOLVE_5X5_16S, 48, gScenarioKernelData145_CONVOLVE_5X5_16S, 4},
{6, 8, 552, 32, 1, gScenarioBufferData146_CONVOLVE_5X5_16S, 48, gScenarioKernelData146_CONVOLVE_5X5_16S, 4},
{8, 8, 688, 32, 1, gScenarioBufferData147_CONVOLVE_5X5_16S, 48, gScenarioKernelData147_CONVOLVE_5X5_16S, 4},
{10, 8, 824, 32, 1, gScenarioBufferData148_CONVOLVE_5X5_16S, 48, gScenarioKernelData148_CONVOLVE_5X5_16S, 4},
{12, 8, 960, 32, 1, gScenarioBufferData149_CONVOLVE_5X5_16S, 48, gScenarioKernelData149_CONVOLVE_5X5_16S, 4},
{14, 8, 1096, 32, 1, gScenarioBufferData150_CONVOLVE_5X5_16S, 48, gScenarioKernelData150_CONVOLVE_5X5_16S, 4},
{16, 8, 1232, 32, 1, gScenarioBufferData151_CONVOLVE_5X5_16S, 48, gScenarioKernelData151_CONVOLVE_5X5_16S, 4},
{18, 8, 1368, 32, 1, gScenarioBufferData152_CONVOLVE_5X5_16S, 48, gScenarioKernelData152_CONVOLVE_5X5_16S, 4},
{20, 8, 1504, 32, 1, gScenarioBufferData153_CONVOLVE_5X5_16S, 48, gScenarioKernelData153_CONVOLVE_5X5_16S, 4},
{22, 8, 1640, 32, 1, gScenarioBufferData154_CONVOLVE_5X5_16S, 48, gScenarioKernelData154_CONVOLVE_5X5_16S, 4},
{24, 8, 1776, 32, 1, gScenarioBufferData155_CONVOLVE_5X5_16S, 48, gScenarioKernelData155_CONVOLVE_5X5_16S, 4},
{26, 8, 1912, 32, 1, gScenarioBufferData156_CONVOLVE_5X5_16S, 48, gScenarioKernelData156_CONVOLVE_5X5_16S, 4},
{28, 8, 2048, 32, 1, gScenarioBufferData157_CONVOLVE_5X5_16S, 48, gScenarioKernelData157_CONVOLVE_5X5_16S, 4},
{30, 8, 2184, 32, 1, gScenarioBufferData158_CONVOLVE_5X5_16S, 48, gScenarioKernelData158_CONVOLVE_5X5_16S, 4},
{32, 8, 2320, 32, 1, gScenarioBufferData159_CONVOLVE_5X5_16S, 48, gScenarioKernelData159_CONVOLVE_5X5_16S, 4},
{36, 8, 2592, 32, 1, gScenarioBufferData160_CONVOLVE_5X5_16S, 48, gScenarioKernelData160_CONVOLVE_5X5_16S, 4},
{40, 8, 2864, 32, 1, gScenarioBufferData161_CONVOLVE_5X5_16S, 48, gScenarioKernelData161_CONVOLVE_5X5_16S, 4},
{44, 8, 3136, 32, 1, gScenarioBufferData162_CONVOLVE_5X5_16S, 48, gScenarioKernelData162_CONVOLVE_5X5_16S, 4},
{48, 8, 3408, 32, 1, gScenarioBufferData163_CONVOLVE_5X5_16S, 48, gScenarioKernelData163_CONVOLVE_5X5_16S, 4},
{52, 8, 3680, 32, 1, gScenarioBufferData164_CONVOLVE_5X5_16S, 48, gScenarioKernelData164_CONVOLVE_5X5_16S, 4},
{2, 10, 344, 32, 1, gScenarioBufferData165_CONVOLVE_5X5_16S, 48, gScenarioKernelData165_CONVOLVE_5X5_16S, 4},
{4, 10, 512, 32, 1, gScenarioBufferData166_CONVOLVE_5X5_16S, 48, gScenarioKernelData166_CONVOLVE_5X5_16S, 4},
{6, 10, 680, 32, 1, gScenarioBufferData167_CONVOLVE_5X5_16S, 48, gScenarioKernelData167_CONVOLVE_5X5_16S, 4},
{8, 10, 848, 32, 1, gScenarioBufferData168_CONVOLVE_5X5_16S, 48, gScenarioKernelData168_CONVOLVE_5X5_16S, 4},
{10, 10, 1016, 32, 1, gScenarioBufferData169_CONVOLVE_5X5_16S, 48, gScenarioKernelData169_CONVOLVE_5X5_16S, 4},
{12, 10, 1184, 32, 1, gScenarioBufferData170_CONVOLVE_5X5_16S, 48, gScenarioKernelData170_CONVOLVE_5X5_16S, 4},
{14, 10, 1352, 32, 1, gScenarioBufferData171_CONVOLVE_5X5_16S, 48, gScenarioKernelData171_CONVOLVE_5X5_16S, 4},
{16, 10, 1520, 32, 1, gScenarioBufferData172_CONVOLVE_5X5_16S, 48, gScenarioKernelData172_CONVOLVE_5X5_16S, 4},
{18, 10, 1688, 32, 1, gScenarioBufferData173_CONVOLVE_5X5_16S, 48, gScenarioKernelData173_CONVOLVE_5X5_16S, 4},
{20, 10, 1856, 32, 1, gScenarioBufferData174_CONVOLVE_5X5_16S, 48, gScenarioKernelData174_CONVOLVE_5X5_16S, 4},
{22, 10, 2024, 32, 1, gScenarioBufferData175_CONVOLVE_5X5_16S, 48, gScenarioKernelData175_CONVOLVE_5X5_16S, 4},
{24, 10, 2192, 32, 1, gScenarioBufferData176_CONVOLVE_5X5_16S, 48, gScenarioKernelData176_CONVOLVE_5X5_16S, 4},
{26, 10, 2360, 32, 1, gScenarioBufferData177_CONVOLVE_5X5_16S, 48, gScenarioKernelData177_CONVOLVE_5X5_16S, 4},
{28, 10, 2528, 32, 1, gScenarioBufferData178_CONVOLVE_5X5_16S, 48, gScenarioKernelData178_CONVOLVE_5X5_16S, 4},
{30, 10, 2696, 32, 1, gScenarioBufferData179_CONVOLVE_5X5_16S, 48, gScenarioKernelData179_CONVOLVE_5X5_16S, 4},
{32, 10, 2864, 32, 1, gScenarioBufferData180_CONVOLVE_5X5_16S, 48, gScenarioKernelData180_CONVOLVE_5X5_16S, 4},
{36, 10, 3200, 32, 1, gScenarioBufferData181_CONVOLVE_5X5_16S, 48, gScenarioKernelData181_CONVOLVE_5X5_16S, 4},
{40, 10, 3536, 32, 1, gScenarioBufferData182_CONVOLVE_5X5_16S, 48, gScenarioKernelData182_CONVOLVE_5X5_16S, 4},
{2, 12, 408, 32, 1, gScenarioBufferData183_CONVOLVE_5X5_16S, 48, gScenarioKernelData183_CONVOLVE_5X5_16S, 4},
{4, 12, 608, 32, 1, gScenarioBufferData184_CONVOLVE_5X5_16S, 48, gScenarioKernelData184_CONVOLVE_5X5_16S, 4},
{6, 12, 808, 32, 1, gScenarioBufferData185_CONVOLVE_5X5_16S, 48, gScenarioKernelData185_CONVOLVE_5X5_16S, 4},
{8, 12, 1008, 32, 1, gScenarioBufferData186_CONVOLVE_5X5_16S, 48, gScenarioKernelData186_CONVOLVE_5X5_16S, 4},
{10, 12, 1208, 32, 1, gScenarioBufferData187_CONVOLVE_5X5_16S, 48, gScenarioKernelData187_CONVOLVE_5X5_16S, 4},
{12, 12, 1408, 32, 1, gScenarioBufferData188_CONVOLVE_5X5_16S, 48, gScenarioKernelData188_CONVOLVE_5X5_16S, 4},
{14, 12, 1608, 32, 1, gScenarioBufferData189_CONVOLVE_5X5_16S, 48, gScenarioKernelData189_CONVOLVE_5X5_16S, 4},
{16, 12, 1808, 32, 1, gScenarioBufferData190_CONVOLVE_5X5_16S, 48, gScenarioKernelData190_CONVOLVE_5X5_16S, 4},
{18, 12, 2008, 32, 1, gScenarioBufferData191_CONVOLVE_5X5_16S, 48, gScenarioKernelData191_CONVOLVE_5X5_16S, 4},
{20, 12, 2208, 32, 1, gScenarioBufferData192_CONVOLVE_5X5_16S, 48, gScenarioKernelData192_CONVOLVE_5X5_16S, 4},
{22, 12, 2408, 32, 1, gScenarioBufferData193_CONVOLVE_5X5_16S, 48, gScenarioKernelData193_CONVOLVE_5X5_16S, 4},
{24, 12, 2608, 32, 1, gScenarioBufferData194_CONVOLVE_5X5_16S, 48, gScenarioKernelData194_CONVOLVE_5X5_16S, 4},
{26, 12, 2808, 32, 1, gScenarioBufferData195_CONVOLVE_5X5_16S, 48, gScenarioKernelData195_CONVOLVE_5X5_16S, 4},
{28, 12, 3008, 32, 1, gScenarioBufferData196_CONVOLVE_5X5_16S, 48, gScenarioKernelData196_CONVOLVE_5X5_16S, 4},
{30, 12, 3208, 32, 1, gScenarioBufferData197_CONVOLVE_5X5_16S, 48, gScenarioKernelData197_CONVOLVE_5X5_16S, 4},
{32, 12, 3408, 32, 1, gScenarioBufferData198_CONVOLVE_5X5_16S, 48, gScenarioKernelData198_CONVOLVE_5X5_16S, 4},
{36, 12, 3808, 32, 1, gScenarioBufferData199_CONVOLVE_5X5_16S, 48, gScenarioKernelData199_CONVOLVE_5X5_16S, 4},
{2, 14, 472, 32, 1, gScenarioBufferData200_CONVOLVE_5X5_16S, 48, gScenarioKernelData200_CONVOLVE_5X5_16S, 4},
{4, 14, 704, 32, 1, gScenarioBufferData201_CONVOLVE_5X5_16S, 48, gScenarioKernelData201_CONVOLVE_5X5_16S, 4},
{6, 14, 936, 32, 1, gScenarioBufferData202_CONVOLVE_5X5_16S, 48, gScenarioKernelData202_CONVOLVE_5X5_16S, 4},
{8, 14, 1168, 32, 1, gScenarioBufferData203_CONVOLVE_5X5_16S, 48, gScenarioKernelData203_CONVOLVE_5X5_16S, 4},
{10, 14, 1400, 32, 1, gScenarioBufferData204_CONVOLVE_5X5_16S, 48, gScenarioKernelData204_CONVOLVE_5X5_16S, 4},
{12, 14, 1632, 32, 1, gScenarioBufferData205_CONVOLVE_5X5_16S, 48, gScenarioKernelData205_CONVOLVE_5X5_16S, 4},
{14, 14, 1864, 32, 1, gScenarioBufferData206_CONVOLVE_5X5_16S, 48, gScenarioKernelData206_CONVOLVE_5X5_16S, 4},
{16, 14, 2096, 32, 1, gScenarioBufferData207_CONVOLVE_5X5_16S, 48, gScenarioKernelData207_CONVOLVE_5X5_16S, 4},
{18, 14, 2328, 32, 1, gScenarioBufferData208_CONVOLVE_5X5_16S, 48, gScenarioKernelData208_CONVOLVE_5X5_16S, 4},
{20, 14, 2560, 32, 1, gScenarioBufferData209_CONVOLVE_5X5_16S, 48, gScenarioKernelData209_CONVOLVE_5X5_16S, 4},
{22, 14, 2792, 32, 1, gScenarioBufferData210_CONVOLVE_5X5_16S, 48, gScenarioKernelData210_CONVOLVE_5X5_16S, 4},
{24, 14, 3024, 32, 1, gScenarioBufferData211_CONVOLVE_5X5_16S, 48, gScenarioKernelData211_CONVOLVE_5X5_16S, 4},
{26, 14, 3256, 32, 1, gScenarioBufferData212_CONVOLVE_5X5_16S, 48, gScenarioKernelData212_CONVOLVE_5X5_16S, 4},
{28, 14, 3488, 32, 1, gScenarioBufferData213_CONVOLVE_5X5_16S, 48, gScenarioKernelData213_CONVOLVE_5X5_16S, 4},
{30, 14, 3720, 32, 1, gScenarioBufferData214_CONVOLVE_5X5_16S, 48, gScenarioKernelData214_CONVOLVE_5X5_16S, 4},
{2, 16, 536, 32, 1, gScenarioBufferData215_CONVOLVE_5X5_16S, 48, gScenarioKernelData215_CONVOLVE_5X5_16S, 4},
{4, 16, 800, 32, 1, gScenarioBufferData216_CONVOLVE_5X5_16S, 48, gScenarioKernelData216_CONVOLVE_5X5_16S, 4},
{6, 16, 1064, 32, 1, gScenarioBufferData217_CONVOLVE_5X5_16S, 48, gScenarioKernelData217_CONVOLVE_5X5_16S, 4},
{8, 16, 1328, 32, 1, gScenarioBufferData218_CONVOLVE_5X5_16S, 48, gScenarioKernelData218_CONVOLVE_5X5_16S, 4},
{10, 16, 1592, 32, 1, gScenarioBufferData219_CONVOLVE_5X5_16S, 48, gScenarioKernelData219_CONVOLVE_5X5_16S, 4},
{12, 16, 1856, 32, 1, gScenarioBufferData220_CONVOLVE_5X5_16S, 48, gScenarioKernelData220_CONVOLVE_5X5_16S, 4},
{14, 16, 2120, 32, 1, gScenarioBufferData221_CONVOLVE_5X5_16S, 48, gScenarioKernelData221_CONVOLVE_5X5_16S, 4},
{16, 16, 2384, 32, 1, gScenarioBufferData222_CONVOLVE_5X5_16S, 48, gScenarioKernelData222_CONVOLVE_5X5_16S, 4},
{18, 16, 2648, 32, 1, gScenarioBufferData223_CONVOLVE_5X5_16S, 48, gScenarioKernelData223_CONVOLVE_5X5_16S, 4},
{20, 16, 2912, 32, 1, gScenarioBufferData224_CONVOLVE_5X5_16S, 48, gScenarioKernelData224_CONVOLVE_5X5_16S, 4},
{22, 16, 3176, 32, 1, gScenarioBufferData225_CONVOLVE_5X5_16S, 48, gScenarioKernelData225_CONVOLVE_5X5_16S, 4},
{24, 16, 3440, 32, 1, gScenarioBufferData226_CONVOLVE_5X5_16S, 48, gScenarioKernelData226_CONVOLVE_5X5_16S, 4},
{26, 16, 3704, 32, 1, gScenarioBufferData227_CONVOLVE_5X5_16S, 48, gScenarioKernelData227_CONVOLVE_5X5_16S, 4},
{2, 18, 600, 32, 1, gScenarioBufferData228_CONVOLVE_5X5_16S, 48, gScenarioKernelData228_CONVOLVE_5X5_16S, 4},
{4, 18, 896, 32, 1, gScenarioBufferData229_CONVOLVE_5X5_16S, 48, gScenarioKernelData229_CONVOLVE_5X5_16S, 4},
{6, 18, 1192, 32, 1, gScenarioBufferData230_CONVOLVE_5X5_16S, 48, gScenarioKernelData230_CONVOLVE_5X5_16S, 4},
{8, 18, 1488, 32, 1, gScenarioBufferData231_CONVOLVE_5X5_16S, 48, gScenarioKernelData231_CONVOLVE_5X5_16S, 4},
{10, 18, 1784, 32, 1, gScenarioBufferData232_CONVOLVE_5X5_16S, 48, gScenarioKernelData232_CONVOLVE_5X5_16S, 4},
{12, 18, 2080, 32, 1, gScenarioBufferData233_CONVOLVE_5X5_16S, 48, gScenarioKernelData233_CONVOLVE_5X5_16S, 4},
{14, 18, 2376, 32, 1, gScenarioBufferData234_CONVOLVE_5X5_16S, 48, gScenarioKernelData234_CONVOLVE_5X5_16S, 4},
{16, 18, 2672, 32, 1, gScenarioBufferData235_CONVOLVE_5X5_16S, 48, gScenarioKernelData235_CONVOLVE_5X5_16S, 4},
{18, 18, 2968, 32, 1, gScenarioBufferData236_CONVOLVE_5X5_16S, 48, gScenarioKernelData236_CONVOLVE_5X5_16S, 4},
{20, 18, 3264, 32, 1, gScenarioBufferData237_CONVOLVE_5X5_16S, 48, gScenarioKernelData237_CONVOLVE_5X5_16S, 4},
{22, 18, 3560, 32, 1, gScenarioBufferData238_CONVOLVE_5X5_16S, 48, gScenarioKernelData238_CONVOLVE_5X5_16S, 4},
{2, 20, 664, 32, 1, gScenarioBufferData239_CONVOLVE_5X5_16S, 48, gScenarioKernelData239_CONVOLVE_5X5_16S, 4},
{4, 20, 992, 32, 1, gScenarioBufferData240_CONVOLVE_5X5_16S, 48, gScenarioKernelData240_CONVOLVE_5X5_16S, 4},
{6, 20, 1320, 32, 1, gScenarioBufferData241_CONVOLVE_5X5_16S, 48, gScenarioKernelData241_CONVOLVE_5X5_16S, 4},
{8, 20, 1648, 32, 1, gScenarioBufferData242_CONVOLVE_5X5_16S, 48, gScenarioKernelData242_CONVOLVE_5X5_16S, 4},
{10, 20, 1976, 32, 1, gScenarioBufferData243_CONVOLVE_5X5_16S, 48, gScenarioKernelData243_CONVOLVE_5X5_16S, 4},
{12, 20, 2304, 32, 1, gScenarioBufferData244_CONVOLVE_5X5_16S, 48, gScenarioKernelData244_CONVOLVE_5X5_16S, 4},
{14, 20, 2632, 32, 1, gScenarioBufferData245_CONVOLVE_5X5_16S, 48, gScenarioKernelData245_CONVOLVE_5X5_16S, 4},
{16, 20, 2960, 32, 1, gScenarioBufferData246_CONVOLVE_5X5_16S, 48, gScenarioKernelData246_CONVOLVE_5X5_16S, 4},
{18, 20, 3288, 32, 1, gScenarioBufferData247_CONVOLVE_5X5_16S, 48, gScenarioKernelData247_CONVOLVE_5X5_16S, 4},
{20, 20, 3616, 32, 1, gScenarioBufferData248_CONVOLVE_5X5_16S, 48, gScenarioKernelData248_CONVOLVE_5X5_16S, 4},
{2, 22, 728, 32, 1, gScenarioBufferData249_CONVOLVE_5X5_16S, 48, gScenarioKernelData249_CONVOLVE_5X5_16S, 4},
{4, 22, 1088, 32, 1, gScenarioBufferData250_CONVOLVE_5X5_16S, 48, gScenarioKernelData250_CONVOLVE_5X5_16S, 4},
{6, 22, 1448, 32, 1, gScenarioBufferData251_CONVOLVE_5X5_16S, 48, gScenarioKernelData251_CONVOLVE_5X5_16S, 4},
{8, 22, 1808, 32, 1, gScenarioBufferData252_CONVOLVE_5X5_16S, 48, gScenarioKernelData252_CONVOLVE_5X5_16S, 4},
{10, 22, 2168, 32, 1, gScenarioBufferData253_CONVOLVE_5X5_16S, 48, gScenarioKernelData253_CONVOLVE_5X5_16S, 4},
{12, 22, 2528, 32, 1, gScenarioBufferData254_CONVOLVE_5X5_16S, 48, gScenarioKernelData254_CONVOLVE_5X5_16S, 4},
{14, 22, 2888, 32, 1, gScenarioBufferData255_CONVOLVE_5X5_16S, 48, gScenarioKernelData255_CONVOLVE_5X5_16S, 4},
{16, 22, 3248, 32, 1, gScenarioBufferData256_CONVOLVE_5X5_16S, 48, gScenarioKernelData256_CONVOLVE_5X5_16S, 4},
{18, 22, 3608, 32, 1, gScenarioBufferData257_CONVOLVE_5X5_16S, 48, gScenarioKernelData257_CONVOLVE_5X5_16S, 4},
{2, 24, 792, 32, 1, gScenarioBufferData258_CONVOLVE_5X5_16S, 48, gScenarioKernelData258_CONVOLVE_5X5_16S, 4},
{4, 24, 1184, 32, 1, gScenarioBufferData259_CONVOLVE_5X5_16S, 48, gScenarioKernelData259_CONVOLVE_5X5_16S, 4},
{6, 24, 1576, 32, 1, gScenarioBufferData260_CONVOLVE_5X5_16S, 48, gScenarioKernelData260_CONVOLVE_5X5_16S, 4},
{8, 24, 1968, 32, 1, gScenarioBufferData261_CONVOLVE_5X5_16S, 48, gScenarioKernelData261_CONVOLVE_5X5_16S, 4},
{10, 24, 2360, 32, 1, gScenarioBufferData262_CONVOLVE_5X5_16S, 48, gScenarioKernelData262_CONVOLVE_5X5_16S, 4},
{12, 24, 2752, 32, 1, gScenarioBufferData263_CONVOLVE_5X5_16S, 48, gScenarioKernelData263_CONVOLVE_5X5_16S, 4},
{14, 24, 3144, 32, 1, gScenarioBufferData264_CONVOLVE_5X5_16S, 48, gScenarioKernelData264_CONVOLVE_5X5_16S, 4},
{16, 24, 3536, 32, 1, gScenarioBufferData265_CONVOLVE_5X5_16S, 48, gScenarioKernelData265_CONVOLVE_5X5_16S, 4},
{2, 26, 856, 32, 1, gScenarioBufferData266_CONVOLVE_5X5_16S, 48, gScenarioKernelData266_CONVOLVE_5X5_16S, 4},
{4, 26, 1280, 32, 1, gScenarioBufferData267_CONVOLVE_5X5_16S, 48, gScenarioKernelData267_CONVOLVE_5X5_16S, 4},
{6, 26, 1704, 32, 1, gScenarioBufferData268_CONVOLVE_5X5_16S, 48, gScenarioKernelData268_CONVOLVE_5X5_16S, 4},
{8, 26, 2128, 32, 1, gScenarioBufferData269_CONVOLVE_5X5_16S, 48, gScenarioKernelData269_CONVOLVE_5X5_16S, 4},
{10, 26, 2552, 32, 1, gScenarioBufferData270_CONVOLVE_5X5_16S, 48, gScenarioKernelData270_CONVOLVE_5X5_16S, 4},
{12, 26, 2976, 32, 1, gScenarioBufferData271_CONVOLVE_5X5_16S, 48, gScenarioKernelData271_CONVOLVE_5X5_16S, 4},
{14, 26, 3400, 32, 1, gScenarioBufferData272_CONVOLVE_5X5_16S, 48, gScenarioKernelData272_CONVOLVE_5X5_16S, 4},
{16, 26, 3824, 32, 1, gScenarioBufferData273_CONVOLVE_5X5_16S, 48, gScenarioKernelData273_CONVOLVE_5X5_16S, 4},
{2, 28, 920, 32, 1, gScenarioBufferData274_CONVOLVE_5X5_16S, 48, gScenarioKernelData274_CONVOLVE_5X5_16S, 4},
{4, 28, 1376, 32, 1, gScenarioBufferData275_CONVOLVE_5X5_16S, 48, gScenarioKernelData275_CONVOLVE_5X5_16S, 4},
{6, 28, 1832, 32, 1, gScenarioBufferData276_CONVOLVE_5X5_16S, 48, gScenarioKernelData276_CONVOLVE_5X5_16S, 4},
{8, 28, 2288, 32, 1, gScenarioBufferData277_CONVOLVE_5X5_16S, 48, gScenarioKernelData277_CONVOLVE_5X5_16S, 4},
{10, 28, 2744, 32, 1, gScenarioBufferData278_CONVOLVE_5X5_16S, 48, gScenarioKernelData278_CONVOLVE_5X5_16S, 4},
{12, 28, 3200, 32, 1, gScenarioBufferData279_CONVOLVE_5X5_16S, 48, gScenarioKernelData279_CONVOLVE_5X5_16S, 4},
{14, 28, 3656, 32, 1, gScenarioBufferData280_CONVOLVE_5X5_16S, 48, gScenarioKernelData280_CONVOLVE_5X5_16S, 4},
{2, 30, 984, 32, 1, gScenarioBufferData281_CONVOLVE_5X5_16S, 48, gScenarioKernelData281_CONVOLVE_5X5_16S, 4},
{4, 30, 1472, 32, 1, gScenarioBufferData282_CONVOLVE_5X5_16S, 48, gScenarioKernelData282_CONVOLVE_5X5_16S, 4},
{6, 30, 1960, 32, 1, gScenarioBufferData283_CONVOLVE_5X5_16S, 48, gScenarioKernelData283_CONVOLVE_5X5_16S, 4},
{8, 30, 2448, 32, 1, gScenarioBufferData284_CONVOLVE_5X5_16S, 48, gScenarioKernelData284_CONVOLVE_5X5_16S, 4},
{10, 30, 2936, 32, 1, gScenarioBufferData285_CONVOLVE_5X5_16S, 48, gScenarioKernelData285_CONVOLVE_5X5_16S, 4},
{12, 30, 3424, 32, 1, gScenarioBufferData286_CONVOLVE_5X5_16S, 48, gScenarioKernelData286_CONVOLVE_5X5_16S, 4},
{2, 32, 1048, 32, 1, gScenarioBufferData287_CONVOLVE_5X5_16S, 48, gScenarioKernelData287_CONVOLVE_5X5_16S, 4},
{4, 32, 1568, 32, 1, gScenarioBufferData288_CONVOLVE_5X5_16S, 48, gScenarioKernelData288_CONVOLVE_5X5_16S, 4},
{6, 32, 2088, 32, 1, gScenarioBufferData289_CONVOLVE_5X5_16S, 48, gScenarioKernelData289_CONVOLVE_5X5_16S, 4},
{8, 32, 2608, 32, 1, gScenarioBufferData290_CONVOLVE_5X5_16S, 48, gScenarioKernelData290_CONVOLVE_5X5_16S, 4},
{10, 32, 3128, 32, 1, gScenarioBufferData291_CONVOLVE_5X5_16S, 48, gScenarioKernelData291_CONVOLVE_5X5_16S, 4},
{12, 32, 3648, 32, 1, gScenarioBufferData292_CONVOLVE_5X5_16S, 48, gScenarioKernelData292_CONVOLVE_5X5_16S, 4}
};

static acf_scenario_list gScenarioList_CONVOLVE_5X5_16S = {
293, //number of scenarios
gScenarioArray_CONVOLVE_5X5_16S};
//**************************************************************

class CONVOLVE_5X5_16S : public ACF_Process_APU
{

public:
   CONVOLVE_5X5_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CONVOLVE_5X5_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CONVOLVE_5X5_16S",
                                        CONVOLVE_5X5_16S_LOAD_SEGMENTS,
                                        CONVOLVE_5X5_16S_LOAD_PMEM, CONVOLVE_5X5_16S_LOAD_PMEM_SIZE,
                                        CONVOLVE_5X5_16S_LOAD_DMEM, CONVOLVE_5X5_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CONVOLVE_5X5_16S_APEX_LOG_BUFFER, CONVOLVE_5X5_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("F_COEFF", icp::DATATYPE_08S, 1, 1, 25, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CONVOLVE_5X5_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CONVOLVE_5X5_16S

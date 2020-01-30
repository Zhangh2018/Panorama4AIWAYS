#ifndef _ACF_PROCESS_APU_ACCUMULATE_16S
#define _ACF_PROCESS_APU_ACCUMULATE_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <ACCUMULATE_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_ACCUMULATE_16S[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_ACCUMULATE_16S[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_ACCUMULATE_16S[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_ACCUMULATE_16S[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_ACCUMULATE_16S[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_ACCUMULATE_16S[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_ACCUMULATE_16S[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_ACCUMULATE_16S[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_ACCUMULATE_16S[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_ACCUMULATE_16S[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_ACCUMULATE_16S[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_ACCUMULATE_16S[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_ACCUMULATE_16S[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_ACCUMULATE_16S[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_ACCUMULATE_16S[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_ACCUMULATE_16S[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_ACCUMULATE_16S[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_ACCUMULATE_16S[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_ACCUMULATE_16S[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_ACCUMULATE_16S[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_ACCUMULATE_16S[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_ACCUMULATE_16S[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_ACCUMULATE_16S[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_ACCUMULATE_16S[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_ACCUMULATE_16S[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_ACCUMULATE_16S[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_ACCUMULATE_16S[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_ACCUMULATE_16S[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_ACCUMULATE_16S[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_ACCUMULATE_16S[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_ACCUMULATE_16S[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_ACCUMULATE_16S[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_ACCUMULATE_16S[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_ACCUMULATE_16S[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_ACCUMULATE_16S[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_ACCUMULATE_16S[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_ACCUMULATE_16S[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_ACCUMULATE_16S[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_ACCUMULATE_16S[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_ACCUMULATE_16S[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_ACCUMULATE_16S[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_ACCUMULATE_16S[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_ACCUMULATE_16S[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_ACCUMULATE_16S[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_ACCUMULATE_16S[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_ACCUMULATE_16S[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_ACCUMULATE_16S[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_ACCUMULATE_16S[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_ACCUMULATE_16S[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_ACCUMULATE_16S[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_ACCUMULATE_16S[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_ACCUMULATE_16S[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_ACCUMULATE_16S[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_ACCUMULATE_16S[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_ACCUMULATE_16S[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_ACCUMULATE_16S[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_ACCUMULATE_16S[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_ACCUMULATE_16S[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_ACCUMULATE_16S[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_ACCUMULATE_16S[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_ACCUMULATE_16S[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_ACCUMULATE_16S[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_ACCUMULATE_16S[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_ACCUMULATE_16S[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_ACCUMULATE_16S[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_ACCUMULATE_16S[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_ACCUMULATE_16S[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_ACCUMULATE_16S[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_ACCUMULATE_16S[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_ACCUMULATE_16S[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_ACCUMULATE_16S[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_ACCUMULATE_16S[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_ACCUMULATE_16S[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_ACCUMULATE_16S[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_ACCUMULATE_16S[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_ACCUMULATE_16S[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_ACCUMULATE_16S[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_ACCUMULATE_16S[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_ACCUMULATE_16S[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_ACCUMULATE_16S[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_ACCUMULATE_16S[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_ACCUMULATE_16S[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_ACCUMULATE_16S[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_ACCUMULATE_16S[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_ACCUMULATE_16S[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_ACCUMULATE_16S[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_ACCUMULATE_16S[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_ACCUMULATE_16S[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_ACCUMULATE_16S[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_ACCUMULATE_16S[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_ACCUMULATE_16S[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_ACCUMULATE_16S[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_ACCUMULATE_16S[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_ACCUMULATE_16S[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_ACCUMULATE_16S[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_ACCUMULATE_16S[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_ACCUMULATE_16S[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_ACCUMULATE_16S[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_ACCUMULATE_16S[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_ACCUMULATE_16S[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_ACCUMULATE_16S[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_ACCUMULATE_16S[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_ACCUMULATE_16S[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_ACCUMULATE_16S[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_ACCUMULATE_16S[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_ACCUMULATE_16S[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_ACCUMULATE_16S[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_ACCUMULATE_16S[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_ACCUMULATE_16S[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_ACCUMULATE_16S[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_ACCUMULATE_16S[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_ACCUMULATE_16S[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_ACCUMULATE_16S[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_ACCUMULATE_16S[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_ACCUMULATE_16S[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_ACCUMULATE_16S[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_ACCUMULATE_16S[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_ACCUMULATE_16S[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_ACCUMULATE_16S[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_ACCUMULATE_16S[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_ACCUMULATE_16S[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_ACCUMULATE_16S[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_ACCUMULATE_16S[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_ACCUMULATE_16S[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_ACCUMULATE_16S[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_ACCUMULATE_16S[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_ACCUMULATE_16S[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_ACCUMULATE_16S[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_ACCUMULATE_16S[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_ACCUMULATE_16S[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_ACCUMULATE_16S[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_ACCUMULATE_16S[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_ACCUMULATE_16S[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_ACCUMULATE_16S[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_ACCUMULATE_16S[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_ACCUMULATE_16S[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_ACCUMULATE_16S[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_ACCUMULATE_16S[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_ACCUMULATE_16S[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_ACCUMULATE_16S[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_ACCUMULATE_16S[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_ACCUMULATE_16S[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_ACCUMULATE_16S[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_ACCUMULATE_16S[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_ACCUMULATE_16S[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_ACCUMULATE_16S[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_ACCUMULATE_16S[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_ACCUMULATE_16S[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_ACCUMULATE_16S[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_ACCUMULATE_16S[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_ACCUMULATE_16S[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_ACCUMULATE_16S[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_ACCUMULATE_16S[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_ACCUMULATE_16S[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_ACCUMULATE_16S[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_ACCUMULATE_16S[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_ACCUMULATE_16S[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_ACCUMULATE_16S[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_ACCUMULATE_16S[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_ACCUMULATE_16S[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_ACCUMULATE_16S[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_ACCUMULATE_16S[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_ACCUMULATE_16S[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_ACCUMULATE_16S[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_ACCUMULATE_16S[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_ACCUMULATE_16S[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_ACCUMULATE_16S[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_ACCUMULATE_16S[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_ACCUMULATE_16S[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_ACCUMULATE_16S[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_ACCUMULATE_16S[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_ACCUMULATE_16S[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_ACCUMULATE_16S[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_ACCUMULATE_16S[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_ACCUMULATE_16S[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_ACCUMULATE_16S[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_ACCUMULATE_16S[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_ACCUMULATE_16S[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_ACCUMULATE_16S[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_ACCUMULATE_16S[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_ACCUMULATE_16S[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_ACCUMULATE_16S[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_ACCUMULATE_16S[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_ACCUMULATE_16S[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_ACCUMULATE_16S[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_ACCUMULATE_16S[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_ACCUMULATE_16S[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_ACCUMULATE_16S[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_ACCUMULATE_16S[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_ACCUMULATE_16S[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_ACCUMULATE_16S[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_ACCUMULATE_16S[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_ACCUMULATE_16S[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_ACCUMULATE_16S[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_ACCUMULATE_16S[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_ACCUMULATE_16S[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_ACCUMULATE_16S[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_ACCUMULATE_16S[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_ACCUMULATE_16S[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_ACCUMULATE_16S[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_ACCUMULATE_16S[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_ACCUMULATE_16S[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_ACCUMULATE_16S[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_ACCUMULATE_16S[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_ACCUMULATE_16S[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_ACCUMULATE_16S[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_ACCUMULATE_16S[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_ACCUMULATE_16S[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_ACCUMULATE_16S[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_ACCUMULATE_16S[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_ACCUMULATE_16S[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_ACCUMULATE_16S[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_ACCUMULATE_16S[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_ACCUMULATE_16S[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_ACCUMULATE_16S[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_ACCUMULATE_16S[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_ACCUMULATE_16S[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_ACCUMULATE_16S[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_ACCUMULATE_16S[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_ACCUMULATE_16S[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_ACCUMULATE_16S[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_ACCUMULATE_16S[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_ACCUMULATE_16S[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_ACCUMULATE_16S[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_ACCUMULATE_16S[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_ACCUMULATE_16S[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_ACCUMULATE_16S[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_ACCUMULATE_16S[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_ACCUMULATE_16S[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_ACCUMULATE_16S[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_ACCUMULATE_16S[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_ACCUMULATE_16S[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_ACCUMULATE_16S[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_ACCUMULATE_16S[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_ACCUMULATE_16S[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_ACCUMULATE_16S[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_ACCUMULATE_16S[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_ACCUMULATE_16S[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_ACCUMULATE_16S[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_ACCUMULATE_16S[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_ACCUMULATE_16S[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_ACCUMULATE_16S[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_ACCUMULATE_16S[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_ACCUMULATE_16S[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_ACCUMULATE_16S[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_ACCUMULATE_16S[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_ACCUMULATE_16S[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_ACCUMULATE_16S[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_ACCUMULATE_16S[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_ACCUMULATE_16S[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_ACCUMULATE_16S[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_ACCUMULATE_16S[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_ACCUMULATE_16S[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_ACCUMULATE_16S[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_ACCUMULATE_16S[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_ACCUMULATE_16S[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_ACCUMULATE_16S[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_ACCUMULATE_16S[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_ACCUMULATE_16S[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_ACCUMULATE_16S[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_ACCUMULATE_16S[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_ACCUMULATE_16S[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_ACCUMULATE_16S[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_ACCUMULATE_16S[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_ACCUMULATE_16S[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_ACCUMULATE_16S[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_ACCUMULATE_16S[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_ACCUMULATE_16S[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_ACCUMULATE_16S[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_ACCUMULATE_16S[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_ACCUMULATE_16S[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_ACCUMULATE_16S[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_ACCUMULATE_16S[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_ACCUMULATE_16S[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_ACCUMULATE_16S[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_ACCUMULATE_16S[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_ACCUMULATE_16S[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_ACCUMULATE_16S[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_ACCUMULATE_16S[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_ACCUMULATE_16S[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_ACCUMULATE_16S[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_ACCUMULATE_16S[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_ACCUMULATE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_ACCUMULATE_16S[] = {{0, 0}};

static acf_scenario gScenarioArray_ACCUMULATE_16S[] = {
{2, 1, 20, 0, 0, gScenarioBufferData0_ACCUMULATE_16S, 48, gScenarioKernelData0_ACCUMULATE_16S, 4},
{4, 1, 40, 0, 0, gScenarioBufferData1_ACCUMULATE_16S, 48, gScenarioKernelData1_ACCUMULATE_16S, 4},
{6, 1, 60, 0, 0, gScenarioBufferData2_ACCUMULATE_16S, 48, gScenarioKernelData2_ACCUMULATE_16S, 4},
{8, 1, 80, 0, 0, gScenarioBufferData3_ACCUMULATE_16S, 48, gScenarioKernelData3_ACCUMULATE_16S, 4},
{10, 1, 100, 0, 0, gScenarioBufferData4_ACCUMULATE_16S, 48, gScenarioKernelData4_ACCUMULATE_16S, 4},
{12, 1, 120, 0, 0, gScenarioBufferData5_ACCUMULATE_16S, 48, gScenarioKernelData5_ACCUMULATE_16S, 4},
{14, 1, 140, 0, 0, gScenarioBufferData6_ACCUMULATE_16S, 48, gScenarioKernelData6_ACCUMULATE_16S, 4},
{16, 1, 160, 0, 0, gScenarioBufferData7_ACCUMULATE_16S, 48, gScenarioKernelData7_ACCUMULATE_16S, 4},
{18, 1, 180, 0, 0, gScenarioBufferData8_ACCUMULATE_16S, 48, gScenarioKernelData8_ACCUMULATE_16S, 4},
{20, 1, 200, 0, 0, gScenarioBufferData9_ACCUMULATE_16S, 48, gScenarioKernelData9_ACCUMULATE_16S, 4},
{22, 1, 220, 0, 0, gScenarioBufferData10_ACCUMULATE_16S, 48, gScenarioKernelData10_ACCUMULATE_16S, 4},
{24, 1, 240, 0, 0, gScenarioBufferData11_ACCUMULATE_16S, 48, gScenarioKernelData11_ACCUMULATE_16S, 4},
{26, 1, 260, 0, 0, gScenarioBufferData12_ACCUMULATE_16S, 48, gScenarioKernelData12_ACCUMULATE_16S, 4},
{28, 1, 280, 0, 0, gScenarioBufferData13_ACCUMULATE_16S, 48, gScenarioKernelData13_ACCUMULATE_16S, 4},
{30, 1, 300, 0, 0, gScenarioBufferData14_ACCUMULATE_16S, 48, gScenarioKernelData14_ACCUMULATE_16S, 4},
{32, 1, 320, 0, 0, gScenarioBufferData15_ACCUMULATE_16S, 48, gScenarioKernelData15_ACCUMULATE_16S, 4},
{36, 1, 360, 0, 0, gScenarioBufferData16_ACCUMULATE_16S, 48, gScenarioKernelData16_ACCUMULATE_16S, 4},
{40, 1, 400, 0, 0, gScenarioBufferData17_ACCUMULATE_16S, 48, gScenarioKernelData17_ACCUMULATE_16S, 4},
{44, 1, 440, 0, 0, gScenarioBufferData18_ACCUMULATE_16S, 48, gScenarioKernelData18_ACCUMULATE_16S, 4},
{48, 1, 480, 0, 0, gScenarioBufferData19_ACCUMULATE_16S, 48, gScenarioKernelData19_ACCUMULATE_16S, 4},
{52, 1, 520, 0, 0, gScenarioBufferData20_ACCUMULATE_16S, 48, gScenarioKernelData20_ACCUMULATE_16S, 4},
{56, 1, 560, 0, 0, gScenarioBufferData21_ACCUMULATE_16S, 48, gScenarioKernelData21_ACCUMULATE_16S, 4},
{60, 1, 600, 0, 0, gScenarioBufferData22_ACCUMULATE_16S, 48, gScenarioKernelData22_ACCUMULATE_16S, 4},
{64, 1, 640, 0, 0, gScenarioBufferData23_ACCUMULATE_16S, 48, gScenarioKernelData23_ACCUMULATE_16S, 4},
{2, 2, 40, 0, 0, gScenarioBufferData24_ACCUMULATE_16S, 48, gScenarioKernelData24_ACCUMULATE_16S, 4},
{4, 2, 80, 0, 0, gScenarioBufferData25_ACCUMULATE_16S, 48, gScenarioKernelData25_ACCUMULATE_16S, 4},
{6, 2, 120, 0, 0, gScenarioBufferData26_ACCUMULATE_16S, 48, gScenarioKernelData26_ACCUMULATE_16S, 4},
{8, 2, 160, 0, 0, gScenarioBufferData27_ACCUMULATE_16S, 48, gScenarioKernelData27_ACCUMULATE_16S, 4},
{10, 2, 200, 0, 0, gScenarioBufferData28_ACCUMULATE_16S, 48, gScenarioKernelData28_ACCUMULATE_16S, 4},
{12, 2, 240, 0, 0, gScenarioBufferData29_ACCUMULATE_16S, 48, gScenarioKernelData29_ACCUMULATE_16S, 4},
{14, 2, 280, 0, 0, gScenarioBufferData30_ACCUMULATE_16S, 48, gScenarioKernelData30_ACCUMULATE_16S, 4},
{16, 2, 320, 0, 0, gScenarioBufferData31_ACCUMULATE_16S, 48, gScenarioKernelData31_ACCUMULATE_16S, 4},
{18, 2, 360, 0, 0, gScenarioBufferData32_ACCUMULATE_16S, 48, gScenarioKernelData32_ACCUMULATE_16S, 4},
{20, 2, 400, 0, 0, gScenarioBufferData33_ACCUMULATE_16S, 48, gScenarioKernelData33_ACCUMULATE_16S, 4},
{22, 2, 440, 0, 0, gScenarioBufferData34_ACCUMULATE_16S, 48, gScenarioKernelData34_ACCUMULATE_16S, 4},
{24, 2, 480, 0, 0, gScenarioBufferData35_ACCUMULATE_16S, 48, gScenarioKernelData35_ACCUMULATE_16S, 4},
{26, 2, 520, 0, 0, gScenarioBufferData36_ACCUMULATE_16S, 48, gScenarioKernelData36_ACCUMULATE_16S, 4},
{28, 2, 560, 0, 0, gScenarioBufferData37_ACCUMULATE_16S, 48, gScenarioKernelData37_ACCUMULATE_16S, 4},
{30, 2, 600, 0, 0, gScenarioBufferData38_ACCUMULATE_16S, 48, gScenarioKernelData38_ACCUMULATE_16S, 4},
{32, 2, 640, 0, 0, gScenarioBufferData39_ACCUMULATE_16S, 48, gScenarioKernelData39_ACCUMULATE_16S, 4},
{36, 2, 720, 0, 0, gScenarioBufferData40_ACCUMULATE_16S, 48, gScenarioKernelData40_ACCUMULATE_16S, 4},
{40, 2, 800, 0, 0, gScenarioBufferData41_ACCUMULATE_16S, 48, gScenarioKernelData41_ACCUMULATE_16S, 4},
{44, 2, 880, 0, 0, gScenarioBufferData42_ACCUMULATE_16S, 48, gScenarioKernelData42_ACCUMULATE_16S, 4},
{48, 2, 960, 0, 0, gScenarioBufferData43_ACCUMULATE_16S, 48, gScenarioKernelData43_ACCUMULATE_16S, 4},
{52, 2, 1040, 0, 0, gScenarioBufferData44_ACCUMULATE_16S, 48, gScenarioKernelData44_ACCUMULATE_16S, 4},
{56, 2, 1120, 0, 0, gScenarioBufferData45_ACCUMULATE_16S, 48, gScenarioKernelData45_ACCUMULATE_16S, 4},
{60, 2, 1200, 0, 0, gScenarioBufferData46_ACCUMULATE_16S, 48, gScenarioKernelData46_ACCUMULATE_16S, 4},
{64, 2, 1280, 0, 0, gScenarioBufferData47_ACCUMULATE_16S, 48, gScenarioKernelData47_ACCUMULATE_16S, 4},
{2, 3, 60, 0, 0, gScenarioBufferData48_ACCUMULATE_16S, 48, gScenarioKernelData48_ACCUMULATE_16S, 4},
{4, 3, 120, 0, 0, gScenarioBufferData49_ACCUMULATE_16S, 48, gScenarioKernelData49_ACCUMULATE_16S, 4},
{6, 3, 180, 0, 0, gScenarioBufferData50_ACCUMULATE_16S, 48, gScenarioKernelData50_ACCUMULATE_16S, 4},
{8, 3, 240, 0, 0, gScenarioBufferData51_ACCUMULATE_16S, 48, gScenarioKernelData51_ACCUMULATE_16S, 4},
{10, 3, 300, 0, 0, gScenarioBufferData52_ACCUMULATE_16S, 48, gScenarioKernelData52_ACCUMULATE_16S, 4},
{12, 3, 360, 0, 0, gScenarioBufferData53_ACCUMULATE_16S, 48, gScenarioKernelData53_ACCUMULATE_16S, 4},
{14, 3, 420, 0, 0, gScenarioBufferData54_ACCUMULATE_16S, 48, gScenarioKernelData54_ACCUMULATE_16S, 4},
{16, 3, 480, 0, 0, gScenarioBufferData55_ACCUMULATE_16S, 48, gScenarioKernelData55_ACCUMULATE_16S, 4},
{18, 3, 540, 0, 0, gScenarioBufferData56_ACCUMULATE_16S, 48, gScenarioKernelData56_ACCUMULATE_16S, 4},
{20, 3, 600, 0, 0, gScenarioBufferData57_ACCUMULATE_16S, 48, gScenarioKernelData57_ACCUMULATE_16S, 4},
{22, 3, 660, 0, 0, gScenarioBufferData58_ACCUMULATE_16S, 48, gScenarioKernelData58_ACCUMULATE_16S, 4},
{24, 3, 720, 0, 0, gScenarioBufferData59_ACCUMULATE_16S, 48, gScenarioKernelData59_ACCUMULATE_16S, 4},
{26, 3, 780, 0, 0, gScenarioBufferData60_ACCUMULATE_16S, 48, gScenarioKernelData60_ACCUMULATE_16S, 4},
{28, 3, 840, 0, 0, gScenarioBufferData61_ACCUMULATE_16S, 48, gScenarioKernelData61_ACCUMULATE_16S, 4},
{30, 3, 900, 0, 0, gScenarioBufferData62_ACCUMULATE_16S, 48, gScenarioKernelData62_ACCUMULATE_16S, 4},
{32, 3, 960, 0, 0, gScenarioBufferData63_ACCUMULATE_16S, 48, gScenarioKernelData63_ACCUMULATE_16S, 4},
{36, 3, 1080, 0, 0, gScenarioBufferData64_ACCUMULATE_16S, 48, gScenarioKernelData64_ACCUMULATE_16S, 4},
{40, 3, 1200, 0, 0, gScenarioBufferData65_ACCUMULATE_16S, 48, gScenarioKernelData65_ACCUMULATE_16S, 4},
{44, 3, 1320, 0, 0, gScenarioBufferData66_ACCUMULATE_16S, 48, gScenarioKernelData66_ACCUMULATE_16S, 4},
{48, 3, 1440, 0, 0, gScenarioBufferData67_ACCUMULATE_16S, 48, gScenarioKernelData67_ACCUMULATE_16S, 4},
{52, 3, 1560, 0, 0, gScenarioBufferData68_ACCUMULATE_16S, 48, gScenarioKernelData68_ACCUMULATE_16S, 4},
{56, 3, 1680, 0, 0, gScenarioBufferData69_ACCUMULATE_16S, 48, gScenarioKernelData69_ACCUMULATE_16S, 4},
{60, 3, 1800, 0, 0, gScenarioBufferData70_ACCUMULATE_16S, 48, gScenarioKernelData70_ACCUMULATE_16S, 4},
{64, 3, 1920, 0, 0, gScenarioBufferData71_ACCUMULATE_16S, 48, gScenarioKernelData71_ACCUMULATE_16S, 4},
{2, 4, 80, 0, 0, gScenarioBufferData72_ACCUMULATE_16S, 48, gScenarioKernelData72_ACCUMULATE_16S, 4},
{4, 4, 160, 0, 0, gScenarioBufferData73_ACCUMULATE_16S, 48, gScenarioKernelData73_ACCUMULATE_16S, 4},
{6, 4, 240, 0, 0, gScenarioBufferData74_ACCUMULATE_16S, 48, gScenarioKernelData74_ACCUMULATE_16S, 4},
{8, 4, 320, 0, 0, gScenarioBufferData75_ACCUMULATE_16S, 48, gScenarioKernelData75_ACCUMULATE_16S, 4},
{10, 4, 400, 0, 0, gScenarioBufferData76_ACCUMULATE_16S, 48, gScenarioKernelData76_ACCUMULATE_16S, 4},
{12, 4, 480, 0, 0, gScenarioBufferData77_ACCUMULATE_16S, 48, gScenarioKernelData77_ACCUMULATE_16S, 4},
{14, 4, 560, 0, 0, gScenarioBufferData78_ACCUMULATE_16S, 48, gScenarioKernelData78_ACCUMULATE_16S, 4},
{16, 4, 640, 0, 0, gScenarioBufferData79_ACCUMULATE_16S, 48, gScenarioKernelData79_ACCUMULATE_16S, 4},
{18, 4, 720, 0, 0, gScenarioBufferData80_ACCUMULATE_16S, 48, gScenarioKernelData80_ACCUMULATE_16S, 4},
{20, 4, 800, 0, 0, gScenarioBufferData81_ACCUMULATE_16S, 48, gScenarioKernelData81_ACCUMULATE_16S, 4},
{22, 4, 880, 0, 0, gScenarioBufferData82_ACCUMULATE_16S, 48, gScenarioKernelData82_ACCUMULATE_16S, 4},
{24, 4, 960, 0, 0, gScenarioBufferData83_ACCUMULATE_16S, 48, gScenarioKernelData83_ACCUMULATE_16S, 4},
{26, 4, 1040, 0, 0, gScenarioBufferData84_ACCUMULATE_16S, 48, gScenarioKernelData84_ACCUMULATE_16S, 4},
{28, 4, 1120, 0, 0, gScenarioBufferData85_ACCUMULATE_16S, 48, gScenarioKernelData85_ACCUMULATE_16S, 4},
{30, 4, 1200, 0, 0, gScenarioBufferData86_ACCUMULATE_16S, 48, gScenarioKernelData86_ACCUMULATE_16S, 4},
{32, 4, 1280, 0, 0, gScenarioBufferData87_ACCUMULATE_16S, 48, gScenarioKernelData87_ACCUMULATE_16S, 4},
{36, 4, 1440, 0, 0, gScenarioBufferData88_ACCUMULATE_16S, 48, gScenarioKernelData88_ACCUMULATE_16S, 4},
{40, 4, 1600, 0, 0, gScenarioBufferData89_ACCUMULATE_16S, 48, gScenarioKernelData89_ACCUMULATE_16S, 4},
{44, 4, 1760, 0, 0, gScenarioBufferData90_ACCUMULATE_16S, 48, gScenarioKernelData90_ACCUMULATE_16S, 4},
{48, 4, 1920, 0, 0, gScenarioBufferData91_ACCUMULATE_16S, 48, gScenarioKernelData91_ACCUMULATE_16S, 4},
{52, 4, 2080, 0, 0, gScenarioBufferData92_ACCUMULATE_16S, 48, gScenarioKernelData92_ACCUMULATE_16S, 4},
{56, 4, 2240, 0, 0, gScenarioBufferData93_ACCUMULATE_16S, 48, gScenarioKernelData93_ACCUMULATE_16S, 4},
{60, 4, 2400, 0, 0, gScenarioBufferData94_ACCUMULATE_16S, 48, gScenarioKernelData94_ACCUMULATE_16S, 4},
{64, 4, 2560, 0, 0, gScenarioBufferData95_ACCUMULATE_16S, 48, gScenarioKernelData95_ACCUMULATE_16S, 4},
{2, 5, 100, 0, 0, gScenarioBufferData96_ACCUMULATE_16S, 48, gScenarioKernelData96_ACCUMULATE_16S, 4},
{4, 5, 200, 0, 0, gScenarioBufferData97_ACCUMULATE_16S, 48, gScenarioKernelData97_ACCUMULATE_16S, 4},
{6, 5, 300, 0, 0, gScenarioBufferData98_ACCUMULATE_16S, 48, gScenarioKernelData98_ACCUMULATE_16S, 4},
{8, 5, 400, 0, 0, gScenarioBufferData99_ACCUMULATE_16S, 48, gScenarioKernelData99_ACCUMULATE_16S, 4},
{10, 5, 500, 0, 0, gScenarioBufferData100_ACCUMULATE_16S, 48, gScenarioKernelData100_ACCUMULATE_16S, 4},
{12, 5, 600, 0, 0, gScenarioBufferData101_ACCUMULATE_16S, 48, gScenarioKernelData101_ACCUMULATE_16S, 4},
{14, 5, 700, 0, 0, gScenarioBufferData102_ACCUMULATE_16S, 48, gScenarioKernelData102_ACCUMULATE_16S, 4},
{16, 5, 800, 0, 0, gScenarioBufferData103_ACCUMULATE_16S, 48, gScenarioKernelData103_ACCUMULATE_16S, 4},
{18, 5, 900, 0, 0, gScenarioBufferData104_ACCUMULATE_16S, 48, gScenarioKernelData104_ACCUMULATE_16S, 4},
{20, 5, 1000, 0, 0, gScenarioBufferData105_ACCUMULATE_16S, 48, gScenarioKernelData105_ACCUMULATE_16S, 4},
{22, 5, 1100, 0, 0, gScenarioBufferData106_ACCUMULATE_16S, 48, gScenarioKernelData106_ACCUMULATE_16S, 4},
{24, 5, 1200, 0, 0, gScenarioBufferData107_ACCUMULATE_16S, 48, gScenarioKernelData107_ACCUMULATE_16S, 4},
{26, 5, 1300, 0, 0, gScenarioBufferData108_ACCUMULATE_16S, 48, gScenarioKernelData108_ACCUMULATE_16S, 4},
{28, 5, 1400, 0, 0, gScenarioBufferData109_ACCUMULATE_16S, 48, gScenarioKernelData109_ACCUMULATE_16S, 4},
{30, 5, 1500, 0, 0, gScenarioBufferData110_ACCUMULATE_16S, 48, gScenarioKernelData110_ACCUMULATE_16S, 4},
{32, 5, 1600, 0, 0, gScenarioBufferData111_ACCUMULATE_16S, 48, gScenarioKernelData111_ACCUMULATE_16S, 4},
{36, 5, 1800, 0, 0, gScenarioBufferData112_ACCUMULATE_16S, 48, gScenarioKernelData112_ACCUMULATE_16S, 4},
{40, 5, 2000, 0, 0, gScenarioBufferData113_ACCUMULATE_16S, 48, gScenarioKernelData113_ACCUMULATE_16S, 4},
{44, 5, 2200, 0, 0, gScenarioBufferData114_ACCUMULATE_16S, 48, gScenarioKernelData114_ACCUMULATE_16S, 4},
{48, 5, 2400, 0, 0, gScenarioBufferData115_ACCUMULATE_16S, 48, gScenarioKernelData115_ACCUMULATE_16S, 4},
{52, 5, 2600, 0, 0, gScenarioBufferData116_ACCUMULATE_16S, 48, gScenarioKernelData116_ACCUMULATE_16S, 4},
{56, 5, 2800, 0, 0, gScenarioBufferData117_ACCUMULATE_16S, 48, gScenarioKernelData117_ACCUMULATE_16S, 4},
{60, 5, 3000, 0, 0, gScenarioBufferData118_ACCUMULATE_16S, 48, gScenarioKernelData118_ACCUMULATE_16S, 4},
{64, 5, 3200, 0, 0, gScenarioBufferData119_ACCUMULATE_16S, 48, gScenarioKernelData119_ACCUMULATE_16S, 4},
{2, 6, 120, 0, 0, gScenarioBufferData120_ACCUMULATE_16S, 48, gScenarioKernelData120_ACCUMULATE_16S, 4},
{4, 6, 240, 0, 0, gScenarioBufferData121_ACCUMULATE_16S, 48, gScenarioKernelData121_ACCUMULATE_16S, 4},
{6, 6, 360, 0, 0, gScenarioBufferData122_ACCUMULATE_16S, 48, gScenarioKernelData122_ACCUMULATE_16S, 4},
{8, 6, 480, 0, 0, gScenarioBufferData123_ACCUMULATE_16S, 48, gScenarioKernelData123_ACCUMULATE_16S, 4},
{10, 6, 600, 0, 0, gScenarioBufferData124_ACCUMULATE_16S, 48, gScenarioKernelData124_ACCUMULATE_16S, 4},
{12, 6, 720, 0, 0, gScenarioBufferData125_ACCUMULATE_16S, 48, gScenarioKernelData125_ACCUMULATE_16S, 4},
{14, 6, 840, 0, 0, gScenarioBufferData126_ACCUMULATE_16S, 48, gScenarioKernelData126_ACCUMULATE_16S, 4},
{16, 6, 960, 0, 0, gScenarioBufferData127_ACCUMULATE_16S, 48, gScenarioKernelData127_ACCUMULATE_16S, 4},
{18, 6, 1080, 0, 0, gScenarioBufferData128_ACCUMULATE_16S, 48, gScenarioKernelData128_ACCUMULATE_16S, 4},
{20, 6, 1200, 0, 0, gScenarioBufferData129_ACCUMULATE_16S, 48, gScenarioKernelData129_ACCUMULATE_16S, 4},
{22, 6, 1320, 0, 0, gScenarioBufferData130_ACCUMULATE_16S, 48, gScenarioKernelData130_ACCUMULATE_16S, 4},
{24, 6, 1440, 0, 0, gScenarioBufferData131_ACCUMULATE_16S, 48, gScenarioKernelData131_ACCUMULATE_16S, 4},
{26, 6, 1560, 0, 0, gScenarioBufferData132_ACCUMULATE_16S, 48, gScenarioKernelData132_ACCUMULATE_16S, 4},
{28, 6, 1680, 0, 0, gScenarioBufferData133_ACCUMULATE_16S, 48, gScenarioKernelData133_ACCUMULATE_16S, 4},
{30, 6, 1800, 0, 0, gScenarioBufferData134_ACCUMULATE_16S, 48, gScenarioKernelData134_ACCUMULATE_16S, 4},
{32, 6, 1920, 0, 0, gScenarioBufferData135_ACCUMULATE_16S, 48, gScenarioKernelData135_ACCUMULATE_16S, 4},
{36, 6, 2160, 0, 0, gScenarioBufferData136_ACCUMULATE_16S, 48, gScenarioKernelData136_ACCUMULATE_16S, 4},
{40, 6, 2400, 0, 0, gScenarioBufferData137_ACCUMULATE_16S, 48, gScenarioKernelData137_ACCUMULATE_16S, 4},
{44, 6, 2640, 0, 0, gScenarioBufferData138_ACCUMULATE_16S, 48, gScenarioKernelData138_ACCUMULATE_16S, 4},
{48, 6, 2880, 0, 0, gScenarioBufferData139_ACCUMULATE_16S, 48, gScenarioKernelData139_ACCUMULATE_16S, 4},
{52, 6, 3120, 0, 0, gScenarioBufferData140_ACCUMULATE_16S, 48, gScenarioKernelData140_ACCUMULATE_16S, 4},
{56, 6, 3360, 0, 0, gScenarioBufferData141_ACCUMULATE_16S, 48, gScenarioKernelData141_ACCUMULATE_16S, 4},
{60, 6, 3600, 0, 0, gScenarioBufferData142_ACCUMULATE_16S, 48, gScenarioKernelData142_ACCUMULATE_16S, 4},
{64, 6, 3840, 0, 0, gScenarioBufferData143_ACCUMULATE_16S, 48, gScenarioKernelData143_ACCUMULATE_16S, 4},
{2, 8, 160, 0, 0, gScenarioBufferData144_ACCUMULATE_16S, 48, gScenarioKernelData144_ACCUMULATE_16S, 4},
{4, 8, 320, 0, 0, gScenarioBufferData145_ACCUMULATE_16S, 48, gScenarioKernelData145_ACCUMULATE_16S, 4},
{6, 8, 480, 0, 0, gScenarioBufferData146_ACCUMULATE_16S, 48, gScenarioKernelData146_ACCUMULATE_16S, 4},
{8, 8, 640, 0, 0, gScenarioBufferData147_ACCUMULATE_16S, 48, gScenarioKernelData147_ACCUMULATE_16S, 4},
{10, 8, 800, 0, 0, gScenarioBufferData148_ACCUMULATE_16S, 48, gScenarioKernelData148_ACCUMULATE_16S, 4},
{12, 8, 960, 0, 0, gScenarioBufferData149_ACCUMULATE_16S, 48, gScenarioKernelData149_ACCUMULATE_16S, 4},
{14, 8, 1120, 0, 0, gScenarioBufferData150_ACCUMULATE_16S, 48, gScenarioKernelData150_ACCUMULATE_16S, 4},
{16, 8, 1280, 0, 0, gScenarioBufferData151_ACCUMULATE_16S, 48, gScenarioKernelData151_ACCUMULATE_16S, 4},
{18, 8, 1440, 0, 0, gScenarioBufferData152_ACCUMULATE_16S, 48, gScenarioKernelData152_ACCUMULATE_16S, 4},
{20, 8, 1600, 0, 0, gScenarioBufferData153_ACCUMULATE_16S, 48, gScenarioKernelData153_ACCUMULATE_16S, 4},
{22, 8, 1760, 0, 0, gScenarioBufferData154_ACCUMULATE_16S, 48, gScenarioKernelData154_ACCUMULATE_16S, 4},
{24, 8, 1920, 0, 0, gScenarioBufferData155_ACCUMULATE_16S, 48, gScenarioKernelData155_ACCUMULATE_16S, 4},
{26, 8, 2080, 0, 0, gScenarioBufferData156_ACCUMULATE_16S, 48, gScenarioKernelData156_ACCUMULATE_16S, 4},
{28, 8, 2240, 0, 0, gScenarioBufferData157_ACCUMULATE_16S, 48, gScenarioKernelData157_ACCUMULATE_16S, 4},
{30, 8, 2400, 0, 0, gScenarioBufferData158_ACCUMULATE_16S, 48, gScenarioKernelData158_ACCUMULATE_16S, 4},
{32, 8, 2560, 0, 0, gScenarioBufferData159_ACCUMULATE_16S, 48, gScenarioKernelData159_ACCUMULATE_16S, 4},
{36, 8, 2880, 0, 0, gScenarioBufferData160_ACCUMULATE_16S, 48, gScenarioKernelData160_ACCUMULATE_16S, 4},
{40, 8, 3200, 0, 0, gScenarioBufferData161_ACCUMULATE_16S, 48, gScenarioKernelData161_ACCUMULATE_16S, 4},
{44, 8, 3520, 0, 0, gScenarioBufferData162_ACCUMULATE_16S, 48, gScenarioKernelData162_ACCUMULATE_16S, 4},
{48, 8, 3840, 0, 0, gScenarioBufferData163_ACCUMULATE_16S, 48, gScenarioKernelData163_ACCUMULATE_16S, 4},
{2, 10, 200, 0, 0, gScenarioBufferData164_ACCUMULATE_16S, 48, gScenarioKernelData164_ACCUMULATE_16S, 4},
{4, 10, 400, 0, 0, gScenarioBufferData165_ACCUMULATE_16S, 48, gScenarioKernelData165_ACCUMULATE_16S, 4},
{6, 10, 600, 0, 0, gScenarioBufferData166_ACCUMULATE_16S, 48, gScenarioKernelData166_ACCUMULATE_16S, 4},
{8, 10, 800, 0, 0, gScenarioBufferData167_ACCUMULATE_16S, 48, gScenarioKernelData167_ACCUMULATE_16S, 4},
{10, 10, 1000, 0, 0, gScenarioBufferData168_ACCUMULATE_16S, 48, gScenarioKernelData168_ACCUMULATE_16S, 4},
{12, 10, 1200, 0, 0, gScenarioBufferData169_ACCUMULATE_16S, 48, gScenarioKernelData169_ACCUMULATE_16S, 4},
{14, 10, 1400, 0, 0, gScenarioBufferData170_ACCUMULATE_16S, 48, gScenarioKernelData170_ACCUMULATE_16S, 4},
{16, 10, 1600, 0, 0, gScenarioBufferData171_ACCUMULATE_16S, 48, gScenarioKernelData171_ACCUMULATE_16S, 4},
{18, 10, 1800, 0, 0, gScenarioBufferData172_ACCUMULATE_16S, 48, gScenarioKernelData172_ACCUMULATE_16S, 4},
{20, 10, 2000, 0, 0, gScenarioBufferData173_ACCUMULATE_16S, 48, gScenarioKernelData173_ACCUMULATE_16S, 4},
{22, 10, 2200, 0, 0, gScenarioBufferData174_ACCUMULATE_16S, 48, gScenarioKernelData174_ACCUMULATE_16S, 4},
{24, 10, 2400, 0, 0, gScenarioBufferData175_ACCUMULATE_16S, 48, gScenarioKernelData175_ACCUMULATE_16S, 4},
{26, 10, 2600, 0, 0, gScenarioBufferData176_ACCUMULATE_16S, 48, gScenarioKernelData176_ACCUMULATE_16S, 4},
{28, 10, 2800, 0, 0, gScenarioBufferData177_ACCUMULATE_16S, 48, gScenarioKernelData177_ACCUMULATE_16S, 4},
{30, 10, 3000, 0, 0, gScenarioBufferData178_ACCUMULATE_16S, 48, gScenarioKernelData178_ACCUMULATE_16S, 4},
{32, 10, 3200, 0, 0, gScenarioBufferData179_ACCUMULATE_16S, 48, gScenarioKernelData179_ACCUMULATE_16S, 4},
{36, 10, 3600, 0, 0, gScenarioBufferData180_ACCUMULATE_16S, 48, gScenarioKernelData180_ACCUMULATE_16S, 4},
{2, 12, 240, 0, 0, gScenarioBufferData181_ACCUMULATE_16S, 48, gScenarioKernelData181_ACCUMULATE_16S, 4},
{4, 12, 480, 0, 0, gScenarioBufferData182_ACCUMULATE_16S, 48, gScenarioKernelData182_ACCUMULATE_16S, 4},
{6, 12, 720, 0, 0, gScenarioBufferData183_ACCUMULATE_16S, 48, gScenarioKernelData183_ACCUMULATE_16S, 4},
{8, 12, 960, 0, 0, gScenarioBufferData184_ACCUMULATE_16S, 48, gScenarioKernelData184_ACCUMULATE_16S, 4},
{10, 12, 1200, 0, 0, gScenarioBufferData185_ACCUMULATE_16S, 48, gScenarioKernelData185_ACCUMULATE_16S, 4},
{12, 12, 1440, 0, 0, gScenarioBufferData186_ACCUMULATE_16S, 48, gScenarioKernelData186_ACCUMULATE_16S, 4},
{14, 12, 1680, 0, 0, gScenarioBufferData187_ACCUMULATE_16S, 48, gScenarioKernelData187_ACCUMULATE_16S, 4},
{16, 12, 1920, 0, 0, gScenarioBufferData188_ACCUMULATE_16S, 48, gScenarioKernelData188_ACCUMULATE_16S, 4},
{18, 12, 2160, 0, 0, gScenarioBufferData189_ACCUMULATE_16S, 48, gScenarioKernelData189_ACCUMULATE_16S, 4},
{20, 12, 2400, 0, 0, gScenarioBufferData190_ACCUMULATE_16S, 48, gScenarioKernelData190_ACCUMULATE_16S, 4},
{22, 12, 2640, 0, 0, gScenarioBufferData191_ACCUMULATE_16S, 48, gScenarioKernelData191_ACCUMULATE_16S, 4},
{24, 12, 2880, 0, 0, gScenarioBufferData192_ACCUMULATE_16S, 48, gScenarioKernelData192_ACCUMULATE_16S, 4},
{26, 12, 3120, 0, 0, gScenarioBufferData193_ACCUMULATE_16S, 48, gScenarioKernelData193_ACCUMULATE_16S, 4},
{28, 12, 3360, 0, 0, gScenarioBufferData194_ACCUMULATE_16S, 48, gScenarioKernelData194_ACCUMULATE_16S, 4},
{30, 12, 3600, 0, 0, gScenarioBufferData195_ACCUMULATE_16S, 48, gScenarioKernelData195_ACCUMULATE_16S, 4},
{32, 12, 3840, 0, 0, gScenarioBufferData196_ACCUMULATE_16S, 48, gScenarioKernelData196_ACCUMULATE_16S, 4},
{2, 14, 280, 0, 0, gScenarioBufferData197_ACCUMULATE_16S, 48, gScenarioKernelData197_ACCUMULATE_16S, 4},
{4, 14, 560, 0, 0, gScenarioBufferData198_ACCUMULATE_16S, 48, gScenarioKernelData198_ACCUMULATE_16S, 4},
{6, 14, 840, 0, 0, gScenarioBufferData199_ACCUMULATE_16S, 48, gScenarioKernelData199_ACCUMULATE_16S, 4},
{8, 14, 1120, 0, 0, gScenarioBufferData200_ACCUMULATE_16S, 48, gScenarioKernelData200_ACCUMULATE_16S, 4},
{10, 14, 1400, 0, 0, gScenarioBufferData201_ACCUMULATE_16S, 48, gScenarioKernelData201_ACCUMULATE_16S, 4},
{12, 14, 1680, 0, 0, gScenarioBufferData202_ACCUMULATE_16S, 48, gScenarioKernelData202_ACCUMULATE_16S, 4},
{14, 14, 1960, 0, 0, gScenarioBufferData203_ACCUMULATE_16S, 48, gScenarioKernelData203_ACCUMULATE_16S, 4},
{16, 14, 2240, 0, 0, gScenarioBufferData204_ACCUMULATE_16S, 48, gScenarioKernelData204_ACCUMULATE_16S, 4},
{18, 14, 2520, 0, 0, gScenarioBufferData205_ACCUMULATE_16S, 48, gScenarioKernelData205_ACCUMULATE_16S, 4},
{20, 14, 2800, 0, 0, gScenarioBufferData206_ACCUMULATE_16S, 48, gScenarioKernelData206_ACCUMULATE_16S, 4},
{22, 14, 3080, 0, 0, gScenarioBufferData207_ACCUMULATE_16S, 48, gScenarioKernelData207_ACCUMULATE_16S, 4},
{24, 14, 3360, 0, 0, gScenarioBufferData208_ACCUMULATE_16S, 48, gScenarioKernelData208_ACCUMULATE_16S, 4},
{26, 14, 3640, 0, 0, gScenarioBufferData209_ACCUMULATE_16S, 48, gScenarioKernelData209_ACCUMULATE_16S, 4},
{2, 16, 320, 0, 0, gScenarioBufferData210_ACCUMULATE_16S, 48, gScenarioKernelData210_ACCUMULATE_16S, 4},
{4, 16, 640, 0, 0, gScenarioBufferData211_ACCUMULATE_16S, 48, gScenarioKernelData211_ACCUMULATE_16S, 4},
{6, 16, 960, 0, 0, gScenarioBufferData212_ACCUMULATE_16S, 48, gScenarioKernelData212_ACCUMULATE_16S, 4},
{8, 16, 1280, 0, 0, gScenarioBufferData213_ACCUMULATE_16S, 48, gScenarioKernelData213_ACCUMULATE_16S, 4},
{10, 16, 1600, 0, 0, gScenarioBufferData214_ACCUMULATE_16S, 48, gScenarioKernelData214_ACCUMULATE_16S, 4},
{12, 16, 1920, 0, 0, gScenarioBufferData215_ACCUMULATE_16S, 48, gScenarioKernelData215_ACCUMULATE_16S, 4},
{14, 16, 2240, 0, 0, gScenarioBufferData216_ACCUMULATE_16S, 48, gScenarioKernelData216_ACCUMULATE_16S, 4},
{16, 16, 2560, 0, 0, gScenarioBufferData217_ACCUMULATE_16S, 48, gScenarioKernelData217_ACCUMULATE_16S, 4},
{18, 16, 2880, 0, 0, gScenarioBufferData218_ACCUMULATE_16S, 48, gScenarioKernelData218_ACCUMULATE_16S, 4},
{20, 16, 3200, 0, 0, gScenarioBufferData219_ACCUMULATE_16S, 48, gScenarioKernelData219_ACCUMULATE_16S, 4},
{22, 16, 3520, 0, 0, gScenarioBufferData220_ACCUMULATE_16S, 48, gScenarioKernelData220_ACCUMULATE_16S, 4},
{24, 16, 3840, 0, 0, gScenarioBufferData221_ACCUMULATE_16S, 48, gScenarioKernelData221_ACCUMULATE_16S, 4},
{2, 18, 360, 0, 0, gScenarioBufferData222_ACCUMULATE_16S, 48, gScenarioKernelData222_ACCUMULATE_16S, 4},
{4, 18, 720, 0, 0, gScenarioBufferData223_ACCUMULATE_16S, 48, gScenarioKernelData223_ACCUMULATE_16S, 4},
{6, 18, 1080, 0, 0, gScenarioBufferData224_ACCUMULATE_16S, 48, gScenarioKernelData224_ACCUMULATE_16S, 4},
{8, 18, 1440, 0, 0, gScenarioBufferData225_ACCUMULATE_16S, 48, gScenarioKernelData225_ACCUMULATE_16S, 4},
{10, 18, 1800, 0, 0, gScenarioBufferData226_ACCUMULATE_16S, 48, gScenarioKernelData226_ACCUMULATE_16S, 4},
{12, 18, 2160, 0, 0, gScenarioBufferData227_ACCUMULATE_16S, 48, gScenarioKernelData227_ACCUMULATE_16S, 4},
{14, 18, 2520, 0, 0, gScenarioBufferData228_ACCUMULATE_16S, 48, gScenarioKernelData228_ACCUMULATE_16S, 4},
{16, 18, 2880, 0, 0, gScenarioBufferData229_ACCUMULATE_16S, 48, gScenarioKernelData229_ACCUMULATE_16S, 4},
{18, 18, 3240, 0, 0, gScenarioBufferData230_ACCUMULATE_16S, 48, gScenarioKernelData230_ACCUMULATE_16S, 4},
{20, 18, 3600, 0, 0, gScenarioBufferData231_ACCUMULATE_16S, 48, gScenarioKernelData231_ACCUMULATE_16S, 4},
{2, 20, 400, 0, 0, gScenarioBufferData232_ACCUMULATE_16S, 48, gScenarioKernelData232_ACCUMULATE_16S, 4},
{4, 20, 800, 0, 0, gScenarioBufferData233_ACCUMULATE_16S, 48, gScenarioKernelData233_ACCUMULATE_16S, 4},
{6, 20, 1200, 0, 0, gScenarioBufferData234_ACCUMULATE_16S, 48, gScenarioKernelData234_ACCUMULATE_16S, 4},
{8, 20, 1600, 0, 0, gScenarioBufferData235_ACCUMULATE_16S, 48, gScenarioKernelData235_ACCUMULATE_16S, 4},
{10, 20, 2000, 0, 0, gScenarioBufferData236_ACCUMULATE_16S, 48, gScenarioKernelData236_ACCUMULATE_16S, 4},
{12, 20, 2400, 0, 0, gScenarioBufferData237_ACCUMULATE_16S, 48, gScenarioKernelData237_ACCUMULATE_16S, 4},
{14, 20, 2800, 0, 0, gScenarioBufferData238_ACCUMULATE_16S, 48, gScenarioKernelData238_ACCUMULATE_16S, 4},
{16, 20, 3200, 0, 0, gScenarioBufferData239_ACCUMULATE_16S, 48, gScenarioKernelData239_ACCUMULATE_16S, 4},
{18, 20, 3600, 0, 0, gScenarioBufferData240_ACCUMULATE_16S, 48, gScenarioKernelData240_ACCUMULATE_16S, 4},
{2, 22, 440, 0, 0, gScenarioBufferData241_ACCUMULATE_16S, 48, gScenarioKernelData241_ACCUMULATE_16S, 4},
{4, 22, 880, 0, 0, gScenarioBufferData242_ACCUMULATE_16S, 48, gScenarioKernelData242_ACCUMULATE_16S, 4},
{6, 22, 1320, 0, 0, gScenarioBufferData243_ACCUMULATE_16S, 48, gScenarioKernelData243_ACCUMULATE_16S, 4},
{8, 22, 1760, 0, 0, gScenarioBufferData244_ACCUMULATE_16S, 48, gScenarioKernelData244_ACCUMULATE_16S, 4},
{10, 22, 2200, 0, 0, gScenarioBufferData245_ACCUMULATE_16S, 48, gScenarioKernelData245_ACCUMULATE_16S, 4},
{12, 22, 2640, 0, 0, gScenarioBufferData246_ACCUMULATE_16S, 48, gScenarioKernelData246_ACCUMULATE_16S, 4},
{14, 22, 3080, 0, 0, gScenarioBufferData247_ACCUMULATE_16S, 48, gScenarioKernelData247_ACCUMULATE_16S, 4},
{16, 22, 3520, 0, 0, gScenarioBufferData248_ACCUMULATE_16S, 48, gScenarioKernelData248_ACCUMULATE_16S, 4},
{2, 24, 480, 0, 0, gScenarioBufferData249_ACCUMULATE_16S, 48, gScenarioKernelData249_ACCUMULATE_16S, 4},
{4, 24, 960, 0, 0, gScenarioBufferData250_ACCUMULATE_16S, 48, gScenarioKernelData250_ACCUMULATE_16S, 4},
{6, 24, 1440, 0, 0, gScenarioBufferData251_ACCUMULATE_16S, 48, gScenarioKernelData251_ACCUMULATE_16S, 4},
{8, 24, 1920, 0, 0, gScenarioBufferData252_ACCUMULATE_16S, 48, gScenarioKernelData252_ACCUMULATE_16S, 4},
{10, 24, 2400, 0, 0, gScenarioBufferData253_ACCUMULATE_16S, 48, gScenarioKernelData253_ACCUMULATE_16S, 4},
{12, 24, 2880, 0, 0, gScenarioBufferData254_ACCUMULATE_16S, 48, gScenarioKernelData254_ACCUMULATE_16S, 4},
{14, 24, 3360, 0, 0, gScenarioBufferData255_ACCUMULATE_16S, 48, gScenarioKernelData255_ACCUMULATE_16S, 4},
{16, 24, 3840, 0, 0, gScenarioBufferData256_ACCUMULATE_16S, 48, gScenarioKernelData256_ACCUMULATE_16S, 4},
{2, 26, 520, 0, 0, gScenarioBufferData257_ACCUMULATE_16S, 48, gScenarioKernelData257_ACCUMULATE_16S, 4},
{4, 26, 1040, 0, 0, gScenarioBufferData258_ACCUMULATE_16S, 48, gScenarioKernelData258_ACCUMULATE_16S, 4},
{6, 26, 1560, 0, 0, gScenarioBufferData259_ACCUMULATE_16S, 48, gScenarioKernelData259_ACCUMULATE_16S, 4},
{8, 26, 2080, 0, 0, gScenarioBufferData260_ACCUMULATE_16S, 48, gScenarioKernelData260_ACCUMULATE_16S, 4},
{10, 26, 2600, 0, 0, gScenarioBufferData261_ACCUMULATE_16S, 48, gScenarioKernelData261_ACCUMULATE_16S, 4},
{12, 26, 3120, 0, 0, gScenarioBufferData262_ACCUMULATE_16S, 48, gScenarioKernelData262_ACCUMULATE_16S, 4},
{14, 26, 3640, 0, 0, gScenarioBufferData263_ACCUMULATE_16S, 48, gScenarioKernelData263_ACCUMULATE_16S, 4},
{2, 28, 560, 0, 0, gScenarioBufferData264_ACCUMULATE_16S, 48, gScenarioKernelData264_ACCUMULATE_16S, 4},
{4, 28, 1120, 0, 0, gScenarioBufferData265_ACCUMULATE_16S, 48, gScenarioKernelData265_ACCUMULATE_16S, 4},
{6, 28, 1680, 0, 0, gScenarioBufferData266_ACCUMULATE_16S, 48, gScenarioKernelData266_ACCUMULATE_16S, 4},
{8, 28, 2240, 0, 0, gScenarioBufferData267_ACCUMULATE_16S, 48, gScenarioKernelData267_ACCUMULATE_16S, 4},
{10, 28, 2800, 0, 0, gScenarioBufferData268_ACCUMULATE_16S, 48, gScenarioKernelData268_ACCUMULATE_16S, 4},
{12, 28, 3360, 0, 0, gScenarioBufferData269_ACCUMULATE_16S, 48, gScenarioKernelData269_ACCUMULATE_16S, 4},
{2, 30, 600, 0, 0, gScenarioBufferData270_ACCUMULATE_16S, 48, gScenarioKernelData270_ACCUMULATE_16S, 4},
{4, 30, 1200, 0, 0, gScenarioBufferData271_ACCUMULATE_16S, 48, gScenarioKernelData271_ACCUMULATE_16S, 4},
{6, 30, 1800, 0, 0, gScenarioBufferData272_ACCUMULATE_16S, 48, gScenarioKernelData272_ACCUMULATE_16S, 4},
{8, 30, 2400, 0, 0, gScenarioBufferData273_ACCUMULATE_16S, 48, gScenarioKernelData273_ACCUMULATE_16S, 4},
{10, 30, 3000, 0, 0, gScenarioBufferData274_ACCUMULATE_16S, 48, gScenarioKernelData274_ACCUMULATE_16S, 4},
{12, 30, 3600, 0, 0, gScenarioBufferData275_ACCUMULATE_16S, 48, gScenarioKernelData275_ACCUMULATE_16S, 4},
{2, 32, 640, 0, 0, gScenarioBufferData276_ACCUMULATE_16S, 48, gScenarioKernelData276_ACCUMULATE_16S, 4},
{4, 32, 1280, 0, 0, gScenarioBufferData277_ACCUMULATE_16S, 48, gScenarioKernelData277_ACCUMULATE_16S, 4},
{6, 32, 1920, 0, 0, gScenarioBufferData278_ACCUMULATE_16S, 48, gScenarioKernelData278_ACCUMULATE_16S, 4},
{8, 32, 2560, 0, 0, gScenarioBufferData279_ACCUMULATE_16S, 48, gScenarioKernelData279_ACCUMULATE_16S, 4},
{10, 32, 3200, 0, 0, gScenarioBufferData280_ACCUMULATE_16S, 48, gScenarioKernelData280_ACCUMULATE_16S, 4},
{12, 32, 3840, 0, 0, gScenarioBufferData281_ACCUMULATE_16S, 48, gScenarioKernelData281_ACCUMULATE_16S, 4}
};

static acf_scenario_list gScenarioList_ACCUMULATE_16S = {
282, //number of scenarios
gScenarioArray_ACCUMULATE_16S};
//**************************************************************

class ACCUMULATE_16S : public ACF_Process_APU
{

public:
   ACCUMULATE_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("ACCUMULATE_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("ACCUMULATE_16S",
                                        ACCUMULATE_16S_LOAD_SEGMENTS,
                                        ACCUMULATE_16S_LOAD_PMEM, ACCUMULATE_16S_LOAD_PMEM_SIZE,
                                        ACCUMULATE_16S_LOAD_DMEM, ACCUMULATE_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(ACCUMULATE_16S_APEX_LOG_BUFFER, ACCUMULATE_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_16S, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_ACCUMULATE_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_ACCUMULATE_16S

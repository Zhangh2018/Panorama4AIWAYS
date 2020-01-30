#ifndef _ACF_PROCESS_APU_DOWN_CONVERT_16S
#define _ACF_PROCESS_APU_DOWN_CONVERT_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <DOWN_CONVERT_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_DOWN_CONVERT_16S[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_DOWN_CONVERT_16S[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_DOWN_CONVERT_16S[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_DOWN_CONVERT_16S[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_DOWN_CONVERT_16S[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_DOWN_CONVERT_16S[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_DOWN_CONVERT_16S[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_DOWN_CONVERT_16S[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_DOWN_CONVERT_16S[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_DOWN_CONVERT_16S[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_DOWN_CONVERT_16S[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_DOWN_CONVERT_16S[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_DOWN_CONVERT_16S[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_DOWN_CONVERT_16S[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_DOWN_CONVERT_16S[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_DOWN_CONVERT_16S[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_DOWN_CONVERT_16S[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_DOWN_CONVERT_16S[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_DOWN_CONVERT_16S[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_DOWN_CONVERT_16S[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_DOWN_CONVERT_16S[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_DOWN_CONVERT_16S[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_DOWN_CONVERT_16S[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_DOWN_CONVERT_16S[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_DOWN_CONVERT_16S[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_DOWN_CONVERT_16S[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_DOWN_CONVERT_16S[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_DOWN_CONVERT_16S[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_DOWN_CONVERT_16S[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_DOWN_CONVERT_16S[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_DOWN_CONVERT_16S[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_DOWN_CONVERT_16S[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_DOWN_CONVERT_16S[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_DOWN_CONVERT_16S[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_DOWN_CONVERT_16S[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_DOWN_CONVERT_16S[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_DOWN_CONVERT_16S[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_DOWN_CONVERT_16S[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_DOWN_CONVERT_16S[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_DOWN_CONVERT_16S[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_DOWN_CONVERT_16S[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_DOWN_CONVERT_16S[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_DOWN_CONVERT_16S[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_DOWN_CONVERT_16S[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_DOWN_CONVERT_16S[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_DOWN_CONVERT_16S[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_DOWN_CONVERT_16S[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_DOWN_CONVERT_16S[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_DOWN_CONVERT_16S[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_DOWN_CONVERT_16S[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_DOWN_CONVERT_16S[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_DOWN_CONVERT_16S[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_DOWN_CONVERT_16S[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_DOWN_CONVERT_16S[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_DOWN_CONVERT_16S[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_DOWN_CONVERT_16S[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_DOWN_CONVERT_16S[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_DOWN_CONVERT_16S[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_DOWN_CONVERT_16S[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_DOWN_CONVERT_16S[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_DOWN_CONVERT_16S[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_DOWN_CONVERT_16S[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_DOWN_CONVERT_16S[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_DOWN_CONVERT_16S[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_DOWN_CONVERT_16S[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_DOWN_CONVERT_16S[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_DOWN_CONVERT_16S[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_DOWN_CONVERT_16S[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_DOWN_CONVERT_16S[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_DOWN_CONVERT_16S[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_DOWN_CONVERT_16S[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_DOWN_CONVERT_16S[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_DOWN_CONVERT_16S[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_DOWN_CONVERT_16S[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_DOWN_CONVERT_16S[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_DOWN_CONVERT_16S[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_DOWN_CONVERT_16S[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_DOWN_CONVERT_16S[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_DOWN_CONVERT_16S[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_DOWN_CONVERT_16S[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_DOWN_CONVERT_16S[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_DOWN_CONVERT_16S[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_DOWN_CONVERT_16S[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_DOWN_CONVERT_16S[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_DOWN_CONVERT_16S[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_DOWN_CONVERT_16S[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_DOWN_CONVERT_16S[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_DOWN_CONVERT_16S[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_DOWN_CONVERT_16S[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_DOWN_CONVERT_16S[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_DOWN_CONVERT_16S[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_DOWN_CONVERT_16S[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_DOWN_CONVERT_16S[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_DOWN_CONVERT_16S[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_DOWN_CONVERT_16S[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_DOWN_CONVERT_16S[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_DOWN_CONVERT_16S[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_DOWN_CONVERT_16S[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_DOWN_CONVERT_16S[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_DOWN_CONVERT_16S[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_DOWN_CONVERT_16S[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_DOWN_CONVERT_16S[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_DOWN_CONVERT_16S[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_DOWN_CONVERT_16S[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_DOWN_CONVERT_16S[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_DOWN_CONVERT_16S[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_DOWN_CONVERT_16S[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_DOWN_CONVERT_16S[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_DOWN_CONVERT_16S[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_DOWN_CONVERT_16S[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_DOWN_CONVERT_16S[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_DOWN_CONVERT_16S[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_DOWN_CONVERT_16S[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_DOWN_CONVERT_16S[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_DOWN_CONVERT_16S[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_DOWN_CONVERT_16S[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_DOWN_CONVERT_16S[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_DOWN_CONVERT_16S[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_DOWN_CONVERT_16S[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_DOWN_CONVERT_16S[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_DOWN_CONVERT_16S[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_DOWN_CONVERT_16S[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_DOWN_CONVERT_16S[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_DOWN_CONVERT_16S[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_DOWN_CONVERT_16S[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_DOWN_CONVERT_16S[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_DOWN_CONVERT_16S[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_DOWN_CONVERT_16S[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_DOWN_CONVERT_16S[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_DOWN_CONVERT_16S[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_DOWN_CONVERT_16S[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_DOWN_CONVERT_16S[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_DOWN_CONVERT_16S[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_DOWN_CONVERT_16S[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_DOWN_CONVERT_16S[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_DOWN_CONVERT_16S[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_DOWN_CONVERT_16S[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_DOWN_CONVERT_16S[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_DOWN_CONVERT_16S[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_DOWN_CONVERT_16S[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_DOWN_CONVERT_16S[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_DOWN_CONVERT_16S[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_DOWN_CONVERT_16S[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_DOWN_CONVERT_16S[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_DOWN_CONVERT_16S[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_DOWN_CONVERT_16S[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_DOWN_CONVERT_16S[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_DOWN_CONVERT_16S[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_DOWN_CONVERT_16S[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_DOWN_CONVERT_16S[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_DOWN_CONVERT_16S[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_DOWN_CONVERT_16S[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_DOWN_CONVERT_16S[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_DOWN_CONVERT_16S[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_DOWN_CONVERT_16S[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_DOWN_CONVERT_16S[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_DOWN_CONVERT_16S[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_DOWN_CONVERT_16S[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_DOWN_CONVERT_16S[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_DOWN_CONVERT_16S[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_DOWN_CONVERT_16S[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_DOWN_CONVERT_16S[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_DOWN_CONVERT_16S[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_DOWN_CONVERT_16S[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_DOWN_CONVERT_16S[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_DOWN_CONVERT_16S[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_DOWN_CONVERT_16S[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_DOWN_CONVERT_16S[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_DOWN_CONVERT_16S[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_DOWN_CONVERT_16S[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_DOWN_CONVERT_16S[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_DOWN_CONVERT_16S[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_DOWN_CONVERT_16S[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_DOWN_CONVERT_16S[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_DOWN_CONVERT_16S[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_DOWN_CONVERT_16S[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_DOWN_CONVERT_16S[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_DOWN_CONVERT_16S[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_DOWN_CONVERT_16S[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_DOWN_CONVERT_16S[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_DOWN_CONVERT_16S[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_DOWN_CONVERT_16S[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_DOWN_CONVERT_16S[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_DOWN_CONVERT_16S[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_DOWN_CONVERT_16S[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_DOWN_CONVERT_16S[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_DOWN_CONVERT_16S[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_DOWN_CONVERT_16S[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_DOWN_CONVERT_16S[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_DOWN_CONVERT_16S[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_DOWN_CONVERT_16S[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_DOWN_CONVERT_16S[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_DOWN_CONVERT_16S[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_DOWN_CONVERT_16S[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_DOWN_CONVERT_16S[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_DOWN_CONVERT_16S[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_DOWN_CONVERT_16S[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_DOWN_CONVERT_16S[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_DOWN_CONVERT_16S[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_DOWN_CONVERT_16S[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_DOWN_CONVERT_16S[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_DOWN_CONVERT_16S[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_DOWN_CONVERT_16S[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_DOWN_CONVERT_16S[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_DOWN_CONVERT_16S[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_DOWN_CONVERT_16S[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_DOWN_CONVERT_16S[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_DOWN_CONVERT_16S[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_DOWN_CONVERT_16S[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_DOWN_CONVERT_16S[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_DOWN_CONVERT_16S[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_DOWN_CONVERT_16S[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_DOWN_CONVERT_16S[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_DOWN_CONVERT_16S[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_DOWN_CONVERT_16S[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_DOWN_CONVERT_16S[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_DOWN_CONVERT_16S[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_DOWN_CONVERT_16S[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_DOWN_CONVERT_16S[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_DOWN_CONVERT_16S[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_DOWN_CONVERT_16S[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_DOWN_CONVERT_16S[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_DOWN_CONVERT_16S[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_DOWN_CONVERT_16S[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_DOWN_CONVERT_16S[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_DOWN_CONVERT_16S[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_DOWN_CONVERT_16S[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_DOWN_CONVERT_16S[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_DOWN_CONVERT_16S[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_DOWN_CONVERT_16S[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_DOWN_CONVERT_16S[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_DOWN_CONVERT_16S[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_DOWN_CONVERT_16S[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_DOWN_CONVERT_16S[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_DOWN_CONVERT_16S[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_DOWN_CONVERT_16S[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_DOWN_CONVERT_16S[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_DOWN_CONVERT_16S[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_DOWN_CONVERT_16S[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_DOWN_CONVERT_16S[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_DOWN_CONVERT_16S[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_DOWN_CONVERT_16S[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_DOWN_CONVERT_16S[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_DOWN_CONVERT_16S[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_DOWN_CONVERT_16S[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_DOWN_CONVERT_16S[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_DOWN_CONVERT_16S[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_DOWN_CONVERT_16S[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_DOWN_CONVERT_16S[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_DOWN_CONVERT_16S[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_DOWN_CONVERT_16S[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_DOWN_CONVERT_16S[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_DOWN_CONVERT_16S[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_DOWN_CONVERT_16S[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_DOWN_CONVERT_16S[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_DOWN_CONVERT_16S[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_DOWN_CONVERT_16S[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_DOWN_CONVERT_16S[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_DOWN_CONVERT_16S[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_DOWN_CONVERT_16S[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_DOWN_CONVERT_16S[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_DOWN_CONVERT_16S[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_DOWN_CONVERT_16S[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_DOWN_CONVERT_16S[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_DOWN_CONVERT_16S[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_DOWN_CONVERT_16S[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_DOWN_CONVERT_16S[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_DOWN_CONVERT_16S[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_DOWN_CONVERT_16S[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_DOWN_CONVERT_16S[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_DOWN_CONVERT_16S[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_DOWN_CONVERT_16S[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_DOWN_CONVERT_16S[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_DOWN_CONVERT_16S[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_DOWN_CONVERT_16S[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_DOWN_CONVERT_16S[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_DOWN_CONVERT_16S[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_DOWN_CONVERT_16S[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_DOWN_CONVERT_16S[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_DOWN_CONVERT_16S[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_DOWN_CONVERT_16S[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_DOWN_CONVERT_16S[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_DOWN_CONVERT_16S[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_DOWN_CONVERT_16S[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_DOWN_CONVERT_16S[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_DOWN_CONVERT_16S[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_DOWN_CONVERT_16S[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_DOWN_CONVERT_16S[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_DOWN_CONVERT_16S[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_DOWN_CONVERT_16S[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_DOWN_CONVERT_16S[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_DOWN_CONVERT_16S[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_DOWN_CONVERT_16S[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_DOWN_CONVERT_16S[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_DOWN_CONVERT_16S[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_DOWN_CONVERT_16S[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_DOWN_CONVERT_16S[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_DOWN_CONVERT_16S[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_DOWN_CONVERT_16S[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_DOWN_CONVERT_16S[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_DOWN_CONVERT_16S[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_DOWN_CONVERT_16S[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_DOWN_CONVERT_16S[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_DOWN_CONVERT_16S[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_DOWN_CONVERT_16S[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_DOWN_CONVERT_16S[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_DOWN_CONVERT_16S[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_DOWN_CONVERT_16S[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_DOWN_CONVERT_16S[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_DOWN_CONVERT_16S[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_DOWN_CONVERT_16S[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_DOWN_CONVERT_16S[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_DOWN_CONVERT_16S[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_DOWN_CONVERT_16S[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_DOWN_CONVERT_16S[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_DOWN_CONVERT_16S[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_DOWN_CONVERT_16S[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_DOWN_CONVERT_16S[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_DOWN_CONVERT_16S[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_DOWN_CONVERT_16S[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_DOWN_CONVERT_16S[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_DOWN_CONVERT_16S[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_DOWN_CONVERT_16S[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_DOWN_CONVERT_16S[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_DOWN_CONVERT_16S[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_DOWN_CONVERT_16S[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_DOWN_CONVERT_16S[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_DOWN_CONVERT_16S[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_DOWN_CONVERT_16S[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_DOWN_CONVERT_16S[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_DOWN_CONVERT_16S[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_DOWN_CONVERT_16S[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_DOWN_CONVERT_16S[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_DOWN_CONVERT_16S[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_DOWN_CONVERT_16S[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_DOWN_CONVERT_16S[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_DOWN_CONVERT_16S[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_DOWN_CONVERT_16S[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_DOWN_CONVERT_16S[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_DOWN_CONVERT_16S[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_DOWN_CONVERT_16S[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_DOWN_CONVERT_16S[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_DOWN_CONVERT_16S[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_DOWN_CONVERT_16S[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_DOWN_CONVERT_16S[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_DOWN_CONVERT_16S[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_DOWN_CONVERT_16S[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_DOWN_CONVERT_16S[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_DOWN_CONVERT_16S[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_DOWN_CONVERT_16S[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_DOWN_CONVERT_16S[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_DOWN_CONVERT_16S[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_DOWN_CONVERT_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_DOWN_CONVERT_16S[] = {{0, 0}};

static acf_scenario gScenarioArray_DOWN_CONVERT_16S[] = {
{2, 1, 12, 32, 0, gScenarioBufferData0_DOWN_CONVERT_16S, 64, gScenarioKernelData0_DOWN_CONVERT_16S, 4},
{4, 1, 24, 32, 0, gScenarioBufferData1_DOWN_CONVERT_16S, 64, gScenarioKernelData1_DOWN_CONVERT_16S, 4},
{6, 1, 36, 32, 0, gScenarioBufferData2_DOWN_CONVERT_16S, 64, gScenarioKernelData2_DOWN_CONVERT_16S, 4},
{8, 1, 48, 32, 0, gScenarioBufferData3_DOWN_CONVERT_16S, 64, gScenarioKernelData3_DOWN_CONVERT_16S, 4},
{10, 1, 60, 32, 0, gScenarioBufferData4_DOWN_CONVERT_16S, 64, gScenarioKernelData4_DOWN_CONVERT_16S, 4},
{12, 1, 72, 32, 0, gScenarioBufferData5_DOWN_CONVERT_16S, 64, gScenarioKernelData5_DOWN_CONVERT_16S, 4},
{14, 1, 84, 32, 0, gScenarioBufferData6_DOWN_CONVERT_16S, 64, gScenarioKernelData6_DOWN_CONVERT_16S, 4},
{16, 1, 96, 32, 0, gScenarioBufferData7_DOWN_CONVERT_16S, 64, gScenarioKernelData7_DOWN_CONVERT_16S, 4},
{18, 1, 108, 32, 0, gScenarioBufferData8_DOWN_CONVERT_16S, 64, gScenarioKernelData8_DOWN_CONVERT_16S, 4},
{20, 1, 120, 32, 0, gScenarioBufferData9_DOWN_CONVERT_16S, 64, gScenarioKernelData9_DOWN_CONVERT_16S, 4},
{22, 1, 132, 32, 0, gScenarioBufferData10_DOWN_CONVERT_16S, 64, gScenarioKernelData10_DOWN_CONVERT_16S, 4},
{24, 1, 144, 32, 0, gScenarioBufferData11_DOWN_CONVERT_16S, 64, gScenarioKernelData11_DOWN_CONVERT_16S, 4},
{26, 1, 156, 32, 0, gScenarioBufferData12_DOWN_CONVERT_16S, 64, gScenarioKernelData12_DOWN_CONVERT_16S, 4},
{28, 1, 168, 32, 0, gScenarioBufferData13_DOWN_CONVERT_16S, 64, gScenarioKernelData13_DOWN_CONVERT_16S, 4},
{30, 1, 180, 32, 0, gScenarioBufferData14_DOWN_CONVERT_16S, 64, gScenarioKernelData14_DOWN_CONVERT_16S, 4},
{32, 1, 192, 32, 0, gScenarioBufferData15_DOWN_CONVERT_16S, 64, gScenarioKernelData15_DOWN_CONVERT_16S, 4},
{36, 1, 216, 32, 0, gScenarioBufferData16_DOWN_CONVERT_16S, 64, gScenarioKernelData16_DOWN_CONVERT_16S, 4},
{40, 1, 240, 32, 0, gScenarioBufferData17_DOWN_CONVERT_16S, 64, gScenarioKernelData17_DOWN_CONVERT_16S, 4},
{44, 1, 264, 32, 0, gScenarioBufferData18_DOWN_CONVERT_16S, 64, gScenarioKernelData18_DOWN_CONVERT_16S, 4},
{48, 1, 288, 32, 0, gScenarioBufferData19_DOWN_CONVERT_16S, 64, gScenarioKernelData19_DOWN_CONVERT_16S, 4},
{52, 1, 312, 32, 0, gScenarioBufferData20_DOWN_CONVERT_16S, 64, gScenarioKernelData20_DOWN_CONVERT_16S, 4},
{56, 1, 336, 32, 0, gScenarioBufferData21_DOWN_CONVERT_16S, 64, gScenarioKernelData21_DOWN_CONVERT_16S, 4},
{60, 1, 360, 32, 0, gScenarioBufferData22_DOWN_CONVERT_16S, 64, gScenarioKernelData22_DOWN_CONVERT_16S, 4},
{64, 1, 384, 32, 0, gScenarioBufferData23_DOWN_CONVERT_16S, 64, gScenarioKernelData23_DOWN_CONVERT_16S, 4},
{2, 2, 24, 32, 0, gScenarioBufferData24_DOWN_CONVERT_16S, 64, gScenarioKernelData24_DOWN_CONVERT_16S, 4},
{4, 2, 48, 32, 0, gScenarioBufferData25_DOWN_CONVERT_16S, 64, gScenarioKernelData25_DOWN_CONVERT_16S, 4},
{6, 2, 72, 32, 0, gScenarioBufferData26_DOWN_CONVERT_16S, 64, gScenarioKernelData26_DOWN_CONVERT_16S, 4},
{8, 2, 96, 32, 0, gScenarioBufferData27_DOWN_CONVERT_16S, 64, gScenarioKernelData27_DOWN_CONVERT_16S, 4},
{10, 2, 120, 32, 0, gScenarioBufferData28_DOWN_CONVERT_16S, 64, gScenarioKernelData28_DOWN_CONVERT_16S, 4},
{12, 2, 144, 32, 0, gScenarioBufferData29_DOWN_CONVERT_16S, 64, gScenarioKernelData29_DOWN_CONVERT_16S, 4},
{14, 2, 168, 32, 0, gScenarioBufferData30_DOWN_CONVERT_16S, 64, gScenarioKernelData30_DOWN_CONVERT_16S, 4},
{16, 2, 192, 32, 0, gScenarioBufferData31_DOWN_CONVERT_16S, 64, gScenarioKernelData31_DOWN_CONVERT_16S, 4},
{18, 2, 216, 32, 0, gScenarioBufferData32_DOWN_CONVERT_16S, 64, gScenarioKernelData32_DOWN_CONVERT_16S, 4},
{20, 2, 240, 32, 0, gScenarioBufferData33_DOWN_CONVERT_16S, 64, gScenarioKernelData33_DOWN_CONVERT_16S, 4},
{22, 2, 264, 32, 0, gScenarioBufferData34_DOWN_CONVERT_16S, 64, gScenarioKernelData34_DOWN_CONVERT_16S, 4},
{24, 2, 288, 32, 0, gScenarioBufferData35_DOWN_CONVERT_16S, 64, gScenarioKernelData35_DOWN_CONVERT_16S, 4},
{26, 2, 312, 32, 0, gScenarioBufferData36_DOWN_CONVERT_16S, 64, gScenarioKernelData36_DOWN_CONVERT_16S, 4},
{28, 2, 336, 32, 0, gScenarioBufferData37_DOWN_CONVERT_16S, 64, gScenarioKernelData37_DOWN_CONVERT_16S, 4},
{30, 2, 360, 32, 0, gScenarioBufferData38_DOWN_CONVERT_16S, 64, gScenarioKernelData38_DOWN_CONVERT_16S, 4},
{32, 2, 384, 32, 0, gScenarioBufferData39_DOWN_CONVERT_16S, 64, gScenarioKernelData39_DOWN_CONVERT_16S, 4},
{36, 2, 432, 32, 0, gScenarioBufferData40_DOWN_CONVERT_16S, 64, gScenarioKernelData40_DOWN_CONVERT_16S, 4},
{40, 2, 480, 32, 0, gScenarioBufferData41_DOWN_CONVERT_16S, 64, gScenarioKernelData41_DOWN_CONVERT_16S, 4},
{44, 2, 528, 32, 0, gScenarioBufferData42_DOWN_CONVERT_16S, 64, gScenarioKernelData42_DOWN_CONVERT_16S, 4},
{48, 2, 576, 32, 0, gScenarioBufferData43_DOWN_CONVERT_16S, 64, gScenarioKernelData43_DOWN_CONVERT_16S, 4},
{52, 2, 624, 32, 0, gScenarioBufferData44_DOWN_CONVERT_16S, 64, gScenarioKernelData44_DOWN_CONVERT_16S, 4},
{56, 2, 672, 32, 0, gScenarioBufferData45_DOWN_CONVERT_16S, 64, gScenarioKernelData45_DOWN_CONVERT_16S, 4},
{60, 2, 720, 32, 0, gScenarioBufferData46_DOWN_CONVERT_16S, 64, gScenarioKernelData46_DOWN_CONVERT_16S, 4},
{64, 2, 768, 32, 0, gScenarioBufferData47_DOWN_CONVERT_16S, 64, gScenarioKernelData47_DOWN_CONVERT_16S, 4},
{2, 3, 36, 32, 0, gScenarioBufferData48_DOWN_CONVERT_16S, 64, gScenarioKernelData48_DOWN_CONVERT_16S, 4},
{4, 3, 72, 32, 0, gScenarioBufferData49_DOWN_CONVERT_16S, 64, gScenarioKernelData49_DOWN_CONVERT_16S, 4},
{6, 3, 108, 32, 0, gScenarioBufferData50_DOWN_CONVERT_16S, 64, gScenarioKernelData50_DOWN_CONVERT_16S, 4},
{8, 3, 144, 32, 0, gScenarioBufferData51_DOWN_CONVERT_16S, 64, gScenarioKernelData51_DOWN_CONVERT_16S, 4},
{10, 3, 180, 32, 0, gScenarioBufferData52_DOWN_CONVERT_16S, 64, gScenarioKernelData52_DOWN_CONVERT_16S, 4},
{12, 3, 216, 32, 0, gScenarioBufferData53_DOWN_CONVERT_16S, 64, gScenarioKernelData53_DOWN_CONVERT_16S, 4},
{14, 3, 252, 32, 0, gScenarioBufferData54_DOWN_CONVERT_16S, 64, gScenarioKernelData54_DOWN_CONVERT_16S, 4},
{16, 3, 288, 32, 0, gScenarioBufferData55_DOWN_CONVERT_16S, 64, gScenarioKernelData55_DOWN_CONVERT_16S, 4},
{18, 3, 324, 32, 0, gScenarioBufferData56_DOWN_CONVERT_16S, 64, gScenarioKernelData56_DOWN_CONVERT_16S, 4},
{20, 3, 360, 32, 0, gScenarioBufferData57_DOWN_CONVERT_16S, 64, gScenarioKernelData57_DOWN_CONVERT_16S, 4},
{22, 3, 396, 32, 0, gScenarioBufferData58_DOWN_CONVERT_16S, 64, gScenarioKernelData58_DOWN_CONVERT_16S, 4},
{24, 3, 432, 32, 0, gScenarioBufferData59_DOWN_CONVERT_16S, 64, gScenarioKernelData59_DOWN_CONVERT_16S, 4},
{26, 3, 468, 32, 0, gScenarioBufferData60_DOWN_CONVERT_16S, 64, gScenarioKernelData60_DOWN_CONVERT_16S, 4},
{28, 3, 504, 32, 0, gScenarioBufferData61_DOWN_CONVERT_16S, 64, gScenarioKernelData61_DOWN_CONVERT_16S, 4},
{30, 3, 540, 32, 0, gScenarioBufferData62_DOWN_CONVERT_16S, 64, gScenarioKernelData62_DOWN_CONVERT_16S, 4},
{32, 3, 576, 32, 0, gScenarioBufferData63_DOWN_CONVERT_16S, 64, gScenarioKernelData63_DOWN_CONVERT_16S, 4},
{36, 3, 648, 32, 0, gScenarioBufferData64_DOWN_CONVERT_16S, 64, gScenarioKernelData64_DOWN_CONVERT_16S, 4},
{40, 3, 720, 32, 0, gScenarioBufferData65_DOWN_CONVERT_16S, 64, gScenarioKernelData65_DOWN_CONVERT_16S, 4},
{44, 3, 792, 32, 0, gScenarioBufferData66_DOWN_CONVERT_16S, 64, gScenarioKernelData66_DOWN_CONVERT_16S, 4},
{48, 3, 864, 32, 0, gScenarioBufferData67_DOWN_CONVERT_16S, 64, gScenarioKernelData67_DOWN_CONVERT_16S, 4},
{52, 3, 936, 32, 0, gScenarioBufferData68_DOWN_CONVERT_16S, 64, gScenarioKernelData68_DOWN_CONVERT_16S, 4},
{56, 3, 1008, 32, 0, gScenarioBufferData69_DOWN_CONVERT_16S, 64, gScenarioKernelData69_DOWN_CONVERT_16S, 4},
{60, 3, 1080, 32, 0, gScenarioBufferData70_DOWN_CONVERT_16S, 64, gScenarioKernelData70_DOWN_CONVERT_16S, 4},
{64, 3, 1152, 32, 0, gScenarioBufferData71_DOWN_CONVERT_16S, 64, gScenarioKernelData71_DOWN_CONVERT_16S, 4},
{2, 4, 48, 32, 0, gScenarioBufferData72_DOWN_CONVERT_16S, 64, gScenarioKernelData72_DOWN_CONVERT_16S, 4},
{4, 4, 96, 32, 0, gScenarioBufferData73_DOWN_CONVERT_16S, 64, gScenarioKernelData73_DOWN_CONVERT_16S, 4},
{6, 4, 144, 32, 0, gScenarioBufferData74_DOWN_CONVERT_16S, 64, gScenarioKernelData74_DOWN_CONVERT_16S, 4},
{8, 4, 192, 32, 0, gScenarioBufferData75_DOWN_CONVERT_16S, 64, gScenarioKernelData75_DOWN_CONVERT_16S, 4},
{10, 4, 240, 32, 0, gScenarioBufferData76_DOWN_CONVERT_16S, 64, gScenarioKernelData76_DOWN_CONVERT_16S, 4},
{12, 4, 288, 32, 0, gScenarioBufferData77_DOWN_CONVERT_16S, 64, gScenarioKernelData77_DOWN_CONVERT_16S, 4},
{14, 4, 336, 32, 0, gScenarioBufferData78_DOWN_CONVERT_16S, 64, gScenarioKernelData78_DOWN_CONVERT_16S, 4},
{16, 4, 384, 32, 0, gScenarioBufferData79_DOWN_CONVERT_16S, 64, gScenarioKernelData79_DOWN_CONVERT_16S, 4},
{18, 4, 432, 32, 0, gScenarioBufferData80_DOWN_CONVERT_16S, 64, gScenarioKernelData80_DOWN_CONVERT_16S, 4},
{20, 4, 480, 32, 0, gScenarioBufferData81_DOWN_CONVERT_16S, 64, gScenarioKernelData81_DOWN_CONVERT_16S, 4},
{22, 4, 528, 32, 0, gScenarioBufferData82_DOWN_CONVERT_16S, 64, gScenarioKernelData82_DOWN_CONVERT_16S, 4},
{24, 4, 576, 32, 0, gScenarioBufferData83_DOWN_CONVERT_16S, 64, gScenarioKernelData83_DOWN_CONVERT_16S, 4},
{26, 4, 624, 32, 0, gScenarioBufferData84_DOWN_CONVERT_16S, 64, gScenarioKernelData84_DOWN_CONVERT_16S, 4},
{28, 4, 672, 32, 0, gScenarioBufferData85_DOWN_CONVERT_16S, 64, gScenarioKernelData85_DOWN_CONVERT_16S, 4},
{30, 4, 720, 32, 0, gScenarioBufferData86_DOWN_CONVERT_16S, 64, gScenarioKernelData86_DOWN_CONVERT_16S, 4},
{32, 4, 768, 32, 0, gScenarioBufferData87_DOWN_CONVERT_16S, 64, gScenarioKernelData87_DOWN_CONVERT_16S, 4},
{36, 4, 864, 32, 0, gScenarioBufferData88_DOWN_CONVERT_16S, 64, gScenarioKernelData88_DOWN_CONVERT_16S, 4},
{40, 4, 960, 32, 0, gScenarioBufferData89_DOWN_CONVERT_16S, 64, gScenarioKernelData89_DOWN_CONVERT_16S, 4},
{44, 4, 1056, 32, 0, gScenarioBufferData90_DOWN_CONVERT_16S, 64, gScenarioKernelData90_DOWN_CONVERT_16S, 4},
{48, 4, 1152, 32, 0, gScenarioBufferData91_DOWN_CONVERT_16S, 64, gScenarioKernelData91_DOWN_CONVERT_16S, 4},
{52, 4, 1248, 32, 0, gScenarioBufferData92_DOWN_CONVERT_16S, 64, gScenarioKernelData92_DOWN_CONVERT_16S, 4},
{56, 4, 1344, 32, 0, gScenarioBufferData93_DOWN_CONVERT_16S, 64, gScenarioKernelData93_DOWN_CONVERT_16S, 4},
{60, 4, 1440, 32, 0, gScenarioBufferData94_DOWN_CONVERT_16S, 64, gScenarioKernelData94_DOWN_CONVERT_16S, 4},
{64, 4, 1536, 32, 0, gScenarioBufferData95_DOWN_CONVERT_16S, 64, gScenarioKernelData95_DOWN_CONVERT_16S, 4},
{2, 5, 60, 32, 0, gScenarioBufferData96_DOWN_CONVERT_16S, 64, gScenarioKernelData96_DOWN_CONVERT_16S, 4},
{4, 5, 120, 32, 0, gScenarioBufferData97_DOWN_CONVERT_16S, 64, gScenarioKernelData97_DOWN_CONVERT_16S, 4},
{6, 5, 180, 32, 0, gScenarioBufferData98_DOWN_CONVERT_16S, 64, gScenarioKernelData98_DOWN_CONVERT_16S, 4},
{8, 5, 240, 32, 0, gScenarioBufferData99_DOWN_CONVERT_16S, 64, gScenarioKernelData99_DOWN_CONVERT_16S, 4},
{10, 5, 300, 32, 0, gScenarioBufferData100_DOWN_CONVERT_16S, 64, gScenarioKernelData100_DOWN_CONVERT_16S, 4},
{12, 5, 360, 32, 0, gScenarioBufferData101_DOWN_CONVERT_16S, 64, gScenarioKernelData101_DOWN_CONVERT_16S, 4},
{14, 5, 420, 32, 0, gScenarioBufferData102_DOWN_CONVERT_16S, 64, gScenarioKernelData102_DOWN_CONVERT_16S, 4},
{16, 5, 480, 32, 0, gScenarioBufferData103_DOWN_CONVERT_16S, 64, gScenarioKernelData103_DOWN_CONVERT_16S, 4},
{18, 5, 540, 32, 0, gScenarioBufferData104_DOWN_CONVERT_16S, 64, gScenarioKernelData104_DOWN_CONVERT_16S, 4},
{20, 5, 600, 32, 0, gScenarioBufferData105_DOWN_CONVERT_16S, 64, gScenarioKernelData105_DOWN_CONVERT_16S, 4},
{22, 5, 660, 32, 0, gScenarioBufferData106_DOWN_CONVERT_16S, 64, gScenarioKernelData106_DOWN_CONVERT_16S, 4},
{24, 5, 720, 32, 0, gScenarioBufferData107_DOWN_CONVERT_16S, 64, gScenarioKernelData107_DOWN_CONVERT_16S, 4},
{26, 5, 780, 32, 0, gScenarioBufferData108_DOWN_CONVERT_16S, 64, gScenarioKernelData108_DOWN_CONVERT_16S, 4},
{28, 5, 840, 32, 0, gScenarioBufferData109_DOWN_CONVERT_16S, 64, gScenarioKernelData109_DOWN_CONVERT_16S, 4},
{30, 5, 900, 32, 0, gScenarioBufferData110_DOWN_CONVERT_16S, 64, gScenarioKernelData110_DOWN_CONVERT_16S, 4},
{32, 5, 960, 32, 0, gScenarioBufferData111_DOWN_CONVERT_16S, 64, gScenarioKernelData111_DOWN_CONVERT_16S, 4},
{36, 5, 1080, 32, 0, gScenarioBufferData112_DOWN_CONVERT_16S, 64, gScenarioKernelData112_DOWN_CONVERT_16S, 4},
{40, 5, 1200, 32, 0, gScenarioBufferData113_DOWN_CONVERT_16S, 64, gScenarioKernelData113_DOWN_CONVERT_16S, 4},
{44, 5, 1320, 32, 0, gScenarioBufferData114_DOWN_CONVERT_16S, 64, gScenarioKernelData114_DOWN_CONVERT_16S, 4},
{48, 5, 1440, 32, 0, gScenarioBufferData115_DOWN_CONVERT_16S, 64, gScenarioKernelData115_DOWN_CONVERT_16S, 4},
{52, 5, 1560, 32, 0, gScenarioBufferData116_DOWN_CONVERT_16S, 64, gScenarioKernelData116_DOWN_CONVERT_16S, 4},
{56, 5, 1680, 32, 0, gScenarioBufferData117_DOWN_CONVERT_16S, 64, gScenarioKernelData117_DOWN_CONVERT_16S, 4},
{60, 5, 1800, 32, 0, gScenarioBufferData118_DOWN_CONVERT_16S, 64, gScenarioKernelData118_DOWN_CONVERT_16S, 4},
{64, 5, 1920, 32, 0, gScenarioBufferData119_DOWN_CONVERT_16S, 64, gScenarioKernelData119_DOWN_CONVERT_16S, 4},
{2, 6, 72, 32, 0, gScenarioBufferData120_DOWN_CONVERT_16S, 64, gScenarioKernelData120_DOWN_CONVERT_16S, 4},
{4, 6, 144, 32, 0, gScenarioBufferData121_DOWN_CONVERT_16S, 64, gScenarioKernelData121_DOWN_CONVERT_16S, 4},
{6, 6, 216, 32, 0, gScenarioBufferData122_DOWN_CONVERT_16S, 64, gScenarioKernelData122_DOWN_CONVERT_16S, 4},
{8, 6, 288, 32, 0, gScenarioBufferData123_DOWN_CONVERT_16S, 64, gScenarioKernelData123_DOWN_CONVERT_16S, 4},
{10, 6, 360, 32, 0, gScenarioBufferData124_DOWN_CONVERT_16S, 64, gScenarioKernelData124_DOWN_CONVERT_16S, 4},
{12, 6, 432, 32, 0, gScenarioBufferData125_DOWN_CONVERT_16S, 64, gScenarioKernelData125_DOWN_CONVERT_16S, 4},
{14, 6, 504, 32, 0, gScenarioBufferData126_DOWN_CONVERT_16S, 64, gScenarioKernelData126_DOWN_CONVERT_16S, 4},
{16, 6, 576, 32, 0, gScenarioBufferData127_DOWN_CONVERT_16S, 64, gScenarioKernelData127_DOWN_CONVERT_16S, 4},
{18, 6, 648, 32, 0, gScenarioBufferData128_DOWN_CONVERT_16S, 64, gScenarioKernelData128_DOWN_CONVERT_16S, 4},
{20, 6, 720, 32, 0, gScenarioBufferData129_DOWN_CONVERT_16S, 64, gScenarioKernelData129_DOWN_CONVERT_16S, 4},
{22, 6, 792, 32, 0, gScenarioBufferData130_DOWN_CONVERT_16S, 64, gScenarioKernelData130_DOWN_CONVERT_16S, 4},
{24, 6, 864, 32, 0, gScenarioBufferData131_DOWN_CONVERT_16S, 64, gScenarioKernelData131_DOWN_CONVERT_16S, 4},
{26, 6, 936, 32, 0, gScenarioBufferData132_DOWN_CONVERT_16S, 64, gScenarioKernelData132_DOWN_CONVERT_16S, 4},
{28, 6, 1008, 32, 0, gScenarioBufferData133_DOWN_CONVERT_16S, 64, gScenarioKernelData133_DOWN_CONVERT_16S, 4},
{30, 6, 1080, 32, 0, gScenarioBufferData134_DOWN_CONVERT_16S, 64, gScenarioKernelData134_DOWN_CONVERT_16S, 4},
{32, 6, 1152, 32, 0, gScenarioBufferData135_DOWN_CONVERT_16S, 64, gScenarioKernelData135_DOWN_CONVERT_16S, 4},
{36, 6, 1296, 32, 0, gScenarioBufferData136_DOWN_CONVERT_16S, 64, gScenarioKernelData136_DOWN_CONVERT_16S, 4},
{40, 6, 1440, 32, 0, gScenarioBufferData137_DOWN_CONVERT_16S, 64, gScenarioKernelData137_DOWN_CONVERT_16S, 4},
{44, 6, 1584, 32, 0, gScenarioBufferData138_DOWN_CONVERT_16S, 64, gScenarioKernelData138_DOWN_CONVERT_16S, 4},
{48, 6, 1728, 32, 0, gScenarioBufferData139_DOWN_CONVERT_16S, 64, gScenarioKernelData139_DOWN_CONVERT_16S, 4},
{52, 6, 1872, 32, 0, gScenarioBufferData140_DOWN_CONVERT_16S, 64, gScenarioKernelData140_DOWN_CONVERT_16S, 4},
{56, 6, 2016, 32, 0, gScenarioBufferData141_DOWN_CONVERT_16S, 64, gScenarioKernelData141_DOWN_CONVERT_16S, 4},
{60, 6, 2160, 32, 0, gScenarioBufferData142_DOWN_CONVERT_16S, 64, gScenarioKernelData142_DOWN_CONVERT_16S, 4},
{64, 6, 2304, 32, 0, gScenarioBufferData143_DOWN_CONVERT_16S, 64, gScenarioKernelData143_DOWN_CONVERT_16S, 4},
{2, 8, 96, 32, 0, gScenarioBufferData144_DOWN_CONVERT_16S, 64, gScenarioKernelData144_DOWN_CONVERT_16S, 4},
{4, 8, 192, 32, 0, gScenarioBufferData145_DOWN_CONVERT_16S, 64, gScenarioKernelData145_DOWN_CONVERT_16S, 4},
{6, 8, 288, 32, 0, gScenarioBufferData146_DOWN_CONVERT_16S, 64, gScenarioKernelData146_DOWN_CONVERT_16S, 4},
{8, 8, 384, 32, 0, gScenarioBufferData147_DOWN_CONVERT_16S, 64, gScenarioKernelData147_DOWN_CONVERT_16S, 4},
{10, 8, 480, 32, 0, gScenarioBufferData148_DOWN_CONVERT_16S, 64, gScenarioKernelData148_DOWN_CONVERT_16S, 4},
{12, 8, 576, 32, 0, gScenarioBufferData149_DOWN_CONVERT_16S, 64, gScenarioKernelData149_DOWN_CONVERT_16S, 4},
{14, 8, 672, 32, 0, gScenarioBufferData150_DOWN_CONVERT_16S, 64, gScenarioKernelData150_DOWN_CONVERT_16S, 4},
{16, 8, 768, 32, 0, gScenarioBufferData151_DOWN_CONVERT_16S, 64, gScenarioKernelData151_DOWN_CONVERT_16S, 4},
{18, 8, 864, 32, 0, gScenarioBufferData152_DOWN_CONVERT_16S, 64, gScenarioKernelData152_DOWN_CONVERT_16S, 4},
{20, 8, 960, 32, 0, gScenarioBufferData153_DOWN_CONVERT_16S, 64, gScenarioKernelData153_DOWN_CONVERT_16S, 4},
{22, 8, 1056, 32, 0, gScenarioBufferData154_DOWN_CONVERT_16S, 64, gScenarioKernelData154_DOWN_CONVERT_16S, 4},
{24, 8, 1152, 32, 0, gScenarioBufferData155_DOWN_CONVERT_16S, 64, gScenarioKernelData155_DOWN_CONVERT_16S, 4},
{26, 8, 1248, 32, 0, gScenarioBufferData156_DOWN_CONVERT_16S, 64, gScenarioKernelData156_DOWN_CONVERT_16S, 4},
{28, 8, 1344, 32, 0, gScenarioBufferData157_DOWN_CONVERT_16S, 64, gScenarioKernelData157_DOWN_CONVERT_16S, 4},
{30, 8, 1440, 32, 0, gScenarioBufferData158_DOWN_CONVERT_16S, 64, gScenarioKernelData158_DOWN_CONVERT_16S, 4},
{32, 8, 1536, 32, 0, gScenarioBufferData159_DOWN_CONVERT_16S, 64, gScenarioKernelData159_DOWN_CONVERT_16S, 4},
{36, 8, 1728, 32, 0, gScenarioBufferData160_DOWN_CONVERT_16S, 64, gScenarioKernelData160_DOWN_CONVERT_16S, 4},
{40, 8, 1920, 32, 0, gScenarioBufferData161_DOWN_CONVERT_16S, 64, gScenarioKernelData161_DOWN_CONVERT_16S, 4},
{44, 8, 2112, 32, 0, gScenarioBufferData162_DOWN_CONVERT_16S, 64, gScenarioKernelData162_DOWN_CONVERT_16S, 4},
{48, 8, 2304, 32, 0, gScenarioBufferData163_DOWN_CONVERT_16S, 64, gScenarioKernelData163_DOWN_CONVERT_16S, 4},
{52, 8, 2496, 32, 0, gScenarioBufferData164_DOWN_CONVERT_16S, 64, gScenarioKernelData164_DOWN_CONVERT_16S, 4},
{56, 8, 2688, 32, 0, gScenarioBufferData165_DOWN_CONVERT_16S, 64, gScenarioKernelData165_DOWN_CONVERT_16S, 4},
{60, 8, 2880, 32, 0, gScenarioBufferData166_DOWN_CONVERT_16S, 64, gScenarioKernelData166_DOWN_CONVERT_16S, 4},
{64, 8, 3072, 32, 0, gScenarioBufferData167_DOWN_CONVERT_16S, 64, gScenarioKernelData167_DOWN_CONVERT_16S, 4},
{2, 10, 120, 32, 0, gScenarioBufferData168_DOWN_CONVERT_16S, 64, gScenarioKernelData168_DOWN_CONVERT_16S, 4},
{4, 10, 240, 32, 0, gScenarioBufferData169_DOWN_CONVERT_16S, 64, gScenarioKernelData169_DOWN_CONVERT_16S, 4},
{6, 10, 360, 32, 0, gScenarioBufferData170_DOWN_CONVERT_16S, 64, gScenarioKernelData170_DOWN_CONVERT_16S, 4},
{8, 10, 480, 32, 0, gScenarioBufferData171_DOWN_CONVERT_16S, 64, gScenarioKernelData171_DOWN_CONVERT_16S, 4},
{10, 10, 600, 32, 0, gScenarioBufferData172_DOWN_CONVERT_16S, 64, gScenarioKernelData172_DOWN_CONVERT_16S, 4},
{12, 10, 720, 32, 0, gScenarioBufferData173_DOWN_CONVERT_16S, 64, gScenarioKernelData173_DOWN_CONVERT_16S, 4},
{14, 10, 840, 32, 0, gScenarioBufferData174_DOWN_CONVERT_16S, 64, gScenarioKernelData174_DOWN_CONVERT_16S, 4},
{16, 10, 960, 32, 0, gScenarioBufferData175_DOWN_CONVERT_16S, 64, gScenarioKernelData175_DOWN_CONVERT_16S, 4},
{18, 10, 1080, 32, 0, gScenarioBufferData176_DOWN_CONVERT_16S, 64, gScenarioKernelData176_DOWN_CONVERT_16S, 4},
{20, 10, 1200, 32, 0, gScenarioBufferData177_DOWN_CONVERT_16S, 64, gScenarioKernelData177_DOWN_CONVERT_16S, 4},
{22, 10, 1320, 32, 0, gScenarioBufferData178_DOWN_CONVERT_16S, 64, gScenarioKernelData178_DOWN_CONVERT_16S, 4},
{24, 10, 1440, 32, 0, gScenarioBufferData179_DOWN_CONVERT_16S, 64, gScenarioKernelData179_DOWN_CONVERT_16S, 4},
{26, 10, 1560, 32, 0, gScenarioBufferData180_DOWN_CONVERT_16S, 64, gScenarioKernelData180_DOWN_CONVERT_16S, 4},
{28, 10, 1680, 32, 0, gScenarioBufferData181_DOWN_CONVERT_16S, 64, gScenarioKernelData181_DOWN_CONVERT_16S, 4},
{30, 10, 1800, 32, 0, gScenarioBufferData182_DOWN_CONVERT_16S, 64, gScenarioKernelData182_DOWN_CONVERT_16S, 4},
{32, 10, 1920, 32, 0, gScenarioBufferData183_DOWN_CONVERT_16S, 64, gScenarioKernelData183_DOWN_CONVERT_16S, 4},
{36, 10, 2160, 32, 0, gScenarioBufferData184_DOWN_CONVERT_16S, 64, gScenarioKernelData184_DOWN_CONVERT_16S, 4},
{40, 10, 2400, 32, 0, gScenarioBufferData185_DOWN_CONVERT_16S, 64, gScenarioKernelData185_DOWN_CONVERT_16S, 4},
{44, 10, 2640, 32, 0, gScenarioBufferData186_DOWN_CONVERT_16S, 64, gScenarioKernelData186_DOWN_CONVERT_16S, 4},
{48, 10, 2880, 32, 0, gScenarioBufferData187_DOWN_CONVERT_16S, 64, gScenarioKernelData187_DOWN_CONVERT_16S, 4},
{52, 10, 3120, 32, 0, gScenarioBufferData188_DOWN_CONVERT_16S, 64, gScenarioKernelData188_DOWN_CONVERT_16S, 4},
{56, 10, 3360, 32, 0, gScenarioBufferData189_DOWN_CONVERT_16S, 64, gScenarioKernelData189_DOWN_CONVERT_16S, 4},
{60, 10, 3600, 32, 0, gScenarioBufferData190_DOWN_CONVERT_16S, 64, gScenarioKernelData190_DOWN_CONVERT_16S, 4},
{64, 10, 3840, 32, 0, gScenarioBufferData191_DOWN_CONVERT_16S, 64, gScenarioKernelData191_DOWN_CONVERT_16S, 4},
{2, 12, 144, 32, 0, gScenarioBufferData192_DOWN_CONVERT_16S, 64, gScenarioKernelData192_DOWN_CONVERT_16S, 4},
{4, 12, 288, 32, 0, gScenarioBufferData193_DOWN_CONVERT_16S, 64, gScenarioKernelData193_DOWN_CONVERT_16S, 4},
{6, 12, 432, 32, 0, gScenarioBufferData194_DOWN_CONVERT_16S, 64, gScenarioKernelData194_DOWN_CONVERT_16S, 4},
{8, 12, 576, 32, 0, gScenarioBufferData195_DOWN_CONVERT_16S, 64, gScenarioKernelData195_DOWN_CONVERT_16S, 4},
{10, 12, 720, 32, 0, gScenarioBufferData196_DOWN_CONVERT_16S, 64, gScenarioKernelData196_DOWN_CONVERT_16S, 4},
{12, 12, 864, 32, 0, gScenarioBufferData197_DOWN_CONVERT_16S, 64, gScenarioKernelData197_DOWN_CONVERT_16S, 4},
{14, 12, 1008, 32, 0, gScenarioBufferData198_DOWN_CONVERT_16S, 64, gScenarioKernelData198_DOWN_CONVERT_16S, 4},
{16, 12, 1152, 32, 0, gScenarioBufferData199_DOWN_CONVERT_16S, 64, gScenarioKernelData199_DOWN_CONVERT_16S, 4},
{18, 12, 1296, 32, 0, gScenarioBufferData200_DOWN_CONVERT_16S, 64, gScenarioKernelData200_DOWN_CONVERT_16S, 4},
{20, 12, 1440, 32, 0, gScenarioBufferData201_DOWN_CONVERT_16S, 64, gScenarioKernelData201_DOWN_CONVERT_16S, 4},
{22, 12, 1584, 32, 0, gScenarioBufferData202_DOWN_CONVERT_16S, 64, gScenarioKernelData202_DOWN_CONVERT_16S, 4},
{24, 12, 1728, 32, 0, gScenarioBufferData203_DOWN_CONVERT_16S, 64, gScenarioKernelData203_DOWN_CONVERT_16S, 4},
{26, 12, 1872, 32, 0, gScenarioBufferData204_DOWN_CONVERT_16S, 64, gScenarioKernelData204_DOWN_CONVERT_16S, 4},
{28, 12, 2016, 32, 0, gScenarioBufferData205_DOWN_CONVERT_16S, 64, gScenarioKernelData205_DOWN_CONVERT_16S, 4},
{30, 12, 2160, 32, 0, gScenarioBufferData206_DOWN_CONVERT_16S, 64, gScenarioKernelData206_DOWN_CONVERT_16S, 4},
{32, 12, 2304, 32, 0, gScenarioBufferData207_DOWN_CONVERT_16S, 64, gScenarioKernelData207_DOWN_CONVERT_16S, 4},
{36, 12, 2592, 32, 0, gScenarioBufferData208_DOWN_CONVERT_16S, 64, gScenarioKernelData208_DOWN_CONVERT_16S, 4},
{40, 12, 2880, 32, 0, gScenarioBufferData209_DOWN_CONVERT_16S, 64, gScenarioKernelData209_DOWN_CONVERT_16S, 4},
{44, 12, 3168, 32, 0, gScenarioBufferData210_DOWN_CONVERT_16S, 64, gScenarioKernelData210_DOWN_CONVERT_16S, 4},
{48, 12, 3456, 32, 0, gScenarioBufferData211_DOWN_CONVERT_16S, 64, gScenarioKernelData211_DOWN_CONVERT_16S, 4},
{52, 12, 3744, 32, 0, gScenarioBufferData212_DOWN_CONVERT_16S, 64, gScenarioKernelData212_DOWN_CONVERT_16S, 4},
{2, 14, 168, 32, 0, gScenarioBufferData213_DOWN_CONVERT_16S, 64, gScenarioKernelData213_DOWN_CONVERT_16S, 4},
{4, 14, 336, 32, 0, gScenarioBufferData214_DOWN_CONVERT_16S, 64, gScenarioKernelData214_DOWN_CONVERT_16S, 4},
{6, 14, 504, 32, 0, gScenarioBufferData215_DOWN_CONVERT_16S, 64, gScenarioKernelData215_DOWN_CONVERT_16S, 4},
{8, 14, 672, 32, 0, gScenarioBufferData216_DOWN_CONVERT_16S, 64, gScenarioKernelData216_DOWN_CONVERT_16S, 4},
{10, 14, 840, 32, 0, gScenarioBufferData217_DOWN_CONVERT_16S, 64, gScenarioKernelData217_DOWN_CONVERT_16S, 4},
{12, 14, 1008, 32, 0, gScenarioBufferData218_DOWN_CONVERT_16S, 64, gScenarioKernelData218_DOWN_CONVERT_16S, 4},
{14, 14, 1176, 32, 0, gScenarioBufferData219_DOWN_CONVERT_16S, 64, gScenarioKernelData219_DOWN_CONVERT_16S, 4},
{16, 14, 1344, 32, 0, gScenarioBufferData220_DOWN_CONVERT_16S, 64, gScenarioKernelData220_DOWN_CONVERT_16S, 4},
{18, 14, 1512, 32, 0, gScenarioBufferData221_DOWN_CONVERT_16S, 64, gScenarioKernelData221_DOWN_CONVERT_16S, 4},
{20, 14, 1680, 32, 0, gScenarioBufferData222_DOWN_CONVERT_16S, 64, gScenarioKernelData222_DOWN_CONVERT_16S, 4},
{22, 14, 1848, 32, 0, gScenarioBufferData223_DOWN_CONVERT_16S, 64, gScenarioKernelData223_DOWN_CONVERT_16S, 4},
{24, 14, 2016, 32, 0, gScenarioBufferData224_DOWN_CONVERT_16S, 64, gScenarioKernelData224_DOWN_CONVERT_16S, 4},
{26, 14, 2184, 32, 0, gScenarioBufferData225_DOWN_CONVERT_16S, 64, gScenarioKernelData225_DOWN_CONVERT_16S, 4},
{28, 14, 2352, 32, 0, gScenarioBufferData226_DOWN_CONVERT_16S, 64, gScenarioKernelData226_DOWN_CONVERT_16S, 4},
{30, 14, 2520, 32, 0, gScenarioBufferData227_DOWN_CONVERT_16S, 64, gScenarioKernelData227_DOWN_CONVERT_16S, 4},
{32, 14, 2688, 32, 0, gScenarioBufferData228_DOWN_CONVERT_16S, 64, gScenarioKernelData228_DOWN_CONVERT_16S, 4},
{36, 14, 3024, 32, 0, gScenarioBufferData229_DOWN_CONVERT_16S, 64, gScenarioKernelData229_DOWN_CONVERT_16S, 4},
{40, 14, 3360, 32, 0, gScenarioBufferData230_DOWN_CONVERT_16S, 64, gScenarioKernelData230_DOWN_CONVERT_16S, 4},
{44, 14, 3696, 32, 0, gScenarioBufferData231_DOWN_CONVERT_16S, 64, gScenarioKernelData231_DOWN_CONVERT_16S, 4},
{2, 16, 192, 32, 0, gScenarioBufferData232_DOWN_CONVERT_16S, 64, gScenarioKernelData232_DOWN_CONVERT_16S, 4},
{4, 16, 384, 32, 0, gScenarioBufferData233_DOWN_CONVERT_16S, 64, gScenarioKernelData233_DOWN_CONVERT_16S, 4},
{6, 16, 576, 32, 0, gScenarioBufferData234_DOWN_CONVERT_16S, 64, gScenarioKernelData234_DOWN_CONVERT_16S, 4},
{8, 16, 768, 32, 0, gScenarioBufferData235_DOWN_CONVERT_16S, 64, gScenarioKernelData235_DOWN_CONVERT_16S, 4},
{10, 16, 960, 32, 0, gScenarioBufferData236_DOWN_CONVERT_16S, 64, gScenarioKernelData236_DOWN_CONVERT_16S, 4},
{12, 16, 1152, 32, 0, gScenarioBufferData237_DOWN_CONVERT_16S, 64, gScenarioKernelData237_DOWN_CONVERT_16S, 4},
{14, 16, 1344, 32, 0, gScenarioBufferData238_DOWN_CONVERT_16S, 64, gScenarioKernelData238_DOWN_CONVERT_16S, 4},
{16, 16, 1536, 32, 0, gScenarioBufferData239_DOWN_CONVERT_16S, 64, gScenarioKernelData239_DOWN_CONVERT_16S, 4},
{18, 16, 1728, 32, 0, gScenarioBufferData240_DOWN_CONVERT_16S, 64, gScenarioKernelData240_DOWN_CONVERT_16S, 4},
{20, 16, 1920, 32, 0, gScenarioBufferData241_DOWN_CONVERT_16S, 64, gScenarioKernelData241_DOWN_CONVERT_16S, 4},
{22, 16, 2112, 32, 0, gScenarioBufferData242_DOWN_CONVERT_16S, 64, gScenarioKernelData242_DOWN_CONVERT_16S, 4},
{24, 16, 2304, 32, 0, gScenarioBufferData243_DOWN_CONVERT_16S, 64, gScenarioKernelData243_DOWN_CONVERT_16S, 4},
{26, 16, 2496, 32, 0, gScenarioBufferData244_DOWN_CONVERT_16S, 64, gScenarioKernelData244_DOWN_CONVERT_16S, 4},
{28, 16, 2688, 32, 0, gScenarioBufferData245_DOWN_CONVERT_16S, 64, gScenarioKernelData245_DOWN_CONVERT_16S, 4},
{30, 16, 2880, 32, 0, gScenarioBufferData246_DOWN_CONVERT_16S, 64, gScenarioKernelData246_DOWN_CONVERT_16S, 4},
{32, 16, 3072, 32, 0, gScenarioBufferData247_DOWN_CONVERT_16S, 64, gScenarioKernelData247_DOWN_CONVERT_16S, 4},
{36, 16, 3456, 32, 0, gScenarioBufferData248_DOWN_CONVERT_16S, 64, gScenarioKernelData248_DOWN_CONVERT_16S, 4},
{40, 16, 3840, 32, 0, gScenarioBufferData249_DOWN_CONVERT_16S, 64, gScenarioKernelData249_DOWN_CONVERT_16S, 4},
{2, 18, 216, 32, 0, gScenarioBufferData250_DOWN_CONVERT_16S, 64, gScenarioKernelData250_DOWN_CONVERT_16S, 4},
{4, 18, 432, 32, 0, gScenarioBufferData251_DOWN_CONVERT_16S, 64, gScenarioKernelData251_DOWN_CONVERT_16S, 4},
{6, 18, 648, 32, 0, gScenarioBufferData252_DOWN_CONVERT_16S, 64, gScenarioKernelData252_DOWN_CONVERT_16S, 4},
{8, 18, 864, 32, 0, gScenarioBufferData253_DOWN_CONVERT_16S, 64, gScenarioKernelData253_DOWN_CONVERT_16S, 4},
{10, 18, 1080, 32, 0, gScenarioBufferData254_DOWN_CONVERT_16S, 64, gScenarioKernelData254_DOWN_CONVERT_16S, 4},
{12, 18, 1296, 32, 0, gScenarioBufferData255_DOWN_CONVERT_16S, 64, gScenarioKernelData255_DOWN_CONVERT_16S, 4},
{14, 18, 1512, 32, 0, gScenarioBufferData256_DOWN_CONVERT_16S, 64, gScenarioKernelData256_DOWN_CONVERT_16S, 4},
{16, 18, 1728, 32, 0, gScenarioBufferData257_DOWN_CONVERT_16S, 64, gScenarioKernelData257_DOWN_CONVERT_16S, 4},
{18, 18, 1944, 32, 0, gScenarioBufferData258_DOWN_CONVERT_16S, 64, gScenarioKernelData258_DOWN_CONVERT_16S, 4},
{20, 18, 2160, 32, 0, gScenarioBufferData259_DOWN_CONVERT_16S, 64, gScenarioKernelData259_DOWN_CONVERT_16S, 4},
{22, 18, 2376, 32, 0, gScenarioBufferData260_DOWN_CONVERT_16S, 64, gScenarioKernelData260_DOWN_CONVERT_16S, 4},
{24, 18, 2592, 32, 0, gScenarioBufferData261_DOWN_CONVERT_16S, 64, gScenarioKernelData261_DOWN_CONVERT_16S, 4},
{26, 18, 2808, 32, 0, gScenarioBufferData262_DOWN_CONVERT_16S, 64, gScenarioKernelData262_DOWN_CONVERT_16S, 4},
{28, 18, 3024, 32, 0, gScenarioBufferData263_DOWN_CONVERT_16S, 64, gScenarioKernelData263_DOWN_CONVERT_16S, 4},
{30, 18, 3240, 32, 0, gScenarioBufferData264_DOWN_CONVERT_16S, 64, gScenarioKernelData264_DOWN_CONVERT_16S, 4},
{32, 18, 3456, 32, 0, gScenarioBufferData265_DOWN_CONVERT_16S, 64, gScenarioKernelData265_DOWN_CONVERT_16S, 4},
{2, 20, 240, 32, 0, gScenarioBufferData266_DOWN_CONVERT_16S, 64, gScenarioKernelData266_DOWN_CONVERT_16S, 4},
{4, 20, 480, 32, 0, gScenarioBufferData267_DOWN_CONVERT_16S, 64, gScenarioKernelData267_DOWN_CONVERT_16S, 4},
{6, 20, 720, 32, 0, gScenarioBufferData268_DOWN_CONVERT_16S, 64, gScenarioKernelData268_DOWN_CONVERT_16S, 4},
{8, 20, 960, 32, 0, gScenarioBufferData269_DOWN_CONVERT_16S, 64, gScenarioKernelData269_DOWN_CONVERT_16S, 4},
{10, 20, 1200, 32, 0, gScenarioBufferData270_DOWN_CONVERT_16S, 64, gScenarioKernelData270_DOWN_CONVERT_16S, 4},
{12, 20, 1440, 32, 0, gScenarioBufferData271_DOWN_CONVERT_16S, 64, gScenarioKernelData271_DOWN_CONVERT_16S, 4},
{14, 20, 1680, 32, 0, gScenarioBufferData272_DOWN_CONVERT_16S, 64, gScenarioKernelData272_DOWN_CONVERT_16S, 4},
{16, 20, 1920, 32, 0, gScenarioBufferData273_DOWN_CONVERT_16S, 64, gScenarioKernelData273_DOWN_CONVERT_16S, 4},
{18, 20, 2160, 32, 0, gScenarioBufferData274_DOWN_CONVERT_16S, 64, gScenarioKernelData274_DOWN_CONVERT_16S, 4},
{20, 20, 2400, 32, 0, gScenarioBufferData275_DOWN_CONVERT_16S, 64, gScenarioKernelData275_DOWN_CONVERT_16S, 4},
{22, 20, 2640, 32, 0, gScenarioBufferData276_DOWN_CONVERT_16S, 64, gScenarioKernelData276_DOWN_CONVERT_16S, 4},
{24, 20, 2880, 32, 0, gScenarioBufferData277_DOWN_CONVERT_16S, 64, gScenarioKernelData277_DOWN_CONVERT_16S, 4},
{26, 20, 3120, 32, 0, gScenarioBufferData278_DOWN_CONVERT_16S, 64, gScenarioKernelData278_DOWN_CONVERT_16S, 4},
{28, 20, 3360, 32, 0, gScenarioBufferData279_DOWN_CONVERT_16S, 64, gScenarioKernelData279_DOWN_CONVERT_16S, 4},
{30, 20, 3600, 32, 0, gScenarioBufferData280_DOWN_CONVERT_16S, 64, gScenarioKernelData280_DOWN_CONVERT_16S, 4},
{32, 20, 3840, 32, 0, gScenarioBufferData281_DOWN_CONVERT_16S, 64, gScenarioKernelData281_DOWN_CONVERT_16S, 4},
{2, 22, 264, 32, 0, gScenarioBufferData282_DOWN_CONVERT_16S, 64, gScenarioKernelData282_DOWN_CONVERT_16S, 4},
{4, 22, 528, 32, 0, gScenarioBufferData283_DOWN_CONVERT_16S, 64, gScenarioKernelData283_DOWN_CONVERT_16S, 4},
{6, 22, 792, 32, 0, gScenarioBufferData284_DOWN_CONVERT_16S, 64, gScenarioKernelData284_DOWN_CONVERT_16S, 4},
{8, 22, 1056, 32, 0, gScenarioBufferData285_DOWN_CONVERT_16S, 64, gScenarioKernelData285_DOWN_CONVERT_16S, 4},
{10, 22, 1320, 32, 0, gScenarioBufferData286_DOWN_CONVERT_16S, 64, gScenarioKernelData286_DOWN_CONVERT_16S, 4},
{12, 22, 1584, 32, 0, gScenarioBufferData287_DOWN_CONVERT_16S, 64, gScenarioKernelData287_DOWN_CONVERT_16S, 4},
{14, 22, 1848, 32, 0, gScenarioBufferData288_DOWN_CONVERT_16S, 64, gScenarioKernelData288_DOWN_CONVERT_16S, 4},
{16, 22, 2112, 32, 0, gScenarioBufferData289_DOWN_CONVERT_16S, 64, gScenarioKernelData289_DOWN_CONVERT_16S, 4},
{18, 22, 2376, 32, 0, gScenarioBufferData290_DOWN_CONVERT_16S, 64, gScenarioKernelData290_DOWN_CONVERT_16S, 4},
{20, 22, 2640, 32, 0, gScenarioBufferData291_DOWN_CONVERT_16S, 64, gScenarioKernelData291_DOWN_CONVERT_16S, 4},
{22, 22, 2904, 32, 0, gScenarioBufferData292_DOWN_CONVERT_16S, 64, gScenarioKernelData292_DOWN_CONVERT_16S, 4},
{24, 22, 3168, 32, 0, gScenarioBufferData293_DOWN_CONVERT_16S, 64, gScenarioKernelData293_DOWN_CONVERT_16S, 4},
{26, 22, 3432, 32, 0, gScenarioBufferData294_DOWN_CONVERT_16S, 64, gScenarioKernelData294_DOWN_CONVERT_16S, 4},
{28, 22, 3696, 32, 0, gScenarioBufferData295_DOWN_CONVERT_16S, 64, gScenarioKernelData295_DOWN_CONVERT_16S, 4},
{2, 24, 288, 32, 0, gScenarioBufferData296_DOWN_CONVERT_16S, 64, gScenarioKernelData296_DOWN_CONVERT_16S, 4},
{4, 24, 576, 32, 0, gScenarioBufferData297_DOWN_CONVERT_16S, 64, gScenarioKernelData297_DOWN_CONVERT_16S, 4},
{6, 24, 864, 32, 0, gScenarioBufferData298_DOWN_CONVERT_16S, 64, gScenarioKernelData298_DOWN_CONVERT_16S, 4},
{8, 24, 1152, 32, 0, gScenarioBufferData299_DOWN_CONVERT_16S, 64, gScenarioKernelData299_DOWN_CONVERT_16S, 4},
{10, 24, 1440, 32, 0, gScenarioBufferData300_DOWN_CONVERT_16S, 64, gScenarioKernelData300_DOWN_CONVERT_16S, 4},
{12, 24, 1728, 32, 0, gScenarioBufferData301_DOWN_CONVERT_16S, 64, gScenarioKernelData301_DOWN_CONVERT_16S, 4},
{14, 24, 2016, 32, 0, gScenarioBufferData302_DOWN_CONVERT_16S, 64, gScenarioKernelData302_DOWN_CONVERT_16S, 4},
{16, 24, 2304, 32, 0, gScenarioBufferData303_DOWN_CONVERT_16S, 64, gScenarioKernelData303_DOWN_CONVERT_16S, 4},
{18, 24, 2592, 32, 0, gScenarioBufferData304_DOWN_CONVERT_16S, 64, gScenarioKernelData304_DOWN_CONVERT_16S, 4},
{20, 24, 2880, 32, 0, gScenarioBufferData305_DOWN_CONVERT_16S, 64, gScenarioKernelData305_DOWN_CONVERT_16S, 4},
{22, 24, 3168, 32, 0, gScenarioBufferData306_DOWN_CONVERT_16S, 64, gScenarioKernelData306_DOWN_CONVERT_16S, 4},
{24, 24, 3456, 32, 0, gScenarioBufferData307_DOWN_CONVERT_16S, 64, gScenarioKernelData307_DOWN_CONVERT_16S, 4},
{26, 24, 3744, 32, 0, gScenarioBufferData308_DOWN_CONVERT_16S, 64, gScenarioKernelData308_DOWN_CONVERT_16S, 4},
{2, 26, 312, 32, 0, gScenarioBufferData309_DOWN_CONVERT_16S, 64, gScenarioKernelData309_DOWN_CONVERT_16S, 4},
{4, 26, 624, 32, 0, gScenarioBufferData310_DOWN_CONVERT_16S, 64, gScenarioKernelData310_DOWN_CONVERT_16S, 4},
{6, 26, 936, 32, 0, gScenarioBufferData311_DOWN_CONVERT_16S, 64, gScenarioKernelData311_DOWN_CONVERT_16S, 4},
{8, 26, 1248, 32, 0, gScenarioBufferData312_DOWN_CONVERT_16S, 64, gScenarioKernelData312_DOWN_CONVERT_16S, 4},
{10, 26, 1560, 32, 0, gScenarioBufferData313_DOWN_CONVERT_16S, 64, gScenarioKernelData313_DOWN_CONVERT_16S, 4},
{12, 26, 1872, 32, 0, gScenarioBufferData314_DOWN_CONVERT_16S, 64, gScenarioKernelData314_DOWN_CONVERT_16S, 4},
{14, 26, 2184, 32, 0, gScenarioBufferData315_DOWN_CONVERT_16S, 64, gScenarioKernelData315_DOWN_CONVERT_16S, 4},
{16, 26, 2496, 32, 0, gScenarioBufferData316_DOWN_CONVERT_16S, 64, gScenarioKernelData316_DOWN_CONVERT_16S, 4},
{18, 26, 2808, 32, 0, gScenarioBufferData317_DOWN_CONVERT_16S, 64, gScenarioKernelData317_DOWN_CONVERT_16S, 4},
{20, 26, 3120, 32, 0, gScenarioBufferData318_DOWN_CONVERT_16S, 64, gScenarioKernelData318_DOWN_CONVERT_16S, 4},
{22, 26, 3432, 32, 0, gScenarioBufferData319_DOWN_CONVERT_16S, 64, gScenarioKernelData319_DOWN_CONVERT_16S, 4},
{24, 26, 3744, 32, 0, gScenarioBufferData320_DOWN_CONVERT_16S, 64, gScenarioKernelData320_DOWN_CONVERT_16S, 4},
{2, 28, 336, 32, 0, gScenarioBufferData321_DOWN_CONVERT_16S, 64, gScenarioKernelData321_DOWN_CONVERT_16S, 4},
{4, 28, 672, 32, 0, gScenarioBufferData322_DOWN_CONVERT_16S, 64, gScenarioKernelData322_DOWN_CONVERT_16S, 4},
{6, 28, 1008, 32, 0, gScenarioBufferData323_DOWN_CONVERT_16S, 64, gScenarioKernelData323_DOWN_CONVERT_16S, 4},
{8, 28, 1344, 32, 0, gScenarioBufferData324_DOWN_CONVERT_16S, 64, gScenarioKernelData324_DOWN_CONVERT_16S, 4},
{10, 28, 1680, 32, 0, gScenarioBufferData325_DOWN_CONVERT_16S, 64, gScenarioKernelData325_DOWN_CONVERT_16S, 4},
{12, 28, 2016, 32, 0, gScenarioBufferData326_DOWN_CONVERT_16S, 64, gScenarioKernelData326_DOWN_CONVERT_16S, 4},
{14, 28, 2352, 32, 0, gScenarioBufferData327_DOWN_CONVERT_16S, 64, gScenarioKernelData327_DOWN_CONVERT_16S, 4},
{16, 28, 2688, 32, 0, gScenarioBufferData328_DOWN_CONVERT_16S, 64, gScenarioKernelData328_DOWN_CONVERT_16S, 4},
{18, 28, 3024, 32, 0, gScenarioBufferData329_DOWN_CONVERT_16S, 64, gScenarioKernelData329_DOWN_CONVERT_16S, 4},
{20, 28, 3360, 32, 0, gScenarioBufferData330_DOWN_CONVERT_16S, 64, gScenarioKernelData330_DOWN_CONVERT_16S, 4},
{22, 28, 3696, 32, 0, gScenarioBufferData331_DOWN_CONVERT_16S, 64, gScenarioKernelData331_DOWN_CONVERT_16S, 4},
{2, 30, 360, 32, 0, gScenarioBufferData332_DOWN_CONVERT_16S, 64, gScenarioKernelData332_DOWN_CONVERT_16S, 4},
{4, 30, 720, 32, 0, gScenarioBufferData333_DOWN_CONVERT_16S, 64, gScenarioKernelData333_DOWN_CONVERT_16S, 4},
{6, 30, 1080, 32, 0, gScenarioBufferData334_DOWN_CONVERT_16S, 64, gScenarioKernelData334_DOWN_CONVERT_16S, 4},
{8, 30, 1440, 32, 0, gScenarioBufferData335_DOWN_CONVERT_16S, 64, gScenarioKernelData335_DOWN_CONVERT_16S, 4},
{10, 30, 1800, 32, 0, gScenarioBufferData336_DOWN_CONVERT_16S, 64, gScenarioKernelData336_DOWN_CONVERT_16S, 4},
{12, 30, 2160, 32, 0, gScenarioBufferData337_DOWN_CONVERT_16S, 64, gScenarioKernelData337_DOWN_CONVERT_16S, 4},
{14, 30, 2520, 32, 0, gScenarioBufferData338_DOWN_CONVERT_16S, 64, gScenarioKernelData338_DOWN_CONVERT_16S, 4},
{16, 30, 2880, 32, 0, gScenarioBufferData339_DOWN_CONVERT_16S, 64, gScenarioKernelData339_DOWN_CONVERT_16S, 4},
{18, 30, 3240, 32, 0, gScenarioBufferData340_DOWN_CONVERT_16S, 64, gScenarioKernelData340_DOWN_CONVERT_16S, 4},
{20, 30, 3600, 32, 0, gScenarioBufferData341_DOWN_CONVERT_16S, 64, gScenarioKernelData341_DOWN_CONVERT_16S, 4},
{2, 32, 384, 32, 0, gScenarioBufferData342_DOWN_CONVERT_16S, 64, gScenarioKernelData342_DOWN_CONVERT_16S, 4},
{4, 32, 768, 32, 0, gScenarioBufferData343_DOWN_CONVERT_16S, 64, gScenarioKernelData343_DOWN_CONVERT_16S, 4},
{6, 32, 1152, 32, 0, gScenarioBufferData344_DOWN_CONVERT_16S, 64, gScenarioKernelData344_DOWN_CONVERT_16S, 4},
{8, 32, 1536, 32, 0, gScenarioBufferData345_DOWN_CONVERT_16S, 64, gScenarioKernelData345_DOWN_CONVERT_16S, 4},
{10, 32, 1920, 32, 0, gScenarioBufferData346_DOWN_CONVERT_16S, 64, gScenarioKernelData346_DOWN_CONVERT_16S, 4},
{12, 32, 2304, 32, 0, gScenarioBufferData347_DOWN_CONVERT_16S, 64, gScenarioKernelData347_DOWN_CONVERT_16S, 4},
{14, 32, 2688, 32, 0, gScenarioBufferData348_DOWN_CONVERT_16S, 64, gScenarioKernelData348_DOWN_CONVERT_16S, 4},
{16, 32, 3072, 32, 0, gScenarioBufferData349_DOWN_CONVERT_16S, 64, gScenarioKernelData349_DOWN_CONVERT_16S, 4},
{18, 32, 3456, 32, 0, gScenarioBufferData350_DOWN_CONVERT_16S, 64, gScenarioKernelData350_DOWN_CONVERT_16S, 4},
{20, 32, 3840, 32, 0, gScenarioBufferData351_DOWN_CONVERT_16S, 64, gScenarioKernelData351_DOWN_CONVERT_16S, 4}
};

static acf_scenario_list gScenarioList_DOWN_CONVERT_16S = {
352, //number of scenarios
gScenarioArray_DOWN_CONVERT_16S};
//**************************************************************

class DOWN_CONVERT_16S : public ACF_Process_APU
{

public:
   DOWN_CONVERT_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("DOWN_CONVERT_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("DOWN_CONVERT_16S",
                                        DOWN_CONVERT_16S_LOAD_SEGMENTS,
                                        DOWN_CONVERT_16S_LOAD_PMEM, DOWN_CONVERT_16S_LOAD_PMEM_SIZE,
                                        DOWN_CONVERT_16S_LOAD_DMEM, DOWN_CONVERT_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(DOWN_CONVERT_16S_APEX_LOG_BUFFER, DOWN_CONVERT_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_DOWN_CONVERT_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_DOWN_CONVERT_16S

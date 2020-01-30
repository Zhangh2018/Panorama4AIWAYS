#ifndef _ACF_PROCESS_APU_AGGCF_FLT_NORM
#define _ACF_PROCESS_APU_AGGCF_FLT_NORM

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_FLT_NORM_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_FLT_NORM[] = {{1, 1, 4, 0, {1, 1, 1, 1}}, {1, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_FLT_NORM[] = {{2, 1, 4, 0, {1, 1, 1, 1}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_FLT_NORM[] = {{3, 1, 4, 0, {1, 1, 1, 1}}, {3, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_FLT_NORM[] = {{4, 1, 4, 0, {1, 1, 1, 1}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_FLT_NORM[] = {{5, 1, 4, 0, {1, 1, 1, 1}}, {5, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_FLT_NORM[] = {{6, 1, 4, 0, {1, 1, 1, 1}}, {6, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_FLT_NORM[] = {{7, 1, 4, 0, {1, 1, 1, 1}}, {7, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_FLT_NORM[] = {{8, 1, 4, 0, {1, 1, 1, 1}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_FLT_NORM[] = {{9, 1, 4, 0, {1, 1, 1, 1}}, {9, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_FLT_NORM[] = {{10, 1, 4, 0, {1, 1, 1, 1}}, {10, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_FLT_NORM[] = {{11, 1, 4, 0, {1, 1, 1, 1}}, {11, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_FLT_NORM[] = {{12, 1, 4, 0, {1, 1, 1, 1}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_FLT_NORM[] = {{13, 1, 4, 0, {1, 1, 1, 1}}, {13, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_FLT_NORM[] = {{14, 1, 4, 0, {1, 1, 1, 1}}, {14, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_FLT_NORM[] = {{15, 1, 4, 0, {1, 1, 1, 1}}, {15, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_FLT_NORM[] = {{16, 1, 4, 0, {1, 1, 1, 1}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_FLT_NORM[] = {{18, 1, 4, 0, {1, 1, 1, 1}}, {18, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_FLT_NORM[] = {{20, 1, 4, 0, {1, 1, 1, 1}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_FLT_NORM[] = {{22, 1, 4, 0, {1, 1, 1, 1}}, {22, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_FLT_NORM[] = {{24, 1, 4, 0, {1, 1, 1, 1}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_FLT_NORM[] = {{26, 1, 4, 0, {1, 1, 1, 1}}, {26, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_FLT_NORM[] = {{28, 1, 4, 0, {1, 1, 1, 1}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_FLT_NORM[] = {{30, 1, 4, 0, {1, 1, 1, 1}}, {30, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_FLT_NORM[] = {{32, 1, 4, 0, {1, 1, 1, 1}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_FLT_NORM[] = {{36, 1, 4, 0, {1, 1, 1, 1}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_FLT_NORM[] = {{40, 1, 4, 0, {1, 1, 1, 1}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_FLT_NORM[] = {{44, 1, 4, 0, {1, 1, 1, 1}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_FLT_NORM[] = {{48, 1, 4, 0, {1, 1, 1, 1}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AGGCF_FLT_NORM[] = {{52, 1, 4, 0, {1, 1, 1, 1}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AGGCF_FLT_NORM[] = {{56, 1, 4, 0, {1, 1, 1, 1}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AGGCF_FLT_NORM[] = {{60, 1, 4, 0, {1, 1, 1, 1}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AGGCF_FLT_NORM[] = {{64, 1, 4, 0, {1, 1, 1, 1}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AGGCF_FLT_NORM[] = {{1, 2, 4, 0, {1, 1, 1, 1}}, {1, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AGGCF_FLT_NORM[] = {{2, 2, 4, 0, {1, 1, 1, 1}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AGGCF_FLT_NORM[] = {{3, 2, 4, 0, {1, 1, 1, 1}}, {3, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AGGCF_FLT_NORM[] = {{4, 2, 4, 0, {1, 1, 1, 1}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AGGCF_FLT_NORM[] = {{5, 2, 4, 0, {1, 1, 1, 1}}, {5, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AGGCF_FLT_NORM[] = {{6, 2, 4, 0, {1, 1, 1, 1}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AGGCF_FLT_NORM[] = {{7, 2, 4, 0, {1, 1, 1, 1}}, {7, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AGGCF_FLT_NORM[] = {{8, 2, 4, 0, {1, 1, 1, 1}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AGGCF_FLT_NORM[] = {{9, 2, 4, 0, {1, 1, 1, 1}}, {9, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AGGCF_FLT_NORM[] = {{10, 2, 4, 0, {1, 1, 1, 1}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AGGCF_FLT_NORM[] = {{11, 2, 4, 0, {1, 1, 1, 1}}, {11, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AGGCF_FLT_NORM[] = {{12, 2, 4, 0, {1, 1, 1, 1}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AGGCF_FLT_NORM[] = {{13, 2, 4, 0, {1, 1, 1, 1}}, {13, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AGGCF_FLT_NORM[] = {{14, 2, 4, 0, {1, 1, 1, 1}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AGGCF_FLT_NORM[] = {{15, 2, 4, 0, {1, 1, 1, 1}}, {15, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AGGCF_FLT_NORM[] = {{16, 2, 4, 0, {1, 1, 1, 1}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AGGCF_FLT_NORM[] = {{18, 2, 4, 0, {1, 1, 1, 1}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AGGCF_FLT_NORM[] = {{20, 2, 4, 0, {1, 1, 1, 1}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AGGCF_FLT_NORM[] = {{22, 2, 4, 0, {1, 1, 1, 1}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AGGCF_FLT_NORM[] = {{24, 2, 4, 0, {1, 1, 1, 1}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AGGCF_FLT_NORM[] = {{26, 2, 4, 0, {1, 1, 1, 1}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AGGCF_FLT_NORM[] = {{28, 2, 4, 0, {1, 1, 1, 1}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AGGCF_FLT_NORM[] = {{30, 2, 4, 0, {1, 1, 1, 1}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AGGCF_FLT_NORM[] = {{32, 2, 4, 0, {1, 1, 1, 1}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_AGGCF_FLT_NORM[] = {{36, 2, 4, 0, {1, 1, 1, 1}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_AGGCF_FLT_NORM[] = {{40, 2, 4, 0, {1, 1, 1, 1}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_AGGCF_FLT_NORM[] = {{44, 2, 4, 0, {1, 1, 1, 1}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_AGGCF_FLT_NORM[] = {{48, 2, 4, 0, {1, 1, 1, 1}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_AGGCF_FLT_NORM[] = {{52, 2, 4, 0, {1, 1, 1, 1}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_AGGCF_FLT_NORM[] = {{56, 2, 4, 0, {1, 1, 1, 1}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_AGGCF_FLT_NORM[] = {{60, 2, 4, 0, {1, 1, 1, 1}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_AGGCF_FLT_NORM[] = {{64, 2, 4, 0, {1, 1, 1, 1}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_AGGCF_FLT_NORM[] = {{1, 3, 4, 0, {1, 1, 1, 1}}, {1, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_AGGCF_FLT_NORM[] = {{2, 3, 4, 0, {1, 1, 1, 1}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_AGGCF_FLT_NORM[] = {{3, 3, 4, 0, {1, 1, 1, 1}}, {3, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_AGGCF_FLT_NORM[] = {{4, 3, 4, 0, {1, 1, 1, 1}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_AGGCF_FLT_NORM[] = {{5, 3, 4, 0, {1, 1, 1, 1}}, {5, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_AGGCF_FLT_NORM[] = {{6, 3, 4, 0, {1, 1, 1, 1}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_AGGCF_FLT_NORM[] = {{7, 3, 4, 0, {1, 1, 1, 1}}, {7, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_AGGCF_FLT_NORM[] = {{8, 3, 4, 0, {1, 1, 1, 1}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_AGGCF_FLT_NORM[] = {{9, 3, 4, 0, {1, 1, 1, 1}}, {9, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_AGGCF_FLT_NORM[] = {{10, 3, 4, 0, {1, 1, 1, 1}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_AGGCF_FLT_NORM[] = {{11, 3, 4, 0, {1, 1, 1, 1}}, {11, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_AGGCF_FLT_NORM[] = {{12, 3, 4, 0, {1, 1, 1, 1}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_AGGCF_FLT_NORM[] = {{13, 3, 4, 0, {1, 1, 1, 1}}, {13, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_AGGCF_FLT_NORM[] = {{14, 3, 4, 0, {1, 1, 1, 1}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_AGGCF_FLT_NORM[] = {{15, 3, 4, 0, {1, 1, 1, 1}}, {15, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_AGGCF_FLT_NORM[] = {{16, 3, 4, 0, {1, 1, 1, 1}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_AGGCF_FLT_NORM[] = {{18, 3, 4, 0, {1, 1, 1, 1}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_AGGCF_FLT_NORM[] = {{20, 3, 4, 0, {1, 1, 1, 1}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_AGGCF_FLT_NORM[] = {{22, 3, 4, 0, {1, 1, 1, 1}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_AGGCF_FLT_NORM[] = {{24, 3, 4, 0, {1, 1, 1, 1}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_AGGCF_FLT_NORM[] = {{26, 3, 4, 0, {1, 1, 1, 1}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_AGGCF_FLT_NORM[] = {{28, 3, 4, 0, {1, 1, 1, 1}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_AGGCF_FLT_NORM[] = {{30, 3, 4, 0, {1, 1, 1, 1}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_AGGCF_FLT_NORM[] = {{32, 3, 4, 0, {1, 1, 1, 1}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_AGGCF_FLT_NORM[] = {{36, 3, 4, 0, {1, 1, 1, 1}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_AGGCF_FLT_NORM[] = {{40, 3, 4, 0, {1, 1, 1, 1}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_AGGCF_FLT_NORM[] = {{44, 3, 4, 0, {1, 1, 1, 1}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_AGGCF_FLT_NORM[] = {{48, 3, 4, 0, {1, 1, 1, 1}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_AGGCF_FLT_NORM[] = {{52, 3, 4, 0, {1, 1, 1, 1}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_AGGCF_FLT_NORM[] = {{56, 3, 4, 0, {1, 1, 1, 1}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_AGGCF_FLT_NORM[] = {{60, 3, 4, 0, {1, 1, 1, 1}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_AGGCF_FLT_NORM[] = {{64, 3, 4, 0, {1, 1, 1, 1}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_AGGCF_FLT_NORM[] = {{1, 4, 4, 0, {1, 1, 1, 1}}, {1, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_AGGCF_FLT_NORM[] = {{2, 4, 4, 0, {1, 1, 1, 1}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_AGGCF_FLT_NORM[] = {{3, 4, 4, 0, {1, 1, 1, 1}}, {3, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_AGGCF_FLT_NORM[] = {{4, 4, 4, 0, {1, 1, 1, 1}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_AGGCF_FLT_NORM[] = {{5, 4, 4, 0, {1, 1, 1, 1}}, {5, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_AGGCF_FLT_NORM[] = {{6, 4, 4, 0, {1, 1, 1, 1}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_AGGCF_FLT_NORM[] = {{7, 4, 4, 0, {1, 1, 1, 1}}, {7, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_AGGCF_FLT_NORM[] = {{8, 4, 4, 0, {1, 1, 1, 1}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_AGGCF_FLT_NORM[] = {{9, 4, 4, 0, {1, 1, 1, 1}}, {9, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_AGGCF_FLT_NORM[] = {{10, 4, 4, 0, {1, 1, 1, 1}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_AGGCF_FLT_NORM[] = {{11, 4, 4, 0, {1, 1, 1, 1}}, {11, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_AGGCF_FLT_NORM[] = {{12, 4, 4, 0, {1, 1, 1, 1}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_AGGCF_FLT_NORM[] = {{13, 4, 4, 0, {1, 1, 1, 1}}, {13, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_AGGCF_FLT_NORM[] = {{14, 4, 4, 0, {1, 1, 1, 1}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_AGGCF_FLT_NORM[] = {{15, 4, 4, 0, {1, 1, 1, 1}}, {15, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_AGGCF_FLT_NORM[] = {{16, 4, 4, 0, {1, 1, 1, 1}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_AGGCF_FLT_NORM[] = {{18, 4, 4, 0, {1, 1, 1, 1}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_AGGCF_FLT_NORM[] = {{20, 4, 4, 0, {1, 1, 1, 1}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_AGGCF_FLT_NORM[] = {{22, 4, 4, 0, {1, 1, 1, 1}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_AGGCF_FLT_NORM[] = {{24, 4, 4, 0, {1, 1, 1, 1}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_AGGCF_FLT_NORM[] = {{26, 4, 4, 0, {1, 1, 1, 1}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_AGGCF_FLT_NORM[] = {{28, 4, 4, 0, {1, 1, 1, 1}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_AGGCF_FLT_NORM[] = {{30, 4, 4, 0, {1, 1, 1, 1}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_AGGCF_FLT_NORM[] = {{32, 4, 4, 0, {1, 1, 1, 1}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_AGGCF_FLT_NORM[] = {{36, 4, 4, 0, {1, 1, 1, 1}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_AGGCF_FLT_NORM[] = {{40, 4, 4, 0, {1, 1, 1, 1}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_AGGCF_FLT_NORM[] = {{44, 4, 4, 0, {1, 1, 1, 1}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_AGGCF_FLT_NORM[] = {{48, 4, 4, 0, {1, 1, 1, 1}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_AGGCF_FLT_NORM[] = {{52, 4, 4, 0, {1, 1, 1, 1}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_AGGCF_FLT_NORM[] = {{56, 4, 4, 0, {1, 1, 1, 1}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_AGGCF_FLT_NORM[] = {{60, 4, 4, 0, {1, 1, 1, 1}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_AGGCF_FLT_NORM[] = {{64, 4, 4, 0, {1, 1, 1, 1}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_AGGCF_FLT_NORM[] = {{1, 5, 4, 0, {1, 1, 1, 1}}, {1, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_AGGCF_FLT_NORM[] = {{2, 5, 4, 0, {1, 1, 1, 1}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_AGGCF_FLT_NORM[] = {{3, 5, 4, 0, {1, 1, 1, 1}}, {3, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_AGGCF_FLT_NORM[] = {{4, 5, 4, 0, {1, 1, 1, 1}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_AGGCF_FLT_NORM[] = {{5, 5, 4, 0, {1, 1, 1, 1}}, {5, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_AGGCF_FLT_NORM[] = {{6, 5, 4, 0, {1, 1, 1, 1}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_AGGCF_FLT_NORM[] = {{7, 5, 4, 0, {1, 1, 1, 1}}, {7, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_AGGCF_FLT_NORM[] = {{8, 5, 4, 0, {1, 1, 1, 1}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_AGGCF_FLT_NORM[] = {{9, 5, 4, 0, {1, 1, 1, 1}}, {9, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_AGGCF_FLT_NORM[] = {{10, 5, 4, 0, {1, 1, 1, 1}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_AGGCF_FLT_NORM[] = {{11, 5, 4, 0, {1, 1, 1, 1}}, {11, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_AGGCF_FLT_NORM[] = {{12, 5, 4, 0, {1, 1, 1, 1}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_AGGCF_FLT_NORM[] = {{13, 5, 4, 0, {1, 1, 1, 1}}, {13, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_AGGCF_FLT_NORM[] = {{14, 5, 4, 0, {1, 1, 1, 1}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_AGGCF_FLT_NORM[] = {{15, 5, 4, 0, {1, 1, 1, 1}}, {15, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_AGGCF_FLT_NORM[] = {{16, 5, 4, 0, {1, 1, 1, 1}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_AGGCF_FLT_NORM[] = {{18, 5, 4, 0, {1, 1, 1, 1}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_AGGCF_FLT_NORM[] = {{20, 5, 4, 0, {1, 1, 1, 1}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_AGGCF_FLT_NORM[] = {{22, 5, 4, 0, {1, 1, 1, 1}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_AGGCF_FLT_NORM[] = {{24, 5, 4, 0, {1, 1, 1, 1}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_AGGCF_FLT_NORM[] = {{26, 5, 4, 0, {1, 1, 1, 1}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_AGGCF_FLT_NORM[] = {{28, 5, 4, 0, {1, 1, 1, 1}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_AGGCF_FLT_NORM[] = {{30, 5, 4, 0, {1, 1, 1, 1}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_AGGCF_FLT_NORM[] = {{32, 5, 4, 0, {1, 1, 1, 1}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_AGGCF_FLT_NORM[] = {{36, 5, 4, 0, {1, 1, 1, 1}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_AGGCF_FLT_NORM[] = {{40, 5, 4, 0, {1, 1, 1, 1}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_AGGCF_FLT_NORM[] = {{44, 5, 4, 0, {1, 1, 1, 1}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_AGGCF_FLT_NORM[] = {{48, 5, 4, 0, {1, 1, 1, 1}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_AGGCF_FLT_NORM[] = {{52, 5, 4, 0, {1, 1, 1, 1}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_AGGCF_FLT_NORM[] = {{56, 5, 4, 0, {1, 1, 1, 1}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_AGGCF_FLT_NORM[] = {{1, 6, 4, 0, {1, 1, 1, 1}}, {1, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_AGGCF_FLT_NORM[] = {{2, 6, 4, 0, {1, 1, 1, 1}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_AGGCF_FLT_NORM[] = {{3, 6, 4, 0, {1, 1, 1, 1}}, {3, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_AGGCF_FLT_NORM[] = {{4, 6, 4, 0, {1, 1, 1, 1}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_AGGCF_FLT_NORM[] = {{5, 6, 4, 0, {1, 1, 1, 1}}, {5, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_AGGCF_FLT_NORM[] = {{6, 6, 4, 0, {1, 1, 1, 1}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_AGGCF_FLT_NORM[] = {{7, 6, 4, 0, {1, 1, 1, 1}}, {7, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_AGGCF_FLT_NORM[] = {{8, 6, 4, 0, {1, 1, 1, 1}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_AGGCF_FLT_NORM[] = {{9, 6, 4, 0, {1, 1, 1, 1}}, {9, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_AGGCF_FLT_NORM[] = {{10, 6, 4, 0, {1, 1, 1, 1}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_AGGCF_FLT_NORM[] = {{11, 6, 4, 0, {1, 1, 1, 1}}, {11, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_AGGCF_FLT_NORM[] = {{12, 6, 4, 0, {1, 1, 1, 1}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_AGGCF_FLT_NORM[] = {{13, 6, 4, 0, {1, 1, 1, 1}}, {13, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_AGGCF_FLT_NORM[] = {{14, 6, 4, 0, {1, 1, 1, 1}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_AGGCF_FLT_NORM[] = {{15, 6, 4, 0, {1, 1, 1, 1}}, {15, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_AGGCF_FLT_NORM[] = {{16, 6, 4, 0, {1, 1, 1, 1}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_AGGCF_FLT_NORM[] = {{18, 6, 4, 0, {1, 1, 1, 1}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_AGGCF_FLT_NORM[] = {{20, 6, 4, 0, {1, 1, 1, 1}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_AGGCF_FLT_NORM[] = {{22, 6, 4, 0, {1, 1, 1, 1}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_AGGCF_FLT_NORM[] = {{24, 6, 4, 0, {1, 1, 1, 1}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_AGGCF_FLT_NORM[] = {{26, 6, 4, 0, {1, 1, 1, 1}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_AGGCF_FLT_NORM[] = {{28, 6, 4, 0, {1, 1, 1, 1}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_AGGCF_FLT_NORM[] = {{30, 6, 4, 0, {1, 1, 1, 1}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_AGGCF_FLT_NORM[] = {{32, 6, 4, 0, {1, 1, 1, 1}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_AGGCF_FLT_NORM[] = {{36, 6, 4, 0, {1, 1, 1, 1}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_AGGCF_FLT_NORM[] = {{40, 6, 4, 0, {1, 1, 1, 1}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_AGGCF_FLT_NORM[] = {{44, 6, 4, 0, {1, 1, 1, 1}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_AGGCF_FLT_NORM[] = {{48, 6, 4, 0, {1, 1, 1, 1}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_AGGCF_FLT_NORM[] = {{1, 8, 4, 0, {1, 1, 1, 1}}, {1, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_AGGCF_FLT_NORM[] = {{2, 8, 4, 0, {1, 1, 1, 1}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_AGGCF_FLT_NORM[] = {{3, 8, 4, 0, {1, 1, 1, 1}}, {3, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_AGGCF_FLT_NORM[] = {{4, 8, 4, 0, {1, 1, 1, 1}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_AGGCF_FLT_NORM[] = {{5, 8, 4, 0, {1, 1, 1, 1}}, {5, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_AGGCF_FLT_NORM[] = {{6, 8, 4, 0, {1, 1, 1, 1}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_AGGCF_FLT_NORM[] = {{7, 8, 4, 0, {1, 1, 1, 1}}, {7, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_AGGCF_FLT_NORM[] = {{8, 8, 4, 0, {1, 1, 1, 1}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_AGGCF_FLT_NORM[] = {{9, 8, 4, 0, {1, 1, 1, 1}}, {9, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_AGGCF_FLT_NORM[] = {{10, 8, 4, 0, {1, 1, 1, 1}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_AGGCF_FLT_NORM[] = {{11, 8, 4, 0, {1, 1, 1, 1}}, {11, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_AGGCF_FLT_NORM[] = {{12, 8, 4, 0, {1, 1, 1, 1}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_AGGCF_FLT_NORM[] = {{13, 8, 4, 0, {1, 1, 1, 1}}, {13, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_AGGCF_FLT_NORM[] = {{14, 8, 4, 0, {1, 1, 1, 1}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_AGGCF_FLT_NORM[] = {{15, 8, 4, 0, {1, 1, 1, 1}}, {15, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_AGGCF_FLT_NORM[] = {{16, 8, 4, 0, {1, 1, 1, 1}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_AGGCF_FLT_NORM[] = {{18, 8, 4, 0, {1, 1, 1, 1}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_AGGCF_FLT_NORM[] = {{20, 8, 4, 0, {1, 1, 1, 1}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_AGGCF_FLT_NORM[] = {{22, 8, 4, 0, {1, 1, 1, 1}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_AGGCF_FLT_NORM[] = {{24, 8, 4, 0, {1, 1, 1, 1}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_AGGCF_FLT_NORM[] = {{26, 8, 4, 0, {1, 1, 1, 1}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_AGGCF_FLT_NORM[] = {{28, 8, 4, 0, {1, 1, 1, 1}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_AGGCF_FLT_NORM[] = {{30, 8, 4, 0, {1, 1, 1, 1}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_AGGCF_FLT_NORM[] = {{32, 8, 4, 0, {1, 1, 1, 1}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_AGGCF_FLT_NORM[] = {{36, 8, 4, 0, {1, 1, 1, 1}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_AGGCF_FLT_NORM[] = {{1, 10, 4, 0, {1, 1, 1, 1}}, {1, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_AGGCF_FLT_NORM[] = {{2, 10, 4, 0, {1, 1, 1, 1}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_AGGCF_FLT_NORM[] = {{3, 10, 4, 0, {1, 1, 1, 1}}, {3, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_AGGCF_FLT_NORM[] = {{4, 10, 4, 0, {1, 1, 1, 1}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_AGGCF_FLT_NORM[] = {{5, 10, 4, 0, {1, 1, 1, 1}}, {5, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_AGGCF_FLT_NORM[] = {{6, 10, 4, 0, {1, 1, 1, 1}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_AGGCF_FLT_NORM[] = {{7, 10, 4, 0, {1, 1, 1, 1}}, {7, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_AGGCF_FLT_NORM[] = {{8, 10, 4, 0, {1, 1, 1, 1}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_AGGCF_FLT_NORM[] = {{9, 10, 4, 0, {1, 1, 1, 1}}, {9, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_AGGCF_FLT_NORM[] = {{10, 10, 4, 0, {1, 1, 1, 1}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_AGGCF_FLT_NORM[] = {{11, 10, 4, 0, {1, 1, 1, 1}}, {11, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_AGGCF_FLT_NORM[] = {{12, 10, 4, 0, {1, 1, 1, 1}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_AGGCF_FLT_NORM[] = {{13, 10, 4, 0, {1, 1, 1, 1}}, {13, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_AGGCF_FLT_NORM[] = {{14, 10, 4, 0, {1, 1, 1, 1}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_AGGCF_FLT_NORM[] = {{15, 10, 4, 0, {1, 1, 1, 1}}, {15, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_AGGCF_FLT_NORM[] = {{16, 10, 4, 0, {1, 1, 1, 1}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_AGGCF_FLT_NORM[] = {{18, 10, 4, 0, {1, 1, 1, 1}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_AGGCF_FLT_NORM[] = {{20, 10, 4, 0, {1, 1, 1, 1}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_AGGCF_FLT_NORM[] = {{22, 10, 4, 0, {1, 1, 1, 1}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_AGGCF_FLT_NORM[] = {{24, 10, 4, 0, {1, 1, 1, 1}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_AGGCF_FLT_NORM[] = {{26, 10, 4, 0, {1, 1, 1, 1}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_AGGCF_FLT_NORM[] = {{28, 10, 4, 0, {1, 1, 1, 1}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_AGGCF_FLT_NORM[] = {{1, 12, 4, 0, {1, 1, 1, 1}}, {1, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_AGGCF_FLT_NORM[] = {{2, 12, 4, 0, {1, 1, 1, 1}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_AGGCF_FLT_NORM[] = {{3, 12, 4, 0, {1, 1, 1, 1}}, {3, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_AGGCF_FLT_NORM[] = {{4, 12, 4, 0, {1, 1, 1, 1}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_AGGCF_FLT_NORM[] = {{5, 12, 4, 0, {1, 1, 1, 1}}, {5, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_AGGCF_FLT_NORM[] = {{6, 12, 4, 0, {1, 1, 1, 1}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_AGGCF_FLT_NORM[] = {{7, 12, 4, 0, {1, 1, 1, 1}}, {7, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_AGGCF_FLT_NORM[] = {{8, 12, 4, 0, {1, 1, 1, 1}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_AGGCF_FLT_NORM[] = {{9, 12, 4, 0, {1, 1, 1, 1}}, {9, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_AGGCF_FLT_NORM[] = {{10, 12, 4, 0, {1, 1, 1, 1}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_AGGCF_FLT_NORM[] = {{11, 12, 4, 0, {1, 1, 1, 1}}, {11, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_AGGCF_FLT_NORM[] = {{12, 12, 4, 0, {1, 1, 1, 1}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_AGGCF_FLT_NORM[] = {{13, 12, 4, 0, {1, 1, 1, 1}}, {13, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_AGGCF_FLT_NORM[] = {{14, 12, 4, 0, {1, 1, 1, 1}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_AGGCF_FLT_NORM[] = {{15, 12, 4, 0, {1, 1, 1, 1}}, {15, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_AGGCF_FLT_NORM[] = {{16, 12, 4, 0, {1, 1, 1, 1}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_AGGCF_FLT_NORM[] = {{18, 12, 4, 0, {1, 1, 1, 1}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_AGGCF_FLT_NORM[] = {{20, 12, 4, 0, {1, 1, 1, 1}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_AGGCF_FLT_NORM[] = {{22, 12, 4, 0, {1, 1, 1, 1}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_AGGCF_FLT_NORM[] = {{24, 12, 4, 0, {1, 1, 1, 1}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_AGGCF_FLT_NORM[] = {{1, 14, 4, 0, {1, 1, 1, 1}}, {1, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_AGGCF_FLT_NORM[] = {{2, 14, 4, 0, {1, 1, 1, 1}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_AGGCF_FLT_NORM[] = {{3, 14, 4, 0, {1, 1, 1, 1}}, {3, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_AGGCF_FLT_NORM[] = {{4, 14, 4, 0, {1, 1, 1, 1}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_AGGCF_FLT_NORM[] = {{5, 14, 4, 0, {1, 1, 1, 1}}, {5, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_AGGCF_FLT_NORM[] = {{6, 14, 4, 0, {1, 1, 1, 1}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_AGGCF_FLT_NORM[] = {{7, 14, 4, 0, {1, 1, 1, 1}}, {7, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_AGGCF_FLT_NORM[] = {{8, 14, 4, 0, {1, 1, 1, 1}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_AGGCF_FLT_NORM[] = {{9, 14, 4, 0, {1, 1, 1, 1}}, {9, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_AGGCF_FLT_NORM[] = {{10, 14, 4, 0, {1, 1, 1, 1}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_AGGCF_FLT_NORM[] = {{11, 14, 4, 0, {1, 1, 1, 1}}, {11, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_AGGCF_FLT_NORM[] = {{12, 14, 4, 0, {1, 1, 1, 1}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_AGGCF_FLT_NORM[] = {{13, 14, 4, 0, {1, 1, 1, 1}}, {13, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_AGGCF_FLT_NORM[] = {{14, 14, 4, 0, {1, 1, 1, 1}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_AGGCF_FLT_NORM[] = {{15, 14, 4, 0, {1, 1, 1, 1}}, {15, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_AGGCF_FLT_NORM[] = {{16, 14, 4, 0, {1, 1, 1, 1}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_AGGCF_FLT_NORM[] = {{18, 14, 4, 0, {1, 1, 1, 1}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_AGGCF_FLT_NORM[] = {{20, 14, 4, 0, {1, 1, 1, 1}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_AGGCF_FLT_NORM[] = {{1, 16, 4, 0, {1, 1, 1, 1}}, {1, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_AGGCF_FLT_NORM[] = {{2, 16, 4, 0, {1, 1, 1, 1}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_AGGCF_FLT_NORM[] = {{3, 16, 4, 0, {1, 1, 1, 1}}, {3, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_AGGCF_FLT_NORM[] = {{4, 16, 4, 0, {1, 1, 1, 1}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_AGGCF_FLT_NORM[] = {{5, 16, 4, 0, {1, 1, 1, 1}}, {5, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_AGGCF_FLT_NORM[] = {{6, 16, 4, 0, {1, 1, 1, 1}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_AGGCF_FLT_NORM[] = {{7, 16, 4, 0, {1, 1, 1, 1}}, {7, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_AGGCF_FLT_NORM[] = {{8, 16, 4, 0, {1, 1, 1, 1}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_AGGCF_FLT_NORM[] = {{9, 16, 4, 0, {1, 1, 1, 1}}, {9, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_AGGCF_FLT_NORM[] = {{10, 16, 4, 0, {1, 1, 1, 1}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_AGGCF_FLT_NORM[] = {{11, 16, 4, 0, {1, 1, 1, 1}}, {11, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_AGGCF_FLT_NORM[] = {{12, 16, 4, 0, {1, 1, 1, 1}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_AGGCF_FLT_NORM[] = {{13, 16, 4, 0, {1, 1, 1, 1}}, {13, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_AGGCF_FLT_NORM[] = {{14, 16, 4, 0, {1, 1, 1, 1}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_AGGCF_FLT_NORM[] = {{15, 16, 4, 0, {1, 1, 1, 1}}, {15, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_AGGCF_FLT_NORM[] = {{16, 16, 4, 0, {1, 1, 1, 1}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_AGGCF_FLT_NORM[] = {{1, 18, 4, 0, {1, 1, 1, 1}}, {1, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_AGGCF_FLT_NORM[] = {{2, 18, 4, 0, {1, 1, 1, 1}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_AGGCF_FLT_NORM[] = {{3, 18, 4, 0, {1, 1, 1, 1}}, {3, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_AGGCF_FLT_NORM[] = {{4, 18, 4, 0, {1, 1, 1, 1}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_AGGCF_FLT_NORM[] = {{5, 18, 4, 0, {1, 1, 1, 1}}, {5, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_AGGCF_FLT_NORM[] = {{6, 18, 4, 0, {1, 1, 1, 1}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_AGGCF_FLT_NORM[] = {{7, 18, 4, 0, {1, 1, 1, 1}}, {7, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_AGGCF_FLT_NORM[] = {{8, 18, 4, 0, {1, 1, 1, 1}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_AGGCF_FLT_NORM[] = {{9, 18, 4, 0, {1, 1, 1, 1}}, {9, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_AGGCF_FLT_NORM[] = {{10, 18, 4, 0, {1, 1, 1, 1}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_AGGCF_FLT_NORM[] = {{11, 18, 4, 0, {1, 1, 1, 1}}, {11, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_AGGCF_FLT_NORM[] = {{12, 18, 4, 0, {1, 1, 1, 1}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_AGGCF_FLT_NORM[] = {{13, 18, 4, 0, {1, 1, 1, 1}}, {13, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_AGGCF_FLT_NORM[] = {{14, 18, 4, 0, {1, 1, 1, 1}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_AGGCF_FLT_NORM[] = {{15, 18, 4, 0, {1, 1, 1, 1}}, {15, 18, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_AGGCF_FLT_NORM[] = {{1, 20, 4, 0, {1, 1, 1, 1}}, {1, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_AGGCF_FLT_NORM[] = {{2, 20, 4, 0, {1, 1, 1, 1}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_AGGCF_FLT_NORM[] = {{3, 20, 4, 0, {1, 1, 1, 1}}, {3, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_AGGCF_FLT_NORM[] = {{4, 20, 4, 0, {1, 1, 1, 1}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_AGGCF_FLT_NORM[] = {{5, 20, 4, 0, {1, 1, 1, 1}}, {5, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_AGGCF_FLT_NORM[] = {{6, 20, 4, 0, {1, 1, 1, 1}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_AGGCF_FLT_NORM[] = {{7, 20, 4, 0, {1, 1, 1, 1}}, {7, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_AGGCF_FLT_NORM[] = {{8, 20, 4, 0, {1, 1, 1, 1}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_AGGCF_FLT_NORM[] = {{9, 20, 4, 0, {1, 1, 1, 1}}, {9, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_AGGCF_FLT_NORM[] = {{10, 20, 4, 0, {1, 1, 1, 1}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_AGGCF_FLT_NORM[] = {{11, 20, 4, 0, {1, 1, 1, 1}}, {11, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_AGGCF_FLT_NORM[] = {{12, 20, 4, 0, {1, 1, 1, 1}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_AGGCF_FLT_NORM[] = {{13, 20, 4, 0, {1, 1, 1, 1}}, {13, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_AGGCF_FLT_NORM[] = {{14, 20, 4, 0, {1, 1, 1, 1}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_AGGCF_FLT_NORM[] = {{1, 22, 4, 0, {1, 1, 1, 1}}, {1, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_AGGCF_FLT_NORM[] = {{2, 22, 4, 0, {1, 1, 1, 1}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_AGGCF_FLT_NORM[] = {{3, 22, 4, 0, {1, 1, 1, 1}}, {3, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_AGGCF_FLT_NORM[] = {{4, 22, 4, 0, {1, 1, 1, 1}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_AGGCF_FLT_NORM[] = {{5, 22, 4, 0, {1, 1, 1, 1}}, {5, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_AGGCF_FLT_NORM[] = {{6, 22, 4, 0, {1, 1, 1, 1}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_AGGCF_FLT_NORM[] = {{7, 22, 4, 0, {1, 1, 1, 1}}, {7, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_AGGCF_FLT_NORM[] = {{8, 22, 4, 0, {1, 1, 1, 1}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_AGGCF_FLT_NORM[] = {{9, 22, 4, 0, {1, 1, 1, 1}}, {9, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_AGGCF_FLT_NORM[] = {{10, 22, 4, 0, {1, 1, 1, 1}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_AGGCF_FLT_NORM[] = {{11, 22, 4, 0, {1, 1, 1, 1}}, {11, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_AGGCF_FLT_NORM[] = {{12, 22, 4, 0, {1, 1, 1, 1}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_AGGCF_FLT_NORM[] = {{1, 24, 4, 0, {1, 1, 1, 1}}, {1, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_AGGCF_FLT_NORM[] = {{2, 24, 4, 0, {1, 1, 1, 1}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_AGGCF_FLT_NORM[] = {{3, 24, 4, 0, {1, 1, 1, 1}}, {3, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_AGGCF_FLT_NORM[] = {{4, 24, 4, 0, {1, 1, 1, 1}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_AGGCF_FLT_NORM[] = {{5, 24, 4, 0, {1, 1, 1, 1}}, {5, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_AGGCF_FLT_NORM[] = {{6, 24, 4, 0, {1, 1, 1, 1}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_AGGCF_FLT_NORM[] = {{7, 24, 4, 0, {1, 1, 1, 1}}, {7, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_AGGCF_FLT_NORM[] = {{8, 24, 4, 0, {1, 1, 1, 1}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_AGGCF_FLT_NORM[] = {{9, 24, 4, 0, {1, 1, 1, 1}}, {9, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_AGGCF_FLT_NORM[] = {{10, 24, 4, 0, {1, 1, 1, 1}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_AGGCF_FLT_NORM[] = {{11, 24, 4, 0, {1, 1, 1, 1}}, {11, 24, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_AGGCF_FLT_NORM[] = {{1, 26, 4, 0, {1, 1, 1, 1}}, {1, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_AGGCF_FLT_NORM[] = {{2, 26, 4, 0, {1, 1, 1, 1}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_AGGCF_FLT_NORM[] = {{3, 26, 4, 0, {1, 1, 1, 1}}, {3, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_AGGCF_FLT_NORM[] = {{4, 26, 4, 0, {1, 1, 1, 1}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_AGGCF_FLT_NORM[] = {{5, 26, 4, 0, {1, 1, 1, 1}}, {5, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_AGGCF_FLT_NORM[] = {{6, 26, 4, 0, {1, 1, 1, 1}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_AGGCF_FLT_NORM[] = {{7, 26, 4, 0, {1, 1, 1, 1}}, {7, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_AGGCF_FLT_NORM[] = {{8, 26, 4, 0, {1, 1, 1, 1}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_AGGCF_FLT_NORM[] = {{9, 26, 4, 0, {1, 1, 1, 1}}, {9, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_AGGCF_FLT_NORM[] = {{10, 26, 4, 0, {1, 1, 1, 1}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_AGGCF_FLT_NORM[] = {{1, 28, 4, 0, {1, 1, 1, 1}}, {1, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_AGGCF_FLT_NORM[] = {{2, 28, 4, 0, {1, 1, 1, 1}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_AGGCF_FLT_NORM[] = {{3, 28, 4, 0, {1, 1, 1, 1}}, {3, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_AGGCF_FLT_NORM[] = {{4, 28, 4, 0, {1, 1, 1, 1}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_AGGCF_FLT_NORM[] = {{5, 28, 4, 0, {1, 1, 1, 1}}, {5, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_AGGCF_FLT_NORM[] = {{6, 28, 4, 0, {1, 1, 1, 1}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_AGGCF_FLT_NORM[] = {{7, 28, 4, 0, {1, 1, 1, 1}}, {7, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_AGGCF_FLT_NORM[] = {{8, 28, 4, 0, {1, 1, 1, 1}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_AGGCF_FLT_NORM[] = {{9, 28, 4, 0, {1, 1, 1, 1}}, {9, 28, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_AGGCF_FLT_NORM[] = {{1, 30, 4, 0, {1, 1, 1, 1}}, {1, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_AGGCF_FLT_NORM[] = {{2, 30, 4, 0, {1, 1, 1, 1}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_AGGCF_FLT_NORM[] = {{3, 30, 4, 0, {1, 1, 1, 1}}, {3, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_AGGCF_FLT_NORM[] = {{4, 30, 4, 0, {1, 1, 1, 1}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_AGGCF_FLT_NORM[] = {{5, 30, 4, 0, {1, 1, 1, 1}}, {5, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_AGGCF_FLT_NORM[] = {{6, 30, 4, 0, {1, 1, 1, 1}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_AGGCF_FLT_NORM[] = {{7, 30, 4, 0, {1, 1, 1, 1}}, {7, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_AGGCF_FLT_NORM[] = {{8, 30, 4, 0, {1, 1, 1, 1}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_AGGCF_FLT_NORM[] = {{9, 30, 4, 0, {1, 1, 1, 1}}, {9, 30, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_AGGCF_FLT_NORM[] = {{1, 32, 4, 0, {1, 1, 1, 1}}, {1, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_AGGCF_FLT_NORM[] = {{2, 32, 4, 0, {1, 1, 1, 1}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_AGGCF_FLT_NORM[] = {{3, 32, 4, 0, {1, 1, 1, 1}}, {3, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_AGGCF_FLT_NORM[] = {{4, 32, 4, 0, {1, 1, 1, 1}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_AGGCF_FLT_NORM[] = {{5, 32, 4, 0, {1, 1, 1, 1}}, {5, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_AGGCF_FLT_NORM[] = {{6, 32, 4, 0, {1, 1, 1, 1}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_AGGCF_FLT_NORM[] = {{7, 32, 4, 0, {1, 1, 1, 1}}, {7, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_AGGCF_FLT_NORM[] = {{8, 32, 4, 0, {1, 1, 1, 1}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {16, 1, 1, 1, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_AGGCF_FLT_NORM[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_AGGCF_FLT_NORM[] = {{1, 0}};

static acf_scenario gScenarioArray_AGGCF_FLT_NORM[] = {
{1, 1, 104, 16, 1, gScenarioBufferData0_AGGCF_FLT_NORM, 64, gScenarioKernelData0_AGGCF_FLT_NORM, 4},
{2, 1, 120, 16, 1, gScenarioBufferData1_AGGCF_FLT_NORM, 64, gScenarioKernelData1_AGGCF_FLT_NORM, 4},
{3, 1, 136, 16, 1, gScenarioBufferData2_AGGCF_FLT_NORM, 64, gScenarioKernelData2_AGGCF_FLT_NORM, 4},
{4, 1, 152, 16, 1, gScenarioBufferData3_AGGCF_FLT_NORM, 64, gScenarioKernelData3_AGGCF_FLT_NORM, 4},
{5, 1, 168, 16, 1, gScenarioBufferData4_AGGCF_FLT_NORM, 64, gScenarioKernelData4_AGGCF_FLT_NORM, 4},
{6, 1, 184, 16, 1, gScenarioBufferData5_AGGCF_FLT_NORM, 64, gScenarioKernelData5_AGGCF_FLT_NORM, 4},
{7, 1, 200, 16, 1, gScenarioBufferData6_AGGCF_FLT_NORM, 64, gScenarioKernelData6_AGGCF_FLT_NORM, 4},
{8, 1, 216, 16, 1, gScenarioBufferData7_AGGCF_FLT_NORM, 64, gScenarioKernelData7_AGGCF_FLT_NORM, 4},
{9, 1, 232, 16, 1, gScenarioBufferData8_AGGCF_FLT_NORM, 64, gScenarioKernelData8_AGGCF_FLT_NORM, 4},
{10, 1, 248, 16, 1, gScenarioBufferData9_AGGCF_FLT_NORM, 64, gScenarioKernelData9_AGGCF_FLT_NORM, 4},
{11, 1, 264, 16, 1, gScenarioBufferData10_AGGCF_FLT_NORM, 64, gScenarioKernelData10_AGGCF_FLT_NORM, 4},
{12, 1, 280, 16, 1, gScenarioBufferData11_AGGCF_FLT_NORM, 64, gScenarioKernelData11_AGGCF_FLT_NORM, 4},
{13, 1, 296, 16, 1, gScenarioBufferData12_AGGCF_FLT_NORM, 64, gScenarioKernelData12_AGGCF_FLT_NORM, 4},
{14, 1, 312, 16, 1, gScenarioBufferData13_AGGCF_FLT_NORM, 64, gScenarioKernelData13_AGGCF_FLT_NORM, 4},
{15, 1, 328, 16, 1, gScenarioBufferData14_AGGCF_FLT_NORM, 64, gScenarioKernelData14_AGGCF_FLT_NORM, 4},
{16, 1, 344, 16, 1, gScenarioBufferData15_AGGCF_FLT_NORM, 64, gScenarioKernelData15_AGGCF_FLT_NORM, 4},
{18, 1, 376, 16, 1, gScenarioBufferData16_AGGCF_FLT_NORM, 64, gScenarioKernelData16_AGGCF_FLT_NORM, 4},
{20, 1, 408, 16, 1, gScenarioBufferData17_AGGCF_FLT_NORM, 64, gScenarioKernelData17_AGGCF_FLT_NORM, 4},
{22, 1, 440, 16, 1, gScenarioBufferData18_AGGCF_FLT_NORM, 64, gScenarioKernelData18_AGGCF_FLT_NORM, 4},
{24, 1, 472, 16, 1, gScenarioBufferData19_AGGCF_FLT_NORM, 64, gScenarioKernelData19_AGGCF_FLT_NORM, 4},
{26, 1, 504, 16, 1, gScenarioBufferData20_AGGCF_FLT_NORM, 64, gScenarioKernelData20_AGGCF_FLT_NORM, 4},
{28, 1, 536, 16, 1, gScenarioBufferData21_AGGCF_FLT_NORM, 64, gScenarioKernelData21_AGGCF_FLT_NORM, 4},
{30, 1, 568, 16, 1, gScenarioBufferData22_AGGCF_FLT_NORM, 64, gScenarioKernelData22_AGGCF_FLT_NORM, 4},
{32, 1, 600, 16, 1, gScenarioBufferData23_AGGCF_FLT_NORM, 64, gScenarioKernelData23_AGGCF_FLT_NORM, 4},
{36, 1, 664, 16, 1, gScenarioBufferData24_AGGCF_FLT_NORM, 64, gScenarioKernelData24_AGGCF_FLT_NORM, 4},
{40, 1, 728, 16, 1, gScenarioBufferData25_AGGCF_FLT_NORM, 64, gScenarioKernelData25_AGGCF_FLT_NORM, 4},
{44, 1, 792, 16, 1, gScenarioBufferData26_AGGCF_FLT_NORM, 64, gScenarioKernelData26_AGGCF_FLT_NORM, 4},
{48, 1, 856, 16, 1, gScenarioBufferData27_AGGCF_FLT_NORM, 64, gScenarioKernelData27_AGGCF_FLT_NORM, 4},
{52, 1, 920, 16, 1, gScenarioBufferData28_AGGCF_FLT_NORM, 64, gScenarioKernelData28_AGGCF_FLT_NORM, 4},
{56, 1, 984, 16, 1, gScenarioBufferData29_AGGCF_FLT_NORM, 64, gScenarioKernelData29_AGGCF_FLT_NORM, 4},
{60, 1, 1048, 16, 1, gScenarioBufferData30_AGGCF_FLT_NORM, 64, gScenarioKernelData30_AGGCF_FLT_NORM, 4},
{64, 1, 1112, 16, 1, gScenarioBufferData31_AGGCF_FLT_NORM, 64, gScenarioKernelData31_AGGCF_FLT_NORM, 4},
{1, 2, 132, 16, 1, gScenarioBufferData32_AGGCF_FLT_NORM, 64, gScenarioKernelData32_AGGCF_FLT_NORM, 4},
{2, 2, 160, 16, 1, gScenarioBufferData33_AGGCF_FLT_NORM, 64, gScenarioKernelData33_AGGCF_FLT_NORM, 4},
{3, 2, 188, 16, 1, gScenarioBufferData34_AGGCF_FLT_NORM, 64, gScenarioKernelData34_AGGCF_FLT_NORM, 4},
{4, 2, 216, 16, 1, gScenarioBufferData35_AGGCF_FLT_NORM, 64, gScenarioKernelData35_AGGCF_FLT_NORM, 4},
{5, 2, 244, 16, 1, gScenarioBufferData36_AGGCF_FLT_NORM, 64, gScenarioKernelData36_AGGCF_FLT_NORM, 4},
{6, 2, 272, 16, 1, gScenarioBufferData37_AGGCF_FLT_NORM, 64, gScenarioKernelData37_AGGCF_FLT_NORM, 4},
{7, 2, 300, 16, 1, gScenarioBufferData38_AGGCF_FLT_NORM, 64, gScenarioKernelData38_AGGCF_FLT_NORM, 4},
{8, 2, 328, 16, 1, gScenarioBufferData39_AGGCF_FLT_NORM, 64, gScenarioKernelData39_AGGCF_FLT_NORM, 4},
{9, 2, 356, 16, 1, gScenarioBufferData40_AGGCF_FLT_NORM, 64, gScenarioKernelData40_AGGCF_FLT_NORM, 4},
{10, 2, 384, 16, 1, gScenarioBufferData41_AGGCF_FLT_NORM, 64, gScenarioKernelData41_AGGCF_FLT_NORM, 4},
{11, 2, 412, 16, 1, gScenarioBufferData42_AGGCF_FLT_NORM, 64, gScenarioKernelData42_AGGCF_FLT_NORM, 4},
{12, 2, 440, 16, 1, gScenarioBufferData43_AGGCF_FLT_NORM, 64, gScenarioKernelData43_AGGCF_FLT_NORM, 4},
{13, 2, 468, 16, 1, gScenarioBufferData44_AGGCF_FLT_NORM, 64, gScenarioKernelData44_AGGCF_FLT_NORM, 4},
{14, 2, 496, 16, 1, gScenarioBufferData45_AGGCF_FLT_NORM, 64, gScenarioKernelData45_AGGCF_FLT_NORM, 4},
{15, 2, 524, 16, 1, gScenarioBufferData46_AGGCF_FLT_NORM, 64, gScenarioKernelData46_AGGCF_FLT_NORM, 4},
{16, 2, 552, 16, 1, gScenarioBufferData47_AGGCF_FLT_NORM, 64, gScenarioKernelData47_AGGCF_FLT_NORM, 4},
{18, 2, 608, 16, 1, gScenarioBufferData48_AGGCF_FLT_NORM, 64, gScenarioKernelData48_AGGCF_FLT_NORM, 4},
{20, 2, 664, 16, 1, gScenarioBufferData49_AGGCF_FLT_NORM, 64, gScenarioKernelData49_AGGCF_FLT_NORM, 4},
{22, 2, 720, 16, 1, gScenarioBufferData50_AGGCF_FLT_NORM, 64, gScenarioKernelData50_AGGCF_FLT_NORM, 4},
{24, 2, 776, 16, 1, gScenarioBufferData51_AGGCF_FLT_NORM, 64, gScenarioKernelData51_AGGCF_FLT_NORM, 4},
{26, 2, 832, 16, 1, gScenarioBufferData52_AGGCF_FLT_NORM, 64, gScenarioKernelData52_AGGCF_FLT_NORM, 4},
{28, 2, 888, 16, 1, gScenarioBufferData53_AGGCF_FLT_NORM, 64, gScenarioKernelData53_AGGCF_FLT_NORM, 4},
{30, 2, 944, 16, 1, gScenarioBufferData54_AGGCF_FLT_NORM, 64, gScenarioKernelData54_AGGCF_FLT_NORM, 4},
{32, 2, 1000, 16, 1, gScenarioBufferData55_AGGCF_FLT_NORM, 64, gScenarioKernelData55_AGGCF_FLT_NORM, 4},
{36, 2, 1112, 16, 1, gScenarioBufferData56_AGGCF_FLT_NORM, 64, gScenarioKernelData56_AGGCF_FLT_NORM, 4},
{40, 2, 1224, 16, 1, gScenarioBufferData57_AGGCF_FLT_NORM, 64, gScenarioKernelData57_AGGCF_FLT_NORM, 4},
{44, 2, 1336, 16, 1, gScenarioBufferData58_AGGCF_FLT_NORM, 64, gScenarioKernelData58_AGGCF_FLT_NORM, 4},
{48, 2, 1448, 16, 1, gScenarioBufferData59_AGGCF_FLT_NORM, 64, gScenarioKernelData59_AGGCF_FLT_NORM, 4},
{52, 2, 1560, 16, 1, gScenarioBufferData60_AGGCF_FLT_NORM, 64, gScenarioKernelData60_AGGCF_FLT_NORM, 4},
{56, 2, 1672, 16, 1, gScenarioBufferData61_AGGCF_FLT_NORM, 64, gScenarioKernelData61_AGGCF_FLT_NORM, 4},
{60, 2, 1784, 16, 1, gScenarioBufferData62_AGGCF_FLT_NORM, 64, gScenarioKernelData62_AGGCF_FLT_NORM, 4},
{64, 2, 1896, 16, 1, gScenarioBufferData63_AGGCF_FLT_NORM, 64, gScenarioKernelData63_AGGCF_FLT_NORM, 4},
{1, 3, 160, 16, 1, gScenarioBufferData64_AGGCF_FLT_NORM, 64, gScenarioKernelData64_AGGCF_FLT_NORM, 4},
{2, 3, 200, 16, 1, gScenarioBufferData65_AGGCF_FLT_NORM, 64, gScenarioKernelData65_AGGCF_FLT_NORM, 4},
{3, 3, 240, 16, 1, gScenarioBufferData66_AGGCF_FLT_NORM, 64, gScenarioKernelData66_AGGCF_FLT_NORM, 4},
{4, 3, 280, 16, 1, gScenarioBufferData67_AGGCF_FLT_NORM, 64, gScenarioKernelData67_AGGCF_FLT_NORM, 4},
{5, 3, 320, 16, 1, gScenarioBufferData68_AGGCF_FLT_NORM, 64, gScenarioKernelData68_AGGCF_FLT_NORM, 4},
{6, 3, 360, 16, 1, gScenarioBufferData69_AGGCF_FLT_NORM, 64, gScenarioKernelData69_AGGCF_FLT_NORM, 4},
{7, 3, 400, 16, 1, gScenarioBufferData70_AGGCF_FLT_NORM, 64, gScenarioKernelData70_AGGCF_FLT_NORM, 4},
{8, 3, 440, 16, 1, gScenarioBufferData71_AGGCF_FLT_NORM, 64, gScenarioKernelData71_AGGCF_FLT_NORM, 4},
{9, 3, 480, 16, 1, gScenarioBufferData72_AGGCF_FLT_NORM, 64, gScenarioKernelData72_AGGCF_FLT_NORM, 4},
{10, 3, 520, 16, 1, gScenarioBufferData73_AGGCF_FLT_NORM, 64, gScenarioKernelData73_AGGCF_FLT_NORM, 4},
{11, 3, 560, 16, 1, gScenarioBufferData74_AGGCF_FLT_NORM, 64, gScenarioKernelData74_AGGCF_FLT_NORM, 4},
{12, 3, 600, 16, 1, gScenarioBufferData75_AGGCF_FLT_NORM, 64, gScenarioKernelData75_AGGCF_FLT_NORM, 4},
{13, 3, 640, 16, 1, gScenarioBufferData76_AGGCF_FLT_NORM, 64, gScenarioKernelData76_AGGCF_FLT_NORM, 4},
{14, 3, 680, 16, 1, gScenarioBufferData77_AGGCF_FLT_NORM, 64, gScenarioKernelData77_AGGCF_FLT_NORM, 4},
{15, 3, 720, 16, 1, gScenarioBufferData78_AGGCF_FLT_NORM, 64, gScenarioKernelData78_AGGCF_FLT_NORM, 4},
{16, 3, 760, 16, 1, gScenarioBufferData79_AGGCF_FLT_NORM, 64, gScenarioKernelData79_AGGCF_FLT_NORM, 4},
{18, 3, 840, 16, 1, gScenarioBufferData80_AGGCF_FLT_NORM, 64, gScenarioKernelData80_AGGCF_FLT_NORM, 4},
{20, 3, 920, 16, 1, gScenarioBufferData81_AGGCF_FLT_NORM, 64, gScenarioKernelData81_AGGCF_FLT_NORM, 4},
{22, 3, 1000, 16, 1, gScenarioBufferData82_AGGCF_FLT_NORM, 64, gScenarioKernelData82_AGGCF_FLT_NORM, 4},
{24, 3, 1080, 16, 1, gScenarioBufferData83_AGGCF_FLT_NORM, 64, gScenarioKernelData83_AGGCF_FLT_NORM, 4},
{26, 3, 1160, 16, 1, gScenarioBufferData84_AGGCF_FLT_NORM, 64, gScenarioKernelData84_AGGCF_FLT_NORM, 4},
{28, 3, 1240, 16, 1, gScenarioBufferData85_AGGCF_FLT_NORM, 64, gScenarioKernelData85_AGGCF_FLT_NORM, 4},
{30, 3, 1320, 16, 1, gScenarioBufferData86_AGGCF_FLT_NORM, 64, gScenarioKernelData86_AGGCF_FLT_NORM, 4},
{32, 3, 1400, 16, 1, gScenarioBufferData87_AGGCF_FLT_NORM, 64, gScenarioKernelData87_AGGCF_FLT_NORM, 4},
{36, 3, 1560, 16, 1, gScenarioBufferData88_AGGCF_FLT_NORM, 64, gScenarioKernelData88_AGGCF_FLT_NORM, 4},
{40, 3, 1720, 16, 1, gScenarioBufferData89_AGGCF_FLT_NORM, 64, gScenarioKernelData89_AGGCF_FLT_NORM, 4},
{44, 3, 1880, 16, 1, gScenarioBufferData90_AGGCF_FLT_NORM, 64, gScenarioKernelData90_AGGCF_FLT_NORM, 4},
{48, 3, 2040, 16, 1, gScenarioBufferData91_AGGCF_FLT_NORM, 64, gScenarioKernelData91_AGGCF_FLT_NORM, 4},
{52, 3, 2200, 16, 1, gScenarioBufferData92_AGGCF_FLT_NORM, 64, gScenarioKernelData92_AGGCF_FLT_NORM, 4},
{56, 3, 2360, 16, 1, gScenarioBufferData93_AGGCF_FLT_NORM, 64, gScenarioKernelData93_AGGCF_FLT_NORM, 4},
{60, 3, 2520, 16, 1, gScenarioBufferData94_AGGCF_FLT_NORM, 64, gScenarioKernelData94_AGGCF_FLT_NORM, 4},
{64, 3, 2680, 16, 1, gScenarioBufferData95_AGGCF_FLT_NORM, 64, gScenarioKernelData95_AGGCF_FLT_NORM, 4},
{1, 4, 188, 16, 1, gScenarioBufferData96_AGGCF_FLT_NORM, 64, gScenarioKernelData96_AGGCF_FLT_NORM, 4},
{2, 4, 240, 16, 1, gScenarioBufferData97_AGGCF_FLT_NORM, 64, gScenarioKernelData97_AGGCF_FLT_NORM, 4},
{3, 4, 292, 16, 1, gScenarioBufferData98_AGGCF_FLT_NORM, 64, gScenarioKernelData98_AGGCF_FLT_NORM, 4},
{4, 4, 344, 16, 1, gScenarioBufferData99_AGGCF_FLT_NORM, 64, gScenarioKernelData99_AGGCF_FLT_NORM, 4},
{5, 4, 396, 16, 1, gScenarioBufferData100_AGGCF_FLT_NORM, 64, gScenarioKernelData100_AGGCF_FLT_NORM, 4},
{6, 4, 448, 16, 1, gScenarioBufferData101_AGGCF_FLT_NORM, 64, gScenarioKernelData101_AGGCF_FLT_NORM, 4},
{7, 4, 500, 16, 1, gScenarioBufferData102_AGGCF_FLT_NORM, 64, gScenarioKernelData102_AGGCF_FLT_NORM, 4},
{8, 4, 552, 16, 1, gScenarioBufferData103_AGGCF_FLT_NORM, 64, gScenarioKernelData103_AGGCF_FLT_NORM, 4},
{9, 4, 604, 16, 1, gScenarioBufferData104_AGGCF_FLT_NORM, 64, gScenarioKernelData104_AGGCF_FLT_NORM, 4},
{10, 4, 656, 16, 1, gScenarioBufferData105_AGGCF_FLT_NORM, 64, gScenarioKernelData105_AGGCF_FLT_NORM, 4},
{11, 4, 708, 16, 1, gScenarioBufferData106_AGGCF_FLT_NORM, 64, gScenarioKernelData106_AGGCF_FLT_NORM, 4},
{12, 4, 760, 16, 1, gScenarioBufferData107_AGGCF_FLT_NORM, 64, gScenarioKernelData107_AGGCF_FLT_NORM, 4},
{13, 4, 812, 16, 1, gScenarioBufferData108_AGGCF_FLT_NORM, 64, gScenarioKernelData108_AGGCF_FLT_NORM, 4},
{14, 4, 864, 16, 1, gScenarioBufferData109_AGGCF_FLT_NORM, 64, gScenarioKernelData109_AGGCF_FLT_NORM, 4},
{15, 4, 916, 16, 1, gScenarioBufferData110_AGGCF_FLT_NORM, 64, gScenarioKernelData110_AGGCF_FLT_NORM, 4},
{16, 4, 968, 16, 1, gScenarioBufferData111_AGGCF_FLT_NORM, 64, gScenarioKernelData111_AGGCF_FLT_NORM, 4},
{18, 4, 1072, 16, 1, gScenarioBufferData112_AGGCF_FLT_NORM, 64, gScenarioKernelData112_AGGCF_FLT_NORM, 4},
{20, 4, 1176, 16, 1, gScenarioBufferData113_AGGCF_FLT_NORM, 64, gScenarioKernelData113_AGGCF_FLT_NORM, 4},
{22, 4, 1280, 16, 1, gScenarioBufferData114_AGGCF_FLT_NORM, 64, gScenarioKernelData114_AGGCF_FLT_NORM, 4},
{24, 4, 1384, 16, 1, gScenarioBufferData115_AGGCF_FLT_NORM, 64, gScenarioKernelData115_AGGCF_FLT_NORM, 4},
{26, 4, 1488, 16, 1, gScenarioBufferData116_AGGCF_FLT_NORM, 64, gScenarioKernelData116_AGGCF_FLT_NORM, 4},
{28, 4, 1592, 16, 1, gScenarioBufferData117_AGGCF_FLT_NORM, 64, gScenarioKernelData117_AGGCF_FLT_NORM, 4},
{30, 4, 1696, 16, 1, gScenarioBufferData118_AGGCF_FLT_NORM, 64, gScenarioKernelData118_AGGCF_FLT_NORM, 4},
{32, 4, 1800, 16, 1, gScenarioBufferData119_AGGCF_FLT_NORM, 64, gScenarioKernelData119_AGGCF_FLT_NORM, 4},
{36, 4, 2008, 16, 1, gScenarioBufferData120_AGGCF_FLT_NORM, 64, gScenarioKernelData120_AGGCF_FLT_NORM, 4},
{40, 4, 2216, 16, 1, gScenarioBufferData121_AGGCF_FLT_NORM, 64, gScenarioKernelData121_AGGCF_FLT_NORM, 4},
{44, 4, 2424, 16, 1, gScenarioBufferData122_AGGCF_FLT_NORM, 64, gScenarioKernelData122_AGGCF_FLT_NORM, 4},
{48, 4, 2632, 16, 1, gScenarioBufferData123_AGGCF_FLT_NORM, 64, gScenarioKernelData123_AGGCF_FLT_NORM, 4},
{52, 4, 2840, 16, 1, gScenarioBufferData124_AGGCF_FLT_NORM, 64, gScenarioKernelData124_AGGCF_FLT_NORM, 4},
{56, 4, 3048, 16, 1, gScenarioBufferData125_AGGCF_FLT_NORM, 64, gScenarioKernelData125_AGGCF_FLT_NORM, 4},
{60, 4, 3256, 16, 1, gScenarioBufferData126_AGGCF_FLT_NORM, 64, gScenarioKernelData126_AGGCF_FLT_NORM, 4},
{64, 4, 3464, 16, 1, gScenarioBufferData127_AGGCF_FLT_NORM, 64, gScenarioKernelData127_AGGCF_FLT_NORM, 4},
{1, 5, 216, 16, 1, gScenarioBufferData128_AGGCF_FLT_NORM, 64, gScenarioKernelData128_AGGCF_FLT_NORM, 4},
{2, 5, 280, 16, 1, gScenarioBufferData129_AGGCF_FLT_NORM, 64, gScenarioKernelData129_AGGCF_FLT_NORM, 4},
{3, 5, 344, 16, 1, gScenarioBufferData130_AGGCF_FLT_NORM, 64, gScenarioKernelData130_AGGCF_FLT_NORM, 4},
{4, 5, 408, 16, 1, gScenarioBufferData131_AGGCF_FLT_NORM, 64, gScenarioKernelData131_AGGCF_FLT_NORM, 4},
{5, 5, 472, 16, 1, gScenarioBufferData132_AGGCF_FLT_NORM, 64, gScenarioKernelData132_AGGCF_FLT_NORM, 4},
{6, 5, 536, 16, 1, gScenarioBufferData133_AGGCF_FLT_NORM, 64, gScenarioKernelData133_AGGCF_FLT_NORM, 4},
{7, 5, 600, 16, 1, gScenarioBufferData134_AGGCF_FLT_NORM, 64, gScenarioKernelData134_AGGCF_FLT_NORM, 4},
{8, 5, 664, 16, 1, gScenarioBufferData135_AGGCF_FLT_NORM, 64, gScenarioKernelData135_AGGCF_FLT_NORM, 4},
{9, 5, 728, 16, 1, gScenarioBufferData136_AGGCF_FLT_NORM, 64, gScenarioKernelData136_AGGCF_FLT_NORM, 4},
{10, 5, 792, 16, 1, gScenarioBufferData137_AGGCF_FLT_NORM, 64, gScenarioKernelData137_AGGCF_FLT_NORM, 4},
{11, 5, 856, 16, 1, gScenarioBufferData138_AGGCF_FLT_NORM, 64, gScenarioKernelData138_AGGCF_FLT_NORM, 4},
{12, 5, 920, 16, 1, gScenarioBufferData139_AGGCF_FLT_NORM, 64, gScenarioKernelData139_AGGCF_FLT_NORM, 4},
{13, 5, 984, 16, 1, gScenarioBufferData140_AGGCF_FLT_NORM, 64, gScenarioKernelData140_AGGCF_FLT_NORM, 4},
{14, 5, 1048, 16, 1, gScenarioBufferData141_AGGCF_FLT_NORM, 64, gScenarioKernelData141_AGGCF_FLT_NORM, 4},
{15, 5, 1112, 16, 1, gScenarioBufferData142_AGGCF_FLT_NORM, 64, gScenarioKernelData142_AGGCF_FLT_NORM, 4},
{16, 5, 1176, 16, 1, gScenarioBufferData143_AGGCF_FLT_NORM, 64, gScenarioKernelData143_AGGCF_FLT_NORM, 4},
{18, 5, 1304, 16, 1, gScenarioBufferData144_AGGCF_FLT_NORM, 64, gScenarioKernelData144_AGGCF_FLT_NORM, 4},
{20, 5, 1432, 16, 1, gScenarioBufferData145_AGGCF_FLT_NORM, 64, gScenarioKernelData145_AGGCF_FLT_NORM, 4},
{22, 5, 1560, 16, 1, gScenarioBufferData146_AGGCF_FLT_NORM, 64, gScenarioKernelData146_AGGCF_FLT_NORM, 4},
{24, 5, 1688, 16, 1, gScenarioBufferData147_AGGCF_FLT_NORM, 64, gScenarioKernelData147_AGGCF_FLT_NORM, 4},
{26, 5, 1816, 16, 1, gScenarioBufferData148_AGGCF_FLT_NORM, 64, gScenarioKernelData148_AGGCF_FLT_NORM, 4},
{28, 5, 1944, 16, 1, gScenarioBufferData149_AGGCF_FLT_NORM, 64, gScenarioKernelData149_AGGCF_FLT_NORM, 4},
{30, 5, 2072, 16, 1, gScenarioBufferData150_AGGCF_FLT_NORM, 64, gScenarioKernelData150_AGGCF_FLT_NORM, 4},
{32, 5, 2200, 16, 1, gScenarioBufferData151_AGGCF_FLT_NORM, 64, gScenarioKernelData151_AGGCF_FLT_NORM, 4},
{36, 5, 2456, 16, 1, gScenarioBufferData152_AGGCF_FLT_NORM, 64, gScenarioKernelData152_AGGCF_FLT_NORM, 4},
{40, 5, 2712, 16, 1, gScenarioBufferData153_AGGCF_FLT_NORM, 64, gScenarioKernelData153_AGGCF_FLT_NORM, 4},
{44, 5, 2968, 16, 1, gScenarioBufferData154_AGGCF_FLT_NORM, 64, gScenarioKernelData154_AGGCF_FLT_NORM, 4},
{48, 5, 3224, 16, 1, gScenarioBufferData155_AGGCF_FLT_NORM, 64, gScenarioKernelData155_AGGCF_FLT_NORM, 4},
{52, 5, 3480, 16, 1, gScenarioBufferData156_AGGCF_FLT_NORM, 64, gScenarioKernelData156_AGGCF_FLT_NORM, 4},
{56, 5, 3736, 16, 1, gScenarioBufferData157_AGGCF_FLT_NORM, 64, gScenarioKernelData157_AGGCF_FLT_NORM, 4},
{1, 6, 244, 16, 1, gScenarioBufferData158_AGGCF_FLT_NORM, 64, gScenarioKernelData158_AGGCF_FLT_NORM, 4},
{2, 6, 320, 16, 1, gScenarioBufferData159_AGGCF_FLT_NORM, 64, gScenarioKernelData159_AGGCF_FLT_NORM, 4},
{3, 6, 396, 16, 1, gScenarioBufferData160_AGGCF_FLT_NORM, 64, gScenarioKernelData160_AGGCF_FLT_NORM, 4},
{4, 6, 472, 16, 1, gScenarioBufferData161_AGGCF_FLT_NORM, 64, gScenarioKernelData161_AGGCF_FLT_NORM, 4},
{5, 6, 548, 16, 1, gScenarioBufferData162_AGGCF_FLT_NORM, 64, gScenarioKernelData162_AGGCF_FLT_NORM, 4},
{6, 6, 624, 16, 1, gScenarioBufferData163_AGGCF_FLT_NORM, 64, gScenarioKernelData163_AGGCF_FLT_NORM, 4},
{7, 6, 700, 16, 1, gScenarioBufferData164_AGGCF_FLT_NORM, 64, gScenarioKernelData164_AGGCF_FLT_NORM, 4},
{8, 6, 776, 16, 1, gScenarioBufferData165_AGGCF_FLT_NORM, 64, gScenarioKernelData165_AGGCF_FLT_NORM, 4},
{9, 6, 852, 16, 1, gScenarioBufferData166_AGGCF_FLT_NORM, 64, gScenarioKernelData166_AGGCF_FLT_NORM, 4},
{10, 6, 928, 16, 1, gScenarioBufferData167_AGGCF_FLT_NORM, 64, gScenarioKernelData167_AGGCF_FLT_NORM, 4},
{11, 6, 1004, 16, 1, gScenarioBufferData168_AGGCF_FLT_NORM, 64, gScenarioKernelData168_AGGCF_FLT_NORM, 4},
{12, 6, 1080, 16, 1, gScenarioBufferData169_AGGCF_FLT_NORM, 64, gScenarioKernelData169_AGGCF_FLT_NORM, 4},
{13, 6, 1156, 16, 1, gScenarioBufferData170_AGGCF_FLT_NORM, 64, gScenarioKernelData170_AGGCF_FLT_NORM, 4},
{14, 6, 1232, 16, 1, gScenarioBufferData171_AGGCF_FLT_NORM, 64, gScenarioKernelData171_AGGCF_FLT_NORM, 4},
{15, 6, 1308, 16, 1, gScenarioBufferData172_AGGCF_FLT_NORM, 64, gScenarioKernelData172_AGGCF_FLT_NORM, 4},
{16, 6, 1384, 16, 1, gScenarioBufferData173_AGGCF_FLT_NORM, 64, gScenarioKernelData173_AGGCF_FLT_NORM, 4},
{18, 6, 1536, 16, 1, gScenarioBufferData174_AGGCF_FLT_NORM, 64, gScenarioKernelData174_AGGCF_FLT_NORM, 4},
{20, 6, 1688, 16, 1, gScenarioBufferData175_AGGCF_FLT_NORM, 64, gScenarioKernelData175_AGGCF_FLT_NORM, 4},
{22, 6, 1840, 16, 1, gScenarioBufferData176_AGGCF_FLT_NORM, 64, gScenarioKernelData176_AGGCF_FLT_NORM, 4},
{24, 6, 1992, 16, 1, gScenarioBufferData177_AGGCF_FLT_NORM, 64, gScenarioKernelData177_AGGCF_FLT_NORM, 4},
{26, 6, 2144, 16, 1, gScenarioBufferData178_AGGCF_FLT_NORM, 64, gScenarioKernelData178_AGGCF_FLT_NORM, 4},
{28, 6, 2296, 16, 1, gScenarioBufferData179_AGGCF_FLT_NORM, 64, gScenarioKernelData179_AGGCF_FLT_NORM, 4},
{30, 6, 2448, 16, 1, gScenarioBufferData180_AGGCF_FLT_NORM, 64, gScenarioKernelData180_AGGCF_FLT_NORM, 4},
{32, 6, 2600, 16, 1, gScenarioBufferData181_AGGCF_FLT_NORM, 64, gScenarioKernelData181_AGGCF_FLT_NORM, 4},
{36, 6, 2904, 16, 1, gScenarioBufferData182_AGGCF_FLT_NORM, 64, gScenarioKernelData182_AGGCF_FLT_NORM, 4},
{40, 6, 3208, 16, 1, gScenarioBufferData183_AGGCF_FLT_NORM, 64, gScenarioKernelData183_AGGCF_FLT_NORM, 4},
{44, 6, 3512, 16, 1, gScenarioBufferData184_AGGCF_FLT_NORM, 64, gScenarioKernelData184_AGGCF_FLT_NORM, 4},
{48, 6, 3816, 16, 1, gScenarioBufferData185_AGGCF_FLT_NORM, 64, gScenarioKernelData185_AGGCF_FLT_NORM, 4},
{1, 8, 300, 16, 1, gScenarioBufferData186_AGGCF_FLT_NORM, 64, gScenarioKernelData186_AGGCF_FLT_NORM, 4},
{2, 8, 400, 16, 1, gScenarioBufferData187_AGGCF_FLT_NORM, 64, gScenarioKernelData187_AGGCF_FLT_NORM, 4},
{3, 8, 500, 16, 1, gScenarioBufferData188_AGGCF_FLT_NORM, 64, gScenarioKernelData188_AGGCF_FLT_NORM, 4},
{4, 8, 600, 16, 1, gScenarioBufferData189_AGGCF_FLT_NORM, 64, gScenarioKernelData189_AGGCF_FLT_NORM, 4},
{5, 8, 700, 16, 1, gScenarioBufferData190_AGGCF_FLT_NORM, 64, gScenarioKernelData190_AGGCF_FLT_NORM, 4},
{6, 8, 800, 16, 1, gScenarioBufferData191_AGGCF_FLT_NORM, 64, gScenarioKernelData191_AGGCF_FLT_NORM, 4},
{7, 8, 900, 16, 1, gScenarioBufferData192_AGGCF_FLT_NORM, 64, gScenarioKernelData192_AGGCF_FLT_NORM, 4},
{8, 8, 1000, 16, 1, gScenarioBufferData193_AGGCF_FLT_NORM, 64, gScenarioKernelData193_AGGCF_FLT_NORM, 4},
{9, 8, 1100, 16, 1, gScenarioBufferData194_AGGCF_FLT_NORM, 64, gScenarioKernelData194_AGGCF_FLT_NORM, 4},
{10, 8, 1200, 16, 1, gScenarioBufferData195_AGGCF_FLT_NORM, 64, gScenarioKernelData195_AGGCF_FLT_NORM, 4},
{11, 8, 1300, 16, 1, gScenarioBufferData196_AGGCF_FLT_NORM, 64, gScenarioKernelData196_AGGCF_FLT_NORM, 4},
{12, 8, 1400, 16, 1, gScenarioBufferData197_AGGCF_FLT_NORM, 64, gScenarioKernelData197_AGGCF_FLT_NORM, 4},
{13, 8, 1500, 16, 1, gScenarioBufferData198_AGGCF_FLT_NORM, 64, gScenarioKernelData198_AGGCF_FLT_NORM, 4},
{14, 8, 1600, 16, 1, gScenarioBufferData199_AGGCF_FLT_NORM, 64, gScenarioKernelData199_AGGCF_FLT_NORM, 4},
{15, 8, 1700, 16, 1, gScenarioBufferData200_AGGCF_FLT_NORM, 64, gScenarioKernelData200_AGGCF_FLT_NORM, 4},
{16, 8, 1800, 16, 1, gScenarioBufferData201_AGGCF_FLT_NORM, 64, gScenarioKernelData201_AGGCF_FLT_NORM, 4},
{18, 8, 2000, 16, 1, gScenarioBufferData202_AGGCF_FLT_NORM, 64, gScenarioKernelData202_AGGCF_FLT_NORM, 4},
{20, 8, 2200, 16, 1, gScenarioBufferData203_AGGCF_FLT_NORM, 64, gScenarioKernelData203_AGGCF_FLT_NORM, 4},
{22, 8, 2400, 16, 1, gScenarioBufferData204_AGGCF_FLT_NORM, 64, gScenarioKernelData204_AGGCF_FLT_NORM, 4},
{24, 8, 2600, 16, 1, gScenarioBufferData205_AGGCF_FLT_NORM, 64, gScenarioKernelData205_AGGCF_FLT_NORM, 4},
{26, 8, 2800, 16, 1, gScenarioBufferData206_AGGCF_FLT_NORM, 64, gScenarioKernelData206_AGGCF_FLT_NORM, 4},
{28, 8, 3000, 16, 1, gScenarioBufferData207_AGGCF_FLT_NORM, 64, gScenarioKernelData207_AGGCF_FLT_NORM, 4},
{30, 8, 3200, 16, 1, gScenarioBufferData208_AGGCF_FLT_NORM, 64, gScenarioKernelData208_AGGCF_FLT_NORM, 4},
{32, 8, 3400, 16, 1, gScenarioBufferData209_AGGCF_FLT_NORM, 64, gScenarioKernelData209_AGGCF_FLT_NORM, 4},
{36, 8, 3800, 16, 1, gScenarioBufferData210_AGGCF_FLT_NORM, 64, gScenarioKernelData210_AGGCF_FLT_NORM, 4},
{1, 10, 356, 16, 1, gScenarioBufferData211_AGGCF_FLT_NORM, 64, gScenarioKernelData211_AGGCF_FLT_NORM, 4},
{2, 10, 480, 16, 1, gScenarioBufferData212_AGGCF_FLT_NORM, 64, gScenarioKernelData212_AGGCF_FLT_NORM, 4},
{3, 10, 604, 16, 1, gScenarioBufferData213_AGGCF_FLT_NORM, 64, gScenarioKernelData213_AGGCF_FLT_NORM, 4},
{4, 10, 728, 16, 1, gScenarioBufferData214_AGGCF_FLT_NORM, 64, gScenarioKernelData214_AGGCF_FLT_NORM, 4},
{5, 10, 852, 16, 1, gScenarioBufferData215_AGGCF_FLT_NORM, 64, gScenarioKernelData215_AGGCF_FLT_NORM, 4},
{6, 10, 976, 16, 1, gScenarioBufferData216_AGGCF_FLT_NORM, 64, gScenarioKernelData216_AGGCF_FLT_NORM, 4},
{7, 10, 1100, 16, 1, gScenarioBufferData217_AGGCF_FLT_NORM, 64, gScenarioKernelData217_AGGCF_FLT_NORM, 4},
{8, 10, 1224, 16, 1, gScenarioBufferData218_AGGCF_FLT_NORM, 64, gScenarioKernelData218_AGGCF_FLT_NORM, 4},
{9, 10, 1348, 16, 1, gScenarioBufferData219_AGGCF_FLT_NORM, 64, gScenarioKernelData219_AGGCF_FLT_NORM, 4},
{10, 10, 1472, 16, 1, gScenarioBufferData220_AGGCF_FLT_NORM, 64, gScenarioKernelData220_AGGCF_FLT_NORM, 4},
{11, 10, 1596, 16, 1, gScenarioBufferData221_AGGCF_FLT_NORM, 64, gScenarioKernelData221_AGGCF_FLT_NORM, 4},
{12, 10, 1720, 16, 1, gScenarioBufferData222_AGGCF_FLT_NORM, 64, gScenarioKernelData222_AGGCF_FLT_NORM, 4},
{13, 10, 1844, 16, 1, gScenarioBufferData223_AGGCF_FLT_NORM, 64, gScenarioKernelData223_AGGCF_FLT_NORM, 4},
{14, 10, 1968, 16, 1, gScenarioBufferData224_AGGCF_FLT_NORM, 64, gScenarioKernelData224_AGGCF_FLT_NORM, 4},
{15, 10, 2092, 16, 1, gScenarioBufferData225_AGGCF_FLT_NORM, 64, gScenarioKernelData225_AGGCF_FLT_NORM, 4},
{16, 10, 2216, 16, 1, gScenarioBufferData226_AGGCF_FLT_NORM, 64, gScenarioKernelData226_AGGCF_FLT_NORM, 4},
{18, 10, 2464, 16, 1, gScenarioBufferData227_AGGCF_FLT_NORM, 64, gScenarioKernelData227_AGGCF_FLT_NORM, 4},
{20, 10, 2712, 16, 1, gScenarioBufferData228_AGGCF_FLT_NORM, 64, gScenarioKernelData228_AGGCF_FLT_NORM, 4},
{22, 10, 2960, 16, 1, gScenarioBufferData229_AGGCF_FLT_NORM, 64, gScenarioKernelData229_AGGCF_FLT_NORM, 4},
{24, 10, 3208, 16, 1, gScenarioBufferData230_AGGCF_FLT_NORM, 64, gScenarioKernelData230_AGGCF_FLT_NORM, 4},
{26, 10, 3456, 16, 1, gScenarioBufferData231_AGGCF_FLT_NORM, 64, gScenarioKernelData231_AGGCF_FLT_NORM, 4},
{28, 10, 3704, 16, 1, gScenarioBufferData232_AGGCF_FLT_NORM, 64, gScenarioKernelData232_AGGCF_FLT_NORM, 4},
{1, 12, 412, 16, 1, gScenarioBufferData233_AGGCF_FLT_NORM, 64, gScenarioKernelData233_AGGCF_FLT_NORM, 4},
{2, 12, 560, 16, 1, gScenarioBufferData234_AGGCF_FLT_NORM, 64, gScenarioKernelData234_AGGCF_FLT_NORM, 4},
{3, 12, 708, 16, 1, gScenarioBufferData235_AGGCF_FLT_NORM, 64, gScenarioKernelData235_AGGCF_FLT_NORM, 4},
{4, 12, 856, 16, 1, gScenarioBufferData236_AGGCF_FLT_NORM, 64, gScenarioKernelData236_AGGCF_FLT_NORM, 4},
{5, 12, 1004, 16, 1, gScenarioBufferData237_AGGCF_FLT_NORM, 64, gScenarioKernelData237_AGGCF_FLT_NORM, 4},
{6, 12, 1152, 16, 1, gScenarioBufferData238_AGGCF_FLT_NORM, 64, gScenarioKernelData238_AGGCF_FLT_NORM, 4},
{7, 12, 1300, 16, 1, gScenarioBufferData239_AGGCF_FLT_NORM, 64, gScenarioKernelData239_AGGCF_FLT_NORM, 4},
{8, 12, 1448, 16, 1, gScenarioBufferData240_AGGCF_FLT_NORM, 64, gScenarioKernelData240_AGGCF_FLT_NORM, 4},
{9, 12, 1596, 16, 1, gScenarioBufferData241_AGGCF_FLT_NORM, 64, gScenarioKernelData241_AGGCF_FLT_NORM, 4},
{10, 12, 1744, 16, 1, gScenarioBufferData242_AGGCF_FLT_NORM, 64, gScenarioKernelData242_AGGCF_FLT_NORM, 4},
{11, 12, 1892, 16, 1, gScenarioBufferData243_AGGCF_FLT_NORM, 64, gScenarioKernelData243_AGGCF_FLT_NORM, 4},
{12, 12, 2040, 16, 1, gScenarioBufferData244_AGGCF_FLT_NORM, 64, gScenarioKernelData244_AGGCF_FLT_NORM, 4},
{13, 12, 2188, 16, 1, gScenarioBufferData245_AGGCF_FLT_NORM, 64, gScenarioKernelData245_AGGCF_FLT_NORM, 4},
{14, 12, 2336, 16, 1, gScenarioBufferData246_AGGCF_FLT_NORM, 64, gScenarioKernelData246_AGGCF_FLT_NORM, 4},
{15, 12, 2484, 16, 1, gScenarioBufferData247_AGGCF_FLT_NORM, 64, gScenarioKernelData247_AGGCF_FLT_NORM, 4},
{16, 12, 2632, 16, 1, gScenarioBufferData248_AGGCF_FLT_NORM, 64, gScenarioKernelData248_AGGCF_FLT_NORM, 4},
{18, 12, 2928, 16, 1, gScenarioBufferData249_AGGCF_FLT_NORM, 64, gScenarioKernelData249_AGGCF_FLT_NORM, 4},
{20, 12, 3224, 16, 1, gScenarioBufferData250_AGGCF_FLT_NORM, 64, gScenarioKernelData250_AGGCF_FLT_NORM, 4},
{22, 12, 3520, 16, 1, gScenarioBufferData251_AGGCF_FLT_NORM, 64, gScenarioKernelData251_AGGCF_FLT_NORM, 4},
{24, 12, 3816, 16, 1, gScenarioBufferData252_AGGCF_FLT_NORM, 64, gScenarioKernelData252_AGGCF_FLT_NORM, 4},
{1, 14, 468, 16, 1, gScenarioBufferData253_AGGCF_FLT_NORM, 64, gScenarioKernelData253_AGGCF_FLT_NORM, 4},
{2, 14, 640, 16, 1, gScenarioBufferData254_AGGCF_FLT_NORM, 64, gScenarioKernelData254_AGGCF_FLT_NORM, 4},
{3, 14, 812, 16, 1, gScenarioBufferData255_AGGCF_FLT_NORM, 64, gScenarioKernelData255_AGGCF_FLT_NORM, 4},
{4, 14, 984, 16, 1, gScenarioBufferData256_AGGCF_FLT_NORM, 64, gScenarioKernelData256_AGGCF_FLT_NORM, 4},
{5, 14, 1156, 16, 1, gScenarioBufferData257_AGGCF_FLT_NORM, 64, gScenarioKernelData257_AGGCF_FLT_NORM, 4},
{6, 14, 1328, 16, 1, gScenarioBufferData258_AGGCF_FLT_NORM, 64, gScenarioKernelData258_AGGCF_FLT_NORM, 4},
{7, 14, 1500, 16, 1, gScenarioBufferData259_AGGCF_FLT_NORM, 64, gScenarioKernelData259_AGGCF_FLT_NORM, 4},
{8, 14, 1672, 16, 1, gScenarioBufferData260_AGGCF_FLT_NORM, 64, gScenarioKernelData260_AGGCF_FLT_NORM, 4},
{9, 14, 1844, 16, 1, gScenarioBufferData261_AGGCF_FLT_NORM, 64, gScenarioKernelData261_AGGCF_FLT_NORM, 4},
{10, 14, 2016, 16, 1, gScenarioBufferData262_AGGCF_FLT_NORM, 64, gScenarioKernelData262_AGGCF_FLT_NORM, 4},
{11, 14, 2188, 16, 1, gScenarioBufferData263_AGGCF_FLT_NORM, 64, gScenarioKernelData263_AGGCF_FLT_NORM, 4},
{12, 14, 2360, 16, 1, gScenarioBufferData264_AGGCF_FLT_NORM, 64, gScenarioKernelData264_AGGCF_FLT_NORM, 4},
{13, 14, 2532, 16, 1, gScenarioBufferData265_AGGCF_FLT_NORM, 64, gScenarioKernelData265_AGGCF_FLT_NORM, 4},
{14, 14, 2704, 16, 1, gScenarioBufferData266_AGGCF_FLT_NORM, 64, gScenarioKernelData266_AGGCF_FLT_NORM, 4},
{15, 14, 2876, 16, 1, gScenarioBufferData267_AGGCF_FLT_NORM, 64, gScenarioKernelData267_AGGCF_FLT_NORM, 4},
{16, 14, 3048, 16, 1, gScenarioBufferData268_AGGCF_FLT_NORM, 64, gScenarioKernelData268_AGGCF_FLT_NORM, 4},
{18, 14, 3392, 16, 1, gScenarioBufferData269_AGGCF_FLT_NORM, 64, gScenarioKernelData269_AGGCF_FLT_NORM, 4},
{20, 14, 3736, 16, 1, gScenarioBufferData270_AGGCF_FLT_NORM, 64, gScenarioKernelData270_AGGCF_FLT_NORM, 4},
{1, 16, 524, 16, 1, gScenarioBufferData271_AGGCF_FLT_NORM, 64, gScenarioKernelData271_AGGCF_FLT_NORM, 4},
{2, 16, 720, 16, 1, gScenarioBufferData272_AGGCF_FLT_NORM, 64, gScenarioKernelData272_AGGCF_FLT_NORM, 4},
{3, 16, 916, 16, 1, gScenarioBufferData273_AGGCF_FLT_NORM, 64, gScenarioKernelData273_AGGCF_FLT_NORM, 4},
{4, 16, 1112, 16, 1, gScenarioBufferData274_AGGCF_FLT_NORM, 64, gScenarioKernelData274_AGGCF_FLT_NORM, 4},
{5, 16, 1308, 16, 1, gScenarioBufferData275_AGGCF_FLT_NORM, 64, gScenarioKernelData275_AGGCF_FLT_NORM, 4},
{6, 16, 1504, 16, 1, gScenarioBufferData276_AGGCF_FLT_NORM, 64, gScenarioKernelData276_AGGCF_FLT_NORM, 4},
{7, 16, 1700, 16, 1, gScenarioBufferData277_AGGCF_FLT_NORM, 64, gScenarioKernelData277_AGGCF_FLT_NORM, 4},
{8, 16, 1896, 16, 1, gScenarioBufferData278_AGGCF_FLT_NORM, 64, gScenarioKernelData278_AGGCF_FLT_NORM, 4},
{9, 16, 2092, 16, 1, gScenarioBufferData279_AGGCF_FLT_NORM, 64, gScenarioKernelData279_AGGCF_FLT_NORM, 4},
{10, 16, 2288, 16, 1, gScenarioBufferData280_AGGCF_FLT_NORM, 64, gScenarioKernelData280_AGGCF_FLT_NORM, 4},
{11, 16, 2484, 16, 1, gScenarioBufferData281_AGGCF_FLT_NORM, 64, gScenarioKernelData281_AGGCF_FLT_NORM, 4},
{12, 16, 2680, 16, 1, gScenarioBufferData282_AGGCF_FLT_NORM, 64, gScenarioKernelData282_AGGCF_FLT_NORM, 4},
{13, 16, 2876, 16, 1, gScenarioBufferData283_AGGCF_FLT_NORM, 64, gScenarioKernelData283_AGGCF_FLT_NORM, 4},
{14, 16, 3072, 16, 1, gScenarioBufferData284_AGGCF_FLT_NORM, 64, gScenarioKernelData284_AGGCF_FLT_NORM, 4},
{15, 16, 3268, 16, 1, gScenarioBufferData285_AGGCF_FLT_NORM, 64, gScenarioKernelData285_AGGCF_FLT_NORM, 4},
{16, 16, 3464, 16, 1, gScenarioBufferData286_AGGCF_FLT_NORM, 64, gScenarioKernelData286_AGGCF_FLT_NORM, 4},
{1, 18, 580, 16, 1, gScenarioBufferData287_AGGCF_FLT_NORM, 64, gScenarioKernelData287_AGGCF_FLT_NORM, 4},
{2, 18, 800, 16, 1, gScenarioBufferData288_AGGCF_FLT_NORM, 64, gScenarioKernelData288_AGGCF_FLT_NORM, 4},
{3, 18, 1020, 16, 1, gScenarioBufferData289_AGGCF_FLT_NORM, 64, gScenarioKernelData289_AGGCF_FLT_NORM, 4},
{4, 18, 1240, 16, 1, gScenarioBufferData290_AGGCF_FLT_NORM, 64, gScenarioKernelData290_AGGCF_FLT_NORM, 4},
{5, 18, 1460, 16, 1, gScenarioBufferData291_AGGCF_FLT_NORM, 64, gScenarioKernelData291_AGGCF_FLT_NORM, 4},
{6, 18, 1680, 16, 1, gScenarioBufferData292_AGGCF_FLT_NORM, 64, gScenarioKernelData292_AGGCF_FLT_NORM, 4},
{7, 18, 1900, 16, 1, gScenarioBufferData293_AGGCF_FLT_NORM, 64, gScenarioKernelData293_AGGCF_FLT_NORM, 4},
{8, 18, 2120, 16, 1, gScenarioBufferData294_AGGCF_FLT_NORM, 64, gScenarioKernelData294_AGGCF_FLT_NORM, 4},
{9, 18, 2340, 16, 1, gScenarioBufferData295_AGGCF_FLT_NORM, 64, gScenarioKernelData295_AGGCF_FLT_NORM, 4},
{10, 18, 2560, 16, 1, gScenarioBufferData296_AGGCF_FLT_NORM, 64, gScenarioKernelData296_AGGCF_FLT_NORM, 4},
{11, 18, 2780, 16, 1, gScenarioBufferData297_AGGCF_FLT_NORM, 64, gScenarioKernelData297_AGGCF_FLT_NORM, 4},
{12, 18, 3000, 16, 1, gScenarioBufferData298_AGGCF_FLT_NORM, 64, gScenarioKernelData298_AGGCF_FLT_NORM, 4},
{13, 18, 3220, 16, 1, gScenarioBufferData299_AGGCF_FLT_NORM, 64, gScenarioKernelData299_AGGCF_FLT_NORM, 4},
{14, 18, 3440, 16, 1, gScenarioBufferData300_AGGCF_FLT_NORM, 64, gScenarioKernelData300_AGGCF_FLT_NORM, 4},
{15, 18, 3660, 16, 1, gScenarioBufferData301_AGGCF_FLT_NORM, 64, gScenarioKernelData301_AGGCF_FLT_NORM, 4},
{1, 20, 636, 16, 1, gScenarioBufferData302_AGGCF_FLT_NORM, 64, gScenarioKernelData302_AGGCF_FLT_NORM, 4},
{2, 20, 880, 16, 1, gScenarioBufferData303_AGGCF_FLT_NORM, 64, gScenarioKernelData303_AGGCF_FLT_NORM, 4},
{3, 20, 1124, 16, 1, gScenarioBufferData304_AGGCF_FLT_NORM, 64, gScenarioKernelData304_AGGCF_FLT_NORM, 4},
{4, 20, 1368, 16, 1, gScenarioBufferData305_AGGCF_FLT_NORM, 64, gScenarioKernelData305_AGGCF_FLT_NORM, 4},
{5, 20, 1612, 16, 1, gScenarioBufferData306_AGGCF_FLT_NORM, 64, gScenarioKernelData306_AGGCF_FLT_NORM, 4},
{6, 20, 1856, 16, 1, gScenarioBufferData307_AGGCF_FLT_NORM, 64, gScenarioKernelData307_AGGCF_FLT_NORM, 4},
{7, 20, 2100, 16, 1, gScenarioBufferData308_AGGCF_FLT_NORM, 64, gScenarioKernelData308_AGGCF_FLT_NORM, 4},
{8, 20, 2344, 16, 1, gScenarioBufferData309_AGGCF_FLT_NORM, 64, gScenarioKernelData309_AGGCF_FLT_NORM, 4},
{9, 20, 2588, 16, 1, gScenarioBufferData310_AGGCF_FLT_NORM, 64, gScenarioKernelData310_AGGCF_FLT_NORM, 4},
{10, 20, 2832, 16, 1, gScenarioBufferData311_AGGCF_FLT_NORM, 64, gScenarioKernelData311_AGGCF_FLT_NORM, 4},
{11, 20, 3076, 16, 1, gScenarioBufferData312_AGGCF_FLT_NORM, 64, gScenarioKernelData312_AGGCF_FLT_NORM, 4},
{12, 20, 3320, 16, 1, gScenarioBufferData313_AGGCF_FLT_NORM, 64, gScenarioKernelData313_AGGCF_FLT_NORM, 4},
{13, 20, 3564, 16, 1, gScenarioBufferData314_AGGCF_FLT_NORM, 64, gScenarioKernelData314_AGGCF_FLT_NORM, 4},
{14, 20, 3808, 16, 1, gScenarioBufferData315_AGGCF_FLT_NORM, 64, gScenarioKernelData315_AGGCF_FLT_NORM, 4},
{1, 22, 692, 16, 1, gScenarioBufferData316_AGGCF_FLT_NORM, 64, gScenarioKernelData316_AGGCF_FLT_NORM, 4},
{2, 22, 960, 16, 1, gScenarioBufferData317_AGGCF_FLT_NORM, 64, gScenarioKernelData317_AGGCF_FLT_NORM, 4},
{3, 22, 1228, 16, 1, gScenarioBufferData318_AGGCF_FLT_NORM, 64, gScenarioKernelData318_AGGCF_FLT_NORM, 4},
{4, 22, 1496, 16, 1, gScenarioBufferData319_AGGCF_FLT_NORM, 64, gScenarioKernelData319_AGGCF_FLT_NORM, 4},
{5, 22, 1764, 16, 1, gScenarioBufferData320_AGGCF_FLT_NORM, 64, gScenarioKernelData320_AGGCF_FLT_NORM, 4},
{6, 22, 2032, 16, 1, gScenarioBufferData321_AGGCF_FLT_NORM, 64, gScenarioKernelData321_AGGCF_FLT_NORM, 4},
{7, 22, 2300, 16, 1, gScenarioBufferData322_AGGCF_FLT_NORM, 64, gScenarioKernelData322_AGGCF_FLT_NORM, 4},
{8, 22, 2568, 16, 1, gScenarioBufferData323_AGGCF_FLT_NORM, 64, gScenarioKernelData323_AGGCF_FLT_NORM, 4},
{9, 22, 2836, 16, 1, gScenarioBufferData324_AGGCF_FLT_NORM, 64, gScenarioKernelData324_AGGCF_FLT_NORM, 4},
{10, 22, 3104, 16, 1, gScenarioBufferData325_AGGCF_FLT_NORM, 64, gScenarioKernelData325_AGGCF_FLT_NORM, 4},
{11, 22, 3372, 16, 1, gScenarioBufferData326_AGGCF_FLT_NORM, 64, gScenarioKernelData326_AGGCF_FLT_NORM, 4},
{12, 22, 3640, 16, 1, gScenarioBufferData327_AGGCF_FLT_NORM, 64, gScenarioKernelData327_AGGCF_FLT_NORM, 4},
{1, 24, 748, 16, 1, gScenarioBufferData328_AGGCF_FLT_NORM, 64, gScenarioKernelData328_AGGCF_FLT_NORM, 4},
{2, 24, 1040, 16, 1, gScenarioBufferData329_AGGCF_FLT_NORM, 64, gScenarioKernelData329_AGGCF_FLT_NORM, 4},
{3, 24, 1332, 16, 1, gScenarioBufferData330_AGGCF_FLT_NORM, 64, gScenarioKernelData330_AGGCF_FLT_NORM, 4},
{4, 24, 1624, 16, 1, gScenarioBufferData331_AGGCF_FLT_NORM, 64, gScenarioKernelData331_AGGCF_FLT_NORM, 4},
{5, 24, 1916, 16, 1, gScenarioBufferData332_AGGCF_FLT_NORM, 64, gScenarioKernelData332_AGGCF_FLT_NORM, 4},
{6, 24, 2208, 16, 1, gScenarioBufferData333_AGGCF_FLT_NORM, 64, gScenarioKernelData333_AGGCF_FLT_NORM, 4},
{7, 24, 2500, 16, 1, gScenarioBufferData334_AGGCF_FLT_NORM, 64, gScenarioKernelData334_AGGCF_FLT_NORM, 4},
{8, 24, 2792, 16, 1, gScenarioBufferData335_AGGCF_FLT_NORM, 64, gScenarioKernelData335_AGGCF_FLT_NORM, 4},
{9, 24, 3084, 16, 1, gScenarioBufferData336_AGGCF_FLT_NORM, 64, gScenarioKernelData336_AGGCF_FLT_NORM, 4},
{10, 24, 3376, 16, 1, gScenarioBufferData337_AGGCF_FLT_NORM, 64, gScenarioKernelData337_AGGCF_FLT_NORM, 4},
{11, 24, 3668, 16, 1, gScenarioBufferData338_AGGCF_FLT_NORM, 64, gScenarioKernelData338_AGGCF_FLT_NORM, 4},
{1, 26, 804, 16, 1, gScenarioBufferData339_AGGCF_FLT_NORM, 64, gScenarioKernelData339_AGGCF_FLT_NORM, 4},
{2, 26, 1120, 16, 1, gScenarioBufferData340_AGGCF_FLT_NORM, 64, gScenarioKernelData340_AGGCF_FLT_NORM, 4},
{3, 26, 1436, 16, 1, gScenarioBufferData341_AGGCF_FLT_NORM, 64, gScenarioKernelData341_AGGCF_FLT_NORM, 4},
{4, 26, 1752, 16, 1, gScenarioBufferData342_AGGCF_FLT_NORM, 64, gScenarioKernelData342_AGGCF_FLT_NORM, 4},
{5, 26, 2068, 16, 1, gScenarioBufferData343_AGGCF_FLT_NORM, 64, gScenarioKernelData343_AGGCF_FLT_NORM, 4},
{6, 26, 2384, 16, 1, gScenarioBufferData344_AGGCF_FLT_NORM, 64, gScenarioKernelData344_AGGCF_FLT_NORM, 4},
{7, 26, 2700, 16, 1, gScenarioBufferData345_AGGCF_FLT_NORM, 64, gScenarioKernelData345_AGGCF_FLT_NORM, 4},
{8, 26, 3016, 16, 1, gScenarioBufferData346_AGGCF_FLT_NORM, 64, gScenarioKernelData346_AGGCF_FLT_NORM, 4},
{9, 26, 3332, 16, 1, gScenarioBufferData347_AGGCF_FLT_NORM, 64, gScenarioKernelData347_AGGCF_FLT_NORM, 4},
{10, 26, 3648, 16, 1, gScenarioBufferData348_AGGCF_FLT_NORM, 64, gScenarioKernelData348_AGGCF_FLT_NORM, 4},
{1, 28, 860, 16, 1, gScenarioBufferData349_AGGCF_FLT_NORM, 64, gScenarioKernelData349_AGGCF_FLT_NORM, 4},
{2, 28, 1200, 16, 1, gScenarioBufferData350_AGGCF_FLT_NORM, 64, gScenarioKernelData350_AGGCF_FLT_NORM, 4},
{3, 28, 1540, 16, 1, gScenarioBufferData351_AGGCF_FLT_NORM, 64, gScenarioKernelData351_AGGCF_FLT_NORM, 4},
{4, 28, 1880, 16, 1, gScenarioBufferData352_AGGCF_FLT_NORM, 64, gScenarioKernelData352_AGGCF_FLT_NORM, 4},
{5, 28, 2220, 16, 1, gScenarioBufferData353_AGGCF_FLT_NORM, 64, gScenarioKernelData353_AGGCF_FLT_NORM, 4},
{6, 28, 2560, 16, 1, gScenarioBufferData354_AGGCF_FLT_NORM, 64, gScenarioKernelData354_AGGCF_FLT_NORM, 4},
{7, 28, 2900, 16, 1, gScenarioBufferData355_AGGCF_FLT_NORM, 64, gScenarioKernelData355_AGGCF_FLT_NORM, 4},
{8, 28, 3240, 16, 1, gScenarioBufferData356_AGGCF_FLT_NORM, 64, gScenarioKernelData356_AGGCF_FLT_NORM, 4},
{9, 28, 3580, 16, 1, gScenarioBufferData357_AGGCF_FLT_NORM, 64, gScenarioKernelData357_AGGCF_FLT_NORM, 4},
{1, 30, 916, 16, 1, gScenarioBufferData358_AGGCF_FLT_NORM, 64, gScenarioKernelData358_AGGCF_FLT_NORM, 4},
{2, 30, 1280, 16, 1, gScenarioBufferData359_AGGCF_FLT_NORM, 64, gScenarioKernelData359_AGGCF_FLT_NORM, 4},
{3, 30, 1644, 16, 1, gScenarioBufferData360_AGGCF_FLT_NORM, 64, gScenarioKernelData360_AGGCF_FLT_NORM, 4},
{4, 30, 2008, 16, 1, gScenarioBufferData361_AGGCF_FLT_NORM, 64, gScenarioKernelData361_AGGCF_FLT_NORM, 4},
{5, 30, 2372, 16, 1, gScenarioBufferData362_AGGCF_FLT_NORM, 64, gScenarioKernelData362_AGGCF_FLT_NORM, 4},
{6, 30, 2736, 16, 1, gScenarioBufferData363_AGGCF_FLT_NORM, 64, gScenarioKernelData363_AGGCF_FLT_NORM, 4},
{7, 30, 3100, 16, 1, gScenarioBufferData364_AGGCF_FLT_NORM, 64, gScenarioKernelData364_AGGCF_FLT_NORM, 4},
{8, 30, 3464, 16, 1, gScenarioBufferData365_AGGCF_FLT_NORM, 64, gScenarioKernelData365_AGGCF_FLT_NORM, 4},
{9, 30, 3828, 16, 1, gScenarioBufferData366_AGGCF_FLT_NORM, 64, gScenarioKernelData366_AGGCF_FLT_NORM, 4},
{1, 32, 972, 16, 1, gScenarioBufferData367_AGGCF_FLT_NORM, 64, gScenarioKernelData367_AGGCF_FLT_NORM, 4},
{2, 32, 1360, 16, 1, gScenarioBufferData368_AGGCF_FLT_NORM, 64, gScenarioKernelData368_AGGCF_FLT_NORM, 4},
{3, 32, 1748, 16, 1, gScenarioBufferData369_AGGCF_FLT_NORM, 64, gScenarioKernelData369_AGGCF_FLT_NORM, 4},
{4, 32, 2136, 16, 1, gScenarioBufferData370_AGGCF_FLT_NORM, 64, gScenarioKernelData370_AGGCF_FLT_NORM, 4},
{5, 32, 2524, 16, 1, gScenarioBufferData371_AGGCF_FLT_NORM, 64, gScenarioKernelData371_AGGCF_FLT_NORM, 4},
{6, 32, 2912, 16, 1, gScenarioBufferData372_AGGCF_FLT_NORM, 64, gScenarioKernelData372_AGGCF_FLT_NORM, 4},
{7, 32, 3300, 16, 1, gScenarioBufferData373_AGGCF_FLT_NORM, 64, gScenarioKernelData373_AGGCF_FLT_NORM, 4},
{8, 32, 3688, 16, 1, gScenarioBufferData374_AGGCF_FLT_NORM, 64, gScenarioKernelData374_AGGCF_FLT_NORM, 4}
};

static acf_scenario_list gScenarioList_AGGCF_FLT_NORM = {
375, //number of scenarios
gScenarioArray_AGGCF_FLT_NORM};
//**************************************************************

class AGGCF_FLT_NORM : public ACF_Process_APU
{

public:
   AGGCF_FLT_NORM(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_FLT_NORM");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_FLT_NORM",
                                        AGGCF_FLT_NORM_LOAD_SEGMENTS,
                                        AGGCF_FLT_NORM_LOAD_PMEM, AGGCF_FLT_NORM_LOAD_PMEM_SIZE,
                                        AGGCF_FLT_NORM_LOAD_DMEM, AGGCF_FLT_NORM_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_FLT_NORM_APEX_LOG_BUFFER, AGGCF_FLT_NORM_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_FLT_NORM);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_FLT_NORM

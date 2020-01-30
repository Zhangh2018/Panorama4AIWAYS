#ifndef _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_LAST
#define _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_LAST

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LAPLACIAN_PYRAMID_LAST_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LAPLACIAN_PYRAMID_LAST[] = {{2, 1, 4, 0, {0, 0, 2, 2}}, {2, 1, 7, 0, {2, 2, 0, 0}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {2, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_LAPLACIAN_PYRAMID_LAST[] = {{4, 1, 4, 0, {0, 0, 2, 2}}, {4, 1, 7, 0, {2, 2, 0, 0}}, {4, 1, 2, 2, {0, 0, 0, 0}}, {4, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_LAPLACIAN_PYRAMID_LAST[] = {{6, 1, 4, 0, {0, 0, 2, 2}}, {6, 1, 7, 0, {2, 2, 0, 0}}, {6, 1, 2, 2, {0, 0, 0, 0}}, {6, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_LAPLACIAN_PYRAMID_LAST[] = {{8, 1, 4, 0, {0, 0, 2, 2}}, {8, 1, 7, 0, {2, 2, 0, 0}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {8, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_LAPLACIAN_PYRAMID_LAST[] = {{10, 1, 4, 0, {0, 0, 2, 2}}, {10, 1, 7, 0, {2, 2, 0, 0}}, {10, 1, 2, 2, {0, 0, 0, 0}}, {10, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_LAPLACIAN_PYRAMID_LAST[] = {{12, 1, 4, 0, {0, 0, 2, 2}}, {12, 1, 7, 0, {2, 2, 0, 0}}, {12, 1, 2, 2, {0, 0, 0, 0}}, {12, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_LAPLACIAN_PYRAMID_LAST[] = {{14, 1, 4, 0, {0, 0, 2, 2}}, {14, 1, 7, 0, {2, 2, 0, 0}}, {14, 1, 2, 2, {0, 0, 0, 0}}, {14, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_LAPLACIAN_PYRAMID_LAST[] = {{16, 1, 4, 0, {0, 0, 2, 2}}, {16, 1, 7, 0, {2, 2, 0, 0}}, {16, 1, 2, 2, {0, 0, 0, 0}}, {16, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_LAPLACIAN_PYRAMID_LAST[] = {{18, 1, 4, 0, {0, 0, 2, 2}}, {18, 1, 7, 0, {2, 2, 0, 0}}, {18, 1, 2, 2, {0, 0, 0, 0}}, {18, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_LAPLACIAN_PYRAMID_LAST[] = {{20, 1, 4, 0, {0, 0, 2, 2}}, {20, 1, 7, 0, {2, 2, 0, 0}}, {20, 1, 2, 2, {0, 0, 0, 0}}, {20, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_LAPLACIAN_PYRAMID_LAST[] = {{22, 1, 4, 0, {0, 0, 2, 2}}, {22, 1, 7, 0, {2, 2, 0, 0}}, {22, 1, 2, 2, {0, 0, 0, 0}}, {22, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_LAPLACIAN_PYRAMID_LAST[] = {{24, 1, 4, 0, {0, 0, 2, 2}}, {24, 1, 7, 0, {2, 2, 0, 0}}, {24, 1, 2, 2, {0, 0, 0, 0}}, {24, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_LAPLACIAN_PYRAMID_LAST[] = {{26, 1, 4, 0, {0, 0, 2, 2}}, {26, 1, 7, 0, {2, 2, 0, 0}}, {26, 1, 2, 2, {0, 0, 0, 0}}, {26, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_LAPLACIAN_PYRAMID_LAST[] = {{28, 1, 4, 0, {0, 0, 2, 2}}, {28, 1, 7, 0, {2, 2, 0, 0}}, {28, 1, 2, 2, {0, 0, 0, 0}}, {28, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_LAPLACIAN_PYRAMID_LAST[] = {{30, 1, 4, 0, {0, 0, 2, 2}}, {30, 1, 7, 0, {2, 2, 0, 0}}, {30, 1, 2, 2, {0, 0, 0, 0}}, {30, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_LAPLACIAN_PYRAMID_LAST[] = {{32, 1, 4, 0, {0, 0, 2, 2}}, {32, 1, 7, 0, {2, 2, 0, 0}}, {32, 1, 2, 2, {0, 0, 0, 0}}, {32, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_LAPLACIAN_PYRAMID_LAST[] = {{36, 1, 4, 0, {0, 0, 2, 2}}, {36, 1, 7, 0, {2, 2, 0, 0}}, {36, 1, 2, 2, {0, 0, 0, 0}}, {36, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_LAPLACIAN_PYRAMID_LAST[] = {{40, 1, 4, 0, {0, 0, 2, 2}}, {40, 1, 7, 0, {2, 2, 0, 0}}, {40, 1, 2, 2, {0, 0, 0, 0}}, {40, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_LAPLACIAN_PYRAMID_LAST[] = {{44, 1, 4, 0, {0, 0, 2, 2}}, {44, 1, 7, 0, {2, 2, 0, 0}}, {44, 1, 2, 2, {0, 0, 0, 0}}, {44, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_LAPLACIAN_PYRAMID_LAST[] = {{48, 1, 4, 0, {0, 0, 2, 2}}, {48, 1, 7, 0, {2, 2, 0, 0}}, {48, 1, 2, 2, {0, 0, 0, 0}}, {48, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_LAPLACIAN_PYRAMID_LAST[] = {{52, 1, 4, 0, {0, 0, 2, 2}}, {52, 1, 7, 0, {2, 2, 0, 0}}, {52, 1, 2, 2, {0, 0, 0, 0}}, {52, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_LAPLACIAN_PYRAMID_LAST[] = {{56, 1, 4, 0, {0, 0, 2, 2}}, {56, 1, 7, 0, {2, 2, 0, 0}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {56, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_LAPLACIAN_PYRAMID_LAST[] = {{60, 1, 4, 0, {0, 0, 2, 2}}, {60, 1, 7, 0, {2, 2, 0, 0}}, {60, 1, 2, 2, {0, 0, 0, 0}}, {60, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_LAPLACIAN_PYRAMID_LAST[] = {{64, 1, 4, 0, {0, 0, 2, 2}}, {64, 1, 7, 0, {2, 2, 0, 0}}, {64, 1, 2, 2, {0, 0, 0, 0}}, {64, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_LAPLACIAN_PYRAMID_LAST[] = {{2, 2, 3, 0, {0, 0, 2, 2}}, {2, 2, 4, 0, {2, 2, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_LAPLACIAN_PYRAMID_LAST[] = {{4, 2, 3, 0, {0, 0, 2, 2}}, {4, 2, 4, 0, {2, 2, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_LAPLACIAN_PYRAMID_LAST[] = {{6, 2, 3, 0, {0, 0, 2, 2}}, {6, 2, 4, 0, {2, 2, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_LAPLACIAN_PYRAMID_LAST[] = {{8, 2, 3, 0, {0, 0, 2, 2}}, {8, 2, 4, 0, {2, 2, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_LAPLACIAN_PYRAMID_LAST[] = {{10, 2, 3, 0, {0, 0, 2, 2}}, {10, 2, 4, 0, {2, 2, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_LAPLACIAN_PYRAMID_LAST[] = {{12, 2, 3, 0, {0, 0, 2, 2}}, {12, 2, 4, 0, {2, 2, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_LAPLACIAN_PYRAMID_LAST[] = {{14, 2, 3, 0, {0, 0, 2, 2}}, {14, 2, 4, 0, {2, 2, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_LAPLACIAN_PYRAMID_LAST[] = {{16, 2, 3, 0, {0, 0, 2, 2}}, {16, 2, 4, 0, {2, 2, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_LAPLACIAN_PYRAMID_LAST[] = {{18, 2, 3, 0, {0, 0, 2, 2}}, {18, 2, 4, 0, {2, 2, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_LAPLACIAN_PYRAMID_LAST[] = {{20, 2, 3, 0, {0, 0, 2, 2}}, {20, 2, 4, 0, {2, 2, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_LAPLACIAN_PYRAMID_LAST[] = {{22, 2, 3, 0, {0, 0, 2, 2}}, {22, 2, 4, 0, {2, 2, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_LAPLACIAN_PYRAMID_LAST[] = {{24, 2, 3, 0, {0, 0, 2, 2}}, {24, 2, 4, 0, {2, 2, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_LAPLACIAN_PYRAMID_LAST[] = {{26, 2, 3, 0, {0, 0, 2, 2}}, {26, 2, 4, 0, {2, 2, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_LAPLACIAN_PYRAMID_LAST[] = {{28, 2, 3, 0, {0, 0, 2, 2}}, {28, 2, 4, 0, {2, 2, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_LAPLACIAN_PYRAMID_LAST[] = {{30, 2, 3, 0, {0, 0, 2, 2}}, {30, 2, 4, 0, {2, 2, 0, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_LAPLACIAN_PYRAMID_LAST[] = {{32, 2, 3, 0, {0, 0, 2, 2}}, {32, 2, 4, 0, {2, 2, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_LAPLACIAN_PYRAMID_LAST[] = {{36, 2, 3, 0, {0, 0, 2, 2}}, {36, 2, 4, 0, {2, 2, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_LAPLACIAN_PYRAMID_LAST[] = {{40, 2, 3, 0, {0, 0, 2, 2}}, {40, 2, 4, 0, {2, 2, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_LAPLACIAN_PYRAMID_LAST[] = {{44, 2, 3, 0, {0, 0, 2, 2}}, {44, 2, 4, 0, {2, 2, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_LAPLACIAN_PYRAMID_LAST[] = {{48, 2, 3, 0, {0, 0, 2, 2}}, {48, 2, 4, 0, {2, 2, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_LAPLACIAN_PYRAMID_LAST[] = {{52, 2, 3, 0, {0, 0, 2, 2}}, {52, 2, 4, 0, {2, 2, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_LAPLACIAN_PYRAMID_LAST[] = {{56, 2, 3, 0, {0, 0, 2, 2}}, {56, 2, 4, 0, {2, 2, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_LAPLACIAN_PYRAMID_LAST[] = {{60, 2, 3, 0, {0, 0, 2, 2}}, {60, 2, 4, 0, {2, 2, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_LAPLACIAN_PYRAMID_LAST[] = {{64, 2, 3, 0, {0, 0, 2, 2}}, {64, 2, 4, 0, {2, 2, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_LAPLACIAN_PYRAMID_LAST[] = {{2, 3, 3, 0, {0, 0, 2, 2}}, {2, 3, 4, 0, {2, 2, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_LAPLACIAN_PYRAMID_LAST[] = {{4, 3, 3, 0, {0, 0, 2, 2}}, {4, 3, 4, 0, {2, 2, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_LAPLACIAN_PYRAMID_LAST[] = {{6, 3, 3, 0, {0, 0, 2, 2}}, {6, 3, 4, 0, {2, 2, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_LAPLACIAN_PYRAMID_LAST[] = {{8, 3, 3, 0, {0, 0, 2, 2}}, {8, 3, 4, 0, {2, 2, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_LAPLACIAN_PYRAMID_LAST[] = {{10, 3, 3, 0, {0, 0, 2, 2}}, {10, 3, 4, 0, {2, 2, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_LAPLACIAN_PYRAMID_LAST[] = {{12, 3, 3, 0, {0, 0, 2, 2}}, {12, 3, 4, 0, {2, 2, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_LAPLACIAN_PYRAMID_LAST[] = {{14, 3, 3, 0, {0, 0, 2, 2}}, {14, 3, 4, 0, {2, 2, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_LAPLACIAN_PYRAMID_LAST[] = {{16, 3, 3, 0, {0, 0, 2, 2}}, {16, 3, 4, 0, {2, 2, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_LAPLACIAN_PYRAMID_LAST[] = {{18, 3, 3, 0, {0, 0, 2, 2}}, {18, 3, 4, 0, {2, 2, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_LAPLACIAN_PYRAMID_LAST[] = {{20, 3, 3, 0, {0, 0, 2, 2}}, {20, 3, 4, 0, {2, 2, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_LAPLACIAN_PYRAMID_LAST[] = {{22, 3, 3, 0, {0, 0, 2, 2}}, {22, 3, 4, 0, {2, 2, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_LAPLACIAN_PYRAMID_LAST[] = {{24, 3, 3, 0, {0, 0, 2, 2}}, {24, 3, 4, 0, {2, 2, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_LAPLACIAN_PYRAMID_LAST[] = {{26, 3, 3, 0, {0, 0, 2, 2}}, {26, 3, 4, 0, {2, 2, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_LAPLACIAN_PYRAMID_LAST[] = {{28, 3, 3, 0, {0, 0, 2, 2}}, {28, 3, 4, 0, {2, 2, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_LAPLACIAN_PYRAMID_LAST[] = {{30, 3, 3, 0, {0, 0, 2, 2}}, {30, 3, 4, 0, {2, 2, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_LAPLACIAN_PYRAMID_LAST[] = {{32, 3, 3, 0, {0, 0, 2, 2}}, {32, 3, 4, 0, {2, 2, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_LAPLACIAN_PYRAMID_LAST[] = {{36, 3, 3, 0, {0, 0, 2, 2}}, {36, 3, 4, 0, {2, 2, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_LAPLACIAN_PYRAMID_LAST[] = {{40, 3, 3, 0, {0, 0, 2, 2}}, {40, 3, 4, 0, {2, 2, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_LAPLACIAN_PYRAMID_LAST[] = {{44, 3, 3, 0, {0, 0, 2, 2}}, {44, 3, 4, 0, {2, 2, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_LAPLACIAN_PYRAMID_LAST[] = {{48, 3, 3, 0, {0, 0, 2, 2}}, {48, 3, 4, 0, {2, 2, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_LAPLACIAN_PYRAMID_LAST[] = {{52, 3, 3, 0, {0, 0, 2, 2}}, {52, 3, 4, 0, {2, 2, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_LAPLACIAN_PYRAMID_LAST[] = {{56, 3, 3, 0, {0, 0, 2, 2}}, {56, 3, 4, 0, {2, 2, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_LAPLACIAN_PYRAMID_LAST[] = {{2, 4, 3, 0, {0, 0, 2, 2}}, {2, 4, 4, 0, {2, 2, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_LAPLACIAN_PYRAMID_LAST[] = {{4, 4, 3, 0, {0, 0, 2, 2}}, {4, 4, 4, 0, {2, 2, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_LAPLACIAN_PYRAMID_LAST[] = {{6, 4, 3, 0, {0, 0, 2, 2}}, {6, 4, 4, 0, {2, 2, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_LAPLACIAN_PYRAMID_LAST[] = {{8, 4, 3, 0, {0, 0, 2, 2}}, {8, 4, 4, 0, {2, 2, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_LAPLACIAN_PYRAMID_LAST[] = {{10, 4, 3, 0, {0, 0, 2, 2}}, {10, 4, 4, 0, {2, 2, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_LAPLACIAN_PYRAMID_LAST[] = {{12, 4, 3, 0, {0, 0, 2, 2}}, {12, 4, 4, 0, {2, 2, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_LAPLACIAN_PYRAMID_LAST[] = {{14, 4, 3, 0, {0, 0, 2, 2}}, {14, 4, 4, 0, {2, 2, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_LAPLACIAN_PYRAMID_LAST[] = {{16, 4, 3, 0, {0, 0, 2, 2}}, {16, 4, 4, 0, {2, 2, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_LAPLACIAN_PYRAMID_LAST[] = {{18, 4, 3, 0, {0, 0, 2, 2}}, {18, 4, 4, 0, {2, 2, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_LAPLACIAN_PYRAMID_LAST[] = {{20, 4, 3, 0, {0, 0, 2, 2}}, {20, 4, 4, 0, {2, 2, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_LAPLACIAN_PYRAMID_LAST[] = {{22, 4, 3, 0, {0, 0, 2, 2}}, {22, 4, 4, 0, {2, 2, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_LAPLACIAN_PYRAMID_LAST[] = {{24, 4, 3, 0, {0, 0, 2, 2}}, {24, 4, 4, 0, {2, 2, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_LAPLACIAN_PYRAMID_LAST[] = {{26, 4, 3, 0, {0, 0, 2, 2}}, {26, 4, 4, 0, {2, 2, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_LAPLACIAN_PYRAMID_LAST[] = {{28, 4, 3, 0, {0, 0, 2, 2}}, {28, 4, 4, 0, {2, 2, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_LAPLACIAN_PYRAMID_LAST[] = {{30, 4, 3, 0, {0, 0, 2, 2}}, {30, 4, 4, 0, {2, 2, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_LAPLACIAN_PYRAMID_LAST[] = {{32, 4, 3, 0, {0, 0, 2, 2}}, {32, 4, 4, 0, {2, 2, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_LAPLACIAN_PYRAMID_LAST[] = {{36, 4, 3, 0, {0, 0, 2, 2}}, {36, 4, 4, 0, {2, 2, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_LAPLACIAN_PYRAMID_LAST[] = {{40, 4, 3, 0, {0, 0, 2, 2}}, {40, 4, 4, 0, {2, 2, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_LAPLACIAN_PYRAMID_LAST[] = {{44, 4, 3, 0, {0, 0, 2, 2}}, {44, 4, 4, 0, {2, 2, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_LAPLACIAN_PYRAMID_LAST[] = {{2, 5, 3, 0, {0, 0, 2, 2}}, {2, 5, 4, 0, {2, 2, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_LAPLACIAN_PYRAMID_LAST[] = {{4, 5, 3, 0, {0, 0, 2, 2}}, {4, 5, 4, 0, {2, 2, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_LAPLACIAN_PYRAMID_LAST[] = {{6, 5, 3, 0, {0, 0, 2, 2}}, {6, 5, 4, 0, {2, 2, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_LAPLACIAN_PYRAMID_LAST[] = {{8, 5, 3, 0, {0, 0, 2, 2}}, {8, 5, 4, 0, {2, 2, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_LAPLACIAN_PYRAMID_LAST[] = {{10, 5, 3, 0, {0, 0, 2, 2}}, {10, 5, 4, 0, {2, 2, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_LAPLACIAN_PYRAMID_LAST[] = {{12, 5, 3, 0, {0, 0, 2, 2}}, {12, 5, 4, 0, {2, 2, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_LAPLACIAN_PYRAMID_LAST[] = {{14, 5, 3, 0, {0, 0, 2, 2}}, {14, 5, 4, 0, {2, 2, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_LAPLACIAN_PYRAMID_LAST[] = {{16, 5, 3, 0, {0, 0, 2, 2}}, {16, 5, 4, 0, {2, 2, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_LAPLACIAN_PYRAMID_LAST[] = {{18, 5, 3, 0, {0, 0, 2, 2}}, {18, 5, 4, 0, {2, 2, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_LAPLACIAN_PYRAMID_LAST[] = {{20, 5, 3, 0, {0, 0, 2, 2}}, {20, 5, 4, 0, {2, 2, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_LAPLACIAN_PYRAMID_LAST[] = {{22, 5, 3, 0, {0, 0, 2, 2}}, {22, 5, 4, 0, {2, 2, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_LAPLACIAN_PYRAMID_LAST[] = {{24, 5, 3, 0, {0, 0, 2, 2}}, {24, 5, 4, 0, {2, 2, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_LAPLACIAN_PYRAMID_LAST[] = {{26, 5, 3, 0, {0, 0, 2, 2}}, {26, 5, 4, 0, {2, 2, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_LAPLACIAN_PYRAMID_LAST[] = {{28, 5, 3, 0, {0, 0, 2, 2}}, {28, 5, 4, 0, {2, 2, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_LAPLACIAN_PYRAMID_LAST[] = {{30, 5, 3, 0, {0, 0, 2, 2}}, {30, 5, 4, 0, {2, 2, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_LAPLACIAN_PYRAMID_LAST[] = {{32, 5, 3, 0, {0, 0, 2, 2}}, {32, 5, 4, 0, {2, 2, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_LAPLACIAN_PYRAMID_LAST[] = {{36, 5, 3, 0, {0, 0, 2, 2}}, {36, 5, 4, 0, {2, 2, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_LAPLACIAN_PYRAMID_LAST[] = {{2, 6, 3, 0, {0, 0, 2, 2}}, {2, 6, 4, 0, {2, 2, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_LAPLACIAN_PYRAMID_LAST[] = {{4, 6, 3, 0, {0, 0, 2, 2}}, {4, 6, 4, 0, {2, 2, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_LAPLACIAN_PYRAMID_LAST[] = {{6, 6, 3, 0, {0, 0, 2, 2}}, {6, 6, 4, 0, {2, 2, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_LAPLACIAN_PYRAMID_LAST[] = {{8, 6, 3, 0, {0, 0, 2, 2}}, {8, 6, 4, 0, {2, 2, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_LAPLACIAN_PYRAMID_LAST[] = {{10, 6, 3, 0, {0, 0, 2, 2}}, {10, 6, 4, 0, {2, 2, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_LAPLACIAN_PYRAMID_LAST[] = {{12, 6, 3, 0, {0, 0, 2, 2}}, {12, 6, 4, 0, {2, 2, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_LAPLACIAN_PYRAMID_LAST[] = {{14, 6, 3, 0, {0, 0, 2, 2}}, {14, 6, 4, 0, {2, 2, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_LAPLACIAN_PYRAMID_LAST[] = {{16, 6, 3, 0, {0, 0, 2, 2}}, {16, 6, 4, 0, {2, 2, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_LAPLACIAN_PYRAMID_LAST[] = {{18, 6, 3, 0, {0, 0, 2, 2}}, {18, 6, 4, 0, {2, 2, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_LAPLACIAN_PYRAMID_LAST[] = {{20, 6, 3, 0, {0, 0, 2, 2}}, {20, 6, 4, 0, {2, 2, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_LAPLACIAN_PYRAMID_LAST[] = {{22, 6, 3, 0, {0, 0, 2, 2}}, {22, 6, 4, 0, {2, 2, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_LAPLACIAN_PYRAMID_LAST[] = {{24, 6, 3, 0, {0, 0, 2, 2}}, {24, 6, 4, 0, {2, 2, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_LAPLACIAN_PYRAMID_LAST[] = {{26, 6, 3, 0, {0, 0, 2, 2}}, {26, 6, 4, 0, {2, 2, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_LAPLACIAN_PYRAMID_LAST[] = {{28, 6, 3, 0, {0, 0, 2, 2}}, {28, 6, 4, 0, {2, 2, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_LAPLACIAN_PYRAMID_LAST[] = {{30, 6, 3, 0, {0, 0, 2, 2}}, {30, 6, 4, 0, {2, 2, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_LAPLACIAN_PYRAMID_LAST[] = {{2, 8, 3, 0, {0, 0, 2, 2}}, {2, 8, 4, 0, {2, 2, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_LAPLACIAN_PYRAMID_LAST[] = {{4, 8, 3, 0, {0, 0, 2, 2}}, {4, 8, 4, 0, {2, 2, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_LAPLACIAN_PYRAMID_LAST[] = {{6, 8, 3, 0, {0, 0, 2, 2}}, {6, 8, 4, 0, {2, 2, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_LAPLACIAN_PYRAMID_LAST[] = {{8, 8, 3, 0, {0, 0, 2, 2}}, {8, 8, 4, 0, {2, 2, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_LAPLACIAN_PYRAMID_LAST[] = {{10, 8, 3, 0, {0, 0, 2, 2}}, {10, 8, 4, 0, {2, 2, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_LAPLACIAN_PYRAMID_LAST[] = {{12, 8, 3, 0, {0, 0, 2, 2}}, {12, 8, 4, 0, {2, 2, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_LAPLACIAN_PYRAMID_LAST[] = {{14, 8, 3, 0, {0, 0, 2, 2}}, {14, 8, 4, 0, {2, 2, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_LAPLACIAN_PYRAMID_LAST[] = {{16, 8, 3, 0, {0, 0, 2, 2}}, {16, 8, 4, 0, {2, 2, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_LAPLACIAN_PYRAMID_LAST[] = {{18, 8, 3, 0, {0, 0, 2, 2}}, {18, 8, 4, 0, {2, 2, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_LAPLACIAN_PYRAMID_LAST[] = {{20, 8, 3, 0, {0, 0, 2, 2}}, {20, 8, 4, 0, {2, 2, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_LAPLACIAN_PYRAMID_LAST[] = {{22, 8, 3, 0, {0, 0, 2, 2}}, {22, 8, 4, 0, {2, 2, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_LAPLACIAN_PYRAMID_LAST[] = {{2, 10, 3, 0, {0, 0, 2, 2}}, {2, 10, 4, 0, {2, 2, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_LAPLACIAN_PYRAMID_LAST[] = {{4, 10, 3, 0, {0, 0, 2, 2}}, {4, 10, 4, 0, {2, 2, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_LAPLACIAN_PYRAMID_LAST[] = {{6, 10, 3, 0, {0, 0, 2, 2}}, {6, 10, 4, 0, {2, 2, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_LAPLACIAN_PYRAMID_LAST[] = {{8, 10, 3, 0, {0, 0, 2, 2}}, {8, 10, 4, 0, {2, 2, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_LAPLACIAN_PYRAMID_LAST[] = {{10, 10, 3, 0, {0, 0, 2, 2}}, {10, 10, 4, 0, {2, 2, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_LAPLACIAN_PYRAMID_LAST[] = {{12, 10, 3, 0, {0, 0, 2, 2}}, {12, 10, 4, 0, {2, 2, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_LAPLACIAN_PYRAMID_LAST[] = {{14, 10, 3, 0, {0, 0, 2, 2}}, {14, 10, 4, 0, {2, 2, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_LAPLACIAN_PYRAMID_LAST[] = {{16, 10, 3, 0, {0, 0, 2, 2}}, {16, 10, 4, 0, {2, 2, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_LAPLACIAN_PYRAMID_LAST[] = {{18, 10, 3, 0, {0, 0, 2, 2}}, {18, 10, 4, 0, {2, 2, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_LAPLACIAN_PYRAMID_LAST[] = {{2, 12, 3, 0, {0, 0, 2, 2}}, {2, 12, 4, 0, {2, 2, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_LAPLACIAN_PYRAMID_LAST[] = {{4, 12, 3, 0, {0, 0, 2, 2}}, {4, 12, 4, 0, {2, 2, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_LAPLACIAN_PYRAMID_LAST[] = {{6, 12, 3, 0, {0, 0, 2, 2}}, {6, 12, 4, 0, {2, 2, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_LAPLACIAN_PYRAMID_LAST[] = {{8, 12, 3, 0, {0, 0, 2, 2}}, {8, 12, 4, 0, {2, 2, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_LAPLACIAN_PYRAMID_LAST[] = {{10, 12, 3, 0, {0, 0, 2, 2}}, {10, 12, 4, 0, {2, 2, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_LAPLACIAN_PYRAMID_LAST[] = {{12, 12, 3, 0, {0, 0, 2, 2}}, {12, 12, 4, 0, {2, 2, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_LAPLACIAN_PYRAMID_LAST[] = {{14, 12, 3, 0, {0, 0, 2, 2}}, {14, 12, 4, 0, {2, 2, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_LAPLACIAN_PYRAMID_LAST[] = {{2, 14, 3, 0, {0, 0, 2, 2}}, {2, 14, 4, 0, {2, 2, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_LAPLACIAN_PYRAMID_LAST[] = {{4, 14, 3, 0, {0, 0, 2, 2}}, {4, 14, 4, 0, {2, 2, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_LAPLACIAN_PYRAMID_LAST[] = {{6, 14, 3, 0, {0, 0, 2, 2}}, {6, 14, 4, 0, {2, 2, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_LAPLACIAN_PYRAMID_LAST[] = {{8, 14, 3, 0, {0, 0, 2, 2}}, {8, 14, 4, 0, {2, 2, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_LAPLACIAN_PYRAMID_LAST[] = {{10, 14, 3, 0, {0, 0, 2, 2}}, {10, 14, 4, 0, {2, 2, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_LAPLACIAN_PYRAMID_LAST[] = {{12, 14, 3, 0, {0, 0, 2, 2}}, {12, 14, 4, 0, {2, 2, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_LAPLACIAN_PYRAMID_LAST[] = {{2, 16, 3, 0, {0, 0, 2, 2}}, {2, 16, 4, 0, {2, 2, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_LAPLACIAN_PYRAMID_LAST[] = {{4, 16, 3, 0, {0, 0, 2, 2}}, {4, 16, 4, 0, {2, 2, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_LAPLACIAN_PYRAMID_LAST[] = {{6, 16, 3, 0, {0, 0, 2, 2}}, {6, 16, 4, 0, {2, 2, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_LAPLACIAN_PYRAMID_LAST[] = {{8, 16, 3, 0, {0, 0, 2, 2}}, {8, 16, 4, 0, {2, 2, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_LAPLACIAN_PYRAMID_LAST[] = {{10, 16, 3, 0, {0, 0, 2, 2}}, {10, 16, 4, 0, {2, 2, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_LAPLACIAN_PYRAMID_LAST[] = {{2, 18, 3, 0, {0, 0, 2, 2}}, {2, 18, 4, 0, {2, 2, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_LAPLACIAN_PYRAMID_LAST[] = {{4, 18, 3, 0, {0, 0, 2, 2}}, {4, 18, 4, 0, {2, 2, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_LAPLACIAN_PYRAMID_LAST[] = {{6, 18, 3, 0, {0, 0, 2, 2}}, {6, 18, 4, 0, {2, 2, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_LAPLACIAN_PYRAMID_LAST[] = {{8, 18, 3, 0, {0, 0, 2, 2}}, {8, 18, 4, 0, {2, 2, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_LAPLACIAN_PYRAMID_LAST[] = {{10, 18, 3, 0, {0, 0, 2, 2}}, {10, 18, 4, 0, {2, 2, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_LAPLACIAN_PYRAMID_LAST[] = {{2, 20, 3, 0, {0, 0, 2, 2}}, {2, 20, 4, 0, {2, 2, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_LAPLACIAN_PYRAMID_LAST[] = {{4, 20, 3, 0, {0, 0, 2, 2}}, {4, 20, 4, 0, {2, 2, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_LAPLACIAN_PYRAMID_LAST[] = {{6, 20, 3, 0, {0, 0, 2, 2}}, {6, 20, 4, 0, {2, 2, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_LAPLACIAN_PYRAMID_LAST[] = {{8, 20, 3, 0, {0, 0, 2, 2}}, {8, 20, 4, 0, {2, 2, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_LAPLACIAN_PYRAMID_LAST[] = {{2, 22, 3, 0, {0, 0, 2, 2}}, {2, 22, 4, 0, {2, 2, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_LAPLACIAN_PYRAMID_LAST[] = {{4, 22, 3, 0, {0, 0, 2, 2}}, {4, 22, 4, 0, {2, 2, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_LAPLACIAN_PYRAMID_LAST[] = {{6, 22, 3, 0, {0, 0, 2, 2}}, {6, 22, 4, 0, {2, 2, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_LAPLACIAN_PYRAMID_LAST[] = {{8, 22, 3, 0, {0, 0, 2, 2}}, {8, 22, 4, 0, {2, 2, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_LAPLACIAN_PYRAMID_LAST[] = {{2, 24, 3, 0, {0, 0, 2, 2}}, {2, 24, 4, 0, {2, 2, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_LAPLACIAN_PYRAMID_LAST[] = {{4, 24, 3, 0, {0, 0, 2, 2}}, {4, 24, 4, 0, {2, 2, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_LAPLACIAN_PYRAMID_LAST[] = {{6, 24, 3, 0, {0, 0, 2, 2}}, {6, 24, 4, 0, {2, 2, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_LAPLACIAN_PYRAMID_LAST[] = {{2, 26, 3, 0, {0, 0, 2, 2}}, {2, 26, 4, 0, {2, 2, 0, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_LAPLACIAN_PYRAMID_LAST[] = {{4, 26, 3, 0, {0, 0, 2, 2}}, {4, 26, 4, 0, {2, 2, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_LAPLACIAN_PYRAMID_LAST[] = {{6, 26, 3, 0, {0, 0, 2, 2}}, {6, 26, 4, 0, {2, 2, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_LAPLACIAN_PYRAMID_LAST[] = {{2, 28, 3, 0, {0, 0, 2, 2}}, {2, 28, 4, 0, {2, 2, 0, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_LAPLACIAN_PYRAMID_LAST[] = {{4, 28, 3, 0, {0, 0, 2, 2}}, {4, 28, 4, 0, {2, 2, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_LAPLACIAN_PYRAMID_LAST[] = {{6, 28, 3, 0, {0, 0, 2, 2}}, {6, 28, 4, 0, {2, 2, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_LAPLACIAN_PYRAMID_LAST[] = {{2, 30, 3, 0, {0, 0, 2, 2}}, {2, 30, 4, 0, {2, 2, 0, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_LAPLACIAN_PYRAMID_LAST[] = {{4, 30, 3, 0, {0, 0, 2, 2}}, {4, 30, 4, 0, {2, 2, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_LAPLACIAN_PYRAMID_LAST[] = {{6, 30, 3, 0, {0, 0, 2, 2}}, {6, 30, 4, 0, {2, 2, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_LAPLACIAN_PYRAMID_LAST[] = {{2, 32, 3, 0, {0, 0, 2, 2}}, {2, 32, 4, 0, {2, 2, 0, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_LAPLACIAN_PYRAMID_LAST[] = {{4, 32, 3, 0, {0, 0, 2, 2}}, {4, 32, 4, 0, {2, 2, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_LAPLACIAN_PYRAMID_LAST[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_LAPLACIAN_PYRAMID_LAST[] = {
{2, 1, 84, 0, 2, gScenarioBufferData0_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData0_LAPLACIAN_PYRAMID_LAST, 8},
{4, 1, 152, 0, 2, gScenarioBufferData1_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData1_LAPLACIAN_PYRAMID_LAST, 8},
{6, 1, 220, 0, 2, gScenarioBufferData2_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData2_LAPLACIAN_PYRAMID_LAST, 8},
{8, 1, 288, 0, 2, gScenarioBufferData3_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData3_LAPLACIAN_PYRAMID_LAST, 8},
{10, 1, 356, 0, 2, gScenarioBufferData4_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData4_LAPLACIAN_PYRAMID_LAST, 8},
{12, 1, 424, 0, 2, gScenarioBufferData5_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData5_LAPLACIAN_PYRAMID_LAST, 8},
{14, 1, 492, 0, 2, gScenarioBufferData6_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData6_LAPLACIAN_PYRAMID_LAST, 8},
{16, 1, 560, 0, 2, gScenarioBufferData7_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData7_LAPLACIAN_PYRAMID_LAST, 8},
{18, 1, 628, 0, 2, gScenarioBufferData8_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData8_LAPLACIAN_PYRAMID_LAST, 8},
{20, 1, 696, 0, 2, gScenarioBufferData9_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData9_LAPLACIAN_PYRAMID_LAST, 8},
{22, 1, 764, 0, 2, gScenarioBufferData10_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData10_LAPLACIAN_PYRAMID_LAST, 8},
{24, 1, 832, 0, 2, gScenarioBufferData11_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData11_LAPLACIAN_PYRAMID_LAST, 8},
{26, 1, 900, 0, 2, gScenarioBufferData12_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData12_LAPLACIAN_PYRAMID_LAST, 8},
{28, 1, 968, 0, 2, gScenarioBufferData13_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData13_LAPLACIAN_PYRAMID_LAST, 8},
{30, 1, 1036, 0, 2, gScenarioBufferData14_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData14_LAPLACIAN_PYRAMID_LAST, 8},
{32, 1, 1104, 0, 2, gScenarioBufferData15_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData15_LAPLACIAN_PYRAMID_LAST, 8},
{36, 1, 1240, 0, 2, gScenarioBufferData16_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData16_LAPLACIAN_PYRAMID_LAST, 8},
{40, 1, 1376, 0, 2, gScenarioBufferData17_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData17_LAPLACIAN_PYRAMID_LAST, 8},
{44, 1, 1512, 0, 2, gScenarioBufferData18_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData18_LAPLACIAN_PYRAMID_LAST, 8},
{48, 1, 1648, 0, 2, gScenarioBufferData19_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData19_LAPLACIAN_PYRAMID_LAST, 8},
{52, 1, 1784, 0, 2, gScenarioBufferData20_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData20_LAPLACIAN_PYRAMID_LAST, 8},
{56, 1, 1920, 0, 2, gScenarioBufferData21_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData21_LAPLACIAN_PYRAMID_LAST, 8},
{60, 1, 2056, 0, 2, gScenarioBufferData22_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData22_LAPLACIAN_PYRAMID_LAST, 8},
{64, 1, 2192, 0, 2, gScenarioBufferData23_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData23_LAPLACIAN_PYRAMID_LAST, 8},
{2, 2, 116, 0, 1, gScenarioBufferData24_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData24_LAPLACIAN_PYRAMID_LAST, 8},
{4, 2, 208, 0, 1, gScenarioBufferData25_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData25_LAPLACIAN_PYRAMID_LAST, 8},
{6, 2, 300, 0, 1, gScenarioBufferData26_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData26_LAPLACIAN_PYRAMID_LAST, 8},
{8, 2, 392, 0, 1, gScenarioBufferData27_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData27_LAPLACIAN_PYRAMID_LAST, 8},
{10, 2, 484, 0, 1, gScenarioBufferData28_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData28_LAPLACIAN_PYRAMID_LAST, 8},
{12, 2, 576, 0, 1, gScenarioBufferData29_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData29_LAPLACIAN_PYRAMID_LAST, 8},
{14, 2, 668, 0, 1, gScenarioBufferData30_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData30_LAPLACIAN_PYRAMID_LAST, 8},
{16, 2, 760, 0, 1, gScenarioBufferData31_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData31_LAPLACIAN_PYRAMID_LAST, 8},
{18, 2, 852, 0, 1, gScenarioBufferData32_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData32_LAPLACIAN_PYRAMID_LAST, 8},
{20, 2, 944, 0, 1, gScenarioBufferData33_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData33_LAPLACIAN_PYRAMID_LAST, 8},
{22, 2, 1036, 0, 1, gScenarioBufferData34_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData34_LAPLACIAN_PYRAMID_LAST, 8},
{24, 2, 1128, 0, 1, gScenarioBufferData35_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData35_LAPLACIAN_PYRAMID_LAST, 8},
{26, 2, 1220, 0, 1, gScenarioBufferData36_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData36_LAPLACIAN_PYRAMID_LAST, 8},
{28, 2, 1312, 0, 1, gScenarioBufferData37_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData37_LAPLACIAN_PYRAMID_LAST, 8},
{30, 2, 1404, 0, 1, gScenarioBufferData38_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData38_LAPLACIAN_PYRAMID_LAST, 8},
{32, 2, 1496, 0, 1, gScenarioBufferData39_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData39_LAPLACIAN_PYRAMID_LAST, 8},
{36, 2, 1680, 0, 1, gScenarioBufferData40_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData40_LAPLACIAN_PYRAMID_LAST, 8},
{40, 2, 1864, 0, 1, gScenarioBufferData41_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData41_LAPLACIAN_PYRAMID_LAST, 8},
{44, 2, 2048, 0, 1, gScenarioBufferData42_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData42_LAPLACIAN_PYRAMID_LAST, 8},
{48, 2, 2232, 0, 1, gScenarioBufferData43_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData43_LAPLACIAN_PYRAMID_LAST, 8},
{52, 2, 2416, 0, 1, gScenarioBufferData44_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData44_LAPLACIAN_PYRAMID_LAST, 8},
{56, 2, 2600, 0, 1, gScenarioBufferData45_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData45_LAPLACIAN_PYRAMID_LAST, 8},
{60, 2, 2784, 0, 1, gScenarioBufferData46_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData46_LAPLACIAN_PYRAMID_LAST, 8},
{64, 2, 2968, 0, 1, gScenarioBufferData47_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData47_LAPLACIAN_PYRAMID_LAST, 8},
{2, 3, 168, 0, 1, gScenarioBufferData48_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData48_LAPLACIAN_PYRAMID_LAST, 8},
{4, 3, 296, 0, 1, gScenarioBufferData49_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData49_LAPLACIAN_PYRAMID_LAST, 8},
{6, 3, 428, 0, 1, gScenarioBufferData50_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData50_LAPLACIAN_PYRAMID_LAST, 8},
{8, 3, 556, 0, 1, gScenarioBufferData51_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData51_LAPLACIAN_PYRAMID_LAST, 8},
{10, 3, 688, 0, 1, gScenarioBufferData52_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData52_LAPLACIAN_PYRAMID_LAST, 8},
{12, 3, 816, 0, 1, gScenarioBufferData53_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData53_LAPLACIAN_PYRAMID_LAST, 8},
{14, 3, 948, 0, 1, gScenarioBufferData54_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData54_LAPLACIAN_PYRAMID_LAST, 8},
{16, 3, 1076, 0, 1, gScenarioBufferData55_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData55_LAPLACIAN_PYRAMID_LAST, 8},
{18, 3, 1208, 0, 1, gScenarioBufferData56_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData56_LAPLACIAN_PYRAMID_LAST, 8},
{20, 3, 1336, 0, 1, gScenarioBufferData57_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData57_LAPLACIAN_PYRAMID_LAST, 8},
{22, 3, 1468, 0, 1, gScenarioBufferData58_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData58_LAPLACIAN_PYRAMID_LAST, 8},
{24, 3, 1596, 0, 1, gScenarioBufferData59_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData59_LAPLACIAN_PYRAMID_LAST, 8},
{26, 3, 1728, 0, 1, gScenarioBufferData60_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData60_LAPLACIAN_PYRAMID_LAST, 8},
{28, 3, 1856, 0, 1, gScenarioBufferData61_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData61_LAPLACIAN_PYRAMID_LAST, 8},
{30, 3, 1988, 0, 1, gScenarioBufferData62_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData62_LAPLACIAN_PYRAMID_LAST, 8},
{32, 3, 2116, 0, 1, gScenarioBufferData63_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData63_LAPLACIAN_PYRAMID_LAST, 8},
{36, 3, 2376, 0, 1, gScenarioBufferData64_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData64_LAPLACIAN_PYRAMID_LAST, 8},
{40, 3, 2636, 0, 1, gScenarioBufferData65_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData65_LAPLACIAN_PYRAMID_LAST, 8},
{44, 3, 2896, 0, 1, gScenarioBufferData66_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData66_LAPLACIAN_PYRAMID_LAST, 8},
{48, 3, 3156, 0, 1, gScenarioBufferData67_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData67_LAPLACIAN_PYRAMID_LAST, 8},
{52, 3, 3416, 0, 1, gScenarioBufferData68_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData68_LAPLACIAN_PYRAMID_LAST, 8},
{56, 3, 3676, 0, 1, gScenarioBufferData69_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData69_LAPLACIAN_PYRAMID_LAST, 8},
{2, 4, 216, 0, 1, gScenarioBufferData70_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData70_LAPLACIAN_PYRAMID_LAST, 8},
{4, 4, 384, 0, 1, gScenarioBufferData71_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData71_LAPLACIAN_PYRAMID_LAST, 8},
{6, 4, 552, 0, 1, gScenarioBufferData72_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData72_LAPLACIAN_PYRAMID_LAST, 8},
{8, 4, 720, 0, 1, gScenarioBufferData73_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData73_LAPLACIAN_PYRAMID_LAST, 8},
{10, 4, 888, 0, 1, gScenarioBufferData74_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData74_LAPLACIAN_PYRAMID_LAST, 8},
{12, 4, 1056, 0, 1, gScenarioBufferData75_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData75_LAPLACIAN_PYRAMID_LAST, 8},
{14, 4, 1224, 0, 1, gScenarioBufferData76_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData76_LAPLACIAN_PYRAMID_LAST, 8},
{16, 4, 1392, 0, 1, gScenarioBufferData77_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData77_LAPLACIAN_PYRAMID_LAST, 8},
{18, 4, 1560, 0, 1, gScenarioBufferData78_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData78_LAPLACIAN_PYRAMID_LAST, 8},
{20, 4, 1728, 0, 1, gScenarioBufferData79_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData79_LAPLACIAN_PYRAMID_LAST, 8},
{22, 4, 1896, 0, 1, gScenarioBufferData80_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData80_LAPLACIAN_PYRAMID_LAST, 8},
{24, 4, 2064, 0, 1, gScenarioBufferData81_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData81_LAPLACIAN_PYRAMID_LAST, 8},
{26, 4, 2232, 0, 1, gScenarioBufferData82_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData82_LAPLACIAN_PYRAMID_LAST, 8},
{28, 4, 2400, 0, 1, gScenarioBufferData83_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData83_LAPLACIAN_PYRAMID_LAST, 8},
{30, 4, 2568, 0, 1, gScenarioBufferData84_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData84_LAPLACIAN_PYRAMID_LAST, 8},
{32, 4, 2736, 0, 1, gScenarioBufferData85_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData85_LAPLACIAN_PYRAMID_LAST, 8},
{36, 4, 3072, 0, 1, gScenarioBufferData86_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData86_LAPLACIAN_PYRAMID_LAST, 8},
{40, 4, 3408, 0, 1, gScenarioBufferData87_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData87_LAPLACIAN_PYRAMID_LAST, 8},
{44, 4, 3744, 0, 1, gScenarioBufferData88_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData88_LAPLACIAN_PYRAMID_LAST, 8},
{2, 5, 268, 0, 1, gScenarioBufferData89_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData89_LAPLACIAN_PYRAMID_LAST, 8},
{4, 5, 472, 0, 1, gScenarioBufferData90_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData90_LAPLACIAN_PYRAMID_LAST, 8},
{6, 5, 680, 0, 1, gScenarioBufferData91_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData91_LAPLACIAN_PYRAMID_LAST, 8},
{8, 5, 884, 0, 1, gScenarioBufferData92_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData92_LAPLACIAN_PYRAMID_LAST, 8},
{10, 5, 1092, 0, 1, gScenarioBufferData93_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData93_LAPLACIAN_PYRAMID_LAST, 8},
{12, 5, 1296, 0, 1, gScenarioBufferData94_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData94_LAPLACIAN_PYRAMID_LAST, 8},
{14, 5, 1504, 0, 1, gScenarioBufferData95_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData95_LAPLACIAN_PYRAMID_LAST, 8},
{16, 5, 1708, 0, 1, gScenarioBufferData96_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData96_LAPLACIAN_PYRAMID_LAST, 8},
{18, 5, 1916, 0, 1, gScenarioBufferData97_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData97_LAPLACIAN_PYRAMID_LAST, 8},
{20, 5, 2120, 0, 1, gScenarioBufferData98_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData98_LAPLACIAN_PYRAMID_LAST, 8},
{22, 5, 2328, 0, 1, gScenarioBufferData99_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData99_LAPLACIAN_PYRAMID_LAST, 8},
{24, 5, 2532, 0, 1, gScenarioBufferData100_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData100_LAPLACIAN_PYRAMID_LAST, 8},
{26, 5, 2740, 0, 1, gScenarioBufferData101_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData101_LAPLACIAN_PYRAMID_LAST, 8},
{28, 5, 2944, 0, 1, gScenarioBufferData102_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData102_LAPLACIAN_PYRAMID_LAST, 8},
{30, 5, 3152, 0, 1, gScenarioBufferData103_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData103_LAPLACIAN_PYRAMID_LAST, 8},
{32, 5, 3356, 0, 1, gScenarioBufferData104_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData104_LAPLACIAN_PYRAMID_LAST, 8},
{36, 5, 3768, 0, 1, gScenarioBufferData105_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData105_LAPLACIAN_PYRAMID_LAST, 8},
{2, 6, 316, 0, 1, gScenarioBufferData106_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData106_LAPLACIAN_PYRAMID_LAST, 8},
{4, 6, 560, 0, 1, gScenarioBufferData107_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData107_LAPLACIAN_PYRAMID_LAST, 8},
{6, 6, 804, 0, 1, gScenarioBufferData108_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData108_LAPLACIAN_PYRAMID_LAST, 8},
{8, 6, 1048, 0, 1, gScenarioBufferData109_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData109_LAPLACIAN_PYRAMID_LAST, 8},
{10, 6, 1292, 0, 1, gScenarioBufferData110_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData110_LAPLACIAN_PYRAMID_LAST, 8},
{12, 6, 1536, 0, 1, gScenarioBufferData111_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData111_LAPLACIAN_PYRAMID_LAST, 8},
{14, 6, 1780, 0, 1, gScenarioBufferData112_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData112_LAPLACIAN_PYRAMID_LAST, 8},
{16, 6, 2024, 0, 1, gScenarioBufferData113_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData113_LAPLACIAN_PYRAMID_LAST, 8},
{18, 6, 2268, 0, 1, gScenarioBufferData114_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData114_LAPLACIAN_PYRAMID_LAST, 8},
{20, 6, 2512, 0, 1, gScenarioBufferData115_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData115_LAPLACIAN_PYRAMID_LAST, 8},
{22, 6, 2756, 0, 1, gScenarioBufferData116_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData116_LAPLACIAN_PYRAMID_LAST, 8},
{24, 6, 3000, 0, 1, gScenarioBufferData117_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData117_LAPLACIAN_PYRAMID_LAST, 8},
{26, 6, 3244, 0, 1, gScenarioBufferData118_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData118_LAPLACIAN_PYRAMID_LAST, 8},
{28, 6, 3488, 0, 1, gScenarioBufferData119_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData119_LAPLACIAN_PYRAMID_LAST, 8},
{30, 6, 3732, 0, 1, gScenarioBufferData120_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData120_LAPLACIAN_PYRAMID_LAST, 8},
{2, 8, 416, 0, 1, gScenarioBufferData121_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData121_LAPLACIAN_PYRAMID_LAST, 8},
{4, 8, 736, 0, 1, gScenarioBufferData122_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData122_LAPLACIAN_PYRAMID_LAST, 8},
{6, 8, 1056, 0, 1, gScenarioBufferData123_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData123_LAPLACIAN_PYRAMID_LAST, 8},
{8, 8, 1376, 0, 1, gScenarioBufferData124_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData124_LAPLACIAN_PYRAMID_LAST, 8},
{10, 8, 1696, 0, 1, gScenarioBufferData125_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData125_LAPLACIAN_PYRAMID_LAST, 8},
{12, 8, 2016, 0, 1, gScenarioBufferData126_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData126_LAPLACIAN_PYRAMID_LAST, 8},
{14, 8, 2336, 0, 1, gScenarioBufferData127_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData127_LAPLACIAN_PYRAMID_LAST, 8},
{16, 8, 2656, 0, 1, gScenarioBufferData128_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData128_LAPLACIAN_PYRAMID_LAST, 8},
{18, 8, 2976, 0, 1, gScenarioBufferData129_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData129_LAPLACIAN_PYRAMID_LAST, 8},
{20, 8, 3296, 0, 1, gScenarioBufferData130_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData130_LAPLACIAN_PYRAMID_LAST, 8},
{22, 8, 3616, 0, 1, gScenarioBufferData131_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData131_LAPLACIAN_PYRAMID_LAST, 8},
{2, 10, 516, 0, 1, gScenarioBufferData132_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData132_LAPLACIAN_PYRAMID_LAST, 8},
{4, 10, 912, 0, 1, gScenarioBufferData133_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData133_LAPLACIAN_PYRAMID_LAST, 8},
{6, 10, 1308, 0, 1, gScenarioBufferData134_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData134_LAPLACIAN_PYRAMID_LAST, 8},
{8, 10, 1704, 0, 1, gScenarioBufferData135_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData135_LAPLACIAN_PYRAMID_LAST, 8},
{10, 10, 2100, 0, 1, gScenarioBufferData136_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData136_LAPLACIAN_PYRAMID_LAST, 8},
{12, 10, 2496, 0, 1, gScenarioBufferData137_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData137_LAPLACIAN_PYRAMID_LAST, 8},
{14, 10, 2892, 0, 1, gScenarioBufferData138_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData138_LAPLACIAN_PYRAMID_LAST, 8},
{16, 10, 3288, 0, 1, gScenarioBufferData139_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData139_LAPLACIAN_PYRAMID_LAST, 8},
{18, 10, 3684, 0, 1, gScenarioBufferData140_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData140_LAPLACIAN_PYRAMID_LAST, 8},
{2, 12, 616, 0, 1, gScenarioBufferData141_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData141_LAPLACIAN_PYRAMID_LAST, 8},
{4, 12, 1088, 0, 1, gScenarioBufferData142_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData142_LAPLACIAN_PYRAMID_LAST, 8},
{6, 12, 1560, 0, 1, gScenarioBufferData143_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData143_LAPLACIAN_PYRAMID_LAST, 8},
{8, 12, 2032, 0, 1, gScenarioBufferData144_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData144_LAPLACIAN_PYRAMID_LAST, 8},
{10, 12, 2504, 0, 1, gScenarioBufferData145_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData145_LAPLACIAN_PYRAMID_LAST, 8},
{12, 12, 2976, 0, 1, gScenarioBufferData146_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData146_LAPLACIAN_PYRAMID_LAST, 8},
{14, 12, 3448, 0, 1, gScenarioBufferData147_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData147_LAPLACIAN_PYRAMID_LAST, 8},
{2, 14, 716, 0, 1, gScenarioBufferData148_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData148_LAPLACIAN_PYRAMID_LAST, 8},
{4, 14, 1264, 0, 1, gScenarioBufferData149_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData149_LAPLACIAN_PYRAMID_LAST, 8},
{6, 14, 1812, 0, 1, gScenarioBufferData150_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData150_LAPLACIAN_PYRAMID_LAST, 8},
{8, 14, 2360, 0, 1, gScenarioBufferData151_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData151_LAPLACIAN_PYRAMID_LAST, 8},
{10, 14, 2908, 0, 1, gScenarioBufferData152_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData152_LAPLACIAN_PYRAMID_LAST, 8},
{12, 14, 3456, 0, 1, gScenarioBufferData153_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData153_LAPLACIAN_PYRAMID_LAST, 8},
{2, 16, 816, 0, 1, gScenarioBufferData154_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData154_LAPLACIAN_PYRAMID_LAST, 8},
{4, 16, 1440, 0, 1, gScenarioBufferData155_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData155_LAPLACIAN_PYRAMID_LAST, 8},
{6, 16, 2064, 0, 1, gScenarioBufferData156_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData156_LAPLACIAN_PYRAMID_LAST, 8},
{8, 16, 2688, 0, 1, gScenarioBufferData157_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData157_LAPLACIAN_PYRAMID_LAST, 8},
{10, 16, 3312, 0, 1, gScenarioBufferData158_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData158_LAPLACIAN_PYRAMID_LAST, 8},
{2, 18, 916, 0, 1, gScenarioBufferData159_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData159_LAPLACIAN_PYRAMID_LAST, 8},
{4, 18, 1616, 0, 1, gScenarioBufferData160_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData160_LAPLACIAN_PYRAMID_LAST, 8},
{6, 18, 2316, 0, 1, gScenarioBufferData161_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData161_LAPLACIAN_PYRAMID_LAST, 8},
{8, 18, 3016, 0, 1, gScenarioBufferData162_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData162_LAPLACIAN_PYRAMID_LAST, 8},
{10, 18, 3716, 0, 1, gScenarioBufferData163_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData163_LAPLACIAN_PYRAMID_LAST, 8},
{2, 20, 1016, 0, 1, gScenarioBufferData164_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData164_LAPLACIAN_PYRAMID_LAST, 8},
{4, 20, 1792, 0, 1, gScenarioBufferData165_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData165_LAPLACIAN_PYRAMID_LAST, 8},
{6, 20, 2568, 0, 1, gScenarioBufferData166_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData166_LAPLACIAN_PYRAMID_LAST, 8},
{8, 20, 3344, 0, 1, gScenarioBufferData167_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData167_LAPLACIAN_PYRAMID_LAST, 8},
{2, 22, 1116, 0, 1, gScenarioBufferData168_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData168_LAPLACIAN_PYRAMID_LAST, 8},
{4, 22, 1968, 0, 1, gScenarioBufferData169_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData169_LAPLACIAN_PYRAMID_LAST, 8},
{6, 22, 2820, 0, 1, gScenarioBufferData170_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData170_LAPLACIAN_PYRAMID_LAST, 8},
{8, 22, 3672, 0, 1, gScenarioBufferData171_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData171_LAPLACIAN_PYRAMID_LAST, 8},
{2, 24, 1216, 0, 1, gScenarioBufferData172_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData172_LAPLACIAN_PYRAMID_LAST, 8},
{4, 24, 2144, 0, 1, gScenarioBufferData173_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData173_LAPLACIAN_PYRAMID_LAST, 8},
{6, 24, 3072, 0, 1, gScenarioBufferData174_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData174_LAPLACIAN_PYRAMID_LAST, 8},
{2, 26, 1316, 0, 1, gScenarioBufferData175_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData175_LAPLACIAN_PYRAMID_LAST, 8},
{4, 26, 2320, 0, 1, gScenarioBufferData176_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData176_LAPLACIAN_PYRAMID_LAST, 8},
{6, 26, 3324, 0, 1, gScenarioBufferData177_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData177_LAPLACIAN_PYRAMID_LAST, 8},
{2, 28, 1416, 0, 1, gScenarioBufferData178_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData178_LAPLACIAN_PYRAMID_LAST, 8},
{4, 28, 2496, 0, 1, gScenarioBufferData179_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData179_LAPLACIAN_PYRAMID_LAST, 8},
{6, 28, 3576, 0, 1, gScenarioBufferData180_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData180_LAPLACIAN_PYRAMID_LAST, 8},
{2, 30, 1516, 0, 1, gScenarioBufferData181_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData181_LAPLACIAN_PYRAMID_LAST, 8},
{4, 30, 2672, 0, 1, gScenarioBufferData182_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData182_LAPLACIAN_PYRAMID_LAST, 8},
{6, 30, 3828, 0, 1, gScenarioBufferData183_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData183_LAPLACIAN_PYRAMID_LAST, 8},
{2, 32, 1616, 0, 1, gScenarioBufferData184_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData184_LAPLACIAN_PYRAMID_LAST, 8},
{4, 32, 2848, 0, 1, gScenarioBufferData185_LAPLACIAN_PYRAMID_LAST, 64, gScenarioKernelData185_LAPLACIAN_PYRAMID_LAST, 8}
};

static acf_scenario_list gScenarioList_LAPLACIAN_PYRAMID_LAST = {
186, //number of scenarios
gScenarioArray_LAPLACIAN_PYRAMID_LAST};
//**************************************************************

class LAPLACIAN_PYRAMID_LAST : public ACF_Process_APU
{

public:
   LAPLACIAN_PYRAMID_LAST(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LAPLACIAN_PYRAMID_LAST");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LAPLACIAN_PYRAMID_LAST",
                                        LAPLACIAN_PYRAMID_LAST_LOAD_SEGMENTS,
                                        LAPLACIAN_PYRAMID_LAST_LOAD_PMEM, LAPLACIAN_PYRAMID_LAST_LOAD_PMEM_SIZE,
                                        LAPLACIAN_PYRAMID_LAST_LOAD_DMEM, LAPLACIAN_PYRAMID_LAST_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LAPLACIAN_PYRAMID_LAST_APEX_LOG_BUFFER, LAPLACIAN_PYRAMID_LAST_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT_PYRAMID", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_RECONSTRUCT", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LAPLACIAN_PYRAMID_LAST);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LAPLACIAN_PYRAMID_LAST

#ifndef _ACF_PROCESS_APU_ADD_08U_16S_16S
#define _ACF_PROCESS_APU_ADD_08U_16S_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <ADD_08U_16S_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_ADD_08U_16S_16S[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_ADD_08U_16S_16S[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_ADD_08U_16S_16S[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_ADD_08U_16S_16S[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_ADD_08U_16S_16S[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_ADD_08U_16S_16S[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_ADD_08U_16S_16S[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_ADD_08U_16S_16S[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_ADD_08U_16S_16S[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_ADD_08U_16S_16S[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_ADD_08U_16S_16S[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_ADD_08U_16S_16S[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_ADD_08U_16S_16S[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_ADD_08U_16S_16S[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_ADD_08U_16S_16S[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_ADD_08U_16S_16S[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_ADD_08U_16S_16S[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_ADD_08U_16S_16S[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_ADD_08U_16S_16S[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_ADD_08U_16S_16S[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_ADD_08U_16S_16S[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_ADD_08U_16S_16S[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_ADD_08U_16S_16S[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_ADD_08U_16S_16S[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_ADD_08U_16S_16S[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_ADD_08U_16S_16S[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_ADD_08U_16S_16S[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_ADD_08U_16S_16S[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_ADD_08U_16S_16S[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_ADD_08U_16S_16S[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_ADD_08U_16S_16S[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_ADD_08U_16S_16S[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_ADD_08U_16S_16S[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_ADD_08U_16S_16S[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_ADD_08U_16S_16S[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_ADD_08U_16S_16S[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_ADD_08U_16S_16S[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_ADD_08U_16S_16S[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_ADD_08U_16S_16S[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_ADD_08U_16S_16S[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_ADD_08U_16S_16S[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_ADD_08U_16S_16S[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_ADD_08U_16S_16S[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_ADD_08U_16S_16S[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_ADD_08U_16S_16S[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_ADD_08U_16S_16S[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_ADD_08U_16S_16S[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_ADD_08U_16S_16S[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_ADD_08U_16S_16S[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_ADD_08U_16S_16S[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_ADD_08U_16S_16S[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_ADD_08U_16S_16S[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_ADD_08U_16S_16S[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_ADD_08U_16S_16S[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_ADD_08U_16S_16S[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_ADD_08U_16S_16S[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_ADD_08U_16S_16S[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_ADD_08U_16S_16S[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_ADD_08U_16S_16S[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_ADD_08U_16S_16S[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_ADD_08U_16S_16S[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_ADD_08U_16S_16S[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_ADD_08U_16S_16S[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_ADD_08U_16S_16S[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_ADD_08U_16S_16S[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_ADD_08U_16S_16S[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_ADD_08U_16S_16S[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_ADD_08U_16S_16S[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_ADD_08U_16S_16S[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_ADD_08U_16S_16S[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_ADD_08U_16S_16S[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_ADD_08U_16S_16S[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_ADD_08U_16S_16S[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_ADD_08U_16S_16S[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_ADD_08U_16S_16S[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_ADD_08U_16S_16S[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_ADD_08U_16S_16S[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_ADD_08U_16S_16S[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_ADD_08U_16S_16S[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_ADD_08U_16S_16S[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_ADD_08U_16S_16S[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_ADD_08U_16S_16S[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_ADD_08U_16S_16S[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_ADD_08U_16S_16S[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_ADD_08U_16S_16S[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_ADD_08U_16S_16S[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_ADD_08U_16S_16S[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_ADD_08U_16S_16S[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_ADD_08U_16S_16S[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_ADD_08U_16S_16S[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_ADD_08U_16S_16S[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_ADD_08U_16S_16S[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_ADD_08U_16S_16S[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_ADD_08U_16S_16S[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_ADD_08U_16S_16S[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_ADD_08U_16S_16S[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_ADD_08U_16S_16S[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_ADD_08U_16S_16S[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_ADD_08U_16S_16S[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_ADD_08U_16S_16S[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_ADD_08U_16S_16S[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_ADD_08U_16S_16S[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_ADD_08U_16S_16S[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_ADD_08U_16S_16S[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_ADD_08U_16S_16S[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_ADD_08U_16S_16S[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_ADD_08U_16S_16S[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_ADD_08U_16S_16S[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_ADD_08U_16S_16S[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_ADD_08U_16S_16S[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_ADD_08U_16S_16S[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_ADD_08U_16S_16S[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_ADD_08U_16S_16S[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_ADD_08U_16S_16S[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_ADD_08U_16S_16S[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_ADD_08U_16S_16S[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_ADD_08U_16S_16S[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_ADD_08U_16S_16S[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_ADD_08U_16S_16S[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_ADD_08U_16S_16S[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_ADD_08U_16S_16S[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_ADD_08U_16S_16S[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_ADD_08U_16S_16S[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_ADD_08U_16S_16S[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_ADD_08U_16S_16S[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_ADD_08U_16S_16S[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_ADD_08U_16S_16S[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_ADD_08U_16S_16S[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_ADD_08U_16S_16S[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_ADD_08U_16S_16S[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_ADD_08U_16S_16S[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_ADD_08U_16S_16S[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_ADD_08U_16S_16S[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_ADD_08U_16S_16S[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_ADD_08U_16S_16S[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_ADD_08U_16S_16S[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_ADD_08U_16S_16S[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_ADD_08U_16S_16S[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_ADD_08U_16S_16S[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_ADD_08U_16S_16S[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_ADD_08U_16S_16S[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_ADD_08U_16S_16S[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_ADD_08U_16S_16S[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_ADD_08U_16S_16S[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_ADD_08U_16S_16S[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_ADD_08U_16S_16S[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_ADD_08U_16S_16S[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_ADD_08U_16S_16S[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_ADD_08U_16S_16S[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_ADD_08U_16S_16S[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_ADD_08U_16S_16S[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_ADD_08U_16S_16S[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_ADD_08U_16S_16S[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_ADD_08U_16S_16S[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_ADD_08U_16S_16S[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_ADD_08U_16S_16S[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_ADD_08U_16S_16S[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_ADD_08U_16S_16S[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_ADD_08U_16S_16S[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_ADD_08U_16S_16S[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_ADD_08U_16S_16S[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_ADD_08U_16S_16S[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_ADD_08U_16S_16S[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_ADD_08U_16S_16S[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_ADD_08U_16S_16S[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_ADD_08U_16S_16S[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_ADD_08U_16S_16S[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_ADD_08U_16S_16S[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_ADD_08U_16S_16S[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_ADD_08U_16S_16S[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_ADD_08U_16S_16S[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_ADD_08U_16S_16S[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_ADD_08U_16S_16S[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_ADD_08U_16S_16S[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_ADD_08U_16S_16S[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_ADD_08U_16S_16S[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_ADD_08U_16S_16S[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_ADD_08U_16S_16S[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_ADD_08U_16S_16S[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_ADD_08U_16S_16S[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_ADD_08U_16S_16S[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_ADD_08U_16S_16S[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_ADD_08U_16S_16S[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_ADD_08U_16S_16S[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_ADD_08U_16S_16S[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_ADD_08U_16S_16S[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_ADD_08U_16S_16S[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_ADD_08U_16S_16S[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_ADD_08U_16S_16S[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_ADD_08U_16S_16S[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_ADD_08U_16S_16S[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_ADD_08U_16S_16S[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_ADD_08U_16S_16S[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_ADD_08U_16S_16S[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_ADD_08U_16S_16S[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_ADD_08U_16S_16S[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_ADD_08U_16S_16S[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_ADD_08U_16S_16S[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_ADD_08U_16S_16S[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_ADD_08U_16S_16S[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_ADD_08U_16S_16S[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_ADD_08U_16S_16S[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_ADD_08U_16S_16S[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_ADD_08U_16S_16S[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_ADD_08U_16S_16S[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_ADD_08U_16S_16S[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_ADD_08U_16S_16S[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_ADD_08U_16S_16S[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_ADD_08U_16S_16S[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_ADD_08U_16S_16S[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_ADD_08U_16S_16S[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_ADD_08U_16S_16S[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_ADD_08U_16S_16S[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_ADD_08U_16S_16S[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_ADD_08U_16S_16S[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_ADD_08U_16S_16S[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_ADD_08U_16S_16S[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_ADD_08U_16S_16S[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_ADD_08U_16S_16S[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_ADD_08U_16S_16S[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_ADD_08U_16S_16S[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_ADD_08U_16S_16S[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_ADD_08U_16S_16S[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_ADD_08U_16S_16S[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_ADD_08U_16S_16S[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_ADD_08U_16S_16S[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_ADD_08U_16S_16S[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_ADD_08U_16S_16S[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_ADD_08U_16S_16S[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_ADD_08U_16S_16S[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_ADD_08U_16S_16S[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_ADD_08U_16S_16S[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_ADD_08U_16S_16S[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_ADD_08U_16S_16S[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_ADD_08U_16S_16S[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_ADD_08U_16S_16S[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_ADD_08U_16S_16S[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_ADD_08U_16S_16S[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_ADD_08U_16S_16S[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_ADD_08U_16S_16S[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_ADD_08U_16S_16S[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_ADD_08U_16S_16S[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_ADD_08U_16S_16S[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_ADD_08U_16S_16S[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_ADD_08U_16S_16S[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_ADD_08U_16S_16S[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_ADD_08U_16S_16S[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_ADD_08U_16S_16S[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_ADD_08U_16S_16S[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_ADD_08U_16S_16S[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_ADD_08U_16S_16S[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_ADD_08U_16S_16S[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_ADD_08U_16S_16S[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_ADD_08U_16S_16S[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_ADD_08U_16S_16S[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_ADD_08U_16S_16S[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_ADD_08U_16S_16S[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_ADD_08U_16S_16S[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_ADD_08U_16S_16S[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_ADD_08U_16S_16S[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_ADD_08U_16S_16S[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_ADD_08U_16S_16S[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_ADD_08U_16S_16S[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_ADD_08U_16S_16S[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_ADD_08U_16S_16S[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_ADD_08U_16S_16S[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_ADD_08U_16S_16S[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_ADD_08U_16S_16S[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_ADD_08U_16S_16S[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_ADD_08U_16S_16S[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_ADD_08U_16S_16S[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_ADD_08U_16S_16S[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_ADD_08U_16S_16S[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_ADD_08U_16S_16S[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_ADD_08U_16S_16S[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_ADD_08U_16S_16S[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_ADD_08U_16S_16S[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_ADD_08U_16S_16S[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_ADD_08U_16S_16S[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_ADD_08U_16S_16S[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_ADD_08U_16S_16S[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_ADD_08U_16S_16S[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_ADD_08U_16S_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_ADD_08U_16S_16S[] = {{0, 0}};

static acf_scenario gScenarioArray_ADD_08U_16S_16S[] = {
{2, 1, 20, 16, 0, gScenarioBufferData0_ADD_08U_16S_16S, 64, gScenarioKernelData0_ADD_08U_16S_16S, 4},
{4, 1, 40, 16, 0, gScenarioBufferData1_ADD_08U_16S_16S, 64, gScenarioKernelData1_ADD_08U_16S_16S, 4},
{6, 1, 60, 16, 0, gScenarioBufferData2_ADD_08U_16S_16S, 64, gScenarioKernelData2_ADD_08U_16S_16S, 4},
{8, 1, 80, 16, 0, gScenarioBufferData3_ADD_08U_16S_16S, 64, gScenarioKernelData3_ADD_08U_16S_16S, 4},
{10, 1, 100, 16, 0, gScenarioBufferData4_ADD_08U_16S_16S, 64, gScenarioKernelData4_ADD_08U_16S_16S, 4},
{12, 1, 120, 16, 0, gScenarioBufferData5_ADD_08U_16S_16S, 64, gScenarioKernelData5_ADD_08U_16S_16S, 4},
{14, 1, 140, 16, 0, gScenarioBufferData6_ADD_08U_16S_16S, 64, gScenarioKernelData6_ADD_08U_16S_16S, 4},
{16, 1, 160, 16, 0, gScenarioBufferData7_ADD_08U_16S_16S, 64, gScenarioKernelData7_ADD_08U_16S_16S, 4},
{18, 1, 180, 16, 0, gScenarioBufferData8_ADD_08U_16S_16S, 64, gScenarioKernelData8_ADD_08U_16S_16S, 4},
{20, 1, 200, 16, 0, gScenarioBufferData9_ADD_08U_16S_16S, 64, gScenarioKernelData9_ADD_08U_16S_16S, 4},
{22, 1, 220, 16, 0, gScenarioBufferData10_ADD_08U_16S_16S, 64, gScenarioKernelData10_ADD_08U_16S_16S, 4},
{24, 1, 240, 16, 0, gScenarioBufferData11_ADD_08U_16S_16S, 64, gScenarioKernelData11_ADD_08U_16S_16S, 4},
{26, 1, 260, 16, 0, gScenarioBufferData12_ADD_08U_16S_16S, 64, gScenarioKernelData12_ADD_08U_16S_16S, 4},
{28, 1, 280, 16, 0, gScenarioBufferData13_ADD_08U_16S_16S, 64, gScenarioKernelData13_ADD_08U_16S_16S, 4},
{30, 1, 300, 16, 0, gScenarioBufferData14_ADD_08U_16S_16S, 64, gScenarioKernelData14_ADD_08U_16S_16S, 4},
{32, 1, 320, 16, 0, gScenarioBufferData15_ADD_08U_16S_16S, 64, gScenarioKernelData15_ADD_08U_16S_16S, 4},
{36, 1, 360, 16, 0, gScenarioBufferData16_ADD_08U_16S_16S, 64, gScenarioKernelData16_ADD_08U_16S_16S, 4},
{40, 1, 400, 16, 0, gScenarioBufferData17_ADD_08U_16S_16S, 64, gScenarioKernelData17_ADD_08U_16S_16S, 4},
{44, 1, 440, 16, 0, gScenarioBufferData18_ADD_08U_16S_16S, 64, gScenarioKernelData18_ADD_08U_16S_16S, 4},
{48, 1, 480, 16, 0, gScenarioBufferData19_ADD_08U_16S_16S, 64, gScenarioKernelData19_ADD_08U_16S_16S, 4},
{52, 1, 520, 16, 0, gScenarioBufferData20_ADD_08U_16S_16S, 64, gScenarioKernelData20_ADD_08U_16S_16S, 4},
{56, 1, 560, 16, 0, gScenarioBufferData21_ADD_08U_16S_16S, 64, gScenarioKernelData21_ADD_08U_16S_16S, 4},
{60, 1, 600, 16, 0, gScenarioBufferData22_ADD_08U_16S_16S, 64, gScenarioKernelData22_ADD_08U_16S_16S, 4},
{64, 1, 640, 16, 0, gScenarioBufferData23_ADD_08U_16S_16S, 64, gScenarioKernelData23_ADD_08U_16S_16S, 4},
{2, 2, 40, 16, 0, gScenarioBufferData24_ADD_08U_16S_16S, 64, gScenarioKernelData24_ADD_08U_16S_16S, 4},
{4, 2, 80, 16, 0, gScenarioBufferData25_ADD_08U_16S_16S, 64, gScenarioKernelData25_ADD_08U_16S_16S, 4},
{6, 2, 120, 16, 0, gScenarioBufferData26_ADD_08U_16S_16S, 64, gScenarioKernelData26_ADD_08U_16S_16S, 4},
{8, 2, 160, 16, 0, gScenarioBufferData27_ADD_08U_16S_16S, 64, gScenarioKernelData27_ADD_08U_16S_16S, 4},
{10, 2, 200, 16, 0, gScenarioBufferData28_ADD_08U_16S_16S, 64, gScenarioKernelData28_ADD_08U_16S_16S, 4},
{12, 2, 240, 16, 0, gScenarioBufferData29_ADD_08U_16S_16S, 64, gScenarioKernelData29_ADD_08U_16S_16S, 4},
{14, 2, 280, 16, 0, gScenarioBufferData30_ADD_08U_16S_16S, 64, gScenarioKernelData30_ADD_08U_16S_16S, 4},
{16, 2, 320, 16, 0, gScenarioBufferData31_ADD_08U_16S_16S, 64, gScenarioKernelData31_ADD_08U_16S_16S, 4},
{18, 2, 360, 16, 0, gScenarioBufferData32_ADD_08U_16S_16S, 64, gScenarioKernelData32_ADD_08U_16S_16S, 4},
{20, 2, 400, 16, 0, gScenarioBufferData33_ADD_08U_16S_16S, 64, gScenarioKernelData33_ADD_08U_16S_16S, 4},
{22, 2, 440, 16, 0, gScenarioBufferData34_ADD_08U_16S_16S, 64, gScenarioKernelData34_ADD_08U_16S_16S, 4},
{24, 2, 480, 16, 0, gScenarioBufferData35_ADD_08U_16S_16S, 64, gScenarioKernelData35_ADD_08U_16S_16S, 4},
{26, 2, 520, 16, 0, gScenarioBufferData36_ADD_08U_16S_16S, 64, gScenarioKernelData36_ADD_08U_16S_16S, 4},
{28, 2, 560, 16, 0, gScenarioBufferData37_ADD_08U_16S_16S, 64, gScenarioKernelData37_ADD_08U_16S_16S, 4},
{30, 2, 600, 16, 0, gScenarioBufferData38_ADD_08U_16S_16S, 64, gScenarioKernelData38_ADD_08U_16S_16S, 4},
{32, 2, 640, 16, 0, gScenarioBufferData39_ADD_08U_16S_16S, 64, gScenarioKernelData39_ADD_08U_16S_16S, 4},
{36, 2, 720, 16, 0, gScenarioBufferData40_ADD_08U_16S_16S, 64, gScenarioKernelData40_ADD_08U_16S_16S, 4},
{40, 2, 800, 16, 0, gScenarioBufferData41_ADD_08U_16S_16S, 64, gScenarioKernelData41_ADD_08U_16S_16S, 4},
{44, 2, 880, 16, 0, gScenarioBufferData42_ADD_08U_16S_16S, 64, gScenarioKernelData42_ADD_08U_16S_16S, 4},
{48, 2, 960, 16, 0, gScenarioBufferData43_ADD_08U_16S_16S, 64, gScenarioKernelData43_ADD_08U_16S_16S, 4},
{52, 2, 1040, 16, 0, gScenarioBufferData44_ADD_08U_16S_16S, 64, gScenarioKernelData44_ADD_08U_16S_16S, 4},
{56, 2, 1120, 16, 0, gScenarioBufferData45_ADD_08U_16S_16S, 64, gScenarioKernelData45_ADD_08U_16S_16S, 4},
{60, 2, 1200, 16, 0, gScenarioBufferData46_ADD_08U_16S_16S, 64, gScenarioKernelData46_ADD_08U_16S_16S, 4},
{64, 2, 1280, 16, 0, gScenarioBufferData47_ADD_08U_16S_16S, 64, gScenarioKernelData47_ADD_08U_16S_16S, 4},
{2, 3, 60, 16, 0, gScenarioBufferData48_ADD_08U_16S_16S, 64, gScenarioKernelData48_ADD_08U_16S_16S, 4},
{4, 3, 120, 16, 0, gScenarioBufferData49_ADD_08U_16S_16S, 64, gScenarioKernelData49_ADD_08U_16S_16S, 4},
{6, 3, 180, 16, 0, gScenarioBufferData50_ADD_08U_16S_16S, 64, gScenarioKernelData50_ADD_08U_16S_16S, 4},
{8, 3, 240, 16, 0, gScenarioBufferData51_ADD_08U_16S_16S, 64, gScenarioKernelData51_ADD_08U_16S_16S, 4},
{10, 3, 300, 16, 0, gScenarioBufferData52_ADD_08U_16S_16S, 64, gScenarioKernelData52_ADD_08U_16S_16S, 4},
{12, 3, 360, 16, 0, gScenarioBufferData53_ADD_08U_16S_16S, 64, gScenarioKernelData53_ADD_08U_16S_16S, 4},
{14, 3, 420, 16, 0, gScenarioBufferData54_ADD_08U_16S_16S, 64, gScenarioKernelData54_ADD_08U_16S_16S, 4},
{16, 3, 480, 16, 0, gScenarioBufferData55_ADD_08U_16S_16S, 64, gScenarioKernelData55_ADD_08U_16S_16S, 4},
{18, 3, 540, 16, 0, gScenarioBufferData56_ADD_08U_16S_16S, 64, gScenarioKernelData56_ADD_08U_16S_16S, 4},
{20, 3, 600, 16, 0, gScenarioBufferData57_ADD_08U_16S_16S, 64, gScenarioKernelData57_ADD_08U_16S_16S, 4},
{22, 3, 660, 16, 0, gScenarioBufferData58_ADD_08U_16S_16S, 64, gScenarioKernelData58_ADD_08U_16S_16S, 4},
{24, 3, 720, 16, 0, gScenarioBufferData59_ADD_08U_16S_16S, 64, gScenarioKernelData59_ADD_08U_16S_16S, 4},
{26, 3, 780, 16, 0, gScenarioBufferData60_ADD_08U_16S_16S, 64, gScenarioKernelData60_ADD_08U_16S_16S, 4},
{28, 3, 840, 16, 0, gScenarioBufferData61_ADD_08U_16S_16S, 64, gScenarioKernelData61_ADD_08U_16S_16S, 4},
{30, 3, 900, 16, 0, gScenarioBufferData62_ADD_08U_16S_16S, 64, gScenarioKernelData62_ADD_08U_16S_16S, 4},
{32, 3, 960, 16, 0, gScenarioBufferData63_ADD_08U_16S_16S, 64, gScenarioKernelData63_ADD_08U_16S_16S, 4},
{36, 3, 1080, 16, 0, gScenarioBufferData64_ADD_08U_16S_16S, 64, gScenarioKernelData64_ADD_08U_16S_16S, 4},
{40, 3, 1200, 16, 0, gScenarioBufferData65_ADD_08U_16S_16S, 64, gScenarioKernelData65_ADD_08U_16S_16S, 4},
{44, 3, 1320, 16, 0, gScenarioBufferData66_ADD_08U_16S_16S, 64, gScenarioKernelData66_ADD_08U_16S_16S, 4},
{48, 3, 1440, 16, 0, gScenarioBufferData67_ADD_08U_16S_16S, 64, gScenarioKernelData67_ADD_08U_16S_16S, 4},
{52, 3, 1560, 16, 0, gScenarioBufferData68_ADD_08U_16S_16S, 64, gScenarioKernelData68_ADD_08U_16S_16S, 4},
{56, 3, 1680, 16, 0, gScenarioBufferData69_ADD_08U_16S_16S, 64, gScenarioKernelData69_ADD_08U_16S_16S, 4},
{60, 3, 1800, 16, 0, gScenarioBufferData70_ADD_08U_16S_16S, 64, gScenarioKernelData70_ADD_08U_16S_16S, 4},
{64, 3, 1920, 16, 0, gScenarioBufferData71_ADD_08U_16S_16S, 64, gScenarioKernelData71_ADD_08U_16S_16S, 4},
{2, 4, 80, 16, 0, gScenarioBufferData72_ADD_08U_16S_16S, 64, gScenarioKernelData72_ADD_08U_16S_16S, 4},
{4, 4, 160, 16, 0, gScenarioBufferData73_ADD_08U_16S_16S, 64, gScenarioKernelData73_ADD_08U_16S_16S, 4},
{6, 4, 240, 16, 0, gScenarioBufferData74_ADD_08U_16S_16S, 64, gScenarioKernelData74_ADD_08U_16S_16S, 4},
{8, 4, 320, 16, 0, gScenarioBufferData75_ADD_08U_16S_16S, 64, gScenarioKernelData75_ADD_08U_16S_16S, 4},
{10, 4, 400, 16, 0, gScenarioBufferData76_ADD_08U_16S_16S, 64, gScenarioKernelData76_ADD_08U_16S_16S, 4},
{12, 4, 480, 16, 0, gScenarioBufferData77_ADD_08U_16S_16S, 64, gScenarioKernelData77_ADD_08U_16S_16S, 4},
{14, 4, 560, 16, 0, gScenarioBufferData78_ADD_08U_16S_16S, 64, gScenarioKernelData78_ADD_08U_16S_16S, 4},
{16, 4, 640, 16, 0, gScenarioBufferData79_ADD_08U_16S_16S, 64, gScenarioKernelData79_ADD_08U_16S_16S, 4},
{18, 4, 720, 16, 0, gScenarioBufferData80_ADD_08U_16S_16S, 64, gScenarioKernelData80_ADD_08U_16S_16S, 4},
{20, 4, 800, 16, 0, gScenarioBufferData81_ADD_08U_16S_16S, 64, gScenarioKernelData81_ADD_08U_16S_16S, 4},
{22, 4, 880, 16, 0, gScenarioBufferData82_ADD_08U_16S_16S, 64, gScenarioKernelData82_ADD_08U_16S_16S, 4},
{24, 4, 960, 16, 0, gScenarioBufferData83_ADD_08U_16S_16S, 64, gScenarioKernelData83_ADD_08U_16S_16S, 4},
{26, 4, 1040, 16, 0, gScenarioBufferData84_ADD_08U_16S_16S, 64, gScenarioKernelData84_ADD_08U_16S_16S, 4},
{28, 4, 1120, 16, 0, gScenarioBufferData85_ADD_08U_16S_16S, 64, gScenarioKernelData85_ADD_08U_16S_16S, 4},
{30, 4, 1200, 16, 0, gScenarioBufferData86_ADD_08U_16S_16S, 64, gScenarioKernelData86_ADD_08U_16S_16S, 4},
{32, 4, 1280, 16, 0, gScenarioBufferData87_ADD_08U_16S_16S, 64, gScenarioKernelData87_ADD_08U_16S_16S, 4},
{36, 4, 1440, 16, 0, gScenarioBufferData88_ADD_08U_16S_16S, 64, gScenarioKernelData88_ADD_08U_16S_16S, 4},
{40, 4, 1600, 16, 0, gScenarioBufferData89_ADD_08U_16S_16S, 64, gScenarioKernelData89_ADD_08U_16S_16S, 4},
{44, 4, 1760, 16, 0, gScenarioBufferData90_ADD_08U_16S_16S, 64, gScenarioKernelData90_ADD_08U_16S_16S, 4},
{48, 4, 1920, 16, 0, gScenarioBufferData91_ADD_08U_16S_16S, 64, gScenarioKernelData91_ADD_08U_16S_16S, 4},
{52, 4, 2080, 16, 0, gScenarioBufferData92_ADD_08U_16S_16S, 64, gScenarioKernelData92_ADD_08U_16S_16S, 4},
{56, 4, 2240, 16, 0, gScenarioBufferData93_ADD_08U_16S_16S, 64, gScenarioKernelData93_ADD_08U_16S_16S, 4},
{60, 4, 2400, 16, 0, gScenarioBufferData94_ADD_08U_16S_16S, 64, gScenarioKernelData94_ADD_08U_16S_16S, 4},
{64, 4, 2560, 16, 0, gScenarioBufferData95_ADD_08U_16S_16S, 64, gScenarioKernelData95_ADD_08U_16S_16S, 4},
{2, 5, 100, 16, 0, gScenarioBufferData96_ADD_08U_16S_16S, 64, gScenarioKernelData96_ADD_08U_16S_16S, 4},
{4, 5, 200, 16, 0, gScenarioBufferData97_ADD_08U_16S_16S, 64, gScenarioKernelData97_ADD_08U_16S_16S, 4},
{6, 5, 300, 16, 0, gScenarioBufferData98_ADD_08U_16S_16S, 64, gScenarioKernelData98_ADD_08U_16S_16S, 4},
{8, 5, 400, 16, 0, gScenarioBufferData99_ADD_08U_16S_16S, 64, gScenarioKernelData99_ADD_08U_16S_16S, 4},
{10, 5, 500, 16, 0, gScenarioBufferData100_ADD_08U_16S_16S, 64, gScenarioKernelData100_ADD_08U_16S_16S, 4},
{12, 5, 600, 16, 0, gScenarioBufferData101_ADD_08U_16S_16S, 64, gScenarioKernelData101_ADD_08U_16S_16S, 4},
{14, 5, 700, 16, 0, gScenarioBufferData102_ADD_08U_16S_16S, 64, gScenarioKernelData102_ADD_08U_16S_16S, 4},
{16, 5, 800, 16, 0, gScenarioBufferData103_ADD_08U_16S_16S, 64, gScenarioKernelData103_ADD_08U_16S_16S, 4},
{18, 5, 900, 16, 0, gScenarioBufferData104_ADD_08U_16S_16S, 64, gScenarioKernelData104_ADD_08U_16S_16S, 4},
{20, 5, 1000, 16, 0, gScenarioBufferData105_ADD_08U_16S_16S, 64, gScenarioKernelData105_ADD_08U_16S_16S, 4},
{22, 5, 1100, 16, 0, gScenarioBufferData106_ADD_08U_16S_16S, 64, gScenarioKernelData106_ADD_08U_16S_16S, 4},
{24, 5, 1200, 16, 0, gScenarioBufferData107_ADD_08U_16S_16S, 64, gScenarioKernelData107_ADD_08U_16S_16S, 4},
{26, 5, 1300, 16, 0, gScenarioBufferData108_ADD_08U_16S_16S, 64, gScenarioKernelData108_ADD_08U_16S_16S, 4},
{28, 5, 1400, 16, 0, gScenarioBufferData109_ADD_08U_16S_16S, 64, gScenarioKernelData109_ADD_08U_16S_16S, 4},
{30, 5, 1500, 16, 0, gScenarioBufferData110_ADD_08U_16S_16S, 64, gScenarioKernelData110_ADD_08U_16S_16S, 4},
{32, 5, 1600, 16, 0, gScenarioBufferData111_ADD_08U_16S_16S, 64, gScenarioKernelData111_ADD_08U_16S_16S, 4},
{36, 5, 1800, 16, 0, gScenarioBufferData112_ADD_08U_16S_16S, 64, gScenarioKernelData112_ADD_08U_16S_16S, 4},
{40, 5, 2000, 16, 0, gScenarioBufferData113_ADD_08U_16S_16S, 64, gScenarioKernelData113_ADD_08U_16S_16S, 4},
{44, 5, 2200, 16, 0, gScenarioBufferData114_ADD_08U_16S_16S, 64, gScenarioKernelData114_ADD_08U_16S_16S, 4},
{48, 5, 2400, 16, 0, gScenarioBufferData115_ADD_08U_16S_16S, 64, gScenarioKernelData115_ADD_08U_16S_16S, 4},
{52, 5, 2600, 16, 0, gScenarioBufferData116_ADD_08U_16S_16S, 64, gScenarioKernelData116_ADD_08U_16S_16S, 4},
{56, 5, 2800, 16, 0, gScenarioBufferData117_ADD_08U_16S_16S, 64, gScenarioKernelData117_ADD_08U_16S_16S, 4},
{60, 5, 3000, 16, 0, gScenarioBufferData118_ADD_08U_16S_16S, 64, gScenarioKernelData118_ADD_08U_16S_16S, 4},
{64, 5, 3200, 16, 0, gScenarioBufferData119_ADD_08U_16S_16S, 64, gScenarioKernelData119_ADD_08U_16S_16S, 4},
{2, 6, 120, 16, 0, gScenarioBufferData120_ADD_08U_16S_16S, 64, gScenarioKernelData120_ADD_08U_16S_16S, 4},
{4, 6, 240, 16, 0, gScenarioBufferData121_ADD_08U_16S_16S, 64, gScenarioKernelData121_ADD_08U_16S_16S, 4},
{6, 6, 360, 16, 0, gScenarioBufferData122_ADD_08U_16S_16S, 64, gScenarioKernelData122_ADD_08U_16S_16S, 4},
{8, 6, 480, 16, 0, gScenarioBufferData123_ADD_08U_16S_16S, 64, gScenarioKernelData123_ADD_08U_16S_16S, 4},
{10, 6, 600, 16, 0, gScenarioBufferData124_ADD_08U_16S_16S, 64, gScenarioKernelData124_ADD_08U_16S_16S, 4},
{12, 6, 720, 16, 0, gScenarioBufferData125_ADD_08U_16S_16S, 64, gScenarioKernelData125_ADD_08U_16S_16S, 4},
{14, 6, 840, 16, 0, gScenarioBufferData126_ADD_08U_16S_16S, 64, gScenarioKernelData126_ADD_08U_16S_16S, 4},
{16, 6, 960, 16, 0, gScenarioBufferData127_ADD_08U_16S_16S, 64, gScenarioKernelData127_ADD_08U_16S_16S, 4},
{18, 6, 1080, 16, 0, gScenarioBufferData128_ADD_08U_16S_16S, 64, gScenarioKernelData128_ADD_08U_16S_16S, 4},
{20, 6, 1200, 16, 0, gScenarioBufferData129_ADD_08U_16S_16S, 64, gScenarioKernelData129_ADD_08U_16S_16S, 4},
{22, 6, 1320, 16, 0, gScenarioBufferData130_ADD_08U_16S_16S, 64, gScenarioKernelData130_ADD_08U_16S_16S, 4},
{24, 6, 1440, 16, 0, gScenarioBufferData131_ADD_08U_16S_16S, 64, gScenarioKernelData131_ADD_08U_16S_16S, 4},
{26, 6, 1560, 16, 0, gScenarioBufferData132_ADD_08U_16S_16S, 64, gScenarioKernelData132_ADD_08U_16S_16S, 4},
{28, 6, 1680, 16, 0, gScenarioBufferData133_ADD_08U_16S_16S, 64, gScenarioKernelData133_ADD_08U_16S_16S, 4},
{30, 6, 1800, 16, 0, gScenarioBufferData134_ADD_08U_16S_16S, 64, gScenarioKernelData134_ADD_08U_16S_16S, 4},
{32, 6, 1920, 16, 0, gScenarioBufferData135_ADD_08U_16S_16S, 64, gScenarioKernelData135_ADD_08U_16S_16S, 4},
{36, 6, 2160, 16, 0, gScenarioBufferData136_ADD_08U_16S_16S, 64, gScenarioKernelData136_ADD_08U_16S_16S, 4},
{40, 6, 2400, 16, 0, gScenarioBufferData137_ADD_08U_16S_16S, 64, gScenarioKernelData137_ADD_08U_16S_16S, 4},
{44, 6, 2640, 16, 0, gScenarioBufferData138_ADD_08U_16S_16S, 64, gScenarioKernelData138_ADD_08U_16S_16S, 4},
{48, 6, 2880, 16, 0, gScenarioBufferData139_ADD_08U_16S_16S, 64, gScenarioKernelData139_ADD_08U_16S_16S, 4},
{52, 6, 3120, 16, 0, gScenarioBufferData140_ADD_08U_16S_16S, 64, gScenarioKernelData140_ADD_08U_16S_16S, 4},
{56, 6, 3360, 16, 0, gScenarioBufferData141_ADD_08U_16S_16S, 64, gScenarioKernelData141_ADD_08U_16S_16S, 4},
{60, 6, 3600, 16, 0, gScenarioBufferData142_ADD_08U_16S_16S, 64, gScenarioKernelData142_ADD_08U_16S_16S, 4},
{64, 6, 3840, 16, 0, gScenarioBufferData143_ADD_08U_16S_16S, 64, gScenarioKernelData143_ADD_08U_16S_16S, 4},
{2, 8, 160, 16, 0, gScenarioBufferData144_ADD_08U_16S_16S, 64, gScenarioKernelData144_ADD_08U_16S_16S, 4},
{4, 8, 320, 16, 0, gScenarioBufferData145_ADD_08U_16S_16S, 64, gScenarioKernelData145_ADD_08U_16S_16S, 4},
{6, 8, 480, 16, 0, gScenarioBufferData146_ADD_08U_16S_16S, 64, gScenarioKernelData146_ADD_08U_16S_16S, 4},
{8, 8, 640, 16, 0, gScenarioBufferData147_ADD_08U_16S_16S, 64, gScenarioKernelData147_ADD_08U_16S_16S, 4},
{10, 8, 800, 16, 0, gScenarioBufferData148_ADD_08U_16S_16S, 64, gScenarioKernelData148_ADD_08U_16S_16S, 4},
{12, 8, 960, 16, 0, gScenarioBufferData149_ADD_08U_16S_16S, 64, gScenarioKernelData149_ADD_08U_16S_16S, 4},
{14, 8, 1120, 16, 0, gScenarioBufferData150_ADD_08U_16S_16S, 64, gScenarioKernelData150_ADD_08U_16S_16S, 4},
{16, 8, 1280, 16, 0, gScenarioBufferData151_ADD_08U_16S_16S, 64, gScenarioKernelData151_ADD_08U_16S_16S, 4},
{18, 8, 1440, 16, 0, gScenarioBufferData152_ADD_08U_16S_16S, 64, gScenarioKernelData152_ADD_08U_16S_16S, 4},
{20, 8, 1600, 16, 0, gScenarioBufferData153_ADD_08U_16S_16S, 64, gScenarioKernelData153_ADD_08U_16S_16S, 4},
{22, 8, 1760, 16, 0, gScenarioBufferData154_ADD_08U_16S_16S, 64, gScenarioKernelData154_ADD_08U_16S_16S, 4},
{24, 8, 1920, 16, 0, gScenarioBufferData155_ADD_08U_16S_16S, 64, gScenarioKernelData155_ADD_08U_16S_16S, 4},
{26, 8, 2080, 16, 0, gScenarioBufferData156_ADD_08U_16S_16S, 64, gScenarioKernelData156_ADD_08U_16S_16S, 4},
{28, 8, 2240, 16, 0, gScenarioBufferData157_ADD_08U_16S_16S, 64, gScenarioKernelData157_ADD_08U_16S_16S, 4},
{30, 8, 2400, 16, 0, gScenarioBufferData158_ADD_08U_16S_16S, 64, gScenarioKernelData158_ADD_08U_16S_16S, 4},
{32, 8, 2560, 16, 0, gScenarioBufferData159_ADD_08U_16S_16S, 64, gScenarioKernelData159_ADD_08U_16S_16S, 4},
{36, 8, 2880, 16, 0, gScenarioBufferData160_ADD_08U_16S_16S, 64, gScenarioKernelData160_ADD_08U_16S_16S, 4},
{40, 8, 3200, 16, 0, gScenarioBufferData161_ADD_08U_16S_16S, 64, gScenarioKernelData161_ADD_08U_16S_16S, 4},
{44, 8, 3520, 16, 0, gScenarioBufferData162_ADD_08U_16S_16S, 64, gScenarioKernelData162_ADD_08U_16S_16S, 4},
{48, 8, 3840, 16, 0, gScenarioBufferData163_ADD_08U_16S_16S, 64, gScenarioKernelData163_ADD_08U_16S_16S, 4},
{2, 10, 200, 16, 0, gScenarioBufferData164_ADD_08U_16S_16S, 64, gScenarioKernelData164_ADD_08U_16S_16S, 4},
{4, 10, 400, 16, 0, gScenarioBufferData165_ADD_08U_16S_16S, 64, gScenarioKernelData165_ADD_08U_16S_16S, 4},
{6, 10, 600, 16, 0, gScenarioBufferData166_ADD_08U_16S_16S, 64, gScenarioKernelData166_ADD_08U_16S_16S, 4},
{8, 10, 800, 16, 0, gScenarioBufferData167_ADD_08U_16S_16S, 64, gScenarioKernelData167_ADD_08U_16S_16S, 4},
{10, 10, 1000, 16, 0, gScenarioBufferData168_ADD_08U_16S_16S, 64, gScenarioKernelData168_ADD_08U_16S_16S, 4},
{12, 10, 1200, 16, 0, gScenarioBufferData169_ADD_08U_16S_16S, 64, gScenarioKernelData169_ADD_08U_16S_16S, 4},
{14, 10, 1400, 16, 0, gScenarioBufferData170_ADD_08U_16S_16S, 64, gScenarioKernelData170_ADD_08U_16S_16S, 4},
{16, 10, 1600, 16, 0, gScenarioBufferData171_ADD_08U_16S_16S, 64, gScenarioKernelData171_ADD_08U_16S_16S, 4},
{18, 10, 1800, 16, 0, gScenarioBufferData172_ADD_08U_16S_16S, 64, gScenarioKernelData172_ADD_08U_16S_16S, 4},
{20, 10, 2000, 16, 0, gScenarioBufferData173_ADD_08U_16S_16S, 64, gScenarioKernelData173_ADD_08U_16S_16S, 4},
{22, 10, 2200, 16, 0, gScenarioBufferData174_ADD_08U_16S_16S, 64, gScenarioKernelData174_ADD_08U_16S_16S, 4},
{24, 10, 2400, 16, 0, gScenarioBufferData175_ADD_08U_16S_16S, 64, gScenarioKernelData175_ADD_08U_16S_16S, 4},
{26, 10, 2600, 16, 0, gScenarioBufferData176_ADD_08U_16S_16S, 64, gScenarioKernelData176_ADD_08U_16S_16S, 4},
{28, 10, 2800, 16, 0, gScenarioBufferData177_ADD_08U_16S_16S, 64, gScenarioKernelData177_ADD_08U_16S_16S, 4},
{30, 10, 3000, 16, 0, gScenarioBufferData178_ADD_08U_16S_16S, 64, gScenarioKernelData178_ADD_08U_16S_16S, 4},
{32, 10, 3200, 16, 0, gScenarioBufferData179_ADD_08U_16S_16S, 64, gScenarioKernelData179_ADD_08U_16S_16S, 4},
{36, 10, 3600, 16, 0, gScenarioBufferData180_ADD_08U_16S_16S, 64, gScenarioKernelData180_ADD_08U_16S_16S, 4},
{2, 12, 240, 16, 0, gScenarioBufferData181_ADD_08U_16S_16S, 64, gScenarioKernelData181_ADD_08U_16S_16S, 4},
{4, 12, 480, 16, 0, gScenarioBufferData182_ADD_08U_16S_16S, 64, gScenarioKernelData182_ADD_08U_16S_16S, 4},
{6, 12, 720, 16, 0, gScenarioBufferData183_ADD_08U_16S_16S, 64, gScenarioKernelData183_ADD_08U_16S_16S, 4},
{8, 12, 960, 16, 0, gScenarioBufferData184_ADD_08U_16S_16S, 64, gScenarioKernelData184_ADD_08U_16S_16S, 4},
{10, 12, 1200, 16, 0, gScenarioBufferData185_ADD_08U_16S_16S, 64, gScenarioKernelData185_ADD_08U_16S_16S, 4},
{12, 12, 1440, 16, 0, gScenarioBufferData186_ADD_08U_16S_16S, 64, gScenarioKernelData186_ADD_08U_16S_16S, 4},
{14, 12, 1680, 16, 0, gScenarioBufferData187_ADD_08U_16S_16S, 64, gScenarioKernelData187_ADD_08U_16S_16S, 4},
{16, 12, 1920, 16, 0, gScenarioBufferData188_ADD_08U_16S_16S, 64, gScenarioKernelData188_ADD_08U_16S_16S, 4},
{18, 12, 2160, 16, 0, gScenarioBufferData189_ADD_08U_16S_16S, 64, gScenarioKernelData189_ADD_08U_16S_16S, 4},
{20, 12, 2400, 16, 0, gScenarioBufferData190_ADD_08U_16S_16S, 64, gScenarioKernelData190_ADD_08U_16S_16S, 4},
{22, 12, 2640, 16, 0, gScenarioBufferData191_ADD_08U_16S_16S, 64, gScenarioKernelData191_ADD_08U_16S_16S, 4},
{24, 12, 2880, 16, 0, gScenarioBufferData192_ADD_08U_16S_16S, 64, gScenarioKernelData192_ADD_08U_16S_16S, 4},
{26, 12, 3120, 16, 0, gScenarioBufferData193_ADD_08U_16S_16S, 64, gScenarioKernelData193_ADD_08U_16S_16S, 4},
{28, 12, 3360, 16, 0, gScenarioBufferData194_ADD_08U_16S_16S, 64, gScenarioKernelData194_ADD_08U_16S_16S, 4},
{30, 12, 3600, 16, 0, gScenarioBufferData195_ADD_08U_16S_16S, 64, gScenarioKernelData195_ADD_08U_16S_16S, 4},
{32, 12, 3840, 16, 0, gScenarioBufferData196_ADD_08U_16S_16S, 64, gScenarioKernelData196_ADD_08U_16S_16S, 4},
{2, 14, 280, 16, 0, gScenarioBufferData197_ADD_08U_16S_16S, 64, gScenarioKernelData197_ADD_08U_16S_16S, 4},
{4, 14, 560, 16, 0, gScenarioBufferData198_ADD_08U_16S_16S, 64, gScenarioKernelData198_ADD_08U_16S_16S, 4},
{6, 14, 840, 16, 0, gScenarioBufferData199_ADD_08U_16S_16S, 64, gScenarioKernelData199_ADD_08U_16S_16S, 4},
{8, 14, 1120, 16, 0, gScenarioBufferData200_ADD_08U_16S_16S, 64, gScenarioKernelData200_ADD_08U_16S_16S, 4},
{10, 14, 1400, 16, 0, gScenarioBufferData201_ADD_08U_16S_16S, 64, gScenarioKernelData201_ADD_08U_16S_16S, 4},
{12, 14, 1680, 16, 0, gScenarioBufferData202_ADD_08U_16S_16S, 64, gScenarioKernelData202_ADD_08U_16S_16S, 4},
{14, 14, 1960, 16, 0, gScenarioBufferData203_ADD_08U_16S_16S, 64, gScenarioKernelData203_ADD_08U_16S_16S, 4},
{16, 14, 2240, 16, 0, gScenarioBufferData204_ADD_08U_16S_16S, 64, gScenarioKernelData204_ADD_08U_16S_16S, 4},
{18, 14, 2520, 16, 0, gScenarioBufferData205_ADD_08U_16S_16S, 64, gScenarioKernelData205_ADD_08U_16S_16S, 4},
{20, 14, 2800, 16, 0, gScenarioBufferData206_ADD_08U_16S_16S, 64, gScenarioKernelData206_ADD_08U_16S_16S, 4},
{22, 14, 3080, 16, 0, gScenarioBufferData207_ADD_08U_16S_16S, 64, gScenarioKernelData207_ADD_08U_16S_16S, 4},
{24, 14, 3360, 16, 0, gScenarioBufferData208_ADD_08U_16S_16S, 64, gScenarioKernelData208_ADD_08U_16S_16S, 4},
{26, 14, 3640, 16, 0, gScenarioBufferData209_ADD_08U_16S_16S, 64, gScenarioKernelData209_ADD_08U_16S_16S, 4},
{2, 16, 320, 16, 0, gScenarioBufferData210_ADD_08U_16S_16S, 64, gScenarioKernelData210_ADD_08U_16S_16S, 4},
{4, 16, 640, 16, 0, gScenarioBufferData211_ADD_08U_16S_16S, 64, gScenarioKernelData211_ADD_08U_16S_16S, 4},
{6, 16, 960, 16, 0, gScenarioBufferData212_ADD_08U_16S_16S, 64, gScenarioKernelData212_ADD_08U_16S_16S, 4},
{8, 16, 1280, 16, 0, gScenarioBufferData213_ADD_08U_16S_16S, 64, gScenarioKernelData213_ADD_08U_16S_16S, 4},
{10, 16, 1600, 16, 0, gScenarioBufferData214_ADD_08U_16S_16S, 64, gScenarioKernelData214_ADD_08U_16S_16S, 4},
{12, 16, 1920, 16, 0, gScenarioBufferData215_ADD_08U_16S_16S, 64, gScenarioKernelData215_ADD_08U_16S_16S, 4},
{14, 16, 2240, 16, 0, gScenarioBufferData216_ADD_08U_16S_16S, 64, gScenarioKernelData216_ADD_08U_16S_16S, 4},
{16, 16, 2560, 16, 0, gScenarioBufferData217_ADD_08U_16S_16S, 64, gScenarioKernelData217_ADD_08U_16S_16S, 4},
{18, 16, 2880, 16, 0, gScenarioBufferData218_ADD_08U_16S_16S, 64, gScenarioKernelData218_ADD_08U_16S_16S, 4},
{20, 16, 3200, 16, 0, gScenarioBufferData219_ADD_08U_16S_16S, 64, gScenarioKernelData219_ADD_08U_16S_16S, 4},
{22, 16, 3520, 16, 0, gScenarioBufferData220_ADD_08U_16S_16S, 64, gScenarioKernelData220_ADD_08U_16S_16S, 4},
{24, 16, 3840, 16, 0, gScenarioBufferData221_ADD_08U_16S_16S, 64, gScenarioKernelData221_ADD_08U_16S_16S, 4},
{2, 18, 360, 16, 0, gScenarioBufferData222_ADD_08U_16S_16S, 64, gScenarioKernelData222_ADD_08U_16S_16S, 4},
{4, 18, 720, 16, 0, gScenarioBufferData223_ADD_08U_16S_16S, 64, gScenarioKernelData223_ADD_08U_16S_16S, 4},
{6, 18, 1080, 16, 0, gScenarioBufferData224_ADD_08U_16S_16S, 64, gScenarioKernelData224_ADD_08U_16S_16S, 4},
{8, 18, 1440, 16, 0, gScenarioBufferData225_ADD_08U_16S_16S, 64, gScenarioKernelData225_ADD_08U_16S_16S, 4},
{10, 18, 1800, 16, 0, gScenarioBufferData226_ADD_08U_16S_16S, 64, gScenarioKernelData226_ADD_08U_16S_16S, 4},
{12, 18, 2160, 16, 0, gScenarioBufferData227_ADD_08U_16S_16S, 64, gScenarioKernelData227_ADD_08U_16S_16S, 4},
{14, 18, 2520, 16, 0, gScenarioBufferData228_ADD_08U_16S_16S, 64, gScenarioKernelData228_ADD_08U_16S_16S, 4},
{16, 18, 2880, 16, 0, gScenarioBufferData229_ADD_08U_16S_16S, 64, gScenarioKernelData229_ADD_08U_16S_16S, 4},
{18, 18, 3240, 16, 0, gScenarioBufferData230_ADD_08U_16S_16S, 64, gScenarioKernelData230_ADD_08U_16S_16S, 4},
{20, 18, 3600, 16, 0, gScenarioBufferData231_ADD_08U_16S_16S, 64, gScenarioKernelData231_ADD_08U_16S_16S, 4},
{2, 20, 400, 16, 0, gScenarioBufferData232_ADD_08U_16S_16S, 64, gScenarioKernelData232_ADD_08U_16S_16S, 4},
{4, 20, 800, 16, 0, gScenarioBufferData233_ADD_08U_16S_16S, 64, gScenarioKernelData233_ADD_08U_16S_16S, 4},
{6, 20, 1200, 16, 0, gScenarioBufferData234_ADD_08U_16S_16S, 64, gScenarioKernelData234_ADD_08U_16S_16S, 4},
{8, 20, 1600, 16, 0, gScenarioBufferData235_ADD_08U_16S_16S, 64, gScenarioKernelData235_ADD_08U_16S_16S, 4},
{10, 20, 2000, 16, 0, gScenarioBufferData236_ADD_08U_16S_16S, 64, gScenarioKernelData236_ADD_08U_16S_16S, 4},
{12, 20, 2400, 16, 0, gScenarioBufferData237_ADD_08U_16S_16S, 64, gScenarioKernelData237_ADD_08U_16S_16S, 4},
{14, 20, 2800, 16, 0, gScenarioBufferData238_ADD_08U_16S_16S, 64, gScenarioKernelData238_ADD_08U_16S_16S, 4},
{16, 20, 3200, 16, 0, gScenarioBufferData239_ADD_08U_16S_16S, 64, gScenarioKernelData239_ADD_08U_16S_16S, 4},
{18, 20, 3600, 16, 0, gScenarioBufferData240_ADD_08U_16S_16S, 64, gScenarioKernelData240_ADD_08U_16S_16S, 4},
{2, 22, 440, 16, 0, gScenarioBufferData241_ADD_08U_16S_16S, 64, gScenarioKernelData241_ADD_08U_16S_16S, 4},
{4, 22, 880, 16, 0, gScenarioBufferData242_ADD_08U_16S_16S, 64, gScenarioKernelData242_ADD_08U_16S_16S, 4},
{6, 22, 1320, 16, 0, gScenarioBufferData243_ADD_08U_16S_16S, 64, gScenarioKernelData243_ADD_08U_16S_16S, 4},
{8, 22, 1760, 16, 0, gScenarioBufferData244_ADD_08U_16S_16S, 64, gScenarioKernelData244_ADD_08U_16S_16S, 4},
{10, 22, 2200, 16, 0, gScenarioBufferData245_ADD_08U_16S_16S, 64, gScenarioKernelData245_ADD_08U_16S_16S, 4},
{12, 22, 2640, 16, 0, gScenarioBufferData246_ADD_08U_16S_16S, 64, gScenarioKernelData246_ADD_08U_16S_16S, 4},
{14, 22, 3080, 16, 0, gScenarioBufferData247_ADD_08U_16S_16S, 64, gScenarioKernelData247_ADD_08U_16S_16S, 4},
{16, 22, 3520, 16, 0, gScenarioBufferData248_ADD_08U_16S_16S, 64, gScenarioKernelData248_ADD_08U_16S_16S, 4},
{2, 24, 480, 16, 0, gScenarioBufferData249_ADD_08U_16S_16S, 64, gScenarioKernelData249_ADD_08U_16S_16S, 4},
{4, 24, 960, 16, 0, gScenarioBufferData250_ADD_08U_16S_16S, 64, gScenarioKernelData250_ADD_08U_16S_16S, 4},
{6, 24, 1440, 16, 0, gScenarioBufferData251_ADD_08U_16S_16S, 64, gScenarioKernelData251_ADD_08U_16S_16S, 4},
{8, 24, 1920, 16, 0, gScenarioBufferData252_ADD_08U_16S_16S, 64, gScenarioKernelData252_ADD_08U_16S_16S, 4},
{10, 24, 2400, 16, 0, gScenarioBufferData253_ADD_08U_16S_16S, 64, gScenarioKernelData253_ADD_08U_16S_16S, 4},
{12, 24, 2880, 16, 0, gScenarioBufferData254_ADD_08U_16S_16S, 64, gScenarioKernelData254_ADD_08U_16S_16S, 4},
{14, 24, 3360, 16, 0, gScenarioBufferData255_ADD_08U_16S_16S, 64, gScenarioKernelData255_ADD_08U_16S_16S, 4},
{16, 24, 3840, 16, 0, gScenarioBufferData256_ADD_08U_16S_16S, 64, gScenarioKernelData256_ADD_08U_16S_16S, 4},
{2, 26, 520, 16, 0, gScenarioBufferData257_ADD_08U_16S_16S, 64, gScenarioKernelData257_ADD_08U_16S_16S, 4},
{4, 26, 1040, 16, 0, gScenarioBufferData258_ADD_08U_16S_16S, 64, gScenarioKernelData258_ADD_08U_16S_16S, 4},
{6, 26, 1560, 16, 0, gScenarioBufferData259_ADD_08U_16S_16S, 64, gScenarioKernelData259_ADD_08U_16S_16S, 4},
{8, 26, 2080, 16, 0, gScenarioBufferData260_ADD_08U_16S_16S, 64, gScenarioKernelData260_ADD_08U_16S_16S, 4},
{10, 26, 2600, 16, 0, gScenarioBufferData261_ADD_08U_16S_16S, 64, gScenarioKernelData261_ADD_08U_16S_16S, 4},
{12, 26, 3120, 16, 0, gScenarioBufferData262_ADD_08U_16S_16S, 64, gScenarioKernelData262_ADD_08U_16S_16S, 4},
{14, 26, 3640, 16, 0, gScenarioBufferData263_ADD_08U_16S_16S, 64, gScenarioKernelData263_ADD_08U_16S_16S, 4},
{2, 28, 560, 16, 0, gScenarioBufferData264_ADD_08U_16S_16S, 64, gScenarioKernelData264_ADD_08U_16S_16S, 4},
{4, 28, 1120, 16, 0, gScenarioBufferData265_ADD_08U_16S_16S, 64, gScenarioKernelData265_ADD_08U_16S_16S, 4},
{6, 28, 1680, 16, 0, gScenarioBufferData266_ADD_08U_16S_16S, 64, gScenarioKernelData266_ADD_08U_16S_16S, 4},
{8, 28, 2240, 16, 0, gScenarioBufferData267_ADD_08U_16S_16S, 64, gScenarioKernelData267_ADD_08U_16S_16S, 4},
{10, 28, 2800, 16, 0, gScenarioBufferData268_ADD_08U_16S_16S, 64, gScenarioKernelData268_ADD_08U_16S_16S, 4},
{12, 28, 3360, 16, 0, gScenarioBufferData269_ADD_08U_16S_16S, 64, gScenarioKernelData269_ADD_08U_16S_16S, 4},
{2, 30, 600, 16, 0, gScenarioBufferData270_ADD_08U_16S_16S, 64, gScenarioKernelData270_ADD_08U_16S_16S, 4},
{4, 30, 1200, 16, 0, gScenarioBufferData271_ADD_08U_16S_16S, 64, gScenarioKernelData271_ADD_08U_16S_16S, 4},
{6, 30, 1800, 16, 0, gScenarioBufferData272_ADD_08U_16S_16S, 64, gScenarioKernelData272_ADD_08U_16S_16S, 4},
{8, 30, 2400, 16, 0, gScenarioBufferData273_ADD_08U_16S_16S, 64, gScenarioKernelData273_ADD_08U_16S_16S, 4},
{10, 30, 3000, 16, 0, gScenarioBufferData274_ADD_08U_16S_16S, 64, gScenarioKernelData274_ADD_08U_16S_16S, 4},
{12, 30, 3600, 16, 0, gScenarioBufferData275_ADD_08U_16S_16S, 64, gScenarioKernelData275_ADD_08U_16S_16S, 4},
{2, 32, 640, 16, 0, gScenarioBufferData276_ADD_08U_16S_16S, 64, gScenarioKernelData276_ADD_08U_16S_16S, 4},
{4, 32, 1280, 16, 0, gScenarioBufferData277_ADD_08U_16S_16S, 64, gScenarioKernelData277_ADD_08U_16S_16S, 4},
{6, 32, 1920, 16, 0, gScenarioBufferData278_ADD_08U_16S_16S, 64, gScenarioKernelData278_ADD_08U_16S_16S, 4},
{8, 32, 2560, 16, 0, gScenarioBufferData279_ADD_08U_16S_16S, 64, gScenarioKernelData279_ADD_08U_16S_16S, 4},
{10, 32, 3200, 16, 0, gScenarioBufferData280_ADD_08U_16S_16S, 64, gScenarioKernelData280_ADD_08U_16S_16S, 4},
{12, 32, 3840, 16, 0, gScenarioBufferData281_ADD_08U_16S_16S, 64, gScenarioKernelData281_ADD_08U_16S_16S, 4}
};

static acf_scenario_list gScenarioList_ADD_08U_16S_16S = {
282, //number of scenarios
gScenarioArray_ADD_08U_16S_16S};
//**************************************************************

class ADD_08U_16S_16S : public ACF_Process_APU
{

public:
   ADD_08U_16S_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("ADD_08U_16S_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("ADD_08U_16S_16S",
                                        ADD_08U_16S_16S_LOAD_SEGMENTS,
                                        ADD_08U_16S_16S_LOAD_PMEM, ADD_08U_16S_16S_LOAD_PMEM_SIZE,
                                        ADD_08U_16S_16S_LOAD_DMEM, ADD_08U_16S_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(ADD_08U_16S_16S_APEX_LOG_BUFFER, ADD_08U_16S_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_16S, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_ADD_08U_16S_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_ADD_08U_16S_16S

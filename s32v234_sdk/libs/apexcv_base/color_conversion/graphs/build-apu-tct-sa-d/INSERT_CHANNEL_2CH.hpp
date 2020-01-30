#ifndef _ACF_PROCESS_APU_INSERT_CHANNEL_2CH
#define _ACF_PROCESS_APU_INSERT_CHANNEL_2CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INSERT_CHANNEL_2CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INSERT_CHANNEL_2CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INSERT_CHANNEL_2CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INSERT_CHANNEL_2CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INSERT_CHANNEL_2CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INSERT_CHANNEL_2CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INSERT_CHANNEL_2CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INSERT_CHANNEL_2CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INSERT_CHANNEL_2CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INSERT_CHANNEL_2CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INSERT_CHANNEL_2CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INSERT_CHANNEL_2CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INSERT_CHANNEL_2CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INSERT_CHANNEL_2CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INSERT_CHANNEL_2CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INSERT_CHANNEL_2CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INSERT_CHANNEL_2CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INSERT_CHANNEL_2CH[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INSERT_CHANNEL_2CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INSERT_CHANNEL_2CH[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INSERT_CHANNEL_2CH[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INSERT_CHANNEL_2CH[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INSERT_CHANNEL_2CH[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INSERT_CHANNEL_2CH[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INSERT_CHANNEL_2CH[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INSERT_CHANNEL_2CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INSERT_CHANNEL_2CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INSERT_CHANNEL_2CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INSERT_CHANNEL_2CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INSERT_CHANNEL_2CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INSERT_CHANNEL_2CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INSERT_CHANNEL_2CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INSERT_CHANNEL_2CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INSERT_CHANNEL_2CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INSERT_CHANNEL_2CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INSERT_CHANNEL_2CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INSERT_CHANNEL_2CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INSERT_CHANNEL_2CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INSERT_CHANNEL_2CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INSERT_CHANNEL_2CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INSERT_CHANNEL_2CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INSERT_CHANNEL_2CH[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INSERT_CHANNEL_2CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INSERT_CHANNEL_2CH[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INSERT_CHANNEL_2CH[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INSERT_CHANNEL_2CH[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_INSERT_CHANNEL_2CH[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_INSERT_CHANNEL_2CH[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_INSERT_CHANNEL_2CH[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_INSERT_CHANNEL_2CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_INSERT_CHANNEL_2CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_INSERT_CHANNEL_2CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_INSERT_CHANNEL_2CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_INSERT_CHANNEL_2CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_INSERT_CHANNEL_2CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_INSERT_CHANNEL_2CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_INSERT_CHANNEL_2CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_INSERT_CHANNEL_2CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_INSERT_CHANNEL_2CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_INSERT_CHANNEL_2CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_INSERT_CHANNEL_2CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_INSERT_CHANNEL_2CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_INSERT_CHANNEL_2CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_INSERT_CHANNEL_2CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_INSERT_CHANNEL_2CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_INSERT_CHANNEL_2CH[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_INSERT_CHANNEL_2CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_INSERT_CHANNEL_2CH[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_INSERT_CHANNEL_2CH[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_INSERT_CHANNEL_2CH[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_INSERT_CHANNEL_2CH[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_INSERT_CHANNEL_2CH[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_INSERT_CHANNEL_2CH[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_INSERT_CHANNEL_2CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_INSERT_CHANNEL_2CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_INSERT_CHANNEL_2CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_INSERT_CHANNEL_2CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_INSERT_CHANNEL_2CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_INSERT_CHANNEL_2CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_INSERT_CHANNEL_2CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_INSERT_CHANNEL_2CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_INSERT_CHANNEL_2CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_INSERT_CHANNEL_2CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_INSERT_CHANNEL_2CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_INSERT_CHANNEL_2CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_INSERT_CHANNEL_2CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_INSERT_CHANNEL_2CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_INSERT_CHANNEL_2CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_INSERT_CHANNEL_2CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_INSERT_CHANNEL_2CH[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_INSERT_CHANNEL_2CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_INSERT_CHANNEL_2CH[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_INSERT_CHANNEL_2CH[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_INSERT_CHANNEL_2CH[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_INSERT_CHANNEL_2CH[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_INSERT_CHANNEL_2CH[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_INSERT_CHANNEL_2CH[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_INSERT_CHANNEL_2CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_INSERT_CHANNEL_2CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_INSERT_CHANNEL_2CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_INSERT_CHANNEL_2CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_INSERT_CHANNEL_2CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_INSERT_CHANNEL_2CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_INSERT_CHANNEL_2CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_INSERT_CHANNEL_2CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_INSERT_CHANNEL_2CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_INSERT_CHANNEL_2CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_INSERT_CHANNEL_2CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_INSERT_CHANNEL_2CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_INSERT_CHANNEL_2CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_INSERT_CHANNEL_2CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_INSERT_CHANNEL_2CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_INSERT_CHANNEL_2CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_INSERT_CHANNEL_2CH[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_INSERT_CHANNEL_2CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_INSERT_CHANNEL_2CH[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_INSERT_CHANNEL_2CH[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_INSERT_CHANNEL_2CH[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_INSERT_CHANNEL_2CH[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_INSERT_CHANNEL_2CH[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_INSERT_CHANNEL_2CH[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_INSERT_CHANNEL_2CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_INSERT_CHANNEL_2CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_INSERT_CHANNEL_2CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_INSERT_CHANNEL_2CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_INSERT_CHANNEL_2CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_INSERT_CHANNEL_2CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_INSERT_CHANNEL_2CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_INSERT_CHANNEL_2CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_INSERT_CHANNEL_2CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_INSERT_CHANNEL_2CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_INSERT_CHANNEL_2CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_INSERT_CHANNEL_2CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_INSERT_CHANNEL_2CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_INSERT_CHANNEL_2CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_INSERT_CHANNEL_2CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_INSERT_CHANNEL_2CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_INSERT_CHANNEL_2CH[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_INSERT_CHANNEL_2CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_INSERT_CHANNEL_2CH[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_INSERT_CHANNEL_2CH[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_INSERT_CHANNEL_2CH[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_INSERT_CHANNEL_2CH[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_INSERT_CHANNEL_2CH[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_INSERT_CHANNEL_2CH[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_INSERT_CHANNEL_2CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_INSERT_CHANNEL_2CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_INSERT_CHANNEL_2CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_INSERT_CHANNEL_2CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_INSERT_CHANNEL_2CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_INSERT_CHANNEL_2CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_INSERT_CHANNEL_2CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_INSERT_CHANNEL_2CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_INSERT_CHANNEL_2CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_INSERT_CHANNEL_2CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_INSERT_CHANNEL_2CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_INSERT_CHANNEL_2CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_INSERT_CHANNEL_2CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_INSERT_CHANNEL_2CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_INSERT_CHANNEL_2CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_INSERT_CHANNEL_2CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_INSERT_CHANNEL_2CH[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_INSERT_CHANNEL_2CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_INSERT_CHANNEL_2CH[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_INSERT_CHANNEL_2CH[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_INSERT_CHANNEL_2CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_INSERT_CHANNEL_2CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_INSERT_CHANNEL_2CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_INSERT_CHANNEL_2CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_INSERT_CHANNEL_2CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_INSERT_CHANNEL_2CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_INSERT_CHANNEL_2CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_INSERT_CHANNEL_2CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_INSERT_CHANNEL_2CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_INSERT_CHANNEL_2CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_INSERT_CHANNEL_2CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_INSERT_CHANNEL_2CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_INSERT_CHANNEL_2CH[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_INSERT_CHANNEL_2CH[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_INSERT_CHANNEL_2CH[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_INSERT_CHANNEL_2CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_INSERT_CHANNEL_2CH[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_INSERT_CHANNEL_2CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_INSERT_CHANNEL_2CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_INSERT_CHANNEL_2CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_INSERT_CHANNEL_2CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_INSERT_CHANNEL_2CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_INSERT_CHANNEL_2CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_INSERT_CHANNEL_2CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_INSERT_CHANNEL_2CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_INSERT_CHANNEL_2CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_INSERT_CHANNEL_2CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_INSERT_CHANNEL_2CH[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_INSERT_CHANNEL_2CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_INSERT_CHANNEL_2CH[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_INSERT_CHANNEL_2CH[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_INSERT_CHANNEL_2CH[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_INSERT_CHANNEL_2CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_INSERT_CHANNEL_2CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_INSERT_CHANNEL_2CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_INSERT_CHANNEL_2CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_INSERT_CHANNEL_2CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_INSERT_CHANNEL_2CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_INSERT_CHANNEL_2CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_INSERT_CHANNEL_2CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_INSERT_CHANNEL_2CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_INSERT_CHANNEL_2CH[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_INSERT_CHANNEL_2CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_INSERT_CHANNEL_2CH[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_INSERT_CHANNEL_2CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_INSERT_CHANNEL_2CH[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_INSERT_CHANNEL_2CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_INSERT_CHANNEL_2CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_INSERT_CHANNEL_2CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_INSERT_CHANNEL_2CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_INSERT_CHANNEL_2CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_INSERT_CHANNEL_2CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_INSERT_CHANNEL_2CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_INSERT_CHANNEL_2CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_INSERT_CHANNEL_2CH[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_INSERT_CHANNEL_2CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_INSERT_CHANNEL_2CH[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_INSERT_CHANNEL_2CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_INSERT_CHANNEL_2CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_INSERT_CHANNEL_2CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_INSERT_CHANNEL_2CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_INSERT_CHANNEL_2CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_INSERT_CHANNEL_2CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_INSERT_CHANNEL_2CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_INSERT_CHANNEL_2CH[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_INSERT_CHANNEL_2CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_INSERT_CHANNEL_2CH[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_INSERT_CHANNEL_2CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_INSERT_CHANNEL_2CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_INSERT_CHANNEL_2CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_INSERT_CHANNEL_2CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_INSERT_CHANNEL_2CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_INSERT_CHANNEL_2CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_INSERT_CHANNEL_2CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_INSERT_CHANNEL_2CH[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_INSERT_CHANNEL_2CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_INSERT_CHANNEL_2CH[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_INSERT_CHANNEL_2CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_INSERT_CHANNEL_2CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_INSERT_CHANNEL_2CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_INSERT_CHANNEL_2CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_INSERT_CHANNEL_2CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_INSERT_CHANNEL_2CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_INSERT_CHANNEL_2CH[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_INSERT_CHANNEL_2CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_INSERT_CHANNEL_2CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_INSERT_CHANNEL_2CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_INSERT_CHANNEL_2CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_INSERT_CHANNEL_2CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_INSERT_CHANNEL_2CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_INSERT_CHANNEL_2CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_INSERT_CHANNEL_2CH[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_INSERT_CHANNEL_2CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_INSERT_CHANNEL_2CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_INSERT_CHANNEL_2CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_INSERT_CHANNEL_2CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_INSERT_CHANNEL_2CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_INSERT_CHANNEL_2CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_INSERT_CHANNEL_2CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_INSERT_CHANNEL_2CH[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_INSERT_CHANNEL_2CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_INSERT_CHANNEL_2CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_INSERT_CHANNEL_2CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_INSERT_CHANNEL_2CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_INSERT_CHANNEL_2CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_INSERT_CHANNEL_2CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_INSERT_CHANNEL_2CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_INSERT_CHANNEL_2CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_INSERT_CHANNEL_2CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_INSERT_CHANNEL_2CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_INSERT_CHANNEL_2CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_INSERT_CHANNEL_2CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_INSERT_CHANNEL_2CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_INSERT_CHANNEL_2CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_INSERT_CHANNEL_2CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_INSERT_CHANNEL_2CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_INSERT_CHANNEL_2CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_INSERT_CHANNEL_2CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_INSERT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_INSERT_CHANNEL_2CH[] = {{0, 0}};

static acf_scenario gScenarioArray_INSERT_CHANNEL_2CH[] = {
{2, 1, 20, 16, 0, gScenarioBufferData0_INSERT_CHANNEL_2CH, 64, gScenarioKernelData0_INSERT_CHANNEL_2CH, 4},
{4, 1, 40, 16, 0, gScenarioBufferData1_INSERT_CHANNEL_2CH, 64, gScenarioKernelData1_INSERT_CHANNEL_2CH, 4},
{6, 1, 60, 16, 0, gScenarioBufferData2_INSERT_CHANNEL_2CH, 64, gScenarioKernelData2_INSERT_CHANNEL_2CH, 4},
{8, 1, 80, 16, 0, gScenarioBufferData3_INSERT_CHANNEL_2CH, 64, gScenarioKernelData3_INSERT_CHANNEL_2CH, 4},
{10, 1, 100, 16, 0, gScenarioBufferData4_INSERT_CHANNEL_2CH, 64, gScenarioKernelData4_INSERT_CHANNEL_2CH, 4},
{12, 1, 120, 16, 0, gScenarioBufferData5_INSERT_CHANNEL_2CH, 64, gScenarioKernelData5_INSERT_CHANNEL_2CH, 4},
{14, 1, 140, 16, 0, gScenarioBufferData6_INSERT_CHANNEL_2CH, 64, gScenarioKernelData6_INSERT_CHANNEL_2CH, 4},
{16, 1, 160, 16, 0, gScenarioBufferData7_INSERT_CHANNEL_2CH, 64, gScenarioKernelData7_INSERT_CHANNEL_2CH, 4},
{18, 1, 180, 16, 0, gScenarioBufferData8_INSERT_CHANNEL_2CH, 64, gScenarioKernelData8_INSERT_CHANNEL_2CH, 4},
{20, 1, 200, 16, 0, gScenarioBufferData9_INSERT_CHANNEL_2CH, 64, gScenarioKernelData9_INSERT_CHANNEL_2CH, 4},
{22, 1, 220, 16, 0, gScenarioBufferData10_INSERT_CHANNEL_2CH, 64, gScenarioKernelData10_INSERT_CHANNEL_2CH, 4},
{24, 1, 240, 16, 0, gScenarioBufferData11_INSERT_CHANNEL_2CH, 64, gScenarioKernelData11_INSERT_CHANNEL_2CH, 4},
{26, 1, 260, 16, 0, gScenarioBufferData12_INSERT_CHANNEL_2CH, 64, gScenarioKernelData12_INSERT_CHANNEL_2CH, 4},
{28, 1, 280, 16, 0, gScenarioBufferData13_INSERT_CHANNEL_2CH, 64, gScenarioKernelData13_INSERT_CHANNEL_2CH, 4},
{30, 1, 300, 16, 0, gScenarioBufferData14_INSERT_CHANNEL_2CH, 64, gScenarioKernelData14_INSERT_CHANNEL_2CH, 4},
{32, 1, 320, 16, 0, gScenarioBufferData15_INSERT_CHANNEL_2CH, 64, gScenarioKernelData15_INSERT_CHANNEL_2CH, 4},
{36, 1, 360, 16, 0, gScenarioBufferData16_INSERT_CHANNEL_2CH, 64, gScenarioKernelData16_INSERT_CHANNEL_2CH, 4},
{40, 1, 400, 16, 0, gScenarioBufferData17_INSERT_CHANNEL_2CH, 64, gScenarioKernelData17_INSERT_CHANNEL_2CH, 4},
{44, 1, 440, 16, 0, gScenarioBufferData18_INSERT_CHANNEL_2CH, 64, gScenarioKernelData18_INSERT_CHANNEL_2CH, 4},
{48, 1, 480, 16, 0, gScenarioBufferData19_INSERT_CHANNEL_2CH, 64, gScenarioKernelData19_INSERT_CHANNEL_2CH, 4},
{52, 1, 520, 16, 0, gScenarioBufferData20_INSERT_CHANNEL_2CH, 64, gScenarioKernelData20_INSERT_CHANNEL_2CH, 4},
{56, 1, 560, 16, 0, gScenarioBufferData21_INSERT_CHANNEL_2CH, 64, gScenarioKernelData21_INSERT_CHANNEL_2CH, 4},
{60, 1, 600, 16, 0, gScenarioBufferData22_INSERT_CHANNEL_2CH, 64, gScenarioKernelData22_INSERT_CHANNEL_2CH, 4},
{64, 1, 640, 16, 0, gScenarioBufferData23_INSERT_CHANNEL_2CH, 64, gScenarioKernelData23_INSERT_CHANNEL_2CH, 4},
{2, 2, 40, 16, 0, gScenarioBufferData24_INSERT_CHANNEL_2CH, 64, gScenarioKernelData24_INSERT_CHANNEL_2CH, 4},
{4, 2, 80, 16, 0, gScenarioBufferData25_INSERT_CHANNEL_2CH, 64, gScenarioKernelData25_INSERT_CHANNEL_2CH, 4},
{6, 2, 120, 16, 0, gScenarioBufferData26_INSERT_CHANNEL_2CH, 64, gScenarioKernelData26_INSERT_CHANNEL_2CH, 4},
{8, 2, 160, 16, 0, gScenarioBufferData27_INSERT_CHANNEL_2CH, 64, gScenarioKernelData27_INSERT_CHANNEL_2CH, 4},
{10, 2, 200, 16, 0, gScenarioBufferData28_INSERT_CHANNEL_2CH, 64, gScenarioKernelData28_INSERT_CHANNEL_2CH, 4},
{12, 2, 240, 16, 0, gScenarioBufferData29_INSERT_CHANNEL_2CH, 64, gScenarioKernelData29_INSERT_CHANNEL_2CH, 4},
{14, 2, 280, 16, 0, gScenarioBufferData30_INSERT_CHANNEL_2CH, 64, gScenarioKernelData30_INSERT_CHANNEL_2CH, 4},
{16, 2, 320, 16, 0, gScenarioBufferData31_INSERT_CHANNEL_2CH, 64, gScenarioKernelData31_INSERT_CHANNEL_2CH, 4},
{18, 2, 360, 16, 0, gScenarioBufferData32_INSERT_CHANNEL_2CH, 64, gScenarioKernelData32_INSERT_CHANNEL_2CH, 4},
{20, 2, 400, 16, 0, gScenarioBufferData33_INSERT_CHANNEL_2CH, 64, gScenarioKernelData33_INSERT_CHANNEL_2CH, 4},
{22, 2, 440, 16, 0, gScenarioBufferData34_INSERT_CHANNEL_2CH, 64, gScenarioKernelData34_INSERT_CHANNEL_2CH, 4},
{24, 2, 480, 16, 0, gScenarioBufferData35_INSERT_CHANNEL_2CH, 64, gScenarioKernelData35_INSERT_CHANNEL_2CH, 4},
{26, 2, 520, 16, 0, gScenarioBufferData36_INSERT_CHANNEL_2CH, 64, gScenarioKernelData36_INSERT_CHANNEL_2CH, 4},
{28, 2, 560, 16, 0, gScenarioBufferData37_INSERT_CHANNEL_2CH, 64, gScenarioKernelData37_INSERT_CHANNEL_2CH, 4},
{30, 2, 600, 16, 0, gScenarioBufferData38_INSERT_CHANNEL_2CH, 64, gScenarioKernelData38_INSERT_CHANNEL_2CH, 4},
{32, 2, 640, 16, 0, gScenarioBufferData39_INSERT_CHANNEL_2CH, 64, gScenarioKernelData39_INSERT_CHANNEL_2CH, 4},
{36, 2, 720, 16, 0, gScenarioBufferData40_INSERT_CHANNEL_2CH, 64, gScenarioKernelData40_INSERT_CHANNEL_2CH, 4},
{40, 2, 800, 16, 0, gScenarioBufferData41_INSERT_CHANNEL_2CH, 64, gScenarioKernelData41_INSERT_CHANNEL_2CH, 4},
{44, 2, 880, 16, 0, gScenarioBufferData42_INSERT_CHANNEL_2CH, 64, gScenarioKernelData42_INSERT_CHANNEL_2CH, 4},
{48, 2, 960, 16, 0, gScenarioBufferData43_INSERT_CHANNEL_2CH, 64, gScenarioKernelData43_INSERT_CHANNEL_2CH, 4},
{52, 2, 1040, 16, 0, gScenarioBufferData44_INSERT_CHANNEL_2CH, 64, gScenarioKernelData44_INSERT_CHANNEL_2CH, 4},
{56, 2, 1120, 16, 0, gScenarioBufferData45_INSERT_CHANNEL_2CH, 64, gScenarioKernelData45_INSERT_CHANNEL_2CH, 4},
{60, 2, 1200, 16, 0, gScenarioBufferData46_INSERT_CHANNEL_2CH, 64, gScenarioKernelData46_INSERT_CHANNEL_2CH, 4},
{64, 2, 1280, 16, 0, gScenarioBufferData47_INSERT_CHANNEL_2CH, 64, gScenarioKernelData47_INSERT_CHANNEL_2CH, 4},
{2, 3, 60, 16, 0, gScenarioBufferData48_INSERT_CHANNEL_2CH, 64, gScenarioKernelData48_INSERT_CHANNEL_2CH, 4},
{4, 3, 120, 16, 0, gScenarioBufferData49_INSERT_CHANNEL_2CH, 64, gScenarioKernelData49_INSERT_CHANNEL_2CH, 4},
{6, 3, 180, 16, 0, gScenarioBufferData50_INSERT_CHANNEL_2CH, 64, gScenarioKernelData50_INSERT_CHANNEL_2CH, 4},
{8, 3, 240, 16, 0, gScenarioBufferData51_INSERT_CHANNEL_2CH, 64, gScenarioKernelData51_INSERT_CHANNEL_2CH, 4},
{10, 3, 300, 16, 0, gScenarioBufferData52_INSERT_CHANNEL_2CH, 64, gScenarioKernelData52_INSERT_CHANNEL_2CH, 4},
{12, 3, 360, 16, 0, gScenarioBufferData53_INSERT_CHANNEL_2CH, 64, gScenarioKernelData53_INSERT_CHANNEL_2CH, 4},
{14, 3, 420, 16, 0, gScenarioBufferData54_INSERT_CHANNEL_2CH, 64, gScenarioKernelData54_INSERT_CHANNEL_2CH, 4},
{16, 3, 480, 16, 0, gScenarioBufferData55_INSERT_CHANNEL_2CH, 64, gScenarioKernelData55_INSERT_CHANNEL_2CH, 4},
{18, 3, 540, 16, 0, gScenarioBufferData56_INSERT_CHANNEL_2CH, 64, gScenarioKernelData56_INSERT_CHANNEL_2CH, 4},
{20, 3, 600, 16, 0, gScenarioBufferData57_INSERT_CHANNEL_2CH, 64, gScenarioKernelData57_INSERT_CHANNEL_2CH, 4},
{22, 3, 660, 16, 0, gScenarioBufferData58_INSERT_CHANNEL_2CH, 64, gScenarioKernelData58_INSERT_CHANNEL_2CH, 4},
{24, 3, 720, 16, 0, gScenarioBufferData59_INSERT_CHANNEL_2CH, 64, gScenarioKernelData59_INSERT_CHANNEL_2CH, 4},
{26, 3, 780, 16, 0, gScenarioBufferData60_INSERT_CHANNEL_2CH, 64, gScenarioKernelData60_INSERT_CHANNEL_2CH, 4},
{28, 3, 840, 16, 0, gScenarioBufferData61_INSERT_CHANNEL_2CH, 64, gScenarioKernelData61_INSERT_CHANNEL_2CH, 4},
{30, 3, 900, 16, 0, gScenarioBufferData62_INSERT_CHANNEL_2CH, 64, gScenarioKernelData62_INSERT_CHANNEL_2CH, 4},
{32, 3, 960, 16, 0, gScenarioBufferData63_INSERT_CHANNEL_2CH, 64, gScenarioKernelData63_INSERT_CHANNEL_2CH, 4},
{36, 3, 1080, 16, 0, gScenarioBufferData64_INSERT_CHANNEL_2CH, 64, gScenarioKernelData64_INSERT_CHANNEL_2CH, 4},
{40, 3, 1200, 16, 0, gScenarioBufferData65_INSERT_CHANNEL_2CH, 64, gScenarioKernelData65_INSERT_CHANNEL_2CH, 4},
{44, 3, 1320, 16, 0, gScenarioBufferData66_INSERT_CHANNEL_2CH, 64, gScenarioKernelData66_INSERT_CHANNEL_2CH, 4},
{48, 3, 1440, 16, 0, gScenarioBufferData67_INSERT_CHANNEL_2CH, 64, gScenarioKernelData67_INSERT_CHANNEL_2CH, 4},
{52, 3, 1560, 16, 0, gScenarioBufferData68_INSERT_CHANNEL_2CH, 64, gScenarioKernelData68_INSERT_CHANNEL_2CH, 4},
{56, 3, 1680, 16, 0, gScenarioBufferData69_INSERT_CHANNEL_2CH, 64, gScenarioKernelData69_INSERT_CHANNEL_2CH, 4},
{60, 3, 1800, 16, 0, gScenarioBufferData70_INSERT_CHANNEL_2CH, 64, gScenarioKernelData70_INSERT_CHANNEL_2CH, 4},
{64, 3, 1920, 16, 0, gScenarioBufferData71_INSERT_CHANNEL_2CH, 64, gScenarioKernelData71_INSERT_CHANNEL_2CH, 4},
{2, 4, 80, 16, 0, gScenarioBufferData72_INSERT_CHANNEL_2CH, 64, gScenarioKernelData72_INSERT_CHANNEL_2CH, 4},
{4, 4, 160, 16, 0, gScenarioBufferData73_INSERT_CHANNEL_2CH, 64, gScenarioKernelData73_INSERT_CHANNEL_2CH, 4},
{6, 4, 240, 16, 0, gScenarioBufferData74_INSERT_CHANNEL_2CH, 64, gScenarioKernelData74_INSERT_CHANNEL_2CH, 4},
{8, 4, 320, 16, 0, gScenarioBufferData75_INSERT_CHANNEL_2CH, 64, gScenarioKernelData75_INSERT_CHANNEL_2CH, 4},
{10, 4, 400, 16, 0, gScenarioBufferData76_INSERT_CHANNEL_2CH, 64, gScenarioKernelData76_INSERT_CHANNEL_2CH, 4},
{12, 4, 480, 16, 0, gScenarioBufferData77_INSERT_CHANNEL_2CH, 64, gScenarioKernelData77_INSERT_CHANNEL_2CH, 4},
{14, 4, 560, 16, 0, gScenarioBufferData78_INSERT_CHANNEL_2CH, 64, gScenarioKernelData78_INSERT_CHANNEL_2CH, 4},
{16, 4, 640, 16, 0, gScenarioBufferData79_INSERT_CHANNEL_2CH, 64, gScenarioKernelData79_INSERT_CHANNEL_2CH, 4},
{18, 4, 720, 16, 0, gScenarioBufferData80_INSERT_CHANNEL_2CH, 64, gScenarioKernelData80_INSERT_CHANNEL_2CH, 4},
{20, 4, 800, 16, 0, gScenarioBufferData81_INSERT_CHANNEL_2CH, 64, gScenarioKernelData81_INSERT_CHANNEL_2CH, 4},
{22, 4, 880, 16, 0, gScenarioBufferData82_INSERT_CHANNEL_2CH, 64, gScenarioKernelData82_INSERT_CHANNEL_2CH, 4},
{24, 4, 960, 16, 0, gScenarioBufferData83_INSERT_CHANNEL_2CH, 64, gScenarioKernelData83_INSERT_CHANNEL_2CH, 4},
{26, 4, 1040, 16, 0, gScenarioBufferData84_INSERT_CHANNEL_2CH, 64, gScenarioKernelData84_INSERT_CHANNEL_2CH, 4},
{28, 4, 1120, 16, 0, gScenarioBufferData85_INSERT_CHANNEL_2CH, 64, gScenarioKernelData85_INSERT_CHANNEL_2CH, 4},
{30, 4, 1200, 16, 0, gScenarioBufferData86_INSERT_CHANNEL_2CH, 64, gScenarioKernelData86_INSERT_CHANNEL_2CH, 4},
{32, 4, 1280, 16, 0, gScenarioBufferData87_INSERT_CHANNEL_2CH, 64, gScenarioKernelData87_INSERT_CHANNEL_2CH, 4},
{36, 4, 1440, 16, 0, gScenarioBufferData88_INSERT_CHANNEL_2CH, 64, gScenarioKernelData88_INSERT_CHANNEL_2CH, 4},
{40, 4, 1600, 16, 0, gScenarioBufferData89_INSERT_CHANNEL_2CH, 64, gScenarioKernelData89_INSERT_CHANNEL_2CH, 4},
{44, 4, 1760, 16, 0, gScenarioBufferData90_INSERT_CHANNEL_2CH, 64, gScenarioKernelData90_INSERT_CHANNEL_2CH, 4},
{48, 4, 1920, 16, 0, gScenarioBufferData91_INSERT_CHANNEL_2CH, 64, gScenarioKernelData91_INSERT_CHANNEL_2CH, 4},
{52, 4, 2080, 16, 0, gScenarioBufferData92_INSERT_CHANNEL_2CH, 64, gScenarioKernelData92_INSERT_CHANNEL_2CH, 4},
{56, 4, 2240, 16, 0, gScenarioBufferData93_INSERT_CHANNEL_2CH, 64, gScenarioKernelData93_INSERT_CHANNEL_2CH, 4},
{60, 4, 2400, 16, 0, gScenarioBufferData94_INSERT_CHANNEL_2CH, 64, gScenarioKernelData94_INSERT_CHANNEL_2CH, 4},
{64, 4, 2560, 16, 0, gScenarioBufferData95_INSERT_CHANNEL_2CH, 64, gScenarioKernelData95_INSERT_CHANNEL_2CH, 4},
{2, 5, 100, 16, 0, gScenarioBufferData96_INSERT_CHANNEL_2CH, 64, gScenarioKernelData96_INSERT_CHANNEL_2CH, 4},
{4, 5, 200, 16, 0, gScenarioBufferData97_INSERT_CHANNEL_2CH, 64, gScenarioKernelData97_INSERT_CHANNEL_2CH, 4},
{6, 5, 300, 16, 0, gScenarioBufferData98_INSERT_CHANNEL_2CH, 64, gScenarioKernelData98_INSERT_CHANNEL_2CH, 4},
{8, 5, 400, 16, 0, gScenarioBufferData99_INSERT_CHANNEL_2CH, 64, gScenarioKernelData99_INSERT_CHANNEL_2CH, 4},
{10, 5, 500, 16, 0, gScenarioBufferData100_INSERT_CHANNEL_2CH, 64, gScenarioKernelData100_INSERT_CHANNEL_2CH, 4},
{12, 5, 600, 16, 0, gScenarioBufferData101_INSERT_CHANNEL_2CH, 64, gScenarioKernelData101_INSERT_CHANNEL_2CH, 4},
{14, 5, 700, 16, 0, gScenarioBufferData102_INSERT_CHANNEL_2CH, 64, gScenarioKernelData102_INSERT_CHANNEL_2CH, 4},
{16, 5, 800, 16, 0, gScenarioBufferData103_INSERT_CHANNEL_2CH, 64, gScenarioKernelData103_INSERT_CHANNEL_2CH, 4},
{18, 5, 900, 16, 0, gScenarioBufferData104_INSERT_CHANNEL_2CH, 64, gScenarioKernelData104_INSERT_CHANNEL_2CH, 4},
{20, 5, 1000, 16, 0, gScenarioBufferData105_INSERT_CHANNEL_2CH, 64, gScenarioKernelData105_INSERT_CHANNEL_2CH, 4},
{22, 5, 1100, 16, 0, gScenarioBufferData106_INSERT_CHANNEL_2CH, 64, gScenarioKernelData106_INSERT_CHANNEL_2CH, 4},
{24, 5, 1200, 16, 0, gScenarioBufferData107_INSERT_CHANNEL_2CH, 64, gScenarioKernelData107_INSERT_CHANNEL_2CH, 4},
{26, 5, 1300, 16, 0, gScenarioBufferData108_INSERT_CHANNEL_2CH, 64, gScenarioKernelData108_INSERT_CHANNEL_2CH, 4},
{28, 5, 1400, 16, 0, gScenarioBufferData109_INSERT_CHANNEL_2CH, 64, gScenarioKernelData109_INSERT_CHANNEL_2CH, 4},
{30, 5, 1500, 16, 0, gScenarioBufferData110_INSERT_CHANNEL_2CH, 64, gScenarioKernelData110_INSERT_CHANNEL_2CH, 4},
{32, 5, 1600, 16, 0, gScenarioBufferData111_INSERT_CHANNEL_2CH, 64, gScenarioKernelData111_INSERT_CHANNEL_2CH, 4},
{36, 5, 1800, 16, 0, gScenarioBufferData112_INSERT_CHANNEL_2CH, 64, gScenarioKernelData112_INSERT_CHANNEL_2CH, 4},
{40, 5, 2000, 16, 0, gScenarioBufferData113_INSERT_CHANNEL_2CH, 64, gScenarioKernelData113_INSERT_CHANNEL_2CH, 4},
{44, 5, 2200, 16, 0, gScenarioBufferData114_INSERT_CHANNEL_2CH, 64, gScenarioKernelData114_INSERT_CHANNEL_2CH, 4},
{48, 5, 2400, 16, 0, gScenarioBufferData115_INSERT_CHANNEL_2CH, 64, gScenarioKernelData115_INSERT_CHANNEL_2CH, 4},
{52, 5, 2600, 16, 0, gScenarioBufferData116_INSERT_CHANNEL_2CH, 64, gScenarioKernelData116_INSERT_CHANNEL_2CH, 4},
{56, 5, 2800, 16, 0, gScenarioBufferData117_INSERT_CHANNEL_2CH, 64, gScenarioKernelData117_INSERT_CHANNEL_2CH, 4},
{60, 5, 3000, 16, 0, gScenarioBufferData118_INSERT_CHANNEL_2CH, 64, gScenarioKernelData118_INSERT_CHANNEL_2CH, 4},
{64, 5, 3200, 16, 0, gScenarioBufferData119_INSERT_CHANNEL_2CH, 64, gScenarioKernelData119_INSERT_CHANNEL_2CH, 4},
{2, 6, 120, 16, 0, gScenarioBufferData120_INSERT_CHANNEL_2CH, 64, gScenarioKernelData120_INSERT_CHANNEL_2CH, 4},
{4, 6, 240, 16, 0, gScenarioBufferData121_INSERT_CHANNEL_2CH, 64, gScenarioKernelData121_INSERT_CHANNEL_2CH, 4},
{6, 6, 360, 16, 0, gScenarioBufferData122_INSERT_CHANNEL_2CH, 64, gScenarioKernelData122_INSERT_CHANNEL_2CH, 4},
{8, 6, 480, 16, 0, gScenarioBufferData123_INSERT_CHANNEL_2CH, 64, gScenarioKernelData123_INSERT_CHANNEL_2CH, 4},
{10, 6, 600, 16, 0, gScenarioBufferData124_INSERT_CHANNEL_2CH, 64, gScenarioKernelData124_INSERT_CHANNEL_2CH, 4},
{12, 6, 720, 16, 0, gScenarioBufferData125_INSERT_CHANNEL_2CH, 64, gScenarioKernelData125_INSERT_CHANNEL_2CH, 4},
{14, 6, 840, 16, 0, gScenarioBufferData126_INSERT_CHANNEL_2CH, 64, gScenarioKernelData126_INSERT_CHANNEL_2CH, 4},
{16, 6, 960, 16, 0, gScenarioBufferData127_INSERT_CHANNEL_2CH, 64, gScenarioKernelData127_INSERT_CHANNEL_2CH, 4},
{18, 6, 1080, 16, 0, gScenarioBufferData128_INSERT_CHANNEL_2CH, 64, gScenarioKernelData128_INSERT_CHANNEL_2CH, 4},
{20, 6, 1200, 16, 0, gScenarioBufferData129_INSERT_CHANNEL_2CH, 64, gScenarioKernelData129_INSERT_CHANNEL_2CH, 4},
{22, 6, 1320, 16, 0, gScenarioBufferData130_INSERT_CHANNEL_2CH, 64, gScenarioKernelData130_INSERT_CHANNEL_2CH, 4},
{24, 6, 1440, 16, 0, gScenarioBufferData131_INSERT_CHANNEL_2CH, 64, gScenarioKernelData131_INSERT_CHANNEL_2CH, 4},
{26, 6, 1560, 16, 0, gScenarioBufferData132_INSERT_CHANNEL_2CH, 64, gScenarioKernelData132_INSERT_CHANNEL_2CH, 4},
{28, 6, 1680, 16, 0, gScenarioBufferData133_INSERT_CHANNEL_2CH, 64, gScenarioKernelData133_INSERT_CHANNEL_2CH, 4},
{30, 6, 1800, 16, 0, gScenarioBufferData134_INSERT_CHANNEL_2CH, 64, gScenarioKernelData134_INSERT_CHANNEL_2CH, 4},
{32, 6, 1920, 16, 0, gScenarioBufferData135_INSERT_CHANNEL_2CH, 64, gScenarioKernelData135_INSERT_CHANNEL_2CH, 4},
{36, 6, 2160, 16, 0, gScenarioBufferData136_INSERT_CHANNEL_2CH, 64, gScenarioKernelData136_INSERT_CHANNEL_2CH, 4},
{40, 6, 2400, 16, 0, gScenarioBufferData137_INSERT_CHANNEL_2CH, 64, gScenarioKernelData137_INSERT_CHANNEL_2CH, 4},
{44, 6, 2640, 16, 0, gScenarioBufferData138_INSERT_CHANNEL_2CH, 64, gScenarioKernelData138_INSERT_CHANNEL_2CH, 4},
{48, 6, 2880, 16, 0, gScenarioBufferData139_INSERT_CHANNEL_2CH, 64, gScenarioKernelData139_INSERT_CHANNEL_2CH, 4},
{52, 6, 3120, 16, 0, gScenarioBufferData140_INSERT_CHANNEL_2CH, 64, gScenarioKernelData140_INSERT_CHANNEL_2CH, 4},
{56, 6, 3360, 16, 0, gScenarioBufferData141_INSERT_CHANNEL_2CH, 64, gScenarioKernelData141_INSERT_CHANNEL_2CH, 4},
{60, 6, 3600, 16, 0, gScenarioBufferData142_INSERT_CHANNEL_2CH, 64, gScenarioKernelData142_INSERT_CHANNEL_2CH, 4},
{64, 6, 3840, 16, 0, gScenarioBufferData143_INSERT_CHANNEL_2CH, 64, gScenarioKernelData143_INSERT_CHANNEL_2CH, 4},
{2, 8, 160, 16, 0, gScenarioBufferData144_INSERT_CHANNEL_2CH, 64, gScenarioKernelData144_INSERT_CHANNEL_2CH, 4},
{4, 8, 320, 16, 0, gScenarioBufferData145_INSERT_CHANNEL_2CH, 64, gScenarioKernelData145_INSERT_CHANNEL_2CH, 4},
{6, 8, 480, 16, 0, gScenarioBufferData146_INSERT_CHANNEL_2CH, 64, gScenarioKernelData146_INSERT_CHANNEL_2CH, 4},
{8, 8, 640, 16, 0, gScenarioBufferData147_INSERT_CHANNEL_2CH, 64, gScenarioKernelData147_INSERT_CHANNEL_2CH, 4},
{10, 8, 800, 16, 0, gScenarioBufferData148_INSERT_CHANNEL_2CH, 64, gScenarioKernelData148_INSERT_CHANNEL_2CH, 4},
{12, 8, 960, 16, 0, gScenarioBufferData149_INSERT_CHANNEL_2CH, 64, gScenarioKernelData149_INSERT_CHANNEL_2CH, 4},
{14, 8, 1120, 16, 0, gScenarioBufferData150_INSERT_CHANNEL_2CH, 64, gScenarioKernelData150_INSERT_CHANNEL_2CH, 4},
{16, 8, 1280, 16, 0, gScenarioBufferData151_INSERT_CHANNEL_2CH, 64, gScenarioKernelData151_INSERT_CHANNEL_2CH, 4},
{18, 8, 1440, 16, 0, gScenarioBufferData152_INSERT_CHANNEL_2CH, 64, gScenarioKernelData152_INSERT_CHANNEL_2CH, 4},
{20, 8, 1600, 16, 0, gScenarioBufferData153_INSERT_CHANNEL_2CH, 64, gScenarioKernelData153_INSERT_CHANNEL_2CH, 4},
{22, 8, 1760, 16, 0, gScenarioBufferData154_INSERT_CHANNEL_2CH, 64, gScenarioKernelData154_INSERT_CHANNEL_2CH, 4},
{24, 8, 1920, 16, 0, gScenarioBufferData155_INSERT_CHANNEL_2CH, 64, gScenarioKernelData155_INSERT_CHANNEL_2CH, 4},
{26, 8, 2080, 16, 0, gScenarioBufferData156_INSERT_CHANNEL_2CH, 64, gScenarioKernelData156_INSERT_CHANNEL_2CH, 4},
{28, 8, 2240, 16, 0, gScenarioBufferData157_INSERT_CHANNEL_2CH, 64, gScenarioKernelData157_INSERT_CHANNEL_2CH, 4},
{30, 8, 2400, 16, 0, gScenarioBufferData158_INSERT_CHANNEL_2CH, 64, gScenarioKernelData158_INSERT_CHANNEL_2CH, 4},
{32, 8, 2560, 16, 0, gScenarioBufferData159_INSERT_CHANNEL_2CH, 64, gScenarioKernelData159_INSERT_CHANNEL_2CH, 4},
{36, 8, 2880, 16, 0, gScenarioBufferData160_INSERT_CHANNEL_2CH, 64, gScenarioKernelData160_INSERT_CHANNEL_2CH, 4},
{40, 8, 3200, 16, 0, gScenarioBufferData161_INSERT_CHANNEL_2CH, 64, gScenarioKernelData161_INSERT_CHANNEL_2CH, 4},
{44, 8, 3520, 16, 0, gScenarioBufferData162_INSERT_CHANNEL_2CH, 64, gScenarioKernelData162_INSERT_CHANNEL_2CH, 4},
{48, 8, 3840, 16, 0, gScenarioBufferData163_INSERT_CHANNEL_2CH, 64, gScenarioKernelData163_INSERT_CHANNEL_2CH, 4},
{2, 10, 200, 16, 0, gScenarioBufferData164_INSERT_CHANNEL_2CH, 64, gScenarioKernelData164_INSERT_CHANNEL_2CH, 4},
{4, 10, 400, 16, 0, gScenarioBufferData165_INSERT_CHANNEL_2CH, 64, gScenarioKernelData165_INSERT_CHANNEL_2CH, 4},
{6, 10, 600, 16, 0, gScenarioBufferData166_INSERT_CHANNEL_2CH, 64, gScenarioKernelData166_INSERT_CHANNEL_2CH, 4},
{8, 10, 800, 16, 0, gScenarioBufferData167_INSERT_CHANNEL_2CH, 64, gScenarioKernelData167_INSERT_CHANNEL_2CH, 4},
{10, 10, 1000, 16, 0, gScenarioBufferData168_INSERT_CHANNEL_2CH, 64, gScenarioKernelData168_INSERT_CHANNEL_2CH, 4},
{12, 10, 1200, 16, 0, gScenarioBufferData169_INSERT_CHANNEL_2CH, 64, gScenarioKernelData169_INSERT_CHANNEL_2CH, 4},
{14, 10, 1400, 16, 0, gScenarioBufferData170_INSERT_CHANNEL_2CH, 64, gScenarioKernelData170_INSERT_CHANNEL_2CH, 4},
{16, 10, 1600, 16, 0, gScenarioBufferData171_INSERT_CHANNEL_2CH, 64, gScenarioKernelData171_INSERT_CHANNEL_2CH, 4},
{18, 10, 1800, 16, 0, gScenarioBufferData172_INSERT_CHANNEL_2CH, 64, gScenarioKernelData172_INSERT_CHANNEL_2CH, 4},
{20, 10, 2000, 16, 0, gScenarioBufferData173_INSERT_CHANNEL_2CH, 64, gScenarioKernelData173_INSERT_CHANNEL_2CH, 4},
{22, 10, 2200, 16, 0, gScenarioBufferData174_INSERT_CHANNEL_2CH, 64, gScenarioKernelData174_INSERT_CHANNEL_2CH, 4},
{24, 10, 2400, 16, 0, gScenarioBufferData175_INSERT_CHANNEL_2CH, 64, gScenarioKernelData175_INSERT_CHANNEL_2CH, 4},
{26, 10, 2600, 16, 0, gScenarioBufferData176_INSERT_CHANNEL_2CH, 64, gScenarioKernelData176_INSERT_CHANNEL_2CH, 4},
{28, 10, 2800, 16, 0, gScenarioBufferData177_INSERT_CHANNEL_2CH, 64, gScenarioKernelData177_INSERT_CHANNEL_2CH, 4},
{30, 10, 3000, 16, 0, gScenarioBufferData178_INSERT_CHANNEL_2CH, 64, gScenarioKernelData178_INSERT_CHANNEL_2CH, 4},
{32, 10, 3200, 16, 0, gScenarioBufferData179_INSERT_CHANNEL_2CH, 64, gScenarioKernelData179_INSERT_CHANNEL_2CH, 4},
{36, 10, 3600, 16, 0, gScenarioBufferData180_INSERT_CHANNEL_2CH, 64, gScenarioKernelData180_INSERT_CHANNEL_2CH, 4},
{2, 12, 240, 16, 0, gScenarioBufferData181_INSERT_CHANNEL_2CH, 64, gScenarioKernelData181_INSERT_CHANNEL_2CH, 4},
{4, 12, 480, 16, 0, gScenarioBufferData182_INSERT_CHANNEL_2CH, 64, gScenarioKernelData182_INSERT_CHANNEL_2CH, 4},
{6, 12, 720, 16, 0, gScenarioBufferData183_INSERT_CHANNEL_2CH, 64, gScenarioKernelData183_INSERT_CHANNEL_2CH, 4},
{8, 12, 960, 16, 0, gScenarioBufferData184_INSERT_CHANNEL_2CH, 64, gScenarioKernelData184_INSERT_CHANNEL_2CH, 4},
{10, 12, 1200, 16, 0, gScenarioBufferData185_INSERT_CHANNEL_2CH, 64, gScenarioKernelData185_INSERT_CHANNEL_2CH, 4},
{12, 12, 1440, 16, 0, gScenarioBufferData186_INSERT_CHANNEL_2CH, 64, gScenarioKernelData186_INSERT_CHANNEL_2CH, 4},
{14, 12, 1680, 16, 0, gScenarioBufferData187_INSERT_CHANNEL_2CH, 64, gScenarioKernelData187_INSERT_CHANNEL_2CH, 4},
{16, 12, 1920, 16, 0, gScenarioBufferData188_INSERT_CHANNEL_2CH, 64, gScenarioKernelData188_INSERT_CHANNEL_2CH, 4},
{18, 12, 2160, 16, 0, gScenarioBufferData189_INSERT_CHANNEL_2CH, 64, gScenarioKernelData189_INSERT_CHANNEL_2CH, 4},
{20, 12, 2400, 16, 0, gScenarioBufferData190_INSERT_CHANNEL_2CH, 64, gScenarioKernelData190_INSERT_CHANNEL_2CH, 4},
{22, 12, 2640, 16, 0, gScenarioBufferData191_INSERT_CHANNEL_2CH, 64, gScenarioKernelData191_INSERT_CHANNEL_2CH, 4},
{24, 12, 2880, 16, 0, gScenarioBufferData192_INSERT_CHANNEL_2CH, 64, gScenarioKernelData192_INSERT_CHANNEL_2CH, 4},
{26, 12, 3120, 16, 0, gScenarioBufferData193_INSERT_CHANNEL_2CH, 64, gScenarioKernelData193_INSERT_CHANNEL_2CH, 4},
{28, 12, 3360, 16, 0, gScenarioBufferData194_INSERT_CHANNEL_2CH, 64, gScenarioKernelData194_INSERT_CHANNEL_2CH, 4},
{30, 12, 3600, 16, 0, gScenarioBufferData195_INSERT_CHANNEL_2CH, 64, gScenarioKernelData195_INSERT_CHANNEL_2CH, 4},
{32, 12, 3840, 16, 0, gScenarioBufferData196_INSERT_CHANNEL_2CH, 64, gScenarioKernelData196_INSERT_CHANNEL_2CH, 4},
{2, 14, 280, 16, 0, gScenarioBufferData197_INSERT_CHANNEL_2CH, 64, gScenarioKernelData197_INSERT_CHANNEL_2CH, 4},
{4, 14, 560, 16, 0, gScenarioBufferData198_INSERT_CHANNEL_2CH, 64, gScenarioKernelData198_INSERT_CHANNEL_2CH, 4},
{6, 14, 840, 16, 0, gScenarioBufferData199_INSERT_CHANNEL_2CH, 64, gScenarioKernelData199_INSERT_CHANNEL_2CH, 4},
{8, 14, 1120, 16, 0, gScenarioBufferData200_INSERT_CHANNEL_2CH, 64, gScenarioKernelData200_INSERT_CHANNEL_2CH, 4},
{10, 14, 1400, 16, 0, gScenarioBufferData201_INSERT_CHANNEL_2CH, 64, gScenarioKernelData201_INSERT_CHANNEL_2CH, 4},
{12, 14, 1680, 16, 0, gScenarioBufferData202_INSERT_CHANNEL_2CH, 64, gScenarioKernelData202_INSERT_CHANNEL_2CH, 4},
{14, 14, 1960, 16, 0, gScenarioBufferData203_INSERT_CHANNEL_2CH, 64, gScenarioKernelData203_INSERT_CHANNEL_2CH, 4},
{16, 14, 2240, 16, 0, gScenarioBufferData204_INSERT_CHANNEL_2CH, 64, gScenarioKernelData204_INSERT_CHANNEL_2CH, 4},
{18, 14, 2520, 16, 0, gScenarioBufferData205_INSERT_CHANNEL_2CH, 64, gScenarioKernelData205_INSERT_CHANNEL_2CH, 4},
{20, 14, 2800, 16, 0, gScenarioBufferData206_INSERT_CHANNEL_2CH, 64, gScenarioKernelData206_INSERT_CHANNEL_2CH, 4},
{22, 14, 3080, 16, 0, gScenarioBufferData207_INSERT_CHANNEL_2CH, 64, gScenarioKernelData207_INSERT_CHANNEL_2CH, 4},
{24, 14, 3360, 16, 0, gScenarioBufferData208_INSERT_CHANNEL_2CH, 64, gScenarioKernelData208_INSERT_CHANNEL_2CH, 4},
{26, 14, 3640, 16, 0, gScenarioBufferData209_INSERT_CHANNEL_2CH, 64, gScenarioKernelData209_INSERT_CHANNEL_2CH, 4},
{2, 16, 320, 16, 0, gScenarioBufferData210_INSERT_CHANNEL_2CH, 64, gScenarioKernelData210_INSERT_CHANNEL_2CH, 4},
{4, 16, 640, 16, 0, gScenarioBufferData211_INSERT_CHANNEL_2CH, 64, gScenarioKernelData211_INSERT_CHANNEL_2CH, 4},
{6, 16, 960, 16, 0, gScenarioBufferData212_INSERT_CHANNEL_2CH, 64, gScenarioKernelData212_INSERT_CHANNEL_2CH, 4},
{8, 16, 1280, 16, 0, gScenarioBufferData213_INSERT_CHANNEL_2CH, 64, gScenarioKernelData213_INSERT_CHANNEL_2CH, 4},
{10, 16, 1600, 16, 0, gScenarioBufferData214_INSERT_CHANNEL_2CH, 64, gScenarioKernelData214_INSERT_CHANNEL_2CH, 4},
{12, 16, 1920, 16, 0, gScenarioBufferData215_INSERT_CHANNEL_2CH, 64, gScenarioKernelData215_INSERT_CHANNEL_2CH, 4},
{14, 16, 2240, 16, 0, gScenarioBufferData216_INSERT_CHANNEL_2CH, 64, gScenarioKernelData216_INSERT_CHANNEL_2CH, 4},
{16, 16, 2560, 16, 0, gScenarioBufferData217_INSERT_CHANNEL_2CH, 64, gScenarioKernelData217_INSERT_CHANNEL_2CH, 4},
{18, 16, 2880, 16, 0, gScenarioBufferData218_INSERT_CHANNEL_2CH, 64, gScenarioKernelData218_INSERT_CHANNEL_2CH, 4},
{20, 16, 3200, 16, 0, gScenarioBufferData219_INSERT_CHANNEL_2CH, 64, gScenarioKernelData219_INSERT_CHANNEL_2CH, 4},
{22, 16, 3520, 16, 0, gScenarioBufferData220_INSERT_CHANNEL_2CH, 64, gScenarioKernelData220_INSERT_CHANNEL_2CH, 4},
{24, 16, 3840, 16, 0, gScenarioBufferData221_INSERT_CHANNEL_2CH, 64, gScenarioKernelData221_INSERT_CHANNEL_2CH, 4},
{2, 18, 360, 16, 0, gScenarioBufferData222_INSERT_CHANNEL_2CH, 64, gScenarioKernelData222_INSERT_CHANNEL_2CH, 4},
{4, 18, 720, 16, 0, gScenarioBufferData223_INSERT_CHANNEL_2CH, 64, gScenarioKernelData223_INSERT_CHANNEL_2CH, 4},
{6, 18, 1080, 16, 0, gScenarioBufferData224_INSERT_CHANNEL_2CH, 64, gScenarioKernelData224_INSERT_CHANNEL_2CH, 4},
{8, 18, 1440, 16, 0, gScenarioBufferData225_INSERT_CHANNEL_2CH, 64, gScenarioKernelData225_INSERT_CHANNEL_2CH, 4},
{10, 18, 1800, 16, 0, gScenarioBufferData226_INSERT_CHANNEL_2CH, 64, gScenarioKernelData226_INSERT_CHANNEL_2CH, 4},
{12, 18, 2160, 16, 0, gScenarioBufferData227_INSERT_CHANNEL_2CH, 64, gScenarioKernelData227_INSERT_CHANNEL_2CH, 4},
{14, 18, 2520, 16, 0, gScenarioBufferData228_INSERT_CHANNEL_2CH, 64, gScenarioKernelData228_INSERT_CHANNEL_2CH, 4},
{16, 18, 2880, 16, 0, gScenarioBufferData229_INSERT_CHANNEL_2CH, 64, gScenarioKernelData229_INSERT_CHANNEL_2CH, 4},
{18, 18, 3240, 16, 0, gScenarioBufferData230_INSERT_CHANNEL_2CH, 64, gScenarioKernelData230_INSERT_CHANNEL_2CH, 4},
{20, 18, 3600, 16, 0, gScenarioBufferData231_INSERT_CHANNEL_2CH, 64, gScenarioKernelData231_INSERT_CHANNEL_2CH, 4},
{2, 20, 400, 16, 0, gScenarioBufferData232_INSERT_CHANNEL_2CH, 64, gScenarioKernelData232_INSERT_CHANNEL_2CH, 4},
{4, 20, 800, 16, 0, gScenarioBufferData233_INSERT_CHANNEL_2CH, 64, gScenarioKernelData233_INSERT_CHANNEL_2CH, 4},
{6, 20, 1200, 16, 0, gScenarioBufferData234_INSERT_CHANNEL_2CH, 64, gScenarioKernelData234_INSERT_CHANNEL_2CH, 4},
{8, 20, 1600, 16, 0, gScenarioBufferData235_INSERT_CHANNEL_2CH, 64, gScenarioKernelData235_INSERT_CHANNEL_2CH, 4},
{10, 20, 2000, 16, 0, gScenarioBufferData236_INSERT_CHANNEL_2CH, 64, gScenarioKernelData236_INSERT_CHANNEL_2CH, 4},
{12, 20, 2400, 16, 0, gScenarioBufferData237_INSERT_CHANNEL_2CH, 64, gScenarioKernelData237_INSERT_CHANNEL_2CH, 4},
{14, 20, 2800, 16, 0, gScenarioBufferData238_INSERT_CHANNEL_2CH, 64, gScenarioKernelData238_INSERT_CHANNEL_2CH, 4},
{16, 20, 3200, 16, 0, gScenarioBufferData239_INSERT_CHANNEL_2CH, 64, gScenarioKernelData239_INSERT_CHANNEL_2CH, 4},
{18, 20, 3600, 16, 0, gScenarioBufferData240_INSERT_CHANNEL_2CH, 64, gScenarioKernelData240_INSERT_CHANNEL_2CH, 4},
{2, 22, 440, 16, 0, gScenarioBufferData241_INSERT_CHANNEL_2CH, 64, gScenarioKernelData241_INSERT_CHANNEL_2CH, 4},
{4, 22, 880, 16, 0, gScenarioBufferData242_INSERT_CHANNEL_2CH, 64, gScenarioKernelData242_INSERT_CHANNEL_2CH, 4},
{6, 22, 1320, 16, 0, gScenarioBufferData243_INSERT_CHANNEL_2CH, 64, gScenarioKernelData243_INSERT_CHANNEL_2CH, 4},
{8, 22, 1760, 16, 0, gScenarioBufferData244_INSERT_CHANNEL_2CH, 64, gScenarioKernelData244_INSERT_CHANNEL_2CH, 4},
{10, 22, 2200, 16, 0, gScenarioBufferData245_INSERT_CHANNEL_2CH, 64, gScenarioKernelData245_INSERT_CHANNEL_2CH, 4},
{12, 22, 2640, 16, 0, gScenarioBufferData246_INSERT_CHANNEL_2CH, 64, gScenarioKernelData246_INSERT_CHANNEL_2CH, 4},
{14, 22, 3080, 16, 0, gScenarioBufferData247_INSERT_CHANNEL_2CH, 64, gScenarioKernelData247_INSERT_CHANNEL_2CH, 4},
{16, 22, 3520, 16, 0, gScenarioBufferData248_INSERT_CHANNEL_2CH, 64, gScenarioKernelData248_INSERT_CHANNEL_2CH, 4},
{2, 24, 480, 16, 0, gScenarioBufferData249_INSERT_CHANNEL_2CH, 64, gScenarioKernelData249_INSERT_CHANNEL_2CH, 4},
{4, 24, 960, 16, 0, gScenarioBufferData250_INSERT_CHANNEL_2CH, 64, gScenarioKernelData250_INSERT_CHANNEL_2CH, 4},
{6, 24, 1440, 16, 0, gScenarioBufferData251_INSERT_CHANNEL_2CH, 64, gScenarioKernelData251_INSERT_CHANNEL_2CH, 4},
{8, 24, 1920, 16, 0, gScenarioBufferData252_INSERT_CHANNEL_2CH, 64, gScenarioKernelData252_INSERT_CHANNEL_2CH, 4},
{10, 24, 2400, 16, 0, gScenarioBufferData253_INSERT_CHANNEL_2CH, 64, gScenarioKernelData253_INSERT_CHANNEL_2CH, 4},
{12, 24, 2880, 16, 0, gScenarioBufferData254_INSERT_CHANNEL_2CH, 64, gScenarioKernelData254_INSERT_CHANNEL_2CH, 4},
{14, 24, 3360, 16, 0, gScenarioBufferData255_INSERT_CHANNEL_2CH, 64, gScenarioKernelData255_INSERT_CHANNEL_2CH, 4},
{16, 24, 3840, 16, 0, gScenarioBufferData256_INSERT_CHANNEL_2CH, 64, gScenarioKernelData256_INSERT_CHANNEL_2CH, 4},
{2, 26, 520, 16, 0, gScenarioBufferData257_INSERT_CHANNEL_2CH, 64, gScenarioKernelData257_INSERT_CHANNEL_2CH, 4},
{4, 26, 1040, 16, 0, gScenarioBufferData258_INSERT_CHANNEL_2CH, 64, gScenarioKernelData258_INSERT_CHANNEL_2CH, 4},
{6, 26, 1560, 16, 0, gScenarioBufferData259_INSERT_CHANNEL_2CH, 64, gScenarioKernelData259_INSERT_CHANNEL_2CH, 4},
{8, 26, 2080, 16, 0, gScenarioBufferData260_INSERT_CHANNEL_2CH, 64, gScenarioKernelData260_INSERT_CHANNEL_2CH, 4},
{10, 26, 2600, 16, 0, gScenarioBufferData261_INSERT_CHANNEL_2CH, 64, gScenarioKernelData261_INSERT_CHANNEL_2CH, 4},
{12, 26, 3120, 16, 0, gScenarioBufferData262_INSERT_CHANNEL_2CH, 64, gScenarioKernelData262_INSERT_CHANNEL_2CH, 4},
{14, 26, 3640, 16, 0, gScenarioBufferData263_INSERT_CHANNEL_2CH, 64, gScenarioKernelData263_INSERT_CHANNEL_2CH, 4},
{2, 28, 560, 16, 0, gScenarioBufferData264_INSERT_CHANNEL_2CH, 64, gScenarioKernelData264_INSERT_CHANNEL_2CH, 4},
{4, 28, 1120, 16, 0, gScenarioBufferData265_INSERT_CHANNEL_2CH, 64, gScenarioKernelData265_INSERT_CHANNEL_2CH, 4},
{6, 28, 1680, 16, 0, gScenarioBufferData266_INSERT_CHANNEL_2CH, 64, gScenarioKernelData266_INSERT_CHANNEL_2CH, 4},
{8, 28, 2240, 16, 0, gScenarioBufferData267_INSERT_CHANNEL_2CH, 64, gScenarioKernelData267_INSERT_CHANNEL_2CH, 4},
{10, 28, 2800, 16, 0, gScenarioBufferData268_INSERT_CHANNEL_2CH, 64, gScenarioKernelData268_INSERT_CHANNEL_2CH, 4},
{12, 28, 3360, 16, 0, gScenarioBufferData269_INSERT_CHANNEL_2CH, 64, gScenarioKernelData269_INSERT_CHANNEL_2CH, 4},
{2, 30, 600, 16, 0, gScenarioBufferData270_INSERT_CHANNEL_2CH, 64, gScenarioKernelData270_INSERT_CHANNEL_2CH, 4},
{4, 30, 1200, 16, 0, gScenarioBufferData271_INSERT_CHANNEL_2CH, 64, gScenarioKernelData271_INSERT_CHANNEL_2CH, 4},
{6, 30, 1800, 16, 0, gScenarioBufferData272_INSERT_CHANNEL_2CH, 64, gScenarioKernelData272_INSERT_CHANNEL_2CH, 4},
{8, 30, 2400, 16, 0, gScenarioBufferData273_INSERT_CHANNEL_2CH, 64, gScenarioKernelData273_INSERT_CHANNEL_2CH, 4},
{10, 30, 3000, 16, 0, gScenarioBufferData274_INSERT_CHANNEL_2CH, 64, gScenarioKernelData274_INSERT_CHANNEL_2CH, 4},
{12, 30, 3600, 16, 0, gScenarioBufferData275_INSERT_CHANNEL_2CH, 64, gScenarioKernelData275_INSERT_CHANNEL_2CH, 4},
{2, 32, 640, 16, 0, gScenarioBufferData276_INSERT_CHANNEL_2CH, 64, gScenarioKernelData276_INSERT_CHANNEL_2CH, 4},
{4, 32, 1280, 16, 0, gScenarioBufferData277_INSERT_CHANNEL_2CH, 64, gScenarioKernelData277_INSERT_CHANNEL_2CH, 4},
{6, 32, 1920, 16, 0, gScenarioBufferData278_INSERT_CHANNEL_2CH, 64, gScenarioKernelData278_INSERT_CHANNEL_2CH, 4},
{8, 32, 2560, 16, 0, gScenarioBufferData279_INSERT_CHANNEL_2CH, 64, gScenarioKernelData279_INSERT_CHANNEL_2CH, 4},
{10, 32, 3200, 16, 0, gScenarioBufferData280_INSERT_CHANNEL_2CH, 64, gScenarioKernelData280_INSERT_CHANNEL_2CH, 4},
{12, 32, 3840, 16, 0, gScenarioBufferData281_INSERT_CHANNEL_2CH, 64, gScenarioKernelData281_INSERT_CHANNEL_2CH, 4}
};

static acf_scenario_list gScenarioList_INSERT_CHANNEL_2CH = {
282, //number of scenarios
gScenarioArray_INSERT_CHANNEL_2CH};
//**************************************************************

class INSERT_CHANNEL_2CH : public ACF_Process_APU
{

public:
   INSERT_CHANNEL_2CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INSERT_CHANNEL_2CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INSERT_CHANNEL_2CH",
                                        INSERT_CHANNEL_2CH_LOAD_SEGMENTS,
                                        INSERT_CHANNEL_2CH_LOAD_PMEM, INSERT_CHANNEL_2CH_LOAD_PMEM_SIZE,
                                        INSERT_CHANNEL_2CH_LOAD_DMEM, INSERT_CHANNEL_2CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INSERT_CHANNEL_2CH_APEX_LOG_BUFFER, INSERT_CHANNEL_2CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 2, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INSERT_CHANNEL_2CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INSERT_CHANNEL_2CH

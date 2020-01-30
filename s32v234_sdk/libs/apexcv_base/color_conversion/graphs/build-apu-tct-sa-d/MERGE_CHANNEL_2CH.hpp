#ifndef _ACF_PROCESS_APU_MERGE_CHANNEL_2CH
#define _ACF_PROCESS_APU_MERGE_CHANNEL_2CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MERGE_CHANNEL_2CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MERGE_CHANNEL_2CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MERGE_CHANNEL_2CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MERGE_CHANNEL_2CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MERGE_CHANNEL_2CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MERGE_CHANNEL_2CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MERGE_CHANNEL_2CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MERGE_CHANNEL_2CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MERGE_CHANNEL_2CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MERGE_CHANNEL_2CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MERGE_CHANNEL_2CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MERGE_CHANNEL_2CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MERGE_CHANNEL_2CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MERGE_CHANNEL_2CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MERGE_CHANNEL_2CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MERGE_CHANNEL_2CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MERGE_CHANNEL_2CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MERGE_CHANNEL_2CH[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MERGE_CHANNEL_2CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MERGE_CHANNEL_2CH[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MERGE_CHANNEL_2CH[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MERGE_CHANNEL_2CH[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MERGE_CHANNEL_2CH[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MERGE_CHANNEL_2CH[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MERGE_CHANNEL_2CH[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MERGE_CHANNEL_2CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MERGE_CHANNEL_2CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MERGE_CHANNEL_2CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MERGE_CHANNEL_2CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MERGE_CHANNEL_2CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MERGE_CHANNEL_2CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MERGE_CHANNEL_2CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MERGE_CHANNEL_2CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MERGE_CHANNEL_2CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MERGE_CHANNEL_2CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MERGE_CHANNEL_2CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MERGE_CHANNEL_2CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MERGE_CHANNEL_2CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MERGE_CHANNEL_2CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MERGE_CHANNEL_2CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MERGE_CHANNEL_2CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MERGE_CHANNEL_2CH[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MERGE_CHANNEL_2CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MERGE_CHANNEL_2CH[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MERGE_CHANNEL_2CH[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MERGE_CHANNEL_2CH[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MERGE_CHANNEL_2CH[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MERGE_CHANNEL_2CH[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MERGE_CHANNEL_2CH[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MERGE_CHANNEL_2CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MERGE_CHANNEL_2CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MERGE_CHANNEL_2CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MERGE_CHANNEL_2CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MERGE_CHANNEL_2CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MERGE_CHANNEL_2CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MERGE_CHANNEL_2CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MERGE_CHANNEL_2CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MERGE_CHANNEL_2CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MERGE_CHANNEL_2CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MERGE_CHANNEL_2CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MERGE_CHANNEL_2CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MERGE_CHANNEL_2CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MERGE_CHANNEL_2CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MERGE_CHANNEL_2CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MERGE_CHANNEL_2CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MERGE_CHANNEL_2CH[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MERGE_CHANNEL_2CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MERGE_CHANNEL_2CH[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MERGE_CHANNEL_2CH[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MERGE_CHANNEL_2CH[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MERGE_CHANNEL_2CH[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MERGE_CHANNEL_2CH[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MERGE_CHANNEL_2CH[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MERGE_CHANNEL_2CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MERGE_CHANNEL_2CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MERGE_CHANNEL_2CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MERGE_CHANNEL_2CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MERGE_CHANNEL_2CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MERGE_CHANNEL_2CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MERGE_CHANNEL_2CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MERGE_CHANNEL_2CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MERGE_CHANNEL_2CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MERGE_CHANNEL_2CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MERGE_CHANNEL_2CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MERGE_CHANNEL_2CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MERGE_CHANNEL_2CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MERGE_CHANNEL_2CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MERGE_CHANNEL_2CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MERGE_CHANNEL_2CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MERGE_CHANNEL_2CH[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MERGE_CHANNEL_2CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MERGE_CHANNEL_2CH[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MERGE_CHANNEL_2CH[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MERGE_CHANNEL_2CH[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MERGE_CHANNEL_2CH[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MERGE_CHANNEL_2CH[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MERGE_CHANNEL_2CH[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MERGE_CHANNEL_2CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MERGE_CHANNEL_2CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MERGE_CHANNEL_2CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MERGE_CHANNEL_2CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MERGE_CHANNEL_2CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MERGE_CHANNEL_2CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MERGE_CHANNEL_2CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MERGE_CHANNEL_2CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MERGE_CHANNEL_2CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MERGE_CHANNEL_2CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MERGE_CHANNEL_2CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MERGE_CHANNEL_2CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MERGE_CHANNEL_2CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MERGE_CHANNEL_2CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MERGE_CHANNEL_2CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MERGE_CHANNEL_2CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MERGE_CHANNEL_2CH[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MERGE_CHANNEL_2CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MERGE_CHANNEL_2CH[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MERGE_CHANNEL_2CH[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MERGE_CHANNEL_2CH[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MERGE_CHANNEL_2CH[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MERGE_CHANNEL_2CH[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MERGE_CHANNEL_2CH[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MERGE_CHANNEL_2CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MERGE_CHANNEL_2CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MERGE_CHANNEL_2CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MERGE_CHANNEL_2CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MERGE_CHANNEL_2CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MERGE_CHANNEL_2CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MERGE_CHANNEL_2CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MERGE_CHANNEL_2CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MERGE_CHANNEL_2CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MERGE_CHANNEL_2CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MERGE_CHANNEL_2CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MERGE_CHANNEL_2CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MERGE_CHANNEL_2CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MERGE_CHANNEL_2CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MERGE_CHANNEL_2CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MERGE_CHANNEL_2CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MERGE_CHANNEL_2CH[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MERGE_CHANNEL_2CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MERGE_CHANNEL_2CH[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MERGE_CHANNEL_2CH[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MERGE_CHANNEL_2CH[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MERGE_CHANNEL_2CH[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MERGE_CHANNEL_2CH[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MERGE_CHANNEL_2CH[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MERGE_CHANNEL_2CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MERGE_CHANNEL_2CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MERGE_CHANNEL_2CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MERGE_CHANNEL_2CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MERGE_CHANNEL_2CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MERGE_CHANNEL_2CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MERGE_CHANNEL_2CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MERGE_CHANNEL_2CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MERGE_CHANNEL_2CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MERGE_CHANNEL_2CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MERGE_CHANNEL_2CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MERGE_CHANNEL_2CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MERGE_CHANNEL_2CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MERGE_CHANNEL_2CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MERGE_CHANNEL_2CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MERGE_CHANNEL_2CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MERGE_CHANNEL_2CH[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MERGE_CHANNEL_2CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MERGE_CHANNEL_2CH[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MERGE_CHANNEL_2CH[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MERGE_CHANNEL_2CH[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MERGE_CHANNEL_2CH[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MERGE_CHANNEL_2CH[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MERGE_CHANNEL_2CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MERGE_CHANNEL_2CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MERGE_CHANNEL_2CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MERGE_CHANNEL_2CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MERGE_CHANNEL_2CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MERGE_CHANNEL_2CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MERGE_CHANNEL_2CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MERGE_CHANNEL_2CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MERGE_CHANNEL_2CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MERGE_CHANNEL_2CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MERGE_CHANNEL_2CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MERGE_CHANNEL_2CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MERGE_CHANNEL_2CH[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MERGE_CHANNEL_2CH[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MERGE_CHANNEL_2CH[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MERGE_CHANNEL_2CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MERGE_CHANNEL_2CH[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MERGE_CHANNEL_2CH[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MERGE_CHANNEL_2CH[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MERGE_CHANNEL_2CH[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MERGE_CHANNEL_2CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MERGE_CHANNEL_2CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MERGE_CHANNEL_2CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MERGE_CHANNEL_2CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MERGE_CHANNEL_2CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MERGE_CHANNEL_2CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MERGE_CHANNEL_2CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MERGE_CHANNEL_2CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MERGE_CHANNEL_2CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MERGE_CHANNEL_2CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MERGE_CHANNEL_2CH[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MERGE_CHANNEL_2CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MERGE_CHANNEL_2CH[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MERGE_CHANNEL_2CH[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MERGE_CHANNEL_2CH[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MERGE_CHANNEL_2CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MERGE_CHANNEL_2CH[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MERGE_CHANNEL_2CH[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MERGE_CHANNEL_2CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MERGE_CHANNEL_2CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MERGE_CHANNEL_2CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MERGE_CHANNEL_2CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MERGE_CHANNEL_2CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MERGE_CHANNEL_2CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MERGE_CHANNEL_2CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MERGE_CHANNEL_2CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MERGE_CHANNEL_2CH[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MERGE_CHANNEL_2CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MERGE_CHANNEL_2CH[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MERGE_CHANNEL_2CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MERGE_CHANNEL_2CH[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MERGE_CHANNEL_2CH[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MERGE_CHANNEL_2CH[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MERGE_CHANNEL_2CH[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MERGE_CHANNEL_2CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MERGE_CHANNEL_2CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MERGE_CHANNEL_2CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MERGE_CHANNEL_2CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MERGE_CHANNEL_2CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MERGE_CHANNEL_2CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MERGE_CHANNEL_2CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MERGE_CHANNEL_2CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MERGE_CHANNEL_2CH[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MERGE_CHANNEL_2CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MERGE_CHANNEL_2CH[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MERGE_CHANNEL_2CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MERGE_CHANNEL_2CH[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MERGE_CHANNEL_2CH[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MERGE_CHANNEL_2CH[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MERGE_CHANNEL_2CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MERGE_CHANNEL_2CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MERGE_CHANNEL_2CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MERGE_CHANNEL_2CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MERGE_CHANNEL_2CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MERGE_CHANNEL_2CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MERGE_CHANNEL_2CH[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MERGE_CHANNEL_2CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MERGE_CHANNEL_2CH[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MERGE_CHANNEL_2CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MERGE_CHANNEL_2CH[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MERGE_CHANNEL_2CH[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MERGE_CHANNEL_2CH[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MERGE_CHANNEL_2CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MERGE_CHANNEL_2CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_MERGE_CHANNEL_2CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_MERGE_CHANNEL_2CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_MERGE_CHANNEL_2CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_MERGE_CHANNEL_2CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_MERGE_CHANNEL_2CH[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_MERGE_CHANNEL_2CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_MERGE_CHANNEL_2CH[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_MERGE_CHANNEL_2CH[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_MERGE_CHANNEL_2CH[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_MERGE_CHANNEL_2CH[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_MERGE_CHANNEL_2CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_MERGE_CHANNEL_2CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_MERGE_CHANNEL_2CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_MERGE_CHANNEL_2CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_MERGE_CHANNEL_2CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_MERGE_CHANNEL_2CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_MERGE_CHANNEL_2CH[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_MERGE_CHANNEL_2CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_MERGE_CHANNEL_2CH[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_MERGE_CHANNEL_2CH[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_MERGE_CHANNEL_2CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_MERGE_CHANNEL_2CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_MERGE_CHANNEL_2CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_MERGE_CHANNEL_2CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_MERGE_CHANNEL_2CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_MERGE_CHANNEL_2CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_MERGE_CHANNEL_2CH[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_MERGE_CHANNEL_2CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_MERGE_CHANNEL_2CH[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_MERGE_CHANNEL_2CH[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_MERGE_CHANNEL_2CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_MERGE_CHANNEL_2CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_MERGE_CHANNEL_2CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_MERGE_CHANNEL_2CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_MERGE_CHANNEL_2CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_MERGE_CHANNEL_2CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_MERGE_CHANNEL_2CH[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_MERGE_CHANNEL_2CH[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_MERGE_CHANNEL_2CH[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_MERGE_CHANNEL_2CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_MERGE_CHANNEL_2CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_MERGE_CHANNEL_2CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_MERGE_CHANNEL_2CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_MERGE_CHANNEL_2CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_MERGE_CHANNEL_2CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_MERGE_CHANNEL_2CH[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_MERGE_CHANNEL_2CH[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_MERGE_CHANNEL_2CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_MERGE_CHANNEL_2CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_MERGE_CHANNEL_2CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_MERGE_CHANNEL_2CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_MERGE_CHANNEL_2CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_MERGE_CHANNEL_2CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_MERGE_CHANNEL_2CH[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_MERGE_CHANNEL_2CH[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_MERGE_CHANNEL_2CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_MERGE_CHANNEL_2CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_MERGE_CHANNEL_2CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_MERGE_CHANNEL_2CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_MERGE_CHANNEL_2CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_MERGE_CHANNEL_2CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_MERGE_CHANNEL_2CH[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_MERGE_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_MERGE_CHANNEL_2CH[] = {{0, 0}};

static acf_scenario gScenarioArray_MERGE_CHANNEL_2CH[] = {
{2, 1, 16, 0, 0, gScenarioBufferData0_MERGE_CHANNEL_2CH, 48, gScenarioKernelData0_MERGE_CHANNEL_2CH, 4},
{4, 1, 32, 0, 0, gScenarioBufferData1_MERGE_CHANNEL_2CH, 48, gScenarioKernelData1_MERGE_CHANNEL_2CH, 4},
{6, 1, 48, 0, 0, gScenarioBufferData2_MERGE_CHANNEL_2CH, 48, gScenarioKernelData2_MERGE_CHANNEL_2CH, 4},
{8, 1, 64, 0, 0, gScenarioBufferData3_MERGE_CHANNEL_2CH, 48, gScenarioKernelData3_MERGE_CHANNEL_2CH, 4},
{10, 1, 80, 0, 0, gScenarioBufferData4_MERGE_CHANNEL_2CH, 48, gScenarioKernelData4_MERGE_CHANNEL_2CH, 4},
{12, 1, 96, 0, 0, gScenarioBufferData5_MERGE_CHANNEL_2CH, 48, gScenarioKernelData5_MERGE_CHANNEL_2CH, 4},
{14, 1, 112, 0, 0, gScenarioBufferData6_MERGE_CHANNEL_2CH, 48, gScenarioKernelData6_MERGE_CHANNEL_2CH, 4},
{16, 1, 128, 0, 0, gScenarioBufferData7_MERGE_CHANNEL_2CH, 48, gScenarioKernelData7_MERGE_CHANNEL_2CH, 4},
{18, 1, 144, 0, 0, gScenarioBufferData8_MERGE_CHANNEL_2CH, 48, gScenarioKernelData8_MERGE_CHANNEL_2CH, 4},
{20, 1, 160, 0, 0, gScenarioBufferData9_MERGE_CHANNEL_2CH, 48, gScenarioKernelData9_MERGE_CHANNEL_2CH, 4},
{22, 1, 176, 0, 0, gScenarioBufferData10_MERGE_CHANNEL_2CH, 48, gScenarioKernelData10_MERGE_CHANNEL_2CH, 4},
{24, 1, 192, 0, 0, gScenarioBufferData11_MERGE_CHANNEL_2CH, 48, gScenarioKernelData11_MERGE_CHANNEL_2CH, 4},
{26, 1, 208, 0, 0, gScenarioBufferData12_MERGE_CHANNEL_2CH, 48, gScenarioKernelData12_MERGE_CHANNEL_2CH, 4},
{28, 1, 224, 0, 0, gScenarioBufferData13_MERGE_CHANNEL_2CH, 48, gScenarioKernelData13_MERGE_CHANNEL_2CH, 4},
{30, 1, 240, 0, 0, gScenarioBufferData14_MERGE_CHANNEL_2CH, 48, gScenarioKernelData14_MERGE_CHANNEL_2CH, 4},
{32, 1, 256, 0, 0, gScenarioBufferData15_MERGE_CHANNEL_2CH, 48, gScenarioKernelData15_MERGE_CHANNEL_2CH, 4},
{36, 1, 288, 0, 0, gScenarioBufferData16_MERGE_CHANNEL_2CH, 48, gScenarioKernelData16_MERGE_CHANNEL_2CH, 4},
{40, 1, 320, 0, 0, gScenarioBufferData17_MERGE_CHANNEL_2CH, 48, gScenarioKernelData17_MERGE_CHANNEL_2CH, 4},
{44, 1, 352, 0, 0, gScenarioBufferData18_MERGE_CHANNEL_2CH, 48, gScenarioKernelData18_MERGE_CHANNEL_2CH, 4},
{48, 1, 384, 0, 0, gScenarioBufferData19_MERGE_CHANNEL_2CH, 48, gScenarioKernelData19_MERGE_CHANNEL_2CH, 4},
{52, 1, 416, 0, 0, gScenarioBufferData20_MERGE_CHANNEL_2CH, 48, gScenarioKernelData20_MERGE_CHANNEL_2CH, 4},
{56, 1, 448, 0, 0, gScenarioBufferData21_MERGE_CHANNEL_2CH, 48, gScenarioKernelData21_MERGE_CHANNEL_2CH, 4},
{60, 1, 480, 0, 0, gScenarioBufferData22_MERGE_CHANNEL_2CH, 48, gScenarioKernelData22_MERGE_CHANNEL_2CH, 4},
{64, 1, 512, 0, 0, gScenarioBufferData23_MERGE_CHANNEL_2CH, 48, gScenarioKernelData23_MERGE_CHANNEL_2CH, 4},
{2, 2, 32, 0, 0, gScenarioBufferData24_MERGE_CHANNEL_2CH, 48, gScenarioKernelData24_MERGE_CHANNEL_2CH, 4},
{4, 2, 64, 0, 0, gScenarioBufferData25_MERGE_CHANNEL_2CH, 48, gScenarioKernelData25_MERGE_CHANNEL_2CH, 4},
{6, 2, 96, 0, 0, gScenarioBufferData26_MERGE_CHANNEL_2CH, 48, gScenarioKernelData26_MERGE_CHANNEL_2CH, 4},
{8, 2, 128, 0, 0, gScenarioBufferData27_MERGE_CHANNEL_2CH, 48, gScenarioKernelData27_MERGE_CHANNEL_2CH, 4},
{10, 2, 160, 0, 0, gScenarioBufferData28_MERGE_CHANNEL_2CH, 48, gScenarioKernelData28_MERGE_CHANNEL_2CH, 4},
{12, 2, 192, 0, 0, gScenarioBufferData29_MERGE_CHANNEL_2CH, 48, gScenarioKernelData29_MERGE_CHANNEL_2CH, 4},
{14, 2, 224, 0, 0, gScenarioBufferData30_MERGE_CHANNEL_2CH, 48, gScenarioKernelData30_MERGE_CHANNEL_2CH, 4},
{16, 2, 256, 0, 0, gScenarioBufferData31_MERGE_CHANNEL_2CH, 48, gScenarioKernelData31_MERGE_CHANNEL_2CH, 4},
{18, 2, 288, 0, 0, gScenarioBufferData32_MERGE_CHANNEL_2CH, 48, gScenarioKernelData32_MERGE_CHANNEL_2CH, 4},
{20, 2, 320, 0, 0, gScenarioBufferData33_MERGE_CHANNEL_2CH, 48, gScenarioKernelData33_MERGE_CHANNEL_2CH, 4},
{22, 2, 352, 0, 0, gScenarioBufferData34_MERGE_CHANNEL_2CH, 48, gScenarioKernelData34_MERGE_CHANNEL_2CH, 4},
{24, 2, 384, 0, 0, gScenarioBufferData35_MERGE_CHANNEL_2CH, 48, gScenarioKernelData35_MERGE_CHANNEL_2CH, 4},
{26, 2, 416, 0, 0, gScenarioBufferData36_MERGE_CHANNEL_2CH, 48, gScenarioKernelData36_MERGE_CHANNEL_2CH, 4},
{28, 2, 448, 0, 0, gScenarioBufferData37_MERGE_CHANNEL_2CH, 48, gScenarioKernelData37_MERGE_CHANNEL_2CH, 4},
{30, 2, 480, 0, 0, gScenarioBufferData38_MERGE_CHANNEL_2CH, 48, gScenarioKernelData38_MERGE_CHANNEL_2CH, 4},
{32, 2, 512, 0, 0, gScenarioBufferData39_MERGE_CHANNEL_2CH, 48, gScenarioKernelData39_MERGE_CHANNEL_2CH, 4},
{36, 2, 576, 0, 0, gScenarioBufferData40_MERGE_CHANNEL_2CH, 48, gScenarioKernelData40_MERGE_CHANNEL_2CH, 4},
{40, 2, 640, 0, 0, gScenarioBufferData41_MERGE_CHANNEL_2CH, 48, gScenarioKernelData41_MERGE_CHANNEL_2CH, 4},
{44, 2, 704, 0, 0, gScenarioBufferData42_MERGE_CHANNEL_2CH, 48, gScenarioKernelData42_MERGE_CHANNEL_2CH, 4},
{48, 2, 768, 0, 0, gScenarioBufferData43_MERGE_CHANNEL_2CH, 48, gScenarioKernelData43_MERGE_CHANNEL_2CH, 4},
{52, 2, 832, 0, 0, gScenarioBufferData44_MERGE_CHANNEL_2CH, 48, gScenarioKernelData44_MERGE_CHANNEL_2CH, 4},
{56, 2, 896, 0, 0, gScenarioBufferData45_MERGE_CHANNEL_2CH, 48, gScenarioKernelData45_MERGE_CHANNEL_2CH, 4},
{60, 2, 960, 0, 0, gScenarioBufferData46_MERGE_CHANNEL_2CH, 48, gScenarioKernelData46_MERGE_CHANNEL_2CH, 4},
{64, 2, 1024, 0, 0, gScenarioBufferData47_MERGE_CHANNEL_2CH, 48, gScenarioKernelData47_MERGE_CHANNEL_2CH, 4},
{2, 3, 48, 0, 0, gScenarioBufferData48_MERGE_CHANNEL_2CH, 48, gScenarioKernelData48_MERGE_CHANNEL_2CH, 4},
{4, 3, 96, 0, 0, gScenarioBufferData49_MERGE_CHANNEL_2CH, 48, gScenarioKernelData49_MERGE_CHANNEL_2CH, 4},
{6, 3, 144, 0, 0, gScenarioBufferData50_MERGE_CHANNEL_2CH, 48, gScenarioKernelData50_MERGE_CHANNEL_2CH, 4},
{8, 3, 192, 0, 0, gScenarioBufferData51_MERGE_CHANNEL_2CH, 48, gScenarioKernelData51_MERGE_CHANNEL_2CH, 4},
{10, 3, 240, 0, 0, gScenarioBufferData52_MERGE_CHANNEL_2CH, 48, gScenarioKernelData52_MERGE_CHANNEL_2CH, 4},
{12, 3, 288, 0, 0, gScenarioBufferData53_MERGE_CHANNEL_2CH, 48, gScenarioKernelData53_MERGE_CHANNEL_2CH, 4},
{14, 3, 336, 0, 0, gScenarioBufferData54_MERGE_CHANNEL_2CH, 48, gScenarioKernelData54_MERGE_CHANNEL_2CH, 4},
{16, 3, 384, 0, 0, gScenarioBufferData55_MERGE_CHANNEL_2CH, 48, gScenarioKernelData55_MERGE_CHANNEL_2CH, 4},
{18, 3, 432, 0, 0, gScenarioBufferData56_MERGE_CHANNEL_2CH, 48, gScenarioKernelData56_MERGE_CHANNEL_2CH, 4},
{20, 3, 480, 0, 0, gScenarioBufferData57_MERGE_CHANNEL_2CH, 48, gScenarioKernelData57_MERGE_CHANNEL_2CH, 4},
{22, 3, 528, 0, 0, gScenarioBufferData58_MERGE_CHANNEL_2CH, 48, gScenarioKernelData58_MERGE_CHANNEL_2CH, 4},
{24, 3, 576, 0, 0, gScenarioBufferData59_MERGE_CHANNEL_2CH, 48, gScenarioKernelData59_MERGE_CHANNEL_2CH, 4},
{26, 3, 624, 0, 0, gScenarioBufferData60_MERGE_CHANNEL_2CH, 48, gScenarioKernelData60_MERGE_CHANNEL_2CH, 4},
{28, 3, 672, 0, 0, gScenarioBufferData61_MERGE_CHANNEL_2CH, 48, gScenarioKernelData61_MERGE_CHANNEL_2CH, 4},
{30, 3, 720, 0, 0, gScenarioBufferData62_MERGE_CHANNEL_2CH, 48, gScenarioKernelData62_MERGE_CHANNEL_2CH, 4},
{32, 3, 768, 0, 0, gScenarioBufferData63_MERGE_CHANNEL_2CH, 48, gScenarioKernelData63_MERGE_CHANNEL_2CH, 4},
{36, 3, 864, 0, 0, gScenarioBufferData64_MERGE_CHANNEL_2CH, 48, gScenarioKernelData64_MERGE_CHANNEL_2CH, 4},
{40, 3, 960, 0, 0, gScenarioBufferData65_MERGE_CHANNEL_2CH, 48, gScenarioKernelData65_MERGE_CHANNEL_2CH, 4},
{44, 3, 1056, 0, 0, gScenarioBufferData66_MERGE_CHANNEL_2CH, 48, gScenarioKernelData66_MERGE_CHANNEL_2CH, 4},
{48, 3, 1152, 0, 0, gScenarioBufferData67_MERGE_CHANNEL_2CH, 48, gScenarioKernelData67_MERGE_CHANNEL_2CH, 4},
{52, 3, 1248, 0, 0, gScenarioBufferData68_MERGE_CHANNEL_2CH, 48, gScenarioKernelData68_MERGE_CHANNEL_2CH, 4},
{56, 3, 1344, 0, 0, gScenarioBufferData69_MERGE_CHANNEL_2CH, 48, gScenarioKernelData69_MERGE_CHANNEL_2CH, 4},
{60, 3, 1440, 0, 0, gScenarioBufferData70_MERGE_CHANNEL_2CH, 48, gScenarioKernelData70_MERGE_CHANNEL_2CH, 4},
{64, 3, 1536, 0, 0, gScenarioBufferData71_MERGE_CHANNEL_2CH, 48, gScenarioKernelData71_MERGE_CHANNEL_2CH, 4},
{2, 4, 64, 0, 0, gScenarioBufferData72_MERGE_CHANNEL_2CH, 48, gScenarioKernelData72_MERGE_CHANNEL_2CH, 4},
{4, 4, 128, 0, 0, gScenarioBufferData73_MERGE_CHANNEL_2CH, 48, gScenarioKernelData73_MERGE_CHANNEL_2CH, 4},
{6, 4, 192, 0, 0, gScenarioBufferData74_MERGE_CHANNEL_2CH, 48, gScenarioKernelData74_MERGE_CHANNEL_2CH, 4},
{8, 4, 256, 0, 0, gScenarioBufferData75_MERGE_CHANNEL_2CH, 48, gScenarioKernelData75_MERGE_CHANNEL_2CH, 4},
{10, 4, 320, 0, 0, gScenarioBufferData76_MERGE_CHANNEL_2CH, 48, gScenarioKernelData76_MERGE_CHANNEL_2CH, 4},
{12, 4, 384, 0, 0, gScenarioBufferData77_MERGE_CHANNEL_2CH, 48, gScenarioKernelData77_MERGE_CHANNEL_2CH, 4},
{14, 4, 448, 0, 0, gScenarioBufferData78_MERGE_CHANNEL_2CH, 48, gScenarioKernelData78_MERGE_CHANNEL_2CH, 4},
{16, 4, 512, 0, 0, gScenarioBufferData79_MERGE_CHANNEL_2CH, 48, gScenarioKernelData79_MERGE_CHANNEL_2CH, 4},
{18, 4, 576, 0, 0, gScenarioBufferData80_MERGE_CHANNEL_2CH, 48, gScenarioKernelData80_MERGE_CHANNEL_2CH, 4},
{20, 4, 640, 0, 0, gScenarioBufferData81_MERGE_CHANNEL_2CH, 48, gScenarioKernelData81_MERGE_CHANNEL_2CH, 4},
{22, 4, 704, 0, 0, gScenarioBufferData82_MERGE_CHANNEL_2CH, 48, gScenarioKernelData82_MERGE_CHANNEL_2CH, 4},
{24, 4, 768, 0, 0, gScenarioBufferData83_MERGE_CHANNEL_2CH, 48, gScenarioKernelData83_MERGE_CHANNEL_2CH, 4},
{26, 4, 832, 0, 0, gScenarioBufferData84_MERGE_CHANNEL_2CH, 48, gScenarioKernelData84_MERGE_CHANNEL_2CH, 4},
{28, 4, 896, 0, 0, gScenarioBufferData85_MERGE_CHANNEL_2CH, 48, gScenarioKernelData85_MERGE_CHANNEL_2CH, 4},
{30, 4, 960, 0, 0, gScenarioBufferData86_MERGE_CHANNEL_2CH, 48, gScenarioKernelData86_MERGE_CHANNEL_2CH, 4},
{32, 4, 1024, 0, 0, gScenarioBufferData87_MERGE_CHANNEL_2CH, 48, gScenarioKernelData87_MERGE_CHANNEL_2CH, 4},
{36, 4, 1152, 0, 0, gScenarioBufferData88_MERGE_CHANNEL_2CH, 48, gScenarioKernelData88_MERGE_CHANNEL_2CH, 4},
{40, 4, 1280, 0, 0, gScenarioBufferData89_MERGE_CHANNEL_2CH, 48, gScenarioKernelData89_MERGE_CHANNEL_2CH, 4},
{44, 4, 1408, 0, 0, gScenarioBufferData90_MERGE_CHANNEL_2CH, 48, gScenarioKernelData90_MERGE_CHANNEL_2CH, 4},
{48, 4, 1536, 0, 0, gScenarioBufferData91_MERGE_CHANNEL_2CH, 48, gScenarioKernelData91_MERGE_CHANNEL_2CH, 4},
{52, 4, 1664, 0, 0, gScenarioBufferData92_MERGE_CHANNEL_2CH, 48, gScenarioKernelData92_MERGE_CHANNEL_2CH, 4},
{56, 4, 1792, 0, 0, gScenarioBufferData93_MERGE_CHANNEL_2CH, 48, gScenarioKernelData93_MERGE_CHANNEL_2CH, 4},
{60, 4, 1920, 0, 0, gScenarioBufferData94_MERGE_CHANNEL_2CH, 48, gScenarioKernelData94_MERGE_CHANNEL_2CH, 4},
{64, 4, 2048, 0, 0, gScenarioBufferData95_MERGE_CHANNEL_2CH, 48, gScenarioKernelData95_MERGE_CHANNEL_2CH, 4},
{2, 5, 80, 0, 0, gScenarioBufferData96_MERGE_CHANNEL_2CH, 48, gScenarioKernelData96_MERGE_CHANNEL_2CH, 4},
{4, 5, 160, 0, 0, gScenarioBufferData97_MERGE_CHANNEL_2CH, 48, gScenarioKernelData97_MERGE_CHANNEL_2CH, 4},
{6, 5, 240, 0, 0, gScenarioBufferData98_MERGE_CHANNEL_2CH, 48, gScenarioKernelData98_MERGE_CHANNEL_2CH, 4},
{8, 5, 320, 0, 0, gScenarioBufferData99_MERGE_CHANNEL_2CH, 48, gScenarioKernelData99_MERGE_CHANNEL_2CH, 4},
{10, 5, 400, 0, 0, gScenarioBufferData100_MERGE_CHANNEL_2CH, 48, gScenarioKernelData100_MERGE_CHANNEL_2CH, 4},
{12, 5, 480, 0, 0, gScenarioBufferData101_MERGE_CHANNEL_2CH, 48, gScenarioKernelData101_MERGE_CHANNEL_2CH, 4},
{14, 5, 560, 0, 0, gScenarioBufferData102_MERGE_CHANNEL_2CH, 48, gScenarioKernelData102_MERGE_CHANNEL_2CH, 4},
{16, 5, 640, 0, 0, gScenarioBufferData103_MERGE_CHANNEL_2CH, 48, gScenarioKernelData103_MERGE_CHANNEL_2CH, 4},
{18, 5, 720, 0, 0, gScenarioBufferData104_MERGE_CHANNEL_2CH, 48, gScenarioKernelData104_MERGE_CHANNEL_2CH, 4},
{20, 5, 800, 0, 0, gScenarioBufferData105_MERGE_CHANNEL_2CH, 48, gScenarioKernelData105_MERGE_CHANNEL_2CH, 4},
{22, 5, 880, 0, 0, gScenarioBufferData106_MERGE_CHANNEL_2CH, 48, gScenarioKernelData106_MERGE_CHANNEL_2CH, 4},
{24, 5, 960, 0, 0, gScenarioBufferData107_MERGE_CHANNEL_2CH, 48, gScenarioKernelData107_MERGE_CHANNEL_2CH, 4},
{26, 5, 1040, 0, 0, gScenarioBufferData108_MERGE_CHANNEL_2CH, 48, gScenarioKernelData108_MERGE_CHANNEL_2CH, 4},
{28, 5, 1120, 0, 0, gScenarioBufferData109_MERGE_CHANNEL_2CH, 48, gScenarioKernelData109_MERGE_CHANNEL_2CH, 4},
{30, 5, 1200, 0, 0, gScenarioBufferData110_MERGE_CHANNEL_2CH, 48, gScenarioKernelData110_MERGE_CHANNEL_2CH, 4},
{32, 5, 1280, 0, 0, gScenarioBufferData111_MERGE_CHANNEL_2CH, 48, gScenarioKernelData111_MERGE_CHANNEL_2CH, 4},
{36, 5, 1440, 0, 0, gScenarioBufferData112_MERGE_CHANNEL_2CH, 48, gScenarioKernelData112_MERGE_CHANNEL_2CH, 4},
{40, 5, 1600, 0, 0, gScenarioBufferData113_MERGE_CHANNEL_2CH, 48, gScenarioKernelData113_MERGE_CHANNEL_2CH, 4},
{44, 5, 1760, 0, 0, gScenarioBufferData114_MERGE_CHANNEL_2CH, 48, gScenarioKernelData114_MERGE_CHANNEL_2CH, 4},
{48, 5, 1920, 0, 0, gScenarioBufferData115_MERGE_CHANNEL_2CH, 48, gScenarioKernelData115_MERGE_CHANNEL_2CH, 4},
{52, 5, 2080, 0, 0, gScenarioBufferData116_MERGE_CHANNEL_2CH, 48, gScenarioKernelData116_MERGE_CHANNEL_2CH, 4},
{56, 5, 2240, 0, 0, gScenarioBufferData117_MERGE_CHANNEL_2CH, 48, gScenarioKernelData117_MERGE_CHANNEL_2CH, 4},
{60, 5, 2400, 0, 0, gScenarioBufferData118_MERGE_CHANNEL_2CH, 48, gScenarioKernelData118_MERGE_CHANNEL_2CH, 4},
{64, 5, 2560, 0, 0, gScenarioBufferData119_MERGE_CHANNEL_2CH, 48, gScenarioKernelData119_MERGE_CHANNEL_2CH, 4},
{2, 6, 96, 0, 0, gScenarioBufferData120_MERGE_CHANNEL_2CH, 48, gScenarioKernelData120_MERGE_CHANNEL_2CH, 4},
{4, 6, 192, 0, 0, gScenarioBufferData121_MERGE_CHANNEL_2CH, 48, gScenarioKernelData121_MERGE_CHANNEL_2CH, 4},
{6, 6, 288, 0, 0, gScenarioBufferData122_MERGE_CHANNEL_2CH, 48, gScenarioKernelData122_MERGE_CHANNEL_2CH, 4},
{8, 6, 384, 0, 0, gScenarioBufferData123_MERGE_CHANNEL_2CH, 48, gScenarioKernelData123_MERGE_CHANNEL_2CH, 4},
{10, 6, 480, 0, 0, gScenarioBufferData124_MERGE_CHANNEL_2CH, 48, gScenarioKernelData124_MERGE_CHANNEL_2CH, 4},
{12, 6, 576, 0, 0, gScenarioBufferData125_MERGE_CHANNEL_2CH, 48, gScenarioKernelData125_MERGE_CHANNEL_2CH, 4},
{14, 6, 672, 0, 0, gScenarioBufferData126_MERGE_CHANNEL_2CH, 48, gScenarioKernelData126_MERGE_CHANNEL_2CH, 4},
{16, 6, 768, 0, 0, gScenarioBufferData127_MERGE_CHANNEL_2CH, 48, gScenarioKernelData127_MERGE_CHANNEL_2CH, 4},
{18, 6, 864, 0, 0, gScenarioBufferData128_MERGE_CHANNEL_2CH, 48, gScenarioKernelData128_MERGE_CHANNEL_2CH, 4},
{20, 6, 960, 0, 0, gScenarioBufferData129_MERGE_CHANNEL_2CH, 48, gScenarioKernelData129_MERGE_CHANNEL_2CH, 4},
{22, 6, 1056, 0, 0, gScenarioBufferData130_MERGE_CHANNEL_2CH, 48, gScenarioKernelData130_MERGE_CHANNEL_2CH, 4},
{24, 6, 1152, 0, 0, gScenarioBufferData131_MERGE_CHANNEL_2CH, 48, gScenarioKernelData131_MERGE_CHANNEL_2CH, 4},
{26, 6, 1248, 0, 0, gScenarioBufferData132_MERGE_CHANNEL_2CH, 48, gScenarioKernelData132_MERGE_CHANNEL_2CH, 4},
{28, 6, 1344, 0, 0, gScenarioBufferData133_MERGE_CHANNEL_2CH, 48, gScenarioKernelData133_MERGE_CHANNEL_2CH, 4},
{30, 6, 1440, 0, 0, gScenarioBufferData134_MERGE_CHANNEL_2CH, 48, gScenarioKernelData134_MERGE_CHANNEL_2CH, 4},
{32, 6, 1536, 0, 0, gScenarioBufferData135_MERGE_CHANNEL_2CH, 48, gScenarioKernelData135_MERGE_CHANNEL_2CH, 4},
{36, 6, 1728, 0, 0, gScenarioBufferData136_MERGE_CHANNEL_2CH, 48, gScenarioKernelData136_MERGE_CHANNEL_2CH, 4},
{40, 6, 1920, 0, 0, gScenarioBufferData137_MERGE_CHANNEL_2CH, 48, gScenarioKernelData137_MERGE_CHANNEL_2CH, 4},
{44, 6, 2112, 0, 0, gScenarioBufferData138_MERGE_CHANNEL_2CH, 48, gScenarioKernelData138_MERGE_CHANNEL_2CH, 4},
{48, 6, 2304, 0, 0, gScenarioBufferData139_MERGE_CHANNEL_2CH, 48, gScenarioKernelData139_MERGE_CHANNEL_2CH, 4},
{52, 6, 2496, 0, 0, gScenarioBufferData140_MERGE_CHANNEL_2CH, 48, gScenarioKernelData140_MERGE_CHANNEL_2CH, 4},
{56, 6, 2688, 0, 0, gScenarioBufferData141_MERGE_CHANNEL_2CH, 48, gScenarioKernelData141_MERGE_CHANNEL_2CH, 4},
{60, 6, 2880, 0, 0, gScenarioBufferData142_MERGE_CHANNEL_2CH, 48, gScenarioKernelData142_MERGE_CHANNEL_2CH, 4},
{64, 6, 3072, 0, 0, gScenarioBufferData143_MERGE_CHANNEL_2CH, 48, gScenarioKernelData143_MERGE_CHANNEL_2CH, 4},
{2, 8, 128, 0, 0, gScenarioBufferData144_MERGE_CHANNEL_2CH, 48, gScenarioKernelData144_MERGE_CHANNEL_2CH, 4},
{4, 8, 256, 0, 0, gScenarioBufferData145_MERGE_CHANNEL_2CH, 48, gScenarioKernelData145_MERGE_CHANNEL_2CH, 4},
{6, 8, 384, 0, 0, gScenarioBufferData146_MERGE_CHANNEL_2CH, 48, gScenarioKernelData146_MERGE_CHANNEL_2CH, 4},
{8, 8, 512, 0, 0, gScenarioBufferData147_MERGE_CHANNEL_2CH, 48, gScenarioKernelData147_MERGE_CHANNEL_2CH, 4},
{10, 8, 640, 0, 0, gScenarioBufferData148_MERGE_CHANNEL_2CH, 48, gScenarioKernelData148_MERGE_CHANNEL_2CH, 4},
{12, 8, 768, 0, 0, gScenarioBufferData149_MERGE_CHANNEL_2CH, 48, gScenarioKernelData149_MERGE_CHANNEL_2CH, 4},
{14, 8, 896, 0, 0, gScenarioBufferData150_MERGE_CHANNEL_2CH, 48, gScenarioKernelData150_MERGE_CHANNEL_2CH, 4},
{16, 8, 1024, 0, 0, gScenarioBufferData151_MERGE_CHANNEL_2CH, 48, gScenarioKernelData151_MERGE_CHANNEL_2CH, 4},
{18, 8, 1152, 0, 0, gScenarioBufferData152_MERGE_CHANNEL_2CH, 48, gScenarioKernelData152_MERGE_CHANNEL_2CH, 4},
{20, 8, 1280, 0, 0, gScenarioBufferData153_MERGE_CHANNEL_2CH, 48, gScenarioKernelData153_MERGE_CHANNEL_2CH, 4},
{22, 8, 1408, 0, 0, gScenarioBufferData154_MERGE_CHANNEL_2CH, 48, gScenarioKernelData154_MERGE_CHANNEL_2CH, 4},
{24, 8, 1536, 0, 0, gScenarioBufferData155_MERGE_CHANNEL_2CH, 48, gScenarioKernelData155_MERGE_CHANNEL_2CH, 4},
{26, 8, 1664, 0, 0, gScenarioBufferData156_MERGE_CHANNEL_2CH, 48, gScenarioKernelData156_MERGE_CHANNEL_2CH, 4},
{28, 8, 1792, 0, 0, gScenarioBufferData157_MERGE_CHANNEL_2CH, 48, gScenarioKernelData157_MERGE_CHANNEL_2CH, 4},
{30, 8, 1920, 0, 0, gScenarioBufferData158_MERGE_CHANNEL_2CH, 48, gScenarioKernelData158_MERGE_CHANNEL_2CH, 4},
{32, 8, 2048, 0, 0, gScenarioBufferData159_MERGE_CHANNEL_2CH, 48, gScenarioKernelData159_MERGE_CHANNEL_2CH, 4},
{36, 8, 2304, 0, 0, gScenarioBufferData160_MERGE_CHANNEL_2CH, 48, gScenarioKernelData160_MERGE_CHANNEL_2CH, 4},
{40, 8, 2560, 0, 0, gScenarioBufferData161_MERGE_CHANNEL_2CH, 48, gScenarioKernelData161_MERGE_CHANNEL_2CH, 4},
{44, 8, 2816, 0, 0, gScenarioBufferData162_MERGE_CHANNEL_2CH, 48, gScenarioKernelData162_MERGE_CHANNEL_2CH, 4},
{48, 8, 3072, 0, 0, gScenarioBufferData163_MERGE_CHANNEL_2CH, 48, gScenarioKernelData163_MERGE_CHANNEL_2CH, 4},
{52, 8, 3328, 0, 0, gScenarioBufferData164_MERGE_CHANNEL_2CH, 48, gScenarioKernelData164_MERGE_CHANNEL_2CH, 4},
{56, 8, 3584, 0, 0, gScenarioBufferData165_MERGE_CHANNEL_2CH, 48, gScenarioKernelData165_MERGE_CHANNEL_2CH, 4},
{60, 8, 3840, 0, 0, gScenarioBufferData166_MERGE_CHANNEL_2CH, 48, gScenarioKernelData166_MERGE_CHANNEL_2CH, 4},
{2, 10, 160, 0, 0, gScenarioBufferData167_MERGE_CHANNEL_2CH, 48, gScenarioKernelData167_MERGE_CHANNEL_2CH, 4},
{4, 10, 320, 0, 0, gScenarioBufferData168_MERGE_CHANNEL_2CH, 48, gScenarioKernelData168_MERGE_CHANNEL_2CH, 4},
{6, 10, 480, 0, 0, gScenarioBufferData169_MERGE_CHANNEL_2CH, 48, gScenarioKernelData169_MERGE_CHANNEL_2CH, 4},
{8, 10, 640, 0, 0, gScenarioBufferData170_MERGE_CHANNEL_2CH, 48, gScenarioKernelData170_MERGE_CHANNEL_2CH, 4},
{10, 10, 800, 0, 0, gScenarioBufferData171_MERGE_CHANNEL_2CH, 48, gScenarioKernelData171_MERGE_CHANNEL_2CH, 4},
{12, 10, 960, 0, 0, gScenarioBufferData172_MERGE_CHANNEL_2CH, 48, gScenarioKernelData172_MERGE_CHANNEL_2CH, 4},
{14, 10, 1120, 0, 0, gScenarioBufferData173_MERGE_CHANNEL_2CH, 48, gScenarioKernelData173_MERGE_CHANNEL_2CH, 4},
{16, 10, 1280, 0, 0, gScenarioBufferData174_MERGE_CHANNEL_2CH, 48, gScenarioKernelData174_MERGE_CHANNEL_2CH, 4},
{18, 10, 1440, 0, 0, gScenarioBufferData175_MERGE_CHANNEL_2CH, 48, gScenarioKernelData175_MERGE_CHANNEL_2CH, 4},
{20, 10, 1600, 0, 0, gScenarioBufferData176_MERGE_CHANNEL_2CH, 48, gScenarioKernelData176_MERGE_CHANNEL_2CH, 4},
{22, 10, 1760, 0, 0, gScenarioBufferData177_MERGE_CHANNEL_2CH, 48, gScenarioKernelData177_MERGE_CHANNEL_2CH, 4},
{24, 10, 1920, 0, 0, gScenarioBufferData178_MERGE_CHANNEL_2CH, 48, gScenarioKernelData178_MERGE_CHANNEL_2CH, 4},
{26, 10, 2080, 0, 0, gScenarioBufferData179_MERGE_CHANNEL_2CH, 48, gScenarioKernelData179_MERGE_CHANNEL_2CH, 4},
{28, 10, 2240, 0, 0, gScenarioBufferData180_MERGE_CHANNEL_2CH, 48, gScenarioKernelData180_MERGE_CHANNEL_2CH, 4},
{30, 10, 2400, 0, 0, gScenarioBufferData181_MERGE_CHANNEL_2CH, 48, gScenarioKernelData181_MERGE_CHANNEL_2CH, 4},
{32, 10, 2560, 0, 0, gScenarioBufferData182_MERGE_CHANNEL_2CH, 48, gScenarioKernelData182_MERGE_CHANNEL_2CH, 4},
{36, 10, 2880, 0, 0, gScenarioBufferData183_MERGE_CHANNEL_2CH, 48, gScenarioKernelData183_MERGE_CHANNEL_2CH, 4},
{40, 10, 3200, 0, 0, gScenarioBufferData184_MERGE_CHANNEL_2CH, 48, gScenarioKernelData184_MERGE_CHANNEL_2CH, 4},
{44, 10, 3520, 0, 0, gScenarioBufferData185_MERGE_CHANNEL_2CH, 48, gScenarioKernelData185_MERGE_CHANNEL_2CH, 4},
{48, 10, 3840, 0, 0, gScenarioBufferData186_MERGE_CHANNEL_2CH, 48, gScenarioKernelData186_MERGE_CHANNEL_2CH, 4},
{2, 12, 192, 0, 0, gScenarioBufferData187_MERGE_CHANNEL_2CH, 48, gScenarioKernelData187_MERGE_CHANNEL_2CH, 4},
{4, 12, 384, 0, 0, gScenarioBufferData188_MERGE_CHANNEL_2CH, 48, gScenarioKernelData188_MERGE_CHANNEL_2CH, 4},
{6, 12, 576, 0, 0, gScenarioBufferData189_MERGE_CHANNEL_2CH, 48, gScenarioKernelData189_MERGE_CHANNEL_2CH, 4},
{8, 12, 768, 0, 0, gScenarioBufferData190_MERGE_CHANNEL_2CH, 48, gScenarioKernelData190_MERGE_CHANNEL_2CH, 4},
{10, 12, 960, 0, 0, gScenarioBufferData191_MERGE_CHANNEL_2CH, 48, gScenarioKernelData191_MERGE_CHANNEL_2CH, 4},
{12, 12, 1152, 0, 0, gScenarioBufferData192_MERGE_CHANNEL_2CH, 48, gScenarioKernelData192_MERGE_CHANNEL_2CH, 4},
{14, 12, 1344, 0, 0, gScenarioBufferData193_MERGE_CHANNEL_2CH, 48, gScenarioKernelData193_MERGE_CHANNEL_2CH, 4},
{16, 12, 1536, 0, 0, gScenarioBufferData194_MERGE_CHANNEL_2CH, 48, gScenarioKernelData194_MERGE_CHANNEL_2CH, 4},
{18, 12, 1728, 0, 0, gScenarioBufferData195_MERGE_CHANNEL_2CH, 48, gScenarioKernelData195_MERGE_CHANNEL_2CH, 4},
{20, 12, 1920, 0, 0, gScenarioBufferData196_MERGE_CHANNEL_2CH, 48, gScenarioKernelData196_MERGE_CHANNEL_2CH, 4},
{22, 12, 2112, 0, 0, gScenarioBufferData197_MERGE_CHANNEL_2CH, 48, gScenarioKernelData197_MERGE_CHANNEL_2CH, 4},
{24, 12, 2304, 0, 0, gScenarioBufferData198_MERGE_CHANNEL_2CH, 48, gScenarioKernelData198_MERGE_CHANNEL_2CH, 4},
{26, 12, 2496, 0, 0, gScenarioBufferData199_MERGE_CHANNEL_2CH, 48, gScenarioKernelData199_MERGE_CHANNEL_2CH, 4},
{28, 12, 2688, 0, 0, gScenarioBufferData200_MERGE_CHANNEL_2CH, 48, gScenarioKernelData200_MERGE_CHANNEL_2CH, 4},
{30, 12, 2880, 0, 0, gScenarioBufferData201_MERGE_CHANNEL_2CH, 48, gScenarioKernelData201_MERGE_CHANNEL_2CH, 4},
{32, 12, 3072, 0, 0, gScenarioBufferData202_MERGE_CHANNEL_2CH, 48, gScenarioKernelData202_MERGE_CHANNEL_2CH, 4},
{36, 12, 3456, 0, 0, gScenarioBufferData203_MERGE_CHANNEL_2CH, 48, gScenarioKernelData203_MERGE_CHANNEL_2CH, 4},
{40, 12, 3840, 0, 0, gScenarioBufferData204_MERGE_CHANNEL_2CH, 48, gScenarioKernelData204_MERGE_CHANNEL_2CH, 4},
{2, 14, 224, 0, 0, gScenarioBufferData205_MERGE_CHANNEL_2CH, 48, gScenarioKernelData205_MERGE_CHANNEL_2CH, 4},
{4, 14, 448, 0, 0, gScenarioBufferData206_MERGE_CHANNEL_2CH, 48, gScenarioKernelData206_MERGE_CHANNEL_2CH, 4},
{6, 14, 672, 0, 0, gScenarioBufferData207_MERGE_CHANNEL_2CH, 48, gScenarioKernelData207_MERGE_CHANNEL_2CH, 4},
{8, 14, 896, 0, 0, gScenarioBufferData208_MERGE_CHANNEL_2CH, 48, gScenarioKernelData208_MERGE_CHANNEL_2CH, 4},
{10, 14, 1120, 0, 0, gScenarioBufferData209_MERGE_CHANNEL_2CH, 48, gScenarioKernelData209_MERGE_CHANNEL_2CH, 4},
{12, 14, 1344, 0, 0, gScenarioBufferData210_MERGE_CHANNEL_2CH, 48, gScenarioKernelData210_MERGE_CHANNEL_2CH, 4},
{14, 14, 1568, 0, 0, gScenarioBufferData211_MERGE_CHANNEL_2CH, 48, gScenarioKernelData211_MERGE_CHANNEL_2CH, 4},
{16, 14, 1792, 0, 0, gScenarioBufferData212_MERGE_CHANNEL_2CH, 48, gScenarioKernelData212_MERGE_CHANNEL_2CH, 4},
{18, 14, 2016, 0, 0, gScenarioBufferData213_MERGE_CHANNEL_2CH, 48, gScenarioKernelData213_MERGE_CHANNEL_2CH, 4},
{20, 14, 2240, 0, 0, gScenarioBufferData214_MERGE_CHANNEL_2CH, 48, gScenarioKernelData214_MERGE_CHANNEL_2CH, 4},
{22, 14, 2464, 0, 0, gScenarioBufferData215_MERGE_CHANNEL_2CH, 48, gScenarioKernelData215_MERGE_CHANNEL_2CH, 4},
{24, 14, 2688, 0, 0, gScenarioBufferData216_MERGE_CHANNEL_2CH, 48, gScenarioKernelData216_MERGE_CHANNEL_2CH, 4},
{26, 14, 2912, 0, 0, gScenarioBufferData217_MERGE_CHANNEL_2CH, 48, gScenarioKernelData217_MERGE_CHANNEL_2CH, 4},
{28, 14, 3136, 0, 0, gScenarioBufferData218_MERGE_CHANNEL_2CH, 48, gScenarioKernelData218_MERGE_CHANNEL_2CH, 4},
{30, 14, 3360, 0, 0, gScenarioBufferData219_MERGE_CHANNEL_2CH, 48, gScenarioKernelData219_MERGE_CHANNEL_2CH, 4},
{32, 14, 3584, 0, 0, gScenarioBufferData220_MERGE_CHANNEL_2CH, 48, gScenarioKernelData220_MERGE_CHANNEL_2CH, 4},
{2, 16, 256, 0, 0, gScenarioBufferData221_MERGE_CHANNEL_2CH, 48, gScenarioKernelData221_MERGE_CHANNEL_2CH, 4},
{4, 16, 512, 0, 0, gScenarioBufferData222_MERGE_CHANNEL_2CH, 48, gScenarioKernelData222_MERGE_CHANNEL_2CH, 4},
{6, 16, 768, 0, 0, gScenarioBufferData223_MERGE_CHANNEL_2CH, 48, gScenarioKernelData223_MERGE_CHANNEL_2CH, 4},
{8, 16, 1024, 0, 0, gScenarioBufferData224_MERGE_CHANNEL_2CH, 48, gScenarioKernelData224_MERGE_CHANNEL_2CH, 4},
{10, 16, 1280, 0, 0, gScenarioBufferData225_MERGE_CHANNEL_2CH, 48, gScenarioKernelData225_MERGE_CHANNEL_2CH, 4},
{12, 16, 1536, 0, 0, gScenarioBufferData226_MERGE_CHANNEL_2CH, 48, gScenarioKernelData226_MERGE_CHANNEL_2CH, 4},
{14, 16, 1792, 0, 0, gScenarioBufferData227_MERGE_CHANNEL_2CH, 48, gScenarioKernelData227_MERGE_CHANNEL_2CH, 4},
{16, 16, 2048, 0, 0, gScenarioBufferData228_MERGE_CHANNEL_2CH, 48, gScenarioKernelData228_MERGE_CHANNEL_2CH, 4},
{18, 16, 2304, 0, 0, gScenarioBufferData229_MERGE_CHANNEL_2CH, 48, gScenarioKernelData229_MERGE_CHANNEL_2CH, 4},
{20, 16, 2560, 0, 0, gScenarioBufferData230_MERGE_CHANNEL_2CH, 48, gScenarioKernelData230_MERGE_CHANNEL_2CH, 4},
{22, 16, 2816, 0, 0, gScenarioBufferData231_MERGE_CHANNEL_2CH, 48, gScenarioKernelData231_MERGE_CHANNEL_2CH, 4},
{24, 16, 3072, 0, 0, gScenarioBufferData232_MERGE_CHANNEL_2CH, 48, gScenarioKernelData232_MERGE_CHANNEL_2CH, 4},
{26, 16, 3328, 0, 0, gScenarioBufferData233_MERGE_CHANNEL_2CH, 48, gScenarioKernelData233_MERGE_CHANNEL_2CH, 4},
{28, 16, 3584, 0, 0, gScenarioBufferData234_MERGE_CHANNEL_2CH, 48, gScenarioKernelData234_MERGE_CHANNEL_2CH, 4},
{30, 16, 3840, 0, 0, gScenarioBufferData235_MERGE_CHANNEL_2CH, 48, gScenarioKernelData235_MERGE_CHANNEL_2CH, 4},
{2, 18, 288, 0, 0, gScenarioBufferData236_MERGE_CHANNEL_2CH, 48, gScenarioKernelData236_MERGE_CHANNEL_2CH, 4},
{4, 18, 576, 0, 0, gScenarioBufferData237_MERGE_CHANNEL_2CH, 48, gScenarioKernelData237_MERGE_CHANNEL_2CH, 4},
{6, 18, 864, 0, 0, gScenarioBufferData238_MERGE_CHANNEL_2CH, 48, gScenarioKernelData238_MERGE_CHANNEL_2CH, 4},
{8, 18, 1152, 0, 0, gScenarioBufferData239_MERGE_CHANNEL_2CH, 48, gScenarioKernelData239_MERGE_CHANNEL_2CH, 4},
{10, 18, 1440, 0, 0, gScenarioBufferData240_MERGE_CHANNEL_2CH, 48, gScenarioKernelData240_MERGE_CHANNEL_2CH, 4},
{12, 18, 1728, 0, 0, gScenarioBufferData241_MERGE_CHANNEL_2CH, 48, gScenarioKernelData241_MERGE_CHANNEL_2CH, 4},
{14, 18, 2016, 0, 0, gScenarioBufferData242_MERGE_CHANNEL_2CH, 48, gScenarioKernelData242_MERGE_CHANNEL_2CH, 4},
{16, 18, 2304, 0, 0, gScenarioBufferData243_MERGE_CHANNEL_2CH, 48, gScenarioKernelData243_MERGE_CHANNEL_2CH, 4},
{18, 18, 2592, 0, 0, gScenarioBufferData244_MERGE_CHANNEL_2CH, 48, gScenarioKernelData244_MERGE_CHANNEL_2CH, 4},
{20, 18, 2880, 0, 0, gScenarioBufferData245_MERGE_CHANNEL_2CH, 48, gScenarioKernelData245_MERGE_CHANNEL_2CH, 4},
{22, 18, 3168, 0, 0, gScenarioBufferData246_MERGE_CHANNEL_2CH, 48, gScenarioKernelData246_MERGE_CHANNEL_2CH, 4},
{24, 18, 3456, 0, 0, gScenarioBufferData247_MERGE_CHANNEL_2CH, 48, gScenarioKernelData247_MERGE_CHANNEL_2CH, 4},
{26, 18, 3744, 0, 0, gScenarioBufferData248_MERGE_CHANNEL_2CH, 48, gScenarioKernelData248_MERGE_CHANNEL_2CH, 4},
{2, 20, 320, 0, 0, gScenarioBufferData249_MERGE_CHANNEL_2CH, 48, gScenarioKernelData249_MERGE_CHANNEL_2CH, 4},
{4, 20, 640, 0, 0, gScenarioBufferData250_MERGE_CHANNEL_2CH, 48, gScenarioKernelData250_MERGE_CHANNEL_2CH, 4},
{6, 20, 960, 0, 0, gScenarioBufferData251_MERGE_CHANNEL_2CH, 48, gScenarioKernelData251_MERGE_CHANNEL_2CH, 4},
{8, 20, 1280, 0, 0, gScenarioBufferData252_MERGE_CHANNEL_2CH, 48, gScenarioKernelData252_MERGE_CHANNEL_2CH, 4},
{10, 20, 1600, 0, 0, gScenarioBufferData253_MERGE_CHANNEL_2CH, 48, gScenarioKernelData253_MERGE_CHANNEL_2CH, 4},
{12, 20, 1920, 0, 0, gScenarioBufferData254_MERGE_CHANNEL_2CH, 48, gScenarioKernelData254_MERGE_CHANNEL_2CH, 4},
{14, 20, 2240, 0, 0, gScenarioBufferData255_MERGE_CHANNEL_2CH, 48, gScenarioKernelData255_MERGE_CHANNEL_2CH, 4},
{16, 20, 2560, 0, 0, gScenarioBufferData256_MERGE_CHANNEL_2CH, 48, gScenarioKernelData256_MERGE_CHANNEL_2CH, 4},
{18, 20, 2880, 0, 0, gScenarioBufferData257_MERGE_CHANNEL_2CH, 48, gScenarioKernelData257_MERGE_CHANNEL_2CH, 4},
{20, 20, 3200, 0, 0, gScenarioBufferData258_MERGE_CHANNEL_2CH, 48, gScenarioKernelData258_MERGE_CHANNEL_2CH, 4},
{22, 20, 3520, 0, 0, gScenarioBufferData259_MERGE_CHANNEL_2CH, 48, gScenarioKernelData259_MERGE_CHANNEL_2CH, 4},
{24, 20, 3840, 0, 0, gScenarioBufferData260_MERGE_CHANNEL_2CH, 48, gScenarioKernelData260_MERGE_CHANNEL_2CH, 4},
{2, 22, 352, 0, 0, gScenarioBufferData261_MERGE_CHANNEL_2CH, 48, gScenarioKernelData261_MERGE_CHANNEL_2CH, 4},
{4, 22, 704, 0, 0, gScenarioBufferData262_MERGE_CHANNEL_2CH, 48, gScenarioKernelData262_MERGE_CHANNEL_2CH, 4},
{6, 22, 1056, 0, 0, gScenarioBufferData263_MERGE_CHANNEL_2CH, 48, gScenarioKernelData263_MERGE_CHANNEL_2CH, 4},
{8, 22, 1408, 0, 0, gScenarioBufferData264_MERGE_CHANNEL_2CH, 48, gScenarioKernelData264_MERGE_CHANNEL_2CH, 4},
{10, 22, 1760, 0, 0, gScenarioBufferData265_MERGE_CHANNEL_2CH, 48, gScenarioKernelData265_MERGE_CHANNEL_2CH, 4},
{12, 22, 2112, 0, 0, gScenarioBufferData266_MERGE_CHANNEL_2CH, 48, gScenarioKernelData266_MERGE_CHANNEL_2CH, 4},
{14, 22, 2464, 0, 0, gScenarioBufferData267_MERGE_CHANNEL_2CH, 48, gScenarioKernelData267_MERGE_CHANNEL_2CH, 4},
{16, 22, 2816, 0, 0, gScenarioBufferData268_MERGE_CHANNEL_2CH, 48, gScenarioKernelData268_MERGE_CHANNEL_2CH, 4},
{18, 22, 3168, 0, 0, gScenarioBufferData269_MERGE_CHANNEL_2CH, 48, gScenarioKernelData269_MERGE_CHANNEL_2CH, 4},
{20, 22, 3520, 0, 0, gScenarioBufferData270_MERGE_CHANNEL_2CH, 48, gScenarioKernelData270_MERGE_CHANNEL_2CH, 4},
{2, 24, 384, 0, 0, gScenarioBufferData271_MERGE_CHANNEL_2CH, 48, gScenarioKernelData271_MERGE_CHANNEL_2CH, 4},
{4, 24, 768, 0, 0, gScenarioBufferData272_MERGE_CHANNEL_2CH, 48, gScenarioKernelData272_MERGE_CHANNEL_2CH, 4},
{6, 24, 1152, 0, 0, gScenarioBufferData273_MERGE_CHANNEL_2CH, 48, gScenarioKernelData273_MERGE_CHANNEL_2CH, 4},
{8, 24, 1536, 0, 0, gScenarioBufferData274_MERGE_CHANNEL_2CH, 48, gScenarioKernelData274_MERGE_CHANNEL_2CH, 4},
{10, 24, 1920, 0, 0, gScenarioBufferData275_MERGE_CHANNEL_2CH, 48, gScenarioKernelData275_MERGE_CHANNEL_2CH, 4},
{12, 24, 2304, 0, 0, gScenarioBufferData276_MERGE_CHANNEL_2CH, 48, gScenarioKernelData276_MERGE_CHANNEL_2CH, 4},
{14, 24, 2688, 0, 0, gScenarioBufferData277_MERGE_CHANNEL_2CH, 48, gScenarioKernelData277_MERGE_CHANNEL_2CH, 4},
{16, 24, 3072, 0, 0, gScenarioBufferData278_MERGE_CHANNEL_2CH, 48, gScenarioKernelData278_MERGE_CHANNEL_2CH, 4},
{18, 24, 3456, 0, 0, gScenarioBufferData279_MERGE_CHANNEL_2CH, 48, gScenarioKernelData279_MERGE_CHANNEL_2CH, 4},
{20, 24, 3840, 0, 0, gScenarioBufferData280_MERGE_CHANNEL_2CH, 48, gScenarioKernelData280_MERGE_CHANNEL_2CH, 4},
{2, 26, 416, 0, 0, gScenarioBufferData281_MERGE_CHANNEL_2CH, 48, gScenarioKernelData281_MERGE_CHANNEL_2CH, 4},
{4, 26, 832, 0, 0, gScenarioBufferData282_MERGE_CHANNEL_2CH, 48, gScenarioKernelData282_MERGE_CHANNEL_2CH, 4},
{6, 26, 1248, 0, 0, gScenarioBufferData283_MERGE_CHANNEL_2CH, 48, gScenarioKernelData283_MERGE_CHANNEL_2CH, 4},
{8, 26, 1664, 0, 0, gScenarioBufferData284_MERGE_CHANNEL_2CH, 48, gScenarioKernelData284_MERGE_CHANNEL_2CH, 4},
{10, 26, 2080, 0, 0, gScenarioBufferData285_MERGE_CHANNEL_2CH, 48, gScenarioKernelData285_MERGE_CHANNEL_2CH, 4},
{12, 26, 2496, 0, 0, gScenarioBufferData286_MERGE_CHANNEL_2CH, 48, gScenarioKernelData286_MERGE_CHANNEL_2CH, 4},
{14, 26, 2912, 0, 0, gScenarioBufferData287_MERGE_CHANNEL_2CH, 48, gScenarioKernelData287_MERGE_CHANNEL_2CH, 4},
{16, 26, 3328, 0, 0, gScenarioBufferData288_MERGE_CHANNEL_2CH, 48, gScenarioKernelData288_MERGE_CHANNEL_2CH, 4},
{18, 26, 3744, 0, 0, gScenarioBufferData289_MERGE_CHANNEL_2CH, 48, gScenarioKernelData289_MERGE_CHANNEL_2CH, 4},
{2, 28, 448, 0, 0, gScenarioBufferData290_MERGE_CHANNEL_2CH, 48, gScenarioKernelData290_MERGE_CHANNEL_2CH, 4},
{4, 28, 896, 0, 0, gScenarioBufferData291_MERGE_CHANNEL_2CH, 48, gScenarioKernelData291_MERGE_CHANNEL_2CH, 4},
{6, 28, 1344, 0, 0, gScenarioBufferData292_MERGE_CHANNEL_2CH, 48, gScenarioKernelData292_MERGE_CHANNEL_2CH, 4},
{8, 28, 1792, 0, 0, gScenarioBufferData293_MERGE_CHANNEL_2CH, 48, gScenarioKernelData293_MERGE_CHANNEL_2CH, 4},
{10, 28, 2240, 0, 0, gScenarioBufferData294_MERGE_CHANNEL_2CH, 48, gScenarioKernelData294_MERGE_CHANNEL_2CH, 4},
{12, 28, 2688, 0, 0, gScenarioBufferData295_MERGE_CHANNEL_2CH, 48, gScenarioKernelData295_MERGE_CHANNEL_2CH, 4},
{14, 28, 3136, 0, 0, gScenarioBufferData296_MERGE_CHANNEL_2CH, 48, gScenarioKernelData296_MERGE_CHANNEL_2CH, 4},
{16, 28, 3584, 0, 0, gScenarioBufferData297_MERGE_CHANNEL_2CH, 48, gScenarioKernelData297_MERGE_CHANNEL_2CH, 4},
{2, 30, 480, 0, 0, gScenarioBufferData298_MERGE_CHANNEL_2CH, 48, gScenarioKernelData298_MERGE_CHANNEL_2CH, 4},
{4, 30, 960, 0, 0, gScenarioBufferData299_MERGE_CHANNEL_2CH, 48, gScenarioKernelData299_MERGE_CHANNEL_2CH, 4},
{6, 30, 1440, 0, 0, gScenarioBufferData300_MERGE_CHANNEL_2CH, 48, gScenarioKernelData300_MERGE_CHANNEL_2CH, 4},
{8, 30, 1920, 0, 0, gScenarioBufferData301_MERGE_CHANNEL_2CH, 48, gScenarioKernelData301_MERGE_CHANNEL_2CH, 4},
{10, 30, 2400, 0, 0, gScenarioBufferData302_MERGE_CHANNEL_2CH, 48, gScenarioKernelData302_MERGE_CHANNEL_2CH, 4},
{12, 30, 2880, 0, 0, gScenarioBufferData303_MERGE_CHANNEL_2CH, 48, gScenarioKernelData303_MERGE_CHANNEL_2CH, 4},
{14, 30, 3360, 0, 0, gScenarioBufferData304_MERGE_CHANNEL_2CH, 48, gScenarioKernelData304_MERGE_CHANNEL_2CH, 4},
{16, 30, 3840, 0, 0, gScenarioBufferData305_MERGE_CHANNEL_2CH, 48, gScenarioKernelData305_MERGE_CHANNEL_2CH, 4},
{2, 32, 512, 0, 0, gScenarioBufferData306_MERGE_CHANNEL_2CH, 48, gScenarioKernelData306_MERGE_CHANNEL_2CH, 4},
{4, 32, 1024, 0, 0, gScenarioBufferData307_MERGE_CHANNEL_2CH, 48, gScenarioKernelData307_MERGE_CHANNEL_2CH, 4},
{6, 32, 1536, 0, 0, gScenarioBufferData308_MERGE_CHANNEL_2CH, 48, gScenarioKernelData308_MERGE_CHANNEL_2CH, 4},
{8, 32, 2048, 0, 0, gScenarioBufferData309_MERGE_CHANNEL_2CH, 48, gScenarioKernelData309_MERGE_CHANNEL_2CH, 4},
{10, 32, 2560, 0, 0, gScenarioBufferData310_MERGE_CHANNEL_2CH, 48, gScenarioKernelData310_MERGE_CHANNEL_2CH, 4},
{12, 32, 3072, 0, 0, gScenarioBufferData311_MERGE_CHANNEL_2CH, 48, gScenarioKernelData311_MERGE_CHANNEL_2CH, 4},
{14, 32, 3584, 0, 0, gScenarioBufferData312_MERGE_CHANNEL_2CH, 48, gScenarioKernelData312_MERGE_CHANNEL_2CH, 4}
};

static acf_scenario_list gScenarioList_MERGE_CHANNEL_2CH = {
313, //number of scenarios
gScenarioArray_MERGE_CHANNEL_2CH};
//**************************************************************

class MERGE_CHANNEL_2CH : public ACF_Process_APU
{

public:
   MERGE_CHANNEL_2CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MERGE_CHANNEL_2CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MERGE_CHANNEL_2CH",
                                        MERGE_CHANNEL_2CH_LOAD_SEGMENTS,
                                        MERGE_CHANNEL_2CH_LOAD_PMEM, MERGE_CHANNEL_2CH_LOAD_PMEM_SIZE,
                                        MERGE_CHANNEL_2CH_LOAD_DMEM, MERGE_CHANNEL_2CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MERGE_CHANNEL_2CH_APEX_LOG_BUFFER, MERGE_CHANNEL_2CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 2, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MERGE_CHANNEL_2CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MERGE_CHANNEL_2CH

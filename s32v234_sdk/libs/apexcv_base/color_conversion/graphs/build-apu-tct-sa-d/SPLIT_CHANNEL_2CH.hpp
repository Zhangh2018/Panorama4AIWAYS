#ifndef _ACF_PROCESS_APU_SPLIT_CHANNEL_2CH
#define _ACF_PROCESS_APU_SPLIT_CHANNEL_2CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <SPLIT_CHANNEL_2CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_SPLIT_CHANNEL_2CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_SPLIT_CHANNEL_2CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_SPLIT_CHANNEL_2CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_SPLIT_CHANNEL_2CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_SPLIT_CHANNEL_2CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_SPLIT_CHANNEL_2CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_SPLIT_CHANNEL_2CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_SPLIT_CHANNEL_2CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_SPLIT_CHANNEL_2CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_SPLIT_CHANNEL_2CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_SPLIT_CHANNEL_2CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_SPLIT_CHANNEL_2CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_SPLIT_CHANNEL_2CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_SPLIT_CHANNEL_2CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_SPLIT_CHANNEL_2CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_SPLIT_CHANNEL_2CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_SPLIT_CHANNEL_2CH[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_SPLIT_CHANNEL_2CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_SPLIT_CHANNEL_2CH[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_SPLIT_CHANNEL_2CH[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_SPLIT_CHANNEL_2CH[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_SPLIT_CHANNEL_2CH[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_SPLIT_CHANNEL_2CH[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_SPLIT_CHANNEL_2CH[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_SPLIT_CHANNEL_2CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_SPLIT_CHANNEL_2CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_SPLIT_CHANNEL_2CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_SPLIT_CHANNEL_2CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_SPLIT_CHANNEL_2CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_SPLIT_CHANNEL_2CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_SPLIT_CHANNEL_2CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_SPLIT_CHANNEL_2CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_SPLIT_CHANNEL_2CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_SPLIT_CHANNEL_2CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_SPLIT_CHANNEL_2CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_SPLIT_CHANNEL_2CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_SPLIT_CHANNEL_2CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_SPLIT_CHANNEL_2CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_SPLIT_CHANNEL_2CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_SPLIT_CHANNEL_2CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_SPLIT_CHANNEL_2CH[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_SPLIT_CHANNEL_2CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_SPLIT_CHANNEL_2CH[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_SPLIT_CHANNEL_2CH[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_SPLIT_CHANNEL_2CH[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_SPLIT_CHANNEL_2CH[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_SPLIT_CHANNEL_2CH[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_SPLIT_CHANNEL_2CH[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_SPLIT_CHANNEL_2CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_SPLIT_CHANNEL_2CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_SPLIT_CHANNEL_2CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_SPLIT_CHANNEL_2CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_SPLIT_CHANNEL_2CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_SPLIT_CHANNEL_2CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_SPLIT_CHANNEL_2CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_SPLIT_CHANNEL_2CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_SPLIT_CHANNEL_2CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_SPLIT_CHANNEL_2CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_SPLIT_CHANNEL_2CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_SPLIT_CHANNEL_2CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_SPLIT_CHANNEL_2CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_SPLIT_CHANNEL_2CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_SPLIT_CHANNEL_2CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_SPLIT_CHANNEL_2CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_SPLIT_CHANNEL_2CH[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_SPLIT_CHANNEL_2CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_SPLIT_CHANNEL_2CH[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_SPLIT_CHANNEL_2CH[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_SPLIT_CHANNEL_2CH[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_SPLIT_CHANNEL_2CH[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_SPLIT_CHANNEL_2CH[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_SPLIT_CHANNEL_2CH[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_SPLIT_CHANNEL_2CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_SPLIT_CHANNEL_2CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_SPLIT_CHANNEL_2CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_SPLIT_CHANNEL_2CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_SPLIT_CHANNEL_2CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_SPLIT_CHANNEL_2CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_SPLIT_CHANNEL_2CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_SPLIT_CHANNEL_2CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_SPLIT_CHANNEL_2CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_SPLIT_CHANNEL_2CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_SPLIT_CHANNEL_2CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_SPLIT_CHANNEL_2CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_SPLIT_CHANNEL_2CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_SPLIT_CHANNEL_2CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_SPLIT_CHANNEL_2CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_SPLIT_CHANNEL_2CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_SPLIT_CHANNEL_2CH[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_SPLIT_CHANNEL_2CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_SPLIT_CHANNEL_2CH[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_SPLIT_CHANNEL_2CH[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_SPLIT_CHANNEL_2CH[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_SPLIT_CHANNEL_2CH[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_SPLIT_CHANNEL_2CH[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_SPLIT_CHANNEL_2CH[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_SPLIT_CHANNEL_2CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_SPLIT_CHANNEL_2CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_SPLIT_CHANNEL_2CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_SPLIT_CHANNEL_2CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_SPLIT_CHANNEL_2CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_SPLIT_CHANNEL_2CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_SPLIT_CHANNEL_2CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_SPLIT_CHANNEL_2CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_SPLIT_CHANNEL_2CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_SPLIT_CHANNEL_2CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_SPLIT_CHANNEL_2CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_SPLIT_CHANNEL_2CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_SPLIT_CHANNEL_2CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_SPLIT_CHANNEL_2CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_SPLIT_CHANNEL_2CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_SPLIT_CHANNEL_2CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_SPLIT_CHANNEL_2CH[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_SPLIT_CHANNEL_2CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_SPLIT_CHANNEL_2CH[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_SPLIT_CHANNEL_2CH[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_SPLIT_CHANNEL_2CH[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_SPLIT_CHANNEL_2CH[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_SPLIT_CHANNEL_2CH[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_SPLIT_CHANNEL_2CH[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_SPLIT_CHANNEL_2CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_SPLIT_CHANNEL_2CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_SPLIT_CHANNEL_2CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_SPLIT_CHANNEL_2CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_SPLIT_CHANNEL_2CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_SPLIT_CHANNEL_2CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_SPLIT_CHANNEL_2CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_SPLIT_CHANNEL_2CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_SPLIT_CHANNEL_2CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_SPLIT_CHANNEL_2CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_SPLIT_CHANNEL_2CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_SPLIT_CHANNEL_2CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_SPLIT_CHANNEL_2CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_SPLIT_CHANNEL_2CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_SPLIT_CHANNEL_2CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_SPLIT_CHANNEL_2CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_SPLIT_CHANNEL_2CH[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_SPLIT_CHANNEL_2CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_SPLIT_CHANNEL_2CH[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_SPLIT_CHANNEL_2CH[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_SPLIT_CHANNEL_2CH[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_SPLIT_CHANNEL_2CH[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_SPLIT_CHANNEL_2CH[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_SPLIT_CHANNEL_2CH[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_SPLIT_CHANNEL_2CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_SPLIT_CHANNEL_2CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_SPLIT_CHANNEL_2CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_SPLIT_CHANNEL_2CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_SPLIT_CHANNEL_2CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_SPLIT_CHANNEL_2CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_SPLIT_CHANNEL_2CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_SPLIT_CHANNEL_2CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_SPLIT_CHANNEL_2CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_SPLIT_CHANNEL_2CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_SPLIT_CHANNEL_2CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_SPLIT_CHANNEL_2CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_SPLIT_CHANNEL_2CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_SPLIT_CHANNEL_2CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_SPLIT_CHANNEL_2CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_SPLIT_CHANNEL_2CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_SPLIT_CHANNEL_2CH[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_SPLIT_CHANNEL_2CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_SPLIT_CHANNEL_2CH[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_SPLIT_CHANNEL_2CH[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_SPLIT_CHANNEL_2CH[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_SPLIT_CHANNEL_2CH[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_SPLIT_CHANNEL_2CH[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_SPLIT_CHANNEL_2CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_SPLIT_CHANNEL_2CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_SPLIT_CHANNEL_2CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_SPLIT_CHANNEL_2CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_SPLIT_CHANNEL_2CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_SPLIT_CHANNEL_2CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_SPLIT_CHANNEL_2CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_SPLIT_CHANNEL_2CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_SPLIT_CHANNEL_2CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_SPLIT_CHANNEL_2CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_SPLIT_CHANNEL_2CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_SPLIT_CHANNEL_2CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_SPLIT_CHANNEL_2CH[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_SPLIT_CHANNEL_2CH[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_SPLIT_CHANNEL_2CH[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_SPLIT_CHANNEL_2CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_SPLIT_CHANNEL_2CH[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_SPLIT_CHANNEL_2CH[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_SPLIT_CHANNEL_2CH[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_SPLIT_CHANNEL_2CH[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_SPLIT_CHANNEL_2CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_SPLIT_CHANNEL_2CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_SPLIT_CHANNEL_2CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_SPLIT_CHANNEL_2CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_SPLIT_CHANNEL_2CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_SPLIT_CHANNEL_2CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_SPLIT_CHANNEL_2CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_SPLIT_CHANNEL_2CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_SPLIT_CHANNEL_2CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_SPLIT_CHANNEL_2CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_SPLIT_CHANNEL_2CH[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_SPLIT_CHANNEL_2CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_SPLIT_CHANNEL_2CH[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_SPLIT_CHANNEL_2CH[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_SPLIT_CHANNEL_2CH[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_SPLIT_CHANNEL_2CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_SPLIT_CHANNEL_2CH[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_SPLIT_CHANNEL_2CH[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_SPLIT_CHANNEL_2CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_SPLIT_CHANNEL_2CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_SPLIT_CHANNEL_2CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_SPLIT_CHANNEL_2CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_SPLIT_CHANNEL_2CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_SPLIT_CHANNEL_2CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_SPLIT_CHANNEL_2CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_SPLIT_CHANNEL_2CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_SPLIT_CHANNEL_2CH[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_SPLIT_CHANNEL_2CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_SPLIT_CHANNEL_2CH[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_SPLIT_CHANNEL_2CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_SPLIT_CHANNEL_2CH[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_SPLIT_CHANNEL_2CH[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_SPLIT_CHANNEL_2CH[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_SPLIT_CHANNEL_2CH[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_SPLIT_CHANNEL_2CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_SPLIT_CHANNEL_2CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_SPLIT_CHANNEL_2CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_SPLIT_CHANNEL_2CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_SPLIT_CHANNEL_2CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_SPLIT_CHANNEL_2CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_SPLIT_CHANNEL_2CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_SPLIT_CHANNEL_2CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_SPLIT_CHANNEL_2CH[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_SPLIT_CHANNEL_2CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_SPLIT_CHANNEL_2CH[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_SPLIT_CHANNEL_2CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_SPLIT_CHANNEL_2CH[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_SPLIT_CHANNEL_2CH[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_SPLIT_CHANNEL_2CH[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_SPLIT_CHANNEL_2CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_SPLIT_CHANNEL_2CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_SPLIT_CHANNEL_2CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_SPLIT_CHANNEL_2CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_SPLIT_CHANNEL_2CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_SPLIT_CHANNEL_2CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_SPLIT_CHANNEL_2CH[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_SPLIT_CHANNEL_2CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_SPLIT_CHANNEL_2CH[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_SPLIT_CHANNEL_2CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_SPLIT_CHANNEL_2CH[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_SPLIT_CHANNEL_2CH[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_SPLIT_CHANNEL_2CH[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_SPLIT_CHANNEL_2CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_SPLIT_CHANNEL_2CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_SPLIT_CHANNEL_2CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_SPLIT_CHANNEL_2CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_SPLIT_CHANNEL_2CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_SPLIT_CHANNEL_2CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_SPLIT_CHANNEL_2CH[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_SPLIT_CHANNEL_2CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_SPLIT_CHANNEL_2CH[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_SPLIT_CHANNEL_2CH[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_SPLIT_CHANNEL_2CH[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_SPLIT_CHANNEL_2CH[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_SPLIT_CHANNEL_2CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_SPLIT_CHANNEL_2CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_SPLIT_CHANNEL_2CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_SPLIT_CHANNEL_2CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_SPLIT_CHANNEL_2CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_SPLIT_CHANNEL_2CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_SPLIT_CHANNEL_2CH[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_SPLIT_CHANNEL_2CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_SPLIT_CHANNEL_2CH[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_SPLIT_CHANNEL_2CH[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_SPLIT_CHANNEL_2CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_SPLIT_CHANNEL_2CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_SPLIT_CHANNEL_2CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_SPLIT_CHANNEL_2CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_SPLIT_CHANNEL_2CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_SPLIT_CHANNEL_2CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_SPLIT_CHANNEL_2CH[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_SPLIT_CHANNEL_2CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_SPLIT_CHANNEL_2CH[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_SPLIT_CHANNEL_2CH[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_SPLIT_CHANNEL_2CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_SPLIT_CHANNEL_2CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_SPLIT_CHANNEL_2CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_SPLIT_CHANNEL_2CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_SPLIT_CHANNEL_2CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_SPLIT_CHANNEL_2CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_SPLIT_CHANNEL_2CH[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_SPLIT_CHANNEL_2CH[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_SPLIT_CHANNEL_2CH[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_SPLIT_CHANNEL_2CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_SPLIT_CHANNEL_2CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_SPLIT_CHANNEL_2CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_SPLIT_CHANNEL_2CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_SPLIT_CHANNEL_2CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_SPLIT_CHANNEL_2CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_SPLIT_CHANNEL_2CH[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_SPLIT_CHANNEL_2CH[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_SPLIT_CHANNEL_2CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_SPLIT_CHANNEL_2CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_SPLIT_CHANNEL_2CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_SPLIT_CHANNEL_2CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_SPLIT_CHANNEL_2CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_SPLIT_CHANNEL_2CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_SPLIT_CHANNEL_2CH[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_SPLIT_CHANNEL_2CH[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_SPLIT_CHANNEL_2CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_SPLIT_CHANNEL_2CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_SPLIT_CHANNEL_2CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_SPLIT_CHANNEL_2CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_SPLIT_CHANNEL_2CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_SPLIT_CHANNEL_2CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_SPLIT_CHANNEL_2CH[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_SPLIT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_SPLIT_CHANNEL_2CH[] = {{0, 0}};

static acf_scenario gScenarioArray_SPLIT_CHANNEL_2CH[] = {
{2, 1, 16, 0, 0, gScenarioBufferData0_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData0_SPLIT_CHANNEL_2CH, 4},
{4, 1, 32, 0, 0, gScenarioBufferData1_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData1_SPLIT_CHANNEL_2CH, 4},
{6, 1, 48, 0, 0, gScenarioBufferData2_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData2_SPLIT_CHANNEL_2CH, 4},
{8, 1, 64, 0, 0, gScenarioBufferData3_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData3_SPLIT_CHANNEL_2CH, 4},
{10, 1, 80, 0, 0, gScenarioBufferData4_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData4_SPLIT_CHANNEL_2CH, 4},
{12, 1, 96, 0, 0, gScenarioBufferData5_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData5_SPLIT_CHANNEL_2CH, 4},
{14, 1, 112, 0, 0, gScenarioBufferData6_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData6_SPLIT_CHANNEL_2CH, 4},
{16, 1, 128, 0, 0, gScenarioBufferData7_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData7_SPLIT_CHANNEL_2CH, 4},
{18, 1, 144, 0, 0, gScenarioBufferData8_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData8_SPLIT_CHANNEL_2CH, 4},
{20, 1, 160, 0, 0, gScenarioBufferData9_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData9_SPLIT_CHANNEL_2CH, 4},
{22, 1, 176, 0, 0, gScenarioBufferData10_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData10_SPLIT_CHANNEL_2CH, 4},
{24, 1, 192, 0, 0, gScenarioBufferData11_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData11_SPLIT_CHANNEL_2CH, 4},
{26, 1, 208, 0, 0, gScenarioBufferData12_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData12_SPLIT_CHANNEL_2CH, 4},
{28, 1, 224, 0, 0, gScenarioBufferData13_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData13_SPLIT_CHANNEL_2CH, 4},
{30, 1, 240, 0, 0, gScenarioBufferData14_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData14_SPLIT_CHANNEL_2CH, 4},
{32, 1, 256, 0, 0, gScenarioBufferData15_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData15_SPLIT_CHANNEL_2CH, 4},
{36, 1, 288, 0, 0, gScenarioBufferData16_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData16_SPLIT_CHANNEL_2CH, 4},
{40, 1, 320, 0, 0, gScenarioBufferData17_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData17_SPLIT_CHANNEL_2CH, 4},
{44, 1, 352, 0, 0, gScenarioBufferData18_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData18_SPLIT_CHANNEL_2CH, 4},
{48, 1, 384, 0, 0, gScenarioBufferData19_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData19_SPLIT_CHANNEL_2CH, 4},
{52, 1, 416, 0, 0, gScenarioBufferData20_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData20_SPLIT_CHANNEL_2CH, 4},
{56, 1, 448, 0, 0, gScenarioBufferData21_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData21_SPLIT_CHANNEL_2CH, 4},
{60, 1, 480, 0, 0, gScenarioBufferData22_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData22_SPLIT_CHANNEL_2CH, 4},
{64, 1, 512, 0, 0, gScenarioBufferData23_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData23_SPLIT_CHANNEL_2CH, 4},
{2, 2, 32, 0, 0, gScenarioBufferData24_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData24_SPLIT_CHANNEL_2CH, 4},
{4, 2, 64, 0, 0, gScenarioBufferData25_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData25_SPLIT_CHANNEL_2CH, 4},
{6, 2, 96, 0, 0, gScenarioBufferData26_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData26_SPLIT_CHANNEL_2CH, 4},
{8, 2, 128, 0, 0, gScenarioBufferData27_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData27_SPLIT_CHANNEL_2CH, 4},
{10, 2, 160, 0, 0, gScenarioBufferData28_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData28_SPLIT_CHANNEL_2CH, 4},
{12, 2, 192, 0, 0, gScenarioBufferData29_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData29_SPLIT_CHANNEL_2CH, 4},
{14, 2, 224, 0, 0, gScenarioBufferData30_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData30_SPLIT_CHANNEL_2CH, 4},
{16, 2, 256, 0, 0, gScenarioBufferData31_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData31_SPLIT_CHANNEL_2CH, 4},
{18, 2, 288, 0, 0, gScenarioBufferData32_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData32_SPLIT_CHANNEL_2CH, 4},
{20, 2, 320, 0, 0, gScenarioBufferData33_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData33_SPLIT_CHANNEL_2CH, 4},
{22, 2, 352, 0, 0, gScenarioBufferData34_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData34_SPLIT_CHANNEL_2CH, 4},
{24, 2, 384, 0, 0, gScenarioBufferData35_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData35_SPLIT_CHANNEL_2CH, 4},
{26, 2, 416, 0, 0, gScenarioBufferData36_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData36_SPLIT_CHANNEL_2CH, 4},
{28, 2, 448, 0, 0, gScenarioBufferData37_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData37_SPLIT_CHANNEL_2CH, 4},
{30, 2, 480, 0, 0, gScenarioBufferData38_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData38_SPLIT_CHANNEL_2CH, 4},
{32, 2, 512, 0, 0, gScenarioBufferData39_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData39_SPLIT_CHANNEL_2CH, 4},
{36, 2, 576, 0, 0, gScenarioBufferData40_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData40_SPLIT_CHANNEL_2CH, 4},
{40, 2, 640, 0, 0, gScenarioBufferData41_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData41_SPLIT_CHANNEL_2CH, 4},
{44, 2, 704, 0, 0, gScenarioBufferData42_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData42_SPLIT_CHANNEL_2CH, 4},
{48, 2, 768, 0, 0, gScenarioBufferData43_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData43_SPLIT_CHANNEL_2CH, 4},
{52, 2, 832, 0, 0, gScenarioBufferData44_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData44_SPLIT_CHANNEL_2CH, 4},
{56, 2, 896, 0, 0, gScenarioBufferData45_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData45_SPLIT_CHANNEL_2CH, 4},
{60, 2, 960, 0, 0, gScenarioBufferData46_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData46_SPLIT_CHANNEL_2CH, 4},
{64, 2, 1024, 0, 0, gScenarioBufferData47_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData47_SPLIT_CHANNEL_2CH, 4},
{2, 3, 48, 0, 0, gScenarioBufferData48_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData48_SPLIT_CHANNEL_2CH, 4},
{4, 3, 96, 0, 0, gScenarioBufferData49_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData49_SPLIT_CHANNEL_2CH, 4},
{6, 3, 144, 0, 0, gScenarioBufferData50_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData50_SPLIT_CHANNEL_2CH, 4},
{8, 3, 192, 0, 0, gScenarioBufferData51_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData51_SPLIT_CHANNEL_2CH, 4},
{10, 3, 240, 0, 0, gScenarioBufferData52_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData52_SPLIT_CHANNEL_2CH, 4},
{12, 3, 288, 0, 0, gScenarioBufferData53_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData53_SPLIT_CHANNEL_2CH, 4},
{14, 3, 336, 0, 0, gScenarioBufferData54_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData54_SPLIT_CHANNEL_2CH, 4},
{16, 3, 384, 0, 0, gScenarioBufferData55_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData55_SPLIT_CHANNEL_2CH, 4},
{18, 3, 432, 0, 0, gScenarioBufferData56_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData56_SPLIT_CHANNEL_2CH, 4},
{20, 3, 480, 0, 0, gScenarioBufferData57_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData57_SPLIT_CHANNEL_2CH, 4},
{22, 3, 528, 0, 0, gScenarioBufferData58_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData58_SPLIT_CHANNEL_2CH, 4},
{24, 3, 576, 0, 0, gScenarioBufferData59_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData59_SPLIT_CHANNEL_2CH, 4},
{26, 3, 624, 0, 0, gScenarioBufferData60_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData60_SPLIT_CHANNEL_2CH, 4},
{28, 3, 672, 0, 0, gScenarioBufferData61_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData61_SPLIT_CHANNEL_2CH, 4},
{30, 3, 720, 0, 0, gScenarioBufferData62_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData62_SPLIT_CHANNEL_2CH, 4},
{32, 3, 768, 0, 0, gScenarioBufferData63_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData63_SPLIT_CHANNEL_2CH, 4},
{36, 3, 864, 0, 0, gScenarioBufferData64_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData64_SPLIT_CHANNEL_2CH, 4},
{40, 3, 960, 0, 0, gScenarioBufferData65_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData65_SPLIT_CHANNEL_2CH, 4},
{44, 3, 1056, 0, 0, gScenarioBufferData66_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData66_SPLIT_CHANNEL_2CH, 4},
{48, 3, 1152, 0, 0, gScenarioBufferData67_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData67_SPLIT_CHANNEL_2CH, 4},
{52, 3, 1248, 0, 0, gScenarioBufferData68_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData68_SPLIT_CHANNEL_2CH, 4},
{56, 3, 1344, 0, 0, gScenarioBufferData69_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData69_SPLIT_CHANNEL_2CH, 4},
{60, 3, 1440, 0, 0, gScenarioBufferData70_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData70_SPLIT_CHANNEL_2CH, 4},
{64, 3, 1536, 0, 0, gScenarioBufferData71_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData71_SPLIT_CHANNEL_2CH, 4},
{2, 4, 64, 0, 0, gScenarioBufferData72_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData72_SPLIT_CHANNEL_2CH, 4},
{4, 4, 128, 0, 0, gScenarioBufferData73_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData73_SPLIT_CHANNEL_2CH, 4},
{6, 4, 192, 0, 0, gScenarioBufferData74_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData74_SPLIT_CHANNEL_2CH, 4},
{8, 4, 256, 0, 0, gScenarioBufferData75_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData75_SPLIT_CHANNEL_2CH, 4},
{10, 4, 320, 0, 0, gScenarioBufferData76_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData76_SPLIT_CHANNEL_2CH, 4},
{12, 4, 384, 0, 0, gScenarioBufferData77_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData77_SPLIT_CHANNEL_2CH, 4},
{14, 4, 448, 0, 0, gScenarioBufferData78_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData78_SPLIT_CHANNEL_2CH, 4},
{16, 4, 512, 0, 0, gScenarioBufferData79_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData79_SPLIT_CHANNEL_2CH, 4},
{18, 4, 576, 0, 0, gScenarioBufferData80_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData80_SPLIT_CHANNEL_2CH, 4},
{20, 4, 640, 0, 0, gScenarioBufferData81_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData81_SPLIT_CHANNEL_2CH, 4},
{22, 4, 704, 0, 0, gScenarioBufferData82_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData82_SPLIT_CHANNEL_2CH, 4},
{24, 4, 768, 0, 0, gScenarioBufferData83_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData83_SPLIT_CHANNEL_2CH, 4},
{26, 4, 832, 0, 0, gScenarioBufferData84_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData84_SPLIT_CHANNEL_2CH, 4},
{28, 4, 896, 0, 0, gScenarioBufferData85_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData85_SPLIT_CHANNEL_2CH, 4},
{30, 4, 960, 0, 0, gScenarioBufferData86_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData86_SPLIT_CHANNEL_2CH, 4},
{32, 4, 1024, 0, 0, gScenarioBufferData87_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData87_SPLIT_CHANNEL_2CH, 4},
{36, 4, 1152, 0, 0, gScenarioBufferData88_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData88_SPLIT_CHANNEL_2CH, 4},
{40, 4, 1280, 0, 0, gScenarioBufferData89_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData89_SPLIT_CHANNEL_2CH, 4},
{44, 4, 1408, 0, 0, gScenarioBufferData90_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData90_SPLIT_CHANNEL_2CH, 4},
{48, 4, 1536, 0, 0, gScenarioBufferData91_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData91_SPLIT_CHANNEL_2CH, 4},
{52, 4, 1664, 0, 0, gScenarioBufferData92_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData92_SPLIT_CHANNEL_2CH, 4},
{56, 4, 1792, 0, 0, gScenarioBufferData93_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData93_SPLIT_CHANNEL_2CH, 4},
{60, 4, 1920, 0, 0, gScenarioBufferData94_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData94_SPLIT_CHANNEL_2CH, 4},
{64, 4, 2048, 0, 0, gScenarioBufferData95_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData95_SPLIT_CHANNEL_2CH, 4},
{2, 5, 80, 0, 0, gScenarioBufferData96_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData96_SPLIT_CHANNEL_2CH, 4},
{4, 5, 160, 0, 0, gScenarioBufferData97_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData97_SPLIT_CHANNEL_2CH, 4},
{6, 5, 240, 0, 0, gScenarioBufferData98_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData98_SPLIT_CHANNEL_2CH, 4},
{8, 5, 320, 0, 0, gScenarioBufferData99_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData99_SPLIT_CHANNEL_2CH, 4},
{10, 5, 400, 0, 0, gScenarioBufferData100_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData100_SPLIT_CHANNEL_2CH, 4},
{12, 5, 480, 0, 0, gScenarioBufferData101_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData101_SPLIT_CHANNEL_2CH, 4},
{14, 5, 560, 0, 0, gScenarioBufferData102_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData102_SPLIT_CHANNEL_2CH, 4},
{16, 5, 640, 0, 0, gScenarioBufferData103_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData103_SPLIT_CHANNEL_2CH, 4},
{18, 5, 720, 0, 0, gScenarioBufferData104_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData104_SPLIT_CHANNEL_2CH, 4},
{20, 5, 800, 0, 0, gScenarioBufferData105_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData105_SPLIT_CHANNEL_2CH, 4},
{22, 5, 880, 0, 0, gScenarioBufferData106_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData106_SPLIT_CHANNEL_2CH, 4},
{24, 5, 960, 0, 0, gScenarioBufferData107_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData107_SPLIT_CHANNEL_2CH, 4},
{26, 5, 1040, 0, 0, gScenarioBufferData108_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData108_SPLIT_CHANNEL_2CH, 4},
{28, 5, 1120, 0, 0, gScenarioBufferData109_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData109_SPLIT_CHANNEL_2CH, 4},
{30, 5, 1200, 0, 0, gScenarioBufferData110_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData110_SPLIT_CHANNEL_2CH, 4},
{32, 5, 1280, 0, 0, gScenarioBufferData111_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData111_SPLIT_CHANNEL_2CH, 4},
{36, 5, 1440, 0, 0, gScenarioBufferData112_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData112_SPLIT_CHANNEL_2CH, 4},
{40, 5, 1600, 0, 0, gScenarioBufferData113_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData113_SPLIT_CHANNEL_2CH, 4},
{44, 5, 1760, 0, 0, gScenarioBufferData114_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData114_SPLIT_CHANNEL_2CH, 4},
{48, 5, 1920, 0, 0, gScenarioBufferData115_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData115_SPLIT_CHANNEL_2CH, 4},
{52, 5, 2080, 0, 0, gScenarioBufferData116_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData116_SPLIT_CHANNEL_2CH, 4},
{56, 5, 2240, 0, 0, gScenarioBufferData117_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData117_SPLIT_CHANNEL_2CH, 4},
{60, 5, 2400, 0, 0, gScenarioBufferData118_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData118_SPLIT_CHANNEL_2CH, 4},
{64, 5, 2560, 0, 0, gScenarioBufferData119_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData119_SPLIT_CHANNEL_2CH, 4},
{2, 6, 96, 0, 0, gScenarioBufferData120_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData120_SPLIT_CHANNEL_2CH, 4},
{4, 6, 192, 0, 0, gScenarioBufferData121_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData121_SPLIT_CHANNEL_2CH, 4},
{6, 6, 288, 0, 0, gScenarioBufferData122_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData122_SPLIT_CHANNEL_2CH, 4},
{8, 6, 384, 0, 0, gScenarioBufferData123_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData123_SPLIT_CHANNEL_2CH, 4},
{10, 6, 480, 0, 0, gScenarioBufferData124_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData124_SPLIT_CHANNEL_2CH, 4},
{12, 6, 576, 0, 0, gScenarioBufferData125_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData125_SPLIT_CHANNEL_2CH, 4},
{14, 6, 672, 0, 0, gScenarioBufferData126_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData126_SPLIT_CHANNEL_2CH, 4},
{16, 6, 768, 0, 0, gScenarioBufferData127_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData127_SPLIT_CHANNEL_2CH, 4},
{18, 6, 864, 0, 0, gScenarioBufferData128_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData128_SPLIT_CHANNEL_2CH, 4},
{20, 6, 960, 0, 0, gScenarioBufferData129_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData129_SPLIT_CHANNEL_2CH, 4},
{22, 6, 1056, 0, 0, gScenarioBufferData130_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData130_SPLIT_CHANNEL_2CH, 4},
{24, 6, 1152, 0, 0, gScenarioBufferData131_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData131_SPLIT_CHANNEL_2CH, 4},
{26, 6, 1248, 0, 0, gScenarioBufferData132_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData132_SPLIT_CHANNEL_2CH, 4},
{28, 6, 1344, 0, 0, gScenarioBufferData133_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData133_SPLIT_CHANNEL_2CH, 4},
{30, 6, 1440, 0, 0, gScenarioBufferData134_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData134_SPLIT_CHANNEL_2CH, 4},
{32, 6, 1536, 0, 0, gScenarioBufferData135_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData135_SPLIT_CHANNEL_2CH, 4},
{36, 6, 1728, 0, 0, gScenarioBufferData136_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData136_SPLIT_CHANNEL_2CH, 4},
{40, 6, 1920, 0, 0, gScenarioBufferData137_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData137_SPLIT_CHANNEL_2CH, 4},
{44, 6, 2112, 0, 0, gScenarioBufferData138_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData138_SPLIT_CHANNEL_2CH, 4},
{48, 6, 2304, 0, 0, gScenarioBufferData139_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData139_SPLIT_CHANNEL_2CH, 4},
{52, 6, 2496, 0, 0, gScenarioBufferData140_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData140_SPLIT_CHANNEL_2CH, 4},
{56, 6, 2688, 0, 0, gScenarioBufferData141_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData141_SPLIT_CHANNEL_2CH, 4},
{60, 6, 2880, 0, 0, gScenarioBufferData142_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData142_SPLIT_CHANNEL_2CH, 4},
{64, 6, 3072, 0, 0, gScenarioBufferData143_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData143_SPLIT_CHANNEL_2CH, 4},
{2, 8, 128, 0, 0, gScenarioBufferData144_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData144_SPLIT_CHANNEL_2CH, 4},
{4, 8, 256, 0, 0, gScenarioBufferData145_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData145_SPLIT_CHANNEL_2CH, 4},
{6, 8, 384, 0, 0, gScenarioBufferData146_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData146_SPLIT_CHANNEL_2CH, 4},
{8, 8, 512, 0, 0, gScenarioBufferData147_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData147_SPLIT_CHANNEL_2CH, 4},
{10, 8, 640, 0, 0, gScenarioBufferData148_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData148_SPLIT_CHANNEL_2CH, 4},
{12, 8, 768, 0, 0, gScenarioBufferData149_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData149_SPLIT_CHANNEL_2CH, 4},
{14, 8, 896, 0, 0, gScenarioBufferData150_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData150_SPLIT_CHANNEL_2CH, 4},
{16, 8, 1024, 0, 0, gScenarioBufferData151_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData151_SPLIT_CHANNEL_2CH, 4},
{18, 8, 1152, 0, 0, gScenarioBufferData152_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData152_SPLIT_CHANNEL_2CH, 4},
{20, 8, 1280, 0, 0, gScenarioBufferData153_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData153_SPLIT_CHANNEL_2CH, 4},
{22, 8, 1408, 0, 0, gScenarioBufferData154_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData154_SPLIT_CHANNEL_2CH, 4},
{24, 8, 1536, 0, 0, gScenarioBufferData155_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData155_SPLIT_CHANNEL_2CH, 4},
{26, 8, 1664, 0, 0, gScenarioBufferData156_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData156_SPLIT_CHANNEL_2CH, 4},
{28, 8, 1792, 0, 0, gScenarioBufferData157_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData157_SPLIT_CHANNEL_2CH, 4},
{30, 8, 1920, 0, 0, gScenarioBufferData158_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData158_SPLIT_CHANNEL_2CH, 4},
{32, 8, 2048, 0, 0, gScenarioBufferData159_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData159_SPLIT_CHANNEL_2CH, 4},
{36, 8, 2304, 0, 0, gScenarioBufferData160_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData160_SPLIT_CHANNEL_2CH, 4},
{40, 8, 2560, 0, 0, gScenarioBufferData161_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData161_SPLIT_CHANNEL_2CH, 4},
{44, 8, 2816, 0, 0, gScenarioBufferData162_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData162_SPLIT_CHANNEL_2CH, 4},
{48, 8, 3072, 0, 0, gScenarioBufferData163_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData163_SPLIT_CHANNEL_2CH, 4},
{52, 8, 3328, 0, 0, gScenarioBufferData164_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData164_SPLIT_CHANNEL_2CH, 4},
{56, 8, 3584, 0, 0, gScenarioBufferData165_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData165_SPLIT_CHANNEL_2CH, 4},
{60, 8, 3840, 0, 0, gScenarioBufferData166_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData166_SPLIT_CHANNEL_2CH, 4},
{2, 10, 160, 0, 0, gScenarioBufferData167_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData167_SPLIT_CHANNEL_2CH, 4},
{4, 10, 320, 0, 0, gScenarioBufferData168_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData168_SPLIT_CHANNEL_2CH, 4},
{6, 10, 480, 0, 0, gScenarioBufferData169_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData169_SPLIT_CHANNEL_2CH, 4},
{8, 10, 640, 0, 0, gScenarioBufferData170_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData170_SPLIT_CHANNEL_2CH, 4},
{10, 10, 800, 0, 0, gScenarioBufferData171_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData171_SPLIT_CHANNEL_2CH, 4},
{12, 10, 960, 0, 0, gScenarioBufferData172_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData172_SPLIT_CHANNEL_2CH, 4},
{14, 10, 1120, 0, 0, gScenarioBufferData173_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData173_SPLIT_CHANNEL_2CH, 4},
{16, 10, 1280, 0, 0, gScenarioBufferData174_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData174_SPLIT_CHANNEL_2CH, 4},
{18, 10, 1440, 0, 0, gScenarioBufferData175_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData175_SPLIT_CHANNEL_2CH, 4},
{20, 10, 1600, 0, 0, gScenarioBufferData176_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData176_SPLIT_CHANNEL_2CH, 4},
{22, 10, 1760, 0, 0, gScenarioBufferData177_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData177_SPLIT_CHANNEL_2CH, 4},
{24, 10, 1920, 0, 0, gScenarioBufferData178_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData178_SPLIT_CHANNEL_2CH, 4},
{26, 10, 2080, 0, 0, gScenarioBufferData179_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData179_SPLIT_CHANNEL_2CH, 4},
{28, 10, 2240, 0, 0, gScenarioBufferData180_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData180_SPLIT_CHANNEL_2CH, 4},
{30, 10, 2400, 0, 0, gScenarioBufferData181_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData181_SPLIT_CHANNEL_2CH, 4},
{32, 10, 2560, 0, 0, gScenarioBufferData182_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData182_SPLIT_CHANNEL_2CH, 4},
{36, 10, 2880, 0, 0, gScenarioBufferData183_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData183_SPLIT_CHANNEL_2CH, 4},
{40, 10, 3200, 0, 0, gScenarioBufferData184_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData184_SPLIT_CHANNEL_2CH, 4},
{44, 10, 3520, 0, 0, gScenarioBufferData185_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData185_SPLIT_CHANNEL_2CH, 4},
{48, 10, 3840, 0, 0, gScenarioBufferData186_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData186_SPLIT_CHANNEL_2CH, 4},
{2, 12, 192, 0, 0, gScenarioBufferData187_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData187_SPLIT_CHANNEL_2CH, 4},
{4, 12, 384, 0, 0, gScenarioBufferData188_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData188_SPLIT_CHANNEL_2CH, 4},
{6, 12, 576, 0, 0, gScenarioBufferData189_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData189_SPLIT_CHANNEL_2CH, 4},
{8, 12, 768, 0, 0, gScenarioBufferData190_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData190_SPLIT_CHANNEL_2CH, 4},
{10, 12, 960, 0, 0, gScenarioBufferData191_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData191_SPLIT_CHANNEL_2CH, 4},
{12, 12, 1152, 0, 0, gScenarioBufferData192_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData192_SPLIT_CHANNEL_2CH, 4},
{14, 12, 1344, 0, 0, gScenarioBufferData193_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData193_SPLIT_CHANNEL_2CH, 4},
{16, 12, 1536, 0, 0, gScenarioBufferData194_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData194_SPLIT_CHANNEL_2CH, 4},
{18, 12, 1728, 0, 0, gScenarioBufferData195_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData195_SPLIT_CHANNEL_2CH, 4},
{20, 12, 1920, 0, 0, gScenarioBufferData196_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData196_SPLIT_CHANNEL_2CH, 4},
{22, 12, 2112, 0, 0, gScenarioBufferData197_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData197_SPLIT_CHANNEL_2CH, 4},
{24, 12, 2304, 0, 0, gScenarioBufferData198_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData198_SPLIT_CHANNEL_2CH, 4},
{26, 12, 2496, 0, 0, gScenarioBufferData199_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData199_SPLIT_CHANNEL_2CH, 4},
{28, 12, 2688, 0, 0, gScenarioBufferData200_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData200_SPLIT_CHANNEL_2CH, 4},
{30, 12, 2880, 0, 0, gScenarioBufferData201_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData201_SPLIT_CHANNEL_2CH, 4},
{32, 12, 3072, 0, 0, gScenarioBufferData202_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData202_SPLIT_CHANNEL_2CH, 4},
{36, 12, 3456, 0, 0, gScenarioBufferData203_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData203_SPLIT_CHANNEL_2CH, 4},
{40, 12, 3840, 0, 0, gScenarioBufferData204_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData204_SPLIT_CHANNEL_2CH, 4},
{2, 14, 224, 0, 0, gScenarioBufferData205_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData205_SPLIT_CHANNEL_2CH, 4},
{4, 14, 448, 0, 0, gScenarioBufferData206_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData206_SPLIT_CHANNEL_2CH, 4},
{6, 14, 672, 0, 0, gScenarioBufferData207_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData207_SPLIT_CHANNEL_2CH, 4},
{8, 14, 896, 0, 0, gScenarioBufferData208_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData208_SPLIT_CHANNEL_2CH, 4},
{10, 14, 1120, 0, 0, gScenarioBufferData209_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData209_SPLIT_CHANNEL_2CH, 4},
{12, 14, 1344, 0, 0, gScenarioBufferData210_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData210_SPLIT_CHANNEL_2CH, 4},
{14, 14, 1568, 0, 0, gScenarioBufferData211_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData211_SPLIT_CHANNEL_2CH, 4},
{16, 14, 1792, 0, 0, gScenarioBufferData212_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData212_SPLIT_CHANNEL_2CH, 4},
{18, 14, 2016, 0, 0, gScenarioBufferData213_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData213_SPLIT_CHANNEL_2CH, 4},
{20, 14, 2240, 0, 0, gScenarioBufferData214_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData214_SPLIT_CHANNEL_2CH, 4},
{22, 14, 2464, 0, 0, gScenarioBufferData215_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData215_SPLIT_CHANNEL_2CH, 4},
{24, 14, 2688, 0, 0, gScenarioBufferData216_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData216_SPLIT_CHANNEL_2CH, 4},
{26, 14, 2912, 0, 0, gScenarioBufferData217_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData217_SPLIT_CHANNEL_2CH, 4},
{28, 14, 3136, 0, 0, gScenarioBufferData218_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData218_SPLIT_CHANNEL_2CH, 4},
{30, 14, 3360, 0, 0, gScenarioBufferData219_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData219_SPLIT_CHANNEL_2CH, 4},
{32, 14, 3584, 0, 0, gScenarioBufferData220_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData220_SPLIT_CHANNEL_2CH, 4},
{2, 16, 256, 0, 0, gScenarioBufferData221_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData221_SPLIT_CHANNEL_2CH, 4},
{4, 16, 512, 0, 0, gScenarioBufferData222_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData222_SPLIT_CHANNEL_2CH, 4},
{6, 16, 768, 0, 0, gScenarioBufferData223_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData223_SPLIT_CHANNEL_2CH, 4},
{8, 16, 1024, 0, 0, gScenarioBufferData224_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData224_SPLIT_CHANNEL_2CH, 4},
{10, 16, 1280, 0, 0, gScenarioBufferData225_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData225_SPLIT_CHANNEL_2CH, 4},
{12, 16, 1536, 0, 0, gScenarioBufferData226_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData226_SPLIT_CHANNEL_2CH, 4},
{14, 16, 1792, 0, 0, gScenarioBufferData227_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData227_SPLIT_CHANNEL_2CH, 4},
{16, 16, 2048, 0, 0, gScenarioBufferData228_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData228_SPLIT_CHANNEL_2CH, 4},
{18, 16, 2304, 0, 0, gScenarioBufferData229_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData229_SPLIT_CHANNEL_2CH, 4},
{20, 16, 2560, 0, 0, gScenarioBufferData230_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData230_SPLIT_CHANNEL_2CH, 4},
{22, 16, 2816, 0, 0, gScenarioBufferData231_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData231_SPLIT_CHANNEL_2CH, 4},
{24, 16, 3072, 0, 0, gScenarioBufferData232_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData232_SPLIT_CHANNEL_2CH, 4},
{26, 16, 3328, 0, 0, gScenarioBufferData233_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData233_SPLIT_CHANNEL_2CH, 4},
{28, 16, 3584, 0, 0, gScenarioBufferData234_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData234_SPLIT_CHANNEL_2CH, 4},
{30, 16, 3840, 0, 0, gScenarioBufferData235_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData235_SPLIT_CHANNEL_2CH, 4},
{2, 18, 288, 0, 0, gScenarioBufferData236_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData236_SPLIT_CHANNEL_2CH, 4},
{4, 18, 576, 0, 0, gScenarioBufferData237_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData237_SPLIT_CHANNEL_2CH, 4},
{6, 18, 864, 0, 0, gScenarioBufferData238_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData238_SPLIT_CHANNEL_2CH, 4},
{8, 18, 1152, 0, 0, gScenarioBufferData239_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData239_SPLIT_CHANNEL_2CH, 4},
{10, 18, 1440, 0, 0, gScenarioBufferData240_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData240_SPLIT_CHANNEL_2CH, 4},
{12, 18, 1728, 0, 0, gScenarioBufferData241_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData241_SPLIT_CHANNEL_2CH, 4},
{14, 18, 2016, 0, 0, gScenarioBufferData242_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData242_SPLIT_CHANNEL_2CH, 4},
{16, 18, 2304, 0, 0, gScenarioBufferData243_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData243_SPLIT_CHANNEL_2CH, 4},
{18, 18, 2592, 0, 0, gScenarioBufferData244_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData244_SPLIT_CHANNEL_2CH, 4},
{20, 18, 2880, 0, 0, gScenarioBufferData245_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData245_SPLIT_CHANNEL_2CH, 4},
{22, 18, 3168, 0, 0, gScenarioBufferData246_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData246_SPLIT_CHANNEL_2CH, 4},
{24, 18, 3456, 0, 0, gScenarioBufferData247_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData247_SPLIT_CHANNEL_2CH, 4},
{26, 18, 3744, 0, 0, gScenarioBufferData248_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData248_SPLIT_CHANNEL_2CH, 4},
{2, 20, 320, 0, 0, gScenarioBufferData249_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData249_SPLIT_CHANNEL_2CH, 4},
{4, 20, 640, 0, 0, gScenarioBufferData250_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData250_SPLIT_CHANNEL_2CH, 4},
{6, 20, 960, 0, 0, gScenarioBufferData251_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData251_SPLIT_CHANNEL_2CH, 4},
{8, 20, 1280, 0, 0, gScenarioBufferData252_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData252_SPLIT_CHANNEL_2CH, 4},
{10, 20, 1600, 0, 0, gScenarioBufferData253_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData253_SPLIT_CHANNEL_2CH, 4},
{12, 20, 1920, 0, 0, gScenarioBufferData254_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData254_SPLIT_CHANNEL_2CH, 4},
{14, 20, 2240, 0, 0, gScenarioBufferData255_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData255_SPLIT_CHANNEL_2CH, 4},
{16, 20, 2560, 0, 0, gScenarioBufferData256_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData256_SPLIT_CHANNEL_2CH, 4},
{18, 20, 2880, 0, 0, gScenarioBufferData257_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData257_SPLIT_CHANNEL_2CH, 4},
{20, 20, 3200, 0, 0, gScenarioBufferData258_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData258_SPLIT_CHANNEL_2CH, 4},
{22, 20, 3520, 0, 0, gScenarioBufferData259_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData259_SPLIT_CHANNEL_2CH, 4},
{24, 20, 3840, 0, 0, gScenarioBufferData260_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData260_SPLIT_CHANNEL_2CH, 4},
{2, 22, 352, 0, 0, gScenarioBufferData261_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData261_SPLIT_CHANNEL_2CH, 4},
{4, 22, 704, 0, 0, gScenarioBufferData262_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData262_SPLIT_CHANNEL_2CH, 4},
{6, 22, 1056, 0, 0, gScenarioBufferData263_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData263_SPLIT_CHANNEL_2CH, 4},
{8, 22, 1408, 0, 0, gScenarioBufferData264_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData264_SPLIT_CHANNEL_2CH, 4},
{10, 22, 1760, 0, 0, gScenarioBufferData265_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData265_SPLIT_CHANNEL_2CH, 4},
{12, 22, 2112, 0, 0, gScenarioBufferData266_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData266_SPLIT_CHANNEL_2CH, 4},
{14, 22, 2464, 0, 0, gScenarioBufferData267_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData267_SPLIT_CHANNEL_2CH, 4},
{16, 22, 2816, 0, 0, gScenarioBufferData268_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData268_SPLIT_CHANNEL_2CH, 4},
{18, 22, 3168, 0, 0, gScenarioBufferData269_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData269_SPLIT_CHANNEL_2CH, 4},
{20, 22, 3520, 0, 0, gScenarioBufferData270_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData270_SPLIT_CHANNEL_2CH, 4},
{2, 24, 384, 0, 0, gScenarioBufferData271_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData271_SPLIT_CHANNEL_2CH, 4},
{4, 24, 768, 0, 0, gScenarioBufferData272_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData272_SPLIT_CHANNEL_2CH, 4},
{6, 24, 1152, 0, 0, gScenarioBufferData273_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData273_SPLIT_CHANNEL_2CH, 4},
{8, 24, 1536, 0, 0, gScenarioBufferData274_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData274_SPLIT_CHANNEL_2CH, 4},
{10, 24, 1920, 0, 0, gScenarioBufferData275_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData275_SPLIT_CHANNEL_2CH, 4},
{12, 24, 2304, 0, 0, gScenarioBufferData276_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData276_SPLIT_CHANNEL_2CH, 4},
{14, 24, 2688, 0, 0, gScenarioBufferData277_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData277_SPLIT_CHANNEL_2CH, 4},
{16, 24, 3072, 0, 0, gScenarioBufferData278_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData278_SPLIT_CHANNEL_2CH, 4},
{18, 24, 3456, 0, 0, gScenarioBufferData279_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData279_SPLIT_CHANNEL_2CH, 4},
{20, 24, 3840, 0, 0, gScenarioBufferData280_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData280_SPLIT_CHANNEL_2CH, 4},
{2, 26, 416, 0, 0, gScenarioBufferData281_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData281_SPLIT_CHANNEL_2CH, 4},
{4, 26, 832, 0, 0, gScenarioBufferData282_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData282_SPLIT_CHANNEL_2CH, 4},
{6, 26, 1248, 0, 0, gScenarioBufferData283_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData283_SPLIT_CHANNEL_2CH, 4},
{8, 26, 1664, 0, 0, gScenarioBufferData284_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData284_SPLIT_CHANNEL_2CH, 4},
{10, 26, 2080, 0, 0, gScenarioBufferData285_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData285_SPLIT_CHANNEL_2CH, 4},
{12, 26, 2496, 0, 0, gScenarioBufferData286_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData286_SPLIT_CHANNEL_2CH, 4},
{14, 26, 2912, 0, 0, gScenarioBufferData287_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData287_SPLIT_CHANNEL_2CH, 4},
{16, 26, 3328, 0, 0, gScenarioBufferData288_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData288_SPLIT_CHANNEL_2CH, 4},
{18, 26, 3744, 0, 0, gScenarioBufferData289_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData289_SPLIT_CHANNEL_2CH, 4},
{2, 28, 448, 0, 0, gScenarioBufferData290_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData290_SPLIT_CHANNEL_2CH, 4},
{4, 28, 896, 0, 0, gScenarioBufferData291_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData291_SPLIT_CHANNEL_2CH, 4},
{6, 28, 1344, 0, 0, gScenarioBufferData292_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData292_SPLIT_CHANNEL_2CH, 4},
{8, 28, 1792, 0, 0, gScenarioBufferData293_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData293_SPLIT_CHANNEL_2CH, 4},
{10, 28, 2240, 0, 0, gScenarioBufferData294_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData294_SPLIT_CHANNEL_2CH, 4},
{12, 28, 2688, 0, 0, gScenarioBufferData295_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData295_SPLIT_CHANNEL_2CH, 4},
{14, 28, 3136, 0, 0, gScenarioBufferData296_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData296_SPLIT_CHANNEL_2CH, 4},
{16, 28, 3584, 0, 0, gScenarioBufferData297_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData297_SPLIT_CHANNEL_2CH, 4},
{2, 30, 480, 0, 0, gScenarioBufferData298_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData298_SPLIT_CHANNEL_2CH, 4},
{4, 30, 960, 0, 0, gScenarioBufferData299_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData299_SPLIT_CHANNEL_2CH, 4},
{6, 30, 1440, 0, 0, gScenarioBufferData300_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData300_SPLIT_CHANNEL_2CH, 4},
{8, 30, 1920, 0, 0, gScenarioBufferData301_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData301_SPLIT_CHANNEL_2CH, 4},
{10, 30, 2400, 0, 0, gScenarioBufferData302_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData302_SPLIT_CHANNEL_2CH, 4},
{12, 30, 2880, 0, 0, gScenarioBufferData303_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData303_SPLIT_CHANNEL_2CH, 4},
{14, 30, 3360, 0, 0, gScenarioBufferData304_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData304_SPLIT_CHANNEL_2CH, 4},
{16, 30, 3840, 0, 0, gScenarioBufferData305_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData305_SPLIT_CHANNEL_2CH, 4},
{2, 32, 512, 0, 0, gScenarioBufferData306_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData306_SPLIT_CHANNEL_2CH, 4},
{4, 32, 1024, 0, 0, gScenarioBufferData307_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData307_SPLIT_CHANNEL_2CH, 4},
{6, 32, 1536, 0, 0, gScenarioBufferData308_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData308_SPLIT_CHANNEL_2CH, 4},
{8, 32, 2048, 0, 0, gScenarioBufferData309_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData309_SPLIT_CHANNEL_2CH, 4},
{10, 32, 2560, 0, 0, gScenarioBufferData310_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData310_SPLIT_CHANNEL_2CH, 4},
{12, 32, 3072, 0, 0, gScenarioBufferData311_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData311_SPLIT_CHANNEL_2CH, 4},
{14, 32, 3584, 0, 0, gScenarioBufferData312_SPLIT_CHANNEL_2CH, 48, gScenarioKernelData312_SPLIT_CHANNEL_2CH, 4}
};

static acf_scenario_list gScenarioList_SPLIT_CHANNEL_2CH = {
313, //number of scenarios
gScenarioArray_SPLIT_CHANNEL_2CH};
//**************************************************************

class SPLIT_CHANNEL_2CH : public ACF_Process_APU
{

public:
   SPLIT_CHANNEL_2CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("SPLIT_CHANNEL_2CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("SPLIT_CHANNEL_2CH",
                                        SPLIT_CHANNEL_2CH_LOAD_SEGMENTS,
                                        SPLIT_CHANNEL_2CH_LOAD_PMEM, SPLIT_CHANNEL_2CH_LOAD_PMEM_SIZE,
                                        SPLIT_CHANNEL_2CH_LOAD_DMEM, SPLIT_CHANNEL_2CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(SPLIT_CHANNEL_2CH_APEX_LOG_BUFFER, SPLIT_CHANNEL_2CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_SPLIT_CHANNEL_2CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_SPLIT_CHANNEL_2CH

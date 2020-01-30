#ifndef _ACF_PROCESS_APU_EXTRACT_CHANNEL_2CH
#define _ACF_PROCESS_APU_EXTRACT_CHANNEL_2CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <EXTRACT_CHANNEL_2CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_EXTRACT_CHANNEL_2CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_EXTRACT_CHANNEL_2CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_EXTRACT_CHANNEL_2CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_EXTRACT_CHANNEL_2CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_EXTRACT_CHANNEL_2CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_EXTRACT_CHANNEL_2CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_EXTRACT_CHANNEL_2CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_EXTRACT_CHANNEL_2CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_EXTRACT_CHANNEL_2CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_EXTRACT_CHANNEL_2CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_EXTRACT_CHANNEL_2CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_EXTRACT_CHANNEL_2CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_EXTRACT_CHANNEL_2CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_EXTRACT_CHANNEL_2CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_EXTRACT_CHANNEL_2CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_EXTRACT_CHANNEL_2CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_EXTRACT_CHANNEL_2CH[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_EXTRACT_CHANNEL_2CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_EXTRACT_CHANNEL_2CH[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_EXTRACT_CHANNEL_2CH[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_EXTRACT_CHANNEL_2CH[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_EXTRACT_CHANNEL_2CH[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_EXTRACT_CHANNEL_2CH[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_EXTRACT_CHANNEL_2CH[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_EXTRACT_CHANNEL_2CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_EXTRACT_CHANNEL_2CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_EXTRACT_CHANNEL_2CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_EXTRACT_CHANNEL_2CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_EXTRACT_CHANNEL_2CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_EXTRACT_CHANNEL_2CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_EXTRACT_CHANNEL_2CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_EXTRACT_CHANNEL_2CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_EXTRACT_CHANNEL_2CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_EXTRACT_CHANNEL_2CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_EXTRACT_CHANNEL_2CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_EXTRACT_CHANNEL_2CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_EXTRACT_CHANNEL_2CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_EXTRACT_CHANNEL_2CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_EXTRACT_CHANNEL_2CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_EXTRACT_CHANNEL_2CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_EXTRACT_CHANNEL_2CH[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_EXTRACT_CHANNEL_2CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_EXTRACT_CHANNEL_2CH[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_EXTRACT_CHANNEL_2CH[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_EXTRACT_CHANNEL_2CH[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_EXTRACT_CHANNEL_2CH[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_EXTRACT_CHANNEL_2CH[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_EXTRACT_CHANNEL_2CH[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_EXTRACT_CHANNEL_2CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_EXTRACT_CHANNEL_2CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_EXTRACT_CHANNEL_2CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_EXTRACT_CHANNEL_2CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_EXTRACT_CHANNEL_2CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_EXTRACT_CHANNEL_2CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_EXTRACT_CHANNEL_2CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_EXTRACT_CHANNEL_2CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_EXTRACT_CHANNEL_2CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_EXTRACT_CHANNEL_2CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_EXTRACT_CHANNEL_2CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_EXTRACT_CHANNEL_2CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_EXTRACT_CHANNEL_2CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_EXTRACT_CHANNEL_2CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_EXTRACT_CHANNEL_2CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_EXTRACT_CHANNEL_2CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_EXTRACT_CHANNEL_2CH[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_EXTRACT_CHANNEL_2CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_EXTRACT_CHANNEL_2CH[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_EXTRACT_CHANNEL_2CH[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_EXTRACT_CHANNEL_2CH[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_EXTRACT_CHANNEL_2CH[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_EXTRACT_CHANNEL_2CH[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_EXTRACT_CHANNEL_2CH[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_EXTRACT_CHANNEL_2CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_EXTRACT_CHANNEL_2CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_EXTRACT_CHANNEL_2CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_EXTRACT_CHANNEL_2CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_EXTRACT_CHANNEL_2CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_EXTRACT_CHANNEL_2CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_EXTRACT_CHANNEL_2CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_EXTRACT_CHANNEL_2CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_EXTRACT_CHANNEL_2CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_EXTRACT_CHANNEL_2CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_EXTRACT_CHANNEL_2CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_EXTRACT_CHANNEL_2CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_EXTRACT_CHANNEL_2CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_EXTRACT_CHANNEL_2CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_EXTRACT_CHANNEL_2CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_EXTRACT_CHANNEL_2CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_EXTRACT_CHANNEL_2CH[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_EXTRACT_CHANNEL_2CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_EXTRACT_CHANNEL_2CH[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_EXTRACT_CHANNEL_2CH[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_EXTRACT_CHANNEL_2CH[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_EXTRACT_CHANNEL_2CH[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_EXTRACT_CHANNEL_2CH[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_EXTRACT_CHANNEL_2CH[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_EXTRACT_CHANNEL_2CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_EXTRACT_CHANNEL_2CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_EXTRACT_CHANNEL_2CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_EXTRACT_CHANNEL_2CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_EXTRACT_CHANNEL_2CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_EXTRACT_CHANNEL_2CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_EXTRACT_CHANNEL_2CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_EXTRACT_CHANNEL_2CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_EXTRACT_CHANNEL_2CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_EXTRACT_CHANNEL_2CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_EXTRACT_CHANNEL_2CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_EXTRACT_CHANNEL_2CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_EXTRACT_CHANNEL_2CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_EXTRACT_CHANNEL_2CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_EXTRACT_CHANNEL_2CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_EXTRACT_CHANNEL_2CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_EXTRACT_CHANNEL_2CH[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_EXTRACT_CHANNEL_2CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_EXTRACT_CHANNEL_2CH[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_EXTRACT_CHANNEL_2CH[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_EXTRACT_CHANNEL_2CH[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_EXTRACT_CHANNEL_2CH[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_EXTRACT_CHANNEL_2CH[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_EXTRACT_CHANNEL_2CH[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_EXTRACT_CHANNEL_2CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_EXTRACT_CHANNEL_2CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_EXTRACT_CHANNEL_2CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_EXTRACT_CHANNEL_2CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_EXTRACT_CHANNEL_2CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_EXTRACT_CHANNEL_2CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_EXTRACT_CHANNEL_2CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_EXTRACT_CHANNEL_2CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_EXTRACT_CHANNEL_2CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_EXTRACT_CHANNEL_2CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_EXTRACT_CHANNEL_2CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_EXTRACT_CHANNEL_2CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_EXTRACT_CHANNEL_2CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_EXTRACT_CHANNEL_2CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_EXTRACT_CHANNEL_2CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_EXTRACT_CHANNEL_2CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_EXTRACT_CHANNEL_2CH[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_EXTRACT_CHANNEL_2CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_EXTRACT_CHANNEL_2CH[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_EXTRACT_CHANNEL_2CH[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_EXTRACT_CHANNEL_2CH[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_EXTRACT_CHANNEL_2CH[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_EXTRACT_CHANNEL_2CH[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_EXTRACT_CHANNEL_2CH[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_EXTRACT_CHANNEL_2CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_EXTRACT_CHANNEL_2CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_EXTRACT_CHANNEL_2CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_EXTRACT_CHANNEL_2CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_EXTRACT_CHANNEL_2CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_EXTRACT_CHANNEL_2CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_EXTRACT_CHANNEL_2CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_EXTRACT_CHANNEL_2CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_EXTRACT_CHANNEL_2CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_EXTRACT_CHANNEL_2CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_EXTRACT_CHANNEL_2CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_EXTRACT_CHANNEL_2CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_EXTRACT_CHANNEL_2CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_EXTRACT_CHANNEL_2CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_EXTRACT_CHANNEL_2CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_EXTRACT_CHANNEL_2CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_EXTRACT_CHANNEL_2CH[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_EXTRACT_CHANNEL_2CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_EXTRACT_CHANNEL_2CH[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_EXTRACT_CHANNEL_2CH[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_EXTRACT_CHANNEL_2CH[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_EXTRACT_CHANNEL_2CH[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_EXTRACT_CHANNEL_2CH[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_EXTRACT_CHANNEL_2CH[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_EXTRACT_CHANNEL_2CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_EXTRACT_CHANNEL_2CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_EXTRACT_CHANNEL_2CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_EXTRACT_CHANNEL_2CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_EXTRACT_CHANNEL_2CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_EXTRACT_CHANNEL_2CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_EXTRACT_CHANNEL_2CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_EXTRACT_CHANNEL_2CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_EXTRACT_CHANNEL_2CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_EXTRACT_CHANNEL_2CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_EXTRACT_CHANNEL_2CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_EXTRACT_CHANNEL_2CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_EXTRACT_CHANNEL_2CH[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_EXTRACT_CHANNEL_2CH[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_EXTRACT_CHANNEL_2CH[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_EXTRACT_CHANNEL_2CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_EXTRACT_CHANNEL_2CH[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_EXTRACT_CHANNEL_2CH[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_EXTRACT_CHANNEL_2CH[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_EXTRACT_CHANNEL_2CH[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_EXTRACT_CHANNEL_2CH[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_EXTRACT_CHANNEL_2CH[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_EXTRACT_CHANNEL_2CH[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_EXTRACT_CHANNEL_2CH[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_EXTRACT_CHANNEL_2CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_EXTRACT_CHANNEL_2CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_EXTRACT_CHANNEL_2CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_EXTRACT_CHANNEL_2CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_EXTRACT_CHANNEL_2CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_EXTRACT_CHANNEL_2CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_EXTRACT_CHANNEL_2CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_EXTRACT_CHANNEL_2CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_EXTRACT_CHANNEL_2CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_EXTRACT_CHANNEL_2CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_EXTRACT_CHANNEL_2CH[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_EXTRACT_CHANNEL_2CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_EXTRACT_CHANNEL_2CH[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_EXTRACT_CHANNEL_2CH[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_EXTRACT_CHANNEL_2CH[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_EXTRACT_CHANNEL_2CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_EXTRACT_CHANNEL_2CH[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_EXTRACT_CHANNEL_2CH[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_EXTRACT_CHANNEL_2CH[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_EXTRACT_CHANNEL_2CH[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_EXTRACT_CHANNEL_2CH[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_EXTRACT_CHANNEL_2CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_EXTRACT_CHANNEL_2CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_EXTRACT_CHANNEL_2CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_EXTRACT_CHANNEL_2CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_EXTRACT_CHANNEL_2CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_EXTRACT_CHANNEL_2CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_EXTRACT_CHANNEL_2CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_EXTRACT_CHANNEL_2CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_EXTRACT_CHANNEL_2CH[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_EXTRACT_CHANNEL_2CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_EXTRACT_CHANNEL_2CH[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_EXTRACT_CHANNEL_2CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_EXTRACT_CHANNEL_2CH[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_EXTRACT_CHANNEL_2CH[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_EXTRACT_CHANNEL_2CH[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_EXTRACT_CHANNEL_2CH[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_EXTRACT_CHANNEL_2CH[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_EXTRACT_CHANNEL_2CH[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_EXTRACT_CHANNEL_2CH[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_EXTRACT_CHANNEL_2CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_EXTRACT_CHANNEL_2CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_EXTRACT_CHANNEL_2CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_EXTRACT_CHANNEL_2CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_EXTRACT_CHANNEL_2CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_EXTRACT_CHANNEL_2CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_EXTRACT_CHANNEL_2CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_EXTRACT_CHANNEL_2CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_EXTRACT_CHANNEL_2CH[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_EXTRACT_CHANNEL_2CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_EXTRACT_CHANNEL_2CH[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_EXTRACT_CHANNEL_2CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_EXTRACT_CHANNEL_2CH[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_EXTRACT_CHANNEL_2CH[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_EXTRACT_CHANNEL_2CH[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_EXTRACT_CHANNEL_2CH[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_EXTRACT_CHANNEL_2CH[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_EXTRACT_CHANNEL_2CH[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_EXTRACT_CHANNEL_2CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_EXTRACT_CHANNEL_2CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_EXTRACT_CHANNEL_2CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_EXTRACT_CHANNEL_2CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_EXTRACT_CHANNEL_2CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_EXTRACT_CHANNEL_2CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_EXTRACT_CHANNEL_2CH[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_EXTRACT_CHANNEL_2CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_EXTRACT_CHANNEL_2CH[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_EXTRACT_CHANNEL_2CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_EXTRACT_CHANNEL_2CH[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_EXTRACT_CHANNEL_2CH[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_EXTRACT_CHANNEL_2CH[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_EXTRACT_CHANNEL_2CH[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_EXTRACT_CHANNEL_2CH[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_EXTRACT_CHANNEL_2CH[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_EXTRACT_CHANNEL_2CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_EXTRACT_CHANNEL_2CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_EXTRACT_CHANNEL_2CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_EXTRACT_CHANNEL_2CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_EXTRACT_CHANNEL_2CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_EXTRACT_CHANNEL_2CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_EXTRACT_CHANNEL_2CH[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_EXTRACT_CHANNEL_2CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_EXTRACT_CHANNEL_2CH[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_EXTRACT_CHANNEL_2CH[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_EXTRACT_CHANNEL_2CH[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_EXTRACT_CHANNEL_2CH[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_EXTRACT_CHANNEL_2CH[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_EXTRACT_CHANNEL_2CH[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_EXTRACT_CHANNEL_2CH[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_EXTRACT_CHANNEL_2CH[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_EXTRACT_CHANNEL_2CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_EXTRACT_CHANNEL_2CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_EXTRACT_CHANNEL_2CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_EXTRACT_CHANNEL_2CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_EXTRACT_CHANNEL_2CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_EXTRACT_CHANNEL_2CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_EXTRACT_CHANNEL_2CH[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_EXTRACT_CHANNEL_2CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_EXTRACT_CHANNEL_2CH[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_EXTRACT_CHANNEL_2CH[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_EXTRACT_CHANNEL_2CH[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_EXTRACT_CHANNEL_2CH[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_EXTRACT_CHANNEL_2CH[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_EXTRACT_CHANNEL_2CH[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_EXTRACT_CHANNEL_2CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_EXTRACT_CHANNEL_2CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_EXTRACT_CHANNEL_2CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_EXTRACT_CHANNEL_2CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_EXTRACT_CHANNEL_2CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_EXTRACT_CHANNEL_2CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_EXTRACT_CHANNEL_2CH[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_EXTRACT_CHANNEL_2CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_EXTRACT_CHANNEL_2CH[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_EXTRACT_CHANNEL_2CH[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_EXTRACT_CHANNEL_2CH[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_EXTRACT_CHANNEL_2CH[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_EXTRACT_CHANNEL_2CH[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_EXTRACT_CHANNEL_2CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_EXTRACT_CHANNEL_2CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_EXTRACT_CHANNEL_2CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_EXTRACT_CHANNEL_2CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_EXTRACT_CHANNEL_2CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_EXTRACT_CHANNEL_2CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_EXTRACT_CHANNEL_2CH[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_EXTRACT_CHANNEL_2CH[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_EXTRACT_CHANNEL_2CH[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_EXTRACT_CHANNEL_2CH[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_EXTRACT_CHANNEL_2CH[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_EXTRACT_CHANNEL_2CH[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_EXTRACT_CHANNEL_2CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_EXTRACT_CHANNEL_2CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_EXTRACT_CHANNEL_2CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_EXTRACT_CHANNEL_2CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_EXTRACT_CHANNEL_2CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_EXTRACT_CHANNEL_2CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_EXTRACT_CHANNEL_2CH[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_EXTRACT_CHANNEL_2CH[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_EXTRACT_CHANNEL_2CH[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_EXTRACT_CHANNEL_2CH[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_EXTRACT_CHANNEL_2CH[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_EXTRACT_CHANNEL_2CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_EXTRACT_CHANNEL_2CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_EXTRACT_CHANNEL_2CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_EXTRACT_CHANNEL_2CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_EXTRACT_CHANNEL_2CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_EXTRACT_CHANNEL_2CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_EXTRACT_CHANNEL_2CH[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_EXTRACT_CHANNEL_2CH[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_EXTRACT_CHANNEL_2CH[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_EXTRACT_CHANNEL_2CH[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_EXTRACT_CHANNEL_2CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_EXTRACT_CHANNEL_2CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_EXTRACT_CHANNEL_2CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_EXTRACT_CHANNEL_2CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_EXTRACT_CHANNEL_2CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_EXTRACT_CHANNEL_2CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_EXTRACT_CHANNEL_2CH[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_EXTRACT_CHANNEL_2CH[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_EXTRACT_CHANNEL_2CH[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_EXTRACT_CHANNEL_2CH[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_EXTRACT_CHANNEL_2CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_EXTRACT_CHANNEL_2CH[] = {{0, 0}};

static acf_scenario gScenarioArray_EXTRACT_CHANNEL_2CH[] = {
{2, 1, 12, 16, 0, gScenarioBufferData0_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData0_EXTRACT_CHANNEL_2CH, 4},
{4, 1, 24, 16, 0, gScenarioBufferData1_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData1_EXTRACT_CHANNEL_2CH, 4},
{6, 1, 36, 16, 0, gScenarioBufferData2_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData2_EXTRACT_CHANNEL_2CH, 4},
{8, 1, 48, 16, 0, gScenarioBufferData3_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData3_EXTRACT_CHANNEL_2CH, 4},
{10, 1, 60, 16, 0, gScenarioBufferData4_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData4_EXTRACT_CHANNEL_2CH, 4},
{12, 1, 72, 16, 0, gScenarioBufferData5_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData5_EXTRACT_CHANNEL_2CH, 4},
{14, 1, 84, 16, 0, gScenarioBufferData6_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData6_EXTRACT_CHANNEL_2CH, 4},
{16, 1, 96, 16, 0, gScenarioBufferData7_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData7_EXTRACT_CHANNEL_2CH, 4},
{18, 1, 108, 16, 0, gScenarioBufferData8_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData8_EXTRACT_CHANNEL_2CH, 4},
{20, 1, 120, 16, 0, gScenarioBufferData9_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData9_EXTRACT_CHANNEL_2CH, 4},
{22, 1, 132, 16, 0, gScenarioBufferData10_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData10_EXTRACT_CHANNEL_2CH, 4},
{24, 1, 144, 16, 0, gScenarioBufferData11_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData11_EXTRACT_CHANNEL_2CH, 4},
{26, 1, 156, 16, 0, gScenarioBufferData12_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData12_EXTRACT_CHANNEL_2CH, 4},
{28, 1, 168, 16, 0, gScenarioBufferData13_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData13_EXTRACT_CHANNEL_2CH, 4},
{30, 1, 180, 16, 0, gScenarioBufferData14_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData14_EXTRACT_CHANNEL_2CH, 4},
{32, 1, 192, 16, 0, gScenarioBufferData15_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData15_EXTRACT_CHANNEL_2CH, 4},
{36, 1, 216, 16, 0, gScenarioBufferData16_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData16_EXTRACT_CHANNEL_2CH, 4},
{40, 1, 240, 16, 0, gScenarioBufferData17_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData17_EXTRACT_CHANNEL_2CH, 4},
{44, 1, 264, 16, 0, gScenarioBufferData18_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData18_EXTRACT_CHANNEL_2CH, 4},
{48, 1, 288, 16, 0, gScenarioBufferData19_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData19_EXTRACT_CHANNEL_2CH, 4},
{52, 1, 312, 16, 0, gScenarioBufferData20_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData20_EXTRACT_CHANNEL_2CH, 4},
{56, 1, 336, 16, 0, gScenarioBufferData21_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData21_EXTRACT_CHANNEL_2CH, 4},
{60, 1, 360, 16, 0, gScenarioBufferData22_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData22_EXTRACT_CHANNEL_2CH, 4},
{64, 1, 384, 16, 0, gScenarioBufferData23_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData23_EXTRACT_CHANNEL_2CH, 4},
{2, 2, 24, 16, 0, gScenarioBufferData24_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData24_EXTRACT_CHANNEL_2CH, 4},
{4, 2, 48, 16, 0, gScenarioBufferData25_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData25_EXTRACT_CHANNEL_2CH, 4},
{6, 2, 72, 16, 0, gScenarioBufferData26_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData26_EXTRACT_CHANNEL_2CH, 4},
{8, 2, 96, 16, 0, gScenarioBufferData27_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData27_EXTRACT_CHANNEL_2CH, 4},
{10, 2, 120, 16, 0, gScenarioBufferData28_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData28_EXTRACT_CHANNEL_2CH, 4},
{12, 2, 144, 16, 0, gScenarioBufferData29_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData29_EXTRACT_CHANNEL_2CH, 4},
{14, 2, 168, 16, 0, gScenarioBufferData30_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData30_EXTRACT_CHANNEL_2CH, 4},
{16, 2, 192, 16, 0, gScenarioBufferData31_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData31_EXTRACT_CHANNEL_2CH, 4},
{18, 2, 216, 16, 0, gScenarioBufferData32_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData32_EXTRACT_CHANNEL_2CH, 4},
{20, 2, 240, 16, 0, gScenarioBufferData33_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData33_EXTRACT_CHANNEL_2CH, 4},
{22, 2, 264, 16, 0, gScenarioBufferData34_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData34_EXTRACT_CHANNEL_2CH, 4},
{24, 2, 288, 16, 0, gScenarioBufferData35_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData35_EXTRACT_CHANNEL_2CH, 4},
{26, 2, 312, 16, 0, gScenarioBufferData36_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData36_EXTRACT_CHANNEL_2CH, 4},
{28, 2, 336, 16, 0, gScenarioBufferData37_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData37_EXTRACT_CHANNEL_2CH, 4},
{30, 2, 360, 16, 0, gScenarioBufferData38_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData38_EXTRACT_CHANNEL_2CH, 4},
{32, 2, 384, 16, 0, gScenarioBufferData39_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData39_EXTRACT_CHANNEL_2CH, 4},
{36, 2, 432, 16, 0, gScenarioBufferData40_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData40_EXTRACT_CHANNEL_2CH, 4},
{40, 2, 480, 16, 0, gScenarioBufferData41_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData41_EXTRACT_CHANNEL_2CH, 4},
{44, 2, 528, 16, 0, gScenarioBufferData42_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData42_EXTRACT_CHANNEL_2CH, 4},
{48, 2, 576, 16, 0, gScenarioBufferData43_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData43_EXTRACT_CHANNEL_2CH, 4},
{52, 2, 624, 16, 0, gScenarioBufferData44_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData44_EXTRACT_CHANNEL_2CH, 4},
{56, 2, 672, 16, 0, gScenarioBufferData45_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData45_EXTRACT_CHANNEL_2CH, 4},
{60, 2, 720, 16, 0, gScenarioBufferData46_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData46_EXTRACT_CHANNEL_2CH, 4},
{64, 2, 768, 16, 0, gScenarioBufferData47_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData47_EXTRACT_CHANNEL_2CH, 4},
{2, 3, 36, 16, 0, gScenarioBufferData48_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData48_EXTRACT_CHANNEL_2CH, 4},
{4, 3, 72, 16, 0, gScenarioBufferData49_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData49_EXTRACT_CHANNEL_2CH, 4},
{6, 3, 108, 16, 0, gScenarioBufferData50_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData50_EXTRACT_CHANNEL_2CH, 4},
{8, 3, 144, 16, 0, gScenarioBufferData51_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData51_EXTRACT_CHANNEL_2CH, 4},
{10, 3, 180, 16, 0, gScenarioBufferData52_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData52_EXTRACT_CHANNEL_2CH, 4},
{12, 3, 216, 16, 0, gScenarioBufferData53_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData53_EXTRACT_CHANNEL_2CH, 4},
{14, 3, 252, 16, 0, gScenarioBufferData54_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData54_EXTRACT_CHANNEL_2CH, 4},
{16, 3, 288, 16, 0, gScenarioBufferData55_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData55_EXTRACT_CHANNEL_2CH, 4},
{18, 3, 324, 16, 0, gScenarioBufferData56_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData56_EXTRACT_CHANNEL_2CH, 4},
{20, 3, 360, 16, 0, gScenarioBufferData57_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData57_EXTRACT_CHANNEL_2CH, 4},
{22, 3, 396, 16, 0, gScenarioBufferData58_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData58_EXTRACT_CHANNEL_2CH, 4},
{24, 3, 432, 16, 0, gScenarioBufferData59_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData59_EXTRACT_CHANNEL_2CH, 4},
{26, 3, 468, 16, 0, gScenarioBufferData60_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData60_EXTRACT_CHANNEL_2CH, 4},
{28, 3, 504, 16, 0, gScenarioBufferData61_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData61_EXTRACT_CHANNEL_2CH, 4},
{30, 3, 540, 16, 0, gScenarioBufferData62_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData62_EXTRACT_CHANNEL_2CH, 4},
{32, 3, 576, 16, 0, gScenarioBufferData63_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData63_EXTRACT_CHANNEL_2CH, 4},
{36, 3, 648, 16, 0, gScenarioBufferData64_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData64_EXTRACT_CHANNEL_2CH, 4},
{40, 3, 720, 16, 0, gScenarioBufferData65_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData65_EXTRACT_CHANNEL_2CH, 4},
{44, 3, 792, 16, 0, gScenarioBufferData66_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData66_EXTRACT_CHANNEL_2CH, 4},
{48, 3, 864, 16, 0, gScenarioBufferData67_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData67_EXTRACT_CHANNEL_2CH, 4},
{52, 3, 936, 16, 0, gScenarioBufferData68_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData68_EXTRACT_CHANNEL_2CH, 4},
{56, 3, 1008, 16, 0, gScenarioBufferData69_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData69_EXTRACT_CHANNEL_2CH, 4},
{60, 3, 1080, 16, 0, gScenarioBufferData70_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData70_EXTRACT_CHANNEL_2CH, 4},
{64, 3, 1152, 16, 0, gScenarioBufferData71_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData71_EXTRACT_CHANNEL_2CH, 4},
{2, 4, 48, 16, 0, gScenarioBufferData72_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData72_EXTRACT_CHANNEL_2CH, 4},
{4, 4, 96, 16, 0, gScenarioBufferData73_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData73_EXTRACT_CHANNEL_2CH, 4},
{6, 4, 144, 16, 0, gScenarioBufferData74_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData74_EXTRACT_CHANNEL_2CH, 4},
{8, 4, 192, 16, 0, gScenarioBufferData75_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData75_EXTRACT_CHANNEL_2CH, 4},
{10, 4, 240, 16, 0, gScenarioBufferData76_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData76_EXTRACT_CHANNEL_2CH, 4},
{12, 4, 288, 16, 0, gScenarioBufferData77_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData77_EXTRACT_CHANNEL_2CH, 4},
{14, 4, 336, 16, 0, gScenarioBufferData78_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData78_EXTRACT_CHANNEL_2CH, 4},
{16, 4, 384, 16, 0, gScenarioBufferData79_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData79_EXTRACT_CHANNEL_2CH, 4},
{18, 4, 432, 16, 0, gScenarioBufferData80_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData80_EXTRACT_CHANNEL_2CH, 4},
{20, 4, 480, 16, 0, gScenarioBufferData81_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData81_EXTRACT_CHANNEL_2CH, 4},
{22, 4, 528, 16, 0, gScenarioBufferData82_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData82_EXTRACT_CHANNEL_2CH, 4},
{24, 4, 576, 16, 0, gScenarioBufferData83_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData83_EXTRACT_CHANNEL_2CH, 4},
{26, 4, 624, 16, 0, gScenarioBufferData84_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData84_EXTRACT_CHANNEL_2CH, 4},
{28, 4, 672, 16, 0, gScenarioBufferData85_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData85_EXTRACT_CHANNEL_2CH, 4},
{30, 4, 720, 16, 0, gScenarioBufferData86_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData86_EXTRACT_CHANNEL_2CH, 4},
{32, 4, 768, 16, 0, gScenarioBufferData87_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData87_EXTRACT_CHANNEL_2CH, 4},
{36, 4, 864, 16, 0, gScenarioBufferData88_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData88_EXTRACT_CHANNEL_2CH, 4},
{40, 4, 960, 16, 0, gScenarioBufferData89_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData89_EXTRACT_CHANNEL_2CH, 4},
{44, 4, 1056, 16, 0, gScenarioBufferData90_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData90_EXTRACT_CHANNEL_2CH, 4},
{48, 4, 1152, 16, 0, gScenarioBufferData91_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData91_EXTRACT_CHANNEL_2CH, 4},
{52, 4, 1248, 16, 0, gScenarioBufferData92_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData92_EXTRACT_CHANNEL_2CH, 4},
{56, 4, 1344, 16, 0, gScenarioBufferData93_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData93_EXTRACT_CHANNEL_2CH, 4},
{60, 4, 1440, 16, 0, gScenarioBufferData94_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData94_EXTRACT_CHANNEL_2CH, 4},
{64, 4, 1536, 16, 0, gScenarioBufferData95_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData95_EXTRACT_CHANNEL_2CH, 4},
{2, 5, 60, 16, 0, gScenarioBufferData96_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData96_EXTRACT_CHANNEL_2CH, 4},
{4, 5, 120, 16, 0, gScenarioBufferData97_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData97_EXTRACT_CHANNEL_2CH, 4},
{6, 5, 180, 16, 0, gScenarioBufferData98_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData98_EXTRACT_CHANNEL_2CH, 4},
{8, 5, 240, 16, 0, gScenarioBufferData99_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData99_EXTRACT_CHANNEL_2CH, 4},
{10, 5, 300, 16, 0, gScenarioBufferData100_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData100_EXTRACT_CHANNEL_2CH, 4},
{12, 5, 360, 16, 0, gScenarioBufferData101_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData101_EXTRACT_CHANNEL_2CH, 4},
{14, 5, 420, 16, 0, gScenarioBufferData102_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData102_EXTRACT_CHANNEL_2CH, 4},
{16, 5, 480, 16, 0, gScenarioBufferData103_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData103_EXTRACT_CHANNEL_2CH, 4},
{18, 5, 540, 16, 0, gScenarioBufferData104_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData104_EXTRACT_CHANNEL_2CH, 4},
{20, 5, 600, 16, 0, gScenarioBufferData105_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData105_EXTRACT_CHANNEL_2CH, 4},
{22, 5, 660, 16, 0, gScenarioBufferData106_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData106_EXTRACT_CHANNEL_2CH, 4},
{24, 5, 720, 16, 0, gScenarioBufferData107_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData107_EXTRACT_CHANNEL_2CH, 4},
{26, 5, 780, 16, 0, gScenarioBufferData108_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData108_EXTRACT_CHANNEL_2CH, 4},
{28, 5, 840, 16, 0, gScenarioBufferData109_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData109_EXTRACT_CHANNEL_2CH, 4},
{30, 5, 900, 16, 0, gScenarioBufferData110_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData110_EXTRACT_CHANNEL_2CH, 4},
{32, 5, 960, 16, 0, gScenarioBufferData111_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData111_EXTRACT_CHANNEL_2CH, 4},
{36, 5, 1080, 16, 0, gScenarioBufferData112_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData112_EXTRACT_CHANNEL_2CH, 4},
{40, 5, 1200, 16, 0, gScenarioBufferData113_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData113_EXTRACT_CHANNEL_2CH, 4},
{44, 5, 1320, 16, 0, gScenarioBufferData114_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData114_EXTRACT_CHANNEL_2CH, 4},
{48, 5, 1440, 16, 0, gScenarioBufferData115_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData115_EXTRACT_CHANNEL_2CH, 4},
{52, 5, 1560, 16, 0, gScenarioBufferData116_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData116_EXTRACT_CHANNEL_2CH, 4},
{56, 5, 1680, 16, 0, gScenarioBufferData117_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData117_EXTRACT_CHANNEL_2CH, 4},
{60, 5, 1800, 16, 0, gScenarioBufferData118_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData118_EXTRACT_CHANNEL_2CH, 4},
{64, 5, 1920, 16, 0, gScenarioBufferData119_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData119_EXTRACT_CHANNEL_2CH, 4},
{2, 6, 72, 16, 0, gScenarioBufferData120_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData120_EXTRACT_CHANNEL_2CH, 4},
{4, 6, 144, 16, 0, gScenarioBufferData121_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData121_EXTRACT_CHANNEL_2CH, 4},
{6, 6, 216, 16, 0, gScenarioBufferData122_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData122_EXTRACT_CHANNEL_2CH, 4},
{8, 6, 288, 16, 0, gScenarioBufferData123_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData123_EXTRACT_CHANNEL_2CH, 4},
{10, 6, 360, 16, 0, gScenarioBufferData124_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData124_EXTRACT_CHANNEL_2CH, 4},
{12, 6, 432, 16, 0, gScenarioBufferData125_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData125_EXTRACT_CHANNEL_2CH, 4},
{14, 6, 504, 16, 0, gScenarioBufferData126_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData126_EXTRACT_CHANNEL_2CH, 4},
{16, 6, 576, 16, 0, gScenarioBufferData127_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData127_EXTRACT_CHANNEL_2CH, 4},
{18, 6, 648, 16, 0, gScenarioBufferData128_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData128_EXTRACT_CHANNEL_2CH, 4},
{20, 6, 720, 16, 0, gScenarioBufferData129_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData129_EXTRACT_CHANNEL_2CH, 4},
{22, 6, 792, 16, 0, gScenarioBufferData130_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData130_EXTRACT_CHANNEL_2CH, 4},
{24, 6, 864, 16, 0, gScenarioBufferData131_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData131_EXTRACT_CHANNEL_2CH, 4},
{26, 6, 936, 16, 0, gScenarioBufferData132_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData132_EXTRACT_CHANNEL_2CH, 4},
{28, 6, 1008, 16, 0, gScenarioBufferData133_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData133_EXTRACT_CHANNEL_2CH, 4},
{30, 6, 1080, 16, 0, gScenarioBufferData134_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData134_EXTRACT_CHANNEL_2CH, 4},
{32, 6, 1152, 16, 0, gScenarioBufferData135_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData135_EXTRACT_CHANNEL_2CH, 4},
{36, 6, 1296, 16, 0, gScenarioBufferData136_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData136_EXTRACT_CHANNEL_2CH, 4},
{40, 6, 1440, 16, 0, gScenarioBufferData137_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData137_EXTRACT_CHANNEL_2CH, 4},
{44, 6, 1584, 16, 0, gScenarioBufferData138_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData138_EXTRACT_CHANNEL_2CH, 4},
{48, 6, 1728, 16, 0, gScenarioBufferData139_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData139_EXTRACT_CHANNEL_2CH, 4},
{52, 6, 1872, 16, 0, gScenarioBufferData140_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData140_EXTRACT_CHANNEL_2CH, 4},
{56, 6, 2016, 16, 0, gScenarioBufferData141_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData141_EXTRACT_CHANNEL_2CH, 4},
{60, 6, 2160, 16, 0, gScenarioBufferData142_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData142_EXTRACT_CHANNEL_2CH, 4},
{64, 6, 2304, 16, 0, gScenarioBufferData143_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData143_EXTRACT_CHANNEL_2CH, 4},
{2, 8, 96, 16, 0, gScenarioBufferData144_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData144_EXTRACT_CHANNEL_2CH, 4},
{4, 8, 192, 16, 0, gScenarioBufferData145_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData145_EXTRACT_CHANNEL_2CH, 4},
{6, 8, 288, 16, 0, gScenarioBufferData146_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData146_EXTRACT_CHANNEL_2CH, 4},
{8, 8, 384, 16, 0, gScenarioBufferData147_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData147_EXTRACT_CHANNEL_2CH, 4},
{10, 8, 480, 16, 0, gScenarioBufferData148_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData148_EXTRACT_CHANNEL_2CH, 4},
{12, 8, 576, 16, 0, gScenarioBufferData149_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData149_EXTRACT_CHANNEL_2CH, 4},
{14, 8, 672, 16, 0, gScenarioBufferData150_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData150_EXTRACT_CHANNEL_2CH, 4},
{16, 8, 768, 16, 0, gScenarioBufferData151_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData151_EXTRACT_CHANNEL_2CH, 4},
{18, 8, 864, 16, 0, gScenarioBufferData152_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData152_EXTRACT_CHANNEL_2CH, 4},
{20, 8, 960, 16, 0, gScenarioBufferData153_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData153_EXTRACT_CHANNEL_2CH, 4},
{22, 8, 1056, 16, 0, gScenarioBufferData154_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData154_EXTRACT_CHANNEL_2CH, 4},
{24, 8, 1152, 16, 0, gScenarioBufferData155_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData155_EXTRACT_CHANNEL_2CH, 4},
{26, 8, 1248, 16, 0, gScenarioBufferData156_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData156_EXTRACT_CHANNEL_2CH, 4},
{28, 8, 1344, 16, 0, gScenarioBufferData157_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData157_EXTRACT_CHANNEL_2CH, 4},
{30, 8, 1440, 16, 0, gScenarioBufferData158_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData158_EXTRACT_CHANNEL_2CH, 4},
{32, 8, 1536, 16, 0, gScenarioBufferData159_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData159_EXTRACT_CHANNEL_2CH, 4},
{36, 8, 1728, 16, 0, gScenarioBufferData160_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData160_EXTRACT_CHANNEL_2CH, 4},
{40, 8, 1920, 16, 0, gScenarioBufferData161_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData161_EXTRACT_CHANNEL_2CH, 4},
{44, 8, 2112, 16, 0, gScenarioBufferData162_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData162_EXTRACT_CHANNEL_2CH, 4},
{48, 8, 2304, 16, 0, gScenarioBufferData163_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData163_EXTRACT_CHANNEL_2CH, 4},
{52, 8, 2496, 16, 0, gScenarioBufferData164_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData164_EXTRACT_CHANNEL_2CH, 4},
{56, 8, 2688, 16, 0, gScenarioBufferData165_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData165_EXTRACT_CHANNEL_2CH, 4},
{60, 8, 2880, 16, 0, gScenarioBufferData166_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData166_EXTRACT_CHANNEL_2CH, 4},
{64, 8, 3072, 16, 0, gScenarioBufferData167_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData167_EXTRACT_CHANNEL_2CH, 4},
{2, 10, 120, 16, 0, gScenarioBufferData168_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData168_EXTRACT_CHANNEL_2CH, 4},
{4, 10, 240, 16, 0, gScenarioBufferData169_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData169_EXTRACT_CHANNEL_2CH, 4},
{6, 10, 360, 16, 0, gScenarioBufferData170_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData170_EXTRACT_CHANNEL_2CH, 4},
{8, 10, 480, 16, 0, gScenarioBufferData171_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData171_EXTRACT_CHANNEL_2CH, 4},
{10, 10, 600, 16, 0, gScenarioBufferData172_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData172_EXTRACT_CHANNEL_2CH, 4},
{12, 10, 720, 16, 0, gScenarioBufferData173_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData173_EXTRACT_CHANNEL_2CH, 4},
{14, 10, 840, 16, 0, gScenarioBufferData174_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData174_EXTRACT_CHANNEL_2CH, 4},
{16, 10, 960, 16, 0, gScenarioBufferData175_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData175_EXTRACT_CHANNEL_2CH, 4},
{18, 10, 1080, 16, 0, gScenarioBufferData176_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData176_EXTRACT_CHANNEL_2CH, 4},
{20, 10, 1200, 16, 0, gScenarioBufferData177_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData177_EXTRACT_CHANNEL_2CH, 4},
{22, 10, 1320, 16, 0, gScenarioBufferData178_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData178_EXTRACT_CHANNEL_2CH, 4},
{24, 10, 1440, 16, 0, gScenarioBufferData179_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData179_EXTRACT_CHANNEL_2CH, 4},
{26, 10, 1560, 16, 0, gScenarioBufferData180_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData180_EXTRACT_CHANNEL_2CH, 4},
{28, 10, 1680, 16, 0, gScenarioBufferData181_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData181_EXTRACT_CHANNEL_2CH, 4},
{30, 10, 1800, 16, 0, gScenarioBufferData182_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData182_EXTRACT_CHANNEL_2CH, 4},
{32, 10, 1920, 16, 0, gScenarioBufferData183_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData183_EXTRACT_CHANNEL_2CH, 4},
{36, 10, 2160, 16, 0, gScenarioBufferData184_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData184_EXTRACT_CHANNEL_2CH, 4},
{40, 10, 2400, 16, 0, gScenarioBufferData185_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData185_EXTRACT_CHANNEL_2CH, 4},
{44, 10, 2640, 16, 0, gScenarioBufferData186_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData186_EXTRACT_CHANNEL_2CH, 4},
{48, 10, 2880, 16, 0, gScenarioBufferData187_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData187_EXTRACT_CHANNEL_2CH, 4},
{52, 10, 3120, 16, 0, gScenarioBufferData188_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData188_EXTRACT_CHANNEL_2CH, 4},
{56, 10, 3360, 16, 0, gScenarioBufferData189_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData189_EXTRACT_CHANNEL_2CH, 4},
{60, 10, 3600, 16, 0, gScenarioBufferData190_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData190_EXTRACT_CHANNEL_2CH, 4},
{64, 10, 3840, 16, 0, gScenarioBufferData191_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData191_EXTRACT_CHANNEL_2CH, 4},
{2, 12, 144, 16, 0, gScenarioBufferData192_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData192_EXTRACT_CHANNEL_2CH, 4},
{4, 12, 288, 16, 0, gScenarioBufferData193_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData193_EXTRACT_CHANNEL_2CH, 4},
{6, 12, 432, 16, 0, gScenarioBufferData194_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData194_EXTRACT_CHANNEL_2CH, 4},
{8, 12, 576, 16, 0, gScenarioBufferData195_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData195_EXTRACT_CHANNEL_2CH, 4},
{10, 12, 720, 16, 0, gScenarioBufferData196_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData196_EXTRACT_CHANNEL_2CH, 4},
{12, 12, 864, 16, 0, gScenarioBufferData197_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData197_EXTRACT_CHANNEL_2CH, 4},
{14, 12, 1008, 16, 0, gScenarioBufferData198_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData198_EXTRACT_CHANNEL_2CH, 4},
{16, 12, 1152, 16, 0, gScenarioBufferData199_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData199_EXTRACT_CHANNEL_2CH, 4},
{18, 12, 1296, 16, 0, gScenarioBufferData200_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData200_EXTRACT_CHANNEL_2CH, 4},
{20, 12, 1440, 16, 0, gScenarioBufferData201_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData201_EXTRACT_CHANNEL_2CH, 4},
{22, 12, 1584, 16, 0, gScenarioBufferData202_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData202_EXTRACT_CHANNEL_2CH, 4},
{24, 12, 1728, 16, 0, gScenarioBufferData203_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData203_EXTRACT_CHANNEL_2CH, 4},
{26, 12, 1872, 16, 0, gScenarioBufferData204_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData204_EXTRACT_CHANNEL_2CH, 4},
{28, 12, 2016, 16, 0, gScenarioBufferData205_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData205_EXTRACT_CHANNEL_2CH, 4},
{30, 12, 2160, 16, 0, gScenarioBufferData206_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData206_EXTRACT_CHANNEL_2CH, 4},
{32, 12, 2304, 16, 0, gScenarioBufferData207_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData207_EXTRACT_CHANNEL_2CH, 4},
{36, 12, 2592, 16, 0, gScenarioBufferData208_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData208_EXTRACT_CHANNEL_2CH, 4},
{40, 12, 2880, 16, 0, gScenarioBufferData209_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData209_EXTRACT_CHANNEL_2CH, 4},
{44, 12, 3168, 16, 0, gScenarioBufferData210_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData210_EXTRACT_CHANNEL_2CH, 4},
{48, 12, 3456, 16, 0, gScenarioBufferData211_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData211_EXTRACT_CHANNEL_2CH, 4},
{52, 12, 3744, 16, 0, gScenarioBufferData212_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData212_EXTRACT_CHANNEL_2CH, 4},
{2, 14, 168, 16, 0, gScenarioBufferData213_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData213_EXTRACT_CHANNEL_2CH, 4},
{4, 14, 336, 16, 0, gScenarioBufferData214_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData214_EXTRACT_CHANNEL_2CH, 4},
{6, 14, 504, 16, 0, gScenarioBufferData215_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData215_EXTRACT_CHANNEL_2CH, 4},
{8, 14, 672, 16, 0, gScenarioBufferData216_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData216_EXTRACT_CHANNEL_2CH, 4},
{10, 14, 840, 16, 0, gScenarioBufferData217_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData217_EXTRACT_CHANNEL_2CH, 4},
{12, 14, 1008, 16, 0, gScenarioBufferData218_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData218_EXTRACT_CHANNEL_2CH, 4},
{14, 14, 1176, 16, 0, gScenarioBufferData219_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData219_EXTRACT_CHANNEL_2CH, 4},
{16, 14, 1344, 16, 0, gScenarioBufferData220_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData220_EXTRACT_CHANNEL_2CH, 4},
{18, 14, 1512, 16, 0, gScenarioBufferData221_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData221_EXTRACT_CHANNEL_2CH, 4},
{20, 14, 1680, 16, 0, gScenarioBufferData222_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData222_EXTRACT_CHANNEL_2CH, 4},
{22, 14, 1848, 16, 0, gScenarioBufferData223_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData223_EXTRACT_CHANNEL_2CH, 4},
{24, 14, 2016, 16, 0, gScenarioBufferData224_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData224_EXTRACT_CHANNEL_2CH, 4},
{26, 14, 2184, 16, 0, gScenarioBufferData225_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData225_EXTRACT_CHANNEL_2CH, 4},
{28, 14, 2352, 16, 0, gScenarioBufferData226_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData226_EXTRACT_CHANNEL_2CH, 4},
{30, 14, 2520, 16, 0, gScenarioBufferData227_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData227_EXTRACT_CHANNEL_2CH, 4},
{32, 14, 2688, 16, 0, gScenarioBufferData228_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData228_EXTRACT_CHANNEL_2CH, 4},
{36, 14, 3024, 16, 0, gScenarioBufferData229_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData229_EXTRACT_CHANNEL_2CH, 4},
{40, 14, 3360, 16, 0, gScenarioBufferData230_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData230_EXTRACT_CHANNEL_2CH, 4},
{44, 14, 3696, 16, 0, gScenarioBufferData231_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData231_EXTRACT_CHANNEL_2CH, 4},
{2, 16, 192, 16, 0, gScenarioBufferData232_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData232_EXTRACT_CHANNEL_2CH, 4},
{4, 16, 384, 16, 0, gScenarioBufferData233_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData233_EXTRACT_CHANNEL_2CH, 4},
{6, 16, 576, 16, 0, gScenarioBufferData234_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData234_EXTRACT_CHANNEL_2CH, 4},
{8, 16, 768, 16, 0, gScenarioBufferData235_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData235_EXTRACT_CHANNEL_2CH, 4},
{10, 16, 960, 16, 0, gScenarioBufferData236_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData236_EXTRACT_CHANNEL_2CH, 4},
{12, 16, 1152, 16, 0, gScenarioBufferData237_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData237_EXTRACT_CHANNEL_2CH, 4},
{14, 16, 1344, 16, 0, gScenarioBufferData238_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData238_EXTRACT_CHANNEL_2CH, 4},
{16, 16, 1536, 16, 0, gScenarioBufferData239_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData239_EXTRACT_CHANNEL_2CH, 4},
{18, 16, 1728, 16, 0, gScenarioBufferData240_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData240_EXTRACT_CHANNEL_2CH, 4},
{20, 16, 1920, 16, 0, gScenarioBufferData241_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData241_EXTRACT_CHANNEL_2CH, 4},
{22, 16, 2112, 16, 0, gScenarioBufferData242_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData242_EXTRACT_CHANNEL_2CH, 4},
{24, 16, 2304, 16, 0, gScenarioBufferData243_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData243_EXTRACT_CHANNEL_2CH, 4},
{26, 16, 2496, 16, 0, gScenarioBufferData244_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData244_EXTRACT_CHANNEL_2CH, 4},
{28, 16, 2688, 16, 0, gScenarioBufferData245_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData245_EXTRACT_CHANNEL_2CH, 4},
{30, 16, 2880, 16, 0, gScenarioBufferData246_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData246_EXTRACT_CHANNEL_2CH, 4},
{32, 16, 3072, 16, 0, gScenarioBufferData247_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData247_EXTRACT_CHANNEL_2CH, 4},
{36, 16, 3456, 16, 0, gScenarioBufferData248_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData248_EXTRACT_CHANNEL_2CH, 4},
{40, 16, 3840, 16, 0, gScenarioBufferData249_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData249_EXTRACT_CHANNEL_2CH, 4},
{2, 18, 216, 16, 0, gScenarioBufferData250_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData250_EXTRACT_CHANNEL_2CH, 4},
{4, 18, 432, 16, 0, gScenarioBufferData251_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData251_EXTRACT_CHANNEL_2CH, 4},
{6, 18, 648, 16, 0, gScenarioBufferData252_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData252_EXTRACT_CHANNEL_2CH, 4},
{8, 18, 864, 16, 0, gScenarioBufferData253_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData253_EXTRACT_CHANNEL_2CH, 4},
{10, 18, 1080, 16, 0, gScenarioBufferData254_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData254_EXTRACT_CHANNEL_2CH, 4},
{12, 18, 1296, 16, 0, gScenarioBufferData255_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData255_EXTRACT_CHANNEL_2CH, 4},
{14, 18, 1512, 16, 0, gScenarioBufferData256_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData256_EXTRACT_CHANNEL_2CH, 4},
{16, 18, 1728, 16, 0, gScenarioBufferData257_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData257_EXTRACT_CHANNEL_2CH, 4},
{18, 18, 1944, 16, 0, gScenarioBufferData258_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData258_EXTRACT_CHANNEL_2CH, 4},
{20, 18, 2160, 16, 0, gScenarioBufferData259_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData259_EXTRACT_CHANNEL_2CH, 4},
{22, 18, 2376, 16, 0, gScenarioBufferData260_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData260_EXTRACT_CHANNEL_2CH, 4},
{24, 18, 2592, 16, 0, gScenarioBufferData261_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData261_EXTRACT_CHANNEL_2CH, 4},
{26, 18, 2808, 16, 0, gScenarioBufferData262_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData262_EXTRACT_CHANNEL_2CH, 4},
{28, 18, 3024, 16, 0, gScenarioBufferData263_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData263_EXTRACT_CHANNEL_2CH, 4},
{30, 18, 3240, 16, 0, gScenarioBufferData264_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData264_EXTRACT_CHANNEL_2CH, 4},
{32, 18, 3456, 16, 0, gScenarioBufferData265_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData265_EXTRACT_CHANNEL_2CH, 4},
{2, 20, 240, 16, 0, gScenarioBufferData266_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData266_EXTRACT_CHANNEL_2CH, 4},
{4, 20, 480, 16, 0, gScenarioBufferData267_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData267_EXTRACT_CHANNEL_2CH, 4},
{6, 20, 720, 16, 0, gScenarioBufferData268_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData268_EXTRACT_CHANNEL_2CH, 4},
{8, 20, 960, 16, 0, gScenarioBufferData269_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData269_EXTRACT_CHANNEL_2CH, 4},
{10, 20, 1200, 16, 0, gScenarioBufferData270_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData270_EXTRACT_CHANNEL_2CH, 4},
{12, 20, 1440, 16, 0, gScenarioBufferData271_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData271_EXTRACT_CHANNEL_2CH, 4},
{14, 20, 1680, 16, 0, gScenarioBufferData272_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData272_EXTRACT_CHANNEL_2CH, 4},
{16, 20, 1920, 16, 0, gScenarioBufferData273_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData273_EXTRACT_CHANNEL_2CH, 4},
{18, 20, 2160, 16, 0, gScenarioBufferData274_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData274_EXTRACT_CHANNEL_2CH, 4},
{20, 20, 2400, 16, 0, gScenarioBufferData275_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData275_EXTRACT_CHANNEL_2CH, 4},
{22, 20, 2640, 16, 0, gScenarioBufferData276_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData276_EXTRACT_CHANNEL_2CH, 4},
{24, 20, 2880, 16, 0, gScenarioBufferData277_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData277_EXTRACT_CHANNEL_2CH, 4},
{26, 20, 3120, 16, 0, gScenarioBufferData278_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData278_EXTRACT_CHANNEL_2CH, 4},
{28, 20, 3360, 16, 0, gScenarioBufferData279_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData279_EXTRACT_CHANNEL_2CH, 4},
{30, 20, 3600, 16, 0, gScenarioBufferData280_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData280_EXTRACT_CHANNEL_2CH, 4},
{32, 20, 3840, 16, 0, gScenarioBufferData281_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData281_EXTRACT_CHANNEL_2CH, 4},
{2, 22, 264, 16, 0, gScenarioBufferData282_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData282_EXTRACT_CHANNEL_2CH, 4},
{4, 22, 528, 16, 0, gScenarioBufferData283_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData283_EXTRACT_CHANNEL_2CH, 4},
{6, 22, 792, 16, 0, gScenarioBufferData284_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData284_EXTRACT_CHANNEL_2CH, 4},
{8, 22, 1056, 16, 0, gScenarioBufferData285_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData285_EXTRACT_CHANNEL_2CH, 4},
{10, 22, 1320, 16, 0, gScenarioBufferData286_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData286_EXTRACT_CHANNEL_2CH, 4},
{12, 22, 1584, 16, 0, gScenarioBufferData287_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData287_EXTRACT_CHANNEL_2CH, 4},
{14, 22, 1848, 16, 0, gScenarioBufferData288_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData288_EXTRACT_CHANNEL_2CH, 4},
{16, 22, 2112, 16, 0, gScenarioBufferData289_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData289_EXTRACT_CHANNEL_2CH, 4},
{18, 22, 2376, 16, 0, gScenarioBufferData290_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData290_EXTRACT_CHANNEL_2CH, 4},
{20, 22, 2640, 16, 0, gScenarioBufferData291_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData291_EXTRACT_CHANNEL_2CH, 4},
{22, 22, 2904, 16, 0, gScenarioBufferData292_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData292_EXTRACT_CHANNEL_2CH, 4},
{24, 22, 3168, 16, 0, gScenarioBufferData293_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData293_EXTRACT_CHANNEL_2CH, 4},
{26, 22, 3432, 16, 0, gScenarioBufferData294_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData294_EXTRACT_CHANNEL_2CH, 4},
{28, 22, 3696, 16, 0, gScenarioBufferData295_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData295_EXTRACT_CHANNEL_2CH, 4},
{2, 24, 288, 16, 0, gScenarioBufferData296_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData296_EXTRACT_CHANNEL_2CH, 4},
{4, 24, 576, 16, 0, gScenarioBufferData297_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData297_EXTRACT_CHANNEL_2CH, 4},
{6, 24, 864, 16, 0, gScenarioBufferData298_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData298_EXTRACT_CHANNEL_2CH, 4},
{8, 24, 1152, 16, 0, gScenarioBufferData299_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData299_EXTRACT_CHANNEL_2CH, 4},
{10, 24, 1440, 16, 0, gScenarioBufferData300_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData300_EXTRACT_CHANNEL_2CH, 4},
{12, 24, 1728, 16, 0, gScenarioBufferData301_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData301_EXTRACT_CHANNEL_2CH, 4},
{14, 24, 2016, 16, 0, gScenarioBufferData302_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData302_EXTRACT_CHANNEL_2CH, 4},
{16, 24, 2304, 16, 0, gScenarioBufferData303_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData303_EXTRACT_CHANNEL_2CH, 4},
{18, 24, 2592, 16, 0, gScenarioBufferData304_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData304_EXTRACT_CHANNEL_2CH, 4},
{20, 24, 2880, 16, 0, gScenarioBufferData305_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData305_EXTRACT_CHANNEL_2CH, 4},
{22, 24, 3168, 16, 0, gScenarioBufferData306_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData306_EXTRACT_CHANNEL_2CH, 4},
{24, 24, 3456, 16, 0, gScenarioBufferData307_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData307_EXTRACT_CHANNEL_2CH, 4},
{26, 24, 3744, 16, 0, gScenarioBufferData308_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData308_EXTRACT_CHANNEL_2CH, 4},
{2, 26, 312, 16, 0, gScenarioBufferData309_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData309_EXTRACT_CHANNEL_2CH, 4},
{4, 26, 624, 16, 0, gScenarioBufferData310_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData310_EXTRACT_CHANNEL_2CH, 4},
{6, 26, 936, 16, 0, gScenarioBufferData311_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData311_EXTRACT_CHANNEL_2CH, 4},
{8, 26, 1248, 16, 0, gScenarioBufferData312_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData312_EXTRACT_CHANNEL_2CH, 4},
{10, 26, 1560, 16, 0, gScenarioBufferData313_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData313_EXTRACT_CHANNEL_2CH, 4},
{12, 26, 1872, 16, 0, gScenarioBufferData314_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData314_EXTRACT_CHANNEL_2CH, 4},
{14, 26, 2184, 16, 0, gScenarioBufferData315_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData315_EXTRACT_CHANNEL_2CH, 4},
{16, 26, 2496, 16, 0, gScenarioBufferData316_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData316_EXTRACT_CHANNEL_2CH, 4},
{18, 26, 2808, 16, 0, gScenarioBufferData317_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData317_EXTRACT_CHANNEL_2CH, 4},
{20, 26, 3120, 16, 0, gScenarioBufferData318_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData318_EXTRACT_CHANNEL_2CH, 4},
{22, 26, 3432, 16, 0, gScenarioBufferData319_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData319_EXTRACT_CHANNEL_2CH, 4},
{24, 26, 3744, 16, 0, gScenarioBufferData320_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData320_EXTRACT_CHANNEL_2CH, 4},
{2, 28, 336, 16, 0, gScenarioBufferData321_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData321_EXTRACT_CHANNEL_2CH, 4},
{4, 28, 672, 16, 0, gScenarioBufferData322_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData322_EXTRACT_CHANNEL_2CH, 4},
{6, 28, 1008, 16, 0, gScenarioBufferData323_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData323_EXTRACT_CHANNEL_2CH, 4},
{8, 28, 1344, 16, 0, gScenarioBufferData324_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData324_EXTRACT_CHANNEL_2CH, 4},
{10, 28, 1680, 16, 0, gScenarioBufferData325_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData325_EXTRACT_CHANNEL_2CH, 4},
{12, 28, 2016, 16, 0, gScenarioBufferData326_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData326_EXTRACT_CHANNEL_2CH, 4},
{14, 28, 2352, 16, 0, gScenarioBufferData327_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData327_EXTRACT_CHANNEL_2CH, 4},
{16, 28, 2688, 16, 0, gScenarioBufferData328_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData328_EXTRACT_CHANNEL_2CH, 4},
{18, 28, 3024, 16, 0, gScenarioBufferData329_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData329_EXTRACT_CHANNEL_2CH, 4},
{20, 28, 3360, 16, 0, gScenarioBufferData330_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData330_EXTRACT_CHANNEL_2CH, 4},
{22, 28, 3696, 16, 0, gScenarioBufferData331_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData331_EXTRACT_CHANNEL_2CH, 4},
{2, 30, 360, 16, 0, gScenarioBufferData332_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData332_EXTRACT_CHANNEL_2CH, 4},
{4, 30, 720, 16, 0, gScenarioBufferData333_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData333_EXTRACT_CHANNEL_2CH, 4},
{6, 30, 1080, 16, 0, gScenarioBufferData334_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData334_EXTRACT_CHANNEL_2CH, 4},
{8, 30, 1440, 16, 0, gScenarioBufferData335_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData335_EXTRACT_CHANNEL_2CH, 4},
{10, 30, 1800, 16, 0, gScenarioBufferData336_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData336_EXTRACT_CHANNEL_2CH, 4},
{12, 30, 2160, 16, 0, gScenarioBufferData337_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData337_EXTRACT_CHANNEL_2CH, 4},
{14, 30, 2520, 16, 0, gScenarioBufferData338_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData338_EXTRACT_CHANNEL_2CH, 4},
{16, 30, 2880, 16, 0, gScenarioBufferData339_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData339_EXTRACT_CHANNEL_2CH, 4},
{18, 30, 3240, 16, 0, gScenarioBufferData340_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData340_EXTRACT_CHANNEL_2CH, 4},
{20, 30, 3600, 16, 0, gScenarioBufferData341_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData341_EXTRACT_CHANNEL_2CH, 4},
{2, 32, 384, 16, 0, gScenarioBufferData342_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData342_EXTRACT_CHANNEL_2CH, 4},
{4, 32, 768, 16, 0, gScenarioBufferData343_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData343_EXTRACT_CHANNEL_2CH, 4},
{6, 32, 1152, 16, 0, gScenarioBufferData344_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData344_EXTRACT_CHANNEL_2CH, 4},
{8, 32, 1536, 16, 0, gScenarioBufferData345_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData345_EXTRACT_CHANNEL_2CH, 4},
{10, 32, 1920, 16, 0, gScenarioBufferData346_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData346_EXTRACT_CHANNEL_2CH, 4},
{12, 32, 2304, 16, 0, gScenarioBufferData347_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData347_EXTRACT_CHANNEL_2CH, 4},
{14, 32, 2688, 16, 0, gScenarioBufferData348_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData348_EXTRACT_CHANNEL_2CH, 4},
{16, 32, 3072, 16, 0, gScenarioBufferData349_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData349_EXTRACT_CHANNEL_2CH, 4},
{18, 32, 3456, 16, 0, gScenarioBufferData350_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData350_EXTRACT_CHANNEL_2CH, 4},
{20, 32, 3840, 16, 0, gScenarioBufferData351_EXTRACT_CHANNEL_2CH, 48, gScenarioKernelData351_EXTRACT_CHANNEL_2CH, 4}
};

static acf_scenario_list gScenarioList_EXTRACT_CHANNEL_2CH = {
352, //number of scenarios
gScenarioArray_EXTRACT_CHANNEL_2CH};
//**************************************************************

class EXTRACT_CHANNEL_2CH : public ACF_Process_APU
{

public:
   EXTRACT_CHANNEL_2CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("EXTRACT_CHANNEL_2CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("EXTRACT_CHANNEL_2CH",
                                        EXTRACT_CHANNEL_2CH_LOAD_SEGMENTS,
                                        EXTRACT_CHANNEL_2CH_LOAD_PMEM, EXTRACT_CHANNEL_2CH_LOAD_PMEM_SIZE,
                                        EXTRACT_CHANNEL_2CH_LOAD_DMEM, EXTRACT_CHANNEL_2CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(EXTRACT_CHANNEL_2CH_APEX_LOG_BUFFER, EXTRACT_CHANNEL_2CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_EXTRACT_CHANNEL_2CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_EXTRACT_CHANNEL_2CH

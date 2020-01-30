#ifndef _ACF_PROCESS_APU_EXTRACT_CHANNEL_4CH
#define _ACF_PROCESS_APU_EXTRACT_CHANNEL_4CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <EXTRACT_CHANNEL_4CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_EXTRACT_CHANNEL_4CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_EXTRACT_CHANNEL_4CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_EXTRACT_CHANNEL_4CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_EXTRACT_CHANNEL_4CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_EXTRACT_CHANNEL_4CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_EXTRACT_CHANNEL_4CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_EXTRACT_CHANNEL_4CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_EXTRACT_CHANNEL_4CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_EXTRACT_CHANNEL_4CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_EXTRACT_CHANNEL_4CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_EXTRACT_CHANNEL_4CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_EXTRACT_CHANNEL_4CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_EXTRACT_CHANNEL_4CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_EXTRACT_CHANNEL_4CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_EXTRACT_CHANNEL_4CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_EXTRACT_CHANNEL_4CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_EXTRACT_CHANNEL_4CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_EXTRACT_CHANNEL_4CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_EXTRACT_CHANNEL_4CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_EXTRACT_CHANNEL_4CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_EXTRACT_CHANNEL_4CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_EXTRACT_CHANNEL_4CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_EXTRACT_CHANNEL_4CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_EXTRACT_CHANNEL_4CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_EXTRACT_CHANNEL_4CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_EXTRACT_CHANNEL_4CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_EXTRACT_CHANNEL_4CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_EXTRACT_CHANNEL_4CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_EXTRACT_CHANNEL_4CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_EXTRACT_CHANNEL_4CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_EXTRACT_CHANNEL_4CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_EXTRACT_CHANNEL_4CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_EXTRACT_CHANNEL_4CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_EXTRACT_CHANNEL_4CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_EXTRACT_CHANNEL_4CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_EXTRACT_CHANNEL_4CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_EXTRACT_CHANNEL_4CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_EXTRACT_CHANNEL_4CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_EXTRACT_CHANNEL_4CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_EXTRACT_CHANNEL_4CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_EXTRACT_CHANNEL_4CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_EXTRACT_CHANNEL_4CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_EXTRACT_CHANNEL_4CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_EXTRACT_CHANNEL_4CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_EXTRACT_CHANNEL_4CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_EXTRACT_CHANNEL_4CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_EXTRACT_CHANNEL_4CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_EXTRACT_CHANNEL_4CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_EXTRACT_CHANNEL_4CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_EXTRACT_CHANNEL_4CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_EXTRACT_CHANNEL_4CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_EXTRACT_CHANNEL_4CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_EXTRACT_CHANNEL_4CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_EXTRACT_CHANNEL_4CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_EXTRACT_CHANNEL_4CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_EXTRACT_CHANNEL_4CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_EXTRACT_CHANNEL_4CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_EXTRACT_CHANNEL_4CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_EXTRACT_CHANNEL_4CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_EXTRACT_CHANNEL_4CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_EXTRACT_CHANNEL_4CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_EXTRACT_CHANNEL_4CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_EXTRACT_CHANNEL_4CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_EXTRACT_CHANNEL_4CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_EXTRACT_CHANNEL_4CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_EXTRACT_CHANNEL_4CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_EXTRACT_CHANNEL_4CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_EXTRACT_CHANNEL_4CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_EXTRACT_CHANNEL_4CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_EXTRACT_CHANNEL_4CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_EXTRACT_CHANNEL_4CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_EXTRACT_CHANNEL_4CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_EXTRACT_CHANNEL_4CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_EXTRACT_CHANNEL_4CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_EXTRACT_CHANNEL_4CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_EXTRACT_CHANNEL_4CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_EXTRACT_CHANNEL_4CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_EXTRACT_CHANNEL_4CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_EXTRACT_CHANNEL_4CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_EXTRACT_CHANNEL_4CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_EXTRACT_CHANNEL_4CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_EXTRACT_CHANNEL_4CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_EXTRACT_CHANNEL_4CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_EXTRACT_CHANNEL_4CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_EXTRACT_CHANNEL_4CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_EXTRACT_CHANNEL_4CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_EXTRACT_CHANNEL_4CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_EXTRACT_CHANNEL_4CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_EXTRACT_CHANNEL_4CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_EXTRACT_CHANNEL_4CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_EXTRACT_CHANNEL_4CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_EXTRACT_CHANNEL_4CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_EXTRACT_CHANNEL_4CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_EXTRACT_CHANNEL_4CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_EXTRACT_CHANNEL_4CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_EXTRACT_CHANNEL_4CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_EXTRACT_CHANNEL_4CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_EXTRACT_CHANNEL_4CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_EXTRACT_CHANNEL_4CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_EXTRACT_CHANNEL_4CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_EXTRACT_CHANNEL_4CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_EXTRACT_CHANNEL_4CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_EXTRACT_CHANNEL_4CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_EXTRACT_CHANNEL_4CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_EXTRACT_CHANNEL_4CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_EXTRACT_CHANNEL_4CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_EXTRACT_CHANNEL_4CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_EXTRACT_CHANNEL_4CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_EXTRACT_CHANNEL_4CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_EXTRACT_CHANNEL_4CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_EXTRACT_CHANNEL_4CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_EXTRACT_CHANNEL_4CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_EXTRACT_CHANNEL_4CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_EXTRACT_CHANNEL_4CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_EXTRACT_CHANNEL_4CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_EXTRACT_CHANNEL_4CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_EXTRACT_CHANNEL_4CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_EXTRACT_CHANNEL_4CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_EXTRACT_CHANNEL_4CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_EXTRACT_CHANNEL_4CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_EXTRACT_CHANNEL_4CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_EXTRACT_CHANNEL_4CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_EXTRACT_CHANNEL_4CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_EXTRACT_CHANNEL_4CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_EXTRACT_CHANNEL_4CH[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_EXTRACT_CHANNEL_4CH[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_EXTRACT_CHANNEL_4CH[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_EXTRACT_CHANNEL_4CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_EXTRACT_CHANNEL_4CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_EXTRACT_CHANNEL_4CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_EXTRACT_CHANNEL_4CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_EXTRACT_CHANNEL_4CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_EXTRACT_CHANNEL_4CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_EXTRACT_CHANNEL_4CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_EXTRACT_CHANNEL_4CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_EXTRACT_CHANNEL_4CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_EXTRACT_CHANNEL_4CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_EXTRACT_CHANNEL_4CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_EXTRACT_CHANNEL_4CH[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_EXTRACT_CHANNEL_4CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_EXTRACT_CHANNEL_4CH[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_EXTRACT_CHANNEL_4CH[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_EXTRACT_CHANNEL_4CH[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_EXTRACT_CHANNEL_4CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_EXTRACT_CHANNEL_4CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_EXTRACT_CHANNEL_4CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_EXTRACT_CHANNEL_4CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_EXTRACT_CHANNEL_4CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_EXTRACT_CHANNEL_4CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_EXTRACT_CHANNEL_4CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_EXTRACT_CHANNEL_4CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_EXTRACT_CHANNEL_4CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_EXTRACT_CHANNEL_4CH[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_EXTRACT_CHANNEL_4CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_EXTRACT_CHANNEL_4CH[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_EXTRACT_CHANNEL_4CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_EXTRACT_CHANNEL_4CH[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_EXTRACT_CHANNEL_4CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_EXTRACT_CHANNEL_4CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_EXTRACT_CHANNEL_4CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_EXTRACT_CHANNEL_4CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_EXTRACT_CHANNEL_4CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_EXTRACT_CHANNEL_4CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_EXTRACT_CHANNEL_4CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_EXTRACT_CHANNEL_4CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_EXTRACT_CHANNEL_4CH[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_EXTRACT_CHANNEL_4CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_EXTRACT_CHANNEL_4CH[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_EXTRACT_CHANNEL_4CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_EXTRACT_CHANNEL_4CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_EXTRACT_CHANNEL_4CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_EXTRACT_CHANNEL_4CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_EXTRACT_CHANNEL_4CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_EXTRACT_CHANNEL_4CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_EXTRACT_CHANNEL_4CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_EXTRACT_CHANNEL_4CH[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_EXTRACT_CHANNEL_4CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_EXTRACT_CHANNEL_4CH[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_EXTRACT_CHANNEL_4CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_EXTRACT_CHANNEL_4CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_EXTRACT_CHANNEL_4CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_EXTRACT_CHANNEL_4CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_EXTRACT_CHANNEL_4CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_EXTRACT_CHANNEL_4CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_EXTRACT_CHANNEL_4CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_EXTRACT_CHANNEL_4CH[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_EXTRACT_CHANNEL_4CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_EXTRACT_CHANNEL_4CH[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_EXTRACT_CHANNEL_4CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_EXTRACT_CHANNEL_4CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_EXTRACT_CHANNEL_4CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_EXTRACT_CHANNEL_4CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_EXTRACT_CHANNEL_4CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_EXTRACT_CHANNEL_4CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_EXTRACT_CHANNEL_4CH[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_EXTRACT_CHANNEL_4CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_EXTRACT_CHANNEL_4CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_EXTRACT_CHANNEL_4CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_EXTRACT_CHANNEL_4CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_EXTRACT_CHANNEL_4CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_EXTRACT_CHANNEL_4CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_EXTRACT_CHANNEL_4CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_EXTRACT_CHANNEL_4CH[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_EXTRACT_CHANNEL_4CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_EXTRACT_CHANNEL_4CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_EXTRACT_CHANNEL_4CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_EXTRACT_CHANNEL_4CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_EXTRACT_CHANNEL_4CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_EXTRACT_CHANNEL_4CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_EXTRACT_CHANNEL_4CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_EXTRACT_CHANNEL_4CH[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_EXTRACT_CHANNEL_4CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_EXTRACT_CHANNEL_4CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_EXTRACT_CHANNEL_4CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_EXTRACT_CHANNEL_4CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_EXTRACT_CHANNEL_4CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_EXTRACT_CHANNEL_4CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_EXTRACT_CHANNEL_4CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_EXTRACT_CHANNEL_4CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_EXTRACT_CHANNEL_4CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_EXTRACT_CHANNEL_4CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_EXTRACT_CHANNEL_4CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_EXTRACT_CHANNEL_4CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_EXTRACT_CHANNEL_4CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_EXTRACT_CHANNEL_4CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_EXTRACT_CHANNEL_4CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_EXTRACT_CHANNEL_4CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_EXTRACT_CHANNEL_4CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_EXTRACT_CHANNEL_4CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_EXTRACT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_EXTRACT_CHANNEL_4CH[] = {{0, 0}};

static acf_scenario gScenarioArray_EXTRACT_CHANNEL_4CH[] = {
{2, 1, 20, 16, 0, gScenarioBufferData0_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData0_EXTRACT_CHANNEL_4CH, 4},
{4, 1, 40, 16, 0, gScenarioBufferData1_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData1_EXTRACT_CHANNEL_4CH, 4},
{6, 1, 60, 16, 0, gScenarioBufferData2_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData2_EXTRACT_CHANNEL_4CH, 4},
{8, 1, 80, 16, 0, gScenarioBufferData3_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData3_EXTRACT_CHANNEL_4CH, 4},
{10, 1, 100, 16, 0, gScenarioBufferData4_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData4_EXTRACT_CHANNEL_4CH, 4},
{12, 1, 120, 16, 0, gScenarioBufferData5_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData5_EXTRACT_CHANNEL_4CH, 4},
{14, 1, 140, 16, 0, gScenarioBufferData6_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData6_EXTRACT_CHANNEL_4CH, 4},
{16, 1, 160, 16, 0, gScenarioBufferData7_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData7_EXTRACT_CHANNEL_4CH, 4},
{18, 1, 180, 16, 0, gScenarioBufferData8_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData8_EXTRACT_CHANNEL_4CH, 4},
{20, 1, 200, 16, 0, gScenarioBufferData9_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData9_EXTRACT_CHANNEL_4CH, 4},
{22, 1, 220, 16, 0, gScenarioBufferData10_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData10_EXTRACT_CHANNEL_4CH, 4},
{24, 1, 240, 16, 0, gScenarioBufferData11_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData11_EXTRACT_CHANNEL_4CH, 4},
{26, 1, 260, 16, 0, gScenarioBufferData12_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData12_EXTRACT_CHANNEL_4CH, 4},
{28, 1, 280, 16, 0, gScenarioBufferData13_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData13_EXTRACT_CHANNEL_4CH, 4},
{30, 1, 300, 16, 0, gScenarioBufferData14_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData14_EXTRACT_CHANNEL_4CH, 4},
{32, 1, 320, 16, 0, gScenarioBufferData15_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData15_EXTRACT_CHANNEL_4CH, 4},
{2, 2, 40, 16, 0, gScenarioBufferData16_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData16_EXTRACT_CHANNEL_4CH, 4},
{4, 2, 80, 16, 0, gScenarioBufferData17_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData17_EXTRACT_CHANNEL_4CH, 4},
{6, 2, 120, 16, 0, gScenarioBufferData18_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData18_EXTRACT_CHANNEL_4CH, 4},
{8, 2, 160, 16, 0, gScenarioBufferData19_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData19_EXTRACT_CHANNEL_4CH, 4},
{10, 2, 200, 16, 0, gScenarioBufferData20_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData20_EXTRACT_CHANNEL_4CH, 4},
{12, 2, 240, 16, 0, gScenarioBufferData21_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData21_EXTRACT_CHANNEL_4CH, 4},
{14, 2, 280, 16, 0, gScenarioBufferData22_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData22_EXTRACT_CHANNEL_4CH, 4},
{16, 2, 320, 16, 0, gScenarioBufferData23_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData23_EXTRACT_CHANNEL_4CH, 4},
{18, 2, 360, 16, 0, gScenarioBufferData24_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData24_EXTRACT_CHANNEL_4CH, 4},
{20, 2, 400, 16, 0, gScenarioBufferData25_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData25_EXTRACT_CHANNEL_4CH, 4},
{22, 2, 440, 16, 0, gScenarioBufferData26_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData26_EXTRACT_CHANNEL_4CH, 4},
{24, 2, 480, 16, 0, gScenarioBufferData27_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData27_EXTRACT_CHANNEL_4CH, 4},
{26, 2, 520, 16, 0, gScenarioBufferData28_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData28_EXTRACT_CHANNEL_4CH, 4},
{28, 2, 560, 16, 0, gScenarioBufferData29_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData29_EXTRACT_CHANNEL_4CH, 4},
{30, 2, 600, 16, 0, gScenarioBufferData30_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData30_EXTRACT_CHANNEL_4CH, 4},
{32, 2, 640, 16, 0, gScenarioBufferData31_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData31_EXTRACT_CHANNEL_4CH, 4},
{2, 3, 60, 16, 0, gScenarioBufferData32_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData32_EXTRACT_CHANNEL_4CH, 4},
{4, 3, 120, 16, 0, gScenarioBufferData33_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData33_EXTRACT_CHANNEL_4CH, 4},
{6, 3, 180, 16, 0, gScenarioBufferData34_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData34_EXTRACT_CHANNEL_4CH, 4},
{8, 3, 240, 16, 0, gScenarioBufferData35_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData35_EXTRACT_CHANNEL_4CH, 4},
{10, 3, 300, 16, 0, gScenarioBufferData36_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData36_EXTRACT_CHANNEL_4CH, 4},
{12, 3, 360, 16, 0, gScenarioBufferData37_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData37_EXTRACT_CHANNEL_4CH, 4},
{14, 3, 420, 16, 0, gScenarioBufferData38_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData38_EXTRACT_CHANNEL_4CH, 4},
{16, 3, 480, 16, 0, gScenarioBufferData39_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData39_EXTRACT_CHANNEL_4CH, 4},
{18, 3, 540, 16, 0, gScenarioBufferData40_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData40_EXTRACT_CHANNEL_4CH, 4},
{20, 3, 600, 16, 0, gScenarioBufferData41_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData41_EXTRACT_CHANNEL_4CH, 4},
{22, 3, 660, 16, 0, gScenarioBufferData42_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData42_EXTRACT_CHANNEL_4CH, 4},
{24, 3, 720, 16, 0, gScenarioBufferData43_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData43_EXTRACT_CHANNEL_4CH, 4},
{26, 3, 780, 16, 0, gScenarioBufferData44_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData44_EXTRACT_CHANNEL_4CH, 4},
{28, 3, 840, 16, 0, gScenarioBufferData45_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData45_EXTRACT_CHANNEL_4CH, 4},
{30, 3, 900, 16, 0, gScenarioBufferData46_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData46_EXTRACT_CHANNEL_4CH, 4},
{32, 3, 960, 16, 0, gScenarioBufferData47_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData47_EXTRACT_CHANNEL_4CH, 4},
{2, 4, 80, 16, 0, gScenarioBufferData48_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData48_EXTRACT_CHANNEL_4CH, 4},
{4, 4, 160, 16, 0, gScenarioBufferData49_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData49_EXTRACT_CHANNEL_4CH, 4},
{6, 4, 240, 16, 0, gScenarioBufferData50_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData50_EXTRACT_CHANNEL_4CH, 4},
{8, 4, 320, 16, 0, gScenarioBufferData51_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData51_EXTRACT_CHANNEL_4CH, 4},
{10, 4, 400, 16, 0, gScenarioBufferData52_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData52_EXTRACT_CHANNEL_4CH, 4},
{12, 4, 480, 16, 0, gScenarioBufferData53_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData53_EXTRACT_CHANNEL_4CH, 4},
{14, 4, 560, 16, 0, gScenarioBufferData54_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData54_EXTRACT_CHANNEL_4CH, 4},
{16, 4, 640, 16, 0, gScenarioBufferData55_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData55_EXTRACT_CHANNEL_4CH, 4},
{18, 4, 720, 16, 0, gScenarioBufferData56_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData56_EXTRACT_CHANNEL_4CH, 4},
{20, 4, 800, 16, 0, gScenarioBufferData57_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData57_EXTRACT_CHANNEL_4CH, 4},
{22, 4, 880, 16, 0, gScenarioBufferData58_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData58_EXTRACT_CHANNEL_4CH, 4},
{24, 4, 960, 16, 0, gScenarioBufferData59_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData59_EXTRACT_CHANNEL_4CH, 4},
{26, 4, 1040, 16, 0, gScenarioBufferData60_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData60_EXTRACT_CHANNEL_4CH, 4},
{28, 4, 1120, 16, 0, gScenarioBufferData61_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData61_EXTRACT_CHANNEL_4CH, 4},
{30, 4, 1200, 16, 0, gScenarioBufferData62_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData62_EXTRACT_CHANNEL_4CH, 4},
{32, 4, 1280, 16, 0, gScenarioBufferData63_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData63_EXTRACT_CHANNEL_4CH, 4},
{2, 5, 100, 16, 0, gScenarioBufferData64_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData64_EXTRACT_CHANNEL_4CH, 4},
{4, 5, 200, 16, 0, gScenarioBufferData65_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData65_EXTRACT_CHANNEL_4CH, 4},
{6, 5, 300, 16, 0, gScenarioBufferData66_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData66_EXTRACT_CHANNEL_4CH, 4},
{8, 5, 400, 16, 0, gScenarioBufferData67_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData67_EXTRACT_CHANNEL_4CH, 4},
{10, 5, 500, 16, 0, gScenarioBufferData68_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData68_EXTRACT_CHANNEL_4CH, 4},
{12, 5, 600, 16, 0, gScenarioBufferData69_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData69_EXTRACT_CHANNEL_4CH, 4},
{14, 5, 700, 16, 0, gScenarioBufferData70_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData70_EXTRACT_CHANNEL_4CH, 4},
{16, 5, 800, 16, 0, gScenarioBufferData71_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData71_EXTRACT_CHANNEL_4CH, 4},
{18, 5, 900, 16, 0, gScenarioBufferData72_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData72_EXTRACT_CHANNEL_4CH, 4},
{20, 5, 1000, 16, 0, gScenarioBufferData73_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData73_EXTRACT_CHANNEL_4CH, 4},
{22, 5, 1100, 16, 0, gScenarioBufferData74_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData74_EXTRACT_CHANNEL_4CH, 4},
{24, 5, 1200, 16, 0, gScenarioBufferData75_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData75_EXTRACT_CHANNEL_4CH, 4},
{26, 5, 1300, 16, 0, gScenarioBufferData76_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData76_EXTRACT_CHANNEL_4CH, 4},
{28, 5, 1400, 16, 0, gScenarioBufferData77_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData77_EXTRACT_CHANNEL_4CH, 4},
{30, 5, 1500, 16, 0, gScenarioBufferData78_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData78_EXTRACT_CHANNEL_4CH, 4},
{32, 5, 1600, 16, 0, gScenarioBufferData79_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData79_EXTRACT_CHANNEL_4CH, 4},
{2, 6, 120, 16, 0, gScenarioBufferData80_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData80_EXTRACT_CHANNEL_4CH, 4},
{4, 6, 240, 16, 0, gScenarioBufferData81_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData81_EXTRACT_CHANNEL_4CH, 4},
{6, 6, 360, 16, 0, gScenarioBufferData82_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData82_EXTRACT_CHANNEL_4CH, 4},
{8, 6, 480, 16, 0, gScenarioBufferData83_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData83_EXTRACT_CHANNEL_4CH, 4},
{10, 6, 600, 16, 0, gScenarioBufferData84_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData84_EXTRACT_CHANNEL_4CH, 4},
{12, 6, 720, 16, 0, gScenarioBufferData85_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData85_EXTRACT_CHANNEL_4CH, 4},
{14, 6, 840, 16, 0, gScenarioBufferData86_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData86_EXTRACT_CHANNEL_4CH, 4},
{16, 6, 960, 16, 0, gScenarioBufferData87_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData87_EXTRACT_CHANNEL_4CH, 4},
{18, 6, 1080, 16, 0, gScenarioBufferData88_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData88_EXTRACT_CHANNEL_4CH, 4},
{20, 6, 1200, 16, 0, gScenarioBufferData89_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData89_EXTRACT_CHANNEL_4CH, 4},
{22, 6, 1320, 16, 0, gScenarioBufferData90_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData90_EXTRACT_CHANNEL_4CH, 4},
{24, 6, 1440, 16, 0, gScenarioBufferData91_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData91_EXTRACT_CHANNEL_4CH, 4},
{26, 6, 1560, 16, 0, gScenarioBufferData92_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData92_EXTRACT_CHANNEL_4CH, 4},
{28, 6, 1680, 16, 0, gScenarioBufferData93_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData93_EXTRACT_CHANNEL_4CH, 4},
{30, 6, 1800, 16, 0, gScenarioBufferData94_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData94_EXTRACT_CHANNEL_4CH, 4},
{32, 6, 1920, 16, 0, gScenarioBufferData95_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData95_EXTRACT_CHANNEL_4CH, 4},
{2, 8, 160, 16, 0, gScenarioBufferData96_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData96_EXTRACT_CHANNEL_4CH, 4},
{4, 8, 320, 16, 0, gScenarioBufferData97_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData97_EXTRACT_CHANNEL_4CH, 4},
{6, 8, 480, 16, 0, gScenarioBufferData98_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData98_EXTRACT_CHANNEL_4CH, 4},
{8, 8, 640, 16, 0, gScenarioBufferData99_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData99_EXTRACT_CHANNEL_4CH, 4},
{10, 8, 800, 16, 0, gScenarioBufferData100_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData100_EXTRACT_CHANNEL_4CH, 4},
{12, 8, 960, 16, 0, gScenarioBufferData101_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData101_EXTRACT_CHANNEL_4CH, 4},
{14, 8, 1120, 16, 0, gScenarioBufferData102_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData102_EXTRACT_CHANNEL_4CH, 4},
{16, 8, 1280, 16, 0, gScenarioBufferData103_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData103_EXTRACT_CHANNEL_4CH, 4},
{18, 8, 1440, 16, 0, gScenarioBufferData104_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData104_EXTRACT_CHANNEL_4CH, 4},
{20, 8, 1600, 16, 0, gScenarioBufferData105_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData105_EXTRACT_CHANNEL_4CH, 4},
{22, 8, 1760, 16, 0, gScenarioBufferData106_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData106_EXTRACT_CHANNEL_4CH, 4},
{24, 8, 1920, 16, 0, gScenarioBufferData107_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData107_EXTRACT_CHANNEL_4CH, 4},
{26, 8, 2080, 16, 0, gScenarioBufferData108_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData108_EXTRACT_CHANNEL_4CH, 4},
{28, 8, 2240, 16, 0, gScenarioBufferData109_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData109_EXTRACT_CHANNEL_4CH, 4},
{30, 8, 2400, 16, 0, gScenarioBufferData110_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData110_EXTRACT_CHANNEL_4CH, 4},
{32, 8, 2560, 16, 0, gScenarioBufferData111_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData111_EXTRACT_CHANNEL_4CH, 4},
{2, 10, 200, 16, 0, gScenarioBufferData112_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData112_EXTRACT_CHANNEL_4CH, 4},
{4, 10, 400, 16, 0, gScenarioBufferData113_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData113_EXTRACT_CHANNEL_4CH, 4},
{6, 10, 600, 16, 0, gScenarioBufferData114_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData114_EXTRACT_CHANNEL_4CH, 4},
{8, 10, 800, 16, 0, gScenarioBufferData115_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData115_EXTRACT_CHANNEL_4CH, 4},
{10, 10, 1000, 16, 0, gScenarioBufferData116_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData116_EXTRACT_CHANNEL_4CH, 4},
{12, 10, 1200, 16, 0, gScenarioBufferData117_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData117_EXTRACT_CHANNEL_4CH, 4},
{14, 10, 1400, 16, 0, gScenarioBufferData118_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData118_EXTRACT_CHANNEL_4CH, 4},
{16, 10, 1600, 16, 0, gScenarioBufferData119_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData119_EXTRACT_CHANNEL_4CH, 4},
{18, 10, 1800, 16, 0, gScenarioBufferData120_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData120_EXTRACT_CHANNEL_4CH, 4},
{20, 10, 2000, 16, 0, gScenarioBufferData121_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData121_EXTRACT_CHANNEL_4CH, 4},
{22, 10, 2200, 16, 0, gScenarioBufferData122_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData122_EXTRACT_CHANNEL_4CH, 4},
{24, 10, 2400, 16, 0, gScenarioBufferData123_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData123_EXTRACT_CHANNEL_4CH, 4},
{26, 10, 2600, 16, 0, gScenarioBufferData124_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData124_EXTRACT_CHANNEL_4CH, 4},
{28, 10, 2800, 16, 0, gScenarioBufferData125_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData125_EXTRACT_CHANNEL_4CH, 4},
{30, 10, 3000, 16, 0, gScenarioBufferData126_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData126_EXTRACT_CHANNEL_4CH, 4},
{32, 10, 3200, 16, 0, gScenarioBufferData127_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData127_EXTRACT_CHANNEL_4CH, 4},
{2, 12, 240, 16, 0, gScenarioBufferData128_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData128_EXTRACT_CHANNEL_4CH, 4},
{4, 12, 480, 16, 0, gScenarioBufferData129_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData129_EXTRACT_CHANNEL_4CH, 4},
{6, 12, 720, 16, 0, gScenarioBufferData130_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData130_EXTRACT_CHANNEL_4CH, 4},
{8, 12, 960, 16, 0, gScenarioBufferData131_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData131_EXTRACT_CHANNEL_4CH, 4},
{10, 12, 1200, 16, 0, gScenarioBufferData132_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData132_EXTRACT_CHANNEL_4CH, 4},
{12, 12, 1440, 16, 0, gScenarioBufferData133_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData133_EXTRACT_CHANNEL_4CH, 4},
{14, 12, 1680, 16, 0, gScenarioBufferData134_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData134_EXTRACT_CHANNEL_4CH, 4},
{16, 12, 1920, 16, 0, gScenarioBufferData135_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData135_EXTRACT_CHANNEL_4CH, 4},
{18, 12, 2160, 16, 0, gScenarioBufferData136_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData136_EXTRACT_CHANNEL_4CH, 4},
{20, 12, 2400, 16, 0, gScenarioBufferData137_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData137_EXTRACT_CHANNEL_4CH, 4},
{22, 12, 2640, 16, 0, gScenarioBufferData138_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData138_EXTRACT_CHANNEL_4CH, 4},
{24, 12, 2880, 16, 0, gScenarioBufferData139_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData139_EXTRACT_CHANNEL_4CH, 4},
{26, 12, 3120, 16, 0, gScenarioBufferData140_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData140_EXTRACT_CHANNEL_4CH, 4},
{28, 12, 3360, 16, 0, gScenarioBufferData141_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData141_EXTRACT_CHANNEL_4CH, 4},
{30, 12, 3600, 16, 0, gScenarioBufferData142_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData142_EXTRACT_CHANNEL_4CH, 4},
{32, 12, 3840, 16, 0, gScenarioBufferData143_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData143_EXTRACT_CHANNEL_4CH, 4},
{2, 14, 280, 16, 0, gScenarioBufferData144_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData144_EXTRACT_CHANNEL_4CH, 4},
{4, 14, 560, 16, 0, gScenarioBufferData145_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData145_EXTRACT_CHANNEL_4CH, 4},
{6, 14, 840, 16, 0, gScenarioBufferData146_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData146_EXTRACT_CHANNEL_4CH, 4},
{8, 14, 1120, 16, 0, gScenarioBufferData147_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData147_EXTRACT_CHANNEL_4CH, 4},
{10, 14, 1400, 16, 0, gScenarioBufferData148_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData148_EXTRACT_CHANNEL_4CH, 4},
{12, 14, 1680, 16, 0, gScenarioBufferData149_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData149_EXTRACT_CHANNEL_4CH, 4},
{14, 14, 1960, 16, 0, gScenarioBufferData150_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData150_EXTRACT_CHANNEL_4CH, 4},
{16, 14, 2240, 16, 0, gScenarioBufferData151_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData151_EXTRACT_CHANNEL_4CH, 4},
{18, 14, 2520, 16, 0, gScenarioBufferData152_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData152_EXTRACT_CHANNEL_4CH, 4},
{20, 14, 2800, 16, 0, gScenarioBufferData153_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData153_EXTRACT_CHANNEL_4CH, 4},
{22, 14, 3080, 16, 0, gScenarioBufferData154_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData154_EXTRACT_CHANNEL_4CH, 4},
{24, 14, 3360, 16, 0, gScenarioBufferData155_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData155_EXTRACT_CHANNEL_4CH, 4},
{26, 14, 3640, 16, 0, gScenarioBufferData156_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData156_EXTRACT_CHANNEL_4CH, 4},
{2, 16, 320, 16, 0, gScenarioBufferData157_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData157_EXTRACT_CHANNEL_4CH, 4},
{4, 16, 640, 16, 0, gScenarioBufferData158_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData158_EXTRACT_CHANNEL_4CH, 4},
{6, 16, 960, 16, 0, gScenarioBufferData159_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData159_EXTRACT_CHANNEL_4CH, 4},
{8, 16, 1280, 16, 0, gScenarioBufferData160_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData160_EXTRACT_CHANNEL_4CH, 4},
{10, 16, 1600, 16, 0, gScenarioBufferData161_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData161_EXTRACT_CHANNEL_4CH, 4},
{12, 16, 1920, 16, 0, gScenarioBufferData162_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData162_EXTRACT_CHANNEL_4CH, 4},
{14, 16, 2240, 16, 0, gScenarioBufferData163_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData163_EXTRACT_CHANNEL_4CH, 4},
{16, 16, 2560, 16, 0, gScenarioBufferData164_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData164_EXTRACT_CHANNEL_4CH, 4},
{18, 16, 2880, 16, 0, gScenarioBufferData165_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData165_EXTRACT_CHANNEL_4CH, 4},
{20, 16, 3200, 16, 0, gScenarioBufferData166_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData166_EXTRACT_CHANNEL_4CH, 4},
{22, 16, 3520, 16, 0, gScenarioBufferData167_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData167_EXTRACT_CHANNEL_4CH, 4},
{24, 16, 3840, 16, 0, gScenarioBufferData168_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData168_EXTRACT_CHANNEL_4CH, 4},
{2, 18, 360, 16, 0, gScenarioBufferData169_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData169_EXTRACT_CHANNEL_4CH, 4},
{4, 18, 720, 16, 0, gScenarioBufferData170_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData170_EXTRACT_CHANNEL_4CH, 4},
{6, 18, 1080, 16, 0, gScenarioBufferData171_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData171_EXTRACT_CHANNEL_4CH, 4},
{8, 18, 1440, 16, 0, gScenarioBufferData172_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData172_EXTRACT_CHANNEL_4CH, 4},
{10, 18, 1800, 16, 0, gScenarioBufferData173_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData173_EXTRACT_CHANNEL_4CH, 4},
{12, 18, 2160, 16, 0, gScenarioBufferData174_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData174_EXTRACT_CHANNEL_4CH, 4},
{14, 18, 2520, 16, 0, gScenarioBufferData175_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData175_EXTRACT_CHANNEL_4CH, 4},
{16, 18, 2880, 16, 0, gScenarioBufferData176_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData176_EXTRACT_CHANNEL_4CH, 4},
{18, 18, 3240, 16, 0, gScenarioBufferData177_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData177_EXTRACT_CHANNEL_4CH, 4},
{20, 18, 3600, 16, 0, gScenarioBufferData178_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData178_EXTRACT_CHANNEL_4CH, 4},
{2, 20, 400, 16, 0, gScenarioBufferData179_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData179_EXTRACT_CHANNEL_4CH, 4},
{4, 20, 800, 16, 0, gScenarioBufferData180_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData180_EXTRACT_CHANNEL_4CH, 4},
{6, 20, 1200, 16, 0, gScenarioBufferData181_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData181_EXTRACT_CHANNEL_4CH, 4},
{8, 20, 1600, 16, 0, gScenarioBufferData182_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData182_EXTRACT_CHANNEL_4CH, 4},
{10, 20, 2000, 16, 0, gScenarioBufferData183_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData183_EXTRACT_CHANNEL_4CH, 4},
{12, 20, 2400, 16, 0, gScenarioBufferData184_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData184_EXTRACT_CHANNEL_4CH, 4},
{14, 20, 2800, 16, 0, gScenarioBufferData185_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData185_EXTRACT_CHANNEL_4CH, 4},
{16, 20, 3200, 16, 0, gScenarioBufferData186_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData186_EXTRACT_CHANNEL_4CH, 4},
{18, 20, 3600, 16, 0, gScenarioBufferData187_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData187_EXTRACT_CHANNEL_4CH, 4},
{2, 22, 440, 16, 0, gScenarioBufferData188_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData188_EXTRACT_CHANNEL_4CH, 4},
{4, 22, 880, 16, 0, gScenarioBufferData189_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData189_EXTRACT_CHANNEL_4CH, 4},
{6, 22, 1320, 16, 0, gScenarioBufferData190_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData190_EXTRACT_CHANNEL_4CH, 4},
{8, 22, 1760, 16, 0, gScenarioBufferData191_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData191_EXTRACT_CHANNEL_4CH, 4},
{10, 22, 2200, 16, 0, gScenarioBufferData192_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData192_EXTRACT_CHANNEL_4CH, 4},
{12, 22, 2640, 16, 0, gScenarioBufferData193_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData193_EXTRACT_CHANNEL_4CH, 4},
{14, 22, 3080, 16, 0, gScenarioBufferData194_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData194_EXTRACT_CHANNEL_4CH, 4},
{16, 22, 3520, 16, 0, gScenarioBufferData195_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData195_EXTRACT_CHANNEL_4CH, 4},
{2, 24, 480, 16, 0, gScenarioBufferData196_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData196_EXTRACT_CHANNEL_4CH, 4},
{4, 24, 960, 16, 0, gScenarioBufferData197_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData197_EXTRACT_CHANNEL_4CH, 4},
{6, 24, 1440, 16, 0, gScenarioBufferData198_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData198_EXTRACT_CHANNEL_4CH, 4},
{8, 24, 1920, 16, 0, gScenarioBufferData199_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData199_EXTRACT_CHANNEL_4CH, 4},
{10, 24, 2400, 16, 0, gScenarioBufferData200_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData200_EXTRACT_CHANNEL_4CH, 4},
{12, 24, 2880, 16, 0, gScenarioBufferData201_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData201_EXTRACT_CHANNEL_4CH, 4},
{14, 24, 3360, 16, 0, gScenarioBufferData202_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData202_EXTRACT_CHANNEL_4CH, 4},
{16, 24, 3840, 16, 0, gScenarioBufferData203_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData203_EXTRACT_CHANNEL_4CH, 4},
{2, 26, 520, 16, 0, gScenarioBufferData204_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData204_EXTRACT_CHANNEL_4CH, 4},
{4, 26, 1040, 16, 0, gScenarioBufferData205_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData205_EXTRACT_CHANNEL_4CH, 4},
{6, 26, 1560, 16, 0, gScenarioBufferData206_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData206_EXTRACT_CHANNEL_4CH, 4},
{8, 26, 2080, 16, 0, gScenarioBufferData207_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData207_EXTRACT_CHANNEL_4CH, 4},
{10, 26, 2600, 16, 0, gScenarioBufferData208_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData208_EXTRACT_CHANNEL_4CH, 4},
{12, 26, 3120, 16, 0, gScenarioBufferData209_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData209_EXTRACT_CHANNEL_4CH, 4},
{14, 26, 3640, 16, 0, gScenarioBufferData210_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData210_EXTRACT_CHANNEL_4CH, 4},
{2, 28, 560, 16, 0, gScenarioBufferData211_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData211_EXTRACT_CHANNEL_4CH, 4},
{4, 28, 1120, 16, 0, gScenarioBufferData212_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData212_EXTRACT_CHANNEL_4CH, 4},
{6, 28, 1680, 16, 0, gScenarioBufferData213_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData213_EXTRACT_CHANNEL_4CH, 4},
{8, 28, 2240, 16, 0, gScenarioBufferData214_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData214_EXTRACT_CHANNEL_4CH, 4},
{10, 28, 2800, 16, 0, gScenarioBufferData215_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData215_EXTRACT_CHANNEL_4CH, 4},
{12, 28, 3360, 16, 0, gScenarioBufferData216_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData216_EXTRACT_CHANNEL_4CH, 4},
{2, 30, 600, 16, 0, gScenarioBufferData217_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData217_EXTRACT_CHANNEL_4CH, 4},
{4, 30, 1200, 16, 0, gScenarioBufferData218_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData218_EXTRACT_CHANNEL_4CH, 4},
{6, 30, 1800, 16, 0, gScenarioBufferData219_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData219_EXTRACT_CHANNEL_4CH, 4},
{8, 30, 2400, 16, 0, gScenarioBufferData220_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData220_EXTRACT_CHANNEL_4CH, 4},
{10, 30, 3000, 16, 0, gScenarioBufferData221_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData221_EXTRACT_CHANNEL_4CH, 4},
{12, 30, 3600, 16, 0, gScenarioBufferData222_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData222_EXTRACT_CHANNEL_4CH, 4},
{2, 32, 640, 16, 0, gScenarioBufferData223_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData223_EXTRACT_CHANNEL_4CH, 4},
{4, 32, 1280, 16, 0, gScenarioBufferData224_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData224_EXTRACT_CHANNEL_4CH, 4},
{6, 32, 1920, 16, 0, gScenarioBufferData225_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData225_EXTRACT_CHANNEL_4CH, 4},
{8, 32, 2560, 16, 0, gScenarioBufferData226_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData226_EXTRACT_CHANNEL_4CH, 4},
{10, 32, 3200, 16, 0, gScenarioBufferData227_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData227_EXTRACT_CHANNEL_4CH, 4},
{12, 32, 3840, 16, 0, gScenarioBufferData228_EXTRACT_CHANNEL_4CH, 48, gScenarioKernelData228_EXTRACT_CHANNEL_4CH, 4}
};

static acf_scenario_list gScenarioList_EXTRACT_CHANNEL_4CH = {
229, //number of scenarios
gScenarioArray_EXTRACT_CHANNEL_4CH};
//**************************************************************

class EXTRACT_CHANNEL_4CH : public ACF_Process_APU
{

public:
   EXTRACT_CHANNEL_4CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("EXTRACT_CHANNEL_4CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("EXTRACT_CHANNEL_4CH",
                                        EXTRACT_CHANNEL_4CH_LOAD_SEGMENTS,
                                        EXTRACT_CHANNEL_4CH_LOAD_PMEM, EXTRACT_CHANNEL_4CH_LOAD_PMEM_SIZE,
                                        EXTRACT_CHANNEL_4CH_LOAD_DMEM, EXTRACT_CHANNEL_4CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(EXTRACT_CHANNEL_4CH_APEX_LOG_BUFFER, EXTRACT_CHANNEL_4CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 4, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_EXTRACT_CHANNEL_4CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_EXTRACT_CHANNEL_4CH

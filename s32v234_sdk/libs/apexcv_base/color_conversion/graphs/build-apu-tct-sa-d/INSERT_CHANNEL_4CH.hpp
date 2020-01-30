#ifndef _ACF_PROCESS_APU_INSERT_CHANNEL_4CH
#define _ACF_PROCESS_APU_INSERT_CHANNEL_4CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INSERT_CHANNEL_4CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INSERT_CHANNEL_4CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INSERT_CHANNEL_4CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INSERT_CHANNEL_4CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INSERT_CHANNEL_4CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INSERT_CHANNEL_4CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INSERT_CHANNEL_4CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INSERT_CHANNEL_4CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INSERT_CHANNEL_4CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INSERT_CHANNEL_4CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INSERT_CHANNEL_4CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INSERT_CHANNEL_4CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INSERT_CHANNEL_4CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INSERT_CHANNEL_4CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INSERT_CHANNEL_4CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INSERT_CHANNEL_4CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INSERT_CHANNEL_4CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INSERT_CHANNEL_4CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INSERT_CHANNEL_4CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INSERT_CHANNEL_4CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INSERT_CHANNEL_4CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INSERT_CHANNEL_4CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INSERT_CHANNEL_4CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INSERT_CHANNEL_4CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INSERT_CHANNEL_4CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INSERT_CHANNEL_4CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INSERT_CHANNEL_4CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INSERT_CHANNEL_4CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INSERT_CHANNEL_4CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INSERT_CHANNEL_4CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INSERT_CHANNEL_4CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INSERT_CHANNEL_4CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INSERT_CHANNEL_4CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INSERT_CHANNEL_4CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INSERT_CHANNEL_4CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INSERT_CHANNEL_4CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INSERT_CHANNEL_4CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INSERT_CHANNEL_4CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INSERT_CHANNEL_4CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INSERT_CHANNEL_4CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INSERT_CHANNEL_4CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INSERT_CHANNEL_4CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INSERT_CHANNEL_4CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INSERT_CHANNEL_4CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INSERT_CHANNEL_4CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INSERT_CHANNEL_4CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_INSERT_CHANNEL_4CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_INSERT_CHANNEL_4CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_INSERT_CHANNEL_4CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_INSERT_CHANNEL_4CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_INSERT_CHANNEL_4CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_INSERT_CHANNEL_4CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_INSERT_CHANNEL_4CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_INSERT_CHANNEL_4CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_INSERT_CHANNEL_4CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_INSERT_CHANNEL_4CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_INSERT_CHANNEL_4CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_INSERT_CHANNEL_4CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_INSERT_CHANNEL_4CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_INSERT_CHANNEL_4CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_INSERT_CHANNEL_4CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_INSERT_CHANNEL_4CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_INSERT_CHANNEL_4CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_INSERT_CHANNEL_4CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_INSERT_CHANNEL_4CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_INSERT_CHANNEL_4CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_INSERT_CHANNEL_4CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_INSERT_CHANNEL_4CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_INSERT_CHANNEL_4CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_INSERT_CHANNEL_4CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_INSERT_CHANNEL_4CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_INSERT_CHANNEL_4CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_INSERT_CHANNEL_4CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_INSERT_CHANNEL_4CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_INSERT_CHANNEL_4CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_INSERT_CHANNEL_4CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_INSERT_CHANNEL_4CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_INSERT_CHANNEL_4CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_INSERT_CHANNEL_4CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_INSERT_CHANNEL_4CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_INSERT_CHANNEL_4CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_INSERT_CHANNEL_4CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_INSERT_CHANNEL_4CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_INSERT_CHANNEL_4CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_INSERT_CHANNEL_4CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_INSERT_CHANNEL_4CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_INSERT_CHANNEL_4CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_INSERT_CHANNEL_4CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_INSERT_CHANNEL_4CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_INSERT_CHANNEL_4CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_INSERT_CHANNEL_4CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_INSERT_CHANNEL_4CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_INSERT_CHANNEL_4CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_INSERT_CHANNEL_4CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_INSERT_CHANNEL_4CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_INSERT_CHANNEL_4CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_INSERT_CHANNEL_4CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_INSERT_CHANNEL_4CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_INSERT_CHANNEL_4CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_INSERT_CHANNEL_4CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_INSERT_CHANNEL_4CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_INSERT_CHANNEL_4CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_INSERT_CHANNEL_4CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_INSERT_CHANNEL_4CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_INSERT_CHANNEL_4CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_INSERT_CHANNEL_4CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_INSERT_CHANNEL_4CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_INSERT_CHANNEL_4CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_INSERT_CHANNEL_4CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_INSERT_CHANNEL_4CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_INSERT_CHANNEL_4CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_INSERT_CHANNEL_4CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_INSERT_CHANNEL_4CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_INSERT_CHANNEL_4CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_INSERT_CHANNEL_4CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_INSERT_CHANNEL_4CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_INSERT_CHANNEL_4CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_INSERT_CHANNEL_4CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_INSERT_CHANNEL_4CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_INSERT_CHANNEL_4CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_INSERT_CHANNEL_4CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_INSERT_CHANNEL_4CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_INSERT_CHANNEL_4CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_INSERT_CHANNEL_4CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_INSERT_CHANNEL_4CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_INSERT_CHANNEL_4CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_INSERT_CHANNEL_4CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_INSERT_CHANNEL_4CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_INSERT_CHANNEL_4CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_INSERT_CHANNEL_4CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_INSERT_CHANNEL_4CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_INSERT_CHANNEL_4CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_INSERT_CHANNEL_4CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_INSERT_CHANNEL_4CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_INSERT_CHANNEL_4CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_INSERT_CHANNEL_4CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_INSERT_CHANNEL_4CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_INSERT_CHANNEL_4CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_INSERT_CHANNEL_4CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_INSERT_CHANNEL_4CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_INSERT_CHANNEL_4CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_INSERT_CHANNEL_4CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_INSERT_CHANNEL_4CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_INSERT_CHANNEL_4CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_INSERT_CHANNEL_4CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_INSERT_CHANNEL_4CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_INSERT_CHANNEL_4CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_INSERT_CHANNEL_4CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_INSERT_CHANNEL_4CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_INSERT_CHANNEL_4CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_INSERT_CHANNEL_4CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_INSERT_CHANNEL_4CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_INSERT_CHANNEL_4CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_INSERT_CHANNEL_4CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_INSERT_CHANNEL_4CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_INSERT_CHANNEL_4CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_INSERT_CHANNEL_4CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_INSERT_CHANNEL_4CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_INSERT_CHANNEL_4CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_INSERT_CHANNEL_4CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_INSERT_CHANNEL_4CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_INSERT_CHANNEL_4CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_INSERT_CHANNEL_4CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_INSERT_CHANNEL_4CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_INSERT_CHANNEL_4CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_INSERT_CHANNEL_4CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_INSERT_CHANNEL_4CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_INSERT_CHANNEL_4CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_INSERT_CHANNEL_4CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_INSERT_CHANNEL_4CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_INSERT_CHANNEL_4CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_INSERT_CHANNEL_4CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_INSERT_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_INSERT_CHANNEL_4CH[] = {{0, 0}};

static acf_scenario gScenarioArray_INSERT_CHANNEL_4CH[] = {
{2, 1, 36, 16, 0, gScenarioBufferData0_INSERT_CHANNEL_4CH, 64, gScenarioKernelData0_INSERT_CHANNEL_4CH, 4},
{4, 1, 72, 16, 0, gScenarioBufferData1_INSERT_CHANNEL_4CH, 64, gScenarioKernelData1_INSERT_CHANNEL_4CH, 4},
{6, 1, 108, 16, 0, gScenarioBufferData2_INSERT_CHANNEL_4CH, 64, gScenarioKernelData2_INSERT_CHANNEL_4CH, 4},
{8, 1, 144, 16, 0, gScenarioBufferData3_INSERT_CHANNEL_4CH, 64, gScenarioKernelData3_INSERT_CHANNEL_4CH, 4},
{10, 1, 180, 16, 0, gScenarioBufferData4_INSERT_CHANNEL_4CH, 64, gScenarioKernelData4_INSERT_CHANNEL_4CH, 4},
{12, 1, 216, 16, 0, gScenarioBufferData5_INSERT_CHANNEL_4CH, 64, gScenarioKernelData5_INSERT_CHANNEL_4CH, 4},
{14, 1, 252, 16, 0, gScenarioBufferData6_INSERT_CHANNEL_4CH, 64, gScenarioKernelData6_INSERT_CHANNEL_4CH, 4},
{16, 1, 288, 16, 0, gScenarioBufferData7_INSERT_CHANNEL_4CH, 64, gScenarioKernelData7_INSERT_CHANNEL_4CH, 4},
{18, 1, 324, 16, 0, gScenarioBufferData8_INSERT_CHANNEL_4CH, 64, gScenarioKernelData8_INSERT_CHANNEL_4CH, 4},
{20, 1, 360, 16, 0, gScenarioBufferData9_INSERT_CHANNEL_4CH, 64, gScenarioKernelData9_INSERT_CHANNEL_4CH, 4},
{22, 1, 396, 16, 0, gScenarioBufferData10_INSERT_CHANNEL_4CH, 64, gScenarioKernelData10_INSERT_CHANNEL_4CH, 4},
{24, 1, 432, 16, 0, gScenarioBufferData11_INSERT_CHANNEL_4CH, 64, gScenarioKernelData11_INSERT_CHANNEL_4CH, 4},
{26, 1, 468, 16, 0, gScenarioBufferData12_INSERT_CHANNEL_4CH, 64, gScenarioKernelData12_INSERT_CHANNEL_4CH, 4},
{28, 1, 504, 16, 0, gScenarioBufferData13_INSERT_CHANNEL_4CH, 64, gScenarioKernelData13_INSERT_CHANNEL_4CH, 4},
{30, 1, 540, 16, 0, gScenarioBufferData14_INSERT_CHANNEL_4CH, 64, gScenarioKernelData14_INSERT_CHANNEL_4CH, 4},
{32, 1, 576, 16, 0, gScenarioBufferData15_INSERT_CHANNEL_4CH, 64, gScenarioKernelData15_INSERT_CHANNEL_4CH, 4},
{2, 2, 72, 16, 0, gScenarioBufferData16_INSERT_CHANNEL_4CH, 64, gScenarioKernelData16_INSERT_CHANNEL_4CH, 4},
{4, 2, 144, 16, 0, gScenarioBufferData17_INSERT_CHANNEL_4CH, 64, gScenarioKernelData17_INSERT_CHANNEL_4CH, 4},
{6, 2, 216, 16, 0, gScenarioBufferData18_INSERT_CHANNEL_4CH, 64, gScenarioKernelData18_INSERT_CHANNEL_4CH, 4},
{8, 2, 288, 16, 0, gScenarioBufferData19_INSERT_CHANNEL_4CH, 64, gScenarioKernelData19_INSERT_CHANNEL_4CH, 4},
{10, 2, 360, 16, 0, gScenarioBufferData20_INSERT_CHANNEL_4CH, 64, gScenarioKernelData20_INSERT_CHANNEL_4CH, 4},
{12, 2, 432, 16, 0, gScenarioBufferData21_INSERT_CHANNEL_4CH, 64, gScenarioKernelData21_INSERT_CHANNEL_4CH, 4},
{14, 2, 504, 16, 0, gScenarioBufferData22_INSERT_CHANNEL_4CH, 64, gScenarioKernelData22_INSERT_CHANNEL_4CH, 4},
{16, 2, 576, 16, 0, gScenarioBufferData23_INSERT_CHANNEL_4CH, 64, gScenarioKernelData23_INSERT_CHANNEL_4CH, 4},
{18, 2, 648, 16, 0, gScenarioBufferData24_INSERT_CHANNEL_4CH, 64, gScenarioKernelData24_INSERT_CHANNEL_4CH, 4},
{20, 2, 720, 16, 0, gScenarioBufferData25_INSERT_CHANNEL_4CH, 64, gScenarioKernelData25_INSERT_CHANNEL_4CH, 4},
{22, 2, 792, 16, 0, gScenarioBufferData26_INSERT_CHANNEL_4CH, 64, gScenarioKernelData26_INSERT_CHANNEL_4CH, 4},
{24, 2, 864, 16, 0, gScenarioBufferData27_INSERT_CHANNEL_4CH, 64, gScenarioKernelData27_INSERT_CHANNEL_4CH, 4},
{26, 2, 936, 16, 0, gScenarioBufferData28_INSERT_CHANNEL_4CH, 64, gScenarioKernelData28_INSERT_CHANNEL_4CH, 4},
{28, 2, 1008, 16, 0, gScenarioBufferData29_INSERT_CHANNEL_4CH, 64, gScenarioKernelData29_INSERT_CHANNEL_4CH, 4},
{30, 2, 1080, 16, 0, gScenarioBufferData30_INSERT_CHANNEL_4CH, 64, gScenarioKernelData30_INSERT_CHANNEL_4CH, 4},
{32, 2, 1152, 16, 0, gScenarioBufferData31_INSERT_CHANNEL_4CH, 64, gScenarioKernelData31_INSERT_CHANNEL_4CH, 4},
{2, 3, 108, 16, 0, gScenarioBufferData32_INSERT_CHANNEL_4CH, 64, gScenarioKernelData32_INSERT_CHANNEL_4CH, 4},
{4, 3, 216, 16, 0, gScenarioBufferData33_INSERT_CHANNEL_4CH, 64, gScenarioKernelData33_INSERT_CHANNEL_4CH, 4},
{6, 3, 324, 16, 0, gScenarioBufferData34_INSERT_CHANNEL_4CH, 64, gScenarioKernelData34_INSERT_CHANNEL_4CH, 4},
{8, 3, 432, 16, 0, gScenarioBufferData35_INSERT_CHANNEL_4CH, 64, gScenarioKernelData35_INSERT_CHANNEL_4CH, 4},
{10, 3, 540, 16, 0, gScenarioBufferData36_INSERT_CHANNEL_4CH, 64, gScenarioKernelData36_INSERT_CHANNEL_4CH, 4},
{12, 3, 648, 16, 0, gScenarioBufferData37_INSERT_CHANNEL_4CH, 64, gScenarioKernelData37_INSERT_CHANNEL_4CH, 4},
{14, 3, 756, 16, 0, gScenarioBufferData38_INSERT_CHANNEL_4CH, 64, gScenarioKernelData38_INSERT_CHANNEL_4CH, 4},
{16, 3, 864, 16, 0, gScenarioBufferData39_INSERT_CHANNEL_4CH, 64, gScenarioKernelData39_INSERT_CHANNEL_4CH, 4},
{18, 3, 972, 16, 0, gScenarioBufferData40_INSERT_CHANNEL_4CH, 64, gScenarioKernelData40_INSERT_CHANNEL_4CH, 4},
{20, 3, 1080, 16, 0, gScenarioBufferData41_INSERT_CHANNEL_4CH, 64, gScenarioKernelData41_INSERT_CHANNEL_4CH, 4},
{22, 3, 1188, 16, 0, gScenarioBufferData42_INSERT_CHANNEL_4CH, 64, gScenarioKernelData42_INSERT_CHANNEL_4CH, 4},
{24, 3, 1296, 16, 0, gScenarioBufferData43_INSERT_CHANNEL_4CH, 64, gScenarioKernelData43_INSERT_CHANNEL_4CH, 4},
{26, 3, 1404, 16, 0, gScenarioBufferData44_INSERT_CHANNEL_4CH, 64, gScenarioKernelData44_INSERT_CHANNEL_4CH, 4},
{28, 3, 1512, 16, 0, gScenarioBufferData45_INSERT_CHANNEL_4CH, 64, gScenarioKernelData45_INSERT_CHANNEL_4CH, 4},
{30, 3, 1620, 16, 0, gScenarioBufferData46_INSERT_CHANNEL_4CH, 64, gScenarioKernelData46_INSERT_CHANNEL_4CH, 4},
{32, 3, 1728, 16, 0, gScenarioBufferData47_INSERT_CHANNEL_4CH, 64, gScenarioKernelData47_INSERT_CHANNEL_4CH, 4},
{2, 4, 144, 16, 0, gScenarioBufferData48_INSERT_CHANNEL_4CH, 64, gScenarioKernelData48_INSERT_CHANNEL_4CH, 4},
{4, 4, 288, 16, 0, gScenarioBufferData49_INSERT_CHANNEL_4CH, 64, gScenarioKernelData49_INSERT_CHANNEL_4CH, 4},
{6, 4, 432, 16, 0, gScenarioBufferData50_INSERT_CHANNEL_4CH, 64, gScenarioKernelData50_INSERT_CHANNEL_4CH, 4},
{8, 4, 576, 16, 0, gScenarioBufferData51_INSERT_CHANNEL_4CH, 64, gScenarioKernelData51_INSERT_CHANNEL_4CH, 4},
{10, 4, 720, 16, 0, gScenarioBufferData52_INSERT_CHANNEL_4CH, 64, gScenarioKernelData52_INSERT_CHANNEL_4CH, 4},
{12, 4, 864, 16, 0, gScenarioBufferData53_INSERT_CHANNEL_4CH, 64, gScenarioKernelData53_INSERT_CHANNEL_4CH, 4},
{14, 4, 1008, 16, 0, gScenarioBufferData54_INSERT_CHANNEL_4CH, 64, gScenarioKernelData54_INSERT_CHANNEL_4CH, 4},
{16, 4, 1152, 16, 0, gScenarioBufferData55_INSERT_CHANNEL_4CH, 64, gScenarioKernelData55_INSERT_CHANNEL_4CH, 4},
{18, 4, 1296, 16, 0, gScenarioBufferData56_INSERT_CHANNEL_4CH, 64, gScenarioKernelData56_INSERT_CHANNEL_4CH, 4},
{20, 4, 1440, 16, 0, gScenarioBufferData57_INSERT_CHANNEL_4CH, 64, gScenarioKernelData57_INSERT_CHANNEL_4CH, 4},
{22, 4, 1584, 16, 0, gScenarioBufferData58_INSERT_CHANNEL_4CH, 64, gScenarioKernelData58_INSERT_CHANNEL_4CH, 4},
{24, 4, 1728, 16, 0, gScenarioBufferData59_INSERT_CHANNEL_4CH, 64, gScenarioKernelData59_INSERT_CHANNEL_4CH, 4},
{26, 4, 1872, 16, 0, gScenarioBufferData60_INSERT_CHANNEL_4CH, 64, gScenarioKernelData60_INSERT_CHANNEL_4CH, 4},
{28, 4, 2016, 16, 0, gScenarioBufferData61_INSERT_CHANNEL_4CH, 64, gScenarioKernelData61_INSERT_CHANNEL_4CH, 4},
{30, 4, 2160, 16, 0, gScenarioBufferData62_INSERT_CHANNEL_4CH, 64, gScenarioKernelData62_INSERT_CHANNEL_4CH, 4},
{32, 4, 2304, 16, 0, gScenarioBufferData63_INSERT_CHANNEL_4CH, 64, gScenarioKernelData63_INSERT_CHANNEL_4CH, 4},
{2, 5, 180, 16, 0, gScenarioBufferData64_INSERT_CHANNEL_4CH, 64, gScenarioKernelData64_INSERT_CHANNEL_4CH, 4},
{4, 5, 360, 16, 0, gScenarioBufferData65_INSERT_CHANNEL_4CH, 64, gScenarioKernelData65_INSERT_CHANNEL_4CH, 4},
{6, 5, 540, 16, 0, gScenarioBufferData66_INSERT_CHANNEL_4CH, 64, gScenarioKernelData66_INSERT_CHANNEL_4CH, 4},
{8, 5, 720, 16, 0, gScenarioBufferData67_INSERT_CHANNEL_4CH, 64, gScenarioKernelData67_INSERT_CHANNEL_4CH, 4},
{10, 5, 900, 16, 0, gScenarioBufferData68_INSERT_CHANNEL_4CH, 64, gScenarioKernelData68_INSERT_CHANNEL_4CH, 4},
{12, 5, 1080, 16, 0, gScenarioBufferData69_INSERT_CHANNEL_4CH, 64, gScenarioKernelData69_INSERT_CHANNEL_4CH, 4},
{14, 5, 1260, 16, 0, gScenarioBufferData70_INSERT_CHANNEL_4CH, 64, gScenarioKernelData70_INSERT_CHANNEL_4CH, 4},
{16, 5, 1440, 16, 0, gScenarioBufferData71_INSERT_CHANNEL_4CH, 64, gScenarioKernelData71_INSERT_CHANNEL_4CH, 4},
{18, 5, 1620, 16, 0, gScenarioBufferData72_INSERT_CHANNEL_4CH, 64, gScenarioKernelData72_INSERT_CHANNEL_4CH, 4},
{20, 5, 1800, 16, 0, gScenarioBufferData73_INSERT_CHANNEL_4CH, 64, gScenarioKernelData73_INSERT_CHANNEL_4CH, 4},
{22, 5, 1980, 16, 0, gScenarioBufferData74_INSERT_CHANNEL_4CH, 64, gScenarioKernelData74_INSERT_CHANNEL_4CH, 4},
{24, 5, 2160, 16, 0, gScenarioBufferData75_INSERT_CHANNEL_4CH, 64, gScenarioKernelData75_INSERT_CHANNEL_4CH, 4},
{26, 5, 2340, 16, 0, gScenarioBufferData76_INSERT_CHANNEL_4CH, 64, gScenarioKernelData76_INSERT_CHANNEL_4CH, 4},
{28, 5, 2520, 16, 0, gScenarioBufferData77_INSERT_CHANNEL_4CH, 64, gScenarioKernelData77_INSERT_CHANNEL_4CH, 4},
{30, 5, 2700, 16, 0, gScenarioBufferData78_INSERT_CHANNEL_4CH, 64, gScenarioKernelData78_INSERT_CHANNEL_4CH, 4},
{32, 5, 2880, 16, 0, gScenarioBufferData79_INSERT_CHANNEL_4CH, 64, gScenarioKernelData79_INSERT_CHANNEL_4CH, 4},
{2, 6, 216, 16, 0, gScenarioBufferData80_INSERT_CHANNEL_4CH, 64, gScenarioKernelData80_INSERT_CHANNEL_4CH, 4},
{4, 6, 432, 16, 0, gScenarioBufferData81_INSERT_CHANNEL_4CH, 64, gScenarioKernelData81_INSERT_CHANNEL_4CH, 4},
{6, 6, 648, 16, 0, gScenarioBufferData82_INSERT_CHANNEL_4CH, 64, gScenarioKernelData82_INSERT_CHANNEL_4CH, 4},
{8, 6, 864, 16, 0, gScenarioBufferData83_INSERT_CHANNEL_4CH, 64, gScenarioKernelData83_INSERT_CHANNEL_4CH, 4},
{10, 6, 1080, 16, 0, gScenarioBufferData84_INSERT_CHANNEL_4CH, 64, gScenarioKernelData84_INSERT_CHANNEL_4CH, 4},
{12, 6, 1296, 16, 0, gScenarioBufferData85_INSERT_CHANNEL_4CH, 64, gScenarioKernelData85_INSERT_CHANNEL_4CH, 4},
{14, 6, 1512, 16, 0, gScenarioBufferData86_INSERT_CHANNEL_4CH, 64, gScenarioKernelData86_INSERT_CHANNEL_4CH, 4},
{16, 6, 1728, 16, 0, gScenarioBufferData87_INSERT_CHANNEL_4CH, 64, gScenarioKernelData87_INSERT_CHANNEL_4CH, 4},
{18, 6, 1944, 16, 0, gScenarioBufferData88_INSERT_CHANNEL_4CH, 64, gScenarioKernelData88_INSERT_CHANNEL_4CH, 4},
{20, 6, 2160, 16, 0, gScenarioBufferData89_INSERT_CHANNEL_4CH, 64, gScenarioKernelData89_INSERT_CHANNEL_4CH, 4},
{22, 6, 2376, 16, 0, gScenarioBufferData90_INSERT_CHANNEL_4CH, 64, gScenarioKernelData90_INSERT_CHANNEL_4CH, 4},
{24, 6, 2592, 16, 0, gScenarioBufferData91_INSERT_CHANNEL_4CH, 64, gScenarioKernelData91_INSERT_CHANNEL_4CH, 4},
{26, 6, 2808, 16, 0, gScenarioBufferData92_INSERT_CHANNEL_4CH, 64, gScenarioKernelData92_INSERT_CHANNEL_4CH, 4},
{28, 6, 3024, 16, 0, gScenarioBufferData93_INSERT_CHANNEL_4CH, 64, gScenarioKernelData93_INSERT_CHANNEL_4CH, 4},
{30, 6, 3240, 16, 0, gScenarioBufferData94_INSERT_CHANNEL_4CH, 64, gScenarioKernelData94_INSERT_CHANNEL_4CH, 4},
{32, 6, 3456, 16, 0, gScenarioBufferData95_INSERT_CHANNEL_4CH, 64, gScenarioKernelData95_INSERT_CHANNEL_4CH, 4},
{2, 8, 288, 16, 0, gScenarioBufferData96_INSERT_CHANNEL_4CH, 64, gScenarioKernelData96_INSERT_CHANNEL_4CH, 4},
{4, 8, 576, 16, 0, gScenarioBufferData97_INSERT_CHANNEL_4CH, 64, gScenarioKernelData97_INSERT_CHANNEL_4CH, 4},
{6, 8, 864, 16, 0, gScenarioBufferData98_INSERT_CHANNEL_4CH, 64, gScenarioKernelData98_INSERT_CHANNEL_4CH, 4},
{8, 8, 1152, 16, 0, gScenarioBufferData99_INSERT_CHANNEL_4CH, 64, gScenarioKernelData99_INSERT_CHANNEL_4CH, 4},
{10, 8, 1440, 16, 0, gScenarioBufferData100_INSERT_CHANNEL_4CH, 64, gScenarioKernelData100_INSERT_CHANNEL_4CH, 4},
{12, 8, 1728, 16, 0, gScenarioBufferData101_INSERT_CHANNEL_4CH, 64, gScenarioKernelData101_INSERT_CHANNEL_4CH, 4},
{14, 8, 2016, 16, 0, gScenarioBufferData102_INSERT_CHANNEL_4CH, 64, gScenarioKernelData102_INSERT_CHANNEL_4CH, 4},
{16, 8, 2304, 16, 0, gScenarioBufferData103_INSERT_CHANNEL_4CH, 64, gScenarioKernelData103_INSERT_CHANNEL_4CH, 4},
{18, 8, 2592, 16, 0, gScenarioBufferData104_INSERT_CHANNEL_4CH, 64, gScenarioKernelData104_INSERT_CHANNEL_4CH, 4},
{20, 8, 2880, 16, 0, gScenarioBufferData105_INSERT_CHANNEL_4CH, 64, gScenarioKernelData105_INSERT_CHANNEL_4CH, 4},
{22, 8, 3168, 16, 0, gScenarioBufferData106_INSERT_CHANNEL_4CH, 64, gScenarioKernelData106_INSERT_CHANNEL_4CH, 4},
{24, 8, 3456, 16, 0, gScenarioBufferData107_INSERT_CHANNEL_4CH, 64, gScenarioKernelData107_INSERT_CHANNEL_4CH, 4},
{26, 8, 3744, 16, 0, gScenarioBufferData108_INSERT_CHANNEL_4CH, 64, gScenarioKernelData108_INSERT_CHANNEL_4CH, 4},
{2, 10, 360, 16, 0, gScenarioBufferData109_INSERT_CHANNEL_4CH, 64, gScenarioKernelData109_INSERT_CHANNEL_4CH, 4},
{4, 10, 720, 16, 0, gScenarioBufferData110_INSERT_CHANNEL_4CH, 64, gScenarioKernelData110_INSERT_CHANNEL_4CH, 4},
{6, 10, 1080, 16, 0, gScenarioBufferData111_INSERT_CHANNEL_4CH, 64, gScenarioKernelData111_INSERT_CHANNEL_4CH, 4},
{8, 10, 1440, 16, 0, gScenarioBufferData112_INSERT_CHANNEL_4CH, 64, gScenarioKernelData112_INSERT_CHANNEL_4CH, 4},
{10, 10, 1800, 16, 0, gScenarioBufferData113_INSERT_CHANNEL_4CH, 64, gScenarioKernelData113_INSERT_CHANNEL_4CH, 4},
{12, 10, 2160, 16, 0, gScenarioBufferData114_INSERT_CHANNEL_4CH, 64, gScenarioKernelData114_INSERT_CHANNEL_4CH, 4},
{14, 10, 2520, 16, 0, gScenarioBufferData115_INSERT_CHANNEL_4CH, 64, gScenarioKernelData115_INSERT_CHANNEL_4CH, 4},
{16, 10, 2880, 16, 0, gScenarioBufferData116_INSERT_CHANNEL_4CH, 64, gScenarioKernelData116_INSERT_CHANNEL_4CH, 4},
{18, 10, 3240, 16, 0, gScenarioBufferData117_INSERT_CHANNEL_4CH, 64, gScenarioKernelData117_INSERT_CHANNEL_4CH, 4},
{20, 10, 3600, 16, 0, gScenarioBufferData118_INSERT_CHANNEL_4CH, 64, gScenarioKernelData118_INSERT_CHANNEL_4CH, 4},
{2, 12, 432, 16, 0, gScenarioBufferData119_INSERT_CHANNEL_4CH, 64, gScenarioKernelData119_INSERT_CHANNEL_4CH, 4},
{4, 12, 864, 16, 0, gScenarioBufferData120_INSERT_CHANNEL_4CH, 64, gScenarioKernelData120_INSERT_CHANNEL_4CH, 4},
{6, 12, 1296, 16, 0, gScenarioBufferData121_INSERT_CHANNEL_4CH, 64, gScenarioKernelData121_INSERT_CHANNEL_4CH, 4},
{8, 12, 1728, 16, 0, gScenarioBufferData122_INSERT_CHANNEL_4CH, 64, gScenarioKernelData122_INSERT_CHANNEL_4CH, 4},
{10, 12, 2160, 16, 0, gScenarioBufferData123_INSERT_CHANNEL_4CH, 64, gScenarioKernelData123_INSERT_CHANNEL_4CH, 4},
{12, 12, 2592, 16, 0, gScenarioBufferData124_INSERT_CHANNEL_4CH, 64, gScenarioKernelData124_INSERT_CHANNEL_4CH, 4},
{14, 12, 3024, 16, 0, gScenarioBufferData125_INSERT_CHANNEL_4CH, 64, gScenarioKernelData125_INSERT_CHANNEL_4CH, 4},
{16, 12, 3456, 16, 0, gScenarioBufferData126_INSERT_CHANNEL_4CH, 64, gScenarioKernelData126_INSERT_CHANNEL_4CH, 4},
{2, 14, 504, 16, 0, gScenarioBufferData127_INSERT_CHANNEL_4CH, 64, gScenarioKernelData127_INSERT_CHANNEL_4CH, 4},
{4, 14, 1008, 16, 0, gScenarioBufferData128_INSERT_CHANNEL_4CH, 64, gScenarioKernelData128_INSERT_CHANNEL_4CH, 4},
{6, 14, 1512, 16, 0, gScenarioBufferData129_INSERT_CHANNEL_4CH, 64, gScenarioKernelData129_INSERT_CHANNEL_4CH, 4},
{8, 14, 2016, 16, 0, gScenarioBufferData130_INSERT_CHANNEL_4CH, 64, gScenarioKernelData130_INSERT_CHANNEL_4CH, 4},
{10, 14, 2520, 16, 0, gScenarioBufferData131_INSERT_CHANNEL_4CH, 64, gScenarioKernelData131_INSERT_CHANNEL_4CH, 4},
{12, 14, 3024, 16, 0, gScenarioBufferData132_INSERT_CHANNEL_4CH, 64, gScenarioKernelData132_INSERT_CHANNEL_4CH, 4},
{14, 14, 3528, 16, 0, gScenarioBufferData133_INSERT_CHANNEL_4CH, 64, gScenarioKernelData133_INSERT_CHANNEL_4CH, 4},
{2, 16, 576, 16, 0, gScenarioBufferData134_INSERT_CHANNEL_4CH, 64, gScenarioKernelData134_INSERT_CHANNEL_4CH, 4},
{4, 16, 1152, 16, 0, gScenarioBufferData135_INSERT_CHANNEL_4CH, 64, gScenarioKernelData135_INSERT_CHANNEL_4CH, 4},
{6, 16, 1728, 16, 0, gScenarioBufferData136_INSERT_CHANNEL_4CH, 64, gScenarioKernelData136_INSERT_CHANNEL_4CH, 4},
{8, 16, 2304, 16, 0, gScenarioBufferData137_INSERT_CHANNEL_4CH, 64, gScenarioKernelData137_INSERT_CHANNEL_4CH, 4},
{10, 16, 2880, 16, 0, gScenarioBufferData138_INSERT_CHANNEL_4CH, 64, gScenarioKernelData138_INSERT_CHANNEL_4CH, 4},
{12, 16, 3456, 16, 0, gScenarioBufferData139_INSERT_CHANNEL_4CH, 64, gScenarioKernelData139_INSERT_CHANNEL_4CH, 4},
{2, 18, 648, 16, 0, gScenarioBufferData140_INSERT_CHANNEL_4CH, 64, gScenarioKernelData140_INSERT_CHANNEL_4CH, 4},
{4, 18, 1296, 16, 0, gScenarioBufferData141_INSERT_CHANNEL_4CH, 64, gScenarioKernelData141_INSERT_CHANNEL_4CH, 4},
{6, 18, 1944, 16, 0, gScenarioBufferData142_INSERT_CHANNEL_4CH, 64, gScenarioKernelData142_INSERT_CHANNEL_4CH, 4},
{8, 18, 2592, 16, 0, gScenarioBufferData143_INSERT_CHANNEL_4CH, 64, gScenarioKernelData143_INSERT_CHANNEL_4CH, 4},
{10, 18, 3240, 16, 0, gScenarioBufferData144_INSERT_CHANNEL_4CH, 64, gScenarioKernelData144_INSERT_CHANNEL_4CH, 4},
{2, 20, 720, 16, 0, gScenarioBufferData145_INSERT_CHANNEL_4CH, 64, gScenarioKernelData145_INSERT_CHANNEL_4CH, 4},
{4, 20, 1440, 16, 0, gScenarioBufferData146_INSERT_CHANNEL_4CH, 64, gScenarioKernelData146_INSERT_CHANNEL_4CH, 4},
{6, 20, 2160, 16, 0, gScenarioBufferData147_INSERT_CHANNEL_4CH, 64, gScenarioKernelData147_INSERT_CHANNEL_4CH, 4},
{8, 20, 2880, 16, 0, gScenarioBufferData148_INSERT_CHANNEL_4CH, 64, gScenarioKernelData148_INSERT_CHANNEL_4CH, 4},
{10, 20, 3600, 16, 0, gScenarioBufferData149_INSERT_CHANNEL_4CH, 64, gScenarioKernelData149_INSERT_CHANNEL_4CH, 4},
{2, 22, 792, 16, 0, gScenarioBufferData150_INSERT_CHANNEL_4CH, 64, gScenarioKernelData150_INSERT_CHANNEL_4CH, 4},
{4, 22, 1584, 16, 0, gScenarioBufferData151_INSERT_CHANNEL_4CH, 64, gScenarioKernelData151_INSERT_CHANNEL_4CH, 4},
{6, 22, 2376, 16, 0, gScenarioBufferData152_INSERT_CHANNEL_4CH, 64, gScenarioKernelData152_INSERT_CHANNEL_4CH, 4},
{8, 22, 3168, 16, 0, gScenarioBufferData153_INSERT_CHANNEL_4CH, 64, gScenarioKernelData153_INSERT_CHANNEL_4CH, 4},
{2, 24, 864, 16, 0, gScenarioBufferData154_INSERT_CHANNEL_4CH, 64, gScenarioKernelData154_INSERT_CHANNEL_4CH, 4},
{4, 24, 1728, 16, 0, gScenarioBufferData155_INSERT_CHANNEL_4CH, 64, gScenarioKernelData155_INSERT_CHANNEL_4CH, 4},
{6, 24, 2592, 16, 0, gScenarioBufferData156_INSERT_CHANNEL_4CH, 64, gScenarioKernelData156_INSERT_CHANNEL_4CH, 4},
{8, 24, 3456, 16, 0, gScenarioBufferData157_INSERT_CHANNEL_4CH, 64, gScenarioKernelData157_INSERT_CHANNEL_4CH, 4},
{2, 26, 936, 16, 0, gScenarioBufferData158_INSERT_CHANNEL_4CH, 64, gScenarioKernelData158_INSERT_CHANNEL_4CH, 4},
{4, 26, 1872, 16, 0, gScenarioBufferData159_INSERT_CHANNEL_4CH, 64, gScenarioKernelData159_INSERT_CHANNEL_4CH, 4},
{6, 26, 2808, 16, 0, gScenarioBufferData160_INSERT_CHANNEL_4CH, 64, gScenarioKernelData160_INSERT_CHANNEL_4CH, 4},
{8, 26, 3744, 16, 0, gScenarioBufferData161_INSERT_CHANNEL_4CH, 64, gScenarioKernelData161_INSERT_CHANNEL_4CH, 4},
{2, 28, 1008, 16, 0, gScenarioBufferData162_INSERT_CHANNEL_4CH, 64, gScenarioKernelData162_INSERT_CHANNEL_4CH, 4},
{4, 28, 2016, 16, 0, gScenarioBufferData163_INSERT_CHANNEL_4CH, 64, gScenarioKernelData163_INSERT_CHANNEL_4CH, 4},
{6, 28, 3024, 16, 0, gScenarioBufferData164_INSERT_CHANNEL_4CH, 64, gScenarioKernelData164_INSERT_CHANNEL_4CH, 4},
{2, 30, 1080, 16, 0, gScenarioBufferData165_INSERT_CHANNEL_4CH, 64, gScenarioKernelData165_INSERT_CHANNEL_4CH, 4},
{4, 30, 2160, 16, 0, gScenarioBufferData166_INSERT_CHANNEL_4CH, 64, gScenarioKernelData166_INSERT_CHANNEL_4CH, 4},
{6, 30, 3240, 16, 0, gScenarioBufferData167_INSERT_CHANNEL_4CH, 64, gScenarioKernelData167_INSERT_CHANNEL_4CH, 4},
{2, 32, 1152, 16, 0, gScenarioBufferData168_INSERT_CHANNEL_4CH, 64, gScenarioKernelData168_INSERT_CHANNEL_4CH, 4},
{4, 32, 2304, 16, 0, gScenarioBufferData169_INSERT_CHANNEL_4CH, 64, gScenarioKernelData169_INSERT_CHANNEL_4CH, 4},
{6, 32, 3456, 16, 0, gScenarioBufferData170_INSERT_CHANNEL_4CH, 64, gScenarioKernelData170_INSERT_CHANNEL_4CH, 4}
};

static acf_scenario_list gScenarioList_INSERT_CHANNEL_4CH = {
171, //number of scenarios
gScenarioArray_INSERT_CHANNEL_4CH};
//**************************************************************

class INSERT_CHANNEL_4CH : public ACF_Process_APU
{

public:
   INSERT_CHANNEL_4CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INSERT_CHANNEL_4CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INSERT_CHANNEL_4CH",
                                        INSERT_CHANNEL_4CH_LOAD_SEGMENTS,
                                        INSERT_CHANNEL_4CH_LOAD_PMEM, INSERT_CHANNEL_4CH_LOAD_PMEM_SIZE,
                                        INSERT_CHANNEL_4CH_LOAD_DMEM, INSERT_CHANNEL_4CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INSERT_CHANNEL_4CH_APEX_LOG_BUFFER, INSERT_CHANNEL_4CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 4, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 4, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INSERT_CHANNEL_4CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INSERT_CHANNEL_4CH

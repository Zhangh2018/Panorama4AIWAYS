#ifndef _ACF_PROCESS_APU_MERGE_CHANNEL_4CH
#define _ACF_PROCESS_APU_MERGE_CHANNEL_4CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MERGE_CHANNEL_4CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MERGE_CHANNEL_4CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MERGE_CHANNEL_4CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MERGE_CHANNEL_4CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MERGE_CHANNEL_4CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MERGE_CHANNEL_4CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MERGE_CHANNEL_4CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MERGE_CHANNEL_4CH[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MERGE_CHANNEL_4CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MERGE_CHANNEL_4CH[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MERGE_CHANNEL_4CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MERGE_CHANNEL_4CH[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MERGE_CHANNEL_4CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MERGE_CHANNEL_4CH[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MERGE_CHANNEL_4CH[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MERGE_CHANNEL_4CH[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MERGE_CHANNEL_4CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MERGE_CHANNEL_4CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MERGE_CHANNEL_4CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MERGE_CHANNEL_4CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MERGE_CHANNEL_4CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MERGE_CHANNEL_4CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MERGE_CHANNEL_4CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MERGE_CHANNEL_4CH[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MERGE_CHANNEL_4CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MERGE_CHANNEL_4CH[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MERGE_CHANNEL_4CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MERGE_CHANNEL_4CH[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MERGE_CHANNEL_4CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MERGE_CHANNEL_4CH[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MERGE_CHANNEL_4CH[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MERGE_CHANNEL_4CH[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MERGE_CHANNEL_4CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MERGE_CHANNEL_4CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MERGE_CHANNEL_4CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MERGE_CHANNEL_4CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MERGE_CHANNEL_4CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MERGE_CHANNEL_4CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MERGE_CHANNEL_4CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MERGE_CHANNEL_4CH[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MERGE_CHANNEL_4CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MERGE_CHANNEL_4CH[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MERGE_CHANNEL_4CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MERGE_CHANNEL_4CH[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MERGE_CHANNEL_4CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MERGE_CHANNEL_4CH[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MERGE_CHANNEL_4CH[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MERGE_CHANNEL_4CH[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MERGE_CHANNEL_4CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MERGE_CHANNEL_4CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MERGE_CHANNEL_4CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MERGE_CHANNEL_4CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MERGE_CHANNEL_4CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MERGE_CHANNEL_4CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MERGE_CHANNEL_4CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MERGE_CHANNEL_4CH[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MERGE_CHANNEL_4CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MERGE_CHANNEL_4CH[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MERGE_CHANNEL_4CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MERGE_CHANNEL_4CH[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MERGE_CHANNEL_4CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MERGE_CHANNEL_4CH[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MERGE_CHANNEL_4CH[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MERGE_CHANNEL_4CH[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MERGE_CHANNEL_4CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MERGE_CHANNEL_4CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MERGE_CHANNEL_4CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MERGE_CHANNEL_4CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MERGE_CHANNEL_4CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MERGE_CHANNEL_4CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MERGE_CHANNEL_4CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MERGE_CHANNEL_4CH[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MERGE_CHANNEL_4CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MERGE_CHANNEL_4CH[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MERGE_CHANNEL_4CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MERGE_CHANNEL_4CH[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MERGE_CHANNEL_4CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MERGE_CHANNEL_4CH[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MERGE_CHANNEL_4CH[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MERGE_CHANNEL_4CH[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MERGE_CHANNEL_4CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MERGE_CHANNEL_4CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MERGE_CHANNEL_4CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MERGE_CHANNEL_4CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MERGE_CHANNEL_4CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MERGE_CHANNEL_4CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MERGE_CHANNEL_4CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MERGE_CHANNEL_4CH[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MERGE_CHANNEL_4CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MERGE_CHANNEL_4CH[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MERGE_CHANNEL_4CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MERGE_CHANNEL_4CH[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MERGE_CHANNEL_4CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MERGE_CHANNEL_4CH[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MERGE_CHANNEL_4CH[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MERGE_CHANNEL_4CH[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MERGE_CHANNEL_4CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MERGE_CHANNEL_4CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MERGE_CHANNEL_4CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MERGE_CHANNEL_4CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MERGE_CHANNEL_4CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MERGE_CHANNEL_4CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MERGE_CHANNEL_4CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MERGE_CHANNEL_4CH[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MERGE_CHANNEL_4CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MERGE_CHANNEL_4CH[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MERGE_CHANNEL_4CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MERGE_CHANNEL_4CH[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MERGE_CHANNEL_4CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MERGE_CHANNEL_4CH[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MERGE_CHANNEL_4CH[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MERGE_CHANNEL_4CH[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MERGE_CHANNEL_4CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MERGE_CHANNEL_4CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MERGE_CHANNEL_4CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MERGE_CHANNEL_4CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MERGE_CHANNEL_4CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MERGE_CHANNEL_4CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MERGE_CHANNEL_4CH[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MERGE_CHANNEL_4CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MERGE_CHANNEL_4CH[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MERGE_CHANNEL_4CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MERGE_CHANNEL_4CH[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MERGE_CHANNEL_4CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MERGE_CHANNEL_4CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MERGE_CHANNEL_4CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MERGE_CHANNEL_4CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MERGE_CHANNEL_4CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MERGE_CHANNEL_4CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MERGE_CHANNEL_4CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MERGE_CHANNEL_4CH[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MERGE_CHANNEL_4CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MERGE_CHANNEL_4CH[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MERGE_CHANNEL_4CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MERGE_CHANNEL_4CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MERGE_CHANNEL_4CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MERGE_CHANNEL_4CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MERGE_CHANNEL_4CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MERGE_CHANNEL_4CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MERGE_CHANNEL_4CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MERGE_CHANNEL_4CH[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MERGE_CHANNEL_4CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MERGE_CHANNEL_4CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MERGE_CHANNEL_4CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MERGE_CHANNEL_4CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MERGE_CHANNEL_4CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MERGE_CHANNEL_4CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MERGE_CHANNEL_4CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MERGE_CHANNEL_4CH[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MERGE_CHANNEL_4CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MERGE_CHANNEL_4CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MERGE_CHANNEL_4CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MERGE_CHANNEL_4CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MERGE_CHANNEL_4CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MERGE_CHANNEL_4CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MERGE_CHANNEL_4CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MERGE_CHANNEL_4CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MERGE_CHANNEL_4CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MERGE_CHANNEL_4CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MERGE_CHANNEL_4CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MERGE_CHANNEL_4CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MERGE_CHANNEL_4CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MERGE_CHANNEL_4CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MERGE_CHANNEL_4CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MERGE_CHANNEL_4CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MERGE_CHANNEL_4CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MERGE_CHANNEL_4CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MERGE_CHANNEL_4CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MERGE_CHANNEL_4CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MERGE_CHANNEL_4CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MERGE_CHANNEL_4CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MERGE_CHANNEL_4CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MERGE_CHANNEL_4CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MERGE_CHANNEL_4CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MERGE_CHANNEL_4CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MERGE_CHANNEL_4CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MERGE_CHANNEL_4CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MERGE_CHANNEL_4CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MERGE_CHANNEL_4CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MERGE_CHANNEL_4CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MERGE_CHANNEL_4CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MERGE_CHANNEL_4CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MERGE_CHANNEL_4CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MERGE_CHANNEL_4CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MERGE_CHANNEL_4CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MERGE_CHANNEL_4CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MERGE_CHANNEL_4CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MERGE_CHANNEL_4CH[] = {{0, 0}};

static acf_scenario gScenarioArray_MERGE_CHANNEL_4CH[] = {
{2, 1, 32, 0, 0, gScenarioBufferData0_MERGE_CHANNEL_4CH, 80, gScenarioKernelData0_MERGE_CHANNEL_4CH, 4},
{4, 1, 64, 0, 0, gScenarioBufferData1_MERGE_CHANNEL_4CH, 80, gScenarioKernelData1_MERGE_CHANNEL_4CH, 4},
{6, 1, 96, 0, 0, gScenarioBufferData2_MERGE_CHANNEL_4CH, 80, gScenarioKernelData2_MERGE_CHANNEL_4CH, 4},
{8, 1, 128, 0, 0, gScenarioBufferData3_MERGE_CHANNEL_4CH, 80, gScenarioKernelData3_MERGE_CHANNEL_4CH, 4},
{10, 1, 160, 0, 0, gScenarioBufferData4_MERGE_CHANNEL_4CH, 80, gScenarioKernelData4_MERGE_CHANNEL_4CH, 4},
{12, 1, 192, 0, 0, gScenarioBufferData5_MERGE_CHANNEL_4CH, 80, gScenarioKernelData5_MERGE_CHANNEL_4CH, 4},
{14, 1, 224, 0, 0, gScenarioBufferData6_MERGE_CHANNEL_4CH, 80, gScenarioKernelData6_MERGE_CHANNEL_4CH, 4},
{16, 1, 256, 0, 0, gScenarioBufferData7_MERGE_CHANNEL_4CH, 80, gScenarioKernelData7_MERGE_CHANNEL_4CH, 4},
{18, 1, 288, 0, 0, gScenarioBufferData8_MERGE_CHANNEL_4CH, 80, gScenarioKernelData8_MERGE_CHANNEL_4CH, 4},
{20, 1, 320, 0, 0, gScenarioBufferData9_MERGE_CHANNEL_4CH, 80, gScenarioKernelData9_MERGE_CHANNEL_4CH, 4},
{22, 1, 352, 0, 0, gScenarioBufferData10_MERGE_CHANNEL_4CH, 80, gScenarioKernelData10_MERGE_CHANNEL_4CH, 4},
{24, 1, 384, 0, 0, gScenarioBufferData11_MERGE_CHANNEL_4CH, 80, gScenarioKernelData11_MERGE_CHANNEL_4CH, 4},
{26, 1, 416, 0, 0, gScenarioBufferData12_MERGE_CHANNEL_4CH, 80, gScenarioKernelData12_MERGE_CHANNEL_4CH, 4},
{28, 1, 448, 0, 0, gScenarioBufferData13_MERGE_CHANNEL_4CH, 80, gScenarioKernelData13_MERGE_CHANNEL_4CH, 4},
{30, 1, 480, 0, 0, gScenarioBufferData14_MERGE_CHANNEL_4CH, 80, gScenarioKernelData14_MERGE_CHANNEL_4CH, 4},
{32, 1, 512, 0, 0, gScenarioBufferData15_MERGE_CHANNEL_4CH, 80, gScenarioKernelData15_MERGE_CHANNEL_4CH, 4},
{2, 2, 64, 0, 0, gScenarioBufferData16_MERGE_CHANNEL_4CH, 80, gScenarioKernelData16_MERGE_CHANNEL_4CH, 4},
{4, 2, 128, 0, 0, gScenarioBufferData17_MERGE_CHANNEL_4CH, 80, gScenarioKernelData17_MERGE_CHANNEL_4CH, 4},
{6, 2, 192, 0, 0, gScenarioBufferData18_MERGE_CHANNEL_4CH, 80, gScenarioKernelData18_MERGE_CHANNEL_4CH, 4},
{8, 2, 256, 0, 0, gScenarioBufferData19_MERGE_CHANNEL_4CH, 80, gScenarioKernelData19_MERGE_CHANNEL_4CH, 4},
{10, 2, 320, 0, 0, gScenarioBufferData20_MERGE_CHANNEL_4CH, 80, gScenarioKernelData20_MERGE_CHANNEL_4CH, 4},
{12, 2, 384, 0, 0, gScenarioBufferData21_MERGE_CHANNEL_4CH, 80, gScenarioKernelData21_MERGE_CHANNEL_4CH, 4},
{14, 2, 448, 0, 0, gScenarioBufferData22_MERGE_CHANNEL_4CH, 80, gScenarioKernelData22_MERGE_CHANNEL_4CH, 4},
{16, 2, 512, 0, 0, gScenarioBufferData23_MERGE_CHANNEL_4CH, 80, gScenarioKernelData23_MERGE_CHANNEL_4CH, 4},
{18, 2, 576, 0, 0, gScenarioBufferData24_MERGE_CHANNEL_4CH, 80, gScenarioKernelData24_MERGE_CHANNEL_4CH, 4},
{20, 2, 640, 0, 0, gScenarioBufferData25_MERGE_CHANNEL_4CH, 80, gScenarioKernelData25_MERGE_CHANNEL_4CH, 4},
{22, 2, 704, 0, 0, gScenarioBufferData26_MERGE_CHANNEL_4CH, 80, gScenarioKernelData26_MERGE_CHANNEL_4CH, 4},
{24, 2, 768, 0, 0, gScenarioBufferData27_MERGE_CHANNEL_4CH, 80, gScenarioKernelData27_MERGE_CHANNEL_4CH, 4},
{26, 2, 832, 0, 0, gScenarioBufferData28_MERGE_CHANNEL_4CH, 80, gScenarioKernelData28_MERGE_CHANNEL_4CH, 4},
{28, 2, 896, 0, 0, gScenarioBufferData29_MERGE_CHANNEL_4CH, 80, gScenarioKernelData29_MERGE_CHANNEL_4CH, 4},
{30, 2, 960, 0, 0, gScenarioBufferData30_MERGE_CHANNEL_4CH, 80, gScenarioKernelData30_MERGE_CHANNEL_4CH, 4},
{32, 2, 1024, 0, 0, gScenarioBufferData31_MERGE_CHANNEL_4CH, 80, gScenarioKernelData31_MERGE_CHANNEL_4CH, 4},
{2, 3, 96, 0, 0, gScenarioBufferData32_MERGE_CHANNEL_4CH, 80, gScenarioKernelData32_MERGE_CHANNEL_4CH, 4},
{4, 3, 192, 0, 0, gScenarioBufferData33_MERGE_CHANNEL_4CH, 80, gScenarioKernelData33_MERGE_CHANNEL_4CH, 4},
{6, 3, 288, 0, 0, gScenarioBufferData34_MERGE_CHANNEL_4CH, 80, gScenarioKernelData34_MERGE_CHANNEL_4CH, 4},
{8, 3, 384, 0, 0, gScenarioBufferData35_MERGE_CHANNEL_4CH, 80, gScenarioKernelData35_MERGE_CHANNEL_4CH, 4},
{10, 3, 480, 0, 0, gScenarioBufferData36_MERGE_CHANNEL_4CH, 80, gScenarioKernelData36_MERGE_CHANNEL_4CH, 4},
{12, 3, 576, 0, 0, gScenarioBufferData37_MERGE_CHANNEL_4CH, 80, gScenarioKernelData37_MERGE_CHANNEL_4CH, 4},
{14, 3, 672, 0, 0, gScenarioBufferData38_MERGE_CHANNEL_4CH, 80, gScenarioKernelData38_MERGE_CHANNEL_4CH, 4},
{16, 3, 768, 0, 0, gScenarioBufferData39_MERGE_CHANNEL_4CH, 80, gScenarioKernelData39_MERGE_CHANNEL_4CH, 4},
{18, 3, 864, 0, 0, gScenarioBufferData40_MERGE_CHANNEL_4CH, 80, gScenarioKernelData40_MERGE_CHANNEL_4CH, 4},
{20, 3, 960, 0, 0, gScenarioBufferData41_MERGE_CHANNEL_4CH, 80, gScenarioKernelData41_MERGE_CHANNEL_4CH, 4},
{22, 3, 1056, 0, 0, gScenarioBufferData42_MERGE_CHANNEL_4CH, 80, gScenarioKernelData42_MERGE_CHANNEL_4CH, 4},
{24, 3, 1152, 0, 0, gScenarioBufferData43_MERGE_CHANNEL_4CH, 80, gScenarioKernelData43_MERGE_CHANNEL_4CH, 4},
{26, 3, 1248, 0, 0, gScenarioBufferData44_MERGE_CHANNEL_4CH, 80, gScenarioKernelData44_MERGE_CHANNEL_4CH, 4},
{28, 3, 1344, 0, 0, gScenarioBufferData45_MERGE_CHANNEL_4CH, 80, gScenarioKernelData45_MERGE_CHANNEL_4CH, 4},
{30, 3, 1440, 0, 0, gScenarioBufferData46_MERGE_CHANNEL_4CH, 80, gScenarioKernelData46_MERGE_CHANNEL_4CH, 4},
{32, 3, 1536, 0, 0, gScenarioBufferData47_MERGE_CHANNEL_4CH, 80, gScenarioKernelData47_MERGE_CHANNEL_4CH, 4},
{2, 4, 128, 0, 0, gScenarioBufferData48_MERGE_CHANNEL_4CH, 80, gScenarioKernelData48_MERGE_CHANNEL_4CH, 4},
{4, 4, 256, 0, 0, gScenarioBufferData49_MERGE_CHANNEL_4CH, 80, gScenarioKernelData49_MERGE_CHANNEL_4CH, 4},
{6, 4, 384, 0, 0, gScenarioBufferData50_MERGE_CHANNEL_4CH, 80, gScenarioKernelData50_MERGE_CHANNEL_4CH, 4},
{8, 4, 512, 0, 0, gScenarioBufferData51_MERGE_CHANNEL_4CH, 80, gScenarioKernelData51_MERGE_CHANNEL_4CH, 4},
{10, 4, 640, 0, 0, gScenarioBufferData52_MERGE_CHANNEL_4CH, 80, gScenarioKernelData52_MERGE_CHANNEL_4CH, 4},
{12, 4, 768, 0, 0, gScenarioBufferData53_MERGE_CHANNEL_4CH, 80, gScenarioKernelData53_MERGE_CHANNEL_4CH, 4},
{14, 4, 896, 0, 0, gScenarioBufferData54_MERGE_CHANNEL_4CH, 80, gScenarioKernelData54_MERGE_CHANNEL_4CH, 4},
{16, 4, 1024, 0, 0, gScenarioBufferData55_MERGE_CHANNEL_4CH, 80, gScenarioKernelData55_MERGE_CHANNEL_4CH, 4},
{18, 4, 1152, 0, 0, gScenarioBufferData56_MERGE_CHANNEL_4CH, 80, gScenarioKernelData56_MERGE_CHANNEL_4CH, 4},
{20, 4, 1280, 0, 0, gScenarioBufferData57_MERGE_CHANNEL_4CH, 80, gScenarioKernelData57_MERGE_CHANNEL_4CH, 4},
{22, 4, 1408, 0, 0, gScenarioBufferData58_MERGE_CHANNEL_4CH, 80, gScenarioKernelData58_MERGE_CHANNEL_4CH, 4},
{24, 4, 1536, 0, 0, gScenarioBufferData59_MERGE_CHANNEL_4CH, 80, gScenarioKernelData59_MERGE_CHANNEL_4CH, 4},
{26, 4, 1664, 0, 0, gScenarioBufferData60_MERGE_CHANNEL_4CH, 80, gScenarioKernelData60_MERGE_CHANNEL_4CH, 4},
{28, 4, 1792, 0, 0, gScenarioBufferData61_MERGE_CHANNEL_4CH, 80, gScenarioKernelData61_MERGE_CHANNEL_4CH, 4},
{30, 4, 1920, 0, 0, gScenarioBufferData62_MERGE_CHANNEL_4CH, 80, gScenarioKernelData62_MERGE_CHANNEL_4CH, 4},
{32, 4, 2048, 0, 0, gScenarioBufferData63_MERGE_CHANNEL_4CH, 80, gScenarioKernelData63_MERGE_CHANNEL_4CH, 4},
{2, 5, 160, 0, 0, gScenarioBufferData64_MERGE_CHANNEL_4CH, 80, gScenarioKernelData64_MERGE_CHANNEL_4CH, 4},
{4, 5, 320, 0, 0, gScenarioBufferData65_MERGE_CHANNEL_4CH, 80, gScenarioKernelData65_MERGE_CHANNEL_4CH, 4},
{6, 5, 480, 0, 0, gScenarioBufferData66_MERGE_CHANNEL_4CH, 80, gScenarioKernelData66_MERGE_CHANNEL_4CH, 4},
{8, 5, 640, 0, 0, gScenarioBufferData67_MERGE_CHANNEL_4CH, 80, gScenarioKernelData67_MERGE_CHANNEL_4CH, 4},
{10, 5, 800, 0, 0, gScenarioBufferData68_MERGE_CHANNEL_4CH, 80, gScenarioKernelData68_MERGE_CHANNEL_4CH, 4},
{12, 5, 960, 0, 0, gScenarioBufferData69_MERGE_CHANNEL_4CH, 80, gScenarioKernelData69_MERGE_CHANNEL_4CH, 4},
{14, 5, 1120, 0, 0, gScenarioBufferData70_MERGE_CHANNEL_4CH, 80, gScenarioKernelData70_MERGE_CHANNEL_4CH, 4},
{16, 5, 1280, 0, 0, gScenarioBufferData71_MERGE_CHANNEL_4CH, 80, gScenarioKernelData71_MERGE_CHANNEL_4CH, 4},
{18, 5, 1440, 0, 0, gScenarioBufferData72_MERGE_CHANNEL_4CH, 80, gScenarioKernelData72_MERGE_CHANNEL_4CH, 4},
{20, 5, 1600, 0, 0, gScenarioBufferData73_MERGE_CHANNEL_4CH, 80, gScenarioKernelData73_MERGE_CHANNEL_4CH, 4},
{22, 5, 1760, 0, 0, gScenarioBufferData74_MERGE_CHANNEL_4CH, 80, gScenarioKernelData74_MERGE_CHANNEL_4CH, 4},
{24, 5, 1920, 0, 0, gScenarioBufferData75_MERGE_CHANNEL_4CH, 80, gScenarioKernelData75_MERGE_CHANNEL_4CH, 4},
{26, 5, 2080, 0, 0, gScenarioBufferData76_MERGE_CHANNEL_4CH, 80, gScenarioKernelData76_MERGE_CHANNEL_4CH, 4},
{28, 5, 2240, 0, 0, gScenarioBufferData77_MERGE_CHANNEL_4CH, 80, gScenarioKernelData77_MERGE_CHANNEL_4CH, 4},
{30, 5, 2400, 0, 0, gScenarioBufferData78_MERGE_CHANNEL_4CH, 80, gScenarioKernelData78_MERGE_CHANNEL_4CH, 4},
{32, 5, 2560, 0, 0, gScenarioBufferData79_MERGE_CHANNEL_4CH, 80, gScenarioKernelData79_MERGE_CHANNEL_4CH, 4},
{2, 6, 192, 0, 0, gScenarioBufferData80_MERGE_CHANNEL_4CH, 80, gScenarioKernelData80_MERGE_CHANNEL_4CH, 4},
{4, 6, 384, 0, 0, gScenarioBufferData81_MERGE_CHANNEL_4CH, 80, gScenarioKernelData81_MERGE_CHANNEL_4CH, 4},
{6, 6, 576, 0, 0, gScenarioBufferData82_MERGE_CHANNEL_4CH, 80, gScenarioKernelData82_MERGE_CHANNEL_4CH, 4},
{8, 6, 768, 0, 0, gScenarioBufferData83_MERGE_CHANNEL_4CH, 80, gScenarioKernelData83_MERGE_CHANNEL_4CH, 4},
{10, 6, 960, 0, 0, gScenarioBufferData84_MERGE_CHANNEL_4CH, 80, gScenarioKernelData84_MERGE_CHANNEL_4CH, 4},
{12, 6, 1152, 0, 0, gScenarioBufferData85_MERGE_CHANNEL_4CH, 80, gScenarioKernelData85_MERGE_CHANNEL_4CH, 4},
{14, 6, 1344, 0, 0, gScenarioBufferData86_MERGE_CHANNEL_4CH, 80, gScenarioKernelData86_MERGE_CHANNEL_4CH, 4},
{16, 6, 1536, 0, 0, gScenarioBufferData87_MERGE_CHANNEL_4CH, 80, gScenarioKernelData87_MERGE_CHANNEL_4CH, 4},
{18, 6, 1728, 0, 0, gScenarioBufferData88_MERGE_CHANNEL_4CH, 80, gScenarioKernelData88_MERGE_CHANNEL_4CH, 4},
{20, 6, 1920, 0, 0, gScenarioBufferData89_MERGE_CHANNEL_4CH, 80, gScenarioKernelData89_MERGE_CHANNEL_4CH, 4},
{22, 6, 2112, 0, 0, gScenarioBufferData90_MERGE_CHANNEL_4CH, 80, gScenarioKernelData90_MERGE_CHANNEL_4CH, 4},
{24, 6, 2304, 0, 0, gScenarioBufferData91_MERGE_CHANNEL_4CH, 80, gScenarioKernelData91_MERGE_CHANNEL_4CH, 4},
{26, 6, 2496, 0, 0, gScenarioBufferData92_MERGE_CHANNEL_4CH, 80, gScenarioKernelData92_MERGE_CHANNEL_4CH, 4},
{28, 6, 2688, 0, 0, gScenarioBufferData93_MERGE_CHANNEL_4CH, 80, gScenarioKernelData93_MERGE_CHANNEL_4CH, 4},
{30, 6, 2880, 0, 0, gScenarioBufferData94_MERGE_CHANNEL_4CH, 80, gScenarioKernelData94_MERGE_CHANNEL_4CH, 4},
{32, 6, 3072, 0, 0, gScenarioBufferData95_MERGE_CHANNEL_4CH, 80, gScenarioKernelData95_MERGE_CHANNEL_4CH, 4},
{2, 8, 256, 0, 0, gScenarioBufferData96_MERGE_CHANNEL_4CH, 80, gScenarioKernelData96_MERGE_CHANNEL_4CH, 4},
{4, 8, 512, 0, 0, gScenarioBufferData97_MERGE_CHANNEL_4CH, 80, gScenarioKernelData97_MERGE_CHANNEL_4CH, 4},
{6, 8, 768, 0, 0, gScenarioBufferData98_MERGE_CHANNEL_4CH, 80, gScenarioKernelData98_MERGE_CHANNEL_4CH, 4},
{8, 8, 1024, 0, 0, gScenarioBufferData99_MERGE_CHANNEL_4CH, 80, gScenarioKernelData99_MERGE_CHANNEL_4CH, 4},
{10, 8, 1280, 0, 0, gScenarioBufferData100_MERGE_CHANNEL_4CH, 80, gScenarioKernelData100_MERGE_CHANNEL_4CH, 4},
{12, 8, 1536, 0, 0, gScenarioBufferData101_MERGE_CHANNEL_4CH, 80, gScenarioKernelData101_MERGE_CHANNEL_4CH, 4},
{14, 8, 1792, 0, 0, gScenarioBufferData102_MERGE_CHANNEL_4CH, 80, gScenarioKernelData102_MERGE_CHANNEL_4CH, 4},
{16, 8, 2048, 0, 0, gScenarioBufferData103_MERGE_CHANNEL_4CH, 80, gScenarioKernelData103_MERGE_CHANNEL_4CH, 4},
{18, 8, 2304, 0, 0, gScenarioBufferData104_MERGE_CHANNEL_4CH, 80, gScenarioKernelData104_MERGE_CHANNEL_4CH, 4},
{20, 8, 2560, 0, 0, gScenarioBufferData105_MERGE_CHANNEL_4CH, 80, gScenarioKernelData105_MERGE_CHANNEL_4CH, 4},
{22, 8, 2816, 0, 0, gScenarioBufferData106_MERGE_CHANNEL_4CH, 80, gScenarioKernelData106_MERGE_CHANNEL_4CH, 4},
{24, 8, 3072, 0, 0, gScenarioBufferData107_MERGE_CHANNEL_4CH, 80, gScenarioKernelData107_MERGE_CHANNEL_4CH, 4},
{26, 8, 3328, 0, 0, gScenarioBufferData108_MERGE_CHANNEL_4CH, 80, gScenarioKernelData108_MERGE_CHANNEL_4CH, 4},
{28, 8, 3584, 0, 0, gScenarioBufferData109_MERGE_CHANNEL_4CH, 80, gScenarioKernelData109_MERGE_CHANNEL_4CH, 4},
{30, 8, 3840, 0, 0, gScenarioBufferData110_MERGE_CHANNEL_4CH, 80, gScenarioKernelData110_MERGE_CHANNEL_4CH, 4},
{2, 10, 320, 0, 0, gScenarioBufferData111_MERGE_CHANNEL_4CH, 80, gScenarioKernelData111_MERGE_CHANNEL_4CH, 4},
{4, 10, 640, 0, 0, gScenarioBufferData112_MERGE_CHANNEL_4CH, 80, gScenarioKernelData112_MERGE_CHANNEL_4CH, 4},
{6, 10, 960, 0, 0, gScenarioBufferData113_MERGE_CHANNEL_4CH, 80, gScenarioKernelData113_MERGE_CHANNEL_4CH, 4},
{8, 10, 1280, 0, 0, gScenarioBufferData114_MERGE_CHANNEL_4CH, 80, gScenarioKernelData114_MERGE_CHANNEL_4CH, 4},
{10, 10, 1600, 0, 0, gScenarioBufferData115_MERGE_CHANNEL_4CH, 80, gScenarioKernelData115_MERGE_CHANNEL_4CH, 4},
{12, 10, 1920, 0, 0, gScenarioBufferData116_MERGE_CHANNEL_4CH, 80, gScenarioKernelData116_MERGE_CHANNEL_4CH, 4},
{14, 10, 2240, 0, 0, gScenarioBufferData117_MERGE_CHANNEL_4CH, 80, gScenarioKernelData117_MERGE_CHANNEL_4CH, 4},
{16, 10, 2560, 0, 0, gScenarioBufferData118_MERGE_CHANNEL_4CH, 80, gScenarioKernelData118_MERGE_CHANNEL_4CH, 4},
{18, 10, 2880, 0, 0, gScenarioBufferData119_MERGE_CHANNEL_4CH, 80, gScenarioKernelData119_MERGE_CHANNEL_4CH, 4},
{20, 10, 3200, 0, 0, gScenarioBufferData120_MERGE_CHANNEL_4CH, 80, gScenarioKernelData120_MERGE_CHANNEL_4CH, 4},
{22, 10, 3520, 0, 0, gScenarioBufferData121_MERGE_CHANNEL_4CH, 80, gScenarioKernelData121_MERGE_CHANNEL_4CH, 4},
{24, 10, 3840, 0, 0, gScenarioBufferData122_MERGE_CHANNEL_4CH, 80, gScenarioKernelData122_MERGE_CHANNEL_4CH, 4},
{2, 12, 384, 0, 0, gScenarioBufferData123_MERGE_CHANNEL_4CH, 80, gScenarioKernelData123_MERGE_CHANNEL_4CH, 4},
{4, 12, 768, 0, 0, gScenarioBufferData124_MERGE_CHANNEL_4CH, 80, gScenarioKernelData124_MERGE_CHANNEL_4CH, 4},
{6, 12, 1152, 0, 0, gScenarioBufferData125_MERGE_CHANNEL_4CH, 80, gScenarioKernelData125_MERGE_CHANNEL_4CH, 4},
{8, 12, 1536, 0, 0, gScenarioBufferData126_MERGE_CHANNEL_4CH, 80, gScenarioKernelData126_MERGE_CHANNEL_4CH, 4},
{10, 12, 1920, 0, 0, gScenarioBufferData127_MERGE_CHANNEL_4CH, 80, gScenarioKernelData127_MERGE_CHANNEL_4CH, 4},
{12, 12, 2304, 0, 0, gScenarioBufferData128_MERGE_CHANNEL_4CH, 80, gScenarioKernelData128_MERGE_CHANNEL_4CH, 4},
{14, 12, 2688, 0, 0, gScenarioBufferData129_MERGE_CHANNEL_4CH, 80, gScenarioKernelData129_MERGE_CHANNEL_4CH, 4},
{16, 12, 3072, 0, 0, gScenarioBufferData130_MERGE_CHANNEL_4CH, 80, gScenarioKernelData130_MERGE_CHANNEL_4CH, 4},
{18, 12, 3456, 0, 0, gScenarioBufferData131_MERGE_CHANNEL_4CH, 80, gScenarioKernelData131_MERGE_CHANNEL_4CH, 4},
{20, 12, 3840, 0, 0, gScenarioBufferData132_MERGE_CHANNEL_4CH, 80, gScenarioKernelData132_MERGE_CHANNEL_4CH, 4},
{2, 14, 448, 0, 0, gScenarioBufferData133_MERGE_CHANNEL_4CH, 80, gScenarioKernelData133_MERGE_CHANNEL_4CH, 4},
{4, 14, 896, 0, 0, gScenarioBufferData134_MERGE_CHANNEL_4CH, 80, gScenarioKernelData134_MERGE_CHANNEL_4CH, 4},
{6, 14, 1344, 0, 0, gScenarioBufferData135_MERGE_CHANNEL_4CH, 80, gScenarioKernelData135_MERGE_CHANNEL_4CH, 4},
{8, 14, 1792, 0, 0, gScenarioBufferData136_MERGE_CHANNEL_4CH, 80, gScenarioKernelData136_MERGE_CHANNEL_4CH, 4},
{10, 14, 2240, 0, 0, gScenarioBufferData137_MERGE_CHANNEL_4CH, 80, gScenarioKernelData137_MERGE_CHANNEL_4CH, 4},
{12, 14, 2688, 0, 0, gScenarioBufferData138_MERGE_CHANNEL_4CH, 80, gScenarioKernelData138_MERGE_CHANNEL_4CH, 4},
{14, 14, 3136, 0, 0, gScenarioBufferData139_MERGE_CHANNEL_4CH, 80, gScenarioKernelData139_MERGE_CHANNEL_4CH, 4},
{16, 14, 3584, 0, 0, gScenarioBufferData140_MERGE_CHANNEL_4CH, 80, gScenarioKernelData140_MERGE_CHANNEL_4CH, 4},
{2, 16, 512, 0, 0, gScenarioBufferData141_MERGE_CHANNEL_4CH, 80, gScenarioKernelData141_MERGE_CHANNEL_4CH, 4},
{4, 16, 1024, 0, 0, gScenarioBufferData142_MERGE_CHANNEL_4CH, 80, gScenarioKernelData142_MERGE_CHANNEL_4CH, 4},
{6, 16, 1536, 0, 0, gScenarioBufferData143_MERGE_CHANNEL_4CH, 80, gScenarioKernelData143_MERGE_CHANNEL_4CH, 4},
{8, 16, 2048, 0, 0, gScenarioBufferData144_MERGE_CHANNEL_4CH, 80, gScenarioKernelData144_MERGE_CHANNEL_4CH, 4},
{10, 16, 2560, 0, 0, gScenarioBufferData145_MERGE_CHANNEL_4CH, 80, gScenarioKernelData145_MERGE_CHANNEL_4CH, 4},
{12, 16, 3072, 0, 0, gScenarioBufferData146_MERGE_CHANNEL_4CH, 80, gScenarioKernelData146_MERGE_CHANNEL_4CH, 4},
{14, 16, 3584, 0, 0, gScenarioBufferData147_MERGE_CHANNEL_4CH, 80, gScenarioKernelData147_MERGE_CHANNEL_4CH, 4},
{2, 18, 576, 0, 0, gScenarioBufferData148_MERGE_CHANNEL_4CH, 80, gScenarioKernelData148_MERGE_CHANNEL_4CH, 4},
{4, 18, 1152, 0, 0, gScenarioBufferData149_MERGE_CHANNEL_4CH, 80, gScenarioKernelData149_MERGE_CHANNEL_4CH, 4},
{6, 18, 1728, 0, 0, gScenarioBufferData150_MERGE_CHANNEL_4CH, 80, gScenarioKernelData150_MERGE_CHANNEL_4CH, 4},
{8, 18, 2304, 0, 0, gScenarioBufferData151_MERGE_CHANNEL_4CH, 80, gScenarioKernelData151_MERGE_CHANNEL_4CH, 4},
{10, 18, 2880, 0, 0, gScenarioBufferData152_MERGE_CHANNEL_4CH, 80, gScenarioKernelData152_MERGE_CHANNEL_4CH, 4},
{12, 18, 3456, 0, 0, gScenarioBufferData153_MERGE_CHANNEL_4CH, 80, gScenarioKernelData153_MERGE_CHANNEL_4CH, 4},
{2, 20, 640, 0, 0, gScenarioBufferData154_MERGE_CHANNEL_4CH, 80, gScenarioKernelData154_MERGE_CHANNEL_4CH, 4},
{4, 20, 1280, 0, 0, gScenarioBufferData155_MERGE_CHANNEL_4CH, 80, gScenarioKernelData155_MERGE_CHANNEL_4CH, 4},
{6, 20, 1920, 0, 0, gScenarioBufferData156_MERGE_CHANNEL_4CH, 80, gScenarioKernelData156_MERGE_CHANNEL_4CH, 4},
{8, 20, 2560, 0, 0, gScenarioBufferData157_MERGE_CHANNEL_4CH, 80, gScenarioKernelData157_MERGE_CHANNEL_4CH, 4},
{10, 20, 3200, 0, 0, gScenarioBufferData158_MERGE_CHANNEL_4CH, 80, gScenarioKernelData158_MERGE_CHANNEL_4CH, 4},
{12, 20, 3840, 0, 0, gScenarioBufferData159_MERGE_CHANNEL_4CH, 80, gScenarioKernelData159_MERGE_CHANNEL_4CH, 4},
{2, 22, 704, 0, 0, gScenarioBufferData160_MERGE_CHANNEL_4CH, 80, gScenarioKernelData160_MERGE_CHANNEL_4CH, 4},
{4, 22, 1408, 0, 0, gScenarioBufferData161_MERGE_CHANNEL_4CH, 80, gScenarioKernelData161_MERGE_CHANNEL_4CH, 4},
{6, 22, 2112, 0, 0, gScenarioBufferData162_MERGE_CHANNEL_4CH, 80, gScenarioKernelData162_MERGE_CHANNEL_4CH, 4},
{8, 22, 2816, 0, 0, gScenarioBufferData163_MERGE_CHANNEL_4CH, 80, gScenarioKernelData163_MERGE_CHANNEL_4CH, 4},
{10, 22, 3520, 0, 0, gScenarioBufferData164_MERGE_CHANNEL_4CH, 80, gScenarioKernelData164_MERGE_CHANNEL_4CH, 4},
{2, 24, 768, 0, 0, gScenarioBufferData165_MERGE_CHANNEL_4CH, 80, gScenarioKernelData165_MERGE_CHANNEL_4CH, 4},
{4, 24, 1536, 0, 0, gScenarioBufferData166_MERGE_CHANNEL_4CH, 80, gScenarioKernelData166_MERGE_CHANNEL_4CH, 4},
{6, 24, 2304, 0, 0, gScenarioBufferData167_MERGE_CHANNEL_4CH, 80, gScenarioKernelData167_MERGE_CHANNEL_4CH, 4},
{8, 24, 3072, 0, 0, gScenarioBufferData168_MERGE_CHANNEL_4CH, 80, gScenarioKernelData168_MERGE_CHANNEL_4CH, 4},
{10, 24, 3840, 0, 0, gScenarioBufferData169_MERGE_CHANNEL_4CH, 80, gScenarioKernelData169_MERGE_CHANNEL_4CH, 4},
{2, 26, 832, 0, 0, gScenarioBufferData170_MERGE_CHANNEL_4CH, 80, gScenarioKernelData170_MERGE_CHANNEL_4CH, 4},
{4, 26, 1664, 0, 0, gScenarioBufferData171_MERGE_CHANNEL_4CH, 80, gScenarioKernelData171_MERGE_CHANNEL_4CH, 4},
{6, 26, 2496, 0, 0, gScenarioBufferData172_MERGE_CHANNEL_4CH, 80, gScenarioKernelData172_MERGE_CHANNEL_4CH, 4},
{8, 26, 3328, 0, 0, gScenarioBufferData173_MERGE_CHANNEL_4CH, 80, gScenarioKernelData173_MERGE_CHANNEL_4CH, 4},
{2, 28, 896, 0, 0, gScenarioBufferData174_MERGE_CHANNEL_4CH, 80, gScenarioKernelData174_MERGE_CHANNEL_4CH, 4},
{4, 28, 1792, 0, 0, gScenarioBufferData175_MERGE_CHANNEL_4CH, 80, gScenarioKernelData175_MERGE_CHANNEL_4CH, 4},
{6, 28, 2688, 0, 0, gScenarioBufferData176_MERGE_CHANNEL_4CH, 80, gScenarioKernelData176_MERGE_CHANNEL_4CH, 4},
{8, 28, 3584, 0, 0, gScenarioBufferData177_MERGE_CHANNEL_4CH, 80, gScenarioKernelData177_MERGE_CHANNEL_4CH, 4},
{2, 30, 960, 0, 0, gScenarioBufferData178_MERGE_CHANNEL_4CH, 80, gScenarioKernelData178_MERGE_CHANNEL_4CH, 4},
{4, 30, 1920, 0, 0, gScenarioBufferData179_MERGE_CHANNEL_4CH, 80, gScenarioKernelData179_MERGE_CHANNEL_4CH, 4},
{6, 30, 2880, 0, 0, gScenarioBufferData180_MERGE_CHANNEL_4CH, 80, gScenarioKernelData180_MERGE_CHANNEL_4CH, 4},
{8, 30, 3840, 0, 0, gScenarioBufferData181_MERGE_CHANNEL_4CH, 80, gScenarioKernelData181_MERGE_CHANNEL_4CH, 4},
{2, 32, 1024, 0, 0, gScenarioBufferData182_MERGE_CHANNEL_4CH, 80, gScenarioKernelData182_MERGE_CHANNEL_4CH, 4},
{4, 32, 2048, 0, 0, gScenarioBufferData183_MERGE_CHANNEL_4CH, 80, gScenarioKernelData183_MERGE_CHANNEL_4CH, 4},
{6, 32, 3072, 0, 0, gScenarioBufferData184_MERGE_CHANNEL_4CH, 80, gScenarioKernelData184_MERGE_CHANNEL_4CH, 4}
};

static acf_scenario_list gScenarioList_MERGE_CHANNEL_4CH = {
185, //number of scenarios
gScenarioArray_MERGE_CHANNEL_4CH};
//**************************************************************

class MERGE_CHANNEL_4CH : public ACF_Process_APU
{

public:
   MERGE_CHANNEL_4CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MERGE_CHANNEL_4CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MERGE_CHANNEL_4CH",
                                        MERGE_CHANNEL_4CH_LOAD_SEGMENTS,
                                        MERGE_CHANNEL_4CH_LOAD_PMEM, MERGE_CHANNEL_4CH_LOAD_PMEM_SIZE,
                                        MERGE_CHANNEL_4CH_LOAD_DMEM, MERGE_CHANNEL_4CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MERGE_CHANNEL_4CH_APEX_LOG_BUFFER, MERGE_CHANNEL_4CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("INPUT_3", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 4, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MERGE_CHANNEL_4CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MERGE_CHANNEL_4CH

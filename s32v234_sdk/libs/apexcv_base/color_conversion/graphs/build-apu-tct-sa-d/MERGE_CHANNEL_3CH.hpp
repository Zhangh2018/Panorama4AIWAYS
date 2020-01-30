#ifndef _ACF_PROCESS_APU_MERGE_CHANNEL_3CH
#define _ACF_PROCESS_APU_MERGE_CHANNEL_3CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MERGE_CHANNEL_3CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MERGE_CHANNEL_3CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MERGE_CHANNEL_3CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MERGE_CHANNEL_3CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MERGE_CHANNEL_3CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MERGE_CHANNEL_3CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MERGE_CHANNEL_3CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MERGE_CHANNEL_3CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MERGE_CHANNEL_3CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MERGE_CHANNEL_3CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MERGE_CHANNEL_3CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MERGE_CHANNEL_3CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MERGE_CHANNEL_3CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MERGE_CHANNEL_3CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MERGE_CHANNEL_3CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MERGE_CHANNEL_3CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MERGE_CHANNEL_3CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MERGE_CHANNEL_3CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MERGE_CHANNEL_3CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MERGE_CHANNEL_3CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MERGE_CHANNEL_3CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MERGE_CHANNEL_3CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MERGE_CHANNEL_3CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MERGE_CHANNEL_3CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MERGE_CHANNEL_3CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MERGE_CHANNEL_3CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MERGE_CHANNEL_3CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MERGE_CHANNEL_3CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MERGE_CHANNEL_3CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MERGE_CHANNEL_3CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MERGE_CHANNEL_3CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MERGE_CHANNEL_3CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MERGE_CHANNEL_3CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MERGE_CHANNEL_3CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MERGE_CHANNEL_3CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MERGE_CHANNEL_3CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MERGE_CHANNEL_3CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MERGE_CHANNEL_3CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MERGE_CHANNEL_3CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MERGE_CHANNEL_3CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MERGE_CHANNEL_3CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MERGE_CHANNEL_3CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MERGE_CHANNEL_3CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MERGE_CHANNEL_3CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MERGE_CHANNEL_3CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MERGE_CHANNEL_3CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MERGE_CHANNEL_3CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MERGE_CHANNEL_3CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MERGE_CHANNEL_3CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MERGE_CHANNEL_3CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MERGE_CHANNEL_3CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MERGE_CHANNEL_3CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MERGE_CHANNEL_3CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MERGE_CHANNEL_3CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MERGE_CHANNEL_3CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MERGE_CHANNEL_3CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MERGE_CHANNEL_3CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MERGE_CHANNEL_3CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MERGE_CHANNEL_3CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MERGE_CHANNEL_3CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MERGE_CHANNEL_3CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MERGE_CHANNEL_3CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MERGE_CHANNEL_3CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MERGE_CHANNEL_3CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MERGE_CHANNEL_3CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MERGE_CHANNEL_3CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MERGE_CHANNEL_3CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MERGE_CHANNEL_3CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MERGE_CHANNEL_3CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MERGE_CHANNEL_3CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MERGE_CHANNEL_3CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MERGE_CHANNEL_3CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MERGE_CHANNEL_3CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MERGE_CHANNEL_3CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MERGE_CHANNEL_3CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MERGE_CHANNEL_3CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MERGE_CHANNEL_3CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MERGE_CHANNEL_3CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MERGE_CHANNEL_3CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MERGE_CHANNEL_3CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MERGE_CHANNEL_3CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MERGE_CHANNEL_3CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MERGE_CHANNEL_3CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MERGE_CHANNEL_3CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MERGE_CHANNEL_3CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MERGE_CHANNEL_3CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MERGE_CHANNEL_3CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MERGE_CHANNEL_3CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MERGE_CHANNEL_3CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MERGE_CHANNEL_3CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MERGE_CHANNEL_3CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MERGE_CHANNEL_3CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MERGE_CHANNEL_3CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MERGE_CHANNEL_3CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MERGE_CHANNEL_3CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MERGE_CHANNEL_3CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MERGE_CHANNEL_3CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MERGE_CHANNEL_3CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MERGE_CHANNEL_3CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MERGE_CHANNEL_3CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MERGE_CHANNEL_3CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MERGE_CHANNEL_3CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MERGE_CHANNEL_3CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MERGE_CHANNEL_3CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MERGE_CHANNEL_3CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MERGE_CHANNEL_3CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MERGE_CHANNEL_3CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MERGE_CHANNEL_3CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MERGE_CHANNEL_3CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MERGE_CHANNEL_3CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MERGE_CHANNEL_3CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MERGE_CHANNEL_3CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MERGE_CHANNEL_3CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MERGE_CHANNEL_3CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MERGE_CHANNEL_3CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MERGE_CHANNEL_3CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MERGE_CHANNEL_3CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MERGE_CHANNEL_3CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MERGE_CHANNEL_3CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MERGE_CHANNEL_3CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MERGE_CHANNEL_3CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MERGE_CHANNEL_3CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MERGE_CHANNEL_3CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MERGE_CHANNEL_3CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MERGE_CHANNEL_3CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MERGE_CHANNEL_3CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MERGE_CHANNEL_3CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MERGE_CHANNEL_3CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MERGE_CHANNEL_3CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MERGE_CHANNEL_3CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MERGE_CHANNEL_3CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MERGE_CHANNEL_3CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MERGE_CHANNEL_3CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MERGE_CHANNEL_3CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MERGE_CHANNEL_3CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MERGE_CHANNEL_3CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MERGE_CHANNEL_3CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MERGE_CHANNEL_3CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MERGE_CHANNEL_3CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MERGE_CHANNEL_3CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MERGE_CHANNEL_3CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MERGE_CHANNEL_3CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MERGE_CHANNEL_3CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MERGE_CHANNEL_3CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MERGE_CHANNEL_3CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MERGE_CHANNEL_3CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MERGE_CHANNEL_3CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MERGE_CHANNEL_3CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MERGE_CHANNEL_3CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MERGE_CHANNEL_3CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MERGE_CHANNEL_3CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MERGE_CHANNEL_3CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MERGE_CHANNEL_3CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MERGE_CHANNEL_3CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MERGE_CHANNEL_3CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MERGE_CHANNEL_3CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MERGE_CHANNEL_3CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MERGE_CHANNEL_3CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MERGE_CHANNEL_3CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MERGE_CHANNEL_3CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MERGE_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MERGE_CHANNEL_3CH[] = {{0, 0}};

static acf_scenario gScenarioArray_MERGE_CHANNEL_3CH[] = {
{2, 1, 24, 0, 0, gScenarioBufferData0_MERGE_CHANNEL_3CH, 64, gScenarioKernelData0_MERGE_CHANNEL_3CH, 4},
{4, 1, 48, 0, 0, gScenarioBufferData1_MERGE_CHANNEL_3CH, 64, gScenarioKernelData1_MERGE_CHANNEL_3CH, 4},
{6, 1, 72, 0, 0, gScenarioBufferData2_MERGE_CHANNEL_3CH, 64, gScenarioKernelData2_MERGE_CHANNEL_3CH, 4},
{8, 1, 96, 0, 0, gScenarioBufferData3_MERGE_CHANNEL_3CH, 64, gScenarioKernelData3_MERGE_CHANNEL_3CH, 4},
{10, 1, 120, 0, 0, gScenarioBufferData4_MERGE_CHANNEL_3CH, 64, gScenarioKernelData4_MERGE_CHANNEL_3CH, 4},
{12, 1, 144, 0, 0, gScenarioBufferData5_MERGE_CHANNEL_3CH, 64, gScenarioKernelData5_MERGE_CHANNEL_3CH, 4},
{16, 1, 192, 0, 0, gScenarioBufferData6_MERGE_CHANNEL_3CH, 64, gScenarioKernelData6_MERGE_CHANNEL_3CH, 4},
{20, 1, 240, 0, 0, gScenarioBufferData7_MERGE_CHANNEL_3CH, 64, gScenarioKernelData7_MERGE_CHANNEL_3CH, 4},
{24, 1, 288, 0, 0, gScenarioBufferData8_MERGE_CHANNEL_3CH, 64, gScenarioKernelData8_MERGE_CHANNEL_3CH, 4},
{32, 1, 384, 0, 0, gScenarioBufferData9_MERGE_CHANNEL_3CH, 64, gScenarioKernelData9_MERGE_CHANNEL_3CH, 4},
{40, 1, 480, 0, 0, gScenarioBufferData10_MERGE_CHANNEL_3CH, 64, gScenarioKernelData10_MERGE_CHANNEL_3CH, 4},
{2, 2, 48, 0, 0, gScenarioBufferData11_MERGE_CHANNEL_3CH, 64, gScenarioKernelData11_MERGE_CHANNEL_3CH, 4},
{4, 2, 96, 0, 0, gScenarioBufferData12_MERGE_CHANNEL_3CH, 64, gScenarioKernelData12_MERGE_CHANNEL_3CH, 4},
{6, 2, 144, 0, 0, gScenarioBufferData13_MERGE_CHANNEL_3CH, 64, gScenarioKernelData13_MERGE_CHANNEL_3CH, 4},
{8, 2, 192, 0, 0, gScenarioBufferData14_MERGE_CHANNEL_3CH, 64, gScenarioKernelData14_MERGE_CHANNEL_3CH, 4},
{10, 2, 240, 0, 0, gScenarioBufferData15_MERGE_CHANNEL_3CH, 64, gScenarioKernelData15_MERGE_CHANNEL_3CH, 4},
{12, 2, 288, 0, 0, gScenarioBufferData16_MERGE_CHANNEL_3CH, 64, gScenarioKernelData16_MERGE_CHANNEL_3CH, 4},
{16, 2, 384, 0, 0, gScenarioBufferData17_MERGE_CHANNEL_3CH, 64, gScenarioKernelData17_MERGE_CHANNEL_3CH, 4},
{20, 2, 480, 0, 0, gScenarioBufferData18_MERGE_CHANNEL_3CH, 64, gScenarioKernelData18_MERGE_CHANNEL_3CH, 4},
{24, 2, 576, 0, 0, gScenarioBufferData19_MERGE_CHANNEL_3CH, 64, gScenarioKernelData19_MERGE_CHANNEL_3CH, 4},
{32, 2, 768, 0, 0, gScenarioBufferData20_MERGE_CHANNEL_3CH, 64, gScenarioKernelData20_MERGE_CHANNEL_3CH, 4},
{40, 2, 960, 0, 0, gScenarioBufferData21_MERGE_CHANNEL_3CH, 64, gScenarioKernelData21_MERGE_CHANNEL_3CH, 4},
{2, 3, 72, 0, 0, gScenarioBufferData22_MERGE_CHANNEL_3CH, 64, gScenarioKernelData22_MERGE_CHANNEL_3CH, 4},
{4, 3, 144, 0, 0, gScenarioBufferData23_MERGE_CHANNEL_3CH, 64, gScenarioKernelData23_MERGE_CHANNEL_3CH, 4},
{6, 3, 216, 0, 0, gScenarioBufferData24_MERGE_CHANNEL_3CH, 64, gScenarioKernelData24_MERGE_CHANNEL_3CH, 4},
{8, 3, 288, 0, 0, gScenarioBufferData25_MERGE_CHANNEL_3CH, 64, gScenarioKernelData25_MERGE_CHANNEL_3CH, 4},
{10, 3, 360, 0, 0, gScenarioBufferData26_MERGE_CHANNEL_3CH, 64, gScenarioKernelData26_MERGE_CHANNEL_3CH, 4},
{12, 3, 432, 0, 0, gScenarioBufferData27_MERGE_CHANNEL_3CH, 64, gScenarioKernelData27_MERGE_CHANNEL_3CH, 4},
{16, 3, 576, 0, 0, gScenarioBufferData28_MERGE_CHANNEL_3CH, 64, gScenarioKernelData28_MERGE_CHANNEL_3CH, 4},
{20, 3, 720, 0, 0, gScenarioBufferData29_MERGE_CHANNEL_3CH, 64, gScenarioKernelData29_MERGE_CHANNEL_3CH, 4},
{24, 3, 864, 0, 0, gScenarioBufferData30_MERGE_CHANNEL_3CH, 64, gScenarioKernelData30_MERGE_CHANNEL_3CH, 4},
{32, 3, 1152, 0, 0, gScenarioBufferData31_MERGE_CHANNEL_3CH, 64, gScenarioKernelData31_MERGE_CHANNEL_3CH, 4},
{40, 3, 1440, 0, 0, gScenarioBufferData32_MERGE_CHANNEL_3CH, 64, gScenarioKernelData32_MERGE_CHANNEL_3CH, 4},
{2, 4, 96, 0, 0, gScenarioBufferData33_MERGE_CHANNEL_3CH, 64, gScenarioKernelData33_MERGE_CHANNEL_3CH, 4},
{4, 4, 192, 0, 0, gScenarioBufferData34_MERGE_CHANNEL_3CH, 64, gScenarioKernelData34_MERGE_CHANNEL_3CH, 4},
{6, 4, 288, 0, 0, gScenarioBufferData35_MERGE_CHANNEL_3CH, 64, gScenarioKernelData35_MERGE_CHANNEL_3CH, 4},
{8, 4, 384, 0, 0, gScenarioBufferData36_MERGE_CHANNEL_3CH, 64, gScenarioKernelData36_MERGE_CHANNEL_3CH, 4},
{10, 4, 480, 0, 0, gScenarioBufferData37_MERGE_CHANNEL_3CH, 64, gScenarioKernelData37_MERGE_CHANNEL_3CH, 4},
{12, 4, 576, 0, 0, gScenarioBufferData38_MERGE_CHANNEL_3CH, 64, gScenarioKernelData38_MERGE_CHANNEL_3CH, 4},
{16, 4, 768, 0, 0, gScenarioBufferData39_MERGE_CHANNEL_3CH, 64, gScenarioKernelData39_MERGE_CHANNEL_3CH, 4},
{20, 4, 960, 0, 0, gScenarioBufferData40_MERGE_CHANNEL_3CH, 64, gScenarioKernelData40_MERGE_CHANNEL_3CH, 4},
{24, 4, 1152, 0, 0, gScenarioBufferData41_MERGE_CHANNEL_3CH, 64, gScenarioKernelData41_MERGE_CHANNEL_3CH, 4},
{32, 4, 1536, 0, 0, gScenarioBufferData42_MERGE_CHANNEL_3CH, 64, gScenarioKernelData42_MERGE_CHANNEL_3CH, 4},
{40, 4, 1920, 0, 0, gScenarioBufferData43_MERGE_CHANNEL_3CH, 64, gScenarioKernelData43_MERGE_CHANNEL_3CH, 4},
{2, 5, 120, 0, 0, gScenarioBufferData44_MERGE_CHANNEL_3CH, 64, gScenarioKernelData44_MERGE_CHANNEL_3CH, 4},
{4, 5, 240, 0, 0, gScenarioBufferData45_MERGE_CHANNEL_3CH, 64, gScenarioKernelData45_MERGE_CHANNEL_3CH, 4},
{6, 5, 360, 0, 0, gScenarioBufferData46_MERGE_CHANNEL_3CH, 64, gScenarioKernelData46_MERGE_CHANNEL_3CH, 4},
{8, 5, 480, 0, 0, gScenarioBufferData47_MERGE_CHANNEL_3CH, 64, gScenarioKernelData47_MERGE_CHANNEL_3CH, 4},
{10, 5, 600, 0, 0, gScenarioBufferData48_MERGE_CHANNEL_3CH, 64, gScenarioKernelData48_MERGE_CHANNEL_3CH, 4},
{12, 5, 720, 0, 0, gScenarioBufferData49_MERGE_CHANNEL_3CH, 64, gScenarioKernelData49_MERGE_CHANNEL_3CH, 4},
{16, 5, 960, 0, 0, gScenarioBufferData50_MERGE_CHANNEL_3CH, 64, gScenarioKernelData50_MERGE_CHANNEL_3CH, 4},
{20, 5, 1200, 0, 0, gScenarioBufferData51_MERGE_CHANNEL_3CH, 64, gScenarioKernelData51_MERGE_CHANNEL_3CH, 4},
{24, 5, 1440, 0, 0, gScenarioBufferData52_MERGE_CHANNEL_3CH, 64, gScenarioKernelData52_MERGE_CHANNEL_3CH, 4},
{32, 5, 1920, 0, 0, gScenarioBufferData53_MERGE_CHANNEL_3CH, 64, gScenarioKernelData53_MERGE_CHANNEL_3CH, 4},
{40, 5, 2400, 0, 0, gScenarioBufferData54_MERGE_CHANNEL_3CH, 64, gScenarioKernelData54_MERGE_CHANNEL_3CH, 4},
{2, 6, 144, 0, 0, gScenarioBufferData55_MERGE_CHANNEL_3CH, 64, gScenarioKernelData55_MERGE_CHANNEL_3CH, 4},
{4, 6, 288, 0, 0, gScenarioBufferData56_MERGE_CHANNEL_3CH, 64, gScenarioKernelData56_MERGE_CHANNEL_3CH, 4},
{6, 6, 432, 0, 0, gScenarioBufferData57_MERGE_CHANNEL_3CH, 64, gScenarioKernelData57_MERGE_CHANNEL_3CH, 4},
{8, 6, 576, 0, 0, gScenarioBufferData58_MERGE_CHANNEL_3CH, 64, gScenarioKernelData58_MERGE_CHANNEL_3CH, 4},
{10, 6, 720, 0, 0, gScenarioBufferData59_MERGE_CHANNEL_3CH, 64, gScenarioKernelData59_MERGE_CHANNEL_3CH, 4},
{12, 6, 864, 0, 0, gScenarioBufferData60_MERGE_CHANNEL_3CH, 64, gScenarioKernelData60_MERGE_CHANNEL_3CH, 4},
{16, 6, 1152, 0, 0, gScenarioBufferData61_MERGE_CHANNEL_3CH, 64, gScenarioKernelData61_MERGE_CHANNEL_3CH, 4},
{20, 6, 1440, 0, 0, gScenarioBufferData62_MERGE_CHANNEL_3CH, 64, gScenarioKernelData62_MERGE_CHANNEL_3CH, 4},
{24, 6, 1728, 0, 0, gScenarioBufferData63_MERGE_CHANNEL_3CH, 64, gScenarioKernelData63_MERGE_CHANNEL_3CH, 4},
{32, 6, 2304, 0, 0, gScenarioBufferData64_MERGE_CHANNEL_3CH, 64, gScenarioKernelData64_MERGE_CHANNEL_3CH, 4},
{40, 6, 2880, 0, 0, gScenarioBufferData65_MERGE_CHANNEL_3CH, 64, gScenarioKernelData65_MERGE_CHANNEL_3CH, 4},
{2, 8, 192, 0, 0, gScenarioBufferData66_MERGE_CHANNEL_3CH, 64, gScenarioKernelData66_MERGE_CHANNEL_3CH, 4},
{4, 8, 384, 0, 0, gScenarioBufferData67_MERGE_CHANNEL_3CH, 64, gScenarioKernelData67_MERGE_CHANNEL_3CH, 4},
{6, 8, 576, 0, 0, gScenarioBufferData68_MERGE_CHANNEL_3CH, 64, gScenarioKernelData68_MERGE_CHANNEL_3CH, 4},
{8, 8, 768, 0, 0, gScenarioBufferData69_MERGE_CHANNEL_3CH, 64, gScenarioKernelData69_MERGE_CHANNEL_3CH, 4},
{10, 8, 960, 0, 0, gScenarioBufferData70_MERGE_CHANNEL_3CH, 64, gScenarioKernelData70_MERGE_CHANNEL_3CH, 4},
{12, 8, 1152, 0, 0, gScenarioBufferData71_MERGE_CHANNEL_3CH, 64, gScenarioKernelData71_MERGE_CHANNEL_3CH, 4},
{16, 8, 1536, 0, 0, gScenarioBufferData72_MERGE_CHANNEL_3CH, 64, gScenarioKernelData72_MERGE_CHANNEL_3CH, 4},
{20, 8, 1920, 0, 0, gScenarioBufferData73_MERGE_CHANNEL_3CH, 64, gScenarioKernelData73_MERGE_CHANNEL_3CH, 4},
{24, 8, 2304, 0, 0, gScenarioBufferData74_MERGE_CHANNEL_3CH, 64, gScenarioKernelData74_MERGE_CHANNEL_3CH, 4},
{32, 8, 3072, 0, 0, gScenarioBufferData75_MERGE_CHANNEL_3CH, 64, gScenarioKernelData75_MERGE_CHANNEL_3CH, 4},
{40, 8, 3840, 0, 0, gScenarioBufferData76_MERGE_CHANNEL_3CH, 64, gScenarioKernelData76_MERGE_CHANNEL_3CH, 4},
{2, 10, 240, 0, 0, gScenarioBufferData77_MERGE_CHANNEL_3CH, 64, gScenarioKernelData77_MERGE_CHANNEL_3CH, 4},
{4, 10, 480, 0, 0, gScenarioBufferData78_MERGE_CHANNEL_3CH, 64, gScenarioKernelData78_MERGE_CHANNEL_3CH, 4},
{6, 10, 720, 0, 0, gScenarioBufferData79_MERGE_CHANNEL_3CH, 64, gScenarioKernelData79_MERGE_CHANNEL_3CH, 4},
{8, 10, 960, 0, 0, gScenarioBufferData80_MERGE_CHANNEL_3CH, 64, gScenarioKernelData80_MERGE_CHANNEL_3CH, 4},
{10, 10, 1200, 0, 0, gScenarioBufferData81_MERGE_CHANNEL_3CH, 64, gScenarioKernelData81_MERGE_CHANNEL_3CH, 4},
{12, 10, 1440, 0, 0, gScenarioBufferData82_MERGE_CHANNEL_3CH, 64, gScenarioKernelData82_MERGE_CHANNEL_3CH, 4},
{16, 10, 1920, 0, 0, gScenarioBufferData83_MERGE_CHANNEL_3CH, 64, gScenarioKernelData83_MERGE_CHANNEL_3CH, 4},
{20, 10, 2400, 0, 0, gScenarioBufferData84_MERGE_CHANNEL_3CH, 64, gScenarioKernelData84_MERGE_CHANNEL_3CH, 4},
{24, 10, 2880, 0, 0, gScenarioBufferData85_MERGE_CHANNEL_3CH, 64, gScenarioKernelData85_MERGE_CHANNEL_3CH, 4},
{32, 10, 3840, 0, 0, gScenarioBufferData86_MERGE_CHANNEL_3CH, 64, gScenarioKernelData86_MERGE_CHANNEL_3CH, 4},
{2, 12, 288, 0, 0, gScenarioBufferData87_MERGE_CHANNEL_3CH, 64, gScenarioKernelData87_MERGE_CHANNEL_3CH, 4},
{4, 12, 576, 0, 0, gScenarioBufferData88_MERGE_CHANNEL_3CH, 64, gScenarioKernelData88_MERGE_CHANNEL_3CH, 4},
{6, 12, 864, 0, 0, gScenarioBufferData89_MERGE_CHANNEL_3CH, 64, gScenarioKernelData89_MERGE_CHANNEL_3CH, 4},
{8, 12, 1152, 0, 0, gScenarioBufferData90_MERGE_CHANNEL_3CH, 64, gScenarioKernelData90_MERGE_CHANNEL_3CH, 4},
{10, 12, 1440, 0, 0, gScenarioBufferData91_MERGE_CHANNEL_3CH, 64, gScenarioKernelData91_MERGE_CHANNEL_3CH, 4},
{12, 12, 1728, 0, 0, gScenarioBufferData92_MERGE_CHANNEL_3CH, 64, gScenarioKernelData92_MERGE_CHANNEL_3CH, 4},
{16, 12, 2304, 0, 0, gScenarioBufferData93_MERGE_CHANNEL_3CH, 64, gScenarioKernelData93_MERGE_CHANNEL_3CH, 4},
{20, 12, 2880, 0, 0, gScenarioBufferData94_MERGE_CHANNEL_3CH, 64, gScenarioKernelData94_MERGE_CHANNEL_3CH, 4},
{24, 12, 3456, 0, 0, gScenarioBufferData95_MERGE_CHANNEL_3CH, 64, gScenarioKernelData95_MERGE_CHANNEL_3CH, 4},
{2, 14, 336, 0, 0, gScenarioBufferData96_MERGE_CHANNEL_3CH, 64, gScenarioKernelData96_MERGE_CHANNEL_3CH, 4},
{4, 14, 672, 0, 0, gScenarioBufferData97_MERGE_CHANNEL_3CH, 64, gScenarioKernelData97_MERGE_CHANNEL_3CH, 4},
{6, 14, 1008, 0, 0, gScenarioBufferData98_MERGE_CHANNEL_3CH, 64, gScenarioKernelData98_MERGE_CHANNEL_3CH, 4},
{8, 14, 1344, 0, 0, gScenarioBufferData99_MERGE_CHANNEL_3CH, 64, gScenarioKernelData99_MERGE_CHANNEL_3CH, 4},
{10, 14, 1680, 0, 0, gScenarioBufferData100_MERGE_CHANNEL_3CH, 64, gScenarioKernelData100_MERGE_CHANNEL_3CH, 4},
{12, 14, 2016, 0, 0, gScenarioBufferData101_MERGE_CHANNEL_3CH, 64, gScenarioKernelData101_MERGE_CHANNEL_3CH, 4},
{16, 14, 2688, 0, 0, gScenarioBufferData102_MERGE_CHANNEL_3CH, 64, gScenarioKernelData102_MERGE_CHANNEL_3CH, 4},
{20, 14, 3360, 0, 0, gScenarioBufferData103_MERGE_CHANNEL_3CH, 64, gScenarioKernelData103_MERGE_CHANNEL_3CH, 4},
{2, 16, 384, 0, 0, gScenarioBufferData104_MERGE_CHANNEL_3CH, 64, gScenarioKernelData104_MERGE_CHANNEL_3CH, 4},
{4, 16, 768, 0, 0, gScenarioBufferData105_MERGE_CHANNEL_3CH, 64, gScenarioKernelData105_MERGE_CHANNEL_3CH, 4},
{6, 16, 1152, 0, 0, gScenarioBufferData106_MERGE_CHANNEL_3CH, 64, gScenarioKernelData106_MERGE_CHANNEL_3CH, 4},
{8, 16, 1536, 0, 0, gScenarioBufferData107_MERGE_CHANNEL_3CH, 64, gScenarioKernelData107_MERGE_CHANNEL_3CH, 4},
{10, 16, 1920, 0, 0, gScenarioBufferData108_MERGE_CHANNEL_3CH, 64, gScenarioKernelData108_MERGE_CHANNEL_3CH, 4},
{12, 16, 2304, 0, 0, gScenarioBufferData109_MERGE_CHANNEL_3CH, 64, gScenarioKernelData109_MERGE_CHANNEL_3CH, 4},
{16, 16, 3072, 0, 0, gScenarioBufferData110_MERGE_CHANNEL_3CH, 64, gScenarioKernelData110_MERGE_CHANNEL_3CH, 4},
{20, 16, 3840, 0, 0, gScenarioBufferData111_MERGE_CHANNEL_3CH, 64, gScenarioKernelData111_MERGE_CHANNEL_3CH, 4},
{2, 18, 432, 0, 0, gScenarioBufferData112_MERGE_CHANNEL_3CH, 64, gScenarioKernelData112_MERGE_CHANNEL_3CH, 4},
{4, 18, 864, 0, 0, gScenarioBufferData113_MERGE_CHANNEL_3CH, 64, gScenarioKernelData113_MERGE_CHANNEL_3CH, 4},
{6, 18, 1296, 0, 0, gScenarioBufferData114_MERGE_CHANNEL_3CH, 64, gScenarioKernelData114_MERGE_CHANNEL_3CH, 4},
{8, 18, 1728, 0, 0, gScenarioBufferData115_MERGE_CHANNEL_3CH, 64, gScenarioKernelData115_MERGE_CHANNEL_3CH, 4},
{10, 18, 2160, 0, 0, gScenarioBufferData116_MERGE_CHANNEL_3CH, 64, gScenarioKernelData116_MERGE_CHANNEL_3CH, 4},
{12, 18, 2592, 0, 0, gScenarioBufferData117_MERGE_CHANNEL_3CH, 64, gScenarioKernelData117_MERGE_CHANNEL_3CH, 4},
{16, 18, 3456, 0, 0, gScenarioBufferData118_MERGE_CHANNEL_3CH, 64, gScenarioKernelData118_MERGE_CHANNEL_3CH, 4},
{2, 20, 480, 0, 0, gScenarioBufferData119_MERGE_CHANNEL_3CH, 64, gScenarioKernelData119_MERGE_CHANNEL_3CH, 4},
{4, 20, 960, 0, 0, gScenarioBufferData120_MERGE_CHANNEL_3CH, 64, gScenarioKernelData120_MERGE_CHANNEL_3CH, 4},
{6, 20, 1440, 0, 0, gScenarioBufferData121_MERGE_CHANNEL_3CH, 64, gScenarioKernelData121_MERGE_CHANNEL_3CH, 4},
{8, 20, 1920, 0, 0, gScenarioBufferData122_MERGE_CHANNEL_3CH, 64, gScenarioKernelData122_MERGE_CHANNEL_3CH, 4},
{10, 20, 2400, 0, 0, gScenarioBufferData123_MERGE_CHANNEL_3CH, 64, gScenarioKernelData123_MERGE_CHANNEL_3CH, 4},
{12, 20, 2880, 0, 0, gScenarioBufferData124_MERGE_CHANNEL_3CH, 64, gScenarioKernelData124_MERGE_CHANNEL_3CH, 4},
{16, 20, 3840, 0, 0, gScenarioBufferData125_MERGE_CHANNEL_3CH, 64, gScenarioKernelData125_MERGE_CHANNEL_3CH, 4},
{2, 22, 528, 0, 0, gScenarioBufferData126_MERGE_CHANNEL_3CH, 64, gScenarioKernelData126_MERGE_CHANNEL_3CH, 4},
{4, 22, 1056, 0, 0, gScenarioBufferData127_MERGE_CHANNEL_3CH, 64, gScenarioKernelData127_MERGE_CHANNEL_3CH, 4},
{6, 22, 1584, 0, 0, gScenarioBufferData128_MERGE_CHANNEL_3CH, 64, gScenarioKernelData128_MERGE_CHANNEL_3CH, 4},
{8, 22, 2112, 0, 0, gScenarioBufferData129_MERGE_CHANNEL_3CH, 64, gScenarioKernelData129_MERGE_CHANNEL_3CH, 4},
{10, 22, 2640, 0, 0, gScenarioBufferData130_MERGE_CHANNEL_3CH, 64, gScenarioKernelData130_MERGE_CHANNEL_3CH, 4},
{12, 22, 3168, 0, 0, gScenarioBufferData131_MERGE_CHANNEL_3CH, 64, gScenarioKernelData131_MERGE_CHANNEL_3CH, 4},
{2, 24, 576, 0, 0, gScenarioBufferData132_MERGE_CHANNEL_3CH, 64, gScenarioKernelData132_MERGE_CHANNEL_3CH, 4},
{4, 24, 1152, 0, 0, gScenarioBufferData133_MERGE_CHANNEL_3CH, 64, gScenarioKernelData133_MERGE_CHANNEL_3CH, 4},
{6, 24, 1728, 0, 0, gScenarioBufferData134_MERGE_CHANNEL_3CH, 64, gScenarioKernelData134_MERGE_CHANNEL_3CH, 4},
{8, 24, 2304, 0, 0, gScenarioBufferData135_MERGE_CHANNEL_3CH, 64, gScenarioKernelData135_MERGE_CHANNEL_3CH, 4},
{10, 24, 2880, 0, 0, gScenarioBufferData136_MERGE_CHANNEL_3CH, 64, gScenarioKernelData136_MERGE_CHANNEL_3CH, 4},
{12, 24, 3456, 0, 0, gScenarioBufferData137_MERGE_CHANNEL_3CH, 64, gScenarioKernelData137_MERGE_CHANNEL_3CH, 4},
{2, 26, 624, 0, 0, gScenarioBufferData138_MERGE_CHANNEL_3CH, 64, gScenarioKernelData138_MERGE_CHANNEL_3CH, 4},
{4, 26, 1248, 0, 0, gScenarioBufferData139_MERGE_CHANNEL_3CH, 64, gScenarioKernelData139_MERGE_CHANNEL_3CH, 4},
{6, 26, 1872, 0, 0, gScenarioBufferData140_MERGE_CHANNEL_3CH, 64, gScenarioKernelData140_MERGE_CHANNEL_3CH, 4},
{8, 26, 2496, 0, 0, gScenarioBufferData141_MERGE_CHANNEL_3CH, 64, gScenarioKernelData141_MERGE_CHANNEL_3CH, 4},
{10, 26, 3120, 0, 0, gScenarioBufferData142_MERGE_CHANNEL_3CH, 64, gScenarioKernelData142_MERGE_CHANNEL_3CH, 4},
{12, 26, 3744, 0, 0, gScenarioBufferData143_MERGE_CHANNEL_3CH, 64, gScenarioKernelData143_MERGE_CHANNEL_3CH, 4},
{2, 28, 672, 0, 0, gScenarioBufferData144_MERGE_CHANNEL_3CH, 64, gScenarioKernelData144_MERGE_CHANNEL_3CH, 4},
{4, 28, 1344, 0, 0, gScenarioBufferData145_MERGE_CHANNEL_3CH, 64, gScenarioKernelData145_MERGE_CHANNEL_3CH, 4},
{6, 28, 2016, 0, 0, gScenarioBufferData146_MERGE_CHANNEL_3CH, 64, gScenarioKernelData146_MERGE_CHANNEL_3CH, 4},
{8, 28, 2688, 0, 0, gScenarioBufferData147_MERGE_CHANNEL_3CH, 64, gScenarioKernelData147_MERGE_CHANNEL_3CH, 4},
{10, 28, 3360, 0, 0, gScenarioBufferData148_MERGE_CHANNEL_3CH, 64, gScenarioKernelData148_MERGE_CHANNEL_3CH, 4},
{2, 30, 720, 0, 0, gScenarioBufferData149_MERGE_CHANNEL_3CH, 64, gScenarioKernelData149_MERGE_CHANNEL_3CH, 4},
{4, 30, 1440, 0, 0, gScenarioBufferData150_MERGE_CHANNEL_3CH, 64, gScenarioKernelData150_MERGE_CHANNEL_3CH, 4},
{6, 30, 2160, 0, 0, gScenarioBufferData151_MERGE_CHANNEL_3CH, 64, gScenarioKernelData151_MERGE_CHANNEL_3CH, 4},
{8, 30, 2880, 0, 0, gScenarioBufferData152_MERGE_CHANNEL_3CH, 64, gScenarioKernelData152_MERGE_CHANNEL_3CH, 4},
{10, 30, 3600, 0, 0, gScenarioBufferData153_MERGE_CHANNEL_3CH, 64, gScenarioKernelData153_MERGE_CHANNEL_3CH, 4},
{2, 32, 768, 0, 0, gScenarioBufferData154_MERGE_CHANNEL_3CH, 64, gScenarioKernelData154_MERGE_CHANNEL_3CH, 4},
{4, 32, 1536, 0, 0, gScenarioBufferData155_MERGE_CHANNEL_3CH, 64, gScenarioKernelData155_MERGE_CHANNEL_3CH, 4},
{6, 32, 2304, 0, 0, gScenarioBufferData156_MERGE_CHANNEL_3CH, 64, gScenarioKernelData156_MERGE_CHANNEL_3CH, 4},
{8, 32, 3072, 0, 0, gScenarioBufferData157_MERGE_CHANNEL_3CH, 64, gScenarioKernelData157_MERGE_CHANNEL_3CH, 4},
{10, 32, 3840, 0, 0, gScenarioBufferData158_MERGE_CHANNEL_3CH, 64, gScenarioKernelData158_MERGE_CHANNEL_3CH, 4}
};

static acf_scenario_list gScenarioList_MERGE_CHANNEL_3CH = {
159, //number of scenarios
gScenarioArray_MERGE_CHANNEL_3CH};
//**************************************************************

class MERGE_CHANNEL_3CH : public ACF_Process_APU
{

public:
   MERGE_CHANNEL_3CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MERGE_CHANNEL_3CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MERGE_CHANNEL_3CH",
                                        MERGE_CHANNEL_3CH_LOAD_SEGMENTS,
                                        MERGE_CHANNEL_3CH_LOAD_PMEM, MERGE_CHANNEL_3CH_LOAD_PMEM_SIZE,
                                        MERGE_CHANNEL_3CH_LOAD_DMEM, MERGE_CHANNEL_3CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MERGE_CHANNEL_3CH_APEX_LOG_BUFFER, MERGE_CHANNEL_3CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MERGE_CHANNEL_3CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MERGE_CHANNEL_3CH

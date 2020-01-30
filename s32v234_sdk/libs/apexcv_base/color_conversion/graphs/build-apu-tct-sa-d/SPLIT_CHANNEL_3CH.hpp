#ifndef _ACF_PROCESS_APU_SPLIT_CHANNEL_3CH
#define _ACF_PROCESS_APU_SPLIT_CHANNEL_3CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <SPLIT_CHANNEL_3CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_SPLIT_CHANNEL_3CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_SPLIT_CHANNEL_3CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_SPLIT_CHANNEL_3CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_SPLIT_CHANNEL_3CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_SPLIT_CHANNEL_3CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_SPLIT_CHANNEL_3CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_SPLIT_CHANNEL_3CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_SPLIT_CHANNEL_3CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_SPLIT_CHANNEL_3CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_SPLIT_CHANNEL_3CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_SPLIT_CHANNEL_3CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_SPLIT_CHANNEL_3CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_SPLIT_CHANNEL_3CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_SPLIT_CHANNEL_3CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_SPLIT_CHANNEL_3CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_SPLIT_CHANNEL_3CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_SPLIT_CHANNEL_3CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_SPLIT_CHANNEL_3CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_SPLIT_CHANNEL_3CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_SPLIT_CHANNEL_3CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_SPLIT_CHANNEL_3CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_SPLIT_CHANNEL_3CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_SPLIT_CHANNEL_3CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_SPLIT_CHANNEL_3CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_SPLIT_CHANNEL_3CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_SPLIT_CHANNEL_3CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_SPLIT_CHANNEL_3CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_SPLIT_CHANNEL_3CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_SPLIT_CHANNEL_3CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_SPLIT_CHANNEL_3CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_SPLIT_CHANNEL_3CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_SPLIT_CHANNEL_3CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_SPLIT_CHANNEL_3CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_SPLIT_CHANNEL_3CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_SPLIT_CHANNEL_3CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_SPLIT_CHANNEL_3CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_SPLIT_CHANNEL_3CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_SPLIT_CHANNEL_3CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_SPLIT_CHANNEL_3CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_SPLIT_CHANNEL_3CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_SPLIT_CHANNEL_3CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_SPLIT_CHANNEL_3CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_SPLIT_CHANNEL_3CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_SPLIT_CHANNEL_3CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_SPLIT_CHANNEL_3CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_SPLIT_CHANNEL_3CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_SPLIT_CHANNEL_3CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_SPLIT_CHANNEL_3CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_SPLIT_CHANNEL_3CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_SPLIT_CHANNEL_3CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_SPLIT_CHANNEL_3CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_SPLIT_CHANNEL_3CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_SPLIT_CHANNEL_3CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_SPLIT_CHANNEL_3CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_SPLIT_CHANNEL_3CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_SPLIT_CHANNEL_3CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_SPLIT_CHANNEL_3CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_SPLIT_CHANNEL_3CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_SPLIT_CHANNEL_3CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_SPLIT_CHANNEL_3CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_SPLIT_CHANNEL_3CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_SPLIT_CHANNEL_3CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_SPLIT_CHANNEL_3CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_SPLIT_CHANNEL_3CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_SPLIT_CHANNEL_3CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_SPLIT_CHANNEL_3CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_SPLIT_CHANNEL_3CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_SPLIT_CHANNEL_3CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_SPLIT_CHANNEL_3CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_SPLIT_CHANNEL_3CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_SPLIT_CHANNEL_3CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_SPLIT_CHANNEL_3CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_SPLIT_CHANNEL_3CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_SPLIT_CHANNEL_3CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_SPLIT_CHANNEL_3CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_SPLIT_CHANNEL_3CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_SPLIT_CHANNEL_3CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_SPLIT_CHANNEL_3CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_SPLIT_CHANNEL_3CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_SPLIT_CHANNEL_3CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_SPLIT_CHANNEL_3CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_SPLIT_CHANNEL_3CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_SPLIT_CHANNEL_3CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_SPLIT_CHANNEL_3CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_SPLIT_CHANNEL_3CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_SPLIT_CHANNEL_3CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_SPLIT_CHANNEL_3CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_SPLIT_CHANNEL_3CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_SPLIT_CHANNEL_3CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_SPLIT_CHANNEL_3CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_SPLIT_CHANNEL_3CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_SPLIT_CHANNEL_3CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_SPLIT_CHANNEL_3CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_SPLIT_CHANNEL_3CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_SPLIT_CHANNEL_3CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_SPLIT_CHANNEL_3CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_SPLIT_CHANNEL_3CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_SPLIT_CHANNEL_3CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_SPLIT_CHANNEL_3CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_SPLIT_CHANNEL_3CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_SPLIT_CHANNEL_3CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_SPLIT_CHANNEL_3CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_SPLIT_CHANNEL_3CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_SPLIT_CHANNEL_3CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_SPLIT_CHANNEL_3CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_SPLIT_CHANNEL_3CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_SPLIT_CHANNEL_3CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_SPLIT_CHANNEL_3CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_SPLIT_CHANNEL_3CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_SPLIT_CHANNEL_3CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_SPLIT_CHANNEL_3CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_SPLIT_CHANNEL_3CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_SPLIT_CHANNEL_3CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_SPLIT_CHANNEL_3CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_SPLIT_CHANNEL_3CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_SPLIT_CHANNEL_3CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_SPLIT_CHANNEL_3CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_SPLIT_CHANNEL_3CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_SPLIT_CHANNEL_3CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_SPLIT_CHANNEL_3CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_SPLIT_CHANNEL_3CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_SPLIT_CHANNEL_3CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_SPLIT_CHANNEL_3CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_SPLIT_CHANNEL_3CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_SPLIT_CHANNEL_3CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_SPLIT_CHANNEL_3CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_SPLIT_CHANNEL_3CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_SPLIT_CHANNEL_3CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_SPLIT_CHANNEL_3CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_SPLIT_CHANNEL_3CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_SPLIT_CHANNEL_3CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_SPLIT_CHANNEL_3CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_SPLIT_CHANNEL_3CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_SPLIT_CHANNEL_3CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_SPLIT_CHANNEL_3CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_SPLIT_CHANNEL_3CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_SPLIT_CHANNEL_3CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_SPLIT_CHANNEL_3CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_SPLIT_CHANNEL_3CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_SPLIT_CHANNEL_3CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_SPLIT_CHANNEL_3CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_SPLIT_CHANNEL_3CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_SPLIT_CHANNEL_3CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_SPLIT_CHANNEL_3CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_SPLIT_CHANNEL_3CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_SPLIT_CHANNEL_3CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_SPLIT_CHANNEL_3CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_SPLIT_CHANNEL_3CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_SPLIT_CHANNEL_3CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_SPLIT_CHANNEL_3CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_SPLIT_CHANNEL_3CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_SPLIT_CHANNEL_3CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_SPLIT_CHANNEL_3CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_SPLIT_CHANNEL_3CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_SPLIT_CHANNEL_3CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_SPLIT_CHANNEL_3CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_SPLIT_CHANNEL_3CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_SPLIT_CHANNEL_3CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_SPLIT_CHANNEL_3CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_SPLIT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_SPLIT_CHANNEL_3CH[] = {{0, 0}};

static acf_scenario gScenarioArray_SPLIT_CHANNEL_3CH[] = {
{2, 1, 24, 0, 0, gScenarioBufferData0_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData0_SPLIT_CHANNEL_3CH, 4},
{4, 1, 48, 0, 0, gScenarioBufferData1_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData1_SPLIT_CHANNEL_3CH, 4},
{6, 1, 72, 0, 0, gScenarioBufferData2_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData2_SPLIT_CHANNEL_3CH, 4},
{8, 1, 96, 0, 0, gScenarioBufferData3_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData3_SPLIT_CHANNEL_3CH, 4},
{10, 1, 120, 0, 0, gScenarioBufferData4_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData4_SPLIT_CHANNEL_3CH, 4},
{12, 1, 144, 0, 0, gScenarioBufferData5_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData5_SPLIT_CHANNEL_3CH, 4},
{16, 1, 192, 0, 0, gScenarioBufferData6_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData6_SPLIT_CHANNEL_3CH, 4},
{20, 1, 240, 0, 0, gScenarioBufferData7_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData7_SPLIT_CHANNEL_3CH, 4},
{24, 1, 288, 0, 0, gScenarioBufferData8_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData8_SPLIT_CHANNEL_3CH, 4},
{32, 1, 384, 0, 0, gScenarioBufferData9_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData9_SPLIT_CHANNEL_3CH, 4},
{40, 1, 480, 0, 0, gScenarioBufferData10_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData10_SPLIT_CHANNEL_3CH, 4},
{2, 2, 48, 0, 0, gScenarioBufferData11_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData11_SPLIT_CHANNEL_3CH, 4},
{4, 2, 96, 0, 0, gScenarioBufferData12_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData12_SPLIT_CHANNEL_3CH, 4},
{6, 2, 144, 0, 0, gScenarioBufferData13_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData13_SPLIT_CHANNEL_3CH, 4},
{8, 2, 192, 0, 0, gScenarioBufferData14_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData14_SPLIT_CHANNEL_3CH, 4},
{10, 2, 240, 0, 0, gScenarioBufferData15_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData15_SPLIT_CHANNEL_3CH, 4},
{12, 2, 288, 0, 0, gScenarioBufferData16_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData16_SPLIT_CHANNEL_3CH, 4},
{16, 2, 384, 0, 0, gScenarioBufferData17_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData17_SPLIT_CHANNEL_3CH, 4},
{20, 2, 480, 0, 0, gScenarioBufferData18_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData18_SPLIT_CHANNEL_3CH, 4},
{24, 2, 576, 0, 0, gScenarioBufferData19_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData19_SPLIT_CHANNEL_3CH, 4},
{32, 2, 768, 0, 0, gScenarioBufferData20_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData20_SPLIT_CHANNEL_3CH, 4},
{40, 2, 960, 0, 0, gScenarioBufferData21_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData21_SPLIT_CHANNEL_3CH, 4},
{2, 3, 72, 0, 0, gScenarioBufferData22_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData22_SPLIT_CHANNEL_3CH, 4},
{4, 3, 144, 0, 0, gScenarioBufferData23_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData23_SPLIT_CHANNEL_3CH, 4},
{6, 3, 216, 0, 0, gScenarioBufferData24_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData24_SPLIT_CHANNEL_3CH, 4},
{8, 3, 288, 0, 0, gScenarioBufferData25_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData25_SPLIT_CHANNEL_3CH, 4},
{10, 3, 360, 0, 0, gScenarioBufferData26_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData26_SPLIT_CHANNEL_3CH, 4},
{12, 3, 432, 0, 0, gScenarioBufferData27_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData27_SPLIT_CHANNEL_3CH, 4},
{16, 3, 576, 0, 0, gScenarioBufferData28_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData28_SPLIT_CHANNEL_3CH, 4},
{20, 3, 720, 0, 0, gScenarioBufferData29_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData29_SPLIT_CHANNEL_3CH, 4},
{24, 3, 864, 0, 0, gScenarioBufferData30_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData30_SPLIT_CHANNEL_3CH, 4},
{32, 3, 1152, 0, 0, gScenarioBufferData31_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData31_SPLIT_CHANNEL_3CH, 4},
{40, 3, 1440, 0, 0, gScenarioBufferData32_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData32_SPLIT_CHANNEL_3CH, 4},
{2, 4, 96, 0, 0, gScenarioBufferData33_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData33_SPLIT_CHANNEL_3CH, 4},
{4, 4, 192, 0, 0, gScenarioBufferData34_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData34_SPLIT_CHANNEL_3CH, 4},
{6, 4, 288, 0, 0, gScenarioBufferData35_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData35_SPLIT_CHANNEL_3CH, 4},
{8, 4, 384, 0, 0, gScenarioBufferData36_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData36_SPLIT_CHANNEL_3CH, 4},
{10, 4, 480, 0, 0, gScenarioBufferData37_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData37_SPLIT_CHANNEL_3CH, 4},
{12, 4, 576, 0, 0, gScenarioBufferData38_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData38_SPLIT_CHANNEL_3CH, 4},
{16, 4, 768, 0, 0, gScenarioBufferData39_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData39_SPLIT_CHANNEL_3CH, 4},
{20, 4, 960, 0, 0, gScenarioBufferData40_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData40_SPLIT_CHANNEL_3CH, 4},
{24, 4, 1152, 0, 0, gScenarioBufferData41_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData41_SPLIT_CHANNEL_3CH, 4},
{32, 4, 1536, 0, 0, gScenarioBufferData42_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData42_SPLIT_CHANNEL_3CH, 4},
{40, 4, 1920, 0, 0, gScenarioBufferData43_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData43_SPLIT_CHANNEL_3CH, 4},
{2, 5, 120, 0, 0, gScenarioBufferData44_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData44_SPLIT_CHANNEL_3CH, 4},
{4, 5, 240, 0, 0, gScenarioBufferData45_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData45_SPLIT_CHANNEL_3CH, 4},
{6, 5, 360, 0, 0, gScenarioBufferData46_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData46_SPLIT_CHANNEL_3CH, 4},
{8, 5, 480, 0, 0, gScenarioBufferData47_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData47_SPLIT_CHANNEL_3CH, 4},
{10, 5, 600, 0, 0, gScenarioBufferData48_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData48_SPLIT_CHANNEL_3CH, 4},
{12, 5, 720, 0, 0, gScenarioBufferData49_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData49_SPLIT_CHANNEL_3CH, 4},
{16, 5, 960, 0, 0, gScenarioBufferData50_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData50_SPLIT_CHANNEL_3CH, 4},
{20, 5, 1200, 0, 0, gScenarioBufferData51_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData51_SPLIT_CHANNEL_3CH, 4},
{24, 5, 1440, 0, 0, gScenarioBufferData52_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData52_SPLIT_CHANNEL_3CH, 4},
{32, 5, 1920, 0, 0, gScenarioBufferData53_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData53_SPLIT_CHANNEL_3CH, 4},
{40, 5, 2400, 0, 0, gScenarioBufferData54_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData54_SPLIT_CHANNEL_3CH, 4},
{2, 6, 144, 0, 0, gScenarioBufferData55_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData55_SPLIT_CHANNEL_3CH, 4},
{4, 6, 288, 0, 0, gScenarioBufferData56_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData56_SPLIT_CHANNEL_3CH, 4},
{6, 6, 432, 0, 0, gScenarioBufferData57_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData57_SPLIT_CHANNEL_3CH, 4},
{8, 6, 576, 0, 0, gScenarioBufferData58_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData58_SPLIT_CHANNEL_3CH, 4},
{10, 6, 720, 0, 0, gScenarioBufferData59_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData59_SPLIT_CHANNEL_3CH, 4},
{12, 6, 864, 0, 0, gScenarioBufferData60_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData60_SPLIT_CHANNEL_3CH, 4},
{16, 6, 1152, 0, 0, gScenarioBufferData61_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData61_SPLIT_CHANNEL_3CH, 4},
{20, 6, 1440, 0, 0, gScenarioBufferData62_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData62_SPLIT_CHANNEL_3CH, 4},
{24, 6, 1728, 0, 0, gScenarioBufferData63_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData63_SPLIT_CHANNEL_3CH, 4},
{32, 6, 2304, 0, 0, gScenarioBufferData64_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData64_SPLIT_CHANNEL_3CH, 4},
{40, 6, 2880, 0, 0, gScenarioBufferData65_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData65_SPLIT_CHANNEL_3CH, 4},
{2, 8, 192, 0, 0, gScenarioBufferData66_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData66_SPLIT_CHANNEL_3CH, 4},
{4, 8, 384, 0, 0, gScenarioBufferData67_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData67_SPLIT_CHANNEL_3CH, 4},
{6, 8, 576, 0, 0, gScenarioBufferData68_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData68_SPLIT_CHANNEL_3CH, 4},
{8, 8, 768, 0, 0, gScenarioBufferData69_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData69_SPLIT_CHANNEL_3CH, 4},
{10, 8, 960, 0, 0, gScenarioBufferData70_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData70_SPLIT_CHANNEL_3CH, 4},
{12, 8, 1152, 0, 0, gScenarioBufferData71_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData71_SPLIT_CHANNEL_3CH, 4},
{16, 8, 1536, 0, 0, gScenarioBufferData72_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData72_SPLIT_CHANNEL_3CH, 4},
{20, 8, 1920, 0, 0, gScenarioBufferData73_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData73_SPLIT_CHANNEL_3CH, 4},
{24, 8, 2304, 0, 0, gScenarioBufferData74_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData74_SPLIT_CHANNEL_3CH, 4},
{32, 8, 3072, 0, 0, gScenarioBufferData75_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData75_SPLIT_CHANNEL_3CH, 4},
{40, 8, 3840, 0, 0, gScenarioBufferData76_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData76_SPLIT_CHANNEL_3CH, 4},
{2, 10, 240, 0, 0, gScenarioBufferData77_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData77_SPLIT_CHANNEL_3CH, 4},
{4, 10, 480, 0, 0, gScenarioBufferData78_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData78_SPLIT_CHANNEL_3CH, 4},
{6, 10, 720, 0, 0, gScenarioBufferData79_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData79_SPLIT_CHANNEL_3CH, 4},
{8, 10, 960, 0, 0, gScenarioBufferData80_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData80_SPLIT_CHANNEL_3CH, 4},
{10, 10, 1200, 0, 0, gScenarioBufferData81_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData81_SPLIT_CHANNEL_3CH, 4},
{12, 10, 1440, 0, 0, gScenarioBufferData82_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData82_SPLIT_CHANNEL_3CH, 4},
{16, 10, 1920, 0, 0, gScenarioBufferData83_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData83_SPLIT_CHANNEL_3CH, 4},
{20, 10, 2400, 0, 0, gScenarioBufferData84_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData84_SPLIT_CHANNEL_3CH, 4},
{24, 10, 2880, 0, 0, gScenarioBufferData85_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData85_SPLIT_CHANNEL_3CH, 4},
{32, 10, 3840, 0, 0, gScenarioBufferData86_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData86_SPLIT_CHANNEL_3CH, 4},
{2, 12, 288, 0, 0, gScenarioBufferData87_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData87_SPLIT_CHANNEL_3CH, 4},
{4, 12, 576, 0, 0, gScenarioBufferData88_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData88_SPLIT_CHANNEL_3CH, 4},
{6, 12, 864, 0, 0, gScenarioBufferData89_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData89_SPLIT_CHANNEL_3CH, 4},
{8, 12, 1152, 0, 0, gScenarioBufferData90_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData90_SPLIT_CHANNEL_3CH, 4},
{10, 12, 1440, 0, 0, gScenarioBufferData91_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData91_SPLIT_CHANNEL_3CH, 4},
{12, 12, 1728, 0, 0, gScenarioBufferData92_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData92_SPLIT_CHANNEL_3CH, 4},
{16, 12, 2304, 0, 0, gScenarioBufferData93_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData93_SPLIT_CHANNEL_3CH, 4},
{20, 12, 2880, 0, 0, gScenarioBufferData94_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData94_SPLIT_CHANNEL_3CH, 4},
{24, 12, 3456, 0, 0, gScenarioBufferData95_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData95_SPLIT_CHANNEL_3CH, 4},
{2, 14, 336, 0, 0, gScenarioBufferData96_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData96_SPLIT_CHANNEL_3CH, 4},
{4, 14, 672, 0, 0, gScenarioBufferData97_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData97_SPLIT_CHANNEL_3CH, 4},
{6, 14, 1008, 0, 0, gScenarioBufferData98_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData98_SPLIT_CHANNEL_3CH, 4},
{8, 14, 1344, 0, 0, gScenarioBufferData99_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData99_SPLIT_CHANNEL_3CH, 4},
{10, 14, 1680, 0, 0, gScenarioBufferData100_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData100_SPLIT_CHANNEL_3CH, 4},
{12, 14, 2016, 0, 0, gScenarioBufferData101_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData101_SPLIT_CHANNEL_3CH, 4},
{16, 14, 2688, 0, 0, gScenarioBufferData102_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData102_SPLIT_CHANNEL_3CH, 4},
{20, 14, 3360, 0, 0, gScenarioBufferData103_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData103_SPLIT_CHANNEL_3CH, 4},
{2, 16, 384, 0, 0, gScenarioBufferData104_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData104_SPLIT_CHANNEL_3CH, 4},
{4, 16, 768, 0, 0, gScenarioBufferData105_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData105_SPLIT_CHANNEL_3CH, 4},
{6, 16, 1152, 0, 0, gScenarioBufferData106_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData106_SPLIT_CHANNEL_3CH, 4},
{8, 16, 1536, 0, 0, gScenarioBufferData107_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData107_SPLIT_CHANNEL_3CH, 4},
{10, 16, 1920, 0, 0, gScenarioBufferData108_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData108_SPLIT_CHANNEL_3CH, 4},
{12, 16, 2304, 0, 0, gScenarioBufferData109_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData109_SPLIT_CHANNEL_3CH, 4},
{16, 16, 3072, 0, 0, gScenarioBufferData110_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData110_SPLIT_CHANNEL_3CH, 4},
{20, 16, 3840, 0, 0, gScenarioBufferData111_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData111_SPLIT_CHANNEL_3CH, 4},
{2, 18, 432, 0, 0, gScenarioBufferData112_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData112_SPLIT_CHANNEL_3CH, 4},
{4, 18, 864, 0, 0, gScenarioBufferData113_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData113_SPLIT_CHANNEL_3CH, 4},
{6, 18, 1296, 0, 0, gScenarioBufferData114_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData114_SPLIT_CHANNEL_3CH, 4},
{8, 18, 1728, 0, 0, gScenarioBufferData115_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData115_SPLIT_CHANNEL_3CH, 4},
{10, 18, 2160, 0, 0, gScenarioBufferData116_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData116_SPLIT_CHANNEL_3CH, 4},
{12, 18, 2592, 0, 0, gScenarioBufferData117_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData117_SPLIT_CHANNEL_3CH, 4},
{16, 18, 3456, 0, 0, gScenarioBufferData118_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData118_SPLIT_CHANNEL_3CH, 4},
{2, 20, 480, 0, 0, gScenarioBufferData119_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData119_SPLIT_CHANNEL_3CH, 4},
{4, 20, 960, 0, 0, gScenarioBufferData120_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData120_SPLIT_CHANNEL_3CH, 4},
{6, 20, 1440, 0, 0, gScenarioBufferData121_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData121_SPLIT_CHANNEL_3CH, 4},
{8, 20, 1920, 0, 0, gScenarioBufferData122_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData122_SPLIT_CHANNEL_3CH, 4},
{10, 20, 2400, 0, 0, gScenarioBufferData123_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData123_SPLIT_CHANNEL_3CH, 4},
{12, 20, 2880, 0, 0, gScenarioBufferData124_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData124_SPLIT_CHANNEL_3CH, 4},
{16, 20, 3840, 0, 0, gScenarioBufferData125_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData125_SPLIT_CHANNEL_3CH, 4},
{2, 22, 528, 0, 0, gScenarioBufferData126_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData126_SPLIT_CHANNEL_3CH, 4},
{4, 22, 1056, 0, 0, gScenarioBufferData127_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData127_SPLIT_CHANNEL_3CH, 4},
{6, 22, 1584, 0, 0, gScenarioBufferData128_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData128_SPLIT_CHANNEL_3CH, 4},
{8, 22, 2112, 0, 0, gScenarioBufferData129_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData129_SPLIT_CHANNEL_3CH, 4},
{10, 22, 2640, 0, 0, gScenarioBufferData130_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData130_SPLIT_CHANNEL_3CH, 4},
{12, 22, 3168, 0, 0, gScenarioBufferData131_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData131_SPLIT_CHANNEL_3CH, 4},
{2, 24, 576, 0, 0, gScenarioBufferData132_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData132_SPLIT_CHANNEL_3CH, 4},
{4, 24, 1152, 0, 0, gScenarioBufferData133_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData133_SPLIT_CHANNEL_3CH, 4},
{6, 24, 1728, 0, 0, gScenarioBufferData134_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData134_SPLIT_CHANNEL_3CH, 4},
{8, 24, 2304, 0, 0, gScenarioBufferData135_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData135_SPLIT_CHANNEL_3CH, 4},
{10, 24, 2880, 0, 0, gScenarioBufferData136_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData136_SPLIT_CHANNEL_3CH, 4},
{12, 24, 3456, 0, 0, gScenarioBufferData137_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData137_SPLIT_CHANNEL_3CH, 4},
{2, 26, 624, 0, 0, gScenarioBufferData138_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData138_SPLIT_CHANNEL_3CH, 4},
{4, 26, 1248, 0, 0, gScenarioBufferData139_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData139_SPLIT_CHANNEL_3CH, 4},
{6, 26, 1872, 0, 0, gScenarioBufferData140_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData140_SPLIT_CHANNEL_3CH, 4},
{8, 26, 2496, 0, 0, gScenarioBufferData141_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData141_SPLIT_CHANNEL_3CH, 4},
{10, 26, 3120, 0, 0, gScenarioBufferData142_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData142_SPLIT_CHANNEL_3CH, 4},
{12, 26, 3744, 0, 0, gScenarioBufferData143_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData143_SPLIT_CHANNEL_3CH, 4},
{2, 28, 672, 0, 0, gScenarioBufferData144_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData144_SPLIT_CHANNEL_3CH, 4},
{4, 28, 1344, 0, 0, gScenarioBufferData145_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData145_SPLIT_CHANNEL_3CH, 4},
{6, 28, 2016, 0, 0, gScenarioBufferData146_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData146_SPLIT_CHANNEL_3CH, 4},
{8, 28, 2688, 0, 0, gScenarioBufferData147_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData147_SPLIT_CHANNEL_3CH, 4},
{10, 28, 3360, 0, 0, gScenarioBufferData148_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData148_SPLIT_CHANNEL_3CH, 4},
{2, 30, 720, 0, 0, gScenarioBufferData149_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData149_SPLIT_CHANNEL_3CH, 4},
{4, 30, 1440, 0, 0, gScenarioBufferData150_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData150_SPLIT_CHANNEL_3CH, 4},
{6, 30, 2160, 0, 0, gScenarioBufferData151_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData151_SPLIT_CHANNEL_3CH, 4},
{8, 30, 2880, 0, 0, gScenarioBufferData152_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData152_SPLIT_CHANNEL_3CH, 4},
{10, 30, 3600, 0, 0, gScenarioBufferData153_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData153_SPLIT_CHANNEL_3CH, 4},
{2, 32, 768, 0, 0, gScenarioBufferData154_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData154_SPLIT_CHANNEL_3CH, 4},
{4, 32, 1536, 0, 0, gScenarioBufferData155_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData155_SPLIT_CHANNEL_3CH, 4},
{6, 32, 2304, 0, 0, gScenarioBufferData156_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData156_SPLIT_CHANNEL_3CH, 4},
{8, 32, 3072, 0, 0, gScenarioBufferData157_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData157_SPLIT_CHANNEL_3CH, 4},
{10, 32, 3840, 0, 0, gScenarioBufferData158_SPLIT_CHANNEL_3CH, 64, gScenarioKernelData158_SPLIT_CHANNEL_3CH, 4}
};

static acf_scenario_list gScenarioList_SPLIT_CHANNEL_3CH = {
159, //number of scenarios
gScenarioArray_SPLIT_CHANNEL_3CH};
//**************************************************************

class SPLIT_CHANNEL_3CH : public ACF_Process_APU
{

public:
   SPLIT_CHANNEL_3CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("SPLIT_CHANNEL_3CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("SPLIT_CHANNEL_3CH",
                                        SPLIT_CHANNEL_3CH_LOAD_SEGMENTS,
                                        SPLIT_CHANNEL_3CH_LOAD_PMEM, SPLIT_CHANNEL_3CH_LOAD_PMEM_SIZE,
                                        SPLIT_CHANNEL_3CH_LOAD_DMEM, SPLIT_CHANNEL_3CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(SPLIT_CHANNEL_3CH_APEX_LOG_BUFFER, SPLIT_CHANNEL_3CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_SPLIT_CHANNEL_3CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_SPLIT_CHANNEL_3CH

#ifndef _ACF_PROCESS_APU_INSERT_CHANNEL_3CH
#define _ACF_PROCESS_APU_INSERT_CHANNEL_3CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INSERT_CHANNEL_3CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INSERT_CHANNEL_3CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INSERT_CHANNEL_3CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INSERT_CHANNEL_3CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INSERT_CHANNEL_3CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INSERT_CHANNEL_3CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INSERT_CHANNEL_3CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INSERT_CHANNEL_3CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INSERT_CHANNEL_3CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INSERT_CHANNEL_3CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INSERT_CHANNEL_3CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INSERT_CHANNEL_3CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INSERT_CHANNEL_3CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INSERT_CHANNEL_3CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INSERT_CHANNEL_3CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INSERT_CHANNEL_3CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INSERT_CHANNEL_3CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INSERT_CHANNEL_3CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INSERT_CHANNEL_3CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INSERT_CHANNEL_3CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INSERT_CHANNEL_3CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INSERT_CHANNEL_3CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INSERT_CHANNEL_3CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INSERT_CHANNEL_3CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INSERT_CHANNEL_3CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INSERT_CHANNEL_3CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INSERT_CHANNEL_3CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INSERT_CHANNEL_3CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INSERT_CHANNEL_3CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INSERT_CHANNEL_3CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INSERT_CHANNEL_3CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INSERT_CHANNEL_3CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INSERT_CHANNEL_3CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INSERT_CHANNEL_3CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INSERT_CHANNEL_3CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INSERT_CHANNEL_3CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INSERT_CHANNEL_3CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INSERT_CHANNEL_3CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INSERT_CHANNEL_3CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INSERT_CHANNEL_3CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INSERT_CHANNEL_3CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INSERT_CHANNEL_3CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INSERT_CHANNEL_3CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INSERT_CHANNEL_3CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INSERT_CHANNEL_3CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INSERT_CHANNEL_3CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_INSERT_CHANNEL_3CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_INSERT_CHANNEL_3CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_INSERT_CHANNEL_3CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_INSERT_CHANNEL_3CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_INSERT_CHANNEL_3CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_INSERT_CHANNEL_3CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_INSERT_CHANNEL_3CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_INSERT_CHANNEL_3CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_INSERT_CHANNEL_3CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_INSERT_CHANNEL_3CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_INSERT_CHANNEL_3CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_INSERT_CHANNEL_3CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_INSERT_CHANNEL_3CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_INSERT_CHANNEL_3CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_INSERT_CHANNEL_3CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_INSERT_CHANNEL_3CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_INSERT_CHANNEL_3CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_INSERT_CHANNEL_3CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_INSERT_CHANNEL_3CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_INSERT_CHANNEL_3CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_INSERT_CHANNEL_3CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_INSERT_CHANNEL_3CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_INSERT_CHANNEL_3CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_INSERT_CHANNEL_3CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_INSERT_CHANNEL_3CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_INSERT_CHANNEL_3CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_INSERT_CHANNEL_3CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_INSERT_CHANNEL_3CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_INSERT_CHANNEL_3CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_INSERT_CHANNEL_3CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_INSERT_CHANNEL_3CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_INSERT_CHANNEL_3CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_INSERT_CHANNEL_3CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_INSERT_CHANNEL_3CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_INSERT_CHANNEL_3CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_INSERT_CHANNEL_3CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_INSERT_CHANNEL_3CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_INSERT_CHANNEL_3CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_INSERT_CHANNEL_3CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_INSERT_CHANNEL_3CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_INSERT_CHANNEL_3CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_INSERT_CHANNEL_3CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_INSERT_CHANNEL_3CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_INSERT_CHANNEL_3CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_INSERT_CHANNEL_3CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_INSERT_CHANNEL_3CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_INSERT_CHANNEL_3CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_INSERT_CHANNEL_3CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_INSERT_CHANNEL_3CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_INSERT_CHANNEL_3CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_INSERT_CHANNEL_3CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_INSERT_CHANNEL_3CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_INSERT_CHANNEL_3CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_INSERT_CHANNEL_3CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_INSERT_CHANNEL_3CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_INSERT_CHANNEL_3CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_INSERT_CHANNEL_3CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_INSERT_CHANNEL_3CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_INSERT_CHANNEL_3CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_INSERT_CHANNEL_3CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_INSERT_CHANNEL_3CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_INSERT_CHANNEL_3CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_INSERT_CHANNEL_3CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_INSERT_CHANNEL_3CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_INSERT_CHANNEL_3CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_INSERT_CHANNEL_3CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_INSERT_CHANNEL_3CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_INSERT_CHANNEL_3CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_INSERT_CHANNEL_3CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_INSERT_CHANNEL_3CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_INSERT_CHANNEL_3CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_INSERT_CHANNEL_3CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_INSERT_CHANNEL_3CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_INSERT_CHANNEL_3CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_INSERT_CHANNEL_3CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_INSERT_CHANNEL_3CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_INSERT_CHANNEL_3CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_INSERT_CHANNEL_3CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_INSERT_CHANNEL_3CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_INSERT_CHANNEL_3CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_INSERT_CHANNEL_3CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_INSERT_CHANNEL_3CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_INSERT_CHANNEL_3CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_INSERT_CHANNEL_3CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_INSERT_CHANNEL_3CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_INSERT_CHANNEL_3CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_INSERT_CHANNEL_3CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_INSERT_CHANNEL_3CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_INSERT_CHANNEL_3CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_INSERT_CHANNEL_3CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_INSERT_CHANNEL_3CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_INSERT_CHANNEL_3CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_INSERT_CHANNEL_3CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_INSERT_CHANNEL_3CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_INSERT_CHANNEL_3CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_INSERT_CHANNEL_3CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_INSERT_CHANNEL_3CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_INSERT_CHANNEL_3CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_INSERT_CHANNEL_3CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_INSERT_CHANNEL_3CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_INSERT_CHANNEL_3CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_INSERT_CHANNEL_3CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_INSERT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_INSERT_CHANNEL_3CH[] = {{0, 0}};

static acf_scenario gScenarioArray_INSERT_CHANNEL_3CH[] = {
{2, 1, 28, 16, 0, gScenarioBufferData0_INSERT_CHANNEL_3CH, 64, gScenarioKernelData0_INSERT_CHANNEL_3CH, 4},
{4, 1, 56, 16, 0, gScenarioBufferData1_INSERT_CHANNEL_3CH, 64, gScenarioKernelData1_INSERT_CHANNEL_3CH, 4},
{6, 1, 84, 16, 0, gScenarioBufferData2_INSERT_CHANNEL_3CH, 64, gScenarioKernelData2_INSERT_CHANNEL_3CH, 4},
{8, 1, 112, 16, 0, gScenarioBufferData3_INSERT_CHANNEL_3CH, 64, gScenarioKernelData3_INSERT_CHANNEL_3CH, 4},
{10, 1, 140, 16, 0, gScenarioBufferData4_INSERT_CHANNEL_3CH, 64, gScenarioKernelData4_INSERT_CHANNEL_3CH, 4},
{12, 1, 168, 16, 0, gScenarioBufferData5_INSERT_CHANNEL_3CH, 64, gScenarioKernelData5_INSERT_CHANNEL_3CH, 4},
{16, 1, 224, 16, 0, gScenarioBufferData6_INSERT_CHANNEL_3CH, 64, gScenarioKernelData6_INSERT_CHANNEL_3CH, 4},
{20, 1, 280, 16, 0, gScenarioBufferData7_INSERT_CHANNEL_3CH, 64, gScenarioKernelData7_INSERT_CHANNEL_3CH, 4},
{24, 1, 336, 16, 0, gScenarioBufferData8_INSERT_CHANNEL_3CH, 64, gScenarioKernelData8_INSERT_CHANNEL_3CH, 4},
{32, 1, 448, 16, 0, gScenarioBufferData9_INSERT_CHANNEL_3CH, 64, gScenarioKernelData9_INSERT_CHANNEL_3CH, 4},
{40, 1, 560, 16, 0, gScenarioBufferData10_INSERT_CHANNEL_3CH, 64, gScenarioKernelData10_INSERT_CHANNEL_3CH, 4},
{2, 2, 56, 16, 0, gScenarioBufferData11_INSERT_CHANNEL_3CH, 64, gScenarioKernelData11_INSERT_CHANNEL_3CH, 4},
{4, 2, 112, 16, 0, gScenarioBufferData12_INSERT_CHANNEL_3CH, 64, gScenarioKernelData12_INSERT_CHANNEL_3CH, 4},
{6, 2, 168, 16, 0, gScenarioBufferData13_INSERT_CHANNEL_3CH, 64, gScenarioKernelData13_INSERT_CHANNEL_3CH, 4},
{8, 2, 224, 16, 0, gScenarioBufferData14_INSERT_CHANNEL_3CH, 64, gScenarioKernelData14_INSERT_CHANNEL_3CH, 4},
{10, 2, 280, 16, 0, gScenarioBufferData15_INSERT_CHANNEL_3CH, 64, gScenarioKernelData15_INSERT_CHANNEL_3CH, 4},
{12, 2, 336, 16, 0, gScenarioBufferData16_INSERT_CHANNEL_3CH, 64, gScenarioKernelData16_INSERT_CHANNEL_3CH, 4},
{16, 2, 448, 16, 0, gScenarioBufferData17_INSERT_CHANNEL_3CH, 64, gScenarioKernelData17_INSERT_CHANNEL_3CH, 4},
{20, 2, 560, 16, 0, gScenarioBufferData18_INSERT_CHANNEL_3CH, 64, gScenarioKernelData18_INSERT_CHANNEL_3CH, 4},
{24, 2, 672, 16, 0, gScenarioBufferData19_INSERT_CHANNEL_3CH, 64, gScenarioKernelData19_INSERT_CHANNEL_3CH, 4},
{32, 2, 896, 16, 0, gScenarioBufferData20_INSERT_CHANNEL_3CH, 64, gScenarioKernelData20_INSERT_CHANNEL_3CH, 4},
{40, 2, 1120, 16, 0, gScenarioBufferData21_INSERT_CHANNEL_3CH, 64, gScenarioKernelData21_INSERT_CHANNEL_3CH, 4},
{2, 3, 84, 16, 0, gScenarioBufferData22_INSERT_CHANNEL_3CH, 64, gScenarioKernelData22_INSERT_CHANNEL_3CH, 4},
{4, 3, 168, 16, 0, gScenarioBufferData23_INSERT_CHANNEL_3CH, 64, gScenarioKernelData23_INSERT_CHANNEL_3CH, 4},
{6, 3, 252, 16, 0, gScenarioBufferData24_INSERT_CHANNEL_3CH, 64, gScenarioKernelData24_INSERT_CHANNEL_3CH, 4},
{8, 3, 336, 16, 0, gScenarioBufferData25_INSERT_CHANNEL_3CH, 64, gScenarioKernelData25_INSERT_CHANNEL_3CH, 4},
{10, 3, 420, 16, 0, gScenarioBufferData26_INSERT_CHANNEL_3CH, 64, gScenarioKernelData26_INSERT_CHANNEL_3CH, 4},
{12, 3, 504, 16, 0, gScenarioBufferData27_INSERT_CHANNEL_3CH, 64, gScenarioKernelData27_INSERT_CHANNEL_3CH, 4},
{16, 3, 672, 16, 0, gScenarioBufferData28_INSERT_CHANNEL_3CH, 64, gScenarioKernelData28_INSERT_CHANNEL_3CH, 4},
{20, 3, 840, 16, 0, gScenarioBufferData29_INSERT_CHANNEL_3CH, 64, gScenarioKernelData29_INSERT_CHANNEL_3CH, 4},
{24, 3, 1008, 16, 0, gScenarioBufferData30_INSERT_CHANNEL_3CH, 64, gScenarioKernelData30_INSERT_CHANNEL_3CH, 4},
{32, 3, 1344, 16, 0, gScenarioBufferData31_INSERT_CHANNEL_3CH, 64, gScenarioKernelData31_INSERT_CHANNEL_3CH, 4},
{40, 3, 1680, 16, 0, gScenarioBufferData32_INSERT_CHANNEL_3CH, 64, gScenarioKernelData32_INSERT_CHANNEL_3CH, 4},
{2, 4, 112, 16, 0, gScenarioBufferData33_INSERT_CHANNEL_3CH, 64, gScenarioKernelData33_INSERT_CHANNEL_3CH, 4},
{4, 4, 224, 16, 0, gScenarioBufferData34_INSERT_CHANNEL_3CH, 64, gScenarioKernelData34_INSERT_CHANNEL_3CH, 4},
{6, 4, 336, 16, 0, gScenarioBufferData35_INSERT_CHANNEL_3CH, 64, gScenarioKernelData35_INSERT_CHANNEL_3CH, 4},
{8, 4, 448, 16, 0, gScenarioBufferData36_INSERT_CHANNEL_3CH, 64, gScenarioKernelData36_INSERT_CHANNEL_3CH, 4},
{10, 4, 560, 16, 0, gScenarioBufferData37_INSERT_CHANNEL_3CH, 64, gScenarioKernelData37_INSERT_CHANNEL_3CH, 4},
{12, 4, 672, 16, 0, gScenarioBufferData38_INSERT_CHANNEL_3CH, 64, gScenarioKernelData38_INSERT_CHANNEL_3CH, 4},
{16, 4, 896, 16, 0, gScenarioBufferData39_INSERT_CHANNEL_3CH, 64, gScenarioKernelData39_INSERT_CHANNEL_3CH, 4},
{20, 4, 1120, 16, 0, gScenarioBufferData40_INSERT_CHANNEL_3CH, 64, gScenarioKernelData40_INSERT_CHANNEL_3CH, 4},
{24, 4, 1344, 16, 0, gScenarioBufferData41_INSERT_CHANNEL_3CH, 64, gScenarioKernelData41_INSERT_CHANNEL_3CH, 4},
{32, 4, 1792, 16, 0, gScenarioBufferData42_INSERT_CHANNEL_3CH, 64, gScenarioKernelData42_INSERT_CHANNEL_3CH, 4},
{40, 4, 2240, 16, 0, gScenarioBufferData43_INSERT_CHANNEL_3CH, 64, gScenarioKernelData43_INSERT_CHANNEL_3CH, 4},
{2, 5, 140, 16, 0, gScenarioBufferData44_INSERT_CHANNEL_3CH, 64, gScenarioKernelData44_INSERT_CHANNEL_3CH, 4},
{4, 5, 280, 16, 0, gScenarioBufferData45_INSERT_CHANNEL_3CH, 64, gScenarioKernelData45_INSERT_CHANNEL_3CH, 4},
{6, 5, 420, 16, 0, gScenarioBufferData46_INSERT_CHANNEL_3CH, 64, gScenarioKernelData46_INSERT_CHANNEL_3CH, 4},
{8, 5, 560, 16, 0, gScenarioBufferData47_INSERT_CHANNEL_3CH, 64, gScenarioKernelData47_INSERT_CHANNEL_3CH, 4},
{10, 5, 700, 16, 0, gScenarioBufferData48_INSERT_CHANNEL_3CH, 64, gScenarioKernelData48_INSERT_CHANNEL_3CH, 4},
{12, 5, 840, 16, 0, gScenarioBufferData49_INSERT_CHANNEL_3CH, 64, gScenarioKernelData49_INSERT_CHANNEL_3CH, 4},
{16, 5, 1120, 16, 0, gScenarioBufferData50_INSERT_CHANNEL_3CH, 64, gScenarioKernelData50_INSERT_CHANNEL_3CH, 4},
{20, 5, 1400, 16, 0, gScenarioBufferData51_INSERT_CHANNEL_3CH, 64, gScenarioKernelData51_INSERT_CHANNEL_3CH, 4},
{24, 5, 1680, 16, 0, gScenarioBufferData52_INSERT_CHANNEL_3CH, 64, gScenarioKernelData52_INSERT_CHANNEL_3CH, 4},
{32, 5, 2240, 16, 0, gScenarioBufferData53_INSERT_CHANNEL_3CH, 64, gScenarioKernelData53_INSERT_CHANNEL_3CH, 4},
{40, 5, 2800, 16, 0, gScenarioBufferData54_INSERT_CHANNEL_3CH, 64, gScenarioKernelData54_INSERT_CHANNEL_3CH, 4},
{2, 6, 168, 16, 0, gScenarioBufferData55_INSERT_CHANNEL_3CH, 64, gScenarioKernelData55_INSERT_CHANNEL_3CH, 4},
{4, 6, 336, 16, 0, gScenarioBufferData56_INSERT_CHANNEL_3CH, 64, gScenarioKernelData56_INSERT_CHANNEL_3CH, 4},
{6, 6, 504, 16, 0, gScenarioBufferData57_INSERT_CHANNEL_3CH, 64, gScenarioKernelData57_INSERT_CHANNEL_3CH, 4},
{8, 6, 672, 16, 0, gScenarioBufferData58_INSERT_CHANNEL_3CH, 64, gScenarioKernelData58_INSERT_CHANNEL_3CH, 4},
{10, 6, 840, 16, 0, gScenarioBufferData59_INSERT_CHANNEL_3CH, 64, gScenarioKernelData59_INSERT_CHANNEL_3CH, 4},
{12, 6, 1008, 16, 0, gScenarioBufferData60_INSERT_CHANNEL_3CH, 64, gScenarioKernelData60_INSERT_CHANNEL_3CH, 4},
{16, 6, 1344, 16, 0, gScenarioBufferData61_INSERT_CHANNEL_3CH, 64, gScenarioKernelData61_INSERT_CHANNEL_3CH, 4},
{20, 6, 1680, 16, 0, gScenarioBufferData62_INSERT_CHANNEL_3CH, 64, gScenarioKernelData62_INSERT_CHANNEL_3CH, 4},
{24, 6, 2016, 16, 0, gScenarioBufferData63_INSERT_CHANNEL_3CH, 64, gScenarioKernelData63_INSERT_CHANNEL_3CH, 4},
{32, 6, 2688, 16, 0, gScenarioBufferData64_INSERT_CHANNEL_3CH, 64, gScenarioKernelData64_INSERT_CHANNEL_3CH, 4},
{40, 6, 3360, 16, 0, gScenarioBufferData65_INSERT_CHANNEL_3CH, 64, gScenarioKernelData65_INSERT_CHANNEL_3CH, 4},
{2, 8, 224, 16, 0, gScenarioBufferData66_INSERT_CHANNEL_3CH, 64, gScenarioKernelData66_INSERT_CHANNEL_3CH, 4},
{4, 8, 448, 16, 0, gScenarioBufferData67_INSERT_CHANNEL_3CH, 64, gScenarioKernelData67_INSERT_CHANNEL_3CH, 4},
{6, 8, 672, 16, 0, gScenarioBufferData68_INSERT_CHANNEL_3CH, 64, gScenarioKernelData68_INSERT_CHANNEL_3CH, 4},
{8, 8, 896, 16, 0, gScenarioBufferData69_INSERT_CHANNEL_3CH, 64, gScenarioKernelData69_INSERT_CHANNEL_3CH, 4},
{10, 8, 1120, 16, 0, gScenarioBufferData70_INSERT_CHANNEL_3CH, 64, gScenarioKernelData70_INSERT_CHANNEL_3CH, 4},
{12, 8, 1344, 16, 0, gScenarioBufferData71_INSERT_CHANNEL_3CH, 64, gScenarioKernelData71_INSERT_CHANNEL_3CH, 4},
{16, 8, 1792, 16, 0, gScenarioBufferData72_INSERT_CHANNEL_3CH, 64, gScenarioKernelData72_INSERT_CHANNEL_3CH, 4},
{20, 8, 2240, 16, 0, gScenarioBufferData73_INSERT_CHANNEL_3CH, 64, gScenarioKernelData73_INSERT_CHANNEL_3CH, 4},
{24, 8, 2688, 16, 0, gScenarioBufferData74_INSERT_CHANNEL_3CH, 64, gScenarioKernelData74_INSERT_CHANNEL_3CH, 4},
{32, 8, 3584, 16, 0, gScenarioBufferData75_INSERT_CHANNEL_3CH, 64, gScenarioKernelData75_INSERT_CHANNEL_3CH, 4},
{2, 10, 280, 16, 0, gScenarioBufferData76_INSERT_CHANNEL_3CH, 64, gScenarioKernelData76_INSERT_CHANNEL_3CH, 4},
{4, 10, 560, 16, 0, gScenarioBufferData77_INSERT_CHANNEL_3CH, 64, gScenarioKernelData77_INSERT_CHANNEL_3CH, 4},
{6, 10, 840, 16, 0, gScenarioBufferData78_INSERT_CHANNEL_3CH, 64, gScenarioKernelData78_INSERT_CHANNEL_3CH, 4},
{8, 10, 1120, 16, 0, gScenarioBufferData79_INSERT_CHANNEL_3CH, 64, gScenarioKernelData79_INSERT_CHANNEL_3CH, 4},
{10, 10, 1400, 16, 0, gScenarioBufferData80_INSERT_CHANNEL_3CH, 64, gScenarioKernelData80_INSERT_CHANNEL_3CH, 4},
{12, 10, 1680, 16, 0, gScenarioBufferData81_INSERT_CHANNEL_3CH, 64, gScenarioKernelData81_INSERT_CHANNEL_3CH, 4},
{16, 10, 2240, 16, 0, gScenarioBufferData82_INSERT_CHANNEL_3CH, 64, gScenarioKernelData82_INSERT_CHANNEL_3CH, 4},
{20, 10, 2800, 16, 0, gScenarioBufferData83_INSERT_CHANNEL_3CH, 64, gScenarioKernelData83_INSERT_CHANNEL_3CH, 4},
{24, 10, 3360, 16, 0, gScenarioBufferData84_INSERT_CHANNEL_3CH, 64, gScenarioKernelData84_INSERT_CHANNEL_3CH, 4},
{2, 12, 336, 16, 0, gScenarioBufferData85_INSERT_CHANNEL_3CH, 64, gScenarioKernelData85_INSERT_CHANNEL_3CH, 4},
{4, 12, 672, 16, 0, gScenarioBufferData86_INSERT_CHANNEL_3CH, 64, gScenarioKernelData86_INSERT_CHANNEL_3CH, 4},
{6, 12, 1008, 16, 0, gScenarioBufferData87_INSERT_CHANNEL_3CH, 64, gScenarioKernelData87_INSERT_CHANNEL_3CH, 4},
{8, 12, 1344, 16, 0, gScenarioBufferData88_INSERT_CHANNEL_3CH, 64, gScenarioKernelData88_INSERT_CHANNEL_3CH, 4},
{10, 12, 1680, 16, 0, gScenarioBufferData89_INSERT_CHANNEL_3CH, 64, gScenarioKernelData89_INSERT_CHANNEL_3CH, 4},
{12, 12, 2016, 16, 0, gScenarioBufferData90_INSERT_CHANNEL_3CH, 64, gScenarioKernelData90_INSERT_CHANNEL_3CH, 4},
{16, 12, 2688, 16, 0, gScenarioBufferData91_INSERT_CHANNEL_3CH, 64, gScenarioKernelData91_INSERT_CHANNEL_3CH, 4},
{20, 12, 3360, 16, 0, gScenarioBufferData92_INSERT_CHANNEL_3CH, 64, gScenarioKernelData92_INSERT_CHANNEL_3CH, 4},
{2, 14, 392, 16, 0, gScenarioBufferData93_INSERT_CHANNEL_3CH, 64, gScenarioKernelData93_INSERT_CHANNEL_3CH, 4},
{4, 14, 784, 16, 0, gScenarioBufferData94_INSERT_CHANNEL_3CH, 64, gScenarioKernelData94_INSERT_CHANNEL_3CH, 4},
{6, 14, 1176, 16, 0, gScenarioBufferData95_INSERT_CHANNEL_3CH, 64, gScenarioKernelData95_INSERT_CHANNEL_3CH, 4},
{8, 14, 1568, 16, 0, gScenarioBufferData96_INSERT_CHANNEL_3CH, 64, gScenarioKernelData96_INSERT_CHANNEL_3CH, 4},
{10, 14, 1960, 16, 0, gScenarioBufferData97_INSERT_CHANNEL_3CH, 64, gScenarioKernelData97_INSERT_CHANNEL_3CH, 4},
{12, 14, 2352, 16, 0, gScenarioBufferData98_INSERT_CHANNEL_3CH, 64, gScenarioKernelData98_INSERT_CHANNEL_3CH, 4},
{16, 14, 3136, 16, 0, gScenarioBufferData99_INSERT_CHANNEL_3CH, 64, gScenarioKernelData99_INSERT_CHANNEL_3CH, 4},
{2, 16, 448, 16, 0, gScenarioBufferData100_INSERT_CHANNEL_3CH, 64, gScenarioKernelData100_INSERT_CHANNEL_3CH, 4},
{4, 16, 896, 16, 0, gScenarioBufferData101_INSERT_CHANNEL_3CH, 64, gScenarioKernelData101_INSERT_CHANNEL_3CH, 4},
{6, 16, 1344, 16, 0, gScenarioBufferData102_INSERT_CHANNEL_3CH, 64, gScenarioKernelData102_INSERT_CHANNEL_3CH, 4},
{8, 16, 1792, 16, 0, gScenarioBufferData103_INSERT_CHANNEL_3CH, 64, gScenarioKernelData103_INSERT_CHANNEL_3CH, 4},
{10, 16, 2240, 16, 0, gScenarioBufferData104_INSERT_CHANNEL_3CH, 64, gScenarioKernelData104_INSERT_CHANNEL_3CH, 4},
{12, 16, 2688, 16, 0, gScenarioBufferData105_INSERT_CHANNEL_3CH, 64, gScenarioKernelData105_INSERT_CHANNEL_3CH, 4},
{16, 16, 3584, 16, 0, gScenarioBufferData106_INSERT_CHANNEL_3CH, 64, gScenarioKernelData106_INSERT_CHANNEL_3CH, 4},
{2, 18, 504, 16, 0, gScenarioBufferData107_INSERT_CHANNEL_3CH, 64, gScenarioKernelData107_INSERT_CHANNEL_3CH, 4},
{4, 18, 1008, 16, 0, gScenarioBufferData108_INSERT_CHANNEL_3CH, 64, gScenarioKernelData108_INSERT_CHANNEL_3CH, 4},
{6, 18, 1512, 16, 0, gScenarioBufferData109_INSERT_CHANNEL_3CH, 64, gScenarioKernelData109_INSERT_CHANNEL_3CH, 4},
{8, 18, 2016, 16, 0, gScenarioBufferData110_INSERT_CHANNEL_3CH, 64, gScenarioKernelData110_INSERT_CHANNEL_3CH, 4},
{10, 18, 2520, 16, 0, gScenarioBufferData111_INSERT_CHANNEL_3CH, 64, gScenarioKernelData111_INSERT_CHANNEL_3CH, 4},
{12, 18, 3024, 16, 0, gScenarioBufferData112_INSERT_CHANNEL_3CH, 64, gScenarioKernelData112_INSERT_CHANNEL_3CH, 4},
{2, 20, 560, 16, 0, gScenarioBufferData113_INSERT_CHANNEL_3CH, 64, gScenarioKernelData113_INSERT_CHANNEL_3CH, 4},
{4, 20, 1120, 16, 0, gScenarioBufferData114_INSERT_CHANNEL_3CH, 64, gScenarioKernelData114_INSERT_CHANNEL_3CH, 4},
{6, 20, 1680, 16, 0, gScenarioBufferData115_INSERT_CHANNEL_3CH, 64, gScenarioKernelData115_INSERT_CHANNEL_3CH, 4},
{8, 20, 2240, 16, 0, gScenarioBufferData116_INSERT_CHANNEL_3CH, 64, gScenarioKernelData116_INSERT_CHANNEL_3CH, 4},
{10, 20, 2800, 16, 0, gScenarioBufferData117_INSERT_CHANNEL_3CH, 64, gScenarioKernelData117_INSERT_CHANNEL_3CH, 4},
{12, 20, 3360, 16, 0, gScenarioBufferData118_INSERT_CHANNEL_3CH, 64, gScenarioKernelData118_INSERT_CHANNEL_3CH, 4},
{2, 22, 616, 16, 0, gScenarioBufferData119_INSERT_CHANNEL_3CH, 64, gScenarioKernelData119_INSERT_CHANNEL_3CH, 4},
{4, 22, 1232, 16, 0, gScenarioBufferData120_INSERT_CHANNEL_3CH, 64, gScenarioKernelData120_INSERT_CHANNEL_3CH, 4},
{6, 22, 1848, 16, 0, gScenarioBufferData121_INSERT_CHANNEL_3CH, 64, gScenarioKernelData121_INSERT_CHANNEL_3CH, 4},
{8, 22, 2464, 16, 0, gScenarioBufferData122_INSERT_CHANNEL_3CH, 64, gScenarioKernelData122_INSERT_CHANNEL_3CH, 4},
{10, 22, 3080, 16, 0, gScenarioBufferData123_INSERT_CHANNEL_3CH, 64, gScenarioKernelData123_INSERT_CHANNEL_3CH, 4},
{12, 22, 3696, 16, 0, gScenarioBufferData124_INSERT_CHANNEL_3CH, 64, gScenarioKernelData124_INSERT_CHANNEL_3CH, 4},
{2, 24, 672, 16, 0, gScenarioBufferData125_INSERT_CHANNEL_3CH, 64, gScenarioKernelData125_INSERT_CHANNEL_3CH, 4},
{4, 24, 1344, 16, 0, gScenarioBufferData126_INSERT_CHANNEL_3CH, 64, gScenarioKernelData126_INSERT_CHANNEL_3CH, 4},
{6, 24, 2016, 16, 0, gScenarioBufferData127_INSERT_CHANNEL_3CH, 64, gScenarioKernelData127_INSERT_CHANNEL_3CH, 4},
{8, 24, 2688, 16, 0, gScenarioBufferData128_INSERT_CHANNEL_3CH, 64, gScenarioKernelData128_INSERT_CHANNEL_3CH, 4},
{10, 24, 3360, 16, 0, gScenarioBufferData129_INSERT_CHANNEL_3CH, 64, gScenarioKernelData129_INSERT_CHANNEL_3CH, 4},
{2, 26, 728, 16, 0, gScenarioBufferData130_INSERT_CHANNEL_3CH, 64, gScenarioKernelData130_INSERT_CHANNEL_3CH, 4},
{4, 26, 1456, 16, 0, gScenarioBufferData131_INSERT_CHANNEL_3CH, 64, gScenarioKernelData131_INSERT_CHANNEL_3CH, 4},
{6, 26, 2184, 16, 0, gScenarioBufferData132_INSERT_CHANNEL_3CH, 64, gScenarioKernelData132_INSERT_CHANNEL_3CH, 4},
{8, 26, 2912, 16, 0, gScenarioBufferData133_INSERT_CHANNEL_3CH, 64, gScenarioKernelData133_INSERT_CHANNEL_3CH, 4},
{10, 26, 3640, 16, 0, gScenarioBufferData134_INSERT_CHANNEL_3CH, 64, gScenarioKernelData134_INSERT_CHANNEL_3CH, 4},
{2, 28, 784, 16, 0, gScenarioBufferData135_INSERT_CHANNEL_3CH, 64, gScenarioKernelData135_INSERT_CHANNEL_3CH, 4},
{4, 28, 1568, 16, 0, gScenarioBufferData136_INSERT_CHANNEL_3CH, 64, gScenarioKernelData136_INSERT_CHANNEL_3CH, 4},
{6, 28, 2352, 16, 0, gScenarioBufferData137_INSERT_CHANNEL_3CH, 64, gScenarioKernelData137_INSERT_CHANNEL_3CH, 4},
{8, 28, 3136, 16, 0, gScenarioBufferData138_INSERT_CHANNEL_3CH, 64, gScenarioKernelData138_INSERT_CHANNEL_3CH, 4},
{2, 30, 840, 16, 0, gScenarioBufferData139_INSERT_CHANNEL_3CH, 64, gScenarioKernelData139_INSERT_CHANNEL_3CH, 4},
{4, 30, 1680, 16, 0, gScenarioBufferData140_INSERT_CHANNEL_3CH, 64, gScenarioKernelData140_INSERT_CHANNEL_3CH, 4},
{6, 30, 2520, 16, 0, gScenarioBufferData141_INSERT_CHANNEL_3CH, 64, gScenarioKernelData141_INSERT_CHANNEL_3CH, 4},
{8, 30, 3360, 16, 0, gScenarioBufferData142_INSERT_CHANNEL_3CH, 64, gScenarioKernelData142_INSERT_CHANNEL_3CH, 4},
{2, 32, 896, 16, 0, gScenarioBufferData143_INSERT_CHANNEL_3CH, 64, gScenarioKernelData143_INSERT_CHANNEL_3CH, 4},
{4, 32, 1792, 16, 0, gScenarioBufferData144_INSERT_CHANNEL_3CH, 64, gScenarioKernelData144_INSERT_CHANNEL_3CH, 4},
{6, 32, 2688, 16, 0, gScenarioBufferData145_INSERT_CHANNEL_3CH, 64, gScenarioKernelData145_INSERT_CHANNEL_3CH, 4},
{8, 32, 3584, 16, 0, gScenarioBufferData146_INSERT_CHANNEL_3CH, 64, gScenarioKernelData146_INSERT_CHANNEL_3CH, 4}
};

static acf_scenario_list gScenarioList_INSERT_CHANNEL_3CH = {
147, //number of scenarios
gScenarioArray_INSERT_CHANNEL_3CH};
//**************************************************************

class INSERT_CHANNEL_3CH : public ACF_Process_APU
{

public:
   INSERT_CHANNEL_3CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INSERT_CHANNEL_3CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INSERT_CHANNEL_3CH",
                                        INSERT_CHANNEL_3CH_LOAD_SEGMENTS,
                                        INSERT_CHANNEL_3CH_LOAD_PMEM, INSERT_CHANNEL_3CH_LOAD_PMEM_SIZE,
                                        INSERT_CHANNEL_3CH_LOAD_DMEM, INSERT_CHANNEL_3CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INSERT_CHANNEL_3CH_APEX_LOG_BUFFER, INSERT_CHANNEL_3CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INSERT_CHANNEL_3CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INSERT_CHANNEL_3CH

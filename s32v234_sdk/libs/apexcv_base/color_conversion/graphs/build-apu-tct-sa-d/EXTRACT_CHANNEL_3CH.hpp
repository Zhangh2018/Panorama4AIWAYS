#ifndef _ACF_PROCESS_APU_EXTRACT_CHANNEL_3CH
#define _ACF_PROCESS_APU_EXTRACT_CHANNEL_3CH

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <EXTRACT_CHANNEL_3CH_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_EXTRACT_CHANNEL_3CH[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_EXTRACT_CHANNEL_3CH[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_EXTRACT_CHANNEL_3CH[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_EXTRACT_CHANNEL_3CH[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_EXTRACT_CHANNEL_3CH[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_EXTRACT_CHANNEL_3CH[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_EXTRACT_CHANNEL_3CH[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_EXTRACT_CHANNEL_3CH[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_EXTRACT_CHANNEL_3CH[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_EXTRACT_CHANNEL_3CH[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_EXTRACT_CHANNEL_3CH[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_EXTRACT_CHANNEL_3CH[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_EXTRACT_CHANNEL_3CH[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_EXTRACT_CHANNEL_3CH[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_EXTRACT_CHANNEL_3CH[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_EXTRACT_CHANNEL_3CH[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_EXTRACT_CHANNEL_3CH[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_EXTRACT_CHANNEL_3CH[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_EXTRACT_CHANNEL_3CH[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_EXTRACT_CHANNEL_3CH[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_EXTRACT_CHANNEL_3CH[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_EXTRACT_CHANNEL_3CH[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_EXTRACT_CHANNEL_3CH[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_EXTRACT_CHANNEL_3CH[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_EXTRACT_CHANNEL_3CH[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_EXTRACT_CHANNEL_3CH[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_EXTRACT_CHANNEL_3CH[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_EXTRACT_CHANNEL_3CH[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_EXTRACT_CHANNEL_3CH[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_EXTRACT_CHANNEL_3CH[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_EXTRACT_CHANNEL_3CH[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_EXTRACT_CHANNEL_3CH[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_EXTRACT_CHANNEL_3CH[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_EXTRACT_CHANNEL_3CH[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_EXTRACT_CHANNEL_3CH[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_EXTRACT_CHANNEL_3CH[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_EXTRACT_CHANNEL_3CH[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_EXTRACT_CHANNEL_3CH[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_EXTRACT_CHANNEL_3CH[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_EXTRACT_CHANNEL_3CH[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_EXTRACT_CHANNEL_3CH[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_EXTRACT_CHANNEL_3CH[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_EXTRACT_CHANNEL_3CH[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_EXTRACT_CHANNEL_3CH[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_EXTRACT_CHANNEL_3CH[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_EXTRACT_CHANNEL_3CH[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_EXTRACT_CHANNEL_3CH[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_EXTRACT_CHANNEL_3CH[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_EXTRACT_CHANNEL_3CH[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_EXTRACT_CHANNEL_3CH[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_EXTRACT_CHANNEL_3CH[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_EXTRACT_CHANNEL_3CH[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_EXTRACT_CHANNEL_3CH[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_EXTRACT_CHANNEL_3CH[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_EXTRACT_CHANNEL_3CH[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_EXTRACT_CHANNEL_3CH[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_EXTRACT_CHANNEL_3CH[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_EXTRACT_CHANNEL_3CH[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_EXTRACT_CHANNEL_3CH[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_EXTRACT_CHANNEL_3CH[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_EXTRACT_CHANNEL_3CH[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_EXTRACT_CHANNEL_3CH[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_EXTRACT_CHANNEL_3CH[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_EXTRACT_CHANNEL_3CH[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_EXTRACT_CHANNEL_3CH[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_EXTRACT_CHANNEL_3CH[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_EXTRACT_CHANNEL_3CH[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_EXTRACT_CHANNEL_3CH[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_EXTRACT_CHANNEL_3CH[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_EXTRACT_CHANNEL_3CH[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_EXTRACT_CHANNEL_3CH[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_EXTRACT_CHANNEL_3CH[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_EXTRACT_CHANNEL_3CH[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_EXTRACT_CHANNEL_3CH[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_EXTRACT_CHANNEL_3CH[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_EXTRACT_CHANNEL_3CH[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_EXTRACT_CHANNEL_3CH[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_EXTRACT_CHANNEL_3CH[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_EXTRACT_CHANNEL_3CH[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_EXTRACT_CHANNEL_3CH[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_EXTRACT_CHANNEL_3CH[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_EXTRACT_CHANNEL_3CH[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_EXTRACT_CHANNEL_3CH[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_EXTRACT_CHANNEL_3CH[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_EXTRACT_CHANNEL_3CH[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_EXTRACT_CHANNEL_3CH[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_EXTRACT_CHANNEL_3CH[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_EXTRACT_CHANNEL_3CH[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_EXTRACT_CHANNEL_3CH[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_EXTRACT_CHANNEL_3CH[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_EXTRACT_CHANNEL_3CH[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_EXTRACT_CHANNEL_3CH[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_EXTRACT_CHANNEL_3CH[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_EXTRACT_CHANNEL_3CH[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_EXTRACT_CHANNEL_3CH[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_EXTRACT_CHANNEL_3CH[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_EXTRACT_CHANNEL_3CH[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_EXTRACT_CHANNEL_3CH[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_EXTRACT_CHANNEL_3CH[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_EXTRACT_CHANNEL_3CH[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_EXTRACT_CHANNEL_3CH[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_EXTRACT_CHANNEL_3CH[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_EXTRACT_CHANNEL_3CH[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_EXTRACT_CHANNEL_3CH[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_EXTRACT_CHANNEL_3CH[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_EXTRACT_CHANNEL_3CH[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_EXTRACT_CHANNEL_3CH[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_EXTRACT_CHANNEL_3CH[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_EXTRACT_CHANNEL_3CH[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_EXTRACT_CHANNEL_3CH[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_EXTRACT_CHANNEL_3CH[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_EXTRACT_CHANNEL_3CH[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_EXTRACT_CHANNEL_3CH[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_EXTRACT_CHANNEL_3CH[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_EXTRACT_CHANNEL_3CH[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_EXTRACT_CHANNEL_3CH[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_EXTRACT_CHANNEL_3CH[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_EXTRACT_CHANNEL_3CH[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_EXTRACT_CHANNEL_3CH[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_EXTRACT_CHANNEL_3CH[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_EXTRACT_CHANNEL_3CH[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_EXTRACT_CHANNEL_3CH[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_EXTRACT_CHANNEL_3CH[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_EXTRACT_CHANNEL_3CH[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_EXTRACT_CHANNEL_3CH[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_EXTRACT_CHANNEL_3CH[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_EXTRACT_CHANNEL_3CH[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_EXTRACT_CHANNEL_3CH[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_EXTRACT_CHANNEL_3CH[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_EXTRACT_CHANNEL_3CH[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_EXTRACT_CHANNEL_3CH[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_EXTRACT_CHANNEL_3CH[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_EXTRACT_CHANNEL_3CH[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_EXTRACT_CHANNEL_3CH[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_EXTRACT_CHANNEL_3CH[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_EXTRACT_CHANNEL_3CH[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_EXTRACT_CHANNEL_3CH[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_EXTRACT_CHANNEL_3CH[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_EXTRACT_CHANNEL_3CH[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_EXTRACT_CHANNEL_3CH[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_EXTRACT_CHANNEL_3CH[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_EXTRACT_CHANNEL_3CH[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_EXTRACT_CHANNEL_3CH[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_EXTRACT_CHANNEL_3CH[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_EXTRACT_CHANNEL_3CH[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_EXTRACT_CHANNEL_3CH[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_EXTRACT_CHANNEL_3CH[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_EXTRACT_CHANNEL_3CH[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_EXTRACT_CHANNEL_3CH[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_EXTRACT_CHANNEL_3CH[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_EXTRACT_CHANNEL_3CH[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_EXTRACT_CHANNEL_3CH[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_EXTRACT_CHANNEL_3CH[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_EXTRACT_CHANNEL_3CH[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_EXTRACT_CHANNEL_3CH[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_EXTRACT_CHANNEL_3CH[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_EXTRACT_CHANNEL_3CH[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_EXTRACT_CHANNEL_3CH[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_EXTRACT_CHANNEL_3CH[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_EXTRACT_CHANNEL_3CH[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_EXTRACT_CHANNEL_3CH[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_EXTRACT_CHANNEL_3CH[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_EXTRACT_CHANNEL_3CH[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_EXTRACT_CHANNEL_3CH[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_EXTRACT_CHANNEL_3CH[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_EXTRACT_CHANNEL_3CH[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_EXTRACT_CHANNEL_3CH[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_EXTRACT_CHANNEL_3CH[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_EXTRACT_CHANNEL_3CH[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_EXTRACT_CHANNEL_3CH[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_EXTRACT_CHANNEL_3CH[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_EXTRACT_CHANNEL_3CH[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_EXTRACT_CHANNEL_3CH[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_EXTRACT_CHANNEL_3CH[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_EXTRACT_CHANNEL_3CH[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_EXTRACT_CHANNEL_3CH[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_EXTRACT_CHANNEL_3CH[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_EXTRACT_CHANNEL_3CH[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_EXTRACT_CHANNEL_3CH[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_EXTRACT_CHANNEL_3CH[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_EXTRACT_CHANNEL_3CH[] = {{0, 0}};

static acf_scenario gScenarioArray_EXTRACT_CHANNEL_3CH[] = {
{2, 1, 16, 16, 0, gScenarioBufferData0_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData0_EXTRACT_CHANNEL_3CH, 4},
{4, 1, 32, 16, 0, gScenarioBufferData1_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData1_EXTRACT_CHANNEL_3CH, 4},
{6, 1, 48, 16, 0, gScenarioBufferData2_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData2_EXTRACT_CHANNEL_3CH, 4},
{8, 1, 64, 16, 0, gScenarioBufferData3_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData3_EXTRACT_CHANNEL_3CH, 4},
{10, 1, 80, 16, 0, gScenarioBufferData4_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData4_EXTRACT_CHANNEL_3CH, 4},
{12, 1, 96, 16, 0, gScenarioBufferData5_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData5_EXTRACT_CHANNEL_3CH, 4},
{16, 1, 128, 16, 0, gScenarioBufferData6_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData6_EXTRACT_CHANNEL_3CH, 4},
{20, 1, 160, 16, 0, gScenarioBufferData7_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData7_EXTRACT_CHANNEL_3CH, 4},
{24, 1, 192, 16, 0, gScenarioBufferData8_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData8_EXTRACT_CHANNEL_3CH, 4},
{32, 1, 256, 16, 0, gScenarioBufferData9_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData9_EXTRACT_CHANNEL_3CH, 4},
{40, 1, 320, 16, 0, gScenarioBufferData10_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData10_EXTRACT_CHANNEL_3CH, 4},
{2, 2, 32, 16, 0, gScenarioBufferData11_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData11_EXTRACT_CHANNEL_3CH, 4},
{4, 2, 64, 16, 0, gScenarioBufferData12_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData12_EXTRACT_CHANNEL_3CH, 4},
{6, 2, 96, 16, 0, gScenarioBufferData13_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData13_EXTRACT_CHANNEL_3CH, 4},
{8, 2, 128, 16, 0, gScenarioBufferData14_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData14_EXTRACT_CHANNEL_3CH, 4},
{10, 2, 160, 16, 0, gScenarioBufferData15_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData15_EXTRACT_CHANNEL_3CH, 4},
{12, 2, 192, 16, 0, gScenarioBufferData16_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData16_EXTRACT_CHANNEL_3CH, 4},
{16, 2, 256, 16, 0, gScenarioBufferData17_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData17_EXTRACT_CHANNEL_3CH, 4},
{20, 2, 320, 16, 0, gScenarioBufferData18_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData18_EXTRACT_CHANNEL_3CH, 4},
{24, 2, 384, 16, 0, gScenarioBufferData19_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData19_EXTRACT_CHANNEL_3CH, 4},
{32, 2, 512, 16, 0, gScenarioBufferData20_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData20_EXTRACT_CHANNEL_3CH, 4},
{40, 2, 640, 16, 0, gScenarioBufferData21_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData21_EXTRACT_CHANNEL_3CH, 4},
{2, 3, 48, 16, 0, gScenarioBufferData22_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData22_EXTRACT_CHANNEL_3CH, 4},
{4, 3, 96, 16, 0, gScenarioBufferData23_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData23_EXTRACT_CHANNEL_3CH, 4},
{6, 3, 144, 16, 0, gScenarioBufferData24_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData24_EXTRACT_CHANNEL_3CH, 4},
{8, 3, 192, 16, 0, gScenarioBufferData25_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData25_EXTRACT_CHANNEL_3CH, 4},
{10, 3, 240, 16, 0, gScenarioBufferData26_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData26_EXTRACT_CHANNEL_3CH, 4},
{12, 3, 288, 16, 0, gScenarioBufferData27_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData27_EXTRACT_CHANNEL_3CH, 4},
{16, 3, 384, 16, 0, gScenarioBufferData28_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData28_EXTRACT_CHANNEL_3CH, 4},
{20, 3, 480, 16, 0, gScenarioBufferData29_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData29_EXTRACT_CHANNEL_3CH, 4},
{24, 3, 576, 16, 0, gScenarioBufferData30_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData30_EXTRACT_CHANNEL_3CH, 4},
{32, 3, 768, 16, 0, gScenarioBufferData31_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData31_EXTRACT_CHANNEL_3CH, 4},
{40, 3, 960, 16, 0, gScenarioBufferData32_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData32_EXTRACT_CHANNEL_3CH, 4},
{2, 4, 64, 16, 0, gScenarioBufferData33_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData33_EXTRACT_CHANNEL_3CH, 4},
{4, 4, 128, 16, 0, gScenarioBufferData34_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData34_EXTRACT_CHANNEL_3CH, 4},
{6, 4, 192, 16, 0, gScenarioBufferData35_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData35_EXTRACT_CHANNEL_3CH, 4},
{8, 4, 256, 16, 0, gScenarioBufferData36_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData36_EXTRACT_CHANNEL_3CH, 4},
{10, 4, 320, 16, 0, gScenarioBufferData37_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData37_EXTRACT_CHANNEL_3CH, 4},
{12, 4, 384, 16, 0, gScenarioBufferData38_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData38_EXTRACT_CHANNEL_3CH, 4},
{16, 4, 512, 16, 0, gScenarioBufferData39_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData39_EXTRACT_CHANNEL_3CH, 4},
{20, 4, 640, 16, 0, gScenarioBufferData40_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData40_EXTRACT_CHANNEL_3CH, 4},
{24, 4, 768, 16, 0, gScenarioBufferData41_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData41_EXTRACT_CHANNEL_3CH, 4},
{32, 4, 1024, 16, 0, gScenarioBufferData42_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData42_EXTRACT_CHANNEL_3CH, 4},
{40, 4, 1280, 16, 0, gScenarioBufferData43_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData43_EXTRACT_CHANNEL_3CH, 4},
{2, 5, 80, 16, 0, gScenarioBufferData44_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData44_EXTRACT_CHANNEL_3CH, 4},
{4, 5, 160, 16, 0, gScenarioBufferData45_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData45_EXTRACT_CHANNEL_3CH, 4},
{6, 5, 240, 16, 0, gScenarioBufferData46_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData46_EXTRACT_CHANNEL_3CH, 4},
{8, 5, 320, 16, 0, gScenarioBufferData47_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData47_EXTRACT_CHANNEL_3CH, 4},
{10, 5, 400, 16, 0, gScenarioBufferData48_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData48_EXTRACT_CHANNEL_3CH, 4},
{12, 5, 480, 16, 0, gScenarioBufferData49_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData49_EXTRACT_CHANNEL_3CH, 4},
{16, 5, 640, 16, 0, gScenarioBufferData50_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData50_EXTRACT_CHANNEL_3CH, 4},
{20, 5, 800, 16, 0, gScenarioBufferData51_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData51_EXTRACT_CHANNEL_3CH, 4},
{24, 5, 960, 16, 0, gScenarioBufferData52_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData52_EXTRACT_CHANNEL_3CH, 4},
{32, 5, 1280, 16, 0, gScenarioBufferData53_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData53_EXTRACT_CHANNEL_3CH, 4},
{40, 5, 1600, 16, 0, gScenarioBufferData54_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData54_EXTRACT_CHANNEL_3CH, 4},
{2, 6, 96, 16, 0, gScenarioBufferData55_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData55_EXTRACT_CHANNEL_3CH, 4},
{4, 6, 192, 16, 0, gScenarioBufferData56_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData56_EXTRACT_CHANNEL_3CH, 4},
{6, 6, 288, 16, 0, gScenarioBufferData57_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData57_EXTRACT_CHANNEL_3CH, 4},
{8, 6, 384, 16, 0, gScenarioBufferData58_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData58_EXTRACT_CHANNEL_3CH, 4},
{10, 6, 480, 16, 0, gScenarioBufferData59_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData59_EXTRACT_CHANNEL_3CH, 4},
{12, 6, 576, 16, 0, gScenarioBufferData60_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData60_EXTRACT_CHANNEL_3CH, 4},
{16, 6, 768, 16, 0, gScenarioBufferData61_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData61_EXTRACT_CHANNEL_3CH, 4},
{20, 6, 960, 16, 0, gScenarioBufferData62_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData62_EXTRACT_CHANNEL_3CH, 4},
{24, 6, 1152, 16, 0, gScenarioBufferData63_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData63_EXTRACT_CHANNEL_3CH, 4},
{32, 6, 1536, 16, 0, gScenarioBufferData64_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData64_EXTRACT_CHANNEL_3CH, 4},
{40, 6, 1920, 16, 0, gScenarioBufferData65_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData65_EXTRACT_CHANNEL_3CH, 4},
{2, 8, 128, 16, 0, gScenarioBufferData66_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData66_EXTRACT_CHANNEL_3CH, 4},
{4, 8, 256, 16, 0, gScenarioBufferData67_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData67_EXTRACT_CHANNEL_3CH, 4},
{6, 8, 384, 16, 0, gScenarioBufferData68_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData68_EXTRACT_CHANNEL_3CH, 4},
{8, 8, 512, 16, 0, gScenarioBufferData69_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData69_EXTRACT_CHANNEL_3CH, 4},
{10, 8, 640, 16, 0, gScenarioBufferData70_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData70_EXTRACT_CHANNEL_3CH, 4},
{12, 8, 768, 16, 0, gScenarioBufferData71_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData71_EXTRACT_CHANNEL_3CH, 4},
{16, 8, 1024, 16, 0, gScenarioBufferData72_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData72_EXTRACT_CHANNEL_3CH, 4},
{20, 8, 1280, 16, 0, gScenarioBufferData73_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData73_EXTRACT_CHANNEL_3CH, 4},
{24, 8, 1536, 16, 0, gScenarioBufferData74_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData74_EXTRACT_CHANNEL_3CH, 4},
{32, 8, 2048, 16, 0, gScenarioBufferData75_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData75_EXTRACT_CHANNEL_3CH, 4},
{40, 8, 2560, 16, 0, gScenarioBufferData76_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData76_EXTRACT_CHANNEL_3CH, 4},
{2, 10, 160, 16, 0, gScenarioBufferData77_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData77_EXTRACT_CHANNEL_3CH, 4},
{4, 10, 320, 16, 0, gScenarioBufferData78_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData78_EXTRACT_CHANNEL_3CH, 4},
{6, 10, 480, 16, 0, gScenarioBufferData79_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData79_EXTRACT_CHANNEL_3CH, 4},
{8, 10, 640, 16, 0, gScenarioBufferData80_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData80_EXTRACT_CHANNEL_3CH, 4},
{10, 10, 800, 16, 0, gScenarioBufferData81_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData81_EXTRACT_CHANNEL_3CH, 4},
{12, 10, 960, 16, 0, gScenarioBufferData82_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData82_EXTRACT_CHANNEL_3CH, 4},
{16, 10, 1280, 16, 0, gScenarioBufferData83_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData83_EXTRACT_CHANNEL_3CH, 4},
{20, 10, 1600, 16, 0, gScenarioBufferData84_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData84_EXTRACT_CHANNEL_3CH, 4},
{24, 10, 1920, 16, 0, gScenarioBufferData85_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData85_EXTRACT_CHANNEL_3CH, 4},
{32, 10, 2560, 16, 0, gScenarioBufferData86_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData86_EXTRACT_CHANNEL_3CH, 4},
{40, 10, 3200, 16, 0, gScenarioBufferData87_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData87_EXTRACT_CHANNEL_3CH, 4},
{2, 12, 192, 16, 0, gScenarioBufferData88_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData88_EXTRACT_CHANNEL_3CH, 4},
{4, 12, 384, 16, 0, gScenarioBufferData89_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData89_EXTRACT_CHANNEL_3CH, 4},
{6, 12, 576, 16, 0, gScenarioBufferData90_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData90_EXTRACT_CHANNEL_3CH, 4},
{8, 12, 768, 16, 0, gScenarioBufferData91_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData91_EXTRACT_CHANNEL_3CH, 4},
{10, 12, 960, 16, 0, gScenarioBufferData92_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData92_EXTRACT_CHANNEL_3CH, 4},
{12, 12, 1152, 16, 0, gScenarioBufferData93_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData93_EXTRACT_CHANNEL_3CH, 4},
{16, 12, 1536, 16, 0, gScenarioBufferData94_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData94_EXTRACT_CHANNEL_3CH, 4},
{20, 12, 1920, 16, 0, gScenarioBufferData95_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData95_EXTRACT_CHANNEL_3CH, 4},
{24, 12, 2304, 16, 0, gScenarioBufferData96_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData96_EXTRACT_CHANNEL_3CH, 4},
{32, 12, 3072, 16, 0, gScenarioBufferData97_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData97_EXTRACT_CHANNEL_3CH, 4},
{40, 12, 3840, 16, 0, gScenarioBufferData98_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData98_EXTRACT_CHANNEL_3CH, 4},
{2, 14, 224, 16, 0, gScenarioBufferData99_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData99_EXTRACT_CHANNEL_3CH, 4},
{4, 14, 448, 16, 0, gScenarioBufferData100_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData100_EXTRACT_CHANNEL_3CH, 4},
{6, 14, 672, 16, 0, gScenarioBufferData101_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData101_EXTRACT_CHANNEL_3CH, 4},
{8, 14, 896, 16, 0, gScenarioBufferData102_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData102_EXTRACT_CHANNEL_3CH, 4},
{10, 14, 1120, 16, 0, gScenarioBufferData103_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData103_EXTRACT_CHANNEL_3CH, 4},
{12, 14, 1344, 16, 0, gScenarioBufferData104_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData104_EXTRACT_CHANNEL_3CH, 4},
{16, 14, 1792, 16, 0, gScenarioBufferData105_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData105_EXTRACT_CHANNEL_3CH, 4},
{20, 14, 2240, 16, 0, gScenarioBufferData106_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData106_EXTRACT_CHANNEL_3CH, 4},
{24, 14, 2688, 16, 0, gScenarioBufferData107_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData107_EXTRACT_CHANNEL_3CH, 4},
{32, 14, 3584, 16, 0, gScenarioBufferData108_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData108_EXTRACT_CHANNEL_3CH, 4},
{2, 16, 256, 16, 0, gScenarioBufferData109_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData109_EXTRACT_CHANNEL_3CH, 4},
{4, 16, 512, 16, 0, gScenarioBufferData110_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData110_EXTRACT_CHANNEL_3CH, 4},
{6, 16, 768, 16, 0, gScenarioBufferData111_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData111_EXTRACT_CHANNEL_3CH, 4},
{8, 16, 1024, 16, 0, gScenarioBufferData112_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData112_EXTRACT_CHANNEL_3CH, 4},
{10, 16, 1280, 16, 0, gScenarioBufferData113_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData113_EXTRACT_CHANNEL_3CH, 4},
{12, 16, 1536, 16, 0, gScenarioBufferData114_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData114_EXTRACT_CHANNEL_3CH, 4},
{16, 16, 2048, 16, 0, gScenarioBufferData115_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData115_EXTRACT_CHANNEL_3CH, 4},
{20, 16, 2560, 16, 0, gScenarioBufferData116_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData116_EXTRACT_CHANNEL_3CH, 4},
{24, 16, 3072, 16, 0, gScenarioBufferData117_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData117_EXTRACT_CHANNEL_3CH, 4},
{2, 18, 288, 16, 0, gScenarioBufferData118_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData118_EXTRACT_CHANNEL_3CH, 4},
{4, 18, 576, 16, 0, gScenarioBufferData119_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData119_EXTRACT_CHANNEL_3CH, 4},
{6, 18, 864, 16, 0, gScenarioBufferData120_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData120_EXTRACT_CHANNEL_3CH, 4},
{8, 18, 1152, 16, 0, gScenarioBufferData121_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData121_EXTRACT_CHANNEL_3CH, 4},
{10, 18, 1440, 16, 0, gScenarioBufferData122_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData122_EXTRACT_CHANNEL_3CH, 4},
{12, 18, 1728, 16, 0, gScenarioBufferData123_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData123_EXTRACT_CHANNEL_3CH, 4},
{16, 18, 2304, 16, 0, gScenarioBufferData124_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData124_EXTRACT_CHANNEL_3CH, 4},
{20, 18, 2880, 16, 0, gScenarioBufferData125_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData125_EXTRACT_CHANNEL_3CH, 4},
{24, 18, 3456, 16, 0, gScenarioBufferData126_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData126_EXTRACT_CHANNEL_3CH, 4},
{2, 20, 320, 16, 0, gScenarioBufferData127_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData127_EXTRACT_CHANNEL_3CH, 4},
{4, 20, 640, 16, 0, gScenarioBufferData128_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData128_EXTRACT_CHANNEL_3CH, 4},
{6, 20, 960, 16, 0, gScenarioBufferData129_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData129_EXTRACT_CHANNEL_3CH, 4},
{8, 20, 1280, 16, 0, gScenarioBufferData130_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData130_EXTRACT_CHANNEL_3CH, 4},
{10, 20, 1600, 16, 0, gScenarioBufferData131_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData131_EXTRACT_CHANNEL_3CH, 4},
{12, 20, 1920, 16, 0, gScenarioBufferData132_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData132_EXTRACT_CHANNEL_3CH, 4},
{16, 20, 2560, 16, 0, gScenarioBufferData133_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData133_EXTRACT_CHANNEL_3CH, 4},
{20, 20, 3200, 16, 0, gScenarioBufferData134_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData134_EXTRACT_CHANNEL_3CH, 4},
{24, 20, 3840, 16, 0, gScenarioBufferData135_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData135_EXTRACT_CHANNEL_3CH, 4},
{2, 22, 352, 16, 0, gScenarioBufferData136_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData136_EXTRACT_CHANNEL_3CH, 4},
{4, 22, 704, 16, 0, gScenarioBufferData137_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData137_EXTRACT_CHANNEL_3CH, 4},
{6, 22, 1056, 16, 0, gScenarioBufferData138_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData138_EXTRACT_CHANNEL_3CH, 4},
{8, 22, 1408, 16, 0, gScenarioBufferData139_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData139_EXTRACT_CHANNEL_3CH, 4},
{10, 22, 1760, 16, 0, gScenarioBufferData140_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData140_EXTRACT_CHANNEL_3CH, 4},
{12, 22, 2112, 16, 0, gScenarioBufferData141_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData141_EXTRACT_CHANNEL_3CH, 4},
{16, 22, 2816, 16, 0, gScenarioBufferData142_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData142_EXTRACT_CHANNEL_3CH, 4},
{20, 22, 3520, 16, 0, gScenarioBufferData143_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData143_EXTRACT_CHANNEL_3CH, 4},
{2, 24, 384, 16, 0, gScenarioBufferData144_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData144_EXTRACT_CHANNEL_3CH, 4},
{4, 24, 768, 16, 0, gScenarioBufferData145_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData145_EXTRACT_CHANNEL_3CH, 4},
{6, 24, 1152, 16, 0, gScenarioBufferData146_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData146_EXTRACT_CHANNEL_3CH, 4},
{8, 24, 1536, 16, 0, gScenarioBufferData147_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData147_EXTRACT_CHANNEL_3CH, 4},
{10, 24, 1920, 16, 0, gScenarioBufferData148_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData148_EXTRACT_CHANNEL_3CH, 4},
{12, 24, 2304, 16, 0, gScenarioBufferData149_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData149_EXTRACT_CHANNEL_3CH, 4},
{16, 24, 3072, 16, 0, gScenarioBufferData150_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData150_EXTRACT_CHANNEL_3CH, 4},
{20, 24, 3840, 16, 0, gScenarioBufferData151_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData151_EXTRACT_CHANNEL_3CH, 4},
{2, 26, 416, 16, 0, gScenarioBufferData152_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData152_EXTRACT_CHANNEL_3CH, 4},
{4, 26, 832, 16, 0, gScenarioBufferData153_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData153_EXTRACT_CHANNEL_3CH, 4},
{6, 26, 1248, 16, 0, gScenarioBufferData154_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData154_EXTRACT_CHANNEL_3CH, 4},
{8, 26, 1664, 16, 0, gScenarioBufferData155_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData155_EXTRACT_CHANNEL_3CH, 4},
{10, 26, 2080, 16, 0, gScenarioBufferData156_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData156_EXTRACT_CHANNEL_3CH, 4},
{12, 26, 2496, 16, 0, gScenarioBufferData157_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData157_EXTRACT_CHANNEL_3CH, 4},
{16, 26, 3328, 16, 0, gScenarioBufferData158_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData158_EXTRACT_CHANNEL_3CH, 4},
{2, 28, 448, 16, 0, gScenarioBufferData159_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData159_EXTRACT_CHANNEL_3CH, 4},
{4, 28, 896, 16, 0, gScenarioBufferData160_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData160_EXTRACT_CHANNEL_3CH, 4},
{6, 28, 1344, 16, 0, gScenarioBufferData161_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData161_EXTRACT_CHANNEL_3CH, 4},
{8, 28, 1792, 16, 0, gScenarioBufferData162_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData162_EXTRACT_CHANNEL_3CH, 4},
{10, 28, 2240, 16, 0, gScenarioBufferData163_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData163_EXTRACT_CHANNEL_3CH, 4},
{12, 28, 2688, 16, 0, gScenarioBufferData164_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData164_EXTRACT_CHANNEL_3CH, 4},
{16, 28, 3584, 16, 0, gScenarioBufferData165_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData165_EXTRACT_CHANNEL_3CH, 4},
{2, 30, 480, 16, 0, gScenarioBufferData166_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData166_EXTRACT_CHANNEL_3CH, 4},
{4, 30, 960, 16, 0, gScenarioBufferData167_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData167_EXTRACT_CHANNEL_3CH, 4},
{6, 30, 1440, 16, 0, gScenarioBufferData168_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData168_EXTRACT_CHANNEL_3CH, 4},
{8, 30, 1920, 16, 0, gScenarioBufferData169_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData169_EXTRACT_CHANNEL_3CH, 4},
{10, 30, 2400, 16, 0, gScenarioBufferData170_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData170_EXTRACT_CHANNEL_3CH, 4},
{12, 30, 2880, 16, 0, gScenarioBufferData171_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData171_EXTRACT_CHANNEL_3CH, 4},
{16, 30, 3840, 16, 0, gScenarioBufferData172_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData172_EXTRACT_CHANNEL_3CH, 4},
{2, 32, 512, 16, 0, gScenarioBufferData173_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData173_EXTRACT_CHANNEL_3CH, 4},
{4, 32, 1024, 16, 0, gScenarioBufferData174_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData174_EXTRACT_CHANNEL_3CH, 4},
{6, 32, 1536, 16, 0, gScenarioBufferData175_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData175_EXTRACT_CHANNEL_3CH, 4},
{8, 32, 2048, 16, 0, gScenarioBufferData176_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData176_EXTRACT_CHANNEL_3CH, 4},
{10, 32, 2560, 16, 0, gScenarioBufferData177_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData177_EXTRACT_CHANNEL_3CH, 4},
{12, 32, 3072, 16, 0, gScenarioBufferData178_EXTRACT_CHANNEL_3CH, 48, gScenarioKernelData178_EXTRACT_CHANNEL_3CH, 4}
};

static acf_scenario_list gScenarioList_EXTRACT_CHANNEL_3CH = {
179, //number of scenarios
gScenarioArray_EXTRACT_CHANNEL_3CH};
//**************************************************************

class EXTRACT_CHANNEL_3CH : public ACF_Process_APU
{

public:
   EXTRACT_CHANNEL_3CH(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("EXTRACT_CHANNEL_3CH");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("EXTRACT_CHANNEL_3CH",
                                        EXTRACT_CHANNEL_3CH_LOAD_SEGMENTS,
                                        EXTRACT_CHANNEL_3CH_LOAD_PMEM, EXTRACT_CHANNEL_3CH_LOAD_PMEM_SIZE,
                                        EXTRACT_CHANNEL_3CH_LOAD_DMEM, EXTRACT_CHANNEL_3CH_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(EXTRACT_CHANNEL_3CH_APEX_LOG_BUFFER, EXTRACT_CHANNEL_3CH_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("CH_ID", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_EXTRACT_CHANNEL_3CH);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_EXTRACT_CHANNEL_3CH

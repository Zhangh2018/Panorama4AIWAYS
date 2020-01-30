#ifndef _ACF_PROCESS_APU_THRESHOLD_32U
#define _ACF_PROCESS_APU_THRESHOLD_32U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <THRESHOLD_32U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_THRESHOLD_32U[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_THRESHOLD_32U[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_THRESHOLD_32U[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_THRESHOLD_32U[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_THRESHOLD_32U[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_THRESHOLD_32U[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_THRESHOLD_32U[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_THRESHOLD_32U[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_THRESHOLD_32U[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_THRESHOLD_32U[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_THRESHOLD_32U[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_THRESHOLD_32U[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_THRESHOLD_32U[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_THRESHOLD_32U[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_THRESHOLD_32U[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_THRESHOLD_32U[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_THRESHOLD_32U[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_THRESHOLD_32U[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_THRESHOLD_32U[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_THRESHOLD_32U[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_THRESHOLD_32U[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_THRESHOLD_32U[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_THRESHOLD_32U[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_THRESHOLD_32U[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_THRESHOLD_32U[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_THRESHOLD_32U[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_THRESHOLD_32U[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_THRESHOLD_32U[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_THRESHOLD_32U[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_THRESHOLD_32U[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_THRESHOLD_32U[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_THRESHOLD_32U[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_THRESHOLD_32U[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_THRESHOLD_32U[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_THRESHOLD_32U[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_THRESHOLD_32U[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_THRESHOLD_32U[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_THRESHOLD_32U[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_THRESHOLD_32U[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_THRESHOLD_32U[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_THRESHOLD_32U[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_THRESHOLD_32U[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_THRESHOLD_32U[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_THRESHOLD_32U[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_THRESHOLD_32U[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_THRESHOLD_32U[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_THRESHOLD_32U[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_THRESHOLD_32U[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_THRESHOLD_32U[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_THRESHOLD_32U[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_THRESHOLD_32U[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_THRESHOLD_32U[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_THRESHOLD_32U[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_THRESHOLD_32U[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_THRESHOLD_32U[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_THRESHOLD_32U[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_THRESHOLD_32U[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_THRESHOLD_32U[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_THRESHOLD_32U[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_THRESHOLD_32U[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_THRESHOLD_32U[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_THRESHOLD_32U[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_THRESHOLD_32U[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_THRESHOLD_32U[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_THRESHOLD_32U[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_THRESHOLD_32U[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_THRESHOLD_32U[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_THRESHOLD_32U[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_THRESHOLD_32U[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_THRESHOLD_32U[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_THRESHOLD_32U[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_THRESHOLD_32U[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_THRESHOLD_32U[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_THRESHOLD_32U[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_THRESHOLD_32U[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_THRESHOLD_32U[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_THRESHOLD_32U[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_THRESHOLD_32U[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_THRESHOLD_32U[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_THRESHOLD_32U[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_THRESHOLD_32U[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_THRESHOLD_32U[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_THRESHOLD_32U[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_THRESHOLD_32U[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_THRESHOLD_32U[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_THRESHOLD_32U[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_THRESHOLD_32U[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_THRESHOLD_32U[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_THRESHOLD_32U[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_THRESHOLD_32U[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_THRESHOLD_32U[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_THRESHOLD_32U[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_THRESHOLD_32U[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_THRESHOLD_32U[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_THRESHOLD_32U[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_THRESHOLD_32U[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_THRESHOLD_32U[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_THRESHOLD_32U[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_THRESHOLD_32U[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_THRESHOLD_32U[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_THRESHOLD_32U[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_THRESHOLD_32U[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_THRESHOLD_32U[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_THRESHOLD_32U[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_THRESHOLD_32U[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_THRESHOLD_32U[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_THRESHOLD_32U[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_THRESHOLD_32U[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_THRESHOLD_32U[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_THRESHOLD_32U[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_THRESHOLD_32U[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_THRESHOLD_32U[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_THRESHOLD_32U[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_THRESHOLD_32U[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_THRESHOLD_32U[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_THRESHOLD_32U[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_THRESHOLD_32U[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_THRESHOLD_32U[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_THRESHOLD_32U[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_THRESHOLD_32U[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_THRESHOLD_32U[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_THRESHOLD_32U[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_THRESHOLD_32U[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_THRESHOLD_32U[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_THRESHOLD_32U[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_THRESHOLD_32U[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_THRESHOLD_32U[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_THRESHOLD_32U[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_THRESHOLD_32U[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_THRESHOLD_32U[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_THRESHOLD_32U[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_THRESHOLD_32U[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_THRESHOLD_32U[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_THRESHOLD_32U[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_THRESHOLD_32U[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_THRESHOLD_32U[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_THRESHOLD_32U[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_THRESHOLD_32U[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_THRESHOLD_32U[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_THRESHOLD_32U[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_THRESHOLD_32U[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_THRESHOLD_32U[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_THRESHOLD_32U[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_THRESHOLD_32U[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_THRESHOLD_32U[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_THRESHOLD_32U[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_THRESHOLD_32U[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_THRESHOLD_32U[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_THRESHOLD_32U[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_THRESHOLD_32U[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_THRESHOLD_32U[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_THRESHOLD_32U[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_THRESHOLD_32U[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_THRESHOLD_32U[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_THRESHOLD_32U[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_THRESHOLD_32U[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_THRESHOLD_32U[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_THRESHOLD_32U[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_THRESHOLD_32U[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_THRESHOLD_32U[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_THRESHOLD_32U[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_THRESHOLD_32U[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_THRESHOLD_32U[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_THRESHOLD_32U[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_THRESHOLD_32U[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_THRESHOLD_32U[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_THRESHOLD_32U[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_THRESHOLD_32U[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_THRESHOLD_32U[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_THRESHOLD_32U[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_THRESHOLD_32U[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_THRESHOLD_32U[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_THRESHOLD_32U[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_THRESHOLD_32U[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_THRESHOLD_32U[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_THRESHOLD_32U[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_THRESHOLD_32U[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_THRESHOLD_32U[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_THRESHOLD_32U[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_THRESHOLD_32U[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_THRESHOLD_32U[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_THRESHOLD_32U[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_THRESHOLD_32U[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_THRESHOLD_32U[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_THRESHOLD_32U[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_THRESHOLD_32U[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_THRESHOLD_32U[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_THRESHOLD_32U[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_THRESHOLD_32U[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_THRESHOLD_32U[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_THRESHOLD_32U[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_THRESHOLD_32U[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_THRESHOLD_32U[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_THRESHOLD_32U[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_THRESHOLD_32U[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_THRESHOLD_32U[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_THRESHOLD_32U[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_THRESHOLD_32U[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_THRESHOLD_32U[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_THRESHOLD_32U[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_THRESHOLD_32U[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_THRESHOLD_32U[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_THRESHOLD_32U[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_THRESHOLD_32U[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_THRESHOLD_32U[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_THRESHOLD_32U[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_THRESHOLD_32U[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_THRESHOLD_32U[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_THRESHOLD_32U[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_THRESHOLD_32U[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_THRESHOLD_32U[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_THRESHOLD_32U[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_THRESHOLD_32U[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_THRESHOLD_32U[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_THRESHOLD_32U[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_THRESHOLD_32U[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_THRESHOLD_32U[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_THRESHOLD_32U[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_THRESHOLD_32U[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_THRESHOLD_32U[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_THRESHOLD_32U[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_THRESHOLD_32U[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_THRESHOLD_32U[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_THRESHOLD_32U[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_THRESHOLD_32U[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_THRESHOLD_32U[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_THRESHOLD_32U[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_THRESHOLD_32U[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_THRESHOLD_32U[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_THRESHOLD_32U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_THRESHOLD_32U[] = {{0, 0}};

static acf_scenario gScenarioArray_THRESHOLD_32U[] = {
{2, 1, 20, 32, 0, gScenarioBufferData0_THRESHOLD_32U, 64, gScenarioKernelData0_THRESHOLD_32U, 4},
{4, 1, 40, 32, 0, gScenarioBufferData1_THRESHOLD_32U, 64, gScenarioKernelData1_THRESHOLD_32U, 4},
{6, 1, 60, 32, 0, gScenarioBufferData2_THRESHOLD_32U, 64, gScenarioKernelData2_THRESHOLD_32U, 4},
{8, 1, 80, 32, 0, gScenarioBufferData3_THRESHOLD_32U, 64, gScenarioKernelData3_THRESHOLD_32U, 4},
{10, 1, 100, 32, 0, gScenarioBufferData4_THRESHOLD_32U, 64, gScenarioKernelData4_THRESHOLD_32U, 4},
{12, 1, 120, 32, 0, gScenarioBufferData5_THRESHOLD_32U, 64, gScenarioKernelData5_THRESHOLD_32U, 4},
{14, 1, 140, 32, 0, gScenarioBufferData6_THRESHOLD_32U, 64, gScenarioKernelData6_THRESHOLD_32U, 4},
{16, 1, 160, 32, 0, gScenarioBufferData7_THRESHOLD_32U, 64, gScenarioKernelData7_THRESHOLD_32U, 4},
{18, 1, 180, 32, 0, gScenarioBufferData8_THRESHOLD_32U, 64, gScenarioKernelData8_THRESHOLD_32U, 4},
{20, 1, 200, 32, 0, gScenarioBufferData9_THRESHOLD_32U, 64, gScenarioKernelData9_THRESHOLD_32U, 4},
{22, 1, 220, 32, 0, gScenarioBufferData10_THRESHOLD_32U, 64, gScenarioKernelData10_THRESHOLD_32U, 4},
{24, 1, 240, 32, 0, gScenarioBufferData11_THRESHOLD_32U, 64, gScenarioKernelData11_THRESHOLD_32U, 4},
{26, 1, 260, 32, 0, gScenarioBufferData12_THRESHOLD_32U, 64, gScenarioKernelData12_THRESHOLD_32U, 4},
{28, 1, 280, 32, 0, gScenarioBufferData13_THRESHOLD_32U, 64, gScenarioKernelData13_THRESHOLD_32U, 4},
{30, 1, 300, 32, 0, gScenarioBufferData14_THRESHOLD_32U, 64, gScenarioKernelData14_THRESHOLD_32U, 4},
{32, 1, 320, 32, 0, gScenarioBufferData15_THRESHOLD_32U, 64, gScenarioKernelData15_THRESHOLD_32U, 4},
{2, 2, 40, 32, 0, gScenarioBufferData16_THRESHOLD_32U, 64, gScenarioKernelData16_THRESHOLD_32U, 4},
{4, 2, 80, 32, 0, gScenarioBufferData17_THRESHOLD_32U, 64, gScenarioKernelData17_THRESHOLD_32U, 4},
{6, 2, 120, 32, 0, gScenarioBufferData18_THRESHOLD_32U, 64, gScenarioKernelData18_THRESHOLD_32U, 4},
{8, 2, 160, 32, 0, gScenarioBufferData19_THRESHOLD_32U, 64, gScenarioKernelData19_THRESHOLD_32U, 4},
{10, 2, 200, 32, 0, gScenarioBufferData20_THRESHOLD_32U, 64, gScenarioKernelData20_THRESHOLD_32U, 4},
{12, 2, 240, 32, 0, gScenarioBufferData21_THRESHOLD_32U, 64, gScenarioKernelData21_THRESHOLD_32U, 4},
{14, 2, 280, 32, 0, gScenarioBufferData22_THRESHOLD_32U, 64, gScenarioKernelData22_THRESHOLD_32U, 4},
{16, 2, 320, 32, 0, gScenarioBufferData23_THRESHOLD_32U, 64, gScenarioKernelData23_THRESHOLD_32U, 4},
{18, 2, 360, 32, 0, gScenarioBufferData24_THRESHOLD_32U, 64, gScenarioKernelData24_THRESHOLD_32U, 4},
{20, 2, 400, 32, 0, gScenarioBufferData25_THRESHOLD_32U, 64, gScenarioKernelData25_THRESHOLD_32U, 4},
{22, 2, 440, 32, 0, gScenarioBufferData26_THRESHOLD_32U, 64, gScenarioKernelData26_THRESHOLD_32U, 4},
{24, 2, 480, 32, 0, gScenarioBufferData27_THRESHOLD_32U, 64, gScenarioKernelData27_THRESHOLD_32U, 4},
{26, 2, 520, 32, 0, gScenarioBufferData28_THRESHOLD_32U, 64, gScenarioKernelData28_THRESHOLD_32U, 4},
{28, 2, 560, 32, 0, gScenarioBufferData29_THRESHOLD_32U, 64, gScenarioKernelData29_THRESHOLD_32U, 4},
{30, 2, 600, 32, 0, gScenarioBufferData30_THRESHOLD_32U, 64, gScenarioKernelData30_THRESHOLD_32U, 4},
{32, 2, 640, 32, 0, gScenarioBufferData31_THRESHOLD_32U, 64, gScenarioKernelData31_THRESHOLD_32U, 4},
{2, 3, 60, 32, 0, gScenarioBufferData32_THRESHOLD_32U, 64, gScenarioKernelData32_THRESHOLD_32U, 4},
{4, 3, 120, 32, 0, gScenarioBufferData33_THRESHOLD_32U, 64, gScenarioKernelData33_THRESHOLD_32U, 4},
{6, 3, 180, 32, 0, gScenarioBufferData34_THRESHOLD_32U, 64, gScenarioKernelData34_THRESHOLD_32U, 4},
{8, 3, 240, 32, 0, gScenarioBufferData35_THRESHOLD_32U, 64, gScenarioKernelData35_THRESHOLD_32U, 4},
{10, 3, 300, 32, 0, gScenarioBufferData36_THRESHOLD_32U, 64, gScenarioKernelData36_THRESHOLD_32U, 4},
{12, 3, 360, 32, 0, gScenarioBufferData37_THRESHOLD_32U, 64, gScenarioKernelData37_THRESHOLD_32U, 4},
{14, 3, 420, 32, 0, gScenarioBufferData38_THRESHOLD_32U, 64, gScenarioKernelData38_THRESHOLD_32U, 4},
{16, 3, 480, 32, 0, gScenarioBufferData39_THRESHOLD_32U, 64, gScenarioKernelData39_THRESHOLD_32U, 4},
{18, 3, 540, 32, 0, gScenarioBufferData40_THRESHOLD_32U, 64, gScenarioKernelData40_THRESHOLD_32U, 4},
{20, 3, 600, 32, 0, gScenarioBufferData41_THRESHOLD_32U, 64, gScenarioKernelData41_THRESHOLD_32U, 4},
{22, 3, 660, 32, 0, gScenarioBufferData42_THRESHOLD_32U, 64, gScenarioKernelData42_THRESHOLD_32U, 4},
{24, 3, 720, 32, 0, gScenarioBufferData43_THRESHOLD_32U, 64, gScenarioKernelData43_THRESHOLD_32U, 4},
{26, 3, 780, 32, 0, gScenarioBufferData44_THRESHOLD_32U, 64, gScenarioKernelData44_THRESHOLD_32U, 4},
{28, 3, 840, 32, 0, gScenarioBufferData45_THRESHOLD_32U, 64, gScenarioKernelData45_THRESHOLD_32U, 4},
{30, 3, 900, 32, 0, gScenarioBufferData46_THRESHOLD_32U, 64, gScenarioKernelData46_THRESHOLD_32U, 4},
{32, 3, 960, 32, 0, gScenarioBufferData47_THRESHOLD_32U, 64, gScenarioKernelData47_THRESHOLD_32U, 4},
{2, 4, 80, 32, 0, gScenarioBufferData48_THRESHOLD_32U, 64, gScenarioKernelData48_THRESHOLD_32U, 4},
{4, 4, 160, 32, 0, gScenarioBufferData49_THRESHOLD_32U, 64, gScenarioKernelData49_THRESHOLD_32U, 4},
{6, 4, 240, 32, 0, gScenarioBufferData50_THRESHOLD_32U, 64, gScenarioKernelData50_THRESHOLD_32U, 4},
{8, 4, 320, 32, 0, gScenarioBufferData51_THRESHOLD_32U, 64, gScenarioKernelData51_THRESHOLD_32U, 4},
{10, 4, 400, 32, 0, gScenarioBufferData52_THRESHOLD_32U, 64, gScenarioKernelData52_THRESHOLD_32U, 4},
{12, 4, 480, 32, 0, gScenarioBufferData53_THRESHOLD_32U, 64, gScenarioKernelData53_THRESHOLD_32U, 4},
{14, 4, 560, 32, 0, gScenarioBufferData54_THRESHOLD_32U, 64, gScenarioKernelData54_THRESHOLD_32U, 4},
{16, 4, 640, 32, 0, gScenarioBufferData55_THRESHOLD_32U, 64, gScenarioKernelData55_THRESHOLD_32U, 4},
{18, 4, 720, 32, 0, gScenarioBufferData56_THRESHOLD_32U, 64, gScenarioKernelData56_THRESHOLD_32U, 4},
{20, 4, 800, 32, 0, gScenarioBufferData57_THRESHOLD_32U, 64, gScenarioKernelData57_THRESHOLD_32U, 4},
{22, 4, 880, 32, 0, gScenarioBufferData58_THRESHOLD_32U, 64, gScenarioKernelData58_THRESHOLD_32U, 4},
{24, 4, 960, 32, 0, gScenarioBufferData59_THRESHOLD_32U, 64, gScenarioKernelData59_THRESHOLD_32U, 4},
{26, 4, 1040, 32, 0, gScenarioBufferData60_THRESHOLD_32U, 64, gScenarioKernelData60_THRESHOLD_32U, 4},
{28, 4, 1120, 32, 0, gScenarioBufferData61_THRESHOLD_32U, 64, gScenarioKernelData61_THRESHOLD_32U, 4},
{30, 4, 1200, 32, 0, gScenarioBufferData62_THRESHOLD_32U, 64, gScenarioKernelData62_THRESHOLD_32U, 4},
{32, 4, 1280, 32, 0, gScenarioBufferData63_THRESHOLD_32U, 64, gScenarioKernelData63_THRESHOLD_32U, 4},
{2, 5, 100, 32, 0, gScenarioBufferData64_THRESHOLD_32U, 64, gScenarioKernelData64_THRESHOLD_32U, 4},
{4, 5, 200, 32, 0, gScenarioBufferData65_THRESHOLD_32U, 64, gScenarioKernelData65_THRESHOLD_32U, 4},
{6, 5, 300, 32, 0, gScenarioBufferData66_THRESHOLD_32U, 64, gScenarioKernelData66_THRESHOLD_32U, 4},
{8, 5, 400, 32, 0, gScenarioBufferData67_THRESHOLD_32U, 64, gScenarioKernelData67_THRESHOLD_32U, 4},
{10, 5, 500, 32, 0, gScenarioBufferData68_THRESHOLD_32U, 64, gScenarioKernelData68_THRESHOLD_32U, 4},
{12, 5, 600, 32, 0, gScenarioBufferData69_THRESHOLD_32U, 64, gScenarioKernelData69_THRESHOLD_32U, 4},
{14, 5, 700, 32, 0, gScenarioBufferData70_THRESHOLD_32U, 64, gScenarioKernelData70_THRESHOLD_32U, 4},
{16, 5, 800, 32, 0, gScenarioBufferData71_THRESHOLD_32U, 64, gScenarioKernelData71_THRESHOLD_32U, 4},
{18, 5, 900, 32, 0, gScenarioBufferData72_THRESHOLD_32U, 64, gScenarioKernelData72_THRESHOLD_32U, 4},
{20, 5, 1000, 32, 0, gScenarioBufferData73_THRESHOLD_32U, 64, gScenarioKernelData73_THRESHOLD_32U, 4},
{22, 5, 1100, 32, 0, gScenarioBufferData74_THRESHOLD_32U, 64, gScenarioKernelData74_THRESHOLD_32U, 4},
{24, 5, 1200, 32, 0, gScenarioBufferData75_THRESHOLD_32U, 64, gScenarioKernelData75_THRESHOLD_32U, 4},
{26, 5, 1300, 32, 0, gScenarioBufferData76_THRESHOLD_32U, 64, gScenarioKernelData76_THRESHOLD_32U, 4},
{28, 5, 1400, 32, 0, gScenarioBufferData77_THRESHOLD_32U, 64, gScenarioKernelData77_THRESHOLD_32U, 4},
{30, 5, 1500, 32, 0, gScenarioBufferData78_THRESHOLD_32U, 64, gScenarioKernelData78_THRESHOLD_32U, 4},
{32, 5, 1600, 32, 0, gScenarioBufferData79_THRESHOLD_32U, 64, gScenarioKernelData79_THRESHOLD_32U, 4},
{2, 6, 120, 32, 0, gScenarioBufferData80_THRESHOLD_32U, 64, gScenarioKernelData80_THRESHOLD_32U, 4},
{4, 6, 240, 32, 0, gScenarioBufferData81_THRESHOLD_32U, 64, gScenarioKernelData81_THRESHOLD_32U, 4},
{6, 6, 360, 32, 0, gScenarioBufferData82_THRESHOLD_32U, 64, gScenarioKernelData82_THRESHOLD_32U, 4},
{8, 6, 480, 32, 0, gScenarioBufferData83_THRESHOLD_32U, 64, gScenarioKernelData83_THRESHOLD_32U, 4},
{10, 6, 600, 32, 0, gScenarioBufferData84_THRESHOLD_32U, 64, gScenarioKernelData84_THRESHOLD_32U, 4},
{12, 6, 720, 32, 0, gScenarioBufferData85_THRESHOLD_32U, 64, gScenarioKernelData85_THRESHOLD_32U, 4},
{14, 6, 840, 32, 0, gScenarioBufferData86_THRESHOLD_32U, 64, gScenarioKernelData86_THRESHOLD_32U, 4},
{16, 6, 960, 32, 0, gScenarioBufferData87_THRESHOLD_32U, 64, gScenarioKernelData87_THRESHOLD_32U, 4},
{18, 6, 1080, 32, 0, gScenarioBufferData88_THRESHOLD_32U, 64, gScenarioKernelData88_THRESHOLD_32U, 4},
{20, 6, 1200, 32, 0, gScenarioBufferData89_THRESHOLD_32U, 64, gScenarioKernelData89_THRESHOLD_32U, 4},
{22, 6, 1320, 32, 0, gScenarioBufferData90_THRESHOLD_32U, 64, gScenarioKernelData90_THRESHOLD_32U, 4},
{24, 6, 1440, 32, 0, gScenarioBufferData91_THRESHOLD_32U, 64, gScenarioKernelData91_THRESHOLD_32U, 4},
{26, 6, 1560, 32, 0, gScenarioBufferData92_THRESHOLD_32U, 64, gScenarioKernelData92_THRESHOLD_32U, 4},
{28, 6, 1680, 32, 0, gScenarioBufferData93_THRESHOLD_32U, 64, gScenarioKernelData93_THRESHOLD_32U, 4},
{30, 6, 1800, 32, 0, gScenarioBufferData94_THRESHOLD_32U, 64, gScenarioKernelData94_THRESHOLD_32U, 4},
{32, 6, 1920, 32, 0, gScenarioBufferData95_THRESHOLD_32U, 64, gScenarioKernelData95_THRESHOLD_32U, 4},
{2, 8, 160, 32, 0, gScenarioBufferData96_THRESHOLD_32U, 64, gScenarioKernelData96_THRESHOLD_32U, 4},
{4, 8, 320, 32, 0, gScenarioBufferData97_THRESHOLD_32U, 64, gScenarioKernelData97_THRESHOLD_32U, 4},
{6, 8, 480, 32, 0, gScenarioBufferData98_THRESHOLD_32U, 64, gScenarioKernelData98_THRESHOLD_32U, 4},
{8, 8, 640, 32, 0, gScenarioBufferData99_THRESHOLD_32U, 64, gScenarioKernelData99_THRESHOLD_32U, 4},
{10, 8, 800, 32, 0, gScenarioBufferData100_THRESHOLD_32U, 64, gScenarioKernelData100_THRESHOLD_32U, 4},
{12, 8, 960, 32, 0, gScenarioBufferData101_THRESHOLD_32U, 64, gScenarioKernelData101_THRESHOLD_32U, 4},
{14, 8, 1120, 32, 0, gScenarioBufferData102_THRESHOLD_32U, 64, gScenarioKernelData102_THRESHOLD_32U, 4},
{16, 8, 1280, 32, 0, gScenarioBufferData103_THRESHOLD_32U, 64, gScenarioKernelData103_THRESHOLD_32U, 4},
{18, 8, 1440, 32, 0, gScenarioBufferData104_THRESHOLD_32U, 64, gScenarioKernelData104_THRESHOLD_32U, 4},
{20, 8, 1600, 32, 0, gScenarioBufferData105_THRESHOLD_32U, 64, gScenarioKernelData105_THRESHOLD_32U, 4},
{22, 8, 1760, 32, 0, gScenarioBufferData106_THRESHOLD_32U, 64, gScenarioKernelData106_THRESHOLD_32U, 4},
{24, 8, 1920, 32, 0, gScenarioBufferData107_THRESHOLD_32U, 64, gScenarioKernelData107_THRESHOLD_32U, 4},
{26, 8, 2080, 32, 0, gScenarioBufferData108_THRESHOLD_32U, 64, gScenarioKernelData108_THRESHOLD_32U, 4},
{28, 8, 2240, 32, 0, gScenarioBufferData109_THRESHOLD_32U, 64, gScenarioKernelData109_THRESHOLD_32U, 4},
{30, 8, 2400, 32, 0, gScenarioBufferData110_THRESHOLD_32U, 64, gScenarioKernelData110_THRESHOLD_32U, 4},
{32, 8, 2560, 32, 0, gScenarioBufferData111_THRESHOLD_32U, 64, gScenarioKernelData111_THRESHOLD_32U, 4},
{2, 10, 200, 32, 0, gScenarioBufferData112_THRESHOLD_32U, 64, gScenarioKernelData112_THRESHOLD_32U, 4},
{4, 10, 400, 32, 0, gScenarioBufferData113_THRESHOLD_32U, 64, gScenarioKernelData113_THRESHOLD_32U, 4},
{6, 10, 600, 32, 0, gScenarioBufferData114_THRESHOLD_32U, 64, gScenarioKernelData114_THRESHOLD_32U, 4},
{8, 10, 800, 32, 0, gScenarioBufferData115_THRESHOLD_32U, 64, gScenarioKernelData115_THRESHOLD_32U, 4},
{10, 10, 1000, 32, 0, gScenarioBufferData116_THRESHOLD_32U, 64, gScenarioKernelData116_THRESHOLD_32U, 4},
{12, 10, 1200, 32, 0, gScenarioBufferData117_THRESHOLD_32U, 64, gScenarioKernelData117_THRESHOLD_32U, 4},
{14, 10, 1400, 32, 0, gScenarioBufferData118_THRESHOLD_32U, 64, gScenarioKernelData118_THRESHOLD_32U, 4},
{16, 10, 1600, 32, 0, gScenarioBufferData119_THRESHOLD_32U, 64, gScenarioKernelData119_THRESHOLD_32U, 4},
{18, 10, 1800, 32, 0, gScenarioBufferData120_THRESHOLD_32U, 64, gScenarioKernelData120_THRESHOLD_32U, 4},
{20, 10, 2000, 32, 0, gScenarioBufferData121_THRESHOLD_32U, 64, gScenarioKernelData121_THRESHOLD_32U, 4},
{22, 10, 2200, 32, 0, gScenarioBufferData122_THRESHOLD_32U, 64, gScenarioKernelData122_THRESHOLD_32U, 4},
{24, 10, 2400, 32, 0, gScenarioBufferData123_THRESHOLD_32U, 64, gScenarioKernelData123_THRESHOLD_32U, 4},
{26, 10, 2600, 32, 0, gScenarioBufferData124_THRESHOLD_32U, 64, gScenarioKernelData124_THRESHOLD_32U, 4},
{28, 10, 2800, 32, 0, gScenarioBufferData125_THRESHOLD_32U, 64, gScenarioKernelData125_THRESHOLD_32U, 4},
{30, 10, 3000, 32, 0, gScenarioBufferData126_THRESHOLD_32U, 64, gScenarioKernelData126_THRESHOLD_32U, 4},
{32, 10, 3200, 32, 0, gScenarioBufferData127_THRESHOLD_32U, 64, gScenarioKernelData127_THRESHOLD_32U, 4},
{2, 12, 240, 32, 0, gScenarioBufferData128_THRESHOLD_32U, 64, gScenarioKernelData128_THRESHOLD_32U, 4},
{4, 12, 480, 32, 0, gScenarioBufferData129_THRESHOLD_32U, 64, gScenarioKernelData129_THRESHOLD_32U, 4},
{6, 12, 720, 32, 0, gScenarioBufferData130_THRESHOLD_32U, 64, gScenarioKernelData130_THRESHOLD_32U, 4},
{8, 12, 960, 32, 0, gScenarioBufferData131_THRESHOLD_32U, 64, gScenarioKernelData131_THRESHOLD_32U, 4},
{10, 12, 1200, 32, 0, gScenarioBufferData132_THRESHOLD_32U, 64, gScenarioKernelData132_THRESHOLD_32U, 4},
{12, 12, 1440, 32, 0, gScenarioBufferData133_THRESHOLD_32U, 64, gScenarioKernelData133_THRESHOLD_32U, 4},
{14, 12, 1680, 32, 0, gScenarioBufferData134_THRESHOLD_32U, 64, gScenarioKernelData134_THRESHOLD_32U, 4},
{16, 12, 1920, 32, 0, gScenarioBufferData135_THRESHOLD_32U, 64, gScenarioKernelData135_THRESHOLD_32U, 4},
{18, 12, 2160, 32, 0, gScenarioBufferData136_THRESHOLD_32U, 64, gScenarioKernelData136_THRESHOLD_32U, 4},
{20, 12, 2400, 32, 0, gScenarioBufferData137_THRESHOLD_32U, 64, gScenarioKernelData137_THRESHOLD_32U, 4},
{22, 12, 2640, 32, 0, gScenarioBufferData138_THRESHOLD_32U, 64, gScenarioKernelData138_THRESHOLD_32U, 4},
{24, 12, 2880, 32, 0, gScenarioBufferData139_THRESHOLD_32U, 64, gScenarioKernelData139_THRESHOLD_32U, 4},
{26, 12, 3120, 32, 0, gScenarioBufferData140_THRESHOLD_32U, 64, gScenarioKernelData140_THRESHOLD_32U, 4},
{28, 12, 3360, 32, 0, gScenarioBufferData141_THRESHOLD_32U, 64, gScenarioKernelData141_THRESHOLD_32U, 4},
{30, 12, 3600, 32, 0, gScenarioBufferData142_THRESHOLD_32U, 64, gScenarioKernelData142_THRESHOLD_32U, 4},
{32, 12, 3840, 32, 0, gScenarioBufferData143_THRESHOLD_32U, 64, gScenarioKernelData143_THRESHOLD_32U, 4},
{2, 14, 280, 32, 0, gScenarioBufferData144_THRESHOLD_32U, 64, gScenarioKernelData144_THRESHOLD_32U, 4},
{4, 14, 560, 32, 0, gScenarioBufferData145_THRESHOLD_32U, 64, gScenarioKernelData145_THRESHOLD_32U, 4},
{6, 14, 840, 32, 0, gScenarioBufferData146_THRESHOLD_32U, 64, gScenarioKernelData146_THRESHOLD_32U, 4},
{8, 14, 1120, 32, 0, gScenarioBufferData147_THRESHOLD_32U, 64, gScenarioKernelData147_THRESHOLD_32U, 4},
{10, 14, 1400, 32, 0, gScenarioBufferData148_THRESHOLD_32U, 64, gScenarioKernelData148_THRESHOLD_32U, 4},
{12, 14, 1680, 32, 0, gScenarioBufferData149_THRESHOLD_32U, 64, gScenarioKernelData149_THRESHOLD_32U, 4},
{14, 14, 1960, 32, 0, gScenarioBufferData150_THRESHOLD_32U, 64, gScenarioKernelData150_THRESHOLD_32U, 4},
{16, 14, 2240, 32, 0, gScenarioBufferData151_THRESHOLD_32U, 64, gScenarioKernelData151_THRESHOLD_32U, 4},
{18, 14, 2520, 32, 0, gScenarioBufferData152_THRESHOLD_32U, 64, gScenarioKernelData152_THRESHOLD_32U, 4},
{20, 14, 2800, 32, 0, gScenarioBufferData153_THRESHOLD_32U, 64, gScenarioKernelData153_THRESHOLD_32U, 4},
{22, 14, 3080, 32, 0, gScenarioBufferData154_THRESHOLD_32U, 64, gScenarioKernelData154_THRESHOLD_32U, 4},
{24, 14, 3360, 32, 0, gScenarioBufferData155_THRESHOLD_32U, 64, gScenarioKernelData155_THRESHOLD_32U, 4},
{26, 14, 3640, 32, 0, gScenarioBufferData156_THRESHOLD_32U, 64, gScenarioKernelData156_THRESHOLD_32U, 4},
{2, 16, 320, 32, 0, gScenarioBufferData157_THRESHOLD_32U, 64, gScenarioKernelData157_THRESHOLD_32U, 4},
{4, 16, 640, 32, 0, gScenarioBufferData158_THRESHOLD_32U, 64, gScenarioKernelData158_THRESHOLD_32U, 4},
{6, 16, 960, 32, 0, gScenarioBufferData159_THRESHOLD_32U, 64, gScenarioKernelData159_THRESHOLD_32U, 4},
{8, 16, 1280, 32, 0, gScenarioBufferData160_THRESHOLD_32U, 64, gScenarioKernelData160_THRESHOLD_32U, 4},
{10, 16, 1600, 32, 0, gScenarioBufferData161_THRESHOLD_32U, 64, gScenarioKernelData161_THRESHOLD_32U, 4},
{12, 16, 1920, 32, 0, gScenarioBufferData162_THRESHOLD_32U, 64, gScenarioKernelData162_THRESHOLD_32U, 4},
{14, 16, 2240, 32, 0, gScenarioBufferData163_THRESHOLD_32U, 64, gScenarioKernelData163_THRESHOLD_32U, 4},
{16, 16, 2560, 32, 0, gScenarioBufferData164_THRESHOLD_32U, 64, gScenarioKernelData164_THRESHOLD_32U, 4},
{18, 16, 2880, 32, 0, gScenarioBufferData165_THRESHOLD_32U, 64, gScenarioKernelData165_THRESHOLD_32U, 4},
{20, 16, 3200, 32, 0, gScenarioBufferData166_THRESHOLD_32U, 64, gScenarioKernelData166_THRESHOLD_32U, 4},
{22, 16, 3520, 32, 0, gScenarioBufferData167_THRESHOLD_32U, 64, gScenarioKernelData167_THRESHOLD_32U, 4},
{24, 16, 3840, 32, 0, gScenarioBufferData168_THRESHOLD_32U, 64, gScenarioKernelData168_THRESHOLD_32U, 4},
{2, 18, 360, 32, 0, gScenarioBufferData169_THRESHOLD_32U, 64, gScenarioKernelData169_THRESHOLD_32U, 4},
{4, 18, 720, 32, 0, gScenarioBufferData170_THRESHOLD_32U, 64, gScenarioKernelData170_THRESHOLD_32U, 4},
{6, 18, 1080, 32, 0, gScenarioBufferData171_THRESHOLD_32U, 64, gScenarioKernelData171_THRESHOLD_32U, 4},
{8, 18, 1440, 32, 0, gScenarioBufferData172_THRESHOLD_32U, 64, gScenarioKernelData172_THRESHOLD_32U, 4},
{10, 18, 1800, 32, 0, gScenarioBufferData173_THRESHOLD_32U, 64, gScenarioKernelData173_THRESHOLD_32U, 4},
{12, 18, 2160, 32, 0, gScenarioBufferData174_THRESHOLD_32U, 64, gScenarioKernelData174_THRESHOLD_32U, 4},
{14, 18, 2520, 32, 0, gScenarioBufferData175_THRESHOLD_32U, 64, gScenarioKernelData175_THRESHOLD_32U, 4},
{16, 18, 2880, 32, 0, gScenarioBufferData176_THRESHOLD_32U, 64, gScenarioKernelData176_THRESHOLD_32U, 4},
{18, 18, 3240, 32, 0, gScenarioBufferData177_THRESHOLD_32U, 64, gScenarioKernelData177_THRESHOLD_32U, 4},
{20, 18, 3600, 32, 0, gScenarioBufferData178_THRESHOLD_32U, 64, gScenarioKernelData178_THRESHOLD_32U, 4},
{2, 20, 400, 32, 0, gScenarioBufferData179_THRESHOLD_32U, 64, gScenarioKernelData179_THRESHOLD_32U, 4},
{4, 20, 800, 32, 0, gScenarioBufferData180_THRESHOLD_32U, 64, gScenarioKernelData180_THRESHOLD_32U, 4},
{6, 20, 1200, 32, 0, gScenarioBufferData181_THRESHOLD_32U, 64, gScenarioKernelData181_THRESHOLD_32U, 4},
{8, 20, 1600, 32, 0, gScenarioBufferData182_THRESHOLD_32U, 64, gScenarioKernelData182_THRESHOLD_32U, 4},
{10, 20, 2000, 32, 0, gScenarioBufferData183_THRESHOLD_32U, 64, gScenarioKernelData183_THRESHOLD_32U, 4},
{12, 20, 2400, 32, 0, gScenarioBufferData184_THRESHOLD_32U, 64, gScenarioKernelData184_THRESHOLD_32U, 4},
{14, 20, 2800, 32, 0, gScenarioBufferData185_THRESHOLD_32U, 64, gScenarioKernelData185_THRESHOLD_32U, 4},
{16, 20, 3200, 32, 0, gScenarioBufferData186_THRESHOLD_32U, 64, gScenarioKernelData186_THRESHOLD_32U, 4},
{18, 20, 3600, 32, 0, gScenarioBufferData187_THRESHOLD_32U, 64, gScenarioKernelData187_THRESHOLD_32U, 4},
{2, 22, 440, 32, 0, gScenarioBufferData188_THRESHOLD_32U, 64, gScenarioKernelData188_THRESHOLD_32U, 4},
{4, 22, 880, 32, 0, gScenarioBufferData189_THRESHOLD_32U, 64, gScenarioKernelData189_THRESHOLD_32U, 4},
{6, 22, 1320, 32, 0, gScenarioBufferData190_THRESHOLD_32U, 64, gScenarioKernelData190_THRESHOLD_32U, 4},
{8, 22, 1760, 32, 0, gScenarioBufferData191_THRESHOLD_32U, 64, gScenarioKernelData191_THRESHOLD_32U, 4},
{10, 22, 2200, 32, 0, gScenarioBufferData192_THRESHOLD_32U, 64, gScenarioKernelData192_THRESHOLD_32U, 4},
{12, 22, 2640, 32, 0, gScenarioBufferData193_THRESHOLD_32U, 64, gScenarioKernelData193_THRESHOLD_32U, 4},
{14, 22, 3080, 32, 0, gScenarioBufferData194_THRESHOLD_32U, 64, gScenarioKernelData194_THRESHOLD_32U, 4},
{16, 22, 3520, 32, 0, gScenarioBufferData195_THRESHOLD_32U, 64, gScenarioKernelData195_THRESHOLD_32U, 4},
{2, 24, 480, 32, 0, gScenarioBufferData196_THRESHOLD_32U, 64, gScenarioKernelData196_THRESHOLD_32U, 4},
{4, 24, 960, 32, 0, gScenarioBufferData197_THRESHOLD_32U, 64, gScenarioKernelData197_THRESHOLD_32U, 4},
{6, 24, 1440, 32, 0, gScenarioBufferData198_THRESHOLD_32U, 64, gScenarioKernelData198_THRESHOLD_32U, 4},
{8, 24, 1920, 32, 0, gScenarioBufferData199_THRESHOLD_32U, 64, gScenarioKernelData199_THRESHOLD_32U, 4},
{10, 24, 2400, 32, 0, gScenarioBufferData200_THRESHOLD_32U, 64, gScenarioKernelData200_THRESHOLD_32U, 4},
{12, 24, 2880, 32, 0, gScenarioBufferData201_THRESHOLD_32U, 64, gScenarioKernelData201_THRESHOLD_32U, 4},
{14, 24, 3360, 32, 0, gScenarioBufferData202_THRESHOLD_32U, 64, gScenarioKernelData202_THRESHOLD_32U, 4},
{16, 24, 3840, 32, 0, gScenarioBufferData203_THRESHOLD_32U, 64, gScenarioKernelData203_THRESHOLD_32U, 4},
{2, 26, 520, 32, 0, gScenarioBufferData204_THRESHOLD_32U, 64, gScenarioKernelData204_THRESHOLD_32U, 4},
{4, 26, 1040, 32, 0, gScenarioBufferData205_THRESHOLD_32U, 64, gScenarioKernelData205_THRESHOLD_32U, 4},
{6, 26, 1560, 32, 0, gScenarioBufferData206_THRESHOLD_32U, 64, gScenarioKernelData206_THRESHOLD_32U, 4},
{8, 26, 2080, 32, 0, gScenarioBufferData207_THRESHOLD_32U, 64, gScenarioKernelData207_THRESHOLD_32U, 4},
{10, 26, 2600, 32, 0, gScenarioBufferData208_THRESHOLD_32U, 64, gScenarioKernelData208_THRESHOLD_32U, 4},
{12, 26, 3120, 32, 0, gScenarioBufferData209_THRESHOLD_32U, 64, gScenarioKernelData209_THRESHOLD_32U, 4},
{14, 26, 3640, 32, 0, gScenarioBufferData210_THRESHOLD_32U, 64, gScenarioKernelData210_THRESHOLD_32U, 4},
{2, 28, 560, 32, 0, gScenarioBufferData211_THRESHOLD_32U, 64, gScenarioKernelData211_THRESHOLD_32U, 4},
{4, 28, 1120, 32, 0, gScenarioBufferData212_THRESHOLD_32U, 64, gScenarioKernelData212_THRESHOLD_32U, 4},
{6, 28, 1680, 32, 0, gScenarioBufferData213_THRESHOLD_32U, 64, gScenarioKernelData213_THRESHOLD_32U, 4},
{8, 28, 2240, 32, 0, gScenarioBufferData214_THRESHOLD_32U, 64, gScenarioKernelData214_THRESHOLD_32U, 4},
{10, 28, 2800, 32, 0, gScenarioBufferData215_THRESHOLD_32U, 64, gScenarioKernelData215_THRESHOLD_32U, 4},
{12, 28, 3360, 32, 0, gScenarioBufferData216_THRESHOLD_32U, 64, gScenarioKernelData216_THRESHOLD_32U, 4},
{2, 30, 600, 32, 0, gScenarioBufferData217_THRESHOLD_32U, 64, gScenarioKernelData217_THRESHOLD_32U, 4},
{4, 30, 1200, 32, 0, gScenarioBufferData218_THRESHOLD_32U, 64, gScenarioKernelData218_THRESHOLD_32U, 4},
{6, 30, 1800, 32, 0, gScenarioBufferData219_THRESHOLD_32U, 64, gScenarioKernelData219_THRESHOLD_32U, 4},
{8, 30, 2400, 32, 0, gScenarioBufferData220_THRESHOLD_32U, 64, gScenarioKernelData220_THRESHOLD_32U, 4},
{10, 30, 3000, 32, 0, gScenarioBufferData221_THRESHOLD_32U, 64, gScenarioKernelData221_THRESHOLD_32U, 4},
{12, 30, 3600, 32, 0, gScenarioBufferData222_THRESHOLD_32U, 64, gScenarioKernelData222_THRESHOLD_32U, 4},
{2, 32, 640, 32, 0, gScenarioBufferData223_THRESHOLD_32U, 64, gScenarioKernelData223_THRESHOLD_32U, 4},
{4, 32, 1280, 32, 0, gScenarioBufferData224_THRESHOLD_32U, 64, gScenarioKernelData224_THRESHOLD_32U, 4},
{6, 32, 1920, 32, 0, gScenarioBufferData225_THRESHOLD_32U, 64, gScenarioKernelData225_THRESHOLD_32U, 4},
{8, 32, 2560, 32, 0, gScenarioBufferData226_THRESHOLD_32U, 64, gScenarioKernelData226_THRESHOLD_32U, 4},
{10, 32, 3200, 32, 0, gScenarioBufferData227_THRESHOLD_32U, 64, gScenarioKernelData227_THRESHOLD_32U, 4},
{12, 32, 3840, 32, 0, gScenarioBufferData228_THRESHOLD_32U, 64, gScenarioKernelData228_THRESHOLD_32U, 4}
};

static acf_scenario_list gScenarioList_THRESHOLD_32U = {
229, //number of scenarios
gScenarioArray_THRESHOLD_32U};
//**************************************************************

class THRESHOLD_32U : public ACF_Process_APU
{

public:
   THRESHOLD_32U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("THRESHOLD_32U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("THRESHOLD_32U",
                                        THRESHOLD_32U_LOAD_SEGMENTS,
                                        THRESHOLD_32U_LOAD_PMEM, THRESHOLD_32U_LOAD_PMEM_SIZE,
                                        THRESHOLD_32U_LOAD_DMEM, THRESHOLD_32U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(THRESHOLD_32U_APEX_LOG_BUFFER, THRESHOLD_32U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_32U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("INPUT_2", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_THRESHOLD_32U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_THRESHOLD_32U

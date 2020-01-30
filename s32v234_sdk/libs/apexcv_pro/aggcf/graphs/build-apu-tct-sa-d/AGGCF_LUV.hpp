#ifndef _ACF_PROCESS_APU_AGGCF_LUV
#define _ACF_PROCESS_APU_AGGCF_LUV

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_LUV_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_LUV[] = {{1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_LUV[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_LUV[] = {{3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_LUV[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_LUV[] = {{5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_LUV[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_LUV[] = {{7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_LUV[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_LUV[] = {{9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_LUV[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_LUV[] = {{11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_LUV[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_LUV[] = {{13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_LUV[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_LUV[] = {{15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_LUV[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_LUV[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_LUV[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_LUV[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_LUV[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_LUV[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_LUV[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_LUV[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_LUV[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_LUV[] = {{1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_LUV[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_LUV[] = {{3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_LUV[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AGGCF_LUV[] = {{5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AGGCF_LUV[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AGGCF_LUV[] = {{7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AGGCF_LUV[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AGGCF_LUV[] = {{9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AGGCF_LUV[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AGGCF_LUV[] = {{11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AGGCF_LUV[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AGGCF_LUV[] = {{13, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AGGCF_LUV[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AGGCF_LUV[] = {{15, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AGGCF_LUV[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AGGCF_LUV[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AGGCF_LUV[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AGGCF_LUV[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AGGCF_LUV[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AGGCF_LUV[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AGGCF_LUV[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AGGCF_LUV[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AGGCF_LUV[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AGGCF_LUV[] = {{1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AGGCF_LUV[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AGGCF_LUV[] = {{3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AGGCF_LUV[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AGGCF_LUV[] = {{5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AGGCF_LUV[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AGGCF_LUV[] = {{7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AGGCF_LUV[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_AGGCF_LUV[] = {{9, 3, 2, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_AGGCF_LUV[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_AGGCF_LUV[] = {{11, 3, 2, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_AGGCF_LUV[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_AGGCF_LUV[] = {{13, 3, 2, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_AGGCF_LUV[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_AGGCF_LUV[] = {{15, 3, 2, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_AGGCF_LUV[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_AGGCF_LUV[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_AGGCF_LUV[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_AGGCF_LUV[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_AGGCF_LUV[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_AGGCF_LUV[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_AGGCF_LUV[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_AGGCF_LUV[] = {{1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_AGGCF_LUV[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_AGGCF_LUV[] = {{3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_AGGCF_LUV[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_AGGCF_LUV[] = {{5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_AGGCF_LUV[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_AGGCF_LUV[] = {{7, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_AGGCF_LUV[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_AGGCF_LUV[] = {{9, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_AGGCF_LUV[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_AGGCF_LUV[] = {{11, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_AGGCF_LUV[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_AGGCF_LUV[] = {{13, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_AGGCF_LUV[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_AGGCF_LUV[] = {{15, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_AGGCF_LUV[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_AGGCF_LUV[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_AGGCF_LUV[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_AGGCF_LUV[] = {{1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_AGGCF_LUV[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_AGGCF_LUV[] = {{3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_AGGCF_LUV[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_AGGCF_LUV[] = {{5, 5, 2, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_AGGCF_LUV[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_AGGCF_LUV[] = {{7, 5, 2, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_AGGCF_LUV[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_AGGCF_LUV[] = {{9, 5, 2, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_AGGCF_LUV[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_AGGCF_LUV[] = {{11, 5, 2, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_AGGCF_LUV[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_AGGCF_LUV[] = {{13, 5, 2, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_AGGCF_LUV[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_AGGCF_LUV[] = {{15, 5, 2, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_AGGCF_LUV[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_AGGCF_LUV[] = {{1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_AGGCF_LUV[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_AGGCF_LUV[] = {{3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_AGGCF_LUV[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_AGGCF_LUV[] = {{5, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_AGGCF_LUV[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_AGGCF_LUV[] = {{7, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_AGGCF_LUV[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_AGGCF_LUV[] = {{9, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_AGGCF_LUV[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_AGGCF_LUV[] = {{11, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_AGGCF_LUV[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_AGGCF_LUV[] = {{13, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_AGGCF_LUV[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_AGGCF_LUV[] = {{1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_AGGCF_LUV[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_AGGCF_LUV[] = {{3, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_AGGCF_LUV[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_AGGCF_LUV[] = {{5, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_AGGCF_LUV[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_AGGCF_LUV[] = {{7, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_AGGCF_LUV[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_AGGCF_LUV[] = {{9, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_AGGCF_LUV[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_AGGCF_LUV[] = {{1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_AGGCF_LUV[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_AGGCF_LUV[] = {{3, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_AGGCF_LUV[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_AGGCF_LUV[] = {{5, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_AGGCF_LUV[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_AGGCF_LUV[] = {{7, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_AGGCF_LUV[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_AGGCF_LUV[] = {{1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_AGGCF_LUV[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_AGGCF_LUV[] = {{3, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_AGGCF_LUV[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_AGGCF_LUV[] = {{5, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_AGGCF_LUV[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_AGGCF_LUV[] = {{7, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_AGGCF_LUV[] = {{1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_AGGCF_LUV[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_AGGCF_LUV[] = {{3, 14, 2, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_AGGCF_LUV[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_AGGCF_LUV[] = {{5, 14, 2, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_AGGCF_LUV[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_AGGCF_LUV[] = {{1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_AGGCF_LUV[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_AGGCF_LUV[] = {{3, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_AGGCF_LUV[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_AGGCF_LUV[] = {{5, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_AGGCF_LUV[] = {{1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_AGGCF_LUV[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_AGGCF_LUV[] = {{3, 18, 2, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_AGGCF_LUV[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_AGGCF_LUV[] = {{1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_AGGCF_LUV[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_AGGCF_LUV[] = {{3, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_AGGCF_LUV[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_AGGCF_LUV[] = {{1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_AGGCF_LUV[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_AGGCF_LUV[] = {{3, 22, 2, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_AGGCF_LUV[] = {{1, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_AGGCF_LUV[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_AGGCF_LUV[] = {{3, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_AGGCF_LUV[] = {{1, 26, 2, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_AGGCF_LUV[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_AGGCF_LUV[] = {{3, 26, 2, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_AGGCF_LUV[] = {{1, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_AGGCF_LUV[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_AGGCF_LUV[] = {{3, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_AGGCF_LUV[] = {{1, 30, 2, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_AGGCF_LUV[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_AGGCF_LUV[] = {{1, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_AGGCF_LUV[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_AGGCF_LUV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_AGGCF_LUV[] = {{0, 0}};

static acf_scenario gScenarioArray_AGGCF_LUV[] = {
{1, 1, 2148, 0, 0, gScenarioBufferData0_AGGCF_LUV, 80, gScenarioKernelData0_AGGCF_LUV, 4},
{2, 1, 2168, 0, 0, gScenarioBufferData1_AGGCF_LUV, 80, gScenarioKernelData1_AGGCF_LUV, 4},
{3, 1, 2188, 0, 0, gScenarioBufferData2_AGGCF_LUV, 80, gScenarioKernelData2_AGGCF_LUV, 4},
{4, 1, 2208, 0, 0, gScenarioBufferData3_AGGCF_LUV, 80, gScenarioKernelData3_AGGCF_LUV, 4},
{5, 1, 2228, 0, 0, gScenarioBufferData4_AGGCF_LUV, 80, gScenarioKernelData4_AGGCF_LUV, 4},
{6, 1, 2248, 0, 0, gScenarioBufferData5_AGGCF_LUV, 80, gScenarioKernelData5_AGGCF_LUV, 4},
{7, 1, 2268, 0, 0, gScenarioBufferData6_AGGCF_LUV, 80, gScenarioKernelData6_AGGCF_LUV, 4},
{8, 1, 2288, 0, 0, gScenarioBufferData7_AGGCF_LUV, 80, gScenarioKernelData7_AGGCF_LUV, 4},
{9, 1, 2308, 0, 0, gScenarioBufferData8_AGGCF_LUV, 80, gScenarioKernelData8_AGGCF_LUV, 4},
{10, 1, 2328, 0, 0, gScenarioBufferData9_AGGCF_LUV, 80, gScenarioKernelData9_AGGCF_LUV, 4},
{11, 1, 2348, 0, 0, gScenarioBufferData10_AGGCF_LUV, 80, gScenarioKernelData10_AGGCF_LUV, 4},
{12, 1, 2368, 0, 0, gScenarioBufferData11_AGGCF_LUV, 80, gScenarioKernelData11_AGGCF_LUV, 4},
{13, 1, 2388, 0, 0, gScenarioBufferData12_AGGCF_LUV, 80, gScenarioKernelData12_AGGCF_LUV, 4},
{14, 1, 2408, 0, 0, gScenarioBufferData13_AGGCF_LUV, 80, gScenarioKernelData13_AGGCF_LUV, 4},
{15, 1, 2428, 0, 0, gScenarioBufferData14_AGGCF_LUV, 80, gScenarioKernelData14_AGGCF_LUV, 4},
{16, 1, 2448, 0, 0, gScenarioBufferData15_AGGCF_LUV, 80, gScenarioKernelData15_AGGCF_LUV, 4},
{18, 1, 2488, 0, 0, gScenarioBufferData16_AGGCF_LUV, 80, gScenarioKernelData16_AGGCF_LUV, 4},
{20, 1, 2528, 0, 0, gScenarioBufferData17_AGGCF_LUV, 80, gScenarioKernelData17_AGGCF_LUV, 4},
{22, 1, 2568, 0, 0, gScenarioBufferData18_AGGCF_LUV, 80, gScenarioKernelData18_AGGCF_LUV, 4},
{24, 1, 2608, 0, 0, gScenarioBufferData19_AGGCF_LUV, 80, gScenarioKernelData19_AGGCF_LUV, 4},
{26, 1, 2648, 0, 0, gScenarioBufferData20_AGGCF_LUV, 80, gScenarioKernelData20_AGGCF_LUV, 4},
{28, 1, 2688, 0, 0, gScenarioBufferData21_AGGCF_LUV, 80, gScenarioKernelData21_AGGCF_LUV, 4},
{30, 1, 2728, 0, 0, gScenarioBufferData22_AGGCF_LUV, 80, gScenarioKernelData22_AGGCF_LUV, 4},
{32, 1, 2768, 0, 0, gScenarioBufferData23_AGGCF_LUV, 80, gScenarioKernelData23_AGGCF_LUV, 4},
{1, 2, 2168, 0, 0, gScenarioBufferData24_AGGCF_LUV, 80, gScenarioKernelData24_AGGCF_LUV, 4},
{2, 2, 2208, 0, 0, gScenarioBufferData25_AGGCF_LUV, 80, gScenarioKernelData25_AGGCF_LUV, 4},
{3, 2, 2248, 0, 0, gScenarioBufferData26_AGGCF_LUV, 80, gScenarioKernelData26_AGGCF_LUV, 4},
{4, 2, 2288, 0, 0, gScenarioBufferData27_AGGCF_LUV, 80, gScenarioKernelData27_AGGCF_LUV, 4},
{5, 2, 2328, 0, 0, gScenarioBufferData28_AGGCF_LUV, 80, gScenarioKernelData28_AGGCF_LUV, 4},
{6, 2, 2368, 0, 0, gScenarioBufferData29_AGGCF_LUV, 80, gScenarioKernelData29_AGGCF_LUV, 4},
{7, 2, 2408, 0, 0, gScenarioBufferData30_AGGCF_LUV, 80, gScenarioKernelData30_AGGCF_LUV, 4},
{8, 2, 2448, 0, 0, gScenarioBufferData31_AGGCF_LUV, 80, gScenarioKernelData31_AGGCF_LUV, 4},
{9, 2, 2488, 0, 0, gScenarioBufferData32_AGGCF_LUV, 80, gScenarioKernelData32_AGGCF_LUV, 4},
{10, 2, 2528, 0, 0, gScenarioBufferData33_AGGCF_LUV, 80, gScenarioKernelData33_AGGCF_LUV, 4},
{11, 2, 2568, 0, 0, gScenarioBufferData34_AGGCF_LUV, 80, gScenarioKernelData34_AGGCF_LUV, 4},
{12, 2, 2608, 0, 0, gScenarioBufferData35_AGGCF_LUV, 80, gScenarioKernelData35_AGGCF_LUV, 4},
{13, 2, 2648, 0, 0, gScenarioBufferData36_AGGCF_LUV, 80, gScenarioKernelData36_AGGCF_LUV, 4},
{14, 2, 2688, 0, 0, gScenarioBufferData37_AGGCF_LUV, 80, gScenarioKernelData37_AGGCF_LUV, 4},
{15, 2, 2728, 0, 0, gScenarioBufferData38_AGGCF_LUV, 80, gScenarioKernelData38_AGGCF_LUV, 4},
{16, 2, 2768, 0, 0, gScenarioBufferData39_AGGCF_LUV, 80, gScenarioKernelData39_AGGCF_LUV, 4},
{18, 2, 2848, 0, 0, gScenarioBufferData40_AGGCF_LUV, 80, gScenarioKernelData40_AGGCF_LUV, 4},
{20, 2, 2928, 0, 0, gScenarioBufferData41_AGGCF_LUV, 80, gScenarioKernelData41_AGGCF_LUV, 4},
{22, 2, 3008, 0, 0, gScenarioBufferData42_AGGCF_LUV, 80, gScenarioKernelData42_AGGCF_LUV, 4},
{24, 2, 3088, 0, 0, gScenarioBufferData43_AGGCF_LUV, 80, gScenarioKernelData43_AGGCF_LUV, 4},
{26, 2, 3168, 0, 0, gScenarioBufferData44_AGGCF_LUV, 80, gScenarioKernelData44_AGGCF_LUV, 4},
{28, 2, 3248, 0, 0, gScenarioBufferData45_AGGCF_LUV, 80, gScenarioKernelData45_AGGCF_LUV, 4},
{30, 2, 3328, 0, 0, gScenarioBufferData46_AGGCF_LUV, 80, gScenarioKernelData46_AGGCF_LUV, 4},
{32, 2, 3408, 0, 0, gScenarioBufferData47_AGGCF_LUV, 80, gScenarioKernelData47_AGGCF_LUV, 4},
{1, 3, 2188, 0, 0, gScenarioBufferData48_AGGCF_LUV, 80, gScenarioKernelData48_AGGCF_LUV, 4},
{2, 3, 2248, 0, 0, gScenarioBufferData49_AGGCF_LUV, 80, gScenarioKernelData49_AGGCF_LUV, 4},
{3, 3, 2308, 0, 0, gScenarioBufferData50_AGGCF_LUV, 80, gScenarioKernelData50_AGGCF_LUV, 4},
{4, 3, 2368, 0, 0, gScenarioBufferData51_AGGCF_LUV, 80, gScenarioKernelData51_AGGCF_LUV, 4},
{5, 3, 2428, 0, 0, gScenarioBufferData52_AGGCF_LUV, 80, gScenarioKernelData52_AGGCF_LUV, 4},
{6, 3, 2488, 0, 0, gScenarioBufferData53_AGGCF_LUV, 80, gScenarioKernelData53_AGGCF_LUV, 4},
{7, 3, 2548, 0, 0, gScenarioBufferData54_AGGCF_LUV, 80, gScenarioKernelData54_AGGCF_LUV, 4},
{8, 3, 2608, 0, 0, gScenarioBufferData55_AGGCF_LUV, 80, gScenarioKernelData55_AGGCF_LUV, 4},
{9, 3, 2668, 0, 0, gScenarioBufferData56_AGGCF_LUV, 80, gScenarioKernelData56_AGGCF_LUV, 4},
{10, 3, 2728, 0, 0, gScenarioBufferData57_AGGCF_LUV, 80, gScenarioKernelData57_AGGCF_LUV, 4},
{11, 3, 2788, 0, 0, gScenarioBufferData58_AGGCF_LUV, 80, gScenarioKernelData58_AGGCF_LUV, 4},
{12, 3, 2848, 0, 0, gScenarioBufferData59_AGGCF_LUV, 80, gScenarioKernelData59_AGGCF_LUV, 4},
{13, 3, 2908, 0, 0, gScenarioBufferData60_AGGCF_LUV, 80, gScenarioKernelData60_AGGCF_LUV, 4},
{14, 3, 2968, 0, 0, gScenarioBufferData61_AGGCF_LUV, 80, gScenarioKernelData61_AGGCF_LUV, 4},
{15, 3, 3028, 0, 0, gScenarioBufferData62_AGGCF_LUV, 80, gScenarioKernelData62_AGGCF_LUV, 4},
{16, 3, 3088, 0, 0, gScenarioBufferData63_AGGCF_LUV, 80, gScenarioKernelData63_AGGCF_LUV, 4},
{18, 3, 3208, 0, 0, gScenarioBufferData64_AGGCF_LUV, 80, gScenarioKernelData64_AGGCF_LUV, 4},
{20, 3, 3328, 0, 0, gScenarioBufferData65_AGGCF_LUV, 80, gScenarioKernelData65_AGGCF_LUV, 4},
{22, 3, 3448, 0, 0, gScenarioBufferData66_AGGCF_LUV, 80, gScenarioKernelData66_AGGCF_LUV, 4},
{24, 3, 3568, 0, 0, gScenarioBufferData67_AGGCF_LUV, 80, gScenarioKernelData67_AGGCF_LUV, 4},
{26, 3, 3688, 0, 0, gScenarioBufferData68_AGGCF_LUV, 80, gScenarioKernelData68_AGGCF_LUV, 4},
{28, 3, 3808, 0, 0, gScenarioBufferData69_AGGCF_LUV, 80, gScenarioKernelData69_AGGCF_LUV, 4},
{1, 4, 2208, 0, 0, gScenarioBufferData70_AGGCF_LUV, 80, gScenarioKernelData70_AGGCF_LUV, 4},
{2, 4, 2288, 0, 0, gScenarioBufferData71_AGGCF_LUV, 80, gScenarioKernelData71_AGGCF_LUV, 4},
{3, 4, 2368, 0, 0, gScenarioBufferData72_AGGCF_LUV, 80, gScenarioKernelData72_AGGCF_LUV, 4},
{4, 4, 2448, 0, 0, gScenarioBufferData73_AGGCF_LUV, 80, gScenarioKernelData73_AGGCF_LUV, 4},
{5, 4, 2528, 0, 0, gScenarioBufferData74_AGGCF_LUV, 80, gScenarioKernelData74_AGGCF_LUV, 4},
{6, 4, 2608, 0, 0, gScenarioBufferData75_AGGCF_LUV, 80, gScenarioKernelData75_AGGCF_LUV, 4},
{7, 4, 2688, 0, 0, gScenarioBufferData76_AGGCF_LUV, 80, gScenarioKernelData76_AGGCF_LUV, 4},
{8, 4, 2768, 0, 0, gScenarioBufferData77_AGGCF_LUV, 80, gScenarioKernelData77_AGGCF_LUV, 4},
{9, 4, 2848, 0, 0, gScenarioBufferData78_AGGCF_LUV, 80, gScenarioKernelData78_AGGCF_LUV, 4},
{10, 4, 2928, 0, 0, gScenarioBufferData79_AGGCF_LUV, 80, gScenarioKernelData79_AGGCF_LUV, 4},
{11, 4, 3008, 0, 0, gScenarioBufferData80_AGGCF_LUV, 80, gScenarioKernelData80_AGGCF_LUV, 4},
{12, 4, 3088, 0, 0, gScenarioBufferData81_AGGCF_LUV, 80, gScenarioKernelData81_AGGCF_LUV, 4},
{13, 4, 3168, 0, 0, gScenarioBufferData82_AGGCF_LUV, 80, gScenarioKernelData82_AGGCF_LUV, 4},
{14, 4, 3248, 0, 0, gScenarioBufferData83_AGGCF_LUV, 80, gScenarioKernelData83_AGGCF_LUV, 4},
{15, 4, 3328, 0, 0, gScenarioBufferData84_AGGCF_LUV, 80, gScenarioKernelData84_AGGCF_LUV, 4},
{16, 4, 3408, 0, 0, gScenarioBufferData85_AGGCF_LUV, 80, gScenarioKernelData85_AGGCF_LUV, 4},
{18, 4, 3568, 0, 0, gScenarioBufferData86_AGGCF_LUV, 80, gScenarioKernelData86_AGGCF_LUV, 4},
{20, 4, 3728, 0, 0, gScenarioBufferData87_AGGCF_LUV, 80, gScenarioKernelData87_AGGCF_LUV, 4},
{1, 5, 2228, 0, 0, gScenarioBufferData88_AGGCF_LUV, 80, gScenarioKernelData88_AGGCF_LUV, 4},
{2, 5, 2328, 0, 0, gScenarioBufferData89_AGGCF_LUV, 80, gScenarioKernelData89_AGGCF_LUV, 4},
{3, 5, 2428, 0, 0, gScenarioBufferData90_AGGCF_LUV, 80, gScenarioKernelData90_AGGCF_LUV, 4},
{4, 5, 2528, 0, 0, gScenarioBufferData91_AGGCF_LUV, 80, gScenarioKernelData91_AGGCF_LUV, 4},
{5, 5, 2628, 0, 0, gScenarioBufferData92_AGGCF_LUV, 80, gScenarioKernelData92_AGGCF_LUV, 4},
{6, 5, 2728, 0, 0, gScenarioBufferData93_AGGCF_LUV, 80, gScenarioKernelData93_AGGCF_LUV, 4},
{7, 5, 2828, 0, 0, gScenarioBufferData94_AGGCF_LUV, 80, gScenarioKernelData94_AGGCF_LUV, 4},
{8, 5, 2928, 0, 0, gScenarioBufferData95_AGGCF_LUV, 80, gScenarioKernelData95_AGGCF_LUV, 4},
{9, 5, 3028, 0, 0, gScenarioBufferData96_AGGCF_LUV, 80, gScenarioKernelData96_AGGCF_LUV, 4},
{10, 5, 3128, 0, 0, gScenarioBufferData97_AGGCF_LUV, 80, gScenarioKernelData97_AGGCF_LUV, 4},
{11, 5, 3228, 0, 0, gScenarioBufferData98_AGGCF_LUV, 80, gScenarioKernelData98_AGGCF_LUV, 4},
{12, 5, 3328, 0, 0, gScenarioBufferData99_AGGCF_LUV, 80, gScenarioKernelData99_AGGCF_LUV, 4},
{13, 5, 3428, 0, 0, gScenarioBufferData100_AGGCF_LUV, 80, gScenarioKernelData100_AGGCF_LUV, 4},
{14, 5, 3528, 0, 0, gScenarioBufferData101_AGGCF_LUV, 80, gScenarioKernelData101_AGGCF_LUV, 4},
{15, 5, 3628, 0, 0, gScenarioBufferData102_AGGCF_LUV, 80, gScenarioKernelData102_AGGCF_LUV, 4},
{16, 5, 3728, 0, 0, gScenarioBufferData103_AGGCF_LUV, 80, gScenarioKernelData103_AGGCF_LUV, 4},
{1, 6, 2248, 0, 0, gScenarioBufferData104_AGGCF_LUV, 80, gScenarioKernelData104_AGGCF_LUV, 4},
{2, 6, 2368, 0, 0, gScenarioBufferData105_AGGCF_LUV, 80, gScenarioKernelData105_AGGCF_LUV, 4},
{3, 6, 2488, 0, 0, gScenarioBufferData106_AGGCF_LUV, 80, gScenarioKernelData106_AGGCF_LUV, 4},
{4, 6, 2608, 0, 0, gScenarioBufferData107_AGGCF_LUV, 80, gScenarioKernelData107_AGGCF_LUV, 4},
{5, 6, 2728, 0, 0, gScenarioBufferData108_AGGCF_LUV, 80, gScenarioKernelData108_AGGCF_LUV, 4},
{6, 6, 2848, 0, 0, gScenarioBufferData109_AGGCF_LUV, 80, gScenarioKernelData109_AGGCF_LUV, 4},
{7, 6, 2968, 0, 0, gScenarioBufferData110_AGGCF_LUV, 80, gScenarioKernelData110_AGGCF_LUV, 4},
{8, 6, 3088, 0, 0, gScenarioBufferData111_AGGCF_LUV, 80, gScenarioKernelData111_AGGCF_LUV, 4},
{9, 6, 3208, 0, 0, gScenarioBufferData112_AGGCF_LUV, 80, gScenarioKernelData112_AGGCF_LUV, 4},
{10, 6, 3328, 0, 0, gScenarioBufferData113_AGGCF_LUV, 80, gScenarioKernelData113_AGGCF_LUV, 4},
{11, 6, 3448, 0, 0, gScenarioBufferData114_AGGCF_LUV, 80, gScenarioKernelData114_AGGCF_LUV, 4},
{12, 6, 3568, 0, 0, gScenarioBufferData115_AGGCF_LUV, 80, gScenarioKernelData115_AGGCF_LUV, 4},
{13, 6, 3688, 0, 0, gScenarioBufferData116_AGGCF_LUV, 80, gScenarioKernelData116_AGGCF_LUV, 4},
{14, 6, 3808, 0, 0, gScenarioBufferData117_AGGCF_LUV, 80, gScenarioKernelData117_AGGCF_LUV, 4},
{1, 8, 2288, 0, 0, gScenarioBufferData118_AGGCF_LUV, 80, gScenarioKernelData118_AGGCF_LUV, 4},
{2, 8, 2448, 0, 0, gScenarioBufferData119_AGGCF_LUV, 80, gScenarioKernelData119_AGGCF_LUV, 4},
{3, 8, 2608, 0, 0, gScenarioBufferData120_AGGCF_LUV, 80, gScenarioKernelData120_AGGCF_LUV, 4},
{4, 8, 2768, 0, 0, gScenarioBufferData121_AGGCF_LUV, 80, gScenarioKernelData121_AGGCF_LUV, 4},
{5, 8, 2928, 0, 0, gScenarioBufferData122_AGGCF_LUV, 80, gScenarioKernelData122_AGGCF_LUV, 4},
{6, 8, 3088, 0, 0, gScenarioBufferData123_AGGCF_LUV, 80, gScenarioKernelData123_AGGCF_LUV, 4},
{7, 8, 3248, 0, 0, gScenarioBufferData124_AGGCF_LUV, 80, gScenarioKernelData124_AGGCF_LUV, 4},
{8, 8, 3408, 0, 0, gScenarioBufferData125_AGGCF_LUV, 80, gScenarioKernelData125_AGGCF_LUV, 4},
{9, 8, 3568, 0, 0, gScenarioBufferData126_AGGCF_LUV, 80, gScenarioKernelData126_AGGCF_LUV, 4},
{10, 8, 3728, 0, 0, gScenarioBufferData127_AGGCF_LUV, 80, gScenarioKernelData127_AGGCF_LUV, 4},
{1, 10, 2328, 0, 0, gScenarioBufferData128_AGGCF_LUV, 80, gScenarioKernelData128_AGGCF_LUV, 4},
{2, 10, 2528, 0, 0, gScenarioBufferData129_AGGCF_LUV, 80, gScenarioKernelData129_AGGCF_LUV, 4},
{3, 10, 2728, 0, 0, gScenarioBufferData130_AGGCF_LUV, 80, gScenarioKernelData130_AGGCF_LUV, 4},
{4, 10, 2928, 0, 0, gScenarioBufferData131_AGGCF_LUV, 80, gScenarioKernelData131_AGGCF_LUV, 4},
{5, 10, 3128, 0, 0, gScenarioBufferData132_AGGCF_LUV, 80, gScenarioKernelData132_AGGCF_LUV, 4},
{6, 10, 3328, 0, 0, gScenarioBufferData133_AGGCF_LUV, 80, gScenarioKernelData133_AGGCF_LUV, 4},
{7, 10, 3528, 0, 0, gScenarioBufferData134_AGGCF_LUV, 80, gScenarioKernelData134_AGGCF_LUV, 4},
{8, 10, 3728, 0, 0, gScenarioBufferData135_AGGCF_LUV, 80, gScenarioKernelData135_AGGCF_LUV, 4},
{1, 12, 2368, 0, 0, gScenarioBufferData136_AGGCF_LUV, 80, gScenarioKernelData136_AGGCF_LUV, 4},
{2, 12, 2608, 0, 0, gScenarioBufferData137_AGGCF_LUV, 80, gScenarioKernelData137_AGGCF_LUV, 4},
{3, 12, 2848, 0, 0, gScenarioBufferData138_AGGCF_LUV, 80, gScenarioKernelData138_AGGCF_LUV, 4},
{4, 12, 3088, 0, 0, gScenarioBufferData139_AGGCF_LUV, 80, gScenarioKernelData139_AGGCF_LUV, 4},
{5, 12, 3328, 0, 0, gScenarioBufferData140_AGGCF_LUV, 80, gScenarioKernelData140_AGGCF_LUV, 4},
{6, 12, 3568, 0, 0, gScenarioBufferData141_AGGCF_LUV, 80, gScenarioKernelData141_AGGCF_LUV, 4},
{7, 12, 3808, 0, 0, gScenarioBufferData142_AGGCF_LUV, 80, gScenarioKernelData142_AGGCF_LUV, 4},
{1, 14, 2408, 0, 0, gScenarioBufferData143_AGGCF_LUV, 80, gScenarioKernelData143_AGGCF_LUV, 4},
{2, 14, 2688, 0, 0, gScenarioBufferData144_AGGCF_LUV, 80, gScenarioKernelData144_AGGCF_LUV, 4},
{3, 14, 2968, 0, 0, gScenarioBufferData145_AGGCF_LUV, 80, gScenarioKernelData145_AGGCF_LUV, 4},
{4, 14, 3248, 0, 0, gScenarioBufferData146_AGGCF_LUV, 80, gScenarioKernelData146_AGGCF_LUV, 4},
{5, 14, 3528, 0, 0, gScenarioBufferData147_AGGCF_LUV, 80, gScenarioKernelData147_AGGCF_LUV, 4},
{6, 14, 3808, 0, 0, gScenarioBufferData148_AGGCF_LUV, 80, gScenarioKernelData148_AGGCF_LUV, 4},
{1, 16, 2448, 0, 0, gScenarioBufferData149_AGGCF_LUV, 80, gScenarioKernelData149_AGGCF_LUV, 4},
{2, 16, 2768, 0, 0, gScenarioBufferData150_AGGCF_LUV, 80, gScenarioKernelData150_AGGCF_LUV, 4},
{3, 16, 3088, 0, 0, gScenarioBufferData151_AGGCF_LUV, 80, gScenarioKernelData151_AGGCF_LUV, 4},
{4, 16, 3408, 0, 0, gScenarioBufferData152_AGGCF_LUV, 80, gScenarioKernelData152_AGGCF_LUV, 4},
{5, 16, 3728, 0, 0, gScenarioBufferData153_AGGCF_LUV, 80, gScenarioKernelData153_AGGCF_LUV, 4},
{1, 18, 2488, 0, 0, gScenarioBufferData154_AGGCF_LUV, 80, gScenarioKernelData154_AGGCF_LUV, 4},
{2, 18, 2848, 0, 0, gScenarioBufferData155_AGGCF_LUV, 80, gScenarioKernelData155_AGGCF_LUV, 4},
{3, 18, 3208, 0, 0, gScenarioBufferData156_AGGCF_LUV, 80, gScenarioKernelData156_AGGCF_LUV, 4},
{4, 18, 3568, 0, 0, gScenarioBufferData157_AGGCF_LUV, 80, gScenarioKernelData157_AGGCF_LUV, 4},
{1, 20, 2528, 0, 0, gScenarioBufferData158_AGGCF_LUV, 80, gScenarioKernelData158_AGGCF_LUV, 4},
{2, 20, 2928, 0, 0, gScenarioBufferData159_AGGCF_LUV, 80, gScenarioKernelData159_AGGCF_LUV, 4},
{3, 20, 3328, 0, 0, gScenarioBufferData160_AGGCF_LUV, 80, gScenarioKernelData160_AGGCF_LUV, 4},
{4, 20, 3728, 0, 0, gScenarioBufferData161_AGGCF_LUV, 80, gScenarioKernelData161_AGGCF_LUV, 4},
{1, 22, 2568, 0, 0, gScenarioBufferData162_AGGCF_LUV, 80, gScenarioKernelData162_AGGCF_LUV, 4},
{2, 22, 3008, 0, 0, gScenarioBufferData163_AGGCF_LUV, 80, gScenarioKernelData163_AGGCF_LUV, 4},
{3, 22, 3448, 0, 0, gScenarioBufferData164_AGGCF_LUV, 80, gScenarioKernelData164_AGGCF_LUV, 4},
{1, 24, 2608, 0, 0, gScenarioBufferData165_AGGCF_LUV, 80, gScenarioKernelData165_AGGCF_LUV, 4},
{2, 24, 3088, 0, 0, gScenarioBufferData166_AGGCF_LUV, 80, gScenarioKernelData166_AGGCF_LUV, 4},
{3, 24, 3568, 0, 0, gScenarioBufferData167_AGGCF_LUV, 80, gScenarioKernelData167_AGGCF_LUV, 4},
{1, 26, 2648, 0, 0, gScenarioBufferData168_AGGCF_LUV, 80, gScenarioKernelData168_AGGCF_LUV, 4},
{2, 26, 3168, 0, 0, gScenarioBufferData169_AGGCF_LUV, 80, gScenarioKernelData169_AGGCF_LUV, 4},
{3, 26, 3688, 0, 0, gScenarioBufferData170_AGGCF_LUV, 80, gScenarioKernelData170_AGGCF_LUV, 4},
{1, 28, 2688, 0, 0, gScenarioBufferData171_AGGCF_LUV, 80, gScenarioKernelData171_AGGCF_LUV, 4},
{2, 28, 3248, 0, 0, gScenarioBufferData172_AGGCF_LUV, 80, gScenarioKernelData172_AGGCF_LUV, 4},
{3, 28, 3808, 0, 0, gScenarioBufferData173_AGGCF_LUV, 80, gScenarioKernelData173_AGGCF_LUV, 4},
{1, 30, 2728, 0, 0, gScenarioBufferData174_AGGCF_LUV, 80, gScenarioKernelData174_AGGCF_LUV, 4},
{2, 30, 3328, 0, 0, gScenarioBufferData175_AGGCF_LUV, 80, gScenarioKernelData175_AGGCF_LUV, 4},
{1, 32, 2768, 0, 0, gScenarioBufferData176_AGGCF_LUV, 80, gScenarioKernelData176_AGGCF_LUV, 4},
{2, 32, 3408, 0, 0, gScenarioBufferData177_AGGCF_LUV, 80, gScenarioKernelData177_AGGCF_LUV, 4}
};

static acf_scenario_list gScenarioList_AGGCF_LUV = {
178, //number of scenarios
gScenarioArray_AGGCF_LUV};
//**************************************************************

class AGGCF_LUV : public ACF_Process_APU
{

public:
   AGGCF_LUV(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_LUV");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_LUV",
                                        AGGCF_LUV_LOAD_SEGMENTS,
                                        AGGCF_LUV_LOAD_PMEM, AGGCF_LUV_LOAD_PMEM_SIZE,
                                        AGGCF_LUV_LOAD_DMEM, AGGCF_LUV_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_LUV_APEX_LOG_BUFFER, AGGCF_LUV_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUTL", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUTU", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUTV", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_LUV);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_LUV

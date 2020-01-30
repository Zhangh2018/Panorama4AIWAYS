#ifndef _ACF_PROCESS_APU_AGGCF_SCALE_DOWN2
#define _ACF_PROCESS_APU_AGGCF_SCALE_DOWN2

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_SCALE_DOWN2_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_SCALE_DOWN2[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_SCALE_DOWN2[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_SCALE_DOWN2[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 1, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_SCALE_DOWN2[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_SCALE_DOWN2[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 1, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_SCALE_DOWN2[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_SCALE_DOWN2[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 1, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_SCALE_DOWN2[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_SCALE_DOWN2[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 1, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_SCALE_DOWN2[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_SCALE_DOWN2[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 1, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_SCALE_DOWN2[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_SCALE_DOWN2[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 1, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_SCALE_DOWN2[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 1, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_SCALE_DOWN2[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 1, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_SCALE_DOWN2[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_SCALE_DOWN2[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 1, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_SCALE_DOWN2[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 1, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_SCALE_DOWN2[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 1, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_SCALE_DOWN2[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 1, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_SCALE_DOWN2[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 1, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_SCALE_DOWN2[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 1, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_SCALE_DOWN2[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 1, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_SCALE_DOWN2[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_SCALE_DOWN2[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 1, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_SCALE_DOWN2[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_SCALE_DOWN2[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 1, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_SCALE_DOWN2[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AGGCF_SCALE_DOWN2[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 1, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AGGCF_SCALE_DOWN2[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AGGCF_SCALE_DOWN2[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 1, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AGGCF_SCALE_DOWN2[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AGGCF_SCALE_DOWN2[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 1, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AGGCF_SCALE_DOWN2[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 1, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AGGCF_SCALE_DOWN2[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 1, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AGGCF_SCALE_DOWN2[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 1, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AGGCF_SCALE_DOWN2[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 1, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AGGCF_SCALE_DOWN2[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 1, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AGGCF_SCALE_DOWN2[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 1, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AGGCF_SCALE_DOWN2[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AGGCF_SCALE_DOWN2[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 1, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AGGCF_SCALE_DOWN2[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 1, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AGGCF_SCALE_DOWN2[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 1, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AGGCF_SCALE_DOWN2[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 1, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AGGCF_SCALE_DOWN2[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 1, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AGGCF_SCALE_DOWN2[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 1, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AGGCF_SCALE_DOWN2[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 1, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AGGCF_SCALE_DOWN2[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 1, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AGGCF_SCALE_DOWN2[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 1, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AGGCF_SCALE_DOWN2[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AGGCF_SCALE_DOWN2[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 1, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AGGCF_SCALE_DOWN2[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AGGCF_SCALE_DOWN2[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 1, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AGGCF_SCALE_DOWN2[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 1, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AGGCF_SCALE_DOWN2[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 1, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AGGCF_SCALE_DOWN2[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_AGGCF_SCALE_DOWN2[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 1, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_AGGCF_SCALE_DOWN2[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 1, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_AGGCF_SCALE_DOWN2[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 1, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_AGGCF_SCALE_DOWN2[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 1, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_AGGCF_SCALE_DOWN2[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 1, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_AGGCF_SCALE_DOWN2[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 1, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_AGGCF_SCALE_DOWN2[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {15, 6, 1, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_AGGCF_SCALE_DOWN2[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_AGGCF_SCALE_DOWN2[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 1, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_AGGCF_SCALE_DOWN2[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 1, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_AGGCF_SCALE_DOWN2[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 1, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_AGGCF_SCALE_DOWN2[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 1, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_AGGCF_SCALE_DOWN2[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 1, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_AGGCF_SCALE_DOWN2[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 1, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_AGGCF_SCALE_DOWN2[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 1, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_AGGCF_SCALE_DOWN2[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 1, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_AGGCF_SCALE_DOWN2[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 1, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_AGGCF_SCALE_DOWN2[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_AGGCF_SCALE_DOWN2[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 1, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_AGGCF_SCALE_DOWN2[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_AGGCF_SCALE_DOWN2[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 1, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_AGGCF_SCALE_DOWN2[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 1, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_AGGCF_SCALE_DOWN2[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 1, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_AGGCF_SCALE_DOWN2[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_AGGCF_SCALE_DOWN2[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 1, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_AGGCF_SCALE_DOWN2[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 1, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_AGGCF_SCALE_DOWN2[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {11, 8, 1, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_AGGCF_SCALE_DOWN2[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_AGGCF_SCALE_DOWN2[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {13, 8, 1, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_AGGCF_SCALE_DOWN2[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 1, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_AGGCF_SCALE_DOWN2[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {15, 8, 1, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_AGGCF_SCALE_DOWN2[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_AGGCF_SCALE_DOWN2[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 1, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_AGGCF_SCALE_DOWN2[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 1, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_AGGCF_SCALE_DOWN2[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 1, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_AGGCF_SCALE_DOWN2[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 1, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_AGGCF_SCALE_DOWN2[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 1, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_AGGCF_SCALE_DOWN2[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 1, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_AGGCF_SCALE_DOWN2[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 1, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_AGGCF_SCALE_DOWN2[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 1, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_AGGCF_SCALE_DOWN2[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 1, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_AGGCF_SCALE_DOWN2[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_AGGCF_SCALE_DOWN2[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 1, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_AGGCF_SCALE_DOWN2[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 1, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_AGGCF_SCALE_DOWN2[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 1, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_AGGCF_SCALE_DOWN2[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 1, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_AGGCF_SCALE_DOWN2[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 1, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_AGGCF_SCALE_DOWN2[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_AGGCF_SCALE_DOWN2[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {9, 10, 1, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_AGGCF_SCALE_DOWN2[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 1, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_AGGCF_SCALE_DOWN2[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {11, 10, 1, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_AGGCF_SCALE_DOWN2[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 1, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_AGGCF_SCALE_DOWN2[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {13, 10, 1, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_AGGCF_SCALE_DOWN2[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 1, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_AGGCF_SCALE_DOWN2[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {15, 10, 1, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_AGGCF_SCALE_DOWN2[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 1, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_AGGCF_SCALE_DOWN2[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 1, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_AGGCF_SCALE_DOWN2[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 1, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_AGGCF_SCALE_DOWN2[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 1, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_AGGCF_SCALE_DOWN2[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 1, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_AGGCF_SCALE_DOWN2[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 1, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_AGGCF_SCALE_DOWN2[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 1, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_AGGCF_SCALE_DOWN2[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 1, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_AGGCF_SCALE_DOWN2[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 1, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_AGGCF_SCALE_DOWN2[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 1, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_AGGCF_SCALE_DOWN2[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_AGGCF_SCALE_DOWN2[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 1, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_AGGCF_SCALE_DOWN2[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 1, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_AGGCF_SCALE_DOWN2[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 1, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_AGGCF_SCALE_DOWN2[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 1, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_AGGCF_SCALE_DOWN2[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 1, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_AGGCF_SCALE_DOWN2[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_AGGCF_SCALE_DOWN2[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {9, 12, 1, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_AGGCF_SCALE_DOWN2[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 1, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_AGGCF_SCALE_DOWN2[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {11, 12, 1, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_AGGCF_SCALE_DOWN2[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 1, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_AGGCF_SCALE_DOWN2[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {13, 12, 1, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_AGGCF_SCALE_DOWN2[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 1, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_AGGCF_SCALE_DOWN2[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {15, 12, 1, 0, {0, 0, 0, 0}}, {15, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_AGGCF_SCALE_DOWN2[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 1, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_AGGCF_SCALE_DOWN2[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 1, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_AGGCF_SCALE_DOWN2[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 1, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_AGGCF_SCALE_DOWN2[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 1, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_AGGCF_SCALE_DOWN2[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 1, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_AGGCF_SCALE_DOWN2[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 1, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_AGGCF_SCALE_DOWN2[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 1, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_AGGCF_SCALE_DOWN2[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 1, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_AGGCF_SCALE_DOWN2[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 1, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_AGGCF_SCALE_DOWN2[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_AGGCF_SCALE_DOWN2[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 1, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_AGGCF_SCALE_DOWN2[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 1, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_AGGCF_SCALE_DOWN2[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {7, 16, 1, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_AGGCF_SCALE_DOWN2[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_AGGCF_SCALE_DOWN2[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {9, 16, 1, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_AGGCF_SCALE_DOWN2[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 1, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_AGGCF_SCALE_DOWN2[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {11, 16, 1, 0, {0, 0, 0, 0}}, {11, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_AGGCF_SCALE_DOWN2[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 1, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_AGGCF_SCALE_DOWN2[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {13, 16, 1, 0, {0, 0, 0, 0}}, {13, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_AGGCF_SCALE_DOWN2[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 1, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_AGGCF_SCALE_DOWN2[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {15, 16, 1, 0, {0, 0, 0, 0}}, {15, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_AGGCF_SCALE_DOWN2[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 1, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_AGGCF_SCALE_DOWN2[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 1, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_AGGCF_SCALE_DOWN2[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 1, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_AGGCF_SCALE_DOWN2[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 1, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_AGGCF_SCALE_DOWN2[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 1, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_AGGCF_SCALE_DOWN2[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 1, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_AGGCF_SCALE_DOWN2[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 1, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_AGGCF_SCALE_DOWN2[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {5, 20, 1, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_AGGCF_SCALE_DOWN2[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 1, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_AGGCF_SCALE_DOWN2[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {7, 20, 1, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_AGGCF_SCALE_DOWN2[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 1, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_AGGCF_SCALE_DOWN2[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {9, 20, 1, 0, {0, 0, 0, 0}}, {9, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_AGGCF_SCALE_DOWN2[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 1, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_AGGCF_SCALE_DOWN2[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {11, 20, 1, 0, {0, 0, 0, 0}}, {11, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_AGGCF_SCALE_DOWN2[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 1, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_AGGCF_SCALE_DOWN2[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {13, 20, 1, 0, {0, 0, 0, 0}}, {13, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_AGGCF_SCALE_DOWN2[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 1, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_AGGCF_SCALE_DOWN2[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {15, 20, 1, 0, {0, 0, 0, 0}}, {15, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_AGGCF_SCALE_DOWN2[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 1, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_AGGCF_SCALE_DOWN2[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 1, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_AGGCF_SCALE_DOWN2[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 1, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_AGGCF_SCALE_DOWN2[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 1, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_AGGCF_SCALE_DOWN2[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_AGGCF_SCALE_DOWN2[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {5, 24, 1, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_AGGCF_SCALE_DOWN2[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 1, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_AGGCF_SCALE_DOWN2[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {7, 24, 1, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_AGGCF_SCALE_DOWN2[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_AGGCF_SCALE_DOWN2[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {9, 24, 1, 0, {0, 0, 0, 0}}, {9, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_AGGCF_SCALE_DOWN2[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 1, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_AGGCF_SCALE_DOWN2[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {11, 24, 1, 0, {0, 0, 0, 0}}, {11, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_AGGCF_SCALE_DOWN2[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 1, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_AGGCF_SCALE_DOWN2[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {13, 24, 1, 0, {0, 0, 0, 0}}, {13, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_AGGCF_SCALE_DOWN2[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 1, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_AGGCF_SCALE_DOWN2[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 1, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_AGGCF_SCALE_DOWN2[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 1, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_AGGCF_SCALE_DOWN2[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 1, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_AGGCF_SCALE_DOWN2[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {5, 28, 1, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_AGGCF_SCALE_DOWN2[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 1, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_AGGCF_SCALE_DOWN2[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {7, 28, 1, 0, {0, 0, 0, 0}}, {7, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_AGGCF_SCALE_DOWN2[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 1, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_AGGCF_SCALE_DOWN2[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {9, 28, 1, 0, {0, 0, 0, 0}}, {9, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_AGGCF_SCALE_DOWN2[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 1, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_AGGCF_SCALE_DOWN2[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {11, 28, 1, 0, {0, 0, 0, 0}}, {11, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_AGGCF_SCALE_DOWN2[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 1, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_AGGCF_SCALE_DOWN2[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 1, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_AGGCF_SCALE_DOWN2[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {3, 32, 1, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_AGGCF_SCALE_DOWN2[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 1, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_AGGCF_SCALE_DOWN2[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {5, 32, 1, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_AGGCF_SCALE_DOWN2[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 1, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_AGGCF_SCALE_DOWN2[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {7, 32, 1, 0, {0, 0, 0, 0}}, {7, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_AGGCF_SCALE_DOWN2[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 1, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_AGGCF_SCALE_DOWN2[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {9, 32, 1, 0, {0, 0, 0, 0}}, {9, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_AGGCF_SCALE_DOWN2[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 1, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_AGGCF_SCALE_DOWN2[] = {{2, 36, 2, 0, {0, 0, 0, 0}}, {1, 36, 1, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_AGGCF_SCALE_DOWN2[] = {{4, 36, 2, 0, {0, 0, 0, 0}}, {2, 36, 1, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_AGGCF_SCALE_DOWN2[] = {{6, 36, 2, 0, {0, 0, 0, 0}}, {3, 36, 1, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_AGGCF_SCALE_DOWN2[] = {{8, 36, 2, 0, {0, 0, 0, 0}}, {4, 36, 1, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_AGGCF_SCALE_DOWN2[] = {{10, 36, 2, 0, {0, 0, 0, 0}}, {5, 36, 1, 0, {0, 0, 0, 0}}, {5, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_AGGCF_SCALE_DOWN2[] = {{12, 36, 2, 0, {0, 0, 0, 0}}, {6, 36, 1, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_AGGCF_SCALE_DOWN2[] = {{14, 36, 2, 0, {0, 0, 0, 0}}, {7, 36, 1, 0, {0, 0, 0, 0}}, {7, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_AGGCF_SCALE_DOWN2[] = {{16, 36, 2, 0, {0, 0, 0, 0}}, {8, 36, 1, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_AGGCF_SCALE_DOWN2[] = {{2, 40, 2, 0, {0, 0, 0, 0}}, {1, 40, 1, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_AGGCF_SCALE_DOWN2[] = {{4, 40, 2, 0, {0, 0, 0, 0}}, {2, 40, 1, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_AGGCF_SCALE_DOWN2[] = {{6, 40, 2, 0, {0, 0, 0, 0}}, {3, 40, 1, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_AGGCF_SCALE_DOWN2[] = {{8, 40, 2, 0, {0, 0, 0, 0}}, {4, 40, 1, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_AGGCF_SCALE_DOWN2[] = {{10, 40, 2, 0, {0, 0, 0, 0}}, {5, 40, 1, 0, {0, 0, 0, 0}}, {5, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_AGGCF_SCALE_DOWN2[] = {{12, 40, 2, 0, {0, 0, 0, 0}}, {6, 40, 1, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_AGGCF_SCALE_DOWN2[] = {{14, 40, 2, 0, {0, 0, 0, 0}}, {7, 40, 1, 0, {0, 0, 0, 0}}, {7, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_AGGCF_SCALE_DOWN2[] = {{16, 40, 2, 0, {0, 0, 0, 0}}, {8, 40, 1, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_AGGCF_SCALE_DOWN2[] = {{2, 44, 2, 0, {0, 0, 0, 0}}, {1, 44, 1, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_AGGCF_SCALE_DOWN2[] = {{4, 44, 2, 0, {0, 0, 0, 0}}, {2, 44, 1, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_AGGCF_SCALE_DOWN2[] = {{6, 44, 2, 0, {0, 0, 0, 0}}, {3, 44, 1, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_AGGCF_SCALE_DOWN2[] = {{8, 44, 2, 0, {0, 0, 0, 0}}, {4, 44, 1, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_AGGCF_SCALE_DOWN2[] = {{10, 44, 2, 0, {0, 0, 0, 0}}, {5, 44, 1, 0, {0, 0, 0, 0}}, {5, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_AGGCF_SCALE_DOWN2[] = {{12, 44, 2, 0, {0, 0, 0, 0}}, {6, 44, 1, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_AGGCF_SCALE_DOWN2[] = {{14, 44, 2, 0, {0, 0, 0, 0}}, {7, 44, 1, 0, {0, 0, 0, 0}}, {7, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_AGGCF_SCALE_DOWN2[] = {{2, 48, 2, 0, {0, 0, 0, 0}}, {1, 48, 1, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_AGGCF_SCALE_DOWN2[] = {{4, 48, 2, 0, {0, 0, 0, 0}}, {2, 48, 1, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_AGGCF_SCALE_DOWN2[] = {{6, 48, 2, 0, {0, 0, 0, 0}}, {3, 48, 1, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_AGGCF_SCALE_DOWN2[] = {{8, 48, 2, 0, {0, 0, 0, 0}}, {4, 48, 1, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_AGGCF_SCALE_DOWN2[] = {{10, 48, 2, 0, {0, 0, 0, 0}}, {5, 48, 1, 0, {0, 0, 0, 0}}, {5, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_AGGCF_SCALE_DOWN2[] = {{12, 48, 2, 0, {0, 0, 0, 0}}, {6, 48, 1, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_AGGCF_SCALE_DOWN2[] = {{2, 52, 2, 0, {0, 0, 0, 0}}, {1, 52, 1, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_AGGCF_SCALE_DOWN2[] = {{4, 52, 2, 0, {0, 0, 0, 0}}, {2, 52, 1, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_AGGCF_SCALE_DOWN2[] = {{6, 52, 2, 0, {0, 0, 0, 0}}, {3, 52, 1, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_AGGCF_SCALE_DOWN2[] = {{8, 52, 2, 0, {0, 0, 0, 0}}, {4, 52, 1, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_AGGCF_SCALE_DOWN2[] = {{10, 52, 2, 0, {0, 0, 0, 0}}, {5, 52, 1, 0, {0, 0, 0, 0}}, {5, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_AGGCF_SCALE_DOWN2[] = {{12, 52, 2, 0, {0, 0, 0, 0}}, {6, 52, 1, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_AGGCF_SCALE_DOWN2[] = {{2, 56, 2, 0, {0, 0, 0, 0}}, {1, 56, 1, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_AGGCF_SCALE_DOWN2[] = {{4, 56, 2, 0, {0, 0, 0, 0}}, {2, 56, 1, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_AGGCF_SCALE_DOWN2[] = {{6, 56, 2, 0, {0, 0, 0, 0}}, {3, 56, 1, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_AGGCF_SCALE_DOWN2[] = {{8, 56, 2, 0, {0, 0, 0, 0}}, {4, 56, 1, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_AGGCF_SCALE_DOWN2[] = {{10, 56, 2, 0, {0, 0, 0, 0}}, {5, 56, 1, 0, {0, 0, 0, 0}}, {5, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_AGGCF_SCALE_DOWN2[] = {{2, 60, 2, 0, {0, 0, 0, 0}}, {1, 60, 1, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_AGGCF_SCALE_DOWN2[] = {{4, 60, 2, 0, {0, 0, 0, 0}}, {2, 60, 1, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_AGGCF_SCALE_DOWN2[] = {{6, 60, 2, 0, {0, 0, 0, 0}}, {3, 60, 1, 0, {0, 0, 0, 0}}, {3, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_AGGCF_SCALE_DOWN2[] = {{8, 60, 2, 0, {0, 0, 0, 0}}, {4, 60, 1, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_AGGCF_SCALE_DOWN2[] = {{10, 60, 2, 0, {0, 0, 0, 0}}, {5, 60, 1, 0, {0, 0, 0, 0}}, {5, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_AGGCF_SCALE_DOWN2[] = {{2, 64, 2, 0, {0, 0, 0, 0}}, {1, 64, 1, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_AGGCF_SCALE_DOWN2[] = {{4, 64, 2, 0, {0, 0, 0, 0}}, {2, 64, 1, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_AGGCF_SCALE_DOWN2[] = {{6, 64, 2, 0, {0, 0, 0, 0}}, {3, 64, 1, 0, {0, 0, 0, 0}}, {3, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_AGGCF_SCALE_DOWN2[] = {{8, 64, 2, 0, {0, 0, 0, 0}}, {4, 64, 1, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_AGGCF_SCALE_DOWN2[] = {{10, 64, 2, 0, {0, 0, 0, 0}}, {5, 64, 1, 0, {0, 0, 0, 0}}, {5, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_AGGCF_SCALE_DOWN2[] = {{0, 0}, {0, 0}};

static acf_scenario gScenarioArray_AGGCF_SCALE_DOWN2[] = {
{1, 1, 24, 0, 0, gScenarioBufferData0_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData0_AGGCF_SCALE_DOWN2, 8},
{2, 1, 48, 0, 0, gScenarioBufferData1_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData1_AGGCF_SCALE_DOWN2, 8},
{3, 1, 72, 0, 0, gScenarioBufferData2_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData2_AGGCF_SCALE_DOWN2, 8},
{4, 1, 96, 0, 0, gScenarioBufferData3_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData3_AGGCF_SCALE_DOWN2, 8},
{5, 1, 120, 0, 0, gScenarioBufferData4_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData4_AGGCF_SCALE_DOWN2, 8},
{6, 1, 144, 0, 0, gScenarioBufferData5_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData5_AGGCF_SCALE_DOWN2, 8},
{7, 1, 168, 0, 0, gScenarioBufferData6_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData6_AGGCF_SCALE_DOWN2, 8},
{8, 1, 192, 0, 0, gScenarioBufferData7_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData7_AGGCF_SCALE_DOWN2, 8},
{9, 1, 216, 0, 0, gScenarioBufferData8_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData8_AGGCF_SCALE_DOWN2, 8},
{10, 1, 240, 0, 0, gScenarioBufferData9_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData9_AGGCF_SCALE_DOWN2, 8},
{11, 1, 264, 0, 0, gScenarioBufferData10_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData10_AGGCF_SCALE_DOWN2, 8},
{12, 1, 288, 0, 0, gScenarioBufferData11_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData11_AGGCF_SCALE_DOWN2, 8},
{13, 1, 312, 0, 0, gScenarioBufferData12_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData12_AGGCF_SCALE_DOWN2, 8},
{14, 1, 336, 0, 0, gScenarioBufferData13_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData13_AGGCF_SCALE_DOWN2, 8},
{15, 1, 360, 0, 0, gScenarioBufferData14_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData14_AGGCF_SCALE_DOWN2, 8},
{16, 1, 384, 0, 0, gScenarioBufferData15_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData15_AGGCF_SCALE_DOWN2, 8},
{18, 1, 432, 0, 0, gScenarioBufferData16_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData16_AGGCF_SCALE_DOWN2, 8},
{20, 1, 480, 0, 0, gScenarioBufferData17_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData17_AGGCF_SCALE_DOWN2, 8},
{22, 1, 528, 0, 0, gScenarioBufferData18_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData18_AGGCF_SCALE_DOWN2, 8},
{24, 1, 576, 0, 0, gScenarioBufferData19_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData19_AGGCF_SCALE_DOWN2, 8},
{26, 1, 624, 0, 0, gScenarioBufferData20_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData20_AGGCF_SCALE_DOWN2, 8},
{28, 1, 672, 0, 0, gScenarioBufferData21_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData21_AGGCF_SCALE_DOWN2, 8},
{30, 1, 720, 0, 0, gScenarioBufferData22_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData22_AGGCF_SCALE_DOWN2, 8},
{32, 1, 768, 0, 0, gScenarioBufferData23_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData23_AGGCF_SCALE_DOWN2, 8},
{1, 2, 48, 0, 0, gScenarioBufferData24_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData24_AGGCF_SCALE_DOWN2, 8},
{2, 2, 96, 0, 0, gScenarioBufferData25_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData25_AGGCF_SCALE_DOWN2, 8},
{3, 2, 144, 0, 0, gScenarioBufferData26_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData26_AGGCF_SCALE_DOWN2, 8},
{4, 2, 192, 0, 0, gScenarioBufferData27_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData27_AGGCF_SCALE_DOWN2, 8},
{5, 2, 240, 0, 0, gScenarioBufferData28_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData28_AGGCF_SCALE_DOWN2, 8},
{6, 2, 288, 0, 0, gScenarioBufferData29_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData29_AGGCF_SCALE_DOWN2, 8},
{7, 2, 336, 0, 0, gScenarioBufferData30_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData30_AGGCF_SCALE_DOWN2, 8},
{8, 2, 384, 0, 0, gScenarioBufferData31_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData31_AGGCF_SCALE_DOWN2, 8},
{9, 2, 432, 0, 0, gScenarioBufferData32_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData32_AGGCF_SCALE_DOWN2, 8},
{10, 2, 480, 0, 0, gScenarioBufferData33_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData33_AGGCF_SCALE_DOWN2, 8},
{11, 2, 528, 0, 0, gScenarioBufferData34_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData34_AGGCF_SCALE_DOWN2, 8},
{12, 2, 576, 0, 0, gScenarioBufferData35_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData35_AGGCF_SCALE_DOWN2, 8},
{13, 2, 624, 0, 0, gScenarioBufferData36_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData36_AGGCF_SCALE_DOWN2, 8},
{14, 2, 672, 0, 0, gScenarioBufferData37_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData37_AGGCF_SCALE_DOWN2, 8},
{15, 2, 720, 0, 0, gScenarioBufferData38_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData38_AGGCF_SCALE_DOWN2, 8},
{16, 2, 768, 0, 0, gScenarioBufferData39_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData39_AGGCF_SCALE_DOWN2, 8},
{18, 2, 864, 0, 0, gScenarioBufferData40_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData40_AGGCF_SCALE_DOWN2, 8},
{20, 2, 960, 0, 0, gScenarioBufferData41_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData41_AGGCF_SCALE_DOWN2, 8},
{22, 2, 1056, 0, 0, gScenarioBufferData42_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData42_AGGCF_SCALE_DOWN2, 8},
{24, 2, 1152, 0, 0, gScenarioBufferData43_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData43_AGGCF_SCALE_DOWN2, 8},
{26, 2, 1248, 0, 0, gScenarioBufferData44_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData44_AGGCF_SCALE_DOWN2, 8},
{28, 2, 1344, 0, 0, gScenarioBufferData45_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData45_AGGCF_SCALE_DOWN2, 8},
{30, 2, 1440, 0, 0, gScenarioBufferData46_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData46_AGGCF_SCALE_DOWN2, 8},
{32, 2, 1536, 0, 0, gScenarioBufferData47_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData47_AGGCF_SCALE_DOWN2, 8},
{1, 3, 72, 0, 0, gScenarioBufferData48_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData48_AGGCF_SCALE_DOWN2, 8},
{2, 3, 144, 0, 0, gScenarioBufferData49_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData49_AGGCF_SCALE_DOWN2, 8},
{3, 3, 216, 0, 0, gScenarioBufferData50_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData50_AGGCF_SCALE_DOWN2, 8},
{4, 3, 288, 0, 0, gScenarioBufferData51_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData51_AGGCF_SCALE_DOWN2, 8},
{5, 3, 360, 0, 0, gScenarioBufferData52_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData52_AGGCF_SCALE_DOWN2, 8},
{6, 3, 432, 0, 0, gScenarioBufferData53_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData53_AGGCF_SCALE_DOWN2, 8},
{7, 3, 504, 0, 0, gScenarioBufferData54_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData54_AGGCF_SCALE_DOWN2, 8},
{8, 3, 576, 0, 0, gScenarioBufferData55_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData55_AGGCF_SCALE_DOWN2, 8},
{9, 3, 648, 0, 0, gScenarioBufferData56_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData56_AGGCF_SCALE_DOWN2, 8},
{10, 3, 720, 0, 0, gScenarioBufferData57_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData57_AGGCF_SCALE_DOWN2, 8},
{11, 3, 792, 0, 0, gScenarioBufferData58_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData58_AGGCF_SCALE_DOWN2, 8},
{12, 3, 864, 0, 0, gScenarioBufferData59_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData59_AGGCF_SCALE_DOWN2, 8},
{13, 3, 936, 0, 0, gScenarioBufferData60_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData60_AGGCF_SCALE_DOWN2, 8},
{14, 3, 1008, 0, 0, gScenarioBufferData61_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData61_AGGCF_SCALE_DOWN2, 8},
{15, 3, 1080, 0, 0, gScenarioBufferData62_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData62_AGGCF_SCALE_DOWN2, 8},
{16, 3, 1152, 0, 0, gScenarioBufferData63_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData63_AGGCF_SCALE_DOWN2, 8},
{18, 3, 1296, 0, 0, gScenarioBufferData64_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData64_AGGCF_SCALE_DOWN2, 8},
{20, 3, 1440, 0, 0, gScenarioBufferData65_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData65_AGGCF_SCALE_DOWN2, 8},
{22, 3, 1584, 0, 0, gScenarioBufferData66_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData66_AGGCF_SCALE_DOWN2, 8},
{24, 3, 1728, 0, 0, gScenarioBufferData67_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData67_AGGCF_SCALE_DOWN2, 8},
{26, 3, 1872, 0, 0, gScenarioBufferData68_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData68_AGGCF_SCALE_DOWN2, 8},
{28, 3, 2016, 0, 0, gScenarioBufferData69_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData69_AGGCF_SCALE_DOWN2, 8},
{30, 3, 2160, 0, 0, gScenarioBufferData70_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData70_AGGCF_SCALE_DOWN2, 8},
{32, 3, 2304, 0, 0, gScenarioBufferData71_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData71_AGGCF_SCALE_DOWN2, 8},
{1, 4, 96, 0, 0, gScenarioBufferData72_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData72_AGGCF_SCALE_DOWN2, 8},
{2, 4, 192, 0, 0, gScenarioBufferData73_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData73_AGGCF_SCALE_DOWN2, 8},
{3, 4, 288, 0, 0, gScenarioBufferData74_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData74_AGGCF_SCALE_DOWN2, 8},
{4, 4, 384, 0, 0, gScenarioBufferData75_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData75_AGGCF_SCALE_DOWN2, 8},
{5, 4, 480, 0, 0, gScenarioBufferData76_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData76_AGGCF_SCALE_DOWN2, 8},
{6, 4, 576, 0, 0, gScenarioBufferData77_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData77_AGGCF_SCALE_DOWN2, 8},
{7, 4, 672, 0, 0, gScenarioBufferData78_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData78_AGGCF_SCALE_DOWN2, 8},
{8, 4, 768, 0, 0, gScenarioBufferData79_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData79_AGGCF_SCALE_DOWN2, 8},
{9, 4, 864, 0, 0, gScenarioBufferData80_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData80_AGGCF_SCALE_DOWN2, 8},
{10, 4, 960, 0, 0, gScenarioBufferData81_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData81_AGGCF_SCALE_DOWN2, 8},
{11, 4, 1056, 0, 0, gScenarioBufferData82_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData82_AGGCF_SCALE_DOWN2, 8},
{12, 4, 1152, 0, 0, gScenarioBufferData83_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData83_AGGCF_SCALE_DOWN2, 8},
{13, 4, 1248, 0, 0, gScenarioBufferData84_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData84_AGGCF_SCALE_DOWN2, 8},
{14, 4, 1344, 0, 0, gScenarioBufferData85_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData85_AGGCF_SCALE_DOWN2, 8},
{15, 4, 1440, 0, 0, gScenarioBufferData86_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData86_AGGCF_SCALE_DOWN2, 8},
{16, 4, 1536, 0, 0, gScenarioBufferData87_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData87_AGGCF_SCALE_DOWN2, 8},
{18, 4, 1728, 0, 0, gScenarioBufferData88_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData88_AGGCF_SCALE_DOWN2, 8},
{20, 4, 1920, 0, 0, gScenarioBufferData89_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData89_AGGCF_SCALE_DOWN2, 8},
{22, 4, 2112, 0, 0, gScenarioBufferData90_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData90_AGGCF_SCALE_DOWN2, 8},
{24, 4, 2304, 0, 0, gScenarioBufferData91_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData91_AGGCF_SCALE_DOWN2, 8},
{26, 4, 2496, 0, 0, gScenarioBufferData92_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData92_AGGCF_SCALE_DOWN2, 8},
{28, 4, 2688, 0, 0, gScenarioBufferData93_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData93_AGGCF_SCALE_DOWN2, 8},
{30, 4, 2880, 0, 0, gScenarioBufferData94_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData94_AGGCF_SCALE_DOWN2, 8},
{32, 4, 3072, 0, 0, gScenarioBufferData95_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData95_AGGCF_SCALE_DOWN2, 8},
{1, 5, 120, 0, 0, gScenarioBufferData96_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData96_AGGCF_SCALE_DOWN2, 8},
{2, 5, 240, 0, 0, gScenarioBufferData97_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData97_AGGCF_SCALE_DOWN2, 8},
{3, 5, 360, 0, 0, gScenarioBufferData98_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData98_AGGCF_SCALE_DOWN2, 8},
{4, 5, 480, 0, 0, gScenarioBufferData99_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData99_AGGCF_SCALE_DOWN2, 8},
{5, 5, 600, 0, 0, gScenarioBufferData100_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData100_AGGCF_SCALE_DOWN2, 8},
{6, 5, 720, 0, 0, gScenarioBufferData101_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData101_AGGCF_SCALE_DOWN2, 8},
{7, 5, 840, 0, 0, gScenarioBufferData102_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData102_AGGCF_SCALE_DOWN2, 8},
{8, 5, 960, 0, 0, gScenarioBufferData103_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData103_AGGCF_SCALE_DOWN2, 8},
{9, 5, 1080, 0, 0, gScenarioBufferData104_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData104_AGGCF_SCALE_DOWN2, 8},
{10, 5, 1200, 0, 0, gScenarioBufferData105_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData105_AGGCF_SCALE_DOWN2, 8},
{11, 5, 1320, 0, 0, gScenarioBufferData106_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData106_AGGCF_SCALE_DOWN2, 8},
{12, 5, 1440, 0, 0, gScenarioBufferData107_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData107_AGGCF_SCALE_DOWN2, 8},
{13, 5, 1560, 0, 0, gScenarioBufferData108_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData108_AGGCF_SCALE_DOWN2, 8},
{14, 5, 1680, 0, 0, gScenarioBufferData109_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData109_AGGCF_SCALE_DOWN2, 8},
{15, 5, 1800, 0, 0, gScenarioBufferData110_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData110_AGGCF_SCALE_DOWN2, 8},
{16, 5, 1920, 0, 0, gScenarioBufferData111_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData111_AGGCF_SCALE_DOWN2, 8},
{18, 5, 2160, 0, 0, gScenarioBufferData112_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData112_AGGCF_SCALE_DOWN2, 8},
{20, 5, 2400, 0, 0, gScenarioBufferData113_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData113_AGGCF_SCALE_DOWN2, 8},
{22, 5, 2640, 0, 0, gScenarioBufferData114_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData114_AGGCF_SCALE_DOWN2, 8},
{24, 5, 2880, 0, 0, gScenarioBufferData115_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData115_AGGCF_SCALE_DOWN2, 8},
{26, 5, 3120, 0, 0, gScenarioBufferData116_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData116_AGGCF_SCALE_DOWN2, 8},
{28, 5, 3360, 0, 0, gScenarioBufferData117_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData117_AGGCF_SCALE_DOWN2, 8},
{30, 5, 3600, 0, 0, gScenarioBufferData118_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData118_AGGCF_SCALE_DOWN2, 8},
{32, 5, 3840, 0, 0, gScenarioBufferData119_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData119_AGGCF_SCALE_DOWN2, 8},
{1, 6, 144, 0, 0, gScenarioBufferData120_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData120_AGGCF_SCALE_DOWN2, 8},
{2, 6, 288, 0, 0, gScenarioBufferData121_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData121_AGGCF_SCALE_DOWN2, 8},
{3, 6, 432, 0, 0, gScenarioBufferData122_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData122_AGGCF_SCALE_DOWN2, 8},
{4, 6, 576, 0, 0, gScenarioBufferData123_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData123_AGGCF_SCALE_DOWN2, 8},
{5, 6, 720, 0, 0, gScenarioBufferData124_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData124_AGGCF_SCALE_DOWN2, 8},
{6, 6, 864, 0, 0, gScenarioBufferData125_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData125_AGGCF_SCALE_DOWN2, 8},
{7, 6, 1008, 0, 0, gScenarioBufferData126_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData126_AGGCF_SCALE_DOWN2, 8},
{8, 6, 1152, 0, 0, gScenarioBufferData127_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData127_AGGCF_SCALE_DOWN2, 8},
{9, 6, 1296, 0, 0, gScenarioBufferData128_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData128_AGGCF_SCALE_DOWN2, 8},
{10, 6, 1440, 0, 0, gScenarioBufferData129_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData129_AGGCF_SCALE_DOWN2, 8},
{11, 6, 1584, 0, 0, gScenarioBufferData130_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData130_AGGCF_SCALE_DOWN2, 8},
{12, 6, 1728, 0, 0, gScenarioBufferData131_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData131_AGGCF_SCALE_DOWN2, 8},
{13, 6, 1872, 0, 0, gScenarioBufferData132_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData132_AGGCF_SCALE_DOWN2, 8},
{14, 6, 2016, 0, 0, gScenarioBufferData133_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData133_AGGCF_SCALE_DOWN2, 8},
{15, 6, 2160, 0, 0, gScenarioBufferData134_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData134_AGGCF_SCALE_DOWN2, 8},
{16, 6, 2304, 0, 0, gScenarioBufferData135_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData135_AGGCF_SCALE_DOWN2, 8},
{18, 6, 2592, 0, 0, gScenarioBufferData136_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData136_AGGCF_SCALE_DOWN2, 8},
{20, 6, 2880, 0, 0, gScenarioBufferData137_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData137_AGGCF_SCALE_DOWN2, 8},
{22, 6, 3168, 0, 0, gScenarioBufferData138_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData138_AGGCF_SCALE_DOWN2, 8},
{24, 6, 3456, 0, 0, gScenarioBufferData139_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData139_AGGCF_SCALE_DOWN2, 8},
{26, 6, 3744, 0, 0, gScenarioBufferData140_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData140_AGGCF_SCALE_DOWN2, 8},
{1, 8, 192, 0, 0, gScenarioBufferData141_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData141_AGGCF_SCALE_DOWN2, 8},
{2, 8, 384, 0, 0, gScenarioBufferData142_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData142_AGGCF_SCALE_DOWN2, 8},
{3, 8, 576, 0, 0, gScenarioBufferData143_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData143_AGGCF_SCALE_DOWN2, 8},
{4, 8, 768, 0, 0, gScenarioBufferData144_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData144_AGGCF_SCALE_DOWN2, 8},
{5, 8, 960, 0, 0, gScenarioBufferData145_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData145_AGGCF_SCALE_DOWN2, 8},
{6, 8, 1152, 0, 0, gScenarioBufferData146_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData146_AGGCF_SCALE_DOWN2, 8},
{7, 8, 1344, 0, 0, gScenarioBufferData147_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData147_AGGCF_SCALE_DOWN2, 8},
{8, 8, 1536, 0, 0, gScenarioBufferData148_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData148_AGGCF_SCALE_DOWN2, 8},
{9, 8, 1728, 0, 0, gScenarioBufferData149_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData149_AGGCF_SCALE_DOWN2, 8},
{10, 8, 1920, 0, 0, gScenarioBufferData150_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData150_AGGCF_SCALE_DOWN2, 8},
{11, 8, 2112, 0, 0, gScenarioBufferData151_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData151_AGGCF_SCALE_DOWN2, 8},
{12, 8, 2304, 0, 0, gScenarioBufferData152_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData152_AGGCF_SCALE_DOWN2, 8},
{13, 8, 2496, 0, 0, gScenarioBufferData153_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData153_AGGCF_SCALE_DOWN2, 8},
{14, 8, 2688, 0, 0, gScenarioBufferData154_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData154_AGGCF_SCALE_DOWN2, 8},
{15, 8, 2880, 0, 0, gScenarioBufferData155_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData155_AGGCF_SCALE_DOWN2, 8},
{16, 8, 3072, 0, 0, gScenarioBufferData156_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData156_AGGCF_SCALE_DOWN2, 8},
{18, 8, 3456, 0, 0, gScenarioBufferData157_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData157_AGGCF_SCALE_DOWN2, 8},
{20, 8, 3840, 0, 0, gScenarioBufferData158_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData158_AGGCF_SCALE_DOWN2, 8},
{1, 10, 240, 0, 0, gScenarioBufferData159_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData159_AGGCF_SCALE_DOWN2, 8},
{2, 10, 480, 0, 0, gScenarioBufferData160_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData160_AGGCF_SCALE_DOWN2, 8},
{3, 10, 720, 0, 0, gScenarioBufferData161_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData161_AGGCF_SCALE_DOWN2, 8},
{4, 10, 960, 0, 0, gScenarioBufferData162_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData162_AGGCF_SCALE_DOWN2, 8},
{5, 10, 1200, 0, 0, gScenarioBufferData163_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData163_AGGCF_SCALE_DOWN2, 8},
{6, 10, 1440, 0, 0, gScenarioBufferData164_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData164_AGGCF_SCALE_DOWN2, 8},
{7, 10, 1680, 0, 0, gScenarioBufferData165_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData165_AGGCF_SCALE_DOWN2, 8},
{8, 10, 1920, 0, 0, gScenarioBufferData166_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData166_AGGCF_SCALE_DOWN2, 8},
{9, 10, 2160, 0, 0, gScenarioBufferData167_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData167_AGGCF_SCALE_DOWN2, 8},
{10, 10, 2400, 0, 0, gScenarioBufferData168_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData168_AGGCF_SCALE_DOWN2, 8},
{11, 10, 2640, 0, 0, gScenarioBufferData169_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData169_AGGCF_SCALE_DOWN2, 8},
{12, 10, 2880, 0, 0, gScenarioBufferData170_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData170_AGGCF_SCALE_DOWN2, 8},
{13, 10, 3120, 0, 0, gScenarioBufferData171_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData171_AGGCF_SCALE_DOWN2, 8},
{14, 10, 3360, 0, 0, gScenarioBufferData172_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData172_AGGCF_SCALE_DOWN2, 8},
{15, 10, 3600, 0, 0, gScenarioBufferData173_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData173_AGGCF_SCALE_DOWN2, 8},
{16, 10, 3840, 0, 0, gScenarioBufferData174_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData174_AGGCF_SCALE_DOWN2, 8},
{1, 12, 288, 0, 0, gScenarioBufferData175_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData175_AGGCF_SCALE_DOWN2, 8},
{2, 12, 576, 0, 0, gScenarioBufferData176_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData176_AGGCF_SCALE_DOWN2, 8},
{3, 12, 864, 0, 0, gScenarioBufferData177_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData177_AGGCF_SCALE_DOWN2, 8},
{4, 12, 1152, 0, 0, gScenarioBufferData178_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData178_AGGCF_SCALE_DOWN2, 8},
{5, 12, 1440, 0, 0, gScenarioBufferData179_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData179_AGGCF_SCALE_DOWN2, 8},
{6, 12, 1728, 0, 0, gScenarioBufferData180_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData180_AGGCF_SCALE_DOWN2, 8},
{7, 12, 2016, 0, 0, gScenarioBufferData181_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData181_AGGCF_SCALE_DOWN2, 8},
{8, 12, 2304, 0, 0, gScenarioBufferData182_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData182_AGGCF_SCALE_DOWN2, 8},
{9, 12, 2592, 0, 0, gScenarioBufferData183_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData183_AGGCF_SCALE_DOWN2, 8},
{10, 12, 2880, 0, 0, gScenarioBufferData184_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData184_AGGCF_SCALE_DOWN2, 8},
{11, 12, 3168, 0, 0, gScenarioBufferData185_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData185_AGGCF_SCALE_DOWN2, 8},
{12, 12, 3456, 0, 0, gScenarioBufferData186_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData186_AGGCF_SCALE_DOWN2, 8},
{13, 12, 3744, 0, 0, gScenarioBufferData187_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData187_AGGCF_SCALE_DOWN2, 8},
{1, 14, 336, 0, 0, gScenarioBufferData188_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData188_AGGCF_SCALE_DOWN2, 8},
{2, 14, 672, 0, 0, gScenarioBufferData189_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData189_AGGCF_SCALE_DOWN2, 8},
{3, 14, 1008, 0, 0, gScenarioBufferData190_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData190_AGGCF_SCALE_DOWN2, 8},
{4, 14, 1344, 0, 0, gScenarioBufferData191_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData191_AGGCF_SCALE_DOWN2, 8},
{5, 14, 1680, 0, 0, gScenarioBufferData192_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData192_AGGCF_SCALE_DOWN2, 8},
{6, 14, 2016, 0, 0, gScenarioBufferData193_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData193_AGGCF_SCALE_DOWN2, 8},
{7, 14, 2352, 0, 0, gScenarioBufferData194_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData194_AGGCF_SCALE_DOWN2, 8},
{8, 14, 2688, 0, 0, gScenarioBufferData195_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData195_AGGCF_SCALE_DOWN2, 8},
{9, 14, 3024, 0, 0, gScenarioBufferData196_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData196_AGGCF_SCALE_DOWN2, 8},
{10, 14, 3360, 0, 0, gScenarioBufferData197_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData197_AGGCF_SCALE_DOWN2, 8},
{11, 14, 3696, 0, 0, gScenarioBufferData198_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData198_AGGCF_SCALE_DOWN2, 8},
{1, 16, 384, 0, 0, gScenarioBufferData199_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData199_AGGCF_SCALE_DOWN2, 8},
{2, 16, 768, 0, 0, gScenarioBufferData200_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData200_AGGCF_SCALE_DOWN2, 8},
{3, 16, 1152, 0, 0, gScenarioBufferData201_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData201_AGGCF_SCALE_DOWN2, 8},
{4, 16, 1536, 0, 0, gScenarioBufferData202_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData202_AGGCF_SCALE_DOWN2, 8},
{5, 16, 1920, 0, 0, gScenarioBufferData203_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData203_AGGCF_SCALE_DOWN2, 8},
{6, 16, 2304, 0, 0, gScenarioBufferData204_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData204_AGGCF_SCALE_DOWN2, 8},
{7, 16, 2688, 0, 0, gScenarioBufferData205_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData205_AGGCF_SCALE_DOWN2, 8},
{8, 16, 3072, 0, 0, gScenarioBufferData206_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData206_AGGCF_SCALE_DOWN2, 8},
{9, 16, 3456, 0, 0, gScenarioBufferData207_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData207_AGGCF_SCALE_DOWN2, 8},
{10, 16, 3840, 0, 0, gScenarioBufferData208_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData208_AGGCF_SCALE_DOWN2, 8},
{1, 18, 432, 0, 0, gScenarioBufferData209_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData209_AGGCF_SCALE_DOWN2, 8},
{2, 18, 864, 0, 0, gScenarioBufferData210_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData210_AGGCF_SCALE_DOWN2, 8},
{3, 18, 1296, 0, 0, gScenarioBufferData211_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData211_AGGCF_SCALE_DOWN2, 8},
{4, 18, 1728, 0, 0, gScenarioBufferData212_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData212_AGGCF_SCALE_DOWN2, 8},
{5, 18, 2160, 0, 0, gScenarioBufferData213_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData213_AGGCF_SCALE_DOWN2, 8},
{6, 18, 2592, 0, 0, gScenarioBufferData214_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData214_AGGCF_SCALE_DOWN2, 8},
{7, 18, 3024, 0, 0, gScenarioBufferData215_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData215_AGGCF_SCALE_DOWN2, 8},
{8, 18, 3456, 0, 0, gScenarioBufferData216_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData216_AGGCF_SCALE_DOWN2, 8},
{1, 20, 480, 0, 0, gScenarioBufferData217_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData217_AGGCF_SCALE_DOWN2, 8},
{2, 20, 960, 0, 0, gScenarioBufferData218_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData218_AGGCF_SCALE_DOWN2, 8},
{3, 20, 1440, 0, 0, gScenarioBufferData219_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData219_AGGCF_SCALE_DOWN2, 8},
{4, 20, 1920, 0, 0, gScenarioBufferData220_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData220_AGGCF_SCALE_DOWN2, 8},
{5, 20, 2400, 0, 0, gScenarioBufferData221_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData221_AGGCF_SCALE_DOWN2, 8},
{6, 20, 2880, 0, 0, gScenarioBufferData222_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData222_AGGCF_SCALE_DOWN2, 8},
{7, 20, 3360, 0, 0, gScenarioBufferData223_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData223_AGGCF_SCALE_DOWN2, 8},
{8, 20, 3840, 0, 0, gScenarioBufferData224_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData224_AGGCF_SCALE_DOWN2, 8},
{1, 22, 528, 0, 0, gScenarioBufferData225_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData225_AGGCF_SCALE_DOWN2, 8},
{2, 22, 1056, 0, 0, gScenarioBufferData226_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData226_AGGCF_SCALE_DOWN2, 8},
{3, 22, 1584, 0, 0, gScenarioBufferData227_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData227_AGGCF_SCALE_DOWN2, 8},
{4, 22, 2112, 0, 0, gScenarioBufferData228_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData228_AGGCF_SCALE_DOWN2, 8},
{5, 22, 2640, 0, 0, gScenarioBufferData229_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData229_AGGCF_SCALE_DOWN2, 8},
{6, 22, 3168, 0, 0, gScenarioBufferData230_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData230_AGGCF_SCALE_DOWN2, 8},
{7, 22, 3696, 0, 0, gScenarioBufferData231_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData231_AGGCF_SCALE_DOWN2, 8},
{1, 24, 576, 0, 0, gScenarioBufferData232_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData232_AGGCF_SCALE_DOWN2, 8},
{2, 24, 1152, 0, 0, gScenarioBufferData233_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData233_AGGCF_SCALE_DOWN2, 8},
{3, 24, 1728, 0, 0, gScenarioBufferData234_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData234_AGGCF_SCALE_DOWN2, 8},
{4, 24, 2304, 0, 0, gScenarioBufferData235_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData235_AGGCF_SCALE_DOWN2, 8},
{5, 24, 2880, 0, 0, gScenarioBufferData236_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData236_AGGCF_SCALE_DOWN2, 8},
{6, 24, 3456, 0, 0, gScenarioBufferData237_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData237_AGGCF_SCALE_DOWN2, 8},
{1, 26, 624, 0, 0, gScenarioBufferData238_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData238_AGGCF_SCALE_DOWN2, 8},
{2, 26, 1248, 0, 0, gScenarioBufferData239_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData239_AGGCF_SCALE_DOWN2, 8},
{3, 26, 1872, 0, 0, gScenarioBufferData240_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData240_AGGCF_SCALE_DOWN2, 8},
{4, 26, 2496, 0, 0, gScenarioBufferData241_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData241_AGGCF_SCALE_DOWN2, 8},
{5, 26, 3120, 0, 0, gScenarioBufferData242_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData242_AGGCF_SCALE_DOWN2, 8},
{6, 26, 3744, 0, 0, gScenarioBufferData243_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData243_AGGCF_SCALE_DOWN2, 8},
{1, 28, 672, 0, 0, gScenarioBufferData244_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData244_AGGCF_SCALE_DOWN2, 8},
{2, 28, 1344, 0, 0, gScenarioBufferData245_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData245_AGGCF_SCALE_DOWN2, 8},
{3, 28, 2016, 0, 0, gScenarioBufferData246_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData246_AGGCF_SCALE_DOWN2, 8},
{4, 28, 2688, 0, 0, gScenarioBufferData247_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData247_AGGCF_SCALE_DOWN2, 8},
{5, 28, 3360, 0, 0, gScenarioBufferData248_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData248_AGGCF_SCALE_DOWN2, 8},
{1, 30, 720, 0, 0, gScenarioBufferData249_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData249_AGGCF_SCALE_DOWN2, 8},
{2, 30, 1440, 0, 0, gScenarioBufferData250_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData250_AGGCF_SCALE_DOWN2, 8},
{3, 30, 2160, 0, 0, gScenarioBufferData251_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData251_AGGCF_SCALE_DOWN2, 8},
{4, 30, 2880, 0, 0, gScenarioBufferData252_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData252_AGGCF_SCALE_DOWN2, 8},
{5, 30, 3600, 0, 0, gScenarioBufferData253_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData253_AGGCF_SCALE_DOWN2, 8},
{1, 32, 768, 0, 0, gScenarioBufferData254_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData254_AGGCF_SCALE_DOWN2, 8},
{2, 32, 1536, 0, 0, gScenarioBufferData255_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData255_AGGCF_SCALE_DOWN2, 8},
{3, 32, 2304, 0, 0, gScenarioBufferData256_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData256_AGGCF_SCALE_DOWN2, 8},
{4, 32, 3072, 0, 0, gScenarioBufferData257_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData257_AGGCF_SCALE_DOWN2, 8},
{5, 32, 3840, 0, 0, gScenarioBufferData258_AGGCF_SCALE_DOWN2, 48, gScenarioKernelData258_AGGCF_SCALE_DOWN2, 8}
};

static acf_scenario_list gScenarioList_AGGCF_SCALE_DOWN2 = {
259, //number of scenarios
gScenarioArray_AGGCF_SCALE_DOWN2};
//**************************************************************

class AGGCF_SCALE_DOWN2 : public ACF_Process_APU
{

public:
   AGGCF_SCALE_DOWN2(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_SCALE_DOWN2");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_SCALE_DOWN2",
                                        AGGCF_SCALE_DOWN2_LOAD_SEGMENTS,
                                        AGGCF_SCALE_DOWN2_LOAD_PMEM, AGGCF_SCALE_DOWN2_LOAD_PMEM_SIZE,
                                        AGGCF_SCALE_DOWN2_LOAD_DMEM, AGGCF_SCALE_DOWN2_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_SCALE_DOWN2_APEX_LOG_BUFFER, AGGCF_SCALE_DOWN2_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("SRC", icp::DATATYPE_16U, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("DST", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_SCALE_DOWN2);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_SCALE_DOWN2

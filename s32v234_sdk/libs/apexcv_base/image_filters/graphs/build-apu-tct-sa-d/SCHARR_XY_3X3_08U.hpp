#ifndef _ACF_PROCESS_APU_SCHARR_XY_3X3_08U
#define _ACF_PROCESS_APU_SCHARR_XY_3X3_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <SCHARR_XY_3X3_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_SCHARR_XY_3X3_08U[] = {{2, 1, 4, 0, {1, 1, 2, 2}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_SCHARR_XY_3X3_08U[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_SCHARR_XY_3X3_08U[] = {{6, 1, 4, 0, {1, 1, 2, 2}}, {6, 1, 2, 1, {0, 0, 0, 0}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_SCHARR_XY_3X3_08U[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_SCHARR_XY_3X3_08U[] = {{10, 1, 4, 0, {1, 1, 2, 2}}, {10, 1, 2, 1, {0, 0, 0, 0}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_SCHARR_XY_3X3_08U[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_SCHARR_XY_3X3_08U[] = {{14, 1, 4, 0, {1, 1, 2, 2}}, {14, 1, 2, 1, {0, 0, 0, 0}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_SCHARR_XY_3X3_08U[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_SCHARR_XY_3X3_08U[] = {{18, 1, 4, 0, {1, 1, 2, 2}}, {18, 1, 2, 1, {0, 0, 0, 0}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_SCHARR_XY_3X3_08U[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_SCHARR_XY_3X3_08U[] = {{22, 1, 4, 0, {1, 1, 2, 2}}, {22, 1, 2, 1, {0, 0, 0, 0}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_SCHARR_XY_3X3_08U[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_SCHARR_XY_3X3_08U[] = {{26, 1, 4, 0, {1, 1, 2, 2}}, {26, 1, 2, 1, {0, 0, 0, 0}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_SCHARR_XY_3X3_08U[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_SCHARR_XY_3X3_08U[] = {{30, 1, 4, 0, {1, 1, 2, 2}}, {30, 1, 2, 1, {0, 0, 0, 0}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_SCHARR_XY_3X3_08U[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_SCHARR_XY_3X3_08U[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_SCHARR_XY_3X3_08U[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_SCHARR_XY_3X3_08U[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_SCHARR_XY_3X3_08U[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_SCHARR_XY_3X3_08U[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_SCHARR_XY_3X3_08U[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_SCHARR_XY_3X3_08U[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_SCHARR_XY_3X3_08U[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_SCHARR_XY_3X3_08U[] = {{2, 2, 4, 0, {1, 1, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_SCHARR_XY_3X3_08U[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_SCHARR_XY_3X3_08U[] = {{6, 2, 4, 0, {1, 1, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_SCHARR_XY_3X3_08U[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_SCHARR_XY_3X3_08U[] = {{10, 2, 4, 0, {1, 1, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_SCHARR_XY_3X3_08U[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_SCHARR_XY_3X3_08U[] = {{14, 2, 4, 0, {1, 1, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_SCHARR_XY_3X3_08U[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_SCHARR_XY_3X3_08U[] = {{18, 2, 4, 0, {1, 1, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_SCHARR_XY_3X3_08U[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_SCHARR_XY_3X3_08U[] = {{22, 2, 4, 0, {1, 1, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_SCHARR_XY_3X3_08U[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_SCHARR_XY_3X3_08U[] = {{26, 2, 4, 0, {1, 1, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_SCHARR_XY_3X3_08U[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_SCHARR_XY_3X3_08U[] = {{30, 2, 4, 0, {1, 1, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_SCHARR_XY_3X3_08U[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_SCHARR_XY_3X3_08U[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_SCHARR_XY_3X3_08U[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_SCHARR_XY_3X3_08U[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_SCHARR_XY_3X3_08U[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_SCHARR_XY_3X3_08U[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_SCHARR_XY_3X3_08U[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_SCHARR_XY_3X3_08U[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_SCHARR_XY_3X3_08U[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_SCHARR_XY_3X3_08U[] = {{2, 3, 4, 0, {1, 1, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_SCHARR_XY_3X3_08U[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_SCHARR_XY_3X3_08U[] = {{6, 3, 4, 0, {1, 1, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_SCHARR_XY_3X3_08U[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_SCHARR_XY_3X3_08U[] = {{10, 3, 4, 0, {1, 1, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_SCHARR_XY_3X3_08U[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_SCHARR_XY_3X3_08U[] = {{14, 3, 4, 0, {1, 1, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_SCHARR_XY_3X3_08U[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_SCHARR_XY_3X3_08U[] = {{18, 3, 4, 0, {1, 1, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_SCHARR_XY_3X3_08U[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_SCHARR_XY_3X3_08U[] = {{22, 3, 4, 0, {1, 1, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_SCHARR_XY_3X3_08U[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_SCHARR_XY_3X3_08U[] = {{26, 3, 4, 0, {1, 1, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_SCHARR_XY_3X3_08U[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_SCHARR_XY_3X3_08U[] = {{30, 3, 4, 0, {1, 1, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_SCHARR_XY_3X3_08U[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_SCHARR_XY_3X3_08U[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_SCHARR_XY_3X3_08U[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_SCHARR_XY_3X3_08U[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_SCHARR_XY_3X3_08U[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_SCHARR_XY_3X3_08U[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_SCHARR_XY_3X3_08U[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_SCHARR_XY_3X3_08U[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_SCHARR_XY_3X3_08U[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_SCHARR_XY_3X3_08U[] = {{2, 4, 4, 0, {1, 1, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_SCHARR_XY_3X3_08U[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_SCHARR_XY_3X3_08U[] = {{6, 4, 4, 0, {1, 1, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_SCHARR_XY_3X3_08U[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_SCHARR_XY_3X3_08U[] = {{10, 4, 4, 0, {1, 1, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_SCHARR_XY_3X3_08U[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_SCHARR_XY_3X3_08U[] = {{14, 4, 4, 0, {1, 1, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_SCHARR_XY_3X3_08U[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_SCHARR_XY_3X3_08U[] = {{18, 4, 4, 0, {1, 1, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_SCHARR_XY_3X3_08U[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_SCHARR_XY_3X3_08U[] = {{22, 4, 4, 0, {1, 1, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_SCHARR_XY_3X3_08U[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_SCHARR_XY_3X3_08U[] = {{26, 4, 4, 0, {1, 1, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_SCHARR_XY_3X3_08U[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_SCHARR_XY_3X3_08U[] = {{30, 4, 4, 0, {1, 1, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_SCHARR_XY_3X3_08U[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_SCHARR_XY_3X3_08U[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_SCHARR_XY_3X3_08U[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_SCHARR_XY_3X3_08U[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_SCHARR_XY_3X3_08U[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_SCHARR_XY_3X3_08U[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_SCHARR_XY_3X3_08U[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_SCHARR_XY_3X3_08U[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_SCHARR_XY_3X3_08U[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_SCHARR_XY_3X3_08U[] = {{2, 5, 4, 0, {1, 1, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_SCHARR_XY_3X3_08U[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_SCHARR_XY_3X3_08U[] = {{6, 5, 4, 0, {1, 1, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_SCHARR_XY_3X3_08U[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_SCHARR_XY_3X3_08U[] = {{10, 5, 4, 0, {1, 1, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_SCHARR_XY_3X3_08U[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_SCHARR_XY_3X3_08U[] = {{14, 5, 4, 0, {1, 1, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_SCHARR_XY_3X3_08U[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_SCHARR_XY_3X3_08U[] = {{18, 5, 4, 0, {1, 1, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_SCHARR_XY_3X3_08U[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_SCHARR_XY_3X3_08U[] = {{22, 5, 4, 0, {1, 1, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_SCHARR_XY_3X3_08U[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_SCHARR_XY_3X3_08U[] = {{26, 5, 4, 0, {1, 1, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_SCHARR_XY_3X3_08U[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_SCHARR_XY_3X3_08U[] = {{30, 5, 4, 0, {1, 1, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_SCHARR_XY_3X3_08U[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_SCHARR_XY_3X3_08U[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_SCHARR_XY_3X3_08U[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_SCHARR_XY_3X3_08U[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_SCHARR_XY_3X3_08U[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_SCHARR_XY_3X3_08U[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_SCHARR_XY_3X3_08U[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_SCHARR_XY_3X3_08U[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_SCHARR_XY_3X3_08U[] = {{2, 6, 4, 0, {1, 1, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_SCHARR_XY_3X3_08U[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_SCHARR_XY_3X3_08U[] = {{6, 6, 4, 0, {1, 1, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_SCHARR_XY_3X3_08U[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_SCHARR_XY_3X3_08U[] = {{10, 6, 4, 0, {1, 1, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_SCHARR_XY_3X3_08U[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_SCHARR_XY_3X3_08U[] = {{14, 6, 4, 0, {1, 1, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_SCHARR_XY_3X3_08U[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_SCHARR_XY_3X3_08U[] = {{18, 6, 4, 0, {1, 1, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_SCHARR_XY_3X3_08U[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_SCHARR_XY_3X3_08U[] = {{22, 6, 4, 0, {1, 1, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_SCHARR_XY_3X3_08U[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_SCHARR_XY_3X3_08U[] = {{26, 6, 4, 0, {1, 1, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_SCHARR_XY_3X3_08U[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_SCHARR_XY_3X3_08U[] = {{30, 6, 4, 0, {1, 1, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_SCHARR_XY_3X3_08U[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_SCHARR_XY_3X3_08U[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_SCHARR_XY_3X3_08U[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_SCHARR_XY_3X3_08U[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_SCHARR_XY_3X3_08U[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_SCHARR_XY_3X3_08U[] = {{2, 8, 4, 0, {1, 1, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_SCHARR_XY_3X3_08U[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_SCHARR_XY_3X3_08U[] = {{6, 8, 4, 0, {1, 1, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_SCHARR_XY_3X3_08U[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_SCHARR_XY_3X3_08U[] = {{10, 8, 4, 0, {1, 1, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_SCHARR_XY_3X3_08U[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_SCHARR_XY_3X3_08U[] = {{14, 8, 4, 0, {1, 1, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_SCHARR_XY_3X3_08U[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_SCHARR_XY_3X3_08U[] = {{18, 8, 4, 0, {1, 1, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_SCHARR_XY_3X3_08U[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_SCHARR_XY_3X3_08U[] = {{22, 8, 4, 0, {1, 1, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_SCHARR_XY_3X3_08U[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_SCHARR_XY_3X3_08U[] = {{26, 8, 4, 0, {1, 1, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_SCHARR_XY_3X3_08U[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_SCHARR_XY_3X3_08U[] = {{30, 8, 4, 0, {1, 1, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_SCHARR_XY_3X3_08U[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_SCHARR_XY_3X3_08U[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_SCHARR_XY_3X3_08U[] = {{2, 10, 4, 0, {1, 1, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_SCHARR_XY_3X3_08U[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_SCHARR_XY_3X3_08U[] = {{6, 10, 4, 0, {1, 1, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_SCHARR_XY_3X3_08U[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_SCHARR_XY_3X3_08U[] = {{10, 10, 4, 0, {1, 1, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_SCHARR_XY_3X3_08U[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_SCHARR_XY_3X3_08U[] = {{14, 10, 4, 0, {1, 1, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_SCHARR_XY_3X3_08U[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_SCHARR_XY_3X3_08U[] = {{18, 10, 4, 0, {1, 1, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_SCHARR_XY_3X3_08U[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_SCHARR_XY_3X3_08U[] = {{22, 10, 4, 0, {1, 1, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_SCHARR_XY_3X3_08U[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_SCHARR_XY_3X3_08U[] = {{26, 10, 4, 0, {1, 1, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_SCHARR_XY_3X3_08U[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_SCHARR_XY_3X3_08U[] = {{30, 10, 4, 0, {1, 1, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_SCHARR_XY_3X3_08U[] = {{2, 12, 4, 0, {1, 1, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_SCHARR_XY_3X3_08U[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_SCHARR_XY_3X3_08U[] = {{6, 12, 4, 0, {1, 1, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_SCHARR_XY_3X3_08U[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_SCHARR_XY_3X3_08U[] = {{10, 12, 4, 0, {1, 1, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_SCHARR_XY_3X3_08U[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_SCHARR_XY_3X3_08U[] = {{14, 12, 4, 0, {1, 1, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_SCHARR_XY_3X3_08U[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_SCHARR_XY_3X3_08U[] = {{18, 12, 4, 0, {1, 1, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_SCHARR_XY_3X3_08U[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_SCHARR_XY_3X3_08U[] = {{22, 12, 4, 0, {1, 1, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_SCHARR_XY_3X3_08U[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_SCHARR_XY_3X3_08U[] = {{2, 14, 4, 0, {1, 1, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_SCHARR_XY_3X3_08U[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_SCHARR_XY_3X3_08U[] = {{6, 14, 4, 0, {1, 1, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_SCHARR_XY_3X3_08U[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_SCHARR_XY_3X3_08U[] = {{10, 14, 4, 0, {1, 1, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_SCHARR_XY_3X3_08U[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_SCHARR_XY_3X3_08U[] = {{14, 14, 4, 0, {1, 1, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_SCHARR_XY_3X3_08U[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_SCHARR_XY_3X3_08U[] = {{18, 14, 4, 0, {1, 1, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_SCHARR_XY_3X3_08U[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_SCHARR_XY_3X3_08U[] = {{2, 16, 4, 0, {1, 1, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_SCHARR_XY_3X3_08U[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_SCHARR_XY_3X3_08U[] = {{6, 16, 4, 0, {1, 1, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_SCHARR_XY_3X3_08U[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_SCHARR_XY_3X3_08U[] = {{10, 16, 4, 0, {1, 1, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_SCHARR_XY_3X3_08U[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_SCHARR_XY_3X3_08U[] = {{14, 16, 4, 0, {1, 1, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_SCHARR_XY_3X3_08U[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_SCHARR_XY_3X3_08U[] = {{18, 16, 4, 0, {1, 1, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_SCHARR_XY_3X3_08U[] = {{2, 18, 4, 0, {1, 1, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_SCHARR_XY_3X3_08U[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_SCHARR_XY_3X3_08U[] = {{6, 18, 4, 0, {1, 1, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_SCHARR_XY_3X3_08U[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_SCHARR_XY_3X3_08U[] = {{10, 18, 4, 0, {1, 1, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_SCHARR_XY_3X3_08U[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_SCHARR_XY_3X3_08U[] = {{14, 18, 4, 0, {1, 1, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_SCHARR_XY_3X3_08U[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_SCHARR_XY_3X3_08U[] = {{2, 20, 4, 0, {1, 1, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_SCHARR_XY_3X3_08U[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_SCHARR_XY_3X3_08U[] = {{6, 20, 4, 0, {1, 1, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_SCHARR_XY_3X3_08U[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_SCHARR_XY_3X3_08U[] = {{10, 20, 4, 0, {1, 1, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_SCHARR_XY_3X3_08U[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_SCHARR_XY_3X3_08U[] = {{14, 20, 4, 0, {1, 1, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_SCHARR_XY_3X3_08U[] = {{2, 22, 4, 0, {1, 1, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_SCHARR_XY_3X3_08U[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_SCHARR_XY_3X3_08U[] = {{6, 22, 4, 0, {1, 1, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_SCHARR_XY_3X3_08U[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_SCHARR_XY_3X3_08U[] = {{10, 22, 4, 0, {1, 1, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_SCHARR_XY_3X3_08U[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_SCHARR_XY_3X3_08U[] = {{2, 24, 4, 0, {1, 1, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_SCHARR_XY_3X3_08U[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_SCHARR_XY_3X3_08U[] = {{6, 24, 4, 0, {1, 1, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_SCHARR_XY_3X3_08U[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_SCHARR_XY_3X3_08U[] = {{10, 24, 4, 0, {1, 1, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_SCHARR_XY_3X3_08U[] = {{2, 26, 4, 0, {1, 1, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_SCHARR_XY_3X3_08U[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_SCHARR_XY_3X3_08U[] = {{6, 26, 4, 0, {1, 1, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_SCHARR_XY_3X3_08U[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_SCHARR_XY_3X3_08U[] = {{10, 26, 4, 0, {1, 1, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_SCHARR_XY_3X3_08U[] = {{2, 28, 4, 0, {1, 1, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_SCHARR_XY_3X3_08U[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_SCHARR_XY_3X3_08U[] = {{6, 28, 4, 0, {1, 1, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_SCHARR_XY_3X3_08U[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_SCHARR_XY_3X3_08U[] = {{10, 28, 4, 0, {1, 1, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_SCHARR_XY_3X3_08U[] = {{2, 30, 4, 0, {1, 1, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_SCHARR_XY_3X3_08U[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_SCHARR_XY_3X3_08U[] = {{6, 30, 4, 0, {1, 1, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_SCHARR_XY_3X3_08U[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_SCHARR_XY_3X3_08U[] = {{2, 32, 4, 0, {1, 1, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_SCHARR_XY_3X3_08U[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_SCHARR_XY_3X3_08U[] = {{6, 32, 4, 0, {1, 1, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_SCHARR_XY_3X3_08U[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_SCHARR_XY_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_SCHARR_XY_3X3_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_SCHARR_XY_3X3_08U[] = {
{2, 1, 52, 0, 1, gScenarioBufferData0_SCHARR_XY_3X3_08U, 48, gScenarioKernelData0_SCHARR_XY_3X3_08U, 4},
{4, 1, 80, 0, 1, gScenarioBufferData1_SCHARR_XY_3X3_08U, 48, gScenarioKernelData1_SCHARR_XY_3X3_08U, 4},
{6, 1, 108, 0, 1, gScenarioBufferData2_SCHARR_XY_3X3_08U, 48, gScenarioKernelData2_SCHARR_XY_3X3_08U, 4},
{8, 1, 136, 0, 1, gScenarioBufferData3_SCHARR_XY_3X3_08U, 48, gScenarioKernelData3_SCHARR_XY_3X3_08U, 4},
{10, 1, 164, 0, 1, gScenarioBufferData4_SCHARR_XY_3X3_08U, 48, gScenarioKernelData4_SCHARR_XY_3X3_08U, 4},
{12, 1, 192, 0, 1, gScenarioBufferData5_SCHARR_XY_3X3_08U, 48, gScenarioKernelData5_SCHARR_XY_3X3_08U, 4},
{14, 1, 220, 0, 1, gScenarioBufferData6_SCHARR_XY_3X3_08U, 48, gScenarioKernelData6_SCHARR_XY_3X3_08U, 4},
{16, 1, 248, 0, 1, gScenarioBufferData7_SCHARR_XY_3X3_08U, 48, gScenarioKernelData7_SCHARR_XY_3X3_08U, 4},
{18, 1, 276, 0, 1, gScenarioBufferData8_SCHARR_XY_3X3_08U, 48, gScenarioKernelData8_SCHARR_XY_3X3_08U, 4},
{20, 1, 304, 0, 1, gScenarioBufferData9_SCHARR_XY_3X3_08U, 48, gScenarioKernelData9_SCHARR_XY_3X3_08U, 4},
{22, 1, 332, 0, 1, gScenarioBufferData10_SCHARR_XY_3X3_08U, 48, gScenarioKernelData10_SCHARR_XY_3X3_08U, 4},
{24, 1, 360, 0, 1, gScenarioBufferData11_SCHARR_XY_3X3_08U, 48, gScenarioKernelData11_SCHARR_XY_3X3_08U, 4},
{26, 1, 388, 0, 1, gScenarioBufferData12_SCHARR_XY_3X3_08U, 48, gScenarioKernelData12_SCHARR_XY_3X3_08U, 4},
{28, 1, 416, 0, 1, gScenarioBufferData13_SCHARR_XY_3X3_08U, 48, gScenarioKernelData13_SCHARR_XY_3X3_08U, 4},
{30, 1, 444, 0, 1, gScenarioBufferData14_SCHARR_XY_3X3_08U, 48, gScenarioKernelData14_SCHARR_XY_3X3_08U, 4},
{32, 1, 472, 0, 1, gScenarioBufferData15_SCHARR_XY_3X3_08U, 48, gScenarioKernelData15_SCHARR_XY_3X3_08U, 4},
{36, 1, 528, 0, 1, gScenarioBufferData16_SCHARR_XY_3X3_08U, 48, gScenarioKernelData16_SCHARR_XY_3X3_08U, 4},
{40, 1, 584, 0, 1, gScenarioBufferData17_SCHARR_XY_3X3_08U, 48, gScenarioKernelData17_SCHARR_XY_3X3_08U, 4},
{44, 1, 640, 0, 1, gScenarioBufferData18_SCHARR_XY_3X3_08U, 48, gScenarioKernelData18_SCHARR_XY_3X3_08U, 4},
{48, 1, 696, 0, 1, gScenarioBufferData19_SCHARR_XY_3X3_08U, 48, gScenarioKernelData19_SCHARR_XY_3X3_08U, 4},
{52, 1, 752, 0, 1, gScenarioBufferData20_SCHARR_XY_3X3_08U, 48, gScenarioKernelData20_SCHARR_XY_3X3_08U, 4},
{56, 1, 808, 0, 1, gScenarioBufferData21_SCHARR_XY_3X3_08U, 48, gScenarioKernelData21_SCHARR_XY_3X3_08U, 4},
{60, 1, 864, 0, 1, gScenarioBufferData22_SCHARR_XY_3X3_08U, 48, gScenarioKernelData22_SCHARR_XY_3X3_08U, 4},
{64, 1, 920, 0, 1, gScenarioBufferData23_SCHARR_XY_3X3_08U, 48, gScenarioKernelData23_SCHARR_XY_3X3_08U, 4},
{2, 2, 92, 0, 1, gScenarioBufferData24_SCHARR_XY_3X3_08U, 48, gScenarioKernelData24_SCHARR_XY_3X3_08U, 4},
{4, 2, 144, 0, 1, gScenarioBufferData25_SCHARR_XY_3X3_08U, 48, gScenarioKernelData25_SCHARR_XY_3X3_08U, 4},
{6, 2, 196, 0, 1, gScenarioBufferData26_SCHARR_XY_3X3_08U, 48, gScenarioKernelData26_SCHARR_XY_3X3_08U, 4},
{8, 2, 248, 0, 1, gScenarioBufferData27_SCHARR_XY_3X3_08U, 48, gScenarioKernelData27_SCHARR_XY_3X3_08U, 4},
{10, 2, 300, 0, 1, gScenarioBufferData28_SCHARR_XY_3X3_08U, 48, gScenarioKernelData28_SCHARR_XY_3X3_08U, 4},
{12, 2, 352, 0, 1, gScenarioBufferData29_SCHARR_XY_3X3_08U, 48, gScenarioKernelData29_SCHARR_XY_3X3_08U, 4},
{14, 2, 404, 0, 1, gScenarioBufferData30_SCHARR_XY_3X3_08U, 48, gScenarioKernelData30_SCHARR_XY_3X3_08U, 4},
{16, 2, 456, 0, 1, gScenarioBufferData31_SCHARR_XY_3X3_08U, 48, gScenarioKernelData31_SCHARR_XY_3X3_08U, 4},
{18, 2, 508, 0, 1, gScenarioBufferData32_SCHARR_XY_3X3_08U, 48, gScenarioKernelData32_SCHARR_XY_3X3_08U, 4},
{20, 2, 560, 0, 1, gScenarioBufferData33_SCHARR_XY_3X3_08U, 48, gScenarioKernelData33_SCHARR_XY_3X3_08U, 4},
{22, 2, 612, 0, 1, gScenarioBufferData34_SCHARR_XY_3X3_08U, 48, gScenarioKernelData34_SCHARR_XY_3X3_08U, 4},
{24, 2, 664, 0, 1, gScenarioBufferData35_SCHARR_XY_3X3_08U, 48, gScenarioKernelData35_SCHARR_XY_3X3_08U, 4},
{26, 2, 716, 0, 1, gScenarioBufferData36_SCHARR_XY_3X3_08U, 48, gScenarioKernelData36_SCHARR_XY_3X3_08U, 4},
{28, 2, 768, 0, 1, gScenarioBufferData37_SCHARR_XY_3X3_08U, 48, gScenarioKernelData37_SCHARR_XY_3X3_08U, 4},
{30, 2, 820, 0, 1, gScenarioBufferData38_SCHARR_XY_3X3_08U, 48, gScenarioKernelData38_SCHARR_XY_3X3_08U, 4},
{32, 2, 872, 0, 1, gScenarioBufferData39_SCHARR_XY_3X3_08U, 48, gScenarioKernelData39_SCHARR_XY_3X3_08U, 4},
{36, 2, 976, 0, 1, gScenarioBufferData40_SCHARR_XY_3X3_08U, 48, gScenarioKernelData40_SCHARR_XY_3X3_08U, 4},
{40, 2, 1080, 0, 1, gScenarioBufferData41_SCHARR_XY_3X3_08U, 48, gScenarioKernelData41_SCHARR_XY_3X3_08U, 4},
{44, 2, 1184, 0, 1, gScenarioBufferData42_SCHARR_XY_3X3_08U, 48, gScenarioKernelData42_SCHARR_XY_3X3_08U, 4},
{48, 2, 1288, 0, 1, gScenarioBufferData43_SCHARR_XY_3X3_08U, 48, gScenarioKernelData43_SCHARR_XY_3X3_08U, 4},
{52, 2, 1392, 0, 1, gScenarioBufferData44_SCHARR_XY_3X3_08U, 48, gScenarioKernelData44_SCHARR_XY_3X3_08U, 4},
{56, 2, 1496, 0, 1, gScenarioBufferData45_SCHARR_XY_3X3_08U, 48, gScenarioKernelData45_SCHARR_XY_3X3_08U, 4},
{60, 2, 1600, 0, 1, gScenarioBufferData46_SCHARR_XY_3X3_08U, 48, gScenarioKernelData46_SCHARR_XY_3X3_08U, 4},
{64, 2, 1704, 0, 1, gScenarioBufferData47_SCHARR_XY_3X3_08U, 48, gScenarioKernelData47_SCHARR_XY_3X3_08U, 4},
{2, 3, 132, 0, 1, gScenarioBufferData48_SCHARR_XY_3X3_08U, 48, gScenarioKernelData48_SCHARR_XY_3X3_08U, 4},
{4, 3, 208, 0, 1, gScenarioBufferData49_SCHARR_XY_3X3_08U, 48, gScenarioKernelData49_SCHARR_XY_3X3_08U, 4},
{6, 3, 284, 0, 1, gScenarioBufferData50_SCHARR_XY_3X3_08U, 48, gScenarioKernelData50_SCHARR_XY_3X3_08U, 4},
{8, 3, 360, 0, 1, gScenarioBufferData51_SCHARR_XY_3X3_08U, 48, gScenarioKernelData51_SCHARR_XY_3X3_08U, 4},
{10, 3, 436, 0, 1, gScenarioBufferData52_SCHARR_XY_3X3_08U, 48, gScenarioKernelData52_SCHARR_XY_3X3_08U, 4},
{12, 3, 512, 0, 1, gScenarioBufferData53_SCHARR_XY_3X3_08U, 48, gScenarioKernelData53_SCHARR_XY_3X3_08U, 4},
{14, 3, 588, 0, 1, gScenarioBufferData54_SCHARR_XY_3X3_08U, 48, gScenarioKernelData54_SCHARR_XY_3X3_08U, 4},
{16, 3, 664, 0, 1, gScenarioBufferData55_SCHARR_XY_3X3_08U, 48, gScenarioKernelData55_SCHARR_XY_3X3_08U, 4},
{18, 3, 740, 0, 1, gScenarioBufferData56_SCHARR_XY_3X3_08U, 48, gScenarioKernelData56_SCHARR_XY_3X3_08U, 4},
{20, 3, 816, 0, 1, gScenarioBufferData57_SCHARR_XY_3X3_08U, 48, gScenarioKernelData57_SCHARR_XY_3X3_08U, 4},
{22, 3, 892, 0, 1, gScenarioBufferData58_SCHARR_XY_3X3_08U, 48, gScenarioKernelData58_SCHARR_XY_3X3_08U, 4},
{24, 3, 968, 0, 1, gScenarioBufferData59_SCHARR_XY_3X3_08U, 48, gScenarioKernelData59_SCHARR_XY_3X3_08U, 4},
{26, 3, 1044, 0, 1, gScenarioBufferData60_SCHARR_XY_3X3_08U, 48, gScenarioKernelData60_SCHARR_XY_3X3_08U, 4},
{28, 3, 1120, 0, 1, gScenarioBufferData61_SCHARR_XY_3X3_08U, 48, gScenarioKernelData61_SCHARR_XY_3X3_08U, 4},
{30, 3, 1196, 0, 1, gScenarioBufferData62_SCHARR_XY_3X3_08U, 48, gScenarioKernelData62_SCHARR_XY_3X3_08U, 4},
{32, 3, 1272, 0, 1, gScenarioBufferData63_SCHARR_XY_3X3_08U, 48, gScenarioKernelData63_SCHARR_XY_3X3_08U, 4},
{36, 3, 1424, 0, 1, gScenarioBufferData64_SCHARR_XY_3X3_08U, 48, gScenarioKernelData64_SCHARR_XY_3X3_08U, 4},
{40, 3, 1576, 0, 1, gScenarioBufferData65_SCHARR_XY_3X3_08U, 48, gScenarioKernelData65_SCHARR_XY_3X3_08U, 4},
{44, 3, 1728, 0, 1, gScenarioBufferData66_SCHARR_XY_3X3_08U, 48, gScenarioKernelData66_SCHARR_XY_3X3_08U, 4},
{48, 3, 1880, 0, 1, gScenarioBufferData67_SCHARR_XY_3X3_08U, 48, gScenarioKernelData67_SCHARR_XY_3X3_08U, 4},
{52, 3, 2032, 0, 1, gScenarioBufferData68_SCHARR_XY_3X3_08U, 48, gScenarioKernelData68_SCHARR_XY_3X3_08U, 4},
{56, 3, 2184, 0, 1, gScenarioBufferData69_SCHARR_XY_3X3_08U, 48, gScenarioKernelData69_SCHARR_XY_3X3_08U, 4},
{60, 3, 2336, 0, 1, gScenarioBufferData70_SCHARR_XY_3X3_08U, 48, gScenarioKernelData70_SCHARR_XY_3X3_08U, 4},
{64, 3, 2488, 0, 1, gScenarioBufferData71_SCHARR_XY_3X3_08U, 48, gScenarioKernelData71_SCHARR_XY_3X3_08U, 4},
{2, 4, 172, 0, 1, gScenarioBufferData72_SCHARR_XY_3X3_08U, 48, gScenarioKernelData72_SCHARR_XY_3X3_08U, 4},
{4, 4, 272, 0, 1, gScenarioBufferData73_SCHARR_XY_3X3_08U, 48, gScenarioKernelData73_SCHARR_XY_3X3_08U, 4},
{6, 4, 372, 0, 1, gScenarioBufferData74_SCHARR_XY_3X3_08U, 48, gScenarioKernelData74_SCHARR_XY_3X3_08U, 4},
{8, 4, 472, 0, 1, gScenarioBufferData75_SCHARR_XY_3X3_08U, 48, gScenarioKernelData75_SCHARR_XY_3X3_08U, 4},
{10, 4, 572, 0, 1, gScenarioBufferData76_SCHARR_XY_3X3_08U, 48, gScenarioKernelData76_SCHARR_XY_3X3_08U, 4},
{12, 4, 672, 0, 1, gScenarioBufferData77_SCHARR_XY_3X3_08U, 48, gScenarioKernelData77_SCHARR_XY_3X3_08U, 4},
{14, 4, 772, 0, 1, gScenarioBufferData78_SCHARR_XY_3X3_08U, 48, gScenarioKernelData78_SCHARR_XY_3X3_08U, 4},
{16, 4, 872, 0, 1, gScenarioBufferData79_SCHARR_XY_3X3_08U, 48, gScenarioKernelData79_SCHARR_XY_3X3_08U, 4},
{18, 4, 972, 0, 1, gScenarioBufferData80_SCHARR_XY_3X3_08U, 48, gScenarioKernelData80_SCHARR_XY_3X3_08U, 4},
{20, 4, 1072, 0, 1, gScenarioBufferData81_SCHARR_XY_3X3_08U, 48, gScenarioKernelData81_SCHARR_XY_3X3_08U, 4},
{22, 4, 1172, 0, 1, gScenarioBufferData82_SCHARR_XY_3X3_08U, 48, gScenarioKernelData82_SCHARR_XY_3X3_08U, 4},
{24, 4, 1272, 0, 1, gScenarioBufferData83_SCHARR_XY_3X3_08U, 48, gScenarioKernelData83_SCHARR_XY_3X3_08U, 4},
{26, 4, 1372, 0, 1, gScenarioBufferData84_SCHARR_XY_3X3_08U, 48, gScenarioKernelData84_SCHARR_XY_3X3_08U, 4},
{28, 4, 1472, 0, 1, gScenarioBufferData85_SCHARR_XY_3X3_08U, 48, gScenarioKernelData85_SCHARR_XY_3X3_08U, 4},
{30, 4, 1572, 0, 1, gScenarioBufferData86_SCHARR_XY_3X3_08U, 48, gScenarioKernelData86_SCHARR_XY_3X3_08U, 4},
{32, 4, 1672, 0, 1, gScenarioBufferData87_SCHARR_XY_3X3_08U, 48, gScenarioKernelData87_SCHARR_XY_3X3_08U, 4},
{36, 4, 1872, 0, 1, gScenarioBufferData88_SCHARR_XY_3X3_08U, 48, gScenarioKernelData88_SCHARR_XY_3X3_08U, 4},
{40, 4, 2072, 0, 1, gScenarioBufferData89_SCHARR_XY_3X3_08U, 48, gScenarioKernelData89_SCHARR_XY_3X3_08U, 4},
{44, 4, 2272, 0, 1, gScenarioBufferData90_SCHARR_XY_3X3_08U, 48, gScenarioKernelData90_SCHARR_XY_3X3_08U, 4},
{48, 4, 2472, 0, 1, gScenarioBufferData91_SCHARR_XY_3X3_08U, 48, gScenarioKernelData91_SCHARR_XY_3X3_08U, 4},
{52, 4, 2672, 0, 1, gScenarioBufferData92_SCHARR_XY_3X3_08U, 48, gScenarioKernelData92_SCHARR_XY_3X3_08U, 4},
{56, 4, 2872, 0, 1, gScenarioBufferData93_SCHARR_XY_3X3_08U, 48, gScenarioKernelData93_SCHARR_XY_3X3_08U, 4},
{60, 4, 3072, 0, 1, gScenarioBufferData94_SCHARR_XY_3X3_08U, 48, gScenarioKernelData94_SCHARR_XY_3X3_08U, 4},
{64, 4, 3272, 0, 1, gScenarioBufferData95_SCHARR_XY_3X3_08U, 48, gScenarioKernelData95_SCHARR_XY_3X3_08U, 4},
{2, 5, 212, 0, 1, gScenarioBufferData96_SCHARR_XY_3X3_08U, 48, gScenarioKernelData96_SCHARR_XY_3X3_08U, 4},
{4, 5, 336, 0, 1, gScenarioBufferData97_SCHARR_XY_3X3_08U, 48, gScenarioKernelData97_SCHARR_XY_3X3_08U, 4},
{6, 5, 460, 0, 1, gScenarioBufferData98_SCHARR_XY_3X3_08U, 48, gScenarioKernelData98_SCHARR_XY_3X3_08U, 4},
{8, 5, 584, 0, 1, gScenarioBufferData99_SCHARR_XY_3X3_08U, 48, gScenarioKernelData99_SCHARR_XY_3X3_08U, 4},
{10, 5, 708, 0, 1, gScenarioBufferData100_SCHARR_XY_3X3_08U, 48, gScenarioKernelData100_SCHARR_XY_3X3_08U, 4},
{12, 5, 832, 0, 1, gScenarioBufferData101_SCHARR_XY_3X3_08U, 48, gScenarioKernelData101_SCHARR_XY_3X3_08U, 4},
{14, 5, 956, 0, 1, gScenarioBufferData102_SCHARR_XY_3X3_08U, 48, gScenarioKernelData102_SCHARR_XY_3X3_08U, 4},
{16, 5, 1080, 0, 1, gScenarioBufferData103_SCHARR_XY_3X3_08U, 48, gScenarioKernelData103_SCHARR_XY_3X3_08U, 4},
{18, 5, 1204, 0, 1, gScenarioBufferData104_SCHARR_XY_3X3_08U, 48, gScenarioKernelData104_SCHARR_XY_3X3_08U, 4},
{20, 5, 1328, 0, 1, gScenarioBufferData105_SCHARR_XY_3X3_08U, 48, gScenarioKernelData105_SCHARR_XY_3X3_08U, 4},
{22, 5, 1452, 0, 1, gScenarioBufferData106_SCHARR_XY_3X3_08U, 48, gScenarioKernelData106_SCHARR_XY_3X3_08U, 4},
{24, 5, 1576, 0, 1, gScenarioBufferData107_SCHARR_XY_3X3_08U, 48, gScenarioKernelData107_SCHARR_XY_3X3_08U, 4},
{26, 5, 1700, 0, 1, gScenarioBufferData108_SCHARR_XY_3X3_08U, 48, gScenarioKernelData108_SCHARR_XY_3X3_08U, 4},
{28, 5, 1824, 0, 1, gScenarioBufferData109_SCHARR_XY_3X3_08U, 48, gScenarioKernelData109_SCHARR_XY_3X3_08U, 4},
{30, 5, 1948, 0, 1, gScenarioBufferData110_SCHARR_XY_3X3_08U, 48, gScenarioKernelData110_SCHARR_XY_3X3_08U, 4},
{32, 5, 2072, 0, 1, gScenarioBufferData111_SCHARR_XY_3X3_08U, 48, gScenarioKernelData111_SCHARR_XY_3X3_08U, 4},
{36, 5, 2320, 0, 1, gScenarioBufferData112_SCHARR_XY_3X3_08U, 48, gScenarioKernelData112_SCHARR_XY_3X3_08U, 4},
{40, 5, 2568, 0, 1, gScenarioBufferData113_SCHARR_XY_3X3_08U, 48, gScenarioKernelData113_SCHARR_XY_3X3_08U, 4},
{44, 5, 2816, 0, 1, gScenarioBufferData114_SCHARR_XY_3X3_08U, 48, gScenarioKernelData114_SCHARR_XY_3X3_08U, 4},
{48, 5, 3064, 0, 1, gScenarioBufferData115_SCHARR_XY_3X3_08U, 48, gScenarioKernelData115_SCHARR_XY_3X3_08U, 4},
{52, 5, 3312, 0, 1, gScenarioBufferData116_SCHARR_XY_3X3_08U, 48, gScenarioKernelData116_SCHARR_XY_3X3_08U, 4},
{56, 5, 3560, 0, 1, gScenarioBufferData117_SCHARR_XY_3X3_08U, 48, gScenarioKernelData117_SCHARR_XY_3X3_08U, 4},
{60, 5, 3808, 0, 1, gScenarioBufferData118_SCHARR_XY_3X3_08U, 48, gScenarioKernelData118_SCHARR_XY_3X3_08U, 4},
{2, 6, 252, 0, 1, gScenarioBufferData119_SCHARR_XY_3X3_08U, 48, gScenarioKernelData119_SCHARR_XY_3X3_08U, 4},
{4, 6, 400, 0, 1, gScenarioBufferData120_SCHARR_XY_3X3_08U, 48, gScenarioKernelData120_SCHARR_XY_3X3_08U, 4},
{6, 6, 548, 0, 1, gScenarioBufferData121_SCHARR_XY_3X3_08U, 48, gScenarioKernelData121_SCHARR_XY_3X3_08U, 4},
{8, 6, 696, 0, 1, gScenarioBufferData122_SCHARR_XY_3X3_08U, 48, gScenarioKernelData122_SCHARR_XY_3X3_08U, 4},
{10, 6, 844, 0, 1, gScenarioBufferData123_SCHARR_XY_3X3_08U, 48, gScenarioKernelData123_SCHARR_XY_3X3_08U, 4},
{12, 6, 992, 0, 1, gScenarioBufferData124_SCHARR_XY_3X3_08U, 48, gScenarioKernelData124_SCHARR_XY_3X3_08U, 4},
{14, 6, 1140, 0, 1, gScenarioBufferData125_SCHARR_XY_3X3_08U, 48, gScenarioKernelData125_SCHARR_XY_3X3_08U, 4},
{16, 6, 1288, 0, 1, gScenarioBufferData126_SCHARR_XY_3X3_08U, 48, gScenarioKernelData126_SCHARR_XY_3X3_08U, 4},
{18, 6, 1436, 0, 1, gScenarioBufferData127_SCHARR_XY_3X3_08U, 48, gScenarioKernelData127_SCHARR_XY_3X3_08U, 4},
{20, 6, 1584, 0, 1, gScenarioBufferData128_SCHARR_XY_3X3_08U, 48, gScenarioKernelData128_SCHARR_XY_3X3_08U, 4},
{22, 6, 1732, 0, 1, gScenarioBufferData129_SCHARR_XY_3X3_08U, 48, gScenarioKernelData129_SCHARR_XY_3X3_08U, 4},
{24, 6, 1880, 0, 1, gScenarioBufferData130_SCHARR_XY_3X3_08U, 48, gScenarioKernelData130_SCHARR_XY_3X3_08U, 4},
{26, 6, 2028, 0, 1, gScenarioBufferData131_SCHARR_XY_3X3_08U, 48, gScenarioKernelData131_SCHARR_XY_3X3_08U, 4},
{28, 6, 2176, 0, 1, gScenarioBufferData132_SCHARR_XY_3X3_08U, 48, gScenarioKernelData132_SCHARR_XY_3X3_08U, 4},
{30, 6, 2324, 0, 1, gScenarioBufferData133_SCHARR_XY_3X3_08U, 48, gScenarioKernelData133_SCHARR_XY_3X3_08U, 4},
{32, 6, 2472, 0, 1, gScenarioBufferData134_SCHARR_XY_3X3_08U, 48, gScenarioKernelData134_SCHARR_XY_3X3_08U, 4},
{36, 6, 2768, 0, 1, gScenarioBufferData135_SCHARR_XY_3X3_08U, 48, gScenarioKernelData135_SCHARR_XY_3X3_08U, 4},
{40, 6, 3064, 0, 1, gScenarioBufferData136_SCHARR_XY_3X3_08U, 48, gScenarioKernelData136_SCHARR_XY_3X3_08U, 4},
{44, 6, 3360, 0, 1, gScenarioBufferData137_SCHARR_XY_3X3_08U, 48, gScenarioKernelData137_SCHARR_XY_3X3_08U, 4},
{48, 6, 3656, 0, 1, gScenarioBufferData138_SCHARR_XY_3X3_08U, 48, gScenarioKernelData138_SCHARR_XY_3X3_08U, 4},
{2, 8, 332, 0, 1, gScenarioBufferData139_SCHARR_XY_3X3_08U, 48, gScenarioKernelData139_SCHARR_XY_3X3_08U, 4},
{4, 8, 528, 0, 1, gScenarioBufferData140_SCHARR_XY_3X3_08U, 48, gScenarioKernelData140_SCHARR_XY_3X3_08U, 4},
{6, 8, 724, 0, 1, gScenarioBufferData141_SCHARR_XY_3X3_08U, 48, gScenarioKernelData141_SCHARR_XY_3X3_08U, 4},
{8, 8, 920, 0, 1, gScenarioBufferData142_SCHARR_XY_3X3_08U, 48, gScenarioKernelData142_SCHARR_XY_3X3_08U, 4},
{10, 8, 1116, 0, 1, gScenarioBufferData143_SCHARR_XY_3X3_08U, 48, gScenarioKernelData143_SCHARR_XY_3X3_08U, 4},
{12, 8, 1312, 0, 1, gScenarioBufferData144_SCHARR_XY_3X3_08U, 48, gScenarioKernelData144_SCHARR_XY_3X3_08U, 4},
{14, 8, 1508, 0, 1, gScenarioBufferData145_SCHARR_XY_3X3_08U, 48, gScenarioKernelData145_SCHARR_XY_3X3_08U, 4},
{16, 8, 1704, 0, 1, gScenarioBufferData146_SCHARR_XY_3X3_08U, 48, gScenarioKernelData146_SCHARR_XY_3X3_08U, 4},
{18, 8, 1900, 0, 1, gScenarioBufferData147_SCHARR_XY_3X3_08U, 48, gScenarioKernelData147_SCHARR_XY_3X3_08U, 4},
{20, 8, 2096, 0, 1, gScenarioBufferData148_SCHARR_XY_3X3_08U, 48, gScenarioKernelData148_SCHARR_XY_3X3_08U, 4},
{22, 8, 2292, 0, 1, gScenarioBufferData149_SCHARR_XY_3X3_08U, 48, gScenarioKernelData149_SCHARR_XY_3X3_08U, 4},
{24, 8, 2488, 0, 1, gScenarioBufferData150_SCHARR_XY_3X3_08U, 48, gScenarioKernelData150_SCHARR_XY_3X3_08U, 4},
{26, 8, 2684, 0, 1, gScenarioBufferData151_SCHARR_XY_3X3_08U, 48, gScenarioKernelData151_SCHARR_XY_3X3_08U, 4},
{28, 8, 2880, 0, 1, gScenarioBufferData152_SCHARR_XY_3X3_08U, 48, gScenarioKernelData152_SCHARR_XY_3X3_08U, 4},
{30, 8, 3076, 0, 1, gScenarioBufferData153_SCHARR_XY_3X3_08U, 48, gScenarioKernelData153_SCHARR_XY_3X3_08U, 4},
{32, 8, 3272, 0, 1, gScenarioBufferData154_SCHARR_XY_3X3_08U, 48, gScenarioKernelData154_SCHARR_XY_3X3_08U, 4},
{36, 8, 3664, 0, 1, gScenarioBufferData155_SCHARR_XY_3X3_08U, 48, gScenarioKernelData155_SCHARR_XY_3X3_08U, 4},
{2, 10, 412, 0, 1, gScenarioBufferData156_SCHARR_XY_3X3_08U, 48, gScenarioKernelData156_SCHARR_XY_3X3_08U, 4},
{4, 10, 656, 0, 1, gScenarioBufferData157_SCHARR_XY_3X3_08U, 48, gScenarioKernelData157_SCHARR_XY_3X3_08U, 4},
{6, 10, 900, 0, 1, gScenarioBufferData158_SCHARR_XY_3X3_08U, 48, gScenarioKernelData158_SCHARR_XY_3X3_08U, 4},
{8, 10, 1144, 0, 1, gScenarioBufferData159_SCHARR_XY_3X3_08U, 48, gScenarioKernelData159_SCHARR_XY_3X3_08U, 4},
{10, 10, 1388, 0, 1, gScenarioBufferData160_SCHARR_XY_3X3_08U, 48, gScenarioKernelData160_SCHARR_XY_3X3_08U, 4},
{12, 10, 1632, 0, 1, gScenarioBufferData161_SCHARR_XY_3X3_08U, 48, gScenarioKernelData161_SCHARR_XY_3X3_08U, 4},
{14, 10, 1876, 0, 1, gScenarioBufferData162_SCHARR_XY_3X3_08U, 48, gScenarioKernelData162_SCHARR_XY_3X3_08U, 4},
{16, 10, 2120, 0, 1, gScenarioBufferData163_SCHARR_XY_3X3_08U, 48, gScenarioKernelData163_SCHARR_XY_3X3_08U, 4},
{18, 10, 2364, 0, 1, gScenarioBufferData164_SCHARR_XY_3X3_08U, 48, gScenarioKernelData164_SCHARR_XY_3X3_08U, 4},
{20, 10, 2608, 0, 1, gScenarioBufferData165_SCHARR_XY_3X3_08U, 48, gScenarioKernelData165_SCHARR_XY_3X3_08U, 4},
{22, 10, 2852, 0, 1, gScenarioBufferData166_SCHARR_XY_3X3_08U, 48, gScenarioKernelData166_SCHARR_XY_3X3_08U, 4},
{24, 10, 3096, 0, 1, gScenarioBufferData167_SCHARR_XY_3X3_08U, 48, gScenarioKernelData167_SCHARR_XY_3X3_08U, 4},
{26, 10, 3340, 0, 1, gScenarioBufferData168_SCHARR_XY_3X3_08U, 48, gScenarioKernelData168_SCHARR_XY_3X3_08U, 4},
{28, 10, 3584, 0, 1, gScenarioBufferData169_SCHARR_XY_3X3_08U, 48, gScenarioKernelData169_SCHARR_XY_3X3_08U, 4},
{30, 10, 3828, 0, 1, gScenarioBufferData170_SCHARR_XY_3X3_08U, 48, gScenarioKernelData170_SCHARR_XY_3X3_08U, 4},
{2, 12, 492, 0, 1, gScenarioBufferData171_SCHARR_XY_3X3_08U, 48, gScenarioKernelData171_SCHARR_XY_3X3_08U, 4},
{4, 12, 784, 0, 1, gScenarioBufferData172_SCHARR_XY_3X3_08U, 48, gScenarioKernelData172_SCHARR_XY_3X3_08U, 4},
{6, 12, 1076, 0, 1, gScenarioBufferData173_SCHARR_XY_3X3_08U, 48, gScenarioKernelData173_SCHARR_XY_3X3_08U, 4},
{8, 12, 1368, 0, 1, gScenarioBufferData174_SCHARR_XY_3X3_08U, 48, gScenarioKernelData174_SCHARR_XY_3X3_08U, 4},
{10, 12, 1660, 0, 1, gScenarioBufferData175_SCHARR_XY_3X3_08U, 48, gScenarioKernelData175_SCHARR_XY_3X3_08U, 4},
{12, 12, 1952, 0, 1, gScenarioBufferData176_SCHARR_XY_3X3_08U, 48, gScenarioKernelData176_SCHARR_XY_3X3_08U, 4},
{14, 12, 2244, 0, 1, gScenarioBufferData177_SCHARR_XY_3X3_08U, 48, gScenarioKernelData177_SCHARR_XY_3X3_08U, 4},
{16, 12, 2536, 0, 1, gScenarioBufferData178_SCHARR_XY_3X3_08U, 48, gScenarioKernelData178_SCHARR_XY_3X3_08U, 4},
{18, 12, 2828, 0, 1, gScenarioBufferData179_SCHARR_XY_3X3_08U, 48, gScenarioKernelData179_SCHARR_XY_3X3_08U, 4},
{20, 12, 3120, 0, 1, gScenarioBufferData180_SCHARR_XY_3X3_08U, 48, gScenarioKernelData180_SCHARR_XY_3X3_08U, 4},
{22, 12, 3412, 0, 1, gScenarioBufferData181_SCHARR_XY_3X3_08U, 48, gScenarioKernelData181_SCHARR_XY_3X3_08U, 4},
{24, 12, 3704, 0, 1, gScenarioBufferData182_SCHARR_XY_3X3_08U, 48, gScenarioKernelData182_SCHARR_XY_3X3_08U, 4},
{2, 14, 572, 0, 1, gScenarioBufferData183_SCHARR_XY_3X3_08U, 48, gScenarioKernelData183_SCHARR_XY_3X3_08U, 4},
{4, 14, 912, 0, 1, gScenarioBufferData184_SCHARR_XY_3X3_08U, 48, gScenarioKernelData184_SCHARR_XY_3X3_08U, 4},
{6, 14, 1252, 0, 1, gScenarioBufferData185_SCHARR_XY_3X3_08U, 48, gScenarioKernelData185_SCHARR_XY_3X3_08U, 4},
{8, 14, 1592, 0, 1, gScenarioBufferData186_SCHARR_XY_3X3_08U, 48, gScenarioKernelData186_SCHARR_XY_3X3_08U, 4},
{10, 14, 1932, 0, 1, gScenarioBufferData187_SCHARR_XY_3X3_08U, 48, gScenarioKernelData187_SCHARR_XY_3X3_08U, 4},
{12, 14, 2272, 0, 1, gScenarioBufferData188_SCHARR_XY_3X3_08U, 48, gScenarioKernelData188_SCHARR_XY_3X3_08U, 4},
{14, 14, 2612, 0, 1, gScenarioBufferData189_SCHARR_XY_3X3_08U, 48, gScenarioKernelData189_SCHARR_XY_3X3_08U, 4},
{16, 14, 2952, 0, 1, gScenarioBufferData190_SCHARR_XY_3X3_08U, 48, gScenarioKernelData190_SCHARR_XY_3X3_08U, 4},
{18, 14, 3292, 0, 1, gScenarioBufferData191_SCHARR_XY_3X3_08U, 48, gScenarioKernelData191_SCHARR_XY_3X3_08U, 4},
{20, 14, 3632, 0, 1, gScenarioBufferData192_SCHARR_XY_3X3_08U, 48, gScenarioKernelData192_SCHARR_XY_3X3_08U, 4},
{2, 16, 652, 0, 1, gScenarioBufferData193_SCHARR_XY_3X3_08U, 48, gScenarioKernelData193_SCHARR_XY_3X3_08U, 4},
{4, 16, 1040, 0, 1, gScenarioBufferData194_SCHARR_XY_3X3_08U, 48, gScenarioKernelData194_SCHARR_XY_3X3_08U, 4},
{6, 16, 1428, 0, 1, gScenarioBufferData195_SCHARR_XY_3X3_08U, 48, gScenarioKernelData195_SCHARR_XY_3X3_08U, 4},
{8, 16, 1816, 0, 1, gScenarioBufferData196_SCHARR_XY_3X3_08U, 48, gScenarioKernelData196_SCHARR_XY_3X3_08U, 4},
{10, 16, 2204, 0, 1, gScenarioBufferData197_SCHARR_XY_3X3_08U, 48, gScenarioKernelData197_SCHARR_XY_3X3_08U, 4},
{12, 16, 2592, 0, 1, gScenarioBufferData198_SCHARR_XY_3X3_08U, 48, gScenarioKernelData198_SCHARR_XY_3X3_08U, 4},
{14, 16, 2980, 0, 1, gScenarioBufferData199_SCHARR_XY_3X3_08U, 48, gScenarioKernelData199_SCHARR_XY_3X3_08U, 4},
{16, 16, 3368, 0, 1, gScenarioBufferData200_SCHARR_XY_3X3_08U, 48, gScenarioKernelData200_SCHARR_XY_3X3_08U, 4},
{18, 16, 3756, 0, 1, gScenarioBufferData201_SCHARR_XY_3X3_08U, 48, gScenarioKernelData201_SCHARR_XY_3X3_08U, 4},
{2, 18, 732, 0, 1, gScenarioBufferData202_SCHARR_XY_3X3_08U, 48, gScenarioKernelData202_SCHARR_XY_3X3_08U, 4},
{4, 18, 1168, 0, 1, gScenarioBufferData203_SCHARR_XY_3X3_08U, 48, gScenarioKernelData203_SCHARR_XY_3X3_08U, 4},
{6, 18, 1604, 0, 1, gScenarioBufferData204_SCHARR_XY_3X3_08U, 48, gScenarioKernelData204_SCHARR_XY_3X3_08U, 4},
{8, 18, 2040, 0, 1, gScenarioBufferData205_SCHARR_XY_3X3_08U, 48, gScenarioKernelData205_SCHARR_XY_3X3_08U, 4},
{10, 18, 2476, 0, 1, gScenarioBufferData206_SCHARR_XY_3X3_08U, 48, gScenarioKernelData206_SCHARR_XY_3X3_08U, 4},
{12, 18, 2912, 0, 1, gScenarioBufferData207_SCHARR_XY_3X3_08U, 48, gScenarioKernelData207_SCHARR_XY_3X3_08U, 4},
{14, 18, 3348, 0, 1, gScenarioBufferData208_SCHARR_XY_3X3_08U, 48, gScenarioKernelData208_SCHARR_XY_3X3_08U, 4},
{16, 18, 3784, 0, 1, gScenarioBufferData209_SCHARR_XY_3X3_08U, 48, gScenarioKernelData209_SCHARR_XY_3X3_08U, 4},
{2, 20, 812, 0, 1, gScenarioBufferData210_SCHARR_XY_3X3_08U, 48, gScenarioKernelData210_SCHARR_XY_3X3_08U, 4},
{4, 20, 1296, 0, 1, gScenarioBufferData211_SCHARR_XY_3X3_08U, 48, gScenarioKernelData211_SCHARR_XY_3X3_08U, 4},
{6, 20, 1780, 0, 1, gScenarioBufferData212_SCHARR_XY_3X3_08U, 48, gScenarioKernelData212_SCHARR_XY_3X3_08U, 4},
{8, 20, 2264, 0, 1, gScenarioBufferData213_SCHARR_XY_3X3_08U, 48, gScenarioKernelData213_SCHARR_XY_3X3_08U, 4},
{10, 20, 2748, 0, 1, gScenarioBufferData214_SCHARR_XY_3X3_08U, 48, gScenarioKernelData214_SCHARR_XY_3X3_08U, 4},
{12, 20, 3232, 0, 1, gScenarioBufferData215_SCHARR_XY_3X3_08U, 48, gScenarioKernelData215_SCHARR_XY_3X3_08U, 4},
{14, 20, 3716, 0, 1, gScenarioBufferData216_SCHARR_XY_3X3_08U, 48, gScenarioKernelData216_SCHARR_XY_3X3_08U, 4},
{2, 22, 892, 0, 1, gScenarioBufferData217_SCHARR_XY_3X3_08U, 48, gScenarioKernelData217_SCHARR_XY_3X3_08U, 4},
{4, 22, 1424, 0, 1, gScenarioBufferData218_SCHARR_XY_3X3_08U, 48, gScenarioKernelData218_SCHARR_XY_3X3_08U, 4},
{6, 22, 1956, 0, 1, gScenarioBufferData219_SCHARR_XY_3X3_08U, 48, gScenarioKernelData219_SCHARR_XY_3X3_08U, 4},
{8, 22, 2488, 0, 1, gScenarioBufferData220_SCHARR_XY_3X3_08U, 48, gScenarioKernelData220_SCHARR_XY_3X3_08U, 4},
{10, 22, 3020, 0, 1, gScenarioBufferData221_SCHARR_XY_3X3_08U, 48, gScenarioKernelData221_SCHARR_XY_3X3_08U, 4},
{12, 22, 3552, 0, 1, gScenarioBufferData222_SCHARR_XY_3X3_08U, 48, gScenarioKernelData222_SCHARR_XY_3X3_08U, 4},
{2, 24, 972, 0, 1, gScenarioBufferData223_SCHARR_XY_3X3_08U, 48, gScenarioKernelData223_SCHARR_XY_3X3_08U, 4},
{4, 24, 1552, 0, 1, gScenarioBufferData224_SCHARR_XY_3X3_08U, 48, gScenarioKernelData224_SCHARR_XY_3X3_08U, 4},
{6, 24, 2132, 0, 1, gScenarioBufferData225_SCHARR_XY_3X3_08U, 48, gScenarioKernelData225_SCHARR_XY_3X3_08U, 4},
{8, 24, 2712, 0, 1, gScenarioBufferData226_SCHARR_XY_3X3_08U, 48, gScenarioKernelData226_SCHARR_XY_3X3_08U, 4},
{10, 24, 3292, 0, 1, gScenarioBufferData227_SCHARR_XY_3X3_08U, 48, gScenarioKernelData227_SCHARR_XY_3X3_08U, 4},
{2, 26, 1052, 0, 1, gScenarioBufferData228_SCHARR_XY_3X3_08U, 48, gScenarioKernelData228_SCHARR_XY_3X3_08U, 4},
{4, 26, 1680, 0, 1, gScenarioBufferData229_SCHARR_XY_3X3_08U, 48, gScenarioKernelData229_SCHARR_XY_3X3_08U, 4},
{6, 26, 2308, 0, 1, gScenarioBufferData230_SCHARR_XY_3X3_08U, 48, gScenarioKernelData230_SCHARR_XY_3X3_08U, 4},
{8, 26, 2936, 0, 1, gScenarioBufferData231_SCHARR_XY_3X3_08U, 48, gScenarioKernelData231_SCHARR_XY_3X3_08U, 4},
{10, 26, 3564, 0, 1, gScenarioBufferData232_SCHARR_XY_3X3_08U, 48, gScenarioKernelData232_SCHARR_XY_3X3_08U, 4},
{2, 28, 1132, 0, 1, gScenarioBufferData233_SCHARR_XY_3X3_08U, 48, gScenarioKernelData233_SCHARR_XY_3X3_08U, 4},
{4, 28, 1808, 0, 1, gScenarioBufferData234_SCHARR_XY_3X3_08U, 48, gScenarioKernelData234_SCHARR_XY_3X3_08U, 4},
{6, 28, 2484, 0, 1, gScenarioBufferData235_SCHARR_XY_3X3_08U, 48, gScenarioKernelData235_SCHARR_XY_3X3_08U, 4},
{8, 28, 3160, 0, 1, gScenarioBufferData236_SCHARR_XY_3X3_08U, 48, gScenarioKernelData236_SCHARR_XY_3X3_08U, 4},
{10, 28, 3836, 0, 1, gScenarioBufferData237_SCHARR_XY_3X3_08U, 48, gScenarioKernelData237_SCHARR_XY_3X3_08U, 4},
{2, 30, 1212, 0, 1, gScenarioBufferData238_SCHARR_XY_3X3_08U, 48, gScenarioKernelData238_SCHARR_XY_3X3_08U, 4},
{4, 30, 1936, 0, 1, gScenarioBufferData239_SCHARR_XY_3X3_08U, 48, gScenarioKernelData239_SCHARR_XY_3X3_08U, 4},
{6, 30, 2660, 0, 1, gScenarioBufferData240_SCHARR_XY_3X3_08U, 48, gScenarioKernelData240_SCHARR_XY_3X3_08U, 4},
{8, 30, 3384, 0, 1, gScenarioBufferData241_SCHARR_XY_3X3_08U, 48, gScenarioKernelData241_SCHARR_XY_3X3_08U, 4},
{2, 32, 1292, 0, 1, gScenarioBufferData242_SCHARR_XY_3X3_08U, 48, gScenarioKernelData242_SCHARR_XY_3X3_08U, 4},
{4, 32, 2064, 0, 1, gScenarioBufferData243_SCHARR_XY_3X3_08U, 48, gScenarioKernelData243_SCHARR_XY_3X3_08U, 4},
{6, 32, 2836, 0, 1, gScenarioBufferData244_SCHARR_XY_3X3_08U, 48, gScenarioKernelData244_SCHARR_XY_3X3_08U, 4},
{8, 32, 3608, 0, 1, gScenarioBufferData245_SCHARR_XY_3X3_08U, 48, gScenarioKernelData245_SCHARR_XY_3X3_08U, 4}
};

static acf_scenario_list gScenarioList_SCHARR_XY_3X3_08U = {
246, //number of scenarios
gScenarioArray_SCHARR_XY_3X3_08U};
//**************************************************************

class SCHARR_XY_3X3_08U : public ACF_Process_APU
{

public:
   SCHARR_XY_3X3_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("SCHARR_XY_3X3_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("SCHARR_XY_3X3_08U",
                                        SCHARR_XY_3X3_08U_LOAD_SEGMENTS,
                                        SCHARR_XY_3X3_08U_LOAD_PMEM, SCHARR_XY_3X3_08U_LOAD_PMEM_SIZE,
                                        SCHARR_XY_3X3_08U_LOAD_DMEM, SCHARR_XY_3X3_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(SCHARR_XY_3X3_08U_APEX_LOG_BUFFER, SCHARR_XY_3X3_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_1", icp::DATATYPE_16S, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_SCHARR_XY_3X3_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_SCHARR_XY_3X3_08U

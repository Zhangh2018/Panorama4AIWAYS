#ifndef _ACF_PROCESS_APU_CONVOLVE_SCALE_3X3_08U
#define _ACF_PROCESS_APU_CONVOLVE_SCALE_3X3_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CONVOLVE_SCALE_3X3_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CONVOLVE_SCALE_3X3_08U[] = {{2, 1, 4, 0, {1, 1, 2, 2}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CONVOLVE_SCALE_3X3_08U[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CONVOLVE_SCALE_3X3_08U[] = {{6, 1, 4, 0, {1, 1, 2, 2}}, {6, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CONVOLVE_SCALE_3X3_08U[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CONVOLVE_SCALE_3X3_08U[] = {{10, 1, 4, 0, {1, 1, 2, 2}}, {10, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CONVOLVE_SCALE_3X3_08U[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CONVOLVE_SCALE_3X3_08U[] = {{14, 1, 4, 0, {1, 1, 2, 2}}, {14, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CONVOLVE_SCALE_3X3_08U[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CONVOLVE_SCALE_3X3_08U[] = {{18, 1, 4, 0, {1, 1, 2, 2}}, {18, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CONVOLVE_SCALE_3X3_08U[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CONVOLVE_SCALE_3X3_08U[] = {{22, 1, 4, 0, {1, 1, 2, 2}}, {22, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CONVOLVE_SCALE_3X3_08U[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CONVOLVE_SCALE_3X3_08U[] = {{26, 1, 4, 0, {1, 1, 2, 2}}, {26, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CONVOLVE_SCALE_3X3_08U[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CONVOLVE_SCALE_3X3_08U[] = {{30, 1, 4, 0, {1, 1, 2, 2}}, {30, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CONVOLVE_SCALE_3X3_08U[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CONVOLVE_SCALE_3X3_08U[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CONVOLVE_SCALE_3X3_08U[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CONVOLVE_SCALE_3X3_08U[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CONVOLVE_SCALE_3X3_08U[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CONVOLVE_SCALE_3X3_08U[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CONVOLVE_SCALE_3X3_08U[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CONVOLVE_SCALE_3X3_08U[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CONVOLVE_SCALE_3X3_08U[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CONVOLVE_SCALE_3X3_08U[] = {{72, 1, 4, 0, {1, 1, 2, 2}}, {72, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CONVOLVE_SCALE_3X3_08U[] = {{80, 1, 4, 0, {1, 1, 2, 2}}, {80, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CONVOLVE_SCALE_3X3_08U[] = {{88, 1, 4, 0, {1, 1, 2, 2}}, {88, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CONVOLVE_SCALE_3X3_08U[] = {{96, 1, 4, 0, {1, 1, 2, 2}}, {96, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CONVOLVE_SCALE_3X3_08U[] = {{104, 1, 4, 0, {1, 1, 2, 2}}, {104, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CONVOLVE_SCALE_3X3_08U[] = {{112, 1, 4, 0, {1, 1, 2, 2}}, {112, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CONVOLVE_SCALE_3X3_08U[] = {{120, 1, 4, 0, {1, 1, 2, 2}}, {120, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CONVOLVE_SCALE_3X3_08U[] = {{128, 1, 4, 0, {1, 1, 2, 2}}, {128, 1, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CONVOLVE_SCALE_3X3_08U[] = {{2, 2, 4, 0, {1, 1, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CONVOLVE_SCALE_3X3_08U[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CONVOLVE_SCALE_3X3_08U[] = {{6, 2, 4, 0, {1, 1, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CONVOLVE_SCALE_3X3_08U[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CONVOLVE_SCALE_3X3_08U[] = {{10, 2, 4, 0, {1, 1, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CONVOLVE_SCALE_3X3_08U[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CONVOLVE_SCALE_3X3_08U[] = {{14, 2, 4, 0, {1, 1, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CONVOLVE_SCALE_3X3_08U[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CONVOLVE_SCALE_3X3_08U[] = {{18, 2, 4, 0, {1, 1, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CONVOLVE_SCALE_3X3_08U[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CONVOLVE_SCALE_3X3_08U[] = {{22, 2, 4, 0, {1, 1, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CONVOLVE_SCALE_3X3_08U[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CONVOLVE_SCALE_3X3_08U[] = {{26, 2, 4, 0, {1, 1, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CONVOLVE_SCALE_3X3_08U[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CONVOLVE_SCALE_3X3_08U[] = {{30, 2, 4, 0, {1, 1, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CONVOLVE_SCALE_3X3_08U[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CONVOLVE_SCALE_3X3_08U[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CONVOLVE_SCALE_3X3_08U[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CONVOLVE_SCALE_3X3_08U[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CONVOLVE_SCALE_3X3_08U[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CONVOLVE_SCALE_3X3_08U[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CONVOLVE_SCALE_3X3_08U[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CONVOLVE_SCALE_3X3_08U[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CONVOLVE_SCALE_3X3_08U[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CONVOLVE_SCALE_3X3_08U[] = {{72, 2, 4, 0, {1, 1, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CONVOLVE_SCALE_3X3_08U[] = {{80, 2, 4, 0, {1, 1, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CONVOLVE_SCALE_3X3_08U[] = {{88, 2, 4, 0, {1, 1, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CONVOLVE_SCALE_3X3_08U[] = {{96, 2, 4, 0, {1, 1, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CONVOLVE_SCALE_3X3_08U[] = {{104, 2, 4, 0, {1, 1, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CONVOLVE_SCALE_3X3_08U[] = {{112, 2, 4, 0, {1, 1, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CONVOLVE_SCALE_3X3_08U[] = {{120, 2, 4, 0, {1, 1, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CONVOLVE_SCALE_3X3_08U[] = {{128, 2, 4, 0, {1, 1, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CONVOLVE_SCALE_3X3_08U[] = {{2, 3, 4, 0, {1, 1, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CONVOLVE_SCALE_3X3_08U[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CONVOLVE_SCALE_3X3_08U[] = {{6, 3, 4, 0, {1, 1, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CONVOLVE_SCALE_3X3_08U[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CONVOLVE_SCALE_3X3_08U[] = {{10, 3, 4, 0, {1, 1, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CONVOLVE_SCALE_3X3_08U[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CONVOLVE_SCALE_3X3_08U[] = {{14, 3, 4, 0, {1, 1, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CONVOLVE_SCALE_3X3_08U[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CONVOLVE_SCALE_3X3_08U[] = {{18, 3, 4, 0, {1, 1, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CONVOLVE_SCALE_3X3_08U[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CONVOLVE_SCALE_3X3_08U[] = {{22, 3, 4, 0, {1, 1, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CONVOLVE_SCALE_3X3_08U[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CONVOLVE_SCALE_3X3_08U[] = {{26, 3, 4, 0, {1, 1, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CONVOLVE_SCALE_3X3_08U[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CONVOLVE_SCALE_3X3_08U[] = {{30, 3, 4, 0, {1, 1, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CONVOLVE_SCALE_3X3_08U[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CONVOLVE_SCALE_3X3_08U[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CONVOLVE_SCALE_3X3_08U[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CONVOLVE_SCALE_3X3_08U[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CONVOLVE_SCALE_3X3_08U[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CONVOLVE_SCALE_3X3_08U[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CONVOLVE_SCALE_3X3_08U[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CONVOLVE_SCALE_3X3_08U[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CONVOLVE_SCALE_3X3_08U[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CONVOLVE_SCALE_3X3_08U[] = {{72, 3, 4, 0, {1, 1, 2, 2}}, {72, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CONVOLVE_SCALE_3X3_08U[] = {{80, 3, 4, 0, {1, 1, 2, 2}}, {80, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CONVOLVE_SCALE_3X3_08U[] = {{88, 3, 4, 0, {1, 1, 2, 2}}, {88, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CONVOLVE_SCALE_3X3_08U[] = {{96, 3, 4, 0, {1, 1, 2, 2}}, {96, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CONVOLVE_SCALE_3X3_08U[] = {{104, 3, 4, 0, {1, 1, 2, 2}}, {104, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CONVOLVE_SCALE_3X3_08U[] = {{112, 3, 4, 0, {1, 1, 2, 2}}, {112, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CONVOLVE_SCALE_3X3_08U[] = {{120, 3, 4, 0, {1, 1, 2, 2}}, {120, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CONVOLVE_SCALE_3X3_08U[] = {{128, 3, 4, 0, {1, 1, 2, 2}}, {128, 3, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CONVOLVE_SCALE_3X3_08U[] = {{2, 4, 4, 0, {1, 1, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CONVOLVE_SCALE_3X3_08U[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CONVOLVE_SCALE_3X3_08U[] = {{6, 4, 4, 0, {1, 1, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CONVOLVE_SCALE_3X3_08U[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CONVOLVE_SCALE_3X3_08U[] = {{10, 4, 4, 0, {1, 1, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CONVOLVE_SCALE_3X3_08U[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CONVOLVE_SCALE_3X3_08U[] = {{14, 4, 4, 0, {1, 1, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CONVOLVE_SCALE_3X3_08U[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_CONVOLVE_SCALE_3X3_08U[] = {{18, 4, 4, 0, {1, 1, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_CONVOLVE_SCALE_3X3_08U[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_CONVOLVE_SCALE_3X3_08U[] = {{22, 4, 4, 0, {1, 1, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_CONVOLVE_SCALE_3X3_08U[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_CONVOLVE_SCALE_3X3_08U[] = {{26, 4, 4, 0, {1, 1, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_CONVOLVE_SCALE_3X3_08U[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_CONVOLVE_SCALE_3X3_08U[] = {{30, 4, 4, 0, {1, 1, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_CONVOLVE_SCALE_3X3_08U[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_CONVOLVE_SCALE_3X3_08U[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_CONVOLVE_SCALE_3X3_08U[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_CONVOLVE_SCALE_3X3_08U[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_CONVOLVE_SCALE_3X3_08U[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_CONVOLVE_SCALE_3X3_08U[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_CONVOLVE_SCALE_3X3_08U[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_CONVOLVE_SCALE_3X3_08U[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_CONVOLVE_SCALE_3X3_08U[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_CONVOLVE_SCALE_3X3_08U[] = {{72, 4, 4, 0, {1, 1, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_CONVOLVE_SCALE_3X3_08U[] = {{80, 4, 4, 0, {1, 1, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_CONVOLVE_SCALE_3X3_08U[] = {{88, 4, 4, 0, {1, 1, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_CONVOLVE_SCALE_3X3_08U[] = {{96, 4, 4, 0, {1, 1, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_CONVOLVE_SCALE_3X3_08U[] = {{104, 4, 4, 0, {1, 1, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_CONVOLVE_SCALE_3X3_08U[] = {{112, 4, 4, 0, {1, 1, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_CONVOLVE_SCALE_3X3_08U[] = {{120, 4, 4, 0, {1, 1, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_CONVOLVE_SCALE_3X3_08U[] = {{128, 4, 4, 0, {1, 1, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_CONVOLVE_SCALE_3X3_08U[] = {{2, 5, 4, 0, {1, 1, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_CONVOLVE_SCALE_3X3_08U[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_CONVOLVE_SCALE_3X3_08U[] = {{6, 5, 4, 0, {1, 1, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_CONVOLVE_SCALE_3X3_08U[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_CONVOLVE_SCALE_3X3_08U[] = {{10, 5, 4, 0, {1, 1, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_CONVOLVE_SCALE_3X3_08U[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_CONVOLVE_SCALE_3X3_08U[] = {{14, 5, 4, 0, {1, 1, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_CONVOLVE_SCALE_3X3_08U[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_CONVOLVE_SCALE_3X3_08U[] = {{18, 5, 4, 0, {1, 1, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_CONVOLVE_SCALE_3X3_08U[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_CONVOLVE_SCALE_3X3_08U[] = {{22, 5, 4, 0, {1, 1, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_CONVOLVE_SCALE_3X3_08U[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_CONVOLVE_SCALE_3X3_08U[] = {{26, 5, 4, 0, {1, 1, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_CONVOLVE_SCALE_3X3_08U[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_CONVOLVE_SCALE_3X3_08U[] = {{30, 5, 4, 0, {1, 1, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_CONVOLVE_SCALE_3X3_08U[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_CONVOLVE_SCALE_3X3_08U[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_CONVOLVE_SCALE_3X3_08U[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_CONVOLVE_SCALE_3X3_08U[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_CONVOLVE_SCALE_3X3_08U[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_CONVOLVE_SCALE_3X3_08U[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_CONVOLVE_SCALE_3X3_08U[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_CONVOLVE_SCALE_3X3_08U[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_CONVOLVE_SCALE_3X3_08U[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_CONVOLVE_SCALE_3X3_08U[] = {{72, 5, 4, 0, {1, 1, 2, 2}}, {72, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_CONVOLVE_SCALE_3X3_08U[] = {{80, 5, 4, 0, {1, 1, 2, 2}}, {80, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_CONVOLVE_SCALE_3X3_08U[] = {{88, 5, 4, 0, {1, 1, 2, 2}}, {88, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_CONVOLVE_SCALE_3X3_08U[] = {{96, 5, 4, 0, {1, 1, 2, 2}}, {96, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_CONVOLVE_SCALE_3X3_08U[] = {{104, 5, 4, 0, {1, 1, 2, 2}}, {104, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_CONVOLVE_SCALE_3X3_08U[] = {{112, 5, 4, 0, {1, 1, 2, 2}}, {112, 5, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_CONVOLVE_SCALE_3X3_08U[] = {{2, 6, 4, 0, {1, 1, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_CONVOLVE_SCALE_3X3_08U[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_CONVOLVE_SCALE_3X3_08U[] = {{6, 6, 4, 0, {1, 1, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_CONVOLVE_SCALE_3X3_08U[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_CONVOLVE_SCALE_3X3_08U[] = {{10, 6, 4, 0, {1, 1, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_CONVOLVE_SCALE_3X3_08U[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_CONVOLVE_SCALE_3X3_08U[] = {{14, 6, 4, 0, {1, 1, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_CONVOLVE_SCALE_3X3_08U[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_CONVOLVE_SCALE_3X3_08U[] = {{18, 6, 4, 0, {1, 1, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_CONVOLVE_SCALE_3X3_08U[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_CONVOLVE_SCALE_3X3_08U[] = {{22, 6, 4, 0, {1, 1, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_CONVOLVE_SCALE_3X3_08U[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_CONVOLVE_SCALE_3X3_08U[] = {{26, 6, 4, 0, {1, 1, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_CONVOLVE_SCALE_3X3_08U[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_CONVOLVE_SCALE_3X3_08U[] = {{30, 6, 4, 0, {1, 1, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_CONVOLVE_SCALE_3X3_08U[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_CONVOLVE_SCALE_3X3_08U[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_CONVOLVE_SCALE_3X3_08U[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_CONVOLVE_SCALE_3X3_08U[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_CONVOLVE_SCALE_3X3_08U[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_CONVOLVE_SCALE_3X3_08U[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_CONVOLVE_SCALE_3X3_08U[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_CONVOLVE_SCALE_3X3_08U[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_CONVOLVE_SCALE_3X3_08U[] = {{64, 6, 4, 0, {1, 1, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_CONVOLVE_SCALE_3X3_08U[] = {{72, 6, 4, 0, {1, 1, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_CONVOLVE_SCALE_3X3_08U[] = {{80, 6, 4, 0, {1, 1, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_CONVOLVE_SCALE_3X3_08U[] = {{88, 6, 4, 0, {1, 1, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_CONVOLVE_SCALE_3X3_08U[] = {{96, 6, 4, 0, {1, 1, 2, 2}}, {96, 6, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_CONVOLVE_SCALE_3X3_08U[] = {{2, 8, 4, 0, {1, 1, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_CONVOLVE_SCALE_3X3_08U[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_CONVOLVE_SCALE_3X3_08U[] = {{6, 8, 4, 0, {1, 1, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_CONVOLVE_SCALE_3X3_08U[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_CONVOLVE_SCALE_3X3_08U[] = {{10, 8, 4, 0, {1, 1, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_CONVOLVE_SCALE_3X3_08U[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_CONVOLVE_SCALE_3X3_08U[] = {{14, 8, 4, 0, {1, 1, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_CONVOLVE_SCALE_3X3_08U[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_CONVOLVE_SCALE_3X3_08U[] = {{18, 8, 4, 0, {1, 1, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_CONVOLVE_SCALE_3X3_08U[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_CONVOLVE_SCALE_3X3_08U[] = {{22, 8, 4, 0, {1, 1, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_CONVOLVE_SCALE_3X3_08U[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_CONVOLVE_SCALE_3X3_08U[] = {{26, 8, 4, 0, {1, 1, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_CONVOLVE_SCALE_3X3_08U[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_CONVOLVE_SCALE_3X3_08U[] = {{30, 8, 4, 0, {1, 1, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_CONVOLVE_SCALE_3X3_08U[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_CONVOLVE_SCALE_3X3_08U[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_CONVOLVE_SCALE_3X3_08U[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_CONVOLVE_SCALE_3X3_08U[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_CONVOLVE_SCALE_3X3_08U[] = {{48, 8, 4, 0, {1, 1, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_CONVOLVE_SCALE_3X3_08U[] = {{52, 8, 4, 0, {1, 1, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_CONVOLVE_SCALE_3X3_08U[] = {{56, 8, 4, 0, {1, 1, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_CONVOLVE_SCALE_3X3_08U[] = {{60, 8, 4, 0, {1, 1, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_CONVOLVE_SCALE_3X3_08U[] = {{64, 8, 4, 0, {1, 1, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_CONVOLVE_SCALE_3X3_08U[] = {{72, 8, 4, 0, {1, 1, 2, 2}}, {72, 8, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_CONVOLVE_SCALE_3X3_08U[] = {{2, 10, 4, 0, {1, 1, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_CONVOLVE_SCALE_3X3_08U[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_CONVOLVE_SCALE_3X3_08U[] = {{6, 10, 4, 0, {1, 1, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_CONVOLVE_SCALE_3X3_08U[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_CONVOLVE_SCALE_3X3_08U[] = {{10, 10, 4, 0, {1, 1, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_CONVOLVE_SCALE_3X3_08U[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_CONVOLVE_SCALE_3X3_08U[] = {{14, 10, 4, 0, {1, 1, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_CONVOLVE_SCALE_3X3_08U[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_CONVOLVE_SCALE_3X3_08U[] = {{18, 10, 4, 0, {1, 1, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_CONVOLVE_SCALE_3X3_08U[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_CONVOLVE_SCALE_3X3_08U[] = {{22, 10, 4, 0, {1, 1, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_CONVOLVE_SCALE_3X3_08U[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_CONVOLVE_SCALE_3X3_08U[] = {{26, 10, 4, 0, {1, 1, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_CONVOLVE_SCALE_3X3_08U[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_CONVOLVE_SCALE_3X3_08U[] = {{30, 10, 4, 0, {1, 1, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_CONVOLVE_SCALE_3X3_08U[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_CONVOLVE_SCALE_3X3_08U[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_CONVOLVE_SCALE_3X3_08U[] = {{40, 10, 4, 0, {1, 1, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_CONVOLVE_SCALE_3X3_08U[] = {{44, 10, 4, 0, {1, 1, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_CONVOLVE_SCALE_3X3_08U[] = {{48, 10, 4, 0, {1, 1, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_CONVOLVE_SCALE_3X3_08U[] = {{52, 10, 4, 0, {1, 1, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_CONVOLVE_SCALE_3X3_08U[] = {{56, 10, 4, 0, {1, 1, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_CONVOLVE_SCALE_3X3_08U[] = {{2, 12, 4, 0, {1, 1, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_CONVOLVE_SCALE_3X3_08U[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_CONVOLVE_SCALE_3X3_08U[] = {{6, 12, 4, 0, {1, 1, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_CONVOLVE_SCALE_3X3_08U[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_CONVOLVE_SCALE_3X3_08U[] = {{10, 12, 4, 0, {1, 1, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_CONVOLVE_SCALE_3X3_08U[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_CONVOLVE_SCALE_3X3_08U[] = {{14, 12, 4, 0, {1, 1, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_CONVOLVE_SCALE_3X3_08U[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_CONVOLVE_SCALE_3X3_08U[] = {{18, 12, 4, 0, {1, 1, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_CONVOLVE_SCALE_3X3_08U[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_CONVOLVE_SCALE_3X3_08U[] = {{22, 12, 4, 0, {1, 1, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_CONVOLVE_SCALE_3X3_08U[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_CONVOLVE_SCALE_3X3_08U[] = {{26, 12, 4, 0, {1, 1, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_CONVOLVE_SCALE_3X3_08U[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_CONVOLVE_SCALE_3X3_08U[] = {{30, 12, 4, 0, {1, 1, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_CONVOLVE_SCALE_3X3_08U[] = {{32, 12, 4, 0, {1, 1, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_CONVOLVE_SCALE_3X3_08U[] = {{36, 12, 4, 0, {1, 1, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_CONVOLVE_SCALE_3X3_08U[] = {{40, 12, 4, 0, {1, 1, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_CONVOLVE_SCALE_3X3_08U[] = {{44, 12, 4, 0, {1, 1, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_CONVOLVE_SCALE_3X3_08U[] = {{48, 12, 4, 0, {1, 1, 2, 2}}, {48, 12, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_CONVOLVE_SCALE_3X3_08U[] = {{2, 14, 4, 0, {1, 1, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_CONVOLVE_SCALE_3X3_08U[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_CONVOLVE_SCALE_3X3_08U[] = {{6, 14, 4, 0, {1, 1, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_CONVOLVE_SCALE_3X3_08U[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_CONVOLVE_SCALE_3X3_08U[] = {{10, 14, 4, 0, {1, 1, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_CONVOLVE_SCALE_3X3_08U[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_CONVOLVE_SCALE_3X3_08U[] = {{14, 14, 4, 0, {1, 1, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_CONVOLVE_SCALE_3X3_08U[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_CONVOLVE_SCALE_3X3_08U[] = {{18, 14, 4, 0, {1, 1, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_CONVOLVE_SCALE_3X3_08U[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_CONVOLVE_SCALE_3X3_08U[] = {{22, 14, 4, 0, {1, 1, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_CONVOLVE_SCALE_3X3_08U[] = {{24, 14, 4, 0, {1, 1, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_CONVOLVE_SCALE_3X3_08U[] = {{26, 14, 4, 0, {1, 1, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_CONVOLVE_SCALE_3X3_08U[] = {{28, 14, 4, 0, {1, 1, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_CONVOLVE_SCALE_3X3_08U[] = {{30, 14, 4, 0, {1, 1, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_CONVOLVE_SCALE_3X3_08U[] = {{32, 14, 4, 0, {1, 1, 2, 2}}, {32, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_CONVOLVE_SCALE_3X3_08U[] = {{36, 14, 4, 0, {1, 1, 2, 2}}, {36, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_CONVOLVE_SCALE_3X3_08U[] = {{40, 14, 4, 0, {1, 1, 2, 2}}, {40, 14, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_CONVOLVE_SCALE_3X3_08U[] = {{2, 16, 4, 0, {1, 1, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_CONVOLVE_SCALE_3X3_08U[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_CONVOLVE_SCALE_3X3_08U[] = {{6, 16, 4, 0, {1, 1, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_CONVOLVE_SCALE_3X3_08U[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_CONVOLVE_SCALE_3X3_08U[] = {{10, 16, 4, 0, {1, 1, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_CONVOLVE_SCALE_3X3_08U[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_CONVOLVE_SCALE_3X3_08U[] = {{14, 16, 4, 0, {1, 1, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_CONVOLVE_SCALE_3X3_08U[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_CONVOLVE_SCALE_3X3_08U[] = {{18, 16, 4, 0, {1, 1, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_CONVOLVE_SCALE_3X3_08U[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_CONVOLVE_SCALE_3X3_08U[] = {{22, 16, 4, 0, {1, 1, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_CONVOLVE_SCALE_3X3_08U[] = {{24, 16, 4, 0, {1, 1, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_CONVOLVE_SCALE_3X3_08U[] = {{26, 16, 4, 0, {1, 1, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_CONVOLVE_SCALE_3X3_08U[] = {{28, 16, 4, 0, {1, 1, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_CONVOLVE_SCALE_3X3_08U[] = {{30, 16, 4, 0, {1, 1, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_CONVOLVE_SCALE_3X3_08U[] = {{32, 16, 4, 0, {1, 1, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_CONVOLVE_SCALE_3X3_08U[] = {{36, 16, 4, 0, {1, 1, 2, 2}}, {36, 16, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_CONVOLVE_SCALE_3X3_08U[] = {{2, 18, 4, 0, {1, 1, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_CONVOLVE_SCALE_3X3_08U[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_CONVOLVE_SCALE_3X3_08U[] = {{6, 18, 4, 0, {1, 1, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_CONVOLVE_SCALE_3X3_08U[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_CONVOLVE_SCALE_3X3_08U[] = {{10, 18, 4, 0, {1, 1, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_CONVOLVE_SCALE_3X3_08U[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_CONVOLVE_SCALE_3X3_08U[] = {{14, 18, 4, 0, {1, 1, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_CONVOLVE_SCALE_3X3_08U[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_CONVOLVE_SCALE_3X3_08U[] = {{18, 18, 4, 0, {1, 1, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_CONVOLVE_SCALE_3X3_08U[] = {{20, 18, 4, 0, {1, 1, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_CONVOLVE_SCALE_3X3_08U[] = {{22, 18, 4, 0, {1, 1, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_CONVOLVE_SCALE_3X3_08U[] = {{24, 18, 4, 0, {1, 1, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_CONVOLVE_SCALE_3X3_08U[] = {{26, 18, 4, 0, {1, 1, 2, 2}}, {26, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_CONVOLVE_SCALE_3X3_08U[] = {{28, 18, 4, 0, {1, 1, 2, 2}}, {28, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_CONVOLVE_SCALE_3X3_08U[] = {{30, 18, 4, 0, {1, 1, 2, 2}}, {30, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_CONVOLVE_SCALE_3X3_08U[] = {{32, 18, 4, 0, {1, 1, 2, 2}}, {32, 18, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_CONVOLVE_SCALE_3X3_08U[] = {{2, 20, 4, 0, {1, 1, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_CONVOLVE_SCALE_3X3_08U[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_CONVOLVE_SCALE_3X3_08U[] = {{6, 20, 4, 0, {1, 1, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_CONVOLVE_SCALE_3X3_08U[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_CONVOLVE_SCALE_3X3_08U[] = {{10, 20, 4, 0, {1, 1, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_CONVOLVE_SCALE_3X3_08U[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_CONVOLVE_SCALE_3X3_08U[] = {{14, 20, 4, 0, {1, 1, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_CONVOLVE_SCALE_3X3_08U[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_CONVOLVE_SCALE_3X3_08U[] = {{18, 20, 4, 0, {1, 1, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_CONVOLVE_SCALE_3X3_08U[] = {{20, 20, 4, 0, {1, 1, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_CONVOLVE_SCALE_3X3_08U[] = {{22, 20, 4, 0, {1, 1, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_CONVOLVE_SCALE_3X3_08U[] = {{24, 20, 4, 0, {1, 1, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_CONVOLVE_SCALE_3X3_08U[] = {{26, 20, 4, 0, {1, 1, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_CONVOLVE_SCALE_3X3_08U[] = {{28, 20, 4, 0, {1, 1, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_CONVOLVE_SCALE_3X3_08U[] = {{2, 22, 4, 0, {1, 1, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_CONVOLVE_SCALE_3X3_08U[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_CONVOLVE_SCALE_3X3_08U[] = {{6, 22, 4, 0, {1, 1, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_CONVOLVE_SCALE_3X3_08U[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_CONVOLVE_SCALE_3X3_08U[] = {{10, 22, 4, 0, {1, 1, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_CONVOLVE_SCALE_3X3_08U[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_CONVOLVE_SCALE_3X3_08U[] = {{14, 22, 4, 0, {1, 1, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_CONVOLVE_SCALE_3X3_08U[] = {{16, 22, 4, 0, {1, 1, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_CONVOLVE_SCALE_3X3_08U[] = {{18, 22, 4, 0, {1, 1, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_CONVOLVE_SCALE_3X3_08U[] = {{20, 22, 4, 0, {1, 1, 2, 2}}, {20, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_CONVOLVE_SCALE_3X3_08U[] = {{22, 22, 4, 0, {1, 1, 2, 2}}, {22, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_CONVOLVE_SCALE_3X3_08U[] = {{24, 22, 4, 0, {1, 1, 2, 2}}, {24, 22, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_CONVOLVE_SCALE_3X3_08U[] = {{2, 24, 4, 0, {1, 1, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_CONVOLVE_SCALE_3X3_08U[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_CONVOLVE_SCALE_3X3_08U[] = {{6, 24, 4, 0, {1, 1, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_CONVOLVE_SCALE_3X3_08U[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_CONVOLVE_SCALE_3X3_08U[] = {{10, 24, 4, 0, {1, 1, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_CONVOLVE_SCALE_3X3_08U[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_CONVOLVE_SCALE_3X3_08U[] = {{14, 24, 4, 0, {1, 1, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_CONVOLVE_SCALE_3X3_08U[] = {{16, 24, 4, 0, {1, 1, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_CONVOLVE_SCALE_3X3_08U[] = {{18, 24, 4, 0, {1, 1, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_CONVOLVE_SCALE_3X3_08U[] = {{20, 24, 4, 0, {1, 1, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_CONVOLVE_SCALE_3X3_08U[] = {{22, 24, 4, 0, {1, 1, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_CONVOLVE_SCALE_3X3_08U[] = {{2, 26, 4, 0, {1, 1, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_CONVOLVE_SCALE_3X3_08U[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_CONVOLVE_SCALE_3X3_08U[] = {{6, 26, 4, 0, {1, 1, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_CONVOLVE_SCALE_3X3_08U[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_CONVOLVE_SCALE_3X3_08U[] = {{10, 26, 4, 0, {1, 1, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_CONVOLVE_SCALE_3X3_08U[] = {{12, 26, 4, 0, {1, 1, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_CONVOLVE_SCALE_3X3_08U[] = {{14, 26, 4, 0, {1, 1, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_CONVOLVE_SCALE_3X3_08U[] = {{16, 26, 4, 0, {1, 1, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_CONVOLVE_SCALE_3X3_08U[] = {{18, 26, 4, 0, {1, 1, 2, 2}}, {18, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_CONVOLVE_SCALE_3X3_08U[] = {{20, 26, 4, 0, {1, 1, 2, 2}}, {20, 26, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_CONVOLVE_SCALE_3X3_08U[] = {{2, 28, 4, 0, {1, 1, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_CONVOLVE_SCALE_3X3_08U[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_CONVOLVE_SCALE_3X3_08U[] = {{6, 28, 4, 0, {1, 1, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_CONVOLVE_SCALE_3X3_08U[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_CONVOLVE_SCALE_3X3_08U[] = {{10, 28, 4, 0, {1, 1, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_CONVOLVE_SCALE_3X3_08U[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_CONVOLVE_SCALE_3X3_08U[] = {{14, 28, 4, 0, {1, 1, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_CONVOLVE_SCALE_3X3_08U[] = {{16, 28, 4, 0, {1, 1, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_CONVOLVE_SCALE_3X3_08U[] = {{18, 28, 4, 0, {1, 1, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_CONVOLVE_SCALE_3X3_08U[] = {{2, 30, 4, 0, {1, 1, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_CONVOLVE_SCALE_3X3_08U[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_CONVOLVE_SCALE_3X3_08U[] = {{6, 30, 4, 0, {1, 1, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_CONVOLVE_SCALE_3X3_08U[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_CONVOLVE_SCALE_3X3_08U[] = {{10, 30, 4, 0, {1, 1, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_CONVOLVE_SCALE_3X3_08U[] = {{12, 30, 4, 0, {1, 1, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_CONVOLVE_SCALE_3X3_08U[] = {{14, 30, 4, 0, {1, 1, 2, 2}}, {14, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_CONVOLVE_SCALE_3X3_08U[] = {{16, 30, 4, 0, {1, 1, 2, 2}}, {16, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_CONVOLVE_SCALE_3X3_08U[] = {{18, 30, 4, 0, {1, 1, 2, 2}}, {18, 30, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_CONVOLVE_SCALE_3X3_08U[] = {{2, 32, 4, 0, {1, 1, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_CONVOLVE_SCALE_3X3_08U[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_CONVOLVE_SCALE_3X3_08U[] = {{6, 32, 4, 0, {1, 1, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_CONVOLVE_SCALE_3X3_08U[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_CONVOLVE_SCALE_3X3_08U[] = {{10, 32, 4, 0, {1, 1, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_CONVOLVE_SCALE_3X3_08U[] = {{12, 32, 4, 0, {1, 1, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_CONVOLVE_SCALE_3X3_08U[] = {{14, 32, 4, 0, {1, 1, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_CONVOLVE_SCALE_3X3_08U[] = {{16, 32, 4, 0, {1, 1, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_CONVOLVE_SCALE_3X3_08U[] = {{1, 0}};

static acf_scenario gScenarioArray_CONVOLVE_SCALE_3X3_08U[] = {
{2, 1, 40, 32, 1, gScenarioBufferData0_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData0_CONVOLVE_SCALE_3X3_08U, 4},
{4, 1, 56, 32, 1, gScenarioBufferData1_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData1_CONVOLVE_SCALE_3X3_08U, 4},
{6, 1, 72, 32, 1, gScenarioBufferData2_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData2_CONVOLVE_SCALE_3X3_08U, 4},
{8, 1, 88, 32, 1, gScenarioBufferData3_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData3_CONVOLVE_SCALE_3X3_08U, 4},
{10, 1, 104, 32, 1, gScenarioBufferData4_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData4_CONVOLVE_SCALE_3X3_08U, 4},
{12, 1, 120, 32, 1, gScenarioBufferData5_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData5_CONVOLVE_SCALE_3X3_08U, 4},
{14, 1, 136, 32, 1, gScenarioBufferData6_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData6_CONVOLVE_SCALE_3X3_08U, 4},
{16, 1, 152, 32, 1, gScenarioBufferData7_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData7_CONVOLVE_SCALE_3X3_08U, 4},
{18, 1, 168, 32, 1, gScenarioBufferData8_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData8_CONVOLVE_SCALE_3X3_08U, 4},
{20, 1, 184, 32, 1, gScenarioBufferData9_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData9_CONVOLVE_SCALE_3X3_08U, 4},
{22, 1, 200, 32, 1, gScenarioBufferData10_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData10_CONVOLVE_SCALE_3X3_08U, 4},
{24, 1, 216, 32, 1, gScenarioBufferData11_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData11_CONVOLVE_SCALE_3X3_08U, 4},
{26, 1, 232, 32, 1, gScenarioBufferData12_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData12_CONVOLVE_SCALE_3X3_08U, 4},
{28, 1, 248, 32, 1, gScenarioBufferData13_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData13_CONVOLVE_SCALE_3X3_08U, 4},
{30, 1, 264, 32, 1, gScenarioBufferData14_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData14_CONVOLVE_SCALE_3X3_08U, 4},
{32, 1, 280, 32, 1, gScenarioBufferData15_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData15_CONVOLVE_SCALE_3X3_08U, 4},
{36, 1, 312, 32, 1, gScenarioBufferData16_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData16_CONVOLVE_SCALE_3X3_08U, 4},
{40, 1, 344, 32, 1, gScenarioBufferData17_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData17_CONVOLVE_SCALE_3X3_08U, 4},
{44, 1, 376, 32, 1, gScenarioBufferData18_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData18_CONVOLVE_SCALE_3X3_08U, 4},
{48, 1, 408, 32, 1, gScenarioBufferData19_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData19_CONVOLVE_SCALE_3X3_08U, 4},
{52, 1, 440, 32, 1, gScenarioBufferData20_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData20_CONVOLVE_SCALE_3X3_08U, 4},
{56, 1, 472, 32, 1, gScenarioBufferData21_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData21_CONVOLVE_SCALE_3X3_08U, 4},
{60, 1, 504, 32, 1, gScenarioBufferData22_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData22_CONVOLVE_SCALE_3X3_08U, 4},
{64, 1, 536, 32, 1, gScenarioBufferData23_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData23_CONVOLVE_SCALE_3X3_08U, 4},
{72, 1, 600, 32, 1, gScenarioBufferData24_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData24_CONVOLVE_SCALE_3X3_08U, 4},
{80, 1, 664, 32, 1, gScenarioBufferData25_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData25_CONVOLVE_SCALE_3X3_08U, 4},
{88, 1, 728, 32, 1, gScenarioBufferData26_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData26_CONVOLVE_SCALE_3X3_08U, 4},
{96, 1, 792, 32, 1, gScenarioBufferData27_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData27_CONVOLVE_SCALE_3X3_08U, 4},
{104, 1, 856, 32, 1, gScenarioBufferData28_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData28_CONVOLVE_SCALE_3X3_08U, 4},
{112, 1, 920, 32, 1, gScenarioBufferData29_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData29_CONVOLVE_SCALE_3X3_08U, 4},
{120, 1, 984, 32, 1, gScenarioBufferData30_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData30_CONVOLVE_SCALE_3X3_08U, 4},
{128, 1, 1048, 32, 1, gScenarioBufferData31_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData31_CONVOLVE_SCALE_3X3_08U, 4},
{2, 2, 68, 32, 1, gScenarioBufferData32_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData32_CONVOLVE_SCALE_3X3_08U, 4},
{4, 2, 96, 32, 1, gScenarioBufferData33_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData33_CONVOLVE_SCALE_3X3_08U, 4},
{6, 2, 124, 32, 1, gScenarioBufferData34_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData34_CONVOLVE_SCALE_3X3_08U, 4},
{8, 2, 152, 32, 1, gScenarioBufferData35_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData35_CONVOLVE_SCALE_3X3_08U, 4},
{10, 2, 180, 32, 1, gScenarioBufferData36_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData36_CONVOLVE_SCALE_3X3_08U, 4},
{12, 2, 208, 32, 1, gScenarioBufferData37_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData37_CONVOLVE_SCALE_3X3_08U, 4},
{14, 2, 236, 32, 1, gScenarioBufferData38_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData38_CONVOLVE_SCALE_3X3_08U, 4},
{16, 2, 264, 32, 1, gScenarioBufferData39_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData39_CONVOLVE_SCALE_3X3_08U, 4},
{18, 2, 292, 32, 1, gScenarioBufferData40_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData40_CONVOLVE_SCALE_3X3_08U, 4},
{20, 2, 320, 32, 1, gScenarioBufferData41_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData41_CONVOLVE_SCALE_3X3_08U, 4},
{22, 2, 348, 32, 1, gScenarioBufferData42_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData42_CONVOLVE_SCALE_3X3_08U, 4},
{24, 2, 376, 32, 1, gScenarioBufferData43_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData43_CONVOLVE_SCALE_3X3_08U, 4},
{26, 2, 404, 32, 1, gScenarioBufferData44_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData44_CONVOLVE_SCALE_3X3_08U, 4},
{28, 2, 432, 32, 1, gScenarioBufferData45_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData45_CONVOLVE_SCALE_3X3_08U, 4},
{30, 2, 460, 32, 1, gScenarioBufferData46_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData46_CONVOLVE_SCALE_3X3_08U, 4},
{32, 2, 488, 32, 1, gScenarioBufferData47_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData47_CONVOLVE_SCALE_3X3_08U, 4},
{36, 2, 544, 32, 1, gScenarioBufferData48_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData48_CONVOLVE_SCALE_3X3_08U, 4},
{40, 2, 600, 32, 1, gScenarioBufferData49_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData49_CONVOLVE_SCALE_3X3_08U, 4},
{44, 2, 656, 32, 1, gScenarioBufferData50_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData50_CONVOLVE_SCALE_3X3_08U, 4},
{48, 2, 712, 32, 1, gScenarioBufferData51_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData51_CONVOLVE_SCALE_3X3_08U, 4},
{52, 2, 768, 32, 1, gScenarioBufferData52_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData52_CONVOLVE_SCALE_3X3_08U, 4},
{56, 2, 824, 32, 1, gScenarioBufferData53_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData53_CONVOLVE_SCALE_3X3_08U, 4},
{60, 2, 880, 32, 1, gScenarioBufferData54_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData54_CONVOLVE_SCALE_3X3_08U, 4},
{64, 2, 936, 32, 1, gScenarioBufferData55_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData55_CONVOLVE_SCALE_3X3_08U, 4},
{72, 2, 1048, 32, 1, gScenarioBufferData56_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData56_CONVOLVE_SCALE_3X3_08U, 4},
{80, 2, 1160, 32, 1, gScenarioBufferData57_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData57_CONVOLVE_SCALE_3X3_08U, 4},
{88, 2, 1272, 32, 1, gScenarioBufferData58_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData58_CONVOLVE_SCALE_3X3_08U, 4},
{96, 2, 1384, 32, 1, gScenarioBufferData59_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData59_CONVOLVE_SCALE_3X3_08U, 4},
{104, 2, 1496, 32, 1, gScenarioBufferData60_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData60_CONVOLVE_SCALE_3X3_08U, 4},
{112, 2, 1608, 32, 1, gScenarioBufferData61_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData61_CONVOLVE_SCALE_3X3_08U, 4},
{120, 2, 1720, 32, 1, gScenarioBufferData62_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData62_CONVOLVE_SCALE_3X3_08U, 4},
{128, 2, 1832, 32, 1, gScenarioBufferData63_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData63_CONVOLVE_SCALE_3X3_08U, 4},
{2, 3, 96, 32, 1, gScenarioBufferData64_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData64_CONVOLVE_SCALE_3X3_08U, 4},
{4, 3, 136, 32, 1, gScenarioBufferData65_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData65_CONVOLVE_SCALE_3X3_08U, 4},
{6, 3, 176, 32, 1, gScenarioBufferData66_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData66_CONVOLVE_SCALE_3X3_08U, 4},
{8, 3, 216, 32, 1, gScenarioBufferData67_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData67_CONVOLVE_SCALE_3X3_08U, 4},
{10, 3, 256, 32, 1, gScenarioBufferData68_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData68_CONVOLVE_SCALE_3X3_08U, 4},
{12, 3, 296, 32, 1, gScenarioBufferData69_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData69_CONVOLVE_SCALE_3X3_08U, 4},
{14, 3, 336, 32, 1, gScenarioBufferData70_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData70_CONVOLVE_SCALE_3X3_08U, 4},
{16, 3, 376, 32, 1, gScenarioBufferData71_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData71_CONVOLVE_SCALE_3X3_08U, 4},
{18, 3, 416, 32, 1, gScenarioBufferData72_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData72_CONVOLVE_SCALE_3X3_08U, 4},
{20, 3, 456, 32, 1, gScenarioBufferData73_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData73_CONVOLVE_SCALE_3X3_08U, 4},
{22, 3, 496, 32, 1, gScenarioBufferData74_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData74_CONVOLVE_SCALE_3X3_08U, 4},
{24, 3, 536, 32, 1, gScenarioBufferData75_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData75_CONVOLVE_SCALE_3X3_08U, 4},
{26, 3, 576, 32, 1, gScenarioBufferData76_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData76_CONVOLVE_SCALE_3X3_08U, 4},
{28, 3, 616, 32, 1, gScenarioBufferData77_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData77_CONVOLVE_SCALE_3X3_08U, 4},
{30, 3, 656, 32, 1, gScenarioBufferData78_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData78_CONVOLVE_SCALE_3X3_08U, 4},
{32, 3, 696, 32, 1, gScenarioBufferData79_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData79_CONVOLVE_SCALE_3X3_08U, 4},
{36, 3, 776, 32, 1, gScenarioBufferData80_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData80_CONVOLVE_SCALE_3X3_08U, 4},
{40, 3, 856, 32, 1, gScenarioBufferData81_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData81_CONVOLVE_SCALE_3X3_08U, 4},
{44, 3, 936, 32, 1, gScenarioBufferData82_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData82_CONVOLVE_SCALE_3X3_08U, 4},
{48, 3, 1016, 32, 1, gScenarioBufferData83_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData83_CONVOLVE_SCALE_3X3_08U, 4},
{52, 3, 1096, 32, 1, gScenarioBufferData84_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData84_CONVOLVE_SCALE_3X3_08U, 4},
{56, 3, 1176, 32, 1, gScenarioBufferData85_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData85_CONVOLVE_SCALE_3X3_08U, 4},
{60, 3, 1256, 32, 1, gScenarioBufferData86_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData86_CONVOLVE_SCALE_3X3_08U, 4},
{64, 3, 1336, 32, 1, gScenarioBufferData87_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData87_CONVOLVE_SCALE_3X3_08U, 4},
{72, 3, 1496, 32, 1, gScenarioBufferData88_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData88_CONVOLVE_SCALE_3X3_08U, 4},
{80, 3, 1656, 32, 1, gScenarioBufferData89_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData89_CONVOLVE_SCALE_3X3_08U, 4},
{88, 3, 1816, 32, 1, gScenarioBufferData90_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData90_CONVOLVE_SCALE_3X3_08U, 4},
{96, 3, 1976, 32, 1, gScenarioBufferData91_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData91_CONVOLVE_SCALE_3X3_08U, 4},
{104, 3, 2136, 32, 1, gScenarioBufferData92_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData92_CONVOLVE_SCALE_3X3_08U, 4},
{112, 3, 2296, 32, 1, gScenarioBufferData93_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData93_CONVOLVE_SCALE_3X3_08U, 4},
{120, 3, 2456, 32, 1, gScenarioBufferData94_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData94_CONVOLVE_SCALE_3X3_08U, 4},
{128, 3, 2616, 32, 1, gScenarioBufferData95_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData95_CONVOLVE_SCALE_3X3_08U, 4},
{2, 4, 124, 32, 1, gScenarioBufferData96_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData96_CONVOLVE_SCALE_3X3_08U, 4},
{4, 4, 176, 32, 1, gScenarioBufferData97_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData97_CONVOLVE_SCALE_3X3_08U, 4},
{6, 4, 228, 32, 1, gScenarioBufferData98_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData98_CONVOLVE_SCALE_3X3_08U, 4},
{8, 4, 280, 32, 1, gScenarioBufferData99_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData99_CONVOLVE_SCALE_3X3_08U, 4},
{10, 4, 332, 32, 1, gScenarioBufferData100_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData100_CONVOLVE_SCALE_3X3_08U, 4},
{12, 4, 384, 32, 1, gScenarioBufferData101_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData101_CONVOLVE_SCALE_3X3_08U, 4},
{14, 4, 436, 32, 1, gScenarioBufferData102_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData102_CONVOLVE_SCALE_3X3_08U, 4},
{16, 4, 488, 32, 1, gScenarioBufferData103_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData103_CONVOLVE_SCALE_3X3_08U, 4},
{18, 4, 540, 32, 1, gScenarioBufferData104_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData104_CONVOLVE_SCALE_3X3_08U, 4},
{20, 4, 592, 32, 1, gScenarioBufferData105_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData105_CONVOLVE_SCALE_3X3_08U, 4},
{22, 4, 644, 32, 1, gScenarioBufferData106_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData106_CONVOLVE_SCALE_3X3_08U, 4},
{24, 4, 696, 32, 1, gScenarioBufferData107_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData107_CONVOLVE_SCALE_3X3_08U, 4},
{26, 4, 748, 32, 1, gScenarioBufferData108_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData108_CONVOLVE_SCALE_3X3_08U, 4},
{28, 4, 800, 32, 1, gScenarioBufferData109_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData109_CONVOLVE_SCALE_3X3_08U, 4},
{30, 4, 852, 32, 1, gScenarioBufferData110_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData110_CONVOLVE_SCALE_3X3_08U, 4},
{32, 4, 904, 32, 1, gScenarioBufferData111_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData111_CONVOLVE_SCALE_3X3_08U, 4},
{36, 4, 1008, 32, 1, gScenarioBufferData112_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData112_CONVOLVE_SCALE_3X3_08U, 4},
{40, 4, 1112, 32, 1, gScenarioBufferData113_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData113_CONVOLVE_SCALE_3X3_08U, 4},
{44, 4, 1216, 32, 1, gScenarioBufferData114_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData114_CONVOLVE_SCALE_3X3_08U, 4},
{48, 4, 1320, 32, 1, gScenarioBufferData115_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData115_CONVOLVE_SCALE_3X3_08U, 4},
{52, 4, 1424, 32, 1, gScenarioBufferData116_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData116_CONVOLVE_SCALE_3X3_08U, 4},
{56, 4, 1528, 32, 1, gScenarioBufferData117_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData117_CONVOLVE_SCALE_3X3_08U, 4},
{60, 4, 1632, 32, 1, gScenarioBufferData118_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData118_CONVOLVE_SCALE_3X3_08U, 4},
{64, 4, 1736, 32, 1, gScenarioBufferData119_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData119_CONVOLVE_SCALE_3X3_08U, 4},
{72, 4, 1944, 32, 1, gScenarioBufferData120_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData120_CONVOLVE_SCALE_3X3_08U, 4},
{80, 4, 2152, 32, 1, gScenarioBufferData121_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData121_CONVOLVE_SCALE_3X3_08U, 4},
{88, 4, 2360, 32, 1, gScenarioBufferData122_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData122_CONVOLVE_SCALE_3X3_08U, 4},
{96, 4, 2568, 32, 1, gScenarioBufferData123_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData123_CONVOLVE_SCALE_3X3_08U, 4},
{104, 4, 2776, 32, 1, gScenarioBufferData124_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData124_CONVOLVE_SCALE_3X3_08U, 4},
{112, 4, 2984, 32, 1, gScenarioBufferData125_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData125_CONVOLVE_SCALE_3X3_08U, 4},
{120, 4, 3192, 32, 1, gScenarioBufferData126_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData126_CONVOLVE_SCALE_3X3_08U, 4},
{128, 4, 3400, 32, 1, gScenarioBufferData127_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData127_CONVOLVE_SCALE_3X3_08U, 4},
{2, 5, 152, 32, 1, gScenarioBufferData128_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData128_CONVOLVE_SCALE_3X3_08U, 4},
{4, 5, 216, 32, 1, gScenarioBufferData129_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData129_CONVOLVE_SCALE_3X3_08U, 4},
{6, 5, 280, 32, 1, gScenarioBufferData130_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData130_CONVOLVE_SCALE_3X3_08U, 4},
{8, 5, 344, 32, 1, gScenarioBufferData131_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData131_CONVOLVE_SCALE_3X3_08U, 4},
{10, 5, 408, 32, 1, gScenarioBufferData132_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData132_CONVOLVE_SCALE_3X3_08U, 4},
{12, 5, 472, 32, 1, gScenarioBufferData133_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData133_CONVOLVE_SCALE_3X3_08U, 4},
{14, 5, 536, 32, 1, gScenarioBufferData134_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData134_CONVOLVE_SCALE_3X3_08U, 4},
{16, 5, 600, 32, 1, gScenarioBufferData135_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData135_CONVOLVE_SCALE_3X3_08U, 4},
{18, 5, 664, 32, 1, gScenarioBufferData136_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData136_CONVOLVE_SCALE_3X3_08U, 4},
{20, 5, 728, 32, 1, gScenarioBufferData137_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData137_CONVOLVE_SCALE_3X3_08U, 4},
{22, 5, 792, 32, 1, gScenarioBufferData138_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData138_CONVOLVE_SCALE_3X3_08U, 4},
{24, 5, 856, 32, 1, gScenarioBufferData139_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData139_CONVOLVE_SCALE_3X3_08U, 4},
{26, 5, 920, 32, 1, gScenarioBufferData140_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData140_CONVOLVE_SCALE_3X3_08U, 4},
{28, 5, 984, 32, 1, gScenarioBufferData141_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData141_CONVOLVE_SCALE_3X3_08U, 4},
{30, 5, 1048, 32, 1, gScenarioBufferData142_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData142_CONVOLVE_SCALE_3X3_08U, 4},
{32, 5, 1112, 32, 1, gScenarioBufferData143_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData143_CONVOLVE_SCALE_3X3_08U, 4},
{36, 5, 1240, 32, 1, gScenarioBufferData144_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData144_CONVOLVE_SCALE_3X3_08U, 4},
{40, 5, 1368, 32, 1, gScenarioBufferData145_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData145_CONVOLVE_SCALE_3X3_08U, 4},
{44, 5, 1496, 32, 1, gScenarioBufferData146_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData146_CONVOLVE_SCALE_3X3_08U, 4},
{48, 5, 1624, 32, 1, gScenarioBufferData147_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData147_CONVOLVE_SCALE_3X3_08U, 4},
{52, 5, 1752, 32, 1, gScenarioBufferData148_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData148_CONVOLVE_SCALE_3X3_08U, 4},
{56, 5, 1880, 32, 1, gScenarioBufferData149_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData149_CONVOLVE_SCALE_3X3_08U, 4},
{60, 5, 2008, 32, 1, gScenarioBufferData150_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData150_CONVOLVE_SCALE_3X3_08U, 4},
{64, 5, 2136, 32, 1, gScenarioBufferData151_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData151_CONVOLVE_SCALE_3X3_08U, 4},
{72, 5, 2392, 32, 1, gScenarioBufferData152_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData152_CONVOLVE_SCALE_3X3_08U, 4},
{80, 5, 2648, 32, 1, gScenarioBufferData153_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData153_CONVOLVE_SCALE_3X3_08U, 4},
{88, 5, 2904, 32, 1, gScenarioBufferData154_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData154_CONVOLVE_SCALE_3X3_08U, 4},
{96, 5, 3160, 32, 1, gScenarioBufferData155_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData155_CONVOLVE_SCALE_3X3_08U, 4},
{104, 5, 3416, 32, 1, gScenarioBufferData156_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData156_CONVOLVE_SCALE_3X3_08U, 4},
{112, 5, 3672, 32, 1, gScenarioBufferData157_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData157_CONVOLVE_SCALE_3X3_08U, 4},
{2, 6, 180, 32, 1, gScenarioBufferData158_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData158_CONVOLVE_SCALE_3X3_08U, 4},
{4, 6, 256, 32, 1, gScenarioBufferData159_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData159_CONVOLVE_SCALE_3X3_08U, 4},
{6, 6, 332, 32, 1, gScenarioBufferData160_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData160_CONVOLVE_SCALE_3X3_08U, 4},
{8, 6, 408, 32, 1, gScenarioBufferData161_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData161_CONVOLVE_SCALE_3X3_08U, 4},
{10, 6, 484, 32, 1, gScenarioBufferData162_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData162_CONVOLVE_SCALE_3X3_08U, 4},
{12, 6, 560, 32, 1, gScenarioBufferData163_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData163_CONVOLVE_SCALE_3X3_08U, 4},
{14, 6, 636, 32, 1, gScenarioBufferData164_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData164_CONVOLVE_SCALE_3X3_08U, 4},
{16, 6, 712, 32, 1, gScenarioBufferData165_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData165_CONVOLVE_SCALE_3X3_08U, 4},
{18, 6, 788, 32, 1, gScenarioBufferData166_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData166_CONVOLVE_SCALE_3X3_08U, 4},
{20, 6, 864, 32, 1, gScenarioBufferData167_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData167_CONVOLVE_SCALE_3X3_08U, 4},
{22, 6, 940, 32, 1, gScenarioBufferData168_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData168_CONVOLVE_SCALE_3X3_08U, 4},
{24, 6, 1016, 32, 1, gScenarioBufferData169_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData169_CONVOLVE_SCALE_3X3_08U, 4},
{26, 6, 1092, 32, 1, gScenarioBufferData170_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData170_CONVOLVE_SCALE_3X3_08U, 4},
{28, 6, 1168, 32, 1, gScenarioBufferData171_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData171_CONVOLVE_SCALE_3X3_08U, 4},
{30, 6, 1244, 32, 1, gScenarioBufferData172_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData172_CONVOLVE_SCALE_3X3_08U, 4},
{32, 6, 1320, 32, 1, gScenarioBufferData173_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData173_CONVOLVE_SCALE_3X3_08U, 4},
{36, 6, 1472, 32, 1, gScenarioBufferData174_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData174_CONVOLVE_SCALE_3X3_08U, 4},
{40, 6, 1624, 32, 1, gScenarioBufferData175_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData175_CONVOLVE_SCALE_3X3_08U, 4},
{44, 6, 1776, 32, 1, gScenarioBufferData176_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData176_CONVOLVE_SCALE_3X3_08U, 4},
{48, 6, 1928, 32, 1, gScenarioBufferData177_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData177_CONVOLVE_SCALE_3X3_08U, 4},
{52, 6, 2080, 32, 1, gScenarioBufferData178_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData178_CONVOLVE_SCALE_3X3_08U, 4},
{56, 6, 2232, 32, 1, gScenarioBufferData179_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData179_CONVOLVE_SCALE_3X3_08U, 4},
{60, 6, 2384, 32, 1, gScenarioBufferData180_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData180_CONVOLVE_SCALE_3X3_08U, 4},
{64, 6, 2536, 32, 1, gScenarioBufferData181_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData181_CONVOLVE_SCALE_3X3_08U, 4},
{72, 6, 2840, 32, 1, gScenarioBufferData182_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData182_CONVOLVE_SCALE_3X3_08U, 4},
{80, 6, 3144, 32, 1, gScenarioBufferData183_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData183_CONVOLVE_SCALE_3X3_08U, 4},
{88, 6, 3448, 32, 1, gScenarioBufferData184_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData184_CONVOLVE_SCALE_3X3_08U, 4},
{96, 6, 3752, 32, 1, gScenarioBufferData185_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData185_CONVOLVE_SCALE_3X3_08U, 4},
{2, 8, 236, 32, 1, gScenarioBufferData186_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData186_CONVOLVE_SCALE_3X3_08U, 4},
{4, 8, 336, 32, 1, gScenarioBufferData187_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData187_CONVOLVE_SCALE_3X3_08U, 4},
{6, 8, 436, 32, 1, gScenarioBufferData188_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData188_CONVOLVE_SCALE_3X3_08U, 4},
{8, 8, 536, 32, 1, gScenarioBufferData189_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData189_CONVOLVE_SCALE_3X3_08U, 4},
{10, 8, 636, 32, 1, gScenarioBufferData190_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData190_CONVOLVE_SCALE_3X3_08U, 4},
{12, 8, 736, 32, 1, gScenarioBufferData191_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData191_CONVOLVE_SCALE_3X3_08U, 4},
{14, 8, 836, 32, 1, gScenarioBufferData192_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData192_CONVOLVE_SCALE_3X3_08U, 4},
{16, 8, 936, 32, 1, gScenarioBufferData193_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData193_CONVOLVE_SCALE_3X3_08U, 4},
{18, 8, 1036, 32, 1, gScenarioBufferData194_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData194_CONVOLVE_SCALE_3X3_08U, 4},
{20, 8, 1136, 32, 1, gScenarioBufferData195_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData195_CONVOLVE_SCALE_3X3_08U, 4},
{22, 8, 1236, 32, 1, gScenarioBufferData196_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData196_CONVOLVE_SCALE_3X3_08U, 4},
{24, 8, 1336, 32, 1, gScenarioBufferData197_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData197_CONVOLVE_SCALE_3X3_08U, 4},
{26, 8, 1436, 32, 1, gScenarioBufferData198_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData198_CONVOLVE_SCALE_3X3_08U, 4},
{28, 8, 1536, 32, 1, gScenarioBufferData199_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData199_CONVOLVE_SCALE_3X3_08U, 4},
{30, 8, 1636, 32, 1, gScenarioBufferData200_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData200_CONVOLVE_SCALE_3X3_08U, 4},
{32, 8, 1736, 32, 1, gScenarioBufferData201_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData201_CONVOLVE_SCALE_3X3_08U, 4},
{36, 8, 1936, 32, 1, gScenarioBufferData202_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData202_CONVOLVE_SCALE_3X3_08U, 4},
{40, 8, 2136, 32, 1, gScenarioBufferData203_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData203_CONVOLVE_SCALE_3X3_08U, 4},
{44, 8, 2336, 32, 1, gScenarioBufferData204_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData204_CONVOLVE_SCALE_3X3_08U, 4},
{48, 8, 2536, 32, 1, gScenarioBufferData205_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData205_CONVOLVE_SCALE_3X3_08U, 4},
{52, 8, 2736, 32, 1, gScenarioBufferData206_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData206_CONVOLVE_SCALE_3X3_08U, 4},
{56, 8, 2936, 32, 1, gScenarioBufferData207_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData207_CONVOLVE_SCALE_3X3_08U, 4},
{60, 8, 3136, 32, 1, gScenarioBufferData208_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData208_CONVOLVE_SCALE_3X3_08U, 4},
{64, 8, 3336, 32, 1, gScenarioBufferData209_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData209_CONVOLVE_SCALE_3X3_08U, 4},
{72, 8, 3736, 32, 1, gScenarioBufferData210_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData210_CONVOLVE_SCALE_3X3_08U, 4},
{2, 10, 292, 32, 1, gScenarioBufferData211_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData211_CONVOLVE_SCALE_3X3_08U, 4},
{4, 10, 416, 32, 1, gScenarioBufferData212_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData212_CONVOLVE_SCALE_3X3_08U, 4},
{6, 10, 540, 32, 1, gScenarioBufferData213_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData213_CONVOLVE_SCALE_3X3_08U, 4},
{8, 10, 664, 32, 1, gScenarioBufferData214_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData214_CONVOLVE_SCALE_3X3_08U, 4},
{10, 10, 788, 32, 1, gScenarioBufferData215_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData215_CONVOLVE_SCALE_3X3_08U, 4},
{12, 10, 912, 32, 1, gScenarioBufferData216_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData216_CONVOLVE_SCALE_3X3_08U, 4},
{14, 10, 1036, 32, 1, gScenarioBufferData217_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData217_CONVOLVE_SCALE_3X3_08U, 4},
{16, 10, 1160, 32, 1, gScenarioBufferData218_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData218_CONVOLVE_SCALE_3X3_08U, 4},
{18, 10, 1284, 32, 1, gScenarioBufferData219_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData219_CONVOLVE_SCALE_3X3_08U, 4},
{20, 10, 1408, 32, 1, gScenarioBufferData220_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData220_CONVOLVE_SCALE_3X3_08U, 4},
{22, 10, 1532, 32, 1, gScenarioBufferData221_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData221_CONVOLVE_SCALE_3X3_08U, 4},
{24, 10, 1656, 32, 1, gScenarioBufferData222_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData222_CONVOLVE_SCALE_3X3_08U, 4},
{26, 10, 1780, 32, 1, gScenarioBufferData223_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData223_CONVOLVE_SCALE_3X3_08U, 4},
{28, 10, 1904, 32, 1, gScenarioBufferData224_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData224_CONVOLVE_SCALE_3X3_08U, 4},
{30, 10, 2028, 32, 1, gScenarioBufferData225_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData225_CONVOLVE_SCALE_3X3_08U, 4},
{32, 10, 2152, 32, 1, gScenarioBufferData226_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData226_CONVOLVE_SCALE_3X3_08U, 4},
{36, 10, 2400, 32, 1, gScenarioBufferData227_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData227_CONVOLVE_SCALE_3X3_08U, 4},
{40, 10, 2648, 32, 1, gScenarioBufferData228_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData228_CONVOLVE_SCALE_3X3_08U, 4},
{44, 10, 2896, 32, 1, gScenarioBufferData229_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData229_CONVOLVE_SCALE_3X3_08U, 4},
{48, 10, 3144, 32, 1, gScenarioBufferData230_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData230_CONVOLVE_SCALE_3X3_08U, 4},
{52, 10, 3392, 32, 1, gScenarioBufferData231_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData231_CONVOLVE_SCALE_3X3_08U, 4},
{56, 10, 3640, 32, 1, gScenarioBufferData232_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData232_CONVOLVE_SCALE_3X3_08U, 4},
{2, 12, 348, 32, 1, gScenarioBufferData233_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData233_CONVOLVE_SCALE_3X3_08U, 4},
{4, 12, 496, 32, 1, gScenarioBufferData234_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData234_CONVOLVE_SCALE_3X3_08U, 4},
{6, 12, 644, 32, 1, gScenarioBufferData235_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData235_CONVOLVE_SCALE_3X3_08U, 4},
{8, 12, 792, 32, 1, gScenarioBufferData236_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData236_CONVOLVE_SCALE_3X3_08U, 4},
{10, 12, 940, 32, 1, gScenarioBufferData237_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData237_CONVOLVE_SCALE_3X3_08U, 4},
{12, 12, 1088, 32, 1, gScenarioBufferData238_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData238_CONVOLVE_SCALE_3X3_08U, 4},
{14, 12, 1236, 32, 1, gScenarioBufferData239_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData239_CONVOLVE_SCALE_3X3_08U, 4},
{16, 12, 1384, 32, 1, gScenarioBufferData240_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData240_CONVOLVE_SCALE_3X3_08U, 4},
{18, 12, 1532, 32, 1, gScenarioBufferData241_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData241_CONVOLVE_SCALE_3X3_08U, 4},
{20, 12, 1680, 32, 1, gScenarioBufferData242_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData242_CONVOLVE_SCALE_3X3_08U, 4},
{22, 12, 1828, 32, 1, gScenarioBufferData243_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData243_CONVOLVE_SCALE_3X3_08U, 4},
{24, 12, 1976, 32, 1, gScenarioBufferData244_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData244_CONVOLVE_SCALE_3X3_08U, 4},
{26, 12, 2124, 32, 1, gScenarioBufferData245_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData245_CONVOLVE_SCALE_3X3_08U, 4},
{28, 12, 2272, 32, 1, gScenarioBufferData246_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData246_CONVOLVE_SCALE_3X3_08U, 4},
{30, 12, 2420, 32, 1, gScenarioBufferData247_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData247_CONVOLVE_SCALE_3X3_08U, 4},
{32, 12, 2568, 32, 1, gScenarioBufferData248_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData248_CONVOLVE_SCALE_3X3_08U, 4},
{36, 12, 2864, 32, 1, gScenarioBufferData249_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData249_CONVOLVE_SCALE_3X3_08U, 4},
{40, 12, 3160, 32, 1, gScenarioBufferData250_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData250_CONVOLVE_SCALE_3X3_08U, 4},
{44, 12, 3456, 32, 1, gScenarioBufferData251_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData251_CONVOLVE_SCALE_3X3_08U, 4},
{48, 12, 3752, 32, 1, gScenarioBufferData252_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData252_CONVOLVE_SCALE_3X3_08U, 4},
{2, 14, 404, 32, 1, gScenarioBufferData253_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData253_CONVOLVE_SCALE_3X3_08U, 4},
{4, 14, 576, 32, 1, gScenarioBufferData254_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData254_CONVOLVE_SCALE_3X3_08U, 4},
{6, 14, 748, 32, 1, gScenarioBufferData255_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData255_CONVOLVE_SCALE_3X3_08U, 4},
{8, 14, 920, 32, 1, gScenarioBufferData256_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData256_CONVOLVE_SCALE_3X3_08U, 4},
{10, 14, 1092, 32, 1, gScenarioBufferData257_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData257_CONVOLVE_SCALE_3X3_08U, 4},
{12, 14, 1264, 32, 1, gScenarioBufferData258_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData258_CONVOLVE_SCALE_3X3_08U, 4},
{14, 14, 1436, 32, 1, gScenarioBufferData259_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData259_CONVOLVE_SCALE_3X3_08U, 4},
{16, 14, 1608, 32, 1, gScenarioBufferData260_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData260_CONVOLVE_SCALE_3X3_08U, 4},
{18, 14, 1780, 32, 1, gScenarioBufferData261_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData261_CONVOLVE_SCALE_3X3_08U, 4},
{20, 14, 1952, 32, 1, gScenarioBufferData262_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData262_CONVOLVE_SCALE_3X3_08U, 4},
{22, 14, 2124, 32, 1, gScenarioBufferData263_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData263_CONVOLVE_SCALE_3X3_08U, 4},
{24, 14, 2296, 32, 1, gScenarioBufferData264_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData264_CONVOLVE_SCALE_3X3_08U, 4},
{26, 14, 2468, 32, 1, gScenarioBufferData265_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData265_CONVOLVE_SCALE_3X3_08U, 4},
{28, 14, 2640, 32, 1, gScenarioBufferData266_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData266_CONVOLVE_SCALE_3X3_08U, 4},
{30, 14, 2812, 32, 1, gScenarioBufferData267_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData267_CONVOLVE_SCALE_3X3_08U, 4},
{32, 14, 2984, 32, 1, gScenarioBufferData268_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData268_CONVOLVE_SCALE_3X3_08U, 4},
{36, 14, 3328, 32, 1, gScenarioBufferData269_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData269_CONVOLVE_SCALE_3X3_08U, 4},
{40, 14, 3672, 32, 1, gScenarioBufferData270_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData270_CONVOLVE_SCALE_3X3_08U, 4},
{2, 16, 460, 32, 1, gScenarioBufferData271_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData271_CONVOLVE_SCALE_3X3_08U, 4},
{4, 16, 656, 32, 1, gScenarioBufferData272_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData272_CONVOLVE_SCALE_3X3_08U, 4},
{6, 16, 852, 32, 1, gScenarioBufferData273_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData273_CONVOLVE_SCALE_3X3_08U, 4},
{8, 16, 1048, 32, 1, gScenarioBufferData274_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData274_CONVOLVE_SCALE_3X3_08U, 4},
{10, 16, 1244, 32, 1, gScenarioBufferData275_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData275_CONVOLVE_SCALE_3X3_08U, 4},
{12, 16, 1440, 32, 1, gScenarioBufferData276_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData276_CONVOLVE_SCALE_3X3_08U, 4},
{14, 16, 1636, 32, 1, gScenarioBufferData277_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData277_CONVOLVE_SCALE_3X3_08U, 4},
{16, 16, 1832, 32, 1, gScenarioBufferData278_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData278_CONVOLVE_SCALE_3X3_08U, 4},
{18, 16, 2028, 32, 1, gScenarioBufferData279_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData279_CONVOLVE_SCALE_3X3_08U, 4},
{20, 16, 2224, 32, 1, gScenarioBufferData280_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData280_CONVOLVE_SCALE_3X3_08U, 4},
{22, 16, 2420, 32, 1, gScenarioBufferData281_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData281_CONVOLVE_SCALE_3X3_08U, 4},
{24, 16, 2616, 32, 1, gScenarioBufferData282_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData282_CONVOLVE_SCALE_3X3_08U, 4},
{26, 16, 2812, 32, 1, gScenarioBufferData283_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData283_CONVOLVE_SCALE_3X3_08U, 4},
{28, 16, 3008, 32, 1, gScenarioBufferData284_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData284_CONVOLVE_SCALE_3X3_08U, 4},
{30, 16, 3204, 32, 1, gScenarioBufferData285_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData285_CONVOLVE_SCALE_3X3_08U, 4},
{32, 16, 3400, 32, 1, gScenarioBufferData286_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData286_CONVOLVE_SCALE_3X3_08U, 4},
{36, 16, 3792, 32, 1, gScenarioBufferData287_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData287_CONVOLVE_SCALE_3X3_08U, 4},
{2, 18, 516, 32, 1, gScenarioBufferData288_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData288_CONVOLVE_SCALE_3X3_08U, 4},
{4, 18, 736, 32, 1, gScenarioBufferData289_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData289_CONVOLVE_SCALE_3X3_08U, 4},
{6, 18, 956, 32, 1, gScenarioBufferData290_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData290_CONVOLVE_SCALE_3X3_08U, 4},
{8, 18, 1176, 32, 1, gScenarioBufferData291_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData291_CONVOLVE_SCALE_3X3_08U, 4},
{10, 18, 1396, 32, 1, gScenarioBufferData292_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData292_CONVOLVE_SCALE_3X3_08U, 4},
{12, 18, 1616, 32, 1, gScenarioBufferData293_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData293_CONVOLVE_SCALE_3X3_08U, 4},
{14, 18, 1836, 32, 1, gScenarioBufferData294_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData294_CONVOLVE_SCALE_3X3_08U, 4},
{16, 18, 2056, 32, 1, gScenarioBufferData295_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData295_CONVOLVE_SCALE_3X3_08U, 4},
{18, 18, 2276, 32, 1, gScenarioBufferData296_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData296_CONVOLVE_SCALE_3X3_08U, 4},
{20, 18, 2496, 32, 1, gScenarioBufferData297_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData297_CONVOLVE_SCALE_3X3_08U, 4},
{22, 18, 2716, 32, 1, gScenarioBufferData298_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData298_CONVOLVE_SCALE_3X3_08U, 4},
{24, 18, 2936, 32, 1, gScenarioBufferData299_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData299_CONVOLVE_SCALE_3X3_08U, 4},
{26, 18, 3156, 32, 1, gScenarioBufferData300_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData300_CONVOLVE_SCALE_3X3_08U, 4},
{28, 18, 3376, 32, 1, gScenarioBufferData301_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData301_CONVOLVE_SCALE_3X3_08U, 4},
{30, 18, 3596, 32, 1, gScenarioBufferData302_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData302_CONVOLVE_SCALE_3X3_08U, 4},
{32, 18, 3816, 32, 1, gScenarioBufferData303_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData303_CONVOLVE_SCALE_3X3_08U, 4},
{2, 20, 572, 32, 1, gScenarioBufferData304_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData304_CONVOLVE_SCALE_3X3_08U, 4},
{4, 20, 816, 32, 1, gScenarioBufferData305_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData305_CONVOLVE_SCALE_3X3_08U, 4},
{6, 20, 1060, 32, 1, gScenarioBufferData306_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData306_CONVOLVE_SCALE_3X3_08U, 4},
{8, 20, 1304, 32, 1, gScenarioBufferData307_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData307_CONVOLVE_SCALE_3X3_08U, 4},
{10, 20, 1548, 32, 1, gScenarioBufferData308_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData308_CONVOLVE_SCALE_3X3_08U, 4},
{12, 20, 1792, 32, 1, gScenarioBufferData309_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData309_CONVOLVE_SCALE_3X3_08U, 4},
{14, 20, 2036, 32, 1, gScenarioBufferData310_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData310_CONVOLVE_SCALE_3X3_08U, 4},
{16, 20, 2280, 32, 1, gScenarioBufferData311_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData311_CONVOLVE_SCALE_3X3_08U, 4},
{18, 20, 2524, 32, 1, gScenarioBufferData312_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData312_CONVOLVE_SCALE_3X3_08U, 4},
{20, 20, 2768, 32, 1, gScenarioBufferData313_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData313_CONVOLVE_SCALE_3X3_08U, 4},
{22, 20, 3012, 32, 1, gScenarioBufferData314_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData314_CONVOLVE_SCALE_3X3_08U, 4},
{24, 20, 3256, 32, 1, gScenarioBufferData315_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData315_CONVOLVE_SCALE_3X3_08U, 4},
{26, 20, 3500, 32, 1, gScenarioBufferData316_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData316_CONVOLVE_SCALE_3X3_08U, 4},
{28, 20, 3744, 32, 1, gScenarioBufferData317_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData317_CONVOLVE_SCALE_3X3_08U, 4},
{2, 22, 628, 32, 1, gScenarioBufferData318_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData318_CONVOLVE_SCALE_3X3_08U, 4},
{4, 22, 896, 32, 1, gScenarioBufferData319_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData319_CONVOLVE_SCALE_3X3_08U, 4},
{6, 22, 1164, 32, 1, gScenarioBufferData320_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData320_CONVOLVE_SCALE_3X3_08U, 4},
{8, 22, 1432, 32, 1, gScenarioBufferData321_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData321_CONVOLVE_SCALE_3X3_08U, 4},
{10, 22, 1700, 32, 1, gScenarioBufferData322_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData322_CONVOLVE_SCALE_3X3_08U, 4},
{12, 22, 1968, 32, 1, gScenarioBufferData323_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData323_CONVOLVE_SCALE_3X3_08U, 4},
{14, 22, 2236, 32, 1, gScenarioBufferData324_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData324_CONVOLVE_SCALE_3X3_08U, 4},
{16, 22, 2504, 32, 1, gScenarioBufferData325_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData325_CONVOLVE_SCALE_3X3_08U, 4},
{18, 22, 2772, 32, 1, gScenarioBufferData326_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData326_CONVOLVE_SCALE_3X3_08U, 4},
{20, 22, 3040, 32, 1, gScenarioBufferData327_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData327_CONVOLVE_SCALE_3X3_08U, 4},
{22, 22, 3308, 32, 1, gScenarioBufferData328_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData328_CONVOLVE_SCALE_3X3_08U, 4},
{24, 22, 3576, 32, 1, gScenarioBufferData329_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData329_CONVOLVE_SCALE_3X3_08U, 4},
{2, 24, 684, 32, 1, gScenarioBufferData330_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData330_CONVOLVE_SCALE_3X3_08U, 4},
{4, 24, 976, 32, 1, gScenarioBufferData331_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData331_CONVOLVE_SCALE_3X3_08U, 4},
{6, 24, 1268, 32, 1, gScenarioBufferData332_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData332_CONVOLVE_SCALE_3X3_08U, 4},
{8, 24, 1560, 32, 1, gScenarioBufferData333_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData333_CONVOLVE_SCALE_3X3_08U, 4},
{10, 24, 1852, 32, 1, gScenarioBufferData334_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData334_CONVOLVE_SCALE_3X3_08U, 4},
{12, 24, 2144, 32, 1, gScenarioBufferData335_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData335_CONVOLVE_SCALE_3X3_08U, 4},
{14, 24, 2436, 32, 1, gScenarioBufferData336_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData336_CONVOLVE_SCALE_3X3_08U, 4},
{16, 24, 2728, 32, 1, gScenarioBufferData337_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData337_CONVOLVE_SCALE_3X3_08U, 4},
{18, 24, 3020, 32, 1, gScenarioBufferData338_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData338_CONVOLVE_SCALE_3X3_08U, 4},
{20, 24, 3312, 32, 1, gScenarioBufferData339_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData339_CONVOLVE_SCALE_3X3_08U, 4},
{22, 24, 3604, 32, 1, gScenarioBufferData340_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData340_CONVOLVE_SCALE_3X3_08U, 4},
{2, 26, 740, 32, 1, gScenarioBufferData341_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData341_CONVOLVE_SCALE_3X3_08U, 4},
{4, 26, 1056, 32, 1, gScenarioBufferData342_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData342_CONVOLVE_SCALE_3X3_08U, 4},
{6, 26, 1372, 32, 1, gScenarioBufferData343_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData343_CONVOLVE_SCALE_3X3_08U, 4},
{8, 26, 1688, 32, 1, gScenarioBufferData344_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData344_CONVOLVE_SCALE_3X3_08U, 4},
{10, 26, 2004, 32, 1, gScenarioBufferData345_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData345_CONVOLVE_SCALE_3X3_08U, 4},
{12, 26, 2320, 32, 1, gScenarioBufferData346_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData346_CONVOLVE_SCALE_3X3_08U, 4},
{14, 26, 2636, 32, 1, gScenarioBufferData347_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData347_CONVOLVE_SCALE_3X3_08U, 4},
{16, 26, 2952, 32, 1, gScenarioBufferData348_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData348_CONVOLVE_SCALE_3X3_08U, 4},
{18, 26, 3268, 32, 1, gScenarioBufferData349_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData349_CONVOLVE_SCALE_3X3_08U, 4},
{20, 26, 3584, 32, 1, gScenarioBufferData350_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData350_CONVOLVE_SCALE_3X3_08U, 4},
{2, 28, 796, 32, 1, gScenarioBufferData351_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData351_CONVOLVE_SCALE_3X3_08U, 4},
{4, 28, 1136, 32, 1, gScenarioBufferData352_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData352_CONVOLVE_SCALE_3X3_08U, 4},
{6, 28, 1476, 32, 1, gScenarioBufferData353_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData353_CONVOLVE_SCALE_3X3_08U, 4},
{8, 28, 1816, 32, 1, gScenarioBufferData354_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData354_CONVOLVE_SCALE_3X3_08U, 4},
{10, 28, 2156, 32, 1, gScenarioBufferData355_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData355_CONVOLVE_SCALE_3X3_08U, 4},
{12, 28, 2496, 32, 1, gScenarioBufferData356_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData356_CONVOLVE_SCALE_3X3_08U, 4},
{14, 28, 2836, 32, 1, gScenarioBufferData357_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData357_CONVOLVE_SCALE_3X3_08U, 4},
{16, 28, 3176, 32, 1, gScenarioBufferData358_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData358_CONVOLVE_SCALE_3X3_08U, 4},
{18, 28, 3516, 32, 1, gScenarioBufferData359_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData359_CONVOLVE_SCALE_3X3_08U, 4},
{2, 30, 852, 32, 1, gScenarioBufferData360_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData360_CONVOLVE_SCALE_3X3_08U, 4},
{4, 30, 1216, 32, 1, gScenarioBufferData361_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData361_CONVOLVE_SCALE_3X3_08U, 4},
{6, 30, 1580, 32, 1, gScenarioBufferData362_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData362_CONVOLVE_SCALE_3X3_08U, 4},
{8, 30, 1944, 32, 1, gScenarioBufferData363_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData363_CONVOLVE_SCALE_3X3_08U, 4},
{10, 30, 2308, 32, 1, gScenarioBufferData364_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData364_CONVOLVE_SCALE_3X3_08U, 4},
{12, 30, 2672, 32, 1, gScenarioBufferData365_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData365_CONVOLVE_SCALE_3X3_08U, 4},
{14, 30, 3036, 32, 1, gScenarioBufferData366_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData366_CONVOLVE_SCALE_3X3_08U, 4},
{16, 30, 3400, 32, 1, gScenarioBufferData367_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData367_CONVOLVE_SCALE_3X3_08U, 4},
{18, 30, 3764, 32, 1, gScenarioBufferData368_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData368_CONVOLVE_SCALE_3X3_08U, 4},
{2, 32, 908, 32, 1, gScenarioBufferData369_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData369_CONVOLVE_SCALE_3X3_08U, 4},
{4, 32, 1296, 32, 1, gScenarioBufferData370_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData370_CONVOLVE_SCALE_3X3_08U, 4},
{6, 32, 1684, 32, 1, gScenarioBufferData371_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData371_CONVOLVE_SCALE_3X3_08U, 4},
{8, 32, 2072, 32, 1, gScenarioBufferData372_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData372_CONVOLVE_SCALE_3X3_08U, 4},
{10, 32, 2460, 32, 1, gScenarioBufferData373_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData373_CONVOLVE_SCALE_3X3_08U, 4},
{12, 32, 2848, 32, 1, gScenarioBufferData374_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData374_CONVOLVE_SCALE_3X3_08U, 4},
{14, 32, 3236, 32, 1, gScenarioBufferData375_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData375_CONVOLVE_SCALE_3X3_08U, 4},
{16, 32, 3624, 32, 1, gScenarioBufferData376_CONVOLVE_SCALE_3X3_08U, 64, gScenarioKernelData376_CONVOLVE_SCALE_3X3_08U, 4}
};

static acf_scenario_list gScenarioList_CONVOLVE_SCALE_3X3_08U = {
377, //number of scenarios
gScenarioArray_CONVOLVE_SCALE_3X3_08U};
//**************************************************************

class CONVOLVE_SCALE_3X3_08U : public ACF_Process_APU
{

public:
   CONVOLVE_SCALE_3X3_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CONVOLVE_SCALE_3X3_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CONVOLVE_SCALE_3X3_08U",
                                        CONVOLVE_SCALE_3X3_08U_LOAD_SEGMENTS,
                                        CONVOLVE_SCALE_3X3_08U_LOAD_PMEM, CONVOLVE_SCALE_3X3_08U_LOAD_PMEM_SIZE,
                                        CONVOLVE_SCALE_3X3_08U_LOAD_DMEM, CONVOLVE_SCALE_3X3_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CONVOLVE_SCALE_3X3_08U_APEX_LOG_BUFFER, CONVOLVE_SCALE_3X3_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("F_COEFF", icp::DATATYPE_08S, 1, 1, 9, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("F_SCALE", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CONVOLVE_SCALE_3X3_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CONVOLVE_SCALE_3X3_08U

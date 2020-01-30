#ifndef _ACF_PROCESS_APU_GAUSSIAN_7X7_08U
#define _ACF_PROCESS_APU_GAUSSIAN_7X7_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <GAUSSIAN_7X7_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_GAUSSIAN_7X7_08U[] = {{4, 1, 2, 0, {0, 0, 4, 4}}, {4, 1, 10, 0, {3, 3, 0, 0}}, {4, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_GAUSSIAN_7X7_08U[] = {{6, 1, 2, 0, {0, 0, 4, 4}}, {6, 1, 10, 0, {3, 3, 0, 0}}, {6, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_GAUSSIAN_7X7_08U[] = {{8, 1, 2, 0, {0, 0, 4, 4}}, {8, 1, 10, 0, {3, 3, 0, 0}}, {8, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_GAUSSIAN_7X7_08U[] = {{10, 1, 2, 0, {0, 0, 4, 4}}, {10, 1, 10, 0, {3, 3, 0, 0}}, {10, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_GAUSSIAN_7X7_08U[] = {{12, 1, 2, 0, {0, 0, 4, 4}}, {12, 1, 10, 0, {3, 3, 0, 0}}, {12, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_GAUSSIAN_7X7_08U[] = {{14, 1, 2, 0, {0, 0, 4, 4}}, {14, 1, 10, 0, {3, 3, 0, 0}}, {14, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_GAUSSIAN_7X7_08U[] = {{16, 1, 2, 0, {0, 0, 4, 4}}, {16, 1, 10, 0, {3, 3, 0, 0}}, {16, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_GAUSSIAN_7X7_08U[] = {{18, 1, 2, 0, {0, 0, 4, 4}}, {18, 1, 10, 0, {3, 3, 0, 0}}, {18, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_GAUSSIAN_7X7_08U[] = {{20, 1, 2, 0, {0, 0, 4, 4}}, {20, 1, 10, 0, {3, 3, 0, 0}}, {20, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_GAUSSIAN_7X7_08U[] = {{22, 1, 2, 0, {0, 0, 4, 4}}, {22, 1, 10, 0, {3, 3, 0, 0}}, {22, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_GAUSSIAN_7X7_08U[] = {{24, 1, 2, 0, {0, 0, 4, 4}}, {24, 1, 10, 0, {3, 3, 0, 0}}, {24, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_GAUSSIAN_7X7_08U[] = {{26, 1, 2, 0, {0, 0, 4, 4}}, {26, 1, 10, 0, {3, 3, 0, 0}}, {26, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_GAUSSIAN_7X7_08U[] = {{28, 1, 2, 0, {0, 0, 4, 4}}, {28, 1, 10, 0, {3, 3, 0, 0}}, {28, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_GAUSSIAN_7X7_08U[] = {{30, 1, 2, 0, {0, 0, 4, 4}}, {30, 1, 10, 0, {3, 3, 0, 0}}, {30, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_GAUSSIAN_7X7_08U[] = {{32, 1, 2, 0, {0, 0, 4, 4}}, {32, 1, 10, 0, {3, 3, 0, 0}}, {32, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_GAUSSIAN_7X7_08U[] = {{36, 1, 2, 0, {0, 0, 4, 4}}, {36, 1, 10, 0, {3, 3, 0, 0}}, {36, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_GAUSSIAN_7X7_08U[] = {{40, 1, 2, 0, {0, 0, 4, 4}}, {40, 1, 10, 0, {3, 3, 0, 0}}, {40, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_GAUSSIAN_7X7_08U[] = {{44, 1, 2, 0, {0, 0, 4, 4}}, {44, 1, 10, 0, {3, 3, 0, 0}}, {44, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_GAUSSIAN_7X7_08U[] = {{48, 1, 2, 0, {0, 0, 4, 4}}, {48, 1, 10, 0, {3, 3, 0, 0}}, {48, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_GAUSSIAN_7X7_08U[] = {{52, 1, 2, 0, {0, 0, 4, 4}}, {52, 1, 10, 0, {3, 3, 0, 0}}, {52, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_GAUSSIAN_7X7_08U[] = {{56, 1, 2, 0, {0, 0, 4, 4}}, {56, 1, 10, 0, {3, 3, 0, 0}}, {56, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_GAUSSIAN_7X7_08U[] = {{60, 1, 2, 0, {0, 0, 4, 4}}, {60, 1, 10, 0, {3, 3, 0, 0}}, {60, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_GAUSSIAN_7X7_08U[] = {{64, 1, 2, 0, {0, 0, 4, 4}}, {64, 1, 10, 0, {3, 3, 0, 0}}, {64, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_GAUSSIAN_7X7_08U[] = {{72, 1, 2, 0, {0, 0, 4, 4}}, {72, 1, 10, 0, {3, 3, 0, 0}}, {72, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_GAUSSIAN_7X7_08U[] = {{80, 1, 2, 0, {0, 0, 4, 4}}, {80, 1, 10, 0, {3, 3, 0, 0}}, {80, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_GAUSSIAN_7X7_08U[] = {{88, 1, 2, 0, {0, 0, 4, 4}}, {88, 1, 10, 0, {3, 3, 0, 0}}, {88, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_GAUSSIAN_7X7_08U[] = {{96, 1, 2, 0, {0, 0, 4, 4}}, {96, 1, 10, 0, {3, 3, 0, 0}}, {96, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_GAUSSIAN_7X7_08U[] = {{104, 1, 2, 0, {0, 0, 4, 4}}, {104, 1, 10, 0, {3, 3, 0, 0}}, {104, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_GAUSSIAN_7X7_08U[] = {{112, 1, 2, 0, {0, 0, 4, 4}}, {112, 1, 10, 0, {3, 3, 0, 0}}, {112, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_GAUSSIAN_7X7_08U[] = {{120, 1, 2, 0, {0, 0, 4, 4}}, {120, 1, 10, 0, {3, 3, 0, 0}}, {120, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_GAUSSIAN_7X7_08U[] = {{128, 1, 2, 0, {0, 0, 4, 4}}, {128, 1, 10, 0, {3, 3, 0, 0}}, {128, 1, 2, 3, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_GAUSSIAN_7X7_08U[] = {{4, 2, 2, 0, {0, 0, 4, 4}}, {4, 2, 7, 0, {3, 3, 0, 0}}, {4, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_GAUSSIAN_7X7_08U[] = {{6, 2, 2, 0, {0, 0, 4, 4}}, {6, 2, 7, 0, {3, 3, 0, 0}}, {6, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_GAUSSIAN_7X7_08U[] = {{8, 2, 2, 0, {0, 0, 4, 4}}, {8, 2, 7, 0, {3, 3, 0, 0}}, {8, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_GAUSSIAN_7X7_08U[] = {{10, 2, 2, 0, {0, 0, 4, 4}}, {10, 2, 7, 0, {3, 3, 0, 0}}, {10, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_GAUSSIAN_7X7_08U[] = {{12, 2, 2, 0, {0, 0, 4, 4}}, {12, 2, 7, 0, {3, 3, 0, 0}}, {12, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_GAUSSIAN_7X7_08U[] = {{14, 2, 2, 0, {0, 0, 4, 4}}, {14, 2, 7, 0, {3, 3, 0, 0}}, {14, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_GAUSSIAN_7X7_08U[] = {{16, 2, 2, 0, {0, 0, 4, 4}}, {16, 2, 7, 0, {3, 3, 0, 0}}, {16, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_GAUSSIAN_7X7_08U[] = {{18, 2, 2, 0, {0, 0, 4, 4}}, {18, 2, 7, 0, {3, 3, 0, 0}}, {18, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_GAUSSIAN_7X7_08U[] = {{20, 2, 2, 0, {0, 0, 4, 4}}, {20, 2, 7, 0, {3, 3, 0, 0}}, {20, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_GAUSSIAN_7X7_08U[] = {{22, 2, 2, 0, {0, 0, 4, 4}}, {22, 2, 7, 0, {3, 3, 0, 0}}, {22, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_GAUSSIAN_7X7_08U[] = {{24, 2, 2, 0, {0, 0, 4, 4}}, {24, 2, 7, 0, {3, 3, 0, 0}}, {24, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_GAUSSIAN_7X7_08U[] = {{26, 2, 2, 0, {0, 0, 4, 4}}, {26, 2, 7, 0, {3, 3, 0, 0}}, {26, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_GAUSSIAN_7X7_08U[] = {{28, 2, 2, 0, {0, 0, 4, 4}}, {28, 2, 7, 0, {3, 3, 0, 0}}, {28, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_GAUSSIAN_7X7_08U[] = {{30, 2, 2, 0, {0, 0, 4, 4}}, {30, 2, 7, 0, {3, 3, 0, 0}}, {30, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_GAUSSIAN_7X7_08U[] = {{32, 2, 2, 0, {0, 0, 4, 4}}, {32, 2, 7, 0, {3, 3, 0, 0}}, {32, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_GAUSSIAN_7X7_08U[] = {{36, 2, 2, 0, {0, 0, 4, 4}}, {36, 2, 7, 0, {3, 3, 0, 0}}, {36, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_GAUSSIAN_7X7_08U[] = {{40, 2, 2, 0, {0, 0, 4, 4}}, {40, 2, 7, 0, {3, 3, 0, 0}}, {40, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_GAUSSIAN_7X7_08U[] = {{44, 2, 2, 0, {0, 0, 4, 4}}, {44, 2, 7, 0, {3, 3, 0, 0}}, {44, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_GAUSSIAN_7X7_08U[] = {{48, 2, 2, 0, {0, 0, 4, 4}}, {48, 2, 7, 0, {3, 3, 0, 0}}, {48, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_GAUSSIAN_7X7_08U[] = {{52, 2, 2, 0, {0, 0, 4, 4}}, {52, 2, 7, 0, {3, 3, 0, 0}}, {52, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_GAUSSIAN_7X7_08U[] = {{56, 2, 2, 0, {0, 0, 4, 4}}, {56, 2, 7, 0, {3, 3, 0, 0}}, {56, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_GAUSSIAN_7X7_08U[] = {{60, 2, 2, 0, {0, 0, 4, 4}}, {60, 2, 7, 0, {3, 3, 0, 0}}, {60, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_GAUSSIAN_7X7_08U[] = {{64, 2, 2, 0, {0, 0, 4, 4}}, {64, 2, 7, 0, {3, 3, 0, 0}}, {64, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_GAUSSIAN_7X7_08U[] = {{72, 2, 2, 0, {0, 0, 4, 4}}, {72, 2, 7, 0, {3, 3, 0, 0}}, {72, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_GAUSSIAN_7X7_08U[] = {{80, 2, 2, 0, {0, 0, 4, 4}}, {80, 2, 7, 0, {3, 3, 0, 0}}, {80, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_GAUSSIAN_7X7_08U[] = {{88, 2, 2, 0, {0, 0, 4, 4}}, {88, 2, 7, 0, {3, 3, 0, 0}}, {88, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_GAUSSIAN_7X7_08U[] = {{96, 2, 2, 0, {0, 0, 4, 4}}, {96, 2, 7, 0, {3, 3, 0, 0}}, {96, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_GAUSSIAN_7X7_08U[] = {{104, 2, 2, 0, {0, 0, 4, 4}}, {104, 2, 7, 0, {3, 3, 0, 0}}, {104, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_GAUSSIAN_7X7_08U[] = {{112, 2, 2, 0, {0, 0, 4, 4}}, {112, 2, 7, 0, {3, 3, 0, 0}}, {112, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_GAUSSIAN_7X7_08U[] = {{120, 2, 2, 0, {0, 0, 4, 4}}, {120, 2, 7, 0, {3, 3, 0, 0}}, {120, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_GAUSSIAN_7X7_08U[] = {{128, 2, 2, 0, {0, 0, 4, 4}}, {128, 2, 7, 0, {3, 3, 0, 0}}, {128, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_GAUSSIAN_7X7_08U[] = {{4, 3, 2, 0, {0, 0, 4, 4}}, {4, 3, 4, 0, {3, 3, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_GAUSSIAN_7X7_08U[] = {{6, 3, 2, 0, {0, 0, 4, 4}}, {6, 3, 4, 0, {3, 3, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_GAUSSIAN_7X7_08U[] = {{8, 3, 2, 0, {0, 0, 4, 4}}, {8, 3, 4, 0, {3, 3, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_GAUSSIAN_7X7_08U[] = {{10, 3, 2, 0, {0, 0, 4, 4}}, {10, 3, 4, 0, {3, 3, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_GAUSSIAN_7X7_08U[] = {{12, 3, 2, 0, {0, 0, 4, 4}}, {12, 3, 4, 0, {3, 3, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_GAUSSIAN_7X7_08U[] = {{14, 3, 2, 0, {0, 0, 4, 4}}, {14, 3, 4, 0, {3, 3, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_GAUSSIAN_7X7_08U[] = {{16, 3, 2, 0, {0, 0, 4, 4}}, {16, 3, 4, 0, {3, 3, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_GAUSSIAN_7X7_08U[] = {{18, 3, 2, 0, {0, 0, 4, 4}}, {18, 3, 4, 0, {3, 3, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_GAUSSIAN_7X7_08U[] = {{20, 3, 2, 0, {0, 0, 4, 4}}, {20, 3, 4, 0, {3, 3, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_GAUSSIAN_7X7_08U[] = {{22, 3, 2, 0, {0, 0, 4, 4}}, {22, 3, 4, 0, {3, 3, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_GAUSSIAN_7X7_08U[] = {{24, 3, 2, 0, {0, 0, 4, 4}}, {24, 3, 4, 0, {3, 3, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_GAUSSIAN_7X7_08U[] = {{26, 3, 2, 0, {0, 0, 4, 4}}, {26, 3, 4, 0, {3, 3, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_GAUSSIAN_7X7_08U[] = {{28, 3, 2, 0, {0, 0, 4, 4}}, {28, 3, 4, 0, {3, 3, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_GAUSSIAN_7X7_08U[] = {{30, 3, 2, 0, {0, 0, 4, 4}}, {30, 3, 4, 0, {3, 3, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_GAUSSIAN_7X7_08U[] = {{32, 3, 2, 0, {0, 0, 4, 4}}, {32, 3, 4, 0, {3, 3, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_GAUSSIAN_7X7_08U[] = {{36, 3, 2, 0, {0, 0, 4, 4}}, {36, 3, 4, 0, {3, 3, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_GAUSSIAN_7X7_08U[] = {{40, 3, 2, 0, {0, 0, 4, 4}}, {40, 3, 4, 0, {3, 3, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_GAUSSIAN_7X7_08U[] = {{44, 3, 2, 0, {0, 0, 4, 4}}, {44, 3, 4, 0, {3, 3, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_GAUSSIAN_7X7_08U[] = {{48, 3, 2, 0, {0, 0, 4, 4}}, {48, 3, 4, 0, {3, 3, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_GAUSSIAN_7X7_08U[] = {{52, 3, 2, 0, {0, 0, 4, 4}}, {52, 3, 4, 0, {3, 3, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_GAUSSIAN_7X7_08U[] = {{56, 3, 2, 0, {0, 0, 4, 4}}, {56, 3, 4, 0, {3, 3, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_GAUSSIAN_7X7_08U[] = {{60, 3, 2, 0, {0, 0, 4, 4}}, {60, 3, 4, 0, {3, 3, 0, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_GAUSSIAN_7X7_08U[] = {{64, 3, 2, 0, {0, 0, 4, 4}}, {64, 3, 4, 0, {3, 3, 0, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_GAUSSIAN_7X7_08U[] = {{72, 3, 2, 0, {0, 0, 4, 4}}, {72, 3, 4, 0, {3, 3, 0, 0}}, {72, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_GAUSSIAN_7X7_08U[] = {{80, 3, 2, 0, {0, 0, 4, 4}}, {80, 3, 4, 0, {3, 3, 0, 0}}, {80, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_GAUSSIAN_7X7_08U[] = {{88, 3, 2, 0, {0, 0, 4, 4}}, {88, 3, 4, 0, {3, 3, 0, 0}}, {88, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_GAUSSIAN_7X7_08U[] = {{96, 3, 2, 0, {0, 0, 4, 4}}, {96, 3, 4, 0, {3, 3, 0, 0}}, {96, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_GAUSSIAN_7X7_08U[] = {{104, 3, 2, 0, {0, 0, 4, 4}}, {104, 3, 4, 0, {3, 3, 0, 0}}, {104, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_GAUSSIAN_7X7_08U[] = {{112, 3, 2, 0, {0, 0, 4, 4}}, {112, 3, 4, 0, {3, 3, 0, 0}}, {112, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_GAUSSIAN_7X7_08U[] = {{120, 3, 2, 0, {0, 0, 4, 4}}, {120, 3, 4, 0, {3, 3, 0, 0}}, {120, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_GAUSSIAN_7X7_08U[] = {{4, 4, 2, 0, {0, 0, 4, 4}}, {4, 4, 4, 0, {3, 3, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_GAUSSIAN_7X7_08U[] = {{6, 4, 2, 0, {0, 0, 4, 4}}, {6, 4, 4, 0, {3, 3, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_GAUSSIAN_7X7_08U[] = {{8, 4, 2, 0, {0, 0, 4, 4}}, {8, 4, 4, 0, {3, 3, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_GAUSSIAN_7X7_08U[] = {{10, 4, 2, 0, {0, 0, 4, 4}}, {10, 4, 4, 0, {3, 3, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_GAUSSIAN_7X7_08U[] = {{12, 4, 2, 0, {0, 0, 4, 4}}, {12, 4, 4, 0, {3, 3, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_GAUSSIAN_7X7_08U[] = {{14, 4, 2, 0, {0, 0, 4, 4}}, {14, 4, 4, 0, {3, 3, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_GAUSSIAN_7X7_08U[] = {{16, 4, 2, 0, {0, 0, 4, 4}}, {16, 4, 4, 0, {3, 3, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_GAUSSIAN_7X7_08U[] = {{18, 4, 2, 0, {0, 0, 4, 4}}, {18, 4, 4, 0, {3, 3, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_GAUSSIAN_7X7_08U[] = {{20, 4, 2, 0, {0, 0, 4, 4}}, {20, 4, 4, 0, {3, 3, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_GAUSSIAN_7X7_08U[] = {{22, 4, 2, 0, {0, 0, 4, 4}}, {22, 4, 4, 0, {3, 3, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_GAUSSIAN_7X7_08U[] = {{24, 4, 2, 0, {0, 0, 4, 4}}, {24, 4, 4, 0, {3, 3, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_GAUSSIAN_7X7_08U[] = {{26, 4, 2, 0, {0, 0, 4, 4}}, {26, 4, 4, 0, {3, 3, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_GAUSSIAN_7X7_08U[] = {{28, 4, 2, 0, {0, 0, 4, 4}}, {28, 4, 4, 0, {3, 3, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_GAUSSIAN_7X7_08U[] = {{30, 4, 2, 0, {0, 0, 4, 4}}, {30, 4, 4, 0, {3, 3, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_GAUSSIAN_7X7_08U[] = {{32, 4, 2, 0, {0, 0, 4, 4}}, {32, 4, 4, 0, {3, 3, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_GAUSSIAN_7X7_08U[] = {{36, 4, 2, 0, {0, 0, 4, 4}}, {36, 4, 4, 0, {3, 3, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_GAUSSIAN_7X7_08U[] = {{40, 4, 2, 0, {0, 0, 4, 4}}, {40, 4, 4, 0, {3, 3, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_GAUSSIAN_7X7_08U[] = {{44, 4, 2, 0, {0, 0, 4, 4}}, {44, 4, 4, 0, {3, 3, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_GAUSSIAN_7X7_08U[] = {{48, 4, 2, 0, {0, 0, 4, 4}}, {48, 4, 4, 0, {3, 3, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_GAUSSIAN_7X7_08U[] = {{52, 4, 2, 0, {0, 0, 4, 4}}, {52, 4, 4, 0, {3, 3, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_GAUSSIAN_7X7_08U[] = {{56, 4, 2, 0, {0, 0, 4, 4}}, {56, 4, 4, 0, {3, 3, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_GAUSSIAN_7X7_08U[] = {{60, 4, 2, 0, {0, 0, 4, 4}}, {60, 4, 4, 0, {3, 3, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_GAUSSIAN_7X7_08U[] = {{64, 4, 2, 0, {0, 0, 4, 4}}, {64, 4, 4, 0, {3, 3, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_GAUSSIAN_7X7_08U[] = {{72, 4, 2, 0, {0, 0, 4, 4}}, {72, 4, 4, 0, {3, 3, 0, 0}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_GAUSSIAN_7X7_08U[] = {{80, 4, 2, 0, {0, 0, 4, 4}}, {80, 4, 4, 0, {3, 3, 0, 0}}, {80, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_GAUSSIAN_7X7_08U[] = {{88, 4, 2, 0, {0, 0, 4, 4}}, {88, 4, 4, 0, {3, 3, 0, 0}}, {88, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_GAUSSIAN_7X7_08U[] = {{96, 4, 2, 0, {0, 0, 4, 4}}, {96, 4, 4, 0, {3, 3, 0, 0}}, {96, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_GAUSSIAN_7X7_08U[] = {{4, 5, 2, 0, {0, 0, 4, 4}}, {4, 5, 4, 0, {3, 3, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_GAUSSIAN_7X7_08U[] = {{6, 5, 2, 0, {0, 0, 4, 4}}, {6, 5, 4, 0, {3, 3, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_GAUSSIAN_7X7_08U[] = {{8, 5, 2, 0, {0, 0, 4, 4}}, {8, 5, 4, 0, {3, 3, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_GAUSSIAN_7X7_08U[] = {{10, 5, 2, 0, {0, 0, 4, 4}}, {10, 5, 4, 0, {3, 3, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_GAUSSIAN_7X7_08U[] = {{12, 5, 2, 0, {0, 0, 4, 4}}, {12, 5, 4, 0, {3, 3, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_GAUSSIAN_7X7_08U[] = {{14, 5, 2, 0, {0, 0, 4, 4}}, {14, 5, 4, 0, {3, 3, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_GAUSSIAN_7X7_08U[] = {{16, 5, 2, 0, {0, 0, 4, 4}}, {16, 5, 4, 0, {3, 3, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_GAUSSIAN_7X7_08U[] = {{18, 5, 2, 0, {0, 0, 4, 4}}, {18, 5, 4, 0, {3, 3, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_GAUSSIAN_7X7_08U[] = {{20, 5, 2, 0, {0, 0, 4, 4}}, {20, 5, 4, 0, {3, 3, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_GAUSSIAN_7X7_08U[] = {{22, 5, 2, 0, {0, 0, 4, 4}}, {22, 5, 4, 0, {3, 3, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_GAUSSIAN_7X7_08U[] = {{24, 5, 2, 0, {0, 0, 4, 4}}, {24, 5, 4, 0, {3, 3, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_GAUSSIAN_7X7_08U[] = {{26, 5, 2, 0, {0, 0, 4, 4}}, {26, 5, 4, 0, {3, 3, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_GAUSSIAN_7X7_08U[] = {{28, 5, 2, 0, {0, 0, 4, 4}}, {28, 5, 4, 0, {3, 3, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_GAUSSIAN_7X7_08U[] = {{30, 5, 2, 0, {0, 0, 4, 4}}, {30, 5, 4, 0, {3, 3, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_GAUSSIAN_7X7_08U[] = {{32, 5, 2, 0, {0, 0, 4, 4}}, {32, 5, 4, 0, {3, 3, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_GAUSSIAN_7X7_08U[] = {{36, 5, 2, 0, {0, 0, 4, 4}}, {36, 5, 4, 0, {3, 3, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_GAUSSIAN_7X7_08U[] = {{40, 5, 2, 0, {0, 0, 4, 4}}, {40, 5, 4, 0, {3, 3, 0, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_GAUSSIAN_7X7_08U[] = {{44, 5, 2, 0, {0, 0, 4, 4}}, {44, 5, 4, 0, {3, 3, 0, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_GAUSSIAN_7X7_08U[] = {{48, 5, 2, 0, {0, 0, 4, 4}}, {48, 5, 4, 0, {3, 3, 0, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_GAUSSIAN_7X7_08U[] = {{52, 5, 2, 0, {0, 0, 4, 4}}, {52, 5, 4, 0, {3, 3, 0, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_GAUSSIAN_7X7_08U[] = {{56, 5, 2, 0, {0, 0, 4, 4}}, {56, 5, 4, 0, {3, 3, 0, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_GAUSSIAN_7X7_08U[] = {{60, 5, 2, 0, {0, 0, 4, 4}}, {60, 5, 4, 0, {3, 3, 0, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_GAUSSIAN_7X7_08U[] = {{64, 5, 2, 0, {0, 0, 4, 4}}, {64, 5, 4, 0, {3, 3, 0, 0}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_GAUSSIAN_7X7_08U[] = {{72, 5, 2, 0, {0, 0, 4, 4}}, {72, 5, 4, 0, {3, 3, 0, 0}}, {72, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_GAUSSIAN_7X7_08U[] = {{80, 5, 2, 0, {0, 0, 4, 4}}, {80, 5, 4, 0, {3, 3, 0, 0}}, {80, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_GAUSSIAN_7X7_08U[] = {{4, 6, 2, 0, {0, 0, 4, 4}}, {4, 6, 4, 0, {3, 3, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_GAUSSIAN_7X7_08U[] = {{6, 6, 2, 0, {0, 0, 4, 4}}, {6, 6, 4, 0, {3, 3, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_GAUSSIAN_7X7_08U[] = {{8, 6, 2, 0, {0, 0, 4, 4}}, {8, 6, 4, 0, {3, 3, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_GAUSSIAN_7X7_08U[] = {{10, 6, 2, 0, {0, 0, 4, 4}}, {10, 6, 4, 0, {3, 3, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_GAUSSIAN_7X7_08U[] = {{12, 6, 2, 0, {0, 0, 4, 4}}, {12, 6, 4, 0, {3, 3, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_GAUSSIAN_7X7_08U[] = {{14, 6, 2, 0, {0, 0, 4, 4}}, {14, 6, 4, 0, {3, 3, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_GAUSSIAN_7X7_08U[] = {{16, 6, 2, 0, {0, 0, 4, 4}}, {16, 6, 4, 0, {3, 3, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_GAUSSIAN_7X7_08U[] = {{18, 6, 2, 0, {0, 0, 4, 4}}, {18, 6, 4, 0, {3, 3, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_GAUSSIAN_7X7_08U[] = {{20, 6, 2, 0, {0, 0, 4, 4}}, {20, 6, 4, 0, {3, 3, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_GAUSSIAN_7X7_08U[] = {{22, 6, 2, 0, {0, 0, 4, 4}}, {22, 6, 4, 0, {3, 3, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_GAUSSIAN_7X7_08U[] = {{24, 6, 2, 0, {0, 0, 4, 4}}, {24, 6, 4, 0, {3, 3, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_GAUSSIAN_7X7_08U[] = {{26, 6, 2, 0, {0, 0, 4, 4}}, {26, 6, 4, 0, {3, 3, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_GAUSSIAN_7X7_08U[] = {{28, 6, 2, 0, {0, 0, 4, 4}}, {28, 6, 4, 0, {3, 3, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_GAUSSIAN_7X7_08U[] = {{30, 6, 2, 0, {0, 0, 4, 4}}, {30, 6, 4, 0, {3, 3, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_GAUSSIAN_7X7_08U[] = {{32, 6, 2, 0, {0, 0, 4, 4}}, {32, 6, 4, 0, {3, 3, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_GAUSSIAN_7X7_08U[] = {{36, 6, 2, 0, {0, 0, 4, 4}}, {36, 6, 4, 0, {3, 3, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_GAUSSIAN_7X7_08U[] = {{40, 6, 2, 0, {0, 0, 4, 4}}, {40, 6, 4, 0, {3, 3, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_GAUSSIAN_7X7_08U[] = {{44, 6, 2, 0, {0, 0, 4, 4}}, {44, 6, 4, 0, {3, 3, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_GAUSSIAN_7X7_08U[] = {{48, 6, 2, 0, {0, 0, 4, 4}}, {48, 6, 4, 0, {3, 3, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_GAUSSIAN_7X7_08U[] = {{52, 6, 2, 0, {0, 0, 4, 4}}, {52, 6, 4, 0, {3, 3, 0, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_GAUSSIAN_7X7_08U[] = {{56, 6, 2, 0, {0, 0, 4, 4}}, {56, 6, 4, 0, {3, 3, 0, 0}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_GAUSSIAN_7X7_08U[] = {{60, 6, 2, 0, {0, 0, 4, 4}}, {60, 6, 4, 0, {3, 3, 0, 0}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_GAUSSIAN_7X7_08U[] = {{64, 6, 2, 0, {0, 0, 4, 4}}, {64, 6, 4, 0, {3, 3, 0, 0}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_GAUSSIAN_7X7_08U[] = {{4, 8, 2, 0, {0, 0, 4, 4}}, {4, 8, 4, 0, {3, 3, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_GAUSSIAN_7X7_08U[] = {{6, 8, 2, 0, {0, 0, 4, 4}}, {6, 8, 4, 0, {3, 3, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_GAUSSIAN_7X7_08U[] = {{8, 8, 2, 0, {0, 0, 4, 4}}, {8, 8, 4, 0, {3, 3, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_GAUSSIAN_7X7_08U[] = {{10, 8, 2, 0, {0, 0, 4, 4}}, {10, 8, 4, 0, {3, 3, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_GAUSSIAN_7X7_08U[] = {{12, 8, 2, 0, {0, 0, 4, 4}}, {12, 8, 4, 0, {3, 3, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_GAUSSIAN_7X7_08U[] = {{14, 8, 2, 0, {0, 0, 4, 4}}, {14, 8, 4, 0, {3, 3, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_GAUSSIAN_7X7_08U[] = {{16, 8, 2, 0, {0, 0, 4, 4}}, {16, 8, 4, 0, {3, 3, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_GAUSSIAN_7X7_08U[] = {{18, 8, 2, 0, {0, 0, 4, 4}}, {18, 8, 4, 0, {3, 3, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_GAUSSIAN_7X7_08U[] = {{20, 8, 2, 0, {0, 0, 4, 4}}, {20, 8, 4, 0, {3, 3, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_GAUSSIAN_7X7_08U[] = {{22, 8, 2, 0, {0, 0, 4, 4}}, {22, 8, 4, 0, {3, 3, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_GAUSSIAN_7X7_08U[] = {{24, 8, 2, 0, {0, 0, 4, 4}}, {24, 8, 4, 0, {3, 3, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_GAUSSIAN_7X7_08U[] = {{26, 8, 2, 0, {0, 0, 4, 4}}, {26, 8, 4, 0, {3, 3, 0, 0}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_GAUSSIAN_7X7_08U[] = {{28, 8, 2, 0, {0, 0, 4, 4}}, {28, 8, 4, 0, {3, 3, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_GAUSSIAN_7X7_08U[] = {{30, 8, 2, 0, {0, 0, 4, 4}}, {30, 8, 4, 0, {3, 3, 0, 0}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_GAUSSIAN_7X7_08U[] = {{32, 8, 2, 0, {0, 0, 4, 4}}, {32, 8, 4, 0, {3, 3, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_GAUSSIAN_7X7_08U[] = {{36, 8, 2, 0, {0, 0, 4, 4}}, {36, 8, 4, 0, {3, 3, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_GAUSSIAN_7X7_08U[] = {{40, 8, 2, 0, {0, 0, 4, 4}}, {40, 8, 4, 0, {3, 3, 0, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_GAUSSIAN_7X7_08U[] = {{44, 8, 2, 0, {0, 0, 4, 4}}, {44, 8, 4, 0, {3, 3, 0, 0}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_GAUSSIAN_7X7_08U[] = {{48, 8, 2, 0, {0, 0, 4, 4}}, {48, 8, 4, 0, {3, 3, 0, 0}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_GAUSSIAN_7X7_08U[] = {{52, 8, 2, 0, {0, 0, 4, 4}}, {52, 8, 4, 0, {3, 3, 0, 0}}, {52, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_GAUSSIAN_7X7_08U[] = {{4, 10, 2, 0, {0, 0, 4, 4}}, {4, 10, 4, 0, {3, 3, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_GAUSSIAN_7X7_08U[] = {{6, 10, 2, 0, {0, 0, 4, 4}}, {6, 10, 4, 0, {3, 3, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_GAUSSIAN_7X7_08U[] = {{8, 10, 2, 0, {0, 0, 4, 4}}, {8, 10, 4, 0, {3, 3, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_GAUSSIAN_7X7_08U[] = {{10, 10, 2, 0, {0, 0, 4, 4}}, {10, 10, 4, 0, {3, 3, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_GAUSSIAN_7X7_08U[] = {{12, 10, 2, 0, {0, 0, 4, 4}}, {12, 10, 4, 0, {3, 3, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_GAUSSIAN_7X7_08U[] = {{14, 10, 2, 0, {0, 0, 4, 4}}, {14, 10, 4, 0, {3, 3, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_GAUSSIAN_7X7_08U[] = {{16, 10, 2, 0, {0, 0, 4, 4}}, {16, 10, 4, 0, {3, 3, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_GAUSSIAN_7X7_08U[] = {{18, 10, 2, 0, {0, 0, 4, 4}}, {18, 10, 4, 0, {3, 3, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_GAUSSIAN_7X7_08U[] = {{20, 10, 2, 0, {0, 0, 4, 4}}, {20, 10, 4, 0, {3, 3, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_GAUSSIAN_7X7_08U[] = {{22, 10, 2, 0, {0, 0, 4, 4}}, {22, 10, 4, 0, {3, 3, 0, 0}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_GAUSSIAN_7X7_08U[] = {{24, 10, 2, 0, {0, 0, 4, 4}}, {24, 10, 4, 0, {3, 3, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_GAUSSIAN_7X7_08U[] = {{26, 10, 2, 0, {0, 0, 4, 4}}, {26, 10, 4, 0, {3, 3, 0, 0}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_GAUSSIAN_7X7_08U[] = {{28, 10, 2, 0, {0, 0, 4, 4}}, {28, 10, 4, 0, {3, 3, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_GAUSSIAN_7X7_08U[] = {{30, 10, 2, 0, {0, 0, 4, 4}}, {30, 10, 4, 0, {3, 3, 0, 0}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_GAUSSIAN_7X7_08U[] = {{32, 10, 2, 0, {0, 0, 4, 4}}, {32, 10, 4, 0, {3, 3, 0, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_GAUSSIAN_7X7_08U[] = {{36, 10, 2, 0, {0, 0, 4, 4}}, {36, 10, 4, 0, {3, 3, 0, 0}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_GAUSSIAN_7X7_08U[] = {{40, 10, 2, 0, {0, 0, 4, 4}}, {40, 10, 4, 0, {3, 3, 0, 0}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_GAUSSIAN_7X7_08U[] = {{4, 12, 2, 0, {0, 0, 4, 4}}, {4, 12, 4, 0, {3, 3, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_GAUSSIAN_7X7_08U[] = {{6, 12, 2, 0, {0, 0, 4, 4}}, {6, 12, 4, 0, {3, 3, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_GAUSSIAN_7X7_08U[] = {{8, 12, 2, 0, {0, 0, 4, 4}}, {8, 12, 4, 0, {3, 3, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_GAUSSIAN_7X7_08U[] = {{10, 12, 2, 0, {0, 0, 4, 4}}, {10, 12, 4, 0, {3, 3, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_GAUSSIAN_7X7_08U[] = {{12, 12, 2, 0, {0, 0, 4, 4}}, {12, 12, 4, 0, {3, 3, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_GAUSSIAN_7X7_08U[] = {{14, 12, 2, 0, {0, 0, 4, 4}}, {14, 12, 4, 0, {3, 3, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_GAUSSIAN_7X7_08U[] = {{16, 12, 2, 0, {0, 0, 4, 4}}, {16, 12, 4, 0, {3, 3, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_GAUSSIAN_7X7_08U[] = {{18, 12, 2, 0, {0, 0, 4, 4}}, {18, 12, 4, 0, {3, 3, 0, 0}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_GAUSSIAN_7X7_08U[] = {{20, 12, 2, 0, {0, 0, 4, 4}}, {20, 12, 4, 0, {3, 3, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_GAUSSIAN_7X7_08U[] = {{22, 12, 2, 0, {0, 0, 4, 4}}, {22, 12, 4, 0, {3, 3, 0, 0}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_GAUSSIAN_7X7_08U[] = {{24, 12, 2, 0, {0, 0, 4, 4}}, {24, 12, 4, 0, {3, 3, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_GAUSSIAN_7X7_08U[] = {{26, 12, 2, 0, {0, 0, 4, 4}}, {26, 12, 4, 0, {3, 3, 0, 0}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_GAUSSIAN_7X7_08U[] = {{28, 12, 2, 0, {0, 0, 4, 4}}, {28, 12, 4, 0, {3, 3, 0, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_GAUSSIAN_7X7_08U[] = {{30, 12, 2, 0, {0, 0, 4, 4}}, {30, 12, 4, 0, {3, 3, 0, 0}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_GAUSSIAN_7X7_08U[] = {{32, 12, 2, 0, {0, 0, 4, 4}}, {32, 12, 4, 0, {3, 3, 0, 0}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_GAUSSIAN_7X7_08U[] = {{4, 14, 2, 0, {0, 0, 4, 4}}, {4, 14, 4, 0, {3, 3, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_GAUSSIAN_7X7_08U[] = {{6, 14, 2, 0, {0, 0, 4, 4}}, {6, 14, 4, 0, {3, 3, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_GAUSSIAN_7X7_08U[] = {{8, 14, 2, 0, {0, 0, 4, 4}}, {8, 14, 4, 0, {3, 3, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_GAUSSIAN_7X7_08U[] = {{10, 14, 2, 0, {0, 0, 4, 4}}, {10, 14, 4, 0, {3, 3, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_GAUSSIAN_7X7_08U[] = {{12, 14, 2, 0, {0, 0, 4, 4}}, {12, 14, 4, 0, {3, 3, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_GAUSSIAN_7X7_08U[] = {{14, 14, 2, 0, {0, 0, 4, 4}}, {14, 14, 4, 0, {3, 3, 0, 0}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_GAUSSIAN_7X7_08U[] = {{16, 14, 2, 0, {0, 0, 4, 4}}, {16, 14, 4, 0, {3, 3, 0, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_GAUSSIAN_7X7_08U[] = {{18, 14, 2, 0, {0, 0, 4, 4}}, {18, 14, 4, 0, {3, 3, 0, 0}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_GAUSSIAN_7X7_08U[] = {{20, 14, 2, 0, {0, 0, 4, 4}}, {20, 14, 4, 0, {3, 3, 0, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_GAUSSIAN_7X7_08U[] = {{22, 14, 2, 0, {0, 0, 4, 4}}, {22, 14, 4, 0, {3, 3, 0, 0}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_GAUSSIAN_7X7_08U[] = {{24, 14, 2, 0, {0, 0, 4, 4}}, {24, 14, 4, 0, {3, 3, 0, 0}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_GAUSSIAN_7X7_08U[] = {{26, 14, 2, 0, {0, 0, 4, 4}}, {26, 14, 4, 0, {3, 3, 0, 0}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_GAUSSIAN_7X7_08U[] = {{28, 14, 2, 0, {0, 0, 4, 4}}, {28, 14, 4, 0, {3, 3, 0, 0}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_GAUSSIAN_7X7_08U[] = {{30, 14, 2, 0, {0, 0, 4, 4}}, {30, 14, 4, 0, {3, 3, 0, 0}}, {30, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_GAUSSIAN_7X7_08U[] = {{4, 16, 2, 0, {0, 0, 4, 4}}, {4, 16, 4, 0, {3, 3, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_GAUSSIAN_7X7_08U[] = {{6, 16, 2, 0, {0, 0, 4, 4}}, {6, 16, 4, 0, {3, 3, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_GAUSSIAN_7X7_08U[] = {{8, 16, 2, 0, {0, 0, 4, 4}}, {8, 16, 4, 0, {3, 3, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_GAUSSIAN_7X7_08U[] = {{10, 16, 2, 0, {0, 0, 4, 4}}, {10, 16, 4, 0, {3, 3, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_GAUSSIAN_7X7_08U[] = {{12, 16, 2, 0, {0, 0, 4, 4}}, {12, 16, 4, 0, {3, 3, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_GAUSSIAN_7X7_08U[] = {{14, 16, 2, 0, {0, 0, 4, 4}}, {14, 16, 4, 0, {3, 3, 0, 0}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_GAUSSIAN_7X7_08U[] = {{16, 16, 2, 0, {0, 0, 4, 4}}, {16, 16, 4, 0, {3, 3, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_GAUSSIAN_7X7_08U[] = {{18, 16, 2, 0, {0, 0, 4, 4}}, {18, 16, 4, 0, {3, 3, 0, 0}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_GAUSSIAN_7X7_08U[] = {{20, 16, 2, 0, {0, 0, 4, 4}}, {20, 16, 4, 0, {3, 3, 0, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_GAUSSIAN_7X7_08U[] = {{22, 16, 2, 0, {0, 0, 4, 4}}, {22, 16, 4, 0, {3, 3, 0, 0}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_GAUSSIAN_7X7_08U[] = {{24, 16, 2, 0, {0, 0, 4, 4}}, {24, 16, 4, 0, {3, 3, 0, 0}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_GAUSSIAN_7X7_08U[] = {{26, 16, 2, 0, {0, 0, 4, 4}}, {26, 16, 4, 0, {3, 3, 0, 0}}, {26, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_GAUSSIAN_7X7_08U[] = {{4, 18, 2, 0, {0, 0, 4, 4}}, {4, 18, 4, 0, {3, 3, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_GAUSSIAN_7X7_08U[] = {{6, 18, 2, 0, {0, 0, 4, 4}}, {6, 18, 4, 0, {3, 3, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_GAUSSIAN_7X7_08U[] = {{8, 18, 2, 0, {0, 0, 4, 4}}, {8, 18, 4, 0, {3, 3, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_GAUSSIAN_7X7_08U[] = {{10, 18, 2, 0, {0, 0, 4, 4}}, {10, 18, 4, 0, {3, 3, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_GAUSSIAN_7X7_08U[] = {{12, 18, 2, 0, {0, 0, 4, 4}}, {12, 18, 4, 0, {3, 3, 0, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_GAUSSIAN_7X7_08U[] = {{14, 18, 2, 0, {0, 0, 4, 4}}, {14, 18, 4, 0, {3, 3, 0, 0}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_GAUSSIAN_7X7_08U[] = {{16, 18, 2, 0, {0, 0, 4, 4}}, {16, 18, 4, 0, {3, 3, 0, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_GAUSSIAN_7X7_08U[] = {{18, 18, 2, 0, {0, 0, 4, 4}}, {18, 18, 4, 0, {3, 3, 0, 0}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_GAUSSIAN_7X7_08U[] = {{20, 18, 2, 0, {0, 0, 4, 4}}, {20, 18, 4, 0, {3, 3, 0, 0}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_GAUSSIAN_7X7_08U[] = {{22, 18, 2, 0, {0, 0, 4, 4}}, {22, 18, 4, 0, {3, 3, 0, 0}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_GAUSSIAN_7X7_08U[] = {{4, 20, 2, 0, {0, 0, 4, 4}}, {4, 20, 4, 0, {3, 3, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_GAUSSIAN_7X7_08U[] = {{6, 20, 2, 0, {0, 0, 4, 4}}, {6, 20, 4, 0, {3, 3, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_GAUSSIAN_7X7_08U[] = {{8, 20, 2, 0, {0, 0, 4, 4}}, {8, 20, 4, 0, {3, 3, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_GAUSSIAN_7X7_08U[] = {{10, 20, 2, 0, {0, 0, 4, 4}}, {10, 20, 4, 0, {3, 3, 0, 0}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_GAUSSIAN_7X7_08U[] = {{12, 20, 2, 0, {0, 0, 4, 4}}, {12, 20, 4, 0, {3, 3, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_GAUSSIAN_7X7_08U[] = {{14, 20, 2, 0, {0, 0, 4, 4}}, {14, 20, 4, 0, {3, 3, 0, 0}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_GAUSSIAN_7X7_08U[] = {{16, 20, 2, 0, {0, 0, 4, 4}}, {16, 20, 4, 0, {3, 3, 0, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_GAUSSIAN_7X7_08U[] = {{18, 20, 2, 0, {0, 0, 4, 4}}, {18, 20, 4, 0, {3, 3, 0, 0}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_GAUSSIAN_7X7_08U[] = {{20, 20, 2, 0, {0, 0, 4, 4}}, {20, 20, 4, 0, {3, 3, 0, 0}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_GAUSSIAN_7X7_08U[] = {{4, 22, 2, 0, {0, 0, 4, 4}}, {4, 22, 4, 0, {3, 3, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_GAUSSIAN_7X7_08U[] = {{6, 22, 2, 0, {0, 0, 4, 4}}, {6, 22, 4, 0, {3, 3, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_GAUSSIAN_7X7_08U[] = {{8, 22, 2, 0, {0, 0, 4, 4}}, {8, 22, 4, 0, {3, 3, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_GAUSSIAN_7X7_08U[] = {{10, 22, 2, 0, {0, 0, 4, 4}}, {10, 22, 4, 0, {3, 3, 0, 0}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_GAUSSIAN_7X7_08U[] = {{12, 22, 2, 0, {0, 0, 4, 4}}, {12, 22, 4, 0, {3, 3, 0, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_GAUSSIAN_7X7_08U[] = {{14, 22, 2, 0, {0, 0, 4, 4}}, {14, 22, 4, 0, {3, 3, 0, 0}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_GAUSSIAN_7X7_08U[] = {{16, 22, 2, 0, {0, 0, 4, 4}}, {16, 22, 4, 0, {3, 3, 0, 0}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_GAUSSIAN_7X7_08U[] = {{18, 22, 2, 0, {0, 0, 4, 4}}, {18, 22, 4, 0, {3, 3, 0, 0}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_GAUSSIAN_7X7_08U[] = {{4, 24, 2, 0, {0, 0, 4, 4}}, {4, 24, 4, 0, {3, 3, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_GAUSSIAN_7X7_08U[] = {{6, 24, 2, 0, {0, 0, 4, 4}}, {6, 24, 4, 0, {3, 3, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_GAUSSIAN_7X7_08U[] = {{8, 24, 2, 0, {0, 0, 4, 4}}, {8, 24, 4, 0, {3, 3, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_GAUSSIAN_7X7_08U[] = {{10, 24, 2, 0, {0, 0, 4, 4}}, {10, 24, 4, 0, {3, 3, 0, 0}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_GAUSSIAN_7X7_08U[] = {{12, 24, 2, 0, {0, 0, 4, 4}}, {12, 24, 4, 0, {3, 3, 0, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_GAUSSIAN_7X7_08U[] = {{14, 24, 2, 0, {0, 0, 4, 4}}, {14, 24, 4, 0, {3, 3, 0, 0}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_GAUSSIAN_7X7_08U[] = {{16, 24, 2, 0, {0, 0, 4, 4}}, {16, 24, 4, 0, {3, 3, 0, 0}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_GAUSSIAN_7X7_08U[] = {{4, 26, 2, 0, {0, 0, 4, 4}}, {4, 26, 4, 0, {3, 3, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_GAUSSIAN_7X7_08U[] = {{6, 26, 2, 0, {0, 0, 4, 4}}, {6, 26, 4, 0, {3, 3, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_GAUSSIAN_7X7_08U[] = {{8, 26, 2, 0, {0, 0, 4, 4}}, {8, 26, 4, 0, {3, 3, 0, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_GAUSSIAN_7X7_08U[] = {{10, 26, 2, 0, {0, 0, 4, 4}}, {10, 26, 4, 0, {3, 3, 0, 0}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_GAUSSIAN_7X7_08U[] = {{12, 26, 2, 0, {0, 0, 4, 4}}, {12, 26, 4, 0, {3, 3, 0, 0}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_GAUSSIAN_7X7_08U[] = {{14, 26, 2, 0, {0, 0, 4, 4}}, {14, 26, 4, 0, {3, 3, 0, 0}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_GAUSSIAN_7X7_08U[] = {{16, 26, 2, 0, {0, 0, 4, 4}}, {16, 26, 4, 0, {3, 3, 0, 0}}, {16, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_GAUSSIAN_7X7_08U[] = {{4, 28, 2, 0, {0, 0, 4, 4}}, {4, 28, 4, 0, {3, 3, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_GAUSSIAN_7X7_08U[] = {{6, 28, 2, 0, {0, 0, 4, 4}}, {6, 28, 4, 0, {3, 3, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_GAUSSIAN_7X7_08U[] = {{8, 28, 2, 0, {0, 0, 4, 4}}, {8, 28, 4, 0, {3, 3, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_GAUSSIAN_7X7_08U[] = {{10, 28, 2, 0, {0, 0, 4, 4}}, {10, 28, 4, 0, {3, 3, 0, 0}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_GAUSSIAN_7X7_08U[] = {{12, 28, 2, 0, {0, 0, 4, 4}}, {12, 28, 4, 0, {3, 3, 0, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_GAUSSIAN_7X7_08U[] = {{14, 28, 2, 0, {0, 0, 4, 4}}, {14, 28, 4, 0, {3, 3, 0, 0}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_GAUSSIAN_7X7_08U[] = {{4, 30, 2, 0, {0, 0, 4, 4}}, {4, 30, 4, 0, {3, 3, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_GAUSSIAN_7X7_08U[] = {{6, 30, 2, 0, {0, 0, 4, 4}}, {6, 30, 4, 0, {3, 3, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_GAUSSIAN_7X7_08U[] = {{8, 30, 2, 0, {0, 0, 4, 4}}, {8, 30, 4, 0, {3, 3, 0, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_GAUSSIAN_7X7_08U[] = {{10, 30, 2, 0, {0, 0, 4, 4}}, {10, 30, 4, 0, {3, 3, 0, 0}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_GAUSSIAN_7X7_08U[] = {{12, 30, 2, 0, {0, 0, 4, 4}}, {12, 30, 4, 0, {3, 3, 0, 0}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_GAUSSIAN_7X7_08U[] = {{4, 32, 2, 0, {0, 0, 4, 4}}, {4, 32, 4, 0, {3, 3, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_GAUSSIAN_7X7_08U[] = {{6, 32, 2, 0, {0, 0, 4, 4}}, {6, 32, 4, 0, {3, 3, 0, 0}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_GAUSSIAN_7X7_08U[] = {{8, 32, 2, 0, {0, 0, 4, 4}}, {8, 32, 4, 0, {3, 3, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_GAUSSIAN_7X7_08U[] = {{10, 32, 2, 0, {0, 0, 4, 4}}, {10, 32, 4, 0, {3, 3, 0, 0}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_GAUSSIAN_7X7_08U[] = {{12, 32, 2, 0, {0, 0, 4, 4}}, {12, 32, 4, 0, {3, 3, 0, 0}}, {12, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_GAUSSIAN_7X7_08U[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_GAUSSIAN_7X7_08U[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_GAUSSIAN_7X7_08U[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_GAUSSIAN_7X7_08U[] = {
{4, 1, 96, 0, 3, gScenarioBufferData0_GAUSSIAN_7X7_08U, 48, gScenarioKernelData0_GAUSSIAN_7X7_08U, 8},
{6, 1, 136, 0, 3, gScenarioBufferData1_GAUSSIAN_7X7_08U, 48, gScenarioKernelData1_GAUSSIAN_7X7_08U, 8},
{8, 1, 176, 0, 3, gScenarioBufferData2_GAUSSIAN_7X7_08U, 48, gScenarioKernelData2_GAUSSIAN_7X7_08U, 8},
{10, 1, 216, 0, 3, gScenarioBufferData3_GAUSSIAN_7X7_08U, 48, gScenarioKernelData3_GAUSSIAN_7X7_08U, 8},
{12, 1, 256, 0, 3, gScenarioBufferData4_GAUSSIAN_7X7_08U, 48, gScenarioKernelData4_GAUSSIAN_7X7_08U, 8},
{14, 1, 296, 0, 3, gScenarioBufferData5_GAUSSIAN_7X7_08U, 48, gScenarioKernelData5_GAUSSIAN_7X7_08U, 8},
{16, 1, 336, 0, 3, gScenarioBufferData6_GAUSSIAN_7X7_08U, 48, gScenarioKernelData6_GAUSSIAN_7X7_08U, 8},
{18, 1, 376, 0, 3, gScenarioBufferData7_GAUSSIAN_7X7_08U, 48, gScenarioKernelData7_GAUSSIAN_7X7_08U, 8},
{20, 1, 416, 0, 3, gScenarioBufferData8_GAUSSIAN_7X7_08U, 48, gScenarioKernelData8_GAUSSIAN_7X7_08U, 8},
{22, 1, 456, 0, 3, gScenarioBufferData9_GAUSSIAN_7X7_08U, 48, gScenarioKernelData9_GAUSSIAN_7X7_08U, 8},
{24, 1, 496, 0, 3, gScenarioBufferData10_GAUSSIAN_7X7_08U, 48, gScenarioKernelData10_GAUSSIAN_7X7_08U, 8},
{26, 1, 536, 0, 3, gScenarioBufferData11_GAUSSIAN_7X7_08U, 48, gScenarioKernelData11_GAUSSIAN_7X7_08U, 8},
{28, 1, 576, 0, 3, gScenarioBufferData12_GAUSSIAN_7X7_08U, 48, gScenarioKernelData12_GAUSSIAN_7X7_08U, 8},
{30, 1, 616, 0, 3, gScenarioBufferData13_GAUSSIAN_7X7_08U, 48, gScenarioKernelData13_GAUSSIAN_7X7_08U, 8},
{32, 1, 656, 0, 3, gScenarioBufferData14_GAUSSIAN_7X7_08U, 48, gScenarioKernelData14_GAUSSIAN_7X7_08U, 8},
{36, 1, 736, 0, 3, gScenarioBufferData15_GAUSSIAN_7X7_08U, 48, gScenarioKernelData15_GAUSSIAN_7X7_08U, 8},
{40, 1, 816, 0, 3, gScenarioBufferData16_GAUSSIAN_7X7_08U, 48, gScenarioKernelData16_GAUSSIAN_7X7_08U, 8},
{44, 1, 896, 0, 3, gScenarioBufferData17_GAUSSIAN_7X7_08U, 48, gScenarioKernelData17_GAUSSIAN_7X7_08U, 8},
{48, 1, 976, 0, 3, gScenarioBufferData18_GAUSSIAN_7X7_08U, 48, gScenarioKernelData18_GAUSSIAN_7X7_08U, 8},
{52, 1, 1056, 0, 3, gScenarioBufferData19_GAUSSIAN_7X7_08U, 48, gScenarioKernelData19_GAUSSIAN_7X7_08U, 8},
{56, 1, 1136, 0, 3, gScenarioBufferData20_GAUSSIAN_7X7_08U, 48, gScenarioKernelData20_GAUSSIAN_7X7_08U, 8},
{60, 1, 1216, 0, 3, gScenarioBufferData21_GAUSSIAN_7X7_08U, 48, gScenarioKernelData21_GAUSSIAN_7X7_08U, 8},
{64, 1, 1296, 0, 3, gScenarioBufferData22_GAUSSIAN_7X7_08U, 48, gScenarioKernelData22_GAUSSIAN_7X7_08U, 8},
{72, 1, 1456, 0, 3, gScenarioBufferData23_GAUSSIAN_7X7_08U, 48, gScenarioKernelData23_GAUSSIAN_7X7_08U, 8},
{80, 1, 1616, 0, 3, gScenarioBufferData24_GAUSSIAN_7X7_08U, 48, gScenarioKernelData24_GAUSSIAN_7X7_08U, 8},
{88, 1, 1776, 0, 3, gScenarioBufferData25_GAUSSIAN_7X7_08U, 48, gScenarioKernelData25_GAUSSIAN_7X7_08U, 8},
{96, 1, 1936, 0, 3, gScenarioBufferData26_GAUSSIAN_7X7_08U, 48, gScenarioKernelData26_GAUSSIAN_7X7_08U, 8},
{104, 1, 2096, 0, 3, gScenarioBufferData27_GAUSSIAN_7X7_08U, 48, gScenarioKernelData27_GAUSSIAN_7X7_08U, 8},
{112, 1, 2256, 0, 3, gScenarioBufferData28_GAUSSIAN_7X7_08U, 48, gScenarioKernelData28_GAUSSIAN_7X7_08U, 8},
{120, 1, 2416, 0, 3, gScenarioBufferData29_GAUSSIAN_7X7_08U, 48, gScenarioKernelData29_GAUSSIAN_7X7_08U, 8},
{128, 1, 2576, 0, 3, gScenarioBufferData30_GAUSSIAN_7X7_08U, 48, gScenarioKernelData30_GAUSSIAN_7X7_08U, 8},
{4, 2, 144, 0, 2, gScenarioBufferData31_GAUSSIAN_7X7_08U, 48, gScenarioKernelData31_GAUSSIAN_7X7_08U, 8},
{6, 2, 200, 0, 2, gScenarioBufferData32_GAUSSIAN_7X7_08U, 48, gScenarioKernelData32_GAUSSIAN_7X7_08U, 8},
{8, 2, 256, 0, 2, gScenarioBufferData33_GAUSSIAN_7X7_08U, 48, gScenarioKernelData33_GAUSSIAN_7X7_08U, 8},
{10, 2, 312, 0, 2, gScenarioBufferData34_GAUSSIAN_7X7_08U, 48, gScenarioKernelData34_GAUSSIAN_7X7_08U, 8},
{12, 2, 368, 0, 2, gScenarioBufferData35_GAUSSIAN_7X7_08U, 48, gScenarioKernelData35_GAUSSIAN_7X7_08U, 8},
{14, 2, 424, 0, 2, gScenarioBufferData36_GAUSSIAN_7X7_08U, 48, gScenarioKernelData36_GAUSSIAN_7X7_08U, 8},
{16, 2, 480, 0, 2, gScenarioBufferData37_GAUSSIAN_7X7_08U, 48, gScenarioKernelData37_GAUSSIAN_7X7_08U, 8},
{18, 2, 536, 0, 2, gScenarioBufferData38_GAUSSIAN_7X7_08U, 48, gScenarioKernelData38_GAUSSIAN_7X7_08U, 8},
{20, 2, 592, 0, 2, gScenarioBufferData39_GAUSSIAN_7X7_08U, 48, gScenarioKernelData39_GAUSSIAN_7X7_08U, 8},
{22, 2, 648, 0, 2, gScenarioBufferData40_GAUSSIAN_7X7_08U, 48, gScenarioKernelData40_GAUSSIAN_7X7_08U, 8},
{24, 2, 704, 0, 2, gScenarioBufferData41_GAUSSIAN_7X7_08U, 48, gScenarioKernelData41_GAUSSIAN_7X7_08U, 8},
{26, 2, 760, 0, 2, gScenarioBufferData42_GAUSSIAN_7X7_08U, 48, gScenarioKernelData42_GAUSSIAN_7X7_08U, 8},
{28, 2, 816, 0, 2, gScenarioBufferData43_GAUSSIAN_7X7_08U, 48, gScenarioKernelData43_GAUSSIAN_7X7_08U, 8},
{30, 2, 872, 0, 2, gScenarioBufferData44_GAUSSIAN_7X7_08U, 48, gScenarioKernelData44_GAUSSIAN_7X7_08U, 8},
{32, 2, 928, 0, 2, gScenarioBufferData45_GAUSSIAN_7X7_08U, 48, gScenarioKernelData45_GAUSSIAN_7X7_08U, 8},
{36, 2, 1040, 0, 2, gScenarioBufferData46_GAUSSIAN_7X7_08U, 48, gScenarioKernelData46_GAUSSIAN_7X7_08U, 8},
{40, 2, 1152, 0, 2, gScenarioBufferData47_GAUSSIAN_7X7_08U, 48, gScenarioKernelData47_GAUSSIAN_7X7_08U, 8},
{44, 2, 1264, 0, 2, gScenarioBufferData48_GAUSSIAN_7X7_08U, 48, gScenarioKernelData48_GAUSSIAN_7X7_08U, 8},
{48, 2, 1376, 0, 2, gScenarioBufferData49_GAUSSIAN_7X7_08U, 48, gScenarioKernelData49_GAUSSIAN_7X7_08U, 8},
{52, 2, 1488, 0, 2, gScenarioBufferData50_GAUSSIAN_7X7_08U, 48, gScenarioKernelData50_GAUSSIAN_7X7_08U, 8},
{56, 2, 1600, 0, 2, gScenarioBufferData51_GAUSSIAN_7X7_08U, 48, gScenarioKernelData51_GAUSSIAN_7X7_08U, 8},
{60, 2, 1712, 0, 2, gScenarioBufferData52_GAUSSIAN_7X7_08U, 48, gScenarioKernelData52_GAUSSIAN_7X7_08U, 8},
{64, 2, 1824, 0, 2, gScenarioBufferData53_GAUSSIAN_7X7_08U, 48, gScenarioKernelData53_GAUSSIAN_7X7_08U, 8},
{72, 2, 2048, 0, 2, gScenarioBufferData54_GAUSSIAN_7X7_08U, 48, gScenarioKernelData54_GAUSSIAN_7X7_08U, 8},
{80, 2, 2272, 0, 2, gScenarioBufferData55_GAUSSIAN_7X7_08U, 48, gScenarioKernelData55_GAUSSIAN_7X7_08U, 8},
{88, 2, 2496, 0, 2, gScenarioBufferData56_GAUSSIAN_7X7_08U, 48, gScenarioKernelData56_GAUSSIAN_7X7_08U, 8},
{96, 2, 2720, 0, 2, gScenarioBufferData57_GAUSSIAN_7X7_08U, 48, gScenarioKernelData57_GAUSSIAN_7X7_08U, 8},
{104, 2, 2944, 0, 2, gScenarioBufferData58_GAUSSIAN_7X7_08U, 48, gScenarioKernelData58_GAUSSIAN_7X7_08U, 8},
{112, 2, 3168, 0, 2, gScenarioBufferData59_GAUSSIAN_7X7_08U, 48, gScenarioKernelData59_GAUSSIAN_7X7_08U, 8},
{120, 2, 3392, 0, 2, gScenarioBufferData60_GAUSSIAN_7X7_08U, 48, gScenarioKernelData60_GAUSSIAN_7X7_08U, 8},
{128, 2, 3616, 0, 2, gScenarioBufferData61_GAUSSIAN_7X7_08U, 48, gScenarioKernelData61_GAUSSIAN_7X7_08U, 8},
{4, 3, 168, 0, 1, gScenarioBufferData62_GAUSSIAN_7X7_08U, 48, gScenarioKernelData62_GAUSSIAN_7X7_08U, 8},
{6, 3, 228, 0, 1, gScenarioBufferData63_GAUSSIAN_7X7_08U, 48, gScenarioKernelData63_GAUSSIAN_7X7_08U, 8},
{8, 3, 288, 0, 1, gScenarioBufferData64_GAUSSIAN_7X7_08U, 48, gScenarioKernelData64_GAUSSIAN_7X7_08U, 8},
{10, 3, 348, 0, 1, gScenarioBufferData65_GAUSSIAN_7X7_08U, 48, gScenarioKernelData65_GAUSSIAN_7X7_08U, 8},
{12, 3, 408, 0, 1, gScenarioBufferData66_GAUSSIAN_7X7_08U, 48, gScenarioKernelData66_GAUSSIAN_7X7_08U, 8},
{14, 3, 468, 0, 1, gScenarioBufferData67_GAUSSIAN_7X7_08U, 48, gScenarioKernelData67_GAUSSIAN_7X7_08U, 8},
{16, 3, 528, 0, 1, gScenarioBufferData68_GAUSSIAN_7X7_08U, 48, gScenarioKernelData68_GAUSSIAN_7X7_08U, 8},
{18, 3, 588, 0, 1, gScenarioBufferData69_GAUSSIAN_7X7_08U, 48, gScenarioKernelData69_GAUSSIAN_7X7_08U, 8},
{20, 3, 648, 0, 1, gScenarioBufferData70_GAUSSIAN_7X7_08U, 48, gScenarioKernelData70_GAUSSIAN_7X7_08U, 8},
{22, 3, 708, 0, 1, gScenarioBufferData71_GAUSSIAN_7X7_08U, 48, gScenarioKernelData71_GAUSSIAN_7X7_08U, 8},
{24, 3, 768, 0, 1, gScenarioBufferData72_GAUSSIAN_7X7_08U, 48, gScenarioKernelData72_GAUSSIAN_7X7_08U, 8},
{26, 3, 828, 0, 1, gScenarioBufferData73_GAUSSIAN_7X7_08U, 48, gScenarioKernelData73_GAUSSIAN_7X7_08U, 8},
{28, 3, 888, 0, 1, gScenarioBufferData74_GAUSSIAN_7X7_08U, 48, gScenarioKernelData74_GAUSSIAN_7X7_08U, 8},
{30, 3, 948, 0, 1, gScenarioBufferData75_GAUSSIAN_7X7_08U, 48, gScenarioKernelData75_GAUSSIAN_7X7_08U, 8},
{32, 3, 1008, 0, 1, gScenarioBufferData76_GAUSSIAN_7X7_08U, 48, gScenarioKernelData76_GAUSSIAN_7X7_08U, 8},
{36, 3, 1128, 0, 1, gScenarioBufferData77_GAUSSIAN_7X7_08U, 48, gScenarioKernelData77_GAUSSIAN_7X7_08U, 8},
{40, 3, 1248, 0, 1, gScenarioBufferData78_GAUSSIAN_7X7_08U, 48, gScenarioKernelData78_GAUSSIAN_7X7_08U, 8},
{44, 3, 1368, 0, 1, gScenarioBufferData79_GAUSSIAN_7X7_08U, 48, gScenarioKernelData79_GAUSSIAN_7X7_08U, 8},
{48, 3, 1488, 0, 1, gScenarioBufferData80_GAUSSIAN_7X7_08U, 48, gScenarioKernelData80_GAUSSIAN_7X7_08U, 8},
{52, 3, 1608, 0, 1, gScenarioBufferData81_GAUSSIAN_7X7_08U, 48, gScenarioKernelData81_GAUSSIAN_7X7_08U, 8},
{56, 3, 1728, 0, 1, gScenarioBufferData82_GAUSSIAN_7X7_08U, 48, gScenarioKernelData82_GAUSSIAN_7X7_08U, 8},
{60, 3, 1848, 0, 1, gScenarioBufferData83_GAUSSIAN_7X7_08U, 48, gScenarioKernelData83_GAUSSIAN_7X7_08U, 8},
{64, 3, 1968, 0, 1, gScenarioBufferData84_GAUSSIAN_7X7_08U, 48, gScenarioKernelData84_GAUSSIAN_7X7_08U, 8},
{72, 3, 2208, 0, 1, gScenarioBufferData85_GAUSSIAN_7X7_08U, 48, gScenarioKernelData85_GAUSSIAN_7X7_08U, 8},
{80, 3, 2448, 0, 1, gScenarioBufferData86_GAUSSIAN_7X7_08U, 48, gScenarioKernelData86_GAUSSIAN_7X7_08U, 8},
{88, 3, 2688, 0, 1, gScenarioBufferData87_GAUSSIAN_7X7_08U, 48, gScenarioKernelData87_GAUSSIAN_7X7_08U, 8},
{96, 3, 2928, 0, 1, gScenarioBufferData88_GAUSSIAN_7X7_08U, 48, gScenarioKernelData88_GAUSSIAN_7X7_08U, 8},
{104, 3, 3168, 0, 1, gScenarioBufferData89_GAUSSIAN_7X7_08U, 48, gScenarioKernelData89_GAUSSIAN_7X7_08U, 8},
{112, 3, 3408, 0, 1, gScenarioBufferData90_GAUSSIAN_7X7_08U, 48, gScenarioKernelData90_GAUSSIAN_7X7_08U, 8},
{120, 3, 3648, 0, 1, gScenarioBufferData91_GAUSSIAN_7X7_08U, 48, gScenarioKernelData91_GAUSSIAN_7X7_08U, 8},
{4, 4, 216, 0, 1, gScenarioBufferData92_GAUSSIAN_7X7_08U, 48, gScenarioKernelData92_GAUSSIAN_7X7_08U, 8},
{6, 4, 292, 0, 1, gScenarioBufferData93_GAUSSIAN_7X7_08U, 48, gScenarioKernelData93_GAUSSIAN_7X7_08U, 8},
{8, 4, 368, 0, 1, gScenarioBufferData94_GAUSSIAN_7X7_08U, 48, gScenarioKernelData94_GAUSSIAN_7X7_08U, 8},
{10, 4, 444, 0, 1, gScenarioBufferData95_GAUSSIAN_7X7_08U, 48, gScenarioKernelData95_GAUSSIAN_7X7_08U, 8},
{12, 4, 520, 0, 1, gScenarioBufferData96_GAUSSIAN_7X7_08U, 48, gScenarioKernelData96_GAUSSIAN_7X7_08U, 8},
{14, 4, 596, 0, 1, gScenarioBufferData97_GAUSSIAN_7X7_08U, 48, gScenarioKernelData97_GAUSSIAN_7X7_08U, 8},
{16, 4, 672, 0, 1, gScenarioBufferData98_GAUSSIAN_7X7_08U, 48, gScenarioKernelData98_GAUSSIAN_7X7_08U, 8},
{18, 4, 748, 0, 1, gScenarioBufferData99_GAUSSIAN_7X7_08U, 48, gScenarioKernelData99_GAUSSIAN_7X7_08U, 8},
{20, 4, 824, 0, 1, gScenarioBufferData100_GAUSSIAN_7X7_08U, 48, gScenarioKernelData100_GAUSSIAN_7X7_08U, 8},
{22, 4, 900, 0, 1, gScenarioBufferData101_GAUSSIAN_7X7_08U, 48, gScenarioKernelData101_GAUSSIAN_7X7_08U, 8},
{24, 4, 976, 0, 1, gScenarioBufferData102_GAUSSIAN_7X7_08U, 48, gScenarioKernelData102_GAUSSIAN_7X7_08U, 8},
{26, 4, 1052, 0, 1, gScenarioBufferData103_GAUSSIAN_7X7_08U, 48, gScenarioKernelData103_GAUSSIAN_7X7_08U, 8},
{28, 4, 1128, 0, 1, gScenarioBufferData104_GAUSSIAN_7X7_08U, 48, gScenarioKernelData104_GAUSSIAN_7X7_08U, 8},
{30, 4, 1204, 0, 1, gScenarioBufferData105_GAUSSIAN_7X7_08U, 48, gScenarioKernelData105_GAUSSIAN_7X7_08U, 8},
{32, 4, 1280, 0, 1, gScenarioBufferData106_GAUSSIAN_7X7_08U, 48, gScenarioKernelData106_GAUSSIAN_7X7_08U, 8},
{36, 4, 1432, 0, 1, gScenarioBufferData107_GAUSSIAN_7X7_08U, 48, gScenarioKernelData107_GAUSSIAN_7X7_08U, 8},
{40, 4, 1584, 0, 1, gScenarioBufferData108_GAUSSIAN_7X7_08U, 48, gScenarioKernelData108_GAUSSIAN_7X7_08U, 8},
{44, 4, 1736, 0, 1, gScenarioBufferData109_GAUSSIAN_7X7_08U, 48, gScenarioKernelData109_GAUSSIAN_7X7_08U, 8},
{48, 4, 1888, 0, 1, gScenarioBufferData110_GAUSSIAN_7X7_08U, 48, gScenarioKernelData110_GAUSSIAN_7X7_08U, 8},
{52, 4, 2040, 0, 1, gScenarioBufferData111_GAUSSIAN_7X7_08U, 48, gScenarioKernelData111_GAUSSIAN_7X7_08U, 8},
{56, 4, 2192, 0, 1, gScenarioBufferData112_GAUSSIAN_7X7_08U, 48, gScenarioKernelData112_GAUSSIAN_7X7_08U, 8},
{60, 4, 2344, 0, 1, gScenarioBufferData113_GAUSSIAN_7X7_08U, 48, gScenarioKernelData113_GAUSSIAN_7X7_08U, 8},
{64, 4, 2496, 0, 1, gScenarioBufferData114_GAUSSIAN_7X7_08U, 48, gScenarioKernelData114_GAUSSIAN_7X7_08U, 8},
{72, 4, 2800, 0, 1, gScenarioBufferData115_GAUSSIAN_7X7_08U, 48, gScenarioKernelData115_GAUSSIAN_7X7_08U, 8},
{80, 4, 3104, 0, 1, gScenarioBufferData116_GAUSSIAN_7X7_08U, 48, gScenarioKernelData116_GAUSSIAN_7X7_08U, 8},
{88, 4, 3408, 0, 1, gScenarioBufferData117_GAUSSIAN_7X7_08U, 48, gScenarioKernelData117_GAUSSIAN_7X7_08U, 8},
{96, 4, 3712, 0, 1, gScenarioBufferData118_GAUSSIAN_7X7_08U, 48, gScenarioKernelData118_GAUSSIAN_7X7_08U, 8},
{4, 5, 264, 0, 1, gScenarioBufferData119_GAUSSIAN_7X7_08U, 48, gScenarioKernelData119_GAUSSIAN_7X7_08U, 8},
{6, 5, 356, 0, 1, gScenarioBufferData120_GAUSSIAN_7X7_08U, 48, gScenarioKernelData120_GAUSSIAN_7X7_08U, 8},
{8, 5, 448, 0, 1, gScenarioBufferData121_GAUSSIAN_7X7_08U, 48, gScenarioKernelData121_GAUSSIAN_7X7_08U, 8},
{10, 5, 540, 0, 1, gScenarioBufferData122_GAUSSIAN_7X7_08U, 48, gScenarioKernelData122_GAUSSIAN_7X7_08U, 8},
{12, 5, 632, 0, 1, gScenarioBufferData123_GAUSSIAN_7X7_08U, 48, gScenarioKernelData123_GAUSSIAN_7X7_08U, 8},
{14, 5, 724, 0, 1, gScenarioBufferData124_GAUSSIAN_7X7_08U, 48, gScenarioKernelData124_GAUSSIAN_7X7_08U, 8},
{16, 5, 816, 0, 1, gScenarioBufferData125_GAUSSIAN_7X7_08U, 48, gScenarioKernelData125_GAUSSIAN_7X7_08U, 8},
{18, 5, 908, 0, 1, gScenarioBufferData126_GAUSSIAN_7X7_08U, 48, gScenarioKernelData126_GAUSSIAN_7X7_08U, 8},
{20, 5, 1000, 0, 1, gScenarioBufferData127_GAUSSIAN_7X7_08U, 48, gScenarioKernelData127_GAUSSIAN_7X7_08U, 8},
{22, 5, 1092, 0, 1, gScenarioBufferData128_GAUSSIAN_7X7_08U, 48, gScenarioKernelData128_GAUSSIAN_7X7_08U, 8},
{24, 5, 1184, 0, 1, gScenarioBufferData129_GAUSSIAN_7X7_08U, 48, gScenarioKernelData129_GAUSSIAN_7X7_08U, 8},
{26, 5, 1276, 0, 1, gScenarioBufferData130_GAUSSIAN_7X7_08U, 48, gScenarioKernelData130_GAUSSIAN_7X7_08U, 8},
{28, 5, 1368, 0, 1, gScenarioBufferData131_GAUSSIAN_7X7_08U, 48, gScenarioKernelData131_GAUSSIAN_7X7_08U, 8},
{30, 5, 1460, 0, 1, gScenarioBufferData132_GAUSSIAN_7X7_08U, 48, gScenarioKernelData132_GAUSSIAN_7X7_08U, 8},
{32, 5, 1552, 0, 1, gScenarioBufferData133_GAUSSIAN_7X7_08U, 48, gScenarioKernelData133_GAUSSIAN_7X7_08U, 8},
{36, 5, 1736, 0, 1, gScenarioBufferData134_GAUSSIAN_7X7_08U, 48, gScenarioKernelData134_GAUSSIAN_7X7_08U, 8},
{40, 5, 1920, 0, 1, gScenarioBufferData135_GAUSSIAN_7X7_08U, 48, gScenarioKernelData135_GAUSSIAN_7X7_08U, 8},
{44, 5, 2104, 0, 1, gScenarioBufferData136_GAUSSIAN_7X7_08U, 48, gScenarioKernelData136_GAUSSIAN_7X7_08U, 8},
{48, 5, 2288, 0, 1, gScenarioBufferData137_GAUSSIAN_7X7_08U, 48, gScenarioKernelData137_GAUSSIAN_7X7_08U, 8},
{52, 5, 2472, 0, 1, gScenarioBufferData138_GAUSSIAN_7X7_08U, 48, gScenarioKernelData138_GAUSSIAN_7X7_08U, 8},
{56, 5, 2656, 0, 1, gScenarioBufferData139_GAUSSIAN_7X7_08U, 48, gScenarioKernelData139_GAUSSIAN_7X7_08U, 8},
{60, 5, 2840, 0, 1, gScenarioBufferData140_GAUSSIAN_7X7_08U, 48, gScenarioKernelData140_GAUSSIAN_7X7_08U, 8},
{64, 5, 3024, 0, 1, gScenarioBufferData141_GAUSSIAN_7X7_08U, 48, gScenarioKernelData141_GAUSSIAN_7X7_08U, 8},
{72, 5, 3392, 0, 1, gScenarioBufferData142_GAUSSIAN_7X7_08U, 48, gScenarioKernelData142_GAUSSIAN_7X7_08U, 8},
{80, 5, 3760, 0, 1, gScenarioBufferData143_GAUSSIAN_7X7_08U, 48, gScenarioKernelData143_GAUSSIAN_7X7_08U, 8},
{4, 6, 312, 0, 1, gScenarioBufferData144_GAUSSIAN_7X7_08U, 48, gScenarioKernelData144_GAUSSIAN_7X7_08U, 8},
{6, 6, 420, 0, 1, gScenarioBufferData145_GAUSSIAN_7X7_08U, 48, gScenarioKernelData145_GAUSSIAN_7X7_08U, 8},
{8, 6, 528, 0, 1, gScenarioBufferData146_GAUSSIAN_7X7_08U, 48, gScenarioKernelData146_GAUSSIAN_7X7_08U, 8},
{10, 6, 636, 0, 1, gScenarioBufferData147_GAUSSIAN_7X7_08U, 48, gScenarioKernelData147_GAUSSIAN_7X7_08U, 8},
{12, 6, 744, 0, 1, gScenarioBufferData148_GAUSSIAN_7X7_08U, 48, gScenarioKernelData148_GAUSSIAN_7X7_08U, 8},
{14, 6, 852, 0, 1, gScenarioBufferData149_GAUSSIAN_7X7_08U, 48, gScenarioKernelData149_GAUSSIAN_7X7_08U, 8},
{16, 6, 960, 0, 1, gScenarioBufferData150_GAUSSIAN_7X7_08U, 48, gScenarioKernelData150_GAUSSIAN_7X7_08U, 8},
{18, 6, 1068, 0, 1, gScenarioBufferData151_GAUSSIAN_7X7_08U, 48, gScenarioKernelData151_GAUSSIAN_7X7_08U, 8},
{20, 6, 1176, 0, 1, gScenarioBufferData152_GAUSSIAN_7X7_08U, 48, gScenarioKernelData152_GAUSSIAN_7X7_08U, 8},
{22, 6, 1284, 0, 1, gScenarioBufferData153_GAUSSIAN_7X7_08U, 48, gScenarioKernelData153_GAUSSIAN_7X7_08U, 8},
{24, 6, 1392, 0, 1, gScenarioBufferData154_GAUSSIAN_7X7_08U, 48, gScenarioKernelData154_GAUSSIAN_7X7_08U, 8},
{26, 6, 1500, 0, 1, gScenarioBufferData155_GAUSSIAN_7X7_08U, 48, gScenarioKernelData155_GAUSSIAN_7X7_08U, 8},
{28, 6, 1608, 0, 1, gScenarioBufferData156_GAUSSIAN_7X7_08U, 48, gScenarioKernelData156_GAUSSIAN_7X7_08U, 8},
{30, 6, 1716, 0, 1, gScenarioBufferData157_GAUSSIAN_7X7_08U, 48, gScenarioKernelData157_GAUSSIAN_7X7_08U, 8},
{32, 6, 1824, 0, 1, gScenarioBufferData158_GAUSSIAN_7X7_08U, 48, gScenarioKernelData158_GAUSSIAN_7X7_08U, 8},
{36, 6, 2040, 0, 1, gScenarioBufferData159_GAUSSIAN_7X7_08U, 48, gScenarioKernelData159_GAUSSIAN_7X7_08U, 8},
{40, 6, 2256, 0, 1, gScenarioBufferData160_GAUSSIAN_7X7_08U, 48, gScenarioKernelData160_GAUSSIAN_7X7_08U, 8},
{44, 6, 2472, 0, 1, gScenarioBufferData161_GAUSSIAN_7X7_08U, 48, gScenarioKernelData161_GAUSSIAN_7X7_08U, 8},
{48, 6, 2688, 0, 1, gScenarioBufferData162_GAUSSIAN_7X7_08U, 48, gScenarioKernelData162_GAUSSIAN_7X7_08U, 8},
{52, 6, 2904, 0, 1, gScenarioBufferData163_GAUSSIAN_7X7_08U, 48, gScenarioKernelData163_GAUSSIAN_7X7_08U, 8},
{56, 6, 3120, 0, 1, gScenarioBufferData164_GAUSSIAN_7X7_08U, 48, gScenarioKernelData164_GAUSSIAN_7X7_08U, 8},
{60, 6, 3336, 0, 1, gScenarioBufferData165_GAUSSIAN_7X7_08U, 48, gScenarioKernelData165_GAUSSIAN_7X7_08U, 8},
{64, 6, 3552, 0, 1, gScenarioBufferData166_GAUSSIAN_7X7_08U, 48, gScenarioKernelData166_GAUSSIAN_7X7_08U, 8},
{4, 8, 408, 0, 1, gScenarioBufferData167_GAUSSIAN_7X7_08U, 48, gScenarioKernelData167_GAUSSIAN_7X7_08U, 8},
{6, 8, 548, 0, 1, gScenarioBufferData168_GAUSSIAN_7X7_08U, 48, gScenarioKernelData168_GAUSSIAN_7X7_08U, 8},
{8, 8, 688, 0, 1, gScenarioBufferData169_GAUSSIAN_7X7_08U, 48, gScenarioKernelData169_GAUSSIAN_7X7_08U, 8},
{10, 8, 828, 0, 1, gScenarioBufferData170_GAUSSIAN_7X7_08U, 48, gScenarioKernelData170_GAUSSIAN_7X7_08U, 8},
{12, 8, 968, 0, 1, gScenarioBufferData171_GAUSSIAN_7X7_08U, 48, gScenarioKernelData171_GAUSSIAN_7X7_08U, 8},
{14, 8, 1108, 0, 1, gScenarioBufferData172_GAUSSIAN_7X7_08U, 48, gScenarioKernelData172_GAUSSIAN_7X7_08U, 8},
{16, 8, 1248, 0, 1, gScenarioBufferData173_GAUSSIAN_7X7_08U, 48, gScenarioKernelData173_GAUSSIAN_7X7_08U, 8},
{18, 8, 1388, 0, 1, gScenarioBufferData174_GAUSSIAN_7X7_08U, 48, gScenarioKernelData174_GAUSSIAN_7X7_08U, 8},
{20, 8, 1528, 0, 1, gScenarioBufferData175_GAUSSIAN_7X7_08U, 48, gScenarioKernelData175_GAUSSIAN_7X7_08U, 8},
{22, 8, 1668, 0, 1, gScenarioBufferData176_GAUSSIAN_7X7_08U, 48, gScenarioKernelData176_GAUSSIAN_7X7_08U, 8},
{24, 8, 1808, 0, 1, gScenarioBufferData177_GAUSSIAN_7X7_08U, 48, gScenarioKernelData177_GAUSSIAN_7X7_08U, 8},
{26, 8, 1948, 0, 1, gScenarioBufferData178_GAUSSIAN_7X7_08U, 48, gScenarioKernelData178_GAUSSIAN_7X7_08U, 8},
{28, 8, 2088, 0, 1, gScenarioBufferData179_GAUSSIAN_7X7_08U, 48, gScenarioKernelData179_GAUSSIAN_7X7_08U, 8},
{30, 8, 2228, 0, 1, gScenarioBufferData180_GAUSSIAN_7X7_08U, 48, gScenarioKernelData180_GAUSSIAN_7X7_08U, 8},
{32, 8, 2368, 0, 1, gScenarioBufferData181_GAUSSIAN_7X7_08U, 48, gScenarioKernelData181_GAUSSIAN_7X7_08U, 8},
{36, 8, 2648, 0, 1, gScenarioBufferData182_GAUSSIAN_7X7_08U, 48, gScenarioKernelData182_GAUSSIAN_7X7_08U, 8},
{40, 8, 2928, 0, 1, gScenarioBufferData183_GAUSSIAN_7X7_08U, 48, gScenarioKernelData183_GAUSSIAN_7X7_08U, 8},
{44, 8, 3208, 0, 1, gScenarioBufferData184_GAUSSIAN_7X7_08U, 48, gScenarioKernelData184_GAUSSIAN_7X7_08U, 8},
{48, 8, 3488, 0, 1, gScenarioBufferData185_GAUSSIAN_7X7_08U, 48, gScenarioKernelData185_GAUSSIAN_7X7_08U, 8},
{52, 8, 3768, 0, 1, gScenarioBufferData186_GAUSSIAN_7X7_08U, 48, gScenarioKernelData186_GAUSSIAN_7X7_08U, 8},
{4, 10, 504, 0, 1, gScenarioBufferData187_GAUSSIAN_7X7_08U, 48, gScenarioKernelData187_GAUSSIAN_7X7_08U, 8},
{6, 10, 676, 0, 1, gScenarioBufferData188_GAUSSIAN_7X7_08U, 48, gScenarioKernelData188_GAUSSIAN_7X7_08U, 8},
{8, 10, 848, 0, 1, gScenarioBufferData189_GAUSSIAN_7X7_08U, 48, gScenarioKernelData189_GAUSSIAN_7X7_08U, 8},
{10, 10, 1020, 0, 1, gScenarioBufferData190_GAUSSIAN_7X7_08U, 48, gScenarioKernelData190_GAUSSIAN_7X7_08U, 8},
{12, 10, 1192, 0, 1, gScenarioBufferData191_GAUSSIAN_7X7_08U, 48, gScenarioKernelData191_GAUSSIAN_7X7_08U, 8},
{14, 10, 1364, 0, 1, gScenarioBufferData192_GAUSSIAN_7X7_08U, 48, gScenarioKernelData192_GAUSSIAN_7X7_08U, 8},
{16, 10, 1536, 0, 1, gScenarioBufferData193_GAUSSIAN_7X7_08U, 48, gScenarioKernelData193_GAUSSIAN_7X7_08U, 8},
{18, 10, 1708, 0, 1, gScenarioBufferData194_GAUSSIAN_7X7_08U, 48, gScenarioKernelData194_GAUSSIAN_7X7_08U, 8},
{20, 10, 1880, 0, 1, gScenarioBufferData195_GAUSSIAN_7X7_08U, 48, gScenarioKernelData195_GAUSSIAN_7X7_08U, 8},
{22, 10, 2052, 0, 1, gScenarioBufferData196_GAUSSIAN_7X7_08U, 48, gScenarioKernelData196_GAUSSIAN_7X7_08U, 8},
{24, 10, 2224, 0, 1, gScenarioBufferData197_GAUSSIAN_7X7_08U, 48, gScenarioKernelData197_GAUSSIAN_7X7_08U, 8},
{26, 10, 2396, 0, 1, gScenarioBufferData198_GAUSSIAN_7X7_08U, 48, gScenarioKernelData198_GAUSSIAN_7X7_08U, 8},
{28, 10, 2568, 0, 1, gScenarioBufferData199_GAUSSIAN_7X7_08U, 48, gScenarioKernelData199_GAUSSIAN_7X7_08U, 8},
{30, 10, 2740, 0, 1, gScenarioBufferData200_GAUSSIAN_7X7_08U, 48, gScenarioKernelData200_GAUSSIAN_7X7_08U, 8},
{32, 10, 2912, 0, 1, gScenarioBufferData201_GAUSSIAN_7X7_08U, 48, gScenarioKernelData201_GAUSSIAN_7X7_08U, 8},
{36, 10, 3256, 0, 1, gScenarioBufferData202_GAUSSIAN_7X7_08U, 48, gScenarioKernelData202_GAUSSIAN_7X7_08U, 8},
{40, 10, 3600, 0, 1, gScenarioBufferData203_GAUSSIAN_7X7_08U, 48, gScenarioKernelData203_GAUSSIAN_7X7_08U, 8},
{4, 12, 600, 0, 1, gScenarioBufferData204_GAUSSIAN_7X7_08U, 48, gScenarioKernelData204_GAUSSIAN_7X7_08U, 8},
{6, 12, 804, 0, 1, gScenarioBufferData205_GAUSSIAN_7X7_08U, 48, gScenarioKernelData205_GAUSSIAN_7X7_08U, 8},
{8, 12, 1008, 0, 1, gScenarioBufferData206_GAUSSIAN_7X7_08U, 48, gScenarioKernelData206_GAUSSIAN_7X7_08U, 8},
{10, 12, 1212, 0, 1, gScenarioBufferData207_GAUSSIAN_7X7_08U, 48, gScenarioKernelData207_GAUSSIAN_7X7_08U, 8},
{12, 12, 1416, 0, 1, gScenarioBufferData208_GAUSSIAN_7X7_08U, 48, gScenarioKernelData208_GAUSSIAN_7X7_08U, 8},
{14, 12, 1620, 0, 1, gScenarioBufferData209_GAUSSIAN_7X7_08U, 48, gScenarioKernelData209_GAUSSIAN_7X7_08U, 8},
{16, 12, 1824, 0, 1, gScenarioBufferData210_GAUSSIAN_7X7_08U, 48, gScenarioKernelData210_GAUSSIAN_7X7_08U, 8},
{18, 12, 2028, 0, 1, gScenarioBufferData211_GAUSSIAN_7X7_08U, 48, gScenarioKernelData211_GAUSSIAN_7X7_08U, 8},
{20, 12, 2232, 0, 1, gScenarioBufferData212_GAUSSIAN_7X7_08U, 48, gScenarioKernelData212_GAUSSIAN_7X7_08U, 8},
{22, 12, 2436, 0, 1, gScenarioBufferData213_GAUSSIAN_7X7_08U, 48, gScenarioKernelData213_GAUSSIAN_7X7_08U, 8},
{24, 12, 2640, 0, 1, gScenarioBufferData214_GAUSSIAN_7X7_08U, 48, gScenarioKernelData214_GAUSSIAN_7X7_08U, 8},
{26, 12, 2844, 0, 1, gScenarioBufferData215_GAUSSIAN_7X7_08U, 48, gScenarioKernelData215_GAUSSIAN_7X7_08U, 8},
{28, 12, 3048, 0, 1, gScenarioBufferData216_GAUSSIAN_7X7_08U, 48, gScenarioKernelData216_GAUSSIAN_7X7_08U, 8},
{30, 12, 3252, 0, 1, gScenarioBufferData217_GAUSSIAN_7X7_08U, 48, gScenarioKernelData217_GAUSSIAN_7X7_08U, 8},
{32, 12, 3456, 0, 1, gScenarioBufferData218_GAUSSIAN_7X7_08U, 48, gScenarioKernelData218_GAUSSIAN_7X7_08U, 8},
{4, 14, 696, 0, 1, gScenarioBufferData219_GAUSSIAN_7X7_08U, 48, gScenarioKernelData219_GAUSSIAN_7X7_08U, 8},
{6, 14, 932, 0, 1, gScenarioBufferData220_GAUSSIAN_7X7_08U, 48, gScenarioKernelData220_GAUSSIAN_7X7_08U, 8},
{8, 14, 1168, 0, 1, gScenarioBufferData221_GAUSSIAN_7X7_08U, 48, gScenarioKernelData221_GAUSSIAN_7X7_08U, 8},
{10, 14, 1404, 0, 1, gScenarioBufferData222_GAUSSIAN_7X7_08U, 48, gScenarioKernelData222_GAUSSIAN_7X7_08U, 8},
{12, 14, 1640, 0, 1, gScenarioBufferData223_GAUSSIAN_7X7_08U, 48, gScenarioKernelData223_GAUSSIAN_7X7_08U, 8},
{14, 14, 1876, 0, 1, gScenarioBufferData224_GAUSSIAN_7X7_08U, 48, gScenarioKernelData224_GAUSSIAN_7X7_08U, 8},
{16, 14, 2112, 0, 1, gScenarioBufferData225_GAUSSIAN_7X7_08U, 48, gScenarioKernelData225_GAUSSIAN_7X7_08U, 8},
{18, 14, 2348, 0, 1, gScenarioBufferData226_GAUSSIAN_7X7_08U, 48, gScenarioKernelData226_GAUSSIAN_7X7_08U, 8},
{20, 14, 2584, 0, 1, gScenarioBufferData227_GAUSSIAN_7X7_08U, 48, gScenarioKernelData227_GAUSSIAN_7X7_08U, 8},
{22, 14, 2820, 0, 1, gScenarioBufferData228_GAUSSIAN_7X7_08U, 48, gScenarioKernelData228_GAUSSIAN_7X7_08U, 8},
{24, 14, 3056, 0, 1, gScenarioBufferData229_GAUSSIAN_7X7_08U, 48, gScenarioKernelData229_GAUSSIAN_7X7_08U, 8},
{26, 14, 3292, 0, 1, gScenarioBufferData230_GAUSSIAN_7X7_08U, 48, gScenarioKernelData230_GAUSSIAN_7X7_08U, 8},
{28, 14, 3528, 0, 1, gScenarioBufferData231_GAUSSIAN_7X7_08U, 48, gScenarioKernelData231_GAUSSIAN_7X7_08U, 8},
{30, 14, 3764, 0, 1, gScenarioBufferData232_GAUSSIAN_7X7_08U, 48, gScenarioKernelData232_GAUSSIAN_7X7_08U, 8},
{4, 16, 792, 0, 1, gScenarioBufferData233_GAUSSIAN_7X7_08U, 48, gScenarioKernelData233_GAUSSIAN_7X7_08U, 8},
{6, 16, 1060, 0, 1, gScenarioBufferData234_GAUSSIAN_7X7_08U, 48, gScenarioKernelData234_GAUSSIAN_7X7_08U, 8},
{8, 16, 1328, 0, 1, gScenarioBufferData235_GAUSSIAN_7X7_08U, 48, gScenarioKernelData235_GAUSSIAN_7X7_08U, 8},
{10, 16, 1596, 0, 1, gScenarioBufferData236_GAUSSIAN_7X7_08U, 48, gScenarioKernelData236_GAUSSIAN_7X7_08U, 8},
{12, 16, 1864, 0, 1, gScenarioBufferData237_GAUSSIAN_7X7_08U, 48, gScenarioKernelData237_GAUSSIAN_7X7_08U, 8},
{14, 16, 2132, 0, 1, gScenarioBufferData238_GAUSSIAN_7X7_08U, 48, gScenarioKernelData238_GAUSSIAN_7X7_08U, 8},
{16, 16, 2400, 0, 1, gScenarioBufferData239_GAUSSIAN_7X7_08U, 48, gScenarioKernelData239_GAUSSIAN_7X7_08U, 8},
{18, 16, 2668, 0, 1, gScenarioBufferData240_GAUSSIAN_7X7_08U, 48, gScenarioKernelData240_GAUSSIAN_7X7_08U, 8},
{20, 16, 2936, 0, 1, gScenarioBufferData241_GAUSSIAN_7X7_08U, 48, gScenarioKernelData241_GAUSSIAN_7X7_08U, 8},
{22, 16, 3204, 0, 1, gScenarioBufferData242_GAUSSIAN_7X7_08U, 48, gScenarioKernelData242_GAUSSIAN_7X7_08U, 8},
{24, 16, 3472, 0, 1, gScenarioBufferData243_GAUSSIAN_7X7_08U, 48, gScenarioKernelData243_GAUSSIAN_7X7_08U, 8},
{26, 16, 3740, 0, 1, gScenarioBufferData244_GAUSSIAN_7X7_08U, 48, gScenarioKernelData244_GAUSSIAN_7X7_08U, 8},
{4, 18, 888, 0, 1, gScenarioBufferData245_GAUSSIAN_7X7_08U, 48, gScenarioKernelData245_GAUSSIAN_7X7_08U, 8},
{6, 18, 1188, 0, 1, gScenarioBufferData246_GAUSSIAN_7X7_08U, 48, gScenarioKernelData246_GAUSSIAN_7X7_08U, 8},
{8, 18, 1488, 0, 1, gScenarioBufferData247_GAUSSIAN_7X7_08U, 48, gScenarioKernelData247_GAUSSIAN_7X7_08U, 8},
{10, 18, 1788, 0, 1, gScenarioBufferData248_GAUSSIAN_7X7_08U, 48, gScenarioKernelData248_GAUSSIAN_7X7_08U, 8},
{12, 18, 2088, 0, 1, gScenarioBufferData249_GAUSSIAN_7X7_08U, 48, gScenarioKernelData249_GAUSSIAN_7X7_08U, 8},
{14, 18, 2388, 0, 1, gScenarioBufferData250_GAUSSIAN_7X7_08U, 48, gScenarioKernelData250_GAUSSIAN_7X7_08U, 8},
{16, 18, 2688, 0, 1, gScenarioBufferData251_GAUSSIAN_7X7_08U, 48, gScenarioKernelData251_GAUSSIAN_7X7_08U, 8},
{18, 18, 2988, 0, 1, gScenarioBufferData252_GAUSSIAN_7X7_08U, 48, gScenarioKernelData252_GAUSSIAN_7X7_08U, 8},
{20, 18, 3288, 0, 1, gScenarioBufferData253_GAUSSIAN_7X7_08U, 48, gScenarioKernelData253_GAUSSIAN_7X7_08U, 8},
{22, 18, 3588, 0, 1, gScenarioBufferData254_GAUSSIAN_7X7_08U, 48, gScenarioKernelData254_GAUSSIAN_7X7_08U, 8},
{4, 20, 984, 0, 1, gScenarioBufferData255_GAUSSIAN_7X7_08U, 48, gScenarioKernelData255_GAUSSIAN_7X7_08U, 8},
{6, 20, 1316, 0, 1, gScenarioBufferData256_GAUSSIAN_7X7_08U, 48, gScenarioKernelData256_GAUSSIAN_7X7_08U, 8},
{8, 20, 1648, 0, 1, gScenarioBufferData257_GAUSSIAN_7X7_08U, 48, gScenarioKernelData257_GAUSSIAN_7X7_08U, 8},
{10, 20, 1980, 0, 1, gScenarioBufferData258_GAUSSIAN_7X7_08U, 48, gScenarioKernelData258_GAUSSIAN_7X7_08U, 8},
{12, 20, 2312, 0, 1, gScenarioBufferData259_GAUSSIAN_7X7_08U, 48, gScenarioKernelData259_GAUSSIAN_7X7_08U, 8},
{14, 20, 2644, 0, 1, gScenarioBufferData260_GAUSSIAN_7X7_08U, 48, gScenarioKernelData260_GAUSSIAN_7X7_08U, 8},
{16, 20, 2976, 0, 1, gScenarioBufferData261_GAUSSIAN_7X7_08U, 48, gScenarioKernelData261_GAUSSIAN_7X7_08U, 8},
{18, 20, 3308, 0, 1, gScenarioBufferData262_GAUSSIAN_7X7_08U, 48, gScenarioKernelData262_GAUSSIAN_7X7_08U, 8},
{20, 20, 3640, 0, 1, gScenarioBufferData263_GAUSSIAN_7X7_08U, 48, gScenarioKernelData263_GAUSSIAN_7X7_08U, 8},
{4, 22, 1080, 0, 1, gScenarioBufferData264_GAUSSIAN_7X7_08U, 48, gScenarioKernelData264_GAUSSIAN_7X7_08U, 8},
{6, 22, 1444, 0, 1, gScenarioBufferData265_GAUSSIAN_7X7_08U, 48, gScenarioKernelData265_GAUSSIAN_7X7_08U, 8},
{8, 22, 1808, 0, 1, gScenarioBufferData266_GAUSSIAN_7X7_08U, 48, gScenarioKernelData266_GAUSSIAN_7X7_08U, 8},
{10, 22, 2172, 0, 1, gScenarioBufferData267_GAUSSIAN_7X7_08U, 48, gScenarioKernelData267_GAUSSIAN_7X7_08U, 8},
{12, 22, 2536, 0, 1, gScenarioBufferData268_GAUSSIAN_7X7_08U, 48, gScenarioKernelData268_GAUSSIAN_7X7_08U, 8},
{14, 22, 2900, 0, 1, gScenarioBufferData269_GAUSSIAN_7X7_08U, 48, gScenarioKernelData269_GAUSSIAN_7X7_08U, 8},
{16, 22, 3264, 0, 1, gScenarioBufferData270_GAUSSIAN_7X7_08U, 48, gScenarioKernelData270_GAUSSIAN_7X7_08U, 8},
{18, 22, 3628, 0, 1, gScenarioBufferData271_GAUSSIAN_7X7_08U, 48, gScenarioKernelData271_GAUSSIAN_7X7_08U, 8},
{4, 24, 1176, 0, 1, gScenarioBufferData272_GAUSSIAN_7X7_08U, 48, gScenarioKernelData272_GAUSSIAN_7X7_08U, 8},
{6, 24, 1572, 0, 1, gScenarioBufferData273_GAUSSIAN_7X7_08U, 48, gScenarioKernelData273_GAUSSIAN_7X7_08U, 8},
{8, 24, 1968, 0, 1, gScenarioBufferData274_GAUSSIAN_7X7_08U, 48, gScenarioKernelData274_GAUSSIAN_7X7_08U, 8},
{10, 24, 2364, 0, 1, gScenarioBufferData275_GAUSSIAN_7X7_08U, 48, gScenarioKernelData275_GAUSSIAN_7X7_08U, 8},
{12, 24, 2760, 0, 1, gScenarioBufferData276_GAUSSIAN_7X7_08U, 48, gScenarioKernelData276_GAUSSIAN_7X7_08U, 8},
{14, 24, 3156, 0, 1, gScenarioBufferData277_GAUSSIAN_7X7_08U, 48, gScenarioKernelData277_GAUSSIAN_7X7_08U, 8},
{16, 24, 3552, 0, 1, gScenarioBufferData278_GAUSSIAN_7X7_08U, 48, gScenarioKernelData278_GAUSSIAN_7X7_08U, 8},
{4, 26, 1272, 0, 1, gScenarioBufferData279_GAUSSIAN_7X7_08U, 48, gScenarioKernelData279_GAUSSIAN_7X7_08U, 8},
{6, 26, 1700, 0, 1, gScenarioBufferData280_GAUSSIAN_7X7_08U, 48, gScenarioKernelData280_GAUSSIAN_7X7_08U, 8},
{8, 26, 2128, 0, 1, gScenarioBufferData281_GAUSSIAN_7X7_08U, 48, gScenarioKernelData281_GAUSSIAN_7X7_08U, 8},
{10, 26, 2556, 0, 1, gScenarioBufferData282_GAUSSIAN_7X7_08U, 48, gScenarioKernelData282_GAUSSIAN_7X7_08U, 8},
{12, 26, 2984, 0, 1, gScenarioBufferData283_GAUSSIAN_7X7_08U, 48, gScenarioKernelData283_GAUSSIAN_7X7_08U, 8},
{14, 26, 3412, 0, 1, gScenarioBufferData284_GAUSSIAN_7X7_08U, 48, gScenarioKernelData284_GAUSSIAN_7X7_08U, 8},
{16, 26, 3840, 0, 1, gScenarioBufferData285_GAUSSIAN_7X7_08U, 48, gScenarioKernelData285_GAUSSIAN_7X7_08U, 8},
{4, 28, 1368, 0, 1, gScenarioBufferData286_GAUSSIAN_7X7_08U, 48, gScenarioKernelData286_GAUSSIAN_7X7_08U, 8},
{6, 28, 1828, 0, 1, gScenarioBufferData287_GAUSSIAN_7X7_08U, 48, gScenarioKernelData287_GAUSSIAN_7X7_08U, 8},
{8, 28, 2288, 0, 1, gScenarioBufferData288_GAUSSIAN_7X7_08U, 48, gScenarioKernelData288_GAUSSIAN_7X7_08U, 8},
{10, 28, 2748, 0, 1, gScenarioBufferData289_GAUSSIAN_7X7_08U, 48, gScenarioKernelData289_GAUSSIAN_7X7_08U, 8},
{12, 28, 3208, 0, 1, gScenarioBufferData290_GAUSSIAN_7X7_08U, 48, gScenarioKernelData290_GAUSSIAN_7X7_08U, 8},
{14, 28, 3668, 0, 1, gScenarioBufferData291_GAUSSIAN_7X7_08U, 48, gScenarioKernelData291_GAUSSIAN_7X7_08U, 8},
{4, 30, 1464, 0, 1, gScenarioBufferData292_GAUSSIAN_7X7_08U, 48, gScenarioKernelData292_GAUSSIAN_7X7_08U, 8},
{6, 30, 1956, 0, 1, gScenarioBufferData293_GAUSSIAN_7X7_08U, 48, gScenarioKernelData293_GAUSSIAN_7X7_08U, 8},
{8, 30, 2448, 0, 1, gScenarioBufferData294_GAUSSIAN_7X7_08U, 48, gScenarioKernelData294_GAUSSIAN_7X7_08U, 8},
{10, 30, 2940, 0, 1, gScenarioBufferData295_GAUSSIAN_7X7_08U, 48, gScenarioKernelData295_GAUSSIAN_7X7_08U, 8},
{12, 30, 3432, 0, 1, gScenarioBufferData296_GAUSSIAN_7X7_08U, 48, gScenarioKernelData296_GAUSSIAN_7X7_08U, 8},
{4, 32, 1560, 0, 1, gScenarioBufferData297_GAUSSIAN_7X7_08U, 48, gScenarioKernelData297_GAUSSIAN_7X7_08U, 8},
{6, 32, 2084, 0, 1, gScenarioBufferData298_GAUSSIAN_7X7_08U, 48, gScenarioKernelData298_GAUSSIAN_7X7_08U, 8},
{8, 32, 2608, 0, 1, gScenarioBufferData299_GAUSSIAN_7X7_08U, 48, gScenarioKernelData299_GAUSSIAN_7X7_08U, 8},
{10, 32, 3132, 0, 1, gScenarioBufferData300_GAUSSIAN_7X7_08U, 48, gScenarioKernelData300_GAUSSIAN_7X7_08U, 8},
{12, 32, 3656, 0, 1, gScenarioBufferData301_GAUSSIAN_7X7_08U, 48, gScenarioKernelData301_GAUSSIAN_7X7_08U, 8}
};

static acf_scenario_list gScenarioList_GAUSSIAN_7X7_08U = {
302, //number of scenarios
gScenarioArray_GAUSSIAN_7X7_08U};
//**************************************************************

class GAUSSIAN_7X7_08U : public ACF_Process_APU
{

public:
   GAUSSIAN_7X7_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("GAUSSIAN_7X7_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("GAUSSIAN_7X7_08U",
                                        GAUSSIAN_7X7_08U_LOAD_SEGMENTS,
                                        GAUSSIAN_7X7_08U_LOAD_PMEM, GAUSSIAN_7X7_08U_LOAD_PMEM_SIZE,
                                        GAUSSIAN_7X7_08U_LOAD_DMEM, GAUSSIAN_7X7_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(GAUSSIAN_7X7_08U_APEX_LOG_BUFFER, GAUSSIAN_7X7_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_GAUSSIAN_7X7_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_GAUSSIAN_7X7_08U

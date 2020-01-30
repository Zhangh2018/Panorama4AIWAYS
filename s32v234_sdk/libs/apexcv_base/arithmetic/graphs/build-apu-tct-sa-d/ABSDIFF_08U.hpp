#ifndef _ACF_PROCESS_APU_ABSDIFF_08U
#define _ACF_PROCESS_APU_ABSDIFF_08U

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <ABSDIFF_08U_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_ABSDIFF_08U[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_ABSDIFF_08U[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_ABSDIFF_08U[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_ABSDIFF_08U[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_ABSDIFF_08U[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_ABSDIFF_08U[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_ABSDIFF_08U[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_ABSDIFF_08U[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_ABSDIFF_08U[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_ABSDIFF_08U[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_ABSDIFF_08U[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_ABSDIFF_08U[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_ABSDIFF_08U[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_ABSDIFF_08U[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_ABSDIFF_08U[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_ABSDIFF_08U[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_ABSDIFF_08U[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_ABSDIFF_08U[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_ABSDIFF_08U[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_ABSDIFF_08U[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_ABSDIFF_08U[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_ABSDIFF_08U[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_ABSDIFF_08U[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_ABSDIFF_08U[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_ABSDIFF_08U[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_ABSDIFF_08U[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_ABSDIFF_08U[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_ABSDIFF_08U[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_ABSDIFF_08U[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_ABSDIFF_08U[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_ABSDIFF_08U[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_ABSDIFF_08U[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_ABSDIFF_08U[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_ABSDIFF_08U[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_ABSDIFF_08U[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_ABSDIFF_08U[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_ABSDIFF_08U[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_ABSDIFF_08U[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_ABSDIFF_08U[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_ABSDIFF_08U[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_ABSDIFF_08U[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_ABSDIFF_08U[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_ABSDIFF_08U[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_ABSDIFF_08U[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_ABSDIFF_08U[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_ABSDIFF_08U[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_ABSDIFF_08U[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_ABSDIFF_08U[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_ABSDIFF_08U[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_ABSDIFF_08U[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_ABSDIFF_08U[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_ABSDIFF_08U[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_ABSDIFF_08U[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_ABSDIFF_08U[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_ABSDIFF_08U[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_ABSDIFF_08U[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_ABSDIFF_08U[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_ABSDIFF_08U[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_ABSDIFF_08U[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_ABSDIFF_08U[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_ABSDIFF_08U[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_ABSDIFF_08U[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_ABSDIFF_08U[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_ABSDIFF_08U[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_ABSDIFF_08U[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_ABSDIFF_08U[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_ABSDIFF_08U[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_ABSDIFF_08U[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_ABSDIFF_08U[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_ABSDIFF_08U[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_ABSDIFF_08U[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_ABSDIFF_08U[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_ABSDIFF_08U[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_ABSDIFF_08U[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_ABSDIFF_08U[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_ABSDIFF_08U[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_ABSDIFF_08U[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_ABSDIFF_08U[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_ABSDIFF_08U[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_ABSDIFF_08U[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_ABSDIFF_08U[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_ABSDIFF_08U[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_ABSDIFF_08U[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_ABSDIFF_08U[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_ABSDIFF_08U[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_ABSDIFF_08U[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_ABSDIFF_08U[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_ABSDIFF_08U[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_ABSDIFF_08U[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_ABSDIFF_08U[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_ABSDIFF_08U[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_ABSDIFF_08U[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_ABSDIFF_08U[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_ABSDIFF_08U[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_ABSDIFF_08U[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_ABSDIFF_08U[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_ABSDIFF_08U[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_ABSDIFF_08U[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_ABSDIFF_08U[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_ABSDIFF_08U[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_ABSDIFF_08U[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_ABSDIFF_08U[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_ABSDIFF_08U[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_ABSDIFF_08U[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_ABSDIFF_08U[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_ABSDIFF_08U[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_ABSDIFF_08U[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_ABSDIFF_08U[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_ABSDIFF_08U[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_ABSDIFF_08U[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_ABSDIFF_08U[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_ABSDIFF_08U[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_ABSDIFF_08U[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_ABSDIFF_08U[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_ABSDIFF_08U[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_ABSDIFF_08U[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_ABSDIFF_08U[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_ABSDIFF_08U[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_ABSDIFF_08U[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_ABSDIFF_08U[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_ABSDIFF_08U[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_ABSDIFF_08U[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_ABSDIFF_08U[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_ABSDIFF_08U[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_ABSDIFF_08U[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_ABSDIFF_08U[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_ABSDIFF_08U[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_ABSDIFF_08U[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_ABSDIFF_08U[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_ABSDIFF_08U[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_ABSDIFF_08U[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_ABSDIFF_08U[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_ABSDIFF_08U[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_ABSDIFF_08U[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_ABSDIFF_08U[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_ABSDIFF_08U[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_ABSDIFF_08U[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_ABSDIFF_08U[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_ABSDIFF_08U[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_ABSDIFF_08U[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_ABSDIFF_08U[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_ABSDIFF_08U[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_ABSDIFF_08U[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_ABSDIFF_08U[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_ABSDIFF_08U[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_ABSDIFF_08U[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_ABSDIFF_08U[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_ABSDIFF_08U[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_ABSDIFF_08U[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_ABSDIFF_08U[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_ABSDIFF_08U[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_ABSDIFF_08U[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_ABSDIFF_08U[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_ABSDIFF_08U[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_ABSDIFF_08U[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_ABSDIFF_08U[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_ABSDIFF_08U[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_ABSDIFF_08U[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_ABSDIFF_08U[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_ABSDIFF_08U[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_ABSDIFF_08U[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_ABSDIFF_08U[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_ABSDIFF_08U[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_ABSDIFF_08U[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_ABSDIFF_08U[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_ABSDIFF_08U[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_ABSDIFF_08U[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_ABSDIFF_08U[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_ABSDIFF_08U[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_ABSDIFF_08U[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_ABSDIFF_08U[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_ABSDIFF_08U[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_ABSDIFF_08U[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_ABSDIFF_08U[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_ABSDIFF_08U[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_ABSDIFF_08U[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_ABSDIFF_08U[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_ABSDIFF_08U[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_ABSDIFF_08U[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_ABSDIFF_08U[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_ABSDIFF_08U[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_ABSDIFF_08U[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_ABSDIFF_08U[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_ABSDIFF_08U[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_ABSDIFF_08U[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_ABSDIFF_08U[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_ABSDIFF_08U[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_ABSDIFF_08U[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_ABSDIFF_08U[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_ABSDIFF_08U[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_ABSDIFF_08U[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_ABSDIFF_08U[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_ABSDIFF_08U[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_ABSDIFF_08U[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_ABSDIFF_08U[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_ABSDIFF_08U[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_ABSDIFF_08U[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_ABSDIFF_08U[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_ABSDIFF_08U[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_ABSDIFF_08U[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_ABSDIFF_08U[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_ABSDIFF_08U[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_ABSDIFF_08U[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_ABSDIFF_08U[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_ABSDIFF_08U[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_ABSDIFF_08U[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_ABSDIFF_08U[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_ABSDIFF_08U[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_ABSDIFF_08U[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_ABSDIFF_08U[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_ABSDIFF_08U[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_ABSDIFF_08U[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_ABSDIFF_08U[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_ABSDIFF_08U[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_ABSDIFF_08U[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_ABSDIFF_08U[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_ABSDIFF_08U[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_ABSDIFF_08U[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_ABSDIFF_08U[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_ABSDIFF_08U[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_ABSDIFF_08U[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_ABSDIFF_08U[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_ABSDIFF_08U[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_ABSDIFF_08U[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_ABSDIFF_08U[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_ABSDIFF_08U[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_ABSDIFF_08U[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_ABSDIFF_08U[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_ABSDIFF_08U[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_ABSDIFF_08U[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_ABSDIFF_08U[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_ABSDIFF_08U[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_ABSDIFF_08U[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_ABSDIFF_08U[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_ABSDIFF_08U[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_ABSDIFF_08U[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_ABSDIFF_08U[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_ABSDIFF_08U[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_ABSDIFF_08U[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_ABSDIFF_08U[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_ABSDIFF_08U[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_ABSDIFF_08U[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_ABSDIFF_08U[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_ABSDIFF_08U[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_ABSDIFF_08U[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_ABSDIFF_08U[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_ABSDIFF_08U[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_ABSDIFF_08U[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_ABSDIFF_08U[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_ABSDIFF_08U[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_ABSDIFF_08U[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_ABSDIFF_08U[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_ABSDIFF_08U[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_ABSDIFF_08U[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_ABSDIFF_08U[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_ABSDIFF_08U[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_ABSDIFF_08U[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_ABSDIFF_08U[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_ABSDIFF_08U[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_ABSDIFF_08U[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_ABSDIFF_08U[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_ABSDIFF_08U[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_ABSDIFF_08U[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_ABSDIFF_08U[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_ABSDIFF_08U[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_ABSDIFF_08U[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_ABSDIFF_08U[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_ABSDIFF_08U[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_ABSDIFF_08U[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_ABSDIFF_08U[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_ABSDIFF_08U[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_ABSDIFF_08U[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_ABSDIFF_08U[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_ABSDIFF_08U[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_ABSDIFF_08U[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_ABSDIFF_08U[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_ABSDIFF_08U[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_ABSDIFF_08U[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_ABSDIFF_08U[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_ABSDIFF_08U[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_ABSDIFF_08U[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_ABSDIFF_08U[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_ABSDIFF_08U[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_ABSDIFF_08U[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_ABSDIFF_08U[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_ABSDIFF_08U[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_ABSDIFF_08U[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_ABSDIFF_08U[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_ABSDIFF_08U[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_ABSDIFF_08U[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_ABSDIFF_08U[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_ABSDIFF_08U[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_ABSDIFF_08U[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_ABSDIFF_08U[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_ABSDIFF_08U[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_ABSDIFF_08U[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_ABSDIFF_08U[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_ABSDIFF_08U[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_ABSDIFF_08U[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_ABSDIFF_08U[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_ABSDIFF_08U[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_ABSDIFF_08U[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_ABSDIFF_08U[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_ABSDIFF_08U[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_ABSDIFF_08U[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_ABSDIFF_08U[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_ABSDIFF_08U[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_ABSDIFF_08U[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_ABSDIFF_08U[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_ABSDIFF_08U[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_ABSDIFF_08U[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_ABSDIFF_08U[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_ABSDIFF_08U[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_ABSDIFF_08U[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_ABSDIFF_08U[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_ABSDIFF_08U[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_ABSDIFF_08U[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_ABSDIFF_08U[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_ABSDIFF_08U[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_ABSDIFF_08U[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_ABSDIFF_08U[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_ABSDIFF_08U[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_ABSDIFF_08U[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_ABSDIFF_08U[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_ABSDIFF_08U[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_ABSDIFF_08U[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_ABSDIFF_08U[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_ABSDIFF_08U[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_ABSDIFF_08U[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_ABSDIFF_08U[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_ABSDIFF_08U[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_ABSDIFF_08U[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_ABSDIFF_08U[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_ABSDIFF_08U[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_ABSDIFF_08U[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_ABSDIFF_08U[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_ABSDIFF_08U[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_ABSDIFF_08U[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_ABSDIFF_08U[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_ABSDIFF_08U[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_ABSDIFF_08U[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_ABSDIFF_08U[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_ABSDIFF_08U[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_ABSDIFF_08U[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_ABSDIFF_08U[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_ABSDIFF_08U[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_ABSDIFF_08U[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_ABSDIFF_08U[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_ABSDIFF_08U[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_ABSDIFF_08U[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_ABSDIFF_08U[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_ABSDIFF_08U[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_ABSDIFF_08U[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_ABSDIFF_08U[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_ABSDIFF_08U[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_ABSDIFF_08U[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_ABSDIFF_08U[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_ABSDIFF_08U[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_ABSDIFF_08U[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_ABSDIFF_08U[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_ABSDIFF_08U[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_ABSDIFF_08U[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_ABSDIFF_08U[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_ABSDIFF_08U[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_ABSDIFF_08U[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_ABSDIFF_08U[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_ABSDIFF_08U[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_ABSDIFF_08U[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_ABSDIFF_08U[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_ABSDIFF_08U[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_ABSDIFF_08U[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_ABSDIFF_08U[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_ABSDIFF_08U[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_ABSDIFF_08U[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_ABSDIFF_08U[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_ABSDIFF_08U[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_ABSDIFF_08U[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_ABSDIFF_08U[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_ABSDIFF_08U[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_ABSDIFF_08U[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_ABSDIFF_08U[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_ABSDIFF_08U[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_ABSDIFF_08U[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_ABSDIFF_08U[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_ABSDIFF_08U[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_ABSDIFF_08U[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_ABSDIFF_08U[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_ABSDIFF_08U[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_ABSDIFF_08U[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_ABSDIFF_08U[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_ABSDIFF_08U[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_ABSDIFF_08U[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_ABSDIFF_08U[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_ABSDIFF_08U[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_ABSDIFF_08U[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_ABSDIFF_08U[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_ABSDIFF_08U[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_ABSDIFF_08U[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_ABSDIFF_08U[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_ABSDIFF_08U[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_ABSDIFF_08U[] = {{0, 0}};

static acf_scenario gScenarioArray_ABSDIFF_08U[] = {
{2, 1, 12, 0, 0, gScenarioBufferData0_ABSDIFF_08U, 48, gScenarioKernelData0_ABSDIFF_08U, 4},
{4, 1, 24, 0, 0, gScenarioBufferData1_ABSDIFF_08U, 48, gScenarioKernelData1_ABSDIFF_08U, 4},
{6, 1, 36, 0, 0, gScenarioBufferData2_ABSDIFF_08U, 48, gScenarioKernelData2_ABSDIFF_08U, 4},
{8, 1, 48, 0, 0, gScenarioBufferData3_ABSDIFF_08U, 48, gScenarioKernelData3_ABSDIFF_08U, 4},
{10, 1, 60, 0, 0, gScenarioBufferData4_ABSDIFF_08U, 48, gScenarioKernelData4_ABSDIFF_08U, 4},
{12, 1, 72, 0, 0, gScenarioBufferData5_ABSDIFF_08U, 48, gScenarioKernelData5_ABSDIFF_08U, 4},
{14, 1, 84, 0, 0, gScenarioBufferData6_ABSDIFF_08U, 48, gScenarioKernelData6_ABSDIFF_08U, 4},
{16, 1, 96, 0, 0, gScenarioBufferData7_ABSDIFF_08U, 48, gScenarioKernelData7_ABSDIFF_08U, 4},
{18, 1, 108, 0, 0, gScenarioBufferData8_ABSDIFF_08U, 48, gScenarioKernelData8_ABSDIFF_08U, 4},
{20, 1, 120, 0, 0, gScenarioBufferData9_ABSDIFF_08U, 48, gScenarioKernelData9_ABSDIFF_08U, 4},
{22, 1, 132, 0, 0, gScenarioBufferData10_ABSDIFF_08U, 48, gScenarioKernelData10_ABSDIFF_08U, 4},
{24, 1, 144, 0, 0, gScenarioBufferData11_ABSDIFF_08U, 48, gScenarioKernelData11_ABSDIFF_08U, 4},
{26, 1, 156, 0, 0, gScenarioBufferData12_ABSDIFF_08U, 48, gScenarioKernelData12_ABSDIFF_08U, 4},
{28, 1, 168, 0, 0, gScenarioBufferData13_ABSDIFF_08U, 48, gScenarioKernelData13_ABSDIFF_08U, 4},
{30, 1, 180, 0, 0, gScenarioBufferData14_ABSDIFF_08U, 48, gScenarioKernelData14_ABSDIFF_08U, 4},
{32, 1, 192, 0, 0, gScenarioBufferData15_ABSDIFF_08U, 48, gScenarioKernelData15_ABSDIFF_08U, 4},
{36, 1, 216, 0, 0, gScenarioBufferData16_ABSDIFF_08U, 48, gScenarioKernelData16_ABSDIFF_08U, 4},
{40, 1, 240, 0, 0, gScenarioBufferData17_ABSDIFF_08U, 48, gScenarioKernelData17_ABSDIFF_08U, 4},
{44, 1, 264, 0, 0, gScenarioBufferData18_ABSDIFF_08U, 48, gScenarioKernelData18_ABSDIFF_08U, 4},
{48, 1, 288, 0, 0, gScenarioBufferData19_ABSDIFF_08U, 48, gScenarioKernelData19_ABSDIFF_08U, 4},
{52, 1, 312, 0, 0, gScenarioBufferData20_ABSDIFF_08U, 48, gScenarioKernelData20_ABSDIFF_08U, 4},
{56, 1, 336, 0, 0, gScenarioBufferData21_ABSDIFF_08U, 48, gScenarioKernelData21_ABSDIFF_08U, 4},
{60, 1, 360, 0, 0, gScenarioBufferData22_ABSDIFF_08U, 48, gScenarioKernelData22_ABSDIFF_08U, 4},
{64, 1, 384, 0, 0, gScenarioBufferData23_ABSDIFF_08U, 48, gScenarioKernelData23_ABSDIFF_08U, 4},
{72, 1, 432, 0, 0, gScenarioBufferData24_ABSDIFF_08U, 48, gScenarioKernelData24_ABSDIFF_08U, 4},
{80, 1, 480, 0, 0, gScenarioBufferData25_ABSDIFF_08U, 48, gScenarioKernelData25_ABSDIFF_08U, 4},
{88, 1, 528, 0, 0, gScenarioBufferData26_ABSDIFF_08U, 48, gScenarioKernelData26_ABSDIFF_08U, 4},
{96, 1, 576, 0, 0, gScenarioBufferData27_ABSDIFF_08U, 48, gScenarioKernelData27_ABSDIFF_08U, 4},
{104, 1, 624, 0, 0, gScenarioBufferData28_ABSDIFF_08U, 48, gScenarioKernelData28_ABSDIFF_08U, 4},
{112, 1, 672, 0, 0, gScenarioBufferData29_ABSDIFF_08U, 48, gScenarioKernelData29_ABSDIFF_08U, 4},
{120, 1, 720, 0, 0, gScenarioBufferData30_ABSDIFF_08U, 48, gScenarioKernelData30_ABSDIFF_08U, 4},
{128, 1, 768, 0, 0, gScenarioBufferData31_ABSDIFF_08U, 48, gScenarioKernelData31_ABSDIFF_08U, 4},
{2, 2, 24, 0, 0, gScenarioBufferData32_ABSDIFF_08U, 48, gScenarioKernelData32_ABSDIFF_08U, 4},
{4, 2, 48, 0, 0, gScenarioBufferData33_ABSDIFF_08U, 48, gScenarioKernelData33_ABSDIFF_08U, 4},
{6, 2, 72, 0, 0, gScenarioBufferData34_ABSDIFF_08U, 48, gScenarioKernelData34_ABSDIFF_08U, 4},
{8, 2, 96, 0, 0, gScenarioBufferData35_ABSDIFF_08U, 48, gScenarioKernelData35_ABSDIFF_08U, 4},
{10, 2, 120, 0, 0, gScenarioBufferData36_ABSDIFF_08U, 48, gScenarioKernelData36_ABSDIFF_08U, 4},
{12, 2, 144, 0, 0, gScenarioBufferData37_ABSDIFF_08U, 48, gScenarioKernelData37_ABSDIFF_08U, 4},
{14, 2, 168, 0, 0, gScenarioBufferData38_ABSDIFF_08U, 48, gScenarioKernelData38_ABSDIFF_08U, 4},
{16, 2, 192, 0, 0, gScenarioBufferData39_ABSDIFF_08U, 48, gScenarioKernelData39_ABSDIFF_08U, 4},
{18, 2, 216, 0, 0, gScenarioBufferData40_ABSDIFF_08U, 48, gScenarioKernelData40_ABSDIFF_08U, 4},
{20, 2, 240, 0, 0, gScenarioBufferData41_ABSDIFF_08U, 48, gScenarioKernelData41_ABSDIFF_08U, 4},
{22, 2, 264, 0, 0, gScenarioBufferData42_ABSDIFF_08U, 48, gScenarioKernelData42_ABSDIFF_08U, 4},
{24, 2, 288, 0, 0, gScenarioBufferData43_ABSDIFF_08U, 48, gScenarioKernelData43_ABSDIFF_08U, 4},
{26, 2, 312, 0, 0, gScenarioBufferData44_ABSDIFF_08U, 48, gScenarioKernelData44_ABSDIFF_08U, 4},
{28, 2, 336, 0, 0, gScenarioBufferData45_ABSDIFF_08U, 48, gScenarioKernelData45_ABSDIFF_08U, 4},
{30, 2, 360, 0, 0, gScenarioBufferData46_ABSDIFF_08U, 48, gScenarioKernelData46_ABSDIFF_08U, 4},
{32, 2, 384, 0, 0, gScenarioBufferData47_ABSDIFF_08U, 48, gScenarioKernelData47_ABSDIFF_08U, 4},
{36, 2, 432, 0, 0, gScenarioBufferData48_ABSDIFF_08U, 48, gScenarioKernelData48_ABSDIFF_08U, 4},
{40, 2, 480, 0, 0, gScenarioBufferData49_ABSDIFF_08U, 48, gScenarioKernelData49_ABSDIFF_08U, 4},
{44, 2, 528, 0, 0, gScenarioBufferData50_ABSDIFF_08U, 48, gScenarioKernelData50_ABSDIFF_08U, 4},
{48, 2, 576, 0, 0, gScenarioBufferData51_ABSDIFF_08U, 48, gScenarioKernelData51_ABSDIFF_08U, 4},
{52, 2, 624, 0, 0, gScenarioBufferData52_ABSDIFF_08U, 48, gScenarioKernelData52_ABSDIFF_08U, 4},
{56, 2, 672, 0, 0, gScenarioBufferData53_ABSDIFF_08U, 48, gScenarioKernelData53_ABSDIFF_08U, 4},
{60, 2, 720, 0, 0, gScenarioBufferData54_ABSDIFF_08U, 48, gScenarioKernelData54_ABSDIFF_08U, 4},
{64, 2, 768, 0, 0, gScenarioBufferData55_ABSDIFF_08U, 48, gScenarioKernelData55_ABSDIFF_08U, 4},
{72, 2, 864, 0, 0, gScenarioBufferData56_ABSDIFF_08U, 48, gScenarioKernelData56_ABSDIFF_08U, 4},
{80, 2, 960, 0, 0, gScenarioBufferData57_ABSDIFF_08U, 48, gScenarioKernelData57_ABSDIFF_08U, 4},
{88, 2, 1056, 0, 0, gScenarioBufferData58_ABSDIFF_08U, 48, gScenarioKernelData58_ABSDIFF_08U, 4},
{96, 2, 1152, 0, 0, gScenarioBufferData59_ABSDIFF_08U, 48, gScenarioKernelData59_ABSDIFF_08U, 4},
{104, 2, 1248, 0, 0, gScenarioBufferData60_ABSDIFF_08U, 48, gScenarioKernelData60_ABSDIFF_08U, 4},
{112, 2, 1344, 0, 0, gScenarioBufferData61_ABSDIFF_08U, 48, gScenarioKernelData61_ABSDIFF_08U, 4},
{120, 2, 1440, 0, 0, gScenarioBufferData62_ABSDIFF_08U, 48, gScenarioKernelData62_ABSDIFF_08U, 4},
{128, 2, 1536, 0, 0, gScenarioBufferData63_ABSDIFF_08U, 48, gScenarioKernelData63_ABSDIFF_08U, 4},
{2, 3, 36, 0, 0, gScenarioBufferData64_ABSDIFF_08U, 48, gScenarioKernelData64_ABSDIFF_08U, 4},
{4, 3, 72, 0, 0, gScenarioBufferData65_ABSDIFF_08U, 48, gScenarioKernelData65_ABSDIFF_08U, 4},
{6, 3, 108, 0, 0, gScenarioBufferData66_ABSDIFF_08U, 48, gScenarioKernelData66_ABSDIFF_08U, 4},
{8, 3, 144, 0, 0, gScenarioBufferData67_ABSDIFF_08U, 48, gScenarioKernelData67_ABSDIFF_08U, 4},
{10, 3, 180, 0, 0, gScenarioBufferData68_ABSDIFF_08U, 48, gScenarioKernelData68_ABSDIFF_08U, 4},
{12, 3, 216, 0, 0, gScenarioBufferData69_ABSDIFF_08U, 48, gScenarioKernelData69_ABSDIFF_08U, 4},
{14, 3, 252, 0, 0, gScenarioBufferData70_ABSDIFF_08U, 48, gScenarioKernelData70_ABSDIFF_08U, 4},
{16, 3, 288, 0, 0, gScenarioBufferData71_ABSDIFF_08U, 48, gScenarioKernelData71_ABSDIFF_08U, 4},
{18, 3, 324, 0, 0, gScenarioBufferData72_ABSDIFF_08U, 48, gScenarioKernelData72_ABSDIFF_08U, 4},
{20, 3, 360, 0, 0, gScenarioBufferData73_ABSDIFF_08U, 48, gScenarioKernelData73_ABSDIFF_08U, 4},
{22, 3, 396, 0, 0, gScenarioBufferData74_ABSDIFF_08U, 48, gScenarioKernelData74_ABSDIFF_08U, 4},
{24, 3, 432, 0, 0, gScenarioBufferData75_ABSDIFF_08U, 48, gScenarioKernelData75_ABSDIFF_08U, 4},
{26, 3, 468, 0, 0, gScenarioBufferData76_ABSDIFF_08U, 48, gScenarioKernelData76_ABSDIFF_08U, 4},
{28, 3, 504, 0, 0, gScenarioBufferData77_ABSDIFF_08U, 48, gScenarioKernelData77_ABSDIFF_08U, 4},
{30, 3, 540, 0, 0, gScenarioBufferData78_ABSDIFF_08U, 48, gScenarioKernelData78_ABSDIFF_08U, 4},
{32, 3, 576, 0, 0, gScenarioBufferData79_ABSDIFF_08U, 48, gScenarioKernelData79_ABSDIFF_08U, 4},
{36, 3, 648, 0, 0, gScenarioBufferData80_ABSDIFF_08U, 48, gScenarioKernelData80_ABSDIFF_08U, 4},
{40, 3, 720, 0, 0, gScenarioBufferData81_ABSDIFF_08U, 48, gScenarioKernelData81_ABSDIFF_08U, 4},
{44, 3, 792, 0, 0, gScenarioBufferData82_ABSDIFF_08U, 48, gScenarioKernelData82_ABSDIFF_08U, 4},
{48, 3, 864, 0, 0, gScenarioBufferData83_ABSDIFF_08U, 48, gScenarioKernelData83_ABSDIFF_08U, 4},
{52, 3, 936, 0, 0, gScenarioBufferData84_ABSDIFF_08U, 48, gScenarioKernelData84_ABSDIFF_08U, 4},
{56, 3, 1008, 0, 0, gScenarioBufferData85_ABSDIFF_08U, 48, gScenarioKernelData85_ABSDIFF_08U, 4},
{60, 3, 1080, 0, 0, gScenarioBufferData86_ABSDIFF_08U, 48, gScenarioKernelData86_ABSDIFF_08U, 4},
{64, 3, 1152, 0, 0, gScenarioBufferData87_ABSDIFF_08U, 48, gScenarioKernelData87_ABSDIFF_08U, 4},
{72, 3, 1296, 0, 0, gScenarioBufferData88_ABSDIFF_08U, 48, gScenarioKernelData88_ABSDIFF_08U, 4},
{80, 3, 1440, 0, 0, gScenarioBufferData89_ABSDIFF_08U, 48, gScenarioKernelData89_ABSDIFF_08U, 4},
{88, 3, 1584, 0, 0, gScenarioBufferData90_ABSDIFF_08U, 48, gScenarioKernelData90_ABSDIFF_08U, 4},
{96, 3, 1728, 0, 0, gScenarioBufferData91_ABSDIFF_08U, 48, gScenarioKernelData91_ABSDIFF_08U, 4},
{104, 3, 1872, 0, 0, gScenarioBufferData92_ABSDIFF_08U, 48, gScenarioKernelData92_ABSDIFF_08U, 4},
{112, 3, 2016, 0, 0, gScenarioBufferData93_ABSDIFF_08U, 48, gScenarioKernelData93_ABSDIFF_08U, 4},
{120, 3, 2160, 0, 0, gScenarioBufferData94_ABSDIFF_08U, 48, gScenarioKernelData94_ABSDIFF_08U, 4},
{128, 3, 2304, 0, 0, gScenarioBufferData95_ABSDIFF_08U, 48, gScenarioKernelData95_ABSDIFF_08U, 4},
{2, 4, 48, 0, 0, gScenarioBufferData96_ABSDIFF_08U, 48, gScenarioKernelData96_ABSDIFF_08U, 4},
{4, 4, 96, 0, 0, gScenarioBufferData97_ABSDIFF_08U, 48, gScenarioKernelData97_ABSDIFF_08U, 4},
{6, 4, 144, 0, 0, gScenarioBufferData98_ABSDIFF_08U, 48, gScenarioKernelData98_ABSDIFF_08U, 4},
{8, 4, 192, 0, 0, gScenarioBufferData99_ABSDIFF_08U, 48, gScenarioKernelData99_ABSDIFF_08U, 4},
{10, 4, 240, 0, 0, gScenarioBufferData100_ABSDIFF_08U, 48, gScenarioKernelData100_ABSDIFF_08U, 4},
{12, 4, 288, 0, 0, gScenarioBufferData101_ABSDIFF_08U, 48, gScenarioKernelData101_ABSDIFF_08U, 4},
{14, 4, 336, 0, 0, gScenarioBufferData102_ABSDIFF_08U, 48, gScenarioKernelData102_ABSDIFF_08U, 4},
{16, 4, 384, 0, 0, gScenarioBufferData103_ABSDIFF_08U, 48, gScenarioKernelData103_ABSDIFF_08U, 4},
{18, 4, 432, 0, 0, gScenarioBufferData104_ABSDIFF_08U, 48, gScenarioKernelData104_ABSDIFF_08U, 4},
{20, 4, 480, 0, 0, gScenarioBufferData105_ABSDIFF_08U, 48, gScenarioKernelData105_ABSDIFF_08U, 4},
{22, 4, 528, 0, 0, gScenarioBufferData106_ABSDIFF_08U, 48, gScenarioKernelData106_ABSDIFF_08U, 4},
{24, 4, 576, 0, 0, gScenarioBufferData107_ABSDIFF_08U, 48, gScenarioKernelData107_ABSDIFF_08U, 4},
{26, 4, 624, 0, 0, gScenarioBufferData108_ABSDIFF_08U, 48, gScenarioKernelData108_ABSDIFF_08U, 4},
{28, 4, 672, 0, 0, gScenarioBufferData109_ABSDIFF_08U, 48, gScenarioKernelData109_ABSDIFF_08U, 4},
{30, 4, 720, 0, 0, gScenarioBufferData110_ABSDIFF_08U, 48, gScenarioKernelData110_ABSDIFF_08U, 4},
{32, 4, 768, 0, 0, gScenarioBufferData111_ABSDIFF_08U, 48, gScenarioKernelData111_ABSDIFF_08U, 4},
{36, 4, 864, 0, 0, gScenarioBufferData112_ABSDIFF_08U, 48, gScenarioKernelData112_ABSDIFF_08U, 4},
{40, 4, 960, 0, 0, gScenarioBufferData113_ABSDIFF_08U, 48, gScenarioKernelData113_ABSDIFF_08U, 4},
{44, 4, 1056, 0, 0, gScenarioBufferData114_ABSDIFF_08U, 48, gScenarioKernelData114_ABSDIFF_08U, 4},
{48, 4, 1152, 0, 0, gScenarioBufferData115_ABSDIFF_08U, 48, gScenarioKernelData115_ABSDIFF_08U, 4},
{52, 4, 1248, 0, 0, gScenarioBufferData116_ABSDIFF_08U, 48, gScenarioKernelData116_ABSDIFF_08U, 4},
{56, 4, 1344, 0, 0, gScenarioBufferData117_ABSDIFF_08U, 48, gScenarioKernelData117_ABSDIFF_08U, 4},
{60, 4, 1440, 0, 0, gScenarioBufferData118_ABSDIFF_08U, 48, gScenarioKernelData118_ABSDIFF_08U, 4},
{64, 4, 1536, 0, 0, gScenarioBufferData119_ABSDIFF_08U, 48, gScenarioKernelData119_ABSDIFF_08U, 4},
{72, 4, 1728, 0, 0, gScenarioBufferData120_ABSDIFF_08U, 48, gScenarioKernelData120_ABSDIFF_08U, 4},
{80, 4, 1920, 0, 0, gScenarioBufferData121_ABSDIFF_08U, 48, gScenarioKernelData121_ABSDIFF_08U, 4},
{88, 4, 2112, 0, 0, gScenarioBufferData122_ABSDIFF_08U, 48, gScenarioKernelData122_ABSDIFF_08U, 4},
{96, 4, 2304, 0, 0, gScenarioBufferData123_ABSDIFF_08U, 48, gScenarioKernelData123_ABSDIFF_08U, 4},
{104, 4, 2496, 0, 0, gScenarioBufferData124_ABSDIFF_08U, 48, gScenarioKernelData124_ABSDIFF_08U, 4},
{112, 4, 2688, 0, 0, gScenarioBufferData125_ABSDIFF_08U, 48, gScenarioKernelData125_ABSDIFF_08U, 4},
{120, 4, 2880, 0, 0, gScenarioBufferData126_ABSDIFF_08U, 48, gScenarioKernelData126_ABSDIFF_08U, 4},
{128, 4, 3072, 0, 0, gScenarioBufferData127_ABSDIFF_08U, 48, gScenarioKernelData127_ABSDIFF_08U, 4},
{2, 5, 60, 0, 0, gScenarioBufferData128_ABSDIFF_08U, 48, gScenarioKernelData128_ABSDIFF_08U, 4},
{4, 5, 120, 0, 0, gScenarioBufferData129_ABSDIFF_08U, 48, gScenarioKernelData129_ABSDIFF_08U, 4},
{6, 5, 180, 0, 0, gScenarioBufferData130_ABSDIFF_08U, 48, gScenarioKernelData130_ABSDIFF_08U, 4},
{8, 5, 240, 0, 0, gScenarioBufferData131_ABSDIFF_08U, 48, gScenarioKernelData131_ABSDIFF_08U, 4},
{10, 5, 300, 0, 0, gScenarioBufferData132_ABSDIFF_08U, 48, gScenarioKernelData132_ABSDIFF_08U, 4},
{12, 5, 360, 0, 0, gScenarioBufferData133_ABSDIFF_08U, 48, gScenarioKernelData133_ABSDIFF_08U, 4},
{14, 5, 420, 0, 0, gScenarioBufferData134_ABSDIFF_08U, 48, gScenarioKernelData134_ABSDIFF_08U, 4},
{16, 5, 480, 0, 0, gScenarioBufferData135_ABSDIFF_08U, 48, gScenarioKernelData135_ABSDIFF_08U, 4},
{18, 5, 540, 0, 0, gScenarioBufferData136_ABSDIFF_08U, 48, gScenarioKernelData136_ABSDIFF_08U, 4},
{20, 5, 600, 0, 0, gScenarioBufferData137_ABSDIFF_08U, 48, gScenarioKernelData137_ABSDIFF_08U, 4},
{22, 5, 660, 0, 0, gScenarioBufferData138_ABSDIFF_08U, 48, gScenarioKernelData138_ABSDIFF_08U, 4},
{24, 5, 720, 0, 0, gScenarioBufferData139_ABSDIFF_08U, 48, gScenarioKernelData139_ABSDIFF_08U, 4},
{26, 5, 780, 0, 0, gScenarioBufferData140_ABSDIFF_08U, 48, gScenarioKernelData140_ABSDIFF_08U, 4},
{28, 5, 840, 0, 0, gScenarioBufferData141_ABSDIFF_08U, 48, gScenarioKernelData141_ABSDIFF_08U, 4},
{30, 5, 900, 0, 0, gScenarioBufferData142_ABSDIFF_08U, 48, gScenarioKernelData142_ABSDIFF_08U, 4},
{32, 5, 960, 0, 0, gScenarioBufferData143_ABSDIFF_08U, 48, gScenarioKernelData143_ABSDIFF_08U, 4},
{36, 5, 1080, 0, 0, gScenarioBufferData144_ABSDIFF_08U, 48, gScenarioKernelData144_ABSDIFF_08U, 4},
{40, 5, 1200, 0, 0, gScenarioBufferData145_ABSDIFF_08U, 48, gScenarioKernelData145_ABSDIFF_08U, 4},
{44, 5, 1320, 0, 0, gScenarioBufferData146_ABSDIFF_08U, 48, gScenarioKernelData146_ABSDIFF_08U, 4},
{48, 5, 1440, 0, 0, gScenarioBufferData147_ABSDIFF_08U, 48, gScenarioKernelData147_ABSDIFF_08U, 4},
{52, 5, 1560, 0, 0, gScenarioBufferData148_ABSDIFF_08U, 48, gScenarioKernelData148_ABSDIFF_08U, 4},
{56, 5, 1680, 0, 0, gScenarioBufferData149_ABSDIFF_08U, 48, gScenarioKernelData149_ABSDIFF_08U, 4},
{60, 5, 1800, 0, 0, gScenarioBufferData150_ABSDIFF_08U, 48, gScenarioKernelData150_ABSDIFF_08U, 4},
{64, 5, 1920, 0, 0, gScenarioBufferData151_ABSDIFF_08U, 48, gScenarioKernelData151_ABSDIFF_08U, 4},
{72, 5, 2160, 0, 0, gScenarioBufferData152_ABSDIFF_08U, 48, gScenarioKernelData152_ABSDIFF_08U, 4},
{80, 5, 2400, 0, 0, gScenarioBufferData153_ABSDIFF_08U, 48, gScenarioKernelData153_ABSDIFF_08U, 4},
{88, 5, 2640, 0, 0, gScenarioBufferData154_ABSDIFF_08U, 48, gScenarioKernelData154_ABSDIFF_08U, 4},
{96, 5, 2880, 0, 0, gScenarioBufferData155_ABSDIFF_08U, 48, gScenarioKernelData155_ABSDIFF_08U, 4},
{104, 5, 3120, 0, 0, gScenarioBufferData156_ABSDIFF_08U, 48, gScenarioKernelData156_ABSDIFF_08U, 4},
{112, 5, 3360, 0, 0, gScenarioBufferData157_ABSDIFF_08U, 48, gScenarioKernelData157_ABSDIFF_08U, 4},
{120, 5, 3600, 0, 0, gScenarioBufferData158_ABSDIFF_08U, 48, gScenarioKernelData158_ABSDIFF_08U, 4},
{128, 5, 3840, 0, 0, gScenarioBufferData159_ABSDIFF_08U, 48, gScenarioKernelData159_ABSDIFF_08U, 4},
{2, 6, 72, 0, 0, gScenarioBufferData160_ABSDIFF_08U, 48, gScenarioKernelData160_ABSDIFF_08U, 4},
{4, 6, 144, 0, 0, gScenarioBufferData161_ABSDIFF_08U, 48, gScenarioKernelData161_ABSDIFF_08U, 4},
{6, 6, 216, 0, 0, gScenarioBufferData162_ABSDIFF_08U, 48, gScenarioKernelData162_ABSDIFF_08U, 4},
{8, 6, 288, 0, 0, gScenarioBufferData163_ABSDIFF_08U, 48, gScenarioKernelData163_ABSDIFF_08U, 4},
{10, 6, 360, 0, 0, gScenarioBufferData164_ABSDIFF_08U, 48, gScenarioKernelData164_ABSDIFF_08U, 4},
{12, 6, 432, 0, 0, gScenarioBufferData165_ABSDIFF_08U, 48, gScenarioKernelData165_ABSDIFF_08U, 4},
{14, 6, 504, 0, 0, gScenarioBufferData166_ABSDIFF_08U, 48, gScenarioKernelData166_ABSDIFF_08U, 4},
{16, 6, 576, 0, 0, gScenarioBufferData167_ABSDIFF_08U, 48, gScenarioKernelData167_ABSDIFF_08U, 4},
{18, 6, 648, 0, 0, gScenarioBufferData168_ABSDIFF_08U, 48, gScenarioKernelData168_ABSDIFF_08U, 4},
{20, 6, 720, 0, 0, gScenarioBufferData169_ABSDIFF_08U, 48, gScenarioKernelData169_ABSDIFF_08U, 4},
{22, 6, 792, 0, 0, gScenarioBufferData170_ABSDIFF_08U, 48, gScenarioKernelData170_ABSDIFF_08U, 4},
{24, 6, 864, 0, 0, gScenarioBufferData171_ABSDIFF_08U, 48, gScenarioKernelData171_ABSDIFF_08U, 4},
{26, 6, 936, 0, 0, gScenarioBufferData172_ABSDIFF_08U, 48, gScenarioKernelData172_ABSDIFF_08U, 4},
{28, 6, 1008, 0, 0, gScenarioBufferData173_ABSDIFF_08U, 48, gScenarioKernelData173_ABSDIFF_08U, 4},
{30, 6, 1080, 0, 0, gScenarioBufferData174_ABSDIFF_08U, 48, gScenarioKernelData174_ABSDIFF_08U, 4},
{32, 6, 1152, 0, 0, gScenarioBufferData175_ABSDIFF_08U, 48, gScenarioKernelData175_ABSDIFF_08U, 4},
{36, 6, 1296, 0, 0, gScenarioBufferData176_ABSDIFF_08U, 48, gScenarioKernelData176_ABSDIFF_08U, 4},
{40, 6, 1440, 0, 0, gScenarioBufferData177_ABSDIFF_08U, 48, gScenarioKernelData177_ABSDIFF_08U, 4},
{44, 6, 1584, 0, 0, gScenarioBufferData178_ABSDIFF_08U, 48, gScenarioKernelData178_ABSDIFF_08U, 4},
{48, 6, 1728, 0, 0, gScenarioBufferData179_ABSDIFF_08U, 48, gScenarioKernelData179_ABSDIFF_08U, 4},
{52, 6, 1872, 0, 0, gScenarioBufferData180_ABSDIFF_08U, 48, gScenarioKernelData180_ABSDIFF_08U, 4},
{56, 6, 2016, 0, 0, gScenarioBufferData181_ABSDIFF_08U, 48, gScenarioKernelData181_ABSDIFF_08U, 4},
{60, 6, 2160, 0, 0, gScenarioBufferData182_ABSDIFF_08U, 48, gScenarioKernelData182_ABSDIFF_08U, 4},
{64, 6, 2304, 0, 0, gScenarioBufferData183_ABSDIFF_08U, 48, gScenarioKernelData183_ABSDIFF_08U, 4},
{72, 6, 2592, 0, 0, gScenarioBufferData184_ABSDIFF_08U, 48, gScenarioKernelData184_ABSDIFF_08U, 4},
{80, 6, 2880, 0, 0, gScenarioBufferData185_ABSDIFF_08U, 48, gScenarioKernelData185_ABSDIFF_08U, 4},
{88, 6, 3168, 0, 0, gScenarioBufferData186_ABSDIFF_08U, 48, gScenarioKernelData186_ABSDIFF_08U, 4},
{96, 6, 3456, 0, 0, gScenarioBufferData187_ABSDIFF_08U, 48, gScenarioKernelData187_ABSDIFF_08U, 4},
{104, 6, 3744, 0, 0, gScenarioBufferData188_ABSDIFF_08U, 48, gScenarioKernelData188_ABSDIFF_08U, 4},
{2, 8, 96, 0, 0, gScenarioBufferData189_ABSDIFF_08U, 48, gScenarioKernelData189_ABSDIFF_08U, 4},
{4, 8, 192, 0, 0, gScenarioBufferData190_ABSDIFF_08U, 48, gScenarioKernelData190_ABSDIFF_08U, 4},
{6, 8, 288, 0, 0, gScenarioBufferData191_ABSDIFF_08U, 48, gScenarioKernelData191_ABSDIFF_08U, 4},
{8, 8, 384, 0, 0, gScenarioBufferData192_ABSDIFF_08U, 48, gScenarioKernelData192_ABSDIFF_08U, 4},
{10, 8, 480, 0, 0, gScenarioBufferData193_ABSDIFF_08U, 48, gScenarioKernelData193_ABSDIFF_08U, 4},
{12, 8, 576, 0, 0, gScenarioBufferData194_ABSDIFF_08U, 48, gScenarioKernelData194_ABSDIFF_08U, 4},
{14, 8, 672, 0, 0, gScenarioBufferData195_ABSDIFF_08U, 48, gScenarioKernelData195_ABSDIFF_08U, 4},
{16, 8, 768, 0, 0, gScenarioBufferData196_ABSDIFF_08U, 48, gScenarioKernelData196_ABSDIFF_08U, 4},
{18, 8, 864, 0, 0, gScenarioBufferData197_ABSDIFF_08U, 48, gScenarioKernelData197_ABSDIFF_08U, 4},
{20, 8, 960, 0, 0, gScenarioBufferData198_ABSDIFF_08U, 48, gScenarioKernelData198_ABSDIFF_08U, 4},
{22, 8, 1056, 0, 0, gScenarioBufferData199_ABSDIFF_08U, 48, gScenarioKernelData199_ABSDIFF_08U, 4},
{24, 8, 1152, 0, 0, gScenarioBufferData200_ABSDIFF_08U, 48, gScenarioKernelData200_ABSDIFF_08U, 4},
{26, 8, 1248, 0, 0, gScenarioBufferData201_ABSDIFF_08U, 48, gScenarioKernelData201_ABSDIFF_08U, 4},
{28, 8, 1344, 0, 0, gScenarioBufferData202_ABSDIFF_08U, 48, gScenarioKernelData202_ABSDIFF_08U, 4},
{30, 8, 1440, 0, 0, gScenarioBufferData203_ABSDIFF_08U, 48, gScenarioKernelData203_ABSDIFF_08U, 4},
{32, 8, 1536, 0, 0, gScenarioBufferData204_ABSDIFF_08U, 48, gScenarioKernelData204_ABSDIFF_08U, 4},
{36, 8, 1728, 0, 0, gScenarioBufferData205_ABSDIFF_08U, 48, gScenarioKernelData205_ABSDIFF_08U, 4},
{40, 8, 1920, 0, 0, gScenarioBufferData206_ABSDIFF_08U, 48, gScenarioKernelData206_ABSDIFF_08U, 4},
{44, 8, 2112, 0, 0, gScenarioBufferData207_ABSDIFF_08U, 48, gScenarioKernelData207_ABSDIFF_08U, 4},
{48, 8, 2304, 0, 0, gScenarioBufferData208_ABSDIFF_08U, 48, gScenarioKernelData208_ABSDIFF_08U, 4},
{52, 8, 2496, 0, 0, gScenarioBufferData209_ABSDIFF_08U, 48, gScenarioKernelData209_ABSDIFF_08U, 4},
{56, 8, 2688, 0, 0, gScenarioBufferData210_ABSDIFF_08U, 48, gScenarioKernelData210_ABSDIFF_08U, 4},
{60, 8, 2880, 0, 0, gScenarioBufferData211_ABSDIFF_08U, 48, gScenarioKernelData211_ABSDIFF_08U, 4},
{64, 8, 3072, 0, 0, gScenarioBufferData212_ABSDIFF_08U, 48, gScenarioKernelData212_ABSDIFF_08U, 4},
{72, 8, 3456, 0, 0, gScenarioBufferData213_ABSDIFF_08U, 48, gScenarioKernelData213_ABSDIFF_08U, 4},
{80, 8, 3840, 0, 0, gScenarioBufferData214_ABSDIFF_08U, 48, gScenarioKernelData214_ABSDIFF_08U, 4},
{2, 10, 120, 0, 0, gScenarioBufferData215_ABSDIFF_08U, 48, gScenarioKernelData215_ABSDIFF_08U, 4},
{4, 10, 240, 0, 0, gScenarioBufferData216_ABSDIFF_08U, 48, gScenarioKernelData216_ABSDIFF_08U, 4},
{6, 10, 360, 0, 0, gScenarioBufferData217_ABSDIFF_08U, 48, gScenarioKernelData217_ABSDIFF_08U, 4},
{8, 10, 480, 0, 0, gScenarioBufferData218_ABSDIFF_08U, 48, gScenarioKernelData218_ABSDIFF_08U, 4},
{10, 10, 600, 0, 0, gScenarioBufferData219_ABSDIFF_08U, 48, gScenarioKernelData219_ABSDIFF_08U, 4},
{12, 10, 720, 0, 0, gScenarioBufferData220_ABSDIFF_08U, 48, gScenarioKernelData220_ABSDIFF_08U, 4},
{14, 10, 840, 0, 0, gScenarioBufferData221_ABSDIFF_08U, 48, gScenarioKernelData221_ABSDIFF_08U, 4},
{16, 10, 960, 0, 0, gScenarioBufferData222_ABSDIFF_08U, 48, gScenarioKernelData222_ABSDIFF_08U, 4},
{18, 10, 1080, 0, 0, gScenarioBufferData223_ABSDIFF_08U, 48, gScenarioKernelData223_ABSDIFF_08U, 4},
{20, 10, 1200, 0, 0, gScenarioBufferData224_ABSDIFF_08U, 48, gScenarioKernelData224_ABSDIFF_08U, 4},
{22, 10, 1320, 0, 0, gScenarioBufferData225_ABSDIFF_08U, 48, gScenarioKernelData225_ABSDIFF_08U, 4},
{24, 10, 1440, 0, 0, gScenarioBufferData226_ABSDIFF_08U, 48, gScenarioKernelData226_ABSDIFF_08U, 4},
{26, 10, 1560, 0, 0, gScenarioBufferData227_ABSDIFF_08U, 48, gScenarioKernelData227_ABSDIFF_08U, 4},
{28, 10, 1680, 0, 0, gScenarioBufferData228_ABSDIFF_08U, 48, gScenarioKernelData228_ABSDIFF_08U, 4},
{30, 10, 1800, 0, 0, gScenarioBufferData229_ABSDIFF_08U, 48, gScenarioKernelData229_ABSDIFF_08U, 4},
{32, 10, 1920, 0, 0, gScenarioBufferData230_ABSDIFF_08U, 48, gScenarioKernelData230_ABSDIFF_08U, 4},
{36, 10, 2160, 0, 0, gScenarioBufferData231_ABSDIFF_08U, 48, gScenarioKernelData231_ABSDIFF_08U, 4},
{40, 10, 2400, 0, 0, gScenarioBufferData232_ABSDIFF_08U, 48, gScenarioKernelData232_ABSDIFF_08U, 4},
{44, 10, 2640, 0, 0, gScenarioBufferData233_ABSDIFF_08U, 48, gScenarioKernelData233_ABSDIFF_08U, 4},
{48, 10, 2880, 0, 0, gScenarioBufferData234_ABSDIFF_08U, 48, gScenarioKernelData234_ABSDIFF_08U, 4},
{52, 10, 3120, 0, 0, gScenarioBufferData235_ABSDIFF_08U, 48, gScenarioKernelData235_ABSDIFF_08U, 4},
{56, 10, 3360, 0, 0, gScenarioBufferData236_ABSDIFF_08U, 48, gScenarioKernelData236_ABSDIFF_08U, 4},
{60, 10, 3600, 0, 0, gScenarioBufferData237_ABSDIFF_08U, 48, gScenarioKernelData237_ABSDIFF_08U, 4},
{64, 10, 3840, 0, 0, gScenarioBufferData238_ABSDIFF_08U, 48, gScenarioKernelData238_ABSDIFF_08U, 4},
{2, 12, 144, 0, 0, gScenarioBufferData239_ABSDIFF_08U, 48, gScenarioKernelData239_ABSDIFF_08U, 4},
{4, 12, 288, 0, 0, gScenarioBufferData240_ABSDIFF_08U, 48, gScenarioKernelData240_ABSDIFF_08U, 4},
{6, 12, 432, 0, 0, gScenarioBufferData241_ABSDIFF_08U, 48, gScenarioKernelData241_ABSDIFF_08U, 4},
{8, 12, 576, 0, 0, gScenarioBufferData242_ABSDIFF_08U, 48, gScenarioKernelData242_ABSDIFF_08U, 4},
{10, 12, 720, 0, 0, gScenarioBufferData243_ABSDIFF_08U, 48, gScenarioKernelData243_ABSDIFF_08U, 4},
{12, 12, 864, 0, 0, gScenarioBufferData244_ABSDIFF_08U, 48, gScenarioKernelData244_ABSDIFF_08U, 4},
{14, 12, 1008, 0, 0, gScenarioBufferData245_ABSDIFF_08U, 48, gScenarioKernelData245_ABSDIFF_08U, 4},
{16, 12, 1152, 0, 0, gScenarioBufferData246_ABSDIFF_08U, 48, gScenarioKernelData246_ABSDIFF_08U, 4},
{18, 12, 1296, 0, 0, gScenarioBufferData247_ABSDIFF_08U, 48, gScenarioKernelData247_ABSDIFF_08U, 4},
{20, 12, 1440, 0, 0, gScenarioBufferData248_ABSDIFF_08U, 48, gScenarioKernelData248_ABSDIFF_08U, 4},
{22, 12, 1584, 0, 0, gScenarioBufferData249_ABSDIFF_08U, 48, gScenarioKernelData249_ABSDIFF_08U, 4},
{24, 12, 1728, 0, 0, gScenarioBufferData250_ABSDIFF_08U, 48, gScenarioKernelData250_ABSDIFF_08U, 4},
{26, 12, 1872, 0, 0, gScenarioBufferData251_ABSDIFF_08U, 48, gScenarioKernelData251_ABSDIFF_08U, 4},
{28, 12, 2016, 0, 0, gScenarioBufferData252_ABSDIFF_08U, 48, gScenarioKernelData252_ABSDIFF_08U, 4},
{30, 12, 2160, 0, 0, gScenarioBufferData253_ABSDIFF_08U, 48, gScenarioKernelData253_ABSDIFF_08U, 4},
{32, 12, 2304, 0, 0, gScenarioBufferData254_ABSDIFF_08U, 48, gScenarioKernelData254_ABSDIFF_08U, 4},
{36, 12, 2592, 0, 0, gScenarioBufferData255_ABSDIFF_08U, 48, gScenarioKernelData255_ABSDIFF_08U, 4},
{40, 12, 2880, 0, 0, gScenarioBufferData256_ABSDIFF_08U, 48, gScenarioKernelData256_ABSDIFF_08U, 4},
{44, 12, 3168, 0, 0, gScenarioBufferData257_ABSDIFF_08U, 48, gScenarioKernelData257_ABSDIFF_08U, 4},
{48, 12, 3456, 0, 0, gScenarioBufferData258_ABSDIFF_08U, 48, gScenarioKernelData258_ABSDIFF_08U, 4},
{52, 12, 3744, 0, 0, gScenarioBufferData259_ABSDIFF_08U, 48, gScenarioKernelData259_ABSDIFF_08U, 4},
{2, 14, 168, 0, 0, gScenarioBufferData260_ABSDIFF_08U, 48, gScenarioKernelData260_ABSDIFF_08U, 4},
{4, 14, 336, 0, 0, gScenarioBufferData261_ABSDIFF_08U, 48, gScenarioKernelData261_ABSDIFF_08U, 4},
{6, 14, 504, 0, 0, gScenarioBufferData262_ABSDIFF_08U, 48, gScenarioKernelData262_ABSDIFF_08U, 4},
{8, 14, 672, 0, 0, gScenarioBufferData263_ABSDIFF_08U, 48, gScenarioKernelData263_ABSDIFF_08U, 4},
{10, 14, 840, 0, 0, gScenarioBufferData264_ABSDIFF_08U, 48, gScenarioKernelData264_ABSDIFF_08U, 4},
{12, 14, 1008, 0, 0, gScenarioBufferData265_ABSDIFF_08U, 48, gScenarioKernelData265_ABSDIFF_08U, 4},
{14, 14, 1176, 0, 0, gScenarioBufferData266_ABSDIFF_08U, 48, gScenarioKernelData266_ABSDIFF_08U, 4},
{16, 14, 1344, 0, 0, gScenarioBufferData267_ABSDIFF_08U, 48, gScenarioKernelData267_ABSDIFF_08U, 4},
{18, 14, 1512, 0, 0, gScenarioBufferData268_ABSDIFF_08U, 48, gScenarioKernelData268_ABSDIFF_08U, 4},
{20, 14, 1680, 0, 0, gScenarioBufferData269_ABSDIFF_08U, 48, gScenarioKernelData269_ABSDIFF_08U, 4},
{22, 14, 1848, 0, 0, gScenarioBufferData270_ABSDIFF_08U, 48, gScenarioKernelData270_ABSDIFF_08U, 4},
{24, 14, 2016, 0, 0, gScenarioBufferData271_ABSDIFF_08U, 48, gScenarioKernelData271_ABSDIFF_08U, 4},
{26, 14, 2184, 0, 0, gScenarioBufferData272_ABSDIFF_08U, 48, gScenarioKernelData272_ABSDIFF_08U, 4},
{28, 14, 2352, 0, 0, gScenarioBufferData273_ABSDIFF_08U, 48, gScenarioKernelData273_ABSDIFF_08U, 4},
{30, 14, 2520, 0, 0, gScenarioBufferData274_ABSDIFF_08U, 48, gScenarioKernelData274_ABSDIFF_08U, 4},
{32, 14, 2688, 0, 0, gScenarioBufferData275_ABSDIFF_08U, 48, gScenarioKernelData275_ABSDIFF_08U, 4},
{36, 14, 3024, 0, 0, gScenarioBufferData276_ABSDIFF_08U, 48, gScenarioKernelData276_ABSDIFF_08U, 4},
{40, 14, 3360, 0, 0, gScenarioBufferData277_ABSDIFF_08U, 48, gScenarioKernelData277_ABSDIFF_08U, 4},
{44, 14, 3696, 0, 0, gScenarioBufferData278_ABSDIFF_08U, 48, gScenarioKernelData278_ABSDIFF_08U, 4},
{2, 16, 192, 0, 0, gScenarioBufferData279_ABSDIFF_08U, 48, gScenarioKernelData279_ABSDIFF_08U, 4},
{4, 16, 384, 0, 0, gScenarioBufferData280_ABSDIFF_08U, 48, gScenarioKernelData280_ABSDIFF_08U, 4},
{6, 16, 576, 0, 0, gScenarioBufferData281_ABSDIFF_08U, 48, gScenarioKernelData281_ABSDIFF_08U, 4},
{8, 16, 768, 0, 0, gScenarioBufferData282_ABSDIFF_08U, 48, gScenarioKernelData282_ABSDIFF_08U, 4},
{10, 16, 960, 0, 0, gScenarioBufferData283_ABSDIFF_08U, 48, gScenarioKernelData283_ABSDIFF_08U, 4},
{12, 16, 1152, 0, 0, gScenarioBufferData284_ABSDIFF_08U, 48, gScenarioKernelData284_ABSDIFF_08U, 4},
{14, 16, 1344, 0, 0, gScenarioBufferData285_ABSDIFF_08U, 48, gScenarioKernelData285_ABSDIFF_08U, 4},
{16, 16, 1536, 0, 0, gScenarioBufferData286_ABSDIFF_08U, 48, gScenarioKernelData286_ABSDIFF_08U, 4},
{18, 16, 1728, 0, 0, gScenarioBufferData287_ABSDIFF_08U, 48, gScenarioKernelData287_ABSDIFF_08U, 4},
{20, 16, 1920, 0, 0, gScenarioBufferData288_ABSDIFF_08U, 48, gScenarioKernelData288_ABSDIFF_08U, 4},
{22, 16, 2112, 0, 0, gScenarioBufferData289_ABSDIFF_08U, 48, gScenarioKernelData289_ABSDIFF_08U, 4},
{24, 16, 2304, 0, 0, gScenarioBufferData290_ABSDIFF_08U, 48, gScenarioKernelData290_ABSDIFF_08U, 4},
{26, 16, 2496, 0, 0, gScenarioBufferData291_ABSDIFF_08U, 48, gScenarioKernelData291_ABSDIFF_08U, 4},
{28, 16, 2688, 0, 0, gScenarioBufferData292_ABSDIFF_08U, 48, gScenarioKernelData292_ABSDIFF_08U, 4},
{30, 16, 2880, 0, 0, gScenarioBufferData293_ABSDIFF_08U, 48, gScenarioKernelData293_ABSDIFF_08U, 4},
{32, 16, 3072, 0, 0, gScenarioBufferData294_ABSDIFF_08U, 48, gScenarioKernelData294_ABSDIFF_08U, 4},
{36, 16, 3456, 0, 0, gScenarioBufferData295_ABSDIFF_08U, 48, gScenarioKernelData295_ABSDIFF_08U, 4},
{40, 16, 3840, 0, 0, gScenarioBufferData296_ABSDIFF_08U, 48, gScenarioKernelData296_ABSDIFF_08U, 4},
{2, 18, 216, 0, 0, gScenarioBufferData297_ABSDIFF_08U, 48, gScenarioKernelData297_ABSDIFF_08U, 4},
{4, 18, 432, 0, 0, gScenarioBufferData298_ABSDIFF_08U, 48, gScenarioKernelData298_ABSDIFF_08U, 4},
{6, 18, 648, 0, 0, gScenarioBufferData299_ABSDIFF_08U, 48, gScenarioKernelData299_ABSDIFF_08U, 4},
{8, 18, 864, 0, 0, gScenarioBufferData300_ABSDIFF_08U, 48, gScenarioKernelData300_ABSDIFF_08U, 4},
{10, 18, 1080, 0, 0, gScenarioBufferData301_ABSDIFF_08U, 48, gScenarioKernelData301_ABSDIFF_08U, 4},
{12, 18, 1296, 0, 0, gScenarioBufferData302_ABSDIFF_08U, 48, gScenarioKernelData302_ABSDIFF_08U, 4},
{14, 18, 1512, 0, 0, gScenarioBufferData303_ABSDIFF_08U, 48, gScenarioKernelData303_ABSDIFF_08U, 4},
{16, 18, 1728, 0, 0, gScenarioBufferData304_ABSDIFF_08U, 48, gScenarioKernelData304_ABSDIFF_08U, 4},
{18, 18, 1944, 0, 0, gScenarioBufferData305_ABSDIFF_08U, 48, gScenarioKernelData305_ABSDIFF_08U, 4},
{20, 18, 2160, 0, 0, gScenarioBufferData306_ABSDIFF_08U, 48, gScenarioKernelData306_ABSDIFF_08U, 4},
{22, 18, 2376, 0, 0, gScenarioBufferData307_ABSDIFF_08U, 48, gScenarioKernelData307_ABSDIFF_08U, 4},
{24, 18, 2592, 0, 0, gScenarioBufferData308_ABSDIFF_08U, 48, gScenarioKernelData308_ABSDIFF_08U, 4},
{26, 18, 2808, 0, 0, gScenarioBufferData309_ABSDIFF_08U, 48, gScenarioKernelData309_ABSDIFF_08U, 4},
{28, 18, 3024, 0, 0, gScenarioBufferData310_ABSDIFF_08U, 48, gScenarioKernelData310_ABSDIFF_08U, 4},
{30, 18, 3240, 0, 0, gScenarioBufferData311_ABSDIFF_08U, 48, gScenarioKernelData311_ABSDIFF_08U, 4},
{32, 18, 3456, 0, 0, gScenarioBufferData312_ABSDIFF_08U, 48, gScenarioKernelData312_ABSDIFF_08U, 4},
{2, 20, 240, 0, 0, gScenarioBufferData313_ABSDIFF_08U, 48, gScenarioKernelData313_ABSDIFF_08U, 4},
{4, 20, 480, 0, 0, gScenarioBufferData314_ABSDIFF_08U, 48, gScenarioKernelData314_ABSDIFF_08U, 4},
{6, 20, 720, 0, 0, gScenarioBufferData315_ABSDIFF_08U, 48, gScenarioKernelData315_ABSDIFF_08U, 4},
{8, 20, 960, 0, 0, gScenarioBufferData316_ABSDIFF_08U, 48, gScenarioKernelData316_ABSDIFF_08U, 4},
{10, 20, 1200, 0, 0, gScenarioBufferData317_ABSDIFF_08U, 48, gScenarioKernelData317_ABSDIFF_08U, 4},
{12, 20, 1440, 0, 0, gScenarioBufferData318_ABSDIFF_08U, 48, gScenarioKernelData318_ABSDIFF_08U, 4},
{14, 20, 1680, 0, 0, gScenarioBufferData319_ABSDIFF_08U, 48, gScenarioKernelData319_ABSDIFF_08U, 4},
{16, 20, 1920, 0, 0, gScenarioBufferData320_ABSDIFF_08U, 48, gScenarioKernelData320_ABSDIFF_08U, 4},
{18, 20, 2160, 0, 0, gScenarioBufferData321_ABSDIFF_08U, 48, gScenarioKernelData321_ABSDIFF_08U, 4},
{20, 20, 2400, 0, 0, gScenarioBufferData322_ABSDIFF_08U, 48, gScenarioKernelData322_ABSDIFF_08U, 4},
{22, 20, 2640, 0, 0, gScenarioBufferData323_ABSDIFF_08U, 48, gScenarioKernelData323_ABSDIFF_08U, 4},
{24, 20, 2880, 0, 0, gScenarioBufferData324_ABSDIFF_08U, 48, gScenarioKernelData324_ABSDIFF_08U, 4},
{26, 20, 3120, 0, 0, gScenarioBufferData325_ABSDIFF_08U, 48, gScenarioKernelData325_ABSDIFF_08U, 4},
{28, 20, 3360, 0, 0, gScenarioBufferData326_ABSDIFF_08U, 48, gScenarioKernelData326_ABSDIFF_08U, 4},
{30, 20, 3600, 0, 0, gScenarioBufferData327_ABSDIFF_08U, 48, gScenarioKernelData327_ABSDIFF_08U, 4},
{32, 20, 3840, 0, 0, gScenarioBufferData328_ABSDIFF_08U, 48, gScenarioKernelData328_ABSDIFF_08U, 4},
{2, 22, 264, 0, 0, gScenarioBufferData329_ABSDIFF_08U, 48, gScenarioKernelData329_ABSDIFF_08U, 4},
{4, 22, 528, 0, 0, gScenarioBufferData330_ABSDIFF_08U, 48, gScenarioKernelData330_ABSDIFF_08U, 4},
{6, 22, 792, 0, 0, gScenarioBufferData331_ABSDIFF_08U, 48, gScenarioKernelData331_ABSDIFF_08U, 4},
{8, 22, 1056, 0, 0, gScenarioBufferData332_ABSDIFF_08U, 48, gScenarioKernelData332_ABSDIFF_08U, 4},
{10, 22, 1320, 0, 0, gScenarioBufferData333_ABSDIFF_08U, 48, gScenarioKernelData333_ABSDIFF_08U, 4},
{12, 22, 1584, 0, 0, gScenarioBufferData334_ABSDIFF_08U, 48, gScenarioKernelData334_ABSDIFF_08U, 4},
{14, 22, 1848, 0, 0, gScenarioBufferData335_ABSDIFF_08U, 48, gScenarioKernelData335_ABSDIFF_08U, 4},
{16, 22, 2112, 0, 0, gScenarioBufferData336_ABSDIFF_08U, 48, gScenarioKernelData336_ABSDIFF_08U, 4},
{18, 22, 2376, 0, 0, gScenarioBufferData337_ABSDIFF_08U, 48, gScenarioKernelData337_ABSDIFF_08U, 4},
{20, 22, 2640, 0, 0, gScenarioBufferData338_ABSDIFF_08U, 48, gScenarioKernelData338_ABSDIFF_08U, 4},
{22, 22, 2904, 0, 0, gScenarioBufferData339_ABSDIFF_08U, 48, gScenarioKernelData339_ABSDIFF_08U, 4},
{24, 22, 3168, 0, 0, gScenarioBufferData340_ABSDIFF_08U, 48, gScenarioKernelData340_ABSDIFF_08U, 4},
{26, 22, 3432, 0, 0, gScenarioBufferData341_ABSDIFF_08U, 48, gScenarioKernelData341_ABSDIFF_08U, 4},
{28, 22, 3696, 0, 0, gScenarioBufferData342_ABSDIFF_08U, 48, gScenarioKernelData342_ABSDIFF_08U, 4},
{2, 24, 288, 0, 0, gScenarioBufferData343_ABSDIFF_08U, 48, gScenarioKernelData343_ABSDIFF_08U, 4},
{4, 24, 576, 0, 0, gScenarioBufferData344_ABSDIFF_08U, 48, gScenarioKernelData344_ABSDIFF_08U, 4},
{6, 24, 864, 0, 0, gScenarioBufferData345_ABSDIFF_08U, 48, gScenarioKernelData345_ABSDIFF_08U, 4},
{8, 24, 1152, 0, 0, gScenarioBufferData346_ABSDIFF_08U, 48, gScenarioKernelData346_ABSDIFF_08U, 4},
{10, 24, 1440, 0, 0, gScenarioBufferData347_ABSDIFF_08U, 48, gScenarioKernelData347_ABSDIFF_08U, 4},
{12, 24, 1728, 0, 0, gScenarioBufferData348_ABSDIFF_08U, 48, gScenarioKernelData348_ABSDIFF_08U, 4},
{14, 24, 2016, 0, 0, gScenarioBufferData349_ABSDIFF_08U, 48, gScenarioKernelData349_ABSDIFF_08U, 4},
{16, 24, 2304, 0, 0, gScenarioBufferData350_ABSDIFF_08U, 48, gScenarioKernelData350_ABSDIFF_08U, 4},
{18, 24, 2592, 0, 0, gScenarioBufferData351_ABSDIFF_08U, 48, gScenarioKernelData351_ABSDIFF_08U, 4},
{20, 24, 2880, 0, 0, gScenarioBufferData352_ABSDIFF_08U, 48, gScenarioKernelData352_ABSDIFF_08U, 4},
{22, 24, 3168, 0, 0, gScenarioBufferData353_ABSDIFF_08U, 48, gScenarioKernelData353_ABSDIFF_08U, 4},
{24, 24, 3456, 0, 0, gScenarioBufferData354_ABSDIFF_08U, 48, gScenarioKernelData354_ABSDIFF_08U, 4},
{26, 24, 3744, 0, 0, gScenarioBufferData355_ABSDIFF_08U, 48, gScenarioKernelData355_ABSDIFF_08U, 4},
{2, 26, 312, 0, 0, gScenarioBufferData356_ABSDIFF_08U, 48, gScenarioKernelData356_ABSDIFF_08U, 4},
{4, 26, 624, 0, 0, gScenarioBufferData357_ABSDIFF_08U, 48, gScenarioKernelData357_ABSDIFF_08U, 4},
{6, 26, 936, 0, 0, gScenarioBufferData358_ABSDIFF_08U, 48, gScenarioKernelData358_ABSDIFF_08U, 4},
{8, 26, 1248, 0, 0, gScenarioBufferData359_ABSDIFF_08U, 48, gScenarioKernelData359_ABSDIFF_08U, 4},
{10, 26, 1560, 0, 0, gScenarioBufferData360_ABSDIFF_08U, 48, gScenarioKernelData360_ABSDIFF_08U, 4},
{12, 26, 1872, 0, 0, gScenarioBufferData361_ABSDIFF_08U, 48, gScenarioKernelData361_ABSDIFF_08U, 4},
{14, 26, 2184, 0, 0, gScenarioBufferData362_ABSDIFF_08U, 48, gScenarioKernelData362_ABSDIFF_08U, 4},
{16, 26, 2496, 0, 0, gScenarioBufferData363_ABSDIFF_08U, 48, gScenarioKernelData363_ABSDIFF_08U, 4},
{18, 26, 2808, 0, 0, gScenarioBufferData364_ABSDIFF_08U, 48, gScenarioKernelData364_ABSDIFF_08U, 4},
{20, 26, 3120, 0, 0, gScenarioBufferData365_ABSDIFF_08U, 48, gScenarioKernelData365_ABSDIFF_08U, 4},
{22, 26, 3432, 0, 0, gScenarioBufferData366_ABSDIFF_08U, 48, gScenarioKernelData366_ABSDIFF_08U, 4},
{24, 26, 3744, 0, 0, gScenarioBufferData367_ABSDIFF_08U, 48, gScenarioKernelData367_ABSDIFF_08U, 4},
{2, 28, 336, 0, 0, gScenarioBufferData368_ABSDIFF_08U, 48, gScenarioKernelData368_ABSDIFF_08U, 4},
{4, 28, 672, 0, 0, gScenarioBufferData369_ABSDIFF_08U, 48, gScenarioKernelData369_ABSDIFF_08U, 4},
{6, 28, 1008, 0, 0, gScenarioBufferData370_ABSDIFF_08U, 48, gScenarioKernelData370_ABSDIFF_08U, 4},
{8, 28, 1344, 0, 0, gScenarioBufferData371_ABSDIFF_08U, 48, gScenarioKernelData371_ABSDIFF_08U, 4},
{10, 28, 1680, 0, 0, gScenarioBufferData372_ABSDIFF_08U, 48, gScenarioKernelData372_ABSDIFF_08U, 4},
{12, 28, 2016, 0, 0, gScenarioBufferData373_ABSDIFF_08U, 48, gScenarioKernelData373_ABSDIFF_08U, 4},
{14, 28, 2352, 0, 0, gScenarioBufferData374_ABSDIFF_08U, 48, gScenarioKernelData374_ABSDIFF_08U, 4},
{16, 28, 2688, 0, 0, gScenarioBufferData375_ABSDIFF_08U, 48, gScenarioKernelData375_ABSDIFF_08U, 4},
{18, 28, 3024, 0, 0, gScenarioBufferData376_ABSDIFF_08U, 48, gScenarioKernelData376_ABSDIFF_08U, 4},
{20, 28, 3360, 0, 0, gScenarioBufferData377_ABSDIFF_08U, 48, gScenarioKernelData377_ABSDIFF_08U, 4},
{22, 28, 3696, 0, 0, gScenarioBufferData378_ABSDIFF_08U, 48, gScenarioKernelData378_ABSDIFF_08U, 4},
{2, 30, 360, 0, 0, gScenarioBufferData379_ABSDIFF_08U, 48, gScenarioKernelData379_ABSDIFF_08U, 4},
{4, 30, 720, 0, 0, gScenarioBufferData380_ABSDIFF_08U, 48, gScenarioKernelData380_ABSDIFF_08U, 4},
{6, 30, 1080, 0, 0, gScenarioBufferData381_ABSDIFF_08U, 48, gScenarioKernelData381_ABSDIFF_08U, 4},
{8, 30, 1440, 0, 0, gScenarioBufferData382_ABSDIFF_08U, 48, gScenarioKernelData382_ABSDIFF_08U, 4},
{10, 30, 1800, 0, 0, gScenarioBufferData383_ABSDIFF_08U, 48, gScenarioKernelData383_ABSDIFF_08U, 4},
{12, 30, 2160, 0, 0, gScenarioBufferData384_ABSDIFF_08U, 48, gScenarioKernelData384_ABSDIFF_08U, 4},
{14, 30, 2520, 0, 0, gScenarioBufferData385_ABSDIFF_08U, 48, gScenarioKernelData385_ABSDIFF_08U, 4},
{16, 30, 2880, 0, 0, gScenarioBufferData386_ABSDIFF_08U, 48, gScenarioKernelData386_ABSDIFF_08U, 4},
{18, 30, 3240, 0, 0, gScenarioBufferData387_ABSDIFF_08U, 48, gScenarioKernelData387_ABSDIFF_08U, 4},
{20, 30, 3600, 0, 0, gScenarioBufferData388_ABSDIFF_08U, 48, gScenarioKernelData388_ABSDIFF_08U, 4},
{2, 32, 384, 0, 0, gScenarioBufferData389_ABSDIFF_08U, 48, gScenarioKernelData389_ABSDIFF_08U, 4},
{4, 32, 768, 0, 0, gScenarioBufferData390_ABSDIFF_08U, 48, gScenarioKernelData390_ABSDIFF_08U, 4},
{6, 32, 1152, 0, 0, gScenarioBufferData391_ABSDIFF_08U, 48, gScenarioKernelData391_ABSDIFF_08U, 4},
{8, 32, 1536, 0, 0, gScenarioBufferData392_ABSDIFF_08U, 48, gScenarioKernelData392_ABSDIFF_08U, 4},
{10, 32, 1920, 0, 0, gScenarioBufferData393_ABSDIFF_08U, 48, gScenarioKernelData393_ABSDIFF_08U, 4},
{12, 32, 2304, 0, 0, gScenarioBufferData394_ABSDIFF_08U, 48, gScenarioKernelData394_ABSDIFF_08U, 4},
{14, 32, 2688, 0, 0, gScenarioBufferData395_ABSDIFF_08U, 48, gScenarioKernelData395_ABSDIFF_08U, 4},
{16, 32, 3072, 0, 0, gScenarioBufferData396_ABSDIFF_08U, 48, gScenarioKernelData396_ABSDIFF_08U, 4},
{18, 32, 3456, 0, 0, gScenarioBufferData397_ABSDIFF_08U, 48, gScenarioKernelData397_ABSDIFF_08U, 4},
{20, 32, 3840, 0, 0, gScenarioBufferData398_ABSDIFF_08U, 48, gScenarioKernelData398_ABSDIFF_08U, 4}
};

static acf_scenario_list gScenarioList_ABSDIFF_08U = {
399, //number of scenarios
gScenarioArray_ABSDIFF_08U};
//**************************************************************

class ABSDIFF_08U : public ACF_Process_APU
{

public:
   ABSDIFF_08U(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("ABSDIFF_08U");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("ABSDIFF_08U",
                                        ABSDIFF_08U_LOAD_SEGMENTS,
                                        ABSDIFF_08U_LOAD_PMEM, ABSDIFF_08U_LOAD_PMEM_SIZE,
                                        ABSDIFF_08U_LOAD_DMEM, ABSDIFF_08U_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(ABSDIFF_08U_APEX_LOG_BUFFER, ABSDIFF_08U_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_ABSDIFF_08U);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_ABSDIFF_08U

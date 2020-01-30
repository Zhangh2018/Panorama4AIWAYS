#ifndef _ACF_PROCESS_APU_PYRAMID_UP
#define _ACF_PROCESS_APU_PYRAMID_UP

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <PYRAMID_UP_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_PYRAMID_UP[] = {{2, 1, 2, 0, {0, 0, 2, 2}}, {4, 2, 4, 0, {2, 2, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_PYRAMID_UP[] = {{4, 1, 2, 0, {0, 0, 2, 2}}, {8, 2, 4, 0, {2, 2, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_PYRAMID_UP[] = {{6, 1, 2, 0, {0, 0, 2, 2}}, {12, 2, 4, 0, {2, 2, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_PYRAMID_UP[] = {{8, 1, 2, 0, {0, 0, 2, 2}}, {16, 2, 4, 0, {2, 2, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_PYRAMID_UP[] = {{10, 1, 2, 0, {0, 0, 2, 2}}, {20, 2, 4, 0, {2, 2, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_PYRAMID_UP[] = {{12, 1, 2, 0, {0, 0, 2, 2}}, {24, 2, 4, 0, {2, 2, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_PYRAMID_UP[] = {{14, 1, 2, 0, {0, 0, 2, 2}}, {28, 2, 4, 0, {2, 2, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_PYRAMID_UP[] = {{16, 1, 2, 0, {0, 0, 2, 2}}, {32, 2, 4, 0, {2, 2, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_PYRAMID_UP[] = {{18, 1, 2, 0, {0, 0, 2, 2}}, {36, 2, 4, 0, {2, 2, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_PYRAMID_UP[] = {{20, 1, 2, 0, {0, 0, 2, 2}}, {40, 2, 4, 0, {2, 2, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_PYRAMID_UP[] = {{22, 1, 2, 0, {0, 0, 2, 2}}, {44, 2, 4, 0, {2, 2, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_PYRAMID_UP[] = {{24, 1, 2, 0, {0, 0, 2, 2}}, {48, 2, 4, 0, {2, 2, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_PYRAMID_UP[] = {{26, 1, 2, 0, {0, 0, 2, 2}}, {52, 2, 4, 0, {2, 2, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_PYRAMID_UP[] = {{28, 1, 2, 0, {0, 0, 2, 2}}, {56, 2, 4, 0, {2, 2, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_PYRAMID_UP[] = {{30, 1, 2, 0, {0, 0, 2, 2}}, {60, 2, 4, 0, {2, 2, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_PYRAMID_UP[] = {{32, 1, 2, 0, {0, 0, 2, 2}}, {64, 2, 4, 0, {2, 2, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_PYRAMID_UP[] = {{36, 1, 2, 0, {0, 0, 2, 2}}, {72, 2, 4, 0, {2, 2, 0, 0}}, {72, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_PYRAMID_UP[] = {{40, 1, 2, 0, {0, 0, 2, 2}}, {80, 2, 4, 0, {2, 2, 0, 0}}, {80, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_PYRAMID_UP[] = {{44, 1, 2, 0, {0, 0, 2, 2}}, {88, 2, 4, 0, {2, 2, 0, 0}}, {88, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_PYRAMID_UP[] = {{48, 1, 2, 0, {0, 0, 2, 2}}, {96, 2, 4, 0, {2, 2, 0, 0}}, {96, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_PYRAMID_UP[] = {{52, 1, 2, 0, {0, 0, 2, 2}}, {104, 2, 4, 0, {2, 2, 0, 0}}, {104, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_PYRAMID_UP[] = {{56, 1, 2, 0, {0, 0, 2, 2}}, {112, 2, 4, 0, {2, 2, 0, 0}}, {112, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_PYRAMID_UP[] = {{60, 1, 2, 0, {0, 0, 2, 2}}, {120, 2, 4, 0, {2, 2, 0, 0}}, {120, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_PYRAMID_UP[] = {{64, 1, 2, 0, {0, 0, 2, 2}}, {128, 2, 4, 0, {2, 2, 0, 0}}, {128, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_PYRAMID_UP[] = {{2, 2, 2, 0, {0, 0, 2, 2}}, {4, 4, 4, 0, {2, 2, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_PYRAMID_UP[] = {{4, 2, 2, 0, {0, 0, 2, 2}}, {8, 4, 4, 0, {2, 2, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_PYRAMID_UP[] = {{6, 2, 2, 0, {0, 0, 2, 2}}, {12, 4, 4, 0, {2, 2, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_PYRAMID_UP[] = {{8, 2, 2, 0, {0, 0, 2, 2}}, {16, 4, 4, 0, {2, 2, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_PYRAMID_UP[] = {{10, 2, 2, 0, {0, 0, 2, 2}}, {20, 4, 4, 0, {2, 2, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_PYRAMID_UP[] = {{12, 2, 2, 0, {0, 0, 2, 2}}, {24, 4, 4, 0, {2, 2, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_PYRAMID_UP[] = {{14, 2, 2, 0, {0, 0, 2, 2}}, {28, 4, 4, 0, {2, 2, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_PYRAMID_UP[] = {{16, 2, 2, 0, {0, 0, 2, 2}}, {32, 4, 4, 0, {2, 2, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_PYRAMID_UP[] = {{18, 2, 2, 0, {0, 0, 2, 2}}, {36, 4, 4, 0, {2, 2, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_PYRAMID_UP[] = {{20, 2, 2, 0, {0, 0, 2, 2}}, {40, 4, 4, 0, {2, 2, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_PYRAMID_UP[] = {{22, 2, 2, 0, {0, 0, 2, 2}}, {44, 4, 4, 0, {2, 2, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_PYRAMID_UP[] = {{24, 2, 2, 0, {0, 0, 2, 2}}, {48, 4, 4, 0, {2, 2, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_PYRAMID_UP[] = {{26, 2, 2, 0, {0, 0, 2, 2}}, {52, 4, 4, 0, {2, 2, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_PYRAMID_UP[] = {{28, 2, 2, 0, {0, 0, 2, 2}}, {56, 4, 4, 0, {2, 2, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_PYRAMID_UP[] = {{30, 2, 2, 0, {0, 0, 2, 2}}, {60, 4, 4, 0, {2, 2, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_PYRAMID_UP[] = {{32, 2, 2, 0, {0, 0, 2, 2}}, {64, 4, 4, 0, {2, 2, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_PYRAMID_UP[] = {{36, 2, 2, 0, {0, 0, 2, 2}}, {72, 4, 4, 0, {2, 2, 0, 0}}, {72, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_PYRAMID_UP[] = {{2, 3, 2, 0, {0, 0, 2, 2}}, {4, 6, 4, 0, {2, 2, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_PYRAMID_UP[] = {{4, 3, 2, 0, {0, 0, 2, 2}}, {8, 6, 4, 0, {2, 2, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_PYRAMID_UP[] = {{6, 3, 2, 0, {0, 0, 2, 2}}, {12, 6, 4, 0, {2, 2, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_PYRAMID_UP[] = {{8, 3, 2, 0, {0, 0, 2, 2}}, {16, 6, 4, 0, {2, 2, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_PYRAMID_UP[] = {{10, 3, 2, 0, {0, 0, 2, 2}}, {20, 6, 4, 0, {2, 2, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_PYRAMID_UP[] = {{12, 3, 2, 0, {0, 0, 2, 2}}, {24, 6, 4, 0, {2, 2, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_PYRAMID_UP[] = {{14, 3, 2, 0, {0, 0, 2, 2}}, {28, 6, 4, 0, {2, 2, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_PYRAMID_UP[] = {{16, 3, 2, 0, {0, 0, 2, 2}}, {32, 6, 4, 0, {2, 2, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_PYRAMID_UP[] = {{18, 3, 2, 0, {0, 0, 2, 2}}, {36, 6, 4, 0, {2, 2, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_PYRAMID_UP[] = {{20, 3, 2, 0, {0, 0, 2, 2}}, {40, 6, 4, 0, {2, 2, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_PYRAMID_UP[] = {{22, 3, 2, 0, {0, 0, 2, 2}}, {44, 6, 4, 0, {2, 2, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_PYRAMID_UP[] = {{24, 3, 2, 0, {0, 0, 2, 2}}, {48, 6, 4, 0, {2, 2, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_PYRAMID_UP[] = {{26, 3, 2, 0, {0, 0, 2, 2}}, {52, 6, 4, 0, {2, 2, 0, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_PYRAMID_UP[] = {{2, 4, 2, 0, {0, 0, 2, 2}}, {4, 8, 4, 0, {2, 2, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_PYRAMID_UP[] = {{4, 4, 2, 0, {0, 0, 2, 2}}, {8, 8, 4, 0, {2, 2, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_PYRAMID_UP[] = {{6, 4, 2, 0, {0, 0, 2, 2}}, {12, 8, 4, 0, {2, 2, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_PYRAMID_UP[] = {{8, 4, 2, 0, {0, 0, 2, 2}}, {16, 8, 4, 0, {2, 2, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_PYRAMID_UP[] = {{10, 4, 2, 0, {0, 0, 2, 2}}, {20, 8, 4, 0, {2, 2, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_PYRAMID_UP[] = {{12, 4, 2, 0, {0, 0, 2, 2}}, {24, 8, 4, 0, {2, 2, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_PYRAMID_UP[] = {{14, 4, 2, 0, {0, 0, 2, 2}}, {28, 8, 4, 0, {2, 2, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_PYRAMID_UP[] = {{16, 4, 2, 0, {0, 0, 2, 2}}, {32, 8, 4, 0, {2, 2, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_PYRAMID_UP[] = {{18, 4, 2, 0, {0, 0, 2, 2}}, {36, 8, 4, 0, {2, 2, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_PYRAMID_UP[] = {{20, 4, 2, 0, {0, 0, 2, 2}}, {40, 8, 4, 0, {2, 2, 0, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_PYRAMID_UP[] = {{2, 5, 2, 0, {0, 0, 2, 2}}, {4, 10, 4, 0, {2, 2, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_PYRAMID_UP[] = {{4, 5, 2, 0, {0, 0, 2, 2}}, {8, 10, 4, 0, {2, 2, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_PYRAMID_UP[] = {{6, 5, 2, 0, {0, 0, 2, 2}}, {12, 10, 4, 0, {2, 2, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_PYRAMID_UP[] = {{8, 5, 2, 0, {0, 0, 2, 2}}, {16, 10, 4, 0, {2, 2, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_PYRAMID_UP[] = {{10, 5, 2, 0, {0, 0, 2, 2}}, {20, 10, 4, 0, {2, 2, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_PYRAMID_UP[] = {{12, 5, 2, 0, {0, 0, 2, 2}}, {24, 10, 4, 0, {2, 2, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_PYRAMID_UP[] = {{14, 5, 2, 0, {0, 0, 2, 2}}, {28, 10, 4, 0, {2, 2, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_PYRAMID_UP[] = {{16, 5, 2, 0, {0, 0, 2, 2}}, {32, 10, 4, 0, {2, 2, 0, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_PYRAMID_UP[] = {{2, 6, 2, 0, {0, 0, 2, 2}}, {4, 12, 4, 0, {2, 2, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_PYRAMID_UP[] = {{4, 6, 2, 0, {0, 0, 2, 2}}, {8, 12, 4, 0, {2, 2, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_PYRAMID_UP[] = {{6, 6, 2, 0, {0, 0, 2, 2}}, {12, 12, 4, 0, {2, 2, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_PYRAMID_UP[] = {{8, 6, 2, 0, {0, 0, 2, 2}}, {16, 12, 4, 0, {2, 2, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_PYRAMID_UP[] = {{10, 6, 2, 0, {0, 0, 2, 2}}, {20, 12, 4, 0, {2, 2, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_PYRAMID_UP[] = {{12, 6, 2, 0, {0, 0, 2, 2}}, {24, 12, 4, 0, {2, 2, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_PYRAMID_UP[] = {{14, 6, 2, 0, {0, 0, 2, 2}}, {28, 12, 4, 0, {2, 2, 0, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_PYRAMID_UP[] = {{2, 8, 2, 0, {0, 0, 2, 2}}, {4, 16, 4, 0, {2, 2, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_PYRAMID_UP[] = {{4, 8, 2, 0, {0, 0, 2, 2}}, {8, 16, 4, 0, {2, 2, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_PYRAMID_UP[] = {{6, 8, 2, 0, {0, 0, 2, 2}}, {12, 16, 4, 0, {2, 2, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_PYRAMID_UP[] = {{8, 8, 2, 0, {0, 0, 2, 2}}, {16, 16, 4, 0, {2, 2, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_PYRAMID_UP[] = {{10, 8, 2, 0, {0, 0, 2, 2}}, {20, 16, 4, 0, {2, 2, 0, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_PYRAMID_UP[] = {{2, 10, 2, 0, {0, 0, 2, 2}}, {4, 20, 4, 0, {2, 2, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_PYRAMID_UP[] = {{4, 10, 2, 0, {0, 0, 2, 2}}, {8, 20, 4, 0, {2, 2, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_PYRAMID_UP[] = {{6, 10, 2, 0, {0, 0, 2, 2}}, {12, 20, 4, 0, {2, 2, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_PYRAMID_UP[] = {{8, 10, 2, 0, {0, 0, 2, 2}}, {16, 20, 4, 0, {2, 2, 0, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_PYRAMID_UP[] = {{2, 12, 2, 0, {0, 0, 2, 2}}, {4, 24, 4, 0, {2, 2, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_PYRAMID_UP[] = {{4, 12, 2, 0, {0, 0, 2, 2}}, {8, 24, 4, 0, {2, 2, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_PYRAMID_UP[] = {{6, 12, 2, 0, {0, 0, 2, 2}}, {12, 24, 4, 0, {2, 2, 0, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_PYRAMID_UP[] = {{2, 14, 2, 0, {0, 0, 2, 2}}, {4, 28, 4, 0, {2, 2, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_PYRAMID_UP[] = {{4, 14, 2, 0, {0, 0, 2, 2}}, {8, 28, 4, 0, {2, 2, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_PYRAMID_UP[] = {{6, 14, 2, 0, {0, 0, 2, 2}}, {12, 28, 4, 0, {2, 2, 0, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_PYRAMID_UP[] = {{2, 16, 2, 0, {0, 0, 2, 2}}, {4, 32, 4, 0, {2, 2, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_PYRAMID_UP[] = {{4, 16, 2, 0, {0, 0, 2, 2}}, {8, 32, 4, 0, {2, 2, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_PYRAMID_UP[] = {{2, 18, 2, 0, {0, 0, 2, 2}}, {4, 36, 4, 0, {2, 2, 0, 0}}, {4, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_PYRAMID_UP[] = {{4, 18, 2, 0, {0, 0, 2, 2}}, {8, 36, 4, 0, {2, 2, 0, 0}}, {8, 36, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_PYRAMID_UP[] = {{2, 20, 2, 0, {0, 0, 2, 2}}, {4, 40, 4, 0, {2, 2, 0, 0}}, {4, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_PYRAMID_UP[] = {{4, 20, 2, 0, {0, 0, 2, 2}}, {8, 40, 4, 0, {2, 2, 0, 0}}, {8, 40, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_PYRAMID_UP[] = {{2, 22, 2, 0, {0, 0, 2, 2}}, {4, 44, 4, 0, {2, 2, 0, 0}}, {4, 44, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_PYRAMID_UP[] = {{2, 24, 2, 0, {0, 0, 2, 2}}, {4, 48, 4, 0, {2, 2, 0, 0}}, {4, 48, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_PYRAMID_UP[] = {{2, 26, 2, 0, {0, 0, 2, 2}}, {4, 52, 4, 0, {2, 2, 0, 0}}, {4, 52, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_PYRAMID_UP[] = {{2, 28, 2, 0, {0, 0, 2, 2}}, {4, 56, 4, 0, {2, 2, 0, 0}}, {4, 56, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_PYRAMID_UP[] = {{2, 30, 2, 0, {0, 0, 2, 2}}, {4, 60, 4, 0, {2, 2, 0, 0}}, {4, 60, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_PYRAMID_UP[] = {{2, 32, 2, 0, {0, 0, 2, 2}}, {4, 64, 4, 0, {2, 2, 0, 0}}, {4, 64, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_PYRAMID_UP[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_PYRAMID_UP[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_PYRAMID_UP[] = {
{2, 1, 124, 0, 1, gScenarioBufferData0_PYRAMID_UP, 48, gScenarioKernelData0_PYRAMID_UP, 8},
{4, 1, 240, 0, 1, gScenarioBufferData1_PYRAMID_UP, 48, gScenarioKernelData1_PYRAMID_UP, 8},
{6, 1, 356, 0, 1, gScenarioBufferData2_PYRAMID_UP, 48, gScenarioKernelData2_PYRAMID_UP, 8},
{8, 1, 472, 0, 1, gScenarioBufferData3_PYRAMID_UP, 48, gScenarioKernelData3_PYRAMID_UP, 8},
{10, 1, 588, 0, 1, gScenarioBufferData4_PYRAMID_UP, 48, gScenarioKernelData4_PYRAMID_UP, 8},
{12, 1, 704, 0, 1, gScenarioBufferData5_PYRAMID_UP, 48, gScenarioKernelData5_PYRAMID_UP, 8},
{14, 1, 820, 0, 1, gScenarioBufferData6_PYRAMID_UP, 48, gScenarioKernelData6_PYRAMID_UP, 8},
{16, 1, 936, 0, 1, gScenarioBufferData7_PYRAMID_UP, 48, gScenarioKernelData7_PYRAMID_UP, 8},
{18, 1, 1052, 0, 1, gScenarioBufferData8_PYRAMID_UP, 48, gScenarioKernelData8_PYRAMID_UP, 8},
{20, 1, 1168, 0, 1, gScenarioBufferData9_PYRAMID_UP, 48, gScenarioKernelData9_PYRAMID_UP, 8},
{22, 1, 1284, 0, 1, gScenarioBufferData10_PYRAMID_UP, 48, gScenarioKernelData10_PYRAMID_UP, 8},
{24, 1, 1400, 0, 1, gScenarioBufferData11_PYRAMID_UP, 48, gScenarioKernelData11_PYRAMID_UP, 8},
{26, 1, 1516, 0, 1, gScenarioBufferData12_PYRAMID_UP, 48, gScenarioKernelData12_PYRAMID_UP, 8},
{28, 1, 1632, 0, 1, gScenarioBufferData13_PYRAMID_UP, 48, gScenarioKernelData13_PYRAMID_UP, 8},
{30, 1, 1748, 0, 1, gScenarioBufferData14_PYRAMID_UP, 48, gScenarioKernelData14_PYRAMID_UP, 8},
{32, 1, 1864, 0, 1, gScenarioBufferData15_PYRAMID_UP, 48, gScenarioKernelData15_PYRAMID_UP, 8},
{36, 1, 2096, 0, 1, gScenarioBufferData16_PYRAMID_UP, 48, gScenarioKernelData16_PYRAMID_UP, 8},
{40, 1, 2328, 0, 1, gScenarioBufferData17_PYRAMID_UP, 48, gScenarioKernelData17_PYRAMID_UP, 8},
{44, 1, 2560, 0, 1, gScenarioBufferData18_PYRAMID_UP, 48, gScenarioKernelData18_PYRAMID_UP, 8},
{48, 1, 2792, 0, 1, gScenarioBufferData19_PYRAMID_UP, 48, gScenarioKernelData19_PYRAMID_UP, 8},
{52, 1, 3024, 0, 1, gScenarioBufferData20_PYRAMID_UP, 48, gScenarioKernelData20_PYRAMID_UP, 8},
{56, 1, 3256, 0, 1, gScenarioBufferData21_PYRAMID_UP, 48, gScenarioKernelData21_PYRAMID_UP, 8},
{60, 1, 3488, 0, 1, gScenarioBufferData22_PYRAMID_UP, 48, gScenarioKernelData22_PYRAMID_UP, 8},
{64, 1, 3720, 0, 1, gScenarioBufferData23_PYRAMID_UP, 48, gScenarioKernelData23_PYRAMID_UP, 8},
{2, 2, 216, 0, 1, gScenarioBufferData24_PYRAMID_UP, 48, gScenarioKernelData24_PYRAMID_UP, 8},
{4, 2, 416, 0, 1, gScenarioBufferData25_PYRAMID_UP, 48, gScenarioKernelData25_PYRAMID_UP, 8},
{6, 2, 616, 0, 1, gScenarioBufferData26_PYRAMID_UP, 48, gScenarioKernelData26_PYRAMID_UP, 8},
{8, 2, 816, 0, 1, gScenarioBufferData27_PYRAMID_UP, 48, gScenarioKernelData27_PYRAMID_UP, 8},
{10, 2, 1016, 0, 1, gScenarioBufferData28_PYRAMID_UP, 48, gScenarioKernelData28_PYRAMID_UP, 8},
{12, 2, 1216, 0, 1, gScenarioBufferData29_PYRAMID_UP, 48, gScenarioKernelData29_PYRAMID_UP, 8},
{14, 2, 1416, 0, 1, gScenarioBufferData30_PYRAMID_UP, 48, gScenarioKernelData30_PYRAMID_UP, 8},
{16, 2, 1616, 0, 1, gScenarioBufferData31_PYRAMID_UP, 48, gScenarioKernelData31_PYRAMID_UP, 8},
{18, 2, 1816, 0, 1, gScenarioBufferData32_PYRAMID_UP, 48, gScenarioKernelData32_PYRAMID_UP, 8},
{20, 2, 2016, 0, 1, gScenarioBufferData33_PYRAMID_UP, 48, gScenarioKernelData33_PYRAMID_UP, 8},
{22, 2, 2216, 0, 1, gScenarioBufferData34_PYRAMID_UP, 48, gScenarioKernelData34_PYRAMID_UP, 8},
{24, 2, 2416, 0, 1, gScenarioBufferData35_PYRAMID_UP, 48, gScenarioKernelData35_PYRAMID_UP, 8},
{26, 2, 2616, 0, 1, gScenarioBufferData36_PYRAMID_UP, 48, gScenarioKernelData36_PYRAMID_UP, 8},
{28, 2, 2816, 0, 1, gScenarioBufferData37_PYRAMID_UP, 48, gScenarioKernelData37_PYRAMID_UP, 8},
{30, 2, 3016, 0, 1, gScenarioBufferData38_PYRAMID_UP, 48, gScenarioKernelData38_PYRAMID_UP, 8},
{32, 2, 3216, 0, 1, gScenarioBufferData39_PYRAMID_UP, 48, gScenarioKernelData39_PYRAMID_UP, 8},
{36, 2, 3616, 0, 1, gScenarioBufferData40_PYRAMID_UP, 48, gScenarioKernelData40_PYRAMID_UP, 8},
{2, 3, 308, 0, 1, gScenarioBufferData41_PYRAMID_UP, 48, gScenarioKernelData41_PYRAMID_UP, 8},
{4, 3, 592, 0, 1, gScenarioBufferData42_PYRAMID_UP, 48, gScenarioKernelData42_PYRAMID_UP, 8},
{6, 3, 876, 0, 1, gScenarioBufferData43_PYRAMID_UP, 48, gScenarioKernelData43_PYRAMID_UP, 8},
{8, 3, 1160, 0, 1, gScenarioBufferData44_PYRAMID_UP, 48, gScenarioKernelData44_PYRAMID_UP, 8},
{10, 3, 1444, 0, 1, gScenarioBufferData45_PYRAMID_UP, 48, gScenarioKernelData45_PYRAMID_UP, 8},
{12, 3, 1728, 0, 1, gScenarioBufferData46_PYRAMID_UP, 48, gScenarioKernelData46_PYRAMID_UP, 8},
{14, 3, 2012, 0, 1, gScenarioBufferData47_PYRAMID_UP, 48, gScenarioKernelData47_PYRAMID_UP, 8},
{16, 3, 2296, 0, 1, gScenarioBufferData48_PYRAMID_UP, 48, gScenarioKernelData48_PYRAMID_UP, 8},
{18, 3, 2580, 0, 1, gScenarioBufferData49_PYRAMID_UP, 48, gScenarioKernelData49_PYRAMID_UP, 8},
{20, 3, 2864, 0, 1, gScenarioBufferData50_PYRAMID_UP, 48, gScenarioKernelData50_PYRAMID_UP, 8},
{22, 3, 3148, 0, 1, gScenarioBufferData51_PYRAMID_UP, 48, gScenarioKernelData51_PYRAMID_UP, 8},
{24, 3, 3432, 0, 1, gScenarioBufferData52_PYRAMID_UP, 48, gScenarioKernelData52_PYRAMID_UP, 8},
{26, 3, 3716, 0, 1, gScenarioBufferData53_PYRAMID_UP, 48, gScenarioKernelData53_PYRAMID_UP, 8},
{2, 4, 400, 0, 1, gScenarioBufferData54_PYRAMID_UP, 48, gScenarioKernelData54_PYRAMID_UP, 8},
{4, 4, 768, 0, 1, gScenarioBufferData55_PYRAMID_UP, 48, gScenarioKernelData55_PYRAMID_UP, 8},
{6, 4, 1136, 0, 1, gScenarioBufferData56_PYRAMID_UP, 48, gScenarioKernelData56_PYRAMID_UP, 8},
{8, 4, 1504, 0, 1, gScenarioBufferData57_PYRAMID_UP, 48, gScenarioKernelData57_PYRAMID_UP, 8},
{10, 4, 1872, 0, 1, gScenarioBufferData58_PYRAMID_UP, 48, gScenarioKernelData58_PYRAMID_UP, 8},
{12, 4, 2240, 0, 1, gScenarioBufferData59_PYRAMID_UP, 48, gScenarioKernelData59_PYRAMID_UP, 8},
{14, 4, 2608, 0, 1, gScenarioBufferData60_PYRAMID_UP, 48, gScenarioKernelData60_PYRAMID_UP, 8},
{16, 4, 2976, 0, 1, gScenarioBufferData61_PYRAMID_UP, 48, gScenarioKernelData61_PYRAMID_UP, 8},
{18, 4, 3344, 0, 1, gScenarioBufferData62_PYRAMID_UP, 48, gScenarioKernelData62_PYRAMID_UP, 8},
{20, 4, 3712, 0, 1, gScenarioBufferData63_PYRAMID_UP, 48, gScenarioKernelData63_PYRAMID_UP, 8},
{2, 5, 492, 0, 1, gScenarioBufferData64_PYRAMID_UP, 48, gScenarioKernelData64_PYRAMID_UP, 8},
{4, 5, 944, 0, 1, gScenarioBufferData65_PYRAMID_UP, 48, gScenarioKernelData65_PYRAMID_UP, 8},
{6, 5, 1396, 0, 1, gScenarioBufferData66_PYRAMID_UP, 48, gScenarioKernelData66_PYRAMID_UP, 8},
{8, 5, 1848, 0, 1, gScenarioBufferData67_PYRAMID_UP, 48, gScenarioKernelData67_PYRAMID_UP, 8},
{10, 5, 2300, 0, 1, gScenarioBufferData68_PYRAMID_UP, 48, gScenarioKernelData68_PYRAMID_UP, 8},
{12, 5, 2752, 0, 1, gScenarioBufferData69_PYRAMID_UP, 48, gScenarioKernelData69_PYRAMID_UP, 8},
{14, 5, 3204, 0, 1, gScenarioBufferData70_PYRAMID_UP, 48, gScenarioKernelData70_PYRAMID_UP, 8},
{16, 5, 3656, 0, 1, gScenarioBufferData71_PYRAMID_UP, 48, gScenarioKernelData71_PYRAMID_UP, 8},
{2, 6, 584, 0, 1, gScenarioBufferData72_PYRAMID_UP, 48, gScenarioKernelData72_PYRAMID_UP, 8},
{4, 6, 1120, 0, 1, gScenarioBufferData73_PYRAMID_UP, 48, gScenarioKernelData73_PYRAMID_UP, 8},
{6, 6, 1656, 0, 1, gScenarioBufferData74_PYRAMID_UP, 48, gScenarioKernelData74_PYRAMID_UP, 8},
{8, 6, 2192, 0, 1, gScenarioBufferData75_PYRAMID_UP, 48, gScenarioKernelData75_PYRAMID_UP, 8},
{10, 6, 2728, 0, 1, gScenarioBufferData76_PYRAMID_UP, 48, gScenarioKernelData76_PYRAMID_UP, 8},
{12, 6, 3264, 0, 1, gScenarioBufferData77_PYRAMID_UP, 48, gScenarioKernelData77_PYRAMID_UP, 8},
{14, 6, 3800, 0, 1, gScenarioBufferData78_PYRAMID_UP, 48, gScenarioKernelData78_PYRAMID_UP, 8},
{2, 8, 768, 0, 1, gScenarioBufferData79_PYRAMID_UP, 48, gScenarioKernelData79_PYRAMID_UP, 8},
{4, 8, 1472, 0, 1, gScenarioBufferData80_PYRAMID_UP, 48, gScenarioKernelData80_PYRAMID_UP, 8},
{6, 8, 2176, 0, 1, gScenarioBufferData81_PYRAMID_UP, 48, gScenarioKernelData81_PYRAMID_UP, 8},
{8, 8, 2880, 0, 1, gScenarioBufferData82_PYRAMID_UP, 48, gScenarioKernelData82_PYRAMID_UP, 8},
{10, 8, 3584, 0, 1, gScenarioBufferData83_PYRAMID_UP, 48, gScenarioKernelData83_PYRAMID_UP, 8},
{2, 10, 952, 0, 1, gScenarioBufferData84_PYRAMID_UP, 48, gScenarioKernelData84_PYRAMID_UP, 8},
{4, 10, 1824, 0, 1, gScenarioBufferData85_PYRAMID_UP, 48, gScenarioKernelData85_PYRAMID_UP, 8},
{6, 10, 2696, 0, 1, gScenarioBufferData86_PYRAMID_UP, 48, gScenarioKernelData86_PYRAMID_UP, 8},
{8, 10, 3568, 0, 1, gScenarioBufferData87_PYRAMID_UP, 48, gScenarioKernelData87_PYRAMID_UP, 8},
{2, 12, 1136, 0, 1, gScenarioBufferData88_PYRAMID_UP, 48, gScenarioKernelData88_PYRAMID_UP, 8},
{4, 12, 2176, 0, 1, gScenarioBufferData89_PYRAMID_UP, 48, gScenarioKernelData89_PYRAMID_UP, 8},
{6, 12, 3216, 0, 1, gScenarioBufferData90_PYRAMID_UP, 48, gScenarioKernelData90_PYRAMID_UP, 8},
{2, 14, 1320, 0, 1, gScenarioBufferData91_PYRAMID_UP, 48, gScenarioKernelData91_PYRAMID_UP, 8},
{4, 14, 2528, 0, 1, gScenarioBufferData92_PYRAMID_UP, 48, gScenarioKernelData92_PYRAMID_UP, 8},
{6, 14, 3736, 0, 1, gScenarioBufferData93_PYRAMID_UP, 48, gScenarioKernelData93_PYRAMID_UP, 8},
{2, 16, 1504, 0, 1, gScenarioBufferData94_PYRAMID_UP, 48, gScenarioKernelData94_PYRAMID_UP, 8},
{4, 16, 2880, 0, 1, gScenarioBufferData95_PYRAMID_UP, 48, gScenarioKernelData95_PYRAMID_UP, 8},
{2, 18, 1688, 0, 1, gScenarioBufferData96_PYRAMID_UP, 48, gScenarioKernelData96_PYRAMID_UP, 8},
{4, 18, 3232, 0, 1, gScenarioBufferData97_PYRAMID_UP, 48, gScenarioKernelData97_PYRAMID_UP, 8},
{2, 20, 1872, 0, 1, gScenarioBufferData98_PYRAMID_UP, 48, gScenarioKernelData98_PYRAMID_UP, 8},
{4, 20, 3584, 0, 1, gScenarioBufferData99_PYRAMID_UP, 48, gScenarioKernelData99_PYRAMID_UP, 8},
{2, 22, 2056, 0, 1, gScenarioBufferData100_PYRAMID_UP, 48, gScenarioKernelData100_PYRAMID_UP, 8},
{2, 24, 2240, 0, 1, gScenarioBufferData101_PYRAMID_UP, 48, gScenarioKernelData101_PYRAMID_UP, 8},
{2, 26, 2424, 0, 1, gScenarioBufferData102_PYRAMID_UP, 48, gScenarioKernelData102_PYRAMID_UP, 8},
{2, 28, 2608, 0, 1, gScenarioBufferData103_PYRAMID_UP, 48, gScenarioKernelData103_PYRAMID_UP, 8},
{2, 30, 2792, 0, 1, gScenarioBufferData104_PYRAMID_UP, 48, gScenarioKernelData104_PYRAMID_UP, 8},
{2, 32, 2976, 0, 1, gScenarioBufferData105_PYRAMID_UP, 48, gScenarioKernelData105_PYRAMID_UP, 8}
};

static acf_scenario_list gScenarioList_PYRAMID_UP = {
106, //number of scenarios
gScenarioArray_PYRAMID_UP};
//**************************************************************

class PYRAMID_UP : public ACF_Process_APU
{

public:
   PYRAMID_UP(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("PYRAMID_UP");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("PYRAMID_UP",
                                        PYRAMID_UP_LOAD_SEGMENTS,
                                        PYRAMID_UP_LOAD_PMEM, PYRAMID_UP_LOAD_PMEM_SIZE,
                                        PYRAMID_UP_LOAD_DMEM, PYRAMID_UP_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(PYRAMID_UP_APEX_LOG_BUFFER, PYRAMID_UP_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_PYRAMID_UP);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_PYRAMID_UP

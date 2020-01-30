#ifndef _ACF_PROCESS_APU_PYRAMID_DOWN
#define _ACF_PROCESS_APU_PYRAMID_DOWN

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <PYRAMID_DOWN_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_PYRAMID_DOWN[] = {{4, 2, 2, 0, {0, 0, 2, 2}}, {4, 2, 4, 0, {2, 2, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_PYRAMID_DOWN[] = {{8, 2, 2, 0, {0, 0, 2, 2}}, {8, 2, 4, 0, {2, 2, 0, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_PYRAMID_DOWN[] = {{12, 2, 2, 0, {0, 0, 2, 2}}, {12, 2, 4, 0, {2, 2, 0, 0}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_PYRAMID_DOWN[] = {{16, 2, 2, 0, {0, 0, 2, 2}}, {16, 2, 4, 0, {2, 2, 0, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_PYRAMID_DOWN[] = {{20, 2, 2, 0, {0, 0, 2, 2}}, {20, 2, 4, 0, {2, 2, 0, 0}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_PYRAMID_DOWN[] = {{24, 2, 2, 0, {0, 0, 2, 2}}, {24, 2, 4, 0, {2, 2, 0, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_PYRAMID_DOWN[] = {{28, 2, 2, 0, {0, 0, 2, 2}}, {28, 2, 4, 0, {2, 2, 0, 0}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_PYRAMID_DOWN[] = {{32, 2, 2, 0, {0, 0, 2, 2}}, {32, 2, 4, 0, {2, 2, 0, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_PYRAMID_DOWN[] = {{36, 2, 2, 0, {0, 0, 2, 2}}, {36, 2, 4, 0, {2, 2, 0, 0}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_PYRAMID_DOWN[] = {{40, 2, 2, 0, {0, 0, 2, 2}}, {40, 2, 4, 0, {2, 2, 0, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_PYRAMID_DOWN[] = {{44, 2, 2, 0, {0, 0, 2, 2}}, {44, 2, 4, 0, {2, 2, 0, 0}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_PYRAMID_DOWN[] = {{48, 2, 2, 0, {0, 0, 2, 2}}, {48, 2, 4, 0, {2, 2, 0, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_PYRAMID_DOWN[] = {{52, 2, 2, 0, {0, 0, 2, 2}}, {52, 2, 4, 0, {2, 2, 0, 0}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_PYRAMID_DOWN[] = {{56, 2, 2, 0, {0, 0, 2, 2}}, {56, 2, 4, 0, {2, 2, 0, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_PYRAMID_DOWN[] = {{60, 2, 2, 0, {0, 0, 2, 2}}, {60, 2, 4, 0, {2, 2, 0, 0}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_PYRAMID_DOWN[] = {{64, 2, 2, 0, {0, 0, 2, 2}}, {64, 2, 4, 0, {2, 2, 0, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_PYRAMID_DOWN[] = {{72, 2, 2, 0, {0, 0, 2, 2}}, {72, 2, 4, 0, {2, 2, 0, 0}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_PYRAMID_DOWN[] = {{80, 2, 2, 0, {0, 0, 2, 2}}, {80, 2, 4, 0, {2, 2, 0, 0}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_PYRAMID_DOWN[] = {{88, 2, 2, 0, {0, 0, 2, 2}}, {88, 2, 4, 0, {2, 2, 0, 0}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_PYRAMID_DOWN[] = {{96, 2, 2, 0, {0, 0, 2, 2}}, {96, 2, 4, 0, {2, 2, 0, 0}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_PYRAMID_DOWN[] = {{104, 2, 2, 0, {0, 0, 2, 2}}, {104, 2, 4, 0, {2, 2, 0, 0}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_PYRAMID_DOWN[] = {{112, 2, 2, 0, {0, 0, 2, 2}}, {112, 2, 4, 0, {2, 2, 0, 0}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_PYRAMID_DOWN[] = {{120, 2, 2, 0, {0, 0, 2, 2}}, {120, 2, 4, 0, {2, 2, 0, 0}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_PYRAMID_DOWN[] = {{128, 2, 2, 0, {0, 0, 2, 2}}, {128, 2, 4, 0, {2, 2, 0, 0}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_PYRAMID_DOWN[] = {{4, 4, 2, 0, {0, 0, 2, 2}}, {4, 4, 4, 0, {2, 2, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_PYRAMID_DOWN[] = {{8, 4, 2, 0, {0, 0, 2, 2}}, {8, 4, 4, 0, {2, 2, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_PYRAMID_DOWN[] = {{12, 4, 2, 0, {0, 0, 2, 2}}, {12, 4, 4, 0, {2, 2, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_PYRAMID_DOWN[] = {{16, 4, 2, 0, {0, 0, 2, 2}}, {16, 4, 4, 0, {2, 2, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_PYRAMID_DOWN[] = {{20, 4, 2, 0, {0, 0, 2, 2}}, {20, 4, 4, 0, {2, 2, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_PYRAMID_DOWN[] = {{24, 4, 2, 0, {0, 0, 2, 2}}, {24, 4, 4, 0, {2, 2, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_PYRAMID_DOWN[] = {{28, 4, 2, 0, {0, 0, 2, 2}}, {28, 4, 4, 0, {2, 2, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_PYRAMID_DOWN[] = {{32, 4, 2, 0, {0, 0, 2, 2}}, {32, 4, 4, 0, {2, 2, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_PYRAMID_DOWN[] = {{36, 4, 2, 0, {0, 0, 2, 2}}, {36, 4, 4, 0, {2, 2, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_PYRAMID_DOWN[] = {{40, 4, 2, 0, {0, 0, 2, 2}}, {40, 4, 4, 0, {2, 2, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_PYRAMID_DOWN[] = {{44, 4, 2, 0, {0, 0, 2, 2}}, {44, 4, 4, 0, {2, 2, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_PYRAMID_DOWN[] = {{48, 4, 2, 0, {0, 0, 2, 2}}, {48, 4, 4, 0, {2, 2, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_PYRAMID_DOWN[] = {{52, 4, 2, 0, {0, 0, 2, 2}}, {52, 4, 4, 0, {2, 2, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_PYRAMID_DOWN[] = {{56, 4, 2, 0, {0, 0, 2, 2}}, {56, 4, 4, 0, {2, 2, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_PYRAMID_DOWN[] = {{60, 4, 2, 0, {0, 0, 2, 2}}, {60, 4, 4, 0, {2, 2, 0, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_PYRAMID_DOWN[] = {{64, 4, 2, 0, {0, 0, 2, 2}}, {64, 4, 4, 0, {2, 2, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_PYRAMID_DOWN[] = {{72, 4, 2, 0, {0, 0, 2, 2}}, {72, 4, 4, 0, {2, 2, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_PYRAMID_DOWN[] = {{4, 6, 2, 0, {0, 0, 2, 2}}, {4, 6, 4, 0, {2, 2, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_PYRAMID_DOWN[] = {{8, 6, 2, 0, {0, 0, 2, 2}}, {8, 6, 4, 0, {2, 2, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_PYRAMID_DOWN[] = {{12, 6, 2, 0, {0, 0, 2, 2}}, {12, 6, 4, 0, {2, 2, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_PYRAMID_DOWN[] = {{16, 6, 2, 0, {0, 0, 2, 2}}, {16, 6, 4, 0, {2, 2, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_PYRAMID_DOWN[] = {{20, 6, 2, 0, {0, 0, 2, 2}}, {20, 6, 4, 0, {2, 2, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_PYRAMID_DOWN[] = {{24, 6, 2, 0, {0, 0, 2, 2}}, {24, 6, 4, 0, {2, 2, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_PYRAMID_DOWN[] = {{28, 6, 2, 0, {0, 0, 2, 2}}, {28, 6, 4, 0, {2, 2, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_PYRAMID_DOWN[] = {{32, 6, 2, 0, {0, 0, 2, 2}}, {32, 6, 4, 0, {2, 2, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_PYRAMID_DOWN[] = {{36, 6, 2, 0, {0, 0, 2, 2}}, {36, 6, 4, 0, {2, 2, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_PYRAMID_DOWN[] = {{40, 6, 2, 0, {0, 0, 2, 2}}, {40, 6, 4, 0, {2, 2, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_PYRAMID_DOWN[] = {{44, 6, 2, 0, {0, 0, 2, 2}}, {44, 6, 4, 0, {2, 2, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_PYRAMID_DOWN[] = {{48, 6, 2, 0, {0, 0, 2, 2}}, {48, 6, 4, 0, {2, 2, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_PYRAMID_DOWN[] = {{52, 6, 2, 0, {0, 0, 2, 2}}, {52, 6, 4, 0, {2, 2, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_PYRAMID_DOWN[] = {{4, 8, 2, 0, {0, 0, 2, 2}}, {4, 8, 4, 0, {2, 2, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_PYRAMID_DOWN[] = {{8, 8, 2, 0, {0, 0, 2, 2}}, {8, 8, 4, 0, {2, 2, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_PYRAMID_DOWN[] = {{12, 8, 2, 0, {0, 0, 2, 2}}, {12, 8, 4, 0, {2, 2, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_PYRAMID_DOWN[] = {{16, 8, 2, 0, {0, 0, 2, 2}}, {16, 8, 4, 0, {2, 2, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_PYRAMID_DOWN[] = {{20, 8, 2, 0, {0, 0, 2, 2}}, {20, 8, 4, 0, {2, 2, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_PYRAMID_DOWN[] = {{24, 8, 2, 0, {0, 0, 2, 2}}, {24, 8, 4, 0, {2, 2, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_PYRAMID_DOWN[] = {{28, 8, 2, 0, {0, 0, 2, 2}}, {28, 8, 4, 0, {2, 2, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_PYRAMID_DOWN[] = {{32, 8, 2, 0, {0, 0, 2, 2}}, {32, 8, 4, 0, {2, 2, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_PYRAMID_DOWN[] = {{36, 8, 2, 0, {0, 0, 2, 2}}, {36, 8, 4, 0, {2, 2, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_PYRAMID_DOWN[] = {{40, 8, 2, 0, {0, 0, 2, 2}}, {40, 8, 4, 0, {2, 2, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_PYRAMID_DOWN[] = {{4, 10, 2, 0, {0, 0, 2, 2}}, {4, 10, 4, 0, {2, 2, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_PYRAMID_DOWN[] = {{8, 10, 2, 0, {0, 0, 2, 2}}, {8, 10, 4, 0, {2, 2, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_PYRAMID_DOWN[] = {{12, 10, 2, 0, {0, 0, 2, 2}}, {12, 10, 4, 0, {2, 2, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_PYRAMID_DOWN[] = {{16, 10, 2, 0, {0, 0, 2, 2}}, {16, 10, 4, 0, {2, 2, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_PYRAMID_DOWN[] = {{20, 10, 2, 0, {0, 0, 2, 2}}, {20, 10, 4, 0, {2, 2, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_PYRAMID_DOWN[] = {{24, 10, 2, 0, {0, 0, 2, 2}}, {24, 10, 4, 0, {2, 2, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_PYRAMID_DOWN[] = {{28, 10, 2, 0, {0, 0, 2, 2}}, {28, 10, 4, 0, {2, 2, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_PYRAMID_DOWN[] = {{32, 10, 2, 0, {0, 0, 2, 2}}, {32, 10, 4, 0, {2, 2, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_PYRAMID_DOWN[] = {{4, 12, 2, 0, {0, 0, 2, 2}}, {4, 12, 4, 0, {2, 2, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_PYRAMID_DOWN[] = {{8, 12, 2, 0, {0, 0, 2, 2}}, {8, 12, 4, 0, {2, 2, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_PYRAMID_DOWN[] = {{12, 12, 2, 0, {0, 0, 2, 2}}, {12, 12, 4, 0, {2, 2, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_PYRAMID_DOWN[] = {{16, 12, 2, 0, {0, 0, 2, 2}}, {16, 12, 4, 0, {2, 2, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_PYRAMID_DOWN[] = {{20, 12, 2, 0, {0, 0, 2, 2}}, {20, 12, 4, 0, {2, 2, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_PYRAMID_DOWN[] = {{24, 12, 2, 0, {0, 0, 2, 2}}, {24, 12, 4, 0, {2, 2, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_PYRAMID_DOWN[] = {{4, 16, 2, 0, {0, 0, 2, 2}}, {4, 16, 4, 0, {2, 2, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_PYRAMID_DOWN[] = {{8, 16, 2, 0, {0, 0, 2, 2}}, {8, 16, 4, 0, {2, 2, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_PYRAMID_DOWN[] = {{12, 16, 2, 0, {0, 0, 2, 2}}, {12, 16, 4, 0, {2, 2, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_PYRAMID_DOWN[] = {{16, 16, 2, 0, {0, 0, 2, 2}}, {16, 16, 4, 0, {2, 2, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_PYRAMID_DOWN[] = {{20, 16, 2, 0, {0, 0, 2, 2}}, {20, 16, 4, 0, {2, 2, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_PYRAMID_DOWN[] = {{4, 20, 2, 0, {0, 0, 2, 2}}, {4, 20, 4, 0, {2, 2, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_PYRAMID_DOWN[] = {{8, 20, 2, 0, {0, 0, 2, 2}}, {8, 20, 4, 0, {2, 2, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_PYRAMID_DOWN[] = {{12, 20, 2, 0, {0, 0, 2, 2}}, {12, 20, 4, 0, {2, 2, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_PYRAMID_DOWN[] = {{16, 20, 2, 0, {0, 0, 2, 2}}, {16, 20, 4, 0, {2, 2, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_PYRAMID_DOWN[] = {{4, 24, 2, 0, {0, 0, 2, 2}}, {4, 24, 4, 0, {2, 2, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_PYRAMID_DOWN[] = {{8, 24, 2, 0, {0, 0, 2, 2}}, {8, 24, 4, 0, {2, 2, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_PYRAMID_DOWN[] = {{12, 24, 2, 0, {0, 0, 2, 2}}, {12, 24, 4, 0, {2, 2, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_PYRAMID_DOWN[] = {{4, 28, 2, 0, {0, 0, 2, 2}}, {4, 28, 4, 0, {2, 2, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_PYRAMID_DOWN[] = {{8, 28, 2, 0, {0, 0, 2, 2}}, {8, 28, 4, 0, {2, 2, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_PYRAMID_DOWN[] = {{4, 32, 2, 0, {0, 0, 2, 2}}, {4, 32, 4, 0, {2, 2, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_PYRAMID_DOWN[] = {{8, 32, 2, 0, {0, 0, 2, 2}}, {8, 32, 4, 0, {2, 2, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_PYRAMID_DOWN[] = {{4, 36, 2, 0, {0, 0, 2, 2}}, {4, 36, 4, 0, {2, 2, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_PYRAMID_DOWN[] = {{8, 36, 2, 0, {0, 0, 2, 2}}, {8, 36, 4, 0, {2, 2, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_PYRAMID_DOWN[] = {{4, 40, 2, 0, {0, 0, 2, 2}}, {4, 40, 4, 0, {2, 2, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_PYRAMID_DOWN[] = {{8, 40, 2, 0, {0, 0, 2, 2}}, {8, 40, 4, 0, {2, 2, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_PYRAMID_DOWN[] = {{4, 44, 2, 0, {0, 0, 2, 2}}, {4, 44, 4, 0, {2, 2, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_PYRAMID_DOWN[] = {{4, 48, 2, 0, {0, 0, 2, 2}}, {4, 48, 4, 0, {2, 2, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_PYRAMID_DOWN[] = {{4, 52, 2, 0, {0, 0, 2, 2}}, {4, 52, 4, 0, {2, 2, 0, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_PYRAMID_DOWN[] = {{4, 56, 2, 0, {0, 0, 2, 2}}, {4, 56, 4, 0, {2, 2, 0, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_PYRAMID_DOWN[] = {{4, 60, 2, 0, {0, 0, 2, 2}}, {4, 60, 4, 0, {2, 2, 0, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_PYRAMID_DOWN[] = {{4, 64, 2, 0, {0, 0, 2, 2}}, {4, 64, 4, 0, {2, 2, 0, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_PYRAMID_DOWN[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_PYRAMID_DOWN[] = {
{2, 1, 132, 0, 1, gScenarioBufferData0_PYRAMID_DOWN, 48, gScenarioKernelData0_PYRAMID_DOWN, 8},
{4, 1, 248, 0, 1, gScenarioBufferData1_PYRAMID_DOWN, 48, gScenarioKernelData1_PYRAMID_DOWN, 8},
{6, 1, 364, 0, 1, gScenarioBufferData2_PYRAMID_DOWN, 48, gScenarioKernelData2_PYRAMID_DOWN, 8},
{8, 1, 480, 0, 1, gScenarioBufferData3_PYRAMID_DOWN, 48, gScenarioKernelData3_PYRAMID_DOWN, 8},
{10, 1, 596, 0, 1, gScenarioBufferData4_PYRAMID_DOWN, 48, gScenarioKernelData4_PYRAMID_DOWN, 8},
{12, 1, 712, 0, 1, gScenarioBufferData5_PYRAMID_DOWN, 48, gScenarioKernelData5_PYRAMID_DOWN, 8},
{14, 1, 828, 0, 1, gScenarioBufferData6_PYRAMID_DOWN, 48, gScenarioKernelData6_PYRAMID_DOWN, 8},
{16, 1, 944, 0, 1, gScenarioBufferData7_PYRAMID_DOWN, 48, gScenarioKernelData7_PYRAMID_DOWN, 8},
{18, 1, 1060, 0, 1, gScenarioBufferData8_PYRAMID_DOWN, 48, gScenarioKernelData8_PYRAMID_DOWN, 8},
{20, 1, 1176, 0, 1, gScenarioBufferData9_PYRAMID_DOWN, 48, gScenarioKernelData9_PYRAMID_DOWN, 8},
{22, 1, 1292, 0, 1, gScenarioBufferData10_PYRAMID_DOWN, 48, gScenarioKernelData10_PYRAMID_DOWN, 8},
{24, 1, 1408, 0, 1, gScenarioBufferData11_PYRAMID_DOWN, 48, gScenarioKernelData11_PYRAMID_DOWN, 8},
{26, 1, 1524, 0, 1, gScenarioBufferData12_PYRAMID_DOWN, 48, gScenarioKernelData12_PYRAMID_DOWN, 8},
{28, 1, 1640, 0, 1, gScenarioBufferData13_PYRAMID_DOWN, 48, gScenarioKernelData13_PYRAMID_DOWN, 8},
{30, 1, 1756, 0, 1, gScenarioBufferData14_PYRAMID_DOWN, 48, gScenarioKernelData14_PYRAMID_DOWN, 8},
{32, 1, 1872, 0, 1, gScenarioBufferData15_PYRAMID_DOWN, 48, gScenarioKernelData15_PYRAMID_DOWN, 8},
{36, 1, 2104, 0, 1, gScenarioBufferData16_PYRAMID_DOWN, 48, gScenarioKernelData16_PYRAMID_DOWN, 8},
{40, 1, 2336, 0, 1, gScenarioBufferData17_PYRAMID_DOWN, 48, gScenarioKernelData17_PYRAMID_DOWN, 8},
{44, 1, 2568, 0, 1, gScenarioBufferData18_PYRAMID_DOWN, 48, gScenarioKernelData18_PYRAMID_DOWN, 8},
{48, 1, 2800, 0, 1, gScenarioBufferData19_PYRAMID_DOWN, 48, gScenarioKernelData19_PYRAMID_DOWN, 8},
{52, 1, 3032, 0, 1, gScenarioBufferData20_PYRAMID_DOWN, 48, gScenarioKernelData20_PYRAMID_DOWN, 8},
{56, 1, 3264, 0, 1, gScenarioBufferData21_PYRAMID_DOWN, 48, gScenarioKernelData21_PYRAMID_DOWN, 8},
{60, 1, 3496, 0, 1, gScenarioBufferData22_PYRAMID_DOWN, 48, gScenarioKernelData22_PYRAMID_DOWN, 8},
{64, 1, 3728, 0, 1, gScenarioBufferData23_PYRAMID_DOWN, 48, gScenarioKernelData23_PYRAMID_DOWN, 8},
{2, 2, 232, 0, 1, gScenarioBufferData24_PYRAMID_DOWN, 48, gScenarioKernelData24_PYRAMID_DOWN, 8},
{4, 2, 432, 0, 1, gScenarioBufferData25_PYRAMID_DOWN, 48, gScenarioKernelData25_PYRAMID_DOWN, 8},
{6, 2, 632, 0, 1, gScenarioBufferData26_PYRAMID_DOWN, 48, gScenarioKernelData26_PYRAMID_DOWN, 8},
{8, 2, 832, 0, 1, gScenarioBufferData27_PYRAMID_DOWN, 48, gScenarioKernelData27_PYRAMID_DOWN, 8},
{10, 2, 1032, 0, 1, gScenarioBufferData28_PYRAMID_DOWN, 48, gScenarioKernelData28_PYRAMID_DOWN, 8},
{12, 2, 1232, 0, 1, gScenarioBufferData29_PYRAMID_DOWN, 48, gScenarioKernelData29_PYRAMID_DOWN, 8},
{14, 2, 1432, 0, 1, gScenarioBufferData30_PYRAMID_DOWN, 48, gScenarioKernelData30_PYRAMID_DOWN, 8},
{16, 2, 1632, 0, 1, gScenarioBufferData31_PYRAMID_DOWN, 48, gScenarioKernelData31_PYRAMID_DOWN, 8},
{18, 2, 1832, 0, 1, gScenarioBufferData32_PYRAMID_DOWN, 48, gScenarioKernelData32_PYRAMID_DOWN, 8},
{20, 2, 2032, 0, 1, gScenarioBufferData33_PYRAMID_DOWN, 48, gScenarioKernelData33_PYRAMID_DOWN, 8},
{22, 2, 2232, 0, 1, gScenarioBufferData34_PYRAMID_DOWN, 48, gScenarioKernelData34_PYRAMID_DOWN, 8},
{24, 2, 2432, 0, 1, gScenarioBufferData35_PYRAMID_DOWN, 48, gScenarioKernelData35_PYRAMID_DOWN, 8},
{26, 2, 2632, 0, 1, gScenarioBufferData36_PYRAMID_DOWN, 48, gScenarioKernelData36_PYRAMID_DOWN, 8},
{28, 2, 2832, 0, 1, gScenarioBufferData37_PYRAMID_DOWN, 48, gScenarioKernelData37_PYRAMID_DOWN, 8},
{30, 2, 3032, 0, 1, gScenarioBufferData38_PYRAMID_DOWN, 48, gScenarioKernelData38_PYRAMID_DOWN, 8},
{32, 2, 3232, 0, 1, gScenarioBufferData39_PYRAMID_DOWN, 48, gScenarioKernelData39_PYRAMID_DOWN, 8},
{36, 2, 3632, 0, 1, gScenarioBufferData40_PYRAMID_DOWN, 48, gScenarioKernelData40_PYRAMID_DOWN, 8},
{2, 3, 332, 0, 1, gScenarioBufferData41_PYRAMID_DOWN, 48, gScenarioKernelData41_PYRAMID_DOWN, 8},
{4, 3, 616, 0, 1, gScenarioBufferData42_PYRAMID_DOWN, 48, gScenarioKernelData42_PYRAMID_DOWN, 8},
{6, 3, 900, 0, 1, gScenarioBufferData43_PYRAMID_DOWN, 48, gScenarioKernelData43_PYRAMID_DOWN, 8},
{8, 3, 1184, 0, 1, gScenarioBufferData44_PYRAMID_DOWN, 48, gScenarioKernelData44_PYRAMID_DOWN, 8},
{10, 3, 1468, 0, 1, gScenarioBufferData45_PYRAMID_DOWN, 48, gScenarioKernelData45_PYRAMID_DOWN, 8},
{12, 3, 1752, 0, 1, gScenarioBufferData46_PYRAMID_DOWN, 48, gScenarioKernelData46_PYRAMID_DOWN, 8},
{14, 3, 2036, 0, 1, gScenarioBufferData47_PYRAMID_DOWN, 48, gScenarioKernelData47_PYRAMID_DOWN, 8},
{16, 3, 2320, 0, 1, gScenarioBufferData48_PYRAMID_DOWN, 48, gScenarioKernelData48_PYRAMID_DOWN, 8},
{18, 3, 2604, 0, 1, gScenarioBufferData49_PYRAMID_DOWN, 48, gScenarioKernelData49_PYRAMID_DOWN, 8},
{20, 3, 2888, 0, 1, gScenarioBufferData50_PYRAMID_DOWN, 48, gScenarioKernelData50_PYRAMID_DOWN, 8},
{22, 3, 3172, 0, 1, gScenarioBufferData51_PYRAMID_DOWN, 48, gScenarioKernelData51_PYRAMID_DOWN, 8},
{24, 3, 3456, 0, 1, gScenarioBufferData52_PYRAMID_DOWN, 48, gScenarioKernelData52_PYRAMID_DOWN, 8},
{26, 3, 3740, 0, 1, gScenarioBufferData53_PYRAMID_DOWN, 48, gScenarioKernelData53_PYRAMID_DOWN, 8},
{2, 4, 432, 0, 1, gScenarioBufferData54_PYRAMID_DOWN, 48, gScenarioKernelData54_PYRAMID_DOWN, 8},
{4, 4, 800, 0, 1, gScenarioBufferData55_PYRAMID_DOWN, 48, gScenarioKernelData55_PYRAMID_DOWN, 8},
{6, 4, 1168, 0, 1, gScenarioBufferData56_PYRAMID_DOWN, 48, gScenarioKernelData56_PYRAMID_DOWN, 8},
{8, 4, 1536, 0, 1, gScenarioBufferData57_PYRAMID_DOWN, 48, gScenarioKernelData57_PYRAMID_DOWN, 8},
{10, 4, 1904, 0, 1, gScenarioBufferData58_PYRAMID_DOWN, 48, gScenarioKernelData58_PYRAMID_DOWN, 8},
{12, 4, 2272, 0, 1, gScenarioBufferData59_PYRAMID_DOWN, 48, gScenarioKernelData59_PYRAMID_DOWN, 8},
{14, 4, 2640, 0, 1, gScenarioBufferData60_PYRAMID_DOWN, 48, gScenarioKernelData60_PYRAMID_DOWN, 8},
{16, 4, 3008, 0, 1, gScenarioBufferData61_PYRAMID_DOWN, 48, gScenarioKernelData61_PYRAMID_DOWN, 8},
{18, 4, 3376, 0, 1, gScenarioBufferData62_PYRAMID_DOWN, 48, gScenarioKernelData62_PYRAMID_DOWN, 8},
{20, 4, 3744, 0, 1, gScenarioBufferData63_PYRAMID_DOWN, 48, gScenarioKernelData63_PYRAMID_DOWN, 8},
{2, 5, 532, 0, 1, gScenarioBufferData64_PYRAMID_DOWN, 48, gScenarioKernelData64_PYRAMID_DOWN, 8},
{4, 5, 984, 0, 1, gScenarioBufferData65_PYRAMID_DOWN, 48, gScenarioKernelData65_PYRAMID_DOWN, 8},
{6, 5, 1436, 0, 1, gScenarioBufferData66_PYRAMID_DOWN, 48, gScenarioKernelData66_PYRAMID_DOWN, 8},
{8, 5, 1888, 0, 1, gScenarioBufferData67_PYRAMID_DOWN, 48, gScenarioKernelData67_PYRAMID_DOWN, 8},
{10, 5, 2340, 0, 1, gScenarioBufferData68_PYRAMID_DOWN, 48, gScenarioKernelData68_PYRAMID_DOWN, 8},
{12, 5, 2792, 0, 1, gScenarioBufferData69_PYRAMID_DOWN, 48, gScenarioKernelData69_PYRAMID_DOWN, 8},
{14, 5, 3244, 0, 1, gScenarioBufferData70_PYRAMID_DOWN, 48, gScenarioKernelData70_PYRAMID_DOWN, 8},
{16, 5, 3696, 0, 1, gScenarioBufferData71_PYRAMID_DOWN, 48, gScenarioKernelData71_PYRAMID_DOWN, 8},
{2, 6, 632, 0, 1, gScenarioBufferData72_PYRAMID_DOWN, 48, gScenarioKernelData72_PYRAMID_DOWN, 8},
{4, 6, 1168, 0, 1, gScenarioBufferData73_PYRAMID_DOWN, 48, gScenarioKernelData73_PYRAMID_DOWN, 8},
{6, 6, 1704, 0, 1, gScenarioBufferData74_PYRAMID_DOWN, 48, gScenarioKernelData74_PYRAMID_DOWN, 8},
{8, 6, 2240, 0, 1, gScenarioBufferData75_PYRAMID_DOWN, 48, gScenarioKernelData75_PYRAMID_DOWN, 8},
{10, 6, 2776, 0, 1, gScenarioBufferData76_PYRAMID_DOWN, 48, gScenarioKernelData76_PYRAMID_DOWN, 8},
{12, 6, 3312, 0, 1, gScenarioBufferData77_PYRAMID_DOWN, 48, gScenarioKernelData77_PYRAMID_DOWN, 8},
{2, 8, 832, 0, 1, gScenarioBufferData78_PYRAMID_DOWN, 48, gScenarioKernelData78_PYRAMID_DOWN, 8},
{4, 8, 1536, 0, 1, gScenarioBufferData79_PYRAMID_DOWN, 48, gScenarioKernelData79_PYRAMID_DOWN, 8},
{6, 8, 2240, 0, 1, gScenarioBufferData80_PYRAMID_DOWN, 48, gScenarioKernelData80_PYRAMID_DOWN, 8},
{8, 8, 2944, 0, 1, gScenarioBufferData81_PYRAMID_DOWN, 48, gScenarioKernelData81_PYRAMID_DOWN, 8},
{10, 8, 3648, 0, 1, gScenarioBufferData82_PYRAMID_DOWN, 48, gScenarioKernelData82_PYRAMID_DOWN, 8},
{2, 10, 1032, 0, 1, gScenarioBufferData83_PYRAMID_DOWN, 48, gScenarioKernelData83_PYRAMID_DOWN, 8},
{4, 10, 1904, 0, 1, gScenarioBufferData84_PYRAMID_DOWN, 48, gScenarioKernelData84_PYRAMID_DOWN, 8},
{6, 10, 2776, 0, 1, gScenarioBufferData85_PYRAMID_DOWN, 48, gScenarioKernelData85_PYRAMID_DOWN, 8},
{8, 10, 3648, 0, 1, gScenarioBufferData86_PYRAMID_DOWN, 48, gScenarioKernelData86_PYRAMID_DOWN, 8},
{2, 12, 1232, 0, 1, gScenarioBufferData87_PYRAMID_DOWN, 48, gScenarioKernelData87_PYRAMID_DOWN, 8},
{4, 12, 2272, 0, 1, gScenarioBufferData88_PYRAMID_DOWN, 48, gScenarioKernelData88_PYRAMID_DOWN, 8},
{6, 12, 3312, 0, 1, gScenarioBufferData89_PYRAMID_DOWN, 48, gScenarioKernelData89_PYRAMID_DOWN, 8},
{2, 14, 1432, 0, 1, gScenarioBufferData90_PYRAMID_DOWN, 48, gScenarioKernelData90_PYRAMID_DOWN, 8},
{4, 14, 2640, 0, 1, gScenarioBufferData91_PYRAMID_DOWN, 48, gScenarioKernelData91_PYRAMID_DOWN, 8},
{2, 16, 1632, 0, 1, gScenarioBufferData92_PYRAMID_DOWN, 48, gScenarioKernelData92_PYRAMID_DOWN, 8},
{4, 16, 3008, 0, 1, gScenarioBufferData93_PYRAMID_DOWN, 48, gScenarioKernelData93_PYRAMID_DOWN, 8},
{2, 18, 1832, 0, 1, gScenarioBufferData94_PYRAMID_DOWN, 48, gScenarioKernelData94_PYRAMID_DOWN, 8},
{4, 18, 3376, 0, 1, gScenarioBufferData95_PYRAMID_DOWN, 48, gScenarioKernelData95_PYRAMID_DOWN, 8},
{2, 20, 2032, 0, 1, gScenarioBufferData96_PYRAMID_DOWN, 48, gScenarioKernelData96_PYRAMID_DOWN, 8},
{4, 20, 3744, 0, 1, gScenarioBufferData97_PYRAMID_DOWN, 48, gScenarioKernelData97_PYRAMID_DOWN, 8},
{2, 22, 2232, 0, 1, gScenarioBufferData98_PYRAMID_DOWN, 48, gScenarioKernelData98_PYRAMID_DOWN, 8},
{2, 24, 2432, 0, 1, gScenarioBufferData99_PYRAMID_DOWN, 48, gScenarioKernelData99_PYRAMID_DOWN, 8},
{2, 26, 2632, 0, 1, gScenarioBufferData100_PYRAMID_DOWN, 48, gScenarioKernelData100_PYRAMID_DOWN, 8},
{2, 28, 2832, 0, 1, gScenarioBufferData101_PYRAMID_DOWN, 48, gScenarioKernelData101_PYRAMID_DOWN, 8},
{2, 30, 3032, 0, 1, gScenarioBufferData102_PYRAMID_DOWN, 48, gScenarioKernelData102_PYRAMID_DOWN, 8},
{2, 32, 3232, 0, 1, gScenarioBufferData103_PYRAMID_DOWN, 48, gScenarioKernelData103_PYRAMID_DOWN, 8}
};

static acf_scenario_list gScenarioList_PYRAMID_DOWN = {
104, //number of scenarios
gScenarioArray_PYRAMID_DOWN};
//**************************************************************

class PYRAMID_DOWN : public ACF_Process_APU
{

public:
   PYRAMID_DOWN(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("PYRAMID_DOWN");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("PYRAMID_DOWN",
                                        PYRAMID_DOWN_LOAD_SEGMENTS,
                                        PYRAMID_DOWN_LOAD_PMEM, PYRAMID_DOWN_LOAD_PMEM_SIZE,
                                        PYRAMID_DOWN_LOAD_DMEM, PYRAMID_DOWN_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(PYRAMID_DOWN_APEX_LOG_BUFFER, PYRAMID_DOWN_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_PYRAMID_DOWN);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_PYRAMID_DOWN

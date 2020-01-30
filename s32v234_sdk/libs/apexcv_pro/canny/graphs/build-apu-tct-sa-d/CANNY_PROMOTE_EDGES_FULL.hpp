#ifndef _ACF_PROCESS_APU_CANNY_PROMOTE_EDGES_FULL
#define _ACF_PROCESS_APU_CANNY_PROMOTE_EDGES_FULL

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CANNY_PROMOTE_EDGES_FULL_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CANNY_PROMOTE_EDGES_FULL[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 4, 1, {1, 1, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CANNY_PROMOTE_EDGES_FULL[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 4, 1, {1, 1, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CANNY_PROMOTE_EDGES_FULL[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 4, 1, {1, 1, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CANNY_PROMOTE_EDGES_FULL[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 4, 1, {1, 1, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CANNY_PROMOTE_EDGES_FULL[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 4, 1, {1, 1, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CANNY_PROMOTE_EDGES_FULL[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 4, 1, {1, 1, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CANNY_PROMOTE_EDGES_FULL[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 4, 1, {1, 1, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CANNY_PROMOTE_EDGES_FULL[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 4, 1, {1, 1, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CANNY_PROMOTE_EDGES_FULL[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 4, 1, {1, 1, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CANNY_PROMOTE_EDGES_FULL[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 4, 1, {1, 1, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CANNY_PROMOTE_EDGES_FULL[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 4, 1, {1, 1, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CANNY_PROMOTE_EDGES_FULL[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 4, 1, {1, 1, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CANNY_PROMOTE_EDGES_FULL[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 4, 1, {1, 1, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CANNY_PROMOTE_EDGES_FULL[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 4, 1, {1, 1, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CANNY_PROMOTE_EDGES_FULL[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 4, 1, {1, 1, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CANNY_PROMOTE_EDGES_FULL[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 4, 1, {1, 1, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CANNY_PROMOTE_EDGES_FULL[] = {{72, 1, 4, 0, {1, 1, 2, 2}}, {72, 1, 4, 1, {1, 1, 2, 2}}, {72, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CANNY_PROMOTE_EDGES_FULL[] = {{80, 1, 4, 0, {1, 1, 2, 2}}, {80, 1, 4, 1, {1, 1, 2, 2}}, {80, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CANNY_PROMOTE_EDGES_FULL[] = {{88, 1, 4, 0, {1, 1, 2, 2}}, {88, 1, 4, 1, {1, 1, 2, 2}}, {88, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CANNY_PROMOTE_EDGES_FULL[] = {{96, 1, 4, 0, {1, 1, 2, 2}}, {96, 1, 4, 1, {1, 1, 2, 2}}, {96, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CANNY_PROMOTE_EDGES_FULL[] = {{104, 1, 4, 0, {1, 1, 2, 2}}, {104, 1, 4, 1, {1, 1, 2, 2}}, {104, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CANNY_PROMOTE_EDGES_FULL[] = {{112, 1, 4, 0, {1, 1, 2, 2}}, {112, 1, 4, 1, {1, 1, 2, 2}}, {112, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CANNY_PROMOTE_EDGES_FULL[] = {{120, 1, 4, 0, {1, 1, 2, 2}}, {120, 1, 4, 1, {1, 1, 2, 2}}, {120, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CANNY_PROMOTE_EDGES_FULL[] = {{128, 1, 4, 0, {1, 1, 2, 2}}, {128, 1, 4, 1, {1, 1, 2, 2}}, {128, 1, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CANNY_PROMOTE_EDGES_FULL[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 4, 1, {1, 1, 2, 2}}, {4, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CANNY_PROMOTE_EDGES_FULL[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 4, 1, {1, 1, 2, 2}}, {8, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CANNY_PROMOTE_EDGES_FULL[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 4, 1, {1, 1, 2, 2}}, {12, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CANNY_PROMOTE_EDGES_FULL[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 4, 1, {1, 1, 2, 2}}, {16, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CANNY_PROMOTE_EDGES_FULL[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 4, 1, {1, 1, 2, 2}}, {20, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CANNY_PROMOTE_EDGES_FULL[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 4, 1, {1, 1, 2, 2}}, {24, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CANNY_PROMOTE_EDGES_FULL[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 4, 1, {1, 1, 2, 2}}, {28, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CANNY_PROMOTE_EDGES_FULL[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 4, 1, {1, 1, 2, 2}}, {32, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CANNY_PROMOTE_EDGES_FULL[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 4, 1, {1, 1, 2, 2}}, {36, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CANNY_PROMOTE_EDGES_FULL[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 4, 1, {1, 1, 2, 2}}, {40, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CANNY_PROMOTE_EDGES_FULL[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 4, 1, {1, 1, 2, 2}}, {44, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CANNY_PROMOTE_EDGES_FULL[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 4, 1, {1, 1, 2, 2}}, {48, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CANNY_PROMOTE_EDGES_FULL[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 4, 1, {1, 1, 2, 2}}, {52, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CANNY_PROMOTE_EDGES_FULL[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 4, 1, {1, 1, 2, 2}}, {56, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CANNY_PROMOTE_EDGES_FULL[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 4, 1, {1, 1, 2, 2}}, {60, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CANNY_PROMOTE_EDGES_FULL[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 4, 1, {1, 1, 2, 2}}, {64, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CANNY_PROMOTE_EDGES_FULL[] = {{72, 2, 4, 0, {1, 1, 2, 2}}, {72, 2, 4, 1, {1, 1, 2, 2}}, {72, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CANNY_PROMOTE_EDGES_FULL[] = {{80, 2, 4, 0, {1, 1, 2, 2}}, {80, 2, 4, 1, {1, 1, 2, 2}}, {80, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CANNY_PROMOTE_EDGES_FULL[] = {{88, 2, 4, 0, {1, 1, 2, 2}}, {88, 2, 4, 1, {1, 1, 2, 2}}, {88, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CANNY_PROMOTE_EDGES_FULL[] = {{96, 2, 4, 0, {1, 1, 2, 2}}, {96, 2, 4, 1, {1, 1, 2, 2}}, {96, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CANNY_PROMOTE_EDGES_FULL[] = {{104, 2, 4, 0, {1, 1, 2, 2}}, {104, 2, 4, 1, {1, 1, 2, 2}}, {104, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CANNY_PROMOTE_EDGES_FULL[] = {{112, 2, 4, 0, {1, 1, 2, 2}}, {112, 2, 4, 1, {1, 1, 2, 2}}, {112, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CANNY_PROMOTE_EDGES_FULL[] = {{120, 2, 4, 0, {1, 1, 2, 2}}, {120, 2, 4, 1, {1, 1, 2, 2}}, {120, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CANNY_PROMOTE_EDGES_FULL[] = {{128, 2, 4, 0, {1, 1, 2, 2}}, {128, 2, 4, 1, {1, 1, 2, 2}}, {128, 2, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CANNY_PROMOTE_EDGES_FULL[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 4, 1, {1, 1, 2, 2}}, {4, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CANNY_PROMOTE_EDGES_FULL[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 4, 1, {1, 1, 2, 2}}, {8, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CANNY_PROMOTE_EDGES_FULL[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 4, 1, {1, 1, 2, 2}}, {12, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CANNY_PROMOTE_EDGES_FULL[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 4, 1, {1, 1, 2, 2}}, {16, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CANNY_PROMOTE_EDGES_FULL[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 4, 1, {1, 1, 2, 2}}, {20, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CANNY_PROMOTE_EDGES_FULL[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 4, 1, {1, 1, 2, 2}}, {24, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CANNY_PROMOTE_EDGES_FULL[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 4, 1, {1, 1, 2, 2}}, {28, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CANNY_PROMOTE_EDGES_FULL[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 4, 1, {1, 1, 2, 2}}, {32, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CANNY_PROMOTE_EDGES_FULL[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 4, 1, {1, 1, 2, 2}}, {36, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CANNY_PROMOTE_EDGES_FULL[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 4, 1, {1, 1, 2, 2}}, {40, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CANNY_PROMOTE_EDGES_FULL[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 4, 1, {1, 1, 2, 2}}, {44, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CANNY_PROMOTE_EDGES_FULL[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 4, 1, {1, 1, 2, 2}}, {48, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CANNY_PROMOTE_EDGES_FULL[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 4, 1, {1, 1, 2, 2}}, {52, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CANNY_PROMOTE_EDGES_FULL[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 4, 1, {1, 1, 2, 2}}, {56, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CANNY_PROMOTE_EDGES_FULL[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 4, 1, {1, 1, 2, 2}}, {60, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CANNY_PROMOTE_EDGES_FULL[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 4, 1, {1, 1, 2, 2}}, {64, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CANNY_PROMOTE_EDGES_FULL[] = {{72, 3, 4, 0, {1, 1, 2, 2}}, {72, 3, 4, 1, {1, 1, 2, 2}}, {72, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CANNY_PROMOTE_EDGES_FULL[] = {{80, 3, 4, 0, {1, 1, 2, 2}}, {80, 3, 4, 1, {1, 1, 2, 2}}, {80, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CANNY_PROMOTE_EDGES_FULL[] = {{88, 3, 4, 0, {1, 1, 2, 2}}, {88, 3, 4, 1, {1, 1, 2, 2}}, {88, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CANNY_PROMOTE_EDGES_FULL[] = {{96, 3, 4, 0, {1, 1, 2, 2}}, {96, 3, 4, 1, {1, 1, 2, 2}}, {96, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CANNY_PROMOTE_EDGES_FULL[] = {{104, 3, 4, 0, {1, 1, 2, 2}}, {104, 3, 4, 1, {1, 1, 2, 2}}, {104, 3, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CANNY_PROMOTE_EDGES_FULL[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 4, 1, {1, 1, 2, 2}}, {4, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CANNY_PROMOTE_EDGES_FULL[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 4, 1, {1, 1, 2, 2}}, {8, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CANNY_PROMOTE_EDGES_FULL[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 4, 1, {1, 1, 2, 2}}, {12, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CANNY_PROMOTE_EDGES_FULL[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 4, 1, {1, 1, 2, 2}}, {16, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CANNY_PROMOTE_EDGES_FULL[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 4, 1, {1, 1, 2, 2}}, {20, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CANNY_PROMOTE_EDGES_FULL[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 4, 1, {1, 1, 2, 2}}, {24, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CANNY_PROMOTE_EDGES_FULL[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 4, 1, {1, 1, 2, 2}}, {28, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CANNY_PROMOTE_EDGES_FULL[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 4, 1, {1, 1, 2, 2}}, {32, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CANNY_PROMOTE_EDGES_FULL[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 4, 1, {1, 1, 2, 2}}, {36, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CANNY_PROMOTE_EDGES_FULL[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 4, 1, {1, 1, 2, 2}}, {40, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CANNY_PROMOTE_EDGES_FULL[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 4, 1, {1, 1, 2, 2}}, {44, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CANNY_PROMOTE_EDGES_FULL[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 4, 1, {1, 1, 2, 2}}, {48, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CANNY_PROMOTE_EDGES_FULL[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 4, 1, {1, 1, 2, 2}}, {52, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CANNY_PROMOTE_EDGES_FULL[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 4, 1, {1, 1, 2, 2}}, {56, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CANNY_PROMOTE_EDGES_FULL[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 4, 1, {1, 1, 2, 2}}, {60, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CANNY_PROMOTE_EDGES_FULL[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 4, 1, {1, 1, 2, 2}}, {64, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CANNY_PROMOTE_EDGES_FULL[] = {{72, 4, 4, 0, {1, 1, 2, 2}}, {72, 4, 4, 1, {1, 1, 2, 2}}, {72, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CANNY_PROMOTE_EDGES_FULL[] = {{80, 4, 4, 0, {1, 1, 2, 2}}, {80, 4, 4, 1, {1, 1, 2, 2}}, {80, 4, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CANNY_PROMOTE_EDGES_FULL[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 4, 1, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CANNY_PROMOTE_EDGES_FULL[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 4, 1, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CANNY_PROMOTE_EDGES_FULL[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 4, 1, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CANNY_PROMOTE_EDGES_FULL[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 4, 1, {1, 1, 2, 2}}, {16, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CANNY_PROMOTE_EDGES_FULL[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 4, 1, {1, 1, 2, 2}}, {20, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CANNY_PROMOTE_EDGES_FULL[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 4, 1, {1, 1, 2, 2}}, {24, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CANNY_PROMOTE_EDGES_FULL[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 4, 1, {1, 1, 2, 2}}, {28, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CANNY_PROMOTE_EDGES_FULL[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 4, 1, {1, 1, 2, 2}}, {32, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CANNY_PROMOTE_EDGES_FULL[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 4, 1, {1, 1, 2, 2}}, {36, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CANNY_PROMOTE_EDGES_FULL[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 4, 1, {1, 1, 2, 2}}, {40, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CANNY_PROMOTE_EDGES_FULL[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 4, 1, {1, 1, 2, 2}}, {44, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CANNY_PROMOTE_EDGES_FULL[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 4, 1, {1, 1, 2, 2}}, {48, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CANNY_PROMOTE_EDGES_FULL[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 4, 1, {1, 1, 2, 2}}, {52, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CANNY_PROMOTE_EDGES_FULL[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 4, 1, {1, 1, 2, 2}}, {56, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CANNY_PROMOTE_EDGES_FULL[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 4, 1, {1, 1, 2, 2}}, {60, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CANNY_PROMOTE_EDGES_FULL[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 4, 1, {1, 1, 2, 2}}, {64, 5, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CANNY_PROMOTE_EDGES_FULL[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 4, 1, {1, 1, 2, 2}}, {4, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_CANNY_PROMOTE_EDGES_FULL[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 4, 1, {1, 1, 2, 2}}, {8, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_CANNY_PROMOTE_EDGES_FULL[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 4, 1, {1, 1, 2, 2}}, {12, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_CANNY_PROMOTE_EDGES_FULL[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 4, 1, {1, 1, 2, 2}}, {16, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_CANNY_PROMOTE_EDGES_FULL[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 4, 1, {1, 1, 2, 2}}, {20, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_CANNY_PROMOTE_EDGES_FULL[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 4, 1, {1, 1, 2, 2}}, {24, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_CANNY_PROMOTE_EDGES_FULL[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 4, 1, {1, 1, 2, 2}}, {28, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_CANNY_PROMOTE_EDGES_FULL[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 4, 1, {1, 1, 2, 2}}, {32, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_CANNY_PROMOTE_EDGES_FULL[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 4, 1, {1, 1, 2, 2}}, {36, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_CANNY_PROMOTE_EDGES_FULL[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 4, 1, {1, 1, 2, 2}}, {40, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_CANNY_PROMOTE_EDGES_FULL[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 4, 1, {1, 1, 2, 2}}, {44, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_CANNY_PROMOTE_EDGES_FULL[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 4, 1, {1, 1, 2, 2}}, {48, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_CANNY_PROMOTE_EDGES_FULL[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 4, 1, {1, 1, 2, 2}}, {52, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_CANNY_PROMOTE_EDGES_FULL[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 4, 1, {1, 1, 2, 2}}, {56, 6, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_CANNY_PROMOTE_EDGES_FULL[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 4, 1, {1, 1, 2, 2}}, {4, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_CANNY_PROMOTE_EDGES_FULL[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 4, 1, {1, 1, 2, 2}}, {8, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_CANNY_PROMOTE_EDGES_FULL[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 4, 1, {1, 1, 2, 2}}, {12, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_CANNY_PROMOTE_EDGES_FULL[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 4, 1, {1, 1, 2, 2}}, {16, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_CANNY_PROMOTE_EDGES_FULL[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 4, 1, {1, 1, 2, 2}}, {20, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_CANNY_PROMOTE_EDGES_FULL[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 4, 1, {1, 1, 2, 2}}, {24, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_CANNY_PROMOTE_EDGES_FULL[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 4, 1, {1, 1, 2, 2}}, {28, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_CANNY_PROMOTE_EDGES_FULL[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 4, 1, {1, 1, 2, 2}}, {32, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_CANNY_PROMOTE_EDGES_FULL[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 4, 1, {1, 1, 2, 2}}, {36, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_CANNY_PROMOTE_EDGES_FULL[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 4, 1, {1, 1, 2, 2}}, {40, 8, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_CANNY_PROMOTE_EDGES_FULL[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 4, 1, {1, 1, 2, 2}}, {4, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_CANNY_PROMOTE_EDGES_FULL[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 4, 1, {1, 1, 2, 2}}, {8, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_CANNY_PROMOTE_EDGES_FULL[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 4, 1, {1, 1, 2, 2}}, {12, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_CANNY_PROMOTE_EDGES_FULL[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 4, 1, {1, 1, 2, 2}}, {16, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_CANNY_PROMOTE_EDGES_FULL[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 4, 1, {1, 1, 2, 2}}, {20, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_CANNY_PROMOTE_EDGES_FULL[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 4, 1, {1, 1, 2, 2}}, {24, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_CANNY_PROMOTE_EDGES_FULL[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 4, 1, {1, 1, 2, 2}}, {28, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_CANNY_PROMOTE_EDGES_FULL[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 4, 1, {1, 1, 2, 2}}, {32, 10, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_CANNY_PROMOTE_EDGES_FULL[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 4, 1, {1, 1, 2, 2}}, {4, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_CANNY_PROMOTE_EDGES_FULL[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 4, 1, {1, 1, 2, 2}}, {8, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_CANNY_PROMOTE_EDGES_FULL[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 4, 1, {1, 1, 2, 2}}, {12, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_CANNY_PROMOTE_EDGES_FULL[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 4, 1, {1, 1, 2, 2}}, {16, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_CANNY_PROMOTE_EDGES_FULL[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 4, 1, {1, 1, 2, 2}}, {20, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_CANNY_PROMOTE_EDGES_FULL[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 4, 1, {1, 1, 2, 2}}, {24, 12, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_CANNY_PROMOTE_EDGES_FULL[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 4, 1, {1, 1, 2, 2}}, {4, 14, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_CANNY_PROMOTE_EDGES_FULL[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 4, 1, {1, 1, 2, 2}}, {8, 14, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_CANNY_PROMOTE_EDGES_FULL[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 4, 1, {1, 1, 2, 2}}, {12, 14, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_CANNY_PROMOTE_EDGES_FULL[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 4, 1, {1, 1, 2, 2}}, {16, 14, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_CANNY_PROMOTE_EDGES_FULL[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 4, 1, {1, 1, 2, 2}}, {20, 14, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_CANNY_PROMOTE_EDGES_FULL[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 4, 1, {1, 1, 2, 2}}, {4, 16, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_CANNY_PROMOTE_EDGES_FULL[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 4, 1, {1, 1, 2, 2}}, {8, 16, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_CANNY_PROMOTE_EDGES_FULL[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 4, 1, {1, 1, 2, 2}}, {12, 16, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_CANNY_PROMOTE_EDGES_FULL[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 4, 1, {1, 1, 2, 2}}, {16, 16, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_CANNY_PROMOTE_EDGES_FULL[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 4, 1, {1, 1, 2, 2}}, {20, 16, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_CANNY_PROMOTE_EDGES_FULL[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 4, 1, {1, 1, 2, 2}}, {4, 18, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_CANNY_PROMOTE_EDGES_FULL[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 4, 1, {1, 1, 2, 2}}, {8, 18, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_CANNY_PROMOTE_EDGES_FULL[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 4, 1, {1, 1, 2, 2}}, {12, 18, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_CANNY_PROMOTE_EDGES_FULL[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 4, 1, {1, 1, 2, 2}}, {16, 18, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_CANNY_PROMOTE_EDGES_FULL[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 4, 1, {1, 1, 2, 2}}, {4, 20, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_CANNY_PROMOTE_EDGES_FULL[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 4, 1, {1, 1, 2, 2}}, {8, 20, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_CANNY_PROMOTE_EDGES_FULL[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 4, 1, {1, 1, 2, 2}}, {12, 20, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_CANNY_PROMOTE_EDGES_FULL[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 4, 1, {1, 1, 2, 2}}, {4, 22, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_CANNY_PROMOTE_EDGES_FULL[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 4, 1, {1, 1, 2, 2}}, {8, 22, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_CANNY_PROMOTE_EDGES_FULL[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 4, 1, {1, 1, 2, 2}}, {12, 22, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_CANNY_PROMOTE_EDGES_FULL[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 4, 1, {1, 1, 2, 2}}, {4, 24, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_CANNY_PROMOTE_EDGES_FULL[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 4, 1, {1, 1, 2, 2}}, {8, 24, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_CANNY_PROMOTE_EDGES_FULL[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 4, 1, {1, 1, 2, 2}}, {12, 24, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_CANNY_PROMOTE_EDGES_FULL[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 4, 1, {1, 1, 2, 2}}, {4, 26, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_CANNY_PROMOTE_EDGES_FULL[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 4, 1, {1, 1, 2, 2}}, {8, 26, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_CANNY_PROMOTE_EDGES_FULL[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 4, 1, {1, 1, 2, 2}}, {4, 28, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_CANNY_PROMOTE_EDGES_FULL[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 4, 1, {1, 1, 2, 2}}, {8, 28, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_CANNY_PROMOTE_EDGES_FULL[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 4, 1, {1, 1, 2, 2}}, {4, 30, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_CANNY_PROMOTE_EDGES_FULL[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 4, 1, {1, 1, 2, 2}}, {8, 30, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_CANNY_PROMOTE_EDGES_FULL[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 4, 1, {1, 1, 2, 2}}, {4, 32, 2, 2, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_CANNY_PROMOTE_EDGES_FULL[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 4, 1, {1, 1, 2, 2}}, {8, 32, 2, 2, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_CANNY_PROMOTE_EDGES_FULL[] = {{1, 0}, {2, 0}};

static acf_scenario gScenarioArray_CANNY_PROMOTE_EDGES_FULL[] = {
{1, 1, 104, 0, 2, gScenarioBufferData0_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData0_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 1, 160, 0, 2, gScenarioBufferData1_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData1_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 1, 216, 0, 2, gScenarioBufferData2_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData2_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 1, 272, 0, 2, gScenarioBufferData3_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData3_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 1, 328, 0, 2, gScenarioBufferData4_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData4_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 1, 384, 0, 2, gScenarioBufferData5_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData5_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 1, 440, 0, 2, gScenarioBufferData6_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData6_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 1, 496, 0, 2, gScenarioBufferData7_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData7_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 1, 552, 0, 2, gScenarioBufferData8_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData8_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 1, 608, 0, 2, gScenarioBufferData9_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData9_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 1, 664, 0, 2, gScenarioBufferData10_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData10_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 1, 720, 0, 2, gScenarioBufferData11_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData11_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 1, 776, 0, 2, gScenarioBufferData12_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData12_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 1, 832, 0, 2, gScenarioBufferData13_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData13_CANNY_PROMOTE_EDGES_FULL, 8},
{15, 1, 888, 0, 2, gScenarioBufferData14_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData14_CANNY_PROMOTE_EDGES_FULL, 8},
{16, 1, 944, 0, 2, gScenarioBufferData15_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData15_CANNY_PROMOTE_EDGES_FULL, 8},
{18, 1, 1056, 0, 2, gScenarioBufferData16_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData16_CANNY_PROMOTE_EDGES_FULL, 8},
{20, 1, 1168, 0, 2, gScenarioBufferData17_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData17_CANNY_PROMOTE_EDGES_FULL, 8},
{22, 1, 1280, 0, 2, gScenarioBufferData18_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData18_CANNY_PROMOTE_EDGES_FULL, 8},
{24, 1, 1392, 0, 2, gScenarioBufferData19_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData19_CANNY_PROMOTE_EDGES_FULL, 8},
{26, 1, 1504, 0, 2, gScenarioBufferData20_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData20_CANNY_PROMOTE_EDGES_FULL, 8},
{28, 1, 1616, 0, 2, gScenarioBufferData21_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData21_CANNY_PROMOTE_EDGES_FULL, 8},
{30, 1, 1728, 0, 2, gScenarioBufferData22_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData22_CANNY_PROMOTE_EDGES_FULL, 8},
{32, 1, 1840, 0, 2, gScenarioBufferData23_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData23_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 2, 176, 0, 2, gScenarioBufferData24_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData24_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 2, 272, 0, 2, gScenarioBufferData25_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData25_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 2, 368, 0, 2, gScenarioBufferData26_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData26_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 2, 464, 0, 2, gScenarioBufferData27_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData27_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 2, 560, 0, 2, gScenarioBufferData28_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData28_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 2, 656, 0, 2, gScenarioBufferData29_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData29_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 2, 752, 0, 2, gScenarioBufferData30_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData30_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 2, 848, 0, 2, gScenarioBufferData31_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData31_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 2, 944, 0, 2, gScenarioBufferData32_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData32_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 2, 1040, 0, 2, gScenarioBufferData33_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData33_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 2, 1136, 0, 2, gScenarioBufferData34_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData34_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 2, 1232, 0, 2, gScenarioBufferData35_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData35_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 2, 1328, 0, 2, gScenarioBufferData36_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData36_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 2, 1424, 0, 2, gScenarioBufferData37_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData37_CANNY_PROMOTE_EDGES_FULL, 8},
{15, 2, 1520, 0, 2, gScenarioBufferData38_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData38_CANNY_PROMOTE_EDGES_FULL, 8},
{16, 2, 1616, 0, 2, gScenarioBufferData39_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData39_CANNY_PROMOTE_EDGES_FULL, 8},
{18, 2, 1808, 0, 2, gScenarioBufferData40_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData40_CANNY_PROMOTE_EDGES_FULL, 8},
{20, 2, 2000, 0, 2, gScenarioBufferData41_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData41_CANNY_PROMOTE_EDGES_FULL, 8},
{22, 2, 2192, 0, 2, gScenarioBufferData42_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData42_CANNY_PROMOTE_EDGES_FULL, 8},
{24, 2, 2384, 0, 2, gScenarioBufferData43_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData43_CANNY_PROMOTE_EDGES_FULL, 8},
{26, 2, 2576, 0, 2, gScenarioBufferData44_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData44_CANNY_PROMOTE_EDGES_FULL, 8},
{28, 2, 2768, 0, 2, gScenarioBufferData45_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData45_CANNY_PROMOTE_EDGES_FULL, 8},
{30, 2, 2960, 0, 2, gScenarioBufferData46_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData46_CANNY_PROMOTE_EDGES_FULL, 8},
{32, 2, 3152, 0, 2, gScenarioBufferData47_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData47_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 3, 248, 0, 2, gScenarioBufferData48_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData48_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 3, 384, 0, 2, gScenarioBufferData49_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData49_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 3, 520, 0, 2, gScenarioBufferData50_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData50_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 3, 656, 0, 2, gScenarioBufferData51_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData51_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 3, 792, 0, 2, gScenarioBufferData52_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData52_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 3, 928, 0, 2, gScenarioBufferData53_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData53_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 3, 1064, 0, 2, gScenarioBufferData54_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData54_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 3, 1200, 0, 2, gScenarioBufferData55_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData55_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 3, 1336, 0, 2, gScenarioBufferData56_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData56_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 3, 1472, 0, 2, gScenarioBufferData57_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData57_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 3, 1608, 0, 2, gScenarioBufferData58_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData58_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 3, 1744, 0, 2, gScenarioBufferData59_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData59_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 3, 1880, 0, 2, gScenarioBufferData60_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData60_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 3, 2016, 0, 2, gScenarioBufferData61_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData61_CANNY_PROMOTE_EDGES_FULL, 8},
{15, 3, 2152, 0, 2, gScenarioBufferData62_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData62_CANNY_PROMOTE_EDGES_FULL, 8},
{16, 3, 2288, 0, 2, gScenarioBufferData63_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData63_CANNY_PROMOTE_EDGES_FULL, 8},
{18, 3, 2560, 0, 2, gScenarioBufferData64_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData64_CANNY_PROMOTE_EDGES_FULL, 8},
{20, 3, 2832, 0, 2, gScenarioBufferData65_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData65_CANNY_PROMOTE_EDGES_FULL, 8},
{22, 3, 3104, 0, 2, gScenarioBufferData66_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData66_CANNY_PROMOTE_EDGES_FULL, 8},
{24, 3, 3376, 0, 2, gScenarioBufferData67_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData67_CANNY_PROMOTE_EDGES_FULL, 8},
{26, 3, 3648, 0, 2, gScenarioBufferData68_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData68_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 4, 320, 0, 2, gScenarioBufferData69_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData69_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 4, 496, 0, 2, gScenarioBufferData70_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData70_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 4, 672, 0, 2, gScenarioBufferData71_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData71_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 4, 848, 0, 2, gScenarioBufferData72_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData72_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 4, 1024, 0, 2, gScenarioBufferData73_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData73_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 4, 1200, 0, 2, gScenarioBufferData74_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData74_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 4, 1376, 0, 2, gScenarioBufferData75_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData75_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 4, 1552, 0, 2, gScenarioBufferData76_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData76_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 4, 1728, 0, 2, gScenarioBufferData77_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData77_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 4, 1904, 0, 2, gScenarioBufferData78_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData78_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 4, 2080, 0, 2, gScenarioBufferData79_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData79_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 4, 2256, 0, 2, gScenarioBufferData80_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData80_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 4, 2432, 0, 2, gScenarioBufferData81_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData81_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 4, 2608, 0, 2, gScenarioBufferData82_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData82_CANNY_PROMOTE_EDGES_FULL, 8},
{15, 4, 2784, 0, 2, gScenarioBufferData83_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData83_CANNY_PROMOTE_EDGES_FULL, 8},
{16, 4, 2960, 0, 2, gScenarioBufferData84_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData84_CANNY_PROMOTE_EDGES_FULL, 8},
{18, 4, 3312, 0, 2, gScenarioBufferData85_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData85_CANNY_PROMOTE_EDGES_FULL, 8},
{20, 4, 3664, 0, 2, gScenarioBufferData86_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData86_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 5, 392, 0, 2, gScenarioBufferData87_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData87_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 5, 608, 0, 2, gScenarioBufferData88_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData88_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 5, 824, 0, 2, gScenarioBufferData89_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData89_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 5, 1040, 0, 2, gScenarioBufferData90_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData90_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 5, 1256, 0, 2, gScenarioBufferData91_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData91_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 5, 1472, 0, 2, gScenarioBufferData92_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData92_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 5, 1688, 0, 2, gScenarioBufferData93_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData93_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 5, 1904, 0, 2, gScenarioBufferData94_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData94_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 5, 2120, 0, 2, gScenarioBufferData95_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData95_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 5, 2336, 0, 2, gScenarioBufferData96_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData96_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 5, 2552, 0, 2, gScenarioBufferData97_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData97_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 5, 2768, 0, 2, gScenarioBufferData98_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData98_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 5, 2984, 0, 2, gScenarioBufferData99_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData99_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 5, 3200, 0, 2, gScenarioBufferData100_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData100_CANNY_PROMOTE_EDGES_FULL, 8},
{15, 5, 3416, 0, 2, gScenarioBufferData101_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData101_CANNY_PROMOTE_EDGES_FULL, 8},
{16, 5, 3632, 0, 2, gScenarioBufferData102_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData102_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 6, 464, 0, 2, gScenarioBufferData103_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData103_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 6, 720, 0, 2, gScenarioBufferData104_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData104_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 6, 976, 0, 2, gScenarioBufferData105_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData105_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 6, 1232, 0, 2, gScenarioBufferData106_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData106_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 6, 1488, 0, 2, gScenarioBufferData107_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData107_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 6, 1744, 0, 2, gScenarioBufferData108_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData108_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 6, 2000, 0, 2, gScenarioBufferData109_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData109_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 6, 2256, 0, 2, gScenarioBufferData110_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData110_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 6, 2512, 0, 2, gScenarioBufferData111_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData111_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 6, 2768, 0, 2, gScenarioBufferData112_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData112_CANNY_PROMOTE_EDGES_FULL, 8},
{11, 6, 3024, 0, 2, gScenarioBufferData113_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData113_CANNY_PROMOTE_EDGES_FULL, 8},
{12, 6, 3280, 0, 2, gScenarioBufferData114_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData114_CANNY_PROMOTE_EDGES_FULL, 8},
{13, 6, 3536, 0, 2, gScenarioBufferData115_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData115_CANNY_PROMOTE_EDGES_FULL, 8},
{14, 6, 3792, 0, 2, gScenarioBufferData116_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData116_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 8, 608, 0, 2, gScenarioBufferData117_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData117_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 8, 944, 0, 2, gScenarioBufferData118_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData118_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 8, 1280, 0, 2, gScenarioBufferData119_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData119_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 8, 1616, 0, 2, gScenarioBufferData120_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData120_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 8, 1952, 0, 2, gScenarioBufferData121_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData121_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 8, 2288, 0, 2, gScenarioBufferData122_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData122_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 8, 2624, 0, 2, gScenarioBufferData123_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData123_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 8, 2960, 0, 2, gScenarioBufferData124_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData124_CANNY_PROMOTE_EDGES_FULL, 8},
{9, 8, 3296, 0, 2, gScenarioBufferData125_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData125_CANNY_PROMOTE_EDGES_FULL, 8},
{10, 8, 3632, 0, 2, gScenarioBufferData126_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData126_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 10, 752, 0, 2, gScenarioBufferData127_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData127_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 10, 1168, 0, 2, gScenarioBufferData128_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData128_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 10, 1584, 0, 2, gScenarioBufferData129_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData129_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 10, 2000, 0, 2, gScenarioBufferData130_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData130_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 10, 2416, 0, 2, gScenarioBufferData131_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData131_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 10, 2832, 0, 2, gScenarioBufferData132_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData132_CANNY_PROMOTE_EDGES_FULL, 8},
{7, 10, 3248, 0, 2, gScenarioBufferData133_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData133_CANNY_PROMOTE_EDGES_FULL, 8},
{8, 10, 3664, 0, 2, gScenarioBufferData134_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData134_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 12, 896, 0, 2, gScenarioBufferData135_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData135_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 12, 1392, 0, 2, gScenarioBufferData136_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData136_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 12, 1888, 0, 2, gScenarioBufferData137_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData137_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 12, 2384, 0, 2, gScenarioBufferData138_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData138_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 12, 2880, 0, 2, gScenarioBufferData139_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData139_CANNY_PROMOTE_EDGES_FULL, 8},
{6, 12, 3376, 0, 2, gScenarioBufferData140_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData140_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 14, 1040, 0, 2, gScenarioBufferData141_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData141_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 14, 1616, 0, 2, gScenarioBufferData142_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData142_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 14, 2192, 0, 2, gScenarioBufferData143_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData143_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 14, 2768, 0, 2, gScenarioBufferData144_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData144_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 14, 3344, 0, 2, gScenarioBufferData145_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData145_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 16, 1184, 0, 2, gScenarioBufferData146_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData146_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 16, 1840, 0, 2, gScenarioBufferData147_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData147_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 16, 2496, 0, 2, gScenarioBufferData148_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData148_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 16, 3152, 0, 2, gScenarioBufferData149_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData149_CANNY_PROMOTE_EDGES_FULL, 8},
{5, 16, 3808, 0, 2, gScenarioBufferData150_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData150_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 18, 1328, 0, 2, gScenarioBufferData151_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData151_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 18, 2064, 0, 2, gScenarioBufferData152_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData152_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 18, 2800, 0, 2, gScenarioBufferData153_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData153_CANNY_PROMOTE_EDGES_FULL, 8},
{4, 18, 3536, 0, 2, gScenarioBufferData154_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData154_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 20, 1472, 0, 2, gScenarioBufferData155_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData155_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 20, 2288, 0, 2, gScenarioBufferData156_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData156_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 20, 3104, 0, 2, gScenarioBufferData157_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData157_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 22, 1616, 0, 2, gScenarioBufferData158_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData158_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 22, 2512, 0, 2, gScenarioBufferData159_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData159_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 22, 3408, 0, 2, gScenarioBufferData160_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData160_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 24, 1760, 0, 2, gScenarioBufferData161_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData161_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 24, 2736, 0, 2, gScenarioBufferData162_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData162_CANNY_PROMOTE_EDGES_FULL, 8},
{3, 24, 3712, 0, 2, gScenarioBufferData163_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData163_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 26, 1904, 0, 2, gScenarioBufferData164_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData164_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 26, 2960, 0, 2, gScenarioBufferData165_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData165_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 28, 2048, 0, 2, gScenarioBufferData166_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData166_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 28, 3184, 0, 2, gScenarioBufferData167_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData167_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 30, 2192, 0, 2, gScenarioBufferData168_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData168_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 30, 3408, 0, 2, gScenarioBufferData169_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData169_CANNY_PROMOTE_EDGES_FULL, 8},
{1, 32, 2336, 0, 2, gScenarioBufferData170_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData170_CANNY_PROMOTE_EDGES_FULL, 8},
{2, 32, 3632, 0, 2, gScenarioBufferData171_CANNY_PROMOTE_EDGES_FULL, 48, gScenarioKernelData171_CANNY_PROMOTE_EDGES_FULL, 8}
};

static acf_scenario_list gScenarioList_CANNY_PROMOTE_EDGES_FULL = {
172, //number of scenarios
gScenarioArray_CANNY_PROMOTE_EDGES_FULL};
//**************************************************************

class CANNY_PROMOTE_EDGES_FULL : public ACF_Process_APU
{

public:
   CANNY_PROMOTE_EDGES_FULL(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CANNY_PROMOTE_EDGES_FULL");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CANNY_PROMOTE_EDGES_FULL",
                                        CANNY_PROMOTE_EDGES_FULL_LOAD_SEGMENTS,
                                        CANNY_PROMOTE_EDGES_FULL_LOAD_PMEM, CANNY_PROMOTE_EDGES_FULL_LOAD_PMEM_SIZE,
                                        CANNY_PROMOTE_EDGES_FULL_LOAD_DMEM, CANNY_PROMOTE_EDGES_FULL_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CANNY_PROMOTE_EDGES_FULL_APEX_LOG_BUFFER, CANNY_PROMOTE_EDGES_FULL_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CANNY_PROMOTE_EDGES_FULL);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CANNY_PROMOTE_EDGES_FULL

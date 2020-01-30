#ifndef _ACF_PROCESS_APU_CANNY_NON_MAXIMA_SUPPRESS
#define _ACF_PROCESS_APU_CANNY_NON_MAXIMA_SUPPRESS

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CANNY_NON_MAXIMA_SUPPRESS_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 1, 7, 0, {2, 2, 2, 2}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 1, 7, 0, {2, 2, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 1, 7, 0, {2, 2, 2, 2}}, {6, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 1, 7, 0, {2, 2, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 1, 7, 0, {2, 2, 2, 2}}, {10, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 1, 7, 0, {2, 2, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 1, 7, 0, {2, 2, 2, 2}}, {14, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 1, 7, 0, {2, 2, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 1, 7, 0, {2, 2, 2, 2}}, {18, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 1, 7, 0, {2, 2, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 1, 7, 0, {2, 2, 2, 2}}, {22, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 1, 7, 0, {2, 2, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 1, 7, 0, {2, 2, 2, 2}}, {26, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 1, 7, 0, {2, 2, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 1, 7, 0, {2, 2, 2, 2}}, {30, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 1, 7, 0, {2, 2, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 1, 7, 0, {2, 2, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 1, 7, 0, {2, 2, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 1, 7, 0, {2, 2, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 1, 7, 0, {2, 2, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 1, 7, 0, {2, 2, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 1, 7, 0, {2, 2, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 1, 7, 0, {2, 2, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 1, 7, 0, {2, 2, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 1, 7, 0, {2, 2, 2, 2}}, {72, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 1, 7, 0, {2, 2, 2, 2}}, {80, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 1, 7, 0, {2, 2, 2, 2}}, {88, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CANNY_NON_MAXIMA_SUPPRESS[] = {{96, 1, 7, 0, {2, 2, 2, 2}}, {96, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CANNY_NON_MAXIMA_SUPPRESS[] = {{104, 1, 7, 0, {2, 2, 2, 2}}, {104, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CANNY_NON_MAXIMA_SUPPRESS[] = {{112, 1, 7, 0, {2, 2, 2, 2}}, {112, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CANNY_NON_MAXIMA_SUPPRESS[] = {{120, 1, 7, 0, {2, 2, 2, 2}}, {120, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CANNY_NON_MAXIMA_SUPPRESS[] = {{128, 1, 7, 0, {2, 2, 2, 2}}, {128, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 2, 4, 0, {2, 2, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 2, 4, 0, {2, 2, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 2, 4, 0, {2, 2, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 2, 4, 0, {2, 2, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 2, 4, 0, {2, 2, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 2, 4, 0, {2, 2, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 2, 4, 0, {2, 2, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 2, 4, 0, {2, 2, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 2, 4, 0, {2, 2, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 2, 4, 0, {2, 2, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 2, 4, 0, {2, 2, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CANNY_NON_MAXIMA_SUPPRESS[] = {{96, 2, 4, 0, {2, 2, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CANNY_NON_MAXIMA_SUPPRESS[] = {{104, 2, 4, 0, {2, 2, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CANNY_NON_MAXIMA_SUPPRESS[] = {{112, 2, 4, 0, {2, 2, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CANNY_NON_MAXIMA_SUPPRESS[] = {{120, 2, 4, 0, {2, 2, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CANNY_NON_MAXIMA_SUPPRESS[] = {{128, 2, 4, 0, {2, 2, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 3, 4, 0, {2, 2, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 3, 4, 0, {2, 2, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 3, 4, 0, {2, 2, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 3, 4, 0, {2, 2, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 3, 4, 0, {2, 2, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 3, 4, 0, {2, 2, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 3, 4, 0, {2, 2, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 3, 4, 0, {2, 2, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 3, 4, 0, {2, 2, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 3, 4, 0, {2, 2, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 3, 4, 0, {2, 2, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 3, 4, 0, {2, 2, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 3, 4, 0, {2, 2, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 3, 4, 0, {2, 2, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 3, 4, 0, {2, 2, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 3, 4, 0, {2, 2, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 3, 4, 0, {2, 2, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 3, 4, 0, {2, 2, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 3, 4, 0, {2, 2, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 3, 4, 0, {2, 2, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 3, 4, 0, {2, 2, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 3, 4, 0, {2, 2, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 3, 4, 0, {2, 2, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 3, 4, 0, {2, 2, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 3, 4, 0, {2, 2, 2, 2}}, {72, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 3, 4, 0, {2, 2, 2, 2}}, {80, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 3, 4, 0, {2, 2, 2, 2}}, {88, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CANNY_NON_MAXIMA_SUPPRESS[] = {{96, 3, 4, 0, {2, 2, 2, 2}}, {96, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CANNY_NON_MAXIMA_SUPPRESS[] = {{104, 3, 4, 0, {2, 2, 2, 2}}, {104, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CANNY_NON_MAXIMA_SUPPRESS[] = {{112, 3, 4, 0, {2, 2, 2, 2}}, {112, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CANNY_NON_MAXIMA_SUPPRESS[] = {{120, 3, 4, 0, {2, 2, 2, 2}}, {120, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CANNY_NON_MAXIMA_SUPPRESS[] = {{128, 3, 4, 0, {2, 2, 2, 2}}, {128, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 4, 4, 0, {2, 2, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 4, 4, 0, {2, 2, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 4, 4, 0, {2, 2, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 4, 4, 0, {2, 2, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 4, 4, 0, {2, 2, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 4, 4, 0, {2, 2, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 4, 4, 0, {2, 2, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 4, 4, 0, {2, 2, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 4, 4, 0, {2, 2, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 4, 4, 0, {2, 2, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 4, 4, 0, {2, 2, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_CANNY_NON_MAXIMA_SUPPRESS[] = {{96, 4, 4, 0, {2, 2, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_CANNY_NON_MAXIMA_SUPPRESS[] = {{104, 4, 4, 0, {2, 2, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_CANNY_NON_MAXIMA_SUPPRESS[] = {{112, 4, 4, 0, {2, 2, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_CANNY_NON_MAXIMA_SUPPRESS[] = {{120, 4, 4, 0, {2, 2, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_CANNY_NON_MAXIMA_SUPPRESS[] = {{128, 4, 4, 0, {2, 2, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 5, 4, 0, {2, 2, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 5, 4, 0, {2, 2, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 5, 4, 0, {2, 2, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 5, 4, 0, {2, 2, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 5, 4, 0, {2, 2, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 5, 4, 0, {2, 2, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 5, 4, 0, {2, 2, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 5, 4, 0, {2, 2, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 5, 4, 0, {2, 2, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 5, 4, 0, {2, 2, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 5, 4, 0, {2, 2, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 5, 4, 0, {2, 2, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 5, 4, 0, {2, 2, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 5, 4, 0, {2, 2, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 5, 4, 0, {2, 2, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 5, 4, 0, {2, 2, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 5, 4, 0, {2, 2, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 5, 4, 0, {2, 2, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 5, 4, 0, {2, 2, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 5, 4, 0, {2, 2, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 5, 4, 0, {2, 2, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 5, 4, 0, {2, 2, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 5, 4, 0, {2, 2, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 5, 4, 0, {2, 2, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 5, 4, 0, {2, 2, 2, 2}}, {72, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 5, 4, 0, {2, 2, 2, 2}}, {80, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 5, 4, 0, {2, 2, 2, 2}}, {88, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_CANNY_NON_MAXIMA_SUPPRESS[] = {{96, 5, 4, 0, {2, 2, 2, 2}}, {96, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_CANNY_NON_MAXIMA_SUPPRESS[] = {{104, 5, 4, 0, {2, 2, 2, 2}}, {104, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 6, 4, 0, {2, 2, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 6, 4, 0, {2, 2, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 6, 4, 0, {2, 2, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 6, 4, 0, {2, 2, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 6, 4, 0, {2, 2, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 6, 4, 0, {2, 2, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 6, 4, 0, {2, 2, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 6, 4, 0, {2, 2, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 6, 4, 0, {2, 2, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 6, 4, 0, {2, 2, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 6, 4, 0, {2, 2, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 6, 4, 0, {2, 2, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 6, 4, 0, {2, 2, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_CANNY_NON_MAXIMA_SUPPRESS[] = {{72, 6, 4, 0, {2, 2, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_CANNY_NON_MAXIMA_SUPPRESS[] = {{80, 6, 4, 0, {2, 2, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_CANNY_NON_MAXIMA_SUPPRESS[] = {{88, 6, 4, 0, {2, 2, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 8, 4, 0, {2, 2, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 8, 4, 0, {2, 2, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 8, 4, 0, {2, 2, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 8, 4, 0, {2, 2, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 8, 4, 0, {2, 2, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 8, 4, 0, {2, 2, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 8, 4, 0, {2, 2, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 8, 4, 0, {2, 2, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 8, 4, 0, {2, 2, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 8, 4, 0, {2, 2, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 8, 4, 0, {2, 2, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 8, 4, 0, {2, 2, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 8, 4, 0, {2, 2, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 8, 4, 0, {2, 2, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_CANNY_NON_MAXIMA_SUPPRESS[] = {{60, 8, 4, 0, {2, 2, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_CANNY_NON_MAXIMA_SUPPRESS[] = {{64, 8, 4, 0, {2, 2, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 10, 4, 0, {2, 2, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 10, 4, 0, {2, 2, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 10, 4, 0, {2, 2, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 10, 4, 0, {2, 2, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 10, 4, 0, {2, 2, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 10, 4, 0, {2, 2, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 10, 4, 0, {2, 2, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 10, 4, 0, {2, 2, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 10, 4, 0, {2, 2, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 10, 4, 0, {2, 2, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 10, 4, 0, {2, 2, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 10, 4, 0, {2, 2, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 10, 4, 0, {2, 2, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_CANNY_NON_MAXIMA_SUPPRESS[] = {{48, 10, 4, 0, {2, 2, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_CANNY_NON_MAXIMA_SUPPRESS[] = {{52, 10, 4, 0, {2, 2, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_CANNY_NON_MAXIMA_SUPPRESS[] = {{56, 10, 4, 0, {2, 2, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 12, 4, 0, {2, 2, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 12, 4, 0, {2, 2, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 12, 4, 0, {2, 2, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 12, 4, 0, {2, 2, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 12, 4, 0, {2, 2, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 12, 4, 0, {2, 2, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 12, 4, 0, {2, 2, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 12, 4, 0, {2, 2, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 12, 4, 0, {2, 2, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 12, 4, 0, {2, 2, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 12, 4, 0, {2, 2, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 12, 4, 0, {2, 2, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 12, 4, 0, {2, 2, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_CANNY_NON_MAXIMA_SUPPRESS[] = {{44, 12, 4, 0, {2, 2, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 14, 4, 0, {2, 2, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 14, 4, 0, {2, 2, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 14, 4, 0, {2, 2, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 14, 4, 0, {2, 2, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 14, 4, 0, {2, 2, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 14, 4, 0, {2, 2, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 14, 4, 0, {2, 2, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 14, 4, 0, {2, 2, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 14, 4, 0, {2, 2, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 14, 4, 0, {2, 2, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 14, 4, 0, {2, 2, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 14, 4, 0, {2, 2, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 14, 4, 0, {2, 2, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 14, 4, 0, {2, 2, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 14, 4, 0, {2, 2, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 14, 4, 0, {2, 2, 2, 2}}, {32, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_CANNY_NON_MAXIMA_SUPPRESS[] = {{36, 14, 4, 0, {2, 2, 2, 2}}, {36, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_CANNY_NON_MAXIMA_SUPPRESS[] = {{40, 14, 4, 0, {2, 2, 2, 2}}, {40, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 16, 4, 0, {2, 2, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 16, 4, 0, {2, 2, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 16, 4, 0, {2, 2, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 16, 4, 0, {2, 2, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 16, 4, 0, {2, 2, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 16, 4, 0, {2, 2, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 16, 4, 0, {2, 2, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 16, 4, 0, {2, 2, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 16, 4, 0, {2, 2, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 16, 4, 0, {2, 2, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 16, 4, 0, {2, 2, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_CANNY_NON_MAXIMA_SUPPRESS[] = {{32, 16, 4, 0, {2, 2, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 18, 4, 0, {2, 2, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 18, 4, 0, {2, 2, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 18, 4, 0, {2, 2, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 18, 4, 0, {2, 2, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 18, 4, 0, {2, 2, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 18, 4, 0, {2, 2, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 18, 4, 0, {2, 2, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 18, 4, 0, {2, 2, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 18, 4, 0, {2, 2, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 18, 4, 0, {2, 2, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 18, 4, 0, {2, 2, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 18, 4, 0, {2, 2, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 18, 4, 0, {2, 2, 2, 2}}, {26, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 18, 4, 0, {2, 2, 2, 2}}, {28, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_CANNY_NON_MAXIMA_SUPPRESS[] = {{30, 18, 4, 0, {2, 2, 2, 2}}, {30, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 20, 4, 0, {2, 2, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 20, 4, 0, {2, 2, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 20, 4, 0, {2, 2, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 20, 4, 0, {2, 2, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 20, 4, 0, {2, 2, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 20, 4, 0, {2, 2, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 20, 4, 0, {2, 2, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 20, 4, 0, {2, 2, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 20, 4, 0, {2, 2, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_CANNY_NON_MAXIMA_SUPPRESS[] = {{26, 20, 4, 0, {2, 2, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_CANNY_NON_MAXIMA_SUPPRESS[] = {{28, 20, 4, 0, {2, 2, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 22, 4, 0, {2, 2, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 22, 4, 0, {2, 2, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 22, 4, 0, {2, 2, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 22, 4, 0, {2, 2, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 22, 4, 0, {2, 2, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 22, 4, 0, {2, 2, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 22, 4, 0, {2, 2, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 22, 4, 0, {2, 2, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 22, 4, 0, {2, 2, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 22, 4, 0, {2, 2, 2, 2}}, {20, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 22, 4, 0, {2, 2, 2, 2}}, {22, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_CANNY_NON_MAXIMA_SUPPRESS[] = {{24, 22, 4, 0, {2, 2, 2, 2}}, {24, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 24, 4, 0, {2, 2, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 24, 4, 0, {2, 2, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 24, 4, 0, {2, 2, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 24, 4, 0, {2, 2, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 24, 4, 0, {2, 2, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 24, 4, 0, {2, 2, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 24, 4, 0, {2, 2, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 24, 4, 0, {2, 2, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_CANNY_NON_MAXIMA_SUPPRESS[] = {{22, 24, 4, 0, {2, 2, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 26, 4, 0, {2, 2, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 26, 4, 0, {2, 2, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 26, 4, 0, {2, 2, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 26, 4, 0, {2, 2, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 26, 4, 0, {2, 2, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 26, 4, 0, {2, 2, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 26, 4, 0, {2, 2, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 26, 4, 0, {2, 2, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 26, 4, 0, {2, 2, 2, 2}}, {18, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_CANNY_NON_MAXIMA_SUPPRESS[] = {{20, 26, 4, 0, {2, 2, 2, 2}}, {20, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 28, 4, 0, {2, 2, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 28, 4, 0, {2, 2, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 28, 4, 0, {2, 2, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 28, 4, 0, {2, 2, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 28, 4, 0, {2, 2, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 28, 4, 0, {2, 2, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 28, 4, 0, {2, 2, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 30, 4, 0, {2, 2, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 30, 4, 0, {2, 2, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 30, 4, 0, {2, 2, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 30, 4, 0, {2, 2, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 30, 4, 0, {2, 2, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 30, 4, 0, {2, 2, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 30, 4, 0, {2, 2, 2, 2}}, {14, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 30, 4, 0, {2, 2, 2, 2}}, {16, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_CANNY_NON_MAXIMA_SUPPRESS[] = {{18, 30, 4, 0, {2, 2, 2, 2}}, {18, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 32, 4, 0, {2, 2, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_CANNY_NON_MAXIMA_SUPPRESS[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_CANNY_NON_MAXIMA_SUPPRESS[] = {{6, 32, 4, 0, {2, 2, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_CANNY_NON_MAXIMA_SUPPRESS[] = {{8, 32, 4, 0, {2, 2, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_CANNY_NON_MAXIMA_SUPPRESS[] = {{10, 32, 4, 0, {2, 2, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_CANNY_NON_MAXIMA_SUPPRESS[] = {{12, 32, 4, 0, {2, 2, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_CANNY_NON_MAXIMA_SUPPRESS[] = {{14, 32, 4, 0, {2, 2, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_CANNY_NON_MAXIMA_SUPPRESS[] = {{16, 32, 4, 0, {2, 2, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CANNY_NON_MAXIMA_SUPPRESS[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_CANNY_NON_MAXIMA_SUPPRESS[] = {{1, 0}};

static acf_scenario gScenarioArray_CANNY_NON_MAXIMA_SUPPRESS[] = {
{1, 1, 72, 32, 2, gScenarioBufferData0_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData0_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 1, 96, 32, 2, gScenarioBufferData1_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData1_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 1, 124, 32, 2, gScenarioBufferData2_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData2_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 1, 148, 32, 2, gScenarioBufferData3_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData3_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 1, 176, 32, 2, gScenarioBufferData4_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData4_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 1, 200, 32, 2, gScenarioBufferData5_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData5_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 1, 228, 32, 2, gScenarioBufferData6_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData6_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 1, 252, 32, 2, gScenarioBufferData7_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData7_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 1, 280, 32, 2, gScenarioBufferData8_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData8_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 1, 304, 32, 2, gScenarioBufferData9_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData9_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 1, 332, 32, 2, gScenarioBufferData10_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData10_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 1, 356, 32, 2, gScenarioBufferData11_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData11_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 1, 384, 32, 2, gScenarioBufferData12_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData12_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 1, 408, 32, 2, gScenarioBufferData13_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData13_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 1, 436, 32, 2, gScenarioBufferData14_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData14_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 1, 460, 32, 2, gScenarioBufferData15_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData15_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 1, 512, 32, 2, gScenarioBufferData16_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData16_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 1, 564, 32, 2, gScenarioBufferData17_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData17_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 1, 616, 32, 2, gScenarioBufferData18_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData18_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 1, 668, 32, 2, gScenarioBufferData19_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData19_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 1, 720, 32, 2, gScenarioBufferData20_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData20_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 1, 772, 32, 2, gScenarioBufferData21_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData21_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 1, 824, 32, 2, gScenarioBufferData22_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData22_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 1, 876, 32, 2, gScenarioBufferData23_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData23_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 1, 980, 32, 2, gScenarioBufferData24_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData24_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 1, 1084, 32, 2, gScenarioBufferData25_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData25_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 1, 1188, 32, 2, gScenarioBufferData26_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData26_CANNY_NON_MAXIMA_SUPPRESS, 4},
{48, 1, 1292, 32, 2, gScenarioBufferData27_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData27_CANNY_NON_MAXIMA_SUPPRESS, 4},
{52, 1, 1396, 32, 2, gScenarioBufferData28_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData28_CANNY_NON_MAXIMA_SUPPRESS, 4},
{56, 1, 1500, 32, 2, gScenarioBufferData29_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData29_CANNY_NON_MAXIMA_SUPPRESS, 4},
{60, 1, 1604, 32, 2, gScenarioBufferData30_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData30_CANNY_NON_MAXIMA_SUPPRESS, 4},
{64, 1, 1708, 32, 2, gScenarioBufferData31_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData31_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 2, 80, 32, 1, gScenarioBufferData32_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData32_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 2, 112, 32, 1, gScenarioBufferData33_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData33_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 2, 144, 32, 1, gScenarioBufferData34_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData34_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 2, 176, 32, 1, gScenarioBufferData35_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData35_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 2, 208, 32, 1, gScenarioBufferData36_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData36_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 2, 240, 32, 1, gScenarioBufferData37_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData37_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 2, 272, 32, 1, gScenarioBufferData38_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData38_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 2, 304, 32, 1, gScenarioBufferData39_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData39_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 2, 336, 32, 1, gScenarioBufferData40_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData40_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 2, 368, 32, 1, gScenarioBufferData41_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData41_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 2, 400, 32, 1, gScenarioBufferData42_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData42_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 2, 432, 32, 1, gScenarioBufferData43_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData43_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 2, 464, 32, 1, gScenarioBufferData44_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData44_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 2, 496, 32, 1, gScenarioBufferData45_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData45_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 2, 528, 32, 1, gScenarioBufferData46_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData46_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 2, 560, 32, 1, gScenarioBufferData47_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData47_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 2, 624, 32, 1, gScenarioBufferData48_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData48_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 2, 688, 32, 1, gScenarioBufferData49_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData49_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 2, 752, 32, 1, gScenarioBufferData50_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData50_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 2, 816, 32, 1, gScenarioBufferData51_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData51_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 2, 880, 32, 1, gScenarioBufferData52_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData52_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 2, 944, 32, 1, gScenarioBufferData53_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData53_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 2, 1008, 32, 1, gScenarioBufferData54_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData54_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 2, 1072, 32, 1, gScenarioBufferData55_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData55_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 2, 1200, 32, 1, gScenarioBufferData56_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData56_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 2, 1328, 32, 1, gScenarioBufferData57_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData57_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 2, 1456, 32, 1, gScenarioBufferData58_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData58_CANNY_NON_MAXIMA_SUPPRESS, 4},
{48, 2, 1584, 32, 1, gScenarioBufferData59_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData59_CANNY_NON_MAXIMA_SUPPRESS, 4},
{52, 2, 1712, 32, 1, gScenarioBufferData60_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData60_CANNY_NON_MAXIMA_SUPPRESS, 4},
{56, 2, 1840, 32, 1, gScenarioBufferData61_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData61_CANNY_NON_MAXIMA_SUPPRESS, 4},
{60, 2, 1968, 32, 1, gScenarioBufferData62_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData62_CANNY_NON_MAXIMA_SUPPRESS, 4},
{64, 2, 2096, 32, 1, gScenarioBufferData63_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData63_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 3, 108, 32, 1, gScenarioBufferData64_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData64_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 3, 152, 32, 1, gScenarioBufferData65_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData65_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 3, 196, 32, 1, gScenarioBufferData66_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData66_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 3, 240, 32, 1, gScenarioBufferData67_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData67_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 3, 284, 32, 1, gScenarioBufferData68_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData68_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 3, 328, 32, 1, gScenarioBufferData69_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData69_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 3, 372, 32, 1, gScenarioBufferData70_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData70_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 3, 416, 32, 1, gScenarioBufferData71_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData71_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 3, 460, 32, 1, gScenarioBufferData72_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData72_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 3, 504, 32, 1, gScenarioBufferData73_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData73_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 3, 548, 32, 1, gScenarioBufferData74_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData74_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 3, 592, 32, 1, gScenarioBufferData75_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData75_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 3, 636, 32, 1, gScenarioBufferData76_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData76_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 3, 680, 32, 1, gScenarioBufferData77_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData77_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 3, 724, 32, 1, gScenarioBufferData78_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData78_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 3, 768, 32, 1, gScenarioBufferData79_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData79_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 3, 856, 32, 1, gScenarioBufferData80_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData80_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 3, 944, 32, 1, gScenarioBufferData81_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData81_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 3, 1032, 32, 1, gScenarioBufferData82_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData82_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 3, 1120, 32, 1, gScenarioBufferData83_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData83_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 3, 1208, 32, 1, gScenarioBufferData84_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData84_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 3, 1296, 32, 1, gScenarioBufferData85_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData85_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 3, 1384, 32, 1, gScenarioBufferData86_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData86_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 3, 1472, 32, 1, gScenarioBufferData87_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData87_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 3, 1648, 32, 1, gScenarioBufferData88_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData88_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 3, 1824, 32, 1, gScenarioBufferData89_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData89_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 3, 2000, 32, 1, gScenarioBufferData90_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData90_CANNY_NON_MAXIMA_SUPPRESS, 4},
{48, 3, 2176, 32, 1, gScenarioBufferData91_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData91_CANNY_NON_MAXIMA_SUPPRESS, 4},
{52, 3, 2352, 32, 1, gScenarioBufferData92_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData92_CANNY_NON_MAXIMA_SUPPRESS, 4},
{56, 3, 2528, 32, 1, gScenarioBufferData93_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData93_CANNY_NON_MAXIMA_SUPPRESS, 4},
{60, 3, 2704, 32, 1, gScenarioBufferData94_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData94_CANNY_NON_MAXIMA_SUPPRESS, 4},
{64, 3, 2880, 32, 1, gScenarioBufferData95_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData95_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 4, 136, 32, 1, gScenarioBufferData96_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData96_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 4, 192, 32, 1, gScenarioBufferData97_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData97_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 4, 248, 32, 1, gScenarioBufferData98_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData98_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 4, 304, 32, 1, gScenarioBufferData99_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData99_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 4, 360, 32, 1, gScenarioBufferData100_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData100_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 4, 416, 32, 1, gScenarioBufferData101_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData101_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 4, 472, 32, 1, gScenarioBufferData102_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData102_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 4, 528, 32, 1, gScenarioBufferData103_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData103_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 4, 584, 32, 1, gScenarioBufferData104_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData104_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 4, 640, 32, 1, gScenarioBufferData105_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData105_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 4, 696, 32, 1, gScenarioBufferData106_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData106_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 4, 752, 32, 1, gScenarioBufferData107_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData107_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 4, 808, 32, 1, gScenarioBufferData108_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData108_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 4, 864, 32, 1, gScenarioBufferData109_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData109_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 4, 920, 32, 1, gScenarioBufferData110_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData110_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 4, 976, 32, 1, gScenarioBufferData111_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData111_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 4, 1088, 32, 1, gScenarioBufferData112_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData112_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 4, 1200, 32, 1, gScenarioBufferData113_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData113_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 4, 1312, 32, 1, gScenarioBufferData114_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData114_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 4, 1424, 32, 1, gScenarioBufferData115_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData115_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 4, 1536, 32, 1, gScenarioBufferData116_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData116_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 4, 1648, 32, 1, gScenarioBufferData117_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData117_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 4, 1760, 32, 1, gScenarioBufferData118_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData118_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 4, 1872, 32, 1, gScenarioBufferData119_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData119_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 4, 2096, 32, 1, gScenarioBufferData120_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData120_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 4, 2320, 32, 1, gScenarioBufferData121_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData121_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 4, 2544, 32, 1, gScenarioBufferData122_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData122_CANNY_NON_MAXIMA_SUPPRESS, 4},
{48, 4, 2768, 32, 1, gScenarioBufferData123_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData123_CANNY_NON_MAXIMA_SUPPRESS, 4},
{52, 4, 2992, 32, 1, gScenarioBufferData124_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData124_CANNY_NON_MAXIMA_SUPPRESS, 4},
{56, 4, 3216, 32, 1, gScenarioBufferData125_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData125_CANNY_NON_MAXIMA_SUPPRESS, 4},
{60, 4, 3440, 32, 1, gScenarioBufferData126_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData126_CANNY_NON_MAXIMA_SUPPRESS, 4},
{64, 4, 3664, 32, 1, gScenarioBufferData127_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData127_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 5, 164, 32, 1, gScenarioBufferData128_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData128_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 5, 232, 32, 1, gScenarioBufferData129_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData129_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 5, 300, 32, 1, gScenarioBufferData130_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData130_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 5, 368, 32, 1, gScenarioBufferData131_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData131_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 5, 436, 32, 1, gScenarioBufferData132_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData132_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 5, 504, 32, 1, gScenarioBufferData133_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData133_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 5, 572, 32, 1, gScenarioBufferData134_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData134_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 5, 640, 32, 1, gScenarioBufferData135_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData135_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 5, 708, 32, 1, gScenarioBufferData136_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData136_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 5, 776, 32, 1, gScenarioBufferData137_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData137_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 5, 844, 32, 1, gScenarioBufferData138_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData138_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 5, 912, 32, 1, gScenarioBufferData139_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData139_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 5, 980, 32, 1, gScenarioBufferData140_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData140_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 5, 1048, 32, 1, gScenarioBufferData141_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData141_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 5, 1116, 32, 1, gScenarioBufferData142_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData142_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 5, 1184, 32, 1, gScenarioBufferData143_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData143_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 5, 1320, 32, 1, gScenarioBufferData144_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData144_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 5, 1456, 32, 1, gScenarioBufferData145_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData145_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 5, 1592, 32, 1, gScenarioBufferData146_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData146_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 5, 1728, 32, 1, gScenarioBufferData147_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData147_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 5, 1864, 32, 1, gScenarioBufferData148_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData148_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 5, 2000, 32, 1, gScenarioBufferData149_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData149_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 5, 2136, 32, 1, gScenarioBufferData150_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData150_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 5, 2272, 32, 1, gScenarioBufferData151_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData151_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 5, 2544, 32, 1, gScenarioBufferData152_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData152_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 5, 2816, 32, 1, gScenarioBufferData153_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData153_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 5, 3088, 32, 1, gScenarioBufferData154_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData154_CANNY_NON_MAXIMA_SUPPRESS, 4},
{48, 5, 3360, 32, 1, gScenarioBufferData155_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData155_CANNY_NON_MAXIMA_SUPPRESS, 4},
{52, 5, 3632, 32, 1, gScenarioBufferData156_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData156_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 6, 192, 32, 1, gScenarioBufferData157_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData157_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 6, 272, 32, 1, gScenarioBufferData158_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData158_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 6, 352, 32, 1, gScenarioBufferData159_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData159_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 6, 432, 32, 1, gScenarioBufferData160_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData160_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 6, 512, 32, 1, gScenarioBufferData161_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData161_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 6, 592, 32, 1, gScenarioBufferData162_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData162_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 6, 672, 32, 1, gScenarioBufferData163_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData163_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 6, 752, 32, 1, gScenarioBufferData164_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData164_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 6, 832, 32, 1, gScenarioBufferData165_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData165_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 6, 912, 32, 1, gScenarioBufferData166_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData166_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 6, 992, 32, 1, gScenarioBufferData167_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData167_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 6, 1072, 32, 1, gScenarioBufferData168_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData168_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 6, 1152, 32, 1, gScenarioBufferData169_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData169_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 6, 1232, 32, 1, gScenarioBufferData170_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData170_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 6, 1312, 32, 1, gScenarioBufferData171_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData171_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 6, 1392, 32, 1, gScenarioBufferData172_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData172_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 6, 1552, 32, 1, gScenarioBufferData173_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData173_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 6, 1712, 32, 1, gScenarioBufferData174_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData174_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 6, 1872, 32, 1, gScenarioBufferData175_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData175_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 6, 2032, 32, 1, gScenarioBufferData176_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData176_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 6, 2192, 32, 1, gScenarioBufferData177_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData177_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 6, 2352, 32, 1, gScenarioBufferData178_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData178_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 6, 2512, 32, 1, gScenarioBufferData179_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData179_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 6, 2672, 32, 1, gScenarioBufferData180_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData180_CANNY_NON_MAXIMA_SUPPRESS, 4},
{36, 6, 2992, 32, 1, gScenarioBufferData181_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData181_CANNY_NON_MAXIMA_SUPPRESS, 4},
{40, 6, 3312, 32, 1, gScenarioBufferData182_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData182_CANNY_NON_MAXIMA_SUPPRESS, 4},
{44, 6, 3632, 32, 1, gScenarioBufferData183_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData183_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 8, 248, 32, 1, gScenarioBufferData184_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData184_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 8, 352, 32, 1, gScenarioBufferData185_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData185_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 8, 456, 32, 1, gScenarioBufferData186_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData186_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 8, 560, 32, 1, gScenarioBufferData187_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData187_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 8, 664, 32, 1, gScenarioBufferData188_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData188_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 8, 768, 32, 1, gScenarioBufferData189_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData189_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 8, 872, 32, 1, gScenarioBufferData190_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData190_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 8, 976, 32, 1, gScenarioBufferData191_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData191_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 8, 1080, 32, 1, gScenarioBufferData192_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData192_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 8, 1184, 32, 1, gScenarioBufferData193_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData193_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 8, 1288, 32, 1, gScenarioBufferData194_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData194_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 8, 1392, 32, 1, gScenarioBufferData195_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData195_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 8, 1496, 32, 1, gScenarioBufferData196_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData196_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 8, 1600, 32, 1, gScenarioBufferData197_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData197_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 8, 1704, 32, 1, gScenarioBufferData198_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData198_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 8, 1808, 32, 1, gScenarioBufferData199_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData199_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 8, 2016, 32, 1, gScenarioBufferData200_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData200_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 8, 2224, 32, 1, gScenarioBufferData201_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData201_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 8, 2432, 32, 1, gScenarioBufferData202_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData202_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 8, 2640, 32, 1, gScenarioBufferData203_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData203_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 8, 2848, 32, 1, gScenarioBufferData204_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData204_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 8, 3056, 32, 1, gScenarioBufferData205_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData205_CANNY_NON_MAXIMA_SUPPRESS, 4},
{30, 8, 3264, 32, 1, gScenarioBufferData206_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData206_CANNY_NON_MAXIMA_SUPPRESS, 4},
{32, 8, 3472, 32, 1, gScenarioBufferData207_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData207_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 10, 304, 32, 1, gScenarioBufferData208_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData208_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 10, 432, 32, 1, gScenarioBufferData209_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData209_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 10, 560, 32, 1, gScenarioBufferData210_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData210_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 10, 688, 32, 1, gScenarioBufferData211_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData211_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 10, 816, 32, 1, gScenarioBufferData212_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData212_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 10, 944, 32, 1, gScenarioBufferData213_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData213_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 10, 1072, 32, 1, gScenarioBufferData214_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData214_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 10, 1200, 32, 1, gScenarioBufferData215_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData215_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 10, 1328, 32, 1, gScenarioBufferData216_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData216_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 10, 1456, 32, 1, gScenarioBufferData217_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData217_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 10, 1584, 32, 1, gScenarioBufferData218_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData218_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 10, 1712, 32, 1, gScenarioBufferData219_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData219_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 10, 1840, 32, 1, gScenarioBufferData220_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData220_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 10, 1968, 32, 1, gScenarioBufferData221_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData221_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 10, 2096, 32, 1, gScenarioBufferData222_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData222_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 10, 2224, 32, 1, gScenarioBufferData223_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData223_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 10, 2480, 32, 1, gScenarioBufferData224_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData224_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 10, 2736, 32, 1, gScenarioBufferData225_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData225_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 10, 2992, 32, 1, gScenarioBufferData226_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData226_CANNY_NON_MAXIMA_SUPPRESS, 4},
{24, 10, 3248, 32, 1, gScenarioBufferData227_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData227_CANNY_NON_MAXIMA_SUPPRESS, 4},
{26, 10, 3504, 32, 1, gScenarioBufferData228_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData228_CANNY_NON_MAXIMA_SUPPRESS, 4},
{28, 10, 3760, 32, 1, gScenarioBufferData229_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData229_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 12, 360, 32, 1, gScenarioBufferData230_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData230_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 12, 512, 32, 1, gScenarioBufferData231_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData231_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 12, 664, 32, 1, gScenarioBufferData232_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData232_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 12, 816, 32, 1, gScenarioBufferData233_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData233_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 12, 968, 32, 1, gScenarioBufferData234_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData234_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 12, 1120, 32, 1, gScenarioBufferData235_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData235_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 12, 1272, 32, 1, gScenarioBufferData236_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData236_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 12, 1424, 32, 1, gScenarioBufferData237_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData237_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 12, 1576, 32, 1, gScenarioBufferData238_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData238_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 12, 1728, 32, 1, gScenarioBufferData239_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData239_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 12, 1880, 32, 1, gScenarioBufferData240_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData240_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 12, 2032, 32, 1, gScenarioBufferData241_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData241_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 12, 2184, 32, 1, gScenarioBufferData242_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData242_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 12, 2336, 32, 1, gScenarioBufferData243_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData243_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 12, 2488, 32, 1, gScenarioBufferData244_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData244_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 12, 2640, 32, 1, gScenarioBufferData245_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData245_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 12, 2944, 32, 1, gScenarioBufferData246_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData246_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 12, 3248, 32, 1, gScenarioBufferData247_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData247_CANNY_NON_MAXIMA_SUPPRESS, 4},
{22, 12, 3552, 32, 1, gScenarioBufferData248_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData248_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 14, 416, 32, 1, gScenarioBufferData249_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData249_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 14, 592, 32, 1, gScenarioBufferData250_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData250_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 14, 768, 32, 1, gScenarioBufferData251_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData251_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 14, 944, 32, 1, gScenarioBufferData252_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData252_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 14, 1120, 32, 1, gScenarioBufferData253_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData253_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 14, 1296, 32, 1, gScenarioBufferData254_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData254_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 14, 1472, 32, 1, gScenarioBufferData255_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData255_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 14, 1648, 32, 1, gScenarioBufferData256_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData256_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 14, 1824, 32, 1, gScenarioBufferData257_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData257_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 14, 2000, 32, 1, gScenarioBufferData258_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData258_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 14, 2176, 32, 1, gScenarioBufferData259_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData259_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 14, 2352, 32, 1, gScenarioBufferData260_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData260_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 14, 2528, 32, 1, gScenarioBufferData261_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData261_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 14, 2704, 32, 1, gScenarioBufferData262_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData262_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 14, 2880, 32, 1, gScenarioBufferData263_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData263_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 14, 3056, 32, 1, gScenarioBufferData264_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData264_CANNY_NON_MAXIMA_SUPPRESS, 4},
{18, 14, 3408, 32, 1, gScenarioBufferData265_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData265_CANNY_NON_MAXIMA_SUPPRESS, 4},
{20, 14, 3760, 32, 1, gScenarioBufferData266_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData266_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 16, 472, 32, 1, gScenarioBufferData267_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData267_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 16, 672, 32, 1, gScenarioBufferData268_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData268_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 16, 872, 32, 1, gScenarioBufferData269_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData269_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 16, 1072, 32, 1, gScenarioBufferData270_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData270_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 16, 1272, 32, 1, gScenarioBufferData271_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData271_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 16, 1472, 32, 1, gScenarioBufferData272_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData272_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 16, 1672, 32, 1, gScenarioBufferData273_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData273_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 16, 1872, 32, 1, gScenarioBufferData274_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData274_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 16, 2072, 32, 1, gScenarioBufferData275_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData275_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 16, 2272, 32, 1, gScenarioBufferData276_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData276_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 16, 2472, 32, 1, gScenarioBufferData277_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData277_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 16, 2672, 32, 1, gScenarioBufferData278_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData278_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 16, 2872, 32, 1, gScenarioBufferData279_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData279_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 16, 3072, 32, 1, gScenarioBufferData280_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData280_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 16, 3272, 32, 1, gScenarioBufferData281_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData281_CANNY_NON_MAXIMA_SUPPRESS, 4},
{16, 16, 3472, 32, 1, gScenarioBufferData282_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData282_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 18, 528, 32, 1, gScenarioBufferData283_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData283_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 18, 752, 32, 1, gScenarioBufferData284_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData284_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 18, 976, 32, 1, gScenarioBufferData285_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData285_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 18, 1200, 32, 1, gScenarioBufferData286_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData286_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 18, 1424, 32, 1, gScenarioBufferData287_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData287_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 18, 1648, 32, 1, gScenarioBufferData288_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData288_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 18, 1872, 32, 1, gScenarioBufferData289_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData289_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 18, 2096, 32, 1, gScenarioBufferData290_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData290_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 18, 2320, 32, 1, gScenarioBufferData291_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData291_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 18, 2544, 32, 1, gScenarioBufferData292_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData292_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 18, 2768, 32, 1, gScenarioBufferData293_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData293_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 18, 2992, 32, 1, gScenarioBufferData294_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData294_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 18, 3216, 32, 1, gScenarioBufferData295_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData295_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 18, 3440, 32, 1, gScenarioBufferData296_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData296_CANNY_NON_MAXIMA_SUPPRESS, 4},
{15, 18, 3664, 32, 1, gScenarioBufferData297_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData297_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 20, 584, 32, 1, gScenarioBufferData298_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData298_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 20, 832, 32, 1, gScenarioBufferData299_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData299_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 20, 1080, 32, 1, gScenarioBufferData300_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData300_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 20, 1328, 32, 1, gScenarioBufferData301_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData301_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 20, 1576, 32, 1, gScenarioBufferData302_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData302_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 20, 1824, 32, 1, gScenarioBufferData303_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData303_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 20, 2072, 32, 1, gScenarioBufferData304_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData304_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 20, 2320, 32, 1, gScenarioBufferData305_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData305_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 20, 2568, 32, 1, gScenarioBufferData306_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData306_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 20, 2816, 32, 1, gScenarioBufferData307_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData307_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 20, 3064, 32, 1, gScenarioBufferData308_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData308_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 20, 3312, 32, 1, gScenarioBufferData309_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData309_CANNY_NON_MAXIMA_SUPPRESS, 4},
{13, 20, 3560, 32, 1, gScenarioBufferData310_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData310_CANNY_NON_MAXIMA_SUPPRESS, 4},
{14, 20, 3808, 32, 1, gScenarioBufferData311_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData311_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 22, 640, 32, 1, gScenarioBufferData312_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData312_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 22, 912, 32, 1, gScenarioBufferData313_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData313_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 22, 1184, 32, 1, gScenarioBufferData314_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData314_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 22, 1456, 32, 1, gScenarioBufferData315_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData315_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 22, 1728, 32, 1, gScenarioBufferData316_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData316_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 22, 2000, 32, 1, gScenarioBufferData317_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData317_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 22, 2272, 32, 1, gScenarioBufferData318_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData318_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 22, 2544, 32, 1, gScenarioBufferData319_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData319_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 22, 2816, 32, 1, gScenarioBufferData320_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData320_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 22, 3088, 32, 1, gScenarioBufferData321_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData321_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 22, 3360, 32, 1, gScenarioBufferData322_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData322_CANNY_NON_MAXIMA_SUPPRESS, 4},
{12, 22, 3632, 32, 1, gScenarioBufferData323_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData323_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 24, 696, 32, 1, gScenarioBufferData324_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData324_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 24, 992, 32, 1, gScenarioBufferData325_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData325_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 24, 1288, 32, 1, gScenarioBufferData326_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData326_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 24, 1584, 32, 1, gScenarioBufferData327_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData327_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 24, 1880, 32, 1, gScenarioBufferData328_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData328_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 24, 2176, 32, 1, gScenarioBufferData329_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData329_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 24, 2472, 32, 1, gScenarioBufferData330_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData330_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 24, 2768, 32, 1, gScenarioBufferData331_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData331_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 24, 3064, 32, 1, gScenarioBufferData332_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData332_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 24, 3360, 32, 1, gScenarioBufferData333_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData333_CANNY_NON_MAXIMA_SUPPRESS, 4},
{11, 24, 3656, 32, 1, gScenarioBufferData334_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData334_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 26, 752, 32, 1, gScenarioBufferData335_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData335_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 26, 1072, 32, 1, gScenarioBufferData336_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData336_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 26, 1392, 32, 1, gScenarioBufferData337_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData337_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 26, 1712, 32, 1, gScenarioBufferData338_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData338_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 26, 2032, 32, 1, gScenarioBufferData339_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData339_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 26, 2352, 32, 1, gScenarioBufferData340_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData340_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 26, 2672, 32, 1, gScenarioBufferData341_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData341_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 26, 2992, 32, 1, gScenarioBufferData342_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData342_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 26, 3312, 32, 1, gScenarioBufferData343_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData343_CANNY_NON_MAXIMA_SUPPRESS, 4},
{10, 26, 3632, 32, 1, gScenarioBufferData344_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData344_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 28, 808, 32, 1, gScenarioBufferData345_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData345_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 28, 1152, 32, 1, gScenarioBufferData346_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData346_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 28, 1496, 32, 1, gScenarioBufferData347_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData347_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 28, 1840, 32, 1, gScenarioBufferData348_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData348_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 28, 2184, 32, 1, gScenarioBufferData349_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData349_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 28, 2528, 32, 1, gScenarioBufferData350_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData350_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 28, 2872, 32, 1, gScenarioBufferData351_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData351_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 28, 3216, 32, 1, gScenarioBufferData352_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData352_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 28, 3560, 32, 1, gScenarioBufferData353_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData353_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 30, 864, 32, 1, gScenarioBufferData354_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData354_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 30, 1232, 32, 1, gScenarioBufferData355_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData355_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 30, 1600, 32, 1, gScenarioBufferData356_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData356_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 30, 1968, 32, 1, gScenarioBufferData357_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData357_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 30, 2336, 32, 1, gScenarioBufferData358_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData358_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 30, 2704, 32, 1, gScenarioBufferData359_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData359_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 30, 3072, 32, 1, gScenarioBufferData360_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData360_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 30, 3440, 32, 1, gScenarioBufferData361_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData361_CANNY_NON_MAXIMA_SUPPRESS, 4},
{9, 30, 3808, 32, 1, gScenarioBufferData362_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData362_CANNY_NON_MAXIMA_SUPPRESS, 4},
{1, 32, 920, 32, 1, gScenarioBufferData363_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData363_CANNY_NON_MAXIMA_SUPPRESS, 4},
{2, 32, 1312, 32, 1, gScenarioBufferData364_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData364_CANNY_NON_MAXIMA_SUPPRESS, 4},
{3, 32, 1704, 32, 1, gScenarioBufferData365_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData365_CANNY_NON_MAXIMA_SUPPRESS, 4},
{4, 32, 2096, 32, 1, gScenarioBufferData366_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData366_CANNY_NON_MAXIMA_SUPPRESS, 4},
{5, 32, 2488, 32, 1, gScenarioBufferData367_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData367_CANNY_NON_MAXIMA_SUPPRESS, 4},
{6, 32, 2880, 32, 1, gScenarioBufferData368_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData368_CANNY_NON_MAXIMA_SUPPRESS, 4},
{7, 32, 3272, 32, 1, gScenarioBufferData369_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData369_CANNY_NON_MAXIMA_SUPPRESS, 4},
{8, 32, 3664, 32, 1, gScenarioBufferData370_CANNY_NON_MAXIMA_SUPPRESS, 64, gScenarioKernelData370_CANNY_NON_MAXIMA_SUPPRESS, 4}
};

static acf_scenario_list gScenarioList_CANNY_NON_MAXIMA_SUPPRESS = {
371, //number of scenarios
gScenarioArray_CANNY_NON_MAXIMA_SUPPRESS};
//**************************************************************

class CANNY_NON_MAXIMA_SUPPRESS : public ACF_Process_APU
{

public:
   CANNY_NON_MAXIMA_SUPPRESS(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CANNY_NON_MAXIMA_SUPPRESS");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CANNY_NON_MAXIMA_SUPPRESS",
                                        CANNY_NON_MAXIMA_SUPPRESS_LOAD_SEGMENTS,
                                        CANNY_NON_MAXIMA_SUPPRESS_LOAD_PMEM, CANNY_NON_MAXIMA_SUPPRESS_LOAD_PMEM_SIZE,
                                        CANNY_NON_MAXIMA_SUPPRESS_LOAD_DMEM, CANNY_NON_MAXIMA_SUPPRESS_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CANNY_NON_MAXIMA_SUPPRESS_APEX_LOG_BUFFER, CANNY_NON_MAXIMA_SUPPRESS_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("LOW_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("HIGH_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CANNY_NON_MAXIMA_SUPPRESS);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CANNY_NON_MAXIMA_SUPPRESS

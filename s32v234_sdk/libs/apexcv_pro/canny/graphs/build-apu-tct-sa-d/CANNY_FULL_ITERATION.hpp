#ifndef _ACF_PROCESS_APU_CANNY_FULL_ITERATION
#define _ACF_PROCESS_APU_CANNY_FULL_ITERATION

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CANNY_FULL_ITERATION_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CANNY_FULL_ITERATION[] = {{4, 1, 7, 0, {2, 2, 2, 2}}, {4, 1, 4, 2, {1, 1, 2, 2}}, {4, 1, 4, 3, {1, 1, 2, 2}}, {4, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CANNY_FULL_ITERATION[] = {{8, 1, 7, 0, {2, 2, 2, 2}}, {8, 1, 4, 2, {1, 1, 2, 2}}, {8, 1, 4, 3, {1, 1, 2, 2}}, {8, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CANNY_FULL_ITERATION[] = {{12, 1, 7, 0, {2, 2, 2, 2}}, {12, 1, 4, 2, {1, 1, 2, 2}}, {12, 1, 4, 3, {1, 1, 2, 2}}, {12, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CANNY_FULL_ITERATION[] = {{16, 1, 7, 0, {2, 2, 2, 2}}, {16, 1, 4, 2, {1, 1, 2, 2}}, {16, 1, 4, 3, {1, 1, 2, 2}}, {16, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CANNY_FULL_ITERATION[] = {{20, 1, 7, 0, {2, 2, 2, 2}}, {20, 1, 4, 2, {1, 1, 2, 2}}, {20, 1, 4, 3, {1, 1, 2, 2}}, {20, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CANNY_FULL_ITERATION[] = {{24, 1, 7, 0, {2, 2, 2, 2}}, {24, 1, 4, 2, {1, 1, 2, 2}}, {24, 1, 4, 3, {1, 1, 2, 2}}, {24, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CANNY_FULL_ITERATION[] = {{28, 1, 7, 0, {2, 2, 2, 2}}, {28, 1, 4, 2, {1, 1, 2, 2}}, {28, 1, 4, 3, {1, 1, 2, 2}}, {28, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CANNY_FULL_ITERATION[] = {{32, 1, 7, 0, {2, 2, 2, 2}}, {32, 1, 4, 2, {1, 1, 2, 2}}, {32, 1, 4, 3, {1, 1, 2, 2}}, {32, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CANNY_FULL_ITERATION[] = {{36, 1, 7, 0, {2, 2, 2, 2}}, {36, 1, 4, 2, {1, 1, 2, 2}}, {36, 1, 4, 3, {1, 1, 2, 2}}, {36, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CANNY_FULL_ITERATION[] = {{40, 1, 7, 0, {2, 2, 2, 2}}, {40, 1, 4, 2, {1, 1, 2, 2}}, {40, 1, 4, 3, {1, 1, 2, 2}}, {40, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CANNY_FULL_ITERATION[] = {{44, 1, 7, 0, {2, 2, 2, 2}}, {44, 1, 4, 2, {1, 1, 2, 2}}, {44, 1, 4, 3, {1, 1, 2, 2}}, {44, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CANNY_FULL_ITERATION[] = {{48, 1, 7, 0, {2, 2, 2, 2}}, {48, 1, 4, 2, {1, 1, 2, 2}}, {48, 1, 4, 3, {1, 1, 2, 2}}, {48, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CANNY_FULL_ITERATION[] = {{52, 1, 7, 0, {2, 2, 2, 2}}, {52, 1, 4, 2, {1, 1, 2, 2}}, {52, 1, 4, 3, {1, 1, 2, 2}}, {52, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CANNY_FULL_ITERATION[] = {{56, 1, 7, 0, {2, 2, 2, 2}}, {56, 1, 4, 2, {1, 1, 2, 2}}, {56, 1, 4, 3, {1, 1, 2, 2}}, {56, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CANNY_FULL_ITERATION[] = {{60, 1, 7, 0, {2, 2, 2, 2}}, {60, 1, 4, 2, {1, 1, 2, 2}}, {60, 1, 4, 3, {1, 1, 2, 2}}, {60, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CANNY_FULL_ITERATION[] = {{64, 1, 7, 0, {2, 2, 2, 2}}, {64, 1, 4, 2, {1, 1, 2, 2}}, {64, 1, 4, 3, {1, 1, 2, 2}}, {64, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CANNY_FULL_ITERATION[] = {{72, 1, 7, 0, {2, 2, 2, 2}}, {72, 1, 4, 2, {1, 1, 2, 2}}, {72, 1, 4, 3, {1, 1, 2, 2}}, {72, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CANNY_FULL_ITERATION[] = {{80, 1, 7, 0, {2, 2, 2, 2}}, {80, 1, 4, 2, {1, 1, 2, 2}}, {80, 1, 4, 3, {1, 1, 2, 2}}, {80, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CANNY_FULL_ITERATION[] = {{88, 1, 7, 0, {2, 2, 2, 2}}, {88, 1, 4, 2, {1, 1, 2, 2}}, {88, 1, 4, 3, {1, 1, 2, 2}}, {88, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CANNY_FULL_ITERATION[] = {{96, 1, 7, 0, {2, 2, 2, 2}}, {96, 1, 4, 2, {1, 1, 2, 2}}, {96, 1, 4, 3, {1, 1, 2, 2}}, {96, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CANNY_FULL_ITERATION[] = {{104, 1, 7, 0, {2, 2, 2, 2}}, {104, 1, 4, 2, {1, 1, 2, 2}}, {104, 1, 4, 3, {1, 1, 2, 2}}, {104, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CANNY_FULL_ITERATION[] = {{112, 1, 7, 0, {2, 2, 2, 2}}, {112, 1, 4, 2, {1, 1, 2, 2}}, {112, 1, 4, 3, {1, 1, 2, 2}}, {112, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CANNY_FULL_ITERATION[] = {{120, 1, 7, 0, {2, 2, 2, 2}}, {120, 1, 4, 2, {1, 1, 2, 2}}, {120, 1, 4, 3, {1, 1, 2, 2}}, {120, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CANNY_FULL_ITERATION[] = {{128, 1, 7, 0, {2, 2, 2, 2}}, {128, 1, 4, 2, {1, 1, 2, 2}}, {128, 1, 4, 3, {1, 1, 2, 2}}, {128, 1, 2, 4, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CANNY_FULL_ITERATION[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 4, 1, {1, 1, 2, 2}}, {4, 2, 4, 2, {1, 1, 2, 2}}, {4, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CANNY_FULL_ITERATION[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 4, 1, {1, 1, 2, 2}}, {8, 2, 4, 2, {1, 1, 2, 2}}, {8, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CANNY_FULL_ITERATION[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 4, 1, {1, 1, 2, 2}}, {12, 2, 4, 2, {1, 1, 2, 2}}, {12, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CANNY_FULL_ITERATION[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 4, 1, {1, 1, 2, 2}}, {16, 2, 4, 2, {1, 1, 2, 2}}, {16, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CANNY_FULL_ITERATION[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 4, 1, {1, 1, 2, 2}}, {20, 2, 4, 2, {1, 1, 2, 2}}, {20, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CANNY_FULL_ITERATION[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 4, 1, {1, 1, 2, 2}}, {24, 2, 4, 2, {1, 1, 2, 2}}, {24, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CANNY_FULL_ITERATION[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 4, 1, {1, 1, 2, 2}}, {28, 2, 4, 2, {1, 1, 2, 2}}, {28, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CANNY_FULL_ITERATION[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 4, 1, {1, 1, 2, 2}}, {32, 2, 4, 2, {1, 1, 2, 2}}, {32, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CANNY_FULL_ITERATION[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 4, 1, {1, 1, 2, 2}}, {36, 2, 4, 2, {1, 1, 2, 2}}, {36, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CANNY_FULL_ITERATION[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 4, 1, {1, 1, 2, 2}}, {40, 2, 4, 2, {1, 1, 2, 2}}, {40, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CANNY_FULL_ITERATION[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 4, 1, {1, 1, 2, 2}}, {44, 2, 4, 2, {1, 1, 2, 2}}, {44, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CANNY_FULL_ITERATION[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 4, 1, {1, 1, 2, 2}}, {48, 2, 4, 2, {1, 1, 2, 2}}, {48, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CANNY_FULL_ITERATION[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 4, 1, {1, 1, 2, 2}}, {52, 2, 4, 2, {1, 1, 2, 2}}, {52, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CANNY_FULL_ITERATION[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 4, 1, {1, 1, 2, 2}}, {56, 2, 4, 2, {1, 1, 2, 2}}, {56, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CANNY_FULL_ITERATION[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 4, 1, {1, 1, 2, 2}}, {60, 2, 4, 2, {1, 1, 2, 2}}, {60, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CANNY_FULL_ITERATION[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 4, 1, {1, 1, 2, 2}}, {64, 2, 4, 2, {1, 1, 2, 2}}, {64, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CANNY_FULL_ITERATION[] = {{72, 2, 4, 0, {2, 2, 2, 2}}, {72, 2, 4, 1, {1, 1, 2, 2}}, {72, 2, 4, 2, {1, 1, 2, 2}}, {72, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CANNY_FULL_ITERATION[] = {{80, 2, 4, 0, {2, 2, 2, 2}}, {80, 2, 4, 1, {1, 1, 2, 2}}, {80, 2, 4, 2, {1, 1, 2, 2}}, {80, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CANNY_FULL_ITERATION[] = {{88, 2, 4, 0, {2, 2, 2, 2}}, {88, 2, 4, 1, {1, 1, 2, 2}}, {88, 2, 4, 2, {1, 1, 2, 2}}, {88, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CANNY_FULL_ITERATION[] = {{96, 2, 4, 0, {2, 2, 2, 2}}, {96, 2, 4, 1, {1, 1, 2, 2}}, {96, 2, 4, 2, {1, 1, 2, 2}}, {96, 2, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CANNY_FULL_ITERATION[] = {{4, 3, 4, 0, {2, 2, 2, 2}}, {4, 3, 4, 1, {1, 1, 2, 2}}, {4, 3, 4, 2, {1, 1, 2, 2}}, {4, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CANNY_FULL_ITERATION[] = {{8, 3, 4, 0, {2, 2, 2, 2}}, {8, 3, 4, 1, {1, 1, 2, 2}}, {8, 3, 4, 2, {1, 1, 2, 2}}, {8, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CANNY_FULL_ITERATION[] = {{12, 3, 4, 0, {2, 2, 2, 2}}, {12, 3, 4, 1, {1, 1, 2, 2}}, {12, 3, 4, 2, {1, 1, 2, 2}}, {12, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CANNY_FULL_ITERATION[] = {{16, 3, 4, 0, {2, 2, 2, 2}}, {16, 3, 4, 1, {1, 1, 2, 2}}, {16, 3, 4, 2, {1, 1, 2, 2}}, {16, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CANNY_FULL_ITERATION[] = {{20, 3, 4, 0, {2, 2, 2, 2}}, {20, 3, 4, 1, {1, 1, 2, 2}}, {20, 3, 4, 2, {1, 1, 2, 2}}, {20, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CANNY_FULL_ITERATION[] = {{24, 3, 4, 0, {2, 2, 2, 2}}, {24, 3, 4, 1, {1, 1, 2, 2}}, {24, 3, 4, 2, {1, 1, 2, 2}}, {24, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CANNY_FULL_ITERATION[] = {{28, 3, 4, 0, {2, 2, 2, 2}}, {28, 3, 4, 1, {1, 1, 2, 2}}, {28, 3, 4, 2, {1, 1, 2, 2}}, {28, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CANNY_FULL_ITERATION[] = {{32, 3, 4, 0, {2, 2, 2, 2}}, {32, 3, 4, 1, {1, 1, 2, 2}}, {32, 3, 4, 2, {1, 1, 2, 2}}, {32, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CANNY_FULL_ITERATION[] = {{36, 3, 4, 0, {2, 2, 2, 2}}, {36, 3, 4, 1, {1, 1, 2, 2}}, {36, 3, 4, 2, {1, 1, 2, 2}}, {36, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CANNY_FULL_ITERATION[] = {{40, 3, 4, 0, {2, 2, 2, 2}}, {40, 3, 4, 1, {1, 1, 2, 2}}, {40, 3, 4, 2, {1, 1, 2, 2}}, {40, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CANNY_FULL_ITERATION[] = {{44, 3, 4, 0, {2, 2, 2, 2}}, {44, 3, 4, 1, {1, 1, 2, 2}}, {44, 3, 4, 2, {1, 1, 2, 2}}, {44, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CANNY_FULL_ITERATION[] = {{48, 3, 4, 0, {2, 2, 2, 2}}, {48, 3, 4, 1, {1, 1, 2, 2}}, {48, 3, 4, 2, {1, 1, 2, 2}}, {48, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CANNY_FULL_ITERATION[] = {{52, 3, 4, 0, {2, 2, 2, 2}}, {52, 3, 4, 1, {1, 1, 2, 2}}, {52, 3, 4, 2, {1, 1, 2, 2}}, {52, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CANNY_FULL_ITERATION[] = {{56, 3, 4, 0, {2, 2, 2, 2}}, {56, 3, 4, 1, {1, 1, 2, 2}}, {56, 3, 4, 2, {1, 1, 2, 2}}, {56, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CANNY_FULL_ITERATION[] = {{60, 3, 4, 0, {2, 2, 2, 2}}, {60, 3, 4, 1, {1, 1, 2, 2}}, {60, 3, 4, 2, {1, 1, 2, 2}}, {60, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CANNY_FULL_ITERATION[] = {{64, 3, 4, 0, {2, 2, 2, 2}}, {64, 3, 4, 1, {1, 1, 2, 2}}, {64, 3, 4, 2, {1, 1, 2, 2}}, {64, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CANNY_FULL_ITERATION[] = {{72, 3, 4, 0, {2, 2, 2, 2}}, {72, 3, 4, 1, {1, 1, 2, 2}}, {72, 3, 4, 2, {1, 1, 2, 2}}, {72, 3, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CANNY_FULL_ITERATION[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 4, 1, {1, 1, 2, 2}}, {4, 4, 4, 2, {1, 1, 2, 2}}, {4, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CANNY_FULL_ITERATION[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 4, 1, {1, 1, 2, 2}}, {8, 4, 4, 2, {1, 1, 2, 2}}, {8, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CANNY_FULL_ITERATION[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 4, 1, {1, 1, 2, 2}}, {12, 4, 4, 2, {1, 1, 2, 2}}, {12, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CANNY_FULL_ITERATION[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 4, 1, {1, 1, 2, 2}}, {16, 4, 4, 2, {1, 1, 2, 2}}, {16, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CANNY_FULL_ITERATION[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 4, 1, {1, 1, 2, 2}}, {20, 4, 4, 2, {1, 1, 2, 2}}, {20, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CANNY_FULL_ITERATION[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 4, 1, {1, 1, 2, 2}}, {24, 4, 4, 2, {1, 1, 2, 2}}, {24, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CANNY_FULL_ITERATION[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 4, 1, {1, 1, 2, 2}}, {28, 4, 4, 2, {1, 1, 2, 2}}, {28, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CANNY_FULL_ITERATION[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 4, 1, {1, 1, 2, 2}}, {32, 4, 4, 2, {1, 1, 2, 2}}, {32, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CANNY_FULL_ITERATION[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 4, 1, {1, 1, 2, 2}}, {36, 4, 4, 2, {1, 1, 2, 2}}, {36, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CANNY_FULL_ITERATION[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 4, 1, {1, 1, 2, 2}}, {40, 4, 4, 2, {1, 1, 2, 2}}, {40, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CANNY_FULL_ITERATION[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 4, 1, {1, 1, 2, 2}}, {44, 4, 4, 2, {1, 1, 2, 2}}, {44, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CANNY_FULL_ITERATION[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 4, 1, {1, 1, 2, 2}}, {48, 4, 4, 2, {1, 1, 2, 2}}, {48, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CANNY_FULL_ITERATION[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 4, 1, {1, 1, 2, 2}}, {52, 4, 4, 2, {1, 1, 2, 2}}, {52, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CANNY_FULL_ITERATION[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 4, 1, {1, 1, 2, 2}}, {56, 4, 4, 2, {1, 1, 2, 2}}, {56, 4, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CANNY_FULL_ITERATION[] = {{4, 5, 4, 0, {2, 2, 2, 2}}, {4, 5, 4, 1, {1, 1, 2, 2}}, {4, 5, 4, 2, {1, 1, 2, 2}}, {4, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CANNY_FULL_ITERATION[] = {{8, 5, 4, 0, {2, 2, 2, 2}}, {8, 5, 4, 1, {1, 1, 2, 2}}, {8, 5, 4, 2, {1, 1, 2, 2}}, {8, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CANNY_FULL_ITERATION[] = {{12, 5, 4, 0, {2, 2, 2, 2}}, {12, 5, 4, 1, {1, 1, 2, 2}}, {12, 5, 4, 2, {1, 1, 2, 2}}, {12, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CANNY_FULL_ITERATION[] = {{16, 5, 4, 0, {2, 2, 2, 2}}, {16, 5, 4, 1, {1, 1, 2, 2}}, {16, 5, 4, 2, {1, 1, 2, 2}}, {16, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CANNY_FULL_ITERATION[] = {{20, 5, 4, 0, {2, 2, 2, 2}}, {20, 5, 4, 1, {1, 1, 2, 2}}, {20, 5, 4, 2, {1, 1, 2, 2}}, {20, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CANNY_FULL_ITERATION[] = {{24, 5, 4, 0, {2, 2, 2, 2}}, {24, 5, 4, 1, {1, 1, 2, 2}}, {24, 5, 4, 2, {1, 1, 2, 2}}, {24, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CANNY_FULL_ITERATION[] = {{28, 5, 4, 0, {2, 2, 2, 2}}, {28, 5, 4, 1, {1, 1, 2, 2}}, {28, 5, 4, 2, {1, 1, 2, 2}}, {28, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CANNY_FULL_ITERATION[] = {{32, 5, 4, 0, {2, 2, 2, 2}}, {32, 5, 4, 1, {1, 1, 2, 2}}, {32, 5, 4, 2, {1, 1, 2, 2}}, {32, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CANNY_FULL_ITERATION[] = {{36, 5, 4, 0, {2, 2, 2, 2}}, {36, 5, 4, 1, {1, 1, 2, 2}}, {36, 5, 4, 2, {1, 1, 2, 2}}, {36, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CANNY_FULL_ITERATION[] = {{40, 5, 4, 0, {2, 2, 2, 2}}, {40, 5, 4, 1, {1, 1, 2, 2}}, {40, 5, 4, 2, {1, 1, 2, 2}}, {40, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CANNY_FULL_ITERATION[] = {{44, 5, 4, 0, {2, 2, 2, 2}}, {44, 5, 4, 1, {1, 1, 2, 2}}, {44, 5, 4, 2, {1, 1, 2, 2}}, {44, 5, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CANNY_FULL_ITERATION[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 4, 1, {1, 1, 2, 2}}, {4, 6, 4, 2, {1, 1, 2, 2}}, {4, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CANNY_FULL_ITERATION[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 4, 1, {1, 1, 2, 2}}, {8, 6, 4, 2, {1, 1, 2, 2}}, {8, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CANNY_FULL_ITERATION[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 4, 1, {1, 1, 2, 2}}, {12, 6, 4, 2, {1, 1, 2, 2}}, {12, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CANNY_FULL_ITERATION[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 4, 1, {1, 1, 2, 2}}, {16, 6, 4, 2, {1, 1, 2, 2}}, {16, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CANNY_FULL_ITERATION[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 4, 1, {1, 1, 2, 2}}, {20, 6, 4, 2, {1, 1, 2, 2}}, {20, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CANNY_FULL_ITERATION[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 4, 1, {1, 1, 2, 2}}, {24, 6, 4, 2, {1, 1, 2, 2}}, {24, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CANNY_FULL_ITERATION[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 4, 1, {1, 1, 2, 2}}, {28, 6, 4, 2, {1, 1, 2, 2}}, {28, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CANNY_FULL_ITERATION[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 4, 1, {1, 1, 2, 2}}, {32, 6, 4, 2, {1, 1, 2, 2}}, {32, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CANNY_FULL_ITERATION[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 4, 1, {1, 1, 2, 2}}, {36, 6, 4, 2, {1, 1, 2, 2}}, {36, 6, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CANNY_FULL_ITERATION[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 4, 1, {1, 1, 2, 2}}, {4, 8, 4, 2, {1, 1, 2, 2}}, {4, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CANNY_FULL_ITERATION[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 4, 1, {1, 1, 2, 2}}, {8, 8, 4, 2, {1, 1, 2, 2}}, {8, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CANNY_FULL_ITERATION[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 4, 1, {1, 1, 2, 2}}, {12, 8, 4, 2, {1, 1, 2, 2}}, {12, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CANNY_FULL_ITERATION[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 4, 1, {1, 1, 2, 2}}, {16, 8, 4, 2, {1, 1, 2, 2}}, {16, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CANNY_FULL_ITERATION[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 4, 1, {1, 1, 2, 2}}, {20, 8, 4, 2, {1, 1, 2, 2}}, {20, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CANNY_FULL_ITERATION[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 4, 1, {1, 1, 2, 2}}, {24, 8, 4, 2, {1, 1, 2, 2}}, {24, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CANNY_FULL_ITERATION[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 4, 1, {1, 1, 2, 2}}, {28, 8, 4, 2, {1, 1, 2, 2}}, {28, 8, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CANNY_FULL_ITERATION[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 4, 1, {1, 1, 2, 2}}, {4, 10, 4, 2, {1, 1, 2, 2}}, {4, 10, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CANNY_FULL_ITERATION[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 4, 1, {1, 1, 2, 2}}, {8, 10, 4, 2, {1, 1, 2, 2}}, {8, 10, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_CANNY_FULL_ITERATION[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 4, 1, {1, 1, 2, 2}}, {12, 10, 4, 2, {1, 1, 2, 2}}, {12, 10, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_CANNY_FULL_ITERATION[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 4, 1, {1, 1, 2, 2}}, {16, 10, 4, 2, {1, 1, 2, 2}}, {16, 10, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_CANNY_FULL_ITERATION[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 4, 1, {1, 1, 2, 2}}, {20, 10, 4, 2, {1, 1, 2, 2}}, {20, 10, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_CANNY_FULL_ITERATION[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 4, 1, {1, 1, 2, 2}}, {4, 12, 4, 2, {1, 1, 2, 2}}, {4, 12, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_CANNY_FULL_ITERATION[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 4, 1, {1, 1, 2, 2}}, {8, 12, 4, 2, {1, 1, 2, 2}}, {8, 12, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_CANNY_FULL_ITERATION[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 4, 1, {1, 1, 2, 2}}, {12, 12, 4, 2, {1, 1, 2, 2}}, {12, 12, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_CANNY_FULL_ITERATION[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 4, 1, {1, 1, 2, 2}}, {16, 12, 4, 2, {1, 1, 2, 2}}, {16, 12, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_CANNY_FULL_ITERATION[] = {{4, 14, 4, 0, {2, 2, 2, 2}}, {4, 14, 4, 1, {1, 1, 2, 2}}, {4, 14, 4, 2, {1, 1, 2, 2}}, {4, 14, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_CANNY_FULL_ITERATION[] = {{8, 14, 4, 0, {2, 2, 2, 2}}, {8, 14, 4, 1, {1, 1, 2, 2}}, {8, 14, 4, 2, {1, 1, 2, 2}}, {8, 14, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_CANNY_FULL_ITERATION[] = {{12, 14, 4, 0, {2, 2, 2, 2}}, {12, 14, 4, 1, {1, 1, 2, 2}}, {12, 14, 4, 2, {1, 1, 2, 2}}, {12, 14, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_CANNY_FULL_ITERATION[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 4, 1, {1, 1, 2, 2}}, {4, 16, 4, 2, {1, 1, 2, 2}}, {4, 16, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_CANNY_FULL_ITERATION[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 4, 1, {1, 1, 2, 2}}, {8, 16, 4, 2, {1, 1, 2, 2}}, {8, 16, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_CANNY_FULL_ITERATION[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 4, 1, {1, 1, 2, 2}}, {12, 16, 4, 2, {1, 1, 2, 2}}, {12, 16, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_CANNY_FULL_ITERATION[] = {{4, 18, 4, 0, {2, 2, 2, 2}}, {4, 18, 4, 1, {1, 1, 2, 2}}, {4, 18, 4, 2, {1, 1, 2, 2}}, {4, 18, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_CANNY_FULL_ITERATION[] = {{8, 18, 4, 0, {2, 2, 2, 2}}, {8, 18, 4, 1, {1, 1, 2, 2}}, {8, 18, 4, 2, {1, 1, 2, 2}}, {8, 18, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_CANNY_FULL_ITERATION[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 4, 1, {1, 1, 2, 2}}, {4, 20, 4, 2, {1, 1, 2, 2}}, {4, 20, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_CANNY_FULL_ITERATION[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 4, 1, {1, 1, 2, 2}}, {8, 20, 4, 2, {1, 1, 2, 2}}, {8, 20, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_CANNY_FULL_ITERATION[] = {{4, 22, 4, 0, {2, 2, 2, 2}}, {4, 22, 4, 1, {1, 1, 2, 2}}, {4, 22, 4, 2, {1, 1, 2, 2}}, {4, 22, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_CANNY_FULL_ITERATION[] = {{8, 22, 4, 0, {2, 2, 2, 2}}, {8, 22, 4, 1, {1, 1, 2, 2}}, {8, 22, 4, 2, {1, 1, 2, 2}}, {8, 22, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_CANNY_FULL_ITERATION[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 4, 1, {1, 1, 2, 2}}, {4, 24, 4, 2, {1, 1, 2, 2}}, {4, 24, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_CANNY_FULL_ITERATION[] = {{4, 26, 4, 0, {2, 2, 2, 2}}, {4, 26, 4, 1, {1, 1, 2, 2}}, {4, 26, 4, 2, {1, 1, 2, 2}}, {4, 26, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_CANNY_FULL_ITERATION[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 4, 1, {1, 1, 2, 2}}, {4, 28, 4, 2, {1, 1, 2, 2}}, {4, 28, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_CANNY_FULL_ITERATION[] = {{4, 30, 4, 0, {2, 2, 2, 2}}, {4, 30, 4, 1, {1, 1, 2, 2}}, {4, 30, 4, 2, {1, 1, 2, 2}}, {4, 30, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_CANNY_FULL_ITERATION[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 4, 1, {1, 1, 2, 2}}, {4, 32, 4, 2, {1, 1, 2, 2}}, {4, 32, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CANNY_FULL_ITERATION[] = {{2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_CANNY_FULL_ITERATION[] = {{1, 0}, {2, 0}, {3, 0}};

static acf_scenario gScenarioArray_CANNY_FULL_ITERATION[] = {
{1, 1, 192, 32, 4, gScenarioBufferData0_CANNY_FULL_ITERATION, 96, gScenarioKernelData0_CANNY_FULL_ITERATION, 12},
{2, 1, 292, 32, 4, gScenarioBufferData1_CANNY_FULL_ITERATION, 96, gScenarioKernelData1_CANNY_FULL_ITERATION, 12},
{3, 1, 392, 32, 4, gScenarioBufferData2_CANNY_FULL_ITERATION, 96, gScenarioKernelData2_CANNY_FULL_ITERATION, 12},
{4, 1, 492, 32, 4, gScenarioBufferData3_CANNY_FULL_ITERATION, 96, gScenarioKernelData3_CANNY_FULL_ITERATION, 12},
{5, 1, 592, 32, 4, gScenarioBufferData4_CANNY_FULL_ITERATION, 96, gScenarioKernelData4_CANNY_FULL_ITERATION, 12},
{6, 1, 692, 32, 4, gScenarioBufferData5_CANNY_FULL_ITERATION, 96, gScenarioKernelData5_CANNY_FULL_ITERATION, 12},
{7, 1, 792, 32, 4, gScenarioBufferData6_CANNY_FULL_ITERATION, 96, gScenarioKernelData6_CANNY_FULL_ITERATION, 12},
{8, 1, 892, 32, 4, gScenarioBufferData7_CANNY_FULL_ITERATION, 96, gScenarioKernelData7_CANNY_FULL_ITERATION, 12},
{9, 1, 992, 32, 4, gScenarioBufferData8_CANNY_FULL_ITERATION, 96, gScenarioKernelData8_CANNY_FULL_ITERATION, 12},
{10, 1, 1092, 32, 4, gScenarioBufferData9_CANNY_FULL_ITERATION, 96, gScenarioKernelData9_CANNY_FULL_ITERATION, 12},
{11, 1, 1192, 32, 4, gScenarioBufferData10_CANNY_FULL_ITERATION, 96, gScenarioKernelData10_CANNY_FULL_ITERATION, 12},
{12, 1, 1292, 32, 4, gScenarioBufferData11_CANNY_FULL_ITERATION, 96, gScenarioKernelData11_CANNY_FULL_ITERATION, 12},
{13, 1, 1392, 32, 4, gScenarioBufferData12_CANNY_FULL_ITERATION, 96, gScenarioKernelData12_CANNY_FULL_ITERATION, 12},
{14, 1, 1492, 32, 4, gScenarioBufferData13_CANNY_FULL_ITERATION, 96, gScenarioKernelData13_CANNY_FULL_ITERATION, 12},
{15, 1, 1592, 32, 4, gScenarioBufferData14_CANNY_FULL_ITERATION, 96, gScenarioKernelData14_CANNY_FULL_ITERATION, 12},
{16, 1, 1692, 32, 4, gScenarioBufferData15_CANNY_FULL_ITERATION, 96, gScenarioKernelData15_CANNY_FULL_ITERATION, 12},
{18, 1, 1892, 32, 4, gScenarioBufferData16_CANNY_FULL_ITERATION, 96, gScenarioKernelData16_CANNY_FULL_ITERATION, 12},
{20, 1, 2092, 32, 4, gScenarioBufferData17_CANNY_FULL_ITERATION, 96, gScenarioKernelData17_CANNY_FULL_ITERATION, 12},
{22, 1, 2292, 32, 4, gScenarioBufferData18_CANNY_FULL_ITERATION, 96, gScenarioKernelData18_CANNY_FULL_ITERATION, 12},
{24, 1, 2492, 32, 4, gScenarioBufferData19_CANNY_FULL_ITERATION, 96, gScenarioKernelData19_CANNY_FULL_ITERATION, 12},
{26, 1, 2692, 32, 4, gScenarioBufferData20_CANNY_FULL_ITERATION, 96, gScenarioKernelData20_CANNY_FULL_ITERATION, 12},
{28, 1, 2892, 32, 4, gScenarioBufferData21_CANNY_FULL_ITERATION, 96, gScenarioKernelData21_CANNY_FULL_ITERATION, 12},
{30, 1, 3092, 32, 4, gScenarioBufferData22_CANNY_FULL_ITERATION, 96, gScenarioKernelData22_CANNY_FULL_ITERATION, 12},
{32, 1, 3292, 32, 4, gScenarioBufferData23_CANNY_FULL_ITERATION, 96, gScenarioKernelData23_CANNY_FULL_ITERATION, 12},
{1, 2, 272, 32, 3, gScenarioBufferData24_CANNY_FULL_ITERATION, 96, gScenarioKernelData24_CANNY_FULL_ITERATION, 12},
{2, 2, 416, 32, 3, gScenarioBufferData25_CANNY_FULL_ITERATION, 96, gScenarioKernelData25_CANNY_FULL_ITERATION, 12},
{3, 2, 560, 32, 3, gScenarioBufferData26_CANNY_FULL_ITERATION, 96, gScenarioKernelData26_CANNY_FULL_ITERATION, 12},
{4, 2, 704, 32, 3, gScenarioBufferData27_CANNY_FULL_ITERATION, 96, gScenarioKernelData27_CANNY_FULL_ITERATION, 12},
{5, 2, 848, 32, 3, gScenarioBufferData28_CANNY_FULL_ITERATION, 96, gScenarioKernelData28_CANNY_FULL_ITERATION, 12},
{6, 2, 992, 32, 3, gScenarioBufferData29_CANNY_FULL_ITERATION, 96, gScenarioKernelData29_CANNY_FULL_ITERATION, 12},
{7, 2, 1136, 32, 3, gScenarioBufferData30_CANNY_FULL_ITERATION, 96, gScenarioKernelData30_CANNY_FULL_ITERATION, 12},
{8, 2, 1280, 32, 3, gScenarioBufferData31_CANNY_FULL_ITERATION, 96, gScenarioKernelData31_CANNY_FULL_ITERATION, 12},
{9, 2, 1424, 32, 3, gScenarioBufferData32_CANNY_FULL_ITERATION, 96, gScenarioKernelData32_CANNY_FULL_ITERATION, 12},
{10, 2, 1568, 32, 3, gScenarioBufferData33_CANNY_FULL_ITERATION, 96, gScenarioKernelData33_CANNY_FULL_ITERATION, 12},
{11, 2, 1712, 32, 3, gScenarioBufferData34_CANNY_FULL_ITERATION, 96, gScenarioKernelData34_CANNY_FULL_ITERATION, 12},
{12, 2, 1856, 32, 3, gScenarioBufferData35_CANNY_FULL_ITERATION, 96, gScenarioKernelData35_CANNY_FULL_ITERATION, 12},
{13, 2, 2000, 32, 3, gScenarioBufferData36_CANNY_FULL_ITERATION, 96, gScenarioKernelData36_CANNY_FULL_ITERATION, 12},
{14, 2, 2144, 32, 3, gScenarioBufferData37_CANNY_FULL_ITERATION, 96, gScenarioKernelData37_CANNY_FULL_ITERATION, 12},
{15, 2, 2288, 32, 3, gScenarioBufferData38_CANNY_FULL_ITERATION, 96, gScenarioKernelData38_CANNY_FULL_ITERATION, 12},
{16, 2, 2432, 32, 3, gScenarioBufferData39_CANNY_FULL_ITERATION, 96, gScenarioKernelData39_CANNY_FULL_ITERATION, 12},
{18, 2, 2720, 32, 3, gScenarioBufferData40_CANNY_FULL_ITERATION, 96, gScenarioKernelData40_CANNY_FULL_ITERATION, 12},
{20, 2, 3008, 32, 3, gScenarioBufferData41_CANNY_FULL_ITERATION, 96, gScenarioKernelData41_CANNY_FULL_ITERATION, 12},
{22, 2, 3296, 32, 3, gScenarioBufferData42_CANNY_FULL_ITERATION, 96, gScenarioKernelData42_CANNY_FULL_ITERATION, 12},
{24, 2, 3584, 32, 3, gScenarioBufferData43_CANNY_FULL_ITERATION, 96, gScenarioKernelData43_CANNY_FULL_ITERATION, 12},
{1, 3, 376, 32, 3, gScenarioBufferData44_CANNY_FULL_ITERATION, 96, gScenarioKernelData44_CANNY_FULL_ITERATION, 12},
{2, 3, 576, 32, 3, gScenarioBufferData45_CANNY_FULL_ITERATION, 96, gScenarioKernelData45_CANNY_FULL_ITERATION, 12},
{3, 3, 776, 32, 3, gScenarioBufferData46_CANNY_FULL_ITERATION, 96, gScenarioKernelData46_CANNY_FULL_ITERATION, 12},
{4, 3, 976, 32, 3, gScenarioBufferData47_CANNY_FULL_ITERATION, 96, gScenarioKernelData47_CANNY_FULL_ITERATION, 12},
{5, 3, 1176, 32, 3, gScenarioBufferData48_CANNY_FULL_ITERATION, 96, gScenarioKernelData48_CANNY_FULL_ITERATION, 12},
{6, 3, 1376, 32, 3, gScenarioBufferData49_CANNY_FULL_ITERATION, 96, gScenarioKernelData49_CANNY_FULL_ITERATION, 12},
{7, 3, 1576, 32, 3, gScenarioBufferData50_CANNY_FULL_ITERATION, 96, gScenarioKernelData50_CANNY_FULL_ITERATION, 12},
{8, 3, 1776, 32, 3, gScenarioBufferData51_CANNY_FULL_ITERATION, 96, gScenarioKernelData51_CANNY_FULL_ITERATION, 12},
{9, 3, 1976, 32, 3, gScenarioBufferData52_CANNY_FULL_ITERATION, 96, gScenarioKernelData52_CANNY_FULL_ITERATION, 12},
{10, 3, 2176, 32, 3, gScenarioBufferData53_CANNY_FULL_ITERATION, 96, gScenarioKernelData53_CANNY_FULL_ITERATION, 12},
{11, 3, 2376, 32, 3, gScenarioBufferData54_CANNY_FULL_ITERATION, 96, gScenarioKernelData54_CANNY_FULL_ITERATION, 12},
{12, 3, 2576, 32, 3, gScenarioBufferData55_CANNY_FULL_ITERATION, 96, gScenarioKernelData55_CANNY_FULL_ITERATION, 12},
{13, 3, 2776, 32, 3, gScenarioBufferData56_CANNY_FULL_ITERATION, 96, gScenarioKernelData56_CANNY_FULL_ITERATION, 12},
{14, 3, 2976, 32, 3, gScenarioBufferData57_CANNY_FULL_ITERATION, 96, gScenarioKernelData57_CANNY_FULL_ITERATION, 12},
{15, 3, 3176, 32, 3, gScenarioBufferData58_CANNY_FULL_ITERATION, 96, gScenarioKernelData58_CANNY_FULL_ITERATION, 12},
{16, 3, 3376, 32, 3, gScenarioBufferData59_CANNY_FULL_ITERATION, 96, gScenarioKernelData59_CANNY_FULL_ITERATION, 12},
{18, 3, 3776, 32, 3, gScenarioBufferData60_CANNY_FULL_ITERATION, 96, gScenarioKernelData60_CANNY_FULL_ITERATION, 12},
{1, 4, 480, 32, 3, gScenarioBufferData61_CANNY_FULL_ITERATION, 96, gScenarioKernelData61_CANNY_FULL_ITERATION, 12},
{2, 4, 736, 32, 3, gScenarioBufferData62_CANNY_FULL_ITERATION, 96, gScenarioKernelData62_CANNY_FULL_ITERATION, 12},
{3, 4, 992, 32, 3, gScenarioBufferData63_CANNY_FULL_ITERATION, 96, gScenarioKernelData63_CANNY_FULL_ITERATION, 12},
{4, 4, 1248, 32, 3, gScenarioBufferData64_CANNY_FULL_ITERATION, 96, gScenarioKernelData64_CANNY_FULL_ITERATION, 12},
{5, 4, 1504, 32, 3, gScenarioBufferData65_CANNY_FULL_ITERATION, 96, gScenarioKernelData65_CANNY_FULL_ITERATION, 12},
{6, 4, 1760, 32, 3, gScenarioBufferData66_CANNY_FULL_ITERATION, 96, gScenarioKernelData66_CANNY_FULL_ITERATION, 12},
{7, 4, 2016, 32, 3, gScenarioBufferData67_CANNY_FULL_ITERATION, 96, gScenarioKernelData67_CANNY_FULL_ITERATION, 12},
{8, 4, 2272, 32, 3, gScenarioBufferData68_CANNY_FULL_ITERATION, 96, gScenarioKernelData68_CANNY_FULL_ITERATION, 12},
{9, 4, 2528, 32, 3, gScenarioBufferData69_CANNY_FULL_ITERATION, 96, gScenarioKernelData69_CANNY_FULL_ITERATION, 12},
{10, 4, 2784, 32, 3, gScenarioBufferData70_CANNY_FULL_ITERATION, 96, gScenarioKernelData70_CANNY_FULL_ITERATION, 12},
{11, 4, 3040, 32, 3, gScenarioBufferData71_CANNY_FULL_ITERATION, 96, gScenarioKernelData71_CANNY_FULL_ITERATION, 12},
{12, 4, 3296, 32, 3, gScenarioBufferData72_CANNY_FULL_ITERATION, 96, gScenarioKernelData72_CANNY_FULL_ITERATION, 12},
{13, 4, 3552, 32, 3, gScenarioBufferData73_CANNY_FULL_ITERATION, 96, gScenarioKernelData73_CANNY_FULL_ITERATION, 12},
{14, 4, 3808, 32, 3, gScenarioBufferData74_CANNY_FULL_ITERATION, 96, gScenarioKernelData74_CANNY_FULL_ITERATION, 12},
{1, 5, 584, 32, 3, gScenarioBufferData75_CANNY_FULL_ITERATION, 96, gScenarioKernelData75_CANNY_FULL_ITERATION, 12},
{2, 5, 896, 32, 3, gScenarioBufferData76_CANNY_FULL_ITERATION, 96, gScenarioKernelData76_CANNY_FULL_ITERATION, 12},
{3, 5, 1208, 32, 3, gScenarioBufferData77_CANNY_FULL_ITERATION, 96, gScenarioKernelData77_CANNY_FULL_ITERATION, 12},
{4, 5, 1520, 32, 3, gScenarioBufferData78_CANNY_FULL_ITERATION, 96, gScenarioKernelData78_CANNY_FULL_ITERATION, 12},
{5, 5, 1832, 32, 3, gScenarioBufferData79_CANNY_FULL_ITERATION, 96, gScenarioKernelData79_CANNY_FULL_ITERATION, 12},
{6, 5, 2144, 32, 3, gScenarioBufferData80_CANNY_FULL_ITERATION, 96, gScenarioKernelData80_CANNY_FULL_ITERATION, 12},
{7, 5, 2456, 32, 3, gScenarioBufferData81_CANNY_FULL_ITERATION, 96, gScenarioKernelData81_CANNY_FULL_ITERATION, 12},
{8, 5, 2768, 32, 3, gScenarioBufferData82_CANNY_FULL_ITERATION, 96, gScenarioKernelData82_CANNY_FULL_ITERATION, 12},
{9, 5, 3080, 32, 3, gScenarioBufferData83_CANNY_FULL_ITERATION, 96, gScenarioKernelData83_CANNY_FULL_ITERATION, 12},
{10, 5, 3392, 32, 3, gScenarioBufferData84_CANNY_FULL_ITERATION, 96, gScenarioKernelData84_CANNY_FULL_ITERATION, 12},
{11, 5, 3704, 32, 3, gScenarioBufferData85_CANNY_FULL_ITERATION, 96, gScenarioKernelData85_CANNY_FULL_ITERATION, 12},
{1, 6, 688, 32, 3, gScenarioBufferData86_CANNY_FULL_ITERATION, 96, gScenarioKernelData86_CANNY_FULL_ITERATION, 12},
{2, 6, 1056, 32, 3, gScenarioBufferData87_CANNY_FULL_ITERATION, 96, gScenarioKernelData87_CANNY_FULL_ITERATION, 12},
{3, 6, 1424, 32, 3, gScenarioBufferData88_CANNY_FULL_ITERATION, 96, gScenarioKernelData88_CANNY_FULL_ITERATION, 12},
{4, 6, 1792, 32, 3, gScenarioBufferData89_CANNY_FULL_ITERATION, 96, gScenarioKernelData89_CANNY_FULL_ITERATION, 12},
{5, 6, 2160, 32, 3, gScenarioBufferData90_CANNY_FULL_ITERATION, 96, gScenarioKernelData90_CANNY_FULL_ITERATION, 12},
{6, 6, 2528, 32, 3, gScenarioBufferData91_CANNY_FULL_ITERATION, 96, gScenarioKernelData91_CANNY_FULL_ITERATION, 12},
{7, 6, 2896, 32, 3, gScenarioBufferData92_CANNY_FULL_ITERATION, 96, gScenarioKernelData92_CANNY_FULL_ITERATION, 12},
{8, 6, 3264, 32, 3, gScenarioBufferData93_CANNY_FULL_ITERATION, 96, gScenarioKernelData93_CANNY_FULL_ITERATION, 12},
{9, 6, 3632, 32, 3, gScenarioBufferData94_CANNY_FULL_ITERATION, 96, gScenarioKernelData94_CANNY_FULL_ITERATION, 12},
{1, 8, 896, 32, 3, gScenarioBufferData95_CANNY_FULL_ITERATION, 96, gScenarioKernelData95_CANNY_FULL_ITERATION, 12},
{2, 8, 1376, 32, 3, gScenarioBufferData96_CANNY_FULL_ITERATION, 96, gScenarioKernelData96_CANNY_FULL_ITERATION, 12},
{3, 8, 1856, 32, 3, gScenarioBufferData97_CANNY_FULL_ITERATION, 96, gScenarioKernelData97_CANNY_FULL_ITERATION, 12},
{4, 8, 2336, 32, 3, gScenarioBufferData98_CANNY_FULL_ITERATION, 96, gScenarioKernelData98_CANNY_FULL_ITERATION, 12},
{5, 8, 2816, 32, 3, gScenarioBufferData99_CANNY_FULL_ITERATION, 96, gScenarioKernelData99_CANNY_FULL_ITERATION, 12},
{6, 8, 3296, 32, 3, gScenarioBufferData100_CANNY_FULL_ITERATION, 96, gScenarioKernelData100_CANNY_FULL_ITERATION, 12},
{7, 8, 3776, 32, 3, gScenarioBufferData101_CANNY_FULL_ITERATION, 96, gScenarioKernelData101_CANNY_FULL_ITERATION, 12},
{1, 10, 1104, 32, 3, gScenarioBufferData102_CANNY_FULL_ITERATION, 96, gScenarioKernelData102_CANNY_FULL_ITERATION, 12},
{2, 10, 1696, 32, 3, gScenarioBufferData103_CANNY_FULL_ITERATION, 96, gScenarioKernelData103_CANNY_FULL_ITERATION, 12},
{3, 10, 2288, 32, 3, gScenarioBufferData104_CANNY_FULL_ITERATION, 96, gScenarioKernelData104_CANNY_FULL_ITERATION, 12},
{4, 10, 2880, 32, 3, gScenarioBufferData105_CANNY_FULL_ITERATION, 96, gScenarioKernelData105_CANNY_FULL_ITERATION, 12},
{5, 10, 3472, 32, 3, gScenarioBufferData106_CANNY_FULL_ITERATION, 96, gScenarioKernelData106_CANNY_FULL_ITERATION, 12},
{1, 12, 1312, 32, 3, gScenarioBufferData107_CANNY_FULL_ITERATION, 96, gScenarioKernelData107_CANNY_FULL_ITERATION, 12},
{2, 12, 2016, 32, 3, gScenarioBufferData108_CANNY_FULL_ITERATION, 96, gScenarioKernelData108_CANNY_FULL_ITERATION, 12},
{3, 12, 2720, 32, 3, gScenarioBufferData109_CANNY_FULL_ITERATION, 96, gScenarioKernelData109_CANNY_FULL_ITERATION, 12},
{4, 12, 3424, 32, 3, gScenarioBufferData110_CANNY_FULL_ITERATION, 96, gScenarioKernelData110_CANNY_FULL_ITERATION, 12},
{1, 14, 1520, 32, 3, gScenarioBufferData111_CANNY_FULL_ITERATION, 96, gScenarioKernelData111_CANNY_FULL_ITERATION, 12},
{2, 14, 2336, 32, 3, gScenarioBufferData112_CANNY_FULL_ITERATION, 96, gScenarioKernelData112_CANNY_FULL_ITERATION, 12},
{3, 14, 3152, 32, 3, gScenarioBufferData113_CANNY_FULL_ITERATION, 96, gScenarioKernelData113_CANNY_FULL_ITERATION, 12},
{1, 16, 1728, 32, 3, gScenarioBufferData114_CANNY_FULL_ITERATION, 96, gScenarioKernelData114_CANNY_FULL_ITERATION, 12},
{2, 16, 2656, 32, 3, gScenarioBufferData115_CANNY_FULL_ITERATION, 96, gScenarioKernelData115_CANNY_FULL_ITERATION, 12},
{3, 16, 3584, 32, 3, gScenarioBufferData116_CANNY_FULL_ITERATION, 96, gScenarioKernelData116_CANNY_FULL_ITERATION, 12},
{1, 18, 1936, 32, 3, gScenarioBufferData117_CANNY_FULL_ITERATION, 96, gScenarioKernelData117_CANNY_FULL_ITERATION, 12},
{2, 18, 2976, 32, 3, gScenarioBufferData118_CANNY_FULL_ITERATION, 96, gScenarioKernelData118_CANNY_FULL_ITERATION, 12},
{1, 20, 2144, 32, 3, gScenarioBufferData119_CANNY_FULL_ITERATION, 96, gScenarioKernelData119_CANNY_FULL_ITERATION, 12},
{2, 20, 3296, 32, 3, gScenarioBufferData120_CANNY_FULL_ITERATION, 96, gScenarioKernelData120_CANNY_FULL_ITERATION, 12},
{1, 22, 2352, 32, 3, gScenarioBufferData121_CANNY_FULL_ITERATION, 96, gScenarioKernelData121_CANNY_FULL_ITERATION, 12},
{2, 22, 3616, 32, 3, gScenarioBufferData122_CANNY_FULL_ITERATION, 96, gScenarioKernelData122_CANNY_FULL_ITERATION, 12},
{1, 24, 2560, 32, 3, gScenarioBufferData123_CANNY_FULL_ITERATION, 96, gScenarioKernelData123_CANNY_FULL_ITERATION, 12},
{1, 26, 2768, 32, 3, gScenarioBufferData124_CANNY_FULL_ITERATION, 96, gScenarioKernelData124_CANNY_FULL_ITERATION, 12},
{1, 28, 2976, 32, 3, gScenarioBufferData125_CANNY_FULL_ITERATION, 96, gScenarioKernelData125_CANNY_FULL_ITERATION, 12},
{1, 30, 3184, 32, 3, gScenarioBufferData126_CANNY_FULL_ITERATION, 96, gScenarioKernelData126_CANNY_FULL_ITERATION, 12},
{1, 32, 3392, 32, 3, gScenarioBufferData127_CANNY_FULL_ITERATION, 96, gScenarioKernelData127_CANNY_FULL_ITERATION, 12}
};

static acf_scenario_list gScenarioList_CANNY_FULL_ITERATION = {
128, //number of scenarios
gScenarioArray_CANNY_FULL_ITERATION};
//**************************************************************

class CANNY_FULL_ITERATION : public ACF_Process_APU
{

public:
   CANNY_FULL_ITERATION(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CANNY_FULL_ITERATION");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CANNY_FULL_ITERATION",
                                        CANNY_FULL_ITERATION_LOAD_SEGMENTS,
                                        CANNY_FULL_ITERATION_LOAD_PMEM, CANNY_FULL_ITERATION_LOAD_PMEM_SIZE,
                                        CANNY_FULL_ITERATION_LOAD_DMEM, CANNY_FULL_ITERATION_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CANNY_FULL_ITERATION_APEX_LOG_BUFFER, CANNY_FULL_ITERATION_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0);
         AddPort("LOW_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("HIGH_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("EDGE_OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CANNY_FULL_ITERATION);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CANNY_FULL_ITERATION

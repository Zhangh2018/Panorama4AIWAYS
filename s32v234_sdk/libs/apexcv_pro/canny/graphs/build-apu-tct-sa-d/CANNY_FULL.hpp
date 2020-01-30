#ifndef _ACF_PROCESS_APU_CANNY_FULL
#define _ACF_PROCESS_APU_CANNY_FULL

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CANNY_FULL_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CANNY_FULL[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 4, 1, {1, 1, 2, 2}}, {4, 2, 2, 2, {0, 0, 0, 0}}, {4, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CANNY_FULL[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 4, 1, {1, 1, 2, 2}}, {8, 2, 2, 2, {0, 0, 0, 0}}, {8, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CANNY_FULL[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 4, 1, {1, 1, 2, 2}}, {12, 2, 2, 2, {0, 0, 0, 0}}, {12, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CANNY_FULL[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 4, 1, {1, 1, 2, 2}}, {16, 2, 2, 2, {0, 0, 0, 0}}, {16, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CANNY_FULL[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 4, 1, {1, 1, 2, 2}}, {20, 2, 2, 2, {0, 0, 0, 0}}, {20, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CANNY_FULL[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 4, 1, {1, 1, 2, 2}}, {24, 2, 2, 2, {0, 0, 0, 0}}, {24, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CANNY_FULL[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 4, 1, {1, 1, 2, 2}}, {28, 2, 2, 2, {0, 0, 0, 0}}, {28, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CANNY_FULL[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 4, 1, {1, 1, 2, 2}}, {32, 2, 2, 2, {0, 0, 0, 0}}, {32, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CANNY_FULL[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 4, 1, {1, 1, 2, 2}}, {36, 2, 2, 2, {0, 0, 0, 0}}, {36, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CANNY_FULL[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 4, 1, {1, 1, 2, 2}}, {40, 2, 2, 2, {0, 0, 0, 0}}, {40, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CANNY_FULL[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 4, 1, {1, 1, 2, 2}}, {44, 2, 2, 2, {0, 0, 0, 0}}, {44, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CANNY_FULL[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 4, 1, {1, 1, 2, 2}}, {48, 2, 2, 2, {0, 0, 0, 0}}, {48, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CANNY_FULL[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 4, 1, {1, 1, 2, 2}}, {52, 2, 2, 2, {0, 0, 0, 0}}, {52, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CANNY_FULL[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 4, 1, {1, 1, 2, 2}}, {56, 2, 2, 2, {0, 0, 0, 0}}, {56, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CANNY_FULL[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 4, 1, {1, 1, 2, 2}}, {60, 2, 2, 2, {0, 0, 0, 0}}, {60, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CANNY_FULL[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 4, 1, {1, 1, 2, 2}}, {64, 2, 2, 2, {0, 0, 0, 0}}, {64, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CANNY_FULL[] = {{72, 2, 4, 0, {2, 2, 2, 2}}, {72, 2, 4, 1, {1, 1, 2, 2}}, {72, 2, 2, 2, {0, 0, 0, 0}}, {72, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CANNY_FULL[] = {{80, 2, 4, 0, {2, 2, 2, 2}}, {80, 2, 4, 1, {1, 1, 2, 2}}, {80, 2, 2, 2, {0, 0, 0, 0}}, {80, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CANNY_FULL[] = {{88, 2, 4, 0, {2, 2, 2, 2}}, {88, 2, 4, 1, {1, 1, 2, 2}}, {88, 2, 2, 2, {0, 0, 0, 0}}, {88, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CANNY_FULL[] = {{96, 2, 4, 0, {2, 2, 2, 2}}, {96, 2, 4, 1, {1, 1, 2, 2}}, {96, 2, 2, 2, {0, 0, 0, 0}}, {96, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CANNY_FULL[] = {{104, 2, 4, 0, {2, 2, 2, 2}}, {104, 2, 4, 1, {1, 1, 2, 2}}, {104, 2, 2, 2, {0, 0, 0, 0}}, {104, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CANNY_FULL[] = {{112, 2, 4, 0, {2, 2, 2, 2}}, {112, 2, 4, 1, {1, 1, 2, 2}}, {112, 2, 2, 2, {0, 0, 0, 0}}, {112, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CANNY_FULL[] = {{120, 2, 4, 0, {2, 2, 2, 2}}, {120, 2, 4, 1, {1, 1, 2, 2}}, {120, 2, 2, 2, {0, 0, 0, 0}}, {120, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CANNY_FULL[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 4, 1, {1, 1, 2, 2}}, {4, 4, 2, 2, {0, 0, 0, 0}}, {4, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CANNY_FULL[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 4, 1, {1, 1, 2, 2}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CANNY_FULL[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 4, 1, {1, 1, 2, 2}}, {12, 4, 2, 2, {0, 0, 0, 0}}, {12, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CANNY_FULL[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 4, 1, {1, 1, 2, 2}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CANNY_FULL[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 4, 1, {1, 1, 2, 2}}, {20, 4, 2, 2, {0, 0, 0, 0}}, {20, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CANNY_FULL[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 4, 1, {1, 1, 2, 2}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CANNY_FULL[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 4, 1, {1, 1, 2, 2}}, {28, 4, 2, 2, {0, 0, 0, 0}}, {28, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CANNY_FULL[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 4, 1, {1, 1, 2, 2}}, {32, 4, 2, 2, {0, 0, 0, 0}}, {32, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CANNY_FULL[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 4, 1, {1, 1, 2, 2}}, {36, 4, 2, 2, {0, 0, 0, 0}}, {36, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CANNY_FULL[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 4, 1, {1, 1, 2, 2}}, {40, 4, 2, 2, {0, 0, 0, 0}}, {40, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CANNY_FULL[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 4, 1, {1, 1, 2, 2}}, {44, 4, 2, 2, {0, 0, 0, 0}}, {44, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CANNY_FULL[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 4, 1, {1, 1, 2, 2}}, {48, 4, 2, 2, {0, 0, 0, 0}}, {48, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CANNY_FULL[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 4, 1, {1, 1, 2, 2}}, {52, 4, 2, 2, {0, 0, 0, 0}}, {52, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CANNY_FULL[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 4, 1, {1, 1, 2, 2}}, {56, 4, 2, 2, {0, 0, 0, 0}}, {56, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CANNY_FULL[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 4, 1, {1, 1, 2, 2}}, {60, 4, 2, 2, {0, 0, 0, 0}}, {60, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CANNY_FULL[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 4, 1, {1, 1, 2, 2}}, {64, 4, 2, 2, {0, 0, 0, 0}}, {64, 4, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CANNY_FULL[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 4, 1, {1, 1, 2, 2}}, {4, 6, 2, 2, {0, 0, 0, 0}}, {4, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CANNY_FULL[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 4, 1, {1, 1, 2, 2}}, {8, 6, 2, 2, {0, 0, 0, 0}}, {8, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CANNY_FULL[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 4, 1, {1, 1, 2, 2}}, {12, 6, 2, 2, {0, 0, 0, 0}}, {12, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CANNY_FULL[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 4, 1, {1, 1, 2, 2}}, {16, 6, 2, 2, {0, 0, 0, 0}}, {16, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CANNY_FULL[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 4, 1, {1, 1, 2, 2}}, {20, 6, 2, 2, {0, 0, 0, 0}}, {20, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CANNY_FULL[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 4, 1, {1, 1, 2, 2}}, {24, 6, 2, 2, {0, 0, 0, 0}}, {24, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CANNY_FULL[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 4, 1, {1, 1, 2, 2}}, {28, 6, 2, 2, {0, 0, 0, 0}}, {28, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CANNY_FULL[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 4, 1, {1, 1, 2, 2}}, {32, 6, 2, 2, {0, 0, 0, 0}}, {32, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CANNY_FULL[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 4, 1, {1, 1, 2, 2}}, {36, 6, 2, 2, {0, 0, 0, 0}}, {36, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CANNY_FULL[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 4, 1, {1, 1, 2, 2}}, {40, 6, 2, 2, {0, 0, 0, 0}}, {40, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CANNY_FULL[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 4, 1, {1, 1, 2, 2}}, {44, 6, 2, 2, {0, 0, 0, 0}}, {44, 6, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CANNY_FULL[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 4, 1, {1, 1, 2, 2}}, {4, 8, 2, 2, {0, 0, 0, 0}}, {4, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CANNY_FULL[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 4, 1, {1, 1, 2, 2}}, {8, 8, 2, 2, {0, 0, 0, 0}}, {8, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CANNY_FULL[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 4, 1, {1, 1, 2, 2}}, {12, 8, 2, 2, {0, 0, 0, 0}}, {12, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CANNY_FULL[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 4, 1, {1, 1, 2, 2}}, {16, 8, 2, 2, {0, 0, 0, 0}}, {16, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CANNY_FULL[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 4, 1, {1, 1, 2, 2}}, {20, 8, 2, 2, {0, 0, 0, 0}}, {20, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CANNY_FULL[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 4, 1, {1, 1, 2, 2}}, {24, 8, 2, 2, {0, 0, 0, 0}}, {24, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CANNY_FULL[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 4, 1, {1, 1, 2, 2}}, {28, 8, 2, 2, {0, 0, 0, 0}}, {28, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CANNY_FULL[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 4, 1, {1, 1, 2, 2}}, {32, 8, 2, 2, {0, 0, 0, 0}}, {32, 8, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CANNY_FULL[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 4, 1, {1, 1, 2, 2}}, {4, 10, 2, 2, {0, 0, 0, 0}}, {4, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CANNY_FULL[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 4, 1, {1, 1, 2, 2}}, {8, 10, 2, 2, {0, 0, 0, 0}}, {8, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CANNY_FULL[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 4, 1, {1, 1, 2, 2}}, {12, 10, 2, 2, {0, 0, 0, 0}}, {12, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CANNY_FULL[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 4, 1, {1, 1, 2, 2}}, {16, 10, 2, 2, {0, 0, 0, 0}}, {16, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CANNY_FULL[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 4, 1, {1, 1, 2, 2}}, {20, 10, 2, 2, {0, 0, 0, 0}}, {20, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CANNY_FULL[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 4, 1, {1, 1, 2, 2}}, {24, 10, 2, 2, {0, 0, 0, 0}}, {24, 10, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CANNY_FULL[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 4, 1, {1, 1, 2, 2}}, {4, 12, 2, 2, {0, 0, 0, 0}}, {4, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CANNY_FULL[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 4, 1, {1, 1, 2, 2}}, {8, 12, 2, 2, {0, 0, 0, 0}}, {8, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CANNY_FULL[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 4, 1, {1, 1, 2, 2}}, {12, 12, 2, 2, {0, 0, 0, 0}}, {12, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CANNY_FULL[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 4, 1, {1, 1, 2, 2}}, {16, 12, 2, 2, {0, 0, 0, 0}}, {16, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CANNY_FULL[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 4, 1, {1, 1, 2, 2}}, {20, 12, 2, 2, {0, 0, 0, 0}}, {20, 12, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CANNY_FULL[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 4, 1, {1, 1, 2, 2}}, {4, 16, 2, 2, {0, 0, 0, 0}}, {4, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CANNY_FULL[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 4, 1, {1, 1, 2, 2}}, {8, 16, 2, 2, {0, 0, 0, 0}}, {8, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CANNY_FULL[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 4, 1, {1, 1, 2, 2}}, {12, 16, 2, 2, {0, 0, 0, 0}}, {12, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CANNY_FULL[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 4, 1, {1, 1, 2, 2}}, {16, 16, 2, 2, {0, 0, 0, 0}}, {16, 16, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CANNY_FULL[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 4, 1, {1, 1, 2, 2}}, {4, 20, 2, 2, {0, 0, 0, 0}}, {4, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CANNY_FULL[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 4, 1, {1, 1, 2, 2}}, {8, 20, 2, 2, {0, 0, 0, 0}}, {8, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CANNY_FULL[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 4, 1, {1, 1, 2, 2}}, {12, 20, 2, 2, {0, 0, 0, 0}}, {12, 20, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CANNY_FULL[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 4, 1, {1, 1, 2, 2}}, {4, 24, 2, 2, {0, 0, 0, 0}}, {4, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CANNY_FULL[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 4, 1, {1, 1, 2, 2}}, {8, 24, 2, 2, {0, 0, 0, 0}}, {8, 24, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CANNY_FULL[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 4, 1, {1, 1, 2, 2}}, {4, 28, 2, 2, {0, 0, 0, 0}}, {4, 28, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CANNY_FULL[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 4, 1, {1, 1, 2, 2}}, {8, 28, 2, 2, {0, 0, 0, 0}}, {8, 28, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CANNY_FULL[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 4, 1, {1, 1, 2, 2}}, {4, 32, 2, 2, {0, 0, 0, 0}}, {4, 32, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CANNY_FULL[] = {{4, 36, 4, 0, {2, 2, 2, 2}}, {4, 36, 4, 1, {1, 1, 2, 2}}, {4, 36, 2, 2, {0, 0, 0, 0}}, {4, 36, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CANNY_FULL[] = {{4, 40, 4, 0, {2, 2, 2, 2}}, {4, 40, 4, 1, {1, 1, 2, 2}}, {4, 40, 2, 2, {0, 0, 0, 0}}, {4, 40, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CANNY_FULL[] = {{4, 44, 4, 0, {2, 2, 2, 2}}, {4, 44, 4, 1, {1, 1, 2, 2}}, {4, 44, 2, 2, {0, 0, 0, 0}}, {4, 44, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CANNY_FULL[] = {{1, 0}, {2, 0}, {2, 0}};

static acf_scenario gScenarioArray_CANNY_FULL[] = {
{1, 1, 208, 32, 2, gScenarioBufferData0_CANNY_FULL, 96, gScenarioKernelData0_CANNY_FULL, 12},
{2, 1, 328, 32, 2, gScenarioBufferData1_CANNY_FULL, 96, gScenarioKernelData1_CANNY_FULL, 12},
{3, 1, 448, 32, 2, gScenarioBufferData2_CANNY_FULL, 96, gScenarioKernelData2_CANNY_FULL, 12},
{4, 1, 568, 32, 2, gScenarioBufferData3_CANNY_FULL, 96, gScenarioKernelData3_CANNY_FULL, 12},
{5, 1, 688, 32, 2, gScenarioBufferData4_CANNY_FULL, 96, gScenarioKernelData4_CANNY_FULL, 12},
{6, 1, 808, 32, 2, gScenarioBufferData5_CANNY_FULL, 96, gScenarioKernelData5_CANNY_FULL, 12},
{7, 1, 928, 32, 2, gScenarioBufferData6_CANNY_FULL, 96, gScenarioKernelData6_CANNY_FULL, 12},
{8, 1, 1048, 32, 2, gScenarioBufferData7_CANNY_FULL, 96, gScenarioKernelData7_CANNY_FULL, 12},
{9, 1, 1168, 32, 2, gScenarioBufferData8_CANNY_FULL, 96, gScenarioKernelData8_CANNY_FULL, 12},
{10, 1, 1288, 32, 2, gScenarioBufferData9_CANNY_FULL, 96, gScenarioKernelData9_CANNY_FULL, 12},
{11, 1, 1408, 32, 2, gScenarioBufferData10_CANNY_FULL, 96, gScenarioKernelData10_CANNY_FULL, 12},
{12, 1, 1528, 32, 2, gScenarioBufferData11_CANNY_FULL, 96, gScenarioKernelData11_CANNY_FULL, 12},
{13, 1, 1648, 32, 2, gScenarioBufferData12_CANNY_FULL, 96, gScenarioKernelData12_CANNY_FULL, 12},
{14, 1, 1768, 32, 2, gScenarioBufferData13_CANNY_FULL, 96, gScenarioKernelData13_CANNY_FULL, 12},
{15, 1, 1888, 32, 2, gScenarioBufferData14_CANNY_FULL, 96, gScenarioKernelData14_CANNY_FULL, 12},
{16, 1, 2008, 32, 2, gScenarioBufferData15_CANNY_FULL, 96, gScenarioKernelData15_CANNY_FULL, 12},
{18, 1, 2248, 32, 2, gScenarioBufferData16_CANNY_FULL, 96, gScenarioKernelData16_CANNY_FULL, 12},
{20, 1, 2488, 32, 2, gScenarioBufferData17_CANNY_FULL, 96, gScenarioKernelData17_CANNY_FULL, 12},
{22, 1, 2728, 32, 2, gScenarioBufferData18_CANNY_FULL, 96, gScenarioKernelData18_CANNY_FULL, 12},
{24, 1, 2968, 32, 2, gScenarioBufferData19_CANNY_FULL, 96, gScenarioKernelData19_CANNY_FULL, 12},
{26, 1, 3208, 32, 2, gScenarioBufferData20_CANNY_FULL, 96, gScenarioKernelData20_CANNY_FULL, 12},
{28, 1, 3448, 32, 2, gScenarioBufferData21_CANNY_FULL, 96, gScenarioKernelData21_CANNY_FULL, 12},
{30, 1, 3688, 32, 2, gScenarioBufferData22_CANNY_FULL, 96, gScenarioKernelData22_CANNY_FULL, 12},
{1, 2, 368, 32, 2, gScenarioBufferData23_CANNY_FULL, 96, gScenarioKernelData23_CANNY_FULL, 12},
{2, 2, 584, 32, 2, gScenarioBufferData24_CANNY_FULL, 96, gScenarioKernelData24_CANNY_FULL, 12},
{3, 2, 800, 32, 2, gScenarioBufferData25_CANNY_FULL, 96, gScenarioKernelData25_CANNY_FULL, 12},
{4, 2, 1016, 32, 2, gScenarioBufferData26_CANNY_FULL, 96, gScenarioKernelData26_CANNY_FULL, 12},
{5, 2, 1232, 32, 2, gScenarioBufferData27_CANNY_FULL, 96, gScenarioKernelData27_CANNY_FULL, 12},
{6, 2, 1448, 32, 2, gScenarioBufferData28_CANNY_FULL, 96, gScenarioKernelData28_CANNY_FULL, 12},
{7, 2, 1664, 32, 2, gScenarioBufferData29_CANNY_FULL, 96, gScenarioKernelData29_CANNY_FULL, 12},
{8, 2, 1880, 32, 2, gScenarioBufferData30_CANNY_FULL, 96, gScenarioKernelData30_CANNY_FULL, 12},
{9, 2, 2096, 32, 2, gScenarioBufferData31_CANNY_FULL, 96, gScenarioKernelData31_CANNY_FULL, 12},
{10, 2, 2312, 32, 2, gScenarioBufferData32_CANNY_FULL, 96, gScenarioKernelData32_CANNY_FULL, 12},
{11, 2, 2528, 32, 2, gScenarioBufferData33_CANNY_FULL, 96, gScenarioKernelData33_CANNY_FULL, 12},
{12, 2, 2744, 32, 2, gScenarioBufferData34_CANNY_FULL, 96, gScenarioKernelData34_CANNY_FULL, 12},
{13, 2, 2960, 32, 2, gScenarioBufferData35_CANNY_FULL, 96, gScenarioKernelData35_CANNY_FULL, 12},
{14, 2, 3176, 32, 2, gScenarioBufferData36_CANNY_FULL, 96, gScenarioKernelData36_CANNY_FULL, 12},
{15, 2, 3392, 32, 2, gScenarioBufferData37_CANNY_FULL, 96, gScenarioKernelData37_CANNY_FULL, 12},
{16, 2, 3608, 32, 2, gScenarioBufferData38_CANNY_FULL, 96, gScenarioKernelData38_CANNY_FULL, 12},
{1, 3, 528, 32, 2, gScenarioBufferData39_CANNY_FULL, 96, gScenarioKernelData39_CANNY_FULL, 12},
{2, 3, 840, 32, 2, gScenarioBufferData40_CANNY_FULL, 96, gScenarioKernelData40_CANNY_FULL, 12},
{3, 3, 1152, 32, 2, gScenarioBufferData41_CANNY_FULL, 96, gScenarioKernelData41_CANNY_FULL, 12},
{4, 3, 1464, 32, 2, gScenarioBufferData42_CANNY_FULL, 96, gScenarioKernelData42_CANNY_FULL, 12},
{5, 3, 1776, 32, 2, gScenarioBufferData43_CANNY_FULL, 96, gScenarioKernelData43_CANNY_FULL, 12},
{6, 3, 2088, 32, 2, gScenarioBufferData44_CANNY_FULL, 96, gScenarioKernelData44_CANNY_FULL, 12},
{7, 3, 2400, 32, 2, gScenarioBufferData45_CANNY_FULL, 96, gScenarioKernelData45_CANNY_FULL, 12},
{8, 3, 2712, 32, 2, gScenarioBufferData46_CANNY_FULL, 96, gScenarioKernelData46_CANNY_FULL, 12},
{9, 3, 3024, 32, 2, gScenarioBufferData47_CANNY_FULL, 96, gScenarioKernelData47_CANNY_FULL, 12},
{10, 3, 3336, 32, 2, gScenarioBufferData48_CANNY_FULL, 96, gScenarioKernelData48_CANNY_FULL, 12},
{11, 3, 3648, 32, 2, gScenarioBufferData49_CANNY_FULL, 96, gScenarioKernelData49_CANNY_FULL, 12},
{1, 4, 688, 32, 2, gScenarioBufferData50_CANNY_FULL, 96, gScenarioKernelData50_CANNY_FULL, 12},
{2, 4, 1096, 32, 2, gScenarioBufferData51_CANNY_FULL, 96, gScenarioKernelData51_CANNY_FULL, 12},
{3, 4, 1504, 32, 2, gScenarioBufferData52_CANNY_FULL, 96, gScenarioKernelData52_CANNY_FULL, 12},
{4, 4, 1912, 32, 2, gScenarioBufferData53_CANNY_FULL, 96, gScenarioKernelData53_CANNY_FULL, 12},
{5, 4, 2320, 32, 2, gScenarioBufferData54_CANNY_FULL, 96, gScenarioKernelData54_CANNY_FULL, 12},
{6, 4, 2728, 32, 2, gScenarioBufferData55_CANNY_FULL, 96, gScenarioKernelData55_CANNY_FULL, 12},
{7, 4, 3136, 32, 2, gScenarioBufferData56_CANNY_FULL, 96, gScenarioKernelData56_CANNY_FULL, 12},
{8, 4, 3544, 32, 2, gScenarioBufferData57_CANNY_FULL, 96, gScenarioKernelData57_CANNY_FULL, 12},
{1, 5, 848, 32, 2, gScenarioBufferData58_CANNY_FULL, 96, gScenarioKernelData58_CANNY_FULL, 12},
{2, 5, 1352, 32, 2, gScenarioBufferData59_CANNY_FULL, 96, gScenarioKernelData59_CANNY_FULL, 12},
{3, 5, 1856, 32, 2, gScenarioBufferData60_CANNY_FULL, 96, gScenarioKernelData60_CANNY_FULL, 12},
{4, 5, 2360, 32, 2, gScenarioBufferData61_CANNY_FULL, 96, gScenarioKernelData61_CANNY_FULL, 12},
{5, 5, 2864, 32, 2, gScenarioBufferData62_CANNY_FULL, 96, gScenarioKernelData62_CANNY_FULL, 12},
{6, 5, 3368, 32, 2, gScenarioBufferData63_CANNY_FULL, 96, gScenarioKernelData63_CANNY_FULL, 12},
{1, 6, 1008, 32, 2, gScenarioBufferData64_CANNY_FULL, 96, gScenarioKernelData64_CANNY_FULL, 12},
{2, 6, 1608, 32, 2, gScenarioBufferData65_CANNY_FULL, 96, gScenarioKernelData65_CANNY_FULL, 12},
{3, 6, 2208, 32, 2, gScenarioBufferData66_CANNY_FULL, 96, gScenarioKernelData66_CANNY_FULL, 12},
{4, 6, 2808, 32, 2, gScenarioBufferData67_CANNY_FULL, 96, gScenarioKernelData67_CANNY_FULL, 12},
{5, 6, 3408, 32, 2, gScenarioBufferData68_CANNY_FULL, 96, gScenarioKernelData68_CANNY_FULL, 12},
{1, 8, 1328, 32, 2, gScenarioBufferData69_CANNY_FULL, 96, gScenarioKernelData69_CANNY_FULL, 12},
{2, 8, 2120, 32, 2, gScenarioBufferData70_CANNY_FULL, 96, gScenarioKernelData70_CANNY_FULL, 12},
{3, 8, 2912, 32, 2, gScenarioBufferData71_CANNY_FULL, 96, gScenarioKernelData71_CANNY_FULL, 12},
{4, 8, 3704, 32, 2, gScenarioBufferData72_CANNY_FULL, 96, gScenarioKernelData72_CANNY_FULL, 12},
{1, 10, 1648, 32, 2, gScenarioBufferData73_CANNY_FULL, 96, gScenarioKernelData73_CANNY_FULL, 12},
{2, 10, 2632, 32, 2, gScenarioBufferData74_CANNY_FULL, 96, gScenarioKernelData74_CANNY_FULL, 12},
{3, 10, 3616, 32, 2, gScenarioBufferData75_CANNY_FULL, 96, gScenarioKernelData75_CANNY_FULL, 12},
{1, 12, 1968, 32, 2, gScenarioBufferData76_CANNY_FULL, 96, gScenarioKernelData76_CANNY_FULL, 12},
{2, 12, 3144, 32, 2, gScenarioBufferData77_CANNY_FULL, 96, gScenarioKernelData77_CANNY_FULL, 12},
{1, 14, 2288, 32, 2, gScenarioBufferData78_CANNY_FULL, 96, gScenarioKernelData78_CANNY_FULL, 12},
{2, 14, 3656, 32, 2, gScenarioBufferData79_CANNY_FULL, 96, gScenarioKernelData79_CANNY_FULL, 12},
{1, 16, 2608, 32, 2, gScenarioBufferData80_CANNY_FULL, 96, gScenarioKernelData80_CANNY_FULL, 12},
{1, 18, 2928, 32, 2, gScenarioBufferData81_CANNY_FULL, 96, gScenarioKernelData81_CANNY_FULL, 12},
{1, 20, 3248, 32, 2, gScenarioBufferData82_CANNY_FULL, 96, gScenarioKernelData82_CANNY_FULL, 12},
{1, 22, 3568, 32, 2, gScenarioBufferData83_CANNY_FULL, 96, gScenarioKernelData83_CANNY_FULL, 12}
};

static acf_scenario_list gScenarioList_CANNY_FULL = {
84, //number of scenarios
gScenarioArray_CANNY_FULL};
//**************************************************************

class CANNY_FULL : public ACF_Process_APU
{

public:
   CANNY_FULL(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CANNY_FULL");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CANNY_FULL",
                                        CANNY_FULL_LOAD_SEGMENTS,
                                        CANNY_FULL_LOAD_PMEM, CANNY_FULL_LOAD_PMEM_SIZE,
                                        CANNY_FULL_LOAD_DMEM, CANNY_FULL_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CANNY_FULL_APEX_LOG_BUFFER, CANNY_FULL_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0);
         AddPort("LOW_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("HIGH_THRESH", icp::DATATYPE_16U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("EDGE_OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CANNY_FULL);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CANNY_FULL

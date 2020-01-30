#ifndef _ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION
#define _ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HISTOGRAM_EQUALIZATION_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HISTOGRAM_EQUALIZATION[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HISTOGRAM_EQUALIZATION[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HISTOGRAM_EQUALIZATION[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HISTOGRAM_EQUALIZATION[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HISTOGRAM_EQUALIZATION[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HISTOGRAM_EQUALIZATION[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HISTOGRAM_EQUALIZATION[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HISTOGRAM_EQUALIZATION[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HISTOGRAM_EQUALIZATION[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HISTOGRAM_EQUALIZATION[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HISTOGRAM_EQUALIZATION[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HISTOGRAM_EQUALIZATION[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HISTOGRAM_EQUALIZATION[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HISTOGRAM_EQUALIZATION[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HISTOGRAM_EQUALIZATION[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HISTOGRAM_EQUALIZATION[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HISTOGRAM_EQUALIZATION[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HISTOGRAM_EQUALIZATION[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HISTOGRAM_EQUALIZATION[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HISTOGRAM_EQUALIZATION[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HISTOGRAM_EQUALIZATION[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HISTOGRAM_EQUALIZATION[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HISTOGRAM_EQUALIZATION[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HISTOGRAM_EQUALIZATION[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HISTOGRAM_EQUALIZATION[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {72, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HISTOGRAM_EQUALIZATION[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HISTOGRAM_EQUALIZATION[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {88, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HISTOGRAM_EQUALIZATION[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HISTOGRAM_EQUALIZATION[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {104, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HISTOGRAM_EQUALIZATION[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HISTOGRAM_EQUALIZATION[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {120, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HISTOGRAM_EQUALIZATION[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HISTOGRAM_EQUALIZATION[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HISTOGRAM_EQUALIZATION[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HISTOGRAM_EQUALIZATION[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HISTOGRAM_EQUALIZATION[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HISTOGRAM_EQUALIZATION[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_HISTOGRAM_EQUALIZATION[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_HISTOGRAM_EQUALIZATION[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_HISTOGRAM_EQUALIZATION[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_HISTOGRAM_EQUALIZATION[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_HISTOGRAM_EQUALIZATION[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_HISTOGRAM_EQUALIZATION[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_HISTOGRAM_EQUALIZATION[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_HISTOGRAM_EQUALIZATION[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_HISTOGRAM_EQUALIZATION[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_HISTOGRAM_EQUALIZATION[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_HISTOGRAM_EQUALIZATION[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_HISTOGRAM_EQUALIZATION[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_HISTOGRAM_EQUALIZATION[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_HISTOGRAM_EQUALIZATION[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_HISTOGRAM_EQUALIZATION[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_HISTOGRAM_EQUALIZATION[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_HISTOGRAM_EQUALIZATION[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_HISTOGRAM_EQUALIZATION[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_HISTOGRAM_EQUALIZATION[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_HISTOGRAM_EQUALIZATION[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {72, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_HISTOGRAM_EQUALIZATION[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_HISTOGRAM_EQUALIZATION[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {88, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_HISTOGRAM_EQUALIZATION[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_HISTOGRAM_EQUALIZATION[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {104, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_HISTOGRAM_EQUALIZATION[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {112, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_HISTOGRAM_EQUALIZATION[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {120, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_HISTOGRAM_EQUALIZATION[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {128, 2, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_HISTOGRAM_EQUALIZATION[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_HISTOGRAM_EQUALIZATION[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_HISTOGRAM_EQUALIZATION[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_HISTOGRAM_EQUALIZATION[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_HISTOGRAM_EQUALIZATION[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_HISTOGRAM_EQUALIZATION[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_HISTOGRAM_EQUALIZATION[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_HISTOGRAM_EQUALIZATION[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_HISTOGRAM_EQUALIZATION[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_HISTOGRAM_EQUALIZATION[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_HISTOGRAM_EQUALIZATION[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_HISTOGRAM_EQUALIZATION[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_HISTOGRAM_EQUALIZATION[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_HISTOGRAM_EQUALIZATION[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_HISTOGRAM_EQUALIZATION[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_HISTOGRAM_EQUALIZATION[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_HISTOGRAM_EQUALIZATION[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_HISTOGRAM_EQUALIZATION[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_HISTOGRAM_EQUALIZATION[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_HISTOGRAM_EQUALIZATION[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_HISTOGRAM_EQUALIZATION[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_HISTOGRAM_EQUALIZATION[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_HISTOGRAM_EQUALIZATION[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_HISTOGRAM_EQUALIZATION[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_HISTOGRAM_EQUALIZATION[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {72, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_HISTOGRAM_EQUALIZATION[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {80, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_HISTOGRAM_EQUALIZATION[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {88, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_HISTOGRAM_EQUALIZATION[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {96, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_HISTOGRAM_EQUALIZATION[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {104, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_HISTOGRAM_EQUALIZATION[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {112, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_HISTOGRAM_EQUALIZATION[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {120, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_HISTOGRAM_EQUALIZATION[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {128, 3, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_HISTOGRAM_EQUALIZATION[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_HISTOGRAM_EQUALIZATION[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_HISTOGRAM_EQUALIZATION[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_HISTOGRAM_EQUALIZATION[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_HISTOGRAM_EQUALIZATION[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_HISTOGRAM_EQUALIZATION[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_HISTOGRAM_EQUALIZATION[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_HISTOGRAM_EQUALIZATION[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_HISTOGRAM_EQUALIZATION[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_HISTOGRAM_EQUALIZATION[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_HISTOGRAM_EQUALIZATION[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_HISTOGRAM_EQUALIZATION[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_HISTOGRAM_EQUALIZATION[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_HISTOGRAM_EQUALIZATION[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_HISTOGRAM_EQUALIZATION[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_HISTOGRAM_EQUALIZATION[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_HISTOGRAM_EQUALIZATION[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_HISTOGRAM_EQUALIZATION[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_HISTOGRAM_EQUALIZATION[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_HISTOGRAM_EQUALIZATION[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_HISTOGRAM_EQUALIZATION[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_HISTOGRAM_EQUALIZATION[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_HISTOGRAM_EQUALIZATION[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_HISTOGRAM_EQUALIZATION[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_HISTOGRAM_EQUALIZATION[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {72, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_HISTOGRAM_EQUALIZATION[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {80, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_HISTOGRAM_EQUALIZATION[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {88, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_HISTOGRAM_EQUALIZATION[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {96, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_HISTOGRAM_EQUALIZATION[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {104, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_HISTOGRAM_EQUALIZATION[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {112, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_HISTOGRAM_EQUALIZATION[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {120, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_HISTOGRAM_EQUALIZATION[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {128, 4, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_HISTOGRAM_EQUALIZATION[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_HISTOGRAM_EQUALIZATION[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_HISTOGRAM_EQUALIZATION[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_HISTOGRAM_EQUALIZATION[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_HISTOGRAM_EQUALIZATION[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_HISTOGRAM_EQUALIZATION[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_HISTOGRAM_EQUALIZATION[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_HISTOGRAM_EQUALIZATION[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_HISTOGRAM_EQUALIZATION[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_HISTOGRAM_EQUALIZATION[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_HISTOGRAM_EQUALIZATION[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_HISTOGRAM_EQUALIZATION[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_HISTOGRAM_EQUALIZATION[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_HISTOGRAM_EQUALIZATION[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_HISTOGRAM_EQUALIZATION[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_HISTOGRAM_EQUALIZATION[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_HISTOGRAM_EQUALIZATION[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_HISTOGRAM_EQUALIZATION[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_HISTOGRAM_EQUALIZATION[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_HISTOGRAM_EQUALIZATION[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_HISTOGRAM_EQUALIZATION[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_HISTOGRAM_EQUALIZATION[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_HISTOGRAM_EQUALIZATION[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_HISTOGRAM_EQUALIZATION[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_HISTOGRAM_EQUALIZATION[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {72, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_HISTOGRAM_EQUALIZATION[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {80, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_HISTOGRAM_EQUALIZATION[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {88, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_HISTOGRAM_EQUALIZATION[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {96, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_HISTOGRAM_EQUALIZATION[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {104, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_HISTOGRAM_EQUALIZATION[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {112, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_HISTOGRAM_EQUALIZATION[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {120, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_HISTOGRAM_EQUALIZATION[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {128, 5, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_HISTOGRAM_EQUALIZATION[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_HISTOGRAM_EQUALIZATION[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_HISTOGRAM_EQUALIZATION[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_HISTOGRAM_EQUALIZATION[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_HISTOGRAM_EQUALIZATION[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_HISTOGRAM_EQUALIZATION[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_HISTOGRAM_EQUALIZATION[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_HISTOGRAM_EQUALIZATION[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_HISTOGRAM_EQUALIZATION[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_HISTOGRAM_EQUALIZATION[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_HISTOGRAM_EQUALIZATION[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_HISTOGRAM_EQUALIZATION[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_HISTOGRAM_EQUALIZATION[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_HISTOGRAM_EQUALIZATION[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_HISTOGRAM_EQUALIZATION[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_HISTOGRAM_EQUALIZATION[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_HISTOGRAM_EQUALIZATION[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_HISTOGRAM_EQUALIZATION[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_HISTOGRAM_EQUALIZATION[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_HISTOGRAM_EQUALIZATION[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_HISTOGRAM_EQUALIZATION[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_HISTOGRAM_EQUALIZATION[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_HISTOGRAM_EQUALIZATION[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_HISTOGRAM_EQUALIZATION[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_HISTOGRAM_EQUALIZATION[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {72, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_HISTOGRAM_EQUALIZATION[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {80, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_HISTOGRAM_EQUALIZATION[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {88, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_HISTOGRAM_EQUALIZATION[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {96, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_HISTOGRAM_EQUALIZATION[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {104, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_HISTOGRAM_EQUALIZATION[] = {{112, 6, 2, 0, {0, 0, 0, 0}}, {112, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_HISTOGRAM_EQUALIZATION[] = {{120, 6, 2, 0, {0, 0, 0, 0}}, {120, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_HISTOGRAM_EQUALIZATION[] = {{128, 6, 2, 0, {0, 0, 0, 0}}, {128, 6, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_HISTOGRAM_EQUALIZATION[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_HISTOGRAM_EQUALIZATION[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_HISTOGRAM_EQUALIZATION[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_HISTOGRAM_EQUALIZATION[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_HISTOGRAM_EQUALIZATION[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_HISTOGRAM_EQUALIZATION[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_HISTOGRAM_EQUALIZATION[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_HISTOGRAM_EQUALIZATION[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_HISTOGRAM_EQUALIZATION[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_HISTOGRAM_EQUALIZATION[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_HISTOGRAM_EQUALIZATION[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_HISTOGRAM_EQUALIZATION[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_HISTOGRAM_EQUALIZATION[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_HISTOGRAM_EQUALIZATION[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_HISTOGRAM_EQUALIZATION[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_HISTOGRAM_EQUALIZATION[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_HISTOGRAM_EQUALIZATION[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_HISTOGRAM_EQUALIZATION[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_HISTOGRAM_EQUALIZATION[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_HISTOGRAM_EQUALIZATION[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_HISTOGRAM_EQUALIZATION[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_HISTOGRAM_EQUALIZATION[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_HISTOGRAM_EQUALIZATION[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_HISTOGRAM_EQUALIZATION[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_HISTOGRAM_EQUALIZATION[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {72, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_HISTOGRAM_EQUALIZATION[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {80, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_HISTOGRAM_EQUALIZATION[] = {{88, 8, 2, 0, {0, 0, 0, 0}}, {88, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_HISTOGRAM_EQUALIZATION[] = {{96, 8, 2, 0, {0, 0, 0, 0}}, {96, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_HISTOGRAM_EQUALIZATION[] = {{104, 8, 2, 0, {0, 0, 0, 0}}, {104, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_HISTOGRAM_EQUALIZATION[] = {{112, 8, 2, 0, {0, 0, 0, 0}}, {112, 8, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_HISTOGRAM_EQUALIZATION[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_HISTOGRAM_EQUALIZATION[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_HISTOGRAM_EQUALIZATION[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_HISTOGRAM_EQUALIZATION[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_HISTOGRAM_EQUALIZATION[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_HISTOGRAM_EQUALIZATION[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_HISTOGRAM_EQUALIZATION[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_HISTOGRAM_EQUALIZATION[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_HISTOGRAM_EQUALIZATION[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_HISTOGRAM_EQUALIZATION[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_HISTOGRAM_EQUALIZATION[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_HISTOGRAM_EQUALIZATION[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_HISTOGRAM_EQUALIZATION[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_HISTOGRAM_EQUALIZATION[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_HISTOGRAM_EQUALIZATION[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_HISTOGRAM_EQUALIZATION[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_HISTOGRAM_EQUALIZATION[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_HISTOGRAM_EQUALIZATION[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_HISTOGRAM_EQUALIZATION[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_HISTOGRAM_EQUALIZATION[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_HISTOGRAM_EQUALIZATION[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_HISTOGRAM_EQUALIZATION[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_HISTOGRAM_EQUALIZATION[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_HISTOGRAM_EQUALIZATION[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_HISTOGRAM_EQUALIZATION[] = {{72, 10, 2, 0, {0, 0, 0, 0}}, {72, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_HISTOGRAM_EQUALIZATION[] = {{80, 10, 2, 0, {0, 0, 0, 0}}, {80, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_HISTOGRAM_EQUALIZATION[] = {{88, 10, 2, 0, {0, 0, 0, 0}}, {88, 10, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_HISTOGRAM_EQUALIZATION[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_HISTOGRAM_EQUALIZATION[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_HISTOGRAM_EQUALIZATION[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_HISTOGRAM_EQUALIZATION[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_HISTOGRAM_EQUALIZATION[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_HISTOGRAM_EQUALIZATION[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_HISTOGRAM_EQUALIZATION[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_HISTOGRAM_EQUALIZATION[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_HISTOGRAM_EQUALIZATION[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_HISTOGRAM_EQUALIZATION[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_HISTOGRAM_EQUALIZATION[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_HISTOGRAM_EQUALIZATION[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_HISTOGRAM_EQUALIZATION[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_HISTOGRAM_EQUALIZATION[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_HISTOGRAM_EQUALIZATION[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {30, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_HISTOGRAM_EQUALIZATION[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_HISTOGRAM_EQUALIZATION[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_HISTOGRAM_EQUALIZATION[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_HISTOGRAM_EQUALIZATION[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_HISTOGRAM_EQUALIZATION[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_HISTOGRAM_EQUALIZATION[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_HISTOGRAM_EQUALIZATION[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {56, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_HISTOGRAM_EQUALIZATION[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {60, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_HISTOGRAM_EQUALIZATION[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {64, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_HISTOGRAM_EQUALIZATION[] = {{72, 12, 2, 0, {0, 0, 0, 0}}, {72, 12, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_HISTOGRAM_EQUALIZATION[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_HISTOGRAM_EQUALIZATION[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_HISTOGRAM_EQUALIZATION[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_HISTOGRAM_EQUALIZATION[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_HISTOGRAM_EQUALIZATION[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_HISTOGRAM_EQUALIZATION[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_HISTOGRAM_EQUALIZATION[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_HISTOGRAM_EQUALIZATION[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_HISTOGRAM_EQUALIZATION[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_HISTOGRAM_EQUALIZATION[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_HISTOGRAM_EQUALIZATION[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_HISTOGRAM_EQUALIZATION[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_HISTOGRAM_EQUALIZATION[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {26, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_HISTOGRAM_EQUALIZATION[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_HISTOGRAM_EQUALIZATION[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {30, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_HISTOGRAM_EQUALIZATION[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_HISTOGRAM_EQUALIZATION[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_HISTOGRAM_EQUALIZATION[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_HISTOGRAM_EQUALIZATION[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_HISTOGRAM_EQUALIZATION[] = {{48, 14, 2, 0, {0, 0, 0, 0}}, {48, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_HISTOGRAM_EQUALIZATION[] = {{52, 14, 2, 0, {0, 0, 0, 0}}, {52, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_HISTOGRAM_EQUALIZATION[] = {{56, 14, 2, 0, {0, 0, 0, 0}}, {56, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_HISTOGRAM_EQUALIZATION[] = {{60, 14, 2, 0, {0, 0, 0, 0}}, {60, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_HISTOGRAM_EQUALIZATION[] = {{64, 14, 2, 0, {0, 0, 0, 0}}, {64, 14, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_HISTOGRAM_EQUALIZATION[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_HISTOGRAM_EQUALIZATION[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_HISTOGRAM_EQUALIZATION[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_HISTOGRAM_EQUALIZATION[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_HISTOGRAM_EQUALIZATION[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_HISTOGRAM_EQUALIZATION[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_HISTOGRAM_EQUALIZATION[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_HISTOGRAM_EQUALIZATION[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_HISTOGRAM_EQUALIZATION[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_HISTOGRAM_EQUALIZATION[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_HISTOGRAM_EQUALIZATION[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {22, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_HISTOGRAM_EQUALIZATION[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_HISTOGRAM_EQUALIZATION[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {26, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_HISTOGRAM_EQUALIZATION[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_HISTOGRAM_EQUALIZATION[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {30, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_HISTOGRAM_EQUALIZATION[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_HISTOGRAM_EQUALIZATION[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_HISTOGRAM_EQUALIZATION[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_HISTOGRAM_EQUALIZATION[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {44, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_HISTOGRAM_EQUALIZATION[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {48, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_HISTOGRAM_EQUALIZATION[] = {{52, 16, 2, 0, {0, 0, 0, 0}}, {52, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_HISTOGRAM_EQUALIZATION[] = {{56, 16, 2, 0, {0, 0, 0, 0}}, {56, 16, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_HISTOGRAM_EQUALIZATION[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_HISTOGRAM_EQUALIZATION[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_HISTOGRAM_EQUALIZATION[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_HISTOGRAM_EQUALIZATION[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_HISTOGRAM_EQUALIZATION[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_HISTOGRAM_EQUALIZATION[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_HISTOGRAM_EQUALIZATION[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_HISTOGRAM_EQUALIZATION[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_HISTOGRAM_EQUALIZATION[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {18, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_HISTOGRAM_EQUALIZATION[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_HISTOGRAM_EQUALIZATION[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {22, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_HISTOGRAM_EQUALIZATION[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_HISTOGRAM_EQUALIZATION[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {26, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_HISTOGRAM_EQUALIZATION[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_HISTOGRAM_EQUALIZATION[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {30, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_HISTOGRAM_EQUALIZATION[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_HISTOGRAM_EQUALIZATION[] = {{36, 18, 2, 0, {0, 0, 0, 0}}, {36, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_HISTOGRAM_EQUALIZATION[] = {{40, 18, 2, 0, {0, 0, 0, 0}}, {40, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_HISTOGRAM_EQUALIZATION[] = {{44, 18, 2, 0, {0, 0, 0, 0}}, {44, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_HISTOGRAM_EQUALIZATION[] = {{48, 18, 2, 0, {0, 0, 0, 0}}, {48, 18, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_HISTOGRAM_EQUALIZATION[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_HISTOGRAM_EQUALIZATION[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_HISTOGRAM_EQUALIZATION[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_HISTOGRAM_EQUALIZATION[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_HISTOGRAM_EQUALIZATION[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_HISTOGRAM_EQUALIZATION[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_HISTOGRAM_EQUALIZATION[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_HISTOGRAM_EQUALIZATION[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_HISTOGRAM_EQUALIZATION[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {18, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_HISTOGRAM_EQUALIZATION[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_HISTOGRAM_EQUALIZATION[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {22, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_HISTOGRAM_EQUALIZATION[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_HISTOGRAM_EQUALIZATION[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {26, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_HISTOGRAM_EQUALIZATION[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_HISTOGRAM_EQUALIZATION[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {30, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_HISTOGRAM_EQUALIZATION[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_HISTOGRAM_EQUALIZATION[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {36, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_HISTOGRAM_EQUALIZATION[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {40, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_HISTOGRAM_EQUALIZATION[] = {{44, 20, 2, 0, {0, 0, 0, 0}}, {44, 20, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_HISTOGRAM_EQUALIZATION[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_HISTOGRAM_EQUALIZATION[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_HISTOGRAM_EQUALIZATION[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_HISTOGRAM_EQUALIZATION[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_HISTOGRAM_EQUALIZATION[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_HISTOGRAM_EQUALIZATION[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_HISTOGRAM_EQUALIZATION[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_HISTOGRAM_EQUALIZATION[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_HISTOGRAM_EQUALIZATION[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {18, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_HISTOGRAM_EQUALIZATION[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_HISTOGRAM_EQUALIZATION[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {22, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_HISTOGRAM_EQUALIZATION[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_HISTOGRAM_EQUALIZATION[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {26, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_HISTOGRAM_EQUALIZATION[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_HISTOGRAM_EQUALIZATION[] = {{30, 22, 2, 0, {0, 0, 0, 0}}, {30, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_HISTOGRAM_EQUALIZATION[] = {{32, 22, 2, 0, {0, 0, 0, 0}}, {32, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_HISTOGRAM_EQUALIZATION[] = {{36, 22, 2, 0, {0, 0, 0, 0}}, {36, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_HISTOGRAM_EQUALIZATION[] = {{40, 22, 2, 0, {0, 0, 0, 0}}, {40, 22, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_HISTOGRAM_EQUALIZATION[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_HISTOGRAM_EQUALIZATION[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_HISTOGRAM_EQUALIZATION[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_HISTOGRAM_EQUALIZATION[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_HISTOGRAM_EQUALIZATION[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_HISTOGRAM_EQUALIZATION[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_HISTOGRAM_EQUALIZATION[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {14, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_HISTOGRAM_EQUALIZATION[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_HISTOGRAM_EQUALIZATION[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {18, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_HISTOGRAM_EQUALIZATION[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_HISTOGRAM_EQUALIZATION[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {22, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_HISTOGRAM_EQUALIZATION[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_HISTOGRAM_EQUALIZATION[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {26, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_HISTOGRAM_EQUALIZATION[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {28, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_HISTOGRAM_EQUALIZATION[] = {{30, 24, 2, 0, {0, 0, 0, 0}}, {30, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_HISTOGRAM_EQUALIZATION[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {32, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_HISTOGRAM_EQUALIZATION[] = {{36, 24, 2, 0, {0, 0, 0, 0}}, {36, 24, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_HISTOGRAM_EQUALIZATION[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_HISTOGRAM_EQUALIZATION[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_HISTOGRAM_EQUALIZATION[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_HISTOGRAM_EQUALIZATION[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_HISTOGRAM_EQUALIZATION[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData399_HISTOGRAM_EQUALIZATION[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData400_HISTOGRAM_EQUALIZATION[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {14, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData401_HISTOGRAM_EQUALIZATION[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData402_HISTOGRAM_EQUALIZATION[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {18, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData403_HISTOGRAM_EQUALIZATION[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData404_HISTOGRAM_EQUALIZATION[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {22, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData405_HISTOGRAM_EQUALIZATION[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData406_HISTOGRAM_EQUALIZATION[] = {{26, 26, 2, 0, {0, 0, 0, 0}}, {26, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData407_HISTOGRAM_EQUALIZATION[] = {{28, 26, 2, 0, {0, 0, 0, 0}}, {28, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData408_HISTOGRAM_EQUALIZATION[] = {{30, 26, 2, 0, {0, 0, 0, 0}}, {30, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData409_HISTOGRAM_EQUALIZATION[] = {{32, 26, 2, 0, {0, 0, 0, 0}}, {32, 26, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData410_HISTOGRAM_EQUALIZATION[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData411_HISTOGRAM_EQUALIZATION[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData412_HISTOGRAM_EQUALIZATION[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData413_HISTOGRAM_EQUALIZATION[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData414_HISTOGRAM_EQUALIZATION[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData415_HISTOGRAM_EQUALIZATION[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData416_HISTOGRAM_EQUALIZATION[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {14, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData417_HISTOGRAM_EQUALIZATION[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData418_HISTOGRAM_EQUALIZATION[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {18, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData419_HISTOGRAM_EQUALIZATION[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData420_HISTOGRAM_EQUALIZATION[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {22, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData421_HISTOGRAM_EQUALIZATION[] = {{24, 28, 2, 0, {0, 0, 0, 0}}, {24, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData422_HISTOGRAM_EQUALIZATION[] = {{26, 28, 2, 0, {0, 0, 0, 0}}, {26, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData423_HISTOGRAM_EQUALIZATION[] = {{28, 28, 2, 0, {0, 0, 0, 0}}, {28, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData424_HISTOGRAM_EQUALIZATION[] = {{30, 28, 2, 0, {0, 0, 0, 0}}, {30, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData425_HISTOGRAM_EQUALIZATION[] = {{32, 28, 2, 0, {0, 0, 0, 0}}, {32, 28, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData426_HISTOGRAM_EQUALIZATION[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData427_HISTOGRAM_EQUALIZATION[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData428_HISTOGRAM_EQUALIZATION[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData429_HISTOGRAM_EQUALIZATION[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData430_HISTOGRAM_EQUALIZATION[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData431_HISTOGRAM_EQUALIZATION[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData432_HISTOGRAM_EQUALIZATION[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {14, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData433_HISTOGRAM_EQUALIZATION[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData434_HISTOGRAM_EQUALIZATION[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {18, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData435_HISTOGRAM_EQUALIZATION[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData436_HISTOGRAM_EQUALIZATION[] = {{22, 30, 2, 0, {0, 0, 0, 0}}, {22, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData437_HISTOGRAM_EQUALIZATION[] = {{24, 30, 2, 0, {0, 0, 0, 0}}, {24, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData438_HISTOGRAM_EQUALIZATION[] = {{26, 30, 2, 0, {0, 0, 0, 0}}, {26, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData439_HISTOGRAM_EQUALIZATION[] = {{28, 30, 2, 0, {0, 0, 0, 0}}, {28, 30, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData440_HISTOGRAM_EQUALIZATION[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData441_HISTOGRAM_EQUALIZATION[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData442_HISTOGRAM_EQUALIZATION[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData443_HISTOGRAM_EQUALIZATION[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData444_HISTOGRAM_EQUALIZATION[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData445_HISTOGRAM_EQUALIZATION[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData446_HISTOGRAM_EQUALIZATION[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {14, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData447_HISTOGRAM_EQUALIZATION[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData448_HISTOGRAM_EQUALIZATION[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {18, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData449_HISTOGRAM_EQUALIZATION[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData450_HISTOGRAM_EQUALIZATION[] = {{22, 32, 2, 0, {0, 0, 0, 0}}, {22, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData451_HISTOGRAM_EQUALIZATION[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {24, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData452_HISTOGRAM_EQUALIZATION[] = {{26, 32, 2, 0, {0, 0, 0, 0}}, {26, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData453_HISTOGRAM_EQUALIZATION[] = {{28, 32, 2, 0, {0, 0, 0, 0}}, {28, 32, 2, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}, {256, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData399_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData400_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData401_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData402_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData403_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData404_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData405_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData406_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData407_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData408_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData409_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData410_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData411_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData412_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData413_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData414_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData415_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData416_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData417_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData418_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData419_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData420_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData421_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData422_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData423_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData424_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData425_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData426_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData427_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData428_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData429_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData430_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData431_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData432_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData433_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData434_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData435_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData436_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData437_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData438_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData439_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData440_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData441_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData442_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData443_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData444_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData445_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData446_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData447_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData448_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData449_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData450_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData451_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData452_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData453_HISTOGRAM_EQUALIZATION[] = {{0, 0}, {0, 0}};

static acf_scenario gScenarioArray_HISTOGRAM_EQUALIZATION[] = {
{2, 1, 264, 1296, 0, gScenarioBufferData0_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData0_HISTOGRAM_EQUALIZATION, 8},
{4, 1, 272, 1296, 0, gScenarioBufferData1_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData1_HISTOGRAM_EQUALIZATION, 8},
{6, 1, 280, 1296, 0, gScenarioBufferData2_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData2_HISTOGRAM_EQUALIZATION, 8},
{8, 1, 288, 1296, 0, gScenarioBufferData3_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData3_HISTOGRAM_EQUALIZATION, 8},
{10, 1, 296, 1296, 0, gScenarioBufferData4_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData4_HISTOGRAM_EQUALIZATION, 8},
{12, 1, 304, 1296, 0, gScenarioBufferData5_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData5_HISTOGRAM_EQUALIZATION, 8},
{14, 1, 312, 1296, 0, gScenarioBufferData6_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData6_HISTOGRAM_EQUALIZATION, 8},
{16, 1, 320, 1296, 0, gScenarioBufferData7_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData7_HISTOGRAM_EQUALIZATION, 8},
{18, 1, 328, 1296, 0, gScenarioBufferData8_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData8_HISTOGRAM_EQUALIZATION, 8},
{20, 1, 336, 1296, 0, gScenarioBufferData9_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData9_HISTOGRAM_EQUALIZATION, 8},
{22, 1, 344, 1296, 0, gScenarioBufferData10_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData10_HISTOGRAM_EQUALIZATION, 8},
{24, 1, 352, 1296, 0, gScenarioBufferData11_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData11_HISTOGRAM_EQUALIZATION, 8},
{26, 1, 360, 1296, 0, gScenarioBufferData12_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData12_HISTOGRAM_EQUALIZATION, 8},
{28, 1, 368, 1296, 0, gScenarioBufferData13_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData13_HISTOGRAM_EQUALIZATION, 8},
{30, 1, 376, 1296, 0, gScenarioBufferData14_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData14_HISTOGRAM_EQUALIZATION, 8},
{32, 1, 384, 1296, 0, gScenarioBufferData15_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData15_HISTOGRAM_EQUALIZATION, 8},
{36, 1, 400, 1296, 0, gScenarioBufferData16_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData16_HISTOGRAM_EQUALIZATION, 8},
{40, 1, 416, 1296, 0, gScenarioBufferData17_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData17_HISTOGRAM_EQUALIZATION, 8},
{44, 1, 432, 1296, 0, gScenarioBufferData18_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData18_HISTOGRAM_EQUALIZATION, 8},
{48, 1, 448, 1296, 0, gScenarioBufferData19_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData19_HISTOGRAM_EQUALIZATION, 8},
{52, 1, 464, 1296, 0, gScenarioBufferData20_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData20_HISTOGRAM_EQUALIZATION, 8},
{56, 1, 480, 1296, 0, gScenarioBufferData21_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData21_HISTOGRAM_EQUALIZATION, 8},
{60, 1, 496, 1296, 0, gScenarioBufferData22_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData22_HISTOGRAM_EQUALIZATION, 8},
{64, 1, 512, 1296, 0, gScenarioBufferData23_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData23_HISTOGRAM_EQUALIZATION, 8},
{72, 1, 544, 1296, 0, gScenarioBufferData24_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData24_HISTOGRAM_EQUALIZATION, 8},
{80, 1, 576, 1296, 0, gScenarioBufferData25_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData25_HISTOGRAM_EQUALIZATION, 8},
{88, 1, 608, 1296, 0, gScenarioBufferData26_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData26_HISTOGRAM_EQUALIZATION, 8},
{96, 1, 640, 1296, 0, gScenarioBufferData27_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData27_HISTOGRAM_EQUALIZATION, 8},
{104, 1, 672, 1296, 0, gScenarioBufferData28_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData28_HISTOGRAM_EQUALIZATION, 8},
{112, 1, 704, 1296, 0, gScenarioBufferData29_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData29_HISTOGRAM_EQUALIZATION, 8},
{120, 1, 736, 1296, 0, gScenarioBufferData30_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData30_HISTOGRAM_EQUALIZATION, 8},
{128, 1, 768, 1296, 0, gScenarioBufferData31_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData31_HISTOGRAM_EQUALIZATION, 8},
{2, 2, 272, 1296, 0, gScenarioBufferData32_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData32_HISTOGRAM_EQUALIZATION, 8},
{4, 2, 288, 1296, 0, gScenarioBufferData33_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData33_HISTOGRAM_EQUALIZATION, 8},
{6, 2, 304, 1296, 0, gScenarioBufferData34_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData34_HISTOGRAM_EQUALIZATION, 8},
{8, 2, 320, 1296, 0, gScenarioBufferData35_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData35_HISTOGRAM_EQUALIZATION, 8},
{10, 2, 336, 1296, 0, gScenarioBufferData36_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData36_HISTOGRAM_EQUALIZATION, 8},
{12, 2, 352, 1296, 0, gScenarioBufferData37_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData37_HISTOGRAM_EQUALIZATION, 8},
{14, 2, 368, 1296, 0, gScenarioBufferData38_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData38_HISTOGRAM_EQUALIZATION, 8},
{16, 2, 384, 1296, 0, gScenarioBufferData39_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData39_HISTOGRAM_EQUALIZATION, 8},
{18, 2, 400, 1296, 0, gScenarioBufferData40_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData40_HISTOGRAM_EQUALIZATION, 8},
{20, 2, 416, 1296, 0, gScenarioBufferData41_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData41_HISTOGRAM_EQUALIZATION, 8},
{22, 2, 432, 1296, 0, gScenarioBufferData42_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData42_HISTOGRAM_EQUALIZATION, 8},
{24, 2, 448, 1296, 0, gScenarioBufferData43_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData43_HISTOGRAM_EQUALIZATION, 8},
{26, 2, 464, 1296, 0, gScenarioBufferData44_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData44_HISTOGRAM_EQUALIZATION, 8},
{28, 2, 480, 1296, 0, gScenarioBufferData45_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData45_HISTOGRAM_EQUALIZATION, 8},
{30, 2, 496, 1296, 0, gScenarioBufferData46_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData46_HISTOGRAM_EQUALIZATION, 8},
{32, 2, 512, 1296, 0, gScenarioBufferData47_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData47_HISTOGRAM_EQUALIZATION, 8},
{36, 2, 544, 1296, 0, gScenarioBufferData48_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData48_HISTOGRAM_EQUALIZATION, 8},
{40, 2, 576, 1296, 0, gScenarioBufferData49_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData49_HISTOGRAM_EQUALIZATION, 8},
{44, 2, 608, 1296, 0, gScenarioBufferData50_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData50_HISTOGRAM_EQUALIZATION, 8},
{48, 2, 640, 1296, 0, gScenarioBufferData51_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData51_HISTOGRAM_EQUALIZATION, 8},
{52, 2, 672, 1296, 0, gScenarioBufferData52_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData52_HISTOGRAM_EQUALIZATION, 8},
{56, 2, 704, 1296, 0, gScenarioBufferData53_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData53_HISTOGRAM_EQUALIZATION, 8},
{60, 2, 736, 1296, 0, gScenarioBufferData54_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData54_HISTOGRAM_EQUALIZATION, 8},
{64, 2, 768, 1296, 0, gScenarioBufferData55_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData55_HISTOGRAM_EQUALIZATION, 8},
{72, 2, 832, 1296, 0, gScenarioBufferData56_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData56_HISTOGRAM_EQUALIZATION, 8},
{80, 2, 896, 1296, 0, gScenarioBufferData57_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData57_HISTOGRAM_EQUALIZATION, 8},
{88, 2, 960, 1296, 0, gScenarioBufferData58_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData58_HISTOGRAM_EQUALIZATION, 8},
{96, 2, 1024, 1296, 0, gScenarioBufferData59_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData59_HISTOGRAM_EQUALIZATION, 8},
{104, 2, 1088, 1296, 0, gScenarioBufferData60_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData60_HISTOGRAM_EQUALIZATION, 8},
{112, 2, 1152, 1296, 0, gScenarioBufferData61_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData61_HISTOGRAM_EQUALIZATION, 8},
{120, 2, 1216, 1296, 0, gScenarioBufferData62_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData62_HISTOGRAM_EQUALIZATION, 8},
{128, 2, 1280, 1296, 0, gScenarioBufferData63_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData63_HISTOGRAM_EQUALIZATION, 8},
{2, 3, 280, 1296, 0, gScenarioBufferData64_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData64_HISTOGRAM_EQUALIZATION, 8},
{4, 3, 304, 1296, 0, gScenarioBufferData65_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData65_HISTOGRAM_EQUALIZATION, 8},
{6, 3, 328, 1296, 0, gScenarioBufferData66_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData66_HISTOGRAM_EQUALIZATION, 8},
{8, 3, 352, 1296, 0, gScenarioBufferData67_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData67_HISTOGRAM_EQUALIZATION, 8},
{10, 3, 376, 1296, 0, gScenarioBufferData68_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData68_HISTOGRAM_EQUALIZATION, 8},
{12, 3, 400, 1296, 0, gScenarioBufferData69_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData69_HISTOGRAM_EQUALIZATION, 8},
{14, 3, 424, 1296, 0, gScenarioBufferData70_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData70_HISTOGRAM_EQUALIZATION, 8},
{16, 3, 448, 1296, 0, gScenarioBufferData71_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData71_HISTOGRAM_EQUALIZATION, 8},
{18, 3, 472, 1296, 0, gScenarioBufferData72_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData72_HISTOGRAM_EQUALIZATION, 8},
{20, 3, 496, 1296, 0, gScenarioBufferData73_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData73_HISTOGRAM_EQUALIZATION, 8},
{22, 3, 520, 1296, 0, gScenarioBufferData74_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData74_HISTOGRAM_EQUALIZATION, 8},
{24, 3, 544, 1296, 0, gScenarioBufferData75_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData75_HISTOGRAM_EQUALIZATION, 8},
{26, 3, 568, 1296, 0, gScenarioBufferData76_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData76_HISTOGRAM_EQUALIZATION, 8},
{28, 3, 592, 1296, 0, gScenarioBufferData77_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData77_HISTOGRAM_EQUALIZATION, 8},
{30, 3, 616, 1296, 0, gScenarioBufferData78_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData78_HISTOGRAM_EQUALIZATION, 8},
{32, 3, 640, 1296, 0, gScenarioBufferData79_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData79_HISTOGRAM_EQUALIZATION, 8},
{36, 3, 688, 1296, 0, gScenarioBufferData80_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData80_HISTOGRAM_EQUALIZATION, 8},
{40, 3, 736, 1296, 0, gScenarioBufferData81_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData81_HISTOGRAM_EQUALIZATION, 8},
{44, 3, 784, 1296, 0, gScenarioBufferData82_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData82_HISTOGRAM_EQUALIZATION, 8},
{48, 3, 832, 1296, 0, gScenarioBufferData83_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData83_HISTOGRAM_EQUALIZATION, 8},
{52, 3, 880, 1296, 0, gScenarioBufferData84_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData84_HISTOGRAM_EQUALIZATION, 8},
{56, 3, 928, 1296, 0, gScenarioBufferData85_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData85_HISTOGRAM_EQUALIZATION, 8},
{60, 3, 976, 1296, 0, gScenarioBufferData86_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData86_HISTOGRAM_EQUALIZATION, 8},
{64, 3, 1024, 1296, 0, gScenarioBufferData87_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData87_HISTOGRAM_EQUALIZATION, 8},
{72, 3, 1120, 1296, 0, gScenarioBufferData88_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData88_HISTOGRAM_EQUALIZATION, 8},
{80, 3, 1216, 1296, 0, gScenarioBufferData89_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData89_HISTOGRAM_EQUALIZATION, 8},
{88, 3, 1312, 1296, 0, gScenarioBufferData90_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData90_HISTOGRAM_EQUALIZATION, 8},
{96, 3, 1408, 1296, 0, gScenarioBufferData91_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData91_HISTOGRAM_EQUALIZATION, 8},
{104, 3, 1504, 1296, 0, gScenarioBufferData92_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData92_HISTOGRAM_EQUALIZATION, 8},
{112, 3, 1600, 1296, 0, gScenarioBufferData93_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData93_HISTOGRAM_EQUALIZATION, 8},
{120, 3, 1696, 1296, 0, gScenarioBufferData94_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData94_HISTOGRAM_EQUALIZATION, 8},
{128, 3, 1792, 1296, 0, gScenarioBufferData95_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData95_HISTOGRAM_EQUALIZATION, 8},
{2, 4, 288, 1296, 0, gScenarioBufferData96_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData96_HISTOGRAM_EQUALIZATION, 8},
{4, 4, 320, 1296, 0, gScenarioBufferData97_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData97_HISTOGRAM_EQUALIZATION, 8},
{6, 4, 352, 1296, 0, gScenarioBufferData98_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData98_HISTOGRAM_EQUALIZATION, 8},
{8, 4, 384, 1296, 0, gScenarioBufferData99_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData99_HISTOGRAM_EQUALIZATION, 8},
{10, 4, 416, 1296, 0, gScenarioBufferData100_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData100_HISTOGRAM_EQUALIZATION, 8},
{12, 4, 448, 1296, 0, gScenarioBufferData101_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData101_HISTOGRAM_EQUALIZATION, 8},
{14, 4, 480, 1296, 0, gScenarioBufferData102_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData102_HISTOGRAM_EQUALIZATION, 8},
{16, 4, 512, 1296, 0, gScenarioBufferData103_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData103_HISTOGRAM_EQUALIZATION, 8},
{18, 4, 544, 1296, 0, gScenarioBufferData104_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData104_HISTOGRAM_EQUALIZATION, 8},
{20, 4, 576, 1296, 0, gScenarioBufferData105_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData105_HISTOGRAM_EQUALIZATION, 8},
{22, 4, 608, 1296, 0, gScenarioBufferData106_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData106_HISTOGRAM_EQUALIZATION, 8},
{24, 4, 640, 1296, 0, gScenarioBufferData107_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData107_HISTOGRAM_EQUALIZATION, 8},
{26, 4, 672, 1296, 0, gScenarioBufferData108_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData108_HISTOGRAM_EQUALIZATION, 8},
{28, 4, 704, 1296, 0, gScenarioBufferData109_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData109_HISTOGRAM_EQUALIZATION, 8},
{30, 4, 736, 1296, 0, gScenarioBufferData110_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData110_HISTOGRAM_EQUALIZATION, 8},
{32, 4, 768, 1296, 0, gScenarioBufferData111_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData111_HISTOGRAM_EQUALIZATION, 8},
{36, 4, 832, 1296, 0, gScenarioBufferData112_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData112_HISTOGRAM_EQUALIZATION, 8},
{40, 4, 896, 1296, 0, gScenarioBufferData113_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData113_HISTOGRAM_EQUALIZATION, 8},
{44, 4, 960, 1296, 0, gScenarioBufferData114_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData114_HISTOGRAM_EQUALIZATION, 8},
{48, 4, 1024, 1296, 0, gScenarioBufferData115_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData115_HISTOGRAM_EQUALIZATION, 8},
{52, 4, 1088, 1296, 0, gScenarioBufferData116_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData116_HISTOGRAM_EQUALIZATION, 8},
{56, 4, 1152, 1296, 0, gScenarioBufferData117_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData117_HISTOGRAM_EQUALIZATION, 8},
{60, 4, 1216, 1296, 0, gScenarioBufferData118_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData118_HISTOGRAM_EQUALIZATION, 8},
{64, 4, 1280, 1296, 0, gScenarioBufferData119_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData119_HISTOGRAM_EQUALIZATION, 8},
{72, 4, 1408, 1296, 0, gScenarioBufferData120_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData120_HISTOGRAM_EQUALIZATION, 8},
{80, 4, 1536, 1296, 0, gScenarioBufferData121_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData121_HISTOGRAM_EQUALIZATION, 8},
{88, 4, 1664, 1296, 0, gScenarioBufferData122_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData122_HISTOGRAM_EQUALIZATION, 8},
{96, 4, 1792, 1296, 0, gScenarioBufferData123_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData123_HISTOGRAM_EQUALIZATION, 8},
{104, 4, 1920, 1296, 0, gScenarioBufferData124_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData124_HISTOGRAM_EQUALIZATION, 8},
{112, 4, 2048, 1296, 0, gScenarioBufferData125_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData125_HISTOGRAM_EQUALIZATION, 8},
{120, 4, 2176, 1296, 0, gScenarioBufferData126_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData126_HISTOGRAM_EQUALIZATION, 8},
{128, 4, 2304, 1296, 0, gScenarioBufferData127_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData127_HISTOGRAM_EQUALIZATION, 8},
{2, 5, 296, 1296, 0, gScenarioBufferData128_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData128_HISTOGRAM_EQUALIZATION, 8},
{4, 5, 336, 1296, 0, gScenarioBufferData129_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData129_HISTOGRAM_EQUALIZATION, 8},
{6, 5, 376, 1296, 0, gScenarioBufferData130_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData130_HISTOGRAM_EQUALIZATION, 8},
{8, 5, 416, 1296, 0, gScenarioBufferData131_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData131_HISTOGRAM_EQUALIZATION, 8},
{10, 5, 456, 1296, 0, gScenarioBufferData132_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData132_HISTOGRAM_EQUALIZATION, 8},
{12, 5, 496, 1296, 0, gScenarioBufferData133_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData133_HISTOGRAM_EQUALIZATION, 8},
{14, 5, 536, 1296, 0, gScenarioBufferData134_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData134_HISTOGRAM_EQUALIZATION, 8},
{16, 5, 576, 1296, 0, gScenarioBufferData135_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData135_HISTOGRAM_EQUALIZATION, 8},
{18, 5, 616, 1296, 0, gScenarioBufferData136_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData136_HISTOGRAM_EQUALIZATION, 8},
{20, 5, 656, 1296, 0, gScenarioBufferData137_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData137_HISTOGRAM_EQUALIZATION, 8},
{22, 5, 696, 1296, 0, gScenarioBufferData138_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData138_HISTOGRAM_EQUALIZATION, 8},
{24, 5, 736, 1296, 0, gScenarioBufferData139_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData139_HISTOGRAM_EQUALIZATION, 8},
{26, 5, 776, 1296, 0, gScenarioBufferData140_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData140_HISTOGRAM_EQUALIZATION, 8},
{28, 5, 816, 1296, 0, gScenarioBufferData141_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData141_HISTOGRAM_EQUALIZATION, 8},
{30, 5, 856, 1296, 0, gScenarioBufferData142_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData142_HISTOGRAM_EQUALIZATION, 8},
{32, 5, 896, 1296, 0, gScenarioBufferData143_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData143_HISTOGRAM_EQUALIZATION, 8},
{36, 5, 976, 1296, 0, gScenarioBufferData144_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData144_HISTOGRAM_EQUALIZATION, 8},
{40, 5, 1056, 1296, 0, gScenarioBufferData145_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData145_HISTOGRAM_EQUALIZATION, 8},
{44, 5, 1136, 1296, 0, gScenarioBufferData146_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData146_HISTOGRAM_EQUALIZATION, 8},
{48, 5, 1216, 1296, 0, gScenarioBufferData147_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData147_HISTOGRAM_EQUALIZATION, 8},
{52, 5, 1296, 1296, 0, gScenarioBufferData148_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData148_HISTOGRAM_EQUALIZATION, 8},
{56, 5, 1376, 1296, 0, gScenarioBufferData149_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData149_HISTOGRAM_EQUALIZATION, 8},
{60, 5, 1456, 1296, 0, gScenarioBufferData150_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData150_HISTOGRAM_EQUALIZATION, 8},
{64, 5, 1536, 1296, 0, gScenarioBufferData151_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData151_HISTOGRAM_EQUALIZATION, 8},
{72, 5, 1696, 1296, 0, gScenarioBufferData152_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData152_HISTOGRAM_EQUALIZATION, 8},
{80, 5, 1856, 1296, 0, gScenarioBufferData153_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData153_HISTOGRAM_EQUALIZATION, 8},
{88, 5, 2016, 1296, 0, gScenarioBufferData154_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData154_HISTOGRAM_EQUALIZATION, 8},
{96, 5, 2176, 1296, 0, gScenarioBufferData155_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData155_HISTOGRAM_EQUALIZATION, 8},
{104, 5, 2336, 1296, 0, gScenarioBufferData156_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData156_HISTOGRAM_EQUALIZATION, 8},
{112, 5, 2496, 1296, 0, gScenarioBufferData157_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData157_HISTOGRAM_EQUALIZATION, 8},
{120, 5, 2656, 1296, 0, gScenarioBufferData158_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData158_HISTOGRAM_EQUALIZATION, 8},
{128, 5, 2816, 1296, 0, gScenarioBufferData159_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData159_HISTOGRAM_EQUALIZATION, 8},
{2, 6, 304, 1296, 0, gScenarioBufferData160_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData160_HISTOGRAM_EQUALIZATION, 8},
{4, 6, 352, 1296, 0, gScenarioBufferData161_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData161_HISTOGRAM_EQUALIZATION, 8},
{6, 6, 400, 1296, 0, gScenarioBufferData162_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData162_HISTOGRAM_EQUALIZATION, 8},
{8, 6, 448, 1296, 0, gScenarioBufferData163_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData163_HISTOGRAM_EQUALIZATION, 8},
{10, 6, 496, 1296, 0, gScenarioBufferData164_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData164_HISTOGRAM_EQUALIZATION, 8},
{12, 6, 544, 1296, 0, gScenarioBufferData165_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData165_HISTOGRAM_EQUALIZATION, 8},
{14, 6, 592, 1296, 0, gScenarioBufferData166_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData166_HISTOGRAM_EQUALIZATION, 8},
{16, 6, 640, 1296, 0, gScenarioBufferData167_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData167_HISTOGRAM_EQUALIZATION, 8},
{18, 6, 688, 1296, 0, gScenarioBufferData168_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData168_HISTOGRAM_EQUALIZATION, 8},
{20, 6, 736, 1296, 0, gScenarioBufferData169_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData169_HISTOGRAM_EQUALIZATION, 8},
{22, 6, 784, 1296, 0, gScenarioBufferData170_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData170_HISTOGRAM_EQUALIZATION, 8},
{24, 6, 832, 1296, 0, gScenarioBufferData171_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData171_HISTOGRAM_EQUALIZATION, 8},
{26, 6, 880, 1296, 0, gScenarioBufferData172_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData172_HISTOGRAM_EQUALIZATION, 8},
{28, 6, 928, 1296, 0, gScenarioBufferData173_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData173_HISTOGRAM_EQUALIZATION, 8},
{30, 6, 976, 1296, 0, gScenarioBufferData174_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData174_HISTOGRAM_EQUALIZATION, 8},
{32, 6, 1024, 1296, 0, gScenarioBufferData175_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData175_HISTOGRAM_EQUALIZATION, 8},
{36, 6, 1120, 1296, 0, gScenarioBufferData176_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData176_HISTOGRAM_EQUALIZATION, 8},
{40, 6, 1216, 1296, 0, gScenarioBufferData177_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData177_HISTOGRAM_EQUALIZATION, 8},
{44, 6, 1312, 1296, 0, gScenarioBufferData178_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData178_HISTOGRAM_EQUALIZATION, 8},
{48, 6, 1408, 1296, 0, gScenarioBufferData179_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData179_HISTOGRAM_EQUALIZATION, 8},
{52, 6, 1504, 1296, 0, gScenarioBufferData180_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData180_HISTOGRAM_EQUALIZATION, 8},
{56, 6, 1600, 1296, 0, gScenarioBufferData181_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData181_HISTOGRAM_EQUALIZATION, 8},
{60, 6, 1696, 1296, 0, gScenarioBufferData182_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData182_HISTOGRAM_EQUALIZATION, 8},
{64, 6, 1792, 1296, 0, gScenarioBufferData183_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData183_HISTOGRAM_EQUALIZATION, 8},
{72, 6, 1984, 1296, 0, gScenarioBufferData184_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData184_HISTOGRAM_EQUALIZATION, 8},
{80, 6, 2176, 1296, 0, gScenarioBufferData185_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData185_HISTOGRAM_EQUALIZATION, 8},
{88, 6, 2368, 1296, 0, gScenarioBufferData186_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData186_HISTOGRAM_EQUALIZATION, 8},
{96, 6, 2560, 1296, 0, gScenarioBufferData187_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData187_HISTOGRAM_EQUALIZATION, 8},
{104, 6, 2752, 1296, 0, gScenarioBufferData188_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData188_HISTOGRAM_EQUALIZATION, 8},
{112, 6, 2944, 1296, 0, gScenarioBufferData189_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData189_HISTOGRAM_EQUALIZATION, 8},
{120, 6, 3136, 1296, 0, gScenarioBufferData190_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData190_HISTOGRAM_EQUALIZATION, 8},
{128, 6, 3328, 1296, 0, gScenarioBufferData191_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData191_HISTOGRAM_EQUALIZATION, 8},
{2, 8, 320, 1296, 0, gScenarioBufferData192_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData192_HISTOGRAM_EQUALIZATION, 8},
{4, 8, 384, 1296, 0, gScenarioBufferData193_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData193_HISTOGRAM_EQUALIZATION, 8},
{6, 8, 448, 1296, 0, gScenarioBufferData194_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData194_HISTOGRAM_EQUALIZATION, 8},
{8, 8, 512, 1296, 0, gScenarioBufferData195_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData195_HISTOGRAM_EQUALIZATION, 8},
{10, 8, 576, 1296, 0, gScenarioBufferData196_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData196_HISTOGRAM_EQUALIZATION, 8},
{12, 8, 640, 1296, 0, gScenarioBufferData197_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData197_HISTOGRAM_EQUALIZATION, 8},
{14, 8, 704, 1296, 0, gScenarioBufferData198_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData198_HISTOGRAM_EQUALIZATION, 8},
{16, 8, 768, 1296, 0, gScenarioBufferData199_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData199_HISTOGRAM_EQUALIZATION, 8},
{18, 8, 832, 1296, 0, gScenarioBufferData200_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData200_HISTOGRAM_EQUALIZATION, 8},
{20, 8, 896, 1296, 0, gScenarioBufferData201_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData201_HISTOGRAM_EQUALIZATION, 8},
{22, 8, 960, 1296, 0, gScenarioBufferData202_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData202_HISTOGRAM_EQUALIZATION, 8},
{24, 8, 1024, 1296, 0, gScenarioBufferData203_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData203_HISTOGRAM_EQUALIZATION, 8},
{26, 8, 1088, 1296, 0, gScenarioBufferData204_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData204_HISTOGRAM_EQUALIZATION, 8},
{28, 8, 1152, 1296, 0, gScenarioBufferData205_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData205_HISTOGRAM_EQUALIZATION, 8},
{30, 8, 1216, 1296, 0, gScenarioBufferData206_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData206_HISTOGRAM_EQUALIZATION, 8},
{32, 8, 1280, 1296, 0, gScenarioBufferData207_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData207_HISTOGRAM_EQUALIZATION, 8},
{36, 8, 1408, 1296, 0, gScenarioBufferData208_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData208_HISTOGRAM_EQUALIZATION, 8},
{40, 8, 1536, 1296, 0, gScenarioBufferData209_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData209_HISTOGRAM_EQUALIZATION, 8},
{44, 8, 1664, 1296, 0, gScenarioBufferData210_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData210_HISTOGRAM_EQUALIZATION, 8},
{48, 8, 1792, 1296, 0, gScenarioBufferData211_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData211_HISTOGRAM_EQUALIZATION, 8},
{52, 8, 1920, 1296, 0, gScenarioBufferData212_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData212_HISTOGRAM_EQUALIZATION, 8},
{56, 8, 2048, 1296, 0, gScenarioBufferData213_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData213_HISTOGRAM_EQUALIZATION, 8},
{60, 8, 2176, 1296, 0, gScenarioBufferData214_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData214_HISTOGRAM_EQUALIZATION, 8},
{64, 8, 2304, 1296, 0, gScenarioBufferData215_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData215_HISTOGRAM_EQUALIZATION, 8},
{72, 8, 2560, 1296, 0, gScenarioBufferData216_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData216_HISTOGRAM_EQUALIZATION, 8},
{80, 8, 2816, 1296, 0, gScenarioBufferData217_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData217_HISTOGRAM_EQUALIZATION, 8},
{88, 8, 3072, 1296, 0, gScenarioBufferData218_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData218_HISTOGRAM_EQUALIZATION, 8},
{96, 8, 3328, 1296, 0, gScenarioBufferData219_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData219_HISTOGRAM_EQUALIZATION, 8},
{104, 8, 3584, 1296, 0, gScenarioBufferData220_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData220_HISTOGRAM_EQUALIZATION, 8},
{112, 8, 3840, 1296, 0, gScenarioBufferData221_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData221_HISTOGRAM_EQUALIZATION, 8},
{2, 10, 336, 1296, 0, gScenarioBufferData222_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData222_HISTOGRAM_EQUALIZATION, 8},
{4, 10, 416, 1296, 0, gScenarioBufferData223_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData223_HISTOGRAM_EQUALIZATION, 8},
{6, 10, 496, 1296, 0, gScenarioBufferData224_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData224_HISTOGRAM_EQUALIZATION, 8},
{8, 10, 576, 1296, 0, gScenarioBufferData225_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData225_HISTOGRAM_EQUALIZATION, 8},
{10, 10, 656, 1296, 0, gScenarioBufferData226_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData226_HISTOGRAM_EQUALIZATION, 8},
{12, 10, 736, 1296, 0, gScenarioBufferData227_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData227_HISTOGRAM_EQUALIZATION, 8},
{14, 10, 816, 1296, 0, gScenarioBufferData228_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData228_HISTOGRAM_EQUALIZATION, 8},
{16, 10, 896, 1296, 0, gScenarioBufferData229_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData229_HISTOGRAM_EQUALIZATION, 8},
{18, 10, 976, 1296, 0, gScenarioBufferData230_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData230_HISTOGRAM_EQUALIZATION, 8},
{20, 10, 1056, 1296, 0, gScenarioBufferData231_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData231_HISTOGRAM_EQUALIZATION, 8},
{22, 10, 1136, 1296, 0, gScenarioBufferData232_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData232_HISTOGRAM_EQUALIZATION, 8},
{24, 10, 1216, 1296, 0, gScenarioBufferData233_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData233_HISTOGRAM_EQUALIZATION, 8},
{26, 10, 1296, 1296, 0, gScenarioBufferData234_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData234_HISTOGRAM_EQUALIZATION, 8},
{28, 10, 1376, 1296, 0, gScenarioBufferData235_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData235_HISTOGRAM_EQUALIZATION, 8},
{30, 10, 1456, 1296, 0, gScenarioBufferData236_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData236_HISTOGRAM_EQUALIZATION, 8},
{32, 10, 1536, 1296, 0, gScenarioBufferData237_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData237_HISTOGRAM_EQUALIZATION, 8},
{36, 10, 1696, 1296, 0, gScenarioBufferData238_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData238_HISTOGRAM_EQUALIZATION, 8},
{40, 10, 1856, 1296, 0, gScenarioBufferData239_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData239_HISTOGRAM_EQUALIZATION, 8},
{44, 10, 2016, 1296, 0, gScenarioBufferData240_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData240_HISTOGRAM_EQUALIZATION, 8},
{48, 10, 2176, 1296, 0, gScenarioBufferData241_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData241_HISTOGRAM_EQUALIZATION, 8},
{52, 10, 2336, 1296, 0, gScenarioBufferData242_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData242_HISTOGRAM_EQUALIZATION, 8},
{56, 10, 2496, 1296, 0, gScenarioBufferData243_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData243_HISTOGRAM_EQUALIZATION, 8},
{60, 10, 2656, 1296, 0, gScenarioBufferData244_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData244_HISTOGRAM_EQUALIZATION, 8},
{64, 10, 2816, 1296, 0, gScenarioBufferData245_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData245_HISTOGRAM_EQUALIZATION, 8},
{72, 10, 3136, 1296, 0, gScenarioBufferData246_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData246_HISTOGRAM_EQUALIZATION, 8},
{80, 10, 3456, 1296, 0, gScenarioBufferData247_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData247_HISTOGRAM_EQUALIZATION, 8},
{88, 10, 3776, 1296, 0, gScenarioBufferData248_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData248_HISTOGRAM_EQUALIZATION, 8},
{2, 12, 352, 1296, 0, gScenarioBufferData249_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData249_HISTOGRAM_EQUALIZATION, 8},
{4, 12, 448, 1296, 0, gScenarioBufferData250_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData250_HISTOGRAM_EQUALIZATION, 8},
{6, 12, 544, 1296, 0, gScenarioBufferData251_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData251_HISTOGRAM_EQUALIZATION, 8},
{8, 12, 640, 1296, 0, gScenarioBufferData252_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData252_HISTOGRAM_EQUALIZATION, 8},
{10, 12, 736, 1296, 0, gScenarioBufferData253_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData253_HISTOGRAM_EQUALIZATION, 8},
{12, 12, 832, 1296, 0, gScenarioBufferData254_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData254_HISTOGRAM_EQUALIZATION, 8},
{14, 12, 928, 1296, 0, gScenarioBufferData255_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData255_HISTOGRAM_EQUALIZATION, 8},
{16, 12, 1024, 1296, 0, gScenarioBufferData256_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData256_HISTOGRAM_EQUALIZATION, 8},
{18, 12, 1120, 1296, 0, gScenarioBufferData257_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData257_HISTOGRAM_EQUALIZATION, 8},
{20, 12, 1216, 1296, 0, gScenarioBufferData258_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData258_HISTOGRAM_EQUALIZATION, 8},
{22, 12, 1312, 1296, 0, gScenarioBufferData259_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData259_HISTOGRAM_EQUALIZATION, 8},
{24, 12, 1408, 1296, 0, gScenarioBufferData260_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData260_HISTOGRAM_EQUALIZATION, 8},
{26, 12, 1504, 1296, 0, gScenarioBufferData261_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData261_HISTOGRAM_EQUALIZATION, 8},
{28, 12, 1600, 1296, 0, gScenarioBufferData262_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData262_HISTOGRAM_EQUALIZATION, 8},
{30, 12, 1696, 1296, 0, gScenarioBufferData263_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData263_HISTOGRAM_EQUALIZATION, 8},
{32, 12, 1792, 1296, 0, gScenarioBufferData264_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData264_HISTOGRAM_EQUALIZATION, 8},
{36, 12, 1984, 1296, 0, gScenarioBufferData265_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData265_HISTOGRAM_EQUALIZATION, 8},
{40, 12, 2176, 1296, 0, gScenarioBufferData266_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData266_HISTOGRAM_EQUALIZATION, 8},
{44, 12, 2368, 1296, 0, gScenarioBufferData267_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData267_HISTOGRAM_EQUALIZATION, 8},
{48, 12, 2560, 1296, 0, gScenarioBufferData268_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData268_HISTOGRAM_EQUALIZATION, 8},
{52, 12, 2752, 1296, 0, gScenarioBufferData269_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData269_HISTOGRAM_EQUALIZATION, 8},
{56, 12, 2944, 1296, 0, gScenarioBufferData270_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData270_HISTOGRAM_EQUALIZATION, 8},
{60, 12, 3136, 1296, 0, gScenarioBufferData271_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData271_HISTOGRAM_EQUALIZATION, 8},
{64, 12, 3328, 1296, 0, gScenarioBufferData272_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData272_HISTOGRAM_EQUALIZATION, 8},
{72, 12, 3712, 1296, 0, gScenarioBufferData273_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData273_HISTOGRAM_EQUALIZATION, 8},
{2, 14, 368, 1296, 0, gScenarioBufferData274_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData274_HISTOGRAM_EQUALIZATION, 8},
{4, 14, 480, 1296, 0, gScenarioBufferData275_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData275_HISTOGRAM_EQUALIZATION, 8},
{6, 14, 592, 1296, 0, gScenarioBufferData276_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData276_HISTOGRAM_EQUALIZATION, 8},
{8, 14, 704, 1296, 0, gScenarioBufferData277_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData277_HISTOGRAM_EQUALIZATION, 8},
{10, 14, 816, 1296, 0, gScenarioBufferData278_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData278_HISTOGRAM_EQUALIZATION, 8},
{12, 14, 928, 1296, 0, gScenarioBufferData279_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData279_HISTOGRAM_EQUALIZATION, 8},
{14, 14, 1040, 1296, 0, gScenarioBufferData280_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData280_HISTOGRAM_EQUALIZATION, 8},
{16, 14, 1152, 1296, 0, gScenarioBufferData281_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData281_HISTOGRAM_EQUALIZATION, 8},
{18, 14, 1264, 1296, 0, gScenarioBufferData282_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData282_HISTOGRAM_EQUALIZATION, 8},
{20, 14, 1376, 1296, 0, gScenarioBufferData283_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData283_HISTOGRAM_EQUALIZATION, 8},
{22, 14, 1488, 1296, 0, gScenarioBufferData284_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData284_HISTOGRAM_EQUALIZATION, 8},
{24, 14, 1600, 1296, 0, gScenarioBufferData285_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData285_HISTOGRAM_EQUALIZATION, 8},
{26, 14, 1712, 1296, 0, gScenarioBufferData286_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData286_HISTOGRAM_EQUALIZATION, 8},
{28, 14, 1824, 1296, 0, gScenarioBufferData287_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData287_HISTOGRAM_EQUALIZATION, 8},
{30, 14, 1936, 1296, 0, gScenarioBufferData288_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData288_HISTOGRAM_EQUALIZATION, 8},
{32, 14, 2048, 1296, 0, gScenarioBufferData289_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData289_HISTOGRAM_EQUALIZATION, 8},
{36, 14, 2272, 1296, 0, gScenarioBufferData290_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData290_HISTOGRAM_EQUALIZATION, 8},
{40, 14, 2496, 1296, 0, gScenarioBufferData291_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData291_HISTOGRAM_EQUALIZATION, 8},
{44, 14, 2720, 1296, 0, gScenarioBufferData292_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData292_HISTOGRAM_EQUALIZATION, 8},
{48, 14, 2944, 1296, 0, gScenarioBufferData293_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData293_HISTOGRAM_EQUALIZATION, 8},
{52, 14, 3168, 1296, 0, gScenarioBufferData294_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData294_HISTOGRAM_EQUALIZATION, 8},
{56, 14, 3392, 1296, 0, gScenarioBufferData295_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData295_HISTOGRAM_EQUALIZATION, 8},
{60, 14, 3616, 1296, 0, gScenarioBufferData296_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData296_HISTOGRAM_EQUALIZATION, 8},
{64, 14, 3840, 1296, 0, gScenarioBufferData297_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData297_HISTOGRAM_EQUALIZATION, 8},
{2, 16, 384, 1296, 0, gScenarioBufferData298_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData298_HISTOGRAM_EQUALIZATION, 8},
{4, 16, 512, 1296, 0, gScenarioBufferData299_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData299_HISTOGRAM_EQUALIZATION, 8},
{6, 16, 640, 1296, 0, gScenarioBufferData300_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData300_HISTOGRAM_EQUALIZATION, 8},
{8, 16, 768, 1296, 0, gScenarioBufferData301_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData301_HISTOGRAM_EQUALIZATION, 8},
{10, 16, 896, 1296, 0, gScenarioBufferData302_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData302_HISTOGRAM_EQUALIZATION, 8},
{12, 16, 1024, 1296, 0, gScenarioBufferData303_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData303_HISTOGRAM_EQUALIZATION, 8},
{14, 16, 1152, 1296, 0, gScenarioBufferData304_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData304_HISTOGRAM_EQUALIZATION, 8},
{16, 16, 1280, 1296, 0, gScenarioBufferData305_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData305_HISTOGRAM_EQUALIZATION, 8},
{18, 16, 1408, 1296, 0, gScenarioBufferData306_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData306_HISTOGRAM_EQUALIZATION, 8},
{20, 16, 1536, 1296, 0, gScenarioBufferData307_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData307_HISTOGRAM_EQUALIZATION, 8},
{22, 16, 1664, 1296, 0, gScenarioBufferData308_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData308_HISTOGRAM_EQUALIZATION, 8},
{24, 16, 1792, 1296, 0, gScenarioBufferData309_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData309_HISTOGRAM_EQUALIZATION, 8},
{26, 16, 1920, 1296, 0, gScenarioBufferData310_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData310_HISTOGRAM_EQUALIZATION, 8},
{28, 16, 2048, 1296, 0, gScenarioBufferData311_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData311_HISTOGRAM_EQUALIZATION, 8},
{30, 16, 2176, 1296, 0, gScenarioBufferData312_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData312_HISTOGRAM_EQUALIZATION, 8},
{32, 16, 2304, 1296, 0, gScenarioBufferData313_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData313_HISTOGRAM_EQUALIZATION, 8},
{36, 16, 2560, 1296, 0, gScenarioBufferData314_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData314_HISTOGRAM_EQUALIZATION, 8},
{40, 16, 2816, 1296, 0, gScenarioBufferData315_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData315_HISTOGRAM_EQUALIZATION, 8},
{44, 16, 3072, 1296, 0, gScenarioBufferData316_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData316_HISTOGRAM_EQUALIZATION, 8},
{48, 16, 3328, 1296, 0, gScenarioBufferData317_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData317_HISTOGRAM_EQUALIZATION, 8},
{52, 16, 3584, 1296, 0, gScenarioBufferData318_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData318_HISTOGRAM_EQUALIZATION, 8},
{56, 16, 3840, 1296, 0, gScenarioBufferData319_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData319_HISTOGRAM_EQUALIZATION, 8},
{2, 18, 400, 1296, 0, gScenarioBufferData320_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData320_HISTOGRAM_EQUALIZATION, 8},
{4, 18, 544, 1296, 0, gScenarioBufferData321_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData321_HISTOGRAM_EQUALIZATION, 8},
{6, 18, 688, 1296, 0, gScenarioBufferData322_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData322_HISTOGRAM_EQUALIZATION, 8},
{8, 18, 832, 1296, 0, gScenarioBufferData323_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData323_HISTOGRAM_EQUALIZATION, 8},
{10, 18, 976, 1296, 0, gScenarioBufferData324_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData324_HISTOGRAM_EQUALIZATION, 8},
{12, 18, 1120, 1296, 0, gScenarioBufferData325_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData325_HISTOGRAM_EQUALIZATION, 8},
{14, 18, 1264, 1296, 0, gScenarioBufferData326_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData326_HISTOGRAM_EQUALIZATION, 8},
{16, 18, 1408, 1296, 0, gScenarioBufferData327_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData327_HISTOGRAM_EQUALIZATION, 8},
{18, 18, 1552, 1296, 0, gScenarioBufferData328_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData328_HISTOGRAM_EQUALIZATION, 8},
{20, 18, 1696, 1296, 0, gScenarioBufferData329_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData329_HISTOGRAM_EQUALIZATION, 8},
{22, 18, 1840, 1296, 0, gScenarioBufferData330_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData330_HISTOGRAM_EQUALIZATION, 8},
{24, 18, 1984, 1296, 0, gScenarioBufferData331_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData331_HISTOGRAM_EQUALIZATION, 8},
{26, 18, 2128, 1296, 0, gScenarioBufferData332_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData332_HISTOGRAM_EQUALIZATION, 8},
{28, 18, 2272, 1296, 0, gScenarioBufferData333_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData333_HISTOGRAM_EQUALIZATION, 8},
{30, 18, 2416, 1296, 0, gScenarioBufferData334_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData334_HISTOGRAM_EQUALIZATION, 8},
{32, 18, 2560, 1296, 0, gScenarioBufferData335_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData335_HISTOGRAM_EQUALIZATION, 8},
{36, 18, 2848, 1296, 0, gScenarioBufferData336_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData336_HISTOGRAM_EQUALIZATION, 8},
{40, 18, 3136, 1296, 0, gScenarioBufferData337_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData337_HISTOGRAM_EQUALIZATION, 8},
{44, 18, 3424, 1296, 0, gScenarioBufferData338_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData338_HISTOGRAM_EQUALIZATION, 8},
{48, 18, 3712, 1296, 0, gScenarioBufferData339_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData339_HISTOGRAM_EQUALIZATION, 8},
{2, 20, 416, 1296, 0, gScenarioBufferData340_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData340_HISTOGRAM_EQUALIZATION, 8},
{4, 20, 576, 1296, 0, gScenarioBufferData341_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData341_HISTOGRAM_EQUALIZATION, 8},
{6, 20, 736, 1296, 0, gScenarioBufferData342_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData342_HISTOGRAM_EQUALIZATION, 8},
{8, 20, 896, 1296, 0, gScenarioBufferData343_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData343_HISTOGRAM_EQUALIZATION, 8},
{10, 20, 1056, 1296, 0, gScenarioBufferData344_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData344_HISTOGRAM_EQUALIZATION, 8},
{12, 20, 1216, 1296, 0, gScenarioBufferData345_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData345_HISTOGRAM_EQUALIZATION, 8},
{14, 20, 1376, 1296, 0, gScenarioBufferData346_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData346_HISTOGRAM_EQUALIZATION, 8},
{16, 20, 1536, 1296, 0, gScenarioBufferData347_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData347_HISTOGRAM_EQUALIZATION, 8},
{18, 20, 1696, 1296, 0, gScenarioBufferData348_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData348_HISTOGRAM_EQUALIZATION, 8},
{20, 20, 1856, 1296, 0, gScenarioBufferData349_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData349_HISTOGRAM_EQUALIZATION, 8},
{22, 20, 2016, 1296, 0, gScenarioBufferData350_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData350_HISTOGRAM_EQUALIZATION, 8},
{24, 20, 2176, 1296, 0, gScenarioBufferData351_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData351_HISTOGRAM_EQUALIZATION, 8},
{26, 20, 2336, 1296, 0, gScenarioBufferData352_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData352_HISTOGRAM_EQUALIZATION, 8},
{28, 20, 2496, 1296, 0, gScenarioBufferData353_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData353_HISTOGRAM_EQUALIZATION, 8},
{30, 20, 2656, 1296, 0, gScenarioBufferData354_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData354_HISTOGRAM_EQUALIZATION, 8},
{32, 20, 2816, 1296, 0, gScenarioBufferData355_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData355_HISTOGRAM_EQUALIZATION, 8},
{36, 20, 3136, 1296, 0, gScenarioBufferData356_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData356_HISTOGRAM_EQUALIZATION, 8},
{40, 20, 3456, 1296, 0, gScenarioBufferData357_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData357_HISTOGRAM_EQUALIZATION, 8},
{44, 20, 3776, 1296, 0, gScenarioBufferData358_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData358_HISTOGRAM_EQUALIZATION, 8},
{2, 22, 432, 1296, 0, gScenarioBufferData359_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData359_HISTOGRAM_EQUALIZATION, 8},
{4, 22, 608, 1296, 0, gScenarioBufferData360_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData360_HISTOGRAM_EQUALIZATION, 8},
{6, 22, 784, 1296, 0, gScenarioBufferData361_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData361_HISTOGRAM_EQUALIZATION, 8},
{8, 22, 960, 1296, 0, gScenarioBufferData362_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData362_HISTOGRAM_EQUALIZATION, 8},
{10, 22, 1136, 1296, 0, gScenarioBufferData363_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData363_HISTOGRAM_EQUALIZATION, 8},
{12, 22, 1312, 1296, 0, gScenarioBufferData364_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData364_HISTOGRAM_EQUALIZATION, 8},
{14, 22, 1488, 1296, 0, gScenarioBufferData365_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData365_HISTOGRAM_EQUALIZATION, 8},
{16, 22, 1664, 1296, 0, gScenarioBufferData366_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData366_HISTOGRAM_EQUALIZATION, 8},
{18, 22, 1840, 1296, 0, gScenarioBufferData367_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData367_HISTOGRAM_EQUALIZATION, 8},
{20, 22, 2016, 1296, 0, gScenarioBufferData368_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData368_HISTOGRAM_EQUALIZATION, 8},
{22, 22, 2192, 1296, 0, gScenarioBufferData369_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData369_HISTOGRAM_EQUALIZATION, 8},
{24, 22, 2368, 1296, 0, gScenarioBufferData370_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData370_HISTOGRAM_EQUALIZATION, 8},
{26, 22, 2544, 1296, 0, gScenarioBufferData371_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData371_HISTOGRAM_EQUALIZATION, 8},
{28, 22, 2720, 1296, 0, gScenarioBufferData372_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData372_HISTOGRAM_EQUALIZATION, 8},
{30, 22, 2896, 1296, 0, gScenarioBufferData373_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData373_HISTOGRAM_EQUALIZATION, 8},
{32, 22, 3072, 1296, 0, gScenarioBufferData374_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData374_HISTOGRAM_EQUALIZATION, 8},
{36, 22, 3424, 1296, 0, gScenarioBufferData375_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData375_HISTOGRAM_EQUALIZATION, 8},
{40, 22, 3776, 1296, 0, gScenarioBufferData376_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData376_HISTOGRAM_EQUALIZATION, 8},
{2, 24, 448, 1296, 0, gScenarioBufferData377_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData377_HISTOGRAM_EQUALIZATION, 8},
{4, 24, 640, 1296, 0, gScenarioBufferData378_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData378_HISTOGRAM_EQUALIZATION, 8},
{6, 24, 832, 1296, 0, gScenarioBufferData379_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData379_HISTOGRAM_EQUALIZATION, 8},
{8, 24, 1024, 1296, 0, gScenarioBufferData380_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData380_HISTOGRAM_EQUALIZATION, 8},
{10, 24, 1216, 1296, 0, gScenarioBufferData381_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData381_HISTOGRAM_EQUALIZATION, 8},
{12, 24, 1408, 1296, 0, gScenarioBufferData382_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData382_HISTOGRAM_EQUALIZATION, 8},
{14, 24, 1600, 1296, 0, gScenarioBufferData383_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData383_HISTOGRAM_EQUALIZATION, 8},
{16, 24, 1792, 1296, 0, gScenarioBufferData384_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData384_HISTOGRAM_EQUALIZATION, 8},
{18, 24, 1984, 1296, 0, gScenarioBufferData385_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData385_HISTOGRAM_EQUALIZATION, 8},
{20, 24, 2176, 1296, 0, gScenarioBufferData386_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData386_HISTOGRAM_EQUALIZATION, 8},
{22, 24, 2368, 1296, 0, gScenarioBufferData387_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData387_HISTOGRAM_EQUALIZATION, 8},
{24, 24, 2560, 1296, 0, gScenarioBufferData388_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData388_HISTOGRAM_EQUALIZATION, 8},
{26, 24, 2752, 1296, 0, gScenarioBufferData389_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData389_HISTOGRAM_EQUALIZATION, 8},
{28, 24, 2944, 1296, 0, gScenarioBufferData390_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData390_HISTOGRAM_EQUALIZATION, 8},
{30, 24, 3136, 1296, 0, gScenarioBufferData391_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData391_HISTOGRAM_EQUALIZATION, 8},
{32, 24, 3328, 1296, 0, gScenarioBufferData392_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData392_HISTOGRAM_EQUALIZATION, 8},
{36, 24, 3712, 1296, 0, gScenarioBufferData393_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData393_HISTOGRAM_EQUALIZATION, 8},
{2, 26, 464, 1296, 0, gScenarioBufferData394_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData394_HISTOGRAM_EQUALIZATION, 8},
{4, 26, 672, 1296, 0, gScenarioBufferData395_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData395_HISTOGRAM_EQUALIZATION, 8},
{6, 26, 880, 1296, 0, gScenarioBufferData396_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData396_HISTOGRAM_EQUALIZATION, 8},
{8, 26, 1088, 1296, 0, gScenarioBufferData397_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData397_HISTOGRAM_EQUALIZATION, 8},
{10, 26, 1296, 1296, 0, gScenarioBufferData398_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData398_HISTOGRAM_EQUALIZATION, 8},
{12, 26, 1504, 1296, 0, gScenarioBufferData399_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData399_HISTOGRAM_EQUALIZATION, 8},
{14, 26, 1712, 1296, 0, gScenarioBufferData400_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData400_HISTOGRAM_EQUALIZATION, 8},
{16, 26, 1920, 1296, 0, gScenarioBufferData401_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData401_HISTOGRAM_EQUALIZATION, 8},
{18, 26, 2128, 1296, 0, gScenarioBufferData402_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData402_HISTOGRAM_EQUALIZATION, 8},
{20, 26, 2336, 1296, 0, gScenarioBufferData403_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData403_HISTOGRAM_EQUALIZATION, 8},
{22, 26, 2544, 1296, 0, gScenarioBufferData404_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData404_HISTOGRAM_EQUALIZATION, 8},
{24, 26, 2752, 1296, 0, gScenarioBufferData405_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData405_HISTOGRAM_EQUALIZATION, 8},
{26, 26, 2960, 1296, 0, gScenarioBufferData406_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData406_HISTOGRAM_EQUALIZATION, 8},
{28, 26, 3168, 1296, 0, gScenarioBufferData407_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData407_HISTOGRAM_EQUALIZATION, 8},
{30, 26, 3376, 1296, 0, gScenarioBufferData408_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData408_HISTOGRAM_EQUALIZATION, 8},
{32, 26, 3584, 1296, 0, gScenarioBufferData409_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData409_HISTOGRAM_EQUALIZATION, 8},
{2, 28, 480, 1296, 0, gScenarioBufferData410_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData410_HISTOGRAM_EQUALIZATION, 8},
{4, 28, 704, 1296, 0, gScenarioBufferData411_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData411_HISTOGRAM_EQUALIZATION, 8},
{6, 28, 928, 1296, 0, gScenarioBufferData412_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData412_HISTOGRAM_EQUALIZATION, 8},
{8, 28, 1152, 1296, 0, gScenarioBufferData413_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData413_HISTOGRAM_EQUALIZATION, 8},
{10, 28, 1376, 1296, 0, gScenarioBufferData414_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData414_HISTOGRAM_EQUALIZATION, 8},
{12, 28, 1600, 1296, 0, gScenarioBufferData415_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData415_HISTOGRAM_EQUALIZATION, 8},
{14, 28, 1824, 1296, 0, gScenarioBufferData416_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData416_HISTOGRAM_EQUALIZATION, 8},
{16, 28, 2048, 1296, 0, gScenarioBufferData417_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData417_HISTOGRAM_EQUALIZATION, 8},
{18, 28, 2272, 1296, 0, gScenarioBufferData418_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData418_HISTOGRAM_EQUALIZATION, 8},
{20, 28, 2496, 1296, 0, gScenarioBufferData419_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData419_HISTOGRAM_EQUALIZATION, 8},
{22, 28, 2720, 1296, 0, gScenarioBufferData420_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData420_HISTOGRAM_EQUALIZATION, 8},
{24, 28, 2944, 1296, 0, gScenarioBufferData421_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData421_HISTOGRAM_EQUALIZATION, 8},
{26, 28, 3168, 1296, 0, gScenarioBufferData422_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData422_HISTOGRAM_EQUALIZATION, 8},
{28, 28, 3392, 1296, 0, gScenarioBufferData423_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData423_HISTOGRAM_EQUALIZATION, 8},
{30, 28, 3616, 1296, 0, gScenarioBufferData424_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData424_HISTOGRAM_EQUALIZATION, 8},
{32, 28, 3840, 1296, 0, gScenarioBufferData425_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData425_HISTOGRAM_EQUALIZATION, 8},
{2, 30, 496, 1296, 0, gScenarioBufferData426_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData426_HISTOGRAM_EQUALIZATION, 8},
{4, 30, 736, 1296, 0, gScenarioBufferData427_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData427_HISTOGRAM_EQUALIZATION, 8},
{6, 30, 976, 1296, 0, gScenarioBufferData428_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData428_HISTOGRAM_EQUALIZATION, 8},
{8, 30, 1216, 1296, 0, gScenarioBufferData429_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData429_HISTOGRAM_EQUALIZATION, 8},
{10, 30, 1456, 1296, 0, gScenarioBufferData430_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData430_HISTOGRAM_EQUALIZATION, 8},
{12, 30, 1696, 1296, 0, gScenarioBufferData431_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData431_HISTOGRAM_EQUALIZATION, 8},
{14, 30, 1936, 1296, 0, gScenarioBufferData432_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData432_HISTOGRAM_EQUALIZATION, 8},
{16, 30, 2176, 1296, 0, gScenarioBufferData433_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData433_HISTOGRAM_EQUALIZATION, 8},
{18, 30, 2416, 1296, 0, gScenarioBufferData434_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData434_HISTOGRAM_EQUALIZATION, 8},
{20, 30, 2656, 1296, 0, gScenarioBufferData435_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData435_HISTOGRAM_EQUALIZATION, 8},
{22, 30, 2896, 1296, 0, gScenarioBufferData436_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData436_HISTOGRAM_EQUALIZATION, 8},
{24, 30, 3136, 1296, 0, gScenarioBufferData437_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData437_HISTOGRAM_EQUALIZATION, 8},
{26, 30, 3376, 1296, 0, gScenarioBufferData438_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData438_HISTOGRAM_EQUALIZATION, 8},
{28, 30, 3616, 1296, 0, gScenarioBufferData439_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData439_HISTOGRAM_EQUALIZATION, 8},
{2, 32, 512, 1296, 0, gScenarioBufferData440_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData440_HISTOGRAM_EQUALIZATION, 8},
{4, 32, 768, 1296, 0, gScenarioBufferData441_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData441_HISTOGRAM_EQUALIZATION, 8},
{6, 32, 1024, 1296, 0, gScenarioBufferData442_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData442_HISTOGRAM_EQUALIZATION, 8},
{8, 32, 1280, 1296, 0, gScenarioBufferData443_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData443_HISTOGRAM_EQUALIZATION, 8},
{10, 32, 1536, 1296, 0, gScenarioBufferData444_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData444_HISTOGRAM_EQUALIZATION, 8},
{12, 32, 1792, 1296, 0, gScenarioBufferData445_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData445_HISTOGRAM_EQUALIZATION, 8},
{14, 32, 2048, 1296, 0, gScenarioBufferData446_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData446_HISTOGRAM_EQUALIZATION, 8},
{16, 32, 2304, 1296, 0, gScenarioBufferData447_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData447_HISTOGRAM_EQUALIZATION, 8},
{18, 32, 2560, 1296, 0, gScenarioBufferData448_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData448_HISTOGRAM_EQUALIZATION, 8},
{20, 32, 2816, 1296, 0, gScenarioBufferData449_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData449_HISTOGRAM_EQUALIZATION, 8},
{22, 32, 3072, 1296, 0, gScenarioBufferData450_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData450_HISTOGRAM_EQUALIZATION, 8},
{24, 32, 3328, 1296, 0, gScenarioBufferData451_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData451_HISTOGRAM_EQUALIZATION, 8},
{26, 32, 3584, 1296, 0, gScenarioBufferData452_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData452_HISTOGRAM_EQUALIZATION, 8},
{28, 32, 3840, 1296, 0, gScenarioBufferData453_HISTOGRAM_EQUALIZATION, 96, gScenarioKernelData453_HISTOGRAM_EQUALIZATION, 8}
};

static acf_scenario_list gScenarioList_HISTOGRAM_EQUALIZATION = {
454, //number of scenarios
gScenarioArray_HISTOGRAM_EQUALIZATION};
//**************************************************************

class HISTOGRAM_EQUALIZATION : public ACF_Process_APU
{

public:
   HISTOGRAM_EQUALIZATION(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HISTOGRAM_EQUALIZATION");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HISTOGRAM_EQUALIZATION",
                                        HISTOGRAM_EQUALIZATION_LOAD_SEGMENTS,
                                        HISTOGRAM_EQUALIZATION_LOAD_PMEM, HISTOGRAM_EQUALIZATION_LOAD_PMEM_SIZE,
                                        HISTOGRAM_EQUALIZATION_LOAD_DMEM, HISTOGRAM_EQUALIZATION_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HISTOGRAM_EQUALIZATION_APEX_LOG_BUFFER, HISTOGRAM_EQUALIZATION_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("NUM_PIXELS", icp::DATATYPE_32U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("IMAGE_HISTOGRAM", icp::DATATYPE_32U, 1, 1, 256, 1, 0, 1, 1, 0, 1, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HISTOGRAM_EQUALIZATION);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION

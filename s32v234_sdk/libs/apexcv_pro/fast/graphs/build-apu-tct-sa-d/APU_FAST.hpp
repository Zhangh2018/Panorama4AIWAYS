#ifndef _ACF_PROCESS_APU_APU_FAST
#define _ACF_PROCESS_APU_APU_FAST

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <APU_FAST_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_APU_FAST[] = {{4, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_APU_FAST[] = {{6, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_APU_FAST[] = {{8, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_APU_FAST[] = {{10, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_APU_FAST[] = {{12, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_APU_FAST[] = {{14, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_APU_FAST[] = {{16, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_APU_FAST[] = {{18, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_APU_FAST[] = {{20, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_APU_FAST[] = {{22, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_APU_FAST[] = {{24, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_APU_FAST[] = {{26, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_APU_FAST[] = {{28, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_APU_FAST[] = {{30, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_APU_FAST[] = {{32, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_APU_FAST[] = {{36, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_APU_FAST[] = {{40, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_APU_FAST[] = {{44, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_APU_FAST[] = {{48, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_APU_FAST[] = {{52, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_APU_FAST[] = {{56, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_APU_FAST[] = {{60, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_APU_FAST[] = {{64, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_APU_FAST[] = {{72, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_APU_FAST[] = {{80, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_APU_FAST[] = {{88, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {88, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_APU_FAST[] = {{96, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {96, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_APU_FAST[] = {{104, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {104, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_APU_FAST[] = {{112, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {112, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_APU_FAST[] = {{120, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {120, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_APU_FAST[] = {{128, 1, 10, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {128, 1, 2, 3, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_APU_FAST[] = {{4, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_APU_FAST[] = {{6, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_APU_FAST[] = {{8, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_APU_FAST[] = {{10, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_APU_FAST[] = {{12, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_APU_FAST[] = {{14, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_APU_FAST[] = {{16, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_APU_FAST[] = {{18, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_APU_FAST[] = {{20, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_APU_FAST[] = {{22, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_APU_FAST[] = {{24, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_APU_FAST[] = {{26, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_APU_FAST[] = {{28, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_APU_FAST[] = {{30, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_APU_FAST[] = {{32, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_APU_FAST[] = {{36, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_APU_FAST[] = {{40, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_APU_FAST[] = {{44, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_APU_FAST[] = {{48, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_APU_FAST[] = {{52, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_APU_FAST[] = {{56, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_APU_FAST[] = {{60, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_APU_FAST[] = {{64, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_APU_FAST[] = {{72, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_APU_FAST[] = {{80, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_APU_FAST[] = {{88, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {88, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_APU_FAST[] = {{96, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {96, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_APU_FAST[] = {{104, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {104, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_APU_FAST[] = {{112, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {112, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_APU_FAST[] = {{120, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {120, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_APU_FAST[] = {{128, 2, 7, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {128, 2, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_APU_FAST[] = {{4, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_APU_FAST[] = {{6, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_APU_FAST[] = {{8, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_APU_FAST[] = {{10, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_APU_FAST[] = {{12, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_APU_FAST[] = {{14, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_APU_FAST[] = {{16, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_APU_FAST[] = {{18, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_APU_FAST[] = {{20, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_APU_FAST[] = {{22, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_APU_FAST[] = {{24, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_APU_FAST[] = {{26, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_APU_FAST[] = {{28, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_APU_FAST[] = {{30, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_APU_FAST[] = {{32, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_APU_FAST[] = {{36, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_APU_FAST[] = {{40, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_APU_FAST[] = {{44, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_APU_FAST[] = {{48, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_APU_FAST[] = {{52, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_APU_FAST[] = {{56, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_APU_FAST[] = {{60, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_APU_FAST[] = {{64, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_APU_FAST[] = {{72, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_APU_FAST[] = {{80, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_APU_FAST[] = {{88, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {88, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_APU_FAST[] = {{96, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {96, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_APU_FAST[] = {{104, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {104, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_APU_FAST[] = {{112, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {112, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_APU_FAST[] = {{120, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {120, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_APU_FAST[] = {{128, 3, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {128, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_APU_FAST[] = {{4, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_APU_FAST[] = {{6, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_APU_FAST[] = {{8, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_APU_FAST[] = {{10, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_APU_FAST[] = {{12, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_APU_FAST[] = {{14, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_APU_FAST[] = {{16, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_APU_FAST[] = {{18, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_APU_FAST[] = {{20, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_APU_FAST[] = {{22, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_APU_FAST[] = {{24, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_APU_FAST[] = {{26, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_APU_FAST[] = {{28, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_APU_FAST[] = {{30, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_APU_FAST[] = {{32, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_APU_FAST[] = {{36, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_APU_FAST[] = {{40, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_APU_FAST[] = {{44, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_APU_FAST[] = {{48, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_APU_FAST[] = {{52, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_APU_FAST[] = {{56, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_APU_FAST[] = {{60, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_APU_FAST[] = {{64, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_APU_FAST[] = {{72, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_APU_FAST[] = {{80, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_APU_FAST[] = {{88, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {88, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_APU_FAST[] = {{96, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {96, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_APU_FAST[] = {{104, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {104, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_APU_FAST[] = {{112, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {112, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_APU_FAST[] = {{120, 4, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {120, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_APU_FAST[] = {{4, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_APU_FAST[] = {{6, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_APU_FAST[] = {{8, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_APU_FAST[] = {{10, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_APU_FAST[] = {{12, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_APU_FAST[] = {{14, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_APU_FAST[] = {{16, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_APU_FAST[] = {{18, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_APU_FAST[] = {{20, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_APU_FAST[] = {{22, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_APU_FAST[] = {{24, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_APU_FAST[] = {{26, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_APU_FAST[] = {{28, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_APU_FAST[] = {{30, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_APU_FAST[] = {{32, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_APU_FAST[] = {{36, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_APU_FAST[] = {{40, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_APU_FAST[] = {{44, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_APU_FAST[] = {{48, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_APU_FAST[] = {{52, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_APU_FAST[] = {{56, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_APU_FAST[] = {{60, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_APU_FAST[] = {{64, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_APU_FAST[] = {{72, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_APU_FAST[] = {{80, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_APU_FAST[] = {{88, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {88, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_APU_FAST[] = {{96, 5, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {96, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_APU_FAST[] = {{4, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_APU_FAST[] = {{6, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_APU_FAST[] = {{8, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_APU_FAST[] = {{10, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_APU_FAST[] = {{12, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_APU_FAST[] = {{14, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_APU_FAST[] = {{16, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_APU_FAST[] = {{18, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_APU_FAST[] = {{20, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_APU_FAST[] = {{22, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_APU_FAST[] = {{24, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_APU_FAST[] = {{26, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_APU_FAST[] = {{28, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_APU_FAST[] = {{30, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_APU_FAST[] = {{32, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_APU_FAST[] = {{36, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_APU_FAST[] = {{40, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_APU_FAST[] = {{44, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_APU_FAST[] = {{48, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_APU_FAST[] = {{52, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_APU_FAST[] = {{56, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_APU_FAST[] = {{60, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_APU_FAST[] = {{64, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_APU_FAST[] = {{72, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {72, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_APU_FAST[] = {{80, 6, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {80, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_APU_FAST[] = {{4, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_APU_FAST[] = {{6, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_APU_FAST[] = {{8, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_APU_FAST[] = {{10, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_APU_FAST[] = {{12, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_APU_FAST[] = {{14, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_APU_FAST[] = {{16, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_APU_FAST[] = {{18, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_APU_FAST[] = {{20, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_APU_FAST[] = {{22, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_APU_FAST[] = {{24, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_APU_FAST[] = {{26, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_APU_FAST[] = {{28, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_APU_FAST[] = {{30, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_APU_FAST[] = {{32, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_APU_FAST[] = {{36, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_APU_FAST[] = {{40, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_APU_FAST[] = {{44, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_APU_FAST[] = {{48, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_APU_FAST[] = {{52, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_APU_FAST[] = {{56, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_APU_FAST[] = {{60, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_APU_FAST[] = {{64, 8, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_APU_FAST[] = {{4, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_APU_FAST[] = {{6, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_APU_FAST[] = {{8, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_APU_FAST[] = {{10, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_APU_FAST[] = {{12, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_APU_FAST[] = {{14, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_APU_FAST[] = {{16, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_APU_FAST[] = {{18, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_APU_FAST[] = {{20, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_APU_FAST[] = {{22, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_APU_FAST[] = {{24, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_APU_FAST[] = {{26, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_APU_FAST[] = {{28, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_APU_FAST[] = {{30, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_APU_FAST[] = {{32, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_APU_FAST[] = {{36, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_APU_FAST[] = {{40, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_APU_FAST[] = {{44, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_APU_FAST[] = {{48, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_APU_FAST[] = {{52, 10, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_APU_FAST[] = {{4, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_APU_FAST[] = {{6, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_APU_FAST[] = {{8, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_APU_FAST[] = {{10, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_APU_FAST[] = {{12, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_APU_FAST[] = {{14, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_APU_FAST[] = {{16, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_APU_FAST[] = {{18, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_APU_FAST[] = {{20, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_APU_FAST[] = {{22, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_APU_FAST[] = {{24, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_APU_FAST[] = {{26, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_APU_FAST[] = {{28, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_APU_FAST[] = {{30, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_APU_FAST[] = {{32, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_APU_FAST[] = {{36, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_APU_FAST[] = {{40, 12, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_APU_FAST[] = {{4, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_APU_FAST[] = {{6, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_APU_FAST[] = {{8, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_APU_FAST[] = {{10, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_APU_FAST[] = {{12, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_APU_FAST[] = {{14, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_APU_FAST[] = {{16, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_APU_FAST[] = {{18, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_APU_FAST[] = {{20, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_APU_FAST[] = {{22, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_APU_FAST[] = {{24, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_APU_FAST[] = {{26, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_APU_FAST[] = {{28, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_APU_FAST[] = {{30, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_APU_FAST[] = {{32, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_APU_FAST[] = {{36, 14, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_APU_FAST[] = {{4, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_APU_FAST[] = {{6, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_APU_FAST[] = {{8, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_APU_FAST[] = {{10, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_APU_FAST[] = {{12, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_APU_FAST[] = {{14, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_APU_FAST[] = {{16, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_APU_FAST[] = {{18, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_APU_FAST[] = {{20, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_APU_FAST[] = {{22, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_APU_FAST[] = {{24, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_APU_FAST[] = {{26, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_APU_FAST[] = {{28, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_APU_FAST[] = {{30, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {30, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_APU_FAST[] = {{32, 16, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_APU_FAST[] = {{4, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_APU_FAST[] = {{6, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_APU_FAST[] = {{8, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_APU_FAST[] = {{10, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_APU_FAST[] = {{12, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_APU_FAST[] = {{14, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_APU_FAST[] = {{16, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_APU_FAST[] = {{18, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_APU_FAST[] = {{20, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_APU_FAST[] = {{22, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_APU_FAST[] = {{24, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_APU_FAST[] = {{26, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {26, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_APU_FAST[] = {{28, 18, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_APU_FAST[] = {{4, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_APU_FAST[] = {{6, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_APU_FAST[] = {{8, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_APU_FAST[] = {{10, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_APU_FAST[] = {{12, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_APU_FAST[] = {{14, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_APU_FAST[] = {{16, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_APU_FAST[] = {{18, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_APU_FAST[] = {{20, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_APU_FAST[] = {{22, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_APU_FAST[] = {{24, 20, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_APU_FAST[] = {{4, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_APU_FAST[] = {{6, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_APU_FAST[] = {{8, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_APU_FAST[] = {{10, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_APU_FAST[] = {{12, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_APU_FAST[] = {{14, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_APU_FAST[] = {{16, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_APU_FAST[] = {{18, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_APU_FAST[] = {{20, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_APU_FAST[] = {{22, 22, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {22, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_APU_FAST[] = {{4, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_APU_FAST[] = {{6, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_APU_FAST[] = {{8, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_APU_FAST[] = {{10, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_APU_FAST[] = {{12, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_APU_FAST[] = {{14, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_APU_FAST[] = {{16, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_APU_FAST[] = {{18, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_APU_FAST[] = {{20, 24, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_APU_FAST[] = {{4, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_APU_FAST[] = {{6, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_APU_FAST[] = {{8, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_APU_FAST[] = {{10, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_APU_FAST[] = {{12, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_APU_FAST[] = {{14, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_APU_FAST[] = {{16, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_APU_FAST[] = {{18, 26, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {18, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_APU_FAST[] = {{4, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_APU_FAST[] = {{6, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_APU_FAST[] = {{8, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_APU_FAST[] = {{10, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_APU_FAST[] = {{12, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_APU_FAST[] = {{14, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_APU_FAST[] = {{16, 28, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_APU_FAST[] = {{4, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_APU_FAST[] = {{6, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_APU_FAST[] = {{8, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_APU_FAST[] = {{10, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_APU_FAST[] = {{12, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_APU_FAST[] = {{14, 30, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {14, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_APU_FAST[] = {{4, 32, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_APU_FAST[] = {{6, 32, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_APU_FAST[] = {{8, 32, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_APU_FAST[] = {{10, 32, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_APU_FAST[] = {{12, 32, 4, 0, {3, 3, 4, 4}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_APU_FAST[] = {{0, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_APU_FAST[] = {{0, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_APU_FAST[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_APU_FAST[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_APU_FAST[] = {
{4, 1, 200, 96, 3, gScenarioBufferData0_APU_FAST, 80, gScenarioKernelData0_APU_FAST, 8},
{6, 1, 236, 96, 3, gScenarioBufferData1_APU_FAST, 80, gScenarioKernelData1_APU_FAST, 8},
{8, 1, 272, 96, 3, gScenarioBufferData2_APU_FAST, 80, gScenarioKernelData2_APU_FAST, 8},
{10, 1, 308, 96, 3, gScenarioBufferData3_APU_FAST, 80, gScenarioKernelData3_APU_FAST, 8},
{12, 1, 344, 96, 3, gScenarioBufferData4_APU_FAST, 80, gScenarioKernelData4_APU_FAST, 8},
{14, 1, 380, 96, 3, gScenarioBufferData5_APU_FAST, 80, gScenarioKernelData5_APU_FAST, 8},
{16, 1, 416, 96, 3, gScenarioBufferData6_APU_FAST, 80, gScenarioKernelData6_APU_FAST, 8},
{18, 1, 452, 96, 3, gScenarioBufferData7_APU_FAST, 80, gScenarioKernelData7_APU_FAST, 8},
{20, 1, 488, 96, 3, gScenarioBufferData8_APU_FAST, 80, gScenarioKernelData8_APU_FAST, 8},
{22, 1, 524, 96, 3, gScenarioBufferData9_APU_FAST, 80, gScenarioKernelData9_APU_FAST, 8},
{24, 1, 560, 96, 3, gScenarioBufferData10_APU_FAST, 80, gScenarioKernelData10_APU_FAST, 8},
{26, 1, 596, 96, 3, gScenarioBufferData11_APU_FAST, 80, gScenarioKernelData11_APU_FAST, 8},
{28, 1, 632, 96, 3, gScenarioBufferData12_APU_FAST, 80, gScenarioKernelData12_APU_FAST, 8},
{30, 1, 668, 96, 3, gScenarioBufferData13_APU_FAST, 80, gScenarioKernelData13_APU_FAST, 8},
{32, 1, 704, 96, 3, gScenarioBufferData14_APU_FAST, 80, gScenarioKernelData14_APU_FAST, 8},
{36, 1, 776, 96, 3, gScenarioBufferData15_APU_FAST, 80, gScenarioKernelData15_APU_FAST, 8},
{40, 1, 848, 96, 3, gScenarioBufferData16_APU_FAST, 80, gScenarioKernelData16_APU_FAST, 8},
{44, 1, 920, 96, 3, gScenarioBufferData17_APU_FAST, 80, gScenarioKernelData17_APU_FAST, 8},
{48, 1, 992, 96, 3, gScenarioBufferData18_APU_FAST, 80, gScenarioKernelData18_APU_FAST, 8},
{52, 1, 1064, 96, 3, gScenarioBufferData19_APU_FAST, 80, gScenarioKernelData19_APU_FAST, 8},
{56, 1, 1136, 96, 3, gScenarioBufferData20_APU_FAST, 80, gScenarioKernelData20_APU_FAST, 8},
{60, 1, 1208, 96, 3, gScenarioBufferData21_APU_FAST, 80, gScenarioKernelData21_APU_FAST, 8},
{64, 1, 1280, 96, 3, gScenarioBufferData22_APU_FAST, 80, gScenarioKernelData22_APU_FAST, 8},
{72, 1, 1424, 96, 3, gScenarioBufferData23_APU_FAST, 80, gScenarioKernelData23_APU_FAST, 8},
{80, 1, 1568, 96, 3, gScenarioBufferData24_APU_FAST, 80, gScenarioKernelData24_APU_FAST, 8},
{88, 1, 1712, 96, 3, gScenarioBufferData25_APU_FAST, 80, gScenarioKernelData25_APU_FAST, 8},
{96, 1, 1856, 96, 3, gScenarioBufferData26_APU_FAST, 80, gScenarioKernelData26_APU_FAST, 8},
{104, 1, 2000, 96, 3, gScenarioBufferData27_APU_FAST, 80, gScenarioKernelData27_APU_FAST, 8},
{112, 1, 2144, 96, 3, gScenarioBufferData28_APU_FAST, 80, gScenarioKernelData28_APU_FAST, 8},
{120, 1, 2288, 96, 3, gScenarioBufferData29_APU_FAST, 80, gScenarioKernelData29_APU_FAST, 8},
{128, 1, 2432, 96, 3, gScenarioBufferData30_APU_FAST, 80, gScenarioKernelData30_APU_FAST, 8},
{4, 2, 256, 96, 2, gScenarioBufferData31_APU_FAST, 80, gScenarioKernelData31_APU_FAST, 8},
{6, 2, 304, 96, 2, gScenarioBufferData32_APU_FAST, 80, gScenarioKernelData32_APU_FAST, 8},
{8, 2, 352, 96, 2, gScenarioBufferData33_APU_FAST, 80, gScenarioKernelData33_APU_FAST, 8},
{10, 2, 400, 96, 2, gScenarioBufferData34_APU_FAST, 80, gScenarioKernelData34_APU_FAST, 8},
{12, 2, 448, 96, 2, gScenarioBufferData35_APU_FAST, 80, gScenarioKernelData35_APU_FAST, 8},
{14, 2, 496, 96, 2, gScenarioBufferData36_APU_FAST, 80, gScenarioKernelData36_APU_FAST, 8},
{16, 2, 544, 96, 2, gScenarioBufferData37_APU_FAST, 80, gScenarioKernelData37_APU_FAST, 8},
{18, 2, 592, 96, 2, gScenarioBufferData38_APU_FAST, 80, gScenarioKernelData38_APU_FAST, 8},
{20, 2, 640, 96, 2, gScenarioBufferData39_APU_FAST, 80, gScenarioKernelData39_APU_FAST, 8},
{22, 2, 688, 96, 2, gScenarioBufferData40_APU_FAST, 80, gScenarioKernelData40_APU_FAST, 8},
{24, 2, 736, 96, 2, gScenarioBufferData41_APU_FAST, 80, gScenarioKernelData41_APU_FAST, 8},
{26, 2, 784, 96, 2, gScenarioBufferData42_APU_FAST, 80, gScenarioKernelData42_APU_FAST, 8},
{28, 2, 832, 96, 2, gScenarioBufferData43_APU_FAST, 80, gScenarioKernelData43_APU_FAST, 8},
{30, 2, 880, 96, 2, gScenarioBufferData44_APU_FAST, 80, gScenarioKernelData44_APU_FAST, 8},
{32, 2, 928, 96, 2, gScenarioBufferData45_APU_FAST, 80, gScenarioKernelData45_APU_FAST, 8},
{36, 2, 1024, 96, 2, gScenarioBufferData46_APU_FAST, 80, gScenarioKernelData46_APU_FAST, 8},
{40, 2, 1120, 96, 2, gScenarioBufferData47_APU_FAST, 80, gScenarioKernelData47_APU_FAST, 8},
{44, 2, 1216, 96, 2, gScenarioBufferData48_APU_FAST, 80, gScenarioKernelData48_APU_FAST, 8},
{48, 2, 1312, 96, 2, gScenarioBufferData49_APU_FAST, 80, gScenarioKernelData49_APU_FAST, 8},
{52, 2, 1408, 96, 2, gScenarioBufferData50_APU_FAST, 80, gScenarioKernelData50_APU_FAST, 8},
{56, 2, 1504, 96, 2, gScenarioBufferData51_APU_FAST, 80, gScenarioKernelData51_APU_FAST, 8},
{60, 2, 1600, 96, 2, gScenarioBufferData52_APU_FAST, 80, gScenarioKernelData52_APU_FAST, 8},
{64, 2, 1696, 96, 2, gScenarioBufferData53_APU_FAST, 80, gScenarioKernelData53_APU_FAST, 8},
{72, 2, 1888, 96, 2, gScenarioBufferData54_APU_FAST, 80, gScenarioKernelData54_APU_FAST, 8},
{80, 2, 2080, 96, 2, gScenarioBufferData55_APU_FAST, 80, gScenarioKernelData55_APU_FAST, 8},
{88, 2, 2272, 96, 2, gScenarioBufferData56_APU_FAST, 80, gScenarioKernelData56_APU_FAST, 8},
{96, 2, 2464, 96, 2, gScenarioBufferData57_APU_FAST, 80, gScenarioKernelData57_APU_FAST, 8},
{104, 2, 2656, 96, 2, gScenarioBufferData58_APU_FAST, 80, gScenarioKernelData58_APU_FAST, 8},
{112, 2, 2848, 96, 2, gScenarioBufferData59_APU_FAST, 80, gScenarioKernelData59_APU_FAST, 8},
{120, 2, 3040, 96, 2, gScenarioBufferData60_APU_FAST, 80, gScenarioKernelData60_APU_FAST, 8},
{128, 2, 3232, 96, 2, gScenarioBufferData61_APU_FAST, 80, gScenarioKernelData61_APU_FAST, 8},
{4, 3, 240, 96, 1, gScenarioBufferData62_APU_FAST, 80, gScenarioKernelData62_APU_FAST, 8},
{6, 3, 288, 96, 1, gScenarioBufferData63_APU_FAST, 80, gScenarioKernelData63_APU_FAST, 8},
{8, 3, 336, 96, 1, gScenarioBufferData64_APU_FAST, 80, gScenarioKernelData64_APU_FAST, 8},
{10, 3, 384, 96, 1, gScenarioBufferData65_APU_FAST, 80, gScenarioKernelData65_APU_FAST, 8},
{12, 3, 432, 96, 1, gScenarioBufferData66_APU_FAST, 80, gScenarioKernelData66_APU_FAST, 8},
{14, 3, 480, 96, 1, gScenarioBufferData67_APU_FAST, 80, gScenarioKernelData67_APU_FAST, 8},
{16, 3, 528, 96, 1, gScenarioBufferData68_APU_FAST, 80, gScenarioKernelData68_APU_FAST, 8},
{18, 3, 576, 96, 1, gScenarioBufferData69_APU_FAST, 80, gScenarioKernelData69_APU_FAST, 8},
{20, 3, 624, 96, 1, gScenarioBufferData70_APU_FAST, 80, gScenarioKernelData70_APU_FAST, 8},
{22, 3, 672, 96, 1, gScenarioBufferData71_APU_FAST, 80, gScenarioKernelData71_APU_FAST, 8},
{24, 3, 720, 96, 1, gScenarioBufferData72_APU_FAST, 80, gScenarioKernelData72_APU_FAST, 8},
{26, 3, 768, 96, 1, gScenarioBufferData73_APU_FAST, 80, gScenarioKernelData73_APU_FAST, 8},
{28, 3, 816, 96, 1, gScenarioBufferData74_APU_FAST, 80, gScenarioKernelData74_APU_FAST, 8},
{30, 3, 864, 96, 1, gScenarioBufferData75_APU_FAST, 80, gScenarioKernelData75_APU_FAST, 8},
{32, 3, 912, 96, 1, gScenarioBufferData76_APU_FAST, 80, gScenarioKernelData76_APU_FAST, 8},
{36, 3, 1008, 96, 1, gScenarioBufferData77_APU_FAST, 80, gScenarioKernelData77_APU_FAST, 8},
{40, 3, 1104, 96, 1, gScenarioBufferData78_APU_FAST, 80, gScenarioKernelData78_APU_FAST, 8},
{44, 3, 1200, 96, 1, gScenarioBufferData79_APU_FAST, 80, gScenarioKernelData79_APU_FAST, 8},
{48, 3, 1296, 96, 1, gScenarioBufferData80_APU_FAST, 80, gScenarioKernelData80_APU_FAST, 8},
{52, 3, 1392, 96, 1, gScenarioBufferData81_APU_FAST, 80, gScenarioKernelData81_APU_FAST, 8},
{56, 3, 1488, 96, 1, gScenarioBufferData82_APU_FAST, 80, gScenarioKernelData82_APU_FAST, 8},
{60, 3, 1584, 96, 1, gScenarioBufferData83_APU_FAST, 80, gScenarioKernelData83_APU_FAST, 8},
{64, 3, 1680, 96, 1, gScenarioBufferData84_APU_FAST, 80, gScenarioKernelData84_APU_FAST, 8},
{72, 3, 1872, 96, 1, gScenarioBufferData85_APU_FAST, 80, gScenarioKernelData85_APU_FAST, 8},
{80, 3, 2064, 96, 1, gScenarioBufferData86_APU_FAST, 80, gScenarioKernelData86_APU_FAST, 8},
{88, 3, 2256, 96, 1, gScenarioBufferData87_APU_FAST, 80, gScenarioKernelData87_APU_FAST, 8},
{96, 3, 2448, 96, 1, gScenarioBufferData88_APU_FAST, 80, gScenarioKernelData88_APU_FAST, 8},
{104, 3, 2640, 96, 1, gScenarioBufferData89_APU_FAST, 80, gScenarioKernelData89_APU_FAST, 8},
{112, 3, 2832, 96, 1, gScenarioBufferData90_APU_FAST, 80, gScenarioKernelData90_APU_FAST, 8},
{120, 3, 3024, 96, 1, gScenarioBufferData91_APU_FAST, 80, gScenarioKernelData91_APU_FAST, 8},
{128, 3, 3216, 96, 1, gScenarioBufferData92_APU_FAST, 80, gScenarioKernelData92_APU_FAST, 8},
{4, 4, 296, 96, 1, gScenarioBufferData93_APU_FAST, 80, gScenarioKernelData93_APU_FAST, 8},
{6, 4, 356, 96, 1, gScenarioBufferData94_APU_FAST, 80, gScenarioKernelData94_APU_FAST, 8},
{8, 4, 416, 96, 1, gScenarioBufferData95_APU_FAST, 80, gScenarioKernelData95_APU_FAST, 8},
{10, 4, 476, 96, 1, gScenarioBufferData96_APU_FAST, 80, gScenarioKernelData96_APU_FAST, 8},
{12, 4, 536, 96, 1, gScenarioBufferData97_APU_FAST, 80, gScenarioKernelData97_APU_FAST, 8},
{14, 4, 596, 96, 1, gScenarioBufferData98_APU_FAST, 80, gScenarioKernelData98_APU_FAST, 8},
{16, 4, 656, 96, 1, gScenarioBufferData99_APU_FAST, 80, gScenarioKernelData99_APU_FAST, 8},
{18, 4, 716, 96, 1, gScenarioBufferData100_APU_FAST, 80, gScenarioKernelData100_APU_FAST, 8},
{20, 4, 776, 96, 1, gScenarioBufferData101_APU_FAST, 80, gScenarioKernelData101_APU_FAST, 8},
{22, 4, 836, 96, 1, gScenarioBufferData102_APU_FAST, 80, gScenarioKernelData102_APU_FAST, 8},
{24, 4, 896, 96, 1, gScenarioBufferData103_APU_FAST, 80, gScenarioKernelData103_APU_FAST, 8},
{26, 4, 956, 96, 1, gScenarioBufferData104_APU_FAST, 80, gScenarioKernelData104_APU_FAST, 8},
{28, 4, 1016, 96, 1, gScenarioBufferData105_APU_FAST, 80, gScenarioKernelData105_APU_FAST, 8},
{30, 4, 1076, 96, 1, gScenarioBufferData106_APU_FAST, 80, gScenarioKernelData106_APU_FAST, 8},
{32, 4, 1136, 96, 1, gScenarioBufferData107_APU_FAST, 80, gScenarioKernelData107_APU_FAST, 8},
{36, 4, 1256, 96, 1, gScenarioBufferData108_APU_FAST, 80, gScenarioKernelData108_APU_FAST, 8},
{40, 4, 1376, 96, 1, gScenarioBufferData109_APU_FAST, 80, gScenarioKernelData109_APU_FAST, 8},
{44, 4, 1496, 96, 1, gScenarioBufferData110_APU_FAST, 80, gScenarioKernelData110_APU_FAST, 8},
{48, 4, 1616, 96, 1, gScenarioBufferData111_APU_FAST, 80, gScenarioKernelData111_APU_FAST, 8},
{52, 4, 1736, 96, 1, gScenarioBufferData112_APU_FAST, 80, gScenarioKernelData112_APU_FAST, 8},
{56, 4, 1856, 96, 1, gScenarioBufferData113_APU_FAST, 80, gScenarioKernelData113_APU_FAST, 8},
{60, 4, 1976, 96, 1, gScenarioBufferData114_APU_FAST, 80, gScenarioKernelData114_APU_FAST, 8},
{64, 4, 2096, 96, 1, gScenarioBufferData115_APU_FAST, 80, gScenarioKernelData115_APU_FAST, 8},
{72, 4, 2336, 96, 1, gScenarioBufferData116_APU_FAST, 80, gScenarioKernelData116_APU_FAST, 8},
{80, 4, 2576, 96, 1, gScenarioBufferData117_APU_FAST, 80, gScenarioKernelData117_APU_FAST, 8},
{88, 4, 2816, 96, 1, gScenarioBufferData118_APU_FAST, 80, gScenarioKernelData118_APU_FAST, 8},
{96, 4, 3056, 96, 1, gScenarioBufferData119_APU_FAST, 80, gScenarioKernelData119_APU_FAST, 8},
{104, 4, 3296, 96, 1, gScenarioBufferData120_APU_FAST, 80, gScenarioKernelData120_APU_FAST, 8},
{112, 4, 3536, 96, 1, gScenarioBufferData121_APU_FAST, 80, gScenarioKernelData121_APU_FAST, 8},
{120, 4, 3776, 96, 1, gScenarioBufferData122_APU_FAST, 80, gScenarioKernelData122_APU_FAST, 8},
{4, 5, 352, 96, 1, gScenarioBufferData123_APU_FAST, 80, gScenarioKernelData123_APU_FAST, 8},
{6, 5, 424, 96, 1, gScenarioBufferData124_APU_FAST, 80, gScenarioKernelData124_APU_FAST, 8},
{8, 5, 496, 96, 1, gScenarioBufferData125_APU_FAST, 80, gScenarioKernelData125_APU_FAST, 8},
{10, 5, 568, 96, 1, gScenarioBufferData126_APU_FAST, 80, gScenarioKernelData126_APU_FAST, 8},
{12, 5, 640, 96, 1, gScenarioBufferData127_APU_FAST, 80, gScenarioKernelData127_APU_FAST, 8},
{14, 5, 712, 96, 1, gScenarioBufferData128_APU_FAST, 80, gScenarioKernelData128_APU_FAST, 8},
{16, 5, 784, 96, 1, gScenarioBufferData129_APU_FAST, 80, gScenarioKernelData129_APU_FAST, 8},
{18, 5, 856, 96, 1, gScenarioBufferData130_APU_FAST, 80, gScenarioKernelData130_APU_FAST, 8},
{20, 5, 928, 96, 1, gScenarioBufferData131_APU_FAST, 80, gScenarioKernelData131_APU_FAST, 8},
{22, 5, 1000, 96, 1, gScenarioBufferData132_APU_FAST, 80, gScenarioKernelData132_APU_FAST, 8},
{24, 5, 1072, 96, 1, gScenarioBufferData133_APU_FAST, 80, gScenarioKernelData133_APU_FAST, 8},
{26, 5, 1144, 96, 1, gScenarioBufferData134_APU_FAST, 80, gScenarioKernelData134_APU_FAST, 8},
{28, 5, 1216, 96, 1, gScenarioBufferData135_APU_FAST, 80, gScenarioKernelData135_APU_FAST, 8},
{30, 5, 1288, 96, 1, gScenarioBufferData136_APU_FAST, 80, gScenarioKernelData136_APU_FAST, 8},
{32, 5, 1360, 96, 1, gScenarioBufferData137_APU_FAST, 80, gScenarioKernelData137_APU_FAST, 8},
{36, 5, 1504, 96, 1, gScenarioBufferData138_APU_FAST, 80, gScenarioKernelData138_APU_FAST, 8},
{40, 5, 1648, 96, 1, gScenarioBufferData139_APU_FAST, 80, gScenarioKernelData139_APU_FAST, 8},
{44, 5, 1792, 96, 1, gScenarioBufferData140_APU_FAST, 80, gScenarioKernelData140_APU_FAST, 8},
{48, 5, 1936, 96, 1, gScenarioBufferData141_APU_FAST, 80, gScenarioKernelData141_APU_FAST, 8},
{52, 5, 2080, 96, 1, gScenarioBufferData142_APU_FAST, 80, gScenarioKernelData142_APU_FAST, 8},
{56, 5, 2224, 96, 1, gScenarioBufferData143_APU_FAST, 80, gScenarioKernelData143_APU_FAST, 8},
{60, 5, 2368, 96, 1, gScenarioBufferData144_APU_FAST, 80, gScenarioKernelData144_APU_FAST, 8},
{64, 5, 2512, 96, 1, gScenarioBufferData145_APU_FAST, 80, gScenarioKernelData145_APU_FAST, 8},
{72, 5, 2800, 96, 1, gScenarioBufferData146_APU_FAST, 80, gScenarioKernelData146_APU_FAST, 8},
{80, 5, 3088, 96, 1, gScenarioBufferData147_APU_FAST, 80, gScenarioKernelData147_APU_FAST, 8},
{88, 5, 3376, 96, 1, gScenarioBufferData148_APU_FAST, 80, gScenarioKernelData148_APU_FAST, 8},
{96, 5, 3664, 96, 1, gScenarioBufferData149_APU_FAST, 80, gScenarioKernelData149_APU_FAST, 8},
{4, 6, 408, 96, 1, gScenarioBufferData150_APU_FAST, 80, gScenarioKernelData150_APU_FAST, 8},
{6, 6, 492, 96, 1, gScenarioBufferData151_APU_FAST, 80, gScenarioKernelData151_APU_FAST, 8},
{8, 6, 576, 96, 1, gScenarioBufferData152_APU_FAST, 80, gScenarioKernelData152_APU_FAST, 8},
{10, 6, 660, 96, 1, gScenarioBufferData153_APU_FAST, 80, gScenarioKernelData153_APU_FAST, 8},
{12, 6, 744, 96, 1, gScenarioBufferData154_APU_FAST, 80, gScenarioKernelData154_APU_FAST, 8},
{14, 6, 828, 96, 1, gScenarioBufferData155_APU_FAST, 80, gScenarioKernelData155_APU_FAST, 8},
{16, 6, 912, 96, 1, gScenarioBufferData156_APU_FAST, 80, gScenarioKernelData156_APU_FAST, 8},
{18, 6, 996, 96, 1, gScenarioBufferData157_APU_FAST, 80, gScenarioKernelData157_APU_FAST, 8},
{20, 6, 1080, 96, 1, gScenarioBufferData158_APU_FAST, 80, gScenarioKernelData158_APU_FAST, 8},
{22, 6, 1164, 96, 1, gScenarioBufferData159_APU_FAST, 80, gScenarioKernelData159_APU_FAST, 8},
{24, 6, 1248, 96, 1, gScenarioBufferData160_APU_FAST, 80, gScenarioKernelData160_APU_FAST, 8},
{26, 6, 1332, 96, 1, gScenarioBufferData161_APU_FAST, 80, gScenarioKernelData161_APU_FAST, 8},
{28, 6, 1416, 96, 1, gScenarioBufferData162_APU_FAST, 80, gScenarioKernelData162_APU_FAST, 8},
{30, 6, 1500, 96, 1, gScenarioBufferData163_APU_FAST, 80, gScenarioKernelData163_APU_FAST, 8},
{32, 6, 1584, 96, 1, gScenarioBufferData164_APU_FAST, 80, gScenarioKernelData164_APU_FAST, 8},
{36, 6, 1752, 96, 1, gScenarioBufferData165_APU_FAST, 80, gScenarioKernelData165_APU_FAST, 8},
{40, 6, 1920, 96, 1, gScenarioBufferData166_APU_FAST, 80, gScenarioKernelData166_APU_FAST, 8},
{44, 6, 2088, 96, 1, gScenarioBufferData167_APU_FAST, 80, gScenarioKernelData167_APU_FAST, 8},
{48, 6, 2256, 96, 1, gScenarioBufferData168_APU_FAST, 80, gScenarioKernelData168_APU_FAST, 8},
{52, 6, 2424, 96, 1, gScenarioBufferData169_APU_FAST, 80, gScenarioKernelData169_APU_FAST, 8},
{56, 6, 2592, 96, 1, gScenarioBufferData170_APU_FAST, 80, gScenarioKernelData170_APU_FAST, 8},
{60, 6, 2760, 96, 1, gScenarioBufferData171_APU_FAST, 80, gScenarioKernelData171_APU_FAST, 8},
{64, 6, 2928, 96, 1, gScenarioBufferData172_APU_FAST, 80, gScenarioKernelData172_APU_FAST, 8},
{72, 6, 3264, 96, 1, gScenarioBufferData173_APU_FAST, 80, gScenarioKernelData173_APU_FAST, 8},
{80, 6, 3600, 96, 1, gScenarioBufferData174_APU_FAST, 80, gScenarioKernelData174_APU_FAST, 8},
{4, 8, 520, 96, 1, gScenarioBufferData175_APU_FAST, 80, gScenarioKernelData175_APU_FAST, 8},
{6, 8, 628, 96, 1, gScenarioBufferData176_APU_FAST, 80, gScenarioKernelData176_APU_FAST, 8},
{8, 8, 736, 96, 1, gScenarioBufferData177_APU_FAST, 80, gScenarioKernelData177_APU_FAST, 8},
{10, 8, 844, 96, 1, gScenarioBufferData178_APU_FAST, 80, gScenarioKernelData178_APU_FAST, 8},
{12, 8, 952, 96, 1, gScenarioBufferData179_APU_FAST, 80, gScenarioKernelData179_APU_FAST, 8},
{14, 8, 1060, 96, 1, gScenarioBufferData180_APU_FAST, 80, gScenarioKernelData180_APU_FAST, 8},
{16, 8, 1168, 96, 1, gScenarioBufferData181_APU_FAST, 80, gScenarioKernelData181_APU_FAST, 8},
{18, 8, 1276, 96, 1, gScenarioBufferData182_APU_FAST, 80, gScenarioKernelData182_APU_FAST, 8},
{20, 8, 1384, 96, 1, gScenarioBufferData183_APU_FAST, 80, gScenarioKernelData183_APU_FAST, 8},
{22, 8, 1492, 96, 1, gScenarioBufferData184_APU_FAST, 80, gScenarioKernelData184_APU_FAST, 8},
{24, 8, 1600, 96, 1, gScenarioBufferData185_APU_FAST, 80, gScenarioKernelData185_APU_FAST, 8},
{26, 8, 1708, 96, 1, gScenarioBufferData186_APU_FAST, 80, gScenarioKernelData186_APU_FAST, 8},
{28, 8, 1816, 96, 1, gScenarioBufferData187_APU_FAST, 80, gScenarioKernelData187_APU_FAST, 8},
{30, 8, 1924, 96, 1, gScenarioBufferData188_APU_FAST, 80, gScenarioKernelData188_APU_FAST, 8},
{32, 8, 2032, 96, 1, gScenarioBufferData189_APU_FAST, 80, gScenarioKernelData189_APU_FAST, 8},
{36, 8, 2248, 96, 1, gScenarioBufferData190_APU_FAST, 80, gScenarioKernelData190_APU_FAST, 8},
{40, 8, 2464, 96, 1, gScenarioBufferData191_APU_FAST, 80, gScenarioKernelData191_APU_FAST, 8},
{44, 8, 2680, 96, 1, gScenarioBufferData192_APU_FAST, 80, gScenarioKernelData192_APU_FAST, 8},
{48, 8, 2896, 96, 1, gScenarioBufferData193_APU_FAST, 80, gScenarioKernelData193_APU_FAST, 8},
{52, 8, 3112, 96, 1, gScenarioBufferData194_APU_FAST, 80, gScenarioKernelData194_APU_FAST, 8},
{56, 8, 3328, 96, 1, gScenarioBufferData195_APU_FAST, 80, gScenarioKernelData195_APU_FAST, 8},
{60, 8, 3544, 96, 1, gScenarioBufferData196_APU_FAST, 80, gScenarioKernelData196_APU_FAST, 8},
{64, 8, 3760, 96, 1, gScenarioBufferData197_APU_FAST, 80, gScenarioKernelData197_APU_FAST, 8},
{4, 10, 632, 96, 1, gScenarioBufferData198_APU_FAST, 80, gScenarioKernelData198_APU_FAST, 8},
{6, 10, 764, 96, 1, gScenarioBufferData199_APU_FAST, 80, gScenarioKernelData199_APU_FAST, 8},
{8, 10, 896, 96, 1, gScenarioBufferData200_APU_FAST, 80, gScenarioKernelData200_APU_FAST, 8},
{10, 10, 1028, 96, 1, gScenarioBufferData201_APU_FAST, 80, gScenarioKernelData201_APU_FAST, 8},
{12, 10, 1160, 96, 1, gScenarioBufferData202_APU_FAST, 80, gScenarioKernelData202_APU_FAST, 8},
{14, 10, 1292, 96, 1, gScenarioBufferData203_APU_FAST, 80, gScenarioKernelData203_APU_FAST, 8},
{16, 10, 1424, 96, 1, gScenarioBufferData204_APU_FAST, 80, gScenarioKernelData204_APU_FAST, 8},
{18, 10, 1556, 96, 1, gScenarioBufferData205_APU_FAST, 80, gScenarioKernelData205_APU_FAST, 8},
{20, 10, 1688, 96, 1, gScenarioBufferData206_APU_FAST, 80, gScenarioKernelData206_APU_FAST, 8},
{22, 10, 1820, 96, 1, gScenarioBufferData207_APU_FAST, 80, gScenarioKernelData207_APU_FAST, 8},
{24, 10, 1952, 96, 1, gScenarioBufferData208_APU_FAST, 80, gScenarioKernelData208_APU_FAST, 8},
{26, 10, 2084, 96, 1, gScenarioBufferData209_APU_FAST, 80, gScenarioKernelData209_APU_FAST, 8},
{28, 10, 2216, 96, 1, gScenarioBufferData210_APU_FAST, 80, gScenarioKernelData210_APU_FAST, 8},
{30, 10, 2348, 96, 1, gScenarioBufferData211_APU_FAST, 80, gScenarioKernelData211_APU_FAST, 8},
{32, 10, 2480, 96, 1, gScenarioBufferData212_APU_FAST, 80, gScenarioKernelData212_APU_FAST, 8},
{36, 10, 2744, 96, 1, gScenarioBufferData213_APU_FAST, 80, gScenarioKernelData213_APU_FAST, 8},
{40, 10, 3008, 96, 1, gScenarioBufferData214_APU_FAST, 80, gScenarioKernelData214_APU_FAST, 8},
{44, 10, 3272, 96, 1, gScenarioBufferData215_APU_FAST, 80, gScenarioKernelData215_APU_FAST, 8},
{48, 10, 3536, 96, 1, gScenarioBufferData216_APU_FAST, 80, gScenarioKernelData216_APU_FAST, 8},
{52, 10, 3800, 96, 1, gScenarioBufferData217_APU_FAST, 80, gScenarioKernelData217_APU_FAST, 8},
{4, 12, 744, 96, 1, gScenarioBufferData218_APU_FAST, 80, gScenarioKernelData218_APU_FAST, 8},
{6, 12, 900, 96, 1, gScenarioBufferData219_APU_FAST, 80, gScenarioKernelData219_APU_FAST, 8},
{8, 12, 1056, 96, 1, gScenarioBufferData220_APU_FAST, 80, gScenarioKernelData220_APU_FAST, 8},
{10, 12, 1212, 96, 1, gScenarioBufferData221_APU_FAST, 80, gScenarioKernelData221_APU_FAST, 8},
{12, 12, 1368, 96, 1, gScenarioBufferData222_APU_FAST, 80, gScenarioKernelData222_APU_FAST, 8},
{14, 12, 1524, 96, 1, gScenarioBufferData223_APU_FAST, 80, gScenarioKernelData223_APU_FAST, 8},
{16, 12, 1680, 96, 1, gScenarioBufferData224_APU_FAST, 80, gScenarioKernelData224_APU_FAST, 8},
{18, 12, 1836, 96, 1, gScenarioBufferData225_APU_FAST, 80, gScenarioKernelData225_APU_FAST, 8},
{20, 12, 1992, 96, 1, gScenarioBufferData226_APU_FAST, 80, gScenarioKernelData226_APU_FAST, 8},
{22, 12, 2148, 96, 1, gScenarioBufferData227_APU_FAST, 80, gScenarioKernelData227_APU_FAST, 8},
{24, 12, 2304, 96, 1, gScenarioBufferData228_APU_FAST, 80, gScenarioKernelData228_APU_FAST, 8},
{26, 12, 2460, 96, 1, gScenarioBufferData229_APU_FAST, 80, gScenarioKernelData229_APU_FAST, 8},
{28, 12, 2616, 96, 1, gScenarioBufferData230_APU_FAST, 80, gScenarioKernelData230_APU_FAST, 8},
{30, 12, 2772, 96, 1, gScenarioBufferData231_APU_FAST, 80, gScenarioKernelData231_APU_FAST, 8},
{32, 12, 2928, 96, 1, gScenarioBufferData232_APU_FAST, 80, gScenarioKernelData232_APU_FAST, 8},
{36, 12, 3240, 96, 1, gScenarioBufferData233_APU_FAST, 80, gScenarioKernelData233_APU_FAST, 8},
{40, 12, 3552, 96, 1, gScenarioBufferData234_APU_FAST, 80, gScenarioKernelData234_APU_FAST, 8},
{4, 14, 856, 96, 1, gScenarioBufferData235_APU_FAST, 80, gScenarioKernelData235_APU_FAST, 8},
{6, 14, 1036, 96, 1, gScenarioBufferData236_APU_FAST, 80, gScenarioKernelData236_APU_FAST, 8},
{8, 14, 1216, 96, 1, gScenarioBufferData237_APU_FAST, 80, gScenarioKernelData237_APU_FAST, 8},
{10, 14, 1396, 96, 1, gScenarioBufferData238_APU_FAST, 80, gScenarioKernelData238_APU_FAST, 8},
{12, 14, 1576, 96, 1, gScenarioBufferData239_APU_FAST, 80, gScenarioKernelData239_APU_FAST, 8},
{14, 14, 1756, 96, 1, gScenarioBufferData240_APU_FAST, 80, gScenarioKernelData240_APU_FAST, 8},
{16, 14, 1936, 96, 1, gScenarioBufferData241_APU_FAST, 80, gScenarioKernelData241_APU_FAST, 8},
{18, 14, 2116, 96, 1, gScenarioBufferData242_APU_FAST, 80, gScenarioKernelData242_APU_FAST, 8},
{20, 14, 2296, 96, 1, gScenarioBufferData243_APU_FAST, 80, gScenarioKernelData243_APU_FAST, 8},
{22, 14, 2476, 96, 1, gScenarioBufferData244_APU_FAST, 80, gScenarioKernelData244_APU_FAST, 8},
{24, 14, 2656, 96, 1, gScenarioBufferData245_APU_FAST, 80, gScenarioKernelData245_APU_FAST, 8},
{26, 14, 2836, 96, 1, gScenarioBufferData246_APU_FAST, 80, gScenarioKernelData246_APU_FAST, 8},
{28, 14, 3016, 96, 1, gScenarioBufferData247_APU_FAST, 80, gScenarioKernelData247_APU_FAST, 8},
{30, 14, 3196, 96, 1, gScenarioBufferData248_APU_FAST, 80, gScenarioKernelData248_APU_FAST, 8},
{32, 14, 3376, 96, 1, gScenarioBufferData249_APU_FAST, 80, gScenarioKernelData249_APU_FAST, 8},
{36, 14, 3736, 96, 1, gScenarioBufferData250_APU_FAST, 80, gScenarioKernelData250_APU_FAST, 8},
{4, 16, 968, 96, 1, gScenarioBufferData251_APU_FAST, 80, gScenarioKernelData251_APU_FAST, 8},
{6, 16, 1172, 96, 1, gScenarioBufferData252_APU_FAST, 80, gScenarioKernelData252_APU_FAST, 8},
{8, 16, 1376, 96, 1, gScenarioBufferData253_APU_FAST, 80, gScenarioKernelData253_APU_FAST, 8},
{10, 16, 1580, 96, 1, gScenarioBufferData254_APU_FAST, 80, gScenarioKernelData254_APU_FAST, 8},
{12, 16, 1784, 96, 1, gScenarioBufferData255_APU_FAST, 80, gScenarioKernelData255_APU_FAST, 8},
{14, 16, 1988, 96, 1, gScenarioBufferData256_APU_FAST, 80, gScenarioKernelData256_APU_FAST, 8},
{16, 16, 2192, 96, 1, gScenarioBufferData257_APU_FAST, 80, gScenarioKernelData257_APU_FAST, 8},
{18, 16, 2396, 96, 1, gScenarioBufferData258_APU_FAST, 80, gScenarioKernelData258_APU_FAST, 8},
{20, 16, 2600, 96, 1, gScenarioBufferData259_APU_FAST, 80, gScenarioKernelData259_APU_FAST, 8},
{22, 16, 2804, 96, 1, gScenarioBufferData260_APU_FAST, 80, gScenarioKernelData260_APU_FAST, 8},
{24, 16, 3008, 96, 1, gScenarioBufferData261_APU_FAST, 80, gScenarioKernelData261_APU_FAST, 8},
{26, 16, 3212, 96, 1, gScenarioBufferData262_APU_FAST, 80, gScenarioKernelData262_APU_FAST, 8},
{28, 16, 3416, 96, 1, gScenarioBufferData263_APU_FAST, 80, gScenarioKernelData263_APU_FAST, 8},
{30, 16, 3620, 96, 1, gScenarioBufferData264_APU_FAST, 80, gScenarioKernelData264_APU_FAST, 8},
{32, 16, 3824, 96, 1, gScenarioBufferData265_APU_FAST, 80, gScenarioKernelData265_APU_FAST, 8},
{4, 18, 1080, 96, 1, gScenarioBufferData266_APU_FAST, 80, gScenarioKernelData266_APU_FAST, 8},
{6, 18, 1308, 96, 1, gScenarioBufferData267_APU_FAST, 80, gScenarioKernelData267_APU_FAST, 8},
{8, 18, 1536, 96, 1, gScenarioBufferData268_APU_FAST, 80, gScenarioKernelData268_APU_FAST, 8},
{10, 18, 1764, 96, 1, gScenarioBufferData269_APU_FAST, 80, gScenarioKernelData269_APU_FAST, 8},
{12, 18, 1992, 96, 1, gScenarioBufferData270_APU_FAST, 80, gScenarioKernelData270_APU_FAST, 8},
{14, 18, 2220, 96, 1, gScenarioBufferData271_APU_FAST, 80, gScenarioKernelData271_APU_FAST, 8},
{16, 18, 2448, 96, 1, gScenarioBufferData272_APU_FAST, 80, gScenarioKernelData272_APU_FAST, 8},
{18, 18, 2676, 96, 1, gScenarioBufferData273_APU_FAST, 80, gScenarioKernelData273_APU_FAST, 8},
{20, 18, 2904, 96, 1, gScenarioBufferData274_APU_FAST, 80, gScenarioKernelData274_APU_FAST, 8},
{22, 18, 3132, 96, 1, gScenarioBufferData275_APU_FAST, 80, gScenarioKernelData275_APU_FAST, 8},
{24, 18, 3360, 96, 1, gScenarioBufferData276_APU_FAST, 80, gScenarioKernelData276_APU_FAST, 8},
{26, 18, 3588, 96, 1, gScenarioBufferData277_APU_FAST, 80, gScenarioKernelData277_APU_FAST, 8},
{28, 18, 3816, 96, 1, gScenarioBufferData278_APU_FAST, 80, gScenarioKernelData278_APU_FAST, 8},
{4, 20, 1192, 96, 1, gScenarioBufferData279_APU_FAST, 80, gScenarioKernelData279_APU_FAST, 8},
{6, 20, 1444, 96, 1, gScenarioBufferData280_APU_FAST, 80, gScenarioKernelData280_APU_FAST, 8},
{8, 20, 1696, 96, 1, gScenarioBufferData281_APU_FAST, 80, gScenarioKernelData281_APU_FAST, 8},
{10, 20, 1948, 96, 1, gScenarioBufferData282_APU_FAST, 80, gScenarioKernelData282_APU_FAST, 8},
{12, 20, 2200, 96, 1, gScenarioBufferData283_APU_FAST, 80, gScenarioKernelData283_APU_FAST, 8},
{14, 20, 2452, 96, 1, gScenarioBufferData284_APU_FAST, 80, gScenarioKernelData284_APU_FAST, 8},
{16, 20, 2704, 96, 1, gScenarioBufferData285_APU_FAST, 80, gScenarioKernelData285_APU_FAST, 8},
{18, 20, 2956, 96, 1, gScenarioBufferData286_APU_FAST, 80, gScenarioKernelData286_APU_FAST, 8},
{20, 20, 3208, 96, 1, gScenarioBufferData287_APU_FAST, 80, gScenarioKernelData287_APU_FAST, 8},
{22, 20, 3460, 96, 1, gScenarioBufferData288_APU_FAST, 80, gScenarioKernelData288_APU_FAST, 8},
{24, 20, 3712, 96, 1, gScenarioBufferData289_APU_FAST, 80, gScenarioKernelData289_APU_FAST, 8},
{4, 22, 1304, 96, 1, gScenarioBufferData290_APU_FAST, 80, gScenarioKernelData290_APU_FAST, 8},
{6, 22, 1580, 96, 1, gScenarioBufferData291_APU_FAST, 80, gScenarioKernelData291_APU_FAST, 8},
{8, 22, 1856, 96, 1, gScenarioBufferData292_APU_FAST, 80, gScenarioKernelData292_APU_FAST, 8},
{10, 22, 2132, 96, 1, gScenarioBufferData293_APU_FAST, 80, gScenarioKernelData293_APU_FAST, 8},
{12, 22, 2408, 96, 1, gScenarioBufferData294_APU_FAST, 80, gScenarioKernelData294_APU_FAST, 8},
{14, 22, 2684, 96, 1, gScenarioBufferData295_APU_FAST, 80, gScenarioKernelData295_APU_FAST, 8},
{16, 22, 2960, 96, 1, gScenarioBufferData296_APU_FAST, 80, gScenarioKernelData296_APU_FAST, 8},
{18, 22, 3236, 96, 1, gScenarioBufferData297_APU_FAST, 80, gScenarioKernelData297_APU_FAST, 8},
{20, 22, 3512, 96, 1, gScenarioBufferData298_APU_FAST, 80, gScenarioKernelData298_APU_FAST, 8},
{22, 22, 3788, 96, 1, gScenarioBufferData299_APU_FAST, 80, gScenarioKernelData299_APU_FAST, 8},
{4, 24, 1416, 96, 1, gScenarioBufferData300_APU_FAST, 80, gScenarioKernelData300_APU_FAST, 8},
{6, 24, 1716, 96, 1, gScenarioBufferData301_APU_FAST, 80, gScenarioKernelData301_APU_FAST, 8},
{8, 24, 2016, 96, 1, gScenarioBufferData302_APU_FAST, 80, gScenarioKernelData302_APU_FAST, 8},
{10, 24, 2316, 96, 1, gScenarioBufferData303_APU_FAST, 80, gScenarioKernelData303_APU_FAST, 8},
{12, 24, 2616, 96, 1, gScenarioBufferData304_APU_FAST, 80, gScenarioKernelData304_APU_FAST, 8},
{14, 24, 2916, 96, 1, gScenarioBufferData305_APU_FAST, 80, gScenarioKernelData305_APU_FAST, 8},
{16, 24, 3216, 96, 1, gScenarioBufferData306_APU_FAST, 80, gScenarioKernelData306_APU_FAST, 8},
{18, 24, 3516, 96, 1, gScenarioBufferData307_APU_FAST, 80, gScenarioKernelData307_APU_FAST, 8},
{20, 24, 3816, 96, 1, gScenarioBufferData308_APU_FAST, 80, gScenarioKernelData308_APU_FAST, 8},
{4, 26, 1528, 96, 1, gScenarioBufferData309_APU_FAST, 80, gScenarioKernelData309_APU_FAST, 8},
{6, 26, 1852, 96, 1, gScenarioBufferData310_APU_FAST, 80, gScenarioKernelData310_APU_FAST, 8},
{8, 26, 2176, 96, 1, gScenarioBufferData311_APU_FAST, 80, gScenarioKernelData311_APU_FAST, 8},
{10, 26, 2500, 96, 1, gScenarioBufferData312_APU_FAST, 80, gScenarioKernelData312_APU_FAST, 8},
{12, 26, 2824, 96, 1, gScenarioBufferData313_APU_FAST, 80, gScenarioKernelData313_APU_FAST, 8},
{14, 26, 3148, 96, 1, gScenarioBufferData314_APU_FAST, 80, gScenarioKernelData314_APU_FAST, 8},
{16, 26, 3472, 96, 1, gScenarioBufferData315_APU_FAST, 80, gScenarioKernelData315_APU_FAST, 8},
{18, 26, 3796, 96, 1, gScenarioBufferData316_APU_FAST, 80, gScenarioKernelData316_APU_FAST, 8},
{4, 28, 1640, 96, 1, gScenarioBufferData317_APU_FAST, 80, gScenarioKernelData317_APU_FAST, 8},
{6, 28, 1988, 96, 1, gScenarioBufferData318_APU_FAST, 80, gScenarioKernelData318_APU_FAST, 8},
{8, 28, 2336, 96, 1, gScenarioBufferData319_APU_FAST, 80, gScenarioKernelData319_APU_FAST, 8},
{10, 28, 2684, 96, 1, gScenarioBufferData320_APU_FAST, 80, gScenarioKernelData320_APU_FAST, 8},
{12, 28, 3032, 96, 1, gScenarioBufferData321_APU_FAST, 80, gScenarioKernelData321_APU_FAST, 8},
{14, 28, 3380, 96, 1, gScenarioBufferData322_APU_FAST, 80, gScenarioKernelData322_APU_FAST, 8},
{16, 28, 3728, 96, 1, gScenarioBufferData323_APU_FAST, 80, gScenarioKernelData323_APU_FAST, 8},
{4, 30, 1752, 96, 1, gScenarioBufferData324_APU_FAST, 80, gScenarioKernelData324_APU_FAST, 8},
{6, 30, 2124, 96, 1, gScenarioBufferData325_APU_FAST, 80, gScenarioKernelData325_APU_FAST, 8},
{8, 30, 2496, 96, 1, gScenarioBufferData326_APU_FAST, 80, gScenarioKernelData326_APU_FAST, 8},
{10, 30, 2868, 96, 1, gScenarioBufferData327_APU_FAST, 80, gScenarioKernelData327_APU_FAST, 8},
{12, 30, 3240, 96, 1, gScenarioBufferData328_APU_FAST, 80, gScenarioKernelData328_APU_FAST, 8},
{14, 30, 3612, 96, 1, gScenarioBufferData329_APU_FAST, 80, gScenarioKernelData329_APU_FAST, 8},
{4, 32, 1864, 96, 1, gScenarioBufferData330_APU_FAST, 80, gScenarioKernelData330_APU_FAST, 8},
{6, 32, 2260, 96, 1, gScenarioBufferData331_APU_FAST, 80, gScenarioKernelData331_APU_FAST, 8},
{8, 32, 2656, 96, 1, gScenarioBufferData332_APU_FAST, 80, gScenarioKernelData332_APU_FAST, 8},
{10, 32, 3052, 96, 1, gScenarioBufferData333_APU_FAST, 80, gScenarioKernelData333_APU_FAST, 8},
{12, 32, 3448, 96, 1, gScenarioBufferData334_APU_FAST, 80, gScenarioKernelData334_APU_FAST, 8}
};

static acf_scenario_list gScenarioList_APU_FAST = {
335, //number of scenarios
gScenarioArray_APU_FAST};
//**************************************************************

class APU_FAST : public ACF_Process_APU
{

public:
   APU_FAST(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("APU_FAST");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("APU_FAST",
                                        APU_FAST_LOAD_SEGMENTS,
                                        APU_FAST_LOAD_PMEM, APU_FAST_LOAD_PMEM_SIZE,
                                        APU_FAST_LOAD_DMEM, APU_FAST_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(APU_FAST_APEX_LOG_BUFFER, APU_FAST_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0);
         AddPort("THRESHOLD", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("CIRCUMFERENCE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_APU_FAST);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_APU_FAST

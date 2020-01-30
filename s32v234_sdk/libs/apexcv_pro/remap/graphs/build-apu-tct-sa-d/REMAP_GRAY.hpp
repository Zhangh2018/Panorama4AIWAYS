#ifndef _ACF_PROCESS_APU_REMAP_GRAY
#define _ACF_PROCESS_APU_REMAP_GRAY

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <REMAP_GRAY_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_REMAP_GRAY[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_REMAP_GRAY[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_REMAP_GRAY[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_REMAP_GRAY[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_REMAP_GRAY[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_REMAP_GRAY[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_REMAP_GRAY[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_REMAP_GRAY[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_REMAP_GRAY[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_REMAP_GRAY[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_REMAP_GRAY[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_REMAP_GRAY[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_REMAP_GRAY[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_REMAP_GRAY[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_REMAP_GRAY[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_REMAP_GRAY[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_REMAP_GRAY[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_REMAP_GRAY[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_REMAP_GRAY[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_REMAP_GRAY[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_REMAP_GRAY[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_REMAP_GRAY[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_REMAP_GRAY[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_REMAP_GRAY[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_REMAP_GRAY[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_REMAP_GRAY[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_REMAP_GRAY[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_REMAP_GRAY[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_REMAP_GRAY[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_REMAP_GRAY[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_REMAP_GRAY[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_REMAP_GRAY[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_REMAP_GRAY[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_REMAP_GRAY[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_REMAP_GRAY[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_REMAP_GRAY[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_REMAP_GRAY[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_REMAP_GRAY[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_REMAP_GRAY[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_REMAP_GRAY[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_REMAP_GRAY[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_REMAP_GRAY[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_REMAP_GRAY[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_REMAP_GRAY[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_REMAP_GRAY[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_REMAP_GRAY[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_REMAP_GRAY[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_REMAP_GRAY[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_REMAP_GRAY[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_REMAP_GRAY[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_REMAP_GRAY[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_REMAP_GRAY[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_REMAP_GRAY[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_REMAP_GRAY[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_REMAP_GRAY[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_REMAP_GRAY[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_REMAP_GRAY[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_REMAP_GRAY[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_REMAP_GRAY[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_REMAP_GRAY[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_REMAP_GRAY[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_REMAP_GRAY[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_REMAP_GRAY[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_REMAP_GRAY[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_REMAP_GRAY[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_REMAP_GRAY[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_REMAP_GRAY[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_REMAP_GRAY[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_REMAP_GRAY[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_REMAP_GRAY[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_REMAP_GRAY[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_REMAP_GRAY[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_REMAP_GRAY[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_REMAP_GRAY[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_REMAP_GRAY[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_REMAP_GRAY[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_REMAP_GRAY[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_REMAP_GRAY[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_REMAP_GRAY[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_REMAP_GRAY[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_REMAP_GRAY[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_REMAP_GRAY[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_REMAP_GRAY[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_REMAP_GRAY[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_REMAP_GRAY[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_REMAP_GRAY[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_REMAP_GRAY[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_REMAP_GRAY[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_REMAP_GRAY[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_REMAP_GRAY[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_REMAP_GRAY[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_REMAP_GRAY[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_REMAP_GRAY[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_REMAP_GRAY[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_REMAP_GRAY[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_REMAP_GRAY[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_REMAP_GRAY[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_REMAP_GRAY[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_REMAP_GRAY[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_REMAP_GRAY[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_REMAP_GRAY[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_REMAP_GRAY[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_REMAP_GRAY[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_REMAP_GRAY[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_REMAP_GRAY[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_REMAP_GRAY[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_REMAP_GRAY[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_REMAP_GRAY[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_REMAP_GRAY[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_REMAP_GRAY[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_REMAP_GRAY[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_REMAP_GRAY[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_REMAP_GRAY[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_REMAP_GRAY[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_REMAP_GRAY[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_REMAP_GRAY[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_REMAP_GRAY[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_REMAP_GRAY[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_REMAP_GRAY[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_REMAP_GRAY[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_REMAP_GRAY[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_REMAP_GRAY[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_REMAP_GRAY[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_REMAP_GRAY[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_REMAP_GRAY[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_REMAP_GRAY[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_REMAP_GRAY[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_REMAP_GRAY[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_REMAP_GRAY[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_REMAP_GRAY[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_REMAP_GRAY[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_REMAP_GRAY[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_REMAP_GRAY[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_REMAP_GRAY[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_REMAP_GRAY[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_REMAP_GRAY[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_REMAP_GRAY[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_REMAP_GRAY[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_REMAP_GRAY[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_REMAP_GRAY[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_REMAP_GRAY[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_REMAP_GRAY[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_REMAP_GRAY[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_REMAP_GRAY[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_REMAP_GRAY[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_REMAP_GRAY[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_REMAP_GRAY[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_REMAP_GRAY[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_REMAP_GRAY[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_REMAP_GRAY[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_REMAP_GRAY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_REMAP_GRAY[] = {{0, 0}};

static acf_scenario gScenarioArray_REMAP_GRAY[] = {
{4, 1, 48, 0, 0, gScenarioBufferData0_REMAP_GRAY, 64, gScenarioKernelData0_REMAP_GRAY, 4},
{8, 1, 96, 0, 0, gScenarioBufferData1_REMAP_GRAY, 64, gScenarioKernelData1_REMAP_GRAY, 4},
{12, 1, 144, 0, 0, gScenarioBufferData2_REMAP_GRAY, 64, gScenarioKernelData2_REMAP_GRAY, 4},
{16, 1, 192, 0, 0, gScenarioBufferData3_REMAP_GRAY, 64, gScenarioKernelData3_REMAP_GRAY, 4},
{20, 1, 240, 0, 0, gScenarioBufferData4_REMAP_GRAY, 64, gScenarioKernelData4_REMAP_GRAY, 4},
{24, 1, 288, 0, 0, gScenarioBufferData5_REMAP_GRAY, 64, gScenarioKernelData5_REMAP_GRAY, 4},
{28, 1, 336, 0, 0, gScenarioBufferData6_REMAP_GRAY, 64, gScenarioKernelData6_REMAP_GRAY, 4},
{32, 1, 384, 0, 0, gScenarioBufferData7_REMAP_GRAY, 64, gScenarioKernelData7_REMAP_GRAY, 4},
{36, 1, 432, 0, 0, gScenarioBufferData8_REMAP_GRAY, 64, gScenarioKernelData8_REMAP_GRAY, 4},
{40, 1, 480, 0, 0, gScenarioBufferData9_REMAP_GRAY, 64, gScenarioKernelData9_REMAP_GRAY, 4},
{44, 1, 528, 0, 0, gScenarioBufferData10_REMAP_GRAY, 64, gScenarioKernelData10_REMAP_GRAY, 4},
{48, 1, 576, 0, 0, gScenarioBufferData11_REMAP_GRAY, 64, gScenarioKernelData11_REMAP_GRAY, 4},
{52, 1, 624, 0, 0, gScenarioBufferData12_REMAP_GRAY, 64, gScenarioKernelData12_REMAP_GRAY, 4},
{56, 1, 672, 0, 0, gScenarioBufferData13_REMAP_GRAY, 64, gScenarioKernelData13_REMAP_GRAY, 4},
{60, 1, 720, 0, 0, gScenarioBufferData14_REMAP_GRAY, 64, gScenarioKernelData14_REMAP_GRAY, 4},
{64, 1, 768, 0, 0, gScenarioBufferData15_REMAP_GRAY, 64, gScenarioKernelData15_REMAP_GRAY, 4},
{4, 2, 96, 0, 0, gScenarioBufferData16_REMAP_GRAY, 64, gScenarioKernelData16_REMAP_GRAY, 4},
{8, 2, 192, 0, 0, gScenarioBufferData17_REMAP_GRAY, 64, gScenarioKernelData17_REMAP_GRAY, 4},
{12, 2, 288, 0, 0, gScenarioBufferData18_REMAP_GRAY, 64, gScenarioKernelData18_REMAP_GRAY, 4},
{16, 2, 384, 0, 0, gScenarioBufferData19_REMAP_GRAY, 64, gScenarioKernelData19_REMAP_GRAY, 4},
{20, 2, 480, 0, 0, gScenarioBufferData20_REMAP_GRAY, 64, gScenarioKernelData20_REMAP_GRAY, 4},
{24, 2, 576, 0, 0, gScenarioBufferData21_REMAP_GRAY, 64, gScenarioKernelData21_REMAP_GRAY, 4},
{28, 2, 672, 0, 0, gScenarioBufferData22_REMAP_GRAY, 64, gScenarioKernelData22_REMAP_GRAY, 4},
{32, 2, 768, 0, 0, gScenarioBufferData23_REMAP_GRAY, 64, gScenarioKernelData23_REMAP_GRAY, 4},
{36, 2, 864, 0, 0, gScenarioBufferData24_REMAP_GRAY, 64, gScenarioKernelData24_REMAP_GRAY, 4},
{40, 2, 960, 0, 0, gScenarioBufferData25_REMAP_GRAY, 64, gScenarioKernelData25_REMAP_GRAY, 4},
{44, 2, 1056, 0, 0, gScenarioBufferData26_REMAP_GRAY, 64, gScenarioKernelData26_REMAP_GRAY, 4},
{48, 2, 1152, 0, 0, gScenarioBufferData27_REMAP_GRAY, 64, gScenarioKernelData27_REMAP_GRAY, 4},
{52, 2, 1248, 0, 0, gScenarioBufferData28_REMAP_GRAY, 64, gScenarioKernelData28_REMAP_GRAY, 4},
{56, 2, 1344, 0, 0, gScenarioBufferData29_REMAP_GRAY, 64, gScenarioKernelData29_REMAP_GRAY, 4},
{60, 2, 1440, 0, 0, gScenarioBufferData30_REMAP_GRAY, 64, gScenarioKernelData30_REMAP_GRAY, 4},
{64, 2, 1536, 0, 0, gScenarioBufferData31_REMAP_GRAY, 64, gScenarioKernelData31_REMAP_GRAY, 4},
{4, 3, 144, 0, 0, gScenarioBufferData32_REMAP_GRAY, 64, gScenarioKernelData32_REMAP_GRAY, 4},
{8, 3, 288, 0, 0, gScenarioBufferData33_REMAP_GRAY, 64, gScenarioKernelData33_REMAP_GRAY, 4},
{12, 3, 432, 0, 0, gScenarioBufferData34_REMAP_GRAY, 64, gScenarioKernelData34_REMAP_GRAY, 4},
{16, 3, 576, 0, 0, gScenarioBufferData35_REMAP_GRAY, 64, gScenarioKernelData35_REMAP_GRAY, 4},
{20, 3, 720, 0, 0, gScenarioBufferData36_REMAP_GRAY, 64, gScenarioKernelData36_REMAP_GRAY, 4},
{24, 3, 864, 0, 0, gScenarioBufferData37_REMAP_GRAY, 64, gScenarioKernelData37_REMAP_GRAY, 4},
{28, 3, 1008, 0, 0, gScenarioBufferData38_REMAP_GRAY, 64, gScenarioKernelData38_REMAP_GRAY, 4},
{32, 3, 1152, 0, 0, gScenarioBufferData39_REMAP_GRAY, 64, gScenarioKernelData39_REMAP_GRAY, 4},
{36, 3, 1296, 0, 0, gScenarioBufferData40_REMAP_GRAY, 64, gScenarioKernelData40_REMAP_GRAY, 4},
{40, 3, 1440, 0, 0, gScenarioBufferData41_REMAP_GRAY, 64, gScenarioKernelData41_REMAP_GRAY, 4},
{44, 3, 1584, 0, 0, gScenarioBufferData42_REMAP_GRAY, 64, gScenarioKernelData42_REMAP_GRAY, 4},
{48, 3, 1728, 0, 0, gScenarioBufferData43_REMAP_GRAY, 64, gScenarioKernelData43_REMAP_GRAY, 4},
{52, 3, 1872, 0, 0, gScenarioBufferData44_REMAP_GRAY, 64, gScenarioKernelData44_REMAP_GRAY, 4},
{56, 3, 2016, 0, 0, gScenarioBufferData45_REMAP_GRAY, 64, gScenarioKernelData45_REMAP_GRAY, 4},
{60, 3, 2160, 0, 0, gScenarioBufferData46_REMAP_GRAY, 64, gScenarioKernelData46_REMAP_GRAY, 4},
{64, 3, 2304, 0, 0, gScenarioBufferData47_REMAP_GRAY, 64, gScenarioKernelData47_REMAP_GRAY, 4},
{4, 4, 192, 0, 0, gScenarioBufferData48_REMAP_GRAY, 64, gScenarioKernelData48_REMAP_GRAY, 4},
{8, 4, 384, 0, 0, gScenarioBufferData49_REMAP_GRAY, 64, gScenarioKernelData49_REMAP_GRAY, 4},
{12, 4, 576, 0, 0, gScenarioBufferData50_REMAP_GRAY, 64, gScenarioKernelData50_REMAP_GRAY, 4},
{16, 4, 768, 0, 0, gScenarioBufferData51_REMAP_GRAY, 64, gScenarioKernelData51_REMAP_GRAY, 4},
{20, 4, 960, 0, 0, gScenarioBufferData52_REMAP_GRAY, 64, gScenarioKernelData52_REMAP_GRAY, 4},
{24, 4, 1152, 0, 0, gScenarioBufferData53_REMAP_GRAY, 64, gScenarioKernelData53_REMAP_GRAY, 4},
{28, 4, 1344, 0, 0, gScenarioBufferData54_REMAP_GRAY, 64, gScenarioKernelData54_REMAP_GRAY, 4},
{32, 4, 1536, 0, 0, gScenarioBufferData55_REMAP_GRAY, 64, gScenarioKernelData55_REMAP_GRAY, 4},
{36, 4, 1728, 0, 0, gScenarioBufferData56_REMAP_GRAY, 64, gScenarioKernelData56_REMAP_GRAY, 4},
{40, 4, 1920, 0, 0, gScenarioBufferData57_REMAP_GRAY, 64, gScenarioKernelData57_REMAP_GRAY, 4},
{44, 4, 2112, 0, 0, gScenarioBufferData58_REMAP_GRAY, 64, gScenarioKernelData58_REMAP_GRAY, 4},
{48, 4, 2304, 0, 0, gScenarioBufferData59_REMAP_GRAY, 64, gScenarioKernelData59_REMAP_GRAY, 4},
{52, 4, 2496, 0, 0, gScenarioBufferData60_REMAP_GRAY, 64, gScenarioKernelData60_REMAP_GRAY, 4},
{56, 4, 2688, 0, 0, gScenarioBufferData61_REMAP_GRAY, 64, gScenarioKernelData61_REMAP_GRAY, 4},
{60, 4, 2880, 0, 0, gScenarioBufferData62_REMAP_GRAY, 64, gScenarioKernelData62_REMAP_GRAY, 4},
{64, 4, 3072, 0, 0, gScenarioBufferData63_REMAP_GRAY, 64, gScenarioKernelData63_REMAP_GRAY, 4},
{4, 5, 240, 0, 0, gScenarioBufferData64_REMAP_GRAY, 64, gScenarioKernelData64_REMAP_GRAY, 4},
{8, 5, 480, 0, 0, gScenarioBufferData65_REMAP_GRAY, 64, gScenarioKernelData65_REMAP_GRAY, 4},
{12, 5, 720, 0, 0, gScenarioBufferData66_REMAP_GRAY, 64, gScenarioKernelData66_REMAP_GRAY, 4},
{16, 5, 960, 0, 0, gScenarioBufferData67_REMAP_GRAY, 64, gScenarioKernelData67_REMAP_GRAY, 4},
{20, 5, 1200, 0, 0, gScenarioBufferData68_REMAP_GRAY, 64, gScenarioKernelData68_REMAP_GRAY, 4},
{24, 5, 1440, 0, 0, gScenarioBufferData69_REMAP_GRAY, 64, gScenarioKernelData69_REMAP_GRAY, 4},
{28, 5, 1680, 0, 0, gScenarioBufferData70_REMAP_GRAY, 64, gScenarioKernelData70_REMAP_GRAY, 4},
{32, 5, 1920, 0, 0, gScenarioBufferData71_REMAP_GRAY, 64, gScenarioKernelData71_REMAP_GRAY, 4},
{36, 5, 2160, 0, 0, gScenarioBufferData72_REMAP_GRAY, 64, gScenarioKernelData72_REMAP_GRAY, 4},
{40, 5, 2400, 0, 0, gScenarioBufferData73_REMAP_GRAY, 64, gScenarioKernelData73_REMAP_GRAY, 4},
{44, 5, 2640, 0, 0, gScenarioBufferData74_REMAP_GRAY, 64, gScenarioKernelData74_REMAP_GRAY, 4},
{48, 5, 2880, 0, 0, gScenarioBufferData75_REMAP_GRAY, 64, gScenarioKernelData75_REMAP_GRAY, 4},
{52, 5, 3120, 0, 0, gScenarioBufferData76_REMAP_GRAY, 64, gScenarioKernelData76_REMAP_GRAY, 4},
{56, 5, 3360, 0, 0, gScenarioBufferData77_REMAP_GRAY, 64, gScenarioKernelData77_REMAP_GRAY, 4},
{60, 5, 3600, 0, 0, gScenarioBufferData78_REMAP_GRAY, 64, gScenarioKernelData78_REMAP_GRAY, 4},
{64, 5, 3840, 0, 0, gScenarioBufferData79_REMAP_GRAY, 64, gScenarioKernelData79_REMAP_GRAY, 4},
{4, 6, 288, 0, 0, gScenarioBufferData80_REMAP_GRAY, 64, gScenarioKernelData80_REMAP_GRAY, 4},
{8, 6, 576, 0, 0, gScenarioBufferData81_REMAP_GRAY, 64, gScenarioKernelData81_REMAP_GRAY, 4},
{12, 6, 864, 0, 0, gScenarioBufferData82_REMAP_GRAY, 64, gScenarioKernelData82_REMAP_GRAY, 4},
{16, 6, 1152, 0, 0, gScenarioBufferData83_REMAP_GRAY, 64, gScenarioKernelData83_REMAP_GRAY, 4},
{20, 6, 1440, 0, 0, gScenarioBufferData84_REMAP_GRAY, 64, gScenarioKernelData84_REMAP_GRAY, 4},
{24, 6, 1728, 0, 0, gScenarioBufferData85_REMAP_GRAY, 64, gScenarioKernelData85_REMAP_GRAY, 4},
{28, 6, 2016, 0, 0, gScenarioBufferData86_REMAP_GRAY, 64, gScenarioKernelData86_REMAP_GRAY, 4},
{32, 6, 2304, 0, 0, gScenarioBufferData87_REMAP_GRAY, 64, gScenarioKernelData87_REMAP_GRAY, 4},
{36, 6, 2592, 0, 0, gScenarioBufferData88_REMAP_GRAY, 64, gScenarioKernelData88_REMAP_GRAY, 4},
{40, 6, 2880, 0, 0, gScenarioBufferData89_REMAP_GRAY, 64, gScenarioKernelData89_REMAP_GRAY, 4},
{44, 6, 3168, 0, 0, gScenarioBufferData90_REMAP_GRAY, 64, gScenarioKernelData90_REMAP_GRAY, 4},
{48, 6, 3456, 0, 0, gScenarioBufferData91_REMAP_GRAY, 64, gScenarioKernelData91_REMAP_GRAY, 4},
{52, 6, 3744, 0, 0, gScenarioBufferData92_REMAP_GRAY, 64, gScenarioKernelData92_REMAP_GRAY, 4},
{4, 8, 384, 0, 0, gScenarioBufferData93_REMAP_GRAY, 64, gScenarioKernelData93_REMAP_GRAY, 4},
{8, 8, 768, 0, 0, gScenarioBufferData94_REMAP_GRAY, 64, gScenarioKernelData94_REMAP_GRAY, 4},
{12, 8, 1152, 0, 0, gScenarioBufferData95_REMAP_GRAY, 64, gScenarioKernelData95_REMAP_GRAY, 4},
{16, 8, 1536, 0, 0, gScenarioBufferData96_REMAP_GRAY, 64, gScenarioKernelData96_REMAP_GRAY, 4},
{20, 8, 1920, 0, 0, gScenarioBufferData97_REMAP_GRAY, 64, gScenarioKernelData97_REMAP_GRAY, 4},
{24, 8, 2304, 0, 0, gScenarioBufferData98_REMAP_GRAY, 64, gScenarioKernelData98_REMAP_GRAY, 4},
{28, 8, 2688, 0, 0, gScenarioBufferData99_REMAP_GRAY, 64, gScenarioKernelData99_REMAP_GRAY, 4},
{32, 8, 3072, 0, 0, gScenarioBufferData100_REMAP_GRAY, 64, gScenarioKernelData100_REMAP_GRAY, 4},
{36, 8, 3456, 0, 0, gScenarioBufferData101_REMAP_GRAY, 64, gScenarioKernelData101_REMAP_GRAY, 4},
{40, 8, 3840, 0, 0, gScenarioBufferData102_REMAP_GRAY, 64, gScenarioKernelData102_REMAP_GRAY, 4},
{4, 10, 480, 0, 0, gScenarioBufferData103_REMAP_GRAY, 64, gScenarioKernelData103_REMAP_GRAY, 4},
{8, 10, 960, 0, 0, gScenarioBufferData104_REMAP_GRAY, 64, gScenarioKernelData104_REMAP_GRAY, 4},
{12, 10, 1440, 0, 0, gScenarioBufferData105_REMAP_GRAY, 64, gScenarioKernelData105_REMAP_GRAY, 4},
{16, 10, 1920, 0, 0, gScenarioBufferData106_REMAP_GRAY, 64, gScenarioKernelData106_REMAP_GRAY, 4},
{20, 10, 2400, 0, 0, gScenarioBufferData107_REMAP_GRAY, 64, gScenarioKernelData107_REMAP_GRAY, 4},
{24, 10, 2880, 0, 0, gScenarioBufferData108_REMAP_GRAY, 64, gScenarioKernelData108_REMAP_GRAY, 4},
{28, 10, 3360, 0, 0, gScenarioBufferData109_REMAP_GRAY, 64, gScenarioKernelData109_REMAP_GRAY, 4},
{32, 10, 3840, 0, 0, gScenarioBufferData110_REMAP_GRAY, 64, gScenarioKernelData110_REMAP_GRAY, 4},
{4, 12, 576, 0, 0, gScenarioBufferData111_REMAP_GRAY, 64, gScenarioKernelData111_REMAP_GRAY, 4},
{8, 12, 1152, 0, 0, gScenarioBufferData112_REMAP_GRAY, 64, gScenarioKernelData112_REMAP_GRAY, 4},
{12, 12, 1728, 0, 0, gScenarioBufferData113_REMAP_GRAY, 64, gScenarioKernelData113_REMAP_GRAY, 4},
{16, 12, 2304, 0, 0, gScenarioBufferData114_REMAP_GRAY, 64, gScenarioKernelData114_REMAP_GRAY, 4},
{20, 12, 2880, 0, 0, gScenarioBufferData115_REMAP_GRAY, 64, gScenarioKernelData115_REMAP_GRAY, 4},
{24, 12, 3456, 0, 0, gScenarioBufferData116_REMAP_GRAY, 64, gScenarioKernelData116_REMAP_GRAY, 4},
{4, 14, 672, 0, 0, gScenarioBufferData117_REMAP_GRAY, 64, gScenarioKernelData117_REMAP_GRAY, 4},
{8, 14, 1344, 0, 0, gScenarioBufferData118_REMAP_GRAY, 64, gScenarioKernelData118_REMAP_GRAY, 4},
{12, 14, 2016, 0, 0, gScenarioBufferData119_REMAP_GRAY, 64, gScenarioKernelData119_REMAP_GRAY, 4},
{16, 14, 2688, 0, 0, gScenarioBufferData120_REMAP_GRAY, 64, gScenarioKernelData120_REMAP_GRAY, 4},
{20, 14, 3360, 0, 0, gScenarioBufferData121_REMAP_GRAY, 64, gScenarioKernelData121_REMAP_GRAY, 4},
{4, 16, 768, 0, 0, gScenarioBufferData122_REMAP_GRAY, 64, gScenarioKernelData122_REMAP_GRAY, 4},
{8, 16, 1536, 0, 0, gScenarioBufferData123_REMAP_GRAY, 64, gScenarioKernelData123_REMAP_GRAY, 4},
{12, 16, 2304, 0, 0, gScenarioBufferData124_REMAP_GRAY, 64, gScenarioKernelData124_REMAP_GRAY, 4},
{16, 16, 3072, 0, 0, gScenarioBufferData125_REMAP_GRAY, 64, gScenarioKernelData125_REMAP_GRAY, 4},
{20, 16, 3840, 0, 0, gScenarioBufferData126_REMAP_GRAY, 64, gScenarioKernelData126_REMAP_GRAY, 4},
{4, 18, 864, 0, 0, gScenarioBufferData127_REMAP_GRAY, 64, gScenarioKernelData127_REMAP_GRAY, 4},
{8, 18, 1728, 0, 0, gScenarioBufferData128_REMAP_GRAY, 64, gScenarioKernelData128_REMAP_GRAY, 4},
{12, 18, 2592, 0, 0, gScenarioBufferData129_REMAP_GRAY, 64, gScenarioKernelData129_REMAP_GRAY, 4},
{16, 18, 3456, 0, 0, gScenarioBufferData130_REMAP_GRAY, 64, gScenarioKernelData130_REMAP_GRAY, 4},
{4, 20, 960, 0, 0, gScenarioBufferData131_REMAP_GRAY, 64, gScenarioKernelData131_REMAP_GRAY, 4},
{8, 20, 1920, 0, 0, gScenarioBufferData132_REMAP_GRAY, 64, gScenarioKernelData132_REMAP_GRAY, 4},
{12, 20, 2880, 0, 0, gScenarioBufferData133_REMAP_GRAY, 64, gScenarioKernelData133_REMAP_GRAY, 4},
{16, 20, 3840, 0, 0, gScenarioBufferData134_REMAP_GRAY, 64, gScenarioKernelData134_REMAP_GRAY, 4},
{4, 22, 1056, 0, 0, gScenarioBufferData135_REMAP_GRAY, 64, gScenarioKernelData135_REMAP_GRAY, 4},
{8, 22, 2112, 0, 0, gScenarioBufferData136_REMAP_GRAY, 64, gScenarioKernelData136_REMAP_GRAY, 4},
{12, 22, 3168, 0, 0, gScenarioBufferData137_REMAP_GRAY, 64, gScenarioKernelData137_REMAP_GRAY, 4},
{4, 24, 1152, 0, 0, gScenarioBufferData138_REMAP_GRAY, 64, gScenarioKernelData138_REMAP_GRAY, 4},
{8, 24, 2304, 0, 0, gScenarioBufferData139_REMAP_GRAY, 64, gScenarioKernelData139_REMAP_GRAY, 4},
{12, 24, 3456, 0, 0, gScenarioBufferData140_REMAP_GRAY, 64, gScenarioKernelData140_REMAP_GRAY, 4},
{4, 26, 1248, 0, 0, gScenarioBufferData141_REMAP_GRAY, 64, gScenarioKernelData141_REMAP_GRAY, 4},
{8, 26, 2496, 0, 0, gScenarioBufferData142_REMAP_GRAY, 64, gScenarioKernelData142_REMAP_GRAY, 4},
{12, 26, 3744, 0, 0, gScenarioBufferData143_REMAP_GRAY, 64, gScenarioKernelData143_REMAP_GRAY, 4},
{4, 28, 1344, 0, 0, gScenarioBufferData144_REMAP_GRAY, 64, gScenarioKernelData144_REMAP_GRAY, 4},
{8, 28, 2688, 0, 0, gScenarioBufferData145_REMAP_GRAY, 64, gScenarioKernelData145_REMAP_GRAY, 4},
{4, 30, 1440, 0, 0, gScenarioBufferData146_REMAP_GRAY, 64, gScenarioKernelData146_REMAP_GRAY, 4},
{8, 30, 2880, 0, 0, gScenarioBufferData147_REMAP_GRAY, 64, gScenarioKernelData147_REMAP_GRAY, 4},
{4, 32, 1536, 0, 0, gScenarioBufferData148_REMAP_GRAY, 64, gScenarioKernelData148_REMAP_GRAY, 4},
{8, 32, 3072, 0, 0, gScenarioBufferData149_REMAP_GRAY, 64, gScenarioKernelData149_REMAP_GRAY, 4}
};

static acf_scenario_list gScenarioList_REMAP_GRAY = {
150, //number of scenarios
gScenarioArray_REMAP_GRAY};
//**************************************************************

class REMAP_GRAY : public ACF_Process_APU
{

public:
   REMAP_GRAY(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("REMAP_GRAY");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("REMAP_GRAY",
                                        REMAP_GRAY_LOAD_SEGMENTS,
                                        REMAP_GRAY_LOAD_PMEM, REMAP_GRAY_LOAD_PMEM_SIZE,
                                        REMAP_GRAY_LOAD_DMEM, REMAP_GRAY_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(REMAP_GRAY_APEX_LOG_BUFFER, REMAP_GRAY_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("SRC_IMG", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("LOCAL_OFFSET", icp::DATATYPE_16U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("DELTA", icp::DATATYPE_08U, 2, 1, 4, 1, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("DST_IMG", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_REMAP_GRAY);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_REMAP_GRAY

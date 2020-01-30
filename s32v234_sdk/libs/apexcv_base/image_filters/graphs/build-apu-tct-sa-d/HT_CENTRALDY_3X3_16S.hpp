#ifndef _ACF_PROCESS_APU_HT_CENTRALDY_3X3_16S
#define _ACF_PROCESS_APU_HT_CENTRALDY_3X3_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HT_CENTRALDY_3X3_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HT_CENTRALDY_3X3_16S[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HT_CENTRALDY_3X3_16S[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HT_CENTRALDY_3X3_16S[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HT_CENTRALDY_3X3_16S[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HT_CENTRALDY_3X3_16S[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HT_CENTRALDY_3X3_16S[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HT_CENTRALDY_3X3_16S[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HT_CENTRALDY_3X3_16S[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HT_CENTRALDY_3X3_16S[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HT_CENTRALDY_3X3_16S[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HT_CENTRALDY_3X3_16S[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HT_CENTRALDY_3X3_16S[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HT_CENTRALDY_3X3_16S[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HT_CENTRALDY_3X3_16S[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HT_CENTRALDY_3X3_16S[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HT_CENTRALDY_3X3_16S[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HT_CENTRALDY_3X3_16S[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HT_CENTRALDY_3X3_16S[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HT_CENTRALDY_3X3_16S[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HT_CENTRALDY_3X3_16S[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HT_CENTRALDY_3X3_16S[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HT_CENTRALDY_3X3_16S[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HT_CENTRALDY_3X3_16S[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HT_CENTRALDY_3X3_16S[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HT_CENTRALDY_3X3_16S[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HT_CENTRALDY_3X3_16S[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HT_CENTRALDY_3X3_16S[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HT_CENTRALDY_3X3_16S[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HT_CENTRALDY_3X3_16S[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HT_CENTRALDY_3X3_16S[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HT_CENTRALDY_3X3_16S[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HT_CENTRALDY_3X3_16S[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HT_CENTRALDY_3X3_16S[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HT_CENTRALDY_3X3_16S[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HT_CENTRALDY_3X3_16S[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HT_CENTRALDY_3X3_16S[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HT_CENTRALDY_3X3_16S[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_HT_CENTRALDY_3X3_16S[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_HT_CENTRALDY_3X3_16S[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_HT_CENTRALDY_3X3_16S[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_HT_CENTRALDY_3X3_16S[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_HT_CENTRALDY_3X3_16S[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_HT_CENTRALDY_3X3_16S[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_HT_CENTRALDY_3X3_16S[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_HT_CENTRALDY_3X3_16S[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_HT_CENTRALDY_3X3_16S[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_HT_CENTRALDY_3X3_16S[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_HT_CENTRALDY_3X3_16S[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_HT_CENTRALDY_3X3_16S[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_HT_CENTRALDY_3X3_16S[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_HT_CENTRALDY_3X3_16S[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_HT_CENTRALDY_3X3_16S[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_HT_CENTRALDY_3X3_16S[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_HT_CENTRALDY_3X3_16S[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_HT_CENTRALDY_3X3_16S[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_HT_CENTRALDY_3X3_16S[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_HT_CENTRALDY_3X3_16S[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_HT_CENTRALDY_3X3_16S[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_HT_CENTRALDY_3X3_16S[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_HT_CENTRALDY_3X3_16S[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_HT_CENTRALDY_3X3_16S[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_HT_CENTRALDY_3X3_16S[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_HT_CENTRALDY_3X3_16S[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_HT_CENTRALDY_3X3_16S[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_HT_CENTRALDY_3X3_16S[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_HT_CENTRALDY_3X3_16S[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_HT_CENTRALDY_3X3_16S[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_HT_CENTRALDY_3X3_16S[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_HT_CENTRALDY_3X3_16S[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_HT_CENTRALDY_3X3_16S[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_HT_CENTRALDY_3X3_16S[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_HT_CENTRALDY_3X3_16S[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_HT_CENTRALDY_3X3_16S[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_HT_CENTRALDY_3X3_16S[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_HT_CENTRALDY_3X3_16S[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_HT_CENTRALDY_3X3_16S[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_HT_CENTRALDY_3X3_16S[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_HT_CENTRALDY_3X3_16S[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_HT_CENTRALDY_3X3_16S[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_HT_CENTRALDY_3X3_16S[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_HT_CENTRALDY_3X3_16S[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_HT_CENTRALDY_3X3_16S[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_HT_CENTRALDY_3X3_16S[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_HT_CENTRALDY_3X3_16S[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_HT_CENTRALDY_3X3_16S[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_HT_CENTRALDY_3X3_16S[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_HT_CENTRALDY_3X3_16S[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_HT_CENTRALDY_3X3_16S[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_HT_CENTRALDY_3X3_16S[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_HT_CENTRALDY_3X3_16S[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_HT_CENTRALDY_3X3_16S[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_HT_CENTRALDY_3X3_16S[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_HT_CENTRALDY_3X3_16S[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_HT_CENTRALDY_3X3_16S[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_HT_CENTRALDY_3X3_16S[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_HT_CENTRALDY_3X3_16S[] = {{64, 6, 4, 0, {1, 1, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_HT_CENTRALDY_3X3_16S[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_HT_CENTRALDY_3X3_16S[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_HT_CENTRALDY_3X3_16S[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_HT_CENTRALDY_3X3_16S[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_HT_CENTRALDY_3X3_16S[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_HT_CENTRALDY_3X3_16S[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_HT_CENTRALDY_3X3_16S[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_HT_CENTRALDY_3X3_16S[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_HT_CENTRALDY_3X3_16S[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_HT_CENTRALDY_3X3_16S[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_HT_CENTRALDY_3X3_16S[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_HT_CENTRALDY_3X3_16S[] = {{48, 8, 4, 0, {1, 1, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_HT_CENTRALDY_3X3_16S[] = {{52, 8, 4, 0, {1, 1, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_HT_CENTRALDY_3X3_16S[] = {{56, 8, 4, 0, {1, 1, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_HT_CENTRALDY_3X3_16S[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_HT_CENTRALDY_3X3_16S[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_HT_CENTRALDY_3X3_16S[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_HT_CENTRALDY_3X3_16S[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_HT_CENTRALDY_3X3_16S[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_HT_CENTRALDY_3X3_16S[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_HT_CENTRALDY_3X3_16S[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_HT_CENTRALDY_3X3_16S[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_HT_CENTRALDY_3X3_16S[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_HT_CENTRALDY_3X3_16S[] = {{40, 10, 4, 0, {1, 1, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_HT_CENTRALDY_3X3_16S[] = {{44, 10, 4, 0, {1, 1, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_HT_CENTRALDY_3X3_16S[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_HT_CENTRALDY_3X3_16S[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_HT_CENTRALDY_3X3_16S[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_HT_CENTRALDY_3X3_16S[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_HT_CENTRALDY_3X3_16S[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_HT_CENTRALDY_3X3_16S[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_HT_CENTRALDY_3X3_16S[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_HT_CENTRALDY_3X3_16S[] = {{32, 12, 4, 0, {1, 1, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_HT_CENTRALDY_3X3_16S[] = {{36, 12, 4, 0, {1, 1, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_HT_CENTRALDY_3X3_16S[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_HT_CENTRALDY_3X3_16S[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_HT_CENTRALDY_3X3_16S[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_HT_CENTRALDY_3X3_16S[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_HT_CENTRALDY_3X3_16S[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_HT_CENTRALDY_3X3_16S[] = {{24, 14, 4, 0, {1, 1, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_HT_CENTRALDY_3X3_16S[] = {{28, 14, 4, 0, {1, 1, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_HT_CENTRALDY_3X3_16S[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_HT_CENTRALDY_3X3_16S[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_HT_CENTRALDY_3X3_16S[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_HT_CENTRALDY_3X3_16S[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_HT_CENTRALDY_3X3_16S[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_HT_CENTRALDY_3X3_16S[] = {{24, 16, 4, 0, {1, 1, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_HT_CENTRALDY_3X3_16S[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_HT_CENTRALDY_3X3_16S[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_HT_CENTRALDY_3X3_16S[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_HT_CENTRALDY_3X3_16S[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_HT_CENTRALDY_3X3_16S[] = {{20, 18, 4, 0, {1, 1, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_HT_CENTRALDY_3X3_16S[] = {{24, 18, 4, 0, {1, 1, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_HT_CENTRALDY_3X3_16S[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_HT_CENTRALDY_3X3_16S[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_HT_CENTRALDY_3X3_16S[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_HT_CENTRALDY_3X3_16S[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_HT_CENTRALDY_3X3_16S[] = {{20, 20, 4, 0, {1, 1, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_HT_CENTRALDY_3X3_16S[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_HT_CENTRALDY_3X3_16S[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_HT_CENTRALDY_3X3_16S[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_HT_CENTRALDY_3X3_16S[] = {{16, 22, 4, 0, {1, 1, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_HT_CENTRALDY_3X3_16S[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_HT_CENTRALDY_3X3_16S[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_HT_CENTRALDY_3X3_16S[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_HT_CENTRALDY_3X3_16S[] = {{16, 24, 4, 0, {1, 1, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_HT_CENTRALDY_3X3_16S[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_HT_CENTRALDY_3X3_16S[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_HT_CENTRALDY_3X3_16S[] = {{12, 26, 4, 0, {1, 1, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_HT_CENTRALDY_3X3_16S[] = {{16, 26, 4, 0, {1, 1, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_HT_CENTRALDY_3X3_16S[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_HT_CENTRALDY_3X3_16S[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_HT_CENTRALDY_3X3_16S[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_HT_CENTRALDY_3X3_16S[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_HT_CENTRALDY_3X3_16S[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_HT_CENTRALDY_3X3_16S[] = {{12, 30, 4, 0, {1, 1, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_HT_CENTRALDY_3X3_16S[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_HT_CENTRALDY_3X3_16S[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_HT_CENTRALDY_3X3_16S[] = {{12, 32, 4, 0, {1, 1, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_HT_CENTRALDY_3X3_16S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_HT_CENTRALDY_3X3_16S[] = {{1, 0}};

static acf_scenario gScenarioArray_HT_CENTRALDY_3X3_16S[] = {
{1, 1, 64, 48, 1, gScenarioBufferData0_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData0_HT_CENTRALDY_3X3_16S, 4},
{2, 1, 104, 48, 1, gScenarioBufferData1_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData1_HT_CENTRALDY_3X3_16S, 4},
{3, 1, 144, 48, 1, gScenarioBufferData2_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData2_HT_CENTRALDY_3X3_16S, 4},
{4, 1, 184, 48, 1, gScenarioBufferData3_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData3_HT_CENTRALDY_3X3_16S, 4},
{5, 1, 224, 48, 1, gScenarioBufferData4_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData4_HT_CENTRALDY_3X3_16S, 4},
{6, 1, 264, 48, 1, gScenarioBufferData5_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData5_HT_CENTRALDY_3X3_16S, 4},
{7, 1, 304, 48, 1, gScenarioBufferData6_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData6_HT_CENTRALDY_3X3_16S, 4},
{8, 1, 344, 48, 1, gScenarioBufferData7_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData7_HT_CENTRALDY_3X3_16S, 4},
{9, 1, 384, 48, 1, gScenarioBufferData8_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData8_HT_CENTRALDY_3X3_16S, 4},
{10, 1, 424, 48, 1, gScenarioBufferData9_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData9_HT_CENTRALDY_3X3_16S, 4},
{11, 1, 464, 48, 1, gScenarioBufferData10_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData10_HT_CENTRALDY_3X3_16S, 4},
{12, 1, 504, 48, 1, gScenarioBufferData11_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData11_HT_CENTRALDY_3X3_16S, 4},
{13, 1, 544, 48, 1, gScenarioBufferData12_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData12_HT_CENTRALDY_3X3_16S, 4},
{14, 1, 584, 48, 1, gScenarioBufferData13_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData13_HT_CENTRALDY_3X3_16S, 4},
{15, 1, 624, 48, 1, gScenarioBufferData14_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData14_HT_CENTRALDY_3X3_16S, 4},
{16, 1, 664, 48, 1, gScenarioBufferData15_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData15_HT_CENTRALDY_3X3_16S, 4},
{1, 2, 112, 48, 1, gScenarioBufferData16_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData16_HT_CENTRALDY_3X3_16S, 4},
{2, 2, 184, 48, 1, gScenarioBufferData17_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData17_HT_CENTRALDY_3X3_16S, 4},
{3, 2, 256, 48, 1, gScenarioBufferData18_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData18_HT_CENTRALDY_3X3_16S, 4},
{4, 2, 328, 48, 1, gScenarioBufferData19_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData19_HT_CENTRALDY_3X3_16S, 4},
{5, 2, 400, 48, 1, gScenarioBufferData20_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData20_HT_CENTRALDY_3X3_16S, 4},
{6, 2, 472, 48, 1, gScenarioBufferData21_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData21_HT_CENTRALDY_3X3_16S, 4},
{7, 2, 544, 48, 1, gScenarioBufferData22_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData22_HT_CENTRALDY_3X3_16S, 4},
{8, 2, 616, 48, 1, gScenarioBufferData23_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData23_HT_CENTRALDY_3X3_16S, 4},
{9, 2, 688, 48, 1, gScenarioBufferData24_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData24_HT_CENTRALDY_3X3_16S, 4},
{10, 2, 760, 48, 1, gScenarioBufferData25_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData25_HT_CENTRALDY_3X3_16S, 4},
{11, 2, 832, 48, 1, gScenarioBufferData26_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData26_HT_CENTRALDY_3X3_16S, 4},
{12, 2, 904, 48, 1, gScenarioBufferData27_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData27_HT_CENTRALDY_3X3_16S, 4},
{13, 2, 976, 48, 1, gScenarioBufferData28_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData28_HT_CENTRALDY_3X3_16S, 4},
{14, 2, 1048, 48, 1, gScenarioBufferData29_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData29_HT_CENTRALDY_3X3_16S, 4},
{15, 2, 1120, 48, 1, gScenarioBufferData30_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData30_HT_CENTRALDY_3X3_16S, 4},
{16, 2, 1192, 48, 1, gScenarioBufferData31_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData31_HT_CENTRALDY_3X3_16S, 4},
{1, 3, 160, 48, 1, gScenarioBufferData32_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData32_HT_CENTRALDY_3X3_16S, 4},
{2, 3, 264, 48, 1, gScenarioBufferData33_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData33_HT_CENTRALDY_3X3_16S, 4},
{3, 3, 368, 48, 1, gScenarioBufferData34_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData34_HT_CENTRALDY_3X3_16S, 4},
{4, 3, 472, 48, 1, gScenarioBufferData35_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData35_HT_CENTRALDY_3X3_16S, 4},
{5, 3, 576, 48, 1, gScenarioBufferData36_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData36_HT_CENTRALDY_3X3_16S, 4},
{6, 3, 680, 48, 1, gScenarioBufferData37_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData37_HT_CENTRALDY_3X3_16S, 4},
{7, 3, 784, 48, 1, gScenarioBufferData38_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData38_HT_CENTRALDY_3X3_16S, 4},
{8, 3, 888, 48, 1, gScenarioBufferData39_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData39_HT_CENTRALDY_3X3_16S, 4},
{9, 3, 992, 48, 1, gScenarioBufferData40_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData40_HT_CENTRALDY_3X3_16S, 4},
{10, 3, 1096, 48, 1, gScenarioBufferData41_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData41_HT_CENTRALDY_3X3_16S, 4},
{11, 3, 1200, 48, 1, gScenarioBufferData42_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData42_HT_CENTRALDY_3X3_16S, 4},
{12, 3, 1304, 48, 1, gScenarioBufferData43_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData43_HT_CENTRALDY_3X3_16S, 4},
{13, 3, 1408, 48, 1, gScenarioBufferData44_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData44_HT_CENTRALDY_3X3_16S, 4},
{14, 3, 1512, 48, 1, gScenarioBufferData45_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData45_HT_CENTRALDY_3X3_16S, 4},
{15, 3, 1616, 48, 1, gScenarioBufferData46_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData46_HT_CENTRALDY_3X3_16S, 4},
{16, 3, 1720, 48, 1, gScenarioBufferData47_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData47_HT_CENTRALDY_3X3_16S, 4},
{1, 4, 208, 48, 1, gScenarioBufferData48_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData48_HT_CENTRALDY_3X3_16S, 4},
{2, 4, 344, 48, 1, gScenarioBufferData49_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData49_HT_CENTRALDY_3X3_16S, 4},
{3, 4, 480, 48, 1, gScenarioBufferData50_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData50_HT_CENTRALDY_3X3_16S, 4},
{4, 4, 616, 48, 1, gScenarioBufferData51_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData51_HT_CENTRALDY_3X3_16S, 4},
{5, 4, 752, 48, 1, gScenarioBufferData52_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData52_HT_CENTRALDY_3X3_16S, 4},
{6, 4, 888, 48, 1, gScenarioBufferData53_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData53_HT_CENTRALDY_3X3_16S, 4},
{7, 4, 1024, 48, 1, gScenarioBufferData54_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData54_HT_CENTRALDY_3X3_16S, 4},
{8, 4, 1160, 48, 1, gScenarioBufferData55_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData55_HT_CENTRALDY_3X3_16S, 4},
{9, 4, 1296, 48, 1, gScenarioBufferData56_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData56_HT_CENTRALDY_3X3_16S, 4},
{10, 4, 1432, 48, 1, gScenarioBufferData57_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData57_HT_CENTRALDY_3X3_16S, 4},
{11, 4, 1568, 48, 1, gScenarioBufferData58_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData58_HT_CENTRALDY_3X3_16S, 4},
{12, 4, 1704, 48, 1, gScenarioBufferData59_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData59_HT_CENTRALDY_3X3_16S, 4},
{13, 4, 1840, 48, 1, gScenarioBufferData60_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData60_HT_CENTRALDY_3X3_16S, 4},
{14, 4, 1976, 48, 1, gScenarioBufferData61_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData61_HT_CENTRALDY_3X3_16S, 4},
{15, 4, 2112, 48, 1, gScenarioBufferData62_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData62_HT_CENTRALDY_3X3_16S, 4},
{16, 4, 2248, 48, 1, gScenarioBufferData63_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData63_HT_CENTRALDY_3X3_16S, 4},
{1, 5, 256, 48, 1, gScenarioBufferData64_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData64_HT_CENTRALDY_3X3_16S, 4},
{2, 5, 424, 48, 1, gScenarioBufferData65_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData65_HT_CENTRALDY_3X3_16S, 4},
{3, 5, 592, 48, 1, gScenarioBufferData66_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData66_HT_CENTRALDY_3X3_16S, 4},
{4, 5, 760, 48, 1, gScenarioBufferData67_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData67_HT_CENTRALDY_3X3_16S, 4},
{5, 5, 928, 48, 1, gScenarioBufferData68_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData68_HT_CENTRALDY_3X3_16S, 4},
{6, 5, 1096, 48, 1, gScenarioBufferData69_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData69_HT_CENTRALDY_3X3_16S, 4},
{7, 5, 1264, 48, 1, gScenarioBufferData70_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData70_HT_CENTRALDY_3X3_16S, 4},
{8, 5, 1432, 48, 1, gScenarioBufferData71_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData71_HT_CENTRALDY_3X3_16S, 4},
{9, 5, 1600, 48, 1, gScenarioBufferData72_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData72_HT_CENTRALDY_3X3_16S, 4},
{10, 5, 1768, 48, 1, gScenarioBufferData73_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData73_HT_CENTRALDY_3X3_16S, 4},
{11, 5, 1936, 48, 1, gScenarioBufferData74_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData74_HT_CENTRALDY_3X3_16S, 4},
{12, 5, 2104, 48, 1, gScenarioBufferData75_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData75_HT_CENTRALDY_3X3_16S, 4},
{13, 5, 2272, 48, 1, gScenarioBufferData76_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData76_HT_CENTRALDY_3X3_16S, 4},
{14, 5, 2440, 48, 1, gScenarioBufferData77_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData77_HT_CENTRALDY_3X3_16S, 4},
{15, 5, 2608, 48, 1, gScenarioBufferData78_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData78_HT_CENTRALDY_3X3_16S, 4},
{16, 5, 2776, 48, 1, gScenarioBufferData79_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData79_HT_CENTRALDY_3X3_16S, 4},
{1, 6, 304, 48, 1, gScenarioBufferData80_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData80_HT_CENTRALDY_3X3_16S, 4},
{2, 6, 504, 48, 1, gScenarioBufferData81_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData81_HT_CENTRALDY_3X3_16S, 4},
{3, 6, 704, 48, 1, gScenarioBufferData82_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData82_HT_CENTRALDY_3X3_16S, 4},
{4, 6, 904, 48, 1, gScenarioBufferData83_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData83_HT_CENTRALDY_3X3_16S, 4},
{5, 6, 1104, 48, 1, gScenarioBufferData84_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData84_HT_CENTRALDY_3X3_16S, 4},
{6, 6, 1304, 48, 1, gScenarioBufferData85_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData85_HT_CENTRALDY_3X3_16S, 4},
{7, 6, 1504, 48, 1, gScenarioBufferData86_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData86_HT_CENTRALDY_3X3_16S, 4},
{8, 6, 1704, 48, 1, gScenarioBufferData87_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData87_HT_CENTRALDY_3X3_16S, 4},
{9, 6, 1904, 48, 1, gScenarioBufferData88_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData88_HT_CENTRALDY_3X3_16S, 4},
{10, 6, 2104, 48, 1, gScenarioBufferData89_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData89_HT_CENTRALDY_3X3_16S, 4},
{11, 6, 2304, 48, 1, gScenarioBufferData90_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData90_HT_CENTRALDY_3X3_16S, 4},
{12, 6, 2504, 48, 1, gScenarioBufferData91_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData91_HT_CENTRALDY_3X3_16S, 4},
{13, 6, 2704, 48, 1, gScenarioBufferData92_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData92_HT_CENTRALDY_3X3_16S, 4},
{14, 6, 2904, 48, 1, gScenarioBufferData93_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData93_HT_CENTRALDY_3X3_16S, 4},
{15, 6, 3104, 48, 1, gScenarioBufferData94_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData94_HT_CENTRALDY_3X3_16S, 4},
{16, 6, 3304, 48, 1, gScenarioBufferData95_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData95_HT_CENTRALDY_3X3_16S, 4},
{1, 8, 400, 48, 1, gScenarioBufferData96_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData96_HT_CENTRALDY_3X3_16S, 4},
{2, 8, 664, 48, 1, gScenarioBufferData97_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData97_HT_CENTRALDY_3X3_16S, 4},
{3, 8, 928, 48, 1, gScenarioBufferData98_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData98_HT_CENTRALDY_3X3_16S, 4},
{4, 8, 1192, 48, 1, gScenarioBufferData99_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData99_HT_CENTRALDY_3X3_16S, 4},
{5, 8, 1456, 48, 1, gScenarioBufferData100_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData100_HT_CENTRALDY_3X3_16S, 4},
{6, 8, 1720, 48, 1, gScenarioBufferData101_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData101_HT_CENTRALDY_3X3_16S, 4},
{7, 8, 1984, 48, 1, gScenarioBufferData102_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData102_HT_CENTRALDY_3X3_16S, 4},
{8, 8, 2248, 48, 1, gScenarioBufferData103_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData103_HT_CENTRALDY_3X3_16S, 4},
{9, 8, 2512, 48, 1, gScenarioBufferData104_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData104_HT_CENTRALDY_3X3_16S, 4},
{10, 8, 2776, 48, 1, gScenarioBufferData105_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData105_HT_CENTRALDY_3X3_16S, 4},
{11, 8, 3040, 48, 1, gScenarioBufferData106_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData106_HT_CENTRALDY_3X3_16S, 4},
{12, 8, 3304, 48, 1, gScenarioBufferData107_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData107_HT_CENTRALDY_3X3_16S, 4},
{13, 8, 3568, 48, 1, gScenarioBufferData108_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData108_HT_CENTRALDY_3X3_16S, 4},
{14, 8, 3832, 48, 1, gScenarioBufferData109_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData109_HT_CENTRALDY_3X3_16S, 4},
{1, 10, 496, 48, 1, gScenarioBufferData110_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData110_HT_CENTRALDY_3X3_16S, 4},
{2, 10, 824, 48, 1, gScenarioBufferData111_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData111_HT_CENTRALDY_3X3_16S, 4},
{3, 10, 1152, 48, 1, gScenarioBufferData112_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData112_HT_CENTRALDY_3X3_16S, 4},
{4, 10, 1480, 48, 1, gScenarioBufferData113_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData113_HT_CENTRALDY_3X3_16S, 4},
{5, 10, 1808, 48, 1, gScenarioBufferData114_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData114_HT_CENTRALDY_3X3_16S, 4},
{6, 10, 2136, 48, 1, gScenarioBufferData115_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData115_HT_CENTRALDY_3X3_16S, 4},
{7, 10, 2464, 48, 1, gScenarioBufferData116_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData116_HT_CENTRALDY_3X3_16S, 4},
{8, 10, 2792, 48, 1, gScenarioBufferData117_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData117_HT_CENTRALDY_3X3_16S, 4},
{9, 10, 3120, 48, 1, gScenarioBufferData118_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData118_HT_CENTRALDY_3X3_16S, 4},
{10, 10, 3448, 48, 1, gScenarioBufferData119_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData119_HT_CENTRALDY_3X3_16S, 4},
{11, 10, 3776, 48, 1, gScenarioBufferData120_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData120_HT_CENTRALDY_3X3_16S, 4},
{1, 12, 592, 48, 1, gScenarioBufferData121_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData121_HT_CENTRALDY_3X3_16S, 4},
{2, 12, 984, 48, 1, gScenarioBufferData122_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData122_HT_CENTRALDY_3X3_16S, 4},
{3, 12, 1376, 48, 1, gScenarioBufferData123_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData123_HT_CENTRALDY_3X3_16S, 4},
{4, 12, 1768, 48, 1, gScenarioBufferData124_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData124_HT_CENTRALDY_3X3_16S, 4},
{5, 12, 2160, 48, 1, gScenarioBufferData125_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData125_HT_CENTRALDY_3X3_16S, 4},
{6, 12, 2552, 48, 1, gScenarioBufferData126_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData126_HT_CENTRALDY_3X3_16S, 4},
{7, 12, 2944, 48, 1, gScenarioBufferData127_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData127_HT_CENTRALDY_3X3_16S, 4},
{8, 12, 3336, 48, 1, gScenarioBufferData128_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData128_HT_CENTRALDY_3X3_16S, 4},
{9, 12, 3728, 48, 1, gScenarioBufferData129_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData129_HT_CENTRALDY_3X3_16S, 4},
{1, 14, 688, 48, 1, gScenarioBufferData130_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData130_HT_CENTRALDY_3X3_16S, 4},
{2, 14, 1144, 48, 1, gScenarioBufferData131_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData131_HT_CENTRALDY_3X3_16S, 4},
{3, 14, 1600, 48, 1, gScenarioBufferData132_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData132_HT_CENTRALDY_3X3_16S, 4},
{4, 14, 2056, 48, 1, gScenarioBufferData133_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData133_HT_CENTRALDY_3X3_16S, 4},
{5, 14, 2512, 48, 1, gScenarioBufferData134_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData134_HT_CENTRALDY_3X3_16S, 4},
{6, 14, 2968, 48, 1, gScenarioBufferData135_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData135_HT_CENTRALDY_3X3_16S, 4},
{7, 14, 3424, 48, 1, gScenarioBufferData136_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData136_HT_CENTRALDY_3X3_16S, 4},
{1, 16, 784, 48, 1, gScenarioBufferData137_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData137_HT_CENTRALDY_3X3_16S, 4},
{2, 16, 1304, 48, 1, gScenarioBufferData138_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData138_HT_CENTRALDY_3X3_16S, 4},
{3, 16, 1824, 48, 1, gScenarioBufferData139_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData139_HT_CENTRALDY_3X3_16S, 4},
{4, 16, 2344, 48, 1, gScenarioBufferData140_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData140_HT_CENTRALDY_3X3_16S, 4},
{5, 16, 2864, 48, 1, gScenarioBufferData141_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData141_HT_CENTRALDY_3X3_16S, 4},
{6, 16, 3384, 48, 1, gScenarioBufferData142_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData142_HT_CENTRALDY_3X3_16S, 4},
{1, 18, 880, 48, 1, gScenarioBufferData143_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData143_HT_CENTRALDY_3X3_16S, 4},
{2, 18, 1464, 48, 1, gScenarioBufferData144_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData144_HT_CENTRALDY_3X3_16S, 4},
{3, 18, 2048, 48, 1, gScenarioBufferData145_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData145_HT_CENTRALDY_3X3_16S, 4},
{4, 18, 2632, 48, 1, gScenarioBufferData146_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData146_HT_CENTRALDY_3X3_16S, 4},
{5, 18, 3216, 48, 1, gScenarioBufferData147_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData147_HT_CENTRALDY_3X3_16S, 4},
{6, 18, 3800, 48, 1, gScenarioBufferData148_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData148_HT_CENTRALDY_3X3_16S, 4},
{1, 20, 976, 48, 1, gScenarioBufferData149_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData149_HT_CENTRALDY_3X3_16S, 4},
{2, 20, 1624, 48, 1, gScenarioBufferData150_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData150_HT_CENTRALDY_3X3_16S, 4},
{3, 20, 2272, 48, 1, gScenarioBufferData151_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData151_HT_CENTRALDY_3X3_16S, 4},
{4, 20, 2920, 48, 1, gScenarioBufferData152_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData152_HT_CENTRALDY_3X3_16S, 4},
{5, 20, 3568, 48, 1, gScenarioBufferData153_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData153_HT_CENTRALDY_3X3_16S, 4},
{1, 22, 1072, 48, 1, gScenarioBufferData154_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData154_HT_CENTRALDY_3X3_16S, 4},
{2, 22, 1784, 48, 1, gScenarioBufferData155_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData155_HT_CENTRALDY_3X3_16S, 4},
{3, 22, 2496, 48, 1, gScenarioBufferData156_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData156_HT_CENTRALDY_3X3_16S, 4},
{4, 22, 3208, 48, 1, gScenarioBufferData157_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData157_HT_CENTRALDY_3X3_16S, 4},
{1, 24, 1168, 48, 1, gScenarioBufferData158_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData158_HT_CENTRALDY_3X3_16S, 4},
{2, 24, 1944, 48, 1, gScenarioBufferData159_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData159_HT_CENTRALDY_3X3_16S, 4},
{3, 24, 2720, 48, 1, gScenarioBufferData160_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData160_HT_CENTRALDY_3X3_16S, 4},
{4, 24, 3496, 48, 1, gScenarioBufferData161_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData161_HT_CENTRALDY_3X3_16S, 4},
{1, 26, 1264, 48, 1, gScenarioBufferData162_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData162_HT_CENTRALDY_3X3_16S, 4},
{2, 26, 2104, 48, 1, gScenarioBufferData163_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData163_HT_CENTRALDY_3X3_16S, 4},
{3, 26, 2944, 48, 1, gScenarioBufferData164_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData164_HT_CENTRALDY_3X3_16S, 4},
{4, 26, 3784, 48, 1, gScenarioBufferData165_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData165_HT_CENTRALDY_3X3_16S, 4},
{1, 28, 1360, 48, 1, gScenarioBufferData166_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData166_HT_CENTRALDY_3X3_16S, 4},
{2, 28, 2264, 48, 1, gScenarioBufferData167_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData167_HT_CENTRALDY_3X3_16S, 4},
{3, 28, 3168, 48, 1, gScenarioBufferData168_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData168_HT_CENTRALDY_3X3_16S, 4},
{1, 30, 1456, 48, 1, gScenarioBufferData169_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData169_HT_CENTRALDY_3X3_16S, 4},
{2, 30, 2424, 48, 1, gScenarioBufferData170_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData170_HT_CENTRALDY_3X3_16S, 4},
{3, 30, 3392, 48, 1, gScenarioBufferData171_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData171_HT_CENTRALDY_3X3_16S, 4},
{1, 32, 1552, 48, 1, gScenarioBufferData172_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData172_HT_CENTRALDY_3X3_16S, 4},
{2, 32, 2584, 48, 1, gScenarioBufferData173_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData173_HT_CENTRALDY_3X3_16S, 4},
{3, 32, 3616, 48, 1, gScenarioBufferData174_HT_CENTRALDY_3X3_16S, 80, gScenarioKernelData174_HT_CENTRALDY_3X3_16S, 4}
};

static acf_scenario_list gScenarioList_HT_CENTRALDY_3X3_16S = {
175, //number of scenarios
gScenarioArray_HT_CENTRALDY_3X3_16S};
//**************************************************************

class HT_CENTRALDY_3X3_16S : public ACF_Process_APU
{

public:
   HT_CENTRALDY_3X3_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HT_CENTRALDY_3X3_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HT_CENTRALDY_3X3_16S",
                                        HT_CENTRALDY_3X3_16S_LOAD_SEGMENTS,
                                        HT_CENTRALDY_3X3_16S_LOAD_PMEM, HT_CENTRALDY_3X3_16S_LOAD_PMEM_SIZE,
                                        HT_CENTRALDY_3X3_16S_LOAD_DMEM, HT_CENTRALDY_3X3_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HT_CENTRALDY_3X3_16S_APEX_LOG_BUFFER, HT_CENTRALDY_3X3_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0);
         AddPort("K_0", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("K_1", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("K_2", icp::DATATYPE_08S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HT_CENTRALDY_3X3_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HT_CENTRALDY_3X3_16S

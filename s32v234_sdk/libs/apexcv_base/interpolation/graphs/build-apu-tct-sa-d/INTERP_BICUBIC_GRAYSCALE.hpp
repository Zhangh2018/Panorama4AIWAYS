#ifndef _ACF_PROCESS_APU_INTERP_BICUBIC_GRAYSCALE
#define _ACF_PROCESS_APU_INTERP_BICUBIC_GRAYSCALE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INTERP_BICUBIC_GRAYSCALE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INTERP_BICUBIC_GRAYSCALE[] = {{2, 1, 7, 0, {2, 2, 2, 2}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INTERP_BICUBIC_GRAYSCALE[] = {{4, 1, 7, 0, {2, 2, 2, 2}}, {4, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INTERP_BICUBIC_GRAYSCALE[] = {{6, 1, 7, 0, {2, 2, 2, 2}}, {6, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INTERP_BICUBIC_GRAYSCALE[] = {{8, 1, 7, 0, {2, 2, 2, 2}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INTERP_BICUBIC_GRAYSCALE[] = {{10, 1, 7, 0, {2, 2, 2, 2}}, {10, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INTERP_BICUBIC_GRAYSCALE[] = {{12, 1, 7, 0, {2, 2, 2, 2}}, {12, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INTERP_BICUBIC_GRAYSCALE[] = {{14, 1, 7, 0, {2, 2, 2, 2}}, {14, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INTERP_BICUBIC_GRAYSCALE[] = {{16, 1, 7, 0, {2, 2, 2, 2}}, {16, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INTERP_BICUBIC_GRAYSCALE[] = {{18, 1, 7, 0, {2, 2, 2, 2}}, {18, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INTERP_BICUBIC_GRAYSCALE[] = {{20, 1, 7, 0, {2, 2, 2, 2}}, {20, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INTERP_BICUBIC_GRAYSCALE[] = {{22, 1, 7, 0, {2, 2, 2, 2}}, {22, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INTERP_BICUBIC_GRAYSCALE[] = {{24, 1, 7, 0, {2, 2, 2, 2}}, {24, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INTERP_BICUBIC_GRAYSCALE[] = {{26, 1, 7, 0, {2, 2, 2, 2}}, {26, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INTERP_BICUBIC_GRAYSCALE[] = {{28, 1, 7, 0, {2, 2, 2, 2}}, {28, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INTERP_BICUBIC_GRAYSCALE[] = {{30, 1, 7, 0, {2, 2, 2, 2}}, {30, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INTERP_BICUBIC_GRAYSCALE[] = {{32, 1, 7, 0, {2, 2, 2, 2}}, {32, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INTERP_BICUBIC_GRAYSCALE[] = {{36, 1, 7, 0, {2, 2, 2, 2}}, {36, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INTERP_BICUBIC_GRAYSCALE[] = {{40, 1, 7, 0, {2, 2, 2, 2}}, {40, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INTERP_BICUBIC_GRAYSCALE[] = {{44, 1, 7, 0, {2, 2, 2, 2}}, {44, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INTERP_BICUBIC_GRAYSCALE[] = {{48, 1, 7, 0, {2, 2, 2, 2}}, {48, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INTERP_BICUBIC_GRAYSCALE[] = {{52, 1, 7, 0, {2, 2, 2, 2}}, {52, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INTERP_BICUBIC_GRAYSCALE[] = {{56, 1, 7, 0, {2, 2, 2, 2}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INTERP_BICUBIC_GRAYSCALE[] = {{60, 1, 7, 0, {2, 2, 2, 2}}, {60, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INTERP_BICUBIC_GRAYSCALE[] = {{64, 1, 7, 0, {2, 2, 2, 2}}, {64, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INTERP_BICUBIC_GRAYSCALE[] = {{72, 1, 7, 0, {2, 2, 2, 2}}, {72, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INTERP_BICUBIC_GRAYSCALE[] = {{80, 1, 7, 0, {2, 2, 2, 2}}, {80, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INTERP_BICUBIC_GRAYSCALE[] = {{88, 1, 7, 0, {2, 2, 2, 2}}, {88, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INTERP_BICUBIC_GRAYSCALE[] = {{96, 1, 7, 0, {2, 2, 2, 2}}, {96, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INTERP_BICUBIC_GRAYSCALE[] = {{104, 1, 7, 0, {2, 2, 2, 2}}, {104, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INTERP_BICUBIC_GRAYSCALE[] = {{112, 1, 7, 0, {2, 2, 2, 2}}, {112, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INTERP_BICUBIC_GRAYSCALE[] = {{120, 1, 7, 0, {2, 2, 2, 2}}, {120, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INTERP_BICUBIC_GRAYSCALE[] = {{128, 1, 7, 0, {2, 2, 2, 2}}, {128, 1, 2, 2, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INTERP_BICUBIC_GRAYSCALE[] = {{2, 2, 4, 0, {2, 2, 2, 2}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INTERP_BICUBIC_GRAYSCALE[] = {{4, 2, 4, 0, {2, 2, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INTERP_BICUBIC_GRAYSCALE[] = {{6, 2, 4, 0, {2, 2, 2, 2}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INTERP_BICUBIC_GRAYSCALE[] = {{8, 2, 4, 0, {2, 2, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INTERP_BICUBIC_GRAYSCALE[] = {{10, 2, 4, 0, {2, 2, 2, 2}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INTERP_BICUBIC_GRAYSCALE[] = {{12, 2, 4, 0, {2, 2, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INTERP_BICUBIC_GRAYSCALE[] = {{14, 2, 4, 0, {2, 2, 2, 2}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INTERP_BICUBIC_GRAYSCALE[] = {{16, 2, 4, 0, {2, 2, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INTERP_BICUBIC_GRAYSCALE[] = {{18, 2, 4, 0, {2, 2, 2, 2}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INTERP_BICUBIC_GRAYSCALE[] = {{20, 2, 4, 0, {2, 2, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INTERP_BICUBIC_GRAYSCALE[] = {{22, 2, 4, 0, {2, 2, 2, 2}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INTERP_BICUBIC_GRAYSCALE[] = {{24, 2, 4, 0, {2, 2, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INTERP_BICUBIC_GRAYSCALE[] = {{26, 2, 4, 0, {2, 2, 2, 2}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_INTERP_BICUBIC_GRAYSCALE[] = {{28, 2, 4, 0, {2, 2, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_INTERP_BICUBIC_GRAYSCALE[] = {{30, 2, 4, 0, {2, 2, 2, 2}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_INTERP_BICUBIC_GRAYSCALE[] = {{32, 2, 4, 0, {2, 2, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_INTERP_BICUBIC_GRAYSCALE[] = {{36, 2, 4, 0, {2, 2, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_INTERP_BICUBIC_GRAYSCALE[] = {{40, 2, 4, 0, {2, 2, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_INTERP_BICUBIC_GRAYSCALE[] = {{44, 2, 4, 0, {2, 2, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_INTERP_BICUBIC_GRAYSCALE[] = {{48, 2, 4, 0, {2, 2, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_INTERP_BICUBIC_GRAYSCALE[] = {{52, 2, 4, 0, {2, 2, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_INTERP_BICUBIC_GRAYSCALE[] = {{56, 2, 4, 0, {2, 2, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_INTERP_BICUBIC_GRAYSCALE[] = {{60, 2, 4, 0, {2, 2, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_INTERP_BICUBIC_GRAYSCALE[] = {{64, 2, 4, 0, {2, 2, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_INTERP_BICUBIC_GRAYSCALE[] = {{72, 2, 4, 0, {2, 2, 2, 2}}, {72, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_INTERP_BICUBIC_GRAYSCALE[] = {{80, 2, 4, 0, {2, 2, 2, 2}}, {80, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_INTERP_BICUBIC_GRAYSCALE[] = {{88, 2, 4, 0, {2, 2, 2, 2}}, {88, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_INTERP_BICUBIC_GRAYSCALE[] = {{96, 2, 4, 0, {2, 2, 2, 2}}, {96, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_INTERP_BICUBIC_GRAYSCALE[] = {{104, 2, 4, 0, {2, 2, 2, 2}}, {104, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_INTERP_BICUBIC_GRAYSCALE[] = {{112, 2, 4, 0, {2, 2, 2, 2}}, {112, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_INTERP_BICUBIC_GRAYSCALE[] = {{120, 2, 4, 0, {2, 2, 2, 2}}, {120, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_INTERP_BICUBIC_GRAYSCALE[] = {{128, 2, 4, 0, {2, 2, 2, 2}}, {128, 2, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_INTERP_BICUBIC_GRAYSCALE[] = {{2, 3, 4, 0, {2, 2, 2, 2}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_INTERP_BICUBIC_GRAYSCALE[] = {{4, 3, 4, 0, {2, 2, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_INTERP_BICUBIC_GRAYSCALE[] = {{6, 3, 4, 0, {2, 2, 2, 2}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_INTERP_BICUBIC_GRAYSCALE[] = {{8, 3, 4, 0, {2, 2, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_INTERP_BICUBIC_GRAYSCALE[] = {{10, 3, 4, 0, {2, 2, 2, 2}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_INTERP_BICUBIC_GRAYSCALE[] = {{12, 3, 4, 0, {2, 2, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_INTERP_BICUBIC_GRAYSCALE[] = {{14, 3, 4, 0, {2, 2, 2, 2}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_INTERP_BICUBIC_GRAYSCALE[] = {{16, 3, 4, 0, {2, 2, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_INTERP_BICUBIC_GRAYSCALE[] = {{18, 3, 4, 0, {2, 2, 2, 2}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_INTERP_BICUBIC_GRAYSCALE[] = {{20, 3, 4, 0, {2, 2, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_INTERP_BICUBIC_GRAYSCALE[] = {{22, 3, 4, 0, {2, 2, 2, 2}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_INTERP_BICUBIC_GRAYSCALE[] = {{24, 3, 4, 0, {2, 2, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_INTERP_BICUBIC_GRAYSCALE[] = {{26, 3, 4, 0, {2, 2, 2, 2}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_INTERP_BICUBIC_GRAYSCALE[] = {{28, 3, 4, 0, {2, 2, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_INTERP_BICUBIC_GRAYSCALE[] = {{30, 3, 4, 0, {2, 2, 2, 2}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_INTERP_BICUBIC_GRAYSCALE[] = {{32, 3, 4, 0, {2, 2, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_INTERP_BICUBIC_GRAYSCALE[] = {{36, 3, 4, 0, {2, 2, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_INTERP_BICUBIC_GRAYSCALE[] = {{40, 3, 4, 0, {2, 2, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_INTERP_BICUBIC_GRAYSCALE[] = {{44, 3, 4, 0, {2, 2, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_INTERP_BICUBIC_GRAYSCALE[] = {{48, 3, 4, 0, {2, 2, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_INTERP_BICUBIC_GRAYSCALE[] = {{52, 3, 4, 0, {2, 2, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_INTERP_BICUBIC_GRAYSCALE[] = {{56, 3, 4, 0, {2, 2, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_INTERP_BICUBIC_GRAYSCALE[] = {{60, 3, 4, 0, {2, 2, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_INTERP_BICUBIC_GRAYSCALE[] = {{64, 3, 4, 0, {2, 2, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_INTERP_BICUBIC_GRAYSCALE[] = {{72, 3, 4, 0, {2, 2, 2, 2}}, {72, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_INTERP_BICUBIC_GRAYSCALE[] = {{80, 3, 4, 0, {2, 2, 2, 2}}, {80, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_INTERP_BICUBIC_GRAYSCALE[] = {{88, 3, 4, 0, {2, 2, 2, 2}}, {88, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_INTERP_BICUBIC_GRAYSCALE[] = {{96, 3, 4, 0, {2, 2, 2, 2}}, {96, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_INTERP_BICUBIC_GRAYSCALE[] = {{104, 3, 4, 0, {2, 2, 2, 2}}, {104, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_INTERP_BICUBIC_GRAYSCALE[] = {{112, 3, 4, 0, {2, 2, 2, 2}}, {112, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_INTERP_BICUBIC_GRAYSCALE[] = {{120, 3, 4, 0, {2, 2, 2, 2}}, {120, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_INTERP_BICUBIC_GRAYSCALE[] = {{128, 3, 4, 0, {2, 2, 2, 2}}, {128, 3, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_INTERP_BICUBIC_GRAYSCALE[] = {{2, 4, 4, 0, {2, 2, 2, 2}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_INTERP_BICUBIC_GRAYSCALE[] = {{4, 4, 4, 0, {2, 2, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_INTERP_BICUBIC_GRAYSCALE[] = {{6, 4, 4, 0, {2, 2, 2, 2}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_INTERP_BICUBIC_GRAYSCALE[] = {{8, 4, 4, 0, {2, 2, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_INTERP_BICUBIC_GRAYSCALE[] = {{10, 4, 4, 0, {2, 2, 2, 2}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_INTERP_BICUBIC_GRAYSCALE[] = {{12, 4, 4, 0, {2, 2, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_INTERP_BICUBIC_GRAYSCALE[] = {{14, 4, 4, 0, {2, 2, 2, 2}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_INTERP_BICUBIC_GRAYSCALE[] = {{16, 4, 4, 0, {2, 2, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_INTERP_BICUBIC_GRAYSCALE[] = {{18, 4, 4, 0, {2, 2, 2, 2}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_INTERP_BICUBIC_GRAYSCALE[] = {{20, 4, 4, 0, {2, 2, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_INTERP_BICUBIC_GRAYSCALE[] = {{22, 4, 4, 0, {2, 2, 2, 2}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_INTERP_BICUBIC_GRAYSCALE[] = {{24, 4, 4, 0, {2, 2, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_INTERP_BICUBIC_GRAYSCALE[] = {{26, 4, 4, 0, {2, 2, 2, 2}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_INTERP_BICUBIC_GRAYSCALE[] = {{28, 4, 4, 0, {2, 2, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_INTERP_BICUBIC_GRAYSCALE[] = {{30, 4, 4, 0, {2, 2, 2, 2}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_INTERP_BICUBIC_GRAYSCALE[] = {{32, 4, 4, 0, {2, 2, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_INTERP_BICUBIC_GRAYSCALE[] = {{36, 4, 4, 0, {2, 2, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_INTERP_BICUBIC_GRAYSCALE[] = {{40, 4, 4, 0, {2, 2, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_INTERP_BICUBIC_GRAYSCALE[] = {{44, 4, 4, 0, {2, 2, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_INTERP_BICUBIC_GRAYSCALE[] = {{48, 4, 4, 0, {2, 2, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_INTERP_BICUBIC_GRAYSCALE[] = {{52, 4, 4, 0, {2, 2, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_INTERP_BICUBIC_GRAYSCALE[] = {{56, 4, 4, 0, {2, 2, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_INTERP_BICUBIC_GRAYSCALE[] = {{60, 4, 4, 0, {2, 2, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_INTERP_BICUBIC_GRAYSCALE[] = {{64, 4, 4, 0, {2, 2, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_INTERP_BICUBIC_GRAYSCALE[] = {{72, 4, 4, 0, {2, 2, 2, 2}}, {72, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_INTERP_BICUBIC_GRAYSCALE[] = {{80, 4, 4, 0, {2, 2, 2, 2}}, {80, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_INTERP_BICUBIC_GRAYSCALE[] = {{88, 4, 4, 0, {2, 2, 2, 2}}, {88, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_INTERP_BICUBIC_GRAYSCALE[] = {{96, 4, 4, 0, {2, 2, 2, 2}}, {96, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_INTERP_BICUBIC_GRAYSCALE[] = {{104, 4, 4, 0, {2, 2, 2, 2}}, {104, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_INTERP_BICUBIC_GRAYSCALE[] = {{112, 4, 4, 0, {2, 2, 2, 2}}, {112, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_INTERP_BICUBIC_GRAYSCALE[] = {{120, 4, 4, 0, {2, 2, 2, 2}}, {120, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_INTERP_BICUBIC_GRAYSCALE[] = {{128, 4, 4, 0, {2, 2, 2, 2}}, {128, 4, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_INTERP_BICUBIC_GRAYSCALE[] = {{2, 5, 4, 0, {2, 2, 2, 2}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_INTERP_BICUBIC_GRAYSCALE[] = {{4, 5, 4, 0, {2, 2, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_INTERP_BICUBIC_GRAYSCALE[] = {{6, 5, 4, 0, {2, 2, 2, 2}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_INTERP_BICUBIC_GRAYSCALE[] = {{8, 5, 4, 0, {2, 2, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_INTERP_BICUBIC_GRAYSCALE[] = {{10, 5, 4, 0, {2, 2, 2, 2}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_INTERP_BICUBIC_GRAYSCALE[] = {{12, 5, 4, 0, {2, 2, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_INTERP_BICUBIC_GRAYSCALE[] = {{14, 5, 4, 0, {2, 2, 2, 2}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_INTERP_BICUBIC_GRAYSCALE[] = {{16, 5, 4, 0, {2, 2, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_INTERP_BICUBIC_GRAYSCALE[] = {{18, 5, 4, 0, {2, 2, 2, 2}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_INTERP_BICUBIC_GRAYSCALE[] = {{20, 5, 4, 0, {2, 2, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_INTERP_BICUBIC_GRAYSCALE[] = {{22, 5, 4, 0, {2, 2, 2, 2}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_INTERP_BICUBIC_GRAYSCALE[] = {{24, 5, 4, 0, {2, 2, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_INTERP_BICUBIC_GRAYSCALE[] = {{26, 5, 4, 0, {2, 2, 2, 2}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_INTERP_BICUBIC_GRAYSCALE[] = {{28, 5, 4, 0, {2, 2, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_INTERP_BICUBIC_GRAYSCALE[] = {{30, 5, 4, 0, {2, 2, 2, 2}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_INTERP_BICUBIC_GRAYSCALE[] = {{32, 5, 4, 0, {2, 2, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_INTERP_BICUBIC_GRAYSCALE[] = {{36, 5, 4, 0, {2, 2, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_INTERP_BICUBIC_GRAYSCALE[] = {{40, 5, 4, 0, {2, 2, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_INTERP_BICUBIC_GRAYSCALE[] = {{44, 5, 4, 0, {2, 2, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_INTERP_BICUBIC_GRAYSCALE[] = {{48, 5, 4, 0, {2, 2, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_INTERP_BICUBIC_GRAYSCALE[] = {{52, 5, 4, 0, {2, 2, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_INTERP_BICUBIC_GRAYSCALE[] = {{56, 5, 4, 0, {2, 2, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_INTERP_BICUBIC_GRAYSCALE[] = {{60, 5, 4, 0, {2, 2, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_INTERP_BICUBIC_GRAYSCALE[] = {{64, 5, 4, 0, {2, 2, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_INTERP_BICUBIC_GRAYSCALE[] = {{72, 5, 4, 0, {2, 2, 2, 2}}, {72, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_INTERP_BICUBIC_GRAYSCALE[] = {{80, 5, 4, 0, {2, 2, 2, 2}}, {80, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_INTERP_BICUBIC_GRAYSCALE[] = {{88, 5, 4, 0, {2, 2, 2, 2}}, {88, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_INTERP_BICUBIC_GRAYSCALE[] = {{96, 5, 4, 0, {2, 2, 2, 2}}, {96, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_INTERP_BICUBIC_GRAYSCALE[] = {{104, 5, 4, 0, {2, 2, 2, 2}}, {104, 5, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_INTERP_BICUBIC_GRAYSCALE[] = {{2, 6, 4, 0, {2, 2, 2, 2}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_INTERP_BICUBIC_GRAYSCALE[] = {{4, 6, 4, 0, {2, 2, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_INTERP_BICUBIC_GRAYSCALE[] = {{6, 6, 4, 0, {2, 2, 2, 2}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_INTERP_BICUBIC_GRAYSCALE[] = {{8, 6, 4, 0, {2, 2, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_INTERP_BICUBIC_GRAYSCALE[] = {{10, 6, 4, 0, {2, 2, 2, 2}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_INTERP_BICUBIC_GRAYSCALE[] = {{12, 6, 4, 0, {2, 2, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_INTERP_BICUBIC_GRAYSCALE[] = {{14, 6, 4, 0, {2, 2, 2, 2}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_INTERP_BICUBIC_GRAYSCALE[] = {{16, 6, 4, 0, {2, 2, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_INTERP_BICUBIC_GRAYSCALE[] = {{18, 6, 4, 0, {2, 2, 2, 2}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_INTERP_BICUBIC_GRAYSCALE[] = {{20, 6, 4, 0, {2, 2, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_INTERP_BICUBIC_GRAYSCALE[] = {{22, 6, 4, 0, {2, 2, 2, 2}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_INTERP_BICUBIC_GRAYSCALE[] = {{24, 6, 4, 0, {2, 2, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_INTERP_BICUBIC_GRAYSCALE[] = {{26, 6, 4, 0, {2, 2, 2, 2}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_INTERP_BICUBIC_GRAYSCALE[] = {{28, 6, 4, 0, {2, 2, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_INTERP_BICUBIC_GRAYSCALE[] = {{30, 6, 4, 0, {2, 2, 2, 2}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_INTERP_BICUBIC_GRAYSCALE[] = {{32, 6, 4, 0, {2, 2, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_INTERP_BICUBIC_GRAYSCALE[] = {{36, 6, 4, 0, {2, 2, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_INTERP_BICUBIC_GRAYSCALE[] = {{40, 6, 4, 0, {2, 2, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_INTERP_BICUBIC_GRAYSCALE[] = {{44, 6, 4, 0, {2, 2, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_INTERP_BICUBIC_GRAYSCALE[] = {{48, 6, 4, 0, {2, 2, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_INTERP_BICUBIC_GRAYSCALE[] = {{52, 6, 4, 0, {2, 2, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_INTERP_BICUBIC_GRAYSCALE[] = {{56, 6, 4, 0, {2, 2, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_INTERP_BICUBIC_GRAYSCALE[] = {{60, 6, 4, 0, {2, 2, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_INTERP_BICUBIC_GRAYSCALE[] = {{64, 6, 4, 0, {2, 2, 2, 2}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_INTERP_BICUBIC_GRAYSCALE[] = {{72, 6, 4, 0, {2, 2, 2, 2}}, {72, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_INTERP_BICUBIC_GRAYSCALE[] = {{80, 6, 4, 0, {2, 2, 2, 2}}, {80, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_INTERP_BICUBIC_GRAYSCALE[] = {{88, 6, 4, 0, {2, 2, 2, 2}}, {88, 6, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_INTERP_BICUBIC_GRAYSCALE[] = {{2, 8, 4, 0, {2, 2, 2, 2}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_INTERP_BICUBIC_GRAYSCALE[] = {{4, 8, 4, 0, {2, 2, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_INTERP_BICUBIC_GRAYSCALE[] = {{6, 8, 4, 0, {2, 2, 2, 2}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_INTERP_BICUBIC_GRAYSCALE[] = {{8, 8, 4, 0, {2, 2, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_INTERP_BICUBIC_GRAYSCALE[] = {{10, 8, 4, 0, {2, 2, 2, 2}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_INTERP_BICUBIC_GRAYSCALE[] = {{12, 8, 4, 0, {2, 2, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_INTERP_BICUBIC_GRAYSCALE[] = {{14, 8, 4, 0, {2, 2, 2, 2}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_INTERP_BICUBIC_GRAYSCALE[] = {{16, 8, 4, 0, {2, 2, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_INTERP_BICUBIC_GRAYSCALE[] = {{18, 8, 4, 0, {2, 2, 2, 2}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_INTERP_BICUBIC_GRAYSCALE[] = {{20, 8, 4, 0, {2, 2, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_INTERP_BICUBIC_GRAYSCALE[] = {{22, 8, 4, 0, {2, 2, 2, 2}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_INTERP_BICUBIC_GRAYSCALE[] = {{24, 8, 4, 0, {2, 2, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_INTERP_BICUBIC_GRAYSCALE[] = {{26, 8, 4, 0, {2, 2, 2, 2}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_INTERP_BICUBIC_GRAYSCALE[] = {{28, 8, 4, 0, {2, 2, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_INTERP_BICUBIC_GRAYSCALE[] = {{30, 8, 4, 0, {2, 2, 2, 2}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_INTERP_BICUBIC_GRAYSCALE[] = {{32, 8, 4, 0, {2, 2, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_INTERP_BICUBIC_GRAYSCALE[] = {{36, 8, 4, 0, {2, 2, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_INTERP_BICUBIC_GRAYSCALE[] = {{40, 8, 4, 0, {2, 2, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_INTERP_BICUBIC_GRAYSCALE[] = {{44, 8, 4, 0, {2, 2, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_INTERP_BICUBIC_GRAYSCALE[] = {{48, 8, 4, 0, {2, 2, 2, 2}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_INTERP_BICUBIC_GRAYSCALE[] = {{52, 8, 4, 0, {2, 2, 2, 2}}, {52, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_INTERP_BICUBIC_GRAYSCALE[] = {{56, 8, 4, 0, {2, 2, 2, 2}}, {56, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_INTERP_BICUBIC_GRAYSCALE[] = {{60, 8, 4, 0, {2, 2, 2, 2}}, {60, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_INTERP_BICUBIC_GRAYSCALE[] = {{64, 8, 4, 0, {2, 2, 2, 2}}, {64, 8, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_INTERP_BICUBIC_GRAYSCALE[] = {{2, 10, 4, 0, {2, 2, 2, 2}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_INTERP_BICUBIC_GRAYSCALE[] = {{4, 10, 4, 0, {2, 2, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_INTERP_BICUBIC_GRAYSCALE[] = {{6, 10, 4, 0, {2, 2, 2, 2}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_INTERP_BICUBIC_GRAYSCALE[] = {{8, 10, 4, 0, {2, 2, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_INTERP_BICUBIC_GRAYSCALE[] = {{10, 10, 4, 0, {2, 2, 2, 2}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_INTERP_BICUBIC_GRAYSCALE[] = {{12, 10, 4, 0, {2, 2, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_INTERP_BICUBIC_GRAYSCALE[] = {{14, 10, 4, 0, {2, 2, 2, 2}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_INTERP_BICUBIC_GRAYSCALE[] = {{16, 10, 4, 0, {2, 2, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_INTERP_BICUBIC_GRAYSCALE[] = {{18, 10, 4, 0, {2, 2, 2, 2}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_INTERP_BICUBIC_GRAYSCALE[] = {{20, 10, 4, 0, {2, 2, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_INTERP_BICUBIC_GRAYSCALE[] = {{22, 10, 4, 0, {2, 2, 2, 2}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_INTERP_BICUBIC_GRAYSCALE[] = {{24, 10, 4, 0, {2, 2, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_INTERP_BICUBIC_GRAYSCALE[] = {{26, 10, 4, 0, {2, 2, 2, 2}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_INTERP_BICUBIC_GRAYSCALE[] = {{28, 10, 4, 0, {2, 2, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_INTERP_BICUBIC_GRAYSCALE[] = {{30, 10, 4, 0, {2, 2, 2, 2}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_INTERP_BICUBIC_GRAYSCALE[] = {{32, 10, 4, 0, {2, 2, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_INTERP_BICUBIC_GRAYSCALE[] = {{36, 10, 4, 0, {2, 2, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_INTERP_BICUBIC_GRAYSCALE[] = {{40, 10, 4, 0, {2, 2, 2, 2}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_INTERP_BICUBIC_GRAYSCALE[] = {{44, 10, 4, 0, {2, 2, 2, 2}}, {44, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_INTERP_BICUBIC_GRAYSCALE[] = {{48, 10, 4, 0, {2, 2, 2, 2}}, {48, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_INTERP_BICUBIC_GRAYSCALE[] = {{52, 10, 4, 0, {2, 2, 2, 2}}, {52, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_INTERP_BICUBIC_GRAYSCALE[] = {{56, 10, 4, 0, {2, 2, 2, 2}}, {56, 10, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_INTERP_BICUBIC_GRAYSCALE[] = {{2, 12, 4, 0, {2, 2, 2, 2}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_INTERP_BICUBIC_GRAYSCALE[] = {{4, 12, 4, 0, {2, 2, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_INTERP_BICUBIC_GRAYSCALE[] = {{6, 12, 4, 0, {2, 2, 2, 2}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_INTERP_BICUBIC_GRAYSCALE[] = {{8, 12, 4, 0, {2, 2, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_INTERP_BICUBIC_GRAYSCALE[] = {{10, 12, 4, 0, {2, 2, 2, 2}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_INTERP_BICUBIC_GRAYSCALE[] = {{12, 12, 4, 0, {2, 2, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_INTERP_BICUBIC_GRAYSCALE[] = {{14, 12, 4, 0, {2, 2, 2, 2}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_INTERP_BICUBIC_GRAYSCALE[] = {{16, 12, 4, 0, {2, 2, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_INTERP_BICUBIC_GRAYSCALE[] = {{18, 12, 4, 0, {2, 2, 2, 2}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_INTERP_BICUBIC_GRAYSCALE[] = {{20, 12, 4, 0, {2, 2, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_INTERP_BICUBIC_GRAYSCALE[] = {{22, 12, 4, 0, {2, 2, 2, 2}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_INTERP_BICUBIC_GRAYSCALE[] = {{24, 12, 4, 0, {2, 2, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_INTERP_BICUBIC_GRAYSCALE[] = {{26, 12, 4, 0, {2, 2, 2, 2}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_INTERP_BICUBIC_GRAYSCALE[] = {{28, 12, 4, 0, {2, 2, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_INTERP_BICUBIC_GRAYSCALE[] = {{30, 12, 4, 0, {2, 2, 2, 2}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_INTERP_BICUBIC_GRAYSCALE[] = {{32, 12, 4, 0, {2, 2, 2, 2}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_INTERP_BICUBIC_GRAYSCALE[] = {{36, 12, 4, 0, {2, 2, 2, 2}}, {36, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_INTERP_BICUBIC_GRAYSCALE[] = {{40, 12, 4, 0, {2, 2, 2, 2}}, {40, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_INTERP_BICUBIC_GRAYSCALE[] = {{44, 12, 4, 0, {2, 2, 2, 2}}, {44, 12, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_INTERP_BICUBIC_GRAYSCALE[] = {{2, 14, 4, 0, {2, 2, 2, 2}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_INTERP_BICUBIC_GRAYSCALE[] = {{4, 14, 4, 0, {2, 2, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_INTERP_BICUBIC_GRAYSCALE[] = {{6, 14, 4, 0, {2, 2, 2, 2}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_INTERP_BICUBIC_GRAYSCALE[] = {{8, 14, 4, 0, {2, 2, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_INTERP_BICUBIC_GRAYSCALE[] = {{10, 14, 4, 0, {2, 2, 2, 2}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_INTERP_BICUBIC_GRAYSCALE[] = {{12, 14, 4, 0, {2, 2, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_INTERP_BICUBIC_GRAYSCALE[] = {{14, 14, 4, 0, {2, 2, 2, 2}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_INTERP_BICUBIC_GRAYSCALE[] = {{16, 14, 4, 0, {2, 2, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_INTERP_BICUBIC_GRAYSCALE[] = {{18, 14, 4, 0, {2, 2, 2, 2}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_INTERP_BICUBIC_GRAYSCALE[] = {{20, 14, 4, 0, {2, 2, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_INTERP_BICUBIC_GRAYSCALE[] = {{22, 14, 4, 0, {2, 2, 2, 2}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_INTERP_BICUBIC_GRAYSCALE[] = {{24, 14, 4, 0, {2, 2, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_INTERP_BICUBIC_GRAYSCALE[] = {{26, 14, 4, 0, {2, 2, 2, 2}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_INTERP_BICUBIC_GRAYSCALE[] = {{28, 14, 4, 0, {2, 2, 2, 2}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_INTERP_BICUBIC_GRAYSCALE[] = {{30, 14, 4, 0, {2, 2, 2, 2}}, {30, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_INTERP_BICUBIC_GRAYSCALE[] = {{32, 14, 4, 0, {2, 2, 2, 2}}, {32, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_INTERP_BICUBIC_GRAYSCALE[] = {{36, 14, 4, 0, {2, 2, 2, 2}}, {36, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_INTERP_BICUBIC_GRAYSCALE[] = {{40, 14, 4, 0, {2, 2, 2, 2}}, {40, 14, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_INTERP_BICUBIC_GRAYSCALE[] = {{2, 16, 4, 0, {2, 2, 2, 2}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_INTERP_BICUBIC_GRAYSCALE[] = {{4, 16, 4, 0, {2, 2, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_INTERP_BICUBIC_GRAYSCALE[] = {{6, 16, 4, 0, {2, 2, 2, 2}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_INTERP_BICUBIC_GRAYSCALE[] = {{8, 16, 4, 0, {2, 2, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_INTERP_BICUBIC_GRAYSCALE[] = {{10, 16, 4, 0, {2, 2, 2, 2}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_INTERP_BICUBIC_GRAYSCALE[] = {{12, 16, 4, 0, {2, 2, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_INTERP_BICUBIC_GRAYSCALE[] = {{14, 16, 4, 0, {2, 2, 2, 2}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_INTERP_BICUBIC_GRAYSCALE[] = {{16, 16, 4, 0, {2, 2, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_INTERP_BICUBIC_GRAYSCALE[] = {{18, 16, 4, 0, {2, 2, 2, 2}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_INTERP_BICUBIC_GRAYSCALE[] = {{20, 16, 4, 0, {2, 2, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_INTERP_BICUBIC_GRAYSCALE[] = {{22, 16, 4, 0, {2, 2, 2, 2}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_INTERP_BICUBIC_GRAYSCALE[] = {{24, 16, 4, 0, {2, 2, 2, 2}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_INTERP_BICUBIC_GRAYSCALE[] = {{26, 16, 4, 0, {2, 2, 2, 2}}, {26, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_INTERP_BICUBIC_GRAYSCALE[] = {{28, 16, 4, 0, {2, 2, 2, 2}}, {28, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_INTERP_BICUBIC_GRAYSCALE[] = {{30, 16, 4, 0, {2, 2, 2, 2}}, {30, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_INTERP_BICUBIC_GRAYSCALE[] = {{32, 16, 4, 0, {2, 2, 2, 2}}, {32, 16, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_INTERP_BICUBIC_GRAYSCALE[] = {{2, 18, 4, 0, {2, 2, 2, 2}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_INTERP_BICUBIC_GRAYSCALE[] = {{4, 18, 4, 0, {2, 2, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_INTERP_BICUBIC_GRAYSCALE[] = {{6, 18, 4, 0, {2, 2, 2, 2}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_INTERP_BICUBIC_GRAYSCALE[] = {{8, 18, 4, 0, {2, 2, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_INTERP_BICUBIC_GRAYSCALE[] = {{10, 18, 4, 0, {2, 2, 2, 2}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_INTERP_BICUBIC_GRAYSCALE[] = {{12, 18, 4, 0, {2, 2, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_INTERP_BICUBIC_GRAYSCALE[] = {{14, 18, 4, 0, {2, 2, 2, 2}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_INTERP_BICUBIC_GRAYSCALE[] = {{16, 18, 4, 0, {2, 2, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_INTERP_BICUBIC_GRAYSCALE[] = {{18, 18, 4, 0, {2, 2, 2, 2}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_INTERP_BICUBIC_GRAYSCALE[] = {{20, 18, 4, 0, {2, 2, 2, 2}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_INTERP_BICUBIC_GRAYSCALE[] = {{22, 18, 4, 0, {2, 2, 2, 2}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_INTERP_BICUBIC_GRAYSCALE[] = {{24, 18, 4, 0, {2, 2, 2, 2}}, {24, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_INTERP_BICUBIC_GRAYSCALE[] = {{26, 18, 4, 0, {2, 2, 2, 2}}, {26, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_INTERP_BICUBIC_GRAYSCALE[] = {{28, 18, 4, 0, {2, 2, 2, 2}}, {28, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_INTERP_BICUBIC_GRAYSCALE[] = {{30, 18, 4, 0, {2, 2, 2, 2}}, {30, 18, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_INTERP_BICUBIC_GRAYSCALE[] = {{2, 20, 4, 0, {2, 2, 2, 2}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_INTERP_BICUBIC_GRAYSCALE[] = {{4, 20, 4, 0, {2, 2, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_INTERP_BICUBIC_GRAYSCALE[] = {{6, 20, 4, 0, {2, 2, 2, 2}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_INTERP_BICUBIC_GRAYSCALE[] = {{8, 20, 4, 0, {2, 2, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_INTERP_BICUBIC_GRAYSCALE[] = {{10, 20, 4, 0, {2, 2, 2, 2}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_INTERP_BICUBIC_GRAYSCALE[] = {{12, 20, 4, 0, {2, 2, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_INTERP_BICUBIC_GRAYSCALE[] = {{14, 20, 4, 0, {2, 2, 2, 2}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_INTERP_BICUBIC_GRAYSCALE[] = {{16, 20, 4, 0, {2, 2, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_INTERP_BICUBIC_GRAYSCALE[] = {{18, 20, 4, 0, {2, 2, 2, 2}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_INTERP_BICUBIC_GRAYSCALE[] = {{20, 20, 4, 0, {2, 2, 2, 2}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_INTERP_BICUBIC_GRAYSCALE[] = {{22, 20, 4, 0, {2, 2, 2, 2}}, {22, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_INTERP_BICUBIC_GRAYSCALE[] = {{24, 20, 4, 0, {2, 2, 2, 2}}, {24, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_INTERP_BICUBIC_GRAYSCALE[] = {{26, 20, 4, 0, {2, 2, 2, 2}}, {26, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_INTERP_BICUBIC_GRAYSCALE[] = {{28, 20, 4, 0, {2, 2, 2, 2}}, {28, 20, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_INTERP_BICUBIC_GRAYSCALE[] = {{2, 22, 4, 0, {2, 2, 2, 2}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_INTERP_BICUBIC_GRAYSCALE[] = {{4, 22, 4, 0, {2, 2, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_INTERP_BICUBIC_GRAYSCALE[] = {{6, 22, 4, 0, {2, 2, 2, 2}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_INTERP_BICUBIC_GRAYSCALE[] = {{8, 22, 4, 0, {2, 2, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_INTERP_BICUBIC_GRAYSCALE[] = {{10, 22, 4, 0, {2, 2, 2, 2}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_INTERP_BICUBIC_GRAYSCALE[] = {{12, 22, 4, 0, {2, 2, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_INTERP_BICUBIC_GRAYSCALE[] = {{14, 22, 4, 0, {2, 2, 2, 2}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_INTERP_BICUBIC_GRAYSCALE[] = {{16, 22, 4, 0, {2, 2, 2, 2}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_INTERP_BICUBIC_GRAYSCALE[] = {{18, 22, 4, 0, {2, 2, 2, 2}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_INTERP_BICUBIC_GRAYSCALE[] = {{20, 22, 4, 0, {2, 2, 2, 2}}, {20, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_INTERP_BICUBIC_GRAYSCALE[] = {{22, 22, 4, 0, {2, 2, 2, 2}}, {22, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_INTERP_BICUBIC_GRAYSCALE[] = {{24, 22, 4, 0, {2, 2, 2, 2}}, {24, 22, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_INTERP_BICUBIC_GRAYSCALE[] = {{2, 24, 4, 0, {2, 2, 2, 2}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_INTERP_BICUBIC_GRAYSCALE[] = {{4, 24, 4, 0, {2, 2, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_INTERP_BICUBIC_GRAYSCALE[] = {{6, 24, 4, 0, {2, 2, 2, 2}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_INTERP_BICUBIC_GRAYSCALE[] = {{8, 24, 4, 0, {2, 2, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_INTERP_BICUBIC_GRAYSCALE[] = {{10, 24, 4, 0, {2, 2, 2, 2}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_INTERP_BICUBIC_GRAYSCALE[] = {{12, 24, 4, 0, {2, 2, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_INTERP_BICUBIC_GRAYSCALE[] = {{14, 24, 4, 0, {2, 2, 2, 2}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_INTERP_BICUBIC_GRAYSCALE[] = {{16, 24, 4, 0, {2, 2, 2, 2}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_INTERP_BICUBIC_GRAYSCALE[] = {{18, 24, 4, 0, {2, 2, 2, 2}}, {18, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_INTERP_BICUBIC_GRAYSCALE[] = {{20, 24, 4, 0, {2, 2, 2, 2}}, {20, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_INTERP_BICUBIC_GRAYSCALE[] = {{22, 24, 4, 0, {2, 2, 2, 2}}, {22, 24, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_INTERP_BICUBIC_GRAYSCALE[] = {{2, 26, 4, 0, {2, 2, 2, 2}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_INTERP_BICUBIC_GRAYSCALE[] = {{4, 26, 4, 0, {2, 2, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_INTERP_BICUBIC_GRAYSCALE[] = {{6, 26, 4, 0, {2, 2, 2, 2}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_INTERP_BICUBIC_GRAYSCALE[] = {{8, 26, 4, 0, {2, 2, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_INTERP_BICUBIC_GRAYSCALE[] = {{10, 26, 4, 0, {2, 2, 2, 2}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_INTERP_BICUBIC_GRAYSCALE[] = {{12, 26, 4, 0, {2, 2, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_INTERP_BICUBIC_GRAYSCALE[] = {{14, 26, 4, 0, {2, 2, 2, 2}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_INTERP_BICUBIC_GRAYSCALE[] = {{16, 26, 4, 0, {2, 2, 2, 2}}, {16, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_INTERP_BICUBIC_GRAYSCALE[] = {{18, 26, 4, 0, {2, 2, 2, 2}}, {18, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_INTERP_BICUBIC_GRAYSCALE[] = {{20, 26, 4, 0, {2, 2, 2, 2}}, {20, 26, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_INTERP_BICUBIC_GRAYSCALE[] = {{2, 28, 4, 0, {2, 2, 2, 2}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_INTERP_BICUBIC_GRAYSCALE[] = {{4, 28, 4, 0, {2, 2, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_INTERP_BICUBIC_GRAYSCALE[] = {{6, 28, 4, 0, {2, 2, 2, 2}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_INTERP_BICUBIC_GRAYSCALE[] = {{8, 28, 4, 0, {2, 2, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_INTERP_BICUBIC_GRAYSCALE[] = {{10, 28, 4, 0, {2, 2, 2, 2}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_INTERP_BICUBIC_GRAYSCALE[] = {{12, 28, 4, 0, {2, 2, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_INTERP_BICUBIC_GRAYSCALE[] = {{14, 28, 4, 0, {2, 2, 2, 2}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_INTERP_BICUBIC_GRAYSCALE[] = {{16, 28, 4, 0, {2, 2, 2, 2}}, {16, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_INTERP_BICUBIC_GRAYSCALE[] = {{18, 28, 4, 0, {2, 2, 2, 2}}, {18, 28, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_INTERP_BICUBIC_GRAYSCALE[] = {{2, 30, 4, 0, {2, 2, 2, 2}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_INTERP_BICUBIC_GRAYSCALE[] = {{4, 30, 4, 0, {2, 2, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_INTERP_BICUBIC_GRAYSCALE[] = {{6, 30, 4, 0, {2, 2, 2, 2}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_INTERP_BICUBIC_GRAYSCALE[] = {{8, 30, 4, 0, {2, 2, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_INTERP_BICUBIC_GRAYSCALE[] = {{10, 30, 4, 0, {2, 2, 2, 2}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_INTERP_BICUBIC_GRAYSCALE[] = {{12, 30, 4, 0, {2, 2, 2, 2}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_INTERP_BICUBIC_GRAYSCALE[] = {{14, 30, 4, 0, {2, 2, 2, 2}}, {14, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_INTERP_BICUBIC_GRAYSCALE[] = {{16, 30, 4, 0, {2, 2, 2, 2}}, {16, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_INTERP_BICUBIC_GRAYSCALE[] = {{18, 30, 4, 0, {2, 2, 2, 2}}, {18, 30, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_INTERP_BICUBIC_GRAYSCALE[] = {{2, 32, 4, 0, {2, 2, 2, 2}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_INTERP_BICUBIC_GRAYSCALE[] = {{4, 32, 4, 0, {2, 2, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_INTERP_BICUBIC_GRAYSCALE[] = {{6, 32, 4, 0, {2, 2, 2, 2}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_INTERP_BICUBIC_GRAYSCALE[] = {{8, 32, 4, 0, {2, 2, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_INTERP_BICUBIC_GRAYSCALE[] = {{10, 32, 4, 0, {2, 2, 2, 2}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_INTERP_BICUBIC_GRAYSCALE[] = {{12, 32, 4, 0, {2, 2, 2, 2}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_INTERP_BICUBIC_GRAYSCALE[] = {{14, 32, 4, 0, {2, 2, 2, 2}}, {14, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_INTERP_BICUBIC_GRAYSCALE[] = {{16, 32, 4, 0, {2, 2, 2, 2}}, {16, 32, 2, 1, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INTERP_BICUBIC_GRAYSCALE[] = {{2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_INTERP_BICUBIC_GRAYSCALE[] = {{1, 0}};

static acf_scenario gScenarioArray_INTERP_BICUBIC_GRAYSCALE[] = {
{2, 1, 72, 32, 2, gScenarioBufferData0_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData0_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 1, 96, 32, 2, gScenarioBufferData1_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData1_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 1, 124, 32, 2, gScenarioBufferData2_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData2_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 1, 148, 32, 2, gScenarioBufferData3_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData3_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 1, 176, 32, 2, gScenarioBufferData4_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData4_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 1, 200, 32, 2, gScenarioBufferData5_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData5_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 1, 228, 32, 2, gScenarioBufferData6_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData6_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 1, 252, 32, 2, gScenarioBufferData7_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData7_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 1, 280, 32, 2, gScenarioBufferData8_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData8_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 1, 304, 32, 2, gScenarioBufferData9_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData9_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 1, 332, 32, 2, gScenarioBufferData10_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData10_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 1, 356, 32, 2, gScenarioBufferData11_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData11_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 1, 384, 32, 2, gScenarioBufferData12_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData12_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 1, 408, 32, 2, gScenarioBufferData13_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData13_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 1, 436, 32, 2, gScenarioBufferData14_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData14_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 1, 460, 32, 2, gScenarioBufferData15_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData15_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 1, 512, 32, 2, gScenarioBufferData16_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData16_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 1, 564, 32, 2, gScenarioBufferData17_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData17_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 1, 616, 32, 2, gScenarioBufferData18_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData18_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 1, 668, 32, 2, gScenarioBufferData19_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData19_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 1, 720, 32, 2, gScenarioBufferData20_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData20_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 1, 772, 32, 2, gScenarioBufferData21_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData21_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 1, 824, 32, 2, gScenarioBufferData22_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData22_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 1, 876, 32, 2, gScenarioBufferData23_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData23_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 1, 980, 32, 2, gScenarioBufferData24_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData24_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 1, 1084, 32, 2, gScenarioBufferData25_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData25_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 1, 1188, 32, 2, gScenarioBufferData26_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData26_INTERP_BICUBIC_GRAYSCALE, 4},
{96, 1, 1292, 32, 2, gScenarioBufferData27_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData27_INTERP_BICUBIC_GRAYSCALE, 4},
{104, 1, 1396, 32, 2, gScenarioBufferData28_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData28_INTERP_BICUBIC_GRAYSCALE, 4},
{112, 1, 1500, 32, 2, gScenarioBufferData29_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData29_INTERP_BICUBIC_GRAYSCALE, 4},
{120, 1, 1604, 32, 2, gScenarioBufferData30_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData30_INTERP_BICUBIC_GRAYSCALE, 4},
{128, 1, 1708, 32, 2, gScenarioBufferData31_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData31_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 2, 80, 32, 1, gScenarioBufferData32_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData32_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 2, 112, 32, 1, gScenarioBufferData33_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData33_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 2, 144, 32, 1, gScenarioBufferData34_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData34_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 2, 176, 32, 1, gScenarioBufferData35_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData35_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 2, 208, 32, 1, gScenarioBufferData36_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData36_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 2, 240, 32, 1, gScenarioBufferData37_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData37_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 2, 272, 32, 1, gScenarioBufferData38_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData38_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 2, 304, 32, 1, gScenarioBufferData39_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData39_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 2, 336, 32, 1, gScenarioBufferData40_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData40_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 2, 368, 32, 1, gScenarioBufferData41_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData41_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 2, 400, 32, 1, gScenarioBufferData42_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData42_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 2, 432, 32, 1, gScenarioBufferData43_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData43_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 2, 464, 32, 1, gScenarioBufferData44_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData44_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 2, 496, 32, 1, gScenarioBufferData45_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData45_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 2, 528, 32, 1, gScenarioBufferData46_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData46_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 2, 560, 32, 1, gScenarioBufferData47_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData47_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 2, 624, 32, 1, gScenarioBufferData48_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData48_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 2, 688, 32, 1, gScenarioBufferData49_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData49_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 2, 752, 32, 1, gScenarioBufferData50_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData50_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 2, 816, 32, 1, gScenarioBufferData51_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData51_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 2, 880, 32, 1, gScenarioBufferData52_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData52_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 2, 944, 32, 1, gScenarioBufferData53_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData53_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 2, 1008, 32, 1, gScenarioBufferData54_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData54_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 2, 1072, 32, 1, gScenarioBufferData55_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData55_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 2, 1200, 32, 1, gScenarioBufferData56_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData56_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 2, 1328, 32, 1, gScenarioBufferData57_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData57_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 2, 1456, 32, 1, gScenarioBufferData58_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData58_INTERP_BICUBIC_GRAYSCALE, 4},
{96, 2, 1584, 32, 1, gScenarioBufferData59_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData59_INTERP_BICUBIC_GRAYSCALE, 4},
{104, 2, 1712, 32, 1, gScenarioBufferData60_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData60_INTERP_BICUBIC_GRAYSCALE, 4},
{112, 2, 1840, 32, 1, gScenarioBufferData61_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData61_INTERP_BICUBIC_GRAYSCALE, 4},
{120, 2, 1968, 32, 1, gScenarioBufferData62_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData62_INTERP_BICUBIC_GRAYSCALE, 4},
{128, 2, 2096, 32, 1, gScenarioBufferData63_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData63_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 3, 108, 32, 1, gScenarioBufferData64_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData64_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 3, 152, 32, 1, gScenarioBufferData65_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData65_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 3, 196, 32, 1, gScenarioBufferData66_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData66_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 3, 240, 32, 1, gScenarioBufferData67_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData67_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 3, 284, 32, 1, gScenarioBufferData68_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData68_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 3, 328, 32, 1, gScenarioBufferData69_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData69_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 3, 372, 32, 1, gScenarioBufferData70_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData70_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 3, 416, 32, 1, gScenarioBufferData71_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData71_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 3, 460, 32, 1, gScenarioBufferData72_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData72_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 3, 504, 32, 1, gScenarioBufferData73_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData73_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 3, 548, 32, 1, gScenarioBufferData74_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData74_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 3, 592, 32, 1, gScenarioBufferData75_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData75_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 3, 636, 32, 1, gScenarioBufferData76_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData76_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 3, 680, 32, 1, gScenarioBufferData77_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData77_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 3, 724, 32, 1, gScenarioBufferData78_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData78_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 3, 768, 32, 1, gScenarioBufferData79_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData79_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 3, 856, 32, 1, gScenarioBufferData80_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData80_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 3, 944, 32, 1, gScenarioBufferData81_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData81_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 3, 1032, 32, 1, gScenarioBufferData82_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData82_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 3, 1120, 32, 1, gScenarioBufferData83_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData83_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 3, 1208, 32, 1, gScenarioBufferData84_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData84_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 3, 1296, 32, 1, gScenarioBufferData85_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData85_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 3, 1384, 32, 1, gScenarioBufferData86_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData86_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 3, 1472, 32, 1, gScenarioBufferData87_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData87_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 3, 1648, 32, 1, gScenarioBufferData88_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData88_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 3, 1824, 32, 1, gScenarioBufferData89_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData89_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 3, 2000, 32, 1, gScenarioBufferData90_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData90_INTERP_BICUBIC_GRAYSCALE, 4},
{96, 3, 2176, 32, 1, gScenarioBufferData91_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData91_INTERP_BICUBIC_GRAYSCALE, 4},
{104, 3, 2352, 32, 1, gScenarioBufferData92_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData92_INTERP_BICUBIC_GRAYSCALE, 4},
{112, 3, 2528, 32, 1, gScenarioBufferData93_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData93_INTERP_BICUBIC_GRAYSCALE, 4},
{120, 3, 2704, 32, 1, gScenarioBufferData94_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData94_INTERP_BICUBIC_GRAYSCALE, 4},
{128, 3, 2880, 32, 1, gScenarioBufferData95_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData95_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 4, 136, 32, 1, gScenarioBufferData96_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData96_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 4, 192, 32, 1, gScenarioBufferData97_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData97_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 4, 248, 32, 1, gScenarioBufferData98_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData98_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 4, 304, 32, 1, gScenarioBufferData99_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData99_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 4, 360, 32, 1, gScenarioBufferData100_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData100_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 4, 416, 32, 1, gScenarioBufferData101_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData101_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 4, 472, 32, 1, gScenarioBufferData102_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData102_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 4, 528, 32, 1, gScenarioBufferData103_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData103_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 4, 584, 32, 1, gScenarioBufferData104_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData104_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 4, 640, 32, 1, gScenarioBufferData105_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData105_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 4, 696, 32, 1, gScenarioBufferData106_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData106_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 4, 752, 32, 1, gScenarioBufferData107_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData107_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 4, 808, 32, 1, gScenarioBufferData108_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData108_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 4, 864, 32, 1, gScenarioBufferData109_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData109_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 4, 920, 32, 1, gScenarioBufferData110_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData110_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 4, 976, 32, 1, gScenarioBufferData111_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData111_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 4, 1088, 32, 1, gScenarioBufferData112_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData112_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 4, 1200, 32, 1, gScenarioBufferData113_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData113_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 4, 1312, 32, 1, gScenarioBufferData114_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData114_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 4, 1424, 32, 1, gScenarioBufferData115_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData115_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 4, 1536, 32, 1, gScenarioBufferData116_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData116_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 4, 1648, 32, 1, gScenarioBufferData117_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData117_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 4, 1760, 32, 1, gScenarioBufferData118_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData118_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 4, 1872, 32, 1, gScenarioBufferData119_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData119_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 4, 2096, 32, 1, gScenarioBufferData120_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData120_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 4, 2320, 32, 1, gScenarioBufferData121_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData121_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 4, 2544, 32, 1, gScenarioBufferData122_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData122_INTERP_BICUBIC_GRAYSCALE, 4},
{96, 4, 2768, 32, 1, gScenarioBufferData123_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData123_INTERP_BICUBIC_GRAYSCALE, 4},
{104, 4, 2992, 32, 1, gScenarioBufferData124_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData124_INTERP_BICUBIC_GRAYSCALE, 4},
{112, 4, 3216, 32, 1, gScenarioBufferData125_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData125_INTERP_BICUBIC_GRAYSCALE, 4},
{120, 4, 3440, 32, 1, gScenarioBufferData126_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData126_INTERP_BICUBIC_GRAYSCALE, 4},
{128, 4, 3664, 32, 1, gScenarioBufferData127_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData127_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 5, 164, 32, 1, gScenarioBufferData128_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData128_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 5, 232, 32, 1, gScenarioBufferData129_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData129_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 5, 300, 32, 1, gScenarioBufferData130_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData130_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 5, 368, 32, 1, gScenarioBufferData131_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData131_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 5, 436, 32, 1, gScenarioBufferData132_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData132_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 5, 504, 32, 1, gScenarioBufferData133_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData133_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 5, 572, 32, 1, gScenarioBufferData134_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData134_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 5, 640, 32, 1, gScenarioBufferData135_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData135_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 5, 708, 32, 1, gScenarioBufferData136_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData136_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 5, 776, 32, 1, gScenarioBufferData137_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData137_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 5, 844, 32, 1, gScenarioBufferData138_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData138_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 5, 912, 32, 1, gScenarioBufferData139_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData139_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 5, 980, 32, 1, gScenarioBufferData140_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData140_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 5, 1048, 32, 1, gScenarioBufferData141_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData141_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 5, 1116, 32, 1, gScenarioBufferData142_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData142_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 5, 1184, 32, 1, gScenarioBufferData143_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData143_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 5, 1320, 32, 1, gScenarioBufferData144_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData144_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 5, 1456, 32, 1, gScenarioBufferData145_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData145_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 5, 1592, 32, 1, gScenarioBufferData146_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData146_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 5, 1728, 32, 1, gScenarioBufferData147_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData147_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 5, 1864, 32, 1, gScenarioBufferData148_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData148_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 5, 2000, 32, 1, gScenarioBufferData149_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData149_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 5, 2136, 32, 1, gScenarioBufferData150_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData150_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 5, 2272, 32, 1, gScenarioBufferData151_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData151_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 5, 2544, 32, 1, gScenarioBufferData152_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData152_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 5, 2816, 32, 1, gScenarioBufferData153_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData153_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 5, 3088, 32, 1, gScenarioBufferData154_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData154_INTERP_BICUBIC_GRAYSCALE, 4},
{96, 5, 3360, 32, 1, gScenarioBufferData155_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData155_INTERP_BICUBIC_GRAYSCALE, 4},
{104, 5, 3632, 32, 1, gScenarioBufferData156_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData156_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 6, 192, 32, 1, gScenarioBufferData157_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData157_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 6, 272, 32, 1, gScenarioBufferData158_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData158_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 6, 352, 32, 1, gScenarioBufferData159_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData159_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 6, 432, 32, 1, gScenarioBufferData160_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData160_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 6, 512, 32, 1, gScenarioBufferData161_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData161_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 6, 592, 32, 1, gScenarioBufferData162_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData162_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 6, 672, 32, 1, gScenarioBufferData163_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData163_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 6, 752, 32, 1, gScenarioBufferData164_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData164_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 6, 832, 32, 1, gScenarioBufferData165_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData165_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 6, 912, 32, 1, gScenarioBufferData166_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData166_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 6, 992, 32, 1, gScenarioBufferData167_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData167_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 6, 1072, 32, 1, gScenarioBufferData168_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData168_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 6, 1152, 32, 1, gScenarioBufferData169_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData169_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 6, 1232, 32, 1, gScenarioBufferData170_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData170_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 6, 1312, 32, 1, gScenarioBufferData171_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData171_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 6, 1392, 32, 1, gScenarioBufferData172_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData172_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 6, 1552, 32, 1, gScenarioBufferData173_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData173_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 6, 1712, 32, 1, gScenarioBufferData174_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData174_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 6, 1872, 32, 1, gScenarioBufferData175_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData175_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 6, 2032, 32, 1, gScenarioBufferData176_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData176_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 6, 2192, 32, 1, gScenarioBufferData177_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData177_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 6, 2352, 32, 1, gScenarioBufferData178_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData178_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 6, 2512, 32, 1, gScenarioBufferData179_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData179_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 6, 2672, 32, 1, gScenarioBufferData180_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData180_INTERP_BICUBIC_GRAYSCALE, 4},
{72, 6, 2992, 32, 1, gScenarioBufferData181_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData181_INTERP_BICUBIC_GRAYSCALE, 4},
{80, 6, 3312, 32, 1, gScenarioBufferData182_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData182_INTERP_BICUBIC_GRAYSCALE, 4},
{88, 6, 3632, 32, 1, gScenarioBufferData183_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData183_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 8, 248, 32, 1, gScenarioBufferData184_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData184_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 8, 352, 32, 1, gScenarioBufferData185_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData185_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 8, 456, 32, 1, gScenarioBufferData186_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData186_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 8, 560, 32, 1, gScenarioBufferData187_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData187_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 8, 664, 32, 1, gScenarioBufferData188_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData188_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 8, 768, 32, 1, gScenarioBufferData189_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData189_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 8, 872, 32, 1, gScenarioBufferData190_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData190_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 8, 976, 32, 1, gScenarioBufferData191_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData191_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 8, 1080, 32, 1, gScenarioBufferData192_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData192_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 8, 1184, 32, 1, gScenarioBufferData193_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData193_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 8, 1288, 32, 1, gScenarioBufferData194_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData194_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 8, 1392, 32, 1, gScenarioBufferData195_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData195_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 8, 1496, 32, 1, gScenarioBufferData196_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData196_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 8, 1600, 32, 1, gScenarioBufferData197_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData197_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 8, 1704, 32, 1, gScenarioBufferData198_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData198_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 8, 1808, 32, 1, gScenarioBufferData199_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData199_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 8, 2016, 32, 1, gScenarioBufferData200_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData200_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 8, 2224, 32, 1, gScenarioBufferData201_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData201_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 8, 2432, 32, 1, gScenarioBufferData202_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData202_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 8, 2640, 32, 1, gScenarioBufferData203_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData203_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 8, 2848, 32, 1, gScenarioBufferData204_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData204_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 8, 3056, 32, 1, gScenarioBufferData205_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData205_INTERP_BICUBIC_GRAYSCALE, 4},
{60, 8, 3264, 32, 1, gScenarioBufferData206_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData206_INTERP_BICUBIC_GRAYSCALE, 4},
{64, 8, 3472, 32, 1, gScenarioBufferData207_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData207_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 10, 304, 32, 1, gScenarioBufferData208_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData208_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 10, 432, 32, 1, gScenarioBufferData209_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData209_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 10, 560, 32, 1, gScenarioBufferData210_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData210_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 10, 688, 32, 1, gScenarioBufferData211_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData211_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 10, 816, 32, 1, gScenarioBufferData212_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData212_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 10, 944, 32, 1, gScenarioBufferData213_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData213_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 10, 1072, 32, 1, gScenarioBufferData214_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData214_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 10, 1200, 32, 1, gScenarioBufferData215_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData215_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 10, 1328, 32, 1, gScenarioBufferData216_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData216_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 10, 1456, 32, 1, gScenarioBufferData217_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData217_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 10, 1584, 32, 1, gScenarioBufferData218_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData218_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 10, 1712, 32, 1, gScenarioBufferData219_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData219_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 10, 1840, 32, 1, gScenarioBufferData220_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData220_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 10, 1968, 32, 1, gScenarioBufferData221_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData221_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 10, 2096, 32, 1, gScenarioBufferData222_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData222_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 10, 2224, 32, 1, gScenarioBufferData223_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData223_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 10, 2480, 32, 1, gScenarioBufferData224_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData224_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 10, 2736, 32, 1, gScenarioBufferData225_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData225_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 10, 2992, 32, 1, gScenarioBufferData226_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData226_INTERP_BICUBIC_GRAYSCALE, 4},
{48, 10, 3248, 32, 1, gScenarioBufferData227_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData227_INTERP_BICUBIC_GRAYSCALE, 4},
{52, 10, 3504, 32, 1, gScenarioBufferData228_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData228_INTERP_BICUBIC_GRAYSCALE, 4},
{56, 10, 3760, 32, 1, gScenarioBufferData229_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData229_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 12, 360, 32, 1, gScenarioBufferData230_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData230_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 12, 512, 32, 1, gScenarioBufferData231_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData231_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 12, 664, 32, 1, gScenarioBufferData232_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData232_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 12, 816, 32, 1, gScenarioBufferData233_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData233_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 12, 968, 32, 1, gScenarioBufferData234_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData234_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 12, 1120, 32, 1, gScenarioBufferData235_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData235_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 12, 1272, 32, 1, gScenarioBufferData236_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData236_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 12, 1424, 32, 1, gScenarioBufferData237_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData237_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 12, 1576, 32, 1, gScenarioBufferData238_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData238_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 12, 1728, 32, 1, gScenarioBufferData239_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData239_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 12, 1880, 32, 1, gScenarioBufferData240_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData240_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 12, 2032, 32, 1, gScenarioBufferData241_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData241_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 12, 2184, 32, 1, gScenarioBufferData242_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData242_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 12, 2336, 32, 1, gScenarioBufferData243_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData243_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 12, 2488, 32, 1, gScenarioBufferData244_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData244_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 12, 2640, 32, 1, gScenarioBufferData245_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData245_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 12, 2944, 32, 1, gScenarioBufferData246_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData246_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 12, 3248, 32, 1, gScenarioBufferData247_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData247_INTERP_BICUBIC_GRAYSCALE, 4},
{44, 12, 3552, 32, 1, gScenarioBufferData248_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData248_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 14, 416, 32, 1, gScenarioBufferData249_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData249_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 14, 592, 32, 1, gScenarioBufferData250_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData250_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 14, 768, 32, 1, gScenarioBufferData251_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData251_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 14, 944, 32, 1, gScenarioBufferData252_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData252_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 14, 1120, 32, 1, gScenarioBufferData253_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData253_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 14, 1296, 32, 1, gScenarioBufferData254_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData254_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 14, 1472, 32, 1, gScenarioBufferData255_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData255_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 14, 1648, 32, 1, gScenarioBufferData256_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData256_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 14, 1824, 32, 1, gScenarioBufferData257_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData257_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 14, 2000, 32, 1, gScenarioBufferData258_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData258_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 14, 2176, 32, 1, gScenarioBufferData259_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData259_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 14, 2352, 32, 1, gScenarioBufferData260_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData260_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 14, 2528, 32, 1, gScenarioBufferData261_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData261_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 14, 2704, 32, 1, gScenarioBufferData262_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData262_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 14, 2880, 32, 1, gScenarioBufferData263_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData263_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 14, 3056, 32, 1, gScenarioBufferData264_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData264_INTERP_BICUBIC_GRAYSCALE, 4},
{36, 14, 3408, 32, 1, gScenarioBufferData265_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData265_INTERP_BICUBIC_GRAYSCALE, 4},
{40, 14, 3760, 32, 1, gScenarioBufferData266_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData266_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 16, 472, 32, 1, gScenarioBufferData267_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData267_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 16, 672, 32, 1, gScenarioBufferData268_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData268_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 16, 872, 32, 1, gScenarioBufferData269_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData269_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 16, 1072, 32, 1, gScenarioBufferData270_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData270_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 16, 1272, 32, 1, gScenarioBufferData271_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData271_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 16, 1472, 32, 1, gScenarioBufferData272_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData272_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 16, 1672, 32, 1, gScenarioBufferData273_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData273_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 16, 1872, 32, 1, gScenarioBufferData274_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData274_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 16, 2072, 32, 1, gScenarioBufferData275_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData275_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 16, 2272, 32, 1, gScenarioBufferData276_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData276_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 16, 2472, 32, 1, gScenarioBufferData277_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData277_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 16, 2672, 32, 1, gScenarioBufferData278_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData278_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 16, 2872, 32, 1, gScenarioBufferData279_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData279_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 16, 3072, 32, 1, gScenarioBufferData280_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData280_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 16, 3272, 32, 1, gScenarioBufferData281_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData281_INTERP_BICUBIC_GRAYSCALE, 4},
{32, 16, 3472, 32, 1, gScenarioBufferData282_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData282_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 18, 528, 32, 1, gScenarioBufferData283_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData283_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 18, 752, 32, 1, gScenarioBufferData284_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData284_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 18, 976, 32, 1, gScenarioBufferData285_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData285_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 18, 1200, 32, 1, gScenarioBufferData286_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData286_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 18, 1424, 32, 1, gScenarioBufferData287_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData287_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 18, 1648, 32, 1, gScenarioBufferData288_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData288_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 18, 1872, 32, 1, gScenarioBufferData289_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData289_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 18, 2096, 32, 1, gScenarioBufferData290_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData290_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 18, 2320, 32, 1, gScenarioBufferData291_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData291_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 18, 2544, 32, 1, gScenarioBufferData292_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData292_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 18, 2768, 32, 1, gScenarioBufferData293_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData293_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 18, 2992, 32, 1, gScenarioBufferData294_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData294_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 18, 3216, 32, 1, gScenarioBufferData295_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData295_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 18, 3440, 32, 1, gScenarioBufferData296_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData296_INTERP_BICUBIC_GRAYSCALE, 4},
{30, 18, 3664, 32, 1, gScenarioBufferData297_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData297_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 20, 584, 32, 1, gScenarioBufferData298_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData298_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 20, 832, 32, 1, gScenarioBufferData299_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData299_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 20, 1080, 32, 1, gScenarioBufferData300_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData300_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 20, 1328, 32, 1, gScenarioBufferData301_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData301_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 20, 1576, 32, 1, gScenarioBufferData302_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData302_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 20, 1824, 32, 1, gScenarioBufferData303_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData303_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 20, 2072, 32, 1, gScenarioBufferData304_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData304_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 20, 2320, 32, 1, gScenarioBufferData305_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData305_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 20, 2568, 32, 1, gScenarioBufferData306_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData306_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 20, 2816, 32, 1, gScenarioBufferData307_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData307_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 20, 3064, 32, 1, gScenarioBufferData308_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData308_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 20, 3312, 32, 1, gScenarioBufferData309_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData309_INTERP_BICUBIC_GRAYSCALE, 4},
{26, 20, 3560, 32, 1, gScenarioBufferData310_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData310_INTERP_BICUBIC_GRAYSCALE, 4},
{28, 20, 3808, 32, 1, gScenarioBufferData311_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData311_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 22, 640, 32, 1, gScenarioBufferData312_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData312_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 22, 912, 32, 1, gScenarioBufferData313_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData313_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 22, 1184, 32, 1, gScenarioBufferData314_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData314_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 22, 1456, 32, 1, gScenarioBufferData315_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData315_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 22, 1728, 32, 1, gScenarioBufferData316_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData316_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 22, 2000, 32, 1, gScenarioBufferData317_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData317_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 22, 2272, 32, 1, gScenarioBufferData318_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData318_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 22, 2544, 32, 1, gScenarioBufferData319_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData319_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 22, 2816, 32, 1, gScenarioBufferData320_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData320_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 22, 3088, 32, 1, gScenarioBufferData321_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData321_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 22, 3360, 32, 1, gScenarioBufferData322_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData322_INTERP_BICUBIC_GRAYSCALE, 4},
{24, 22, 3632, 32, 1, gScenarioBufferData323_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData323_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 24, 696, 32, 1, gScenarioBufferData324_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData324_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 24, 992, 32, 1, gScenarioBufferData325_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData325_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 24, 1288, 32, 1, gScenarioBufferData326_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData326_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 24, 1584, 32, 1, gScenarioBufferData327_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData327_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 24, 1880, 32, 1, gScenarioBufferData328_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData328_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 24, 2176, 32, 1, gScenarioBufferData329_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData329_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 24, 2472, 32, 1, gScenarioBufferData330_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData330_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 24, 2768, 32, 1, gScenarioBufferData331_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData331_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 24, 3064, 32, 1, gScenarioBufferData332_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData332_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 24, 3360, 32, 1, gScenarioBufferData333_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData333_INTERP_BICUBIC_GRAYSCALE, 4},
{22, 24, 3656, 32, 1, gScenarioBufferData334_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData334_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 26, 752, 32, 1, gScenarioBufferData335_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData335_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 26, 1072, 32, 1, gScenarioBufferData336_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData336_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 26, 1392, 32, 1, gScenarioBufferData337_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData337_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 26, 1712, 32, 1, gScenarioBufferData338_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData338_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 26, 2032, 32, 1, gScenarioBufferData339_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData339_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 26, 2352, 32, 1, gScenarioBufferData340_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData340_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 26, 2672, 32, 1, gScenarioBufferData341_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData341_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 26, 2992, 32, 1, gScenarioBufferData342_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData342_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 26, 3312, 32, 1, gScenarioBufferData343_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData343_INTERP_BICUBIC_GRAYSCALE, 4},
{20, 26, 3632, 32, 1, gScenarioBufferData344_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData344_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 28, 808, 32, 1, gScenarioBufferData345_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData345_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 28, 1152, 32, 1, gScenarioBufferData346_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData346_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 28, 1496, 32, 1, gScenarioBufferData347_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData347_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 28, 1840, 32, 1, gScenarioBufferData348_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData348_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 28, 2184, 32, 1, gScenarioBufferData349_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData349_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 28, 2528, 32, 1, gScenarioBufferData350_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData350_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 28, 2872, 32, 1, gScenarioBufferData351_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData351_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 28, 3216, 32, 1, gScenarioBufferData352_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData352_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 28, 3560, 32, 1, gScenarioBufferData353_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData353_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 30, 864, 32, 1, gScenarioBufferData354_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData354_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 30, 1232, 32, 1, gScenarioBufferData355_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData355_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 30, 1600, 32, 1, gScenarioBufferData356_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData356_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 30, 1968, 32, 1, gScenarioBufferData357_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData357_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 30, 2336, 32, 1, gScenarioBufferData358_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData358_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 30, 2704, 32, 1, gScenarioBufferData359_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData359_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 30, 3072, 32, 1, gScenarioBufferData360_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData360_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 30, 3440, 32, 1, gScenarioBufferData361_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData361_INTERP_BICUBIC_GRAYSCALE, 4},
{18, 30, 3808, 32, 1, gScenarioBufferData362_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData362_INTERP_BICUBIC_GRAYSCALE, 4},
{2, 32, 920, 32, 1, gScenarioBufferData363_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData363_INTERP_BICUBIC_GRAYSCALE, 4},
{4, 32, 1312, 32, 1, gScenarioBufferData364_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData364_INTERP_BICUBIC_GRAYSCALE, 4},
{6, 32, 1704, 32, 1, gScenarioBufferData365_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData365_INTERP_BICUBIC_GRAYSCALE, 4},
{8, 32, 2096, 32, 1, gScenarioBufferData366_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData366_INTERP_BICUBIC_GRAYSCALE, 4},
{10, 32, 2488, 32, 1, gScenarioBufferData367_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData367_INTERP_BICUBIC_GRAYSCALE, 4},
{12, 32, 2880, 32, 1, gScenarioBufferData368_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData368_INTERP_BICUBIC_GRAYSCALE, 4},
{14, 32, 3272, 32, 1, gScenarioBufferData369_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData369_INTERP_BICUBIC_GRAYSCALE, 4},
{16, 32, 3664, 32, 1, gScenarioBufferData370_INTERP_BICUBIC_GRAYSCALE, 64, gScenarioKernelData370_INTERP_BICUBIC_GRAYSCALE, 4}
};

static acf_scenario_list gScenarioList_INTERP_BICUBIC_GRAYSCALE = {
371, //number of scenarios
gScenarioArray_INTERP_BICUBIC_GRAYSCALE};
//**************************************************************

class INTERP_BICUBIC_GRAYSCALE : public ACF_Process_APU
{

public:
   INTERP_BICUBIC_GRAYSCALE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INTERP_BICUBIC_GRAYSCALE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INTERP_BICUBIC_GRAYSCALE",
                                        INTERP_BICUBIC_GRAYSCALE_LOAD_SEGMENTS,
                                        INTERP_BICUBIC_GRAYSCALE_LOAD_PMEM, INTERP_BICUBIC_GRAYSCALE_LOAD_PMEM_SIZE,
                                        INTERP_BICUBIC_GRAYSCALE_LOAD_DMEM, INTERP_BICUBIC_GRAYSCALE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INTERP_BICUBIC_GRAYSCALE_APEX_LOG_BUFFER, INTERP_BICUBIC_GRAYSCALE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("Y_OFFSET", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("X_OFFSET", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INTERP_BICUBIC_GRAYSCALE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INTERP_BICUBIC_GRAYSCALE

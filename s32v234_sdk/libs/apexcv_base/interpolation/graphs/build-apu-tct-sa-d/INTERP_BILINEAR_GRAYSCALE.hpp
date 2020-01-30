#ifndef _ACF_PROCESS_APU_INTERP_BILINEAR_GRAYSCALE
#define _ACF_PROCESS_APU_INTERP_BILINEAR_GRAYSCALE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INTERP_BILINEAR_GRAYSCALE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INTERP_BILINEAR_GRAYSCALE[] = {{2, 1, 3, 0, {0, 1, 2, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INTERP_BILINEAR_GRAYSCALE[] = {{4, 1, 3, 0, {0, 1, 2, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INTERP_BILINEAR_GRAYSCALE[] = {{6, 1, 3, 0, {0, 1, 2, 0}}, {6, 1, 2, 1, {0, 0, 0, 0}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INTERP_BILINEAR_GRAYSCALE[] = {{8, 1, 3, 0, {0, 1, 2, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INTERP_BILINEAR_GRAYSCALE[] = {{10, 1, 3, 0, {0, 1, 2, 0}}, {10, 1, 2, 1, {0, 0, 0, 0}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INTERP_BILINEAR_GRAYSCALE[] = {{12, 1, 3, 0, {0, 1, 2, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INTERP_BILINEAR_GRAYSCALE[] = {{14, 1, 3, 0, {0, 1, 2, 0}}, {14, 1, 2, 1, {0, 0, 0, 0}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INTERP_BILINEAR_GRAYSCALE[] = {{16, 1, 3, 0, {0, 1, 2, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INTERP_BILINEAR_GRAYSCALE[] = {{18, 1, 3, 0, {0, 1, 2, 0}}, {18, 1, 2, 1, {0, 0, 0, 0}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INTERP_BILINEAR_GRAYSCALE[] = {{20, 1, 3, 0, {0, 1, 2, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INTERP_BILINEAR_GRAYSCALE[] = {{22, 1, 3, 0, {0, 1, 2, 0}}, {22, 1, 2, 1, {0, 0, 0, 0}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INTERP_BILINEAR_GRAYSCALE[] = {{24, 1, 3, 0, {0, 1, 2, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INTERP_BILINEAR_GRAYSCALE[] = {{26, 1, 3, 0, {0, 1, 2, 0}}, {26, 1, 2, 1, {0, 0, 0, 0}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INTERP_BILINEAR_GRAYSCALE[] = {{28, 1, 3, 0, {0, 1, 2, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INTERP_BILINEAR_GRAYSCALE[] = {{30, 1, 3, 0, {0, 1, 2, 0}}, {30, 1, 2, 1, {0, 0, 0, 0}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INTERP_BILINEAR_GRAYSCALE[] = {{32, 1, 3, 0, {0, 1, 2, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INTERP_BILINEAR_GRAYSCALE[] = {{36, 1, 3, 0, {0, 1, 2, 0}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INTERP_BILINEAR_GRAYSCALE[] = {{40, 1, 3, 0, {0, 1, 2, 0}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INTERP_BILINEAR_GRAYSCALE[] = {{44, 1, 3, 0, {0, 1, 2, 0}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INTERP_BILINEAR_GRAYSCALE[] = {{48, 1, 3, 0, {0, 1, 2, 0}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INTERP_BILINEAR_GRAYSCALE[] = {{52, 1, 3, 0, {0, 1, 2, 0}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INTERP_BILINEAR_GRAYSCALE[] = {{56, 1, 3, 0, {0, 1, 2, 0}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INTERP_BILINEAR_GRAYSCALE[] = {{60, 1, 3, 0, {0, 1, 2, 0}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INTERP_BILINEAR_GRAYSCALE[] = {{64, 1, 3, 0, {0, 1, 2, 0}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INTERP_BILINEAR_GRAYSCALE[] = {{2, 2, 3, 0, {0, 1, 2, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INTERP_BILINEAR_GRAYSCALE[] = {{4, 2, 3, 0, {0, 1, 2, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INTERP_BILINEAR_GRAYSCALE[] = {{6, 2, 3, 0, {0, 1, 2, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INTERP_BILINEAR_GRAYSCALE[] = {{8, 2, 3, 0, {0, 1, 2, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INTERP_BILINEAR_GRAYSCALE[] = {{10, 2, 3, 0, {0, 1, 2, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INTERP_BILINEAR_GRAYSCALE[] = {{12, 2, 3, 0, {0, 1, 2, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INTERP_BILINEAR_GRAYSCALE[] = {{14, 2, 3, 0, {0, 1, 2, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INTERP_BILINEAR_GRAYSCALE[] = {{16, 2, 3, 0, {0, 1, 2, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INTERP_BILINEAR_GRAYSCALE[] = {{18, 2, 3, 0, {0, 1, 2, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INTERP_BILINEAR_GRAYSCALE[] = {{20, 2, 3, 0, {0, 1, 2, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INTERP_BILINEAR_GRAYSCALE[] = {{22, 2, 3, 0, {0, 1, 2, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INTERP_BILINEAR_GRAYSCALE[] = {{24, 2, 3, 0, {0, 1, 2, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INTERP_BILINEAR_GRAYSCALE[] = {{26, 2, 3, 0, {0, 1, 2, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INTERP_BILINEAR_GRAYSCALE[] = {{28, 2, 3, 0, {0, 1, 2, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INTERP_BILINEAR_GRAYSCALE[] = {{30, 2, 3, 0, {0, 1, 2, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}, {30, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INTERP_BILINEAR_GRAYSCALE[] = {{32, 2, 3, 0, {0, 1, 2, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INTERP_BILINEAR_GRAYSCALE[] = {{36, 2, 3, 0, {0, 1, 2, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INTERP_BILINEAR_GRAYSCALE[] = {{40, 2, 3, 0, {0, 1, 2, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INTERP_BILINEAR_GRAYSCALE[] = {{44, 2, 3, 0, {0, 1, 2, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INTERP_BILINEAR_GRAYSCALE[] = {{48, 2, 3, 0, {0, 1, 2, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INTERP_BILINEAR_GRAYSCALE[] = {{52, 2, 3, 0, {0, 1, 2, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_INTERP_BILINEAR_GRAYSCALE[] = {{56, 2, 3, 0, {0, 1, 2, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_INTERP_BILINEAR_GRAYSCALE[] = {{60, 2, 3, 0, {0, 1, 2, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_INTERP_BILINEAR_GRAYSCALE[] = {{64, 2, 3, 0, {0, 1, 2, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_INTERP_BILINEAR_GRAYSCALE[] = {{2, 3, 3, 0, {0, 1, 2, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_INTERP_BILINEAR_GRAYSCALE[] = {{4, 3, 3, 0, {0, 1, 2, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_INTERP_BILINEAR_GRAYSCALE[] = {{6, 3, 3, 0, {0, 1, 2, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_INTERP_BILINEAR_GRAYSCALE[] = {{8, 3, 3, 0, {0, 1, 2, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_INTERP_BILINEAR_GRAYSCALE[] = {{10, 3, 3, 0, {0, 1, 2, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_INTERP_BILINEAR_GRAYSCALE[] = {{12, 3, 3, 0, {0, 1, 2, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_INTERP_BILINEAR_GRAYSCALE[] = {{14, 3, 3, 0, {0, 1, 2, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_INTERP_BILINEAR_GRAYSCALE[] = {{16, 3, 3, 0, {0, 1, 2, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_INTERP_BILINEAR_GRAYSCALE[] = {{18, 3, 3, 0, {0, 1, 2, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_INTERP_BILINEAR_GRAYSCALE[] = {{20, 3, 3, 0, {0, 1, 2, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_INTERP_BILINEAR_GRAYSCALE[] = {{22, 3, 3, 0, {0, 1, 2, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}, {22, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_INTERP_BILINEAR_GRAYSCALE[] = {{24, 3, 3, 0, {0, 1, 2, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_INTERP_BILINEAR_GRAYSCALE[] = {{26, 3, 3, 0, {0, 1, 2, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}, {26, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_INTERP_BILINEAR_GRAYSCALE[] = {{28, 3, 3, 0, {0, 1, 2, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_INTERP_BILINEAR_GRAYSCALE[] = {{30, 3, 3, 0, {0, 1, 2, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}, {30, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_INTERP_BILINEAR_GRAYSCALE[] = {{32, 3, 3, 0, {0, 1, 2, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_INTERP_BILINEAR_GRAYSCALE[] = {{36, 3, 3, 0, {0, 1, 2, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_INTERP_BILINEAR_GRAYSCALE[] = {{40, 3, 3, 0, {0, 1, 2, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_INTERP_BILINEAR_GRAYSCALE[] = {{44, 3, 3, 0, {0, 1, 2, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_INTERP_BILINEAR_GRAYSCALE[] = {{48, 3, 3, 0, {0, 1, 2, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_INTERP_BILINEAR_GRAYSCALE[] = {{52, 3, 3, 0, {0, 1, 2, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_INTERP_BILINEAR_GRAYSCALE[] = {{56, 3, 3, 0, {0, 1, 2, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_INTERP_BILINEAR_GRAYSCALE[] = {{60, 3, 3, 0, {0, 1, 2, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_INTERP_BILINEAR_GRAYSCALE[] = {{64, 3, 3, 0, {0, 1, 2, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_INTERP_BILINEAR_GRAYSCALE[] = {{2, 4, 3, 0, {0, 1, 2, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_INTERP_BILINEAR_GRAYSCALE[] = {{4, 4, 3, 0, {0, 1, 2, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_INTERP_BILINEAR_GRAYSCALE[] = {{6, 4, 3, 0, {0, 1, 2, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_INTERP_BILINEAR_GRAYSCALE[] = {{8, 4, 3, 0, {0, 1, 2, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_INTERP_BILINEAR_GRAYSCALE[] = {{10, 4, 3, 0, {0, 1, 2, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_INTERP_BILINEAR_GRAYSCALE[] = {{12, 4, 3, 0, {0, 1, 2, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_INTERP_BILINEAR_GRAYSCALE[] = {{14, 4, 3, 0, {0, 1, 2, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_INTERP_BILINEAR_GRAYSCALE[] = {{16, 4, 3, 0, {0, 1, 2, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_INTERP_BILINEAR_GRAYSCALE[] = {{18, 4, 3, 0, {0, 1, 2, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}, {18, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_INTERP_BILINEAR_GRAYSCALE[] = {{20, 4, 3, 0, {0, 1, 2, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_INTERP_BILINEAR_GRAYSCALE[] = {{22, 4, 3, 0, {0, 1, 2, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}, {22, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_INTERP_BILINEAR_GRAYSCALE[] = {{24, 4, 3, 0, {0, 1, 2, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_INTERP_BILINEAR_GRAYSCALE[] = {{26, 4, 3, 0, {0, 1, 2, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}, {26, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_INTERP_BILINEAR_GRAYSCALE[] = {{28, 4, 3, 0, {0, 1, 2, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_INTERP_BILINEAR_GRAYSCALE[] = {{30, 4, 3, 0, {0, 1, 2, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}, {30, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_INTERP_BILINEAR_GRAYSCALE[] = {{32, 4, 3, 0, {0, 1, 2, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_INTERP_BILINEAR_GRAYSCALE[] = {{36, 4, 3, 0, {0, 1, 2, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_INTERP_BILINEAR_GRAYSCALE[] = {{40, 4, 3, 0, {0, 1, 2, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_INTERP_BILINEAR_GRAYSCALE[] = {{44, 4, 3, 0, {0, 1, 2, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_INTERP_BILINEAR_GRAYSCALE[] = {{48, 4, 3, 0, {0, 1, 2, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_INTERP_BILINEAR_GRAYSCALE[] = {{52, 4, 3, 0, {0, 1, 2, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_INTERP_BILINEAR_GRAYSCALE[] = {{56, 4, 3, 0, {0, 1, 2, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_INTERP_BILINEAR_GRAYSCALE[] = {{60, 4, 3, 0, {0, 1, 2, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_INTERP_BILINEAR_GRAYSCALE[] = {{64, 4, 3, 0, {0, 1, 2, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_INTERP_BILINEAR_GRAYSCALE[] = {{2, 5, 3, 0, {0, 1, 2, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_INTERP_BILINEAR_GRAYSCALE[] = {{4, 5, 3, 0, {0, 1, 2, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_INTERP_BILINEAR_GRAYSCALE[] = {{6, 5, 3, 0, {0, 1, 2, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_INTERP_BILINEAR_GRAYSCALE[] = {{8, 5, 3, 0, {0, 1, 2, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_INTERP_BILINEAR_GRAYSCALE[] = {{10, 5, 3, 0, {0, 1, 2, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_INTERP_BILINEAR_GRAYSCALE[] = {{12, 5, 3, 0, {0, 1, 2, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_INTERP_BILINEAR_GRAYSCALE[] = {{14, 5, 3, 0, {0, 1, 2, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}, {14, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_INTERP_BILINEAR_GRAYSCALE[] = {{16, 5, 3, 0, {0, 1, 2, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_INTERP_BILINEAR_GRAYSCALE[] = {{18, 5, 3, 0, {0, 1, 2, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}, {18, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_INTERP_BILINEAR_GRAYSCALE[] = {{20, 5, 3, 0, {0, 1, 2, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_INTERP_BILINEAR_GRAYSCALE[] = {{22, 5, 3, 0, {0, 1, 2, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}, {22, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_INTERP_BILINEAR_GRAYSCALE[] = {{24, 5, 3, 0, {0, 1, 2, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_INTERP_BILINEAR_GRAYSCALE[] = {{26, 5, 3, 0, {0, 1, 2, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}, {26, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_INTERP_BILINEAR_GRAYSCALE[] = {{28, 5, 3, 0, {0, 1, 2, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_INTERP_BILINEAR_GRAYSCALE[] = {{30, 5, 3, 0, {0, 1, 2, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}, {30, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_INTERP_BILINEAR_GRAYSCALE[] = {{32, 5, 3, 0, {0, 1, 2, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_INTERP_BILINEAR_GRAYSCALE[] = {{36, 5, 3, 0, {0, 1, 2, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_INTERP_BILINEAR_GRAYSCALE[] = {{40, 5, 3, 0, {0, 1, 2, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_INTERP_BILINEAR_GRAYSCALE[] = {{44, 5, 3, 0, {0, 1, 2, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_INTERP_BILINEAR_GRAYSCALE[] = {{48, 5, 3, 0, {0, 1, 2, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_INTERP_BILINEAR_GRAYSCALE[] = {{52, 5, 3, 0, {0, 1, 2, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_INTERP_BILINEAR_GRAYSCALE[] = {{56, 5, 3, 0, {0, 1, 2, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_INTERP_BILINEAR_GRAYSCALE[] = {{60, 5, 3, 0, {0, 1, 2, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_INTERP_BILINEAR_GRAYSCALE[] = {{64, 5, 3, 0, {0, 1, 2, 0}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_INTERP_BILINEAR_GRAYSCALE[] = {{2, 6, 3, 0, {0, 1, 2, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_INTERP_BILINEAR_GRAYSCALE[] = {{4, 6, 3, 0, {0, 1, 2, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_INTERP_BILINEAR_GRAYSCALE[] = {{6, 6, 3, 0, {0, 1, 2, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_INTERP_BILINEAR_GRAYSCALE[] = {{8, 6, 3, 0, {0, 1, 2, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_INTERP_BILINEAR_GRAYSCALE[] = {{10, 6, 3, 0, {0, 1, 2, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}, {10, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_INTERP_BILINEAR_GRAYSCALE[] = {{12, 6, 3, 0, {0, 1, 2, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_INTERP_BILINEAR_GRAYSCALE[] = {{14, 6, 3, 0, {0, 1, 2, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}, {14, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_INTERP_BILINEAR_GRAYSCALE[] = {{16, 6, 3, 0, {0, 1, 2, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_INTERP_BILINEAR_GRAYSCALE[] = {{18, 6, 3, 0, {0, 1, 2, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}, {18, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_INTERP_BILINEAR_GRAYSCALE[] = {{20, 6, 3, 0, {0, 1, 2, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_INTERP_BILINEAR_GRAYSCALE[] = {{22, 6, 3, 0, {0, 1, 2, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}, {22, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_INTERP_BILINEAR_GRAYSCALE[] = {{24, 6, 3, 0, {0, 1, 2, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_INTERP_BILINEAR_GRAYSCALE[] = {{26, 6, 3, 0, {0, 1, 2, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}, {26, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_INTERP_BILINEAR_GRAYSCALE[] = {{28, 6, 3, 0, {0, 1, 2, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_INTERP_BILINEAR_GRAYSCALE[] = {{30, 6, 3, 0, {0, 1, 2, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}, {30, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_INTERP_BILINEAR_GRAYSCALE[] = {{32, 6, 3, 0, {0, 1, 2, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_INTERP_BILINEAR_GRAYSCALE[] = {{36, 6, 3, 0, {0, 1, 2, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_INTERP_BILINEAR_GRAYSCALE[] = {{40, 6, 3, 0, {0, 1, 2, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_INTERP_BILINEAR_GRAYSCALE[] = {{44, 6, 3, 0, {0, 1, 2, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_INTERP_BILINEAR_GRAYSCALE[] = {{48, 6, 3, 0, {0, 1, 2, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_INTERP_BILINEAR_GRAYSCALE[] = {{52, 6, 3, 0, {0, 1, 2, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_INTERP_BILINEAR_GRAYSCALE[] = {{56, 6, 3, 0, {0, 1, 2, 0}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_INTERP_BILINEAR_GRAYSCALE[] = {{60, 6, 3, 0, {0, 1, 2, 0}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_INTERP_BILINEAR_GRAYSCALE[] = {{64, 6, 3, 0, {0, 1, 2, 0}}, {64, 6, 2, 1, {0, 0, 0, 0}}, {64, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_INTERP_BILINEAR_GRAYSCALE[] = {{2, 8, 3, 0, {0, 1, 2, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_INTERP_BILINEAR_GRAYSCALE[] = {{4, 8, 3, 0, {0, 1, 2, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_INTERP_BILINEAR_GRAYSCALE[] = {{6, 8, 3, 0, {0, 1, 2, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_INTERP_BILINEAR_GRAYSCALE[] = {{8, 8, 3, 0, {0, 1, 2, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_INTERP_BILINEAR_GRAYSCALE[] = {{10, 8, 3, 0, {0, 1, 2, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}, {10, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_INTERP_BILINEAR_GRAYSCALE[] = {{12, 8, 3, 0, {0, 1, 2, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_INTERP_BILINEAR_GRAYSCALE[] = {{14, 8, 3, 0, {0, 1, 2, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}, {14, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_INTERP_BILINEAR_GRAYSCALE[] = {{16, 8, 3, 0, {0, 1, 2, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_INTERP_BILINEAR_GRAYSCALE[] = {{18, 8, 3, 0, {0, 1, 2, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}, {18, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_INTERP_BILINEAR_GRAYSCALE[] = {{20, 8, 3, 0, {0, 1, 2, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_INTERP_BILINEAR_GRAYSCALE[] = {{22, 8, 3, 0, {0, 1, 2, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}, {22, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_INTERP_BILINEAR_GRAYSCALE[] = {{24, 8, 3, 0, {0, 1, 2, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_INTERP_BILINEAR_GRAYSCALE[] = {{26, 8, 3, 0, {0, 1, 2, 0}}, {26, 8, 2, 1, {0, 0, 0, 0}}, {26, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_INTERP_BILINEAR_GRAYSCALE[] = {{28, 8, 3, 0, {0, 1, 2, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_INTERP_BILINEAR_GRAYSCALE[] = {{30, 8, 3, 0, {0, 1, 2, 0}}, {30, 8, 2, 1, {0, 0, 0, 0}}, {30, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_INTERP_BILINEAR_GRAYSCALE[] = {{32, 8, 3, 0, {0, 1, 2, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_INTERP_BILINEAR_GRAYSCALE[] = {{36, 8, 3, 0, {0, 1, 2, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_INTERP_BILINEAR_GRAYSCALE[] = {{40, 8, 3, 0, {0, 1, 2, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_INTERP_BILINEAR_GRAYSCALE[] = {{44, 8, 3, 0, {0, 1, 2, 0}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_INTERP_BILINEAR_GRAYSCALE[] = {{48, 8, 3, 0, {0, 1, 2, 0}}, {48, 8, 2, 1, {0, 0, 0, 0}}, {48, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_INTERP_BILINEAR_GRAYSCALE[] = {{2, 10, 3, 0, {0, 1, 2, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_INTERP_BILINEAR_GRAYSCALE[] = {{4, 10, 3, 0, {0, 1, 2, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_INTERP_BILINEAR_GRAYSCALE[] = {{6, 10, 3, 0, {0, 1, 2, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}, {6, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_INTERP_BILINEAR_GRAYSCALE[] = {{8, 10, 3, 0, {0, 1, 2, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_INTERP_BILINEAR_GRAYSCALE[] = {{10, 10, 3, 0, {0, 1, 2, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}, {10, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_INTERP_BILINEAR_GRAYSCALE[] = {{12, 10, 3, 0, {0, 1, 2, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_INTERP_BILINEAR_GRAYSCALE[] = {{14, 10, 3, 0, {0, 1, 2, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}, {14, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_INTERP_BILINEAR_GRAYSCALE[] = {{16, 10, 3, 0, {0, 1, 2, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_INTERP_BILINEAR_GRAYSCALE[] = {{18, 10, 3, 0, {0, 1, 2, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}, {18, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_INTERP_BILINEAR_GRAYSCALE[] = {{20, 10, 3, 0, {0, 1, 2, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_INTERP_BILINEAR_GRAYSCALE[] = {{22, 10, 3, 0, {0, 1, 2, 0}}, {22, 10, 2, 1, {0, 0, 0, 0}}, {22, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_INTERP_BILINEAR_GRAYSCALE[] = {{24, 10, 3, 0, {0, 1, 2, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_INTERP_BILINEAR_GRAYSCALE[] = {{26, 10, 3, 0, {0, 1, 2, 0}}, {26, 10, 2, 1, {0, 0, 0, 0}}, {26, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_INTERP_BILINEAR_GRAYSCALE[] = {{28, 10, 3, 0, {0, 1, 2, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_INTERP_BILINEAR_GRAYSCALE[] = {{30, 10, 3, 0, {0, 1, 2, 0}}, {30, 10, 2, 1, {0, 0, 0, 0}}, {30, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_INTERP_BILINEAR_GRAYSCALE[] = {{32, 10, 3, 0, {0, 1, 2, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_INTERP_BILINEAR_GRAYSCALE[] = {{36, 10, 3, 0, {0, 1, 2, 0}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_INTERP_BILINEAR_GRAYSCALE[] = {{40, 10, 3, 0, {0, 1, 2, 0}}, {40, 10, 2, 1, {0, 0, 0, 0}}, {40, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_INTERP_BILINEAR_GRAYSCALE[] = {{2, 12, 3, 0, {0, 1, 2, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_INTERP_BILINEAR_GRAYSCALE[] = {{4, 12, 3, 0, {0, 1, 2, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_INTERP_BILINEAR_GRAYSCALE[] = {{6, 12, 3, 0, {0, 1, 2, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}, {6, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_INTERP_BILINEAR_GRAYSCALE[] = {{8, 12, 3, 0, {0, 1, 2, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_INTERP_BILINEAR_GRAYSCALE[] = {{10, 12, 3, 0, {0, 1, 2, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}, {10, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_INTERP_BILINEAR_GRAYSCALE[] = {{12, 12, 3, 0, {0, 1, 2, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_INTERP_BILINEAR_GRAYSCALE[] = {{14, 12, 3, 0, {0, 1, 2, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}, {14, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_INTERP_BILINEAR_GRAYSCALE[] = {{16, 12, 3, 0, {0, 1, 2, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_INTERP_BILINEAR_GRAYSCALE[] = {{18, 12, 3, 0, {0, 1, 2, 0}}, {18, 12, 2, 1, {0, 0, 0, 0}}, {18, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_INTERP_BILINEAR_GRAYSCALE[] = {{20, 12, 3, 0, {0, 1, 2, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_INTERP_BILINEAR_GRAYSCALE[] = {{22, 12, 3, 0, {0, 1, 2, 0}}, {22, 12, 2, 1, {0, 0, 0, 0}}, {22, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_INTERP_BILINEAR_GRAYSCALE[] = {{24, 12, 3, 0, {0, 1, 2, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_INTERP_BILINEAR_GRAYSCALE[] = {{26, 12, 3, 0, {0, 1, 2, 0}}, {26, 12, 2, 1, {0, 0, 0, 0}}, {26, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_INTERP_BILINEAR_GRAYSCALE[] = {{28, 12, 3, 0, {0, 1, 2, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_INTERP_BILINEAR_GRAYSCALE[] = {{30, 12, 3, 0, {0, 1, 2, 0}}, {30, 12, 2, 1, {0, 0, 0, 0}}, {30, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_INTERP_BILINEAR_GRAYSCALE[] = {{32, 12, 3, 0, {0, 1, 2, 0}}, {32, 12, 2, 1, {0, 0, 0, 0}}, {32, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_INTERP_BILINEAR_GRAYSCALE[] = {{2, 14, 3, 0, {0, 1, 2, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_INTERP_BILINEAR_GRAYSCALE[] = {{4, 14, 3, 0, {0, 1, 2, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_INTERP_BILINEAR_GRAYSCALE[] = {{6, 14, 3, 0, {0, 1, 2, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}, {6, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_INTERP_BILINEAR_GRAYSCALE[] = {{8, 14, 3, 0, {0, 1, 2, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_INTERP_BILINEAR_GRAYSCALE[] = {{10, 14, 3, 0, {0, 1, 2, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}, {10, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_INTERP_BILINEAR_GRAYSCALE[] = {{12, 14, 3, 0, {0, 1, 2, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_INTERP_BILINEAR_GRAYSCALE[] = {{14, 14, 3, 0, {0, 1, 2, 0}}, {14, 14, 2, 1, {0, 0, 0, 0}}, {14, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_INTERP_BILINEAR_GRAYSCALE[] = {{16, 14, 3, 0, {0, 1, 2, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_INTERP_BILINEAR_GRAYSCALE[] = {{18, 14, 3, 0, {0, 1, 2, 0}}, {18, 14, 2, 1, {0, 0, 0, 0}}, {18, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_INTERP_BILINEAR_GRAYSCALE[] = {{20, 14, 3, 0, {0, 1, 2, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_INTERP_BILINEAR_GRAYSCALE[] = {{22, 14, 3, 0, {0, 1, 2, 0}}, {22, 14, 2, 1, {0, 0, 0, 0}}, {22, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_INTERP_BILINEAR_GRAYSCALE[] = {{24, 14, 3, 0, {0, 1, 2, 0}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_INTERP_BILINEAR_GRAYSCALE[] = {{26, 14, 3, 0, {0, 1, 2, 0}}, {26, 14, 2, 1, {0, 0, 0, 0}}, {26, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_INTERP_BILINEAR_GRAYSCALE[] = {{28, 14, 3, 0, {0, 1, 2, 0}}, {28, 14, 2, 1, {0, 0, 0, 0}}, {28, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_INTERP_BILINEAR_GRAYSCALE[] = {{2, 16, 3, 0, {0, 1, 2, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_INTERP_BILINEAR_GRAYSCALE[] = {{4, 16, 3, 0, {0, 1, 2, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_INTERP_BILINEAR_GRAYSCALE[] = {{6, 16, 3, 0, {0, 1, 2, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}, {6, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_INTERP_BILINEAR_GRAYSCALE[] = {{8, 16, 3, 0, {0, 1, 2, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_INTERP_BILINEAR_GRAYSCALE[] = {{10, 16, 3, 0, {0, 1, 2, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}, {10, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_INTERP_BILINEAR_GRAYSCALE[] = {{12, 16, 3, 0, {0, 1, 2, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_INTERP_BILINEAR_GRAYSCALE[] = {{14, 16, 3, 0, {0, 1, 2, 0}}, {14, 16, 2, 1, {0, 0, 0, 0}}, {14, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_INTERP_BILINEAR_GRAYSCALE[] = {{16, 16, 3, 0, {0, 1, 2, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_INTERP_BILINEAR_GRAYSCALE[] = {{18, 16, 3, 0, {0, 1, 2, 0}}, {18, 16, 2, 1, {0, 0, 0, 0}}, {18, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_INTERP_BILINEAR_GRAYSCALE[] = {{20, 16, 3, 0, {0, 1, 2, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_INTERP_BILINEAR_GRAYSCALE[] = {{22, 16, 3, 0, {0, 1, 2, 0}}, {22, 16, 2, 1, {0, 0, 0, 0}}, {22, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_INTERP_BILINEAR_GRAYSCALE[] = {{24, 16, 3, 0, {0, 1, 2, 0}}, {24, 16, 2, 1, {0, 0, 0, 0}}, {24, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_INTERP_BILINEAR_GRAYSCALE[] = {{2, 18, 3, 0, {0, 1, 2, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_INTERP_BILINEAR_GRAYSCALE[] = {{4, 18, 3, 0, {0, 1, 2, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_INTERP_BILINEAR_GRAYSCALE[] = {{6, 18, 3, 0, {0, 1, 2, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}, {6, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_INTERP_BILINEAR_GRAYSCALE[] = {{8, 18, 3, 0, {0, 1, 2, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_INTERP_BILINEAR_GRAYSCALE[] = {{10, 18, 3, 0, {0, 1, 2, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}, {10, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_INTERP_BILINEAR_GRAYSCALE[] = {{12, 18, 3, 0, {0, 1, 2, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_INTERP_BILINEAR_GRAYSCALE[] = {{14, 18, 3, 0, {0, 1, 2, 0}}, {14, 18, 2, 1, {0, 0, 0, 0}}, {14, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_INTERP_BILINEAR_GRAYSCALE[] = {{16, 18, 3, 0, {0, 1, 2, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_INTERP_BILINEAR_GRAYSCALE[] = {{18, 18, 3, 0, {0, 1, 2, 0}}, {18, 18, 2, 1, {0, 0, 0, 0}}, {18, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_INTERP_BILINEAR_GRAYSCALE[] = {{20, 18, 3, 0, {0, 1, 2, 0}}, {20, 18, 2, 1, {0, 0, 0, 0}}, {20, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_INTERP_BILINEAR_GRAYSCALE[] = {{22, 18, 3, 0, {0, 1, 2, 0}}, {22, 18, 2, 1, {0, 0, 0, 0}}, {22, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_INTERP_BILINEAR_GRAYSCALE[] = {{2, 20, 3, 0, {0, 1, 2, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_INTERP_BILINEAR_GRAYSCALE[] = {{4, 20, 3, 0, {0, 1, 2, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_INTERP_BILINEAR_GRAYSCALE[] = {{6, 20, 3, 0, {0, 1, 2, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}, {6, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_INTERP_BILINEAR_GRAYSCALE[] = {{8, 20, 3, 0, {0, 1, 2, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_INTERP_BILINEAR_GRAYSCALE[] = {{10, 20, 3, 0, {0, 1, 2, 0}}, {10, 20, 2, 1, {0, 0, 0, 0}}, {10, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_INTERP_BILINEAR_GRAYSCALE[] = {{12, 20, 3, 0, {0, 1, 2, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_INTERP_BILINEAR_GRAYSCALE[] = {{14, 20, 3, 0, {0, 1, 2, 0}}, {14, 20, 2, 1, {0, 0, 0, 0}}, {14, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_INTERP_BILINEAR_GRAYSCALE[] = {{16, 20, 3, 0, {0, 1, 2, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_INTERP_BILINEAR_GRAYSCALE[] = {{18, 20, 3, 0, {0, 1, 2, 0}}, {18, 20, 2, 1, {0, 0, 0, 0}}, {18, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_INTERP_BILINEAR_GRAYSCALE[] = {{20, 20, 3, 0, {0, 1, 2, 0}}, {20, 20, 2, 1, {0, 0, 0, 0}}, {20, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_INTERP_BILINEAR_GRAYSCALE[] = {{2, 22, 3, 0, {0, 1, 2, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_INTERP_BILINEAR_GRAYSCALE[] = {{4, 22, 3, 0, {0, 1, 2, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_INTERP_BILINEAR_GRAYSCALE[] = {{6, 22, 3, 0, {0, 1, 2, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}, {6, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_INTERP_BILINEAR_GRAYSCALE[] = {{8, 22, 3, 0, {0, 1, 2, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_INTERP_BILINEAR_GRAYSCALE[] = {{10, 22, 3, 0, {0, 1, 2, 0}}, {10, 22, 2, 1, {0, 0, 0, 0}}, {10, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_INTERP_BILINEAR_GRAYSCALE[] = {{12, 22, 3, 0, {0, 1, 2, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_INTERP_BILINEAR_GRAYSCALE[] = {{14, 22, 3, 0, {0, 1, 2, 0}}, {14, 22, 2, 1, {0, 0, 0, 0}}, {14, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_INTERP_BILINEAR_GRAYSCALE[] = {{16, 22, 3, 0, {0, 1, 2, 0}}, {16, 22, 2, 1, {0, 0, 0, 0}}, {16, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_INTERP_BILINEAR_GRAYSCALE[] = {{18, 22, 3, 0, {0, 1, 2, 0}}, {18, 22, 2, 1, {0, 0, 0, 0}}, {18, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_INTERP_BILINEAR_GRAYSCALE[] = {{2, 24, 3, 0, {0, 1, 2, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_INTERP_BILINEAR_GRAYSCALE[] = {{4, 24, 3, 0, {0, 1, 2, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_INTERP_BILINEAR_GRAYSCALE[] = {{6, 24, 3, 0, {0, 1, 2, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}, {6, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_INTERP_BILINEAR_GRAYSCALE[] = {{8, 24, 3, 0, {0, 1, 2, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_INTERP_BILINEAR_GRAYSCALE[] = {{10, 24, 3, 0, {0, 1, 2, 0}}, {10, 24, 2, 1, {0, 0, 0, 0}}, {10, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_INTERP_BILINEAR_GRAYSCALE[] = {{12, 24, 3, 0, {0, 1, 2, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_INTERP_BILINEAR_GRAYSCALE[] = {{14, 24, 3, 0, {0, 1, 2, 0}}, {14, 24, 2, 1, {0, 0, 0, 0}}, {14, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_INTERP_BILINEAR_GRAYSCALE[] = {{16, 24, 3, 0, {0, 1, 2, 0}}, {16, 24, 2, 1, {0, 0, 0, 0}}, {16, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_INTERP_BILINEAR_GRAYSCALE[] = {{2, 26, 3, 0, {0, 1, 2, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}, {2, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_INTERP_BILINEAR_GRAYSCALE[] = {{4, 26, 3, 0, {0, 1, 2, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_INTERP_BILINEAR_GRAYSCALE[] = {{6, 26, 3, 0, {0, 1, 2, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}, {6, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_INTERP_BILINEAR_GRAYSCALE[] = {{8, 26, 3, 0, {0, 1, 2, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_INTERP_BILINEAR_GRAYSCALE[] = {{10, 26, 3, 0, {0, 1, 2, 0}}, {10, 26, 2, 1, {0, 0, 0, 0}}, {10, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_INTERP_BILINEAR_GRAYSCALE[] = {{12, 26, 3, 0, {0, 1, 2, 0}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_INTERP_BILINEAR_GRAYSCALE[] = {{14, 26, 3, 0, {0, 1, 2, 0}}, {14, 26, 2, 1, {0, 0, 0, 0}}, {14, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_INTERP_BILINEAR_GRAYSCALE[] = {{2, 28, 3, 0, {0, 1, 2, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}, {2, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_INTERP_BILINEAR_GRAYSCALE[] = {{4, 28, 3, 0, {0, 1, 2, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_INTERP_BILINEAR_GRAYSCALE[] = {{6, 28, 3, 0, {0, 1, 2, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}, {6, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_INTERP_BILINEAR_GRAYSCALE[] = {{8, 28, 3, 0, {0, 1, 2, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_INTERP_BILINEAR_GRAYSCALE[] = {{10, 28, 3, 0, {0, 1, 2, 0}}, {10, 28, 2, 1, {0, 0, 0, 0}}, {10, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_INTERP_BILINEAR_GRAYSCALE[] = {{12, 28, 3, 0, {0, 1, 2, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_INTERP_BILINEAR_GRAYSCALE[] = {{14, 28, 3, 0, {0, 1, 2, 0}}, {14, 28, 2, 1, {0, 0, 0, 0}}, {14, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_INTERP_BILINEAR_GRAYSCALE[] = {{2, 30, 3, 0, {0, 1, 2, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}, {2, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_INTERP_BILINEAR_GRAYSCALE[] = {{4, 30, 3, 0, {0, 1, 2, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_INTERP_BILINEAR_GRAYSCALE[] = {{6, 30, 3, 0, {0, 1, 2, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}, {6, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_INTERP_BILINEAR_GRAYSCALE[] = {{8, 30, 3, 0, {0, 1, 2, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_INTERP_BILINEAR_GRAYSCALE[] = {{10, 30, 3, 0, {0, 1, 2, 0}}, {10, 30, 2, 1, {0, 0, 0, 0}}, {10, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_INTERP_BILINEAR_GRAYSCALE[] = {{12, 30, 3, 0, {0, 1, 2, 0}}, {12, 30, 2, 1, {0, 0, 0, 0}}, {12, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_INTERP_BILINEAR_GRAYSCALE[] = {{2, 32, 3, 0, {0, 1, 2, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}, {2, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_INTERP_BILINEAR_GRAYSCALE[] = {{4, 32, 3, 0, {0, 1, 2, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_INTERP_BILINEAR_GRAYSCALE[] = {{6, 32, 3, 0, {0, 1, 2, 0}}, {6, 32, 2, 1, {0, 0, 0, 0}}, {6, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_INTERP_BILINEAR_GRAYSCALE[] = {{8, 32, 3, 0, {0, 1, 2, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_INTERP_BILINEAR_GRAYSCALE[] = {{10, 32, 3, 0, {0, 1, 2, 0}}, {10, 32, 2, 1, {0, 0, 0, 0}}, {10, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_INTERP_BILINEAR_GRAYSCALE[] = {{12, 32, 3, 0, {0, 1, 2, 0}}, {12, 32, 2, 1, {0, 0, 0, 0}}, {12, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_INTERP_BILINEAR_GRAYSCALE[] = {{1, 0}};

static acf_scenario gScenarioArray_INTERP_BILINEAR_GRAYSCALE[] = {
{2, 1, 28, 0, 1, gScenarioBufferData0_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData0_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 1, 48, 0, 1, gScenarioBufferData1_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData1_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 1, 68, 0, 1, gScenarioBufferData2_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData2_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 1, 88, 0, 1, gScenarioBufferData3_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData3_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 1, 108, 0, 1, gScenarioBufferData4_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData4_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 1, 128, 0, 1, gScenarioBufferData5_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData5_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 1, 148, 0, 1, gScenarioBufferData6_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData6_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 1, 168, 0, 1, gScenarioBufferData7_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData7_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 1, 188, 0, 1, gScenarioBufferData8_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData8_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 1, 208, 0, 1, gScenarioBufferData9_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData9_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 1, 228, 0, 1, gScenarioBufferData10_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData10_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 1, 248, 0, 1, gScenarioBufferData11_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData11_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 1, 268, 0, 1, gScenarioBufferData12_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData12_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 1, 288, 0, 1, gScenarioBufferData13_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData13_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 1, 308, 0, 1, gScenarioBufferData14_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData14_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 1, 328, 0, 1, gScenarioBufferData15_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData15_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 1, 368, 0, 1, gScenarioBufferData16_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData16_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 1, 408, 0, 1, gScenarioBufferData17_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData17_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 1, 448, 0, 1, gScenarioBufferData18_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData18_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 1, 488, 0, 1, gScenarioBufferData19_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData19_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 1, 528, 0, 1, gScenarioBufferData20_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData20_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 1, 568, 0, 1, gScenarioBufferData21_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData21_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 1, 608, 0, 1, gScenarioBufferData22_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData22_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 1, 648, 0, 1, gScenarioBufferData23_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData23_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 2, 52, 0, 1, gScenarioBufferData24_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData24_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 2, 92, 0, 1, gScenarioBufferData25_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData25_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 2, 128, 0, 1, gScenarioBufferData26_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData26_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 2, 168, 0, 1, gScenarioBufferData27_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData27_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 2, 204, 0, 1, gScenarioBufferData28_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData28_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 2, 244, 0, 1, gScenarioBufferData29_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData29_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 2, 280, 0, 1, gScenarioBufferData30_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData30_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 2, 320, 0, 1, gScenarioBufferData31_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData31_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 2, 356, 0, 1, gScenarioBufferData32_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData32_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 2, 396, 0, 1, gScenarioBufferData33_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData33_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 2, 432, 0, 1, gScenarioBufferData34_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData34_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 2, 472, 0, 1, gScenarioBufferData35_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData35_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 2, 508, 0, 1, gScenarioBufferData36_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData36_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 2, 548, 0, 1, gScenarioBufferData37_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData37_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 2, 584, 0, 1, gScenarioBufferData38_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData38_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 2, 624, 0, 1, gScenarioBufferData39_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData39_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 2, 700, 0, 1, gScenarioBufferData40_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData40_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 2, 776, 0, 1, gScenarioBufferData41_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData41_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 2, 852, 0, 1, gScenarioBufferData42_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData42_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 2, 928, 0, 1, gScenarioBufferData43_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData43_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 2, 1004, 0, 1, gScenarioBufferData44_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData44_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 2, 1080, 0, 1, gScenarioBufferData45_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData45_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 2, 1156, 0, 1, gScenarioBufferData46_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData46_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 2, 1232, 0, 1, gScenarioBufferData47_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData47_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 3, 76, 0, 1, gScenarioBufferData48_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData48_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 3, 132, 0, 1, gScenarioBufferData49_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData49_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 3, 188, 0, 1, gScenarioBufferData50_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData50_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 3, 244, 0, 1, gScenarioBufferData51_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData51_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 3, 300, 0, 1, gScenarioBufferData52_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData52_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 3, 356, 0, 1, gScenarioBufferData53_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData53_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 3, 412, 0, 1, gScenarioBufferData54_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData54_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 3, 468, 0, 1, gScenarioBufferData55_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData55_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 3, 524, 0, 1, gScenarioBufferData56_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData56_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 3, 580, 0, 1, gScenarioBufferData57_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData57_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 3, 636, 0, 1, gScenarioBufferData58_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData58_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 3, 692, 0, 1, gScenarioBufferData59_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData59_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 3, 748, 0, 1, gScenarioBufferData60_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData60_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 3, 804, 0, 1, gScenarioBufferData61_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData61_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 3, 860, 0, 1, gScenarioBufferData62_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData62_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 3, 916, 0, 1, gScenarioBufferData63_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData63_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 3, 1028, 0, 1, gScenarioBufferData64_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData64_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 3, 1140, 0, 1, gScenarioBufferData65_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData65_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 3, 1252, 0, 1, gScenarioBufferData66_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData66_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 3, 1364, 0, 1, gScenarioBufferData67_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData67_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 3, 1476, 0, 1, gScenarioBufferData68_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData68_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 3, 1588, 0, 1, gScenarioBufferData69_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData69_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 3, 1700, 0, 1, gScenarioBufferData70_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData70_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 3, 1812, 0, 1, gScenarioBufferData71_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData71_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 4, 100, 0, 1, gScenarioBufferData72_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData72_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 4, 176, 0, 1, gScenarioBufferData73_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData73_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 4, 248, 0, 1, gScenarioBufferData74_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData74_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 4, 324, 0, 1, gScenarioBufferData75_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData75_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 4, 396, 0, 1, gScenarioBufferData76_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData76_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 4, 472, 0, 1, gScenarioBufferData77_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData77_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 4, 544, 0, 1, gScenarioBufferData78_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData78_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 4, 620, 0, 1, gScenarioBufferData79_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData79_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 4, 692, 0, 1, gScenarioBufferData80_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData80_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 4, 768, 0, 1, gScenarioBufferData81_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData81_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 4, 840, 0, 1, gScenarioBufferData82_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData82_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 4, 916, 0, 1, gScenarioBufferData83_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData83_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 4, 988, 0, 1, gScenarioBufferData84_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData84_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 4, 1064, 0, 1, gScenarioBufferData85_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData85_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 4, 1136, 0, 1, gScenarioBufferData86_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData86_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 4, 1212, 0, 1, gScenarioBufferData87_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData87_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 4, 1360, 0, 1, gScenarioBufferData88_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData88_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 4, 1508, 0, 1, gScenarioBufferData89_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData89_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 4, 1656, 0, 1, gScenarioBufferData90_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData90_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 4, 1804, 0, 1, gScenarioBufferData91_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData91_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 4, 1952, 0, 1, gScenarioBufferData92_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData92_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 4, 2100, 0, 1, gScenarioBufferData93_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData93_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 4, 2248, 0, 1, gScenarioBufferData94_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData94_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 4, 2396, 0, 1, gScenarioBufferData95_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData95_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 5, 124, 0, 1, gScenarioBufferData96_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData96_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 5, 216, 0, 1, gScenarioBufferData97_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData97_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 5, 308, 0, 1, gScenarioBufferData98_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData98_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 5, 400, 0, 1, gScenarioBufferData99_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData99_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 5, 492, 0, 1, gScenarioBufferData100_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData100_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 5, 584, 0, 1, gScenarioBufferData101_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData101_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 5, 676, 0, 1, gScenarioBufferData102_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData102_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 5, 768, 0, 1, gScenarioBufferData103_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData103_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 5, 860, 0, 1, gScenarioBufferData104_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData104_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 5, 952, 0, 1, gScenarioBufferData105_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData105_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 5, 1044, 0, 1, gScenarioBufferData106_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData106_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 5, 1136, 0, 1, gScenarioBufferData107_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData107_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 5, 1228, 0, 1, gScenarioBufferData108_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData108_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 5, 1320, 0, 1, gScenarioBufferData109_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData109_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 5, 1412, 0, 1, gScenarioBufferData110_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData110_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 5, 1504, 0, 1, gScenarioBufferData111_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData111_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 5, 1688, 0, 1, gScenarioBufferData112_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData112_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 5, 1872, 0, 1, gScenarioBufferData113_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData113_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 5, 2056, 0, 1, gScenarioBufferData114_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData114_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 5, 2240, 0, 1, gScenarioBufferData115_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData115_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 5, 2424, 0, 1, gScenarioBufferData116_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData116_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 5, 2608, 0, 1, gScenarioBufferData117_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData117_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 5, 2792, 0, 1, gScenarioBufferData118_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData118_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 5, 2976, 0, 1, gScenarioBufferData119_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData119_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 6, 148, 0, 1, gScenarioBufferData120_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData120_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 6, 260, 0, 1, gScenarioBufferData121_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData121_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 6, 368, 0, 1, gScenarioBufferData122_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData122_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 6, 480, 0, 1, gScenarioBufferData123_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData123_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 6, 588, 0, 1, gScenarioBufferData124_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData124_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 6, 700, 0, 1, gScenarioBufferData125_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData125_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 6, 808, 0, 1, gScenarioBufferData126_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData126_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 6, 920, 0, 1, gScenarioBufferData127_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData127_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 6, 1028, 0, 1, gScenarioBufferData128_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData128_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 6, 1140, 0, 1, gScenarioBufferData129_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData129_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 6, 1248, 0, 1, gScenarioBufferData130_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData130_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 6, 1360, 0, 1, gScenarioBufferData131_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData131_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 6, 1468, 0, 1, gScenarioBufferData132_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData132_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 6, 1580, 0, 1, gScenarioBufferData133_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData133_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 6, 1688, 0, 1, gScenarioBufferData134_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData134_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 6, 1800, 0, 1, gScenarioBufferData135_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData135_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 6, 2020, 0, 1, gScenarioBufferData136_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData136_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 6, 2240, 0, 1, gScenarioBufferData137_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData137_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 6, 2460, 0, 1, gScenarioBufferData138_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData138_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 6, 2680, 0, 1, gScenarioBufferData139_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData139_INTERP_BILINEAR_GRAYSCALE, 4},
{52, 6, 2900, 0, 1, gScenarioBufferData140_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData140_INTERP_BILINEAR_GRAYSCALE, 4},
{56, 6, 3120, 0, 1, gScenarioBufferData141_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData141_INTERP_BILINEAR_GRAYSCALE, 4},
{60, 6, 3340, 0, 1, gScenarioBufferData142_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData142_INTERP_BILINEAR_GRAYSCALE, 4},
{64, 6, 3560, 0, 1, gScenarioBufferData143_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData143_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 8, 196, 0, 1, gScenarioBufferData144_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData144_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 8, 344, 0, 1, gScenarioBufferData145_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData145_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 8, 488, 0, 1, gScenarioBufferData146_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData146_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 8, 636, 0, 1, gScenarioBufferData147_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData147_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 8, 780, 0, 1, gScenarioBufferData148_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData148_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 8, 928, 0, 1, gScenarioBufferData149_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData149_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 8, 1072, 0, 1, gScenarioBufferData150_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData150_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 8, 1220, 0, 1, gScenarioBufferData151_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData151_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 8, 1364, 0, 1, gScenarioBufferData152_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData152_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 8, 1512, 0, 1, gScenarioBufferData153_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData153_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 8, 1656, 0, 1, gScenarioBufferData154_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData154_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 8, 1804, 0, 1, gScenarioBufferData155_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData155_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 8, 1948, 0, 1, gScenarioBufferData156_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData156_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 8, 2096, 0, 1, gScenarioBufferData157_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData157_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 8, 2240, 0, 1, gScenarioBufferData158_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData158_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 8, 2388, 0, 1, gScenarioBufferData159_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData159_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 8, 2680, 0, 1, gScenarioBufferData160_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData160_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 8, 2972, 0, 1, gScenarioBufferData161_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData161_INTERP_BILINEAR_GRAYSCALE, 4},
{44, 8, 3264, 0, 1, gScenarioBufferData162_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData162_INTERP_BILINEAR_GRAYSCALE, 4},
{48, 8, 3556, 0, 1, gScenarioBufferData163_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData163_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 10, 244, 0, 1, gScenarioBufferData164_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData164_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 10, 428, 0, 1, gScenarioBufferData165_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData165_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 10, 608, 0, 1, gScenarioBufferData166_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData166_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 10, 792, 0, 1, gScenarioBufferData167_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData167_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 10, 972, 0, 1, gScenarioBufferData168_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData168_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 10, 1156, 0, 1, gScenarioBufferData169_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData169_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 10, 1336, 0, 1, gScenarioBufferData170_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData170_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 10, 1520, 0, 1, gScenarioBufferData171_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData171_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 10, 1700, 0, 1, gScenarioBufferData172_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData172_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 10, 1884, 0, 1, gScenarioBufferData173_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData173_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 10, 2064, 0, 1, gScenarioBufferData174_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData174_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 10, 2248, 0, 1, gScenarioBufferData175_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData175_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 10, 2428, 0, 1, gScenarioBufferData176_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData176_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 10, 2612, 0, 1, gScenarioBufferData177_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData177_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 10, 2792, 0, 1, gScenarioBufferData178_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData178_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 10, 2976, 0, 1, gScenarioBufferData179_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData179_INTERP_BILINEAR_GRAYSCALE, 4},
{36, 10, 3340, 0, 1, gScenarioBufferData180_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData180_INTERP_BILINEAR_GRAYSCALE, 4},
{40, 10, 3704, 0, 1, gScenarioBufferData181_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData181_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 12, 292, 0, 1, gScenarioBufferData182_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData182_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 12, 512, 0, 1, gScenarioBufferData183_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData183_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 12, 728, 0, 1, gScenarioBufferData184_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData184_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 12, 948, 0, 1, gScenarioBufferData185_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData185_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 12, 1164, 0, 1, gScenarioBufferData186_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData186_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 12, 1384, 0, 1, gScenarioBufferData187_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData187_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 12, 1600, 0, 1, gScenarioBufferData188_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData188_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 12, 1820, 0, 1, gScenarioBufferData189_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData189_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 12, 2036, 0, 1, gScenarioBufferData190_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData190_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 12, 2256, 0, 1, gScenarioBufferData191_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData191_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 12, 2472, 0, 1, gScenarioBufferData192_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData192_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 12, 2692, 0, 1, gScenarioBufferData193_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData193_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 12, 2908, 0, 1, gScenarioBufferData194_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData194_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 12, 3128, 0, 1, gScenarioBufferData195_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData195_INTERP_BILINEAR_GRAYSCALE, 4},
{30, 12, 3344, 0, 1, gScenarioBufferData196_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData196_INTERP_BILINEAR_GRAYSCALE, 4},
{32, 12, 3564, 0, 1, gScenarioBufferData197_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData197_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 14, 340, 0, 1, gScenarioBufferData198_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData198_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 14, 596, 0, 1, gScenarioBufferData199_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData199_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 14, 848, 0, 1, gScenarioBufferData200_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData200_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 14, 1104, 0, 1, gScenarioBufferData201_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData201_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 14, 1356, 0, 1, gScenarioBufferData202_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData202_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 14, 1612, 0, 1, gScenarioBufferData203_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData203_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 14, 1864, 0, 1, gScenarioBufferData204_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData204_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 14, 2120, 0, 1, gScenarioBufferData205_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData205_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 14, 2372, 0, 1, gScenarioBufferData206_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData206_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 14, 2628, 0, 1, gScenarioBufferData207_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData207_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 14, 2880, 0, 1, gScenarioBufferData208_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData208_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 14, 3136, 0, 1, gScenarioBufferData209_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData209_INTERP_BILINEAR_GRAYSCALE, 4},
{26, 14, 3388, 0, 1, gScenarioBufferData210_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData210_INTERP_BILINEAR_GRAYSCALE, 4},
{28, 14, 3644, 0, 1, gScenarioBufferData211_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData211_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 16, 388, 0, 1, gScenarioBufferData212_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData212_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 16, 680, 0, 1, gScenarioBufferData213_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData213_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 16, 968, 0, 1, gScenarioBufferData214_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData214_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 16, 1260, 0, 1, gScenarioBufferData215_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData215_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 16, 1548, 0, 1, gScenarioBufferData216_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData216_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 16, 1840, 0, 1, gScenarioBufferData217_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData217_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 16, 2128, 0, 1, gScenarioBufferData218_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData218_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 16, 2420, 0, 1, gScenarioBufferData219_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData219_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 16, 2708, 0, 1, gScenarioBufferData220_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData220_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 16, 3000, 0, 1, gScenarioBufferData221_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData221_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 16, 3288, 0, 1, gScenarioBufferData222_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData222_INTERP_BILINEAR_GRAYSCALE, 4},
{24, 16, 3580, 0, 1, gScenarioBufferData223_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData223_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 18, 436, 0, 1, gScenarioBufferData224_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData224_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 18, 764, 0, 1, gScenarioBufferData225_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData225_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 18, 1088, 0, 1, gScenarioBufferData226_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData226_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 18, 1416, 0, 1, gScenarioBufferData227_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData227_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 18, 1740, 0, 1, gScenarioBufferData228_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData228_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 18, 2068, 0, 1, gScenarioBufferData229_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData229_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 18, 2392, 0, 1, gScenarioBufferData230_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData230_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 18, 2720, 0, 1, gScenarioBufferData231_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData231_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 18, 3044, 0, 1, gScenarioBufferData232_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData232_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 18, 3372, 0, 1, gScenarioBufferData233_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData233_INTERP_BILINEAR_GRAYSCALE, 4},
{22, 18, 3696, 0, 1, gScenarioBufferData234_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData234_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 20, 484, 0, 1, gScenarioBufferData235_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData235_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 20, 848, 0, 1, gScenarioBufferData236_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData236_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 20, 1208, 0, 1, gScenarioBufferData237_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData237_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 20, 1572, 0, 1, gScenarioBufferData238_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData238_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 20, 1932, 0, 1, gScenarioBufferData239_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData239_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 20, 2296, 0, 1, gScenarioBufferData240_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData240_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 20, 2656, 0, 1, gScenarioBufferData241_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData241_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 20, 3020, 0, 1, gScenarioBufferData242_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData242_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 20, 3380, 0, 1, gScenarioBufferData243_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData243_INTERP_BILINEAR_GRAYSCALE, 4},
{20, 20, 3744, 0, 1, gScenarioBufferData244_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData244_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 22, 532, 0, 1, gScenarioBufferData245_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData245_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 22, 932, 0, 1, gScenarioBufferData246_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData246_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 22, 1328, 0, 1, gScenarioBufferData247_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData247_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 22, 1728, 0, 1, gScenarioBufferData248_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData248_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 22, 2124, 0, 1, gScenarioBufferData249_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData249_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 22, 2524, 0, 1, gScenarioBufferData250_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData250_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 22, 2920, 0, 1, gScenarioBufferData251_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData251_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 22, 3320, 0, 1, gScenarioBufferData252_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData252_INTERP_BILINEAR_GRAYSCALE, 4},
{18, 22, 3716, 0, 1, gScenarioBufferData253_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData253_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 24, 580, 0, 1, gScenarioBufferData254_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData254_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 24, 1016, 0, 1, gScenarioBufferData255_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData255_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 24, 1448, 0, 1, gScenarioBufferData256_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData256_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 24, 1884, 0, 1, gScenarioBufferData257_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData257_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 24, 2316, 0, 1, gScenarioBufferData258_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData258_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 24, 2752, 0, 1, gScenarioBufferData259_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData259_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 24, 3184, 0, 1, gScenarioBufferData260_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData260_INTERP_BILINEAR_GRAYSCALE, 4},
{16, 24, 3620, 0, 1, gScenarioBufferData261_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData261_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 26, 628, 0, 1, gScenarioBufferData262_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData262_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 26, 1100, 0, 1, gScenarioBufferData263_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData263_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 26, 1568, 0, 1, gScenarioBufferData264_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData264_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 26, 2040, 0, 1, gScenarioBufferData265_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData265_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 26, 2508, 0, 1, gScenarioBufferData266_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData266_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 26, 2980, 0, 1, gScenarioBufferData267_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData267_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 26, 3448, 0, 1, gScenarioBufferData268_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData268_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 28, 676, 0, 1, gScenarioBufferData269_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData269_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 28, 1184, 0, 1, gScenarioBufferData270_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData270_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 28, 1688, 0, 1, gScenarioBufferData271_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData271_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 28, 2196, 0, 1, gScenarioBufferData272_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData272_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 28, 2700, 0, 1, gScenarioBufferData273_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData273_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 28, 3208, 0, 1, gScenarioBufferData274_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData274_INTERP_BILINEAR_GRAYSCALE, 4},
{14, 28, 3712, 0, 1, gScenarioBufferData275_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData275_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 30, 724, 0, 1, gScenarioBufferData276_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData276_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 30, 1268, 0, 1, gScenarioBufferData277_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData277_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 30, 1808, 0, 1, gScenarioBufferData278_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData278_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 30, 2352, 0, 1, gScenarioBufferData279_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData279_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 30, 2892, 0, 1, gScenarioBufferData280_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData280_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 30, 3436, 0, 1, gScenarioBufferData281_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData281_INTERP_BILINEAR_GRAYSCALE, 4},
{2, 32, 772, 0, 1, gScenarioBufferData282_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData282_INTERP_BILINEAR_GRAYSCALE, 4},
{4, 32, 1352, 0, 1, gScenarioBufferData283_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData283_INTERP_BILINEAR_GRAYSCALE, 4},
{6, 32, 1928, 0, 1, gScenarioBufferData284_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData284_INTERP_BILINEAR_GRAYSCALE, 4},
{8, 32, 2508, 0, 1, gScenarioBufferData285_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData285_INTERP_BILINEAR_GRAYSCALE, 4},
{10, 32, 3084, 0, 1, gScenarioBufferData286_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData286_INTERP_BILINEAR_GRAYSCALE, 4},
{12, 32, 3664, 0, 1, gScenarioBufferData287_INTERP_BILINEAR_GRAYSCALE, 48, gScenarioKernelData287_INTERP_BILINEAR_GRAYSCALE, 4}
};

static acf_scenario_list gScenarioList_INTERP_BILINEAR_GRAYSCALE = {
288, //number of scenarios
gScenarioArray_INTERP_BILINEAR_GRAYSCALE};
//**************************************************************

class INTERP_BILINEAR_GRAYSCALE : public ACF_Process_APU
{

public:
   INTERP_BILINEAR_GRAYSCALE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INTERP_BILINEAR_GRAYSCALE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INTERP_BILINEAR_GRAYSCALE",
                                        INTERP_BILINEAR_GRAYSCALE_LOAD_SEGMENTS,
                                        INTERP_BILINEAR_GRAYSCALE_LOAD_PMEM, INTERP_BILINEAR_GRAYSCALE_LOAD_PMEM_SIZE,
                                        INTERP_BILINEAR_GRAYSCALE_LOAD_DMEM, INTERP_BILINEAR_GRAYSCALE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INTERP_BILINEAR_GRAYSCALE_APEX_LOG_BUFFER, INTERP_BILINEAR_GRAYSCALE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0);
         AddPort("DELTA_XY", icp::DATATYPE_08U, 2, 1, 2, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INTERP_BILINEAR_GRAYSCALE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INTERP_BILINEAR_GRAYSCALE

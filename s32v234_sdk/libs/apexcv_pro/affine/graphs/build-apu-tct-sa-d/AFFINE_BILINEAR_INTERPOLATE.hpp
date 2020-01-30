#ifndef _ACF_PROCESS_APU_AFFINE_BILINEAR_INTERPOLATE
#define _ACF_PROCESS_APU_AFFINE_BILINEAR_INTERPOLATE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AFFINE_BILINEAR_INTERPOLATE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {52, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {56, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {60, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {64, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {44, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {48, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {52, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {56, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {60, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {64, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {36, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {44, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {48, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {52, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {56, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {60, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {64, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {28, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {36, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {40, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {44, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 14, 2, 0, {0, 0, 0, 0}}, {48, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 14, 2, 0, {0, 0, 0, 0}}, {52, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 14, 2, 0, {0, 0, 0, 0}}, {56, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 14, 2, 0, {0, 0, 0, 0}}, {60, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_AFFINE_BILINEAR_INTERPOLATE[] = {{64, 14, 2, 0, {0, 0, 0, 0}}, {64, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {28, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {32, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {36, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {40, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {44, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {48, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 16, 2, 0, {0, 0, 0, 0}}, {52, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_AFFINE_BILINEAR_INTERPOLATE[] = {{56, 16, 2, 0, {0, 0, 0, 0}}, {56, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_AFFINE_BILINEAR_INTERPOLATE[] = {{60, 16, 2, 0, {0, 0, 0, 0}}, {60, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {28, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {32, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 18, 2, 0, {0, 0, 0, 0}}, {36, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 18, 2, 0, {0, 0, 0, 0}}, {40, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 18, 2, 0, {0, 0, 0, 0}}, {44, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 18, 2, 0, {0, 0, 0, 0}}, {48, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_AFFINE_BILINEAR_INTERPOLATE[] = {{52, 18, 2, 0, {0, 0, 0, 0}}, {52, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {28, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {32, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {36, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {40, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_AFFINE_BILINEAR_INTERPOLATE[] = {{44, 20, 2, 0, {0, 0, 0, 0}}, {44, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_AFFINE_BILINEAR_INTERPOLATE[] = {{48, 20, 2, 0, {0, 0, 0, 0}}, {48, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {24, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {28, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 22, 2, 0, {0, 0, 0, 0}}, {32, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 22, 2, 0, {0, 0, 0, 0}}, {36, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 22, 2, 0, {0, 0, 0, 0}}, {40, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {24, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {28, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {32, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 24, 2, 0, {0, 0, 0, 0}}, {36, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_AFFINE_BILINEAR_INTERPOLATE[] = {{40, 24, 2, 0, {0, 0, 0, 0}}, {40, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {20, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {24, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 26, 2, 0, {0, 0, 0, 0}}, {28, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 26, 2, 0, {0, 0, 0, 0}}, {32, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_AFFINE_BILINEAR_INTERPOLATE[] = {{36, 26, 2, 0, {0, 0, 0, 0}}, {36, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {20, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 28, 2, 0, {0, 0, 0, 0}}, {24, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 28, 2, 0, {0, 0, 0, 0}}, {28, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 28, 2, 0, {0, 0, 0, 0}}, {32, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {20, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 30, 2, 0, {0, 0, 0, 0}}, {24, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 30, 2, 0, {0, 0, 0, 0}}, {28, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_AFFINE_BILINEAR_INTERPOLATE[] = {{32, 30, 2, 0, {0, 0, 0, 0}}, {32, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_AFFINE_BILINEAR_INTERPOLATE[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_AFFINE_BILINEAR_INTERPOLATE[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_AFFINE_BILINEAR_INTERPOLATE[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_AFFINE_BILINEAR_INTERPOLATE[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {16, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_AFFINE_BILINEAR_INTERPOLATE[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {20, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_AFFINE_BILINEAR_INTERPOLATE[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {24, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_AFFINE_BILINEAR_INTERPOLATE[] = {{28, 32, 2, 0, {0, 0, 0, 0}}, {28, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_AFFINE_BILINEAR_INTERPOLATE[] = {{0, 0}};

static acf_scenario gScenarioArray_AFFINE_BILINEAR_INTERPOLATE[] = {
{4, 1, 16, 64, 0, gScenarioBufferData0_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData0_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 1, 32, 64, 0, gScenarioBufferData1_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData1_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 1, 48, 64, 0, gScenarioBufferData2_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData2_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 1, 64, 64, 0, gScenarioBufferData3_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData3_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 1, 80, 64, 0, gScenarioBufferData4_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData4_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 1, 96, 64, 0, gScenarioBufferData5_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData5_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 1, 112, 64, 0, gScenarioBufferData6_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData6_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 1, 128, 64, 0, gScenarioBufferData7_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData7_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 1, 144, 64, 0, gScenarioBufferData8_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData8_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 1, 160, 64, 0, gScenarioBufferData9_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData9_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 1, 176, 64, 0, gScenarioBufferData10_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData10_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 1, 192, 64, 0, gScenarioBufferData11_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData11_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 1, 208, 64, 0, gScenarioBufferData12_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData12_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 1, 224, 64, 0, gScenarioBufferData13_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData13_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 1, 240, 64, 0, gScenarioBufferData14_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData14_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 1, 256, 64, 0, gScenarioBufferData15_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData15_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 2, 32, 64, 0, gScenarioBufferData16_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData16_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 2, 64, 64, 0, gScenarioBufferData17_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData17_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 2, 96, 64, 0, gScenarioBufferData18_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData18_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 2, 128, 64, 0, gScenarioBufferData19_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData19_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 2, 160, 64, 0, gScenarioBufferData20_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData20_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 2, 192, 64, 0, gScenarioBufferData21_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData21_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 2, 224, 64, 0, gScenarioBufferData22_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData22_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 2, 256, 64, 0, gScenarioBufferData23_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData23_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 2, 288, 64, 0, gScenarioBufferData24_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData24_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 2, 320, 64, 0, gScenarioBufferData25_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData25_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 2, 352, 64, 0, gScenarioBufferData26_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData26_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 2, 384, 64, 0, gScenarioBufferData27_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData27_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 2, 416, 64, 0, gScenarioBufferData28_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData28_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 2, 448, 64, 0, gScenarioBufferData29_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData29_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 2, 480, 64, 0, gScenarioBufferData30_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData30_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 2, 512, 64, 0, gScenarioBufferData31_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData31_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 3, 48, 64, 0, gScenarioBufferData32_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData32_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 3, 96, 64, 0, gScenarioBufferData33_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData33_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 3, 144, 64, 0, gScenarioBufferData34_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData34_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 3, 192, 64, 0, gScenarioBufferData35_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData35_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 3, 240, 64, 0, gScenarioBufferData36_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData36_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 3, 288, 64, 0, gScenarioBufferData37_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData37_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 3, 336, 64, 0, gScenarioBufferData38_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData38_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 3, 384, 64, 0, gScenarioBufferData39_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData39_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 3, 432, 64, 0, gScenarioBufferData40_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData40_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 3, 480, 64, 0, gScenarioBufferData41_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData41_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 3, 528, 64, 0, gScenarioBufferData42_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData42_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 3, 576, 64, 0, gScenarioBufferData43_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData43_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 3, 624, 64, 0, gScenarioBufferData44_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData44_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 3, 672, 64, 0, gScenarioBufferData45_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData45_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 3, 720, 64, 0, gScenarioBufferData46_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData46_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 3, 768, 64, 0, gScenarioBufferData47_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData47_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 4, 64, 64, 0, gScenarioBufferData48_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData48_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 4, 128, 64, 0, gScenarioBufferData49_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData49_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 4, 192, 64, 0, gScenarioBufferData50_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData50_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 4, 256, 64, 0, gScenarioBufferData51_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData51_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 4, 320, 64, 0, gScenarioBufferData52_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData52_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 4, 384, 64, 0, gScenarioBufferData53_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData53_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 4, 448, 64, 0, gScenarioBufferData54_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData54_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 4, 512, 64, 0, gScenarioBufferData55_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData55_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 4, 576, 64, 0, gScenarioBufferData56_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData56_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 4, 640, 64, 0, gScenarioBufferData57_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData57_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 4, 704, 64, 0, gScenarioBufferData58_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData58_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 4, 768, 64, 0, gScenarioBufferData59_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData59_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 4, 832, 64, 0, gScenarioBufferData60_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData60_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 4, 896, 64, 0, gScenarioBufferData61_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData61_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 4, 960, 64, 0, gScenarioBufferData62_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData62_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 4, 1024, 64, 0, gScenarioBufferData63_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData63_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 5, 80, 64, 0, gScenarioBufferData64_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData64_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 5, 160, 64, 0, gScenarioBufferData65_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData65_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 5, 240, 64, 0, gScenarioBufferData66_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData66_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 5, 320, 64, 0, gScenarioBufferData67_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData67_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 5, 400, 64, 0, gScenarioBufferData68_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData68_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 5, 480, 64, 0, gScenarioBufferData69_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData69_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 5, 560, 64, 0, gScenarioBufferData70_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData70_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 5, 640, 64, 0, gScenarioBufferData71_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData71_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 5, 720, 64, 0, gScenarioBufferData72_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData72_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 5, 800, 64, 0, gScenarioBufferData73_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData73_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 5, 880, 64, 0, gScenarioBufferData74_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData74_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 5, 960, 64, 0, gScenarioBufferData75_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData75_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 5, 1040, 64, 0, gScenarioBufferData76_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData76_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 5, 1120, 64, 0, gScenarioBufferData77_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData77_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 5, 1200, 64, 0, gScenarioBufferData78_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData78_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 5, 1280, 64, 0, gScenarioBufferData79_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData79_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 6, 96, 64, 0, gScenarioBufferData80_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData80_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 6, 192, 64, 0, gScenarioBufferData81_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData81_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 6, 288, 64, 0, gScenarioBufferData82_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData82_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 6, 384, 64, 0, gScenarioBufferData83_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData83_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 6, 480, 64, 0, gScenarioBufferData84_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData84_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 6, 576, 64, 0, gScenarioBufferData85_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData85_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 6, 672, 64, 0, gScenarioBufferData86_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData86_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 6, 768, 64, 0, gScenarioBufferData87_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData87_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 6, 864, 64, 0, gScenarioBufferData88_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData88_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 6, 960, 64, 0, gScenarioBufferData89_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData89_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 6, 1056, 64, 0, gScenarioBufferData90_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData90_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 6, 1152, 64, 0, gScenarioBufferData91_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData91_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 6, 1248, 64, 0, gScenarioBufferData92_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData92_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 6, 1344, 64, 0, gScenarioBufferData93_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData93_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 6, 1440, 64, 0, gScenarioBufferData94_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData94_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 6, 1536, 64, 0, gScenarioBufferData95_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData95_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 8, 128, 64, 0, gScenarioBufferData96_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData96_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 8, 256, 64, 0, gScenarioBufferData97_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData97_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 8, 384, 64, 0, gScenarioBufferData98_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData98_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 8, 512, 64, 0, gScenarioBufferData99_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData99_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 8, 640, 64, 0, gScenarioBufferData100_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData100_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 8, 768, 64, 0, gScenarioBufferData101_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData101_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 8, 896, 64, 0, gScenarioBufferData102_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData102_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 8, 1024, 64, 0, gScenarioBufferData103_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData103_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 8, 1152, 64, 0, gScenarioBufferData104_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData104_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 8, 1280, 64, 0, gScenarioBufferData105_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData105_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 8, 1408, 64, 0, gScenarioBufferData106_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData106_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 8, 1536, 64, 0, gScenarioBufferData107_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData107_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 8, 1664, 64, 0, gScenarioBufferData108_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData108_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 8, 1792, 64, 0, gScenarioBufferData109_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData109_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 8, 1920, 64, 0, gScenarioBufferData110_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData110_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 8, 2048, 64, 0, gScenarioBufferData111_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData111_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 10, 160, 64, 0, gScenarioBufferData112_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData112_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 10, 320, 64, 0, gScenarioBufferData113_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData113_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 10, 480, 64, 0, gScenarioBufferData114_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData114_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 10, 640, 64, 0, gScenarioBufferData115_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData115_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 10, 800, 64, 0, gScenarioBufferData116_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData116_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 10, 960, 64, 0, gScenarioBufferData117_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData117_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 10, 1120, 64, 0, gScenarioBufferData118_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData118_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 10, 1280, 64, 0, gScenarioBufferData119_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData119_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 10, 1440, 64, 0, gScenarioBufferData120_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData120_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 10, 1600, 64, 0, gScenarioBufferData121_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData121_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 10, 1760, 64, 0, gScenarioBufferData122_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData122_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 10, 1920, 64, 0, gScenarioBufferData123_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData123_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 10, 2080, 64, 0, gScenarioBufferData124_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData124_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 10, 2240, 64, 0, gScenarioBufferData125_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData125_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 10, 2400, 64, 0, gScenarioBufferData126_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData126_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 10, 2560, 64, 0, gScenarioBufferData127_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData127_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 12, 192, 64, 0, gScenarioBufferData128_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData128_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 12, 384, 64, 0, gScenarioBufferData129_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData129_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 12, 576, 64, 0, gScenarioBufferData130_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData130_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 12, 768, 64, 0, gScenarioBufferData131_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData131_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 12, 960, 64, 0, gScenarioBufferData132_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData132_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 12, 1152, 64, 0, gScenarioBufferData133_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData133_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 12, 1344, 64, 0, gScenarioBufferData134_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData134_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 12, 1536, 64, 0, gScenarioBufferData135_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData135_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 12, 1728, 64, 0, gScenarioBufferData136_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData136_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 12, 1920, 64, 0, gScenarioBufferData137_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData137_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 12, 2112, 64, 0, gScenarioBufferData138_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData138_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 12, 2304, 64, 0, gScenarioBufferData139_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData139_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 12, 2496, 64, 0, gScenarioBufferData140_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData140_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 12, 2688, 64, 0, gScenarioBufferData141_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData141_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 12, 2880, 64, 0, gScenarioBufferData142_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData142_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 12, 3072, 64, 0, gScenarioBufferData143_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData143_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 14, 224, 64, 0, gScenarioBufferData144_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData144_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 14, 448, 64, 0, gScenarioBufferData145_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData145_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 14, 672, 64, 0, gScenarioBufferData146_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData146_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 14, 896, 64, 0, gScenarioBufferData147_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData147_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 14, 1120, 64, 0, gScenarioBufferData148_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData148_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 14, 1344, 64, 0, gScenarioBufferData149_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData149_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 14, 1568, 64, 0, gScenarioBufferData150_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData150_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 14, 1792, 64, 0, gScenarioBufferData151_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData151_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 14, 2016, 64, 0, gScenarioBufferData152_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData152_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 14, 2240, 64, 0, gScenarioBufferData153_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData153_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 14, 2464, 64, 0, gScenarioBufferData154_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData154_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 14, 2688, 64, 0, gScenarioBufferData155_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData155_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 14, 2912, 64, 0, gScenarioBufferData156_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData156_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 14, 3136, 64, 0, gScenarioBufferData157_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData157_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 14, 3360, 64, 0, gScenarioBufferData158_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData158_AFFINE_BILINEAR_INTERPOLATE, 4},
{64, 14, 3584, 64, 0, gScenarioBufferData159_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData159_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 16, 256, 64, 0, gScenarioBufferData160_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData160_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 16, 512, 64, 0, gScenarioBufferData161_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData161_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 16, 768, 64, 0, gScenarioBufferData162_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData162_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 16, 1024, 64, 0, gScenarioBufferData163_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData163_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 16, 1280, 64, 0, gScenarioBufferData164_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData164_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 16, 1536, 64, 0, gScenarioBufferData165_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData165_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 16, 1792, 64, 0, gScenarioBufferData166_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData166_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 16, 2048, 64, 0, gScenarioBufferData167_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData167_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 16, 2304, 64, 0, gScenarioBufferData168_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData168_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 16, 2560, 64, 0, gScenarioBufferData169_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData169_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 16, 2816, 64, 0, gScenarioBufferData170_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData170_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 16, 3072, 64, 0, gScenarioBufferData171_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData171_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 16, 3328, 64, 0, gScenarioBufferData172_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData172_AFFINE_BILINEAR_INTERPOLATE, 4},
{56, 16, 3584, 64, 0, gScenarioBufferData173_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData173_AFFINE_BILINEAR_INTERPOLATE, 4},
{60, 16, 3840, 64, 0, gScenarioBufferData174_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData174_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 18, 288, 64, 0, gScenarioBufferData175_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData175_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 18, 576, 64, 0, gScenarioBufferData176_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData176_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 18, 864, 64, 0, gScenarioBufferData177_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData177_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 18, 1152, 64, 0, gScenarioBufferData178_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData178_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 18, 1440, 64, 0, gScenarioBufferData179_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData179_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 18, 1728, 64, 0, gScenarioBufferData180_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData180_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 18, 2016, 64, 0, gScenarioBufferData181_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData181_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 18, 2304, 64, 0, gScenarioBufferData182_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData182_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 18, 2592, 64, 0, gScenarioBufferData183_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData183_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 18, 2880, 64, 0, gScenarioBufferData184_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData184_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 18, 3168, 64, 0, gScenarioBufferData185_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData185_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 18, 3456, 64, 0, gScenarioBufferData186_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData186_AFFINE_BILINEAR_INTERPOLATE, 4},
{52, 18, 3744, 64, 0, gScenarioBufferData187_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData187_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 20, 320, 64, 0, gScenarioBufferData188_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData188_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 20, 640, 64, 0, gScenarioBufferData189_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData189_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 20, 960, 64, 0, gScenarioBufferData190_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData190_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 20, 1280, 64, 0, gScenarioBufferData191_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData191_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 20, 1600, 64, 0, gScenarioBufferData192_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData192_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 20, 1920, 64, 0, gScenarioBufferData193_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData193_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 20, 2240, 64, 0, gScenarioBufferData194_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData194_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 20, 2560, 64, 0, gScenarioBufferData195_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData195_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 20, 2880, 64, 0, gScenarioBufferData196_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData196_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 20, 3200, 64, 0, gScenarioBufferData197_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData197_AFFINE_BILINEAR_INTERPOLATE, 4},
{44, 20, 3520, 64, 0, gScenarioBufferData198_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData198_AFFINE_BILINEAR_INTERPOLATE, 4},
{48, 20, 3840, 64, 0, gScenarioBufferData199_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData199_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 22, 352, 64, 0, gScenarioBufferData200_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData200_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 22, 704, 64, 0, gScenarioBufferData201_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData201_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 22, 1056, 64, 0, gScenarioBufferData202_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData202_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 22, 1408, 64, 0, gScenarioBufferData203_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData203_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 22, 1760, 64, 0, gScenarioBufferData204_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData204_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 22, 2112, 64, 0, gScenarioBufferData205_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData205_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 22, 2464, 64, 0, gScenarioBufferData206_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData206_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 22, 2816, 64, 0, gScenarioBufferData207_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData207_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 22, 3168, 64, 0, gScenarioBufferData208_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData208_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 22, 3520, 64, 0, gScenarioBufferData209_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData209_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 24, 384, 64, 0, gScenarioBufferData210_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData210_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 24, 768, 64, 0, gScenarioBufferData211_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData211_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 24, 1152, 64, 0, gScenarioBufferData212_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData212_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 24, 1536, 64, 0, gScenarioBufferData213_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData213_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 24, 1920, 64, 0, gScenarioBufferData214_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData214_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 24, 2304, 64, 0, gScenarioBufferData215_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData215_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 24, 2688, 64, 0, gScenarioBufferData216_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData216_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 24, 3072, 64, 0, gScenarioBufferData217_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData217_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 24, 3456, 64, 0, gScenarioBufferData218_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData218_AFFINE_BILINEAR_INTERPOLATE, 4},
{40, 24, 3840, 64, 0, gScenarioBufferData219_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData219_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 26, 416, 64, 0, gScenarioBufferData220_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData220_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 26, 832, 64, 0, gScenarioBufferData221_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData221_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 26, 1248, 64, 0, gScenarioBufferData222_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData222_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 26, 1664, 64, 0, gScenarioBufferData223_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData223_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 26, 2080, 64, 0, gScenarioBufferData224_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData224_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 26, 2496, 64, 0, gScenarioBufferData225_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData225_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 26, 2912, 64, 0, gScenarioBufferData226_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData226_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 26, 3328, 64, 0, gScenarioBufferData227_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData227_AFFINE_BILINEAR_INTERPOLATE, 4},
{36, 26, 3744, 64, 0, gScenarioBufferData228_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData228_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 28, 448, 64, 0, gScenarioBufferData229_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData229_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 28, 896, 64, 0, gScenarioBufferData230_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData230_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 28, 1344, 64, 0, gScenarioBufferData231_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData231_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 28, 1792, 64, 0, gScenarioBufferData232_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData232_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 28, 2240, 64, 0, gScenarioBufferData233_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData233_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 28, 2688, 64, 0, gScenarioBufferData234_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData234_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 28, 3136, 64, 0, gScenarioBufferData235_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData235_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 28, 3584, 64, 0, gScenarioBufferData236_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData236_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 30, 480, 64, 0, gScenarioBufferData237_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData237_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 30, 960, 64, 0, gScenarioBufferData238_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData238_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 30, 1440, 64, 0, gScenarioBufferData239_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData239_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 30, 1920, 64, 0, gScenarioBufferData240_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData240_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 30, 2400, 64, 0, gScenarioBufferData241_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData241_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 30, 2880, 64, 0, gScenarioBufferData242_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData242_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 30, 3360, 64, 0, gScenarioBufferData243_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData243_AFFINE_BILINEAR_INTERPOLATE, 4},
{32, 30, 3840, 64, 0, gScenarioBufferData244_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData244_AFFINE_BILINEAR_INTERPOLATE, 4},
{4, 32, 512, 64, 0, gScenarioBufferData245_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData245_AFFINE_BILINEAR_INTERPOLATE, 4},
{8, 32, 1024, 64, 0, gScenarioBufferData246_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData246_AFFINE_BILINEAR_INTERPOLATE, 4},
{12, 32, 1536, 64, 0, gScenarioBufferData247_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData247_AFFINE_BILINEAR_INTERPOLATE, 4},
{16, 32, 2048, 64, 0, gScenarioBufferData248_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData248_AFFINE_BILINEAR_INTERPOLATE, 4},
{20, 32, 2560, 64, 0, gScenarioBufferData249_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData249_AFFINE_BILINEAR_INTERPOLATE, 4},
{24, 32, 3072, 64, 0, gScenarioBufferData250_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData250_AFFINE_BILINEAR_INTERPOLATE, 4},
{28, 32, 3584, 64, 0, gScenarioBufferData251_AFFINE_BILINEAR_INTERPOLATE, 80, gScenarioKernelData251_AFFINE_BILINEAR_INTERPOLATE, 4}
};

static acf_scenario_list gScenarioList_AFFINE_BILINEAR_INTERPOLATE = {
252, //number of scenarios
gScenarioArray_AFFINE_BILINEAR_INTERPOLATE};
//**************************************************************

class AFFINE_BILINEAR_INTERPOLATE : public ACF_Process_APU
{

public:
   AFFINE_BILINEAR_INTERPOLATE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AFFINE_BILINEAR_INTERPOLATE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AFFINE_BILINEAR_INTERPOLATE",
                                        AFFINE_BILINEAR_INTERPOLATE_LOAD_SEGMENTS,
                                        AFFINE_BILINEAR_INTERPOLATE_LOAD_PMEM, AFFINE_BILINEAR_INTERPOLATE_LOAD_PMEM_SIZE,
                                        AFFINE_BILINEAR_INTERPOLATE_LOAD_DMEM, AFFINE_BILINEAR_INTERPOLATE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AFFINE_BILINEAR_INTERPOLATE_APEX_LOG_BUFFER, AFFINE_BILINEAR_INTERPOLATE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("MATRIX", icp::DATATYPE_32S, 6, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("IMAGE_WIDTH", icp::DATATYPE_16S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("IMAGE_HEIGHT", icp::DATATYPE_16S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AFFINE_BILINEAR_INTERPOLATE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AFFINE_BILINEAR_INTERPOLATE

#ifndef _ACF_PROCESS_APU_MEAN
#define _ACF_PROCESS_APU_MEAN

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MEAN_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MEAN[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MEAN[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MEAN[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MEAN[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MEAN[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MEAN[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MEAN[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MEAN[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MEAN[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MEAN[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MEAN[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MEAN[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MEAN[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MEAN[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MEAN[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MEAN[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MEAN[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MEAN[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MEAN[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MEAN[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MEAN[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MEAN[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MEAN[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MEAN[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MEAN[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MEAN[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MEAN[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MEAN[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MEAN[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MEAN[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MEAN[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MEAN[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MEAN[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MEAN[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MEAN[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MEAN[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MEAN[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MEAN[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MEAN[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MEAN[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MEAN[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MEAN[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MEAN[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MEAN[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MEAN[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MEAN[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MEAN[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MEAN[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MEAN[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MEAN[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MEAN[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MEAN[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MEAN[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MEAN[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MEAN[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MEAN[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MEAN[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MEAN[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MEAN[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MEAN[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MEAN[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MEAN[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MEAN[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MEAN[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MEAN[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MEAN[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MEAN[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MEAN[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MEAN[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MEAN[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MEAN[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MEAN[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MEAN[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MEAN[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MEAN[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MEAN[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MEAN[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MEAN[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MEAN[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MEAN[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MEAN[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MEAN[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MEAN[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MEAN[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MEAN[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MEAN[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MEAN[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MEAN[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MEAN[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MEAN[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MEAN[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MEAN[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MEAN[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MEAN[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MEAN[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MEAN[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MEAN[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MEAN[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MEAN[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MEAN[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MEAN[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MEAN[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MEAN[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MEAN[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MEAN[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MEAN[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MEAN[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MEAN[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MEAN[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MEAN[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MEAN[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MEAN[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MEAN[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MEAN[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MEAN[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MEAN[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MEAN[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MEAN[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MEAN[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MEAN[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MEAN[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MEAN[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MEAN[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MEAN[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MEAN[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MEAN[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MEAN[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MEAN[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MEAN[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MEAN[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MEAN[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MEAN[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MEAN[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MEAN[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MEAN[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MEAN[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MEAN[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MEAN[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MEAN[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MEAN[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MEAN[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MEAN[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MEAN[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MEAN[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MEAN[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MEAN[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MEAN[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MEAN[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MEAN[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MEAN[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MEAN[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MEAN[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MEAN[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MEAN[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MEAN[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MEAN[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MEAN[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MEAN[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MEAN[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MEAN[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MEAN[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MEAN[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MEAN[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MEAN[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MEAN[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MEAN[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MEAN[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MEAN[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MEAN[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MEAN[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MEAN[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MEAN[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MEAN[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MEAN[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MEAN[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MEAN[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MEAN[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MEAN[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MEAN[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MEAN[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MEAN[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MEAN[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MEAN[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MEAN[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MEAN[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MEAN[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MEAN[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MEAN[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MEAN[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MEAN[] = {{112, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MEAN[] = {{120, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MEAN[] = {{128, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MEAN[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MEAN[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MEAN[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MEAN[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MEAN[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MEAN[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MEAN[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MEAN[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MEAN[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MEAN[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MEAN[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MEAN[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MEAN[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MEAN[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MEAN[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MEAN[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MEAN[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MEAN[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MEAN[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MEAN[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MEAN[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MEAN[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MEAN[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MEAN[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MEAN[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MEAN[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MEAN[] = {{88, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MEAN[] = {{96, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MEAN[] = {{104, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MEAN[] = {{112, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MEAN[] = {{120, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MEAN[] = {{128, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MEAN[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MEAN[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MEAN[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MEAN[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MEAN[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MEAN[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MEAN[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MEAN[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MEAN[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MEAN[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MEAN[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MEAN[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MEAN[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MEAN[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MEAN[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MEAN[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MEAN[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MEAN[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MEAN[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MEAN[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MEAN[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MEAN[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MEAN[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MEAN[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MEAN[] = {{72, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MEAN[] = {{80, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MEAN[] = {{88, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_MEAN[] = {{96, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_MEAN[] = {{104, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_MEAN[] = {{112, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_MEAN[] = {{120, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_MEAN[] = {{128, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_MEAN[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_MEAN[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_MEAN[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_MEAN[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_MEAN[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_MEAN[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_MEAN[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_MEAN[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_MEAN[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_MEAN[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_MEAN[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_MEAN[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_MEAN[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_MEAN[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_MEAN[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_MEAN[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_MEAN[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_MEAN[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_MEAN[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_MEAN[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_MEAN[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_MEAN[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_MEAN[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_MEAN[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_MEAN[] = {{72, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_MEAN[] = {{80, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_MEAN[] = {{88, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_MEAN[] = {{96, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_MEAN[] = {{104, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_MEAN[] = {{112, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_MEAN[] = {{120, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_MEAN[] = {{128, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_MEAN[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_MEAN[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_MEAN[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_MEAN[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_MEAN[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_MEAN[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_MEAN[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_MEAN[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_MEAN[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_MEAN[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_MEAN[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_MEAN[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_MEAN[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_MEAN[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_MEAN[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_MEAN[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_MEAN[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_MEAN[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_MEAN[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_MEAN[] = {{48, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_MEAN[] = {{52, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_MEAN[] = {{56, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_MEAN[] = {{60, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_MEAN[] = {{64, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_MEAN[] = {{72, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_MEAN[] = {{80, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_MEAN[] = {{88, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_MEAN[] = {{96, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_MEAN[] = {{104, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_MEAN[] = {{112, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_MEAN[] = {{120, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_MEAN[] = {{128, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_MEAN[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_MEAN[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_MEAN[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_MEAN[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_MEAN[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_MEAN[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_MEAN[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_MEAN[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_MEAN[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_MEAN[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_MEAN[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_MEAN[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_MEAN[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_MEAN[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_MEAN[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_MEAN[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_MEAN[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_MEAN[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_MEAN[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_MEAN[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_MEAN[] = {{52, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_MEAN[] = {{56, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_MEAN[] = {{60, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_MEAN[] = {{64, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_MEAN[] = {{72, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_MEAN[] = {{80, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_MEAN[] = {{88, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_MEAN[] = {{96, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_MEAN[] = {{104, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_MEAN[] = {{112, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_MEAN[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_MEAN[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_MEAN[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_MEAN[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_MEAN[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_MEAN[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_MEAN[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_MEAN[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_MEAN[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_MEAN[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_MEAN[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_MEAN[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_MEAN[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_MEAN[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_MEAN[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_MEAN[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_MEAN[] = {{36, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_MEAN[] = {{40, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_MEAN[] = {{44, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_MEAN[] = {{48, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_MEAN[] = {{52, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_MEAN[] = {{56, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_MEAN[] = {{60, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_MEAN[] = {{64, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_MEAN[] = {{72, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_MEAN[] = {{80, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_MEAN[] = {{88, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_MEAN[] = {{96, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_MEAN[] = {{104, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_MEAN[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_MEAN[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_MEAN[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_MEAN[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_MEAN[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_MEAN[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_MEAN[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_MEAN[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_MEAN[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_MEAN[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_MEAN[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_MEAN[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_MEAN[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_MEAN[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_MEAN[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_MEAN[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_MEAN[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_MEAN[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_MEAN[] = {{44, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_MEAN[] = {{48, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData399_MEAN[] = {{52, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData400_MEAN[] = {{56, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData401_MEAN[] = {{60, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData402_MEAN[] = {{64, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData403_MEAN[] = {{72, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData404_MEAN[] = {{80, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData405_MEAN[] = {{88, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData406_MEAN[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData407_MEAN[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData408_MEAN[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData409_MEAN[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData410_MEAN[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData411_MEAN[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData412_MEAN[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData413_MEAN[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData414_MEAN[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData415_MEAN[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData416_MEAN[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData417_MEAN[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData418_MEAN[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData419_MEAN[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData420_MEAN[] = {{30, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData421_MEAN[] = {{32, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData422_MEAN[] = {{36, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData423_MEAN[] = {{40, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData424_MEAN[] = {{44, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData425_MEAN[] = {{48, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData426_MEAN[] = {{52, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData427_MEAN[] = {{56, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData428_MEAN[] = {{60, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData429_MEAN[] = {{64, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData430_MEAN[] = {{72, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData431_MEAN[] = {{80, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData432_MEAN[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData433_MEAN[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData434_MEAN[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData435_MEAN[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData436_MEAN[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData437_MEAN[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData438_MEAN[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData439_MEAN[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData440_MEAN[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData441_MEAN[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData442_MEAN[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData443_MEAN[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData444_MEAN[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData445_MEAN[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData446_MEAN[] = {{30, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData447_MEAN[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData448_MEAN[] = {{36, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData449_MEAN[] = {{40, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData450_MEAN[] = {{44, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData451_MEAN[] = {{48, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData452_MEAN[] = {{52, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData453_MEAN[] = {{56, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData454_MEAN[] = {{60, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData455_MEAN[] = {{64, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData456_MEAN[] = {{72, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData457_MEAN[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData458_MEAN[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData459_MEAN[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData460_MEAN[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData461_MEAN[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData462_MEAN[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData463_MEAN[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData464_MEAN[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData465_MEAN[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData466_MEAN[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData467_MEAN[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData468_MEAN[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData469_MEAN[] = {{26, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData470_MEAN[] = {{28, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData471_MEAN[] = {{30, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData472_MEAN[] = {{32, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData473_MEAN[] = {{36, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData474_MEAN[] = {{40, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData475_MEAN[] = {{44, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData476_MEAN[] = {{48, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData477_MEAN[] = {{52, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData478_MEAN[] = {{56, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData479_MEAN[] = {{60, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData480_MEAN[] = {{64, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData481_MEAN[] = {{72, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData482_MEAN[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData483_MEAN[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData484_MEAN[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData485_MEAN[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData486_MEAN[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData487_MEAN[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData488_MEAN[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData489_MEAN[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData490_MEAN[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData491_MEAN[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData492_MEAN[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData493_MEAN[] = {{24, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData494_MEAN[] = {{26, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData495_MEAN[] = {{28, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData496_MEAN[] = {{30, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData497_MEAN[] = {{32, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData498_MEAN[] = {{36, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData499_MEAN[] = {{40, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData500_MEAN[] = {{44, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData501_MEAN[] = {{48, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData502_MEAN[] = {{52, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData503_MEAN[] = {{56, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData504_MEAN[] = {{60, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData505_MEAN[] = {{64, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData506_MEAN[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData507_MEAN[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData508_MEAN[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData509_MEAN[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData510_MEAN[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData511_MEAN[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData512_MEAN[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData513_MEAN[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData514_MEAN[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData515_MEAN[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData516_MEAN[] = {{22, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData517_MEAN[] = {{24, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData518_MEAN[] = {{26, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData519_MEAN[] = {{28, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData520_MEAN[] = {{30, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData521_MEAN[] = {{32, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData522_MEAN[] = {{36, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData523_MEAN[] = {{40, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData524_MEAN[] = {{44, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData525_MEAN[] = {{48, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData526_MEAN[] = {{52, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData527_MEAN[] = {{56, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData528_MEAN[] = {{60, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData529_MEAN[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData530_MEAN[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData531_MEAN[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData532_MEAN[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData533_MEAN[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData534_MEAN[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData535_MEAN[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData536_MEAN[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData537_MEAN[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData538_MEAN[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData539_MEAN[] = {{22, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData540_MEAN[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData541_MEAN[] = {{26, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData542_MEAN[] = {{28, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData543_MEAN[] = {{30, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData544_MEAN[] = {{32, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData545_MEAN[] = {{36, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData546_MEAN[] = {{40, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData547_MEAN[] = {{44, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData548_MEAN[] = {{48, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData549_MEAN[] = {{52, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData550_MEAN[] = {{56, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData399_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData400_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData401_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData402_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData403_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData404_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData405_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData406_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData407_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData408_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData409_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData410_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData411_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData412_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData413_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData414_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData415_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData416_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData417_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData418_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData419_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData420_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData421_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData422_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData423_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData424_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData425_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData426_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData427_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData428_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData429_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData430_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData431_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData432_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData433_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData434_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData435_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData436_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData437_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData438_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData439_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData440_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData441_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData442_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData443_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData444_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData445_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData446_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData447_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData448_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData449_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData450_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData451_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData452_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData453_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData454_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData455_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData456_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData457_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData458_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData459_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData460_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData461_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData462_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData463_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData464_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData465_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData466_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData467_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData468_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData469_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData470_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData471_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData472_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData473_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData474_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData475_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData476_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData477_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData478_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData479_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData480_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData481_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData482_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData483_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData484_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData485_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData486_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData487_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData488_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData489_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData490_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData491_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData492_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData493_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData494_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData495_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData496_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData497_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData498_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData499_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData500_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData501_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData502_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData503_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData504_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData505_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData506_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData507_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData508_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData509_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData510_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData511_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData512_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData513_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData514_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData515_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData516_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData517_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData518_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData519_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData520_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData521_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData522_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData523_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData524_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData525_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData526_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData527_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData528_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData529_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData530_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData531_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData532_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData533_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData534_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData535_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData536_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData537_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData538_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData539_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData540_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData541_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData542_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData543_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData544_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData545_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData546_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData547_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData548_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData549_MEAN[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData550_MEAN[] = {{0, 0}};

static acf_scenario gScenarioArray_MEAN[] = {
{2, 1, 8, 16, 0, gScenarioBufferData0_MEAN, 48, gScenarioKernelData0_MEAN, 4},
{4, 1, 12, 16, 0, gScenarioBufferData1_MEAN, 48, gScenarioKernelData1_MEAN, 4},
{6, 1, 16, 16, 0, gScenarioBufferData2_MEAN, 48, gScenarioKernelData2_MEAN, 4},
{8, 1, 20, 16, 0, gScenarioBufferData3_MEAN, 48, gScenarioKernelData3_MEAN, 4},
{10, 1, 24, 16, 0, gScenarioBufferData4_MEAN, 48, gScenarioKernelData4_MEAN, 4},
{12, 1, 28, 16, 0, gScenarioBufferData5_MEAN, 48, gScenarioKernelData5_MEAN, 4},
{14, 1, 32, 16, 0, gScenarioBufferData6_MEAN, 48, gScenarioKernelData6_MEAN, 4},
{16, 1, 36, 16, 0, gScenarioBufferData7_MEAN, 48, gScenarioKernelData7_MEAN, 4},
{18, 1, 40, 16, 0, gScenarioBufferData8_MEAN, 48, gScenarioKernelData8_MEAN, 4},
{20, 1, 44, 16, 0, gScenarioBufferData9_MEAN, 48, gScenarioKernelData9_MEAN, 4},
{22, 1, 48, 16, 0, gScenarioBufferData10_MEAN, 48, gScenarioKernelData10_MEAN, 4},
{24, 1, 52, 16, 0, gScenarioBufferData11_MEAN, 48, gScenarioKernelData11_MEAN, 4},
{26, 1, 56, 16, 0, gScenarioBufferData12_MEAN, 48, gScenarioKernelData12_MEAN, 4},
{28, 1, 60, 16, 0, gScenarioBufferData13_MEAN, 48, gScenarioKernelData13_MEAN, 4},
{30, 1, 64, 16, 0, gScenarioBufferData14_MEAN, 48, gScenarioKernelData14_MEAN, 4},
{32, 1, 68, 16, 0, gScenarioBufferData15_MEAN, 48, gScenarioKernelData15_MEAN, 4},
{36, 1, 76, 16, 0, gScenarioBufferData16_MEAN, 48, gScenarioKernelData16_MEAN, 4},
{40, 1, 84, 16, 0, gScenarioBufferData17_MEAN, 48, gScenarioKernelData17_MEAN, 4},
{44, 1, 92, 16, 0, gScenarioBufferData18_MEAN, 48, gScenarioKernelData18_MEAN, 4},
{48, 1, 100, 16, 0, gScenarioBufferData19_MEAN, 48, gScenarioKernelData19_MEAN, 4},
{52, 1, 108, 16, 0, gScenarioBufferData20_MEAN, 48, gScenarioKernelData20_MEAN, 4},
{56, 1, 116, 16, 0, gScenarioBufferData21_MEAN, 48, gScenarioKernelData21_MEAN, 4},
{60, 1, 124, 16, 0, gScenarioBufferData22_MEAN, 48, gScenarioKernelData22_MEAN, 4},
{64, 1, 132, 16, 0, gScenarioBufferData23_MEAN, 48, gScenarioKernelData23_MEAN, 4},
{72, 1, 148, 16, 0, gScenarioBufferData24_MEAN, 48, gScenarioKernelData24_MEAN, 4},
{80, 1, 164, 16, 0, gScenarioBufferData25_MEAN, 48, gScenarioKernelData25_MEAN, 4},
{88, 1, 180, 16, 0, gScenarioBufferData26_MEAN, 48, gScenarioKernelData26_MEAN, 4},
{96, 1, 196, 16, 0, gScenarioBufferData27_MEAN, 48, gScenarioKernelData27_MEAN, 4},
{104, 1, 212, 16, 0, gScenarioBufferData28_MEAN, 48, gScenarioKernelData28_MEAN, 4},
{112, 1, 228, 16, 0, gScenarioBufferData29_MEAN, 48, gScenarioKernelData29_MEAN, 4},
{120, 1, 244, 16, 0, gScenarioBufferData30_MEAN, 48, gScenarioKernelData30_MEAN, 4},
{128, 1, 260, 16, 0, gScenarioBufferData31_MEAN, 48, gScenarioKernelData31_MEAN, 4},
{2, 2, 12, 16, 0, gScenarioBufferData32_MEAN, 48, gScenarioKernelData32_MEAN, 4},
{4, 2, 20, 16, 0, gScenarioBufferData33_MEAN, 48, gScenarioKernelData33_MEAN, 4},
{6, 2, 28, 16, 0, gScenarioBufferData34_MEAN, 48, gScenarioKernelData34_MEAN, 4},
{8, 2, 36, 16, 0, gScenarioBufferData35_MEAN, 48, gScenarioKernelData35_MEAN, 4},
{10, 2, 44, 16, 0, gScenarioBufferData36_MEAN, 48, gScenarioKernelData36_MEAN, 4},
{12, 2, 52, 16, 0, gScenarioBufferData37_MEAN, 48, gScenarioKernelData37_MEAN, 4},
{14, 2, 60, 16, 0, gScenarioBufferData38_MEAN, 48, gScenarioKernelData38_MEAN, 4},
{16, 2, 68, 16, 0, gScenarioBufferData39_MEAN, 48, gScenarioKernelData39_MEAN, 4},
{18, 2, 76, 16, 0, gScenarioBufferData40_MEAN, 48, gScenarioKernelData40_MEAN, 4},
{20, 2, 84, 16, 0, gScenarioBufferData41_MEAN, 48, gScenarioKernelData41_MEAN, 4},
{22, 2, 92, 16, 0, gScenarioBufferData42_MEAN, 48, gScenarioKernelData42_MEAN, 4},
{24, 2, 100, 16, 0, gScenarioBufferData43_MEAN, 48, gScenarioKernelData43_MEAN, 4},
{26, 2, 108, 16, 0, gScenarioBufferData44_MEAN, 48, gScenarioKernelData44_MEAN, 4},
{28, 2, 116, 16, 0, gScenarioBufferData45_MEAN, 48, gScenarioKernelData45_MEAN, 4},
{30, 2, 124, 16, 0, gScenarioBufferData46_MEAN, 48, gScenarioKernelData46_MEAN, 4},
{32, 2, 132, 16, 0, gScenarioBufferData47_MEAN, 48, gScenarioKernelData47_MEAN, 4},
{36, 2, 148, 16, 0, gScenarioBufferData48_MEAN, 48, gScenarioKernelData48_MEAN, 4},
{40, 2, 164, 16, 0, gScenarioBufferData49_MEAN, 48, gScenarioKernelData49_MEAN, 4},
{44, 2, 180, 16, 0, gScenarioBufferData50_MEAN, 48, gScenarioKernelData50_MEAN, 4},
{48, 2, 196, 16, 0, gScenarioBufferData51_MEAN, 48, gScenarioKernelData51_MEAN, 4},
{52, 2, 212, 16, 0, gScenarioBufferData52_MEAN, 48, gScenarioKernelData52_MEAN, 4},
{56, 2, 228, 16, 0, gScenarioBufferData53_MEAN, 48, gScenarioKernelData53_MEAN, 4},
{60, 2, 244, 16, 0, gScenarioBufferData54_MEAN, 48, gScenarioKernelData54_MEAN, 4},
{64, 2, 260, 16, 0, gScenarioBufferData55_MEAN, 48, gScenarioKernelData55_MEAN, 4},
{72, 2, 292, 16, 0, gScenarioBufferData56_MEAN, 48, gScenarioKernelData56_MEAN, 4},
{80, 2, 324, 16, 0, gScenarioBufferData57_MEAN, 48, gScenarioKernelData57_MEAN, 4},
{88, 2, 356, 16, 0, gScenarioBufferData58_MEAN, 48, gScenarioKernelData58_MEAN, 4},
{96, 2, 388, 16, 0, gScenarioBufferData59_MEAN, 48, gScenarioKernelData59_MEAN, 4},
{104, 2, 420, 16, 0, gScenarioBufferData60_MEAN, 48, gScenarioKernelData60_MEAN, 4},
{112, 2, 452, 16, 0, gScenarioBufferData61_MEAN, 48, gScenarioKernelData61_MEAN, 4},
{120, 2, 484, 16, 0, gScenarioBufferData62_MEAN, 48, gScenarioKernelData62_MEAN, 4},
{128, 2, 516, 16, 0, gScenarioBufferData63_MEAN, 48, gScenarioKernelData63_MEAN, 4},
{2, 3, 16, 16, 0, gScenarioBufferData64_MEAN, 48, gScenarioKernelData64_MEAN, 4},
{4, 3, 28, 16, 0, gScenarioBufferData65_MEAN, 48, gScenarioKernelData65_MEAN, 4},
{6, 3, 40, 16, 0, gScenarioBufferData66_MEAN, 48, gScenarioKernelData66_MEAN, 4},
{8, 3, 52, 16, 0, gScenarioBufferData67_MEAN, 48, gScenarioKernelData67_MEAN, 4},
{10, 3, 64, 16, 0, gScenarioBufferData68_MEAN, 48, gScenarioKernelData68_MEAN, 4},
{12, 3, 76, 16, 0, gScenarioBufferData69_MEAN, 48, gScenarioKernelData69_MEAN, 4},
{14, 3, 88, 16, 0, gScenarioBufferData70_MEAN, 48, gScenarioKernelData70_MEAN, 4},
{16, 3, 100, 16, 0, gScenarioBufferData71_MEAN, 48, gScenarioKernelData71_MEAN, 4},
{18, 3, 112, 16, 0, gScenarioBufferData72_MEAN, 48, gScenarioKernelData72_MEAN, 4},
{20, 3, 124, 16, 0, gScenarioBufferData73_MEAN, 48, gScenarioKernelData73_MEAN, 4},
{22, 3, 136, 16, 0, gScenarioBufferData74_MEAN, 48, gScenarioKernelData74_MEAN, 4},
{24, 3, 148, 16, 0, gScenarioBufferData75_MEAN, 48, gScenarioKernelData75_MEAN, 4},
{26, 3, 160, 16, 0, gScenarioBufferData76_MEAN, 48, gScenarioKernelData76_MEAN, 4},
{28, 3, 172, 16, 0, gScenarioBufferData77_MEAN, 48, gScenarioKernelData77_MEAN, 4},
{30, 3, 184, 16, 0, gScenarioBufferData78_MEAN, 48, gScenarioKernelData78_MEAN, 4},
{32, 3, 196, 16, 0, gScenarioBufferData79_MEAN, 48, gScenarioKernelData79_MEAN, 4},
{36, 3, 220, 16, 0, gScenarioBufferData80_MEAN, 48, gScenarioKernelData80_MEAN, 4},
{40, 3, 244, 16, 0, gScenarioBufferData81_MEAN, 48, gScenarioKernelData81_MEAN, 4},
{44, 3, 268, 16, 0, gScenarioBufferData82_MEAN, 48, gScenarioKernelData82_MEAN, 4},
{48, 3, 292, 16, 0, gScenarioBufferData83_MEAN, 48, gScenarioKernelData83_MEAN, 4},
{52, 3, 316, 16, 0, gScenarioBufferData84_MEAN, 48, gScenarioKernelData84_MEAN, 4},
{56, 3, 340, 16, 0, gScenarioBufferData85_MEAN, 48, gScenarioKernelData85_MEAN, 4},
{60, 3, 364, 16, 0, gScenarioBufferData86_MEAN, 48, gScenarioKernelData86_MEAN, 4},
{64, 3, 388, 16, 0, gScenarioBufferData87_MEAN, 48, gScenarioKernelData87_MEAN, 4},
{72, 3, 436, 16, 0, gScenarioBufferData88_MEAN, 48, gScenarioKernelData88_MEAN, 4},
{80, 3, 484, 16, 0, gScenarioBufferData89_MEAN, 48, gScenarioKernelData89_MEAN, 4},
{88, 3, 532, 16, 0, gScenarioBufferData90_MEAN, 48, gScenarioKernelData90_MEAN, 4},
{96, 3, 580, 16, 0, gScenarioBufferData91_MEAN, 48, gScenarioKernelData91_MEAN, 4},
{104, 3, 628, 16, 0, gScenarioBufferData92_MEAN, 48, gScenarioKernelData92_MEAN, 4},
{112, 3, 676, 16, 0, gScenarioBufferData93_MEAN, 48, gScenarioKernelData93_MEAN, 4},
{120, 3, 724, 16, 0, gScenarioBufferData94_MEAN, 48, gScenarioKernelData94_MEAN, 4},
{128, 3, 772, 16, 0, gScenarioBufferData95_MEAN, 48, gScenarioKernelData95_MEAN, 4},
{2, 4, 20, 16, 0, gScenarioBufferData96_MEAN, 48, gScenarioKernelData96_MEAN, 4},
{4, 4, 36, 16, 0, gScenarioBufferData97_MEAN, 48, gScenarioKernelData97_MEAN, 4},
{6, 4, 52, 16, 0, gScenarioBufferData98_MEAN, 48, gScenarioKernelData98_MEAN, 4},
{8, 4, 68, 16, 0, gScenarioBufferData99_MEAN, 48, gScenarioKernelData99_MEAN, 4},
{10, 4, 84, 16, 0, gScenarioBufferData100_MEAN, 48, gScenarioKernelData100_MEAN, 4},
{12, 4, 100, 16, 0, gScenarioBufferData101_MEAN, 48, gScenarioKernelData101_MEAN, 4},
{14, 4, 116, 16, 0, gScenarioBufferData102_MEAN, 48, gScenarioKernelData102_MEAN, 4},
{16, 4, 132, 16, 0, gScenarioBufferData103_MEAN, 48, gScenarioKernelData103_MEAN, 4},
{18, 4, 148, 16, 0, gScenarioBufferData104_MEAN, 48, gScenarioKernelData104_MEAN, 4},
{20, 4, 164, 16, 0, gScenarioBufferData105_MEAN, 48, gScenarioKernelData105_MEAN, 4},
{22, 4, 180, 16, 0, gScenarioBufferData106_MEAN, 48, gScenarioKernelData106_MEAN, 4},
{24, 4, 196, 16, 0, gScenarioBufferData107_MEAN, 48, gScenarioKernelData107_MEAN, 4},
{26, 4, 212, 16, 0, gScenarioBufferData108_MEAN, 48, gScenarioKernelData108_MEAN, 4},
{28, 4, 228, 16, 0, gScenarioBufferData109_MEAN, 48, gScenarioKernelData109_MEAN, 4},
{30, 4, 244, 16, 0, gScenarioBufferData110_MEAN, 48, gScenarioKernelData110_MEAN, 4},
{32, 4, 260, 16, 0, gScenarioBufferData111_MEAN, 48, gScenarioKernelData111_MEAN, 4},
{36, 4, 292, 16, 0, gScenarioBufferData112_MEAN, 48, gScenarioKernelData112_MEAN, 4},
{40, 4, 324, 16, 0, gScenarioBufferData113_MEAN, 48, gScenarioKernelData113_MEAN, 4},
{44, 4, 356, 16, 0, gScenarioBufferData114_MEAN, 48, gScenarioKernelData114_MEAN, 4},
{48, 4, 388, 16, 0, gScenarioBufferData115_MEAN, 48, gScenarioKernelData115_MEAN, 4},
{52, 4, 420, 16, 0, gScenarioBufferData116_MEAN, 48, gScenarioKernelData116_MEAN, 4},
{56, 4, 452, 16, 0, gScenarioBufferData117_MEAN, 48, gScenarioKernelData117_MEAN, 4},
{60, 4, 484, 16, 0, gScenarioBufferData118_MEAN, 48, gScenarioKernelData118_MEAN, 4},
{64, 4, 516, 16, 0, gScenarioBufferData119_MEAN, 48, gScenarioKernelData119_MEAN, 4},
{72, 4, 580, 16, 0, gScenarioBufferData120_MEAN, 48, gScenarioKernelData120_MEAN, 4},
{80, 4, 644, 16, 0, gScenarioBufferData121_MEAN, 48, gScenarioKernelData121_MEAN, 4},
{88, 4, 708, 16, 0, gScenarioBufferData122_MEAN, 48, gScenarioKernelData122_MEAN, 4},
{96, 4, 772, 16, 0, gScenarioBufferData123_MEAN, 48, gScenarioKernelData123_MEAN, 4},
{104, 4, 836, 16, 0, gScenarioBufferData124_MEAN, 48, gScenarioKernelData124_MEAN, 4},
{112, 4, 900, 16, 0, gScenarioBufferData125_MEAN, 48, gScenarioKernelData125_MEAN, 4},
{120, 4, 964, 16, 0, gScenarioBufferData126_MEAN, 48, gScenarioKernelData126_MEAN, 4},
{128, 4, 1028, 16, 0, gScenarioBufferData127_MEAN, 48, gScenarioKernelData127_MEAN, 4},
{2, 5, 24, 16, 0, gScenarioBufferData128_MEAN, 48, gScenarioKernelData128_MEAN, 4},
{4, 5, 44, 16, 0, gScenarioBufferData129_MEAN, 48, gScenarioKernelData129_MEAN, 4},
{6, 5, 64, 16, 0, gScenarioBufferData130_MEAN, 48, gScenarioKernelData130_MEAN, 4},
{8, 5, 84, 16, 0, gScenarioBufferData131_MEAN, 48, gScenarioKernelData131_MEAN, 4},
{10, 5, 104, 16, 0, gScenarioBufferData132_MEAN, 48, gScenarioKernelData132_MEAN, 4},
{12, 5, 124, 16, 0, gScenarioBufferData133_MEAN, 48, gScenarioKernelData133_MEAN, 4},
{14, 5, 144, 16, 0, gScenarioBufferData134_MEAN, 48, gScenarioKernelData134_MEAN, 4},
{16, 5, 164, 16, 0, gScenarioBufferData135_MEAN, 48, gScenarioKernelData135_MEAN, 4},
{18, 5, 184, 16, 0, gScenarioBufferData136_MEAN, 48, gScenarioKernelData136_MEAN, 4},
{20, 5, 204, 16, 0, gScenarioBufferData137_MEAN, 48, gScenarioKernelData137_MEAN, 4},
{22, 5, 224, 16, 0, gScenarioBufferData138_MEAN, 48, gScenarioKernelData138_MEAN, 4},
{24, 5, 244, 16, 0, gScenarioBufferData139_MEAN, 48, gScenarioKernelData139_MEAN, 4},
{26, 5, 264, 16, 0, gScenarioBufferData140_MEAN, 48, gScenarioKernelData140_MEAN, 4},
{28, 5, 284, 16, 0, gScenarioBufferData141_MEAN, 48, gScenarioKernelData141_MEAN, 4},
{30, 5, 304, 16, 0, gScenarioBufferData142_MEAN, 48, gScenarioKernelData142_MEAN, 4},
{32, 5, 324, 16, 0, gScenarioBufferData143_MEAN, 48, gScenarioKernelData143_MEAN, 4},
{36, 5, 364, 16, 0, gScenarioBufferData144_MEAN, 48, gScenarioKernelData144_MEAN, 4},
{40, 5, 404, 16, 0, gScenarioBufferData145_MEAN, 48, gScenarioKernelData145_MEAN, 4},
{44, 5, 444, 16, 0, gScenarioBufferData146_MEAN, 48, gScenarioKernelData146_MEAN, 4},
{48, 5, 484, 16, 0, gScenarioBufferData147_MEAN, 48, gScenarioKernelData147_MEAN, 4},
{52, 5, 524, 16, 0, gScenarioBufferData148_MEAN, 48, gScenarioKernelData148_MEAN, 4},
{56, 5, 564, 16, 0, gScenarioBufferData149_MEAN, 48, gScenarioKernelData149_MEAN, 4},
{60, 5, 604, 16, 0, gScenarioBufferData150_MEAN, 48, gScenarioKernelData150_MEAN, 4},
{64, 5, 644, 16, 0, gScenarioBufferData151_MEAN, 48, gScenarioKernelData151_MEAN, 4},
{72, 5, 724, 16, 0, gScenarioBufferData152_MEAN, 48, gScenarioKernelData152_MEAN, 4},
{80, 5, 804, 16, 0, gScenarioBufferData153_MEAN, 48, gScenarioKernelData153_MEAN, 4},
{88, 5, 884, 16, 0, gScenarioBufferData154_MEAN, 48, gScenarioKernelData154_MEAN, 4},
{96, 5, 964, 16, 0, gScenarioBufferData155_MEAN, 48, gScenarioKernelData155_MEAN, 4},
{104, 5, 1044, 16, 0, gScenarioBufferData156_MEAN, 48, gScenarioKernelData156_MEAN, 4},
{112, 5, 1124, 16, 0, gScenarioBufferData157_MEAN, 48, gScenarioKernelData157_MEAN, 4},
{120, 5, 1204, 16, 0, gScenarioBufferData158_MEAN, 48, gScenarioKernelData158_MEAN, 4},
{128, 5, 1284, 16, 0, gScenarioBufferData159_MEAN, 48, gScenarioKernelData159_MEAN, 4},
{2, 6, 28, 16, 0, gScenarioBufferData160_MEAN, 48, gScenarioKernelData160_MEAN, 4},
{4, 6, 52, 16, 0, gScenarioBufferData161_MEAN, 48, gScenarioKernelData161_MEAN, 4},
{6, 6, 76, 16, 0, gScenarioBufferData162_MEAN, 48, gScenarioKernelData162_MEAN, 4},
{8, 6, 100, 16, 0, gScenarioBufferData163_MEAN, 48, gScenarioKernelData163_MEAN, 4},
{10, 6, 124, 16, 0, gScenarioBufferData164_MEAN, 48, gScenarioKernelData164_MEAN, 4},
{12, 6, 148, 16, 0, gScenarioBufferData165_MEAN, 48, gScenarioKernelData165_MEAN, 4},
{14, 6, 172, 16, 0, gScenarioBufferData166_MEAN, 48, gScenarioKernelData166_MEAN, 4},
{16, 6, 196, 16, 0, gScenarioBufferData167_MEAN, 48, gScenarioKernelData167_MEAN, 4},
{18, 6, 220, 16, 0, gScenarioBufferData168_MEAN, 48, gScenarioKernelData168_MEAN, 4},
{20, 6, 244, 16, 0, gScenarioBufferData169_MEAN, 48, gScenarioKernelData169_MEAN, 4},
{22, 6, 268, 16, 0, gScenarioBufferData170_MEAN, 48, gScenarioKernelData170_MEAN, 4},
{24, 6, 292, 16, 0, gScenarioBufferData171_MEAN, 48, gScenarioKernelData171_MEAN, 4},
{26, 6, 316, 16, 0, gScenarioBufferData172_MEAN, 48, gScenarioKernelData172_MEAN, 4},
{28, 6, 340, 16, 0, gScenarioBufferData173_MEAN, 48, gScenarioKernelData173_MEAN, 4},
{30, 6, 364, 16, 0, gScenarioBufferData174_MEAN, 48, gScenarioKernelData174_MEAN, 4},
{32, 6, 388, 16, 0, gScenarioBufferData175_MEAN, 48, gScenarioKernelData175_MEAN, 4},
{36, 6, 436, 16, 0, gScenarioBufferData176_MEAN, 48, gScenarioKernelData176_MEAN, 4},
{40, 6, 484, 16, 0, gScenarioBufferData177_MEAN, 48, gScenarioKernelData177_MEAN, 4},
{44, 6, 532, 16, 0, gScenarioBufferData178_MEAN, 48, gScenarioKernelData178_MEAN, 4},
{48, 6, 580, 16, 0, gScenarioBufferData179_MEAN, 48, gScenarioKernelData179_MEAN, 4},
{52, 6, 628, 16, 0, gScenarioBufferData180_MEAN, 48, gScenarioKernelData180_MEAN, 4},
{56, 6, 676, 16, 0, gScenarioBufferData181_MEAN, 48, gScenarioKernelData181_MEAN, 4},
{60, 6, 724, 16, 0, gScenarioBufferData182_MEAN, 48, gScenarioKernelData182_MEAN, 4},
{64, 6, 772, 16, 0, gScenarioBufferData183_MEAN, 48, gScenarioKernelData183_MEAN, 4},
{72, 6, 868, 16, 0, gScenarioBufferData184_MEAN, 48, gScenarioKernelData184_MEAN, 4},
{80, 6, 964, 16, 0, gScenarioBufferData185_MEAN, 48, gScenarioKernelData185_MEAN, 4},
{88, 6, 1060, 16, 0, gScenarioBufferData186_MEAN, 48, gScenarioKernelData186_MEAN, 4},
{96, 6, 1156, 16, 0, gScenarioBufferData187_MEAN, 48, gScenarioKernelData187_MEAN, 4},
{104, 6, 1252, 16, 0, gScenarioBufferData188_MEAN, 48, gScenarioKernelData188_MEAN, 4},
{112, 6, 1348, 16, 0, gScenarioBufferData189_MEAN, 48, gScenarioKernelData189_MEAN, 4},
{120, 6, 1444, 16, 0, gScenarioBufferData190_MEAN, 48, gScenarioKernelData190_MEAN, 4},
{128, 6, 1540, 16, 0, gScenarioBufferData191_MEAN, 48, gScenarioKernelData191_MEAN, 4},
{2, 8, 36, 16, 0, gScenarioBufferData192_MEAN, 48, gScenarioKernelData192_MEAN, 4},
{4, 8, 68, 16, 0, gScenarioBufferData193_MEAN, 48, gScenarioKernelData193_MEAN, 4},
{6, 8, 100, 16, 0, gScenarioBufferData194_MEAN, 48, gScenarioKernelData194_MEAN, 4},
{8, 8, 132, 16, 0, gScenarioBufferData195_MEAN, 48, gScenarioKernelData195_MEAN, 4},
{10, 8, 164, 16, 0, gScenarioBufferData196_MEAN, 48, gScenarioKernelData196_MEAN, 4},
{12, 8, 196, 16, 0, gScenarioBufferData197_MEAN, 48, gScenarioKernelData197_MEAN, 4},
{14, 8, 228, 16, 0, gScenarioBufferData198_MEAN, 48, gScenarioKernelData198_MEAN, 4},
{16, 8, 260, 16, 0, gScenarioBufferData199_MEAN, 48, gScenarioKernelData199_MEAN, 4},
{18, 8, 292, 16, 0, gScenarioBufferData200_MEAN, 48, gScenarioKernelData200_MEAN, 4},
{20, 8, 324, 16, 0, gScenarioBufferData201_MEAN, 48, gScenarioKernelData201_MEAN, 4},
{22, 8, 356, 16, 0, gScenarioBufferData202_MEAN, 48, gScenarioKernelData202_MEAN, 4},
{24, 8, 388, 16, 0, gScenarioBufferData203_MEAN, 48, gScenarioKernelData203_MEAN, 4},
{26, 8, 420, 16, 0, gScenarioBufferData204_MEAN, 48, gScenarioKernelData204_MEAN, 4},
{28, 8, 452, 16, 0, gScenarioBufferData205_MEAN, 48, gScenarioKernelData205_MEAN, 4},
{30, 8, 484, 16, 0, gScenarioBufferData206_MEAN, 48, gScenarioKernelData206_MEAN, 4},
{32, 8, 516, 16, 0, gScenarioBufferData207_MEAN, 48, gScenarioKernelData207_MEAN, 4},
{36, 8, 580, 16, 0, gScenarioBufferData208_MEAN, 48, gScenarioKernelData208_MEAN, 4},
{40, 8, 644, 16, 0, gScenarioBufferData209_MEAN, 48, gScenarioKernelData209_MEAN, 4},
{44, 8, 708, 16, 0, gScenarioBufferData210_MEAN, 48, gScenarioKernelData210_MEAN, 4},
{48, 8, 772, 16, 0, gScenarioBufferData211_MEAN, 48, gScenarioKernelData211_MEAN, 4},
{52, 8, 836, 16, 0, gScenarioBufferData212_MEAN, 48, gScenarioKernelData212_MEAN, 4},
{56, 8, 900, 16, 0, gScenarioBufferData213_MEAN, 48, gScenarioKernelData213_MEAN, 4},
{60, 8, 964, 16, 0, gScenarioBufferData214_MEAN, 48, gScenarioKernelData214_MEAN, 4},
{64, 8, 1028, 16, 0, gScenarioBufferData215_MEAN, 48, gScenarioKernelData215_MEAN, 4},
{72, 8, 1156, 16, 0, gScenarioBufferData216_MEAN, 48, gScenarioKernelData216_MEAN, 4},
{80, 8, 1284, 16, 0, gScenarioBufferData217_MEAN, 48, gScenarioKernelData217_MEAN, 4},
{88, 8, 1412, 16, 0, gScenarioBufferData218_MEAN, 48, gScenarioKernelData218_MEAN, 4},
{96, 8, 1540, 16, 0, gScenarioBufferData219_MEAN, 48, gScenarioKernelData219_MEAN, 4},
{104, 8, 1668, 16, 0, gScenarioBufferData220_MEAN, 48, gScenarioKernelData220_MEAN, 4},
{112, 8, 1796, 16, 0, gScenarioBufferData221_MEAN, 48, gScenarioKernelData221_MEAN, 4},
{120, 8, 1924, 16, 0, gScenarioBufferData222_MEAN, 48, gScenarioKernelData222_MEAN, 4},
{128, 8, 2052, 16, 0, gScenarioBufferData223_MEAN, 48, gScenarioKernelData223_MEAN, 4},
{2, 10, 44, 16, 0, gScenarioBufferData224_MEAN, 48, gScenarioKernelData224_MEAN, 4},
{4, 10, 84, 16, 0, gScenarioBufferData225_MEAN, 48, gScenarioKernelData225_MEAN, 4},
{6, 10, 124, 16, 0, gScenarioBufferData226_MEAN, 48, gScenarioKernelData226_MEAN, 4},
{8, 10, 164, 16, 0, gScenarioBufferData227_MEAN, 48, gScenarioKernelData227_MEAN, 4},
{10, 10, 204, 16, 0, gScenarioBufferData228_MEAN, 48, gScenarioKernelData228_MEAN, 4},
{12, 10, 244, 16, 0, gScenarioBufferData229_MEAN, 48, gScenarioKernelData229_MEAN, 4},
{14, 10, 284, 16, 0, gScenarioBufferData230_MEAN, 48, gScenarioKernelData230_MEAN, 4},
{16, 10, 324, 16, 0, gScenarioBufferData231_MEAN, 48, gScenarioKernelData231_MEAN, 4},
{18, 10, 364, 16, 0, gScenarioBufferData232_MEAN, 48, gScenarioKernelData232_MEAN, 4},
{20, 10, 404, 16, 0, gScenarioBufferData233_MEAN, 48, gScenarioKernelData233_MEAN, 4},
{22, 10, 444, 16, 0, gScenarioBufferData234_MEAN, 48, gScenarioKernelData234_MEAN, 4},
{24, 10, 484, 16, 0, gScenarioBufferData235_MEAN, 48, gScenarioKernelData235_MEAN, 4},
{26, 10, 524, 16, 0, gScenarioBufferData236_MEAN, 48, gScenarioKernelData236_MEAN, 4},
{28, 10, 564, 16, 0, gScenarioBufferData237_MEAN, 48, gScenarioKernelData237_MEAN, 4},
{30, 10, 604, 16, 0, gScenarioBufferData238_MEAN, 48, gScenarioKernelData238_MEAN, 4},
{32, 10, 644, 16, 0, gScenarioBufferData239_MEAN, 48, gScenarioKernelData239_MEAN, 4},
{36, 10, 724, 16, 0, gScenarioBufferData240_MEAN, 48, gScenarioKernelData240_MEAN, 4},
{40, 10, 804, 16, 0, gScenarioBufferData241_MEAN, 48, gScenarioKernelData241_MEAN, 4},
{44, 10, 884, 16, 0, gScenarioBufferData242_MEAN, 48, gScenarioKernelData242_MEAN, 4},
{48, 10, 964, 16, 0, gScenarioBufferData243_MEAN, 48, gScenarioKernelData243_MEAN, 4},
{52, 10, 1044, 16, 0, gScenarioBufferData244_MEAN, 48, gScenarioKernelData244_MEAN, 4},
{56, 10, 1124, 16, 0, gScenarioBufferData245_MEAN, 48, gScenarioKernelData245_MEAN, 4},
{60, 10, 1204, 16, 0, gScenarioBufferData246_MEAN, 48, gScenarioKernelData246_MEAN, 4},
{64, 10, 1284, 16, 0, gScenarioBufferData247_MEAN, 48, gScenarioKernelData247_MEAN, 4},
{72, 10, 1444, 16, 0, gScenarioBufferData248_MEAN, 48, gScenarioKernelData248_MEAN, 4},
{80, 10, 1604, 16, 0, gScenarioBufferData249_MEAN, 48, gScenarioKernelData249_MEAN, 4},
{88, 10, 1764, 16, 0, gScenarioBufferData250_MEAN, 48, gScenarioKernelData250_MEAN, 4},
{96, 10, 1924, 16, 0, gScenarioBufferData251_MEAN, 48, gScenarioKernelData251_MEAN, 4},
{104, 10, 2084, 16, 0, gScenarioBufferData252_MEAN, 48, gScenarioKernelData252_MEAN, 4},
{112, 10, 2244, 16, 0, gScenarioBufferData253_MEAN, 48, gScenarioKernelData253_MEAN, 4},
{120, 10, 2404, 16, 0, gScenarioBufferData254_MEAN, 48, gScenarioKernelData254_MEAN, 4},
{128, 10, 2564, 16, 0, gScenarioBufferData255_MEAN, 48, gScenarioKernelData255_MEAN, 4},
{2, 12, 52, 16, 0, gScenarioBufferData256_MEAN, 48, gScenarioKernelData256_MEAN, 4},
{4, 12, 100, 16, 0, gScenarioBufferData257_MEAN, 48, gScenarioKernelData257_MEAN, 4},
{6, 12, 148, 16, 0, gScenarioBufferData258_MEAN, 48, gScenarioKernelData258_MEAN, 4},
{8, 12, 196, 16, 0, gScenarioBufferData259_MEAN, 48, gScenarioKernelData259_MEAN, 4},
{10, 12, 244, 16, 0, gScenarioBufferData260_MEAN, 48, gScenarioKernelData260_MEAN, 4},
{12, 12, 292, 16, 0, gScenarioBufferData261_MEAN, 48, gScenarioKernelData261_MEAN, 4},
{14, 12, 340, 16, 0, gScenarioBufferData262_MEAN, 48, gScenarioKernelData262_MEAN, 4},
{16, 12, 388, 16, 0, gScenarioBufferData263_MEAN, 48, gScenarioKernelData263_MEAN, 4},
{18, 12, 436, 16, 0, gScenarioBufferData264_MEAN, 48, gScenarioKernelData264_MEAN, 4},
{20, 12, 484, 16, 0, gScenarioBufferData265_MEAN, 48, gScenarioKernelData265_MEAN, 4},
{22, 12, 532, 16, 0, gScenarioBufferData266_MEAN, 48, gScenarioKernelData266_MEAN, 4},
{24, 12, 580, 16, 0, gScenarioBufferData267_MEAN, 48, gScenarioKernelData267_MEAN, 4},
{26, 12, 628, 16, 0, gScenarioBufferData268_MEAN, 48, gScenarioKernelData268_MEAN, 4},
{28, 12, 676, 16, 0, gScenarioBufferData269_MEAN, 48, gScenarioKernelData269_MEAN, 4},
{30, 12, 724, 16, 0, gScenarioBufferData270_MEAN, 48, gScenarioKernelData270_MEAN, 4},
{32, 12, 772, 16, 0, gScenarioBufferData271_MEAN, 48, gScenarioKernelData271_MEAN, 4},
{36, 12, 868, 16, 0, gScenarioBufferData272_MEAN, 48, gScenarioKernelData272_MEAN, 4},
{40, 12, 964, 16, 0, gScenarioBufferData273_MEAN, 48, gScenarioKernelData273_MEAN, 4},
{44, 12, 1060, 16, 0, gScenarioBufferData274_MEAN, 48, gScenarioKernelData274_MEAN, 4},
{48, 12, 1156, 16, 0, gScenarioBufferData275_MEAN, 48, gScenarioKernelData275_MEAN, 4},
{52, 12, 1252, 16, 0, gScenarioBufferData276_MEAN, 48, gScenarioKernelData276_MEAN, 4},
{56, 12, 1348, 16, 0, gScenarioBufferData277_MEAN, 48, gScenarioKernelData277_MEAN, 4},
{60, 12, 1444, 16, 0, gScenarioBufferData278_MEAN, 48, gScenarioKernelData278_MEAN, 4},
{64, 12, 1540, 16, 0, gScenarioBufferData279_MEAN, 48, gScenarioKernelData279_MEAN, 4},
{72, 12, 1732, 16, 0, gScenarioBufferData280_MEAN, 48, gScenarioKernelData280_MEAN, 4},
{80, 12, 1924, 16, 0, gScenarioBufferData281_MEAN, 48, gScenarioKernelData281_MEAN, 4},
{88, 12, 2116, 16, 0, gScenarioBufferData282_MEAN, 48, gScenarioKernelData282_MEAN, 4},
{96, 12, 2308, 16, 0, gScenarioBufferData283_MEAN, 48, gScenarioKernelData283_MEAN, 4},
{104, 12, 2500, 16, 0, gScenarioBufferData284_MEAN, 48, gScenarioKernelData284_MEAN, 4},
{112, 12, 2692, 16, 0, gScenarioBufferData285_MEAN, 48, gScenarioKernelData285_MEAN, 4},
{120, 12, 2884, 16, 0, gScenarioBufferData286_MEAN, 48, gScenarioKernelData286_MEAN, 4},
{128, 12, 3076, 16, 0, gScenarioBufferData287_MEAN, 48, gScenarioKernelData287_MEAN, 4},
{2, 14, 60, 16, 0, gScenarioBufferData288_MEAN, 48, gScenarioKernelData288_MEAN, 4},
{4, 14, 116, 16, 0, gScenarioBufferData289_MEAN, 48, gScenarioKernelData289_MEAN, 4},
{6, 14, 172, 16, 0, gScenarioBufferData290_MEAN, 48, gScenarioKernelData290_MEAN, 4},
{8, 14, 228, 16, 0, gScenarioBufferData291_MEAN, 48, gScenarioKernelData291_MEAN, 4},
{10, 14, 284, 16, 0, gScenarioBufferData292_MEAN, 48, gScenarioKernelData292_MEAN, 4},
{12, 14, 340, 16, 0, gScenarioBufferData293_MEAN, 48, gScenarioKernelData293_MEAN, 4},
{14, 14, 396, 16, 0, gScenarioBufferData294_MEAN, 48, gScenarioKernelData294_MEAN, 4},
{16, 14, 452, 16, 0, gScenarioBufferData295_MEAN, 48, gScenarioKernelData295_MEAN, 4},
{18, 14, 508, 16, 0, gScenarioBufferData296_MEAN, 48, gScenarioKernelData296_MEAN, 4},
{20, 14, 564, 16, 0, gScenarioBufferData297_MEAN, 48, gScenarioKernelData297_MEAN, 4},
{22, 14, 620, 16, 0, gScenarioBufferData298_MEAN, 48, gScenarioKernelData298_MEAN, 4},
{24, 14, 676, 16, 0, gScenarioBufferData299_MEAN, 48, gScenarioKernelData299_MEAN, 4},
{26, 14, 732, 16, 0, gScenarioBufferData300_MEAN, 48, gScenarioKernelData300_MEAN, 4},
{28, 14, 788, 16, 0, gScenarioBufferData301_MEAN, 48, gScenarioKernelData301_MEAN, 4},
{30, 14, 844, 16, 0, gScenarioBufferData302_MEAN, 48, gScenarioKernelData302_MEAN, 4},
{32, 14, 900, 16, 0, gScenarioBufferData303_MEAN, 48, gScenarioKernelData303_MEAN, 4},
{36, 14, 1012, 16, 0, gScenarioBufferData304_MEAN, 48, gScenarioKernelData304_MEAN, 4},
{40, 14, 1124, 16, 0, gScenarioBufferData305_MEAN, 48, gScenarioKernelData305_MEAN, 4},
{44, 14, 1236, 16, 0, gScenarioBufferData306_MEAN, 48, gScenarioKernelData306_MEAN, 4},
{48, 14, 1348, 16, 0, gScenarioBufferData307_MEAN, 48, gScenarioKernelData307_MEAN, 4},
{52, 14, 1460, 16, 0, gScenarioBufferData308_MEAN, 48, gScenarioKernelData308_MEAN, 4},
{56, 14, 1572, 16, 0, gScenarioBufferData309_MEAN, 48, gScenarioKernelData309_MEAN, 4},
{60, 14, 1684, 16, 0, gScenarioBufferData310_MEAN, 48, gScenarioKernelData310_MEAN, 4},
{64, 14, 1796, 16, 0, gScenarioBufferData311_MEAN, 48, gScenarioKernelData311_MEAN, 4},
{72, 14, 2020, 16, 0, gScenarioBufferData312_MEAN, 48, gScenarioKernelData312_MEAN, 4},
{80, 14, 2244, 16, 0, gScenarioBufferData313_MEAN, 48, gScenarioKernelData313_MEAN, 4},
{88, 14, 2468, 16, 0, gScenarioBufferData314_MEAN, 48, gScenarioKernelData314_MEAN, 4},
{96, 14, 2692, 16, 0, gScenarioBufferData315_MEAN, 48, gScenarioKernelData315_MEAN, 4},
{104, 14, 2916, 16, 0, gScenarioBufferData316_MEAN, 48, gScenarioKernelData316_MEAN, 4},
{112, 14, 3140, 16, 0, gScenarioBufferData317_MEAN, 48, gScenarioKernelData317_MEAN, 4},
{120, 14, 3364, 16, 0, gScenarioBufferData318_MEAN, 48, gScenarioKernelData318_MEAN, 4},
{128, 14, 3588, 16, 0, gScenarioBufferData319_MEAN, 48, gScenarioKernelData319_MEAN, 4},
{2, 16, 68, 16, 0, gScenarioBufferData320_MEAN, 48, gScenarioKernelData320_MEAN, 4},
{4, 16, 132, 16, 0, gScenarioBufferData321_MEAN, 48, gScenarioKernelData321_MEAN, 4},
{6, 16, 196, 16, 0, gScenarioBufferData322_MEAN, 48, gScenarioKernelData322_MEAN, 4},
{8, 16, 260, 16, 0, gScenarioBufferData323_MEAN, 48, gScenarioKernelData323_MEAN, 4},
{10, 16, 324, 16, 0, gScenarioBufferData324_MEAN, 48, gScenarioKernelData324_MEAN, 4},
{12, 16, 388, 16, 0, gScenarioBufferData325_MEAN, 48, gScenarioKernelData325_MEAN, 4},
{14, 16, 452, 16, 0, gScenarioBufferData326_MEAN, 48, gScenarioKernelData326_MEAN, 4},
{16, 16, 516, 16, 0, gScenarioBufferData327_MEAN, 48, gScenarioKernelData327_MEAN, 4},
{18, 16, 580, 16, 0, gScenarioBufferData328_MEAN, 48, gScenarioKernelData328_MEAN, 4},
{20, 16, 644, 16, 0, gScenarioBufferData329_MEAN, 48, gScenarioKernelData329_MEAN, 4},
{22, 16, 708, 16, 0, gScenarioBufferData330_MEAN, 48, gScenarioKernelData330_MEAN, 4},
{24, 16, 772, 16, 0, gScenarioBufferData331_MEAN, 48, gScenarioKernelData331_MEAN, 4},
{26, 16, 836, 16, 0, gScenarioBufferData332_MEAN, 48, gScenarioKernelData332_MEAN, 4},
{28, 16, 900, 16, 0, gScenarioBufferData333_MEAN, 48, gScenarioKernelData333_MEAN, 4},
{30, 16, 964, 16, 0, gScenarioBufferData334_MEAN, 48, gScenarioKernelData334_MEAN, 4},
{32, 16, 1028, 16, 0, gScenarioBufferData335_MEAN, 48, gScenarioKernelData335_MEAN, 4},
{36, 16, 1156, 16, 0, gScenarioBufferData336_MEAN, 48, gScenarioKernelData336_MEAN, 4},
{40, 16, 1284, 16, 0, gScenarioBufferData337_MEAN, 48, gScenarioKernelData337_MEAN, 4},
{44, 16, 1412, 16, 0, gScenarioBufferData338_MEAN, 48, gScenarioKernelData338_MEAN, 4},
{48, 16, 1540, 16, 0, gScenarioBufferData339_MEAN, 48, gScenarioKernelData339_MEAN, 4},
{52, 16, 1668, 16, 0, gScenarioBufferData340_MEAN, 48, gScenarioKernelData340_MEAN, 4},
{56, 16, 1796, 16, 0, gScenarioBufferData341_MEAN, 48, gScenarioKernelData341_MEAN, 4},
{60, 16, 1924, 16, 0, gScenarioBufferData342_MEAN, 48, gScenarioKernelData342_MEAN, 4},
{64, 16, 2052, 16, 0, gScenarioBufferData343_MEAN, 48, gScenarioKernelData343_MEAN, 4},
{72, 16, 2308, 16, 0, gScenarioBufferData344_MEAN, 48, gScenarioKernelData344_MEAN, 4},
{80, 16, 2564, 16, 0, gScenarioBufferData345_MEAN, 48, gScenarioKernelData345_MEAN, 4},
{88, 16, 2820, 16, 0, gScenarioBufferData346_MEAN, 48, gScenarioKernelData346_MEAN, 4},
{96, 16, 3076, 16, 0, gScenarioBufferData347_MEAN, 48, gScenarioKernelData347_MEAN, 4},
{104, 16, 3332, 16, 0, gScenarioBufferData348_MEAN, 48, gScenarioKernelData348_MEAN, 4},
{112, 16, 3588, 16, 0, gScenarioBufferData349_MEAN, 48, gScenarioKernelData349_MEAN, 4},
{2, 18, 76, 16, 0, gScenarioBufferData350_MEAN, 48, gScenarioKernelData350_MEAN, 4},
{4, 18, 148, 16, 0, gScenarioBufferData351_MEAN, 48, gScenarioKernelData351_MEAN, 4},
{6, 18, 220, 16, 0, gScenarioBufferData352_MEAN, 48, gScenarioKernelData352_MEAN, 4},
{8, 18, 292, 16, 0, gScenarioBufferData353_MEAN, 48, gScenarioKernelData353_MEAN, 4},
{10, 18, 364, 16, 0, gScenarioBufferData354_MEAN, 48, gScenarioKernelData354_MEAN, 4},
{12, 18, 436, 16, 0, gScenarioBufferData355_MEAN, 48, gScenarioKernelData355_MEAN, 4},
{14, 18, 508, 16, 0, gScenarioBufferData356_MEAN, 48, gScenarioKernelData356_MEAN, 4},
{16, 18, 580, 16, 0, gScenarioBufferData357_MEAN, 48, gScenarioKernelData357_MEAN, 4},
{18, 18, 652, 16, 0, gScenarioBufferData358_MEAN, 48, gScenarioKernelData358_MEAN, 4},
{20, 18, 724, 16, 0, gScenarioBufferData359_MEAN, 48, gScenarioKernelData359_MEAN, 4},
{22, 18, 796, 16, 0, gScenarioBufferData360_MEAN, 48, gScenarioKernelData360_MEAN, 4},
{24, 18, 868, 16, 0, gScenarioBufferData361_MEAN, 48, gScenarioKernelData361_MEAN, 4},
{26, 18, 940, 16, 0, gScenarioBufferData362_MEAN, 48, gScenarioKernelData362_MEAN, 4},
{28, 18, 1012, 16, 0, gScenarioBufferData363_MEAN, 48, gScenarioKernelData363_MEAN, 4},
{30, 18, 1084, 16, 0, gScenarioBufferData364_MEAN, 48, gScenarioKernelData364_MEAN, 4},
{32, 18, 1156, 16, 0, gScenarioBufferData365_MEAN, 48, gScenarioKernelData365_MEAN, 4},
{36, 18, 1300, 16, 0, gScenarioBufferData366_MEAN, 48, gScenarioKernelData366_MEAN, 4},
{40, 18, 1444, 16, 0, gScenarioBufferData367_MEAN, 48, gScenarioKernelData367_MEAN, 4},
{44, 18, 1588, 16, 0, gScenarioBufferData368_MEAN, 48, gScenarioKernelData368_MEAN, 4},
{48, 18, 1732, 16, 0, gScenarioBufferData369_MEAN, 48, gScenarioKernelData369_MEAN, 4},
{52, 18, 1876, 16, 0, gScenarioBufferData370_MEAN, 48, gScenarioKernelData370_MEAN, 4},
{56, 18, 2020, 16, 0, gScenarioBufferData371_MEAN, 48, gScenarioKernelData371_MEAN, 4},
{60, 18, 2164, 16, 0, gScenarioBufferData372_MEAN, 48, gScenarioKernelData372_MEAN, 4},
{64, 18, 2308, 16, 0, gScenarioBufferData373_MEAN, 48, gScenarioKernelData373_MEAN, 4},
{72, 18, 2596, 16, 0, gScenarioBufferData374_MEAN, 48, gScenarioKernelData374_MEAN, 4},
{80, 18, 2884, 16, 0, gScenarioBufferData375_MEAN, 48, gScenarioKernelData375_MEAN, 4},
{88, 18, 3172, 16, 0, gScenarioBufferData376_MEAN, 48, gScenarioKernelData376_MEAN, 4},
{96, 18, 3460, 16, 0, gScenarioBufferData377_MEAN, 48, gScenarioKernelData377_MEAN, 4},
{104, 18, 3748, 16, 0, gScenarioBufferData378_MEAN, 48, gScenarioKernelData378_MEAN, 4},
{2, 20, 84, 16, 0, gScenarioBufferData379_MEAN, 48, gScenarioKernelData379_MEAN, 4},
{4, 20, 164, 16, 0, gScenarioBufferData380_MEAN, 48, gScenarioKernelData380_MEAN, 4},
{6, 20, 244, 16, 0, gScenarioBufferData381_MEAN, 48, gScenarioKernelData381_MEAN, 4},
{8, 20, 324, 16, 0, gScenarioBufferData382_MEAN, 48, gScenarioKernelData382_MEAN, 4},
{10, 20, 404, 16, 0, gScenarioBufferData383_MEAN, 48, gScenarioKernelData383_MEAN, 4},
{12, 20, 484, 16, 0, gScenarioBufferData384_MEAN, 48, gScenarioKernelData384_MEAN, 4},
{14, 20, 564, 16, 0, gScenarioBufferData385_MEAN, 48, gScenarioKernelData385_MEAN, 4},
{16, 20, 644, 16, 0, gScenarioBufferData386_MEAN, 48, gScenarioKernelData386_MEAN, 4},
{18, 20, 724, 16, 0, gScenarioBufferData387_MEAN, 48, gScenarioKernelData387_MEAN, 4},
{20, 20, 804, 16, 0, gScenarioBufferData388_MEAN, 48, gScenarioKernelData388_MEAN, 4},
{22, 20, 884, 16, 0, gScenarioBufferData389_MEAN, 48, gScenarioKernelData389_MEAN, 4},
{24, 20, 964, 16, 0, gScenarioBufferData390_MEAN, 48, gScenarioKernelData390_MEAN, 4},
{26, 20, 1044, 16, 0, gScenarioBufferData391_MEAN, 48, gScenarioKernelData391_MEAN, 4},
{28, 20, 1124, 16, 0, gScenarioBufferData392_MEAN, 48, gScenarioKernelData392_MEAN, 4},
{30, 20, 1204, 16, 0, gScenarioBufferData393_MEAN, 48, gScenarioKernelData393_MEAN, 4},
{32, 20, 1284, 16, 0, gScenarioBufferData394_MEAN, 48, gScenarioKernelData394_MEAN, 4},
{36, 20, 1444, 16, 0, gScenarioBufferData395_MEAN, 48, gScenarioKernelData395_MEAN, 4},
{40, 20, 1604, 16, 0, gScenarioBufferData396_MEAN, 48, gScenarioKernelData396_MEAN, 4},
{44, 20, 1764, 16, 0, gScenarioBufferData397_MEAN, 48, gScenarioKernelData397_MEAN, 4},
{48, 20, 1924, 16, 0, gScenarioBufferData398_MEAN, 48, gScenarioKernelData398_MEAN, 4},
{52, 20, 2084, 16, 0, gScenarioBufferData399_MEAN, 48, gScenarioKernelData399_MEAN, 4},
{56, 20, 2244, 16, 0, gScenarioBufferData400_MEAN, 48, gScenarioKernelData400_MEAN, 4},
{60, 20, 2404, 16, 0, gScenarioBufferData401_MEAN, 48, gScenarioKernelData401_MEAN, 4},
{64, 20, 2564, 16, 0, gScenarioBufferData402_MEAN, 48, gScenarioKernelData402_MEAN, 4},
{72, 20, 2884, 16, 0, gScenarioBufferData403_MEAN, 48, gScenarioKernelData403_MEAN, 4},
{80, 20, 3204, 16, 0, gScenarioBufferData404_MEAN, 48, gScenarioKernelData404_MEAN, 4},
{88, 20, 3524, 16, 0, gScenarioBufferData405_MEAN, 48, gScenarioKernelData405_MEAN, 4},
{2, 22, 92, 16, 0, gScenarioBufferData406_MEAN, 48, gScenarioKernelData406_MEAN, 4},
{4, 22, 180, 16, 0, gScenarioBufferData407_MEAN, 48, gScenarioKernelData407_MEAN, 4},
{6, 22, 268, 16, 0, gScenarioBufferData408_MEAN, 48, gScenarioKernelData408_MEAN, 4},
{8, 22, 356, 16, 0, gScenarioBufferData409_MEAN, 48, gScenarioKernelData409_MEAN, 4},
{10, 22, 444, 16, 0, gScenarioBufferData410_MEAN, 48, gScenarioKernelData410_MEAN, 4},
{12, 22, 532, 16, 0, gScenarioBufferData411_MEAN, 48, gScenarioKernelData411_MEAN, 4},
{14, 22, 620, 16, 0, gScenarioBufferData412_MEAN, 48, gScenarioKernelData412_MEAN, 4},
{16, 22, 708, 16, 0, gScenarioBufferData413_MEAN, 48, gScenarioKernelData413_MEAN, 4},
{18, 22, 796, 16, 0, gScenarioBufferData414_MEAN, 48, gScenarioKernelData414_MEAN, 4},
{20, 22, 884, 16, 0, gScenarioBufferData415_MEAN, 48, gScenarioKernelData415_MEAN, 4},
{22, 22, 972, 16, 0, gScenarioBufferData416_MEAN, 48, gScenarioKernelData416_MEAN, 4},
{24, 22, 1060, 16, 0, gScenarioBufferData417_MEAN, 48, gScenarioKernelData417_MEAN, 4},
{26, 22, 1148, 16, 0, gScenarioBufferData418_MEAN, 48, gScenarioKernelData418_MEAN, 4},
{28, 22, 1236, 16, 0, gScenarioBufferData419_MEAN, 48, gScenarioKernelData419_MEAN, 4},
{30, 22, 1324, 16, 0, gScenarioBufferData420_MEAN, 48, gScenarioKernelData420_MEAN, 4},
{32, 22, 1412, 16, 0, gScenarioBufferData421_MEAN, 48, gScenarioKernelData421_MEAN, 4},
{36, 22, 1588, 16, 0, gScenarioBufferData422_MEAN, 48, gScenarioKernelData422_MEAN, 4},
{40, 22, 1764, 16, 0, gScenarioBufferData423_MEAN, 48, gScenarioKernelData423_MEAN, 4},
{44, 22, 1940, 16, 0, gScenarioBufferData424_MEAN, 48, gScenarioKernelData424_MEAN, 4},
{48, 22, 2116, 16, 0, gScenarioBufferData425_MEAN, 48, gScenarioKernelData425_MEAN, 4},
{52, 22, 2292, 16, 0, gScenarioBufferData426_MEAN, 48, gScenarioKernelData426_MEAN, 4},
{56, 22, 2468, 16, 0, gScenarioBufferData427_MEAN, 48, gScenarioKernelData427_MEAN, 4},
{60, 22, 2644, 16, 0, gScenarioBufferData428_MEAN, 48, gScenarioKernelData428_MEAN, 4},
{64, 22, 2820, 16, 0, gScenarioBufferData429_MEAN, 48, gScenarioKernelData429_MEAN, 4},
{72, 22, 3172, 16, 0, gScenarioBufferData430_MEAN, 48, gScenarioKernelData430_MEAN, 4},
{80, 22, 3524, 16, 0, gScenarioBufferData431_MEAN, 48, gScenarioKernelData431_MEAN, 4},
{2, 24, 100, 16, 0, gScenarioBufferData432_MEAN, 48, gScenarioKernelData432_MEAN, 4},
{4, 24, 196, 16, 0, gScenarioBufferData433_MEAN, 48, gScenarioKernelData433_MEAN, 4},
{6, 24, 292, 16, 0, gScenarioBufferData434_MEAN, 48, gScenarioKernelData434_MEAN, 4},
{8, 24, 388, 16, 0, gScenarioBufferData435_MEAN, 48, gScenarioKernelData435_MEAN, 4},
{10, 24, 484, 16, 0, gScenarioBufferData436_MEAN, 48, gScenarioKernelData436_MEAN, 4},
{12, 24, 580, 16, 0, gScenarioBufferData437_MEAN, 48, gScenarioKernelData437_MEAN, 4},
{14, 24, 676, 16, 0, gScenarioBufferData438_MEAN, 48, gScenarioKernelData438_MEAN, 4},
{16, 24, 772, 16, 0, gScenarioBufferData439_MEAN, 48, gScenarioKernelData439_MEAN, 4},
{18, 24, 868, 16, 0, gScenarioBufferData440_MEAN, 48, gScenarioKernelData440_MEAN, 4},
{20, 24, 964, 16, 0, gScenarioBufferData441_MEAN, 48, gScenarioKernelData441_MEAN, 4},
{22, 24, 1060, 16, 0, gScenarioBufferData442_MEAN, 48, gScenarioKernelData442_MEAN, 4},
{24, 24, 1156, 16, 0, gScenarioBufferData443_MEAN, 48, gScenarioKernelData443_MEAN, 4},
{26, 24, 1252, 16, 0, gScenarioBufferData444_MEAN, 48, gScenarioKernelData444_MEAN, 4},
{28, 24, 1348, 16, 0, gScenarioBufferData445_MEAN, 48, gScenarioKernelData445_MEAN, 4},
{30, 24, 1444, 16, 0, gScenarioBufferData446_MEAN, 48, gScenarioKernelData446_MEAN, 4},
{32, 24, 1540, 16, 0, gScenarioBufferData447_MEAN, 48, gScenarioKernelData447_MEAN, 4},
{36, 24, 1732, 16, 0, gScenarioBufferData448_MEAN, 48, gScenarioKernelData448_MEAN, 4},
{40, 24, 1924, 16, 0, gScenarioBufferData449_MEAN, 48, gScenarioKernelData449_MEAN, 4},
{44, 24, 2116, 16, 0, gScenarioBufferData450_MEAN, 48, gScenarioKernelData450_MEAN, 4},
{48, 24, 2308, 16, 0, gScenarioBufferData451_MEAN, 48, gScenarioKernelData451_MEAN, 4},
{52, 24, 2500, 16, 0, gScenarioBufferData452_MEAN, 48, gScenarioKernelData452_MEAN, 4},
{56, 24, 2692, 16, 0, gScenarioBufferData453_MEAN, 48, gScenarioKernelData453_MEAN, 4},
{60, 24, 2884, 16, 0, gScenarioBufferData454_MEAN, 48, gScenarioKernelData454_MEAN, 4},
{64, 24, 3076, 16, 0, gScenarioBufferData455_MEAN, 48, gScenarioKernelData455_MEAN, 4},
{72, 24, 3460, 16, 0, gScenarioBufferData456_MEAN, 48, gScenarioKernelData456_MEAN, 4},
{2, 26, 108, 16, 0, gScenarioBufferData457_MEAN, 48, gScenarioKernelData457_MEAN, 4},
{4, 26, 212, 16, 0, gScenarioBufferData458_MEAN, 48, gScenarioKernelData458_MEAN, 4},
{6, 26, 316, 16, 0, gScenarioBufferData459_MEAN, 48, gScenarioKernelData459_MEAN, 4},
{8, 26, 420, 16, 0, gScenarioBufferData460_MEAN, 48, gScenarioKernelData460_MEAN, 4},
{10, 26, 524, 16, 0, gScenarioBufferData461_MEAN, 48, gScenarioKernelData461_MEAN, 4},
{12, 26, 628, 16, 0, gScenarioBufferData462_MEAN, 48, gScenarioKernelData462_MEAN, 4},
{14, 26, 732, 16, 0, gScenarioBufferData463_MEAN, 48, gScenarioKernelData463_MEAN, 4},
{16, 26, 836, 16, 0, gScenarioBufferData464_MEAN, 48, gScenarioKernelData464_MEAN, 4},
{18, 26, 940, 16, 0, gScenarioBufferData465_MEAN, 48, gScenarioKernelData465_MEAN, 4},
{20, 26, 1044, 16, 0, gScenarioBufferData466_MEAN, 48, gScenarioKernelData466_MEAN, 4},
{22, 26, 1148, 16, 0, gScenarioBufferData467_MEAN, 48, gScenarioKernelData467_MEAN, 4},
{24, 26, 1252, 16, 0, gScenarioBufferData468_MEAN, 48, gScenarioKernelData468_MEAN, 4},
{26, 26, 1356, 16, 0, gScenarioBufferData469_MEAN, 48, gScenarioKernelData469_MEAN, 4},
{28, 26, 1460, 16, 0, gScenarioBufferData470_MEAN, 48, gScenarioKernelData470_MEAN, 4},
{30, 26, 1564, 16, 0, gScenarioBufferData471_MEAN, 48, gScenarioKernelData471_MEAN, 4},
{32, 26, 1668, 16, 0, gScenarioBufferData472_MEAN, 48, gScenarioKernelData472_MEAN, 4},
{36, 26, 1876, 16, 0, gScenarioBufferData473_MEAN, 48, gScenarioKernelData473_MEAN, 4},
{40, 26, 2084, 16, 0, gScenarioBufferData474_MEAN, 48, gScenarioKernelData474_MEAN, 4},
{44, 26, 2292, 16, 0, gScenarioBufferData475_MEAN, 48, gScenarioKernelData475_MEAN, 4},
{48, 26, 2500, 16, 0, gScenarioBufferData476_MEAN, 48, gScenarioKernelData476_MEAN, 4},
{52, 26, 2708, 16, 0, gScenarioBufferData477_MEAN, 48, gScenarioKernelData477_MEAN, 4},
{56, 26, 2916, 16, 0, gScenarioBufferData478_MEAN, 48, gScenarioKernelData478_MEAN, 4},
{60, 26, 3124, 16, 0, gScenarioBufferData479_MEAN, 48, gScenarioKernelData479_MEAN, 4},
{64, 26, 3332, 16, 0, gScenarioBufferData480_MEAN, 48, gScenarioKernelData480_MEAN, 4},
{72, 26, 3748, 16, 0, gScenarioBufferData481_MEAN, 48, gScenarioKernelData481_MEAN, 4},
{2, 28, 116, 16, 0, gScenarioBufferData482_MEAN, 48, gScenarioKernelData482_MEAN, 4},
{4, 28, 228, 16, 0, gScenarioBufferData483_MEAN, 48, gScenarioKernelData483_MEAN, 4},
{6, 28, 340, 16, 0, gScenarioBufferData484_MEAN, 48, gScenarioKernelData484_MEAN, 4},
{8, 28, 452, 16, 0, gScenarioBufferData485_MEAN, 48, gScenarioKernelData485_MEAN, 4},
{10, 28, 564, 16, 0, gScenarioBufferData486_MEAN, 48, gScenarioKernelData486_MEAN, 4},
{12, 28, 676, 16, 0, gScenarioBufferData487_MEAN, 48, gScenarioKernelData487_MEAN, 4},
{14, 28, 788, 16, 0, gScenarioBufferData488_MEAN, 48, gScenarioKernelData488_MEAN, 4},
{16, 28, 900, 16, 0, gScenarioBufferData489_MEAN, 48, gScenarioKernelData489_MEAN, 4},
{18, 28, 1012, 16, 0, gScenarioBufferData490_MEAN, 48, gScenarioKernelData490_MEAN, 4},
{20, 28, 1124, 16, 0, gScenarioBufferData491_MEAN, 48, gScenarioKernelData491_MEAN, 4},
{22, 28, 1236, 16, 0, gScenarioBufferData492_MEAN, 48, gScenarioKernelData492_MEAN, 4},
{24, 28, 1348, 16, 0, gScenarioBufferData493_MEAN, 48, gScenarioKernelData493_MEAN, 4},
{26, 28, 1460, 16, 0, gScenarioBufferData494_MEAN, 48, gScenarioKernelData494_MEAN, 4},
{28, 28, 1572, 16, 0, gScenarioBufferData495_MEAN, 48, gScenarioKernelData495_MEAN, 4},
{30, 28, 1684, 16, 0, gScenarioBufferData496_MEAN, 48, gScenarioKernelData496_MEAN, 4},
{32, 28, 1796, 16, 0, gScenarioBufferData497_MEAN, 48, gScenarioKernelData497_MEAN, 4},
{36, 28, 2020, 16, 0, gScenarioBufferData498_MEAN, 48, gScenarioKernelData498_MEAN, 4},
{40, 28, 2244, 16, 0, gScenarioBufferData499_MEAN, 48, gScenarioKernelData499_MEAN, 4},
{44, 28, 2468, 16, 0, gScenarioBufferData500_MEAN, 48, gScenarioKernelData500_MEAN, 4},
{48, 28, 2692, 16, 0, gScenarioBufferData501_MEAN, 48, gScenarioKernelData501_MEAN, 4},
{52, 28, 2916, 16, 0, gScenarioBufferData502_MEAN, 48, gScenarioKernelData502_MEAN, 4},
{56, 28, 3140, 16, 0, gScenarioBufferData503_MEAN, 48, gScenarioKernelData503_MEAN, 4},
{60, 28, 3364, 16, 0, gScenarioBufferData504_MEAN, 48, gScenarioKernelData504_MEAN, 4},
{64, 28, 3588, 16, 0, gScenarioBufferData505_MEAN, 48, gScenarioKernelData505_MEAN, 4},
{2, 30, 124, 16, 0, gScenarioBufferData506_MEAN, 48, gScenarioKernelData506_MEAN, 4},
{4, 30, 244, 16, 0, gScenarioBufferData507_MEAN, 48, gScenarioKernelData507_MEAN, 4},
{6, 30, 364, 16, 0, gScenarioBufferData508_MEAN, 48, gScenarioKernelData508_MEAN, 4},
{8, 30, 484, 16, 0, gScenarioBufferData509_MEAN, 48, gScenarioKernelData509_MEAN, 4},
{10, 30, 604, 16, 0, gScenarioBufferData510_MEAN, 48, gScenarioKernelData510_MEAN, 4},
{12, 30, 724, 16, 0, gScenarioBufferData511_MEAN, 48, gScenarioKernelData511_MEAN, 4},
{14, 30, 844, 16, 0, gScenarioBufferData512_MEAN, 48, gScenarioKernelData512_MEAN, 4},
{16, 30, 964, 16, 0, gScenarioBufferData513_MEAN, 48, gScenarioKernelData513_MEAN, 4},
{18, 30, 1084, 16, 0, gScenarioBufferData514_MEAN, 48, gScenarioKernelData514_MEAN, 4},
{20, 30, 1204, 16, 0, gScenarioBufferData515_MEAN, 48, gScenarioKernelData515_MEAN, 4},
{22, 30, 1324, 16, 0, gScenarioBufferData516_MEAN, 48, gScenarioKernelData516_MEAN, 4},
{24, 30, 1444, 16, 0, gScenarioBufferData517_MEAN, 48, gScenarioKernelData517_MEAN, 4},
{26, 30, 1564, 16, 0, gScenarioBufferData518_MEAN, 48, gScenarioKernelData518_MEAN, 4},
{28, 30, 1684, 16, 0, gScenarioBufferData519_MEAN, 48, gScenarioKernelData519_MEAN, 4},
{30, 30, 1804, 16, 0, gScenarioBufferData520_MEAN, 48, gScenarioKernelData520_MEAN, 4},
{32, 30, 1924, 16, 0, gScenarioBufferData521_MEAN, 48, gScenarioKernelData521_MEAN, 4},
{36, 30, 2164, 16, 0, gScenarioBufferData522_MEAN, 48, gScenarioKernelData522_MEAN, 4},
{40, 30, 2404, 16, 0, gScenarioBufferData523_MEAN, 48, gScenarioKernelData523_MEAN, 4},
{44, 30, 2644, 16, 0, gScenarioBufferData524_MEAN, 48, gScenarioKernelData524_MEAN, 4},
{48, 30, 2884, 16, 0, gScenarioBufferData525_MEAN, 48, gScenarioKernelData525_MEAN, 4},
{52, 30, 3124, 16, 0, gScenarioBufferData526_MEAN, 48, gScenarioKernelData526_MEAN, 4},
{56, 30, 3364, 16, 0, gScenarioBufferData527_MEAN, 48, gScenarioKernelData527_MEAN, 4},
{60, 30, 3604, 16, 0, gScenarioBufferData528_MEAN, 48, gScenarioKernelData528_MEAN, 4},
{2, 32, 132, 16, 0, gScenarioBufferData529_MEAN, 48, gScenarioKernelData529_MEAN, 4},
{4, 32, 260, 16, 0, gScenarioBufferData530_MEAN, 48, gScenarioKernelData530_MEAN, 4},
{6, 32, 388, 16, 0, gScenarioBufferData531_MEAN, 48, gScenarioKernelData531_MEAN, 4},
{8, 32, 516, 16, 0, gScenarioBufferData532_MEAN, 48, gScenarioKernelData532_MEAN, 4},
{10, 32, 644, 16, 0, gScenarioBufferData533_MEAN, 48, gScenarioKernelData533_MEAN, 4},
{12, 32, 772, 16, 0, gScenarioBufferData534_MEAN, 48, gScenarioKernelData534_MEAN, 4},
{14, 32, 900, 16, 0, gScenarioBufferData535_MEAN, 48, gScenarioKernelData535_MEAN, 4},
{16, 32, 1028, 16, 0, gScenarioBufferData536_MEAN, 48, gScenarioKernelData536_MEAN, 4},
{18, 32, 1156, 16, 0, gScenarioBufferData537_MEAN, 48, gScenarioKernelData537_MEAN, 4},
{20, 32, 1284, 16, 0, gScenarioBufferData538_MEAN, 48, gScenarioKernelData538_MEAN, 4},
{22, 32, 1412, 16, 0, gScenarioBufferData539_MEAN, 48, gScenarioKernelData539_MEAN, 4},
{24, 32, 1540, 16, 0, gScenarioBufferData540_MEAN, 48, gScenarioKernelData540_MEAN, 4},
{26, 32, 1668, 16, 0, gScenarioBufferData541_MEAN, 48, gScenarioKernelData541_MEAN, 4},
{28, 32, 1796, 16, 0, gScenarioBufferData542_MEAN, 48, gScenarioKernelData542_MEAN, 4},
{30, 32, 1924, 16, 0, gScenarioBufferData543_MEAN, 48, gScenarioKernelData543_MEAN, 4},
{32, 32, 2052, 16, 0, gScenarioBufferData544_MEAN, 48, gScenarioKernelData544_MEAN, 4},
{36, 32, 2308, 16, 0, gScenarioBufferData545_MEAN, 48, gScenarioKernelData545_MEAN, 4},
{40, 32, 2564, 16, 0, gScenarioBufferData546_MEAN, 48, gScenarioKernelData546_MEAN, 4},
{44, 32, 2820, 16, 0, gScenarioBufferData547_MEAN, 48, gScenarioKernelData547_MEAN, 4},
{48, 32, 3076, 16, 0, gScenarioBufferData548_MEAN, 48, gScenarioKernelData548_MEAN, 4},
{52, 32, 3332, 16, 0, gScenarioBufferData549_MEAN, 48, gScenarioKernelData549_MEAN, 4},
{56, 32, 3588, 16, 0, gScenarioBufferData550_MEAN, 48, gScenarioKernelData550_MEAN, 4}
};

static acf_scenario_list gScenarioList_MEAN = {
551, //number of scenarios
gScenarioArray_MEAN};
//**************************************************************

class MEAN : public ACF_Process_APU
{

public:
   MEAN(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MEAN");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MEAN",
                                        MEAN_LOAD_SEGMENTS,
                                        MEAN_LOAD_PMEM, MEAN_LOAD_PMEM_SIZE,
                                        MEAN_LOAD_DMEM, MEAN_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MEAN_APEX_LOG_BUFFER, MEAN_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_32U, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MEAN);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MEAN

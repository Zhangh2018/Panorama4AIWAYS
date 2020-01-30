#ifndef _ACF_PROCESS_APU_MEANSTDDEV
#define _ACF_PROCESS_APU_MEANSTDDEV

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MEANSTDDEV_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MEANSTDDEV[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MEANSTDDEV[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MEANSTDDEV[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MEANSTDDEV[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MEANSTDDEV[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MEANSTDDEV[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MEANSTDDEV[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MEANSTDDEV[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MEANSTDDEV[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MEANSTDDEV[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MEANSTDDEV[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MEANSTDDEV[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MEANSTDDEV[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MEANSTDDEV[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MEANSTDDEV[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MEANSTDDEV[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MEANSTDDEV[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MEANSTDDEV[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MEANSTDDEV[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MEANSTDDEV[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MEANSTDDEV[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MEANSTDDEV[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MEANSTDDEV[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MEANSTDDEV[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MEANSTDDEV[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MEANSTDDEV[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MEANSTDDEV[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MEANSTDDEV[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MEANSTDDEV[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MEANSTDDEV[] = {{112, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MEANSTDDEV[] = {{120, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MEANSTDDEV[] = {{128, 1, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MEANSTDDEV[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MEANSTDDEV[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MEANSTDDEV[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MEANSTDDEV[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MEANSTDDEV[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MEANSTDDEV[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MEANSTDDEV[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MEANSTDDEV[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MEANSTDDEV[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MEANSTDDEV[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MEANSTDDEV[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MEANSTDDEV[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MEANSTDDEV[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MEANSTDDEV[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MEANSTDDEV[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MEANSTDDEV[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MEANSTDDEV[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MEANSTDDEV[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MEANSTDDEV[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MEANSTDDEV[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MEANSTDDEV[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MEANSTDDEV[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MEANSTDDEV[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MEANSTDDEV[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MEANSTDDEV[] = {{72, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MEANSTDDEV[] = {{80, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MEANSTDDEV[] = {{88, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MEANSTDDEV[] = {{96, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MEANSTDDEV[] = {{104, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MEANSTDDEV[] = {{112, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MEANSTDDEV[] = {{120, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MEANSTDDEV[] = {{128, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MEANSTDDEV[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MEANSTDDEV[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MEANSTDDEV[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MEANSTDDEV[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MEANSTDDEV[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MEANSTDDEV[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MEANSTDDEV[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MEANSTDDEV[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MEANSTDDEV[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MEANSTDDEV[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MEANSTDDEV[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MEANSTDDEV[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MEANSTDDEV[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MEANSTDDEV[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MEANSTDDEV[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MEANSTDDEV[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MEANSTDDEV[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MEANSTDDEV[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MEANSTDDEV[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MEANSTDDEV[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MEANSTDDEV[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MEANSTDDEV[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MEANSTDDEV[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MEANSTDDEV[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MEANSTDDEV[] = {{72, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MEANSTDDEV[] = {{80, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MEANSTDDEV[] = {{88, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MEANSTDDEV[] = {{96, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MEANSTDDEV[] = {{104, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MEANSTDDEV[] = {{112, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MEANSTDDEV[] = {{120, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MEANSTDDEV[] = {{128, 3, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MEANSTDDEV[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MEANSTDDEV[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MEANSTDDEV[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MEANSTDDEV[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MEANSTDDEV[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MEANSTDDEV[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MEANSTDDEV[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MEANSTDDEV[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MEANSTDDEV[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MEANSTDDEV[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MEANSTDDEV[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MEANSTDDEV[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MEANSTDDEV[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MEANSTDDEV[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MEANSTDDEV[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MEANSTDDEV[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MEANSTDDEV[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MEANSTDDEV[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MEANSTDDEV[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MEANSTDDEV[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MEANSTDDEV[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MEANSTDDEV[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MEANSTDDEV[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MEANSTDDEV[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MEANSTDDEV[] = {{72, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MEANSTDDEV[] = {{80, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MEANSTDDEV[] = {{88, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MEANSTDDEV[] = {{96, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MEANSTDDEV[] = {{104, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MEANSTDDEV[] = {{112, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MEANSTDDEV[] = {{120, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MEANSTDDEV[] = {{128, 4, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MEANSTDDEV[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MEANSTDDEV[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MEANSTDDEV[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MEANSTDDEV[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MEANSTDDEV[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MEANSTDDEV[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MEANSTDDEV[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MEANSTDDEV[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MEANSTDDEV[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MEANSTDDEV[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MEANSTDDEV[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MEANSTDDEV[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MEANSTDDEV[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MEANSTDDEV[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MEANSTDDEV[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MEANSTDDEV[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MEANSTDDEV[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MEANSTDDEV[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MEANSTDDEV[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MEANSTDDEV[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MEANSTDDEV[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MEANSTDDEV[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MEANSTDDEV[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MEANSTDDEV[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MEANSTDDEV[] = {{72, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MEANSTDDEV[] = {{80, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MEANSTDDEV[] = {{88, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MEANSTDDEV[] = {{96, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MEANSTDDEV[] = {{104, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MEANSTDDEV[] = {{112, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MEANSTDDEV[] = {{120, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MEANSTDDEV[] = {{128, 5, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MEANSTDDEV[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MEANSTDDEV[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MEANSTDDEV[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MEANSTDDEV[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MEANSTDDEV[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MEANSTDDEV[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MEANSTDDEV[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MEANSTDDEV[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MEANSTDDEV[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MEANSTDDEV[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MEANSTDDEV[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MEANSTDDEV[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MEANSTDDEV[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MEANSTDDEV[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MEANSTDDEV[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MEANSTDDEV[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MEANSTDDEV[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MEANSTDDEV[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MEANSTDDEV[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MEANSTDDEV[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MEANSTDDEV[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MEANSTDDEV[] = {{56, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MEANSTDDEV[] = {{60, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MEANSTDDEV[] = {{64, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MEANSTDDEV[] = {{72, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MEANSTDDEV[] = {{80, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MEANSTDDEV[] = {{88, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MEANSTDDEV[] = {{96, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MEANSTDDEV[] = {{104, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MEANSTDDEV[] = {{112, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MEANSTDDEV[] = {{120, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MEANSTDDEV[] = {{128, 6, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MEANSTDDEV[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MEANSTDDEV[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MEANSTDDEV[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MEANSTDDEV[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MEANSTDDEV[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MEANSTDDEV[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MEANSTDDEV[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MEANSTDDEV[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MEANSTDDEV[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MEANSTDDEV[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MEANSTDDEV[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MEANSTDDEV[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MEANSTDDEV[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MEANSTDDEV[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MEANSTDDEV[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MEANSTDDEV[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MEANSTDDEV[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MEANSTDDEV[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MEANSTDDEV[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MEANSTDDEV[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MEANSTDDEV[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MEANSTDDEV[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MEANSTDDEV[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MEANSTDDEV[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MEANSTDDEV[] = {{72, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MEANSTDDEV[] = {{80, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MEANSTDDEV[] = {{88, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MEANSTDDEV[] = {{96, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MEANSTDDEV[] = {{104, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MEANSTDDEV[] = {{112, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MEANSTDDEV[] = {{120, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MEANSTDDEV[] = {{128, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MEANSTDDEV[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MEANSTDDEV[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MEANSTDDEV[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MEANSTDDEV[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MEANSTDDEV[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MEANSTDDEV[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MEANSTDDEV[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MEANSTDDEV[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MEANSTDDEV[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MEANSTDDEV[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MEANSTDDEV[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MEANSTDDEV[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MEANSTDDEV[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MEANSTDDEV[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MEANSTDDEV[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MEANSTDDEV[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MEANSTDDEV[] = {{36, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MEANSTDDEV[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MEANSTDDEV[] = {{44, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MEANSTDDEV[] = {{48, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MEANSTDDEV[] = {{52, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MEANSTDDEV[] = {{56, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MEANSTDDEV[] = {{60, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MEANSTDDEV[] = {{64, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MEANSTDDEV[] = {{72, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MEANSTDDEV[] = {{80, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MEANSTDDEV[] = {{88, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_MEANSTDDEV[] = {{96, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_MEANSTDDEV[] = {{104, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_MEANSTDDEV[] = {{112, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_MEANSTDDEV[] = {{120, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_MEANSTDDEV[] = {{128, 10, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_MEANSTDDEV[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_MEANSTDDEV[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_MEANSTDDEV[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_MEANSTDDEV[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_MEANSTDDEV[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_MEANSTDDEV[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_MEANSTDDEV[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_MEANSTDDEV[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_MEANSTDDEV[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_MEANSTDDEV[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_MEANSTDDEV[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_MEANSTDDEV[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_MEANSTDDEV[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_MEANSTDDEV[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_MEANSTDDEV[] = {{30, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_MEANSTDDEV[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_MEANSTDDEV[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_MEANSTDDEV[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_MEANSTDDEV[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_MEANSTDDEV[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_MEANSTDDEV[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_MEANSTDDEV[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_MEANSTDDEV[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_MEANSTDDEV[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_MEANSTDDEV[] = {{72, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_MEANSTDDEV[] = {{80, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_MEANSTDDEV[] = {{88, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_MEANSTDDEV[] = {{96, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_MEANSTDDEV[] = {{104, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_MEANSTDDEV[] = {{112, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_MEANSTDDEV[] = {{120, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_MEANSTDDEV[] = {{128, 12, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_MEANSTDDEV[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_MEANSTDDEV[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_MEANSTDDEV[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_MEANSTDDEV[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_MEANSTDDEV[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_MEANSTDDEV[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_MEANSTDDEV[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_MEANSTDDEV[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_MEANSTDDEV[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_MEANSTDDEV[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_MEANSTDDEV[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_MEANSTDDEV[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_MEANSTDDEV[] = {{26, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_MEANSTDDEV[] = {{28, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_MEANSTDDEV[] = {{30, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_MEANSTDDEV[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_MEANSTDDEV[] = {{36, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_MEANSTDDEV[] = {{40, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_MEANSTDDEV[] = {{44, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_MEANSTDDEV[] = {{48, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_MEANSTDDEV[] = {{52, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_MEANSTDDEV[] = {{56, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_MEANSTDDEV[] = {{60, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_MEANSTDDEV[] = {{64, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_MEANSTDDEV[] = {{72, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_MEANSTDDEV[] = {{80, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_MEANSTDDEV[] = {{88, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_MEANSTDDEV[] = {{96, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_MEANSTDDEV[] = {{104, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_MEANSTDDEV[] = {{112, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_MEANSTDDEV[] = {{120, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_MEANSTDDEV[] = {{128, 14, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_MEANSTDDEV[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_MEANSTDDEV[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_MEANSTDDEV[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_MEANSTDDEV[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_MEANSTDDEV[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_MEANSTDDEV[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_MEANSTDDEV[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_MEANSTDDEV[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_MEANSTDDEV[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_MEANSTDDEV[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_MEANSTDDEV[] = {{22, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_MEANSTDDEV[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_MEANSTDDEV[] = {{26, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_MEANSTDDEV[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_MEANSTDDEV[] = {{30, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_MEANSTDDEV[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_MEANSTDDEV[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_MEANSTDDEV[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_MEANSTDDEV[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_MEANSTDDEV[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_MEANSTDDEV[] = {{52, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_MEANSTDDEV[] = {{56, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_MEANSTDDEV[] = {{60, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_MEANSTDDEV[] = {{64, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_MEANSTDDEV[] = {{72, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_MEANSTDDEV[] = {{80, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_MEANSTDDEV[] = {{88, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_MEANSTDDEV[] = {{96, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_MEANSTDDEV[] = {{104, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_MEANSTDDEV[] = {{112, 16, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_MEANSTDDEV[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_MEANSTDDEV[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_MEANSTDDEV[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_MEANSTDDEV[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_MEANSTDDEV[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_MEANSTDDEV[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_MEANSTDDEV[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_MEANSTDDEV[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_MEANSTDDEV[] = {{18, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_MEANSTDDEV[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_MEANSTDDEV[] = {{22, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_MEANSTDDEV[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_MEANSTDDEV[] = {{26, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_MEANSTDDEV[] = {{28, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_MEANSTDDEV[] = {{30, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_MEANSTDDEV[] = {{32, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_MEANSTDDEV[] = {{36, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_MEANSTDDEV[] = {{40, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_MEANSTDDEV[] = {{44, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_MEANSTDDEV[] = {{48, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_MEANSTDDEV[] = {{52, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_MEANSTDDEV[] = {{56, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_MEANSTDDEV[] = {{60, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_MEANSTDDEV[] = {{64, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_MEANSTDDEV[] = {{72, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_MEANSTDDEV[] = {{80, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_MEANSTDDEV[] = {{88, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_MEANSTDDEV[] = {{96, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_MEANSTDDEV[] = {{104, 18, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_MEANSTDDEV[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_MEANSTDDEV[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_MEANSTDDEV[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_MEANSTDDEV[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_MEANSTDDEV[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_MEANSTDDEV[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_MEANSTDDEV[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_MEANSTDDEV[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_MEANSTDDEV[] = {{18, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_MEANSTDDEV[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_MEANSTDDEV[] = {{22, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_MEANSTDDEV[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_MEANSTDDEV[] = {{26, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_MEANSTDDEV[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_MEANSTDDEV[] = {{30, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_MEANSTDDEV[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_MEANSTDDEV[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_MEANSTDDEV[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_MEANSTDDEV[] = {{44, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_MEANSTDDEV[] = {{48, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData399_MEANSTDDEV[] = {{52, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData400_MEANSTDDEV[] = {{56, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData401_MEANSTDDEV[] = {{60, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData402_MEANSTDDEV[] = {{64, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData403_MEANSTDDEV[] = {{72, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData404_MEANSTDDEV[] = {{80, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData405_MEANSTDDEV[] = {{88, 20, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData406_MEANSTDDEV[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData407_MEANSTDDEV[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData408_MEANSTDDEV[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData409_MEANSTDDEV[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData410_MEANSTDDEV[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData411_MEANSTDDEV[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData412_MEANSTDDEV[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData413_MEANSTDDEV[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData414_MEANSTDDEV[] = {{18, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData415_MEANSTDDEV[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData416_MEANSTDDEV[] = {{22, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData417_MEANSTDDEV[] = {{24, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData418_MEANSTDDEV[] = {{26, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData419_MEANSTDDEV[] = {{28, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData420_MEANSTDDEV[] = {{30, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData421_MEANSTDDEV[] = {{32, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData422_MEANSTDDEV[] = {{36, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData423_MEANSTDDEV[] = {{40, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData424_MEANSTDDEV[] = {{44, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData425_MEANSTDDEV[] = {{48, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData426_MEANSTDDEV[] = {{52, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData427_MEANSTDDEV[] = {{56, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData428_MEANSTDDEV[] = {{60, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData429_MEANSTDDEV[] = {{64, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData430_MEANSTDDEV[] = {{72, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData431_MEANSTDDEV[] = {{80, 22, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData432_MEANSTDDEV[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData433_MEANSTDDEV[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData434_MEANSTDDEV[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData435_MEANSTDDEV[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData436_MEANSTDDEV[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData437_MEANSTDDEV[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData438_MEANSTDDEV[] = {{14, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData439_MEANSTDDEV[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData440_MEANSTDDEV[] = {{18, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData441_MEANSTDDEV[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData442_MEANSTDDEV[] = {{22, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData443_MEANSTDDEV[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData444_MEANSTDDEV[] = {{26, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData445_MEANSTDDEV[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData446_MEANSTDDEV[] = {{30, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData447_MEANSTDDEV[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData448_MEANSTDDEV[] = {{36, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData449_MEANSTDDEV[] = {{40, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData450_MEANSTDDEV[] = {{44, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData451_MEANSTDDEV[] = {{48, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData452_MEANSTDDEV[] = {{52, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData453_MEANSTDDEV[] = {{56, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData454_MEANSTDDEV[] = {{60, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData455_MEANSTDDEV[] = {{64, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData456_MEANSTDDEV[] = {{72, 24, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData457_MEANSTDDEV[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData458_MEANSTDDEV[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData459_MEANSTDDEV[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData460_MEANSTDDEV[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData461_MEANSTDDEV[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData462_MEANSTDDEV[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData463_MEANSTDDEV[] = {{14, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData464_MEANSTDDEV[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData465_MEANSTDDEV[] = {{18, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData466_MEANSTDDEV[] = {{20, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData467_MEANSTDDEV[] = {{22, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData468_MEANSTDDEV[] = {{24, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData469_MEANSTDDEV[] = {{26, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData470_MEANSTDDEV[] = {{28, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData471_MEANSTDDEV[] = {{30, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData472_MEANSTDDEV[] = {{32, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData473_MEANSTDDEV[] = {{36, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData474_MEANSTDDEV[] = {{40, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData475_MEANSTDDEV[] = {{44, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData476_MEANSTDDEV[] = {{48, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData477_MEANSTDDEV[] = {{52, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData478_MEANSTDDEV[] = {{56, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData479_MEANSTDDEV[] = {{60, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData480_MEANSTDDEV[] = {{64, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData481_MEANSTDDEV[] = {{72, 26, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData482_MEANSTDDEV[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData483_MEANSTDDEV[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData484_MEANSTDDEV[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData485_MEANSTDDEV[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData486_MEANSTDDEV[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData487_MEANSTDDEV[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData488_MEANSTDDEV[] = {{14, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData489_MEANSTDDEV[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData490_MEANSTDDEV[] = {{18, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData491_MEANSTDDEV[] = {{20, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData492_MEANSTDDEV[] = {{22, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData493_MEANSTDDEV[] = {{24, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData494_MEANSTDDEV[] = {{26, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData495_MEANSTDDEV[] = {{28, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData496_MEANSTDDEV[] = {{30, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData497_MEANSTDDEV[] = {{32, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData498_MEANSTDDEV[] = {{36, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData499_MEANSTDDEV[] = {{40, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData500_MEANSTDDEV[] = {{44, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData501_MEANSTDDEV[] = {{48, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData502_MEANSTDDEV[] = {{52, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData503_MEANSTDDEV[] = {{56, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData504_MEANSTDDEV[] = {{60, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData505_MEANSTDDEV[] = {{64, 28, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData506_MEANSTDDEV[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData507_MEANSTDDEV[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData508_MEANSTDDEV[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData509_MEANSTDDEV[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData510_MEANSTDDEV[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData511_MEANSTDDEV[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData512_MEANSTDDEV[] = {{14, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData513_MEANSTDDEV[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData514_MEANSTDDEV[] = {{18, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData515_MEANSTDDEV[] = {{20, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData516_MEANSTDDEV[] = {{22, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData517_MEANSTDDEV[] = {{24, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData518_MEANSTDDEV[] = {{26, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData519_MEANSTDDEV[] = {{28, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData520_MEANSTDDEV[] = {{30, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData521_MEANSTDDEV[] = {{32, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData522_MEANSTDDEV[] = {{36, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData523_MEANSTDDEV[] = {{40, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData524_MEANSTDDEV[] = {{44, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData525_MEANSTDDEV[] = {{48, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData526_MEANSTDDEV[] = {{52, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData527_MEANSTDDEV[] = {{56, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData528_MEANSTDDEV[] = {{60, 30, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData529_MEANSTDDEV[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData530_MEANSTDDEV[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData531_MEANSTDDEV[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData532_MEANSTDDEV[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData533_MEANSTDDEV[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData534_MEANSTDDEV[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData535_MEANSTDDEV[] = {{14, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData536_MEANSTDDEV[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData537_MEANSTDDEV[] = {{18, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData538_MEANSTDDEV[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData539_MEANSTDDEV[] = {{22, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData540_MEANSTDDEV[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData541_MEANSTDDEV[] = {{26, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData542_MEANSTDDEV[] = {{28, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData543_MEANSTDDEV[] = {{30, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData544_MEANSTDDEV[] = {{32, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData545_MEANSTDDEV[] = {{36, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData546_MEANSTDDEV[] = {{40, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData547_MEANSTDDEV[] = {{44, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData548_MEANSTDDEV[] = {{48, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData549_MEANSTDDEV[] = {{52, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData550_MEANSTDDEV[] = {{56, 32, 2, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData399_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData400_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData401_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData402_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData403_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData404_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData405_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData406_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData407_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData408_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData409_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData410_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData411_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData412_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData413_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData414_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData415_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData416_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData417_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData418_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData419_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData420_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData421_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData422_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData423_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData424_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData425_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData426_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData427_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData428_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData429_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData430_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData431_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData432_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData433_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData434_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData435_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData436_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData437_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData438_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData439_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData440_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData441_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData442_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData443_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData444_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData445_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData446_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData447_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData448_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData449_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData450_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData451_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData452_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData453_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData454_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData455_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData456_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData457_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData458_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData459_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData460_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData461_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData462_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData463_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData464_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData465_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData466_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData467_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData468_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData469_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData470_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData471_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData472_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData473_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData474_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData475_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData476_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData477_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData478_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData479_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData480_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData481_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData482_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData483_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData484_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData485_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData486_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData487_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData488_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData489_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData490_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData491_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData492_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData493_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData494_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData495_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData496_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData497_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData498_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData499_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData500_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData501_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData502_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData503_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData504_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData505_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData506_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData507_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData508_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData509_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData510_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData511_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData512_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData513_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData514_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData515_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData516_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData517_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData518_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData519_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData520_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData521_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData522_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData523_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData524_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData525_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData526_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData527_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData528_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData529_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData530_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData531_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData532_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData533_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData534_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData535_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData536_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData537_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData538_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData539_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData540_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData541_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData542_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData543_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData544_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData545_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData546_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData547_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData548_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData549_MEANSTDDEV[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData550_MEANSTDDEV[] = {{0, 0}};

static acf_scenario gScenarioArray_MEANSTDDEV[] = {
{2, 1, 12, 16, 0, gScenarioBufferData0_MEANSTDDEV, 48, gScenarioKernelData0_MEANSTDDEV, 4},
{4, 1, 16, 16, 0, gScenarioBufferData1_MEANSTDDEV, 48, gScenarioKernelData1_MEANSTDDEV, 4},
{6, 1, 20, 16, 0, gScenarioBufferData2_MEANSTDDEV, 48, gScenarioKernelData2_MEANSTDDEV, 4},
{8, 1, 24, 16, 0, gScenarioBufferData3_MEANSTDDEV, 48, gScenarioKernelData3_MEANSTDDEV, 4},
{10, 1, 28, 16, 0, gScenarioBufferData4_MEANSTDDEV, 48, gScenarioKernelData4_MEANSTDDEV, 4},
{12, 1, 32, 16, 0, gScenarioBufferData5_MEANSTDDEV, 48, gScenarioKernelData5_MEANSTDDEV, 4},
{14, 1, 36, 16, 0, gScenarioBufferData6_MEANSTDDEV, 48, gScenarioKernelData6_MEANSTDDEV, 4},
{16, 1, 40, 16, 0, gScenarioBufferData7_MEANSTDDEV, 48, gScenarioKernelData7_MEANSTDDEV, 4},
{18, 1, 44, 16, 0, gScenarioBufferData8_MEANSTDDEV, 48, gScenarioKernelData8_MEANSTDDEV, 4},
{20, 1, 48, 16, 0, gScenarioBufferData9_MEANSTDDEV, 48, gScenarioKernelData9_MEANSTDDEV, 4},
{22, 1, 52, 16, 0, gScenarioBufferData10_MEANSTDDEV, 48, gScenarioKernelData10_MEANSTDDEV, 4},
{24, 1, 56, 16, 0, gScenarioBufferData11_MEANSTDDEV, 48, gScenarioKernelData11_MEANSTDDEV, 4},
{26, 1, 60, 16, 0, gScenarioBufferData12_MEANSTDDEV, 48, gScenarioKernelData12_MEANSTDDEV, 4},
{28, 1, 64, 16, 0, gScenarioBufferData13_MEANSTDDEV, 48, gScenarioKernelData13_MEANSTDDEV, 4},
{30, 1, 68, 16, 0, gScenarioBufferData14_MEANSTDDEV, 48, gScenarioKernelData14_MEANSTDDEV, 4},
{32, 1, 72, 16, 0, gScenarioBufferData15_MEANSTDDEV, 48, gScenarioKernelData15_MEANSTDDEV, 4},
{36, 1, 80, 16, 0, gScenarioBufferData16_MEANSTDDEV, 48, gScenarioKernelData16_MEANSTDDEV, 4},
{40, 1, 88, 16, 0, gScenarioBufferData17_MEANSTDDEV, 48, gScenarioKernelData17_MEANSTDDEV, 4},
{44, 1, 96, 16, 0, gScenarioBufferData18_MEANSTDDEV, 48, gScenarioKernelData18_MEANSTDDEV, 4},
{48, 1, 104, 16, 0, gScenarioBufferData19_MEANSTDDEV, 48, gScenarioKernelData19_MEANSTDDEV, 4},
{52, 1, 112, 16, 0, gScenarioBufferData20_MEANSTDDEV, 48, gScenarioKernelData20_MEANSTDDEV, 4},
{56, 1, 120, 16, 0, gScenarioBufferData21_MEANSTDDEV, 48, gScenarioKernelData21_MEANSTDDEV, 4},
{60, 1, 128, 16, 0, gScenarioBufferData22_MEANSTDDEV, 48, gScenarioKernelData22_MEANSTDDEV, 4},
{64, 1, 136, 16, 0, gScenarioBufferData23_MEANSTDDEV, 48, gScenarioKernelData23_MEANSTDDEV, 4},
{72, 1, 152, 16, 0, gScenarioBufferData24_MEANSTDDEV, 48, gScenarioKernelData24_MEANSTDDEV, 4},
{80, 1, 168, 16, 0, gScenarioBufferData25_MEANSTDDEV, 48, gScenarioKernelData25_MEANSTDDEV, 4},
{88, 1, 184, 16, 0, gScenarioBufferData26_MEANSTDDEV, 48, gScenarioKernelData26_MEANSTDDEV, 4},
{96, 1, 200, 16, 0, gScenarioBufferData27_MEANSTDDEV, 48, gScenarioKernelData27_MEANSTDDEV, 4},
{104, 1, 216, 16, 0, gScenarioBufferData28_MEANSTDDEV, 48, gScenarioKernelData28_MEANSTDDEV, 4},
{112, 1, 232, 16, 0, gScenarioBufferData29_MEANSTDDEV, 48, gScenarioKernelData29_MEANSTDDEV, 4},
{120, 1, 248, 16, 0, gScenarioBufferData30_MEANSTDDEV, 48, gScenarioKernelData30_MEANSTDDEV, 4},
{128, 1, 264, 16, 0, gScenarioBufferData31_MEANSTDDEV, 48, gScenarioKernelData31_MEANSTDDEV, 4},
{2, 2, 16, 16, 0, gScenarioBufferData32_MEANSTDDEV, 48, gScenarioKernelData32_MEANSTDDEV, 4},
{4, 2, 24, 16, 0, gScenarioBufferData33_MEANSTDDEV, 48, gScenarioKernelData33_MEANSTDDEV, 4},
{6, 2, 32, 16, 0, gScenarioBufferData34_MEANSTDDEV, 48, gScenarioKernelData34_MEANSTDDEV, 4},
{8, 2, 40, 16, 0, gScenarioBufferData35_MEANSTDDEV, 48, gScenarioKernelData35_MEANSTDDEV, 4},
{10, 2, 48, 16, 0, gScenarioBufferData36_MEANSTDDEV, 48, gScenarioKernelData36_MEANSTDDEV, 4},
{12, 2, 56, 16, 0, gScenarioBufferData37_MEANSTDDEV, 48, gScenarioKernelData37_MEANSTDDEV, 4},
{14, 2, 64, 16, 0, gScenarioBufferData38_MEANSTDDEV, 48, gScenarioKernelData38_MEANSTDDEV, 4},
{16, 2, 72, 16, 0, gScenarioBufferData39_MEANSTDDEV, 48, gScenarioKernelData39_MEANSTDDEV, 4},
{18, 2, 80, 16, 0, gScenarioBufferData40_MEANSTDDEV, 48, gScenarioKernelData40_MEANSTDDEV, 4},
{20, 2, 88, 16, 0, gScenarioBufferData41_MEANSTDDEV, 48, gScenarioKernelData41_MEANSTDDEV, 4},
{22, 2, 96, 16, 0, gScenarioBufferData42_MEANSTDDEV, 48, gScenarioKernelData42_MEANSTDDEV, 4},
{24, 2, 104, 16, 0, gScenarioBufferData43_MEANSTDDEV, 48, gScenarioKernelData43_MEANSTDDEV, 4},
{26, 2, 112, 16, 0, gScenarioBufferData44_MEANSTDDEV, 48, gScenarioKernelData44_MEANSTDDEV, 4},
{28, 2, 120, 16, 0, gScenarioBufferData45_MEANSTDDEV, 48, gScenarioKernelData45_MEANSTDDEV, 4},
{30, 2, 128, 16, 0, gScenarioBufferData46_MEANSTDDEV, 48, gScenarioKernelData46_MEANSTDDEV, 4},
{32, 2, 136, 16, 0, gScenarioBufferData47_MEANSTDDEV, 48, gScenarioKernelData47_MEANSTDDEV, 4},
{36, 2, 152, 16, 0, gScenarioBufferData48_MEANSTDDEV, 48, gScenarioKernelData48_MEANSTDDEV, 4},
{40, 2, 168, 16, 0, gScenarioBufferData49_MEANSTDDEV, 48, gScenarioKernelData49_MEANSTDDEV, 4},
{44, 2, 184, 16, 0, gScenarioBufferData50_MEANSTDDEV, 48, gScenarioKernelData50_MEANSTDDEV, 4},
{48, 2, 200, 16, 0, gScenarioBufferData51_MEANSTDDEV, 48, gScenarioKernelData51_MEANSTDDEV, 4},
{52, 2, 216, 16, 0, gScenarioBufferData52_MEANSTDDEV, 48, gScenarioKernelData52_MEANSTDDEV, 4},
{56, 2, 232, 16, 0, gScenarioBufferData53_MEANSTDDEV, 48, gScenarioKernelData53_MEANSTDDEV, 4},
{60, 2, 248, 16, 0, gScenarioBufferData54_MEANSTDDEV, 48, gScenarioKernelData54_MEANSTDDEV, 4},
{64, 2, 264, 16, 0, gScenarioBufferData55_MEANSTDDEV, 48, gScenarioKernelData55_MEANSTDDEV, 4},
{72, 2, 296, 16, 0, gScenarioBufferData56_MEANSTDDEV, 48, gScenarioKernelData56_MEANSTDDEV, 4},
{80, 2, 328, 16, 0, gScenarioBufferData57_MEANSTDDEV, 48, gScenarioKernelData57_MEANSTDDEV, 4},
{88, 2, 360, 16, 0, gScenarioBufferData58_MEANSTDDEV, 48, gScenarioKernelData58_MEANSTDDEV, 4},
{96, 2, 392, 16, 0, gScenarioBufferData59_MEANSTDDEV, 48, gScenarioKernelData59_MEANSTDDEV, 4},
{104, 2, 424, 16, 0, gScenarioBufferData60_MEANSTDDEV, 48, gScenarioKernelData60_MEANSTDDEV, 4},
{112, 2, 456, 16, 0, gScenarioBufferData61_MEANSTDDEV, 48, gScenarioKernelData61_MEANSTDDEV, 4},
{120, 2, 488, 16, 0, gScenarioBufferData62_MEANSTDDEV, 48, gScenarioKernelData62_MEANSTDDEV, 4},
{128, 2, 520, 16, 0, gScenarioBufferData63_MEANSTDDEV, 48, gScenarioKernelData63_MEANSTDDEV, 4},
{2, 3, 20, 16, 0, gScenarioBufferData64_MEANSTDDEV, 48, gScenarioKernelData64_MEANSTDDEV, 4},
{4, 3, 32, 16, 0, gScenarioBufferData65_MEANSTDDEV, 48, gScenarioKernelData65_MEANSTDDEV, 4},
{6, 3, 44, 16, 0, gScenarioBufferData66_MEANSTDDEV, 48, gScenarioKernelData66_MEANSTDDEV, 4},
{8, 3, 56, 16, 0, gScenarioBufferData67_MEANSTDDEV, 48, gScenarioKernelData67_MEANSTDDEV, 4},
{10, 3, 68, 16, 0, gScenarioBufferData68_MEANSTDDEV, 48, gScenarioKernelData68_MEANSTDDEV, 4},
{12, 3, 80, 16, 0, gScenarioBufferData69_MEANSTDDEV, 48, gScenarioKernelData69_MEANSTDDEV, 4},
{14, 3, 92, 16, 0, gScenarioBufferData70_MEANSTDDEV, 48, gScenarioKernelData70_MEANSTDDEV, 4},
{16, 3, 104, 16, 0, gScenarioBufferData71_MEANSTDDEV, 48, gScenarioKernelData71_MEANSTDDEV, 4},
{18, 3, 116, 16, 0, gScenarioBufferData72_MEANSTDDEV, 48, gScenarioKernelData72_MEANSTDDEV, 4},
{20, 3, 128, 16, 0, gScenarioBufferData73_MEANSTDDEV, 48, gScenarioKernelData73_MEANSTDDEV, 4},
{22, 3, 140, 16, 0, gScenarioBufferData74_MEANSTDDEV, 48, gScenarioKernelData74_MEANSTDDEV, 4},
{24, 3, 152, 16, 0, gScenarioBufferData75_MEANSTDDEV, 48, gScenarioKernelData75_MEANSTDDEV, 4},
{26, 3, 164, 16, 0, gScenarioBufferData76_MEANSTDDEV, 48, gScenarioKernelData76_MEANSTDDEV, 4},
{28, 3, 176, 16, 0, gScenarioBufferData77_MEANSTDDEV, 48, gScenarioKernelData77_MEANSTDDEV, 4},
{30, 3, 188, 16, 0, gScenarioBufferData78_MEANSTDDEV, 48, gScenarioKernelData78_MEANSTDDEV, 4},
{32, 3, 200, 16, 0, gScenarioBufferData79_MEANSTDDEV, 48, gScenarioKernelData79_MEANSTDDEV, 4},
{36, 3, 224, 16, 0, gScenarioBufferData80_MEANSTDDEV, 48, gScenarioKernelData80_MEANSTDDEV, 4},
{40, 3, 248, 16, 0, gScenarioBufferData81_MEANSTDDEV, 48, gScenarioKernelData81_MEANSTDDEV, 4},
{44, 3, 272, 16, 0, gScenarioBufferData82_MEANSTDDEV, 48, gScenarioKernelData82_MEANSTDDEV, 4},
{48, 3, 296, 16, 0, gScenarioBufferData83_MEANSTDDEV, 48, gScenarioKernelData83_MEANSTDDEV, 4},
{52, 3, 320, 16, 0, gScenarioBufferData84_MEANSTDDEV, 48, gScenarioKernelData84_MEANSTDDEV, 4},
{56, 3, 344, 16, 0, gScenarioBufferData85_MEANSTDDEV, 48, gScenarioKernelData85_MEANSTDDEV, 4},
{60, 3, 368, 16, 0, gScenarioBufferData86_MEANSTDDEV, 48, gScenarioKernelData86_MEANSTDDEV, 4},
{64, 3, 392, 16, 0, gScenarioBufferData87_MEANSTDDEV, 48, gScenarioKernelData87_MEANSTDDEV, 4},
{72, 3, 440, 16, 0, gScenarioBufferData88_MEANSTDDEV, 48, gScenarioKernelData88_MEANSTDDEV, 4},
{80, 3, 488, 16, 0, gScenarioBufferData89_MEANSTDDEV, 48, gScenarioKernelData89_MEANSTDDEV, 4},
{88, 3, 536, 16, 0, gScenarioBufferData90_MEANSTDDEV, 48, gScenarioKernelData90_MEANSTDDEV, 4},
{96, 3, 584, 16, 0, gScenarioBufferData91_MEANSTDDEV, 48, gScenarioKernelData91_MEANSTDDEV, 4},
{104, 3, 632, 16, 0, gScenarioBufferData92_MEANSTDDEV, 48, gScenarioKernelData92_MEANSTDDEV, 4},
{112, 3, 680, 16, 0, gScenarioBufferData93_MEANSTDDEV, 48, gScenarioKernelData93_MEANSTDDEV, 4},
{120, 3, 728, 16, 0, gScenarioBufferData94_MEANSTDDEV, 48, gScenarioKernelData94_MEANSTDDEV, 4},
{128, 3, 776, 16, 0, gScenarioBufferData95_MEANSTDDEV, 48, gScenarioKernelData95_MEANSTDDEV, 4},
{2, 4, 24, 16, 0, gScenarioBufferData96_MEANSTDDEV, 48, gScenarioKernelData96_MEANSTDDEV, 4},
{4, 4, 40, 16, 0, gScenarioBufferData97_MEANSTDDEV, 48, gScenarioKernelData97_MEANSTDDEV, 4},
{6, 4, 56, 16, 0, gScenarioBufferData98_MEANSTDDEV, 48, gScenarioKernelData98_MEANSTDDEV, 4},
{8, 4, 72, 16, 0, gScenarioBufferData99_MEANSTDDEV, 48, gScenarioKernelData99_MEANSTDDEV, 4},
{10, 4, 88, 16, 0, gScenarioBufferData100_MEANSTDDEV, 48, gScenarioKernelData100_MEANSTDDEV, 4},
{12, 4, 104, 16, 0, gScenarioBufferData101_MEANSTDDEV, 48, gScenarioKernelData101_MEANSTDDEV, 4},
{14, 4, 120, 16, 0, gScenarioBufferData102_MEANSTDDEV, 48, gScenarioKernelData102_MEANSTDDEV, 4},
{16, 4, 136, 16, 0, gScenarioBufferData103_MEANSTDDEV, 48, gScenarioKernelData103_MEANSTDDEV, 4},
{18, 4, 152, 16, 0, gScenarioBufferData104_MEANSTDDEV, 48, gScenarioKernelData104_MEANSTDDEV, 4},
{20, 4, 168, 16, 0, gScenarioBufferData105_MEANSTDDEV, 48, gScenarioKernelData105_MEANSTDDEV, 4},
{22, 4, 184, 16, 0, gScenarioBufferData106_MEANSTDDEV, 48, gScenarioKernelData106_MEANSTDDEV, 4},
{24, 4, 200, 16, 0, gScenarioBufferData107_MEANSTDDEV, 48, gScenarioKernelData107_MEANSTDDEV, 4},
{26, 4, 216, 16, 0, gScenarioBufferData108_MEANSTDDEV, 48, gScenarioKernelData108_MEANSTDDEV, 4},
{28, 4, 232, 16, 0, gScenarioBufferData109_MEANSTDDEV, 48, gScenarioKernelData109_MEANSTDDEV, 4},
{30, 4, 248, 16, 0, gScenarioBufferData110_MEANSTDDEV, 48, gScenarioKernelData110_MEANSTDDEV, 4},
{32, 4, 264, 16, 0, gScenarioBufferData111_MEANSTDDEV, 48, gScenarioKernelData111_MEANSTDDEV, 4},
{36, 4, 296, 16, 0, gScenarioBufferData112_MEANSTDDEV, 48, gScenarioKernelData112_MEANSTDDEV, 4},
{40, 4, 328, 16, 0, gScenarioBufferData113_MEANSTDDEV, 48, gScenarioKernelData113_MEANSTDDEV, 4},
{44, 4, 360, 16, 0, gScenarioBufferData114_MEANSTDDEV, 48, gScenarioKernelData114_MEANSTDDEV, 4},
{48, 4, 392, 16, 0, gScenarioBufferData115_MEANSTDDEV, 48, gScenarioKernelData115_MEANSTDDEV, 4},
{52, 4, 424, 16, 0, gScenarioBufferData116_MEANSTDDEV, 48, gScenarioKernelData116_MEANSTDDEV, 4},
{56, 4, 456, 16, 0, gScenarioBufferData117_MEANSTDDEV, 48, gScenarioKernelData117_MEANSTDDEV, 4},
{60, 4, 488, 16, 0, gScenarioBufferData118_MEANSTDDEV, 48, gScenarioKernelData118_MEANSTDDEV, 4},
{64, 4, 520, 16, 0, gScenarioBufferData119_MEANSTDDEV, 48, gScenarioKernelData119_MEANSTDDEV, 4},
{72, 4, 584, 16, 0, gScenarioBufferData120_MEANSTDDEV, 48, gScenarioKernelData120_MEANSTDDEV, 4},
{80, 4, 648, 16, 0, gScenarioBufferData121_MEANSTDDEV, 48, gScenarioKernelData121_MEANSTDDEV, 4},
{88, 4, 712, 16, 0, gScenarioBufferData122_MEANSTDDEV, 48, gScenarioKernelData122_MEANSTDDEV, 4},
{96, 4, 776, 16, 0, gScenarioBufferData123_MEANSTDDEV, 48, gScenarioKernelData123_MEANSTDDEV, 4},
{104, 4, 840, 16, 0, gScenarioBufferData124_MEANSTDDEV, 48, gScenarioKernelData124_MEANSTDDEV, 4},
{112, 4, 904, 16, 0, gScenarioBufferData125_MEANSTDDEV, 48, gScenarioKernelData125_MEANSTDDEV, 4},
{120, 4, 968, 16, 0, gScenarioBufferData126_MEANSTDDEV, 48, gScenarioKernelData126_MEANSTDDEV, 4},
{128, 4, 1032, 16, 0, gScenarioBufferData127_MEANSTDDEV, 48, gScenarioKernelData127_MEANSTDDEV, 4},
{2, 5, 28, 16, 0, gScenarioBufferData128_MEANSTDDEV, 48, gScenarioKernelData128_MEANSTDDEV, 4},
{4, 5, 48, 16, 0, gScenarioBufferData129_MEANSTDDEV, 48, gScenarioKernelData129_MEANSTDDEV, 4},
{6, 5, 68, 16, 0, gScenarioBufferData130_MEANSTDDEV, 48, gScenarioKernelData130_MEANSTDDEV, 4},
{8, 5, 88, 16, 0, gScenarioBufferData131_MEANSTDDEV, 48, gScenarioKernelData131_MEANSTDDEV, 4},
{10, 5, 108, 16, 0, gScenarioBufferData132_MEANSTDDEV, 48, gScenarioKernelData132_MEANSTDDEV, 4},
{12, 5, 128, 16, 0, gScenarioBufferData133_MEANSTDDEV, 48, gScenarioKernelData133_MEANSTDDEV, 4},
{14, 5, 148, 16, 0, gScenarioBufferData134_MEANSTDDEV, 48, gScenarioKernelData134_MEANSTDDEV, 4},
{16, 5, 168, 16, 0, gScenarioBufferData135_MEANSTDDEV, 48, gScenarioKernelData135_MEANSTDDEV, 4},
{18, 5, 188, 16, 0, gScenarioBufferData136_MEANSTDDEV, 48, gScenarioKernelData136_MEANSTDDEV, 4},
{20, 5, 208, 16, 0, gScenarioBufferData137_MEANSTDDEV, 48, gScenarioKernelData137_MEANSTDDEV, 4},
{22, 5, 228, 16, 0, gScenarioBufferData138_MEANSTDDEV, 48, gScenarioKernelData138_MEANSTDDEV, 4},
{24, 5, 248, 16, 0, gScenarioBufferData139_MEANSTDDEV, 48, gScenarioKernelData139_MEANSTDDEV, 4},
{26, 5, 268, 16, 0, gScenarioBufferData140_MEANSTDDEV, 48, gScenarioKernelData140_MEANSTDDEV, 4},
{28, 5, 288, 16, 0, gScenarioBufferData141_MEANSTDDEV, 48, gScenarioKernelData141_MEANSTDDEV, 4},
{30, 5, 308, 16, 0, gScenarioBufferData142_MEANSTDDEV, 48, gScenarioKernelData142_MEANSTDDEV, 4},
{32, 5, 328, 16, 0, gScenarioBufferData143_MEANSTDDEV, 48, gScenarioKernelData143_MEANSTDDEV, 4},
{36, 5, 368, 16, 0, gScenarioBufferData144_MEANSTDDEV, 48, gScenarioKernelData144_MEANSTDDEV, 4},
{40, 5, 408, 16, 0, gScenarioBufferData145_MEANSTDDEV, 48, gScenarioKernelData145_MEANSTDDEV, 4},
{44, 5, 448, 16, 0, gScenarioBufferData146_MEANSTDDEV, 48, gScenarioKernelData146_MEANSTDDEV, 4},
{48, 5, 488, 16, 0, gScenarioBufferData147_MEANSTDDEV, 48, gScenarioKernelData147_MEANSTDDEV, 4},
{52, 5, 528, 16, 0, gScenarioBufferData148_MEANSTDDEV, 48, gScenarioKernelData148_MEANSTDDEV, 4},
{56, 5, 568, 16, 0, gScenarioBufferData149_MEANSTDDEV, 48, gScenarioKernelData149_MEANSTDDEV, 4},
{60, 5, 608, 16, 0, gScenarioBufferData150_MEANSTDDEV, 48, gScenarioKernelData150_MEANSTDDEV, 4},
{64, 5, 648, 16, 0, gScenarioBufferData151_MEANSTDDEV, 48, gScenarioKernelData151_MEANSTDDEV, 4},
{72, 5, 728, 16, 0, gScenarioBufferData152_MEANSTDDEV, 48, gScenarioKernelData152_MEANSTDDEV, 4},
{80, 5, 808, 16, 0, gScenarioBufferData153_MEANSTDDEV, 48, gScenarioKernelData153_MEANSTDDEV, 4},
{88, 5, 888, 16, 0, gScenarioBufferData154_MEANSTDDEV, 48, gScenarioKernelData154_MEANSTDDEV, 4},
{96, 5, 968, 16, 0, gScenarioBufferData155_MEANSTDDEV, 48, gScenarioKernelData155_MEANSTDDEV, 4},
{104, 5, 1048, 16, 0, gScenarioBufferData156_MEANSTDDEV, 48, gScenarioKernelData156_MEANSTDDEV, 4},
{112, 5, 1128, 16, 0, gScenarioBufferData157_MEANSTDDEV, 48, gScenarioKernelData157_MEANSTDDEV, 4},
{120, 5, 1208, 16, 0, gScenarioBufferData158_MEANSTDDEV, 48, gScenarioKernelData158_MEANSTDDEV, 4},
{128, 5, 1288, 16, 0, gScenarioBufferData159_MEANSTDDEV, 48, gScenarioKernelData159_MEANSTDDEV, 4},
{2, 6, 32, 16, 0, gScenarioBufferData160_MEANSTDDEV, 48, gScenarioKernelData160_MEANSTDDEV, 4},
{4, 6, 56, 16, 0, gScenarioBufferData161_MEANSTDDEV, 48, gScenarioKernelData161_MEANSTDDEV, 4},
{6, 6, 80, 16, 0, gScenarioBufferData162_MEANSTDDEV, 48, gScenarioKernelData162_MEANSTDDEV, 4},
{8, 6, 104, 16, 0, gScenarioBufferData163_MEANSTDDEV, 48, gScenarioKernelData163_MEANSTDDEV, 4},
{10, 6, 128, 16, 0, gScenarioBufferData164_MEANSTDDEV, 48, gScenarioKernelData164_MEANSTDDEV, 4},
{12, 6, 152, 16, 0, gScenarioBufferData165_MEANSTDDEV, 48, gScenarioKernelData165_MEANSTDDEV, 4},
{14, 6, 176, 16, 0, gScenarioBufferData166_MEANSTDDEV, 48, gScenarioKernelData166_MEANSTDDEV, 4},
{16, 6, 200, 16, 0, gScenarioBufferData167_MEANSTDDEV, 48, gScenarioKernelData167_MEANSTDDEV, 4},
{18, 6, 224, 16, 0, gScenarioBufferData168_MEANSTDDEV, 48, gScenarioKernelData168_MEANSTDDEV, 4},
{20, 6, 248, 16, 0, gScenarioBufferData169_MEANSTDDEV, 48, gScenarioKernelData169_MEANSTDDEV, 4},
{22, 6, 272, 16, 0, gScenarioBufferData170_MEANSTDDEV, 48, gScenarioKernelData170_MEANSTDDEV, 4},
{24, 6, 296, 16, 0, gScenarioBufferData171_MEANSTDDEV, 48, gScenarioKernelData171_MEANSTDDEV, 4},
{26, 6, 320, 16, 0, gScenarioBufferData172_MEANSTDDEV, 48, gScenarioKernelData172_MEANSTDDEV, 4},
{28, 6, 344, 16, 0, gScenarioBufferData173_MEANSTDDEV, 48, gScenarioKernelData173_MEANSTDDEV, 4},
{30, 6, 368, 16, 0, gScenarioBufferData174_MEANSTDDEV, 48, gScenarioKernelData174_MEANSTDDEV, 4},
{32, 6, 392, 16, 0, gScenarioBufferData175_MEANSTDDEV, 48, gScenarioKernelData175_MEANSTDDEV, 4},
{36, 6, 440, 16, 0, gScenarioBufferData176_MEANSTDDEV, 48, gScenarioKernelData176_MEANSTDDEV, 4},
{40, 6, 488, 16, 0, gScenarioBufferData177_MEANSTDDEV, 48, gScenarioKernelData177_MEANSTDDEV, 4},
{44, 6, 536, 16, 0, gScenarioBufferData178_MEANSTDDEV, 48, gScenarioKernelData178_MEANSTDDEV, 4},
{48, 6, 584, 16, 0, gScenarioBufferData179_MEANSTDDEV, 48, gScenarioKernelData179_MEANSTDDEV, 4},
{52, 6, 632, 16, 0, gScenarioBufferData180_MEANSTDDEV, 48, gScenarioKernelData180_MEANSTDDEV, 4},
{56, 6, 680, 16, 0, gScenarioBufferData181_MEANSTDDEV, 48, gScenarioKernelData181_MEANSTDDEV, 4},
{60, 6, 728, 16, 0, gScenarioBufferData182_MEANSTDDEV, 48, gScenarioKernelData182_MEANSTDDEV, 4},
{64, 6, 776, 16, 0, gScenarioBufferData183_MEANSTDDEV, 48, gScenarioKernelData183_MEANSTDDEV, 4},
{72, 6, 872, 16, 0, gScenarioBufferData184_MEANSTDDEV, 48, gScenarioKernelData184_MEANSTDDEV, 4},
{80, 6, 968, 16, 0, gScenarioBufferData185_MEANSTDDEV, 48, gScenarioKernelData185_MEANSTDDEV, 4},
{88, 6, 1064, 16, 0, gScenarioBufferData186_MEANSTDDEV, 48, gScenarioKernelData186_MEANSTDDEV, 4},
{96, 6, 1160, 16, 0, gScenarioBufferData187_MEANSTDDEV, 48, gScenarioKernelData187_MEANSTDDEV, 4},
{104, 6, 1256, 16, 0, gScenarioBufferData188_MEANSTDDEV, 48, gScenarioKernelData188_MEANSTDDEV, 4},
{112, 6, 1352, 16, 0, gScenarioBufferData189_MEANSTDDEV, 48, gScenarioKernelData189_MEANSTDDEV, 4},
{120, 6, 1448, 16, 0, gScenarioBufferData190_MEANSTDDEV, 48, gScenarioKernelData190_MEANSTDDEV, 4},
{128, 6, 1544, 16, 0, gScenarioBufferData191_MEANSTDDEV, 48, gScenarioKernelData191_MEANSTDDEV, 4},
{2, 8, 40, 16, 0, gScenarioBufferData192_MEANSTDDEV, 48, gScenarioKernelData192_MEANSTDDEV, 4},
{4, 8, 72, 16, 0, gScenarioBufferData193_MEANSTDDEV, 48, gScenarioKernelData193_MEANSTDDEV, 4},
{6, 8, 104, 16, 0, gScenarioBufferData194_MEANSTDDEV, 48, gScenarioKernelData194_MEANSTDDEV, 4},
{8, 8, 136, 16, 0, gScenarioBufferData195_MEANSTDDEV, 48, gScenarioKernelData195_MEANSTDDEV, 4},
{10, 8, 168, 16, 0, gScenarioBufferData196_MEANSTDDEV, 48, gScenarioKernelData196_MEANSTDDEV, 4},
{12, 8, 200, 16, 0, gScenarioBufferData197_MEANSTDDEV, 48, gScenarioKernelData197_MEANSTDDEV, 4},
{14, 8, 232, 16, 0, gScenarioBufferData198_MEANSTDDEV, 48, gScenarioKernelData198_MEANSTDDEV, 4},
{16, 8, 264, 16, 0, gScenarioBufferData199_MEANSTDDEV, 48, gScenarioKernelData199_MEANSTDDEV, 4},
{18, 8, 296, 16, 0, gScenarioBufferData200_MEANSTDDEV, 48, gScenarioKernelData200_MEANSTDDEV, 4},
{20, 8, 328, 16, 0, gScenarioBufferData201_MEANSTDDEV, 48, gScenarioKernelData201_MEANSTDDEV, 4},
{22, 8, 360, 16, 0, gScenarioBufferData202_MEANSTDDEV, 48, gScenarioKernelData202_MEANSTDDEV, 4},
{24, 8, 392, 16, 0, gScenarioBufferData203_MEANSTDDEV, 48, gScenarioKernelData203_MEANSTDDEV, 4},
{26, 8, 424, 16, 0, gScenarioBufferData204_MEANSTDDEV, 48, gScenarioKernelData204_MEANSTDDEV, 4},
{28, 8, 456, 16, 0, gScenarioBufferData205_MEANSTDDEV, 48, gScenarioKernelData205_MEANSTDDEV, 4},
{30, 8, 488, 16, 0, gScenarioBufferData206_MEANSTDDEV, 48, gScenarioKernelData206_MEANSTDDEV, 4},
{32, 8, 520, 16, 0, gScenarioBufferData207_MEANSTDDEV, 48, gScenarioKernelData207_MEANSTDDEV, 4},
{36, 8, 584, 16, 0, gScenarioBufferData208_MEANSTDDEV, 48, gScenarioKernelData208_MEANSTDDEV, 4},
{40, 8, 648, 16, 0, gScenarioBufferData209_MEANSTDDEV, 48, gScenarioKernelData209_MEANSTDDEV, 4},
{44, 8, 712, 16, 0, gScenarioBufferData210_MEANSTDDEV, 48, gScenarioKernelData210_MEANSTDDEV, 4},
{48, 8, 776, 16, 0, gScenarioBufferData211_MEANSTDDEV, 48, gScenarioKernelData211_MEANSTDDEV, 4},
{52, 8, 840, 16, 0, gScenarioBufferData212_MEANSTDDEV, 48, gScenarioKernelData212_MEANSTDDEV, 4},
{56, 8, 904, 16, 0, gScenarioBufferData213_MEANSTDDEV, 48, gScenarioKernelData213_MEANSTDDEV, 4},
{60, 8, 968, 16, 0, gScenarioBufferData214_MEANSTDDEV, 48, gScenarioKernelData214_MEANSTDDEV, 4},
{64, 8, 1032, 16, 0, gScenarioBufferData215_MEANSTDDEV, 48, gScenarioKernelData215_MEANSTDDEV, 4},
{72, 8, 1160, 16, 0, gScenarioBufferData216_MEANSTDDEV, 48, gScenarioKernelData216_MEANSTDDEV, 4},
{80, 8, 1288, 16, 0, gScenarioBufferData217_MEANSTDDEV, 48, gScenarioKernelData217_MEANSTDDEV, 4},
{88, 8, 1416, 16, 0, gScenarioBufferData218_MEANSTDDEV, 48, gScenarioKernelData218_MEANSTDDEV, 4},
{96, 8, 1544, 16, 0, gScenarioBufferData219_MEANSTDDEV, 48, gScenarioKernelData219_MEANSTDDEV, 4},
{104, 8, 1672, 16, 0, gScenarioBufferData220_MEANSTDDEV, 48, gScenarioKernelData220_MEANSTDDEV, 4},
{112, 8, 1800, 16, 0, gScenarioBufferData221_MEANSTDDEV, 48, gScenarioKernelData221_MEANSTDDEV, 4},
{120, 8, 1928, 16, 0, gScenarioBufferData222_MEANSTDDEV, 48, gScenarioKernelData222_MEANSTDDEV, 4},
{128, 8, 2056, 16, 0, gScenarioBufferData223_MEANSTDDEV, 48, gScenarioKernelData223_MEANSTDDEV, 4},
{2, 10, 48, 16, 0, gScenarioBufferData224_MEANSTDDEV, 48, gScenarioKernelData224_MEANSTDDEV, 4},
{4, 10, 88, 16, 0, gScenarioBufferData225_MEANSTDDEV, 48, gScenarioKernelData225_MEANSTDDEV, 4},
{6, 10, 128, 16, 0, gScenarioBufferData226_MEANSTDDEV, 48, gScenarioKernelData226_MEANSTDDEV, 4},
{8, 10, 168, 16, 0, gScenarioBufferData227_MEANSTDDEV, 48, gScenarioKernelData227_MEANSTDDEV, 4},
{10, 10, 208, 16, 0, gScenarioBufferData228_MEANSTDDEV, 48, gScenarioKernelData228_MEANSTDDEV, 4},
{12, 10, 248, 16, 0, gScenarioBufferData229_MEANSTDDEV, 48, gScenarioKernelData229_MEANSTDDEV, 4},
{14, 10, 288, 16, 0, gScenarioBufferData230_MEANSTDDEV, 48, gScenarioKernelData230_MEANSTDDEV, 4},
{16, 10, 328, 16, 0, gScenarioBufferData231_MEANSTDDEV, 48, gScenarioKernelData231_MEANSTDDEV, 4},
{18, 10, 368, 16, 0, gScenarioBufferData232_MEANSTDDEV, 48, gScenarioKernelData232_MEANSTDDEV, 4},
{20, 10, 408, 16, 0, gScenarioBufferData233_MEANSTDDEV, 48, gScenarioKernelData233_MEANSTDDEV, 4},
{22, 10, 448, 16, 0, gScenarioBufferData234_MEANSTDDEV, 48, gScenarioKernelData234_MEANSTDDEV, 4},
{24, 10, 488, 16, 0, gScenarioBufferData235_MEANSTDDEV, 48, gScenarioKernelData235_MEANSTDDEV, 4},
{26, 10, 528, 16, 0, gScenarioBufferData236_MEANSTDDEV, 48, gScenarioKernelData236_MEANSTDDEV, 4},
{28, 10, 568, 16, 0, gScenarioBufferData237_MEANSTDDEV, 48, gScenarioKernelData237_MEANSTDDEV, 4},
{30, 10, 608, 16, 0, gScenarioBufferData238_MEANSTDDEV, 48, gScenarioKernelData238_MEANSTDDEV, 4},
{32, 10, 648, 16, 0, gScenarioBufferData239_MEANSTDDEV, 48, gScenarioKernelData239_MEANSTDDEV, 4},
{36, 10, 728, 16, 0, gScenarioBufferData240_MEANSTDDEV, 48, gScenarioKernelData240_MEANSTDDEV, 4},
{40, 10, 808, 16, 0, gScenarioBufferData241_MEANSTDDEV, 48, gScenarioKernelData241_MEANSTDDEV, 4},
{44, 10, 888, 16, 0, gScenarioBufferData242_MEANSTDDEV, 48, gScenarioKernelData242_MEANSTDDEV, 4},
{48, 10, 968, 16, 0, gScenarioBufferData243_MEANSTDDEV, 48, gScenarioKernelData243_MEANSTDDEV, 4},
{52, 10, 1048, 16, 0, gScenarioBufferData244_MEANSTDDEV, 48, gScenarioKernelData244_MEANSTDDEV, 4},
{56, 10, 1128, 16, 0, gScenarioBufferData245_MEANSTDDEV, 48, gScenarioKernelData245_MEANSTDDEV, 4},
{60, 10, 1208, 16, 0, gScenarioBufferData246_MEANSTDDEV, 48, gScenarioKernelData246_MEANSTDDEV, 4},
{64, 10, 1288, 16, 0, gScenarioBufferData247_MEANSTDDEV, 48, gScenarioKernelData247_MEANSTDDEV, 4},
{72, 10, 1448, 16, 0, gScenarioBufferData248_MEANSTDDEV, 48, gScenarioKernelData248_MEANSTDDEV, 4},
{80, 10, 1608, 16, 0, gScenarioBufferData249_MEANSTDDEV, 48, gScenarioKernelData249_MEANSTDDEV, 4},
{88, 10, 1768, 16, 0, gScenarioBufferData250_MEANSTDDEV, 48, gScenarioKernelData250_MEANSTDDEV, 4},
{96, 10, 1928, 16, 0, gScenarioBufferData251_MEANSTDDEV, 48, gScenarioKernelData251_MEANSTDDEV, 4},
{104, 10, 2088, 16, 0, gScenarioBufferData252_MEANSTDDEV, 48, gScenarioKernelData252_MEANSTDDEV, 4},
{112, 10, 2248, 16, 0, gScenarioBufferData253_MEANSTDDEV, 48, gScenarioKernelData253_MEANSTDDEV, 4},
{120, 10, 2408, 16, 0, gScenarioBufferData254_MEANSTDDEV, 48, gScenarioKernelData254_MEANSTDDEV, 4},
{128, 10, 2568, 16, 0, gScenarioBufferData255_MEANSTDDEV, 48, gScenarioKernelData255_MEANSTDDEV, 4},
{2, 12, 56, 16, 0, gScenarioBufferData256_MEANSTDDEV, 48, gScenarioKernelData256_MEANSTDDEV, 4},
{4, 12, 104, 16, 0, gScenarioBufferData257_MEANSTDDEV, 48, gScenarioKernelData257_MEANSTDDEV, 4},
{6, 12, 152, 16, 0, gScenarioBufferData258_MEANSTDDEV, 48, gScenarioKernelData258_MEANSTDDEV, 4},
{8, 12, 200, 16, 0, gScenarioBufferData259_MEANSTDDEV, 48, gScenarioKernelData259_MEANSTDDEV, 4},
{10, 12, 248, 16, 0, gScenarioBufferData260_MEANSTDDEV, 48, gScenarioKernelData260_MEANSTDDEV, 4},
{12, 12, 296, 16, 0, gScenarioBufferData261_MEANSTDDEV, 48, gScenarioKernelData261_MEANSTDDEV, 4},
{14, 12, 344, 16, 0, gScenarioBufferData262_MEANSTDDEV, 48, gScenarioKernelData262_MEANSTDDEV, 4},
{16, 12, 392, 16, 0, gScenarioBufferData263_MEANSTDDEV, 48, gScenarioKernelData263_MEANSTDDEV, 4},
{18, 12, 440, 16, 0, gScenarioBufferData264_MEANSTDDEV, 48, gScenarioKernelData264_MEANSTDDEV, 4},
{20, 12, 488, 16, 0, gScenarioBufferData265_MEANSTDDEV, 48, gScenarioKernelData265_MEANSTDDEV, 4},
{22, 12, 536, 16, 0, gScenarioBufferData266_MEANSTDDEV, 48, gScenarioKernelData266_MEANSTDDEV, 4},
{24, 12, 584, 16, 0, gScenarioBufferData267_MEANSTDDEV, 48, gScenarioKernelData267_MEANSTDDEV, 4},
{26, 12, 632, 16, 0, gScenarioBufferData268_MEANSTDDEV, 48, gScenarioKernelData268_MEANSTDDEV, 4},
{28, 12, 680, 16, 0, gScenarioBufferData269_MEANSTDDEV, 48, gScenarioKernelData269_MEANSTDDEV, 4},
{30, 12, 728, 16, 0, gScenarioBufferData270_MEANSTDDEV, 48, gScenarioKernelData270_MEANSTDDEV, 4},
{32, 12, 776, 16, 0, gScenarioBufferData271_MEANSTDDEV, 48, gScenarioKernelData271_MEANSTDDEV, 4},
{36, 12, 872, 16, 0, gScenarioBufferData272_MEANSTDDEV, 48, gScenarioKernelData272_MEANSTDDEV, 4},
{40, 12, 968, 16, 0, gScenarioBufferData273_MEANSTDDEV, 48, gScenarioKernelData273_MEANSTDDEV, 4},
{44, 12, 1064, 16, 0, gScenarioBufferData274_MEANSTDDEV, 48, gScenarioKernelData274_MEANSTDDEV, 4},
{48, 12, 1160, 16, 0, gScenarioBufferData275_MEANSTDDEV, 48, gScenarioKernelData275_MEANSTDDEV, 4},
{52, 12, 1256, 16, 0, gScenarioBufferData276_MEANSTDDEV, 48, gScenarioKernelData276_MEANSTDDEV, 4},
{56, 12, 1352, 16, 0, gScenarioBufferData277_MEANSTDDEV, 48, gScenarioKernelData277_MEANSTDDEV, 4},
{60, 12, 1448, 16, 0, gScenarioBufferData278_MEANSTDDEV, 48, gScenarioKernelData278_MEANSTDDEV, 4},
{64, 12, 1544, 16, 0, gScenarioBufferData279_MEANSTDDEV, 48, gScenarioKernelData279_MEANSTDDEV, 4},
{72, 12, 1736, 16, 0, gScenarioBufferData280_MEANSTDDEV, 48, gScenarioKernelData280_MEANSTDDEV, 4},
{80, 12, 1928, 16, 0, gScenarioBufferData281_MEANSTDDEV, 48, gScenarioKernelData281_MEANSTDDEV, 4},
{88, 12, 2120, 16, 0, gScenarioBufferData282_MEANSTDDEV, 48, gScenarioKernelData282_MEANSTDDEV, 4},
{96, 12, 2312, 16, 0, gScenarioBufferData283_MEANSTDDEV, 48, gScenarioKernelData283_MEANSTDDEV, 4},
{104, 12, 2504, 16, 0, gScenarioBufferData284_MEANSTDDEV, 48, gScenarioKernelData284_MEANSTDDEV, 4},
{112, 12, 2696, 16, 0, gScenarioBufferData285_MEANSTDDEV, 48, gScenarioKernelData285_MEANSTDDEV, 4},
{120, 12, 2888, 16, 0, gScenarioBufferData286_MEANSTDDEV, 48, gScenarioKernelData286_MEANSTDDEV, 4},
{128, 12, 3080, 16, 0, gScenarioBufferData287_MEANSTDDEV, 48, gScenarioKernelData287_MEANSTDDEV, 4},
{2, 14, 64, 16, 0, gScenarioBufferData288_MEANSTDDEV, 48, gScenarioKernelData288_MEANSTDDEV, 4},
{4, 14, 120, 16, 0, gScenarioBufferData289_MEANSTDDEV, 48, gScenarioKernelData289_MEANSTDDEV, 4},
{6, 14, 176, 16, 0, gScenarioBufferData290_MEANSTDDEV, 48, gScenarioKernelData290_MEANSTDDEV, 4},
{8, 14, 232, 16, 0, gScenarioBufferData291_MEANSTDDEV, 48, gScenarioKernelData291_MEANSTDDEV, 4},
{10, 14, 288, 16, 0, gScenarioBufferData292_MEANSTDDEV, 48, gScenarioKernelData292_MEANSTDDEV, 4},
{12, 14, 344, 16, 0, gScenarioBufferData293_MEANSTDDEV, 48, gScenarioKernelData293_MEANSTDDEV, 4},
{14, 14, 400, 16, 0, gScenarioBufferData294_MEANSTDDEV, 48, gScenarioKernelData294_MEANSTDDEV, 4},
{16, 14, 456, 16, 0, gScenarioBufferData295_MEANSTDDEV, 48, gScenarioKernelData295_MEANSTDDEV, 4},
{18, 14, 512, 16, 0, gScenarioBufferData296_MEANSTDDEV, 48, gScenarioKernelData296_MEANSTDDEV, 4},
{20, 14, 568, 16, 0, gScenarioBufferData297_MEANSTDDEV, 48, gScenarioKernelData297_MEANSTDDEV, 4},
{22, 14, 624, 16, 0, gScenarioBufferData298_MEANSTDDEV, 48, gScenarioKernelData298_MEANSTDDEV, 4},
{24, 14, 680, 16, 0, gScenarioBufferData299_MEANSTDDEV, 48, gScenarioKernelData299_MEANSTDDEV, 4},
{26, 14, 736, 16, 0, gScenarioBufferData300_MEANSTDDEV, 48, gScenarioKernelData300_MEANSTDDEV, 4},
{28, 14, 792, 16, 0, gScenarioBufferData301_MEANSTDDEV, 48, gScenarioKernelData301_MEANSTDDEV, 4},
{30, 14, 848, 16, 0, gScenarioBufferData302_MEANSTDDEV, 48, gScenarioKernelData302_MEANSTDDEV, 4},
{32, 14, 904, 16, 0, gScenarioBufferData303_MEANSTDDEV, 48, gScenarioKernelData303_MEANSTDDEV, 4},
{36, 14, 1016, 16, 0, gScenarioBufferData304_MEANSTDDEV, 48, gScenarioKernelData304_MEANSTDDEV, 4},
{40, 14, 1128, 16, 0, gScenarioBufferData305_MEANSTDDEV, 48, gScenarioKernelData305_MEANSTDDEV, 4},
{44, 14, 1240, 16, 0, gScenarioBufferData306_MEANSTDDEV, 48, gScenarioKernelData306_MEANSTDDEV, 4},
{48, 14, 1352, 16, 0, gScenarioBufferData307_MEANSTDDEV, 48, gScenarioKernelData307_MEANSTDDEV, 4},
{52, 14, 1464, 16, 0, gScenarioBufferData308_MEANSTDDEV, 48, gScenarioKernelData308_MEANSTDDEV, 4},
{56, 14, 1576, 16, 0, gScenarioBufferData309_MEANSTDDEV, 48, gScenarioKernelData309_MEANSTDDEV, 4},
{60, 14, 1688, 16, 0, gScenarioBufferData310_MEANSTDDEV, 48, gScenarioKernelData310_MEANSTDDEV, 4},
{64, 14, 1800, 16, 0, gScenarioBufferData311_MEANSTDDEV, 48, gScenarioKernelData311_MEANSTDDEV, 4},
{72, 14, 2024, 16, 0, gScenarioBufferData312_MEANSTDDEV, 48, gScenarioKernelData312_MEANSTDDEV, 4},
{80, 14, 2248, 16, 0, gScenarioBufferData313_MEANSTDDEV, 48, gScenarioKernelData313_MEANSTDDEV, 4},
{88, 14, 2472, 16, 0, gScenarioBufferData314_MEANSTDDEV, 48, gScenarioKernelData314_MEANSTDDEV, 4},
{96, 14, 2696, 16, 0, gScenarioBufferData315_MEANSTDDEV, 48, gScenarioKernelData315_MEANSTDDEV, 4},
{104, 14, 2920, 16, 0, gScenarioBufferData316_MEANSTDDEV, 48, gScenarioKernelData316_MEANSTDDEV, 4},
{112, 14, 3144, 16, 0, gScenarioBufferData317_MEANSTDDEV, 48, gScenarioKernelData317_MEANSTDDEV, 4},
{120, 14, 3368, 16, 0, gScenarioBufferData318_MEANSTDDEV, 48, gScenarioKernelData318_MEANSTDDEV, 4},
{128, 14, 3592, 16, 0, gScenarioBufferData319_MEANSTDDEV, 48, gScenarioKernelData319_MEANSTDDEV, 4},
{2, 16, 72, 16, 0, gScenarioBufferData320_MEANSTDDEV, 48, gScenarioKernelData320_MEANSTDDEV, 4},
{4, 16, 136, 16, 0, gScenarioBufferData321_MEANSTDDEV, 48, gScenarioKernelData321_MEANSTDDEV, 4},
{6, 16, 200, 16, 0, gScenarioBufferData322_MEANSTDDEV, 48, gScenarioKernelData322_MEANSTDDEV, 4},
{8, 16, 264, 16, 0, gScenarioBufferData323_MEANSTDDEV, 48, gScenarioKernelData323_MEANSTDDEV, 4},
{10, 16, 328, 16, 0, gScenarioBufferData324_MEANSTDDEV, 48, gScenarioKernelData324_MEANSTDDEV, 4},
{12, 16, 392, 16, 0, gScenarioBufferData325_MEANSTDDEV, 48, gScenarioKernelData325_MEANSTDDEV, 4},
{14, 16, 456, 16, 0, gScenarioBufferData326_MEANSTDDEV, 48, gScenarioKernelData326_MEANSTDDEV, 4},
{16, 16, 520, 16, 0, gScenarioBufferData327_MEANSTDDEV, 48, gScenarioKernelData327_MEANSTDDEV, 4},
{18, 16, 584, 16, 0, gScenarioBufferData328_MEANSTDDEV, 48, gScenarioKernelData328_MEANSTDDEV, 4},
{20, 16, 648, 16, 0, gScenarioBufferData329_MEANSTDDEV, 48, gScenarioKernelData329_MEANSTDDEV, 4},
{22, 16, 712, 16, 0, gScenarioBufferData330_MEANSTDDEV, 48, gScenarioKernelData330_MEANSTDDEV, 4},
{24, 16, 776, 16, 0, gScenarioBufferData331_MEANSTDDEV, 48, gScenarioKernelData331_MEANSTDDEV, 4},
{26, 16, 840, 16, 0, gScenarioBufferData332_MEANSTDDEV, 48, gScenarioKernelData332_MEANSTDDEV, 4},
{28, 16, 904, 16, 0, gScenarioBufferData333_MEANSTDDEV, 48, gScenarioKernelData333_MEANSTDDEV, 4},
{30, 16, 968, 16, 0, gScenarioBufferData334_MEANSTDDEV, 48, gScenarioKernelData334_MEANSTDDEV, 4},
{32, 16, 1032, 16, 0, gScenarioBufferData335_MEANSTDDEV, 48, gScenarioKernelData335_MEANSTDDEV, 4},
{36, 16, 1160, 16, 0, gScenarioBufferData336_MEANSTDDEV, 48, gScenarioKernelData336_MEANSTDDEV, 4},
{40, 16, 1288, 16, 0, gScenarioBufferData337_MEANSTDDEV, 48, gScenarioKernelData337_MEANSTDDEV, 4},
{44, 16, 1416, 16, 0, gScenarioBufferData338_MEANSTDDEV, 48, gScenarioKernelData338_MEANSTDDEV, 4},
{48, 16, 1544, 16, 0, gScenarioBufferData339_MEANSTDDEV, 48, gScenarioKernelData339_MEANSTDDEV, 4},
{52, 16, 1672, 16, 0, gScenarioBufferData340_MEANSTDDEV, 48, gScenarioKernelData340_MEANSTDDEV, 4},
{56, 16, 1800, 16, 0, gScenarioBufferData341_MEANSTDDEV, 48, gScenarioKernelData341_MEANSTDDEV, 4},
{60, 16, 1928, 16, 0, gScenarioBufferData342_MEANSTDDEV, 48, gScenarioKernelData342_MEANSTDDEV, 4},
{64, 16, 2056, 16, 0, gScenarioBufferData343_MEANSTDDEV, 48, gScenarioKernelData343_MEANSTDDEV, 4},
{72, 16, 2312, 16, 0, gScenarioBufferData344_MEANSTDDEV, 48, gScenarioKernelData344_MEANSTDDEV, 4},
{80, 16, 2568, 16, 0, gScenarioBufferData345_MEANSTDDEV, 48, gScenarioKernelData345_MEANSTDDEV, 4},
{88, 16, 2824, 16, 0, gScenarioBufferData346_MEANSTDDEV, 48, gScenarioKernelData346_MEANSTDDEV, 4},
{96, 16, 3080, 16, 0, gScenarioBufferData347_MEANSTDDEV, 48, gScenarioKernelData347_MEANSTDDEV, 4},
{104, 16, 3336, 16, 0, gScenarioBufferData348_MEANSTDDEV, 48, gScenarioKernelData348_MEANSTDDEV, 4},
{112, 16, 3592, 16, 0, gScenarioBufferData349_MEANSTDDEV, 48, gScenarioKernelData349_MEANSTDDEV, 4},
{2, 18, 80, 16, 0, gScenarioBufferData350_MEANSTDDEV, 48, gScenarioKernelData350_MEANSTDDEV, 4},
{4, 18, 152, 16, 0, gScenarioBufferData351_MEANSTDDEV, 48, gScenarioKernelData351_MEANSTDDEV, 4},
{6, 18, 224, 16, 0, gScenarioBufferData352_MEANSTDDEV, 48, gScenarioKernelData352_MEANSTDDEV, 4},
{8, 18, 296, 16, 0, gScenarioBufferData353_MEANSTDDEV, 48, gScenarioKernelData353_MEANSTDDEV, 4},
{10, 18, 368, 16, 0, gScenarioBufferData354_MEANSTDDEV, 48, gScenarioKernelData354_MEANSTDDEV, 4},
{12, 18, 440, 16, 0, gScenarioBufferData355_MEANSTDDEV, 48, gScenarioKernelData355_MEANSTDDEV, 4},
{14, 18, 512, 16, 0, gScenarioBufferData356_MEANSTDDEV, 48, gScenarioKernelData356_MEANSTDDEV, 4},
{16, 18, 584, 16, 0, gScenarioBufferData357_MEANSTDDEV, 48, gScenarioKernelData357_MEANSTDDEV, 4},
{18, 18, 656, 16, 0, gScenarioBufferData358_MEANSTDDEV, 48, gScenarioKernelData358_MEANSTDDEV, 4},
{20, 18, 728, 16, 0, gScenarioBufferData359_MEANSTDDEV, 48, gScenarioKernelData359_MEANSTDDEV, 4},
{22, 18, 800, 16, 0, gScenarioBufferData360_MEANSTDDEV, 48, gScenarioKernelData360_MEANSTDDEV, 4},
{24, 18, 872, 16, 0, gScenarioBufferData361_MEANSTDDEV, 48, gScenarioKernelData361_MEANSTDDEV, 4},
{26, 18, 944, 16, 0, gScenarioBufferData362_MEANSTDDEV, 48, gScenarioKernelData362_MEANSTDDEV, 4},
{28, 18, 1016, 16, 0, gScenarioBufferData363_MEANSTDDEV, 48, gScenarioKernelData363_MEANSTDDEV, 4},
{30, 18, 1088, 16, 0, gScenarioBufferData364_MEANSTDDEV, 48, gScenarioKernelData364_MEANSTDDEV, 4},
{32, 18, 1160, 16, 0, gScenarioBufferData365_MEANSTDDEV, 48, gScenarioKernelData365_MEANSTDDEV, 4},
{36, 18, 1304, 16, 0, gScenarioBufferData366_MEANSTDDEV, 48, gScenarioKernelData366_MEANSTDDEV, 4},
{40, 18, 1448, 16, 0, gScenarioBufferData367_MEANSTDDEV, 48, gScenarioKernelData367_MEANSTDDEV, 4},
{44, 18, 1592, 16, 0, gScenarioBufferData368_MEANSTDDEV, 48, gScenarioKernelData368_MEANSTDDEV, 4},
{48, 18, 1736, 16, 0, gScenarioBufferData369_MEANSTDDEV, 48, gScenarioKernelData369_MEANSTDDEV, 4},
{52, 18, 1880, 16, 0, gScenarioBufferData370_MEANSTDDEV, 48, gScenarioKernelData370_MEANSTDDEV, 4},
{56, 18, 2024, 16, 0, gScenarioBufferData371_MEANSTDDEV, 48, gScenarioKernelData371_MEANSTDDEV, 4},
{60, 18, 2168, 16, 0, gScenarioBufferData372_MEANSTDDEV, 48, gScenarioKernelData372_MEANSTDDEV, 4},
{64, 18, 2312, 16, 0, gScenarioBufferData373_MEANSTDDEV, 48, gScenarioKernelData373_MEANSTDDEV, 4},
{72, 18, 2600, 16, 0, gScenarioBufferData374_MEANSTDDEV, 48, gScenarioKernelData374_MEANSTDDEV, 4},
{80, 18, 2888, 16, 0, gScenarioBufferData375_MEANSTDDEV, 48, gScenarioKernelData375_MEANSTDDEV, 4},
{88, 18, 3176, 16, 0, gScenarioBufferData376_MEANSTDDEV, 48, gScenarioKernelData376_MEANSTDDEV, 4},
{96, 18, 3464, 16, 0, gScenarioBufferData377_MEANSTDDEV, 48, gScenarioKernelData377_MEANSTDDEV, 4},
{104, 18, 3752, 16, 0, gScenarioBufferData378_MEANSTDDEV, 48, gScenarioKernelData378_MEANSTDDEV, 4},
{2, 20, 88, 16, 0, gScenarioBufferData379_MEANSTDDEV, 48, gScenarioKernelData379_MEANSTDDEV, 4},
{4, 20, 168, 16, 0, gScenarioBufferData380_MEANSTDDEV, 48, gScenarioKernelData380_MEANSTDDEV, 4},
{6, 20, 248, 16, 0, gScenarioBufferData381_MEANSTDDEV, 48, gScenarioKernelData381_MEANSTDDEV, 4},
{8, 20, 328, 16, 0, gScenarioBufferData382_MEANSTDDEV, 48, gScenarioKernelData382_MEANSTDDEV, 4},
{10, 20, 408, 16, 0, gScenarioBufferData383_MEANSTDDEV, 48, gScenarioKernelData383_MEANSTDDEV, 4},
{12, 20, 488, 16, 0, gScenarioBufferData384_MEANSTDDEV, 48, gScenarioKernelData384_MEANSTDDEV, 4},
{14, 20, 568, 16, 0, gScenarioBufferData385_MEANSTDDEV, 48, gScenarioKernelData385_MEANSTDDEV, 4},
{16, 20, 648, 16, 0, gScenarioBufferData386_MEANSTDDEV, 48, gScenarioKernelData386_MEANSTDDEV, 4},
{18, 20, 728, 16, 0, gScenarioBufferData387_MEANSTDDEV, 48, gScenarioKernelData387_MEANSTDDEV, 4},
{20, 20, 808, 16, 0, gScenarioBufferData388_MEANSTDDEV, 48, gScenarioKernelData388_MEANSTDDEV, 4},
{22, 20, 888, 16, 0, gScenarioBufferData389_MEANSTDDEV, 48, gScenarioKernelData389_MEANSTDDEV, 4},
{24, 20, 968, 16, 0, gScenarioBufferData390_MEANSTDDEV, 48, gScenarioKernelData390_MEANSTDDEV, 4},
{26, 20, 1048, 16, 0, gScenarioBufferData391_MEANSTDDEV, 48, gScenarioKernelData391_MEANSTDDEV, 4},
{28, 20, 1128, 16, 0, gScenarioBufferData392_MEANSTDDEV, 48, gScenarioKernelData392_MEANSTDDEV, 4},
{30, 20, 1208, 16, 0, gScenarioBufferData393_MEANSTDDEV, 48, gScenarioKernelData393_MEANSTDDEV, 4},
{32, 20, 1288, 16, 0, gScenarioBufferData394_MEANSTDDEV, 48, gScenarioKernelData394_MEANSTDDEV, 4},
{36, 20, 1448, 16, 0, gScenarioBufferData395_MEANSTDDEV, 48, gScenarioKernelData395_MEANSTDDEV, 4},
{40, 20, 1608, 16, 0, gScenarioBufferData396_MEANSTDDEV, 48, gScenarioKernelData396_MEANSTDDEV, 4},
{44, 20, 1768, 16, 0, gScenarioBufferData397_MEANSTDDEV, 48, gScenarioKernelData397_MEANSTDDEV, 4},
{48, 20, 1928, 16, 0, gScenarioBufferData398_MEANSTDDEV, 48, gScenarioKernelData398_MEANSTDDEV, 4},
{52, 20, 2088, 16, 0, gScenarioBufferData399_MEANSTDDEV, 48, gScenarioKernelData399_MEANSTDDEV, 4},
{56, 20, 2248, 16, 0, gScenarioBufferData400_MEANSTDDEV, 48, gScenarioKernelData400_MEANSTDDEV, 4},
{60, 20, 2408, 16, 0, gScenarioBufferData401_MEANSTDDEV, 48, gScenarioKernelData401_MEANSTDDEV, 4},
{64, 20, 2568, 16, 0, gScenarioBufferData402_MEANSTDDEV, 48, gScenarioKernelData402_MEANSTDDEV, 4},
{72, 20, 2888, 16, 0, gScenarioBufferData403_MEANSTDDEV, 48, gScenarioKernelData403_MEANSTDDEV, 4},
{80, 20, 3208, 16, 0, gScenarioBufferData404_MEANSTDDEV, 48, gScenarioKernelData404_MEANSTDDEV, 4},
{88, 20, 3528, 16, 0, gScenarioBufferData405_MEANSTDDEV, 48, gScenarioKernelData405_MEANSTDDEV, 4},
{2, 22, 96, 16, 0, gScenarioBufferData406_MEANSTDDEV, 48, gScenarioKernelData406_MEANSTDDEV, 4},
{4, 22, 184, 16, 0, gScenarioBufferData407_MEANSTDDEV, 48, gScenarioKernelData407_MEANSTDDEV, 4},
{6, 22, 272, 16, 0, gScenarioBufferData408_MEANSTDDEV, 48, gScenarioKernelData408_MEANSTDDEV, 4},
{8, 22, 360, 16, 0, gScenarioBufferData409_MEANSTDDEV, 48, gScenarioKernelData409_MEANSTDDEV, 4},
{10, 22, 448, 16, 0, gScenarioBufferData410_MEANSTDDEV, 48, gScenarioKernelData410_MEANSTDDEV, 4},
{12, 22, 536, 16, 0, gScenarioBufferData411_MEANSTDDEV, 48, gScenarioKernelData411_MEANSTDDEV, 4},
{14, 22, 624, 16, 0, gScenarioBufferData412_MEANSTDDEV, 48, gScenarioKernelData412_MEANSTDDEV, 4},
{16, 22, 712, 16, 0, gScenarioBufferData413_MEANSTDDEV, 48, gScenarioKernelData413_MEANSTDDEV, 4},
{18, 22, 800, 16, 0, gScenarioBufferData414_MEANSTDDEV, 48, gScenarioKernelData414_MEANSTDDEV, 4},
{20, 22, 888, 16, 0, gScenarioBufferData415_MEANSTDDEV, 48, gScenarioKernelData415_MEANSTDDEV, 4},
{22, 22, 976, 16, 0, gScenarioBufferData416_MEANSTDDEV, 48, gScenarioKernelData416_MEANSTDDEV, 4},
{24, 22, 1064, 16, 0, gScenarioBufferData417_MEANSTDDEV, 48, gScenarioKernelData417_MEANSTDDEV, 4},
{26, 22, 1152, 16, 0, gScenarioBufferData418_MEANSTDDEV, 48, gScenarioKernelData418_MEANSTDDEV, 4},
{28, 22, 1240, 16, 0, gScenarioBufferData419_MEANSTDDEV, 48, gScenarioKernelData419_MEANSTDDEV, 4},
{30, 22, 1328, 16, 0, gScenarioBufferData420_MEANSTDDEV, 48, gScenarioKernelData420_MEANSTDDEV, 4},
{32, 22, 1416, 16, 0, gScenarioBufferData421_MEANSTDDEV, 48, gScenarioKernelData421_MEANSTDDEV, 4},
{36, 22, 1592, 16, 0, gScenarioBufferData422_MEANSTDDEV, 48, gScenarioKernelData422_MEANSTDDEV, 4},
{40, 22, 1768, 16, 0, gScenarioBufferData423_MEANSTDDEV, 48, gScenarioKernelData423_MEANSTDDEV, 4},
{44, 22, 1944, 16, 0, gScenarioBufferData424_MEANSTDDEV, 48, gScenarioKernelData424_MEANSTDDEV, 4},
{48, 22, 2120, 16, 0, gScenarioBufferData425_MEANSTDDEV, 48, gScenarioKernelData425_MEANSTDDEV, 4},
{52, 22, 2296, 16, 0, gScenarioBufferData426_MEANSTDDEV, 48, gScenarioKernelData426_MEANSTDDEV, 4},
{56, 22, 2472, 16, 0, gScenarioBufferData427_MEANSTDDEV, 48, gScenarioKernelData427_MEANSTDDEV, 4},
{60, 22, 2648, 16, 0, gScenarioBufferData428_MEANSTDDEV, 48, gScenarioKernelData428_MEANSTDDEV, 4},
{64, 22, 2824, 16, 0, gScenarioBufferData429_MEANSTDDEV, 48, gScenarioKernelData429_MEANSTDDEV, 4},
{72, 22, 3176, 16, 0, gScenarioBufferData430_MEANSTDDEV, 48, gScenarioKernelData430_MEANSTDDEV, 4},
{80, 22, 3528, 16, 0, gScenarioBufferData431_MEANSTDDEV, 48, gScenarioKernelData431_MEANSTDDEV, 4},
{2, 24, 104, 16, 0, gScenarioBufferData432_MEANSTDDEV, 48, gScenarioKernelData432_MEANSTDDEV, 4},
{4, 24, 200, 16, 0, gScenarioBufferData433_MEANSTDDEV, 48, gScenarioKernelData433_MEANSTDDEV, 4},
{6, 24, 296, 16, 0, gScenarioBufferData434_MEANSTDDEV, 48, gScenarioKernelData434_MEANSTDDEV, 4},
{8, 24, 392, 16, 0, gScenarioBufferData435_MEANSTDDEV, 48, gScenarioKernelData435_MEANSTDDEV, 4},
{10, 24, 488, 16, 0, gScenarioBufferData436_MEANSTDDEV, 48, gScenarioKernelData436_MEANSTDDEV, 4},
{12, 24, 584, 16, 0, gScenarioBufferData437_MEANSTDDEV, 48, gScenarioKernelData437_MEANSTDDEV, 4},
{14, 24, 680, 16, 0, gScenarioBufferData438_MEANSTDDEV, 48, gScenarioKernelData438_MEANSTDDEV, 4},
{16, 24, 776, 16, 0, gScenarioBufferData439_MEANSTDDEV, 48, gScenarioKernelData439_MEANSTDDEV, 4},
{18, 24, 872, 16, 0, gScenarioBufferData440_MEANSTDDEV, 48, gScenarioKernelData440_MEANSTDDEV, 4},
{20, 24, 968, 16, 0, gScenarioBufferData441_MEANSTDDEV, 48, gScenarioKernelData441_MEANSTDDEV, 4},
{22, 24, 1064, 16, 0, gScenarioBufferData442_MEANSTDDEV, 48, gScenarioKernelData442_MEANSTDDEV, 4},
{24, 24, 1160, 16, 0, gScenarioBufferData443_MEANSTDDEV, 48, gScenarioKernelData443_MEANSTDDEV, 4},
{26, 24, 1256, 16, 0, gScenarioBufferData444_MEANSTDDEV, 48, gScenarioKernelData444_MEANSTDDEV, 4},
{28, 24, 1352, 16, 0, gScenarioBufferData445_MEANSTDDEV, 48, gScenarioKernelData445_MEANSTDDEV, 4},
{30, 24, 1448, 16, 0, gScenarioBufferData446_MEANSTDDEV, 48, gScenarioKernelData446_MEANSTDDEV, 4},
{32, 24, 1544, 16, 0, gScenarioBufferData447_MEANSTDDEV, 48, gScenarioKernelData447_MEANSTDDEV, 4},
{36, 24, 1736, 16, 0, gScenarioBufferData448_MEANSTDDEV, 48, gScenarioKernelData448_MEANSTDDEV, 4},
{40, 24, 1928, 16, 0, gScenarioBufferData449_MEANSTDDEV, 48, gScenarioKernelData449_MEANSTDDEV, 4},
{44, 24, 2120, 16, 0, gScenarioBufferData450_MEANSTDDEV, 48, gScenarioKernelData450_MEANSTDDEV, 4},
{48, 24, 2312, 16, 0, gScenarioBufferData451_MEANSTDDEV, 48, gScenarioKernelData451_MEANSTDDEV, 4},
{52, 24, 2504, 16, 0, gScenarioBufferData452_MEANSTDDEV, 48, gScenarioKernelData452_MEANSTDDEV, 4},
{56, 24, 2696, 16, 0, gScenarioBufferData453_MEANSTDDEV, 48, gScenarioKernelData453_MEANSTDDEV, 4},
{60, 24, 2888, 16, 0, gScenarioBufferData454_MEANSTDDEV, 48, gScenarioKernelData454_MEANSTDDEV, 4},
{64, 24, 3080, 16, 0, gScenarioBufferData455_MEANSTDDEV, 48, gScenarioKernelData455_MEANSTDDEV, 4},
{72, 24, 3464, 16, 0, gScenarioBufferData456_MEANSTDDEV, 48, gScenarioKernelData456_MEANSTDDEV, 4},
{2, 26, 112, 16, 0, gScenarioBufferData457_MEANSTDDEV, 48, gScenarioKernelData457_MEANSTDDEV, 4},
{4, 26, 216, 16, 0, gScenarioBufferData458_MEANSTDDEV, 48, gScenarioKernelData458_MEANSTDDEV, 4},
{6, 26, 320, 16, 0, gScenarioBufferData459_MEANSTDDEV, 48, gScenarioKernelData459_MEANSTDDEV, 4},
{8, 26, 424, 16, 0, gScenarioBufferData460_MEANSTDDEV, 48, gScenarioKernelData460_MEANSTDDEV, 4},
{10, 26, 528, 16, 0, gScenarioBufferData461_MEANSTDDEV, 48, gScenarioKernelData461_MEANSTDDEV, 4},
{12, 26, 632, 16, 0, gScenarioBufferData462_MEANSTDDEV, 48, gScenarioKernelData462_MEANSTDDEV, 4},
{14, 26, 736, 16, 0, gScenarioBufferData463_MEANSTDDEV, 48, gScenarioKernelData463_MEANSTDDEV, 4},
{16, 26, 840, 16, 0, gScenarioBufferData464_MEANSTDDEV, 48, gScenarioKernelData464_MEANSTDDEV, 4},
{18, 26, 944, 16, 0, gScenarioBufferData465_MEANSTDDEV, 48, gScenarioKernelData465_MEANSTDDEV, 4},
{20, 26, 1048, 16, 0, gScenarioBufferData466_MEANSTDDEV, 48, gScenarioKernelData466_MEANSTDDEV, 4},
{22, 26, 1152, 16, 0, gScenarioBufferData467_MEANSTDDEV, 48, gScenarioKernelData467_MEANSTDDEV, 4},
{24, 26, 1256, 16, 0, gScenarioBufferData468_MEANSTDDEV, 48, gScenarioKernelData468_MEANSTDDEV, 4},
{26, 26, 1360, 16, 0, gScenarioBufferData469_MEANSTDDEV, 48, gScenarioKernelData469_MEANSTDDEV, 4},
{28, 26, 1464, 16, 0, gScenarioBufferData470_MEANSTDDEV, 48, gScenarioKernelData470_MEANSTDDEV, 4},
{30, 26, 1568, 16, 0, gScenarioBufferData471_MEANSTDDEV, 48, gScenarioKernelData471_MEANSTDDEV, 4},
{32, 26, 1672, 16, 0, gScenarioBufferData472_MEANSTDDEV, 48, gScenarioKernelData472_MEANSTDDEV, 4},
{36, 26, 1880, 16, 0, gScenarioBufferData473_MEANSTDDEV, 48, gScenarioKernelData473_MEANSTDDEV, 4},
{40, 26, 2088, 16, 0, gScenarioBufferData474_MEANSTDDEV, 48, gScenarioKernelData474_MEANSTDDEV, 4},
{44, 26, 2296, 16, 0, gScenarioBufferData475_MEANSTDDEV, 48, gScenarioKernelData475_MEANSTDDEV, 4},
{48, 26, 2504, 16, 0, gScenarioBufferData476_MEANSTDDEV, 48, gScenarioKernelData476_MEANSTDDEV, 4},
{52, 26, 2712, 16, 0, gScenarioBufferData477_MEANSTDDEV, 48, gScenarioKernelData477_MEANSTDDEV, 4},
{56, 26, 2920, 16, 0, gScenarioBufferData478_MEANSTDDEV, 48, gScenarioKernelData478_MEANSTDDEV, 4},
{60, 26, 3128, 16, 0, gScenarioBufferData479_MEANSTDDEV, 48, gScenarioKernelData479_MEANSTDDEV, 4},
{64, 26, 3336, 16, 0, gScenarioBufferData480_MEANSTDDEV, 48, gScenarioKernelData480_MEANSTDDEV, 4},
{72, 26, 3752, 16, 0, gScenarioBufferData481_MEANSTDDEV, 48, gScenarioKernelData481_MEANSTDDEV, 4},
{2, 28, 120, 16, 0, gScenarioBufferData482_MEANSTDDEV, 48, gScenarioKernelData482_MEANSTDDEV, 4},
{4, 28, 232, 16, 0, gScenarioBufferData483_MEANSTDDEV, 48, gScenarioKernelData483_MEANSTDDEV, 4},
{6, 28, 344, 16, 0, gScenarioBufferData484_MEANSTDDEV, 48, gScenarioKernelData484_MEANSTDDEV, 4},
{8, 28, 456, 16, 0, gScenarioBufferData485_MEANSTDDEV, 48, gScenarioKernelData485_MEANSTDDEV, 4},
{10, 28, 568, 16, 0, gScenarioBufferData486_MEANSTDDEV, 48, gScenarioKernelData486_MEANSTDDEV, 4},
{12, 28, 680, 16, 0, gScenarioBufferData487_MEANSTDDEV, 48, gScenarioKernelData487_MEANSTDDEV, 4},
{14, 28, 792, 16, 0, gScenarioBufferData488_MEANSTDDEV, 48, gScenarioKernelData488_MEANSTDDEV, 4},
{16, 28, 904, 16, 0, gScenarioBufferData489_MEANSTDDEV, 48, gScenarioKernelData489_MEANSTDDEV, 4},
{18, 28, 1016, 16, 0, gScenarioBufferData490_MEANSTDDEV, 48, gScenarioKernelData490_MEANSTDDEV, 4},
{20, 28, 1128, 16, 0, gScenarioBufferData491_MEANSTDDEV, 48, gScenarioKernelData491_MEANSTDDEV, 4},
{22, 28, 1240, 16, 0, gScenarioBufferData492_MEANSTDDEV, 48, gScenarioKernelData492_MEANSTDDEV, 4},
{24, 28, 1352, 16, 0, gScenarioBufferData493_MEANSTDDEV, 48, gScenarioKernelData493_MEANSTDDEV, 4},
{26, 28, 1464, 16, 0, gScenarioBufferData494_MEANSTDDEV, 48, gScenarioKernelData494_MEANSTDDEV, 4},
{28, 28, 1576, 16, 0, gScenarioBufferData495_MEANSTDDEV, 48, gScenarioKernelData495_MEANSTDDEV, 4},
{30, 28, 1688, 16, 0, gScenarioBufferData496_MEANSTDDEV, 48, gScenarioKernelData496_MEANSTDDEV, 4},
{32, 28, 1800, 16, 0, gScenarioBufferData497_MEANSTDDEV, 48, gScenarioKernelData497_MEANSTDDEV, 4},
{36, 28, 2024, 16, 0, gScenarioBufferData498_MEANSTDDEV, 48, gScenarioKernelData498_MEANSTDDEV, 4},
{40, 28, 2248, 16, 0, gScenarioBufferData499_MEANSTDDEV, 48, gScenarioKernelData499_MEANSTDDEV, 4},
{44, 28, 2472, 16, 0, gScenarioBufferData500_MEANSTDDEV, 48, gScenarioKernelData500_MEANSTDDEV, 4},
{48, 28, 2696, 16, 0, gScenarioBufferData501_MEANSTDDEV, 48, gScenarioKernelData501_MEANSTDDEV, 4},
{52, 28, 2920, 16, 0, gScenarioBufferData502_MEANSTDDEV, 48, gScenarioKernelData502_MEANSTDDEV, 4},
{56, 28, 3144, 16, 0, gScenarioBufferData503_MEANSTDDEV, 48, gScenarioKernelData503_MEANSTDDEV, 4},
{60, 28, 3368, 16, 0, gScenarioBufferData504_MEANSTDDEV, 48, gScenarioKernelData504_MEANSTDDEV, 4},
{64, 28, 3592, 16, 0, gScenarioBufferData505_MEANSTDDEV, 48, gScenarioKernelData505_MEANSTDDEV, 4},
{2, 30, 128, 16, 0, gScenarioBufferData506_MEANSTDDEV, 48, gScenarioKernelData506_MEANSTDDEV, 4},
{4, 30, 248, 16, 0, gScenarioBufferData507_MEANSTDDEV, 48, gScenarioKernelData507_MEANSTDDEV, 4},
{6, 30, 368, 16, 0, gScenarioBufferData508_MEANSTDDEV, 48, gScenarioKernelData508_MEANSTDDEV, 4},
{8, 30, 488, 16, 0, gScenarioBufferData509_MEANSTDDEV, 48, gScenarioKernelData509_MEANSTDDEV, 4},
{10, 30, 608, 16, 0, gScenarioBufferData510_MEANSTDDEV, 48, gScenarioKernelData510_MEANSTDDEV, 4},
{12, 30, 728, 16, 0, gScenarioBufferData511_MEANSTDDEV, 48, gScenarioKernelData511_MEANSTDDEV, 4},
{14, 30, 848, 16, 0, gScenarioBufferData512_MEANSTDDEV, 48, gScenarioKernelData512_MEANSTDDEV, 4},
{16, 30, 968, 16, 0, gScenarioBufferData513_MEANSTDDEV, 48, gScenarioKernelData513_MEANSTDDEV, 4},
{18, 30, 1088, 16, 0, gScenarioBufferData514_MEANSTDDEV, 48, gScenarioKernelData514_MEANSTDDEV, 4},
{20, 30, 1208, 16, 0, gScenarioBufferData515_MEANSTDDEV, 48, gScenarioKernelData515_MEANSTDDEV, 4},
{22, 30, 1328, 16, 0, gScenarioBufferData516_MEANSTDDEV, 48, gScenarioKernelData516_MEANSTDDEV, 4},
{24, 30, 1448, 16, 0, gScenarioBufferData517_MEANSTDDEV, 48, gScenarioKernelData517_MEANSTDDEV, 4},
{26, 30, 1568, 16, 0, gScenarioBufferData518_MEANSTDDEV, 48, gScenarioKernelData518_MEANSTDDEV, 4},
{28, 30, 1688, 16, 0, gScenarioBufferData519_MEANSTDDEV, 48, gScenarioKernelData519_MEANSTDDEV, 4},
{30, 30, 1808, 16, 0, gScenarioBufferData520_MEANSTDDEV, 48, gScenarioKernelData520_MEANSTDDEV, 4},
{32, 30, 1928, 16, 0, gScenarioBufferData521_MEANSTDDEV, 48, gScenarioKernelData521_MEANSTDDEV, 4},
{36, 30, 2168, 16, 0, gScenarioBufferData522_MEANSTDDEV, 48, gScenarioKernelData522_MEANSTDDEV, 4},
{40, 30, 2408, 16, 0, gScenarioBufferData523_MEANSTDDEV, 48, gScenarioKernelData523_MEANSTDDEV, 4},
{44, 30, 2648, 16, 0, gScenarioBufferData524_MEANSTDDEV, 48, gScenarioKernelData524_MEANSTDDEV, 4},
{48, 30, 2888, 16, 0, gScenarioBufferData525_MEANSTDDEV, 48, gScenarioKernelData525_MEANSTDDEV, 4},
{52, 30, 3128, 16, 0, gScenarioBufferData526_MEANSTDDEV, 48, gScenarioKernelData526_MEANSTDDEV, 4},
{56, 30, 3368, 16, 0, gScenarioBufferData527_MEANSTDDEV, 48, gScenarioKernelData527_MEANSTDDEV, 4},
{60, 30, 3608, 16, 0, gScenarioBufferData528_MEANSTDDEV, 48, gScenarioKernelData528_MEANSTDDEV, 4},
{2, 32, 136, 16, 0, gScenarioBufferData529_MEANSTDDEV, 48, gScenarioKernelData529_MEANSTDDEV, 4},
{4, 32, 264, 16, 0, gScenarioBufferData530_MEANSTDDEV, 48, gScenarioKernelData530_MEANSTDDEV, 4},
{6, 32, 392, 16, 0, gScenarioBufferData531_MEANSTDDEV, 48, gScenarioKernelData531_MEANSTDDEV, 4},
{8, 32, 520, 16, 0, gScenarioBufferData532_MEANSTDDEV, 48, gScenarioKernelData532_MEANSTDDEV, 4},
{10, 32, 648, 16, 0, gScenarioBufferData533_MEANSTDDEV, 48, gScenarioKernelData533_MEANSTDDEV, 4},
{12, 32, 776, 16, 0, gScenarioBufferData534_MEANSTDDEV, 48, gScenarioKernelData534_MEANSTDDEV, 4},
{14, 32, 904, 16, 0, gScenarioBufferData535_MEANSTDDEV, 48, gScenarioKernelData535_MEANSTDDEV, 4},
{16, 32, 1032, 16, 0, gScenarioBufferData536_MEANSTDDEV, 48, gScenarioKernelData536_MEANSTDDEV, 4},
{18, 32, 1160, 16, 0, gScenarioBufferData537_MEANSTDDEV, 48, gScenarioKernelData537_MEANSTDDEV, 4},
{20, 32, 1288, 16, 0, gScenarioBufferData538_MEANSTDDEV, 48, gScenarioKernelData538_MEANSTDDEV, 4},
{22, 32, 1416, 16, 0, gScenarioBufferData539_MEANSTDDEV, 48, gScenarioKernelData539_MEANSTDDEV, 4},
{24, 32, 1544, 16, 0, gScenarioBufferData540_MEANSTDDEV, 48, gScenarioKernelData540_MEANSTDDEV, 4},
{26, 32, 1672, 16, 0, gScenarioBufferData541_MEANSTDDEV, 48, gScenarioKernelData541_MEANSTDDEV, 4},
{28, 32, 1800, 16, 0, gScenarioBufferData542_MEANSTDDEV, 48, gScenarioKernelData542_MEANSTDDEV, 4},
{30, 32, 1928, 16, 0, gScenarioBufferData543_MEANSTDDEV, 48, gScenarioKernelData543_MEANSTDDEV, 4},
{32, 32, 2056, 16, 0, gScenarioBufferData544_MEANSTDDEV, 48, gScenarioKernelData544_MEANSTDDEV, 4},
{36, 32, 2312, 16, 0, gScenarioBufferData545_MEANSTDDEV, 48, gScenarioKernelData545_MEANSTDDEV, 4},
{40, 32, 2568, 16, 0, gScenarioBufferData546_MEANSTDDEV, 48, gScenarioKernelData546_MEANSTDDEV, 4},
{44, 32, 2824, 16, 0, gScenarioBufferData547_MEANSTDDEV, 48, gScenarioKernelData547_MEANSTDDEV, 4},
{48, 32, 3080, 16, 0, gScenarioBufferData548_MEANSTDDEV, 48, gScenarioKernelData548_MEANSTDDEV, 4},
{52, 32, 3336, 16, 0, gScenarioBufferData549_MEANSTDDEV, 48, gScenarioKernelData549_MEANSTDDEV, 4},
{56, 32, 3592, 16, 0, gScenarioBufferData550_MEANSTDDEV, 48, gScenarioKernelData550_MEANSTDDEV, 4}
};

static acf_scenario_list gScenarioList_MEANSTDDEV = {
551, //number of scenarios
gScenarioArray_MEANSTDDEV};
//**************************************************************

class MEANSTDDEV : public ACF_Process_APU
{

public:
   MEANSTDDEV(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MEANSTDDEV");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MEANSTDDEV",
                                        MEANSTDDEV_LOAD_SEGMENTS,
                                        MEANSTDDEV_LOAD_PMEM, MEANSTDDEV_LOAD_PMEM_SIZE,
                                        MEANSTDDEV_LOAD_DMEM, MEANSTDDEV_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MEANSTDDEV_APEX_LOG_BUFFER, MEANSTDDEV_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_32U, 1, 1, 1, 2, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MEANSTDDEV);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MEANSTDDEV

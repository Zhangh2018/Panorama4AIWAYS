#ifndef _ACF_PROCESS_APU_BGR888_TO_GREY
#define _ACF_PROCESS_APU_BGR888_TO_GREY

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <BGR888_TO_GREY_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_BGR888_TO_GREY[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_BGR888_TO_GREY[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_BGR888_TO_GREY[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_BGR888_TO_GREY[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_BGR888_TO_GREY[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_BGR888_TO_GREY[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_BGR888_TO_GREY[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_BGR888_TO_GREY[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_BGR888_TO_GREY[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_BGR888_TO_GREY[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_BGR888_TO_GREY[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_BGR888_TO_GREY[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_BGR888_TO_GREY[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_BGR888_TO_GREY[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_BGR888_TO_GREY[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_BGR888_TO_GREY[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_BGR888_TO_GREY[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_BGR888_TO_GREY[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_BGR888_TO_GREY[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_BGR888_TO_GREY[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_BGR888_TO_GREY[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_BGR888_TO_GREY[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_BGR888_TO_GREY[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_BGR888_TO_GREY[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_BGR888_TO_GREY[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_BGR888_TO_GREY[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_BGR888_TO_GREY[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_BGR888_TO_GREY[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_BGR888_TO_GREY[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_BGR888_TO_GREY[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_BGR888_TO_GREY[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_BGR888_TO_GREY[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_BGR888_TO_GREY[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_BGR888_TO_GREY[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_BGR888_TO_GREY[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_BGR888_TO_GREY[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_BGR888_TO_GREY[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_BGR888_TO_GREY[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_BGR888_TO_GREY[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_BGR888_TO_GREY[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_BGR888_TO_GREY[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_BGR888_TO_GREY[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_BGR888_TO_GREY[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_BGR888_TO_GREY[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_BGR888_TO_GREY[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_BGR888_TO_GREY[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_BGR888_TO_GREY[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_BGR888_TO_GREY[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_BGR888_TO_GREY[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_BGR888_TO_GREY[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_BGR888_TO_GREY[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_BGR888_TO_GREY[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_BGR888_TO_GREY[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_BGR888_TO_GREY[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_BGR888_TO_GREY[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_BGR888_TO_GREY[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_BGR888_TO_GREY[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_BGR888_TO_GREY[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_BGR888_TO_GREY[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_BGR888_TO_GREY[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_BGR888_TO_GREY[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_BGR888_TO_GREY[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_BGR888_TO_GREY[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_BGR888_TO_GREY[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_BGR888_TO_GREY[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_BGR888_TO_GREY[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_BGR888_TO_GREY[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_BGR888_TO_GREY[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_BGR888_TO_GREY[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_BGR888_TO_GREY[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_BGR888_TO_GREY[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_BGR888_TO_GREY[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_BGR888_TO_GREY[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_BGR888_TO_GREY[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_BGR888_TO_GREY[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_BGR888_TO_GREY[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_BGR888_TO_GREY[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_BGR888_TO_GREY[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_BGR888_TO_GREY[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_BGR888_TO_GREY[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_BGR888_TO_GREY[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_BGR888_TO_GREY[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_BGR888_TO_GREY[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_BGR888_TO_GREY[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_BGR888_TO_GREY[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_BGR888_TO_GREY[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_BGR888_TO_GREY[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_BGR888_TO_GREY[] = {{40, 10, 2, 0, {0, 0, 0, 0}}, {40, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_BGR888_TO_GREY[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_BGR888_TO_GREY[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_BGR888_TO_GREY[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_BGR888_TO_GREY[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_BGR888_TO_GREY[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_BGR888_TO_GREY[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_BGR888_TO_GREY[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_BGR888_TO_GREY[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_BGR888_TO_GREY[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_BGR888_TO_GREY[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_BGR888_TO_GREY[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {40, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_BGR888_TO_GREY[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_BGR888_TO_GREY[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_BGR888_TO_GREY[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_BGR888_TO_GREY[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_BGR888_TO_GREY[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_BGR888_TO_GREY[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_BGR888_TO_GREY[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_BGR888_TO_GREY[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_BGR888_TO_GREY[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_BGR888_TO_GREY[] = {{32, 14, 2, 0, {0, 0, 0, 0}}, {32, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_BGR888_TO_GREY[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_BGR888_TO_GREY[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_BGR888_TO_GREY[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_BGR888_TO_GREY[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_BGR888_TO_GREY[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_BGR888_TO_GREY[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_BGR888_TO_GREY[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_BGR888_TO_GREY[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_BGR888_TO_GREY[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_BGR888_TO_GREY[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_BGR888_TO_GREY[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_BGR888_TO_GREY[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_BGR888_TO_GREY[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_BGR888_TO_GREY[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_BGR888_TO_GREY[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_BGR888_TO_GREY[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_BGR888_TO_GREY[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_BGR888_TO_GREY[] = {{24, 18, 2, 0, {0, 0, 0, 0}}, {24, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_BGR888_TO_GREY[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_BGR888_TO_GREY[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_BGR888_TO_GREY[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_BGR888_TO_GREY[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_BGR888_TO_GREY[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_BGR888_TO_GREY[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_BGR888_TO_GREY[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_BGR888_TO_GREY[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {20, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_BGR888_TO_GREY[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {24, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_BGR888_TO_GREY[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_BGR888_TO_GREY[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_BGR888_TO_GREY[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_BGR888_TO_GREY[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_BGR888_TO_GREY[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_BGR888_TO_GREY[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_BGR888_TO_GREY[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_BGR888_TO_GREY[] = {{20, 22, 2, 0, {0, 0, 0, 0}}, {20, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_BGR888_TO_GREY[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_BGR888_TO_GREY[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_BGR888_TO_GREY[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_BGR888_TO_GREY[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_BGR888_TO_GREY[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_BGR888_TO_GREY[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_BGR888_TO_GREY[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_BGR888_TO_GREY[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {20, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_BGR888_TO_GREY[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_BGR888_TO_GREY[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_BGR888_TO_GREY[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_BGR888_TO_GREY[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_BGR888_TO_GREY[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_BGR888_TO_GREY[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_BGR888_TO_GREY[] = {{16, 26, 2, 0, {0, 0, 0, 0}}, {16, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_BGR888_TO_GREY[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_BGR888_TO_GREY[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_BGR888_TO_GREY[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_BGR888_TO_GREY[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_BGR888_TO_GREY[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_BGR888_TO_GREY[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_BGR888_TO_GREY[] = {{16, 28, 2, 0, {0, 0, 0, 0}}, {16, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_BGR888_TO_GREY[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_BGR888_TO_GREY[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_BGR888_TO_GREY[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_BGR888_TO_GREY[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_BGR888_TO_GREY[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_BGR888_TO_GREY[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_BGR888_TO_GREY[] = {{16, 30, 2, 0, {0, 0, 0, 0}}, {16, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_BGR888_TO_GREY[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_BGR888_TO_GREY[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_BGR888_TO_GREY[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_BGR888_TO_GREY[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_BGR888_TO_GREY[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_BGR888_TO_GREY[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_BGR888_TO_GREY[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_BGR888_TO_GREY[] = {{0, 0}};

static acf_scenario gScenarioArray_BGR888_TO_GREY[] = {
{2, 1, 16, 0, 0, gScenarioBufferData0_BGR888_TO_GREY, 32, gScenarioKernelData0_BGR888_TO_GREY, 4},
{4, 1, 32, 0, 0, gScenarioBufferData1_BGR888_TO_GREY, 32, gScenarioKernelData1_BGR888_TO_GREY, 4},
{6, 1, 48, 0, 0, gScenarioBufferData2_BGR888_TO_GREY, 32, gScenarioKernelData2_BGR888_TO_GREY, 4},
{8, 1, 64, 0, 0, gScenarioBufferData3_BGR888_TO_GREY, 32, gScenarioKernelData3_BGR888_TO_GREY, 4},
{10, 1, 80, 0, 0, gScenarioBufferData4_BGR888_TO_GREY, 32, gScenarioKernelData4_BGR888_TO_GREY, 4},
{12, 1, 96, 0, 0, gScenarioBufferData5_BGR888_TO_GREY, 32, gScenarioKernelData5_BGR888_TO_GREY, 4},
{16, 1, 128, 0, 0, gScenarioBufferData6_BGR888_TO_GREY, 32, gScenarioKernelData6_BGR888_TO_GREY, 4},
{20, 1, 160, 0, 0, gScenarioBufferData7_BGR888_TO_GREY, 32, gScenarioKernelData7_BGR888_TO_GREY, 4},
{24, 1, 192, 0, 0, gScenarioBufferData8_BGR888_TO_GREY, 32, gScenarioKernelData8_BGR888_TO_GREY, 4},
{32, 1, 256, 0, 0, gScenarioBufferData9_BGR888_TO_GREY, 32, gScenarioKernelData9_BGR888_TO_GREY, 4},
{40, 1, 320, 0, 0, gScenarioBufferData10_BGR888_TO_GREY, 32, gScenarioKernelData10_BGR888_TO_GREY, 4},
{2, 2, 32, 0, 0, gScenarioBufferData11_BGR888_TO_GREY, 32, gScenarioKernelData11_BGR888_TO_GREY, 4},
{4, 2, 64, 0, 0, gScenarioBufferData12_BGR888_TO_GREY, 32, gScenarioKernelData12_BGR888_TO_GREY, 4},
{6, 2, 96, 0, 0, gScenarioBufferData13_BGR888_TO_GREY, 32, gScenarioKernelData13_BGR888_TO_GREY, 4},
{8, 2, 128, 0, 0, gScenarioBufferData14_BGR888_TO_GREY, 32, gScenarioKernelData14_BGR888_TO_GREY, 4},
{10, 2, 160, 0, 0, gScenarioBufferData15_BGR888_TO_GREY, 32, gScenarioKernelData15_BGR888_TO_GREY, 4},
{12, 2, 192, 0, 0, gScenarioBufferData16_BGR888_TO_GREY, 32, gScenarioKernelData16_BGR888_TO_GREY, 4},
{16, 2, 256, 0, 0, gScenarioBufferData17_BGR888_TO_GREY, 32, gScenarioKernelData17_BGR888_TO_GREY, 4},
{20, 2, 320, 0, 0, gScenarioBufferData18_BGR888_TO_GREY, 32, gScenarioKernelData18_BGR888_TO_GREY, 4},
{24, 2, 384, 0, 0, gScenarioBufferData19_BGR888_TO_GREY, 32, gScenarioKernelData19_BGR888_TO_GREY, 4},
{32, 2, 512, 0, 0, gScenarioBufferData20_BGR888_TO_GREY, 32, gScenarioKernelData20_BGR888_TO_GREY, 4},
{40, 2, 640, 0, 0, gScenarioBufferData21_BGR888_TO_GREY, 32, gScenarioKernelData21_BGR888_TO_GREY, 4},
{2, 3, 48, 0, 0, gScenarioBufferData22_BGR888_TO_GREY, 32, gScenarioKernelData22_BGR888_TO_GREY, 4},
{4, 3, 96, 0, 0, gScenarioBufferData23_BGR888_TO_GREY, 32, gScenarioKernelData23_BGR888_TO_GREY, 4},
{6, 3, 144, 0, 0, gScenarioBufferData24_BGR888_TO_GREY, 32, gScenarioKernelData24_BGR888_TO_GREY, 4},
{8, 3, 192, 0, 0, gScenarioBufferData25_BGR888_TO_GREY, 32, gScenarioKernelData25_BGR888_TO_GREY, 4},
{10, 3, 240, 0, 0, gScenarioBufferData26_BGR888_TO_GREY, 32, gScenarioKernelData26_BGR888_TO_GREY, 4},
{12, 3, 288, 0, 0, gScenarioBufferData27_BGR888_TO_GREY, 32, gScenarioKernelData27_BGR888_TO_GREY, 4},
{16, 3, 384, 0, 0, gScenarioBufferData28_BGR888_TO_GREY, 32, gScenarioKernelData28_BGR888_TO_GREY, 4},
{20, 3, 480, 0, 0, gScenarioBufferData29_BGR888_TO_GREY, 32, gScenarioKernelData29_BGR888_TO_GREY, 4},
{24, 3, 576, 0, 0, gScenarioBufferData30_BGR888_TO_GREY, 32, gScenarioKernelData30_BGR888_TO_GREY, 4},
{32, 3, 768, 0, 0, gScenarioBufferData31_BGR888_TO_GREY, 32, gScenarioKernelData31_BGR888_TO_GREY, 4},
{40, 3, 960, 0, 0, gScenarioBufferData32_BGR888_TO_GREY, 32, gScenarioKernelData32_BGR888_TO_GREY, 4},
{2, 4, 64, 0, 0, gScenarioBufferData33_BGR888_TO_GREY, 32, gScenarioKernelData33_BGR888_TO_GREY, 4},
{4, 4, 128, 0, 0, gScenarioBufferData34_BGR888_TO_GREY, 32, gScenarioKernelData34_BGR888_TO_GREY, 4},
{6, 4, 192, 0, 0, gScenarioBufferData35_BGR888_TO_GREY, 32, gScenarioKernelData35_BGR888_TO_GREY, 4},
{8, 4, 256, 0, 0, gScenarioBufferData36_BGR888_TO_GREY, 32, gScenarioKernelData36_BGR888_TO_GREY, 4},
{10, 4, 320, 0, 0, gScenarioBufferData37_BGR888_TO_GREY, 32, gScenarioKernelData37_BGR888_TO_GREY, 4},
{12, 4, 384, 0, 0, gScenarioBufferData38_BGR888_TO_GREY, 32, gScenarioKernelData38_BGR888_TO_GREY, 4},
{16, 4, 512, 0, 0, gScenarioBufferData39_BGR888_TO_GREY, 32, gScenarioKernelData39_BGR888_TO_GREY, 4},
{20, 4, 640, 0, 0, gScenarioBufferData40_BGR888_TO_GREY, 32, gScenarioKernelData40_BGR888_TO_GREY, 4},
{24, 4, 768, 0, 0, gScenarioBufferData41_BGR888_TO_GREY, 32, gScenarioKernelData41_BGR888_TO_GREY, 4},
{32, 4, 1024, 0, 0, gScenarioBufferData42_BGR888_TO_GREY, 32, gScenarioKernelData42_BGR888_TO_GREY, 4},
{40, 4, 1280, 0, 0, gScenarioBufferData43_BGR888_TO_GREY, 32, gScenarioKernelData43_BGR888_TO_GREY, 4},
{2, 5, 80, 0, 0, gScenarioBufferData44_BGR888_TO_GREY, 32, gScenarioKernelData44_BGR888_TO_GREY, 4},
{4, 5, 160, 0, 0, gScenarioBufferData45_BGR888_TO_GREY, 32, gScenarioKernelData45_BGR888_TO_GREY, 4},
{6, 5, 240, 0, 0, gScenarioBufferData46_BGR888_TO_GREY, 32, gScenarioKernelData46_BGR888_TO_GREY, 4},
{8, 5, 320, 0, 0, gScenarioBufferData47_BGR888_TO_GREY, 32, gScenarioKernelData47_BGR888_TO_GREY, 4},
{10, 5, 400, 0, 0, gScenarioBufferData48_BGR888_TO_GREY, 32, gScenarioKernelData48_BGR888_TO_GREY, 4},
{12, 5, 480, 0, 0, gScenarioBufferData49_BGR888_TO_GREY, 32, gScenarioKernelData49_BGR888_TO_GREY, 4},
{16, 5, 640, 0, 0, gScenarioBufferData50_BGR888_TO_GREY, 32, gScenarioKernelData50_BGR888_TO_GREY, 4},
{20, 5, 800, 0, 0, gScenarioBufferData51_BGR888_TO_GREY, 32, gScenarioKernelData51_BGR888_TO_GREY, 4},
{24, 5, 960, 0, 0, gScenarioBufferData52_BGR888_TO_GREY, 32, gScenarioKernelData52_BGR888_TO_GREY, 4},
{32, 5, 1280, 0, 0, gScenarioBufferData53_BGR888_TO_GREY, 32, gScenarioKernelData53_BGR888_TO_GREY, 4},
{40, 5, 1600, 0, 0, gScenarioBufferData54_BGR888_TO_GREY, 32, gScenarioKernelData54_BGR888_TO_GREY, 4},
{2, 6, 96, 0, 0, gScenarioBufferData55_BGR888_TO_GREY, 32, gScenarioKernelData55_BGR888_TO_GREY, 4},
{4, 6, 192, 0, 0, gScenarioBufferData56_BGR888_TO_GREY, 32, gScenarioKernelData56_BGR888_TO_GREY, 4},
{6, 6, 288, 0, 0, gScenarioBufferData57_BGR888_TO_GREY, 32, gScenarioKernelData57_BGR888_TO_GREY, 4},
{8, 6, 384, 0, 0, gScenarioBufferData58_BGR888_TO_GREY, 32, gScenarioKernelData58_BGR888_TO_GREY, 4},
{10, 6, 480, 0, 0, gScenarioBufferData59_BGR888_TO_GREY, 32, gScenarioKernelData59_BGR888_TO_GREY, 4},
{12, 6, 576, 0, 0, gScenarioBufferData60_BGR888_TO_GREY, 32, gScenarioKernelData60_BGR888_TO_GREY, 4},
{16, 6, 768, 0, 0, gScenarioBufferData61_BGR888_TO_GREY, 32, gScenarioKernelData61_BGR888_TO_GREY, 4},
{20, 6, 960, 0, 0, gScenarioBufferData62_BGR888_TO_GREY, 32, gScenarioKernelData62_BGR888_TO_GREY, 4},
{24, 6, 1152, 0, 0, gScenarioBufferData63_BGR888_TO_GREY, 32, gScenarioKernelData63_BGR888_TO_GREY, 4},
{32, 6, 1536, 0, 0, gScenarioBufferData64_BGR888_TO_GREY, 32, gScenarioKernelData64_BGR888_TO_GREY, 4},
{40, 6, 1920, 0, 0, gScenarioBufferData65_BGR888_TO_GREY, 32, gScenarioKernelData65_BGR888_TO_GREY, 4},
{2, 8, 128, 0, 0, gScenarioBufferData66_BGR888_TO_GREY, 32, gScenarioKernelData66_BGR888_TO_GREY, 4},
{4, 8, 256, 0, 0, gScenarioBufferData67_BGR888_TO_GREY, 32, gScenarioKernelData67_BGR888_TO_GREY, 4},
{6, 8, 384, 0, 0, gScenarioBufferData68_BGR888_TO_GREY, 32, gScenarioKernelData68_BGR888_TO_GREY, 4},
{8, 8, 512, 0, 0, gScenarioBufferData69_BGR888_TO_GREY, 32, gScenarioKernelData69_BGR888_TO_GREY, 4},
{10, 8, 640, 0, 0, gScenarioBufferData70_BGR888_TO_GREY, 32, gScenarioKernelData70_BGR888_TO_GREY, 4},
{12, 8, 768, 0, 0, gScenarioBufferData71_BGR888_TO_GREY, 32, gScenarioKernelData71_BGR888_TO_GREY, 4},
{16, 8, 1024, 0, 0, gScenarioBufferData72_BGR888_TO_GREY, 32, gScenarioKernelData72_BGR888_TO_GREY, 4},
{20, 8, 1280, 0, 0, gScenarioBufferData73_BGR888_TO_GREY, 32, gScenarioKernelData73_BGR888_TO_GREY, 4},
{24, 8, 1536, 0, 0, gScenarioBufferData74_BGR888_TO_GREY, 32, gScenarioKernelData74_BGR888_TO_GREY, 4},
{32, 8, 2048, 0, 0, gScenarioBufferData75_BGR888_TO_GREY, 32, gScenarioKernelData75_BGR888_TO_GREY, 4},
{40, 8, 2560, 0, 0, gScenarioBufferData76_BGR888_TO_GREY, 32, gScenarioKernelData76_BGR888_TO_GREY, 4},
{2, 10, 160, 0, 0, gScenarioBufferData77_BGR888_TO_GREY, 32, gScenarioKernelData77_BGR888_TO_GREY, 4},
{4, 10, 320, 0, 0, gScenarioBufferData78_BGR888_TO_GREY, 32, gScenarioKernelData78_BGR888_TO_GREY, 4},
{6, 10, 480, 0, 0, gScenarioBufferData79_BGR888_TO_GREY, 32, gScenarioKernelData79_BGR888_TO_GREY, 4},
{8, 10, 640, 0, 0, gScenarioBufferData80_BGR888_TO_GREY, 32, gScenarioKernelData80_BGR888_TO_GREY, 4},
{10, 10, 800, 0, 0, gScenarioBufferData81_BGR888_TO_GREY, 32, gScenarioKernelData81_BGR888_TO_GREY, 4},
{12, 10, 960, 0, 0, gScenarioBufferData82_BGR888_TO_GREY, 32, gScenarioKernelData82_BGR888_TO_GREY, 4},
{16, 10, 1280, 0, 0, gScenarioBufferData83_BGR888_TO_GREY, 32, gScenarioKernelData83_BGR888_TO_GREY, 4},
{20, 10, 1600, 0, 0, gScenarioBufferData84_BGR888_TO_GREY, 32, gScenarioKernelData84_BGR888_TO_GREY, 4},
{24, 10, 1920, 0, 0, gScenarioBufferData85_BGR888_TO_GREY, 32, gScenarioKernelData85_BGR888_TO_GREY, 4},
{32, 10, 2560, 0, 0, gScenarioBufferData86_BGR888_TO_GREY, 32, gScenarioKernelData86_BGR888_TO_GREY, 4},
{40, 10, 3200, 0, 0, gScenarioBufferData87_BGR888_TO_GREY, 32, gScenarioKernelData87_BGR888_TO_GREY, 4},
{2, 12, 192, 0, 0, gScenarioBufferData88_BGR888_TO_GREY, 32, gScenarioKernelData88_BGR888_TO_GREY, 4},
{4, 12, 384, 0, 0, gScenarioBufferData89_BGR888_TO_GREY, 32, gScenarioKernelData89_BGR888_TO_GREY, 4},
{6, 12, 576, 0, 0, gScenarioBufferData90_BGR888_TO_GREY, 32, gScenarioKernelData90_BGR888_TO_GREY, 4},
{8, 12, 768, 0, 0, gScenarioBufferData91_BGR888_TO_GREY, 32, gScenarioKernelData91_BGR888_TO_GREY, 4},
{10, 12, 960, 0, 0, gScenarioBufferData92_BGR888_TO_GREY, 32, gScenarioKernelData92_BGR888_TO_GREY, 4},
{12, 12, 1152, 0, 0, gScenarioBufferData93_BGR888_TO_GREY, 32, gScenarioKernelData93_BGR888_TO_GREY, 4},
{16, 12, 1536, 0, 0, gScenarioBufferData94_BGR888_TO_GREY, 32, gScenarioKernelData94_BGR888_TO_GREY, 4},
{20, 12, 1920, 0, 0, gScenarioBufferData95_BGR888_TO_GREY, 32, gScenarioKernelData95_BGR888_TO_GREY, 4},
{24, 12, 2304, 0, 0, gScenarioBufferData96_BGR888_TO_GREY, 32, gScenarioKernelData96_BGR888_TO_GREY, 4},
{32, 12, 3072, 0, 0, gScenarioBufferData97_BGR888_TO_GREY, 32, gScenarioKernelData97_BGR888_TO_GREY, 4},
{40, 12, 3840, 0, 0, gScenarioBufferData98_BGR888_TO_GREY, 32, gScenarioKernelData98_BGR888_TO_GREY, 4},
{2, 14, 224, 0, 0, gScenarioBufferData99_BGR888_TO_GREY, 32, gScenarioKernelData99_BGR888_TO_GREY, 4},
{4, 14, 448, 0, 0, gScenarioBufferData100_BGR888_TO_GREY, 32, gScenarioKernelData100_BGR888_TO_GREY, 4},
{6, 14, 672, 0, 0, gScenarioBufferData101_BGR888_TO_GREY, 32, gScenarioKernelData101_BGR888_TO_GREY, 4},
{8, 14, 896, 0, 0, gScenarioBufferData102_BGR888_TO_GREY, 32, gScenarioKernelData102_BGR888_TO_GREY, 4},
{10, 14, 1120, 0, 0, gScenarioBufferData103_BGR888_TO_GREY, 32, gScenarioKernelData103_BGR888_TO_GREY, 4},
{12, 14, 1344, 0, 0, gScenarioBufferData104_BGR888_TO_GREY, 32, gScenarioKernelData104_BGR888_TO_GREY, 4},
{16, 14, 1792, 0, 0, gScenarioBufferData105_BGR888_TO_GREY, 32, gScenarioKernelData105_BGR888_TO_GREY, 4},
{20, 14, 2240, 0, 0, gScenarioBufferData106_BGR888_TO_GREY, 32, gScenarioKernelData106_BGR888_TO_GREY, 4},
{24, 14, 2688, 0, 0, gScenarioBufferData107_BGR888_TO_GREY, 32, gScenarioKernelData107_BGR888_TO_GREY, 4},
{32, 14, 3584, 0, 0, gScenarioBufferData108_BGR888_TO_GREY, 32, gScenarioKernelData108_BGR888_TO_GREY, 4},
{2, 16, 256, 0, 0, gScenarioBufferData109_BGR888_TO_GREY, 32, gScenarioKernelData109_BGR888_TO_GREY, 4},
{4, 16, 512, 0, 0, gScenarioBufferData110_BGR888_TO_GREY, 32, gScenarioKernelData110_BGR888_TO_GREY, 4},
{6, 16, 768, 0, 0, gScenarioBufferData111_BGR888_TO_GREY, 32, gScenarioKernelData111_BGR888_TO_GREY, 4},
{8, 16, 1024, 0, 0, gScenarioBufferData112_BGR888_TO_GREY, 32, gScenarioKernelData112_BGR888_TO_GREY, 4},
{10, 16, 1280, 0, 0, gScenarioBufferData113_BGR888_TO_GREY, 32, gScenarioKernelData113_BGR888_TO_GREY, 4},
{12, 16, 1536, 0, 0, gScenarioBufferData114_BGR888_TO_GREY, 32, gScenarioKernelData114_BGR888_TO_GREY, 4},
{16, 16, 2048, 0, 0, gScenarioBufferData115_BGR888_TO_GREY, 32, gScenarioKernelData115_BGR888_TO_GREY, 4},
{20, 16, 2560, 0, 0, gScenarioBufferData116_BGR888_TO_GREY, 32, gScenarioKernelData116_BGR888_TO_GREY, 4},
{24, 16, 3072, 0, 0, gScenarioBufferData117_BGR888_TO_GREY, 32, gScenarioKernelData117_BGR888_TO_GREY, 4},
{2, 18, 288, 0, 0, gScenarioBufferData118_BGR888_TO_GREY, 32, gScenarioKernelData118_BGR888_TO_GREY, 4},
{4, 18, 576, 0, 0, gScenarioBufferData119_BGR888_TO_GREY, 32, gScenarioKernelData119_BGR888_TO_GREY, 4},
{6, 18, 864, 0, 0, gScenarioBufferData120_BGR888_TO_GREY, 32, gScenarioKernelData120_BGR888_TO_GREY, 4},
{8, 18, 1152, 0, 0, gScenarioBufferData121_BGR888_TO_GREY, 32, gScenarioKernelData121_BGR888_TO_GREY, 4},
{10, 18, 1440, 0, 0, gScenarioBufferData122_BGR888_TO_GREY, 32, gScenarioKernelData122_BGR888_TO_GREY, 4},
{12, 18, 1728, 0, 0, gScenarioBufferData123_BGR888_TO_GREY, 32, gScenarioKernelData123_BGR888_TO_GREY, 4},
{16, 18, 2304, 0, 0, gScenarioBufferData124_BGR888_TO_GREY, 32, gScenarioKernelData124_BGR888_TO_GREY, 4},
{20, 18, 2880, 0, 0, gScenarioBufferData125_BGR888_TO_GREY, 32, gScenarioKernelData125_BGR888_TO_GREY, 4},
{24, 18, 3456, 0, 0, gScenarioBufferData126_BGR888_TO_GREY, 32, gScenarioKernelData126_BGR888_TO_GREY, 4},
{2, 20, 320, 0, 0, gScenarioBufferData127_BGR888_TO_GREY, 32, gScenarioKernelData127_BGR888_TO_GREY, 4},
{4, 20, 640, 0, 0, gScenarioBufferData128_BGR888_TO_GREY, 32, gScenarioKernelData128_BGR888_TO_GREY, 4},
{6, 20, 960, 0, 0, gScenarioBufferData129_BGR888_TO_GREY, 32, gScenarioKernelData129_BGR888_TO_GREY, 4},
{8, 20, 1280, 0, 0, gScenarioBufferData130_BGR888_TO_GREY, 32, gScenarioKernelData130_BGR888_TO_GREY, 4},
{10, 20, 1600, 0, 0, gScenarioBufferData131_BGR888_TO_GREY, 32, gScenarioKernelData131_BGR888_TO_GREY, 4},
{12, 20, 1920, 0, 0, gScenarioBufferData132_BGR888_TO_GREY, 32, gScenarioKernelData132_BGR888_TO_GREY, 4},
{16, 20, 2560, 0, 0, gScenarioBufferData133_BGR888_TO_GREY, 32, gScenarioKernelData133_BGR888_TO_GREY, 4},
{20, 20, 3200, 0, 0, gScenarioBufferData134_BGR888_TO_GREY, 32, gScenarioKernelData134_BGR888_TO_GREY, 4},
{24, 20, 3840, 0, 0, gScenarioBufferData135_BGR888_TO_GREY, 32, gScenarioKernelData135_BGR888_TO_GREY, 4},
{2, 22, 352, 0, 0, gScenarioBufferData136_BGR888_TO_GREY, 32, gScenarioKernelData136_BGR888_TO_GREY, 4},
{4, 22, 704, 0, 0, gScenarioBufferData137_BGR888_TO_GREY, 32, gScenarioKernelData137_BGR888_TO_GREY, 4},
{6, 22, 1056, 0, 0, gScenarioBufferData138_BGR888_TO_GREY, 32, gScenarioKernelData138_BGR888_TO_GREY, 4},
{8, 22, 1408, 0, 0, gScenarioBufferData139_BGR888_TO_GREY, 32, gScenarioKernelData139_BGR888_TO_GREY, 4},
{10, 22, 1760, 0, 0, gScenarioBufferData140_BGR888_TO_GREY, 32, gScenarioKernelData140_BGR888_TO_GREY, 4},
{12, 22, 2112, 0, 0, gScenarioBufferData141_BGR888_TO_GREY, 32, gScenarioKernelData141_BGR888_TO_GREY, 4},
{16, 22, 2816, 0, 0, gScenarioBufferData142_BGR888_TO_GREY, 32, gScenarioKernelData142_BGR888_TO_GREY, 4},
{20, 22, 3520, 0, 0, gScenarioBufferData143_BGR888_TO_GREY, 32, gScenarioKernelData143_BGR888_TO_GREY, 4},
{2, 24, 384, 0, 0, gScenarioBufferData144_BGR888_TO_GREY, 32, gScenarioKernelData144_BGR888_TO_GREY, 4},
{4, 24, 768, 0, 0, gScenarioBufferData145_BGR888_TO_GREY, 32, gScenarioKernelData145_BGR888_TO_GREY, 4},
{6, 24, 1152, 0, 0, gScenarioBufferData146_BGR888_TO_GREY, 32, gScenarioKernelData146_BGR888_TO_GREY, 4},
{8, 24, 1536, 0, 0, gScenarioBufferData147_BGR888_TO_GREY, 32, gScenarioKernelData147_BGR888_TO_GREY, 4},
{10, 24, 1920, 0, 0, gScenarioBufferData148_BGR888_TO_GREY, 32, gScenarioKernelData148_BGR888_TO_GREY, 4},
{12, 24, 2304, 0, 0, gScenarioBufferData149_BGR888_TO_GREY, 32, gScenarioKernelData149_BGR888_TO_GREY, 4},
{16, 24, 3072, 0, 0, gScenarioBufferData150_BGR888_TO_GREY, 32, gScenarioKernelData150_BGR888_TO_GREY, 4},
{20, 24, 3840, 0, 0, gScenarioBufferData151_BGR888_TO_GREY, 32, gScenarioKernelData151_BGR888_TO_GREY, 4},
{2, 26, 416, 0, 0, gScenarioBufferData152_BGR888_TO_GREY, 32, gScenarioKernelData152_BGR888_TO_GREY, 4},
{4, 26, 832, 0, 0, gScenarioBufferData153_BGR888_TO_GREY, 32, gScenarioKernelData153_BGR888_TO_GREY, 4},
{6, 26, 1248, 0, 0, gScenarioBufferData154_BGR888_TO_GREY, 32, gScenarioKernelData154_BGR888_TO_GREY, 4},
{8, 26, 1664, 0, 0, gScenarioBufferData155_BGR888_TO_GREY, 32, gScenarioKernelData155_BGR888_TO_GREY, 4},
{10, 26, 2080, 0, 0, gScenarioBufferData156_BGR888_TO_GREY, 32, gScenarioKernelData156_BGR888_TO_GREY, 4},
{12, 26, 2496, 0, 0, gScenarioBufferData157_BGR888_TO_GREY, 32, gScenarioKernelData157_BGR888_TO_GREY, 4},
{16, 26, 3328, 0, 0, gScenarioBufferData158_BGR888_TO_GREY, 32, gScenarioKernelData158_BGR888_TO_GREY, 4},
{2, 28, 448, 0, 0, gScenarioBufferData159_BGR888_TO_GREY, 32, gScenarioKernelData159_BGR888_TO_GREY, 4},
{4, 28, 896, 0, 0, gScenarioBufferData160_BGR888_TO_GREY, 32, gScenarioKernelData160_BGR888_TO_GREY, 4},
{6, 28, 1344, 0, 0, gScenarioBufferData161_BGR888_TO_GREY, 32, gScenarioKernelData161_BGR888_TO_GREY, 4},
{8, 28, 1792, 0, 0, gScenarioBufferData162_BGR888_TO_GREY, 32, gScenarioKernelData162_BGR888_TO_GREY, 4},
{10, 28, 2240, 0, 0, gScenarioBufferData163_BGR888_TO_GREY, 32, gScenarioKernelData163_BGR888_TO_GREY, 4},
{12, 28, 2688, 0, 0, gScenarioBufferData164_BGR888_TO_GREY, 32, gScenarioKernelData164_BGR888_TO_GREY, 4},
{16, 28, 3584, 0, 0, gScenarioBufferData165_BGR888_TO_GREY, 32, gScenarioKernelData165_BGR888_TO_GREY, 4},
{2, 30, 480, 0, 0, gScenarioBufferData166_BGR888_TO_GREY, 32, gScenarioKernelData166_BGR888_TO_GREY, 4},
{4, 30, 960, 0, 0, gScenarioBufferData167_BGR888_TO_GREY, 32, gScenarioKernelData167_BGR888_TO_GREY, 4},
{6, 30, 1440, 0, 0, gScenarioBufferData168_BGR888_TO_GREY, 32, gScenarioKernelData168_BGR888_TO_GREY, 4},
{8, 30, 1920, 0, 0, gScenarioBufferData169_BGR888_TO_GREY, 32, gScenarioKernelData169_BGR888_TO_GREY, 4},
{10, 30, 2400, 0, 0, gScenarioBufferData170_BGR888_TO_GREY, 32, gScenarioKernelData170_BGR888_TO_GREY, 4},
{12, 30, 2880, 0, 0, gScenarioBufferData171_BGR888_TO_GREY, 32, gScenarioKernelData171_BGR888_TO_GREY, 4},
{16, 30, 3840, 0, 0, gScenarioBufferData172_BGR888_TO_GREY, 32, gScenarioKernelData172_BGR888_TO_GREY, 4},
{2, 32, 512, 0, 0, gScenarioBufferData173_BGR888_TO_GREY, 32, gScenarioKernelData173_BGR888_TO_GREY, 4},
{4, 32, 1024, 0, 0, gScenarioBufferData174_BGR888_TO_GREY, 32, gScenarioKernelData174_BGR888_TO_GREY, 4},
{6, 32, 1536, 0, 0, gScenarioBufferData175_BGR888_TO_GREY, 32, gScenarioKernelData175_BGR888_TO_GREY, 4},
{8, 32, 2048, 0, 0, gScenarioBufferData176_BGR888_TO_GREY, 32, gScenarioKernelData176_BGR888_TO_GREY, 4},
{10, 32, 2560, 0, 0, gScenarioBufferData177_BGR888_TO_GREY, 32, gScenarioKernelData177_BGR888_TO_GREY, 4},
{12, 32, 3072, 0, 0, gScenarioBufferData178_BGR888_TO_GREY, 32, gScenarioKernelData178_BGR888_TO_GREY, 4}
};

static acf_scenario_list gScenarioList_BGR888_TO_GREY = {
179, //number of scenarios
gScenarioArray_BGR888_TO_GREY};
//**************************************************************

class BGR888_TO_GREY : public ACF_Process_APU
{

public:
   BGR888_TO_GREY(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("BGR888_TO_GREY");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("BGR888_TO_GREY",
                                        BGR888_TO_GREY_LOAD_SEGMENTS,
                                        BGR888_TO_GREY_LOAD_PMEM, BGR888_TO_GREY_LOAD_PMEM_SIZE,
                                        BGR888_TO_GREY_LOAD_DMEM, BGR888_TO_GREY_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(BGR888_TO_GREY_APEX_LOG_BUFFER, BGR888_TO_GREY_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_BGR888_TO_GREY);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_BGR888_TO_GREY

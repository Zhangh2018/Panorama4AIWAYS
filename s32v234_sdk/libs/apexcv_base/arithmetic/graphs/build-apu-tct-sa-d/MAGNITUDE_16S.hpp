#ifndef _ACF_PROCESS_APU_MAGNITUDE_16S
#define _ACF_PROCESS_APU_MAGNITUDE_16S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <MAGNITUDE_16S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_MAGNITUDE_16S[] = {{1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_MAGNITUDE_16S[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_MAGNITUDE_16S[] = {{3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_MAGNITUDE_16S[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_MAGNITUDE_16S[] = {{5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_MAGNITUDE_16S[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_MAGNITUDE_16S[] = {{7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_MAGNITUDE_16S[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_MAGNITUDE_16S[] = {{9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_MAGNITUDE_16S[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_MAGNITUDE_16S[] = {{11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_MAGNITUDE_16S[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_MAGNITUDE_16S[] = {{13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_MAGNITUDE_16S[] = {{14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_MAGNITUDE_16S[] = {{15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_MAGNITUDE_16S[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_MAGNITUDE_16S[] = {{18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}, {18, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_MAGNITUDE_16S[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_MAGNITUDE_16S[] = {{22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}, {22, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_MAGNITUDE_16S[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_MAGNITUDE_16S[] = {{26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}, {26, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_MAGNITUDE_16S[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_MAGNITUDE_16S[] = {{30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}, {30, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_MAGNITUDE_16S[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_MAGNITUDE_16S[] = {{36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}, {36, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_MAGNITUDE_16S[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_MAGNITUDE_16S[] = {{44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}, {44, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_MAGNITUDE_16S[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_MAGNITUDE_16S[] = {{52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}, {52, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_MAGNITUDE_16S[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}, {56, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_MAGNITUDE_16S[] = {{60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}, {60, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_MAGNITUDE_16S[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_MAGNITUDE_16S[] = {{1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_MAGNITUDE_16S[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_MAGNITUDE_16S[] = {{3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_MAGNITUDE_16S[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_MAGNITUDE_16S[] = {{5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_MAGNITUDE_16S[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_MAGNITUDE_16S[] = {{7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_MAGNITUDE_16S[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_MAGNITUDE_16S[] = {{9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_MAGNITUDE_16S[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_MAGNITUDE_16S[] = {{11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_MAGNITUDE_16S[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_MAGNITUDE_16S[] = {{13, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_MAGNITUDE_16S[] = {{14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_MAGNITUDE_16S[] = {{15, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_MAGNITUDE_16S[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_MAGNITUDE_16S[] = {{18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}, {18, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_MAGNITUDE_16S[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_MAGNITUDE_16S[] = {{22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}, {22, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_MAGNITUDE_16S[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_MAGNITUDE_16S[] = {{26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}, {26, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_MAGNITUDE_16S[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_MAGNITUDE_16S[] = {{30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}, {30, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_MAGNITUDE_16S[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_MAGNITUDE_16S[] = {{36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}, {36, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_MAGNITUDE_16S[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_MAGNITUDE_16S[] = {{44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}, {44, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_MAGNITUDE_16S[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_MAGNITUDE_16S[] = {{52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}, {52, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_MAGNITUDE_16S[] = {{56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}, {56, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_MAGNITUDE_16S[] = {{60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}, {60, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_MAGNITUDE_16S[] = {{64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_MAGNITUDE_16S[] = {{1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_MAGNITUDE_16S[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_MAGNITUDE_16S[] = {{3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_MAGNITUDE_16S[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_MAGNITUDE_16S[] = {{5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_MAGNITUDE_16S[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_MAGNITUDE_16S[] = {{7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_MAGNITUDE_16S[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_MAGNITUDE_16S[] = {{9, 3, 2, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_MAGNITUDE_16S[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_MAGNITUDE_16S[] = {{11, 3, 2, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_MAGNITUDE_16S[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_MAGNITUDE_16S[] = {{13, 3, 2, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_MAGNITUDE_16S[] = {{14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_MAGNITUDE_16S[] = {{15, 3, 2, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_MAGNITUDE_16S[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_MAGNITUDE_16S[] = {{18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}, {18, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_MAGNITUDE_16S[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_MAGNITUDE_16S[] = {{22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}, {22, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_MAGNITUDE_16S[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_MAGNITUDE_16S[] = {{26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}, {26, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_MAGNITUDE_16S[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_MAGNITUDE_16S[] = {{30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}, {30, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_MAGNITUDE_16S[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_MAGNITUDE_16S[] = {{36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}, {36, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_MAGNITUDE_16S[] = {{40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}, {40, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_MAGNITUDE_16S[] = {{44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}, {44, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_MAGNITUDE_16S[] = {{48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_MAGNITUDE_16S[] = {{52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}, {52, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_MAGNITUDE_16S[] = {{56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}, {56, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_MAGNITUDE_16S[] = {{60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}, {60, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_MAGNITUDE_16S[] = {{64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_MAGNITUDE_16S[] = {{1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_MAGNITUDE_16S[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_MAGNITUDE_16S[] = {{3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_MAGNITUDE_16S[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_MAGNITUDE_16S[] = {{5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_MAGNITUDE_16S[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_MAGNITUDE_16S[] = {{7, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_MAGNITUDE_16S[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_MAGNITUDE_16S[] = {{9, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_MAGNITUDE_16S[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_MAGNITUDE_16S[] = {{11, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_MAGNITUDE_16S[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_MAGNITUDE_16S[] = {{13, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_MAGNITUDE_16S[] = {{14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_MAGNITUDE_16S[] = {{15, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_MAGNITUDE_16S[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_MAGNITUDE_16S[] = {{18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}, {18, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_MAGNITUDE_16S[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_MAGNITUDE_16S[] = {{22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}, {22, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_MAGNITUDE_16S[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_MAGNITUDE_16S[] = {{26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}, {26, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_MAGNITUDE_16S[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_MAGNITUDE_16S[] = {{30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}, {30, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_MAGNITUDE_16S[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_MAGNITUDE_16S[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_MAGNITUDE_16S[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_MAGNITUDE_16S[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_MAGNITUDE_16S[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_MAGNITUDE_16S[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_MAGNITUDE_16S[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_MAGNITUDE_16S[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_MAGNITUDE_16S[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_MAGNITUDE_16S[] = {{1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_MAGNITUDE_16S[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_MAGNITUDE_16S[] = {{3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_MAGNITUDE_16S[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_MAGNITUDE_16S[] = {{5, 5, 2, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_MAGNITUDE_16S[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_MAGNITUDE_16S[] = {{7, 5, 2, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_MAGNITUDE_16S[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_MAGNITUDE_16S[] = {{9, 5, 2, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_MAGNITUDE_16S[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_MAGNITUDE_16S[] = {{11, 5, 2, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}, {11, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_MAGNITUDE_16S[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_MAGNITUDE_16S[] = {{13, 5, 2, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}, {13, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_MAGNITUDE_16S[] = {{14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}, {14, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_MAGNITUDE_16S[] = {{15, 5, 2, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}, {15, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_MAGNITUDE_16S[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_MAGNITUDE_16S[] = {{18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}, {18, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_MAGNITUDE_16S[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_MAGNITUDE_16S[] = {{22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}, {22, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_MAGNITUDE_16S[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_MAGNITUDE_16S[] = {{26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}, {26, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_MAGNITUDE_16S[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_MAGNITUDE_16S[] = {{30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}, {30, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_MAGNITUDE_16S[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_MAGNITUDE_16S[] = {{36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}, {36, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_MAGNITUDE_16S[] = {{40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}, {40, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_MAGNITUDE_16S[] = {{44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}, {44, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_MAGNITUDE_16S[] = {{48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}, {48, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_MAGNITUDE_16S[] = {{52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}, {52, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData157_MAGNITUDE_16S[] = {{56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}, {56, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData158_MAGNITUDE_16S[] = {{60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}, {60, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData159_MAGNITUDE_16S[] = {{64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}, {64, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData160_MAGNITUDE_16S[] = {{1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData161_MAGNITUDE_16S[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData162_MAGNITUDE_16S[] = {{3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData163_MAGNITUDE_16S[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData164_MAGNITUDE_16S[] = {{5, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData165_MAGNITUDE_16S[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData166_MAGNITUDE_16S[] = {{7, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData167_MAGNITUDE_16S[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData168_MAGNITUDE_16S[] = {{9, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}, {9, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData169_MAGNITUDE_16S[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData170_MAGNITUDE_16S[] = {{11, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}, {11, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData171_MAGNITUDE_16S[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData172_MAGNITUDE_16S[] = {{13, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}, {13, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData173_MAGNITUDE_16S[] = {{14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}, {14, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData174_MAGNITUDE_16S[] = {{15, 6, 2, 0, {0, 0, 0, 0}}, {15, 6, 2, 0, {0, 0, 0, 0}}, {15, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData175_MAGNITUDE_16S[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData176_MAGNITUDE_16S[] = {{18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}, {18, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData177_MAGNITUDE_16S[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData178_MAGNITUDE_16S[] = {{22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}, {22, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData179_MAGNITUDE_16S[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData180_MAGNITUDE_16S[] = {{26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}, {26, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData181_MAGNITUDE_16S[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData182_MAGNITUDE_16S[] = {{30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}, {30, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData183_MAGNITUDE_16S[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData184_MAGNITUDE_16S[] = {{36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}, {36, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData185_MAGNITUDE_16S[] = {{40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}, {40, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData186_MAGNITUDE_16S[] = {{44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}, {44, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData187_MAGNITUDE_16S[] = {{48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}, {48, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData188_MAGNITUDE_16S[] = {{52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}, {52, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData189_MAGNITUDE_16S[] = {{1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData190_MAGNITUDE_16S[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData191_MAGNITUDE_16S[] = {{3, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData192_MAGNITUDE_16S[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData193_MAGNITUDE_16S[] = {{5, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData194_MAGNITUDE_16S[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData195_MAGNITUDE_16S[] = {{7, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData196_MAGNITUDE_16S[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData197_MAGNITUDE_16S[] = {{9, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData198_MAGNITUDE_16S[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData199_MAGNITUDE_16S[] = {{11, 8, 2, 0, {0, 0, 0, 0}}, {11, 8, 2, 0, {0, 0, 0, 0}}, {11, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData200_MAGNITUDE_16S[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData201_MAGNITUDE_16S[] = {{13, 8, 2, 0, {0, 0, 0, 0}}, {13, 8, 2, 0, {0, 0, 0, 0}}, {13, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData202_MAGNITUDE_16S[] = {{14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData203_MAGNITUDE_16S[] = {{15, 8, 2, 0, {0, 0, 0, 0}}, {15, 8, 2, 0, {0, 0, 0, 0}}, {15, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData204_MAGNITUDE_16S[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData205_MAGNITUDE_16S[] = {{18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}, {18, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData206_MAGNITUDE_16S[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData207_MAGNITUDE_16S[] = {{22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}, {22, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData208_MAGNITUDE_16S[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData209_MAGNITUDE_16S[] = {{26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}, {26, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData210_MAGNITUDE_16S[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData211_MAGNITUDE_16S[] = {{30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}, {30, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData212_MAGNITUDE_16S[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData213_MAGNITUDE_16S[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData214_MAGNITUDE_16S[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData215_MAGNITUDE_16S[] = {{1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData216_MAGNITUDE_16S[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData217_MAGNITUDE_16S[] = {{3, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData218_MAGNITUDE_16S[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData219_MAGNITUDE_16S[] = {{5, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData220_MAGNITUDE_16S[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData221_MAGNITUDE_16S[] = {{7, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}, {7, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData222_MAGNITUDE_16S[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData223_MAGNITUDE_16S[] = {{9, 10, 2, 0, {0, 0, 0, 0}}, {9, 10, 2, 0, {0, 0, 0, 0}}, {9, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData224_MAGNITUDE_16S[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData225_MAGNITUDE_16S[] = {{11, 10, 2, 0, {0, 0, 0, 0}}, {11, 10, 2, 0, {0, 0, 0, 0}}, {11, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData226_MAGNITUDE_16S[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData227_MAGNITUDE_16S[] = {{13, 10, 2, 0, {0, 0, 0, 0}}, {13, 10, 2, 0, {0, 0, 0, 0}}, {13, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData228_MAGNITUDE_16S[] = {{14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}, {14, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData229_MAGNITUDE_16S[] = {{15, 10, 2, 0, {0, 0, 0, 0}}, {15, 10, 2, 0, {0, 0, 0, 0}}, {15, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData230_MAGNITUDE_16S[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData231_MAGNITUDE_16S[] = {{18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}, {18, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData232_MAGNITUDE_16S[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData233_MAGNITUDE_16S[] = {{22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}, {22, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData234_MAGNITUDE_16S[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData235_MAGNITUDE_16S[] = {{26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}, {26, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData236_MAGNITUDE_16S[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData237_MAGNITUDE_16S[] = {{30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}, {30, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData238_MAGNITUDE_16S[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData239_MAGNITUDE_16S[] = {{1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData240_MAGNITUDE_16S[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData241_MAGNITUDE_16S[] = {{3, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData242_MAGNITUDE_16S[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData243_MAGNITUDE_16S[] = {{5, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData244_MAGNITUDE_16S[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData245_MAGNITUDE_16S[] = {{7, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData246_MAGNITUDE_16S[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData247_MAGNITUDE_16S[] = {{9, 12, 2, 0, {0, 0, 0, 0}}, {9, 12, 2, 0, {0, 0, 0, 0}}, {9, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData248_MAGNITUDE_16S[] = {{10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData249_MAGNITUDE_16S[] = {{11, 12, 2, 0, {0, 0, 0, 0}}, {11, 12, 2, 0, {0, 0, 0, 0}}, {11, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData250_MAGNITUDE_16S[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData251_MAGNITUDE_16S[] = {{13, 12, 2, 0, {0, 0, 0, 0}}, {13, 12, 2, 0, {0, 0, 0, 0}}, {13, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData252_MAGNITUDE_16S[] = {{14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData253_MAGNITUDE_16S[] = {{15, 12, 2, 0, {0, 0, 0, 0}}, {15, 12, 2, 0, {0, 0, 0, 0}}, {15, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData254_MAGNITUDE_16S[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData255_MAGNITUDE_16S[] = {{18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}, {18, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData256_MAGNITUDE_16S[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData257_MAGNITUDE_16S[] = {{22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}, {22, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData258_MAGNITUDE_16S[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData259_MAGNITUDE_16S[] = {{26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}, {26, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData260_MAGNITUDE_16S[] = {{1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData261_MAGNITUDE_16S[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData262_MAGNITUDE_16S[] = {{3, 14, 2, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData263_MAGNITUDE_16S[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData264_MAGNITUDE_16S[] = {{5, 14, 2, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}, {5, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData265_MAGNITUDE_16S[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData266_MAGNITUDE_16S[] = {{7, 14, 2, 0, {0, 0, 0, 0}}, {7, 14, 2, 0, {0, 0, 0, 0}}, {7, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData267_MAGNITUDE_16S[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData268_MAGNITUDE_16S[] = {{9, 14, 2, 0, {0, 0, 0, 0}}, {9, 14, 2, 0, {0, 0, 0, 0}}, {9, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData269_MAGNITUDE_16S[] = {{10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}, {10, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData270_MAGNITUDE_16S[] = {{11, 14, 2, 0, {0, 0, 0, 0}}, {11, 14, 2, 0, {0, 0, 0, 0}}, {11, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData271_MAGNITUDE_16S[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData272_MAGNITUDE_16S[] = {{13, 14, 2, 0, {0, 0, 0, 0}}, {13, 14, 2, 0, {0, 0, 0, 0}}, {13, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData273_MAGNITUDE_16S[] = {{14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}, {14, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData274_MAGNITUDE_16S[] = {{15, 14, 2, 0, {0, 0, 0, 0}}, {15, 14, 2, 0, {0, 0, 0, 0}}, {15, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData275_MAGNITUDE_16S[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData276_MAGNITUDE_16S[] = {{18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}, {18, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData277_MAGNITUDE_16S[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData278_MAGNITUDE_16S[] = {{22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}, {22, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData279_MAGNITUDE_16S[] = {{1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData280_MAGNITUDE_16S[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData281_MAGNITUDE_16S[] = {{3, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData282_MAGNITUDE_16S[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData283_MAGNITUDE_16S[] = {{5, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData284_MAGNITUDE_16S[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData285_MAGNITUDE_16S[] = {{7, 16, 2, 0, {0, 0, 0, 0}}, {7, 16, 2, 0, {0, 0, 0, 0}}, {7, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData286_MAGNITUDE_16S[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData287_MAGNITUDE_16S[] = {{9, 16, 2, 0, {0, 0, 0, 0}}, {9, 16, 2, 0, {0, 0, 0, 0}}, {9, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData288_MAGNITUDE_16S[] = {{10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData289_MAGNITUDE_16S[] = {{11, 16, 2, 0, {0, 0, 0, 0}}, {11, 16, 2, 0, {0, 0, 0, 0}}, {11, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData290_MAGNITUDE_16S[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData291_MAGNITUDE_16S[] = {{13, 16, 2, 0, {0, 0, 0, 0}}, {13, 16, 2, 0, {0, 0, 0, 0}}, {13, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData292_MAGNITUDE_16S[] = {{14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}, {14, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData293_MAGNITUDE_16S[] = {{15, 16, 2, 0, {0, 0, 0, 0}}, {15, 16, 2, 0, {0, 0, 0, 0}}, {15, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData294_MAGNITUDE_16S[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData295_MAGNITUDE_16S[] = {{18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}, {18, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData296_MAGNITUDE_16S[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData297_MAGNITUDE_16S[] = {{1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData298_MAGNITUDE_16S[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData299_MAGNITUDE_16S[] = {{3, 18, 2, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}, {3, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData300_MAGNITUDE_16S[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData301_MAGNITUDE_16S[] = {{5, 18, 2, 0, {0, 0, 0, 0}}, {5, 18, 2, 0, {0, 0, 0, 0}}, {5, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData302_MAGNITUDE_16S[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData303_MAGNITUDE_16S[] = {{7, 18, 2, 0, {0, 0, 0, 0}}, {7, 18, 2, 0, {0, 0, 0, 0}}, {7, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData304_MAGNITUDE_16S[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData305_MAGNITUDE_16S[] = {{9, 18, 2, 0, {0, 0, 0, 0}}, {9, 18, 2, 0, {0, 0, 0, 0}}, {9, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData306_MAGNITUDE_16S[] = {{10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}, {10, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData307_MAGNITUDE_16S[] = {{11, 18, 2, 0, {0, 0, 0, 0}}, {11, 18, 2, 0, {0, 0, 0, 0}}, {11, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData308_MAGNITUDE_16S[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData309_MAGNITUDE_16S[] = {{13, 18, 2, 0, {0, 0, 0, 0}}, {13, 18, 2, 0, {0, 0, 0, 0}}, {13, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData310_MAGNITUDE_16S[] = {{14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}, {14, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData311_MAGNITUDE_16S[] = {{15, 18, 2, 0, {0, 0, 0, 0}}, {15, 18, 2, 0, {0, 0, 0, 0}}, {15, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData312_MAGNITUDE_16S[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData313_MAGNITUDE_16S[] = {{1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData314_MAGNITUDE_16S[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData315_MAGNITUDE_16S[] = {{3, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData316_MAGNITUDE_16S[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData317_MAGNITUDE_16S[] = {{5, 20, 2, 0, {0, 0, 0, 0}}, {5, 20, 2, 0, {0, 0, 0, 0}}, {5, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData318_MAGNITUDE_16S[] = {{6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData319_MAGNITUDE_16S[] = {{7, 20, 2, 0, {0, 0, 0, 0}}, {7, 20, 2, 0, {0, 0, 0, 0}}, {7, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData320_MAGNITUDE_16S[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData321_MAGNITUDE_16S[] = {{9, 20, 2, 0, {0, 0, 0, 0}}, {9, 20, 2, 0, {0, 0, 0, 0}}, {9, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData322_MAGNITUDE_16S[] = {{10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData323_MAGNITUDE_16S[] = {{11, 20, 2, 0, {0, 0, 0, 0}}, {11, 20, 2, 0, {0, 0, 0, 0}}, {11, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData324_MAGNITUDE_16S[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData325_MAGNITUDE_16S[] = {{13, 20, 2, 0, {0, 0, 0, 0}}, {13, 20, 2, 0, {0, 0, 0, 0}}, {13, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData326_MAGNITUDE_16S[] = {{14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}, {14, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData327_MAGNITUDE_16S[] = {{15, 20, 2, 0, {0, 0, 0, 0}}, {15, 20, 2, 0, {0, 0, 0, 0}}, {15, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData328_MAGNITUDE_16S[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData329_MAGNITUDE_16S[] = {{1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData330_MAGNITUDE_16S[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData331_MAGNITUDE_16S[] = {{3, 22, 2, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}, {3, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData332_MAGNITUDE_16S[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData333_MAGNITUDE_16S[] = {{5, 22, 2, 0, {0, 0, 0, 0}}, {5, 22, 2, 0, {0, 0, 0, 0}}, {5, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData334_MAGNITUDE_16S[] = {{6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}, {6, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData335_MAGNITUDE_16S[] = {{7, 22, 2, 0, {0, 0, 0, 0}}, {7, 22, 2, 0, {0, 0, 0, 0}}, {7, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData336_MAGNITUDE_16S[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData337_MAGNITUDE_16S[] = {{9, 22, 2, 0, {0, 0, 0, 0}}, {9, 22, 2, 0, {0, 0, 0, 0}}, {9, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData338_MAGNITUDE_16S[] = {{10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}, {10, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData339_MAGNITUDE_16S[] = {{11, 22, 2, 0, {0, 0, 0, 0}}, {11, 22, 2, 0, {0, 0, 0, 0}}, {11, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData340_MAGNITUDE_16S[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData341_MAGNITUDE_16S[] = {{13, 22, 2, 0, {0, 0, 0, 0}}, {13, 22, 2, 0, {0, 0, 0, 0}}, {13, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData342_MAGNITUDE_16S[] = {{14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}, {14, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData343_MAGNITUDE_16S[] = {{1, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData344_MAGNITUDE_16S[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData345_MAGNITUDE_16S[] = {{3, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData346_MAGNITUDE_16S[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData347_MAGNITUDE_16S[] = {{5, 24, 2, 0, {0, 0, 0, 0}}, {5, 24, 2, 0, {0, 0, 0, 0}}, {5, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData348_MAGNITUDE_16S[] = {{6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData349_MAGNITUDE_16S[] = {{7, 24, 2, 0, {0, 0, 0, 0}}, {7, 24, 2, 0, {0, 0, 0, 0}}, {7, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData350_MAGNITUDE_16S[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData351_MAGNITUDE_16S[] = {{9, 24, 2, 0, {0, 0, 0, 0}}, {9, 24, 2, 0, {0, 0, 0, 0}}, {9, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData352_MAGNITUDE_16S[] = {{10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}, {10, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData353_MAGNITUDE_16S[] = {{11, 24, 2, 0, {0, 0, 0, 0}}, {11, 24, 2, 0, {0, 0, 0, 0}}, {11, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData354_MAGNITUDE_16S[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData355_MAGNITUDE_16S[] = {{13, 24, 2, 0, {0, 0, 0, 0}}, {13, 24, 2, 0, {0, 0, 0, 0}}, {13, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData356_MAGNITUDE_16S[] = {{1, 26, 2, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData357_MAGNITUDE_16S[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData358_MAGNITUDE_16S[] = {{3, 26, 2, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}, {3, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData359_MAGNITUDE_16S[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData360_MAGNITUDE_16S[] = {{5, 26, 2, 0, {0, 0, 0, 0}}, {5, 26, 2, 0, {0, 0, 0, 0}}, {5, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData361_MAGNITUDE_16S[] = {{6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}, {6, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData362_MAGNITUDE_16S[] = {{7, 26, 2, 0, {0, 0, 0, 0}}, {7, 26, 2, 0, {0, 0, 0, 0}}, {7, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData363_MAGNITUDE_16S[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData364_MAGNITUDE_16S[] = {{9, 26, 2, 0, {0, 0, 0, 0}}, {9, 26, 2, 0, {0, 0, 0, 0}}, {9, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData365_MAGNITUDE_16S[] = {{10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}, {10, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData366_MAGNITUDE_16S[] = {{11, 26, 2, 0, {0, 0, 0, 0}}, {11, 26, 2, 0, {0, 0, 0, 0}}, {11, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData367_MAGNITUDE_16S[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData368_MAGNITUDE_16S[] = {{1, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData369_MAGNITUDE_16S[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData370_MAGNITUDE_16S[] = {{3, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}, {3, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData371_MAGNITUDE_16S[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData372_MAGNITUDE_16S[] = {{5, 28, 2, 0, {0, 0, 0, 0}}, {5, 28, 2, 0, {0, 0, 0, 0}}, {5, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData373_MAGNITUDE_16S[] = {{6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}, {6, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData374_MAGNITUDE_16S[] = {{7, 28, 2, 0, {0, 0, 0, 0}}, {7, 28, 2, 0, {0, 0, 0, 0}}, {7, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData375_MAGNITUDE_16S[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData376_MAGNITUDE_16S[] = {{9, 28, 2, 0, {0, 0, 0, 0}}, {9, 28, 2, 0, {0, 0, 0, 0}}, {9, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData377_MAGNITUDE_16S[] = {{10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}, {10, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData378_MAGNITUDE_16S[] = {{11, 28, 2, 0, {0, 0, 0, 0}}, {11, 28, 2, 0, {0, 0, 0, 0}}, {11, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData379_MAGNITUDE_16S[] = {{1, 30, 2, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData380_MAGNITUDE_16S[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData381_MAGNITUDE_16S[] = {{3, 30, 2, 0, {0, 0, 0, 0}}, {3, 30, 2, 0, {0, 0, 0, 0}}, {3, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData382_MAGNITUDE_16S[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData383_MAGNITUDE_16S[] = {{5, 30, 2, 0, {0, 0, 0, 0}}, {5, 30, 2, 0, {0, 0, 0, 0}}, {5, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData384_MAGNITUDE_16S[] = {{6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}, {6, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData385_MAGNITUDE_16S[] = {{7, 30, 2, 0, {0, 0, 0, 0}}, {7, 30, 2, 0, {0, 0, 0, 0}}, {7, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData386_MAGNITUDE_16S[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData387_MAGNITUDE_16S[] = {{9, 30, 2, 0, {0, 0, 0, 0}}, {9, 30, 2, 0, {0, 0, 0, 0}}, {9, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData388_MAGNITUDE_16S[] = {{10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}, {10, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData389_MAGNITUDE_16S[] = {{1, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData390_MAGNITUDE_16S[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData391_MAGNITUDE_16S[] = {{3, 32, 2, 0, {0, 0, 0, 0}}, {3, 32, 2, 0, {0, 0, 0, 0}}, {3, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData392_MAGNITUDE_16S[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData393_MAGNITUDE_16S[] = {{5, 32, 2, 0, {0, 0, 0, 0}}, {5, 32, 2, 0, {0, 0, 0, 0}}, {5, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData394_MAGNITUDE_16S[] = {{6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData395_MAGNITUDE_16S[] = {{7, 32, 2, 0, {0, 0, 0, 0}}, {7, 32, 2, 0, {0, 0, 0, 0}}, {7, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData396_MAGNITUDE_16S[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData397_MAGNITUDE_16S[] = {{9, 32, 2, 0, {0, 0, 0, 0}}, {9, 32, 2, 0, {0, 0, 0, 0}}, {9, 32, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData398_MAGNITUDE_16S[] = {{10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}, {10, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData157_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData158_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData159_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData160_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData161_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData162_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData163_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData164_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData165_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData166_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData167_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData168_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData169_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData170_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData171_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData172_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData173_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData174_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData175_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData176_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData177_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData178_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData179_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData180_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData181_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData182_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData183_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData184_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData185_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData186_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData187_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData188_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData189_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData190_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData191_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData192_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData193_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData194_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData195_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData196_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData197_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData198_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData199_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData200_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData201_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData202_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData203_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData204_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData205_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData206_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData207_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData208_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData209_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData210_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData211_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData212_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData213_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData214_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData215_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData216_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData217_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData218_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData219_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData220_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData221_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData222_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData223_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData224_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData225_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData226_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData227_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData228_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData229_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData230_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData231_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData232_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData233_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData234_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData235_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData236_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData237_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData238_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData239_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData240_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData241_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData242_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData243_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData244_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData245_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData246_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData247_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData248_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData249_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData250_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData251_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData252_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData253_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData254_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData255_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData256_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData257_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData258_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData259_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData260_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData261_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData262_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData263_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData264_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData265_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData266_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData267_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData268_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData269_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData270_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData271_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData272_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData273_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData274_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData275_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData276_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData277_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData278_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData279_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData280_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData281_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData282_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData283_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData284_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData285_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData286_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData287_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData288_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData289_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData290_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData291_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData292_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData293_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData294_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData295_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData296_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData297_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData298_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData299_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData300_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData301_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData302_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData303_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData304_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData305_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData306_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData307_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData308_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData309_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData310_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData311_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData312_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData313_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData314_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData315_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData316_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData317_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData318_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData319_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData320_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData321_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData322_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData323_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData324_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData325_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData326_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData327_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData328_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData329_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData330_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData331_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData332_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData333_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData334_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData335_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData336_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData337_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData338_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData339_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData340_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData341_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData342_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData343_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData344_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData345_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData346_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData347_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData348_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData349_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData350_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData351_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData352_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData353_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData354_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData355_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData356_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData357_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData358_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData359_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData360_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData361_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData362_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData363_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData364_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData365_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData366_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData367_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData368_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData369_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData370_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData371_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData372_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData373_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData374_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData375_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData376_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData377_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData378_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData379_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData380_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData381_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData382_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData383_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData384_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData385_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData386_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData387_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData388_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData389_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData390_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData391_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData392_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData393_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData394_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData395_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData396_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData397_MAGNITUDE_16S[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData398_MAGNITUDE_16S[] = {{0, 0}};

static acf_scenario gScenarioArray_MAGNITUDE_16S[] = {
{1, 1, 12, 0, 0, gScenarioBufferData0_MAGNITUDE_16S, 48, gScenarioKernelData0_MAGNITUDE_16S, 4},
{2, 1, 24, 0, 0, gScenarioBufferData1_MAGNITUDE_16S, 48, gScenarioKernelData1_MAGNITUDE_16S, 4},
{3, 1, 36, 0, 0, gScenarioBufferData2_MAGNITUDE_16S, 48, gScenarioKernelData2_MAGNITUDE_16S, 4},
{4, 1, 48, 0, 0, gScenarioBufferData3_MAGNITUDE_16S, 48, gScenarioKernelData3_MAGNITUDE_16S, 4},
{5, 1, 60, 0, 0, gScenarioBufferData4_MAGNITUDE_16S, 48, gScenarioKernelData4_MAGNITUDE_16S, 4},
{6, 1, 72, 0, 0, gScenarioBufferData5_MAGNITUDE_16S, 48, gScenarioKernelData5_MAGNITUDE_16S, 4},
{7, 1, 84, 0, 0, gScenarioBufferData6_MAGNITUDE_16S, 48, gScenarioKernelData6_MAGNITUDE_16S, 4},
{8, 1, 96, 0, 0, gScenarioBufferData7_MAGNITUDE_16S, 48, gScenarioKernelData7_MAGNITUDE_16S, 4},
{9, 1, 108, 0, 0, gScenarioBufferData8_MAGNITUDE_16S, 48, gScenarioKernelData8_MAGNITUDE_16S, 4},
{10, 1, 120, 0, 0, gScenarioBufferData9_MAGNITUDE_16S, 48, gScenarioKernelData9_MAGNITUDE_16S, 4},
{11, 1, 132, 0, 0, gScenarioBufferData10_MAGNITUDE_16S, 48, gScenarioKernelData10_MAGNITUDE_16S, 4},
{12, 1, 144, 0, 0, gScenarioBufferData11_MAGNITUDE_16S, 48, gScenarioKernelData11_MAGNITUDE_16S, 4},
{13, 1, 156, 0, 0, gScenarioBufferData12_MAGNITUDE_16S, 48, gScenarioKernelData12_MAGNITUDE_16S, 4},
{14, 1, 168, 0, 0, gScenarioBufferData13_MAGNITUDE_16S, 48, gScenarioKernelData13_MAGNITUDE_16S, 4},
{15, 1, 180, 0, 0, gScenarioBufferData14_MAGNITUDE_16S, 48, gScenarioKernelData14_MAGNITUDE_16S, 4},
{16, 1, 192, 0, 0, gScenarioBufferData15_MAGNITUDE_16S, 48, gScenarioKernelData15_MAGNITUDE_16S, 4},
{18, 1, 216, 0, 0, gScenarioBufferData16_MAGNITUDE_16S, 48, gScenarioKernelData16_MAGNITUDE_16S, 4},
{20, 1, 240, 0, 0, gScenarioBufferData17_MAGNITUDE_16S, 48, gScenarioKernelData17_MAGNITUDE_16S, 4},
{22, 1, 264, 0, 0, gScenarioBufferData18_MAGNITUDE_16S, 48, gScenarioKernelData18_MAGNITUDE_16S, 4},
{24, 1, 288, 0, 0, gScenarioBufferData19_MAGNITUDE_16S, 48, gScenarioKernelData19_MAGNITUDE_16S, 4},
{26, 1, 312, 0, 0, gScenarioBufferData20_MAGNITUDE_16S, 48, gScenarioKernelData20_MAGNITUDE_16S, 4},
{28, 1, 336, 0, 0, gScenarioBufferData21_MAGNITUDE_16S, 48, gScenarioKernelData21_MAGNITUDE_16S, 4},
{30, 1, 360, 0, 0, gScenarioBufferData22_MAGNITUDE_16S, 48, gScenarioKernelData22_MAGNITUDE_16S, 4},
{32, 1, 384, 0, 0, gScenarioBufferData23_MAGNITUDE_16S, 48, gScenarioKernelData23_MAGNITUDE_16S, 4},
{36, 1, 432, 0, 0, gScenarioBufferData24_MAGNITUDE_16S, 48, gScenarioKernelData24_MAGNITUDE_16S, 4},
{40, 1, 480, 0, 0, gScenarioBufferData25_MAGNITUDE_16S, 48, gScenarioKernelData25_MAGNITUDE_16S, 4},
{44, 1, 528, 0, 0, gScenarioBufferData26_MAGNITUDE_16S, 48, gScenarioKernelData26_MAGNITUDE_16S, 4},
{48, 1, 576, 0, 0, gScenarioBufferData27_MAGNITUDE_16S, 48, gScenarioKernelData27_MAGNITUDE_16S, 4},
{52, 1, 624, 0, 0, gScenarioBufferData28_MAGNITUDE_16S, 48, gScenarioKernelData28_MAGNITUDE_16S, 4},
{56, 1, 672, 0, 0, gScenarioBufferData29_MAGNITUDE_16S, 48, gScenarioKernelData29_MAGNITUDE_16S, 4},
{60, 1, 720, 0, 0, gScenarioBufferData30_MAGNITUDE_16S, 48, gScenarioKernelData30_MAGNITUDE_16S, 4},
{64, 1, 768, 0, 0, gScenarioBufferData31_MAGNITUDE_16S, 48, gScenarioKernelData31_MAGNITUDE_16S, 4},
{1, 2, 24, 0, 0, gScenarioBufferData32_MAGNITUDE_16S, 48, gScenarioKernelData32_MAGNITUDE_16S, 4},
{2, 2, 48, 0, 0, gScenarioBufferData33_MAGNITUDE_16S, 48, gScenarioKernelData33_MAGNITUDE_16S, 4},
{3, 2, 72, 0, 0, gScenarioBufferData34_MAGNITUDE_16S, 48, gScenarioKernelData34_MAGNITUDE_16S, 4},
{4, 2, 96, 0, 0, gScenarioBufferData35_MAGNITUDE_16S, 48, gScenarioKernelData35_MAGNITUDE_16S, 4},
{5, 2, 120, 0, 0, gScenarioBufferData36_MAGNITUDE_16S, 48, gScenarioKernelData36_MAGNITUDE_16S, 4},
{6, 2, 144, 0, 0, gScenarioBufferData37_MAGNITUDE_16S, 48, gScenarioKernelData37_MAGNITUDE_16S, 4},
{7, 2, 168, 0, 0, gScenarioBufferData38_MAGNITUDE_16S, 48, gScenarioKernelData38_MAGNITUDE_16S, 4},
{8, 2, 192, 0, 0, gScenarioBufferData39_MAGNITUDE_16S, 48, gScenarioKernelData39_MAGNITUDE_16S, 4},
{9, 2, 216, 0, 0, gScenarioBufferData40_MAGNITUDE_16S, 48, gScenarioKernelData40_MAGNITUDE_16S, 4},
{10, 2, 240, 0, 0, gScenarioBufferData41_MAGNITUDE_16S, 48, gScenarioKernelData41_MAGNITUDE_16S, 4},
{11, 2, 264, 0, 0, gScenarioBufferData42_MAGNITUDE_16S, 48, gScenarioKernelData42_MAGNITUDE_16S, 4},
{12, 2, 288, 0, 0, gScenarioBufferData43_MAGNITUDE_16S, 48, gScenarioKernelData43_MAGNITUDE_16S, 4},
{13, 2, 312, 0, 0, gScenarioBufferData44_MAGNITUDE_16S, 48, gScenarioKernelData44_MAGNITUDE_16S, 4},
{14, 2, 336, 0, 0, gScenarioBufferData45_MAGNITUDE_16S, 48, gScenarioKernelData45_MAGNITUDE_16S, 4},
{15, 2, 360, 0, 0, gScenarioBufferData46_MAGNITUDE_16S, 48, gScenarioKernelData46_MAGNITUDE_16S, 4},
{16, 2, 384, 0, 0, gScenarioBufferData47_MAGNITUDE_16S, 48, gScenarioKernelData47_MAGNITUDE_16S, 4},
{18, 2, 432, 0, 0, gScenarioBufferData48_MAGNITUDE_16S, 48, gScenarioKernelData48_MAGNITUDE_16S, 4},
{20, 2, 480, 0, 0, gScenarioBufferData49_MAGNITUDE_16S, 48, gScenarioKernelData49_MAGNITUDE_16S, 4},
{22, 2, 528, 0, 0, gScenarioBufferData50_MAGNITUDE_16S, 48, gScenarioKernelData50_MAGNITUDE_16S, 4},
{24, 2, 576, 0, 0, gScenarioBufferData51_MAGNITUDE_16S, 48, gScenarioKernelData51_MAGNITUDE_16S, 4},
{26, 2, 624, 0, 0, gScenarioBufferData52_MAGNITUDE_16S, 48, gScenarioKernelData52_MAGNITUDE_16S, 4},
{28, 2, 672, 0, 0, gScenarioBufferData53_MAGNITUDE_16S, 48, gScenarioKernelData53_MAGNITUDE_16S, 4},
{30, 2, 720, 0, 0, gScenarioBufferData54_MAGNITUDE_16S, 48, gScenarioKernelData54_MAGNITUDE_16S, 4},
{32, 2, 768, 0, 0, gScenarioBufferData55_MAGNITUDE_16S, 48, gScenarioKernelData55_MAGNITUDE_16S, 4},
{36, 2, 864, 0, 0, gScenarioBufferData56_MAGNITUDE_16S, 48, gScenarioKernelData56_MAGNITUDE_16S, 4},
{40, 2, 960, 0, 0, gScenarioBufferData57_MAGNITUDE_16S, 48, gScenarioKernelData57_MAGNITUDE_16S, 4},
{44, 2, 1056, 0, 0, gScenarioBufferData58_MAGNITUDE_16S, 48, gScenarioKernelData58_MAGNITUDE_16S, 4},
{48, 2, 1152, 0, 0, gScenarioBufferData59_MAGNITUDE_16S, 48, gScenarioKernelData59_MAGNITUDE_16S, 4},
{52, 2, 1248, 0, 0, gScenarioBufferData60_MAGNITUDE_16S, 48, gScenarioKernelData60_MAGNITUDE_16S, 4},
{56, 2, 1344, 0, 0, gScenarioBufferData61_MAGNITUDE_16S, 48, gScenarioKernelData61_MAGNITUDE_16S, 4},
{60, 2, 1440, 0, 0, gScenarioBufferData62_MAGNITUDE_16S, 48, gScenarioKernelData62_MAGNITUDE_16S, 4},
{64, 2, 1536, 0, 0, gScenarioBufferData63_MAGNITUDE_16S, 48, gScenarioKernelData63_MAGNITUDE_16S, 4},
{1, 3, 36, 0, 0, gScenarioBufferData64_MAGNITUDE_16S, 48, gScenarioKernelData64_MAGNITUDE_16S, 4},
{2, 3, 72, 0, 0, gScenarioBufferData65_MAGNITUDE_16S, 48, gScenarioKernelData65_MAGNITUDE_16S, 4},
{3, 3, 108, 0, 0, gScenarioBufferData66_MAGNITUDE_16S, 48, gScenarioKernelData66_MAGNITUDE_16S, 4},
{4, 3, 144, 0, 0, gScenarioBufferData67_MAGNITUDE_16S, 48, gScenarioKernelData67_MAGNITUDE_16S, 4},
{5, 3, 180, 0, 0, gScenarioBufferData68_MAGNITUDE_16S, 48, gScenarioKernelData68_MAGNITUDE_16S, 4},
{6, 3, 216, 0, 0, gScenarioBufferData69_MAGNITUDE_16S, 48, gScenarioKernelData69_MAGNITUDE_16S, 4},
{7, 3, 252, 0, 0, gScenarioBufferData70_MAGNITUDE_16S, 48, gScenarioKernelData70_MAGNITUDE_16S, 4},
{8, 3, 288, 0, 0, gScenarioBufferData71_MAGNITUDE_16S, 48, gScenarioKernelData71_MAGNITUDE_16S, 4},
{9, 3, 324, 0, 0, gScenarioBufferData72_MAGNITUDE_16S, 48, gScenarioKernelData72_MAGNITUDE_16S, 4},
{10, 3, 360, 0, 0, gScenarioBufferData73_MAGNITUDE_16S, 48, gScenarioKernelData73_MAGNITUDE_16S, 4},
{11, 3, 396, 0, 0, gScenarioBufferData74_MAGNITUDE_16S, 48, gScenarioKernelData74_MAGNITUDE_16S, 4},
{12, 3, 432, 0, 0, gScenarioBufferData75_MAGNITUDE_16S, 48, gScenarioKernelData75_MAGNITUDE_16S, 4},
{13, 3, 468, 0, 0, gScenarioBufferData76_MAGNITUDE_16S, 48, gScenarioKernelData76_MAGNITUDE_16S, 4},
{14, 3, 504, 0, 0, gScenarioBufferData77_MAGNITUDE_16S, 48, gScenarioKernelData77_MAGNITUDE_16S, 4},
{15, 3, 540, 0, 0, gScenarioBufferData78_MAGNITUDE_16S, 48, gScenarioKernelData78_MAGNITUDE_16S, 4},
{16, 3, 576, 0, 0, gScenarioBufferData79_MAGNITUDE_16S, 48, gScenarioKernelData79_MAGNITUDE_16S, 4},
{18, 3, 648, 0, 0, gScenarioBufferData80_MAGNITUDE_16S, 48, gScenarioKernelData80_MAGNITUDE_16S, 4},
{20, 3, 720, 0, 0, gScenarioBufferData81_MAGNITUDE_16S, 48, gScenarioKernelData81_MAGNITUDE_16S, 4},
{22, 3, 792, 0, 0, gScenarioBufferData82_MAGNITUDE_16S, 48, gScenarioKernelData82_MAGNITUDE_16S, 4},
{24, 3, 864, 0, 0, gScenarioBufferData83_MAGNITUDE_16S, 48, gScenarioKernelData83_MAGNITUDE_16S, 4},
{26, 3, 936, 0, 0, gScenarioBufferData84_MAGNITUDE_16S, 48, gScenarioKernelData84_MAGNITUDE_16S, 4},
{28, 3, 1008, 0, 0, gScenarioBufferData85_MAGNITUDE_16S, 48, gScenarioKernelData85_MAGNITUDE_16S, 4},
{30, 3, 1080, 0, 0, gScenarioBufferData86_MAGNITUDE_16S, 48, gScenarioKernelData86_MAGNITUDE_16S, 4},
{32, 3, 1152, 0, 0, gScenarioBufferData87_MAGNITUDE_16S, 48, gScenarioKernelData87_MAGNITUDE_16S, 4},
{36, 3, 1296, 0, 0, gScenarioBufferData88_MAGNITUDE_16S, 48, gScenarioKernelData88_MAGNITUDE_16S, 4},
{40, 3, 1440, 0, 0, gScenarioBufferData89_MAGNITUDE_16S, 48, gScenarioKernelData89_MAGNITUDE_16S, 4},
{44, 3, 1584, 0, 0, gScenarioBufferData90_MAGNITUDE_16S, 48, gScenarioKernelData90_MAGNITUDE_16S, 4},
{48, 3, 1728, 0, 0, gScenarioBufferData91_MAGNITUDE_16S, 48, gScenarioKernelData91_MAGNITUDE_16S, 4},
{52, 3, 1872, 0, 0, gScenarioBufferData92_MAGNITUDE_16S, 48, gScenarioKernelData92_MAGNITUDE_16S, 4},
{56, 3, 2016, 0, 0, gScenarioBufferData93_MAGNITUDE_16S, 48, gScenarioKernelData93_MAGNITUDE_16S, 4},
{60, 3, 2160, 0, 0, gScenarioBufferData94_MAGNITUDE_16S, 48, gScenarioKernelData94_MAGNITUDE_16S, 4},
{64, 3, 2304, 0, 0, gScenarioBufferData95_MAGNITUDE_16S, 48, gScenarioKernelData95_MAGNITUDE_16S, 4},
{1, 4, 48, 0, 0, gScenarioBufferData96_MAGNITUDE_16S, 48, gScenarioKernelData96_MAGNITUDE_16S, 4},
{2, 4, 96, 0, 0, gScenarioBufferData97_MAGNITUDE_16S, 48, gScenarioKernelData97_MAGNITUDE_16S, 4},
{3, 4, 144, 0, 0, gScenarioBufferData98_MAGNITUDE_16S, 48, gScenarioKernelData98_MAGNITUDE_16S, 4},
{4, 4, 192, 0, 0, gScenarioBufferData99_MAGNITUDE_16S, 48, gScenarioKernelData99_MAGNITUDE_16S, 4},
{5, 4, 240, 0, 0, gScenarioBufferData100_MAGNITUDE_16S, 48, gScenarioKernelData100_MAGNITUDE_16S, 4},
{6, 4, 288, 0, 0, gScenarioBufferData101_MAGNITUDE_16S, 48, gScenarioKernelData101_MAGNITUDE_16S, 4},
{7, 4, 336, 0, 0, gScenarioBufferData102_MAGNITUDE_16S, 48, gScenarioKernelData102_MAGNITUDE_16S, 4},
{8, 4, 384, 0, 0, gScenarioBufferData103_MAGNITUDE_16S, 48, gScenarioKernelData103_MAGNITUDE_16S, 4},
{9, 4, 432, 0, 0, gScenarioBufferData104_MAGNITUDE_16S, 48, gScenarioKernelData104_MAGNITUDE_16S, 4},
{10, 4, 480, 0, 0, gScenarioBufferData105_MAGNITUDE_16S, 48, gScenarioKernelData105_MAGNITUDE_16S, 4},
{11, 4, 528, 0, 0, gScenarioBufferData106_MAGNITUDE_16S, 48, gScenarioKernelData106_MAGNITUDE_16S, 4},
{12, 4, 576, 0, 0, gScenarioBufferData107_MAGNITUDE_16S, 48, gScenarioKernelData107_MAGNITUDE_16S, 4},
{13, 4, 624, 0, 0, gScenarioBufferData108_MAGNITUDE_16S, 48, gScenarioKernelData108_MAGNITUDE_16S, 4},
{14, 4, 672, 0, 0, gScenarioBufferData109_MAGNITUDE_16S, 48, gScenarioKernelData109_MAGNITUDE_16S, 4},
{15, 4, 720, 0, 0, gScenarioBufferData110_MAGNITUDE_16S, 48, gScenarioKernelData110_MAGNITUDE_16S, 4},
{16, 4, 768, 0, 0, gScenarioBufferData111_MAGNITUDE_16S, 48, gScenarioKernelData111_MAGNITUDE_16S, 4},
{18, 4, 864, 0, 0, gScenarioBufferData112_MAGNITUDE_16S, 48, gScenarioKernelData112_MAGNITUDE_16S, 4},
{20, 4, 960, 0, 0, gScenarioBufferData113_MAGNITUDE_16S, 48, gScenarioKernelData113_MAGNITUDE_16S, 4},
{22, 4, 1056, 0, 0, gScenarioBufferData114_MAGNITUDE_16S, 48, gScenarioKernelData114_MAGNITUDE_16S, 4},
{24, 4, 1152, 0, 0, gScenarioBufferData115_MAGNITUDE_16S, 48, gScenarioKernelData115_MAGNITUDE_16S, 4},
{26, 4, 1248, 0, 0, gScenarioBufferData116_MAGNITUDE_16S, 48, gScenarioKernelData116_MAGNITUDE_16S, 4},
{28, 4, 1344, 0, 0, gScenarioBufferData117_MAGNITUDE_16S, 48, gScenarioKernelData117_MAGNITUDE_16S, 4},
{30, 4, 1440, 0, 0, gScenarioBufferData118_MAGNITUDE_16S, 48, gScenarioKernelData118_MAGNITUDE_16S, 4},
{32, 4, 1536, 0, 0, gScenarioBufferData119_MAGNITUDE_16S, 48, gScenarioKernelData119_MAGNITUDE_16S, 4},
{36, 4, 1728, 0, 0, gScenarioBufferData120_MAGNITUDE_16S, 48, gScenarioKernelData120_MAGNITUDE_16S, 4},
{40, 4, 1920, 0, 0, gScenarioBufferData121_MAGNITUDE_16S, 48, gScenarioKernelData121_MAGNITUDE_16S, 4},
{44, 4, 2112, 0, 0, gScenarioBufferData122_MAGNITUDE_16S, 48, gScenarioKernelData122_MAGNITUDE_16S, 4},
{48, 4, 2304, 0, 0, gScenarioBufferData123_MAGNITUDE_16S, 48, gScenarioKernelData123_MAGNITUDE_16S, 4},
{52, 4, 2496, 0, 0, gScenarioBufferData124_MAGNITUDE_16S, 48, gScenarioKernelData124_MAGNITUDE_16S, 4},
{56, 4, 2688, 0, 0, gScenarioBufferData125_MAGNITUDE_16S, 48, gScenarioKernelData125_MAGNITUDE_16S, 4},
{60, 4, 2880, 0, 0, gScenarioBufferData126_MAGNITUDE_16S, 48, gScenarioKernelData126_MAGNITUDE_16S, 4},
{64, 4, 3072, 0, 0, gScenarioBufferData127_MAGNITUDE_16S, 48, gScenarioKernelData127_MAGNITUDE_16S, 4},
{1, 5, 60, 0, 0, gScenarioBufferData128_MAGNITUDE_16S, 48, gScenarioKernelData128_MAGNITUDE_16S, 4},
{2, 5, 120, 0, 0, gScenarioBufferData129_MAGNITUDE_16S, 48, gScenarioKernelData129_MAGNITUDE_16S, 4},
{3, 5, 180, 0, 0, gScenarioBufferData130_MAGNITUDE_16S, 48, gScenarioKernelData130_MAGNITUDE_16S, 4},
{4, 5, 240, 0, 0, gScenarioBufferData131_MAGNITUDE_16S, 48, gScenarioKernelData131_MAGNITUDE_16S, 4},
{5, 5, 300, 0, 0, gScenarioBufferData132_MAGNITUDE_16S, 48, gScenarioKernelData132_MAGNITUDE_16S, 4},
{6, 5, 360, 0, 0, gScenarioBufferData133_MAGNITUDE_16S, 48, gScenarioKernelData133_MAGNITUDE_16S, 4},
{7, 5, 420, 0, 0, gScenarioBufferData134_MAGNITUDE_16S, 48, gScenarioKernelData134_MAGNITUDE_16S, 4},
{8, 5, 480, 0, 0, gScenarioBufferData135_MAGNITUDE_16S, 48, gScenarioKernelData135_MAGNITUDE_16S, 4},
{9, 5, 540, 0, 0, gScenarioBufferData136_MAGNITUDE_16S, 48, gScenarioKernelData136_MAGNITUDE_16S, 4},
{10, 5, 600, 0, 0, gScenarioBufferData137_MAGNITUDE_16S, 48, gScenarioKernelData137_MAGNITUDE_16S, 4},
{11, 5, 660, 0, 0, gScenarioBufferData138_MAGNITUDE_16S, 48, gScenarioKernelData138_MAGNITUDE_16S, 4},
{12, 5, 720, 0, 0, gScenarioBufferData139_MAGNITUDE_16S, 48, gScenarioKernelData139_MAGNITUDE_16S, 4},
{13, 5, 780, 0, 0, gScenarioBufferData140_MAGNITUDE_16S, 48, gScenarioKernelData140_MAGNITUDE_16S, 4},
{14, 5, 840, 0, 0, gScenarioBufferData141_MAGNITUDE_16S, 48, gScenarioKernelData141_MAGNITUDE_16S, 4},
{15, 5, 900, 0, 0, gScenarioBufferData142_MAGNITUDE_16S, 48, gScenarioKernelData142_MAGNITUDE_16S, 4},
{16, 5, 960, 0, 0, gScenarioBufferData143_MAGNITUDE_16S, 48, gScenarioKernelData143_MAGNITUDE_16S, 4},
{18, 5, 1080, 0, 0, gScenarioBufferData144_MAGNITUDE_16S, 48, gScenarioKernelData144_MAGNITUDE_16S, 4},
{20, 5, 1200, 0, 0, gScenarioBufferData145_MAGNITUDE_16S, 48, gScenarioKernelData145_MAGNITUDE_16S, 4},
{22, 5, 1320, 0, 0, gScenarioBufferData146_MAGNITUDE_16S, 48, gScenarioKernelData146_MAGNITUDE_16S, 4},
{24, 5, 1440, 0, 0, gScenarioBufferData147_MAGNITUDE_16S, 48, gScenarioKernelData147_MAGNITUDE_16S, 4},
{26, 5, 1560, 0, 0, gScenarioBufferData148_MAGNITUDE_16S, 48, gScenarioKernelData148_MAGNITUDE_16S, 4},
{28, 5, 1680, 0, 0, gScenarioBufferData149_MAGNITUDE_16S, 48, gScenarioKernelData149_MAGNITUDE_16S, 4},
{30, 5, 1800, 0, 0, gScenarioBufferData150_MAGNITUDE_16S, 48, gScenarioKernelData150_MAGNITUDE_16S, 4},
{32, 5, 1920, 0, 0, gScenarioBufferData151_MAGNITUDE_16S, 48, gScenarioKernelData151_MAGNITUDE_16S, 4},
{36, 5, 2160, 0, 0, gScenarioBufferData152_MAGNITUDE_16S, 48, gScenarioKernelData152_MAGNITUDE_16S, 4},
{40, 5, 2400, 0, 0, gScenarioBufferData153_MAGNITUDE_16S, 48, gScenarioKernelData153_MAGNITUDE_16S, 4},
{44, 5, 2640, 0, 0, gScenarioBufferData154_MAGNITUDE_16S, 48, gScenarioKernelData154_MAGNITUDE_16S, 4},
{48, 5, 2880, 0, 0, gScenarioBufferData155_MAGNITUDE_16S, 48, gScenarioKernelData155_MAGNITUDE_16S, 4},
{52, 5, 3120, 0, 0, gScenarioBufferData156_MAGNITUDE_16S, 48, gScenarioKernelData156_MAGNITUDE_16S, 4},
{56, 5, 3360, 0, 0, gScenarioBufferData157_MAGNITUDE_16S, 48, gScenarioKernelData157_MAGNITUDE_16S, 4},
{60, 5, 3600, 0, 0, gScenarioBufferData158_MAGNITUDE_16S, 48, gScenarioKernelData158_MAGNITUDE_16S, 4},
{64, 5, 3840, 0, 0, gScenarioBufferData159_MAGNITUDE_16S, 48, gScenarioKernelData159_MAGNITUDE_16S, 4},
{1, 6, 72, 0, 0, gScenarioBufferData160_MAGNITUDE_16S, 48, gScenarioKernelData160_MAGNITUDE_16S, 4},
{2, 6, 144, 0, 0, gScenarioBufferData161_MAGNITUDE_16S, 48, gScenarioKernelData161_MAGNITUDE_16S, 4},
{3, 6, 216, 0, 0, gScenarioBufferData162_MAGNITUDE_16S, 48, gScenarioKernelData162_MAGNITUDE_16S, 4},
{4, 6, 288, 0, 0, gScenarioBufferData163_MAGNITUDE_16S, 48, gScenarioKernelData163_MAGNITUDE_16S, 4},
{5, 6, 360, 0, 0, gScenarioBufferData164_MAGNITUDE_16S, 48, gScenarioKernelData164_MAGNITUDE_16S, 4},
{6, 6, 432, 0, 0, gScenarioBufferData165_MAGNITUDE_16S, 48, gScenarioKernelData165_MAGNITUDE_16S, 4},
{7, 6, 504, 0, 0, gScenarioBufferData166_MAGNITUDE_16S, 48, gScenarioKernelData166_MAGNITUDE_16S, 4},
{8, 6, 576, 0, 0, gScenarioBufferData167_MAGNITUDE_16S, 48, gScenarioKernelData167_MAGNITUDE_16S, 4},
{9, 6, 648, 0, 0, gScenarioBufferData168_MAGNITUDE_16S, 48, gScenarioKernelData168_MAGNITUDE_16S, 4},
{10, 6, 720, 0, 0, gScenarioBufferData169_MAGNITUDE_16S, 48, gScenarioKernelData169_MAGNITUDE_16S, 4},
{11, 6, 792, 0, 0, gScenarioBufferData170_MAGNITUDE_16S, 48, gScenarioKernelData170_MAGNITUDE_16S, 4},
{12, 6, 864, 0, 0, gScenarioBufferData171_MAGNITUDE_16S, 48, gScenarioKernelData171_MAGNITUDE_16S, 4},
{13, 6, 936, 0, 0, gScenarioBufferData172_MAGNITUDE_16S, 48, gScenarioKernelData172_MAGNITUDE_16S, 4},
{14, 6, 1008, 0, 0, gScenarioBufferData173_MAGNITUDE_16S, 48, gScenarioKernelData173_MAGNITUDE_16S, 4},
{15, 6, 1080, 0, 0, gScenarioBufferData174_MAGNITUDE_16S, 48, gScenarioKernelData174_MAGNITUDE_16S, 4},
{16, 6, 1152, 0, 0, gScenarioBufferData175_MAGNITUDE_16S, 48, gScenarioKernelData175_MAGNITUDE_16S, 4},
{18, 6, 1296, 0, 0, gScenarioBufferData176_MAGNITUDE_16S, 48, gScenarioKernelData176_MAGNITUDE_16S, 4},
{20, 6, 1440, 0, 0, gScenarioBufferData177_MAGNITUDE_16S, 48, gScenarioKernelData177_MAGNITUDE_16S, 4},
{22, 6, 1584, 0, 0, gScenarioBufferData178_MAGNITUDE_16S, 48, gScenarioKernelData178_MAGNITUDE_16S, 4},
{24, 6, 1728, 0, 0, gScenarioBufferData179_MAGNITUDE_16S, 48, gScenarioKernelData179_MAGNITUDE_16S, 4},
{26, 6, 1872, 0, 0, gScenarioBufferData180_MAGNITUDE_16S, 48, gScenarioKernelData180_MAGNITUDE_16S, 4},
{28, 6, 2016, 0, 0, gScenarioBufferData181_MAGNITUDE_16S, 48, gScenarioKernelData181_MAGNITUDE_16S, 4},
{30, 6, 2160, 0, 0, gScenarioBufferData182_MAGNITUDE_16S, 48, gScenarioKernelData182_MAGNITUDE_16S, 4},
{32, 6, 2304, 0, 0, gScenarioBufferData183_MAGNITUDE_16S, 48, gScenarioKernelData183_MAGNITUDE_16S, 4},
{36, 6, 2592, 0, 0, gScenarioBufferData184_MAGNITUDE_16S, 48, gScenarioKernelData184_MAGNITUDE_16S, 4},
{40, 6, 2880, 0, 0, gScenarioBufferData185_MAGNITUDE_16S, 48, gScenarioKernelData185_MAGNITUDE_16S, 4},
{44, 6, 3168, 0, 0, gScenarioBufferData186_MAGNITUDE_16S, 48, gScenarioKernelData186_MAGNITUDE_16S, 4},
{48, 6, 3456, 0, 0, gScenarioBufferData187_MAGNITUDE_16S, 48, gScenarioKernelData187_MAGNITUDE_16S, 4},
{52, 6, 3744, 0, 0, gScenarioBufferData188_MAGNITUDE_16S, 48, gScenarioKernelData188_MAGNITUDE_16S, 4},
{1, 8, 96, 0, 0, gScenarioBufferData189_MAGNITUDE_16S, 48, gScenarioKernelData189_MAGNITUDE_16S, 4},
{2, 8, 192, 0, 0, gScenarioBufferData190_MAGNITUDE_16S, 48, gScenarioKernelData190_MAGNITUDE_16S, 4},
{3, 8, 288, 0, 0, gScenarioBufferData191_MAGNITUDE_16S, 48, gScenarioKernelData191_MAGNITUDE_16S, 4},
{4, 8, 384, 0, 0, gScenarioBufferData192_MAGNITUDE_16S, 48, gScenarioKernelData192_MAGNITUDE_16S, 4},
{5, 8, 480, 0, 0, gScenarioBufferData193_MAGNITUDE_16S, 48, gScenarioKernelData193_MAGNITUDE_16S, 4},
{6, 8, 576, 0, 0, gScenarioBufferData194_MAGNITUDE_16S, 48, gScenarioKernelData194_MAGNITUDE_16S, 4},
{7, 8, 672, 0, 0, gScenarioBufferData195_MAGNITUDE_16S, 48, gScenarioKernelData195_MAGNITUDE_16S, 4},
{8, 8, 768, 0, 0, gScenarioBufferData196_MAGNITUDE_16S, 48, gScenarioKernelData196_MAGNITUDE_16S, 4},
{9, 8, 864, 0, 0, gScenarioBufferData197_MAGNITUDE_16S, 48, gScenarioKernelData197_MAGNITUDE_16S, 4},
{10, 8, 960, 0, 0, gScenarioBufferData198_MAGNITUDE_16S, 48, gScenarioKernelData198_MAGNITUDE_16S, 4},
{11, 8, 1056, 0, 0, gScenarioBufferData199_MAGNITUDE_16S, 48, gScenarioKernelData199_MAGNITUDE_16S, 4},
{12, 8, 1152, 0, 0, gScenarioBufferData200_MAGNITUDE_16S, 48, gScenarioKernelData200_MAGNITUDE_16S, 4},
{13, 8, 1248, 0, 0, gScenarioBufferData201_MAGNITUDE_16S, 48, gScenarioKernelData201_MAGNITUDE_16S, 4},
{14, 8, 1344, 0, 0, gScenarioBufferData202_MAGNITUDE_16S, 48, gScenarioKernelData202_MAGNITUDE_16S, 4},
{15, 8, 1440, 0, 0, gScenarioBufferData203_MAGNITUDE_16S, 48, gScenarioKernelData203_MAGNITUDE_16S, 4},
{16, 8, 1536, 0, 0, gScenarioBufferData204_MAGNITUDE_16S, 48, gScenarioKernelData204_MAGNITUDE_16S, 4},
{18, 8, 1728, 0, 0, gScenarioBufferData205_MAGNITUDE_16S, 48, gScenarioKernelData205_MAGNITUDE_16S, 4},
{20, 8, 1920, 0, 0, gScenarioBufferData206_MAGNITUDE_16S, 48, gScenarioKernelData206_MAGNITUDE_16S, 4},
{22, 8, 2112, 0, 0, gScenarioBufferData207_MAGNITUDE_16S, 48, gScenarioKernelData207_MAGNITUDE_16S, 4},
{24, 8, 2304, 0, 0, gScenarioBufferData208_MAGNITUDE_16S, 48, gScenarioKernelData208_MAGNITUDE_16S, 4},
{26, 8, 2496, 0, 0, gScenarioBufferData209_MAGNITUDE_16S, 48, gScenarioKernelData209_MAGNITUDE_16S, 4},
{28, 8, 2688, 0, 0, gScenarioBufferData210_MAGNITUDE_16S, 48, gScenarioKernelData210_MAGNITUDE_16S, 4},
{30, 8, 2880, 0, 0, gScenarioBufferData211_MAGNITUDE_16S, 48, gScenarioKernelData211_MAGNITUDE_16S, 4},
{32, 8, 3072, 0, 0, gScenarioBufferData212_MAGNITUDE_16S, 48, gScenarioKernelData212_MAGNITUDE_16S, 4},
{36, 8, 3456, 0, 0, gScenarioBufferData213_MAGNITUDE_16S, 48, gScenarioKernelData213_MAGNITUDE_16S, 4},
{40, 8, 3840, 0, 0, gScenarioBufferData214_MAGNITUDE_16S, 48, gScenarioKernelData214_MAGNITUDE_16S, 4},
{1, 10, 120, 0, 0, gScenarioBufferData215_MAGNITUDE_16S, 48, gScenarioKernelData215_MAGNITUDE_16S, 4},
{2, 10, 240, 0, 0, gScenarioBufferData216_MAGNITUDE_16S, 48, gScenarioKernelData216_MAGNITUDE_16S, 4},
{3, 10, 360, 0, 0, gScenarioBufferData217_MAGNITUDE_16S, 48, gScenarioKernelData217_MAGNITUDE_16S, 4},
{4, 10, 480, 0, 0, gScenarioBufferData218_MAGNITUDE_16S, 48, gScenarioKernelData218_MAGNITUDE_16S, 4},
{5, 10, 600, 0, 0, gScenarioBufferData219_MAGNITUDE_16S, 48, gScenarioKernelData219_MAGNITUDE_16S, 4},
{6, 10, 720, 0, 0, gScenarioBufferData220_MAGNITUDE_16S, 48, gScenarioKernelData220_MAGNITUDE_16S, 4},
{7, 10, 840, 0, 0, gScenarioBufferData221_MAGNITUDE_16S, 48, gScenarioKernelData221_MAGNITUDE_16S, 4},
{8, 10, 960, 0, 0, gScenarioBufferData222_MAGNITUDE_16S, 48, gScenarioKernelData222_MAGNITUDE_16S, 4},
{9, 10, 1080, 0, 0, gScenarioBufferData223_MAGNITUDE_16S, 48, gScenarioKernelData223_MAGNITUDE_16S, 4},
{10, 10, 1200, 0, 0, gScenarioBufferData224_MAGNITUDE_16S, 48, gScenarioKernelData224_MAGNITUDE_16S, 4},
{11, 10, 1320, 0, 0, gScenarioBufferData225_MAGNITUDE_16S, 48, gScenarioKernelData225_MAGNITUDE_16S, 4},
{12, 10, 1440, 0, 0, gScenarioBufferData226_MAGNITUDE_16S, 48, gScenarioKernelData226_MAGNITUDE_16S, 4},
{13, 10, 1560, 0, 0, gScenarioBufferData227_MAGNITUDE_16S, 48, gScenarioKernelData227_MAGNITUDE_16S, 4},
{14, 10, 1680, 0, 0, gScenarioBufferData228_MAGNITUDE_16S, 48, gScenarioKernelData228_MAGNITUDE_16S, 4},
{15, 10, 1800, 0, 0, gScenarioBufferData229_MAGNITUDE_16S, 48, gScenarioKernelData229_MAGNITUDE_16S, 4},
{16, 10, 1920, 0, 0, gScenarioBufferData230_MAGNITUDE_16S, 48, gScenarioKernelData230_MAGNITUDE_16S, 4},
{18, 10, 2160, 0, 0, gScenarioBufferData231_MAGNITUDE_16S, 48, gScenarioKernelData231_MAGNITUDE_16S, 4},
{20, 10, 2400, 0, 0, gScenarioBufferData232_MAGNITUDE_16S, 48, gScenarioKernelData232_MAGNITUDE_16S, 4},
{22, 10, 2640, 0, 0, gScenarioBufferData233_MAGNITUDE_16S, 48, gScenarioKernelData233_MAGNITUDE_16S, 4},
{24, 10, 2880, 0, 0, gScenarioBufferData234_MAGNITUDE_16S, 48, gScenarioKernelData234_MAGNITUDE_16S, 4},
{26, 10, 3120, 0, 0, gScenarioBufferData235_MAGNITUDE_16S, 48, gScenarioKernelData235_MAGNITUDE_16S, 4},
{28, 10, 3360, 0, 0, gScenarioBufferData236_MAGNITUDE_16S, 48, gScenarioKernelData236_MAGNITUDE_16S, 4},
{30, 10, 3600, 0, 0, gScenarioBufferData237_MAGNITUDE_16S, 48, gScenarioKernelData237_MAGNITUDE_16S, 4},
{32, 10, 3840, 0, 0, gScenarioBufferData238_MAGNITUDE_16S, 48, gScenarioKernelData238_MAGNITUDE_16S, 4},
{1, 12, 144, 0, 0, gScenarioBufferData239_MAGNITUDE_16S, 48, gScenarioKernelData239_MAGNITUDE_16S, 4},
{2, 12, 288, 0, 0, gScenarioBufferData240_MAGNITUDE_16S, 48, gScenarioKernelData240_MAGNITUDE_16S, 4},
{3, 12, 432, 0, 0, gScenarioBufferData241_MAGNITUDE_16S, 48, gScenarioKernelData241_MAGNITUDE_16S, 4},
{4, 12, 576, 0, 0, gScenarioBufferData242_MAGNITUDE_16S, 48, gScenarioKernelData242_MAGNITUDE_16S, 4},
{5, 12, 720, 0, 0, gScenarioBufferData243_MAGNITUDE_16S, 48, gScenarioKernelData243_MAGNITUDE_16S, 4},
{6, 12, 864, 0, 0, gScenarioBufferData244_MAGNITUDE_16S, 48, gScenarioKernelData244_MAGNITUDE_16S, 4},
{7, 12, 1008, 0, 0, gScenarioBufferData245_MAGNITUDE_16S, 48, gScenarioKernelData245_MAGNITUDE_16S, 4},
{8, 12, 1152, 0, 0, gScenarioBufferData246_MAGNITUDE_16S, 48, gScenarioKernelData246_MAGNITUDE_16S, 4},
{9, 12, 1296, 0, 0, gScenarioBufferData247_MAGNITUDE_16S, 48, gScenarioKernelData247_MAGNITUDE_16S, 4},
{10, 12, 1440, 0, 0, gScenarioBufferData248_MAGNITUDE_16S, 48, gScenarioKernelData248_MAGNITUDE_16S, 4},
{11, 12, 1584, 0, 0, gScenarioBufferData249_MAGNITUDE_16S, 48, gScenarioKernelData249_MAGNITUDE_16S, 4},
{12, 12, 1728, 0, 0, gScenarioBufferData250_MAGNITUDE_16S, 48, gScenarioKernelData250_MAGNITUDE_16S, 4},
{13, 12, 1872, 0, 0, gScenarioBufferData251_MAGNITUDE_16S, 48, gScenarioKernelData251_MAGNITUDE_16S, 4},
{14, 12, 2016, 0, 0, gScenarioBufferData252_MAGNITUDE_16S, 48, gScenarioKernelData252_MAGNITUDE_16S, 4},
{15, 12, 2160, 0, 0, gScenarioBufferData253_MAGNITUDE_16S, 48, gScenarioKernelData253_MAGNITUDE_16S, 4},
{16, 12, 2304, 0, 0, gScenarioBufferData254_MAGNITUDE_16S, 48, gScenarioKernelData254_MAGNITUDE_16S, 4},
{18, 12, 2592, 0, 0, gScenarioBufferData255_MAGNITUDE_16S, 48, gScenarioKernelData255_MAGNITUDE_16S, 4},
{20, 12, 2880, 0, 0, gScenarioBufferData256_MAGNITUDE_16S, 48, gScenarioKernelData256_MAGNITUDE_16S, 4},
{22, 12, 3168, 0, 0, gScenarioBufferData257_MAGNITUDE_16S, 48, gScenarioKernelData257_MAGNITUDE_16S, 4},
{24, 12, 3456, 0, 0, gScenarioBufferData258_MAGNITUDE_16S, 48, gScenarioKernelData258_MAGNITUDE_16S, 4},
{26, 12, 3744, 0, 0, gScenarioBufferData259_MAGNITUDE_16S, 48, gScenarioKernelData259_MAGNITUDE_16S, 4},
{1, 14, 168, 0, 0, gScenarioBufferData260_MAGNITUDE_16S, 48, gScenarioKernelData260_MAGNITUDE_16S, 4},
{2, 14, 336, 0, 0, gScenarioBufferData261_MAGNITUDE_16S, 48, gScenarioKernelData261_MAGNITUDE_16S, 4},
{3, 14, 504, 0, 0, gScenarioBufferData262_MAGNITUDE_16S, 48, gScenarioKernelData262_MAGNITUDE_16S, 4},
{4, 14, 672, 0, 0, gScenarioBufferData263_MAGNITUDE_16S, 48, gScenarioKernelData263_MAGNITUDE_16S, 4},
{5, 14, 840, 0, 0, gScenarioBufferData264_MAGNITUDE_16S, 48, gScenarioKernelData264_MAGNITUDE_16S, 4},
{6, 14, 1008, 0, 0, gScenarioBufferData265_MAGNITUDE_16S, 48, gScenarioKernelData265_MAGNITUDE_16S, 4},
{7, 14, 1176, 0, 0, gScenarioBufferData266_MAGNITUDE_16S, 48, gScenarioKernelData266_MAGNITUDE_16S, 4},
{8, 14, 1344, 0, 0, gScenarioBufferData267_MAGNITUDE_16S, 48, gScenarioKernelData267_MAGNITUDE_16S, 4},
{9, 14, 1512, 0, 0, gScenarioBufferData268_MAGNITUDE_16S, 48, gScenarioKernelData268_MAGNITUDE_16S, 4},
{10, 14, 1680, 0, 0, gScenarioBufferData269_MAGNITUDE_16S, 48, gScenarioKernelData269_MAGNITUDE_16S, 4},
{11, 14, 1848, 0, 0, gScenarioBufferData270_MAGNITUDE_16S, 48, gScenarioKernelData270_MAGNITUDE_16S, 4},
{12, 14, 2016, 0, 0, gScenarioBufferData271_MAGNITUDE_16S, 48, gScenarioKernelData271_MAGNITUDE_16S, 4},
{13, 14, 2184, 0, 0, gScenarioBufferData272_MAGNITUDE_16S, 48, gScenarioKernelData272_MAGNITUDE_16S, 4},
{14, 14, 2352, 0, 0, gScenarioBufferData273_MAGNITUDE_16S, 48, gScenarioKernelData273_MAGNITUDE_16S, 4},
{15, 14, 2520, 0, 0, gScenarioBufferData274_MAGNITUDE_16S, 48, gScenarioKernelData274_MAGNITUDE_16S, 4},
{16, 14, 2688, 0, 0, gScenarioBufferData275_MAGNITUDE_16S, 48, gScenarioKernelData275_MAGNITUDE_16S, 4},
{18, 14, 3024, 0, 0, gScenarioBufferData276_MAGNITUDE_16S, 48, gScenarioKernelData276_MAGNITUDE_16S, 4},
{20, 14, 3360, 0, 0, gScenarioBufferData277_MAGNITUDE_16S, 48, gScenarioKernelData277_MAGNITUDE_16S, 4},
{22, 14, 3696, 0, 0, gScenarioBufferData278_MAGNITUDE_16S, 48, gScenarioKernelData278_MAGNITUDE_16S, 4},
{1, 16, 192, 0, 0, gScenarioBufferData279_MAGNITUDE_16S, 48, gScenarioKernelData279_MAGNITUDE_16S, 4},
{2, 16, 384, 0, 0, gScenarioBufferData280_MAGNITUDE_16S, 48, gScenarioKernelData280_MAGNITUDE_16S, 4},
{3, 16, 576, 0, 0, gScenarioBufferData281_MAGNITUDE_16S, 48, gScenarioKernelData281_MAGNITUDE_16S, 4},
{4, 16, 768, 0, 0, gScenarioBufferData282_MAGNITUDE_16S, 48, gScenarioKernelData282_MAGNITUDE_16S, 4},
{5, 16, 960, 0, 0, gScenarioBufferData283_MAGNITUDE_16S, 48, gScenarioKernelData283_MAGNITUDE_16S, 4},
{6, 16, 1152, 0, 0, gScenarioBufferData284_MAGNITUDE_16S, 48, gScenarioKernelData284_MAGNITUDE_16S, 4},
{7, 16, 1344, 0, 0, gScenarioBufferData285_MAGNITUDE_16S, 48, gScenarioKernelData285_MAGNITUDE_16S, 4},
{8, 16, 1536, 0, 0, gScenarioBufferData286_MAGNITUDE_16S, 48, gScenarioKernelData286_MAGNITUDE_16S, 4},
{9, 16, 1728, 0, 0, gScenarioBufferData287_MAGNITUDE_16S, 48, gScenarioKernelData287_MAGNITUDE_16S, 4},
{10, 16, 1920, 0, 0, gScenarioBufferData288_MAGNITUDE_16S, 48, gScenarioKernelData288_MAGNITUDE_16S, 4},
{11, 16, 2112, 0, 0, gScenarioBufferData289_MAGNITUDE_16S, 48, gScenarioKernelData289_MAGNITUDE_16S, 4},
{12, 16, 2304, 0, 0, gScenarioBufferData290_MAGNITUDE_16S, 48, gScenarioKernelData290_MAGNITUDE_16S, 4},
{13, 16, 2496, 0, 0, gScenarioBufferData291_MAGNITUDE_16S, 48, gScenarioKernelData291_MAGNITUDE_16S, 4},
{14, 16, 2688, 0, 0, gScenarioBufferData292_MAGNITUDE_16S, 48, gScenarioKernelData292_MAGNITUDE_16S, 4},
{15, 16, 2880, 0, 0, gScenarioBufferData293_MAGNITUDE_16S, 48, gScenarioKernelData293_MAGNITUDE_16S, 4},
{16, 16, 3072, 0, 0, gScenarioBufferData294_MAGNITUDE_16S, 48, gScenarioKernelData294_MAGNITUDE_16S, 4},
{18, 16, 3456, 0, 0, gScenarioBufferData295_MAGNITUDE_16S, 48, gScenarioKernelData295_MAGNITUDE_16S, 4},
{20, 16, 3840, 0, 0, gScenarioBufferData296_MAGNITUDE_16S, 48, gScenarioKernelData296_MAGNITUDE_16S, 4},
{1, 18, 216, 0, 0, gScenarioBufferData297_MAGNITUDE_16S, 48, gScenarioKernelData297_MAGNITUDE_16S, 4},
{2, 18, 432, 0, 0, gScenarioBufferData298_MAGNITUDE_16S, 48, gScenarioKernelData298_MAGNITUDE_16S, 4},
{3, 18, 648, 0, 0, gScenarioBufferData299_MAGNITUDE_16S, 48, gScenarioKernelData299_MAGNITUDE_16S, 4},
{4, 18, 864, 0, 0, gScenarioBufferData300_MAGNITUDE_16S, 48, gScenarioKernelData300_MAGNITUDE_16S, 4},
{5, 18, 1080, 0, 0, gScenarioBufferData301_MAGNITUDE_16S, 48, gScenarioKernelData301_MAGNITUDE_16S, 4},
{6, 18, 1296, 0, 0, gScenarioBufferData302_MAGNITUDE_16S, 48, gScenarioKernelData302_MAGNITUDE_16S, 4},
{7, 18, 1512, 0, 0, gScenarioBufferData303_MAGNITUDE_16S, 48, gScenarioKernelData303_MAGNITUDE_16S, 4},
{8, 18, 1728, 0, 0, gScenarioBufferData304_MAGNITUDE_16S, 48, gScenarioKernelData304_MAGNITUDE_16S, 4},
{9, 18, 1944, 0, 0, gScenarioBufferData305_MAGNITUDE_16S, 48, gScenarioKernelData305_MAGNITUDE_16S, 4},
{10, 18, 2160, 0, 0, gScenarioBufferData306_MAGNITUDE_16S, 48, gScenarioKernelData306_MAGNITUDE_16S, 4},
{11, 18, 2376, 0, 0, gScenarioBufferData307_MAGNITUDE_16S, 48, gScenarioKernelData307_MAGNITUDE_16S, 4},
{12, 18, 2592, 0, 0, gScenarioBufferData308_MAGNITUDE_16S, 48, gScenarioKernelData308_MAGNITUDE_16S, 4},
{13, 18, 2808, 0, 0, gScenarioBufferData309_MAGNITUDE_16S, 48, gScenarioKernelData309_MAGNITUDE_16S, 4},
{14, 18, 3024, 0, 0, gScenarioBufferData310_MAGNITUDE_16S, 48, gScenarioKernelData310_MAGNITUDE_16S, 4},
{15, 18, 3240, 0, 0, gScenarioBufferData311_MAGNITUDE_16S, 48, gScenarioKernelData311_MAGNITUDE_16S, 4},
{16, 18, 3456, 0, 0, gScenarioBufferData312_MAGNITUDE_16S, 48, gScenarioKernelData312_MAGNITUDE_16S, 4},
{1, 20, 240, 0, 0, gScenarioBufferData313_MAGNITUDE_16S, 48, gScenarioKernelData313_MAGNITUDE_16S, 4},
{2, 20, 480, 0, 0, gScenarioBufferData314_MAGNITUDE_16S, 48, gScenarioKernelData314_MAGNITUDE_16S, 4},
{3, 20, 720, 0, 0, gScenarioBufferData315_MAGNITUDE_16S, 48, gScenarioKernelData315_MAGNITUDE_16S, 4},
{4, 20, 960, 0, 0, gScenarioBufferData316_MAGNITUDE_16S, 48, gScenarioKernelData316_MAGNITUDE_16S, 4},
{5, 20, 1200, 0, 0, gScenarioBufferData317_MAGNITUDE_16S, 48, gScenarioKernelData317_MAGNITUDE_16S, 4},
{6, 20, 1440, 0, 0, gScenarioBufferData318_MAGNITUDE_16S, 48, gScenarioKernelData318_MAGNITUDE_16S, 4},
{7, 20, 1680, 0, 0, gScenarioBufferData319_MAGNITUDE_16S, 48, gScenarioKernelData319_MAGNITUDE_16S, 4},
{8, 20, 1920, 0, 0, gScenarioBufferData320_MAGNITUDE_16S, 48, gScenarioKernelData320_MAGNITUDE_16S, 4},
{9, 20, 2160, 0, 0, gScenarioBufferData321_MAGNITUDE_16S, 48, gScenarioKernelData321_MAGNITUDE_16S, 4},
{10, 20, 2400, 0, 0, gScenarioBufferData322_MAGNITUDE_16S, 48, gScenarioKernelData322_MAGNITUDE_16S, 4},
{11, 20, 2640, 0, 0, gScenarioBufferData323_MAGNITUDE_16S, 48, gScenarioKernelData323_MAGNITUDE_16S, 4},
{12, 20, 2880, 0, 0, gScenarioBufferData324_MAGNITUDE_16S, 48, gScenarioKernelData324_MAGNITUDE_16S, 4},
{13, 20, 3120, 0, 0, gScenarioBufferData325_MAGNITUDE_16S, 48, gScenarioKernelData325_MAGNITUDE_16S, 4},
{14, 20, 3360, 0, 0, gScenarioBufferData326_MAGNITUDE_16S, 48, gScenarioKernelData326_MAGNITUDE_16S, 4},
{15, 20, 3600, 0, 0, gScenarioBufferData327_MAGNITUDE_16S, 48, gScenarioKernelData327_MAGNITUDE_16S, 4},
{16, 20, 3840, 0, 0, gScenarioBufferData328_MAGNITUDE_16S, 48, gScenarioKernelData328_MAGNITUDE_16S, 4},
{1, 22, 264, 0, 0, gScenarioBufferData329_MAGNITUDE_16S, 48, gScenarioKernelData329_MAGNITUDE_16S, 4},
{2, 22, 528, 0, 0, gScenarioBufferData330_MAGNITUDE_16S, 48, gScenarioKernelData330_MAGNITUDE_16S, 4},
{3, 22, 792, 0, 0, gScenarioBufferData331_MAGNITUDE_16S, 48, gScenarioKernelData331_MAGNITUDE_16S, 4},
{4, 22, 1056, 0, 0, gScenarioBufferData332_MAGNITUDE_16S, 48, gScenarioKernelData332_MAGNITUDE_16S, 4},
{5, 22, 1320, 0, 0, gScenarioBufferData333_MAGNITUDE_16S, 48, gScenarioKernelData333_MAGNITUDE_16S, 4},
{6, 22, 1584, 0, 0, gScenarioBufferData334_MAGNITUDE_16S, 48, gScenarioKernelData334_MAGNITUDE_16S, 4},
{7, 22, 1848, 0, 0, gScenarioBufferData335_MAGNITUDE_16S, 48, gScenarioKernelData335_MAGNITUDE_16S, 4},
{8, 22, 2112, 0, 0, gScenarioBufferData336_MAGNITUDE_16S, 48, gScenarioKernelData336_MAGNITUDE_16S, 4},
{9, 22, 2376, 0, 0, gScenarioBufferData337_MAGNITUDE_16S, 48, gScenarioKernelData337_MAGNITUDE_16S, 4},
{10, 22, 2640, 0, 0, gScenarioBufferData338_MAGNITUDE_16S, 48, gScenarioKernelData338_MAGNITUDE_16S, 4},
{11, 22, 2904, 0, 0, gScenarioBufferData339_MAGNITUDE_16S, 48, gScenarioKernelData339_MAGNITUDE_16S, 4},
{12, 22, 3168, 0, 0, gScenarioBufferData340_MAGNITUDE_16S, 48, gScenarioKernelData340_MAGNITUDE_16S, 4},
{13, 22, 3432, 0, 0, gScenarioBufferData341_MAGNITUDE_16S, 48, gScenarioKernelData341_MAGNITUDE_16S, 4},
{14, 22, 3696, 0, 0, gScenarioBufferData342_MAGNITUDE_16S, 48, gScenarioKernelData342_MAGNITUDE_16S, 4},
{1, 24, 288, 0, 0, gScenarioBufferData343_MAGNITUDE_16S, 48, gScenarioKernelData343_MAGNITUDE_16S, 4},
{2, 24, 576, 0, 0, gScenarioBufferData344_MAGNITUDE_16S, 48, gScenarioKernelData344_MAGNITUDE_16S, 4},
{3, 24, 864, 0, 0, gScenarioBufferData345_MAGNITUDE_16S, 48, gScenarioKernelData345_MAGNITUDE_16S, 4},
{4, 24, 1152, 0, 0, gScenarioBufferData346_MAGNITUDE_16S, 48, gScenarioKernelData346_MAGNITUDE_16S, 4},
{5, 24, 1440, 0, 0, gScenarioBufferData347_MAGNITUDE_16S, 48, gScenarioKernelData347_MAGNITUDE_16S, 4},
{6, 24, 1728, 0, 0, gScenarioBufferData348_MAGNITUDE_16S, 48, gScenarioKernelData348_MAGNITUDE_16S, 4},
{7, 24, 2016, 0, 0, gScenarioBufferData349_MAGNITUDE_16S, 48, gScenarioKernelData349_MAGNITUDE_16S, 4},
{8, 24, 2304, 0, 0, gScenarioBufferData350_MAGNITUDE_16S, 48, gScenarioKernelData350_MAGNITUDE_16S, 4},
{9, 24, 2592, 0, 0, gScenarioBufferData351_MAGNITUDE_16S, 48, gScenarioKernelData351_MAGNITUDE_16S, 4},
{10, 24, 2880, 0, 0, gScenarioBufferData352_MAGNITUDE_16S, 48, gScenarioKernelData352_MAGNITUDE_16S, 4},
{11, 24, 3168, 0, 0, gScenarioBufferData353_MAGNITUDE_16S, 48, gScenarioKernelData353_MAGNITUDE_16S, 4},
{12, 24, 3456, 0, 0, gScenarioBufferData354_MAGNITUDE_16S, 48, gScenarioKernelData354_MAGNITUDE_16S, 4},
{13, 24, 3744, 0, 0, gScenarioBufferData355_MAGNITUDE_16S, 48, gScenarioKernelData355_MAGNITUDE_16S, 4},
{1, 26, 312, 0, 0, gScenarioBufferData356_MAGNITUDE_16S, 48, gScenarioKernelData356_MAGNITUDE_16S, 4},
{2, 26, 624, 0, 0, gScenarioBufferData357_MAGNITUDE_16S, 48, gScenarioKernelData357_MAGNITUDE_16S, 4},
{3, 26, 936, 0, 0, gScenarioBufferData358_MAGNITUDE_16S, 48, gScenarioKernelData358_MAGNITUDE_16S, 4},
{4, 26, 1248, 0, 0, gScenarioBufferData359_MAGNITUDE_16S, 48, gScenarioKernelData359_MAGNITUDE_16S, 4},
{5, 26, 1560, 0, 0, gScenarioBufferData360_MAGNITUDE_16S, 48, gScenarioKernelData360_MAGNITUDE_16S, 4},
{6, 26, 1872, 0, 0, gScenarioBufferData361_MAGNITUDE_16S, 48, gScenarioKernelData361_MAGNITUDE_16S, 4},
{7, 26, 2184, 0, 0, gScenarioBufferData362_MAGNITUDE_16S, 48, gScenarioKernelData362_MAGNITUDE_16S, 4},
{8, 26, 2496, 0, 0, gScenarioBufferData363_MAGNITUDE_16S, 48, gScenarioKernelData363_MAGNITUDE_16S, 4},
{9, 26, 2808, 0, 0, gScenarioBufferData364_MAGNITUDE_16S, 48, gScenarioKernelData364_MAGNITUDE_16S, 4},
{10, 26, 3120, 0, 0, gScenarioBufferData365_MAGNITUDE_16S, 48, gScenarioKernelData365_MAGNITUDE_16S, 4},
{11, 26, 3432, 0, 0, gScenarioBufferData366_MAGNITUDE_16S, 48, gScenarioKernelData366_MAGNITUDE_16S, 4},
{12, 26, 3744, 0, 0, gScenarioBufferData367_MAGNITUDE_16S, 48, gScenarioKernelData367_MAGNITUDE_16S, 4},
{1, 28, 336, 0, 0, gScenarioBufferData368_MAGNITUDE_16S, 48, gScenarioKernelData368_MAGNITUDE_16S, 4},
{2, 28, 672, 0, 0, gScenarioBufferData369_MAGNITUDE_16S, 48, gScenarioKernelData369_MAGNITUDE_16S, 4},
{3, 28, 1008, 0, 0, gScenarioBufferData370_MAGNITUDE_16S, 48, gScenarioKernelData370_MAGNITUDE_16S, 4},
{4, 28, 1344, 0, 0, gScenarioBufferData371_MAGNITUDE_16S, 48, gScenarioKernelData371_MAGNITUDE_16S, 4},
{5, 28, 1680, 0, 0, gScenarioBufferData372_MAGNITUDE_16S, 48, gScenarioKernelData372_MAGNITUDE_16S, 4},
{6, 28, 2016, 0, 0, gScenarioBufferData373_MAGNITUDE_16S, 48, gScenarioKernelData373_MAGNITUDE_16S, 4},
{7, 28, 2352, 0, 0, gScenarioBufferData374_MAGNITUDE_16S, 48, gScenarioKernelData374_MAGNITUDE_16S, 4},
{8, 28, 2688, 0, 0, gScenarioBufferData375_MAGNITUDE_16S, 48, gScenarioKernelData375_MAGNITUDE_16S, 4},
{9, 28, 3024, 0, 0, gScenarioBufferData376_MAGNITUDE_16S, 48, gScenarioKernelData376_MAGNITUDE_16S, 4},
{10, 28, 3360, 0, 0, gScenarioBufferData377_MAGNITUDE_16S, 48, gScenarioKernelData377_MAGNITUDE_16S, 4},
{11, 28, 3696, 0, 0, gScenarioBufferData378_MAGNITUDE_16S, 48, gScenarioKernelData378_MAGNITUDE_16S, 4},
{1, 30, 360, 0, 0, gScenarioBufferData379_MAGNITUDE_16S, 48, gScenarioKernelData379_MAGNITUDE_16S, 4},
{2, 30, 720, 0, 0, gScenarioBufferData380_MAGNITUDE_16S, 48, gScenarioKernelData380_MAGNITUDE_16S, 4},
{3, 30, 1080, 0, 0, gScenarioBufferData381_MAGNITUDE_16S, 48, gScenarioKernelData381_MAGNITUDE_16S, 4},
{4, 30, 1440, 0, 0, gScenarioBufferData382_MAGNITUDE_16S, 48, gScenarioKernelData382_MAGNITUDE_16S, 4},
{5, 30, 1800, 0, 0, gScenarioBufferData383_MAGNITUDE_16S, 48, gScenarioKernelData383_MAGNITUDE_16S, 4},
{6, 30, 2160, 0, 0, gScenarioBufferData384_MAGNITUDE_16S, 48, gScenarioKernelData384_MAGNITUDE_16S, 4},
{7, 30, 2520, 0, 0, gScenarioBufferData385_MAGNITUDE_16S, 48, gScenarioKernelData385_MAGNITUDE_16S, 4},
{8, 30, 2880, 0, 0, gScenarioBufferData386_MAGNITUDE_16S, 48, gScenarioKernelData386_MAGNITUDE_16S, 4},
{9, 30, 3240, 0, 0, gScenarioBufferData387_MAGNITUDE_16S, 48, gScenarioKernelData387_MAGNITUDE_16S, 4},
{10, 30, 3600, 0, 0, gScenarioBufferData388_MAGNITUDE_16S, 48, gScenarioKernelData388_MAGNITUDE_16S, 4},
{1, 32, 384, 0, 0, gScenarioBufferData389_MAGNITUDE_16S, 48, gScenarioKernelData389_MAGNITUDE_16S, 4},
{2, 32, 768, 0, 0, gScenarioBufferData390_MAGNITUDE_16S, 48, gScenarioKernelData390_MAGNITUDE_16S, 4},
{3, 32, 1152, 0, 0, gScenarioBufferData391_MAGNITUDE_16S, 48, gScenarioKernelData391_MAGNITUDE_16S, 4},
{4, 32, 1536, 0, 0, gScenarioBufferData392_MAGNITUDE_16S, 48, gScenarioKernelData392_MAGNITUDE_16S, 4},
{5, 32, 1920, 0, 0, gScenarioBufferData393_MAGNITUDE_16S, 48, gScenarioKernelData393_MAGNITUDE_16S, 4},
{6, 32, 2304, 0, 0, gScenarioBufferData394_MAGNITUDE_16S, 48, gScenarioKernelData394_MAGNITUDE_16S, 4},
{7, 32, 2688, 0, 0, gScenarioBufferData395_MAGNITUDE_16S, 48, gScenarioKernelData395_MAGNITUDE_16S, 4},
{8, 32, 3072, 0, 0, gScenarioBufferData396_MAGNITUDE_16S, 48, gScenarioKernelData396_MAGNITUDE_16S, 4},
{9, 32, 3456, 0, 0, gScenarioBufferData397_MAGNITUDE_16S, 48, gScenarioKernelData397_MAGNITUDE_16S, 4},
{10, 32, 3840, 0, 0, gScenarioBufferData398_MAGNITUDE_16S, 48, gScenarioKernelData398_MAGNITUDE_16S, 4}
};

static acf_scenario_list gScenarioList_MAGNITUDE_16S = {
399, //number of scenarios
gScenarioArray_MAGNITUDE_16S};
//**************************************************************

class MAGNITUDE_16S : public ACF_Process_APU
{

public:
   MAGNITUDE_16S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("MAGNITUDE_16S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("MAGNITUDE_16S",
                                        MAGNITUDE_16S_LOAD_SEGMENTS,
                                        MAGNITUDE_16S_LOAD_PMEM, MAGNITUDE_16S_LOAD_PMEM_SIZE,
                                        MAGNITUDE_16S_LOAD_DMEM, MAGNITUDE_16S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(MAGNITUDE_16S_APEX_LOG_BUFFER, MAGNITUDE_16S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_16S, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUT_1", icp::DATATYPE_16S, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_MAGNITUDE_16S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_MAGNITUDE_16S

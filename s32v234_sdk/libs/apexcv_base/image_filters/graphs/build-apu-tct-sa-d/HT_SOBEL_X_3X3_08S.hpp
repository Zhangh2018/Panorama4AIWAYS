#ifndef _ACF_PROCESS_APU_HT_SOBEL_X_3X3_08S
#define _ACF_PROCESS_APU_HT_SOBEL_X_3X3_08S

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HT_SOBEL_X_3X3_08S_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HT_SOBEL_X_3X3_08S[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 1, {0, 0, 0, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HT_SOBEL_X_3X3_08S[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 1, {0, 0, 0, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HT_SOBEL_X_3X3_08S[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 1, {0, 0, 0, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HT_SOBEL_X_3X3_08S[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 1, {0, 0, 0, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HT_SOBEL_X_3X3_08S[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 1, {0, 0, 0, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HT_SOBEL_X_3X3_08S[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 1, {0, 0, 0, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HT_SOBEL_X_3X3_08S[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 1, {0, 0, 0, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HT_SOBEL_X_3X3_08S[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 1, {0, 0, 0, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HT_SOBEL_X_3X3_08S[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 1, {0, 0, 0, 0}}, {36, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HT_SOBEL_X_3X3_08S[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 1, {0, 0, 0, 0}}, {40, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HT_SOBEL_X_3X3_08S[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 1, {0, 0, 0, 0}}, {44, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HT_SOBEL_X_3X3_08S[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 1, {0, 0, 0, 0}}, {48, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HT_SOBEL_X_3X3_08S[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 1, {0, 0, 0, 0}}, {52, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HT_SOBEL_X_3X3_08S[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 1, {0, 0, 0, 0}}, {56, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HT_SOBEL_X_3X3_08S[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 2, 1, {0, 0, 0, 0}}, {60, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HT_SOBEL_X_3X3_08S[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 2, 1, {0, 0, 0, 0}}, {64, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HT_SOBEL_X_3X3_08S[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HT_SOBEL_X_3X3_08S[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HT_SOBEL_X_3X3_08S[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HT_SOBEL_X_3X3_08S[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HT_SOBEL_X_3X3_08S[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HT_SOBEL_X_3X3_08S[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HT_SOBEL_X_3X3_08S[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HT_SOBEL_X_3X3_08S[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HT_SOBEL_X_3X3_08S[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HT_SOBEL_X_3X3_08S[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HT_SOBEL_X_3X3_08S[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HT_SOBEL_X_3X3_08S[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HT_SOBEL_X_3X3_08S[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HT_SOBEL_X_3X3_08S[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HT_SOBEL_X_3X3_08S[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HT_SOBEL_X_3X3_08S[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HT_SOBEL_X_3X3_08S[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 2, 1, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HT_SOBEL_X_3X3_08S[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 2, 1, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HT_SOBEL_X_3X3_08S[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 2, 1, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HT_SOBEL_X_3X3_08S[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 2, 1, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HT_SOBEL_X_3X3_08S[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 2, 1, {0, 0, 0, 0}}, {20, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_HT_SOBEL_X_3X3_08S[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 2, 1, {0, 0, 0, 0}}, {24, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_HT_SOBEL_X_3X3_08S[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 2, 1, {0, 0, 0, 0}}, {28, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_HT_SOBEL_X_3X3_08S[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 2, 1, {0, 0, 0, 0}}, {32, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_HT_SOBEL_X_3X3_08S[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 2, 1, {0, 0, 0, 0}}, {36, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_HT_SOBEL_X_3X3_08S[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 2, 1, {0, 0, 0, 0}}, {40, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_HT_SOBEL_X_3X3_08S[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 2, 1, {0, 0, 0, 0}}, {44, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_HT_SOBEL_X_3X3_08S[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 2, 1, {0, 0, 0, 0}}, {48, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_HT_SOBEL_X_3X3_08S[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 2, 1, {0, 0, 0, 0}}, {52, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_HT_SOBEL_X_3X3_08S[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 2, 1, {0, 0, 0, 0}}, {56, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_HT_SOBEL_X_3X3_08S[] = {{60, 3, 4, 0, {1, 1, 2, 2}}, {60, 3, 2, 1, {0, 0, 0, 0}}, {60, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_HT_SOBEL_X_3X3_08S[] = {{64, 3, 4, 0, {1, 1, 2, 2}}, {64, 3, 2, 1, {0, 0, 0, 0}}, {64, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_HT_SOBEL_X_3X3_08S[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_HT_SOBEL_X_3X3_08S[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_HT_SOBEL_X_3X3_08S[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_HT_SOBEL_X_3X3_08S[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_HT_SOBEL_X_3X3_08S[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_HT_SOBEL_X_3X3_08S[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_HT_SOBEL_X_3X3_08S[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_HT_SOBEL_X_3X3_08S[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_HT_SOBEL_X_3X3_08S[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_HT_SOBEL_X_3X3_08S[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_HT_SOBEL_X_3X3_08S[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_HT_SOBEL_X_3X3_08S[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_HT_SOBEL_X_3X3_08S[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_HT_SOBEL_X_3X3_08S[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 2, 1, {0, 0, 0, 0}}, {56, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_HT_SOBEL_X_3X3_08S[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 2, 1, {0, 0, 0, 0}}, {60, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_HT_SOBEL_X_3X3_08S[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 2, 1, {0, 0, 0, 0}}, {64, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_HT_SOBEL_X_3X3_08S[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 1, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_HT_SOBEL_X_3X3_08S[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 1, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_HT_SOBEL_X_3X3_08S[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 1, {0, 0, 0, 0}}, {12, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_HT_SOBEL_X_3X3_08S[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 2, 1, {0, 0, 0, 0}}, {16, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_HT_SOBEL_X_3X3_08S[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 2, 1, {0, 0, 0, 0}}, {20, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_HT_SOBEL_X_3X3_08S[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 2, 1, {0, 0, 0, 0}}, {24, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_HT_SOBEL_X_3X3_08S[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 2, 1, {0, 0, 0, 0}}, {28, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_HT_SOBEL_X_3X3_08S[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 2, 1, {0, 0, 0, 0}}, {32, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_HT_SOBEL_X_3X3_08S[] = {{36, 5, 4, 0, {1, 1, 2, 2}}, {36, 5, 2, 1, {0, 0, 0, 0}}, {36, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_HT_SOBEL_X_3X3_08S[] = {{40, 5, 4, 0, {1, 1, 2, 2}}, {40, 5, 2, 1, {0, 0, 0, 0}}, {40, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_HT_SOBEL_X_3X3_08S[] = {{44, 5, 4, 0, {1, 1, 2, 2}}, {44, 5, 2, 1, {0, 0, 0, 0}}, {44, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_HT_SOBEL_X_3X3_08S[] = {{48, 5, 4, 0, {1, 1, 2, 2}}, {48, 5, 2, 1, {0, 0, 0, 0}}, {48, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_HT_SOBEL_X_3X3_08S[] = {{52, 5, 4, 0, {1, 1, 2, 2}}, {52, 5, 2, 1, {0, 0, 0, 0}}, {52, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_HT_SOBEL_X_3X3_08S[] = {{56, 5, 4, 0, {1, 1, 2, 2}}, {56, 5, 2, 1, {0, 0, 0, 0}}, {56, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_HT_SOBEL_X_3X3_08S[] = {{60, 5, 4, 0, {1, 1, 2, 2}}, {60, 5, 2, 1, {0, 0, 0, 0}}, {60, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_HT_SOBEL_X_3X3_08S[] = {{64, 5, 4, 0, {1, 1, 2, 2}}, {64, 5, 2, 1, {0, 0, 0, 0}}, {64, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_HT_SOBEL_X_3X3_08S[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_HT_SOBEL_X_3X3_08S[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_HT_SOBEL_X_3X3_08S[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_HT_SOBEL_X_3X3_08S[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_HT_SOBEL_X_3X3_08S[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_HT_SOBEL_X_3X3_08S[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_HT_SOBEL_X_3X3_08S[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_HT_SOBEL_X_3X3_08S[] = {{32, 6, 4, 0, {1, 1, 2, 2}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_HT_SOBEL_X_3X3_08S[] = {{36, 6, 4, 0, {1, 1, 2, 2}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_HT_SOBEL_X_3X3_08S[] = {{40, 6, 4, 0, {1, 1, 2, 2}}, {40, 6, 2, 1, {0, 0, 0, 0}}, {40, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_HT_SOBEL_X_3X3_08S[] = {{44, 6, 4, 0, {1, 1, 2, 2}}, {44, 6, 2, 1, {0, 0, 0, 0}}, {44, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_HT_SOBEL_X_3X3_08S[] = {{48, 6, 4, 0, {1, 1, 2, 2}}, {48, 6, 2, 1, {0, 0, 0, 0}}, {48, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_HT_SOBEL_X_3X3_08S[] = {{52, 6, 4, 0, {1, 1, 2, 2}}, {52, 6, 2, 1, {0, 0, 0, 0}}, {52, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_HT_SOBEL_X_3X3_08S[] = {{56, 6, 4, 0, {1, 1, 2, 2}}, {56, 6, 2, 1, {0, 0, 0, 0}}, {56, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_HT_SOBEL_X_3X3_08S[] = {{60, 6, 4, 0, {1, 1, 2, 2}}, {60, 6, 2, 1, {0, 0, 0, 0}}, {60, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_HT_SOBEL_X_3X3_08S[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_HT_SOBEL_X_3X3_08S[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_HT_SOBEL_X_3X3_08S[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_HT_SOBEL_X_3X3_08S[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_HT_SOBEL_X_3X3_08S[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_HT_SOBEL_X_3X3_08S[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_HT_SOBEL_X_3X3_08S[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_HT_SOBEL_X_3X3_08S[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 2, 1, {0, 0, 0, 0}}, {32, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_HT_SOBEL_X_3X3_08S[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 2, 1, {0, 0, 0, 0}}, {36, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_HT_SOBEL_X_3X3_08S[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 2, 1, {0, 0, 0, 0}}, {40, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_HT_SOBEL_X_3X3_08S[] = {{44, 8, 4, 0, {1, 1, 2, 2}}, {44, 8, 2, 1, {0, 0, 0, 0}}, {44, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_HT_SOBEL_X_3X3_08S[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_HT_SOBEL_X_3X3_08S[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_HT_SOBEL_X_3X3_08S[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_HT_SOBEL_X_3X3_08S[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_HT_SOBEL_X_3X3_08S[] = {{20, 10, 4, 0, {1, 1, 2, 2}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_HT_SOBEL_X_3X3_08S[] = {{24, 10, 4, 0, {1, 1, 2, 2}}, {24, 10, 2, 1, {0, 0, 0, 0}}, {24, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_HT_SOBEL_X_3X3_08S[] = {{28, 10, 4, 0, {1, 1, 2, 2}}, {28, 10, 2, 1, {0, 0, 0, 0}}, {28, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData113_HT_SOBEL_X_3X3_08S[] = {{32, 10, 4, 0, {1, 1, 2, 2}}, {32, 10, 2, 1, {0, 0, 0, 0}}, {32, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData114_HT_SOBEL_X_3X3_08S[] = {{36, 10, 4, 0, {1, 1, 2, 2}}, {36, 10, 2, 1, {0, 0, 0, 0}}, {36, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData115_HT_SOBEL_X_3X3_08S[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData116_HT_SOBEL_X_3X3_08S[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData117_HT_SOBEL_X_3X3_08S[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData118_HT_SOBEL_X_3X3_08S[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData119_HT_SOBEL_X_3X3_08S[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 2, 1, {0, 0, 0, 0}}, {20, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData120_HT_SOBEL_X_3X3_08S[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 2, 1, {0, 0, 0, 0}}, {24, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData121_HT_SOBEL_X_3X3_08S[] = {{28, 12, 4, 0, {1, 1, 2, 2}}, {28, 12, 2, 1, {0, 0, 0, 0}}, {28, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData122_HT_SOBEL_X_3X3_08S[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 2, 1, {0, 0, 0, 0}}, {4, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData123_HT_SOBEL_X_3X3_08S[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 2, 1, {0, 0, 0, 0}}, {8, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData124_HT_SOBEL_X_3X3_08S[] = {{12, 14, 4, 0, {1, 1, 2, 2}}, {12, 14, 2, 1, {0, 0, 0, 0}}, {12, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData125_HT_SOBEL_X_3X3_08S[] = {{16, 14, 4, 0, {1, 1, 2, 2}}, {16, 14, 2, 1, {0, 0, 0, 0}}, {16, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData126_HT_SOBEL_X_3X3_08S[] = {{20, 14, 4, 0, {1, 1, 2, 2}}, {20, 14, 2, 1, {0, 0, 0, 0}}, {20, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData127_HT_SOBEL_X_3X3_08S[] = {{24, 14, 4, 0, {1, 1, 2, 2}}, {24, 14, 2, 1, {0, 0, 0, 0}}, {24, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData128_HT_SOBEL_X_3X3_08S[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData129_HT_SOBEL_X_3X3_08S[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData130_HT_SOBEL_X_3X3_08S[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData131_HT_SOBEL_X_3X3_08S[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 2, 1, {0, 0, 0, 0}}, {16, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData132_HT_SOBEL_X_3X3_08S[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 2, 1, {0, 0, 0, 0}}, {20, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData133_HT_SOBEL_X_3X3_08S[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 2, 1, {0, 0, 0, 0}}, {4, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData134_HT_SOBEL_X_3X3_08S[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 2, 1, {0, 0, 0, 0}}, {8, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData135_HT_SOBEL_X_3X3_08S[] = {{12, 18, 4, 0, {1, 1, 2, 2}}, {12, 18, 2, 1, {0, 0, 0, 0}}, {12, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData136_HT_SOBEL_X_3X3_08S[] = {{16, 18, 4, 0, {1, 1, 2, 2}}, {16, 18, 2, 1, {0, 0, 0, 0}}, {16, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData137_HT_SOBEL_X_3X3_08S[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData138_HT_SOBEL_X_3X3_08S[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData139_HT_SOBEL_X_3X3_08S[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 2, 1, {0, 0, 0, 0}}, {12, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData140_HT_SOBEL_X_3X3_08S[] = {{16, 20, 4, 0, {1, 1, 2, 2}}, {16, 20, 2, 1, {0, 0, 0, 0}}, {16, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData141_HT_SOBEL_X_3X3_08S[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 2, 1, {0, 0, 0, 0}}, {4, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData142_HT_SOBEL_X_3X3_08S[] = {{8, 22, 4, 0, {1, 1, 2, 2}}, {8, 22, 2, 1, {0, 0, 0, 0}}, {8, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData143_HT_SOBEL_X_3X3_08S[] = {{12, 22, 4, 0, {1, 1, 2, 2}}, {12, 22, 2, 1, {0, 0, 0, 0}}, {12, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData144_HT_SOBEL_X_3X3_08S[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData145_HT_SOBEL_X_3X3_08S[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData146_HT_SOBEL_X_3X3_08S[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 2, 1, {0, 0, 0, 0}}, {12, 24, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData147_HT_SOBEL_X_3X3_08S[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 2, 1, {0, 0, 0, 0}}, {4, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData148_HT_SOBEL_X_3X3_08S[] = {{8, 26, 4, 0, {1, 1, 2, 2}}, {8, 26, 2, 1, {0, 0, 0, 0}}, {8, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData149_HT_SOBEL_X_3X3_08S[] = {{12, 26, 4, 0, {1, 1, 2, 2}}, {12, 26, 2, 1, {0, 0, 0, 0}}, {12, 26, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData150_HT_SOBEL_X_3X3_08S[] = {{4, 28, 4, 0, {1, 1, 2, 2}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData151_HT_SOBEL_X_3X3_08S[] = {{8, 28, 4, 0, {1, 1, 2, 2}}, {8, 28, 2, 1, {0, 0, 0, 0}}, {8, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData152_HT_SOBEL_X_3X3_08S[] = {{12, 28, 4, 0, {1, 1, 2, 2}}, {12, 28, 2, 1, {0, 0, 0, 0}}, {12, 28, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData153_HT_SOBEL_X_3X3_08S[] = {{4, 30, 4, 0, {1, 1, 2, 2}}, {4, 30, 2, 1, {0, 0, 0, 0}}, {4, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData154_HT_SOBEL_X_3X3_08S[] = {{8, 30, 4, 0, {1, 1, 2, 2}}, {8, 30, 2, 1, {0, 0, 0, 0}}, {8, 30, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData155_HT_SOBEL_X_3X3_08S[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData156_HT_SOBEL_X_3X3_08S[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 2, 1, {0, 0, 0, 0}}, {8, 32, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData113_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData114_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData115_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData116_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData117_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData118_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData119_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData120_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData121_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData122_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData123_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData124_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData125_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData126_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData127_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData128_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData129_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData130_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData131_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData132_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData133_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData134_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData135_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData136_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData137_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData138_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData139_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData140_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData141_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData142_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData143_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData144_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData145_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData146_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData147_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData148_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData149_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData150_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData151_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData152_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData153_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData154_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData155_HT_SOBEL_X_3X3_08S[] = {{1, 0}};
static acf_scenario_kernel_data gScenarioKernelData156_HT_SOBEL_X_3X3_08S[] = {{1, 0}};

static acf_scenario gScenarioArray_HT_SOBEL_X_3X3_08S[] = {
{1, 1, 72, 0, 1, gScenarioBufferData0_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData0_HT_SOBEL_X_3X3_08S, 4},
{2, 1, 120, 0, 1, gScenarioBufferData1_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData1_HT_SOBEL_X_3X3_08S, 4},
{3, 1, 168, 0, 1, gScenarioBufferData2_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData2_HT_SOBEL_X_3X3_08S, 4},
{4, 1, 216, 0, 1, gScenarioBufferData3_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData3_HT_SOBEL_X_3X3_08S, 4},
{5, 1, 264, 0, 1, gScenarioBufferData4_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData4_HT_SOBEL_X_3X3_08S, 4},
{6, 1, 312, 0, 1, gScenarioBufferData5_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData5_HT_SOBEL_X_3X3_08S, 4},
{7, 1, 360, 0, 1, gScenarioBufferData6_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData6_HT_SOBEL_X_3X3_08S, 4},
{8, 1, 408, 0, 1, gScenarioBufferData7_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData7_HT_SOBEL_X_3X3_08S, 4},
{9, 1, 456, 0, 1, gScenarioBufferData8_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData8_HT_SOBEL_X_3X3_08S, 4},
{10, 1, 504, 0, 1, gScenarioBufferData9_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData9_HT_SOBEL_X_3X3_08S, 4},
{11, 1, 552, 0, 1, gScenarioBufferData10_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData10_HT_SOBEL_X_3X3_08S, 4},
{12, 1, 600, 0, 1, gScenarioBufferData11_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData11_HT_SOBEL_X_3X3_08S, 4},
{13, 1, 648, 0, 1, gScenarioBufferData12_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData12_HT_SOBEL_X_3X3_08S, 4},
{14, 1, 696, 0, 1, gScenarioBufferData13_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData13_HT_SOBEL_X_3X3_08S, 4},
{15, 1, 744, 0, 1, gScenarioBufferData14_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData14_HT_SOBEL_X_3X3_08S, 4},
{16, 1, 792, 0, 1, gScenarioBufferData15_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData15_HT_SOBEL_X_3X3_08S, 4},
{1, 2, 128, 0, 1, gScenarioBufferData16_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData16_HT_SOBEL_X_3X3_08S, 4},
{2, 2, 216, 0, 1, gScenarioBufferData17_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData17_HT_SOBEL_X_3X3_08S, 4},
{3, 2, 304, 0, 1, gScenarioBufferData18_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData18_HT_SOBEL_X_3X3_08S, 4},
{4, 2, 392, 0, 1, gScenarioBufferData19_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData19_HT_SOBEL_X_3X3_08S, 4},
{5, 2, 480, 0, 1, gScenarioBufferData20_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData20_HT_SOBEL_X_3X3_08S, 4},
{6, 2, 568, 0, 1, gScenarioBufferData21_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData21_HT_SOBEL_X_3X3_08S, 4},
{7, 2, 656, 0, 1, gScenarioBufferData22_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData22_HT_SOBEL_X_3X3_08S, 4},
{8, 2, 744, 0, 1, gScenarioBufferData23_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData23_HT_SOBEL_X_3X3_08S, 4},
{9, 2, 832, 0, 1, gScenarioBufferData24_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData24_HT_SOBEL_X_3X3_08S, 4},
{10, 2, 920, 0, 1, gScenarioBufferData25_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData25_HT_SOBEL_X_3X3_08S, 4},
{11, 2, 1008, 0, 1, gScenarioBufferData26_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData26_HT_SOBEL_X_3X3_08S, 4},
{12, 2, 1096, 0, 1, gScenarioBufferData27_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData27_HT_SOBEL_X_3X3_08S, 4},
{13, 2, 1184, 0, 1, gScenarioBufferData28_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData28_HT_SOBEL_X_3X3_08S, 4},
{14, 2, 1272, 0, 1, gScenarioBufferData29_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData29_HT_SOBEL_X_3X3_08S, 4},
{15, 2, 1360, 0, 1, gScenarioBufferData30_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData30_HT_SOBEL_X_3X3_08S, 4},
{16, 2, 1448, 0, 1, gScenarioBufferData31_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData31_HT_SOBEL_X_3X3_08S, 4},
{1, 3, 184, 0, 1, gScenarioBufferData32_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData32_HT_SOBEL_X_3X3_08S, 4},
{2, 3, 312, 0, 1, gScenarioBufferData33_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData33_HT_SOBEL_X_3X3_08S, 4},
{3, 3, 440, 0, 1, gScenarioBufferData34_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData34_HT_SOBEL_X_3X3_08S, 4},
{4, 3, 568, 0, 1, gScenarioBufferData35_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData35_HT_SOBEL_X_3X3_08S, 4},
{5, 3, 696, 0, 1, gScenarioBufferData36_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData36_HT_SOBEL_X_3X3_08S, 4},
{6, 3, 824, 0, 1, gScenarioBufferData37_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData37_HT_SOBEL_X_3X3_08S, 4},
{7, 3, 952, 0, 1, gScenarioBufferData38_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData38_HT_SOBEL_X_3X3_08S, 4},
{8, 3, 1080, 0, 1, gScenarioBufferData39_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData39_HT_SOBEL_X_3X3_08S, 4},
{9, 3, 1208, 0, 1, gScenarioBufferData40_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData40_HT_SOBEL_X_3X3_08S, 4},
{10, 3, 1336, 0, 1, gScenarioBufferData41_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData41_HT_SOBEL_X_3X3_08S, 4},
{11, 3, 1464, 0, 1, gScenarioBufferData42_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData42_HT_SOBEL_X_3X3_08S, 4},
{12, 3, 1592, 0, 1, gScenarioBufferData43_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData43_HT_SOBEL_X_3X3_08S, 4},
{13, 3, 1720, 0, 1, gScenarioBufferData44_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData44_HT_SOBEL_X_3X3_08S, 4},
{14, 3, 1848, 0, 1, gScenarioBufferData45_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData45_HT_SOBEL_X_3X3_08S, 4},
{15, 3, 1976, 0, 1, gScenarioBufferData46_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData46_HT_SOBEL_X_3X3_08S, 4},
{16, 3, 2104, 0, 1, gScenarioBufferData47_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData47_HT_SOBEL_X_3X3_08S, 4},
{1, 4, 240, 0, 1, gScenarioBufferData48_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData48_HT_SOBEL_X_3X3_08S, 4},
{2, 4, 408, 0, 1, gScenarioBufferData49_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData49_HT_SOBEL_X_3X3_08S, 4},
{3, 4, 576, 0, 1, gScenarioBufferData50_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData50_HT_SOBEL_X_3X3_08S, 4},
{4, 4, 744, 0, 1, gScenarioBufferData51_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData51_HT_SOBEL_X_3X3_08S, 4},
{5, 4, 912, 0, 1, gScenarioBufferData52_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData52_HT_SOBEL_X_3X3_08S, 4},
{6, 4, 1080, 0, 1, gScenarioBufferData53_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData53_HT_SOBEL_X_3X3_08S, 4},
{7, 4, 1248, 0, 1, gScenarioBufferData54_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData54_HT_SOBEL_X_3X3_08S, 4},
{8, 4, 1416, 0, 1, gScenarioBufferData55_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData55_HT_SOBEL_X_3X3_08S, 4},
{9, 4, 1584, 0, 1, gScenarioBufferData56_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData56_HT_SOBEL_X_3X3_08S, 4},
{10, 4, 1752, 0, 1, gScenarioBufferData57_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData57_HT_SOBEL_X_3X3_08S, 4},
{11, 4, 1920, 0, 1, gScenarioBufferData58_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData58_HT_SOBEL_X_3X3_08S, 4},
{12, 4, 2088, 0, 1, gScenarioBufferData59_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData59_HT_SOBEL_X_3X3_08S, 4},
{13, 4, 2256, 0, 1, gScenarioBufferData60_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData60_HT_SOBEL_X_3X3_08S, 4},
{14, 4, 2424, 0, 1, gScenarioBufferData61_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData61_HT_SOBEL_X_3X3_08S, 4},
{15, 4, 2592, 0, 1, gScenarioBufferData62_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData62_HT_SOBEL_X_3X3_08S, 4},
{16, 4, 2760, 0, 1, gScenarioBufferData63_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData63_HT_SOBEL_X_3X3_08S, 4},
{1, 5, 296, 0, 1, gScenarioBufferData64_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData64_HT_SOBEL_X_3X3_08S, 4},
{2, 5, 504, 0, 1, gScenarioBufferData65_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData65_HT_SOBEL_X_3X3_08S, 4},
{3, 5, 712, 0, 1, gScenarioBufferData66_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData66_HT_SOBEL_X_3X3_08S, 4},
{4, 5, 920, 0, 1, gScenarioBufferData67_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData67_HT_SOBEL_X_3X3_08S, 4},
{5, 5, 1128, 0, 1, gScenarioBufferData68_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData68_HT_SOBEL_X_3X3_08S, 4},
{6, 5, 1336, 0, 1, gScenarioBufferData69_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData69_HT_SOBEL_X_3X3_08S, 4},
{7, 5, 1544, 0, 1, gScenarioBufferData70_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData70_HT_SOBEL_X_3X3_08S, 4},
{8, 5, 1752, 0, 1, gScenarioBufferData71_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData71_HT_SOBEL_X_3X3_08S, 4},
{9, 5, 1960, 0, 1, gScenarioBufferData72_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData72_HT_SOBEL_X_3X3_08S, 4},
{10, 5, 2168, 0, 1, gScenarioBufferData73_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData73_HT_SOBEL_X_3X3_08S, 4},
{11, 5, 2376, 0, 1, gScenarioBufferData74_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData74_HT_SOBEL_X_3X3_08S, 4},
{12, 5, 2584, 0, 1, gScenarioBufferData75_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData75_HT_SOBEL_X_3X3_08S, 4},
{13, 5, 2792, 0, 1, gScenarioBufferData76_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData76_HT_SOBEL_X_3X3_08S, 4},
{14, 5, 3000, 0, 1, gScenarioBufferData77_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData77_HT_SOBEL_X_3X3_08S, 4},
{15, 5, 3208, 0, 1, gScenarioBufferData78_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData78_HT_SOBEL_X_3X3_08S, 4},
{16, 5, 3416, 0, 1, gScenarioBufferData79_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData79_HT_SOBEL_X_3X3_08S, 4},
{1, 6, 352, 0, 1, gScenarioBufferData80_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData80_HT_SOBEL_X_3X3_08S, 4},
{2, 6, 600, 0, 1, gScenarioBufferData81_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData81_HT_SOBEL_X_3X3_08S, 4},
{3, 6, 848, 0, 1, gScenarioBufferData82_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData82_HT_SOBEL_X_3X3_08S, 4},
{4, 6, 1096, 0, 1, gScenarioBufferData83_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData83_HT_SOBEL_X_3X3_08S, 4},
{5, 6, 1344, 0, 1, gScenarioBufferData84_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData84_HT_SOBEL_X_3X3_08S, 4},
{6, 6, 1592, 0, 1, gScenarioBufferData85_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData85_HT_SOBEL_X_3X3_08S, 4},
{7, 6, 1840, 0, 1, gScenarioBufferData86_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData86_HT_SOBEL_X_3X3_08S, 4},
{8, 6, 2088, 0, 1, gScenarioBufferData87_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData87_HT_SOBEL_X_3X3_08S, 4},
{9, 6, 2336, 0, 1, gScenarioBufferData88_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData88_HT_SOBEL_X_3X3_08S, 4},
{10, 6, 2584, 0, 1, gScenarioBufferData89_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData89_HT_SOBEL_X_3X3_08S, 4},
{11, 6, 2832, 0, 1, gScenarioBufferData90_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData90_HT_SOBEL_X_3X3_08S, 4},
{12, 6, 3080, 0, 1, gScenarioBufferData91_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData91_HT_SOBEL_X_3X3_08S, 4},
{13, 6, 3328, 0, 1, gScenarioBufferData92_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData92_HT_SOBEL_X_3X3_08S, 4},
{14, 6, 3576, 0, 1, gScenarioBufferData93_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData93_HT_SOBEL_X_3X3_08S, 4},
{15, 6, 3824, 0, 1, gScenarioBufferData94_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData94_HT_SOBEL_X_3X3_08S, 4},
{1, 8, 464, 0, 1, gScenarioBufferData95_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData95_HT_SOBEL_X_3X3_08S, 4},
{2, 8, 792, 0, 1, gScenarioBufferData96_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData96_HT_SOBEL_X_3X3_08S, 4},
{3, 8, 1120, 0, 1, gScenarioBufferData97_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData97_HT_SOBEL_X_3X3_08S, 4},
{4, 8, 1448, 0, 1, gScenarioBufferData98_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData98_HT_SOBEL_X_3X3_08S, 4},
{5, 8, 1776, 0, 1, gScenarioBufferData99_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData99_HT_SOBEL_X_3X3_08S, 4},
{6, 8, 2104, 0, 1, gScenarioBufferData100_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData100_HT_SOBEL_X_3X3_08S, 4},
{7, 8, 2432, 0, 1, gScenarioBufferData101_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData101_HT_SOBEL_X_3X3_08S, 4},
{8, 8, 2760, 0, 1, gScenarioBufferData102_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData102_HT_SOBEL_X_3X3_08S, 4},
{9, 8, 3088, 0, 1, gScenarioBufferData103_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData103_HT_SOBEL_X_3X3_08S, 4},
{10, 8, 3416, 0, 1, gScenarioBufferData104_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData104_HT_SOBEL_X_3X3_08S, 4},
{11, 8, 3744, 0, 1, gScenarioBufferData105_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData105_HT_SOBEL_X_3X3_08S, 4},
{1, 10, 576, 0, 1, gScenarioBufferData106_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData106_HT_SOBEL_X_3X3_08S, 4},
{2, 10, 984, 0, 1, gScenarioBufferData107_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData107_HT_SOBEL_X_3X3_08S, 4},
{3, 10, 1392, 0, 1, gScenarioBufferData108_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData108_HT_SOBEL_X_3X3_08S, 4},
{4, 10, 1800, 0, 1, gScenarioBufferData109_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData109_HT_SOBEL_X_3X3_08S, 4},
{5, 10, 2208, 0, 1, gScenarioBufferData110_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData110_HT_SOBEL_X_3X3_08S, 4},
{6, 10, 2616, 0, 1, gScenarioBufferData111_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData111_HT_SOBEL_X_3X3_08S, 4},
{7, 10, 3024, 0, 1, gScenarioBufferData112_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData112_HT_SOBEL_X_3X3_08S, 4},
{8, 10, 3432, 0, 1, gScenarioBufferData113_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData113_HT_SOBEL_X_3X3_08S, 4},
{9, 10, 3840, 0, 1, gScenarioBufferData114_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData114_HT_SOBEL_X_3X3_08S, 4},
{1, 12, 688, 0, 1, gScenarioBufferData115_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData115_HT_SOBEL_X_3X3_08S, 4},
{2, 12, 1176, 0, 1, gScenarioBufferData116_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData116_HT_SOBEL_X_3X3_08S, 4},
{3, 12, 1664, 0, 1, gScenarioBufferData117_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData117_HT_SOBEL_X_3X3_08S, 4},
{4, 12, 2152, 0, 1, gScenarioBufferData118_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData118_HT_SOBEL_X_3X3_08S, 4},
{5, 12, 2640, 0, 1, gScenarioBufferData119_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData119_HT_SOBEL_X_3X3_08S, 4},
{6, 12, 3128, 0, 1, gScenarioBufferData120_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData120_HT_SOBEL_X_3X3_08S, 4},
{7, 12, 3616, 0, 1, gScenarioBufferData121_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData121_HT_SOBEL_X_3X3_08S, 4},
{1, 14, 800, 0, 1, gScenarioBufferData122_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData122_HT_SOBEL_X_3X3_08S, 4},
{2, 14, 1368, 0, 1, gScenarioBufferData123_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData123_HT_SOBEL_X_3X3_08S, 4},
{3, 14, 1936, 0, 1, gScenarioBufferData124_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData124_HT_SOBEL_X_3X3_08S, 4},
{4, 14, 2504, 0, 1, gScenarioBufferData125_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData125_HT_SOBEL_X_3X3_08S, 4},
{5, 14, 3072, 0, 1, gScenarioBufferData126_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData126_HT_SOBEL_X_3X3_08S, 4},
{6, 14, 3640, 0, 1, gScenarioBufferData127_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData127_HT_SOBEL_X_3X3_08S, 4},
{1, 16, 912, 0, 1, gScenarioBufferData128_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData128_HT_SOBEL_X_3X3_08S, 4},
{2, 16, 1560, 0, 1, gScenarioBufferData129_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData129_HT_SOBEL_X_3X3_08S, 4},
{3, 16, 2208, 0, 1, gScenarioBufferData130_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData130_HT_SOBEL_X_3X3_08S, 4},
{4, 16, 2856, 0, 1, gScenarioBufferData131_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData131_HT_SOBEL_X_3X3_08S, 4},
{5, 16, 3504, 0, 1, gScenarioBufferData132_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData132_HT_SOBEL_X_3X3_08S, 4},
{1, 18, 1024, 0, 1, gScenarioBufferData133_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData133_HT_SOBEL_X_3X3_08S, 4},
{2, 18, 1752, 0, 1, gScenarioBufferData134_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData134_HT_SOBEL_X_3X3_08S, 4},
{3, 18, 2480, 0, 1, gScenarioBufferData135_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData135_HT_SOBEL_X_3X3_08S, 4},
{4, 18, 3208, 0, 1, gScenarioBufferData136_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData136_HT_SOBEL_X_3X3_08S, 4},
{1, 20, 1136, 0, 1, gScenarioBufferData137_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData137_HT_SOBEL_X_3X3_08S, 4},
{2, 20, 1944, 0, 1, gScenarioBufferData138_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData138_HT_SOBEL_X_3X3_08S, 4},
{3, 20, 2752, 0, 1, gScenarioBufferData139_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData139_HT_SOBEL_X_3X3_08S, 4},
{4, 20, 3560, 0, 1, gScenarioBufferData140_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData140_HT_SOBEL_X_3X3_08S, 4},
{1, 22, 1248, 0, 1, gScenarioBufferData141_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData141_HT_SOBEL_X_3X3_08S, 4},
{2, 22, 2136, 0, 1, gScenarioBufferData142_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData142_HT_SOBEL_X_3X3_08S, 4},
{3, 22, 3024, 0, 1, gScenarioBufferData143_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData143_HT_SOBEL_X_3X3_08S, 4},
{1, 24, 1360, 0, 1, gScenarioBufferData144_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData144_HT_SOBEL_X_3X3_08S, 4},
{2, 24, 2328, 0, 1, gScenarioBufferData145_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData145_HT_SOBEL_X_3X3_08S, 4},
{3, 24, 3296, 0, 1, gScenarioBufferData146_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData146_HT_SOBEL_X_3X3_08S, 4},
{1, 26, 1472, 0, 1, gScenarioBufferData147_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData147_HT_SOBEL_X_3X3_08S, 4},
{2, 26, 2520, 0, 1, gScenarioBufferData148_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData148_HT_SOBEL_X_3X3_08S, 4},
{3, 26, 3568, 0, 1, gScenarioBufferData149_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData149_HT_SOBEL_X_3X3_08S, 4},
{1, 28, 1584, 0, 1, gScenarioBufferData150_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData150_HT_SOBEL_X_3X3_08S, 4},
{2, 28, 2712, 0, 1, gScenarioBufferData151_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData151_HT_SOBEL_X_3X3_08S, 4},
{3, 28, 3840, 0, 1, gScenarioBufferData152_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData152_HT_SOBEL_X_3X3_08S, 4},
{1, 30, 1696, 0, 1, gScenarioBufferData153_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData153_HT_SOBEL_X_3X3_08S, 4},
{2, 30, 2904, 0, 1, gScenarioBufferData154_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData154_HT_SOBEL_X_3X3_08S, 4},
{1, 32, 1808, 0, 1, gScenarioBufferData155_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData155_HT_SOBEL_X_3X3_08S, 4},
{2, 32, 3096, 0, 1, gScenarioBufferData156_HT_SOBEL_X_3X3_08S, 48, gScenarioKernelData156_HT_SOBEL_X_3X3_08S, 4}
};

static acf_scenario_list gScenarioList_HT_SOBEL_X_3X3_08S = {
157, //number of scenarios
gScenarioArray_HT_SOBEL_X_3X3_08S};
//**************************************************************

class HT_SOBEL_X_3X3_08S : public ACF_Process_APU
{

public:
   HT_SOBEL_X_3X3_08S(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HT_SOBEL_X_3X3_08S");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HT_SOBEL_X_3X3_08S",
                                        HT_SOBEL_X_3X3_08S_LOAD_SEGMENTS,
                                        HT_SOBEL_X_3X3_08S_LOAD_PMEM, HT_SOBEL_X_3X3_08S_LOAD_PMEM_SIZE,
                                        HT_SOBEL_X_3X3_08S_LOAD_DMEM, HT_SOBEL_X_3X3_08S_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HT_SOBEL_X_3X3_08S_APEX_LOG_BUFFER, HT_SOBEL_X_3X3_08S_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("BUFFER_0", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HT_SOBEL_X_3X3_08S);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HT_SOBEL_X_3X3_08S

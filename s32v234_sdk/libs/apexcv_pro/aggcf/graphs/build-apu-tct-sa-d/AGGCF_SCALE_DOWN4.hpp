#ifndef _ACF_PROCESS_APU_AGGCF_SCALE_DOWN4
#define _ACF_PROCESS_APU_AGGCF_SCALE_DOWN4

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_SCALE_DOWN4_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_SCALE_DOWN4[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_SCALE_DOWN4[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_SCALE_DOWN4[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 1, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_SCALE_DOWN4[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_SCALE_DOWN4[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 1, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_SCALE_DOWN4[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_SCALE_DOWN4[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {7, 4, 1, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_SCALE_DOWN4[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_SCALE_DOWN4[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {9, 4, 1, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_SCALE_DOWN4[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_SCALE_DOWN4[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {11, 4, 1, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_SCALE_DOWN4[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_SCALE_DOWN4[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {13, 4, 1, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_SCALE_DOWN4[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {14, 4, 1, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_SCALE_DOWN4[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {15, 4, 1, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_SCALE_DOWN4[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_SCALE_DOWN4[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 1, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_SCALE_DOWN4[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_SCALE_DOWN4[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {3, 8, 1, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_SCALE_DOWN4[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_SCALE_DOWN4[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {5, 8, 1, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_SCALE_DOWN4[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_SCALE_DOWN4[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {7, 8, 1, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_SCALE_DOWN4[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_SCALE_DOWN4[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {9, 8, 1, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_SCALE_DOWN4[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 1, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_SCALE_DOWN4[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {11, 8, 1, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_SCALE_DOWN4[] = {{48, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 1, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AGGCF_SCALE_DOWN4[] = {{52, 8, 2, 0, {0, 0, 0, 0}}, {13, 8, 1, 0, {0, 0, 0, 0}}, {13, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AGGCF_SCALE_DOWN4[] = {{56, 8, 2, 0, {0, 0, 0, 0}}, {14, 8, 1, 0, {0, 0, 0, 0}}, {14, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AGGCF_SCALE_DOWN4[] = {{60, 8, 2, 0, {0, 0, 0, 0}}, {15, 8, 1, 0, {0, 0, 0, 0}}, {15, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AGGCF_SCALE_DOWN4[] = {{64, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AGGCF_SCALE_DOWN4[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 1, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AGGCF_SCALE_DOWN4[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AGGCF_SCALE_DOWN4[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {3, 12, 1, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AGGCF_SCALE_DOWN4[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AGGCF_SCALE_DOWN4[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {5, 12, 1, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AGGCF_SCALE_DOWN4[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 1, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AGGCF_SCALE_DOWN4[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {7, 12, 1, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AGGCF_SCALE_DOWN4[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AGGCF_SCALE_DOWN4[] = {{36, 12, 2, 0, {0, 0, 0, 0}}, {9, 12, 1, 0, {0, 0, 0, 0}}, {9, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AGGCF_SCALE_DOWN4[] = {{40, 12, 2, 0, {0, 0, 0, 0}}, {10, 12, 1, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AGGCF_SCALE_DOWN4[] = {{44, 12, 2, 0, {0, 0, 0, 0}}, {11, 12, 1, 0, {0, 0, 0, 0}}, {11, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AGGCF_SCALE_DOWN4[] = {{48, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 1, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AGGCF_SCALE_DOWN4[] = {{52, 12, 2, 0, {0, 0, 0, 0}}, {13, 12, 1, 0, {0, 0, 0, 0}}, {13, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AGGCF_SCALE_DOWN4[] = {{56, 12, 2, 0, {0, 0, 0, 0}}, {14, 12, 1, 0, {0, 0, 0, 0}}, {14, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AGGCF_SCALE_DOWN4[] = {{60, 12, 2, 0, {0, 0, 0, 0}}, {15, 12, 1, 0, {0, 0, 0, 0}}, {15, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AGGCF_SCALE_DOWN4[] = {{64, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AGGCF_SCALE_DOWN4[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 1, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AGGCF_SCALE_DOWN4[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AGGCF_SCALE_DOWN4[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {3, 16, 1, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AGGCF_SCALE_DOWN4[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AGGCF_SCALE_DOWN4[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {5, 16, 1, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AGGCF_SCALE_DOWN4[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 1, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AGGCF_SCALE_DOWN4[] = {{28, 16, 2, 0, {0, 0, 0, 0}}, {7, 16, 1, 0, {0, 0, 0, 0}}, {7, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AGGCF_SCALE_DOWN4[] = {{32, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_AGGCF_SCALE_DOWN4[] = {{36, 16, 2, 0, {0, 0, 0, 0}}, {9, 16, 1, 0, {0, 0, 0, 0}}, {9, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_AGGCF_SCALE_DOWN4[] = {{40, 16, 2, 0, {0, 0, 0, 0}}, {10, 16, 1, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_AGGCF_SCALE_DOWN4[] = {{44, 16, 2, 0, {0, 0, 0, 0}}, {11, 16, 1, 0, {0, 0, 0, 0}}, {11, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_AGGCF_SCALE_DOWN4[] = {{48, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_AGGCF_SCALE_DOWN4[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 1, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_AGGCF_SCALE_DOWN4[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_AGGCF_SCALE_DOWN4[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {3, 20, 1, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_AGGCF_SCALE_DOWN4[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 1, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_AGGCF_SCALE_DOWN4[] = {{20, 20, 2, 0, {0, 0, 0, 0}}, {5, 20, 1, 0, {0, 0, 0, 0}}, {5, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_AGGCF_SCALE_DOWN4[] = {{24, 20, 2, 0, {0, 0, 0, 0}}, {6, 20, 1, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_AGGCF_SCALE_DOWN4[] = {{28, 20, 2, 0, {0, 0, 0, 0}}, {7, 20, 1, 0, {0, 0, 0, 0}}, {7, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_AGGCF_SCALE_DOWN4[] = {{32, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_AGGCF_SCALE_DOWN4[] = {{36, 20, 2, 0, {0, 0, 0, 0}}, {9, 20, 1, 0, {0, 0, 0, 0}}, {9, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_AGGCF_SCALE_DOWN4[] = {{40, 20, 2, 0, {0, 0, 0, 0}}, {10, 20, 1, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_AGGCF_SCALE_DOWN4[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {1, 24, 1, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_AGGCF_SCALE_DOWN4[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_AGGCF_SCALE_DOWN4[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {3, 24, 1, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_AGGCF_SCALE_DOWN4[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 1, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_AGGCF_SCALE_DOWN4[] = {{20, 24, 2, 0, {0, 0, 0, 0}}, {5, 24, 1, 0, {0, 0, 0, 0}}, {5, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_AGGCF_SCALE_DOWN4[] = {{24, 24, 2, 0, {0, 0, 0, 0}}, {6, 24, 1, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_AGGCF_SCALE_DOWN4[] = {{28, 24, 2, 0, {0, 0, 0, 0}}, {7, 24, 1, 0, {0, 0, 0, 0}}, {7, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_AGGCF_SCALE_DOWN4[] = {{32, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_AGGCF_SCALE_DOWN4[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {1, 32, 1, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_AGGCF_SCALE_DOWN4[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 1, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_AGGCF_SCALE_DOWN4[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {3, 32, 1, 0, {0, 0, 0, 0}}, {3, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_AGGCF_SCALE_DOWN4[] = {{16, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_AGGCF_SCALE_DOWN4[] = {{20, 32, 2, 0, {0, 0, 0, 0}}, {5, 32, 1, 0, {0, 0, 0, 0}}, {5, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_AGGCF_SCALE_DOWN4[] = {{24, 32, 2, 0, {0, 0, 0, 0}}, {6, 32, 1, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_AGGCF_SCALE_DOWN4[] = {{4, 40, 2, 0, {0, 0, 0, 0}}, {1, 40, 1, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_AGGCF_SCALE_DOWN4[] = {{8, 40, 2, 0, {0, 0, 0, 0}}, {2, 40, 1, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_AGGCF_SCALE_DOWN4[] = {{12, 40, 2, 0, {0, 0, 0, 0}}, {3, 40, 1, 0, {0, 0, 0, 0}}, {3, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_AGGCF_SCALE_DOWN4[] = {{16, 40, 2, 0, {0, 0, 0, 0}}, {4, 40, 1, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_AGGCF_SCALE_DOWN4[] = {{20, 40, 2, 0, {0, 0, 0, 0}}, {5, 40, 1, 0, {0, 0, 0, 0}}, {5, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_AGGCF_SCALE_DOWN4[] = {{4, 48, 2, 0, {0, 0, 0, 0}}, {1, 48, 1, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_AGGCF_SCALE_DOWN4[] = {{8, 48, 2, 0, {0, 0, 0, 0}}, {2, 48, 1, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_AGGCF_SCALE_DOWN4[] = {{12, 48, 2, 0, {0, 0, 0, 0}}, {3, 48, 1, 0, {0, 0, 0, 0}}, {3, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_AGGCF_SCALE_DOWN4[] = {{16, 48, 2, 0, {0, 0, 0, 0}}, {4, 48, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_AGGCF_SCALE_DOWN4[] = {{4, 56, 2, 0, {0, 0, 0, 0}}, {1, 56, 1, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_AGGCF_SCALE_DOWN4[] = {{8, 56, 2, 0, {0, 0, 0, 0}}, {2, 56, 1, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_AGGCF_SCALE_DOWN4[] = {{12, 56, 2, 0, {0, 0, 0, 0}}, {3, 56, 1, 0, {0, 0, 0, 0}}, {3, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_AGGCF_SCALE_DOWN4[] = {{4, 64, 2, 0, {0, 0, 0, 0}}, {1, 64, 1, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_AGGCF_SCALE_DOWN4[] = {{8, 64, 2, 0, {0, 0, 0, 0}}, {2, 64, 1, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_AGGCF_SCALE_DOWN4[] = {{12, 64, 2, 0, {0, 0, 0, 0}}, {3, 64, 1, 0, {0, 0, 0, 0}}, {3, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_AGGCF_SCALE_DOWN4[] = {{4, 72, 2, 0, {0, 0, 0, 0}}, {1, 72, 1, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_AGGCF_SCALE_DOWN4[] = {{8, 72, 2, 0, {0, 0, 0, 0}}, {2, 72, 1, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_AGGCF_SCALE_DOWN4[] = {{4, 80, 2, 0, {0, 0, 0, 0}}, {1, 80, 1, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_AGGCF_SCALE_DOWN4[] = {{8, 80, 2, 0, {0, 0, 0, 0}}, {2, 80, 1, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_AGGCF_SCALE_DOWN4[] = {{4, 88, 2, 0, {0, 0, 0, 0}}, {1, 88, 1, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_AGGCF_SCALE_DOWN4[] = {{8, 88, 2, 0, {0, 0, 0, 0}}, {2, 88, 1, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_AGGCF_SCALE_DOWN4[] = {{4, 96, 2, 0, {0, 0, 0, 0}}, {1, 96, 1, 0, {0, 0, 0, 0}}, {1, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_AGGCF_SCALE_DOWN4[] = {{8, 96, 2, 0, {0, 0, 0, 0}}, {2, 96, 1, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_AGGCF_SCALE_DOWN4[] = {{4, 104, 2, 0, {0, 0, 0, 0}}, {1, 104, 1, 0, {0, 0, 0, 0}}, {1, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_AGGCF_SCALE_DOWN4[] = {{4, 112, 2, 0, {0, 0, 0, 0}}, {1, 112, 1, 0, {0, 0, 0, 0}}, {1, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_AGGCF_SCALE_DOWN4[] = {{4, 120, 2, 0, {0, 0, 0, 0}}, {1, 120, 1, 0, {0, 0, 0, 0}}, {1, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_AGGCF_SCALE_DOWN4[] = {{4, 128, 2, 0, {0, 0, 0, 0}}, {1, 128, 1, 0, {0, 0, 0, 0}}, {1, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_AGGCF_SCALE_DOWN4[] = {{0, 0}, {0, 0}};

static acf_scenario gScenarioArray_AGGCF_SCALE_DOWN4[] = {
{1, 1, 76, 0, 0, gScenarioBufferData0_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData0_AGGCF_SCALE_DOWN4, 8},
{2, 1, 152, 0, 0, gScenarioBufferData1_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData1_AGGCF_SCALE_DOWN4, 8},
{3, 1, 228, 0, 0, gScenarioBufferData2_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData2_AGGCF_SCALE_DOWN4, 8},
{4, 1, 304, 0, 0, gScenarioBufferData3_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData3_AGGCF_SCALE_DOWN4, 8},
{5, 1, 380, 0, 0, gScenarioBufferData4_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData4_AGGCF_SCALE_DOWN4, 8},
{6, 1, 456, 0, 0, gScenarioBufferData5_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData5_AGGCF_SCALE_DOWN4, 8},
{7, 1, 532, 0, 0, gScenarioBufferData6_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData6_AGGCF_SCALE_DOWN4, 8},
{8, 1, 608, 0, 0, gScenarioBufferData7_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData7_AGGCF_SCALE_DOWN4, 8},
{9, 1, 684, 0, 0, gScenarioBufferData8_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData8_AGGCF_SCALE_DOWN4, 8},
{10, 1, 760, 0, 0, gScenarioBufferData9_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData9_AGGCF_SCALE_DOWN4, 8},
{11, 1, 836, 0, 0, gScenarioBufferData10_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData10_AGGCF_SCALE_DOWN4, 8},
{12, 1, 912, 0, 0, gScenarioBufferData11_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData11_AGGCF_SCALE_DOWN4, 8},
{13, 1, 988, 0, 0, gScenarioBufferData12_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData12_AGGCF_SCALE_DOWN4, 8},
{14, 1, 1064, 0, 0, gScenarioBufferData13_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData13_AGGCF_SCALE_DOWN4, 8},
{15, 1, 1140, 0, 0, gScenarioBufferData14_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData14_AGGCF_SCALE_DOWN4, 8},
{16, 1, 1216, 0, 0, gScenarioBufferData15_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData15_AGGCF_SCALE_DOWN4, 8},
{1, 2, 152, 0, 0, gScenarioBufferData16_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData16_AGGCF_SCALE_DOWN4, 8},
{2, 2, 304, 0, 0, gScenarioBufferData17_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData17_AGGCF_SCALE_DOWN4, 8},
{3, 2, 456, 0, 0, gScenarioBufferData18_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData18_AGGCF_SCALE_DOWN4, 8},
{4, 2, 608, 0, 0, gScenarioBufferData19_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData19_AGGCF_SCALE_DOWN4, 8},
{5, 2, 760, 0, 0, gScenarioBufferData20_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData20_AGGCF_SCALE_DOWN4, 8},
{6, 2, 912, 0, 0, gScenarioBufferData21_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData21_AGGCF_SCALE_DOWN4, 8},
{7, 2, 1064, 0, 0, gScenarioBufferData22_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData22_AGGCF_SCALE_DOWN4, 8},
{8, 2, 1216, 0, 0, gScenarioBufferData23_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData23_AGGCF_SCALE_DOWN4, 8},
{9, 2, 1368, 0, 0, gScenarioBufferData24_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData24_AGGCF_SCALE_DOWN4, 8},
{10, 2, 1520, 0, 0, gScenarioBufferData25_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData25_AGGCF_SCALE_DOWN4, 8},
{11, 2, 1672, 0, 0, gScenarioBufferData26_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData26_AGGCF_SCALE_DOWN4, 8},
{12, 2, 1824, 0, 0, gScenarioBufferData27_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData27_AGGCF_SCALE_DOWN4, 8},
{13, 2, 1976, 0, 0, gScenarioBufferData28_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData28_AGGCF_SCALE_DOWN4, 8},
{14, 2, 2128, 0, 0, gScenarioBufferData29_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData29_AGGCF_SCALE_DOWN4, 8},
{15, 2, 2280, 0, 0, gScenarioBufferData30_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData30_AGGCF_SCALE_DOWN4, 8},
{16, 2, 2432, 0, 0, gScenarioBufferData31_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData31_AGGCF_SCALE_DOWN4, 8},
{1, 3, 228, 0, 0, gScenarioBufferData32_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData32_AGGCF_SCALE_DOWN4, 8},
{2, 3, 456, 0, 0, gScenarioBufferData33_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData33_AGGCF_SCALE_DOWN4, 8},
{3, 3, 684, 0, 0, gScenarioBufferData34_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData34_AGGCF_SCALE_DOWN4, 8},
{4, 3, 912, 0, 0, gScenarioBufferData35_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData35_AGGCF_SCALE_DOWN4, 8},
{5, 3, 1140, 0, 0, gScenarioBufferData36_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData36_AGGCF_SCALE_DOWN4, 8},
{6, 3, 1368, 0, 0, gScenarioBufferData37_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData37_AGGCF_SCALE_DOWN4, 8},
{7, 3, 1596, 0, 0, gScenarioBufferData38_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData38_AGGCF_SCALE_DOWN4, 8},
{8, 3, 1824, 0, 0, gScenarioBufferData39_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData39_AGGCF_SCALE_DOWN4, 8},
{9, 3, 2052, 0, 0, gScenarioBufferData40_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData40_AGGCF_SCALE_DOWN4, 8},
{10, 3, 2280, 0, 0, gScenarioBufferData41_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData41_AGGCF_SCALE_DOWN4, 8},
{11, 3, 2508, 0, 0, gScenarioBufferData42_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData42_AGGCF_SCALE_DOWN4, 8},
{12, 3, 2736, 0, 0, gScenarioBufferData43_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData43_AGGCF_SCALE_DOWN4, 8},
{13, 3, 2964, 0, 0, gScenarioBufferData44_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData44_AGGCF_SCALE_DOWN4, 8},
{14, 3, 3192, 0, 0, gScenarioBufferData45_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData45_AGGCF_SCALE_DOWN4, 8},
{15, 3, 3420, 0, 0, gScenarioBufferData46_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData46_AGGCF_SCALE_DOWN4, 8},
{16, 3, 3648, 0, 0, gScenarioBufferData47_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData47_AGGCF_SCALE_DOWN4, 8},
{1, 4, 304, 0, 0, gScenarioBufferData48_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData48_AGGCF_SCALE_DOWN4, 8},
{2, 4, 608, 0, 0, gScenarioBufferData49_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData49_AGGCF_SCALE_DOWN4, 8},
{3, 4, 912, 0, 0, gScenarioBufferData50_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData50_AGGCF_SCALE_DOWN4, 8},
{4, 4, 1216, 0, 0, gScenarioBufferData51_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData51_AGGCF_SCALE_DOWN4, 8},
{5, 4, 1520, 0, 0, gScenarioBufferData52_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData52_AGGCF_SCALE_DOWN4, 8},
{6, 4, 1824, 0, 0, gScenarioBufferData53_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData53_AGGCF_SCALE_DOWN4, 8},
{7, 4, 2128, 0, 0, gScenarioBufferData54_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData54_AGGCF_SCALE_DOWN4, 8},
{8, 4, 2432, 0, 0, gScenarioBufferData55_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData55_AGGCF_SCALE_DOWN4, 8},
{9, 4, 2736, 0, 0, gScenarioBufferData56_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData56_AGGCF_SCALE_DOWN4, 8},
{10, 4, 3040, 0, 0, gScenarioBufferData57_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData57_AGGCF_SCALE_DOWN4, 8},
{11, 4, 3344, 0, 0, gScenarioBufferData58_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData58_AGGCF_SCALE_DOWN4, 8},
{12, 4, 3648, 0, 0, gScenarioBufferData59_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData59_AGGCF_SCALE_DOWN4, 8},
{1, 5, 380, 0, 0, gScenarioBufferData60_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData60_AGGCF_SCALE_DOWN4, 8},
{2, 5, 760, 0, 0, gScenarioBufferData61_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData61_AGGCF_SCALE_DOWN4, 8},
{3, 5, 1140, 0, 0, gScenarioBufferData62_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData62_AGGCF_SCALE_DOWN4, 8},
{4, 5, 1520, 0, 0, gScenarioBufferData63_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData63_AGGCF_SCALE_DOWN4, 8},
{5, 5, 1900, 0, 0, gScenarioBufferData64_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData64_AGGCF_SCALE_DOWN4, 8},
{6, 5, 2280, 0, 0, gScenarioBufferData65_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData65_AGGCF_SCALE_DOWN4, 8},
{7, 5, 2660, 0, 0, gScenarioBufferData66_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData66_AGGCF_SCALE_DOWN4, 8},
{8, 5, 3040, 0, 0, gScenarioBufferData67_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData67_AGGCF_SCALE_DOWN4, 8},
{9, 5, 3420, 0, 0, gScenarioBufferData68_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData68_AGGCF_SCALE_DOWN4, 8},
{10, 5, 3800, 0, 0, gScenarioBufferData69_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData69_AGGCF_SCALE_DOWN4, 8},
{1, 6, 456, 0, 0, gScenarioBufferData70_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData70_AGGCF_SCALE_DOWN4, 8},
{2, 6, 912, 0, 0, gScenarioBufferData71_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData71_AGGCF_SCALE_DOWN4, 8},
{3, 6, 1368, 0, 0, gScenarioBufferData72_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData72_AGGCF_SCALE_DOWN4, 8},
{4, 6, 1824, 0, 0, gScenarioBufferData73_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData73_AGGCF_SCALE_DOWN4, 8},
{5, 6, 2280, 0, 0, gScenarioBufferData74_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData74_AGGCF_SCALE_DOWN4, 8},
{6, 6, 2736, 0, 0, gScenarioBufferData75_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData75_AGGCF_SCALE_DOWN4, 8},
{7, 6, 3192, 0, 0, gScenarioBufferData76_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData76_AGGCF_SCALE_DOWN4, 8},
{8, 6, 3648, 0, 0, gScenarioBufferData77_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData77_AGGCF_SCALE_DOWN4, 8},
{1, 8, 608, 0, 0, gScenarioBufferData78_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData78_AGGCF_SCALE_DOWN4, 8},
{2, 8, 1216, 0, 0, gScenarioBufferData79_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData79_AGGCF_SCALE_DOWN4, 8},
{3, 8, 1824, 0, 0, gScenarioBufferData80_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData80_AGGCF_SCALE_DOWN4, 8},
{4, 8, 2432, 0, 0, gScenarioBufferData81_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData81_AGGCF_SCALE_DOWN4, 8},
{5, 8, 3040, 0, 0, gScenarioBufferData82_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData82_AGGCF_SCALE_DOWN4, 8},
{6, 8, 3648, 0, 0, gScenarioBufferData83_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData83_AGGCF_SCALE_DOWN4, 8},
{1, 10, 760, 0, 0, gScenarioBufferData84_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData84_AGGCF_SCALE_DOWN4, 8},
{2, 10, 1520, 0, 0, gScenarioBufferData85_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData85_AGGCF_SCALE_DOWN4, 8},
{3, 10, 2280, 0, 0, gScenarioBufferData86_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData86_AGGCF_SCALE_DOWN4, 8},
{4, 10, 3040, 0, 0, gScenarioBufferData87_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData87_AGGCF_SCALE_DOWN4, 8},
{5, 10, 3800, 0, 0, gScenarioBufferData88_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData88_AGGCF_SCALE_DOWN4, 8},
{1, 12, 912, 0, 0, gScenarioBufferData89_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData89_AGGCF_SCALE_DOWN4, 8},
{2, 12, 1824, 0, 0, gScenarioBufferData90_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData90_AGGCF_SCALE_DOWN4, 8},
{3, 12, 2736, 0, 0, gScenarioBufferData91_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData91_AGGCF_SCALE_DOWN4, 8},
{4, 12, 3648, 0, 0, gScenarioBufferData92_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData92_AGGCF_SCALE_DOWN4, 8},
{1, 14, 1064, 0, 0, gScenarioBufferData93_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData93_AGGCF_SCALE_DOWN4, 8},
{2, 14, 2128, 0, 0, gScenarioBufferData94_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData94_AGGCF_SCALE_DOWN4, 8},
{3, 14, 3192, 0, 0, gScenarioBufferData95_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData95_AGGCF_SCALE_DOWN4, 8},
{1, 16, 1216, 0, 0, gScenarioBufferData96_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData96_AGGCF_SCALE_DOWN4, 8},
{2, 16, 2432, 0, 0, gScenarioBufferData97_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData97_AGGCF_SCALE_DOWN4, 8},
{3, 16, 3648, 0, 0, gScenarioBufferData98_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData98_AGGCF_SCALE_DOWN4, 8},
{1, 18, 1368, 0, 0, gScenarioBufferData99_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData99_AGGCF_SCALE_DOWN4, 8},
{2, 18, 2736, 0, 0, gScenarioBufferData100_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData100_AGGCF_SCALE_DOWN4, 8},
{1, 20, 1520, 0, 0, gScenarioBufferData101_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData101_AGGCF_SCALE_DOWN4, 8},
{2, 20, 3040, 0, 0, gScenarioBufferData102_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData102_AGGCF_SCALE_DOWN4, 8},
{1, 22, 1672, 0, 0, gScenarioBufferData103_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData103_AGGCF_SCALE_DOWN4, 8},
{2, 22, 3344, 0, 0, gScenarioBufferData104_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData104_AGGCF_SCALE_DOWN4, 8},
{1, 24, 1824, 0, 0, gScenarioBufferData105_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData105_AGGCF_SCALE_DOWN4, 8},
{2, 24, 3648, 0, 0, gScenarioBufferData106_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData106_AGGCF_SCALE_DOWN4, 8},
{1, 26, 1976, 0, 0, gScenarioBufferData107_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData107_AGGCF_SCALE_DOWN4, 8},
{1, 28, 2128, 0, 0, gScenarioBufferData108_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData108_AGGCF_SCALE_DOWN4, 8},
{1, 30, 2280, 0, 0, gScenarioBufferData109_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData109_AGGCF_SCALE_DOWN4, 8},
{1, 32, 2432, 0, 0, gScenarioBufferData110_AGGCF_SCALE_DOWN4, 48, gScenarioKernelData110_AGGCF_SCALE_DOWN4, 8}
};

static acf_scenario_list gScenarioList_AGGCF_SCALE_DOWN4 = {
111, //number of scenarios
gScenarioArray_AGGCF_SCALE_DOWN4};
//**************************************************************

class AGGCF_SCALE_DOWN4 : public ACF_Process_APU
{

public:
   AGGCF_SCALE_DOWN4(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_SCALE_DOWN4");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_SCALE_DOWN4",
                                        AGGCF_SCALE_DOWN4_LOAD_SEGMENTS,
                                        AGGCF_SCALE_DOWN4_LOAD_PMEM, AGGCF_SCALE_DOWN4_LOAD_PMEM_SIZE,
                                        AGGCF_SCALE_DOWN4_LOAD_DMEM, AGGCF_SCALE_DOWN4_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_SCALE_DOWN4_APEX_LOG_BUFFER, AGGCF_SCALE_DOWN4_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("SRC", icp::DATATYPE_16U, 1, 1, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("DST", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_SCALE_DOWN4);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_SCALE_DOWN4

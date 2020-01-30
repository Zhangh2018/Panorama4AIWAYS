#ifndef _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID
#define _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <LAPLACIAN_PYRAMID_MID_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_LAPLACIAN_PYRAMID_MID[] = {{4, 2, 3, 0, {0, 0, 2, 2}}, {4, 2, 4, 0, {2, 2, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}, {2, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_LAPLACIAN_PYRAMID_MID[] = {{8, 2, 3, 0, {0, 0, 2, 2}}, {8, 2, 4, 0, {2, 2, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}, {4, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_LAPLACIAN_PYRAMID_MID[] = {{12, 2, 3, 0, {0, 0, 2, 2}}, {12, 2, 4, 0, {2, 2, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}, {6, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_LAPLACIAN_PYRAMID_MID[] = {{16, 2, 3, 0, {0, 0, 2, 2}}, {16, 2, 4, 0, {2, 2, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}, {8, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_LAPLACIAN_PYRAMID_MID[] = {{20, 2, 3, 0, {0, 0, 2, 2}}, {20, 2, 4, 0, {2, 2, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}, {10, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_LAPLACIAN_PYRAMID_MID[] = {{24, 2, 3, 0, {0, 0, 2, 2}}, {24, 2, 4, 0, {2, 2, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}, {12, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_LAPLACIAN_PYRAMID_MID[] = {{28, 2, 3, 0, {0, 0, 2, 2}}, {28, 2, 4, 0, {2, 2, 0, 0}}, {28, 2, 2, 1, {0, 0, 0, 0}}, {14, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_LAPLACIAN_PYRAMID_MID[] = {{32, 2, 3, 0, {0, 0, 2, 2}}, {32, 2, 4, 0, {2, 2, 0, 0}}, {32, 2, 2, 1, {0, 0, 0, 0}}, {16, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_LAPLACIAN_PYRAMID_MID[] = {{36, 2, 3, 0, {0, 0, 2, 2}}, {36, 2, 4, 0, {2, 2, 0, 0}}, {36, 2, 2, 1, {0, 0, 0, 0}}, {18, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_LAPLACIAN_PYRAMID_MID[] = {{40, 2, 3, 0, {0, 0, 2, 2}}, {40, 2, 4, 0, {2, 2, 0, 0}}, {40, 2, 2, 1, {0, 0, 0, 0}}, {20, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_LAPLACIAN_PYRAMID_MID[] = {{44, 2, 3, 0, {0, 0, 2, 2}}, {44, 2, 4, 0, {2, 2, 0, 0}}, {44, 2, 2, 1, {0, 0, 0, 0}}, {22, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_LAPLACIAN_PYRAMID_MID[] = {{48, 2, 3, 0, {0, 0, 2, 2}}, {48, 2, 4, 0, {2, 2, 0, 0}}, {48, 2, 2, 1, {0, 0, 0, 0}}, {24, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_LAPLACIAN_PYRAMID_MID[] = {{52, 2, 3, 0, {0, 0, 2, 2}}, {52, 2, 4, 0, {2, 2, 0, 0}}, {52, 2, 2, 1, {0, 0, 0, 0}}, {26, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_LAPLACIAN_PYRAMID_MID[] = {{56, 2, 3, 0, {0, 0, 2, 2}}, {56, 2, 4, 0, {2, 2, 0, 0}}, {56, 2, 2, 1, {0, 0, 0, 0}}, {28, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_LAPLACIAN_PYRAMID_MID[] = {{60, 2, 3, 0, {0, 0, 2, 2}}, {60, 2, 4, 0, {2, 2, 0, 0}}, {60, 2, 2, 1, {0, 0, 0, 0}}, {30, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_LAPLACIAN_PYRAMID_MID[] = {{64, 2, 3, 0, {0, 0, 2, 2}}, {64, 2, 4, 0, {2, 2, 0, 0}}, {64, 2, 2, 1, {0, 0, 0, 0}}, {32, 1, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_LAPLACIAN_PYRAMID_MID[] = {{4, 4, 3, 0, {0, 0, 2, 2}}, {4, 4, 4, 0, {2, 2, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}, {2, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_LAPLACIAN_PYRAMID_MID[] = {{8, 4, 3, 0, {0, 0, 2, 2}}, {8, 4, 4, 0, {2, 2, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}, {4, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_LAPLACIAN_PYRAMID_MID[] = {{12, 4, 3, 0, {0, 0, 2, 2}}, {12, 4, 4, 0, {2, 2, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}, {6, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_LAPLACIAN_PYRAMID_MID[] = {{16, 4, 3, 0, {0, 0, 2, 2}}, {16, 4, 4, 0, {2, 2, 0, 0}}, {16, 4, 2, 1, {0, 0, 0, 0}}, {8, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_LAPLACIAN_PYRAMID_MID[] = {{20, 4, 3, 0, {0, 0, 2, 2}}, {20, 4, 4, 0, {2, 2, 0, 0}}, {20, 4, 2, 1, {0, 0, 0, 0}}, {10, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_LAPLACIAN_PYRAMID_MID[] = {{24, 4, 3, 0, {0, 0, 2, 2}}, {24, 4, 4, 0, {2, 2, 0, 0}}, {24, 4, 2, 1, {0, 0, 0, 0}}, {12, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_LAPLACIAN_PYRAMID_MID[] = {{28, 4, 3, 0, {0, 0, 2, 2}}, {28, 4, 4, 0, {2, 2, 0, 0}}, {28, 4, 2, 1, {0, 0, 0, 0}}, {14, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_LAPLACIAN_PYRAMID_MID[] = {{32, 4, 3, 0, {0, 0, 2, 2}}, {32, 4, 4, 0, {2, 2, 0, 0}}, {32, 4, 2, 1, {0, 0, 0, 0}}, {16, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_LAPLACIAN_PYRAMID_MID[] = {{36, 4, 3, 0, {0, 0, 2, 2}}, {36, 4, 4, 0, {2, 2, 0, 0}}, {36, 4, 2, 1, {0, 0, 0, 0}}, {18, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_LAPLACIAN_PYRAMID_MID[] = {{40, 4, 3, 0, {0, 0, 2, 2}}, {40, 4, 4, 0, {2, 2, 0, 0}}, {40, 4, 2, 1, {0, 0, 0, 0}}, {20, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_LAPLACIAN_PYRAMID_MID[] = {{44, 4, 3, 0, {0, 0, 2, 2}}, {44, 4, 4, 0, {2, 2, 0, 0}}, {44, 4, 2, 1, {0, 0, 0, 0}}, {22, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_LAPLACIAN_PYRAMID_MID[] = {{48, 4, 3, 0, {0, 0, 2, 2}}, {48, 4, 4, 0, {2, 2, 0, 0}}, {48, 4, 2, 1, {0, 0, 0, 0}}, {24, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_LAPLACIAN_PYRAMID_MID[] = {{52, 4, 3, 0, {0, 0, 2, 2}}, {52, 4, 4, 0, {2, 2, 0, 0}}, {52, 4, 2, 1, {0, 0, 0, 0}}, {26, 2, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_LAPLACIAN_PYRAMID_MID[] = {{4, 6, 3, 0, {0, 0, 2, 2}}, {4, 6, 4, 0, {2, 2, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}, {2, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_LAPLACIAN_PYRAMID_MID[] = {{8, 6, 3, 0, {0, 0, 2, 2}}, {8, 6, 4, 0, {2, 2, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}, {4, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_LAPLACIAN_PYRAMID_MID[] = {{12, 6, 3, 0, {0, 0, 2, 2}}, {12, 6, 4, 0, {2, 2, 0, 0}}, {12, 6, 2, 1, {0, 0, 0, 0}}, {6, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_LAPLACIAN_PYRAMID_MID[] = {{16, 6, 3, 0, {0, 0, 2, 2}}, {16, 6, 4, 0, {2, 2, 0, 0}}, {16, 6, 2, 1, {0, 0, 0, 0}}, {8, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_LAPLACIAN_PYRAMID_MID[] = {{20, 6, 3, 0, {0, 0, 2, 2}}, {20, 6, 4, 0, {2, 2, 0, 0}}, {20, 6, 2, 1, {0, 0, 0, 0}}, {10, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_LAPLACIAN_PYRAMID_MID[] = {{24, 6, 3, 0, {0, 0, 2, 2}}, {24, 6, 4, 0, {2, 2, 0, 0}}, {24, 6, 2, 1, {0, 0, 0, 0}}, {12, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_LAPLACIAN_PYRAMID_MID[] = {{28, 6, 3, 0, {0, 0, 2, 2}}, {28, 6, 4, 0, {2, 2, 0, 0}}, {28, 6, 2, 1, {0, 0, 0, 0}}, {14, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_LAPLACIAN_PYRAMID_MID[] = {{32, 6, 3, 0, {0, 0, 2, 2}}, {32, 6, 4, 0, {2, 2, 0, 0}}, {32, 6, 2, 1, {0, 0, 0, 0}}, {16, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_LAPLACIAN_PYRAMID_MID[] = {{36, 6, 3, 0, {0, 0, 2, 2}}, {36, 6, 4, 0, {2, 2, 0, 0}}, {36, 6, 2, 1, {0, 0, 0, 0}}, {18, 3, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_LAPLACIAN_PYRAMID_MID[] = {{4, 8, 3, 0, {0, 0, 2, 2}}, {4, 8, 4, 0, {2, 2, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}, {2, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_LAPLACIAN_PYRAMID_MID[] = {{8, 8, 3, 0, {0, 0, 2, 2}}, {8, 8, 4, 0, {2, 2, 0, 0}}, {8, 8, 2, 1, {0, 0, 0, 0}}, {4, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_LAPLACIAN_PYRAMID_MID[] = {{12, 8, 3, 0, {0, 0, 2, 2}}, {12, 8, 4, 0, {2, 2, 0, 0}}, {12, 8, 2, 1, {0, 0, 0, 0}}, {6, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_LAPLACIAN_PYRAMID_MID[] = {{16, 8, 3, 0, {0, 0, 2, 2}}, {16, 8, 4, 0, {2, 2, 0, 0}}, {16, 8, 2, 1, {0, 0, 0, 0}}, {8, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_LAPLACIAN_PYRAMID_MID[] = {{20, 8, 3, 0, {0, 0, 2, 2}}, {20, 8, 4, 0, {2, 2, 0, 0}}, {20, 8, 2, 1, {0, 0, 0, 0}}, {10, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_LAPLACIAN_PYRAMID_MID[] = {{24, 8, 3, 0, {0, 0, 2, 2}}, {24, 8, 4, 0, {2, 2, 0, 0}}, {24, 8, 2, 1, {0, 0, 0, 0}}, {12, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_LAPLACIAN_PYRAMID_MID[] = {{28, 8, 3, 0, {0, 0, 2, 2}}, {28, 8, 4, 0, {2, 2, 0, 0}}, {28, 8, 2, 1, {0, 0, 0, 0}}, {14, 4, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_LAPLACIAN_PYRAMID_MID[] = {{4, 10, 3, 0, {0, 0, 2, 2}}, {4, 10, 4, 0, {2, 2, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}, {2, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_LAPLACIAN_PYRAMID_MID[] = {{8, 10, 3, 0, {0, 0, 2, 2}}, {8, 10, 4, 0, {2, 2, 0, 0}}, {8, 10, 2, 1, {0, 0, 0, 0}}, {4, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_LAPLACIAN_PYRAMID_MID[] = {{12, 10, 3, 0, {0, 0, 2, 2}}, {12, 10, 4, 0, {2, 2, 0, 0}}, {12, 10, 2, 1, {0, 0, 0, 0}}, {6, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_LAPLACIAN_PYRAMID_MID[] = {{16, 10, 3, 0, {0, 0, 2, 2}}, {16, 10, 4, 0, {2, 2, 0, 0}}, {16, 10, 2, 1, {0, 0, 0, 0}}, {8, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_LAPLACIAN_PYRAMID_MID[] = {{20, 10, 3, 0, {0, 0, 2, 2}}, {20, 10, 4, 0, {2, 2, 0, 0}}, {20, 10, 2, 1, {0, 0, 0, 0}}, {10, 5, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_LAPLACIAN_PYRAMID_MID[] = {{4, 12, 3, 0, {0, 0, 2, 2}}, {4, 12, 4, 0, {2, 2, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}, {2, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_LAPLACIAN_PYRAMID_MID[] = {{8, 12, 3, 0, {0, 0, 2, 2}}, {8, 12, 4, 0, {2, 2, 0, 0}}, {8, 12, 2, 1, {0, 0, 0, 0}}, {4, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_LAPLACIAN_PYRAMID_MID[] = {{12, 12, 3, 0, {0, 0, 2, 2}}, {12, 12, 4, 0, {2, 2, 0, 0}}, {12, 12, 2, 1, {0, 0, 0, 0}}, {6, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_LAPLACIAN_PYRAMID_MID[] = {{16, 12, 3, 0, {0, 0, 2, 2}}, {16, 12, 4, 0, {2, 2, 0, 0}}, {16, 12, 2, 1, {0, 0, 0, 0}}, {8, 6, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_LAPLACIAN_PYRAMID_MID[] = {{4, 16, 3, 0, {0, 0, 2, 2}}, {4, 16, 4, 0, {2, 2, 0, 0}}, {4, 16, 2, 1, {0, 0, 0, 0}}, {2, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_LAPLACIAN_PYRAMID_MID[] = {{8, 16, 3, 0, {0, 0, 2, 2}}, {8, 16, 4, 0, {2, 2, 0, 0}}, {8, 16, 2, 1, {0, 0, 0, 0}}, {4, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_LAPLACIAN_PYRAMID_MID[] = {{12, 16, 3, 0, {0, 0, 2, 2}}, {12, 16, 4, 0, {2, 2, 0, 0}}, {12, 16, 2, 1, {0, 0, 0, 0}}, {6, 8, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_LAPLACIAN_PYRAMID_MID[] = {{4, 20, 3, 0, {0, 0, 2, 2}}, {4, 20, 4, 0, {2, 2, 0, 0}}, {4, 20, 2, 1, {0, 0, 0, 0}}, {2, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_LAPLACIAN_PYRAMID_MID[] = {{8, 20, 3, 0, {0, 0, 2, 2}}, {8, 20, 4, 0, {2, 2, 0, 0}}, {8, 20, 2, 1, {0, 0, 0, 0}}, {4, 10, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_LAPLACIAN_PYRAMID_MID[] = {{4, 24, 3, 0, {0, 0, 2, 2}}, {4, 24, 4, 0, {2, 2, 0, 0}}, {4, 24, 2, 1, {0, 0, 0, 0}}, {2, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_LAPLACIAN_PYRAMID_MID[] = {{8, 24, 3, 0, {0, 0, 2, 2}}, {8, 24, 4, 0, {2, 2, 0, 0}}, {8, 24, 2, 1, {0, 0, 0, 0}}, {4, 12, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_LAPLACIAN_PYRAMID_MID[] = {{4, 28, 3, 0, {0, 0, 2, 2}}, {4, 28, 4, 0, {2, 2, 0, 0}}, {4, 28, 2, 1, {0, 0, 0, 0}}, {2, 14, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_LAPLACIAN_PYRAMID_MID[] = {{4, 32, 3, 0, {0, 0, 2, 2}}, {4, 32, 4, 0, {2, 2, 0, 0}}, {4, 32, 2, 1, {0, 0, 0, 0}}, {2, 16, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_LAPLACIAN_PYRAMID_MID[] = {{4, 36, 3, 0, {0, 0, 2, 2}}, {4, 36, 4, 0, {2, 2, 0, 0}}, {4, 36, 2, 1, {0, 0, 0, 0}}, {2, 18, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_LAPLACIAN_PYRAMID_MID[] = {{4, 40, 3, 0, {0, 0, 2, 2}}, {4, 40, 4, 0, {2, 2, 0, 0}}, {4, 40, 2, 1, {0, 0, 0, 0}}, {2, 20, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_LAPLACIAN_PYRAMID_MID[] = {{4, 44, 3, 0, {0, 0, 2, 2}}, {4, 44, 4, 0, {2, 2, 0, 0}}, {4, 44, 2, 1, {0, 0, 0, 0}}, {2, 22, 2, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_LAPLACIAN_PYRAMID_MID[] = {{4, 48, 3, 0, {0, 0, 2, 2}}, {4, 48, 4, 0, {2, 2, 0, 0}}, {4, 48, 2, 1, {0, 0, 0, 0}}, {2, 24, 2, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_LAPLACIAN_PYRAMID_MID[] = {{0, 0}, {1, 0}};

static acf_scenario gScenarioArray_LAPLACIAN_PYRAMID_MID[] = {
{2, 1, 180, 0, 1, gScenarioBufferData0_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData0_LAPLACIAN_PYRAMID_MID, 8},
{4, 1, 336, 0, 1, gScenarioBufferData1_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData1_LAPLACIAN_PYRAMID_MID, 8},
{6, 1, 492, 0, 1, gScenarioBufferData2_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData2_LAPLACIAN_PYRAMID_MID, 8},
{8, 1, 648, 0, 1, gScenarioBufferData3_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData3_LAPLACIAN_PYRAMID_MID, 8},
{10, 1, 804, 0, 1, gScenarioBufferData4_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData4_LAPLACIAN_PYRAMID_MID, 8},
{12, 1, 960, 0, 1, gScenarioBufferData5_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData5_LAPLACIAN_PYRAMID_MID, 8},
{14, 1, 1116, 0, 1, gScenarioBufferData6_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData6_LAPLACIAN_PYRAMID_MID, 8},
{16, 1, 1272, 0, 1, gScenarioBufferData7_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData7_LAPLACIAN_PYRAMID_MID, 8},
{18, 1, 1428, 0, 1, gScenarioBufferData8_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData8_LAPLACIAN_PYRAMID_MID, 8},
{20, 1, 1584, 0, 1, gScenarioBufferData9_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData9_LAPLACIAN_PYRAMID_MID, 8},
{22, 1, 1740, 0, 1, gScenarioBufferData10_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData10_LAPLACIAN_PYRAMID_MID, 8},
{24, 1, 1896, 0, 1, gScenarioBufferData11_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData11_LAPLACIAN_PYRAMID_MID, 8},
{26, 1, 2052, 0, 1, gScenarioBufferData12_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData12_LAPLACIAN_PYRAMID_MID, 8},
{28, 1, 2208, 0, 1, gScenarioBufferData13_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData13_LAPLACIAN_PYRAMID_MID, 8},
{30, 1, 2364, 0, 1, gScenarioBufferData14_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData14_LAPLACIAN_PYRAMID_MID, 8},
{32, 1, 2520, 0, 1, gScenarioBufferData15_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData15_LAPLACIAN_PYRAMID_MID, 8},
{2, 2, 328, 0, 1, gScenarioBufferData16_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData16_LAPLACIAN_PYRAMID_MID, 8},
{4, 2, 608, 0, 1, gScenarioBufferData17_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData17_LAPLACIAN_PYRAMID_MID, 8},
{6, 2, 888, 0, 1, gScenarioBufferData18_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData18_LAPLACIAN_PYRAMID_MID, 8},
{8, 2, 1168, 0, 1, gScenarioBufferData19_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData19_LAPLACIAN_PYRAMID_MID, 8},
{10, 2, 1448, 0, 1, gScenarioBufferData20_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData20_LAPLACIAN_PYRAMID_MID, 8},
{12, 2, 1728, 0, 1, gScenarioBufferData21_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData21_LAPLACIAN_PYRAMID_MID, 8},
{14, 2, 2008, 0, 1, gScenarioBufferData22_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData22_LAPLACIAN_PYRAMID_MID, 8},
{16, 2, 2288, 0, 1, gScenarioBufferData23_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData23_LAPLACIAN_PYRAMID_MID, 8},
{18, 2, 2568, 0, 1, gScenarioBufferData24_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData24_LAPLACIAN_PYRAMID_MID, 8},
{20, 2, 2848, 0, 1, gScenarioBufferData25_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData25_LAPLACIAN_PYRAMID_MID, 8},
{22, 2, 3128, 0, 1, gScenarioBufferData26_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData26_LAPLACIAN_PYRAMID_MID, 8},
{24, 2, 3408, 0, 1, gScenarioBufferData27_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData27_LAPLACIAN_PYRAMID_MID, 8},
{26, 2, 3688, 0, 1, gScenarioBufferData28_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData28_LAPLACIAN_PYRAMID_MID, 8},
{2, 3, 476, 0, 1, gScenarioBufferData29_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData29_LAPLACIAN_PYRAMID_MID, 8},
{4, 3, 880, 0, 1, gScenarioBufferData30_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData30_LAPLACIAN_PYRAMID_MID, 8},
{6, 3, 1284, 0, 1, gScenarioBufferData31_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData31_LAPLACIAN_PYRAMID_MID, 8},
{8, 3, 1688, 0, 1, gScenarioBufferData32_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData32_LAPLACIAN_PYRAMID_MID, 8},
{10, 3, 2092, 0, 1, gScenarioBufferData33_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData33_LAPLACIAN_PYRAMID_MID, 8},
{12, 3, 2496, 0, 1, gScenarioBufferData34_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData34_LAPLACIAN_PYRAMID_MID, 8},
{14, 3, 2900, 0, 1, gScenarioBufferData35_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData35_LAPLACIAN_PYRAMID_MID, 8},
{16, 3, 3304, 0, 1, gScenarioBufferData36_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData36_LAPLACIAN_PYRAMID_MID, 8},
{18, 3, 3708, 0, 1, gScenarioBufferData37_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData37_LAPLACIAN_PYRAMID_MID, 8},
{2, 4, 624, 0, 1, gScenarioBufferData38_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData38_LAPLACIAN_PYRAMID_MID, 8},
{4, 4, 1152, 0, 1, gScenarioBufferData39_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData39_LAPLACIAN_PYRAMID_MID, 8},
{6, 4, 1680, 0, 1, gScenarioBufferData40_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData40_LAPLACIAN_PYRAMID_MID, 8},
{8, 4, 2208, 0, 1, gScenarioBufferData41_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData41_LAPLACIAN_PYRAMID_MID, 8},
{10, 4, 2736, 0, 1, gScenarioBufferData42_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData42_LAPLACIAN_PYRAMID_MID, 8},
{12, 4, 3264, 0, 1, gScenarioBufferData43_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData43_LAPLACIAN_PYRAMID_MID, 8},
{14, 4, 3792, 0, 1, gScenarioBufferData44_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData44_LAPLACIAN_PYRAMID_MID, 8},
{2, 5, 772, 0, 1, gScenarioBufferData45_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData45_LAPLACIAN_PYRAMID_MID, 8},
{4, 5, 1424, 0, 1, gScenarioBufferData46_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData46_LAPLACIAN_PYRAMID_MID, 8},
{6, 5, 2076, 0, 1, gScenarioBufferData47_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData47_LAPLACIAN_PYRAMID_MID, 8},
{8, 5, 2728, 0, 1, gScenarioBufferData48_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData48_LAPLACIAN_PYRAMID_MID, 8},
{10, 5, 3380, 0, 1, gScenarioBufferData49_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData49_LAPLACIAN_PYRAMID_MID, 8},
{2, 6, 920, 0, 1, gScenarioBufferData50_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData50_LAPLACIAN_PYRAMID_MID, 8},
{4, 6, 1696, 0, 1, gScenarioBufferData51_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData51_LAPLACIAN_PYRAMID_MID, 8},
{6, 6, 2472, 0, 1, gScenarioBufferData52_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData52_LAPLACIAN_PYRAMID_MID, 8},
{8, 6, 3248, 0, 1, gScenarioBufferData53_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData53_LAPLACIAN_PYRAMID_MID, 8},
{2, 8, 1216, 0, 1, gScenarioBufferData54_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData54_LAPLACIAN_PYRAMID_MID, 8},
{4, 8, 2240, 0, 1, gScenarioBufferData55_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData55_LAPLACIAN_PYRAMID_MID, 8},
{6, 8, 3264, 0, 1, gScenarioBufferData56_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData56_LAPLACIAN_PYRAMID_MID, 8},
{2, 10, 1512, 0, 1, gScenarioBufferData57_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData57_LAPLACIAN_PYRAMID_MID, 8},
{4, 10, 2784, 0, 1, gScenarioBufferData58_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData58_LAPLACIAN_PYRAMID_MID, 8},
{2, 12, 1808, 0, 1, gScenarioBufferData59_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData59_LAPLACIAN_PYRAMID_MID, 8},
{4, 12, 3328, 0, 1, gScenarioBufferData60_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData60_LAPLACIAN_PYRAMID_MID, 8},
{2, 14, 2104, 0, 1, gScenarioBufferData61_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData61_LAPLACIAN_PYRAMID_MID, 8},
{2, 16, 2400, 0, 1, gScenarioBufferData62_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData62_LAPLACIAN_PYRAMID_MID, 8},
{2, 18, 2696, 0, 1, gScenarioBufferData63_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData63_LAPLACIAN_PYRAMID_MID, 8},
{2, 20, 2992, 0, 1, gScenarioBufferData64_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData64_LAPLACIAN_PYRAMID_MID, 8},
{2, 22, 3288, 0, 1, gScenarioBufferData65_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData65_LAPLACIAN_PYRAMID_MID, 8},
{2, 24, 3584, 0, 1, gScenarioBufferData66_LAPLACIAN_PYRAMID_MID, 64, gScenarioKernelData66_LAPLACIAN_PYRAMID_MID, 8}
};

static acf_scenario_list gScenarioList_LAPLACIAN_PYRAMID_MID = {
67, //number of scenarios
gScenarioArray_LAPLACIAN_PYRAMID_MID};
//**************************************************************

class LAPLACIAN_PYRAMID_MID : public ACF_Process_APU
{

public:
   LAPLACIAN_PYRAMID_MID(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("LAPLACIAN_PYRAMID_MID");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("LAPLACIAN_PYRAMID_MID",
                                        LAPLACIAN_PYRAMID_MID_LOAD_SEGMENTS,
                                        LAPLACIAN_PYRAMID_MID_LOAD_PMEM, LAPLACIAN_PYRAMID_MID_LOAD_PMEM_SIZE,
                                        LAPLACIAN_PYRAMID_MID_LOAD_DMEM, LAPLACIAN_PYRAMID_MID_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(LAPLACIAN_PYRAMID_MID_APEX_LOG_BUFFER, LAPLACIAN_PYRAMID_MID_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 2, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0);
         AddPort("OUTPUT_PYRAMID", icp::DATATYPE_16S, 1, 1, 4, 2, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_NEXT", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_LAPLACIAN_PYRAMID_MID);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID

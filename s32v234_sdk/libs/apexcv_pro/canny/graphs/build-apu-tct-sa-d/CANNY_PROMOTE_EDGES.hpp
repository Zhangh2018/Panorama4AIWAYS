#ifndef _ACF_PROCESS_APU_CANNY_PROMOTE_EDGES
#define _ACF_PROCESS_APU_CANNY_PROMOTE_EDGES

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <CANNY_PROMOTE_EDGES_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_CANNY_PROMOTE_EDGES[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 4, 1, {1, 1, 2, 2}}, {4, 1, 4, 2, {1, 1, 2, 2}}, {4, 1, 4, 3, {1, 1, 2, 2}}, {4, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_CANNY_PROMOTE_EDGES[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 4, 1, {1, 1, 2, 2}}, {8, 1, 4, 2, {1, 1, 2, 2}}, {8, 1, 4, 3, {1, 1, 2, 2}}, {8, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_CANNY_PROMOTE_EDGES[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 4, 1, {1, 1, 2, 2}}, {12, 1, 4, 2, {1, 1, 2, 2}}, {12, 1, 4, 3, {1, 1, 2, 2}}, {12, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_CANNY_PROMOTE_EDGES[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 4, 1, {1, 1, 2, 2}}, {16, 1, 4, 2, {1, 1, 2, 2}}, {16, 1, 4, 3, {1, 1, 2, 2}}, {16, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_CANNY_PROMOTE_EDGES[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 4, 1, {1, 1, 2, 2}}, {20, 1, 4, 2, {1, 1, 2, 2}}, {20, 1, 4, 3, {1, 1, 2, 2}}, {20, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_CANNY_PROMOTE_EDGES[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 4, 1, {1, 1, 2, 2}}, {24, 1, 4, 2, {1, 1, 2, 2}}, {24, 1, 4, 3, {1, 1, 2, 2}}, {24, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_CANNY_PROMOTE_EDGES[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 4, 1, {1, 1, 2, 2}}, {28, 1, 4, 2, {1, 1, 2, 2}}, {28, 1, 4, 3, {1, 1, 2, 2}}, {28, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_CANNY_PROMOTE_EDGES[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 4, 1, {1, 1, 2, 2}}, {32, 1, 4, 2, {1, 1, 2, 2}}, {32, 1, 4, 3, {1, 1, 2, 2}}, {32, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_CANNY_PROMOTE_EDGES[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 4, 1, {1, 1, 2, 2}}, {36, 1, 4, 2, {1, 1, 2, 2}}, {36, 1, 4, 3, {1, 1, 2, 2}}, {36, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_CANNY_PROMOTE_EDGES[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 4, 1, {1, 1, 2, 2}}, {40, 1, 4, 2, {1, 1, 2, 2}}, {40, 1, 4, 3, {1, 1, 2, 2}}, {40, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_CANNY_PROMOTE_EDGES[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 4, 1, {1, 1, 2, 2}}, {44, 1, 4, 2, {1, 1, 2, 2}}, {44, 1, 4, 3, {1, 1, 2, 2}}, {44, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_CANNY_PROMOTE_EDGES[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 4, 1, {1, 1, 2, 2}}, {48, 1, 4, 2, {1, 1, 2, 2}}, {48, 1, 4, 3, {1, 1, 2, 2}}, {48, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_CANNY_PROMOTE_EDGES[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 4, 1, {1, 1, 2, 2}}, {52, 1, 4, 2, {1, 1, 2, 2}}, {52, 1, 4, 3, {1, 1, 2, 2}}, {52, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_CANNY_PROMOTE_EDGES[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 4, 1, {1, 1, 2, 2}}, {56, 1, 4, 2, {1, 1, 2, 2}}, {56, 1, 4, 3, {1, 1, 2, 2}}, {56, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_CANNY_PROMOTE_EDGES[] = {{60, 1, 4, 0, {1, 1, 2, 2}}, {60, 1, 4, 1, {1, 1, 2, 2}}, {60, 1, 4, 2, {1, 1, 2, 2}}, {60, 1, 4, 3, {1, 1, 2, 2}}, {60, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_CANNY_PROMOTE_EDGES[] = {{64, 1, 4, 0, {1, 1, 2, 2}}, {64, 1, 4, 1, {1, 1, 2, 2}}, {64, 1, 4, 2, {1, 1, 2, 2}}, {64, 1, 4, 3, {1, 1, 2, 2}}, {64, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_CANNY_PROMOTE_EDGES[] = {{72, 1, 4, 0, {1, 1, 2, 2}}, {72, 1, 4, 1, {1, 1, 2, 2}}, {72, 1, 4, 2, {1, 1, 2, 2}}, {72, 1, 4, 3, {1, 1, 2, 2}}, {72, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_CANNY_PROMOTE_EDGES[] = {{80, 1, 4, 0, {1, 1, 2, 2}}, {80, 1, 4, 1, {1, 1, 2, 2}}, {80, 1, 4, 2, {1, 1, 2, 2}}, {80, 1, 4, 3, {1, 1, 2, 2}}, {80, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_CANNY_PROMOTE_EDGES[] = {{88, 1, 4, 0, {1, 1, 2, 2}}, {88, 1, 4, 1, {1, 1, 2, 2}}, {88, 1, 4, 2, {1, 1, 2, 2}}, {88, 1, 4, 3, {1, 1, 2, 2}}, {88, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_CANNY_PROMOTE_EDGES[] = {{96, 1, 4, 0, {1, 1, 2, 2}}, {96, 1, 4, 1, {1, 1, 2, 2}}, {96, 1, 4, 2, {1, 1, 2, 2}}, {96, 1, 4, 3, {1, 1, 2, 2}}, {96, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_CANNY_PROMOTE_EDGES[] = {{104, 1, 4, 0, {1, 1, 2, 2}}, {104, 1, 4, 1, {1, 1, 2, 2}}, {104, 1, 4, 2, {1, 1, 2, 2}}, {104, 1, 4, 3, {1, 1, 2, 2}}, {104, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_CANNY_PROMOTE_EDGES[] = {{112, 1, 4, 0, {1, 1, 2, 2}}, {112, 1, 4, 1, {1, 1, 2, 2}}, {112, 1, 4, 2, {1, 1, 2, 2}}, {112, 1, 4, 3, {1, 1, 2, 2}}, {112, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_CANNY_PROMOTE_EDGES[] = {{120, 1, 4, 0, {1, 1, 2, 2}}, {120, 1, 4, 1, {1, 1, 2, 2}}, {120, 1, 4, 2, {1, 1, 2, 2}}, {120, 1, 4, 3, {1, 1, 2, 2}}, {120, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_CANNY_PROMOTE_EDGES[] = {{128, 1, 4, 0, {1, 1, 2, 2}}, {128, 1, 4, 1, {1, 1, 2, 2}}, {128, 1, 4, 2, {1, 1, 2, 2}}, {128, 1, 4, 3, {1, 1, 2, 2}}, {128, 1, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_CANNY_PROMOTE_EDGES[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 4, 1, {1, 1, 2, 2}}, {4, 2, 4, 2, {1, 1, 2, 2}}, {4, 2, 4, 3, {1, 1, 2, 2}}, {4, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_CANNY_PROMOTE_EDGES[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 4, 1, {1, 1, 2, 2}}, {8, 2, 4, 2, {1, 1, 2, 2}}, {8, 2, 4, 3, {1, 1, 2, 2}}, {8, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_CANNY_PROMOTE_EDGES[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 4, 1, {1, 1, 2, 2}}, {12, 2, 4, 2, {1, 1, 2, 2}}, {12, 2, 4, 3, {1, 1, 2, 2}}, {12, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_CANNY_PROMOTE_EDGES[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 4, 1, {1, 1, 2, 2}}, {16, 2, 4, 2, {1, 1, 2, 2}}, {16, 2, 4, 3, {1, 1, 2, 2}}, {16, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_CANNY_PROMOTE_EDGES[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 4, 1, {1, 1, 2, 2}}, {20, 2, 4, 2, {1, 1, 2, 2}}, {20, 2, 4, 3, {1, 1, 2, 2}}, {20, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_CANNY_PROMOTE_EDGES[] = {{24, 2, 4, 0, {1, 1, 2, 2}}, {24, 2, 4, 1, {1, 1, 2, 2}}, {24, 2, 4, 2, {1, 1, 2, 2}}, {24, 2, 4, 3, {1, 1, 2, 2}}, {24, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_CANNY_PROMOTE_EDGES[] = {{28, 2, 4, 0, {1, 1, 2, 2}}, {28, 2, 4, 1, {1, 1, 2, 2}}, {28, 2, 4, 2, {1, 1, 2, 2}}, {28, 2, 4, 3, {1, 1, 2, 2}}, {28, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_CANNY_PROMOTE_EDGES[] = {{32, 2, 4, 0, {1, 1, 2, 2}}, {32, 2, 4, 1, {1, 1, 2, 2}}, {32, 2, 4, 2, {1, 1, 2, 2}}, {32, 2, 4, 3, {1, 1, 2, 2}}, {32, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_CANNY_PROMOTE_EDGES[] = {{36, 2, 4, 0, {1, 1, 2, 2}}, {36, 2, 4, 1, {1, 1, 2, 2}}, {36, 2, 4, 2, {1, 1, 2, 2}}, {36, 2, 4, 3, {1, 1, 2, 2}}, {36, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_CANNY_PROMOTE_EDGES[] = {{40, 2, 4, 0, {1, 1, 2, 2}}, {40, 2, 4, 1, {1, 1, 2, 2}}, {40, 2, 4, 2, {1, 1, 2, 2}}, {40, 2, 4, 3, {1, 1, 2, 2}}, {40, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_CANNY_PROMOTE_EDGES[] = {{44, 2, 4, 0, {1, 1, 2, 2}}, {44, 2, 4, 1, {1, 1, 2, 2}}, {44, 2, 4, 2, {1, 1, 2, 2}}, {44, 2, 4, 3, {1, 1, 2, 2}}, {44, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_CANNY_PROMOTE_EDGES[] = {{48, 2, 4, 0, {1, 1, 2, 2}}, {48, 2, 4, 1, {1, 1, 2, 2}}, {48, 2, 4, 2, {1, 1, 2, 2}}, {48, 2, 4, 3, {1, 1, 2, 2}}, {48, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_CANNY_PROMOTE_EDGES[] = {{52, 2, 4, 0, {1, 1, 2, 2}}, {52, 2, 4, 1, {1, 1, 2, 2}}, {52, 2, 4, 2, {1, 1, 2, 2}}, {52, 2, 4, 3, {1, 1, 2, 2}}, {52, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_CANNY_PROMOTE_EDGES[] = {{56, 2, 4, 0, {1, 1, 2, 2}}, {56, 2, 4, 1, {1, 1, 2, 2}}, {56, 2, 4, 2, {1, 1, 2, 2}}, {56, 2, 4, 3, {1, 1, 2, 2}}, {56, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_CANNY_PROMOTE_EDGES[] = {{60, 2, 4, 0, {1, 1, 2, 2}}, {60, 2, 4, 1, {1, 1, 2, 2}}, {60, 2, 4, 2, {1, 1, 2, 2}}, {60, 2, 4, 3, {1, 1, 2, 2}}, {60, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_CANNY_PROMOTE_EDGES[] = {{64, 2, 4, 0, {1, 1, 2, 2}}, {64, 2, 4, 1, {1, 1, 2, 2}}, {64, 2, 4, 2, {1, 1, 2, 2}}, {64, 2, 4, 3, {1, 1, 2, 2}}, {64, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_CANNY_PROMOTE_EDGES[] = {{72, 2, 4, 0, {1, 1, 2, 2}}, {72, 2, 4, 1, {1, 1, 2, 2}}, {72, 2, 4, 2, {1, 1, 2, 2}}, {72, 2, 4, 3, {1, 1, 2, 2}}, {72, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_CANNY_PROMOTE_EDGES[] = {{80, 2, 4, 0, {1, 1, 2, 2}}, {80, 2, 4, 1, {1, 1, 2, 2}}, {80, 2, 4, 2, {1, 1, 2, 2}}, {80, 2, 4, 3, {1, 1, 2, 2}}, {80, 2, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_CANNY_PROMOTE_EDGES[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 4, 1, {1, 1, 2, 2}}, {4, 3, 4, 2, {1, 1, 2, 2}}, {4, 3, 4, 3, {1, 1, 2, 2}}, {4, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_CANNY_PROMOTE_EDGES[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 4, 1, {1, 1, 2, 2}}, {8, 3, 4, 2, {1, 1, 2, 2}}, {8, 3, 4, 3, {1, 1, 2, 2}}, {8, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_CANNY_PROMOTE_EDGES[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 4, 1, {1, 1, 2, 2}}, {12, 3, 4, 2, {1, 1, 2, 2}}, {12, 3, 4, 3, {1, 1, 2, 2}}, {12, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_CANNY_PROMOTE_EDGES[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 4, 1, {1, 1, 2, 2}}, {16, 3, 4, 2, {1, 1, 2, 2}}, {16, 3, 4, 3, {1, 1, 2, 2}}, {16, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_CANNY_PROMOTE_EDGES[] = {{20, 3, 4, 0, {1, 1, 2, 2}}, {20, 3, 4, 1, {1, 1, 2, 2}}, {20, 3, 4, 2, {1, 1, 2, 2}}, {20, 3, 4, 3, {1, 1, 2, 2}}, {20, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_CANNY_PROMOTE_EDGES[] = {{24, 3, 4, 0, {1, 1, 2, 2}}, {24, 3, 4, 1, {1, 1, 2, 2}}, {24, 3, 4, 2, {1, 1, 2, 2}}, {24, 3, 4, 3, {1, 1, 2, 2}}, {24, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_CANNY_PROMOTE_EDGES[] = {{28, 3, 4, 0, {1, 1, 2, 2}}, {28, 3, 4, 1, {1, 1, 2, 2}}, {28, 3, 4, 2, {1, 1, 2, 2}}, {28, 3, 4, 3, {1, 1, 2, 2}}, {28, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_CANNY_PROMOTE_EDGES[] = {{32, 3, 4, 0, {1, 1, 2, 2}}, {32, 3, 4, 1, {1, 1, 2, 2}}, {32, 3, 4, 2, {1, 1, 2, 2}}, {32, 3, 4, 3, {1, 1, 2, 2}}, {32, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_CANNY_PROMOTE_EDGES[] = {{36, 3, 4, 0, {1, 1, 2, 2}}, {36, 3, 4, 1, {1, 1, 2, 2}}, {36, 3, 4, 2, {1, 1, 2, 2}}, {36, 3, 4, 3, {1, 1, 2, 2}}, {36, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_CANNY_PROMOTE_EDGES[] = {{40, 3, 4, 0, {1, 1, 2, 2}}, {40, 3, 4, 1, {1, 1, 2, 2}}, {40, 3, 4, 2, {1, 1, 2, 2}}, {40, 3, 4, 3, {1, 1, 2, 2}}, {40, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_CANNY_PROMOTE_EDGES[] = {{44, 3, 4, 0, {1, 1, 2, 2}}, {44, 3, 4, 1, {1, 1, 2, 2}}, {44, 3, 4, 2, {1, 1, 2, 2}}, {44, 3, 4, 3, {1, 1, 2, 2}}, {44, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_CANNY_PROMOTE_EDGES[] = {{48, 3, 4, 0, {1, 1, 2, 2}}, {48, 3, 4, 1, {1, 1, 2, 2}}, {48, 3, 4, 2, {1, 1, 2, 2}}, {48, 3, 4, 3, {1, 1, 2, 2}}, {48, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_CANNY_PROMOTE_EDGES[] = {{52, 3, 4, 0, {1, 1, 2, 2}}, {52, 3, 4, 1, {1, 1, 2, 2}}, {52, 3, 4, 2, {1, 1, 2, 2}}, {52, 3, 4, 3, {1, 1, 2, 2}}, {52, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_CANNY_PROMOTE_EDGES[] = {{56, 3, 4, 0, {1, 1, 2, 2}}, {56, 3, 4, 1, {1, 1, 2, 2}}, {56, 3, 4, 2, {1, 1, 2, 2}}, {56, 3, 4, 3, {1, 1, 2, 2}}, {56, 3, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_CANNY_PROMOTE_EDGES[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 4, 1, {1, 1, 2, 2}}, {4, 4, 4, 2, {1, 1, 2, 2}}, {4, 4, 4, 3, {1, 1, 2, 2}}, {4, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_CANNY_PROMOTE_EDGES[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 4, 1, {1, 1, 2, 2}}, {8, 4, 4, 2, {1, 1, 2, 2}}, {8, 4, 4, 3, {1, 1, 2, 2}}, {8, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_CANNY_PROMOTE_EDGES[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 4, 1, {1, 1, 2, 2}}, {12, 4, 4, 2, {1, 1, 2, 2}}, {12, 4, 4, 3, {1, 1, 2, 2}}, {12, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_CANNY_PROMOTE_EDGES[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 4, 1, {1, 1, 2, 2}}, {16, 4, 4, 2, {1, 1, 2, 2}}, {16, 4, 4, 3, {1, 1, 2, 2}}, {16, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_CANNY_PROMOTE_EDGES[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 4, 1, {1, 1, 2, 2}}, {20, 4, 4, 2, {1, 1, 2, 2}}, {20, 4, 4, 3, {1, 1, 2, 2}}, {20, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_CANNY_PROMOTE_EDGES[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 4, 1, {1, 1, 2, 2}}, {24, 4, 4, 2, {1, 1, 2, 2}}, {24, 4, 4, 3, {1, 1, 2, 2}}, {24, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_CANNY_PROMOTE_EDGES[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 4, 1, {1, 1, 2, 2}}, {28, 4, 4, 2, {1, 1, 2, 2}}, {28, 4, 4, 3, {1, 1, 2, 2}}, {28, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_CANNY_PROMOTE_EDGES[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 4, 1, {1, 1, 2, 2}}, {32, 4, 4, 2, {1, 1, 2, 2}}, {32, 4, 4, 3, {1, 1, 2, 2}}, {32, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_CANNY_PROMOTE_EDGES[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 4, 1, {1, 1, 2, 2}}, {36, 4, 4, 2, {1, 1, 2, 2}}, {36, 4, 4, 3, {1, 1, 2, 2}}, {36, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_CANNY_PROMOTE_EDGES[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 4, 1, {1, 1, 2, 2}}, {40, 4, 4, 2, {1, 1, 2, 2}}, {40, 4, 4, 3, {1, 1, 2, 2}}, {40, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_CANNY_PROMOTE_EDGES[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 4, 1, {1, 1, 2, 2}}, {44, 4, 4, 2, {1, 1, 2, 2}}, {44, 4, 4, 3, {1, 1, 2, 2}}, {44, 4, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_CANNY_PROMOTE_EDGES[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 4, 1, {1, 1, 2, 2}}, {4, 5, 4, 2, {1, 1, 2, 2}}, {4, 5, 4, 3, {1, 1, 2, 2}}, {4, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_CANNY_PROMOTE_EDGES[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 4, 1, {1, 1, 2, 2}}, {8, 5, 4, 2, {1, 1, 2, 2}}, {8, 5, 4, 3, {1, 1, 2, 2}}, {8, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_CANNY_PROMOTE_EDGES[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 4, 1, {1, 1, 2, 2}}, {12, 5, 4, 2, {1, 1, 2, 2}}, {12, 5, 4, 3, {1, 1, 2, 2}}, {12, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_CANNY_PROMOTE_EDGES[] = {{16, 5, 4, 0, {1, 1, 2, 2}}, {16, 5, 4, 1, {1, 1, 2, 2}}, {16, 5, 4, 2, {1, 1, 2, 2}}, {16, 5, 4, 3, {1, 1, 2, 2}}, {16, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_CANNY_PROMOTE_EDGES[] = {{20, 5, 4, 0, {1, 1, 2, 2}}, {20, 5, 4, 1, {1, 1, 2, 2}}, {20, 5, 4, 2, {1, 1, 2, 2}}, {20, 5, 4, 3, {1, 1, 2, 2}}, {20, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_CANNY_PROMOTE_EDGES[] = {{24, 5, 4, 0, {1, 1, 2, 2}}, {24, 5, 4, 1, {1, 1, 2, 2}}, {24, 5, 4, 2, {1, 1, 2, 2}}, {24, 5, 4, 3, {1, 1, 2, 2}}, {24, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_CANNY_PROMOTE_EDGES[] = {{28, 5, 4, 0, {1, 1, 2, 2}}, {28, 5, 4, 1, {1, 1, 2, 2}}, {28, 5, 4, 2, {1, 1, 2, 2}}, {28, 5, 4, 3, {1, 1, 2, 2}}, {28, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_CANNY_PROMOTE_EDGES[] = {{32, 5, 4, 0, {1, 1, 2, 2}}, {32, 5, 4, 1, {1, 1, 2, 2}}, {32, 5, 4, 2, {1, 1, 2, 2}}, {32, 5, 4, 3, {1, 1, 2, 2}}, {32, 5, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_CANNY_PROMOTE_EDGES[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 4, 1, {1, 1, 2, 2}}, {4, 6, 4, 2, {1, 1, 2, 2}}, {4, 6, 4, 3, {1, 1, 2, 2}}, {4, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_CANNY_PROMOTE_EDGES[] = {{8, 6, 4, 0, {1, 1, 2, 2}}, {8, 6, 4, 1, {1, 1, 2, 2}}, {8, 6, 4, 2, {1, 1, 2, 2}}, {8, 6, 4, 3, {1, 1, 2, 2}}, {8, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_CANNY_PROMOTE_EDGES[] = {{12, 6, 4, 0, {1, 1, 2, 2}}, {12, 6, 4, 1, {1, 1, 2, 2}}, {12, 6, 4, 2, {1, 1, 2, 2}}, {12, 6, 4, 3, {1, 1, 2, 2}}, {12, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_CANNY_PROMOTE_EDGES[] = {{16, 6, 4, 0, {1, 1, 2, 2}}, {16, 6, 4, 1, {1, 1, 2, 2}}, {16, 6, 4, 2, {1, 1, 2, 2}}, {16, 6, 4, 3, {1, 1, 2, 2}}, {16, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_CANNY_PROMOTE_EDGES[] = {{20, 6, 4, 0, {1, 1, 2, 2}}, {20, 6, 4, 1, {1, 1, 2, 2}}, {20, 6, 4, 2, {1, 1, 2, 2}}, {20, 6, 4, 3, {1, 1, 2, 2}}, {20, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_CANNY_PROMOTE_EDGES[] = {{24, 6, 4, 0, {1, 1, 2, 2}}, {24, 6, 4, 1, {1, 1, 2, 2}}, {24, 6, 4, 2, {1, 1, 2, 2}}, {24, 6, 4, 3, {1, 1, 2, 2}}, {24, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_CANNY_PROMOTE_EDGES[] = {{28, 6, 4, 0, {1, 1, 2, 2}}, {28, 6, 4, 1, {1, 1, 2, 2}}, {28, 6, 4, 2, {1, 1, 2, 2}}, {28, 6, 4, 3, {1, 1, 2, 2}}, {28, 6, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_CANNY_PROMOTE_EDGES[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 4, 1, {1, 1, 2, 2}}, {4, 8, 4, 2, {1, 1, 2, 2}}, {4, 8, 4, 3, {1, 1, 2, 2}}, {4, 8, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_CANNY_PROMOTE_EDGES[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 4, 1, {1, 1, 2, 2}}, {8, 8, 4, 2, {1, 1, 2, 2}}, {8, 8, 4, 3, {1, 1, 2, 2}}, {8, 8, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_CANNY_PROMOTE_EDGES[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 4, 1, {1, 1, 2, 2}}, {12, 8, 4, 2, {1, 1, 2, 2}}, {12, 8, 4, 3, {1, 1, 2, 2}}, {12, 8, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_CANNY_PROMOTE_EDGES[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 4, 1, {1, 1, 2, 2}}, {16, 8, 4, 2, {1, 1, 2, 2}}, {16, 8, 4, 3, {1, 1, 2, 2}}, {16, 8, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_CANNY_PROMOTE_EDGES[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 4, 1, {1, 1, 2, 2}}, {20, 8, 4, 2, {1, 1, 2, 2}}, {20, 8, 4, 3, {1, 1, 2, 2}}, {20, 8, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_CANNY_PROMOTE_EDGES[] = {{4, 10, 4, 0, {1, 1, 2, 2}}, {4, 10, 4, 1, {1, 1, 2, 2}}, {4, 10, 4, 2, {1, 1, 2, 2}}, {4, 10, 4, 3, {1, 1, 2, 2}}, {4, 10, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_CANNY_PROMOTE_EDGES[] = {{8, 10, 4, 0, {1, 1, 2, 2}}, {8, 10, 4, 1, {1, 1, 2, 2}}, {8, 10, 4, 2, {1, 1, 2, 2}}, {8, 10, 4, 3, {1, 1, 2, 2}}, {8, 10, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_CANNY_PROMOTE_EDGES[] = {{12, 10, 4, 0, {1, 1, 2, 2}}, {12, 10, 4, 1, {1, 1, 2, 2}}, {12, 10, 4, 2, {1, 1, 2, 2}}, {12, 10, 4, 3, {1, 1, 2, 2}}, {12, 10, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_CANNY_PROMOTE_EDGES[] = {{16, 10, 4, 0, {1, 1, 2, 2}}, {16, 10, 4, 1, {1, 1, 2, 2}}, {16, 10, 4, 2, {1, 1, 2, 2}}, {16, 10, 4, 3, {1, 1, 2, 2}}, {16, 10, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_CANNY_PROMOTE_EDGES[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 4, 1, {1, 1, 2, 2}}, {4, 12, 4, 2, {1, 1, 2, 2}}, {4, 12, 4, 3, {1, 1, 2, 2}}, {4, 12, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_CANNY_PROMOTE_EDGES[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 4, 1, {1, 1, 2, 2}}, {8, 12, 4, 2, {1, 1, 2, 2}}, {8, 12, 4, 3, {1, 1, 2, 2}}, {8, 12, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_CANNY_PROMOTE_EDGES[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 4, 1, {1, 1, 2, 2}}, {12, 12, 4, 2, {1, 1, 2, 2}}, {12, 12, 4, 3, {1, 1, 2, 2}}, {12, 12, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_CANNY_PROMOTE_EDGES[] = {{4, 14, 4, 0, {1, 1, 2, 2}}, {4, 14, 4, 1, {1, 1, 2, 2}}, {4, 14, 4, 2, {1, 1, 2, 2}}, {4, 14, 4, 3, {1, 1, 2, 2}}, {4, 14, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_CANNY_PROMOTE_EDGES[] = {{8, 14, 4, 0, {1, 1, 2, 2}}, {8, 14, 4, 1, {1, 1, 2, 2}}, {8, 14, 4, 2, {1, 1, 2, 2}}, {8, 14, 4, 3, {1, 1, 2, 2}}, {8, 14, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_CANNY_PROMOTE_EDGES[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 4, 1, {1, 1, 2, 2}}, {4, 16, 4, 2, {1, 1, 2, 2}}, {4, 16, 4, 3, {1, 1, 2, 2}}, {4, 16, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_CANNY_PROMOTE_EDGES[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 4, 1, {1, 1, 2, 2}}, {8, 16, 4, 2, {1, 1, 2, 2}}, {8, 16, 4, 3, {1, 1, 2, 2}}, {8, 16, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_CANNY_PROMOTE_EDGES[] = {{4, 18, 4, 0, {1, 1, 2, 2}}, {4, 18, 4, 1, {1, 1, 2, 2}}, {4, 18, 4, 2, {1, 1, 2, 2}}, {4, 18, 4, 3, {1, 1, 2, 2}}, {4, 18, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_CANNY_PROMOTE_EDGES[] = {{8, 18, 4, 0, {1, 1, 2, 2}}, {8, 18, 4, 1, {1, 1, 2, 2}}, {8, 18, 4, 2, {1, 1, 2, 2}}, {8, 18, 4, 3, {1, 1, 2, 2}}, {8, 18, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_CANNY_PROMOTE_EDGES[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 4, 1, {1, 1, 2, 2}}, {4, 20, 4, 2, {1, 1, 2, 2}}, {4, 20, 4, 3, {1, 1, 2, 2}}, {4, 20, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_CANNY_PROMOTE_EDGES[] = {{4, 22, 4, 0, {1, 1, 2, 2}}, {4, 22, 4, 1, {1, 1, 2, 2}}, {4, 22, 4, 2, {1, 1, 2, 2}}, {4, 22, 4, 3, {1, 1, 2, 2}}, {4, 22, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_CANNY_PROMOTE_EDGES[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 4, 1, {1, 1, 2, 2}}, {4, 24, 4, 2, {1, 1, 2, 2}}, {4, 24, 4, 3, {1, 1, 2, 2}}, {4, 24, 2, 4, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_CANNY_PROMOTE_EDGES[] = {{4, 26, 4, 0, {1, 1, 2, 2}}, {4, 26, 4, 1, {1, 1, 2, 2}}, {4, 26, 4, 2, {1, 1, 2, 2}}, {4, 26, 4, 3, {1, 1, 2, 2}}, {4, 26, 2, 4, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_CANNY_PROMOTE_EDGES[] = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};

static acf_scenario gScenarioArray_CANNY_PROMOTE_EDGES[] = {
{1, 1, 200, 0, 4, gScenarioBufferData0_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData0_CANNY_PROMOTE_EDGES, 16},
{2, 1, 304, 0, 4, gScenarioBufferData1_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData1_CANNY_PROMOTE_EDGES, 16},
{3, 1, 408, 0, 4, gScenarioBufferData2_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData2_CANNY_PROMOTE_EDGES, 16},
{4, 1, 512, 0, 4, gScenarioBufferData3_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData3_CANNY_PROMOTE_EDGES, 16},
{5, 1, 616, 0, 4, gScenarioBufferData4_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData4_CANNY_PROMOTE_EDGES, 16},
{6, 1, 720, 0, 4, gScenarioBufferData5_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData5_CANNY_PROMOTE_EDGES, 16},
{7, 1, 824, 0, 4, gScenarioBufferData6_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData6_CANNY_PROMOTE_EDGES, 16},
{8, 1, 928, 0, 4, gScenarioBufferData7_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData7_CANNY_PROMOTE_EDGES, 16},
{9, 1, 1032, 0, 4, gScenarioBufferData8_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData8_CANNY_PROMOTE_EDGES, 16},
{10, 1, 1136, 0, 4, gScenarioBufferData9_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData9_CANNY_PROMOTE_EDGES, 16},
{11, 1, 1240, 0, 4, gScenarioBufferData10_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData10_CANNY_PROMOTE_EDGES, 16},
{12, 1, 1344, 0, 4, gScenarioBufferData11_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData11_CANNY_PROMOTE_EDGES, 16},
{13, 1, 1448, 0, 4, gScenarioBufferData12_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData12_CANNY_PROMOTE_EDGES, 16},
{14, 1, 1552, 0, 4, gScenarioBufferData13_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData13_CANNY_PROMOTE_EDGES, 16},
{15, 1, 1656, 0, 4, gScenarioBufferData14_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData14_CANNY_PROMOTE_EDGES, 16},
{16, 1, 1760, 0, 4, gScenarioBufferData15_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData15_CANNY_PROMOTE_EDGES, 16},
{18, 1, 1968, 0, 4, gScenarioBufferData16_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData16_CANNY_PROMOTE_EDGES, 16},
{20, 1, 2176, 0, 4, gScenarioBufferData17_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData17_CANNY_PROMOTE_EDGES, 16},
{22, 1, 2384, 0, 4, gScenarioBufferData18_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData18_CANNY_PROMOTE_EDGES, 16},
{24, 1, 2592, 0, 4, gScenarioBufferData19_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData19_CANNY_PROMOTE_EDGES, 16},
{26, 1, 2800, 0, 4, gScenarioBufferData20_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData20_CANNY_PROMOTE_EDGES, 16},
{28, 1, 3008, 0, 4, gScenarioBufferData21_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData21_CANNY_PROMOTE_EDGES, 16},
{30, 1, 3216, 0, 4, gScenarioBufferData22_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData22_CANNY_PROMOTE_EDGES, 16},
{32, 1, 3424, 0, 4, gScenarioBufferData23_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData23_CANNY_PROMOTE_EDGES, 16},
{1, 2, 336, 0, 4, gScenarioBufferData24_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData24_CANNY_PROMOTE_EDGES, 16},
{2, 2, 512, 0, 4, gScenarioBufferData25_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData25_CANNY_PROMOTE_EDGES, 16},
{3, 2, 688, 0, 4, gScenarioBufferData26_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData26_CANNY_PROMOTE_EDGES, 16},
{4, 2, 864, 0, 4, gScenarioBufferData27_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData27_CANNY_PROMOTE_EDGES, 16},
{5, 2, 1040, 0, 4, gScenarioBufferData28_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData28_CANNY_PROMOTE_EDGES, 16},
{6, 2, 1216, 0, 4, gScenarioBufferData29_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData29_CANNY_PROMOTE_EDGES, 16},
{7, 2, 1392, 0, 4, gScenarioBufferData30_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData30_CANNY_PROMOTE_EDGES, 16},
{8, 2, 1568, 0, 4, gScenarioBufferData31_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData31_CANNY_PROMOTE_EDGES, 16},
{9, 2, 1744, 0, 4, gScenarioBufferData32_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData32_CANNY_PROMOTE_EDGES, 16},
{10, 2, 1920, 0, 4, gScenarioBufferData33_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData33_CANNY_PROMOTE_EDGES, 16},
{11, 2, 2096, 0, 4, gScenarioBufferData34_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData34_CANNY_PROMOTE_EDGES, 16},
{12, 2, 2272, 0, 4, gScenarioBufferData35_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData35_CANNY_PROMOTE_EDGES, 16},
{13, 2, 2448, 0, 4, gScenarioBufferData36_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData36_CANNY_PROMOTE_EDGES, 16},
{14, 2, 2624, 0, 4, gScenarioBufferData37_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData37_CANNY_PROMOTE_EDGES, 16},
{15, 2, 2800, 0, 4, gScenarioBufferData38_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData38_CANNY_PROMOTE_EDGES, 16},
{16, 2, 2976, 0, 4, gScenarioBufferData39_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData39_CANNY_PROMOTE_EDGES, 16},
{18, 2, 3328, 0, 4, gScenarioBufferData40_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData40_CANNY_PROMOTE_EDGES, 16},
{20, 2, 3680, 0, 4, gScenarioBufferData41_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData41_CANNY_PROMOTE_EDGES, 16},
{1, 3, 472, 0, 4, gScenarioBufferData42_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData42_CANNY_PROMOTE_EDGES, 16},
{2, 3, 720, 0, 4, gScenarioBufferData43_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData43_CANNY_PROMOTE_EDGES, 16},
{3, 3, 968, 0, 4, gScenarioBufferData44_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData44_CANNY_PROMOTE_EDGES, 16},
{4, 3, 1216, 0, 4, gScenarioBufferData45_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData45_CANNY_PROMOTE_EDGES, 16},
{5, 3, 1464, 0, 4, gScenarioBufferData46_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData46_CANNY_PROMOTE_EDGES, 16},
{6, 3, 1712, 0, 4, gScenarioBufferData47_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData47_CANNY_PROMOTE_EDGES, 16},
{7, 3, 1960, 0, 4, gScenarioBufferData48_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData48_CANNY_PROMOTE_EDGES, 16},
{8, 3, 2208, 0, 4, gScenarioBufferData49_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData49_CANNY_PROMOTE_EDGES, 16},
{9, 3, 2456, 0, 4, gScenarioBufferData50_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData50_CANNY_PROMOTE_EDGES, 16},
{10, 3, 2704, 0, 4, gScenarioBufferData51_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData51_CANNY_PROMOTE_EDGES, 16},
{11, 3, 2952, 0, 4, gScenarioBufferData52_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData52_CANNY_PROMOTE_EDGES, 16},
{12, 3, 3200, 0, 4, gScenarioBufferData53_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData53_CANNY_PROMOTE_EDGES, 16},
{13, 3, 3448, 0, 4, gScenarioBufferData54_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData54_CANNY_PROMOTE_EDGES, 16},
{14, 3, 3696, 0, 4, gScenarioBufferData55_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData55_CANNY_PROMOTE_EDGES, 16},
{1, 4, 608, 0, 4, gScenarioBufferData56_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData56_CANNY_PROMOTE_EDGES, 16},
{2, 4, 928, 0, 4, gScenarioBufferData57_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData57_CANNY_PROMOTE_EDGES, 16},
{3, 4, 1248, 0, 4, gScenarioBufferData58_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData58_CANNY_PROMOTE_EDGES, 16},
{4, 4, 1568, 0, 4, gScenarioBufferData59_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData59_CANNY_PROMOTE_EDGES, 16},
{5, 4, 1888, 0, 4, gScenarioBufferData60_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData60_CANNY_PROMOTE_EDGES, 16},
{6, 4, 2208, 0, 4, gScenarioBufferData61_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData61_CANNY_PROMOTE_EDGES, 16},
{7, 4, 2528, 0, 4, gScenarioBufferData62_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData62_CANNY_PROMOTE_EDGES, 16},
{8, 4, 2848, 0, 4, gScenarioBufferData63_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData63_CANNY_PROMOTE_EDGES, 16},
{9, 4, 3168, 0, 4, gScenarioBufferData64_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData64_CANNY_PROMOTE_EDGES, 16},
{10, 4, 3488, 0, 4, gScenarioBufferData65_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData65_CANNY_PROMOTE_EDGES, 16},
{11, 4, 3808, 0, 4, gScenarioBufferData66_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData66_CANNY_PROMOTE_EDGES, 16},
{1, 5, 744, 0, 4, gScenarioBufferData67_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData67_CANNY_PROMOTE_EDGES, 16},
{2, 5, 1136, 0, 4, gScenarioBufferData68_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData68_CANNY_PROMOTE_EDGES, 16},
{3, 5, 1528, 0, 4, gScenarioBufferData69_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData69_CANNY_PROMOTE_EDGES, 16},
{4, 5, 1920, 0, 4, gScenarioBufferData70_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData70_CANNY_PROMOTE_EDGES, 16},
{5, 5, 2312, 0, 4, gScenarioBufferData71_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData71_CANNY_PROMOTE_EDGES, 16},
{6, 5, 2704, 0, 4, gScenarioBufferData72_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData72_CANNY_PROMOTE_EDGES, 16},
{7, 5, 3096, 0, 4, gScenarioBufferData73_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData73_CANNY_PROMOTE_EDGES, 16},
{8, 5, 3488, 0, 4, gScenarioBufferData74_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData74_CANNY_PROMOTE_EDGES, 16},
{1, 6, 880, 0, 4, gScenarioBufferData75_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData75_CANNY_PROMOTE_EDGES, 16},
{2, 6, 1344, 0, 4, gScenarioBufferData76_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData76_CANNY_PROMOTE_EDGES, 16},
{3, 6, 1808, 0, 4, gScenarioBufferData77_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData77_CANNY_PROMOTE_EDGES, 16},
{4, 6, 2272, 0, 4, gScenarioBufferData78_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData78_CANNY_PROMOTE_EDGES, 16},
{5, 6, 2736, 0, 4, gScenarioBufferData79_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData79_CANNY_PROMOTE_EDGES, 16},
{6, 6, 3200, 0, 4, gScenarioBufferData80_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData80_CANNY_PROMOTE_EDGES, 16},
{7, 6, 3664, 0, 4, gScenarioBufferData81_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData81_CANNY_PROMOTE_EDGES, 16},
{1, 8, 1152, 0, 4, gScenarioBufferData82_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData82_CANNY_PROMOTE_EDGES, 16},
{2, 8, 1760, 0, 4, gScenarioBufferData83_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData83_CANNY_PROMOTE_EDGES, 16},
{3, 8, 2368, 0, 4, gScenarioBufferData84_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData84_CANNY_PROMOTE_EDGES, 16},
{4, 8, 2976, 0, 4, gScenarioBufferData85_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData85_CANNY_PROMOTE_EDGES, 16},
{5, 8, 3584, 0, 4, gScenarioBufferData86_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData86_CANNY_PROMOTE_EDGES, 16},
{1, 10, 1424, 0, 4, gScenarioBufferData87_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData87_CANNY_PROMOTE_EDGES, 16},
{2, 10, 2176, 0, 4, gScenarioBufferData88_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData88_CANNY_PROMOTE_EDGES, 16},
{3, 10, 2928, 0, 4, gScenarioBufferData89_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData89_CANNY_PROMOTE_EDGES, 16},
{4, 10, 3680, 0, 4, gScenarioBufferData90_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData90_CANNY_PROMOTE_EDGES, 16},
{1, 12, 1696, 0, 4, gScenarioBufferData91_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData91_CANNY_PROMOTE_EDGES, 16},
{2, 12, 2592, 0, 4, gScenarioBufferData92_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData92_CANNY_PROMOTE_EDGES, 16},
{3, 12, 3488, 0, 4, gScenarioBufferData93_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData93_CANNY_PROMOTE_EDGES, 16},
{1, 14, 1968, 0, 4, gScenarioBufferData94_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData94_CANNY_PROMOTE_EDGES, 16},
{2, 14, 3008, 0, 4, gScenarioBufferData95_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData95_CANNY_PROMOTE_EDGES, 16},
{1, 16, 2240, 0, 4, gScenarioBufferData96_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData96_CANNY_PROMOTE_EDGES, 16},
{2, 16, 3424, 0, 4, gScenarioBufferData97_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData97_CANNY_PROMOTE_EDGES, 16},
{1, 18, 2512, 0, 4, gScenarioBufferData98_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData98_CANNY_PROMOTE_EDGES, 16},
{2, 18, 3840, 0, 4, gScenarioBufferData99_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData99_CANNY_PROMOTE_EDGES, 16},
{1, 20, 2784, 0, 4, gScenarioBufferData100_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData100_CANNY_PROMOTE_EDGES, 16},
{1, 22, 3056, 0, 4, gScenarioBufferData101_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData101_CANNY_PROMOTE_EDGES, 16},
{1, 24, 3328, 0, 4, gScenarioBufferData102_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData102_CANNY_PROMOTE_EDGES, 16},
{1, 26, 3600, 0, 4, gScenarioBufferData103_CANNY_PROMOTE_EDGES, 80, gScenarioKernelData103_CANNY_PROMOTE_EDGES, 16}
};

static acf_scenario_list gScenarioList_CANNY_PROMOTE_EDGES = {
104, //number of scenarios
gScenarioArray_CANNY_PROMOTE_EDGES};
//**************************************************************

class CANNY_PROMOTE_EDGES : public ACF_Process_APU
{

public:
   CANNY_PROMOTE_EDGES(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("CANNY_PROMOTE_EDGES");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("CANNY_PROMOTE_EDGES",
                                        CANNY_PROMOTE_EDGES_LOAD_SEGMENTS,
                                        CANNY_PROMOTE_EDGES_LOAD_PMEM, CANNY_PROMOTE_EDGES_LOAD_PMEM_SIZE,
                                        CANNY_PROMOTE_EDGES_LOAD_DMEM, CANNY_PROMOTE_EDGES_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(CANNY_PROMOTE_EDGES_APEX_LOG_BUFFER, CANNY_PROMOTE_EDGES_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_CANNY_PROMOTE_EDGES);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_CANNY_PROMOTE_EDGES

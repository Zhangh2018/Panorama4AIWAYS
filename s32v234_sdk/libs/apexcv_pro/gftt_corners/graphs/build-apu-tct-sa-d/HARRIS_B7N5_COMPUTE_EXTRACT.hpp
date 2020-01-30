#ifndef _ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE_EXTRACT
#define _ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE_EXTRACT

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HARRIS_B7N5_COMPUTE_EXTRACT_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 1, 4, 0, {1, 1, 2, 2}}, {4, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{8, 1, 4, 0, {1, 1, 2, 2}}, {8, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{12, 1, 4, 0, {1, 1, 2, 2}}, {12, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{16, 1, 4, 0, {1, 1, 2, 2}}, {16, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{20, 1, 4, 0, {1, 1, 2, 2}}, {20, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{24, 1, 4, 0, {1, 1, 2, 2}}, {24, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{28, 1, 4, 0, {1, 1, 2, 2}}, {28, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{32, 1, 4, 0, {1, 1, 2, 2}}, {32, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{36, 1, 4, 0, {1, 1, 2, 2}}, {36, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{40, 1, 4, 0, {1, 1, 2, 2}}, {40, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{44, 1, 4, 0, {1, 1, 2, 2}}, {44, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{48, 1, 4, 0, {1, 1, 2, 2}}, {48, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{52, 1, 4, 0, {1, 1, 2, 2}}, {52, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{56, 1, 4, 0, {1, 1, 2, 2}}, {56, 1, 2, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {8192, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {4096, 1, 1, 6, {0, 0, 0, 0}}, {1, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {20, 1, 1, 6, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 2, 4, 0, {1, 1, 2, 2}}, {4, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{8, 2, 4, 0, {1, 1, 2, 2}}, {8, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{12, 2, 4, 0, {1, 1, 2, 2}}, {12, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{16, 2, 4, 0, {1, 1, 2, 2}}, {16, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{20, 2, 4, 0, {1, 1, 2, 2}}, {20, 2, 3, 4, {0, 1, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {8192, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {4096, 1, 1, 4, {0, 0, 0, 0}}, {1, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {20, 1, 1, 4, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 3, 4, 0, {1, 1, 2, 2}}, {4, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{8, 3, 4, 0, {1, 1, 2, 2}}, {8, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{12, 3, 4, 0, {1, 1, 2, 2}}, {12, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{16, 3, 4, 0, {1, 1, 2, 2}}, {16, 3, 3, 3, {0, 1, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 3, 3, {0, 3, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 3, 3, {0, 3, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {8192, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {4096, 1, 1, 3, {0, 0, 0, 0}}, {1, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {20, 1, 1, 3, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 5, 4, 0, {1, 1, 2, 2}}, {4, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{8, 5, 4, 0, {1, 1, 2, 2}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{12, 5, 4, 0, {1, 1, 2, 2}}, {12, 5, 2, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 6, 4, 0, {1, 1, 2, 2}}, {4, 6, 3, 2, {0, 1, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 3, 2, {0, 3, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {8192, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {4096, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {20, 1, 1, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 8, 1, 1, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {38, 1, 1, 1, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 5}, {6, 0}, {6, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 3}, {4, 0}, {4, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 2}, {3, 0}, {3, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HARRIS_B7N5_COMPUTE_EXTRACT[] = {{1, 1}, {2, 0}, {2, 0}};

static acf_scenario gScenarioArray_HARRIS_B7N5_COMPUTE_EXTRACT[] = {
{1, 1, 3296, 65600, 6, gScenarioBufferData0_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData0_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{2, 1, 3336, 65600, 6, gScenarioBufferData1_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData1_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{3, 1, 3376, 65600, 6, gScenarioBufferData2_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData2_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{4, 1, 3416, 65600, 6, gScenarioBufferData3_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData3_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{5, 1, 3456, 65600, 6, gScenarioBufferData4_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData4_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{6, 1, 3496, 65600, 6, gScenarioBufferData5_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData5_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{7, 1, 3536, 65600, 6, gScenarioBufferData6_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData6_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{8, 1, 3576, 65600, 6, gScenarioBufferData7_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData7_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{9, 1, 3616, 65600, 6, gScenarioBufferData8_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData8_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{10, 1, 3656, 65600, 6, gScenarioBufferData9_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData9_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{11, 1, 3696, 65600, 6, gScenarioBufferData10_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData10_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{12, 1, 3736, 65600, 6, gScenarioBufferData11_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData11_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{13, 1, 3776, 65600, 6, gScenarioBufferData12_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData12_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{14, 1, 3816, 65600, 6, gScenarioBufferData13_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData13_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 2, 3368, 65600, 4, gScenarioBufferData14_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData14_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{2, 2, 3464, 65600, 4, gScenarioBufferData15_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData15_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{3, 2, 3560, 65600, 4, gScenarioBufferData16_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData16_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{4, 2, 3656, 65600, 4, gScenarioBufferData17_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData17_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{5, 2, 3752, 65600, 4, gScenarioBufferData18_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData18_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 3, 3424, 65600, 3, gScenarioBufferData19_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData19_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{2, 3, 3560, 65600, 3, gScenarioBufferData20_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData20_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{3, 3, 3696, 65600, 3, gScenarioBufferData21_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData21_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{4, 3, 3832, 65600, 3, gScenarioBufferData22_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData22_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 4, 3496, 65600, 3, gScenarioBufferData23_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData23_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{2, 4, 3688, 65600, 3, gScenarioBufferData24_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData24_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 5, 3488, 65600, 2, gScenarioBufferData25_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData25_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{2, 5, 3656, 65600, 2, gScenarioBufferData26_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData26_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{3, 5, 3824, 65600, 2, gScenarioBufferData27_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData27_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 6, 3592, 65600, 2, gScenarioBufferData28_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData28_HARRIS_B7N5_COMPUTE_EXTRACT, 12},
{1, 8, 3720, 65600, 2, gScenarioBufferData29_HARRIS_B7N5_COMPUTE_EXTRACT, 304, gScenarioKernelData29_HARRIS_B7N5_COMPUTE_EXTRACT, 12}
};

static acf_scenario_list gScenarioList_HARRIS_B7N5_COMPUTE_EXTRACT = {
30, //number of scenarios
gScenarioArray_HARRIS_B7N5_COMPUTE_EXTRACT};
//**************************************************************

class HARRIS_B7N5_COMPUTE_EXTRACT : public ACF_Process_APU
{

public:
   HARRIS_B7N5_COMPUTE_EXTRACT(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HARRIS_B7N5_COMPUTE_EXTRACT");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HARRIS_B7N5_COMPUTE_EXTRACT",
                                        HARRIS_B7N5_COMPUTE_EXTRACT_LOAD_SEGMENTS,
                                        HARRIS_B7N5_COMPUTE_EXTRACT_LOAD_PMEM, HARRIS_B7N5_COMPUTE_EXTRACT_LOAD_PMEM_SIZE,
                                        HARRIS_B7N5_COMPUTE_EXTRACT_LOAD_DMEM, HARRIS_B7N5_COMPUTE_EXTRACT_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HARRIS_B7N5_COMPUTE_EXTRACT_APEX_LOG_BUFFER, HARRIS_B7N5_COMPUTE_EXTRACT_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_08U, 1, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 7, 1, 0, 1, 1, 0, 1, 0, 18, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_IMAGE", icp::DATATYPE_16S, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_CORNERLIST", icp::DATATYPE_32S, 1, 1, 8192, 1, 1, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_COUNT", icp::DATATYPE_32S, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HARRIS_B7N5_COMPUTE_EXTRACT);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HARRIS_B7N5_COMPUTE_EXTRACT

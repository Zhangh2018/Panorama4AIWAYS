#ifndef _ACF_PROCESS_APU_RGB888X_TO_Y
#define _ACF_PROCESS_APU_RGB888X_TO_Y

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <RGB888X_TO_Y_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_RGB888X_TO_Y[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_RGB888X_TO_Y[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_RGB888X_TO_Y[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_RGB888X_TO_Y[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_RGB888X_TO_Y[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_RGB888X_TO_Y[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_RGB888X_TO_Y[] = {{28, 1, 2, 0, {0, 0, 0, 0}}, {28, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_RGB888X_TO_Y[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_RGB888X_TO_Y[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_RGB888X_TO_Y[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_RGB888X_TO_Y[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_RGB888X_TO_Y[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_RGB888X_TO_Y[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_RGB888X_TO_Y[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_RGB888X_TO_Y[] = {{28, 2, 2, 0, {0, 0, 0, 0}}, {28, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_RGB888X_TO_Y[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_RGB888X_TO_Y[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_RGB888X_TO_Y[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_RGB888X_TO_Y[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_RGB888X_TO_Y[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_RGB888X_TO_Y[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_RGB888X_TO_Y[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_RGB888X_TO_Y[] = {{28, 3, 2, 0, {0, 0, 0, 0}}, {28, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_RGB888X_TO_Y[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_RGB888X_TO_Y[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_RGB888X_TO_Y[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_RGB888X_TO_Y[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_RGB888X_TO_Y[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_RGB888X_TO_Y[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_RGB888X_TO_Y[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_RGB888X_TO_Y[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_RGB888X_TO_Y[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_RGB888X_TO_Y[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_RGB888X_TO_Y[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_RGB888X_TO_Y[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_RGB888X_TO_Y[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_RGB888X_TO_Y[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_RGB888X_TO_Y[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_RGB888X_TO_Y[] = {{28, 5, 2, 0, {0, 0, 0, 0}}, {28, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_RGB888X_TO_Y[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_RGB888X_TO_Y[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_RGB888X_TO_Y[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_RGB888X_TO_Y[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_RGB888X_TO_Y[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_RGB888X_TO_Y[] = {{20, 6, 2, 0, {0, 0, 0, 0}}, {20, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_RGB888X_TO_Y[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_RGB888X_TO_Y[] = {{28, 6, 2, 0, {0, 0, 0, 0}}, {28, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_RGB888X_TO_Y[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_RGB888X_TO_Y[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_RGB888X_TO_Y[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_RGB888X_TO_Y[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_RGB888X_TO_Y[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_RGB888X_TO_Y[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_RGB888X_TO_Y[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_RGB888X_TO_Y[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_RGB888X_TO_Y[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_RGB888X_TO_Y[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_RGB888X_TO_Y[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_RGB888X_TO_Y[] = {{12, 10, 2, 0, {0, 0, 0, 0}}, {12, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_RGB888X_TO_Y[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_RGB888X_TO_Y[] = {{20, 10, 2, 0, {0, 0, 0, 0}}, {20, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_RGB888X_TO_Y[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_RGB888X_TO_Y[] = {{28, 10, 2, 0, {0, 0, 0, 0}}, {28, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_RGB888X_TO_Y[] = {{32, 10, 2, 0, {0, 0, 0, 0}}, {32, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_RGB888X_TO_Y[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_RGB888X_TO_Y[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_RGB888X_TO_Y[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_RGB888X_TO_Y[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_RGB888X_TO_Y[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_RGB888X_TO_Y[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_RGB888X_TO_Y[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_RGB888X_TO_Y[] = {{32, 12, 2, 0, {0, 0, 0, 0}}, {32, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_RGB888X_TO_Y[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_RGB888X_TO_Y[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_RGB888X_TO_Y[] = {{12, 14, 2, 0, {0, 0, 0, 0}}, {12, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_RGB888X_TO_Y[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_RGB888X_TO_Y[] = {{20, 14, 2, 0, {0, 0, 0, 0}}, {20, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_RGB888X_TO_Y[] = {{24, 14, 2, 0, {0, 0, 0, 0}}, {24, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_RGB888X_TO_Y[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_RGB888X_TO_Y[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_RGB888X_TO_Y[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_RGB888X_TO_Y[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_RGB888X_TO_Y[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_RGB888X_TO_Y[] = {{24, 16, 2, 0, {0, 0, 0, 0}}, {24, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_RGB888X_TO_Y[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_RGB888X_TO_Y[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_RGB888X_TO_Y[] = {{12, 18, 2, 0, {0, 0, 0, 0}}, {12, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_RGB888X_TO_Y[] = {{16, 18, 2, 0, {0, 0, 0, 0}}, {16, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_RGB888X_TO_Y[] = {{20, 18, 2, 0, {0, 0, 0, 0}}, {20, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_RGB888X_TO_Y[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData90_RGB888X_TO_Y[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData91_RGB888X_TO_Y[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData92_RGB888X_TO_Y[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData93_RGB888X_TO_Y[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData94_RGB888X_TO_Y[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData95_RGB888X_TO_Y[] = {{12, 22, 2, 0, {0, 0, 0, 0}}, {12, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData96_RGB888X_TO_Y[] = {{16, 22, 2, 0, {0, 0, 0, 0}}, {16, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData97_RGB888X_TO_Y[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData98_RGB888X_TO_Y[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData99_RGB888X_TO_Y[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData100_RGB888X_TO_Y[] = {{16, 24, 2, 0, {0, 0, 0, 0}}, {16, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData101_RGB888X_TO_Y[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData102_RGB888X_TO_Y[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData103_RGB888X_TO_Y[] = {{12, 26, 2, 0, {0, 0, 0, 0}}, {12, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData104_RGB888X_TO_Y[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData105_RGB888X_TO_Y[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData106_RGB888X_TO_Y[] = {{12, 28, 2, 0, {0, 0, 0, 0}}, {12, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData107_RGB888X_TO_Y[] = {{4, 30, 2, 0, {0, 0, 0, 0}}, {4, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData108_RGB888X_TO_Y[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData109_RGB888X_TO_Y[] = {{12, 30, 2, 0, {0, 0, 0, 0}}, {12, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData110_RGB888X_TO_Y[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData111_RGB888X_TO_Y[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData112_RGB888X_TO_Y[] = {{12, 32, 2, 0, {0, 0, 0, 0}}, {12, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData90_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData91_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData92_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData93_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData94_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData95_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData96_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData97_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData98_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData99_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData100_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData101_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData102_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData103_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData104_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData105_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData106_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData107_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData108_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData109_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData110_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData111_RGB888X_TO_Y[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData112_RGB888X_TO_Y[] = {{0, 0}};

static acf_scenario gScenarioArray_RGB888X_TO_Y[] = {
{1, 1, 40, 48, 0, gScenarioBufferData0_RGB888X_TO_Y, 80, gScenarioKernelData0_RGB888X_TO_Y, 4},
{2, 1, 80, 48, 0, gScenarioBufferData1_RGB888X_TO_Y, 80, gScenarioKernelData1_RGB888X_TO_Y, 4},
{3, 1, 120, 48, 0, gScenarioBufferData2_RGB888X_TO_Y, 80, gScenarioKernelData2_RGB888X_TO_Y, 4},
{4, 1, 160, 48, 0, gScenarioBufferData3_RGB888X_TO_Y, 80, gScenarioKernelData3_RGB888X_TO_Y, 4},
{5, 1, 200, 48, 0, gScenarioBufferData4_RGB888X_TO_Y, 80, gScenarioKernelData4_RGB888X_TO_Y, 4},
{6, 1, 240, 48, 0, gScenarioBufferData5_RGB888X_TO_Y, 80, gScenarioKernelData5_RGB888X_TO_Y, 4},
{7, 1, 280, 48, 0, gScenarioBufferData6_RGB888X_TO_Y, 80, gScenarioKernelData6_RGB888X_TO_Y, 4},
{8, 1, 320, 48, 0, gScenarioBufferData7_RGB888X_TO_Y, 80, gScenarioKernelData7_RGB888X_TO_Y, 4},
{1, 2, 80, 48, 0, gScenarioBufferData8_RGB888X_TO_Y, 80, gScenarioKernelData8_RGB888X_TO_Y, 4},
{2, 2, 160, 48, 0, gScenarioBufferData9_RGB888X_TO_Y, 80, gScenarioKernelData9_RGB888X_TO_Y, 4},
{3, 2, 240, 48, 0, gScenarioBufferData10_RGB888X_TO_Y, 80, gScenarioKernelData10_RGB888X_TO_Y, 4},
{4, 2, 320, 48, 0, gScenarioBufferData11_RGB888X_TO_Y, 80, gScenarioKernelData11_RGB888X_TO_Y, 4},
{5, 2, 400, 48, 0, gScenarioBufferData12_RGB888X_TO_Y, 80, gScenarioKernelData12_RGB888X_TO_Y, 4},
{6, 2, 480, 48, 0, gScenarioBufferData13_RGB888X_TO_Y, 80, gScenarioKernelData13_RGB888X_TO_Y, 4},
{7, 2, 560, 48, 0, gScenarioBufferData14_RGB888X_TO_Y, 80, gScenarioKernelData14_RGB888X_TO_Y, 4},
{8, 2, 640, 48, 0, gScenarioBufferData15_RGB888X_TO_Y, 80, gScenarioKernelData15_RGB888X_TO_Y, 4},
{1, 3, 120, 48, 0, gScenarioBufferData16_RGB888X_TO_Y, 80, gScenarioKernelData16_RGB888X_TO_Y, 4},
{2, 3, 240, 48, 0, gScenarioBufferData17_RGB888X_TO_Y, 80, gScenarioKernelData17_RGB888X_TO_Y, 4},
{3, 3, 360, 48, 0, gScenarioBufferData18_RGB888X_TO_Y, 80, gScenarioKernelData18_RGB888X_TO_Y, 4},
{4, 3, 480, 48, 0, gScenarioBufferData19_RGB888X_TO_Y, 80, gScenarioKernelData19_RGB888X_TO_Y, 4},
{5, 3, 600, 48, 0, gScenarioBufferData20_RGB888X_TO_Y, 80, gScenarioKernelData20_RGB888X_TO_Y, 4},
{6, 3, 720, 48, 0, gScenarioBufferData21_RGB888X_TO_Y, 80, gScenarioKernelData21_RGB888X_TO_Y, 4},
{7, 3, 840, 48, 0, gScenarioBufferData22_RGB888X_TO_Y, 80, gScenarioKernelData22_RGB888X_TO_Y, 4},
{8, 3, 960, 48, 0, gScenarioBufferData23_RGB888X_TO_Y, 80, gScenarioKernelData23_RGB888X_TO_Y, 4},
{1, 4, 160, 48, 0, gScenarioBufferData24_RGB888X_TO_Y, 80, gScenarioKernelData24_RGB888X_TO_Y, 4},
{2, 4, 320, 48, 0, gScenarioBufferData25_RGB888X_TO_Y, 80, gScenarioKernelData25_RGB888X_TO_Y, 4},
{3, 4, 480, 48, 0, gScenarioBufferData26_RGB888X_TO_Y, 80, gScenarioKernelData26_RGB888X_TO_Y, 4},
{4, 4, 640, 48, 0, gScenarioBufferData27_RGB888X_TO_Y, 80, gScenarioKernelData27_RGB888X_TO_Y, 4},
{5, 4, 800, 48, 0, gScenarioBufferData28_RGB888X_TO_Y, 80, gScenarioKernelData28_RGB888X_TO_Y, 4},
{6, 4, 960, 48, 0, gScenarioBufferData29_RGB888X_TO_Y, 80, gScenarioKernelData29_RGB888X_TO_Y, 4},
{7, 4, 1120, 48, 0, gScenarioBufferData30_RGB888X_TO_Y, 80, gScenarioKernelData30_RGB888X_TO_Y, 4},
{8, 4, 1280, 48, 0, gScenarioBufferData31_RGB888X_TO_Y, 80, gScenarioKernelData31_RGB888X_TO_Y, 4},
{1, 5, 200, 48, 0, gScenarioBufferData32_RGB888X_TO_Y, 80, gScenarioKernelData32_RGB888X_TO_Y, 4},
{2, 5, 400, 48, 0, gScenarioBufferData33_RGB888X_TO_Y, 80, gScenarioKernelData33_RGB888X_TO_Y, 4},
{3, 5, 600, 48, 0, gScenarioBufferData34_RGB888X_TO_Y, 80, gScenarioKernelData34_RGB888X_TO_Y, 4},
{4, 5, 800, 48, 0, gScenarioBufferData35_RGB888X_TO_Y, 80, gScenarioKernelData35_RGB888X_TO_Y, 4},
{5, 5, 1000, 48, 0, gScenarioBufferData36_RGB888X_TO_Y, 80, gScenarioKernelData36_RGB888X_TO_Y, 4},
{6, 5, 1200, 48, 0, gScenarioBufferData37_RGB888X_TO_Y, 80, gScenarioKernelData37_RGB888X_TO_Y, 4},
{7, 5, 1400, 48, 0, gScenarioBufferData38_RGB888X_TO_Y, 80, gScenarioKernelData38_RGB888X_TO_Y, 4},
{8, 5, 1600, 48, 0, gScenarioBufferData39_RGB888X_TO_Y, 80, gScenarioKernelData39_RGB888X_TO_Y, 4},
{1, 6, 240, 48, 0, gScenarioBufferData40_RGB888X_TO_Y, 80, gScenarioKernelData40_RGB888X_TO_Y, 4},
{2, 6, 480, 48, 0, gScenarioBufferData41_RGB888X_TO_Y, 80, gScenarioKernelData41_RGB888X_TO_Y, 4},
{3, 6, 720, 48, 0, gScenarioBufferData42_RGB888X_TO_Y, 80, gScenarioKernelData42_RGB888X_TO_Y, 4},
{4, 6, 960, 48, 0, gScenarioBufferData43_RGB888X_TO_Y, 80, gScenarioKernelData43_RGB888X_TO_Y, 4},
{5, 6, 1200, 48, 0, gScenarioBufferData44_RGB888X_TO_Y, 80, gScenarioKernelData44_RGB888X_TO_Y, 4},
{6, 6, 1440, 48, 0, gScenarioBufferData45_RGB888X_TO_Y, 80, gScenarioKernelData45_RGB888X_TO_Y, 4},
{7, 6, 1680, 48, 0, gScenarioBufferData46_RGB888X_TO_Y, 80, gScenarioKernelData46_RGB888X_TO_Y, 4},
{8, 6, 1920, 48, 0, gScenarioBufferData47_RGB888X_TO_Y, 80, gScenarioKernelData47_RGB888X_TO_Y, 4},
{1, 8, 320, 48, 0, gScenarioBufferData48_RGB888X_TO_Y, 80, gScenarioKernelData48_RGB888X_TO_Y, 4},
{2, 8, 640, 48, 0, gScenarioBufferData49_RGB888X_TO_Y, 80, gScenarioKernelData49_RGB888X_TO_Y, 4},
{3, 8, 960, 48, 0, gScenarioBufferData50_RGB888X_TO_Y, 80, gScenarioKernelData50_RGB888X_TO_Y, 4},
{4, 8, 1280, 48, 0, gScenarioBufferData51_RGB888X_TO_Y, 80, gScenarioKernelData51_RGB888X_TO_Y, 4},
{5, 8, 1600, 48, 0, gScenarioBufferData52_RGB888X_TO_Y, 80, gScenarioKernelData52_RGB888X_TO_Y, 4},
{6, 8, 1920, 48, 0, gScenarioBufferData53_RGB888X_TO_Y, 80, gScenarioKernelData53_RGB888X_TO_Y, 4},
{7, 8, 2240, 48, 0, gScenarioBufferData54_RGB888X_TO_Y, 80, gScenarioKernelData54_RGB888X_TO_Y, 4},
{8, 8, 2560, 48, 0, gScenarioBufferData55_RGB888X_TO_Y, 80, gScenarioKernelData55_RGB888X_TO_Y, 4},
{1, 10, 400, 48, 0, gScenarioBufferData56_RGB888X_TO_Y, 80, gScenarioKernelData56_RGB888X_TO_Y, 4},
{2, 10, 800, 48, 0, gScenarioBufferData57_RGB888X_TO_Y, 80, gScenarioKernelData57_RGB888X_TO_Y, 4},
{3, 10, 1200, 48, 0, gScenarioBufferData58_RGB888X_TO_Y, 80, gScenarioKernelData58_RGB888X_TO_Y, 4},
{4, 10, 1600, 48, 0, gScenarioBufferData59_RGB888X_TO_Y, 80, gScenarioKernelData59_RGB888X_TO_Y, 4},
{5, 10, 2000, 48, 0, gScenarioBufferData60_RGB888X_TO_Y, 80, gScenarioKernelData60_RGB888X_TO_Y, 4},
{6, 10, 2400, 48, 0, gScenarioBufferData61_RGB888X_TO_Y, 80, gScenarioKernelData61_RGB888X_TO_Y, 4},
{7, 10, 2800, 48, 0, gScenarioBufferData62_RGB888X_TO_Y, 80, gScenarioKernelData62_RGB888X_TO_Y, 4},
{8, 10, 3200, 48, 0, gScenarioBufferData63_RGB888X_TO_Y, 80, gScenarioKernelData63_RGB888X_TO_Y, 4},
{1, 12, 480, 48, 0, gScenarioBufferData64_RGB888X_TO_Y, 80, gScenarioKernelData64_RGB888X_TO_Y, 4},
{2, 12, 960, 48, 0, gScenarioBufferData65_RGB888X_TO_Y, 80, gScenarioKernelData65_RGB888X_TO_Y, 4},
{3, 12, 1440, 48, 0, gScenarioBufferData66_RGB888X_TO_Y, 80, gScenarioKernelData66_RGB888X_TO_Y, 4},
{4, 12, 1920, 48, 0, gScenarioBufferData67_RGB888X_TO_Y, 80, gScenarioKernelData67_RGB888X_TO_Y, 4},
{5, 12, 2400, 48, 0, gScenarioBufferData68_RGB888X_TO_Y, 80, gScenarioKernelData68_RGB888X_TO_Y, 4},
{6, 12, 2880, 48, 0, gScenarioBufferData69_RGB888X_TO_Y, 80, gScenarioKernelData69_RGB888X_TO_Y, 4},
{7, 12, 3360, 48, 0, gScenarioBufferData70_RGB888X_TO_Y, 80, gScenarioKernelData70_RGB888X_TO_Y, 4},
{8, 12, 3840, 48, 0, gScenarioBufferData71_RGB888X_TO_Y, 80, gScenarioKernelData71_RGB888X_TO_Y, 4},
{1, 14, 560, 48, 0, gScenarioBufferData72_RGB888X_TO_Y, 80, gScenarioKernelData72_RGB888X_TO_Y, 4},
{2, 14, 1120, 48, 0, gScenarioBufferData73_RGB888X_TO_Y, 80, gScenarioKernelData73_RGB888X_TO_Y, 4},
{3, 14, 1680, 48, 0, gScenarioBufferData74_RGB888X_TO_Y, 80, gScenarioKernelData74_RGB888X_TO_Y, 4},
{4, 14, 2240, 48, 0, gScenarioBufferData75_RGB888X_TO_Y, 80, gScenarioKernelData75_RGB888X_TO_Y, 4},
{5, 14, 2800, 48, 0, gScenarioBufferData76_RGB888X_TO_Y, 80, gScenarioKernelData76_RGB888X_TO_Y, 4},
{6, 14, 3360, 48, 0, gScenarioBufferData77_RGB888X_TO_Y, 80, gScenarioKernelData77_RGB888X_TO_Y, 4},
{1, 16, 640, 48, 0, gScenarioBufferData78_RGB888X_TO_Y, 80, gScenarioKernelData78_RGB888X_TO_Y, 4},
{2, 16, 1280, 48, 0, gScenarioBufferData79_RGB888X_TO_Y, 80, gScenarioKernelData79_RGB888X_TO_Y, 4},
{3, 16, 1920, 48, 0, gScenarioBufferData80_RGB888X_TO_Y, 80, gScenarioKernelData80_RGB888X_TO_Y, 4},
{4, 16, 2560, 48, 0, gScenarioBufferData81_RGB888X_TO_Y, 80, gScenarioKernelData81_RGB888X_TO_Y, 4},
{5, 16, 3200, 48, 0, gScenarioBufferData82_RGB888X_TO_Y, 80, gScenarioKernelData82_RGB888X_TO_Y, 4},
{6, 16, 3840, 48, 0, gScenarioBufferData83_RGB888X_TO_Y, 80, gScenarioKernelData83_RGB888X_TO_Y, 4},
{1, 18, 720, 48, 0, gScenarioBufferData84_RGB888X_TO_Y, 80, gScenarioKernelData84_RGB888X_TO_Y, 4},
{2, 18, 1440, 48, 0, gScenarioBufferData85_RGB888X_TO_Y, 80, gScenarioKernelData85_RGB888X_TO_Y, 4},
{3, 18, 2160, 48, 0, gScenarioBufferData86_RGB888X_TO_Y, 80, gScenarioKernelData86_RGB888X_TO_Y, 4},
{4, 18, 2880, 48, 0, gScenarioBufferData87_RGB888X_TO_Y, 80, gScenarioKernelData87_RGB888X_TO_Y, 4},
{5, 18, 3600, 48, 0, gScenarioBufferData88_RGB888X_TO_Y, 80, gScenarioKernelData88_RGB888X_TO_Y, 4},
{1, 20, 800, 48, 0, gScenarioBufferData89_RGB888X_TO_Y, 80, gScenarioKernelData89_RGB888X_TO_Y, 4},
{2, 20, 1600, 48, 0, gScenarioBufferData90_RGB888X_TO_Y, 80, gScenarioKernelData90_RGB888X_TO_Y, 4},
{3, 20, 2400, 48, 0, gScenarioBufferData91_RGB888X_TO_Y, 80, gScenarioKernelData91_RGB888X_TO_Y, 4},
{4, 20, 3200, 48, 0, gScenarioBufferData92_RGB888X_TO_Y, 80, gScenarioKernelData92_RGB888X_TO_Y, 4},
{1, 22, 880, 48, 0, gScenarioBufferData93_RGB888X_TO_Y, 80, gScenarioKernelData93_RGB888X_TO_Y, 4},
{2, 22, 1760, 48, 0, gScenarioBufferData94_RGB888X_TO_Y, 80, gScenarioKernelData94_RGB888X_TO_Y, 4},
{3, 22, 2640, 48, 0, gScenarioBufferData95_RGB888X_TO_Y, 80, gScenarioKernelData95_RGB888X_TO_Y, 4},
{4, 22, 3520, 48, 0, gScenarioBufferData96_RGB888X_TO_Y, 80, gScenarioKernelData96_RGB888X_TO_Y, 4},
{1, 24, 960, 48, 0, gScenarioBufferData97_RGB888X_TO_Y, 80, gScenarioKernelData97_RGB888X_TO_Y, 4},
{2, 24, 1920, 48, 0, gScenarioBufferData98_RGB888X_TO_Y, 80, gScenarioKernelData98_RGB888X_TO_Y, 4},
{3, 24, 2880, 48, 0, gScenarioBufferData99_RGB888X_TO_Y, 80, gScenarioKernelData99_RGB888X_TO_Y, 4},
{4, 24, 3840, 48, 0, gScenarioBufferData100_RGB888X_TO_Y, 80, gScenarioKernelData100_RGB888X_TO_Y, 4},
{1, 26, 1040, 48, 0, gScenarioBufferData101_RGB888X_TO_Y, 80, gScenarioKernelData101_RGB888X_TO_Y, 4},
{2, 26, 2080, 48, 0, gScenarioBufferData102_RGB888X_TO_Y, 80, gScenarioKernelData102_RGB888X_TO_Y, 4},
{3, 26, 3120, 48, 0, gScenarioBufferData103_RGB888X_TO_Y, 80, gScenarioKernelData103_RGB888X_TO_Y, 4},
{1, 28, 1120, 48, 0, gScenarioBufferData104_RGB888X_TO_Y, 80, gScenarioKernelData104_RGB888X_TO_Y, 4},
{2, 28, 2240, 48, 0, gScenarioBufferData105_RGB888X_TO_Y, 80, gScenarioKernelData105_RGB888X_TO_Y, 4},
{3, 28, 3360, 48, 0, gScenarioBufferData106_RGB888X_TO_Y, 80, gScenarioKernelData106_RGB888X_TO_Y, 4},
{1, 30, 1200, 48, 0, gScenarioBufferData107_RGB888X_TO_Y, 80, gScenarioKernelData107_RGB888X_TO_Y, 4},
{2, 30, 2400, 48, 0, gScenarioBufferData108_RGB888X_TO_Y, 80, gScenarioKernelData108_RGB888X_TO_Y, 4},
{3, 30, 3600, 48, 0, gScenarioBufferData109_RGB888X_TO_Y, 80, gScenarioKernelData109_RGB888X_TO_Y, 4},
{1, 32, 1280, 48, 0, gScenarioBufferData110_RGB888X_TO_Y, 80, gScenarioKernelData110_RGB888X_TO_Y, 4},
{2, 32, 2560, 48, 0, gScenarioBufferData111_RGB888X_TO_Y, 80, gScenarioKernelData111_RGB888X_TO_Y, 4},
{3, 32, 3840, 48, 0, gScenarioBufferData112_RGB888X_TO_Y, 80, gScenarioKernelData112_RGB888X_TO_Y, 4}
};

static acf_scenario_list gScenarioList_RGB888X_TO_Y = {
113, //number of scenarios
gScenarioArray_RGB888X_TO_Y};
//**************************************************************

class RGB888X_TO_Y : public ACF_Process_APU
{

public:
   RGB888X_TO_Y(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("RGB888X_TO_Y");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("RGB888X_TO_Y",
                                        RGB888X_TO_Y_LOAD_SEGMENTS,
                                        RGB888X_TO_Y_LOAD_PMEM, RGB888X_TO_Y_LOAD_PMEM_SIZE,
                                        RGB888X_TO_Y_LOAD_DMEM, RGB888X_TO_Y_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(RGB888X_TO_Y_APEX_LOG_BUFFER, RGB888X_TO_Y_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 4, 1, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("R2Y", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 0, 0, 0, 0, 0);
         AddPort("G2Y", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 3, 0, 0, 0, 0, 0);
         AddPort("B2Y", icp::DATATYPE_32S, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_08U, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_RGB888X_TO_Y);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_RGB888X_TO_Y

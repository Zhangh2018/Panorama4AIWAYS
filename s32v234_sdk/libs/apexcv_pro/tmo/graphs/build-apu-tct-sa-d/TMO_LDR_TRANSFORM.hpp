#ifndef _ACF_PROCESS_APU_TMO_LDR_TRANSFORM
#define _ACF_PROCESS_APU_TMO_LDR_TRANSFORM

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <TMO_LDR_TRANSFORM_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_TMO_LDR_TRANSFORM[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_TMO_LDR_TRANSFORM[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_TMO_LDR_TRANSFORM[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_TMO_LDR_TRANSFORM[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_TMO_LDR_TRANSFORM[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_TMO_LDR_TRANSFORM[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_TMO_LDR_TRANSFORM[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_TMO_LDR_TRANSFORM[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_TMO_LDR_TRANSFORM[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_TMO_LDR_TRANSFORM[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_TMO_LDR_TRANSFORM[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {40, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_TMO_LDR_TRANSFORM[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_TMO_LDR_TRANSFORM[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_TMO_LDR_TRANSFORM[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_TMO_LDR_TRANSFORM[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_TMO_LDR_TRANSFORM[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_TMO_LDR_TRANSFORM[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_TMO_LDR_TRANSFORM[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_TMO_LDR_TRANSFORM[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_TMO_LDR_TRANSFORM[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_TMO_LDR_TRANSFORM[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_TMO_LDR_TRANSFORM[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {40, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_TMO_LDR_TRANSFORM[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_TMO_LDR_TRANSFORM[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_TMO_LDR_TRANSFORM[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_TMO_LDR_TRANSFORM[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_TMO_LDR_TRANSFORM[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_TMO_LDR_TRANSFORM[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_TMO_LDR_TRANSFORM[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_TMO_LDR_TRANSFORM[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_TMO_LDR_TRANSFORM[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_TMO_LDR_TRANSFORM[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_TMO_LDR_TRANSFORM[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_TMO_LDR_TRANSFORM[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_TMO_LDR_TRANSFORM[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_TMO_LDR_TRANSFORM[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_TMO_LDR_TRANSFORM[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_TMO_LDR_TRANSFORM[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_TMO_LDR_TRANSFORM[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_TMO_LDR_TRANSFORM[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_TMO_LDR_TRANSFORM[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_TMO_LDR_TRANSFORM[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_TMO_LDR_TRANSFORM[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_TMO_LDR_TRANSFORM[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_TMO_LDR_TRANSFORM[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_TMO_LDR_TRANSFORM[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_TMO_LDR_TRANSFORM[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_TMO_LDR_TRANSFORM[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_TMO_LDR_TRANSFORM[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_TMO_LDR_TRANSFORM[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_TMO_LDR_TRANSFORM[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_TMO_LDR_TRANSFORM[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_TMO_LDR_TRANSFORM[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_TMO_LDR_TRANSFORM[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_TMO_LDR_TRANSFORM[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_TMO_LDR_TRANSFORM[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_TMO_LDR_TRANSFORM[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_TMO_LDR_TRANSFORM[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_TMO_LDR_TRANSFORM[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_TMO_LDR_TRANSFORM[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_TMO_LDR_TRANSFORM[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_TMO_LDR_TRANSFORM[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_TMO_LDR_TRANSFORM[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_TMO_LDR_TRANSFORM[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_TMO_LDR_TRANSFORM[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_TMO_LDR_TRANSFORM[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_TMO_LDR_TRANSFORM[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_TMO_LDR_TRANSFORM[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_TMO_LDR_TRANSFORM[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_TMO_LDR_TRANSFORM[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_TMO_LDR_TRANSFORM[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_TMO_LDR_TRANSFORM[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_TMO_LDR_TRANSFORM[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_TMO_LDR_TRANSFORM[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_TMO_LDR_TRANSFORM[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_TMO_LDR_TRANSFORM[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_TMO_LDR_TRANSFORM[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_TMO_LDR_TRANSFORM[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_TMO_LDR_TRANSFORM[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_TMO_LDR_TRANSFORM[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_TMO_LDR_TRANSFORM[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_TMO_LDR_TRANSFORM[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_TMO_LDR_TRANSFORM[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_TMO_LDR_TRANSFORM[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData84_TMO_LDR_TRANSFORM[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData85_TMO_LDR_TRANSFORM[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData86_TMO_LDR_TRANSFORM[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData87_TMO_LDR_TRANSFORM[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData88_TMO_LDR_TRANSFORM[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData89_TMO_LDR_TRANSFORM[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {64, 8, 1, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData84_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData85_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData86_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData87_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData88_TMO_LDR_TRANSFORM[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData89_TMO_LDR_TRANSFORM[] = {{0, 0}};

static acf_scenario gScenarioArray_TMO_LDR_TRANSFORM[] = {
{2, 1, 1580, 48, 0, gScenarioBufferData0_TMO_LDR_TRANSFORM, 160, gScenarioKernelData0_TMO_LDR_TRANSFORM, 4},
{4, 1, 1624, 48, 0, gScenarioBufferData1_TMO_LDR_TRANSFORM, 160, gScenarioKernelData1_TMO_LDR_TRANSFORM, 4},
{6, 1, 1668, 48, 0, gScenarioBufferData2_TMO_LDR_TRANSFORM, 160, gScenarioKernelData2_TMO_LDR_TRANSFORM, 4},
{8, 1, 1712, 48, 0, gScenarioBufferData3_TMO_LDR_TRANSFORM, 160, gScenarioKernelData3_TMO_LDR_TRANSFORM, 4},
{10, 1, 1756, 48, 0, gScenarioBufferData4_TMO_LDR_TRANSFORM, 160, gScenarioKernelData4_TMO_LDR_TRANSFORM, 4},
{12, 1, 1800, 48, 0, gScenarioBufferData5_TMO_LDR_TRANSFORM, 160, gScenarioKernelData5_TMO_LDR_TRANSFORM, 4},
{16, 1, 1888, 48, 0, gScenarioBufferData6_TMO_LDR_TRANSFORM, 160, gScenarioKernelData6_TMO_LDR_TRANSFORM, 4},
{20, 1, 1976, 48, 0, gScenarioBufferData7_TMO_LDR_TRANSFORM, 160, gScenarioKernelData7_TMO_LDR_TRANSFORM, 4},
{24, 1, 2064, 48, 0, gScenarioBufferData8_TMO_LDR_TRANSFORM, 160, gScenarioKernelData8_TMO_LDR_TRANSFORM, 4},
{32, 1, 2240, 48, 0, gScenarioBufferData9_TMO_LDR_TRANSFORM, 160, gScenarioKernelData9_TMO_LDR_TRANSFORM, 4},
{40, 1, 2416, 48, 0, gScenarioBufferData10_TMO_LDR_TRANSFORM, 160, gScenarioKernelData10_TMO_LDR_TRANSFORM, 4},
{2, 2, 1624, 48, 0, gScenarioBufferData11_TMO_LDR_TRANSFORM, 160, gScenarioKernelData11_TMO_LDR_TRANSFORM, 4},
{4, 2, 1712, 48, 0, gScenarioBufferData12_TMO_LDR_TRANSFORM, 160, gScenarioKernelData12_TMO_LDR_TRANSFORM, 4},
{6, 2, 1800, 48, 0, gScenarioBufferData13_TMO_LDR_TRANSFORM, 160, gScenarioKernelData13_TMO_LDR_TRANSFORM, 4},
{8, 2, 1888, 48, 0, gScenarioBufferData14_TMO_LDR_TRANSFORM, 160, gScenarioKernelData14_TMO_LDR_TRANSFORM, 4},
{10, 2, 1976, 48, 0, gScenarioBufferData15_TMO_LDR_TRANSFORM, 160, gScenarioKernelData15_TMO_LDR_TRANSFORM, 4},
{12, 2, 2064, 48, 0, gScenarioBufferData16_TMO_LDR_TRANSFORM, 160, gScenarioKernelData16_TMO_LDR_TRANSFORM, 4},
{16, 2, 2240, 48, 0, gScenarioBufferData17_TMO_LDR_TRANSFORM, 160, gScenarioKernelData17_TMO_LDR_TRANSFORM, 4},
{20, 2, 2416, 48, 0, gScenarioBufferData18_TMO_LDR_TRANSFORM, 160, gScenarioKernelData18_TMO_LDR_TRANSFORM, 4},
{24, 2, 2592, 48, 0, gScenarioBufferData19_TMO_LDR_TRANSFORM, 160, gScenarioKernelData19_TMO_LDR_TRANSFORM, 4},
{32, 2, 2944, 48, 0, gScenarioBufferData20_TMO_LDR_TRANSFORM, 160, gScenarioKernelData20_TMO_LDR_TRANSFORM, 4},
{40, 2, 3296, 48, 0, gScenarioBufferData21_TMO_LDR_TRANSFORM, 160, gScenarioKernelData21_TMO_LDR_TRANSFORM, 4},
{2, 3, 1668, 48, 0, gScenarioBufferData22_TMO_LDR_TRANSFORM, 160, gScenarioKernelData22_TMO_LDR_TRANSFORM, 4},
{4, 3, 1800, 48, 0, gScenarioBufferData23_TMO_LDR_TRANSFORM, 160, gScenarioKernelData23_TMO_LDR_TRANSFORM, 4},
{6, 3, 1932, 48, 0, gScenarioBufferData24_TMO_LDR_TRANSFORM, 160, gScenarioKernelData24_TMO_LDR_TRANSFORM, 4},
{8, 3, 2064, 48, 0, gScenarioBufferData25_TMO_LDR_TRANSFORM, 160, gScenarioKernelData25_TMO_LDR_TRANSFORM, 4},
{10, 3, 2196, 48, 0, gScenarioBufferData26_TMO_LDR_TRANSFORM, 160, gScenarioKernelData26_TMO_LDR_TRANSFORM, 4},
{12, 3, 2328, 48, 0, gScenarioBufferData27_TMO_LDR_TRANSFORM, 160, gScenarioKernelData27_TMO_LDR_TRANSFORM, 4},
{16, 3, 2592, 48, 0, gScenarioBufferData28_TMO_LDR_TRANSFORM, 160, gScenarioKernelData28_TMO_LDR_TRANSFORM, 4},
{20, 3, 2856, 48, 0, gScenarioBufferData29_TMO_LDR_TRANSFORM, 160, gScenarioKernelData29_TMO_LDR_TRANSFORM, 4},
{24, 3, 3120, 48, 0, gScenarioBufferData30_TMO_LDR_TRANSFORM, 160, gScenarioKernelData30_TMO_LDR_TRANSFORM, 4},
{32, 3, 3648, 48, 0, gScenarioBufferData31_TMO_LDR_TRANSFORM, 160, gScenarioKernelData31_TMO_LDR_TRANSFORM, 4},
{2, 4, 1712, 48, 0, gScenarioBufferData32_TMO_LDR_TRANSFORM, 160, gScenarioKernelData32_TMO_LDR_TRANSFORM, 4},
{4, 4, 1888, 48, 0, gScenarioBufferData33_TMO_LDR_TRANSFORM, 160, gScenarioKernelData33_TMO_LDR_TRANSFORM, 4},
{6, 4, 2064, 48, 0, gScenarioBufferData34_TMO_LDR_TRANSFORM, 160, gScenarioKernelData34_TMO_LDR_TRANSFORM, 4},
{8, 4, 2240, 48, 0, gScenarioBufferData35_TMO_LDR_TRANSFORM, 160, gScenarioKernelData35_TMO_LDR_TRANSFORM, 4},
{10, 4, 2416, 48, 0, gScenarioBufferData36_TMO_LDR_TRANSFORM, 160, gScenarioKernelData36_TMO_LDR_TRANSFORM, 4},
{12, 4, 2592, 48, 0, gScenarioBufferData37_TMO_LDR_TRANSFORM, 160, gScenarioKernelData37_TMO_LDR_TRANSFORM, 4},
{16, 4, 2944, 48, 0, gScenarioBufferData38_TMO_LDR_TRANSFORM, 160, gScenarioKernelData38_TMO_LDR_TRANSFORM, 4},
{20, 4, 3296, 48, 0, gScenarioBufferData39_TMO_LDR_TRANSFORM, 160, gScenarioKernelData39_TMO_LDR_TRANSFORM, 4},
{24, 4, 3648, 48, 0, gScenarioBufferData40_TMO_LDR_TRANSFORM, 160, gScenarioKernelData40_TMO_LDR_TRANSFORM, 4},
{2, 5, 1756, 48, 0, gScenarioBufferData41_TMO_LDR_TRANSFORM, 160, gScenarioKernelData41_TMO_LDR_TRANSFORM, 4},
{4, 5, 1976, 48, 0, gScenarioBufferData42_TMO_LDR_TRANSFORM, 160, gScenarioKernelData42_TMO_LDR_TRANSFORM, 4},
{6, 5, 2196, 48, 0, gScenarioBufferData43_TMO_LDR_TRANSFORM, 160, gScenarioKernelData43_TMO_LDR_TRANSFORM, 4},
{8, 5, 2416, 48, 0, gScenarioBufferData44_TMO_LDR_TRANSFORM, 160, gScenarioKernelData44_TMO_LDR_TRANSFORM, 4},
{10, 5, 2636, 48, 0, gScenarioBufferData45_TMO_LDR_TRANSFORM, 160, gScenarioKernelData45_TMO_LDR_TRANSFORM, 4},
{12, 5, 2856, 48, 0, gScenarioBufferData46_TMO_LDR_TRANSFORM, 160, gScenarioKernelData46_TMO_LDR_TRANSFORM, 4},
{16, 5, 3296, 48, 0, gScenarioBufferData47_TMO_LDR_TRANSFORM, 160, gScenarioKernelData47_TMO_LDR_TRANSFORM, 4},
{20, 5, 3736, 48, 0, gScenarioBufferData48_TMO_LDR_TRANSFORM, 160, gScenarioKernelData48_TMO_LDR_TRANSFORM, 4},
{2, 6, 1800, 48, 0, gScenarioBufferData49_TMO_LDR_TRANSFORM, 160, gScenarioKernelData49_TMO_LDR_TRANSFORM, 4},
{4, 6, 2064, 48, 0, gScenarioBufferData50_TMO_LDR_TRANSFORM, 160, gScenarioKernelData50_TMO_LDR_TRANSFORM, 4},
{6, 6, 2328, 48, 0, gScenarioBufferData51_TMO_LDR_TRANSFORM, 160, gScenarioKernelData51_TMO_LDR_TRANSFORM, 4},
{8, 6, 2592, 48, 0, gScenarioBufferData52_TMO_LDR_TRANSFORM, 160, gScenarioKernelData52_TMO_LDR_TRANSFORM, 4},
{10, 6, 2856, 48, 0, gScenarioBufferData53_TMO_LDR_TRANSFORM, 160, gScenarioKernelData53_TMO_LDR_TRANSFORM, 4},
{12, 6, 3120, 48, 0, gScenarioBufferData54_TMO_LDR_TRANSFORM, 160, gScenarioKernelData54_TMO_LDR_TRANSFORM, 4},
{16, 6, 3648, 48, 0, gScenarioBufferData55_TMO_LDR_TRANSFORM, 160, gScenarioKernelData55_TMO_LDR_TRANSFORM, 4},
{2, 8, 1888, 48, 0, gScenarioBufferData56_TMO_LDR_TRANSFORM, 160, gScenarioKernelData56_TMO_LDR_TRANSFORM, 4},
{4, 8, 2240, 48, 0, gScenarioBufferData57_TMO_LDR_TRANSFORM, 160, gScenarioKernelData57_TMO_LDR_TRANSFORM, 4},
{6, 8, 2592, 48, 0, gScenarioBufferData58_TMO_LDR_TRANSFORM, 160, gScenarioKernelData58_TMO_LDR_TRANSFORM, 4},
{8, 8, 2944, 48, 0, gScenarioBufferData59_TMO_LDR_TRANSFORM, 160, gScenarioKernelData59_TMO_LDR_TRANSFORM, 4},
{10, 8, 3296, 48, 0, gScenarioBufferData60_TMO_LDR_TRANSFORM, 160, gScenarioKernelData60_TMO_LDR_TRANSFORM, 4},
{12, 8, 3648, 48, 0, gScenarioBufferData61_TMO_LDR_TRANSFORM, 160, gScenarioKernelData61_TMO_LDR_TRANSFORM, 4},
{2, 10, 1976, 48, 0, gScenarioBufferData62_TMO_LDR_TRANSFORM, 160, gScenarioKernelData62_TMO_LDR_TRANSFORM, 4},
{4, 10, 2416, 48, 0, gScenarioBufferData63_TMO_LDR_TRANSFORM, 160, gScenarioKernelData63_TMO_LDR_TRANSFORM, 4},
{6, 10, 2856, 48, 0, gScenarioBufferData64_TMO_LDR_TRANSFORM, 160, gScenarioKernelData64_TMO_LDR_TRANSFORM, 4},
{8, 10, 3296, 48, 0, gScenarioBufferData65_TMO_LDR_TRANSFORM, 160, gScenarioKernelData65_TMO_LDR_TRANSFORM, 4},
{10, 10, 3736, 48, 0, gScenarioBufferData66_TMO_LDR_TRANSFORM, 160, gScenarioKernelData66_TMO_LDR_TRANSFORM, 4},
{2, 12, 2064, 48, 0, gScenarioBufferData67_TMO_LDR_TRANSFORM, 160, gScenarioKernelData67_TMO_LDR_TRANSFORM, 4},
{4, 12, 2592, 48, 0, gScenarioBufferData68_TMO_LDR_TRANSFORM, 160, gScenarioKernelData68_TMO_LDR_TRANSFORM, 4},
{6, 12, 3120, 48, 0, gScenarioBufferData69_TMO_LDR_TRANSFORM, 160, gScenarioKernelData69_TMO_LDR_TRANSFORM, 4},
{8, 12, 3648, 48, 0, gScenarioBufferData70_TMO_LDR_TRANSFORM, 160, gScenarioKernelData70_TMO_LDR_TRANSFORM, 4},
{2, 14, 2152, 48, 0, gScenarioBufferData71_TMO_LDR_TRANSFORM, 160, gScenarioKernelData71_TMO_LDR_TRANSFORM, 4},
{4, 14, 2768, 48, 0, gScenarioBufferData72_TMO_LDR_TRANSFORM, 160, gScenarioKernelData72_TMO_LDR_TRANSFORM, 4},
{6, 14, 3384, 48, 0, gScenarioBufferData73_TMO_LDR_TRANSFORM, 160, gScenarioKernelData73_TMO_LDR_TRANSFORM, 4},
{2, 16, 2240, 48, 0, gScenarioBufferData74_TMO_LDR_TRANSFORM, 160, gScenarioKernelData74_TMO_LDR_TRANSFORM, 4},
{4, 16, 2944, 48, 0, gScenarioBufferData75_TMO_LDR_TRANSFORM, 160, gScenarioKernelData75_TMO_LDR_TRANSFORM, 4},
{6, 16, 3648, 48, 0, gScenarioBufferData76_TMO_LDR_TRANSFORM, 160, gScenarioKernelData76_TMO_LDR_TRANSFORM, 4},
{2, 18, 2328, 48, 0, gScenarioBufferData77_TMO_LDR_TRANSFORM, 160, gScenarioKernelData77_TMO_LDR_TRANSFORM, 4},
{4, 18, 3120, 48, 0, gScenarioBufferData78_TMO_LDR_TRANSFORM, 160, gScenarioKernelData78_TMO_LDR_TRANSFORM, 4},
{2, 20, 2416, 48, 0, gScenarioBufferData79_TMO_LDR_TRANSFORM, 160, gScenarioKernelData79_TMO_LDR_TRANSFORM, 4},
{4, 20, 3296, 48, 0, gScenarioBufferData80_TMO_LDR_TRANSFORM, 160, gScenarioKernelData80_TMO_LDR_TRANSFORM, 4},
{2, 22, 2504, 48, 0, gScenarioBufferData81_TMO_LDR_TRANSFORM, 160, gScenarioKernelData81_TMO_LDR_TRANSFORM, 4},
{4, 22, 3472, 48, 0, gScenarioBufferData82_TMO_LDR_TRANSFORM, 160, gScenarioKernelData82_TMO_LDR_TRANSFORM, 4},
{2, 24, 2592, 48, 0, gScenarioBufferData83_TMO_LDR_TRANSFORM, 160, gScenarioKernelData83_TMO_LDR_TRANSFORM, 4},
{4, 24, 3648, 48, 0, gScenarioBufferData84_TMO_LDR_TRANSFORM, 160, gScenarioKernelData84_TMO_LDR_TRANSFORM, 4},
{2, 26, 2680, 48, 0, gScenarioBufferData85_TMO_LDR_TRANSFORM, 160, gScenarioKernelData85_TMO_LDR_TRANSFORM, 4},
{4, 26, 3824, 48, 0, gScenarioBufferData86_TMO_LDR_TRANSFORM, 160, gScenarioKernelData86_TMO_LDR_TRANSFORM, 4},
{2, 28, 2768, 48, 0, gScenarioBufferData87_TMO_LDR_TRANSFORM, 160, gScenarioKernelData87_TMO_LDR_TRANSFORM, 4},
{2, 30, 2856, 48, 0, gScenarioBufferData88_TMO_LDR_TRANSFORM, 160, gScenarioKernelData88_TMO_LDR_TRANSFORM, 4},
{2, 32, 2944, 48, 0, gScenarioBufferData89_TMO_LDR_TRANSFORM, 160, gScenarioKernelData89_TMO_LDR_TRANSFORM, 4}
};

static acf_scenario_list gScenarioList_TMO_LDR_TRANSFORM = {
90, //number of scenarios
gScenarioArray_TMO_LDR_TRANSFORM};
//**************************************************************

class TMO_LDR_TRANSFORM : public ACF_Process_APU
{

public:
   TMO_LDR_TRANSFORM(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("TMO_LDR_TRANSFORM");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("TMO_LDR_TRANSFORM",
                                        TMO_LDR_TRANSFORM_LOAD_SEGMENTS,
                                        TMO_LDR_TRANSFORM_LOAD_PMEM, TMO_LDR_TRANSFORM_LOAD_PMEM_SIZE,
                                        TMO_LDR_TRANSFORM_LOAD_DMEM, TMO_LDR_TRANSFORM_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(TMO_LDR_TRANSFORM_APEX_LOG_BUFFER, TMO_LDR_TRANSFORM_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INTERMEDIATE_EXP", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INTERMEDIATE_MAN", icp::DATATYPE_08U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("WORLD_EXP", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0);
         AddPort("WORLD_MAN", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0);
         AddPort("GEO_MEAN_EXP", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("DIV_VALUE", icp::DATATYPE_16U, 2, 1, 1, 1, 0, 1, 1, 0, 1, 0, 8, 0, 0, 0, 0, 0);
         AddPort("KEY_VALUE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("OUTPUT", icp::DATATYPE_08U, 3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_TMO_LDR_TRANSFORM);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_TMO_LDR_TRANSFORM

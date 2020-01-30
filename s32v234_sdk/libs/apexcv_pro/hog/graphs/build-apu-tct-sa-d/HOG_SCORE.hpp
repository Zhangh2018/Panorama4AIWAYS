#ifndef _ACF_PROCESS_APU_HOG_SCORE
#define _ACF_PROCESS_APU_HOG_SCORE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HOG_SCORE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HOG_SCORE[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 1, 1, {0, 0, 0, 0}}, {1, 1, 3, 1, {0, 1, 1, 0}}, {1, 32, 1, 2, {0, 0, 0, 0}}, {1, 1, 2, 2, {0, 0, 0, 0}}, {16, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HOG_SCORE[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 1, 1, {0, 0, 0, 0}}, {2, 1, 3, 1, {0, 1, 1, 0}}, {2, 32, 1, 2, {0, 0, 0, 0}}, {2, 1, 2, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {2, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {2, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {8, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HOG_SCORE[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 1, 1, {0, 0, 0, 0}}, {3, 1, 3, 1, {0, 1, 1, 0}}, {3, 32, 1, 2, {0, 0, 0, 0}}, {3, 1, 2, 2, {0, 0, 0, 0}}, {48, 1, 1, 2, {0, 0, 0, 0}}, {3, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {3, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {12, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HOG_SCORE[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 1, 1, {0, 0, 0, 0}}, {4, 1, 3, 1, {0, 1, 1, 0}}, {4, 32, 1, 2, {0, 0, 0, 0}}, {4, 1, 2, 2, {0, 0, 0, 0}}, {64, 1, 1, 2, {0, 0, 0, 0}}, {4, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {16, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HOG_SCORE[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 1, 1, {0, 0, 0, 0}}, {5, 1, 3, 1, {0, 1, 1, 0}}, {5, 32, 1, 2, {0, 0, 0, 0}}, {5, 1, 2, 2, {0, 0, 0, 0}}, {80, 1, 1, 2, {0, 0, 0, 0}}, {5, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {5, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {20, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HOG_SCORE[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 1, 1, {0, 0, 0, 0}}, {6, 1, 3, 1, {0, 1, 1, 0}}, {6, 32, 1, 2, {0, 0, 0, 0}}, {6, 1, 2, 2, {0, 0, 0, 0}}, {96, 1, 1, 2, {0, 0, 0, 0}}, {6, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {6, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {24, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HOG_SCORE[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 1, 1, {0, 0, 0, 0}}, {7, 1, 3, 1, {0, 1, 1, 0}}, {7, 32, 1, 2, {0, 0, 0, 0}}, {7, 1, 2, 2, {0, 0, 0, 0}}, {112, 1, 1, 2, {0, 0, 0, 0}}, {7, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {7, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {28, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HOG_SCORE[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {8, 1, 3, 1, {0, 1, 1, 0}}, {8, 32, 1, 2, {0, 0, 0, 0}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {128, 1, 1, 2, {0, 0, 0, 0}}, {8, 1, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {8, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HOG_SCORE[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 1, 1, {0, 0, 0, 0}}, {1, 2, 3, 1, {0, 1, 1, 0}}, {1, 64, 1, 2, {0, 0, 0, 0}}, {1, 2, 2, 2, {0, 0, 0, 0}}, {16, 2, 1, 2, {0, 0, 0, 0}}, {1, 2, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 2, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 8, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HOG_SCORE[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 1, 1, {0, 0, 0, 0}}, {2, 2, 3, 1, {0, 1, 1, 0}}, {2, 64, 1, 2, {0, 0, 0, 0}}, {2, 2, 2, 2, {0, 0, 0, 0}}, {32, 2, 1, 2, {0, 0, 0, 0}}, {2, 2, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {2, 2, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {8, 8, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HOG_SCORE[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 1, 1, {0, 0, 0, 0}}, {3, 2, 3, 1, {0, 1, 1, 0}}, {3, 64, 1, 2, {0, 0, 0, 0}}, {3, 2, 2, 2, {0, 0, 0, 0}}, {48, 2, 1, 2, {0, 0, 0, 0}}, {3, 2, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {3, 2, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {12, 8, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HOG_SCORE[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 1, 1, {0, 0, 0, 0}}, {4, 2, 3, 1, {0, 1, 1, 0}}, {4, 64, 1, 2, {0, 0, 0, 0}}, {4, 2, 2, 2, {0, 0, 0, 0}}, {64, 2, 1, 2, {0, 0, 0, 0}}, {4, 2, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 2, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {16, 8, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HOG_SCORE[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 1, 1, {0, 0, 0, 0}}, {1, 3, 3, 1, {0, 1, 1, 0}}, {1, 96, 1, 2, {0, 0, 0, 0}}, {1, 3, 2, 2, {0, 0, 0, 0}}, {16, 3, 1, 2, {0, 0, 0, 0}}, {1, 3, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 3, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 12, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HOG_SCORE[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 1, 1, {0, 0, 0, 0}}, {2, 3, 3, 1, {0, 1, 1, 0}}, {2, 96, 1, 2, {0, 0, 0, 0}}, {2, 3, 2, 2, {0, 0, 0, 0}}, {32, 3, 1, 2, {0, 0, 0, 0}}, {2, 3, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {2, 3, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {8, 12, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HOG_SCORE[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 1, 1, {0, 0, 0, 0}}, {1, 4, 3, 1, {0, 1, 1, 0}}, {1, 128, 1, 2, {0, 0, 0, 0}}, {1, 4, 2, 2, {0, 0, 0, 0}}, {16, 4, 1, 2, {0, 0, 0, 0}}, {1, 4, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 4, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 16, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HOG_SCORE[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 1, 1, {0, 0, 0, 0}}, {2, 4, 3, 1, {0, 1, 1, 0}}, {2, 128, 1, 2, {0, 0, 0, 0}}, {2, 4, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 2, {0, 0, 0, 0}}, {2, 4, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {2, 4, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {8, 16, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HOG_SCORE[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 1, 1, {0, 0, 0, 0}}, {1, 5, 3, 1, {0, 1, 1, 0}}, {1, 160, 1, 2, {0, 0, 0, 0}}, {1, 5, 2, 2, {0, 0, 0, 0}}, {16, 5, 1, 2, {0, 0, 0, 0}}, {1, 5, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 5, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 20, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HOG_SCORE[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 1, 1, {0, 0, 0, 0}}, {1, 6, 3, 1, {0, 1, 1, 0}}, {1, 192, 1, 2, {0, 0, 0, 0}}, {1, 6, 2, 2, {0, 0, 0, 0}}, {16, 6, 1, 2, {0, 0, 0, 0}}, {1, 6, 1, 2, {0, 0, 0, 0}}, {32, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {1, 6, 1, 2, {0, 0, 0, 0}}, {1, 1, 1, 2, {0, 0, 0, 0}}, {4, 24, 1, 1, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HOG_SCORE[] = {{1, 0}, {1, 0}, {2, 0}, {2, 0}};

static acf_scenario gScenarioArray_HOG_SCORE[] = {
{1, 1, 568, 2208, 2, gScenarioBufferData0_HOG_SCORE, 256, gScenarioKernelData0_HOG_SCORE, 16},
{2, 1, 996, 2224, 2, gScenarioBufferData1_HOG_SCORE, 256, gScenarioKernelData1_HOG_SCORE, 16},
{3, 1, 1428, 2240, 2, gScenarioBufferData2_HOG_SCORE, 256, gScenarioKernelData2_HOG_SCORE, 16},
{4, 1, 1856, 2256, 2, gScenarioBufferData3_HOG_SCORE, 256, gScenarioKernelData3_HOG_SCORE, 16},
{5, 1, 2288, 2272, 2, gScenarioBufferData4_HOG_SCORE, 256, gScenarioKernelData4_HOG_SCORE, 16},
{6, 1, 2716, 2288, 2, gScenarioBufferData5_HOG_SCORE, 256, gScenarioKernelData5_HOG_SCORE, 16},
{7, 1, 3148, 2304, 2, gScenarioBufferData6_HOG_SCORE, 256, gScenarioKernelData6_HOG_SCORE, 16},
{8, 1, 3576, 2320, 2, gScenarioBufferData7_HOG_SCORE, 256, gScenarioKernelData7_HOG_SCORE, 16},
{1, 2, 1084, 2240, 2, gScenarioBufferData8_HOG_SCORE, 256, gScenarioKernelData8_HOG_SCORE, 16},
{2, 2, 1920, 2272, 2, gScenarioBufferData9_HOG_SCORE, 256, gScenarioKernelData9_HOG_SCORE, 16},
{3, 2, 2756, 2304, 2, gScenarioBufferData10_HOG_SCORE, 256, gScenarioKernelData10_HOG_SCORE, 16},
{4, 2, 3592, 2336, 2, gScenarioBufferData11_HOG_SCORE, 256, gScenarioKernelData11_HOG_SCORE, 16},
{1, 3, 1604, 2272, 2, gScenarioBufferData12_HOG_SCORE, 256, gScenarioKernelData12_HOG_SCORE, 16},
{2, 3, 2844, 2320, 2, gScenarioBufferData13_HOG_SCORE, 256, gScenarioKernelData13_HOG_SCORE, 16},
{1, 4, 2120, 2304, 2, gScenarioBufferData14_HOG_SCORE, 256, gScenarioKernelData14_HOG_SCORE, 16},
{2, 4, 3768, 2368, 2, gScenarioBufferData15_HOG_SCORE, 256, gScenarioKernelData15_HOG_SCORE, 16},
{1, 5, 2640, 2336, 2, gScenarioBufferData16_HOG_SCORE, 256, gScenarioKernelData16_HOG_SCORE, 16},
{1, 6, 3156, 2368, 2, gScenarioBufferData17_HOG_SCORE, 256, gScenarioKernelData17_HOG_SCORE, 16}
};

static acf_scenario_list gScenarioList_HOG_SCORE = {
18, //number of scenarios
gScenarioArray_HOG_SCORE};
//**************************************************************

class HOG_SCORE : public ACF_Process_APU
{

public:
   HOG_SCORE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HOG_SCORE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HOG_SCORE",
                                        HOG_SCORE_LOAD_SEGMENTS,
                                        HOG_SCORE_LOAD_PMEM, HOG_SCORE_LOAD_PMEM_SIZE,
                                        HOG_SCORE_LOAD_DMEM, HOG_SCORE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HOG_SCORE_APEX_LOG_BUFFER, HOG_SCORE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("IMAGE", icp::DATATYPE_08U, 1, 1, 4, 4, 0, 0, 0, 0, 0, 0, 0, 1, 5, 5, 1, 0);
         AddPort("SVM_FUNCTION", icp::DATATYPE_08S, 1, 1, 2050, 1, 0, 1, 1, 0, 1, 0, 13, 0, 0, 0, 0, 0);
         AddPort("SVM_TRANSFORM_MODE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 14, 0, 0, 0, 0, 0);
         AddPort("WIN_BLOCKS", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 15, 0, 0, 0, 0, 0);
         AddPort("SCORES", icp::DATATYPE_16S, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HOG_SCORE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HOG_SCORE

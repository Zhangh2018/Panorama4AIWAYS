#ifndef _ACF_PROCESS_APU_HOG_DESCRIPTOR
#define _ACF_PROCESS_APU_HOG_DESCRIPTOR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HOG_DESCRIPTOR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HOG_DESCRIPTOR[] = {{4, 4, 4, 0, {1, 1, 2, 2}}, {4, 4, 1, 1, {0, 0, 0, 0}}, {1, 1, 3, 1, {0, 1, 1, 0}}, {8, 1, 2, 2, {0, 0, 0, 0}}, {4, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HOG_DESCRIPTOR[] = {{8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 1, 1, {0, 0, 0, 0}}, {2, 1, 3, 1, {0, 1, 1, 0}}, {16, 1, 2, 2, {0, 0, 0, 0}}, {8, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HOG_DESCRIPTOR[] = {{12, 4, 4, 0, {1, 1, 2, 2}}, {12, 4, 1, 1, {0, 0, 0, 0}}, {3, 1, 3, 1, {0, 1, 1, 0}}, {24, 1, 2, 2, {0, 0, 0, 0}}, {12, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HOG_DESCRIPTOR[] = {{16, 4, 4, 0, {1, 1, 2, 2}}, {16, 4, 1, 1, {0, 0, 0, 0}}, {4, 1, 3, 1, {0, 1, 1, 0}}, {32, 1, 2, 2, {0, 0, 0, 0}}, {16, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HOG_DESCRIPTOR[] = {{20, 4, 4, 0, {1, 1, 2, 2}}, {20, 4, 1, 1, {0, 0, 0, 0}}, {5, 1, 3, 1, {0, 1, 1, 0}}, {40, 1, 2, 2, {0, 0, 0, 0}}, {20, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HOG_DESCRIPTOR[] = {{24, 4, 4, 0, {1, 1, 2, 2}}, {24, 4, 1, 1, {0, 0, 0, 0}}, {6, 1, 3, 1, {0, 1, 1, 0}}, {48, 1, 2, 2, {0, 0, 0, 0}}, {24, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HOG_DESCRIPTOR[] = {{28, 4, 4, 0, {1, 1, 2, 2}}, {28, 4, 1, 1, {0, 0, 0, 0}}, {7, 1, 3, 1, {0, 1, 1, 0}}, {56, 1, 2, 2, {0, 0, 0, 0}}, {28, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HOG_DESCRIPTOR[] = {{32, 4, 4, 0, {1, 1, 2, 2}}, {32, 4, 1, 1, {0, 0, 0, 0}}, {8, 1, 3, 1, {0, 1, 1, 0}}, {64, 1, 2, 2, {0, 0, 0, 0}}, {32, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HOG_DESCRIPTOR[] = {{36, 4, 4, 0, {1, 1, 2, 2}}, {36, 4, 1, 1, {0, 0, 0, 0}}, {9, 1, 3, 1, {0, 1, 1, 0}}, {72, 1, 2, 2, {0, 0, 0, 0}}, {36, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HOG_DESCRIPTOR[] = {{40, 4, 4, 0, {1, 1, 2, 2}}, {40, 4, 1, 1, {0, 0, 0, 0}}, {10, 1, 3, 1, {0, 1, 1, 0}}, {80, 1, 2, 2, {0, 0, 0, 0}}, {40, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HOG_DESCRIPTOR[] = {{44, 4, 4, 0, {1, 1, 2, 2}}, {44, 4, 1, 1, {0, 0, 0, 0}}, {11, 1, 3, 1, {0, 1, 1, 0}}, {88, 1, 2, 2, {0, 0, 0, 0}}, {44, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HOG_DESCRIPTOR[] = {{48, 4, 4, 0, {1, 1, 2, 2}}, {48, 4, 1, 1, {0, 0, 0, 0}}, {12, 1, 3, 1, {0, 1, 1, 0}}, {96, 1, 2, 2, {0, 0, 0, 0}}, {48, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HOG_DESCRIPTOR[] = {{52, 4, 4, 0, {1, 1, 2, 2}}, {52, 4, 1, 1, {0, 0, 0, 0}}, {13, 1, 3, 1, {0, 1, 1, 0}}, {104, 1, 2, 2, {0, 0, 0, 0}}, {52, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HOG_DESCRIPTOR[] = {{56, 4, 4, 0, {1, 1, 2, 2}}, {56, 4, 1, 1, {0, 0, 0, 0}}, {14, 1, 3, 1, {0, 1, 1, 0}}, {112, 1, 2, 2, {0, 0, 0, 0}}, {56, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HOG_DESCRIPTOR[] = {{60, 4, 4, 0, {1, 1, 2, 2}}, {60, 4, 1, 1, {0, 0, 0, 0}}, {15, 1, 3, 1, {0, 1, 1, 0}}, {120, 1, 2, 2, {0, 0, 0, 0}}, {60, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HOG_DESCRIPTOR[] = {{64, 4, 4, 0, {1, 1, 2, 2}}, {64, 4, 1, 1, {0, 0, 0, 0}}, {16, 1, 3, 1, {0, 1, 1, 0}}, {128, 1, 2, 2, {0, 0, 0, 0}}, {64, 4, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HOG_DESCRIPTOR[] = {{4, 8, 4, 0, {1, 1, 2, 2}}, {4, 8, 1, 1, {0, 0, 0, 0}}, {1, 2, 3, 1, {0, 1, 1, 0}}, {8, 2, 2, 2, {0, 0, 0, 0}}, {4, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HOG_DESCRIPTOR[] = {{8, 8, 4, 0, {1, 1, 2, 2}}, {8, 8, 1, 1, {0, 0, 0, 0}}, {2, 2, 3, 1, {0, 1, 1, 0}}, {16, 2, 2, 2, {0, 0, 0, 0}}, {8, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HOG_DESCRIPTOR[] = {{12, 8, 4, 0, {1, 1, 2, 2}}, {12, 8, 1, 1, {0, 0, 0, 0}}, {3, 2, 3, 1, {0, 1, 1, 0}}, {24, 2, 2, 2, {0, 0, 0, 0}}, {12, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HOG_DESCRIPTOR[] = {{16, 8, 4, 0, {1, 1, 2, 2}}, {16, 8, 1, 1, {0, 0, 0, 0}}, {4, 2, 3, 1, {0, 1, 1, 0}}, {32, 2, 2, 2, {0, 0, 0, 0}}, {16, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HOG_DESCRIPTOR[] = {{20, 8, 4, 0, {1, 1, 2, 2}}, {20, 8, 1, 1, {0, 0, 0, 0}}, {5, 2, 3, 1, {0, 1, 1, 0}}, {40, 2, 2, 2, {0, 0, 0, 0}}, {20, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HOG_DESCRIPTOR[] = {{24, 8, 4, 0, {1, 1, 2, 2}}, {24, 8, 1, 1, {0, 0, 0, 0}}, {6, 2, 3, 1, {0, 1, 1, 0}}, {48, 2, 2, 2, {0, 0, 0, 0}}, {24, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HOG_DESCRIPTOR[] = {{28, 8, 4, 0, {1, 1, 2, 2}}, {28, 8, 1, 1, {0, 0, 0, 0}}, {7, 2, 3, 1, {0, 1, 1, 0}}, {56, 2, 2, 2, {0, 0, 0, 0}}, {28, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HOG_DESCRIPTOR[] = {{32, 8, 4, 0, {1, 1, 2, 2}}, {32, 8, 1, 1, {0, 0, 0, 0}}, {8, 2, 3, 1, {0, 1, 1, 0}}, {64, 2, 2, 2, {0, 0, 0, 0}}, {32, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HOG_DESCRIPTOR[] = {{36, 8, 4, 0, {1, 1, 2, 2}}, {36, 8, 1, 1, {0, 0, 0, 0}}, {9, 2, 3, 1, {0, 1, 1, 0}}, {72, 2, 2, 2, {0, 0, 0, 0}}, {36, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HOG_DESCRIPTOR[] = {{40, 8, 4, 0, {1, 1, 2, 2}}, {40, 8, 1, 1, {0, 0, 0, 0}}, {10, 2, 3, 1, {0, 1, 1, 0}}, {80, 2, 2, 2, {0, 0, 0, 0}}, {40, 8, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HOG_DESCRIPTOR[] = {{4, 12, 4, 0, {1, 1, 2, 2}}, {4, 12, 1, 1, {0, 0, 0, 0}}, {1, 3, 3, 1, {0, 1, 1, 0}}, {8, 3, 2, 2, {0, 0, 0, 0}}, {4, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HOG_DESCRIPTOR[] = {{8, 12, 4, 0, {1, 1, 2, 2}}, {8, 12, 1, 1, {0, 0, 0, 0}}, {2, 3, 3, 1, {0, 1, 1, 0}}, {16, 3, 2, 2, {0, 0, 0, 0}}, {8, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HOG_DESCRIPTOR[] = {{12, 12, 4, 0, {1, 1, 2, 2}}, {12, 12, 1, 1, {0, 0, 0, 0}}, {3, 3, 3, 1, {0, 1, 1, 0}}, {24, 3, 2, 2, {0, 0, 0, 0}}, {12, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HOG_DESCRIPTOR[] = {{16, 12, 4, 0, {1, 1, 2, 2}}, {16, 12, 1, 1, {0, 0, 0, 0}}, {4, 3, 3, 1, {0, 1, 1, 0}}, {32, 3, 2, 2, {0, 0, 0, 0}}, {16, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HOG_DESCRIPTOR[] = {{20, 12, 4, 0, {1, 1, 2, 2}}, {20, 12, 1, 1, {0, 0, 0, 0}}, {5, 3, 3, 1, {0, 1, 1, 0}}, {40, 3, 2, 2, {0, 0, 0, 0}}, {20, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HOG_DESCRIPTOR[] = {{24, 12, 4, 0, {1, 1, 2, 2}}, {24, 12, 1, 1, {0, 0, 0, 0}}, {6, 3, 3, 1, {0, 1, 1, 0}}, {48, 3, 2, 2, {0, 0, 0, 0}}, {24, 12, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HOG_DESCRIPTOR[] = {{4, 16, 4, 0, {1, 1, 2, 2}}, {4, 16, 1, 1, {0, 0, 0, 0}}, {1, 4, 3, 1, {0, 1, 1, 0}}, {8, 4, 2, 2, {0, 0, 0, 0}}, {4, 16, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_HOG_DESCRIPTOR[] = {{8, 16, 4, 0, {1, 1, 2, 2}}, {8, 16, 1, 1, {0, 0, 0, 0}}, {2, 4, 3, 1, {0, 1, 1, 0}}, {16, 4, 2, 2, {0, 0, 0, 0}}, {8, 16, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_HOG_DESCRIPTOR[] = {{12, 16, 4, 0, {1, 1, 2, 2}}, {12, 16, 1, 1, {0, 0, 0, 0}}, {3, 4, 3, 1, {0, 1, 1, 0}}, {24, 4, 2, 2, {0, 0, 0, 0}}, {12, 16, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_HOG_DESCRIPTOR[] = {{16, 16, 4, 0, {1, 1, 2, 2}}, {16, 16, 1, 1, {0, 0, 0, 0}}, {4, 4, 3, 1, {0, 1, 1, 0}}, {32, 4, 2, 2, {0, 0, 0, 0}}, {16, 16, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_HOG_DESCRIPTOR[] = {{20, 16, 4, 0, {1, 1, 2, 2}}, {20, 16, 1, 1, {0, 0, 0, 0}}, {5, 4, 3, 1, {0, 1, 1, 0}}, {40, 4, 2, 2, {0, 0, 0, 0}}, {20, 16, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_HOG_DESCRIPTOR[] = {{4, 20, 4, 0, {1, 1, 2, 2}}, {4, 20, 1, 1, {0, 0, 0, 0}}, {1, 5, 3, 1, {0, 1, 1, 0}}, {8, 5, 2, 2, {0, 0, 0, 0}}, {4, 20, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_HOG_DESCRIPTOR[] = {{8, 20, 4, 0, {1, 1, 2, 2}}, {8, 20, 1, 1, {0, 0, 0, 0}}, {2, 5, 3, 1, {0, 1, 1, 0}}, {16, 5, 2, 2, {0, 0, 0, 0}}, {8, 20, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_HOG_DESCRIPTOR[] = {{12, 20, 4, 0, {1, 1, 2, 2}}, {12, 20, 1, 1, {0, 0, 0, 0}}, {3, 5, 3, 1, {0, 1, 1, 0}}, {24, 5, 2, 2, {0, 0, 0, 0}}, {12, 20, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_HOG_DESCRIPTOR[] = {{4, 24, 4, 0, {1, 1, 2, 2}}, {4, 24, 1, 1, {0, 0, 0, 0}}, {1, 6, 3, 1, {0, 1, 1, 0}}, {8, 6, 2, 2, {0, 0, 0, 0}}, {4, 24, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_HOG_DESCRIPTOR[] = {{8, 24, 4, 0, {1, 1, 2, 2}}, {8, 24, 1, 1, {0, 0, 0, 0}}, {2, 6, 3, 1, {0, 1, 1, 0}}, {16, 6, 2, 2, {0, 0, 0, 0}}, {8, 24, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_HOG_DESCRIPTOR[] = {{12, 24, 4, 0, {1, 1, 2, 2}}, {12, 24, 1, 1, {0, 0, 0, 0}}, {3, 6, 3, 1, {0, 1, 1, 0}}, {24, 6, 2, 2, {0, 0, 0, 0}}, {12, 24, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_HOG_DESCRIPTOR[] = {{4, 32, 4, 0, {1, 1, 2, 2}}, {4, 32, 1, 1, {0, 0, 0, 0}}, {1, 8, 3, 1, {0, 1, 1, 0}}, {8, 8, 2, 2, {0, 0, 0, 0}}, {4, 32, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_HOG_DESCRIPTOR[] = {{8, 32, 4, 0, {1, 1, 2, 2}}, {8, 32, 1, 1, {0, 0, 0, 0}}, {2, 8, 3, 1, {0, 1, 1, 0}}, {16, 8, 2, 2, {0, 0, 0, 0}}, {8, 32, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_HOG_DESCRIPTOR[] = {{4, 40, 4, 0, {1, 1, 2, 2}}, {4, 40, 1, 1, {0, 0, 0, 0}}, {1, 10, 3, 1, {0, 1, 1, 0}}, {8, 10, 2, 2, {0, 0, 0, 0}}, {4, 40, 1, 1, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_HOG_DESCRIPTOR[] = {{4, 48, 4, 0, {1, 1, 2, 2}}, {4, 48, 1, 1, {0, 0, 0, 0}}, {1, 12, 3, 1, {0, 1, 1, 0}}, {8, 12, 2, 2, {0, 0, 0, 0}}, {4, 48, 1, 1, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_HOG_DESCRIPTOR[] = {{1, 0}, {1, 0}, {2, 0}};

static acf_scenario gScenarioArray_HOG_DESCRIPTOR[] = {
{1, 1, 320, 0, 2, gScenarioBufferData0_HOG_DESCRIPTOR, 80, gScenarioKernelData0_HOG_DESCRIPTOR, 12},
{2, 1, 504, 0, 2, gScenarioBufferData1_HOG_DESCRIPTOR, 80, gScenarioKernelData1_HOG_DESCRIPTOR, 12},
{3, 1, 688, 0, 2, gScenarioBufferData2_HOG_DESCRIPTOR, 80, gScenarioKernelData2_HOG_DESCRIPTOR, 12},
{4, 1, 872, 0, 2, gScenarioBufferData3_HOG_DESCRIPTOR, 80, gScenarioKernelData3_HOG_DESCRIPTOR, 12},
{5, 1, 1056, 0, 2, gScenarioBufferData4_HOG_DESCRIPTOR, 80, gScenarioKernelData4_HOG_DESCRIPTOR, 12},
{6, 1, 1240, 0, 2, gScenarioBufferData5_HOG_DESCRIPTOR, 80, gScenarioKernelData5_HOG_DESCRIPTOR, 12},
{7, 1, 1424, 0, 2, gScenarioBufferData6_HOG_DESCRIPTOR, 80, gScenarioKernelData6_HOG_DESCRIPTOR, 12},
{8, 1, 1608, 0, 2, gScenarioBufferData7_HOG_DESCRIPTOR, 80, gScenarioKernelData7_HOG_DESCRIPTOR, 12},
{9, 1, 1792, 0, 2, gScenarioBufferData8_HOG_DESCRIPTOR, 80, gScenarioKernelData8_HOG_DESCRIPTOR, 12},
{10, 1, 1976, 0, 2, gScenarioBufferData9_HOG_DESCRIPTOR, 80, gScenarioKernelData9_HOG_DESCRIPTOR, 12},
{11, 1, 2160, 0, 2, gScenarioBufferData10_HOG_DESCRIPTOR, 80, gScenarioKernelData10_HOG_DESCRIPTOR, 12},
{12, 1, 2344, 0, 2, gScenarioBufferData11_HOG_DESCRIPTOR, 80, gScenarioKernelData11_HOG_DESCRIPTOR, 12},
{13, 1, 2528, 0, 2, gScenarioBufferData12_HOG_DESCRIPTOR, 80, gScenarioKernelData12_HOG_DESCRIPTOR, 12},
{14, 1, 2712, 0, 2, gScenarioBufferData13_HOG_DESCRIPTOR, 80, gScenarioKernelData13_HOG_DESCRIPTOR, 12},
{15, 1, 2896, 0, 2, gScenarioBufferData14_HOG_DESCRIPTOR, 80, gScenarioKernelData14_HOG_DESCRIPTOR, 12},
{16, 1, 3080, 0, 2, gScenarioBufferData15_HOG_DESCRIPTOR, 80, gScenarioKernelData15_HOG_DESCRIPTOR, 12},
{1, 2, 592, 0, 2, gScenarioBufferData16_HOG_DESCRIPTOR, 80, gScenarioKernelData16_HOG_DESCRIPTOR, 12},
{2, 2, 936, 0, 2, gScenarioBufferData17_HOG_DESCRIPTOR, 80, gScenarioKernelData17_HOG_DESCRIPTOR, 12},
{3, 2, 1280, 0, 2, gScenarioBufferData18_HOG_DESCRIPTOR, 80, gScenarioKernelData18_HOG_DESCRIPTOR, 12},
{4, 2, 1624, 0, 2, gScenarioBufferData19_HOG_DESCRIPTOR, 80, gScenarioKernelData19_HOG_DESCRIPTOR, 12},
{5, 2, 1968, 0, 2, gScenarioBufferData20_HOG_DESCRIPTOR, 80, gScenarioKernelData20_HOG_DESCRIPTOR, 12},
{6, 2, 2312, 0, 2, gScenarioBufferData21_HOG_DESCRIPTOR, 80, gScenarioKernelData21_HOG_DESCRIPTOR, 12},
{7, 2, 2656, 0, 2, gScenarioBufferData22_HOG_DESCRIPTOR, 80, gScenarioKernelData22_HOG_DESCRIPTOR, 12},
{8, 2, 3000, 0, 2, gScenarioBufferData23_HOG_DESCRIPTOR, 80, gScenarioKernelData23_HOG_DESCRIPTOR, 12},
{9, 2, 3344, 0, 2, gScenarioBufferData24_HOG_DESCRIPTOR, 80, gScenarioKernelData24_HOG_DESCRIPTOR, 12},
{10, 2, 3688, 0, 2, gScenarioBufferData25_HOG_DESCRIPTOR, 80, gScenarioKernelData25_HOG_DESCRIPTOR, 12},
{1, 3, 864, 0, 2, gScenarioBufferData26_HOG_DESCRIPTOR, 80, gScenarioKernelData26_HOG_DESCRIPTOR, 12},
{2, 3, 1368, 0, 2, gScenarioBufferData27_HOG_DESCRIPTOR, 80, gScenarioKernelData27_HOG_DESCRIPTOR, 12},
{3, 3, 1872, 0, 2, gScenarioBufferData28_HOG_DESCRIPTOR, 80, gScenarioKernelData28_HOG_DESCRIPTOR, 12},
{4, 3, 2376, 0, 2, gScenarioBufferData29_HOG_DESCRIPTOR, 80, gScenarioKernelData29_HOG_DESCRIPTOR, 12},
{5, 3, 2880, 0, 2, gScenarioBufferData30_HOG_DESCRIPTOR, 80, gScenarioKernelData30_HOG_DESCRIPTOR, 12},
{6, 3, 3384, 0, 2, gScenarioBufferData31_HOG_DESCRIPTOR, 80, gScenarioKernelData31_HOG_DESCRIPTOR, 12},
{1, 4, 1136, 0, 2, gScenarioBufferData32_HOG_DESCRIPTOR, 80, gScenarioKernelData32_HOG_DESCRIPTOR, 12},
{2, 4, 1800, 0, 2, gScenarioBufferData33_HOG_DESCRIPTOR, 80, gScenarioKernelData33_HOG_DESCRIPTOR, 12},
{3, 4, 2464, 0, 2, gScenarioBufferData34_HOG_DESCRIPTOR, 80, gScenarioKernelData34_HOG_DESCRIPTOR, 12},
{4, 4, 3128, 0, 2, gScenarioBufferData35_HOG_DESCRIPTOR, 80, gScenarioKernelData35_HOG_DESCRIPTOR, 12},
{5, 4, 3792, 0, 2, gScenarioBufferData36_HOG_DESCRIPTOR, 80, gScenarioKernelData36_HOG_DESCRIPTOR, 12},
{1, 5, 1408, 0, 2, gScenarioBufferData37_HOG_DESCRIPTOR, 80, gScenarioKernelData37_HOG_DESCRIPTOR, 12},
{2, 5, 2232, 0, 2, gScenarioBufferData38_HOG_DESCRIPTOR, 80, gScenarioKernelData38_HOG_DESCRIPTOR, 12},
{3, 5, 3056, 0, 2, gScenarioBufferData39_HOG_DESCRIPTOR, 80, gScenarioKernelData39_HOG_DESCRIPTOR, 12},
{1, 6, 1680, 0, 2, gScenarioBufferData40_HOG_DESCRIPTOR, 80, gScenarioKernelData40_HOG_DESCRIPTOR, 12},
{2, 6, 2664, 0, 2, gScenarioBufferData41_HOG_DESCRIPTOR, 80, gScenarioKernelData41_HOG_DESCRIPTOR, 12},
{3, 6, 3648, 0, 2, gScenarioBufferData42_HOG_DESCRIPTOR, 80, gScenarioKernelData42_HOG_DESCRIPTOR, 12},
{1, 8, 2224, 0, 2, gScenarioBufferData43_HOG_DESCRIPTOR, 80, gScenarioKernelData43_HOG_DESCRIPTOR, 12},
{2, 8, 3528, 0, 2, gScenarioBufferData44_HOG_DESCRIPTOR, 80, gScenarioKernelData44_HOG_DESCRIPTOR, 12},
{1, 10, 2768, 0, 2, gScenarioBufferData45_HOG_DESCRIPTOR, 80, gScenarioKernelData45_HOG_DESCRIPTOR, 12},
{1, 12, 3312, 0, 2, gScenarioBufferData46_HOG_DESCRIPTOR, 80, gScenarioKernelData46_HOG_DESCRIPTOR, 12}
};

static acf_scenario_list gScenarioList_HOG_DESCRIPTOR = {
47, //number of scenarios
gScenarioArray_HOG_DESCRIPTOR};
//**************************************************************

class HOG_DESCRIPTOR : public ACF_Process_APU
{

public:
   HOG_DESCRIPTOR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HOG_DESCRIPTOR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HOG_DESCRIPTOR",
                                        HOG_DESCRIPTOR_LOAD_SEGMENTS,
                                        HOG_DESCRIPTOR_LOAD_PMEM, HOG_DESCRIPTOR_LOAD_PMEM_SIZE,
                                        HOG_DESCRIPTOR_LOAD_DMEM, HOG_DESCRIPTOR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HOG_DESCRIPTOR_APEX_LOG_BUFFER, HOG_DESCRIPTOR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("IMAGE", icp::DATATYPE_08U, 1, 1, 4, 4, 0, 0, 0, 0, 0, 0, 0, 1, 5, 5, 1, 0);
         AddPort("BLOCK_HISTOGRAMS", icp::DATATYPE_08U, 1, 1, 8, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HOG_DESCRIPTOR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HOG_DESCRIPTOR

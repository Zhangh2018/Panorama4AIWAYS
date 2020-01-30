#ifndef _ACF_PROCESS_APU_AGGCF_HIST_NT4
#define _ACF_PROCESS_APU_AGGCF_HIST_NT4

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <AGGCF_HIST_NT4_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_AGGCF_HIST_NT4[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_AGGCF_HIST_NT4[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_AGGCF_HIST_NT4[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_AGGCF_HIST_NT4[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_AGGCF_HIST_NT4[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_AGGCF_HIST_NT4[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_AGGCF_HIST_NT4[] = {{28, 4, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_AGGCF_HIST_NT4[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_AGGCF_HIST_NT4[] = {{36, 4, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_AGGCF_HIST_NT4[] = {{40, 4, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_AGGCF_HIST_NT4[] = {{44, 4, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_AGGCF_HIST_NT4[] = {{48, 4, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {48, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_AGGCF_HIST_NT4[] = {{52, 4, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {52, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_AGGCF_HIST_NT4[] = {{56, 4, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {14, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {56, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_AGGCF_HIST_NT4[] = {{60, 4, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {15, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {60, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_AGGCF_HIST_NT4[] = {{64, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {64, 4, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_AGGCF_HIST_NT4[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_AGGCF_HIST_NT4[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_AGGCF_HIST_NT4[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_AGGCF_HIST_NT4[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_AGGCF_HIST_NT4[] = {{20, 8, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_AGGCF_HIST_NT4[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_AGGCF_HIST_NT4[] = {{28, 8, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {7, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_AGGCF_HIST_NT4[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_AGGCF_HIST_NT4[] = {{36, 8, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {9, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {36, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_AGGCF_HIST_NT4[] = {{40, 8, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {40, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_AGGCF_HIST_NT4[] = {{44, 8, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {11, 2, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {44, 8, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_AGGCF_HIST_NT4[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_AGGCF_HIST_NT4[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_AGGCF_HIST_NT4[] = {{12, 12, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_AGGCF_HIST_NT4[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_AGGCF_HIST_NT4[] = {{20, 12, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {5, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_AGGCF_HIST_NT4[] = {{24, 12, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {24, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_AGGCF_HIST_NT4[] = {{28, 12, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {7, 3, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {28, 12, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_AGGCF_HIST_NT4[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_AGGCF_HIST_NT4[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_AGGCF_HIST_NT4[] = {{12, 16, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 16, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_AGGCF_HIST_NT4[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_AGGCF_HIST_NT4[] = {{20, 16, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {5, 4, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {20, 16, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_AGGCF_HIST_NT4[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_AGGCF_HIST_NT4[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_AGGCF_HIST_NT4[] = {{12, 20, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {3, 5, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 20, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_AGGCF_HIST_NT4[] = {{16, 20, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 20, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_AGGCF_HIST_NT4[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_AGGCF_HIST_NT4[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_AGGCF_HIST_NT4[] = {{12, 24, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {3, 6, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {12, 24, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_AGGCF_HIST_NT4[] = {{4, 32, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 32, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_AGGCF_HIST_NT4[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_AGGCF_HIST_NT4[] = {{4, 40, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 40, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_AGGCF_HIST_NT4[] = {{8, 40, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {8, 40, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_AGGCF_HIST_NT4[] = {{4, 48, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 48, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_AGGCF_HIST_NT4[] = {{4, 56, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {1, 14, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 56, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_AGGCF_HIST_NT4[] = {{4, 64, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {1, 16, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 64, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_AGGCF_HIST_NT4[] = {{4, 72, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {1, 18, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 72, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_AGGCF_HIST_NT4[] = {{4, 80, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {1, 20, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 80, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_AGGCF_HIST_NT4[] = {{4, 88, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {1, 22, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {4, 88, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_AGGCF_HIST_NT4[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_AGGCF_HIST_NT4[] = {{0, 0}};

static acf_scenario gScenarioArray_AGGCF_HIST_NT4[] = {
{1, 1, 408, 528, 0, gScenarioBufferData0_AGGCF_HIST_NT4, 224, gScenarioKernelData0_AGGCF_HIST_NT4, 4},
{2, 1, 560, 528, 0, gScenarioBufferData1_AGGCF_HIST_NT4, 224, gScenarioKernelData1_AGGCF_HIST_NT4, 4},
{3, 1, 712, 528, 0, gScenarioBufferData2_AGGCF_HIST_NT4, 224, gScenarioKernelData2_AGGCF_HIST_NT4, 4},
{4, 1, 864, 528, 0, gScenarioBufferData3_AGGCF_HIST_NT4, 224, gScenarioKernelData3_AGGCF_HIST_NT4, 4},
{5, 1, 1016, 528, 0, gScenarioBufferData4_AGGCF_HIST_NT4, 224, gScenarioKernelData4_AGGCF_HIST_NT4, 4},
{6, 1, 1168, 528, 0, gScenarioBufferData5_AGGCF_HIST_NT4, 224, gScenarioKernelData5_AGGCF_HIST_NT4, 4},
{7, 1, 1320, 528, 0, gScenarioBufferData6_AGGCF_HIST_NT4, 224, gScenarioKernelData6_AGGCF_HIST_NT4, 4},
{8, 1, 1472, 528, 0, gScenarioBufferData7_AGGCF_HIST_NT4, 224, gScenarioKernelData7_AGGCF_HIST_NT4, 4},
{9, 1, 1624, 528, 0, gScenarioBufferData8_AGGCF_HIST_NT4, 224, gScenarioKernelData8_AGGCF_HIST_NT4, 4},
{10, 1, 1776, 528, 0, gScenarioBufferData9_AGGCF_HIST_NT4, 224, gScenarioKernelData9_AGGCF_HIST_NT4, 4},
{11, 1, 1928, 528, 0, gScenarioBufferData10_AGGCF_HIST_NT4, 224, gScenarioKernelData10_AGGCF_HIST_NT4, 4},
{12, 1, 2080, 528, 0, gScenarioBufferData11_AGGCF_HIST_NT4, 224, gScenarioKernelData11_AGGCF_HIST_NT4, 4},
{13, 1, 2232, 528, 0, gScenarioBufferData12_AGGCF_HIST_NT4, 224, gScenarioKernelData12_AGGCF_HIST_NT4, 4},
{14, 1, 2384, 528, 0, gScenarioBufferData13_AGGCF_HIST_NT4, 224, gScenarioKernelData13_AGGCF_HIST_NT4, 4},
{15, 1, 2536, 528, 0, gScenarioBufferData14_AGGCF_HIST_NT4, 224, gScenarioKernelData14_AGGCF_HIST_NT4, 4},
{16, 1, 2688, 528, 0, gScenarioBufferData15_AGGCF_HIST_NT4, 224, gScenarioKernelData15_AGGCF_HIST_NT4, 4},
{1, 2, 560, 528, 0, gScenarioBufferData16_AGGCF_HIST_NT4, 224, gScenarioKernelData16_AGGCF_HIST_NT4, 4},
{2, 2, 864, 528, 0, gScenarioBufferData17_AGGCF_HIST_NT4, 224, gScenarioKernelData17_AGGCF_HIST_NT4, 4},
{3, 2, 1168, 528, 0, gScenarioBufferData18_AGGCF_HIST_NT4, 224, gScenarioKernelData18_AGGCF_HIST_NT4, 4},
{4, 2, 1472, 528, 0, gScenarioBufferData19_AGGCF_HIST_NT4, 224, gScenarioKernelData19_AGGCF_HIST_NT4, 4},
{5, 2, 1776, 528, 0, gScenarioBufferData20_AGGCF_HIST_NT4, 224, gScenarioKernelData20_AGGCF_HIST_NT4, 4},
{6, 2, 2080, 528, 0, gScenarioBufferData21_AGGCF_HIST_NT4, 224, gScenarioKernelData21_AGGCF_HIST_NT4, 4},
{7, 2, 2384, 528, 0, gScenarioBufferData22_AGGCF_HIST_NT4, 224, gScenarioKernelData22_AGGCF_HIST_NT4, 4},
{8, 2, 2688, 528, 0, gScenarioBufferData23_AGGCF_HIST_NT4, 224, gScenarioKernelData23_AGGCF_HIST_NT4, 4},
{9, 2, 2992, 528, 0, gScenarioBufferData24_AGGCF_HIST_NT4, 224, gScenarioKernelData24_AGGCF_HIST_NT4, 4},
{10, 2, 3296, 528, 0, gScenarioBufferData25_AGGCF_HIST_NT4, 224, gScenarioKernelData25_AGGCF_HIST_NT4, 4},
{11, 2, 3600, 528, 0, gScenarioBufferData26_AGGCF_HIST_NT4, 224, gScenarioKernelData26_AGGCF_HIST_NT4, 4},
{1, 3, 712, 528, 0, gScenarioBufferData27_AGGCF_HIST_NT4, 224, gScenarioKernelData27_AGGCF_HIST_NT4, 4},
{2, 3, 1168, 528, 0, gScenarioBufferData28_AGGCF_HIST_NT4, 224, gScenarioKernelData28_AGGCF_HIST_NT4, 4},
{3, 3, 1624, 528, 0, gScenarioBufferData29_AGGCF_HIST_NT4, 224, gScenarioKernelData29_AGGCF_HIST_NT4, 4},
{4, 3, 2080, 528, 0, gScenarioBufferData30_AGGCF_HIST_NT4, 224, gScenarioKernelData30_AGGCF_HIST_NT4, 4},
{5, 3, 2536, 528, 0, gScenarioBufferData31_AGGCF_HIST_NT4, 224, gScenarioKernelData31_AGGCF_HIST_NT4, 4},
{6, 3, 2992, 528, 0, gScenarioBufferData32_AGGCF_HIST_NT4, 224, gScenarioKernelData32_AGGCF_HIST_NT4, 4},
{7, 3, 3448, 528, 0, gScenarioBufferData33_AGGCF_HIST_NT4, 224, gScenarioKernelData33_AGGCF_HIST_NT4, 4},
{1, 4, 864, 528, 0, gScenarioBufferData34_AGGCF_HIST_NT4, 224, gScenarioKernelData34_AGGCF_HIST_NT4, 4},
{2, 4, 1472, 528, 0, gScenarioBufferData35_AGGCF_HIST_NT4, 224, gScenarioKernelData35_AGGCF_HIST_NT4, 4},
{3, 4, 2080, 528, 0, gScenarioBufferData36_AGGCF_HIST_NT4, 224, gScenarioKernelData36_AGGCF_HIST_NT4, 4},
{4, 4, 2688, 528, 0, gScenarioBufferData37_AGGCF_HIST_NT4, 224, gScenarioKernelData37_AGGCF_HIST_NT4, 4},
{5, 4, 3296, 528, 0, gScenarioBufferData38_AGGCF_HIST_NT4, 224, gScenarioKernelData38_AGGCF_HIST_NT4, 4},
{1, 5, 1016, 528, 0, gScenarioBufferData39_AGGCF_HIST_NT4, 224, gScenarioKernelData39_AGGCF_HIST_NT4, 4},
{2, 5, 1776, 528, 0, gScenarioBufferData40_AGGCF_HIST_NT4, 224, gScenarioKernelData40_AGGCF_HIST_NT4, 4},
{3, 5, 2536, 528, 0, gScenarioBufferData41_AGGCF_HIST_NT4, 224, gScenarioKernelData41_AGGCF_HIST_NT4, 4},
{4, 5, 3296, 528, 0, gScenarioBufferData42_AGGCF_HIST_NT4, 224, gScenarioKernelData42_AGGCF_HIST_NT4, 4},
{1, 6, 1168, 528, 0, gScenarioBufferData43_AGGCF_HIST_NT4, 224, gScenarioKernelData43_AGGCF_HIST_NT4, 4},
{2, 6, 2080, 528, 0, gScenarioBufferData44_AGGCF_HIST_NT4, 224, gScenarioKernelData44_AGGCF_HIST_NT4, 4},
{3, 6, 2992, 528, 0, gScenarioBufferData45_AGGCF_HIST_NT4, 224, gScenarioKernelData45_AGGCF_HIST_NT4, 4},
{1, 8, 1472, 528, 0, gScenarioBufferData46_AGGCF_HIST_NT4, 224, gScenarioKernelData46_AGGCF_HIST_NT4, 4},
{2, 8, 2688, 528, 0, gScenarioBufferData47_AGGCF_HIST_NT4, 224, gScenarioKernelData47_AGGCF_HIST_NT4, 4},
{1, 10, 1776, 528, 0, gScenarioBufferData48_AGGCF_HIST_NT4, 224, gScenarioKernelData48_AGGCF_HIST_NT4, 4},
{2, 10, 3296, 528, 0, gScenarioBufferData49_AGGCF_HIST_NT4, 224, gScenarioKernelData49_AGGCF_HIST_NT4, 4},
{1, 12, 2080, 528, 0, gScenarioBufferData50_AGGCF_HIST_NT4, 224, gScenarioKernelData50_AGGCF_HIST_NT4, 4},
{1, 14, 2384, 528, 0, gScenarioBufferData51_AGGCF_HIST_NT4, 224, gScenarioKernelData51_AGGCF_HIST_NT4, 4},
{1, 16, 2688, 528, 0, gScenarioBufferData52_AGGCF_HIST_NT4, 224, gScenarioKernelData52_AGGCF_HIST_NT4, 4},
{1, 18, 2992, 528, 0, gScenarioBufferData53_AGGCF_HIST_NT4, 224, gScenarioKernelData53_AGGCF_HIST_NT4, 4},
{1, 20, 3296, 528, 0, gScenarioBufferData54_AGGCF_HIST_NT4, 224, gScenarioKernelData54_AGGCF_HIST_NT4, 4},
{1, 22, 3600, 528, 0, gScenarioBufferData55_AGGCF_HIST_NT4, 224, gScenarioKernelData55_AGGCF_HIST_NT4, 4}
};

static acf_scenario_list gScenarioList_AGGCF_HIST_NT4 = {
56, //number of scenarios
gScenarioArray_AGGCF_HIST_NT4};
//**************************************************************

class AGGCF_HIST_NT4 : public ACF_Process_APU
{

public:
   AGGCF_HIST_NT4(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("AGGCF_HIST_NT4");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("AGGCF_HIST_NT4",
                                        AGGCF_HIST_NT4_LOAD_SEGMENTS,
                                        AGGCF_HIST_NT4_LOAD_PMEM, AGGCF_HIST_NT4_LOAD_PMEM_SIZE,
                                        AGGCF_HIST_NT4_LOAD_DMEM, AGGCF_HIST_NT4_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(AGGCF_HIST_NT4_APEX_LOG_BUFFER, AGGCF_HIST_NT4_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUTM", icp::DATATYPE_16U, 1, 1, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INPUTO", icp::DATATYPE_16U, 1, 1, 4, 4, 0, 0, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0);
         AddPort("OFFSET", icp::DATATYPE_32S, 1, 1, 128, 1, 0, 1, 1, 0, 1, 0, 12, 0, 0, 0, 0, 0);
         AddPort("PARAMS", icp::DATATYPE_32S, 1, 1, 4, 1, 0, 1, 1, 0, 1, 0, 13, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH1", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH2", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH3", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH4", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH5", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0);
         AddPort("OUTPUTH6", icp::DATATYPE_16U, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_AGGCF_HIST_NT4);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_AGGCF_HIST_NT4

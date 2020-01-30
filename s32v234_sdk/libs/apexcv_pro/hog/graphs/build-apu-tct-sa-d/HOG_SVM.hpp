#ifndef _ACF_PROCESS_APU_HOG_SVM
#define _ACF_PROCESS_APU_HOG_SVM

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <HOG_SVM_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_HOG_SVM[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {1, 32, 1, 0, {0, 0, 0, 0}}, {1, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_HOG_SVM[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {2, 32, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_HOG_SVM[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {3, 32, 1, 0, {0, 0, 0, 0}}, {3, 1, 2, 0, {0, 0, 0, 0}}, {48, 1, 1, 0, {0, 0, 0, 0}}, {3, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {3, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_HOG_SVM[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {4, 32, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {64, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {4, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_HOG_SVM[] = {{40, 1, 2, 0, {0, 0, 0, 0}}, {5, 32, 1, 0, {0, 0, 0, 0}}, {5, 1, 2, 0, {0, 0, 0, 0}}, {80, 1, 1, 0, {0, 0, 0, 0}}, {5, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {5, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_HOG_SVM[] = {{48, 1, 2, 0, {0, 0, 0, 0}}, {6, 32, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {96, 1, 1, 0, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {6, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_HOG_SVM[] = {{56, 1, 2, 0, {0, 0, 0, 0}}, {7, 32, 1, 0, {0, 0, 0, 0}}, {7, 1, 2, 0, {0, 0, 0, 0}}, {112, 1, 1, 0, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {7, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_HOG_SVM[] = {{64, 1, 2, 0, {0, 0, 0, 0}}, {8, 32, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {128, 1, 1, 0, {0, 0, 0, 0}}, {8, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {8, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_HOG_SVM[] = {{72, 1, 2, 0, {0, 0, 0, 0}}, {9, 32, 1, 0, {0, 0, 0, 0}}, {9, 1, 2, 0, {0, 0, 0, 0}}, {144, 1, 1, 0, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {9, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_HOG_SVM[] = {{80, 1, 2, 0, {0, 0, 0, 0}}, {10, 32, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {160, 1, 1, 0, {0, 0, 0, 0}}, {10, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {10, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_HOG_SVM[] = {{88, 1, 2, 0, {0, 0, 0, 0}}, {11, 32, 1, 0, {0, 0, 0, 0}}, {11, 1, 2, 0, {0, 0, 0, 0}}, {176, 1, 1, 0, {0, 0, 0, 0}}, {11, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {11, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_HOG_SVM[] = {{96, 1, 2, 0, {0, 0, 0, 0}}, {12, 32, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {192, 1, 1, 0, {0, 0, 0, 0}}, {12, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {12, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_HOG_SVM[] = {{104, 1, 2, 0, {0, 0, 0, 0}}, {13, 32, 1, 0, {0, 0, 0, 0}}, {13, 1, 2, 0, {0, 0, 0, 0}}, {208, 1, 1, 0, {0, 0, 0, 0}}, {13, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {13, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_HOG_SVM[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {1, 64, 1, 0, {0, 0, 0, 0}}, {1, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 1, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_HOG_SVM[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {2, 64, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 1, 0, {0, 0, 0, 0}}, {2, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_HOG_SVM[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {3, 64, 1, 0, {0, 0, 0, 0}}, {3, 2, 2, 0, {0, 0, 0, 0}}, {48, 2, 1, 0, {0, 0, 0, 0}}, {3, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {3, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_HOG_SVM[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {4, 64, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {64, 2, 1, 0, {0, 0, 0, 0}}, {4, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {4, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_HOG_SVM[] = {{40, 2, 2, 0, {0, 0, 0, 0}}, {5, 64, 1, 0, {0, 0, 0, 0}}, {5, 2, 2, 0, {0, 0, 0, 0}}, {80, 2, 1, 0, {0, 0, 0, 0}}, {5, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {5, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_HOG_SVM[] = {{48, 2, 2, 0, {0, 0, 0, 0}}, {6, 64, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {96, 2, 1, 0, {0, 0, 0, 0}}, {6, 2, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {6, 2, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_HOG_SVM[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {1, 96, 1, 0, {0, 0, 0, 0}}, {1, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 1, 0, {0, 0, 0, 0}}, {1, 3, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 3, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_HOG_SVM[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {2, 96, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 1, 0, {0, 0, 0, 0}}, {2, 3, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 3, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_HOG_SVM[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {3, 96, 1, 0, {0, 0, 0, 0}}, {3, 3, 2, 0, {0, 0, 0, 0}}, {48, 3, 1, 0, {0, 0, 0, 0}}, {3, 3, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {3, 3, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_HOG_SVM[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {4, 96, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {64, 3, 1, 0, {0, 0, 0, 0}}, {4, 3, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {4, 3, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_HOG_SVM[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {1, 128, 1, 0, {0, 0, 0, 0}}, {1, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 1, 0, {0, 0, 0, 0}}, {1, 4, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 4, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_HOG_SVM[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {2, 128, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 1, 0, {0, 0, 0, 0}}, {2, 4, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 4, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_HOG_SVM[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {3, 128, 1, 0, {0, 0, 0, 0}}, {3, 4, 2, 0, {0, 0, 0, 0}}, {48, 4, 1, 0, {0, 0, 0, 0}}, {3, 4, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {3, 4, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_HOG_SVM[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {1, 160, 1, 0, {0, 0, 0, 0}}, {1, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 1, 0, {0, 0, 0, 0}}, {1, 5, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 5, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_HOG_SVM[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {2, 160, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 1, 0, {0, 0, 0, 0}}, {2, 5, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 5, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_HOG_SVM[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {1, 192, 1, 0, {0, 0, 0, 0}}, {1, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 1, 0, {0, 0, 0, 0}}, {1, 6, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 6, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_HOG_SVM[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {2, 192, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 1, 0, {0, 0, 0, 0}}, {2, 6, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 6, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_HOG_SVM[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {1, 256, 1, 0, {0, 0, 0, 0}}, {1, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 1, 0, {0, 0, 0, 0}}, {1, 8, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 8, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_HOG_SVM[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {1, 320, 1, 0, {0, 0, 0, 0}}, {1, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 1, 0, {0, 0, 0, 0}}, {1, 10, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 10, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_HOG_SVM[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {1, 384, 1, 0, {0, 0, 0, 0}}, {1, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 1, 0, {0, 0, 0, 0}}, {1, 12, 1, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 12, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2050, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {2, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_HOG_SVM[] = {{0, 0}, {0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_HOG_SVM[] = {{0, 0}, {0, 0}};

static acf_scenario gScenarioArray_HOG_SVM[] = {
{1, 1, 280, 2208, 0, gScenarioBufferData0_HOG_SVM, 208, gScenarioKernelData0_HOG_SVM, 8},
{2, 1, 556, 2224, 0, gScenarioBufferData1_HOG_SVM, 208, gScenarioKernelData1_HOG_SVM, 8},
{3, 1, 836, 2240, 0, gScenarioBufferData2_HOG_SVM, 208, gScenarioKernelData2_HOG_SVM, 8},
{4, 1, 1112, 2256, 0, gScenarioBufferData3_HOG_SVM, 208, gScenarioKernelData3_HOG_SVM, 8},
{5, 1, 1392, 2272, 0, gScenarioBufferData4_HOG_SVM, 208, gScenarioKernelData4_HOG_SVM, 8},
{6, 1, 1668, 2288, 0, gScenarioBufferData5_HOG_SVM, 208, gScenarioKernelData5_HOG_SVM, 8},
{7, 1, 1948, 2304, 0, gScenarioBufferData6_HOG_SVM, 208, gScenarioKernelData6_HOG_SVM, 8},
{8, 1, 2224, 2320, 0, gScenarioBufferData7_HOG_SVM, 208, gScenarioKernelData7_HOG_SVM, 8},
{9, 1, 2504, 2336, 0, gScenarioBufferData8_HOG_SVM, 208, gScenarioKernelData8_HOG_SVM, 8},
{10, 1, 2780, 2352, 0, gScenarioBufferData9_HOG_SVM, 208, gScenarioKernelData9_HOG_SVM, 8},
{11, 1, 3060, 2368, 0, gScenarioBufferData10_HOG_SVM, 208, gScenarioKernelData10_HOG_SVM, 8},
{12, 1, 3336, 2384, 0, gScenarioBufferData11_HOG_SVM, 208, gScenarioKernelData11_HOG_SVM, 8},
{13, 1, 3616, 2400, 0, gScenarioBufferData12_HOG_SVM, 208, gScenarioKernelData12_HOG_SVM, 8},
{1, 2, 556, 2240, 0, gScenarioBufferData13_HOG_SVM, 208, gScenarioKernelData13_HOG_SVM, 8},
{2, 2, 1112, 2272, 0, gScenarioBufferData14_HOG_SVM, 208, gScenarioKernelData14_HOG_SVM, 8},
{3, 2, 1668, 2304, 0, gScenarioBufferData15_HOG_SVM, 208, gScenarioKernelData15_HOG_SVM, 8},
{4, 2, 2224, 2336, 0, gScenarioBufferData16_HOG_SVM, 208, gScenarioKernelData16_HOG_SVM, 8},
{5, 2, 2780, 2368, 0, gScenarioBufferData17_HOG_SVM, 208, gScenarioKernelData17_HOG_SVM, 8},
{6, 2, 3336, 2400, 0, gScenarioBufferData18_HOG_SVM, 208, gScenarioKernelData18_HOG_SVM, 8},
{1, 3, 836, 2272, 0, gScenarioBufferData19_HOG_SVM, 208, gScenarioKernelData19_HOG_SVM, 8},
{2, 3, 1668, 2320, 0, gScenarioBufferData20_HOG_SVM, 208, gScenarioKernelData20_HOG_SVM, 8},
{3, 3, 2504, 2368, 0, gScenarioBufferData21_HOG_SVM, 208, gScenarioKernelData21_HOG_SVM, 8},
{4, 3, 3336, 2416, 0, gScenarioBufferData22_HOG_SVM, 208, gScenarioKernelData22_HOG_SVM, 8},
{1, 4, 1112, 2304, 0, gScenarioBufferData23_HOG_SVM, 208, gScenarioKernelData23_HOG_SVM, 8},
{2, 4, 2224, 2368, 0, gScenarioBufferData24_HOG_SVM, 208, gScenarioKernelData24_HOG_SVM, 8},
{3, 4, 3336, 2432, 0, gScenarioBufferData25_HOG_SVM, 208, gScenarioKernelData25_HOG_SVM, 8},
{1, 5, 1392, 2336, 0, gScenarioBufferData26_HOG_SVM, 208, gScenarioKernelData26_HOG_SVM, 8},
{2, 5, 2780, 2416, 0, gScenarioBufferData27_HOG_SVM, 208, gScenarioKernelData27_HOG_SVM, 8},
{1, 6, 1668, 2368, 0, gScenarioBufferData28_HOG_SVM, 208, gScenarioKernelData28_HOG_SVM, 8},
{2, 6, 3336, 2464, 0, gScenarioBufferData29_HOG_SVM, 208, gScenarioKernelData29_HOG_SVM, 8},
{1, 8, 2224, 2432, 0, gScenarioBufferData30_HOG_SVM, 208, gScenarioKernelData30_HOG_SVM, 8},
{1, 10, 2780, 2496, 0, gScenarioBufferData31_HOG_SVM, 208, gScenarioKernelData31_HOG_SVM, 8},
{1, 12, 3336, 2560, 0, gScenarioBufferData32_HOG_SVM, 208, gScenarioKernelData32_HOG_SVM, 8}
};

static acf_scenario_list gScenarioList_HOG_SVM = {
33, //number of scenarios
gScenarioArray_HOG_SVM};
//**************************************************************

class HOG_SVM : public ACF_Process_APU
{

public:
   HOG_SVM(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("HOG_SVM");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("HOG_SVM",
                                        HOG_SVM_LOAD_SEGMENTS,
                                        HOG_SVM_LOAD_PMEM, HOG_SVM_LOAD_PMEM_SIZE,
                                        HOG_SVM_LOAD_DMEM, HOG_SVM_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(HOG_SVM_APEX_LOG_BUFFER, HOG_SVM_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("IMAGE_BLOCK_HISTOGRAMS", icp::DATATYPE_08U, 1, 1, 8, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("SVM_FUNCTION", icp::DATATYPE_08S, 1, 1, 2050, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("SVM_TRANSFORM_MODE", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 11, 0, 0, 0, 0, 0);
         AddPort("WIN_BLOCKS", icp::DATATYPE_08U, 1, 1, 2, 1, 0, 1, 1, 0, 1, 0, 12, 0, 0, 0, 0, 0);
         AddPort("SCORES", icp::DATATYPE_16S, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_HOG_SVM);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_HOG_SVM

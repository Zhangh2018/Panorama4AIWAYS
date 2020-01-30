#ifndef _ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_OPENEXR
#define _ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_OPENEXR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <TMO_CALC_GEO_MEAN_OPENEXR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {2, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {4, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {6, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {10, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {12, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_TMO_CALC_GEO_MEAN_OPENEXR[] = {{20, 1, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {20, 1, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {2, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {4, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {6, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {10, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {12, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_TMO_CALC_GEO_MEAN_OPENEXR[] = {{20, 2, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {20, 2, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {2, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {4, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {6, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {10, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {12, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_TMO_CALC_GEO_MEAN_OPENEXR[] = {{20, 3, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {20, 3, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {2, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {4, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {6, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {10, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {12, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_TMO_CALC_GEO_MEAN_OPENEXR[] = {{20, 4, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {20, 4, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {2, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {4, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {6, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {10, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {12, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_TMO_CALC_GEO_MEAN_OPENEXR[] = {{20, 5, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {20, 5, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {2, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {4, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {6, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {10, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData45_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {12, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData46_TMO_CALC_GEO_MEAN_OPENEXR[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData47_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {2, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData48_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {4, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData49_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {6, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData50_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData51_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {10, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData52_TMO_CALC_GEO_MEAN_OPENEXR[] = {{12, 8, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {12, 8, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData53_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {2, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData54_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {4, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData55_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {6, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData56_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData57_TMO_CALC_GEO_MEAN_OPENEXR[] = {{10, 10, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {10, 10, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData58_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {2, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData59_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {4, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData60_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {6, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData61_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData62_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {2, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData63_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {4, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData64_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {6, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData65_TMO_CALC_GEO_MEAN_OPENEXR[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData66_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {2, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData67_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {4, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData68_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 16, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {6, 16, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData69_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 18, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {2, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData70_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 18, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {4, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData71_TMO_CALC_GEO_MEAN_OPENEXR[] = {{6, 18, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {6, 18, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData72_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 20, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {2, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData73_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 20, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {4, 20, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData74_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 22, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {2, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData75_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 22, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {4, 22, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData76_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 24, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {2, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData77_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 24, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {4, 24, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData78_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 26, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {2, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData79_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 26, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {4, 26, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData80_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 28, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {2, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData81_TMO_CALC_GEO_MEAN_OPENEXR[] = {{4, 28, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {4, 28, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData82_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 30, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {2, 30, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData83_TMO_CALC_GEO_MEAN_OPENEXR[] = {{2, 32, 2, 0, {0, 0, 0, 0}}, {64, 4, 1, 0, {0, 0, 0, 0}}, {128, 2, 1, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {2, 32, 2, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData45_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData46_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData47_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData48_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData49_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData50_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData51_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData52_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData53_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData54_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData55_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData56_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData57_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData58_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData59_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData60_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData61_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData62_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData63_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData64_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData65_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData66_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData67_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData68_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData69_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData70_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData71_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData72_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData73_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData74_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData75_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData76_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData77_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData78_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData79_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData80_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData81_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData82_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData83_TMO_CALC_GEO_MEAN_OPENEXR[] = {{0, 0}};

static acf_scenario gScenarioArray_TMO_CALC_GEO_MEAN_OPENEXR[] = {
{2, 1, 568, 288, 0, gScenarioBufferData0_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData0_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 1, 624, 288, 0, gScenarioBufferData1_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData1_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 1, 680, 288, 0, gScenarioBufferData2_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData2_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 1, 736, 288, 0, gScenarioBufferData3_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData3_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 1, 792, 288, 0, gScenarioBufferData4_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData4_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 1, 848, 288, 0, gScenarioBufferData5_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData5_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 1, 960, 288, 0, gScenarioBufferData6_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData6_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{20, 1, 1072, 288, 0, gScenarioBufferData7_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData7_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 2, 624, 288, 0, gScenarioBufferData8_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData8_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 2, 736, 288, 0, gScenarioBufferData9_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData9_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 2, 848, 288, 0, gScenarioBufferData10_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData10_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 2, 960, 288, 0, gScenarioBufferData11_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData11_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 2, 1072, 288, 0, gScenarioBufferData12_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData12_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 2, 1184, 288, 0, gScenarioBufferData13_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData13_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 2, 1408, 288, 0, gScenarioBufferData14_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData14_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{20, 2, 1632, 288, 0, gScenarioBufferData15_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData15_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 3, 680, 288, 0, gScenarioBufferData16_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData16_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 3, 848, 288, 0, gScenarioBufferData17_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData17_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 3, 1016, 288, 0, gScenarioBufferData18_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData18_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 3, 1184, 288, 0, gScenarioBufferData19_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData19_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 3, 1352, 288, 0, gScenarioBufferData20_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData20_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 3, 1520, 288, 0, gScenarioBufferData21_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData21_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 3, 1856, 288, 0, gScenarioBufferData22_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData22_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{20, 3, 2192, 288, 0, gScenarioBufferData23_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData23_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 4, 736, 288, 0, gScenarioBufferData24_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData24_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 4, 960, 288, 0, gScenarioBufferData25_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData25_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 4, 1184, 288, 0, gScenarioBufferData26_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData26_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 4, 1408, 288, 0, gScenarioBufferData27_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData27_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 4, 1632, 288, 0, gScenarioBufferData28_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData28_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 4, 1856, 288, 0, gScenarioBufferData29_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData29_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 4, 2304, 288, 0, gScenarioBufferData30_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData30_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{20, 4, 2752, 288, 0, gScenarioBufferData31_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData31_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 5, 792, 288, 0, gScenarioBufferData32_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData32_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 5, 1072, 288, 0, gScenarioBufferData33_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData33_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 5, 1352, 288, 0, gScenarioBufferData34_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData34_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 5, 1632, 288, 0, gScenarioBufferData35_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData35_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 5, 1912, 288, 0, gScenarioBufferData36_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData36_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 5, 2192, 288, 0, gScenarioBufferData37_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData37_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 5, 2752, 288, 0, gScenarioBufferData38_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData38_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{20, 5, 3312, 288, 0, gScenarioBufferData39_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData39_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 6, 848, 288, 0, gScenarioBufferData40_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData40_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 6, 1184, 288, 0, gScenarioBufferData41_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData41_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 6, 1520, 288, 0, gScenarioBufferData42_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData42_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 6, 1856, 288, 0, gScenarioBufferData43_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData43_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 6, 2192, 288, 0, gScenarioBufferData44_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData44_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 6, 2528, 288, 0, gScenarioBufferData45_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData45_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{16, 6, 3200, 288, 0, gScenarioBufferData46_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData46_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 8, 960, 288, 0, gScenarioBufferData47_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData47_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 8, 1408, 288, 0, gScenarioBufferData48_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData48_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 8, 1856, 288, 0, gScenarioBufferData49_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData49_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 8, 2304, 288, 0, gScenarioBufferData50_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData50_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 8, 2752, 288, 0, gScenarioBufferData51_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData51_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{12, 8, 3200, 288, 0, gScenarioBufferData52_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData52_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 10, 1072, 288, 0, gScenarioBufferData53_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData53_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 10, 1632, 288, 0, gScenarioBufferData54_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData54_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 10, 2192, 288, 0, gScenarioBufferData55_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData55_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 10, 2752, 288, 0, gScenarioBufferData56_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData56_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{10, 10, 3312, 288, 0, gScenarioBufferData57_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData57_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 12, 1184, 288, 0, gScenarioBufferData58_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData58_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 12, 1856, 288, 0, gScenarioBufferData59_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData59_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 12, 2528, 288, 0, gScenarioBufferData60_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData60_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 12, 3200, 288, 0, gScenarioBufferData61_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData61_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 14, 1296, 288, 0, gScenarioBufferData62_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData62_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 14, 2080, 288, 0, gScenarioBufferData63_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData63_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 14, 2864, 288, 0, gScenarioBufferData64_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData64_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{8, 14, 3648, 288, 0, gScenarioBufferData65_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData65_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 16, 1408, 288, 0, gScenarioBufferData66_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData66_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 16, 2304, 288, 0, gScenarioBufferData67_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData67_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 16, 3200, 288, 0, gScenarioBufferData68_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData68_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 18, 1520, 288, 0, gScenarioBufferData69_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData69_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 18, 2528, 288, 0, gScenarioBufferData70_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData70_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{6, 18, 3536, 288, 0, gScenarioBufferData71_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData71_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 20, 1632, 288, 0, gScenarioBufferData72_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData72_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 20, 2752, 288, 0, gScenarioBufferData73_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData73_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 22, 1744, 288, 0, gScenarioBufferData74_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData74_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 22, 2976, 288, 0, gScenarioBufferData75_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData75_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 24, 1856, 288, 0, gScenarioBufferData76_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData76_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 24, 3200, 288, 0, gScenarioBufferData77_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData77_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 26, 1968, 288, 0, gScenarioBufferData78_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData78_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 26, 3424, 288, 0, gScenarioBufferData79_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData79_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 28, 2080, 288, 0, gScenarioBufferData80_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData80_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{4, 28, 3648, 288, 0, gScenarioBufferData81_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData81_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 30, 2192, 288, 0, gScenarioBufferData82_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData82_TMO_CALC_GEO_MEAN_OPENEXR, 4},
{2, 32, 2304, 288, 0, gScenarioBufferData83_TMO_CALC_GEO_MEAN_OPENEXR, 144, gScenarioKernelData83_TMO_CALC_GEO_MEAN_OPENEXR, 4}
};

static acf_scenario_list gScenarioList_TMO_CALC_GEO_MEAN_OPENEXR = {
84, //number of scenarios
gScenarioArray_TMO_CALC_GEO_MEAN_OPENEXR};
//**************************************************************

class TMO_CALC_GEO_MEAN_OPENEXR : public ACF_Process_APU
{

public:
   TMO_CALC_GEO_MEAN_OPENEXR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("TMO_CALC_GEO_MEAN_OPENEXR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("TMO_CALC_GEO_MEAN_OPENEXR",
                                        TMO_CALC_GEO_MEAN_OPENEXR_LOAD_SEGMENTS,
                                        TMO_CALC_GEO_MEAN_OPENEXR_LOAD_PMEM, TMO_CALC_GEO_MEAN_OPENEXR_LOAD_PMEM_SIZE,
                                        TMO_CALC_GEO_MEAN_OPENEXR_LOAD_DMEM, TMO_CALC_GEO_MEAN_OPENEXR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(TMO_CALC_GEO_MEAN_OPENEXR_APEX_LOG_BUFFER, TMO_CALC_GEO_MEAN_OPENEXR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT", icp::DATATYPE_16U, 3, 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("INTERMEDIATE_EXP", icp::DATATYPE_08U, 3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0);
         AddPort("INTERMEDIATE_MAN", icp::DATATYPE_08U, 3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0);
         AddPort("WORLD_EXP", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0);
         AddPort("WORLD_MAN", icp::DATATYPE_08U, 1, 1, 2, 1, 1, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0);
         AddPort("GEO_MEAN_EXP", icp::DATATYPE_08U, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 7, 0, 0, 0, 0, 0);
         AddPort("GEO_MEAN_MAN", icp::DATATYPE_08U, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 8, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_TMO_CALC_GEO_MEAN_OPENEXR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_OPENEXR

#ifndef _ACF_PROCESS_APU_INTEGRAL_IMAGE
#define _ACF_PROCESS_APU_INTEGRAL_IMAGE

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <INTEGRAL_IMAGE_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_INTEGRAL_IMAGE[] = {{8, 1, 2, 0, {0, 0, 0, 0}}, {8, 1, 1, 0, {0, 0, 0, 0}}, {8, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData1_INTEGRAL_IMAGE[] = {{16, 1, 2, 0, {0, 0, 0, 0}}, {16, 1, 1, 0, {0, 0, 0, 0}}, {16, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData2_INTEGRAL_IMAGE[] = {{24, 1, 2, 0, {0, 0, 0, 0}}, {24, 1, 1, 0, {0, 0, 0, 0}}, {24, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData3_INTEGRAL_IMAGE[] = {{32, 1, 2, 0, {0, 0, 0, 0}}, {32, 1, 1, 0, {0, 0, 0, 0}}, {32, 1, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData4_INTEGRAL_IMAGE[] = {{8, 2, 2, 0, {0, 0, 0, 0}}, {8, 2, 1, 0, {0, 0, 0, 0}}, {8, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData5_INTEGRAL_IMAGE[] = {{16, 2, 2, 0, {0, 0, 0, 0}}, {16, 2, 1, 0, {0, 0, 0, 0}}, {16, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData6_INTEGRAL_IMAGE[] = {{24, 2, 2, 0, {0, 0, 0, 0}}, {24, 2, 1, 0, {0, 0, 0, 0}}, {24, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData7_INTEGRAL_IMAGE[] = {{32, 2, 2, 0, {0, 0, 0, 0}}, {32, 2, 1, 0, {0, 0, 0, 0}}, {32, 2, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData8_INTEGRAL_IMAGE[] = {{8, 3, 2, 0, {0, 0, 0, 0}}, {8, 3, 1, 0, {0, 0, 0, 0}}, {8, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData9_INTEGRAL_IMAGE[] = {{16, 3, 2, 0, {0, 0, 0, 0}}, {16, 3, 1, 0, {0, 0, 0, 0}}, {16, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData10_INTEGRAL_IMAGE[] = {{24, 3, 2, 0, {0, 0, 0, 0}}, {24, 3, 1, 0, {0, 0, 0, 0}}, {24, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData11_INTEGRAL_IMAGE[] = {{32, 3, 2, 0, {0, 0, 0, 0}}, {32, 3, 1, 0, {0, 0, 0, 0}}, {32, 3, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData12_INTEGRAL_IMAGE[] = {{8, 4, 2, 0, {0, 0, 0, 0}}, {8, 4, 1, 0, {0, 0, 0, 0}}, {8, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData13_INTEGRAL_IMAGE[] = {{16, 4, 2, 0, {0, 0, 0, 0}}, {16, 4, 1, 0, {0, 0, 0, 0}}, {16, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData14_INTEGRAL_IMAGE[] = {{24, 4, 2, 0, {0, 0, 0, 0}}, {24, 4, 1, 0, {0, 0, 0, 0}}, {24, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData15_INTEGRAL_IMAGE[] = {{32, 4, 2, 0, {0, 0, 0, 0}}, {32, 4, 1, 0, {0, 0, 0, 0}}, {32, 4, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData16_INTEGRAL_IMAGE[] = {{8, 5, 2, 0, {0, 0, 0, 0}}, {8, 5, 1, 0, {0, 0, 0, 0}}, {8, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData17_INTEGRAL_IMAGE[] = {{16, 5, 2, 0, {0, 0, 0, 0}}, {16, 5, 1, 0, {0, 0, 0, 0}}, {16, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData18_INTEGRAL_IMAGE[] = {{24, 5, 2, 0, {0, 0, 0, 0}}, {24, 5, 1, 0, {0, 0, 0, 0}}, {24, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData19_INTEGRAL_IMAGE[] = {{32, 5, 2, 0, {0, 0, 0, 0}}, {32, 5, 1, 0, {0, 0, 0, 0}}, {32, 5, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData20_INTEGRAL_IMAGE[] = {{8, 6, 2, 0, {0, 0, 0, 0}}, {8, 6, 1, 0, {0, 0, 0, 0}}, {8, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData21_INTEGRAL_IMAGE[] = {{16, 6, 2, 0, {0, 0, 0, 0}}, {16, 6, 1, 0, {0, 0, 0, 0}}, {16, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData22_INTEGRAL_IMAGE[] = {{24, 6, 2, 0, {0, 0, 0, 0}}, {24, 6, 1, 0, {0, 0, 0, 0}}, {24, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData23_INTEGRAL_IMAGE[] = {{32, 6, 2, 0, {0, 0, 0, 0}}, {32, 6, 1, 0, {0, 0, 0, 0}}, {32, 6, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData24_INTEGRAL_IMAGE[] = {{8, 8, 2, 0, {0, 0, 0, 0}}, {8, 8, 1, 0, {0, 0, 0, 0}}, {8, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData25_INTEGRAL_IMAGE[] = {{16, 8, 2, 0, {0, 0, 0, 0}}, {16, 8, 1, 0, {0, 0, 0, 0}}, {16, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData26_INTEGRAL_IMAGE[] = {{24, 8, 2, 0, {0, 0, 0, 0}}, {24, 8, 1, 0, {0, 0, 0, 0}}, {24, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData27_INTEGRAL_IMAGE[] = {{32, 8, 2, 0, {0, 0, 0, 0}}, {32, 8, 1, 0, {0, 0, 0, 0}}, {32, 8, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData28_INTEGRAL_IMAGE[] = {{8, 10, 2, 0, {0, 0, 0, 0}}, {8, 10, 1, 0, {0, 0, 0, 0}}, {8, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData29_INTEGRAL_IMAGE[] = {{16, 10, 2, 0, {0, 0, 0, 0}}, {16, 10, 1, 0, {0, 0, 0, 0}}, {16, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData30_INTEGRAL_IMAGE[] = {{24, 10, 2, 0, {0, 0, 0, 0}}, {24, 10, 1, 0, {0, 0, 0, 0}}, {24, 10, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData31_INTEGRAL_IMAGE[] = {{8, 12, 2, 0, {0, 0, 0, 0}}, {8, 12, 1, 0, {0, 0, 0, 0}}, {8, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData32_INTEGRAL_IMAGE[] = {{16, 12, 2, 0, {0, 0, 0, 0}}, {16, 12, 1, 0, {0, 0, 0, 0}}, {16, 12, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData33_INTEGRAL_IMAGE[] = {{8, 14, 2, 0, {0, 0, 0, 0}}, {8, 14, 1, 0, {0, 0, 0, 0}}, {8, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData34_INTEGRAL_IMAGE[] = {{16, 14, 2, 0, {0, 0, 0, 0}}, {16, 14, 1, 0, {0, 0, 0, 0}}, {16, 14, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData35_INTEGRAL_IMAGE[] = {{8, 16, 2, 0, {0, 0, 0, 0}}, {8, 16, 1, 0, {0, 0, 0, 0}}, {8, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData36_INTEGRAL_IMAGE[] = {{16, 16, 2, 0, {0, 0, 0, 0}}, {16, 16, 1, 0, {0, 0, 0, 0}}, {16, 16, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData37_INTEGRAL_IMAGE[] = {{8, 18, 2, 0, {0, 0, 0, 0}}, {8, 18, 1, 0, {0, 0, 0, 0}}, {8, 18, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData38_INTEGRAL_IMAGE[] = {{8, 20, 2, 0, {0, 0, 0, 0}}, {8, 20, 1, 0, {0, 0, 0, 0}}, {8, 20, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData39_INTEGRAL_IMAGE[] = {{8, 22, 2, 0, {0, 0, 0, 0}}, {8, 22, 1, 0, {0, 0, 0, 0}}, {8, 22, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData40_INTEGRAL_IMAGE[] = {{8, 24, 2, 0, {0, 0, 0, 0}}, {8, 24, 1, 0, {0, 0, 0, 0}}, {8, 24, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData41_INTEGRAL_IMAGE[] = {{8, 26, 2, 0, {0, 0, 0, 0}}, {8, 26, 1, 0, {0, 0, 0, 0}}, {8, 26, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData42_INTEGRAL_IMAGE[] = {{8, 28, 2, 0, {0, 0, 0, 0}}, {8, 28, 1, 0, {0, 0, 0, 0}}, {8, 28, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData43_INTEGRAL_IMAGE[] = {{8, 30, 2, 0, {0, 0, 0, 0}}, {8, 30, 1, 0, {0, 0, 0, 0}}, {8, 30, 2, 0, {0, 0, 0, 0}}};
static acf_scenario_buffer_data gScenarioBufferData44_INTEGRAL_IMAGE[] = {{8, 32, 2, 0, {0, 0, 0, 0}}, {8, 32, 1, 0, {0, 0, 0, 0}}, {8, 32, 2, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData1_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData2_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData3_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData4_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData5_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData6_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData7_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData8_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData9_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData10_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData11_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData12_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData13_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData14_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData15_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData16_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData17_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData18_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData19_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData20_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData21_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData22_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData23_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData24_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData25_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData26_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData27_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData28_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData29_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData30_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData31_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData32_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData33_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData34_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData35_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData36_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData37_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData38_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData39_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData40_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData41_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData42_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData43_INTEGRAL_IMAGE[] = {{0, 0}};
static acf_scenario_kernel_data gScenarioKernelData44_INTEGRAL_IMAGE[] = {{0, 0}};

static acf_scenario gScenarioArray_INTEGRAL_IMAGE[] = {
{1, 1, 112, 0, 0, gScenarioBufferData0_INTEGRAL_IMAGE, 48, gScenarioKernelData0_INTEGRAL_IMAGE, 4},
{2, 1, 224, 0, 0, gScenarioBufferData1_INTEGRAL_IMAGE, 48, gScenarioKernelData1_INTEGRAL_IMAGE, 4},
{3, 1, 336, 0, 0, gScenarioBufferData2_INTEGRAL_IMAGE, 48, gScenarioKernelData2_INTEGRAL_IMAGE, 4},
{4, 1, 448, 0, 0, gScenarioBufferData3_INTEGRAL_IMAGE, 48, gScenarioKernelData3_INTEGRAL_IMAGE, 4},
{1, 2, 224, 0, 0, gScenarioBufferData4_INTEGRAL_IMAGE, 48, gScenarioKernelData4_INTEGRAL_IMAGE, 4},
{2, 2, 448, 0, 0, gScenarioBufferData5_INTEGRAL_IMAGE, 48, gScenarioKernelData5_INTEGRAL_IMAGE, 4},
{3, 2, 672, 0, 0, gScenarioBufferData6_INTEGRAL_IMAGE, 48, gScenarioKernelData6_INTEGRAL_IMAGE, 4},
{4, 2, 896, 0, 0, gScenarioBufferData7_INTEGRAL_IMAGE, 48, gScenarioKernelData7_INTEGRAL_IMAGE, 4},
{1, 3, 336, 0, 0, gScenarioBufferData8_INTEGRAL_IMAGE, 48, gScenarioKernelData8_INTEGRAL_IMAGE, 4},
{2, 3, 672, 0, 0, gScenarioBufferData9_INTEGRAL_IMAGE, 48, gScenarioKernelData9_INTEGRAL_IMAGE, 4},
{3, 3, 1008, 0, 0, gScenarioBufferData10_INTEGRAL_IMAGE, 48, gScenarioKernelData10_INTEGRAL_IMAGE, 4},
{4, 3, 1344, 0, 0, gScenarioBufferData11_INTEGRAL_IMAGE, 48, gScenarioKernelData11_INTEGRAL_IMAGE, 4},
{1, 4, 448, 0, 0, gScenarioBufferData12_INTEGRAL_IMAGE, 48, gScenarioKernelData12_INTEGRAL_IMAGE, 4},
{2, 4, 896, 0, 0, gScenarioBufferData13_INTEGRAL_IMAGE, 48, gScenarioKernelData13_INTEGRAL_IMAGE, 4},
{3, 4, 1344, 0, 0, gScenarioBufferData14_INTEGRAL_IMAGE, 48, gScenarioKernelData14_INTEGRAL_IMAGE, 4},
{4, 4, 1792, 0, 0, gScenarioBufferData15_INTEGRAL_IMAGE, 48, gScenarioKernelData15_INTEGRAL_IMAGE, 4},
{1, 5, 560, 0, 0, gScenarioBufferData16_INTEGRAL_IMAGE, 48, gScenarioKernelData16_INTEGRAL_IMAGE, 4},
{2, 5, 1120, 0, 0, gScenarioBufferData17_INTEGRAL_IMAGE, 48, gScenarioKernelData17_INTEGRAL_IMAGE, 4},
{3, 5, 1680, 0, 0, gScenarioBufferData18_INTEGRAL_IMAGE, 48, gScenarioKernelData18_INTEGRAL_IMAGE, 4},
{4, 5, 2240, 0, 0, gScenarioBufferData19_INTEGRAL_IMAGE, 48, gScenarioKernelData19_INTEGRAL_IMAGE, 4},
{1, 6, 672, 0, 0, gScenarioBufferData20_INTEGRAL_IMAGE, 48, gScenarioKernelData20_INTEGRAL_IMAGE, 4},
{2, 6, 1344, 0, 0, gScenarioBufferData21_INTEGRAL_IMAGE, 48, gScenarioKernelData21_INTEGRAL_IMAGE, 4},
{3, 6, 2016, 0, 0, gScenarioBufferData22_INTEGRAL_IMAGE, 48, gScenarioKernelData22_INTEGRAL_IMAGE, 4},
{4, 6, 2688, 0, 0, gScenarioBufferData23_INTEGRAL_IMAGE, 48, gScenarioKernelData23_INTEGRAL_IMAGE, 4},
{1, 8, 896, 0, 0, gScenarioBufferData24_INTEGRAL_IMAGE, 48, gScenarioKernelData24_INTEGRAL_IMAGE, 4},
{2, 8, 1792, 0, 0, gScenarioBufferData25_INTEGRAL_IMAGE, 48, gScenarioKernelData25_INTEGRAL_IMAGE, 4},
{3, 8, 2688, 0, 0, gScenarioBufferData26_INTEGRAL_IMAGE, 48, gScenarioKernelData26_INTEGRAL_IMAGE, 4},
{4, 8, 3584, 0, 0, gScenarioBufferData27_INTEGRAL_IMAGE, 48, gScenarioKernelData27_INTEGRAL_IMAGE, 4},
{1, 10, 1120, 0, 0, gScenarioBufferData28_INTEGRAL_IMAGE, 48, gScenarioKernelData28_INTEGRAL_IMAGE, 4},
{2, 10, 2240, 0, 0, gScenarioBufferData29_INTEGRAL_IMAGE, 48, gScenarioKernelData29_INTEGRAL_IMAGE, 4},
{3, 10, 3360, 0, 0, gScenarioBufferData30_INTEGRAL_IMAGE, 48, gScenarioKernelData30_INTEGRAL_IMAGE, 4},
{1, 12, 1344, 0, 0, gScenarioBufferData31_INTEGRAL_IMAGE, 48, gScenarioKernelData31_INTEGRAL_IMAGE, 4},
{2, 12, 2688, 0, 0, gScenarioBufferData32_INTEGRAL_IMAGE, 48, gScenarioKernelData32_INTEGRAL_IMAGE, 4},
{1, 14, 1568, 0, 0, gScenarioBufferData33_INTEGRAL_IMAGE, 48, gScenarioKernelData33_INTEGRAL_IMAGE, 4},
{2, 14, 3136, 0, 0, gScenarioBufferData34_INTEGRAL_IMAGE, 48, gScenarioKernelData34_INTEGRAL_IMAGE, 4},
{1, 16, 1792, 0, 0, gScenarioBufferData35_INTEGRAL_IMAGE, 48, gScenarioKernelData35_INTEGRAL_IMAGE, 4},
{2, 16, 3584, 0, 0, gScenarioBufferData36_INTEGRAL_IMAGE, 48, gScenarioKernelData36_INTEGRAL_IMAGE, 4},
{1, 18, 2016, 0, 0, gScenarioBufferData37_INTEGRAL_IMAGE, 48, gScenarioKernelData37_INTEGRAL_IMAGE, 4},
{1, 20, 2240, 0, 0, gScenarioBufferData38_INTEGRAL_IMAGE, 48, gScenarioKernelData38_INTEGRAL_IMAGE, 4},
{1, 22, 2464, 0, 0, gScenarioBufferData39_INTEGRAL_IMAGE, 48, gScenarioKernelData39_INTEGRAL_IMAGE, 4},
{1, 24, 2688, 0, 0, gScenarioBufferData40_INTEGRAL_IMAGE, 48, gScenarioKernelData40_INTEGRAL_IMAGE, 4},
{1, 26, 2912, 0, 0, gScenarioBufferData41_INTEGRAL_IMAGE, 48, gScenarioKernelData41_INTEGRAL_IMAGE, 4},
{1, 28, 3136, 0, 0, gScenarioBufferData42_INTEGRAL_IMAGE, 48, gScenarioKernelData42_INTEGRAL_IMAGE, 4},
{1, 30, 3360, 0, 0, gScenarioBufferData43_INTEGRAL_IMAGE, 48, gScenarioKernelData43_INTEGRAL_IMAGE, 4},
{1, 32, 3584, 0, 0, gScenarioBufferData44_INTEGRAL_IMAGE, 48, gScenarioKernelData44_INTEGRAL_IMAGE, 4}
};

static acf_scenario_list gScenarioList_INTEGRAL_IMAGE = {
45, //number of scenarios
gScenarioArray_INTEGRAL_IMAGE};
//**************************************************************

class INTEGRAL_IMAGE : public ACF_Process_APU
{

public:
   INTEGRAL_IMAGE(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("INTEGRAL_IMAGE");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("INTEGRAL_IMAGE",
                                        INTEGRAL_IMAGE_LOAD_SEGMENTS,
                                        INTEGRAL_IMAGE_LOAD_PMEM, INTEGRAL_IMAGE_LOAD_PMEM_SIZE,
                                        INTEGRAL_IMAGE_LOAD_DMEM, INTEGRAL_IMAGE_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(INTEGRAL_IMAGE_APEX_LOG_BUFFER, INTEGRAL_IMAGE_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("INPUT_0", icp::DATATYPE_08U, 1, 1, 8, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
         AddPort("OUTPUT_0", icp::DATATYPE_32U, 1, 1, 8, 1, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_INTEGRAL_IMAGE);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_INTEGRAL_IMAGE

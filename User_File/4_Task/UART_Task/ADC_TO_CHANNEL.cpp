//
// Created by chengfeng on 2026/5/23.
//

#include "ADC_TO_CHANNEL.h"
#include "main.h"

#define LX_MIN 6
#define LX_MAX 4059
#define RX_MIN 69
#define RX_MAX 4089
#define RY_MIN 26
#define RY_MAX 3738

#define LX_MID 1875
#define RX_MID 2153
#define RY_MID 1996

struct G_CHANNEL_DATA g_channel_data;

// 辅助函数：基于中位值进行分段线性映射，并限幅到 [-100.0, 100.0]
static float map_to_normalized_range(uint16_t val, uint16_t mid, uint16_t min_val, uint16_t max_val) {
  float mapped_val = 0.0f;

  // 全部转为 float 再计算，避免 uint16_t 相减产生下溢出（补码变成极大正数）
  float f_val = (float)val;
  float f_mid = (float)mid;
  float f_min = (float)min_val;
  float f_max = (float)max_val;

  if (f_val >= f_mid) {
    // 正半轴映射：[mid, max] 映射到 [0.0, 100.0]
    mapped_val = ((f_val - f_mid) / (f_max - f_mid)) * 100.0f;
  } else {
    // 负半轴映射：[min, mid] 映射到 [-100.0, 0.0]
    mapped_val = ((f_val - f_mid) / (f_mid - f_min)) * 100.0f;
  }

  // 限幅处理，防止 ADC 越界（如用力推杆超过了设定的 MAX/MIN 值）
  if (mapped_val > 100.0f) {
    return 100.0f;
  } else if (mapped_val < -100.0f) {
    return -100.0f;
  }

  return mapped_val;
}

void ADC_to_Channel(uint16_t adc1, uint16_t adc2, uint16_t adc3) {
  // 直接传入无符号 ADC 原始值与对应的 MID, MIN, MAX 进行计算
  g_channel_data.Lx = map_to_normalized_range(adc1, LX_MID, LX_MIN, LX_MAX);
  g_channel_data.Rx = map_to_normalized_range(adc2, RX_MID, RX_MIN, RX_MAX);
  g_channel_data.Ry = map_to_normalized_range(adc3, RY_MID, RY_MIN, RY_MAX);
}
//
// Created by chengfeng on 2026/5/23.
//

#ifndef DM02_TEST_ADC_TO_CHANNEL_H
#define DM02_TEST_ADC_TO_CHANNEL_H

#include "main.h"

struct G_CHANNEL_DATA{
  float Lx;
  float Rx;
  float Ry;
};



extern struct G_CHANNEL_DATA g_channel_data;

void ADC_to_Channel(uint16_t adc1, uint16_t adc2, uint16_t adc3);

#endif // DM02_TEST_ADC_TO_CHANNEL_H

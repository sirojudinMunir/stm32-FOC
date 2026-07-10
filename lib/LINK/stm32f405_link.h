#ifndef STM32F405_LINK_H
#define STM32F405_LINK_H

#include "stm32f4xx_hal.h"

void link_set_pwm_freq(TIM_HandleTypeDef *htim, uint32_t freq);
uint32_t link_get_max_pwm(TIM_HandleTypeDef *htim);
int link_enable_pwm(TIM_HandleTypeDef *htim);
int link_disable_pwm(TIM_HandleTypeDef *htim);
int link_enable_adc(ADC_HandleTypeDef *hadc);
int link_disable_adc(ADC_HandleTypeDef *hadc);

#endif // STM32F405_LINK_H
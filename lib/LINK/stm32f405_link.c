#include "stm32f405_link.h"

// low level implementation on stm32f405rgt6

static uint32_t get_timer_clock(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1 || htim->Instance == TIM8 || 
      htim->Instance == TIM9 || htim->Instance == TIM10 || 
      htim->Instance == TIM11) {
    // Timer in APB2
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
      return pclk * 2;  // Timer clock doubled if APB prescaler != 1
    else
      return pclk;
  } else {
    // Timer in APB1
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
      return pclk * 2;
    else
      return pclk;
  }
}

void link_set_pwm_freq(TIM_HandleTypeDef *htim, uint32_t freq) {
  const uint32_t timer_clock = get_timer_clock(htim);

  // prescaler and period for center-aligned mode
  uint32_t prescaler = 0;
  uint32_t period = (timer_clock / (2 * freq)) - 1;

  if (period > 0xFFFF) {
    prescaler = period / 0xFFFF;
    period = (timer_clock / (2 * freq * (prescaler + 1))) - 1;
  }

  htim->Instance->PSC = prescaler;
  htim->Instance->ARR = period;
}

uint32_t link_get_max_pwm(TIM_HandleTypeDef *htim) {
  return htim->Instance->ARR;
}

int link_enable_pwm(TIM_HandleTypeDef *htim) {
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  // channel 4 is used to trigger the ADC
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4) != HAL_OK) return -1;
  htim->Instance->CCR4 = 10;
  return 0;
}

int link_disable_pwm(TIM_HandleTypeDef *htim) {
  htim->Instance->CCR1 = 0;
  htim->Instance->CCR2 = 0;
  htim->Instance->CCR3 = 0;
  htim->Instance->CCR4 = 0;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4) != HAL_OK) return -1;
  return 0;
}

int link_enable_adc(ADC_HandleTypeDef *hadc) {
  if (HAL_ADCEx_InjectedStart_IT(hadc) != HAL_OK) return -1;
  return 0;
}

int link_disable_adc(ADC_HandleTypeDef *hadc) {
  if (HAL_ADCEx_InjectedStop_IT(hadc) != HAL_OK) return -1;
  return 0;
}

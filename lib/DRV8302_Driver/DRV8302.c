/*
 * DRV8302.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "DRV8302.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


void DRV8302_GPIO_MPWM_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->mpwm_port = port;
	cfg->mpwm_pin = pin;
}

void DRV8302_GPIO_MOC_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->moc_port = port;
	cfg->moc_pin = pin;
}

void DRV8302_GPIO_GAIN_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->gain_port = port;
	cfg->gain_pin = pin;
}

void DRV8302_GPIO_DCCAL_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->dc_cal_port = port;
	cfg->dc_cal_pin = pin;
}

void DRV8302_GPIO_OCTW_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->octw_port = port;
	cfg->octw_pin = pin;
}

void DRV8302_GPIO_FAULT_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->fault_port = port;
	cfg->fault_pin = pin;
}

void DRV8302_GPIO_ENGATE_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin) {
	cfg->en_gate_port = port;
	cfg->en_gate_pin = pin;
}

void DRV8302_ADC_config(DRV8302_t *cfg, volatile uint32_t *adc_ia, volatile uint32_t *adc_ib) {
    cfg->adc_ia = adc_ia;
    cfg->adc_ib = adc_ib;
}

static uint32_t get_timer_clock(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || 
        htim->Instance == TIM9 || htim->Instance == TIM10 || 
        htim->Instance == TIM11) {
        // Timer di APB2
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
            return pclk * 2;  // Timer clock doubled if APB prescaler != 1
        else
            return pclk;
    } else {
        // Timer di APB1
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
            return pclk * 2;
        else
            return pclk;
    }
}

int DRV8302_TIMER_config(DRV8302_t *cfg, TIM_HandleTypeDef *timer, uint32_t freq) {
	if (cfg == NULL || timer == NULL) return 0;

	cfg->timer = timer;

    const uint32_t timer_clock = get_timer_clock(timer);

    // prescaler dan period untuk mode center-aligned
    uint32_t prescaler = 0;
    uint32_t period = (timer_clock / (2 * freq)) - 1;

    if (period > 0xFFFF) {
        prescaler = period / 0xFFFF;
        period = (timer_clock / (2 * freq * (prescaler + 1))) - 1;
    }

    cfg->timer->Instance->PSC = prescaler;
    cfg->timer->Instance->ARR = period;
    
	cfg->pwm_resolution = period;

	return 1;
}

int DRV8302_stop_pwm(DRV8302_t *cfg) {
    if (cfg == NULL || cfg->timer == NULL) {
        return 0;
    }

    HAL_TIM_PWM_Stop(cfg->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(cfg->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(cfg->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(cfg->timer, TIM_CHANNEL_4);
    
    if (cfg->pwm_mode == _6_PWM_MODE) {
        HAL_TIMEx_PWMN_Stop(cfg->timer, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(cfg->timer, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(cfg->timer, TIM_CHANNEL_3);
    }

	return 1;
}

int DRV8302_start_pwm(DRV8302_t *cfg) {
    if (cfg == NULL || cfg->timer == NULL) {
        return 0;
    }

    HAL_TIM_PWM_Start(cfg->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(cfg->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(cfg->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(cfg->timer, TIM_CHANNEL_4);
    cfg->timer->Instance->CCR4 = cfg->timer->Instance->ARR - 1;
    
    if (cfg->pwm_mode == _6_PWM_MODE) {
        HAL_TIMEx_PWMN_Start(cfg->timer, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(cfg->timer, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(cfg->timer, TIM_CHANNEL_3);
    }

	return 1;
}

void DRV8302_set_mode(DRV8302_t *cfg, PWM_mode_t pwm_mode, OC_mode_t oc_mode) {
	cfg->pwm_mode = pwm_mode;
	cfg->oc_mode = oc_mode;
}

int DRV8302_current_sens_config(DRV8302_t *cfg, CSA_gain_t gain, float R_shunt, float v_offset_a, float v_offset_b) {
	if (cfg == NULL || gain > _40VPV || R_shunt <= 0) return 0;

	cfg->gain_mode = gain;
	cfg->R_shunt = R_shunt;
	cfg->v_offset_a = v_offset_a;
	cfg->v_offset_b = v_offset_b;

	const float i_gain = (cfg->gain_mode == _10VPV)? 10.0f : 40.0f;
	cfg->v_to_current = 1.0f / (i_gain * cfg->R_shunt);

	return 1;
}

int DRV8302_init(DRV8302_t *cfg) {
	if (cfg == NULL || cfg->timer == NULL) return 0;

#if USE_MPWM_EXTPIN
	HAL_GPIO_WritePin(cfg->mpwm_port, cfg->mpwm_pin, (cfg->pwm_mode == _3_PWM_MODE)? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#if USE_MOC_EXTPIN
	HAL_GPIO_WritePin(cfg->moc_port, cfg->moc_pin, (cfg->oc_mode == _SHUTDOWN_MODE)? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#if USE_GAIN_EXTPIN
	HAL_GPIO_WritePin(cfg->gain_port, cfg->gain_pin, (cfg->gain_mode == _40VPV)? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

	DRV8302_start_pwm(cfg);

	return 1;
}

void DRV8302_set_pwm(DRV8302_t *cfg, uint32_t pwma, uint32_t pwmb, uint32_t pwmc) {
    cfg->timer->Instance->CCR3 = pwma;
    cfg->timer->Instance->CCR2 = pwmb;
    cfg->timer->Instance->CCR1 = pwmc;
}

void DRV8302_get_current(DRV8302_t *cfg, float *ia, float *ib) {
    // Static filter state (retain between calls)
    static float ia_filtered = 0.0f;
    static float ib_filtered = 0.0f;

    // Convert to voltage
    const float vshunt_a = (float)*cfg->adc_ia * ADC_2_VOLT;
    const float vshunt_b = (float)*cfg->adc_ib * ADC_2_VOLT;

    // Compute raw phase currents (with shunt resistor gain and polarity)
    float ia_raw = (vshunt_a - cfg->v_offset_a) * cfg->v_to_current;
    float ib_raw = (vshunt_b - cfg->v_offset_b) * cfg->v_to_current;

    // IIR Low Pass Filter
    ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ia_filtered + CURRENT_FILTER_ALPHA * ia_raw;
    ib_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ib_filtered + CURRENT_FILTER_ALPHA * ib_raw;

    // Output
    *ia = ia_filtered;
    *ib = ib_filtered;
}


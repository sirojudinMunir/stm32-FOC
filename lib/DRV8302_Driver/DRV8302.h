/*
 * DRV8302.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef DRV8302_DRIVER_INC_DRV8302_H_
#define DRV8302_DRIVER_INC_DRV8302_H_

#include "DRV8302_config.h"

typedef enum {
	_6_PWM_MODE, // the device supports 6 independent PWM inputs
	_3_PWM_MODE // the device must be connected to ONLY 3 PWM input signals on INH_x
}PWM_mode_t;

typedef enum {
	_SHUTDOWN_MODE, // the gate driver will shutdown the channel which detected an over-current event
	_CYCLE_MODE // the gate driver will operate in a cycle-by-cycle current limiting mode
}OC_mode_t;

typedef enum {
	_10VPV,
	_40VPV
}CSA_gain_t;

typedef struct {
	GPIO_TypeDef *mpwm_port;  
	uint16_t mpwm_pin;   
	GPIO_TypeDef *moc_port;  
	uint16_t moc_pin;   
	GPIO_TypeDef *gain_port;  
	uint16_t gain_pin;   
	GPIO_TypeDef *dc_cal_port;
	uint16_t dc_cal_pin;
	GPIO_TypeDef *octw_port;
	uint16_t octw_pin;
	GPIO_TypeDef *fault_port;
	uint16_t fault_pin;
	GPIO_TypeDef *en_gate_port;
	uint16_t en_gate_pin;

    TIM_HandleTypeDef *timer;

	volatile uint32_t *adc_ia;
	volatile uint32_t *adc_ib;

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;

	uint32_t pwm_freq;
	uint32_t pwm_resolution;

	PWM_mode_t pwm_mode;
	OC_mode_t oc_mode;
	CSA_gain_t gain_mode;

	float R_shunt;
	float v_offset_a;
	float v_offset_b;
	float v_to_current; //pre-calculate conversion from voltaeg(V) to current(A)
}DRV8302_t;

#if USE_EN_GATE_EXTPIN
#define DRV8302_enable_gate(cfg) ((cfg)->en_gate_port->BSRR = (cfg)->en_gate_pin)
#define DRV8302_disable_gate(cfg) ((cfg)->en_gate_port->BSRR = (cfg)->en_gate_pin<<16)
#endif

#if USE_DC_CAL_EXTPIN
#define DRV8302_enable_dc_cal(cfg) ((cfg)->dc_cal_port->BSRR = (cfg)->dc_cal_pin)
#define DRV8302_disable_dc_cal(cfg) ((cfg)->dc_cal_port->BSRR = (cfg)->dc_cal_pin<<16)
#endif

void DRV8302_GPIO_MPWM_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_MOC_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_GAIN_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_DCCAL_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_OCTW_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_FAULT_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_ENGATE_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_ADC_config(DRV8302_t *cfg, volatile uint32_t *adc_ia, volatile uint32_t *adc_ib);
int DRV8302_TIMER_config(DRV8302_t *cfg, TIM_HandleTypeDef *timer, uint32_t freq);
int DRV8302_current_sens_config(DRV8302_t *cfg, CSA_gain_t gain, float R_shunt, float v_offset_a, float v_offset_b);
int DRV8302_stop_pwm(DRV8302_t *cfg);
int DRV8302_start_pwm(DRV8302_t *cfg);
void DRV8302_set_mode(DRV8302_t *cfg, PWM_mode_t pwm_mode, OC_mode_t oc_mode);
int DRV8302_init(DRV8302_t *cfg);
void DRV8302_set_pwm(DRV8302_t *cfg, uint32_t pwma, uint32_t pwmb, uint32_t pwmc);
void DRV8302_get_current(DRV8302_t *cfg, float *ia, float *ib);

#endif /* DRV8302_DRIVER_INC_DRV8302_H_ */

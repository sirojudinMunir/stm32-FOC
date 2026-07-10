#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <math.h>
#include "FOC_math.h"

typedef struct {
  volatile uint32_t *p_pwm_va;
  volatile uint32_t *p_pwm_vb;
  volatile uint32_t *p_pwm_vc;
  uint32_t *p_adc_ia;
  uint32_t *p_adc_ib;
  uint32_t *p_adc_ic;
  uint32_t *p_adc_vbus;
  float current_sense_gain;
  float ia_offset;
  float ib_offset;
  float ic_offset;
  float vbus_sense_gain;
  float vbus_actual;
  float ia;
  float ib;
  float ic;
  float elec_angle_rad;
}motor_t;

void motor_init_pwm(motor_t *hmotor, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c);
void motor_init_adc_current_sense(motor_t *hmotor, uint32_t *adc_a, uint32_t *adc_b, uint32_t *adc_c);
void motor_set_current_sense_gain(motor_t *hmotor, float gain);
void motor_set_current_sense_offset(motor_t *hmotor, float ia_offset, float ib_offset, float ic_offset);
void motor_set_pwm(motor_t *hmotor, uint32_t pwm_a, uint32_t pwm_b, uint32_t pwm_c);
void motor_get_current(motor_t *hmotor, float *ia, float *ib, float *ic);
void motor_calculate_current(motor_t *hmotor);


#endif // MOTOR_H
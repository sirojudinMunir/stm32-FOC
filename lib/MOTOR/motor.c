#include "motor.h"

void motor_init_pwm(motor_t *hmotor, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c) {
  hmotor->p_pwm_va = pwm_a;
  hmotor->p_pwm_vb = pwm_b;
  hmotor->p_pwm_vc = pwm_c;
}

void motor_init_adc_current_sense(motor_t *hmotor, uint32_t *adc_a, uint32_t *adc_b, uint32_t *adc_c) {
  hmotor->p_adc_ia = adc_a;
  hmotor->p_adc_ib = adc_b;
  hmotor->p_adc_ic = adc_c;
}

void motor_set_current_sense_gain(motor_t *hmotor, float gain) {
  hmotor->current_sense_gain = gain;
}

void motor_set_current_sense_offset(motor_t *hmotor, float ia_offset, float ib_offset, float ic_offset) {
  hmotor->ia_offset = ia_offset;
  hmotor->ib_offset = ib_offset;
  hmotor->ic_offset = ic_offset;
}

void motor_set_pwm(motor_t *hmotor, uint32_t pwm_a, uint32_t pwm_b, uint32_t pwm_c) {
  *hmotor->p_pwm_va = pwm_a;
  *hmotor->p_pwm_vb = pwm_b;
  *hmotor->p_pwm_vc = pwm_c;
}

void motor_get_current(motor_t *hmotor, float *ia, float *ib, float *ic) {
  *ia = hmotor->ia;
  *ib = hmotor->ib;
  *ic = hmotor->ic;
}

void motor_calculate_current(motor_t *hmotor) {
  float gain = hmotor->current_sense_gain;
  float current_a = (float)((int32_t)*hmotor->p_adc_ia - 2048) * gain - hmotor->ia_offset;
  float current_b = (float)((int32_t)*hmotor->p_adc_ib - 2048) * gain - hmotor->ib_offset;
  float current_c = 0.0f;
  if (hmotor->p_adc_ic) {
    current_c = (float)((int32_t)*hmotor->p_adc_ic - 2048) * gain - hmotor->ic_offset;
  }
  else {
    current_c = -current_a - current_b;
  }
  hmotor->ia = current_a;
  hmotor->ib = current_b;
  hmotor->ic = current_c;
}

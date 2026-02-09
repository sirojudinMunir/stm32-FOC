/*
 * pid_utils.h
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#ifndef PID_INC_PID_UTILS_H_
#define PID_INC_PID_UTILS_H_

#include <stdint.h>

typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Differential gain
    float integral;  // Integral accumulator
    float out_max;   // Output upper limit (clamp)
    float out_max_dynamic;
    float ts;
    float e_deadband;
    float last_error;
    float d_alpha_filter; // Derivative filter coefficient
    float d_filtered;
    float d_max;
    float mv;
} PID_Controller_t;

float pi_control(PID_Controller_t *pi, float error);
float pd_control(PID_Controller_t *pd, float error);
float pid_control(PID_Controller_t *pid, float error);
void pid_reset(PID_Controller_t *p);
void pid_set_kp(PID_Controller_t *pid, float kp);
void pid_set_ki(PID_Controller_t *pid, float ki);
void pid_set_kd(PID_Controller_t *pid, float kd);
void pid_set_ts(PID_Controller_t *pid, float ts);
void pid_set_max_out(PID_Controller_t *pid, float max);
void pid_set_max_out_dynamic(PID_Controller_t *pid, float max);
void pid_set_deadband(PID_Controller_t *pid, float deadband);
void pid_set_d_filter_fc(PID_Controller_t *pid, float fc);
void pid_set_max_d(PID_Controller_t *pid, float max);

#endif /* PID_INC_PID_UTILS_H_ */

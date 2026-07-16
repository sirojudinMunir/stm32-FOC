/*
 * pid_utils.c
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#include "pid_utils.h"

#ifndef TWO_PI
#define TWO_PI 6.2831853f
#endif

float pi_control(PID_Controller_t *pi, float error) {
    if (error >= -pi->e_deadband && error <= pi->e_deadband) {
        error = 0.0f;
    }

    float p_term = pi->kp * error;

    float new_integral = pi->integral + error * pi->ki * pi->ts;

    float output = p_term + new_integral;

    _Bool is_saturated = 0;

    // Anti-windup with clamping
    if (output > pi->out_max) {
        output = pi->out_max;
        if (p_term < output) is_saturated = 1;
    }
    else if (output < pi->out_min) {
        output = pi->out_min;
        if (p_term > output) is_saturated = 1;
    }

    if (is_saturated) {
        pi->integral = output - p_term;
    }
    else {
        pi->integral = new_integral;
    }

    pi->mv = output;

    return output;
}

float pd_control(PID_Controller_t *pd, float error) {
    if (error >= -pd->e_deadband && error <= pd->e_deadband) {
        pd->last_error = 0.0f;  // Reset last_error ketika dalam deadband
        return 0.0f;
    }

    float derivative = (error - pd->last_error);

    pd->last_error = error;

    float p_term = pd->kp * error;
    float d_term = pd->kd / pd->ts * derivative;

    float output = p_term + d_term;

    // Output clamping
    if (output > pd->out_max) {
        output = pd->out_max;
    }
    else if (output < pd->out_min) {
        output = pd->out_min;
    }

    return output;
}

float pid_control(PID_Controller_t *pid, float error) {
    if (error >= -pid->e_deadband && error <= pid->e_deadband) {
        error = 0.0f;
    }

    float p_term = pid->kp * error;

    float error_derivative = (error - pid->last_error) / pid->ts;
    pid->d_filtered = (1.0f - pid->d_alpha_filter) * pid->d_filtered + pid->d_alpha_filter * error_derivative;
    // clamp derivative
    if (pid->d_filtered > pid->d_max) pid->d_filtered = pid->d_max;
    else if (pid->d_filtered < -pid->d_max) pid->d_filtered = -pid->d_max;
    float d_term = pid->d_filtered * pid->kd;
    pid->last_error = error;

    float new_integral = pid->integral + error * pid->ki * pid->ts;
    float pd_term = p_term + d_term;
    float output = pd_term + new_integral;

    _Bool is_saturated = 0;

    // Anti-windup with clamping
    if (output > pid->out_max) {
        output = pid->out_max;
        if (pd_term < output) is_saturated = 1;
    }
    else if (output < pid->out_min) {
        output = pid->out_min;
        if (pd_term > output) is_saturated = 1;
    }

    if (is_saturated) {
        pid->integral = output - pd_term;
    }
    else {
        pid->integral = new_integral;
    }

    return output;
}

void pid_reset(PID_Controller_t *p) {
	p->integral = 0;
    p->last_error = 0.0f;
}

void pid_set_kp(PID_Controller_t *pid, float kp) {
    if (kp < 0) return;
    pid->kp = kp;
}

void pid_set_ki(PID_Controller_t *pid, float ki) {
    if (ki < 0) return;
    pid->ki = ki;
}

void pid_set_kd(PID_Controller_t *pid, float kd) {
    if (kd < 0) return;
    pid->kd = kd;
}

void pid_set_ts(PID_Controller_t *pid, float ts) {
    if (ts <= 0) return;
    pid->ts = ts;
}

void pid_set_out_constraint(PID_Controller_t *pid, float max, float min) {
    pid->out_max = max;
    pid->out_min = min;
}

void pid_set_deadband(PID_Controller_t *pid, float deadband) {
    pid->e_deadband = deadband;
}

void pid_set_d_filter_fc(PID_Controller_t *pid, float fc) {
    pid->d_fc_lpf = fc;
    float tau = 1.0f / (TWO_PI * fc);
    pid->d_alpha_filter = pid->ts / (tau + pid->ts);
    if (pid->d_alpha_filter > 1.0f) pid->d_alpha_filter = 1.0f;
}

void pid_set_max_d(PID_Controller_t *pid, float max) {
    if (max <= 0) return;
    pid->d_max = max;
}

float pid_get_kp(PID_Controller_t *pid) {
    return pid->kp;
}

float pid_get_ki(PID_Controller_t *pid) {
    return pid->ki;
}

float pid_get_kd(PID_Controller_t *pid) {
    return pid->kd;
}

float pid_get_ts(PID_Controller_t *pid) {
    return pid->ts;
}

float pid_get_out_max(PID_Controller_t *pid) {
    return pid->out_max;
}

float pid_get_out_min(PID_Controller_t *pid) {
    return pid->out_min;
}

float pid_get_deadband(PID_Controller_t *pid) {
    return pid->e_deadband;
}

float pid_get_d_filter_fc(PID_Controller_t *pid) {
    return pid->d_fc_lpf;
}

float pid_get_d_alpha_filter(PID_Controller_t *pid) {
    return pid->d_alpha_filter;
}

float pid_get_max_d(PID_Controller_t *pid) {
    return pid->d_max;
}
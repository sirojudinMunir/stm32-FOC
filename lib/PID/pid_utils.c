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

    // Anti-windup with clamping
    if (output > pi->out_max) {
        output = pi->out_max;
        if (error * (output - p_term) <= 0) { 
            pi->integral = new_integral;
        }
    }
    else if (output < -pi->out_max) {
        output = -pi->out_max;
        if (error * (output - p_term) <= 0) {
            pi->integral = new_integral;
        }
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
    else if (output < -pd->out_max) {
        output = -pd->out_max;
    }

    return output;
}

float pid_control(PID_Controller_t *pid, float error) {
    if (error >= -pid->e_deadband && error <= pid->e_deadband) {
        error = 0.0f;
    }

    float p_term = pid->kp * error;

#if 1
    float error_derivative = (error - pid->last_error) / pid->ts;
    pid->d_filtered = (1.0f - pid->d_alpha_filter) * pid->d_filtered + pid->d_alpha_filter * error_derivative;
    // clamp derivative
    if (pid->d_filtered > pid->d_max) pid->d_filtered = pid->d_max;
    else if (pid->d_filtered < -pid->d_max) pid->d_filtered = -pid->d_max;
    float d_term = pid->d_filtered * pid->kd;
#else
    float error_derivative = error - pid->last_error;
    pid->d_filtered += pid->d_alpha_filter * ((pid->kd * error_derivative / pid->ts) - pid->d_filtered);
    // clamp derivative
    if (pid->d_filtered > pid->d_max) pid->d_filtered = pid->d_max;
    else if (pid->d_filtered < -pid->d_max) pid->d_filtered = -pid->d_max;
    float d_term = pid->d_filtered;
#endif
    pid->last_error = error;

    float new_integral = pid->integral + error * pid->ki * pid->ts;

    float output = p_term + d_term + new_integral;

    // Anti-windup with clamping
    if (output > pid->out_max) {
        output = pid->out_max;
        if ((output - (p_term + d_term)) * error <= 0) {
            pid->integral = new_integral; 
        }
    }
    else if (output < -pid->out_max) {
        output = -pid->out_max;
        if ((output - (p_term + d_term)) * error <= 0) {
            pid->integral = new_integral;
        }
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

void pid_set_max_out(PID_Controller_t *pid, float max) {
    if (max <= 0) return;
    pid->out_max = max;
}

void pid_set_max_out_dynamic(PID_Controller_t *pid, float max) {
    if (max <= 0) return;
    pid->out_max_dynamic = max;
}

void pid_set_deadband(PID_Controller_t *pid, float deadband) {
    pid->e_deadband = deadband;
}

void pid_set_d_filter_fc(PID_Controller_t *pid, float fc) {
    float tau = 1.0f / (TWO_PI * fc);
    pid->d_alpha_filter = pid->ts / (tau + pid->ts);
    if (pid->d_alpha_filter > 1.0f) pid->d_alpha_filter = 1.0f;
}

void pid_set_max_d(PID_Controller_t *pid, float max) {
    if (max <= 0) return;
    pid->d_max = max;
}
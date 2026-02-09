/*
 * sliding_mode_observer.h
 *
 *  Created on: Oct 30, 2025
 *      Author: munir
 */

#ifndef SLIDING_MODE_OBSERVER_H_
#define SLIDING_MODE_OBSERVER_H_

#include <stdint.h>

typedef struct {
    float pole_pairs;
    float min_operating_emf;
    float Ts;

    float A, b, b_inv;
    float k_gain; // sliding gain
    float g; // BEMF observer gain
    float last_i_alpha_error;
    float last_i_beta_error;
    
    float theta_est;
    float omega_est;
    float e_theta;
    float last_e_theta;
    float rpm_est;
    float i_alpha_est, i_beta_est;
    float e_alpha_est, e_beta_est;
    float emf_magnitude;

    // Filtering
    float omega_alpha_filter;
}smo_t;

void smo_init(smo_t *hsmo, float Rs, float Ls, float pole_pairs, float Ts);
void smo_reset(smo_t *hsmo);
void smo_set_min_emf(smo_t *hsmo, float min_emf);
void smo_update_R_L(smo_t *hsmo, float Rs, float Ls);

int smo_update_arctan(smo_t *hsmo, float v_alpha, float v_beta,
                      float i_alpha_meas, float i_beta_meas);
float smo_get_rotor_angle(smo_t *hsmo);
float smo_get_rotor_speed(smo_t *hsmo);

#endif

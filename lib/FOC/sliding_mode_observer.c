#include "sliding_mode_observer.h"
#include "FOC_math.h"
#include <math.h>

float smo_sign(float x, float deadband) {
    if (x >= deadband) return 1.0f;
    if (x <= -deadband) return -1.0f;
    return x / deadband; // Linear region
}

float smo_tanh(float x, float scale) {
    return tanhf(scale * x);
}

void smo_init(smo_t *hsmo, float Rs, float Ls, float pole_pairs, float Ts) {
    hsmo->pole_pairs = pole_pairs;
    hsmo->Ts = Ts;

    hsmo->A = expf(-(Rs / Ls) * Ts);
    hsmo->b = (1.0f - hsmo->A) / Rs;
    hsmo->b_inv = 1.0f / hsmo->b;
    
    // sliding mode control gain
    hsmo->k_gain = 5.0f; //2.0f;
    
    // back-emf observer gain (0 < g < 1)
    hsmo->g = 0.1f;
    
    hsmo->omega_alpha_filter = 0.05f; //0.02f;
    
    hsmo->min_operating_emf = 0.25f;

    smo_reset(hsmo);
}

void smo_reset(smo_t *hsmo) {
    hsmo->theta_est = 0.0f;
    hsmo->omega_est = 0.0f;
    hsmo->i_alpha_est = 0.0f;
    hsmo->i_beta_est = 0.0f;
    hsmo->e_alpha_est = 0.0f;
    hsmo->e_beta_est = 0.0f;
    hsmo->last_i_alpha_error = 0.0f;
    hsmo->last_i_beta_error = 0.0f;
}

void smo_set_min_emf(smo_t *hsmo, float min_emf) {
    hsmo->min_operating_emf = min_emf;
}

void smo_update_R_L(smo_t *hsmo, float Rs, float Ls) {
    hsmo->A = expf(-(Rs / Ls) * hsmo->Ts);
    hsmo->b = (1.0f - hsmo->A) / Rs;
}

// reference: https://www.mathworks.com/help/mcb/ref/slidingmodeobserver.html#mw_753c13aa-dd04-4172-9ccf-d469fedead4d
int smo_update_arctan(smo_t *hsmo, float v_alpha, float v_beta,
                      float i_alpha_meas, float i_beta_meas) {
    // estimate I in alpha beta frame
    float i_alpha_error = hsmo->i_alpha_est - i_alpha_meas;
    float i_beta_error = hsmo->i_beta_est - i_beta_meas;

    float z_alpha = hsmo->k_gain * smo_sign(i_alpha_error, 0.1f);
    float z_beta = hsmo->k_gain * smo_sign(i_beta_error, 0.1f);
    
    hsmo->i_alpha_est = hsmo->A * hsmo->i_alpha_est + hsmo->b * (v_alpha - hsmo->e_alpha_est) - z_alpha;
    hsmo->i_beta_est  = hsmo->A * hsmo->i_beta_est + hsmo->b * (v_beta - hsmo->e_beta_est) - z_beta;

    // estimate back-emf in alpha beta frame    
    float last_z_alpha = hsmo->k_gain * smo_sign(hsmo->last_i_alpha_error, 0.1f);
    float last_z_beta = hsmo->k_gain * smo_sign(hsmo->last_i_beta_error, 0.1f);

    hsmo->e_alpha_est += hsmo->b_inv * hsmo->g * (i_alpha_error - hsmo->A * hsmo->last_i_alpha_error + last_z_alpha);
    hsmo->e_beta_est += hsmo->b_inv * hsmo->g * (i_beta_error - hsmo->A * hsmo->last_i_beta_error + last_z_beta);

    hsmo->last_i_alpha_error = i_alpha_error;
    hsmo->last_i_beta_error = i_beta_error;

    hsmo->emf_magnitude = sqrtf(hsmo->e_alpha_est * hsmo->e_alpha_est + 
                               hsmo->e_beta_est * hsmo->e_beta_est);
    
    // estimate angle position and speed
    if (hsmo->emf_magnitude > hsmo->min_operating_emf) {
        hsmo->e_theta = fast_atan2(hsmo->e_beta_est, hsmo->e_alpha_est);

        if (hsmo->omega_est > 0) {
            hsmo->theta_est = hsmo->e_theta - 0.5f*PI;
        }
        else {
            hsmo->theta_est = hsmo->e_theta + 0.5f*PI;
        }

        if (hsmo->theta_est < 0.0f) {
            hsmo->theta_est += TWO_PI;
        }
        else if (hsmo->theta_est > TWO_PI) {
            hsmo->theta_est -= TWO_PI;
        }
        
        float delta_theta = hsmo->e_theta - hsmo->last_e_theta;
        
        if (delta_theta > PI) delta_theta -= TWO_PI;
        if (delta_theta < -PI) delta_theta += TWO_PI;
        
        float delta_theta_mech = delta_theta / hsmo->pole_pairs;
        float omega_instant = delta_theta_mech / hsmo->Ts;
        
        hsmo->omega_est = hsmo->omega_est * (1.0f - hsmo->omega_alpha_filter) + 
                         omega_instant * hsmo->omega_alpha_filter;

        hsmo->rpm_est = hsmo->omega_est * 60.0 / TWO_PI;
        
        hsmo->last_e_theta = hsmo->e_theta;
        return 1;
    } else {
        hsmo->omega_est = 0.0f;
        hsmo->rpm_est = 0.0f;
        // hsmo->theta_est = 0.0f;
    }
    return 0;
}

float smo_get_rotor_angle(smo_t *hsmo) {
    return hsmo->theta_est;
}

float smo_get_rotor_speed(smo_t *hsmo) {
    return hsmo->rpm_est;
}
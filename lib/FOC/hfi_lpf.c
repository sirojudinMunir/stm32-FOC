#include "hfi_lpf.h"
// Reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8746146

void hfi_lpf_init(hfi_lpf_t *hhfi, float Ld, float v_h, float f_h, float lpf_fc, float sampling_freq) {
    hhfi->omega_h = TWO_PI * f_h;
    hhfi->v_h = v_h;
    float k_h = hhfi->v_h / (hhfi->omega_h * Ld);
    pll_init(&hhfi->pll, 1000.0f, 5000.0f, k_h, 50000.0f);
    
    second_order_lpf_init(&hhfi->i_alpha_l_lpf, lpf_fc, sampling_freq);
    second_order_lpf_init(&hhfi->i_beta_lpf, lpf_fc, sampling_freq);
}

void hfi_lpf_update_estimate_position(hfi_lpf_t *hhfi, float i_alpha, float i_beta, float Ts) {
    float two_sin_omega_t = 2.0f * fast_sin(hhfi->phase_h);
    float i_alpha_l_raw = i_alpha * two_sin_omega_t;
    float i_beta_l_raw = i_beta * two_sin_omega_t;
    
    hhfi->i_alpha_l = second_order_lpf_update(&hhfi->i_alpha_l_lpf, i_alpha_l_raw);
    hhfi->i_beta_l = second_order_lpf_update(&hhfi->i_beta_lpf, i_beta_l_raw);

    // Phase Detector
    float error = hhfi->i_beta_l * fast_cos(hhfi->pll.theta_est) - hhfi->i_alpha_l * fast_sin(hhfi->pll.theta_est);
    pll_update(&hhfi->pll, error, Ts);

    // update phase
    hhfi->phase_h += (hhfi->omega_h * Ts);
    if (hhfi->phase_h >= TWO_PI) hhfi->phase_h -= TWO_PI;
}

float hfi_lpf_get_v_inj(hfi_lpf_t *hhfi) {
    float v_inj = hhfi->v_h * fast_cos(hhfi->phase_h);
    return v_inj;
}

float hfi_lpf_get_estimate_position(hfi_lpf_t *hhfi) {
    return hhfi->pll.theta_est;
}

float hfi_lpf_get_estimate_omega(hfi_lpf_t *hhfi) {
    return hhfi->pll.omega_est;
}

void hfi_lpf_force_estimate_position(hfi_lpf_t *hhfi, float theta) {
    hhfi->pll.theta_est = theta;
}

void hfi_lpf_reset(hfi_lpf_t *hhfi) {
    pll_reset(&hhfi->pll);

    hhfi->phase_h = 0.0f;
}
#include "hfi_sdft.h"
#include "math.h"
#include <string.h>
// Reference: https://www.sciencedirect.com/science/article/pii/S2214914723002155

void hfi_init(hfi_t *hhfi, float v_h, float f_h, float sampling_freq) {
    hhfi->omega_h = TWO_PI * f_h;
    hhfi->v_h = v_h;
    hhfi->f_h = f_h;
    pll_init(&hhfi->pll, 2000.0f, 6000.0f, 1.0f, 50000.0f);
    
    hhfi->sdft_window_size = sampling_freq / f_h;
    hhfi->sdft_buff_idx = 0;
    
    // calculate rotation factor: e^(j2pi/N) for k=1
    float theta = TWO_PI / hhfi->sdft_window_size;
    hhfi->sdft_rotation_factor.real = fast_cos(theta);
    hhfi->sdft_rotation_factor.imag = fast_sin(theta);

    // Reset buffer
    for(int i = 0; i < hhfi->sdft_window_size; i++) {
        hhfi->sdft_iq_buffer[i] = 0.0f;
    }
    
    // Reset SDFT state
    hhfi->Xk_real = 0.0f;
    hhfi->Xk_imag = 0.0f;
    hhfi->last_Xk.real = 0.0f;
    hhfi->last_Xk.imag = 0.0f;
}

void hfi_update_estimate_position(hfi_t *hhfi, float iq, float Ts) {
    int N = hhfi->sdft_window_size;

    // SDFT update
    float oldest_iq = hhfi->sdft_iq_buffer[0];
    memmove(hhfi->sdft_iq_buffer, &hhfi->sdft_iq_buffer[1], (N-1) * sizeof(float));
    hhfi->sdft_iq_buffer[N - 1] = iq;
    
    // SDFT: X_k[n] = e^(j2pik/N) * (X_k[n-1] + x[n] - x[n-N])
    float delta = iq - oldest_iq;
    complex_t delta_complex = {delta, 0.0f};
    complex_t sum = complex_add(hhfi->last_Xk, delta_complex);
    
    // Multiply with rotation factor
    complex_t Xk_new = complex_multiply(sum, hhfi->sdft_rotation_factor);
    hhfi->last_Xk = Xk_new;
    
    // Extract amplitude (eq 14)
    hhfi->sdft_amplitude = 2.0f * sqrtf(Xk_new.real * Xk_new.real + Xk_new.imag * Xk_new.imag) / (float)N;

    // i_gamma1 (eq 17)
    float theta_0 = fast_atan2(Xk_new.imag, Xk_new.real);
    float phase_correction = (float)(N - 1) / (float)N * TWO_PI;
    float i_gamma1 = hhfi->sdft_amplitude * fast_cos(theta_0 + phase_correction);
    
    hhfi->sdft_fundamental = i_gamma1;
    
    // sign detector (eq 18)
    float sign_product = i_gamma1 * fast_cos(hhfi->last_phase_h);
    hhfi->sdft_phase = sign_product;
    int8_t p;
    if (sign_product >= 0) {
        hhfi->s++;
        if (hhfi->s >= 1) {
            hhfi->s = 1;
            p = 1;
        }
        else {
            hhfi->s = 0;
            p = -1;
        }
    }
    else {
        hhfi->s--;
        if (hhfi->s <= -1) {
            hhfi->s = -1;
            p = -1;
        }
        else {
            hhfi->s = 0;
            p = 1;
        }
    }
    
    float hfi_error = (p > 0)? -hhfi->sdft_amplitude : hhfi->sdft_amplitude;
    pll_update(&hhfi->pll, hfi_error, Ts);

    // update phase
    hhfi->last_phase_h = hhfi->phase_h;
    hhfi->phase_h += (hhfi->omega_h * Ts);
    if (hhfi->phase_h >= TWO_PI) hhfi->phase_h -= TWO_PI;
}

float hfi_get_v_inj(hfi_t *hhfi) {
    float v_inj = hhfi->v_h * fast_sin(hhfi->phase_h);
    return v_inj;
}

float hfi_get_estimate_position(hfi_t *hhfi) {
    return hhfi->pll.theta_est;
}

float hfi_get_estimate_omega(hfi_t *hhfi) {
    return hhfi->pll.omega_est;
}

void hfi_force_estimate_position(hfi_t *hhfi, float theta) {
    hhfi->pll.theta_est = theta;
}

void hfi_reset(hfi_t *hhfi) {
    pll_reset(&hhfi->pll);

    hhfi->phase_h = 0.0f;
    hhfi->sdft_buff_idx = 0;

    // Reset buffer
    for(int i = 0; i < hhfi->sdft_window_size; i++) {
        hhfi->sdft_iq_buffer[i] = 0.0f;
    }
    
    // Reset SDFT state
    hhfi->Xk_real = 0.0f;
    hhfi->Xk_imag = 0.0f;
    hhfi->last_Xk.real = 0.0f;
    hhfi->last_Xk.imag = 0.0f;
}
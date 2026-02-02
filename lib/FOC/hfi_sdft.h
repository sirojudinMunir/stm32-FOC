#ifndef HFI_SDFT_H_
#define HFI_SDFT_H_

#include <stdint.h>
#include "FOC_math.h"
#include "pll.h"

typedef struct {
	float phase_h;
	float last_phase_h;
	float v_h; // amplitudo
	float f_h; // frequency 
	float omega_h;
	int sdft_window_size;
	int sdft_buff_idx;
	float sdft_iq_buffer[128];
	float sdft_amplitude;
	float sdft_phase;
	float sdft_fundamental;
	float theta_error;
	float Xk_real;
	float Xk_imag;
	complex_t last_Xk;
	complex_t sdft_rotation_factor;
	pll_t pll;
    int8_t s;
}hfi_t;

void hfi_init(hfi_t *hhfi, float v_h, float f_h, float sampling_freq);
void hfi_update_estimate_position(hfi_t *hhfi, float iq, float Ts);
float hfi_get_v_inj(hfi_t *hhfi);
float hfi_get_estimate_position(hfi_t *hhfi);
float hfi_get_estimate_omega(hfi_t *hhfi);
void hfi_force_estimate_position(hfi_t *hhfi, float theta);
void hfi_reset(hfi_t *hhfi);

#endif
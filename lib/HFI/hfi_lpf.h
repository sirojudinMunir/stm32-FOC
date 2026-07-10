#ifndef HFI_LPF_H_
#define HFI_LPF_H_

#include <stdint.h>
#include "FOC_math.h"
#include "lpf.h"
#include "pll.h"

typedef struct {
	float phase_h;
	float v_h; // amplitudo
	float f_h; // frequency 
	float omega_h;
	float alpha_filter_h;
	float i_alpha_l; // low-frequency / fundamental
	float i_beta_l; // low-frequency / fundamental
	pll_t pll;
	SecondOrderLPF i_alpha_l_lpf;
	SecondOrderLPF i_beta_lpf;
}hfi_lpf_t;

void hfi_lpf_init(hfi_lpf_t *hhfi, float Ld, float v_h, float f_h, float lpf_fc, float sampling_freq);
void hfi_lpf_update_estimate_position(hfi_lpf_t *hhfi, float i_alpha, float i_beta, float Ts);
float hfi_lpf_get_v_inj(hfi_lpf_t *hhfi);
float hfi_lpf_get_estimate_position(hfi_lpf_t *hhfi);
float hfi_lpf_get_estimate_omega(hfi_lpf_t *hhfi);
void hfi_lpf_force_estimate_position(hfi_lpf_t *hhfi, float theta);
void hfi_lpf_reset(hfi_lpf_t *hhfi);

#endif
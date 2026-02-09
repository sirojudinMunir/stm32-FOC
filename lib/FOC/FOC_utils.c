/*
 * FOC_utils.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "FOC_utils.h"
#include "flash.h"
#include "controller_app.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// extern from main.c
extern motor_config_t m_config;

_Bool foc_ready = 0;

float Vd_buff[MAX_I_SAMPLE];
float Vq_buff[MAX_I_SAMPLE];
float Id_buff[MAX_I_SAMPLE];
float Iq_buff[MAX_I_SAMPLE];

#if DEBUG_HFI
float param1_debug_buff[MAX_SAMPLE_BUFF];
float param2_debug_buff[MAX_SAMPLE_BUFF];
float param3_debug_buff[MAX_SAMPLE_BUFF];
float param4_debug_buff[MAX_SAMPLE_BUFF];
#endif

float error_temp[ERROR_LUT_SIZE] = {0};

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv) {
	if (hfoc == NULL || pole_pairs == 0 || kv <= 0) {
		return;
	}

	hfoc->pole_pairs = pole_pairs;
	hfoc->kv = kv;
}

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir) {
	if (hfoc == NULL) return;

	hfoc->m_angle_offset = m_rad_offset;
	hfoc->sensor_dir = sensor_dir;
}

void foc_gear_reducer_init(foc_t *hfoc, float ratio) {
	if (hfoc == NULL) return;

	hfoc->gear_ratio = ratio;
}

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	if (hfoc == NULL) return;

	hfoc->max_current = i_limit;
}

void foc_set_mode(foc_t *hfoc, foc_mode_t mode) {
    hfoc->foc_mode = mode;
}

void foc_disable(foc_t *hfoc) {
    DRV8302_disable_gate(&hfoc->drv8302);
}

void foc_speed_control_update(foc_t *hfoc, float rpm_reference) {
	if (hfoc == NULL || (hfoc->control_mode != SPEED_CONTROL_MODE && hfoc->control_mode != POSITION_CONTROL_MODE)) {
		hfoc->speed_ctrl.integral = 0.0f;
		hfoc->speed_ctrl.last_error = 0.0f;
		return;
	}

    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = pid_control(&hfoc->speed_ctrl, rpm_reference - hfoc->actual_rpm);
}

void foc_position_control_update(foc_t *hfoc, float deg_reference) {
    if (hfoc == NULL || hfoc->control_mode != POSITION_CONTROL_MODE) {
        hfoc->pos_ctrl.integral = 0.0f;
        hfoc->pos_ctrl.last_error = 0.0f;
        return;
    }
    
    // Normalize reference angle to 0-360 degrees
    deg_reference = fmodf(deg_reference, 360.0f);
    if (deg_reference < 0) {
        deg_reference += 360.0f;
    }
    
    if (hfoc->loop_count >= (POSITION_CONTROL_CYCLE / SPEED_CONTROL_CYCLE)) {
        hfoc->loop_count = 0;
        
        // Calculate shortest path error with wrap-around
        float error = deg_reference - hfoc->actual_angle;
        
        // Handle wrap-around for shortest path
        if (error > 180.0f) {
            error -= 360.0f;
        } else if (error < -180.0f) {
            error += 360.0f;
        }
        
        hfoc->rpm_ref = pid_control(&hfoc->pos_ctrl, error);
    }
    hfoc->loop_count++;

    foc_speed_control_update(hfoc, hfoc->rpm_ref);
}

void foc_sensored_calc_electric_angle(foc_t *hfoc) {
    // Check for NULL pointer and invalid parameters
    if (hfoc == NULL || hfoc->pole_pairs <= 0 || 
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI ||
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
        return;
    }

    float angle_deg = AS5047P_get_degree(&hfoc->as5047p);

    // Normalize mechanical angle
    hfoc->m_angle_rad = DEG_TO_RAD(angle_deg) - hfoc->m_angle_offset;
    norm_angle_rad(&hfoc->m_angle_rad);

    // Calculate raw electric angle
    float e_rad = hfoc->m_angle_rad * hfoc->pole_pairs;
    
    // Handle sensor direction
    if (hfoc->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }

    hfoc->e_angle_rad = e_rad;

    // Calculate LUT index with wrap-around
    float lut_idx_f = (hfoc->m_angle_rad / TWO_PI) * ERROR_LUT_SIZE;
    lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE);
    if (lut_idx_f < 0) {
        lut_idx_f += ERROR_LUT_SIZE;
    }

    // Get neighboring indices with wrap-around
    int idx0 = (int)lut_idx_f % ERROR_LUT_SIZE;
    int idx1 = (idx0 + 1) % ERROR_LUT_SIZE;
    float frac = lut_idx_f - (float)idx0;

    // Linear interpolation
    float encoder_error = m_config.encd_error_comp[idx0] * (1.0f - frac) + m_config.encd_error_comp[idx1] * frac;
    e_rad += encoder_error;
    
    // Normalize final electric angle
    norm_angle_rad(&e_rad);

    hfoc->e_angle_rad_comp = e_rad;
    
    AS5047P_set_val_flag();
}

float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm) {
    if (hfoc->sensor_dir == REVERSE_DIR) {
        hfoc->actual_rpm = -encd_rpm;
    }
    else {
        hfoc->actual_rpm = encd_rpm;
    }
    return hfoc->actual_rpm;
}

float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg) {
    if (hfoc->sensor_dir == REVERSE_DIR) {
        hfoc->actual_angle = -encd_deg * hfoc->gear_ratio;
    }
    else {
        hfoc->actual_angle = encd_deg * hfoc->gear_ratio;
    }
    return hfoc->actual_angle;
}

void foc_cal_encoder_misalignment(foc_t *hfoc) {
  open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
  foc_delay_ms(500);
  float rad_offset = 0.0f;
  for (int i = 0; i < CAL_ITERATION; i++) {
    rad_offset += DEG_TO_RAD(hfoc->as5047p.angle_filtered);
    foc_delay_ms(1);
  }
  open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
  rad_offset = rad_offset / (float)CAL_ITERATION;
  hfoc->m_angle_offset = rad_offset;
  m_config.encd_offset = rad_offset;
}

void foc_cal_encoder(foc_t *hfoc) {
  memset(error_temp, 0, sizeof(error_temp));

  foc_cal_encoder_misalignment(hfoc);

  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
    float elec_rad = DEG_TO_RAD(mech_deg * hfoc->pole_pairs);
    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, elec_rad);
    foc_delay_ms(5);

    float mech_rad = hfoc->m_angle_rad;
    float raw_delta = elec_rad - hfoc->e_angle_rad;
    float delta = elec_rad - hfoc->e_angle_rad_comp;
    
    raw_delta -= TWO_PI * floorf((raw_delta + PI) / TWO_PI);
    delta -= TWO_PI * floorf((delta + PI) / TWO_PI);
    
    float lut_pos = (mech_rad / TWO_PI) * ERROR_LUT_SIZE;
    int index = (int)(lut_pos);

    while (index < 0) {
      index += ERROR_LUT_SIZE;
    }
    index %= ERROR_LUT_SIZE;

    error_temp[index] = raw_delta;
    
    // debug
    // float buffer_val[2];
    // buffer_val[0] = raw_delta;
    // buffer_val[1] = delta;
    // send_data_float(buffer_val, 2);
  }
  
  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    if (error_temp[i] == 0) {
      int last_i = i - 1;
      int next_i = i + 1;
      if (last_i < 0) last_i += ERROR_LUT_SIZE;
      if (next_i > ERROR_LUT_SIZE) next_i -= ERROR_LUT_SIZE;
      error_temp[i] = (error_temp[last_i] + error_temp[next_i]) / 2.0f;
    }
  }

  memcpy(m_config.encd_error_comp, error_temp, sizeof(error_temp));

  open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
}

void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth) {
    hfoc->I_ctrl_bandwidth = bandwidth;
}

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    uint32_t da, db, dc;
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus, hfoc->drv8302.pwm_resolution, &da, &db, &dc);
    DRV8302_set_pwm(&hfoc->drv8302, da, db, dc);
}

void meas_inj_dq_process(foc_t *hfoc, float ts) {
    const uint32_t wt = 256; // waiting time

    if (hfoc->meas_inj_start_flag) {
        float vd = 0.0f, vq = 0.0f;
        if (hfoc->meas_inj_target == RS) {
            vd = hfoc->meas_inj_amp;
            vq = 0.0f;
        }
        else {
            float angle = hfoc->meas_inj_omega * hfoc->meas_inj_n * ts;
            float v_inj = hfoc->meas_inj_amp * fast_sin(angle);
            if (hfoc->meas_inj_target == LD) {
                vd = v_inj;
                vq = 0.0f;
            }
            else if (hfoc->meas_inj_target == LQ) {
                vd = 0.0f;
                vq = v_inj;
            }
        }

        // float theta_e = hfoc->e_angle_rad_comp;
        float theta_e = 0.0f;

        open_loop_voltage_control(hfoc, vd, vq, theta_e);

        DRV8302_get_current(&hfoc->drv8302, &hfoc->ia, &hfoc->ib);

        float sin_theta, cos_theta;
        float id, iq;
        pre_calc_sin_cos(theta_e, &sin_theta, &cos_theta);
        clarke_park_transform(hfoc->ia, hfoc->ib, sin_theta, cos_theta, &id, &iq);

        if (hfoc->meas_inj_n >= wt) {
            if (hfoc->meas_inj_target == RS || hfoc->meas_inj_target == LD) {
                Vd_buff[hfoc->meas_inj_n - wt] = vd;
                Id_buff[hfoc->meas_inj_n - wt] = id;
            }
            else if (hfoc->meas_inj_target == LQ) {
                Vq_buff[hfoc->meas_inj_n - wt] = vq;
                Iq_buff[hfoc->meas_inj_n - wt] = iq;
            }
        }

        hfoc->meas_inj_n++;
        if (hfoc->meas_inj_n >= (MAX_I_SAMPLE + wt)) {
            hfoc->meas_inj_n = 0;
            hfoc->meas_inj_start_flag = 0;
            open_loop_voltage_control(hfoc, 0, 0, 0);
        }
    }
}

void estimate_resistance(foc_t *hfoc) {
    float mean_vd = 0, mean_id = 0;

    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_id += Id_buff[i];
    }
    mean_vd /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;

    hfoc->Rs = mean_vd / mean_id;
}

void estimate_inductance(foc_t *hfoc, float ts) {
    float Vc_d = 0, Vs_d = 0, Ic_d = 0, Is_d = 0;
    float Vc_q = 0, Vs_q = 0, Ic_q = 0, Is_q = 0;
    float mean_vd = 0, mean_vq = 0, mean_id = 0, mean_iq = 0;

    // Remove DC offset
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_vq += Vq_buff[i];
        mean_id += Id_buff[i];
        mean_iq += Iq_buff[i];
    }
    mean_vd /= MAX_I_SAMPLE;
    mean_vq /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;
    mean_iq /= MAX_I_SAMPLE;

    // Single-frequency DFT
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        float angle = hfoc->meas_inj_omega * i * ts;

        float vd = Vd_buff[i] - mean_vd;
        float vq = Vq_buff[i] - mean_vq;
        float id = Id_buff[i] - mean_id;
        float iq = Iq_buff[i] - mean_iq;

        Vc_d += vd * fast_cos(angle);
        Vs_d += vd * fast_sin(angle);
        Ic_d += id * fast_cos(angle);
        Is_d += id * fast_sin(angle);

        Vc_q += vq * fast_cos(angle);
        Vs_q += vq * fast_sin(angle);
        Ic_q += iq * fast_cos(angle);
        Is_q += iq * fast_sin(angle);
    }

    float norm = 2.0f / MAX_I_SAMPLE;
    Vc_d *= norm; Vs_d *= norm;
    Ic_d *= norm; Is_d *= norm;
    Vc_q *= norm; Vs_q *= norm;
    Ic_q *= norm; Is_q *= norm;

    // V & I amplitude
    float Vd_mag = sqrtf(Vc_d * Vc_d + Vs_d * Vs_d);
    float Id_mag = sqrtf(Ic_d * Ic_d + Is_d * Is_d);
    float Vq_mag = sqrtf(Vc_q * Vc_q + Vs_q * Vs_q);
    float Iq_mag = sqrtf(Ic_q * Ic_q + Is_q * Is_q);

    // (phi = arctan(Vs/Vc) - arctan(Is/Ic))
    float phi_d = atan2f(Vs_d, Vc_d) - atan2f(Is_d, Ic_d);
    float phi_q = atan2f(Vs_q, Vc_q) - atan2f(Is_q, Ic_q);

    // Impedansi & parameters
    float Zd_mag = Vd_mag / Id_mag;
    float Zq_mag = Vq_mag / Iq_mag;

    // float Rs_d = Zd_mag * cosf(phi_d);
    // float Rs_q = Zq_mag * cosf(phi_q);
    float Ld_est = (Zd_mag * sinf(phi_d)) / hfoc->meas_inj_omega;
    float Lq_est = (Zq_mag * sinf(phi_q)) / hfoc->meas_inj_omega;

    // hfoc->Rs = (Rs_d + Rs_q) * 0.5;
    hfoc->Ld = fabs(Ld_est);
    hfoc->Lq = fabs(Lq_est);
}

// Fungsi inisialisasi HFI system
void foc_sensorless_init(foc_t *hfoc, float sampling_freq) {
    float Rs = hfoc->Rs;
    float Ls = (hfoc->Ld + hfoc->Lq) / 2.0f;
    smo_init(&hfoc->smo, Rs, Ls, hfoc->pole_pairs, 1.0f / sampling_freq);

    second_order_lpf_init(&hfoc->id_lpf, HFI_ID_LPF_FC, sampling_freq);
    second_order_lpf_init(&hfoc->iq_lpf, HFI_IQ_LPF_FC, sampling_freq);

#if HFI_NEW
    hfi_init(&hfoc->hfi, HFI_AMP, HFI_FREQ, sampling_freq);
#else
    hfi_lpf_init(&hfoc->hfi_lpf, hfoc->Ld, HFI_AMP, HFI_FREQ, HFI_I_ALPHA_BETA_LPF_FC, sampling_freq);
#endif

    hfoc->pd_time = 20;
    hfoc->pd_v_pulse = 0.0f;
}

void foc_sensorless_polarity_detection(foc_t *hfoc) {
    if (hfoc->pd_state == P_DET_STOP) return;

    if (hfoc->pd_state > P_DET_START) {
        if (hfoc->pd_i_p < hfoc->id) {
            hfoc->pd_i_p = hfoc->id;
        }
        if (hfoc->pd_i_n > hfoc->id) {
            hfoc->pd_i_n = hfoc->id;
        }
    }

    hfoc->pd_count++;
    if (hfoc->pd_count >= hfoc->pd_time && hfoc->pd_state < 5) {
        hfoc->pd_count = 0;
        switch(hfoc->pd_state){
            case P_DET_START:
                hfoc->pd_v_pulse = 0.0f;
                hfoc->pd_time = PD_WAITING_TIME;
                hfoc->pd_state = P_DET_POSITIVE;
                break;
            case P_DET_POSITIVE:
                hfoc->pd_v_pulse = PD_V_PULSE;
                hfoc->pd_time = PD_PULSE_TIME;
                hfoc->pd_state = P_DET_WAITING_POSITIVE;
                break;
            case P_DET_WAITING_POSITIVE:
                hfoc->pd_v_pulse = 0.0f;
                hfoc->pd_time = PD_WAITING_TIME;
                hfoc->pd_state = P_DET_NEGATIVE;
                break;
            case P_DET_NEGATIVE:
                hfoc->pd_v_pulse = -PD_V_PULSE;
                hfoc->pd_time = PD_PULSE_TIME;
                hfoc->pd_state = P_DET_WAITING_NEGATIVE;
                break;
            case P_DET_WAITING_NEGATIVE:
                hfoc->pd_v_pulse = 0.0f;
                hfoc->pd_time = PD_PULSE_TIME;
                if (hfoc->pd_i_p < fabsf(hfoc->pd_i_n)) {
                    hfi_force_estimate_position(&hfoc->hfi, hfoc->e_rad + PI);
                }
                hfoc->pd_i_p = 0.0f;
                hfoc->pd_i_n = 0.0f;
                hfoc->pd_state = P_DET_STOP;
                break;
            default:
            break;
        }
    }
}

void foc_current_control_update(foc_t *hfoc, float Ts) {
	if (hfoc == NULL || Ts <= 0.0f || hfoc->control_mode == AUDIO_MODE) {
		hfoc->id_ctrl.integral = 0.0f;
		hfoc->id_ctrl.last_error = 0.0f;
		hfoc->iq_ctrl.integral = 0.0f;
		hfoc->iq_ctrl.last_error = 0.0f;
		return;
	}

	float id_ref = hfoc->id_ref;
	float iq_ref = hfoc->iq_ref;
    const float v_bus = hfoc->v_bus;

    float ia, ib;
    float sin_theta, cos_theta;
    float i_alpha, i_beta;
    float id, iq;

    float id_error = 0.0f, iq_error = 0.0f;
    float vd_ref = 0.0f, vq_ref = 0.0f;

    uint32_t da, db, dc;
    uint32_t pwm_res = hfoc->drv8302.pwm_resolution;
    const float pwm_to_v = v_bus / (float)pwm_res;

    // get currents
    DRV8302_get_current(&hfoc->drv8302, &ia, &ib);

    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -hfoc->max_current, hfoc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -hfoc->max_current, hfoc->max_current);

    // pre calculate sin & cos
    pre_calc_sin_cos(hfoc->e_rad, &sin_theta, &cos_theta);

    clarke_transform(ia, ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_theta, cos_theta, &id, &iq);
    
    if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI ||
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
        // LPF id & iq
        hfoc->id_filtered = second_order_lpf_update(&hfoc->id_lpf, id);
        hfoc->iq_filtered = second_order_lpf_update(&hfoc->iq_lpf, iq);

        foc_sensorless_polarity_detection(hfoc);

        _Bool smo_ret = smo_update_arctan(&hfoc->smo, hfoc->v_alpha, hfoc->v_beta, i_alpha, i_beta);

        if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
            hfi_update_estimate_position(&hfoc->hfi, iq, Ts);
        }
        else {
            hfi_lpf_update_estimate_position(&hfoc->hfi_lpf, i_alpha, i_beta, Ts);
        }
        switch (hfoc->state) {
            case MOTOR_STATE_HFI: {
                id_error = id_ref - hfoc->id_filtered;
                iq_error = iq_ref - hfoc->iq_filtered;

                float v_inj = 0.0f;
                float omega = 0.0f;
                if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
                    v_inj = hfi_get_v_inj(&hfoc->hfi);
                    hfoc->e_rad = hfi_get_estimate_position(&hfoc->hfi);
                    omega = hfi_get_estimate_omega(&hfoc->hfi);
                }
                else {
                    v_inj = hfi_lpf_get_v_inj(&hfoc->hfi_lpf);
                    hfoc->e_rad = hfi_lpf_get_estimate_position(&hfoc->hfi_lpf);
                    omega = hfi_lpf_get_estimate_omega(&hfoc->hfi_lpf);
                }

                if (hfoc->pd_state > P_DET_START && hfoc->pd_state < P_DET_STOP)
                    vd_ref = hfoc->pd_v_pulse;
                else {
                    vd_ref = v_inj;

                    hfoc->actual_rpm = (omega * 60.0 / TWO_PI) / hfoc->pole_pairs;
                    if (fabsf(hfoc->actual_rpm) > HFI_TO_SMO_THRESHOLD) {
                        hfoc->state = MOTOR_STATE_SMO;
                    }
                }
                break;
            }
            case MOTOR_STATE_SMO: {
                id_error = id_ref - id;
                iq_error = iq_ref - iq;

                if (!smo_ret) {
                    hfoc->state = MOTOR_STATE_HFI;
                    break;
                }
                hfoc->e_rad = smo_get_rotor_angle(&hfoc->smo);
                hfoc->actual_rpm = smo_get_rotor_speed(&hfoc->smo);
                if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
                    hfi_force_estimate_position(&hfoc->hfi, hfoc->e_rad);
                }
                else {
                    hfi_lpf_force_estimate_position(&hfoc->hfi_lpf, hfoc->e_rad);
                }
                break;
            }
            default:
            break;
        }
#if DEBUG_HFI
        if (!hfoc->collect_sample_flag){
            int n = hfoc->sample_index;
            param1_debug_buff[n] = hfoc->hfi.sdft_fundamental;
            param2_debug_buff[n] = hfoc->hfi.sdft_amplitude;
            // param3_debug_buff[n] = hfoc->hfi.sdft_fundamental;
            // param4_debug_buff[n] = hfoc->pd_v_pulse;
            hfoc->sample_index++;
            if (hfoc->sample_index > MAX_SAMPLE_BUFF) {
                hfoc->sample_index = 0;
                hfoc->collect_sample_flag = 1;
            }
        }
#endif
    }
    else if (hfoc->foc_mode == FOC_MODE_SENSORED) {
        id_error = id_ref - id;
        iq_error = iq_ref - iq;
        hfoc->e_rad = hfoc->e_angle_rad_comp;
        hfoc->actual_rpm = AS5047P_get_rpm(&hfoc->as5047p, Ts);
        if (hfoc->sensor_dir == REVERSE_DIR) {
            hfoc->actual_rpm = -hfoc->actual_rpm;
        }
        
        if (AS5047P_get_val_flag()) {
            AS5047P_reset_val_flag();
            AS5047P_start(&hfoc->as5047p);
        }
    }

    // set dynamic max output vd and vq
    hfoc->id_ctrl.out_max = hfoc->id_ctrl.out_max_dynamic * v_bus;
    hfoc->iq_ctrl.out_max = hfoc->iq_ctrl.out_max_dynamic * v_bus;

    vd_ref += pi_control(&hfoc->id_ctrl, id_error);
    vq_ref = pi_control(&hfoc->iq_ctrl, iq_error);

    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, v_bus, pwm_res, &da, &db, &dc);
    DRV8302_set_pwm(&hfoc->drv8302, da, db, dc);

    // copy to struct for debug
    hfoc->ia = ia;
    hfoc->ib = ib;
    hfoc->ic = -ia - ib;
    hfoc->i_alpha = i_alpha;
    hfoc->i_beta = i_beta;
    hfoc->id = id;
    hfoc->iq = iq;

    hfoc->va = da * pwm_to_v;
    hfoc->vb = db * pwm_to_v;
    hfoc->vc = dc * pwm_to_v;
    hfoc->vd = vd_ref;
    hfoc->vq = vq_ref;
}

float foc_get_mech_degree(foc_t *hfoc) {
    float angle_diff = hfoc->e_rad - hfoc->last_e_rad;
    hfoc->last_e_rad = hfoc->e_rad;

    if (angle_diff < -PI) {
        hfoc->m_angle_overflow_count++;
    } else if (angle_diff > PI) {
        hfoc->m_angle_overflow_count--;
    }

    float total_e_angle = hfoc->e_rad + (float)hfoc->m_angle_overflow_count * TWO_PI;

    if (abs(hfoc->m_angle_overflow_count) > 1000000) {
        hfoc->m_angle_overflow_count = 0;
        hfoc->last_e_rad = hfoc->e_rad;
    }

    float mechanical_angle_deg = RAD_TO_DEG(total_e_angle) / hfoc->pole_pairs;

    // Normalize to 0-360 degrees
    mechanical_angle_deg = fmodf(mechanical_angle_deg, 360.0f);
    if (mechanical_angle_deg < 0) {
    mechanical_angle_deg += 360.0f;
    }

    hfoc->actual_angle = mechanical_angle_deg;

    return hfoc->actual_angle;
}

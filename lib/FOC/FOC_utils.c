/*
 * FOC_utils.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "FOC_utils.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if DEBUG_HFI
float param1_debug_buff[MAX_SAMPLE_BUFF];
float param2_debug_buff[MAX_SAMPLE_BUFF];
float param3_debug_buff[MAX_SAMPLE_BUFF];
float param4_debug_buff[MAX_SAMPLE_BUFF];
#endif

float error_temp[ERROR_LUT_SIZE] = {0};


void foc_inverter_init(foc_t *hfoc, void (*enable_motor)(void), void (*disable_motor)(void), uint32_t (*get_pwm_res)(void)) {
    hfoc->enable_motor = enable_motor;
    hfoc->disable_motor = disable_motor;
    hfoc->get_pwm_res = get_pwm_res;
}

void foc_feedback_sensor_init(foc_t *hfoc, float (*get_mech_degre)(void), float (*get_mech_rpm)(void)) {
    hfoc->get_mech_degre = get_mech_degre;
    hfoc->get_mech_rpm = get_mech_rpm;
}

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

void foc_reset(foc_t *hfoc) {
    pid_reset(&hfoc->id_ctrl);
    pid_reset(&hfoc->iq_ctrl);
    pid_reset(&hfoc->speed_ctrl);
    pid_reset(&hfoc->pos_ctrl);
    pid_reset(&hfoc->fw_ctrl);
    if (hfoc->foc_mode != FOC_MODE_SENSORED) {
        if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
            hfi_reset(&hfoc->hfi);
        }
        else if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI) {
            hfi_lpf_reset(&hfoc->hfi_lpf);
        }

        hfoc->pd_time = 20;
        hfoc->pd_v_pulse = 0.0f;
        hfoc->pd_state = P_DET_START;
    }
    switch(hfoc->motor_mode) {
        case MOTOR_MODE_TORQUE_CONTROL: {
            foc_set_current_set_point(hfoc, 0);
            break;
        }
        case MOTOR_MODE_SPEED_CONTROL: {
            foc_set_speed_set_point(hfoc, 0);
            break;
        }
        case MOTOR_MODE_POSITION_CONTROL: {
            foc_set_position_set_point(hfoc, hfoc->actual_angle);
            break;
        }
        default:
            break;
    }
}


void foc_speed_control_update(foc_t *hfoc, float Ts) {
    hfoc->speed_control_loop_count++;
    if (hfoc->speed_control_loop_count >= SPEED_CONTROL_CYCLE) {
        float error = hfoc->rpm_ref - hfoc->actual_rpm;
        hfoc->Is_ref = pi_control(&hfoc->speed_ctrl, error);
    }
    foc_current_control_update(hfoc, Ts);
}

void foc_position_control_update(foc_t *hfoc, float Ts) {
    hfoc->position_control_loop_count++;
    if (hfoc->position_control_loop_count >= POSITION_CONTROL_CYCLE) {
        float error = hfoc->pos_ref - hfoc->actual_angle;
        hfoc->rpm_ref = pid_control(&hfoc->pos_ctrl, error);
    }
    foc_speed_control_update(hfoc, Ts);
}

void foc_sensored_calc_electric_angle(foc_t *hfoc) {
    // Check for NULL pointer and invalid parameters
    if (hfoc == NULL || hfoc->pole_pairs <= 0 || 
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI ||
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
        return;
    }

    float angle_deg = hfoc->get_mech_degre();

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
    float encoder_error = 0;
    e_rad += encoder_error;
    
    // Normalize final electric angle
    norm_angle_rad(&e_rad);

    hfoc->e_angle_rad_comp = e_rad;
}

void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth) {
    hfoc->I_ctrl_bandwidth = bandwidth;
}

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    uint32_t da, db, dc;
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus, hfoc->get_pwm_res(), &da, &db, &dc);
    motor_set_pwm(&hfoc->motor, da, db, dc);
}

void foc_sensorless_init(foc_t *hfoc, float sampling_freq) {

    float Rs = hfoc->Rs;
    float Ls = (hfoc->Ld + hfoc->Lq) / 2.0f;
    smo_init(&hfoc->smo, Rs, Ls, hfoc->pole_pairs, 1.0f / sampling_freq);

    second_order_lpf_init(&hfoc->id_lpf, HFI_ID_LPF_FC, sampling_freq);
    second_order_lpf_init(&hfoc->iq_lpf, HFI_IQ_LPF_FC, sampling_freq);

    if (hfoc->foc_mode == FOC_MODE_HYBRID) {
        hfoc->state = MOTOR_STATE_SENSORED;
        return;
    }
    else if (hfoc->foc_mode == FOC_MODE_SENSORED) return;

    if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
        hfi_init(&hfoc->hfi, HFI_AMP, HFI_FREQ, sampling_freq);
    }
    else if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI) {
        hfi_lpf_init(&hfoc->hfi_lpf, hfoc->Ld, HFI_AMP, HFI_FREQ, HFI_I_ALPHA_BETA_LPF_FC, sampling_freq);
    }

    hfoc->pd_time = 20;
    hfoc->pd_v_pulse = 0.0f;
    hfoc->pd_state = P_DET_START;
    hfoc->state = MOTOR_STATE_HFI;
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

void foc_MTPA(foc_t *hfoc, float Is, float *Id_ref, float *Iq_ref) {
    float L_diff = hfoc->Lq - hfoc->Ld;
    if (hfoc->mtpa_enable == 0 || L_diff < 0 || hfoc->Ld <= 0 || hfoc->Lq <= 0) {
        *Id_ref = 0.0f;
        *Iq_ref = Is;
        return;
    }
    float Is_square = Is * Is;
    float temp = sqrtf(hfoc->flux_linkage * hfoc->flux_linkage + 8.0f * (L_diff * L_diff) * Is_square);
    float id = (hfoc->flux_linkage - temp) / (4.0f * L_diff);
    float iq = sqrtf(Is_square - id * id);
    if (Is < 0) iq = -iq;

    *Id_ref = id;
    *Iq_ref = iq;
}

void foc_fw_set_vs_ref(foc_t *hfoc, float vs_ref) {
    hfoc->fw_vs_ref = vs_ref;
}

float foc_fw_update(foc_t *hfoc) {
    if (!hfoc->fw_enable) return 0.0f;
    float vs = sqrtf(hfoc->vd * hfoc->vd + hfoc->vq * hfoc->vq);
    float error = hfoc->fw_vs_ref - vs;
    float mv = pi_control(&hfoc->fw_ctrl, error);
    if (mv > 0.0f) mv = 0.0f;
    return mv;
}

void foc_current_limit(float *id_ref, float *iq_ref, float max_current) {
    if (*id_ref > max_current) {
        *id_ref = max_current;
    }
    else if (*id_ref < -max_current) {
        *id_ref = -max_current;
    }
    float max_iq = sqrtf(max_current * max_current - *id_ref * *id_ref);
    if (*iq_ref > max_iq) {
        *iq_ref = max_iq;
    }
    else if (*iq_ref < -max_iq) {
        *iq_ref = -max_iq;
    }
}

void foc_voltage_control_update(foc_t *hfoc) {
    if (hfoc->motor_mode != MOTOR_MODE_VOLTAGE_CONTROL) return;

    uint32_t da, db, dc;
    float sin_theta, cos_theta;
    pre_calc_sin_cos(hfoc->e_rad, &sin_theta, &cos_theta);
    inverse_park_transform(hfoc->vd, hfoc->vq, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus, hfoc->get_pwm_res(), &da, &db, &dc);
    motor_set_pwm(&hfoc->motor, da, db, dc);

    // calculate current
    motor_calculate_current(&hfoc->motor);
    motor_get_current(&hfoc->motor, &hfoc->ia, &hfoc->ib, &hfoc->ic);
    clarke_park_transform(hfoc->ia, hfoc->ib, sin_theta, cos_theta, &hfoc->id, &hfoc->iq);
}

void foc_current_control_update(foc_t *hfoc, float Ts) {
	if (hfoc == NULL || Ts <= 0.0f || hfoc->motor_mode == MOTOR_MODE_AUDIO) {
		hfoc->id_ctrl.integral = 0.0f;
		hfoc->id_ctrl.last_error = 0.0f;
		hfoc->iq_ctrl.integral = 0.0f;
		hfoc->iq_ctrl.last_error = 0.0f;
		return;
	}

    const float v_bus = hfoc->v_bus;
    const float vs_max = v_bus * ONE_BY_SQRT3;

	float id_ref = 0.0f;
	float iq_ref = 0.0f;

    float ia, ib, ic;
    float sin_theta, cos_theta;
    float i_alpha, i_beta;
    float id, iq;

    float id_error = 0.0f, iq_error = 0.0f;
    float vd_ref = 0.0f, vq_ref = 0.0f;

    uint32_t da, db, dc;
    uint32_t pwm_res = hfoc->get_pwm_res();
    const float pwm_to_v = v_bus / (float)pwm_res;

    foc_MTPA(hfoc, hfoc->Is_ref, &id_ref, &iq_ref);
    foc_fw_set_vs_ref(hfoc, 0.90f * vs_max);
    id_ref += foc_fw_update(hfoc);

    // Hard limit references
    foc_current_limit(&id_ref, &iq_ref, hfoc->max_current);

    // get currents
    motor_calculate_current(&hfoc->motor);
    motor_get_current(&hfoc->motor, &ia, &ib, &ic);

    // pre calculate sin & cos
    pre_calc_sin_cos(hfoc->e_rad, &sin_theta, &cos_theta);

    clarke_transform(ia, ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_theta, cos_theta, &id, &iq);
    
    // LPF id & iq
    hfoc->id_filtered = second_order_lpf_update(&hfoc->id_lpf, id);
    hfoc->iq_filtered = second_order_lpf_update(&hfoc->iq_lpf, iq);

    if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI ||
        hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
        foc_sensorless_polarity_detection(hfoc);
        if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
            hfi_update_estimate_position(&hfoc->hfi, iq, Ts);
        }
        else {
            hfi_lpf_update_estimate_position(&hfoc->hfi_lpf, i_alpha, i_beta, Ts);
        }

        _Bool smo_ret = smo_update_arctan(&hfoc->smo, hfoc->v_alpha, hfoc->v_beta, i_alpha, i_beta);

        switch (hfoc->state) {
            case MOTOR_STATE_HFI: {
                id_error = id_ref - hfoc->id_filtered;
                iq_error = iq_ref - hfoc->iq_filtered;

                float v_inj = 0.0f;
                if (hfoc->foc_mode == FOC_MODE_SENSORLESS_SMO_HFI_NEW) {
                    v_inj = hfi_get_v_inj(&hfoc->hfi);
                    hfoc->e_rad = hfi_get_estimate_position(&hfoc->hfi);
                    hfoc->e_omega = hfi_get_estimate_omega(&hfoc->hfi);
                }
                else {
                    v_inj = hfi_lpf_get_v_inj(&hfoc->hfi_lpf);
                    hfoc->e_rad = hfi_lpf_get_estimate_position(&hfoc->hfi_lpf);
                    hfoc->e_omega = hfi_lpf_get_estimate_omega(&hfoc->hfi_lpf);
                }

                if (hfoc->pd_state > P_DET_START && hfoc->pd_state < P_DET_STOP)
                    vd_ref = hfoc->pd_v_pulse;
                else {
                    vd_ref = v_inj;

                    hfoc->actual_rpm = (hfoc->e_omega * 60.0f / TWO_PI) / hfoc->pole_pairs;
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
                hfoc->e_omega = smo_get_omega(&hfoc->smo);
                hfoc->actual_rpm = (hfoc->e_omega * 60.0f / TWO_PI) / hfoc->pole_pairs;
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
            param3_debug_buff[n] = hfoc->e_angle_rad_comp;
            // param4_debug_buff[n] = hfoc->pd_v_pulse;
            hfoc->sample_index++;
            if (hfoc->sample_index > MAX_SAMPLE_BUFF) {
                hfoc->sample_index = 0;
                hfoc->collect_sample_flag = 1;
            }
        }
#endif
    }
    else if (hfoc->foc_mode == FOC_MODE_HYBRID) {
        id_error = id_ref - id;
        iq_error = iq_ref - iq;
        _Bool smo_ret = smo_update_arctan(&hfoc->smo, hfoc->v_alpha, hfoc->v_beta, i_alpha, i_beta);
        float rpm_encoder = hfoc->get_mech_rpm();
        switch (hfoc->state) {
            case MOTOR_STATE_SENSORED: {
                hfoc->e_rad = hfoc->e_angle_rad_comp;
                hfoc->actual_rpm = rpm_encoder;
                if (fabsf(hfoc->actual_rpm) > 500.0f) {
                    hfoc->state = MOTOR_STATE_SMO;
                }
                break;
            }
            case MOTOR_STATE_SMO: {
                if (!smo_ret) {
                    hfoc->state = MOTOR_STATE_SENSORED;
                    break;
                }
                hfoc->e_rad = smo_get_rotor_angle(&hfoc->smo);
                hfoc->e_omega = smo_get_omega(&hfoc->smo);
                hfoc->actual_rpm = (hfoc->e_omega * 60.0f / TWO_PI) / hfoc->pole_pairs;
                break;
            }
            default:
                break;
        }
    }
    else if (hfoc->foc_mode == FOC_MODE_SENSORED) {
        id_error = id_ref - id;
        iq_error = iq_ref - iq;
        hfoc->e_rad = hfoc->e_angle_rad_comp;
        hfoc->actual_rpm = hfoc->get_mech_rpm();
    }

    // voltage limit
    pid_set_out_constraint(&hfoc->id_ctrl, vs_max, -vs_max);
    vd_ref += pi_control(&hfoc->id_ctrl, id_error);

    float vq_max = sqrtf(vs_max * vs_max - vd_ref * vd_ref);
    pid_set_out_constraint(&hfoc->iq_ctrl, vq_max, -vq_max);
    vq_ref = pi_control(&hfoc->iq_ctrl, iq_error);

    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, v_bus, pwm_res, &da, &db, &dc);
    motor_set_pwm(&hfoc->motor, da, db, dc);

    // copy to struct for debug
    hfoc->ia = ia;
    hfoc->ib = ib;
    hfoc->ic = ic;
    hfoc->i_alpha = i_alpha;
    hfoc->i_beta = i_beta;
    hfoc->id = id;
    hfoc->iq = iq;
    hfoc->id_ref = id_ref;
    hfoc->iq_ref = iq_ref;

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

    // if (abs(hfoc->m_angle_overflow_count) > 1000000) {
    //     hfoc->m_angle_overflow_count = 0;
    //     hfoc->last_e_rad = hfoc->e_rad;
    // }

    float mechanical_angle_deg = RAD_TO_DEG(total_e_angle) / (float)hfoc->pole_pairs * hfoc->gear_ratio;

    // Normalize to 0-360 degrees
    // mechanical_angle_deg = fmodf(mechanical_angle_deg, 360.0f);
    // if (mechanical_angle_deg < 0) {
    //     mechanical_angle_deg += 360.0f;
    // }

    hfoc->actual_angle = mechanical_angle_deg;

    return hfoc->actual_angle;
}

void foc_update(foc_t *hfoc, float Ts) {
    foc_get_mech_degree(hfoc);
    foc_sensored_calc_electric_angle(hfoc);
    switch (hfoc->motor_mode) {
        case MOTOR_MODE_TORQUE_CONTROL: {
            foc_current_control_update(hfoc, Ts);
            break;
        }
        case MOTOR_MODE_VOLTAGE_CONTROL: {
            foc_voltage_control_update(hfoc);
            break;
        }
        case MOTOR_MODE_SPEED_CONTROL: {
            foc_speed_control_update(hfoc, Ts);
            break;
        }
        case MOTOR_MODE_POSITION_CONTROL: {
            foc_position_control_update(hfoc, Ts);
            break;
        }
        default:
            break;
    }
}

void foc_set_mode(foc_t *hfoc, foc_mode_t mode) {
    hfoc->foc_mode = mode;
}

foc_mode_t foc_get_mode(foc_t *hfoc) {
    return hfoc->foc_mode;
}

void foc_set_motor_mode(foc_t *hfoc, motor_mode_t mode) {
    if (hfoc->motor_mode == mode) return;
    hfoc->motor_mode = mode;
    foc_reset(hfoc);
}

motor_mode_t foc_get_motor_mode(foc_t *hfoc) {
    return hfoc->motor_mode;
}

void foc_set_motor_pole_pairs(foc_t *hfoc, uint8_t pole_pairs) {
    hfoc->pole_pairs = pole_pairs;
}

uint8_t foc_get_motor_pole_pairs(foc_t *hfoc) {
    return hfoc->pole_pairs;
}

void foc_set_motor_kv(foc_t *hfoc, float kv) {
    hfoc->kv = kv;
}

float foc_get_motor_kv(foc_t *hfoc) {
    return hfoc->kv;
}

void foc_set_motor_Rs(foc_t *hfoc, float Rs) {
    hfoc->Rs = Rs;
}

float foc_get_motor_Rs(foc_t *hfoc) {
    return hfoc->Rs;
}

void foc_set_motor_Ld(foc_t *hfoc, float Ld) {
    hfoc->Ld = Ld;
}

float foc_get_motor_Ld(foc_t *hfoc) {
    return hfoc->Ld;
}

void foc_set_motor_Lq(foc_t *hfoc, float Lq) {
    hfoc->Lq = Lq;
}

float foc_get_motor_Lq(foc_t *hfoc) {
    return hfoc->Lq;
}

void foc_set_motor_flux_linkage(foc_t *hfoc, float flux_linkage) {
    hfoc->flux_linkage = flux_linkage;
}

float foc_get_motor_flux_linkage(foc_t *hfoc) {
    return hfoc->flux_linkage;
}

void foc_disable(foc_t *hfoc) {
    hfoc->disable_motor();
    foc_reset(hfoc);
    hfoc->motor_mode = MOTOR_MODE_DISABLE;
}

void foc_enable(foc_t *hfoc) {
    hfoc->enable_motor();
}

void foc_set_fw_enable(foc_t *hfoc, _Bool enable) {
    hfoc->fw_enable = enable;
    if (!enable) {
        pid_reset(&hfoc->fw_ctrl);
    }
}

_Bool foc_get_fw_enable(foc_t *hfoc) {
    return hfoc->fw_enable;
}

void foc_set_mtpa_enable(foc_t *hfoc, _Bool enable) {
    hfoc->mtpa_enable = enable;
}

_Bool foc_get_mtpa_enable(foc_t *hfoc) {
    return hfoc->mtpa_enable;
}

void foc_set_open_loop_voltage(foc_t *hfoc, float vd, float vq, float e_rad) {
    if (hfoc->motor_mode != MOTOR_MODE_VOLTAGE_CONTROL) return;
    hfoc->vd = vd;
    hfoc->vq = vq;
    hfoc->e_rad = e_rad;
}

void foc_set_current_set_point(foc_t *hfoc, float Is) {
    if (hfoc->motor_mode != MOTOR_MODE_TORQUE_CONTROL) return;
    hfoc->Is_ref = Is;
}

void foc_set_speed_set_point(foc_t *hfoc, float rpm) {
    if (hfoc->motor_mode != MOTOR_MODE_SPEED_CONTROL) return;
    hfoc->rpm_ref = rpm;
}

void foc_set_position_set_point(foc_t *hfoc, float pos_deg) {
    if (hfoc->motor_mode != MOTOR_MODE_POSITION_CONTROL) return;
    hfoc->pos_ref = pos_deg;
}

void foc_get_idiq(foc_t *hfoc, float *id, float *iq) {
    *id = hfoc->id;
    *iq = hfoc->iq;
}



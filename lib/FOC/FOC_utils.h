/*
 * FOC_utils.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_UTILS_H_
#define FOC_INC_FOC_UTILS_H_

#include <stdint.h>
#include "FOC_config.h"
#include "FOC_math.h"
#include "pid_utils.h"
#include "sliding_mode_observer.h"
#include "pll.h"
#include "lpf.h"
#include "hfi_sdft.h"
#include "hfi_lpf.h"
#include "motor.h"

#define ERROR_LUT_SIZE (1024)

#define MAG_CAL_RES (1024*2)
#define MAG_CAL_STEP ((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

/* extern variable */
extern _Bool foc_ready;

#if DEBUG_HFI
extern float param1_debug_buff[MAX_SAMPLE_BUFF];
extern float param2_debug_buff[MAX_SAMPLE_BUFF];
extern float param3_debug_buff[MAX_SAMPLE_BUFF];
extern float param4_debug_buff[MAX_SAMPLE_BUFF];
#endif

typedef enum {
	MOTOR_MODE_TORQUE_CONTROL,
	MOTOR_MODE_SPEED_CONTROL,
	MOTOR_MODE_POSITION_CONTROL,
	MOTOR_MODE_CALIBRATION_ENCODER,
	MOTOR_MODE_AUDIO,
	MOTOR_MODE_VOLTAGE_CONTROL,
	MOTOR_MODE_DISABLE,
}motor_mode_t;

typedef enum {
	FOC_MODE_SENSORED,
	FOC_MODE_SENSORLESS_SMO_HFI,
	FOC_MODE_SENSORLESS_SMO_HFI_NEW,
	FOC_MODE_HYBRID
}foc_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;

// state machine for HFI
typedef enum {
	MOTOR_STATE_HFI,
	MOTOR_STATE_SMO,
	MOTOR_STATE_SMO_TO_HFI,
	MOTOR_STATE_SENSORED,
}motor_state_t;

// state machine for polarity detection
typedef enum {
	P_DET_START,
	P_DET_POSITIVE,
	P_DET_WAITING_POSITIVE,
	P_DET_NEGATIVE,
	P_DET_WAITING_NEGATIVE,
	P_DET_STOP
}p_det_state_t;

typedef struct {
	motor_t motor;
	foc_mode_t foc_mode;
	motor_mode_t motor_mode;

	uint8_t pole_pairs;
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float max_current;
	float flux_linkage;

	float m_angle_rad; // mechanical angle
	float e_angle_rad; // electrical angle
	float e_angle_rad_comp; // electrical angle
	float m_angle_offset;
	float e_rad;
	float last_e_rad;
	float e_omega;

	float vd, vq;
	float id, iq;
	float id_filtered, iq_filtered;
	float v_alpha, v_beta;
	float i_alpha, i_beta;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float i_bus;

	float rpm_temp;
	float actual_rpm;
	float actual_angle;
	int32_t m_angle_overflow_count;

	float I_ctrl_bandwidth;
	float Is_ref;
	float id_ref, iq_ref;
	float rpm_ref;
	float pos_ref;

    uint8_t loop_count;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	//field weakening
	PID_Controller_t fw_ctrl;
	float fw_vs_ref;
	float fw_vs;
	_Bool fw_enable;

	// MTPA
	_Bool mtpa_enable;

	float gear_ratio;
	dir_mode_t sensor_dir;

	motor_state_t state;

	smo_t smo;
	hfi_t hfi;
	hfi_lpf_t hfi_lpf;

	SecondOrderLPF id_lpf;
	SecondOrderLPF iq_lpf;

	//polarity detection
	p_det_state_t pd_state;
	float pd_v_pulse;
	float pd_i_p;
	float pd_i_n;
	uint16_t pd_time;
	uint16_t pd_count;

	//debug
	int sample_index;
	_Bool collect_sample_flag;

	void (*enable_motor)(void);
	void (*disable_motor)(void);
	uint32_t (*get_pwm_res)(void);
	float (*get_mech_degre)(void);
	float (*get_mech_rpm)(float);
}foc_t;

void foc_inverter_init(foc_t *hfoc, void (*enable_motor)(void), void (*disable_motor)(void), uint32_t (*get_pwm_res)(void));
void foc_feedback_sensor_init(foc_t *hfoc, float (*get_mech_degre)(void), float (*get_mech_rpm)(float));
void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);
void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);
void foc_gear_reducer_init(foc_t *hfoc, float ratio);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_speed_control_update(foc_t *hfoc, float rpm_reference);
void foc_position_control_update(foc_t *hfoc, float deg_reference);
float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm);
float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg);
void foc_sensored_calc_electric_angle(foc_t *hfoc);
void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);

void foc_sensorless_init(foc_t *hfoc, float sampling_freq);
void foc_sensorless_polarity_detection(foc_t *hfoc);
void foc_current_control_update(foc_t *hfoc, float Ts);
float foc_get_mech_degree(foc_t *hfoc);

void foc_update(foc_t *hfoc, float Ts);

void foc_set_mode(foc_t *hfoc, foc_mode_t mode);
foc_mode_t foc_get_mode(foc_t *hfoc);
void foc_set_motor_mode(foc_t *hfoc, motor_mode_t mode);
motor_mode_t foc_get_motor_mode(foc_t *hfoc);
void foc_set_motor_pole_pairs(foc_t *hfoc, uint8_t pole_pairs);
uint8_t foc_get_motor_pole_pairs(foc_t *hfoc);
void foc_set_motor_kv(foc_t *hfoc, float kv);
float foc_get_motor_kv(foc_t *hfoc);
void foc_set_motor_Rs(foc_t *hfoc, float Rs);
float foc_get_motor_Rs(foc_t *hfoc);
void foc_set_motor_Ld(foc_t *hfoc, float Ld);
float foc_get_motor_Ld(foc_t *hfoc);
void foc_set_motor_Lq(foc_t *hfoc, float Lq);
float foc_get_motor_Lq(foc_t *hfoc);
void foc_set_motor_flux_linkage(foc_t *hfoc, float flux_linkage);
float foc_get_motor_flux_linkage(foc_t *hfoc);

void foc_disable(foc_t *hfoc);
void foc_enable(foc_t *hfoc);
void foc_set_fw_enable(foc_t *hfoc, _Bool enable);
_Bool foc_get_fw_enable(foc_t *hfoc);

void foc_set_mtpa_enable(foc_t *hfoc, _Bool enable);
_Bool foc_get_mtpa_enable(foc_t *hfoc);

void foc_set_open_loop_voltage(foc_t *hfoc, float vd, float vq, float e_rad);
void foc_set_current_set_point(foc_t *hfoc, float Is);
void foc_set_speed_set_point(foc_t *hfoc, float rpm);
void foc_set_position_set_point(foc_t *hfoc, float pos_deg);
void foc_get_idiq(foc_t *hfoc, float *id, float *iq);

#endif /* FOC_INC_FOC_UTILS_H_ */

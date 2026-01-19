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
#if HFI_NEW
#include "hfi_sdft.h"
#else
#include "hfi_lpf.h"
#endif

#define ERROR_LUT_SIZE (1024)

#define MAG_CAL_RES (1024*2)
#define MAG_CAL_STEP ((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define CAL_ITERATION 100

#define VD_CAL 0.6f
#define VQ_CAL 0.0f

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

/* extern variable */
extern _Bool foc_ready;
extern float Vd_buff[MAX_I_SAMPLE];
extern float Vq_buff[MAX_I_SAMPLE];
extern float Id_buff[MAX_I_SAMPLE];
extern float Iq_buff[MAX_I_SAMPLE];

#if DEBUG_HFI
extern float i_alpha_buff[MAX_SAMPLE_BUFF];
extern float i_beta_buff[MAX_SAMPLE_BUFF];
extern float i_alpha_l_buff[MAX_SAMPLE_BUFF];
extern float i_beta_l_buff[MAX_SAMPLE_BUFF];
#endif

typedef enum {
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE,
	AUDIO_MODE,
	TEST_MODE,
	POWER_UP_MODE,
}motor_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;

typedef enum {
  RS, LD, LQ
}inject_taregt_t;

// state machine for HFI
typedef enum {
	MOTOR_STATE_HFI,
	MOTOR_STATE_SMO
}motor_state_t;

typedef enum {
	STARTUP_IDLE, STARTUP_ALIGN, STARTUP_OPEN_LOOP_RAMP
}startup_state_t;

typedef struct {
	uint8_t pole_pairs;
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float max_current;

	float meas_inj_freq;
	float meas_inj_amp;
	float meas_inj_omega;
	inject_taregt_t meas_inj_target;
	int meas_inj_n;
	_Bool meas_inj_start_flag;

	float m_angle_rad; // mechanical angle
	float e_angle_rad; // electrical angle
	float e_angle_rad_comp; // electrical angle
	float m_angle_offset;
	float e_rad;
	float e_angle_rad_smo;

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

	float I_ctrl_bandwidth;
	float id_ref, iq_ref;
	float rpm_ref;

    uint8_t loop_count;

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;
	uint32_t pwm_res;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	motor_mode_t control_mode;

	float gear_ratio;
	dir_mode_t sensor_dir;
	float *angle_filtered;

	float startup_timer;
	float startup_rad;
	float startup_vd;
	startup_state_t startup_state;
	motor_state_t state;

	smo_t smo;
#if HFI_NEW
	hfi_t hfi;
#else
	hfi_lpf_t hfi_lpf;
#endif

	// test HFI
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
	SecondOrderLPF id_lpf;
	SecondOrderLPF iq_lpf;

	//debug
	int sample_index;
	_Bool collect_sample_flag;
}foc_t;

void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c,
		uint32_t pwm_res);
void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);
void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);
void foc_gear_reducer_init(foc_t *hfoc, float ratio);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_current_control_update(foc_t *hfoc);
void foc_speed_control_update(foc_t *hfoc, float rpm_reference);
void foc_position_control_update(foc_t *hfoc, float deg_reference);
float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm);
float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg);
void foc_calc_electric_angle(foc_t *hfoc, float m_rad);
void foc_cal_encoder_misalignment(foc_t *hfoc);
void foc_cal_encoder(foc_t *hfoc);
void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);
void meas_inj_dq_process(foc_t *hfoc, float ts);
void estimate_resistance(foc_t *hfoc);
void estimate_inductance(foc_t *hfoc, float ts);
int foc_startup_rotor(foc_t *hfoc, smo_t *hsmo, _Bool dir, float dt);

void foc_sensorless_init(foc_t *hfoc, float sampling_freq);
void foc_sensorless_current_control_update(foc_t *hfoc, float Ts);

#endif /* FOC_INC_FOC_UTILS_H_ */

#ifndef STORAGE_H_
#define STORAGE_H_

#include "FOC_utils.h"


#define SOF_FLAG 0xAA
#define EOF_FLAG 0x55

typedef struct {
	foc_mode_t foc_mode;
	motor_mode_t motor_mode;
	uint8_t pole_pairs;
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float flux_linkage;
    uint32_t pwm_freq;
	float gear_ratio;
}storage_motor_config_t;

typedef struct {
    float kp;
    float ki;
    float e_deadband;
}storage_current_control_t;

typedef struct {
    float kp;
    float ki;
    float out_max;
    float e_deadband;
}storage_speed_control_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_max;
    float e_deadband;
    float derivative_fc_lpf;
}storage_position_control_t;

typedef struct {
    float kp;
    float ki;
    float out_min;
    _Bool enable;
}storage_field_weakening_t;

typedef struct {
    _Bool enable;
}storage_mtpa_t;

typedef struct {
    float scale; // adc to A
    float ia_offset;
    float ib_offset;
    float ic_offset;
}storage_current_sens_t;

typedef struct {
    float scale; // raw value to deg
    float elec_phase_offset_deg;
    float mech_offset_deg;
    float error_comp_deg[ERROR_LUT_SIZE];
}storage_encoder_config_t;

typedef struct {
    uint8_t valid_SOF;

    storage_motor_config_t motor_config;
    storage_current_control_t id_control;
    storage_current_control_t iq_control;
    storage_speed_control_t speed_control;
    storage_position_control_t position_control;
    storage_field_weakening_t field_weakening;
    storage_mtpa_t mtpa;
    storage_current_sens_t current_sens;
    storage_encoder_config_t encoder_config;

    uint8_t valid_EOF;
}memory_t;

typedef struct {
    memory_t memory;
    int (*write_storage)(void*, uint32_t);
    int (*read_storage)(void*, uint32_t);
}storage_t;

void storage_init(storage_t *s, int (*write_storage)(void*, uint32_t), int (*read_storage)(void*, uint32_t));
int storage_save_config(storage_t *s);
int storage_read_config(storage_t *s);
void storage_default_config(storage_t *s);
void storage_copy_to_local(storage_t *s, foc_t *hfoc);
void storage_copy_from_local(storage_t *s, foc_t *hfoc);


#endif
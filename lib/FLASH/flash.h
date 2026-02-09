/*
 * flash.h
 *
 *  Created on: August 8, 2025
 *      Author: munir
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx_hal.h"
#include "FOC_utils.h"

#define FLASH_SECTOR_ADDR  ((uint32_t)0x080E0000)
#define FLASH_SECTOR_NUM   FLASH_SECTOR_11

#define SOF_FLAG 0xAA
#define EOF_FLAG 0x55

typedef struct {
    uint8_t valid_SOF;

    float id_kp;
    float id_ki;
    float id_out_max;
    float id_e_deadband;
    float iq_kp;
    float iq_ki;
    float iq_out_max;
    float iq_e_deadband;
	float I_ctrl_bandwidth;
    
    float speed_kp;
    float speed_ki;
    float speed_out_max;
    float speed_e_deadband;
    
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_out_max;
    float pos_e_deadband;
    
    float voffset_a;
    float voffset_b;

    float encd_offset;
    float encd_error_comp[ERROR_LUT_SIZE];

    uint32_t freq;
    dir_mode_t dir;
	float gear_ratio;

    float Rs;
    float Ld;
    float Lq;

    uint8_t valid_EOF;
}motor_config_t;

HAL_StatusTypeDef flash_save_config(motor_config_t *data);
void flash_read_config(motor_config_t *data);
void flash_default_config(motor_config_t *data);
void flash_auto_tuning_torque_control(motor_config_t *data);

void copy_to_local(motor_config_t *data, foc_t *hfoc);
void copy_from_local(motor_config_t *data, foc_t *hfoc);

#endif
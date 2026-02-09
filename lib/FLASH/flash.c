#include "flash.h"
#include <string.h>

HAL_StatusTypeDef flash_save_config(motor_config_t *data) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    data->valid_SOF = SOF_FLAG;
    data->valid_EOF = EOF_FLAG;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_SECTOR_NUM;
    EraseInitStruct.NbSectors    = 1;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    uint32_t address = FLASH_SECTOR_ADDR;
    uint8_t *src = (uint8_t *)data;

    for (uint32_t i = 0; i < sizeof(motor_config_t); i += 4) {
        uint32_t word = *(uint32_t*)(src + i);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 4;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

void flash_read_config(motor_config_t *data) {
    memcpy(data, (void*)FLASH_SECTOR_ADDR, sizeof(motor_config_t));

    // Validasi SOF dan EOF
    if (data->valid_SOF != SOF_FLAG || data->valid_EOF != EOF_FLAG) {
        // set default
        flash_default_config(data);
    }
}

void flash_default_config(motor_config_t *data) {
    data->id_kp = 0.01f;
    data->id_ki = 12.0f;
    data->id_out_max = 0.8f;
    data->id_e_deadband = 0.0001f;

    data->iq_kp = 0.01f;
    data->iq_ki = 12.0f;
    data->iq_out_max = 0.8f;
    data->iq_e_deadband = 0.0001f;

    data->I_ctrl_bandwidth = 50.0f;

    data->speed_kp = 0.01f;
    data->speed_ki = 0.1f;
    data->speed_out_max = 10.0f;
    data->speed_e_deadband = 0.01f;

    data->pos_kp = 4.1f;
    data->pos_ki = 0.0f;
    data->pos_kd = 0.21f;
    data->pos_out_max = 3.0f;
    data->pos_e_deadband = 0.1f;

    data->voffset_a = 1.65f;
    data->voffset_b = 1.65f;

    data->encd_offset = 0.0f;
    memset(data->encd_error_comp, 0, sizeof(data->encd_error_comp));

    data->freq = 10000;
    data->dir = NORMAL_DIR;
    data->gear_ratio = 1.0f;

    data->Rs = 0.26f;
    data->Ld = 0.000160f;
    data->Lq = 0.000160f;
}

void copy_to_local(motor_config_t *data, foc_t *hfoc) {
    hfoc->id_ctrl.kp = data->id_kp;
    hfoc->id_ctrl.ki = data->id_ki;
    hfoc->id_ctrl.out_max_dynamic = data->id_out_max;
    hfoc->id_ctrl.e_deadband = data->id_e_deadband;
    
    hfoc->iq_ctrl.kp = data->iq_kp;
    hfoc->iq_ctrl.ki = data->iq_ki;
    hfoc->iq_ctrl.out_max_dynamic = data->iq_out_max;
    hfoc->iq_ctrl.e_deadband = data->iq_e_deadband;

    hfoc->I_ctrl_bandwidth = data->I_ctrl_bandwidth;
    
    hfoc->speed_ctrl.kp = data->speed_kp;
    hfoc->speed_ctrl.ki = data->speed_ki;
    hfoc->speed_ctrl.out_max = data->speed_out_max;
    hfoc->speed_ctrl.e_deadband = data->speed_e_deadband;
    
    hfoc->pos_ctrl.kp = data->pos_kp;
    hfoc->pos_ctrl.ki = data->pos_ki;
    hfoc->pos_ctrl.kd = data->pos_kd;
    hfoc->pos_ctrl.out_max = data->pos_out_max;
    hfoc->pos_ctrl.e_deadband = data->pos_e_deadband;

    hfoc->drv8302.v_offset_a = data->voffset_a;
    hfoc->drv8302.v_offset_b = data->voffset_b;

    hfoc->sensor_dir = data->dir;
    hfoc->gear_ratio = data->gear_ratio;

    hfoc->Rs = data->Rs;
    hfoc->Ld = data->Ld;
    hfoc->Lq = data->Lq;
}

void copy_from_local(motor_config_t *data, foc_t *hfoc) {
    data->id_kp = hfoc->id_ctrl.kp;
    data->id_ki = hfoc->id_ctrl.ki;
    data->id_out_max = hfoc->id_ctrl.out_max_dynamic;
    data->id_e_deadband = hfoc->id_ctrl.e_deadband;

    data->iq_kp = hfoc->iq_ctrl.kp;
    data->iq_ki = hfoc->iq_ctrl.ki;
    data->iq_out_max = hfoc->iq_ctrl.out_max_dynamic;
    data->iq_e_deadband = hfoc->iq_ctrl.e_deadband;

    data->I_ctrl_bandwidth = hfoc->I_ctrl_bandwidth;

    data->speed_kp = hfoc->speed_ctrl.kp;
    data->speed_ki = hfoc->speed_ctrl.ki;
    data->speed_out_max = hfoc->speed_ctrl.out_max;
    data->speed_e_deadband = hfoc->speed_ctrl.e_deadband;

    data->pos_kp = hfoc->pos_ctrl.kp;
    data->pos_ki = hfoc->pos_ctrl.ki;
    data->pos_kd = hfoc->pos_ctrl.kd;
    data->pos_out_max = hfoc->pos_ctrl.out_max;
    data->pos_e_deadband = hfoc->pos_ctrl.e_deadband;

    data->dir = hfoc->sensor_dir;
    data->gear_ratio = hfoc->gear_ratio;

    data->Rs = hfoc->Rs;
    data->Ld = hfoc->Ld;
    data->Lq = hfoc->Lq;
}


void flash_auto_tuning_torque_control(motor_config_t *data) {
  if (data->Rs <= 0.0f || data->Rs > 3.0f ||
      data->Ld <= 0.0f || data->Ld > 3.0f ||
      data->Lq <= 0.0f || data->Lq > 3.0f ||
      data->I_ctrl_bandwidth <= 0.0f)
      return;

  float omega = TWO_PI * data->I_ctrl_bandwidth;

  data->id_kp = data->Ld * omega;
  data->id_ki = data->Rs * omega;
  data->iq_kp = data->Lq * omega;
  data->iq_ki = data->Rs * omega;
}
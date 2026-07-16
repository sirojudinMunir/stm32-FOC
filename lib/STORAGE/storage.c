#include "storage.h"
#include <string.h>


void storage_init(storage_t *s, int (*write_storage)(void*, uint32_t), int (*read_storage)(void*, uint32_t)) {
    s->write_storage = write_storage;
    s->read_storage = read_storage;
}

int storage_save_config(storage_t *s) {

    s->memory.valid_SOF = SOF_FLAG;
    s->memory.valid_EOF = EOF_FLAG;

    if (s->write_storage(&s->memory, sizeof(s->memory)) != 0) return -1;
    return 0;
}

int storage_read_config(storage_t *s) {
    if (s->read_storage(&s->memory, sizeof(s->memory)) != 0) return -1;

    // validate SOF and EOF
    if (s->memory.valid_SOF != SOF_FLAG || s->memory.valid_EOF != EOF_FLAG) {
        // set default
        storage_default_config(s);
    }
    return 0;
}

void storage_default_config(storage_t *s) {
    s->memory.motor_config.foc_mode = FOC_MODE_SENSORED;
    s->memory.motor_config.motor_mode = MOTOR_MODE_DISABLE;
    s->memory.motor_config.pole_pairs = 7;
    s->memory.motor_config.kv = 270;
    s->memory.motor_config.Rs = 0;
    s->memory.motor_config.Ld = 0;
    s->memory.motor_config.Lq = 0;
    s->memory.motor_config.flux_linkage = 0;
    s->memory.motor_config.pwm_freq = 20000;
    s->memory.motor_config.gear_ratio = 1.0f;

    s->memory.id_control.kp = 0.02f;
    s->memory.id_control.ki = 12.0f;
    s->memory.id_control.e_deadband = 0.0f;

    s->memory.iq_control.kp = 0.02f;
    s->memory.iq_control.ki = 12.0f;
    s->memory.iq_control.e_deadband = 0.0f;

    s->memory.speed_control.kp = 0.01f;
    s->memory.speed_control.ki = 0.1f;
    s->memory.speed_control.out_max = 10.0f;
    s->memory.speed_control.e_deadband = 0.01f;

    s->memory.position_control.kp = 4.1f;
    s->memory.position_control.ki = 0.0f;
    s->memory.position_control.kd = 0.21f;
    s->memory.position_control.out_max = 3.0f;
    s->memory.position_control.e_deadband = 0.01f;
    s->memory.position_control.derivative_fc_lpf = 100.0f;
    
    s->memory.field_weakening.kp = 0.1f;
    s->memory.field_weakening.ki = 1.0f;
    s->memory.field_weakening.out_min = -3.5;
    s->memory.field_weakening.enable = 0;
    
    s->memory.mtpa.enable = 0;

    s->memory.current_sens.scale = (3.3f / 4095.0f) / (0.01f * 20.0f);
    s->memory.current_sens.ia_offset = 0.0f;
    s->memory.current_sens.ib_offset = 0.0f;
    s->memory.current_sens.ic_offset = 0.0f;

    s->memory.encoder_config.scale = (360.0f / 16383.0f);
    s->memory.encoder_config.elec_phase_offset_deg = 0;
    s->memory.encoder_config.mech_offset_deg = 0;
    memset(s->memory.encoder_config.error_comp_deg, 0, sizeof(s->memory.encoder_config.error_comp_deg));
}

void storage_copy_to_local(storage_t *s, foc_t *hfoc) {
    hfoc->foc_mode = s->memory.motor_config.foc_mode;
    hfoc->motor_mode = s->memory.motor_config.motor_mode;
    hfoc->pole_pairs = s->memory.motor_config.pole_pairs;
    hfoc->kv = s->memory.motor_config.kv;
    hfoc->Rs = s->memory.motor_config.Rs;
    hfoc->Ld = s->memory.motor_config.Ld;
    hfoc->Lq = s->memory.motor_config.Lq;
    hfoc->flux_linkage = s->memory.motor_config.flux_linkage;
    hfoc->gear_ratio = s->memory.motor_config.gear_ratio;

    hfoc->id_ctrl.kp = s->memory.id_control.kp;
    hfoc->id_ctrl.ki = s->memory.id_control.ki;
    hfoc->id_ctrl.e_deadband = s->memory.id_control.e_deadband;
    
    hfoc->iq_ctrl.kp = s->memory.iq_control.kp;
    hfoc->iq_ctrl.ki = s->memory.iq_control.ki;
    hfoc->iq_ctrl.e_deadband = s->memory.iq_control.e_deadband;
    
    hfoc->speed_ctrl.kp = s->memory.speed_control.kp;
    hfoc->speed_ctrl.ki = s->memory.speed_control.ki;
    pid_set_out_constraint(&hfoc->speed_ctrl, s->memory.speed_control.out_max, -s->memory.speed_control.out_max);
    hfoc->speed_ctrl.e_deadband = s->memory.speed_control.e_deadband;
    
    hfoc->pos_ctrl.kp = s->memory.position_control.kp;
    hfoc->pos_ctrl.ki = s->memory.position_control.ki;
    hfoc->pos_ctrl.kd = s->memory.position_control.kd;
    pid_set_out_constraint(&hfoc->pos_ctrl, s->memory.position_control.out_max, -s->memory.position_control.out_max);
    hfoc->pos_ctrl.e_deadband = s->memory.position_control.e_deadband;
    pid_set_d_filter_fc(&hfoc->pos_ctrl, s->memory.position_control.derivative_fc_lpf);

    hfoc->fw_ctrl.kp = s->memory.field_weakening.kp;
    hfoc->fw_ctrl.ki = s->memory.field_weakening.ki;
    pid_set_out_constraint(&hfoc->fw_ctrl, 0.0f, s->memory.field_weakening.out_min);
    // hfoc->fw_ctrl.out_min = s->memory.field_weakening.out_min;
    hfoc->fw_enable = s->memory.field_weakening.enable;
    
    hfoc->mtpa_enable = s->memory.mtpa.enable;

    hfoc->motor.ia_offset = s->memory.current_sens.ia_offset;
    hfoc->motor.ib_offset = s->memory.current_sens.ib_offset;
    hfoc->motor.ic_offset = s->memory.current_sens.ic_offset;
}

void storage_copy_from_local(storage_t *s, foc_t *hfoc) {
    // Motor configuration
    s->memory.motor_config.foc_mode = hfoc->foc_mode;
    s->memory.motor_config.motor_mode = hfoc->motor_mode;
    s->memory.motor_config.pole_pairs = hfoc->pole_pairs;
    s->memory.motor_config.kv = hfoc->kv;
    s->memory.motor_config.Rs = hfoc->Rs;
    s->memory.motor_config.Ld = hfoc->Ld;
    s->memory.motor_config.Lq = hfoc->Lq;
    s->memory.motor_config.flux_linkage = hfoc->flux_linkage;
    s->memory.motor_config.gear_ratio = hfoc->gear_ratio;

    // ID Control
    s->memory.id_control.kp = hfoc->id_ctrl.kp;
    s->memory.id_control.ki = hfoc->id_ctrl.ki;
    s->memory.id_control.e_deadband = hfoc->id_ctrl.e_deadband;
    
    // IQ Control
    s->memory.iq_control.kp = hfoc->iq_ctrl.kp;
    s->memory.iq_control.ki = hfoc->iq_ctrl.ki;
    s->memory.iq_control.e_deadband = hfoc->iq_ctrl.e_deadband;
    
    // Speed Control
    s->memory.speed_control.kp = hfoc->speed_ctrl.kp;
    s->memory.speed_control.ki = hfoc->speed_ctrl.ki;
    s->memory.speed_control.out_max = hfoc->speed_ctrl.out_max;
    s->memory.speed_control.e_deadband = hfoc->speed_ctrl.e_deadband;
    
    // Position Control
    s->memory.position_control.kp = hfoc->pos_ctrl.kp;
    s->memory.position_control.ki = hfoc->pos_ctrl.ki;
    s->memory.position_control.kd = hfoc->pos_ctrl.kd;
    s->memory.position_control.out_max = hfoc->pos_ctrl.out_max;
    s->memory.position_control.e_deadband = hfoc->pos_ctrl.e_deadband;
    s->memory.position_control.derivative_fc_lpf = hfoc->pos_ctrl.d_fc_lpf;

    // Field Weakening
    s->memory.field_weakening.kp = hfoc->fw_ctrl.kp;
    s->memory.field_weakening.ki = hfoc->fw_ctrl.ki;
    s->memory.field_weakening.out_min = hfoc->fw_ctrl.out_min;
    s->memory.field_weakening.enable = hfoc->fw_enable;
    
    // MTPA
    s->memory.mtpa.enable = hfoc->mtpa_enable;

    // Current Sensor Offsets
    s->memory.current_sens.ia_offset = hfoc->motor.ia_offset;
    s->memory.current_sens.ib_offset = hfoc->motor.ib_offset;
    s->memory.current_sens.ic_offset = hfoc->motor.ic_offset;
}

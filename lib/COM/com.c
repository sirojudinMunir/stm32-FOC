#include "com.h"

/****************************************************************************** */

int8_t com_send(com_t *com, void *value, uint16_t size) {
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

int8_t com_send_uint8(com_t *com, uint8_t *value, uint16_t len) {
  uint16_t size = len * sizeof(uint8_t);
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

int8_t com_send_int8(com_t *com, int8_t *value, uint16_t len) {
  uint16_t size = len * sizeof(int8_t);
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

int8_t com_send_uint16(com_t *com, uint16_t *value, uint16_t len) {
  uint16_t size = len * sizeof(uint16_t);
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

int8_t com_send_int16(com_t *com, int16_t *value, uint16_t len) {
  uint16_t size = len * sizeof(int16_t);
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

int8_t com_send_float32(com_t *com, float *value, uint16_t len) {
  uint16_t size = len * sizeof(float);
  uint8_t tx_buff[size];
  memcpy(tx_buff, value, size);
  if (com->send_data(tx_buff, size) != 0) return -1;
  return 0;
}

/****************************************************************************** */

uint8_t com_recv_uint8(uint8_t *data_rx) {
    uint8_t val;
    memcpy(&val, data_rx, sizeof(uint8_t));
    return val;
}

int8_t com_recv_int8(uint8_t *data_rx) {
    int8_t val;
    memcpy(&val, data_rx, sizeof(int8_t));
    return val;
}

uint16_t com_recv_uint16(uint8_t *data_rx) {
    uint16_t val;
    memcpy(&val, data_rx, sizeof(uint16_t));
    return val;
}

int16_t com_recv_int16(uint8_t *data_rx) {
    int16_t val;
    memcpy(&val, data_rx, sizeof(int16_t));
    return val;
}

float com_recv_float32(uint8_t *data_rx) {
    float val;
    memcpy(&val, data_rx, sizeof(float));
    return val;
}

/****************************************************************************** */

int8_t com_receive_value(com_t *com, void *var, uint32_t var_len) {
    int8_t ret_val = 0;
    if (com->data_rx_len == sizeof(uint8_t) + var_len) {
        memcpy(var, &com->data_rx[1], var_len);
    }
    else {
        ret_val = -1;
    }
    // com_send_int8(com, &ret_val, 1);
    return ret_val;
}


/****************************************************************************** */

int8_t com_set_default_config(com_t *com) {
    int8_t ret_val = 0;
    storage_default_config(com->pstorage);
    storage_copy_to_local(com->pstorage, com->pfoc);
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_save_config(com_t *com) {
    int8_t ret_val = 0;
    storage_copy_from_local(com->pstorage, com->pfoc);
    if (storage_save_config(com->pstorage) != 0) {
        ret_val = -1;
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

/****************************************************************************** */

int8_t com_set_foc_mode(com_t *com) {
    uint8_t mode;
    int8_t ret_val = com_receive_value(com, &mode, sizeof(uint8_t));
    if (ret_val == 0) {
        foc_set_mode(com->pfoc, (foc_mode_t)mode);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_mode(com_t *com) {
    uint8_t mode = (uint8_t)foc_get_mode(com->pfoc);
    com_send_uint8(com, &mode, 1);
    return 0;
}

int8_t com_set_foc_motor_mode(com_t *com) {
    uint8_t mode;
    int8_t ret_val = com_receive_value(com, &mode, sizeof(uint8_t));
    if (ret_val == 0) {
        foc_set_motor_mode(com->pfoc, (motor_mode_t)mode);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_motor_mode(com_t *com) {
    uint8_t mode = (uint8_t)foc_get_motor_mode(com->pfoc);
    com_send_uint8(com, &mode, 1);
    return 0;
}

/****************************************************************************** */

int8_t com_set_pole_pairs(com_t *com) {
    uint8_t pole_pairs;
    int8_t ret_val = com_receive_value(com, &pole_pairs, sizeof(uint8_t));
    if (ret_val == 0) {
        foc_set_motor_pole_pairs(com->pfoc, pole_pairs);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_pole_pairs(com_t *com) {
    uint8_t pole_pairs = foc_get_motor_pole_pairs(com->pfoc);
    com_send_uint8(com, &pole_pairs, 1);
    return 0;
}

int8_t com_set_kv(com_t *com) {
    float kv;
    int8_t ret_val = com_receive_value(com, &kv, sizeof(float));
    if (ret_val == 0) {
        foc_set_motor_kv(com->pfoc, kv);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_kv(com_t *com) {
    float kv = foc_get_motor_kv(com->pfoc);
    com_send_float32(com, &kv, 1);
    return 0;
}

int8_t com_set_Rs(com_t *com) {
    float Rs;
    int8_t ret_val = com_receive_value(com, &Rs, sizeof(float));
    if (ret_val == 0) {
        foc_set_motor_Rs(com->pfoc, Rs);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_Rs(com_t *com) {
    float Rs = foc_get_motor_Rs(com->pfoc);
    com_send_float32(com, &Rs, 1);
    return 0;
}

int8_t com_set_Ld(com_t *com) {
    float Ld;
    int8_t ret_val = com_receive_value(com, &Ld, sizeof(float));
    if (ret_val == 0) {
        foc_set_motor_Ld(com->pfoc, Ld);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_Ld(com_t *com) {
    float Ld = foc_get_motor_Ld(com->pfoc);
    com_send_float32(com, &Ld, 1);
    return 0;
}

int8_t com_set_Lq(com_t *com) {
    float Lq;
    int8_t ret_val = com_receive_value(com, &Lq, sizeof(float));
    if (ret_val == 0) {
        foc_set_motor_Lq(com->pfoc, Lq);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_Lq(com_t *com) {
    float Lq = foc_get_motor_Lq(com->pfoc);
    com_send_float32(com, &Lq, 1);
    return 0;
}

int8_t com_set_flux_linkage(com_t *com) {
    float flux_linkage;
    int8_t ret_val = com_receive_value(com, &flux_linkage, sizeof(float));
    if (ret_val == 0) {
        foc_set_motor_flux_linkage(com->pfoc, flux_linkage);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_flux_linkage(com_t *com) {
    float flux_linkage = foc_get_motor_flux_linkage(com->pfoc);
    com_send_float32(com, &flux_linkage, 1);
    return 0;
}

/****************************************************************************** */

int8_t com_set_foc_pid_id(com_t *com) {
    float values[3];
    int8_t ret_val = com_receive_value(com, values, sizeof(values));
    if (ret_val == 0) {
        pid_set_kp(&com->pfoc->id_ctrl, values[0]);
        pid_set_ki(&com->pfoc->id_ctrl, values[1]);
        pid_set_deadband(&com->pfoc->id_ctrl, values[2]);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_pid_id(com_t *com) {
    float values[3];
    values[0] = pid_get_kp(&com->pfoc->id_ctrl);
    values[1] = pid_get_ki(&com->pfoc->id_ctrl);
    values[2] = pid_get_deadband(&com->pfoc->id_ctrl);
    com_send_float32(com, values, 3);
    return 0;
}

int8_t com_set_foc_pid_iq(com_t *com) {
    float values[3];
    int8_t ret_val = com_receive_value(com, values, sizeof(values));
    if (ret_val == 0) {
        pid_set_kp(&com->pfoc->iq_ctrl, values[0]);
        pid_set_ki(&com->pfoc->iq_ctrl, values[1]);
        pid_set_deadband(&com->pfoc->iq_ctrl, values[2]);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_pid_iq(com_t *com) {
    float values[3];
    values[0] = pid_get_kp(&com->pfoc->iq_ctrl);
    values[1] = pid_get_ki(&com->pfoc->iq_ctrl);
    values[2] = pid_get_deadband(&com->pfoc->iq_ctrl);
    com_send_float32(com, values, 3);
    return 0;
}

/****************************************************************************** */

int8_t com_set_foc_pid_speed(com_t *com) {
    float values[4];
    int8_t ret_val = com_receive_value(com, values, sizeof(values));
    if (ret_val == 0) {
        pid_set_kp(&com->pfoc->speed_ctrl, values[0]);
        pid_set_ki(&com->pfoc->speed_ctrl, values[1]);
        pid_set_out_constraint(&com->pfoc->speed_ctrl, values[2], -values[2]);
        pid_set_deadband(&com->pfoc->speed_ctrl, values[3]);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_pid_speed(com_t *com) {
    float values[4];
    values[0] = pid_get_kp(&com->pfoc->speed_ctrl);
    values[1] = pid_get_ki(&com->pfoc->speed_ctrl);
    values[2] = pid_get_out_max(&com->pfoc->speed_ctrl);
    values[3] = pid_get_deadband(&com->pfoc->speed_ctrl);
    com_send_float32(com, values, 4);
    return 0;
}

int8_t com_set_foc_pid_position(com_t *com) {
    float values[6];
    int8_t ret_val = com_receive_value(com, values, sizeof(values));
    if (ret_val == 0) {
        pid_set_kp(&com->pfoc->pos_ctrl, values[0]);
        pid_set_ki(&com->pfoc->pos_ctrl, values[1]);
        pid_set_kd(&com->pfoc->pos_ctrl, values[2]);
        pid_set_out_constraint(&com->pfoc->pos_ctrl, values[3], -values[3]);
        pid_set_deadband(&com->pfoc->pos_ctrl, values[4]);
        pid_set_d_filter_fc(&com->pfoc->pos_ctrl, values[5]);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_foc_pid_position(com_t *com) {
    float values[6];
    values[0] = pid_get_kp(&com->pfoc->pos_ctrl);
    values[1] = pid_get_ki(&com->pfoc->pos_ctrl);
    values[2] = pid_get_kd(&com->pfoc->pos_ctrl);
    values[3] = pid_get_out_max(&com->pfoc->pos_ctrl);
    values[4] = pid_get_deadband(&com->pfoc->pos_ctrl);
    values[5] = pid_get_d_filter_fc(&com->pfoc->pos_ctrl);
    com_send_float32(com, values, 6);
    return 0;
}

/****************************************************************************** */

int8_t com_set_field_weakening_config(com_t *com) {
    float values[3];
    int8_t ret_val = com_receive_value(com, values, sizeof(values));
    if (ret_val == 0) {
        pid_set_kp(&com->pfoc->fw_ctrl, values[0]);
        pid_set_ki(&com->pfoc->fw_ctrl, values[1]);
        pid_set_out_constraint(&com->pfoc->fw_ctrl, 0, values[2]);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_field_weakening_config(com_t *com) {
    float values[3];
    values[0] = pid_get_kp(&com->pfoc->fw_ctrl);
    values[1] = pid_get_ki(&com->pfoc->fw_ctrl);
    values[2] = pid_get_out_min(&com->pfoc->fw_ctrl);
    com_send_float32(com, values, 3);
    return 0;
}

int8_t com_set_field_weakening_enable(com_t *com) {
    uint8_t enable;
    int8_t ret_val = com_receive_value(com, &enable, sizeof(enable));
    if (ret_val == 0) {
        foc_set_fw_enable(com->pfoc, (_Bool)enable);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_field_weakening_enable(com_t *com) {
    _Bool enable = foc_get_fw_enable(com->pfoc);
    com_send_uint8(com, (uint8_t*)&enable, 1);
    return 0;
}

/****************************************************************************** */

int8_t com_set_mtpa_enable(com_t *com) {
    uint8_t enable;
    int8_t ret_val = com_receive_value(com, &enable, sizeof(enable));
    if (ret_val == 0) {
        foc_set_mtpa_enable(com->pfoc, (_Bool)enable);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_get_mtpa_enable(com_t *com) {
    _Bool enable = foc_get_mtpa_enable(com->pfoc);
    com_send_uint8(com, (uint8_t*)&enable, 1);
    return 0;
}

/****************************************************************************** */

int8_t com_foc_set_current_set_point(com_t *com) {
    float Is;
    int8_t ret_val = com_receive_value(com, &Is, sizeof(Is));
    if (ret_val == 0) {
        if (Is > 1.0f) Is = 1.0f;
        else if (Is < -1.0f) Is = -1.0f;
        foc_set_current_set_point(com->pfoc, Is);
    }
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_foc_get_current_set_point(com_t *com) {
    float set_point = com->pfoc->Is_ref;
    com_send_float32(com, &set_point, 1);
    return 0;
}

int8_t com_foc_set_speed_set_point(com_t *com) {
    float rpm;
    int8_t ret_val = com_receive_value(com, &rpm, sizeof(rpm));
    if (ret_val == 0) {
        foc_set_speed_set_point(com->pfoc, rpm);
    }
    com_send_int8(com, &ret_val, 1);
    return 0;
}

int8_t com_foc_get_speed_set_point(com_t *com) {
    float set_point = com->pfoc->rpm_ref;
    com_send_float32(com, &set_point, 1);
    return 0;
}

int8_t com_foc_set_position_set_point(com_t *com) {
    float deg;
    int8_t ret_val = com_receive_value(com, &deg, sizeof(deg));
    if (ret_val == 0) {
        foc_set_position_set_point(com->pfoc, deg);
    }
    com_send_int8(com, &ret_val, 1);
    return 0;
}

int8_t com_foc_get_position_set_point(com_t *com) {
    float set_point = com->pfoc->pos_ref;
    com_send_float32(com, &set_point, 1);
    return 0;
}

/****************************************************************************** */

int8_t com_start_measure_resistance(com_t *com) {
    int8_t ret_val = 0;
    if (sc_start_measure_motor_resistance(com->psc) != 0) ret_val = -1;
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_start_measure_ld(com_t *com) {
    int8_t ret_val = 0;
    if (sc_start_measure_motor_Ld(com->psc) != 0) ret_val = -1;
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

int8_t com_start_measure_lq(com_t *com) {
    int8_t ret_val = 0;
    if (sc_start_measure_motor_Lq(com->psc) != 0) ret_val = -1;
    com_send_int8(com, &ret_val, 1);
    return ret_val;
}

/****************************************************************************** */

void com_init(com_t *com, int (*recv_data)(uint8_t*, uint16_t), int (*send_data)(uint8_t*, uint16_t),
              foc_t *pfoc, storage_t *pstorage, self_commissioning_t *psc) {
    com->recv_data = recv_data;
    com->send_data = send_data;
    com->pfoc = pfoc;
    com->pstorage = pstorage;
    com->psc = psc;
}

void com_update(com_t *com) {
    if (!com->incomming_data_flag) return;
    com->incomming_data_flag = 0;
    switch(com->data_rx[0]) {
        case 9: com_set_default_config(com); break;
        case 10: com_save_config(com); break;
        case 11: com_set_foc_mode(com); break;
        case 12: com_get_foc_mode(com); break;
        case 13: com_set_foc_motor_mode(com); break;
        case 14: com_get_foc_motor_mode(com); break;
        case 15: com_set_pole_pairs(com); break;
        case 16: com_get_pole_pairs(com); break;
        case 17: com_set_kv(com); break;
        case 18: com_get_kv(com); break;
        case 19: com_set_Rs(com); break;
        case 20: com_get_Rs(com); break;
        case 21: com_set_Ld(com); break;
        case 22: com_get_Ld(com); break;
        case 23: com_set_Lq(com); break;
        case 24: com_get_Lq(com); break;
        case 25: com_set_flux_linkage(com); break;
        case 26: com_get_flux_linkage(com); break;

        case 27: com_set_foc_pid_id(com); break;
        case 28: com_get_foc_pid_id(com); break;
        case 29: com_set_foc_pid_iq(com); break;
        case 30: com_get_foc_pid_iq(com); break;
        case 31: com_set_foc_pid_speed(com); break;
        case 32: com_get_foc_pid_speed(com); break;
        case 33: com_set_foc_pid_position(com); break;
        case 34: com_get_foc_pid_position(com); break;
        case 35: com_set_field_weakening_config(com); break;
        case 36: com_get_field_weakening_config(com); break;
        case 37: com_set_field_weakening_enable(com); break;
        case 38: com_get_field_weakening_enable(com); break;
        case 39: com_set_mtpa_enable(com); break;
        case 40: com_get_mtpa_enable(com); break;

        case 41: com_foc_set_current_set_point(com); break;
        case 42: com_foc_get_current_set_point(com); break;
        case 43: com_foc_set_speed_set_point(com); break;
        case 44: com_foc_get_speed_set_point(com); break;
        case 45: com_foc_set_position_set_point(com); break;
        case 46: com_foc_get_position_set_point(com); break;

        case 47: com_start_measure_resistance(com); break;
        case 48: com_start_measure_ld(com); break;
        case 49: com_start_measure_lq(com); break;
    }
}


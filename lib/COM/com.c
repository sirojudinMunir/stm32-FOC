#include "com.h"

/****************************************************************************** */

int8_t com_send_value(com_t *com, void *value, uint16_t size) {
  const uint16_t header = 0xA55A;
  const uint16_t total_size = size + 2; 
  uint8_t tx_buff[total_size];
  memcpy(tx_buff, &header, 2);
  memcpy(tx_buff + 2, value, size);
  while (com->send_data(tx_buff, total_size) != 0) ;
  return 0;
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
  return ret_val;
}

/****************************************************************************** */

float plotter_get_value_from_dictionary(com_t *com, uint16_t dictionary_addr) {
  float value = 0;
  switch(dictionary_addr) {
    case 0xA000: value = com->pfoc->ia; break;
    case 0xA001: value = com->pfoc->ib; break;
    case 0xA002: value = com->pfoc->ic; break;
    case 0xA003: value = com->pfoc->i_alpha; break;
    case 0xA004: value = com->pfoc->i_beta; break;
    case 0xA005: value = com->pfoc->id; break;
    case 0xA006: value = com->pfoc->iq; break;

    case 0xA007: value = com->pfoc->va; break;
    case 0xA008: value = com->pfoc->vb; break;
    case 0xA009: value = com->pfoc->vc; break;
    case 0xA00A: value = com->pfoc->v_alpha; break;
    case 0xA00B: value = com->pfoc->v_beta; break;
    case 0xA00C: value = com->pfoc->vd; break;
    case 0xA00D: value = com->pfoc->vq; break;

    case 0xA00E: value = com->pfoc->e_rad; break;
    case 0xA00F: value = com->pfoc->actual_rpm; break;
    case 0xA010: value = com->pfoc->actual_angle; break;

    case 0xA011: value = com->pfoc->Is_ref; break;
    case 0xA012: value = com->pfoc->rpm_ref; break;
    case 0xA013: value = com->pfoc->pos_ref; break;
  }
  return value;
}

void plotter_add_line_addr(com_t *com, uint16_t dictionary_addr) {
  if (com->plotter_line_count == MAX_PLOTTER_LINE || dictionary_addr == 0) return;
  for (uint8_t i = 0; i < com->plotter_line_count; i++) {
    if (com->plotter_line_addr[i] == dictionary_addr) {
      return;
    }
  }
  com->plotter_line_addr[com->plotter_line_count] = dictionary_addr;
  com->plotter_line_count++;
}

void plotter_remove_line_by_addr(com_t *com, uint16_t dictionary_addr) {
  if (com->plotter_line_count == 0) return;
  uint8_t start_idx = MAX_PLOTTER_LINE;
  for (uint8_t i = 0; i < com->plotter_line_count; i++) {
    if (com->plotter_line_addr[i] == dictionary_addr) {
      start_idx = i;
      break;
    }
  }
  for (uint8_t i = start_idx; i < (com->plotter_line_count - 1); i++) {
    com->plotter_line_addr[i] = com->plotter_line_addr[i+1];
  }
  if (start_idx < MAX_PLOTTER_LINE) {
    com->plotter_line_count--;
  }
}

int8_t com_send_plotter_update(com_t *com) {
  if (com->plotter_line_count == 0 && com->plotter_last_line_count == 0) {
    return -1;
  }
  // memset(com->plotter_data, 0, sizeof(com->plotter_data));
  for (uint8_t i = 0; i < com->plotter_line_count; i++) {
    if (com->plotter_line_addr[i] != 0) {
      com->plotter_data[i] = plotter_get_value_from_dictionary(com, com->plotter_line_addr[i]);
    }
  }

  const uint16_t header = 0xABCD;
  const uint16_t data_size = com->plotter_line_count * sizeof(float);
  const uint16_t total_size = sizeof(header) + sizeof(uint8_t) + data_size;
  uint8_t tx_buff[total_size];
  uint16_t offset = 0;
  memcpy(tx_buff + offset, &header, sizeof(header));
  offset += sizeof(header);
  tx_buff[offset++] = com->plotter_line_count;
  memcpy(tx_buff + offset, com->plotter_data, data_size);
  com->plotter_last_line_count = com->plotter_line_count;

  return (com->send_data(tx_buff, total_size) == 0) ? 0 : -1;
}

/****************************************************************************** */

int8_t com_plotter_add_line(com_t *com) {
  uint16_t addr;
  int8_t ret_val = com_receive_value(com, &addr, sizeof(uint16_t));
  if (ret_val == 0) {
    plotter_add_line_addr(com, addr);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_plotter_remove_line(com_t *com) {
  uint16_t addr;
  int8_t ret_val = com_receive_value(com, &addr, sizeof(uint16_t));
  if (ret_val == 0) {
    plotter_remove_line_by_addr(com, addr);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

/****************************************************************************** */

int8_t com_set_default_config(com_t *com) {
  int8_t ret_val = 0;
  storage_default_config(com->pstorage);
  storage_copy_to_local(com->pstorage, com->pfoc);
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_save_config(com_t *com) {
  int8_t ret_val = 0;
  storage_copy_from_local(com->pstorage, com->pfoc);
  if (storage_save_config(com->pstorage) != 0) {
    ret_val = -1;
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

/****************************************************************************** */

int8_t com_set_foc_mode(com_t *com) {
  uint8_t mode;
  int8_t ret_val = com_receive_value(com, &mode, sizeof(uint8_t));
  if (ret_val == 0) {
    foc_set_mode(com->pfoc, (foc_mode_t)mode);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_foc_mode(com_t *com) {
  uint8_t mode = (uint8_t)foc_get_mode(com->pfoc);
  com_send_value(com, &mode, sizeof(mode));
  return 0;
}

int8_t com_set_foc_motor_mode(com_t *com) {
  uint8_t mode;
  int8_t ret_val = com_receive_value(com, &mode, sizeof(uint8_t));
  if (ret_val == 0) {
    foc_set_motor_mode(com->pfoc, (motor_mode_t)mode);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_foc_motor_mode(com_t *com) {
  uint8_t mode = (uint8_t)foc_get_motor_mode(com->pfoc);
  com_send_value(com, &mode, sizeof(mode));
  return 0;
}

/****************************************************************************** */

int8_t com_set_pole_pairs(com_t *com) {
  uint8_t pole_pairs;
  int8_t ret_val = com_receive_value(com, &pole_pairs, sizeof(uint8_t));
  if (ret_val == 0) {
    foc_set_motor_pole_pairs(com->pfoc, pole_pairs);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_pole_pairs(com_t *com) {
  uint8_t pole_pairs = foc_get_motor_pole_pairs(com->pfoc);
  com_send_value(com, &pole_pairs, sizeof(pole_pairs));
  return 0;
}

int8_t com_set_kv(com_t *com) {
  float kv;
  int8_t ret_val = com_receive_value(com, &kv, sizeof(float));
  if (ret_val == 0) {
    foc_set_motor_kv(com->pfoc, kv);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_kv(com_t *com) {
  float kv = foc_get_motor_kv(com->pfoc);
  com_send_value(com, &kv, sizeof(kv));
  return 0;
}

int8_t com_set_Rs(com_t *com) {
  float Rs;
  int8_t ret_val = com_receive_value(com, &Rs, sizeof(float));
  if (ret_val == 0) {
    foc_set_motor_Rs(com->pfoc, Rs);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_Rs(com_t *com) {
  float Rs = foc_get_motor_Rs(com->pfoc);
  com_send_value(com, &Rs, sizeof(Rs));
  return 0;
}

int8_t com_set_Ld(com_t *com) {
  float Ld;
  int8_t ret_val = com_receive_value(com, &Ld, sizeof(float));
  if (ret_val == 0) {
    foc_set_motor_Ld(com->pfoc, Ld);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_Ld(com_t *com) {
  float Ld = foc_get_motor_Ld(com->pfoc);
  com_send_value(com, &Ld, sizeof(Ld));
  return 0;
}

int8_t com_set_Lq(com_t *com) {
  float Lq;
  int8_t ret_val = com_receive_value(com, &Lq, sizeof(float));
  if (ret_val == 0) {
    foc_set_motor_Lq(com->pfoc, Lq);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_Lq(com_t *com) {
  float Lq = foc_get_motor_Lq(com->pfoc);
  com_send_value(com, &Lq, sizeof(Lq));
  return 0;
}

int8_t com_set_flux_linkage(com_t *com) {
  float flux_linkage;
  int8_t ret_val = com_receive_value(com, &flux_linkage, sizeof(float));
  if (ret_val == 0) {
    foc_set_motor_flux_linkage(com->pfoc, flux_linkage);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_flux_linkage(com_t *com) {
  float flux_linkage = foc_get_motor_flux_linkage(com->pfoc);
  com_send_value(com, &flux_linkage, sizeof(flux_linkage));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_foc_pid_id(com_t *com) {
  float values[3];
  values[0] = pid_get_kp(&com->pfoc->id_ctrl);
  values[1] = pid_get_ki(&com->pfoc->id_ctrl);
  values[2] = pid_get_deadband(&com->pfoc->id_ctrl);
  com_send_value(com, values, sizeof(values));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_foc_pid_iq(com_t *com) {
  float values[3];
  values[0] = pid_get_kp(&com->pfoc->iq_ctrl);
  values[1] = pid_get_ki(&com->pfoc->iq_ctrl);
  values[2] = pid_get_deadband(&com->pfoc->iq_ctrl);
  com_send_value(com, values, sizeof(values));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_foc_pid_speed(com_t *com) {
  float values[4];
  values[0] = pid_get_kp(&com->pfoc->speed_ctrl);
  values[1] = pid_get_ki(&com->pfoc->speed_ctrl);
  values[2] = pid_get_out_max(&com->pfoc->speed_ctrl);
  values[3] = pid_get_deadband(&com->pfoc->speed_ctrl);
  com_send_value(com, values, sizeof(values));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
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
  com_send_value(com, values, sizeof(values));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_field_weakening_config(com_t *com) {
  float values[3];
  values[0] = pid_get_kp(&com->pfoc->fw_ctrl);
  values[1] = pid_get_ki(&com->pfoc->fw_ctrl);
  values[2] = pid_get_out_min(&com->pfoc->fw_ctrl);
  com_send_value(com, values, sizeof(values));
  return 0;
}

int8_t com_set_field_weakening_enable(com_t *com) {
  uint8_t enable;
  int8_t ret_val = com_receive_value(com, &enable, sizeof(enable));
  if (ret_val == 0) {
    foc_set_fw_enable(com->pfoc, (_Bool)enable);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_field_weakening_enable(com_t *com) {
  _Bool enable = foc_get_fw_enable(com->pfoc);
  com_send_value(com, (uint8_t*)&enable, sizeof(uint8_t));
  return 0;
}

/****************************************************************************** */

int8_t com_set_mtpa_enable(com_t *com) {
  uint8_t enable;
  int8_t ret_val = com_receive_value(com, &enable, sizeof(enable));
  if (ret_val == 0) {
    foc_set_mtpa_enable(com->pfoc, (_Bool)enable);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_get_mtpa_enable(com_t *com) {
  _Bool enable = foc_get_mtpa_enable(com->pfoc);
  com_send_value(com, (uint8_t*)&enable, sizeof(uint8_t));
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
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_foc_get_current_set_point(com_t *com) {
  float set_point = com->pfoc->Is_ref;
  com_send_value(com, &set_point, sizeof(set_point));
  return 0;
}

int8_t com_foc_set_speed_set_point(com_t *com) {
  float rpm;
  int8_t ret_val = com_receive_value(com, &rpm, sizeof(rpm));
  if (ret_val == 0) {
    foc_set_speed_set_point(com->pfoc, rpm);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return 0;
}

int8_t com_foc_get_speed_set_point(com_t *com) {
  float set_point = com->pfoc->rpm_ref;
  com_send_value(com, &set_point, sizeof(set_point));
  return 0;
}

int8_t com_foc_set_position_set_point(com_t *com) {
  float deg;
  int8_t ret_val = com_receive_value(com, &deg, sizeof(deg));
  if (ret_val == 0) {
    foc_set_position_set_point(com->pfoc, deg);
  }
  com_send_value(com, &ret_val, sizeof(ret_val));
  return 0;
}

int8_t com_foc_get_position_set_point(com_t *com) {
  float set_point = com->pfoc->pos_ref;
  com_send_value(com, &set_point, sizeof(set_point));
  return 0;
}

/****************************************************************************** */

int8_t com_start_measure_resistance(com_t *com) {
  int8_t ret_val = 0;
  if (sc_start_measure_motor_resistance(com->psc) != 0) ret_val = -1;
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_start_measure_ld(com_t *com) {
  int8_t ret_val = 0;
  if (sc_start_measure_motor_Ld(com->psc) != 0) ret_val = -1;
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

int8_t com_start_measure_lq(com_t *com) {
  int8_t ret_val = 0;
  if (sc_start_measure_motor_Lq(com->psc) != 0) ret_val = -1;
  com_send_value(com, &ret_val, sizeof(ret_val));
  return ret_val;
}

/****************************************************************************** */

void com_init(com_t *com, int (*recv_data)(uint8_t*, uint16_t), int (*send_data)(uint8_t*, uint16_t), uint32_t (*get_tick_ms)(void),
              foc_t *pfoc, storage_t *pstorage, self_commissioning_t *psc) {
  com->recv_data = recv_data;
  com->send_data = send_data;
  com->get_tick_ms = get_tick_ms;
  com->pfoc = pfoc;
  com->pstorage = pstorage;
  com->psc = psc;
}

void com_update(com_t *com) {
  if (com->incomming_data_flag) {
    com->incomming_data_flag = 0;
    switch(com->data_rx[0]) {
      case 7: com_plotter_add_line(com); break;
      case 8: com_plotter_remove_line(com); break;
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
  else {
    if (com->get_tick_ms() - com->plotter_tick >= 5) {
      com->plotter_tick = com->get_tick_ms();
      com_send_plotter_update(com);
    }
  }
}


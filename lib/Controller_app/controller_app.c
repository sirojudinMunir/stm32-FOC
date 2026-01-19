#include "controller_app.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include "FOC_utils.h"
#include "flash.h"

extern float sp_input;
extern motor_config_t m_config;
extern foc_t hfoc;

extern int note_piano[];

extern int start_cal;
extern _Bool calibration_flag;
extern _Bool test_bw_flag;
extern _Bool pos_demo_flag;


char usb_send_buff[128];

void send_data_float(const float* values, uint8_t count) {
  if (count == 0 || values == NULL) return;

  // Total frame: 2 byte header + 4*count float + 2 byte footer
  uint16_t total_size = 2 + count * 4 + 2;
  uint8_t frame[256];

  if (total_size > sizeof(frame)) return;

  // Header: 0x55, 0xAA
  frame[0] = 0x55;
  frame[1] = 0xAA;

  // Copy floats
  for (uint8_t i = 0; i < count; i++) {
    union { float f; uint8_t b[4]; } u;
    u.f = values[i];
    frame[2 + i * 4 + 0] = u.b[0];
    frame[2 + i * 4 + 1] = u.b[1];
    frame[2 + i * 4 + 2] = u.b[2];
    frame[2 + i * 4 + 3] = u.b[3];
  }

  // Footer: 0xAA, 0x55
  frame[2 + count * 4 + 0] = 0xAA;
  frame[2 + count * 4 + 1] = 0x55;

  // Transmit
  CDC_Transmit_FS(frame, total_size);
}

void change_legend(uint8_t index, const char *str) {
  uint8_t frame[64];
  uint16_t total_size = 0;

  frame[0] = 0x03;
  frame[1] = index;  

  size_t len = strlen(str);
  if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
  memcpy(&frame[2], str, len);

  total_size = 2 + len;

  CDC_Transmit_FS(frame, total_size);
}

void change_title(const char *str) {
  uint8_t frame[64];
  uint16_t total_size = 0;

  frame[0] = 0x04;
  frame[1] = 0x00;  

  size_t len = strlen(str);
  if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
  memcpy(&frame[2], str, len);

  total_size = 2 + len;

  CDC_Transmit_FS(frame, total_size);
}

void erase_graph(void) {
  uint8_t frame[2];
  frame[0] = 0x02;
  frame[1] = 0x00;  
  CDC_Transmit_FS(frame, 2);
}

/******************************************************************************************* */


int parse_float_value(const char *input, char separator, float *out_value) {
    if (!input || !out_value) return 0;

    // Cari posisi separator
    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    // Lompat ke karakter setelah separator
    const char *val_str = sep_pos + 1;

    // Lewati whitespace
    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    // Cek apakah setelah spasi masih ada karakter valid
    if (*val_str == '\0') return 0;

    // Parsing float
    char *endptr;
    float value = strtof(val_str, &endptr);

    if (val_str == endptr) {
        // Tidak ada konversi yang terjadi
        return 0;
    }

    *out_value = value;
    return 1;
}

int parse_int_value(const char *input, char separator, int *out_value) {
    if (!input || !out_value) return 0;

    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    const char *val_str = sep_pos + 1;

    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    if (*val_str == '\0') return 0;

    char *endptr;
    long value = strtol(val_str, &endptr, 10);

    // Validasi: endptr harus berada di akhir angka atau hanya whitespace
    while (isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') return 0; // Ada karakter tidak valid setelah angka

    *out_value = (int)value;
    return 1;
}

void parse_piano(uint8_t *buf) {
	int tone = buf[1] - 1;
	if (tone < 0) tone = 0;

	if (buf[0] == 0xD3) note_piano[tone] = 1;
	else if (buf[0] == 0xD2) note_piano[tone] = 0;
}

void get_pid_param(void) {
  char write_buffer[128];
  uint16_t len = 0;

  switch (hfoc.control_mode) {
  case TORQUE_CONTROL_MODE:
    len = sprintf(write_buffer, "Current Control param:\n"
                                "Kp(Id):%f\n"
                                "Ki(Id):%f\n"
                                "Kp(Iq):%f\n"
                                "Ki(Iq):%f\n"
                                "Deadband:%f\n"
                                "Max output:%f\n",  
                                hfoc.id_ctrl.kp, hfoc.id_ctrl.ki, 
                                hfoc.iq_ctrl.kp, hfoc.iq_ctrl.ki, 
                                hfoc.id_ctrl.e_deadband, hfoc.id_ctrl.out_max_dynamic);
    break;
  case SPEED_CONTROL_MODE:
    len = sprintf(write_buffer, "Speed Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n"
                                "Deadband:%f\n"
                                "Max output:%f\n",
                                hfoc.speed_ctrl.kp, hfoc.speed_ctrl.ki, hfoc.speed_ctrl.e_deadband, hfoc.speed_ctrl.out_max);
    break;
  case POSITION_CONTROL_MODE:
    len = sprintf(write_buffer, "Position Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n"
                                "Kd:%f\n"
                                "Deadband:%f\n"
                                "Max output:%f\n", 
                                hfoc.pos_ctrl.kp, hfoc.pos_ctrl.ki, hfoc.pos_ctrl.kd, hfoc.pos_ctrl.e_deadband, hfoc.pos_ctrl.out_max);
    break;
  default:
    len = sprintf(write_buffer, "Wrong Mode\n");
    break;
  }
  CDC_Transmit_FS((uint8_t*)write_buffer, len);
}

void print_mode(motor_mode_t mode) {
  char write_buffer[64];
  uint16_t len = 0;

  switch (mode) {
  case TORQUE_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[0]: Current Control\n");
    break;
  case SPEED_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[1]: Speed Control\n");
    break;
  case POSITION_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[2]: Position Control\n");
    break;
  case CALIBRATION_MODE:
    len = sprintf(write_buffer, "Control Mode[3]: Calibration\n");
    break;
  case AUDIO_MODE:
    len = sprintf(write_buffer, "Control Mode[4]: Audio\n");
    break;
  default:
    len = sprintf(write_buffer, "Wrong Mode\n");
    break;
  }
  CDC_Transmit_FS((uint8_t*)write_buffer, len);
}

void parse_command(char *cmd) {
	_Bool set_pid_detected = 0;

	if (strstr(cmd, "cal")) {
    usb_print("start callibration\r\n");
    hfoc.control_mode = CALIBRATION_MODE;
    calibration_flag = 1;
  }
	else if (strstr(cmd, "run")) {
    // usb_print("start test pulse response\r\n");
    // hfoc.control_mode = TEST_MODE;
    // test_bw_flag = 1;
    usb_print("start demo\r\n");
    pos_demo_flag = 1;
  }
	else if (strstr(cmd, "stop")) {
    usb_print("stop demo\r\n");
    pos_demo_flag = 0;
  }
	else if (strstr(cmd, "get_bandwidth")) {
    usb_print("Current Bandwidth:%.2f\r\n", hfoc.I_ctrl_bandwidth);
  }
	else if (strstr(cmd, "set_bandwidth")) {
    float bandwidth;
		parse_float_value(cmd, '=', &bandwidth);
    hfoc.I_ctrl_bandwidth = bandwidth;
    m_config.I_ctrl_bandwidth = bandwidth;
    flash_auto_tuning_torque_control(&m_config);
    hfoc.id_ctrl.kp = m_config.id_kp;
    hfoc.id_ctrl.ki = m_config.id_ki;
    hfoc.iq_ctrl.kp = m_config.iq_kp;
    hfoc.iq_ctrl.ki = m_config.iq_ki;

    usb_print("New Bandwidth:%.2f\r\n", hfoc.I_ctrl_bandwidth);
	}
	else if (strstr(cmd, "sp")) {
		parse_float_value(cmd, '=', &sp_input);
	}
	else if (strstr(cmd, "kp")) {
    float kp;
		parse_float_value(cmd, '=', &kp);

    switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        hfoc.id_ctrl.kp = kp;
        m_config.id_kp = kp;
        hfoc.iq_ctrl.kp = kp;
        m_config.iq_kp = kp;
        break;
      case SPEED_CONTROL_MODE:
        hfoc.speed_ctrl.kp = kp;
        m_config.speed_kp = kp;
        break;
      case POSITION_CONTROL_MODE:
        hfoc.pos_ctrl.kp = kp;
        m_config.pos_kp = kp;
        break;
      default:
      break;
    }
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "ki")) {
    float ki;
		parse_float_value(cmd, '=', &ki);
    
    switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        hfoc.id_ctrl.ki = ki;
        m_config.id_ki = ki;
        hfoc.iq_ctrl.ki = ki;
        m_config.iq_ki = ki;
        break;
      case SPEED_CONTROL_MODE:
        hfoc.speed_ctrl.ki = ki;
        m_config.speed_ki = ki;
        break;
      case POSITION_CONTROL_MODE:
        hfoc.pos_ctrl.ki = ki;
        m_config.pos_ki = ki;
        break;
      default:
        break;
    }
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "kd")) {
    float kd;
		parse_float_value(cmd, '=', &kd);
    
    switch (hfoc.control_mode) {
      case POSITION_CONTROL_MODE:
        hfoc.pos_ctrl.kd = kd;
        m_config.pos_kd = kd;
        break;
      default:
        break;
    }
		set_pid_detected = 1;
	}
  else if (strstr(cmd, "deadband")) {
    float db;
		parse_float_value(cmd, '=', &db);
    
    switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        hfoc.id_ctrl.e_deadband = db;
        m_config.id_e_deadband = db;
        hfoc.iq_ctrl.e_deadband = db;
        m_config.iq_e_deadband = db;
        break;
      case SPEED_CONTROL_MODE:
        hfoc.speed_ctrl.e_deadband = db;
        m_config.speed_e_deadband = db;
        break;
      case POSITION_CONTROL_MODE:
        hfoc.pos_ctrl.e_deadband = db;
        m_config.pos_e_deadband = db;
        break;
      default:
      break;
    }
		set_pid_detected = 1;
  }
  else if (strstr(cmd, "max")) {
    float max;
		parse_float_value(cmd, '=', &max);
    
    switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        hfoc.id_ctrl.out_max_dynamic = max;
        m_config.id_out_max = max;
        hfoc.iq_ctrl.out_max_dynamic = max;
        m_config.iq_out_max = max;
        break;
      case SPEED_CONTROL_MODE:
        hfoc.speed_ctrl.out_max = max;
        m_config.speed_out_max = max;
        break;
      case POSITION_CONTROL_MODE:
        hfoc.pos_ctrl.out_max = max;
        m_config.pos_out_max = max;
        break;
      default:
        break;
    }
		set_pid_detected = 1;
  }
	else if (strstr(cmd, "mode") != NULL) {
    motor_mode_t mode = TORQUE_CONTROL_MODE;

		parse_int_value(cmd, '=', (int*)&mode);
    print_mode(mode);

    if (mode <= AUDIO_MODE) {
      hfoc.control_mode = mode;
      sp_input = 0;
    }
	}
  else if (strstr(cmd, "ratio") != NULL) {
    float ratio;
		parse_float_value(cmd, '=', &ratio);
    hfoc.gear_ratio = ratio;
  }
  else if (strstr(cmd, "set_zero") != NULL) {
    usb_print("success set zero offset\r\n");
  }
  else if (strstr(cmd, "save") != NULL) {
    flash_save_config(&m_config);
    usb_print("success save configuration\r\n");
  }
  else if (strstr(cmd, "set_default") != NULL) {
    flash_default_config(&m_config);
    usb_print("success reset configuration\r\n");
  }
  else if (strstr(cmd, "get_param") != NULL) {
    get_pid_param();
  }

	if (set_pid_detected) {
    get_pid_param();
	}
}